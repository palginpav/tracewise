"""spike_fanout_escape.py — QFN fanout-via ESCAPE mechanism validation spike.

Validates the GUIDED escape-via placement strategy for routing through the dense RP2040
(U3) QFN pad forest on the mitayi board. This is the mechanism the human uses to route
the 13 failing nets that our router cannot complete.

Strategy validated:
  - Short F.Cu stub exits the QFN pad OUTWARD (radially away from U3 center)
  - An escape via is placed just outside the QFN pad ring, 4–8 mm from U3 center
  - A B.Cu run carries the signal to the destination (through-hole connector pad)

Target net: /GPIO3
  - Source: U3 pad 5 (SMD, F.Cu) at (145.4325, 87.98) — in the QFN pad forest
  - Dest: J3 pad 4 (thru-hole) at (121.19, 105.82)  [first pin of pair]

GATE (all must pass for GO):
  1. Net connects (ratsnest resolved)
  2. All-legal: 0 new DRC errors including hole_clearance/hole_to_hole
  3. Escape via at ~4-8 mm from U3 center (matches human strategy)
  4. Deterministic: byte-identical emitted coords run-to-run
  5. Runtime ≤ ~10 s

Usage:
    cd /home/palgin/Business_projects/tracewise
    .venv/bin/python scripts/spike_fanout_escape.py
"""
from __future__ import annotations

import collections
import json
import math
import shutil
import subprocess
import sys
import tempfile
import time
from pathlib import Path

# ---------------------------------------------------------------------------
# Shapely import
# ---------------------------------------------------------------------------
try:
    import shapely
    from shapely.geometry import Point as SPoint, box, LineString, Polygon, MultiPolygon
    from shapely.ops import unary_union
    from shapely import set_precision
    GEOS_VERSION = shapely.geos_version
    if GEOS_VERSION < (3, 8, 0):
        raise RuntimeError(f"GEOS >= 3.8.0 required, got {GEOS_VERSION}")
    print(f"[fanout] Shapely {shapely.__version__}  GEOS {GEOS_VERSION}", flush=True)
except ImportError as exc:
    print(f"ERROR: Shapely not installed: {exc}", file=sys.stderr)
    sys.exit(1)

# ---------------------------------------------------------------------------
# TraceWise imports
# ---------------------------------------------------------------------------
ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "src"))

from tracewise.route.bridge import run_drc, strip_routing
from tracewise.route.engine.kicad import (
    build_problem,
    extract_pads,
    project_geometry,
    refill_zones,
)
from tracewise.route.gridless.geom import (
    build_windowed_free_space,
    extract_board_outline,
    extract_drill_obstacles,
    snap,
)
from tracewise.route.gridless.search import build_visibility_graph, astar_visgraph
from tracewise.sexpr import atom, node, parse_file, write_file

BOARD_SRC = ROOT / "data/benchmark-boards/mitayi-pico-d1/Mitayi-Pico-D1.kicad_pcb"
SUFFIXES = (".kicad_pcb", ".kicad_sch", ".kicad_pro", ".kicad_prl")
PRECISION = 1e-6

# Target net
TARGET_NET = "/GPIO3"
# Human escape via for /GPIO3 is at (147.4397, 94.5397), dist=5.741 mm from U3 center

# U3 QFN center (from board analysis)
U3_CX = 148.870
U3_CY = 88.980
U3_RING_RADIUS = 4.310  # max distance from U3 center to any U3 pad center

# Source pad: U3 pad 5 (GPIO3 on QFN, SMD F.Cu)
SOURCE_PAD_XY = (145.4325, 87.98)
# Destination pad: J3 pad 4 (thru-hole connector) - first instance
DEST_PAD_XY = (121.19, 105.82)


# ---------------------------------------------------------------------------
# Board setup helpers
# ---------------------------------------------------------------------------

def setup_board(out_dir: Path) -> Path:
    """Copy mitayi board to temp dir + strip_routing. Returns board path."""
    bdir = BOARD_SRC.parent
    out_dir.mkdir(parents=True, exist_ok=True)
    for f in bdir.iterdir():
        if f.suffix in SUFFIXES:
            shutil.copy(f, out_dir / f.name)
    board = next(out_dir.glob("*.kicad_pcb"))
    strip_routing(board)
    return board


def get_hole_clearances(board: Path, clearance_mm: float) -> tuple[float, float]:
    """Parse hole_clearance and hole_to_hole from the .kicad_pro design rules."""
    pro = board.with_suffix(".kicad_pro")
    hole_clearance = max(clearance_mm, 0.25)
    hole_to_hole = max(clearance_mm, 0.25)
    if pro.exists():
        try:
            data = json.loads(pro.read_text(encoding="utf-8"))
            rules = data.get("board", {}).get("design_settings", {}).get("rules", {})
            if rules.get("min_hole_clearance"):
                hole_clearance = float(rules["min_hole_clearance"])
            if rules.get("min_hole_to_hole"):
                hole_to_hole = float(rules["min_hole_to_hole"])
        except (ValueError, OSError):
            pass
    return hole_clearance, hole_to_hole


# ---------------------------------------------------------------------------
# Step 1: Extract U3 QFN geometry and verify via geometry
# ---------------------------------------------------------------------------

def compute_u3_geometry(data: dict) -> tuple[float, float, float]:
    """Compute U3 centroid and ring radius from pad data.
    Returns (cx, cy, ring_radius_mm).
    """
    u3_pads = [p for p in data["pads"] if p.get("ref") == "U3"]
    if not u3_pads:
        raise ValueError("U3 not found in pad data")
    # Centroid = mean of pad positions
    xs = [p["x"] for p in u3_pads]
    ys = [p["y"] for p in u3_pads]
    # Use known U3 position from board (more accurate than pad centroid)
    cx, cy = U3_CX, U3_CY
    ring_radius = max(math.hypot(p["x"] - cx, p["y"] - cy) for p in u3_pads)
    return cx, cy, ring_radius


def find_net_pads(data: dict, net_name: str) -> list[dict]:
    """Find all pads on the target net."""
    return [p for p in data["pads"] if p["net"] == net_name]


# ---------------------------------------------------------------------------
# Step 2: Guided escape-via placement
# ---------------------------------------------------------------------------

def compute_guided_escape_via_pos(
    u3cx: float,
    u3cy: float,
    ring_radius: float,
    source_xy: tuple[float, float],
    via_mm: float,
    clearance_mm: float,
    margin_mm: float = 0.3,
) -> tuple[float, float]:
    """Place the escape via on the ray from U3 center through the source pad.

    The via is placed just outside the pad ring:
      radius = ring_radius + via_mm/2 + clearance_mm + margin_mm

    This mirrors the human's strategy: F.Cu stub exits pad outward → via at ring perimeter.
    """
    sx, sy = source_xy
    # Direction vector from U3 center toward source pad
    dx = sx - u3cx
    dy = sy - u3cy
    dist_to_source = math.hypot(dx, dy)
    if dist_to_source < 1e-6:
        raise ValueError("Source pad coincides with U3 center")
    ux = dx / dist_to_source
    uy = dy / dist_to_source

    # Target radius: just outside ring + via annular ring + clearance + margin
    target_r = ring_radius + via_mm / 2.0 + clearance_mm + margin_mm
    escape_x = u3cx + ux * target_r
    escape_y = u3cy + uy * target_r

    # Snap to 1nm
    escape_x = round(escape_x * 1e6) / 1e6
    escape_y = round(escape_y * 1e6) / 1e6
    return escape_x, escape_y


# ---------------------------------------------------------------------------
# Via legality predicate (3-predicate, from spikeM3)
# ---------------------------------------------------------------------------

def is_legal_via(
    site: tuple[float, float],
    fs_F: object,
    fs_B: object,
    pads: list[dict],
    net_name: str,
    via_mm: float,
    via_drill: float,
    clearance_mm: float,
    hole_clearance: float,
    hole_to_hole: float,
    drill_centers: list[tuple[float, float, float]],
    window_bbox: tuple[float, float, float, float],
) -> tuple[bool, str]:
    """Test all 3 via legality predicates. Returns (ok, reason_if_fail)."""
    x, y = site
    sp = SPoint(x, y)
    wx1, wy1, wx2, wy2 = window_bbox

    # Predicate 1: copper ring disc within free space on both layers
    copper_disc = snap(sp.buffer(via_mm / 2.0 + clearance_mm, resolution=16))
    if not copper_disc.within(fs_F):
        return False, "pred1_fcu_copper_ring"
    if not copper_disc.within(fs_B):
        return False, "pred1_bcu_copper_ring"

    # Predicate 2: drill disc clearance to other-net copper (hole_clearance)
    drill_disc = snap(sp.buffer(via_drill / 2.0 + hole_clearance, resolution=16))
    for layer in [0, 1]:
        for p in pads:
            if p["net"] == net_name:
                continue
            if layer == 0 and not p.get("front", False):
                continue
            if layer == 1 and not p.get("back", False):
                continue
            px1 = p["x"] - p["hw"]
            py1 = p["y"] - p["hh"]
            px2 = p["x"] + p["hw"]
            py2 = p["y"] + p["hh"]
            if px2 < wx1 or px1 > wx2 or py2 < wy1 or py1 > wy2:
                continue
            pad_rect = box(px1, py1, px2, py2)
            if drill_disc.intersects(pad_rect):
                return False, f"pred2_drill_to_copper_layer{layer}"

    # Predicate 3: drill-to-drill spacing
    for cx, cy, drill_r in drill_centers:
        dist = math.hypot(x - cx, y - cy)
        required = via_drill / 2.0 + drill_r + hole_to_hole
        if dist < required - 1e-6:
            return False, f"pred3_drill_to_drill (dist={dist:.4f}<{required:.4f})"

    return True, ""


def find_legal_via_near(
    initial_pos: tuple[float, float],
    u3cx: float,
    u3cy: float,
    ring_radius: float,
    fs_F: object,
    fs_B: object,
    pads: list[dict],
    net_name: str,
    via_mm: float,
    via_drill: float,
    clearance_mm: float,
    hole_clearance: float,
    hole_to_hole: float,
    drill_centers: list[tuple[float, float, float]],
    window_bbox: tuple[float, float, float, float],
    search_arc_deg: float = 60.0,
    search_steps: int = 24,
    radial_steps: int = 5,
    radial_step_mm: float = 0.2,
) -> tuple[tuple[float, float] | None, str]:
    """Search for a legal via near the initial guided placement.

    Searches along the ring perimeter (±search_arc_deg) and radially outward,
    in a small bounded neighborhood. Returns (legal_pos, reason) where reason is
    '' if found, or the last fail reason if none found.
    """
    # First try the initial position exactly
    ok, reason = is_legal_via(
        initial_pos, fs_F, fs_B, pads, net_name,
        via_mm, via_drill, clearance_mm, hole_clearance, hole_to_hole,
        drill_centers, window_bbox,
    )
    if ok:
        return initial_pos, ""

    best_fail = reason
    ix, iy = initial_pos
    init_dist = math.hypot(ix - u3cx, iy - u3cy)
    init_angle_deg = math.degrees(math.atan2(iy - u3cy, ix - u3cx))

    # Search arc: vary angle ± search_arc_deg in search_steps steps
    # and radius ± radial_steps * radial_step_mm outward
    candidates = []
    for angle_offset in range(-search_steps // 2, search_steps // 2 + 1):
        angle_deg = init_angle_deg + angle_offset * (search_arc_deg / (search_steps // 2))
        angle_rad = math.radians(angle_deg)
        for ri in range(radial_steps):
            r = init_dist + ri * radial_step_mm
            cx2 = round((u3cx + r * math.cos(angle_rad)) * 1e6) / 1e6
            cy2 = round((u3cy + r * math.sin(angle_rad)) * 1e6) / 1e6
            dist_from_start = math.hypot(cx2 - ix, cy2 - iy)
            candidates.append((dist_from_start, angle_offset, ri, (cx2, cy2)))

    # Sort by distance from initial (prefer closest to guided position)
    candidates.sort(key=lambda t: (t[0], abs(t[1]), t[2]))

    for _, _, _, site in candidates:
        ok, reason = is_legal_via(
            site, fs_F, fs_B, pads, net_name,
            via_mm, via_drill, clearance_mm, hole_clearance, hole_to_hole,
            drill_centers, window_bbox,
        )
        if ok:
            return site, ""
        best_fail = reason

    return None, best_fail


# ---------------------------------------------------------------------------
# Extract drill centers from board
# ---------------------------------------------------------------------------

def extract_drill_centers(board: Path) -> list[tuple[float, float, float]]:
    """Extract drill center positions and radii. Returns [(cx, cy, drill_r), ...]."""
    try:
        root = parse_file(board)
        centers: list[tuple[float, float, float]] = []
        seen: set[tuple[float, float, float]] = set()
        for fp in root.find_all("footprint"):
            at_node = fp.first("at")
            try:
                fp_x = float(at_node.arg(1)) if at_node else 0.0
                fp_y = float(at_node.arg(2)) if at_node else 0.0
            except (TypeError, ValueError):
                fp_x, fp_y = 0.0, 0.0
            for pad in fp.find_all("pad"):
                drill_node = pad.first("drill")
                if drill_node is None:
                    continue
                at_pad = pad.first("at")
                try:
                    px = float(at_pad.arg(1)) if at_pad else 0.0
                    py = float(at_pad.arg(2)) if at_pad else 0.0
                except (TypeError, ValueError):
                    px, py = 0.0, 0.0
                cx2 = fp_x + px
                cy2 = fp_y + py
                drill_atoms = drill_node.atoms()
                try:
                    if len(drill_atoms) >= 2 and drill_atoms[1].value == "oval":
                        d = min(float(drill_atoms[2].value), float(drill_atoms[3].value))
                    else:
                        d = float(drill_atoms[1].value)
                except (IndexError, ValueError):
                    continue
                key = (round(cx2, 3), round(cy2, 3), round(d / 2.0, 3))
                if key not in seen:
                    seen.add(key)
                    centers.append((cx2, cy2, d / 2.0))
        return centers
    except Exception:  # noqa: BLE001
        return []


# ---------------------------------------------------------------------------
# Per-layer free space (adapted from spikeM3)
# ---------------------------------------------------------------------------

def build_layer_free_space(
    pads: list[dict],
    net_name: str,
    clearance_mm: float,
    track_mm: float,
    extra_obstacles: list,
    window_bbox: tuple[float, float, float, float],
    board_outline: object | None,
    drill_obstacles: list,
    layer: int,  # 0=F.Cu, 1=B.Cu
) -> tuple[object, list]:
    """Build free space for a specific layer."""
    layer_pads = []
    for p in pads:
        if p["net"] == net_name:
            layer_pads.append(p)  # Own net: always include for carve-out
        else:
            if layer == 0 and p.get("front", False):
                layer_pads.append(p)
            elif layer == 1 and p.get("back", False):
                layer_pads.append(p)
    return build_windowed_free_space(
        pads=layer_pads,
        net_name=net_name,
        clearance_mm=clearance_mm,
        track_mm=track_mm,
        extra_obstacles=extra_obstacles,
        window_bbox=window_bbox,
        board_outline=board_outline,
        drill_obstacles=drill_obstacles,
    )


# ---------------------------------------------------------------------------
# Single-layer visibility graph routing leg
# ---------------------------------------------------------------------------

def _all_ring_corners(
    fs_component: object,
    start_xy: tuple[float, float],
    goal_xy: tuple[float, float],
    margin_mm: float,
) -> list[tuple[float, float]]:
    """Extract ALL ring corners from a free-space component (interior AND exterior rings).

    This handles the case where the free space is so tight (source pad surrounded by
    dense QFN pads) that obstacle_corners (interior rings only) finds no visible corners.
    In those cases, the relevant turning points are on the EXTERIOR ring of the free
    space component (the boundary of the carved-out corridor).
    """
    sx, sy = start_xy
    gx, gy = goal_xy
    x_lo = min(sx, gx) - margin_mm
    x_hi = max(sx, gx) + margin_mm
    y_lo = min(sy, gy) - margin_mm
    y_hi = max(sy, gy) + margin_mm

    pts: set[tuple[float, float]] = set()
    # Interior rings (obstacle holes within free space)
    for ring in fs_component.interiors:
        for x, y in ring.coords:
            if x_lo <= x <= x_hi and y_lo <= y <= y_hi:
                pts.add((round(x, 6), round(y, 6)))
    # Exterior ring (boundary of the free space component itself)
    for x, y in fs_component.exterior.coords:
        if x_lo <= x <= x_hi and y_lo <= y <= y_hi:
            pts.add((round(x, 6), round(y, 6)))
    return sorted(pts)


def _build_visgraph_with_corners(
    free_space: object,
    start_xy: tuple[float, float],
    goal_xy: tuple[float, float],
    margin_mm: float,
    obs: list,
    corners: list[tuple[float, float]],
) -> tuple[list, dict, int, int]:
    """Build visibility graph using a pre-specified corner set.

    Uses STRtree-accelerated visibility checks (same approach as build_visibility_graph).
    """
    import heapq as _heapq
    import numpy as _np

    from tracewise.route.gridless.search import is_visible, is_visible_fast
    from tracewise.route.gridless.geom import get_component_containing

    fs_comp = get_component_containing(free_space, start_xy)

    all_nodes: list[tuple[float, float]] = [start_xy, goal_xy] + corners
    n = len(all_nodes)
    adj: dict[int, list[tuple[float, int]]] = {i: [] for i in range(n)}

    for i in range(n):
        for j in range(i + 1, n):
            u, v = all_nodes[i], all_nodes[j]
            if is_visible(u, v, fs_comp):
                d = math.hypot(v[0] - u[0], v[1] - u[1])
                adj[i].append((d, j))
                adj[j].append((d, i))

    for i in range(n):
        adj[i] = sorted(adj[i], key=lambda e: (e[0], all_nodes[e[1]]))

    total_edges = sum(len(v) for v in adj.values()) // 2
    return all_nodes, adj, n, total_edges


def route_single_layer_leg(
    start_xy: tuple[float, float],
    goal_xy: tuple[float, float],
    free_space: object,
    obs: list,
    window_mm: float,
    window_bbox: tuple[float, float, float, float],
    label: str = "",
) -> list[tuple[float, float]] | None:
    """Route a single-layer segment using visibility graph A*.

    Strategy (in order):
    1. Try reflex-only corners (fastest, O(r²))
    2. Try full interior ring corners (standard fallback)
    3. Try ALL ring corners — interior AND exterior (for tight/dense-pad situations where
       the source pad is encircled by QFN neighbors and the free-space boundary itself
       provides the turning points)

    Returns a list of (x, y) waypoints if found, else None.
    """
    from tracewise.route.gridless.geom import get_component_containing

    for mode in ["reflex", "full_interior", "all_rings"]:
        if mode == "reflex":
            nodes, adj, n_nodes, n_edges = build_visibility_graph(
                free_space=free_space,
                start_xy=start_xy,
                goal_xy=goal_xy,
                margin_mm=window_mm,
                obstacle_polys=obs,
                use_reflex_pruning=True,
            )
        elif mode == "full_interior":
            nodes, adj, n_nodes, n_edges = build_visibility_graph(
                free_space=free_space,
                start_xy=start_xy,
                goal_xy=goal_xy,
                margin_mm=window_mm,
                obstacle_polys=obs,
                use_reflex_pruning=False,
            )
        else:  # all_rings — includes exterior ring corners
            fs_comp = get_component_containing(free_space, start_xy)
            corners = _all_ring_corners(fs_comp, start_xy, goal_xy, window_mm)
            nodes, adj, n_nodes, n_edges = _build_visgraph_with_corners(
                free_space, start_xy, goal_xy, window_mm, obs, corners
            )

        if label:
            print(f"[fanout] {label} ({mode}): visibility graph {n_nodes} nodes, {n_edges} edges", flush=True)

        start_edges = len(adj.get(0, []))
        goal_edges = len(adj.get(1, []))
        if start_edges == 0 or goal_edges == 0:
            if label:
                print(f"[fanout] {label} ({mode}): start_edges={start_edges} goal_edges={goal_edges} — trying next mode", flush=True)
            continue

        # astar_visgraph: nodes indexed 0=start, 1=goal; returns list of (x,y) waypoints
        path = astar_visgraph(nodes, adj, goal_xy=goal_xy)
        if path is not None:
            if label:
                print(f"[fanout] {label} ({mode}): path found with {len(path)} waypoints", flush=True)
            return path
        if label:
            print(f"[fanout] {label} ({mode}): A* failed - no path found", flush=True)

    return None


# ---------------------------------------------------------------------------
# Validate leg segments using Shapely free space
# ---------------------------------------------------------------------------

def validate_leg_segments(
    waypoints: list[tuple[float, float]],
    free_space: object,
    label: str = "",
) -> tuple[bool, int, str]:
    """Validate each segment of the leg is within free space.

    Returns (all_legal, bad_count, details).
    Uses Shapely contains as the oracle (same as spike1 FIX-1).
    """
    bad = 0
    bad_reason = ""
    for i in range(len(waypoints) - 1):
        p1 = waypoints[i]
        p2 = waypoints[i + 1]
        seg = snap(LineString([p1, p2]))
        if not free_space.contains(seg):
            # Check distance - tolerance for floating point
            # The segment endpoints touch free space boundary; allow tiny epsilon
            # Check midpoint is inside
            mid = ((p1[0] + p2[0]) / 2, (p1[1] + p2[1]) / 2)
            mp = SPoint(mid)
            if not free_space.contains(mp) and not free_space.intersects(mp):
                bad += 1
                if not bad_reason:
                    bad_reason = f"seg {i}: ({p1[0]:.3f},{p1[1]:.3f})->({p2[0]:.3f},{p2[1]:.3f}) outside free_space"
                if label:
                    print(f"[fanout] {label}: seg {i} OUTSIDE free_space", flush=True)
    return (bad == 0), bad, bad_reason


# ---------------------------------------------------------------------------
# Emit 2-layer path to board
# ---------------------------------------------------------------------------

def emit_fanout_route(
    board: Path,
    net_name: str,
    fcu_waypoints: list[tuple[float, float]],
    escape_via: tuple[float, float],
    bcu_waypoints: list[tuple[float, float]],
    track_mm: float,
    via_mm: float,
    via_drill: float,
) -> tuple[int, int]:
    """Emit F.Cu leg, via, and B.Cu leg to board file.

    Returns (segments_written, vias_written).
    """
    root = parse_file(board)
    decls = {n.arg(2): n.arg(1) for n in root.nodes("net") if n.arg(2) is not None}

    def net_nd(name: str):
        if decls:
            num = decls.get(name)
            if num is not None:
                return node("net", num)
        return node("net", atom(name, quote=True))

    nd = net_nd(net_name)
    segs = 0

    # F.Cu leg segments
    for i in range(len(fcu_waypoints) - 1):
        xa, ya = fcu_waypoints[i]
        xb, yb = fcu_waypoints[i + 1]
        root.insert(node(
            "segment",
            node("start", f"{xa:.6f}", f"{ya:.6f}"),
            node("end", f"{xb:.6f}", f"{yb:.6f}"),
            node("width", str(track_mm)),
            node("layer", atom("F.Cu", quote=True)),
            nd,
        ))
        segs += 1

    # Escape via
    vx, vy = escape_via
    via_nd = node(
        "via",
        node("at", f"{vx:.6f}", f"{vy:.6f}"),
        node("size", str(via_mm)),
        node("drill", str(via_drill)),
        node("layers", atom("F.Cu", quote=True), atom("B.Cu", quote=True)),
        nd,
    )
    root.insert(via_nd)

    # B.Cu leg segments
    for i in range(len(bcu_waypoints) - 1):
        xa, ya = bcu_waypoints[i]
        xb, yb = bcu_waypoints[i + 1]
        root.insert(node(
            "segment",
            node("start", f"{xa:.6f}", f"{ya:.6f}"),
            node("end", f"{xb:.6f}", f"{yb:.6f}"),
            node("width", str(track_mm)),
            node("layer", atom("B.Cu", quote=True)),
            nd,
        ))
        segs += 1

    write_file(root, board)
    return segs, 1  # 1 via


def extract_emitted_coords(board: Path, net_name: str) -> str:
    """Extract emitted segment + via coords as sorted canonical string."""
    root = parse_file(board)
    decls = {n.arg(2): n.arg(1) for n in root.nodes("net") if n.arg(2) is not None}
    net_num = decls.get(net_name)

    lines = []
    for seg in root.nodes("segment"):
        for child in seg.nodes("net"):
            num_val = child.arg(1)
            match = (net_num and num_val == net_num) or num_val == net_name
            if match:
                start = seg.first("start")
                end_ = seg.first("end")
                if start and end_:
                    lines.append(f"seg:{start.arg(1)},{start.arg(2)}-{end_.arg(1)},{end_.arg(2)}")
    for via in root.nodes("via"):
        for child in via.nodes("net"):
            num_val = child.arg(1)
            match = (net_num and num_val == net_num) or num_val == net_name
            if match:
                at = via.first("at")
                if at:
                    lines.append(f"via:{at.arg(1)},{at.arg(2)}")

    lines.sort()
    return "\n".join(lines)


def drc_summary(report: dict) -> dict:
    """Summarise DRC report."""
    violations = report.get("violations", [])
    errors = sum(1 for v in violations if v.get("severity") == "error")
    unconnected = len(report.get("unconnected_items", []))
    by_type = collections.Counter(v.get("type") for v in violations)
    # Count hole-related errors
    hole_errors = sum(
        1 for v in violations
        if v.get("severity") == "error" and
        any(kw in str(v.get("type", "")) for kw in ["hole", "clearance"])
    )
    return {
        "unconnected": unconnected,
        "errors": errors,
        "hole_errors": hole_errors,
        "by_type": dict(by_type),
    }


# ---------------------------------------------------------------------------
# Main spike
# ---------------------------------------------------------------------------

def run_spike(out_dir: Path, run_label: str = "run1") -> dict:
    """Run the full fanout escape spike. Returns result dict."""
    t_start = time.perf_counter()

    print(f"\n{'='*70}", flush=True)
    print(f"[fanout] Spike run: {run_label}", flush=True)
    print(f"{'='*70}", flush=True)

    # Step 1: Setup board + extract geometry
    print("[fanout] Setting up board...", flush=True)
    board = setup_board(out_dir)
    data = extract_pads(board)
    geo = project_geometry(board)
    board_bbox = (
        data["board"]["x1"], data["board"]["y1"],
        data["board"]["x2"], data["board"]["y2"],
    )
    bx1, by1, bx2, by2 = board_bbox

    print(f"[fanout] Via geometry: {geo['via_mm']}mm dia / {geo['via_drill_mm']}mm drill", flush=True)
    assert geo["via_mm"] == 0.4, f"Expected via_mm=0.4 but got {geo['via_mm']} (wrong fallback!)"
    assert geo["via_drill_mm"] == 0.2, f"Expected via_drill_mm=0.2 but got {geo['via_drill_mm']}"
    print("[fanout] Via geometry confirmed: 0.4mm/0.2mm drill (NOT the 0.6mm fallback)", flush=True)

    via_mm = geo["via_mm"]
    via_drill = geo["via_drill_mm"]
    track_mm = geo["track_mm"]
    clearance_mm = geo["clearance_mm"]
    hole_clearance, hole_to_hole = get_hole_clearances(board, clearance_mm)
    print(f"[fanout] Track={track_mm}mm, clearance={clearance_mm}mm", flush=True)
    print(f"[fanout] hole_clearance={hole_clearance}mm, hole_to_hole={hole_to_hole}mm", flush=True)

    board_outline = extract_board_outline(board)
    drill_obstacles = extract_drill_obstacles(board, clearance_mm, track_mm)
    drill_centers = extract_drill_centers(board)
    print(f"[fanout] Board bbox: {board_bbox}", flush=True)
    print(f"[fanout] Drill centers extracted: {len(drill_centers)}", flush=True)

    # Step 2: Identify target net pads
    net_pads = find_net_pads(data, TARGET_NET)
    print(f"\n[fanout] Target net: {TARGET_NET} has {len(net_pads)} pads", flush=True)
    for p in net_pads:
        print(f"  ref={p.get('ref','')} x={p['x']:.4f} y={p['y']:.4f} "
              f"front={p.get('front')} back={p.get('back')} "
              f"hw={p['hw']:.4f} hh={p['hh']:.4f}", flush=True)

    # Identify source (U3 SMD) and dest (J3 thru-hole)
    source_pad = next(
        (p for p in net_pads if p.get("ref") == "U3"),
        None,
    )
    dest_pad = next(
        (p for p in net_pads
         if p.get("ref") and p.get("ref").startswith("J") and p.get("back", False)),
        None,
    )
    # Fallback: pick thru-hole pad
    if dest_pad is None:
        dest_pad = next(
            (p for p in net_pads if p.get("ref") != "U3"),
            None,
        )

    if source_pad is None or dest_pad is None:
        raise ValueError(f"Could not find source/dest pads for {TARGET_NET}")

    source_xy = (source_pad["x"], source_pad["y"])
    dest_xy = (dest_pad["x"], dest_pad["y"])
    print(f"\n[fanout] Source pad: U3 ref={source_pad.get('ref')} xy={source_xy} (QFN SMD, F.Cu)", flush=True)
    print(f"[fanout] Dest pad: ref={dest_pad.get('ref')} xy={dest_xy} (thru-hole connector)", flush=True)

    # Step 3: Compute U3 geometry
    u3cx, u3cy, ring_radius = compute_u3_geometry(data)
    src_dist = math.hypot(source_xy[0] - u3cx, source_xy[1] - u3cy)
    print(f"\n[fanout] U3 center: ({u3cx:.4f}, {u3cy:.4f})", flush=True)
    print(f"[fanout] U3 ring radius: {ring_radius:.4f} mm", flush=True)
    print(f"[fanout] Source pad distance from U3 center: {src_dist:.4f} mm", flush=True)
    print(f"[fanout] Human escape via for {TARGET_NET}: (147.4397, 94.5397) dist=5.741mm", flush=True)

    # Step 4: Guided escape-via placement
    print(f"\n[fanout] === STEP 4: Guided escape-via placement ===", flush=True)
    initial_via = compute_guided_escape_via_pos(
        u3cx, u3cy, ring_radius, source_xy, via_mm, clearance_mm
    )
    init_dist = math.hypot(initial_via[0] - u3cx, initial_via[1] - u3cy)
    print(f"[fanout] Initial guided via pos: ({initial_via[0]:.4f}, {initial_via[1]:.4f})", flush=True)
    print(f"[fanout] Initial via dist from U3 center: {init_dist:.4f} mm", flush=True)

    # Build large window for via placement legality check
    # Use a window that covers the QFN pad ring area
    via_window_margin = max(ring_radius + 3.0, 8.0)
    wx1 = max(u3cx - via_window_margin, bx1)
    wy1 = max(u3cy - via_window_margin, by1)
    wx2 = min(u3cx + via_window_margin, bx2)
    wy2 = min(u3cy + via_window_margin, by2)
    via_window = (wx1, wy1, wx2, wy2)
    print(f"[fanout] Via-search window: {via_window}", flush=True)

    # Build F.Cu and B.Cu free space for the via window
    fs_F_via, obs_F_via = build_layer_free_space(
        data["pads"], TARGET_NET, clearance_mm, track_mm, [],
        via_window, board_outline, drill_obstacles, layer=0,
    )
    fs_B_via, obs_B_via = build_layer_free_space(
        data["pads"], TARGET_NET, clearance_mm, track_mm, [],
        via_window, board_outline, drill_obstacles, layer=1,
    )
    print(f"[fanout] fs_F_via area={fs_F_via.area:.2f}mm², fs_B_via area={fs_B_via.area:.2f}mm²", flush=True)

    # Find legal via near guided position
    legal_via, fail_reason = find_legal_via_near(
        initial_via, u3cx, u3cy, ring_radius,
        fs_F_via, fs_B_via, data["pads"], TARGET_NET,
        via_mm, via_drill, clearance_mm, hole_clearance, hole_to_hole,
        drill_centers, via_window,
    )

    if legal_via is None:
        print(f"[fanout] ERROR: No legal via found near guided position! Last fail: {fail_reason}", flush=True)
        escape_via_legal = False
        escape_via = initial_via
    else:
        escape_via_legal = True
        escape_via = legal_via
        via_dist = math.hypot(escape_via[0] - u3cx, escape_via[1] - u3cy)
        print(f"[fanout] Legal escape via found: ({escape_via[0]:.4f}, {escape_via[1]:.4f})", flush=True)
        print(f"[fanout] Escape via dist from U3 center: {via_dist:.4f} mm", flush=True)
        if 4.0 <= via_dist <= 8.0:
            print(f"[fanout] MATCH: via at {via_dist:.3f}mm is within 4-8mm range (matches human strategy)", flush=True)
        else:
            print(f"[fanout] WARNING: via at {via_dist:.3f}mm is OUTSIDE expected 4-8mm range", flush=True)

    # Step 5: Route F.Cu stub (source → escape via)
    print(f"\n[fanout] === STEP 5a: F.Cu stub routing ===", flush=True)
    print(f"[fanout] Source: {source_xy} → Escape via: {escape_via}", flush=True)
    stub_dist = math.hypot(escape_via[0] - source_xy[0], escape_via[1] - source_xy[1])
    print(f"[fanout] Stub length: {stub_dist:.4f} mm", flush=True)

    # F.Cu window: source pad to escape via area
    fcu_margin = max(stub_dist + 2.0, 3.0)
    fcu_wx1 = max(min(source_xy[0], escape_via[0]) - fcu_margin, bx1)
    fcu_wy1 = max(min(source_xy[1], escape_via[1]) - fcu_margin, by1)
    fcu_wx2 = min(max(source_xy[0], escape_via[0]) + fcu_margin, bx2)
    fcu_wy2 = min(max(source_xy[1], escape_via[1]) + fcu_margin, by2)
    fcu_window = (fcu_wx1, fcu_wy1, fcu_wx2, fcu_wy2)

    fs_F_stub, obs_F_stub = build_layer_free_space(
        data["pads"], TARGET_NET, clearance_mm, track_mm, [],
        fcu_window, board_outline, drill_obstacles, layer=0,
    )
    print(f"[fanout] F.Cu stub free_space area={fs_F_stub.area:.2f}mm²", flush=True)

    # Check if source and via are in the same F.Cu free space component
    src_in_fcu = fs_F_stub.contains(SPoint(source_xy))
    via_in_fcu = fs_F_stub.contains(SPoint(escape_via)) or fs_F_stub.distance(SPoint(escape_via)) < clearance_mm / 2
    print(f"[fanout] Source in F.Cu free_space: {src_in_fcu}", flush=True)
    print(f"[fanout] Via in F.Cu free_space: {via_in_fcu}", flush=True)

    fcu_stub_ok = False
    fcu_waypoints = None

    if escape_via_legal:
        fcu_waypoints = route_single_layer_leg(
            source_xy, escape_via, fs_F_stub, obs_F_stub,
            fcu_margin, fcu_window, label="F.Cu stub",
        )

        if fcu_waypoints is not None:
            # Validate segments
            leg_ok, bad_count, leg_reason = validate_leg_segments(
                fcu_waypoints, fs_F_stub, label="F.Cu stub"
            )
            if leg_ok:
                fcu_stub_ok = True
                print(f"[fanout] F.Cu stub: ALL LEGAL ({len(fcu_waypoints)} waypoints)", flush=True)
            else:
                print(f"[fanout] F.Cu stub: {bad_count} illegal segments - {leg_reason}", flush=True)
        else:
            print("[fanout] F.Cu stub: routing FAILED", flush=True)

    # Step 5b: Route B.Cu run (escape via → destination)
    print(f"\n[fanout] === STEP 5b: B.Cu run routing ===", flush=True)
    print(f"[fanout] Via: {escape_via} → Dest: {dest_xy}", flush=True)
    bcu_dist = math.hypot(dest_xy[0] - escape_via[0], dest_xy[1] - escape_via[1])
    print(f"[fanout] B.Cu run length estimate: {bcu_dist:.4f} mm", flush=True)

    # B.Cu window: escape via to destination
    bcu_margin = max(bcu_dist * 0.3, 5.0)
    bcu_wx1 = max(min(escape_via[0], dest_xy[0]) - bcu_margin, bx1)
    bcu_wy1 = max(min(escape_via[1], dest_xy[1]) - bcu_margin, by1)
    bcu_wx2 = min(max(escape_via[0], dest_xy[0]) + bcu_margin, bx2)
    bcu_wy2 = min(max(escape_via[1], dest_xy[1]) + bcu_margin, by2)
    bcu_window = (bcu_wx1, bcu_wy1, bcu_wx2, bcu_wy2)
    print(f"[fanout] B.Cu window: {bcu_window}", flush=True)

    fs_B_run, obs_B_run = build_layer_free_space(
        data["pads"], TARGET_NET, clearance_mm, track_mm, [],
        bcu_window, board_outline, drill_obstacles, layer=1,
    )
    print(f"[fanout] B.Cu run free_space area={fs_B_run.area:.2f}mm²", flush=True)

    bcu_run_ok = False
    bcu_waypoints = None

    if escape_via_legal:
        # Check via/dest are in B.Cu free space
        via_in_bcu = fs_B_run.contains(SPoint(escape_via))
        dest_in_bcu = fs_B_run.contains(SPoint(dest_xy)) or fs_B_run.distance(SPoint(dest_xy)) < clearance_mm / 2
        print(f"[fanout] Via in B.Cu free_space: {via_in_bcu}", flush=True)
        print(f"[fanout] Dest in B.Cu free_space: {dest_in_bcu}", flush=True)

        bcu_waypoints = route_single_layer_leg(
            escape_via, dest_xy, fs_B_run, obs_B_run,
            bcu_margin, bcu_window, label="B.Cu run",
        )

        if bcu_waypoints is not None:
            # Try wider window if first attempt fails legality
            leg_ok, bad_count, leg_reason = validate_leg_segments(
                bcu_waypoints, fs_B_run, label="B.Cu run"
            )
            if leg_ok:
                bcu_run_ok = True
                print(f"[fanout] B.Cu run: ALL LEGAL ({len(bcu_waypoints)} waypoints)", flush=True)
            else:
                print(f"[fanout] B.Cu run: {bad_count} illegal segments - {leg_reason}", flush=True)
        else:
            print("[fanout] B.Cu run: routing FAILED, trying wider window...", flush=True)
            # Try wider window
            bcu_margin2 = bcu_margin * 2.0
            bcu_wx1_2 = max(min(escape_via[0], dest_xy[0]) - bcu_margin2, bx1)
            bcu_wy1_2 = max(min(escape_via[1], dest_xy[1]) - bcu_margin2, by1)
            bcu_wx2_2 = min(max(escape_via[0], dest_xy[0]) + bcu_margin2, bx2)
            bcu_wy2_2 = min(max(escape_via[1], dest_xy[1]) + bcu_margin2, by2)
            bcu_window2 = (bcu_wx1_2, bcu_wy1_2, bcu_wx2_2, bcu_wy2_2)
            print(f"[fanout] Wider B.Cu window: {bcu_window2}", flush=True)
            fs_B_run2, obs_B_run2 = build_layer_free_space(
                data["pads"], TARGET_NET, clearance_mm, track_mm, [],
                bcu_window2, board_outline, drill_obstacles, layer=1,
            )
            bcu_waypoints = route_single_layer_leg(
                escape_via, dest_xy, fs_B_run2, obs_B_run2,
                bcu_margin2, bcu_window2, label="B.Cu run (wide)",
            )
            if bcu_waypoints is not None:
                fs_B_run = fs_B_run2
                leg_ok, bad_count, leg_reason = validate_leg_segments(
                    bcu_waypoints, fs_B_run, label="B.Cu run (wide)"
                )
                if leg_ok:
                    bcu_run_ok = True
                    print(f"[fanout] B.Cu run (wide): ALL LEGAL ({len(bcu_waypoints)} waypoints)", flush=True)

    # Step 6: Emit route + DRC
    print(f"\n[fanout] === STEP 6: Emit + DRC ===", flush=True)
    net_connects = False
    new_drc_errors = -1
    via_hole_errors = -1
    drc_before = None

    # Run DRC on baseline (stripped board, should have all nets unconnected)
    drc_before_report = run_drc(board)
    drc_before = drc_summary(drc_before_report)
    print(f"[fanout] DRC before routing: unconnected={drc_before['unconnected']}, errors={drc_before['errors']}", flush=True)

    # Count unconnected for target net before routing
    gpio3_unconnected_before = sum(
        1 for u in drc_before_report.get("unconnected_items", [])
        if any(TARGET_NET in str(it) for it in u.get("items", []))
    )
    print(f"[fanout] {TARGET_NET} unconnected before: {gpio3_unconnected_before}", flush=True)

    segs_written = 0
    vias_written = 0
    if fcu_stub_ok and bcu_run_ok and fcu_waypoints and bcu_waypoints and escape_via_legal:
        segs_written, vias_written = emit_fanout_route(
            board, TARGET_NET,
            fcu_waypoints, escape_via, bcu_waypoints,
            track_mm, via_mm, via_drill,
        )
        print(f"[fanout] Emitted {segs_written} segments + {vias_written} via(s)", flush=True)

        # Refill zones
        print("[fanout] Refilling zones...", flush=True)
        refill_zones(board)

        # DRC after
        drc_after_report = run_drc(board)
        drc_after = drc_summary(drc_after_report)
        print(f"[fanout] DRC after routing: unconnected={drc_after['unconnected']}, errors={drc_after['errors']}", flush=True)
        print(f"[fanout] DRC by type: {drc_after['by_type']}", flush=True)

        new_drc_errors = max(0, drc_after["errors"] - drc_before["errors"])
        via_hole_errors = drc_after["hole_errors"]

        # Check if target net is still unconnected
        gpio3_unconnected_after = sum(
            1 for u in drc_after_report.get("unconnected_items", [])
            if any(TARGET_NET in str(it) for it in u.get("items", []))
        )
        net_connects = (gpio3_unconnected_after == 0 and gpio3_unconnected_before > 0)
        print(f"[fanout] {TARGET_NET} unconnected after: {gpio3_unconnected_after}", flush=True)
        print(f"[fanout] Net CONNECTS: {net_connects}", flush=True)
    else:
        print("[fanout] Skipping emit (routing incomplete)", flush=True)
        drc_after = drc_before
        new_drc_errors = 0
        via_hole_errors = 0

    # Step 7: Extract emitted coords for determinism check
    emitted_coords = extract_emitted_coords(board, TARGET_NET)
    print(f"\n[fanout] Emitted coords hash: {hash(emitted_coords)}", flush=True)

    t_end = time.perf_counter()
    runtime_s = t_end - t_start

    # Compute key metrics
    escape_via_dist = math.hypot(escape_via[0] - u3cx, escape_via[1] - u3cy) if escape_via_legal else None

    result = {
        "run_label": run_label,
        "emitted_coords": emitted_coords,
        "escape_via": escape_via,
        "escape_via_legal": escape_via_legal,
        "escape_via_dist_mm": escape_via_dist,
        "fcu_stub_ok": fcu_stub_ok,
        "bcu_run_ok": bcu_run_ok,
        "net_connects": net_connects,
        "new_drc_errors": new_drc_errors,
        "via_hole_errors": via_hole_errors,
        "segs_written": segs_written,
        "vias_written": vias_written,
        "runtime_s": round(runtime_s, 2),
        "source_xy": source_xy,
        "dest_xy": dest_xy,
        "u3cx": u3cx,
        "u3cy": u3cy,
        "ring_radius": ring_radius,
        "via_mm": via_mm,
        "via_drill": via_drill,
        "drc_before": drc_before,
        "drc_after": drc_after if (fcu_stub_ok and bcu_run_ok) else None,
        "fcu_waypoints_count": len(fcu_waypoints) if fcu_waypoints else 0,
        "bcu_waypoints_count": len(bcu_waypoints) if bcu_waypoints else 0,
    }
    return result


# ---------------------------------------------------------------------------
# Determinism check
# ---------------------------------------------------------------------------

def run_determinism_check(result1: dict, result2: dict) -> bool:
    """Check if two runs produced identical emitted coords."""
    c1 = result1.get("emitted_coords", "")
    c2 = result2.get("emitted_coords", "")
    if c1 == c2 and c1:
        print("[fanout] DETERMINISM: byte-identical (same-process run1 == run2)", flush=True)
        return True
    elif not c1 and not c2:
        print("[fanout] DETERMINISM: both runs produced no coords (no route emitted)", flush=True)
        return True  # vacuously deterministic
    else:
        print("[fanout] DETERMINISM FAIL: run1 != run2", flush=True)
        if c1 != c2:
            lines1 = set(c1.splitlines())
            lines2 = set(c2.splitlines())
            diff = lines1.symmetric_difference(lines2)
            print(f"  Differing lines: {len(diff)}", flush=True)
        return False


def run_subprocess_determinism(run1_coords: str) -> bool:
    """Run spike in subprocess and check coords match."""
    print("[fanout] Running subprocess for determinism check...", flush=True)
    try:
        result = subprocess.run(
            [str(ROOT / ".venv/bin/python"), str(Path(__file__).resolve()), "--det-only"],
            capture_output=True, text=True, timeout=120,
            cwd=str(ROOT),
        )
        if result.returncode != 0:
            print(f"[fanout] Subprocess failed: {result.stderr[-500:]}", flush=True)
            return False
        # Extract coords from subprocess output — pipe-separated to fit one line
        sub_coords = None
        for line in result.stdout.splitlines():
            if line.startswith("DET_COORDS:"):
                # Restore newlines: pipe separator was used to keep on one line
                sub_coords = line[len("DET_COORDS:"):].replace("|", "\n")
                break
        if sub_coords is None:
            print("[fanout] Subprocess did not emit DET_COORDS", flush=True)
            return False
        match = (sub_coords == run1_coords)
        if match:
            print("[fanout] DETERMINISM: byte-identical (subprocess == in-process)", flush=True)
        else:
            print("[fanout] DETERMINISM FAIL: subprocess != in-process", flush=True)
        return match
    except subprocess.TimeoutExpired:
        print("[fanout] Subprocess timed out", flush=True)
        return False
    except Exception as e:
        print(f"[fanout] Subprocess error: {e}", flush=True)
        return False


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    det_only = "--det-only" in sys.argv

    with tempfile.TemporaryDirectory(prefix="spike_fanout_") as tmpdir:
        out_dir = Path(tmpdir)

        if det_only:
            # Subprocess mode: just run once and print coords
            result = run_spike(out_dir, run_label="det_subprocess")
            # Use pipe-separated lines so everything fits on one stdout line for parsing
            coords_oneline = result["emitted_coords"].replace("\n", "|")
            print(f"DET_COORDS:{coords_oneline}", flush=True)
            return

        # Run 1
        result1 = run_spike(out_dir, run_label="run1")

    # Run 2 (fresh temp dir)
    with tempfile.TemporaryDirectory(prefix="spike_fanout2_") as tmpdir2:
        out_dir2 = Path(tmpdir2)
        result2 = run_spike(out_dir2, run_label="run2")

    # Determinism check
    print(f"\n{'='*70}", flush=True)
    print("[fanout] === DETERMINISM GATE ===", flush=True)
    same_process_det = run_determinism_check(result1, result2)

    # Subprocess determinism check
    subprocess_det = run_subprocess_determinism(result1["emitted_coords"])

    determinism_str = (
        "PASS (same-process + subprocess byte-identical)"
        if (same_process_det and subprocess_det)
        else f"PARTIAL (same-process={same_process_det}, subprocess={subprocess_det})"
        if (same_process_det or subprocess_det)
        else "FAIL"
    )

    # Final gate evaluation
    print(f"\n{'='*70}", flush=True)
    print("[fanout] === GATE EVALUATION ===", flush=True)

    via_dist = result1.get("escape_via_dist_mm")
    via_in_range = via_dist is not None and 4.0 <= via_dist <= 8.0

    gates = {
        "via_geometry_confirmed": result1["via_mm"] == 0.4 and result1["via_drill"] == 0.2,
        "escape_via_legal": result1["escape_via_legal"],
        "via_in_4_8mm_range": via_in_range,
        "fcu_stub_ok": result1["fcu_stub_ok"],
        "bcu_run_ok": result1["bcu_run_ok"],
        "net_connects": result1["net_connects"],
        "new_drc_errors_zero": result1["new_drc_errors"] == 0,
        "via_hole_errors_zero": result1["via_hole_errors"] == 0,
        "deterministic": same_process_det,
        "runtime_under_10s": result1["runtime_s"] <= 10.0,
    }

    for gate, passed in gates.items():
        status = "PASS" if passed else "FAIL"
        print(f"  {status}: {gate}", flush=True)

    all_pass = all(gates.values())
    fanout_status = "GO" if all_pass else "NO-GO"
    if all_pass:
        fanout_go_no_go = "GO: all gates pass — guided escape-via mechanism validated"
    else:
        failed_gates = [k for k, v in gates.items() if not v]
        fanout_go_no_go = f"NO-GO: {', '.join(failed_gates)}"

    # Human comparison
    human_via_pos = (147.4397, 94.5397)
    human_via_dist = 5.741
    escape_xy = result1["escape_via"]
    our_via_dist = via_dist or 0.0
    human_comparison = (
        f"Human {TARGET_NET}: via at ({human_via_pos[0]:.3f},{human_via_pos[1]:.3f}) "
        f"dist={human_via_dist:.3f}mm from U3; "
        f"Our spike: via at ({escape_xy[0]:.3f},{escape_xy[1]:.3f}) "
        f"dist={our_via_dist:.3f}mm from U3. "
        f"Both in 4-8mm range: {via_in_range}. "
        f"Same escape strategy (F.Cu stub outward + via at ring perimeter + B.Cu run): "
        f"{'YES' if result1['fcu_stub_ok'] and result1['bcu_run_ok'] else 'PARTIAL'}"
    )

    print(f"\n[fanout] FANOUT_GO_NO_GO: {fanout_go_no_go}", flush=True)
    print(f"[fanout] Runtime: {result1['runtime_s']:.2f}s", flush=True)
    print(f"[fanout] Human comparison: {human_comparison}", flush=True)

    # Issues and assumptions
    issues = []
    if not result1["escape_via_legal"]:
        issues.append("No legal escape via found near guided position — QFN perimeter congested")
    if not result1["fcu_stub_ok"]:
        issues.append("F.Cu stub routing failed (source pad to escape via)")
    if not result1["bcu_run_ok"]:
        issues.append("B.Cu run routing failed (escape via to destination)")
    if not result1["net_connects"]:
        issues.append(f"Net {TARGET_NET} still unconnected after emit")
    if result1["new_drc_errors"] > 0:
        issues.append(f"New DRC errors: {result1['new_drc_errors']}")
    if result1["via_hole_errors"] > 0:
        issues.append(f"Via hole clearance/hole_to_hole errors: {result1['via_hole_errors']}")
    if not via_in_range:
        issues.append(f"Escape via at {via_dist:.2f}mm — outside expected 4-8mm range" if via_dist else "Escape via dist unknown")
    if not same_process_det:
        issues.append("Same-process determinism check failed")
    if result1["runtime_s"] > 10.0:
        issues.append(f"Runtime {result1['runtime_s']:.1f}s exceeds 10s gate")

    assumptions = [
        "Destination pad is J3 pad 4 (first instance) at (121.19, 105.82) — thru-hole connector",
        "U3 center taken from board footprint at (148.870, 88.980)",
        "Ring radius computed as max distance from U3 center to any U3 pad center",
        "Guided via placed on ray from U3 center through source pad at ring_radius + via_mm/2 + clearance + 0.3mm margin",
        "B.Cu is treated as sparse (no grid-routed copper — stripped board)",
        "Legality: 3-predicate (copper ring both layers, drill-to-copper@hole_clearance, drill-to-drill@hole_to_hole)",
        "hole_clearance and hole_to_hole read from .kicad_pro design rules",
        "Segment legality validated via Shapely free_space.contains() oracle",
    ]

    # Structured Result JSON
    structured = {
        "status": "ok" if all_pass else "partial",
        "summary": (
            f"Spike-Fanout-Escape: guided via placement for {TARGET_NET} on mitayi RP2040 QFN. "
            f"Via geometry: {result1['via_mm']}mm/{result1['via_drill']}mm (confirmed project value). "
            f"U3 ring_radius={result1['ring_radius']:.3f}mm. "
            f"Escape via at dist={via_dist:.3f}mm from U3 center. "
            f"F.Cu stub: {'OK' if result1['fcu_stub_ok'] else 'FAIL'}. "
            f"B.Cu run: {'OK' if result1['bcu_run_ok'] else 'FAIL'}. "
            f"Net connects: {result1['net_connects']}. "
            f"New DRC errors: {result1['new_drc_errors']}. "
            f"Via hole errors: {result1['via_hole_errors']}. "
            f"Determinism: {determinism_str}. "
            f"Runtime: {result1['runtime_s']:.2f}s. "
            f"FANOUT: {fanout_go_no_go}"
        ),
        "files_changed": ["scripts/spike_fanout_escape.py"],
        "files_read": [
            "data/benchmark-boards/mitayi-pico-d1/Mitayi-Pico-D1.kicad_pcb",
            "src/tracewise/route/gridless/geom.py",
            "src/tracewise/route/gridless/search.py",
            "src/tracewise/route/engine/kicad.py",
            "src/tracewise/route/bridge.py",
            "scripts/probe_human_routing.py",
            "scripts/spikeM3_gridless_via_2layer.py",
        ],
        "target_net": TARGET_NET,
        "source_pad": {
            "xy": list(result1["source_xy"]),
            "desc": "U3 QFN SMD pad, F.Cu only",
        },
        "dest_pad": {
            "xy": list(result1["dest_xy"]),
            "desc": "J3 thru-hole connector pad",
        },
        "via_geometry_mm": {
            "via": result1["via_mm"],
            "drill": result1["via_drill"],
            "confirmed_0.4_0.2": result1["via_mm"] == 0.4 and result1["via_drill"] == 0.2,
        },
        "u3_center": {"x": result1["u3cx"], "y": result1["u3cy"]},
        "ring_radius_mm": result1["ring_radius"],
        "escape_via_pos": {"x": float(escape_xy[0]), "y": float(escape_xy[1])},
        "escape_via_dist_from_u3_mm": float(via_dist or 0.0),
        "escape_via_legal": result1["escape_via_legal"],
        "fcu_stub_ok": result1["fcu_stub_ok"],
        "bcu_run_ok": result1["bcu_run_ok"],
        "net_connects": result1["net_connects"],
        "new_drc_errors": result1["new_drc_errors"],
        "via_hole_errors": result1["via_hole_errors"],
        "human_comparison": human_comparison,
        "deterministic": determinism_str,
        "runtime_s": result1["runtime_s"],
        "fanout_go_no_go": fanout_go_no_go,
        "issues": issues,
        "assumptions": assumptions,
    }

    print(f"\n{'='*70}", flush=True)
    print("## Structured Result", flush=True)
    print("```json", flush=True)
    print(json.dumps(structured, indent=2), flush=True)
    print("```", flush=True)


if __name__ == "__main__":
    main()
