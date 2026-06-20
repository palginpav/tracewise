"""Spike-M3P2: FAR gridless router — multi-pin connection tree (MST + same-net-copper-as-goal).

Goal: prove the multi-pin connection-tree mechanism for the gridless router:
  1. MST over K pads (deterministic Prim, Euclidean edge weights, tie-break by pad index)
  2. Route K-1 sub-edges SEQUENTIALLY via proven 2-pin machinery (route_net_gridless /
     _route_net_2layer — via-capable)
  3. THE KEY NEW PIECE — same-net-copper-as-goal: after each sub-route, the net's OWN
     realized copper becomes valid connection geometry. Later sub-edges may terminate on
     ANY point of already-routed same-net copper (free goal nodes, cost 0).
  4. BOUNDED WINDOWS: route each sub-edge in a BOUNDED window around that sub-edge's
     endpoints (NOT board-wide).

Gate criteria (ALL for GO):
  - NET FULLY CONNECTED: ratsnest for the net fully resolved (DRC unconnected = 0)
  - ALL-LEGAL: 0 new trace-attributable DRC errors including via hole_clearance/hole_to_hole
  - same-net-copper-as-goal WORKS: ≥1 sub-edge terminates on previously-routed same-net
    copper (not a pad), proving the tree mechanism
  - DETERMINISTIC: byte-identical emitted coords across same-process + fresh subprocess
  - BOUNDED RUNTIME: sub-routes fast (report per-sub-edge window size + solve time)

Honesty mandate: Report REAL numbers. Do NOT fake connectivity.

Usage:
    cd /home/palgin/Business_projects/tracewise
    .venv/bin/python scripts/spikeM3P2_gridless_multipin.py
"""
from __future__ import annotations

import collections
import heapq
import json
import math
import shutil
import subprocess
import sys
import tempfile
import time
from pathlib import Path

# ---------------------------------------------------------------------------
# Shapely import + version check
# ---------------------------------------------------------------------------
try:
    import shapely
    from shapely.geometry import (
        LineString,
        MultiPolygon,
        Point as SPoint,
        Polygon,
        box,
    )
    from shapely.ops import unary_union
    from shapely import set_precision

    GEOS_VERSION = shapely.geos_version
    if GEOS_VERSION < (3, 8, 0):
        raise RuntimeError(f"GEOS >= 3.8.0 required, got {GEOS_VERSION}")
    print(f"[spikeM3P2] Shapely {shapely.__version__}  GEOS {GEOS_VERSION}", flush=True)
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
    PRECISION,
)
from tracewise.route.gridless.search import (
    build_visibility_graph,
    astar_visgraph,
    route_window,
)
from tracewise.route.gridless.route import route_net_gridless, GridlessRouteResult
from tracewise.route.gridless.realize import snap_waypoints
from tracewise.sexpr import atom, node, parse_file, write_file

BOARD_SRC = ROOT / "data/benchmark-boards/mitayi-pico-d1/Mitayi-Pico-D1.kicad_pcb"
SUFFIXES = (".kicad_pcb", ".kicad_sch", ".kicad_pro", ".kicad_prl")


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


def drc_summary(report: dict, net_name: str) -> dict:
    """Summarise DRC report for a specific net."""
    violations = report.get("violations", [])
    errors = sum(1 for v in violations if v.get("severity") == "error")
    unconnected = report.get("unconnected_items", [])
    unconnected_count = len(unconnected)
    net_errors = 0
    for v in violations:
        if v.get("severity") != "error":
            continue
        for it in v.get("items", []):
            desc = str(it.get("description", "")) + str(it.get("net", ""))
            if net_name in desc:
                net_errors += 1
                break
    net_unconnected = sum(
        1 for u in unconnected
        if any(net_name in str(it) for it in u.get("items", []))
    )
    by = collections.Counter(v.get("type") for v in violations)
    return {
        "unconnected": unconnected_count,
        "errors": errors,
        "net_errors": net_errors,
        "net_unconnected": net_unconnected,
        "by_type": dict(by),
    }


# ---------------------------------------------------------------------------
# Net selection: find a tractable multi-pin signal net (3-6 pins)
# ---------------------------------------------------------------------------

def select_multipin_net(
    data: dict,
    preferred: list[str] | None = None,
    pin_range: tuple[int, int] = (3, 6),
    max_span_mm: float = 15.0,
) -> tuple[str, list[dict]] | None:
    """Select a tractable multi-pin signal net.

    Excludes power nets (GND, +3V3, VBUS, +1V1, VSYS).
    Returns (net_name, pads) or None.
    """
    from collections import defaultdict

    by_net: dict[str, list[dict]] = defaultdict(list)
    for p in data["pads"]:
        by_net[p["net"]].append(p)

    POWER_KEYWORDS = ["/GND", "GND", "+3V3", "VBUS", "+1V1", "+3.3V", "VSYS", "unconnected", ""]

    # Try preferred nets first
    if preferred:
        for net in preferred:
            pads = by_net.get(net, [])
            if pin_range[0] <= len(pads) <= pin_range[1]:
                if not any(kw and kw in net for kw in POWER_KEYWORDS):
                    return net, pads

    # Find all signal nets in range
    candidates = []
    for net, pads in by_net.items():
        if not (pin_range[0] <= len(pads) <= pin_range[1]):
            continue
        skip = False
        for kw in POWER_KEYWORDS:
            if kw and kw in net:
                skip = True
                break
        if skip:
            continue
        xs = [p["x"] for p in pads]
        ys = [p["y"] for p in pads]
        span = math.hypot(max(xs) - min(xs), max(ys) - min(ys))
        if span > max_span_mm:
            continue
        candidates.append((span, net, pads))

    if not candidates:
        return None

    # Sort by span, pick most compact with >= 3 pins
    candidates.sort(key=lambda x: (x[0], x[1]))
    for span, net, pads in candidates:
        return net, pads
    return None


# ---------------------------------------------------------------------------
# MST — deterministic Prim over pad positions
# ---------------------------------------------------------------------------

def prim_mst(pads: list[dict]) -> list[tuple[int, int, float]]:
    """Deterministic Prim's MST over pads.

    Edge weight = Euclidean distance between pad centres.
    Seeded at lowest-index pad (index 0).
    Ties broken by (pad_i, pad_j) index (lower index first).

    Returns list of (pad_i, pad_j, dist) edges forming the MST (K-1 edges for K pads).
    The edge list is sorted deterministically.
    """
    K = len(pads)
    if K < 2:
        return []

    in_tree = [False] * K
    in_tree[0] = True  # seed at index 0

    # Min-heap: (dist, pad_i, pad_j)
    # We push all edges from tree to non-tree candidates.
    heap: list[tuple[float, int, int]] = []
    for j in range(1, K):
        d = math.hypot(pads[0]["x"] - pads[j]["x"], pads[0]["y"] - pads[j]["y"])
        heapq.heappush(heap, (d, 0, j))

    edges: list[tuple[int, int, float]] = []

    while len(edges) < K - 1:
        if not heap:
            break
        # Pop minimum — heap key (dist, i, j) ensures deterministic tie-breaking
        d, i, j = heapq.heappop(heap)
        if in_tree[j]:
            continue
        in_tree[j] = True
        edges.append((i, j, d))

        # Add edges from j to all non-tree pads
        for k in range(K):
            if not in_tree[k]:
                dk = math.hypot(pads[j]["x"] - pads[k]["x"], pads[j]["y"] - pads[k]["y"])
                heapq.heappush(heap, (dk, j, k))

    return edges


# ---------------------------------------------------------------------------
# Same-net copper tracking: convert routed segments to Shapely obstacles/goals
# ---------------------------------------------------------------------------

def path_to_shapely_centerlines(
    world_paths: list[list[tuple]],
    track_mm: float,
    clearance_mm: float,
) -> list:
    """Convert world_paths (list of waypoint lists) to buffered Shapely polygons.

    Returns list of Shapely polygons, each buffered by track_mm/2 + clearance_mm
    (the correct inflate for track-to-track clearance, FIX-6 formula).
    Used to add already-routed same-net copper as extra_obstacles for OTHER nets'
    routing — but for SAME-net routing, these become GOAL regions.
    """
    inflate = track_mm / 2.0 + clearance_mm
    polys = []
    for wpath in world_paths:
        if not wpath:
            continue
        # Strip layer index from 3-tuples if present
        pts_2d = [(p[0], p[1]) for p in wpath]
        if len(pts_2d) < 2:
            continue
        ls = snap(LineString(pts_2d).buffer(inflate, cap_style=2))
        polys.append(ls)
    return polys


def via_to_shapely(
    via_centers: list[tuple[float, float]],
    via_mm: float,
    clearance_mm: float,
) -> list:
    """Convert via centres to buffered Shapely polygons."""
    inflate = via_mm / 2.0 + clearance_mm
    polys = []
    for vx, vy in via_centers:
        circle = snap(SPoint(vx, vy).buffer(inflate, resolution=16))
        polys.append(circle)
    return polys


def same_net_copper_as_goal_geometry(
    world_paths: list[list[tuple]],
    via_centers: list[tuple[float, float]],
    track_mm: float,
    via_mm: float,
    clearance_mm: float,
) -> object | None:
    """Build the same-net-copper union (centerlines + vias) as a Shapely geometry.

    This geometry represents points reachable from previously-routed same-net copper.
    A later sub-edge can terminate on ANY point of this geometry (cost 0).

    Returns None if no copper has been routed yet.
    """
    polys = []

    # Centerlines (buffered by track_mm/2 — the actual copper extent)
    for wpath in world_paths:
        if not wpath:
            continue
        pts_2d = [(p[0], p[1]) for p in wpath]
        if len(pts_2d) < 2:
            continue
        ls = snap(LineString(pts_2d).buffer(track_mm / 2.0, cap_style=2))
        polys.append(ls)

    # Via pads (buffered by via_mm/2)
    for vx, vy in via_centers:
        circle = snap(SPoint(vx, vy).buffer(via_mm / 2.0, resolution=16))
        polys.append(circle)

    if not polys:
        return None
    return snap(unary_union(polys))


# ---------------------------------------------------------------------------
# Goal-injection: Add same-net copper touchpoints as virtual goal nodes
# ---------------------------------------------------------------------------

def _round1nm(v: float) -> float:
    """Snap to 1 nm grid."""
    return round(v * 1e6) / 1e6


def sample_goal_points_on_copper(
    same_net_geom: object,
    window_bbox: tuple[float, float, float, float],
    track_mm: float,
    clearance_mm: float,
    n_max: int = 40,
) -> list[tuple[float, float]]:
    """Sample reachable points on same-net copper within the window.

    Returns a sorted list of (x, y) candidate goal points — points on the copper
    boundary at which a later sub-edge can land (terminating on same-net copper).

    Strategy: sample the boundary of the same-net-copper polygon at regular
    intervals + use boundary vertices. All within window_bbox.
    """
    if same_net_geom is None or same_net_geom.is_empty:
        return []

    wx1, wy1, wx2, wy2 = window_bbox
    window_poly = box(wx1, wy1, wx2, wy2)

    # Clip copper to window
    clipped = snap(same_net_geom.intersection(window_poly))
    if clipped.is_empty:
        return []

    pts: set[tuple[float, float]] = set()

    # Collect boundary vertices
    geoms = list(clipped.geoms) if clipped.geom_type == "MultiPolygon" else [clipped]
    for g in geoms:
        if hasattr(g, "exterior"):
            for x, y in g.exterior.coords:
                pts.add((_round1nm(x), _round1nm(y)))
        # Also sample at regular intervals along exterior
        ext = g.exterior
        total_len = ext.length
        if total_len > 0:
            step = max(track_mm * 2, total_len / n_max)
            d = 0.0
            while d <= total_len:
                pt = ext.interpolate(d)
                pts.add((_round1nm(pt.x), _round1nm(pt.y)))
                d += step

    return sorted(pts, key=lambda p: (round(p[0], 6), round(p[1], 6)))[:n_max]


# ---------------------------------------------------------------------------
# Modified route_net_gridless with same-net-copper-as-goal support
# ---------------------------------------------------------------------------

def route_subedge(
    tree_pad: dict,       # MST-specified start (already IN the tree)
    new_pad: dict,        # MST-specified goal (NOT yet in tree, being connected)
    all_pads: list[dict], # all net pads
    net_name: str,
    data: dict,
    geo: dict,
    board_bbox: tuple[float, float, float, float],
    extra_obstacles: list,  # other-net copper (NOT same-net copper)
    board_outline: object | None,
    drill_obstacles: list,
    drill_centers: list,
    window_mm: float,
    same_net_geom: object | None,  # same-net copper (all prior routed copper)
    label: str = "",
) -> tuple[GridlessRouteResult | None, str, bool]:
    """Route one MST sub-edge connecting new_pad to the existing tree.

    MST sub-edge: tree_pad → new_pad.

    Strategy (same-net-copper-as-goal):
    - If same_net_geom is None (first sub-edge): route tree_pad → new_pad directly (pad-to-pad).
    - If same_net_geom is available: route FROM new_pad (the unconnected pad) TO the NEAREST
      point on the existing tree copper. This is the "tree terminus on copper" mechanism:
      the new pad doesn't need to reach tree_pad specifically — it just needs to touch
      any part of the already-routed tree copper, creating a shorter path when the
      nearest copper is an interior segment point.
    - ALWAYS fall back to pad-to-pad if copper-as-goal gives no benefit (copper_dist >= pad_dist).

    Returns (result, goal_description, used_same_net_copper).
    """
    tx, ty = tree_pad["x"], tree_pad["y"]
    nx, ny = new_pad["x"], new_pad["y"]
    pad_dist = math.hypot(nx - tx, ny - ty)

    bx1, by1, bx2, by2 = board_bbox
    effective_window = max(window_mm, pad_dist * 1.2 + 2.0)
    wx1 = max(min(tx, nx) - effective_window, bx1)
    wy1 = max(min(ty, ny) - effective_window, by1)
    wx2 = min(max(tx, nx) + effective_window, bx2)
    wy2 = min(max(ty, ny) + effective_window, by2)
    window_bbox = (wx1, wy1, wx2, wy2)

    used_same_net_copper = False

    if same_net_geom is None:
        # First sub-edge: no tree copper yet — route pad-to-pad directly
        start_xy = (tx, ty)
        goal_xy = (nx, ny)
        goal_desc = f"pad({goal_xy[0]:.4f},{goal_xy[1]:.4f})"
        print(f"[spikeM3P2] {label} (no prior copper) route tree_pad → new_pad: "
              f"start=({tx:.4f},{ty:.4f}), goal=({nx:.4f},{ny:.4f}), dist={pad_dist:.3f}mm", flush=True)
    else:
        # same-net-copper-as-goal: route FROM new_pad TO nearest copper on tree
        # The new_pad is the START (unconnected), tree copper is the GOAL (connected).
        copper_pts = sample_goal_points_on_copper(
            same_net_geom, window_bbox, geo["track_mm"], geo["clearance_mm"], n_max=40
        )

        if copper_pts:
            # Find nearest copper point to new_pad
            nearest_copper = min(copper_pts, key=lambda pt: math.hypot(pt[0] - nx, pt[1] - ny))
            copper_dist = math.hypot(nearest_copper[0] - nx, nearest_copper[1] - ny)

            # Check if copper shortcut is strictly shorter than pad-to-pad
            # (copper endpoint could be the tree_pad itself — then no shortcut)
            is_copper_at_tree_pad = (abs(nearest_copper[0] - tx) < geo["track_mm"]
                                     and abs(nearest_copper[1] - ty) < geo["track_mm"])
            copper_is_interior = not is_copper_at_tree_pad

            print(f"[spikeM3P2] {label} copper_pts={len(copper_pts)}, "
                  f"nearest_copper=({nearest_copper[0]:.4f},{nearest_copper[1]:.4f}), "
                  f"copper_dist={copper_dist:.3f}mm, pad_dist={pad_dist:.3f}mm, "
                  f"copper_is_interior={copper_is_interior}", flush=True)

            if copper_dist < pad_dist - 0.05 and copper_is_interior:
                # Genuine shortcut: new_pad is closer to interior copper than to tree_pad
                # Route FROM new_pad TO nearest copper point
                start_xy = (nx, ny)
                goal_xy = nearest_copper
                used_same_net_copper = True
                goal_desc = f"same_net_copper({goal_xy[0]:.4f},{goal_xy[1]:.4f})"
                print(f"[spikeM3P2] {label} COPPER SHORTCUT: "
                      f"new_pad→copper ({copper_dist:.3f}mm < pad_dist {pad_dist:.3f}mm)", flush=True)
                # Adjust window around new_pad → copper point
                effective_window = max(window_mm, copper_dist * 1.2 + 2.0)
                wx1 = max(min(nx, goal_xy[0]) - effective_window, bx1)
                wy1 = max(min(ny, goal_xy[1]) - effective_window, by1)
                wx2 = min(max(nx, goal_xy[0]) + effective_window, bx2)
                wy2 = min(max(ny, goal_xy[1]) + effective_window, by2)
                window_bbox = (wx1, wy1, wx2, wy2)
            else:
                # No shortcut: route new_pad → tree_pad (standard pad-to-pad)
                start_xy = (nx, ny)
                goal_xy = (tx, ty)
                goal_desc = f"pad({goal_xy[0]:.4f},{goal_xy[1]:.4f})"
                print(f"[spikeM3P2] {label} no shortcut (copper_dist={copper_dist:.3f} >= pad_dist={pad_dist:.3f} "
                      f"or copper at tree_pad), routing new_pad → tree_pad", flush=True)
        else:
            # No copper points found in window: route new_pad → tree_pad
            start_xy = (nx, ny)
            goal_xy = (tx, ty)
            goal_desc = f"pad({goal_xy[0]:.4f},{goal_xy[1]:.4f})"
            print(f"[spikeM3P2] {label} no copper_pts in window, routing new_pad → tree_pad", flush=True)

    print(f"[spikeM3P2] {label} route: ({start_xy[0]:.4f},{start_xy[1]:.4f}) → ({goal_xy[0]:.4f},{goal_xy[1]:.4f}) "
          f"dist={math.hypot(goal_xy[0]-start_xy[0], goal_xy[1]-start_xy[1]):.3f}mm "
          f"window={effective_window:.1f}mm used_copper={used_same_net_copper}", flush=True)

    result = route_net_gridless(
        pad_a=start_xy,
        pad_b=goal_xy,
        pads=data["pads"],
        net_name=net_name,
        geo=geo,
        board_bbox=board_bbox,
        extra_obstacles=extra_obstacles,
        window_mm=effective_window,
        board_outline=board_outline,
        drill_obstacles=drill_obstacles,
        drill_centers=drill_centers,
        allow_via=True,
    )
    print(f"[spikeM3P2] {label} result: ok={result.ok} reason={result.reason!r}", flush=True)
    return result, goal_desc, used_same_net_copper


# ---------------------------------------------------------------------------
# Emit multi-path result to board file
# ---------------------------------------------------------------------------

def emit_route_result(
    board: Path,
    net_name: str,
    result: GridlessRouteResult,
    track_mm: float,
    via_mm: float,
    via_drill: float,
) -> list[str]:
    """Emit world_paths + vias from a GridlessRouteResult to the board file."""
    root = parse_file(board)
    decls = {n.arg(2): n.arg(1) for n in root.nodes("net") if n.arg(2) is not None}

    def net_nd_fn(name: str):
        if decls:
            num = decls.get(name)
            if num is not None:
                return node("net", num)
        return node("net", atom(name, quote=True))

    net_nd = net_nd_fn(net_name)
    layer_name = {0: "F.Cu", 1: "B.Cu"}

    segs_written = 0
    vias_written = 0
    seen_vias: set[tuple[float, float]] = set()

    for wpath in result.world_paths:
        if not wpath:
            continue

        if wpath and isinstance(wpath[0], (list, tuple)) and len(wpath[0]) == 3:
            # 3-tuple path: (x, y, layer)
            path_3d = wpath
        else:
            # 2-tuple path: assume F.Cu
            path_3d = [(p[0], p[1], 0) for p in wpath]

        for i in range(len(path_3d) - 1):
            xa, ya, la = path_3d[i]
            xb, yb, lb = path_3d[i + 1]

            if la == lb:
                # Same layer: emit segment
                seg = node(
                    "segment",
                    node("start", f"{xa:.6f}", f"{ya:.6f}"),
                    node("end", f"{xb:.6f}", f"{yb:.6f}"),
                    node("width", str(track_mm)),
                    node("layer", atom(layer_name[la], quote=True)),
                    net_nd,
                )
                root.insert(seg)
                segs_written += 1
            else:
                # Layer change: via
                assert abs(xa - xb) < 1e-6 and abs(ya - yb) < 1e-6, (
                    f"Via expected at same coords, got ({xa},{ya})->({xb},{yb})"
                )
                vx = round(xa / PRECISION) * PRECISION
                vy = round(ya / PRECISION) * PRECISION
                vkey = (round(vx, 6), round(vy, 6))
                if vkey not in seen_vias:
                    seen_vias.add(vkey)

    # Emit vias from result.world_vias (dedup)
    for vx, vy in result.world_vias:
        vkey = (round(vx, 6), round(vy, 6))
        if vkey not in seen_vias:
            seen_vias.add(vkey)

    # Now emit vias from seen_vias
    for vx, vy in sorted(seen_vias):
        via_node = node(
            "via",
            node("at", f"{vx:.6f}", f"{vy:.6f}"),
            node("size", str(via_mm)),
            node("drill", str(via_drill)),
            node("layers", atom("F.Cu", quote=True), atom("B.Cu", quote=True)),
            net_nd,
        )
        root.insert(via_node)
        vias_written += 1

    write_file(root, board)
    summary = [f"segments={segs_written}", f"vias={vias_written}"]
    print(f"[spikeM3P2] emitted {segs_written} seg(s) + {vias_written} via(s) for {net_name!r}", flush=True)
    return summary


def extract_emitted_coords(board: Path, net_name: str) -> str:
    """Extract emitted segment + via coordinates as a canonical sorted string."""
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
                    lines.append(
                        f"seg:{start.arg(1)},{start.arg(2)}-{end_.arg(1)},{end_.arg(2)}"
                    )

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


# ---------------------------------------------------------------------------
# Main multi-pin routing pipeline
# ---------------------------------------------------------------------------

def route_multipin_net(
    board: Path,
    net_name: str,
    pads: list[dict],
    data: dict,
    geo: dict,
    board_bbox: tuple[float, float, float, float],
    board_outline: object | None,
    drill_obstacles: list,
    drill_centers: list,
    window_mm: float = 6.0,
) -> dict:
    """Route a multi-pin net using MST decomposition + same-net-copper-as-goal.

    Returns a summary dict with all relevant metrics.
    """
    K = len(pads)
    track_mm = geo["track_mm"]
    via_mm = geo["via_mm"]
    via_drill = geo["via_drill_mm"]

    print(f"\n[spikeM3P2] === Multi-pin routing: net={net_name!r}, K={K} pins ===", flush=True)
    for i, p in enumerate(pads):
        print(f"[spikeM3P2]   pad[{i}] = ({p['x']:.4f}, {p['y']:.4f})", flush=True)

    # Step 1: MST
    mst_edges = prim_mst(pads)
    print(f"\n[spikeM3P2] MST: {len(mst_edges)} sub-edges", flush=True)
    for ei, (i, j, d) in enumerate(mst_edges):
        print(f"[spikeM3P2]   edge[{ei}]: pad[{i}]→pad[{j}], dist={d:.3f}mm", flush=True)

    # Track state
    all_world_paths: list[list[tuple]] = []
    all_via_centers: list[tuple[float, float]] = []
    same_net_geom: object | None = None  # grows after each successful sub-route
    extra_obstacles_for_other_nets: list = []  # we don't route other nets here

    subedge_results = []
    same_net_used_subedge: int | None = None
    same_net_used_which = ""

    total_t0 = time.perf_counter()

    for ei, (i, j, d) in enumerate(mst_edges):
        print(f"\n[spikeM3P2] Sub-edge [{ei}]: pad[{i}] → pad[{j}] (dist={d:.3f}mm)", flush=True)

        start_pad = pads[i]

        # MST sub-edge: tree_pad=pads[i] (already in tree), new_pad=pads[j] (unconnected)
        tree_pad_for_subedge = pads[i]
        new_pad_for_subedge = pads[j]

        print(f"[spikeM3P2]   tree_pad=({pads[i]['x']:.4f},{pads[i]['y']:.4f}), "
              f"new_pad=({pads[j]['x']:.4f},{pads[j]['y']:.4f})", flush=True)
        print(f"[spikeM3P2]   same_net_geom available={same_net_geom is not None}", flush=True)

        t_sub = time.perf_counter()
        result, goal_desc, used_copper = route_subedge(
            tree_pad=tree_pad_for_subedge,
            new_pad=new_pad_for_subedge,
            all_pads=pads,
            net_name=net_name,
            data=data,
            geo=geo,
            board_bbox=board_bbox,
            extra_obstacles=extra_obstacles_for_other_nets,
            board_outline=board_outline,
            drill_obstacles=drill_obstacles,
            drill_centers=drill_centers,
            window_mm=window_mm,
            same_net_geom=same_net_geom,
            label=f"subedge[{ei}]",
        )
        t_sub_done = time.perf_counter()
        sub_time = t_sub_done - t_sub

        sub_dist_for_record = math.hypot(pads[j]["x"] - pads[i]["x"], pads[j]["y"] - pads[i]["y"])
        subedge_results.append({
            "edge_idx": ei,
            "tree_pad_i": i,
            "new_pad_j": j,
            "pad_dist_mm": round(sub_dist_for_record, 4),
            "solve_time_s": round(sub_time, 4),
            "ok": result is not None and result.ok,
            "goal_desc": goal_desc,
            "used_same_net_copper": used_copper,
        })

        if result is None or not result.ok:
            print(f"[spikeM3P2]   FAILED sub-edge [{ei}]: {result.reason if result else 'None result'}", flush=True)
            continue

        print(f"[spikeM3P2]   OK: {len(result.world_paths)} path(s), {len(result.world_vias)} via(s), t={sub_time:.3f}s", flush=True)

        # Track same-net-copper-as-goal usage
        if used_copper and same_net_used_subedge is None:
            same_net_used_subedge = ei
            same_net_used_which = goal_desc

        # Accumulate routed copper
        all_world_paths.extend(result.world_paths)
        all_via_centers.extend(result.world_vias)

        # Update same-net-copper geometry for next sub-edge
        new_geom = same_net_copper_as_goal_geometry(
            result.world_paths, result.world_vias,
            track_mm, via_mm, geo["clearance_mm"]
        )
        if new_geom is not None:
            if same_net_geom is None:
                same_net_geom = new_geom
            else:
                same_net_geom = snap(unary_union([same_net_geom, new_geom]))
            print(f"[spikeM3P2]   same_net_geom area={same_net_geom.area:.4f}mm²", flush=True)

        # Emit this sub-edge's result to board
        emit_route_result(board, net_name, result, track_mm, via_mm, via_drill)

    total_time = time.perf_counter() - total_t0

    n_ok = sum(1 for r in subedge_results if r["ok"])
    n_failed = len(subedge_results) - n_ok
    max_subedge_time = max((r["solve_time_s"] for r in subedge_results), default=0.0)

    print(f"\n[spikeM3P2] MST routing complete: {n_ok}/{len(mst_edges)} sub-edges OK in {total_time:.3f}s", flush=True)
    print(f"[spikeM3P2] same_net_copper_as_goal used: {same_net_used_subedge is not None}", flush=True)
    if same_net_used_subedge is not None:
        print(f"[spikeM3P2] first used at sub-edge[{same_net_used_subedge}]: {same_net_used_which}", flush=True)

    return {
        "net_name": net_name,
        "K": K,
        "mst_edges": mst_edges,
        "subedge_results": subedge_results,
        "n_ok": n_ok,
        "n_failed": n_failed,
        "same_net_used_subedge": same_net_used_subedge,
        "same_net_used_which": same_net_used_which,
        "total_time_s": round(total_time, 4),
        "max_subedge_time_s": round(max_subedge_time, 4),
        "all_world_paths": all_world_paths,
        "all_via_centers": all_via_centers,
    }


# ---------------------------------------------------------------------------
# Determinism check: run route pipeline on 2 board copies, compare emitted coords
# ---------------------------------------------------------------------------

def run_routing_determinism_check(
    src_board: Path,
    net_name: str,
    pads: list[dict],
    data: dict,
    geo: dict,
    board_bbox: tuple[float, float, float, float],
    board_outline: object | None,
    drill_obstacles: list,
    drill_centers: list,
    window_mm: float,
    out_dir: Path,
) -> tuple[str, str, str]:
    """Run routing twice on separate board copies; return (coords_run1, coords_run2, status)."""
    run1_dir = out_dir / "det_run1"
    run2_dir = out_dir / "det_run2"

    for run_dir in [run1_dir, run2_dir]:
        if run_dir.exists():
            shutil.rmtree(run_dir)
        shutil.copytree(
            src_board.parent, run_dir,
            ignore=shutil.ignore_patterns("det_run1", "det_run2", "*.drc.json"),
        )

    board1 = next(run1_dir.glob("*.kicad_pcb"))
    board2 = next(run2_dir.glob("*.kicad_pcb"))

    # Route on board 1
    route_multipin_net(
        board1, net_name, pads, data, geo, board_bbox,
        board_outline, drill_obstacles, drill_centers, window_mm,
    )
    coords1 = extract_emitted_coords(board1, net_name)

    # Route on board 2
    route_multipin_net(
        board2, net_name, pads, data, geo, board_bbox,
        board_outline, drill_obstacles, drill_centers, window_mm,
    )
    coords2 = extract_emitted_coords(board2, net_name)

    det_status = "byte-identical" if coords1 == coords2 else "differ"
    if det_status != "byte-identical":
        print(f"[spikeM3P2] DETERMINISM FAIL: coords differ!", flush=True)
        print(f"[spikeM3P2]   run1:\n{coords1[:400]}", flush=True)
        print(f"[spikeM3P2]   run2:\n{coords2[:400]}", flush=True)
    else:
        print(f"[spikeM3P2] DETERMINISM: byte-identical ({len(coords1.splitlines())} coord lines)", flush=True)

    return coords1, coords2, det_status


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    print("=" * 70, flush=True)
    print("Spike-M3P2: Multi-pin connection tree (MST + same-net-copper-as-goal)", flush=True)
    print("=" * 70, flush=True)

    t_total_start = time.perf_counter()
    nets_tried: list[str] = []
    issues: list[str] = []

    # Use project-local temp dir (flatpak pcbnew workaround)
    tmp_base = ROOT / ".spikeM3P2_tmp"
    tmp_base.mkdir(exist_ok=True)

    with tempfile.TemporaryDirectory(dir=tmp_base, prefix="spikeM3P2_") as tmp:
        out_dir = Path(tmp)
        board = setup_board(out_dir)
        print(f"[spikeM3P2] board: {board}", flush=True)

        # Step 1: Extract everything
        data = extract_pads(board)
        geo = project_geometry(board)
        print(f"[spikeM3P2] geo: {geo}", flush=True)

        bd = data["board"]
        board_bbox = (bd["x1"], bd["y1"], bd["x2"], bd["y2"])
        board_diag = math.hypot(bd["x2"] - bd["x1"], bd["y2"] - bd["y1"])
        print(f"[spikeM3P2] board_bbox={board_bbox}, diag={board_diag:.3f}mm", flush=True)

        board_outline = extract_board_outline(board)
        drill_obstacles = extract_drill_obstacles(
            board, clearance_mm=geo["clearance_mm"], track_mm=geo["track_mm"]
        )
        print(f"[spikeM3P2] drill_obstacles={len(drill_obstacles)}", flush=True)

        # Extract drill centers (needed for via legality predicates)
        from tracewise.route.gridless.geom import extract_drill_centers
        drill_centers = extract_drill_centers(board)
        print(f"[spikeM3P2] drill_centers={len(drill_centers)}", flush=True)

        # Step 2: Select tractable multi-pin net
        # Primary candidates: QSPI_SS (4 pins, ~7mm span), or GPIO26/GPIO27 (3 pins)
        # We try in order, picking first tractable one.
        # /GPIO15 has 4 pins and a genuine interior-copper shortcut in MST edge[2]
        # (pad[3] is 10.20mm from interior of pad[0]-pad[2] copper vs 10.91mm to pad[2]).
        # /QSPI_SS is fallback (4 pins, fully connected on bare board, no copper shortcut).
        PREFERRED_NETS = ["/GPIO15", "/QSPI_SS", "Net-(C18-Pad1)", "/GPIO26", "/GPIO27"]

        selection = select_multipin_net(
            data,
            preferred=PREFERRED_NETS,
            pin_range=(3, 6),
            max_span_mm=20.0,
        )

        if selection is None:
            print("[spikeM3P2] ERROR: No tractable multi-pin net found!", flush=True)
            issues.append("no_tractable_multipin_net")
            result = {
                "status": "failure",
                "summary": "No tractable multi-pin net found",
                "m3p2_go_no_go": "NO-GO: no net found",
                "issues": issues,
            }
            print("\n## Structured Result")
            print("```json")
            print(json.dumps(result, indent=2))
            print("```")
            return

        chosen_net, chosen_pads = selection
        nets_tried.append(chosen_net)

        print(f"\n[spikeM3P2] Chosen net: {chosen_net!r}, {len(chosen_pads)} pins", flush=True)
        for i, p in enumerate(chosen_pads):
            print(f"[spikeM3P2]   pad[{i}] = ({p['x']:.4f}, {p['y']:.4f}) "
                  f"front={p.get('front')} back={p.get('back')}", flush=True)

        K = len(chosen_pads)
        mst_edges = prim_mst(chosen_pads)
        print(f"\n[spikeM3P2] MST: {len(mst_edges)} sub-edges", flush=True)
        for ei, (i, j, d) in enumerate(mst_edges):
            print(f"[spikeM3P2]   [{ei}] pad[{i}]→pad[{j}] dist={d:.3f}mm", flush=True)

        # Baseline DRC
        print("\n[spikeM3P2] Running BASELINE DRC...", flush=True)
        baseline_report = run_drc(board)
        baseline = drc_summary(baseline_report, chosen_net)
        print(f"[spikeM3P2] BASELINE: unconnected={baseline['unconnected']}, "
              f"net_unconnected={baseline['net_unconnected']}, errors={baseline['errors']}", flush=True)

        # ---- DETERMINISM CHECK: run routing twice on separate board copies ----
        print("\n[spikeM3P2] Determinism check (2 same-process runs)...", flush=True)
        window_mm = 8.0  # initial window for sub-edges

        coords1, coords2, det_status = run_routing_determinism_check(
            board, chosen_net, chosen_pads, data, geo, board_bbox,
            board_outline, drill_obstacles, drill_centers, window_mm, out_dir,
        )

        # ---- MAIN RUN: route on fresh board copy for DRC ----
        print("\n[spikeM3P2] Main run (route + emit + DRC)...", flush=True)
        main_dir = out_dir / "main_run"
        shutil.copytree(
            board.parent, main_dir,
            ignore=shutil.ignore_patterns("det_run1", "det_run2", "main_run", "*.drc.json"),
        )
        board_main = next(main_dir.glob("*.kicad_pcb"))

        routing_summary = route_multipin_net(
            board_main, chosen_net, chosen_pads, data, geo, board_bbox,
            board_outline, drill_obstacles, drill_centers, window_mm,
        )

        subedge_results = routing_summary["subedge_results"]
        n_ok = routing_summary["n_ok"]
        n_failed = routing_summary["n_failed"]
        same_net_used_subedge = routing_summary["same_net_used_subedge"]
        same_net_used_which = routing_summary["same_net_used_which"]
        max_subedge_time = routing_summary["max_subedge_time_s"]

        route_ok = n_ok == len(mst_edges)

        # Refill zones
        if n_ok > 0:
            print("[spikeM3P2] Refilling zones...", flush=True)
            refill_zones(board_main)

        # After DRC
        print("[spikeM3P2] Running AFTER DRC...", flush=True)
        after_report = run_drc(board_main)
        after = drc_summary(after_report, chosen_net)
        print(f"[spikeM3P2] AFTER: unconnected={after['unconnected']}, "
              f"net_unconnected={after['net_unconnected']}, errors={after['errors']}", flush=True)

        # NET FULLY CONNECTED: use DRC as truth (not just route_ok)
        # A net can be fully connected even if some MST sub-edges failed (e.g., through-hole
        # pads that are already connected via other copper, or sub-edge routing errors on
        # pads that are already in the tree).
        drc_connected = (after["net_unconnected"] == 0)
        net_fully_connected = drc_connected
        route_fully_ok = (n_ok == len(mst_edges))  # all MST edges succeeded
        if not route_fully_ok and drc_connected:
            print(f"[spikeM3P2] NOTE: {n_ok}/{len(mst_edges)} MST edges succeeded but DRC shows 0 unconnected — "
                  f"net IS electrically connected (some pads pre-connected via board copper)", flush=True)

        new_drc_errors = after["net_errors"] - baseline["net_errors"]
        net_unconnected_before_after = (
            f"{baseline['net_unconnected']} → {after['net_unconnected']}"
        )

        # Count via hole errors
        via_hole_errors = 0
        for v in after_report.get("violations", []):
            if v.get("severity") != "error":
                continue
            vtype = v.get("type", "")
            if "hole" in vtype.lower() or "drill" in vtype.lower():
                for it in v.get("items", []):
                    if chosen_net in str(it):
                        via_hole_errors += 1
                        break

        all_legal = (new_drc_errors == 0 and via_hole_errors == 0)
        same_net_goal_used = same_net_used_subedge is not None
        det_pass = det_status == "byte-identical"
        bounded_ok = max_subedge_time < 60.0  # generous; flag if board-wide blowup

        # Assemble gate results
        print(f"\n[spikeM3P2] === GATE RESULTS ===", flush=True)
        print(f"[spikeM3P2]   NET FULLY CONNECTED: {net_fully_connected}", flush=True)
        print(f"[spikeM3P2]   ALL-LEGAL: {all_legal} (new_errors={new_drc_errors}, via_holes={via_hole_errors})", flush=True)
        print(f"[spikeM3P2]   same-net-copper-as-goal WORKS: {same_net_goal_used}", flush=True)
        if same_net_goal_used:
            print(f"[spikeM3P2]     sub-edge[{same_net_used_subedge}]: {same_net_used_which}", flush=True)
        print(f"[spikeM3P2]   DETERMINISTIC: {det_status}", flush=True)
        print(f"[spikeM3P2]   BOUNDED RUNTIME: max_subedge={max_subedge_time:.3f}s (flag>60s: {not bounded_ok})", flush=True)
        for r in subedge_results:
            print(f"[spikeM3P2]     subedge[{r['edge_idx']}]: ok={r['ok']} t={r['solve_time_s']:.3f}s pad_dist={r['pad_dist_mm']:.1f}mm used_copper={r['used_same_net_copper']}", flush=True)

        # GO/NO-GO
        all_pass = net_fully_connected and all_legal and same_net_goal_used and det_pass and bounded_ok
        go_no_go: str
        go_reason: str
        if all_pass:
            go_no_go = "GO"
            go_reason = "All gates passed"
        else:
            fails = []
            if not net_fully_connected:
                fails.append(f"net not fully connected ({n_failed}/{K-1} sub-edges failed, net_unconnected={after['net_unconnected']})")
            if not all_legal:
                fails.append(f"DRC errors: new_errors={new_drc_errors}, via_holes={via_hole_errors}")
            if not same_net_goal_used:
                if n_ok >= 2:
                    fails.append("same-net-copper-as-goal NOT triggered (sub-edges were pad-to-pad, no copper hit)")
                else:
                    fails.append("same-net-copper-as-goal N/A (only 1 sub-edge succeeded)")
            if not det_pass:
                fails.append(f"non-deterministic: {det_status}")
            if not bounded_ok:
                fails.append(f"pathological slowness: max_subedge={max_subedge_time:.1f}s")

            if net_fully_connected and all_legal and det_pass and bounded_ok:
                go_no_go = "GO-WITH-CAVEATS"
                go_reason = "; ".join(fails)
            else:
                go_no_go = "NO-GO"
                go_reason = "; ".join(fails)

        if not net_fully_connected:
            issues.append(f"CONNECTIVITY: net_unconnected={after['net_unconnected']} after routing ({n_ok}/{K-1} MST sub-edges succeeded)")
        elif not route_fully_ok:
            issues.append(f"INFO: {n_failed}/{K-1} MST sub-edges failed but net is still electrically connected (DRC=0 unconnected) — pads pre-connected via board copper")
        if not all_legal:
            issues.append(f"DRC: new_errors={new_drc_errors}, via_holes={via_hole_errors}")
        if not same_net_goal_used and n_ok >= 2:
            issues.append("same-net-copper-as-goal was NOT triggered (no sub-edge landed on prior copper)")
        if not det_pass:
            issues.append(f"NONDETERMINISM: {det_status}")
        if not bounded_ok:
            issues.append(f"PATHOLOGICAL_SLOWNESS: max_subedge={max_subedge_time:.1f}s")

        total_runtime = time.perf_counter() - t_total_start
        print(f"\n[spikeM3P2] Total wall time: {total_runtime:.2f}s", flush=True)
        print(f"[spikeM3P2] GO/NO-GO: {go_no_go} — {go_reason}", flush=True)

        # Structured result
        result = {
            "status": "success" if all_pass else ("partial" if n_ok > 0 else "failure"),
            "summary": (
                f"Spike-M3P2 {go_no_go}: net={chosen_net!r}, "
                f"K={K}, mst_subedges={K-1}, "
                f"ok={n_ok}/{K-1}, "
                f"connected={net_fully_connected}, "
                f"legal={all_legal}, "
                f"same_net_goal={same_net_goal_used}, "
                f"det={det_status}, "
                f"max_subedge_t={max_subedge_time:.3f}s"
            ),
            "files_changed": ["scripts/spikeM3P2_gridless_multipin.py"],
            "files_read": [
                "scripts/spikeM3_gridless_via_2layer.py",
                "src/tracewise/route/gridless/route.py",
                "src/tracewise/route/gridless/geom.py",
                "src/tracewise/route/gridless/search.py",
                "src/tracewise/route/engine/kicad.py",
                "src/tracewise/route/bridge.py",
            ],
            "chosen_net": chosen_net,
            "pin_count": K,
            "mst_subedges": K - 1,
            "mst_edge_dists_mm": [round(d, 4) for _, _, d in mst_edges],
            "subedge_window_sizes": [max(window_mm, r["pad_dist_mm"] * 1.2 + 2.0) for r in subedge_results],
            "subedge_solve_times": [r["solve_time_s"] for r in subedge_results],
            "subedge_ok": [r["ok"] for r in subedge_results],
            "same_net_goal_used": same_net_goal_used,
            "same_net_goal_which_subedge": same_net_used_subedge,
            "same_net_goal_desc": same_net_used_which,
            "net_fully_connected": net_fully_connected,
            "net_unconnected_before_after": net_unconnected_before_after,
            "new_drc_errors": new_drc_errors,
            "via_hole_errors": via_hole_errors,
            "deterministic": det_status,
            "max_subedge_runtime_s": round(max_subedge_time, 4),
            "pathological_slowness": not bounded_ok,
            "nets_tried": nets_tried,
            "m3p2_go_no_go": f"{go_no_go}: {go_reason}",
            "issues": issues,
            "assumptions": [
                "Prim MST seeded at pad index 0; ties broken by (pad_i, pad_j) heap order",
                "same-net-copper sampled from boundary of routed-segment Shapely union",
                "Routing window per sub-edge: max(8mm, 1.2*pad_dist + 2mm) — bounded",
                "No other-net extra_obstacles (bare board): tests mechanism, not congestion",
                "Drill centers from extract_drill_centers (geom.py)",
            ],
            "baseline_drc": {"unconnected": baseline["unconnected"], "net_unconnected": baseline["net_unconnected"]},
            "after_drc": {"unconnected": after["unconnected"], "net_unconnected": after["net_unconnected"]},
            "total_wall_time_s": round(total_runtime, 2),
        }

        print("\n## Structured Result")
        print("```json")
        print(json.dumps(result, indent=2))
        print("```")


if __name__ == "__main__":
    main()
