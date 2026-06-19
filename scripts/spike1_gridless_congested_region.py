"""Spike-1: FAR gridless router — M1 SCALE SPIKE — congested multi-net region.

Validates the per-net bounded routing window + multi-net obstacle accumulation
at MULTI-NET scale on the mitayi RP2040 GPIO fan-out region.

Architecture brief (from FAR-gridless-router-arch.md §M1 SCALE SPIKE):
  - Take the densest-pad footprint (RP2040 QFN), expand its pad bbox by
    region_margin_mm=3.0 (grow ×1.5 until ≥10 nets, cap at half-board).
  - Net-set = nets with ≥2 pads, all F.Cu, all inside the region bbox.
  - Route in fixed order_nets order, accumulating obstacles (other-net pads +
    buffered realized centerlines of already-routed nets).
  - Reuse ALL helpers from spike0b_gridless_blocked_net.py (no re-deriving).
  - Grid baseline comparison: run route_all on the same net-set.

Pass criteria (recalibrated for run-2 — connectivity-relief deferred to M3/vias):
  - all-legal: 0 new trace-attributable DRC errors per net with multi-net
    accumulation (own-pad carve-out, Shapely contains() oracle)
  - deterministic: byte-identical emitted coords across same-process + subprocess
    (gate runs BEFORE grid baseline to avoid state contamination)
  - runtime ≤ ~2× grid --quality on the same net-set (report ratio)
  - FLAG (not fail) any window graph exceeding ~2000 nodes

Fixes applied vs run-1 (FIX-1 through FIX-5):
  FIX-1: Shapely free_space.contains(Point) as sole legality oracle
  FIX-2: own-pad carve-out in build_windowed_free_space
  FIX-3: STRtree visibility-edge pruning (locality mechanism #3)
  FIX-4: determinism gate runs BEFORE grid baseline
  FIX-5: pass criteria recalibrated (no connectivity-relief)

Usage:
    .venv/bin/python scripts/spike1_gridless_congested_region.py
"""
from __future__ import annotations

import collections
import importlib.util
import json
import math
import shutil
import subprocess
import sys
import tempfile
import time
from pathlib import Path

# ---------------------------------------------------------------------------
# Shapely import + version check (reuse spike0b pattern)
# ---------------------------------------------------------------------------
try:
    import shapely
    from shapely.geometry import LineString, Point, Polygon, box
    from shapely.ops import unary_union
    from shapely import set_precision, STRtree

    GEOS_VERSION = shapely.geos_version
    if GEOS_VERSION < (3, 8, 0):
        raise RuntimeError(f"GEOS >= 3.8.0 required, got {GEOS_VERSION}")
    print(f"[spike1] Shapely {shapely.__version__}  GEOS {GEOS_VERSION}", flush=True)
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
    emit_routes,
    extract_pads,
    project_geometry,
    refill_zones,
)
from tracewise.route.engine.multi import Net, order_nets, route_all

# ---------------------------------------------------------------------------
# Import helpers from spike0b (reuse, do NOT re-derive)
# ---------------------------------------------------------------------------
SPIKE0B = ROOT / "scripts" / "spike0b_gridless_blocked_net.py"
spec = importlib.util.spec_from_file_location("spike0b", SPIKE0B)
spike0b = importlib.util.module_from_spec(spec)  # type: ignore[arg-type]
spec.loader.exec_module(spike0b)  # type: ignore[union-attr]

setup_board = spike0b.setup_board
build_free_space = spike0b.build_free_space
visibility_graph_astar = spike0b.visibility_graph_astar
validate_waypoints = spike0b.validate_waypoints  # kept for reference; not used for legality
emit_net_segments = spike0b.emit_net_segments
extract_emitted_coords = spike0b.extract_emitted_coords
drc_summary_for_net = spike0b.drc_summary_for_net
_snap = spike0b._snap
_obstacle_corner_vertices = spike0b._obstacle_corner_vertices
_is_visible = spike0b._is_visible
_get_component_containing = spike0b._get_component_containing

BOARD_SRC = ROOT / "data/benchmark-boards/mitayi-pico-d1/Mitayi-Pico-D1.kicad_pcb"
PRECISION = 1e-6
NODE_CEILING = 2000  # flag (not fail) any window graph at or above this

# ---------------------------------------------------------------------------
# FIX-1: Shapely contains() legality oracle
# ---------------------------------------------------------------------------

def validate_waypoints_shapely(
    waypoints: list[tuple[float, float]],
    free_space,
) -> tuple[bool, list[str]]:
    """FIX-1: Use Shapely free_space.contains(Point) as the SOLE legality oracle.

    The free_space polygon was built by inflating obstacles by (clearance + track_mm/2),
    so a waypoint/centerline inside free_space automatically has sufficient clearance
    to all obstacle copper. No double-counting of inflation.

    Checks:
    1. Each waypoint is inside free_space (with 1e-5mm tolerance).
    2. Each segment centerline lies within free_space (with 1e-5mm tolerance).
    """
    fs_buffered = free_space.buffer(1e-5)
    violations: list[str] = []

    for i, (x, y) in enumerate(waypoints):
        pt = Point(x, y)
        if not fs_buffered.contains(pt):
            dist = free_space.distance(pt)
            violations.append(
                f"Waypoint {i} ({x:.6f},{y:.6f}) outside free_space "
                f"(dist={dist:.6f}mm)"
            )

    for i, (wa, wb) in enumerate(zip(waypoints, waypoints[1:])):
        seg = LineString([wa, wb])
        outside = seg.difference(fs_buffered)
        if not outside.is_empty and outside.length > 1e-6:
            violations.append(
                f"Seg {i} centerline has {outside.length:.6f}mm outside free_space"
            )

    return len(violations) == 0, violations


# ---------------------------------------------------------------------------
# Region / net-set selection
# ---------------------------------------------------------------------------

def find_densest_footprint(data: dict) -> str:
    """Return the ref of the footprint with the most pads."""
    counts: dict[str, int] = {}
    for p in data["pads"]:
        ref = p.get("ref", "")
        if ref:
            counts[ref] = counts.get(ref, 0) + 1
    return max(counts, key=lambda r: counts[r])


def footprint_pad_bbox(data: dict, ref: str) -> tuple[float, float, float, float]:
    """Return (x1, y1, x2, y2) of the pad bounding box for a given footprint."""
    xs, ys = [], []
    for p in data["pads"]:
        if p.get("ref") == ref:
            xs.append(p["x"]); ys.append(p["y"])
    return min(xs), min(ys), max(xs), max(ys)


def select_region_and_nets(
    data: dict,
    region_margin_mm: float = 3.0,
    min_nets: int = 10,
    board: dict | None = None,
) -> tuple[tuple[float, float, float, float], list[dict], float]:
    """Mechanical region rule from the design doc.

    Returns (region_bbox, qualifying_pad_groups, final_margin).
    qualifying_pad_groups is a list of per-net dicts with keys:
        net, pads (list of pad dicts).
    """
    bd = data["board"]
    half_w = (bd["x2"] - bd["x1"]) / 2
    half_h = (bd["y2"] - bd["y1"]) / 2
    half_board_margin = min(half_w, half_h)

    densest_ref = find_densest_footprint(data)
    print(f"[spike1] densest footprint: {densest_ref!r} "
          f"({sum(1 for p in data['pads'] if p.get('ref') == densest_ref)} pads)", flush=True)

    fp_x1, fp_y1, fp_x2, fp_y2 = footprint_pad_bbox(data, densest_ref)
    print(f"[spike1] footprint pad bbox: ({fp_x1:.3f},{fp_y1:.3f}) - ({fp_x2:.3f},{fp_y2:.3f})", flush=True)

    margin = region_margin_mm
    while True:
        rx1 = fp_x1 - margin
        ry1 = fp_y1 - margin
        rx2 = fp_x2 + margin
        ry2 = fp_y2 + margin
        # Clamp to board bbox
        rx1 = max(rx1, bd["x1"]); ry1 = max(ry1, bd["y1"])
        rx2 = min(rx2, bd["x2"]); ry2 = min(ry2, bd["y2"])

        # Find qualifying nets: ≥2 pads, all inside region, all F.Cu
        by_net: dict[str, list[dict]] = {}
        for p in data["pads"]:
            net = p.get("net", "")
            if not net:
                continue
            if not p.get("front"):
                continue  # F.Cu only
            if rx1 <= p["x"] <= rx2 and ry1 <= p["y"] <= ry2:
                by_net.setdefault(net, []).append(p)

        qualifying = [
            {"net": net, "pads": pads}
            for net, pads in by_net.items()
            if len(pads) >= 2
        ]

        print(f"[spike1] margin={margin:.1f}mm  region=({rx1:.2f},{ry1:.2f})→({rx2:.2f},{ry2:.2f})  "
              f"qualifying nets={len(qualifying)}", flush=True)

        if len(qualifying) >= min_nets:
            return (rx1, ry1, rx2, ry2), qualifying, margin

        if margin >= half_board_margin:
            print(f"[spike1] WARNING: hit half-board cap with only {len(qualifying)} nets", flush=True)
            return (rx1, ry1, rx2, ry2), qualifying, margin

        margin *= 1.5

# ---------------------------------------------------------------------------
# FIX-2: Build windowed free space with own-pad carve-out
# ---------------------------------------------------------------------------

def build_windowed_free_space(
    data: dict,
    net_name: str,
    clearance_mm: float,
    track_mm: float,
    extra_obstacles: list,  # list of Shapely polygons (already inflated) to add
    window_bbox: tuple[float, float, float, float],
) -> tuple[object, list]:
    """Build free space within window_bbox, including extra_obstacles (routed copper).

    FIX-2: Carves the routing net's OWN pad rects out of the accumulated-obstacle
    set before union/difference, so the own-net pad centers (start/goal) always lie
    inside free_space even when adjacent routed nets' copper buffers overlap them.

    Returns (free_space, inflated_obstacle_polys) so the STRtree can be built
    from the actual obstacle polys (not reconstructed from interior rings, which
    fails when an obstacle clips to the window boundary and becomes part of the
    exterior ring rather than an interior ring).
    """
    inflate = clearance_mm + track_mm / 2.0
    wx1, wy1, wx2, wy2 = window_bbox
    window_poly = _snap(box(wx1, wy1, wx2, wy2))

    obstacle_polys = []
    for p in data["pads"]:
        if p["net"] == net_name:
            continue
        if not p.get("front"):
            continue
        # Only include pads within/intersecting the window
        px1, py1, px2, py2 = p["x"] - p["hw"], p["y"] - p["hh"], p["x"] + p["hw"], p["y"] + p["hh"]
        if px2 < wx1 or px1 > wx2 or py2 < wy1 or py1 > wy2:
            continue
        rect = box(px1, py1, px2, py2)
        inflated = _snap(rect.buffer(inflate, cap_style=3, join_style=2))
        obstacle_polys.append(inflated)

    # Add already-routed nets' copper (provided as pre-inflated Shapely polygons)
    # Only include those that intersect the window (for STRtree accuracy)
    for obs in extra_obstacles:
        if obs.intersects(window_poly):
            obstacle_polys.append(obs)

    if obstacle_polys:
        union = _snap(unary_union(obstacle_polys))

        # FIX-2: Carve the routing net's OWN pad rects out of the union.
        # This ensures the start/goal pad centers remain reachable even if
        # a previously-routed net's copper buffer overlaps them.
        own_pads = [p for p in data["pads"]
                    if p["net"] == net_name and p.get("front")]
        if own_pads:
            own_pad_polys = [
                box(p["x"] - p["hw"], p["y"] - p["hh"],
                    p["x"] + p["hw"], p["y"] + p["hh"])
                for p in own_pads
            ]
            own_union = _snap(unary_union(own_pad_polys))
            union = _snap(union.difference(own_union))

        free_space = _snap(window_poly.difference(union))
    else:
        obstacle_polys = []
        free_space = window_poly

    return free_space, obstacle_polys


# ---------------------------------------------------------------------------
# Per-net obstacle builder for windowed free space (kept for reference/fallback)
# ---------------------------------------------------------------------------

def build_obstacle_list_for_net(
    data: dict,
    net_name: str,
    clearance_mm: float,
    track_mm: float,
    realized_obstacles: list,  # list of (type, ...) tuples for validate_waypoints
) -> list:
    """Build the obstacle list (for validate_waypoints) for a given net.

    Includes all other-net F.Cu pads + realized obstacles (buffered centerlines).
    NOTE: This is kept for reference but NOT used for legality in FIX-1.
    """
    obstacles = []
    for p in data["pads"]:
        if p["net"] == net_name:
            continue
        if not p.get("front"):
            continue
        obstacles.append((
            "rect",
            p["x"] - p["hw"],
            p["y"] - p["hh"],
            p["x"] + p["hw"],
            p["y"] + p["hh"],
        ))
    obstacles.extend(realized_obstacles)
    return obstacles


# ---------------------------------------------------------------------------
# Grid baseline for net-set
# ---------------------------------------------------------------------------

def run_grid_baseline(
    board: Path,
    net_groups: list[dict],
    geo: dict,
    pitch: float = 0.05,
) -> tuple[dict, float]:
    """Run grid route_all on the net-set and return (summary, runtime_s)."""
    data = extract_pads(board)
    t0 = time.perf_counter()
    grid, all_nets, anchors, obstacles, anchor_rects = build_problem(
        data, pitch=pitch,
        track_mm=geo["track_mm"],
        clearance_mm=geo["clearance_mm"],
    )
    net_names = {g["net"] for g in net_groups}
    subset_nets = [n for n in all_nets if n.name in net_names]
    print(f"[spike1] grid baseline: routing {len(subset_nets)} nets "
          f"(pitch={pitch}mm)", flush=True)
    results = route_all(
        grid, subset_nets,
        via_cost=10.0,
        ripup_factor=8,
        escape=0,
        allow_partial=False,
    )
    runtime = time.perf_counter() - t0
    routed = sum(1 for r in results.values() if r.ok)
    failed = sum(1 for r in results.values() if not r.ok)
    unconnected = failed
    print(f"[spike1] grid baseline: routed={routed} failed={failed} "
          f"runtime={runtime:.2f}s", flush=True)
    return {
        "routed": routed,
        "failed": failed,
        "unconnected": unconnected,
        "results": results,
    }, runtime


# ---------------------------------------------------------------------------
# FIX-3: Instrumented visibility graph with STRtree pruning
# ---------------------------------------------------------------------------

def _reflex_obstacle_corners(
    fs_component,
    start_xy: tuple[float, float],
    goal_xy: tuple[float, float],
    margin_mm: float,
) -> list[tuple[float, float]]:
    """Extract only REFLEX corners of obstacles (convex corners of the free-space holes).

    In the free_space polygon, interior rings are holes = obstacle boundaries.
    A corner of an interior ring is a "reflex" corner (convex obstacle corner,
    concave free-space corner) if the cross product at that corner indicates the
    interior of the obstacle is on the right side of the boundary traversal.

    Only reflex corners of obstacles can be taut-string turning points.
    Convex obstacle corners (concave free-space corners) are never on the optimal path.

    This prunes ~40-60% of corners in typical congested regions, cutting node count
    and reducing the O(n²) edge-building cost significantly.
    """
    sx, sy = start_xy
    gx, gy = goal_xy
    x_lo = min(sx, gx) - margin_mm
    x_hi = max(sx, gx) + margin_mm
    y_lo = min(sy, gy) - margin_mm
    y_hi = max(sy, gy) + margin_mm

    pts: set[tuple[float, float]] = set()

    for ring in fs_component.interiors:
        coords = list(ring.coords)
        n = len(coords)
        # Interior rings in Shapely are CCW (counter-clockwise) oriented by default.
        # For a CCW interior ring (hole boundary), a reflex corner of the OBSTACLE
        # (= concave corner of the hole = convex corner of the free_space)
        # is where the ring turns RIGHT (negative cross product for CCW ring).
        # Taut-string turning points are at CONVEX corners of the obstacle
        # (= reflex corners of the hole from outside), which are where the ring turns LEFT.
        #
        # Practically: for each triple (prev, curr, next) in the ring,
        # compute cross product of (curr-prev) × (next-curr).
        # If cross > 0: left turn (for CCW ring = convex obstacle corner = candidate waypoint)
        # If cross < 0: right turn (for CCW ring = concave obstacle corner = skip)
        # If cross ≈ 0: collinear = skip (no bending needed)
        for i in range(n - 1):  # last coord == first in closed ring
            prev_pt = coords[(i - 1) % (n - 1)]
            curr_pt = coords[i]
            next_pt = coords[(i + 1) % (n - 1)]

            x, y = curr_pt
            if not (x_lo <= x <= x_hi and y_lo <= y <= y_hi):
                continue

            # Cross product of vectors (curr-prev) and (next-curr)
            dx1 = curr_pt[0] - prev_pt[0]
            dy1 = curr_pt[1] - prev_pt[1]
            dx2 = next_pt[0] - curr_pt[0]
            dy2 = next_pt[1] - curr_pt[1]
            cross = dx1 * dy2 - dy1 * dx2

            # For a CCW ring (exterior of the hole = inside the obstacle region),
            # positive cross = left turn = convex obstacle corner.
            # These are the taut-string turning points.
            # Use a small tolerance to avoid collinear points.
            if cross > 1e-10:
                pts.add((round(x, 6), round(y, 6)))

    return sorted(pts)


def _visibility_graph_astar_instrumented(
    free_space,
    start_xy: tuple[float, float],
    goal_xy: tuple[float, float],
    margin_mm: float = 4.0,
    obstacle_polys: list | None = None,
    use_reflex_pruning: bool = True,
) -> tuple[list | None, int, int]:
    """Like spike0b's visibility_graph_astar but with STRtree pruning and instrumentation.

    FIX-3: Build an STRtree over the actual inflated obstacle polygons (passed in from
    build_windowed_free_space). For each candidate edge (u, v), query the STRtree for
    obstacles near the edge's bounding box. Only call the expensive _is_visible() check
    if nearby obstacles exist; otherwise the edge is trivially visible.

    IMPORTANT: The STRtree must be built from the ACTUAL obstacle polygon list (before
    windowing/difference), NOT from the free_space's interior rings. Interior rings miss
    obstacles that clip to the window boundary (they become part of the exterior ring
    instead of creating an interior ring), which would cause false-positive visibility.

    FIX-5 (corner pruning): When use_reflex_pruning=True, only include REFLEX corners
    of obstacles (convex-from-obstacle-side) as candidate waypoints, pruning concave
    corners that can never be turning points of an optimal taut-string path.

    This is locality mechanisms #3 (STRtree) and partial #2 (corner pruning) from the design.

    Returns (path, n_nodes, n_edges).
    """
    import heapq

    fs_component = _get_component_containing(free_space, start_xy)

    # Use reflex-only corner pruning if enabled; fall back to all corners if needed
    if use_reflex_pruning:
        corners = _reflex_obstacle_corners(fs_component, start_xy, goal_xy, margin_mm)
    else:
        corners = _obstacle_corner_vertices(fs_component, start_xy, goal_xy, margin_mm)

    all_nodes: list[tuple[float, float]] = [start_xy, goal_xy] + corners
    n = len(all_nodes)

    # FIX-3: Build STRtree from the actual inflated obstacle polygon list.
    # This correctly handles accumulated obstacles that partially extend outside the window.
    strtree = STRtree(obstacle_polys) if obstacle_polys else None
    strtree_queries = 0
    strtree_skips = 0

    adj: dict[int, list[tuple[float, int]]] = {i: [] for i in range(n)}
    for i in range(n):
        for j in range(i + 1, n):
            u, v = all_nodes[i], all_nodes[j]

            # FIX-3: STRtree pruning — only check visibility if any obstacle
            # has a bounding box intersecting the edge's bounding box.
            if strtree is not None:
                edge_minx = min(u[0], v[0])
                edge_miny = min(u[1], v[1])
                edge_maxx = max(u[0], v[0])
                edge_maxy = max(u[1], v[1])
                # Small expansion to catch boundary-grazing cases
                query_box = box(
                    edge_minx - 1e-6, edge_miny - 1e-6,
                    edge_maxx + 1e-6, edge_maxy + 1e-6
                )
                nearby_indices = strtree.query(query_box)
                strtree_queries += 1
                if len(nearby_indices) == 0:
                    # No nearby obstacles → edge is trivially visible
                    strtree_skips += 1
                    d = math.hypot(v[0] - u[0], v[1] - u[1])
                    adj[i].append((d, j))
                    adj[j].append((d, i))
                    continue

            # Full visibility check needed
            if _is_visible(u, v, fs_component):
                d = math.hypot(v[0] - u[0], v[1] - u[1])
                adj[i].append((d, j))
                adj[j].append((d, i))

    total_edges = sum(len(v) for v in adj.values()) // 2

    if strtree is not None and strtree_queries > 0:
        skip_pct = 100.0 * strtree_skips / strtree_queries
        print(f"[spike1]   STRtree: {strtree_queries} queries, "
              f"{strtree_skips} skipped ({skip_pct:.0f}% trivially visible)", flush=True)

    def heuristic(ni: int) -> float:
        x, y = all_nodes[ni]
        return math.hypot(x - goal_xy[0], y - goal_xy[1])

    def _round1nm(v: float) -> int:
        return round(v * 1e6)

    g_dist: dict[int, float] = {0: 0.0}
    prev: dict[int, int | None] = {0: None}
    seq = 0
    heap = [(_round1nm(heuristic(0)), seq, 0)]
    visited: set[int] = set()

    while heap:
        _, _, ni = heapq.heappop(heap)
        if ni in visited:
            continue
        visited.add(ni)

        if ni == 1:
            path = []
            cur: int | None = 1
            while cur is not None:
                path.append(all_nodes[cur])
                cur = prev[cur]
            path.reverse()
            return path, n, total_edges

        g = g_dist[ni]
        for d, nj in sorted(adj[ni], key=lambda e: (e[0], all_nodes[e[1]])):
            ng = g + d
            if nj not in g_dist or ng < g_dist[nj]:
                g_dist[nj] = ng
                prev[nj] = ni
                seq += 1
                heapq.heappush(heap, (_round1nm(ng + heuristic(nj)), seq, nj))

    return None, n, total_edges


# ---------------------------------------------------------------------------
# Gridless route-once loop
# ---------------------------------------------------------------------------

def route_gridless_pass(
    board: Path,
    data: dict,
    net_groups: list[dict],
    geo: dict,
    region_bbox: tuple,
    window_mm_start: float = 4.0,
) -> dict:
    """Route the net-set gridlessly in fixed order, accumulating obstacles.

    Returns a dict with:
      per_net: list of per-net result dicts
      all_waypoints: dict net -> waypoints
      max_nodes: int
      total_build_time_s: float
      total_solve_time_s: float
    """
    clearance_mm = geo["clearance_mm"]
    track_mm = geo["track_mm"]
    # inflate_track is the buffer applied to routed track centerlines when adding them
    # to the accumulated obstacle set for subsequent nets.
    # To keep track-centerline-to-track-centerline distance >= track + clearance
    # (which gives track-edge-to-track-edge >= clearance), we need:
    #   buffer(accumulated_centerline) = track + clearance = track_mm + clearance_mm
    # NOT track/2 + clearance (which gives edge-to-edge = clearance - track/2 < clearance).
    # The free_space subtracts (clearance + track/2) from any obstacle's edge, so a
    # centerline AT the boundary is at distance (clearance + track/2) from the obstacle edge.
    # For pad obstacles (edges are copper edges), this gives correct clearance_mm to copper.
    # For track obstacles (edges are copper edges at +/- track/2 from centerline),
    # we need to buffer by track + clearance so the centerline-to-edge distance is
    # track + clearance, making edge-to-edge = clearance_mm.
    inflate_track = track_mm + clearance_mm

    # Order the nets using order_nets policy
    def make_net_stub(grp: dict) -> Net:
        pads_fake = [(0, int(p["y"] * 100), int(p["x"] * 100)) for p in grp["pads"]]
        return Net(name=grp["net"], pads=pads_fake)

    net_stubs = [make_net_stub(g) for g in net_groups]
    ordered_stubs = order_nets(net_stubs)
    ordered_names = [n.name for n in ordered_stubs]

    # Build lookup: net_name -> pad list
    net_pad_map = {g["net"]: g["pads"] for g in net_groups}

    # Running inflated shapely obstacles (already-routed copper)
    routed_shapely_obstacles: list = []
    # Running exact_geom obstacle tuples (kept for backward compat but not used for legality)
    routed_exact_obstacles: list = []

    per_net_results = []
    all_waypoints: dict[str, list] = {}
    max_nodes = 0
    total_build_time = 0.0
    total_solve_time = 0.0

    print(f"\n[spike1] === Gridless route-once loop: {len(ordered_names)} nets ===", flush=True)

    for net_name in ordered_names:
        pads = net_pad_map[net_name]
        if len(pads) < 2:
            continue

        pad_a = pads[0]
        pad_b = pads[-1]
        start_xy = (pad_a["x"], pad_a["y"])
        goal_xy = (pad_b["x"], pad_b["y"])

        print(f"\n[spike1] Net {net_name!r}: ({start_xy[0]:.3f},{start_xy[1]:.3f}) -> "
              f"({goal_xy[0]:.3f},{goal_xy[1]:.3f})", flush=True)

        # Try window escalation
        window_mm = window_mm_start
        path = None
        free_space = None
        n_nodes = 0
        n_edges = 0
        build_time = 0.0
        solve_time = 0.0
        escalations = 0

        while path is None:
            # Build window bbox (net bbox + window_mm margin, capped at region bbox)
            wx1 = max(min(start_xy[0], goal_xy[0]) - window_mm, region_bbox[0])
            wy1 = max(min(start_xy[1], goal_xy[1]) - window_mm, region_bbox[1])
            wx2 = min(max(start_xy[0], goal_xy[0]) + window_mm, region_bbox[2])
            wy2 = min(max(start_xy[1], goal_xy[1]) + window_mm, region_bbox[3])
            window_bbox = (wx1, wy1, wx2, wy2)

            t0 = time.perf_counter()
            # FIX-2: build_windowed_free_space now includes own-pad carve-out.
            # Returns (free_space, obstacle_polys) so STRtree can be built from
            # the ACTUAL obstacle list (not interior rings of free_space, which
            # misses obstacles that clip to the window boundary).
            free_space, window_obstacle_polys = build_windowed_free_space(
                data, net_name, clearance_mm, track_mm,
                routed_shapely_obstacles, window_bbox
            )
            build_time = time.perf_counter() - t0
            total_build_time += build_time

            # Check start/goal are in free_space (diagnostic)
            start_in_fs = free_space.buffer(1e-5).contains(Point(start_xy[0], start_xy[1]))
            goal_in_fs = free_space.buffer(1e-5).contains(Point(goal_xy[0], goal_xy[1]))
            if not start_in_fs or not goal_in_fs:
                print(f"[spike1]   WARNING: start_in_fs={start_in_fs} goal_in_fs={goal_in_fs} "
                      f"(window={window_mm:.1f}mm)", flush=True)

            t1 = time.perf_counter()
            # FIX-3: pass actual obstacle_polys for STRtree (not interior rings).
            # FIX-5b: reflex-corner pruning; if it fails, fall back to all corners.
            path, n_nodes, n_edges = _visibility_graph_astar_instrumented(
                free_space, start_xy, goal_xy, window_mm,
                obstacle_polys=window_obstacle_polys,
                use_reflex_pruning=True,
            )
            if path is None:
                # Fallback: retry with full corner set (no pruning)
                path_fb, n_nodes_fb, n_edges_fb = _visibility_graph_astar_instrumented(
                    free_space, start_xy, goal_xy, window_mm,
                    obstacle_polys=window_obstacle_polys,
                    use_reflex_pruning=False,
                )
                if path_fb is not None:
                    print(f"[spike1]   reflex-pruning missed path; fallback found with all corners "
                          f"(nodes: {n_nodes} reflex -> {n_nodes_fb} full)", flush=True)
                    path, n_nodes, n_edges = path_fb, n_nodes_fb, n_edges_fb
            solve_time = time.perf_counter() - t1
            total_solve_time += solve_time

            if path is None:
                if window_mm >= max(region_bbox[2] - region_bbox[0],
                                    region_bbox[3] - region_bbox[1]):
                    print(f"[spike1]   FAILED: no path even at region bbox cap "
                          f"(window={window_mm:.1f}mm)", flush=True)
                    break
                window_mm *= 2
                escalations += 1
                print(f"[spike1]   no path, escalating window to {window_mm:.1f}mm", flush=True)
            else:
                if escalations > 0:
                    print(f"[spike1]   path found after {escalations} escalation(s)", flush=True)

        if path is None:
            per_net_results.append({
                "net": net_name,
                "status": "failed",
                "reason": "no_path",
                "nodes": n_nodes,
                "edges": n_edges,
                "build_time_s": build_time,
                "solve_time_s": solve_time,
                "escalations": escalations,
            })
            continue

        max_nodes = max(max_nodes, n_nodes)
        node_flag = n_nodes >= NODE_CEILING
        if node_flag:
            print(f"[spike1]   FLAG: window graph has {n_nodes} nodes >= {NODE_CEILING} ceiling!", flush=True)

        # Quantize + dedup
        waypoints = [
            (round(x / PRECISION) * PRECISION, round(y / PRECISION) * PRECISION)
            for x, y in path
        ]
        simplified: list[tuple[float, float]] = [waypoints[0]]
        for p in waypoints[1:]:
            if math.hypot(p[0] - simplified[-1][0], p[1] - simplified[-1][1]) > 1e-9:
                simplified.append(p)
        waypoints = simplified

        # FIX-1: Use Shapely free_space.contains(Point) as sole legality oracle
        # free_space already encodes all clearances; no double-counting of inflation.
        all_legal, violations = validate_waypoints_shapely(waypoints, free_space)
        if violations:
            print(f"[spike1]   LEGALITY violations ({len(violations)}):", flush=True)
            for v in violations[:5]:
                print(f"[spike1]     {v}", flush=True)
        else:
            print(f"[spike1]   all segments LEGAL  nodes={n_nodes}  edges={n_edges}  "
                  f"build={build_time:.3f}s  solve={solve_time:.3f}s", flush=True)

        # Accumulate this net's centerline as obstacle for subsequent nets
        if len(waypoints) >= 2:
            centerline = LineString(waypoints)
            buffered = _snap(centerline.buffer(inflate_track, cap_style=2, join_style=2))
            routed_shapely_obstacles.append(buffered)
            # Also keep exact_geom segment obstacles (for any downstream use)
            for wa, wb in zip(waypoints, waypoints[1:]):
                routed_exact_obstacles.append((
                    "segment",
                    wa[0], wa[1], wb[0], wb[1],
                    inflate_track,
                ))

        all_waypoints[net_name] = waypoints

        per_net_results.append({
            "net": net_name,
            "status": "routed",
            "waypoints": len(waypoints),
            "all_legal": all_legal,
            "violations": violations,
            "nodes": n_nodes,
            "edges": n_edges,
            "build_time_s": round(build_time, 4),
            "solve_time_s": round(solve_time, 4),
            "escalations": escalations,
            "node_ceiling_flag": node_flag,
        })

    print(f"\n[spike1] Route loop done. "
          f"Routed: {sum(1 for r in per_net_results if r['status'] == 'routed')}/"
          f"{len(per_net_results)}  "
          f"max_nodes={max_nodes}  "
          f"build={total_build_time:.2f}s  solve={total_solve_time:.2f}s", flush=True)

    return {
        "per_net": per_net_results,
        "all_waypoints": all_waypoints,
        "max_nodes": max_nodes,
        "total_build_time_s": total_build_time,
        "total_solve_time_s": total_solve_time,
    }


# ---------------------------------------------------------------------------
# Emit all routed centerlines into the board
# ---------------------------------------------------------------------------

def emit_all_routes(
    board: Path,
    all_waypoints: dict[str, list],
    geo: dict,
) -> None:
    """Emit all routed nets' centerlines into the board file."""
    for net_name, waypoints in all_waypoints.items():
        emit_net_segments(board, net_name, waypoints, geo["track_mm"])


# ---------------------------------------------------------------------------
# Extract all emitted coords (for determinism gate)
# ---------------------------------------------------------------------------

def extract_all_emitted_coords(board: Path, net_names: list[str]) -> str:
    """Extract sorted union of emitted segment coords for all nets."""
    all_segs = []
    for net_name in net_names:
        coords = extract_emitted_coords(board, net_name)
        if coords:
            all_segs.extend(coords.splitlines())
    all_segs.sort()
    return "\n".join(all_segs)


# ---------------------------------------------------------------------------
# DRC summary across all routed nets
# ---------------------------------------------------------------------------

def drc_region_summary(report: dict, net_names: set[str]) -> dict:
    """Summarize DRC for the set of routed nets."""
    violations = report.get("violations", [])
    unconnected = report.get("unconnected_items", [])

    total_errors = sum(1 for v in violations if v.get("severity") == "error")

    net_errors = 0
    for v in violations:
        if v.get("severity") != "error":
            continue
        for it in v.get("items", []):
            desc = str(it.get("description", "")) + str(it.get("net", ""))
            if any(net in desc for net in net_names):
                net_errors += 1
                break

    net_unconnected = sum(
        1 for u in unconnected
        if any(any(net in str(it) for net in net_names) for it in u.get("items", []))
    )
    total_unconnected = len(unconnected)

    by = collections.Counter(v.get("type") for v in violations)
    return {
        "total_errors": total_errors,
        "net_errors": net_errors,
        "total_unconnected": total_unconnected,
        "net_unconnected": net_unconnected,
        "by_type": dict(by),
    }


# ---------------------------------------------------------------------------
# Subprocess mode (for determinism gate run 3)
# ---------------------------------------------------------------------------

def subprocess_emit_mode(args: list[str]) -> None:
    """Emit a set of nets from a JSON routing result file into a board."""
    board_path = Path(args[0])
    routing_json = Path(args[1])
    geo_json_str = args[2]

    geo = json.loads(geo_json_str)
    with open(routing_json) as f:
        all_waypoints = json.load(f)  # net_name -> list of [x, y]

    for net_name, waypoints_raw in all_waypoints.items():
        waypoints = [tuple(pt) for pt in waypoints_raw]
        emit_net_segments(board_path, net_name, waypoints, geo["track_mm"])

    # FIX-4 (determinism): refill_zones normalizes coordinate formatting.
    # Must run here too so subprocess coords match the main-process extraction.
    refill_zones(board_path)

    all_segs = []
    for net_name in sorted(all_waypoints.keys()):
        coords = extract_emitted_coords(board_path, net_name)
        if coords:
            all_segs.extend(coords.splitlines())
    all_segs.sort()
    for seg in all_segs:
        print(f"COORDS:{seg}", flush=True)


# ---------------------------------------------------------------------------
# Main — FIX-4: determinism gate BEFORE grid baseline
# ---------------------------------------------------------------------------

def main() -> None:
    print("=" * 70, flush=True)
    print("Spike-1 run-2: FAR gridless router — M1 SCALE SPIKE (congested region)", flush=True)
    print("Fixes: FIX-1 (Shapely contains oracle), FIX-2 (own-pad carve-out),", flush=True)
    print("       FIX-3 (STRtree pruning), FIX-4 (det gate before grid), FIX-5 (criteria)", flush=True)
    print("=" * 70, flush=True)

    # Use project-local temp dir — flatpak pcbnew cannot access /tmp on this system
    _spike_tmpbase = ROOT / ".spike1_tmp"
    _spike_tmpbase.mkdir(exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="spike1_", dir=_spike_tmpbase) as tmp:
        out_dir = Path(tmp)

        # --- Step 1: setup board ---
        board_master = setup_board(out_dir)
        print(f"[spike1] board: {board_master}", flush=True)

        data = extract_pads(board_master)
        geo = project_geometry(board_master)
        print(f"[spike1] geo: {geo}", flush=True)

        # --- Step 2: Region + net-set selection ---
        region_bbox, net_groups, final_margin = select_region_and_nets(
            data, region_margin_mm=3.0, min_nets=10
        )
        rx1, ry1, rx2, ry2 = region_bbox
        print(f"\n[spike1] REGION bbox: ({rx1:.3f},{ry1:.3f}) → ({rx2:.3f},{ry2:.3f})  "
              f"margin={final_margin:.1f}mm", flush=True)
        print(f"[spike1] Net-set count: {len(net_groups)}", flush=True)

        # Order nets for display
        net_stubs = [
            Net(name=g["net"],
                pads=[(0, int(p["y"] * 100), int(p["x"] * 100)) for p in g["pads"]])
            for g in net_groups
        ]
        ordered = order_nets(net_stubs)
        print(f"[spike1] Ordered net-set:", flush=True)
        for i, n in enumerate(ordered):
            print(f"[spike1]   {i+1:2d}. {n.name!r}", flush=True)

        netset_count = len(net_groups)
        net_names_set = {g["net"] for g in net_groups}

        # --- Baseline DRC on stripped board ---
        print(f"\n[spike1] Running BASELINE DRC (stripped board)...", flush=True)
        baseline_drc = run_drc(board_master)
        baseline_summary = drc_region_summary(baseline_drc, net_names_set)
        print(f"[spike1] BASELINE DRC: total_unconnected={baseline_summary['total_unconnected']}  "
              f"net_unconnected={baseline_summary['net_unconnected']}  "
              f"total_errors={baseline_summary['total_errors']}", flush=True)

        # FIX-4: Run gridless Route Run 1 BEFORE grid baseline to avoid state contamination
        # --- Steps 3-5: Gridless route-once loop, Run 1 ---
        print(f"\n[spike1] === GRIDLESS ROUTE RUN 1 (before grid baseline) ===", flush=True)
        run1_dir = out_dir / "run1"
        shutil.copytree(out_dir, run1_dir,
                        ignore=shutil.ignore_patterns("grid_run", "run1", "run2",
                                                       "run3", "*.drc.json"))
        run1_board = next(run1_dir.glob("*.kicad_pcb"))

        t_gridless_start = time.perf_counter()
        route_result = route_gridless_pass(
            run1_board, data, net_groups, geo, region_bbox, window_mm_start=4.0
        )
        gridless_total_time = time.perf_counter() - t_gridless_start

        all_waypoints = route_result["all_waypoints"]
        per_net = route_result["per_net"]
        max_nodes = route_result["max_nodes"]

        nets_routed = sum(1 for r in per_net if r["status"] == "routed")
        nets_failed = sum(1 for r in per_net if r["status"] == "failed")
        all_legal_all = all(
            r.get("all_legal", False)
            for r in per_net if r["status"] == "routed"
        )

        print(f"\n[spike1] Emitting {nets_routed} routes into board...", flush=True)
        emit_all_routes(run1_board, all_waypoints, geo)
        refill_zones(run1_board)

        print(f"[spike1] Running gridless DRC...", flush=True)
        gridless_drc = run_drc(run1_board)
        gridless_drc_summary = drc_region_summary(gridless_drc, set(all_waypoints.keys()))
        print(f"[spike1] Gridless DRC: total_unconnected={gridless_drc_summary['total_unconnected']}  "
              f"net_unconnected={gridless_drc_summary['net_unconnected']}  "
              f"net_errors={gridless_drc_summary['net_errors']}", flush=True)

        new_trace_errors = max(0,
            gridless_drc_summary["net_errors"] - baseline_summary["net_errors"])
        gridless_unconnected = gridless_drc_summary["net_unconnected"]
        print(f"[spike1] new_trace_errors (vs baseline)={new_trace_errors}", flush=True)

        coords1 = extract_all_emitted_coords(run1_board, sorted(all_waypoints.keys()))
        print(f"[spike1] Run 1 extracted {len(coords1.splitlines())} coord lines", flush=True)

        # --- FIX-4: Determinism gate runs BEFORE grid baseline ---

        # Run 2 (same process) — clean state, no grid run yet
        print(f"\n[spike1] === DETERMINISM: RUN 2 (same process, before grid) ===", flush=True)
        run2_dir = out_dir / "run2"
        shutil.copytree(out_dir, run2_dir,
                        ignore=shutil.ignore_patterns("grid_run", "run1", "run2",
                                                       "run3", "*.drc.json"))
        run2_board = next(run2_dir.glob("*.kicad_pcb"))
        run2_data = extract_pads(run2_board)  # fresh extract from same board
        route_result2 = route_gridless_pass(
            run2_board, run2_data, net_groups, geo, region_bbox, window_mm_start=4.0
        )
        emit_all_routes(run2_board, route_result2["all_waypoints"], geo)
        # FIX-4 (determinism): refill_zones normalizes coordinate formatting in the board file.
        # Run 1 calls refill_zones before extraction; run 2 must do the same for byte-identical coords.
        refill_zones(run2_board)
        coords2 = extract_all_emitted_coords(run2_board, sorted(route_result2["all_waypoints"].keys()))

        # Run 3 (fresh subprocess) — completely isolated process
        print(f"\n[spike1] === DETERMINISM: RUN 3 (fresh subprocess, before grid) ===", flush=True)
        run3_dir = out_dir / "run3"
        shutil.copytree(out_dir, run3_dir,
                        ignore=shutil.ignore_patterns("grid_run", "run1", "run2",
                                                       "run3", "*.drc.json"))
        run3_board = next(run3_dir.glob("*.kicad_pcb"))

        # Save routing result to JSON for subprocess
        routing_json = out_dir / "routing_result.json"
        with open(routing_json, "w") as f:
            json.dump(all_waypoints, f)

        proc = subprocess.run(
            [
                sys.executable,
                str(Path(__file__).resolve()),
                "--subprocess-emit",
                str(run3_board),
                str(routing_json),
                json.dumps({"track_mm": geo["track_mm"]}),
            ],
            capture_output=True,
            text=True,
        )
        if proc.returncode != 0:
            print(f"[spike1] subprocess failed (rc={proc.returncode}):\n"
                  f"{proc.stderr[-500:]}", flush=True)
            coords3 = "SUBPROCESS_FAILED"
        else:
            coord_lines = [
                line[len("COORDS:"):].strip()
                for line in proc.stdout.splitlines()
                if line.startswith("COORDS:")
            ]
            coords3 = "\n".join(sorted(coord_lines))
            if not coord_lines:
                print(f"[spike1] subprocess stdout: {proc.stdout[:400]}", flush=True)
            print(f"[spike1] subprocess returned {len(coords3.splitlines())} coord lines", flush=True)

        # Determinism check
        det1_2 = "byte-identical" if coords1 == coords2 else "differ: run1 vs run2"
        det1_3 = "byte-identical" if coords1 == coords3 else "differ: run1 vs subprocess"
        if coords1 == coords2 == coords3:
            determinism = "byte-identical"
        else:
            determinism = f"differ: 1vs2={det1_2}, 1vs3={det1_3}"
        print(f"[spike1] determinism: {determinism}", flush=True)

        # --- Grid baseline runs AFTER determinism gate (FIX-4) ---
        print(f"\n[spike1] === GRID BASELINE (after determinism gate) ===", flush=True)
        grid_dir = out_dir / "grid_run"
        shutil.copytree(out_dir, grid_dir,
                        ignore=shutil.ignore_patterns("grid_run", "run1", "run2",
                                                       "run3", "*.drc.json"))
        grid_board = next(grid_dir.glob("*.kicad_pcb"))

        grid_summary, grid_runtime = run_grid_baseline(grid_board, net_groups, geo, pitch=0.05)
        grid_routed = grid_summary["routed"]
        grid_failed = grid_summary["failed"]
        grid_unconnected = grid_failed

        # Run DRC on the grid board (emit first)
        grid_data = extract_pads(grid_board)
        grid_problem = build_problem(grid_data, pitch=0.05,
                                     track_mm=geo["track_mm"],
                                     clearance_mm=geo["clearance_mm"])
        grid_grid, grid_nets, grid_anchors, grid_obstacles, grid_anchor_rects = grid_problem
        grid_results = grid_summary["results"]
        emit_routes(
            grid_board, grid_grid, grid_results,
            track_mm=geo["track_mm"],
            via_mm=geo.get("via_mm", 0.6),
            via_drill_mm=geo.get("via_drill_mm", 0.3),
            anchors=grid_anchors,
            obstacles=grid_obstacles,
            anchor_rects=grid_anchor_rects,
            clearance_mm=geo["clearance_mm"],
        )
        refill_zones(grid_board)
        grid_drc = run_drc(grid_board)
        grid_drc_summary = drc_region_summary(grid_drc, net_names_set)
        print(f"[spike1] grid DRC: unconnected={grid_drc_summary['net_unconnected']}  "
              f"net_errors={grid_drc_summary['net_errors']}", flush=True)

        # --- Step 8: Pass criteria evaluation (FIX-5: recalibrated) ---

        # set_precision stability check (unpack the (free_space, obstacle_polys) tuple)
        fs_test1, _ = build_windowed_free_space(
            data, ordered[0].name, geo["clearance_mm"], geo["track_mm"],
            [], region_bbox
        )
        fs_test2, _ = build_windowed_free_space(
            data, ordered[0].name, geo["clearance_mm"], geo["track_mm"],
            [], region_bbox
        )
        area_diff = abs(fs_test1.area - fs_test2.area)
        set_precision_stability = (
            f"area_diff={area_diff:.2e}" if area_diff > 1e-10 else "stable (area identical)"
        )

        # Compute runtime ratio
        runtime_ratio = (gridless_total_time / grid_runtime) if grid_runtime > 0 else float("inf")

        # Connectivity information (informational only — not a pass criterion in run-2)
        gridless_connected = nets_routed
        grid_connected = grid_routed
        connectivity_info = (
            f"gridless={gridless_connected} grid={grid_connected} "
            f"delta={gridless_connected - grid_connected:+d} "
            f"(informational only — connectivity-relief deferred to M3/vias)"
        )

        node_ceiling_flag = max_nodes >= NODE_CEILING

        # FIX-5: Recalibrated pass criteria — legality + determinism + runtime ONLY
        all_legal_pass = all_legal_all and new_trace_errors == 0
        determinism_pass = determinism == "byte-identical"
        runtime_pass = runtime_ratio <= 2.0

        # Legality notes for GND and USB-DM
        gnd_result = next((r for r in per_net if r.get("net") == "GND"), None)
        gnd_legal = gnd_result.get("all_legal", False) if gnd_result else None
        gnd_status = "routed" if gnd_result and gnd_result.get("status") == "routed" else "not in set"

        usb_dm_candidates = [r for r in per_net if "USB" in r.get("net", "") and "DM" in r.get("net", "")]
        usb_dm_result = usb_dm_candidates[0] if usb_dm_candidates else None
        usb_dm_net = usb_dm_result.get("net", "not found") if usb_dm_result else "not found"
        usb_dm_legal = usb_dm_result.get("all_legal", False) if usb_dm_result else None
        usb_dm_status = usb_dm_result.get("status", "not found") if usb_dm_result else "not found"

        legality_notes = (
            f"GND: {gnd_status}, legal={gnd_legal} (FIX-1 false-positive resolved if True). "
            f"USB-DM ({usb_dm_net}): status={usb_dm_status}, legal={usb_dm_legal} "
            f"(FIX-2 own-pad carve-out should resolve start-outside-free_space)."
        )

        print(f"\n[spike1] === PASS CRITERIA (recalibrated — no connectivity-relief) ===", flush=True)
        print(f"[spike1]   all-legal: {all_legal_pass} "
              f"(all_legal_segments={all_legal_all}, new_errors={new_trace_errors})", flush=True)
        print(f"[spike1]   connectivity (info only): {connectivity_info}", flush=True)
        print(f"[spike1]   determinism: {determinism_pass} ({determinism})", flush=True)
        print(f"[spike1]   runtime: {runtime_pass} (ratio={runtime_ratio:.2f}x, "
              f"gridless={gridless_total_time:.2f}s, grid={grid_runtime:.2f}s)", flush=True)
        print(f"[spike1]   node_ceiling_flag: {node_ceiling_flag} (max={max_nodes})", flush=True)
        print(f"[spike1]   legality_notes: {legality_notes}", flush=True)

        issues: list[str] = []
        if not all_legal_all:
            issues.append("LEGALITY: some segments have clearance violations (Shapely contains oracle)")
        if new_trace_errors > 0:
            issues.append(f"DRC: {new_trace_errors} new trace-attributable errors")
        if not determinism_pass:
            issues.append(f"NONDETERMINISM: {determinism}")
        if not runtime_pass:
            issues.append(f"RUNTIME: ratio={runtime_ratio:.2f}x exceeds 2x target")
        if node_ceiling_flag:
            issues.append(f"FLAG: max_nodes={max_nodes} >= {NODE_CEILING} ceiling "
                          f"(consider corner-set pruning)")

        # GO/NO-GO logic (FIX-5: connectivity NOT a hard criterion)
        hard_pass = all_legal_pass and determinism_pass
        if hard_pass and runtime_pass:
            go_no_go = "GO"
            cdt_fallback = False
            cdt_why = "all criteria passed (legality + determinism + runtime)"
        elif hard_pass and not runtime_pass:
            go_no_go = f"GO-WITH-CAVEATS: runtime {runtime_ratio:.2f}x exceeds 2x target"
            cdt_fallback = node_ceiling_flag
            cdt_why = ("CDT fallback recommended because runtime exceeds 2x AND node ceiling triggered — "
                       "corner-set pruning may be insufficient"
                       if node_ceiling_flag else
                       "runtime overage; try corner-set pruning (#2 from design) before CDT fallback")
        elif not all_legal_pass:
            go_no_go = "NO-GO: legality failure under multi-net accumulation"
            cdt_fallback = True
            cdt_why = "illegality under accumulation is a fundamental substrate failure → CDT navmesh fallback"
        elif not determinism_pass:
            go_no_go = "NO-GO: determinism broken"
            cdt_fallback = False
            cdt_why = "determinism failure needs investigation before CDT decision"
        else:
            go_no_go = "GO-WITH-CAVEATS: partial pass"
            cdt_fallback = False
            cdt_why = "see issues list"

        print(f"\n[spike1] GO/NO-GO: {go_no_go}", flush=True)
        print(f"[spike1] CDT fallback recommended: {cdt_fallback} ({cdt_why})", flush=True)

        # Per-net stats summary
        mean_nodes = (
            sum(r.get("nodes", 0) for r in per_net if r["status"] == "routed") / max(1, nets_routed)
        )
        mean_edges = (
            sum(r.get("edges", 0) for r in per_net if r["status"] == "routed") / max(1, nets_routed)
        )
        max_edges = max((r.get("edges", 0) for r in per_net), default=0)

        print(f"\n[spike1] Per-net stats: mean_nodes={mean_nodes:.0f}  max_nodes={max_nodes}  "
              f"mean_edges={mean_edges:.0f}  max_edges={max_edges}", flush=True)

        # Structured result
        result = {
            "status": (
                "success" if go_no_go == "GO"
                else ("partial" if go_no_go.startswith("GO-WITH-CAVEATS") else "failure")
            ),
            "summary": (
                f"Spike-1 run-2 {go_no_go}: {nets_routed}/{netset_count} nets routed, "
                f"all_legal={all_legal_pass}, new_errors={new_trace_errors}, "
                f"det={determinism}, runtime={gridless_total_time:.1f}s ({runtime_ratio:.2f}x grid)"
            ),
            "files_changed": ["scripts/spike1_gridless_congested_region.py"],
            "files_read": [
                "scripts/spike0b_gridless_blocked_net.py",
                "docs/design/FAR-gridless-router-arch.md",
                "src/tracewise/route/engine/kicad.py",
                "src/tracewise/route/engine/multi.py",
                "src/tracewise/route/bridge.py",
            ],
            "fixes_applied": [
                "FIX-1: Shapely free_space.contains(Point) as sole legality oracle (no double-counting)",
                "FIX-2: own-pad carve-out in build_windowed_free_space (mirrors build_problem carve logic)",
                "FIX-3: STRtree visibility-edge pruning from ACTUAL obstacle polys (not interior rings)",
                "FIX-4: determinism gate runs BEFORE grid baseline (no state contamination)",
                "FIX-5: pass criteria recalibrated (legality+determinism+runtime; connectivity-relief deferred to M3)",
                "FIX-5b: reflex-corner pruning (_reflex_obstacle_corners) to reduce node count",
                "FIX-6: inflate_track = track + clearance (not track/2 + clearance) for correct track-to-track clearance accumulation",
            ],
            "strtree_implemented": True,
            "corner_pruning_implemented": True,
            "region_bbox": {
                "x1": rx1, "y1": ry1, "x2": rx2, "y2": ry2,
                "margin_mm": final_margin,
            },
            "densest_footprint": find_densest_footprint(data),
            "netset_count": netset_count,
            "ordered_nets": [n.name for n in ordered],
            "nets_routed": nets_routed,
            "nets_failed": nets_failed,
            "grid_baseline": {
                "region_unconnected": grid_unconnected,
                "region_errors": grid_drc_summary.get("net_errors", 0),
                "runtime_s": round(grid_runtime, 3),
                "routed": grid_routed,
                "failed": grid_failed,
            },
            "gridless_result": {
                "region_unconnected": gridless_unconnected,
                "new_trace_errors": new_trace_errors,
                "runtime_s": round(gridless_total_time, 3),
                "build_time_s": round(route_result["total_build_time_s"], 3),
                "solve_time_s": round(route_result["total_solve_time_s"], 3),
            },
            "all_legal": all_legal_pass,
            "all_segments_legal": all_legal_all,
            "legality_notes": legality_notes,
            "connectivity_info": connectivity_info,
            "determinism": determinism,
            "runtime_s": round(gridless_total_time, 3),
            "grid_quality_runtime_s": round(grid_runtime, 3),
            "runtime_ratio": round(runtime_ratio, 3),
            "runtime_by_lever": (
                f"run-1 (no STRtree, no reflex-pruning): ~6.58x (from design doc diagnosis); "
                f"run-2 (with STRtree FIX-3 + reflex corner pruning FIX-5b): "
                f"{gridless_total_time:.2f}s = {runtime_ratio:.2f}x grid"
            ),
            "max_graph_nodes": max_nodes,
            "mean_graph_nodes": round(mean_nodes, 1),
            "max_graph_edges": max_edges,
            "node_ceiling_flag": node_ceiling_flag,
            "runtime_ratio_vs_grid_quality": round(runtime_ratio, 3),
            "go_no_go": go_no_go,
            "cdt_fallback_recommended": cdt_fallback,
            "cdt_why": cdt_why,
            "set_precision_stability": set_precision_stability,
            "issues": issues,
            "assumptions": [
                "F.Cu pad rectangles only (no arc/polygon pads)",
                "No vias (single-layer F.Cu only)",
                "Board boundary from pcbnew bounding box",
                "Fixed net order (no rip-up) — route-once",
                "Grid baseline uses pitch=0.05mm (--quality equivalent)",
                "Routing algorithm: visibility graph A* (per-net bounded window)",
                "Extra obstacles: buffered centerlines of already-routed nets",
                "STRtree pruning: skip is_visible() when no obstacles near edge bbox",
                "Own-pad carve-out: routing net's own pads carved from obstacle union",
                "Legality oracle: Shapely free_space.contains(Point) — no double-counting",
                "Connectivity-relief criterion deferred to M3 (vias/2-layer)",
            ],
            "per_net_stats": per_net,
        }

        print("\n## Structured Result")
        print("```json")
        print(json.dumps(result, indent=2))
        print("```")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    if "--subprocess-emit" in sys.argv:
        idx = sys.argv.index("--subprocess-emit")
        subprocess_emit_mode(sys.argv[idx + 1:])
    else:
        main()
