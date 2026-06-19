"""Spike-2 RUN-2: FAR gridless router — M2 CONGESTION NEGOTIATION + BOUNDED RIP-UP (run-2).

Three fixes over run-1 (spike2_gridless_congestion_ripup.py):

  FIX-A  Remove pre-classification BYPASS.
         Wide-window nets MUST participate in rip-up starting from their
         min_needed_window, NOT be routed directly at escalated window.
         They enter the rip-up queue like any other net; only when the
         blockage oracle says "fixed obstacle only" (not rippable, even in
         pad-only free space) do they escalate.

  FIX-B  Midpoint-blocked edge pre-filter in build_congestion_priced_visgraph.
         Before the expensive seg.buffer().difference() visibility check,
         sample a few points along each edge and reject the edge immediately
         if any sample falls inside an obstacle polygon.  This keeps the graph
         sparse even at large windows with many nodes (run-1's 287-node windows
         dominated runtime).  Target: ≤ 3× grid --quality.

  FIX-C  Geometry-blocked classification.
         When a net's min_needed_window (in pad-only free space) exceeds 50%
         of the board diagonal, flag it "geometry-blocked → M3 (needs B.Cu/via)"
         and report separately.  Do NOT board-scale-escalate such nets.
         Board-scale escalation count must be 0.

Usage:
    .venv/bin/python scripts/spike2_run2_congestion_ripup.py
"""
from __future__ import annotations

import importlib.util
import json
import math
import shutil
import subprocess
import sys
import tempfile
import time
from dataclasses import dataclass, field
from pathlib import Path

# ---------------------------------------------------------------------------
# Dependency check
# ---------------------------------------------------------------------------
try:
    import numpy as np
    from shapely import STRtree
    from shapely.geometry import LineString, Point, box
    pass  # unary_union not needed directly; used via geom module
except ImportError as exc:
    print(f"ERROR: Required dependency missing: {exc}", file=sys.stderr)
    sys.exit(1)

# ---------------------------------------------------------------------------
# Path setup
# ---------------------------------------------------------------------------
ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "src"))

from tracewise.route.bridge import run_drc
from tracewise.route.engine.kicad import (
    extract_pads,
    project_geometry,
    refill_zones,
)
from tracewise.route.engine.multi import Net, order_nets, route_all
from tracewise.route.gridless.geom import (
    build_windowed_free_space,
    get_component_containing,
)
from tracewise.route.gridless.realize import snap_waypoints
from tracewise.route.gridless.search import (
    edge_blocked_by_obstacles_fast,
    is_visible_fast,
    obstacle_corners,
    reflex_obstacle_corners,
)

# Import helpers from spike1
SPIKE1 = ROOT / "scripts" / "spike1_gridless_congested_region.py"
spec = importlib.util.spec_from_file_location("spike1", SPIKE1)
spike1 = importlib.util.module_from_spec(spec)  # type: ignore[arg-type]
spec.loader.exec_module(spike1)  # type: ignore[union-attr]

setup_board = spike1.setup_board
emit_net_segments = spike1.emit_net_segments
extract_emitted_coords = spike1.extract_emitted_coords
extract_all_emitted_coords = spike1.extract_all_emitted_coords
drc_region_summary = spike1.drc_region_summary
select_region_and_nets = spike1.select_region_and_nets
emit_all_routes = spike1.emit_all_routes
validate_waypoints_shapely = spike1.validate_waypoints_shapely
_snap = spike1._snap

BOARD_SRC = ROOT / "data/benchmark-boards/mitayi-pico-d1/Mitayi-Pico-D1.kicad_pcb"
NODE_CEILING = 2000

# ---------------------------------------------------------------------------
# Super-cell congestion lattice (unchanged from run-1)
# ---------------------------------------------------------------------------

SUPERCELL_SIZE_MM = 0.5


@dataclass
class SuperCellGrid:
    x0: float
    y0: float
    nx: int
    ny: int
    history: np.ndarray = field(default_factory=lambda: np.array([]))

    def __post_init__(self):
        if self.history.shape == (0,):
            self.history = np.zeros((self.ny, self.nx), dtype=np.float64)

    def supercell_of(self, x: float, y: float) -> tuple[int, int]:
        sx = int(math.floor((x - self.x0) / SUPERCELL_SIZE_MM))
        sy = int(math.floor((y - self.y0) / SUPERCELL_SIZE_MM))
        sx = max(0, min(self.nx - 1, sx))
        sy = max(0, min(self.ny - 1, sy))
        return sy, sx

    def edge_history_cost(
        self,
        u: tuple[float, float],
        v: tuple[float, float],
        history_factor: float,
    ) -> float:
        d = math.hypot(v[0] - u[0], v[1] - u[1])
        if d < 1e-9:
            return 0.0
        if history_factor == 0.0:
            return d
        step = SUPERCELL_SIZE_MM / 2.0
        n_steps = max(1, int(math.ceil(d / step)))
        cells: set[tuple[int, int]] = set()
        for k in range(n_steps + 1):
            t = k / n_steps
            x = u[0] + t * (v[0] - u[0])
            y = u[1] + t * (v[1] - u[1])
            cells.add(self.supercell_of(x, y))
        if cells:
            mean_hist = float(np.mean([self.history[sy, sx] for sy, sx in cells]))
        else:
            mean_hist = 0.0
        return d * (1.0 + history_factor * mean_hist)

    def supercells_for_path(
        self, waypoints: list[tuple[float, float]]
    ) -> list[tuple[int, int]]:
        seen: set[tuple[int, int]] = set()
        result: list[tuple[int, int]] = []
        for (x0, y0), (x1, y1) in zip(waypoints, waypoints[1:]):
            d = math.hypot(x1 - x0, y1 - y0)
            step = SUPERCELL_SIZE_MM / 2.0
            n_steps = max(1, int(math.ceil(d / step))) if d > 1e-9 else 1
            for k in range(n_steps + 1):
                t = k / n_steps if n_steps > 0 else 0.0
                x = x0 + t * (x1 - x0)
                y = y0 + t * (y1 - y0)
                c = self.supercell_of(x, y)
                if c not in seen:
                    seen.add(c)
                    result.append(c)
        return result

    def deposit(self, supercells: list[tuple[int, int]], amount: float = 1.0) -> None:
        for sy, sx in supercells:
            self.history[sy, sx] += amount


def make_supercell_grid(region_bbox: tuple[float, float, float, float]) -> SuperCellGrid:
    x1, y1, x2, y2 = region_bbox
    nx = max(1, int(math.ceil((x2 - x1) / SUPERCELL_SIZE_MM)) + 2)
    ny = max(1, int(math.ceil((y2 - y1) / SUPERCELL_SIZE_MM)) + 2)
    x0 = x1 - SUPERCELL_SIZE_MM
    y0 = y1 - SUPERCELL_SIZE_MM
    return SuperCellGrid(x0=x0, y0=y0, nx=nx, ny=ny)


# ---------------------------------------------------------------------------
# FIX-B: Midpoint-blocked edge pre-filter
# ---------------------------------------------------------------------------

def _edge_is_blocked_by_obstacle(
    u: tuple[float, float],
    v: tuple[float, float],
    nearby_obstacles: list,
    n_samples: int = 3,
) -> bool:
    """Quickly reject edges whose sampled interior points are inside an obstacle.

    Samples n_samples evenly-spaced interior points along the edge (not endpoints)
    and checks if any falls inside a nearby obstacle polygon.  This is a fast
    pre-filter before the expensive seg.buffer().difference() visibility check.

    Conservative: only rejects if a sample is INSIDE an obstacle (not just near it).
    This avoids false rejections for boundary-tangent paths.
    """
    if not nearby_obstacles or n_samples < 1:
        return False
    for k in range(1, n_samples + 1):
        t = k / (n_samples + 1)
        px = u[0] + t * (v[0] - u[0])
        py = u[1] + t * (v[1] - u[1])
        pt = Point(px, py)
        for obs in nearby_obstacles:
            try:
                if obs.contains(pt):
                    return True
            except Exception:
                pass
    return False


def _round1nm(v: float) -> int:
    return round(v * 1e6)


def build_congestion_priced_visgraph(
    free_space,
    start_xy: tuple[float, float],
    goal_xy: tuple[float, float],
    margin_mm: float,
    obstacle_polys: list,
    sc_grid: SuperCellGrid,
    history_factor: float,
    use_reflex_pruning: bool = True,
) -> tuple[list[tuple[float, float]], dict[int, list[tuple[float, int]]], int, int]:
    """Build a visibility graph with congestion-priced edge costs.

    FIX-B: Before the expensive seg.buffer().difference() visibility check,
    run a midpoint-blocked pre-filter using the nearby obstacles from the STRtree.
    This significantly reduces calls to the expensive Shapely check for large windows.

    OPT-2 (vectorized contains): replace the ``Point()``-per-sample loop in the
    midpoint pre-filter with ``edge_blocked_by_obstacles_fast`` which uses
    ``shapely.contains_xy`` with a numpy array of obstacles — ~6× faster.

    OPT-3 (batch STRtree query): pre-compute all edge bounding boxes and issue
    a single STRtree.query(array) call instead of O(n²) individual queries —
    ~8× speedup on the STRtree step.
    """
    fs_component = get_component_containing(free_space, start_xy)

    if use_reflex_pruning:
        corners = reflex_obstacle_corners(fs_component, start_xy, goal_xy, margin_mm)
    else:
        corners = obstacle_corners(fs_component, start_xy, goal_xy, margin_mm)

    all_nodes: list[tuple[float, float]] = [start_xy, goal_xy] + corners
    n = len(all_nodes)

    obs_poly_arr = np.array(obstacle_polys) if obstacle_polys else None

    # OPT-3: batch all edge bounding-box STRtree queries in one call.
    # Build the (i,j) edge list and their corresponding query boxes.
    edges_ij: list[tuple[int, int]] = []
    for i in range(n):
        for j in range(i + 1, n):
            edges_ij.append((i, j))
    n_edges_total = len(edges_ij)

    # nearby_obs_per_edge[k] = numpy array of obstacle indices near edge k
    # (None means no strtree / no obstacles)
    if obstacle_polys and obs_poly_arr is not None:
        strtree = STRtree(obstacle_polys)
        n_obs = len(obstacle_polys)

        # Build query boxes for all edges at once
        xs = np.array([all_nodes[i][0] for i, j in edges_ij], dtype=np.float64)
        ys = np.array([all_nodes[i][1] for i, j in edges_ij], dtype=np.float64)
        xe = np.array([all_nodes[j][0] for i, j in edges_ij], dtype=np.float64)
        ye = np.array([all_nodes[j][1] for i, j in edges_ij], dtype=np.float64)

        minx = np.minimum(xs, xe) - 1e-6
        miny = np.minimum(ys, ye) - 1e-6
        maxx = np.maximum(xs, xe) + 1e-6
        maxy = np.maximum(ys, ye) + 1e-6

        # Build all edge query boxes as a numpy array of Shapely boxes
        # box(minx, miny, maxx, maxy) vectorized
        import shapely as _shapely
        edge_q_boxes = _shapely.box(minx, miny, maxx, maxy)

        # Batch STRtree query: returns (input_idx, tree_idx) pairs
        input_idxs, tree_idxs = strtree.query(edge_q_boxes)

        # Group tree_idxs by input (edge) index
        # Only keep valid tree_idxs (< n_obs)
        valid_mask = tree_idxs < n_obs
        input_idxs_valid = input_idxs[valid_mask]
        tree_idxs_valid = tree_idxs[valid_mask]

        # Build per-edge nearby obstacle arrays using np.unique to find split points
        if len(input_idxs_valid) > 0:
            # Sort by input_idx for efficient grouping
            sort_order = np.argsort(input_idxs_valid, kind='stable')
            sorted_input = input_idxs_valid[sort_order]
            sorted_tree = tree_idxs_valid[sort_order]
            split_points = np.searchsorted(sorted_input, np.arange(n_edges_total + 1))
            nearby_tree_per_edge = [
                sorted_tree[split_points[k]:split_points[k + 1]]
                for k in range(n_edges_total)
            ]
        else:
            nearby_tree_per_edge = [np.array([], dtype=np.int64)] * n_edges_total
    else:
        nearby_tree_per_edge = None

    adj: dict[int, list[tuple[float, int]]] = {i: [] for i in range(n)}
    for edge_idx, (i, j) in enumerate(edges_ij):
        u, v = all_nodes[i], all_nodes[j]

        if nearby_tree_per_edge is not None:
            nearby_indices = nearby_tree_per_edge[edge_idx]
            if len(nearby_indices) == 0:
                # No nearby obstacles -> trivially visible
                d_priced = sc_grid.edge_history_cost(u, v, history_factor)
                adj[i].append((d_priced, j))
                adj[j].append((d_priced, i))
                continue

            # OPT-2 (FIX-B): midpoint pre-filter using vectorized contains_xy
            nearby_obs_arr = obs_poly_arr[nearby_indices]
            if edge_blocked_by_obstacles_fast(u, v, nearby_obs_arr, n_samples=3):
                # Fast reject — skip the expensive visibility check
                continue

        # Full visibility check (only for edges that pass the pre-filter)
        if is_visible_fast(u, v, fs_component):
            d_priced = sc_grid.edge_history_cost(u, v, history_factor)
            adj[i].append((d_priced, j))
            adj[j].append((d_priced, i))

    # Sort adjacency deterministically
    for i in range(n):
        adj[i] = sorted(adj[i], key=lambda e: (e[0], all_nodes[e[1]]))

    total_edges = sum(len(v) for v in adj.values()) // 2
    return all_nodes, adj, n, total_edges


def astar_congestion_priced(
    all_nodes: list[tuple[float, float]],
    adj: dict[int, list[tuple[float, int]]],
    goal_xy: tuple[float, float],
) -> list[tuple[float, float]] | None:
    """Deterministic A* with congestion-priced edge costs."""
    import heapq

    n = len(all_nodes)
    if n == 0:
        return None

    def heuristic(ni: int) -> float:
        x, y = all_nodes[ni]
        return math.hypot(x - goal_xy[0], y - goal_xy[1])

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
            path: list[tuple[float, float]] = []
            cur: int | None = 1
            while cur is not None:
                path.append(all_nodes[cur])
                cur = prev[cur]
            path.reverse()
            return path

        g = g_dist[ni]
        for d, nj in adj[ni]:
            ng = g + d
            if nj not in g_dist or ng < g_dist[nj]:
                g_dist[nj] = ng
                prev[nj] = ni
                seq += 1
                heapq.heappush(heap, (_round1nm(ng + heuristic(nj)), seq, nj))

    return None


# ---------------------------------------------------------------------------
# Routed net state
# ---------------------------------------------------------------------------

@dataclass
class GridlessRoute:
    net_name: str
    waypoints: list[tuple[float, float]]
    shapely_obstacle: object
    supercells_used: list[tuple[int, int]]
    n_nodes: int = 0
    n_edges: int = 0
    build_time_s: float = 0.0
    solve_time_s: float = 0.0
    escalations: int = 0
    window_mm: float = 0.0


# ---------------------------------------------------------------------------
# Classify blockage: fixed-obstacle vs rippable-net
# ---------------------------------------------------------------------------

def _classify_blockage(
    start_xy: tuple[float, float],
    goal_xy: tuple[float, float],
    routed: dict[str, GridlessRoute],
    pad_obstacles: list,
    free_space_padonly,
    window_bbox: tuple,
    protected_nets: set[str] | None = None,
) -> tuple[bool, str | None]:
    """Determine whether a routing failure is due to fixed obstacles or a rippable net.

    FIX-D: protected_nets are excluded from victim selection.  Escalated routes
    (routes that needed a wider window to succeed) are protected to prevent the
    thrash cycle: ripping them forces them to escalate again, which blocks the
    original net again, creating an infinite cycle.
    """
    from tracewise.route.gridless.search import route_window

    if protected_nets is None:
        protected_nets = set()

    if not routed:
        return False, None

    margin = max(abs(goal_xy[0] - start_xy[0]), abs(goal_xy[1] - start_xy[1]), 2.0)
    path_padonly, _, _ = route_window(
        free_space_padonly,
        start_xy,
        goal_xy,
        margin,
        pad_obstacles,
    )

    if path_padonly is None:
        return False, None

    # Pad-only has a path -> routed copper is blocking
    sx, sy = start_xy
    gx, gy = goal_xy
    try:
        straight_line = LineString([(sx, sy), (gx, gy)])
        corridor = straight_line.buffer(1.5, cap_style=2)
    except Exception:
        wx1, wy1, wx2, wy2 = window_bbox
        corridor = box(wx1, wy1, wx2, wy2)

    best_victim: str | None = None
    best_overlap: float = -1.0
    for net_name, nr in routed.items():
        if net_name in protected_nets:
            # FIX-D: Skip escalated/protected nets as victims to prevent thrash cycle
            continue
        try:
            ov = nr.shapely_obstacle.intersection(corridor).area
        except Exception:
            ov = 0.0
        if ov > best_overlap:
            best_overlap = ov
            best_victim = net_name

    if best_overlap < 1e-6:
        return False, None

    return True, best_victim


# ---------------------------------------------------------------------------
# FIX-C: Geometry-blocked classification
# ---------------------------------------------------------------------------

def _classify_geometry_blocked(
    net_name: str,
    min_needed_window: float,
    board_bbox: tuple[float, float, float, float],
    geom_block_threshold: float = 0.5,
) -> bool:
    """Return True if this net is geometry-blocked on single-layer F.Cu.

    A net is geometry-blocked when the minimum window it needs in pad-only
    free space exceeds geom_block_threshold * board_diagonal.  Such nets
    need B.Cu / vias (M3 work) and should NOT trigger board-scale escalation.
    """
    bx1, by1, bx2, by2 = board_bbox
    board_diag = math.hypot(bx2 - bx1, by2 - by1)
    return min_needed_window > geom_block_threshold * board_diag


# ---------------------------------------------------------------------------
# Single-net route attempt with congestion pricing
# ---------------------------------------------------------------------------

def _route_one_net(
    net_name: str,
    start_xy: tuple[float, float],
    goal_xy: tuple[float, float],
    data: dict,
    geo: dict,
    region_bbox: tuple,
    routed_obstacles: list,
    sc_grid: SuperCellGrid,
    history_factor: float,
    window_mm_start: float,
    board_bbox: tuple,
    allow_window_escalation: bool = True,
) -> tuple[list[tuple[float, float]] | None, float, int, int, int, int]:
    """Attempt to route net_name with congestion pricing.

    Returns (waypoints, window_mm_used, n_nodes, n_edges, escalations, n_board_scale_escs).
    """
    clearance_mm = geo["clearance_mm"]
    track_mm = geo["track_mm"]

    rx1, ry1, rx2, ry2 = region_bbox
    bx1, by1, bx2, by2 = board_bbox
    region_diag = math.hypot(rx2 - rx1, ry2 - ry1)

    window_mm = window_mm_start
    path: list[tuple[float, float]] | None = None
    n_nodes = 0
    n_edges = 0
    escalations = 0
    board_scale_escalations = 0

    while True:
        wx1 = max(min(start_xy[0], goal_xy[0]) - window_mm, bx1)
        wy1 = max(min(start_xy[1], goal_xy[1]) - window_mm, by1)
        wx2 = min(max(start_xy[0], goal_xy[0]) + window_mm, bx2)
        wy2 = min(max(start_xy[1], goal_xy[1]) + window_mm, by2)
        window_bbox = (wx1, wy1, wx2, wy2)

        free_space, obstacle_polys = build_windowed_free_space(
            data["pads"], net_name, clearance_mm, track_mm,
            routed_obstacles, window_bbox,
        )

        fs_buf = free_space.buffer(1e-5)
        if not fs_buf.contains(Point(*start_xy)) or not fs_buf.contains(Point(*goal_xy)):
            if allow_window_escalation and window_mm < region_diag:
                window_mm = min(window_mm * 2.0, region_diag)
                escalations += 1
                continue
            break

        for use_reflex in [True, False]:
            all_nodes, adj, n_nodes, n_edges = build_congestion_priced_visgraph(
                free_space, start_xy, goal_xy, window_mm, obstacle_polys,
                sc_grid, history_factor, use_reflex_pruning=use_reflex,
            )
            path = astar_congestion_priced(all_nodes, adj, goal_xy)
            if path is not None:
                break

        if path is not None:
            break

        if not allow_window_escalation:
            break

        current_window_size = max(wx2 - wx1, wy2 - wy1)
        board_scale = max(bx2 - bx1, by2 - by1)
        if current_window_size >= board_scale * 0.95:
            board_scale_escalations += 1
            break

        window_mm = min(window_mm * 2.0, region_diag * 2.0)
        escalations += 1

    return path, window_mm, n_nodes, n_edges, escalations, board_scale_escalations


# ---------------------------------------------------------------------------
# Accept a routed net
# ---------------------------------------------------------------------------

def _accept_routed_net(
    net_name: str,
    start_xy: tuple[float, float],
    goal_xy: tuple[float, float],
    path: list,
    window_used: float,
    n_nodes: int,
    n_edges: int,
    escalations: int,
    build_time: float,
    inflate_track: float,
    sc_grid: SuperCellGrid,
    data: dict,
    geo: dict,
    routed_obstacles: list,
    board_bbox: tuple,
    is_board_scale: bool = False,
) -> tuple[GridlessRoute, dict]:
    clearance_mm = geo["clearance_mm"]
    track_mm = geo["track_mm"]
    waypoints = snap_waypoints(path)
    node_flag = n_nodes >= NODE_CEILING

    wx1 = max(min(start_xy[0], goal_xy[0]) - window_used, board_bbox[0])
    wy1 = max(min(start_xy[1], goal_xy[1]) - window_used, board_bbox[1])
    wx2 = min(max(start_xy[0], goal_xy[0]) + window_used, board_bbox[2])
    wy2 = min(max(start_xy[1], goal_xy[1]) + window_used, board_bbox[3])
    window_bbox = (wx1, wy1, wx2, wy2)

    free_space_check, _ = build_windowed_free_space(
        data["pads"], net_name, clearance_mm, track_mm,
        routed_obstacles, window_bbox,
    )
    all_legal, violations = validate_waypoints_shapely(waypoints, free_space_check)
    if violations:
        all_legal = False

    if len(waypoints) >= 2:
        ls = LineString(waypoints)
        shapely_obs = _snap(ls.buffer(inflate_track, cap_style=2, join_style=2))
    else:
        shapely_obs = Point(*waypoints[0]).buffer(inflate_track)

    supercells = sc_grid.supercells_for_path(waypoints)

    nr = GridlessRoute(
        net_name=net_name,
        waypoints=waypoints,
        shapely_obstacle=shapely_obs,
        supercells_used=supercells,
        n_nodes=n_nodes,
        n_edges=n_edges,
        build_time_s=build_time,
        window_mm=window_used,
        escalations=escalations,
    )
    result_dict = {
        "net": net_name,
        "status": "routed",
        "waypoints": len(waypoints),
        "all_legal": all_legal,
        "violations": violations,
        "nodes": n_nodes,
        "edges": n_edges,
        "window_mm": window_used,
        "escalations": escalations,
        "node_ceiling_flag": node_flag,
        "board_scale_escalation": is_board_scale,
    }
    return nr, result_dict


# ---------------------------------------------------------------------------
# Bounded rip-up loop (M2 core — run-2 with FIX-A, FIX-B, FIX-C)
# ---------------------------------------------------------------------------

def route_gridless_with_ripup_run2(
    board: Path,
    data: dict,
    net_groups: list[dict],
    geo: dict,
    region_bbox: tuple,
    window_mm_start: float = 4.0,
    history_factor: float = 3.0,
    ripup_factor: int = 8,
    geom_block_threshold: float = 0.5,
) -> dict:
    """Route the net-set with congestion history pricing and bounded rip-up.

    Run-2 changes vs run-1:
      FIX-A: Wide-window nets PARTICIPATE in rip-up (no pre-classification bypass).
      FIX-B: Midpoint-blocked edge pre-filter reduces visibility-graph build time.
      FIX-C: Geometry-blocked nets (need >50% board-diag window in pad-only) are
             flagged and reported; they do NOT trigger board-scale escalation.
    """
    clearance_mm = geo["clearance_mm"]
    track_mm = geo["track_mm"]
    inflate_track = track_mm + clearance_mm
    n_nets = max(len(net_groups), 1)

    sc_grid = make_supercell_grid(region_bbox)
    print(f"[spike2r2] Supercell grid: {sc_grid.ny}x{sc_grid.nx} = "
          f"{sc_grid.ny * sc_grid.nx} cells at {SUPERCELL_SIZE_MM}mm pitch", flush=True)

    def make_net_stub(grp: dict) -> Net:
        pads_fake = [(0, int(p["y"] * 100), int(p["x"] * 100)) for p in grp["pads"]]
        return Net(name=grp["net"], pads=pads_fake)

    net_stubs = [make_net_stub(g) for g in net_groups]
    ordered_stubs = order_nets(net_stubs)
    ordered_names = [n.name for n in ordered_stubs]
    net_pad_map = {g["net"]: g["pads"] for g in net_groups}

    # Board bbox
    all_pads = data["pads"]
    bx1 = min(p["x"] - p["hw"] for p in all_pads)
    by1 = min(p["y"] - p["hh"] for p in all_pads)
    bx2 = max(p["x"] + p["hw"] for p in all_pads)
    by2 = max(p["y"] + p["hh"] for p in all_pads)
    board_bbox = (bx1, by1, bx2, by2)
    board_diag = math.hypot(bx2 - bx1, by2 - by1)

    # FIX-A + FIX-C: Pre-classify ONLY for geometry-blocked detection.
    # Wide-window nets still participate in rip-up; only truly geometry-blocked
    # nets (pad-only window > 50% board diagonal) are quarantined.
    print("\n[spike2r2] Pre-classifying min_needed_window (FIX-A: no bypass for wide-window)...",
          flush=True)
    min_needed_window: dict[str, float] = {}
    geometry_blocked: set[str] = set()

    for grp in net_groups:
        nname = grp["net"]
        pads_g = grp["pads"]
        if len(pads_g) < 2:
            min_needed_window[nname] = window_mm_start
            continue
        sxy = (pads_g[0]["x"], pads_g[0]["y"])
        gxy = (pads_g[-1]["x"], pads_g[-1]["y"])
        win = window_mm_start
        found = False
        for _ in range(6):  # up to 6 doublings (64x window_mm_start)
            wx1_t = max(min(sxy[0], gxy[0]) - win, bx1)
            wy1_t = max(min(sxy[1], gxy[1]) - win, by1)
            wx2_t = min(max(sxy[0], gxy[0]) + win, bx2)
            wy2_t = min(max(sxy[1], gxy[1]) + win, by2)
            fs_t, obs_t = build_windowed_free_space(
                data["pads"], nname, clearance_mm, track_mm, [], (wx1_t, wy1_t, wx2_t, wy2_t)
            )
            from tracewise.route.gridless.search import route_window
            p_t, _, _ = route_window(fs_t, sxy, gxy, win, obs_t)
            if p_t is not None:
                found = True
                min_needed_window[nname] = win
                break
            win = min(win * 2.0, max(board_bbox[2] - board_bbox[0], board_bbox[3] - board_bbox[1]))
        if not found:
            min_needed_window[nname] = win

        # FIX-C: Check if geometry-blocked (needs > threshold * board_diag in pad-only)
        if _classify_geometry_blocked(nname, min_needed_window[nname], board_bbox, geom_block_threshold):
            geometry_blocked.add(nname)
            print(f"[spike2r2]   {nname!r}: GEOMETRY-BLOCKED "
                  f"(min_needed_window={min_needed_window[nname]:.1f}mm > "
                  f"{geom_block_threshold*100:.0f}% board_diag={board_diag:.1f}mm) "
                  f"-> M3 (needs B.Cu/via)", flush=True)
        elif min_needed_window[nname] > window_mm_start:
            # FIX-A: Wide-window but NOT geometry-blocked -> participates in rip-up
            print(f"[spike2r2]   {nname!r}: wide-window (min_needed={min_needed_window[nname]:.1f}mm) "
                  f"-> PARTICIPATES in rip-up (FIX-A)", flush=True)

    # State
    routed: dict[str, GridlessRoute] = {}
    budget = ripup_factor * n_nets
    attempts: dict[str, int] = {}
    ripup_count: dict[str, int] = {}

    # FIX-A: All non-geometry-blocked nets go into the rip-up queue
    queue: list[str] = [n for n in ordered_names if n not in geometry_blocked]

    final_results: dict[str, dict] = {}

    # Geometry-blocked nets are pre-failed (M3 required)
    for nname in geometry_blocked:
        final_results[nname] = {
            "net": nname,
            "status": "geometry_blocked",
            "reason": "geometry-blocked: needs B.Cu/via (M3)",
            "min_needed_window_mm": min_needed_window.get(nname, 0),
            "board_diag_mm": board_diag,
        }

    rip_up_events: list[dict] = []
    total_build_time = 0.0
    max_nodes = 0
    board_scale_esc_total = 0
    ripup_rounds = 0

    # FIX-D: Lower thrash guard from n_nets to 3.
    # A net that needs >3 rip-up attempts has NOT found a stable solution —
    # continuing just burns budget.  After 3 initiations, force it to escalate.
    MAX_RIPUP_PER_NET = 3

    # FIX-D: Track per-escalated-net which other nets caused its escalation.
    # An escalated net is protected ONLY from being ripped by the nets it cycled with.
    # Other nets (not involved in the cycle) can still rip it if they need to.
    # This prevents +3V3 from being stuck because GND escalated due to /GPIO2.
    # Structure: escalated_net -> set of nets that are NOT allowed to rip it
    escalated_protected_from: dict[str, set[str]] = {}  # net -> {nets that must not rip it}

    # Track which net was most recently ripped by each net (to record the cycle pair)
    last_victim_of: dict[str, str] = {}  # net_name -> victim_name from last rip-up

    print(f"\n[spike2r2] === Rip-up loop: {len(queue)} nets in queue "
          f"({len(geometry_blocked)} geometry-blocked, excluded), "
          f"budget={budget}, max_ripup_per_net={MAX_RIPUP_PER_NET} ===", flush=True)

    while queue and budget > 0:
        net_name = queue.pop(0)
        budget -= 1
        attempts[net_name] = attempts.get(net_name, 0) + 1

        pads = net_pad_map[net_name]
        if len(pads) < 2:
            final_results[net_name] = {"status": "routed_trivial", "net": net_name}
            continue

        pad_a = pads[0]
        pad_b = pads[-1]
        start_xy = (pad_a["x"], pad_a["y"])
        goal_xy = (pad_b["x"], pad_b["y"])

        print(f"\n[spike2r2] Net {net_name!r} (attempt {attempts[net_name]}, "
              f"ripups_initiated={ripup_count.get(net_name, 0)}): "
              f"({start_xy[0]:.3f},{start_xy[1]:.3f}) -> ({goal_xy[0]:.3f},{goal_xy[1]:.3f})  "
              f"budget_left={budget+1}", flush=True)

        routed_obstacles = [
            nr.shapely_obstacle for n, nr in routed.items() if n != net_name
        ]

        t_build_start = time.perf_counter()

        # FIX-A: Use min_needed_window as the STARTING window for rip-up attempts,
        # but still participate in rip-up (allow_window_escalation=False means we
        # WANT to negotiate space, not just widen unconditionally).
        effective_window = min_needed_window.get(net_name, window_mm_start)

        path, window_used, n_nodes, n_edges, escalations, bse = _route_one_net(
            net_name, start_xy, goal_xy, data, geo, region_bbox,
            routed_obstacles, sc_grid, history_factor, effective_window,
            board_bbox, allow_window_escalation=False,
        )
        board_scale_esc_total += bse
        build_time = time.perf_counter() - t_build_start
        total_build_time += build_time

        if path is not None:
            max_nodes = max(max_nodes, n_nodes)
            nr, result_dict = _accept_routed_net(
                net_name, start_xy, goal_xy, path, window_used, n_nodes, n_edges,
                escalations, build_time, inflate_track, sc_grid, data, geo,
                routed_obstacles, board_bbox,
            )
            routed[net_name] = nr
            result_dict["attempt"] = attempts[net_name]

            node_flag = n_nodes >= NODE_CEILING
            print(f"[spike2r2]   ROUTED  nodes={n_nodes}  edges={n_edges}  "
                  f"window={window_used:.1f}mm  escalations={escalations}  "
                  f"legal={result_dict['all_legal']}"
                  + ("  [NODE FLAG]" if node_flag else ""), flush=True)
            final_results[net_name] = result_dict
            continue

        # --- No path in bounded window ---
        print(f"[spike2r2]   No path in bounded window={effective_window:.1f}mm", flush=True)

        can_ripup = ripup_count.get(net_name, 0) <= MAX_RIPUP_PER_NET

        if can_ripup:
            # Classify: fixed obstacle vs rippable net
            classify_win = max(effective_window * 2, window_mm_start * 2)
            wx1_c = max(min(start_xy[0], goal_xy[0]) - classify_win, board_bbox[0])
            wy1_c = max(min(start_xy[1], goal_xy[1]) - classify_win, board_bbox[1])
            wx2_c = min(max(start_xy[0], goal_xy[0]) + classify_win, board_bbox[2])
            wy2_c = min(max(start_xy[1], goal_xy[1]) + classify_win, board_bbox[3])
            classify_window = (wx1_c, wy1_c, wx2_c, wy2_c)

            free_space_padonly, pad_obstacles_only = build_windowed_free_space(
                data["pads"], net_name, clearance_mm, track_mm, [], classify_window
            )
            # FIX-D: Each escalated net is protected only from the net(s)
            # it cycled with.  Build the set of nets that are fully protected
            # from being ripped BY net_name.
            nets_protected_from_this_requester = {
                escalated_net
                for escalated_net, protected_from_set in escalated_protected_from.items()
                if net_name in protected_from_set
            }
            is_rippable, victim_name = _classify_blockage(
                start_xy, goal_xy, routed, pad_obstacles_only,
                free_space_padonly, classify_window,
                protected_nets=nets_protected_from_this_requester,
            )
        else:
            print(f"[spike2r2]   THRASH GUARD: {net_name!r} has initiated "
                  f"{ripup_count.get(net_name,0)} rip-ups -> force escalation", flush=True)
            is_rippable, victim_name = False, None

        if is_rippable and victim_name is not None and victim_name in routed:
            # RIP UP the victim
            victim_nr = routed.pop(victim_name)
            print(f"[spike2r2]   -> RIP UP {victim_name!r} (corridor blocker; "
                  f"supercells={len(victim_nr.supercells_used)})", flush=True)

            sc_grid.deposit(victim_nr.supercells_used, 1.0)

            rip_up_events.append({
                "round": attempts[net_name],
                "failed_net": net_name,
                "victim_net": victim_name,
                "history_deposited": len(victim_nr.supercells_used),
                "budget_remaining": budget,
            })
            ripup_rounds += 1
            ripup_count[net_name] = ripup_count.get(net_name, 0) + 1
            last_victim_of[net_name] = victim_name

            final_results.pop(victim_name, None)

            # Requeue: failed net first, then victim
            queue.insert(0, victim_name)
            queue.insert(0, net_name)

        else:
            # Fixed obstacle blocking or thrash guard -> try window escalation
            reason_str = ("thrash-guard" if not can_ripup else
                          f"fixed-obstacle (rippable={is_rippable})")
            print(f"[spike2r2]   -> {reason_str}; trying window escalation...", flush=True)

            path2, window_used2, n_nodes2, n_edges2, esc2, bse2 = _route_one_net(
                net_name, start_xy, goal_xy, data, geo, region_bbox,
                routed_obstacles, sc_grid, history_factor,
                max(effective_window * 2, window_mm_start * 2),
                board_bbox, allow_window_escalation=True,
            )
            board_scale_esc_total += bse2

            board_w = board_bbox[2] - board_bbox[0]
            board_h = board_bbox[3] - board_bbox[1]
            board_size = max(board_w, board_h)
            is_board_scale = window_used2 >= board_size * 0.5
            if is_board_scale:
                # FIX-C: Only count as board-scale escalation if it's NOT already
                # classified as geometry-blocked.  Geometry-blocked nets are excluded
                # from the queue, so this case means a fixed-obstacle net that truly
                # needs the whole board.  Still flag it but it won't add to the total
                # that matters for pass criteria (geometry_blocked is separate).
                board_scale_esc_total += 1
                print(f"[spike2r2]   -> BOARD-SCALE escalation (window={window_used2:.1f}mm = "
                      f"{window_used2/board_size*100:.0f}% board) [FLAGGED]", flush=True)

            if path2 is not None:
                max_nodes = max(max_nodes, n_nodes2)
                nr2, result_dict2 = _accept_routed_net(
                    net_name, start_xy, goal_xy, path2, window_used2, n_nodes2, n_edges2,
                    esc2, build_time, inflate_track, sc_grid, data, geo,
                    routed_obstacles, board_bbox, is_board_scale=is_board_scale,
                )
                routed[net_name] = nr2
                result_dict2["attempt"] = attempts[net_name]
                # FIX-D: Mark this net as escalated -> protected ONLY from the net
                # it cycled with (the net that last ripped it / was last ripped by it).
                # Other nets that haven't been in a cycle with this net CAN still rip it.
                cycle_partner = last_victim_of.get(net_name)
                if cycle_partner is not None:
                    if net_name not in escalated_protected_from:
                        escalated_protected_from[net_name] = set()
                    escalated_protected_from[net_name].add(cycle_partner)
                    # Also protect from cycle_partner trying to rip this net again
                    # (mutual protection for the specific pair)
                    if cycle_partner not in escalated_protected_from:
                        escalated_protected_from[cycle_partner] = set()
                    escalated_protected_from[cycle_partner].add(net_name)
                    print(f"[spike2r2]   ROUTED via escalation  nodes={n_nodes2}  "
                          f"window={window_used2:.1f}mm"
                          + (" [BOARD_SCALE]" if is_board_scale else "")
                          + f" [PROTECTED: {net_name}<->{cycle_partner}]", flush=True)
                else:
                    print(f"[spike2r2]   ROUTED via escalation  nodes={n_nodes2}  "
                          f"window={window_used2:.1f}mm"
                          + (" [BOARD_SCALE]" if is_board_scale else ""), flush=True)
                final_results[net_name] = result_dict2
            else:
                print(f"[spike2r2]   FAILED ({reason_str}; no path even at escalated window)",
                      flush=True)
                final_results[net_name] = {
                    "net": net_name,
                    "status": "failed",
                    "reason": f"{reason_str}_no_path",
                    "nodes": n_nodes2,
                    "escalations": esc2,
                    "attempt": attempts[net_name],
                }

    # Budget exhausted for remaining in queue
    for net_name in queue:
        if net_name not in final_results:
            final_results[net_name] = {
                "net": net_name,
                "status": "failed",
                "reason": "budget_exhausted",
                "attempt": attempts.get(net_name, 0),
            }

    per_net_results = []
    for name in ordered_names:
        r = final_results.get(name, {"net": name, "status": "failed", "reason": "not_attempted"})
        per_net_results.append(r)

    all_waypoints = {name: nr.waypoints for name, nr in routed.items()}

    print("\n[spike2r2] Rip-up loop done.", flush=True)
    routed_count = sum(1 for r in per_net_results if r["status"] == "routed")
    geom_blocked_count = sum(1 for r in per_net_results if r["status"] == "geometry_blocked")
    failed_count = sum(1 for r in per_net_results
                       if r["status"] == "failed")
    print(f"[spike2r2]   Routed: {routed_count}/{len(per_net_results)}  "
          f"Geometry-blocked: {geom_blocked_count}  Failed (other): {failed_count}", flush=True)
    print(f"[spike2r2]   Rip-up events: {ripup_rounds}", flush=True)
    print(f"[spike2r2]   Board-scale escalations: {board_scale_esc_total}", flush=True)
    print(f"[spike2r2]   Max nodes in any window: {max_nodes}", flush=True)
    print(f"[spike2r2]   Escalated (protected) nets: {sorted(escalated_protected_from.keys())}", flush=True)
    print(f"[spike2r2]   Supercell history max: {sc_grid.history.max():.1f}  "
          f"mean: {sc_grid.history.mean():.3f}", flush=True)

    return {
        "per_net": per_net_results,
        "all_waypoints": all_waypoints,
        "max_nodes": max_nodes,
        "rip_up_events": rip_up_events,
        "ripup_rounds": ripup_rounds,
        "board_scale_esc_total": board_scale_esc_total,
        "geometry_blocked_nets": sorted(geometry_blocked),
        "supercell_history_max": float(sc_grid.history.max()),
        "supercell_history_mean": float(sc_grid.history.mean()),
        "total_build_time_s": total_build_time,
    }


# ---------------------------------------------------------------------------
# Grid baseline
# ---------------------------------------------------------------------------

def run_grid_baseline(
    board_master: Path,
    data: dict,
    net_groups: list[dict],
    geo: dict,
    region_bbox: tuple,
    out_dir: Path,
) -> dict:
    from tracewise.route.engine.kicad import build_problem as kicad_build_problem

    grid_dir = out_dir / "grid_run"
    grid_dir.mkdir(exist_ok=True)
    grid_board = grid_dir / board_master.name
    shutil.copy2(board_master, grid_board)

    net_names_set = {g["net"] for g in net_groups}

    t0 = time.perf_counter()
    grid, grid_nets, *_ = kicad_build_problem(data)
    subset_nets = [n for n in grid_nets if n.name in net_names_set]

    results = route_all(
        grid, subset_nets,
        via_cost=10.0,
        ripup_factor=8,
        history_factor=0.3,
    )
    grid_elapsed = time.perf_counter() - t0

    routed = sum(1 for r in results.values() if r.ok)
    failed = sum(1 for r in results.values() if not r.ok)
    print(f"[spike2r2] Grid baseline: routed={routed}/{len(results)}  "
          f"failed={failed}  time={grid_elapsed:.2f}s", flush=True)

    return {
        "routed": routed,
        "failed": failed,
        "elapsed_s": grid_elapsed,
        "results": results,
        "grid": grid,
        "board": grid_board,
    }


# ---------------------------------------------------------------------------
# Subprocess mode (for determinism gate)
# ---------------------------------------------------------------------------

def subprocess_emit_mode(args: list[str]) -> None:
    board_path = Path(args[0])
    routing_json = Path(args[1])
    geo_json_str = args[2]

    geo = json.loads(geo_json_str)
    with open(routing_json) as f:
        all_waypoints = json.load(f)

    for net_name, waypoints_raw in all_waypoints.items():
        waypoints = [tuple(pt) for pt in waypoints_raw]
        emit_net_segments(board_path, net_name, waypoints, geo["track_mm"])

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
# Fixed-order baseline
# ---------------------------------------------------------------------------

def run_fixed_order_baseline(
    board: Path,
    data: dict,
    net_groups: list[dict],
    geo: dict,
    region_bbox: tuple,
    window_mm_start: float = 4.0,
) -> dict:
    return spike1.route_gridless_pass(
        board, data, net_groups, geo, region_bbox, window_mm_start=window_mm_start
    )


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    print("=" * 70, flush=True)
    print("Spike-2 RUN-2: FAR gridless router — M2 CONGESTION NEGOTIATION + RIP-UP", flush=True)
    print("Fixes: FIX-A (no bypass), FIX-B (midpoint pre-filter), FIX-C (geom-blocked)", flush=True)
    print("=" * 70, flush=True)

    _spike_tmpbase = ROOT / ".spike2r2_tmp"
    _spike_tmpbase.mkdir(exist_ok=True)

    with tempfile.TemporaryDirectory(prefix="spike2r2_", dir=_spike_tmpbase) as tmp:
        out_dir = Path(tmp)

        # --- Step 1: Setup board ---
        board_master = setup_board(out_dir)
        print(f"[spike2r2] board: {board_master}", flush=True)

        data = extract_pads(board_master)
        geo = project_geometry(board_master)
        print(f"[spike2r2] geo: {geo}", flush=True)

        # --- Step 2: Region + net-set selection ---
        region_bbox, net_groups, final_margin = select_region_and_nets(
            data, region_margin_mm=3.0, min_nets=10
        )
        rx1, ry1, rx2, ry2 = region_bbox
        print(f"\n[spike2r2] REGION bbox: ({rx1:.3f},{ry1:.3f}) -> ({rx2:.3f},{ry2:.3f})  "
              f"margin={final_margin:.1f}mm", flush=True)
        print(f"[spike2r2] Net-set count: {len(net_groups)}", flush=True)

        net_names_set = {g["net"] for g in net_groups}

        net_stubs = [
            Net(name=g["net"],
                pads=[(0, int(p["y"] * 100), int(p["x"] * 100)) for p in g["pads"]])
            for g in net_groups
        ]
        ordered = order_nets(net_stubs)
        print("[spike2r2] Ordered net-set:", flush=True)
        for i, n in enumerate(ordered):
            print(f"[spike2r2]   {i+1:2d}. {n.name!r}", flush=True)

        # --- Step 3: Baseline DRC on stripped board ---
        baseline_drc = run_drc(board_master)
        baseline_summary = drc_region_summary(baseline_drc, net_names_set)
        print(f"\n[spike2r2] BASELINE DRC: unconnected={baseline_summary['total_unconnected']}  "
              f"errors={baseline_summary['total_errors']}", flush=True)

        # --- Step 4: Fixed-order baseline ---
        print("\n[spike2r2] === FIXED-ORDER BASELINE ===", flush=True)
        fixed_dir = out_dir / "fixed_order"
        shutil.copytree(out_dir, fixed_dir,
                        ignore=shutil.ignore_patterns("fixed_order", "ripup_run",
                                                       "ripup_run2", "grid_run", "*.drc.json"))
        fixed_board = next(fixed_dir.glob("*.kicad_pcb"))

        t_fixed_start = time.perf_counter()
        fixed_result = run_fixed_order_baseline(fixed_board, data, net_groups, geo, region_bbox)
        t_fixed_elapsed = time.perf_counter() - t_fixed_start

        fixed_per_net = fixed_result["per_net"]
        fixed_waypoints = fixed_result["all_waypoints"]
        fixed_routed_n = sum(1 for r in fixed_per_net if r["status"] == "routed")
        fixed_failed_n = sum(1 for r in fixed_per_net if r["status"] == "failed")
        fixed_failed_names = [r["net"] for r in fixed_per_net if r["status"] == "failed"]

        print(f"\n[spike2r2] Fixed-order: routed={fixed_routed_n}/{len(fixed_per_net)}  "
              f"failed={fixed_failed_n}  time={t_fixed_elapsed:.2f}s", flush=True)
        print(f"[spike2r2] Fixed-order FAILED: {fixed_failed_names}", flush=True)

        emit_all_routes(fixed_board, fixed_waypoints, geo)
        refill_zones(fixed_board)
        fixed_drc = run_drc(fixed_board)
        fixed_drc_summary = drc_region_summary(fixed_drc, set(fixed_waypoints.keys()))
        print(f"[spike2r2] Fixed-order DRC: unconnected={fixed_drc_summary['total_unconnected']}  "
              f"net_errors={fixed_drc_summary['net_errors']}", flush=True)

        # --- Step 5: Congestion + rip-up (Run-2 with fixes) ---
        print("\n[spike2r2] === CONGESTION+RIP-UP (Run-2: FIX-A/B/C) ===", flush=True)
        ripup_dir1 = out_dir / "ripup_run"
        shutil.copytree(out_dir, ripup_dir1,
                        ignore=shutil.ignore_patterns("fixed_order", "ripup_run",
                                                      "ripup_run2", "grid_run", "*.drc.json"))
        ripup_board1 = next(ripup_dir1.glob("*.kicad_pcb"))

        t_ripup_start = time.perf_counter()
        ripup_result1 = route_gridless_with_ripup_run2(
            ripup_board1, data, net_groups, geo, region_bbox,
            window_mm_start=4.0,
            history_factor=3.0,
            ripup_factor=8,
            geom_block_threshold=0.5,
        )
        t_ripup_elapsed1 = time.perf_counter() - t_ripup_start

        ripup_per_net1 = ripup_result1["per_net"]
        ripup_waypoints1 = ripup_result1["all_waypoints"]
        geom_blocked_nets = ripup_result1["geometry_blocked_nets"]
        ripup_routed_n1 = sum(1 for r in ripup_per_net1 if r["status"] == "routed")
        ripup_failed_n1 = sum(1 for r in ripup_per_net1 if r["status"] == "failed")
        ripup_geom_blocked_n1 = sum(1 for r in ripup_per_net1 if r["status"] == "geometry_blocked")

        print(f"\n[spike2r2] Rip-up Run-2: routed={ripup_routed_n1}/{len(ripup_per_net1)}  "
              f"geometry_blocked={ripup_geom_blocked_n1}  failed={ripup_failed_n1}  "
              f"time={t_ripup_elapsed1:.2f}s", flush=True)

        emit_all_routes(ripup_board1, ripup_waypoints1, geo)
        refill_zones(ripup_board1)
        ripup_drc1 = run_drc(ripup_board1)
        ripup_drc_summary1 = drc_region_summary(ripup_drc1, set(ripup_waypoints1.keys()))
        print(f"[spike2r2] Rip-up DRC: unconnected={ripup_drc_summary1['total_unconnected']}  "
              f"net_errors={ripup_drc_summary1['net_errors']}", flush=True)

        new_trace_errors1 = max(0, ripup_drc_summary1["net_errors"] -
                                baseline_summary["net_errors"])
        all_legal1 = (new_trace_errors1 == 0)

        # --- Step 6: Determinism gate --- Run 2 in same process ---
        print("\n[spike2r2] === DETERMINISM GATE (Run 2 same process) ===", flush=True)

        routing_json = out_dir / "ripup_routing_result.json"
        with open(routing_json, "w") as f:
            json.dump(
                {k: [list(pt) for pt in v] for k, v in ripup_waypoints1.items()},
                f
            )

        ripup_dir2 = out_dir / "ripup_run2"
        shutil.copytree(out_dir, ripup_dir2,
                        ignore=shutil.ignore_patterns("fixed_order", "ripup_run",
                                                      "ripup_run2", "grid_run", "*.drc.json"))
        ripup_board2 = next(ripup_dir2.glob("*.kicad_pcb"))

        t_run2 = time.perf_counter()
        ripup_result2 = route_gridless_with_ripup_run2(
            ripup_board2, data, net_groups, geo, region_bbox,
            window_mm_start=4.0,
            history_factor=3.0,
            ripup_factor=8,
            geom_block_threshold=0.5,
        )
        _t_run2_elapsed = time.perf_counter() - t_run2  # noqa: F841

        ripup_waypoints2 = ripup_result2["all_waypoints"]

        coords1 = extract_all_emitted_coords(ripup_board1, sorted(ripup_waypoints1.keys()))

        emit_all_routes(ripup_board2, ripup_waypoints2, geo)
        refill_zones(ripup_board2)
        coords2 = extract_all_emitted_coords(ripup_board2, sorted(ripup_waypoints2.keys()))

        determinism_ok = (coords1 == coords2)
        if determinism_ok:
            print("[spike2r2] DETERMINISM same-process: byte-identical", flush=True)
        else:
            lines1 = set(coords1.splitlines())
            lines2 = set(coords2.splitlines())
            diff_lines = lines1.symmetric_difference(lines2)
            print(f"[spike2r2] DETERMINISM FAIL: {len(diff_lines)} differing coord lines",
                  flush=True)
            for l in sorted(diff_lines)[:10]:
                print(f"[spike2r2]   {l}", flush=True)

        # --- Step 7: Subprocess determinism ---
        print("\n[spike2r2] === SUBPROCESS DETERMINISM ===", flush=True)
        subprocess_det_ok = False
        try:
            sub_board_dir = out_dir / "sub_run"
            shutil.copytree(out_dir, sub_board_dir,
                            ignore=shutil.ignore_patterns("fixed_order", "ripup_run",
                                                          "ripup_run2", "grid_run",
                                                          "sub_run", "*.drc.json"))
            sub_board = next(sub_board_dir.glob("*.kicad_pcb"))

            proc = subprocess.run(
                [
                    sys.executable, __file__,
                    "--emit-only",
                    str(sub_board),
                    str(routing_json),
                    json.dumps(geo),
                ],
                capture_output=True, text=True, timeout=120,
            )
            sub_coords = "\n".join(
                line[len("COORDS:"):].strip()
                for line in proc.stdout.splitlines()
                if line.startswith("COORDS:")
            )
            subprocess_det_ok = (sub_coords.strip() == coords1.strip())
            if subprocess_det_ok:
                print("[spike2r2] SUBPROCESS DETERMINISM: byte-identical", flush=True)
            else:
                lines_sub = set(sub_coords.splitlines())
                lines_r1 = set(coords1.splitlines())
                diff2 = lines_sub.symmetric_difference(lines_r1)
                print(f"[spike2r2] SUBPROCESS DETERMINISM FAIL: {len(diff2)} differing lines",
                      flush=True)
        except Exception as exc:
            subprocess_det_ok = False
            print(f"[spike2r2] SUBPROCESS DETERMINISM ERROR: {exc}", flush=True)

        determinism_ok = determinism_ok and subprocess_det_ok
        det_str = "byte-identical" if determinism_ok else "differ"

        # --- Step 8: Grid baseline ---
        print("\n[spike2r2] === GRID BASELINE ===", flush=True)
        try:
            t_grid = time.perf_counter()
            grid_data = run_grid_baseline(
                board_master, data, net_groups, geo, region_bbox, out_dir
            )
            _t_grid_elapsed = time.perf_counter() - t_grid  # noqa: F841
            grid_routed = grid_data["routed"]
            grid_failed = grid_data["failed"]
            grid_elapsed_s = grid_data["elapsed_s"]
        except Exception as exc:
            import traceback
            print(f"[spike2r2] Grid baseline error: {exc}", flush=True)
            traceback.print_exc()
            grid_routed = None
            grid_failed = None
            grid_elapsed_s = None

        # --- Step 9: Analysis ---
        budget_total = 8 * len(net_groups)
        ripup_converged = ripup_result1["ripup_rounds"] <= budget_total

        ripup_net_names_routed = {r["net"] for r in ripup_per_net1 if r["status"] == "routed"}
        fixed_net_names_routed = set(fixed_waypoints.keys())

        # Exclude geometry-blocked nets from regression analysis
        # (they're M3 findings, not M2 failures)
        geom_blocked_set = set(geom_blocked_nets)
        fixed_net_names_routed_excl_geom = fixed_net_names_routed - geom_blocked_set

        newly_routed = ripup_net_names_routed - fixed_net_names_routed
        newly_failed = fixed_net_names_routed_excl_geom - ripup_net_names_routed

        # Order-dependent failures: fixed-order routed them, rip-up failed them,
        # and they're NOT geometry-blocked
        order_dependent_failures = sorted(newly_failed)
        nets_relieved = len(newly_routed)
        print("\n[spike2r2] Relief analysis:", flush=True)
        print(f"  Newly routed (was failed in fixed-order): {sorted(newly_routed)}", flush=True)
        print(f"  Newly failed (was routed in fixed-order, NOT geom-blocked): "
              f"{sorted(newly_failed)}", flush=True)
        print(f"  Geometry-blocked (M3 required): {sorted(geom_blocked_set)}", flush=True)

        board_scale_escalations = ripup_result1["board_scale_esc_total"]

        # --- Step 10: GO/NO-GO ---
        # Regression: if we LOSE a net that fixed-order routes (excluding geometry-blocked ones),
        # that's an order-dependent failure = real M2 bug
        subset_regression = bool(order_dependent_failures)

        net_count_improves = ripup_routed_n1 > fixed_routed_n  # noqa: F841
        bounded_ok = (board_scale_escalations == 0)

        if (all_legal1 and ripup_converged and determinism_ok
                and not subset_regression and bounded_ok):
            go_no_go = "GO"
            go_reason = (
                "Negotiation correct: no regression, bounded (0 board-scale esc), "
                "deterministic, converges, DRC-clean. "
                f"Geometry-blocked nets correctly quarantined as M3: {sorted(geom_blocked_set)}."
            )
        elif all_legal1 and ripup_converged and determinism_ok:
            go_no_go = "GO-WITH-CAVEATS"
            caveats = []
            if subset_regression:
                caveats.append(f"ORDER-DEPENDENT regression: lost {order_dependent_failures} "
                               f"(fixed-order routed them, rip-up failed them, not geom-blocked)")
            if not bounded_ok:
                caveats.append(f"board-scale escalations={board_scale_escalations}")
            go_reason = "; ".join(caveats) if caveats else "minor caveats"
        else:
            go_no_go = "NO-GO"
            issues_list = []
            if not all_legal1:
                issues_list.append(f"legality failure: {new_trace_errors1} new DRC errors")
            if not ripup_converged:
                issues_list.append("rip-up did not converge within budget")
            if not determinism_ok:
                issues_list.append("determinism failure")
            go_reason = "; ".join(issues_list)

        runtime_ratio = (
            round(t_ripup_elapsed1 / grid_elapsed_s, 2)
            if grid_elapsed_s and grid_elapsed_s > 0
            else None
        )

        # --- Step 11: Report ---
        print("\n" + "=" * 70, flush=True)
        print("SPIKE-2 RUN-2 SUMMARY", flush=True)
        print("=" * 70, flush=True)
        print(f"  Region: ({rx1:.2f},{ry1:.2f}) -> ({rx2:.2f},{ry2:.2f})", flush=True)
        print(f"  Net-set: {len(net_groups)} nets", flush=True)
        print(f"  Fixed-order baseline: routed={fixed_routed_n}  failed={fixed_failed_n}  "
              f"failed_nets={fixed_failed_names}", flush=True)
        print(f"  Congestion+rip-up:    routed={ripup_routed_n1}  "
              f"geom_blocked={ripup_geom_blocked_n1}  failed={ripup_failed_n1}  "
              f"time={t_ripup_elapsed1:.2f}s", flush=True)
        print(f"  Nets relieved by rip-up (newly routed): {nets_relieved}", flush=True)
        print(f"  Geometry-blocked (M3): {sorted(geom_blocked_set)}", flush=True)
        print(f"  Order-dependent failures (regression): {order_dependent_failures}", flush=True)
        print(f"  New DRC errors: {new_trace_errors1}  all_legal={all_legal1}", flush=True)
        print(f"  Deterministic: {det_str}", flush=True)
        print(f"  Rip-up converged: {ripup_converged}  "
              f"(rounds={ripup_result1['ripup_rounds']}  budget={budget_total})", flush=True)
        print(f"  Board-scale escalations: {board_scale_escalations}", flush=True)
        grid_elapsed_str = f"{grid_elapsed_s:.2f}s" if grid_elapsed_s is not None else "N/A"
        print(f"  Grid baseline: routed={grid_routed}  failed={grid_failed}  "
              f"time={grid_elapsed_str}", flush=True)
        print(f"  Runtime ratio (ripup/grid): {runtime_ratio}", flush=True)
        print(f"  GO/NO-GO: {go_no_go}  ({go_reason})", flush=True)
        print("=" * 70, flush=True)

        # --- Structured result ---
        structured = {
            "status": "complete",
            "run": "spike2-run2",
            "summary": f"{go_no_go}: {go_reason}",
            "files_changed": [str(ROOT / "scripts/spike2_run2_congestion_ripup.py")],
            "files_read": [
                str(ROOT / "docs/design/FAR-gridless-router-arch.md"),
                str(ROOT / "src/tracewise/route/gridless/search.py"),
                str(ROOT / "src/tracewise/route/gridless/geom.py"),
                str(ROOT / "src/tracewise/route/engine/multi.py"),
                str(ROOT / "scripts/spike1_gridless_congested_region.py"),
                str(ROOT / "scripts/spike2_gridless_congestion_ripup.py"),
            ],
            "fixes_applied": [
                "FIX-A: Removed pre-classification BYPASS — wide-window nets participate in rip-up "
                "starting from min_needed_window; only geometry-blocked nets are quarantined",
                "FIX-B: Midpoint-blocked edge pre-filter in build_congestion_priced_visgraph — "
                "samples 3 interior points before expensive is_visible() call to reduce runtime",
                "FIX-C: Geometry-blocked classification — nets needing >50% board_diag window "
                "in pad-only free space are flagged M3 and excluded from queue (0 board-scale esc)",
                "FIX-D: Lowered thrash guard from n_nets=10 to 3 rip-up attempts; escalated nets "
                "marked as protected and excluded from future victim selection to break thrash cycles",
            ],
            "fixed_order_baseline": {
                "routed": fixed_routed_n,
                "failed": fixed_failed_n,
                "failed_nets": fixed_failed_names,
            },
            "negotiated_result": {
                "routed": ripup_routed_n1,
                "geometry_blocked": ripup_geom_blocked_n1,
                "failed": ripup_failed_n1,
                "new_drc_errors": new_trace_errors1,
            },
            "regression_check": (
                "NO REGRESSION" if not order_dependent_failures else
                f"REGRESSION: lost {order_dependent_failures} (order-dependent failures)"
            ),
            "geometry_blocked_nets": sorted(geom_blocked_set),
            "order_dependent_failures": order_dependent_failures,
            "runtime_s": round(t_ripup_elapsed1, 2),
            "grid_quality_runtime_s": round(grid_elapsed_s, 2) if grid_elapsed_s else None,
            "runtime_ratio": runtime_ratio,
            "board_scale_escalations": board_scale_escalations,
            "deterministic": det_str,
            "ripup_converged": {
                "converged": ripup_converged,
                "rounds_used": ripup_result1["ripup_rounds"],
                "budget": budget_total,
            },
            "go_no_go": f"{go_no_go}: {go_reason}",
            "issues": ([] if go_no_go == "GO" else
                       ([go_reason] if go_no_go != "GO" else [])),
            "assumptions": [
                "2-pin nets only (Phase 1; multi-pin MST deferred to M3)",
                "F.Cu single-layer only (no vias)",
                "history_factor=3.0, ripup_factor=8 (tunable)",
                "supercell_size=0.5mm (5x routing pitch)",
                "Geometry-blocked threshold: 50% of board diagonal in pad-only free space",
                "FIX-A: wide-window non-geom-blocked nets participate in rip-up from min_needed_window",
                "FIX-B: 3-sample midpoint pre-filter before is_visible() — conservative (false rejects impossible, only inside-obstacle kills edge)",
                "FIX-C: geometry-blocked nets quarantined as M3; NOT board-scale-escalated",
                "Victim selection: most-overlapping routed net in failure window corridor",
            ],
            "max_nodes": ripup_result1["max_nodes"],
            "rip_up_events": ripup_result1["rip_up_events"],
        }

        print("\n## Structured Result", flush=True)
        print("```json", flush=True)
        print(json.dumps(structured, indent=2), flush=True)
        print("```", flush=True)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "--emit-only":
        subprocess_emit_mode(sys.argv[2:])
    else:
        main()
