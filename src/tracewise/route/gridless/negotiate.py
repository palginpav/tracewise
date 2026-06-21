"""negotiate — M2 congestion-negotiation + bounded rip-up for the FAR gridless router.

Promotes the spike2_run2_congestion_ripup.py validated mechanism into the production
package.  The entry point is ``route_gridless_set``.

Algorithm (validated 10/10 mitayi GPIO fan-out, 0 DRC errors, deterministic, 2.69× grid):
  1. Pre-classify every net: find its ``min_needed_window`` in pad-only free space.
     Nets whose window exceeds ``geom_block_threshold × board_diagonal`` are quarantined
     as geometry-blocked (need B.Cu/vias; M3 work) and reported with status
     ``"geometry_blocked"``.
  2. Route the remaining net-set via a congestion-priced visibility-graph A*:
     edge cost = length × (1 + history_factor × mean_supercell_history).
     The 0.5 mm super-cell history field is shared across all routing attempts in this
     call (both initial routes and rip-up retries).
  3. Bounded rip-up: when a net fails in its bounded window,
     (a) classify whether the block is due to a rippable routed net or a fixed obstacle;
     (b) if rippable: rip the victim, deposit its supercells into history, requeue both;
     (c) if fixed / thrash-guard triggered: escalate window unconditionally.
     Budget = ``ripup_factor × n_nets`` total attempts.
  4. Cycle-breaking: per-pair escalation protection prevents thrash; nets that already
     escalated are protected only from the partner they cycled with.
  5. Return ``GridlessRouteResult``-like records keyed by net name as a
     ``dict[str, GridlessSetNetResult]`` with ``ok``, ``world_paths``, ``status``,
     ``reason``.

Design decisions:
  - Reuses the optimised ``search.py`` (STRtree batch, reflex pruning, OPT-2/3).
  - Deterministic: all data structures are sorted/seeded; A* uses integer 1 nm heap key.
  - Hard invariant: calling with empty net_set returns empty dict with no side-effects.
  - Geometry-blocked nets are reported failed with reason ``"geometry-blocked (M3)"``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from tracewise.route.gridless.geom import HAVE_SHAPELY, _require_shapely
from tracewise.route.gridless.realize import snap_waypoints

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

SUPERCELL_SIZE_MM: float = 0.5  # shared super-cell history field pitch (mm)
NODE_CEILING: int = 2000         # flag (not hard-stop) for large visibility graphs


# ---------------------------------------------------------------------------
# Result dataclass
# ---------------------------------------------------------------------------

@dataclass
class GridlessSetNetResult:
    """Result for a single net within ``route_gridless_set``.

    Mirrors ``GridlessRouteResult`` but adds ``status`` (``"routed"``,
    ``"geometry_blocked"``, ``"failed"``) and ``stats`` fields so callers can
    distinguish the M3-quarantine case from routing failures.
    """
    ok: bool
    world_paths: list[list[tuple]] = field(default_factory=list)
    world_vias: list[tuple[float, float]] = field(default_factory=list)
    status: str = ""          # "routed" | "geometry_blocked" | "failed"
    reason: str = ""
    stats: dict = field(default_factory=dict)


# ---------------------------------------------------------------------------
# Super-cell congestion lattice
# ---------------------------------------------------------------------------

class _SuperCellGrid:
    """0.5 mm super-cell history lattice for congestion pricing."""

    def __init__(self, x0: float, y0: float, nx: int, ny: int) -> None:
        if not HAVE_SHAPELY:
            return
        import numpy as np
        self.x0 = x0
        self.y0 = y0
        self.nx = nx
        self.ny = ny
        self.history: object = np.zeros((ny, nx), dtype=np.float64)

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
        import numpy as np
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
        mean_hist = float(np.mean([self.history[sy, sx]  # type: ignore[index]
                                   for sy, sx in cells])) if cells else 0.0
        return d * (1.0 + history_factor * mean_hist)

    def supercells_for_path(
        self, waypoints: list[tuple[float, float]]
    ) -> list[tuple[int, int]]:
        seen: set[tuple[int, int]] = set()
        result: list[tuple[int, int]] = []
        for (x0, y0_), (x1, y1) in zip(waypoints, waypoints[1:], strict=False):
            d = math.hypot(x1 - x0, y1 - y0_)
            step = SUPERCELL_SIZE_MM / 2.0
            n_steps = max(1, int(math.ceil(d / step))) if d > 1e-9 else 1
            for k in range(n_steps + 1):
                t = k / n_steps if n_steps > 0 else 0.0
                x = x0 + t * (x1 - x0)
                y = y0_ + t * (y1 - y0_)
                c = self.supercell_of(x, y)
                if c not in seen:
                    seen.add(c)
                    result.append(c)
        return result

    def deposit(self, supercells: list[tuple[int, int]], amount: float = 1.0) -> None:
        for sy, sx in supercells:
            self.history[sy, sx] += amount  # type: ignore[index]


def _make_supercell_grid(board_bbox: tuple[float, float, float, float]) -> _SuperCellGrid:
    bx1, by1, bx2, by2 = board_bbox
    nx = max(1, int(math.ceil((bx2 - bx1) / SUPERCELL_SIZE_MM)) + 2)
    ny = max(1, int(math.ceil((by2 - by1) / SUPERCELL_SIZE_MM)) + 2)
    x0 = bx1 - SUPERCELL_SIZE_MM
    y0 = by1 - SUPERCELL_SIZE_MM
    return _SuperCellGrid(x0=x0, y0=y0, nx=nx, ny=ny)


# ---------------------------------------------------------------------------
# Congestion-priced visibility graph  (mirrors spike2 build_congestion_priced_visgraph)
# ---------------------------------------------------------------------------

def _build_congestion_visgraph(
    free_space: object,
    start_xy: tuple[float, float],
    goal_xy: tuple[float, float],
    margin_mm: float,
    obstacle_polys: list,
    sc_grid: _SuperCellGrid,
    history_factor: float,
    use_reflex_pruning: bool = True,
) -> tuple[list[tuple[float, float]], dict[int, list[tuple[float, int]]], int, int]:
    """Build visibility graph with congestion-priced edge costs.

    Returns ``(all_nodes, adj, n_nodes, n_edges)``.
    """
    import numpy as np
    import shapely as _shapely
    from shapely import STRtree

    from tracewise.route.gridless.geom import get_component_containing
    from tracewise.route.gridless.search import (
        edge_blocked_by_obstacles_fast,
        is_visible_fast,
        obstacle_corners,
        reflex_obstacle_corners,
    )

    fs_component = get_component_containing(free_space, start_xy)

    if use_reflex_pruning:
        corners = reflex_obstacle_corners(fs_component, start_xy, goal_xy, margin_mm)
    else:
        corners = obstacle_corners(fs_component, start_xy, goal_xy, margin_mm)

    all_nodes: list[tuple[float, float]] = [start_xy, goal_xy] + corners
    n = len(all_nodes)

    obs_poly_arr = np.array(obstacle_polys) if obstacle_polys else None

    edges_ij: list[tuple[int, int]] = [
        (i, j) for i in range(n) for j in range(i + 1, n)
    ]
    n_edges_total = len(edges_ij)

    if obstacle_polys and obs_poly_arr is not None:
        strtree = STRtree(obstacle_polys)
        n_obs = len(obstacle_polys)

        xs = np.array([all_nodes[i][0] for i, j in edges_ij], dtype=np.float64)
        ys = np.array([all_nodes[i][1] for i, j in edges_ij], dtype=np.float64)
        xe = np.array([all_nodes[j][0] for i, j in edges_ij], dtype=np.float64)
        ye = np.array([all_nodes[j][1] for i, j in edges_ij], dtype=np.float64)

        minx = np.minimum(xs, xe) - 1e-6
        miny = np.minimum(ys, ye) - 1e-6
        maxx = np.maximum(xs, xe) + 1e-6
        maxy = np.maximum(ys, ye) + 1e-6

        edge_q_boxes = _shapely.box(minx, miny, maxx, maxy)
        input_idxs, tree_idxs = strtree.query(edge_q_boxes)

        valid_mask = tree_idxs < n_obs
        input_idxs_v = input_idxs[valid_mask]
        tree_idxs_v = tree_idxs[valid_mask]

        if len(input_idxs_v) > 0:
            sort_order = np.argsort(input_idxs_v, kind='stable')
            sorted_input = input_idxs_v[sort_order]
            sorted_tree = tree_idxs_v[sort_order]
            splits = np.searchsorted(sorted_input, np.arange(n_edges_total + 1))
            nearby_per_edge: list | None = [
                sorted_tree[splits[k]:splits[k + 1]] for k in range(n_edges_total)
            ]
        else:
            nearby_per_edge = [np.array([], dtype=np.int64)] * n_edges_total
    else:
        nearby_per_edge = None

    adj: dict[int, list[tuple[float, int]]] = {i: [] for i in range(n)}

    def _round1nm(v: float) -> int:
        return round(v * 1e6)

    for edge_idx, (i, j) in enumerate(edges_ij):
        u, v = all_nodes[i], all_nodes[j]

        if nearby_per_edge is not None:
            nearby = nearby_per_edge[edge_idx]
            if len(nearby) == 0:
                d_priced = sc_grid.edge_history_cost(u, v, history_factor)
                adj[i].append((d_priced, j))
                adj[j].append((d_priced, i))
                continue
            nearby_obs_arr = obs_poly_arr[nearby]  # type: ignore[index]
            if edge_blocked_by_obstacles_fast(u, v, nearby_obs_arr, n_samples=3):
                continue

        if is_visible_fast(u, v, fs_component):
            d_priced = sc_grid.edge_history_cost(u, v, history_factor)
            adj[i].append((d_priced, j))
            adj[j].append((d_priced, i))

    for i in range(n):
        adj[i] = sorted(adj[i], key=lambda e: (e[0], all_nodes[e[1]]))

    total_edges = sum(len(v) for v in adj.values()) // 2
    return all_nodes, adj, n, total_edges


def _astar_congestion(
    all_nodes: list[tuple[float, float]],
    adj: dict[int, list[tuple[float, int]]],
    goal_xy: tuple[float, float],
) -> list[tuple[float, float]] | None:
    """Deterministic A* with congestion-priced edge costs.  Index 0=start, 1=goal."""
    import heapq

    n = len(all_nodes)
    if n == 0:
        return None

    def _round1nm(v: float) -> int:
        return round(v * 1e6)

    def heuristic(ni: int) -> float:
        x, y = all_nodes[ni]
        return math.hypot(x - goal_xy[0], y - goal_xy[1])

    g_dist: dict[int, float] = {0: 0.0}
    prev: dict[int, int | None] = {0: None}
    seq = 0
    heap: list[tuple[int, int, int]] = [(_round1nm(heuristic(0)), seq, 0)]
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
# Single-net route attempt with congestion pricing
# ---------------------------------------------------------------------------

def _route_one_net_congestion(
    net_name: str,
    start_xy: tuple[float, float],
    goal_xy: tuple[float, float],
    pads: list[dict],
    geo: dict,
    routed_obstacles: list,
    sc_grid: _SuperCellGrid,
    history_factor: float,
    window_mm_start: float,
    board_bbox: tuple[float, float, float, float],
    allow_window_escalation: bool = True,
    board_outline: object | None = None,
    drill_obstacles: list | None = None,
    max_window_mm: float | None = None,
) -> tuple[list[tuple[float, float]] | None, float, int, int, int]:
    """Route net_name with congestion pricing.

    Returns ``(waypoints_or_None, window_mm_used, n_nodes, n_edges, escalations)``.

    ``max_window_mm`` caps how large the window can grow during escalation.
    Default ``None`` means original behaviour (escalate to board diagonal).
    """
    from shapely.geometry import Point

    from tracewise.route.gridless.geom import build_windowed_free_space

    clearance_mm = geo["clearance_mm"]
    track_mm = geo["track_mm"]
    bx1, by1, bx2, by2 = board_bbox
    board_size = max(bx2 - bx1, by2 - by1)
    _esc_ceil = min(max_window_mm, board_size) if max_window_mm is not None else board_size

    window_mm = window_mm_start
    path: list[tuple[float, float]] | None = None
    n_nodes = 0
    n_edges = 0
    escalations = 0

    while True:
        wx1 = max(min(start_xy[0], goal_xy[0]) - window_mm, bx1)
        wy1 = max(min(start_xy[1], goal_xy[1]) - window_mm, by1)
        wx2 = min(max(start_xy[0], goal_xy[0]) + window_mm, bx2)
        wy2 = min(max(start_xy[1], goal_xy[1]) + window_mm, by2)
        window_bbox = (wx1, wy1, wx2, wy2)

        free_space, obstacle_polys = build_windowed_free_space(
            pads, net_name, clearance_mm, track_mm,
            routed_obstacles, window_bbox,
            board_outline=board_outline,
            drill_obstacles=drill_obstacles,
        )

        # Check start/goal are in free space
        fs_buf = free_space.buffer(1e-5)
        if not fs_buf.contains(Point(*start_xy)) or not fs_buf.contains(Point(*goal_xy)):
            if allow_window_escalation and window_mm < _esc_ceil:
                window_mm = min(window_mm * 2.0, _esc_ceil)
                escalations += 1
                continue
            break

        for use_reflex in [True, False]:
            all_nodes, adj, n_nodes, n_edges = _build_congestion_visgraph(
                free_space, start_xy, goal_xy, window_mm, obstacle_polys,
                sc_grid, history_factor, use_reflex_pruning=use_reflex,
            )
            path = _astar_congestion(all_nodes, adj, goal_xy)
            if path is not None:
                break

        if path is not None:
            break

        if not allow_window_escalation:
            break

        current_window_size = max(wx2 - wx1, wy2 - wy1)
        if current_window_size >= _esc_ceil * 0.95:
            break

        window_mm = min(window_mm * 2.0, _esc_ceil * 2.0)
        escalations += 1

    return path, window_mm, n_nodes, n_edges, escalations


# ---------------------------------------------------------------------------
# Blockage classification
# ---------------------------------------------------------------------------

def _classify_blockage(
    start_xy: tuple[float, float],
    goal_xy: tuple[float, float],
    routed: dict[str, dict],  # net_name -> {"shapely_obstacle": ..., "supercells": ...}
    pads: list[dict],
    geo: dict,
    window_bbox: tuple[float, float, float, float],
    board_bbox: tuple[float, float, float, float],
    protected_nets: set[str],
) -> tuple[bool, str | None]:
    """Is the failure due to a rippable routed net?  Returns (is_rippable, victim_name)."""
    from shapely.geometry import LineString, box

    from tracewise.route.gridless.geom import build_windowed_free_space
    from tracewise.route.gridless.search import route_window

    if not routed:
        return False, None

    clearance_mm = geo["clearance_mm"]
    track_mm = geo["track_mm"]

    # Use a unique dummy net name so no own pads are carved out (all pads = obstacles).
    # This gives pad-only free space: if there's a path here, routed copper is blocking.
    classify_fs_padonly, classify_obs_padonly = build_windowed_free_space(
        pads, "__dummy_classify__", clearance_mm, track_mm, [], window_bbox
    )

    path_padonly, _, _ = route_window(
        classify_fs_padonly, start_xy, goal_xy,
        max(abs(goal_xy[0] - start_xy[0]), abs(goal_xy[1] - start_xy[1]), 2.0),
        classify_obs_padonly,
    )

    if path_padonly is None:
        return False, None

    # Pad-only has a path → routed copper is blocking
    sx, sy = start_xy
    gx, gy = goal_xy
    try:
        straight = LineString([(sx, sy), (gx, gy)])
        corridor = straight.buffer(1.5, cap_style=2)
    except Exception:  # noqa: BLE001
        wx1, wy1, wx2, wy2 = window_bbox
        corridor = box(wx1, wy1, wx2, wy2)

    best_victim: str | None = None
    best_overlap: float = -1.0
    for net_name, nr in routed.items():
        if net_name in protected_nets:
            continue
        obs = nr.get("shapely_obstacle")
        if obs is None:
            continue
        try:
            ov = obs.intersection(corridor).area
        except Exception:  # noqa: BLE001
            ov = 0.0
        if ov > best_overlap:
            best_overlap = ov
            best_victim = net_name

    if best_overlap < 1e-6:
        return False, None

    return True, best_victim


# ---------------------------------------------------------------------------
# Entry point: route_gridless_set
# ---------------------------------------------------------------------------

def route_gridless_set(
    net_set: list[dict],
    pads: list[dict],
    geo: dict,
    board_bbox: tuple[float, float, float, float],
    anchors: dict | None = None,
    history_factor: float = 3.0,
    ripup_factor: int = 8,
    window_mm_start: float = 4.0,
    geom_block_threshold: float = 0.5,
    board_outline: object | None = None,
    drill_obstacles: list | None = None,
    max_classify_window_mm: float | None = None,
    max_route_window_mm: float | None = None,
) -> dict[str, GridlessSetNetResult]:
    """Route a set of gridless nets with congestion negotiation and bounded rip-up.

    Parameters
    ----------
    net_set:
        List of dicts, each with keys:
          ``"net_name"`` (str): net name.
          ``"pad_a"`` (tuple[float, float]): start pad world-mm (x, y).
          ``"pad_b"`` (tuple[float, float]): goal pad world-mm (x, y).
    pads:
        All board pads from ``extract_pads`` (dicts with net, x, y, hw, hh, front).
    geo:
        Project geometry dict with ``track_mm``, ``clearance_mm``.
    board_bbox:
        ``(x1, y1, x2, y2)`` of the board boundary.
    anchors:
        Optional mapping ``(layer, iy, ix) -> (x_mm, y_mm)`` (ignored; present for
        interface compatibility with the engine caller).
    history_factor:
        Congestion-history pricing weight (default 3.0; validated in spike2 run-2).
    ripup_factor:
        Total budget = ``ripup_factor × n_nets`` routing attempts.
    window_mm_start:
        Initial visibility-graph window half-margin (mm).
    geom_block_threshold:
        Fraction of board diagonal beyond which a net is geometry-blocked (default 0.5).
    max_classify_window_mm:
        Hard cap on the pre-classification window escalation (mm).  When provided,
        the 6-doubling loop stops at this window size instead of the board diagonal.
        Critical for the ``gridless_first`` path on clean boards where large windows
        produce O(n²) visibility graphs (hundreds of corners from all pad obstacles).
        Default ``None`` = board diagonal (original behaviour, unchanged).
        Set to ~20-25 mm for the gridless-first pre-route to match the Probe-Order
        bounded-window strategy (fast path, <7 s/net).
    max_route_window_mm:
        Hard cap on the window escalation during routing (both the initial bounded
        attempt and the fallback escalation path).  When provided, the fallback
        ``allow_window_escalation=True`` call in the rip-up loop starts from at most
        this value and cannot exceed it.  Default ``None`` = board diagonal
        (original behaviour).  Set to ~25-30 mm for the ``gridless_first`` path
        to prevent board-wide visibility-graph blowup during the fallback pass.

    Returns
    -------
    ``dict[str, GridlessSetNetResult]`` keyed by net name.
    Geometry-blocked nets have ``ok=False``, ``status="geometry_blocked"``,
    ``reason="geometry-blocked (M3)"``.
    Other failed nets have ``ok=False``, ``status="failed"``.
    Successfully routed nets have ``ok=True``, ``status="routed"``,
    and ``world_paths`` populated.
    """
    _require_shapely()

    from tracewise.route.engine.multi import Net, order_nets

    if not net_set:
        return {}

    bx1, by1, bx2, by2 = board_bbox
    board_diag = math.hypot(bx2 - bx1, by2 - by1)
    clearance_mm = geo["clearance_mm"]
    track_mm = geo["track_mm"]
    inflate_track = track_mm + clearance_mm

    # Routing window cap: limits the fallback escalation to avoid O(n²) blowup.
    # None = original behaviour (escalates to board diagonal).
    _board_max_route = max(bx2 - bx1, by2 - by1)
    _route_cap = (
        min(max_route_window_mm, _board_max_route)
        if max_route_window_mm is not None
        else _board_max_route
    )

    sc_grid = _make_supercell_grid(board_bbox)

    # Build ordering stubs (reuse order_nets for power-first ordering)
    stubs = [
        Net(
            name=nd["net_name"],
            pads=[(0, int(nd["pad_a"][1] * 100), int(nd["pad_a"][0] * 100)),
                  (0, int(nd["pad_b"][1] * 100), int(nd["pad_b"][0] * 100))],
        )
        for nd in net_set
    ]
    ordered_names = [n.name for n in order_nets(stubs)]
    pad_map: dict[str, dict] = {nd["net_name"]: nd for nd in net_set}

    # ---------------------------------------------------------------------------
    # Pre-classification: find min_needed_window and detect geometry-blocked nets
    # ---------------------------------------------------------------------------
    from tracewise.route.gridless.geom import build_windowed_free_space
    from tracewise.route.gridless.search import route_window

    min_needed_window: dict[str, float] = {}
    geometry_blocked: set[str] = set()

    # Determine the ceiling for the classify window escalation.
    # max_classify_window_mm caps the 6-doubling loop to avoid O(n²) blowup
    # on large clean-board free spaces (gridless-first path).
    _board_max = max(bx2 - bx1, by2 - by1)
    _classify_cap = (
        min(max_classify_window_mm, _board_max)
        if max_classify_window_mm is not None
        else _board_max
    )

    for nd in net_set:
        nname = nd["net_name"]
        sxy = nd["pad_a"]
        gxy = nd["pad_b"]

        win = min(window_mm_start, _classify_cap)
        found = False
        prev_win = -1.0
        for _ in range(6):  # up to 6 doublings
            if win == prev_win:
                break  # already at cap; no point retrying
            prev_win = win
            wx1 = max(min(sxy[0], gxy[0]) - win, bx1)
            wy1 = max(min(sxy[1], gxy[1]) - win, by1)
            wx2 = min(max(sxy[0], gxy[0]) + win, bx2)
            wy2 = min(max(sxy[1], gxy[1]) + win, by2)
            fs_t, obs_t = build_windowed_free_space(
                pads, nname, clearance_mm, track_mm, [], (wx1, wy1, wx2, wy2),
                board_outline=board_outline, drill_obstacles=drill_obstacles,
            )
            p_t, _, _ = route_window(fs_t, sxy, gxy, win, obs_t)
            if p_t is not None:
                found = True
                min_needed_window[nname] = win
                break
            win = min(win * 2.0, _classify_cap)
        if not found:
            min_needed_window[nname] = win

        if min_needed_window[nname] > geom_block_threshold * board_diag:
            geometry_blocked.add(nname)

    # ---------------------------------------------------------------------------
    # Rip-up loop
    # ---------------------------------------------------------------------------
    results: dict[str, GridlessSetNetResult] = {}

    # Geometry-blocked: pre-failed
    for nname in geometry_blocked:
        results[nname] = GridlessSetNetResult(
            ok=False,
            status="geometry_blocked",
            reason="geometry-blocked (M3)",
            stats={"min_needed_window_mm": min_needed_window.get(nname, 0.0),
                   "board_diag_mm": board_diag},
        )

    queue: list[str] = [n for n in ordered_names if n not in geometry_blocked]
    n_nets = max(len(queue), 1)
    budget = ripup_factor * n_nets
    attempts: dict[str, int] = {}
    ripup_count: dict[str, int] = {}
    MAX_RIPUP_PER_NET = 3

    # routed: net_name -> {"world_paths": ..., "shapely_obstacle": ..., "supercells": ...}
    routed: dict[str, dict] = {}

    # Cycle protection: escalated_net -> set of nets that must not rip it
    escalated_protected_from: dict[str, set[str]] = {}
    last_victim_of: dict[str, str] = {}  # net -> last victim it ripped

    while queue and budget > 0:
        net_name = queue.pop(0)
        budget -= 1
        attempts[net_name] = attempts.get(net_name, 0) + 1

        nd = pad_map[net_name]
        start_xy = nd["pad_a"]
        goal_xy = nd["pad_b"]

        routed_obstacles = [
            nr["shapely_obstacle"]
            for n, nr in routed.items()
            if n != net_name and nr.get("shapely_obstacle") is not None
        ]

        effective_window = min(
            min_needed_window.get(net_name, window_mm_start),
            _route_cap,
        )

        path, window_used, n_nodes, n_edges, escalations = _route_one_net_congestion(
            net_name, start_xy, goal_xy, pads, geo,
            routed_obstacles, sc_grid, history_factor,
            effective_window, board_bbox,
            allow_window_escalation=False,  # bounded; escalate only when no rip-up
            board_outline=board_outline,
            drill_obstacles=drill_obstacles,
            max_window_mm=_route_cap,
        )

        if path is not None:
            waypoints = snap_waypoints(path)
            from shapely.geometry import LineString

            from tracewise.route.gridless.geom import snap
            if len(waypoints) >= 2:
                ls = LineString(waypoints)
                shapely_obs = snap(ls.buffer(inflate_track, cap_style=2, join_style=2))
            else:
                from shapely.geometry import Point
                shapely_obs = Point(*waypoints[0]).buffer(inflate_track)

            supercells = sc_grid.supercells_for_path(waypoints)
            routed[net_name] = {
                "world_paths": [waypoints],
                "shapely_obstacle": shapely_obs,
                "supercells": supercells,
            }
            results[net_name] = GridlessSetNetResult(
                ok=True,
                world_paths=[waypoints],
                status="routed",
                stats={"nodes": n_nodes, "edges": n_edges,
                       "window_mm": window_used, "escalations": escalations,
                       "attempt": attempts[net_name]},
            )
            continue

        # --- No path in bounded window ---
        can_ripup = ripup_count.get(net_name, 0) <= MAX_RIPUP_PER_NET

        if can_ripup:
            classify_win = max(effective_window * 2, window_mm_start * 2)
            wx1_c = max(min(start_xy[0], goal_xy[0]) - classify_win, bx1)
            wy1_c = max(min(start_xy[1], goal_xy[1]) - classify_win, by1)
            wx2_c = min(max(start_xy[0], goal_xy[0]) + classify_win, bx2)
            wy2_c = min(max(start_xy[1], goal_xy[1]) + classify_win, by2)
            classify_window = (wx1_c, wy1_c, wx2_c, wy2_c)

            # Nets this net is protected from ripping (cycle partners)
            nets_protected_from_this: set[str] = {
                escalated_net
                for escalated_net, prot_from in escalated_protected_from.items()
                if net_name in prot_from
            }

            is_rippable, victim_name = _classify_blockage(
                start_xy, goal_xy, routed, pads, geo,
                classify_window, board_bbox,
                protected_nets=nets_protected_from_this,
            )
        else:
            is_rippable, victim_name = False, None

        if is_rippable and victim_name is not None and victim_name in routed:
            victim_nr = routed.pop(victim_name)
            sc_grid.deposit(victim_nr["supercells"], 1.0)
            results.pop(victim_name, None)
            ripup_count[net_name] = ripup_count.get(net_name, 0) + 1
            last_victim_of[net_name] = victim_name
            queue.insert(0, victim_name)
            queue.insert(0, net_name)
        else:
            # Fixed obstacle / thrash guard → window escalation
            # Cap the escalation start to _route_cap to avoid O(n²) blowup.
            _fallback_win = min(
                max(effective_window * 2, window_mm_start * 2),
                _route_cap,
            )
            path2, window_used2, n_nodes2, n_edges2, esc2 = _route_one_net_congestion(
                net_name, start_xy, goal_xy, pads, geo,
                routed_obstacles, sc_grid, history_factor,
                _fallback_win,
                board_bbox,
                # allow_window_escalation only if _route_cap hasn't been hit yet
                allow_window_escalation=(_fallback_win < _route_cap),
                board_outline=board_outline,
                drill_obstacles=drill_obstacles,
                max_window_mm=_route_cap,
            )

            if path2 is not None:
                waypoints2 = snap_waypoints(path2)
                from shapely.geometry import LineString

                from tracewise.route.gridless.geom import snap
                if len(waypoints2) >= 2:
                    ls2 = LineString(waypoints2)
                    shapely_obs2 = snap(ls2.buffer(inflate_track, cap_style=2, join_style=2))
                else:
                    from shapely.geometry import Point
                    shapely_obs2 = Point(*waypoints2[0]).buffer(inflate_track)

                supercells2 = sc_grid.supercells_for_path(waypoints2)
                routed[net_name] = {
                    "world_paths": [waypoints2],
                    "shapely_obstacle": shapely_obs2,
                    "supercells": supercells2,
                }

                # Cycle-break: mutual protection for this pair
                cycle_partner = last_victim_of.get(net_name)
                if cycle_partner is not None:
                    escalated_protected_from.setdefault(net_name, set()).add(cycle_partner)
                    escalated_protected_from.setdefault(cycle_partner, set()).add(net_name)

                results[net_name] = GridlessSetNetResult(
                    ok=True,
                    world_paths=[waypoints2],
                    status="routed",
                    stats={"nodes": n_nodes2, "edges": n_edges2,
                           "window_mm": window_used2, "escalations": esc2,
                           "attempt": attempts[net_name]},
                )
            else:
                results[net_name] = GridlessSetNetResult(
                    ok=False,
                    status="failed",
                    reason="no_path_after_escalation",
                    stats={"nodes": n_nodes2, "edges": n_edges2,
                           "escalations": esc2, "attempt": attempts[net_name]},
                )

    for net_name in queue:
        if net_name not in results:
            results[net_name] = GridlessSetNetResult(
                ok=False,
                status="failed",
                reason="budget_exhausted",
                stats={"attempt": attempts.get(net_name, 0)},
            )

    return results
