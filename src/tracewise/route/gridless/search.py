"""search — windowed visibility graph + deterministic A* for the FAR gridless router.

Implements the core search substrate:
  1. Extract reflex obstacle corners (only convex obstacle corners = taut-string turning
     points) from the free-space interior rings.  Falls back to all corners if reflex-
     only finds no path.
  2. Build a visibility graph over those corners + start/goal, with STRtree pruning
     (locality mechanism #3) to skip the expensive ``is_visible`` check for edges
     whose bounding boxes have no nearby obstacles.
  3. Run deterministic A* with an integer 1 nm heap key and sorted-neighbour expansion
     so routes are byte-identical run-to-run on the same install.

Determinism guarantees:
  - Corners are collected into a ``set``, then emitted ``sorted((round(x,6), round(y,6)))``.
  - Node list is ``[start, goal, *sorted_corners]`` with fixed indices 0=start, 1=goal.
  - Adjacency is built in fixed ``for i in range(n): for j in range(i+1, n)`` order.
  - A* heap key: ``(round(f * 1e6), insertion_seq, node_idx)`` — never a float or
    Shapely object.
  - Neighbour expansion is ``sorted(adj[ni], key=lambda e: (e[0], all_nodes[e[1]]))``.
"""

from __future__ import annotations

import heapq
import math

from tracewise.route.gridless.geom import HAVE_SHAPELY, _require_shapely

if HAVE_SHAPELY:
    import numpy as np
    import shapely
    from shapely import STRtree
    from shapely.geometry import LineString


# ---------------------------------------------------------------------------
# Corner extraction
# ---------------------------------------------------------------------------


def reflex_obstacle_corners(
    fs_component: object,
    start_xy: tuple[float, float],
    goal_xy: tuple[float, float],
    margin_mm: float,
) -> list[tuple[float, float]]:
    """Extract REFLEX (convex-from-obstacle-side) corners within the margin window.

    In the free-space polygon, interior rings are holes whose boundaries are the
    inflated obstacle edges.  A vertex of an interior ring is a taut-string turning
    point iff the path bends *around* the convex corner of the obstacle — that is,
    where the ring turns LEFT for a CCW-oriented interior ring (positive cross product).

    Concave obstacle corners (right-turn vertices) are never optimal turning points
    and are skipped, pruning O(n) → O(r) nodes and the O(n²) → O(r²) edge count.

    Parameters
    ----------
    fs_component:
        A single Shapely Polygon (one connected component of the free space).
    start_xy, goal_xy:
        Route endpoints; only corners within their bounding box expanded by
        *margin_mm* are returned.
    margin_mm:
        Expansion margin applied to the start→goal bounding box.

    Returns
    -------
    Sorted list of ``(round(x, 6), round(y, 6))`` corner coordinates.
    """
    _require_shapely()
    sx, sy = start_xy
    gx, gy = goal_xy
    x_lo = min(sx, gx) - margin_mm
    x_hi = max(sx, gx) + margin_mm
    y_lo = min(sy, gy) - margin_mm
    y_hi = max(sy, gy) + margin_mm

    pts: set[tuple[float, float]] = set()

    for ring in fs_component.interiors:  # type: ignore[union-attr]
        coords = list(ring.coords)
        n = len(coords)
        # last coord == first in a closed ring; iterate unique vertices 0..n-2
        for i in range(n - 1):
            prev_pt = coords[(i - 1) % (n - 1)]
            curr_pt = coords[i]
            next_pt = coords[(i + 1) % (n - 1)]

            x, y = curr_pt
            if not (x_lo <= x <= x_hi and y_lo <= y <= y_hi):
                continue

            # Cross product of (curr - prev) × (next - curr)
            dx1 = curr_pt[0] - prev_pt[0]
            dy1 = curr_pt[1] - prev_pt[1]
            dx2 = next_pt[0] - curr_pt[0]
            dy2 = next_pt[1] - curr_pt[1]
            cross = dx1 * dy2 - dy1 * dx2

            # CCW ring: positive cross = left turn = convex obstacle corner = waypoint
            if cross > 1e-10:
                pts.add((round(x, 6), round(y, 6)))

    return sorted(pts)


def obstacle_corners(
    fs_component: object,
    start_xy: tuple[float, float],
    goal_xy: tuple[float, float],
    margin_mm: float,
) -> list[tuple[float, float]]:
    """Extract ALL interior-ring vertices within the margin window (fallback).

    Use when reflex-only corner extraction fails to find a path.  The full corner
    set is guaranteed to contain all turning points of any shortest path.
    """
    _require_shapely()
    sx, sy = start_xy
    gx, gy = goal_xy
    x_lo = min(sx, gx) - margin_mm
    x_hi = max(sx, gx) + margin_mm
    y_lo = min(sy, gy) - margin_mm
    y_hi = max(sy, gy) + margin_mm

    pts: set[tuple[float, float]] = set()
    for ring in fs_component.interiors:  # type: ignore[union-attr]
        for x, y in ring.coords:
            if x_lo <= x <= x_hi and y_lo <= y <= y_hi:
                pts.add((round(x, 6), round(y, 6)))
    return sorted(pts)


# ---------------------------------------------------------------------------
# Visibility check
# ---------------------------------------------------------------------------


def is_visible(
    u: tuple[float, float],
    v: tuple[float, float],
    fs_component: object,
) -> bool:
    """True iff segment *u*→*v* lies entirely within *fs_component*.

    Uses a thin buffer (1e-5 mm) around the segment to accept boundary-grazing
    paths — a track centreline that touches an inflated obstacle corner is legal.
    """
    _require_shapely()
    if math.hypot(u[0] - v[0], u[1] - v[1]) < 1e-9:
        return True
    seg = LineString([u, v])
    seg_buf = seg.buffer(1e-5, cap_style=2)
    diff = seg_buf.difference(fs_component)  # type: ignore[union-attr]
    return diff.is_empty or diff.area < 1e-8


def is_visible_fast(
    u: tuple[float, float],
    v: tuple[float, float],
    prepared_fs: object,
) -> bool:
    """Visibility check identical to ``is_visible`` but using a pre-prepared geometry.

    ``shapely.prepare()`` caches internal index structures on the free-space polygon
    so that the subsequent ``seg_buf.difference(prepared_fs)`` set operation can
    skip full re-indexing.  This gives a modest speedup (roughly 1.3×) over calling
    ``is_visible`` with an unprepared geometry.

    IMPORTANT: the semantics are byte-identical to ``is_visible`` — the same
    ``seg_buf.difference(fs_component).is_empty or area < 1e-8`` criterion is used.
    Do NOT replace this with a ``covers`` or ``within`` predicate: those lack the
    1e-8 area tolerance and reject valid boundary-grazing edges (changing routing
    results).

    *prepared_fs* must have been passed to ``shapely.prepare()`` before this call.
    (``shapely.prepare`` modifies the geometry in-place; the return value is None.)

    Parameters
    ----------
    u, v:
        Segment endpoints.
    prepared_fs:
        A Shapely Polygon that has been passed to ``shapely.prepare()`` in-place.

    Returns
    -------
    True iff the buffered segment lies within *prepared_fs* (with 1e-8 area tolerance).
    """
    _require_shapely()
    if math.hypot(u[0] - v[0], u[1] - v[1]) < 1e-9:
        return True
    seg = LineString([u, v])
    seg_buf = seg.buffer(1e-5, cap_style=2)
    diff = seg_buf.difference(prepared_fs)  # type: ignore[union-attr]
    return diff.is_empty or diff.area < 1e-8


def edge_blocked_by_obstacles_fast(
    u: tuple[float, float],
    v: tuple[float, float],
    nearby_obs_arr: object,
    n_samples: int = 3,
) -> bool:
    """Fast pre-filter: True iff any interior sample of segment u→v is inside an obstacle.

    Uses ``shapely.contains_xy`` with a pre-built numpy array of obstacle geometries,
    avoiding per-sample ``Point`` object creation and per-obstacle Python method calls.
    This is ~6× faster than the loop-based ``_edge_is_blocked_by_obstacle`` helper.

    Parameters
    ----------
    u, v:
        Segment endpoints.
    nearby_obs_arr:
        Numpy array of Shapely geometries (nearby obstacle polygons), obtained by
        ``np.array(nearby_obstacle_list)``.  Must be non-empty.
    n_samples:
        Number of evenly-spaced interior sample points (not endpoints).

    Returns
    -------
    True if any sample point is inside any obstacle (fast rejection).
    False if no sample is blocked (edge is a candidate for full visibility check).
    """
    _require_shapely()
    for k in range(1, n_samples + 1):
        t = k / (n_samples + 1)
        px = u[0] + t * (v[0] - u[0])
        py = u[1] + t * (v[1] - u[1])
        if np.any(shapely.contains_xy(nearby_obs_arr, px, py)):
            return True
    return False


# ---------------------------------------------------------------------------
# Visibility graph builder
# ---------------------------------------------------------------------------

def _round1nm(v: float) -> int:
    """Round *v* to integer 1 nm units for the A* heap key."""
    return round(v * 1e6)


def build_visibility_graph(
    free_space: object,
    start_xy: tuple[float, float],
    goal_xy: tuple[float, float],
    margin_mm: float,
    obstacle_polys: list,
    use_reflex_pruning: bool = True,
    use_all_rings: bool = False,
) -> tuple[list[tuple[float, float]], dict[int, list[tuple[float, int]]], int, int]:
    """Build a deterministic visibility graph over the free-space corner set.

    Parameters
    ----------
    free_space:
        Shapely geometry (Polygon or MultiPolygon) representing the routing window
        minus inflated obstacles.
    start_xy, goal_xy:
        Route endpoints (already inside *free_space* or close to its boundary).
    margin_mm:
        Window margin used to bound the corner-extraction search box.
    obstacle_polys:
        Actual inflated obstacle polygons for STRtree construction (FIX-3: must be
        the real polygon list, NOT the interior rings of *free_space*, which miss
        obstacles clipped to the window boundary).
    use_reflex_pruning:
        If True, use only reflex obstacle corners as candidate waypoints (the
        dominant O(n²)→O(r²) speedup, FIX-5b).  Caller falls back to False on
        failure.  Ignored when *use_all_rings* is True.
    use_all_rings:
        If True, collect BOTH interior-ring AND exterior-ring vertices as corner
        candidates.  Necessary for dense-pad (QFN) source pads whose relevant
        turning points live on the exterior ring of a fragmented free-space
        component.  When False (default), only interior-ring corners are used
        (original behaviour — byte-identical to pre-fanout-escape code).

    Returns
    -------
    ``(all_nodes, adj, n_nodes, n_edges)``
        *all_nodes*: list of (x, y) waypoints; index 0=start, 1=goal, 2..=corners.
        *adj*: dict mapping node index → list of ``(dist, neighbour_idx)`` edges,
               sorted by ``(dist, all_nodes[neighbour_idx])``.
        *n_nodes*: total node count.
        *n_edges*: undirected edge count.
    """
    _require_shapely()

    # Get the connected component containing start
    from tracewise.route.gridless.geom import exterior_ring_corners, get_component_containing
    fs_component = get_component_containing(free_space, start_xy)

    # Extract corners
    if use_all_rings:
        # Include both interior and exterior ring vertices — needed for QFN source pads
        # where the free-space component boundary IS the obstacle boundary.
        corners = exterior_ring_corners(fs_component, start_xy, goal_xy, margin_mm)
    elif use_reflex_pruning:
        corners = reflex_obstacle_corners(fs_component, start_xy, goal_xy, margin_mm)
    else:
        corners = obstacle_corners(fs_component, start_xy, goal_xy, margin_mm)

    all_nodes: list[tuple[float, float]] = [start_xy, goal_xy] + corners
    n = len(all_nodes)

    # OPT-2: pre-convert obstacle polys to numpy array for fast indexing in prefilter
    obs_poly_arr = np.array(obstacle_polys) if obstacle_polys else None

    # OPT-3 (batch STRtree): build all edge query boxes and issue a single
    # STRtree.query(array) call instead of O(n²) individual queries.
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
        edge_q_boxes = shapely.box(minx, miny, maxx, maxy)

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

    # Build adjacency — fixed i<j order for determinism
    adj: dict[int, list[tuple[float, int]]] = {i: [] for i in range(n)}
    for edge_idx, (i, j) in enumerate(edges_ij):
        u, v = all_nodes[i], all_nodes[j]

        if nearby_per_edge is not None:
            nearby = nearby_per_edge[edge_idx]
            if len(nearby) == 0:
                # No nearby obstacles → trivially visible
                d = math.hypot(v[0] - u[0], v[1] - u[1])
                adj[i].append((d, j))
                adj[j].append((d, i))
                continue

            # OPT-2: midpoint pre-filter using vectorized contains_xy
            nearby_obs_arr = obs_poly_arr[nearby]  # type: ignore[index]
            if edge_blocked_by_obstacles_fast(u, v, nearby_obs_arr, n_samples=3):
                continue  # fast reject

        # Full visibility check
        if is_visible(u, v, fs_component):
            d = math.hypot(v[0] - u[0], v[1] - u[1])
            adj[i].append((d, j))
            adj[j].append((d, i))

    # Sort each adjacency list deterministically
    for i in range(n):
        adj[i] = sorted(adj[i], key=lambda e: (e[0], all_nodes[e[1]]))

    total_edges = sum(len(v) for v in adj.values()) // 2
    return all_nodes, adj, n, total_edges


# ---------------------------------------------------------------------------
# A* search
# ---------------------------------------------------------------------------


def astar_visgraph(
    all_nodes: list[tuple[float, float]],
    adj: dict[int, list[tuple[float, int]]],
    goal_xy: tuple[float, float],
) -> list[tuple[float, float]] | None:
    """Deterministic A* over the pre-built visibility graph.

    Heap key: ``(round(f * 1e6), insertion_seq, node_idx)`` — integer 1 nm
    buckets ensure no float tie-break nondeterminism.  Neighbour expansion uses
    the pre-sorted adjacency list.

    Parameters
    ----------
    all_nodes:
        Node list; index 0 = start, index 1 = goal.
    adj:
        Pre-built adjacency dict from ``build_visibility_graph``.
    goal_xy:
        Goal coordinates (used for the Euclidean heuristic).

    Returns
    -------
    List of (x, y) waypoints from start to goal, or ``None`` if unreachable.
    """
    n = len(all_nodes)
    if n == 0:
        return None

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
            # Reconstruct path
            path: list[tuple[float, float]] = []
            cur: int | None = 1
            while cur is not None:
                path.append(all_nodes[cur])
                cur = prev[cur]
            path.reverse()
            return path

        g = g_dist[ni]
        for d, nj in adj[ni]:  # already sorted in build_visibility_graph
            ng = g + d
            if nj not in g_dist or ng < g_dist[nj]:
                g_dist[nj] = ng
                prev[nj] = ni
                seq += 1
                heapq.heappush(
                    heap,
                    (_round1nm(ng + heuristic(nj)), seq, nj),
                )

    return None


# ---------------------------------------------------------------------------
# Combined route-window helper
# ---------------------------------------------------------------------------


# ---------------------------------------------------------------------------
# 2-layer A* (M3: node = (x, y, layer))
# ---------------------------------------------------------------------------


def astar_2layer(
    all_nodes: list[tuple[float, float, int]],
    adj: dict[int, list[tuple[float, int]]],
    start_xy: tuple[float, float],
    goal_xy: tuple[float, float],
    start_layer: int,
    goal_layer: int,
    via_cost: float,
) -> list[tuple[float, float, int]] | None:
    """Deterministic 2-layer A* over a merged ``(x, y, layer)`` node space.

    Parameters
    ----------
    all_nodes:
        Flat list of ``(x, y, layer)`` nodes.  By convention index 0 is
        ``(start_xy[0], start_xy[1], start_layer)`` and index 1 is
        ``(goal_xy[0], goal_xy[1], goal_layer)`` (``build_two_layer_visgraph``
        contract).
    adj:
        Adjacency dict from ``build_two_layer_visgraph``.  In-plane edges carry
        Euclidean length; cross-layer via edges carry ``via_cost``.
    start_xy, goal_xy:
        Pad centres in world mm.
    start_layer:
        Layer of the starting pad (0=F.Cu, 1=B.Cu).
    goal_layer:
        Layer of the goal pad (-1 = reachable on any layer).
    via_cost:
        Cost of a cross-layer via transition (from ``project_geometry``).

    Returns
    -------
    List of ``(x, y, layer)`` waypoints from start to goal, or ``None`` if
    unreachable.  A layer change between consecutive waypoints at the same
    ``(x, y)`` position is a via insertion point.

    Determinism guarantees
    ----------------------
    Heap key: ``(round(f * 1e6), insertion_seq, node_idx)`` — integer 1 nm
    buckets + monotonic counter, no float or Shapely-object key.
    """
    n = len(all_nodes)
    if n == 0:
        return None

    # Find start and goal indices
    start_indices: list[int] = []
    goal_indices: list[int] = []
    for i, (x, y, lyr) in enumerate(all_nodes):
        if abs(x - start_xy[0]) < 1e-6 and abs(y - start_xy[1]) < 1e-6:
            if start_layer == -1 or lyr == start_layer:
                start_indices.append(i)
        if abs(x - goal_xy[0]) < 1e-6 and abs(y - goal_xy[1]) < 1e-6:
            if goal_layer == -1 or lyr == goal_layer:
                goal_indices.append(i)

    # Fallback: use indices 0 (start on start_layer) and 1 (goal on goal_layer)
    if not start_indices:
        start_indices = [0]
    if not goal_indices:
        # Try to find goal among all nodes with matching coords
        goal_indices = [
            i for i, (x, y, _) in enumerate(all_nodes)
            if abs(x - goal_xy[0]) < 1e-6 and abs(y - goal_xy[1]) < 1e-6
        ]
        if not goal_indices:
            goal_indices = [1]

    goal_set = set(goal_indices)

    def heuristic(ni: int) -> float:
        x, y, lyr = all_nodes[ni]
        h = math.hypot(x - goal_xy[0], y - goal_xy[1])
        # Add via_cost if a layer change is unavoidable (goal only on one layer
        # and we are on the other)
        if goal_layer != -1 and lyr != goal_layer:
            goal_on_current = any(all_nodes[gi][2] == lyr for gi in goal_indices)
            if not goal_on_current:
                h += via_cost
        return h

    def _round1nm_loc(v: float) -> int:
        return round(v * 1e6)

    g_dist: dict[int, float] = {}
    prev: dict[int, int | None] = {}
    seq = 0
    heap: list[tuple[int, int, int]] = []

    for si in start_indices:
        g_dist[si] = 0.0
        prev[si] = None
        seq += 1
        heapq.heappush(heap, (_round1nm_loc(heuristic(si)), seq, si))

    visited: set[int] = set()

    while heap:
        _, _, ni = heapq.heappop(heap)
        if ni in visited:
            continue
        visited.add(ni)

        if ni in goal_set:
            # Reconstruct path
            path: list[tuple[float, float, int]] = []
            cur: int | None = ni
            while cur is not None:
                path.append(all_nodes[cur])
                cur = prev.get(cur)
            path.reverse()
            return path

        g = g_dist[ni]
        for d, nj in adj.get(ni, []):  # already sorted
            ng = g + d
            if nj not in g_dist or ng < g_dist[nj]:
                g_dist[nj] = ng
                prev[nj] = ni
                seq += 1
                heapq.heappush(
                    heap,
                    (_round1nm_loc(ng + heuristic(nj)), seq, nj),
                )

    return None


# ---------------------------------------------------------------------------
# 2-layer visibility graph builder (M3)
# ---------------------------------------------------------------------------


def build_two_layer_visgraph(
    fs_F: object,
    fs_B: object,
    obs_F: list,
    obs_B: list,
    start_xy: tuple[float, float],
    goal_xy: tuple[float, float],
    legal_via_sites: list[tuple[float, float]],
    via_cost: float,
    margin_mm: float,
) -> tuple[
    list[tuple[float, float, int]],
    dict[int, list[tuple[float, int]]],
    int,
    int,
]:
    """Build a merged 2-layer visibility graph with via-transition edges.

    F.Cu free space after grid routing is typically a fragmented MultiPolygon
    (obstacles appear on exterior boundaries, not as interior rings). This
    function builds the F.Cu graph **per component** — only components that
    contain start, goal, or a legal via site get their corners as nodes; other
    components are invisible (unreachable) anyway.  This cuts edge-build ~93%
    vs a global cross-component graph (spike measured 964K→64K pairs).

    B.Cu is typically single-connected on mitayi and is built using the normal
    ``build_visibility_graph`` with legal via sites injected.

    Parameters
    ----------
    fs_F, fs_B:
        Per-layer free-space polygons from ``build_windowed_free_space``.
    obs_F, obs_B:
        Per-layer obstacle polygon lists (for STRtree pruning on B.Cu).
    start_xy, goal_xy:
        Route endpoints.
    legal_via_sites:
        Candidate via positions that passed the 3-predicate legality test.
    via_cost:
        Cost of a cross-layer transition.
    margin_mm:
        Window margin for the corner-extraction bounding box.

    Returns
    -------
    ``(all_nodes_3d, adj_3d, n_nodes, n_edges)``
        *all_nodes_3d*: list of ``(x, y, layer)``; F.Cu nodes come first,
        B.Cu nodes second.  Index 0 = F.Cu start, index 1 = F.Cu goal.
        *adj_3d*: global adjacency dict (in-plane + cross-layer via edges).
        *n_nodes*, *n_edges*: counts.
    """
    _require_shapely()

    if HAVE_SHAPELY:
        import shapely as _shapely  # type: ignore[import]

    via_site_set: dict[tuple[float, float], tuple[float, float]] = {
        (round(vx, 6), round(vy, 6)): (vx, vy)
        for vx, vy in legal_via_sites
    }

    sx, sy = start_xy
    gx, gy = goal_xy
    x_lo = min(sx, gx) - margin_mm
    x_hi = max(sx, gx) + margin_mm
    y_lo = min(sy, gy) - margin_mm
    y_hi = max(sy, gy) + margin_mm

    # --- Build F.Cu graph per-component ---
    if fs_F.geom_type == "MultiPolygon":  # type: ignore[union-attr]
        fcu_comps = list(fs_F.geoms)  # type: ignore[union-attr]
    else:
        fcu_comps = [fs_F]

    nodes_F: list[tuple[float, float]] = [start_xy, goal_xy]
    seen_F: dict[tuple[float, float], int] = {
        (round(sx, 6), round(sy, 6)): 0,
        (round(gx, 6), round(gy, 6)): 1,
    }

    def _add_node_F(key: tuple[float, float], xy: tuple[float, float]) -> int:
        if key not in seen_F:
            idx = len(nodes_F)
            nodes_F.append(xy)
            seen_F[key] = idx
            return idx
        return seen_F[key]

    # Find which components contain start, goal
    from shapely.geometry import Point as _SPoint  # type: ignore[import]

    start_comp: int | None = None
    goal_comp: int | None = None
    for ci, comp in enumerate(fcu_comps):
        if comp.contains(_SPoint(start_xy)):
            start_comp = ci
        if comp.contains(_SPoint(goal_xy)):
            goal_comp = ci

    # Relevant components: those containing start, goal, or a legal via site
    relevant_fcu: dict[int, object] = {}
    for ci, comp in enumerate(fcu_comps):
        via_here = any(comp.contains(_SPoint(vs)) for vs in legal_via_sites)
        if ci == start_comp or ci == goal_comp or via_here:
            relevant_fcu[ci] = comp

    # Build per-component node sets and adjacency
    comp_node_ranges: dict[int, list[int]] = {}

    for ci in sorted(relevant_fcu):
        comp = fcu_comps[ci]
        comp_idx_set: set[int] = set()
        if ci == start_comp:
            comp_idx_set.add(0)
        if ci == goal_comp:
            comp_idx_set.add(1)

        # Collect corners: interior rings (holes) + exterior ring (fragmented)
        corner_set: set[tuple[float, float]] = set()
        for ring in comp.interiors:  # type: ignore[union-attr]
            for x, y in ring.coords:
                if x_lo <= x <= x_hi and y_lo <= y <= y_hi:
                    corner_set.add((round(x, 6), round(y, 6)))
        for x, y in comp.exterior.coords:  # type: ignore[union-attr]
            if x_lo <= x <= x_hi and y_lo <= y <= y_hi:
                corner_set.add((round(x, 6), round(y, 6)))

        for key in sorted(corner_set):
            comp_idx_set.add(_add_node_F(key, key))

        # Via sites in this component
        for key, (vx, vy) in sorted(via_site_set.items()):
            if comp.contains(_SPoint(vx, vy)):
                comp_idx_set.add(_add_node_F(key, (vx, vy)))

        comp_node_ranges[ci] = sorted(comp_idx_set)

    n_F = len(nodes_F)
    adj_F: dict[int, list[tuple[float, int]]] = {i: [] for i in range(n_F)}

    for ci in sorted(relevant_fcu):
        comp = fcu_comps[ci]
        if HAVE_SHAPELY:
            _shapely.prepare(comp)
        indices = comp_node_ranges[ci]
        m = len(indices)
        for ii in range(m):
            for jj in range(ii + 1, m):
                i = indices[ii]
                j = indices[jj]
                u = nodes_F[i]
                v = nodes_F[j]
                if is_visible(u, v, comp):
                    d = math.hypot(v[0] - u[0], v[1] - u[1])
                    adj_F[i].append((d, j))
                    adj_F[j].append((d, i))

    for i in range(n_F):
        adj_F[i] = sorted(adj_F[i], key=lambda e: (e[0], nodes_F[e[1]]))

    # --- Build B.Cu graph using build_visibility_graph + inject via sites ---
    nodes_B_base, adj_B_base, _n_B_base, _ne_B_base = build_visibility_graph(
        free_space=fs_B,
        start_xy=start_xy,
        goal_xy=goal_xy,
        margin_mm=margin_mm,
        obstacle_polys=obs_B,
        use_reflex_pruning=True,
    )

    nodes_B = list(nodes_B_base)
    base_B_n = len(nodes_B)

    # Inject legal via sites into B.Cu that aren't already nodes
    seen_B: set[tuple[float, float]] = {
        (round(x, 6), round(y, 6)) for x, y in nodes_B
    }
    for key, (vx, vy) in sorted(via_site_set.items()):
        sp = _SPoint(vx, vy)
        if fs_B.contains(sp) and key not in seen_B:  # type: ignore[union-attr]
            seen_B.add(key)
            nodes_B.append((vx, vy))

    n_B = len(nodes_B)
    adj_B: dict[int, list[tuple[float, int]]] = {
        i: list(adj_B_base.get(i, [])) for i in range(base_B_n)
    }
    for i in range(base_B_n, n_B):
        adj_B[i] = []

    # Determine B.Cu components for visibility checks on injected via nodes
    if fs_B.geom_type == "MultiPolygon":  # type: ignore[union-attr]
        bcu_comps = list(fs_B.geoms)  # type: ignore[union-attr]
    else:
        bcu_comps = [fs_B]

    def _find_bcu_comp(pt: tuple[float, float]) -> object:
        sp = _SPoint(pt)
        for c in bcu_comps:
            if c.contains(sp):
                return c
        return fs_B

    # Connect injected via nodes to existing B.Cu nodes
    for vi in range(base_B_n, n_B):
        u = nodes_B[vi]
        bcomp = _find_bcu_comp(u)
        if HAVE_SHAPELY:
            _shapely.prepare(bcomp)
        for j in range(n_B):
            if j == vi:
                continue
            v = nodes_B[j]
            if is_visible(u, v, bcomp):
                d = math.hypot(v[0] - u[0], v[1] - u[1])
                adj_B[vi].append((d, j))
                adj_B.setdefault(j, []).append((d, vi))

    # De-dup B.Cu adjacency
    for i in range(n_B):
        seen_edges: dict[int, float] = {}
        for d, nb in adj_B[i]:
            if nb not in seen_edges or d < seen_edges[nb]:
                seen_edges[nb] = d
        adj_B[i] = sorted(
            [(d, nb) for nb, d in seen_edges.items()],
            key=lambda e: (e[0], nodes_B[e[1]]),
        )

    # --- Merge into unified 3D node space ---
    all_nodes_3d: list[tuple[float, float, int]] = []
    for x, y in nodes_F:
        all_nodes_3d.append((x, y, 0))
    for x, y in nodes_B:
        all_nodes_3d.append((x, y, 1))

    n_total = len(all_nodes_3d)
    adj_3d: dict[int, list[tuple[float, int]]] = {i: [] for i in range(n_total)}

    # In-plane F.Cu edges
    for i, edges in adj_F.items():
        adj_3d[i] = list(edges)

    # In-plane B.Cu edges (offset by n_F)
    for i, edges in adj_B.items():
        adj_3d[i + n_F] = [(d, j + n_F) for d, j in edges]

    # Cross-layer via edges
    node_xy_to_idx_F: dict[tuple[float, float], int] = {}
    for i, (x, y) in enumerate(nodes_F):
        node_xy_to_idx_F[(round(x, 6), round(y, 6))] = i

    node_xy_to_idx_B: dict[tuple[float, float], int] = {}
    for i, (x, y) in enumerate(nodes_B):
        node_xy_to_idx_B[(round(x, 6), round(y, 6))] = i

    for key in sorted(via_site_set.keys()):
        fi = node_xy_to_idx_F.get(key)
        bi = node_xy_to_idx_B.get(key)
        if fi is not None and bi is not None:
            adj_3d[fi].append((via_cost, bi + n_F))
            adj_3d[bi + n_F].append((via_cost, fi))

    # Sort adjacency lists deterministically
    for i in range(n_total):
        adj_3d[i] = sorted(adj_3d[i], key=lambda e: (e[0], all_nodes_3d[e[1]]))

    total_edges = sum(len(v) for v in adj_3d.values()) // 2
    return all_nodes_3d, adj_3d, n_total, total_edges


def route_window(
    free_space: object,
    start_xy: tuple[float, float],
    goal_xy: tuple[float, float],
    margin_mm: float,
    obstacle_polys: list,
) -> tuple[list[tuple[float, float]] | None, int, int]:
    """Build the visibility graph and run A* for one window attempt.

    Tries reflex-only corner pruning first; falls back to full corner set
    if reflex pruning produces no path.

    Returns
    -------
    ``(path, n_nodes, n_edges)``
        *path* is ``None`` if no path was found.
    """
    _require_shapely()

    all_nodes, adj, n_nodes, n_edges = build_visibility_graph(
        free_space,
        start_xy,
        goal_xy,
        margin_mm,
        obstacle_polys,
        use_reflex_pruning=True,
    )
    path = astar_visgraph(all_nodes, adj, goal_xy)

    if path is None:
        # Fallback: full corner set (no reflex pruning)
        all_nodes_fb, adj_fb, n_nodes_fb, n_edges_fb = build_visibility_graph(
            free_space,
            start_xy,
            goal_xy,
            margin_mm,
            obstacle_polys,
            use_reflex_pruning=False,
        )
        path_fb = astar_visgraph(all_nodes_fb, adj_fb, goal_xy)
        if path_fb is not None:
            return path_fb, n_nodes_fb, n_edges_fb

    return path, n_nodes, n_edges
