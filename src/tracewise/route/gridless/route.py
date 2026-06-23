"""route — top-level route_net_gridless entry point for the FAR gridless router.

This module provides the standalone ``route_net_gridless`` function and the
``GridlessRouteResult`` dataclass.  It is a self-contained Phase 1 deliverable;
the ``GridlessNetRoute`` adapter (``NetRoute`` IS-A) and engine wiring belong to
Phase 2.

M3-P2 adds ``route_net_multipin`` for K>2 nets: MST decomposition + sequential
sub-edge 2-pin routing + same-net-copper-as-goal + bounded windows.

Routing algorithm (per-net):
  1. Derive the routing window from the pad bounding box + *window_mm* margin,
     capped at the board bbox.
  2. Build the windowed free-space polygon (Minkowski-inflated obstacles,
     own-pad carve-out).
  3. Build the visibility graph over reflex obstacle corners, STRtree-pruned.
  4. Run deterministic A* (integer 1 nm heap key, sorted-neighbour expansion).
  5. If no path: double *window_mm* and retry (capped at board diagonal).
  6. Realize the path to a legal world-mm centreline (per-segment Shapely assert).
  7. Return a ``GridlessRouteResult``.

All sub-steps are byte-identical across runs on the same install.
"""

from __future__ import annotations

import heapq
import math
import time
from dataclasses import dataclass, field

from tracewise.route.gridless.geom import (
    _require_shapely,
    build_windowed_free_space,
    candidate_via_sites,
    is_legal_via,
)
from tracewise.route.gridless.realize import realize_centerline
from tracewise.route.gridless.search import astar_2layer, build_two_layer_visgraph, route_window

# ---------------------------------------------------------------------------
# Result dataclass
# ---------------------------------------------------------------------------


@dataclass
class GridlessRouteResult:
    """Lightweight result from ``route_net_gridless``.

    Phase 2 will wrap this in a ``GridlessNetRoute`` (IS-A ``NetRoute``) for engine
    integration.  This dataclass is self-contained for Phase 1 testing.

    Attributes
    ----------
    ok:
        True if the net was routed successfully.
    world_paths:
        List of path segments, each a list of waypoints.  For single-layer
        routes: ``[(x_mm, y_mm), ...]`` (2-tuples).  For 2-layer routes:
        ``[(x_mm, y_mm, layer), ...]`` (3-tuples with ``layer ∈ {0, 1}``).
        Normally one segment per 2-pin net; multi-pin MST is Phase 3.
    world_vias:
        Via centres for 2-layer routes: ``[(x_mm, y_mm), ...]``.
        Empty for single-layer routes.
    stats:
        Performance counters: ``nodes``, ``edges``, ``escalations``,
        ``build_time_s``, ``solve_time_s``, ``total_time_s``.
    reason:
        Human-readable failure reason when ``ok=False``.
    """

    ok: bool
    world_paths: list[list[tuple]] = field(default_factory=list)
    world_vias: list[tuple[float, float]] = field(default_factory=list)
    stats: dict = field(default_factory=dict)
    reason: str = ""


# ---------------------------------------------------------------------------
# Top-level routing function
# ---------------------------------------------------------------------------


def route_net_gridless(
    pad_a: tuple[float, float],
    pad_b: tuple[float, float],
    pads: list[dict],
    net_name: str,
    geo: dict,
    board_bbox: tuple[float, float, float, float],
    extra_obstacles: list | None = None,
    window_mm: float = 4.0,
    board_outline: object | None = None,
    drill_obstacles: list | None = None,
    drill_centers: list | None = None,
    allow_via: bool = True,
    max_window_mm: float | None = None,
    skip_full_corner_fallback: bool = False,
) -> GridlessRouteResult:
    """Route a 2-pin net gridlessly via visibility-graph A*.

    Parameters
    ----------
    pad_a, pad_b:
        Start and goal pad centres in world mm ``(x, y)``.
    pads:
        All board pads from ``extract_pads`` (dicts with keys ``net``, ``x``,
        ``y``, ``hw``, ``hh``, ``front``, ``back``).  Used to build the
        obstacle set.
    net_name:
        The net being routed (own-pad carve-out uses this to protect pad centres).
    geo:
        Project geometry dict with at least ``track_mm`` and ``clearance_mm``.
        M3: also reads ``via_mm``, ``via_drill_mm``, ``hole_clearance_mm``,
        ``hole_to_hole_mm`` for 2-layer routing.
    board_bbox:
        ``(x1, y1, x2, y2)`` of the board boundary; caps window escalation.
    extra_obstacles:
        Pre-inflated Shapely polygons of already-routed copper.  Each polygon
        should be the track centreline buffered by ``track_mm + clearance_mm``
        (FIX-6).  Pass ``None`` or ``[]`` when routing the first net.
    window_mm:
        Initial routing window half-margin around the pad bounding box.
    board_outline:
        Optional Shapely Polygon of the board interior (from Edge.Cuts).  When
        provided, free space is clipped to the outline shrunk inward by
        ``clearance_mm + track_mm / 2`` so track centrelines stay
        legal-distance from the board edge.
    drill_obstacles:
        Optional list of pre-inflated Shapely circle Polygons for drill holes
        (through-hole pads and vias), inflated by ``clearance_mm + track_mm/2``.
    drill_centers:
        Optional list of ``(cx, cy, drill_r)`` tuples for the hole-to-hole
        predicate (M3 via legality predicate 3).  When ``None``, falls back to
        ``[]`` (no drill-to-drill check — conservative but safe for routes with
        few nearby drills).
    allow_via:
        When ``True`` (default), attempt 2-layer F.Cu→via→B.Cu routing if the
        single-layer route fails.  Set to ``False`` to disable the 2-layer
        fallback (reproduces pre-M3 behaviour for specific callers).
    max_window_mm:
        Hard cap on the window escalation loop (mm).  When provided, the window
        never exceeds this value (even if the board diagonal is larger).  Use
        ~20-25 mm for the gridless-first fast path on clean boards to prevent
        O(n²) visibility-graph blowup.  ``None`` (default) = board diagonal
        (original behaviour, unchanged).
    skip_full_corner_fallback:
        When True, skip the expensive full-corner fallback in ``route_window``.
        The fallback extracts ALL obstacle polygon vertices (up to 6000+ on
        dense boards) and builds O(n²) numpy edge arrays → 4-5 GB RSS.  Set
        True when ``extra_obstacles`` is large (e.g. combined grid track
        obstacles ~1680 on mitayi).  Default ``False`` preserves existing
        behaviour for all other callers.

    Returns
    -------
    ``GridlessRouteResult`` with ``ok=True`` and ``world_paths`` on success, or
    ``ok=False`` with a ``reason`` string on failure.
    """
    _require_shapely()

    if extra_obstacles is None:
        extra_obstacles = []

    track_mm: float = geo["track_mm"]
    clearance_mm: float = geo["clearance_mm"]

    bx1, by1, bx2, by2 = board_bbox
    board_w = bx2 - bx1
    board_h = by2 - by1
    _board_max = max(board_w, board_h)
    max_window = min(max_window_mm, _board_max) if max_window_mm is not None else _board_max

    t_total_start = time.perf_counter()

    # Window escalation loop
    current_window = window_mm
    escalations = 0
    path: list[tuple[float, float]] | None = None
    free_space: object = None
    n_nodes = 0
    n_edges = 0
    build_time = 0.0
    solve_time = 0.0

    while True:
        # Derive routing window bbox (capped at board bbox)
        wx1 = max(min(pad_a[0], pad_b[0]) - current_window, bx1)
        wy1 = max(min(pad_a[1], pad_b[1]) - current_window, by1)
        wx2 = min(max(pad_a[0], pad_b[0]) + current_window, bx2)
        wy2 = min(max(pad_a[1], pad_b[1]) + current_window, by2)
        window_bbox = (wx1, wy1, wx2, wy2)

        t_build = time.perf_counter()
        free_space, obstacle_polys = build_windowed_free_space(
            pads,
            net_name,
            clearance_mm,
            track_mm,
            extra_obstacles,
            window_bbox,
            board_outline=board_outline,
            drill_obstacles=drill_obstacles,
        )
        build_time = time.perf_counter() - t_build

        t_solve = time.perf_counter()
        path, n_nodes, n_edges = route_window(
            free_space,
            pad_a,
            pad_b,
            current_window,
            obstacle_polys,
            skip_full_corner_fallback=skip_full_corner_fallback,
        )
        solve_time = time.perf_counter() - t_solve

        if path is not None:
            break

        # Escalate window
        if current_window >= max_window:
            # Already at board scale — give up
            break

        new_window = min(current_window * 2.0, max_window)
        if new_window == current_window:
            break  # can't grow further
        current_window = new_window
        escalations += 1

    total_time = time.perf_counter() - t_total_start

    stats = {
        "nodes": n_nodes,
        "edges": n_edges,
        "escalations": escalations,
        "build_time_s": round(build_time, 6),
        "solve_time_s": round(solve_time, 6),
        "total_time_s": round(total_time, 6),
    }

    if path is None or len(path) < 2:
        # Try 2-layer routing if allowed and geo carries via parameters
        if allow_via and geo.get("via_mm") and geo.get("via_drill_mm"):
            result_2l = _route_net_2layer(
                pad_a,
                pad_b,
                pads,
                net_name,
                geo,
                board_bbox,
                extra_obstacles or [],
                board_outline,
                drill_obstacles or [],
                drill_centers or [],
                window_mm,  # use original window_mm (not escalated) for 2L
                max_window_mm=max_window_mm,  # pass cap through to 2-layer search
            )
            if result_2l is not None:
                result_2l.stats.update(stats)
                result_2l.stats["single_layer_reason"] = (
                    f"no_path (window escalated to {current_window:.1f} mm, "
                    f"nodes={n_nodes}, edges={n_edges})"
                )
                return result_2l
        return GridlessRouteResult(
            ok=False,
            stats=stats,
            reason=f"no_path (window escalated to {current_window:.1f} mm, "
                   f"nodes={n_nodes}, edges={n_edges})",
        )

    # Realize centreline: snap + dedup + assert legal
    try:
        waypoints = realize_centerline(path, free_space)
    except (ValueError, RuntimeError) as exc:
        return GridlessRouteResult(
            ok=False,
            stats=stats,
            reason=f"realize_failed: {exc}",
        )

    return GridlessRouteResult(
        ok=True,
        world_paths=[waypoints],
        stats=stats,
    )


# ---------------------------------------------------------------------------
# 2-layer fallback helper (M3-P1)
# ---------------------------------------------------------------------------


def _route_net_2layer(
    pad_a: tuple[float, float],
    pad_b: tuple[float, float],
    pads: list[dict],
    net_name: str,
    geo: dict,
    board_bbox: tuple[float, float, float, float],
    extra_obstacles: list,
    board_outline: object | None,
    drill_obstacles: list,
    drill_centers: list,
    window_mm: float,
    max_window_mm: float | None = None,
) -> GridlessRouteResult | None:
    """Attempt a 2-layer F.Cu→via→B.Cu route for a geometry-blocked net.

    Returns a ``GridlessRouteResult`` with ``ok=True`` on success, or ``None``
    if no 2-layer path was found (caller will fall back to ``ok=False``).

    This helper is only called after the single-layer A* has exhausted all
    window escalations.  It does NOT escalate the window further; the full-board
    window (board diagonal) is used as the single attempt so via sites are
    drawn from the complete routing surface.

    ``max_window_mm`` caps the via-search window.  ``None`` (default) means
    full-board (the original behaviour).  Callers that know the board free space
    is large (e.g. the gridless-FIRST negotiate path on a clean board) can pass
    a smaller cap to avoid the O(n²) visibility-graph explosion.
    """
    _require_shapely()

    track_mm: float = geo["track_mm"]
    clearance_mm: float = geo["clearance_mm"]
    via_mm: float = geo["via_mm"]
    via_drill: float = geo["via_drill_mm"]
    via_cost: float = geo.get("via_cost_mm", 3.0)
    hole_clearance: float = geo.get("hole_clearance_mm", 0.25)
    hole_to_hole: float = geo.get("hole_to_hole_mm", 0.25)

    bx1, by1, bx2, by2 = board_bbox
    # Via-search window: default = full board (rescue path needs this because the
    # fragmented free space after grid routing means the local component may be
    # too small to find via sites).  max_window_mm lets callers cap this.
    board_max = max(abs(bx2 - bx1), abs(by2 - by1))
    window_mm_2l = board_max if max_window_mm is None else min(max_window_mm, board_max)
    wx1 = max(min(pad_a[0], pad_b[0]) - window_mm_2l, bx1)
    wy1 = max(min(pad_a[1], pad_b[1]) - window_mm_2l, by1)
    wx2 = min(max(pad_a[0], pad_b[0]) + window_mm_2l, bx2)
    wy2 = min(max(pad_a[1], pad_b[1]) + window_mm_2l, by2)
    window_bbox = (wx1, wy1, wx2, wy2)

    t0 = time.perf_counter()

    # Build free-space on both layers
    fs_F, obs_F = build_windowed_free_space(
        pads,
        net_name,
        clearance_mm,
        track_mm,
        extra_obstacles,
        window_bbox,
        board_outline=board_outline,
        drill_obstacles=drill_obstacles,
        layer=0,
    )
    fs_B, obs_B = build_windowed_free_space(
        pads,
        net_name,
        clearance_mm,
        track_mm,
        extra_obstacles,
        window_bbox,
        board_outline=board_outline,
        drill_obstacles=drill_obstacles,
        layer=1,
    )

    # Collect candidate via sites
    candidates = candidate_via_sites(
        fs_F,
        fs_B,
        pad_a,
        pad_b,
        window_bbox,
        via_mm,
        clearance_mm,
    )

    # Filter to legal via sites
    legal_sites: list[tuple[float, float]] = []
    for site in candidates:
        ok_via, _reason = is_legal_via(
            site,
            fs_F,
            fs_B,
            pads,
            net_name,
            via_mm,
            via_drill,
            clearance_mm,
            hole_clearance,
            hole_to_hole,
            drill_centers,
            window_bbox,
        )
        if ok_via:
            legal_sites.append(site)

    if not legal_sites:
        return None  # no viable via site → can't 2-layer route

    # Build 2-layer visibility graph
    all_nodes, adj, n_nodes, n_edges = build_two_layer_visgraph(
        fs_F,
        fs_B,
        obs_F,
        obs_B,
        pad_a,
        pad_b,
        legal_sites,
        via_cost,
        window_mm,
    )

    # Run 2-layer A*
    path_3d = astar_2layer(
        all_nodes,
        adj,
        pad_a,
        pad_b,
        start_layer=0,    # F.Cu start
        goal_layer=-1,    # any layer accepted at goal
        via_cost=via_cost,
    )

    build_and_solve_s = round(time.perf_counter() - t0, 6)

    if path_3d is None or len(path_3d) < 2:
        return None

    # Use path_3d directly after deduplication of consecutive identical waypoints.
    # realize_centerline expects single-layer 2-tuples; skip it for 3-tuple paths.
    waypoints_3d: list[tuple[float, float, int]] = []
    prev_xy3: tuple[float, float, int] | None = None
    for wp in path_3d:
        if wp != prev_xy3:
            waypoints_3d.append(wp)
            prev_xy3 = wp

    # Extract via centres: consecutive waypoints with same (x,y) but different layer
    via_centers: list[tuple[float, float]] = []
    for i in range(len(waypoints_3d) - 1):
        ax, ay, al = waypoints_3d[i]
        bx, by, bl = waypoints_3d[i + 1]
        if abs(ax - bx) < 1e-7 and abs(ay - by) < 1e-7 and al != bl:
            via_centers.append((ax, ay))

    stats_2l = {
        "nodes": n_nodes,
        "edges": n_edges,
        "via_candidates": len(candidates),
        "via_legal": len(legal_sites),
        "vias_placed": len(via_centers),
        "build_and_solve_s": build_and_solve_s,
    }

    return GridlessRouteResult(
        ok=True,
        world_paths=[waypoints_3d],
        world_vias=via_centers,
        stats=stats_2l,
    )


# ---------------------------------------------------------------------------
# Fanout-escape route (QFN dense-component escape via strategy)
# ---------------------------------------------------------------------------


def route_net_fanout_escape(
    source_xy: tuple[float, float],
    component_cx: float,
    component_cy: float,
    ring_radius: float,
    pads: list[dict],
    net_name: str,
    geo: dict,
    board_bbox: tuple[float, float, float, float],
    extra_obstacles: list | None = None,
    board_outline: object | None = None,
    drill_obstacles: list | None = None,
    drill_centers: list | None = None,
    window_mm: float = 4.0,
    max_window_mm: float | None = None,
    max_bcu_window_mm: float | None = None,
    dest_xy: tuple[float, float] | None = None,
    bcu_extra_obstacles: list | None = None,
    fcu_stub_extra_obstacles: list | None = None,
) -> GridlessRouteResult:
    """Route a QFN pad out of its dense ring via a guided escape via.

    Two modes controlled by ``dest_xy``:

    **Stub-only mode** (``dest_xy=None``, default — for multi-pin MST integration):

    1. Place a legal escape via on the ray from the component centroid through
       the source pad, just outside the pad ring.
    2. Route an F.Cu stub from ``source_xy`` to the escape via using the
       ``all_rings`` corner mode (exterior + interior ring corners).

    The caller injects the escape via as a virtual through-hole pad and runs
    the MST from the non-QFN pads to it on F.Cu.  No long B.Cu run is placed,
    so the grid router has unobstructed access to both layers.

    **Full two-layer mode** (``dest_xy`` provided — for 2-pin geometry-blocked nets):

    Same steps 1-2 as above, then:

    3. Route a B.Cu run from the escape via to ``dest_xy``.

    Use this mode only when ``dest_xy`` is a through-hole pad (``back=True``)
    that can accept a B.Cu termination.

    Parameters
    ----------
    source_xy:
        Source pad centre (QFN SMD F.Cu pad).
    dest_xy:
        Destination pad centre for 2-pin mode.  ``None`` → stub-only mode.
    component_cx, component_cy:
        Component centroid.
    ring_radius:
        Mean distance from centroid to pad centres.
    pads:
        All board pads from ``extract_pads``.
    net_name:
        Net name (own-pad carve-out).
    geo:
        Project geometry dict with ``track_mm``, ``clearance_mm``, ``via_mm``,
        ``via_drill_mm``, ``hole_clearance_mm``, ``hole_to_hole_mm``.
    board_bbox:
        ``(x1, y1, x2, y2)`` board bounding box.
    extra_obstacles:
        Pre-inflated Shapely obstacle polygons for F.Cu stub routing.
        Should contain only F.Cu obstacles so the layer-unaware 2D Shapely
        free-space does not block the F.Cu path with B.Cu-only copper.
    bcu_extra_obstacles:
        Pre-inflated Shapely obstacle polygons for B.Cu run routing.
        When provided, used instead of ``extra_obstacles`` for the B.Cu
        free-space computation.  Should contain all-layer copper (including
        B.Cu routes from previously-escaped nets) to prevent B.Cu crossing.
        When ``None``, falls back to ``extra_obstacles`` (legacy behaviour).
    board_outline:
        Optional board outline polygon.
    drill_obstacles:
        Pre-inflated drill hole obstacle circles.
    drill_centers:
        ``(cx, cy, drill_r)`` tuples for hole-to-hole predicate.
    window_mm:
        Base routing window half-margin (mm).
    max_window_mm:
        Hard cap on window escalation (applies to via-placement and F.Cu stub
        windows).  Does NOT cap the B.Cu run window — use ``max_bcu_window_mm``
        for that.
    max_bcu_window_mm:
        Hard cap on the B.Cu run window margin.  When ``None``, falls back to
        ``max_window_mm``.  Set this to a small value (e.g. 6–10 mm) to avoid
        O(n²) Shapely ``unary_union`` on boards with many B.Cu track obstacles.
        The B.Cu window is ``bcu_margin_max`` on each side of the via→dest
        bounding box; with 814 B.Cu segments on the mitayi board a 40 mm cap
        encompasses most of the board → 18 GB RSS.  8 mm keeps the window
        tight while still allowing Manhattan direct/L-shaped paths.

    Returns
    -------
    ``GridlessRouteResult`` with F.Cu 3-tuple ``world_paths`` (layer=0) and
    ``world_vias`` containing the single escape via, or ``ok=False`` on failure.
    The ``stats`` dict includes ``escape_via`` for caller use as a virtual pad.
    """
    _require_shapely()
    import math as _math
    import time as _time

    from tracewise.route.gridless.geom import (
        build_windowed_free_space,
        guided_escape_via,
    )
    from tracewise.route.gridless.search import astar_visgraph, build_visibility_graph

    if extra_obstacles is None:
        extra_obstacles = []
    if drill_obstacles is None:
        drill_obstacles = []
    if drill_centers is None:
        drill_centers = []

    track_mm: float = geo["track_mm"]
    clearance_mm: float = geo["clearance_mm"]
    via_mm: float = geo.get("via_mm", 0.4)
    via_drill: float = geo.get("via_drill_mm", 0.2)
    hole_clearance: float = geo.get("hole_clearance_mm", 0.25)
    hole_to_hole: float = geo.get("hole_to_hole_mm", 0.25)

    bx1, by1, bx2, by2 = board_bbox
    t0 = _time.perf_counter()

    # --- Step 1: Build via-placement window and per-layer free spaces ---
    via_window_margin = max(ring_radius + 3.0, 8.0)
    if max_window_mm is not None:
        via_window_margin = min(via_window_margin, max_window_mm)
    vwx1 = max(component_cx - via_window_margin, bx1)
    vwy1 = max(component_cy - via_window_margin, by1)
    vwx2 = min(component_cx + via_window_margin, bx2)
    vwy2 = min(component_cy + via_window_margin, by2)
    via_window = (vwx1, vwy1, vwx2, vwy2)

    fs_F_via, _obs_F_via = build_windowed_free_space(
        pads, net_name, clearance_mm, track_mm,
        extra_obstacles, via_window,
        board_outline=board_outline, drill_obstacles=drill_obstacles, layer=0,
    )
    # B.Cu via placement: use bcu_extra_obstacles when provided so that F.Cu
    # track copper (in extra_obstacles) is NOT projected as a B.Cu obstacle,
    # which would block the escape-via placement near the QFN ring.
    _bcu_extra_via = (
        bcu_extra_obstacles if bcu_extra_obstacles is not None else extra_obstacles
    )
    fs_B_via, _obs_B_via = build_windowed_free_space(
        pads, net_name, clearance_mm, track_mm,
        _bcu_extra_via, via_window,
        board_outline=board_outline, drill_obstacles=drill_obstacles, layer=1,
    )

    # --- Step 2: Guided escape-via placement ---
    escape_via = guided_escape_via(
        source_pad_xy=source_xy,
        component_cx=component_cx,
        component_cy=component_cy,
        ring_radius=ring_radius,
        fs_F=fs_F_via,
        fs_B=fs_B_via,
        pads=pads,
        net_name=net_name,
        via_mm=via_mm,
        via_drill=via_drill,
        clearance_mm=clearance_mm,
        hole_clearance=hole_clearance,
        hole_to_hole=hole_to_hole,
        drill_centers=drill_centers,
        window_bbox=via_window,
    )

    if escape_via is None:
        return GridlessRouteResult(
            ok=False,
            reason="fanout_escape: no_legal_via_near_ring",
            stats={"total_time_s": round(_time.perf_counter() - t0, 6)},
        )

    vx, vy = escape_via

    # --- Step 3: Route F.Cu stub (source_xy → escape_via) ---
    stub_dist = _math.hypot(vx - source_xy[0], vy - source_xy[1])
    # Use a generous window so the visibility-graph can find paths around
    # dense F.Cu grid copper near the QFN pad ring.  The stub is short
    # (typically 1–3 mm) so a 6 mm margin covers several pad-ring radii.
    fcu_margin = max(stub_dist + 5.0, 6.0)
    if max_window_mm is not None:
        fcu_margin = min(fcu_margin, max_window_mm)
    fcu_wx1 = max(min(source_xy[0], vx) - fcu_margin, bx1)
    fcu_wy1 = max(min(source_xy[1], vy) - fcu_margin, by1)
    fcu_wx2 = min(max(source_xy[0], vx) + fcu_margin, bx2)
    fcu_wy2 = min(max(source_xy[1], vy) + fcu_margin, by2)
    fcu_window = (fcu_wx1, fcu_wy1, fcu_wx2, fcu_wy2)

    # F.Cu stub free space: use fcu_stub_extra_obstacles when provided.
    # fcu_stub_extra_obstacles contains F.Cu track copper so the stub avoids
    # grid tracks (preventing shorts) without blocking the via placement step
    # which uses extra_obstacles (drills only) for F.Cu free space.
    _fcu_stub_obs = (
        fcu_stub_extra_obstacles if fcu_stub_extra_obstacles is not None
        else extra_obstacles
    )
    fs_F_stub, obs_F_stub = build_windowed_free_space(
        pads, net_name, clearance_mm, track_mm,
        _fcu_stub_obs, fcu_window,
        board_outline=board_outline, drill_obstacles=drill_obstacles, layer=0,
    )

    # Try all_rings mode first (needed for QFN dense pads), then fall back
    fcu_path: list[tuple[float, float]] | None = None
    for use_ar in [True, False]:
        nodes_f, adj_f, _nn, _ne = build_visibility_graph(
            free_space=fs_F_stub,
            start_xy=source_xy,
            goal_xy=escape_via,
            margin_mm=fcu_margin,
            obstacle_polys=obs_F_stub,
            use_reflex_pruning=(not use_ar),
            use_all_rings=use_ar,
        )
        # Need at least one edge from start (idx 0)
        if adj_f.get(0):
            path_candidate = astar_visgraph(nodes_f, adj_f, goal_xy=escape_via)
            if path_candidate is not None:
                fcu_path = path_candidate
                break

    if fcu_path is None or len(fcu_path) < 2:
        return GridlessRouteResult(
            ok=False,
            reason="fanout_escape: fcu_stub_failed",
            stats={"total_time_s": round(_time.perf_counter() - t0, 6)},
        )

    # F.Cu leg (layer=0)
    fcu_3d: list[tuple[float, float, int]] = [(x, y, 0) for x, y in fcu_path]

    # --- Stub-only mode: return F.Cu stub + escape via for MST integration ---
    if dest_xy is None:
        total_time = round(_time.perf_counter() - t0, 6)
        stats = {
            "fcu_waypoints": len(fcu_3d),
            "escape_via": escape_via,
            "total_time_s": total_time,
        }
        return GridlessRouteResult(
            ok=True,
            world_paths=[fcu_3d],
            world_vias=[escape_via],
            stats=stats,
        )

    # --- Full two-layer mode: also route B.Cu run to dest_xy ---
    # Manhattan routing strategy: prefer axis-aligned paths to minimise grid
    # corridor occupancy.  Diagonal escape runs block a wide swath of B.Cu
    # cells, displacing grid vias for non-target nets and causing hole_to_hole
    # DRC violations.  Manhattan paths (vertical-first or horizontal-first)
    # confine blockage to narrow column + J3/J4-row bands so grid routing
    # for adjacent non-target nets has unobstructed corridors.
    bcu_dist = _math.hypot(dest_xy[0] - vx, dest_xy[1] - vy)
    bcu_margin = max(bcu_dist * 0.35, 5.0)
    # B.Cu run window cap: use max_bcu_window_mm when provided, else max_window_mm.
    # This is separate from max_window_mm (which governs via-placement and F.Cu
    # stub) because the B.Cu window can be large (via→dest span + margin) and
    # capture hundreds of B.Cu track obstacles → O(n²) unary_union → OOM.
    _bcu_cap = max_bcu_window_mm if max_bcu_window_mm is not None else max_window_mm
    if _bcu_cap is not None:
        bcu_margin = min(bcu_margin, _bcu_cap)

    # Build the B.Cu free-space once with the largest window
    _bcu_obs = bcu_extra_obstacles if bcu_extra_obstacles is not None else extra_obstacles
    _bcu_margin_max = (min(bcu_margin * 2.0, _bcu_cap)
                       if _bcu_cap is not None else bcu_margin * 2.0)
    bcu_wx1 = max(min(vx, dest_xy[0]) - _bcu_margin_max, bx1)
    bcu_wy1 = max(min(vy, dest_xy[1]) - _bcu_margin_max, by1)
    bcu_wx2 = min(max(vx, dest_xy[0]) + _bcu_margin_max, bx2)
    bcu_wy2 = min(max(vy, dest_xy[1]) + _bcu_margin_max, by2)
    bcu_window_max = (bcu_wx1, bcu_wy1, bcu_wx2, bcu_wy2)
    fs_B_run, obs_B_run = build_windowed_free_space(
        pads, net_name, clearance_mm, track_mm,
        _bcu_obs, bcu_window_max,
        board_outline=board_outline, drill_obstacles=drill_obstacles, layer=1,
    )

    dx, dy = dest_xy

    def _seg_in_free(p1: tuple[float, float], p2: tuple[float, float]) -> bool:
        """Return True if the track centerline segment p1→p2 lies inside fs."""
        try:
            from shapely.geometry import LineString as _LSeg
            seg = _LSeg([p1, p2]).buffer(track_mm / 2.0, cap_style=2)
            return bool(fs_B_run.contains(seg))
        except Exception:  # noqa: BLE001
            return False

    bcu_path: list[tuple[float, float]] | None = None

    # Option 0: direct straight line (degenerate but avoids extra bend)
    if _seg_in_free((vx, vy), (dx, dy)):
        bcu_path = [(vx, vy), (dx, dy)]

    if bcu_path is None:
        # Option A: vertical-first  (vx,vy)→(vx,dy)→(dx,dy)
        corner_a = (vx, dy)
        if _seg_in_free((vx, vy), corner_a) and _seg_in_free(corner_a, (dx, dy)):
            bcu_path = [(vx, vy), corner_a, (dx, dy)]

    if bcu_path is None:
        # Option B: horizontal-first  (vx,vy)→(dx,vy)→(dx,dy)
        corner_b = (dx, vy)
        if _seg_in_free((vx, vy), corner_b) and _seg_in_free(corner_b, (dx, dy)):
            bcu_path = [(vx, vy), corner_b, (dx, dy)]

    if bcu_path is None:
        # Fall back to visibility-graph A* if Manhattan paths are blocked
        for _attempt, cur_bcu_margin in enumerate([bcu_margin, bcu_margin * 2.0]):
            if _bcu_cap is not None:
                cur_bcu_margin = min(cur_bcu_margin, _bcu_cap)
            bcu_wx1_fb = max(min(vx, dx) - cur_bcu_margin, bx1)
            bcu_wy1_fb = max(min(vy, dy) - cur_bcu_margin, by1)
            bcu_wx2_fb = min(max(vx, dx) + cur_bcu_margin, bx2)
            bcu_wy2_fb = min(max(vy, dy) + cur_bcu_margin, by2)
            bcu_window_fb = (bcu_wx1_fb, bcu_wy1_fb, bcu_wx2_fb, bcu_wy2_fb)

            fs_B_fb, obs_B_fb = build_windowed_free_space(
                pads, net_name, clearance_mm, track_mm,
                _bcu_obs, bcu_window_fb,
                board_outline=board_outline, drill_obstacles=drill_obstacles, layer=1,
            )

            # Only try reflex pruning: the full-corner fallback (use_reflex=False)
            # extracts ALL obstacle polygon vertices (up to 6000+ on mitayi) and
            # builds O(n²) numpy edge arrays → 4-5 GB RSS.  If reflex-only fails,
            # accept that the B.Cu run failed for this window attempt; the outer
            # bcu_path=None check will return ok=False for this TH destination so
            # the caller tries the next nearest TH pad.
            nodes_b, adj_b, _nn2, _ne2 = build_visibility_graph(
                free_space=fs_B_fb,
                start_xy=escape_via,
                goal_xy=dest_xy,
                margin_mm=cur_bcu_margin,
                obstacle_polys=obs_B_fb,
                use_reflex_pruning=True,
            )
            path_b = astar_visgraph(nodes_b, adj_b, goal_xy=dest_xy)
            if path_b is not None:
                bcu_path = path_b
                break

    if bcu_path is None or len(bcu_path) < 2:
        return GridlessRouteResult(
            ok=False,
            reason="fanout_escape: bcu_run_failed",
            stats={"total_time_s": round(_time.perf_counter() - t0, 6)},
        )

    bcu_3d: list[tuple[float, float, int]] = [(x, y, 1) for x, y in bcu_path]

    # Deduplicate: remove exact duplicate same-layer points at junction
    combined: list[tuple[float, float, int]] = []
    prev: tuple[float, float, int] | None = None
    for wp in fcu_3d + bcu_3d:
        if (
            prev is not None
            and abs(wp[0] - prev[0]) < 1e-9
            and abs(wp[1] - prev[1]) < 1e-9
            and wp[2] == prev[2]
        ):
            continue
        combined.append(wp)
        prev = wp

    total_time = round(_time.perf_counter() - t0, 6)
    stats = {
        "fcu_waypoints": len(fcu_3d),
        "bcu_waypoints": len(bcu_3d),
        "escape_via": escape_via,
        "total_time_s": total_time,
    }

    return GridlessRouteResult(
        ok=True,
        world_paths=[combined],
        world_vias=[escape_via],
        stats=stats,
    )


# ---------------------------------------------------------------------------
# M3-P2: Deterministic Prim's MST over pads
# ---------------------------------------------------------------------------


def _prim_mst(pads: list[dict]) -> list[tuple[int, int, float]]:
    """Deterministic Prim's MST over pads.

    Edge weight = Euclidean distance between pad centres.
    Seeded at index 0 (lowest-index pad).
    Ties broken by ``(dist, pad_i, pad_j)`` heap key.

    Returns
    -------
    List of ``(pad_i, pad_j, dist)`` edges forming the MST (K-1 edges for
    K pads), in the order they were added to the tree.
    """
    K = len(pads)
    if K < 2:
        return []

    in_tree = [False] * K
    in_tree[0] = True

    heap: list[tuple[float, int, int]] = []
    for j in range(1, K):
        d = math.hypot(pads[0]["x"] - pads[j]["x"], pads[0]["y"] - pads[j]["y"])
        heapq.heappush(heap, (d, 0, j))

    edges: list[tuple[int, int, float]] = []

    while len(edges) < K - 1:
        if not heap:
            break
        d, i, j = heapq.heappop(heap)
        if in_tree[j]:
            continue
        in_tree[j] = True
        edges.append((i, j, d))

        for k in range(K):
            if not in_tree[k]:
                dk = math.hypot(pads[j]["x"] - pads[k]["x"], pads[j]["y"] - pads[k]["y"])
                heapq.heappush(heap, (dk, j, k))

    return edges


# ---------------------------------------------------------------------------
# M3-P2: Same-net copper as goal geometry (Shapely)
# ---------------------------------------------------------------------------


def _same_net_copper_geom(
    world_paths: list[list[tuple]],
    via_centers: list[tuple[float, float]],
    track_mm: float,
    via_mm: float,
) -> object | None:
    """Build the same-net copper union (centerlines + vias) as a Shapely geometry.

    Buffered by ``track_mm/2`` — the ACTUAL copper extent, not the clearance
    inflate.  Later sub-edges can terminate anywhere on this geometry.

    Returns ``None`` if no copper has been routed yet.
    """
    try:
        from shapely.geometry import LineString, Point
        from shapely.ops import unary_union

        from tracewise.route.gridless.geom import snap

        polys: list = []
        for wpath in world_paths:
            if not wpath:
                continue
            pts_2d = [(p[0], p[1]) for p in wpath]
            if len(pts_2d) < 2:
                continue
            ls = snap(LineString(pts_2d).buffer(track_mm / 2.0, cap_style=2))
            polys.append(ls)

        for vx, vy in via_centers:
            circle = snap(Point(vx, vy).buffer(via_mm / 2.0, resolution=16))
            polys.append(circle)

        if not polys:
            return None
        return snap(unary_union(polys))
    except Exception:  # noqa: BLE001
        return None


def _sample_copper_goal_points(
    same_net_geom: object,
    window_bbox: tuple[float, float, float, float],
    track_mm: float,
    n_max: int = 40,
) -> list[tuple[float, float]]:
    """Sample reachable connection points on same-net copper within the window.

    Returns a sorted list of ``(x, y)`` candidates for same-net-copper-as-goal.
    """
    try:
        from shapely.geometry import box as _box

        from tracewise.route.gridless.geom import snap

        if same_net_geom is None or same_net_geom.is_empty:
            return []

        wx1, wy1, wx2, wy2 = window_bbox
        window_poly = _box(wx1, wy1, wx2, wy2)
        clipped = snap(same_net_geom.intersection(window_poly))
        if clipped.is_empty:
            return []

        pts: set[tuple[float, float]] = set()
        geoms = list(clipped.geoms) if clipped.geom_type == "MultiPolygon" else [clipped]
        for g in geoms:
            if not hasattr(g, "exterior"):
                continue
            for x, y in g.exterior.coords:
                pts.add((round(x * 1e6) / 1e6, round(y * 1e6) / 1e6))
            ext = g.exterior
            total_len = ext.length
            if total_len > 0:
                step = max(track_mm * 2, total_len / n_max)
                d = 0.0
                while d <= total_len:
                    pt = ext.interpolate(d)
                    pts.add((round(pt.x * 1e6) / 1e6, round(pt.y * 1e6) / 1e6))
                    d += step

        result = sorted(pts, key=lambda p: (round(p[0], 6), round(p[1], 6)))[:n_max]
        return result
    except Exception:  # noqa: BLE001
        return []


# ---------------------------------------------------------------------------
# M3-P2: Multi-pin connection-tree routing (MST + same-net-copper-as-goal)
# ---------------------------------------------------------------------------


def route_net_multipin(
    pads_of_net: list[dict],
    net_name: str,
    all_pads: list[dict],
    geo: dict,
    board_bbox: tuple[float, float, float, float],
    extra_obstacles: list | None = None,
    window_mm: float = 8.0,
    board_outline: object | None = None,
    drill_obstacles: list | None = None,
    drill_centers: list | None = None,
    max_window_mm: float | None = None,
    allow_via: bool = True,
    skip_full_corner_fallback: bool = False,
) -> GridlessRouteResult:
    """Route a multi-pin net as a Minimum Spanning Tree (M3-P2).

    Decomposes the net into K-1 sub-edges via deterministic Prim's MST, then
    routes each sub-edge sequentially using ``route_net_gridless`` with
    **bounded per-sub-edge windows**.  After each successful sub-edge, the
    net's realized copper is added as same-net connection geometry so later
    sub-edges can terminate anywhere on already-routed copper (copper shortcut).

    Parameters
    ----------
    pads_of_net:
        Pads belonging to this net (from ``extract_pads``).  Must have keys
        ``x``, ``y``, ``hw``, ``hh``, ``front``, ``back``, ``net``.
    net_name:
        Net name (used for own-pad carve-out in free-space construction).
    all_pads:
        All board pads (including other nets), used by ``route_net_gridless``
        to build the obstacle set.
    geo:
        Project geometry dict (``track_mm``, ``clearance_mm``, ``via_mm``,
        ``via_drill_mm``, etc.).
    board_bbox:
        ``(x1, y1, x2, y2)`` board bounding box.
    extra_obstacles:
        Pre-inflated Shapely polygons of already-routed OTHER-net copper.
    window_mm:
        Initial routing window half-margin per sub-edge.
    board_outline:
        Optional Shapely Polygon of the board interior.
    drill_obstacles:
        Pre-inflated Shapely circle obstacles for drill holes.
    drill_centers:
        ``(cx, cy, drill_r)`` tuples for hole-to-hole via legality check.
    max_window_mm:
        Hard cap on the per-sub-edge window escalation (mm).  When provided,
        the sub-edge window never exceeds this value.  Use ~20-25 mm for the
        gridless-first fast path on clean boards to prevent O(n²) blowup.
        ``None`` (default) = board diagonal (original behaviour, unchanged).
    allow_via:
        When ``True`` (default), each sub-edge attempts 2-layer F.Cu→via→B.Cu
        if single-layer fails.  Set to ``False`` for the gridless-first fast
        path on clean boards: via-site search is expensive on large free spaces;
        single-layer-only mirrors the Probe-Order fast path that routes 17 nets
        in ~39 s.  Failed sub-edges are skipped cleanly (partial tree).
    skip_full_corner_fallback:
        Forwarded to each ``route_net_gridless`` sub-edge call.  When True,
        skips the expensive full-corner fallback in ``route_window`` that can
        consume 4-5 GB RAM when ``extra_obstacles`` is large.  Default False.

    Returns
    -------
    ``GridlessRouteResult`` with combined ``world_paths`` and ``world_vias``
    from all successful sub-edges.  ``ok=True`` iff ALL K-1 MST sub-edges
    routed successfully.
    """
    _require_shapely()

    K = len(pads_of_net)
    if K < 2:
        return GridlessRouteResult(ok=True, world_paths=[], world_vias=[],
                                   reason="trivially connected (< 2 pads)")
    if K == 2:
        # Fast path: 2-pin net → delegate to the standard route_net_gridless.
        pa = pads_of_net[0]
        pb = pads_of_net[1]
        return route_net_gridless(
            pad_a=(pa["x"], pa["y"]),
            pad_b=(pb["x"], pb["y"]),
            pads=all_pads,
            net_name=net_name,
            geo=geo,
            board_bbox=board_bbox,
            extra_obstacles=extra_obstacles,
            window_mm=window_mm,
            board_outline=board_outline,
            drill_obstacles=drill_obstacles,
            drill_centers=drill_centers,
            allow_via=allow_via,
            max_window_mm=max_window_mm,
            skip_full_corner_fallback=skip_full_corner_fallback,
        )

    if extra_obstacles is None:
        extra_obstacles = []
    if drill_obstacles is None:
        drill_obstacles = []
    if drill_centers is None:
        drill_centers = []

    track_mm: float = geo["track_mm"]
    via_mm: float = geo.get("via_mm", 0.6)
    bx1, by1, bx2, by2 = board_bbox

    # --- Step 1: Build MST (deterministic Prim, seed=0, tie-break by dist,i,j) ---
    mst_edges = _prim_mst(pads_of_net)

    # --- Step 2: Route K-1 sub-edges sequentially ---
    all_world_paths: list[list[tuple]] = []
    all_via_centers: list[tuple[float, float]] = []
    same_net_geom: object | None = None  # grows after each successful sub-edge

    n_ok = 0
    n_failed = 0
    total_stats: dict = {
        "nodes": 0, "edges": 0, "escalations": 0,
        "build_time_s": 0.0, "solve_time_s": 0.0, "total_time_s": 0.0,
    }

    t_total_start = time.perf_counter()

    for _ei, (i, j, _dist) in enumerate(mst_edges):
        tree_pad = pads_of_net[i]
        new_pad = pads_of_net[j]
        tx, ty = tree_pad["x"], tree_pad["y"]
        nx, ny = new_pad["x"], new_pad["y"]
        pad_dist = math.hypot(nx - tx, ny - ty)

        # Bounded window for this sub-edge: max(window_mm, pad_dist*1.2+2mm),
        # further capped at max_window_mm when provided (prevents O(n²) blowup
        # on clean boards in the gridless-first fast path).
        _sub_window_uncapped = max(window_mm, pad_dist * 1.2 + 2.0)
        sub_window = (
            min(_sub_window_uncapped, max_window_mm)
            if max_window_mm is not None
            else _sub_window_uncapped
        )
        wx1 = max(min(tx, nx) - sub_window, bx1)
        wy1 = max(min(ty, ny) - sub_window, by1)
        wx2 = min(max(tx, nx) + sub_window, bx2)
        wy2 = min(max(ty, ny) + sub_window, by2)
        sub_window_bbox = (wx1, wy1, wx2, wy2)

        # Choose start/goal: use same-net copper shortcut if available
        if same_net_geom is None:
            # First sub-edge: route tree_pad → new_pad directly
            start_xy = (tx, ty)
            goal_xy = (nx, ny)
        else:
            # Later sub-edge: route FROM new_pad TO nearest point on tree copper.
            copper_pts = _sample_copper_goal_points(
                same_net_geom, sub_window_bbox, track_mm, n_max=40
            )

            if copper_pts:
                nearest = min(copper_pts,
                              key=lambda pt: math.hypot(pt[0] - nx, pt[1] - ny))
                copper_dist = math.hypot(nearest[0] - nx, nearest[1] - ny)

                # Use copper shortcut only if strictly shorter than pad-to-pad
                # AND the copper point is NOT at the tree_pad (interior shortcut).
                is_at_tree_pad = (
                    abs(nearest[0] - tx) < track_mm
                    and abs(nearest[1] - ty) < track_mm
                )
                if copper_dist < pad_dist - 0.05 and not is_at_tree_pad:
                    # Genuine copper shortcut
                    start_xy = (nx, ny)
                    goal_xy = nearest
                    # Re-derive window around new sub-edge endpoints (also capped)
                    _sub_window2_uncapped = max(window_mm, copper_dist * 1.2 + 2.0)
                    sub_window2 = (
                        min(_sub_window2_uncapped, max_window_mm)
                        if max_window_mm is not None
                        else _sub_window2_uncapped
                    )
                    wx1 = max(min(nx, goal_xy[0]) - sub_window2, bx1)
                    wy1 = max(min(ny, goal_xy[1]) - sub_window2, by1)
                    wx2 = min(max(nx, goal_xy[0]) + sub_window2, bx2)
                    wy2 = min(max(ny, goal_xy[1]) + sub_window2, by2)
                else:
                    # No genuine shortcut: route new_pad → tree_pad
                    start_xy = (nx, ny)
                    goal_xy = (tx, ty)
            else:
                # No copper points found in window: route new_pad → tree_pad
                start_xy = (nx, ny)
                goal_xy = (tx, ty)

        result = route_net_gridless(
            pad_a=start_xy,
            pad_b=goal_xy,
            pads=all_pads,
            net_name=net_name,
            geo=geo,
            board_bbox=board_bbox,
            extra_obstacles=extra_obstacles,
            window_mm=sub_window,
            board_outline=board_outline,
            drill_obstacles=drill_obstacles,
            drill_centers=drill_centers,
            allow_via=allow_via,
            max_window_mm=max_window_mm,
            skip_full_corner_fallback=skip_full_corner_fallback,
        )

        # Accumulate stats
        for k in ("nodes", "edges", "escalations"):
            total_stats[k] = max(total_stats[k], result.stats.get(k, 0))
        for k in ("build_time_s", "solve_time_s", "total_time_s"):
            total_stats[k] = total_stats.get(k, 0.0) + result.stats.get(k, 0.0)

        if not result.ok:
            n_failed += 1
            # Failure: continue with remaining sub-edges (partial tree)
            continue

        n_ok += 1
        all_world_paths.extend(result.world_paths)
        all_via_centers.extend(result.world_vias)

        # Update same-net copper geometry for next sub-edge
        new_geom = _same_net_copper_geom(
            result.world_paths, result.world_vias, track_mm, via_mm
        )
        if new_geom is not None:
            if same_net_geom is None:
                same_net_geom = new_geom
            else:
                try:
                    from shapely.ops import unary_union

                    from tracewise.route.gridless.geom import snap
                    same_net_geom = snap(unary_union([same_net_geom, new_geom]))
                except Exception:  # noqa: BLE001
                    same_net_geom = new_geom

    total_stats["total_time_s"] = round(time.perf_counter() - t_total_start, 6)

    if n_ok == 0:
        return GridlessRouteResult(
            ok=False,
            stats=total_stats,
            reason=f"all {len(mst_edges)} MST sub-edges failed",
        )

    # Partial success is still returned with ok=True for the sub-set that connected;
    # engine wiring decides whether to accept partial connectivity.
    ok = (n_failed == 0)
    reason = (
        "" if ok
        else f"{n_failed}/{len(mst_edges)} MST sub-edge(s) failed"
    )

    return GridlessRouteResult(
        ok=ok,
        world_paths=all_world_paths,
        world_vias=all_via_centers,
        stats=total_stats,
        reason=reason,
    )
