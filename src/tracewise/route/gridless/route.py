"""route — top-level route_net_gridless entry point for the FAR gridless router.

This module provides the standalone ``route_net_gridless`` function and the
``GridlessRouteResult`` dataclass.  It is a self-contained Phase 1 deliverable;
the ``GridlessNetRoute`` adapter (``NetRoute`` IS-A) and engine wiring belong to
Phase 2.

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
    max_window = max(board_w, board_h)

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
) -> GridlessRouteResult | None:
    """Attempt a 2-layer F.Cu→via→B.Cu route for a geometry-blocked net.

    Returns a ``GridlessRouteResult`` with ``ok=True`` on success, or ``None``
    if no 2-layer path was found (caller will fall back to ``ok=False``).

    This helper is only called after the single-layer A* has exhausted all
    window escalations.  It does NOT escalate the window further; the full-board
    window (board diagonal) is used as the single attempt so via sites are
    drawn from the complete routing surface.
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
    # Use full-board window so via candidates aren't artificially restricted
    window_mm_2l = max(abs(bx2 - bx1), abs(by2 - by1))
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
