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

from tracewise.route.gridless.geom import _require_shapely, build_windowed_free_space
from tracewise.route.gridless.realize import realize_centerline
from tracewise.route.gridless.search import route_window

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
        List of path segments, each a list of ``(x_mm, y_mm)`` waypoints.
        Normally one segment per 2-pin net; multi-pin MST is Phase 3.
    stats:
        Performance counters: ``nodes``, ``edges``, ``escalations``,
        ``build_time_s``, ``solve_time_s``, ``total_time_s``.
    reason:
        Human-readable failure reason when ``ok=False``.
    """

    ok: bool
    world_paths: list[list[tuple[float, float]]] = field(default_factory=list)
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
) -> GridlessRouteResult:
    """Route a 2-pin net gridlessly via visibility-graph A*.

    Parameters
    ----------
    pad_a, pad_b:
        Start and goal pad centres in world mm ``(x, y)``.
    pads:
        All board pads from ``extract_pads`` (dicts with keys ``net``, ``x``,
        ``y``, ``hw``, ``hh``, ``front``).  Used to build the obstacle set.
    net_name:
        The net being routed (own-pad carve-out uses this to protect pad centres).
    geo:
        Project geometry dict with at least ``track_mm`` and ``clearance_mm``.
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
