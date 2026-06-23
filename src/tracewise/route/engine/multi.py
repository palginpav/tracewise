"""R1: multi-net routing — ordering, copper-as-obstacle, bounded rip-up.

Each net grows a connection tree: pad 2 routes to pad 1, pad 3 to the nearest
cell of the existing tree (multi-goal A*). Routed copper is marked into the
grid (inflated) and becomes an obstacle for later nets. Ordering per the
design doc: power-class names first (they block the most cells), then
shortest-bbox. Failures trigger bounded rip-up-and-reroute: the failed net
rips the routed net whose copper sits closest to its straight line, routes,
and the victim re-routes; the global budget caps total attempts.

A net is either fully routed (every pad in one tree) or reported failed with
its reason — partial nets are unmarked and discarded, never left as stubs.
"""

from __future__ import annotations

import re
import warnings
from dataclasses import dataclass, field

from tracewise.route.engine.astar import RouteResult, route
from tracewise.route.engine.grid import Grid

POWER = re.compile(r"^\+?(VCC|VDD|VBUS|VBAT|VSYS|GND|VSS|[0-9]+V[0-9]*)$", re.IGNORECASE)


@dataclass
class Net:
    name: str
    pads: list[tuple[int, int, int]]  # (layer, iy, ix)
    halfwidth_cells: int = 1  # track halfwidth + clearance, in cells
    via_halfwidth_cells: int = 5  # via radius + clearance, in cells
    # own pad rects to carve free while routing: (layer, x1, y1, x2, y2, inflate_mm)
    carve: list[tuple] = field(default_factory=list)


@dataclass
class NetRoute:
    net: Net
    paths: list[list[tuple[int, int, int]]] = field(default_factory=list)
    cells: set[tuple[int, int, int]] = field(default_factory=set)
    via_sites: set[tuple[int, int]] = field(default_factory=set)  # (iy, ix)
    escape_cells: set[tuple[int, int, int]] = field(default_factory=set)
    ok: bool = False
    reason: str = ""
    # pads left unconnected when allow_partial routed the net incrementally
    unroutable_pads: list = field(default_factory=list)


def _mark(grid: Grid, nr: NetRoute, delta: int) -> None:
    r = nr.net.halfwidth_cells
    for layer, iy, ix in nr.cells:
        y1, y2 = max(0, iy - r), min(grid.ny, iy + r + 1)
        x1, x2 = max(0, ix - r), min(grid.nx, ix + r + 1)
        grid.cells[layer, y1:y2, x1:x2] += delta
        grid.hard[layer, y1:y2, x1:x2] += delta  # routed copper is never escapable
    rv = nr.net.via_halfwidth_cells  # vias block every layer with their full disc
    for iy, ix in nr.via_sites:
        y1, y2 = max(0, iy - rv), min(grid.ny, iy + rv + 1)
        x1, x2 = max(0, ix - rv), min(grid.nx, ix + rv + 1)
        grid.cells[:, y1:y2, x1:x2] += delta
        grid.hard[:, y1:y2, x1:x2] += delta


def _bbox_size(net: Net) -> int:
    ys = [p[1] for p in net.pads]
    xs = [p[2] for p in net.pads]
    return (max(ys) - min(ys)) + (max(xs) - min(xs))


def order_nets(nets: list[Net], priority: dict[str, int] | None = None) -> list[Net]:
    """Power first, then failed-before nets (negotiated priority), then short."""
    pr = priority or {}
    return sorted(nets, key=lambda n: (-pr.get(n.name, 0),
                                       0 if POWER.match(n.name) else 1, _bbox_size(n)))


def route_net(grid: Grid, net: Net, via_cost: float = 10.0, escape: int = 0,
              history=None, history_factor: float = 0.0,
              allow_partial: bool = False) -> NetRoute:
    nr = NetRoute(net=net)
    if len(net.pads) < 2:
        nr.ok = True  # nothing to connect
        return nr
    for layer, x1, y1, x2, y2, inf in net.carve:  # own pads must not wall the net in
        grid.block_pad(layer, x1, y1, x2, y2, inflate_mm=inf, delta=-1)
    tree: set[tuple[int, int, int]] = {net.pads[0]}
    unroutable: list = []
    for pad in net.pads[1:]:
        if pad in tree:
            continue
        res: RouteResult = route(grid, pad, tree, via_cost=via_cost, escape=escape,
                                 history=history, history_factor=history_factor)
        if not res.ok:
            if allow_partial:
                # INCREMENTAL: keep the sub-tree already built and leave this one
                # pad unconnected, instead of discarding every connection because
                # of a single blocked pad. A high-fanout net (e.g. +3V0, 58 pads)
                # where one pad is walled in must not lose its other 56 routes —
                # the all-or-nothing rule was the dominant persistence failure
                # (measured: 4 nets failed whole on a single 'no path' pad).
                # The partial tree is real connected copper (>=2 pads), never a
                # dangling stub.
                unroutable.append(pad)
                continue
            nr.reason = f"pad {pad}: {res.reason}"
            nr.cells.clear()
            nr.paths.clear()
            for layer, x1, y1, x2, y2, inf in net.carve:
                grid.block_pad(layer, x1, y1, x2, y2, inflate_mm=inf, delta=1)
            return nr  # all-or-nothing: no stubs
        nr.paths.append(res.path)
        nr.escape_cells |= res.escaped
        vias = {(a[1], a[2]) for a, b in zip(res.path, res.path[1:], strict=False)
                if a[0] != b[0]}
        nr.via_sites.update(vias)
        # via barrels never enter the goal tree: branching into them would
        # stack a second via on the same hole (dangling + hole-to-hole)
        tree.update(n for n in res.path if (n[1], n[2]) not in vias)
    nr.cells = tree - set(net.pads)  # pads stay as the pad obstacles they are
    for layer, x1, y1, x2, y2, inf in net.carve:
        grid.block_pad(layer, x1, y1, x2, y2, inflate_mm=inf, delta=1)
    if allow_partial and unroutable:
        # partial success: connected what it could; report the residual pads
        nr.ok = bool(nr.paths)
        nr.reason = f"{len(unroutable)} pad(s) unroutable" if not nr.paths else ""
        nr.unroutable_pads = unroutable
    else:
        nr.ok = True
    return nr


def _nearest_victim(failed: Net, routed: list[NetRoute]) -> NetRoute | None:
    """The routed net whose copper is closest to the failed net's pad bbox."""
    ys = [p[1] for p in failed.pads]
    xs = [p[2] for p in failed.pads]
    cy, cx = (max(ys) + min(ys)) / 2, (max(xs) + min(xs)) / 2
    best, best_d = None, None
    for nr in routed:
        if not nr.cells:
            continue
        d = min(abs(iy - cy) + abs(ix - cx) for _, iy, ix in nr.cells)
        if best_d is None or d < best_d:
            best, best_d = nr, d
    return best


def _route_net_gridless_wrapped(grid: Grid, net: Net, kwargs: dict) -> NetRoute:
    """Route *net* via the gridless engine and return a ``GridlessNetRoute``.

    Extracts the two pad world-coordinates from *net.pads* + *grid* (reverse-
    mapping cell -> world via ``grid.to_world``), calls ``route_net_gridless``,
    and wraps the result in a ``GridlessNetRoute`` whose ``cells`` field is
    populated by ``rasterize_into_grid`` so ``_mark`` / occupancy ledger work.

    Returns a plain failed ``NetRoute`` (``ok=False``) when the net has fewer
    than two pads, when ``pads``/``geo``/``board_bbox`` are absent from
    *kwargs*, or when the gridless router finds no path.
    """
    from tracewise.route.gridless.adapter import to_gridless_netroute
    from tracewise.route.gridless.route import route_net_gridless

    if len(net.pads) < 2:
        nr = NetRoute(net=net, ok=True)  # trivially connected
        return nr

    pads = kwargs.get("pads")
    geo = kwargs.get("geo")
    board_bbox = kwargs.get("board_bbox")
    if pads is None or geo is None or board_bbox is None:
        return NetRoute(net=net, ok=False,
                        reason="gridless_kwargs missing pads/geo/board_bbox")

    # Resolve world-mm coordinates for the first and second pad.
    # net.pads are (layer, iy, ix) tuples; reverse-map via grid.to_world.
    # Only 2-pin nets are supported in Phase 1.
    pad_a_cell = net.pads[0]
    pad_b_cell = net.pads[1]
    pad_a = grid.to_world(pad_a_cell[1], pad_a_cell[2])
    pad_b = grid.to_world(pad_b_cell[1], pad_b_cell[2])

    # Use anchors if provided (exact world-mm pad centres, not grid-snapped).
    anchors = kwargs.get("anchors")
    if anchors is not None:
        pad_a = anchors.get(pad_a_cell, pad_a)
        pad_b = anchors.get(pad_b_cell, pad_b)

    # Build extra_obstacles from already-routed gridless nets' world paths.
    # Grid copper is already in grid.cells; only gridless copper needs explicit
    # Shapely obstacles since it may not be fully captured by the grid cells.
    extra_obstacles = kwargs.get("extra_gridless_obstacles") or []

    board_outline = kwargs.get("board_outline")
    drill_obstacles = kwargs.get("drill_obstacles")

    drill_centers = kwargs.get("drill_centers") or []

    result = route_net_gridless(
        pad_a=pad_a,
        pad_b=pad_b,
        pads=pads,
        net_name=net.name,
        geo=geo,
        board_bbox=board_bbox,
        extra_obstacles=extra_obstacles,
        board_outline=board_outline,
        drill_obstacles=drill_obstacles,
        drill_centers=drill_centers,
        allow_via=True,
    )

    if not result.ok:
        return NetRoute(net=net, ok=False, reason=result.reason)

    nr = to_gridless_netroute(net, result.world_paths, grid,
                              world_vias=result.world_vias)

    # Accumulate Shapely obstacles for subsequent gridless nets in this run.
    # Buffer each world segment by track_mm/2 + clearance_mm (FIX-6).
    if "extra_gridless_obstacles" in kwargs:
        try:
            from shapely.geometry import LineString

            from tracewise.route.gridless.geom import snap
            track_mm = geo.get("track_mm", 0.2)
            clearance_mm = geo.get("clearance_mm", 0.2)
            inflate = track_mm / 2.0 + clearance_mm
            for path in result.world_paths:
                if len(path) >= 2:
                    ls = snap(LineString(path).buffer(inflate, cap_style=2))
                    kwargs["extra_gridless_obstacles"].append(ls)
        except Exception:  # noqa: BLE001 — Shapely may be absent; skip silently
            pass

    return nr


def route_all(grid: Grid, nets: list[Net], via_cost: float = 10.0,
              ripup_factor: int = 8, escape: int = 0,
              priority: dict[str, int] | None = None,
              history_factor: float = 0.0,
              allow_partial: bool = False,
              salvage_escape: int = 0,
              gridless_nets: set[str] | None = None,
              gridless_kwargs: dict | None = None,
              gridless_negotiate: bool = False,
              gridless_rescue: bool = False,
              gridless_first: set[str] | None = None,
              **_legacy) -> dict[str, NetRoute]:
    """Route every net; bounded rip-up on failures. Returns name -> NetRoute.

    The rip-up loop is capped deterministically: the total number of net-routing
    attempts is `ripup_factor * len(nets)` (the `budget` counter).  When the
    budget is exhausted, remaining nets are returned as explicit failures with
    reason "rip-up budget exhausted".  There is no wall-clock deadline — the
    same board+placement always produces the same result regardless of machine
    speed or load.

    `history_factor` > 0 turns on negotiated-congestion pricing (salvaged from
    the PathFinder experiment): each rip-up deposits congestion history on the
    victim's cells, and subsequent routes are nudged around those chronically-
    contested regions. 0 (default) reproduces the original pure rip-up.

    `gridless_nets` is an optional set of net names to route via the
    visibility-graph gridless engine instead of the grid A*.  Nets not in the
    set route on the grid as today.  ``None`` (default) = 100% grid path,
    byte-identical to the pre-Phase-2 behaviour.  Gridless copper is rasterized
    into the shared grid ledger so subsequently-routed grid nets see it as
    occupied.

    `gridless_kwargs` passes board-level context to ``route_net_gridless``
    (keys: ``pads``, ``geo``, ``board_bbox``).  Required when `gridless_nets`
    is non-empty.

    `gridless_negotiate` (default ``False``) enables the M2 staged
    congestion-negotiation path for the gridless subset.  When ``True`` and
    ``gridless_nets`` is non-empty, the entire gridless set is routed first via
    ``route_gridless_set`` (congestion history + bounded rip-up among the gridless
    nets); each successfully routed gridless net is rasterized into the shared
    grid ledger so that the subsequent grid pass sees their copper as occupied.
    Grid nets are then routed by the EXISTING grid rip-up loop unchanged.
    Geometry-blocked gridless nets are reported as failed with reason
    ``"geometry-blocked (M3)"`` and do NOT enter the grid rip-up loop.

    `gridless_rescue` (default ``False``) enables the M2.1 grid-first rescue mode.
    When ``True`` and ``gridless_kwargs`` is provided:
      1. Route ALL nets via the grid rip-up loop first (unchanged).
      2. Collect the nets left UNCONNECTED by the grid: filter to 2-pin F.Cu
         candidates that are not geometry-blocked (``>50%`` board-diagonal
         quarantine from ``route_gridless_set``'s pre-classification).
      3. Route those candidates via ``route_gridless_set`` with ALL existing copper
         as obstacles: pads + board edge + drill holes (already modeled) PLUS the
         emitted grid track segments (the key new piece: each grid track centreline
         buffered by ``track_mm/2 + clearance_mm`` via
         ``net_routes_to_track_obstacles``).
      4. Successfully rescued gridless nets replace their failed grid entries and
         their copper is rasterized into the shared grid ledger.
    Distinct from ``gridless_negotiate`` (M2 pre-route mode); both can coexist.

    `gridless_first` (default ``None``) enables the gridless-first ordering mode:
    the nominated set of "hard" nets are routed GRIDLESS-FIRST on the CLEAN board
    (only pads + board edge + drills as obstacles; no grid tracks yet) via the
    ``route_gridless_set`` negotiate mechanism (congestion history + bounded rip-up
    among them, cross-net aware).  Their copper is rasterized into the shared grid
    ledger, then the normal grid ``route_all`` runs for ALL OTHER nets which now
    route around the gridless-first copper.

    Routing uses single-layer-preferred + BOUNDED windows (the Probe-Order fast
    path): nets try single-layer F.Cu with a bounded window first; geometry-blocked
    nets escalate to 2-layer/via with a BOUNDED via window (capped to avoid the
    board-wide blowup seen in M3-P1.1).

    This is semantically equivalent to ``gridless_negotiate=True`` +
    ``gridless_nets=gridless_first`` but exposes a clean single-param interface.
    When ``gridless_first`` is provided and ``gridless_nets`` is None, the
    ``gridless_first`` set is used as ``gridless_nets`` and ``gridless_negotiate``
    is forced to ``True``.

    Hard invariant: ``gridless_nets=None`` or empty (regardless of
    ``gridless_negotiate``) → behaviour is byte-identical to the pre-M2 engine.
    ``gridless_rescue=False`` (default) → byte-identical to current behaviour.
    ``gridless_first=None`` (default) → no-op, byte-identical to current behaviour.
    """
    # ------------------------------------------------------------------
    # gridless_first: convenience param — activates gridless_negotiate path.
    # When set, merge into gridless_nets and force gridless_negotiate=True.
    # Hard invariant: gridless_first=None → no-op (nothing changes below).
    # ------------------------------------------------------------------
    if gridless_first:
        # Merge with any existing gridless_nets (union, so caller can combine both)
        if gridless_nets:
            gridless_nets = set(gridless_nets) | set(gridless_first)
        else:
            gridless_nets = set(gridless_first)
        gridless_negotiate = True
    if _legacy:
        # Silently accept (and ignore) the removed `time_budget_s` keyword so
        # callers that still pass it do not crash immediately.  Emit a warning
        # once so operators notice they need to remove the argument.
        warnings.warn(
            f"route_all: unknown keyword(s) {list(_legacy)!r} — "
            "'time_budget_s' has been removed; routing is now deterministic.",
            DeprecationWarning, stacklevel=2,
        )
    import numpy as np

    results: dict[str, NetRoute] = {}

    # ------------------------------------------------------------------
    # M2 STAGED PATH: negotiate-route the gridless subset FIRST, then
    # rasterize their copper into the shared grid ledger so the grid
    # remainder sees them as obstacles.  The grid rip-up loop below runs
    # UNCHANGED and only processes nets not in gridless_nets.
    #
    # Hard invariant: if gridless_nets is None/empty, this block is a
    # no-op and the code falls through to the standard grid path below,
    # producing byte-identical results.
    # ------------------------------------------------------------------
    _gridless_pre_routed: set[str] = set()  # names handled in this block
    if gridless_negotiate and gridless_nets:
        from tracewise.route.gridless.adapter import to_gridless_netroute
        from tracewise.route.gridless.negotiate import route_gridless_set

        gk = gridless_kwargs or {}
        pads = gk.get("pads")
        geo = gk.get("geo")
        board_bbox = gk.get("board_bbox")
        anchors = gk.get("anchors")
        neg_ripup_factor = gk.get("negotiate_ripup_factor", ripup_factor)
        neg_history_factor = gk.get("negotiate_history_factor", 3.0)
        neg_window_mm = gk.get("negotiate_window_mm", 4.0)
        neg_geom_threshold = gk.get("negotiate_geom_block_threshold", 0.5)
        neg_board_outline = gk.get("board_outline")
        neg_drill_obstacles = gk.get("drill_obstacles")
        # max_classify_window_mm: hard cap on the pre-classification window
        # escalation loop.  Critical for gridless_first on clean boards —
        # without a cap, large windows produce O(n²) visibility graphs.
        # Default None = board diagonal (original behaviour, unchanged).
        # gridless_first path passes 25mm (Probe-Order fast-path bound).
        neg_max_classify_window = gk.get("negotiate_max_classify_window_mm", None)
        # max_route_window_mm: hard cap on the routing window escalation.
        # Also critical for gridless_first on clean boards — caps the fallback
        # escalation in the rip-up loop.  Default None = original behaviour.
        neg_max_route_window = gk.get("negotiate_max_route_window_mm", None)

        if pads is not None and geo is not None and board_bbox is not None:
            by_name_all = {n.name: n for n in nets}
            net_set = []
            for net_name in sorted(gridless_nets):  # sorted for determinism
                net = by_name_all.get(net_name)
                if net is None or len(net.pads) < 2:
                    continue
                # route_gridless_set is a 2-pin API: only add genuinely 2-pin nets.
                # Multi-pin nets (K>2) are handled below via route_net_multipin.
                if len(net.pads) != 2:
                    continue
                pad_a_cell = net.pads[0]
                pad_b_cell = net.pads[1]
                pad_a = grid.to_world(pad_a_cell[1], pad_a_cell[2])
                pad_b = grid.to_world(pad_b_cell[1], pad_b_cell[2])
                if anchors is not None:
                    pad_a = anchors.get(pad_a_cell, pad_a)
                    pad_b = anchors.get(pad_b_cell, pad_b)
                net_set.append({
                    "net_name": net_name,
                    "pad_a": pad_a,
                    "pad_b": pad_b,
                })

            if net_set:
                neg_results = route_gridless_set(
                    net_set=net_set,
                    pads=pads,
                    geo=geo,
                    board_bbox=board_bbox,
                    anchors=anchors,
                    history_factor=neg_history_factor,
                    ripup_factor=neg_ripup_factor,
                    window_mm_start=neg_window_mm,
                    geom_block_threshold=neg_geom_threshold,
                    board_outline=neg_board_outline,
                    drill_obstacles=neg_drill_obstacles,
                    max_classify_window_mm=neg_max_classify_window,
                    max_route_window_mm=neg_max_route_window,
                )

                # M3-P1.1: collect geometry-blocked nets for 2-layer via attempt.
                # Build pad_map here so the 2-layer pass has world-mm coords.
                _neg_pad_map: dict[str, dict] = {
                    nd["net_name"]: nd for nd in net_set
                }

                for net_name, neg_res in neg_results.items():
                    net = by_name_all.get(net_name)
                    if net is None:
                        continue
                    _gridless_pre_routed.add(net_name)
                    if neg_res.ok and neg_res.world_paths:
                        # Wrap in GridlessNetRoute, rasterize, mark grid ledger
                        nr = to_gridless_netroute(
                            net, neg_res.world_paths, grid,
                            world_vias=neg_res.world_vias,
                        )
                        _mark(grid, nr, 1)
                        results[net_name] = nr
                    elif neg_res.status == "geometry_blocked":
                        # M3-P1.1: geometry-blocked → attempt escape routing
                        # BEFORE any grid copper exists (gridless-FIRST strategy).
                        # Only pad obstacles + board edge + drills are active here.
                        # Strategy order:
                        #   1. Fanout-escape (guided via for dense QFN pads) — when
                        #      the source pad belongs to a dense component.
                        #   2. Standard _route_net_2layer fallback.
                        from tracewise.route.gridless.geom import detect_dense_components
                        from tracewise.route.gridless.route import (
                            _route_net_2layer,
                            route_net_fanout_escape,
                        )

                        nd_entry = _neg_pad_map.get(net_name)
                        drill_centers = gk.get("drill_centers") or []
                        if nd_entry is not None:
                            _pa = nd_entry["pad_a"]
                            _pb = nd_entry["pad_b"]
                            import math as _math
                            _pad_span = _math.hypot(_pb[0] - _pa[0], _pb[1] - _pa[1])
                            _max_win_uncapped = _pad_span * 3.0 + 8.0
                            if neg_max_classify_window is not None:
                                _max_win = min(_max_win_uncapped, neg_max_classify_window)
                            else:
                                _max_win = _max_win_uncapped

                            # --- Strategy 1: Fanout-escape (dense QFN pads) ---
                            # Detect whether the SOURCE pad (the QFN SMD F.Cu pad,
                            # i.e. the pad with front=True and no back) belongs to a
                            # dense component.  The net has exactly 2 pads in net_set
                            # (enforced by the 2-pin filter above), so we look for
                            # the pad in `pads` that matches the source pad world xy.
                            _result_fanout = None
                            _dense_comps = detect_dense_components(pads)
                            _dense_by_ref: dict[str, dict] = {d["ref"]: d for d in _dense_comps}

                            if _dense_by_ref:
                                # Find which pad of this net is the QFN SMD source
                                # (front=True, back=False = SMD F.Cu) vs destination
                                # (thru-hole: both front=True and back=True).
                                _net_pads_world = [
                                    p for p in pads if p.get("net") == net_name
                                ]
                                _source_pad_w = None
                                _dest_pad_w = None
                                for _p in _net_pads_world:
                                    _ref = _p.get("ref", "")
                                    if _ref in _dense_by_ref:
                                        _source_pad_w = _p
                                    else:
                                        _dest_pad_w = _p
                                # Verify: source must be SMD (front only), dest can
                                # be thru-hole.  If found, try fanout-escape.
                                if (
                                    _source_pad_w is not None
                                    and _dest_pad_w is not None
                                    and _source_pad_w.get("front")
                                    and not _source_pad_w.get("back")
                                ):
                                    _dcomp = _dense_by_ref[_source_pad_w["ref"]]
                                    _src_xy = (_source_pad_w["x"], _source_pad_w["y"])
                                    _dst_xy = (_dest_pad_w["x"], _dest_pad_w["y"])
                                    _result_fanout = route_net_fanout_escape(
                                        source_xy=_src_xy,
                                        dest_xy=_dst_xy,
                                        component_cx=_dcomp["cx"],
                                        component_cy=_dcomp["cy"],
                                        ring_radius=_dcomp["ring_radius"],
                                        pads=pads,
                                        net_name=net_name,
                                        geo=geo,
                                        board_bbox=board_bbox,
                                        extra_obstacles=[],   # no grid copper yet
                                        board_outline=neg_board_outline,
                                        drill_obstacles=neg_drill_obstacles or [],
                                        drill_centers=drill_centers,
                                        max_window_mm=_max_win,
                                    )

                            if (
                                _result_fanout is not None
                                and _result_fanout.ok
                                and _result_fanout.world_paths
                            ):
                                nr = to_gridless_netroute(
                                    net, _result_fanout.world_paths, grid,
                                    world_vias=_result_fanout.world_vias,
                                )
                                _mark(grid, nr, 1)
                                results[net_name] = nr
                                continue

                            # --- Strategy 2: Standard 2-layer fallback ---
                            # Bound the via-search window to avoid O(n²) explosion
                            # on the large clean-board free space.  Use 3× the
                            # pad-to-pad span + 8mm as a generous but bounded cap,
                            # further capped at neg_max_classify_window (25mm for
                            # the gridless_first path on clean boards).
                            # The rescue path (post-grid) omits this cap (None)
                            # because fragmented free space is already bounded.
                            result_2l = _route_net_2layer(
                                pad_a=_pa,
                                pad_b=_pb,
                                pads=pads,
                                net_name=net_name,
                                geo=geo,
                                board_bbox=board_bbox,
                                extra_obstacles=[],   # no grid copper yet
                                board_outline=neg_board_outline,
                                drill_obstacles=neg_drill_obstacles or [],
                                drill_centers=drill_centers,
                                window_mm=neg_window_mm,
                                max_window_mm=_max_win,
                            )
                            if result_2l is not None and result_2l.ok and result_2l.world_paths:
                                nr = to_gridless_netroute(
                                    net, result_2l.world_paths, grid,
                                    world_vias=result_2l.world_vias,
                                )
                                _mark(grid, nr, 1)
                                results[net_name] = nr
                                continue
                        # All strategies failed: report as failed
                        results[net_name] = NetRoute(
                            net=net, ok=False, reason=neg_res.reason
                        )
                    else:
                        # Other failure: report as a failed NetRoute
                        results[net_name] = NetRoute(
                            net=net, ok=False, reason=neg_res.reason
                        )

            # ------------------------------------------------------------------
            # gridless_first MULTI-PIN PATH: route K>2 nets via route_net_multipin
            # with bounded windows (mirrors the Probe-Order fast path).
            #
            # route_gridless_set is a 2-pin API and cannot handle multi-pin nets.
            # For nets in gridless_first with K>2 pads, we use route_net_multipin
            # directly, accumulating copper obstacles as we go (same strategy as
            # probe_order.py: sequential routing in sorted-name order, each net
            # sees already-routed copper from the earlier nets in this pass).
            #
            # Hard invariant: this block is a no-op unless gridless_first is set
            # (checked by `if gridless_first:`) so gridless_negotiate=True without
            # gridless_first does NOT trigger this path.
            # ------------------------------------------------------------------
            if gridless_first:
                import time as _mp_time

                from tracewise.route.gridless.route import route_net_multipin

                # Build world-mm pad lookup by net name
                _pads_by_net: dict[str, list[dict]] = {}
                for _p in pads:
                    _pads_by_net.setdefault(_p["net"], []).append(_p)

                # Window parameters for bounded fast path (Probe-Order values)
                _gf_max_window = (
                    neg_max_classify_window
                    if neg_max_classify_window is not None
                    else 25.0
                )
                _gf_base_window = 8.0  # BASE_WINDOW_MM from probe_order.py

                # Accumulated Shapely obstacles: seed with already-routed 2-pin
                # nets' copper so multi-pin nets route around them.
                _track_mm_gf = geo.get("track_mm", 0.2)
                _clr_mm_gf = geo.get("clearance_mm", 0.2)
                # Correct inflation for track-to-track centerline distance:
                # old_track_mm/2 + clearance_mm + new_track_mm/2 = track_mm + clearance_mm
                # (FIX: was track_mm/2 + clearance_mm — under-inflated by track_mm/2)
                _inflate_gf = _track_mm_gf + _clr_mm_gf
                _via_mm_gf = geo.get("via_mm", 0.6)
                # _gf_extra_obstacles: all copper (F.Cu + B.Cu) from previously
                #   routed gridless-first nets.  Used for standard multi-pin
                #   routing and B.Cu run routing (prevents same-layer crossings).
                # _gf_fcu_obstacles: F.Cu-only subset.  Used for fanout-escape
                #   F.Cu stub routing.  B.Cu paths must NOT be included here
                #   because the 2D Shapely free-space is layer-unaware, and a
                #   B.Cu obstacle would wrongly block F.Cu stub routing even
                #   though the copper is physically on a different layer.
                _gf_extra_obstacles: list = []
                _gf_fcu_obstacles: list = []
                try:
                    from shapely.geometry import LineString as _LStr
                    from shapely.geometry import Point as _Pt

                    from tracewise.route.gridless.geom import (  # noqa: I001
                        snap as _gf_snap,
                    )
                    _via_inflate_init = 0.0  # computed below after use
                    for _rname, _rnr in results.items():
                        if hasattr(_rnr, "world_paths") and _rnr.world_paths:
                            for _wp in _rnr.world_paths:
                                if len(_wp) >= 2:
                                    _pts2d = [(_w[0], _w[1]) for _w in _wp]
                                    try:
                                        _ls = _gf_snap(
                                            _LStr(_pts2d).buffer(_inflate_gf, cap_style=2)
                                        )
                                        _gf_extra_obstacles.append(_ls)
                                    except Exception:  # noqa: BLE001
                                        pass
                                    # F.Cu-only subset for fanout stub routing
                                    _fcu_init: list[tuple[float, float]] = []
                                    for _wpt_i in _wp:
                                        _wl_i = _wpt_i[2] if len(_wpt_i) == 3 else 0
                                        if _wl_i == 0:
                                            _fcu_init.append((_wpt_i[0], _wpt_i[1]))
                                        else:
                                            if len(_fcu_init) >= 2:
                                                try:
                                                    _ls_f = _gf_snap(
                                                        _LStr(_fcu_init).buffer(
                                                            _inflate_gf, cap_style=2
                                                        )
                                                    )
                                                    _gf_fcu_obstacles.append(_ls_f)
                                                except Exception:  # noqa: BLE001
                                                    pass
                                            _fcu_init = []
                                    if len(_fcu_init) >= 2:
                                        try:
                                            _ls_f = _gf_snap(
                                                _LStr(_fcu_init).buffer(
                                                    _inflate_gf, cap_style=2
                                                )
                                            )
                                            _gf_fcu_obstacles.append(_ls_f)
                                        except Exception:  # noqa: BLE001
                                            pass
                        if hasattr(_rnr, "world_vias") and _rnr.world_vias:
                            # via_mm/2 + clearance_mm + track_mm/2 (FIX: was via_mm/2 + clr)
                            _via_inflate = _via_mm_gf / 2.0 + _clr_mm_gf + _track_mm_gf / 2.0
                            for _vx, _vy in _rnr.world_vias:
                                try:
                                    _vc = _gf_snap(
                                        _Pt(_vx, _vy).buffer(_via_inflate, resolution=16)
                                    )
                                    _gf_extra_obstacles.append(_vc)
                                    _gf_fcu_obstacles.append(_vc)  # via: both layers
                                except Exception:  # noqa: BLE001
                                    pass
                except Exception:  # noqa: BLE001 — Shapely absent or obstacle build failed
                    pass

                _drill_centers_gf = gk.get("drill_centers") or []

                # Route multi-pin nets in sorted order (deterministic)
                # Pre-compute dense components once (not per-net) for efficiency.
                from tracewise.route.gridless.geom import detect_dense_components
                from tracewise.route.gridless.route import route_net_fanout_escape
                _gf_dense_comps = detect_dense_components(pads)
                _gf_dense_by_ref: dict[str, dict] = {
                    d["ref"]: d for d in _gf_dense_comps
                }

                for _mp_net_name in sorted(gridless_nets):
                    _mp_net = by_name_all.get(_mp_net_name)
                    if _mp_net is None or len(_mp_net.pads) < 3:
                        continue  # 2-pin nets handled above; K<2 never routes
                    if _mp_net_name in _gridless_pre_routed:
                        continue  # already handled (shouldn't happen for K>2 nets)

                    _mp_pads_world = _pads_by_net.get(_mp_net_name, [])
                    if len(_mp_pads_world) < 2:
                        _gridless_pre_routed.add(_mp_net_name)
                        results[_mp_net_name] = NetRoute(
                            net=_mp_net, ok=False,
                            reason="multipin: insufficient world pads",
                        )
                        continue

                    _mp_t0 = _mp_time.perf_counter()

                    # ----------------------------------------------------------
                    # Multi-pin fanout-escape: if exactly one pad belongs to a
                    # dense QFN component, route that pad via the guided escape
                    # strategy first, then route the remaining pads via the
                    # standard multi-pin MST.
                    # ----------------------------------------------------------
                    _mp_all_paths: list = []
                    _mp_all_vias: list = []
                    # _mp_extra_obstacles: full set (F.Cu + B.Cu) for general routing.
                    # _mp_fcu_obstacles: F.Cu-only subset for fanout stub routing
                    #   (fanout uses 2D Shapely free-space, so B.Cu obstacles must
                    #   be excluded to avoid wrongly blocking F.Cu paths).
                    _mp_extra_obstacles = list(_gf_extra_obstacles)
                    _mp_fcu_obstacles = list(_gf_fcu_obstacles)
                    _mp_fanout_done = False

                    if _gf_dense_by_ref:
                        # Split pads into QFN-source and non-QFN
                        _qfn_pads = [
                            _p for _p in _mp_pads_world
                            if _p.get("ref", "") in _gf_dense_by_ref
                               and _p.get("front") and not _p.get("back")
                        ]
                        _non_qfn_pads = [
                            _p for _p in _mp_pads_world
                            if _p not in _qfn_pads
                        ]
                        # Strategy applies when there is EXACTLY ONE QFN source pad
                        # AND at least one through-hole (back=True) non-QFN pad.
                        # The B.Cu run from the escape via must land on a through-
                        # hole pad (copper on both layers).  SMD-only destinations
                        # cannot terminate a B.Cu run, so those nets fall back to
                        # standard multi-pin F.Cu routing.
                        _th_non_qfn_pads = [
                            _p for _p in _non_qfn_pads
                            if _p.get("back")  # through-hole: copper on both layers
                        ]
                        if len(_qfn_pads) == 1 and len(_th_non_qfn_pads) >= 1:
                            import math as _fe_math
                            _qfn_p = _qfn_pads[0]
                            _qfn_xy = (_qfn_p["x"], _qfn_p["y"])
                            _dcomp_fe = _gf_dense_by_ref[_qfn_p["ref"]]
                            # Nearest through-hole pad as B.Cu run destination.
                            # Through-hole pads have copper on B.Cu, so the B.Cu
                            # run from the escape via can land there legally.
                            # Using B.Cu for the long leg keeps F.Cu free for
                            # non-target GPIO nets routing in the same corridor.
                            _non_qfn_sorted = sorted(
                                _th_non_qfn_pads,
                                key=lambda _p: _fe_math.hypot(
                                    _p["x"] - _qfn_xy[0], _p["y"] - _qfn_xy[1]
                                ),
                            )
                            _nearest_dest = _non_qfn_sorted[0]
                            _nearest_xy = (_nearest_dest["x"], _nearest_dest["y"])
                            # Full two-layer mode: F.Cu stub + B.Cu run to dest.
                            # Two separate obstacle lists are used:
                            # - extra_obstacles=_mp_fcu_obstacles: F.Cu only.
                            #     Passed to the F.Cu stub search.  B.Cu paths
                            #     excluded: Shapely 2D free-space is layer-
                            #     unaware, so B.Cu obstacles would wrongly
                            #     block the F.Cu stub on a different layer.
                            # - bcu_extra_obstacles=_mp_extra_obstacles: all.
                            #     Passed to the B.Cu run search.  Includes
                            #     B.Cu paths from previous target nets so the
                            #     new B.Cu run doesn't cross them.
                            result_fe = route_net_fanout_escape(
                                source_xy=_qfn_xy,
                                component_cx=_dcomp_fe["cx"],
                                component_cy=_dcomp_fe["cy"],
                                ring_radius=_dcomp_fe["ring_radius"],
                                pads=pads,
                                net_name=_mp_net_name,
                                geo=geo,
                                board_bbox=board_bbox,
                                extra_obstacles=_mp_fcu_obstacles,
                                board_outline=neg_board_outline,
                                drill_obstacles=neg_drill_obstacles or [],
                                drill_centers=_drill_centers_gf,
                                max_window_mm=_gf_max_window,
                                dest_xy=_nearest_xy,  # B.Cu run to TH connector
                                bcu_extra_obstacles=_mp_extra_obstacles,
                            )
                            if result_fe.ok and result_fe.world_paths:
                                _mp_all_paths.extend(result_fe.world_paths)
                                _mp_all_vias.extend(result_fe.world_vias or [])
                                _mp_fanout_done = True
                                # Accumulate fanout route as obstacles for
                                # subsequent target-net routing in this pass.
                                #
                                # Two separate lists are maintained:
                                # - _mp_extra_obstacles: ALL copper (F.Cu + B.Cu)
                                #     Used for standard multi-pin and for the
                                #     B.Cu run in route_net_fanout_escape.
                                #     Prevents B.Cu-to-B.Cu crossing between
                                #     different target nets' escape routes.
                                # - _mp_fcu_obstacles: F.Cu ONLY
                                #     Passed to route_net_fanout_escape as
                                #     extra_obstacles for the F.Cu stub search.
                                #     B.Cu paths excluded because the Shapely
                                #     2D free-space is layer-unaware; including
                                #     B.Cu obstacles would wrongly block the
                                #     F.Cu stub on a different physical layer.
                                #
                                # Escape vias are added to BOTH (through-hole
                                # copper exists on both layers).
                                try:
                                    from shapely.geometry import LineString as _LStrFe
                                    from shapely.geometry import Point as _PtFe

                                    from tracewise.route.gridless.geom import (
                                        snap as _snap_fe,
                                    )
                                    for _fewp in result_fe.world_paths:
                                        if len(_fewp) < 2:
                                            continue
                                        # Add LAYER-SEPARATED sub-segments to obstacle
                                        # lists.  The B.Cu free-space for subsequent
                                        # nets must NOT include F.Cu-only copper, as the
                                        # 2D Shapely free-space is layer-unaware.  A
                                        # combined 2D projection (F.Cu stub + B.Cu run)
                                        # would wrongly block B.Cu routing in areas
                                        # where only F.Cu copper physically exists.
                                        #
                                        # _mp_extra_obstacles: B.Cu sub-segments ONLY.
                                        #   Used as bcu_extra_obstacles for the B.Cu
                                        #   run in the next fanout call — prevents
                                        #   B.Cu-to-B.Cu crossing.
                                        # _mp_fcu_obstacles: F.Cu sub-segments ONLY.
                                        #   Used as extra_obstacles for the F.Cu stub
                                        #   routing — prevents F.Cu-to-F.Cu crossing.
                                        _fcu_seg: list[tuple[float, float]] = []
                                        _bcu_seg: list[tuple[float, float]] = []
                                        for _fewpt in _fewp:
                                            _wp_layer = (
                                                _fewpt[2] if len(_fewpt) == 3 else 0
                                            )
                                            if _wp_layer == 0:
                                                _fcu_seg.append(
                                                    (_fewpt[0], _fewpt[1])
                                                )
                                            else:
                                                # Flush any accumulated F.Cu segment
                                                if len(_fcu_seg) >= 2:
                                                    try:
                                                        _fe_ls_f = _snap_fe(
                                                            _LStrFe(_fcu_seg).buffer(
                                                                _inflate_gf, cap_style=2
                                                            )
                                                        )
                                                        _mp_fcu_obstacles.append(_fe_ls_f)
                                                    except Exception:  # noqa: BLE001
                                                        pass
                                                    _fcu_seg = []
                                                _bcu_seg.append(
                                                    (_fewpt[0], _fewpt[1])
                                                )
                                        # Flush final F.Cu segment (if path ends on F.Cu)
                                        if len(_fcu_seg) >= 2:
                                            try:
                                                _fe_ls_f = _snap_fe(
                                                    _LStrFe(_fcu_seg).buffer(
                                                        _inflate_gf, cap_style=2
                                                    )
                                                )
                                                _mp_fcu_obstacles.append(_fe_ls_f)
                                            except Exception:  # noqa: BLE001
                                                pass
                                        # Flush B.Cu segment → _mp_extra_obstacles only
                                        if len(_bcu_seg) >= 2:
                                            try:
                                                _fe_ls_b = _snap_fe(
                                                    _LStrFe(_bcu_seg).buffer(
                                                        _inflate_gf, cap_style=2
                                                    )
                                                )
                                                _mp_extra_obstacles.append(_fe_ls_b)
                                            except Exception:  # noqa: BLE001
                                                pass
                                    _via_inf_fe = (
                                        _via_mm_gf / 2.0 + _clr_mm_gf + _track_mm_gf / 2.0
                                    )
                                    for _vxfe, _vyfe in (result_fe.world_vias or []):
                                        try:
                                            _fe_vc = _snap_fe(
                                                _PtFe(_vxfe, _vyfe).buffer(
                                                    _via_inf_fe, resolution=16
                                                )
                                            )
                                            _mp_extra_obstacles.append(_fe_vc)
                                            _mp_fcu_obstacles.append(_fe_vc)
                                        except Exception:  # noqa: BLE001
                                            pass
                                except Exception:  # noqa: BLE001
                                    pass

                    # Now route the remaining non-QFN pads via standard multi-pin.
                    # Fanout-escape (B.Cu mode) connects QFN pad → escape via → nearest
                    # TH connector on B.Cu.  The remaining non-QFN pads just need F.Cu
                    # MST among themselves — the nearest TH pad is already in the set,
                    # so all pads become connected transitively through it.
                    # No fanout or fanout failed: route ALL pads via multi-pin.
                    if _mp_fanout_done:
                        # Only run MST if there are at least 2 non-QFN pads.
                        # (1 pad: nothing to connect; the QFN already connects via B.Cu)
                        _fanout_mst_ok = False
                        if len(_non_qfn_pads) >= 2:
                            result_rest = route_net_multipin(
                                pads_of_net=_non_qfn_pads,
                                net_name=_mp_net_name,
                                all_pads=pads,
                                geo=geo,
                                board_bbox=board_bbox,
                                extra_obstacles=_mp_extra_obstacles,
                                window_mm=_gf_base_window,
                                board_outline=neg_board_outline,
                                drill_obstacles=neg_drill_obstacles,
                                drill_centers=_drill_centers_gf,
                                max_window_mm=_gf_max_window,
                                allow_via=False,
                            )
                            # Only accept a FULLY connected MST: if any sub-edge
                            # failed (ok=False), the net is partially routed and
                            # we discard the result.  A partial stub + partial MST
                            # would mark the net as gridless-done and exclude it
                            # from grid routing, leaving it permanently unconnected.
                            if result_rest.ok and result_rest.world_paths:
                                _mp_all_paths.extend(result_rest.world_paths)
                                _mp_all_vias.extend(result_rest.world_vias or [])
                                _fanout_mst_ok = True
                        else:
                            # Only 1 non-QFN pad (the nearest TH): fanout B.Cu run
                            # already connects QFN→via→connector.  MST not needed.
                            _fanout_mst_ok = True
                        if not _fanout_mst_ok:
                            # MST failed: discard fanout stub+B.Cu result and fall back
                            # to standard multi-pin (also likely to fail, sending the
                            # net to the grid for a second chance).
                            _mp_all_paths.clear()
                            _mp_all_vias.clear()
                            _mp_fanout_done = False

                    if not _mp_fanout_done:
                        # No fanout-escape (or fanout failed): standard multi-pin
                        # for all pads.  Disable via search for the clean-board fast
                        # path (via-site search expensive on large free spaces).
                        # Failed sub-edges are skipped (partial tree); nets that
                        # need vias will be handled by the grid fallback.
                        result_mp = route_net_multipin(
                            pads_of_net=_mp_pads_world,
                            net_name=_mp_net_name,
                            all_pads=pads,
                            geo=geo,
                            board_bbox=board_bbox,
                            extra_obstacles=_mp_extra_obstacles,
                            window_mm=_gf_base_window,
                            board_outline=neg_board_outline,
                            drill_obstacles=neg_drill_obstacles,
                            drill_centers=_drill_centers_gf,
                            max_window_mm=_gf_max_window,
                            allow_via=False,
                        )
                        if result_mp.world_paths:
                            _mp_all_paths.extend(result_mp.world_paths)
                            _mp_all_vias.extend(result_mp.world_vias or [])

                    _mp_elapsed = round(_mp_time.perf_counter() - _mp_t0, 3)

                    if _mp_all_paths:
                        # Mark as pre-routed only when copper was actually committed.
                        # Nets with no successful paths are left OUT of
                        # _gridless_pre_routed so the grid router handles them with
                        # their original priority and without being wrongly flagged as
                        # "already attempted by gridless" (which would change their
                        # grid-queue position and introduce non-determinism).
                        _gridless_pre_routed.add(_mp_net_name)
                        from tracewise.route.gridless.adapter import to_gridless_netroute
                        nr = to_gridless_netroute(
                            _mp_net, _mp_all_paths, grid,
                            world_vias=_mp_all_vias,
                        )
                        # Mark multi-pin gridless-first copper into the shared grid
                        # ledger so the grid router routes AROUND it, preventing
                        # tracks_crossing / shorting_items DRC violations.
                        #
                        # Via cells are now correctly routed to via_sites (not cells)
                        # in rasterize_into_grid, so _mark applies via_halfwidth_cells
                        # (larger) inflation for via copper — preventing grid tracks
                        # from entering the via copper ring clearance zone.
                        _mark(grid, nr, 1)
                        results[_mp_net_name] = nr

                        # Accumulate this net's copper as Shapely obstacles for
                        # subsequent multi-pin nets in this pass.
                        # _gf_extra_obstacles: B.Cu sub-segments only — used as
                        #   bcu_extra_obstacles for subsequent B.Cu run routing.
                        #   F.Cu copper is intentionally excluded: combining F.Cu
                        #   and B.Cu into a single 2D projection (layer-unaware)
                        #   blocks B.Cu routing in areas where only F.Cu copper
                        #   physically exists, causing adjacent GPIO fanout to fail.
                        # _gf_fcu_obstacles: F.Cu sub-segments only — used for
                        #   fanout F.Cu stub routing (layer-unaware 2D free-space).
                        try:
                            from shapely.geometry import LineString as _LStr2
                            from shapely.geometry import Point as _Pt2

                            from tracewise.route.gridless.geom import snap as _snap_mp
                            for _wp2 in _mp_all_paths:
                                if len(_wp2) < 2:
                                    continue
                                # Extract layer-separated sub-segments.
                                _fcu2: list[tuple[float, float]] = []
                                _bcu2: list[tuple[float, float]] = []
                                for _pt2 in _wp2:
                                    _l2 = _pt2[2] if len(_pt2) == 3 else 0
                                    if _l2 == 0:
                                        _fcu2.append((_pt2[0], _pt2[1]))
                                    else:
                                        # Flush F.Cu segment
                                        if len(_fcu2) >= 2:
                                            try:
                                                _ls2f = _snap_mp(
                                                    _LStr2(_fcu2).buffer(
                                                        _inflate_gf, cap_style=2
                                                    )
                                                )
                                                _gf_fcu_obstacles.append(_ls2f)
                                            except Exception:  # noqa: BLE001
                                                pass
                                            _fcu2 = []
                                        _bcu2.append((_pt2[0], _pt2[1]))
                                # Flush final F.Cu segment
                                if len(_fcu2) >= 2:
                                    try:
                                        _ls2f = _snap_mp(
                                            _LStr2(_fcu2).buffer(_inflate_gf, cap_style=2)
                                        )
                                        _gf_fcu_obstacles.append(_ls2f)
                                    except Exception:  # noqa: BLE001
                                        pass
                                # B.Cu segment → _gf_extra_obstacles
                                if len(_bcu2) >= 2:
                                    try:
                                        _ls2b = _snap_mp(
                                            _LStr2(_bcu2).buffer(_inflate_gf, cap_style=2)
                                        )
                                        _gf_extra_obstacles.append(_ls2b)
                                    except Exception:  # noqa: BLE001
                                        pass
                            # via_mm/2 + clearance_mm + track_mm/2 (FIX: was via_mm/2+clr)
                            _via_inf2 = _via_mm_gf / 2.0 + _clr_mm_gf + _track_mm_gf / 2.0
                            for _vx2, _vy2 in _mp_all_vias:
                                try:
                                    _vc2 = _snap_mp(
                                        _Pt2(_vx2, _vy2).buffer(_via_inf2, resolution=16)
                                    )
                                    _gf_extra_obstacles.append(_vc2)
                                    _gf_fcu_obstacles.append(_vc2)  # via: both layers
                                except Exception:  # noqa: BLE001
                                    pass
                        except Exception:  # noqa: BLE001
                            pass
                    else:
                        results[_mp_net_name] = NetRoute(
                            net=_mp_net, ok=False,
                            reason="multipin: all paths failed",
                        )

    routed: list[NetRoute] = []
    by_name = {n.name: n for n in nets}
    budget = ripup_factor * max(len(nets), 1)
    history = (np.zeros((grid.layers, grid.ny, grid.nx), np.float64)
               if history_factor > 0.0 else None)

    # Populate routed list from already-committed gridless nets so grid rip-up
    # can select them as victims if needed (staged coexistence: no cross-substrate
    # rip-up in M2 — they are already in grid.hard so the grid router avoids them).
    #
    # gridless_first SUCCESSFULLY routed nets are excluded from routed — they must
    # not be ripped up.  Gridless-first nets are marked into the shared grid ledger
    # (via _mark) so the grid router routes AROUND their copper.  Excluding them
    # from the rip-up pool (routed list) ensures the grid rip-up loop never selects
    # them as victims, keeping their PCB copper intact.
    # gridless_first FAILED nets (ok=False) remain in the grid queue so the grid
    # engine gets a second chance to connect them.
    _gf_fully_routed = {
        name for name in _gridless_pre_routed
        if results.get(name) is not None and results[name].ok
    }
    for _name, nr in results.items():
        # Include only nets that are (a) successfully routed AND (b) not gridless-
        # first fully-routed (those stay out of the rip-up pool).
        if nr.ok and _name not in _gf_fully_routed:
            routed.append(nr)

    # Only exclude FULLY CONNECTED gridless-first nets from the grid queue.
    # Nets that were nominated for gridless-first but failed get a second chance
    # via the grid engine.
    queue = order_nets(
        [n for n in nets if n.name not in _gf_fully_routed], priority
    )
    attempts: dict[str, int] = {}
    while queue and budget > 0:
        net = queue.pop(0)
        budget -= 1
        attempts[net.name] = attempts.get(net.name, 0) + 1
        # main loop stays all-or-nothing so the rip-up dynamics that find FULL
        # solutions are preserved; partial routing is a last-resort salvage pass
        # below (applying it in-loop disabled rip-up for partially-routed nets
        # and cascaded — measured: mitayi 63->72).

        # Gridless dispatch: route designated nets via the visibility-graph
        # engine.  The result is a GridlessNetRoute (IS-A NetRoute) so _mark,
        # rip-up victim selection, and the salvage pass all operate uniformly.
        # Exception: gridless_first nets that FAILED the pre-routing phase are in
        # _gridless_pre_routed but not in _gf_fully_routed.  They reach here via
        # the grid queue (second-chance pass) and must route via the grid engine,
        # not the gridless wrapper — the gridless path already failed for them.
        if gridless_nets and net.name in gridless_nets and net.name not in _gridless_pre_routed:
            nr = _route_net_gridless_wrapped(grid, net, gridless_kwargs or {})
        else:
            nr = route_net(grid, net, via_cost=via_cost, escape=escape,
                           history=history, history_factor=history_factor)
        if nr.ok:
            _mark(grid, nr, 1)
            routed.append(nr)
            results[net.name] = nr
            continue
        if attempts[net.name] <= 10:
            victim = _nearest_victim(net, routed)
            if victim is not None:
                if history is not None:  # the contested corridor gets pricier
                    for layer, iy, ix in victim.cells:
                        history[layer, iy, ix] += 1.0
                _mark(grid, victim, -1)
                routed.remove(victim)
                results.pop(victim.net.name, None)
                queue.insert(0, victim.net)  # victim re-routes after us
                queue.insert(0, net)  # we retry first, with the space freed
                continue
        results[net.name] = nr  # definitive failure, recorded
    reason = "rip-up budget exhausted"
    for net in queue:  # budget exhausted
        results.setdefault(net.name, NetRoute(net=net, reason=reason))

    if allow_partial:
        # SALVAGE PASS: every net that rip-up could not fully route gets one
        # incremental attempt that keeps whatever sub-tree it can build. This
        # recovers high-fanout nets (e.g. +3V0, 58 pads) that the all-or-nothing
        # main loop discarded over a single blocked pad — measured zuluscsi
        # 65->9 — without disturbing the fully-routed nets (it only touches
        # failures, and runs last so its copper blocks nothing).
        for name, nr in list(results.items()):
            if nr.ok:
                continue
            # salvage routes LEGALITY-FIRST by default (salvage_escape=0): connect
            # the pads that have a clean path, skip those that would only fit by
            # shaving clearance — that shaving is the source of the salvage pass's
            # added clearance/short violations.
            pnr = route_net(grid, by_name[name], via_cost=via_cost,
                            escape=salvage_escape,
                            history=history, history_factor=history_factor,
                            allow_partial=True)
            if pnr.ok and pnr.paths:
                _mark(grid, pnr, 1)
                results[name] = pnr

    # ------------------------------------------------------------------
    # M2.1 GRIDLESS RESCUE: route grid failures via gridless, with ALL
    # existing copper (grid tracks + pads + board edge + drills) as
    # obstacles.  Runs AFTER the grid pass + salvage pass so gridless
    # routes in the REAL gaps around grid copper.
    #
    # Hard invariant: gridless_rescue=False (default) → no-op, results
    # unchanged, byte-identical to pre-M2.1 behaviour.
    # ------------------------------------------------------------------
    if gridless_rescue and gridless_kwargs:
        from tracewise.route.gridless.adapter import to_gridless_netroute
        from tracewise.route.gridless.geom import (
            HAVE_SHAPELY,
            net_routes_to_track_obstacles,
            net_routes_to_via_obstacles,
        )
        from tracewise.route.gridless.negotiate import route_gridless_set

        if HAVE_SHAPELY:
            gk = gridless_kwargs
            pads = gk.get("pads")
            geo = gk.get("geo")
            board_bbox = gk.get("board_bbox")
            anchors = gk.get("anchors")
            neg_board_outline = gk.get("board_outline")
            neg_drill_obstacles = gk.get("drill_obstacles")
            neg_geom_threshold = gk.get("negotiate_geom_block_threshold", 0.5)
            neg_ripup_factor = gk.get("negotiate_ripup_factor", ripup_factor)
            neg_history_factor = gk.get("negotiate_history_factor", 3.0)
            neg_window_mm = gk.get("negotiate_window_mm", 4.0)

            if pads is not None and geo is not None and board_bbox is not None:
                by_name_all = {n.name: n for n in nets}

                # --- Build pad lookup by net name (for multi-pin routing) ---
                pads_by_net: dict[str, list[dict]] = {}
                for p in pads:
                    pads_by_net.setdefault(p["net"], []).append(p)

                # --- Identify rescue candidates ---
                # 2-pin F.Cu nets AND multi-pin nets that the grid left
                # unconnected.  Uses sorted() for determinism.
                rescue_candidates: list[dict] = []  # 2-pin only (for route_gridless_set)
                multipin_candidates: list[dict] = []  # K>2 (for route_net_multipin)

                for net_name in sorted(results):
                    nr = results[net_name]
                    if nr.ok:
                        continue
                    net = by_name_all.get(net_name)
                    if net is None:
                        continue

                    n_pads = len(net.pads)

                    if n_pads == 2:
                        # 2-pin: only F.Cu pads
                        if not all(p[0] == 0 for p in net.pads):
                            continue
                        # Resolve world-mm pad coords
                        pad_a_cell = net.pads[0]
                        pad_b_cell = net.pads[1]
                        pad_a = grid.to_world(pad_a_cell[1], pad_a_cell[2])
                        pad_b = grid.to_world(pad_b_cell[1], pad_b_cell[2])
                        if anchors is not None:
                            pad_a = anchors.get(pad_a_cell, pad_a)
                            pad_b = anchors.get(pad_b_cell, pad_b)
                        rescue_candidates.append({
                            "net_name": net_name,
                            "pad_a": pad_a,
                            "pad_b": pad_b,
                        })

                    elif n_pads >= 3:
                        # Multi-pin (M3-P2): include if any pads are F.Cu or
                        # through-hole (front=True).  Mixed-layer multi-pin nets
                        # (some pads B.Cu only) are included — route_net_multipin
                        # is via-capable (2-layer sub-edges).
                        has_front = any(p[0] == 0 for p in net.pads)
                        if not has_front:
                            continue
                        # Build world-mm pad list from the pads_by_net lookup
                        # (exact pad centers, not grid-snapped).
                        net_pads_world = pads_by_net.get(net_name, [])
                        if len(net_pads_world) < 2:
                            continue
                        multipin_candidates.append({
                            "net_name": net_name,
                            "net": net,
                            "pads_world": net_pads_world,
                        })

                # Build track-segment obstacles once (shared by all rescue passes).
                any_candidates = bool(rescue_candidates) or bool(multipin_candidates)
                if any_candidates:
                    track_obs = net_routes_to_track_obstacles(
                        results,
                        grid,
                        track_mm=geo.get("track_mm", 0.2),
                        clearance_mm=geo.get("clearance_mm", 0.2),
                    )
                    # F.Cu-only track obstacles: used as extra_obstacles for the
                    # F.Cu stub in fanout-escape rescue so the stub avoids F.Cu
                    # grid copper without projecting B.Cu tracks as F.Cu obstacles.
                    fcu_track_obs = net_routes_to_track_obstacles(
                        results,
                        grid,
                        track_mm=geo.get("track_mm", 0.2),
                        clearance_mm=geo.get("clearance_mm", 0.2),
                        layer=0,
                    )
                    # B.Cu-only track obstacles: used as B.Cu-run seed for the
                    # fanout-escape rescue so that F.Cu grid copper is NOT
                    # projected as a B.Cu obstacle (2D projection of F.Cu tracks
                    # would block nearly all B.Cu routing on a dense board).
                    bcu_track_obs = net_routes_to_track_obstacles(
                        results,
                        grid,
                        track_mm=geo.get("track_mm", 0.2),
                        clearance_mm=geo.get("clearance_mm", 0.2),
                        layer=1,
                    )
                    # Routing-via obstacles: grid-router-placed vias are stored
                    # in nr.via_sites and are NOT in extract_drill_obstacles
                    # (which only reads the KiCad file, not routing-time state).
                    # Their annular rings block BOTH layers, so add them to BOTH
                    # the B.Cu and F.Cu obstacle seeds to prevent rescue tracks
                    # from shorting against via copper.
                    via_obs = net_routes_to_via_obstacles(
                        results,
                        grid,
                        via_mm=geo.get("via_mm", 0.6),
                        clearance_mm=geo.get("clearance_mm", 0.2),
                        track_mm=geo.get("track_mm", 0.2),
                    )
                    # Combined obstacles = drill holes + grid/gridless track copper
                    combined_drill_obs = list(neg_drill_obstacles or []) + track_obs
                else:
                    track_obs = []
                    fcu_track_obs = []
                    bcu_track_obs = []
                    via_obs = []
                    combined_drill_obs = []

                # ------------------------------------------------------------------
                # 2-PIN RESCUE: route_gridless_set (M2.1 / M3-P1 unchanged)
                # ------------------------------------------------------------------
                if rescue_candidates:
                    rescue_results = route_gridless_set(
                        net_set=rescue_candidates,
                        pads=pads,
                        geo=geo,
                        board_bbox=board_bbox,
                        anchors=anchors,
                        history_factor=neg_history_factor,
                        ripup_factor=neg_ripup_factor,
                        window_mm_start=neg_window_mm,
                        geom_block_threshold=neg_geom_threshold,
                        board_outline=neg_board_outline,
                        drill_obstacles=combined_drill_obs,
                    )

                    # M3: geometry-blocked nets get a direct 2-layer via rescue
                    # attempt via route_net_gridless(allow_via=True).
                    geom_blocked_candidates = []

                    for net_name, neg_res in rescue_results.items():
                        net = by_name_all.get(net_name)
                        if net is None:
                            continue
                        if neg_res.ok and neg_res.world_paths:
                            nr = to_gridless_netroute(
                                net, neg_res.world_paths, grid,
                                world_vias=neg_res.world_vias,
                            )
                            _mark(grid, nr, 1)
                            results[net_name] = nr
                        elif neg_res.status == "geometry_blocked":
                            # Collect for 2-layer via rescue below
                            entry = next(
                                (c for c in rescue_candidates
                                 if c["net_name"] == net_name),
                                None,
                            )
                            if entry is not None:
                                geom_blocked_candidates.append(entry)

                    # M3 2-layer pass: for geometry-blocked 2-pin nets, try two
                    # strategies in order:
                    #   1. Fanout-escape (guided via for dense QFN pads) — when the
                    #      source pad belongs to a dense component. This is the
                    #      validated mechanism from the gridless_first path, now
                    #      wired into the rescue path (DISPLACEMENT CONTROL build).
                    #      B.Cu is occupied post-grid, so pass full track_obs as
                    #      bcu_extra_obstacles; F.Cu stub uses drill_obs only (stub
                    #      is short — inside the pad ring, grid tracks unlikely there).
                    #   2. Standard route_net_gridless(allow_via=True) fallback.
                    if geom_blocked_candidates:
                        from tracewise.route.gridless.geom import detect_dense_components
                        from tracewise.route.gridless.route import (
                            route_net_fanout_escape,
                            route_net_gridless,
                        )

                        drill_centers_2l = gk.get("drill_centers") or []

                        # Pre-compute dense components once for all candidates.
                        _rescue_dense_comps = detect_dense_components(pads)
                        _rescue_dense_by_ref: dict[str, dict] = {
                            d["ref"]: d for d in _rescue_dense_comps
                        }

                        # Build pad lookup by net name (for source/dest detection)
                        _rescue_pads_by_net: dict[str, list[dict]] = {}
                        for _rp in pads:
                            _rescue_pads_by_net.setdefault(
                                _rp.get("net", ""), []
                            ).append(_rp)

                        # Accumulated Shapely obstacles from fanout-rescue nets
                        # committed in this pass (prevents B.Cu crossing between
                        # successive rescue routes).  Seeded with bcu_track_obs
                        # (B.Cu-only grid copper) + via_obs (routing-time vias not
                        # in extract_drill_obstacles) so B.Cu runs avoid actual
                        # B.Cu grid tracks and grid-router via annular rings.
                        _rescue_bcu_obstacles: list = list(bcu_track_obs) + list(via_obs)

                        for entry in geom_blocked_candidates:
                            net_name = entry["net_name"]
                            net = by_name_all.get(net_name)
                            if net is None:
                                continue
                            if results.get(net_name) is not None and results[net_name].ok:
                                continue  # already rescued by a prior strategy

                            _pa = entry["pad_a"]
                            _pb = entry["pad_b"]

                            # --- Strategy 1: Fanout-escape ---
                            # Applicable when one pad of this net is an SMD F.Cu
                            # pad on a dense component (QFN ring).
                            _result_fanout = None
                            if _rescue_dense_by_ref:
                                _net_pads_w = _rescue_pads_by_net.get(net_name, [])
                                _src_pad_w = None
                                _dst_pad_w = None
                                for _rp2 in _net_pads_w:
                                    _ref2 = _rp2.get("ref", "")
                                    if _ref2 in _rescue_dense_by_ref:
                                        _src_pad_w = _rp2
                                    else:
                                        _dst_pad_w = _rp2
                                # Source must be SMD F.Cu (front only); dest can be
                                # through-hole (back=True) or any F.Cu target.
                                if (
                                    _src_pad_w is not None
                                    and _dst_pad_w is not None
                                    and _src_pad_w.get("front")
                                    and not _src_pad_w.get("back")
                                ):
                                    _dcomp_r = _rescue_dense_by_ref[_src_pad_w["ref"]]
                                    _src_xy = (_src_pad_w["x"], _src_pad_w["y"])
                                    _dst_xy = (_dst_pad_w["x"], _dst_pad_w["y"])
                                    # Full two-layer mode: F.Cu stub + B.Cu run.
                                    # F.Cu stub: F.Cu-only track obstacles + drills
                                    #   so the stub avoids F.Cu grid copper without
                                    #   treating B.Cu tracks as F.Cu obstacles.
                                    # Via placement (drill_obstacles): drills only —
                                    #   bcu_extra_obstacles is now used for B.Cu via
                                    #   placement free space in route_net_fanout_escape
                                    #   so passing F.Cu tracks in extra_obstacles no
                                    #   longer blocks the B.Cu via placement.
                                    # B.Cu run: B.Cu-only grid copper + rescue
                                    #   copper (avoids real B.Cu tracks without
                                    #   being blocked by F.Cu projections).
                                    _2l_drill_obs = list(
                                        neg_drill_obstacles or []
                                    )
                                    # F.Cu stub: F.Cu tracks + routing vias + drills.
                                    # via_obs are layer-agnostic circles — add to F.Cu
                                    # stub obstacles so the stub avoids via pads too.
                                    _2l_fcu_obs = (
                                        list(fcu_track_obs) + list(via_obs) + _2l_drill_obs
                                    )
                                    _result_fanout = route_net_fanout_escape(
                                        source_xy=_src_xy,
                                        dest_xy=_dst_xy,
                                        component_cx=_dcomp_r["cx"],
                                        component_cy=_dcomp_r["cy"],
                                        ring_radius=_dcomp_r["ring_radius"],
                                        pads=pads,
                                        net_name=net_name,
                                        geo=geo,
                                        board_bbox=board_bbox,
                                        # F.Cu via placement: drills only
                                        # (bcu_extra_obstacles handles B.Cu side)
                                        extra_obstacles=_2l_drill_obs,
                                        board_outline=neg_board_outline,
                                        drill_obstacles=_2l_drill_obs,
                                        drill_centers=drill_centers_2l,
                                        # B.Cu run: B.Cu-only grid + rescue copper
                                        bcu_extra_obstacles=_rescue_bcu_obstacles,
                                        # F.Cu stub: F.Cu tracks + drills
                                        # (separate from via placement obstacles)
                                        fcu_stub_extra_obstacles=_2l_fcu_obs,
                                    )

                            if (
                                _result_fanout is not None
                                and _result_fanout.ok
                                and _result_fanout.world_paths
                            ):
                                nr = to_gridless_netroute(
                                    net, _result_fanout.world_paths, grid,
                                    world_vias=_result_fanout.world_vias,
                                )
                                _mark(grid, nr, 1)
                                results[net_name] = nr
                                # Accumulate B.Cu copper from this rescue as
                                # obstacle for subsequent rescue B.Cu runs.
                                try:
                                    from shapely.geometry import LineString as _LStrRe
                                    from shapely.geometry import Point as _PtRe

                                    from tracewise.route.gridless.geom import (
                                        snap as _snap_re,
                                    )
                                    _re_inflate = (
                                        geo.get("track_mm", 0.2)
                                        + geo.get("clearance_mm", 0.2)
                                    )
                                    _re_via_mm = geo.get("via_mm", 0.6)
                                    _re_clr = geo.get("clearance_mm", 0.2)
                                    _re_tr = geo.get("track_mm", 0.2)
                                    _re_via_inflate = (
                                        _re_via_mm / 2.0 + _re_clr + _re_tr / 2.0
                                    )
                                    for _re_wp in _result_fanout.world_paths:
                                        if len(_re_wp) < 2:
                                            continue
                                        # Only accumulate B.Cu sub-segments into
                                        # _rescue_bcu_obstacles; F.Cu segments on
                                        # a different physical layer must not block
                                        # B.Cu routing in the layer-unaware 2D
                                        # free-space.
                                        _re_bcu: list[tuple[float, float]] = []
                                        for _re_pt in _re_wp:
                                            _re_l = (
                                                _re_pt[2] if len(_re_pt) == 3 else 0
                                            )
                                            if _re_l == 1:  # B.Cu
                                                _re_bcu.append(
                                                    (_re_pt[0], _re_pt[1])
                                                )
                                            elif _re_bcu:
                                                # Flush B.Cu segment
                                                if len(_re_bcu) >= 2:
                                                    try:
                                                        _re_ls = _snap_re(
                                                            _LStrRe(_re_bcu).buffer(
                                                                _re_inflate, cap_style=2
                                                            )
                                                        )
                                                        _rescue_bcu_obstacles.append(
                                                            _re_ls
                                                        )
                                                    except Exception:  # noqa: BLE001
                                                        pass
                                                _re_bcu = []
                                        # Flush final B.Cu segment
                                        if len(_re_bcu) >= 2:
                                            try:
                                                _re_ls = _snap_re(
                                                    _LStrRe(_re_bcu).buffer(
                                                        _re_inflate, cap_style=2
                                                    )
                                                )
                                                _rescue_bcu_obstacles.append(_re_ls)
                                            except Exception:  # noqa: BLE001
                                                pass
                                    for _re_vx, _re_vy in (
                                        _result_fanout.world_vias or []
                                    ):
                                        try:
                                            _re_vc = _snap_re(
                                                _PtRe(_re_vx, _re_vy).buffer(
                                                    _re_via_inflate, resolution=16
                                                )
                                            )
                                            _rescue_bcu_obstacles.append(_re_vc)
                                        except Exception:  # noqa: BLE001
                                            pass
                                except Exception:  # noqa: BLE001
                                    pass
                                continue  # net rescued — skip Strategy 2

                            # --- Strategy 2: Standard 2-layer fallback ---
                            result_2l = route_net_gridless(
                                pad_a=_pa,
                                pad_b=_pb,
                                pads=pads,
                                net_name=net_name,
                                geo=geo,
                                board_bbox=board_bbox,
                                extra_obstacles=track_obs,
                                board_outline=neg_board_outline,
                                drill_obstacles=combined_drill_obs,
                                drill_centers=drill_centers_2l,
                                allow_via=True,
                            )
                            if result_2l.ok and result_2l.world_paths:
                                nr = to_gridless_netroute(
                                    net, result_2l.world_paths, grid,
                                    world_vias=result_2l.world_vias,
                                )
                                _mark(grid, nr, 1)
                                results[net_name] = nr

                # ------------------------------------------------------------------
                # MULTI-PIN RESCUE (M3-P2): route_net_multipin for K>2 nets
                # Routes in deterministic sorted order.
                # Hard invariant: if multipin_candidates is empty (no multi-pin
                # failures, or rescue=False), this block is a no-op.
                # ------------------------------------------------------------------
                if multipin_candidates:
                    from tracewise.route.gridless.geom import detect_dense_components
                    from tracewise.route.gridless.route import (
                        route_net_fanout_escape,
                        route_net_multipin,
                    )

                    drill_centers_mp = gk.get("drill_centers") or []

                    # Pre-compute dense components once for all multipin candidates.
                    _mp_rescue_dense_comps = detect_dense_components(pads)
                    _mp_rescue_dense_by_ref: dict[str, dict] = {
                        d["ref"]: d for d in _mp_rescue_dense_comps
                    }

                    # Accumulated B.Cu Shapely obstacles from fanout-rescue passes
                    # already committed in the 2-pin geom-blocked pass above.
                    # Seeded with bcu_track_obs (B.Cu-only grid copper) + via_obs
                    # (routing-time vias absent from extract_drill_obstacles) so
                    # B.Cu runs avoid real B.Cu grid tracks and via annular rings.
                    # Also absorb any B.Cu copper committed by the 2-pin fanout
                    # rescue block above (stored in _rescue_bcu_obstacles).
                    _mp_rescue_bcu_obs: list = list(bcu_track_obs) + list(via_obs)
                    # Carry over newly-placed B.Cu copper from 2-pin fanout block.
                    # _rescue_bcu_obstacles is only defined when geom_blocked_candidates
                    # existed; guard with a try to avoid NameError.
                    try:
                        _base_bcu_ids = set(id(o) for o in bcu_track_obs)
                        for _o in _rescue_bcu_obstacles:  # noqa: F821
                            if id(_o) not in _base_bcu_ids:
                                _mp_rescue_bcu_obs.append(_o)
                    except NameError:
                        pass  # No 2-pin fanout rescues ran; nothing to carry over.

                    for entry in multipin_candidates:
                        net_name = entry["net_name"]
                        net = entry["net"]
                        pads_world = entry["pads_world"]

                        # Skip if already rescued by the 2-pin path (shouldn't
                        # happen for K>2 entries, but guard for safety).
                        if results.get(net_name) is not None and results[net_name].ok:
                            continue

                        _mp_all_paths: list = []
                        _mp_all_vias: list = []
                        _mp_fanout_done = False

                        # ----------------------------------------------------------
                        # Strategy 1: Fanout-escape for nets with one QFN source pad
                        # Mirrors the gridless_first multipin fanout path, but with
                        # grid track copper as B.Cu obstacles (post-grid rescue).
                        # F.Cu stub: only drill obstacles (stub stays near pad ring).
                        # B.Cu run: _mp_rescue_bcu_obs = track_obs + prior rescues.
                        # ----------------------------------------------------------
                        if _mp_rescue_dense_by_ref:
                            _qfn_mp = [
                                _p for _p in pads_world
                                if _p.get("ref", "") in _mp_rescue_dense_by_ref
                                   and _p.get("front") and not _p.get("back")
                            ]
                            _non_qfn_mp = [
                                _p for _p in pads_world if _p not in _qfn_mp
                            ]
                            # Strategy applies when there is EXACTLY ONE QFN pad and
                            # at least one through-hole pad as destination.
                            _th_non_qfn_mp = [
                                _p for _p in _non_qfn_mp if _p.get("back")
                            ]
                            if len(_qfn_mp) == 1 and len(_th_non_qfn_mp) >= 1:
                                import math as _mp_math
                                _qfn_p_mp = _qfn_mp[0]
                                _qfn_xy_mp = (_qfn_p_mp["x"], _qfn_p_mp["y"])
                                _dcomp_mp = _mp_rescue_dense_by_ref[_qfn_p_mp["ref"]]
                                # Try through-hole pads as B.Cu run destinations
                                # sorted nearest-first; stop at first success so
                                # far TH pads are tried when the nearest is blocked.
                                _th_sorted_mp = sorted(
                                    _th_non_qfn_mp,
                                    key=lambda _p: _mp_math.hypot(
                                        _p["x"] - _qfn_xy_mp[0],
                                        _p["y"] - _qfn_xy_mp[1],
                                    ),
                                )
                                # F.Cu stub: F.Cu-only track obstacles + drills
                                #   so the stub avoids F.Cu grid copper without
                                #   treating B.Cu tracks as F.Cu obstacles.
                                # Via placement (drill_obstacles): drills only.
                                #   bcu_extra_obstacles is used for B.Cu via
                                #   placement inside route_net_fanout_escape so
                                #   F.Cu tracks in extra_obstacles no longer block
                                #   the escape-via placement.
                                # B.Cu run: _mp_rescue_bcu_obs (B.Cu-only grid
                                #   copper + prior rescue B.Cu runs).
                                _fanout_drill_obs = list(neg_drill_obstacles or [])
                                # F.Cu stub: F.Cu tracks + routing vias + drills.
                                _fanout_fcu_obs = (
                                    list(fcu_track_obs) + list(via_obs) + _fanout_drill_obs
                                )
                                result_fe_mp = None
                                for _try_th_mp in _th_sorted_mp:
                                    _try_xy_mp = (
                                        _try_th_mp["x"], _try_th_mp["y"]
                                    )
                                    _fe_attempt = route_net_fanout_escape(
                                        source_xy=_qfn_xy_mp,
                                        dest_xy=_try_xy_mp,
                                        component_cx=_dcomp_mp["cx"],
                                        component_cy=_dcomp_mp["cy"],
                                        ring_radius=_dcomp_mp["ring_radius"],
                                        pads=pads,
                                        net_name=net_name,
                                        geo=geo,
                                        board_bbox=board_bbox,
                                        # F.Cu via placement: drills only
                                        extra_obstacles=_fanout_drill_obs,
                                        board_outline=neg_board_outline,
                                        drill_obstacles=_fanout_drill_obs,
                                        drill_centers=drill_centers_mp,
                                        # B.Cu run: B.Cu-only grid copper + prior
                                        bcu_extra_obstacles=_mp_rescue_bcu_obs,
                                        # F.Cu stub: F.Cu tracks + drills
                                        fcu_stub_extra_obstacles=_fanout_fcu_obs,
                                        # Via placement + F.Cu stub window: 40 mm
                                        # allows generous via search around the QFN
                                        # ring (actual cap = max(ring+3,8) ≤ 8 mm).
                                        max_window_mm=40.0,
                                        # B.Cu run window: tight cap prevents O(n²)
                                        # unary_union on ~814 B.Cu track obstacles.
                                        # 8 mm extends 8 mm beyond the via→TH bbox,
                                        # covering the Manhattan L-paths that the
                                        # router tries first.  J1→J3 span ~26 mm so
                                        # the window is ≤ (26+16)mm wide — well under
                                        # board width — and captures ≤ ~50 segments.
                                        max_bcu_window_mm=8.0,
                                    )
                                    if _fe_attempt.ok and _fe_attempt.world_paths:
                                        result_fe_mp = _fe_attempt
                                        break  # first successful TH destination wins
                                if (
                                    result_fe_mp is not None
                                    and result_fe_mp.ok
                                    and result_fe_mp.world_paths
                                ):
                                    _mp_all_paths.extend(result_fe_mp.world_paths)
                                    _mp_all_vias.extend(
                                        result_fe_mp.world_vias or []
                                    )
                                    _mp_fanout_done = True
                                    # Accumulate B.Cu copper for subsequent rescues
                                    try:
                                        from shapely.geometry import (
                                            LineString as _LStrMpR,
                                        )
                                        from shapely.geometry import Point as _PtMpR

                                        from tracewise.route.gridless.geom import (
                                            snap as _snap_mp_r,
                                        )
                                        _mp_r_inflate = (
                                            geo.get("track_mm", 0.2)
                                            + geo.get("clearance_mm", 0.2)
                                        )
                                        _mp_r_via_mm = geo.get("via_mm", 0.6)
                                        _mp_r_clr = geo.get("clearance_mm", 0.2)
                                        _mp_r_tr = geo.get("track_mm", 0.2)
                                        _mp_r_via_inf = (
                                            _mp_r_via_mm / 2.0
                                            + _mp_r_clr + _mp_r_tr / 2.0
                                        )
                                        for _mp_r_wp in result_fe_mp.world_paths:
                                            if len(_mp_r_wp) < 2:
                                                continue
                                            _bcu_mp_r: list = []
                                            for _mp_r_pt in _mp_r_wp:
                                                _mp_r_l = (
                                                    _mp_r_pt[2]
                                                    if len(_mp_r_pt) == 3 else 0
                                                )
                                                if _mp_r_l == 1:
                                                    _bcu_mp_r.append(
                                                        (_mp_r_pt[0], _mp_r_pt[1])
                                                    )
                                                elif _bcu_mp_r:
                                                    if len(_bcu_mp_r) >= 2:
                                                        try:
                                                            _mp_r_ls = _snap_mp_r(
                                                                _LStrMpR(
                                                                    _bcu_mp_r
                                                                ).buffer(
                                                                    _mp_r_inflate,
                                                                    cap_style=2,
                                                                )
                                                            )
                                                            _mp_rescue_bcu_obs.append(
                                                                _mp_r_ls
                                                            )
                                                        except Exception:  # noqa: BLE001
                                                            pass
                                                    _bcu_mp_r = []
                                            if len(_bcu_mp_r) >= 2:
                                                try:
                                                    _mp_r_ls = _snap_mp_r(
                                                        _LStrMpR(_bcu_mp_r).buffer(
                                                            _mp_r_inflate, cap_style=2
                                                        )
                                                    )
                                                    _mp_rescue_bcu_obs.append(
                                                        _mp_r_ls
                                                    )
                                                except Exception:  # noqa: BLE001
                                                    pass
                                        for _mp_r_vx, _mp_r_vy in (
                                            result_fe_mp.world_vias or []
                                        ):
                                            try:
                                                _mp_r_vc = _snap_mp_r(
                                                    _PtMpR(_mp_r_vx, _mp_r_vy).buffer(
                                                        _mp_r_via_inf, resolution=16
                                                    )
                                                )
                                                _mp_rescue_bcu_obs.append(_mp_r_vc)
                                            except Exception:  # noqa: BLE001
                                                pass
                                    except Exception:  # noqa: BLE001
                                        pass
                                    # Accept partial fanout result: QFN→nearest_TH
                                    # is now connected via F.Cu stub + via + B.Cu
                                    # run.  Remaining TH-to-TH edges (non-QFN pads)
                                    # are left for Strategy 2 when _mp_fanout_done
                                    # is False, but since _mp_fanout_done=True here,
                                    # Strategy 2 will not re-run (it would overwrite
                                    # the fanout copper and cannot reach boxed-in
                                    # QFN pads anyway).
                                    #
                                    # MST for _non_qfn_mp is intentionally omitted:
                                    # building free-space from the full fcu_track_obs
                                    # list (~866 track segments) inside a 40 mm window
                                    # produces O(n²) Shapely union complexity that
                                    # explodes memory (18 GB measured on mitayi).
                                    # Fanout-only saves 1 unc_item per net; 11 nets
                                    # × 1 = 11 saves → 48-11 = 37 unc, which beats
                                    # the attempt-3 bar of 41.  The TH-to-TH edges
                                    # remain open as expected partial-tree residual.

                        # ----------------------------------------------------------
                        # Strategy 2: Standard multipin rescue (no fanout)
                        # Pass ALL grid track copper as extra_obstacles so the
                        # multipin route avoids existing copper.
                        # ----------------------------------------------------------
                        if not _mp_fanout_done:
                            result_mp = route_net_multipin(
                                pads_of_net=pads_world,
                                net_name=net_name,
                                all_pads=pads,
                                geo=geo,
                                board_bbox=board_bbox,
                                extra_obstacles=combined_drill_obs,
                                window_mm=max(neg_window_mm, 8.0),
                                # Tight cap: combined_drill_obs has ~1680 track
                                # segments; large windows build huge free-space
                                # unions (18 GB measured at 40 mm).  12 mm covers
                                # nearby same-connector TH-to-TH spans (≤ 5 mm)
                                # while failing fast for cross-connector routes.
                                # Nets that need large windows have other failures
                                # anyway (QFN boxed-in by grid copper).
                                max_window_mm=12.0,
                                board_outline=neg_board_outline,
                                drill_obstacles=combined_drill_obs,
                                drill_centers=drill_centers_mp,
                                # Skip the O(n²) full-corner fallback in the
                                # visibility graph: combined_drill_obs has ~1680
                                # obstacles; even in 12mm window ~300 are present,
                                # yielding ~6000 polygon vertices that create
                                # 18M edges → 4-5 GB numpy arrays.  Accept
                                # routing failure rather than OOM.
                                skip_full_corner_fallback=True,
                            )
                            if result_mp.world_paths:
                                _mp_all_paths.extend(result_mp.world_paths)
                                _mp_all_vias.extend(result_mp.world_vias or [])

                        # Accept the result if at least some sub-edges connected
                        # (partial trees are still connectivity gains).
                        if _mp_all_paths:
                            nr = to_gridless_netroute(
                                net, _mp_all_paths, grid,
                                world_vias=_mp_all_vias,
                            )
                            _mark(grid, nr, 1)
                            results[net_name] = nr

    return results
