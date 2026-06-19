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
    )

    if not result.ok:
        return NetRoute(net=net, ok=False, reason=result.reason)

    nr = to_gridless_netroute(net, result.world_paths, grid)

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

    Hard invariant: ``gridless_nets=None`` or empty (regardless of
    ``gridless_negotiate``) → behaviour is byte-identical to the pre-M2 engine."""
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

        if pads is not None and geo is not None and board_bbox is not None:
            by_name_all = {n.name: n for n in nets}
            net_set = []
            for net_name in sorted(gridless_nets):  # sorted for determinism
                net = by_name_all.get(net_name)
                if net is None or len(net.pads) < 2:
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
                )

                for net_name, neg_res in neg_results.items():
                    net = by_name_all.get(net_name)
                    if net is None:
                        continue
                    _gridless_pre_routed.add(net_name)
                    if neg_res.ok and neg_res.world_paths:
                        # Wrap in GridlessNetRoute, rasterize, mark grid ledger
                        nr = to_gridless_netroute(net, neg_res.world_paths, grid)
                        _mark(grid, nr, 1)
                        results[net_name] = nr
                    else:
                        # Failed / geometry-blocked: report as a failed NetRoute
                        results[net_name] = NetRoute(
                            net=net, ok=False, reason=neg_res.reason
                        )

    routed: list[NetRoute] = []
    by_name = {n.name: n for n in nets}
    budget = ripup_factor * max(len(nets), 1)
    history = (np.zeros((grid.layers, grid.ny, grid.nx), np.float64)
               if history_factor > 0.0 else None)

    # Populate routed list from already-committed gridless nets so grid rip-up
    # can select them as victims if needed (staged coexistence: no cross-substrate
    # rip-up in M2 — they are already in grid.hard so the grid router avoids them)
    for _name, nr in results.items():
        if nr.ok:
            routed.append(nr)

    queue = order_nets(
        [n for n in nets if n.name not in _gridless_pre_routed], priority
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
        if gridless_nets and net.name in gridless_nets:
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
    return results
