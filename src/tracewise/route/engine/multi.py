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
from dataclasses import dataclass, field

from tracewise.route.engine.astar import RouteResult, route
from tracewise.route.engine.grid import FREE, Grid

POWER = re.compile(r"^\+?(VCC|VDD|VBUS|VBAT|VSYS|GND|VSS|[0-9]+V[0-9]*)$", re.IGNORECASE)


@dataclass
class Net:
    name: str
    pads: list[tuple[int, int, int]]  # (layer, iy, ix)
    halfwidth_cells: int = 1  # track halfwidth + clearance, in cells
    # own pad discs to carve free while this net routes: (layer, x_mm, y_mm, r_mm)
    carve: list[tuple[int, float, float, float]] = field(default_factory=list)


@dataclass
class NetRoute:
    net: Net
    paths: list[list[tuple[int, int, int]]] = field(default_factory=list)
    cells: set[tuple[int, int, int]] = field(default_factory=set)
    ok: bool = False
    reason: str = ""


def _mark(grid: Grid, nr: NetRoute, value: int) -> None:
    r = nr.net.halfwidth_cells
    for layer, iy, ix in nr.cells:
        y1, y2 = max(0, iy - r), min(grid.ny, iy + r + 1)
        x1, x2 = max(0, ix - r), min(grid.nx, ix + r + 1)
        grid.cells[layer, y1:y2, x1:x2] = value


def _bbox_size(net: Net) -> int:
    ys = [p[1] for p in net.pads]
    xs = [p[2] for p in net.pads]
    return (max(ys) - min(ys)) + (max(xs) - min(xs))


def order_nets(nets: list[Net]) -> list[Net]:
    return sorted(nets, key=lambda n: (0 if POWER.match(n.name) else 1, _bbox_size(n)))


def route_net(grid: Grid, net: Net, via_cost: float = 10.0) -> NetRoute:
    nr = NetRoute(net=net)
    if len(net.pads) < 2:
        nr.ok = True  # nothing to connect
        return nr
    for layer, x, y, r in net.carve:  # own pads must not wall the net in
        grid.block_disc(layer, x, y, r, value=FREE)
    tree: set[tuple[int, int, int]] = {net.pads[0]}
    for pad in net.pads[1:]:
        if pad in tree:
            continue
        res: RouteResult = route(grid, pad, tree, via_cost=via_cost)
        if not res.ok:
            nr.reason = f"pad {pad}: {res.reason}"
            nr.cells.clear()
            nr.paths.clear()
            for layer, x, y, r in net.carve:
                grid.block_disc(layer, x, y, r)
            return nr  # all-or-nothing: no stubs
        nr.paths.append(res.path)
        tree.update(res.path)
    nr.cells = tree - set(net.pads)  # pads stay as the pad obstacles they are
    for layer, x, y, r in net.carve:
        grid.block_disc(layer, x, y, r)
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


def route_all(grid: Grid, nets: list[Net], via_cost: float = 10.0,
              ripup_factor: int = 8) -> dict[str, NetRoute]:
    """Route every net; bounded rip-up on failures. Returns name -> NetRoute."""
    results: dict[str, NetRoute] = {}
    routed: list[NetRoute] = []
    budget = ripup_factor * max(len(nets), 1)

    queue = order_nets(nets)
    attempts: dict[str, int] = {}
    while queue and budget > 0:
        net = queue.pop(0)
        budget -= 1
        attempts[net.name] = attempts.get(net.name, 0) + 1
        nr = route_net(grid, net, via_cost=via_cost)
        if nr.ok:
            _mark(grid, nr, 1)
            routed.append(nr)
            results[net.name] = nr
            continue
        if attempts[net.name] <= 10:
            victim = _nearest_victim(net, routed)
            if victim is not None:
                _mark(grid, victim, FREE)
                routed.remove(victim)
                results.pop(victim.net.name, None)
                queue.insert(0, victim.net)  # victim re-routes after us
                queue.insert(0, net)  # we retry first, with the space freed
                continue
        results[net.name] = nr  # definitive failure, recorded
    for net in queue:  # budget exhausted
        results.setdefault(net.name, NetRoute(net=net, reason="rip-up budget exhausted"))
    return results
