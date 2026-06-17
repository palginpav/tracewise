"""PathFinder negotiated-congestion router (docs/FORMULATION.md §5.3).

The existing rip-up router treats grid occupancy as a HARD wall: a cell used by
net A is forbidden to net B, so when a net cannot find a free path it fails or
shaves clearance — "negotiated congestion minus the negotiation". Every measured
wall (more-time -> more-violations, forced clearance-shaving, no dominant lever)
is the signature of forbidding contention instead of pricing it.

PathFinder prices it. Hard obstacles (pads, keepouts, their clearance halos)
stay forbidden — capacity 0. Routed-track cells are negotiable, capacity 1:
two nets MAY share a cell, but at a cost that rises with present congestion and
with history (how often the cell has been over-used across iterations). Route
all nets allowing overuse, then iterate: rip up and re-route over-used nets at
ever-higher congestion price until no cell is shared (a legal, short-free
routing) or the budget is spent. Nets with alternatives vacate contested cells
and leave them to nets that have none — the negotiation the greedy router lacks.

A single iteration reuses the same A* as the rip-up router; only the cell-entry
cost changes from {0 if free else infinity} to base*(1+h*hist)*(1+p*present).
"""

from __future__ import annotations

import heapq
import math
from dataclasses import dataclass, field

import numpy as np

from tracewise.route.engine.grid import Grid
from tracewise.route.engine.multi import Net, NetRoute, order_nets

SQRT2 = math.sqrt(2.0)
DIRS = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
        (-1, -1, SQRT2), (-1, 1, SQRT2), (1, -1, SQRT2), (1, 1, SQRT2)]


@dataclass
class PFState:
    """Mutable congestion state shared across the negotiation iterations."""
    grid: Grid
    occ: np.ndarray  # [L,ny,nx] int: routed-track nets occupying each cell
    hist: np.ndarray  # [L,ny,nx] float: accumulated historical congestion cost
    net_cells: dict = field(default_factory=dict)  # name -> set[(l,iy,ix)]

    @classmethod
    def make(cls, grid: Grid) -> PFState:
        shape = grid.cells.shape
        return cls(grid=grid, occ=np.zeros(shape, np.int32),
                   hist=np.zeros(shape, np.float64))

    def add(self, name: str, cells: set) -> None:
        self.net_cells[name] = cells
        for layer, iy, ix in cells:
            self.occ[layer, iy, ix] += 1

    def remove(self, name: str) -> None:
        for layer, iy, ix in self.net_cells.pop(name, ()):  # noqa: B007
            self.occ[layer, iy, ix] -= 1


def _pf_route(state: PFState, net: Net, h_fac: float, p_fac: float,
              via_cost: float, max_expansions: int) -> tuple[list, set] | None:
    """A* for one net's connection tree on the SOFT cost surface. Hard cells
    (pads/keepouts, occ via grid.hard) are forbidden except the net's own pads
    (the A* goals). Returns (paths, cells) or None if a pad can't be reached."""
    grid = state.grid
    occ, hist = state.occ, state.hist
    L, H, W = grid.cells.shape

    def cell_cost(layer, iy, ix, move):
        # present congestion = nets already on this cell (this net is removed
        # from occ before routing); capacity 1, so each extra net adds 1.
        present = occ[layer, iy, ix]
        return move * (1.0 + h_fac * hist[layer, iy, ix]) * (1.0 + p_fac * present)

    all_paths: list = []
    tree: set = {net.pads[0]}
    for pad in net.pads[1:]:
        if pad in tree:
            continue
        goals = tree
        res = _astar_soft(grid, pad, goals, cell_cost, via_cost,
                          state.net_cells.get(net.name), max_expansions)
        if res is None:
            return None  # unroutable even through congestion -> a real failure
        all_paths.append(res)
        tree.update(res)
    cells = set().union(*all_paths) if all_paths else set()
    cells -= set(net.pads)  # pads are their own hard obstacles, not track cells
    return all_paths, cells


def _astar_soft(grid: Grid, start, goals: set, cell_cost, via_cost: float,
                own_cells, max_expansions: int):
    """Soft-cost A* from start to nearest goal. Enters any cell whose grid.hard
    count is 0 (or which this net already owns, or which is a goal); priced by
    cell_cost. Octile heuristic (admissible: min cell cost is the base move)."""
    hard = grid.hard
    L, H, W = grid.cells.shape
    own = own_cells or set()

    def h(node):
        nl, ny, nx = node
        best = math.inf
        for gl, gy, gx in goals:
            dy, dx = abs(gy - ny), abs(gx - nx)
            d = (dy + dx) + (SQRT2 - 2) * min(dy, dx)
            if gl != nl:
                d += via_cost
            best = min(best, d)
        return best

    open_q = [(h(start), 0.0, start)]
    came = {start: None}
    g = {start: 0.0}
    exp = 0
    while open_q:
        _, gc, node = heapq.heappop(open_q)
        if node in goals:
            path = []
            cur = node
            while cur is not None:
                path.append(cur)
                cur = came[cur]
            return path
        if gc > g.get(node, math.inf):
            continue
        exp += 1
        if exp > max_expansions:
            return None
        nl, ny, nx = node
        for dy, dx, mv in DIRS:
            iy, ix = ny + dy, nx + dx
            if not (0 <= iy < H and 0 <= ix < W):
                continue
            nxt = (nl, iy, ix)
            if nxt not in goals and hard[nl, iy, ix] > 0 and nxt not in own:
                continue  # hard obstacle (pad/keepout) — never negotiable
            step = mv if nxt in goals else cell_cost(nl, iy, ix, mv)
            ng = gc + step
            if ng < g.get(nxt, math.inf):
                g[nxt] = ng
                came[nxt] = node
                heapq.heappush(open_q, (ng + h(nxt), ng, nxt))
        for layer in range(L):  # via
            if layer == nl:
                continue
            nxt = (layer, ny, nx)
            if nxt not in goals and hard[layer, ny, nx] > 0 and nxt not in own:
                continue
            step = via_cost if nxt in goals else cell_cost(layer, ny, nx, via_cost)
            ng = gc + step
            if ng < g.get(nxt, math.inf):
                g[nxt] = ng
                came[nxt] = node
                heapq.heappush(open_q, (ng + h(nxt), ng, nxt))
    return None


def route_all_pathfinder(
    grid: Grid, nets: list[Net], via_cost: float = 10.0, priority=None,
    iters: int = 20, h_fac: float = 0.4, p_growth: float = 1.8,
    max_expansions: int | None = None,
) -> dict[str, NetRoute]:
    """Negotiated-congestion routing. Returns name -> NetRoute. A net is `ok`
    iff its connection tree is built AND none of its cells is over-used (shared)
    at convergence — so an ok route is short-free by construction."""
    if max_expansions is None:
        max_expansions = 2 * grid.layers * grid.ny * grid.nx
    state = PFState.make(grid)
    ordered = order_nets(nets, priority)
    routed_nets = [n for n in ordered if len(n.pads) >= 2]
    results: dict[str, NetRoute] = {}
    paths_of: dict[str, list] = {}
    p_fac = 0.5

    # iteration 0: route everyone once (cheap congestion), then negotiate
    for _it in range(iters):
        over = state.occ > 1  # any cell used by >1 net
        to_route = [n for n in routed_nets
                    if n.name not in state.net_cells
                    or any(over[c] for c in state.net_cells[n.name])]
        if not to_route:
            break  # legal: no over-used cell
        for n in to_route:
            state.remove(n.name)
            res = _pf_route(state, n, h_fac, p_fac, via_cost, max_expansions)
            if res is None:
                paths_of[n.name] = []
                state.add(n.name, set())  # routed as empty (failed) — no cells
                continue
            all_paths, cells = res
            paths_of[n.name] = all_paths
            state.add(n.name, cells)
        # history: charge cells still over-used after this sweep; ramp the price
        state.hist += np.maximum(0.0, state.occ.astype(np.float64) - 1.0)
        p_fac *= p_growth

    over = state.occ > 1
    for n in routed_nets:
        cells = state.net_cells.get(n.name, set())
        paths = paths_of.get(n.name, [])
        shared = any(over[c] for c in cells)
        ok = bool(paths) and not shared
        nr = NetRoute(net=n, paths=paths, cells=cells, ok=ok,
                      reason="" if ok else ("congested" if shared else "no path"))
        results[n.name] = nr
    for n in nets:  # single-pad / trivial nets
        results.setdefault(n.name, NetRoute(net=n, ok=True))
    return results
