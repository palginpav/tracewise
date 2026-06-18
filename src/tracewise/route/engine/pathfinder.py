"""PathFinder negotiated-congestion router (docs/FORMULATION.md §5.3).

The rip-up router treats grid occupancy as a HARD wall: a cell used by net A is
forbidden to net B, so when a net cannot find a free path it fails or shaves
clearance — "negotiated congestion minus the negotiation". Every measured wall
(more-time -> more-violations, forced clearance-shaving, no dominant lever) is
the signature of forbidding contention instead of pricing it.

PathFinder prices it. Two planes:

  * `hard` — inflated pads + keepouts (and their clearance). Capacity 0: never
    enterable (except a net's own pads, carved while it routes). This keeps the
    router clear of fixed copper.
  * `occ` — track-halo congestion: when a net commits, its centerline is
    inflated by (halfwidth + clearance) and added here. Capacity 1, so two nets
    whose copper would come within clearance share an `occ` cell. Entering an
    `occ` cell is allowed but PRICED by present congestion and history (how
    often the cell has been over-used across iterations).

Route everyone allowing overuse, then iterate: rip up and reroute over-used nets
at an ever-higher congestion price until no cell is shared — a legal, clearance-
respecting, short-free routing — or the budget is spent. Nets with alternatives
vacate contested cells and leave them to nets that have none.
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
class _Net:
    name: str
    cells: set = field(default_factory=set)  # committed centerline path cells
    vias: set = field(default_factory=set)   # (iy,ix) via sites
    halo: set = field(default_factory=set)   # occ-inflated cells (for rip-up)
    paths: list = field(default_factory=list)


def _halo_cells(grid: Grid, cells: set, vias: set, hw: int, vhw: int) -> set:
    """Inflate centerline cells (by hw) and via sites (by vhw, all layers) to
    the clearance footprint used for congestion."""
    out = set()
    L, H, W = grid.cells.shape
    for layer, iy, ix in cells:
        for dy in range(-hw, hw + 1):
            yy = iy + dy
            if 0 <= yy < H:
                for dx in range(-hw, hw + 1):
                    xx = ix + dx
                    if 0 <= xx < W:
                        out.add((layer, yy, xx))
    for iy, ix in vias:
        for dy in range(-vhw, vhw + 1):
            yy = iy + dy
            if 0 <= yy < H:
                for dx in range(-vhw, vhw + 1):
                    xx = ix + dx
                    if 0 <= xx < W:
                        for layer in range(L):
                            out.add((layer, yy, xx))
    return out


def _dilate(a: np.ndarray, r: int) -> np.ndarray:
    """Diamond max-dilation by r cells (per layer). Pricing a candidate
    centerline by dilated congestion makes the cost HALO-aware: a track whose
    inflated footprint would overlap another net's copper is priced even though
    its centerline cell looks free (the bug that let two nets pile into one gap,
    each on a 'free' centerline while their halos collided)."""
    if r <= 0:
        return a
    d = a
    for _ in range(r):
        nd = d.copy()
        nd[:, 1:, :] = np.maximum(nd[:, 1:, :], d[:, :-1, :])
        nd[:, :-1, :] = np.maximum(nd[:, :-1, :], d[:, 1:, :])
        nd[:, :, 1:] = np.maximum(nd[:, :, 1:], d[:, :, :-1])
        nd[:, :, :-1] = np.maximum(nd[:, :, :-1], d[:, :, 1:])
        d = nd
    return d


def _astar(hard, fixed, occ, hist, start, goals: set, via_cost, h_fac, p_fac,
           fixed_pen, max_expansions: int):
    """Soft-cost A* to the nearest goal. `hard` (bool, actual copper/keepout)
    cells are forbidden unless they are a goal. `fixed` marks fixed-pad
    clearance halos: enterable at a CONSTANT premium (escape, generalised) so a
    pad sealed by its neighbours' clearance can still be left, while the price
    does not escalate with the negotiation. Inter-net congestion (`occ`) and
    history escalate. Cost = move*(1+fixed_pen*fixed)*(1+h*hist)*(1+p*occ).
    Octile heuristic (admissible: min step = move)."""
    L, H, W = hard.shape

    def heur(node):
        nl, ny, nx = node
        best = math.inf
        for gl, gy, gx in goals:
            dy, dx = abs(gy - ny), abs(gx - nx)
            d = (dy + dx) + (SQRT2 - 2) * min(dy, dx)
            if gl != nl:
                d += via_cost
            best = min(best, d)
        return best

    open_q = [(heur(start), 0.0, start)]
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
        nbrs = []
        for dy, dx, mv in DIRS:
            iy, ix = ny + dy, nx + dx
            if 0 <= iy < H and 0 <= ix < W:
                nbrs.append(((nl, iy, ix), mv))
        for layer in range(L):
            if layer != nl:
                nbrs.append(((layer, ny, nx), via_cost))
        for nxt, mv in nbrs:
            nl2, iy, ix = nxt
            is_goal = nxt in goals
            if not is_goal and hard[nl2, iy, ix]:
                continue
            step = mv if is_goal else mv * (
                1.0 + fixed_pen * fixed[nl2, iy, ix]) * (
                1.0 + h_fac * hist[nl2, iy, ix]) * (
                1.0 + p_fac * occ[nl2, iy, ix])
            ng = gc + step
            if ng < g.get(nxt, math.inf):
                g[nxt] = ng
                came[nxt] = node
                heapq.heappush(open_q, (ng + heur(nxt), ng, nxt))
    return None


def _route_one(hard, fixed, occ, hist, net: Net, via_cost, h_fac, p_fac,
               fixed_pen, max_exp):
    """Route a net's connection tree on the soft surface. Own pads are carved
    from `hard`/`fixed` by the caller. Returns (_Net) or None on a real
    no-path."""
    out = _Net(net.name)
    tree = {net.pads[0]}
    for pad in net.pads[1:]:
        if pad in tree:
            continue
        path = _astar(hard, fixed, occ, hist, pad, tree, via_cost, h_fac, p_fac,
                      fixed_pen, max_exp)
        if path is None:
            return None
        out.paths.append(path)
        for a, b in zip(path, path[1:], strict=False):
            if a[0] != b[0]:
                out.vias.add((a[1], a[2]))
        tree.update(path)
    out.cells = (tree - set(net.pads))
    return out


def route_all_pathfinder(
    grid: Grid, nets: list[Net], via_cost: float = 10.0, priority=None,
    iters: int = 40, h_fac: float = 1.0, p_init: float = 0.5,
    p_growth: float = 1.3, p_max: float = 20.0,
    fixed_pen: float = 4.0, max_expansions: int | None = None,
) -> dict[str, NetRoute]:
    """Negotiated-congestion routing. A net is `ok` iff its tree is built AND
    none of its halo cells is over-used at convergence — an ok route is
    clearance-respecting and short-free by construction.

    Obstacle model mirrors the rip-up router's hard/halo split: only actual
    copper and keepouts (`grid.hard`) are forbidden; fixed-pad clearance halos
    (`grid.cells` minus `grid.hard`) are enterable at the constant `fixed_pen`
    premium so pads sealed by their neighbours' clearance can still escape."""
    if max_expansions is None:
        max_expansions = 2 * grid.layers * grid.ny * grid.nx
    L, H, W = grid.cells.shape
    base_hard = grid.hard > 0  # actual copper + keepouts -> forbidden
    fixed = ((grid.cells > 0) & (grid.hard == 0)).astype(np.float64)  # clearance halos -> priced
    occ = np.zeros((L, H, W), np.int32)
    hist = np.zeros((L, H, W), np.float64)

    ordered = order_nets(nets, priority)
    routed = [n for n in ordered if len(n.pads) >= 2]
    by_name = {n.name: n for n in routed}
    state: dict[str, _Net] = {}

    def carve(net: Net, on: bool):
        # free the net's own pad footprints (copper AND clearance) so it can
        # leave its pads; restore the fixed obstacle state afterwards
        for layer, x1, y1, x2, y2, inf in getattr(net, "carve", ()):
            cy1, cx1 = grid.to_cell(min(x1, x2) - inf, min(y1, y2) - inf)
            cy2, cx2 = grid.to_cell(max(x1, x2) + inf, max(y1, y2) + inf)
            sl = (slice(max(0, cy1), min(H, cy2 + 1)), slice(max(0, cx1), min(W, cx2 + 1)))
            if on:
                base_hard[layer][sl] = False
                fixed[layer][sl] = 0.0
            else:
                base_hard[layer][sl] = grid.hard[layer][sl] > 0
                fixed[layer][sl] = ((grid.cells[layer][sl] > 0)
                                    & (grid.hard[layer][sl] == 0)).astype(np.float64)

    def commit(net: Net, r: _Net, delta: int):
        for c in r.halo:
            occ[c] += delta

    # McMurchie-Ebeling schedule (Phase 0 research): iteration 0 is a FREE
    # exploration pass (p_fac=0) so every net finds a path before costs escalate;
    # then present-cost starts at p_init and grows by p_growth, capped at p_max.
    # The crude version started at 0.5 and grew x1.8 -> a hard wall by iter ~5
    # (RC1 of the 0/61 divergence).
    p_fac = 0.0
    for it in range(iters):
        over = occ > 1
        to_route = [n for n in routed
                    if n.name not in state
                    or not state[n.name].paths          # RC3 fix: retry FAILED nets
                    or any(over[c] for c in state[n.name].halo)]
        if not to_route:
            break
        for net in to_route:
            if net.name in state:
                commit(net, state[net.name], -1)
                del state[net.name]
            carve(net, True)
            # price by HALO-dilated congestion + history so a centerline whose
            # footprint would overlap another net's copper is penalised (not
            # just exact centerline overlaps)
            hw = net.halfwidth_cells
            occ_cost = _dilate(occ, hw)
            hist_cost = _dilate(hist, hw)
            r = _route_one(base_hard, fixed, occ_cost, hist_cost, net, via_cost,
                           h_fac, p_fac, fixed_pen, max_expansions)
            carve(net, False)
            if r is None:
                state[net.name] = _Net(net.name)  # failed: no cells
                continue
            r.halo = _halo_cells(grid, r.cells, r.vias, net.halfwidth_cells,
                                 net.via_halfwidth_cells)
            commit(net, r, 1)
            state[net.name] = r
        hist += np.maximum(0.0, occ.astype(np.float64) - 1.0)
        # advance the present-cost schedule: p_init after the free iteration 0,
        # then geometric growth capped at p_max (avoids the hard-wall divergence)
        p_fac = p_init if it == 0 else min(p_fac * p_growth, p_max)

    over = occ > 1
    results: dict[str, NetRoute] = {}
    for name, r in state.items():
        net = by_name[name]
        shared = any(over[c] for c in r.halo)
        ok = bool(r.paths) and not shared
        results[name] = NetRoute(net=net, paths=r.paths, cells=r.cells,
                                 via_sites=r.vias, ok=ok,
                                 reason="" if ok else ("congested" if shared else "no path"))
    for n in nets:
        results.setdefault(n.name, NetRoute(net=n, ok=len(n.pads) < 2))
    return results
