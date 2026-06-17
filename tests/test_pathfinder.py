"""PathFinder negotiated-congestion router tests."""

from tracewise.route.engine.grid import Grid
from tracewise.route.engine.multi import Net
from tracewise.route.engine.pathfinder import route_all_pathfinder


def net(name, pads):
    # small halos keep the synthetic tests legible
    return Net(name, pads, halfwidth_cells=1, via_halfwidth_cells=1)


def open_grid(w=6.0, h=6.0, layers=2):
    return Grid(x0=0, y0=0, width_mm=w, height_mm=h, pitch=0.1, layers=layers)


def overuse(res):
    # congestion = a cell whose clearance halo is claimed by >= 2 DISTINCT nets.
    # Dedup each net's halo to a set first; a single track's own neighbouring
    # cells must not count as self-overuse.
    occ = {}
    for nr in res.values():
        if not nr.cells:
            continue
        hw = nr.net.halfwidth_cells
        halo = set()
        for layer, iy, ix in nr.cells:
            for dy in range(-hw, hw + 1):
                for dx in range(-hw, hw + 1):
                    halo.add((layer, iy + dy, ix + dx))
        for c in halo:
            occ[c] = occ.get(c, 0) + 1
    over = sum(1 for v in occ.values() if v > 1)
    return over


def test_pathfinder_connects_simple_net():
    g = open_grid()
    res = route_all_pathfinder(g, [net("N", [(0, 30, 5), (0, 30, 55)])])
    assert res["N"].ok and res["N"].paths


def _two_gap_nets():
    g = open_grid(12.0, 12.0, layers=1)
    for plane in (g.cells, g.hard):  # a forbidden wall = actual copper
        plane[0, :, 60] = 1  # wall at col 60 ...
        plane[0, 20:30, 60] = 0  # ... wide gap A
        plane[0, 90:100, 60] = 0  # ... wide gap B
    nets = [
        net("A", [(0, 40, 8), (0, 40, 112)]),  # both nearer gap A
        net("B", [(0, 50, 8), (0, 50, 112)]),
    ]
    return g, nets


def test_pathfinder_negotiation_resolves_contention():
    # one greedy pass piles both nets onto the nearer gap (overuse); the
    # negotiation must raise the congestion price until they spread and no halo
    # cell is shared -- both nets clearance-legal by construction.
    g0, n0 = _two_gap_nets()
    greedy = route_all_pathfinder(g0, n0, iters=1)  # no negotiation
    assert not (greedy["A"].ok and greedy["B"].ok), "greedy should over-use here"
    g1, n1 = _two_gap_nets()
    full = route_all_pathfinder(g1, n1, iters=60)
    assert full["A"].ok and full["B"].ok, "negotiation failed to resolve contention"
    assert overuse(full) <= 1, "PathFinder left an over-used cell with room to spare"


def test_pathfinder_reports_unroutable_explicitly():
    g = open_grid(layers=1)
    g.cells[0, :, 30] = 1  # full wall, no gap
    g.hard[0, :, 30] = 1  # actual copper -> forbidden
    res = route_all_pathfinder(g, [net("N", [(0, 30, 5), (0, 30, 55)])])
    assert not res["N"].ok and res["N"].reason


def test_pathfinder_respects_forbidden_cells():
    g = open_grid()
    g.cells[0, 30, 30] = 1  # forbidden (pad/keepout) at the would-be crossing
    g.hard[0, 30, 30] = 1  # actual copper -> forbidden
    res = route_all_pathfinder(g, [net("H", [(0, 30, 4), (0, 30, 56)])])
    assert res["H"].ok
    assert (0, 30, 30) not in res["H"].cells  # routed around the forbidden cell
