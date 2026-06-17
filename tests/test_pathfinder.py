"""PathFinder negotiated-congestion router tests."""

import numpy as np

from tracewise.route.engine.grid import Grid
from tracewise.route.engine.multi import Net
from tracewise.route.engine.pathfinder import route_all_pathfinder


def open_grid(w=4.0, h=4.0, layers=2):
    return Grid(x0=0, y0=0, width_mm=w, height_mm=h, pitch=0.1, layers=layers)


def test_pathfinder_resolves_crossing_to_zero_overuse():
    # two nets whose shortest paths both want the center cell (10,20): a
    # horizontal net and a vertical net crossing. PathFinder must give them
    # non-overlapping copper (occ<=1 everywhere) — short-free by construction.
    g = open_grid()
    nets = [
        Net("H", [(0, 20, 2), (0, 20, 38)]),  # row 20, layer 0
        Net("V", [(0, 2, 20), (0, 38, 20)]),  # col 20, layer 0
    ]
    res = route_all_pathfinder(g, nets, iters=30)
    assert res["H"].ok and res["V"].ok
    # reconstruct occupancy from the returned cells: no cell shared
    occ = {}
    for nr in (res["H"], res["V"]):
        for c in nr.cells:
            occ[c] = occ.get(c, 0) + 1
    assert max(occ.values()) <= 1, "PathFinder left an over-used (shorted) cell"


def test_pathfinder_connects_simple_net():
    g = open_grid()
    res = route_all_pathfinder(g, [Net("N", [(0, 10, 5), (0, 10, 35)])])
    assert res["N"].ok and res["N"].paths


def test_pathfinder_reports_unroutable_explicitly():
    # a net walled in on a single layer with no gap -> explicit no-path failure
    g = open_grid(layers=1)
    g.cells[0, :, 20] = 1  # full wall
    g.hard[0, :, 20] = 1
    res = route_all_pathfinder(g, [Net("N", [(0, 10, 5), (0, 10, 35)])])
    assert not res["N"].ok and res["N"].reason


def test_pathfinder_respects_hard_obstacles():
    # a pad/keepout (hard) cell is never used by another net's track
    g = open_grid()
    g.cells[0, 20, 20] = 1
    g.hard[0, 20, 20] = 1  # hard obstacle at the crossing cell
    nets = [Net("H", [(0, 20, 2), (0, 20, 38)])]
    res = route_all_pathfinder(g, nets)
    assert res["H"].ok
    assert (0, 20, 20) not in res["H"].cells  # routed around the hard cell
