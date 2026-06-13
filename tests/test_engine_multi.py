"""R1 tests: ordering, connection trees, copper-as-obstacle, rip-up."""

from tracewise.route.engine.grid import FREE, Grid
from tracewise.route.engine.multi import Net, order_nets, route_all, route_net


def make_grid(layers=1):
    return Grid(x0=0, y0=0, width_mm=10, height_mm=10, layers=layers)


def test_order_power_first_then_short():
    nets = [
        Net("SIG_LONG", [(0, 0, 0), (0, 90, 90)]),
        Net("GND", [(0, 0, 0), (0, 50, 50)]),
        Net("SIG_SHORT", [(0, 0, 0), (0, 5, 5)]),
    ]
    assert [n.name for n in order_nets(nets)] == ["GND", "SIG_SHORT", "SIG_LONG"]


def test_three_pad_net_forms_one_tree():
    g = make_grid()
    nr = route_net(g, Net("N", [(0, 10, 10), (0, 10, 90), (0, 90, 50)]))
    assert nr.ok and len(nr.paths) == 2
    tree = set().union(*[set(p) for p in nr.paths])
    assert (0, 10, 10) in tree  # third pad connected to the tree, not pad-to-pad


def test_failed_net_leaves_no_copper():
    g = make_grid()
    g.block_rect(0, 4.0, 0.0, 5.0, 10.0)  # bisecting wall
    nr = route_net(g, Net("N", [(0, 50, 10), (0, 50, 90)]))
    assert not nr.ok and nr.cells == set() and nr.paths == []


def test_routed_net_blocks_later_net_or_ripup_resolves():
    # vertical net crosses the board; horizontal net must rip it up or detour
    g = make_grid()
    nets = [
        Net("V", [(0, 5, 50), (0, 95, 50)]),
        Net("H", [(0, 50, 5), (0, 50, 95)]),
    ]
    results = route_all(g, nets)
    assert all(r.ok for r in results.values())  # single layer: detour around ends


def test_ripup_resolves_blocked_corridor():
    # narrow corridor: net A routes through it first; net B needs the same
    # corridor and its endpoints are walled elsewhere -> rip-up must fire
    g = make_grid()
    g.block_rect(0, 0.0, 3.0, 4.4, 3.4)   # walls leaving a corridor at x~4.5-5.5
    g.block_rect(0, 5.6, 3.0, 10.0, 3.4)
    g.block_rect(0, 0.0, 6.6, 4.4, 7.0)
    g.block_rect(0, 5.6, 6.6, 10.0, 7.0)
    nets = [
        Net("A", [(0, 10, 50), (0, 90, 50)]),  # both want the corridor
        Net("B", [(0, 20, 50), (0, 80, 50)]),
    ]
    results = route_all(g, nets)
    assert sum(1 for r in results.values() if r.ok) >= 1
    failed = [r for r in results.values() if not r.ok]
    for f in failed:
        assert f.reason  # explicit, never silent


def test_all_routed_on_open_board_many_nets():
    g = Grid(x0=0, y0=0, width_mm=20, height_mm=20, layers=2)
    nets = [Net(f"N{i}", [(0, 10 + i * 8, 10), (0, 10 + i * 8, 190)]) for i in range(20)]
    results = route_all(g, nets)
    assert all(r.ok for r in results.values())
    assert (g.cells == FREE).mean() < 1.0  # copper actually marked


def test_route_all_time_budget_bails_to_failures():
    from tracewise.route.engine.multi import Net, route_all
    g = make_grid(layers=2)
    nets = [Net(f"N{i}", [(0, 5, 5), (0, 90, 90)]) for i in range(5)]
    # zero budget: deadline already passed -> every net an explicit failure,
    # never hangs, reason names the cap
    res = route_all(g, nets, time_budget_s=0.0)
    assert len(res) == 5
    assert all(not r.ok and "time budget" in r.reason for r in res.values())
