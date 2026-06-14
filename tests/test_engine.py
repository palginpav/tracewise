"""R0 engine tests: grid semantics, A* completeness, path simplification."""

import pytest

from tracewise.route.engine.astar import route, simplify
from tracewise.route.engine.grid import BLOCKED, FREE, Grid


def make_grid(w=10.0, h=10.0, layers=2):
    return Grid(x0=0.0, y0=0.0, width_mm=w, height_mm=h, layers=layers)


# --- grid --------------------------------------------------------------------


def test_grid_dimensions_and_transforms():
    g = make_grid(10.0, 5.0)
    assert (g.nx, g.ny) == (100, 50)
    iy, ix = g.to_cell(1.0, 2.0)
    assert (iy, ix) == (20, 10)
    assert g.to_world(iy, ix) == pytest.approx((1.0, 2.0))


def test_block_disc_inflates():
    g = make_grid()
    g.block_disc(0, 5.0, 5.0, radius_mm=0.5)
    cy, cx = g.to_cell(5.0, 5.0)
    assert g.cells[0, cy, cx] == BLOCKED
    assert g.cells[0, cy, cx + 4] == BLOCKED  # within 0.5mm
    assert g.cells[0, cy, cx + 6] == FREE  # outside
    assert g.cells[1].sum() == 0  # other layer untouched


def test_block_rect_clamps_to_bounds():
    g = make_grid()
    g.block_rect(0, -5.0, -5.0, 1.0, 1.0, inflate_mm=0.2)
    assert g.cells[0, 0, 0] == BLOCKED  # no crash, clipped


# --- A* ----------------------------------------------------------------------


def test_route_straight_line_empty_grid():
    g = make_grid()
    r = route(g, (0, 10, 10), {(0, 10, 90)})
    assert r.ok and r.path[0] == (0, 10, 10) and r.path[-1] == (0, 10, 90)
    assert r.cost == pytest.approx(80.0)  # straight, no diagonal needed


def test_route_around_obstacle():
    g = make_grid(layers=1)  # single layer: no via shortcut, must detour
    g.block_rect(0, 4.0, 0.0, 5.0, 9.0)  # wall with a gap at the top
    r = route(g, (0, 10, 10), {(0, 10, 80)})
    assert r.ok
    assert all(g.cells[layer, iy, ix] == FREE for layer, iy, ix in r.path)
    assert max(iy for _, iy, _ in r.path) >= 90  # detoured through the gap


def test_route_uses_via_when_layer_blocked():
    g = make_grid()
    g.block_rect(0, 4.0, 0.0, 5.0, 10.0)  # full wall on F.Cu
    r = route(g, (0, 50, 10), {(0, 50, 90)}, via_cost=10.0)
    assert r.ok
    layers_used = {layer for layer, _, _ in r.path}
    assert layers_used == {0, 1}  # had to dive to B.Cu and come back


def test_route_fails_explicitly_when_walled_in():
    g = make_grid(layers=1)
    g.block_rect(0, 4.0, 0.0, 5.0, 10.0)  # full wall, single layer
    r = route(g, (0, 50, 10), {(0, 50, 90)})
    assert not r.ok and r.reason == "no path"
    assert r.path == []  # no partial path — stubs are inexpressible


def test_route_to_nearest_of_multiple_goals():
    g = make_grid()
    r = route(g, (0, 50, 50), {(0, 50, 90), (0, 50, 60)})
    assert r.ok and r.path[-1] == (0, 50, 60)


def test_goal_cell_may_be_blocked():
    g = make_grid()
    g.block_disc(0, 9.0, 5.0, 0.3)  # the target pad blocks its own cells
    goal = (0, *g.to_cell(9.0, 5.0))
    r = route(g, (0, 50, 10), {goal})
    assert r.ok and r.path[-1] == goal


# --- simplification ------------------------------------------------------------


def test_simplify_collinear_and_diagonal():
    path = [(0, 0, 0), (0, 0, 1), (0, 0, 2), (0, 1, 3), (0, 2, 4), (0, 3, 4)]
    runs = simplify(path)
    assert len(runs) == 1
    assert runs[0] == [(0, 0, 0), (0, 0, 2), (0, 2, 4), (0, 3, 4)]


def test_simplify_splits_at_via():
    path = [(0, 0, 0), (0, 0, 1), (1, 0, 1), (1, 0, 2)]
    runs = simplify(path)
    assert len(runs) == 2
    assert runs[0][-1] == (0, 0, 1) and runs[1][0] == (1, 0, 1)


def test_block_polygon_rectangle():
    g = make_grid(10.0, 10.0, layers=2)
    # 2x2mm rectangle keepout centered at (5,5), layer 0 only
    g.block_polygon(0, [(4.0, 4.0), (6.0, 4.0), (6.0, 6.0), (4.0, 6.0)])
    cy, cx = g.to_cell(5.0, 5.0)
    assert not g.free(0, cy, cx)  # inside keepout -> blocked
    assert g.hard[0, cy, cx] > 0  # hard (escape can't cross)
    assert g.free(1, cy, cx)  # other layer untouched
    oy, ox = g.to_cell(1.0, 1.0)
    assert g.free(0, oy, ox)  # outside keepout -> free


def test_extract_keepouts_parses_tracks_not_allowed(tmp_path):
    from tracewise.route.engine.kicad import extract_keepouts
    pcb = tmp_path / "k.kicad_pcb"
    pcb.write_text("""(kicad_pcb (version 1) (generator "t")
      (zone (net 0) (layers "F&B.Cu")
        (keepout (tracks not_allowed) (vias not_allowed) (copperpour not_allowed))
        (polygon (pts (xy 10 10) (xy 20 10) (xy 20 20) (xy 10 20)))))""")
    kos = extract_keepouts(pcb)
    assert len(kos) == 1
    assert kos[0]["layers"] == {0, 1}
    assert len(kos[0]["pts"]) == 4
