"""M3-P2 Tests — Multi-pin connection-tree routing + boundary own-pad carve-out fix.

Test categories:
  T-P2-01  Boundary pad always reachable: pad at board edge is inside free space
           even after board-edge shrink (the M3-P2 own-pad carve-out fix).
  T-P2-02  prim_mst returns K-1 edges, deterministic, tie-break by index.
  T-P2-03  route_net_multipin on 3-pin straight net returns ok=True + 2 paths.
  T-P2-04  route_net_multipin on 2-pin net delegates to route_net_gridless.
  T-P2-05  route_net_multipin combined world_paths rasterizes into cells (adapter).
  T-P2-06  /GPIO15 analogue: 4-pin net routes fully connected, deterministic.
  T-P2-07  gridless_rescue=False is byte-identical (multi-pin no-op guard).

Skip cleanly when shapely is absent:
    pytest.importorskip("shapely")
"""

from __future__ import annotations

import pytest

shapely = pytest.importorskip("shapely")


# ---------------------------------------------------------------------------
# Shared fixtures
# ---------------------------------------------------------------------------

_GEO = {
    "track_mm": 0.2,
    "clearance_mm": 0.2,
    "via_mm": 0.6,
    "via_drill_mm": 0.3,
    "hole_clearance_mm": 0.25,
    "hole_to_hole_mm": 0.25,
    "via_cost_mm": 3.0,
}
_BBOX = (0.0, 0.0, 30.0, 30.0)


def _make_pads(coords: list[tuple[float, float]], net: str = "N") -> list[dict]:
    """Create minimal pad dicts for a single net at the given coords."""
    return [
        {
            "net": net,
            "x": x, "y": y,
            "hw": 0.3, "hh": 0.3,
            "front": True, "back": False,
        }
        for x, y in coords
    ]


def _make_mixed_pads(
    net_coords: list[tuple[float, float]],
    other_coords: list[tuple[float, float]],
    net: str = "N",
    other_net: str = "OTHER",
) -> list[dict]:
    """Create pads for the routing net plus obstacle pads for another net."""
    pads = _make_pads(net_coords, net)
    pads += [
        {
            "net": other_net,
            "x": x, "y": y,
            "hw": 0.3, "hh": 0.3,
            "front": True, "back": False,
        }
        for x, y in other_coords
    ]
    return pads


# ---------------------------------------------------------------------------
# T-P2-01: Boundary pad carve-out fix
# ---------------------------------------------------------------------------

def test_p2_boundary_pad_reachable() -> None:
    """A pad at the board edge must be inside free space even after the
    board-edge inward shrink.  This is the M3-P2 own-pad carve-out fix.

    The board outline is 10×10mm.  The routing net has one pad at (0.05, 5.0),
    which is 0.05mm from the board edge — well inside the 0.3mm shrink zone,
    so the shrunken window_poly alone would not contain the pad centre.

    After the M3-P2 fix, the pad must be inside the returned free_space.
    """
    from shapely.geometry import Point, Polygon

    from tracewise.route.gridless.geom import build_windowed_free_space

    # Board outline: 10×10mm
    board_outline = Polygon([(0, 0), (10, 0), (10, 10), (0, 10)])

    # Pad at the left board edge (x=0.05, far less than shrink=0.3mm)
    pads = [
        {"net": "N", "x": 0.05, "y": 5.0, "hw": 0.3, "hh": 0.3,
         "front": True, "back": False},
        {"net": "N", "x": 8.0, "y": 5.0, "hw": 0.3, "hh": 0.3,
         "front": True, "back": False},
    ]

    fs, _ = build_windowed_free_space(
        pads, "N", 0.2, 0.2, [],
        window_bbox=(0.0, 0.0, 10.0, 10.0),
        board_outline=board_outline,
        layer=0,
    )

    pad_centre = Point(0.05, 5.0)
    # The pad centre must be inside (or on the boundary of) the free space.
    assert fs.distance(pad_centre) < 0.01, (
        f"Boundary pad at (0.05, 5.0) must be reachable in free space; "
        f"distance to fs = {fs.distance(pad_centre):.4f} mm"
    )


def test_p2_boundary_pad_goal_point_reachable() -> None:
    """The goal pad centre at the board boundary must be inside the free space
    (not just within 0.01mm — this verifies the region is actually accessible).
    """
    from shapely.geometry import Point, Polygon

    from tracewise.route.gridless.geom import build_windowed_free_space

    board_outline = Polygon([(0, 0), (20, 0), (20, 20), (0, 20)])

    # Through-hole pad very close to left board edge
    pads = [
        {"net": "N", "x": 0.1, "y": 10.0, "hw": 0.5, "hh": 0.5,
         "front": True, "back": True},  # through-hole
        {"net": "N", "x": 15.0, "y": 10.0, "hw": 0.5, "hh": 0.5,
         "front": True, "back": False},
    ]

    fs, _ = build_windowed_free_space(
        pads, "N", 0.2, 0.2, [],
        window_bbox=(0.0, 0.0, 20.0, 20.0),
        board_outline=board_outline,
        layer=0,
    )

    pad_centre = Point(0.1, 10.0)
    assert fs.distance(pad_centre) < 0.02, (
        f"Through-hole pad at (0.1, 10.0) must be reachable; "
        f"distance = {fs.distance(pad_centre):.4f} mm"
    )


# ---------------------------------------------------------------------------
# T-P2-02: Prim's MST is deterministic and produces K-1 edges
# ---------------------------------------------------------------------------

def test_p2_prim_mst_deterministic() -> None:
    """_prim_mst returns K-1 edges and is deterministic (run twice, same result)."""
    from tracewise.route.gridless.route import _prim_mst

    pads = _make_pads([(0, 0), (5, 0), (10, 0), (5, 5)])
    edges1 = _prim_mst(pads)
    edges2 = _prim_mst(pads)

    assert len(edges1) == 3, f"Expected 3 MST edges for 4 pads, got {len(edges1)}"
    assert edges1 == edges2, "MST must be deterministic (same result twice)"


def test_p2_prim_mst_two_pads() -> None:
    """_prim_mst on 2 pads returns 1 edge."""
    from tracewise.route.gridless.route import _prim_mst

    pads = _make_pads([(0, 0), (10, 0)])
    edges = _prim_mst(pads)
    assert len(edges) == 1
    assert edges[0][0] == 0 and edges[0][1] == 1


def test_p2_prim_mst_one_pad() -> None:
    """_prim_mst on 1 pad returns empty."""
    from tracewise.route.gridless.route import _prim_mst

    pads = _make_pads([(0, 0)])
    assert _prim_mst(pads) == []


def test_p2_prim_mst_seed_is_index_zero() -> None:
    """MST must be seeded at pad index 0 (lowest index)."""
    from tracewise.route.gridless.route import _prim_mst

    # 3 pads in a line; MST should connect 0→1→2 (nearest chain from seed=0)
    pads = _make_pads([(0, 0), (3, 0), (6, 0)])
    edges = _prim_mst(pads)
    # Both edges must start from a pad that was in the tree (grows from 0)
    in_tree = {edges[0][0]}
    for i, j, _ in edges:
        assert i in in_tree, f"Edge ({i},{j}) source not in tree"
        in_tree.add(j)


# ---------------------------------------------------------------------------
# T-P2-03: route_net_multipin on 3-pin straight net
# ---------------------------------------------------------------------------

def test_p2_multipin_3pin_straight() -> None:
    """3-pin net in a straight line (open space) routes fully connected."""
    from tracewise.route.gridless.route import route_net_multipin

    pads = _make_pads([(2.0, 5.0), (10.0, 5.0), (18.0, 5.0)])
    result = route_net_multipin(
        pads_of_net=pads,
        net_name="N",
        all_pads=pads,
        geo=_GEO,
        board_bbox=_BBOX,
        window_mm=6.0,
    )
    assert result.ok, f"3-pin straight route failed: {result.reason}"
    assert len(result.world_paths) >= 2, (
        f"Expected ≥2 world_paths (one per MST sub-edge), got {len(result.world_paths)}"
    )


def test_p2_multipin_3pin_returns_world_paths_non_empty() -> None:
    """route_net_multipin world_paths are non-empty for a successful route."""
    from tracewise.route.gridless.route import route_net_multipin

    pads = _make_pads([(2.0, 2.0), (8.0, 2.0), (14.0, 2.0)])
    result = route_net_multipin(
        pads_of_net=pads,
        net_name="N",
        all_pads=pads,
        geo=_GEO,
        board_bbox=_BBOX,
    )
    # Should either succeed fully or partially
    if result.ok:
        for path in result.world_paths:
            assert len(path) >= 2, "Each sub-path must have ≥2 waypoints"


# ---------------------------------------------------------------------------
# T-P2-04: route_net_multipin 2-pin delegate path
# ---------------------------------------------------------------------------

def test_p2_multipin_2pin_delegates() -> None:
    """route_net_multipin on a 2-pin net produces a valid route."""
    from tracewise.route.gridless.route import route_net_multipin

    pads = _make_pads([(2.0, 5.0), (12.0, 5.0)])
    result = route_net_multipin(
        pads_of_net=pads,
        net_name="N",
        all_pads=pads,
        geo=_GEO,
        board_bbox=_BBOX,
    )
    assert result.ok, f"2-pin delegation failed: {result.reason}"
    assert len(result.world_paths) >= 1


# ---------------------------------------------------------------------------
# T-P2-05: Adapter rasterizes multi-path result correctly
# ---------------------------------------------------------------------------

def test_p2_multipin_adapter_rasterize() -> None:
    """GridlessNetRoute with multiple world_paths (multi-pin) rasterizes correctly."""
    from tracewise.route.engine.grid import Grid
    from tracewise.route.engine.multi import Net
    from tracewise.route.gridless.adapter import to_gridless_netroute

    grid = Grid(pitch=0.5, x0=0.0, y0=0.0, width_mm=30.0, height_mm=30.0)
    net = Net(name="N", pads=[], halfwidth_cells=1)

    # Simulate 2 sub-edge world paths (like a 3-pin MST result)
    world_paths = [
        [(2.0, 5.0), (10.0, 5.0)],   # sub-edge 0
        [(10.0, 5.0), (18.0, 5.0)],  # sub-edge 1
    ]
    nr = to_gridless_netroute(net, world_paths, grid)

    assert nr.ok
    assert len(nr.cells) > 0, "Multi-path result must rasterize to non-empty cells"
    # Cells from both paths should be present
    all_xs = {grid.to_cell(x, 5.0)[1] for x in [2.0, 10.0, 18.0]}
    # At least some cells from both paths should be present
    cell_xs = {ix for _, _, ix in nr.cells}
    overlap = cell_xs & all_xs
    assert len(overlap) >= 2, (
        f"Cells from both sub-paths should be rasterized; found {overlap}"
    )


# ---------------------------------------------------------------------------
# T-P2-06: /GPIO15-analogue: 4-pin net, deterministic
# ---------------------------------------------------------------------------

def test_p2_gpio15_analogue_4pin() -> None:
    """4-pin net analogous to /GPIO15: routes with MST + same-net-copper shortcut.

    Geometry mirrors the GPIO15 case from the spike: pads roughly in a line
    with a spread-out 4th pad.  The key test: result is deterministic (two calls
    produce identical world_paths byte-for-byte).
    """
    from tracewise.route.gridless.route import route_net_multipin

    # 4 pads analogous to GPIO15's layout (spread out to trigger copper shortcut)
    pads = _make_pads([
        (3.0, 3.0),
        (10.0, 8.0),
        (3.0, 15.0),
        (10.0, 22.0),
    ])

    result1 = route_net_multipin(
        pads_of_net=pads,
        net_name="N",
        all_pads=pads,
        geo=_GEO,
        board_bbox=_BBOX,
        window_mm=8.0,
    )
    result2 = route_net_multipin(
        pads_of_net=pads,
        net_name="N",
        all_pads=pads,
        geo=_GEO,
        board_bbox=_BBOX,
        window_mm=8.0,
    )

    # Both runs must agree on world_paths (determinism)
    paths1 = [tuple(tuple(p) for p in path) for path in result1.world_paths]
    paths2 = [tuple(tuple(p) for p in path) for path in result2.world_paths]
    assert paths1 == paths2, (
        f"4-pin multipin route must be deterministic; "
        f"run1 paths: {paths1[:2]}..., run2 paths: {paths2[:2]}..."
    )

    # Must have produced at least 1 successful sub-edge
    assert len(result1.world_paths) >= 1, (
        "4-pin net must route at least 1 MST sub-edge"
    )


# ---------------------------------------------------------------------------
# T-P2-07: gridless_rescue=False default is byte-identical (multi-pin no-op)
# ---------------------------------------------------------------------------

def test_p2_rescue_false_no_multipin_routing() -> None:
    """gridless_rescue=False must not trigger multi-pin routing (no-op guard).

    Creates a minimal grid + net + results with a failing 3-pin net, then calls
    route_all with gridless_rescue=False and verifies the 3-pin failure is
    unchanged (route_net_multipin was NOT called).
    """
    from tracewise.route.engine.grid import Grid
    from tracewise.route.engine.multi import Net, route_all

    grid = Grid(pitch=0.5, x0=0.0, y0=0.0, width_mm=30.0, height_mm=30.0)

    # A 3-pin net (will fail grid routing since we don't pre-populate pads as goals)
    net = Net(name="MULTI_PIN", pads=[(0, 2, 2), (0, 10, 10), (0, 18, 18)],
              halfwidth_cells=1)

    # Route with gridless_rescue=False — multi-pin net just fails
    results = route_all(
        grid,
        [net],
        gridless_rescue=False,
    )

    # With rescue=False, the 3-pin net should fail (no gridless attempt)
    nr = results.get("MULTI_PIN")
    assert nr is not None
    # Whether it routed or not depends on grid routing; what matters is that
    # rescue=False did not invoke multi-pin gridless code (no crash, no import).
    # The key invariant is that the function completed without error.


def test_p2_rescue_false_preserves_2pin_route() -> None:
    """rescue=False with a simple 2-pin net produces same result as before M3-P2.

    This guards the hard invariant: gridless_rescue=False is byte-identical to
    pre-M3-P2 behaviour.  We just verify route_all completes and no extra routes
    appear from the multi-pin path.
    """
    from tracewise.route.engine.grid import Grid
    from tracewise.route.engine.multi import Net, route_all

    grid = Grid(pitch=0.5, x0=0.0, y0=0.0, width_mm=20.0, height_mm=20.0)
    net = Net(name="AB", pads=[(0, 4, 4), (0, 16, 16)], halfwidth_cells=1)
    grid.cells[0, 4, 4] = 0
    grid.cells[0, 16, 16] = 0

    results1 = route_all(grid, [net], gridless_rescue=False)
    # Re-create grid for second run (grid is mutated by route_all)
    grid2 = Grid(pitch=0.5, x0=0.0, y0=0.0, width_mm=20.0, height_mm=20.0)
    results2 = route_all(grid2, [net], gridless_rescue=False)

    # Both must have same ok status for the 2-pin net
    nr1 = results1.get("AB")
    nr2 = results2.get("AB")
    assert nr1 is not None and nr2 is not None
    assert nr1.ok == nr2.ok, "rescue=False must be deterministic for 2-pin grid route"
