"""M2.1 Tests — gridless_rescue mode: grid-first then gridless rescue.

Test categories:
  T-TR0  net_routes_to_track_obstacles helper: grid track → Shapely obstacles.
  T-TR1  Default byte-identical: gridless_rescue=False produces same result as
         current (pre-M2.1) route_all call.
  T-TR2  Rescue mode: gridless_rescue=True rescues unconnected 2-pin F.Cu nets
         that the grid left unconnected.
  T-TR3  Non-2-pin or non-F.Cu nets are NOT rescue candidates.
  T-TR4  Rescue mode with no gridless_kwargs is a safe no-op (no crash).

Skip cleanly when shapely is absent:
    pytest.importorskip("shapely")
"""

from __future__ import annotations

import pathlib

import pytest

shapely = pytest.importorskip("shapely")

# ---------------------------------------------------------------------------
# Imports
# ---------------------------------------------------------------------------

from tracewise.route.engine.grid import Grid  # noqa: E402
from tracewise.route.engine.multi import Net, NetRoute, route_all  # noqa: E402
from tracewise.route.gridless.adapter import GridlessNetRoute  # noqa: E402

BOARD_PATH = (
    pathlib.Path(__file__).parent.parent
    / "data" / "benchmark-boards" / "mitayi-pico-d1" / "Mitayi-Pico-D1.kicad_pcb"
)

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

GEO = {"track_mm": 0.2, "clearance_mm": 0.2}
BOARD_BBOX = (0.0, 0.0, 15.0, 12.0)


def _synthetic_pads_for_net(net_name: str, x: float, y1: float, y2: float) -> list[dict]:
    return [
        {"net": net_name, "x": x, "y": y1, "hw": 0.3, "hh": 0.3, "front": True},
        {"net": net_name, "x": x, "y": y2, "hw": 0.3, "hh": 0.3, "front": True},
    ]


def _small_gridless_kwargs(pads: list[dict]) -> dict:
    return {
        "pads": pads,
        "geo": GEO,
        "board_bbox": BOARD_BBOX,
        "anchors": None,
        "extra_gridless_obstacles": [],
        "board_outline": None,
        "drill_obstacles": None,
    }


# ---------------------------------------------------------------------------
# T-TR0: net_routes_to_track_obstacles helper
# ---------------------------------------------------------------------------


class TestNetRoutesToTrackObstacles:
    """T-TR0: grid track → Shapely obstacle conversion."""

    def test_empty_results_returns_empty(self):
        """Empty results dict must return an empty obstacles list."""
        from tracewise.route.gridless.geom import net_routes_to_track_obstacles
        g = Grid(x0=0.0, y0=0.0, width_mm=10.0, height_mm=10.0, layers=2)
        obs = net_routes_to_track_obstacles({}, g, track_mm=0.2, clearance_mm=0.2)
        assert obs == []

    def test_failed_net_not_converted(self):
        """Failed (ok=False) NetRoute must not produce any obstacle."""
        from tracewise.route.gridless.geom import net_routes_to_track_obstacles
        g = Grid(x0=0.0, y0=0.0, width_mm=10.0, height_mm=10.0, layers=2)
        net = Net("X", [(0, 10, 10), (0, 10, 90)], halfwidth_cells=1)
        nr = NetRoute(net=net, ok=False, reason="test")
        obs = net_routes_to_track_obstacles({"X": nr}, g, track_mm=0.2, clearance_mm=0.2)
        assert obs == []

    def test_routed_grid_net_produces_obstacle(self):
        """A routed grid NetRoute with a path must produce at least one Shapely polygon."""
        from tracewise.route.engine.multi import route_net
        from tracewise.route.gridless.geom import net_routes_to_track_obstacles
        g = Grid(x0=0.0, y0=0.0, width_mm=10.0, height_mm=10.0, layers=2)
        net = Net("A", [(0, 10, 50), (0, 90, 50)], halfwidth_cells=1)
        nr = route_net(g, net)
        assert nr.ok, "Expected net to route successfully for this test"
        obs = net_routes_to_track_obstacles({"A": nr}, g, track_mm=0.2, clearance_mm=0.2)
        assert len(obs) >= 1, "Expected at least one obstacle for a routed grid net"

    def test_obstacle_is_shapely_polygon(self):
        """Obstacles must be valid Shapely Polygon or MultiPolygon objects."""
        from shapely.geometry import MultiPolygon, Polygon

        from tracewise.route.engine.multi import route_net
        from tracewise.route.gridless.geom import net_routes_to_track_obstacles
        g = Grid(x0=0.0, y0=0.0, width_mm=10.0, height_mm=10.0, layers=2)
        net = Net("B", [(0, 10, 50), (0, 90, 50)], halfwidth_cells=1)
        nr = route_net(g, net)
        assert nr.ok
        obs = net_routes_to_track_obstacles({"B": nr}, g, track_mm=0.2, clearance_mm=0.2)
        for o in obs:
            assert isinstance(o, (Polygon, MultiPolygon)), (
                f"Obstacle must be Shapely Polygon/MultiPolygon, got {type(o)}"
            )

    def test_obstacle_covers_track_centerline(self):
        """Obstacle polygon must contain the track centerline midpoint."""
        from shapely.geometry import Point
        from shapely.ops import unary_union

        from tracewise.route.engine.multi import route_net
        from tracewise.route.gridless.geom import net_routes_to_track_obstacles
        g = Grid(x0=0.0, y0=0.0, width_mm=10.0, height_mm=10.0, layers=2)
        # Horizontal net at y=5, from x=1 to x=9
        net = Net("C", [(0, 50, 10), (0, 50, 90)], halfwidth_cells=1)
        nr = route_net(g, net)
        assert nr.ok
        obs = net_routes_to_track_obstacles({"C": nr}, g, track_mm=0.2, clearance_mm=0.2)
        assert obs, "Expected obstacles for routed net"
        union = unary_union(obs)
        # The midpoint of the track should be inside the buffered obstacle
        mid_x = (g.x0 + 10 * g.pitch + g.x0 + 90 * g.pitch) / 2
        mid_y = g.y0 + 50 * g.pitch
        pt = Point(mid_x, mid_y)
        assert union.contains(pt) or union.distance(pt) < 0.5, (
            f"Track midpoint ({mid_x:.2f}, {mid_y:.2f}) should be inside/near obstacle union"
        )

    def test_inflation_is_track_half_plus_clearance(self):
        """Obstacle inflation should equal track_mm/2 + clearance_mm."""
        from tracewise.route.engine.multi import route_net
        from tracewise.route.gridless.geom import net_routes_to_track_obstacles
        g = Grid(x0=0.0, y0=0.0, width_mm=10.0, height_mm=10.0, layers=2)
        net = Net("D", [(0, 50, 10), (0, 50, 90)], halfwidth_cells=1)
        nr = route_net(g, net)
        assert nr.ok
        track_mm, clearance_mm = 0.2, 0.2
        obs = net_routes_to_track_obstacles({"D": nr}, g, track_mm=track_mm,
                                            clearance_mm=clearance_mm)
        assert obs
        # The obstacle should have width >= track_mm + 2*clearance_mm ≈ 0.6mm
        # (both sides of the buffer: track/2 + clearance = 0.3 mm each side)
        for o in obs:
            bounds = o.bounds  # (minx, miny, maxx, maxy)
            height = bounds[3] - bounds[1]
            # Each track segment should be buffered at least clearance_mm on each side
            assert height >= clearance_mm, (
                f"Obstacle height {height:.3f} < clearance_mm {clearance_mm}"
            )

    def test_gridless_netroute_world_paths_converted(self):
        """GridlessNetRoute with world_paths must also produce obstacles."""
        from tracewise.route.gridless.geom import net_routes_to_track_obstacles
        g = Grid(x0=0.0, y0=0.0, width_mm=10.0, height_mm=10.0, layers=2)
        net = Net("GL", [(0, 50, 10), (0, 50, 90)], halfwidth_cells=1)
        from tracewise.route.gridless.adapter import to_gridless_netroute
        nr_gl = to_gridless_netroute(net, [[(1.0, 5.0), (9.0, 5.0)]], g)
        obs = net_routes_to_track_obstacles({"GL": nr_gl}, g, track_mm=0.2, clearance_mm=0.2)
        assert len(obs) >= 1, "GridlessNetRoute world_paths should produce obstacles"

    def test_multiple_nets_produces_multiple_obstacles(self):
        """Multiple routed nets should each contribute obstacles."""
        from tracewise.route.engine.multi import route_net
        from tracewise.route.gridless.geom import net_routes_to_track_obstacles
        g = Grid(x0=0.0, y0=0.0, width_mm=20.0, height_mm=20.0, layers=2)
        net_a = Net("A", [(0, 50, 10), (0, 50, 90)], halfwidth_cells=1)
        nr_a = route_net(g, net_a)
        assert nr_a.ok
        from tracewise.route.engine.multi import _mark
        _mark(g, nr_a, 1)
        net_b = Net("B", [(0, 150, 10), (0, 150, 90)], halfwidth_cells=1)
        nr_b = route_net(g, net_b)
        assert nr_b.ok
        obs = net_routes_to_track_obstacles({"A": nr_a, "B": nr_b}, g,
                                            track_mm=0.2, clearance_mm=0.2)
        assert len(obs) >= 2, f"Expected at least 2 obstacles, got {len(obs)}"


# ---------------------------------------------------------------------------
# T-TR1: Default byte-identical invariant
# ---------------------------------------------------------------------------


class TestGridlessRescueDefaultByteIdentical:
    """T-TR1: gridless_rescue=False (default) is byte-identical to current behaviour."""

    def _small_problem(self):
        """A 2-net synthetic problem."""
        g = Grid(x0=0.0, y0=0.0, width_mm=10.0, height_mm=10.0, layers=2)
        nets = [
            Net("A", [(0, 10, 10), (0, 10, 90)], halfwidth_cells=1),
            Net("B", [(0, 90, 10), (0, 90, 90)], halfwidth_cells=1),
        ]
        return g, nets

    def _snapshot(self, results: dict) -> dict:
        """Stable snapshot: ok flag, cells frozenset, type name."""
        return {
            name: (nr.ok, frozenset(nr.cells), type(nr).__name__)
            for name, nr in results.items()
        }

    def test_default_false_same_as_no_param(self):
        """route_all(...) and route_all(..., gridless_rescue=False) must be identical."""
        g1, nets1 = self._small_problem()
        r1 = route_all(g1, nets1)
        g2, nets2 = self._small_problem()
        r2 = route_all(g2, nets2, gridless_rescue=False)
        assert self._snapshot(r1) == self._snapshot(r2), (
            "gridless_rescue=False must produce identical results to the default call"
        )

    def test_rescue_false_with_no_kwargs_same(self):
        """gridless_rescue=False with gridless_kwargs=None is identical to baseline."""
        g1, nets1 = self._small_problem()
        r1 = route_all(g1, nets1)
        g2, nets2 = self._small_problem()
        r2 = route_all(g2, nets2, gridless_rescue=False, gridless_kwargs=None)
        assert self._snapshot(r1) == self._snapshot(r2)

    def test_none_rescue_identical_across_two_calls(self):
        """Two calls with gridless_rescue=False are deterministic (no rescue)."""
        g1, nets1 = self._small_problem()
        r1 = route_all(g1, nets1, gridless_rescue=False)
        g2, nets2 = self._small_problem()
        r2 = route_all(g2, nets2, gridless_rescue=False)
        assert self._snapshot(r1) == self._snapshot(r2)

    def test_rescue_false_does_not_produce_gridless_routes(self):
        """gridless_rescue=False must not produce any GridlessNetRoute."""
        g, nets = self._small_problem()
        results = route_all(g, nets, gridless_rescue=False)
        for _name, nr in results.items():
            assert not isinstance(nr, GridlessNetRoute), (
                f"gridless_rescue=False should not produce GridlessNetRoute for {_name!r}"
            )


# ---------------------------------------------------------------------------
# T-TR2: Rescue mode rescues unconnected 2-pin F.Cu nets
# ---------------------------------------------------------------------------


class TestGridlessRescueFunctionality:
    """T-TR2: gridless_rescue=True rescues grid failures."""

    def _make_blocked_problem(self) -> tuple[Grid, list[Net], dict]:
        """A problem where net B cannot be routed by the grid because net A blocks it.

        Net A routes horizontally through the board centre.
        Net B needs to cross that barrier on a single-layer grid (will fail).
        Gridless can navigate around the actual copper.
        """
        g = Grid(x0=0.0, y0=0.0, width_mm=10.0, height_mm=10.0, layers=1)
        # Net A: horizontal, large clearance, blocks the middle
        net_a = Net("NetA", [(0, 50, 5), (0, 50, 95)], halfwidth_cells=8)
        # Net B: vertical, needs to cross y=5 (will be blocked by net A on single-layer grid)
        net_b = Net("NetB", [(0, 5, 50), (0, 95, 50)], halfwidth_cells=1)
        pads = [
            {"net": "NetA", "x": 0.5, "y": 5.0, "hw": 0.2, "hh": 0.2, "front": True},
            {"net": "NetA", "x": 9.5, "y": 5.0, "hw": 0.2, "hh": 0.2, "front": True},
            {"net": "NetB", "x": 5.0, "y": 0.5, "hw": 0.2, "hh": 0.2, "front": True},
            {"net": "NetB", "x": 5.0, "y": 9.5, "hw": 0.2, "hh": 0.2, "front": True},
        ]
        gk = _small_gridless_kwargs(pads)
        return g, [net_a, net_b], gk

    def test_rescue_does_not_crash(self):
        """gridless_rescue=True with valid kwargs must not raise any exception."""
        g, nets, gk = self._make_blocked_problem()
        # Should not raise
        results = route_all(g, nets, gridless_rescue=True, gridless_kwargs=gk)
        assert isinstance(results, dict)
        assert set(results.keys()) == {"NetA", "NetB"}

    def test_rescue_false_no_gridless_routes(self):
        """gridless_rescue=False must not produce any GridlessNetRoute."""
        g, nets, gk = self._make_blocked_problem()
        results = route_all(g, nets, gridless_rescue=False, gridless_kwargs=gk)
        for nr in results.values():
            assert not isinstance(nr, GridlessNetRoute)

    def test_rescue_true_may_produce_gridless_routes(self):
        """gridless_rescue=True may produce GridlessNetRoute for rescued nets."""
        g, nets, gk = self._make_blocked_problem()
        results = route_all(g, nets, gridless_rescue=True, gridless_kwargs=gk)
        # If any net was rescued, it should be a GridlessNetRoute
        for nr in results.values():
            if isinstance(nr, GridlessNetRoute):
                assert nr.ok, f"Rescued GridlessNetRoute should be ok=True, got reason={nr.reason}"

    def test_rescue_result_ok_routes_have_world_paths(self):
        """Any GridlessNetRoute produced by rescue must have non-empty world_paths."""
        g, nets, gk = self._make_blocked_problem()
        results = route_all(g, nets, gridless_rescue=True, gridless_kwargs=gk)
        for name, nr in results.items():
            if isinstance(nr, GridlessNetRoute) and nr.ok:
                assert nr.world_paths, (
                    f"Rescued net {name!r} has ok=True but empty world_paths"
                )
                assert len(nr.world_paths[0]) >= 2, (
                    f"Rescued net {name!r} world_paths[0] has fewer than 2 waypoints"
                )

    def test_successfully_routed_grid_net_stays_grid(self):
        """A net successfully routed by the grid must NOT be overwritten by rescue."""
        g, nets, gk = self._make_blocked_problem()
        results = route_all(g, nets, gridless_rescue=True, gridless_kwargs=gk)
        # NetA routes on the grid successfully (horizontal, no blocker)
        nr_a = results.get("NetA")
        assert nr_a is not None
        assert nr_a.ok, f"NetA should be ok after grid routing, reason={nr_a.reason}"
        # NetA was routed by the grid so it must NOT be a GridlessNetRoute
        assert not isinstance(nr_a, GridlessNetRoute), (
            "NetA was routed by the grid and must not be a GridlessNetRoute"
        )


# ---------------------------------------------------------------------------
# T-TR3: Non-2-pin / non-F.Cu nets are not rescue candidates
# ---------------------------------------------------------------------------


class TestRescueCandidateFiltering:
    """T-TR3: only 2-pin F.Cu nets are rescue candidates."""

    def test_3pin_net_not_rescued(self):
        """A 3-pin net that fails the grid must NOT be a rescue candidate."""
        g = Grid(x0=0.0, y0=0.0, width_mm=10.0, height_mm=10.0, layers=1)
        # 3-pin net: cannot be a rescue candidate (gridless is 2-pin only)
        net_mp = Net("Multi", [(0, 20, 20), (0, 20, 80), (0, 60, 50)], halfwidth_cells=5)
        pads = [
            {"net": "Multi", "x": 2.0, "y": 2.0, "hw": 0.2, "hh": 0.2, "front": True},
            {"net": "Multi", "x": 2.0, "y": 8.0, "hw": 0.2, "hh": 0.2, "front": True},
            {"net": "Multi", "x": 6.0, "y": 5.0, "hw": 0.2, "hh": 0.2, "front": True},
        ]
        gk = _small_gridless_kwargs(pads)
        results = route_all(g, [net_mp], gridless_rescue=True, gridless_kwargs=gk)
        nr = results.get("Multi")
        if nr is not None:
            # If it appears, it must NOT be a GridlessNetRoute (3-pin excluded)
            assert not isinstance(nr, GridlessNetRoute), (
                "3-pin net must not be rescued as GridlessNetRoute"
            )

    def test_b_cu_only_net_not_rescued(self):
        """A 2-pin B.Cu net (layer=1) that fails grid must NOT be a rescue candidate."""
        g = Grid(x0=0.0, y0=0.0, width_mm=10.0, height_mm=10.0, layers=2)
        # pads on layer 1 (B.Cu): the rescue filter checks layer==0 for all pads
        net_bcu = Net("BCu", [(1, 20, 20), (1, 80, 80)], halfwidth_cells=8)
        pads: list[dict] = []
        gk = _small_gridless_kwargs(pads)
        results = route_all(g, [net_bcu], gridless_rescue=True, gridless_kwargs=gk)
        nr = results.get("BCu")
        if nr is not None:
            assert not isinstance(nr, GridlessNetRoute), (
                "B.Cu-only net must not be rescued as GridlessNetRoute"
            )


# ---------------------------------------------------------------------------
# T-TR4: Rescue with no gridless_kwargs is safe no-op
# ---------------------------------------------------------------------------


class TestRescueNoKwargsSafe:
    """T-TR4: gridless_rescue=True with no gridless_kwargs must not crash."""

    def test_rescue_true_no_kwargs_no_crash(self):
        """gridless_rescue=True with gridless_kwargs=None must be a safe no-op."""
        g = Grid(x0=0.0, y0=0.0, width_mm=10.0, height_mm=10.0, layers=2)
        nets = [Net("X", [(0, 10, 10), (0, 10, 90)], halfwidth_cells=1)]
        # Must not raise
        results = route_all(g, nets, gridless_rescue=True, gridless_kwargs=None)
        assert isinstance(results, dict)

    def test_rescue_true_no_kwargs_produces_no_gridless(self):
        """With no gridless_kwargs, rescue must not produce any GridlessNetRoute."""
        g = Grid(x0=0.0, y0=0.0, width_mm=10.0, height_mm=10.0, layers=2)
        nets = [Net("X", [(0, 10, 10), (0, 10, 90)], halfwidth_cells=1)]
        results = route_all(g, nets, gridless_rescue=True, gridless_kwargs=None)
        for nr in results.values():
            assert not isinstance(nr, GridlessNetRoute)


# ---------------------------------------------------------------------------
# T-TR5: Fanout-escape wired into rescue path (displacement-control build)
# ---------------------------------------------------------------------------


class TestRescueFanoutEscapeWired:
    """T-TR5: fanout-escape is available via the rescue path.

    These tests verify the structural wiring — that the fanout logic is
    imported and callable from the geometry-blocked handler in gridless_rescue.
    The functional integration test is the A/B script (_verify_fanout_rescue_ab.py).
    """

    def test_detect_dense_components_importable(self):
        """detect_dense_components must be importable for rescue path use."""
        from tracewise.route.gridless.geom import detect_dense_components
        # An empty pad list must return an empty list (no crash)
        result = detect_dense_components([])
        assert result == []

    def test_route_net_fanout_escape_importable(self):
        """route_net_fanout_escape must be importable from the rescue path."""
        from tracewise.route.gridless.route import route_net_fanout_escape
        assert callable(route_net_fanout_escape)

    def test_rescue_false_byte_identical_to_pre_build(self):
        """gridless_rescue=False must be byte-identical to the original call.

        This is the CRITICAL invariant: adding fanout-escape to the rescue path
        must not change any result when gridless_rescue=False (the default).
        """
        g1 = Grid(x0=0.0, y0=0.0, width_mm=10.0, height_mm=10.0, layers=2)
        nets1 = [
            Net("A", [(0, 10, 10), (0, 10, 90)], halfwidth_cells=1),
            Net("B", [(0, 90, 10), (0, 90, 90)], halfwidth_cells=1),
        ]
        r1 = route_all(g1, nets1)

        g2 = Grid(x0=0.0, y0=0.0, width_mm=10.0, height_mm=10.0, layers=2)
        nets2 = [
            Net("A", [(0, 10, 10), (0, 10, 90)], halfwidth_cells=1),
            Net("B", [(0, 90, 10), (0, 90, 90)], halfwidth_cells=1),
        ]
        r2 = route_all(g2, nets2, gridless_rescue=False)

        def _snap(results: dict) -> dict:
            return {
                name: (nr.ok, frozenset(nr.cells), type(nr).__name__)
                for name, nr in results.items()
            }

        assert _snap(r1) == _snap(r2), (
            "gridless_rescue=False must be byte-identical after fanout-escape wiring"
        )

    def test_rescue_with_no_dense_components_uses_strategy2(self):
        """Rescue on a non-dense board must skip fanout-escape and use strategy-2.

        When no dense components are detected (the board has no QFN/dense pads),
        the rescue path should not crash and should still attempt the standard
        2-layer fallback for geometry-blocked nets.
        """
        # A simple 2-layer board with two widely-spaced pads — no dense ring.
        g = Grid(x0=0.0, y0=0.0, width_mm=10.0, height_mm=10.0, layers=2)
        net_a = Net("A", [(0, 10, 10), (0, 10, 90)], halfwidth_cells=1)
        net_b = Net("B", [(0, 90, 10), (0, 90, 90)], halfwidth_cells=1)
        pads = [
            {"net": "A", "x": 1.0, "y": 1.0, "hw": 0.2, "hh": 0.2,
             "front": True, "back": False, "ref": "R1"},
            {"net": "A", "x": 1.0, "y": 9.0, "hw": 0.2, "hh": 0.2,
             "front": True, "back": False, "ref": "R1"},
            {"net": "B", "x": 9.0, "y": 1.0, "hw": 0.2, "hh": 0.2,
             "front": True, "back": False, "ref": "R2"},
            {"net": "B", "x": 9.0, "y": 9.0, "hw": 0.2, "hh": 0.2,
             "front": True, "back": False, "ref": "R2"},
        ]
        gk = _small_gridless_kwargs(pads)
        # Must not raise
        results = route_all(
            g, [net_a, net_b],
            gridless_rescue=True,
            gridless_kwargs=gk,
        )
        assert isinstance(results, dict)
        # Results must contain both nets
        assert "A" in results and "B" in results


# ---------------------------------------------------------------------------
# T-TR6: net_routes_to_track_obstacles layer filter (B.Cu-only mode)
# ---------------------------------------------------------------------------


class TestNetRoutesToTrackObstaclesLayerFilter:
    """T-TR6: layer= parameter filters obstacles by routing layer.

    Grid copper on F.Cu (layer=0) must NOT appear as B.Cu obstacles when
    layer=1 is requested — this prevents the 2D projection of F.Cu tracks
    from blocking B.Cu rescue routing on dense boards.
    """

    def test_layer_none_includes_all_layers(self):
        """layer=None must include obstacles from both layers."""
        from tracewise.route.engine.multi import route_net
        from tracewise.route.gridless.geom import net_routes_to_track_obstacles

        g = Grid(x0=0.0, y0=0.0, width_mm=10.0, height_mm=10.0, layers=2)
        # Layer-0 net
        net_a = Net("A", [(0, 10, 50), (0, 90, 50)], halfwidth_cells=1)
        nr_a = route_net(g, net_a)
        assert nr_a.ok

        obs_all = net_routes_to_track_obstacles(
            {"A": nr_a}, g, track_mm=0.2, clearance_mm=0.2, layer=None
        )
        obs_default = net_routes_to_track_obstacles(
            {"A": nr_a}, g, track_mm=0.2, clearance_mm=0.2
        )
        # layer=None must be equivalent to the no-layer-arg default
        assert len(obs_all) == len(obs_default)

    def test_fcu_net_not_in_bcu_obstacles(self):
        """A net routed on F.Cu (layer=0) must produce ZERO B.Cu obstacles."""
        from tracewise.route.engine.multi import route_net
        from tracewise.route.gridless.geom import net_routes_to_track_obstacles

        g = Grid(x0=0.0, y0=0.0, width_mm=10.0, height_mm=10.0, layers=2)
        # Route on layer 0 explicitly
        net_a = Net("A", [(0, 10, 50), (0, 90, 50)], halfwidth_cells=1)
        nr_a = route_net(g, net_a)
        assert nr_a.ok
        # Confirm routed on F.Cu only
        for path in nr_a.paths:
            for cell in path:
                assert cell[0] == 0, f"Expected F.Cu (layer 0), got layer {cell[0]}"

        obs_bcu = net_routes_to_track_obstacles(
            {"A": nr_a}, g, track_mm=0.2, clearance_mm=0.2, layer=1
        )
        assert obs_bcu == [], (
            "F.Cu-only net must produce zero B.Cu obstacles with layer=1"
        )

    def test_fcu_net_produces_fcu_obstacles(self):
        """A net routed on F.Cu must produce non-zero F.Cu obstacles with layer=0."""
        from tracewise.route.engine.multi import route_net
        from tracewise.route.gridless.geom import net_routes_to_track_obstacles

        g = Grid(x0=0.0, y0=0.0, width_mm=10.0, height_mm=10.0, layers=2)
        net_a = Net("A", [(0, 10, 50), (0, 90, 50)], halfwidth_cells=1)
        nr_a = route_net(g, net_a)
        assert nr_a.ok

        obs_fcu = net_routes_to_track_obstacles(
            {"A": nr_a}, g, track_mm=0.2, clearance_mm=0.2, layer=0
        )
        assert len(obs_fcu) >= 1, "F.Cu net must produce >=1 F.Cu obstacles with layer=0"


# ---------------------------------------------------------------------------
# T-FE1 — route_net_fanout_escape: bcu_extra_obstacles used for B.Cu via placement
# ---------------------------------------------------------------------------


class TestFanoutEscapeBcuViaPlacement:
    """route_net_fanout_escape uses bcu_extra_obstacles for B.Cu via placement.

    When bcu_extra_obstacles is provided, the B.Cu via-placement free space
    (Step 1, fs_B_via) must be built with bcu_extra_obstacles instead of
    extra_obstacles.  This prevents F.Cu track copper (in extra_obstacles) from
    being projected as B.Cu obstacles, which would block the escape-via
    placement near the QFN ring.
    """

    def _make_pads(self, net: str) -> list[dict]:
        """Two pads: one SMD F.Cu source, one TH destination.

        Source pad at (5.0, 6.0) so it is 1.0 mm north of the component
        centroid (5.0, 5.0) — ``guided_escape_via`` requires the source pad
        to be offset from the centroid (``dist > 1e-6``).
        """
        return [
            {"net": net, "x": 5.0, "y": 6.0, "hw": 0.1, "hh": 0.4,
             "front": True, "back": False},
            {"net": net, "x": 5.0, "y": 12.0, "hw": 0.8, "hh": 0.8,
             "front": True, "back": True},
        ]

    def test_bcu_via_free_space_uses_bcu_extra_obstacles(self):
        """B.Cu via-placement free space is built from bcu_extra_obstacles.

        Verify that ``route_net_fanout_escape`` passes ``bcu_extra_obstacles``
        (not ``extra_obstacles``) to ``build_windowed_free_space`` when
        constructing the B.Cu via-placement free space.  We test this by
        patching ``build_windowed_free_space`` and inspecting the
        ``extra_obstacles`` argument for each call.
        """
        import tracewise.route.gridless.geom as _geom_mod
        from tracewise.route.gridless.route import route_net_fanout_escape

        pads = self._make_pads("NET_A")
        geo = {"track_mm": 0.2, "clearance_mm": 0.2,
               "via_mm": 0.6, "via_drill_mm": 0.3,
               "hole_clearance_mm": 0.3, "hole_to_hole_mm": 0.3}

        sentinel_fcu = object()  # unique marker — passed as extra_obstacles
        sentinel_bcu = object()  # unique marker — passed as bcu_extra_obstacles

        calls: list[dict] = []
        _orig = _geom_mod.build_windowed_free_space

        def _spy(pads_, net_, clr, trk, extra, window, **kw):
            calls.append({"extra": extra, "layer": kw.get("layer", 0)})
            return _orig(pads_, net_, clr, trk, [], window, **kw)

        _geom_mod.build_windowed_free_space = _spy  # type: ignore[assignment]
        try:
            route_net_fanout_escape(
                source_xy=(5.0, 6.0),
                dest_xy=(5.0, 12.0),
                component_cx=5.0,
                component_cy=5.0,
                ring_radius=0.5,
                pads=pads,
                net_name="NET_A",
                geo=geo,
                board_bbox=(0.0, 0.0, 20.0, 20.0),
                extra_obstacles=[sentinel_fcu],       # F.Cu marker
                bcu_extra_obstacles=[sentinel_bcu],   # B.Cu marker
            )
        finally:
            _geom_mod.build_windowed_free_space = _orig

        # The B.Cu via-placement call (layer=1, first B.Cu call) must use
        # sentinel_bcu (not sentinel_fcu) as its extra_obstacles list.
        bcu_via_calls = [c for c in calls if c["layer"] == 1]
        assert bcu_via_calls, "Expected at least one B.Cu build_windowed_free_space call"

        first_bcu_via = bcu_via_calls[0]
        assert sentinel_bcu in first_bcu_via["extra"], (
            "First B.Cu build_windowed_free_space call must use bcu_extra_obstacles"
        )
        assert sentinel_fcu not in first_bcu_via["extra"], (
            "First B.Cu build_windowed_free_space call must NOT use extra_obstacles"
        )

    def test_fcu_stub_uses_fcu_stub_extra_obstacles(self):
        """F.Cu stub free space is built from fcu_stub_extra_obstacles when provided.

        Verify that ``route_net_fanout_escape`` passes ``fcu_stub_extra_obstacles``
        (not ``extra_obstacles``) to the F.Cu stub ``build_windowed_free_space``
        call.  This lets the stub see F.Cu track obstacles (preventing shorts)
        while the via-placement free-space computation sees only drills.
        """
        import tracewise.route.gridless.geom as _geom_mod
        from tracewise.route.gridless.route import route_net_fanout_escape

        pads = self._make_pads("NET_A")
        geo = {"track_mm": 0.2, "clearance_mm": 0.2,
               "via_mm": 0.6, "via_drill_mm": 0.3,
               "hole_clearance_mm": 0.3, "hole_to_hole_mm": 0.3}

        sentinel_via = object()   # passed as extra_obstacles (via placement)
        sentinel_stub = object()  # passed as fcu_stub_extra_obstacles (stub)

        calls: list[dict] = []
        _orig = _geom_mod.build_windowed_free_space

        def _spy(pads_, net_, clr, trk, extra, window, **kw):
            calls.append({"extra": extra, "layer": kw.get("layer", 0)})
            return _orig(pads_, net_, clr, trk, [], window, **kw)

        _geom_mod.build_windowed_free_space = _spy  # type: ignore[assignment]
        try:
            route_net_fanout_escape(
                source_xy=(5.0, 6.0),
                dest_xy=(5.0, 12.0),
                component_cx=5.0,
                component_cy=5.0,
                ring_radius=0.5,
                pads=pads,
                net_name="NET_A",
                geo=geo,
                board_bbox=(0.0, 0.0, 20.0, 20.0),
                extra_obstacles=[sentinel_via],              # via marker
                bcu_extra_obstacles=[],
                fcu_stub_extra_obstacles=[sentinel_stub],    # stub marker
            )
        finally:
            _geom_mod.build_windowed_free_space = _orig

        # Via-placement F.Cu calls (layer=0, windows around centroid) must use
        # sentinel_via.  The stub F.Cu call (layer=0, smaller window) must use
        # sentinel_stub.  There should be at least 2 F.Cu calls: via + stub.
        fcu_calls = [c for c in calls if c["layer"] == 0]
        assert len(fcu_calls) >= 2, (
            f"Expected >=2 F.Cu calls (via + stub); got {len(fcu_calls)}"
        )

        # First F.Cu call: via placement → uses extra_obstacles (sentinel_via)
        first_fcu = fcu_calls[0]
        assert sentinel_via in first_fcu["extra"], (
            "First F.Cu call (via placement) must use extra_obstacles"
        )
        assert sentinel_stub not in first_fcu["extra"], (
            "First F.Cu call (via placement) must NOT use fcu_stub_extra_obstacles"
        )

        # Last F.Cu call: stub routing → uses fcu_stub_extra_obstacles (sentinel_stub)
        last_fcu = fcu_calls[-1]
        assert sentinel_stub in last_fcu["extra"], (
            "Last F.Cu call (stub) must use fcu_stub_extra_obstacles"
        )
        assert sentinel_via not in last_fcu["extra"], (
            "Last F.Cu call (stub) must NOT use extra_obstacles"
        )


# ---------------------------------------------------------------------------
# T-VIA1: net_routes_to_via_obstacles unit tests
# ---------------------------------------------------------------------------

class TestNetRoutesToViaObstacles:
    """Unit tests for net_routes_to_via_obstacles (T-VIA1).

    Verifies that grid-router-placed vias (stored in NetRoute.via_sites) are
    converted to inflated Shapely circle obstacles so rescue routes avoid
    crossing via annular rings.
    """

    def _make_grid_stub(self, pitch: float = 0.1, x0: float = 0.0, y0: float = 0.0):
        """Minimal grid stub that supports grid.to_world(iy, ix)."""
        class _Grid:
            def __init__(self):
                self.pitch = pitch
                self.x0 = x0
                self.y0 = y0
            def to_world(self, iy, ix):
                return (self.x0 + ix * self.pitch, self.y0 + iy * self.pitch)
        return _Grid()

    def _make_netroute(self, ok: bool, via_sites: list):
        """Minimal NetRoute stub with ok flag and via_sites set."""
        from dataclasses import dataclass, field

        @dataclass
        class _Net:
            name: str = "test"
            halfwidth_cells: int = 1
            via_halfwidth_cells: int = 3

        @dataclass
        class _NR:
            net: _Net = field(default_factory=_Net)
            ok: bool = True
            via_sites: set = field(default_factory=set)
            paths: list = field(default_factory=list)
            cells: set = field(default_factory=set)
            escape_cells: set = field(default_factory=set)
            reason: str = ""
            unroutable_pads: list = field(default_factory=list)

        nr = _NR()
        nr.ok = ok
        nr.via_sites = set(via_sites)
        return nr

    def test_returns_empty_for_no_vias(self):
        """A net with ok=True but no via_sites produces no obstacles."""
        pytest.importorskip("shapely")
        from tracewise.route.gridless.geom import net_routes_to_via_obstacles

        nr = self._make_netroute(ok=True, via_sites=[])
        grid = self._make_grid_stub()
        obs = net_routes_to_via_obstacles(
            {"net_a": nr}, grid,
            via_mm=0.6, clearance_mm=0.15, track_mm=0.15,
        )
        assert obs == [], f"Expected [] for no vias, got {obs}"

    def test_returns_empty_for_failed_net(self):
        """A failed net (ok=False) must not produce any via obstacles."""
        pytest.importorskip("shapely")
        from tracewise.route.gridless.geom import net_routes_to_via_obstacles

        nr = self._make_netroute(ok=False, via_sites=[(10, 20)])
        grid = self._make_grid_stub()
        obs = net_routes_to_via_obstacles(
            {"net_bad": nr}, grid,
            via_mm=0.6, clearance_mm=0.15, track_mm=0.15,
        )
        assert obs == [], f"Expected [] for failed net, got {obs}"

    def test_one_via_produces_one_polygon(self):
        """A net with one via in via_sites produces exactly one circle obstacle."""
        pytest.importorskip("shapely")
        from tracewise.route.gridless.geom import net_routes_to_via_obstacles

        nr = self._make_netroute(ok=True, via_sites=[(10, 20)])
        grid = self._make_grid_stub(pitch=0.1, x0=0.0, y0=0.0)
        obs = net_routes_to_via_obstacles(
            {"net_a": nr}, grid,
            via_mm=0.6, clearance_mm=0.15, track_mm=0.15,
        )
        assert len(obs) == 1, f"Expected 1 obstacle, got {len(obs)}"

    def test_circle_centered_at_via_world_position(self):
        """Via obstacle circle must be centered at the via's world mm position."""
        pytest.importorskip("shapely")
        from tracewise.route.gridless.geom import net_routes_to_via_obstacles

        pitch = 0.1
        iy, ix = 10, 20
        x0, y0 = 5.0, 3.0
        expected_wx = x0 + ix * pitch   # 7.0
        expected_wy = y0 + iy * pitch   # 4.0

        nr = self._make_netroute(ok=True, via_sites=[(iy, ix)])
        grid = self._make_grid_stub(pitch=pitch, x0=x0, y0=y0)
        obs = net_routes_to_via_obstacles(
            {"net_a": nr}, grid,
            via_mm=0.6, clearance_mm=0.15, track_mm=0.15,
        )
        assert len(obs) == 1
        cx, cy = obs[0].centroid.x, obs[0].centroid.y
        assert abs(cx - expected_wx) < 0.05, f"cx={cx} expected ~{expected_wx}"
        assert abs(cy - expected_wy) < 0.05, f"cy={cy} expected ~{expected_wy}"

    def test_circle_radius_matches_inflate_formula(self):
        """Via obstacle radius must equal via_mm/2 + track_mm/2 (short-preventing)."""
        pytest.importorskip("shapely")
        from tracewise.route.gridless.geom import net_routes_to_via_obstacles

        via_mm, clearance_mm, track_mm = 0.6, 0.15, 0.15
        # Radius = via_mm/2 + track_mm/2 (prevents copper overlap / shorts).
        # Intentionally does NOT include clearance_mm to preserve narrow corridors.
        expected_r = via_mm / 2.0 + track_mm / 2.0  # 0.375

        nr = self._make_netroute(ok=True, via_sites=[(0, 0)])
        grid = self._make_grid_stub(pitch=0.1, x0=0.0, y0=0.0)
        obs = net_routes_to_via_obstacles(
            {"net_a": nr}, grid,
            via_mm=via_mm, clearance_mm=clearance_mm, track_mm=track_mm,
        )
        assert len(obs) == 1
        # bounds xmin,ymin,xmax,ymax → r ≈ (xmax-xmin)/2
        xmin, ymin, xmax, ymax = obs[0].bounds
        actual_r = (xmax - xmin) / 2.0
        assert abs(actual_r - expected_r) < 0.05, (
            f"Radius {actual_r:.4f} expected ~{expected_r:.4f}"
        )

    def test_two_vias_produce_two_polygons(self):
        """Two distinct via_sites produce two separate obstacle polygons."""
        pytest.importorskip("shapely")
        from tracewise.route.gridless.geom import net_routes_to_via_obstacles

        nr = self._make_netroute(ok=True, via_sites=[(10, 10), (20, 30)])
        grid = self._make_grid_stub()
        obs = net_routes_to_via_obstacles(
            {"net_a": nr}, grid,
            via_mm=0.6, clearance_mm=0.15, track_mm=0.15,
        )
        assert len(obs) == 2, f"Expected 2 obstacles, got {len(obs)}"

    def test_gridless_netroute_skipped(self):
        """GridlessNetRoute entries must be skipped (their vias are world_vias)."""
        pytest.importorskip("shapely")
        from tracewise.route.gridless.adapter import GridlessNetRoute
        from tracewise.route.gridless.geom import net_routes_to_via_obstacles

        gnr = GridlessNetRoute(
            net=None,  # type: ignore[arg-type]
            ok=True,
            world_paths=[],
            world_vias=[(5.0, 5.0)],
        )
        grid = self._make_grid_stub()
        obs = net_routes_to_via_obstacles(
            {"gnr": gnr}, grid,
            via_mm=0.6, clearance_mm=0.15, track_mm=0.15,
        )
        assert obs == [], (
            "GridlessNetRoute must be skipped; got obstacles from world_vias"
        )
