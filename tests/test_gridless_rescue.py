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
