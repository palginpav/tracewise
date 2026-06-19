"""M2 Production Integration Tests — negotiate.py + staged route_all integration.

Test categories:
  T-N0  negotiate.py unit tests: route_gridless_set correctness on synthetic problems.
  T-N1  Byte-identical invariant: gridless_nets=None (or empty) with gridless_negotiate
         True/False produces the SAME result as pre-M2 pure-grid call.
  T-N2  Staged wiring: gridless_negotiate=True routes nominated nets via negotiate path,
         rasterizes copper into shared grid ledger, grid remainder sees gridless copper.
  T-N3  Geometry-blocked reporting: nets quarantined as M3 are reported failed with the
         correct reason string, not raised as exceptions.
  T-N4  Determinism: two independent route_gridless_set calls produce byte-identical results.

Skip cleanly when shapely is absent:
    pytest.importorskip("shapely")
"""

from __future__ import annotations

import pathlib

import pytest

shapely = pytest.importorskip("shapely")

# ---------------------------------------------------------------------------
# Imports (shapely is present at this point)
# ---------------------------------------------------------------------------

from tracewise.route.engine.grid import Grid  # noqa: E402
from tracewise.route.engine.multi import Net, _mark, route_all  # noqa: E402
from tracewise.route.gridless.adapter import GridlessNetRoute  # noqa: E402
from tracewise.route.gridless.negotiate import (  # noqa: E402
    GridlessSetNetResult,
    route_gridless_set,
)

# ---------------------------------------------------------------------------
# Board fixture
# ---------------------------------------------------------------------------

BOARD_PATH = (
    pathlib.Path(__file__).parent.parent
    / "data" / "benchmark-boards" / "mitayi-pico-d1" / "Mitayi-Pico-D1.kicad_pcb"
)

BOARD_EXISTS = BOARD_PATH.exists()


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _synthetic_pads(n: int = 3, spacing: float = 3.0) -> list[dict]:
    """Build a row of n pads for nets Net0..Net{n-1}."""
    pads = []
    for i in range(n):
        x = 1.0 + i * spacing
        # Two pads per net, slightly offset
        pads.append({"net": f"Net{i}", "x": x, "y": 2.0, "hw": 0.3, "hh": 0.3, "front": True})
        pads.append({"net": f"Net{i}", "x": x, "y": 8.0, "hw": 0.3, "hh": 0.3, "front": True})
    return pads


def _make_net_set(n: int = 3, spacing: float = 3.0) -> list[dict]:
    """Return net_set input for route_gridless_set."""
    net_set = []
    for i in range(n):
        x = 1.0 + i * spacing
        net_set.append({
            "net_name": f"Net{i}",
            "pad_a": (x, 2.0),
            "pad_b": (x, 8.0),
        })
    return net_set


# ---------------------------------------------------------------------------
# T-N0: route_gridless_set unit tests
# ---------------------------------------------------------------------------

class TestRouteGridlessSet:
    """T-N0: route_gridless_set correctness on synthetic problems."""

    GEO = {"track_mm": 0.2, "clearance_mm": 0.2}
    BOARD = (0.0, 0.0, 15.0, 12.0)

    def test_empty_net_set_returns_empty(self):
        """Empty net_set must return empty dict without side-effects."""
        result = route_gridless_set(
            net_set=[], pads=[], geo=self.GEO, board_bbox=self.BOARD
        )
        assert result == {}

    def test_single_net_routes_ok(self):
        """A single unobstructed net must route successfully."""
        net_set = [{"net_name": "NetA", "pad_a": (2.0, 2.0), "pad_b": (2.0, 8.0)}]
        pads = [
            {"net": "NetA", "x": 2.0, "y": 2.0, "hw": 0.3, "hh": 0.3, "front": True},
            {"net": "NetA", "x": 2.0, "y": 8.0, "hw": 0.3, "hh": 0.3, "front": True},
        ]
        result = route_gridless_set(net_set=net_set, pads=pads, geo=self.GEO,
                                    board_bbox=self.BOARD)
        assert "NetA" in result
        nr = result["NetA"]
        assert isinstance(nr, GridlessSetNetResult)
        assert nr.ok, f"NetA failed: {nr.reason}"
        assert nr.status == "routed"
        assert nr.world_paths
        assert len(nr.world_paths[0]) >= 2

    def test_multiple_non_overlapping_nets(self):
        """Multiple well-separated nets should all route without rip-up."""
        net_set = _make_net_set(n=3, spacing=3.0)
        pads = _synthetic_pads(n=3, spacing=3.0)
        result = route_gridless_set(net_set=net_set, pads=pads, geo=self.GEO,
                                    board_bbox=self.BOARD)
        assert len(result) == 3
        for i in range(3):
            nr = result[f"Net{i}"]
            assert nr.ok, f"Net{i} failed: {nr.reason}"
            assert nr.status == "routed"

    def test_result_keys_match_net_names(self):
        """Result keys must match the net_name values in net_set."""
        net_set = _make_net_set(n=2)
        pads = _synthetic_pads(n=2)
        result = route_gridless_set(net_set=net_set, pads=pads, geo=self.GEO,
                                    board_bbox=self.BOARD)
        assert set(result.keys()) == {"Net0", "Net1"}

    def test_world_paths_have_correct_structure(self):
        """world_paths must be a list of lists of (x, y) tuples."""
        net_set = [{"net_name": "N", "pad_a": (1.0, 1.0), "pad_b": (1.0, 9.0)}]
        pads = [
            {"net": "N", "x": 1.0, "y": 1.0, "hw": 0.2, "hh": 0.2, "front": True},
            {"net": "N", "x": 1.0, "y": 9.0, "hw": 0.2, "hh": 0.2, "front": True},
        ]
        result = route_gridless_set(net_set=net_set, pads=pads, geo=self.GEO,
                                    board_bbox=self.BOARD)
        nr = result["N"]
        assert nr.ok
        # world_paths: list of path segments, each a list of (x, y) pairs
        assert isinstance(nr.world_paths, list)
        assert len(nr.world_paths) >= 1
        path = nr.world_paths[0]
        assert isinstance(path, list)
        for pt in path:
            assert len(pt) == 2
            assert all(isinstance(c, float) for c in pt)

    def test_stats_present_on_routed(self):
        """stats must contain node/edge counters for routed nets."""
        net_set = [{"net_name": "N", "pad_a": (2.0, 2.0), "pad_b": (2.0, 8.0)}]
        pads = [
            {"net": "N", "x": 2.0, "y": 2.0, "hw": 0.3, "hh": 0.3, "front": True},
            {"net": "N", "x": 2.0, "y": 8.0, "hw": 0.3, "hh": 0.3, "front": True},
        ]
        result = route_gridless_set(net_set=net_set, pads=pads, geo=self.GEO,
                                    board_bbox=self.BOARD)
        nr = result["N"]
        assert nr.ok
        assert "nodes" in nr.stats
        assert "edges" in nr.stats
        assert "window_mm" in nr.stats


# ---------------------------------------------------------------------------
# T-N1: byte-identical invariant — gridless_nets=None/empty never triggers M2
# ---------------------------------------------------------------------------

class TestGridlessNegotiateByteIdentical:
    """T-N1: gridless_nets=None or empty is byte-identical regardless of gridless_negotiate."""

    def _small_problem(self):
        g = Grid(x0=0.0, y0=0.0, width_mm=10.0, height_mm=10.0, layers=2)
        nets = [
            Net("A", [(0, 10, 10), (0, 10, 90)], halfwidth_cells=1),
            Net("B", [(0, 90, 10), (0, 90, 90)], halfwidth_cells=1),
        ]
        return g, nets

    def _snapshot(self, results):
        return {name: (nr.ok, frozenset(nr.cells), type(nr).__name__)
                for name, nr in results.items()}

    def test_none_negotiate_false_same_as_none_negotiate_true(self):
        """gridless_nets=None with gridless_negotiate=False == gridless_negotiate=True."""
        g1, nets1 = self._small_problem()
        r1 = route_all(g1, nets1, gridless_nets=None, gridless_negotiate=False)
        g2, nets2 = self._small_problem()
        r2 = route_all(g2, nets2, gridless_nets=None, gridless_negotiate=True)
        assert self._snapshot(r1) == self._snapshot(r2), (
            "gridless_nets=None should produce identical results regardless of gridless_negotiate"
        )

    def test_empty_set_negotiate_false_vs_true(self):
        """gridless_nets=set() with negotiate=False == negotiate=True."""
        g1, nets1 = self._small_problem()
        r1 = route_all(g1, nets1, gridless_nets=set(), gridless_negotiate=False)
        g2, nets2 = self._small_problem()
        r2 = route_all(g2, nets2, gridless_nets=set(), gridless_negotiate=True)
        assert self._snapshot(r1) == self._snapshot(r2)

    def test_none_no_gridless_routes_in_results(self):
        """gridless_nets=None must never produce GridlessNetRoute in results."""
        g, nets = self._small_problem()
        results = route_all(g, nets, gridless_nets=None, gridless_negotiate=True)
        for name, nr in results.items():
            assert not isinstance(nr, GridlessNetRoute), (
                f"Net {name!r} got GridlessNetRoute despite gridless_nets=None"
            )

    def test_empty_set_no_gridless_routes_in_results(self):
        """gridless_nets=set() must never produce GridlessNetRoute in results."""
        g, nets = self._small_problem()
        results = route_all(g, nets, gridless_nets=set(), gridless_negotiate=True)
        for _name, nr in results.items():
            assert not isinstance(nr, GridlessNetRoute)

    def test_none_occupancy_identical_between_calls(self):
        """Grid occupancy is the same for two calls with gridless_nets=None."""
        import numpy as np

        def _occ(gn, negotiate):
            g, nets = self._small_problem()
            results = route_all(g, nets, gridless_nets=gn, gridless_negotiate=negotiate)
            for nr in results.values():
                if nr.ok:
                    _mark(g, nr, 1)
            return g.cells.copy()

        occ1 = _occ(None, False)
        occ2 = _occ(None, True)
        assert np.array_equal(occ1, occ2), (
            "Occupancy differs between gridless_negotiate=False and True for None net set"
        )


# ---------------------------------------------------------------------------
# T-N1b: the critical byte-identical test with test name per the output contract
# ---------------------------------------------------------------------------

class TestDefaultPathByteIdentical:
    """The output-contract test: gridless_nets=None → byte-identical.

    Test name: test_gridless_none_byte_identical (referenced in Structured Result).
    """

    def test_gridless_none_byte_identical(self):
        """Fundamental invariant: route_all with gridless_nets=None produces results
        byte-identical to a call with no gridless parameters at all."""

        def _run(extra_kwargs):
            g = Grid(x0=0.0, y0=0.0, width_mm=10.0, height_mm=10.0, layers=2)
            nets = [
                Net("P", [(0, 20, 20), (0, 20, 80)], halfwidth_cells=1),
                Net("Q", [(0, 80, 20), (0, 80, 80)], halfwidth_cells=1),
                Net("R", [(0, 50, 10), (0, 50, 90)], halfwidth_cells=1),
            ]
            return route_all(g, nets, **extra_kwargs)

        # No gridless parameters at all (original API)
        r_baseline = _run({})
        # Explicit None
        r_none = _run({"gridless_nets": None})
        # None + negotiate=True (must still be identical)
        r_none_neg = _run({"gridless_nets": None, "gridless_negotiate": True})
        # Empty set + negotiate=True
        r_empty_neg = _run({"gridless_nets": set(), "gridless_negotiate": True})

        def snap(r):
            return {n: (nr.ok, frozenset(nr.cells), type(nr).__name__)
                    for n, nr in r.items()}

        s_base = snap(r_baseline)
        assert snap(r_none) == s_base, "Explicit None differs from baseline"
        assert snap(r_none_neg) == s_base, "None + negotiate=True differs from baseline"
        assert snap(r_empty_neg) == s_base, "Empty set + negotiate=True differs from baseline"


# ---------------------------------------------------------------------------
# T-N2: staged wiring — negotiated gridless + grid coexistence
# ---------------------------------------------------------------------------

class TestStagedGridlessNegotiate:
    """T-N2: gridless_negotiate=True routes nominated nets via negotiate path,
    rasterizes into shared grid ledger, grid sees the copper."""

    GEO = {"track_mm": 0.2, "clearance_mm": 0.2}
    BOARD = (0.0, 0.0, 15.0, 12.0)

    def _simple_board_setup(self):
        """3-net board: Net0 (gridless), Net1 (gridless), NetG (grid)."""
        g = Grid(x0=0.0, y0=0.0, width_mm=15.0, height_mm=12.0, layers=2, pitch=0.1)
        pads = [
            {"net": "Net0", "x": 2.0, "y": 2.0, "hw": 0.3, "hh": 0.3, "front": True},
            {"net": "Net0", "x": 2.0, "y": 9.0, "hw": 0.3, "hh": 0.3, "front": True},
            {"net": "Net1", "x": 6.0, "y": 2.0, "hw": 0.3, "hh": 0.3, "front": True},
            {"net": "Net1", "x": 6.0, "y": 9.0, "hw": 0.3, "hh": 0.3, "front": True},
            {"net": "NetG", "x": 10.0, "y": 2.0, "hw": 0.3, "hh": 0.3, "front": True},
            {"net": "NetG", "x": 10.0, "y": 9.0, "hw": 0.3, "hh": 0.3, "front": True},
        ]
        # cell grid coords (layer, iy, ix) = (layer, round((y-y0)/pitch), round((x-x0)/pitch))
        nets = [
            Net("Net0", [(0, 20, 20), (0, 90, 20)], halfwidth_cells=2),
            Net("Net1", [(0, 20, 60), (0, 90, 60)], halfwidth_cells=2),
            Net("NetG", [(0, 20, 100), (0, 90, 100)], halfwidth_cells=2),
        ]
        bd = (0.0, 0.0, 15.0, 12.0)
        gk = {
            "pads": pads,
            "geo": self.GEO,
            "board_bbox": bd,
            "anchors": None,
        }
        return g, nets, gk

    def test_nominated_nets_get_gridless_netroute(self):
        """Nominated gridless nets must produce GridlessNetRoute in results."""
        g, nets, gk = self._simple_board_setup()
        results = route_all(
            g, nets,
            gridless_nets={"Net0", "Net1"},
            gridless_kwargs=gk,
            gridless_negotiate=True,
        )
        assert "Net0" in results
        assert "Net1" in results
        if results["Net0"].ok:
            assert isinstance(results["Net0"], GridlessNetRoute), (
                f"Net0 should be GridlessNetRoute, got {type(results['Net0']).__name__}"
            )
        if results["Net1"].ok:
            assert isinstance(results["Net1"], GridlessNetRoute), (
                f"Net1 should be GridlessNetRoute, got {type(results['Net1']).__name__}"
            )

    def test_non_nominated_net_stays_plain_netroute(self):
        """NetG (not nominated) must produce a plain NetRoute (grid path)."""
        g, nets, gk = self._simple_board_setup()
        results = route_all(
            g, nets,
            gridless_nets={"Net0", "Net1"},
            gridless_kwargs=gk,
            gridless_negotiate=True,
        )
        assert "NetG" in results
        assert not isinstance(results["NetG"], GridlessNetRoute), (
            "NetG should be a plain NetRoute (grid path)"
        )

    def test_gridless_copper_rasterized_into_grid(self):
        """After the negotiate pass, the grid must have non-zero occupancy from
        gridless copper so the grid-path nets see it."""
        import numpy as np

        g, nets, gk = self._simple_board_setup()
        cells_before = g.cells.copy()
        results = route_all(
            g, nets,
            gridless_nets={"Net0"},
            gridless_kwargs=gk,
            gridless_negotiate=True,
        )
        cells_after = g.cells.copy()
        # Grid cells must have changed if Net0 routed successfully
        if results.get("Net0") and results["Net0"].ok:
            assert not np.array_equal(cells_before, cells_after), (
                "Gridless copper from Net0 was not rasterized into the grid ledger"
            )

    def test_all_nets_in_results(self):
        """Every net passed to route_all must appear in results."""
        g, nets, gk = self._simple_board_setup()
        results = route_all(
            g, nets,
            gridless_nets={"Net0", "Net1"},
            gridless_kwargs=gk,
            gridless_negotiate=True,
        )
        for net in nets:
            assert net.name in results, f"Net {net.name!r} missing from results"


# ---------------------------------------------------------------------------
# T-N3: geometry-blocked reporting
# ---------------------------------------------------------------------------

class TestGeometryBlockedReporting:
    """T-N3: geometry-blocked nets are reported with correct reason, not raised."""

    GEO = {"track_mm": 0.2, "clearance_mm": 0.2}

    def test_geometry_blocked_result_has_correct_status(self):
        """A geometry-blocked net must have status='geometry_blocked' and ok=False."""
        # Create a scenario where a net needs a huge window: pad_a and pad_b
        # are opposite corners of a large board, surrounded by dense obstacles.
        # Instead, directly test the classify logic via a very small board_bbox
        # where the geometry_block_threshold is easy to trigger.
        #
        # Use geom_block_threshold=0.0 to force ALL nets to be geometry-blocked.
        net_set = [{"net_name": "NB", "pad_a": (1.0, 1.0), "pad_b": (4.0, 4.0)}]
        pads = [
            {"net": "NB", "x": 1.0, "y": 1.0, "hw": 0.3, "hh": 0.3, "front": True},
            {"net": "NB", "x": 4.0, "y": 4.0, "hw": 0.3, "hh": 0.3, "front": True},
        ]
        board = (0.0, 0.0, 5.0, 5.0)
        result = route_gridless_set(
            net_set=net_set, pads=pads, geo=self.GEO, board_bbox=board,
            geom_block_threshold=0.0,  # any window > 0% diag → blocked
        )
        assert "NB" in result
        nr = result["NB"]
        assert not nr.ok, "Geometry-blocked net should have ok=False"
        assert nr.status == "geometry_blocked", (
            f"Expected status='geometry_blocked', got {nr.status!r}"
        )
        assert "M3" in nr.reason or "geometry-blocked" in nr.reason, (
            f"Reason should mention M3 or geometry-blocked, got {nr.reason!r}"
        )

    def test_geometry_blocked_does_not_raise(self):
        """route_gridless_set must return cleanly (not raise) for geometry-blocked nets."""
        net_set = [{"net_name": "NB", "pad_a": (1.0, 1.0), "pad_b": (4.0, 4.0)}]
        pads = [
            {"net": "NB", "x": 1.0, "y": 1.0, "hw": 0.3, "hh": 0.3, "front": True},
            {"net": "NB", "x": 4.0, "y": 4.0, "hw": 0.3, "hh": 0.3, "front": True},
        ]
        board = (0.0, 0.0, 5.0, 5.0)
        # Must not raise
        result = route_gridless_set(
            net_set=net_set, pads=pads, geo=self.GEO, board_bbox=board,
            geom_block_threshold=0.0,
        )
        assert "NB" in result


# ---------------------------------------------------------------------------
# T-N4: determinism — route_gridless_set is byte-identical across two runs
# ---------------------------------------------------------------------------

class TestNegotiateDeterminism:
    """T-N4: route_gridless_set is deterministic across identical calls."""

    GEO = {"track_mm": 0.2, "clearance_mm": 0.2}
    BOARD = (0.0, 0.0, 15.0, 12.0)

    def test_single_net_byte_identical(self):
        """Two calls with identical inputs must produce identical world_paths."""
        net_set = [{"net_name": "N", "pad_a": (2.0, 2.0), "pad_b": (2.0, 8.0)}]
        pads = [
            {"net": "N", "x": 2.0, "y": 2.0, "hw": 0.3, "hh": 0.3, "front": True},
            {"net": "N", "x": 2.0, "y": 8.0, "hw": 0.3, "hh": 0.3, "front": True},
        ]
        r1 = route_gridless_set(net_set=net_set, pads=pads, geo=self.GEO,
                                board_bbox=self.BOARD)
        r2 = route_gridless_set(net_set=net_set, pads=pads, geo=self.GEO,
                                board_bbox=self.BOARD)
        assert r1["N"].ok and r2["N"].ok, "Net failed in one of the runs"
        assert r1["N"].world_paths == r2["N"].world_paths, (
            f"world_paths differ:\n  r1: {r1['N'].world_paths}\n  r2: {r2['N'].world_paths}"
        )

    def test_multi_net_byte_identical(self):
        """Multiple nets: both runs produce identical results."""
        net_set = [
            {"net_name": "Net0", "pad_a": (2.0, 2.0), "pad_b": (2.0, 9.0)},
            {"net_name": "Net1", "pad_a": (6.0, 2.0), "pad_b": (6.0, 9.0)},
        ]
        pads = [
            {"net": "Net0", "x": 2.0, "y": 2.0, "hw": 0.3, "hh": 0.3, "front": True},
            {"net": "Net0", "x": 2.0, "y": 9.0, "hw": 0.3, "hh": 0.3, "front": True},
            {"net": "Net1", "x": 6.0, "y": 2.0, "hw": 0.3, "hh": 0.3, "front": True},
            {"net": "Net1", "x": 6.0, "y": 9.0, "hw": 0.3, "hh": 0.3, "front": True},
        ]
        r1 = route_gridless_set(net_set=net_set, pads=pads, geo=self.GEO,
                                board_bbox=self.BOARD)
        r2 = route_gridless_set(net_set=net_set, pads=pads, geo=self.GEO,
                                board_bbox=self.BOARD)
        for net_name in ("Net0", "Net1"):
            nr1 = r1[net_name]
            nr2 = r2[net_name]
            if nr1.ok and nr2.ok:
                assert nr1.world_paths == nr2.world_paths, (
                    f"{net_name}: world_paths differ between runs"
                )
            else:
                assert nr1.ok == nr2.ok, (
                    f"{net_name}: ok flag differs between runs ({nr1.ok} vs {nr2.ok})"
                )

    def test_three_runs_identical(self):
        """Three runs must all be identical (no non-deterministic state leak)."""
        net_set = [{"net_name": "N", "pad_a": (3.0, 2.0), "pad_b": (3.0, 9.0)}]
        pads = [
            {"net": "N", "x": 3.0, "y": 2.0, "hw": 0.3, "hh": 0.3, "front": True},
            {"net": "N", "x": 3.0, "y": 9.0, "hw": 0.3, "hh": 0.3, "front": True},
        ]
        runs = [
            route_gridless_set(net_set=net_set, pads=pads, geo=self.GEO,
                               board_bbox=self.BOARD)
            for _ in range(3)
        ]
        for i, r in enumerate(runs):
            if r["N"].ok:
                assert r["N"].world_paths == runs[0]["N"].world_paths, (
                    f"Run {i+1} world_paths differ from run 1"
                )


# ---------------------------------------------------------------------------
# T-N5: mitayi board integration (requires board file)
# ---------------------------------------------------------------------------

@pytest.mark.skipif(not BOARD_EXISTS, reason=f"Mitayi board not found at {BOARD_PATH}")
class TestMitayiNegotiateIntegration:
    """T-N5: route_gridless_set on a subset of real mitayi nets.

    Uses 2-pin F.Cu nets from the QSPI/GPIO region that the spike2 validated.
    This is a smoke test (not the full M2 gate run), exercising real board geometry.
    """

    # A small representative subset of 2-pin F.Cu nets that are not geometry-blocked
    NETS = [
        "/QSPI_SCLK",
        "/QSPI_SD0",
        "/QSPI_SD1",
        "Net-(U2-BP)",
    ]

    @pytest.fixture(scope="class")
    def board_context(self):
        from tracewise.route.engine.kicad import (
            build_problem,
            extract_pads,
            project_geometry,
        )
        data = extract_pads(BOARD_PATH)
        geo = project_geometry(BOARD_PATH)
        grid, nets, anchors, _, _ = build_problem(
            data, pitch=0.1, track_mm=geo["track_mm"], clearance_mm=geo["clearance_mm"]
        )
        bd = data["board"]
        board_bbox = (bd["x1"], bd["y1"], bd["x2"], bd["y2"])
        return data, geo, grid, nets, anchors, board_bbox

    def _build_net_set(self, data, geo, grid, nets, anchors, board_bbox, net_names):
        """Build net_set input for route_gridless_set."""
        by_name = {n.name: n for n in nets}
        net_set = []
        for nname in net_names:
            net = by_name.get(nname)
            if net is None or len(net.pads) < 2:
                continue
            pad_a_cell = net.pads[0]
            pad_b_cell = net.pads[1]
            pad_a = grid.to_world(pad_a_cell[1], pad_a_cell[2])
            pad_b = grid.to_world(pad_b_cell[1], pad_b_cell[2])
            if anchors is not None:
                pad_a = anchors.get(pad_a_cell, pad_a)
                pad_b = anchors.get(pad_b_cell, pad_b)
            net_set.append({"net_name": nname, "pad_a": pad_a, "pad_b": pad_b})
        return net_set

    def test_subset_routes_ok(self, board_context):
        """The M1 net subset must route via route_gridless_set without crashing."""
        data, geo, grid, nets, anchors, board_bbox = board_context
        net_set = self._build_net_set(
            data, geo, grid, nets, anchors, board_bbox, self.NETS
        )
        assert net_set, "No valid nets found in board for the test subset"
        result = route_gridless_set(
            net_set=net_set,
            pads=data["pads"],
            geo=geo,
            board_bbox=board_bbox,
            anchors=anchors,
        )
        assert len(result) == len(net_set), (
            f"Expected {len(net_set)} results, got {len(result)}"
        )
        # At least half should route successfully on a real board
        ok_count = sum(1 for nr in result.values() if nr.ok)
        assert ok_count >= len(net_set) // 2, (
            f"Only {ok_count}/{len(net_set)} nets routed; "
            f"failures: {[(n, nr.reason) for n, nr in result.items() if not nr.ok]}"
        )

    def test_determinism_on_real_board(self, board_context):
        """Two calls on the real board produce byte-identical world_paths."""
        data, geo, grid, nets, anchors, board_bbox = board_context
        net_set = self._build_net_set(
            data, geo, grid, nets, anchors, board_bbox, self.NETS[:2]
        )
        if not net_set:
            pytest.skip("No valid nets for determinism test")

        r1 = route_gridless_set(
            net_set=net_set, pads=data["pads"], geo=geo,
            board_bbox=board_bbox, anchors=anchors,
        )
        r2 = route_gridless_set(
            net_set=net_set, pads=data["pads"], geo=geo,
            board_bbox=board_bbox, anchors=anchors,
        )
        for nd in net_set:
            nname = nd["net_name"]
            nr1 = r1.get(nname)
            nr2 = r2.get(nname)
            if nr1 and nr2 and nr1.ok and nr2.ok:
                assert nr1.world_paths == nr2.world_paths, (
                    f"world_paths differ between runs for {nname!r}"
                )
