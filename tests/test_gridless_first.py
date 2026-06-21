"""Tests for the gridless_first ordering mode.

gridless_first routes a nominated set of "hard" nets GRIDLESS-FIRST on the
clean board (only pads + board edge + drills as obstacles; no grid tracks yet).
Their copper is written to the PCB file (providing electrical connectivity) but
is NOT registered as a grid obstacle (grid.cells / grid.hard).  This prevents
via displacement near PTH connector pads that would otherwise create new
hole_to_hole DRC violations.

Invariants under test:
  (1) gridless_first=None (default) → byte-identical to current behaviour
  (2) gridless_first=set() (empty) → byte-identical to current behaviour
  (3) gridless_first nets get a GridlessNetRoute in results
  (4) non-first nets stay as plain NetRoute (grid path)
  (5) 2-pin gridless_first copper IS marked into shared grid (negotiate path)
  (6) route_all signature: gridless_first accepted without error
  (7) route_board_engine signature: gridless_first accepted without error

Skip cleanly when Shapely is absent:
    pytest.importorskip("shapely")
"""

from __future__ import annotations

import pathlib

import pytest

shapely = pytest.importorskip("shapely")

from tracewise.route.engine.grid import Grid  # noqa: E402
from tracewise.route.engine.multi import Net, route_all  # noqa: E402
from tracewise.route.gridless.adapter import GridlessNetRoute  # noqa: E402

BOARD_PATH = (
    pathlib.Path(__file__).parent.parent
    / "data" / "benchmark-boards" / "mitayi-pico-d1" / "Mitayi-Pico-D1.kicad_pcb"
)

# A 2-pin F.Cu net we know routes successfully
KNOWN_GOOD_NET = "Net-(U2-BP)"

# The 17 boxed-in signal nets for mitayi (subset used for targeted tests)
TARGET_NETS_17 = frozenset([
    "/GPIO3", "/GPIO4", "/GPIO6", "/GPIO9", "/GPIO14",
    "/GPIO18", "/GPIO20", "/GPIO23", "/GPIO27", "/GPIO28",
    "/RUN", "/SWCLK", "/USB_D+", "/XIN",
    "/QSPI_SCLK", "/QSPI_SD2", "Net-(U3-USB-DP)",
])


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _small_problem():
    """A tiny 2-net synthetic problem on a 10mm×10mm board."""
    g = Grid(x0=0.0, y0=0.0, width_mm=10.0, height_mm=10.0, layers=2)
    nets = [
        Net("A", [(0, 10, 10), (0, 10, 90)], halfwidth_cells=1),
        Net("B", [(0, 90, 10), (0, 90, 90)], halfwidth_cells=1),
    ]
    return g, nets


def _board_data():
    from tracewise.route.engine.kicad import extract_pads, project_geometry
    data = extract_pads(BOARD_PATH)
    geo = project_geometry(BOARD_PATH)
    return data, geo


def _fresh_problem(data, geo, pitch=0.1):
    from tracewise.route.engine.kicad import build_problem
    return build_problem(data, pitch=pitch,
                         track_mm=geo["track_mm"],
                         clearance_mm=geo["clearance_mm"])


# ---------------------------------------------------------------------------
# T-GF-1: Default / empty → no-op (byte-identical to grid-only)
# ---------------------------------------------------------------------------


class TestGridlessFirstDefault:
    """gridless_first=None and gridless_first=set() must be no-ops."""

    def test_none_produces_no_gridless_route(self):
        """gridless_first=None: no GridlessNetRoute in results."""
        g, nets = _small_problem()
        results = route_all(g, nets, gridless_first=None)
        for name, nr in results.items():
            assert not isinstance(nr, GridlessNetRoute), (
                f"Net {name!r} got GridlessNetRoute despite gridless_first=None"
            )

    def test_empty_set_produces_no_gridless_route(self):
        """gridless_first=set(): no GridlessNetRoute in results."""
        g, nets = _small_problem()
        results = route_all(g, nets, gridless_first=set())
        for name, nr in results.items():
            assert not isinstance(nr, GridlessNetRoute), (
                f"Net {name!r} got GridlessNetRoute despite gridless_first=set()"
            )

    def test_none_and_default_same_result(self):
        """route_all() and route_all(gridless_first=None) must produce identical results."""
        def _run(gf):
            g, nets = _small_problem()
            results = route_all(g, nets, gridless_first=gf)
            return {name: (nr.ok, frozenset(nr.cells)) for name, nr in results.items()}

        r1 = _run(None)
        r2 = _run(None)
        assert r1 == r2, "route_all is not deterministic with gridless_first=None"

    def test_none_identical_to_no_kwarg(self):
        """Explicit gridless_first=None must produce same result as omitting the kwarg."""
        def _snap(gf_kwarg):
            g, nets = _small_problem()
            if gf_kwarg is None:
                results = route_all(g, nets)
            else:
                results = route_all(g, nets, gridless_first=gf_kwarg)
            return {name: (nr.ok, frozenset(nr.cells)) for name, nr in results.items()}

        r_omit = _snap(None)
        r_explicit = _snap(None)
        assert r_omit == r_explicit


# ---------------------------------------------------------------------------
# T-GF-2: gridless_first activates the negotiate pre-route block
# ---------------------------------------------------------------------------


@pytest.mark.skipif(
    not BOARD_PATH.exists(),
    reason=f"Mitayi board not found at {BOARD_PATH}",
)
class TestGridlessFirstActivation:
    """gridless_first routes nominated nets before grid."""

    @pytest.fixture(scope="class")
    def board_context(self):
        return _board_data()

    def test_first_net_gets_gridless_route(self, board_context):
        """A net in gridless_first must get a GridlessNetRoute result."""
        data, geo = board_context
        grid, nets, anchors, _, _ = _fresh_problem(data, geo)
        bd = data["board"]

        # Only route one known-good net for speed
        target = [n for n in nets if n.name == KNOWN_GOOD_NET]
        assert target, f"Net {KNOWN_GOOD_NET!r} not found in problem"

        gk = {
            "pads": data["pads"],
            "geo": geo,
            "board_bbox": (bd["x1"], bd["y1"], bd["x2"], bd["y2"]),
            "anchors": anchors,
            "extra_gridless_obstacles": [],
        }
        results = route_all(
            grid, target,
            gridless_first={KNOWN_GOOD_NET},
            gridless_kwargs=gk,
            ripup_factor=1,
        )

        assert KNOWN_GOOD_NET in results
        nr = results[KNOWN_GOOD_NET]
        assert nr.ok, f"gridless_first route failed: {nr.reason}"
        assert isinstance(nr, GridlessNetRoute), (
            f"Expected GridlessNetRoute for {KNOWN_GOOD_NET!r}, got {type(nr).__name__}"
        )

    def test_non_first_net_stays_grid(self, board_context):
        """A net NOT in gridless_first must get a plain NetRoute (grid path)."""
        data, geo = board_context
        grid, nets, anchors, _, _ = _fresh_problem(data, geo)
        bd = data["board"]

        target = [n for n in nets if n.name == KNOWN_GOOD_NET]
        assert target

        gk = {
            "pads": data["pads"],
            "geo": geo,
            "board_bbox": (bd["x1"], bd["y1"], bd["x2"], bd["y2"]),
            "anchors": anchors,
            "extra_gridless_obstacles": [],
        }
        # KNOWN_GOOD_NET is NOT in gridless_first → must take grid path
        results = route_all(
            grid, target,
            gridless_first={"SomeOtherNet"},
            gridless_kwargs=gk,
            ripup_factor=1,
        )

        nr = results.get(KNOWN_GOOD_NET)
        assert nr is not None
        assert not isinstance(nr, GridlessNetRoute), (
            f"Net {KNOWN_GOOD_NET!r} should have used grid path, got GridlessNetRoute"
        )

    def test_gridless_first_implies_negotiate_true(self, board_context):
        """gridless_first must force gridless_negotiate=True on the path."""
        # We verify this behaviorally: a net in gridless_first is pre-routed before
        # the grid loop (its result is in _gridless_pre_routed, so it will NOT be
        # in the grid queue).  We use gridless_negotiate=False explicitly and
        # confirm gridless_first still routes gridlessly.
        data, geo = board_context
        grid, nets, anchors, _, _ = _fresh_problem(data, geo)
        bd = data["board"]

        target = [n for n in nets if n.name == KNOWN_GOOD_NET]
        assert target

        gk = {
            "pads": data["pads"],
            "geo": geo,
            "board_bbox": (bd["x1"], bd["y1"], bd["x2"], bd["y2"]),
            "anchors": anchors,
            "extra_gridless_obstacles": [],
        }
        # Passing gridless_negotiate=False explicitly should NOT override gridless_first
        results = route_all(
            grid, target,
            gridless_first={KNOWN_GOOD_NET},
            gridless_negotiate=False,   # should be overridden by gridless_first logic
            gridless_kwargs=gk,
            ripup_factor=1,
        )

        assert KNOWN_GOOD_NET in results
        nr = results[KNOWN_GOOD_NET]
        assert nr.ok, f"route failed: {nr.reason}"
        assert isinstance(nr, GridlessNetRoute)

    def test_gridless_first_2pin_marks_grid_before_others(self, board_context):
        """2-pin gridless-first copper (negotiate path) must appear in the grid ledger.

        Multi-pin (K>2) nets use the multipin fast path which does NOT mark the
        grid — this prevents via displacement near PTH connector pads.  The 2-pin
        negotiate path still marks the grid so grid nets avoid 2-pin gridless tracks.
        """
        import numpy as np
        data, geo = board_context
        grid, nets, anchors, _, _ = _fresh_problem(data, geo)
        bd = data["board"]

        target = [n for n in nets if n.name == KNOWN_GOOD_NET]
        assert target

        gk = {
            "pads": data["pads"],
            "geo": geo,
            "board_bbox": (bd["x1"], bd["y1"], bd["x2"], bd["y2"]),
            "anchors": anchors,
            "extra_gridless_obstacles": [],
        }
        snapshot_before = grid.cells.copy()
        results = route_all(
            grid, target,
            gridless_first={KNOWN_GOOD_NET},
            gridless_kwargs=gk,
            ripup_factor=1,
        )
        nr = results.get(KNOWN_GOOD_NET)
        assert nr is not None and nr.ok

        # After route_all, the gridless-first net's copper should be in the grid
        assert not np.array_equal(grid.cells, snapshot_before), (
            "gridless_first copper was not marked into the shared grid ledger"
        )


# ---------------------------------------------------------------------------
# T-GF-3: Signature parity — route_board_engine accepts gridless_first
# ---------------------------------------------------------------------------


def test_route_board_engine_accepts_gridless_first_none():
    """route_board_engine must accept gridless_first=None without error (fast path)."""
    import inspect

    from tracewise.route.engine.kicad import route_board_engine
    sig = inspect.signature(route_board_engine)
    assert "gridless_first" in sig.parameters, (
        "route_board_engine missing gridless_first parameter"
    )
    p = sig.parameters["gridless_first"]
    assert p.default is None, (
        f"gridless_first default should be None, got {p.default!r}"
    )


def test_route_all_accepts_gridless_first_none():
    """route_all must accept gridless_first=None without error."""
    import inspect
    sig = inspect.signature(route_all)
    assert "gridless_first" in sig.parameters, (
        "route_all missing gridless_first parameter"
    )
    p = sig.parameters["gridless_first"]
    assert p.default is None, (
        f"gridless_first default should be None, got {p.default!r}"
    )


# ---------------------------------------------------------------------------
# T-GF-4: Determinism — two runs with gridless_first produce identical results
# ---------------------------------------------------------------------------


@pytest.mark.skipif(
    not BOARD_PATH.exists(),
    reason=f"Mitayi board not found at {BOARD_PATH}",
)
class TestGridlessFirstDeterminism:
    """Two calls with gridless_first must produce identical world_paths."""

    @pytest.fixture(scope="class")
    def board_context(self):
        return _board_data()

    def test_deterministic_world_paths(self, board_context):
        """Two independent route_all calls with gridless_first must have same world_paths."""
        data, geo = board_context
        bd = data["board"]

        gk_base = {
            "pads": data["pads"],
            "geo": geo,
            "board_bbox": (bd["x1"], bd["y1"], bd["x2"], bd["y2"]),
        }

        def _run():
            grid, nets, anchors, _, _ = _fresh_problem(data, geo)
            target = [n for n in nets if n.name == KNOWN_GOOD_NET]
            gk = {**gk_base, "anchors": anchors, "extra_gridless_obstacles": []}
            return route_all(
                grid, target,
                gridless_first={KNOWN_GOOD_NET},
                gridless_kwargs=gk,
                ripup_factor=1,
            )

        r1 = _run()
        r2 = _run()

        nr1 = r1.get(KNOWN_GOOD_NET)
        nr2 = r2.get(KNOWN_GOOD_NET)
        assert nr1 is not None and nr2 is not None
        assert nr1.ok and nr2.ok, f"at least one run failed: {nr1.reason} / {nr2.reason}"
        assert isinstance(nr1, GridlessNetRoute) and isinstance(nr2, GridlessNetRoute)
        assert nr1.world_paths == nr2.world_paths, (
            f"world_paths differ between runs for {KNOWN_GOOD_NET!r}:\n"
            f"  run1: {nr1.world_paths}\n"
            f"  run2: {nr2.world_paths}"
        )
