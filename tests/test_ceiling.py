"""Tests for the L1 ceiling detector (tracewise.route.engine.ceiling).

Pure-grid unit tests
--------------------
These tests do NOT require pcbnew or kicad-cli.  They build synthetic Grid
objects directly and verify the connected-component logic and gap-classification
logic in isolation.

Topology tested
---------------
Connected (recoverable) case:
  A 10×10 mm, 2-layer grid, entirely free.  Two endpoint pairs that are
  clearly in the same free-space component.

Walled (unroutable) case:
  Same board, but a solid vertical wall of obstacles divides the grid into a
  left half and a right half.  Endpoints on opposite sides must be classified
  UNROUTABLE_2LAYER.

Via connectivity case:
  Two cells separated by a blocked band on layer 0, but connected via a free
  via position on layer 1 (where the band is free).  Must be classified as
  ROUTER_RECOVERABLE.

Label-array shape / determinism:
  label_components returns the expected shape and is deterministic (same result
  on two calls).
"""

from __future__ import annotations

import numpy as np
import pytest

from tracewise.route.engine.ceiling import (
    _WINDOW,
    _nearest_free,
    classify_unrouted,
    label_components,
)
from tracewise.route.engine.grid import Grid

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def make_grid(w: float = 10.0, h: float = 10.0, layers: int = 2) -> Grid:
    return Grid(x0=0.0, y0=0.0, width_mm=w, height_mm=h, layers=2)


def _wall(grid: Grid, layer: int, col: int) -> None:
    """Block an entire vertical column on the given layer."""
    grid.cells[layer, :, col] += 1


# ---------------------------------------------------------------------------
# label_components
# ---------------------------------------------------------------------------


class TestLabelComponents:
    def test_shape_matches_grid(self):
        g = make_grid(5.0, 5.0)
        lbl = label_components(g)
        assert lbl.shape == (g.layers, g.ny, g.nx)
        assert lbl.dtype == np.int32

    def test_fully_free_grid_single_component(self):
        """An obstacle-free grid is one big connected component."""
        g = make_grid(5.0, 5.0)
        lbl = label_components(g)
        free_mask = g.cells == 0
        # All free cells should share ONE label value.
        free_labels = lbl[free_mask]
        assert free_labels.min() >= 1
        assert len(set(free_labels.tolist())) == 1, (
            f"Expected 1 component, got {len(set(free_labels.tolist()))}"
        )

    def test_wall_splits_into_two_components(self):
        """A full vertical wall creates exactly 2 free-space components."""
        g = make_grid(5.0, 5.0)
        mid = g.nx // 2
        # Block the wall on BOTH layers (otherwise the via edge would bridge them).
        for li in range(g.layers):
            _wall(g, li, mid)
        lbl = label_components(g)
        free_mask = g.cells == 0
        free_labels = set(lbl[free_mask].tolist())
        assert len(free_labels) == 2, (
            f"Expected 2 components after wall, got {len(free_labels)}"
        )

    def test_obstacle_cells_have_label_zero(self):
        """Obstacle cells must carry label 0 (the sentinel)."""
        g = make_grid(5.0, 5.0)
        _wall(g, 0, 10)
        lbl = label_components(g)
        obstacle_mask = g.cells != 0
        assert (lbl[obstacle_mask] == 0).all()

    def test_deterministic(self):
        """Two calls on the same grid must return bit-identical arrays."""
        g = make_grid(4.0, 4.0)
        g.cells[0, 10, 10] += 1
        lbl1 = label_components(g)
        lbl2 = label_components(g)
        np.testing.assert_array_equal(lbl1, lbl2)

    def test_via_connects_across_blocked_layer(self):
        """A via at a clear position bridges two cells separated by a wall
        on layer 0.  Without the via edge, they would be in different components;
        with it, they must be in the SAME component.

        Setup: small grid, wall blocks column mid on layer 0 only.
        Layer 1 is entirely free.  Via-legality requires VIA_RING=2 clear cells,
        so we make the grid wide enough and block the wall away from the edges.
        """
        # Grid must be wide enough: wall at col mid, with VIA_RING clear margin
        # on each side.  Use a 5×3 mm grid (50 cols × 30 rows at 0.1 mm pitch).
        g = Grid(x0=0.0, y0=0.0, width_mm=5.0, height_mm=3.0)
        mid = g.nx // 2
        # Block ONLY layer 0 at the wall column.
        g.cells[0, :, mid] += 1
        # Layer 1 is entirely free → via at any (iy, ix) away from edges is legal.
        lbl = label_components(g)
        # Pick a cell to the left and right of the wall on layer 0.
        # They must share a label because layer 1 connects them.
        left_l0 = int(lbl[0, g.ny // 2, mid - 3])
        right_l0 = int(lbl[0, g.ny // 2, mid + 3])
        assert left_l0 >= 1 and right_l0 >= 1, "Cells must be in a valid component"
        assert left_l0 == right_l0, (
            "Left and right of wall should be in the SAME component via layer-1 bridge"
        )

    def test_isolated_cell_gets_own_component(self):
        """A single free cell surrounded by obstacles gets its own component label."""
        g = make_grid(3.0, 3.0)
        # Block everything.
        g.cells[:] = 1
        # Free exactly one cell on each layer (at different positions).
        g.cells[0, 5, 5] = 0
        g.cells[1, 10, 10] = 0
        lbl = label_components(g)
        assert lbl[0, 5, 5] >= 1
        assert lbl[1, 10, 10] >= 1
        assert lbl[0, 5, 5] != lbl[1, 10, 10]


# ---------------------------------------------------------------------------
# _nearest_free
# ---------------------------------------------------------------------------


class TestNearestFree:
    def test_finds_adjacent_free_cell(self):
        """If the target cell is blocked but a neighbour is free, it is found."""
        g = make_grid(3.0, 3.0)
        # Block cell (10, 10) on layer 0.
        g.cells[0, 10, 10] += 1
        # Compute labels with the blocked cell.
        lbl2 = label_components(g)
        rep, label = _nearest_free(g, lbl2, 10, 10, [0])
        assert rep is not None
        assert label >= 1
        layer, iy, ix = rep
        # Must be within _WINDOW cells of (10, 10).
        assert max(abs(iy - 10), abs(ix - 10)) <= _WINDOW

    def test_returns_none_when_all_blocked(self):
        """Returns (None, -1) when the window contains no free cells."""
        g = make_grid(3.0, 3.0)
        # Block all cells.
        g.cells[:] = 1
        lbl = label_components(g)
        rep, lbl_val = _nearest_free(g, lbl, 15, 15, [0])
        assert rep is None
        assert lbl_val == -1

    def test_prefers_distance_zero_if_free(self):
        """If the target cell itself is free, it is returned at distance 0."""
        g = make_grid(3.0, 3.0)
        lbl = label_components(g)
        rep, label = _nearest_free(g, lbl, 10, 10, [0])
        assert rep == (0, 10, 10)
        assert label >= 1


# ---------------------------------------------------------------------------
# classify_unrouted — pure-grid mock (no kicad-cli)
# ---------------------------------------------------------------------------


class TestClassifyUnroutedMock:
    """Test classify_unrouted by monkeypatching run_drc to return synthetic
    unconnected-item data.  No kicad-cli or board file needed."""

    def _make_drc_item(self, net: str, x1: float, y1: float,
                       x2: float, y2: float) -> dict:
        # Mirror real kicad-cli DRC structure: the net is embedded in each
        # endpoint's localized description as "[NET]", NOT a top-level field.
        return {
            "items": [
                {"pos": {"x": x1, "y": y1}, "description": f"Pad 1 [{net}] from U1"},
                {"pos": {"x": x2, "y": y2}, "description": f"Pad 2 [{net}] from U2"},
            ],
        }

    def test_same_component_recoverable(self, monkeypatch):
        """Two endpoints in the same free-space component → ROUTER_RECOVERABLE."""
        g = make_grid(10.0, 10.0)
        # Grid is entirely free → both endpoints share one component.
        fake_report = {
            "unconnected_items": [
                self._make_drc_item("NET_A", 1.0, 1.0, 8.0, 8.0),
            ]
        }

        import tracewise.route.bridge as bridge_mod
        monkeypatch.setattr(bridge_mod, "run_drc",
                            lambda board: fake_report,
                            raising=False)

        result = classify_unrouted("fake_board.kicad_pcb", g)
        assert result.recoverable == 1
        assert result.unroutable_2layer == 0
        assert len(result.details) == 1
        assert result.details[0].classification == "ROUTER_RECOVERABLE"

    def test_different_components_unroutable(self, monkeypatch):
        """Two endpoints separated by a full wall → UNROUTABLE_2LAYER."""
        g = make_grid(5.0, 5.0)
        mid = g.nx // 2
        # Block both layers at the wall column to prevent via bridging.
        for li in range(g.layers):
            _wall(g, li, mid)

        # Endpoints: one on each side of the wall.
        x_left = g.x0 + (mid - 5) * g.pitch
        x_right = g.x0 + (mid + 5) * g.pitch
        y_mid = g.y0 + (g.ny // 2) * g.pitch

        fake_report = {
            "unconnected_items": [
                self._make_drc_item("NET_B", x_left, y_mid, x_right, y_mid),
            ]
        }

        import tracewise.route.bridge as bridge_mod
        monkeypatch.setattr(bridge_mod, "run_drc",
                            lambda board: fake_report,
                            raising=False)

        result = classify_unrouted("fake_board.kicad_pcb", g)
        assert result.unroutable_2layer == 1
        assert result.recoverable == 0
        assert result.details[0].classification == "UNROUTABLE_2LAYER"

    def test_empty_unconnected_returns_empty_result(self, monkeypatch):
        """When DRC reports no unconnected items, CeilingResult is all zeros."""
        g = make_grid(5.0, 5.0)
        import tracewise.route.bridge as bridge_mod
        monkeypatch.setattr(bridge_mod, "run_drc",
                            lambda board: {"unconnected_items": []},
                            raising=False)
        result = classify_unrouted("fake_board.kicad_pcb", g)
        assert result.recoverable == 0
        assert result.unroutable_2layer == 0
        assert result.details == []

    def test_by_net_aggregation(self, monkeypatch):
        """by_net accumulates counts per net correctly."""
        g = make_grid(10.0, 10.0)
        fake_report = {
            "unconnected_items": [
                self._make_drc_item("GND", 1.0, 1.0, 2.0, 2.0),
                self._make_drc_item("GND", 3.0, 3.0, 4.0, 4.0),
                self._make_drc_item("VCC", 5.0, 5.0, 6.0, 6.0),
            ]
        }
        import tracewise.route.bridge as bridge_mod
        monkeypatch.setattr(bridge_mod, "run_drc",
                            lambda board: fake_report,
                            raising=False)
        result = classify_unrouted("fake_board.kicad_pcb", g)
        assert result.by_net["GND"]["recoverable"] == 2
        assert result.by_net["VCC"]["recoverable"] == 1

    def test_as_dict_serialisable(self, monkeypatch):
        """as_dict() returns a plain dict with the expected keys."""
        import json

        g = make_grid(5.0, 5.0)
        import tracewise.route.bridge as bridge_mod
        monkeypatch.setattr(bridge_mod, "run_drc",
                            lambda board: {"unconnected_items": []},
                            raising=False)
        result = classify_unrouted("fake_board.kicad_pcb", g)
        d = result.as_dict()
        assert set(d.keys()) == {"recoverable", "unroutable_2layer", "unknown", "by_net", "details"}
        json.dumps(d)  # must not raise

    def test_drc_error_returns_gracefully(self, monkeypatch):
        """If run_drc raises, classify_unrouted returns a result with a DRC_ERROR detail."""
        g = make_grid(5.0, 5.0)
        import tracewise.route.bridge as bridge_mod

        def _raise(board):
            raise RuntimeError("kicad-cli not found")

        monkeypatch.setattr(bridge_mod, "run_drc", _raise, raising=False)
        result = classify_unrouted("fake_board.kicad_pcb", g)
        # Should not raise; should return gracefully with an error detail.
        assert result.recoverable == 0
        assert result.unroutable_2layer == 0
        assert any("DRC_ERROR" in d.classification for d in result.details)


# ---------------------------------------------------------------------------
# route_board_engine signature (import only — no kicad needed)
# ---------------------------------------------------------------------------


def test_route_board_engine_signature():
    """route_board_engine must accept report_ceiling kwarg without error at
    import / inspection time (no board needed for this check)."""
    import inspect

    from tracewise.route.engine.kicad import route_board_engine

    sig = inspect.signature(route_board_engine)
    assert "report_ceiling" in sig.parameters
    param = sig.parameters["report_ceiling"]
    assert param.default is False


# ---------------------------------------------------------------------------
# kicad-dependent test (skipped if pcbnew/kicad-cli unavailable)
# ---------------------------------------------------------------------------


def _kicad_cli_available() -> bool:
    try:
        from tracewise.netlist import find_kicad_cli
        cli = find_kicad_cli()
        return cli is not None
    except Exception:
        return False


@pytest.mark.skipif(not _kicad_cli_available(), reason="kicad-cli not available")
def test_classify_unrouted_with_real_board():
    """Smoke-test on any available fixture board: result must have the
    expected structure and the function must not modify the board."""
    import hashlib
    from pathlib import Path

    from tracewise.route.engine.ceiling import classify_unrouted
    from tracewise.route.engine.grid import Grid

    fixture_dir = Path(__file__).parent / "fixtures"
    boards = list(fixture_dir.glob("*.kicad_pcb"))
    if not boards:
        pytest.skip("no .kicad_pcb fixtures available")
    board = boards[0]
    original_hash = hashlib.md5(board.read_bytes()).hexdigest()

    # Build a minimal grid covering the board — just for structure test.
    g = Grid(x0=0.0, y0=0.0, width_mm=40.0, height_mm=40.0)
    result = classify_unrouted(board, g)
    # Board must not have been modified.
    assert hashlib.md5(board.read_bytes()).hexdigest() == original_hash
    # Result structure.
    d = result.as_dict()
    assert "recoverable" in d
    assert "unroutable_2layer" in d
