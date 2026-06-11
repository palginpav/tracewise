"""Placement core tests — synthetic problems, deterministic. Self-skip
without torch (an optional [place] extra; CI installs dev only)."""

import pytest

torch = pytest.importorskip("torch")

from tracewise.place.core import (  # noqa: E402
    PlaceProblem,
    boundary_penalty,
    build_problem,
    optimize,
    overlap_penalty,
    smooth_hpwl,
    true_hpwl,
)


def two_part_problem(d=10.0, movable=(True, True)):
    """Two single-pad parts connected by one net, d mm apart."""
    return PlaceProblem(
        refs=["U1", "U2"],
        pos0=torch.tensor([[0.0, 0.0], [d, 0.0]], dtype=torch.float64),
        size=torch.tensor([[2.0, 2.0], [2.0, 2.0]], dtype=torch.float64),
        movable=torch.tensor(list(movable)),
        nets=[(torch.tensor([0, 1]), torch.zeros(2, 2, dtype=torch.float64))],
        board=(-50.0, -50.0, 50.0, 50.0),
        decap=[],
    )


def test_true_hpwl():
    p = two_part_problem(d=10.0)
    assert true_hpwl(p.pos0, p.nets) == pytest.approx(10.0)


def test_smooth_hpwl_approaches_true_at_low_tau():
    p = two_part_problem(d=10.0)
    smooth = float(smooth_hpwl(p.pos0, p.nets, tau=0.05))
    assert smooth == pytest.approx(10.0, abs=0.5)


def test_overlap_zero_when_apart_positive_when_overlapping():
    p = two_part_problem(d=10.0)
    assert float(overlap_penalty(p.pos0, p.size)) == 0.0
    pos = torch.tensor([[0.0, 0.0], [1.0, 0.0]], dtype=torch.float64)  # 2x2 boxes, 1mm apart
    assert float(overlap_penalty(pos, p.size)) == pytest.approx(1.0 * 2.0)  # 1x2 mm intersection


def test_boundary_penalty():
    p = two_part_problem()
    inside = torch.tensor([[0.0, 0.0], [5.0, 0.0]], dtype=torch.float64)
    assert float(boundary_penalty(inside, p.size, p.board)) == 0.0
    outside = torch.tensor([[60.0, 0.0], [0.0, 0.0]], dtype=torch.float64)
    assert float(boundary_penalty(outside, p.size, p.board)) > 0


def test_optimize_pulls_connected_parts_together():
    p = two_part_problem(d=20.0)
    r = optimize(p, iters=300, lr=0.3)
    assert r["hpwl_after"] < r["hpwl_before"] * 0.5
    assert r["overlap_after"] < 0.5  # but not on top of each other


def test_locked_part_does_not_move():
    p = two_part_problem(d=20.0, movable=(False, True))
    r = optimize(p, iters=300, lr=0.3)
    assert "U1" not in r["positions"]  # locked part emitted no move
    assert r["hpwl_after"] < r["hpwl_before"]  # U2 came to U1


def test_build_problem_from_extract_shape():
    data = {
        "footprints": [
            {"ref": "U1", "x": 10, "y": 10, "w": 5, "h": 5, "locked": False,
             "pads": [{"net": "VCC", "dx": 1, "dy": 0}, {"net": "GND", "dx": -1, "dy": 0}]},
            {"ref": "C1", "x": 30, "y": 10, "w": 1, "h": 2, "locked": False,
             "pads": [{"net": "VCC", "dx": 0, "dy": 0.5}, {"net": "GND", "dx": 0, "dy": -0.5}]},
            {"ref": "J1", "x": 0, "y": 0, "w": 8, "h": 8, "locked": True, "pads": []},
        ],
        "board": {"x1": 0, "y1": 0, "x2": 50, "y2": 40},
    }
    prob = build_problem(data)
    assert prob.refs == ["U1", "C1", "J1"]
    assert not bool(prob.movable[2])  # locked
    assert len(prob.nets) == 2  # VCC, GND
    assert len(prob.decap) == 1  # C1 attracted to U1's VCC pad
    r = optimize(prob, iters=200, lr=0.3)
    assert r["hpwl_after"] <= r["hpwl_before"]
