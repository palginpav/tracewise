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
        coff=torch.zeros(2, 2, dtype=torch.float64),
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


def test_legalize_removes_overlap():
    from tracewise.place.core import legalize
    pos = torch.tensor([[0.0, 0.0], [0.5, 0.0], [0.2, 0.3]], dtype=torch.float64)
    size = torch.tensor([[2.0, 2.0]] * 3, dtype=torch.float64)
    movable = torch.tensor([True, True, True])
    out = legalize(pos, size, movable, (-50.0, -50.0, 50.0, 50.0))
    assert float(overlap_penalty(out, size)) == pytest.approx(0.0, abs=1e-6)


def test_legalize_respects_locked():
    from tracewise.place.core import legalize
    pos = torch.tensor([[0.0, 0.0], [0.5, 0.0]], dtype=torch.float64)
    size = torch.tensor([[2.0, 2.0]] * 2, dtype=torch.float64)
    movable = torch.tensor([False, True])
    out = legalize(pos, size, movable, (-50.0, -50.0, 50.0, 50.0))
    assert torch.allclose(out[0], pos[0])
    assert float(overlap_penalty(out, size)) == pytest.approx(0.0, abs=1e-6)


def test_center_offset_keeps_box_on_board():
    """A header anchored at pin 1: origin at left end, box center 12.5mm right.
    Boundary math must use the box center, not the origin."""
    prob = PlaceProblem(
        refs=["J1", "U1"],
        pos0=torch.tensor([[40.0, 10.0], [10.0, 10.0]], dtype=torch.float64),
        size=torch.tensor([[25.0, 2.5], [5.0, 5.0]], dtype=torch.float64),
        coff=torch.tensor([[12.5, 0.0], [0.0, 0.0]], dtype=torch.float64),
        movable=torch.tensor([True, True]),
        nets=[(torch.tensor([0, 1]), torch.zeros(2, 2, dtype=torch.float64))],
        board=(0.0, 0.0, 51.0, 21.0),
        decap=[],
    )
    r = optimize(prob, iters=300, lr=0.3)
    x = r["positions"]["J1"][0]
    # box spans [x, x+25] — right edge must stay on the 51mm board
    assert x + 25.0 <= 51.0 + 0.5
    assert x >= -0.5


def test_congestion_penalty_prefers_spread():
    from tracewise.place.core import congestion_penalty
    board = (0.0, 0.0, 40.0, 40.0)
    off = torch.zeros(2, 2, dtype=torch.float64)
    nets = [(torch.tensor([0, 1]), off), (torch.tensor([2, 3]), off)]
    clumped = torch.tensor([[20.0, 20.0], [20.5, 20.0], [20.0, 20.5], [20.5, 20.5]],
                           dtype=torch.float64)
    spread = torch.tensor([[5.0, 5.0], [35.0, 5.0], [5.0, 35.0], [35.0, 35.0]],
                          dtype=torch.float64)
    assert float(congestion_penalty(clumped, nets, board)) > \
        float(congestion_penalty(spread, nets, board))


def test_tetris_zero_overlap_on_clump():
    from tracewise.place.core import legalize_tetris
    pos = torch.tensor([[10.0, 10.0], [10.3, 10.1], [10.1, 10.4], [9.8, 9.9]],
                       dtype=torch.float64)
    size = torch.tensor([[3.0, 3.0]] * 4, dtype=torch.float64)
    movable = torch.tensor([True] * 4)
    out, rot = legalize_tetris(pos, size, movable, (0.0, 0.0, 50.0, 50.0))
    assert float(overlap_penalty(out, size)) == pytest.approx(0.0, abs=1e-9)


def test_tetris_respects_locked_and_board():
    from tracewise.place.core import legalize_tetris
    pos = torch.tensor([[5.0, 5.0], [5.2, 5.0]], dtype=torch.float64)
    size = torch.tensor([[4.0, 4.0]] * 2, dtype=torch.float64)
    movable = torch.tensor([False, True])
    out, rot = legalize_tetris(pos, size, movable, (0.0, 0.0, 20.0, 20.0))
    assert torch.allclose(out[0], pos[0])  # locked unmoved
    assert float(overlap_penalty(out, size)) == pytest.approx(0.0, abs=1e-9)
    assert 2.0 <= float(out[1, 0]) <= 18.0 and 2.0 <= float(out[1, 1]) <= 18.0


def test_tetris_rotates_to_fit_slot():
    from tracewise.place.core import legalize_tetris
    # tall 2x8 slot between two locked blocks; an 8x2 part fits only rotated
    pos = torch.tensor([[10.0, 10.0], [4.0, 10.0], [16.0, 10.0]], dtype=torch.float64)
    size = torch.tensor([[8.0, 2.0], [9.5, 20.0], [9.5, 20.0]], dtype=torch.float64)
    movable = torch.tensor([True, False, False])
    rotatable = torch.tensor([True, False, False])
    out, rot = legalize_tetris(pos, size, movable, (0.0, 0.0, 20.0, 20.0),
                               rotatable=rotatable)
    assert 0 in rot  # had to rotate
    sz = size.clone()
    sz[0] = torch.tensor([2.0, 8.0])
    assert float(overlap_penalty(out, sz)) == pytest.approx(0.0, abs=1e-9)


def test_patch_data_flip_swaps_layers():
    from tracewise.route.engine.eccf import patch_data
    data = {"pads": [
        {"ref": "C1", "x": 10.0, "y": 10.0, "hw": 0.5, "hh": 0.5,
         "front": True, "back": False, "net": "VCC"},
        {"ref": "C1", "x": 11.0, "y": 10.0, "hw": 0.5, "hh": 0.5,
         "front": True, "back": False, "net": "GND"},
    ]}
    out = patch_data(data, "C1", 10.5, 10.0, 0.0, flip=True)
    assert all(p["back"] and not p["front"] for p in out["pads"])
    # original untouched (deepcopy)
    assert all(p["front"] for p in data["pads"])


def test_rank_by_storm_orders_by_density():
    from tracewise.route.engine.eccf import rank_by_storm
    parts = [
        {"ref": "C1", "x": 0.0, "y": 0.0},   # 3 hot points within 4mm
        {"ref": "C2", "x": 50.0, "y": 50.0},  # none nearby
        {"ref": "R1", "x": 1.0, "y": 1.0},   # 3 hot points within 4mm
    ]
    hot = [(0.5, 0.5), (1.0, 0.0), (0.0, 1.5)]
    ranked = rank_by_storm(parts, hot, radius=4.0)
    refs = [fp["ref"] for _, fp in ranked]
    assert "C2" not in refs  # zero hot points -> excluded
    assert set(refs) == {"C1", "R1"}
    assert all(n > 0 for n, _ in ranked)


def test_overlap_penalty_per_side():
    from tracewise.place.core import overlap_penalty
    # two 2x2 boxes at the same (x,y)
    pos = torch.tensor([[10.0, 10.0], [10.0, 10.0]], dtype=torch.float64)
    size = torch.tensor([[2.0, 2.0], [2.0, 2.0]], dtype=torch.float64)
    same = torch.tensor([0, 0])
    opp = torch.tensor([0, 1])
    assert float(overlap_penalty(pos, size, same)) == pytest.approx(4.0)  # collide
    assert float(overlap_penalty(pos, size, opp)) == pytest.approx(0.0)   # opposite sides
    assert float(overlap_penalty(pos, size)) == pytest.approx(4.0)  # side=None back-compat


def test_legalize_tetris_per_side_no_false_separation():
    from tracewise.place.core import legalize_tetris, overlap_penalty
    # two parts stacked at the same spot, opposite sides — must NOT be moved
    pos = torch.tensor([[10.0, 10.0], [10.0, 10.0]], dtype=torch.float64)
    size = torch.tensor([[3.0, 3.0], [3.0, 3.0]], dtype=torch.float64)
    movable = torch.tensor([True, True])
    side = torch.tensor([0, 1])
    out, _ = legalize_tetris(pos, size, movable, (0.0, 0.0, 50.0, 50.0), side=side)
    assert torch.allclose(out, pos)  # opposite sides: legal as-is, no separation
    assert float(overlap_penalty(out, size, side)) == pytest.approx(0.0)
    # same side: must separate
    out2, _ = legalize_tetris(pos, size, movable, (0.0, 0.0, 50.0, 50.0),
                              side=torch.tensor([0, 0]))
    assert float(overlap_penalty(out2, size, torch.tensor([0, 0]))) == pytest.approx(0.0, abs=1e-9)
    assert not torch.allclose(out2, pos)  # same side: had to move


def test_back_free_fraction_poured_vs_empty(tmp_path):
    from tracewise.route.engine.eccf import back_free_fraction
    # a board whose B.Cu pour bbox covers the whole zone-vertex extent -> ~0 free
    poured = tmp_path / "poured.kicad_pcb"
    poured.write_text("""(kicad_pcb (version 1) (generator "t")
      (zone (net 1) (layer "B.Cu") (polygon (pts
        (xy 0 0) (xy 100 0) (xy 100 100) (xy 0 100)))))""")
    assert back_free_fraction(poured) < 0.01
    # front-only pour -> back is empty -> ~1.0 free
    front = tmp_path / "front.kicad_pcb"
    front.write_text("""(kicad_pcb (version 1) (generator "t")
      (zone (net 1) (layer "F.Cu") (polygon (pts
        (xy 0 0) (xy 100 0) (xy 100 100) (xy 0 100)))))""")
    assert back_free_fraction(front) > 0.99
