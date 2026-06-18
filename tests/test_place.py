"""Placement core tests — synthetic problems, deterministic. Self-skip
without torch (an optional [place] extra; CI installs dev only)."""

import pytest

torch = pytest.importorskip("torch")

from tracewise.place.core import (  # noqa: E402
    PlaceProblem,
    boundary_penalty,
    build_groups,
    build_problem,
    cluster_penalty,
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


# ---------------------------------------------------------------------------
# Step B: build_groups
# ---------------------------------------------------------------------------

def _make_group_data():
    """Synthetic board with:
    - U1: IC, 4 pads (anchor) on nets: SIG_A, SIG_B, VCC, GND
    - C1: 2-pin cap sharing SIG_A with U1 only  -> should group with U1
    - R1: 2-pin resistor sharing SIG_B with U1 only -> should group with U1
    - C2: 2-pin cap sharing only GND/VCC (power nets) -> must NOT group
    - U2: IC, 3 pads (anchor) on: SIG_C, VCC, GND
    - C3: 2-pin cap sharing SIG_C with BOTH U1 (we add it there too) and U2
          -> two anchors on signal net -> must NOT group (not single-anchor)
    """
    return {
        "footprints": [
            {
                "ref": "U1", "x": 10, "y": 10, "w": 5, "h": 5, "locked": False,
                "pads": [
                    {"net": "SIG_A", "dx": 1, "dy": 0},
                    {"net": "SIG_B", "dx": -1, "dy": 0},
                    {"net": "VCC", "dx": 0, "dy": 1},
                    {"net": "GND", "dx": 0, "dy": -1},
                ],
            },
            {
                "ref": "C1", "x": 20, "y": 10, "w": 1, "h": 2, "locked": False,
                "pads": [
                    {"net": "SIG_A", "dx": 0, "dy": 0.5},
                    {"net": "GND", "dx": 0, "dy": -0.5},
                ],
            },
            {
                "ref": "R1", "x": 25, "y": 10, "w": 1, "h": 2, "locked": False,
                "pads": [
                    {"net": "SIG_B", "dx": 0, "dy": 0.5},
                    {"net": "VCC", "dx": 0, "dy": -0.5},
                ],
            },
            {
                "ref": "C2", "x": 30, "y": 10, "w": 1, "h": 2, "locked": False,
                "pads": [
                    {"net": "VCC", "dx": 0, "dy": 0.5},
                    {"net": "GND", "dx": 0, "dy": -0.5},
                ],
            },
            {
                "ref": "U2", "x": 40, "y": 10, "w": 5, "h": 5, "locked": False,
                "pads": [
                    {"net": "SIG_C", "dx": 1, "dy": 0},
                    {"net": "VCC", "dx": 0, "dy": 1},
                    {"net": "GND", "dx": 0, "dy": -1},
                ],
            },
            {
                "ref": "C3", "x": 45, "y": 10, "w": 1, "h": 2, "locked": False,
                "pads": [
                    # SIG_C connects C3 to both U1 (added below) and U2
                    {"net": "SIG_C", "dx": 0, "dy": 0.5},
                    {"net": "GND", "dx": 0, "dy": -0.5},
                ],
            },
        ],
        "board": {"x1": 0, "y1": 0, "x2": 60, "y2": 30},
    }


def test_build_groups_clusters_signal_passive_with_single_anchor():
    """C1 and R1 each share exactly one signal net with U1 -> grouped."""
    data = _make_group_data()
    # Add U1 onto SIG_C too so C3 sees two anchors (U1 + U2) -> not grouped
    data["footprints"][0]["pads"].append({"net": "SIG_C", "dx": 2, "dy": 0})

    groups = build_groups(data)
    # Build a lookup: anchor_ref -> set of member refs
    refs = [f["ref"] for f in data["footprints"]]
    group_map = {}
    for g in groups:
        anchor_ref = refs[g[0]]
        member_refs = {refs[i] for i in g}
        group_map[anchor_ref] = member_refs

    # C1 and R1 should be grouped with U1
    assert "U1" in group_map, f"U1 anchor missing; groups={groups}"
    assert "C1" in group_map["U1"], "C1 (SIG_A only with U1) should be in U1 group"
    assert "R1" in group_map["U1"], "R1 (SIG_B only with U1) should be in U1 group"

    # C2 only shares power nets (VCC, GND) -> must NOT appear in any group
    all_members = {refs[i] for g in groups for i in g}
    assert "C2" not in all_members, "C2 shares only power nets — must not be grouped"

    # C3 shares SIG_C with both U1 and U2 -> two anchors -> must NOT be grouped
    assert "C3" not in all_members, "C3 has two anchors on SIG_C — must not be grouped"


def test_build_groups_skips_pour_nets_by_fanout():
    """A net with > 8 pins is treated as a rail/pour and excluded."""
    # 9-pin 'RAIL' net connecting one IC and 8 caps
    pads_ic = [{"net": "RAIL", "dx": float(i), "dy": 0.0} for i in range(4)]
    pads_ic += [{"net": "GND", "dx": 0.0, "dy": -1.0}]
    footprints = [
        {"ref": "U1", "x": 0.0, "y": 0.0, "w": 5.0, "h": 5.0, "locked": False,
         "pads": pads_ic},
    ]
    for j in range(8):
        footprints.append({
            "ref": f"C{j + 1}", "x": float(10 + j * 3), "y": 0.0,
            "w": 1.0, "h": 2.0, "locked": False,
            "pads": [
                {"net": "RAIL", "dx": 0.0, "dy": 0.5},
                {"net": "GND", "dx": 0.0, "dy": -0.5},
            ],
        })
    data = {"footprints": footprints, "board": {"x1": 0, "y1": -10, "x2": 100, "y2": 10}}
    groups = build_groups(data)
    # RAIL has 9 pins (> 8) -> treated as pour, nothing grouped
    assert groups == [], f"Expected no groups for high-fanout rail, got {groups}"


def test_build_groups_deterministic():
    """Same data produces the same groups across two calls."""
    data = _make_group_data()
    g1 = build_groups(data)
    g2 = build_groups(data)
    assert g1 == g2


# ---------------------------------------------------------------------------
# Step C: cluster_penalty
# ---------------------------------------------------------------------------

def test_cluster_penalty_zero_when_colocated():
    """All group members at the same position -> penalty is 0."""
    pos = torch.tensor([[5.0, 5.0], [5.0, 5.0], [5.0, 5.0]], dtype=torch.float64)
    groups = [[0, 1, 2]]
    val = float(cluster_penalty(pos, groups))
    assert val == pytest.approx(0.0)


def test_cluster_penalty_positive_when_separated():
    """Members separated from anchor -> penalty > 0."""
    # anchor at (0,0), two members at (3,4) and (-3,-4)
    pos = torch.tensor([[0.0, 0.0], [3.0, 4.0], [-3.0, -4.0]], dtype=torch.float64)
    groups = [[0, 1, 2]]
    # distance^2 from each member to anchor: 3^2+4^2=25 each, total=50
    val = float(cluster_penalty(pos, groups))
    assert val == pytest.approx(50.0)


def test_cluster_penalty_zero_for_empty_groups():
    pos = torch.tensor([[0.0, 0.0], [1.0, 1.0]], dtype=torch.float64)
    assert float(cluster_penalty(pos, [])) == 0.0


def test_cluster_penalty_is_differentiable():
    """backward() must produce non-None, non-zero gradients for separated members."""
    pos = torch.tensor([[0.0, 0.0], [3.0, 4.0]], dtype=torch.float64,
                       requires_grad=True)
    groups = [[0, 1]]
    loss = cluster_penalty(pos, groups)
    loss.backward()
    assert pos.grad is not None
    assert pos.grad.abs().sum().item() > 0.0


# ---------------------------------------------------------------------------
# Step C integration: optimize reduces cluster_penalty
# ---------------------------------------------------------------------------

def _make_cluster_problem():
    """One IC (anchor) at origin, one cap far away, connected by one net.
    Both are movable; board is large enough for free movement."""
    data = {
        "footprints": [
            {
                "ref": "U1", "x": 0.0, "y": 0.0, "w": 3.0, "h": 3.0, "locked": False,
                "pads": [
                    {"net": "SIG_X", "dx": 1.0, "dy": 0.0},
                    {"net": "SIG_Y", "dx": -1.0, "dy": 0.0},
                    {"net": "GND",   "dx": 0.0,  "dy": -1.0},
                ],
            },
            {
                "ref": "C1", "x": 40.0, "y": 40.0, "w": 1.0, "h": 2.0, "locked": False,
                "pads": [
                    {"net": "SIG_X", "dx": 0.0, "dy": 0.5},
                    {"net": "GND",   "dx": 0.0, "dy": -0.5},
                ],
            },
        ],
        "board": {"x1": -60.0, "y1": -60.0, "x2": 60.0, "y2": 60.0},
    }
    return data


def test_optimize_reduces_cluster_penalty_with_w_cluster():
    """With w_cluster > 0, optimizer moves C1 closer to U1."""
    data = _make_cluster_problem()
    prob = build_problem(data)
    assert len(prob.groups) >= 1, "Expected at least one group (C1 with U1)"

    # measure initial cluster_penalty using initial positions
    cp_before = float(cluster_penalty(prob.pos0, prob.groups))
    assert cp_before > 0.0, "C1 is far from U1 — initial penalty must be > 0"

    result = optimize(prob, iters=400, lr=0.5, w_cluster=0.2, seed=42)

    # reconstruct final positions tensor in index order
    pos_final = prob.pos0.clone()
    for i, ref in enumerate(prob.refs):
        if ref in result["positions"]:
            x, y, _ = result["positions"][ref]
            pos_final[i, 0] = x
            pos_final[i, 1] = y

    cp_after = float(cluster_penalty(pos_final, prob.groups))
    assert cp_after < cp_before, (
        f"cluster_penalty should decrease: before={cp_before:.3f}, after={cp_after:.3f}"
    )


def test_optimize_with_w_cluster_zero_unchanged_behavior():
    """w_cluster=0 must produce the same result as not having groups."""
    data = _make_cluster_problem()
    prob_with = build_problem(data)
    prob_without = build_problem(data)
    prob_without.groups = []

    r_with = optimize(prob_with, iters=200, lr=0.3, w_cluster=0.0, seed=7)
    r_without = optimize(prob_without, iters=200, lr=0.3, w_cluster=0.0, seed=7)

    for ref in prob_with.refs:
        if ref in r_with["positions"] and ref in r_without["positions"]:
            x1, y1, _ = r_with["positions"][ref]
            x2, y2, _ = r_without["positions"][ref]
            assert abs(x1 - x2) < 1e-6 and abs(y1 - y2) < 1e-6, (
                f"{ref}: positions differ when w_cluster=0 vs empty groups"
            )
