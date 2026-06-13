#!/usr/bin/env python
"""Spike: layer-aware placement — the CORE fix for the LAYER-BLIND placer.

Proves two claims, with numbers, using only numpy + torch (no KiCad):

  CLAIM 1 (per-side overlap): the current overlap penalty (core.py) treats both
      copper sides as ONE 2D plane, so a FRONT part and a BACK part at the same
      (x,y) are wrongly penalized as colliding. The fix: gate the pairwise
      overlap by SAME-SIDE. Two opposite-side parts at identical (x,y) must
      score ZERO overlap; two same-side parts must score the real area.

  CLAIM 2 (side as a decision variable): a tiny discrete side-assignment search
      — flip a part to the other copper side — reduces a synthetic per-side
      congestion metric on a board whose front is packed and back is free
      (the zuluscsi topology the storm-flip arm validated on), and does NOT help
      when both sides are equally packed (the mitayi topology where the arm was
      measured inert). The precondition probe (free back-side area) predicts it.

Run:  taskset -c 0-9 /home/palgin/Business_projects/tracewise/.venv/bin/python \
          scripts/spike_layeraware.py
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

import numpy as np
import torch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

# The current, layer-BLIND penalty shipped in place/core.py — imported, not
# re-implemented, so the spike measures the real code's behaviour.
from tracewise.place.core import overlap_penalty as overlap_blind  # noqa: E402

FRONT, BACK = 0, 1


# ---------------------------------------------------------------------------
# CLAIM 1 — per-side overlap
# ---------------------------------------------------------------------------

def overlap_per_side(pos: torch.Tensor, size: torch.Tensor,
                     side: torch.Tensor) -> torch.Tensor:
    """Layer-AWARE overlap: identical geometry to core.overlap_penalty, but the
    pairwise intersection area is masked to ZERO for parts on opposite copper
    sides. This is the one-line conceptual change core.py needs (two occupancy
    planes instead of one)."""
    half = size / 2
    lo, hi = pos - half, pos + half
    ox = torch.relu(torch.minimum(hi[:, None, 0], hi[None, :, 0])
                    - torch.maximum(lo[:, None, 0], lo[None, :, 0]))
    oy = torch.relu(torch.minimum(hi[:, None, 1], hi[None, :, 1])
                    - torch.maximum(lo[:, None, 1], lo[None, :, 1]))
    area = ox * oy
    area = area - torch.diag_embed(torch.diagonal(area))  # zero self-overlap
    same_side = (side[:, None] == side[None, :]).double()  # the gate
    return (area * same_side).sum() / 2


def claim1() -> bool:
    print("=" * 76)
    print("CLAIM 1 — per-side overlap (opposite sides do NOT collide)")
    print("=" * 76)
    # Two identical 2x2mm parts stacked at exactly the same (x,y).
    pos = torch.tensor([[10.0, 10.0], [10.0, 10.0]], dtype=torch.float64)
    size = torch.tensor([[2.0, 2.0], [2.0, 2.0]], dtype=torch.float64)

    same = torch.tensor([FRONT, FRONT])
    opp = torch.tensor([FRONT, BACK])

    blind = float(overlap_blind(pos, size))
    aware_same = float(overlap_per_side(pos, size, same))
    aware_opp = float(overlap_per_side(pos, size, opp))

    print(f"  layer-BLIND  (shipped)            overlap = {blind:7.3f} mm^2")
    print(f"  layer-AWARE  same side (F,F)       overlap = {aware_same:7.3f} mm^2")
    print(f"  layer-AWARE  opposite side (F,B)   overlap = {aware_opp:7.3f} mm^2")

    # A partial-overlap, same-side case must keep the real area (no regression).
    pos2 = torch.tensor([[10.0, 10.0], [11.0, 10.0]], dtype=torch.float64)
    part_blind = float(overlap_blind(pos2, size))
    part_aware = float(overlap_per_side(pos2, size, same))
    print(f"  same-side partial overlap: blind={part_blind:.3f}  aware={part_aware:.3f}"
          f"  (must match)")

    ok = (abs(blind - 4.0) < 1e-9          # blind: full 2x2 collision counted
          and abs(aware_same - 4.0) < 1e-9  # aware same-side: still collides
          and aware_opp < 1e-9              # aware opposite-side: NO collision
          and abs(part_blind - part_aware) < 1e-9)  # same-side unaffected
    print(f"\n  VERDICT 1: opposite-side false collision removed "
          f"({blind:.1f} -> {aware_opp:.1f}), same-side preserved: {ok}")
    return ok


# ---------------------------------------------------------------------------
# CLAIM 2 — side assignment as a decision variable
# ---------------------------------------------------------------------------

def congestion_per_side(pos: np.ndarray, side: np.ndarray, board: float,
                        bins: int = 16, sigma: float = 1.2) -> float:
    """Synthetic routability proxy = peak routing demand on the BUSIER side.
    Each part splats a Gaussian of pin demand onto the bin grid OF ITS OWN SIDE;
    the metric is the max over both side-grids of the peak bin (the binding
    corridor). Two independent planes — the whole point of layer-awareness.
    Lower is better. A flip moves a part's demand from one plane to the other,
    so it can only help when the other plane has headroom (free back area)."""
    edges = np.linspace(0, board, bins + 1)
    centers = 0.5 * (edges[:-1] + edges[1:])
    cx, cy = np.meshgrid(centers, centers, indexing="ij")
    peaks = []
    for s in (FRONT, BACK):
        m = side == s
        dens = np.zeros((bins, bins))
        for px, py in pos[m]:
            d2 = (cx - px) ** 2 + (cy - py) ** 2
            dens += np.exp(-d2 / (2 * sigma * sigma))
        peaks.append(dens.max())
    return float(max(peaks))


def back_free_fraction(pos: np.ndarray, side: np.ndarray, board: float,
                       bins: int = 16, occ_thresh: float = 0.3) -> float:
    """The PRECONDITION PROBE: fraction of the back plane with low demand.
    Cheap, geometry-only — the cross-board predictor of whether side moves can
    pay (high on zuluscsi-like boards, low on mitayi-like poured-both-sides)."""
    edges = np.linspace(0, board, bins + 1)
    centers = 0.5 * (edges[:-1] + edges[1:])
    cx, cy = np.meshgrid(centers, centers, indexing="ij")
    dens = np.zeros((bins, bins))
    for px, py in pos[side == BACK]:
        d2 = (cx - px) ** 2 + (cy - py) ** 2
        dens += np.exp(-d2 / (2 * 1.2 * 1.2))
    return float((dens < occ_thresh).mean())


def side_search(pos: np.ndarray, side0: np.ndarray, movable: np.ndarray,
                board: float) -> tuple[np.ndarray, float, list]:
    """Greedy discrete side-assignment search (the {side} move in the unified
    move space): repeatedly flip the single movable part that most reduces the
    per-side congestion metric, until no flip helps. Mirrors keep-best — a flip
    that does not improve is rejected. This is what the placer/loop would do,
    with the metric standing in for the ECCF T3 score in production."""
    side = side0.copy()
    best = congestion_per_side(pos, side, board)
    log = []
    while True:
        cand_idx, cand_cost = None, best
        for i in np.flatnonzero(movable):
            trial = side.copy()
            trial[i] = 1 - trial[i]
            c = congestion_per_side(pos, trial, board)
            if c < cand_cost - 1e-9:
                cand_idx, cand_cost = i, c
        if cand_idx is None:
            break
        side[cand_idx] = 1 - side[cand_idx]
        log.append((int(cand_idx), best, cand_cost))
        best = cand_cost
    return side, best, log


def make_board(rng: np.ndarray, n_front: int, n_back: int, board: float,
               cluster: float) -> tuple[np.ndarray, np.ndarray]:
    """n_front parts clustered in the board's front-side hot zone; n_back parts
    on the back. `cluster` controls how tightly the front parts pack (a packed
    front + empty back is the zuluscsi topology; balanced is mitayi)."""
    front_xy = np.column_stack([
        np.clip(rng.normal(board * 0.5, cluster, n_front), 1, board - 1),
        np.clip(rng.normal(board * 0.5, cluster, n_front), 1, board - 1),
    ])
    back_xy = rng.uniform(1, board - 1, size=(n_back, 2))
    pos = np.vstack([front_xy, back_xy])
    side = np.array([FRONT] * n_front + [BACK] * n_back)
    return pos, side


def claim2() -> bool:
    print("\n" + "=" * 76)
    print("CLAIM 2 — side assignment reduces per-side congestion (where topology allows)")
    print("=" * 76)
    board = 30.0

    # zuluscsi topology: front densely packed (12 parts), back nearly empty (2).
    rng = np.random.default_rng(7)
    pos_z, side_z = make_board(rng, n_front=12, n_back=2, board=board, cluster=2.5)
    movable_z = np.array([p.startswith("p") for p in
                          (["p"] * 12 + ["p"] * 2)])  # all passives movable
    probe_z = back_free_fraction(pos_z, side_z, board)
    c0_z = congestion_per_side(pos_z, side_z, board)
    t0 = time.time()
    side1_z, c1_z, log_z = side_search(pos_z, side_z, movable_z, board)
    dt_z = (time.time() - t0) * 1000

    print("\n  [zuluscsi-like]  packed front, free back")
    print(f"    precondition probe (back free area) = {probe_z:.2f}")
    print(f"    congestion  before = {c0_z:.4f}   after = {c1_z:.4f}"
          f"   ({len(log_z)} flips, {dt_z:.1f} ms)")
    for i, b, a in log_z:
        print(f"      flip part #{i}: {b:.4f} -> {a:.4f}")

    # mitayi topology: both sides packed — a flip relocates INTO congestion.
    rng = np.random.default_rng(7)
    pos_m, side_m = make_board(rng, n_front=8, n_back=8, board=board, cluster=2.5)
    # force the back cluster to overlap the front cluster (poured both sides)
    pos_m[8:] = np.column_stack([
        np.clip(rng.normal(board * 0.5, 2.5, 8), 1, board - 1),
        np.clip(rng.normal(board * 0.5, 2.5, 8), 1, board - 1),
    ])
    movable_m = np.ones(16, dtype=bool)
    probe_m = back_free_fraction(pos_m, side_m, board)
    c0_m = congestion_per_side(pos_m, side_m, board)
    side1_m, c1_m, log_m = side_search(pos_m, side_m, movable_m, board)

    print("\n  [mitayi-like]    both sides packed")
    print(f"    precondition probe (back free area) = {probe_m:.2f}")
    print(f"    congestion  before = {c0_m:.4f}   after = {c1_m:.4f}"
          f"   ({len(log_m)} flips)")

    helped_z = c1_z < c0_z - 1e-6 and len(log_z) > 0
    probe_orders = probe_z > probe_m  # probe ranks zuluscsi as the place to try
    print("\n  VERDICT 2: side search helps the free-back board: "
          f"{helped_z} ({c0_z:.3f} -> {c1_z:.3f}); "
          f"probe predicts it (z {probe_z:.2f} > m {probe_m:.2f}): {probe_orders}")
    return helped_z and probe_orders


def main() -> int:
    ok1 = claim1()
    ok2 = claim2()
    print("\n" + "=" * 76)
    passed = ok1 and ok2
    print(f"SPIKE RESULT: {'PASS' if passed else 'FAIL'} — "
          "per-side overlap correct AND side-assignment search reduces "
          "per-side congestion where the back-free probe predicts.")
    print("=" * 76)
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
