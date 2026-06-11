"""The analytical placement core: differentiable costs + Adam.

Cost = wirelength + overlap + boundary + decoupling, all in mm units:

- **Wirelength** — smooth half-perimeter (HPWL) per net via logsumexp
  (temperature-annealed toward the true max/min)
- **Overlap** — pairwise soft rectangle intersection area between footprint
  bounding boxes, weight annealed upward so parts spread late, route early
- **Boundary** — quadratic penalty for leaving the board outline (with the
  footprint's own half-extent respected)
- **Decoupling** — capacitors are attracted to the nearest same-net IC power
  pad (the electronics-aware term a generic placer lacks)

Locked footprints contribute to costs but receive no gradient.
"""

from __future__ import annotations

from dataclasses import dataclass

import torch


@dataclass
class PlaceProblem:
    refs: list[str]
    pos0: torch.Tensor  # [N,2] initial positions (mm)
    size: torch.Tensor  # [N,2] (w,h)
    movable: torch.Tensor  # [N] bool
    # nets: list of (fp_index tensor [k], offset tensor [k,2])
    nets: list[tuple[torch.Tensor, torch.Tensor]]
    board: tuple[float, float, float, float]  # x1,y1,x2,y2
    # decoupling pairs: (cap_index, target_fp_index, target_offset[2])
    decap: list[tuple[int, int, torch.Tensor]]


def build_problem(data: dict, lock_refs: set[str] | None = None) -> PlaceProblem:
    lock_refs = lock_refs or set()
    fps = data["footprints"]
    refs = [f["ref"] for f in fps]
    pos0 = torch.tensor([[f["x"], f["y"]] for f in fps], dtype=torch.float64)
    size = torch.tensor([[max(f["w"], 0.1), max(f["h"], 0.1)] for f in fps], dtype=torch.float64)
    movable = torch.tensor(
        [not f["locked"] and f["ref"] not in lock_refs for f in fps], dtype=torch.bool
    )

    by_net: dict[str, list[tuple[int, float, float]]] = {}
    for i, f in enumerate(fps):
        for p in f["pads"]:
            if p["net"]:
                by_net.setdefault(p["net"], []).append((i, p["dx"], p["dy"]))
    nets = []
    for pins in by_net.values():
        if len(pins) < 2:
            continue
        ii = torch.tensor([p[0] for p in pins], dtype=torch.long)
        off = torch.tensor([[p[1], p[2]] for p in pins], dtype=torch.float64)
        nets.append((ii, off))

    # decoupling: capacitor refs (C*) sharing a non-GND net with another part's
    # power-ish pad — attract the cap to that pad
    decap = []
    for net, pins in by_net.items():
        if net.upper().rsplit("/", 1)[-1] in ("GND", "VSS", "AGND", "DGND"):
            continue
        caps = [p for p in pins if refs[p[0]].startswith("C")]
        others = [p for p in pins if not refs[p[0]].startswith("C")]
        for c in caps:
            if others:
                t = others[0]
                decap.append((c[0], t[0], torch.tensor([t[1], t[2]], dtype=torch.float64)))

    return PlaceProblem(
        refs=refs, pos0=pos0, size=size, movable=movable, nets=nets,
        board=(data["board"]["x1"], data["board"]["y1"],
               data["board"]["x2"], data["board"]["y2"]),
        decap=decap,
    )


def smooth_hpwl(pos: torch.Tensor, nets, tau: float = 1.0) -> torch.Tensor:
    total = pos.new_zeros(())
    for ii, off in nets:
        pins = pos[ii] + off
        for d in (0, 1):
            v = pins[:, d]
            total = total + tau * torch.logsumexp(v / tau, 0) + tau * torch.logsumexp(-v / tau, 0)
    return total


def true_hpwl(pos: torch.Tensor, nets) -> float:
    total = 0.0
    for ii, off in nets:
        pins = pos[ii] + off
        total += float(pins[:, 0].max() - pins[:, 0].min())
        total += float(pins[:, 1].max() - pins[:, 1].min())
    return total


def overlap_penalty(pos: torch.Tensor, size: torch.Tensor) -> torch.Tensor:
    half = size / 2
    lo, hi = pos - half, pos + half
    ox = torch.relu(torch.minimum(hi[:, None, 0], hi[None, :, 0])
                    - torch.maximum(lo[:, None, 0], lo[None, :, 0]))
    oy = torch.relu(torch.minimum(hi[:, None, 1], hi[None, :, 1])
                    - torch.maximum(lo[:, None, 1], lo[None, :, 1]))
    area = ox * oy
    area = area - torch.diag_embed(torch.diagonal(area))  # zero self-overlap
    return area.sum() / 2


def boundary_penalty(pos: torch.Tensor, size: torch.Tensor, board) -> torch.Tensor:
    x1, y1, x2, y2 = board
    half = size / 2
    p = (
        torch.relu(x1 + half[:, 0] - pos[:, 0]) ** 2
        + torch.relu(pos[:, 0] - (x2 - half[:, 0])) ** 2
        + torch.relu(y1 + half[:, 1] - pos[:, 1]) ** 2
        + torch.relu(pos[:, 1] - (y2 - half[:, 1])) ** 2
    )
    return p.sum()


def decap_penalty(pos: torch.Tensor, decap) -> torch.Tensor:
    if not decap:
        return pos.new_zeros(())
    total = pos.new_zeros(())
    for ci, ti, toff in decap:
        d = pos[ci] - (pos[ti] + toff)
        total = total + (d * d).sum()
    return total


def optimize(
    prob: PlaceProblem,
    iters: int = 800,
    lr: float = 0.5,
    w_overlap_final: float = 4.0,
    w_decap: float = 0.02,
    seed: int = 0,
) -> dict:
    torch.manual_seed(seed)
    delta = torch.zeros_like(prob.pos0, requires_grad=True)
    mask = prob.movable.double().unsqueeze(1)
    opt = torch.optim.Adam([delta], lr=lr)
    for it in range(iters):
        opt.zero_grad()
        pos = prob.pos0 + delta * mask
        t = it / max(iters - 1, 1)
        tau = 2.0 * (1 - t) + 0.1  # anneal smooth-max toward true max
        w_ov = w_overlap_final * t  # let parts pass through each other early
        cost = (
            smooth_hpwl(pos, prob.nets, tau=tau)
            + w_ov * overlap_penalty(pos, prob.size)
            + 10.0 * boundary_penalty(pos, prob.size, prob.board)
            + w_decap * decap_penalty(pos, prob.decap)
        )
        cost.backward()
        opt.step()
    final = (prob.pos0 + delta.detach() * mask)
    return {
        "positions": {r: (float(final[i, 0]), float(final[i, 1]))
                      for i, r in enumerate(prob.refs) if bool(prob.movable[i])},
        "hpwl_before": true_hpwl(prob.pos0, prob.nets),
        "hpwl_after": true_hpwl(final, prob.nets),
        "overlap_after": float(overlap_penalty(final, prob.size)),
    }
