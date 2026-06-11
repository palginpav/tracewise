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
    coff: torch.Tensor  # [N,2] box-center offset from footprint origin
    movable: torch.Tensor  # [N] bool
    # nets: list of (fp_index tensor [k], offset tensor [k,2])
    nets: list[tuple[torch.Tensor, torch.Tensor]]
    board: tuple[float, float, float, float]  # x1,y1,x2,y2
    # decoupling pairs: (cap_index, target_fp_index, target_offset[2])
    decap: list[tuple[int, int, torch.Tensor]]
    net_w: torch.Tensor | None = None  # optional per-net HPWL weights


def build_problem(data: dict, lock_refs: set[str] | None = None,
                  net_weights: dict[str, float] | None = None) -> PlaceProblem:
    lock_refs = lock_refs or set()
    fps = data["footprints"]
    refs = [f["ref"] for f in fps]
    pos0 = torch.tensor([[f["x"], f["y"]] for f in fps], dtype=torch.float64)
    size = torch.tensor([[max(f["w"], 0.1), max(f["h"], 0.1)] for f in fps], dtype=torch.float64)
    coff = torch.tensor([[f.get("cx", 0.0), f.get("cy", 0.0)] for f in fps], dtype=torch.float64)
    movable = torch.tensor(
        [not f["locked"] and f["ref"] not in lock_refs for f in fps], dtype=torch.bool
    )

    by_net: dict[str, list[tuple[int, float, float]]] = {}
    for i, f in enumerate(fps):
        for p in f["pads"]:
            if p["net"]:
                by_net.setdefault(p["net"], []).append((i, p["dx"], p["dy"]))
    nets = []
    weights = []
    for net_name, pins in by_net.items():
        if len(pins) < 2:
            continue
        ii = torch.tensor([p[0] for p in pins], dtype=torch.long)
        off = torch.tensor([[p[1], p[2]] for p in pins], dtype=torch.float64)
        nets.append((ii, off))
        weights.append((net_weights or {}).get(net_name, 1.0))

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
        refs=refs, pos0=pos0, size=size, coff=coff, movable=movable, nets=nets,
        board=(data["board"]["x1"], data["board"]["y1"],
               data["board"]["x2"], data["board"]["y2"]),
        decap=decap,
        net_w=torch.tensor(weights, dtype=torch.float64) if net_weights else None,
    )


def smooth_hpwl(pos: torch.Tensor, nets, tau: float = 1.0,
                net_w: torch.Tensor | None = None) -> torch.Tensor:
    total = pos.new_zeros(())
    for k, (ii, off) in enumerate(nets):
        pins = pos[ii] + off
        w = float(net_w[k]) if net_w is not None else 1.0
        for d in (0, 1):
            v = pins[:, d]
            total = total + w * (tau * torch.logsumexp(v / tau, 0)
                                 + tau * torch.logsumexp(-v / tau, 0))
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


def congestion_penalty(pos: torch.Tensor, nets, board, bins: int = 24,
                       sigma: float = 1.5, cap_factor: float = 1.6) -> torch.Tensor:
    """Differentiable routing-demand proxy: pin density. Every pin splats a
    Gaussian onto a coarse bin grid; density above capacity is penalized
    quadratically. Pins are where wires must terminate — crowded pins mean
    corridors that cannot exist, which no router fixes afterward (measured:
    the v0.3 placer's HPWL-only layouts routed worse than human ones)."""
    pins = []
    for ii, off in nets:
        pins.append(pos[ii] + off)
    if not pins:
        return pos.new_zeros(())
    pts = torch.cat(pins, dim=0)  # [P,2]
    x1, y1, x2, y2 = board
    gx = torch.linspace(x1, x2, bins, dtype=pos.dtype)
    gy = torch.linspace(y1, y2, bins, dtype=pos.dtype)
    cx, cy = torch.meshgrid(gx, gy, indexing="ij")
    centers = torch.stack([cx.reshape(-1), cy.reshape(-1)], dim=1)  # [B²,2]
    d2 = ((pts[:, None, :] - centers[None, :, :]) ** 2).sum(-1)  # [P,B²]
    dens = torch.exp(-d2 / (2 * sigma * sigma)).sum(0)  # [B²]
    cap = dens.sum() / (bins * bins) * cap_factor
    over = torch.relu(dens - cap)
    return (over * over).sum()


def decap_penalty(pos: torch.Tensor, decap) -> torch.Tensor:
    if not decap:
        return pos.new_zeros(())
    total = pos.new_zeros(())
    for ci, ti, toff in decap:
        d = pos[ci] - (pos[ti] + toff)
        total = total + (d * d).sum()
    return total


def legalize(
    pos: torch.Tensor,
    size: torch.Tensor,
    movable: torch.Tensor,
    board,
    iters: int = 800,
    margin: float = 0.25,
) -> torch.Tensor:
    """Remove residual overlaps by iterative pairwise push-apart along the
    axis of least separation (locked parts never move), then clamp to the
    board. Simple relaxation — adequate at PCB densities; not a packer."""
    pos = pos.clone()
    half = (size + margin) / 2
    x1, y1, x2, y2 = board
    mv = movable.double().unsqueeze(1)
    for _ in range(iters):
        d = pos[:, None, :] - pos[None, :, :]  # [N,N,2]
        need = half[:, None, :] + half[None, :, :]
        ov = need - d.abs()  # positive where overlapping per-axis
        overlapping = (ov[..., 0] > 0) & (ov[..., 1] > 0)
        overlapping = overlapping & ~torch.eye(len(pos), dtype=torch.bool)
        if not overlapping.any():
            break
        push = torch.zeros_like(pos)
        # push along the axis with the smaller penetration
        axis = (ov[..., 0] > ov[..., 1]).long()  # 1 -> push in y, 0 -> push in x
        for a in (0, 1):
            sel = overlapping & (axis == a)
            if not sel.any():
                continue
            sign = torch.sign(d[..., a]) + (d[..., a] == 0).double()
            amount = ov[..., a] * sel.double() * sign * 0.2
            push[:, a] += amount.sum(dim=1)
        pos = pos + push.clamp(-2.0, 2.0) * mv
        clamped = torch.stack(
            [
                torch.maximum(torch.minimum(pos[:, 0], x2 - half[:, 0]), x1 + half[:, 0]),
                torch.maximum(torch.minimum(pos[:, 1], y2 - half[:, 1]), y1 + half[:, 1]),
            ],
            dim=1,
        )
        pos = torch.where(movable.unsqueeze(1), clamped, pos)
    return pos


def legalize_tetris(
    pos: torch.Tensor,
    size: torch.Tensor,
    movable: torch.Tensor,
    board,
    margin: float = 0.2,
    step: float = 0.25,
    max_r: float = 12.0,
    rotatable: torch.Tensor | None = None,
) -> tuple[torch.Tensor, set[int]]:
    """Tetris-style legalization in box-center space: components legalize
    one at a time (largest first), each snapping to the nearest position
    (expanding ring search) that overlaps nothing already legal. Locked parts
    are immovable obstacles. Guarantees pairwise-legal boxes where the board
    has room — the measured gate for routability (overlapping courtyards
    block routing corridors directly)."""
    x1, y1, x2, y2 = board
    half = (size + margin) / 2
    n = len(pos)
    out = pos.clone()
    placed: list[tuple[float, float, float, float]] = []
    for i in range(n):
        if not bool(movable[i]):
            hx, hy = float(half[i, 0]), float(half[i, 1])
            cx, cy = float(out[i, 0]), float(out[i, 1])
            placed.append((cx - hx, cy - hy, cx + hx, cy + hy))

    # ring offsets sorted by distance, generated once
    k = int(max_r / step)
    offs = sorted(((dx * step, dy * step) for dx in range(-k, k + 1)
                   for dy in range(-k, k + 1)),
                  key=lambda o: o[0] * o[0] + o[1] * o[1])

    rotated: set[int] = set()
    order = sorted((i for i in range(n) if bool(movable[i])),
                   key=lambda i: -float(size[i, 0] * size[i, 1]))
    for i in order:
        dx0, dy0 = float(out[i, 0]), float(out[i, 1])
        variants = [(float(half[i, 0]), float(half[i, 1]), False)]
        can_rot = rotatable is not None and bool(rotatable[i])
        if can_rot and abs(float(half[i, 0]) - float(half[i, 1])) > step:
            variants.append((float(half[i, 1]), float(half[i, 0]), True))
        placed_ok = False
        for ox, oy in offs:
            for hx, hy, rot in variants:
                cx = min(max(dx0 + ox, x1 + hx), x2 - hx)
                cy = min(max(dy0 + oy, y1 + hy), y2 - hy)
                r = (cx - hx, cy - hy, cx + hx, cy + hy)
                if all(r[2] <= q[0] or q[2] <= r[0] or r[3] <= q[1] or q[3] <= r[1]
                       for q in placed):
                    out[i, 0], out[i, 1] = cx, cy
                    placed.append(r)
                    placed_ok = True
                    if rot:
                        rotated.add(i)
                    break
            if placed_ok:
                break
        if not placed_ok:
            hxr, hyr = float(half[i, 0]), float(half[i, 1])
            cx, cy = float(out[i, 0]), float(out[i, 1])
            placed.append((cx - hxr, cy - hyr, cx + hxr, cy + hyr))  # leave as-is
    return out, rotated


def optimize(
    prob: PlaceProblem,
    iters: int = 800,
    lr: float = 0.5,
    w_overlap_final: float = 4.0,
    w_decap: float = 0.02,
    w_congestion: float = 0.3,
    rotate: bool = False,
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
            smooth_hpwl(pos, prob.nets, tau=tau, net_w=prob.net_w)
            + w_ov * overlap_penalty(pos + prob.coff, prob.size)
            + 10.0 * boundary_penalty(pos + prob.coff, prob.size, prob.board)
            + w_decap * decap_penalty(pos, prob.decap)
            + (w_congestion * t) * congestion_penalty(pos, prob.nets, prob.board)
        )
        cost.backward()
        opt.step()
    final = prob.pos0 + delta.detach() * mask
    centers = final + prob.coff
    overlap_global = float(overlap_penalty(centers, prob.size))
    # rotation candidates: origin ≈ box center (passives) — orientation swap
    # keeps the box model exact without offset-rotation bookkeeping
    rotatable = None
    if rotate:  # v1 box-fit rotation measured NEGATIVE (scrambles pad axes);
        # off by default until rotation scoring includes rotated-offset HPWL
        rotatable = (prob.coff.abs().amax(dim=1)
                     < 0.3 * prob.size.amin(dim=1)) & prob.movable
    centers, rotated = legalize_tetris(centers, prob.size, prob.movable,
                                       prob.board, rotatable=rotatable)
    final = centers - prob.coff
    return {
        "positions": {r: (float(final[i, 0]), float(final[i, 1]),
                          90.0 if i in rotated else 0.0)
                      for i, r in enumerate(prob.refs) if bool(prob.movable[i])},
        "rotated": len(rotated),
        "hpwl_before": true_hpwl(prob.pos0, prob.nets),
        "overlap_initial": float(overlap_penalty(prob.pos0 + prob.coff, prob.size)),
        "hpwl_after": true_hpwl(final, prob.nets),
        "overlap_global": overlap_global,
        "overlap_after": float(overlap_penalty(final + prob.coff, prob.size)),
    }
