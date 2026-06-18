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
    # per-part pins: idx -> list of (net_index, dx, dy) for rotation scoring
    part_pins: dict | None = None
    side: torch.Tensor | None = None  # [N] int, 0=front 1=back (layer-aware)
    # functional groups: each inner list is a set of footprint indices that
    # should cluster together (anchor first, then its associated passives)
    groups: list[list[int]] = None

    def __post_init__(self):
        if self.groups is None:
            self.groups = []


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
    side = torch.tensor([int(f.get("side", 0)) for f in fps], dtype=torch.long)

    by_net: dict[str, list[tuple[int, float, float]]] = {}
    for i, f in enumerate(fps):
        for p in f["pads"]:
            if p["net"]:
                by_net.setdefault(p["net"], []).append((i, p["dx"], p["dy"]))
    nets = []
    weights = []
    part_pins: dict[int, list] = {}
    for net_name, pins in by_net.items():
        if len(pins) < 2:
            continue
        k = len(nets)
        ii = torch.tensor([p[0] for p in pins], dtype=torch.long)
        off = torch.tensor([[p[1], p[2]] for p in pins], dtype=torch.float64)
        nets.append((ii, off))
        weights.append((net_weights or {}).get(net_name, 1.0))
        for i, dx, dy in pins:
            part_pins.setdefault(i, []).append((k, dx, dy))

    # decoupling: capacitor refs (C*) sharing a non-GND (power) net with another
    # part's pad — attract the cap to the NEAREST such pad. Pairing to the first
    # pad (others[0]) attached a decoupling cap to an arbitrary supply pin
    # instead of the IC power pin it actually bypasses; choosing the closest
    # supply pad by initial position keeps the cap on the right side of the
    # shortest power loop.
    decap = []
    for net, pins in by_net.items():
        if net.upper().rsplit("/", 1)[-1] in ("GND", "VSS", "AGND", "DGND"):
            continue
        caps = [p for p in pins if refs[p[0]].startswith("C")]
        others = [p for p in pins if not refs[p[0]].startswith("C")]
        for c in caps:
            if others:
                cx = float(pos0[c[0]][0]) + c[1]
                cy = float(pos0[c[0]][1]) + c[2]
                t = min(others, key=lambda o: (float(pos0[o[0]][0]) + o[1] - cx) ** 2
                        + (float(pos0[o[0]][1]) + o[2] - cy) ** 2)
                decap.append((c[0], t[0], torch.tensor([t[1], t[2]], dtype=torch.float64)))

    groups = build_groups(data, by_net=by_net)
    return PlaceProblem(
        refs=refs, pos0=pos0, size=size, coff=coff, movable=movable, nets=nets,
        board=(data["board"]["x1"], data["board"]["y1"],
               data["board"]["x2"], data["board"]["y2"]),
        decap=decap,
        net_w=torch.tensor(weights, dtype=torch.float64) if net_weights else None,
        part_pins=part_pins,
        side=side,
        groups=groups,
    )


_POUR_NET_SUFFIXES = frozenset([
    # ground variants
    "GND", "VSS", "AGND", "DGND", "PGND", "SGND", "GNDA", "GNDD",
    # supply rail variants (single-name nets like VCC, VDD, 3V3, 5V, VBUS)
    "VCC", "VDD", "AVCC", "DVDD", "AVDD", "VIN", "VOUT", "VBUS",
    "3V3", "5V", "3V0", "1V8", "1V2", "VCCA", "VCCD",
])
_POUR_NET_MAX_PINS = 8  # nets with more pins than this are power rails / pours


def build_groups(
    data: dict,
    by_net: dict[str, list[tuple[int, float, float]]] | None = None,
) -> list[list[int]]:
    """Return functional clusters: groups of footprint indices that should
    be placed near each other because they form a sub-circuit unit.

    Heuristic (connectivity-only, no schematic hierarchy required):

    **Anchor** — any part with >= 3 pads (IC, connector, crystal, regulator).

    **2-pin passive** — ref starts with "C" (capacitor) or "R" (resistor) and
    has exactly 2 pads.

    For each 2-pin passive P, find the set of anchor parts P shares a
    *signal* net with. Signal nets are those that are NOT:
      - power/ground nets (name ends in GND/VSS/AGND/DGND/… after the last
        "/" hierarchy separator), and
      - rails/pour nets (> ``_POUR_NET_MAX_PINS`` pins — these connect
        everything and would create one giant spurious group).

    If P connects to **exactly one** anchor via such signal nets, P is added
    to that anchor's group. This is the *single-anchor rule* — it captures
    crystal load-caps (XIN/XOUT), pull-up/pull-down resistors, bypass caps
    on dedicated signal pins, etc., WITHOUT roping in bulk decoupling caps
    on the power rail (which share a net with multiple ICs and therefore
    do not satisfy the single-anchor rule).

    Groups contain >= 2 members and are deterministically sorted so the
    anchor index (the part with the most pads, tie-broken by lowest index)
    is first.
    """
    fps = data["footprints"]
    refs = [f["ref"] for f in fps]
    pad_counts = [len(f["pads"]) for f in fps]

    if by_net is None:
        by_net = {}
        for i, f in enumerate(fps):
            for p in f["pads"]:
                if p["net"]:
                    by_net.setdefault(p["net"], []).append((i, p["dx"], p["dy"]))

    # Identify anchor indices (>= 3 pads)
    anchor_set = {i for i, c in enumerate(pad_counts) if c >= 3}

    # Identify 2-pin passives (C* or R* with exactly 2 pads)
    def _is_2pin_passive(idx: int) -> bool:
        r = refs[idx]
        return pad_counts[idx] == 2 and (r.startswith("C") or r.startswith("R"))

    # For each signal net, collect which anchors appear on it
    # signal net: not a pour/ground name AND not a high-fanout rail
    def _is_signal_net(net_name: str, pins: list) -> bool:
        bare = net_name.rsplit("/", 1)[-1].upper()
        if bare in _POUR_NET_SUFFIXES:
            return False
        if len(pins) > _POUR_NET_MAX_PINS:
            return False
        return True

    # passive_idx -> set of anchor indices sharing a signal net
    passive_anchors: dict[int, set[int]] = {}
    for net_name, pins in by_net.items():
        if not _is_signal_net(net_name, pins):
            continue
        net_part_ids = [p[0] for p in pins]
        anchors_on_net = [idx for idx in net_part_ids if idx in anchor_set]
        passives_on_net = [idx for idx in net_part_ids if _is_2pin_passive(idx)]
        for p_idx in passives_on_net:
            passive_anchors.setdefault(p_idx, set()).update(anchors_on_net)

    # Apply single-anchor rule: passive connects to exactly one anchor
    anchor_to_passives: dict[int, list[int]] = {}
    for p_idx, anchors in passive_anchors.items():
        if len(anchors) == 1:
            a_idx = next(iter(anchors))
            anchor_to_passives.setdefault(a_idx, []).append(p_idx)

    # Build groups: anchor first (sorted by pad count desc, then index asc
    # as tie-break), then its passives sorted by index
    groups: list[list[int]] = []
    for a_idx in sorted(anchor_to_passives.keys()):
        members = sorted(anchor_to_passives[a_idx])
        group = [a_idx] + members
        if len(group) >= 2:
            groups.append(group)

    return groups


def cluster_penalty(pos: torch.Tensor, groups: list[list[int]]) -> torch.Tensor:
    """Soft attraction pulling each group member toward the anchor (first index).

    For each functional group G = [anchor, passive1, passive2, …] the penalty
    is the sum of squared Euclidean distances from each non-anchor member's
    centre to the *anchor's* centre:

        penalty = Σ_{i in G[1:]} || pos[i] - pos[G[0]] ||²

    Using the anchor (not the centroid) as the attraction target means the
    pull is directional: passives move toward the anchor, while the anchor
    itself is free to move under the HPWL and other forces. This avoids the
    centroid drifting toward the most-constrained member and collapsing the
    whole group into one point.

    The function is fully differentiable via PyTorch autograd — ``pos`` is a
    leaf or intermediate tensor created inside the optimizer loop and will
    receive gradients through this term.

    Returns a scalar tensor (0.0 when ``groups`` is empty).
    """
    if not groups:
        return pos.new_zeros(())
    total = pos.new_zeros(())
    for g in groups:
        if len(g) < 2:
            continue
        anchor_pos = pos[g[0]]  # shape [2]
        for member_idx in g[1:]:
            d = pos[member_idx] - anchor_pos
            total = total + (d * d).sum()
    return total


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


def overlap_penalty(pos: torch.Tensor, size: torch.Tensor,
                    side: torch.Tensor | None = None) -> torch.Tensor:
    half = size / 2
    lo, hi = pos - half, pos + half
    ox = torch.relu(torch.minimum(hi[:, None, 0], hi[None, :, 0])
                    - torch.maximum(lo[:, None, 0], lo[None, :, 0]))
    oy = torch.relu(torch.minimum(hi[:, None, 1], hi[None, :, 1])
                    - torch.maximum(lo[:, None, 1], lo[None, :, 1]))
    area = ox * oy
    if side is not None:
        # parts on opposite copper sides occupy different physical planes and
        # cannot collide — only same-side pairs count (layer-aware placement)
        area = area * (side[:, None] == side[None, :]).to(area.dtype)
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
    side: torch.Tensor | None = None,
) -> tuple[torch.Tensor, set[int]]:
    """Tetris-style legalization in box-center space: components legalize
    one at a time (largest first), each snapping to the nearest position
    (expanding ring search) that overlaps nothing already legal ON THE SAME
    COPPER SIDE. Locked parts are immovable obstacles. Guarantees pairwise-
    legal boxes per side where the board has room — the measured gate for
    routability (overlapping courtyards block routing corridors directly)."""
    x1, y1, x2, y2 = board
    half = (size + margin) / 2
    n = len(pos)
    out = pos.clone()
    sd = side.tolist() if side is not None else [0] * n  # 0=front 1=back
    placed: list[tuple[float, float, float, float, int]] = []  # rect + side
    for i in range(n):
        if not bool(movable[i]):
            hx, hy = float(half[i, 0]), float(half[i, 1])
            cx, cy = float(out[i, 0]), float(out[i, 1])
            placed.append((cx - hx, cy - hy, cx + hx, cy + hy, sd[i]))

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
                       for q in placed if q[4] == sd[i]):  # same-side only
                    out[i, 0], out[i, 1] = cx, cy
                    placed.append((*r, sd[i]))
                    placed_ok = True
                    if rot:
                        rotated.add(i)
                    break
            if placed_ok:
                break
        if not placed_ok:
            hxr, hyr = float(half[i, 0]), float(half[i, 1])
            cx, cy = float(out[i, 0]), float(out[i, 1])
            placed.append((cx - hxr, cy - hyr, cx + hxr, cy + hyr, sd[i]))  # as-is
    return out, rotated


def optimize(
    prob: PlaceProblem,
    iters: int = 800,
    lr: float = 0.5,
    w_overlap_final: float = 4.0,
    w_decap: float = 0.02,
    w_congestion: float = 0.3,
    # w_cluster=0.1 is the measured routability sweet spot on mitayi (place+route
    # sweep: unc 110->103, err 208->193 vs no clustering; 0.2 reaches unc 98 but
    # distorts HPWL, a robustness risk without cross-board data). Functional
    # clustering monotonically improved routability — the lever validated.
    w_cluster: float = 0.1,
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
            + w_ov * overlap_penalty(pos + prob.coff, prob.size, prob.side)
            + 10.0 * boundary_penalty(pos + prob.coff, prob.size, prob.board)
            + w_decap * decap_penalty(pos, prob.decap)
            + (w_congestion * t) * congestion_penalty(pos, prob.nets, prob.board)
            + (w_cluster * t) * cluster_penalty(pos, prob.groups)
        )
        cost.backward()
        opt.step()
    final = prob.pos0 + delta.detach() * mask
    centers = final + prob.coff
    overlap_global = float(overlap_penalty(centers, prob.size, prob.side))
    # rotation candidates: origin ≈ box center (passives) — orientation swap
    # keeps the box model exact without offset-rotation bookkeeping
    rotatable = None
    if rotate and prob.part_pins:
        # v2: offer rotation only where it is wirelength-neutral or better,
        # scored with properly rotated pad offsets (+90 in KiCad: (dx,dy) ->
        # (dy,-dx), verified empirically against pcbnew). v1 (fit-only) was
        # measured negative: it scrambled pad axes.
        rotatable = (prob.coff.abs().amax(dim=1)
                     < 0.3 * prob.size.amin(dim=1)) & prob.movable
        for i in range(len(prob.refs)):
            if not bool(rotatable[i]):
                continue
            delta = 0.0
            for k, dx, dy in prob.part_pins.get(i, []):
                ii, off = prob.nets[k]
                others = [(int(j), float(off[m, 0]), float(off[m, 1]))
                          for m, j in enumerate(ii.tolist()) if j != i]
                if not others:
                    continue
                ax = sum(float(final[j, 0]) + odx for j, odx, _ in others) / len(others)
                ay = sum(float(final[j, 1]) + ody for j, _, ody in others) / len(others)
                px, py = float(final[i, 0]), float(final[i, 1])
                d_now = ((px + dx - ax) ** 2 + (py + dy - ay) ** 2) ** 0.5
                d_rot = ((px + dy - ax) ** 2 + (py - dx - ay) ** 2) ** 0.5
                delta += d_rot - d_now
            if delta > 1.0:  # rotation would cost wirelength — veto
                rotatable[i] = False
    centers, rotated = legalize_tetris(centers, prob.size, prob.movable,
                                       prob.board, rotatable=rotatable, side=prob.side)
    final = centers - prob.coff
    return {
        "positions": {r: (float(final[i, 0]), float(final[i, 1]),
                          90.0 if i in rotated else 0.0)
                      for i, r in enumerate(prob.refs) if bool(prob.movable[i])},
        "rotated": len(rotated),
        "hpwl_before": true_hpwl(prob.pos0, prob.nets),
        "overlap_initial": float(overlap_penalty(prob.pos0 + prob.coff, prob.size, prob.side)),
        "hpwl_after": true_hpwl(final, prob.nets),
        "overlap_global": overlap_global,
        "overlap_after": float(overlap_penalty(final + prob.coff, prob.size, prob.side)),
    }
