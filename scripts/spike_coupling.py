#!/usr/bin/env python
"""Spike: escape-coupled cost fields — a router-grounded routability signal
for placement (docs/PLACE-ROUTE-COUPLING.md).

The mechanism under test, in three tiers:

  T1  per-net COST-TO-GO FIELDS: multi-source min-plus wavefront from a net's
      terminals over the production router's clearance-inflated occupancy grid
      (both layers, via transitions). Built ONCE per placement round, cached;
      optionally on an optimistically-coarsened grid.
  T2  ESCAPE STITCH: to score a candidate (a part at some orientation or
      position), run a tiny bounded Dijkstra from each of its pads through the
      candidate's exact local geometry; the score of a frontier cell is
      local-cost + cached-field value. Captures escape direction and sealing.
  T3  FIELD-GUIDED PSEUDO-ROUTE: sequential one-shot A* per attached net using
      the cached field as an (admissible, near-exact) heuristic, with
      reservation splats between nets. Captures corridor CAPACITY, which
      independent fields cannot see.

Ground truth: the production engine itself (route_all with rip-up, escape=12).

Scenario A — the rotation killer: two HPWL-near-neutral 180° orientations of a
6-signal header inside a connector pocket; pads face the open side vs the wall.
Scenario B — corridor capacity: five nets through a wide vs a too-narrow gap.

Run:  .venv/bin/python scripts/spike_coupling.py
"""

from __future__ import annotations

import heapq
import math
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from tracewise.route.engine.grid import Grid  # noqa: E402
from tracewise.route.engine.multi import Net, route_all  # noqa: E402

PITCH = 0.1
TRACK = 0.2
CLEAR = 0.2
INFLATE = TRACK / 2 + CLEAR  # 0.3 mm — same per-net inflation the router uses
HW = max(1, math.ceil(INFLATE / PITCH))  # 3 cells track halfwidth+clearance
VIA_HW = 6  # via radius + clearance + track halfwidth, cells (kicad.py math)
VIA_COST = 10.0
SQRT2 = math.sqrt(2.0)
DIRS8 = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
         (-1, -1, SQRT2), (-1, 1, SQRT2), (1, -1, SQRT2), (1, 1, SQRT2)]
SEAL = 1e4  # penalty when a pad cannot reach any finite-field cell


# --------------------------------------------------------------------------
# T1 — cost-to-go fields (vectorized min-plus / Bellman wavefront)
# --------------------------------------------------------------------------

def build_field(free: np.ndarray, seeds: list[tuple[int, int, int]],
                via_cost: float = VIA_COST, step: float = 1.0,
                check_every: int = 12, max_iters: int = 4000) -> np.ndarray:
    """d[l,y,x] = octile-ish distance (in FINE-cell units) to the nearest seed,
    walking only free cells, with layer flips at via-legal cells. `step` is the
    fine-cell length of one move (== coarsening factor on a coarse grid)."""
    L, H, W = free.shape
    d = np.full((L, H, W), np.inf, np.float32)
    for lay, iy, ix in seeds:
        if 0 <= iy < H and 0 <= ix < W:
            d[lay, iy, ix] = 0.0
    seed_mask = d == 0.0
    recv = free | seed_mask  # blocked non-seed cells never receive a value
    via_ok = free.all(axis=0)  # approximation of the router's via ring check
    iters = 0
    while iters < max_iters:
        snap = d
        for _ in range(check_every):
            nd = d.copy()
            for dy, dx, c in DIRS8:
                ys_d = slice(max(0, dy), H + min(0, dy))
                xs_d = slice(max(0, dx), W + min(0, dx))
                ys_s = slice(max(0, -dy), H + min(0, -dy))
                xs_s = slice(max(0, -dx), W + min(0, -dx))
                np.minimum(nd[:, ys_d, xs_d], d[:, ys_s, xs_s] + np.float32(c * step),
                           out=nd[:, ys_d, xs_d])
            np.minimum(nd[0], np.where(via_ok, nd[1] + np.float32(via_cost), np.inf),
                       out=nd[0])
            np.minimum(nd[1], np.where(via_ok, nd[0] + np.float32(via_cost), np.inf),
                       out=nd[1])
            nd[~recv] = np.inf
            nd[seed_mask] = 0.0
            d = nd
            iters += 1
        if np.array_equal(snap, d):
            break
    return d


def coarsen_free(free: np.ndarray, f: int) -> np.ndarray:
    """Optimistic pooling: a coarse cell is free if ANY fine cell in its f×f
    block is free. Optimism is safe here because the local stitch (T2) is
    exact at full resolution where the candidate geometry lives."""
    L, H, W = free.shape
    hc, wc = H // f, W // f
    return free[:, :hc * f, :wc * f].reshape(L, hc, f, wc, f).any(axis=(2, 4))


def field_at(field: np.ndarray, f: int, lay: int, iy: int, ix: int) -> float:
    if f == 1:
        return float(field[lay, iy, ix])
    cy = min(iy // f, field.shape[1] - 1)
    cx = min(ix // f, field.shape[2] - 1)
    return float(field[lay, cy, cx])


# --------------------------------------------------------------------------
# T2 — escape stitch (per-candidate, bounded local Dijkstra onto the field)
# --------------------------------------------------------------------------

def stitch(free: np.ndarray, start: tuple[int, int, int], field: np.ndarray,
           f: int = 1, radius: int = 30, via_cost: float = VIA_COST) -> float:
    """Exact local wavefront within a (2R+1)² window around the pad on the
    CANDIDATE grid, stitched onto the cached (candidate-blind) field at the
    window's EXIT RING: score = min over ring cells of (local cost + field).

    The ring formulation matters: the field is 1-Lipschitz, so taking the min
    over *interior* cells collapses to field(start) and prices none of the
    candidate's own geometry (measured — the v1 stitch did exactly that).
    Forcing the hand-off at Chebyshev distance R makes the pad pay the true
    local escape cost through its own body/halo field first. SEAL if the ring
    is unreachable (the pad is walled in — the rotation killer, detected)."""
    L, H, W = free.shape
    sl, sy, sx = start
    y1, y2 = max(0, sy - radius), min(H, sy + radius + 1)
    x1, x2 = max(0, sx - radius), min(W, sx + radius + 1)
    win = np.ascontiguousarray(free[:, y1:y2, x1:x2])
    d = build_field(win, [(sl, sy - y1, sx - x1)], via_cost=via_cost,
                    check_every=8, max_iters=12 * radius)
    yy, xx = np.meshgrid(np.arange(y1, y2), np.arange(x1, x2), indexing="ij")
    ring = np.maximum(np.abs(yy - sy), np.abs(xx - sx)) == radius
    best = math.inf
    for lay in range(L):
        sel = ring & np.isfinite(d[lay])
        if not sel.any():
            continue
        ys, xs = yy[sel], xx[sel]
        if f == 1:
            fv = field[lay, ys, xs]
        else:
            fv = field[lay, np.minimum(ys // f, field.shape[1] - 1),
                       np.minimum(xs // f, field.shape[2] - 1)]
        tot = d[lay][sel] + fv
        if tot.size:
            best = min(best, float(np.min(tot)))
    return best if math.isfinite(best) else SEAL


# --------------------------------------------------------------------------
# T3 — field-guided pseudo-route (sequential, reservation-aware, no rip-up)
# --------------------------------------------------------------------------

def pseudo_route(free: np.ndarray, start: tuple[int, int, int],
                 goals: set[tuple[int, int, int]], field: np.ndarray,
                 reserved: np.ndarray, f: int = 1, via_cost: float = VIA_COST,
                 cap: int = 40000) -> tuple[list | None, float, int]:
    """One-shot A*, h = cached field (admissible: reservations only ADD cost).
    Returns (path, geometric_cost, expansions)."""
    L, H, W = free.shape

    def h(node):
        return field_at(field, f, *node)

    g0 = 0.0
    pq = [(h(start), g0, 0.0, start)]
    came: dict = {start: None}
    gs = {start: 0.0}
    exp = 0
    while pq:
        _, g, geo, node = heapq.heappop(pq)
        if g > gs.get(node, math.inf):
            continue
        if node in goals:
            path = []
            cur = node
            while cur is not None:
                path.append(cur)
                cur = came[cur]
            return path[::-1], geo, exp
        exp += 1
        if exp > cap:
            return None, math.inf, exp
        lay, iy, ix = node
        nbrs = []
        for dy, dx, c in DIRS8:
            ny, nx = iy + dy, ix + dx
            if 0 <= ny < H and 0 <= nx < W:
                nxt = (lay, ny, nx)
                if nxt in goals or free[lay, ny, nx]:
                    nbrs.append((nxt, c))
        if free[:, iy, ix].all():
            nbrs.append(((1 - lay, iy, ix), via_cost))
        for nxt, c in nbrs:
            ng = g + c + float(reserved[nxt])
            if ng < gs.get(nxt, math.inf):
                gs[nxt] = ng
                came[nxt] = node
                heapq.heappush(pq, (ng + h(nxt), ng, geo + c, nxt))
    return None, math.inf, exp


def reserve(reserved: np.ndarray, path: list, hw: int = HW,
            amount: float = 1000.0) -> None:
    """Splat a pseudo-routed net into the shared reservation map (the same
    halfwidth the router marks); layer transitions reserve a via disc."""
    L, H, W = reserved.shape
    for (a, b) in zip(path, path[1:], strict=False):
        if a[0] != b[0]:
            iy, ix = a[1], a[2]
            reserved[:, max(0, iy - VIA_HW):iy + VIA_HW + 1,
                     max(0, ix - VIA_HW):ix + VIA_HW + 1] = amount
    for lay, iy, ix in path:
        reserved[lay, max(0, iy - hw):iy + hw + 1,
                 max(0, ix - hw):ix + hw + 1] = amount


# --------------------------------------------------------------------------
# board-building helpers (mirror kicad.build_problem conventions)
# --------------------------------------------------------------------------

def pad_rect(x: float, y: float, hw: float = 0.4, hh: float = 0.4):
    return (x - hw, y - hh, x + hw, y + hh)


def block_both(grid: Grid, rect, inflate: float = INFLATE, delta: int = 1):
    for layer in (0, 1):
        grid.block_pad(layer, *rect, inflate_mm=inflate, delta=delta)


def make_net(name: str, rects_a: list, rects_b: list, grid: Grid) -> Net:
    """Two-terminal net between two through-hole pad groups (both layers)."""
    pads, carve = [], []
    for r in rects_a + rects_b:
        cx, cy = (r[0] + r[2]) / 2, (r[1] + r[3]) / 2
        cell = grid.clamp_cell(*grid.to_cell(cx, cy))
        for layer in (0, 1):
            pads.append((layer, *cell))
            carve.append((layer, *r, INFLATE))
    return Net(name, pads, halfwidth_cells=HW, via_halfwidth_cells=VIA_HW,
               carve=carve)


def hpwl_mm(pairs: list[tuple[tuple, tuple]]) -> float:
    return sum(abs(a[0] - b[0]) + abs(a[1] - b[1]) for a, b in pairs)


def center(rect) -> tuple[float, float]:
    return ((rect[0] + rect[2]) / 2, (rect[1] + rect[3]) / 2)


def octile_lb(p: tuple[float, float], q: tuple[float, float]) -> float:
    """Obstacle-free octile lower bound between two mm points, in cells —
    the part of any score that is plain wirelength. The COUPLING signal is
    the EXCESS above this (HPWL already prices the rest)."""
    dx = abs(p[0] - q[0]) / PITCH
    dy = abs(p[1] - q[1]) / PITCH
    return (dx + dy) + (SQRT2 - 2) * min(dx, dy)


# --------------------------------------------------------------------------
# Scenario A — escape direction (the rotation killer)
# --------------------------------------------------------------------------

WALLS_A = [
    (8.0, 4.0, 20.0, 7.4),    # pocket back (north)
    (8.0, 4.0, 9.5, 13.0),    # pocket west flank
    (18.5, 4.0, 20.0, 13.0),  # pocket east flank
]
U1_C = (14.0, 10.7)           # part center inside the pocket mouth
ROW_DY = 2.6                  # pad rows at center_y ± ROW_DY (8.1 / 13.3)
N_SIG = 6
PIN_PITCH = 1.27
J2_X, J2_Y0, J2_PITCH = 35.0, 6.75, 1.5


def u1_pads(orient: str) -> tuple[list, list]:
    """(signal_rects in net order 0..5, unused_rects). 180° rotation moves the
    signal row to the other body edge AND mirrors pin order in x."""
    cx, cy = U1_C
    sig_y = cy + ROW_DY if orient == "good" else cy - ROW_DY
    oth_y = cy - ROW_DY if orient == "good" else cy + ROW_DY
    sign = 1.0 if orient == "good" else -1.0
    xs = [cx + sign * (k - (N_SIG - 1) / 2) * PIN_PITCH for k in range(N_SIG)]
    sig = [pad_rect(x, sig_y) for x in xs]
    oth = [pad_rect(x, oth_y) for x in xs]
    return sig, oth


def j2_pads() -> list:
    return [pad_rect(J2_X, J2_Y0 + k * J2_PITCH) for k in range(N_SIG)]


def grid_a(include_u1: str | None) -> Grid:
    g = Grid(x0=0, y0=0, width_mm=51.0, height_mm=21.0, pitch=PITCH, layers=2)
    for w in WALLS_A:
        block_both(g, w)
    for r in j2_pads():
        block_both(g, r)
    if include_u1:
        sig, oth = u1_pads(include_u1)
        for r in sig + oth:
            block_both(g, r)
    return g


def scenario_a() -> dict:
    print("=" * 76)
    print("Scenario A — escape direction (180° rotation, HPWL-near-neutral)")
    print("=" * 76)
    j2 = j2_pads()
    out: dict = {}

    # HPWL neutrality check (the placer's view of the two orientations)
    for orient in ("good", "bad"):
        sig, _ = u1_pads(orient)
        pairs = [(((r[0] + r[2]) / 2, (r[1] + r[3]) / 2),
                  ((q[0] + q[2]) / 2, (q[1] + q[3]) / 2))
                 for r, q in zip(sig, j2, strict=True)]
        out[f"hpwl_{orient}"] = hpwl_mm(pairs)
    print(f"HPWL  good={out['hpwl_good']:.1f}mm  bad={out['hpwl_bad']:.1f}mm  "
          f"(delta {100 * (out['hpwl_bad'] / out['hpwl_good'] - 1):+.1f}% — "
          "what HPWL-veto rotation sees)")

    # T1: cached fields — built ONCE on the base grid WITHOUT U1, reused for
    # every candidate orientation. Per net: carve its J2 pad, wavefront.
    base = grid_a(include_u1=None)
    fields_full, fields_coarse = [], []
    t_full = t_coarse = 0.0
    F = 3  # coarsening factor
    for k in range(N_SIG):
        r = j2[k]
        for layer in (0, 1):
            base.block_pad(layer, *r, inflate_mm=INFLATE, delta=-1)
        free = base.cells == 0
        cell = base.clamp_cell(*base.to_cell((r[0] + r[2]) / 2, (r[1] + r[3]) / 2))
        seeds = [(0, *cell), (1, *cell)]
        t0 = time.perf_counter()
        fields_full.append(build_field(free, seeds))
        t_full += time.perf_counter() - t0
        t0 = time.perf_counter()
        fc = coarsen_free(free, F)
        fields_coarse.append(build_field(fc, [(lay, y // F, x // F) for lay, y, x in seeds],
                                         step=float(F)))
        t_coarse += time.perf_counter() - t0
        for layer in (0, 1):
            base.block_pad(layer, *r, inflate_mm=INFLATE, delta=1)
    out["t_field_full_ms"] = 1000 * t_full / N_SIG
    out["t_field_coarse_ms"] = 1000 * t_coarse / N_SIG
    print(f"T1 field build (cached, once per round): full-res "
          f"{out['t_field_full_ms']:.0f}ms/net, coarse(f={F}) "
          f"{out['t_field_coarse_ms']:.0f}ms/net  "
          f"(grid {base.cells.shape})")

    # T2 + T3 per orientation; ground truth = production route_all
    for orient in ("good", "bad"):
        g = grid_a(include_u1=orient)
        sig, _ = u1_pads(orient)
        ideal = sum(octile_lb(center(sig[k]), center(j2[k])) for k in range(N_SIG))
        out[f"ideal_{orient}"] = ideal

        # T2 escape stitch: carve the net's own U1 pad, bounded Dijkstra
        t0 = time.perf_counter()
        s_full = s_coarse = 0.0
        sealed = 0
        for k in range(N_SIG):
            r = sig[k]
            for layer in (0, 1):
                g.block_pad(layer, *r, inflate_mm=INFLATE, delta=-1)
            free = g.cells == 0
            cell = g.clamp_cell(*g.to_cell((r[0] + r[2]) / 2, (r[1] + r[3]) / 2))
            v_full = stitch(free, (0, *cell), fields_full[k], f=1)
            v_coarse = stitch(free, (0, *cell), fields_coarse[k], f=F)
            s_full += v_full
            s_coarse += v_coarse
            sealed += v_full >= SEAL
            for layer in (0, 1):
                g.block_pad(layer, *r, inflate_mm=INFLATE, delta=1)
        t2_ms = 1000 * (time.perf_counter() - t0)
        out[f"stitch_{orient}"] = s_full
        out[f"stitch_coarse_{orient}"] = s_coarse
        out[f"t_stitch_{orient}_ms"] = t2_ms

        # T3 pseudo-route (sequential reservations, no rip-up)
        t0 = time.perf_counter()
        reserved = np.zeros(g.cells.shape, np.float32)
        p_cost, p_fail = 0.0, 0
        for k in range(N_SIG):
            r, q = sig[k], j2[k]
            for layer in (0, 1):
                g.block_pad(layer, *r, inflate_mm=INFLATE, delta=-1)
                g.block_pad(layer, *q, inflate_mm=INFLATE, delta=-1)
            free = g.cells == 0
            c_u = g.clamp_cell(*g.to_cell((r[0] + r[2]) / 2, (r[1] + r[3]) / 2))
            c_j = g.clamp_cell(*g.to_cell((q[0] + q[2]) / 2, (q[1] + q[3]) / 2))
            path, geo, _ = pseudo_route(free, (0, *c_u),
                                        {(0, *c_j), (1, *c_j)},
                                        fields_full[k], reserved)
            if path is None:
                p_fail += 1
                p_cost += SEAL
            else:
                p_cost += geo
                reserve(reserved, path)
            for layer in (0, 1):
                g.block_pad(layer, *r, inflate_mm=INFLATE, delta=1)
                g.block_pad(layer, *q, inflate_mm=INFLATE, delta=1)
        t3_ms = 1000 * (time.perf_counter() - t0)
        out[f"pseudo_{orient}"] = p_cost
        out[f"pseudo_fail_{orient}"] = p_fail
        out[f"t_pseudo_{orient}_ms"] = t3_ms

        # ground truth: the production router, production settings
        gt_grid = grid_a(include_u1=orient)
        nets = [make_net(f"N{k}", [sig[k]], [j2[k]], gt_grid) for k in range(N_SIG)]
        t0 = time.perf_counter()
        res = route_all(gt_grid, nets, escape=12, ripup_factor=8)
        t_gt = time.perf_counter() - t0
        routed = sum(1 for r_ in res.values() if r_.ok)
        copper = sum(len(r_.cells) for r_ in res.values() if r_.ok)
        out[f"gt_routed_{orient}"] = routed
        out[f"gt_copper_{orient}"] = copper
        out[f"t_gt_{orient}_s"] = t_gt

        print(f"[{orient:4s}] T2 excess={s_full - ideal:7.1f} "
              f"(coarse {s_coarse - ideal:7.1f}, sealed {sealed}, "
              f"{t2_ms:5.1f}ms/cand) | "
              f"T3 excess={p_cost - ideal:7.1f} fails={p_fail} ({t3_ms:5.1f}ms) | "
              f"GT routed {routed}/{N_SIG}, copper {copper} ({t_gt:.1f}s)")

    # the coupling signal is the EXCESS over the obstacle-free lower bound:
    # the lower bound itself is wirelength, which HPWL already prices
    ex2 = {o: out[f"stitch_{o}"] - out[f"ideal_{o}"] for o in ("good", "bad")}
    ex2c = {o: out[f"stitch_coarse_{o}"] - out[f"ideal_{o}"] for o in ("good", "bad")}
    ex3 = {o: out[f"pseudo_{o}"] - out[f"ideal_{o}"] for o in ("good", "bad")}
    ok2 = ex2["bad"] > 1.5 * max(ex2["good"], 1.0)
    ok2c = ex2c["bad"] > 1.5 * max(ex2c["good"], 1.0)
    ok3 = ex3["bad"] > 1.5 * max(ex3["good"], 1.0)
    okgt = (out["gt_routed_bad"], -out["gt_copper_bad"]) \
        < (out["gt_routed_good"], -out["gt_copper_good"])
    print(f"VERDICT A: T2 separates {ok2} (excess {ex2['good']:.0f} -> "
          f"{ex2['bad']:.0f}; coarse {ok2c}), T3 separates {ok3} "
          f"(excess {ex3['good']:.0f} -> {ex3['bad']:.0f}), "
          f"ground truth confirms damage: {okgt} "
          f"(router effort x{out['t_gt_bad_s'] / max(out['t_gt_good_s'], 1e-9):.0f})")
    out["verdict"] = bool(ok2 and ok3 and okgt)
    return out


# --------------------------------------------------------------------------
# Scenario B — corridor capacity (zero-sum competition)
# --------------------------------------------------------------------------

def scenario_b() -> dict:
    print()
    print("=" * 76)
    print("Scenario B — corridor capacity (independent fields vs pseudo-route)")
    print("=" * 76)
    n = 5
    west = [pad_rect(4.0, 7.0 + 1.5 * k) for k in range(n)]
    east = [pad_rect(26.0, 7.0 + 1.5 * k) for k in range(n)]
    out: dict = {}

    for label, gap in (("wide", (8.0, 12.0)), ("narrow", (9.5, 10.5))):
        g = Grid(x0=0, y0=0, width_mm=30.0, height_mm=20.0, pitch=PITCH, layers=2)
        block_both(g, (14.0, 0.0, 16.0, gap[0]))   # full-height slabs: the gap
        block_both(g, (14.0, gap[1], 16.0, 20.0))  # is the ONLY corridor
        for r in west + east:
            block_both(g, r)

        # T1 fields from east pads (per net), T2 independent sum, T3 pseudo
        fields = []
        for k in range(n):
            for layer in (0, 1):
                g.block_pad(layer, *east[k], inflate_mm=INFLATE, delta=-1)
            free = g.cells == 0
            c = g.clamp_cell(*g.to_cell(26.0, 7.0 + 1.5 * k))
            fields.append(build_field(free, [(0, *c), (1, *c)]))
            for layer in (0, 1):
                g.block_pad(layer, *east[k], inflate_mm=INFLATE, delta=1)

        s2 = 0.0
        reserved = np.zeros(g.cells.shape, np.float32)
        p_cost, p_fail = 0.0, 0
        t0 = time.perf_counter()
        for k in range(n):
            for layer in (0, 1):
                g.block_pad(layer, *west[k], inflate_mm=INFLATE, delta=-1)
                g.block_pad(layer, *east[k], inflate_mm=INFLATE, delta=-1)
            free = g.cells == 0
            cw = g.clamp_cell(*g.to_cell(4.0, 7.0 + 1.5 * k))
            ce = g.clamp_cell(*g.to_cell(26.0, 7.0 + 1.5 * k))
            s2 += stitch(free, (0, *cw), fields[k], f=1)
            path, geo, _ = pseudo_route(free, (0, *cw), {(0, *ce), (1, *ce)},
                                        fields[k], reserved)
            if path is None:
                p_fail += 1
                p_cost += SEAL
            else:
                p_cost += geo
                reserve(reserved, path)
            for layer in (0, 1):
                g.block_pad(layer, *west[k], inflate_mm=INFLATE, delta=1)
                g.block_pad(layer, *east[k], inflate_mm=INFLATE, delta=1)
        t23_ms = 1000 * (time.perf_counter() - t0)

        gt = Grid(x0=0, y0=0, width_mm=30.0, height_mm=20.0, pitch=PITCH, layers=2)
        block_both(gt, (14.0, 0.0, 16.0, gap[0]))
        block_both(gt, (14.0, gap[1], 16.0, 20.0))
        for r in west + east:
            block_both(gt, r)
        nets = [make_net(f"B{k}", [west[k]], [east[k]], gt) for k in range(n)]
        res = route_all(gt, nets, escape=12, ripup_factor=8)
        routed = sum(1 for r_ in res.values() if r_.ok)

        out[f"stitch_{label}"] = s2
        out[f"pseudo_{label}"] = p_cost
        out[f"pseudo_fail_{label}"] = p_fail
        out[f"gt_routed_{label}"] = routed
        print(f"[{label:6s}] T2 independent={s2:8.1f} | T3 pseudo={p_cost:9.1f} "
              f"fails={p_fail} ({t23_ms:5.1f}ms for all {n}) | "
              f"GT routed {routed}/{n}")

    blind = out["stitch_narrow"] < out["stitch_wide"] * 1.3  # T2 can't see it
    sees = out["pseudo_narrow"] > out["pseudo_wide"] * 1.5   # T3 can
    gt_drop = out["gt_routed_narrow"] < out["gt_routed_wide"]
    print(f"VERDICT B: T2 capacity-blind as predicted: {blind}; "
          f"T3 sees the capacity wall: {sees}; ground truth drops: {gt_drop}")
    out["verdict"] = bool(blind and sees and gt_drop)
    return out


if __name__ == "__main__":
    a = scenario_a()
    b = scenario_b()
    print()
    print("=" * 76)
    ok = a["verdict"] and b["verdict"]
    print(f"SPIKE RESULT: {'PASS' if ok else 'FAIL'} — escape direction "
          "separated by T2/T3 and confirmed by the production router; "
          "capacity separated by T3 only (as designed).")
    sys.exit(0 if ok else 1)
