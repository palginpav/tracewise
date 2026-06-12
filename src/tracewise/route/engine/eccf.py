"""ECCF: Escape-Coupled Cost Fields — router-grounded routability scoring.

Validated on mitayi (docs/PLACE-ROUTE-COUPLING.md): V1 human-orientation
recovery 100%, V2 Spearman rho 0.579 vs routed outcomes. T1 fields and the
T2 escape stitch mirror the production router's semantics: clearance-inflated
occupancy, net-own-pad carving, endpoint escape allowance.
"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np

from tracewise.route.engine.kicad import build_problem, extract_pads, project_geometry

VIA_COST = 10.0
SEAL = 1e4
DIRS8 = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
         (-1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)),
         (1, -1, math.sqrt(2)), (1, 1, math.sqrt(2))]


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


ESCAPE_CELLS = 12  # mirror the router's endpoint escape allowance


def t2_score(board: str | Path, refs: set[str]) -> float:
    """Sum of T2 escape scores over the given parts' pads (own pads carved).

    Escape-aware: within ESCAPE_CELLS of the pad, clearance-halo cells (blocked
    but copper-free) are passable — mirroring the router's escape allowance.
    Without this the window seals on dense boards and the signal saturates
    (measured: 14/18 zero deltas in the first V2 run)."""
    data = extract_pads(board)
    geo = project_geometry(board)
    grid, _, _ = build_problem(data, track_mm=geo["track_mm"],
                               clearance_mm=geo["clearance_mm"])
    inflate = geo["track_mm"] / 2 + geo["clearance_mm"]
    total = 0.0
    for ref in refs:
        pads = [p for p in data["pads"] if p["ref"] == ref]
        g = grid.cells.copy()
        hard = grid.hard.copy()
        for p in pads:
            for lay in ([0] if p["front"] else []) + ([1] if p["back"] else []):
                y1 = int((p["y"] - p["hh"] - inflate - grid.y0) / grid.pitch)
                y2 = int(np.ceil((p["y"] + p["hh"] + inflate - grid.y0) / grid.pitch))
                x1 = int((p["x"] - p["hw"] - inflate - grid.x0) / grid.pitch)
                x2 = int(np.ceil((p["x"] + p["hw"] + inflate - grid.x0) / grid.pitch))
                g[lay, max(0, y1):y2 + 1, max(0, x1):x2 + 1] = 0
                hard[lay, max(0, y1):y2 + 1, max(0, x1):x2 + 1] = 0
        free = g == 0
        halo_ok = hard == 0
        for p in pads:
            if not p["net"]:
                continue
            seeds = []
            net_pads = [q for q in data["pads"] if q["net"] == p["net"]]
            # the net's OWN pads must be carved before field build — seeds sit
            # inside their own halos otherwise and the field saturates to inf
            # (measured: every stitch returned SEAL on the real board)
            free_net = g.copy()
            for q in net_pads:
                for lay in ([0] if q["front"] else []) + ([1] if q["back"] else []):
                    qy1 = int((q["y"] - q["hh"] - inflate - grid.y0) / grid.pitch)
                    qy2 = int(np.ceil((q["y"] + q["hh"] + inflate - grid.y0) / grid.pitch))
                    qx1 = int((q["x"] - q["hw"] - inflate - grid.x0) / grid.pitch)
                    qx2 = int(np.ceil((q["x"] + q["hw"] + inflate - grid.x0) / grid.pitch))
                    free_net[lay, max(0, qy1):qy2 + 1, max(0, qx1):qx2 + 1] = 0
            free_n = free_net == 0
            for q in net_pads:
                if q["ref"] == ref:
                    continue
                cell = grid.clamp_cell(*grid.to_cell(q["x"], q["y"]))
                seeds += [(lay, *cell) for lay in
                          ([0] if q["front"] else []) + ([1] if q["back"] else [])]
            if not seeds:
                continue
            field = build_field(free_n, seeds)
            cell = grid.clamp_cell(*grid.to_cell(p["x"], p["y"]))
            lay = 0 if p["front"] else 1
            # escape-aware window passability around this pad
            cy, cx = cell
            esc = np.zeros_like(free)
            y1e, y2e = max(0, cy - ESCAPE_CELLS), cy + ESCAPE_CELLS + 1
            x1e, x2e = max(0, cx - ESCAPE_CELLS), cx + ESCAPE_CELLS + 1
            esc[:, y1e:y2e, x1e:x2e] = halo_ok[:, y1e:y2e, x1e:x2e]
            total += min(stitch(free_n | esc, (lay, *cell), field), 1e6)
    return total


