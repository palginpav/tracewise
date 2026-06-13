"""ECCF: Escape-Coupled Cost Fields — router-grounded routability scoring.

Validated on mitayi (docs/PLACE-ROUTE-COUPLING.md): V1 human-orientation
recovery 100%, V2 Spearman rho 0.579 vs routed outcomes. T1 fields and the
T2 escape stitch mirror the production router's semantics: clearance-inflated
occupancy, net-own-pad carving, endpoint escape allowance.
"""

from __future__ import annotations

import heapq
import math
from pathlib import Path

import numpy as np

from tracewise.route.engine.kicad import build_problem, extract_pads, project_geometry

VIA_COST = 10.0
HW = 3  # track halfwidth+clearance cells
VIA_HW = 5
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


def t2_score(board: str | Path, refs: set[str], data: dict | None = None,
             geo: dict | None = None) -> float:
    """Sum of T2 escape scores over the given parts' pads (own pads carved).

    Escape-aware: within ESCAPE_CELLS of the pad, clearance-halo cells (blocked
    but copper-free) are passable — mirroring the router's escape allowance.
    Without this the window seals on dense boards and the signal saturates
    (measured: 14/18 zero deltas in the first V2 run)."""
    data = data or extract_pads(board)
    geo = geo or project_geometry(board)
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




def field_at(field: np.ndarray, f: int, lay: int, iy: int, ix: int) -> float:
    if f == 1:
        return float(field[lay, iy, ix])
    cy = min(iy // f, field.shape[1] - 1)
    cx = min(ix // f, field.shape[2] - 1)
    return float(field[lay, cy, cx])


# --------------------------------------------------------------------------
# T2 — escape stitch (per-candidate, bounded local Dijkstra onto the field)
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


def _carve(arr, pads, grid, inflate):
    for q in pads:
        for lay in ([0] if q["front"] else []) + ([1] if q["back"] else []):
            y1 = int((q["y"] - q["hh"] - inflate - grid.y0) / grid.pitch)
            y2 = int(np.ceil((q["y"] + q["hh"] + inflate - grid.y0) / grid.pitch))
            x1 = int((q["x"] - q["hw"] - inflate - grid.x0) / grid.pitch)
            x2 = int(np.ceil((q["x"] + q["hw"] + inflate - grid.x0) / grid.pitch))
            arr[lay, max(0, y1):y2 + 1, max(0, x1):x2 + 1] = 0


def t3_verify(board: str | Path, nets: set[str]) -> tuple[float, int]:
    """T3: field-guided sequential pseudo-route of the given nets with shared
    reservations — prices zero-sum corridor capacity that per-part T2 cannot
    see (the spike's scenario B; the round-1 integration lesson). Returns
    (total geometric cost, failed pad connections)."""
    data = extract_pads(board)
    geo = project_geometry(board)
    grid, _, _ = build_problem(data, track_mm=geo["track_mm"],
                               clearance_mm=geo["clearance_mm"])
    inflate = geo["track_mm"] / 2 + geo["clearance_mm"]
    reserved = np.zeros_like(grid.cells, dtype=np.float64)
    total, fails = 0.0, 0
    for net in sorted(nets):
        net_pads = [q for q in data["pads"] if q["net"] == net]
        if len(net_pads) < 2 or len(net_pads) > 10:
            continue  # pour-class nets (GND etc.) connect by zone, not corridor
        free_net = grid.cells.copy()
        _carve(free_net, net_pads, grid, inflate)
        free = free_net == 0
        halo_ok = grid.hard == 0
        cells = []
        for q in net_pads:
            cell = grid.clamp_cell(*grid.to_cell(q["x"], q["y"]))
            for lay in ([0] if q["front"] else []) + ([1] if q["back"] else []):
                cells.append((lay, *cell))
        tree = {cells[0]}
        for pad in cells[1:]:
            if pad in tree:
                continue
            # escape-aware near the pad (mirrors the router and T2)
            esc = np.zeros_like(free)
            cy, cx = pad[1], pad[2]
            y1e, y2e = max(0, cy - 12), cy + 13
            x1e, x2e = max(0, cx - 12), cx + 13
            esc[:, y1e:y2e, x1e:x2e] = halo_ok[:, y1e:y2e, x1e:x2e]
            free_esc = free | esc
            field = build_field(free_esc, list(tree))
            path, cost, _ = pseudo_route(free_esc, pad, tree, field, reserved,
                                         cap=8000)
            if path is None:
                fails += 1
                total += SEAL
                continue
            total += cost
            tree.update(path)
            for (a, b) in zip(path, path[1:], strict=False):
                if a[0] != b[0]:
                    iy, ix = a[1], a[2]
                    reserved[:, max(0, iy - VIA_HW):iy + VIA_HW + 1,
                             max(0, ix - VIA_HW):ix + VIA_HW + 1] = 1000.0
            for lay, iy, ix in path:
                reserved[lay, max(0, iy - HW):iy + HW + 1,
                         max(0, ix - HW):ix + HW + 1] = 1000.0
    return total, fails


def patch_data(data: dict, ref: str, nx: float, ny: float, rot: float,
               flip: bool = False) -> dict:
    """Candidate evaluation without board writes: shift, optionally rotate 90
    degrees about the pad centroid, and optionally FLIP to the other copper
    side (front<->back) — the "center of the storm" relief move for passives:
    a flipped pad frees its old-layer corridor and blocks the opposite one."""
    import copy

    out = copy.deepcopy(data)
    pads = [p for p in out["pads"] if p["ref"] == ref]
    if not pads:
        return out
    cx = sum(p["x"] for p in pads) / len(pads)
    cy = sum(p["y"] for p in pads) / len(pads)
    for p in pads:
        dx, dy = p["x"] - cx, p["y"] - cy
        if rot:
            dx, dy = dy, -dx  # +90 KiCad, verified
            p["hw"], p["hh"] = p["hh"], p["hw"]
        p["x"], p["y"] = nx + dx, ny + dy
        if flip:  # SMD copper moves to the opposite layer
            p["front"], p["back"] = p["back"], p["front"]
    return out


def back_free_fraction(board: str | Path) -> float:
    """Fraction of the board area NOT covered by back-side (B.Cu) copper pours
    — the precondition for storm-flips / back-side relief to pay (measured:
    flips help on zuluscsi's free back, inert on mitayi's poured back). Uses
    zone bounding boxes on B.Cu vs board area; pour-aware and cheap (no route).
    1.0 = empty back, 0.0 = fully poured."""
    from tracewise.sexpr import parse_file

    try:
        root = parse_file(board)
    except (OSError, ValueError):
        return 1.0
    # approximate board area from the union of all zone vertex extents (cheap;
    # avoids a kicad-cli Edge.Cuts query).
    xs, ys = [], []
    for z in root.find_all("zone"):
        for pt in z.find_all("xy"):
            try:
                xs.append(float(pt.arg(1)))
                ys.append(float(pt.arg(2)))
            except (TypeError, ValueError):
                pass
    if not xs or not ys:
        return 1.0
    bw, bh = max(xs) - min(xs), max(ys) - min(ys)
    board_area = max(bw * bh, 1e-6)
    pour = 0.0
    for z in root.find_all("zone"):
        layers = " ".join((ly.arg() or "") for ly in z.find_all("layer"))
        if "B.Cu" not in layers:
            continue
        zxs, zys = [], []
        for pt in z.find_all("xy"):
            try:
                zxs.append(float(pt.arg(1)))
                zys.append(float(pt.arg(2)))
            except (TypeError, ValueError):
                pass
        if zxs and zys:
            pour += (max(zxs) - min(zxs)) * (max(zys) - min(zys))
    return max(0.0, 1.0 - min(pour / board_area, 1.0))


def rank_by_storm(parts: list, hot_points: list, radius: float = 4.0) -> list:
    """Rank parts by how many congestion hot-points (failing-net pads + DRC
    violation sites) sit within `radius` mm of the part centre — the literal
    'center of the storm'. Returns (count, part) pairs, busiest first, count>0
    only. Pure geometry: unit-testable without KiCad."""
    scored = []
    r2 = radius * radius
    for fp in parts:
        n = sum(1 for hx, hy in hot_points
                if (fp["x"] - hx) ** 2 + (fp["y"] - hy) ** 2 <= r2)
        if n:
            scored.append((n, fp))
    scored.sort(key=lambda s: -s[0])
    return scored
