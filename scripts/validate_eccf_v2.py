"""ECCF gate V2: rank correlation on a real board.

20 random single-part nudges of the human placement; Spearman rho between the
ECCF (T2) score delta and the routed-unconnected delta must be >= 0.5.
Each trial runs in its own subprocess (fresh state; an in-process corruption
after ~15 route rounds is on record as an open engine bug).
"""

from __future__ import annotations

import random
import shutil
import subprocess
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from spike_coupling import build_field, stitch  # noqa: E402
from tracewise.place.extract import apply_positions  # noqa: E402
from tracewise.place.extract import extract as place_extract  # noqa: E402
from tracewise.route.bridge import drc_summary, run_drc, strip_routing  # noqa: E402
from tracewise.route.engine.kicad import (  # noqa: E402
    build_problem,
    extract_pads,
    project_geometry,
    route_board_engine,
)

SRC = Path("data/benchmark-boards/mitayi-pico-d1")
WORK = Path.home() / ".cache" / "tracewise" / "eccf-v2"


def t2_score(board: str | Path, refs: set[str]) -> float:
    """Sum of T2 escape scores over the given parts' pads (own pads carved)."""
    data = extract_pads(board)
    geo = project_geometry(board)
    grid, _, _ = build_problem(data, track_mm=geo["track_mm"],
                               clearance_mm=geo["clearance_mm"])
    inflate = geo["track_mm"] / 2 + geo["clearance_mm"]
    total = 0.0
    for ref in refs:
        pads = [p for p in data["pads"] if p["ref"] == ref]
        g = grid.cells.copy()
        for p in pads:
            for lay in ([0] if p["front"] else []) + ([1] if p["back"] else []):
                y1 = int((p["y"] - p["hh"] - inflate - grid.y0) / grid.pitch)
                y2 = int(np.ceil((p["y"] + p["hh"] + inflate - grid.y0) / grid.pitch))
                x1 = int((p["x"] - p["hw"] - inflate - grid.x0) / grid.pitch)
                x2 = int(np.ceil((p["x"] + p["hw"] + inflate - grid.x0) / grid.pitch))
                g[lay, max(0, y1):y2 + 1, max(0, x1):x2 + 1] = 0
        free = g == 0
        for p in pads:
            if not p["net"]:
                continue
            seeds = []
            for q in data["pads"]:
                if q["net"] == p["net"] and q["ref"] != ref:
                    cell = grid.clamp_cell(*grid.to_cell(q["x"], q["y"]))
                    seeds += [(lay, *cell) for lay in
                              ([0] if q["front"] else []) + ([1] if q["back"] else [])]
            if not seeds:
                continue
            field = build_field(free, seeds)
            cell = grid.clamp_cell(*grid.to_cell(p["x"], p["y"]))
            lay = 0 if p["front"] else 1
            total += min(stitch(free, (lay, *cell), field), 1e6)
    return total


def fresh_board() -> Path:
    WORK.mkdir(parents=True, exist_ok=True)
    base = WORK / "base.kicad_pcb"
    shutil.copy(SRC / "Mitayi-Pico-D1.kicad_pcb", base)
    shutil.copy(SRC / "Mitayi-Pico-D1.kicad_pro", WORK / "base.kicad_pro")
    strip_routing(base)
    return base


def nth_move(base: Path, n: int):
    """Deterministic replay of the RNG sequence to trial n's move."""
    random.seed(11)
    pdata = place_extract(base)
    small = [f for f in pdata["footprints"]
             if not f["locked"] and len(f["pads"]) <= 4 and f["pads"]]
    bd_x = (121.45, 172.55)
    bd_y = (78.75, 99.85)
    fp = nx = ny = None
    for _ in range(n + 1):
        fp = random.choice(small)
        nx = min(max(fp["x"] + random.uniform(-4, 4), bd_x[0] + 2), bd_x[1] - 2)
        ny = min(max(fp["y"] + random.uniform(-4, 4), bd_y[0] + 2), bd_y[1] - 2)
    return fp["ref"], nx, ny


def one_trial(n: int) -> None:
    base = fresh_board()
    ref, nx, ny = nth_move(base, n)
    s_before = t2_score(base, {ref})
    apply_positions(base, {ref: (nx, ny, 0.0)})
    s_after = t2_score(base, {ref})
    route_board_engine(base)
    d = drc_summary(run_drc(base))["unconnected"]
    print(f"RESULT {ref} {s_after - s_before:.2f} {d}")


def baseline() -> None:
    base = fresh_board()
    route_board_engine(base)
    print(f"RESULT baseline 0 {drc_summary(run_drc(base))['unconnected']}")


def spearman(a: list[float], b: list[float]) -> float:
    def ranks(v):
        order = sorted(range(len(v)), key=lambda i: v[i])
        r = [0.0] * len(v)
        i = 0
        while i < len(order):
            j = i
            while j + 1 < len(order) and v[order[j + 1]] == v[order[i]]:
                j += 1
            avg = (i + j) / 2 + 1
            for k in range(i, j + 1):
                r[order[k]] = avg
            i = j + 1
        return r
    ra, rb = ranks(a), ranks(b)
    ma, mb = sum(ra) / len(ra), sum(rb) / len(rb)
    num = sum((x - ma) * (y - mb) for x, y in zip(ra, rb, strict=True))
    da = sum((x - ma) ** 2 for x in ra) ** 0.5
    db = sum((y - mb) ** 2 for y in rb) ** 0.5
    return num / (da * db) if da and db else 0.0


def main() -> None:
    py = sys.executable
    me = str(Path(__file__).resolve())
    out = subprocess.run([py, me, "--baseline"], capture_output=True, text=True)
    d0 = int(out.stdout.strip().split()[-1])
    print(f"baseline unconnected: {d0}", flush=True)
    pairs = []
    for n in range(20):
        r = subprocess.run([py, me, "--trial", str(n)], capture_output=True, text=True)
        lines = [ln for ln in r.stdout.splitlines() if ln.startswith("RESULT")]
        if not lines:
            err = r.stderr.strip().splitlines()[-1][:90] if r.stderr.strip() else "?"
            print(f"  {n:2d} CRASHED: {err}", flush=True)
            continue
        _, ref, ed, d = lines[0].split()
        pairs.append((float(ed), int(d) - d0))
        print(f"  {n:2d} {ref:>6} eccf_delta={float(ed):9.1f} "
              f"unconn_delta={int(d) - d0:+d}", flush=True)
    rho = spearman([p[0] for p in pairs], [p[1] for p in pairs])
    print(f"V2: Spearman rho = {rho:.3f}  ({len(pairs)} valid trials)")
    print("PASS (>=0.5)" if rho >= 0.5 else "FAIL (<0.5)")


if __name__ == "__main__":
    if "--baseline" in sys.argv:
        baseline()
    elif "--trial" in sys.argv:
        one_trial(int(sys.argv[sys.argv.index("--trial") + 1]))
    else:
        main()
