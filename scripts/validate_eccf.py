"""ECCF gate V1: human-orientation recovery on a real board (no routing).

For every orientable part on the human placement, score all four orientations
with T2 (escape stitch over T1 fields built on the production routing grid,
part's own pads carved). The human's as-placed orientation must rank top-2
for >= 70% of parts (kill at <= 50%). See docs/PLACE-ROUTE-COUPLING.md §7.
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from spike_coupling import build_field, stitch  # noqa: E402
from tracewise.route.engine.kicad import build_problem, extract_pads, project_geometry  # noqa: E402

BOARD = sys.argv[1] if len(sys.argv) > 1 else \
    "data/benchmark-boards/mitayi-pico-d1/Mitayi-Pico-D1.kicad_pcb"


def rot(dx: float, dy: float, k: int) -> tuple[float, float]:
    for _ in range(k % 4):
        dx, dy = dy, -dx  # +90 in KiCad, verified against pcbnew
    return dx, dy


def main() -> None:
    data = extract_pads(BOARD)
    geo = project_geometry(BOARD)
    grid, nets, anchors, _, _ = build_problem(
        data, track_mm=geo["track_mm"], clearance_mm=geo["clearance_mm"])
    inflate = geo["track_mm"] / 2 + geo["clearance_mm"]

    by_ref: dict[str, list[dict]] = {}
    for p in data["pads"]:
        by_ref.setdefault(p["ref"], []).append(p)

    # orientable = 2+ pads on >=1 named net, pad spread anisotropic, not huge
    candidates = []
    for ref, pads in by_ref.items():
        named = [p for p in pads if p["net"]]
        if len(named) < 2 or len(pads) > 6:
            continue
        xs = [p["x"] for p in pads]
        ys = [p["y"] for p in pads]
        if abs((max(xs) - min(xs)) - (max(ys) - min(ys))) < 0.2:
            continue  # symmetric spread: orientation indistinct
        candidates.append((ref, pads))

    ok = total = 0
    for ref, pads in candidates:
        cx = sum(p["x"] for p in pads) / len(pads)
        cy = sum(p["y"] for p in pads) / len(pads)
        # free grid with this part's pads carved
        g = grid.cells.copy()
        for p in pads:
            layers = ([0] if p["front"] else []) + ([1] if p["back"] else [])
            for lay in layers:
                y1 = int((p["y"] - p["hh"] - inflate - grid.y0) / grid.pitch)
                y2 = int(np.ceil((p["y"] + p["hh"] + inflate - grid.y0) / grid.pitch))
                x1 = int((p["x"] - p["hw"] - inflate - grid.x0) / grid.pitch)
                x2 = int(np.ceil((p["x"] + p["hw"] + inflate - grid.x0) / grid.pitch))
                g[lay, max(0, y1):y2 + 1, max(0, x1):x2 + 1] = 0
        free = g == 0

        # T1 fields per net of this part (seeds = net's other pads)
        fields = {}
        for p in pads:
            if not p["net"] or p["net"] in fields:
                continue
            seeds = []
            for q in data["pads"]:
                if q["net"] == p["net"] and q["ref"] != ref:
                    layers = ([0] if q["front"] else []) + ([1] if q["back"] else [])
                    cell = grid.clamp_cell(*grid.to_cell(q["x"], q["y"]))
                    seeds += [(lay, *cell) for lay in layers]
            if seeds:
                fields[p["net"]] = build_field(free, seeds)
        if not fields:
            continue

        scores = []
        for k in range(4):
            s = 0.0
            for p in pads:
                if not p["net"] or p["net"] not in fields:
                    continue
                dx, dy = rot(p["x"] - cx, p["y"] - cy, k)
                cell = grid.clamp_cell(*grid.to_cell(cx + dx, cy + dy))
                lay = 0 if p["front"] else 1
                s += min(stitch(free, (lay, *cell), fields[p["net"]]), 1e6)
            scores.append((s, k))
        scores.sort()
        rank = next(i for i, (_, k) in enumerate(scores) if k == 0)
        total += 1
        if rank <= 1:
            ok += 1
        else:
            print(f"  miss: {ref} human-orientation rank {rank + 1}/4")

    rate = ok / max(total, 1)
    print(f"V1: {ok}/{total} parts recover human orientation in top-2 = {rate:.0%}")
    print("PASS (>=70%)" if rate >= 0.7 else
          ("KILL (<=50%)" if rate <= 0.5 else "WEAK (50-70%) — judgment call"))


if __name__ == "__main__":
    main()
