"""Measure placement quality by ROUTING-IN-THE-LOOP — the only metric that counts
(local placement scores mislead, per docs/PLAN.md). Route the board's current
(human) placement as a baseline, then re-place from scratch with the analytical
placer, route again, and report unconnected + errors for both.

    .venv/bin/python scripts/place_route_measure.py <board_dir> [iters]
"""

from __future__ import annotations

import re
import shutil
import sys
import time
from collections import Counter
from pathlib import Path

from tracewise.place.core import build_problem, optimize
from tracewise.place.extract import apply_positions, extract
from tracewise.route.bridge import drc_summary, run_drc, strip_routing
from tracewise.route.engine.kicad import route_board_engine

SUFFIXES = (".kicad_pcb", ".kicad_sch", ".kicad_pro", ".kicad_prl")


def measure(board: Path, tag: str) -> None:
    report = run_drc(board)
    unc: Counter = Counter()
    for u in report.get("unconnected_items", []):
        names = {
            m.group(1)
            for i in u["items"]
            if (m := re.search(r"\[([^\]]+)\]", i["description"]))
        }
        unc[tuple(sorted(names))] += 1
    drc = drc_summary(report)
    print(f"[{tag}] unc={sum(unc.values())} err={drc['by_severity'].get('error', 0)}",
          flush=True)


def main() -> None:
    bdir = Path(sys.argv[1])
    iters = int(sys.argv[2]) if len(sys.argv) > 2 else 400

    work = Path(f"/tmp/pr_{bdir.name}")
    shutil.rmtree(work, ignore_errors=True)
    work.mkdir(parents=True)
    for f in bdir.iterdir():
        if f.suffix in SUFFIXES:
            shutil.copy(f, work / f.name)
    board = next(work.glob("*.kicad_pcb"))

    # 1) baseline: route the human placement.
    strip_routing(board)
    route_board_engine(board)
    measure(board, "human-placement")

    # 2) re-place from scratch with the analytical placer, then route.
    prob = build_problem(extract(board))
    t = time.time()
    r = optimize(prob, iters=iters)
    apply_positions(board, r["positions"])
    print(
        f"placer: {time.time() - t:.0f}s "
        f"hpwl {r['hpwl_before']:.1f}->{r['hpwl_after']:.1f} "
        f"overlap_after={r['overlap_after']:.1f}",
        flush=True,
    )
    strip_routing(board)
    route_board_engine(board)
    measure(board, "placer-placement")
    print("PR DONE", flush=True)


if __name__ == "__main__":
    main()
