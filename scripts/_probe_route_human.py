"""Phase-A probe helper: route mitayi HUMAN placement clean, save board + DRC.

Mirrors place_route_measure.py step 1 (human-placement baseline) only — no
re-place — so the routed board matches the scorecard's "104 err / 74 clearance"
profile. Output dir is stable so the probe analyzer can read it.

    taskset -c 0-9 .venv/bin/python scripts/_probe_route_human.py <board_dir> <out_dir>
"""
from __future__ import annotations

import collections
import json
import shutil
import sys
import time
from pathlib import Path

from tracewise.route.bridge import run_drc, strip_routing
from tracewise.route.engine.kicad import route_board_engine

SUFFIXES = (".kicad_pcb", ".kicad_sch", ".kicad_pro", ".kicad_prl")


def main() -> None:
    bdir = Path(sys.argv[1]).resolve()
    out = Path(sys.argv[2]).resolve()
    shutil.rmtree(out, ignore_errors=True)
    out.mkdir(parents=True)
    for f in bdir.iterdir():
        if f.suffix in SUFFIXES:
            shutil.copy(f, out / f.name)
    board = next(out.glob("*.kicad_pcb"))

    t = time.time()
    strip_routing(board)
    summary = route_board_engine(board, pitch=0.1)
    report = run_drc(board)
    by = collections.Counter(v.get("type") for v in report.get("violations", []))
    errs = sum(1 for v in report.get("violations", []) if v.get("severity") == "error")
    print(f"ROUTED {board}", flush=True)
    print(f"engine summary: {summary}", flush=True)
    print(f"unconnected={len(report.get('unconnected_items', []))} errors={errs}", flush=True)
    print(f"by_type={dict(by)}", flush=True)
    print(f"elapsed={time.time() - t:.0f}s", flush=True)
    print(f"BOARD={board}", flush=True)
    print(f"DRC={board.with_suffix('.drc.json') if False else out / (board.stem + '.drc.json')}", flush=True)


if __name__ == "__main__":
    main()
