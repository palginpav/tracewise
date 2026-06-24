"""b4_route_measure.py — B4 error-axis attack: measure fix impact on all 3 benchmark boards.

Routes mitayi, zuluscsi, and rp2040 with the B4 fixes applied:
  Fix 1 (kicad.py): PAD_SCRIPT now exports bb_center; build_problem uses
          bb_center (not pad position) as the obstacle rect center.
          Targets: castellated-pad shorts on ZuluSCSI (~30/40) and RP2040 (~3/23).
  Fix 2 (astar.py): Via placed at goal cell now runs via_hard_ok() (hard copper
          check) even though it's the net's own pad — prevents via copper
          overlapping adjacent-net copper on fine-pitch parts.
          Targets: shorting_items/hole_clearance on RP2040/mitayi.

B3 baseline (canonical):
  mitayi:    48 unconnected / 88  errors
  zuluscsi:  15 unconnected / 149 errors
  rp2040:    33 unconnected / 152 errors

Usage:
    .venv/bin/python scripts/b4_route_measure.py [--board mitayi|zuluscsi|rp2040]

If --board is not given, all three boards are run sequentially.
Runtime budget: 600s per board hard-abort guard.
"""
from __future__ import annotations

import argparse
import collections
import json
import resource
import shutil
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "src"))

from tracewise.route.bridge import run_drc, strip_routing
from tracewise.route.engine.kicad import route_board_engine

BOARDS = {
    "mitayi": {
        "dir": ROOT / "data/benchmark-boards/mitayi-pico-d1",
        "pcb": "Mitayi-Pico-D1.kicad_pcb",
        "b3_unc": 48,
        "b3_err": 88,
    },
    "zuluscsi": {
        "dir": ROOT / "data/benchmark-boards/zuluscsi-pico-oshw",
        "pcb": "ZuluSCSI-Pico-OSHW.kicad_pcb",
        "b3_unc": 15,
        "b3_err": 149,
    },
    "rp2040": {
        "dir": ROOT / "data/benchmark-boards/rp2040-dev-board",
        "pcb": "RP2040 Dev Board.kicad_pcb",
        "b3_unc": 33,
        "b3_err": 152,
    },
}

SUFFIXES = (".kicad_pcb", ".kicad_sch", ".kicad_pro", ".kicad_prl")
OUT_DIR = ROOT / ".b4_scorecard"
TIME_LIMIT_S = 600
RSS_HARD_FAIL_GB = 2.0

def _rss_gb() -> float:
    ru = resource.getrusage(resource.RUSAGE_SELF)
    return ru.ru_maxrss / 1e6  # Linux: KB -> GB


def route_one(name: str, cfg: dict) -> dict:
    out = OUT_DIR / f"{name}_b4"
    shutil.rmtree(out, ignore_errors=True)
    out.mkdir(parents=True)
    src_dir = cfg["dir"]
    for f in src_dir.iterdir():
        if f.suffix in SUFFIXES:
            shutil.copy(f, out / f.name)
    board = out / cfg["pcb"]

    print(f"\n{'='*60}", flush=True)
    print(f"  ROUTING {name} ({board.name})", flush=True)
    print(f"  B3 baseline: {cfg['b3_unc']} unconnected / {cfg['b3_err']} errors", flush=True)
    print(f"{'='*60}", flush=True)

    rss_start = _rss_gb()
    print(f"  [RSS] {rss_start:.3f}GB at start", flush=True)

    t = time.time()
    strip_routing(board)
    strip_t = time.time() - t
    print(f"  strip_routing: {strip_t:.1f}s", flush=True)

    rss_after_strip = _rss_gb()
    if rss_after_strip > RSS_HARD_FAIL_GB:
        print(f"ABORT: RSS {rss_after_strip:.2f}GB > {RSS_HARD_FAIL_GB}GB after strip", flush=True)
        return {"board": name, "status": "rss_abort", "rss_gb": rss_after_strip}

    t2 = time.time()
    try:
        summary = route_board_engine(board, pitch=0.1)
    except Exception as e:
        elapsed = time.time() - t2
        print(f"  route_board_engine FAILED after {elapsed:.1f}s: {e}", flush=True)
        return {"board": name, "status": "route_error", "error": str(e),
                "elapsed_s": elapsed}

    route_t = time.time() - t2
    rss_after_route = _rss_gb()
    print(f"  route_board_engine: {route_t:.1f}s  RSS={rss_after_route:.3f}GB", flush=True)
    print(f"  engine summary: {summary}", flush=True)

    if rss_after_route > RSS_HARD_FAIL_GB:
        print(f"ABORT: RSS {rss_after_route:.2f}GB > {RSS_HARD_FAIL_GB}GB after route", flush=True)
        return {"board": name, "status": "rss_abort", "rss_gb": rss_after_route}

    if route_t > TIME_LIMIT_S:
        print(f"ABORT: route took {route_t:.1f}s > {TIME_LIMIT_S}s limit", flush=True)
        return {"board": name, "status": "time_abort", "elapsed_s": route_t}

    t3 = time.time()
    report = run_drc(board)
    drc_t = time.time() - t3
    print(f"  run_drc: {drc_t:.1f}s", flush=True)

    # Save DRC JSON next to board
    drc_path = out / (board.stem + ".drc.json")
    drc_path.write_text(json.dumps(report, indent=2))

    unc = len(report.get("unconnected_items", []))
    errs = sum(1 for v in report.get("violations", []) if v.get("severity") == "error")
    by_type = dict(collections.Counter(
        v.get("type") for v in report.get("violations", [])
        if v.get("severity") == "error"
    ))

    delta_unc = unc - cfg["b3_unc"]
    delta_err = errs - cfg["b3_err"]

    print(f"  RESULT: unconnected={unc} ({delta_unc:+d} vs B3)  errors={errs} ({delta_err:+d} vs B3)", flush=True)
    print(f"  by_type: {by_type}", flush=True)
    print(f"  total elapsed: {time.time()-t:.1f}s", flush=True)

    connectivity_ok = (unc <= cfg["b3_unc"])
    verdict = "PASS" if connectivity_ok else "CONNECTIVITY_REGRESSED"
    print(f"  [{verdict}] connectivity: {unc} vs B3 {cfg['b3_unc']}", flush=True)

    return {
        "board": name,
        "status": "done",
        "b3_unc": cfg["b3_unc"],
        "b3_err": cfg["b3_err"],
        "b4_unc": unc,
        "b4_err": errs,
        "delta_unc": delta_unc,
        "delta_err": delta_err,
        "by_type": by_type,
        "connectivity_ok": connectivity_ok,
        "elapsed_route_s": route_t,
        "elapsed_total_s": time.time() - t,
        "rss_gb": rss_after_route,
        "drc_json": str(drc_path),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--board", choices=list(BOARDS.keys()),
                        help="Route only this board (default: all)")
    args = parser.parse_args()

    OUT_DIR.mkdir(parents=True, exist_ok=True)

    if args.board:
        names = [args.board]
    else:
        names = list(BOARDS.keys())

    results = []
    for name in names:
        r = route_one(name, BOARDS[name])
        results.append(r)
        summary_path = OUT_DIR / "b4_results.json"
        summary_path.write_text(json.dumps(results, indent=2))

    print("\n" + "="*60, flush=True)
    print("B4 SUMMARY", flush=True)
    print("="*60, flush=True)
    for r in results:
        if r["status"] == "done":
            flag = "OK" if r["connectivity_ok"] else "REGRESSED"
            print(f"  {r['board']:12s}: unc {r['b3_unc']}->{r['b4_unc']} ({r['delta_unc']:+d})  "
                  f"err {r['b3_err']}->{r['b4_err']} ({r['delta_err']:+d})  [{flag}]", flush=True)
        else:
            print(f"  {r['board']:12s}: {r['status']}", flush=True)
    print(f"\nResults: {OUT_DIR / 'b4_results.json'}", flush=True)


if __name__ == "__main__":
    main()
