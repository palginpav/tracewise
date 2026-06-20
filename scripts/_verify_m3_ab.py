"""M3-P1 controlled A/B: route mitayi HUMAN placement with gridless_rescue OFF
vs ON, SAME canonical method (route_board_engine defaults + run_drc,
severity==error), and DIFF the result. Answers the only questions that matter:
  - does rescue connect QSPI nets the grid leaves unconnected (unconnected drops)?
  - does it introduce NEW via hole_clearance / hole_to_hole errors?

    taskset -c 0-9 .venv/bin/python scripts/_verify_m3_ab.py
"""
from __future__ import annotations

import collections
import re
import shutil
from pathlib import Path

from tracewise.route.bridge import run_drc, strip_routing
from tracewise.route.engine.kicad import route_board_engine

SUFFIXES = (".kicad_pcb", ".kicad_sch", ".kicad_pro", ".kicad_prl")
BDIR = Path("data/benchmark-boards/mitayi-pico-d1").resolve()
QSPI = ("/QSPI_SD0", "/QSPI_SD1", "/QSPI_SD2", "/QSPI_SD3", "/QSPI_SCLK")


def _unconnected_nets(report: dict) -> set[str]:
    names = set()
    for u in report.get("unconnected_items", []):
        for it in u.get("items", []):
            for m in re.findall(r"\[([^\]]+)\]", it.get("description", "")):
                names.add(m)
    return names


def _measure(tag: str, rescue: bool, out: Path) -> dict:
    shutil.rmtree(out, ignore_errors=True)
    out.mkdir(parents=True)
    for f in BDIR.iterdir():
        if f.suffix in SUFFIXES:
            shutil.copy(f, out / f.name)
    board = next(out.glob("*.kicad_pcb"))
    strip_routing(board)
    route_board_engine(board, pitch=0.1, gridless_rescue=rescue)
    rep = run_drc(board)
    errs = [v for v in rep.get("violations", []) if v.get("severity") == "error"]
    by = collections.Counter(v.get("type") for v in errs)
    unc_items = len(rep.get("unconnected_items", []))
    unc_nets = _unconnected_nets(rep)
    res = {
        "unc_items": unc_items,
        "errors": len(errs),
        "by_type": dict(by),
        "qspi_unconnected": sorted(n for n in unc_nets if n in QSPI),
        "hole_clearance": by.get("hole_clearance", 0),
        "hole_to_hole": by.get("hole_to_hole", 0),
        "copper_edge_clearance": by.get("copper_edge_clearance", 0),
    }
    print(f"[{tag}] unc_items={unc_items} errors={len(errs)} "
          f"hole_clr={res['hole_clearance']} hole2hole={res['hole_to_hole']} "
          f"cec={res['copper_edge_clearance']}", flush=True)
    print(f"[{tag}] QSPI still unconnected: {res['qspi_unconnected']}", flush=True)
    print(f"[{tag}] by_type={res['by_type']}", flush=True)
    return res


def main() -> None:
    off = _measure("GRID-ONLY (rescue=False)", False, Path("/tmp/m3ab_off"))
    on = _measure("M3 RESCUE (rescue=True)", True, Path("/tmp/m3ab_on"))
    print("=== M3-P1 A/B DIFF ===", flush=True)
    print(f"  unconnected_items: {off['unc_items']} -> {on['unc_items']} "
          f"(delta {on['unc_items'] - off['unc_items']})", flush=True)
    qspi_connected = sorted(set(off["qspi_unconnected"]) - set(on["qspi_unconnected"]))
    print(f"  QSPI nets connected by rescue: {qspi_connected}", flush=True)
    print(f"  QSPI still unconnected after rescue: {on['qspi_unconnected']}", flush=True)
    print(f"  errors: {off['errors']} -> {on['errors']} (delta {on['errors'] - off['errors']})", flush=True)
    print(f"  hole_clearance: {off['hole_clearance']} -> {on['hole_clearance']} "
          f"(NEW {on['hole_clearance'] - off['hole_clearance']})", flush=True)
    print(f"  hole_to_hole: {off['hole_to_hole']} -> {on['hole_to_hole']} "
          f"(NEW {on['hole_to_hole'] - off['hole_to_hole']})", flush=True)
    real_gain = on["unc_items"] < off["unc_items"]
    no_hole_regression = (on["hole_clearance"] <= off["hole_clearance"]
                          and on["hole_to_hole"] <= off["hole_to_hole"])
    print(f"  CONNECTIVITY GAIN (unc lower than grid-only): {real_gain}", flush=True)
    print(f"  NO via-hole regression: {no_hole_regression}", flush=True)
    print(f"  M3-P1 VERDICT: {'PASS' if real_gain and no_hole_regression else 'NOT MET'}", flush=True)


if __name__ == "__main__":
    main()
