"""M3-P2 verification + strategic residual analysis: route mitayi with
gridless_rescue=True (canonical method), confirm unconnected vs the verified 48
baseline, and CLASSIFY the still-unconnected nets by pin-count / power-vs-signal
— to determine whether mitayi's remaining gap is a routing problem (signal
multi-pin, fixable by more gridless work) or a power-pour-coverage problem
(different chapter).

    taskset -c 0-9 .venv/bin/python scripts/_verify_m3p2_residual.py
"""
from __future__ import annotations

import collections
import re
import shutil
from pathlib import Path

from tracewise.route.bridge import run_drc, strip_routing
from tracewise.route.engine.kicad import build_problem, extract_pads, route_board_engine

SUFFIXES = (".kicad_pcb", ".kicad_sch", ".kicad_pro", ".kicad_prl")
BDIR = Path("data/benchmark-boards/mitayi-pico-d1").resolve()
OUT = Path("/tmp/m3p2_residual")
POWER_HINTS = ("GND", "+3V3", "+1V1", "VBUS", "VCC", "+5V", "VDD", "VSS", "PWR")


def main() -> None:
    # pad/pin counts per net (for classification)
    shutil.rmtree(OUT, ignore_errors=True)
    OUT.mkdir(parents=True)
    for f in BDIR.iterdir():
        if f.suffix in SUFFIXES:
            shutil.copy(f, OUT / f.name)
    board = next(OUT.glob("*.kicad_pcb"))
    strip_routing(board)

    data = extract_pads(board)
    pin_count: collections.Counter = collections.Counter()
    for p in data["pads"]:
        if p.get("net"):
            pin_count[p["net"]] += 1

    summary = route_board_engine(board, pitch=0.1, gridless_rescue=True)
    rep = run_drc(board)
    errs = [v for v in rep.get("violations", []) if v.get("severity") == "error"]
    by = collections.Counter(v.get("type") for v in errs)
    unc_items = len(rep.get("unconnected_items", []))

    unc_nets: set[str] = set()
    for u in rep.get("unconnected_items", []):
        for it in u.get("items", []):
            for m in re.findall(r"\[([^\]]+)\]", it.get("description", "")):
                unc_nets.add(m)

    print(f"engine: routed={summary.get('routed')}/{summary.get('nets')} vias={summary.get('vias')}", flush=True)
    print(f"[rescue=True] unconnected_items={unc_items} (baseline 48) errors={len(errs)} (baseline 89)", flush=True)
    print(f"  hole_clearance={by.get('hole_clearance',0)} (base 32) hole_to_hole={by.get('hole_to_hole',0)} (base 14) cec={by.get('copper_edge_clearance',0)}", flush=True)
    print(f"  GATE unconnected<48: {unc_items < 48}", flush=True)
    print(f"=== {len(unc_nets)} unconnected net names, classified ===", flush=True)
    power, signal = [], []
    for n in sorted(unc_nets):
        k = pin_count.get(n, 0)
        is_power = any(h in n.upper() for h in POWER_HINTS) or k >= 10
        (power if is_power else signal).append((n, k))
    print(f"  POWER / high-pin (pour-coverage class): {len(power)}", flush=True)
    for n, k in power:
        print(f"    {n}  ({k} pins)", flush=True)
    print(f"  SIGNAL multi/2-pin (routable class): {len(signal)}", flush=True)
    for n, k in signal:
        print(f"    {n}  ({k} pins)", flush=True)
    print("=== STRATEGIC READ: if residual is dominated by POWER/high-pin, mitayi's gap is "
          "pour-coverage, not routing ===", flush=True)


if __name__ == "__main__":
    main()
