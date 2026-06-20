"""M3-P1.1 focused test: route ONLY the two nets the grid leaves unconnected
(/QSPI_SCLK, /QSPI_SD2) GRIDLESS-FIRST (gridless_negotiate, 2-layer via-capable),
then grid the rest. Compare to the verified grid-only baseline 48/89.
Smaller subset than all-5-QSPI => faster, decisive test of the gridless-first
strategy hypothesis.

    taskset -c 0-9 .venv/bin/python scripts/_verify_m3p11.py
"""
from __future__ import annotations

import collections
import re
import time
from pathlib import Path
import shutil

from tracewise.route.bridge import run_drc, strip_routing
from tracewise.route.engine.kicad import route_board_engine

SUFFIXES = (".kicad_pcb", ".kicad_sch", ".kicad_pro", ".kicad_prl")
BDIR = Path("data/benchmark-boards/mitayi-pico-d1").resolve()
SUBSET = {"/QSPI_SCLK", "/QSPI_SD2"}
QSPI = ("/QSPI_SD0", "/QSPI_SD1", "/QSPI_SD2", "/QSPI_SD3", "/QSPI_SCLK")


def _unc_nets(rep: dict) -> set[str]:
    s = set()
    for u in rep.get("unconnected_items", []):
        for it in u.get("items", []):
            for m in re.findall(r"\[([^\]]+)\]", it.get("description", "")):
                s.add(m)
    return s


def main() -> None:
    out = Path("/tmp/m3p11_B")
    shutil.rmtree(out, ignore_errors=True)
    out.mkdir(parents=True)
    for f in BDIR.iterdir():
        if f.suffix in SUFFIXES:
            shutil.copy(f, out / f.name)
    board = next(out.glob("*.kicad_pcb"))
    strip_routing(board)
    print(f"gridless-FIRST subset (2-layer capable): {sorted(SUBSET)}", flush=True)
    t = time.time()
    summary = route_board_engine(board, pitch=0.1, gridless_nets=set(SUBSET), gridless_negotiate=True)
    print(f"route done in {time.time() - t:.0f}s: routed={summary.get('routed')}/{summary.get('nets')} "
          f"vias={summary.get('vias')}", flush=True)
    rep = run_drc(board)
    errs = [v for v in rep.get("violations", []) if v.get("severity") == "error"]
    by = collections.Counter(v.get("type") for v in errs)
    unc = len(rep.get("unconnected_items", []))
    qspi_unc = sorted(n for n in _unc_nets(rep) if n in QSPI)
    print(f"[B gridless-first] unc_items={unc} errors={len(errs)} by_type={dict(by)}", flush=True)
    print(f"[B] QSPI still unconnected: {qspi_unc}", flush=True)
    print("=== vs verified grid-only baseline: 48 unc / 89 err, QSPI unconnected [/QSPI_SCLK,/QSPI_SD2] ===", flush=True)
    connected = sorted(SUBSET - set(qspi_unc))
    print(f"  subset nets now CONNECTED: {connected}", flush=True)
    print(f"  unconnected: 48 -> {unc} (gain if < 48: {unc < 48})", flush=True)
    print(f"  hole_clearance={by.get('hole_clearance',0)} (baseline 32) "
          f"hole_to_hole={by.get('hole_to_hole',0)} (baseline 14) "
          f"new_via_hole={(by.get('hole_clearance',0)-32)+(by.get('hole_to_hole',0)-14)}", flush=True)
    gate = unc < 48 and by.get('hole_clearance',0) <= 32 and by.get('hole_to_hole',0) <= 14
    print(f"  M3-P1.1 GATE (unc<48, no via-hole regression): {'PASS' if gate else 'NOT MET'}", flush=True)


if __name__ == "__main__":
    main()
