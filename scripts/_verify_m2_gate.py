"""M2 gate verification — mirrors _probe_route_human EXACTLY (route_board_engine
defaults: synth_power_pours=True, pitch=0.1) so it is apples-to-apples with the
verified grid-only baseline (48 unconnected / 89 errors), but routes a gridless
subset via the negotiated path. Confirms the M2 gate: errors <= 89 AND
unconnected <= 48, copper_edge_clearance == 0, and determinism across 2 runs.

    taskset -c 0-9 .venv/bin/python scripts/_verify_m2_gate.py
"""
from __future__ import annotations

import collections
import shutil
from pathlib import Path

from tracewise.route.bridge import run_drc, strip_routing
from tracewise.route.engine.kicad import route_board_engine

SUFFIXES = (".kicad_pcb", ".kicad_sch", ".kicad_pro", ".kicad_prl")
BDIR = Path("data/benchmark-boards/mitayi-pico-d1").resolve()
SUBSET = {"/QSPI_SD0", "/QSPI_SD1", "/QSPI_SD3", "Net-(D2-A)", "Net-(D2-K)", "Net-(J2-CC1)"}


def _route_and_drc(tag: str, out: Path) -> tuple[int, int, dict]:
    shutil.rmtree(out, ignore_errors=True)
    out.mkdir(parents=True)
    for f in BDIR.iterdir():
        if f.suffix in SUFFIXES:
            shutil.copy(f, out / f.name)
    board = next(out.glob("*.kicad_pcb"))
    strip_routing(board)
    # defaults match _probe_route_human (synth_power_pours=True, pitch=0.1)
    route_board_engine(board, pitch=0.1, gridless_nets=set(SUBSET), gridless_negotiate=True)
    report = run_drc(board)
    errs = [v for v in report.get("violations", []) if v.get("severity") == "error"]
    by = collections.Counter(v.get("type") for v in errs)
    unc = len(report.get("unconnected_items", []))
    print(f"[{tag}] unconnected={unc} errors={len(errs)} by_type={dict(by)}", flush=True)
    return unc, len(errs), dict(by)


def main() -> None:
    print(f"gridless subset (negotiated): {sorted(SUBSET)}", flush=True)
    u1, e1, by1 = _route_and_drc("run1", Path("/tmp/verify_m2_1"))
    u2, e2, by2 = _route_and_drc("run2", Path("/tmp/verify_m2_2"))
    cec = by1.get("copper_edge_clearance", 0)
    det = "byte-identical-counts" if (u1, e1, by1) == (u2, e2, by2) else f"DIFFER ({u1}/{e1} vs {u2}/{e2})"
    print("=== M2 GATE (baseline 48 unc / 89 err, same method) ===", flush=True)
    print(f"  unconnected={u1} (<=48? {u1 <= 48})  errors={e1} (<=89? {e1 <= 89})", flush=True)
    print(f"  copper_edge_clearance={cec} (==0? {cec == 0})", flush=True)
    print(f"  determinism: {det}", flush=True)
    gate = u1 <= 48 and e1 <= 89 and cec == 0 and det == "byte-identical-counts"
    print(f"  M2 GATE MET: {gate}", flush=True)


if __name__ == "__main__":
    main()
