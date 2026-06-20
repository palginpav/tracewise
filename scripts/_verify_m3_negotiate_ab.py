"""M3-P1.1 controlled A/B: gridless-FIRST (negotiate) vs grid-only.

Compares:
  A = grid-only (gridless_negotiate=False) → expect 48/89, QSPI unconnected.
  B = gridless-first (gridless_negotiate=True, gridless_nets=QSPI 5) →
      negotiate path routes QSPI nets FIRST via 2-layer F.Cu→via→B.Cu BEFORE
      the grid; grid then routes remainder seeing their copper as occupied.

Determinism check: B is run twice; unconnected count must be stable.

Run (8-15 min):
    taskset -c 0-9 .venv/bin/python scripts/_verify_m3_negotiate_ab.py
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
QSPI_SET = set(QSPI)


def _unconnected_nets(report: dict) -> set[str]:
    names: set[str] = set()
    for u in report.get("unconnected_items", []):
        for it in u.get("items", []):
            for m in re.findall(r"\[([^\]]+)\]", it.get("description", "")):
                names.add(m)
    return names


def _measure(tag: str, negotiate: bool, gridless_nets: set[str] | None, out: Path) -> dict:
    shutil.rmtree(out, ignore_errors=True)
    out.mkdir(parents=True)
    for f in BDIR.iterdir():
        if f.suffix in SUFFIXES:
            shutil.copy(f, out / f.name)
    board = next(out.glob("*.kicad_pcb"))
    strip_routing(board)
    print(f"[{tag}] routing (gridless_negotiate={negotiate}, "
          f"gridless_nets={sorted(gridless_nets) if gridless_nets else None}) ...",
          flush=True)
    route_board_engine(board, pitch=0.1,
                       gridless_negotiate=negotiate,
                       gridless_nets=gridless_nets)
    print(f"[{tag}] routing done, running DRC ...", flush=True)
    rep = run_drc(board)
    errs = [v for v in rep.get("violations", []) if v.get("severity") == "error"]
    by = collections.Counter(v.get("type") for v in errs)
    unc_items = len(rep.get("unconnected_items", []))
    unc_nets = _unconnected_nets(rep)
    res = {
        "unc_items": unc_items,
        "errors": len(errs),
        "by_type": dict(by),
        "qspi_unconnected": sorted(n for n in unc_nets if n in QSPI_SET),
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
    # A: grid-only baseline
    a = _measure("A GRID-ONLY", False, None, Path("/tmp/m3neg_A"))

    # B1: gridless-first (run 1)
    b1 = _measure("B1 GRIDLESS-FIRST", True, QSPI_SET, Path("/tmp/m3neg_B1"))

    # B2: gridless-first (run 2 — determinism check)
    b2 = _measure("B2 GRIDLESS-FIRST", True, QSPI_SET, Path("/tmp/m3neg_B2"))

    print("\n=== M3-P1.1 NEGOTIATE A/B DIFF ===", flush=True)

    # A vs B1
    qspi_connected = sorted(QSPI_SET - set(b1["qspi_unconnected"]))
    unc_delta = b1["unc_items"] - a["unc_items"]
    print(f"  unconnected_items: A={a['unc_items']} -> B1={b1['unc_items']} "
          f"(delta {unc_delta:+d})", flush=True)
    print(f"  QSPI nets connected by gridless-first: {qspi_connected}", flush=True)
    print(f"  QSPI still unconnected after B: {b1['qspi_unconnected']}", flush=True)
    print(f"  errors: A={a['errors']} -> B1={b1['errors']} "
          f"(delta {b1['errors'] - a['errors']:+d})", flush=True)
    new_hole_clr = b1["hole_clearance"] - a["hole_clearance"]
    new_hole2hole = b1["hole_to_hole"] - a["hole_to_hole"]
    print(f"  hole_clearance: A={a['hole_clearance']} -> B1={b1['hole_clearance']} "
          f"(NEW {new_hole_clr:+d})", flush=True)
    print(f"  hole_to_hole: A={a['hole_to_hole']} -> B1={b1['hole_to_hole']} "
          f"(NEW {new_hole2hole:+d})", flush=True)
    print(f"  copper_edge_clearance: A={a['copper_edge_clearance']} -> "
          f"B1={b1['copper_edge_clearance']}", flush=True)
    print(f"  B1 by_type: {b1['by_type']}", flush=True)

    # Determinism: B1 vs B2
    det_stable = b1["unc_items"] == b2["unc_items"]
    print(f"\n  Determinism B1 vs B2: "
          f"unc {b1['unc_items']} vs {b2['unc_items']} → "
          f"{'STABLE' if det_stable else 'UNSTABLE'}", flush=True)

    # Gate evaluation
    connectivity_gain = b1["unc_items"] < 48
    no_hole_regression = (new_hole_clr <= 0 and new_hole2hole <= 0)
    errors_ok = b1["errors"] <= a["errors"] + 5  # allow small zone-fill noise

    print("\n  === GATE EVALUATION ===", flush=True)
    print(f"  unconnected < 48 (connectivity gain): {connectivity_gain} "
          f"(B1={b1['unc_items']} vs baseline 48)", flush=True)
    print(f"  0 NEW via hole_clearance/hole_to_hole errors: {no_hole_regression} "
          f"(hole_clr NEW={new_hole_clr:+d}, hole2hole NEW={new_hole2hole:+d})", flush=True)
    print(f"  errors not materially worse: {errors_ok} "
          f"(B1={b1['errors']} vs A={a['errors']})", flush=True)
    print(f"  deterministic: {det_stable}", flush=True)
    print(f"  QSPI nets connected: {len(qspi_connected)}/{len(QSPI)} → {qspi_connected}",
          flush=True)
    gate_met = connectivity_gain and no_hole_regression and errors_ok and det_stable
    print(f"\n  M3-P1.1 GATE: {'PASS' if gate_met else 'NOT MET'}", flush=True)

    if not connectivity_gain:
        print("\n  >>> Option-2 signal: gridless-first still does not beat 48.", flush=True)
        print("  >>> QSPI nets geometry-blocked even with no grid copper present.",
              flush=True)
        print("  >>> Next: cross-substrate rip-up (option 2).", flush=True)


if __name__ == "__main__":
    main()
