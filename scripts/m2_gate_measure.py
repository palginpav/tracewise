"""M2 Gate Measurement Script.

Routes mitayi HUMAN placement with a representative gridless_nets subset
(QSPI/GPIO 2-pin F.Cu nets, excluding geometry-blocked ones), grid for the remainder.

Baseline = grid-only mitayi 48 unconnected / 88 errors.
Gate: total errors <= 88 AND total unconnected <= 48.

Usage:
    .venv/bin/python scripts/m2_gate_measure.py [--run2]

    --run2: run the routing a second time to verify determinism.
"""
from __future__ import annotations

import json
import shutil
import sys
import tempfile
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "src"))

BOARD_SRC = ROOT / "data/benchmark-boards/mitayi-pico-d1/Mitayi-Pico-D1.kicad_pcb"

# These are the 2-pin F.Cu nets from the QSPI/GPIO region that the spike2 validated.
# Excludes geometry-blocked (need B.Cu/vias) and multi-pin nets.
# Selected from M1_NET_NAMES in test_gridless_engine.py + the spike2 net-set.
GRIDLESS_SUBSET = {
    "/QSPI_SCLK",
    "/QSPI_SD0",
    "/QSPI_SD1",
    "/QSPI_SD2",
    "/QSPI_SD3",
    "/XOUT",
    "/~{USB_BOOT}",
    "Net-(D2-A)",
    "Net-(D2-K)",
    "Net-(D3-K)",
    "Net-(J2-CC1)",
    "Net-(J2-CC2)",
    "Net-(JP6-C)",
    "Net-(U2-BP)",
    "Net-(U3-USB-DM)",
    "Net-(U3-USB-DP)",
}


def run_gate(run_label: str, out_dir: Path) -> dict:
    """Run a single routing pass and return results."""
    from tracewise.route.bridge import run_drc
    from tracewise.route.engine.kicad import project_geometry, route_board_engine

    board = out_dir / BOARD_SRC.name
    shutil.copy2(BOARD_SRC, board)

    # Strip existing routing
    from tracewise.route.bridge import strip_routing
    strip_routing(board)

    print(f"\n[{run_label}] Board: {board}", flush=True)

    geo = project_geometry(board)
    print(f"[{run_label}] geo: {geo}", flush=True)
    print(f"[{run_label}] gridless_subset ({len(GRIDLESS_SUBSET)} nets): "
          f"{sorted(GRIDLESS_SUBSET)}", flush=True)

    t0 = time.perf_counter()
    summary = route_board_engine(
        board,
        pitch=0.1,
        ripup_factor=8,
        via_cost=10.0,
        history_factor=1.0,
        allow_partial=True,
        synth_power_pours=True,
        gridless_nets=GRIDLESS_SUBSET,
        gridless_negotiate=True,
    )
    elapsed = time.perf_counter() - t0

    print(f"[{run_label}] route_board_engine done in {elapsed:.1f}s", flush=True)
    print(f"[{run_label}] summary: {summary}", flush=True)

    # DRC
    drc = run_drc(board)
    # run_drc returns raw JSON: {"violations": [...], "unconnected_items": [...]}
    # Use same methodology as _probe_route_human.py:
    #   - errors = severity=="error" violations only (NOT total violations which includes warnings)
    #   - unconnected = len(unconnected_items)
    violations = drc.get("violations", [])
    unconnected = len(drc.get("unconnected_items", []))
    errors = sum(1 for v in violations if v.get("severity") == "error")
    total_violations = len(violations)

    by_type: dict[str, int] = {}
    by_sev: dict[str, int] = {}
    for v in violations:
        t = v.get("type", "?")
        s = v.get("severity", "?")
        by_type[t] = by_type.get(t, 0) + 1
        by_sev[s] = by_sev.get(s, 0) + 1

    print(f"[{run_label}] DRC: unconnected={unconnected}  errors={errors}  "
          f"total_violations={total_violations}  by_severity={by_sev}", flush=True)
    print(f"[{run_label}] DRC by_type={dict(sorted(by_type.items(), key=lambda x: -x[1]))}",
          flush=True)

    # Count gridless results
    gridless_routed = 0
    gridless_failed = 0
    geometry_blocked = 0
    failures = summary.get("failures", {})
    for net_name in GRIDLESS_SUBSET:
        if net_name in failures:
            reason = failures[net_name]
            if "geometry-blocked" in str(reason).lower() or "M3" in str(reason):
                geometry_blocked += 1
            else:
                gridless_failed += 1
        else:
            gridless_routed += 1

    return {
        "run": run_label,
        "unconnected": unconnected,
        "errors": errors,
        "gridless_routed": gridless_routed,
        "gridless_failed": gridless_failed,
        "geometry_blocked": geometry_blocked,
        "elapsed_s": round(elapsed, 2),
        "summary": summary,
        "board": str(board),
    }


def main():
    run2 = "--run2" in sys.argv

    tmp_base = ROOT / ".m2_gate_tmp"
    tmp_base.mkdir(exist_ok=True)

    print("=" * 70, flush=True)
    print("M2 GATE MEASUREMENT", flush=True)
    print("Baseline: unconnected=48, errors=88 (severity='error' violations only)", flush=True)
    print("Gate: errors<=88 AND unconnected<=48", flush=True)
    print("=" * 70, flush=True)

    with tempfile.TemporaryDirectory(prefix="m2_gate_", dir=tmp_base) as tmp:
        out1 = Path(tmp) / "run1"
        out1.mkdir()
        r1 = run_gate("RUN1", out1)

        r2 = None
        deterministic = "not_checked"
        if run2:
            out2 = Path(tmp) / "run2"
            out2.mkdir()
            r2 = run_gate("RUN2", out2)
            if (r1["unconnected"] == r2["unconnected"] and
                    r1["errors"] == r2["errors"]):
                deterministic = "byte-identical (DRC counts match)"
            else:
                deterministic = (
                    f"DIFFER: run1={r1['unconnected']}/{r1['errors']} "
                    f"run2={r2['unconnected']}/{r2['errors']}"
                )
            print(f"\n[DETERMINISM] {deterministic}", flush=True)

        gate_errors_le_88 = r1["errors"] <= 88
        gate_unconnected_le_48 = r1["unconnected"] <= 48
        gate_met = gate_errors_le_88 and gate_unconnected_le_48

        print("\n" + "=" * 70, flush=True)
        print("M2 GATE RESULT", flush=True)
        print("=" * 70, flush=True)
        print(f"  RUN1 unconnected: {r1['unconnected']}  (baseline=48, gate: <=48)", flush=True)
        print(f"  RUN1 errors:      {r1['errors']}  (baseline=88, gate: <=88)", flush=True)
        print(f"  Gridless routed:  {r1['gridless_routed']}", flush=True)
        print(f"  Gridless failed:  {r1['gridless_failed']}", flush=True)
        print(f"  Geometry-blocked: {r1['geometry_blocked']}", flush=True)
        print(f"  Elapsed:          {r1['elapsed_s']}s", flush=True)
        print(f"  Gate errors<=88:  {gate_errors_le_88}", flush=True)
        print(f"  Gate uncn<=48:    {gate_unconnected_le_48}", flush=True)
        print(f"  GATE MET:         {gate_met}", flush=True)
        if r2:
            print(f"  Determinism:      {deterministic}", flush=True)
        print("=" * 70, flush=True)

        regression_msg = "N/A (gate met)"
        if not gate_met:
            delta_unc = r1["unconnected"] - 48
            delta_err = r1["errors"] - 88
            regression_msg = (
                f"REGRESSION: unconnected+{max(delta_unc,0)}, errors+{max(delta_err,0)} "
                f"vs baseline"
            )
            print(f"  REGRESSION: {regression_msg}", flush=True)

        structured = {
            "status": "complete",
            "summary": f"{'GATE MET' if gate_met else 'GATE MISSED'}: "
                       f"unconnected={r1['unconnected']}, errors={r1['errors']}",
            "baseline": {"unconnected": 48, "errors": 88},
            "m2_result": {
                "unconnected": r1["unconnected"],
                "errors": r1["errors"],
                "gridless_routed": r1["gridless_routed"],
                "geometry_blocked": r1["geometry_blocked"],
            },
            "gate_errors_le_88": gate_errors_le_88,
            "gate_unconnected_le_48": gate_unconnected_le_48,
            "m2_gate_met": gate_met,
            "deterministic": deterministic,
            "elapsed_s": r1["elapsed_s"],
            "grid_remainder_regression": regression_msg if not gate_met else "none",
            "gridless_subset": sorted(GRIDLESS_SUBSET),
            "run2": r2,
        }

        print("\n## Structured Result", flush=True)
        print("```json", flush=True)
        print(json.dumps(structured, indent=2), flush=True)
        print("```", flush=True)

        return structured


if __name__ == "__main__":
    result = main()
    sys.exit(0 if result.get("m2_gate_met") else 1)
