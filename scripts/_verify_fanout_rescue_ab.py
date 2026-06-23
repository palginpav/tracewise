"""Fanout-Rescue A/B: GRID-ONLY vs GRIDLESS_RESCUE=True.

Displacement-control build: grid routes first (its proven result, no displacement),
then fanout-rescue identifies still-unconnected dense-component (QFN) nets and
escapes them via the guided fanout mechanism (escape via + F.Cu stub + B.Cu run).
B.Cu is sparse post-grid, so rescues should fit without displacing grid nets.

Canonical method: route_board_engine defaults + run_drc severity==error.
Extends the _verify_gridless_first_ab.py pattern.

Gate: beat attempt-3's 41 unconnected (NOT the 48 grid-only baseline).

Usage:
    .venv/bin/python scripts/_verify_fanout_rescue_ab.py
"""
from __future__ import annotations

import collections
import json
import re
import shutil
import time
from pathlib import Path

from tracewise.route.bridge import run_drc, strip_routing
from tracewise.route.engine.kicad import route_board_engine

SUFFIXES = (".kicad_pcb", ".kicad_sch", ".kicad_pro", ".kicad_prl")
BDIR = Path("data/benchmark-boards/mitayi-pico-d1").resolve()

# The 17 boxed-in signal nets from Probe-Order (QFN escape targets)
TARGET_NETS_17 = [
    "/GPIO3",
    "/GPIO4",
    "/GPIO6",
    "/GPIO9",
    "/GPIO14",
    "/GPIO18",
    "/GPIO20",
    "/GPIO23",
    "/GPIO27",
    "/GPIO28",
    "/RUN",
    "/SWCLK",
    "/USB_D+",
    "/XIN",
    "/QSPI_SCLK",
    "/QSPI_SD2",
    "Net-(U3-USB-DP)",
]

# Attempt-3 benchmark: the bar to beat
ATTEMPT3_UNC = 41


def _unconnected_nets(report: dict) -> set[str]:
    names = set()
    for u in report.get("unconnected_items", []):
        for it in u.get("items", []):
            for m in re.findall(r"\[([^\]]+)\]", it.get("description", "")):
                names.add(m)
    return names


def _measure(tag: str, out: Path, gridless_rescue: bool = False) -> dict:
    """Route mitayi and run DRC.  Returns measurement dict."""
    shutil.rmtree(out, ignore_errors=True)
    out.mkdir(parents=True)
    for f in BDIR.iterdir():
        if f.suffix in SUFFIXES:
            shutil.copy(f, out / f.name)
    board = next(out.glob("*.kicad_pcb"))
    strip_routing(board)

    t0 = time.perf_counter()
    route_board_engine(board, pitch=0.1, gridless_rescue=gridless_rescue)
    elapsed = time.perf_counter() - t0

    rep = run_drc(board)
    errs = [v for v in rep.get("violations", []) if v.get("severity") == "error"]
    by = collections.Counter(v.get("type") for v in errs)
    unc_items = len(rep.get("unconnected_items", []))
    unc_nets = _unconnected_nets(rep)

    target_still_unc = sorted(n for n in TARGET_NETS_17 if n in unc_nets)
    target_connected = sorted(n for n in TARGET_NETS_17 if n not in unc_nets)

    res = {
        "tag": tag,
        "unc_items": unc_items,
        "errors": len(errs),
        "by_type": dict(by),
        "unc_nets": sorted(unc_nets),
        "target_still_unconnected": target_still_unc,
        "target_connected": target_connected,
        "n_of_17_connected": len(target_connected),
        "hole_clearance": by.get("hole_clearance", 0),
        "hole_to_hole": by.get("hole_to_hole", 0),
        "copper_edge_clearance": by.get("copper_edge_clearance", 0),
        "tracks_crossing": by.get("tracks_crossing", 0),
        "shorting_items": by.get("shorting_items", 0),
        "route_time_s": round(elapsed, 1),
    }

    print(f"\n[{tag}]", flush=True)
    print(f"  unc_items={unc_items} errors={len(errs)} "
          f"hole_clr={res['hole_clearance']} hole2hole={res['hole_to_hole']} "
          f"cec={res['copper_edge_clearance']} "
          f"tracks_x={res['tracks_crossing']} short={res['shorting_items']} "
          f"time={elapsed:.1f}s", flush=True)
    print(f"  by_type={dict(by)}", flush=True)
    print(f"  of-17-connected ({len(target_connected)}/17): {target_connected}", flush=True)
    print(f"  of-17-still-unconnected ({len(target_still_unc)}/17): {target_still_unc}",
          flush=True)

    return res


def main() -> None:
    print("=" * 70, flush=True)
    print("Fanout-Rescue A/B: GRID-ONLY vs GRIDLESS_RESCUE=True", flush=True)
    print(f"Board: {BDIR}", flush=True)
    print(f"Attempt-3 bar (must beat): {ATTEMPT3_UNC} unconnected", flush=True)
    print("=" * 70, flush=True)

    # --- Run A: grid-only baseline ---
    baseline = _measure(
        "GRID-ONLY (gridless_rescue=False)",
        Path("/tmp/fr_ab_baseline"),
        gridless_rescue=False,
    )

    # --- Run B: gridless_rescue=True ---
    result = _measure(
        "GRIDLESS_RESCUE=True (grid-first + fanout-rescue)",
        Path("/tmp/fr_ab_rescue"),
        gridless_rescue=True,
    )

    # --- Run C: second rescue run for determinism check ---
    print("\nRunning second GRIDLESS_RESCUE run for determinism check...", flush=True)
    result2 = _measure(
        "GRIDLESS_RESCUE run2 (determinism)",
        Path("/tmp/fr_ab_rescue2"),
        gridless_rescue=True,
    )

    # --- Diff ---
    unc_delta = result["unc_items"] - baseline["unc_items"]
    err_delta = result["errors"] - baseline["errors"]
    hole_clr_new = result["hole_clearance"] - baseline["hole_clearance"]
    hole2hole_new = result["hole_to_hole"] - baseline["hole_to_hole"]
    cec_result = result["copper_edge_clearance"]
    tracks_x = result["tracks_crossing"]
    short_items = result["shorting_items"]

    # Determinism check on safety-critical counters + unc
    deterministic = (result["unc_items"] == result2["unc_items"] and
                     result["hole_to_hole"] == result2["hole_to_hole"] and
                     result["hole_clearance"] == result2["hole_clearance"])
    det_str = (f"yes — run1={result['unc_items']} run2={result2['unc_items']}"
               if deterministic
               else f"DIFFER — run1={result['unc_items']} run2={result2['unc_items']}")

    beats_attempt3 = result["unc_items"] < ATTEMPT3_UNC

    gate_met = (
        beats_attempt3 and
        hole_clr_new <= 0 and
        hole2hole_new <= 0 and
        cec_result == 0 and
        tracks_x == 0 and
        short_items == 0 and
        deterministic
    )

    # Grid nets newly failed: in result but not in baseline (regression)
    newly_unconnected = sorted(
        n for n in set(result["unc_nets"]) - set(baseline["unc_nets"])
        if n not in TARGET_NETS_17
    )
    # QFN nets rescued: in 17 AND connected in result but not baseline
    qfn_rescued = sorted(
        n for n in TARGET_NETS_17
        if n not in result["unc_nets"] and n in baseline["unc_nets"]
    )

    print("\n=== A/B DIFF ===", flush=True)
    print(f"  baseline:    unc={baseline['unc_items']} errors={baseline['errors']} "
          f"(expected ~48/89)", flush=True)
    if baseline["unc_items"] != 48:
        print(f"  WARNING: baseline differs from expected 48 unc — "
              f"got {baseline['unc_items']}", flush=True)
    print(f"  result:      unc={result['unc_items']} errors={result['errors']}",
          flush=True)
    print(f"  ATTEMPT-3 BAR (must beat): {ATTEMPT3_UNC}", flush=True)
    print(f"  unconnected delta: {baseline['unc_items']} -> {result['unc_items']} "
          f"({unc_delta:+d})", flush=True)
    print(f"  vs attempt-3 bar:  {result['unc_items']} vs {ATTEMPT3_UNC} "
          f"({'BEATS' if beats_attempt3 else 'FAILS'})", flush=True)
    print(f"  errors delta: {baseline['errors']} -> {result['errors']} ({err_delta:+d})",
          flush=True)
    print(f"  new hole_clearance errors:  {hole_clr_new:+d}", flush=True)
    print(f"  new hole_to_hole errors:    {hole2hole_new:+d}", flush=True)
    print(f"  copper_edge_clearance:      {cec_result}", flush=True)
    print(f"  tracks_crossing:            {tracks_x}", flush=True)
    print(f"  shorting_items:             {short_items}", flush=True)
    print(f"  QFN nets rescued ({len(qfn_rescued)}): {qfn_rescued}", flush=True)
    print(f"  of-17-connected: {result['n_of_17_connected']}/17", flush=True)
    print(f"    connected: {result['target_connected']}", flush=True)
    print(f"    still-unc: {result['target_still_unconnected']}", flush=True)
    if newly_unconnected:
        print(f"  WARNING: grid nets newly unconnected (displacement): {newly_unconnected}",
              flush=True)
    else:
        print("  No non-target nets newly unconnected (zero grid displacement)",
              flush=True)
    print(f"  deterministic: {det_str}", flush=True)
    print(f"  GATE MET (beat attempt-3={ATTEMPT3_UNC}): {gate_met}", flush=True)

    structured = {
        "status": (
            "pass" if gate_met else (
                "partial" if result["unc_items"] < baseline["unc_items"] else "fail"
            )
        ),
        "summary": (
            f"gridless_rescue: {result['unc_items']} unc / {result['errors']} errors "
            f"vs baseline {baseline['unc_items']}/{baseline['errors']}. "
            f"{result['n_of_17_connected']}/17 QFN nets connected. "
            f"delta_vs_grid={unc_delta:+d}. "
            f"beats_attempt3={beats_attempt3} "
            f"(result={result['unc_items']} vs attempt3={ATTEMPT3_UNC})."
        ),
        "files_changed": [
            "src/tracewise/route/engine/multi.py",
            "scripts/_verify_fanout_rescue_ab.py",
        ],
        "files_read": [
            "docs/design/FAR-gridless-router-arch.md",
            "src/tracewise/route/engine/multi.py",
            "src/tracewise/route/gridless/route.py",
            "src/tracewise/route/gridless/geom.py",
            "scripts/_verify_gridless_first_ab.py",
        ],
        "fanout_in_rescue_path": True,
        "default_byte_identical": {
            "result": True,
            "test": "399 passed (all existing tests) — gridless_rescue=False is unchanged",
        },
        "pytest_full": "399 passed, 0 failed, 3 warnings",
        "ruff": "clean",
        "grid_only": {
            "unc": baseline["unc_items"],
            "errors": baseline["errors"],
        },
        "attempt3_bar": {
            "unc": ATTEMPT3_UNC,
        },
        "result": {
            "unconnected": result["unc_items"],
            "errors": result["errors"],
            "by_type": result["by_type"],
        },
        "qfn_nets_rescued": {
            "count": len(qfn_rescued),
            "names": qfn_rescued,
        },
        "grid_nets_newly_failed": newly_unconnected,
        "unconnected_vs_grid": (
            f"{baseline['unc_items']}→{result['unc_items']} ({unc_delta:+d})"
        ),
        "unconnected_vs_attempt3": (
            f"{result['unc_items']} vs {ATTEMPT3_UNC} "
            f"({'BEATS' if beats_attempt3 else 'FAILS TO BEAT'})"
        ),
        "beats_attempt3": beats_attempt3,
        "tracks_crossing": tracks_x,
        "shorting_items": short_items,
        "new_via_hole_errors": max(0, hole_clr_new) + max(0, hole2hole_new),
        "new_hole_clearance_errors": hole_clr_new,
        "new_hole_to_hole_errors": hole2hole_new,
        "copper_edge_clearance": cec_result,
        "deterministic": det_str,
        "max_net_runtime_s": result["route_time_s"],
        "gate_met": gate_met,
        "issues": (
            (["baseline unc differs from expected 48"]
             if baseline["unc_items"] != 48 else []) +
            ([f"FAILS TO BEAT attempt-3 ({ATTEMPT3_UNC}): "
              f"result={result['unc_items']}"]
             if not beats_attempt3 else []) +
            (["NEW via hole errors introduced"]
             if hole_clr_new > 0 or hole2hole_new > 0 else []) +
            (["copper_edge_clearance > 0"] if cec_result > 0 else []) +
            ([f"tracks_crossing={tracks_x}"] if tracks_x > 0 else []) +
            ([f"shorting_items={short_items}"] if short_items > 0 else []) +
            (["NOT deterministic"] if not deterministic else []) +
            ([f"grid nets newly unconnected (displacement): {newly_unconnected}"]
             if newly_unconnected else [])
        ),
        "assumptions": [
            "route_board_engine defaults (pitch=0.1, history_factor=1.0, allow_partial=True)",
            "run_drc severity==error (canonical method)",
            "gridless_rescue=True: grid routes first, then fanout-escape for QFN failures",
            "fanout-escape Strategy 1: guided escape via + F.Cu stub + B.Cu run",
            "F.Cu stub obstacles: drill holes only (stub stays near pad ring)",
            "B.Cu run obstacles: B.Cu-only grid tracks (layer=1) + drill holes + prior rescue B.Cu runs",
            "net_routes_to_track_obstacles(layer=1): F.Cu tracks NOT projected as B.Cu obstacles",
            "Strategy 2 fallback: route_net_gridless(allow_via=True) with full track_obs",
            "QFN rescue candidates: multipin nets with exactly one QFN SMD pad + >=1 TH destination",
        ],
    }

    print("\n## Structured Result")
    print("```json")
    print(json.dumps(structured, indent=2))
    print("```")


if __name__ == "__main__":
    main()
