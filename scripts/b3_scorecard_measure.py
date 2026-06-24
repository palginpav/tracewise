"""B3 Full-Scorecard Measurement: all 3 benchmark boards × {default, gridless_rescue}.

Canonical method:
  strip_routing -> route_board_engine -> run_drc
  severity==error counted; unconnected = len(unconnected_items)

Human targets (from scorecard):
  mitayi-pico-d1:       0 unconnected / 0 errors
  zuluscsi-pico-oshw:   0 unconnected / 5 errors
  rp2040-dev-board:     3 unconnected / 65 errors

Usage:
    .venv/bin/python scripts/b3_scorecard_measure.py [--boards all|mitayi|zuluscsi|rp2040]

The script runs in order: mitayi default, mitayi rescue, zuluscsi default,
zuluscsi rescue, rp2040 default, rp2040 rescue.  If RSS exceeds 1.8 GB or
wall-clock for a single run exceeds 600s, that run is aborted and marked partial.
"""
from __future__ import annotations

import argparse
import collections
import json
import os
import shutil
import sys
import time
from pathlib import Path

# ---------------------------------------------------------------------------
# Project root on path
# ---------------------------------------------------------------------------
HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT / "src"))

from tracewise.route.bridge import run_drc, strip_routing  # noqa: E402
from tracewise.route.engine.kicad import route_board_engine  # noqa: E402

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
SUFFIXES = (".kicad_pcb", ".kicad_sch", ".kicad_pro", ".kicad_prl")

BOARDS = {
    "mitayi": {
        "dir": ROOT / "data/benchmark-boards/mitayi-pico-d1",
        "human_unconnected": 0,
        "human_errors": 5,  # note: brief says 0/0, will use 0/0
        "human_unconnected_target": 0,
        "human_errors_target": 0,
    },
    "zuluscsi": {
        "dir": ROOT / "data/benchmark-boards/zuluscsi-pico-oshw",
        "human_unconnected_target": 0,
        "human_errors_target": 5,
    },
    "rp2040": {
        "dir": ROOT / "data/benchmark-boards/rp2040-dev-board",
        "human_unconnected_target": 3,
        "human_errors_target": 65,
    },
}

RSS_LIMIT_GB = 1.8
TIME_LIMIT_S = 600  # 10 min per run


# ---------------------------------------------------------------------------
# RSS check
# ---------------------------------------------------------------------------
def _check_rss_gb() -> float:
    """Return current RSS in GB (Linux /proc/self/status)."""
    try:
        for line in Path("/proc/self/status").read_text().splitlines():
            if line.startswith("VmRSS:"):
                kb = int(line.split()[1])
                return kb / (1024 ** 2)
    except Exception:
        pass
    return 0.0


# ---------------------------------------------------------------------------
# Measure one run
# ---------------------------------------------------------------------------
def _measure(
    board_key: str,
    config: str,
    bdir: Path,
    out: Path,
    gridless_rescue: bool,
    time_limit_s: float = TIME_LIMIT_S,
    rss_limit_gb: float = RSS_LIMIT_GB,
) -> dict:
    """Route one board with one config. Returns a measurement dict."""
    print(f"\n{'='*70}", flush=True)
    print(f"[{board_key.upper()} / {config}]  bdir={bdir}", flush=True)
    print(f"  gridless_rescue={gridless_rescue}", flush=True)
    print(f"  out={out}", flush=True)

    # Prepare working copy
    shutil.rmtree(out, ignore_errors=True)
    out.mkdir(parents=True)
    for f in bdir.iterdir():
        if f.suffix in SUFFIXES:
            shutil.copy(f, out / f.name)

    # Handle board filenames with spaces (rp2040)
    board_files = list(out.glob("*.kicad_pcb"))
    if not board_files:
        return {
            "board": board_key,
            "config": config,
            "status": "error",
            "error": "no .kicad_pcb found",
        }
    board = board_files[0]

    t0 = time.perf_counter()
    rss_start = _check_rss_gb()

    # Strip routing
    try:
        strip_routing(board)
    except Exception as e:
        return {
            "board": board_key, "config": config,
            "status": "error", "error": f"strip_routing failed: {e}",
        }

    # Route
    try:
        summary = route_board_engine(board, pitch=0.1, gridless_rescue=gridless_rescue)
        elapsed_route = time.perf_counter() - t0
    except Exception as e:
        elapsed_route = time.perf_counter() - t0
        return {
            "board": board_key, "config": config,
            "status": "error", "error": f"route_board_engine failed: {e}",
            "elapsed_s": round(elapsed_route, 1),
        }

    rss_mid = _check_rss_gb()
    print(f"  route done in {elapsed_route:.1f}s  RSS={rss_mid:.2f}GB", flush=True)

    if elapsed_route > time_limit_s:
        print(f"  ABORT: elapsed {elapsed_route:.0f}s > limit {time_limit_s}s", flush=True)
        return {
            "board": board_key, "config": config,
            "status": "aborted_time",
            "error": f"exceeded time limit {time_limit_s}s",
            "elapsed_s": round(elapsed_route, 1),
            "rss_gb": round(rss_mid, 3),
        }

    if rss_mid > rss_limit_gb:
        print(f"  ABORT: RSS {rss_mid:.2f}GB > limit {rss_limit_gb}GB", flush=True)
        return {
            "board": board_key, "config": config,
            "status": "aborted_rss",
            "error": f"exceeded RSS limit {rss_limit_gb}GB",
            "elapsed_s": round(elapsed_route, 1),
            "rss_gb": round(rss_mid, 3),
        }

    # DRC
    try:
        report = run_drc(board)
    except Exception as e:
        return {
            "board": board_key, "config": config,
            "status": "error", "error": f"run_drc failed: {e}",
            "elapsed_s": round(elapsed_route, 1),
        }

    elapsed_total = time.perf_counter() - t0
    rss_end = _check_rss_gb()
    peak_rss = max(rss_start, rss_mid, rss_end)

    violations = report.get("violations", [])
    errs = [v for v in violations if v.get("severity") == "error"]
    by_type = dict(collections.Counter(v.get("type") for v in errs))
    unconnected = len(report.get("unconnected_items", []))
    n_errors = len(errs)

    print(f"  unconnected={unconnected} errors={n_errors}", flush=True)
    print(f"  by_type={by_type}", flush=True)
    print(f"  total_elapsed={elapsed_total:.1f}s  peak_RSS={peak_rss:.2f}GB", flush=True)

    return {
        "board": board_key,
        "config": config,
        "status": "ok",
        "unconnected": unconnected,
        "errors": n_errors,
        "by_type": by_type,
        "elapsed_s": round(elapsed_total, 1),
        "route_elapsed_s": round(elapsed_route, 1),
        "peak_rss_gb": round(peak_rss, 3),
        "engine_summary": summary,
    }


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main() -> None:
    parser = argparse.ArgumentParser(description="B3 Full-Scorecard Measurement")
    parser.add_argument(
        "--boards", default="all",
        help="Comma-separated subset: all|mitayi|zuluscsi|rp2040"
    )
    args = parser.parse_args()

    board_keys = (
        ["mitayi", "zuluscsi", "rp2040"]
        if args.boards == "all"
        else [b.strip() for b in args.boards.split(",")]
    )

    human_targets = {
        "mitayi":   {"unconnected": 0, "errors": 0},
        "zuluscsi": {"unconnected": 0, "errors": 5},
        "rp2040":   {"unconnected": 3, "errors": 65},
    }

    t_global_start = time.perf_counter()
    results: list[dict] = []
    tmpdir = Path("/tmp/b3_scorecard")

    for board_key in board_keys:
        bdir = BOARDS[board_key]["dir"]
        assert bdir.exists(), f"Board dir not found: {bdir}"

        for config, rescue in [("default", False), ("gridless_rescue", True)]:
            out = tmpdir / f"{board_key}_{config}"
            r = _measure(
                board_key=board_key,
                config=config,
                bdir=bdir,
                out=out,
                gridless_rescue=rescue,
            )
            results.append(r)

            # Abort board's rescue run if we're getting close on memory
            rss_now = _check_rss_gb()
            if rss_now > RSS_LIMIT_GB and config == "default":
                print(f"  WARNING: RSS {rss_now:.2f}GB after default run; "
                      f"will proceed with rescue cautiously", flush=True)

    total_runtime_s = round(time.perf_counter() - t_global_start, 1)

    # -----------------------------------------------------------------------
    # Build scorecard table
    # -----------------------------------------------------------------------
    scorecard = []
    for r in results:
        bk = r["board"]
        tgt = human_targets.get(bk, {})
        if r.get("status") == "ok":
            vs_human_unc = r["unconnected"] - tgt.get("unconnected", 0)
            vs_human_err = r["errors"] - tgt.get("errors", 0)
            scorecard.append({
                "board": bk,
                "config": r["config"],
                "unconnected": r["unconnected"],
                "errors": r["errors"],
                "by_type": r["by_type"],
                "human_target": tgt,
                "vs_human": {
                    "unconnected_delta": vs_human_unc,
                    "errors_delta": vs_human_err,
                },
                "elapsed_s": r.get("elapsed_s"),
                "peak_rss_gb": r.get("peak_rss_gb"),
            })
        else:
            scorecard.append({
                "board": bk,
                "config": r["config"],
                "unconnected": None,
                "errors": None,
                "by_type": None,
                "human_target": tgt,
                "vs_human": None,
                "status": r.get("status"),
                "error": r.get("error"),
                "elapsed_s": r.get("elapsed_s"),
                "peak_rss_gb": r.get("peak_rss_gb"),
            })

    # -----------------------------------------------------------------------
    # Compute per-board gridless_rescue effect
    # -----------------------------------------------------------------------
    gridless_rescue_effect = {}
    for bk in board_keys:
        default_r = next((r for r in results if r["board"] == bk and r["config"] == "default"), None)
        rescue_r = next((r for r in results if r["board"] == bk and r["config"] == "gridless_rescue"), None)
        if default_r and rescue_r and default_r.get("status") == "ok" and rescue_r.get("status") == "ok":
            d_unc = default_r["unconnected"]
            r_unc = rescue_r["unconnected"]
            d_err = default_r["errors"]
            r_err = rescue_r["errors"]
            unc_delta = r_unc - d_unc
            err_delta = r_err - d_err
            if unc_delta < 0:
                effect = "helps"
            elif unc_delta > 0:
                effect = "hurts"
            else:
                # same unconnected — check errors
                if err_delta < 0:
                    effect = "helps_errors"
                elif err_delta > 0:
                    effect = "hurts_errors_noop_unc"
                else:
                    effect = "noop"
            gridless_rescue_effect[bk] = {
                "effect": effect,
                "default_unconnected": d_unc,
                "rescue_unconnected": r_unc,
                "unconnected_delta": unc_delta,
                "default_errors": d_err,
                "rescue_errors": r_err,
                "errors_delta": err_delta,
            }
        else:
            gridless_rescue_effect[bk] = {
                "effect": "unknown",
                "default_status": default_r.get("status") if default_r else "missing",
                "rescue_status": rescue_r.get("status") if rescue_r else "missing",
            }

    # -----------------------------------------------------------------------
    # Closest / furthest from human
    # -----------------------------------------------------------------------
    def distance_from_human(r: dict) -> float | None:
        if r.get("status") != "ok":
            return None
        tgt = human_targets.get(r["board"], {})
        unc_dist = abs(r["unconnected"] - tgt.get("unconnected", 0))
        err_dist = abs(r["errors"] - tgt.get("errors", 0))
        return unc_dist + err_dist * 0.1  # weight: unc is primary metric

    scored = [(r, distance_from_human(r)) for r in results if distance_from_human(r) is not None]
    scored.sort(key=lambda x: x[1])

    closest_to_human = (
        f"{scored[0][0]['board']}:{scored[0][0]['config']} "
        f"(delta_unc={scored[0][0]['unconnected'] - human_targets[scored[0][0]['board']]['unconnected']:+d}, "
        f"delta_err={scored[0][0]['errors'] - human_targets[scored[0][0]['board']]['errors']:+d})"
        if scored else "unknown"
    )
    furthest_from_human = (
        f"{scored[-1][0]['board']}:{scored[-1][0]['config']} "
        f"(delta_unc={scored[-1][0]['unconnected'] - human_targets[scored[-1][0]['board']]['unconnected']:+d}, "
        f"delta_err={scored[-1][0]['errors'] - human_targets[scored[-1][0]['board']]['errors']:+d})"
        if scored else "unknown"
    )

    # -----------------------------------------------------------------------
    # Biggest error opportunity (largest absolute error count that can be attacked)
    # -----------------------------------------------------------------------
    ok_results = [r for r in results if r.get("status") == "ok"]
    biggest_opportunity = "unknown"
    if ok_results:
        # Default-config results only (the baseline a user gets)
        default_ok = [r for r in ok_results if r["config"] == "default"]
        if default_ok:
            worst = max(default_ok, key=lambda r: r["errors"])
            by_t = worst.get("by_type", {})
            top_types = sorted(by_t.items(), key=lambda x: x[1], reverse=True)[:3]
            top_str = ", ".join(f"{t}={n}" for t, n in top_types)
            biggest_opportunity = (
                f"{worst['board']}:default ({worst['errors']} errors; "
                f"top error classes: {top_str})"
            )

    # -----------------------------------------------------------------------
    # Regressions
    # -----------------------------------------------------------------------
    regressions_found = []
    for bk, eff in gridless_rescue_effect.items():
        if eff.get("effect") in ("hurts", "hurts_errors_noop_unc"):
            regressions_found.append({
                "board": bk,
                "type": eff["effect"],
                "details": eff,
            })

    # -----------------------------------------------------------------------
    # Peak RSS and issues
    # -----------------------------------------------------------------------
    peak_rss_gb = max(
        (r.get("peak_rss_gb") or 0.0 for r in results), default=0.0
    )

    issues = []
    for r in results:
        if r.get("status") not in ("ok",):
            issues.append(
                f"{r['board']}:{r['config']} status={r.get('status')} "
                f"error={r.get('error', 'n/a')}"
            )
    if regressions_found:
        for reg in regressions_found:
            issues.append(f"REGRESSION: gridless_rescue hurts {reg['board']}: {reg['details']}")

    # -----------------------------------------------------------------------
    # Print human-readable scorecard table
    # -----------------------------------------------------------------------
    print("\n\n" + "=" * 80, flush=True)
    print("B3 SCORECARD TABLE", flush=True)
    print("=" * 80, flush=True)
    hdr = f"{'BOARD':<14} {'CONFIG':<18} {'UNC':>5} {'ERR':>5} {'H_UNC':>6} {'H_ERR':>6} {'D_UNC':>6} {'D_ERR':>6}"
    print(hdr, flush=True)
    print("-" * 80, flush=True)
    for s in scorecard:
        unc = str(s["unconnected"]) if s["unconnected"] is not None else "FAIL"
        err = str(s["errors"]) if s["errors"] is not None else "FAIL"
        tgt = s["human_target"]
        h_unc = str(tgt.get("unconnected", "?"))
        h_err = str(tgt.get("errors", "?"))
        vs = s.get("vs_human") or {}
        d_unc = f"{vs.get('unconnected_delta', '?'):+d}" if vs else "n/a"
        d_err = f"{vs.get('errors_delta', '?'):+d}" if vs else "n/a"
        print(f"{s['board']:<14} {s['config']:<18} {unc:>5} {err:>5} {h_unc:>6} {h_err:>6} {d_unc:>6} {d_err:>6}",
              flush=True)
    print("=" * 80, flush=True)
    print(f"\nClosest to human:  {closest_to_human}", flush=True)
    print(f"Furthest from human: {furthest_from_human}", flush=True)
    print(f"Biggest error opportunity: {biggest_opportunity}", flush=True)
    print(f"\nGridless rescue effect per board:", flush=True)
    for bk, eff in gridless_rescue_effect.items():
        print(f"  {bk}: {eff}", flush=True)
    if regressions_found:
        print(f"\nREGRESSIONS: {regressions_found}", flush=True)
    print(f"\nTotal runtime: {total_runtime_s}s  Peak RSS: {peak_rss_gb:.2f}GB", flush=True)

    # -----------------------------------------------------------------------
    # Structured result
    # -----------------------------------------------------------------------
    structured = {
        "status": "partial" if issues else "complete",
        "summary": (
            f"B3 scorecard: {len([r for r in results if r.get('status')=='ok'])}/"
            f"{len(results)} runs completed. "
            f"Closest: {closest_to_human}. "
            f"Furthest: {furthest_from_human}. "
            f"Biggest opportunity: {biggest_opportunity}."
        ),
        "files_changed": [],
        "files_read": [
            "scripts/_probe_route_human.py",
            "scripts/_verify_m3_ab.py",
            "scripts/_verify_fanout_rescue_ab.py",
            "src/tracewise/route/engine/kicad.py",
            "src/tracewise/route/bridge.py",
        ],
        "scorecard": scorecard,
        "gridless_rescue_effect": gridless_rescue_effect,
        "closest_to_human": closest_to_human,
        "furthest_from_human": furthest_from_human,
        "biggest_error_opportunity": biggest_opportunity,
        "regressions_found": regressions_found,
        "peak_rss_gb": round(peak_rss_gb, 3),
        "total_runtime_s": total_runtime_s,
        "issues": issues,
        "assumptions": [
            "Canonical: strip_routing -> route_board_engine(pitch=0.1) -> run_drc",
            "severity==error for error count (unconnected_items for unc count)",
            "default = gridless_rescue=False (all other params at defaults)",
            "gridless_rescue = gridless_rescue=True (all other params at defaults)",
            "RSS limit: 1.8GB per run (abort if exceeded)",
            "Time limit: 600s per run (abort if exceeded)",
            "Human targets: mitayi 0/0, zuluscsi 0/5, rp2040 3/65",
        ],
    }

    print("\n## Structured Result", flush=True)
    print("```json", flush=True)
    print(json.dumps(structured, indent=2), flush=True)
    print("```", flush=True)


if __name__ == "__main__":
    main()
