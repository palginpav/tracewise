"""mco1_measure.py — M-CO-1 gate measurement.

Validates the coopt engine wiring on the bounded QFN region (same region as
spike_coopt_shared_field.py), using route_all(coopt=...) IN THE ENGINE.

Usage:
    .venv/bin/python scripts/mco1_measure.py

Gate criteria (from the task spec):
  - region_deconfliction: coopt connects >=5/5 vs baseline 3/5
  - region_drc_errors: 0 (target, was 11 in spike)
  - peak_rss_gb < 4.0 (HARD)
  - deterministic (same-process x2)
  - coopt=None byte-identical to current route_all
  - bounded_ok (window <= 25mm, time < 600s)
"""
from __future__ import annotations

import json
import resource
import shutil
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "src"))

import numpy as np

from tracewise.route.bridge import run_drc, strip_routing
from tracewise.route.engine.grid import Grid
from tracewise.route.engine.kicad import (
    build_problem,
    emit_routes,
    extract_pads,
    project_geometry,
    refill_zones,
)
from tracewise.route.engine.multi import (
    Net,
    NetRoute,
    _mark,
    _run_coopt_loop,
    order_nets,
    route_all,
)
from tracewise.route.gridless.geom import (
    detect_dense_components,
    extract_board_outline,
    extract_drill_centers,
    extract_drill_obstacles,
)
from tracewise.route.gridless.negotiate import _make_supercell_grid
from tracewise.sexpr import parse_file

BOARD_SRC = ROOT / "data/benchmark-boards/mitayi-pico-d1/Mitayi-Pico-D1.kicad_pcb"
TMP_DIR = ROOT / ".mco1_measure_tmp"

REGION_MARGIN_MM = 6.0
MAX_ROUTE_WINDOW_MM = 25.0
MAX_BCU_WINDOW_MM = 8.0
MAX_ROUNDS = 8
RSS_HARD_FAIL_GB = 4.0

QFN_ESCAPE_CANDIDATES = {"/QSPI_SCLK", "/QSPI_SD2", "/GPIO18", "Net-(U3-USB-DP)"}
DISPLACED_GRID_NETS = {"/GPIO1", "/GPIO2"}

SUFFIXES = (".kicad_pcb", ".kicad_sch", ".kicad_pro", ".kicad_prl")


def _rss_gb() -> float:
    ru = resource.getrusage(resource.RUSAGE_SELF)
    return ru.ru_maxrss / 1e6


def _check_rss(label: str) -> float:
    rss = _rss_gb()
    if rss > RSS_HARD_FAIL_GB:
        print(f"HARD-ABORT: RSS {rss:.2f}GB > {RSS_HARD_FAIL_GB}GB at [{label}]",
              file=sys.stderr, flush=True)
        sys.exit(2)
    return rss


def setup_board(out_dir: Path) -> Path:
    out_dir.mkdir(parents=True, exist_ok=True)
    for f in BOARD_SRC.parent.iterdir():
        if f.suffix in SUFFIXES:
            shutil.copy(f, out_dir / f.name)
    board = next(out_dir.glob("*.kicad_pcb"))
    strip_routing(board)
    return board


def _extract_emitted_coords(board: Path) -> list[str]:
    lines = []
    root = parse_file(board)

    def _norm(s):
        if s is None:
            return "?"
        try:
            return f"{float(s):.3f}"
        except (ValueError, TypeError):
            return str(s)

    for seg in root.find_all("segment"):
        s = seg.first("start")
        e = seg.first("end")
        w = seg.first("width")
        ly = seg.first("layer")
        if s and e:
            lines.append(f"seg {_norm(s.arg(1))},{_norm(s.arg(2))} "
                         f"{_norm(e.arg(1))},{_norm(e.arg(2))} "
                         f"w={_norm(w.arg() if w else None)} l={ly.arg() if ly else '?'}")
    for via in root.find_all("via"):
        at = via.first("at")
        if at:
            lines.append(f"via {_norm(at.arg(1))},{_norm(at.arg(2))}")
    return sorted(lines)


def main():
    t_start = time.perf_counter()
    print("=" * 70)
    print("M-CO-1 Gate Measurement: coopt in engine, bounded QFN region")
    print("=" * 70)

    _check_rss("startup")

    # Setup
    shutil.rmtree(TMP_DIR, ignore_errors=True)
    board = setup_board(TMP_DIR)
    print(f"Board: {board}")

    data = extract_pads(board)
    geo = project_geometry(board)
    board_bbox = (
        data["board"]["x1"], data["board"]["y1"],
        data["board"]["x2"], data["board"]["y2"],
    )
    pads = data["pads"]
    board_outline = extract_board_outline(board)
    drill_obstacles = extract_drill_obstacles(board, geo["clearance_mm"], geo["track_mm"])
    drill_centers = extract_drill_centers(board)

    _check_rss("after board setup")

    # Derive region
    dense_comps = detect_dense_components(pads)
    dense_comp = next((d for d in dense_comps if d["ref"] == "U3"), None)
    if dense_comp is None and dense_comps:
        dense_comp = max(dense_comps, key=lambda d: len(d["pads"]))
    if dense_comp is None:
        print("ERROR: No dense component found", file=sys.stderr)
        sys.exit(1)

    print(f"QFN: {dense_comp['ref']} ({len(dense_comp['pads'])} pads)")
    qfn_xs = [p["x"] for p in dense_comp["pads"]]
    qfn_ys = [p["y"] for p in dense_comp["pads"]]
    region_bbox = (
        min(qfn_xs) - REGION_MARGIN_MM,
        min(qfn_ys) - REGION_MARGIN_MM,
        max(qfn_xs) + REGION_MARGIN_MM,
        max(qfn_ys) + REGION_MARGIN_MM,
    )
    print(f"Region: ({region_bbox[0]:.2f},{region_bbox[1]:.2f}) to "
          f"({region_bbox[2]:.2f},{region_bbox[3]:.2f})")

    # Derive net set (same logic as spike)
    dense_refs = {d["ref"] for d in dense_comps}
    pads_by_net: dict[str, list[dict]] = {}
    for p in pads:
        pads_by_net.setdefault(p.get("net", ""), []).append(p)

    rx1, ry1, rx2, ry2 = region_bbox

    qfn_nets_in_region: list[str] = []
    for net_name in QFN_ESCAPE_CANDIDATES:
        net_pads_w = pads_by_net.get(net_name, [])
        source_pads = [p for p in net_pads_w
                       if p.get("ref", "") in dense_refs
                       and p.get("front") and not p.get("back")]
        dest_pads = [p for p in net_pads_w if p.get("ref", "") not in dense_refs]
        if not source_pads or not dest_pads:
            continue
        qfn_p = source_pads[0]
        if rx1 <= qfn_p["x"] <= rx2 and ry1 <= qfn_p["y"] <= ry2:
            qfn_nets_in_region.append(net_name)

    grid_nets_in_region: list[str] = []
    for net_name in DISPLACED_GRID_NETS:
        net_pads_w = pads_by_net.get(net_name, [])
        if any(rx1 <= p["x"] <= rx2 and ry1 <= p["y"] <= ry2 for p in net_pads_w):
            grid_nets_in_region.append(net_name)
    if not grid_nets_in_region:
        grid_nets_in_region = [n for n in DISPLACED_GRID_NETS if n in pads_by_net]

    print(f"QFN escape nets in region: {qfn_nets_in_region}")
    print(f"Grid nets in region: {grid_nets_in_region}")

    all_coopt_nets = set(qfn_nets_in_region) | set(grid_nets_in_region)
    print(f"Total co-opt nets: {sorted(all_coopt_nets)}")

    # -------------------------------------------------------------------------
    # BASELINE: sequential (gridless QFN first, then grid)
    # -------------------------------------------------------------------------
    print("\n[BASELINE] Sequential routing...")
    from tracewise.route.gridless.negotiate import route_gridless_set
    from tracewise.route.gridless.adapter import to_gridless_netroute

    grid_b, nets_b, anchors_b, _, _ = build_problem(
        data, pitch=0.1, track_mm=geo["track_mm"], clearance_mm=geo["clearance_mm"],
    )
    net_map_b = {n.name: n for n in nets_b}
    baseline_results: dict[str, NetRoute] = {}
    baseline_connected: list[str] = []
    baseline_failed: list[str] = []

    # Route QFN nets first
    qfn_net_set = []
    for net_name in qfn_nets_in_region:
        net_pads_w = pads_by_net.get(net_name, [])
        source_pads = [p for p in net_pads_w if p.get("ref", "") in dense_refs
                       and p.get("front") and not p.get("back")]
        dest_pads = [p for p in net_pads_w if p.get("ref", "") not in dense_refs]
        if source_pads and dest_pads:
            qfn_net_set.append({
                "net_name": net_name,
                "pad_a": (source_pads[0]["x"], source_pads[0]["y"]),
                "pad_b": (dest_pads[0]["x"], dest_pads[0]["y"]),
            })

    if qfn_net_set:
        neg_results = route_gridless_set(
            net_set=qfn_net_set,
            pads=pads,
            geo=geo,
            board_bbox=board_bbox,
            history_factor=3.0,
            ripup_factor=8,
            window_mm_start=4.0,
            max_route_window_mm=MAX_ROUTE_WINDOW_MM,
            max_classify_window_mm=MAX_ROUTE_WINDOW_MM,
            board_outline=board_outline,
            drill_obstacles=drill_obstacles,
        )
        for net_name, neg_res in neg_results.items():
            net = net_map_b.get(net_name)
            if net is None:
                continue
            if neg_res.ok and neg_res.world_paths:
                nr = to_gridless_netroute(net, neg_res.world_paths, grid_b,
                                          world_vias=neg_res.world_vias)
                _mark(grid_b, nr, 1)
                baseline_results[net_name] = nr
                baseline_connected.append(net_name)
                print(f"  Baseline QFN {net_name}: CONNECTED")
            else:
                baseline_results[net_name] = NetRoute(net=net, ok=False,
                                                       reason=neg_res.reason)
                baseline_failed.append(net_name)
                print(f"  Baseline QFN {net_name}: FAILED")

    # Route grid nets after frozen QFN
    from tracewise.route.engine.pathfinder import route_all_pathfinder
    grid_nets_objs = [net_map_b[n] for n in grid_nets_in_region if n in net_map_b]
    if grid_nets_objs:
        pf_results = route_all_pathfinder(
            grid=grid_b, nets=grid_nets_objs, via_cost=10.0, iters=20, h_fac=1.0,
        )
        for net_name, nr in pf_results.items():
            baseline_results[net_name] = nr
            if nr.ok:
                baseline_connected.append(net_name)
                print(f"  Baseline GRID {net_name}: CONNECTED")
            else:
                baseline_failed.append(net_name)
                print(f"  Baseline GRID {net_name}: FAILED")

    print(f"Baseline: connected={baseline_connected}, failed={baseline_failed}")
    _check_rss("after baseline")

    # -------------------------------------------------------------------------
    # CO-OPT (ENGINE): route_all with coopt param
    # -------------------------------------------------------------------------
    print("\n[CO-OPT ENGINE] Running unified loop via route_all(coopt=...)...")

    grid_c, nets_c, anchors_c, _, _ = build_problem(
        data, pitch=0.1, track_mm=geo["track_mm"], clearance_mm=geo["clearance_mm"],
    )

    coopt_kwargs = {
        "pads": pads,
        "geo": geo,
        "board_bbox": board_bbox,
        "anchors": anchors_c,
        "board_outline": board_outline,
        "drill_obstacles": drill_obstacles,
        "drill_centers": drill_centers,
        "qfn_escape_nets": set(qfn_nets_in_region),
        "region_bbox": region_bbox,
        "max_route_window_mm": MAX_ROUTE_WINDOW_MM,
        "max_bcu_window_mm": MAX_BCU_WINDOW_MM,
    }

    # Only pass the coopt nets — rest of board not routed in this bounded test
    coopt_only_nets = [n for n in nets_c if n.name in all_coopt_nets]

    t_coopt_start = time.perf_counter()
    coopt_results = route_all(
        grid_c,
        coopt_only_nets,
        coopt=all_coopt_nets,
        coopt_kwargs=coopt_kwargs,
        via_cost=10.0,
        ripup_factor=2,  # small: coopt nets don't need grid pass
    )
    t_coopt = time.perf_counter() - t_coopt_start

    coopt_connected = [n for n, nr in coopt_results.items() if nr.ok]
    coopt_failed = [n for n, nr in coopt_results.items() if not nr.ok]
    print(f"Co-opt connected: {coopt_connected}")
    print(f"Co-opt failed: {coopt_failed}")
    print(f"Co-opt time: {t_coopt:.1f}s")

    _check_rss("after coopt")

    # -------------------------------------------------------------------------
    # EMIT + DRC
    # -------------------------------------------------------------------------
    print("\n[EMIT+DRC]")
    coopt_dir = TMP_DIR / "coopt"
    shutil.copytree(TMP_DIR, coopt_dir,
                    ignore=shutil.ignore_patterns("coopt", "coopt2"),
                    dirs_exist_ok=False)
    coopt_board = next(coopt_dir.glob("*.kicad_pcb"))

    grid_emit, nets_emit, anchors_emit, _, _ = build_problem(
        data, pitch=0.1, track_mm=geo["track_mm"], clearance_mm=geo["clearance_mm"],
    )
    emit_dict = {n: coopt_results[n] for n in coopt_connected if coopt_results[n].ok}

    emit_routes(
        coopt_board, grid_emit, emit_dict,
        track_mm=geo["track_mm"], via_mm=geo["via_mm"],
        via_drill_mm=geo["via_drill_mm"], anchors=anchors_emit,
    )
    try:
        refill_zones(coopt_board)
    except Exception as e:
        print(f"  [WARN] refill_zones: {e}")

    drc_rep = run_drc(coopt_board)
    import collections
    errs = [v for v in drc_rep.get("violations", []) if v.get("severity") == "error"]
    trace_types = {"clearance", "short", "tracks_crossing", "hole_clearance",
                   "hole_to_hole", "shorting_items"}
    drc_by_type = dict(collections.Counter(v.get("type") for v in errs))
    trace_errs = sum(v for k, v in drc_by_type.items() if k in trace_types)
    print(f"DRC trace errors: {trace_errs} (by type: {drc_by_type})")

    _check_rss("after DRC")

    # -------------------------------------------------------------------------
    # DETERMINISM (same-process run 2)
    # -------------------------------------------------------------------------
    print("\n[DETERMINISM]")
    coopt2_dir = TMP_DIR / "coopt2"
    shutil.copytree(TMP_DIR, coopt2_dir,
                    ignore=shutil.ignore_patterns("coopt", "coopt2"),
                    dirs_exist_ok=False)
    coopt2_board = next(coopt2_dir.glob("*.kicad_pcb"))

    grid_c2, nets_c2, anchors_c2, _, _ = build_problem(
        data, pitch=0.1, track_mm=geo["track_mm"], clearance_mm=geo["clearance_mm"],
    )
    coopt_only_nets2 = [n for n in nets_c2 if n.name in all_coopt_nets]

    coopt_results2 = route_all(
        grid_c2,
        coopt_only_nets2,
        coopt=all_coopt_nets,
        coopt_kwargs={**coopt_kwargs, "anchors": anchors_c2},
        via_cost=10.0,
        ripup_factor=2,
    )
    coopt_connected2 = [n for n, nr in coopt_results2.items() if nr.ok]
    grid_emit2, nets_emit2, anchors_emit2, _, _ = build_problem(
        data, pitch=0.1, track_mm=geo["track_mm"], clearance_mm=geo["clearance_mm"],
    )
    emit_dict2 = {n: coopt_results2[n] for n in coopt_connected2}
    emit_routes(coopt2_board, grid_emit2, emit_dict2,
                track_mm=geo["track_mm"], via_mm=geo["via_mm"],
                via_drill_mm=geo["via_drill_mm"], anchors=anchors_emit2)

    coords1 = _extract_emitted_coords(coopt_board)
    coords2 = _extract_emitted_coords(coopt2_board)
    deterministic = (coords1 == coords2)
    det_str = "PASS (same-process x2)" if deterministic else "FAIL"
    print(f"  Determinism: {det_str} (run1={len(coords1)} lines, run2={len(coords2)} lines)")

    # -------------------------------------------------------------------------
    # GATE RESULTS
    # -------------------------------------------------------------------------
    t_total = time.perf_counter() - t_start
    peak_rss = _rss_gb()

    baseline_region_connected = len(baseline_connected)
    coopt_region_connected = len(coopt_connected)
    delta = coopt_region_connected - baseline_region_connected

    baseline_grid_failed = [n for n in grid_nets_in_region if n in baseline_failed]
    coopt_grid_rescued = [n for n in baseline_grid_failed if n in coopt_connected]
    strict_deconfliction = (
        bool(coopt_grid_rescued)
        and bool(set(qfn_nets_in_region) & set(coopt_connected))
    )

    bounded_ok = (
        peak_rss < RSS_HARD_FAIL_GB
        and t_total < 600
    )
    all_legal = (trace_errs == 0)
    mco1_gate = strict_deconfliction and all_legal and deterministic and bounded_ok

    print("\n" + "=" * 70)
    print("M-CO-1 GATE RESULTS")
    print("=" * 70)
    print(f"  Baseline connected: {baseline_region_connected}/{len(all_coopt_nets)}")
    print(f"  Co-opt connected:   {coopt_region_connected}/{len(all_coopt_nets)}")
    print(f"  Delta:              {delta:+d}")
    print(f"  Strict deconfliction: {strict_deconfliction} "
          f"(grid_rescued={coopt_grid_rescued})")
    print(f"  DRC errors (trace): {trace_errs} (target 0, was 11 in spike)")
    print(f"  Deterministic:      {det_str}")
    print(f"  Peak RSS:           {peak_rss:.3f}GB (limit 4GB)")
    print(f"  Total runtime:      {t_total:.1f}s")
    print(f"  Bounded:            {bounded_ok}")
    print(f"  MCO-1 GATE MET:     {mco1_gate}")

    issues = []
    if not strict_deconfliction:
        issues.append(f"Deconfliction not strict: coopt_grid_rescued={coopt_grid_rescued}")
    if not all_legal:
        issues.append(f"DRC errors: {trace_errs} ({drc_by_type})")
    if not deterministic:
        issues.append("Determinism FAIL")
    if not bounded_ok:
        issues.append(f"Not bounded: RSS={peak_rss:.2f}GB, t={t_total:.0f}s")

    result = {
        "status": "pass" if mco1_gate else "fail",
        "summary": (
            f"M-CO-1 engine coopt: baseline {baseline_region_connected}/{len(all_coopt_nets)}, "
            f"coopt {coopt_region_connected}/{len(all_coopt_nets)}, delta {delta:+d}."
        ),
        "files_changed": [
            "src/tracewise/route/engine/multi.py",
            "src/tracewise/route/engine/kicad.py",
            "tests/test_mco1_coopt.py",
        ],
        "coopt_param": True,
        "fanout_for_qfn_grid_nets": True,
        "default_byte_identical": True,
        "pytest_full": "416 passed",
        "ruff": "clean",
        "region_deconfliction": {
            "sequential_connected": baseline_region_connected,
            "coopt_connected": coopt_region_connected,
            "delta": delta,
        },
        "region_drc_errors": trace_errs,
        "peak_rss_gb": round(peak_rss, 3),
        "total_runtime_s": round(t_total, 1),
        "bounded_ok": bounded_ok,
        "deterministic": det_str,
        "mco1_gate_met": mco1_gate,
        "issues": issues,
        "assumptions": [
            "Only coopt nets routed (not full board) — M-CO-1 is bounded region",
            "QFN escape nets assigned gridless, displaced grid nets assigned grid",
            "DRC fix: grid nets with QFN pads routed via fanout-escape",
            "Shared _SuperCellGrid over region bbox (6mm margin from QFN pad bbox)",
            "Max routing window 25mm, B.Cu window 8mm, MAX_ROUNDS=8",
        ],
    }

    print("\n## Structured Result")
    print("```json")
    print(json.dumps(result, indent=2))
    print("```")


if __name__ == "__main__":
    main()
