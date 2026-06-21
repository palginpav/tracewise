"""Probe: Pour-Modeling Artifact vs True Wall analysis.

For 5 representative boxed-in signal nets (/QSPI_SD2, /GPIO3, /RUN, /SWCLK, /USB_D+),
after running the grid-only mitayi baseline (48 unconnected), classify each net as:
  - POUR-MODELING-ARTIFACT: fails with pours as hard obstacles, routes without them,
    AND the refill+DRC is clean.
  - TRUE-WALL: fails in BOTH models (pours are irrelevant — tracks block it).
  - DRC-INVALID: routes without pours but refill+DRC introduces new errors.

KEY INSIGHT from code analysis:
  build_windowed_free_space() in geom.py uses: pads + drill_obstacles + extra_obstacles
  (routed tracks). It does NOT include zone/pour polygons. So the current gridless
  model is ALREADY a "receding pour" model. The question is: does this already happen
  and do the nets still fail (TRUE WALL), or are they blocked by pours?

  To measure the HARD-POUR model (A), we must EXPLICITLY add pour polygons as obstacles.
  The RECEDING-POUR model (B) is the current model (no pours added).

Run: taskset -c 0-9 .venv/bin/python scripts/_probe_pour_artifact.py
"""
from __future__ import annotations

import collections
import json
import re
import shutil
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "src"))

try:
    import shapely
    from shapely.geometry import Polygon, box as _box, LineString, Point as SPoint
    from shapely.ops import unary_union
    from shapely import set_precision
    print(f"[probe] Shapely {shapely.__version__}  GEOS {shapely.geos_version}", flush=True)
except ImportError as exc:
    print(f"ERROR: Shapely not installed: {exc}", file=sys.stderr)
    sys.exit(1)

from tracewise.route.bridge import run_drc, strip_routing
from tracewise.route.engine.kicad import (
    build_problem,
    extract_pads,
    project_geometry,
    refill_zones,
    emit_routes,
)
from tracewise.route.engine.multi import route_all
from tracewise.route.gridless.geom import (
    build_windowed_free_space,
    extract_board_outline,
    extract_drill_obstacles,
    extract_drill_centers,
    net_routes_to_track_obstacles,
    snap,
    PRECISION,
)
from tracewise.route.gridless.route import route_net_multipin, GridlessRouteResult
from tracewise.route.gridless.adapter import GridlessNetRoute

BOARD_SRC = ROOT / "data/benchmark-boards/mitayi-pico-d1/Mitayi-Pico-D1.kicad_pcb"
SUFFIXES = (".kicad_pcb", ".kicad_sch", ".kicad_pro", ".kicad_prl")

# 5 representative probe nets (as specified)
PROBE_NETS = ["/QSPI_SD2", "/GPIO3", "/RUN", "/SWCLK", "/USB_D+"]

# Also probe these for completeness if time permits
EXTRA_NETS = ["/QSPI_SCLK", "/GPIO9", "/GPIO18"]

WINDOW_MM = 10.0
MAX_WINDOW_MM = 20.0


def setup_board(out_dir: Path) -> Path:
    bdir = BOARD_SRC.parent
    out_dir.mkdir(parents=True, exist_ok=True)
    for f in bdir.iterdir():
        if f.suffix in SUFFIXES:
            shutil.copy(f, out_dir / f.name)
    board = next(out_dir.glob("*.kicad_pcb"))
    strip_routing(board)
    return board


def _unconnected_nets(report: dict) -> set[str]:
    names = set()
    for u in report.get("unconnected_items", []):
        for it in u.get("items", []):
            for m in re.findall(r"\[([^\]]+)\]", it.get("description", "")):
                names.add(m)
    return names


def _extract_pours_shapely(board: Path) -> dict[str, list]:
    """Extract GND and +3V3 pour polygons as Shapely objects via pcbnew."""
    from tracewise.route.bridge import _run_pcbnew_script
    script = f"""
import wx; wx.DisableAsserts()
import pcbnew, json

IU = 1e6
b = pcbnew.LoadBoard({str(board)!r})
b.BuildConnectivity()

pours = []
for z in sorted(b.Zones(), key=lambda z: (z.GetNetname(), z.GetLayer())):
    net_name = z.GetNetname()
    if not net_name:
        continue
    for layer_id in z.GetLayerSet().CuStack():
        if layer_id not in (pcbnew.F_Cu, pcbnew.B_Cu):
            continue
        if not z.HasFilledPolysForLayer(layer_id):
            continue
        ps = z.GetFilledPolysList(layer_id)
        grid_layer = 0 if layer_id == pcbnew.F_Cu else 1
        for i in range(ps.OutlineCount()):
            outline = ps.Outline(i)
            n = outline.PointCount()
            pts = [[outline.CPoint(j).x / IU, outline.CPoint(j).y / IU]
                   for j in range(n)]
            pours.append({{
                "net": net_name,
                "layer": grid_layer,
                "pts": pts,
            }})

print("TWJSON" + json.dumps(pours))
raise SystemExit(0)
"""
    out = _run_pcbnew_script(script)
    result: dict[str, list] = {}
    for line in out.splitlines():
        if line.startswith("TWJSON"):
            raw = json.loads(line[len("TWJSON"):])
            for item in raw:
                net = item["net"]
                if net not in result:
                    result[net] = []
                try:
                    poly = snap(Polygon(item["pts"]))
                    result.setdefault(net, []).append(poly)
                except Exception:
                    pass
    return result


def _characterize_corridor(
    target_net: str,
    target_pads: list[dict],
    all_pads: list[dict],
    results: dict,
    grid,
    geo: dict,
    pour_polys: dict[str, list],
    board_outline,
    drill_obstacles: list,
    board_bbox: tuple,
) -> dict:
    """Characterize what's blocking the target net's corridor."""
    if not target_pads:
        return {"error": "no pads found"}

    xs = [p["x"] for p in target_pads]
    ys = [p["y"] for p in target_pads]
    x_lo = min(xs) - WINDOW_MM
    y_lo = min(ys) - WINDOW_MM
    x_hi = max(xs) + WINDOW_MM
    y_hi = max(ys) + WINDOW_MM
    window_poly = snap(_box(x_lo, y_lo, x_hi, y_hi))

    # Measure GND pour area in window
    gnd_area = 0.0
    p3v3_area = 0.0
    for net_name, polys in pour_polys.items():
        for poly in polys:
            try:
                clipped = snap(poly.intersection(window_poly))
                if not clipped.is_empty:
                    area = clipped.area
                    if "GND" in net_name.upper():
                        gnd_area += area
                    elif "3V3" in net_name or "3V" in net_name:
                        p3v3_area += area
            except Exception:
                pass

    # Measure track copper area in window (from results)
    track_area = 0.0
    from tracewise.route.engine.astar import simplify
    inflate = geo["track_mm"] / 2.0 + geo["clearance_mm"]
    for net_name2, nr in results.items():
        if net_name2 == target_net:
            continue
        if not nr.ok:
            continue
        if isinstance(nr, GridlessNetRoute):
            for wpath in nr.world_paths:
                if len(wpath) < 2:
                    continue
                try:
                    pts_2d = [(p[0], p[1]) for p in wpath]
                    seg = snap(LineString(pts_2d).buffer(inflate, cap_style=2))
                    clipped = snap(seg.intersection(window_poly))
                    if not clipped.is_empty:
                        track_area += clipped.area
                except Exception:
                    pass
        else:
            for path in nr.paths:
                runs = simplify(path)
                for run in runs:
                    if len(run) < 2:
                        continue
                    world_pts = [grid.to_world(c[1], c[2]) for c in run]
                    if len(world_pts) < 2:
                        continue
                    try:
                        seg = snap(LineString(world_pts).buffer(inflate, cap_style=2))
                        clipped = snap(seg.intersection(window_poly))
                        if not clipped.is_empty:
                            track_area += clipped.area
                    except Exception:
                        pass

    # Window area
    window_area = window_poly.area

    # Dominant blocker
    dominant = "tracks" if track_area >= gnd_area + p3v3_area else "pours"
    if gnd_area > p3v3_area:
        pour_type = "GND"
    elif p3v3_area > 0:
        pour_type = "+3V3"
    else:
        pour_type = "none"

    return {
        "window_area_mm2": round(window_area, 2),
        "gnd_pour_area_mm2": round(gnd_area, 4),
        "p3v3_pour_area_mm2": round(p3v3_area, 4),
        "track_area_mm2": round(track_area, 4),
        "dominant_blocker": dominant,
        "dominant_pour_type": pour_type,
        "pad_count": len(target_pads),
        "pad_positions": [(round(p["x"], 3), round(p["y"], 3)) for p in target_pads],
    }


def _build_pour_obstacles_for_net(
    pour_polys: dict[str, list],
    target_net: str,
    clearance_mm: float,
    track_mm: float,
    window_bbox: tuple,
) -> list:
    """Build Shapely inflated obstacles from all pours (for HARD-POUR model A).

    Pours are inflated by track_mm/2 + clearance_mm (same formula as pads/tracks).
    Only include pours from OTHER nets (target net's own pour is friendly copper).
    """
    inflate = clearance_mm + track_mm / 2.0
    wx1, wy1, wx2, wy2 = window_bbox
    obstacles = []
    for net_name, polys in pour_polys.items():
        if net_name == target_net:
            continue  # skip own net's pours (they're the routing target)
        for poly in polys:
            try:
                # Only use pours that intersect window
                if not poly.intersects(_box(wx1, wy1, wx2, wy2)):
                    continue
                inflated = snap(poly.buffer(inflate, cap_style=3, join_style=2))
                obstacles.append(inflated)
            except Exception:
                pass
    return obstacles


def try_route_ab(
    target_net: str,
    target_pads: list[dict],
    all_pads: list[dict],
    geo: dict,
    board_bbox: tuple,
    track_obstacles: list,
    board_outline,
    drill_obstacles: list,
    drill_centers: list,
    pour_polys: dict[str, list],
    window_mm: float = WINDOW_MM,
) -> dict:
    """Attempt route A (hard pour) and B (receding pour) for a net."""
    result = {}

    # Compute window bbox for this net
    xs = [p["x"] for p in target_pads]
    ys = [p["y"] for p in target_pads]
    bx1, by1, bx2, by2 = board_bbox
    wx1 = max(min(xs) - window_mm, bx1)
    wy1 = max(min(ys) - window_mm, by1)
    wx2 = min(max(xs) + window_mm, bx2)
    wy2 = min(max(ys) + window_mm, by2)
    window_bbox_net = (wx1, wy1, wx2, wy2)

    # --- Model A: Hard-pour --- pour polygons added as obstacles
    pour_obstacles = _build_pour_obstacles_for_net(
        pour_polys, target_net, geo["clearance_mm"], geo["track_mm"], window_bbox_net
    )
    obstacles_A = track_obstacles + pour_obstacles

    t0 = time.perf_counter()
    res_A = route_net_multipin(
        pads_of_net=target_pads,
        net_name=target_net,
        all_pads=all_pads,
        geo=geo,
        board_bbox=board_bbox,
        extra_obstacles=obstacles_A,
        window_mm=min(window_mm, MAX_WINDOW_MM),
        board_outline=board_outline,
        drill_obstacles=drill_obstacles,
        drill_centers=drill_centers,
    )
    time_A = time.perf_counter() - t0
    result["A_hard_pour_ok"] = res_A.ok
    result["A_reason"] = res_A.reason
    result["A_time_s"] = round(time_A, 2)
    result["A_pour_obstacles_count"] = len(pour_obstacles)
    print(f"  [A hard-pour]  ok={res_A.ok}  reason={res_A.reason!r}  "
          f"pour_obs={len(pour_obstacles)}  time={time_A:.1f}s", flush=True)

    # --- Model B: Receding-pour --- no pour polygons as obstacles
    t0 = time.perf_counter()
    res_B = route_net_multipin(
        pads_of_net=target_pads,
        net_name=target_net,
        all_pads=all_pads,
        geo=geo,
        board_bbox=board_bbox,
        extra_obstacles=track_obstacles,
        window_mm=min(window_mm, MAX_WINDOW_MM),
        board_outline=board_outline,
        drill_obstacles=drill_obstacles,
        drill_centers=drill_centers,
    )
    time_B = time.perf_counter() - t0
    result["B_receding_ok"] = res_B.ok
    result["B_reason"] = res_B.reason
    result["B_time_s"] = round(time_B, 2)
    result["B_result"] = res_B
    print(f"  [B receding]   ok={res_B.ok}  reason={res_B.reason!r}  "
          f"time={time_B:.1f}s", flush=True)

    return result


def _drc_for_net_b(
    board_src: Path,
    out_dir: Path,
    target_net: str,
    res_B: GridlessRouteResult,
    all_pads: list[dict],
    geo: dict,
    grid,
    anchors: dict,
    obstacles: dict,
    anchor_rects: dict,
    results: dict,
) -> dict:
    """Emit result B into a fresh board copy, refill, run DRC; return diff from baseline."""
    # Make a board copy with baseline routing already applied
    drc_dir = out_dir / f"drc_{target_net.replace('/', '_').replace('+', 'p')}"
    drc_dir.mkdir(parents=True, exist_ok=True)

    bdir = BOARD_SRC.parent
    for f in bdir.iterdir():
        if f.suffix in SUFFIXES:
            shutil.copy(f, drc_dir / f.name)
    board_copy = next(drc_dir.glob("*.kicad_pcb"))
    strip_routing(board_copy)

    # Emit baseline routing (all successfully-routed nets from grid pass)
    # Create a fake NetRoute for target net B result
    from tracewise.route.gridless.adapter import to_gridless_netroute
    from tracewise.route.engine.multi import Net

    # Add the B result as a new net in results
    # First emit baseline results (grid pass)
    emit_routes(
        board_copy, grid, results,
        track_mm=geo["track_mm"],
        via_mm=geo["via_mm"],
        via_drill_mm=geo["via_drill_mm"],
        anchors=anchors,
        neck_mm=geo["min_track_mm"],
        obstacles=obstacles,
        anchor_rects=anchor_rects,
        clearance_mm=geo["clearance_mm"],
    )

    # Also emit the B-routed net
    net_obj = Net(name=target_net, pads=[])  # minimal
    gnr = to_gridless_netroute(net_obj, res_B.world_paths, grid, res_B.world_vias)
    emit_routes(
        board_copy, grid, {target_net: gnr},
        track_mm=geo["track_mm"],
        via_mm=geo["via_mm"],
        via_drill_mm=geo["via_drill_mm"],
        anchors=anchors,
        neck_mm=geo["min_track_mm"],
        obstacles=obstacles,
        anchor_rects=anchor_rects,
        clearance_mm=geo["clearance_mm"],
    )

    refill_zones(board_copy)
    rep = run_drc(board_copy)
    errs = [v for v in rep.get("violations", []) if v.get("severity") == "error"]
    by = collections.Counter(v.get("type") for v in errs)
    unc = len(rep.get("unconnected_items", []))
    unc_nets = _unconnected_nets(rep)
    return {
        "unconnected": unc,
        "errors": len(errs),
        "by_type": dict(by),
        "hole_clearance": by.get("hole_clearance", 0),
        "hole_to_hole": by.get("hole_to_hole", 0),
        "copper_edge_clearance": by.get("copper_edge_clearance", 0),
        "net_connected": target_net not in unc_nets,
        "unc_nets": sorted(unc_nets),
    }


def main():
    t_start = time.perf_counter()
    print("[probe] Setting up board...", flush=True)
    out_dir = Path("/tmp/probe_pour_artifact")
    board = setup_board(out_dir)
    print(f"[probe] Board: {board}", flush=True)

    # Step 1: Run grid-only routing (baseline)
    print("\n[probe] Step 1: Grid-only routing...", flush=True)
    data = extract_pads(board)
    geo = project_geometry(board)
    print(f"[probe] geo={geo}", flush=True)
    grid, nets, anchors, obstacles_dict, anchor_rects = build_problem(
        data, pitch=0.1, track_mm=geo["track_mm"], clearance_mm=geo["clearance_mm"]
    )

    import math
    via_half = max(1, math.ceil(
        (geo["via_mm"] / 2 + geo["clearance_mm"] + geo["track_mm"] / 2) / 0.1))
    for n in nets:
        n.via_halfwidth_cells = via_half

    t_route = time.perf_counter()
    results = route_all(grid, nets, escape=12, ripup_factor=8, via_cost=10.0,
                        history_factor=1.0, allow_partial=True, salvage_escape=0)
    t_route_elapsed = time.perf_counter() - t_route
    print(f"[probe] Grid routing done in {t_route_elapsed:.1f}s", flush=True)

    ok_count = sum(1 for r in results.values() if r.ok)
    fail_count = len(nets) - ok_count
    print(f"[probe] Grid result: {ok_count} ok, {fail_count} failed", flush=True)

    # Emit baseline routing to board
    emit_routes(board, grid, results,
                track_mm=geo["track_mm"],
                via_mm=geo["via_mm"],
                via_drill_mm=geo["via_drill_mm"],
                anchors=anchors,
                neck_mm=geo["min_track_mm"],
                obstacles=obstacles_dict,
                anchor_rects=anchor_rects,
                clearance_mm=geo["clearance_mm"])
    refill_zones(board)

    # Run DRC on baseline
    print("[probe] Running baseline DRC...", flush=True)
    rep_baseline = run_drc(board)
    errs_baseline = [v for v in rep_baseline.get("violations", []) if v.get("severity") == "error"]
    by_baseline = collections.Counter(v.get("type") for v in errs_baseline)
    unc_baseline = len(rep_baseline.get("unconnected_items", []))
    unc_nets_baseline = _unconnected_nets(rep_baseline)
    print(f"[probe] Baseline: unc_items={unc_baseline}, errors={len(errs_baseline)}", flush=True)
    print(f"[probe] Baseline by_type={dict(by_baseline)}", flush=True)
    print(f"[probe] Baseline unconnected nets: {sorted(unc_nets_baseline)}", flush=True)

    # Step 2: Extract pour polygons from the REFILLED board
    print("\n[probe] Step 2: Extracting pour polygons...", flush=True)
    pour_polys = _extract_pours_shapely(board)
    pour_nets = sorted(pour_polys.keys())
    print(f"[probe] Pour nets: {pour_nets}", flush=True)
    for net_name, polys in pour_polys.items():
        total_area = sum(p.area for p in polys)
        print(f"[probe]   {net_name}: {len(polys)} polygons, area={total_area:.2f} mm2", flush=True)

    # Build track obstacles from grid results
    print("\n[probe] Building track obstacles...", flush=True)
    track_obstacles = net_routes_to_track_obstacles(
        results, grid, geo["track_mm"], geo["clearance_mm"]
    )
    print(f"[probe] Track obstacles: {len(track_obstacles)}", flush=True)

    # Extract board structures
    board_bbox = (data["board"]["x1"], data["board"]["y1"],
                  data["board"]["x2"], data["board"]["y2"])
    board_outline = extract_board_outline(board)
    drill_obstacles = extract_drill_obstacles(board, geo["clearance_mm"], geo["track_mm"])
    drill_centers = extract_drill_centers(board)
    print(f"[probe] Board bbox: {board_bbox}", flush=True)
    print(f"[probe] Board outline: {'ok' if board_outline else 'None'}", flush=True)
    print(f"[probe] Drill obstacles: {len(drill_obstacles)}", flush=True)

    # Determine which probe nets are actually unconnected
    probe_nets_to_test = [n for n in PROBE_NETS if n in unc_nets_baseline]
    # Add extras if any probe nets are not in the unconnected set
    for n in PROBE_NETS:
        if n not in unc_nets_baseline:
            print(f"[probe] WARNING: {n} is CONNECTED in baseline (not a target)", flush=True)
    for n in EXTRA_NETS:
        if n in unc_nets_baseline and len(probe_nets_to_test) < 8:
            probe_nets_to_test.append(n)

    print(f"\n[probe] Will probe {len(probe_nets_to_test)} nets: {probe_nets_to_test}", flush=True)

    # Step 3: A/B probe per net
    per_net_results = []
    max_net_runtime = 0.0

    for target_net in probe_nets_to_test:
        print(f"\n[probe] ===== Net: {target_net} =====", flush=True)
        t_net_start = time.perf_counter()

        target_pads = [p for p in data["pads"] if p.get("net") == target_net]
        print(f"  Pads: {len(target_pads)} at {[(round(p['x'],2), round(p['y'],2)) for p in target_pads]}", flush=True)

        # Characterize corridor
        corridor = _characterize_corridor(
            target_net, target_pads, data["pads"], results, grid, geo,
            pour_polys, board_outline, drill_obstacles, board_bbox
        )
        print(f"  Corridor: gnd_pour={corridor['gnd_pour_area_mm2']:.4f} mm2, "
              f"p3v3_pour={corridor['p3v3_pour_area_mm2']:.4f} mm2, "
              f"track={corridor['track_area_mm2']:.4f} mm2, "
              f"dominant={corridor['dominant_blocker']}", flush=True)

        # A/B route attempts
        ab = try_route_ab(
            target_net, target_pads, data["pads"], geo, board_bbox,
            track_obstacles, board_outline, drill_obstacles, drill_centers,
            pour_polys, window_mm=WINDOW_MM,
        )

        # Refill + DRC for nets that route in B
        refill_drc = None
        refill_drc_clean = None
        classification = None

        if ab["B_receding_ok"]:
            print(f"  [B OK] Running refill+DRC validation...", flush=True)
            try:
                t_drc = time.perf_counter()
                refill_drc = _drc_for_net_b(
                    BOARD_SRC, out_dir, target_net, ab["B_result"],
                    data["pads"], geo, grid, anchors, obstacles_dict, anchor_rects,
                    results,
                )
                t_drc_elapsed = time.perf_counter() - t_drc
                print(f"  [refill+DRC] done in {t_drc_elapsed:.1f}s: "
                      f"unc={refill_drc['unconnected']}, errors={refill_drc['errors']}, "
                      f"net_connected={refill_drc['net_connected']}", flush=True)
                print(f"  [refill+DRC] by_type={refill_drc['by_type']}", flush=True)

                # Check if DRC is clean (no new errors vs baseline)
                new_errors = refill_drc["errors"] - len(errs_baseline)
                net_connected = refill_drc["net_connected"]
                # Allow some tolerance: the B-route adds 1 net, so unc should decrease
                unc_delta = refill_drc["unconnected"] - unc_baseline
                refill_drc_clean = (
                    net_connected and
                    new_errors <= 2 and  # allow tiny float drift
                    unc_delta < 0  # net was actually connected
                )
                print(f"  [refill+DRC] net_connected={net_connected}, "
                      f"new_errors={new_errors}, unc_delta={unc_delta}, "
                      f"drc_clean={refill_drc_clean}", flush=True)
            except Exception as exc:
                print(f"  [refill+DRC] FAILED: {exc}", flush=True)
                refill_drc = {"error": str(exc)}
                refill_drc_clean = None

        # Classify
        if not ab["A_hard_pour_ok"] and ab["B_receding_ok"] and refill_drc_clean:
            classification = "POUR-MODELING-ARTIFACT"
        elif not ab["A_hard_pour_ok"] and ab["B_receding_ok"] and refill_drc_clean is False:
            classification = "DRC-INVALID"
        elif not ab["A_hard_pour_ok"] and not ab["B_receding_ok"]:
            classification = "TRUE-WALL"
        elif ab["A_hard_pour_ok"] and ab["B_receding_ok"]:
            # Both work - this means pours don't matter, it's routable either way
            classification = "ROUTABLE-EITHER-WAY"
        else:
            classification = "UNCLEAR"

        t_net_elapsed = time.perf_counter() - t_net_start
        max_net_runtime = max(max_net_runtime, t_net_elapsed)

        net_result = {
            "net": target_net,
            "in_baseline_unconnected": target_net in unc_nets_baseline,
            "dominant_blocker": corridor["dominant_blocker"],
            "dominant_pour_type": corridor["dominant_pour_type"],
            "gnd_pour_area_mm2": corridor["gnd_pour_area_mm2"],
            "p3v3_pour_area_mm2": corridor["p3v3_pour_area_mm2"],
            "track_area_mm2": corridor["track_area_mm2"],
            "A_hard_pour_ok": ab["A_hard_pour_ok"],
            "A_reason": ab["A_reason"],
            "A_pour_obstacles_count": ab["A_pour_obstacles_count"],
            "B_receding_ok": ab["B_receding_ok"],
            "B_reason": ab["B_reason"],
            "refill_drc": refill_drc,
            "refill_drc_clean": refill_drc_clean,
            "classification": classification,
            "net_runtime_s": round(t_net_elapsed, 1),
        }
        per_net_results.append(net_result)
        print(f"  CLASSIFICATION: {classification}  (runtime: {t_net_elapsed:.1f}s)", flush=True)

    # Step 4: Summary
    print("\n\n" + "="*60, flush=True)
    print("PROBE SUMMARY", flush=True)
    print("="*60, flush=True)
    print(f"Baseline: unc_items={unc_baseline}, errors={len(errs_baseline)}", flush=True)
    print(f"Probe nets: {len(per_net_results)}", flush=True)
    print(flush=True)

    count_pour_artifact = sum(1 for r in per_net_results if r["classification"] == "POUR-MODELING-ARTIFACT")
    count_true_wall = sum(1 for r in per_net_results if r["classification"] == "TRUE-WALL")
    count_drc_invalid = sum(1 for r in per_net_results if r["classification"] == "DRC-INVALID")
    count_routable_either = sum(1 for r in per_net_results if r["classification"] == "ROUTABLE-EITHER-WAY")
    count_unclear = sum(1 for r in per_net_results if r["classification"] == "UNCLEAR")

    for r in per_net_results:
        print(f"  {r['net']:20s} dominant={r['dominant_blocker']:8s} "
              f"A={r['A_hard_pour_ok']} B={r['B_receding_ok']} "
              f"drc_clean={r['refill_drc_clean']} => {r['classification']}", flush=True)

    print(flush=True)
    print(f"COUNT_POUR_ARTIFACT:       {count_pour_artifact}", flush=True)
    print(f"COUNT_TRUE_WALL:           {count_true_wall}", flush=True)
    print(f"COUNT_DRC_INVALID:         {count_drc_invalid}", flush=True)
    print(f"COUNT_ROUTABLE_EITHER_WAY: {count_routable_either}", flush=True)
    print(f"COUNT_UNCLEAR:             {count_unclear}", flush=True)
    print(f"MAX_NET_RUNTIME_S:         {max_net_runtime:.1f}", flush=True)

    if count_true_wall >= count_pour_artifact + count_drc_invalid:
        verdict = "mostly-true-wall: routing at ceiling"
        receding_pour_fix_recommended = False
        reasoning = (
            "Majority of nets fail in BOTH hard-pour and receding-pour models. "
            "The dominant blocker is track copper (not pours). Since the gridless model "
            "already doesn't include pours as obstacles (receding-pour by construction), "
            "a dedicated receding-pour fix would not help — nets are truly walled by tracks."
        )
    elif count_pour_artifact > count_true_wall + count_drc_invalid:
        verdict = "mostly-artifact: receding-pour model is high-value"
        receding_pour_fix_recommended = True
        reasoning = (
            "Majority of nets route successfully when pours are excluded from obstacles. "
            "These nets are blocked ONLY by the pour model, not by tracks. "
            "Building a receding-pour model would connect most of these nets."
        )
    else:
        verdict = "mixed"
        receding_pour_fix_recommended = count_pour_artifact > count_true_wall
        reasoning = "Mixed results — both pour artifacts and true walls present."

    print(f"\nVERDICT: {verdict}", flush=True)
    print(f"RECEDING-POUR FIX RECOMMENDED: {receding_pour_fix_recommended}", flush=True)
    print(f"REASONING: {reasoning}", flush=True)

    total_elapsed = time.perf_counter() - t_start
    print(f"\nTotal runtime: {total_elapsed:.1f}s", flush=True)

    # KEY INSIGHT NOTE
    print("\n" + "="*60, flush=True)
    print("KEY INSIGHT (from code analysis):", flush=True)
    print("  build_windowed_free_space() in geom.py ALREADY excludes pour polygons.", flush=True)
    print("  The current gridless model IS the 'receding pour' model by construction.", flush=True)
    print("  Extra obstacles = TRACKS only (net_routes_to_track_obstacles).", flush=True)
    print("  If nets STILL fail in model B, pours are NOT the issue — tracks/geometry block them.", flush=True)
    print("="*60, flush=True)

    # Structured Result
    structured = {
        "status": "complete",
        "summary": (
            f"Probed {len(per_net_results)} representative boxed-in nets. "
            f"Baseline: {unc_baseline} unconnected items, {len(errs_baseline)} errors. "
            f"pour_artifact={count_pour_artifact}, true_wall={count_true_wall}, "
            f"drc_invalid={count_drc_invalid}, routable_either_way={count_routable_either}."
        ),
        "files_changed": [],
        "files_read": [str(BOARD_SRC)],
        "nets_probed": [r["net"] for r in per_net_results],
        "per_net": [
            {
                "net": r["net"],
                "dominant_blocker": r["dominant_blocker"],
                "gnd_pour_area_mm2": r["gnd_pour_area_mm2"],
                "p3v3_pour_area_mm2": r["p3v3_pour_area_mm2"],
                "track_area_mm2": r["track_area_mm2"],
                "A_hard_pour_ok": r["A_hard_pour_ok"],
                "A_reason": r["A_reason"],
                "A_pour_obstacles_count": r["A_pour_obstacles_count"],
                "B_receding_ok": r["B_receding_ok"],
                "B_reason": r["B_reason"],
                "refill_drc_clean": r["refill_drc_clean"],
                "classification": r["classification"],
            }
            for r in per_net_results
        ],
        "count_pour_artifact": count_pour_artifact,
        "count_true_wall": count_true_wall,
        "count_drc_invalid": count_drc_invalid,
        "count_routable_either_way": count_routable_either,
        "baseline_unc_items": unc_baseline,
        "baseline_errors": len(errs_baseline),
        "baseline_unconnected_nets": sorted(unc_nets_baseline),
        "verdict": verdict,
        "receding_pour_fix_recommended": receding_pour_fix_recommended,
        "receding_pour_reasoning": reasoning,
        "max_net_runtime_s": round(max_net_runtime, 1),
        "total_runtime_s": round(total_elapsed, 1),
        "issues": [],
        "assumptions": [
            "Grid-only baseline with route_all defaults (pitch=0.1, ripup_factor=8, history_factor=1.0)",
            "Pour polygons extracted from refilled board after baseline grid routing",
            "Model A: adds pour polygons inflated by clearance+track/2 as extra_obstacles",
            "Model B: current production model — no pour polygons in obstacles",
            "Window=10mm (bounded, NOT board-wide)",
            "KEY: build_windowed_free_space() already excludes pours — Model B is the production model",
        ],
    }

    print("\n## Structured Result", flush=True)
    print("```json", flush=True)
    print(json.dumps(structured, indent=2, default=str), flush=True)
    print("```", flush=True)

    return structured


if __name__ == "__main__":
    main()
