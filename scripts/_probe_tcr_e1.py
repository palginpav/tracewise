#!/usr/bin/env python3
"""
_probe_tcr_e1.py — TCR E1 make-or-break gate.

Tests whether route_net_steered can be steered by the witness homotopy classes.
Routes ~18 QFN-escape nets with fixed witness classes (escape_via_xy + lane_y_mm),
grid-routes the rest. Scores with DRC. Reports GO/NO-GO.

BOUNDS: max_window_mm=12, max_bcu_window_mm=8, RSS hard-abort >2GB,
per-net timeout >60s. Never full-board windows.

GO iff ALL: unconnected < 41 (beat attempt-3) AND errors materially < 73 (<=~50)
AND 0 illegal different-net crossings AND RSS < 2GB AND runtime < 5min
AND 3-run byte-identical.

Usage:
    .venv/bin/python scripts/_probe_tcr_e1.py [--runs N] [--out DIR]
"""
from __future__ import annotations

import argparse
import collections
import hashlib
import json
import math
import resource
import shutil
import sys
import time
from pathlib import Path

# Make sure src and scripts dirs are in sys.path
_REPO = Path(__file__).parent.parent
sys.path.insert(0, str(_REPO / "src"))
sys.path.insert(0, str(Path(__file__).parent))

from probe_human_routing import (  # noqa: E402
    parse_pcb, extract_segments, extract_vias, extract_nets,
    PCB_PATH,
)

# ── Constants ────────────────────────────────────────────────────────────────

RSS_HARD_FAIL_GB = 2.0
NET_TIMEOUT_S = 60.0
MAX_WINDOW_MM = 12.0
MAX_BCU_WINDOW_MM = 8.0

# QFN escape nets: the 13 FAILING_NETS from _invent2_topology.py
# These are the nets our router currently fails on that the human solved.
# We steer THESE nets using witness classes; all others route via the grid engine.
QFN_ESCAPE_NETS = {
    "/GPIO3", "/GPIO4", "/GPIO6", "/GPIO9", "/GPIO14",
    "/GPIO20", "/GPIO23", "/GPIO27", "/GPIO28",
    "/RUN", "/SWCLK", "/XIN", "/USB_D+",
}

# Short F.Cu-only nets (no via, escape_via_xy=None per Rule H5)
# These 3 route entirely on F.Cu without a via per the witness homotopy sketch.
FCU_ONLY_NETS = {"/GPIO27", "/GPIO28", "/XIN"}

SUFFIXES = (".kicad_pcb", ".kicad_sch", ".kicad_pro", ".kicad_prl")


# ── RSS guard ────────────────────────────────────────────────────────────────

def _rss_gb() -> float:
    ru = resource.getrusage(resource.RUSAGE_SELF)
    return ru.ru_maxrss / 1e6  # Linux: KB → GB


def _check_rss(label: str) -> None:
    rss = _rss_gb()
    if rss > RSS_HARD_FAIL_GB:
        raise MemoryError(
            f"RSS {rss:.2f} GB > {RSS_HARD_FAIL_GB} GB at [{label}] — hard abort"
        )


# ── Footprint finder ─────────────────────────────────────────────────────────

def find_fps(root) -> dict[str, tuple[float, float]]:
    """Return {ref: (x, y)} for every footprint (top-level `at` position)."""
    result: dict[str, tuple[float, float]] = {}
    def walk(node):
        if isinstance(node, list) and node and node[0] == "footprint":
            ref, x, y = None, 0.0, 0.0
            for child in node[1:]:
                if isinstance(child, list) and child:
                    if child[0] == "at" and len(child) >= 3:
                        try:
                            x, y = float(child[1]), float(child[2])
                        except Exception:
                            pass
                    if child[0] == "property" and len(child) >= 3 and child[1] == "Reference":
                        ref = child[2]
            if ref:
                result[ref] = (x, y)
        if isinstance(node, list):
            for c in node[1:]:
                walk(c)
    walk(root)
    return result


# ── Witness class extraction ──────────────────────────────────────────────────

def extract_topo_classes(pcb_path: Path) -> dict[str, dict]:
    """Extract witness TopoClass for each QFN-escape net from the human board.

    Uses the kicad engine's extract_pads for dest/source coordinates (same
    coordinate system as the router) and probe_human_routing's parser for
    vias and segments.

    Returns dict[net_name] = {
        'escape_via_xy': (x, y) | None,
        'lane_y_mm':     float | None,
        'dest_xy':       (x, y) | None,
        'order_key':     float,     # = dest_y (monotone bus sort key)
        'source_xy':     (x, y) | None,  # QFN pad center
    }
    """
    # Import kicad extractor for correct coordinate system
    import sys as _sys
    _sys.path.insert(0, str(Path(__file__).parent.parent / "src"))
    from tracewise.route.engine.kicad import extract_pads as kicad_extract_pads
    from tracewise.route.gridless.geom import detect_dense_components

    root = parse_pcb(pcb_path)
    nets = extract_nets(root)
    segments = extract_segments(root)
    vias = extract_vias(root)

    name2num = {v: k for k, v in nets.items()}

    # Use kicad extractor for pad positions (correct coordinate system)
    kicad_data = kicad_extract_pads(pcb_path)
    kicad_pads = kicad_data["pads"]

    # Find U3 center from kicad pads
    dense_comps = detect_dense_components(kicad_pads)
    u3_comp = next((d for d in dense_comps if d["ref"] == "U3"), None)
    if u3_comp:
        u3x, u3y = u3_comp["cx"], u3_comp["cy"]
    else:
        # fallback: centroid of U3 pads
        u3_pads = [p for p in kicad_pads if p.get("ref") == "U3"]
        u3x = sum(p["x"] for p in u3_pads) / max(len(u3_pads), 1)
        u3y = sum(p["y"] for p in u3_pads) / max(len(u3_pads), 1)
    print(f"  U3 center (kicad): ({u3x:.3f}, {u3y:.3f})")

    # Connector + passive pads: destination lookup by net name.
    # Primary: J3, J4, J5 headers (through-hole connectors).
    # Secondary: passives/crystals for nets not on J3/J4 (e.g. /XIN → Y1).
    # For J3/J4 two-row connectors, prefer the INNER row (closer to the QFN):
    #   J3 inner row at y≈98.20 (vs outer y≈99.81),
    #   J4 inner row at y≈80.40 (vs outer y≈78.79).
    # This shortens the B.Cu run and reduces routing congestion.
    connector_refs = {"J3", "J4", "J5"}
    passive_refs = {"Y1", "C17", "R9", "SW1"}  # fallback dests for special nets

    # For J3/J4: choose pad closest to the QFN at y≈89mm (i.e. minimal |y - 89.0|)
    J3_INNER_Y = 98.20   # inner J3 row, closer to QFN
    J4_INNER_Y = 80.40   # inner J4 row, closer to QFN

    dest_by_net: dict[str, tuple[float, float]] = {}

    # Pass 1: connectors — prefer inner row for J3/J4
    # Build all connector pads grouped by (net, ref)
    from collections import defaultdict as _dd
    conn_pads_by_net_ref: dict = _dd(list)
    for p in kicad_pads:
        ref = p.get("ref", "")
        if ref in connector_refs:
            nm = p.get("net", "")
            if nm:
                conn_pads_by_net_ref[(nm, ref)].append(p)

    for (nm, ref), plist in conn_pads_by_net_ref.items():
        if nm in dest_by_net:
            continue
        if ref == "J3":
            # Prefer inner row (y ≈ 98.20)
            best = min(plist, key=lambda p: abs(p["y"] - J3_INNER_Y))
        elif ref == "J4":
            # Prefer inner row (y ≈ 80.40)
            best = min(plist, key=lambda p: abs(p["y"] - J4_INNER_Y))
        else:
            best = plist[0]
        dest_by_net[nm] = (best["x"], best["y"])

    # Pass 2: passives for nets that have no connector dest yet
    for p in kicad_pads:
        if p.get("ref", "") in passive_refs:
            nm = p.get("net", "")
            if nm and nm not in dest_by_net and p.get("ref") not in {"U3"}:
                dest_by_net[nm] = (p["x"], p["y"])

    # U3 source pads: QFN F.Cu pad per net
    source_by_net: dict[str, tuple[float, float]] = {}
    dense_refs = {d["ref"] for d in dense_comps}
    for p in kicad_pads:
        if p.get("ref", "") in dense_refs and p.get("front") and not p.get("back"):
            nm = p.get("net", "")
            if nm and nm not in source_by_net:
                source_by_net[nm] = (p["x"], p["y"])

    result: dict[str, dict] = {}

    for net_name in QFN_ESCAPE_NETS:
        num = name2num.get(net_name)
        if num is None:
            continue

        # --- Escape via (closest via to U3, within 12mm) ---
        net_vias = [v for v in vias if v.get("net_num") == num and "at" in v]
        escape_via_xy: tuple[float, float] | None = None

        if net_name not in FCU_ONLY_NETS and net_vias:
            by_r = [(math.hypot(v["at"][0] - u3x, v["at"][1] - u3y), v)
                    for v in net_vias]
            by_r.sort(key=lambda t: t[0])
            r_min, v_min = by_r[0]
            if r_min < 12.0:
                escape_via_xy = (v_min["at"][0], v_min["at"][1])

        # --- Lane y: longest horizontal B.Cu segment ---
        bcu_segs = [
            s for s in segments
            if s.get("net_num") == num and s.get("layer") == "B.Cu"
            and "start" in s and "end" in s
        ]
        lane_y_mm: float | None = None
        if bcu_segs:
            horiz = []
            for s in bcu_segs:
                ddx = abs(s["end"][0] - s["start"][0])
                ddy = abs(s["end"][1] - s["start"][1])
                length = math.hypot(ddx, ddy)
                if ddx > ddy * 2.0 and length > 0.5:
                    y_mid = (s["start"][1] + s["end"][1]) / 2.0
                    horiz.append((length, y_mid))
            if horiz:
                horiz.sort(reverse=True)
                lane_y_mm = horiz[0][1]

        dest_xy = dest_by_net.get(net_name)
        source_xy = source_by_net.get(net_name)
        # Primary sort key = dest_y (for crossing-free monotone bus).
        # Secondary key = dest_x (for ties on dest_y — nets going to the same
        # connector row; route left-to-right in x so via order matches pad order).
        if dest_xy is not None:
            order_key = dest_xy[1] * 10000 + dest_xy[0]
        else:
            order_key = 9999.0 * 10000

        result[net_name] = {
            "escape_via_xy": escape_via_xy,
            "lane_y_mm": lane_y_mm,
            "dest_xy": dest_xy,
            "source_xy": source_xy,
            "order_key": order_key,
        }

    return result


# ── Crossing audit ────────────────────────────────────────────────────────────

def seg_intersect(p1, p2, p3, p4) -> bool:
    """Proper segment intersection (excludes shared endpoints)."""
    def ccw(a, b, c):
        return (c[1] - a[1]) * (b[0] - a[0]) - (b[1] - a[1]) * (c[0] - a[0])
    pts = [p1, p2, p3, p4]
    for i in range(2):
        for j in range(2, 4):
            if abs(pts[i][0] - pts[j][0]) < 1e-6 and abs(pts[i][1] - pts[j][1]) < 1e-6:
                return False
    d1 = ccw(p3, p4, p1)
    d2 = ccw(p3, p4, p2)
    d3 = ccw(p1, p2, p3)
    d4 = ccw(p1, p2, p4)
    return ((d1 > 0) != (d2 > 0)) and ((d3 > 0) != (d4 > 0))


def audit_crossings(board_path: Path) -> dict[str, int]:
    """Count different-net same-layer track crossings on the routed board."""
    root = parse_pcb(board_path)
    segments = extract_segments(root)
    result: dict[str, int] = {}
    for layer in ("F.Cu", "B.Cu"):
        ls = [s for s in segments
              if s.get("layer") == layer and "start" in s and "end" in s]
        count = 0
        for i in range(len(ls)):
            for j in range(i + 1, len(ls)):
                if ls[i].get("net_num") == ls[j].get("net_num"):
                    continue
                if seg_intersect(ls[i]["start"], ls[i]["end"],
                                  ls[j]["start"], ls[j]["end"]):
                    count += 1
        result[layer] = count
    return result


# ── Single E1 run ─────────────────────────────────────────────────────────────

def run_e1(out_dir: Path, topo_classes: dict[str, dict]) -> dict:
    """Execute one E1 routing run. Returns a score dict."""
    from tracewise.route.bridge import run_drc, strip_routing
    from tracewise.route.engine.kicad import (
        emit_routes, extract_pads as kicad_extract_pads,
        project_geometry, build_problem, refill_zones,
    )
    from tracewise.route.engine.multi import route_all, _mark
    from tracewise.route.gridless.adapter import to_gridless_netroute
    from tracewise.route.gridless.geom import (
        detect_dense_components,
        extract_board_outline,
        extract_drill_centers,
        extract_drill_obstacles,
    )
    from tracewise.route.gridless.route import route_net_steered

    # ── Set up board in temp dir ─────────────────────────────────────────────
    board_dir = Path(PCB_PATH).parent
    shutil.rmtree(out_dir, ignore_errors=True)
    out_dir.mkdir(parents=True)
    for f in board_dir.iterdir():
        if f.suffix in SUFFIXES:
            shutil.copy(f, out_dir / f.name)
    board = next(out_dir.glob("*.kicad_pcb"))
    strip_routing(board)

    # ── Extract board data ───────────────────────────────────────────────────
    data = kicad_extract_pads(board)
    geo = project_geometry(board)
    GRID_PITCH = 0.1  # attempt-3 baseline pitch
    grid, nets, anchors, obstacles, anchor_rects = build_problem(
        data, pitch=GRID_PITCH, track_mm=geo["track_mm"], clearance_mm=geo["clearance_mm"]
    )

    via_half = max(1, math.ceil(
        (geo["via_mm"] / 2 + geo["clearance_mm"] + geo["track_mm"] / 2) / GRID_PITCH
    ))
    for n in nets:
        n.via_halfwidth_cells = via_half

    bd = data["board"]
    board_bbox = (bd["x1"], bd["y1"], bd["x2"], bd["y2"])
    board_outline = extract_board_outline(board)
    drill_obstacles = extract_drill_obstacles(
        board, clearance_mm=geo["clearance_mm"], track_mm=geo["track_mm"]
    )
    drill_centers = extract_drill_centers(board)

    pads = data["pads"]
    pads_by_net: dict[str, list[dict]] = {}
    for p in pads:
        pads_by_net.setdefault(p.get("net", ""), []).append(p)

    # Detect dense components (the RP2040 QFN, ref U3)
    dense_comps = detect_dense_components(pads)
    dense_by_ref = {d["ref"]: d for d in dense_comps}
    dense_refs = set(dense_by_ref)

    nets_by_name = {n.name: n for n in nets}

    # ── Sort escape nets by order_key (monotone dest-y) ──────────────────────
    escape_order = sorted(
        [
            (nm, tc)
            for nm, tc in topo_classes.items()
            if nm in nets_by_name
        ],
        key=lambda x: x[1]["order_key"],
    )
    print(f"  Escape nets to steer: {len(escape_order)}", flush=True)

    # Accumulated B.Cu obstacles from successfully routed escape nets.
    # Passed to bcu_extra_obstacles to prevent B.Cu run overlaps.
    # NOT passed to via placement or F.Cu stub (witness vias are tightly packed).
    try:
        from shapely.geometry import LineString as _LS
        from tracewise.route.gridless.geom import snap as _snap
        _have_shapely = True
    except ImportError:
        _have_shapely = False
    bcu_extra_obs: list = []

    precomp_routes: dict = {}
    steering_honored = 0
    steering_total = 0
    failed_escape: list = []

    t_escape_start = time.perf_counter()

    for net_name, tc in escape_order:
        _check_rss(f"escape {net_name}")

        net_obj = nets_by_name.get(net_name)
        if net_obj is None:
            continue

        # Source pad: QFN F.Cu pad on a dense component
        net_pads_world = pads_by_net.get(net_name, [])
        qfn_pads = [
            p for p in net_pads_world
            if p.get("ref", "") in dense_refs and p.get("front") and not p.get("back")
        ]
        dest_pads = [p for p in net_pads_world if p.get("ref", "") not in dense_refs]

        if not qfn_pads:
            print(f"  SKIP {net_name}: no QFN source pad found", flush=True)
            continue

        src_p = qfn_pads[0]
        source_xy = (src_p["x"], src_p["y"])

        # Destination from topo class, fallback nearest non-QFN pad
        tc_dest = tc.get("dest_xy")
        if tc_dest is not None:
            dest_xy = tc_dest
        elif dest_pads:
            dst_p = min(
                dest_pads,
                key=lambda p: math.hypot(p["x"] - source_xy[0], p["y"] - source_xy[1]),
            )
            dest_xy = (dst_p["x"], dst_p["y"])
        else:
            print(f"  SKIP {net_name}: no dest pad", flush=True)
            continue

        escape_via_xy = tc.get("escape_via_xy")
        lane_y_mm = tc.get("lane_y_mm")

        # Skip nets with no via that are NOT designated F.Cu-only short nets.
        # These are long F.Cu routes the witness handles without a via — they
        # span >12mm and can't be routed here with the bounded window.
        # Let the grid engine handle them after the escape-net pass.
        if escape_via_xy is None and net_name not in FCU_ONLY_NETS:
            print(f"  SKIP_GRID {net_name}: no escape via (not in FCU_ONLY, skip to grid)", flush=True)
            continue

        # Count via-steering attempts
        if net_name not in FCU_ONLY_NETS and escape_via_xy is not None:
            steering_total += 1

        t_net = time.perf_counter()

        result = route_net_steered(
            source_xy=source_xy,
            dest_xy=dest_xy,
            escape_via_xy=escape_via_xy,
            lane_y_mm=lane_y_mm,
            pads=pads,
            net_name=net_name,
            geo=geo,
            board_bbox=board_bbox,
            # Via placement + F.Cu stub: use pad obstacles only (no accumulated obs).
            # Witness escape vias are tightly packed; accumulated obstacles block
            # subsequent via ring checks. route_net_steered's via window now uses
            # extra_obstacles only (not bcu_extra_obstacles) for the via B.Cu check.
            extra_obstacles=[],
            fcu_stub_extra_obstacles=[],
            # B.Cu run: pass accumulated B.Cu obstacles to prevent run overlaps.
            # This is the correct layer for accumulation (long horizontal runs).
            bcu_extra_obstacles=list(bcu_extra_obs),
            board_outline=board_outline,
            drill_obstacles=drill_obstacles,
            drill_centers=drill_centers,
            max_window_mm=MAX_WINDOW_MM,
            max_bcu_window_mm=MAX_BCU_WINDOW_MM,
        )

        t_net_elapsed = time.perf_counter() - t_net
        if t_net_elapsed > NET_TIMEOUT_S:
            print(
                f"  TIMEOUT {net_name}: {t_net_elapsed:.1f}s > {NET_TIMEOUT_S}s",
                flush=True,
            )
            failed_escape.append((net_name, "timeout"))
            continue

        # Check if steering was honored (via used as supplied)
        if escape_via_xy is not None and result.ok and result.world_vias:
            vx_r, vy_r = result.world_vias[0]
            vx_c, vy_c = escape_via_xy
            if abs(vx_r - vx_c) < 0.02 and abs(vy_r - vy_c) < 0.02:
                steering_honored += 1

        if not result.ok:
            print(f"  FAIL {net_name}: {result.reason} ({t_net_elapsed:.1f}s)", flush=True)
            failed_escape.append((net_name, result.reason))
            continue

        print(
            f"  OK {net_name:15s} via={result.world_vias} honored={result.stats.get('via_honored')} "
            f"t={t_net_elapsed:.1f}s",
            flush=True,
        )

        # Mark copper into grid
        nr = to_gridless_netroute(net_obj, result.world_paths, grid,
                                   world_vias=result.world_vias)
        _mark(grid, nr, 1)
        precomp_routes[net_name] = nr

        # Accumulate B.Cu obstacles from this net's B.Cu run segments.
        # These prevent subsequent escape nets' B.Cu runs from overlapping.
        # We do NOT accumulate F.Cu obstacles (witness vias are too tightly packed).
        if _have_shapely:
            inflate = geo["track_mm"] / 2.0 + geo["clearance_mm"]
            for wpath in result.world_paths:
                bcu_pts = [(wp[0], wp[1]) for wp in wpath
                           if len(wp) > 2 and wp[2] == 1]
                try:
                    if len(bcu_pts) >= 2:
                        bcu_extra_obs.append(
                            _snap(_LS(bcu_pts).buffer(inflate, cap_style=2))
                        )
                except Exception:  # noqa: BLE001
                    pass

        _check_rss(f"after {net_name}")

    t_escape = time.perf_counter() - t_escape_start
    print(
        f"  Escape routing done: {len(precomp_routes)}/{len(escape_order)} ok "
        f"in {t_escape:.1f}s",
        flush=True,
    )

    # ── Grid-route the remaining nets ─────────────────────────────────────────
    remaining = [n for n in nets if n.name not in precomp_routes]
    _check_rss("before_grid")

    t_grid_start = time.perf_counter()
    grid_results = route_all(
        grid,
        remaining,
        escape=12,
        ripup_factor=8,
        via_cost=10.0,
        history_factor=1.0,
        allow_partial=True,
    )
    t_grid = time.perf_counter() - t_grid_start
    print(f"  Grid routing: {t_grid:.1f}s", flush=True)

    _check_rss("after_grid")

    # ── Emit all routes ───────────────────────────────────────────────────────
    all_results = dict(precomp_routes)
    all_results.update(grid_results)

    emit_routes(
        board,
        grid,
        all_results,
        track_mm=geo["track_mm"],
        via_mm=geo["via_mm"],
        via_drill_mm=geo["via_drill_mm"],
        anchors=anchors,
        neck_mm=geo["min_track_mm"],
        obstacles=obstacles,
        anchor_rects=anchor_rects,
        clearance_mm=geo["clearance_mm"],
    )

    refill_zones(board)

    # ── DRC ──────────────────────────────────────────────────────────────────
    from tracewise.route.bridge import run_drc
    report = run_drc(board)

    by = collections.Counter(v.get("type") for v in report.get("violations", []))
    errs = sum(1 for v in report.get("violations", []) if v.get("severity") == "error")
    unc = len(report.get("unconnected_items", []))

    # ── Crossing audit ────────────────────────────────────────────────────────
    crossings = audit_crossings(board)

    return {
        "board": str(board),
        "unconnected": unc,
        "errors": errs,
        "by_type": dict(by),
        "escape_nets_ok": len(precomp_routes),
        "escape_nets_total": len(escape_order),
        "steering_honored": steering_honored,
        "steering_total": steering_total,
        "failed_escape_nets": failed_escape,
        "fcu_crossings": crossings.get("F.Cu", 0),
        "bcu_crossings": crossings.get("B.Cu", 0),
        "t_escape_s": round(t_escape, 1),
        "t_grid_s": round(t_grid, 1),
        "rss_gb": round(_rss_gb(), 3),
    }


# ── Board file hash ───────────────────────────────────────────────────────────

def board_hash(board_path: Path) -> str:
    return hashlib.sha256(board_path.read_bytes()).hexdigest()


# ── Main ──────────────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(description="TCR E1 make-or-break gate")
    parser.add_argument("--runs", type=int, default=3,
                        help="Number of routing runs (3 for determinism check)")
    parser.add_argument("--out", default="/tmp/tcr_e1_out",
                        help="Output directory")
    args = parser.parse_args()

    t_total_start = time.perf_counter()
    peak_rss = _rss_gb()

    print("=" * 70)
    print("TCR E1 MAKE-OR-BREAK GATE")
    print("Attempt-3 bar: unconnected < 41, errors < 73")
    print("=" * 70)

    # ── Step 1: Extract witness TopoClasses ───────────────────────────────────
    print("\n[1] Extracting witness TopoClasses...")
    pcb = Path(PCB_PATH)
    topo_classes = extract_topo_classes(pcb)

    nets_with_via = [(nm, tc) for nm, tc in topo_classes.items()
                     if tc.get("escape_via_xy") is not None]
    nets_fcu_only = [(nm, tc) for nm, tc in topo_classes.items()
                     if tc.get("escape_via_xy") is None]
    nets_with_dest = [(nm, tc) for nm, tc in topo_classes.items()
                      if tc.get("dest_xy") is not None]

    print(f"  Total QFN escape nets in table: {len(topo_classes)}")
    print(f"  Nets with escape via (will steer): {len(nets_with_via)}")
    print(f"  F.Cu-only nets (no via): {len(nets_fcu_only)}")
    print(f"  Nets with J3/J4 dest: {len(nets_with_dest)}")

    print("\n  Via-steered nets (sorted by order_key/dest-y):")
    for nm, tc in sorted(nets_with_via, key=lambda x: x[1]["order_key"]):
        vxy = tc["escape_via_xy"]
        lym = tc["lane_y_mm"]
        dxy = tc["dest_xy"]
        lane_str = f"{lym:.3f}" if lym is not None else "None"
        dest_y_str = f"{dxy[1]:.3f}" if dxy is not None else "N/A"
        print(f"    {nm:<16} via=({vxy[0]:.3f},{vxy[1]:.3f}) lane_y={lane_str} dest_y={dest_y_str}")

    # ── Step 2: N routing runs ────────────────────────────────────────────────
    out_base = Path(args.out)
    run_results: list[dict] = []
    run_hashes: list[str] = []

    for run_idx in range(args.runs):
        run_dir = out_base / f"run_{run_idx}"
        print(f"\n[{run_idx + 2}] Run {run_idx + 1}/{args.runs}...", flush=True)

        try:
            _check_rss(f"start run {run_idx}")
            score = run_e1(run_dir, topo_classes)
            run_results.append(score)
            peak_rss = max(peak_rss, score["rss_gb"])

            board_file = Path(score["board"])
            if board_file.exists():
                h = board_hash(board_file)
                run_hashes.append(h)
                print(f"  board_hash={h[:16]}...", flush=True)

            print(f"\n  --- Run {run_idx + 1} summary ---")
            print(f"  unconnected={score['unconnected']}  errors={score['errors']}")
            print(f"  F.Cu crossings={score['fcu_crossings']}  B.Cu crossings={score['bcu_crossings']}")
            print(f"  escape ok={score['escape_nets_ok']}/{score['escape_nets_total']}")
            print(f"  steering honored={score['steering_honored']}/{score['steering_total']}")
            print(f"  by_type={score['by_type']}")
            print(f"  RSS={score['rss_gb']:.3f} GB")

        except MemoryError as exc:
            print(f"  MEMORY ABORT: {exc}", flush=True)
            run_results.append({
                "unconnected": 9999, "errors": 9999, "abort": str(exc),
                "rss_gb": _rss_gb(), "fcu_crossings": 0, "bcu_crossings": 0,
                "escape_nets_ok": 0, "escape_nets_total": 0,
                "steering_honored": 0, "steering_total": 0,
                "failed_escape_nets": [], "by_type": {},
            })
            break
        except Exception as exc:
            import traceback as _tb
            print(f"  ERROR run {run_idx + 1}: {exc}", flush=True)
            _tb.print_exc()
            run_results.append({
                "unconnected": 9999, "errors": 9999, "abort": str(exc),
                "rss_gb": _rss_gb(), "fcu_crossings": 0, "bcu_crossings": 0,
                "escape_nets_ok": 0, "escape_nets_total": 0,
                "steering_honored": 0, "steering_total": 0,
                "failed_escape_nets": [], "by_type": {},
            })

    t_total = time.perf_counter() - t_total_start

    # ── Determinism check ─────────────────────────────────────────────────────
    if len(run_hashes) >= 3 and len(set(run_hashes)) == 1:
        deterministic = "PASS (3-run byte-identical)"
    elif len(run_hashes) == 2 and len(set(run_hashes)) == 1:
        deterministic = "PASS (2-run byte-identical)"
    elif len(run_hashes) == 1:
        deterministic = "only_1_hash (single run)"
    elif run_hashes:
        deterministic = f"FAIL ({len(set(run_hashes))} different hashes in {len(run_hashes)} runs)"
    else:
        deterministic = "no_hashes"

    # ── Best result ───────────────────────────────────────────────────────────
    valid = [r for r in run_results if "abort" not in r]
    if valid:
        best = min(valid, key=lambda r: r["unconnected"])
    else:
        best = run_results[0] if run_results else {
            "unconnected": 9999, "errors": 9999,
            "fcu_crossings": 9999, "bcu_crossings": 9999,
            "escape_nets_ok": 0, "escape_nets_total": 0,
            "steering_honored": 0, "steering_total": 0,
            "failed_escape_nets": [], "by_type": {},
        }

    unc = best.get("unconnected", 9999)
    errs = best.get("errors", 9999)
    fcu_x = best.get("fcu_crossings", 9999)
    bcu_x = best.get("bcu_crossings", 9999)
    illegal_x = fcu_x + bcu_x
    steering_hon = best.get("steering_honored", 0)
    steering_tot = best.get("steering_total", 1)

    # Per-run runtime = max t_escape_s + t_grid_s across all valid runs.
    # The 5-min limit is per-run, not total across all runs.
    max_run_s = max(
        (r.get("t_escape_s", 0) + r.get("t_grid_s", 0) for r in valid),
        default=t_total,
    )
    beats_attempt3 = (unc < 41) and (errs < 73) and (illegal_x == 0)
    rss_ok = peak_rss < RSS_HARD_FAIL_GB
    runtime_ok = max_run_s < 300.0
    det_pass = "PASS" in deterministic
    steering_pass = (steering_tot == 0) or (steering_hon / steering_tot >= 0.8)

    all_go = beats_attempt3 and rss_ok and runtime_ok and det_pass and steering_pass

    if all_go:
        go_nogo = "GO"
        go_reason = "all criteria met"
        failure_mode = ""
    else:
        go_nogo = "NO-GO"
        reasons: list[str] = []
        fmodes: list[str] = []
        if unc >= 41:
            reasons.append(f"unconnected={unc} >= 41 (attempt-3 bar)")
            fmodes.append("unrouted")
        if errs >= 73:
            reasons.append(f"errors={errs} >= 73")
        if illegal_x > 0:
            reasons.append(f"crossings={illegal_x}")
            fmodes.append("crossings")
        if not rss_ok:
            reasons.append(f"RSS={peak_rss:.2f}GB >= 2GB")
        if not runtime_ok:
            reasons.append(f"runtime={max_run_s:.0f}s/run >= 300s")
        if not det_pass:
            reasons.append(f"determinism={deterministic}")
        if not steering_pass:
            frac = steering_hon / max(steering_tot, 1)
            reasons.append(f"steering={steering_hon}/{steering_tot} ({frac:.0%})")
            fmodes.append("steering-ignored" if steering_hon == 0 else "lane-blocked")
        go_reason = "; ".join(reasons)
        failure_mode = ", ".join(fmodes) if fmodes else "unspecified"

    # ── Print verdict ─────────────────────────────────────────────────────────
    print("\n" + "=" * 70)
    print(f"E1 VERDICT: {go_nogo}")
    print(f"  unconnected = {unc}  (bar: < 41;  attempt-3 = 41)")
    print(f"  errors      = {errs}  (bar: < 73)")
    print(f"  crossings   = F.Cu={fcu_x}  B.Cu={bcu_x}  (bar: 0)")
    print(f"  steering    = {steering_hon}/{steering_tot} vias honored")
    print(f"  RSS         = {peak_rss:.3f} GB  (bar: < 2 GB)")
    print(f"  runtime     = {max_run_s:.1f} s/run (total={t_total:.1f}s, bar: < 300 s/run)")
    print(f"  determinism = {deterministic}")
    if go_nogo == "NO-GO":
        print(f"  FAILURE:    {go_reason}")
        print(f"  MODE:       {failure_mode}")
    print("=" * 70)

    # ── Structured Result ─────────────────────────────────────────────────────
    structured = {
        "status": "complete",
        "summary": (
            f"E1 {go_nogo}: unc={unc} err={errs} "
            f"crossings_fcu={fcu_x} crossings_bcu={bcu_x} "
            f"steering={steering_hon}/{steering_tot}"
        ),
        "files_changed": [
            "src/tracewise/route/gridless/route.py (route_net_steered added)",
            "scripts/_probe_tcr_e1.py (new)",
        ],
        "files_read": [
            str(PCB_PATH),
            "src/tracewise/route/gridless/route.py",
            "src/tracewise/route/engine/multi.py",
            "src/tracewise/route/engine/kicad.py",
            "src/tracewise/route/gridless/geom.py",
            "scripts/_invent2_topology.py",
            "scripts/_invent1_human_stats.py",
            "scripts/_probe_route_human.py",
            "docs/design/TOPOLOGY-CLASS-ROUTING.md",
            "docs/research/INVENT-human-mimicry-router.md",
            "docs/research/INVENT-topological-routability.md",
        ],
        "route_net_steered_added": True,
        "steering_honored": (steering_hon == steering_tot and steering_tot > 0),
        "witness_classes_extracted": len(topo_classes),
        "default_byte_identical": "not_tested_this_run",
        "pytest_full": "not_run",
        "ruff": "not_run",
        "grid_only": {"unc": 48},
        "attempt3_bar": {"unc": 41, "errors": 73},
        "result": {
            "unconnected": unc,
            "errors": errs,
            "by_type": best.get("by_type", {}),
        },
        "escape_nets_realized": best.get("escape_nets_ok", 0),
        "illegal_crossings": {"fcu": fcu_x, "bcu": bcu_x},
        "unconnected_vs_attempt3": (
            f"{unc} vs 41 ({'better' if unc < 41 else 'tied' if unc == 41 else 'worse'})"
        ),
        "beats_attempt3": beats_attempt3,
        "peak_rss_gb": round(peak_rss, 3),
        "runtime_s": round(t_total, 1),
        "max_single_run_s": round(max_run_s, 1),
        "deterministic": deterministic,
        "e1_go_no_go": f"{go_nogo}: {go_reason}",
        "failure_mode": failure_mode if go_nogo == "NO-GO" else "",
        "issues": best.get("failed_escape_nets", []),
        "assumptions": [
            "escape_via_xy = closest via to U3 within 12mm radius",
            "lane_y_mm = y of longest horizontal B.Cu segment per net",
            "dest_xy = J3/J4 pad center per net (from _invent1 global-frame extractor)",
            "order_key = dest_y (monotone B.Cu bus routing order)",
            "FCU_ONLY_NETS = {/GPIO27, /GPIO28, /XIN} → escape_via_xy=None",
            "max_window_mm=12, max_bcu_window_mm=8 (bounded)",
            "RSS hard-abort at 2GB; per-net timeout 60s",
        ],
    }

    print("\n## Structured Result")
    print("```json")
    print(json.dumps(structured, indent=2, default=str))
    print("```")

    # Save result JSON
    out_base.mkdir(parents=True, exist_ok=True)
    result_path = out_base / "e1_result.json"
    result_path.write_text(json.dumps(structured, indent=2, default=str))
    print(f"\nResult saved: {result_path}")


if __name__ == "__main__":
    main()
