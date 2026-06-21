"""Spike-CSRU2: Cross-Substrate Rip-Up with multi-pin rerouting (M3-P2 upgrade).

Prove that cross-substrate rip-up yields a real connectivity gain on the mitayi
board baseline (48 unconnected / 89 errors), NOW THAT route_net_multipin exists.

The original CSRU (spike_csru_cross_substrate_ripup.py) was NO-GO because
rerouting the displaced multi-pin blockers wasn't possible (only 2-pin). M3-P2
adds route_net_multipin — this spike uses it for blocker rerouting.

Algorithm:
  1. Route mitayi grid-only (defaults) → capture baseline + unconnected signal nets.
  2. For each boxed-in signal target (deterministic order: QSPI first, then others):
     a. Confirm route FAILS with full grid copper as obstacles (boxed-in check).
     b. Identify blocking grid nets (BOUNDED window — pad-span based, NOT board-wide).
     c. Rip blockers; re-attempt target route → should succeed.
     d. Reroute ripped blockers with route_net_multipin (BOUNDED windows).
     e. Accept ONLY if net-positive (targets connected >= blockers that failed reroute).
  3. Emit final routing; refill_zones; real kicad DRC.
  4. Run twice; assert deterministic.

Honesty mandate: report REAL numbers. NO-GO is a legitimate outcome.
"""
from __future__ import annotations

import collections
import json
import math
import re
import shutil
import sys
import time
from pathlib import Path

# Shapely guard
try:
    import shapely
    from shapely.geometry import LineString, Point as SPoint, box
    from shapely.ops import unary_union
    from shapely import set_precision
    GEOS_VERSION = shapely.geos_version
    print(f"[csru2] Shapely {shapely.__version__}  GEOS {GEOS_VERSION}", flush=True)
except ImportError as exc:
    print(f"ERROR: Shapely not installed: {exc}", file=sys.stderr)
    sys.exit(1)

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "src"))

from tracewise.route.bridge import run_drc, strip_routing
from tracewise.route.engine.kicad import (
    build_problem,
    extract_pads,
    project_geometry,
    refill_zones,
    emit_routes,
)
from tracewise.route.engine.multi import route_all, _mark, order_nets, Net, NetRoute
from tracewise.route.gridless.geom import (
    build_windowed_free_space,
    extract_board_outline,
    extract_drill_obstacles,
    extract_drill_centers,
    net_routes_to_track_obstacles,
    snap,
)
from tracewise.route.gridless.route import (
    route_net_gridless,
    route_net_multipin,
    _route_net_2layer,
    GridlessRouteResult,
)
from tracewise.route.gridless.adapter import to_gridless_netroute, GridlessNetRoute

BOARD_SRC = ROOT / "data/benchmark-boards/mitayi-pico-d1/Mitayi-Pico-D1.kicad_pcb"
SUFFIXES = (".kicad_pcb", ".kicad_sch", ".kicad_pro", ".kicad_prl")

# Power net hints — these are pour-coverage class, not routing class
POWER_HINTS = ("GND", "+3V3", "+1V1", "VBUS", "VCC", "+5V", "VDD", "VSS", "PWR")

# The 17 signal targets to attempt rescue (deterministic order: QSPI first, then others)
# Per verified residual analysis from _verify_m3p2_residual.py
SIGNAL_RESCUE_TARGETS_ORDERED = [
    "/QSPI_SCLK",
    "/QSPI_SD2",
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
    "Net-(U3-USB-DP)",
]

# BOUNDED window parameters — must NOT use board-wide windows
WINDOW_INITIAL_MM = 8.0       # initial pad-span-based window for targets
WINDOW_BLOCKER_MM = 6.0       # initial window for blocker rerouting
WINDOW_EXPAND_MM = 3.0        # expansion for identifying blocking nets
MAX_WINDOW_MM = 25.0          # hard cap — prevents board-wide blowup


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


def drc_counts(report: dict) -> dict:
    errs = [v for v in report.get("violations", []) if v.get("severity") == "error"]
    by = collections.Counter(v.get("type") for v in errs)
    unc = len(report.get("unconnected_items", []))
    unc_nets = _unconnected_nets(report)
    return {
        "unconnected": unc,
        "errors": len(errs),
        "by_type": dict(by),
        "unconnected_nets": unc_nets,
        "hole_clearance": by.get("hole_clearance", 0),
        "hole_to_hole": by.get("hole_to_hole", 0),
    }


def is_power_net(net_name: str, pin_count: int) -> bool:
    """Returns True if net is a power/pour class net (not a routing target)."""
    return any(h in net_name.upper() for h in POWER_HINTS) or pin_count >= 10


def get_pads_for_net(net_name: str, all_pads: list[dict]) -> list[dict]:
    """Return all pads belonging to the given net."""
    return [p for p in all_pads if p.get("net") == net_name]


def build_track_obstacles_excluding(
    results: dict,
    exclude_nets: set[str],
    grid,
    geo: dict,
) -> list:
    """Build Shapely track obstacles from all routed nets EXCEPT excluded set."""
    filtered = {k: v for k, v in results.items() if k not in exclude_nets}
    return net_routes_to_track_obstacles(
        filtered, grid, geo["track_mm"], geo["clearance_mm"]
    )


def identify_blocking_nets_bounded(
    target_net_name: str,
    target_pads: list[dict],
    results: dict,
    grid,
    geo: dict,
    window_expand_mm: float = WINDOW_EXPAND_MM,
) -> list[str]:
    """Find grid nets whose copper intersects the target's BOUNDED routing window.

    Window is BOUNDED to pad-span + expand_mm — NOT board-wide.
    Returns sorted list of blocking net names (deterministic).
    """
    if not target_pads:
        return []

    xs = [p["x"] for p in target_pads]
    ys = [p["y"] for p in target_pads]
    x_lo = min(xs) - window_expand_mm
    y_lo = min(ys) - window_expand_mm
    x_hi = max(xs) + window_expand_mm
    y_hi = max(ys) + window_expand_mm
    window_poly = snap(box(x_lo, y_lo, x_hi, y_hi))

    # Build corridor(s) between nearest-pair pad combinations
    corridors = []
    inflate = geo["track_mm"] + geo["clearance_mm"] * 2
    if len(target_pads) >= 2:
        for i in range(len(target_pads)):
            for j in range(i + 1, len(target_pads)):
                pa = (target_pads[i]["x"], target_pads[i]["y"])
                pb = (target_pads[j]["x"], target_pads[j]["y"])
                try:
                    cor = snap(LineString([pa, pb]).buffer(inflate, cap_style=2))
                    corridors.append(cor)
                except Exception:
                    pass

    from tracewise.route.engine.astar import simplify

    blocking = []
    inflate_obs = geo["track_mm"] / 2.0 + geo["clearance_mm"]

    for net_name, nr in results.items():
        if net_name == target_net_name:
            continue
        if not nr.ok:
            continue
        if isinstance(nr, GridlessNetRoute):
            # For GridlessNetRoutes, use world_paths directly
            if not nr.world_paths:
                continue
            net_blocks = False
            for wpath in nr.world_paths:
                if net_blocks:
                    break
                if len(wpath) < 2:
                    continue
                try:
                    pts_2d = [(p[0], p[1]) for p in wpath]
                    seg = snap(LineString(pts_2d).buffer(inflate_obs, cap_style=2))
                    if seg.intersects(window_poly):
                        net_blocks = True
                    if not net_blocks:
                        for cor in corridors:
                            if seg.intersects(cor):
                                net_blocks = True
                                break
                except Exception:
                    pass
            if net_blocks:
                blocking.append(net_name)
            continue

        # Grid NetRoute: paths are (layer, iy, ix) cell tuples
        if not nr.paths:
            continue
        net_blocks = False
        for path in nr.paths:
            if net_blocks:
                break
            runs = simplify(path)
            for run in runs:
                if len(run) < 2:
                    continue
                world_pts = [grid.to_world(c[1], c[2]) for c in run]
                if len(world_pts) < 2:
                    continue
                try:
                    seg = snap(LineString(world_pts).buffer(inflate_obs, cap_style=2))
                    if seg.intersects(window_poly):
                        net_blocks = True
                        break
                    for cor in corridors:
                        if seg.intersects(cor):
                            net_blocks = True
                            break
                    if net_blocks:
                        break
                except Exception:
                    pass

        if net_blocks:
            blocking.append(net_name)

    return sorted(blocking)


def try_route_target(
    net_name: str,
    target_pads: list[dict],
    all_pads: list[dict],
    geo: dict,
    board_bbox: tuple[float, float, float, float],
    extra_obstacles: list,
    board_outline,
    drill_obstacles: list,
    drill_centers: list,
    window_mm: float = WINDOW_INITIAL_MM,
    label: str = "",
) -> GridlessRouteResult:
    """Route a target net (2-pin or multi-pin) with BOUNDED window.

    Uses route_net_multipin for multi-pin (which handles 2-pin via fast path).
    Caps window at MAX_WINDOW_MM to prevent board-wide blowup.
    """
    t0 = time.perf_counter()
    capped_window = min(window_mm, MAX_WINDOW_MM)

    result = route_net_multipin(
        pads_of_net=target_pads,
        net_name=net_name,
        all_pads=all_pads,
        geo=geo,
        board_bbox=board_bbox,
        extra_obstacles=extra_obstacles,
        window_mm=capped_window,
        board_outline=board_outline,
        drill_obstacles=drill_obstacles,
        drill_centers=drill_centers,
    )
    elapsed = time.perf_counter() - t0
    tag = label or net_name
    if result.ok:
        print(f"[csru2]   {tag}: route SUCCEEDED in {elapsed:.2f}s "
              f"(pads={len(target_pads)}, window={capped_window:.1f}mm)", flush=True)
    else:
        print(f"[csru2]   {tag}: route FAILED in {elapsed:.2f}s "
              f"({result.reason}) window={capped_window:.1f}mm", flush=True)
    return result


def reroute_blocker_bounded(
    net_name: str,
    blocker_pads: list[dict],
    all_pads: list[dict],
    geo: dict,
    board_bbox: tuple[float, float, float, float],
    extra_obstacles: list,
    board_outline,
    drill_obstacles: list,
    drill_centers: list,
    window_mm: float = WINDOW_BLOCKER_MM,
    max_pads: int = 6,
) -> GridlessRouteResult:
    """Reroute a ripped blocker net using route_net_multipin with BOUNDED window.

    This is the key upgrade from CSRU v1: multi-pin blockers can now be rerouted.

    max_pads: if the blocker has more pads than this, it is a power/complex net
    that cannot be rerouted by gridless traces → immediately return failure.
    Power nets (GND/+3V3/VBUS with 5-59 pads) always fail rerouting and are
    pour-coverage class. Skipping them fast prevents pathological slowness.
    """
    # Fast-path failure for power nets / very high-pin-count nets
    if is_power_net(net_name, len(blocker_pads)) or len(blocker_pads) > max_pads:
        print(f"[csru2]     {net_name}: SKIP reroute (power/high-pin: "
              f"{len(blocker_pads)} pads > max_pads={max_pads})", flush=True)
        return GridlessRouteResult(
            ok=False,
            reason=f"power/high-pin-count net ({len(blocker_pads)} pads) — cannot reroute with traces"
        )

    capped_window = min(window_mm, MAX_WINDOW_MM)
    t0 = time.perf_counter()

    result = route_net_multipin(
        pads_of_net=blocker_pads,
        net_name=net_name,
        all_pads=all_pads,
        geo=geo,
        board_bbox=board_bbox,
        extra_obstacles=extra_obstacles,
        window_mm=capped_window,
        board_outline=board_outline,
        drill_obstacles=drill_obstacles,
        drill_centers=drill_centers,
    )
    elapsed = time.perf_counter() - t0
    if result.ok:
        print(f"[csru2]     {net_name}: blocker rerouted OK in {elapsed:.2f}s "
              f"(pads={len(blocker_pads)}, window={capped_window:.1f}mm)", flush=True)
    else:
        print(f"[csru2]     {net_name}: blocker reroute FAILED in {elapsed:.2f}s "
              f"({result.reason}) pads={len(blocker_pads)}", flush=True)
    return result


def build_copper_obstacles_from_paths(
    world_paths: list[list[tuple]],
    world_vias: list[tuple[float, float]],
    geo: dict,
) -> list:
    """Build Shapely obstacles from world_paths (newly-placed copper)."""
    inflate = geo["track_mm"] / 2.0 + geo["clearance_mm"]
    via_inflate = geo.get("via_mm", 0.6) / 2.0 + geo["clearance_mm"]
    obstacles = []
    for wpath in world_paths:
        if len(wpath) < 2:
            continue
        pts_2d = [(p[0], p[1]) for p in wpath]
        if len(pts_2d) < 2:
            continue
        try:
            ls = snap(LineString(pts_2d).buffer(inflate, cap_style=2))
            obstacles.append(ls)
        except Exception:
            pass
    for vx, vy in (world_vias or []):
        try:
            circle = snap(SPoint(vx, vy).buffer(via_inflate, resolution=16))
            obstacles.append(circle)
        except Exception:
            pass
    return obstacles


def run_csru2(out_dir: Path, run_label: str = "run1") -> dict:
    """Run the full CSRU2 spike. Returns a result dict."""
    t_start = time.perf_counter()
    per_net_runtimes: dict[str, float] = {}

    print(f"\n[csru2] === CSRU2 {run_label} ===", flush=True)

    # -------------------------------------------------------------------------
    # Step 1: Setup board + grid-only route
    # -------------------------------------------------------------------------
    board = setup_board(out_dir)
    print(f"[csru2] Board: {board}", flush=True)

    data = extract_pads(board)
    geo = project_geometry(board)
    print(f"[csru2] geo={geo}", flush=True)

    bd = data["board"]
    board_bbox = (bd["x1"], bd["y1"], bd["x2"], bd["y2"])
    board_outline = extract_board_outline(board)
    drill_obstacles = extract_drill_obstacles(
        board, clearance_mm=geo["clearance_mm"], track_mm=geo["track_mm"]
    )
    drill_centers = extract_drill_centers(board)
    print(f"[csru2] drill_centers: {len(drill_centers)}, "
          f"drill_obstacles: {len(drill_obstacles)}", flush=True)

    # Build grid problem
    grid, nets, anchors, obstacles_raw, anchor_rects = build_problem(
        data, track_mm=geo["track_mm"], clearance_mm=geo["clearance_mm"]
    )
    via_half = max(1, math.ceil(
        (geo["via_mm"] / 2 + geo["clearance_mm"] + geo["track_mm"] / 2) / 0.1))
    for n in nets:
        n.via_halfwidth_cells = via_half

    by_name = {n.name: n for n in nets}

    # Pin counts per net (for power classification)
    pin_count: collections.Counter = collections.Counter()
    for p in data["pads"]:
        if p.get("net"):
            pin_count[p["net"]] += 1

    # Route grid-only (all nets, default parameters)
    print("[csru2] Step 1: Grid-only route (all nets)...", flush=True)
    t_grid = time.perf_counter()
    results_grid = route_all(
        grid, nets, escape=12, ripup_factor=8, via_cost=10.0,
        history_factor=1.0, allow_partial=True, salvage_escape=0,
    )
    t_grid_done = time.perf_counter()
    print(f"[csru2] Grid route done in {t_grid_done - t_grid:.2f}s", flush=True)

    # Emit grid routes to board
    emit_routes(
        board, grid, results_grid,
        track_mm=geo["track_mm"],
        via_mm=geo["via_mm"],
        via_drill_mm=geo["via_drill_mm"],
        anchors=anchors,
        neck_mm=geo["min_track_mm"],
        obstacles=obstacles_raw,
        anchor_rects=anchor_rects,
        clearance_mm=geo["clearance_mm"],
    )
    refill_zones(board)

    # DRC on grid-only
    print("[csru2] Running DRC on grid-only...", flush=True)
    rep_grid = run_drc(board)
    counts_grid = drc_counts(rep_grid)
    print(f"[csru2] Grid-only: unconnected={counts_grid['unconnected']}, "
          f"errors={counts_grid['errors']}, "
          f"hole_clearance={counts_grid['hole_clearance']}, "
          f"hole_to_hole={counts_grid['hole_to_hole']}", flush=True)

    if counts_grid["unconnected"] != 48:
        print(f"[csru2] WARNING: baseline unconnected={counts_grid['unconnected']}, "
              f"expected 48 — proceeding but reporting this discrepancy.", flush=True)

    # Identify which signal nets are still unconnected after grid-only
    unconnected_nets_grid = counts_grid["unconnected_nets"]
    signal_unconnected = [
        n for n in unconnected_nets_grid
        if not is_power_net(n, pin_count.get(n, 0))
    ]
    print(f"[csru2] Signal nets unconnected after grid: {sorted(signal_unconnected)}",
          flush=True)

    # -------------------------------------------------------------------------
    # Step 2: Build all grid track obstacles
    # -------------------------------------------------------------------------
    print("\n[csru2] Building grid track obstacles...", flush=True)
    all_track_obs = net_routes_to_track_obstacles(
        results_grid, grid, geo["track_mm"], geo["clearance_mm"]
    )
    print(f"[csru2] Total grid track obstacles: {len(all_track_obs)}", flush=True)

    # -------------------------------------------------------------------------
    # Step 3: CSRU2 — For each boxed-in signal target
    # -------------------------------------------------------------------------
    print("\n[csru2] === Step 3: CSRU2 Cross-Substrate Rip-Up ===", flush=True)

    # Determine the ordered set of targets to attempt
    # Only attempt nets that are actually unconnected after grid routing
    targets_to_attempt = [
        n for n in SIGNAL_RESCUE_TARGETS_ORDERED
        if n in unconnected_nets_grid
    ]
    # Also include any signal unconnected nets not in our ordered list (fallback)
    for n in sorted(signal_unconnected):
        if n not in targets_to_attempt:
            targets_to_attempt.append(n)

    print(f"[csru2] Targets to attempt ({len(targets_to_attempt)}): "
          f"{targets_to_attempt}", flush=True)

    # Track state
    targets_attempted: list[str] = []
    targets_connected: list[str] = []
    blockers_ripped: list[str] = []
    blockers_failed_reroute: list[str] = []
    boxed_in_confirmed: list[str] = []  # nets that confirmed boxed-in

    # Working copy of results
    working_results = dict(results_grid)

    # Accumulate newly-placed copper obstacles as targets are connected
    placed_copper_obstacles: list = []

    for target_net in targets_to_attempt:
        t_net_start = time.perf_counter()
        target_pads = get_pads_for_net(target_net, data["pads"])
        if not target_pads:
            print(f"[csru2] {target_net}: no pads found — skip", flush=True)
            continue

        print(f"\n[csru2] --- Attempting {target_net} ({len(target_pads)} pads) ---",
              flush=True)
        targets_attempted.append(target_net)

        # Step 3a: Try route WITH all grid copper as obstacles → confirm boxed-in
        print(f"[csru2]   3a: Try WITH full grid copper...", flush=True)
        result_with_copper = try_route_target(
            net_name=target_net,
            target_pads=target_pads,
            all_pads=data["pads"],
            geo=geo,
            board_bbox=board_bbox,
            extra_obstacles=all_track_obs + placed_copper_obstacles,
            board_outline=board_outline,
            drill_obstacles=drill_obstacles,
            drill_centers=drill_centers,
            window_mm=WINDOW_INITIAL_MM,
            label=f"{target_net} WITH grid copper",
        )

        if result_with_copper.ok:
            # Not boxed-in — accept this as a free connection
            print(f"[csru2]   {target_net}: routes WITHOUT rip-up (not boxed-in)!",
                  flush=True)
            net_obj = by_name.get(target_net)
            if net_obj:
                nr = to_gridless_netroute(
                    net_obj,
                    result_with_copper.world_paths,
                    grid,
                    world_vias=result_with_copper.world_vias,
                )
                working_results[target_net] = nr
                targets_connected.append(target_net)
                # Add new copper as obstacles for subsequent targets
                new_obs = build_copper_obstacles_from_paths(
                    result_with_copper.world_paths, result_with_copper.world_vias, geo
                )
                placed_copper_obstacles.extend(new_obs)
            per_net_runtimes[target_net] = round(time.perf_counter() - t_net_start, 3)
            continue

        boxed_in_confirmed.append(target_net)

        # Step 3b: Identify blocking grid nets (BOUNDED window)
        print(f"[csru2]   3b: Identifying blockers (bounded window "
              f"+{WINDOW_EXPAND_MM}mm)...", flush=True)
        blocking_nets = identify_blocking_nets_bounded(
            target_net, target_pads, working_results, grid, geo,
            window_expand_mm=WINDOW_EXPAND_MM,
        )
        print(f"[csru2]   Blocking nets ({len(blocking_nets)}): {blocking_nets}",
              flush=True)

        if not blocking_nets:
            print(f"[csru2]   {target_net}: no blockers identified — "
                  f"geometry may be fundamental — skip", flush=True)
            per_net_runtimes[target_net] = round(time.perf_counter() - t_net_start, 3)
            continue

        # *** EARLY EXIT: if any blocker is power/high-pin, swap is net-negative ***
        # Power nets (GND, +3V3, VBUS etc.) and nets with >6 pads CANNOT be rerouted
        # with gridless traces — they are pour-coverage class. Any swap touching these
        # is immediately net-negative. Skip step 3c (the expensive route attempt)
        # to avoid pathological slowness (~160s per net via board-wide via search).
        power_blocker_failures_pre = [
            b for b in blocking_nets
            if is_power_net(b, len(get_pads_for_net(b, data["pads"]))) or
            len(get_pads_for_net(b, data["pads"])) > 6
        ]
        if len(power_blocker_failures_pre) >= 1:
            print(f"[csru2]   3b-EARLY-EXIT: {len(power_blocker_failures_pre)} power/high-pin "
                  f"blockers: {power_blocker_failures_pre}. "
                  f"Swap guaranteed net-negative — skip (no route attempt).", flush=True)
            per_net_runtimes[target_net] = round(time.perf_counter() - t_net_start, 3)
            continue

        # Step 3c: Rip blockers, re-attempt target route
        print(f"[csru2]   3c: Ripping {len(blocking_nets)} blockers, "
              f"re-attempting target route...", flush=True)
        track_obs_no_blockers = build_track_obstacles_excluding(
            working_results,
            exclude_nets=set(blocking_nets) | {target_net},
            grid=grid,
            geo=geo,
        )
        # Also include already-placed CSRU copper (from previous targets)
        print(f"[csru2]   Track obstacles without blockers: "
              f"{len(track_obs_no_blockers)} (was {len(all_track_obs)})", flush=True)

        result_after_ripup = try_route_target(
            net_name=target_net,
            target_pads=target_pads,
            all_pads=data["pads"],
            geo=geo,
            board_bbox=board_bbox,
            extra_obstacles=track_obs_no_blockers + placed_copper_obstacles,
            board_outline=board_outline,
            drill_obstacles=drill_obstacles,
            drill_centers=drill_centers,
            window_mm=WINDOW_INITIAL_MM,
            label=f"{target_net} AFTER rip-up",
        )

        if not result_after_ripup.ok:
            print(f"[csru2]   {target_net}: STILL FAILS after rip-up! Skip.", flush=True)
            per_net_runtimes[target_net] = round(time.perf_counter() - t_net_start, 3)
            continue

        # Step 3d: Reroute the ripped blockers with route_net_multipin (BOUNDED)
        # All blockers here are signal nets ≤6 pads — power nets excluded at 3b.
        print(f"[csru2]   3d: Rerouting {len(blocking_nets)} signal blockers with "
              f"route_net_multipin...", flush=True)

        # Target copper obstacles (just placed)
        target_copper_obs = build_copper_obstacles_from_paths(
            result_after_ripup.world_paths, result_after_ripup.world_vias, geo
        )
        # Combined obstacles for blocker rerouting: everything except the blocker itself
        # + target copper
        base_obs_for_blockers = track_obs_no_blockers + placed_copper_obstacles + target_copper_obs

        newly_failed: list[str] = []
        successfully_rerouted: list[str] = []
        rerouted_nrs: dict[str, GridlessNetRoute] = {}
        cumulative_reroute_obs = list(base_obs_for_blockers)

        for blocker_name in blocking_nets:
            blocker_pads = get_pads_for_net(blocker_name, data["pads"])
            print(f"[csru2]     Rerouting blocker {blocker_name} "
                  f"({len(blocker_pads)} pads)...", flush=True)

            if not blocker_pads:
                print(f"[csru2]     {blocker_name}: no pads found — mark as failed",
                      flush=True)
                newly_failed.append(blocker_name)
                continue

            res_blocker = reroute_blocker_bounded(
                net_name=blocker_name,
                blocker_pads=blocker_pads,
                all_pads=data["pads"],
                geo=geo,
                board_bbox=board_bbox,
                extra_obstacles=cumulative_reroute_obs,
                board_outline=board_outline,
                drill_obstacles=drill_obstacles,
                drill_centers=drill_centers,
                window_mm=WINDOW_BLOCKER_MM,
            )

            if res_blocker.ok:
                net_obj = by_name.get(blocker_name)
                if net_obj:
                    nr = to_gridless_netroute(
                        net_obj,
                        res_blocker.world_paths,
                        grid,
                        world_vias=res_blocker.world_vias,
                    )
                    successfully_rerouted.append(blocker_name)
                    rerouted_nrs[blocker_name] = nr
                    # Add blocker's new copper to cumulative obstacles
                    new_obs = build_copper_obstacles_from_paths(
                        res_blocker.world_paths, res_blocker.world_vias, geo
                    )
                    cumulative_reroute_obs.extend(new_obs)
                else:
                    newly_failed.append(blocker_name)
            else:
                newly_failed.append(blocker_name)

        # Step 3e: Accept swap ONLY if net-positive
        target_gain = 1  # one target net connected
        blocker_loss = len(newly_failed)
        net_positive = target_gain > blocker_loss  # strictly positive

        print(f"[csru2]   Net swap: +{target_gain} target, "
              f"-{blocker_loss} blockers failed reroute → "
              f"{'NET-POSITIVE' if net_positive else 'NET-NEUTRAL/NEGATIVE'}", flush=True)

        if not net_positive:
            print(f"[csru2]   {target_net}: REJECTING swap "
                  f"(loss={blocker_loss} >= gain={target_gain})", flush=True)
            per_net_runtimes[target_net] = round(time.perf_counter() - t_net_start, 3)
            continue

        # Accept the swap
        print(f"[csru2]   {target_net}: ACCEPTING swap (+1 target, "
              f"-{blocker_loss} blockers)", flush=True)
        blockers_ripped.extend(blocking_nets)
        targets_connected.append(target_net)
        blockers_failed_reroute.extend(newly_failed)

        # Update working_results: target net
        net_obj = by_name.get(target_net)
        if net_obj:
            nr = to_gridless_netroute(
                net_obj,
                result_after_ripup.world_paths,
                grid,
                world_vias=result_after_ripup.world_vias,
            )
            working_results[target_net] = nr

        # Update working_results: rerouted blockers (success) + failed blockers
        for blocker_name in blocking_nets:
            if blocker_name in rerouted_nrs:
                working_results[blocker_name] = rerouted_nrs[blocker_name]
            else:
                # Ripped but failed reroute → mark as failed
                blocker_net_obj = by_name.get(blocker_name)
                if blocker_net_obj:
                    working_results[blocker_name] = NetRoute(
                        net=blocker_net_obj, ok=False,
                        reason="ripped-by-csru2-failed-reroute"
                    )

        # Update placed_copper_obstacles: target + successfully-rerouted blockers
        placed_copper_obstacles.extend(target_copper_obs)
        for blocker_name, nr in rerouted_nrs.items():
            new_obs = build_copper_obstacles_from_paths(
                nr.world_paths, nr.world_vias, geo
            )
            placed_copper_obstacles.extend(new_obs)

        per_net_runtimes[target_net] = round(time.perf_counter() - t_net_start, 3)

    # -------------------------------------------------------------------------
    # Step 4: Emit final routing and DRC
    # -------------------------------------------------------------------------
    print(f"\n[csru2] === Step 4: Emit final routing ===", flush=True)

    final_board_dir = out_dir / "final"
    final_board = setup_board(final_board_dir)

    emit_routes(
        final_board, grid, working_results,
        track_mm=geo["track_mm"],
        via_mm=geo["via_mm"],
        via_drill_mm=geo["via_drill_mm"],
        anchors=anchors,
        neck_mm=geo["min_track_mm"],
        obstacles=obstacles_raw,
        anchor_rects=anchor_rects,
        clearance_mm=geo["clearance_mm"],
    )
    refill_zones(final_board)

    print("[csru2] Running final DRC...", flush=True)
    rep_final = run_drc(final_board)
    counts_final = drc_counts(rep_final)
    print(f"[csru2] Final: unconnected={counts_final['unconnected']}, "
          f"errors={counts_final['errors']}, "
          f"hole_clearance={counts_final['hole_clearance']}, "
          f"hole_to_hole={counts_final['hole_to_hole']}", flush=True)

    # -------------------------------------------------------------------------
    # Determinism fingerprint: extract target+blocker net coords
    # -------------------------------------------------------------------------
    from tracewise.sexpr import parse_file as _parse
    root = _parse(final_board)
    coord_lines = []
    net_decls = {}
    for n in root.nodes("net"):
        num = n.arg(1)
        name = n.arg(2)
        if num is not None and name is not None:
            net_decls[name] = num

    # Include changed nets (targets + blockers ripped)
    changed_nets = set(targets_connected) | set(blockers_ripped)
    for cnet in sorted(changed_nets):
        net_num = net_decls.get(cnet)
        for seg in root.nodes("segment"):
            for child in seg.nodes("net"):
                num_val = child.arg(1)
                if (net_num and num_val == net_num) or num_val == cnet:
                    start = seg.first("start")
                    end_ = seg.first("end")
                    if start and end_:
                        coord_lines.append(
                            f"seg:{cnet}:{start.arg(1)},{start.arg(2)}"
                            f"-{end_.arg(1)},{end_.arg(2)}"
                        )
        for via in root.nodes("via"):
            for child in via.nodes("net"):
                num_val = child.arg(1)
                if (net_num and num_val == net_num) or num_val == cnet:
                    at = via.first("at")
                    if at:
                        coord_lines.append(f"via:{cnet}:{at.arg(1)},{at.arg(2)}")

    coord_lines.sort()
    emitted_coords = "\n".join(coord_lines)

    t_done = time.perf_counter()
    runtime_s = round(t_done - t_start, 2)
    print(f"\n[csru2] Total runtime: {runtime_s}s", flush=True)

    max_net_runtime = max(per_net_runtimes.values()) if per_net_runtimes else 0.0
    print(f"[csru2] Per-net runtimes: {per_net_runtimes}", flush=True)
    print(f"[csru2] Max net runtime: {max_net_runtime:.2f}s", flush=True)

    return {
        "run_label": run_label,
        "grid_baseline_unconnected": counts_grid["unconnected"],
        "grid_baseline_errors": counts_grid["errors"],
        "signal_unconnected_grid": sorted(signal_unconnected),
        "targets_attempted": sorted(set(targets_attempted)),
        "targets_connected": sorted(set(targets_connected)),
        "boxed_in_confirmed": sorted(set(boxed_in_confirmed)),
        "blockers_ripped": sorted(set(blockers_ripped)),
        "blockers_failed_reroute": sorted(set(blockers_failed_reroute)),
        "final_unconnected": counts_final["unconnected"],
        "final_errors": counts_final["errors"],
        "final_unconnected_nets": sorted(counts_final["unconnected_nets"]),
        "hole_clearance_grid": counts_grid["hole_clearance"],
        "hole_to_hole_grid": counts_grid["hole_to_hole"],
        "hole_clearance_final": counts_final["hole_clearance"],
        "hole_to_hole_final": counts_final["hole_to_hole"],
        "emitted_coords": emitted_coords,
        "per_net_runtimes_s": per_net_runtimes,
        "max_net_runtime_s": max_net_runtime,
        "runtime_s": runtime_s,
        "final_board": str(final_board),
    }


def main() -> None:
    import shutil as _shutil
    base_tmp = Path("/tmp") / "csru2_spike"
    _shutil.rmtree(base_tmp, ignore_errors=True)

    t_total = time.perf_counter()

    # -------------------------------------------------------------------------
    # Run 1
    # -------------------------------------------------------------------------
    run1_dir = base_tmp / "run1"
    r1 = run_csru2(run1_dir, "run1")

    print(f"\n[csru2] === Run 1 Summary ===", flush=True)
    print(f"  grid_baseline_unconnected  = {r1['grid_baseline_unconnected']}", flush=True)
    print(f"  grid_baseline_errors       = {r1['grid_baseline_errors']}", flush=True)
    print(f"  targets_attempted          = {r1['targets_attempted']}", flush=True)
    print(f"  targets_connected          = {r1['targets_connected']}", flush=True)
    print(f"  boxed_in_confirmed         = {r1['boxed_in_confirmed']}", flush=True)
    print(f"  blockers_ripped            = {r1['blockers_ripped']}", flush=True)
    print(f"  blockers_failed_reroute    = {r1['blockers_failed_reroute']}", flush=True)
    print(f"  final_unconnected          = {r1['final_unconnected']}", flush=True)
    print(f"  final_errors               = {r1['final_errors']}", flush=True)
    print(f"  new_hole_clearance         = "
          f"{r1['hole_clearance_final'] - r1['hole_clearance_grid']}", flush=True)
    print(f"  new_hole_to_hole           = "
          f"{r1['hole_to_hole_final'] - r1['hole_to_hole_grid']}", flush=True)
    print(f"  max_net_runtime_s          = {r1['max_net_runtime_s']:.2f}s", flush=True)
    print(f"  runtime_s                  = {r1['runtime_s']}s", flush=True)

    # -------------------------------------------------------------------------
    # Run 2 (determinism check)
    # -------------------------------------------------------------------------
    print(f"\n[csru2] === Run 2 (determinism check) ===", flush=True)
    run2_dir = base_tmp / "run2"
    r2 = run_csru2(run2_dir, "run2")

    # -------------------------------------------------------------------------
    # Determinism check
    # -------------------------------------------------------------------------
    det_unconnected_stable = r1["final_unconnected"] == r2["final_unconnected"]
    det_coords_identical = r1["emitted_coords"] == r2["emitted_coords"]
    det_targets_same = r1["targets_connected"] == r2["targets_connected"]

    print(f"\n[csru2] Determinism:", flush=True)
    print(f"  unconnected stable: {det_unconnected_stable} "
          f"({r1['final_unconnected']} vs {r2['final_unconnected']})", flush=True)
    print(f"  coords identical:   {det_coords_identical}", flush=True)
    print(f"  targets_connected same: {det_targets_same}", flush=True)

    if not det_coords_identical:
        print("[csru2] WARNING: coord diff between run1 and run2:", flush=True)
        lines1 = set(r1["emitted_coords"].splitlines())
        lines2 = set(r2["emitted_coords"].splitlines())
        for ln in sorted(lines1 - lines2):
            print(f"  -run1: {ln}", flush=True)
        for ln in sorted(lines2 - lines1):
            print(f"  +run2: {ln}", flush=True)

    # -------------------------------------------------------------------------
    # Gate evaluation
    # -------------------------------------------------------------------------
    total_runtime = round(time.perf_counter() - t_total, 2)

    new_hole_clearance = r1["hole_clearance_final"] - r1["hole_clearance_grid"]
    new_hole_to_hole = r1["hole_to_hole_final"] - r1["hole_to_hole_grid"]
    new_via_hole_errors = max(0, new_hole_clearance) + max(0, new_hole_to_hole)

    final_unc = r1["final_unconnected"]
    unconnected_lt_47 = final_unc < 47
    unconnected_lt_48 = final_unc < 48
    errors_ok = r1["final_errors"] <= 89 + 20

    # Net-positive swap balance
    n_connected = len(r1["targets_connected"])
    n_failed_reroute = len(r1["blockers_failed_reroute"])
    net_balance = n_connected - n_failed_reroute
    net_swap_balance = (
        f"+{n_connected} targets connected, "
        f"-{n_failed_reroute} blockers failed reroute = "
        f"net {'+' if net_balance >= 0 else ''}{net_balance}"
    )

    pathological_slowness = (
        r1["max_net_runtime_s"] > 60.0 or total_runtime > 600.0
    )

    # CSRU2 verdict
    all_gates_pass = (
        unconnected_lt_47
        and new_via_hole_errors == 0
        and errors_ok
        and det_unconnected_stable
    )

    if all_gates_pass and n_connected > 0:
        csru_verdict = "GO"
        csru_reason = (
            f"CSRU2 connects {n_connected} target(s): {r1['targets_connected']}, "
            f"unconnected {r1['grid_baseline_unconnected']} -> {final_unc} (< 47), "
            f"0 new via-hole errors, deterministic"
        )
    elif unconnected_lt_47 and n_connected > 0 and not det_unconnected_stable:
        csru_verdict = "GO-WITH-CAVEATS"
        csru_reason = (
            f"Connects {n_connected} target(s) but non-deterministic unconnected count"
        )
    elif unconnected_lt_48 and not unconnected_lt_47 and n_connected > 0:
        csru_verdict = "GO-WITH-CAVEATS"
        csru_reason = (
            f"Connects {n_connected} target(s), unconnected={final_unc} (< 48 but not < 47). "
            f"Net balance: {net_swap_balance}. "
            f"Beats grid-only by 1 but not M3-P2 rescue baseline."
        )
    elif n_connected == 0:
        csru_verdict = "NO-GO"
        csru_reason = (
            f"No targets connected by rip-up. "
            f"Targets attempted: {r1['targets_attempted']}. "
            f"Either all are still geometry-blocked after rip-up, "
            f"or all swaps were net-negative."
        )
    elif new_via_hole_errors > 0:
        csru_verdict = "NO-GO"
        csru_reason = f"New via-hole errors introduced: {new_via_hole_errors}"
    else:
        csru_verdict = "NO-GO"
        csru_reason = (
            f"unconnected={final_unc} >= 47; net balance {net_swap_balance}"
        )

    # Issues list
    issues = []
    if r1["grid_baseline_unconnected"] != 48:
        issues.append(
            f"Grid baseline unconnected={r1['grid_baseline_unconnected']}, expected 48"
        )
    if not unconnected_lt_47:
        issues.append(
            f"final_unconnected={final_unc} >= 47 (gate: strictly < 47)"
        )
    if not unconnected_lt_48:
        issues.append(f"final_unconnected={final_unc} >= 48 (no gain over grid-only)")
    if new_via_hole_errors > 0:
        issues.append(
            f"New via-hole errors: hole_clearance={new_hole_clearance}, "
            f"hole_to_hole={new_hole_to_hole}"
        )
    if not det_coords_identical:
        issues.append("Emitted coords not byte-identical across runs")
    if not det_unconnected_stable:
        issues.append(
            f"Unconnected not stable: {r1['final_unconnected']} vs {r2['final_unconnected']}"
        )
    if pathological_slowness:
        issues.append(
            f"Pathological slowness: max_net={r1['max_net_runtime_s']:.1f}s, "
            f"total={total_runtime}s"
        )
    if n_failed_reroute > 0:
        issues.append(
            f"Blockers that failed reroute (newly unconnected): "
            f"{r1['blockers_failed_reroute']}"
        )

    # -------------------------------------------------------------------------
    # Final report
    # -------------------------------------------------------------------------
    det_str = (
        "PASS" if (det_unconnected_stable and det_coords_identical) else
        f"FAIL (unconnected_stable={det_unconnected_stable}, "
        f"coords_identical={det_coords_identical})"
    )

    print("\n" + "=" * 70, flush=True)
    print(f"CSRU2 VERDICT: {csru_verdict}", flush=True)
    print(f"Reason: {csru_reason}", flush=True)
    print(f"Gates:", flush=True)
    print(f"  unconnected_lt_47      = {unconnected_lt_47} ({final_unc} < 47)", flush=True)
    print(f"  unconnected_lt_48      = {unconnected_lt_48} ({final_unc} < 48)", flush=True)
    print(f"  new_via_hole_errors    = {new_via_hole_errors}", flush=True)
    print(f"  errors_ok              = {errors_ok} ({r1['final_errors']} <= ~109)",
          flush=True)
    print(f"  deterministic          = {det_unconnected_stable and det_coords_identical}",
          flush=True)
    print(f"  max_net_runtime_s      = {r1['max_net_runtime_s']:.2f}s", flush=True)
    print(f"  pathological_slowness  = {pathological_slowness}", flush=True)
    print(f"  total_runtime_s        = {total_runtime}s", flush=True)
    print(f"Net swap balance: {net_swap_balance}", flush=True)
    if issues:
        print(f"Issues:", flush=True)
        for iss in issues:
            print(f"  - {iss}", flush=True)
    print("=" * 70, flush=True)

    # -------------------------------------------------------------------------
    # Structured Result
    # -------------------------------------------------------------------------
    structured = {
        "status": csru_verdict,
        "summary": csru_reason,
        "files_changed": [
            "/home/palgin/Business_projects/tracewise/scripts/spike_csru2_multipin_ripup.py"
        ],
        "files_read": [
            "src/tracewise/route/gridless/route.py",
            "src/tracewise/route/gridless/geom.py",
            "src/tracewise/route/gridless/adapter.py",
            "src/tracewise/route/engine/multi.py",
            "src/tracewise/route/engine/kicad.py",
            "src/tracewise/route/bridge.py",
            "scripts/spike_csru_cross_substrate_ripup.py",
            "scripts/_verify_m3p2_residual.py",
        ],
        "grid_baseline": {
            "unconnected": r1["grid_baseline_unconnected"],
            "errors": r1["grid_baseline_errors"],
            "signal_unconnected": r1["signal_unconnected_grid"],
        },
        "targets_attempted": r1["targets_attempted"],
        "targets_connected": r1["targets_connected"],
        "boxed_in_confirmed": r1["boxed_in_confirmed"],
        "blockers_ripped": r1["blockers_ripped"],
        "blockers_failed_reroute": r1["blockers_failed_reroute"],
        "net_swap_balance": net_swap_balance,
        "final_unconnected": final_unc,
        "final_unconnected_nets": r1["final_unconnected_nets"],
        "unconnected_lt_47": unconnected_lt_47,
        "unconnected_lt_48": unconnected_lt_48,
        "new_via_hole_errors": new_via_hole_errors,
        "final_errors": r1["final_errors"],
        "per_net_runtimes_s": r1["per_net_runtimes_s"],
        "max_net_runtime_s": r1["max_net_runtime_s"],
        "pathological_slowness": pathological_slowness,
        "deterministic": det_str,
        "csru2_go_no_go": f"{csru_verdict}: {csru_reason}",
        "issues": issues,
        "assumptions": [
            "Grid-only baseline is 48 unconnected / 89 errors",
            "Power nets (GND/+3V3/+1V1) excluded from targets (pour-coverage class)",
            "BOUNDED windows: max {MAX_WINDOW_MM}mm — no board-wide blowup",
            "route_net_multipin handles both 2-pin and multi-pin targets+blockers",
            "Net-positive criterion: targets_connected > blockers_failed_reroute (strictly)",
            "Blocking-net detection uses BOUNDED pad-span window (+WINDOW_EXPAND_MM mm)",
            "rerouted blockers add their copper to obstacles for subsequent blocker rerouting",
        ],
    }

    print("\n## Structured Result", flush=True)
    print("```json", flush=True)
    print(json.dumps(structured, indent=2), flush=True)
    print("```", flush=True)


if __name__ == "__main__":
    main()
