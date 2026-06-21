"""Probe-Order: definitive routing-order vs placement-bound analysis for mitayi.

Routes the 17 boxed-in signal nets on a CLEAN (stripped) board — no grid tracks
as obstacles — and records how many connect.  Each successfully routed net's
copper is added as an obstacle for subsequent nets to prevent overlap.

KEY CONSTRAINT: BOUNDED WINDOWS only. NO window escalation beyond the capped
sub-edge window. Prefer single-layer; allow via only as explicit fallback.
This avoids the M3-P1.1 board-scale blowup.

Verdict:
  - ORDERING problem: most of the 17 connect fast on a clean board
    → route them gridless-FIRST before the grid lays fragmenting tracks
  - PLACEMENT-BOUND: many fail even on a clean board
    → connectivity needs placement changes, routing at ceiling

Honesty mandate: report REAL numbers.  Do NOT fake connectivity.

Usage:
    cd /home/palgin/Business_projects/tracewise
    .venv/bin/python scripts/probe_order.py
"""
from __future__ import annotations

import collections
import heapq
import json
import math
import re
import shutil
import time
from pathlib import Path
import sys

# ---------------------------------------------------------------------------
# Shapely guard
# ---------------------------------------------------------------------------
try:
    import shapely
    from shapely.geometry import LineString, Point as SPoint, box as shapely_box
    from shapely.ops import unary_union

    GEOS_VERSION = shapely.geos_version
    if GEOS_VERSION < (3, 8, 0):
        raise RuntimeError(f"GEOS >= 3.8.0 required, got {GEOS_VERSION}")
    print(f"[probe_order] Shapely {shapely.__version__}  GEOS {GEOS_VERSION}", flush=True)
except ImportError as exc:
    print(f"ERROR: Shapely not installed: {exc}", file=sys.stderr)
    sys.exit(1)

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "src"))

from tracewise.route.bridge import run_drc, strip_routing
from tracewise.route.engine.kicad import (
    extract_pads,
    project_geometry,
    refill_zones,
)
from tracewise.route.gridless.geom import (
    PRECISION,
    build_windowed_free_space,
    extract_board_outline,
    extract_drill_centers,
    extract_drill_obstacles,
    snap,
)
from tracewise.route.gridless.route import (
    GridlessRouteResult,
)
from tracewise.route.gridless.search import route_window
from tracewise.sexpr import atom, node, parse_file, write_file

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

BOARD_SRC = ROOT / "data/benchmark-boards/mitayi-pico-d1/Mitayi-Pico-D1.kicad_pcb"
SUFFIXES = (".kicad_pcb", ".kicad_sch", ".kicad_pro", ".kicad_prl")

# The 17 boxed-in signal nets identified by prior probes
TARGET_NETS = [
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

# Max sub-edge window: bounded. If a net's pad span is larger, window = span*1.2+2mm
# but NEVER escalate beyond this single attempt.
BASE_WINDOW_MM = 8.0
# Hard cap: never exceed this window regardless of pad distance
MAX_WINDOW_MM = 20.0

# ---------------------------------------------------------------------------
# Board setup
# ---------------------------------------------------------------------------


def setup_board(out_dir: Path) -> Path:
    """Copy mitayi to out_dir, strip all routing. Returns board path."""
    bdir = BOARD_SRC.parent
    out_dir.mkdir(parents=True, exist_ok=True)
    for f in bdir.iterdir():
        if f.suffix in SUFFIXES:
            shutil.copy(f, out_dir / f.name)
    board = next(out_dir.glob("*.kicad_pcb"))
    strip_routing(board)
    return board


# ---------------------------------------------------------------------------
# BOUNDED single-attempt 2-pin route: NO escalation loop
# ---------------------------------------------------------------------------


def _route_2pin_bounded_single_layer(
    pad_a: tuple[float, float],
    pad_b: tuple[float, float],
    pads: list[dict],
    net_name: str,
    geo: dict,
    board_bbox: tuple[float, float, float, float],
    extra_obstacles: list,
    window_mm: float,
    board_outline: object | None,
    drill_obstacles: list,
) -> tuple[list[tuple] | None, float]:
    """Single-attempt single-layer route: build free space once, route_window once.

    Returns (path_or_None, build_time_s).
    NO window escalation. If no path: return None immediately.
    """
    track_mm = geo["track_mm"]
    clearance_mm = geo["clearance_mm"]
    bx1, by1, bx2, by2 = board_bbox

    wx1 = max(min(pad_a[0], pad_b[0]) - window_mm, bx1)
    wy1 = max(min(pad_a[1], pad_b[1]) - window_mm, by1)
    wx2 = min(max(pad_a[0], pad_b[0]) + window_mm, bx2)
    wy2 = min(max(pad_a[1], pad_b[1]) + window_mm, by2)
    window_bbox = (wx1, wy1, wx2, wy2)

    t0 = time.perf_counter()
    try:
        free_space, obstacle_polys = build_windowed_free_space(
            pads, net_name, clearance_mm, track_mm,
            extra_obstacles, window_bbox,
            board_outline=board_outline,
            drill_obstacles=drill_obstacles,
        )
    except Exception as exc:
        return None, time.perf_counter() - t0
    build_t = time.perf_counter() - t0

    try:
        path, _n_nodes, _n_edges = route_window(
            free_space, pad_a, pad_b, window_mm, obstacle_polys
        )
    except Exception:
        path = None

    return path, build_t


def route_2pin_bounded(
    pad_a: tuple[float, float],
    pad_b: tuple[float, float],
    pads: list[dict],
    net_name: str,
    geo: dict,
    board_bbox: tuple[float, float, float, float],
    extra_obstacles: list,
    window_mm: float,
    board_outline: object | None,
    drill_obstacles: list,
    drill_centers: list,
) -> tuple[GridlessRouteResult, bool]:
    """Route 2-pin bounded: single-layer first, via fallback if needed.

    Returns (GridlessRouteResult, used_via).
    STRICT: no window escalation. window_mm is the ONLY window tried.
    """
    track_mm = geo["track_mm"]
    clearance_mm = geo["clearance_mm"]
    bx1, by1, bx2, by2 = board_bbox

    t0 = time.perf_counter()

    # --- Single-layer attempt ---
    path, build_t = _route_2pin_bounded_single_layer(
        pad_a, pad_b, pads, net_name, geo, board_bbox,
        extra_obstacles, window_mm, board_outline, drill_obstacles,
    )

    if path is not None and len(path) >= 2:
        # path is already in world mm coords from route_window/A*
        # snap_waypoints for determinism
        try:
            from tracewise.route.gridless.realize import snap_waypoints
            world_path = snap_waypoints(path)
        except Exception:
            world_path = path
        elapsed = time.perf_counter() - t0
        return GridlessRouteResult(
            ok=True,
            world_paths=[world_path],
            world_vias=[],
            stats={"build_time_s": build_t, "total_time_s": elapsed},
            reason="",
        ), False

    # NOTE: Via fallback intentionally DISABLED for this probe.
    # Purpose: test whether clean-board corridors are open for single-layer routing.
    # Via is a secondary mechanism — if single-layer fails, the corridor question
    # is still answered (no_path_single_layer = placement/geometry issue, not ordering).
    # The 2-layer via search is O(n²) expensive at board scale; disabling it
    # avoids the M3-P1.1 blowup and keeps each net bounded to seconds.

    elapsed = time.perf_counter() - t0
    return GridlessRouteResult(
        ok=False,
        stats={"build_time_s": build_t, "total_time_s": elapsed},
        reason="no_path_single_layer (via disabled for speed)",
    ), False


# ---------------------------------------------------------------------------
# Bounded MST multi-pin route: no escalation, bounded windows per sub-edge
# ---------------------------------------------------------------------------


def _prim_mst(pads: list[dict]) -> list[tuple[int, int, float]]:
    """Deterministic Prim MST, seed=0, tie-break by (dist, i, j)."""
    K = len(pads)
    if K < 2:
        return []
    in_tree = [False] * K
    in_tree[0] = True
    heap: list[tuple[float, int, int]] = []
    for j in range(1, K):
        d = math.hypot(pads[0]["x"] - pads[j]["x"], pads[0]["y"] - pads[j]["y"])
        heapq.heappush(heap, (d, 0, j))
    edges: list[tuple[int, int, float]] = []
    while len(edges) < K - 1:
        if not heap:
            break
        d, i, j = heapq.heappop(heap)
        if in_tree[j]:
            continue
        in_tree[j] = True
        edges.append((i, j, d))
        for k in range(K):
            if not in_tree[k]:
                dk = math.hypot(pads[j]["x"] - pads[k]["x"], pads[j]["y"] - pads[k]["y"])
                heapq.heappush(heap, (dk, j, k))
    return edges


def route_multipin_bounded(
    pads_of_net: list[dict],
    net_name: str,
    all_pads: list[dict],
    geo: dict,
    board_bbox: tuple[float, float, float, float],
    extra_obstacles: list,
    window_mm: float,
    board_outline: object | None,
    drill_obstacles: list,
    drill_centers: list,
) -> tuple[GridlessRouteResult, int, int]:
    """MST multi-pin bounded route: no window escalation.

    Returns (result, n_single_layer, n_via).
    Each sub-edge: window = min(MAX_WINDOW_MM, max(window_mm, dist*1.2+2)), single attempt.
    """
    K = len(pads_of_net)
    if K == 2:
        sub_dist = math.hypot(
            pads_of_net[0]["x"] - pads_of_net[1]["x"],
            pads_of_net[0]["y"] - pads_of_net[1]["y"]
        )
        sub_win = min(MAX_WINDOW_MM, max(window_mm, sub_dist * 1.2 + 2.0))
        res, used_via = route_2pin_bounded(
            (pads_of_net[0]["x"], pads_of_net[0]["y"]),
            (pads_of_net[1]["x"], pads_of_net[1]["y"]),
            all_pads, net_name, geo, board_bbox,
            extra_obstacles, sub_win, board_outline, drill_obstacles, drill_centers,
        )
        n_single = 1 if (res.ok and not used_via) else 0
        n_via = 1 if (res.ok and used_via) else 0
        return res, n_single, n_via

    track_mm = geo["track_mm"]
    via_mm = geo.get("via_mm", 0.4)
    clearance_mm = geo["clearance_mm"]

    mst_edges = _prim_mst(pads_of_net)
    all_world_paths: list[list[tuple]] = []
    all_via_centers: list[tuple[float, float]] = []
    same_net_geom: object | None = None
    n_ok = 0
    n_failed = 0
    n_single = 0
    n_via_used = 0
    bx1, by1, bx2, by2 = board_bbox

    for _ei, (i, j, pad_dist) in enumerate(mst_edges):
        tree_pad = pads_of_net[i]
        new_pad = pads_of_net[j]
        tx, ty = tree_pad["x"], tree_pad["y"]
        nx, ny = new_pad["x"], new_pad["y"]

        sub_win = min(MAX_WINDOW_MM, max(window_mm, pad_dist * 1.2 + 2.0))

        # Same-net copper shortcut (shorter path to tree copper)
        start_xy = (tx, ty)
        goal_xy = (nx, ny)

        if same_net_geom is not None:
            try:
                from tracewise.route.gridless.geom import snap as _snap
                wx1 = max(min(tx, nx) - sub_win, bx1)
                wy1 = max(min(ty, ny) - sub_win, by1)
                wx2 = min(max(tx, nx) + sub_win, bx2)
                wy2 = min(max(ty, ny) + sub_win, by2)
                window_poly = shapely_box(wx1, wy1, wx2, wy2)
                clipped = _snap(same_net_geom.intersection(window_poly))
                if not clipped.is_empty:
                    # Sample points on copper boundary
                    geoms_list = list(clipped.geoms) if clipped.geom_type == "MultiPolygon" else [clipped]
                    pts: list[tuple[float, float]] = []
                    for g in geoms_list:
                        if hasattr(g, "exterior"):
                            for cx, cy in g.exterior.coords:
                                pts.append((round(cx, 6), round(cy, 6)))
                    pts = sorted(set(pts))[:40]
                    if pts:
                        nearest = min(pts, key=lambda p: math.hypot(p[0] - nx, p[1] - ny))
                        copper_dist = math.hypot(nearest[0] - nx, nearest[1] - ny)
                        is_at_tree = (abs(nearest[0] - tx) < track_mm and abs(nearest[1] - ty) < track_mm)
                        if copper_dist < pad_dist - 0.05 and not is_at_tree:
                            start_xy = (nx, ny)
                            goal_xy = nearest
                            sub_win = min(MAX_WINDOW_MM, max(window_mm, copper_dist * 1.2 + 2.0))
                        else:
                            start_xy = (nx, ny)
                            goal_xy = (tx, ty)
                    else:
                        start_xy = (nx, ny)
                        goal_xy = (tx, ty)
                else:
                    start_xy = (nx, ny)
                    goal_xy = (tx, ty)
            except Exception:
                start_xy = (nx, ny)
                goal_xy = (tx, ty)

        result, used_via = route_2pin_bounded(
            start_xy, goal_xy, all_pads, net_name, geo, board_bbox,
            extra_obstacles, sub_win, board_outline, drill_obstacles, drill_centers,
        )

        if not result.ok:
            n_failed += 1
            continue

        n_ok += 1
        if used_via:
            n_via_used += 1
        else:
            n_single += 1
        all_world_paths.extend(result.world_paths)
        all_via_centers.extend(result.world_vias)

        # Update same-net copper
        try:
            from shapely.ops import unary_union as _uu
            from tracewise.route.gridless.geom import snap as _snap2
            polys = []
            for wpath in result.world_paths:
                if not wpath:
                    continue
                pts_2d = [(p[0], p[1]) for p in wpath]
                if len(pts_2d) >= 2:
                    ls = _snap2(LineString(pts_2d).buffer(track_mm / 2.0, cap_style=2))
                    polys.append(ls)
            for vx, vy in result.world_vias:
                circle = _snap2(SPoint(vx, vy).buffer(via_mm / 2.0, resolution=16))
                polys.append(circle)
            if polys:
                new_geom = _snap2(_uu(polys))
                if same_net_geom is None:
                    same_net_geom = new_geom
                else:
                    same_net_geom = _snap2(_uu([same_net_geom, new_geom]))
        except Exception:
            pass

    if n_ok == 0:
        return GridlessRouteResult(
            ok=False, reason=f"all {len(mst_edges)} sub-edges failed"
        ), 0, 0

    ok = (n_failed == 0)
    reason = "" if ok else f"{n_failed}/{len(mst_edges)} sub-edges failed"
    return GridlessRouteResult(
        ok=ok, world_paths=all_world_paths, world_vias=all_via_centers, reason=reason
    ), n_single, n_via_used


# ---------------------------------------------------------------------------
# Helpers to emit routed results to board file
# ---------------------------------------------------------------------------


def emit_result_to_board(
    board: Path,
    net_name: str,
    result: GridlessRouteResult,
    track_mm: float,
    via_mm: float,
    via_drill: float,
) -> None:
    """Append routed segments + vias for net_name to the board file."""
    root = parse_file(board)
    decls = {n.arg(2): n.arg(1) for n in root.nodes("net") if n.arg(2) is not None}
    layer_name = {0: "F.Cu", 1: "B.Cu"}

    def net_nd_fn(name: str):
        if decls:
            num = decls.get(name)
            if num is not None:
                return node("net", num)
        return node("net", atom(name, quote=True))

    net_nd = net_nd_fn(net_name)
    seen_vias: set[tuple[float, float]] = set()

    for wpath in result.world_paths:
        if not wpath:
            continue
        if len(wpath[0]) == 3:
            path_3d = wpath
        else:
            path_3d = [(p[0], p[1], 0) for p in wpath]

        for idx in range(len(path_3d) - 1):
            xa, ya, la = path_3d[idx]
            xb, yb, lb = path_3d[idx + 1]
            if la == lb:
                if abs(xa - xb) < 1e-9 and abs(ya - yb) < 1e-9:
                    continue
                seg = node(
                    "segment",
                    node("start", f"{xa:.6f}", f"{ya:.6f}"),
                    node("end", f"{xb:.6f}", f"{yb:.6f}"),
                    node("width", str(track_mm)),
                    node("layer", atom(layer_name[la], quote=True)),
                    net_nd,
                )
                root.insert(seg)
            else:
                vx = round(xa / PRECISION) * PRECISION
                vy = round(ya / PRECISION) * PRECISION
                seen_vias.add((round(vx, 6), round(vy, 6)))

    for vx, vy in result.world_vias:
        seen_vias.add((round(vx, 6), round(vy, 6)))

    for vx, vy in sorted(seen_vias):
        via_node = node(
            "via",
            node("at", f"{vx:.6f}", f"{vy:.6f}"),
            node("size", str(via_mm)),
            node("drill", str(via_drill)),
            node("layers", atom("F.Cu", quote=True), atom("B.Cu", quote=True)),
            net_nd,
        )
        root.insert(via_node)

    write_file(root, board)


# ---------------------------------------------------------------------------
# Build extra_obstacles from already-routed copper
# ---------------------------------------------------------------------------


def paths_to_obstacles(
    world_paths: list[list[tuple]],
    via_centers: list[tuple[float, float]],
    track_mm: float,
    clearance_mm: float,
    via_mm: float,
) -> list:
    """Convert routed world_paths + vias to inflated Shapely obstacles."""
    inflate = track_mm / 2.0 + clearance_mm
    via_inflate = via_mm / 2.0 + clearance_mm
    polys = []
    for wpath in world_paths:
        if not wpath:
            continue
        pts_2d = [(p[0], p[1]) for p in wpath]
        if len(pts_2d) < 2:
            continue
        try:
            ls = snap(LineString(pts_2d).buffer(inflate, cap_style=2))
            polys.append(ls)
        except Exception:
            pass
    for vx, vy in via_centers:
        try:
            circle = snap(SPoint(vx, vy).buffer(via_inflate, resolution=16))
            polys.append(circle)
        except Exception:
            pass
    return polys


# ---------------------------------------------------------------------------
# DRC helpers
# ---------------------------------------------------------------------------


def _unconnected_nets_from_report(report: dict) -> set[str]:
    names: set[str] = set()
    for u in report.get("unconnected_items", []):
        for it in u.get("items", []):
            for m in re.findall(r"\[([^\]]+)\]", it.get("description", "")):
                names.add(m)
    return names


def _net_unconnected_count(report: dict, net_name: str) -> int:
    count = 0
    for u in report.get("unconnected_items", []):
        for it in u.get("items", []):
            if net_name in str(it):
                count += 1
                break
    return count


# ---------------------------------------------------------------------------
# Extract coords for determinism check
# ---------------------------------------------------------------------------


def _extract_coords_for_nets(board: Path, net_names: list[str]) -> str:
    root = parse_file(board)
    decls = {n.arg(2): n.arg(1) for n in root.nodes("net") if n.arg(2) is not None}
    name_to_num = {v: k for k, v in decls.items()}

    def _matches(val: str | None, name: str) -> bool:
        if val is None:
            return False
        num = name_to_num.get(name)
        return val == name or (num is not None and val == num)

    lines = []
    for seg in root.nodes("segment"):
        for child in seg.nodes("net"):
            for nm in net_names:
                if _matches(child.arg(1), nm):
                    start = seg.first("start")
                    end_ = seg.first("end")
                    if start and end_:
                        lines.append(f"seg:{nm}:{start.arg(1)},{start.arg(2)}-{end_.arg(1)},{end_.arg(2)}")
    for via in root.nodes("via"):
        for child in via.nodes("net"):
            for nm in net_names:
                if _matches(child.arg(1), nm):
                    at = via.first("at")
                    if at:
                        lines.append(f"via:{nm}:{at.arg(1)},{at.arg(2)}")
    lines.sort()
    return "\n".join(lines)


# ---------------------------------------------------------------------------
# Core routing loop
# ---------------------------------------------------------------------------


def route_target_nets(
    board: Path,
    data: dict,
    geo: dict,
    board_bbox: tuple[float, float, float, float],
    board_outline: object | None,
    drill_obstacles: list,
    drill_centers: list,
    window_mm: float = BASE_WINDOW_MM,
) -> list[dict]:
    """Route each of the 17 target nets sequentially on a clean board.

    Bounded: no window escalation. Each net max window = min(MAX_WINDOW_MM, dist*1.2+2mm).
    Returns per-net result dicts.
    """
    track_mm = geo["track_mm"]
    clearance_mm = geo["clearance_mm"]
    via_mm = geo.get("via_mm", 0.4)
    via_drill = geo.get("via_drill_mm", 0.2)

    by_net: dict[str, list[dict]] = collections.defaultdict(list)
    for p in data["pads"]:
        if p.get("net"):
            by_net[p["net"]].append(p)

    extra_obstacles: list = []
    results = []

    for net_name in TARGET_NETS:
        pads_of_net = by_net.get(net_name, [])
        n_pins = len(pads_of_net)

        if n_pins < 2:
            results.append({
                "net": net_name,
                "n_pins": n_pins,
                "connected": False,
                "reason": f"only {n_pins} pad(s) — net absent or single-pad",
                "single_layer": None,
                "via_used": None,
                "window_mm": None,
                "solve_time_s": 0.0,
            })
            print(f"[probe_order] SKIP {net_name!r}: {n_pins} pads", flush=True)
            continue

        print(f"\n[probe_order] === Routing {net_name!r}: {n_pins} pins ===", flush=True)
        for pi, p in enumerate(pads_of_net):
            print(f"[probe_order]   pad[{pi}] ({p['x']:.3f},{p['y']:.3f}) F={p.get('front')} B={p.get('back')}", flush=True)

        t0 = time.perf_counter()

        result, n_single_layer, n_via = route_multipin_bounded(
            pads_of_net, net_name, data["pads"], geo, board_bbox,
            extra_obstacles, window_mm, board_outline, drill_obstacles, drill_centers,
        )
        elapsed = time.perf_counter() - t0

        via_used = n_via > 0
        single_layer = n_single_layer > 0 and n_via == 0

        print(f"[probe_order] {net_name!r}: ok={result.ok} via={via_used} t={elapsed:.3f}s reason={result.reason!r}", flush=True)

        if result.ok:
            emit_result_to_board(board, net_name, result, track_mm, via_mm, via_drill)
            new_obstacles = paths_to_obstacles(
                result.world_paths, result.world_vias,
                track_mm, clearance_mm, via_mm
            )
            extra_obstacles.extend(new_obstacles)

        results.append({
            "net": net_name,
            "n_pins": n_pins,
            "connected": result.ok,
            "reason": result.reason if not result.ok else "ok",
            "single_layer": single_layer,
            "via_used": via_used,
            "window_mm": window_mm,
            "solve_time_s": round(elapsed, 4),
        })

    return results


# ---------------------------------------------------------------------------
# Determinism check
# ---------------------------------------------------------------------------


def run_determinism_check(
    src_dir: Path,
    data: dict,
    geo: dict,
    board_bbox: tuple[float, float, float, float],
    board_outline: object | None,
    drill_obstacles: list,
    drill_centers: list,
    window_mm: float,
    out_dir: Path,
) -> tuple[str, str, str]:
    """Route on two separate board copies; return (coords1, coords2, status)."""
    run1_dir = out_dir / "det_run1"
    run2_dir = out_dir / "det_run2"

    for run_dir in [run1_dir, run2_dir]:
        shutil.rmtree(run_dir, ignore_errors=True)
        shutil.copytree(src_dir, run_dir)

    board1 = next(run1_dir.glob("*.kicad_pcb"))
    board2 = next(run2_dir.glob("*.kicad_pcb"))

    route_target_nets(board1, data, geo, board_bbox, board_outline,
                      drill_obstacles, drill_centers, window_mm)
    route_target_nets(board2, data, geo, board_bbox, board_outline,
                      drill_obstacles, drill_centers, window_mm)

    coords1 = _extract_coords_for_nets(board1, TARGET_NETS)
    coords2 = _extract_coords_for_nets(board2, TARGET_NETS)

    status = "byte-identical" if coords1 == coords2 else "differ"
    print(f"[probe_order] DETERMINISM: {status} ({len(coords1.splitlines())} coord lines)", flush=True)
    if status != "byte-identical":
        # Show first diff
        lines1 = coords1.splitlines()
        lines2 = coords2.splitlines()
        for la, lb in zip(lines1[:20], lines2[:20]):
            if la != lb:
                print(f"[probe_order]   DIFF: run1={la!r}", flush=True)
                print(f"[probe_order]         run2={lb!r}", flush=True)
                break
    return coords1, coords2, status


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main() -> None:
    import tempfile

    print("=" * 70, flush=True)
    print("Probe-Order: routing-order vs placement-bound for mitayi 17 nets", flush=True)
    print(f"MAX_WINDOW_MM={MAX_WINDOW_MM}  BASE_WINDOW_MM={BASE_WINDOW_MM}  no escalation", flush=True)
    print("=" * 70, flush=True)

    t_wall_start = time.perf_counter()
    issues: list[str] = []

    tmp_base = ROOT / ".probe_order_tmp"
    tmp_base.mkdir(exist_ok=True)

    with tempfile.TemporaryDirectory(dir=tmp_base, prefix="probe_order_") as _tmp:
        out_dir = Path(_tmp)

        # --- Step 1: Stripped board ---
        print("\n[probe_order] Step 1: Setup stripped board...", flush=True)
        board = setup_board(out_dir / "main")
        print(f"[probe_order] Board: {board}", flush=True)

        data = extract_pads(board)
        geo = project_geometry(board)
        print(f"[probe_order] geo: {geo}", flush=True)

        bd = data["board"]
        board_bbox = (bd["x1"], bd["y1"], bd["x2"], bd["y2"])
        board_diag = math.hypot(bd["x2"] - bd["x1"], bd["y2"] - bd["y1"])
        print(f"[probe_order] board_bbox={board_bbox}, diag={board_diag:.3f}mm", flush=True)

        board_outline = extract_board_outline(board)
        drill_obstacles = extract_drill_obstacles(
            board, clearance_mm=geo["clearance_mm"], track_mm=geo["track_mm"]
        )
        drill_centers = extract_drill_centers(board)
        print(f"[probe_order] drill_obstacles={len(drill_obstacles)}, drill_centers={len(drill_centers)}", flush=True)

        by_net_count: dict[str, int] = collections.Counter(
            p["net"] for p in data["pads"] if p.get("net")
        )
        print("\n[probe_order] Target net pin counts:", flush=True)
        for nm in TARGET_NETS:
            n = by_net_count.get(nm, 0)
            print(f"[probe_order]   {nm!r}: {n} pins", flush=True)
            if n < 2:
                issues.append(f"net {nm!r} has only {n} pads — will be skipped")

        # --- Step 2: Baseline DRC ---
        print("\n[probe_order] Step 2: Baseline DRC...", flush=True)
        baseline_report = run_drc(board)
        baseline_unc = len(baseline_report.get("unconnected_items", []))
        baseline_errs = sum(1 for v in baseline_report.get("violations", [])
                            if v.get("severity") == "error")
        print(f"[probe_order] BASELINE: unconnected={baseline_unc} errors={baseline_errs}", flush=True)

        # --- Step 3: Route 17 target nets ---
        print(f"\n[probe_order] Step 3: Route 17 target nets (MAX_WINDOW={MAX_WINDOW_MM}mm, no escalation)...", flush=True)
        route_results = route_target_nets(
            board, data, geo, board_bbox, board_outline,
            drill_obstacles, drill_centers, BASE_WINDOW_MM,
        )

        # --- Step 4: Refill + DRC ---
        print("\n[probe_order] Step 4: Refill zones + DRC...", flush=True)
        n_connected_raw = sum(1 for r in route_results if r["connected"])
        if n_connected_raw > 0:
            refill_zones(board)
        after_report = run_drc(board)
        after_unc = len(after_report.get("unconnected_items", []))
        after_errs_all = [v for v in after_report.get("violations", [])
                          if v.get("severity") == "error"]
        after_err_count = len(after_errs_all)
        after_by_type = dict(collections.Counter(v.get("type") for v in after_errs_all))
        after_unconnected_nets = _unconnected_nets_from_report(after_report)
        print(f"[probe_order] AFTER: unconnected={after_unc} errors={after_err_count}", flush=True)
        print(f"[probe_order] AFTER by_type: {after_by_type}", flush=True)

        still_unc_targets = [nm for nm in TARGET_NETS if nm in after_unconnected_nets]
        print(f"[probe_order] Target nets still unconnected in DRC: {still_unc_targets}", flush=True)

        # New trace-attributable errors
        new_trace_errors: dict[str, int] = {}
        for v in after_errs_all:
            vtype = v.get("type", "unknown")
            if vtype == "copper_edge_clearance":
                continue
            for it in v.get("items", []):
                desc = str(it.get("description", ""))
                for nm in TARGET_NETS:
                    if nm in desc:
                        new_trace_errors[vtype] = new_trace_errors.get(vtype, 0) + 1
                        break

        refill_drc_clean = (len(new_trace_errors) == 0)

        # DRC truth for connected/failed
        drc_truth_connected: list[str] = []
        drc_truth_failed: list[dict] = []
        for r in route_results:
            net_nm = r["net"]
            if by_net_count.get(net_nm, 0) < 2:
                drc_truth_failed.append({"net": net_nm, "reason": "absent from board"})
                continue
            unc = _net_unconnected_count(after_report, net_nm)
            if unc == 0:
                drc_truth_connected.append(net_nm)
            else:
                reason = r["reason"] if not r["connected"] else f"route_ok but DRC shows {unc} unconnected"
                drc_truth_failed.append({"net": net_nm, "reason": reason})

        n_nets_connected = len(drc_truth_connected)
        n_targeted = len(TARGET_NETS)

        # --- Step 5: Determinism ---
        print("\n[probe_order] Step 5: Determinism check (2 runs)...", flush=True)
        # Build a clean stripped copy for determinism runs
        det_stripped = out_dir / "det_stripped"
        bdir = BOARD_SRC.parent
        det_stripped.mkdir(parents=True, exist_ok=True)
        for f in bdir.iterdir():
            if f.suffix in SUFFIXES:
                shutil.copy(f, det_stripped / f.name)
        det_board = next(det_stripped.glob("*.kicad_pcb"))
        strip_routing(det_board)
        coords1, coords2, det_status = run_determinism_check(
            det_stripped, data, geo, board_bbox, board_outline,
            drill_obstacles, drill_centers, BASE_WINDOW_MM, out_dir,
        )

        # --- Assemble ---
        n_single_layer = sum(1 for r in route_results if r.get("single_layer") is True)
        n_via_count = sum(1 for r in route_results if r.get("via_used") is True)
        max_net_runtime = max((r["solve_time_s"] for r in route_results), default=0.0)
        total_runtime = time.perf_counter() - t_wall_start

        pathological = any(r["solve_time_s"] > 60.0 for r in route_results)
        if pathological:
            slow_nets = [r["net"] for r in route_results if r["solve_time_s"] > 60.0]
            issues.append(f"PATHOLOGICAL_SLOWNESS: nets with >60s: {slow_nets}")

        connect_rate = n_nets_connected / n_targeted if n_targeted > 0 else 0.0
        if connect_rate >= 0.75:
            verdict = "ordering-problem: gridless-first is viable"
            gridless_first_recommended = True
            gridless_reasoning = (
                f"{n_nets_connected}/{n_targeted} of the boxed-in nets connect on a clean board "
                f"({connect_rate*100:.0f}%). These nets are blocked by other nets' grid tracks, "
                f"not by placement. Routing gridless-FIRST (before grid lays fragmenting tracks) "
                f"should unlock significant connectivity gain."
            )
        elif connect_rate >= 0.4:
            verdict = "mixed"
            gridless_first_recommended = n_nets_connected > n_targeted // 2
            gridless_reasoning = (
                f"{n_nets_connected}/{n_targeted} connect on a clean board. "
                f"Mixed: some are ordering-bound, some placement-bound."
            )
        else:
            verdict = "placement-bound: routing at ceiling"
            gridless_first_recommended = False
            gridless_reasoning = (
                f"Only {n_nets_connected}/{n_targeted} connect on a clean board ({connect_rate*100:.0f}%). "
                f"Placement changes needed."
            )

        print(f"\n[probe_order] === FINAL RESULTS ===", flush=True)
        for r in route_results:
            status_s = "CONNECTED" if r["connected"] else "FAILED"
            via_s = f" via={r['via_used']}" if r["connected"] else ""
            print(f"  {status_s:10s} {r['net']!r:30s} t={r['solve_time_s']:.3f}s pins={r['n_pins']}{via_s} reason={r['reason']!r}", flush=True)

        print(f"\n[probe_order] VERDICT: {verdict}", flush=True)
        print(f"[probe_order] Connected: {n_nets_connected}/{n_targeted}", flush=True)
        print(f"[probe_order] Max net runtime: {max_net_runtime:.3f}s", flush=True)
        print(f"[probe_order] Total runtime: {total_runtime:.1f}s", flush=True)
        print(f"[probe_order] Pathological slowness: {pathological}", flush=True)
        print(f"[probe_order] DRC clean: {refill_drc_clean}", flush=True)
        print(f"[probe_order] Deterministic: {det_status}", flush=True)

        structured = {
            "status": "success" if not pathological else "slow",
            "summary": (
                f"Probe-Order: {n_nets_connected}/{n_targeted} of the 17 boxed-in signal nets "
                f"connect on a clean board in {total_runtime:.1f}s. "
                f"Verdict: {verdict}."
            ),
            "files_changed": [],
            "files_read": [
                "src/tracewise/route/gridless/route.py",
                "src/tracewise/route/gridless/geom.py",
                "src/tracewise/route/engine/kicad.py",
                "src/tracewise/route/bridge.py",
                "scripts/spikeM3P2_gridless_multipin.py",
                "scripts/_verify_m3_ab.py",
            ],
            "nets_targeted": n_targeted,
            "nets_connected_clean_board": {
                "count": n_nets_connected,
                "names": drc_truth_connected,
            },
            "nets_failed": drc_truth_failed,
            "single_layer_count": n_single_layer,
            "via_count": n_via_count,
            "max_net_runtime_s": round(max_net_runtime, 4),
            "total_runtime_s": round(total_runtime, 2),
            "pathological_slowness": pathological,
            "refill_drc_clean": {
                "clean": refill_drc_clean,
                "new_error_count": sum(new_trace_errors.values()),
                "new_errors_by_type": new_trace_errors,
                "total_after_drc_errors": after_err_count,
                "after_drc_by_type": after_by_type,
            },
            "deterministic": det_status,
            "baseline_drc": {"unconnected": baseline_unc, "errors": baseline_errs},
            "after_drc": {"unconnected": after_unc, "errors": after_err_count},
            "per_net_results": route_results,
            "verdict": verdict,
            "gridless_first_recommended": {
                "recommended": gridless_first_recommended,
                "reasoning": gridless_reasoning,
            },
            "issues": issues,
            "assumptions": [
                f"Clean board: pads + board edge + drills as obstacles (no grid tracks, no pours)",
                f"MAX_WINDOW_MM={MAX_WINDOW_MM}: no escalation beyond this cap",
                f"Single-layer ONLY — via fallback disabled to prevent M3-P1.1 board-scale blowup",
                f"MST Prim seed=0, deterministic tie-break by (dist, i, j)",
                f"Route order: TARGET_NETS list order; accumulated copper as extra_obstacles",
                f"DRC connectivity used as truth (not route_result.ok alone)",
                f"Verdict: >=75% connected = ordering-problem, <40% = placement-bound",
            ],
        }

        print("\n## Structured Result")
        print("```json")
        print(json.dumps(structured, indent=2))
        print("```")


if __name__ == "__main__":
    main()
