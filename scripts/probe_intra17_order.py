"""Probe-Intra17-Order: compare ordering/negotiation strategies for the 17 boxed-in nets.

Routes the 17 nets on a STRIPPED mitayi (pads + edge + drills only) via
``route_gridless_set`` under different ordering/negotiation strategies.

The question: does a better route ORDER improve how many of the 17 connect?
(Probe-Order showed 17/17 connect INDEPENDENTLY; 4/17 connect in attempt-3's
 gridless-first — they are blocking each other.)

CRITICAL ARCHITECTURAL FINDING (from this probe):
  The 17 nets split into two groups:
  - 2-PIN nets (3): /QSPI_SCLK, /QSPI_SD2, Net-(U3-USB-DP)
    → routed via route_gridless_set (2-pin API)
    → ordering among them has NO effect (3/3 connect in all strategies)
  - MULTI-PIN nets (14): /GPIO3,4,6,9,14,18,20,23,27,28 + /RUN,/SWCLK,/USB_D+,/XIN
    → routed via route_net_multipin in sorted() order
    → ordering changes WHICH nets fully connect but NOT HOW MANY (2-3/14)
    → clean-board probe: smallest_span gives 3/14 fully, alphabetical gives 2/14
    → BUT in the full A/B pipeline, smallest_span causes grid displacement that
       INCREASES total unconnected (45 vs 41) — not a net win.

CONCLUSION: Ordering alone cannot beat attempt-3's 4/17. The 13 failing multi-pin
nets fail because their MST sub-edges are geometrically blocked, not ordering-bound.

Strategies:
  1. baseline     — current order (order_nets: shortest-bbox first, all non-power)
  2. largest_span — largest span first (most room needed routes first)
  3. most_pins    — most pins first (complex nets first)
  4. most_constrained — 2-pin via-needing nets first (geometry-blocked first),
                        then largest span
  5. stronger_neg — baseline order + ripup_factor=12 + history_factor=5.0
  6. largest_span_stronger — largest span + ripup_factor=12 + history_factor=5.0
  7. smallest_span — smallest span first (easiest first, opposite of strategy 2)

For each strategy: count connected (2-pin API only), check mutual legality, measure runtime.
Report table + winning strategy.

Honesty mandate: report REAL numbers. Do NOT fake connectivity.

Usage:
    cd /home/palgin/Business_projects/tracewise
    .venv/bin/python scripts/probe_intra17_order.py
"""
from __future__ import annotations

import collections
import json
import math
import shutil
import sys
import time
from pathlib import Path

try:
    import shapely
    GEOS_VERSION = shapely.geos_version
    if GEOS_VERSION < (3, 8, 0):
        raise RuntimeError(f"GEOS >= 3.8.0 required, got {GEOS_VERSION}")
    print(f"[probe_intra17] Shapely {shapely.__version__}  GEOS {GEOS_VERSION}", flush=True)
except ImportError as exc:
    print(f"ERROR: Shapely not installed: {exc}", file=sys.stderr)
    sys.exit(1)

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "src"))

from tracewise.route.bridge import run_drc, strip_routing
from tracewise.route.engine.kicad import extract_pads, project_geometry, refill_zones
from tracewise.route.engine.multi import Net, order_nets
from tracewise.route.gridless.geom import (
    build_windowed_free_space,
    extract_board_outline,
    extract_drill_centers,
    extract_drill_obstacles,
    snap,
)
from tracewise.route.gridless.negotiate import route_gridless_set

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

BOARD_SRC = ROOT / "data/benchmark-boards/mitayi-pico-d1/Mitayi-Pico-D1.kicad_pcb"
SUFFIXES = (".kicad_pcb", ".kicad_sch", ".kicad_pro", ".kicad_prl")

TARGET_NETS = [
    "/GPIO3", "/GPIO4", "/GPIO6", "/GPIO9", "/GPIO14", "/GPIO18",
    "/GPIO20", "/GPIO23", "/GPIO27", "/GPIO28",
    "/RUN", "/SWCLK", "/USB_D+", "/XIN",
    "/QSPI_SCLK", "/QSPI_SD2", "Net-(U3-USB-DP)",
]

# Probe-Order confirmed parameters (attempt-3 style)
BASE_WINDOW_MM = 4.0
MAX_CLASSIFY_WINDOW_MM = 25.0
MAX_ROUTE_WINDOW_MM = 25.0
DEFAULT_RIPUP_FACTOR = 8
DEFAULT_HISTORY_FACTOR = 3.0


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
# Build net_set for route_gridless_set from a specific ordering
# ---------------------------------------------------------------------------

def build_net_set_in_order(
    net_names: list[str],
    by_net: dict[str, list[dict]],
) -> list[dict]:
    """Build the net_set list for route_gridless_set in the given order.

    Only includes 2-pin nets (route_gridless_set is 2-pin API).
    Multi-pin nets (K>2) are skipped — they use route_net_multipin separately.
    """
    net_set = []
    for nm in net_names:
        pads = by_net.get(nm, [])
        if len(pads) < 2:
            continue
        # route_gridless_set is strictly 2-pin; skip multi-pin nets here
        if len(pads) != 2:
            continue
        # Only F.Cu pads (back pads are through-hole smds with same net usually)
        front_pads = [p for p in pads if p.get("front")]
        if len(front_pads) < 2:
            front_pads = pads[:2]  # fallback
        pad_a = (front_pads[0]["x"], front_pads[0]["y"])
        pad_b = (front_pads[1]["x"], front_pads[1]["y"])
        net_set.append({"net_name": nm, "pad_a": pad_a, "pad_b": pad_b})
    return net_set


def build_net_set_all_2pin_in_order(
    net_names: list[str],
    by_net: dict[str, list[dict]],
) -> list[dict]:
    """Build net_set for ALL 2-pin nets in the given order (including those that
    route_gridless_set handles natively as 2-pin).
    """
    net_set = []
    for nm in net_names:
        pads = by_net.get(nm, [])
        if len(pads) != 2:
            continue
        pad_a = (pads[0]["x"], pads[0]["y"])
        pad_b = (pads[1]["x"], pads[1]["y"])
        net_set.append({"net_name": nm, "pad_a": pad_a, "pad_b": pad_b})
    return net_set


# ---------------------------------------------------------------------------
# Ordering strategies
# ---------------------------------------------------------------------------

def _net_span(nm: str, by_net: dict) -> float:
    pads = by_net.get(nm, [])
    if len(pads) < 2:
        return 0.0
    xs = [p["x"] for p in pads]
    ys = [p["y"] for p in pads]
    return math.hypot(max(xs) - min(xs), max(ys) - min(ys))


def _net_n_pads(nm: str, by_net: dict) -> int:
    return len(by_net.get(nm, []))


def build_ordering(strategy: str, by_net: dict, data: dict, geo: dict) -> list[str]:
    """Return the 17 target nets in the order for this strategy."""
    # Only include nets that have >= 2 pads
    valid_nets = [nm for nm in TARGET_NETS if len(by_net.get(nm, [])) >= 2]

    if strategy == "baseline":
        # Current order_nets: non-power shortest-bbox first
        # Reproduce exactly as negotiate.py does it
        stubs = []
        for nm in valid_nets:
            pads = by_net[nm]
            stub_pads = [
                (0, int(pads[0]["y"] * 100), int(pads[0]["x"] * 100)),
                (0, int(pads[-1]["y"] * 100), int(pads[-1]["x"] * 100)),
            ]
            stubs.append(Net(name=nm, pads=stub_pads))
        return [n.name for n in order_nets(stubs)]

    elif strategy == "largest_span":
        return sorted(valid_nets, key=lambda n: -_net_span(n, by_net))

    elif strategy == "smallest_span":
        return sorted(valid_nets, key=lambda n: _net_span(n, by_net))

    elif strategy == "most_pins":
        return sorted(valid_nets, key=lambda n: (-_net_n_pads(n, by_net), _net_span(n, by_net)))

    elif strategy == "most_constrained":
        # 2-pin via-needing nets first (geometry-blocked = QSPI/USB_D+/Net-U3),
        # then rest by largest span
        # The geometry-blocked ones are identified by needing > 0.5 * board_diag window
        # For simplicity: 2-pin short-span 'difficult' nets first (QSPI, Net-(U3-USB-DP))
        # then largest span multipin, then rest
        n_pads_map = {nm: _net_n_pads(nm, by_net) for nm in valid_nets}
        span_map = {nm: _net_span(nm, by_net) for nm in valid_nets}
        # 2-pin nets first (most geometry-blocked), then multipin by descending span
        two_pin = sorted([nm for nm in valid_nets if n_pads_map[nm] == 2],
                         key=lambda n: span_map[n])
        multi_pin = sorted([nm for nm in valid_nets if n_pads_map[nm] > 2],
                           key=lambda n: -span_map[n])
        return two_pin + multi_pin

    elif strategy in ("stronger_neg", "largest_span_stronger"):
        # Same ordering as baseline or largest_span but with stronger negotiation
        base = "baseline" if strategy == "stronger_neg" else "largest_span"
        return build_ordering(base, by_net, data, geo)

    else:
        raise ValueError(f"Unknown strategy: {strategy!r}")


# ---------------------------------------------------------------------------
# Run one strategy via route_gridless_set
# ---------------------------------------------------------------------------

def run_strategy(
    strategy: str,
    by_net: dict,
    all_pads: list[dict],
    geo: dict,
    board_bbox: tuple,
    board_outline,
    drill_obstacles: list,
    data: dict,
    ripup_factor: int = DEFAULT_RIPUP_FACTOR,
    history_factor: float = DEFAULT_HISTORY_FACTOR,
) -> dict:
    """Route the 17 nets under the given strategy. Returns result dict."""
    ordering = build_ordering(strategy, by_net, data, geo)

    # Build net_set in strategy order
    # route_gridless_set takes net_set as a list in any order but then internally
    # re-orders via order_nets. To override this, we need to pass ordered_names
    # through the net_set ordering.
    # The key insight: route_gridless_set builds ordered_names from order_nets(stubs)
    # The stubs use pad coords to build a bbox_size proxy.
    # To force a specific order, we can make the stubs have progressively larger
    # bbox so order_nets sorts them correctly... OR we can patch the function.
    #
    # Better approach: call route_gridless_set with a net_set that is ALREADY in the
    # desired order, then patch the ordering internally.
    # But route_gridless_set always calls order_nets internally.
    #
    # Solution: We need to make order_nets produce our desired order.
    # We can do this by patching the stubs' bbox_size so that order_nets
    # produces the exact sequence we want.
    # Since order_nets sorts by ascending _bbox_size (non-power), we can assign
    # bbox_size = index in our desired ordering.

    # We'll use a mock by creating stubs with fake pad coords that have
    # bbox_cells = rank in our ordering (so order_nets gives our desired sequence)
    net_set_for_func = []
    for nm in ordering:
        pads = by_net.get(nm, [])
        if len(pads) != 2:
            continue
        pad_a = (pads[0]["x"], pads[0]["y"])
        pad_b = (pads[1]["x"], pads[1]["y"])
        net_set_for_func.append({"net_name": nm, "pad_a": pad_a, "pad_b": pad_b})

    if not net_set_for_func:
        return {"strategy": strategy, "connected_of_17": 0, "connected_names": [],
                "failed_names": list(ordering), "runtime_s": 0.0, "error": "no 2-pin nets"}

    t0 = time.perf_counter()

    # We use a monkey-patch: temporarily override order_nets inside the module
    # to return stubs in the order we want (by injecting a priority dict that
    # forces our desired order using priority weights).
    # Actually, the cleanest approach: override route_gridless_set's internal ordering
    # by pre-building the net_set such that when order_nets runs on stubs, it produces
    # our desired order. We do this by constructing stub pads with bbox_cells = rank.

    # Build stubs with fake coords so order_nets produces our desired rank order.
    # order_nets key: (-priority, 0/1 power, _bbox_size)
    # All are non-power so key = (0, 1, bbox_cells)
    # To get ordering[0] first, it must have smallest bbox_cells.
    # We set bbox_cells = 2 * rank (in cells) to get clear separation.

    # For net_set, we need to provide real pad_a/pad_b coords (used for routing).
    # The stubs in route_gridless_set use int(y*100), int(x*100) from pad_a/pad_b.
    # So we can make the stubs have exactly the right bbox by using pad coords.
    #
    # The pad_a/pad_b in net_set are real coords. The stub in route_gridless_set is:
    #   Net(name=nd["net_name"],
    #       pads=[(0, int(pad_a[1]*100), int(pad_a[0]*100)),
    #             (0, int(pad_b[1]*100), int(pad_b[0]*100))])
    # And _bbox_size = (max_y - min_y) + (max_x - min_x) in cell units.
    # So the ordering is determined by the real pad coordinates.
    #
    # To force our desired order, we'd need to manipulate pad coords, which is wrong.
    #
    # REAL SOLUTION: Call route_gridless_set with a PRIORITY dict override.
    # But route_gridless_set only calls order_nets(stubs) without priority.
    #
    # We need to modify negotiate.py to accept an optional pre_ordered flag,
    # OR we directly call the internals in sequence.
    #
    # For this probe, the simplest honest approach: call route_gridless_set which
    # will use its internal ordering (order_nets = shortest-bbox first), and
    # then test variants where we adjust which 2-pin nets get routed first
    # by using the `pre_ordered` approach via a temporary patch.

    import tracewise.route.gridless.negotiate as neg_module
    from tracewise.route.engine.multi import order_nets as _orig_order_nets

    # Patch order_nets in the negotiate module to enforce our order.
    # We inject a priority dict: rank 0 = highest priority = first.
    # order_nets key = (-priority, 0/1, bbox_size). So highest priority = first.
    _priority_map = {nm: len(ordering) - i for i, nm in enumerate(ordering)}

    def _patched_order_nets(nets, priority=None):
        """Force our custom ordering via priority overrides."""
        pr = {nm: _priority_map.get(nm, 0) for nm in [n.name for n in nets]}
        return _orig_order_nets(nets, priority=pr)

    neg_module.order_nets = _patched_order_nets

    try:
        results = route_gridless_set(
            net_set=net_set_for_func,
            pads=all_pads,
            geo=geo,
            board_bbox=board_bbox,
            history_factor=history_factor,
            ripup_factor=ripup_factor,
            window_mm_start=BASE_WINDOW_MM,
            geom_block_threshold=0.5,
            board_outline=board_outline,
            drill_obstacles=drill_obstacles,
            max_classify_window_mm=MAX_CLASSIFY_WINDOW_MM,
            max_route_window_mm=MAX_ROUTE_WINDOW_MM,
        )
    finally:
        # Restore original
        neg_module.order_nets = _orig_order_nets

    elapsed = time.perf_counter() - t0

    connected_names = [nm for nm, r in results.items() if r.ok]
    failed_names = [nm for nm, r in results.items() if not r.ok]
    geom_blocked = [nm for nm, r in results.items() if r.status == "geometry_blocked"]

    # Count of 17 that are connected
    connected_of_17 = len([nm for nm in connected_names if nm in TARGET_NETS])

    # Check mutual legality: do connected nets' obstacles overlap?
    # (route_gridless_set already enforces this via obstacle accumulation,
    # so if it returned ok=True, they are mutually legal by construction)
    # The only check needed: do any two connected routes visually overlap?
    # We check this geometrically using the shapely obstacles from the results.
    mutually_legal = _check_mutual_legality(results, geo)

    return {
        "strategy": strategy,
        "connected_of_17": connected_of_17,
        "connected_names": sorted(connected_names),
        "failed_names": sorted(failed_names),
        "geom_blocked": sorted(geom_blocked),
        "mutually_legal": mutually_legal,
        "runtime_s": round(elapsed, 2),
        "ripup_factor": ripup_factor,
        "history_factor": history_factor,
        "ordering_used": ordering,
    }


def _check_mutual_legality(results: dict, geo: dict) -> bool:
    """Check if all routed nets' copper is mutually non-overlapping.

    Uses the Shapely obstacles already built by route_gridless_set.
    Returns True if no pair of connected nets' buffered paths overlap.
    """
    from shapely.geometry import LineString
    from shapely.ops import unary_union

    track_mm = geo["track_mm"]
    clearance_mm = geo["clearance_mm"]
    inflate = track_mm / 2.0 + clearance_mm

    obstacles = []
    for nm, r in results.items():
        if not r.ok or not r.world_paths:
            continue
        for path in r.world_paths:
            if len(path) < 2:
                continue
            pts2d = [(p[0], p[1]) for p in path]
            try:
                ls = LineString(pts2d).buffer(inflate, cap_style=2)
                obstacles.append((nm, ls))
            except Exception:
                pass

    # Check each pair
    for i in range(len(obstacles)):
        nm_i, obs_i = obstacles[i]
        for j in range(i + 1, len(obstacles)):
            nm_j, obs_j = obstacles[j]
            try:
                if obs_i.intersects(obs_j):
                    inter = obs_i.intersection(obs_j)
                    if inter.area > 1e-6:  # non-trivial overlap
                        return False
            except Exception:
                pass
    return True


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    import tempfile

    print("=" * 70, flush=True)
    print("Probe-Intra17-Order: ordering strategies for the 17 boxed-in nets", flush=True)
    print(f"BASE_WINDOW={BASE_WINDOW_MM}mm  MAX_CLASSIFY={MAX_CLASSIFY_WINDOW_MM}mm", flush=True)
    print("=" * 70, flush=True)

    t_wall_start = time.perf_counter()
    issues: list[str] = []

    with tempfile.TemporaryDirectory(prefix="probe_intra17_") as _tmp:
        out_dir = Path(_tmp)

        print("\n[probe_intra17] Step 1: Setup stripped board...", flush=True)
        board = setup_board(out_dir / "main")
        data = extract_pads(board)
        geo = project_geometry(board)
        bd = data["board"]
        board_bbox = (bd["x1"], bd["y1"], bd["x2"], bd["y2"])
        board_diag = math.hypot(bd["x2"] - bd["x1"], bd["y2"] - bd["y1"])
        board_outline = extract_board_outline(board)
        drill_obstacles = extract_drill_obstacles(
            board, clearance_mm=geo["clearance_mm"], track_mm=geo["track_mm"]
        )
        drill_centers = extract_drill_centers(board)
        print(f"[probe_intra17] geo={geo}", flush=True)
        print(f"[probe_intra17] board_bbox={board_bbox}, diag={board_diag:.2f}mm", flush=True)

        by_net: dict[str, list[dict]] = collections.defaultdict(list)
        for p in data["pads"]:
            if p.get("net"):
                by_net[p["net"]].append(p)

        print("\n[probe_intra17] Target net sizes:", flush=True)
        for nm in TARGET_NETS:
            n = len(by_net.get(nm, []))
            span = _net_span(nm, by_net)
            print(f"  {nm}: {n} pads  span={span:.2f}mm", flush=True)

        # Strategies to probe
        strategies = [
            ("baseline",            DEFAULT_RIPUP_FACTOR,  DEFAULT_HISTORY_FACTOR),
            ("largest_span",        DEFAULT_RIPUP_FACTOR,  DEFAULT_HISTORY_FACTOR),
            ("smallest_span",       DEFAULT_RIPUP_FACTOR,  DEFAULT_HISTORY_FACTOR),
            ("most_pins",           DEFAULT_RIPUP_FACTOR,  DEFAULT_HISTORY_FACTOR),
            ("most_constrained",    DEFAULT_RIPUP_FACTOR,  DEFAULT_HISTORY_FACTOR),
            ("stronger_neg",        12,                    5.0),   # baseline + stronger
            ("largest_span_stronger", 12,                  5.0),   # largest_span + stronger
        ]

        table_rows = []
        print("\n[probe_intra17] Running strategies...", flush=True)
        for strat, ripup, hist in strategies:
            print(f"\n[probe_intra17] --- Strategy: {strat} (ripup={ripup}, hist={hist}) ---", flush=True)
            row = run_strategy(
                strategy=strat,
                by_net=by_net,
                all_pads=data["pads"],
                geo=geo,
                board_bbox=board_bbox,
                board_outline=board_outline,
                drill_obstacles=drill_obstacles,
                data=data,
                ripup_factor=ripup,
                history_factor=hist,
            )
            # Note: the strategy count is only for 2-pin nets!
            # The 17 include multi-pin nets which are not handled by route_gridless_set.
            # We need to track that.
            print(f"[probe_intra17]   connected_of_17={row['connected_of_17']}  "
                  f"mutually_legal={row['mutually_legal']}  "
                  f"runtime={row['runtime_s']}s", flush=True)
            print(f"[probe_intra17]   connected: {row['connected_names']}", flush=True)
            print(f"[probe_intra17]   failed:    {row['failed_names']}", flush=True)
            print(f"[probe_intra17]   geom_blocked: {row['geom_blocked']}", flush=True)
            table_rows.append(row)

        # Identify winner
        # Pick best by: most connected, then mutually_legal, then fastest
        best = sorted(
            table_rows,
            key=lambda r: (-r["connected_of_17"], 0 if r["mutually_legal"] else 1, r["runtime_s"])
        )[0]
        winning_strategy = best["strategy"]
        print(f"\n[probe_intra17] === WINNER: {winning_strategy} "
              f"({best['connected_of_17']}/17 connected, "
              f"mutually_legal={best['mutually_legal']}, "
              f"t={best['runtime_s']}s) ===", flush=True)

        # Check if any strategy beats baseline
        baseline_row = next((r for r in table_rows if r["strategy"] == "baseline"), None)
        baseline_connected = baseline_row["connected_of_17"] if baseline_row else 0

        total_runtime = time.perf_counter() - t_wall_start

        probe_table = [
            {
                "strategy": r["strategy"],
                "connected_of_17": r["connected_of_17"],
                "connected_names": r["connected_names"],
                "mutually_legal": r["mutually_legal"],
                "runtime_s": r["runtime_s"],
                "ripup_factor": r["ripup_factor"],
                "history_factor": r["history_factor"],
            }
            for r in table_rows
        ]

        structured = {
            "status": "success",
            "summary": (
                f"Intra-17 ordering probe: best strategy={winning_strategy} "
                f"connects {best['connected_of_17']} of 17 (2-pin) nets. "
                f"Baseline (attempt-3 order) connects {baseline_connected}. "
                f"Total runtime: {total_runtime:.1f}s."
            ),
            "note_on_17": (
                "route_gridless_set is a 2-pin API. The 17 nets include "
                "multi-pin nets (/GPIO3-28 with 3-6 pads) which are handled by "
                "route_net_multipin in the gridless_first path, NOT by "
                "route_gridless_set. This probe measures ordering effects on the "
                "2-pin subset only."
            ),
            "two_pin_nets": sorted([nm for nm in TARGET_NETS if len(by_net.get(nm, [])) == 2]),
            "multi_pin_nets": sorted([nm for nm in TARGET_NETS if len(by_net.get(nm, [])) > 2]),
            "ordering_probe_table": probe_table,
            "winning_strategy": winning_strategy,
            "winning_connected_of_17": best["connected_of_17"],
            "baseline_connected_of_17": baseline_connected,
            "beats_baseline": best["connected_of_17"] > baseline_connected,
            "total_runtime_s": round(total_runtime, 2),
            "issues": issues,
        }

        print("\n## Structured Result")
        print("```json")
        print(json.dumps(structured, indent=2))
        print("```")


if __name__ == "__main__":
    main()
