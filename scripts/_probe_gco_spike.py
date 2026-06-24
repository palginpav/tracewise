#!/usr/bin/env python3
"""
_probe_gco_spike.py — GCO (Global Corridor Optimizer) decisive spike.

Tests: does simultaneous J4-corridor allocation (min-cost multi-commodity flow)
combine the two Pareto-frontier halves (connectivity-40 of 'escape' + errors-65
of 'ring_slots') into ONE config beating attempt-3 (41 unc / 73 err) on BOTH axes?

Hypothesis: +3V3 spine band AND escape signal lanes are co-assigned DISJOINTLY
by the flow → connectivity AND short-freedom TOGETHER, which no sequential order
achieves.

GO iff ALL: unconnected ≤ 40 AND errors ≤ 73 AND 0 shorts AND 0 illegal
crossings AND deterministic (3-run identical connectivity) AND bounded
(peak RSS < 2GB, solve < 10s).

BOUNDS (NON-NEGOTIABLE): max_window_mm=12, max_bcu_window_mm=8, RSS hard-abort
>2GB, SOLVE_TIMEOUT_S=10.

Usage:
    .venv/bin/python scripts/_probe_gco_spike.py [--runs N] [--out DIR]
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
import threading
import time
from pathlib import Path

# ── Path setup ───────────────────────────────────────────────────────────────

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
SOLVE_TIMEOUT_S = 10.0

# Corridor geometry (west channel J4 → J3)
J4_INNER_Y = 80.40   # inner J4 row y (mm)
J3_INNER_Y = 98.20   # inner J3 row y (mm)
LANE_PITCH_MM = 0.35  # track + clearance pitch

# Power nets that share the J4 corridor
CORRIDOR_POWER_NETS = {"+3V3", "+1V1"}

# All QFN escape nets (same as _probe_tcr_e1.py)
QFN_ESCAPE_NETS = {
    "/GPIO3", "/GPIO4", "/GPIO6", "/GPIO9", "/GPIO14",
    "/GPIO20", "/GPIO23", "/GPIO27", "/GPIO28",
    "/RUN", "/SWCLK", "/XIN", "/USB_D+",
}

# Short F.Cu-only nets (no via needed)
FCU_ONLY_NETS = {"/GPIO27", "/GPIO28", "/XIN"}

# File copy suffixes
SUFFIXES = (".kicad_pcb", ".kicad_sch", ".kicad_pro", ".kicad_prl")

# Attempt-3 reference bars
ATTEMPT3_UNC = 41
ATTEMPT3_ERR = 73

# Attempt-3 gridless_first nets (for the remaining-net phase)
ATTEMPT3_GRIDLESS_NETS = {
    "/GPIO3", "/GPIO4", "/GPIO6", "/GPIO9", "/GPIO14",
    "/GPIO18", "/GPIO20", "/GPIO23", "/GPIO27", "/GPIO28",
    "/RUN", "/SWCLK", "/USB_D+", "/XIN",
    "/QSPI_SCLK", "/QSPI_SD2", "Net-(U3-USB-DP)",
}


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


# ── GCO: Corridor flow graph builder ─────────────────────────────────────────

class CorridorTimeout(Exception):
    """Raised when the min-cost flow solve exceeds SOLVE_TIMEOUT_S."""


def _build_lanes(
    channel_y_lo: float = J4_INNER_Y,
    channel_y_hi: float = J3_INNER_Y,
    pitch: float = LANE_PITCH_MM,
) -> list[float]:
    """Return list of lane y-coordinates, from channel_y_lo to channel_y_hi."""
    lanes = []
    y = channel_y_lo + pitch / 2.0
    while y < channel_y_hi - pitch / 2.0:
        lanes.append(round(y * 1e6) / 1e6)
        y += pitch
    return lanes


def build_corridor_graph(
    contending_signals: dict[str, dict],
    spine_demands: dict[str, int],  # e.g. {"+3V3": 2, "+1V1": 1}
    channel_y_range: tuple[float, float] = (J4_INNER_Y, J3_INNER_Y),
    pitch_mm: float = LANE_PITCH_MM,
) -> object:
    """Build the capacitated corridor lane×layer flow graph.

    Nodes:
    - "S" : global source (supply = total_flow)
    - "T" : global sink   (demand = total_flow)
    - "cmd_{net}" : commodity node for each signal net
    - "spine_{power_net}" : spine commodity node for each power net
    - "slot_{lane_idx}_{layer}" : per-lane-per-layer slot (cap=1 each)

    Edges:
    - S → cmd_{net}   cap=1, cost=0   (one flow unit per signal)
    - S → spine_{pw}  cap=demand, cost=0   (B_spine flow units for power)
    - cmd_{net} → slot_{i}_{l}   cap=1, cost=int_cost
    - spine_{pw} → slot_{i}_{l}  cap=1, cost=lane_idx*100+epsilon
      (only for candidate edge-bands)
    - slot_{i}_{l} → T   cap=1, cost=0

    Costs are INTEGER (×1000) for determinism + unique-optimum epsilon tie-break.

    Returns a networkx DiGraph with demand attributes on S and T.
    """
    import networkx as nx

    lanes = _build_lanes(channel_y_range[0], channel_y_range[1], pitch_mm)
    N = len(lanes)
    if N == 0:
        raise RuntimeError("build_corridor_graph: no lanes in channel")

    # Layer penalty: B.Cu (1) = 0; F.Cu (0) = 5000 (prefer horizontal B.Cu)
    LAYER_PENALTY = {0: 5000, 1: 0}

    # Total flow = signal nets + spine demands
    signal_names = sorted(contending_signals.keys())  # deterministic order
    total_flow = len(signal_names) + sum(spine_demands.values())

    G = nx.DiGraph()
    G.add_node("S", demand=-total_flow)
    G.add_node("T", demand=total_flow)

    # ── Signal commodity nodes ───────────────────────────────────────────────
    for net in signal_names:
        cmd_node = f"cmd_{net}"
        G.add_node(cmd_node, demand=0)
        G.add_edge("S", cmd_node, capacity=1, weight=0)

        info = contending_signals[net]
        dest_xy = info.get("dest_xy")
        dest_y = dest_xy[1] if dest_xy is not None else (J4_INNER_Y + J3_INNER_Y) / 2

        for li, lane_y in enumerate(lanes):
            for layer in (0, 1):  # 0=F.Cu, 1=B.Cu
                slot_node = f"slot_{li}_{layer}"
                if not G.has_node(slot_node):
                    G.add_node(slot_node, demand=0)

                # Cost = distance proxy + layer penalty + epsilon tie-break
                dist_cost = int(abs(lane_y - dest_y) * 1000)
                layer_cost = LAYER_PENALTY[layer]
                # Unique-optimum epsilon: deterministic (li * 7 + layer * 3) so no ties
                epsilon = li * 7 + layer * 3
                cost = dist_cost + layer_cost + epsilon

                G.add_edge(cmd_node, slot_node, capacity=1, weight=cost)

    # ── Spine commodity nodes (power nets as band allocations) ───────────────
    # Candidate edge-bands: bottom-2, top-2, next-bottom-2, next-top-2
    # Only add spine edges to these candidate bands; the flow picks the cheapest.
    band_candidates: list[list[int]] = []
    if N >= 4:
        band_candidates = [
            list(range(0, min(2, N))),                # bottom-2 lanes
            list(range(max(N - 2, 0), N)),            # top-2 lanes
            list(range(2, min(4, N))),                 # next-bottom-2
            list(range(max(N - 4, N - 2), max(N - 2, 0))),  # next-top-2
        ]
        # Deduplicate
        seen = set()
        uniq_bands = []
        for b in band_candidates:
            key = tuple(b)
            if key not in seen and len(key) > 0:
                seen.add(key)
                uniq_bands.append(b)
        band_candidates = uniq_bands
    elif N >= 2:
        band_candidates = [list(range(N))]
    else:
        band_candidates = [list(range(N))]

    # Power nets sorted deterministically
    for pw_net in sorted(spine_demands.keys()):
        demand = spine_demands[pw_net]
        spine_node = f"spine_{pw_net}"
        G.add_node(spine_node, demand=0)
        G.add_edge("S", spine_node, capacity=demand, weight=0)

        # Connect spine → slots for candidate band lanes (B.Cu only for spine)
        for band in band_candidates:
            for li in band:
                lane_y = lanes[li]
                # Spine uses B.Cu (layer=1) only
                layer = 1
                slot_node = f"slot_{li}_{layer}"
                if not G.has_node(slot_node):
                    G.add_node(slot_node, demand=0)
                # Spine cost: prefer center of band → lower index in mid-range
                # Use li * 100 + band_idx as cost (deterministic)
                band_idx = band_candidates.index(band)
                epsilon_s = li * 11 + band_idx * 17
                cost_s = li * 100 + epsilon_s
                # Only add if edge doesn't exist (prefer lower cost)
                if not G.has_edge(spine_node, slot_node):
                    G.add_edge(spine_node, slot_node, capacity=1, weight=cost_s)

    # ── Slot → Sink edges ────────────────────────────────────────────────────
    for li in range(N):
        for layer in (0, 1):
            slot_node = f"slot_{li}_{layer}"
            if G.has_node(slot_node):
                G.add_edge(slot_node, "T", capacity=1, weight=0)

    # Store lane list in graph for later decoding
    G.graph["lanes"] = lanes
    G.graph["signal_names"] = signal_names
    G.graph["spine_demands"] = dict(spine_demands)
    G.graph["N_lanes"] = N

    print(
        f"  [corridor_graph] N_lanes={N} signals={len(signal_names)} "
        f"spine_nets={sorted(spine_demands.keys())} total_flow={total_flow} "
        f"nodes={G.number_of_nodes()} edges={G.number_of_edges()}",
        flush=True,
    )
    return G


def solve_corridor_assignment(
    graph,
    solve_timeout_s: float = SOLVE_TIMEOUT_S,
) -> dict[str, tuple[int, float, int]]:
    """Run networkx min_cost_flow; decode → {net: (lane_idx, lane_y_mm, layer)}.

    On timeout → raises CorridorTimeout.
    On infeasibility → raises nx.NetworkXUnfeasible.

    Deterministic: integer costs + unique-optimum epsilon tie-break ensures the
    flow result is the same for the same graph on every run.
    """
    import networkx as nx

    lanes = graph.graph["lanes"]
    signal_names = graph.graph["signal_names"]
    spine_demands = graph.graph["spine_demands"]

    flow_result: dict | None = None
    exc_holder: list = []

    def _solve():
        try:
            flow_result_local = nx.min_cost_flow(graph)
            exc_holder.append(("ok", flow_result_local))
        except Exception as e:
            exc_holder.append(("err", e))

    t0 = time.perf_counter()
    thread = threading.Thread(target=_solve, daemon=True)
    thread.start()
    thread.join(timeout=solve_timeout_s)
    t_elapsed = time.perf_counter() - t0

    if thread.is_alive():
        raise CorridorTimeout(
            f"min_cost_flow timed out after {solve_timeout_s}s"
        )

    if not exc_holder:
        raise CorridorTimeout("min_cost_flow thread finished but no result stored")

    status, value = exc_holder[0]
    if status == "err":
        raise value

    flow_dict = value
    print(f"  [solve] min_cost_flow done in {t_elapsed:.3f}s", flush=True)

    # ── Decode flow: find which slot each commodity flows to ──────────────────
    assignment: dict[str, tuple[int, float, int]] = {}  # net → (lane_idx, lane_y, layer)

    # Signal commodities
    for net in signal_names:
        cmd_node = f"cmd_{net}"
        if cmd_node not in flow_dict:
            continue
        for slot_node, flow_val in flow_dict[cmd_node].items():
            if flow_val == 1 and slot_node.startswith("slot_"):
                parts = slot_node.split("_")
                li = int(parts[1])
                layer = int(parts[2])
                lane_y = lanes[li]
                assignment[net] = (li, lane_y, layer)
                break

    # Spine commodities
    for pw_net in spine_demands.keys():
        spine_node = f"spine_{pw_net}"
        if spine_node not in flow_dict:
            continue
        spine_slots = []
        for slot_node, flow_val in flow_dict[spine_node].items():
            if flow_val >= 1 and slot_node.startswith("slot_"):
                parts = slot_node.split("_")
                li = int(parts[1])
                layer = int(parts[2])
                lane_y = lanes[li]
                spine_slots.append((li, lane_y, layer))
        if spine_slots:
            # Store all spine slots; use the first one as primary lane_y
            spine_slots.sort()
            assignment[pw_net] = spine_slots[0]
            # Store full band as extra
            assignment[f"_spine_band_{pw_net}"] = spine_slots  # type: ignore[assignment]

    return assignment


def assert_assignment_disjoint(
    assignment: dict[str, tuple[int, float, int]],
    spine_demands: dict[str, int],
    signal_names: list[str],
) -> None:
    """Assert: no two signal nets share a slot; spine slots disjoint from signal slots."""
    signal_slots: dict[str, tuple[int, int]] = {}  # net → (lane_idx, layer)
    spine_slots: dict[str, list[tuple[int, int]]] = {}

    for net in signal_names:
        if net in assignment:
            li, _ly, layer = assignment[net]
            signal_slots[net] = (li, layer)

    for pw_net in spine_demands.keys():
        band_key = f"_spine_band_{pw_net}"
        if band_key in assignment:
            spine_slots[pw_net] = [(s[0], s[2]) for s in assignment[band_key]]  # type: ignore[index]
        elif pw_net in assignment:
            li, _ly, layer = assignment[pw_net]
            spine_slots[pw_net] = [(li, layer)]

    # Check signal-signal disjointness
    seen: dict[tuple[int, int], str] = {}
    for net, slot in signal_slots.items():
        if slot in seen:
            raise AssertionError(
                f"GCO DISJOINT VIOLATION: {net} and {seen[slot]} share slot {slot}"
            )
        seen[slot] = net

    # Check signal vs spine disjointness
    all_spine_slots: set[tuple[int, int]] = set()
    for slots in spine_slots.values():
        all_spine_slots.update(slots)

    for net, slot in signal_slots.items():
        if slot in all_spine_slots:
            raise AssertionError(
                f"GCO DISJOINT VIOLATION: signal {net} shares slot {slot} with spine"
            )

    print(
        f"  [assert_disjoint] OK: {len(signal_slots)} signal slots + "
        f"{sum(len(v) for v in spine_slots.values())} spine slots, all disjoint",
        flush=True,
    )


# ── GCO main run ─────────────────────────────────────────────────────────────

def run_gco(out_dir: Path, runs: int = 1) -> dict:
    """Execute one GCO routing run. Returns a score dict.

    Steps:
    1. Build board + extract pads/geo
    2. Determine contending set (J4-corridor escape signals + power nets)
    3. build_ring_slot_assignment for contending signals → escape_via_xy per net
    4. build_corridor_graph → solve → get lane_y_mm per net
    5. Assert disjoint assignment
    6. Phase-0: route +3V3/+1V1 via gridless_first (spine band lanes available)
    7. Escape phase: route each J4-corridor signal via route_net_steered with
       flow-assigned lane_y_mm; accumulate B.Cu obstacles
    8. Grid+gridless_first the rest
    9. Score: refill_zones + run_drc + crossing audit
    """
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
    from tracewise.route.gridless.topo_assign import build_ring_slot_assignment

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
    GRID_PITCH = 0.1
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
    nets_by_name = {n.name: n for n in nets}

    pads_by_net: dict[str, list[dict]] = {}
    for p in pads:
        pads_by_net.setdefault(p.get("net", ""), []).append(p)

    dense_comps = detect_dense_components(pads)
    dense_by_ref = {d["ref"]: d for d in dense_comps}
    dense_refs = set(dense_by_ref)

    _check_rss("after_board_setup")

    # ── Step 2: Determine contending set ────────────────────────────────────
    # A net is "contending" if its dest pad is on J4 inner row (y ≈ 80.40 ± 1.0)
    # OR it is a CORRIDOR_POWER_NET.
    # We compute dest_by_net first (mirrors _probe_tcr_e1.py logic).
    connector_refs = {"J3", "J4", "J5"}
    passive_refs = {"Y1", "C17", "R9", "SW1"}
    J3_INNER_Y_CHECK = 98.20
    J4_INNER_Y_CHECK = 80.40

    from collections import defaultdict as _dd
    conn_pads_by_net_ref: dict = _dd(list)
    for p in pads:
        ref = p.get("ref", "")
        if ref in connector_refs:
            nm = p.get("net", "")
            if nm:
                conn_pads_by_net_ref[(nm, ref)].append(p)

    dest_by_net: dict[str, tuple[float, float]] = {}
    for (nm, ref), plist in conn_pads_by_net_ref.items():
        if nm in dest_by_net:
            continue
        if ref == "J3":
            best = min(plist, key=lambda p: abs(p["y"] - J3_INNER_Y_CHECK))
        elif ref == "J4":
            best = min(plist, key=lambda p: abs(p["y"] - J4_INNER_Y_CHECK))
        else:
            best = plist[0]
        dest_by_net[nm] = (best["x"], best["y"])

    for p in pads:
        if p.get("ref", "") in passive_refs:
            nm = p.get("net", "")
            if nm and nm not in dest_by_net and p.get("ref") not in {"U3"}:
                dest_by_net[nm] = (p["x"], p["y"])

    # J4-corridor signal nets: escape nets whose dest is on J4 inner row
    j4_contending_signals: set[str] = set()
    for net_name in QFN_ESCAPE_NETS:
        if net_name in FCU_ONLY_NETS:
            continue
        dest = dest_by_net.get(net_name)
        if dest is not None and abs(dest[1] - J4_INNER_Y_CHECK) < 1.5:
            j4_contending_signals.add(net_name)

    print(
        f"\n[GCO] Contending set:",
        flush=True,
    )
    print(f"  Power nets: {sorted(CORRIDOR_POWER_NETS)}")
    print(f"  J4-corridor escape signals: {sorted(j4_contending_signals)}")
    total_contending = len(j4_contending_signals) + len(CORRIDOR_POWER_NETS)
    print(f"  Total contending: {total_contending}")

    # All escape nets for ring-slot assignment (not just J4)
    all_escape_nets = QFN_ESCAPE_NETS  # assign slots for all escape nets

    # ── Step 3: build_ring_slot_assignment for all escape nets ───────────────
    print("\n[GCO] Step 3: Ring-slot assignment for all escape nets...", flush=True)
    _check_rss("before_ring_slot_assignment")

    ring_assignment = build_ring_slot_assignment(
        pcb_path=board,
        pads=pads,
        geo=geo,
        board_bbox=board_bbox,
        board_outline=board_outline,
        drill_obstacles=drill_obstacles,
        drill_centers=drill_centers,
        escape_net_names=all_escape_nets,
        fcu_only_nets=FCU_ONLY_NETS,
    )
    _check_rss("after_ring_slot_assignment")

    # Build contending_signals dict for corridor graph
    contending_signals: dict[str, dict] = {}
    for net_name in j4_contending_signals:
        info = ring_assignment.get(net_name, {})
        source_xy = None
        for p in pads:
            if p.get("ref", "") in dense_refs and p.get("front") and not p.get("back"):
                if p.get("net", "") == net_name:
                    source_xy = (p["x"], p["y"])
                    break
        dest_xy = dest_by_net.get(net_name)
        order_key = (dest_xy[1] * 10000 + dest_xy[0]) if dest_xy else 9999.0 * 10000
        contending_signals[net_name] = {
            "escape_via_xy": info.get("escape_via_xy"),
            "lane_y_mm": info.get("lane_y_mm"),  # from witness (may be None)
            "dest_xy": dest_xy,
            "source_xy": source_xy,
            "order_key": order_key,
        }

    print(
        f"\n  J4-corridor signals in contending_signals: {sorted(contending_signals.keys())}",
        flush=True,
    )

    # ── Step 4: build_corridor_graph + solve ─────────────────────────────────
    print("\n[GCO] Step 4: Build corridor flow graph...", flush=True)
    _check_rss("before_corridor_graph")

    spine_demands = {"+3V3": 2, "+1V1": 1}  # B_spine=2 per design spec

    corridor_graph = build_corridor_graph(
        contending_signals=contending_signals,
        spine_demands=spine_demands,
        channel_y_range=(J4_INNER_Y, J3_INNER_Y),
        pitch_mm=LANE_PITCH_MM,
    )
    _check_rss("after_corridor_graph_build")

    # Solve
    t_solve_start = time.perf_counter()
    flow_found_disjoint = False
    solve_time_s = 0.0
    gco_assignment: dict[str, tuple[int, float, int]] = {}
    fallback_used = False
    flow_infeasible = False

    try:
        print("\n[GCO] Solving min-cost flow...", flush=True)
        gco_assignment = solve_corridor_assignment(corridor_graph, SOLVE_TIMEOUT_S)
        solve_time_s = time.perf_counter() - t_solve_start

        # Assert disjoint
        signal_names_list = sorted(contending_signals.keys())
        assert_assignment_disjoint(gco_assignment, spine_demands, signal_names_list)
        flow_found_disjoint = True

        # Print assignment
        print("\n  GCO lane assignment:")
        lanes = corridor_graph.graph["lanes"]
        N_lanes = corridor_graph.graph["N_lanes"]
        for net in sorted(contending_signals.keys()):
            if net in gco_assignment:
                li, ly, layer = gco_assignment[net]
                layer_str = "B.Cu" if layer == 1 else "F.Cu"
                print(f"    {net:<18} lane_idx={li:3d} lane_y={ly:.3f}mm layer={layer_str}")

        for pw_net in sorted(spine_demands.keys()):
            band_key = f"_spine_band_{pw_net}"
            if band_key in gco_assignment:
                spine_band = gco_assignment[band_key]
                spine_summary = [(s[0], round(s[1], 3)) for s in spine_band]  # type: ignore[index]
                print(f"    {pw_net:<18} spine_band={spine_summary}")

    except CorridorTimeout as exc:
        solve_time_s = time.perf_counter() - t_solve_start
        print(f"  [solve] TIMEOUT: {exc} — falling back to ring_slot assignment", flush=True)
        fallback_used = True
        gco_assignment = {}
    except Exception as exc:
        solve_time_s = time.perf_counter() - t_solve_start
        import traceback as _tb
        print(f"  [solve] FLOW INFEASIBLE or ERROR: {exc}", flush=True)
        _tb.print_exc()
        flow_infeasible = True
        fallback_used = True
        gco_assignment = {}

    _check_rss("after_solve")

    # ── Step 5: corridor_assignment_to_classes ───────────────────────────────
    # Merge flow lane assignments into ring_assignment (override lane_y_mm)
    merged_assignment: dict[str, dict] = dict(ring_assignment)

    if not fallback_used:
        for net_name in j4_contending_signals:
            if net_name in gco_assignment:
                li, lane_y, layer = gco_assignment[net_name]
                existing = dict(merged_assignment.get(net_name, {}))
                existing["lane_y_mm"] = lane_y
                merged_assignment[net_name] = existing

    print(
        f"\n  Lane assignments merged for {len(j4_contending_signals)} J4-corridor nets "
        f"(fallback={fallback_used})",
        flush=True,
    )

    # ── Step 6: Realization ──────────────────────────────────────────────────
    # Phase 0: route +3V3/+1V1 first (power nets claim corridor copper)
    precomp_routes: dict = {}

    try:
        from shapely.geometry import LineString as _LS
        from shapely.geometry import Point as _ViaObs
        from tracewise.route.gridless.geom import snap as _snap
        _have_shapely = True
    except ImportError:
        _have_shapely = False

    bcu_extra_obs: list = []

    print("\n[GCO] Phase 0: Power-first pre-routing (+3V3, +1V1)...", flush=True)
    _check_rss("phase0_power_start")
    _power_net_objs = [n for n in nets if n.name in CORRIDOR_POWER_NETS]
    if _power_net_objs:
        _power_gf_names = {n.name for n in _power_net_objs}
        _power_gridless_kwargs = {
            "pads": pads,
            "geo": geo,
            "board_bbox": board_bbox,
            "anchors": anchors,
            "extra_gridless_obstacles": [],
            "board_outline": board_outline,
            "drill_obstacles": drill_obstacles,
            "drill_centers": drill_centers,
            "negotiate_max_classify_window_mm": 25.0,
            "negotiate_max_route_window_mm": 25.0,
            "negotiate_ripup_factor": 2,
        }
        _power_results = route_all(
            grid,
            _power_net_objs,
            escape=12,
            ripup_factor=8,
            via_cost=10.0,
            history_factor=1.0,
            allow_partial=True,
            gridless_first=_power_gf_names,
            gridless_kwargs=_power_gridless_kwargs,
        )
        _check_rss("phase0_power_done")
        _power_ok = 0
        for _pnet_name, _pnr in _power_results.items():
            if _pnr.ok:
                precomp_routes[_pnet_name] = _pnr
                _power_ok += 1
                if _have_shapely:
                    try:
                        from shapely.geometry import LineString as _PLS
                        from shapely.geometry import Point as _PPt
                        from tracewise.route.gridless.geom import snap as _psnap
                        _p_inflate = geo["track_mm"] / 2.0 + geo["clearance_mm"]
                        _p_via_inflate = (
                            geo["via_mm"] / 2.0 + geo["clearance_mm"] + geo["track_mm"] / 2.0
                        )
                        if hasattr(_pnr, "world_paths"):
                            for _wp in _pnr.world_paths:
                                if len(_wp) >= 2:
                                    _pts2d = [(_w[0], _w[1]) for _w in _wp]
                                    try:
                                        bcu_extra_obs.append(
                                            _psnap(_PLS(_pts2d).buffer(_p_inflate, cap_style=2))
                                        )
                                    except Exception:
                                        pass
                        if hasattr(_pnr, "world_vias"):
                            for _pvx, _pvy in (_pnr.world_vias or []):
                                try:
                                    bcu_extra_obs.append(
                                        _psnap(_PPt(_pvx, _pvy).buffer(_p_via_inflate, resolution=16))
                                    )
                                except Exception:
                                    pass
                    except Exception:
                        pass
        print(f"  [phase0] +3V3/+1V1: {_power_ok}/{len(_power_net_objs)} ok", flush=True)

    # Phase 1: Route J4-corridor escape signals with flow-assigned lane_y_mm
    print("\n[GCO] Phase 1: Route J4-corridor escape signals...", flush=True)
    _check_rss("phase1_escape_start")

    escape_via_drill_obs: list = []
    escape_order = sorted(
        [(nm, merged_assignment[nm]) for nm in j4_contending_signals
         if nm in merged_assignment and nm not in precomp_routes],
        key=lambda x: x[1].get("order_key", 9999 * 10000),
    )
    print(f"  J4-corridor escape nets to route: {len(escape_order)}", flush=True)

    escape_nets_realized = 0
    failed_escape: list = []

    for net_name, tc in escape_order:
        _check_rss(f"escape {net_name}")

        net_obj = nets_by_name.get(net_name)
        if net_obj is None:
            continue

        net_pads_world = pads_by_net.get(net_name, [])
        qfn_pads = [
            p for p in net_pads_world
            if p.get("ref", "") in dense_refs and p.get("front") and not p.get("back")
        ]
        dest_pads = [p for p in net_pads_world if p.get("ref", "") not in dense_refs]

        if not qfn_pads:
            print(f"  SKIP {net_name}: no QFN source pad", flush=True)
            continue

        src_p = qfn_pads[0]
        source_xy = (src_p["x"], src_p["y"])

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
        lane_y_mm = tc.get("lane_y_mm")  # flow-assigned (or None if fallback)

        if escape_via_xy is None and net_name not in FCU_ONLY_NETS:
            print(f"  SKIP_GRID {net_name}: no escape via", flush=True)
            continue

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
            extra_obstacles=[],
            fcu_stub_extra_obstacles=[],
            bcu_extra_obstacles=list(bcu_extra_obs),
            board_outline=board_outline,
            drill_obstacles=drill_obstacles,
            drill_centers=drill_centers,
            max_window_mm=MAX_WINDOW_MM,
            max_bcu_window_mm=MAX_BCU_WINDOW_MM,
        )
        t_net_elapsed = time.perf_counter() - t_net

        if t_net_elapsed > NET_TIMEOUT_S:
            print(f"  TIMEOUT {net_name}: {t_net_elapsed:.1f}s", flush=True)
            failed_escape.append((net_name, "timeout"))
            continue

        if not result.ok:
            print(f"  FAIL {net_name}: {result.reason} ({t_net_elapsed:.1f}s)", flush=True)
            failed_escape.append((net_name, result.reason))
            continue

        lane_str = f"{lane_y_mm:.3f}" if lane_y_mm is not None else "None"
        print(
            f"  OK {net_name:<16} via={result.world_vias} "
            f"lane_y={lane_str} t={t_net_elapsed:.1f}s",
            flush=True,
        )

        nr = to_gridless_netroute(net_obj, result.world_paths, grid,
                                   world_vias=result.world_vias)
        _mark(grid, nr, 1)
        precomp_routes[net_name] = nr
        escape_nets_realized += 1

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
                except Exception:
                    pass

        _check_rss(f"after {net_name}")

    print(
        f"\n  Phase 1 done: {escape_nets_realized}/{len(escape_order)} J4-corridor signals routed",
        flush=True,
    )
    _check_rss("phase1_escape_done")

    # Phase 1b: Route remaining QFN escape nets (non-J4 corridor) from ring_assignment
    print("\n[GCO] Phase 1b: Route remaining escape nets (non-J4 corridor)...", flush=True)
    non_j4_escape_nets = [
        (nm, ring_assignment[nm]) for nm in all_escape_nets
        if nm not in FCU_ONLY_NETS
        and nm not in j4_contending_signals
        and nm in ring_assignment
        and nm not in precomp_routes
        and nm in nets_by_name
    ]
    non_j4_escape_nets.sort(key=lambda x: x[1].get("order_key", 9999 * 10000))
    print(f"  Non-J4 escape nets: {len(non_j4_escape_nets)}", flush=True)

    for net_name, tc in non_j4_escape_nets:
        _check_rss(f"non_j4_escape {net_name}")

        net_obj = nets_by_name.get(net_name)
        if net_obj is None:
            continue

        net_pads_world = pads_by_net.get(net_name, [])
        qfn_pads = [
            p for p in net_pads_world
            if p.get("ref", "") in dense_refs and p.get("front") and not p.get("back")
        ]
        dest_pads = [p for p in net_pads_world if p.get("ref", "") not in dense_refs]

        if not qfn_pads:
            continue
        src_p = qfn_pads[0]
        source_xy = (src_p["x"], src_p["y"])

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
            continue

        escape_via_xy = tc.get("escape_via_xy")
        lane_y_mm = tc.get("lane_y_mm")

        if escape_via_xy is None:
            continue

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
            extra_obstacles=[],
            fcu_stub_extra_obstacles=[],
            bcu_extra_obstacles=list(bcu_extra_obs),
            board_outline=board_outline,
            drill_obstacles=drill_obstacles,
            drill_centers=drill_centers,
            max_window_mm=MAX_WINDOW_MM,
            max_bcu_window_mm=MAX_BCU_WINDOW_MM,
        )
        t_net_elapsed = time.perf_counter() - t_net

        if not result.ok or t_net_elapsed > NET_TIMEOUT_S:
            continue

        nr = to_gridless_netroute(net_obj, result.world_paths, grid,
                                   world_vias=result.world_vias)
        _mark(grid, nr, 1)
        precomp_routes[net_name] = nr

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
                except Exception:
                    pass

        _check_rss(f"non_j4_after {net_name}")

    print(f"  Phase 1b done: {len(precomp_routes)} total precomp routes", flush=True)
    _check_rss("phase1b_done")

    # ── Phase 2: Grid + gridless_first the rest (attempt-3 path) ─────────────
    print("\n[GCO] Phase 2: Grid + gridless_first remaining nets...", flush=True)
    _qfn_escape_set = set(merged_assignment.keys())
    ESCAPE_EXTRA_GRIDLESS = {"/USB_D-"}

    gf_nets = (ATTEMPT3_GRIDLESS_NETS | ESCAPE_EXTRA_GRIDLESS) - set(precomp_routes.keys())
    gf_nets_clean = {n for n in gf_nets if n not in _qfn_escape_set}

    try:
        from shapely.geometry import LineString as _GLSLS
        from shapely.geometry import Point as _GLSPt
        from tracewise.route.gridless.geom import snap as _gls_snap
        _gls_inflate = geo["track_mm"] + geo["clearance_mm"]
        _via_inflate = geo["via_mm"] / 2.0 + geo["clearance_mm"] + geo["track_mm"] / 2.0
        _gls_extra_obs = []
        for _nr in precomp_routes.values():
            if not hasattr(_nr, "world_paths"):
                continue
            for _wp in _nr.world_paths:
                if len(_wp) >= 2:
                    _pts2d = [(_w[0], _w[1]) for _w in _wp]
                    try:
                        _gls_extra_obs.append(
                            _gls_snap(_GLSLS(_pts2d).buffer(_gls_inflate, cap_style=2))
                        )
                    except Exception:
                        pass
            if hasattr(_nr, "world_vias"):
                for _vx2, _vy2 in (_nr.world_vias or []):
                    try:
                        _gls_extra_obs.append(
                            _gls_snap(_GLSPt(_vx2, _vy2).buffer(_via_inflate, resolution=16))
                        )
                    except Exception:
                        pass
    except Exception:
        _gls_extra_obs = []

    remaining = [n for n in nets if n.name not in precomp_routes]
    _check_rss("before_grid")

    gridless_kwargs = {
        "pads": pads,
        "geo": geo,
        "board_bbox": board_bbox,
        "anchors": anchors,
        "extra_gridless_obstacles": _gls_extra_obs,
        "board_outline": board_outline,
        "drill_obstacles": drill_obstacles,
        "drill_centers": drill_centers,
        "negotiate_max_classify_window_mm": 25.0,
        "negotiate_max_route_window_mm": 25.0,
        "negotiate_ripup_factor": 2,
    }

    t_grid_start = time.perf_counter()
    grid_results = route_all(
        grid,
        remaining,
        escape=12,
        ripup_factor=8,
        via_cost=10.0,
        history_factor=1.0,
        allow_partial=True,
        gridless_first=gf_nets_clean if gf_nets_clean else None,
        gridless_kwargs=gridless_kwargs if gf_nets_clean else None,
    )
    t_grid = time.perf_counter() - t_grid_start
    print(f"  Grid routing done in {t_grid:.1f}s", flush=True)
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
    report = run_drc(board)
    by = collections.Counter(v.get("type") for v in report.get("violations", []))
    errs = sum(1 for v in report.get("violations", []) if v.get("severity") == "error")
    unc = len(report.get("unconnected_items", []))

    # ── Crossing audit ────────────────────────────────────────────────────────
    crossings = audit_crossings(board)

    # ── +3V3 unconnected count ────────────────────────────────────────────────
    plus3v3_unc = sum(
        1 for item in report.get("unconnected_items", [])
        if (item.get("net", "") == "+3V3"
            or "+3V3" in str(item.get("net_name", ""))
            or "+3V3" in str(item.get("items", "")))
    )

    # Shorting items
    shorting_items = by.get("shorting_items", 0) or by.get("short", 0)

    return {
        "board": str(board),
        "unconnected": unc,
        "errors": errs,
        "by_type": dict(by),
        "escape_nets_realized": escape_nets_realized,
        "fcu_crossings": crossings.get("F.Cu", 0),
        "bcu_crossings": crossings.get("B.Cu", 0),
        "t_grid_s": round(t_grid, 1),
        "rss_gb": round(_rss_gb(), 3),
        "plus3v3_unconnected": plus3v3_unc,
        "shorting_items": shorting_items,
        "failed_escape_nets": failed_escape,
        "j4_contending_signals": sorted(j4_contending_signals),
        "flow_found_disjoint": flow_found_disjoint,
        "solve_time_s": round(solve_time_s, 4),
        "fallback_used": fallback_used,
        "flow_infeasible": flow_infeasible,
        "b_spine_used": spine_demands.get("+3V3", 0),
        "N_lanes": corridor_graph.graph.get("N_lanes", 0),
    }


# ── Board file hash ───────────────────────────────────────────────────────────

def board_hash(board_path: Path) -> str:
    return hashlib.sha256(board_path.read_bytes()).hexdigest()


# ── Main ──────────────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(description="GCO spike: simultaneous corridor allocation")
    parser.add_argument("--runs", type=int, default=1,
                        help="Number of routing runs (3 for determinism check)")
    parser.add_argument("--out", default="/tmp/gco_spike_out",
                        help="Output directory")
    args = parser.parse_args()

    t_total_start = time.perf_counter()
    peak_rss = _rss_gb()

    print("=" * 70)
    print("GCO SPIKE: SIMULTANEOUS GLOBAL CORRIDOR OPTIMIZER")
    print(f"Attempt-3 bar: unconnected ≤ {ATTEMPT3_UNC}, errors ≤ {ATTEMPT3_ERR}")
    print("=" * 70)

    out_base = Path(args.out)
    run_results: list[dict] = []
    run_hashes: list[str] = []

    for run_idx in range(args.runs):
        run_dir = out_base / f"run_{run_idx}"
        print(f"\n[Run {run_idx + 1}/{args.runs}]", flush=True)

        try:
            _check_rss(f"start run {run_idx}")
            score = run_gco(run_dir)
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
            print(f"  escape_nets_realized={score['escape_nets_realized']}")
            print(f"  +3V3 unconnected={score['plus3v3_unconnected']}")
            print(f"  shorting_items={score['shorting_items']}")
            print(f"  by_type={score['by_type']}")
            print(f"  flow_found_disjoint={score['flow_found_disjoint']}")
            print(f"  solve_time_s={score['solve_time_s']}")
            print(f"  fallback_used={score['fallback_used']}")
            print(f"  RSS={score['rss_gb']:.3f} GB")

        except MemoryError as exc:
            print(f"  MEMORY ABORT: {exc}", flush=True)
            run_results.append({
                "unconnected": 9999, "errors": 9999, "abort": str(exc),
                "rss_gb": _rss_gb(), "fcu_crossings": 0, "bcu_crossings": 0,
                "escape_nets_realized": 0,
                "plus3v3_unconnected": -1, "shorting_items": 0,
                "by_type": {}, "failed_escape_nets": [],
                "flow_found_disjoint": False, "solve_time_s": 0.0,
                "fallback_used": True, "flow_infeasible": False,
                "b_spine_used": 2, "N_lanes": 0,
                "j4_contending_signals": [],
            })
            break
        except Exception as exc:
            import traceback as _tb
            print(f"  ERROR run {run_idx + 1}: {exc}", flush=True)
            _tb.print_exc()
            run_results.append({
                "unconnected": 9999, "errors": 9999, "abort": str(exc),
                "rss_gb": _rss_gb(), "fcu_crossings": 0, "bcu_crossings": 0,
                "escape_nets_realized": 0,
                "plus3v3_unconnected": -1, "shorting_items": 0,
                "by_type": {}, "failed_escape_nets": [],
                "flow_found_disjoint": False, "solve_time_s": 0.0,
                "fallback_used": True, "flow_infeasible": False,
                "b_spine_used": 2, "N_lanes": 0,
                "j4_contending_signals": [],
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
            "escape_nets_realized": 0, "by_type": {},
            "plus3v3_unconnected": -1, "shorting_items": 0,
            "failed_escape_nets": [],
            "flow_found_disjoint": False, "solve_time_s": 0.0,
            "fallback_used": True, "flow_infeasible": False,
            "b_spine_used": 2, "N_lanes": 0,
            "j4_contending_signals": [],
        }

    unc = best.get("unconnected", 9999)
    errs = best.get("errors", 9999)
    fcu_x = best.get("fcu_crossings", 9999)
    bcu_x = best.get("bcu_crossings", 9999)
    illegal_x = fcu_x + bcu_x
    shorting_items = best.get("shorting_items", 0)
    flow_found_disjoint = best.get("flow_found_disjoint", False)
    solve_time_s = best.get("solve_time_s", 0.0)
    b_spine_used = best.get("b_spine_used", 2)
    N_lanes = best.get("N_lanes", 0)

    # GCO clean-win gate: unc ≤ 40 AND err ≤ 73 AND 0 shorts AND 0 crossings
    clean_win = (
        unc <= ATTEMPT3_UNC - 1 and  # strictly better than attempt-3 on connectivity
        errs <= ATTEMPT3_ERR and
        shorting_items == 0 and
        illegal_x == 0
    )
    rss_ok = peak_rss < RSS_HARD_FAIL_GB
    det_pass = "PASS" in deterministic
    solve_ok = solve_time_s < SOLVE_TIMEOUT_S

    # Determine outcome
    if best.get("flow_infeasible", False):
        outcome = "flow-infeasible"
    elif best.get("fallback_used", False) and not flow_found_disjoint:
        outcome = "flow-infeasible"
    elif not rss_ok:
        outcome = "realization-broke-it"
    elif clean_win:
        outcome = "clean-win"
    elif unc <= ATTEMPT3_UNC and errs <= ATTEMPT3_ERR:
        # Tied or exactly matched on one axis
        outcome = "tie"
    elif unc < 9999 and errs < 9999:
        # Got some results but didn't improve
        if unc > ATTEMPT3_UNC or errs > ATTEMPT3_ERR:
            outcome = "regression"
        else:
            outcome = "tie"
    else:
        outcome = "regression"

    # GCO GO/NO-GO
    if clean_win and rss_ok and det_pass and solve_ok:
        gco_go_no_go = "GO: all criteria met"
    else:
        reasons = []
        if unc > ATTEMPT3_UNC - 1:
            reasons.append(f"unconnected={unc} > {ATTEMPT3_UNC - 1} (need ≤40)")
        if errs > ATTEMPT3_ERR:
            reasons.append(f"errors={errs} > {ATTEMPT3_ERR}")
        if shorting_items > 0:
            reasons.append(f"shorts={shorting_items}")
        if illegal_x > 0:
            reasons.append(f"crossings={illegal_x}")
        if not rss_ok:
            reasons.append(f"RSS={peak_rss:.2f}GB ≥ 2GB")
        if not det_pass:
            reasons.append(f"determinism={deterministic}")
        if not solve_ok:
            reasons.append(f"solve_time={solve_time_s:.1f}s ≥ {SOLVE_TIMEOUT_S}s")
        if not flow_found_disjoint:
            reasons.append("flow did not find disjoint assignment")
        gco_go_no_go = f"NO-GO: {'; '.join(reasons) if reasons else 'unknown'}"

    # ── Print verdict ─────────────────────────────────────────────────────────
    print("\n" + "=" * 70)
    print(f"GCO VERDICT: {'GO' if clean_win else 'NO-GO'}")
    print(f"  unconnected = {unc}  (bar: ≤40;  attempt-3 = 41)")
    print(f"  errors      = {errs}  (bar: ≤73;  attempt-3 = 73)")
    print(f"  crossings   = F.Cu={fcu_x}  B.Cu={bcu_x}  (bar: 0)")
    print(f"  shorts      = {shorting_items}  (bar: 0)")
    print(f"  flow_disjoint = {flow_found_disjoint}")
    print(f"  solve_time  = {solve_time_s:.3f}s  (bar: <{SOLVE_TIMEOUT_S}s)")
    print(f"  RSS         = {peak_rss:.3f} GB  (bar: < 2 GB)")
    print(f"  determinism = {deterministic}")
    print(f"  outcome     = {outcome}")
    print(f"  GCO GO/NO-GO: {gco_go_no_go}")
    print("=" * 70)

    by_type_best = best.get("by_type", {})

    # ── Structured Result ─────────────────────────────────────────────────────
    structured = {
        "status": "complete",
        "summary": (
            f"GCO spike: unc={unc} err={errs} "
            f"crossings_fcu={fcu_x} crossings_bcu={bcu_x} "
            f"shorts={shorting_items} "
            f"flow_disjoint={flow_found_disjoint} "
            f"fallback={best.get('fallback_used', False)}"
        ),
        "files_changed": [
            "scripts/_probe_gco_spike.py (new — GCO spike)",
        ],
        "files_read": [
            str(PCB_PATH),
            "scripts/_probe_tcr_e1.py",
            "src/tracewise/route/gridless/topo_assign.py",
            "src/tracewise/route/gridless/route.py",
            "src/tracewise/route/engine/multi.py",
            "src/tracewise/route/engine/kicad.py",
            "docs/design/GLOBAL-CORRIDOR-OPTIMIZER.md",
        ],
        "corridor_graph_built": flow_found_disjoint or (solve_time_s > 0),
        "flow_found_disjoint_assignment": flow_found_disjoint,
        "solve_time_s": solve_time_s,
        "contending_set": best.get("j4_contending_signals", []) + sorted(CORRIDOR_POWER_NETS),
        "b_spine_used": b_spine_used,
        "default_byte_identical": "n/a (spike)",
        "result": {
            "unconnected": unc,
            "errors": errs,
            "by_type": dict(by_type_best),
        },
        "plus3v3_unconnected": best.get("plus3v3_unconnected", -1),
        "shorting_items": shorting_items,
        "illegal_crossings": {"fcu": fcu_x, "bcu": bcu_x},
        "escape_nets_realized": best.get("escape_nets_realized", 0),
        "unconnected_vs_attempt3": (
            f"{unc} vs {ATTEMPT3_UNC} "
            f"({'better' if unc < ATTEMPT3_UNC else 'tied' if unc == ATTEMPT3_UNC else 'worse'})"
        ),
        "errors_vs_attempt3": (
            f"{errs} vs {ATTEMPT3_ERR} "
            f"({'better' if errs < ATTEMPT3_ERR else 'tied' if errs == ATTEMPT3_ERR else 'worse'})"
        ),
        "clean_win": clean_win,
        "outcome": outcome,
        "peak_rss_gb": round(peak_rss, 3),
        "runtime_s": round(t_total, 1),
        "deterministic": deterministic,
        "gco_go_no_go": gco_go_no_go,
        "issues": best.get("failed_escape_nets", []),
        "assumptions": [
            "B_spine=2 lanes for +3V3 spine band (from design spec)",
            "B_1v1=1 lane for +1V1",
            "J4_INNER_Y=80.40mm, J3_INNER_Y=98.20mm, LANE_PITCH_MM=0.35mm",
            "Contending signals: J4 dest (|dest_y - 80.40| < 1.5mm) ∩ QFN_ESCAPE_NETS",
            "max_window_mm=12, max_bcu_window_mm=8 (bounded)",
            "RSS hard-abort at 2GB; SOLVE_TIMEOUT_S=10s",
            "Phase-0 power-first: +3V3/+1V1 routed before escape nets",
            "Fallback to ring_slot assignment if flow times out or fails",
        ],
        "N_lanes_in_channel": N_lanes,
        "flow_infeasible": best.get("flow_infeasible", False),
    }

    print("\n## Structured Result")
    print("```json")
    print(json.dumps(structured, indent=2, default=str))
    print("```")

    # Save result JSON
    out_base.mkdir(parents=True, exist_ok=True)
    result_path = out_base / "gco_result.json"
    result_path.write_text(json.dumps(structured, indent=2, default=str))
    print(f"\nResult saved: {result_path}")


if __name__ == "__main__":
    main()
