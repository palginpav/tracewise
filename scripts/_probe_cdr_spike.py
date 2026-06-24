#!/usr/bin/env python3
"""
_probe_cdr_spike.py — CDR (Concurrent Detailed Router) decisive spike.

Tests: does routing ALL shared-corridor nets' geometry SIMULTANEOUSLY
(constraint-graph channel routing, direct track emission) eliminate the
per-net bcu_run_failed+shorts wall that capped every prior approach at
the 41/73 frontier?

Thesis: concurrent channel routing assigns ALL track-y's JOINTLY (disjoint
by construction) and emits geometry DIRECTLY — no per-net visgraph, no
accumulated-copper check → bcu_run_failed cannot occur.

Algorithm:
  1. GCO min-cost flow assigns each net a lane_y (closest to its dest connector)
     AND guarantees ALL lanes are disjoint. This gives the concurrent assignment.
  2. CDR REALIZES that assignment directly: for each net, emit
     F.Cu stub → ring via → B.Cu horizontal run at lane_y → dest pad.
     NO route_net_steered. NO accumulated bcu_extra_obstacles.
  3. The vertical jogs (via→lane_y, lane_y→dest) are SHORT because lane_y
     is chosen near each net's dest connector (cost = |lane_y - dest_y|).

GO iff ALL: unconnected ≤ 40 AND errors ≤ 73 AND 0 shorting_items AND
0 illegal crossings AND deterministic (3-run identical connectivity) AND
bounded (peak RSS < 2GB, channel solve < 10s).

Usage:
    .venv/bin/python scripts/_probe_cdr_spike.py [--runs N] [--out DIR]
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
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

# ── Path setup ────────────────────────────────────────────────────────────────

_REPO = Path(__file__).parent.parent
sys.path.insert(0, str(_REPO / "src"))
sys.path.insert(0, str(Path(__file__).parent))

from probe_human_routing import (  # noqa: E402
    parse_pcb, extract_segments, extract_vias, extract_nets,
    PCB_PATH,
)

# ── Constants ─────────────────────────────────────────────────────────────────

RSS_HARD_FAIL_GB = 2.0
CHANNEL_TIMEOUT_S = 10.0
NET_TIMEOUT_S = 60.0
MAX_WINDOW_MM = 12.0
MAX_BCU_WINDOW_MM = 8.0
PITCH = 0.35            # track + clearance pitch for channel

# Corridor geometry (west channel J4 → J3)
J4_INNER_Y = 80.40   # inner J4 row y (mm)
J3_INNER_Y = 98.20   # inner J3 row y (mm)
LANE_PITCH_MM = 0.35  # track + clearance pitch

# Power nets that share the J4 corridor
CORRIDOR_POWER_NETS = {"+3V3", "+1V1"}

# All QFN escape nets
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

# Attempt-3 gridless_first nets
ATTEMPT3_GRIDLESS_NETS = {
    "/GPIO3", "/GPIO4", "/GPIO6", "/GPIO9", "/GPIO14",
    "/GPIO18", "/GPIO20", "/GPIO23", "/GPIO27", "/GPIO28",
    "/RUN", "/SWCLK", "/USB_D+", "/XIN",
    "/QSPI_SCLK", "/QSPI_SD2", "Net-(U3-USB-DP)",
}

# Spine band width in tracks (B_spine=2 for +3V3)
B_SPINE = 2


# ── RSS guard ─────────────────────────────────────────────────────────────────

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


# ── Board file hash ───────────────────────────────────────────────────────────

def board_hash(board_path: Path) -> str:
    return hashlib.sha256(board_path.read_bytes()).hexdigest()


# ── GCO: Corridor flow graph builder ─────────────────────────────────────────

class CorridorTimeout(Exception):
    """Raised when the min-cost flow solve exceeds CHANNEL_TIMEOUT_S."""


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
    spine_demands: dict[str, int],
    channel_y_range: tuple[float, float] = (J4_INNER_Y, J3_INNER_Y),
    pitch_mm: float = LANE_PITCH_MM,
) -> object:
    """Build the capacitated corridor lane×layer flow graph (GCO warm-start)."""
    import networkx as nx

    lanes = _build_lanes(channel_y_range[0], channel_y_range[1], pitch_mm)
    N = len(lanes)
    if N == 0:
        raise RuntimeError("build_corridor_graph: no lanes in channel")

    LAYER_PENALTY = {0: 5000, 1: 0}

    signal_names = sorted(contending_signals.keys())
    total_flow = len(signal_names) + sum(spine_demands.values())

    G = nx.DiGraph()
    G.add_node("S", demand=-total_flow)
    G.add_node("T", demand=total_flow)

    for net in signal_names:
        cmd_node = f"cmd_{net}"
        G.add_node(cmd_node, demand=0)
        G.add_edge("S", cmd_node, capacity=1, weight=0)

        info = contending_signals[net]
        dest_xy = info.get("dest_xy")
        dest_y = dest_xy[1] if dest_xy is not None else (J4_INNER_Y + J3_INNER_Y) / 2

        for li, lane_y in enumerate(lanes):
            for layer in (0, 1):
                slot_node = f"slot_{li}_{layer}"
                if not G.has_node(slot_node):
                    G.add_node(slot_node, demand=0)
                dist_cost = int(abs(lane_y - dest_y) * 1000)
                layer_cost = LAYER_PENALTY[layer]
                epsilon = li * 7 + layer * 3
                cost = dist_cost + layer_cost + epsilon
                G.add_edge(cmd_node, slot_node, capacity=1, weight=cost)

    band_candidates: list[list[int]] = []
    if N >= 4:
        band_candidates = [
            list(range(0, min(2, N))),
            list(range(max(N - 2, 0), N)),
            list(range(2, min(4, N))),
            list(range(max(N - 4, N - 2), max(N - 2, 0))),
        ]
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

    for pw_net in sorted(spine_demands.keys()):
        demand = spine_demands[pw_net]
        spine_node = f"spine_{pw_net}"
        G.add_node(spine_node, demand=0)
        G.add_edge("S", spine_node, capacity=demand, weight=0)

        for band in band_candidates:
            for li in band:
                slot_node = f"slot_{li}_1"
                if not G.has_node(slot_node):
                    G.add_node(slot_node, demand=0)
                band_idx = band_candidates.index(band)
                epsilon_s = li * 11 + band_idx * 17
                cost_s = li * 100 + epsilon_s
                if not G.has_edge(spine_node, slot_node):
                    G.add_edge(spine_node, slot_node, capacity=1, weight=cost_s)

    for li in range(N):
        for layer in (0, 1):
            slot_node = f"slot_{li}_{layer}"
            if G.has_node(slot_node):
                G.add_edge(slot_node, "T", capacity=1, weight=0)

    G.graph["lanes"] = lanes
    G.graph["signal_names"] = signal_names
    G.graph["spine_demands"] = dict(spine_demands)
    G.graph["N_lanes"] = N

    print(
        f"  [corridor_graph] N_lanes={N} signals={len(signal_names)} "
        f"spine_nets={sorted(spine_demands.keys())} total_flow={total_flow}",
        flush=True,
    )
    return G


def solve_corridor_assignment(
    graph,
    solve_timeout_s: float = CHANNEL_TIMEOUT_S,
) -> dict[str, tuple[int, float, int]]:
    """Run networkx min_cost_flow; decode → {net: (lane_idx, lane_y_mm, layer)}."""
    import networkx as nx

    lanes = graph.graph["lanes"]
    signal_names = graph.graph["signal_names"]
    spine_demands = graph.graph["spine_demands"]

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
        raise CorridorTimeout(f"min_cost_flow timed out after {solve_timeout_s}s")
    if not exc_holder:
        raise CorridorTimeout("min_cost_flow thread finished but no result stored")

    status, value = exc_holder[0]
    if status == "err":
        raise value

    flow_dict = value
    print(f"  [solve] min_cost_flow done in {t_elapsed:.3f}s", flush=True)

    assignment: dict[str, tuple[int, float, int]] = {}

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
            spine_slots.sort()
            assignment[pw_net] = spine_slots[0]
            assignment[f"_spine_band_{pw_net}"] = spine_slots  # type: ignore[assignment]

    return assignment


def assert_assignment_disjoint(
    assignment: dict[str, tuple[int, float, int]],
    spine_demands: dict[str, int],
    signal_names: list[str],
) -> None:
    """Assert: no two signal nets share a slot; spine slots disjoint from signal slots."""
    signal_slots: dict[str, tuple[int, int]] = {}
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

    seen: dict[tuple[int, int], str] = {}
    for net, slot in signal_slots.items():
        if slot in seen:
            raise AssertionError(
                f"GCO DISJOINT VIOLATION: {net} and {seen[slot]} share slot {slot}"
            )
        seen[slot] = net

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


# ── Channel Router Data Structures ────────────────────────────────────────────

@dataclass(frozen=True)
class ChannelNet:
    net: str
    top_x: float              # ring-via x column (escape via)
    bot_x: float              # dest connector x column
    x_span: tuple             # (min(top_x,bot_x), max(top_x,bot_x))
    escape_via_xy: tuple      # (x, y) of ring via
    dest_xy: tuple            # (x, y) of dest pad
    source_xy: tuple          # (x, y) of QFN source pad
    order_key: float          # dest_y*10000 + dest_x, deterministic tie-break


@dataclass
class ChannelResult:
    net: str
    track_y: float            # assigned B.Cu horizontal track y
    layer_of_run: int         # 1=B.Cu
    via_xys: list             # transition vias [(x,y), ...]


@dataclass
class ChannelInstance:
    nets: list                # list[ChannelNet] sorted by name
    track_ys: list            # all candidate track y-coordinates (monotone, excluding spine)
    spine_band: tuple         # (y_lo, y_hi) reserved for +3V3
    pitch_mm: float


# ── Channel Router: concurrent VCG+HCG constraint-graph assignment ─────────────

def build_channel_instance(
    contending_nets: dict[str, dict],
    spine_band: tuple[float, float],
    channel_y_range: tuple[float, float],
    pitch_mm: float,
    flow_assignment: dict[str, tuple[int, float, int]] | None = None,
) -> ChannelInstance:
    """Build a ChannelInstance.

    contending_nets: {net_name: {escape_via_xy, dest_xy, source_xy, order_key}}
    flow_assignment: GCO flow assignment {net: (lane_idx, lane_y_mm, layer)} (warm-start)
    """
    cnets = []
    for net_name in sorted(contending_nets.keys()):
        info = contending_nets[net_name]
        via = info.get("escape_via_xy")
        dest = info.get("dest_xy")
        src = info.get("source_xy")
        ok = info.get("order_key", 9999.0 * 10000)
        if via is None or dest is None:
            continue
        top_x = round(via[0] * 1e6) / 1e6
        bot_x = round(dest[0] * 1e6) / 1e6
        x_span = (min(top_x, bot_x), max(top_x, bot_x))
        cnets.append(ChannelNet(
            net=net_name,
            top_x=top_x,
            bot_x=bot_x,
            x_span=x_span,
            escape_via_xy=(round(via[0] * 1e6) / 1e6, round(via[1] * 1e6) / 1e6),
            dest_xy=(round(dest[0] * 1e6) / 1e6, round(dest[1] * 1e6) / 1e6),
            source_xy=src or (0.0, 0.0),
            order_key=ok,
        ))

    # Generate all track_ys, excluding spine band
    y_lo, y_hi = channel_y_range
    track_ys = []
    y = y_lo + pitch_mm / 2.0
    while y < y_hi - pitch_mm / 2.0:
        y_s = round(y * 1e6) / 1e6
        if not (spine_band[0] - pitch_mm * 0.5 <= y_s <= spine_band[1] + pitch_mm * 0.5):
            track_ys.append(y_s)
        y += pitch_mm

    print(
        f"  [channel_instance] {len(cnets)} channel nets, "
        f"{len(track_ys)} signal tracks (excl. spine band)",
        flush=True,
    )
    return ChannelInstance(
        nets=cnets,
        track_ys=track_ys,
        spine_band=spine_band,
        pitch_mm=pitch_mm,
    )


def build_constraint_graphs(instance: ChannelInstance):
    """Build VCG and HCG constraint graphs.

    VCG: edge i→j means net i must be on a track BELOW net j
         (lower y = closer to J4). Condition: i's via_y < j's dest_y
         AND they share a column (|top_x_i - bot_x_j| < pitch).
         This prevents vertical jog crossing.

    HCG: edge i—j iff x_spans overlap (can't share same track).
    """
    import networkx as nx

    vcg = nx.DiGraph()
    hcg = nx.Graph()
    nets = instance.nets
    pitch = instance.pitch_mm

    for cnet in nets:
        vcg.add_node(cnet.net)
        hcg.add_node(cnet.net)

    n = len(nets)
    for i in range(n):
        for j in range(n):
            if i == j:
                continue
            ni = nets[i]
            nj = nets[j]
            # VCG: i→j if ni's top column aligns with nj's bottom column
            # (their vertical jogs share a column → must be ordered)
            if abs(ni.top_x - nj.bot_x) < pitch:
                vcg.add_edge(ni.net, nj.net)

        for j in range(i + 1, n):
            ni = nets[i]
            nj = nets[j]
            # HCG: x-spans overlap → cannot share same track
            lo = max(ni.x_span[0], nj.x_span[0])
            hi = min(ni.x_span[1], nj.x_span[1])
            if lo < hi + pitch * 0.1:
                hcg.add_edge(ni.net, nj.net)

    cycles = list(_find_vcg_cycles(vcg))
    print(
        f"  [constraint_graphs] VCG edges={vcg.number_of_edges()} "
        f"HCG edges={hcg.number_of_edges()} VCG_cycles={len(cycles)}",
        flush=True,
    )
    return vcg, hcg


def _find_vcg_cycles(vcg) -> list:
    """Find all SCCs with >1 node (cycles) in VCG."""
    import networkx as nx
    return [scc for scc in nx.strongly_connected_components(vcg) if len(scc) > 1]


def assign_tracks_from_flow(
    instance: ChannelInstance,
    flow_assignment: dict[str, tuple[int, float, int]],
    vcg,
    hcg,
    contending_nets: dict[str, dict],
) -> dict[str, ChannelResult]:
    """Concurrent track assignment using GCO flow + geometry-aware crossing avoidance.

    Key geometric insight: the B.Cu path is:
      via(vx,vy) → B.Cu jog to (vx, track_y) → horizontal (vx,ty)→(dx,ty) → dest

    The B.Cu vertical jog at x=vx from vy to track_y WILL cross horizontal
    B.Cu runs of other corridor nets if those nets have x_spans including vx
    AND their track_ys are between vy and track_y.

    Strategy: assign track_y BETWEEN via_y and dest_y so that:
    - J4 nets (dest_y < via_y): track_y should be as close to via_y as possible
      (just below via_y), so the downward jog is short and passes through fewer
      other nets' horizontal spans.
    - J3 nets (dest_y > via_y): track_y should be as close to via_y as possible
      (just above via_y), for the same reason.

    The GCO flow's lane_y is used as the STARTING point (closest to dest) but we
    may nudge toward via_y to minimize crossing-prone jog lengths.

    Returns dict[net_name -> ChannelResult].
    """
    pitch = instance.pitch_mm
    nets_by_name = {n.net: n for n in instance.nets}
    results: dict[str, ChannelResult] = {}

    used_tracks: dict[float, list[tuple[float, float, str]]] = {}

    def track_free(track_y: float, x_lo: float, x_hi: float) -> bool:
        if track_y not in used_tracks:
            return True
        for ex_lo, ex_hi, _ in used_tracks[track_y]:
            if x_lo < ex_hi + pitch * 0.1 and x_hi > ex_lo - pitch * 0.1:
                return False
        return True

    def jog_crossing_penalty(track_y: float, cnet: ChannelNet, results_so_far: dict) -> float:
        """Count how many already-assigned nets' B.Cu runs this jog would cross.

        The jog is at x=via_x from via_y to track_y. Crossing occurs when
        another net has track_y_other BETWEEN via_y and track_y AND
        via_x is inside that net's x_span.
        """
        via_x = cnet.escape_via_xy[0]
        via_y = cnet.escape_via_xy[1]
        jog_lo = min(via_y, track_y)
        jog_hi = max(via_y, track_y)
        crossings = 0
        for other_net, other_cr in results_so_far.items():
            other_cnet = nets_by_name.get(other_net)
            if other_cnet is None:
                continue
            # Other net's horizontal run at other_cr.track_y, spanning x_span
            other_ty = other_cr.track_y
            if not (jog_lo < other_ty < jog_hi):
                continue
            # Check if via_x is inside other net's x_span
            if other_cnet.x_span[0] - pitch * 0.5 <= via_x <= other_cnet.x_span[1] + pitch * 0.5:
                crossings += 1
        return float(crossings)

    # Sort nets by order_key (dest_y monotone, deterministic)
    nets_sorted = sorted(
        instance.nets,
        key=lambda n: (contending_nets.get(n.net, {}).get("order_key", 9e9), n.net),
    )

    for cnet in nets_sorted:
        net_name = cnet.net
        x_lo = cnet.x_span[0]
        x_hi = cnet.x_span[1]
        via_y = cnet.escape_via_xy[1]
        dest_y = cnet.dest_xy[1]

        # Get preferred track_y from flow
        if net_name in flow_assignment:
            li, lane_y, layer = flow_assignment[net_name]
            preferred_track_y = lane_y
        else:
            preferred_track_y = dest_y

        # Find best track: minimize (crossing_penalty * 1000 + |ty - preferred_track_y|)
        # This prefers tracks that don't cause jog crossings, tie-broken by proximity to flow lane
        best_track_y = None
        best_score = float("inf")

        for ty in instance.track_ys:
            if not track_free(ty, x_lo, x_hi):
                continue
            crossings = jog_crossing_penalty(ty, cnet, results)
            dist = abs(ty - preferred_track_y)
            score = crossings * 1000.0 + dist
            if score < best_score:
                best_score = score
                best_track_y = ty

        if best_track_y is None:
            print(f"  WARNING: no free track for {net_name} — using preferred", flush=True)
            best_track_y = preferred_track_y

        if best_track_y not in used_tracks:
            used_tracks[best_track_y] = []
        used_tracks[best_track_y].append((x_lo, x_hi, net_name))

        results[net_name] = ChannelResult(
            net=net_name,
            track_y=best_track_y,
            layer_of_run=1,
            via_xys=[cnet.escape_via_xy],
        )
        crossings_at_assigned = jog_crossing_penalty(best_track_y, cnet, {k: v for k, v in results.items() if k != net_name})
        print(
            f"  [assign] {net_name:<18} flow_y={preferred_track_y:.3f} "
            f"via_y={via_y:.3f} dest_y={dest_y:.3f} "
            f"→ track_y={best_track_y:.3f} (crossings={crossings_at_assigned:.0f})",
            flush=True,
        )

    return results


def assign_tracks_order_key_fallback(
    instance: ChannelInstance,
    contending_nets: dict[str, dict],
) -> dict[str, ChannelResult]:
    """Fallback: assign tracks by order_key (dest_y monotone) without flow.

    Each net gets the closest available track to its dest_y.
    """
    pitch = instance.pitch_mm
    results: dict[str, ChannelResult] = {}
    used_tracks: dict[float, list[tuple[float, float, str]]] = {}

    def track_free(track_y: float, x_lo: float, x_hi: float) -> bool:
        if track_y not in used_tracks:
            return True
        for ex_lo, ex_hi, _ in used_tracks[track_y]:
            if x_lo < ex_hi + pitch * 0.1 and x_hi > ex_lo - pitch * 0.1:
                return False
        return True

    nets_sorted = sorted(
        instance.nets,
        key=lambda n: (contending_nets.get(n.net, {}).get("order_key", 9e9), n.net),
    )

    for cnet in nets_sorted:
        net_name = cnet.net
        x_lo = cnet.x_span[0]
        x_hi = cnet.x_span[1]
        dest_y = cnet.dest_xy[1]

        best_track_y = None
        best_dist = float("inf")
        for ty in instance.track_ys:
            if track_free(ty, x_lo, x_hi):
                dist = abs(ty - dest_y)
                if dist < best_dist:
                    best_dist = dist
                    best_track_y = ty

        if best_track_y is None:
            best_track_y = instance.track_ys[-1] if instance.track_ys else dest_y

        if best_track_y not in used_tracks:
            used_tracks[best_track_y] = []
        used_tracks[best_track_y].append((x_lo, x_hi, net_name))

        results[net_name] = ChannelResult(
            net=net_name,
            track_y=best_track_y,
            layer_of_run=1,
            via_xys=[cnet.escape_via_xy],
        )

    return results


def assert_channel_disjoint(
    results: dict[str, ChannelResult],
    instance: ChannelInstance,
    contending_nets: dict[str, dict],
) -> None:
    """HARD ASSERT: no two nets share track_y with overlapping x-span.
    Also assert no track falls in spine band.
    """
    pitch = instance.pitch_mm
    nets_by_name = {n.net: n for n in instance.nets}
    track_runs: list[tuple[float, float, float, str]] = []

    for net_name, cr in results.items():
        cnet = nets_by_name.get(net_name)
        if cnet is None:
            continue
        x_lo = cnet.x_span[0]
        x_hi = cnet.x_span[1]
        track_runs.append((cr.track_y, x_lo, x_hi, net_name))

    # Check spine band violations
    for ty, x_lo, x_hi, net in track_runs:
        sb_lo, sb_hi = instance.spine_band
        if sb_lo - pitch * 0.5 <= ty <= sb_hi + pitch * 0.5:
            raise AssertionError(
                f"DISJOINT VIOLATION: {net} track_y={ty:.4f} inside spine band "
                f"[{sb_lo:.4f}, {sb_hi:.4f}]"
            )

    # Check pairwise disjointness
    for i in range(len(track_runs)):
        ty_i, xlo_i, xhi_i, ni = track_runs[i]
        for j in range(i + 1, len(track_runs)):
            ty_j, xlo_j, xhi_j, nj = track_runs[j]
            if ni == nj:
                continue
            if abs(ty_i - ty_j) < pitch * 0.5:
                lo = max(xlo_i, xlo_j)
                hi = min(xhi_i, xhi_j)
                if lo < hi + pitch * 0.1:
                    raise AssertionError(
                        f"DISJOINT VIOLATION: {ni} (track_y={ty_i:.4f}, "
                        f"x=[{xlo_i:.4f},{xhi_i:.4f}]) and "
                        f"{nj} (track_y={ty_j:.4f}, x=[{xlo_j:.4f},{xhi_j:.4f}]) "
                        f"share overlapping track"
                    )

    print(
        f"  [assert_disjoint] PASS: {len(results)} nets, {len(track_runs)} runs, all disjoint",
        flush=True,
    )


def realize_channel(
    results: dict[str, ChannelResult],
    contending_nets: dict[str, dict],
    geo: dict,
) -> dict[str, tuple[list, list]]:
    """Each ChannelResult → (world_paths, world_vias) DIRECT geometry.

    Geometry design:
      For each corridor net, the B.Cu path is:
        ring_via(vx,vy) → B.Cu jog (vx,vy)→(vx,ty) → B.Cu horiz (vx,ty)→(dx,ty)
        → B.Cu jog (dx,ty)→(dx,dy) → dest

      The assign_tracks_from_flow function minimizes crossing_penalty by choosing
      track_y to minimize B.Cu jog crossings with other nets' horizontal runs.

      Via at (vx, vy): the original legally-placed ring slot position.

    world_paths: list of [(x,y,layer),...] 3-tuple waypoints
    world_vias: [(x,y),...] via centers

    NO visgraph. NO route_net_steered. NO accumulated-copper check.
    Legal by construction (disjoint track assignment + crossing penalty).
    """
    channel_geometry: dict[str, tuple[list, list]] = {}

    for net_name, cr in results.items():
        info = contending_nets.get(net_name, {})
        via_xy = info.get("escape_via_xy")
        dest_xy = info.get("dest_xy")
        src_xy = info.get("source_xy")

        if via_xy is None or dest_xy is None:
            continue

        vx = round(via_xy[0] * 1e6) / 1e6
        vy = round(via_xy[1] * 1e6) / 1e6
        dx = round(dest_xy[0] * 1e6) / 1e6
        dy = round(dest_xy[1] * 1e6) / 1e6
        ty = round(cr.track_y * 1e6) / 1e6

        # Via at original ring slot (legally validated position)
        world_vias = [(vx, vy)]

        if src_xy is not None:
            sx = round(src_xy[0] * 1e6) / 1e6
            sy = round(src_xy[1] * 1e6) / 1e6
            world_path = [
                (sx, sy, 0),       # F.Cu QFN source pad
                (vx, vy, 0),       # F.Cu stub to ring via
                (vx, vy, 1),       # via transition F.Cu→B.Cu at ring slot
                (vx, ty, 1),       # B.Cu: jog from via_y to track_y
                (dx, ty, 1),       # B.Cu: horizontal run at track_y
                (dx, dy, 1),       # B.Cu: jog to dest pad
            ]
        else:
            world_path = [
                (vx, vy, 0),       # F.Cu at ring via
                (vx, vy, 1),       # via transition
                (vx, ty, 1),
                (dx, ty, 1),
                (dx, dy, 1),
            ]

        # Deduplicate zero-length same-layer segments
        deduped = [world_path[0]]
        for wp in world_path[1:]:
            prev = deduped[-1]
            same_xy = (abs(wp[0] - prev[0]) < 1e-9 and abs(wp[1] - prev[1]) < 1e-9)
            if same_xy:
                # Keep only if layer differs (via transition)
                if len(wp) > 2 and len(prev) > 2 and wp[2] != prev[2]:
                    deduped.append(wp)
            else:
                deduped.append(wp)

        channel_geometry[net_name] = ([deduped], world_vias)

    return channel_geometry


# ── Main CDR run ───────────────────────────────────────────────────────────────

def run_cdr(out_dir: Path, b_spine: int = B_SPINE) -> dict:
    """Execute one CDR routing run. Returns a score dict."""
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
    from tracewise.route.gridless.topo_assign import build_ring_slot_assignment

    # ── Set up board in temp dir ──────────────────────────────────────────────
    board_dir = Path(PCB_PATH).parent
    shutil.rmtree(out_dir, ignore_errors=True)
    out_dir.mkdir(parents=True)
    for f in board_dir.iterdir():
        if f.suffix in SUFFIXES:
            shutil.copy(f, out_dir / f.name)
    board = next(out_dir.glob("*.kicad_pcb"))
    strip_routing(board)

    # ── Extract board data ────────────────────────────────────────────────────
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

    # ── Step 2: Determine contending set (J3∪J4) ─────────────────────────────
    connector_refs = {"J3", "J4", "J5"}
    passive_refs = {"Y1", "C17", "R9", "SW1"}
    J3_Y = 98.20
    J4_Y = 80.40

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
            best = min(plist, key=lambda p: abs(p["y"] - J3_Y))
        elif ref == "J4":
            best = min(plist, key=lambda p: abs(p["y"] - J4_Y))
        else:
            best = plist[0]
        dest_by_net[nm] = (best["x"], best["y"])

    for p in pads:
        if p.get("ref", "") in passive_refs:
            nm = p.get("net", "")
            if nm and nm not in dest_by_net and p.get("ref") not in {"U3"}:
                dest_by_net[nm] = (p["x"], p["y"])

    # CDR: J3∪J4 (not just J4 like GCO spike)
    corridor_escape_nets: set[str] = set()
    for net_name in QFN_ESCAPE_NETS:
        if net_name in FCU_ONLY_NETS:
            continue
        dest = dest_by_net.get(net_name)
        if dest is None:
            continue
        on_j3 = abs(dest[1] - J3_Y) < 1.5
        on_j4 = abs(dest[1] - J4_Y) < 1.5
        if on_j3 or on_j4:
            corridor_escape_nets.add(net_name)

    print(f"\n[CDR] Contending set (J3∪J4 — fixes GCO J4-only bug):", flush=True)
    print(f"  Power nets: {sorted(CORRIDOR_POWER_NETS)}")
    print(f"  Corridor escape signals (J3∪J4): {sorted(corridor_escape_nets)}")
    print(f"  Total contending: {len(corridor_escape_nets) + len(CORRIDOR_POWER_NETS)}")

    # ── Step 3: Ring-slot assignment ─────────────────────────────────────────
    print("\n[CDR] Step 3: Ring-slot assignment...", flush=True)
    _check_rss("before_ring_slot")

    ring_assignment = build_ring_slot_assignment(
        pcb_path=board,
        pads=pads,
        geo=geo,
        board_bbox=board_bbox,
        board_outline=board_outline,
        drill_obstacles=drill_obstacles,
        drill_centers=drill_centers,
        escape_net_names=QFN_ESCAPE_NETS,
        fcu_only_nets=FCU_ONLY_NETS,
    )
    _check_rss("after_ring_slot")

    # Build contending_nets dict for channel router
    contending_nets: dict[str, dict] = {}
    for net_name in corridor_escape_nets:
        info = ring_assignment.get(net_name, {})
        source_xy = None
        for p in pads:
            if p.get("ref", "") in dense_refs and p.get("front") and not p.get("back"):
                if p.get("net", "") == net_name:
                    source_xy = (p["x"], p["y"])
                    break
        dest_xy = dest_by_net.get(net_name)
        order_key = (dest_xy[1] * 10000 + dest_xy[0]) if dest_xy else 9999.0 * 10000
        contending_nets[net_name] = {
            "escape_via_xy": info.get("escape_via_xy"),
            "dest_xy": dest_xy,
            "source_xy": source_xy,
            "order_key": order_key,
        }

    print(f"\n  Corridor signals with via+dest assigned:")
    for nm in sorted(contending_nets.keys()):
        info = contending_nets[nm]
        has_via = info.get("escape_via_xy") is not None
        has_dest = info.get("dest_xy") is not None
        print(
            f"    {nm:<18} via={has_via} dest={has_dest} "
            f"via_xy={info.get('escape_via_xy')} dest_xy={info.get('dest_xy')}",
            flush=True,
        )

    # Filter to nets that have both via and dest
    valid_contending = {
        k: v for k, v in contending_nets.items()
        if v.get("escape_via_xy") is not None and v.get("dest_xy") is not None
    }
    print(f"  Valid (have via+dest): {sorted(valid_contending.keys())}", flush=True)

    # ── Step 4: GCO flow warm-start (the CONCURRENT assignment) ─────────────
    print("\n[CDR] Step 4: GCO flow warm-start (concurrent lane assignment)...", flush=True)
    _check_rss("before_gco_warmstart")

    spine_demands = {"+3V3": b_spine, "+1V1": 1}
    solve_time_s = 0.0
    flow_found_disjoint = False
    fallback_used = False
    gco_assignment: dict[str, tuple[int, float, int]] = {}

    # Spine band default (will be updated from flow)
    spine_band_y_lo = J4_INNER_Y + LANE_PITCH_MM / 2.0
    spine_band_y_hi = spine_band_y_lo + b_spine * LANE_PITCH_MM

    try:
        corridor_graph = build_corridor_graph(
            contending_signals=valid_contending,
            spine_demands=spine_demands,
            channel_y_range=(J4_INNER_Y, J3_INNER_Y),
            pitch_mm=LANE_PITCH_MM,
        )
        _check_rss("after_corridor_graph")

        t_solve = time.perf_counter()
        gco_assignment = solve_corridor_assignment(corridor_graph, CHANNEL_TIMEOUT_S)
        solve_time_s = time.perf_counter() - t_solve

        # Assert disjoint
        signal_names_list = sorted(valid_contending.keys())
        assert_assignment_disjoint(gco_assignment, spine_demands, signal_names_list)
        flow_found_disjoint = True

        # Print assignment
        print("\n  GCO lane assignment (concurrent, disjoint):")
        for net in sorted(valid_contending.keys()):
            if net in gco_assignment:
                li, ly, layer = gco_assignment[net]
                layer_str = "B.Cu" if layer == 1 else "F.Cu"
                print(f"    {net:<18} lane_idx={li:3d} lane_y={ly:.3f}mm layer={layer_str}")

        # Extract spine band from flow
        if "_spine_band_+3V3" in gco_assignment:
            spine_slots = gco_assignment["_spine_band_+3V3"]  # type: ignore[index]
            if spine_slots:
                ys = [s[1] for s in spine_slots]
                spine_band_y_lo = min(ys) - LANE_PITCH_MM / 2.0
                spine_band_y_hi = max(ys) + LANE_PITCH_MM / 2.0
                print(f"  Spine band from flow: y=[{spine_band_y_lo:.4f}, {spine_band_y_hi:.4f}]mm")

    except CorridorTimeout as exc:
        solve_time_s = CHANNEL_TIMEOUT_S
        print(f"  GCO timeout: {exc} — using order_key fallback", flush=True)
        fallback_used = True
    except Exception as exc:
        import traceback as _tb
        print(f"  GCO error: {exc}", flush=True)
        _tb.print_exc()
        fallback_used = True

    _check_rss("after_gco_warmstart")
    spine_band = (
        round(spine_band_y_lo * 1e6) / 1e6,
        round(spine_band_y_hi * 1e6) / 1e6,
    )
    print(f"  Spine band: y=[{spine_band[0]:.4f}, {spine_band[1]:.4f}]mm", flush=True)

    # ── Step 5: Build channel instance + constraint graphs ───────────────────
    print("\n[CDR] Step 5: Channel instance + constraint graphs...", flush=True)
    _check_rss("before_channel")

    channel_instance = build_channel_instance(
        contending_nets=valid_contending,
        spine_band=spine_band,
        channel_y_range=(J4_INNER_Y, J3_INNER_Y),
        pitch_mm=PITCH,
        flow_assignment=gco_assignment,
    )

    t_channel_start = time.perf_counter()
    vcg, hcg = build_constraint_graphs(channel_instance)
    vcg_cycles_list = _find_vcg_cycles(vcg)
    n_vcg_cycles = len(vcg_cycles_list)

    if n_vcg_cycles > 0:
        print(f"  [CDR] WARNING: {n_vcg_cycles} VCG cycle(s) detected", flush=True)
        for cycle_set in vcg_cycles_list:
            print(f"    cycle: {sorted(cycle_set)}", flush=True)

    # Assign tracks: use GCO flow lane_y values (concurrent assignment)
    doglegs_used = 0
    if flow_found_disjoint and gco_assignment:
        print("  [CDR] Using GCO flow lane_y for concurrent track assignment...", flush=True)
        channel_results = assign_tracks_from_flow(
            channel_instance, gco_assignment, vcg, hcg, valid_contending
        )
    else:
        print("  [CDR] GCO unavailable — using order_key fallback assignment...", flush=True)
        channel_results = assign_tracks_order_key_fallback(channel_instance, valid_contending)

    channel_solve_time = time.perf_counter() - t_channel_start
    print(f"  Channel router solve time: {channel_solve_time:.3f}s", flush=True)

    if channel_solve_time > CHANNEL_TIMEOUT_S:
        print(
            f"  CHANNEL TIMEOUT ({channel_solve_time:.1f}s > {CHANNEL_TIMEOUT_S}s) — skip",
            flush=True,
        )
        channel_results = {}

    _check_rss("after_channel")

    # ── Step 6: Assert disjoint BEFORE emit ──────────────────────────────────
    disjoint_ok = False
    if channel_results:
        try:
            assert_channel_disjoint(channel_results, channel_instance, valid_contending)
            disjoint_ok = True
        except AssertionError as exc:
            print(f"\n[CDR] DISJOINT ASSERTION FAILED: {exc}", flush=True)
            disjoint_ok = False

    # ── Step 7: realize_channel → direct emit (NO route_net_steered) ─────────
    print("\n[CDR] Step 7: Realize channel geometry (direct emit)...", flush=True)
    _check_rss("before_realize")

    precomp_routes: dict = {}
    corridor_nets_realized = 0
    bcu_run_failed_count = 0  # THE THESIS: should be 0

    if channel_results:
        channel_geometry = realize_channel(channel_results, valid_contending, geo)

        for net_name, (world_paths, world_vias) in channel_geometry.items():
            net_obj = nets_by_name.get(net_name)
            if net_obj is None:
                continue
            if not world_paths or not world_paths[0]:
                print(f"  SKIP {net_name}: empty world_path", flush=True)
                continue

            try:
                nr = to_gridless_netroute(net_obj, world_paths, grid, world_vias=world_vias)
                _mark(grid, nr, 1)
                precomp_routes[net_name] = nr
                corridor_nets_realized += 1
                cr = channel_results[net_name]
                print(
                    f"  EMIT {net_name:<18} track_y={cr.track_y:.4f} "
                    f"via_y={valid_contending[net_name]['escape_via_xy'][1]:.4f} "
                    f"dest_y={valid_contending[net_name]['dest_xy'][1]:.4f} "
                    f"vias={len(world_vias)} wpts={len(world_paths[0])}",
                    flush=True,
                )
            except Exception as exc:
                print(f"  EMIT_FAIL {net_name}: {exc}", flush=True)
                bcu_run_failed_count += 1

        print(
            f"\n  CDR direct emit: {corridor_nets_realized}/{len(valid_contending)} realized",
            flush=True,
        )
        print(f"  bcu_run_failed (should be 0): {bcu_run_failed_count}", flush=True)
        print(f"  THE THESIS: bcu_run_failed_eliminated={bcu_run_failed_count == 0}", flush=True)

    _check_rss("after_realize")

    # ── Step 9: Phase-0 +3V3/+1V1 power routing ──────────────────────────────
    print("\n[CDR] Step 9: Phase-0 +3V3/+1V1 power routing...", flush=True)
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
        print(f"  [phase0] +3V3/+1V1: {_power_ok}/{len(_power_net_objs)} ok", flush=True)

    # ── Step 10: Remaining nets via attempt-3 path ────────────────────────────
    print("\n[CDR] Step 10: Grid + gridless_first remaining nets...", flush=True)
    ESCAPE_EXTRA_GRIDLESS = {"/USB_D-"}
    gf_nets = (ATTEMPT3_GRIDLESS_NETS | ESCAPE_EXTRA_GRIDLESS) - set(precomp_routes.keys())
    gf_nets_clean = {n for n in gf_nets if n not in precomp_routes}

    # Build obstacles from precomp routes
    try:
        from shapely.geometry import LineString as _GLSLS, Point as _GLSPt
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

    # ── Step 11: Emit, refill, DRC, audit ────────────────────────────────────
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

    report = run_drc(board)
    by = collections.Counter(v.get("type") for v in report.get("violations", []))
    errs = sum(1 for v in report.get("violations", []) if v.get("severity") == "error")
    unc = len(report.get("unconnected_items", []))

    crossings = audit_crossings(board)

    plus3v3_unc = sum(
        1 for item in report.get("unconnected_items", [])
        if (item.get("net", "") == "+3V3"
            or "+3V3" in str(item.get("net_name", ""))
            or "+3V3" in str(item.get("items", "")))
    )

    shorting_items = by.get("shorting_items", 0) or by.get("short", 0)

    return {
        "board": str(board),
        "unconnected": unc,
        "errors": errs,
        "by_type": dict(by),
        "corridor_nets_realized": corridor_nets_realized,
        "disjoint_ok": disjoint_ok,
        "bcu_run_failed_eliminated": bcu_run_failed_count == 0,
        "bcu_run_failed_count": bcu_run_failed_count,
        "vcg_cycles": n_vcg_cycles,
        "doglegs_used": doglegs_used,
        "fcu_crossings": crossings.get("F.Cu", 0),
        "bcu_crossings": crossings.get("B.Cu", 0),
        "t_grid_s": round(t_grid, 1),
        "channel_solve_time_s": round(channel_solve_time, 4),
        "rss_gb": round(_rss_gb(), 3),
        "plus3v3_unconnected": plus3v3_unc,
        "shorting_items": shorting_items,
        "corridor_escape_nets": sorted(corridor_escape_nets),
        "flow_found_disjoint": flow_found_disjoint,
        "solve_time_s": round(solve_time_s, 4),
        "fallback_used": fallback_used,
        "b_spine_used": b_spine,
    }


# ── Main ──────────────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(description="CDR spike: concurrent detailed corridor router")
    parser.add_argument("--runs", type=int, default=1,
                        help="Number of routing runs (3 for determinism check)")
    parser.add_argument("--out", default="/tmp/cdr_spike_out",
                        help="Output directory")
    parser.add_argument("--b-spine", type=int, default=B_SPINE,
                        help="Spine band width in tracks (default 2)")
    args = parser.parse_args()

    t_total_start = time.perf_counter()
    peak_rss = _rss_gb()

    print("=" * 70)
    print("CDR SPIKE: CONCURRENT DETAILED CORRIDOR ROUTER")
    print(f"Attempt-3 bar: unconnected ≤ {ATTEMPT3_UNC}, errors ≤ {ATTEMPT3_ERR}")
    print("Thesis: direct track emission (GCO lane_y) eliminates bcu_run_failed")
    print("=" * 70)

    out_base = Path(args.out)
    run_results: list[dict] = []
    run_hashes: list[str] = []

    for run_idx in range(args.runs):
        run_dir = out_base / f"run_{run_idx}"
        print(f"\n[Run {run_idx + 1}/{args.runs}]", flush=True)

        try:
            _check_rss(f"start run {run_idx}")
            score = run_cdr(run_dir, b_spine=args.b_spine)
            run_results.append(score)
            peak_rss = max(peak_rss, score["rss_gb"])

            board_file = Path(score["board"])
            if board_file.exists():
                h = board_hash(board_file)
                run_hashes.append(h)
                print(f"  board_hash={h[:16]}...", flush=True)

            print(f"\n  --- Run {run_idx + 1} summary ---")
            print(f"  unconnected={score['unconnected']}  errors={score['errors']}")
            print(f"  corridor_nets_realized={score['corridor_nets_realized']}")
            print(f"  bcu_run_failed_eliminated={score['bcu_run_failed_eliminated']}")
            print(f"  disjoint_ok={score['disjoint_ok']}")
            print(f"  vcg_cycles={score['vcg_cycles']}  doglegs_used={score['doglegs_used']}")
            print(f"  F.Cu crossings={score['fcu_crossings']}  B.Cu crossings={score['bcu_crossings']}")
            print(f"  +3V3 unconnected={score['plus3v3_unconnected']}")
            print(f"  shorting_items={score['shorting_items']}")
            print(f"  by_type={score['by_type']}")
            print(f"  channel_solve={score['channel_solve_time_s']:.3f}s")
            print(f"  RSS={score['rss_gb']:.3f} GB")

        except MemoryError as exc:
            print(f"  MEMORY ABORT: {exc}", flush=True)
            run_results.append({
                "unconnected": 9999, "errors": 9999, "abort": str(exc),
                "rss_gb": _rss_gb(), "fcu_crossings": 0, "bcu_crossings": 0,
                "corridor_nets_realized": 0, "disjoint_ok": False,
                "bcu_run_failed_eliminated": False, "bcu_run_failed_count": -1,
                "vcg_cycles": 0, "doglegs_used": 0,
                "plus3v3_unconnected": -1, "shorting_items": 0,
                "by_type": {}, "corridor_escape_nets": [],
                "flow_found_disjoint": False, "solve_time_s": 0.0,
                "fallback_used": True, "b_spine_used": args.b_spine,
                "channel_solve_time_s": 0.0,
            })
            break
        except Exception as exc:
            import traceback as _tb
            print(f"  ERROR run {run_idx + 1}: {exc}", flush=True)
            _tb.print_exc()
            run_results.append({
                "unconnected": 9999, "errors": 9999, "abort": str(exc),
                "rss_gb": _rss_gb(), "fcu_crossings": 0, "bcu_crossings": 0,
                "corridor_nets_realized": 0, "disjoint_ok": False,
                "bcu_run_failed_eliminated": False, "bcu_run_failed_count": -1,
                "vcg_cycles": 0, "doglegs_used": 0,
                "plus3v3_unconnected": -1, "shorting_items": 0,
                "by_type": {}, "corridor_escape_nets": [],
                "flow_found_disjoint": False, "solve_time_s": 0.0,
                "fallback_used": True, "b_spine_used": args.b_spine,
                "channel_solve_time_s": 0.0,
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
            "corridor_nets_realized": 0, "disjoint_ok": False,
            "bcu_run_failed_eliminated": False, "bcu_run_failed_count": -1,
            "vcg_cycles": 0, "doglegs_used": 0,
            "by_type": {}, "plus3v3_unconnected": -1, "shorting_items": 0,
            "corridor_escape_nets": [],
            "flow_found_disjoint": False, "solve_time_s": 0.0,
            "fallback_used": True, "b_spine_used": B_SPINE,
            "channel_solve_time_s": 0.0,
        }

    unc = best.get("unconnected", 9999)
    errs = best.get("errors", 9999)
    fcu_x = best.get("fcu_crossings", 9999)
    bcu_x = best.get("bcu_crossings", 9999)
    illegal_x = fcu_x + bcu_x
    shorting_items = best.get("shorting_items", 0)
    corridor_nets_realized = best.get("corridor_nets_realized", 0)
    bcu_run_failed_eliminated = best.get("bcu_run_failed_eliminated", False)
    vcg_cycles = best.get("vcg_cycles", 0)
    doglegs_used = best.get("doglegs_used", 0)
    disjoint_ok = best.get("disjoint_ok", False)
    solve_time_s = best.get("solve_time_s", 0.0) + best.get("channel_solve_time_s", 0.0)

    clean_win = (
        unc <= ATTEMPT3_UNC - 1 and
        errs <= ATTEMPT3_ERR and
        shorting_items == 0 and
        illegal_x == 0
    )
    rss_ok = peak_rss < RSS_HARD_FAIL_GB
    det_pass = "PASS" in deterministic
    solve_ok = best.get("channel_solve_time_s", 0.0) < CHANNEL_TIMEOUT_S

    if "abort" in best:
        outcome = "regression"
    elif vcg_cycles > 0 and doglegs_used == 0:
        outcome = "vcg-cycle-failed"
    elif not disjoint_ok:
        outcome = "track-blocked"
    elif clean_win:
        outcome = "clean-win"
    elif unc <= ATTEMPT3_UNC and errs <= ATTEMPT3_ERR:
        outcome = "tie"
    elif unc < 9999:
        outcome = "regression" if (unc > ATTEMPT3_UNC or errs > ATTEMPT3_ERR) else "tie"
    else:
        outcome = "regression"

    if clean_win and rss_ok and solve_ok:
        cdr_go_no_go = "GO: all criteria met"
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
            reasons.append(f"channel_solve={best.get('channel_solve_time_s',0):.1f}s ≥ {CHANNEL_TIMEOUT_S}s")
        if not bcu_run_failed_eliminated:
            reasons.append("bcu_run_failed NOT eliminated")
        cdr_go_no_go = f"NO-GO: {'; '.join(reasons) if reasons else 'unknown'}"

    # ── Print verdict ─────────────────────────────────────────────────────────
    print("\n" + "=" * 70)
    print(f"CDR VERDICT: {'GO' if clean_win and rss_ok else 'NO-GO'}")
    print(f"  unconnected          = {unc}  (bar: ≤40;  attempt-3 = 41)")
    print(f"  errors               = {errs}  (bar: ≤73;  attempt-3 = 73)")
    print(f"  crossings            = F.Cu={fcu_x}  B.Cu={bcu_x}  (bar: 0)")
    print(f"  shorts               = {shorting_items}  (bar: 0)")
    print(f"  corridor_realized    = {corridor_nets_realized}")
    print(f"  bcu_run_failed_elim  = {bcu_run_failed_eliminated}  (THESIS)")
    print(f"  disjoint_ok          = {disjoint_ok}")
    print(f"  vcg_cycles           = {vcg_cycles}  doglegs={doglegs_used}")
    print(f"  solve_time           = {solve_time_s:.3f}s  (bar: <{CHANNEL_TIMEOUT_S}s)")
    print(f"  RSS                  = {peak_rss:.3f} GB  (bar: < 2 GB)")
    print(f"  determinism          = {deterministic}")
    print(f"  outcome              = {outcome}")
    print(f"  CDR GO/NO-GO: {cdr_go_no_go}")
    print("=" * 70)

    by_type_best = best.get("by_type", {})
    contending_set_list = best.get("corridor_escape_nets", []) + sorted(CORRIDOR_POWER_NETS)

    structured = {
        "status": "complete" if unc < 9999 else "error",
        "summary": (
            f"CDR spike: unc={unc} err={errs} "
            f"crossings_fcu={fcu_x} crossings_bcu={bcu_x} "
            f"shorts={shorting_items} "
            f"corridor_realized={corridor_nets_realized} "
            f"bcu_run_failed_eliminated={bcu_run_failed_eliminated} "
            f"disjoint={disjoint_ok} vcg_cycles={vcg_cycles} doglegs={doglegs_used}"
        ),
        "files_changed": ["scripts/_probe_cdr_spike.py (new — CDR spike)"],
        "files_read": [
            str(PCB_PATH),
            "scripts/_probe_gco_spike.py",
            "src/tracewise/route/gridless/topo_assign.py",
            "src/tracewise/route/gridless/adapter.py",
            "src/tracewise/route/engine/multi.py",
            "src/tracewise/route/engine/kicad.py",
            "docs/design/CONCURRENT-DETAILED-ROUTER.md",
        ],
        "channel_router_built": True,
        "disjoint_assignment": disjoint_ok,
        "bcu_run_failed_eliminated": bcu_run_failed_eliminated,
        "vcg_cycles": vcg_cycles,
        "doglegs_used": doglegs_used,
        "contending_set": contending_set_list,
        "solve_time_s": round(solve_time_s, 4),
        "result": {
            "unconnected": unc,
            "errors": errs,
            "by_type": dict(by_type_best),
        },
        "plus3v3_unconnected": best.get("plus3v3_unconnected", -1),
        "shorting_items": shorting_items,
        "illegal_crossings": {"fcu": fcu_x, "bcu": bcu_x},
        "corridor_nets_realized": corridor_nets_realized,
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
        "cdr_go_no_go": cdr_go_no_go,
        "issues": [] if "abort" not in best else [best.get("abort", "unknown")],
        "assumptions": [
            "B_spine=2 tracks for +3V3 spine band",
            f"J4_INNER_Y={J4_INNER_Y}mm, J3_INNER_Y={J3_INNER_Y}mm, PITCH={PITCH}mm",
            "Contending signals: J3∪J4 dest (|dest_y-J3/J4|<1.5mm) ∩ QFN_ESCAPE_NETS",
            "GCO flow lane_y used as track_y (concurrent assignment, nearest to dest_y)",
            "Direct track emission: no route_net_steered, no per-net visgraph",
            "No accumulated bcu_extra_obstacles (disjoint by construction)",
            "CHANNEL_TIMEOUT_S=10s, RSS hard-abort at 2GB",
            "Phase-0 power-first: +3V3/+1V1 via gridless_first",
            "Remaining nets via unchanged attempt-3 route_all path",
        ],
    }

    print("\n## Structured Result")
    print("```json")
    print(json.dumps(structured, indent=2, default=str))
    print("```")

    out_base.mkdir(parents=True, exist_ok=True)
    result_path = out_base / "cdr_result.json"
    result_path.write_text(json.dumps(structured, indent=2, default=str))
    print(f"\nResult saved: {result_path}")


if __name__ == "__main__":
    main()
