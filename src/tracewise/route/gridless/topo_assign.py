"""topo_assign — Global ring-slot assignment pre-pass (TCR Steps A-C).

Replaces the per-net witness-via extraction in _probe_tcr_e1.py with a
GLOBAL assignment that:

  Step A  Generate K=10-12 candidate via slots PER EDGE (N/S/E/W) in the
          ring band [r_min, r_max] from the component centroid.  Each slot is
          filtered by ``is_legal_via`` (3-predicate: copper ring both layers +
          drill-to-copper + drill-to-drill).  Deterministic: slots are sorted
          by (edge, angle).

  Step B  Assign nets to slots via CYCLIC-ORDER MATCHING: sort escape nets by
          source-pad angle θ around the component; assign slots in monotone
          angular order so the escape fan is crossing-free; prefer the slot on
          the net's facing edge.  No two nets share a slot.

  Step C  Return the assignment as a dict[net_name] = {"escape_via_xy": (x,y),
          ...} that ``_probe_tcr_e1.run_e1`` feeds into ``route_net_steered``.

All geometry in mm; KiCad coordinate frame (y grows down).
"""

from __future__ import annotations

import math
from typing import Literal

# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

RING_R_MIN: float = 4.5   # inner radius of the witness ring band (mm)
RING_R_MAX: float = 8.0   # outer radius (from _invent1_human_stats: 4.55-8.18mm)
SLOTS_PER_EDGE: int = 12  # target K per edge (N/S/E/W)

EdgeKey = Literal["E", "S", "W", "N"]

_EDGE_ANGLE_RANGES: dict[EdgeKey, tuple[float, float]] = {
    "E": (-45.0, 45.0),    # -45..45 deg (right)
    "S": (45.0, 135.0),    # 45..135 deg (down, KiCad y-down)
    "W": (135.0, 225.0),   # 135..225 deg (left)
    "N": (225.0, 315.0),   # 225..315 deg (up)
}


def _angle_to_edge(angle_deg: float) -> EdgeKey:
    """Map angle (−180..180) to the nearest QFN edge (E/S/W/N)."""
    a = angle_deg % 360.0
    for edge, (lo, hi) in _EDGE_ANGLE_RANGES.items():
        if lo <= a < hi:
            return edge
    return "E"  # default for exactly ±180 / 0 boundary


def generate_ring_slots(
    component_cx: float,
    component_cy: float,
    pads: list[dict],
    geo: dict,
    board_bbox: tuple[float, float, float, float],
    board_outline: object | None,
    drill_obstacles: list,
    drill_centers: list[tuple[float, float, float]],
    r_min: float = RING_R_MIN,
    r_max: float = RING_R_MAX,
    slots_per_edge: int = SLOTS_PER_EDGE,
    radial_steps: int = 3,
) -> dict[EdgeKey, list[tuple[float, float]]]:
    """Step A: generate legal via slots in the ring band around component_cx,cy.

    For each edge (E/S/W/N), generate ``slots_per_edge`` angular candidates
    uniformly distributed in that edge's ±45° sector at each of ``radial_steps``
    radii in [r_min, r_max].  Filter by ``is_legal_via`` (net-agnostic check
    using a sentinel empty-string net so no carve-outs interfere).  Return the
    FIRST ``slots_per_edge`` legal slots per edge (sorted by angle, deterministic).

    Parameters
    ----------
    component_cx, component_cy:
        Component centroid (U3 center).
    pads:
        All board pads.
    geo:
        Project geometry dict (track_mm, clearance_mm, via_mm, via_drill_mm,
        hole_clearance_mm, hole_to_hole_mm).
    board_bbox:
        (x1, y1, x2, y2) board boundary.
    board_outline:
        Optional Shapely Polygon for the board edge clip.
    drill_obstacles:
        Pre-inflated drill obstacle circles.
    drill_centers:
        (cx, cy, drill_r) tuples for hole-to-hole predicate.
    r_min, r_max:
        Inner/outer radius of the witness ring band.
    slots_per_edge:
        Maximum legal slots to keep per edge.
    radial_steps:
        Number of radii to sample between r_min and r_max.

    Returns
    -------
    dict mapping each edge key → sorted list of (x, y) legal slot positions.
    """
    try:
        from tracewise.route.gridless.geom import (
            build_windowed_free_space,
            is_legal_via,
        )
    except ImportError as exc:
        raise ImportError(
            "topo_assign requires shapely; pip install tracewise[gridless]"
        ) from exc

    track_mm = geo["track_mm"]
    clearance_mm = geo["clearance_mm"]
    via_mm = geo.get("via_mm", 0.4)
    via_drill = geo.get("via_drill_mm", 0.2)
    hole_clearance = geo.get("hole_clearance_mm", 0.25)
    hole_to_hole = geo.get("hole_to_hole_mm", 0.25)
    bx1, by1, bx2, by2 = board_bbox

    # Build per-layer free spaces over the whole ring-band window.
    # Use an empty net_name="" so no own-pad carve-out distorts the check;
    # the is_legal_via call per-net below will catch proximity to own pads.
    win_r = r_max + via_mm / 2.0 + clearance_mm + 1.0  # generous
    wx1 = max(component_cx - win_r, bx1)
    wy1 = max(component_cy - win_r, by1)
    wx2 = min(component_cx + win_r, bx2)
    wy2 = min(component_cy + win_r, by2)
    window_bbox = (wx1, wy1, wx2, wy2)

    # Use a sentinel net name "_slot_check_" — no pad has this net so the
    # free space will have all pads as obstacles (the most conservative check).
    SENTINEL_NET = "_slot_check_"
    fs_F, _ = build_windowed_free_space(
        pads, SENTINEL_NET, clearance_mm, track_mm,
        [], window_bbox,
        board_outline=board_outline,
        drill_obstacles=drill_obstacles,
        layer=0,
    )
    fs_B, _ = build_windowed_free_space(
        pads, SENTINEL_NET, clearance_mm, track_mm,
        [], window_bbox,
        board_outline=board_outline,
        drill_obstacles=drill_obstacles,
        layer=1,
    )

    # Radii to sample (uniform spacing r_min..r_max)
    if radial_steps <= 1:
        radii = [(r_min + r_max) / 2.0]
    else:
        step = (r_max - r_min) / (radial_steps - 1)
        radii = [r_min + i * step for i in range(radial_steps)]

    result: dict[EdgeKey, list[tuple[float, float]]] = {"E": [], "S": [], "W": [], "N": []}

    # For each edge, sample angular positions uniformly in its ±45° sector.
    # We over-generate then take the first `slots_per_edge` legal ones.
    n_angular = max(slots_per_edge * 3, 30)  # over-sample 3x for filtering

    for edge, (lo_deg, hi_deg) in _EDGE_ANGLE_RANGES.items():
        candidates: list[tuple[float, float, float]] = []  # (angle, x, y)

        for ai in range(n_angular + 1):
            frac = ai / max(n_angular, 1)
            angle_deg = lo_deg + frac * (hi_deg - lo_deg)
            angle_rad = math.radians(angle_deg)

            for r in radii:
                x = component_cx + r * math.cos(angle_rad)
                y = component_cy + r * math.sin(angle_rad)

                # Clamp to board bounds (skip out-of-board)
                if x < bx1 or x > bx2 or y < by1 or y > by2:
                    continue

                # Snap to 1 nm grid for determinism
                x = round(x * 1e6) / 1e6
                y = round(y * 1e6) / 1e6

                ok, _ = is_legal_via(
                    (x, y), fs_F, fs_B, pads, SENTINEL_NET,
                    via_mm, via_drill, clearance_mm, hole_clearance, hole_to_hole,
                    drill_centers, window_bbox,
                )
                if ok:
                    candidates.append((angle_deg, x, y))

        # Sort by angle, then deduplicate by proximity (via_mm + clearance_mm pitch).
        # Check against ALL previously accepted slots (not just the last) so that
        # slots at different radii but similar angles are also deduplicated.
        candidates.sort(key=lambda t: t[0])
        min_slot_pitch = via_mm + clearance_mm
        deduped: list[tuple[float, float]] = []
        for _, x, y in candidates:
            # Reject if too close to ANY already-accepted slot
            too_close = any(
                math.hypot(x - ex, y - ey) < min_slot_pitch * 0.8
                for ex, ey in deduped
            )
            if too_close:
                continue
            deduped.append((x, y))
            if len(deduped) >= slots_per_edge:
                break

        result[edge] = deduped

    return result


def assign_nets_to_slots(
    escape_nets_info: dict[str, dict],
    ring_slots: dict[EdgeKey, list[tuple[float, float]]],
    component_cx: float,
    component_cy: float,
    fcu_only_nets: set[str],
) -> dict[str, dict]:
    """Step B: assign each escape net one legal ring slot.

    Sort escape nets by source-pad angle θ around the component (cyclic order).
    Assign slots in monotone angular order so the escape fan is crossing-free.
    Prefer the slot on the net's FACING edge.  No two nets share a slot.

    Parameters
    ----------
    escape_nets_info:
        Dict produced by ``_collect_net_info`` — keys are net names, values
        carry ``source_xy``, ``dest_xy``, ``order_key``, ``lane_y_mm``.
    ring_slots:
        Legal slot positions per edge from ``generate_ring_slots``.
    component_cx, component_cy:
        Component centroid.
    fcu_only_nets:
        Net names that must NOT receive a via slot (F.Cu-only short nets).

    Returns
    -------
    Dict[net_name] = {
        "escape_via_xy": (x, y) | None,
        "lane_y_mm": float | None,
        "dest_xy": (x, y) | None,
        "source_xy": (x, y) | None,
        "order_key": float,
    }
    """
    # Flatten all slots with their edge labels, sorted by global angle from center
    all_slots: list[tuple[float, EdgeKey, tuple[float, float]]] = []
    for edge, slots in ring_slots.items():
        for x, y in slots:
            angle = math.degrees(math.atan2(y - component_cy, x - component_cx)) % 360.0
            all_slots.append((angle, edge, (x, y)))
    all_slots.sort(key=lambda t: t[0])  # deterministic angle order

    # Sort escape nets by source-pad angle (for crossing-free monotone fan)
    nets_with_angle: list[tuple[float, str, dict]] = []
    for net_name, info in escape_nets_info.items():
        if net_name in fcu_only_nets:
            continue
        src = info.get("source_xy")
        if src is None:
            continue
        angle = math.degrees(
            math.atan2(src[1] - component_cy, src[0] - component_cx)
        ) % 360.0
        nets_with_angle.append((angle, net_name, info))
    nets_with_angle.sort(key=lambda t: t[0])

    # Two-phase monotone assignment:
    # Phase 1: group nets by facing edge, assign slots from the SAME edge in
    #           the same angular order as source pads (crossing-free by construction).
    # Phase 2: any net that couldn't get a same-edge slot falls back to the
    #           globally nearest unassigned slot.
    used_slots: set[int] = set()  # indices into all_slots
    assignment: dict[str, tuple[float, float]] = {}

    # Build per-edge lists of (net_angle, net_name) sorted by angle
    _adj_edges: dict[str, set[str]] = {
        "E": {"S", "N"}, "S": {"E", "W"}, "W": {"N", "S"}, "N": {"E", "W"}
    }
    edge_nets: dict[str, list[tuple[float, str]]] = {"E": [], "S": [], "W": [], "N": []}
    unplaceable: list[tuple[float, str, dict]] = []  # nets without source_xy

    for net_angle, net_name, info in nets_with_angle:
        src = info.get("source_xy")
        if src is None:
            unplaceable.append((net_angle, net_name, info))
            continue
        facing_edge = _angle_to_edge(
            math.degrees(math.atan2(src[1] - component_cy, src[0] - component_cx))
        )
        edge_nets[facing_edge].append((net_angle, net_name))

    # Per-edge: sort slots for that edge by slot angle; assign in matching
    # angular order (lowest net angle → lowest slot angle, etc.).
    for edge, net_list in edge_nets.items():
        if not net_list:
            continue
        # Find all unassigned slots on this edge, sorted by slot angle
        edge_slot_indices = sorted(
            [i for i, (_, se, _) in enumerate(all_slots) if se == edge and i not in used_slots],
            key=lambda i: all_slots[i][0],  # sort by slot angle
        )
        # Net list is already sorted by source pad angle (nets_with_angle was sorted)
        for k, (_angle, net_name) in enumerate(net_list):
            if k < len(edge_slot_indices):
                idx = edge_slot_indices[k]
                used_slots.add(idx)
                assignment[net_name] = all_slots[idx][2]
            else:
                # Not enough same-edge slots: defer to Phase 2
                unplaceable.append((net_list[k][0], net_name,
                                    escape_nets_info[net_name]))

    # Phase 2: assign leftover nets to nearest unassigned slot (any edge)
    for net_angle, net_name, _info in sorted(unplaceable, key=lambda t: t[0]):
        best_idx: int | None = None
        best_dist = float("inf")
        for i, (slot_angle, _se, _sxy) in enumerate(all_slots):
            if i in used_slots:
                continue
            d = abs((slot_angle - net_angle + 180.0) % 360.0 - 180.0)
            if d < best_dist:
                best_dist = d
                best_idx = i
        if best_idx is not None:
            used_slots.add(best_idx)
            assignment[net_name] = all_slots[best_idx][2]

    # Build result dict (same structure as extract_topo_classes output)
    result: dict[str, dict] = {}
    for net_name, info in escape_nets_info.items():
        dest_xy = info.get("dest_xy")
        source_xy = info.get("source_xy")
        order_key = info.get("order_key", 9999.0 * 10000)
        lane_y_mm = info.get("lane_y_mm")

        if net_name in fcu_only_nets:
            # F.Cu-only: no via
            escape_via_xy = None
        else:
            escape_via_xy = assignment.get(net_name)

        result[net_name] = {
            "escape_via_xy": escape_via_xy,
            "lane_y_mm": lane_y_mm,
            "dest_xy": dest_xy,
            "source_xy": source_xy,
            "order_key": order_key,
        }

    return result


def build_ring_slot_assignment(
    pcb_path: object,
    pads: list[dict],
    geo: dict,
    board_bbox: tuple[float, float, float, float],
    board_outline: object | None,
    drill_obstacles: list,
    drill_centers: list[tuple[float, float, float]],
    escape_net_names: set[str],
    fcu_only_nets: set[str],
    r_min: float = RING_R_MIN,
    r_max: float = RING_R_MAX,
    slots_per_edge: int = SLOTS_PER_EDGE,
) -> dict[str, dict]:
    """Top-level TCR Steps A-C: generate slots and assign escape nets.

    Collects source/dest pad info from ``pads``, runs Step A (generate_ring_slots)
    and Step B (assign_nets_to_slots).  Returns the same dict shape as
    ``extract_topo_classes`` in ``_probe_tcr_e1.py``.

    Parameters
    ----------
    pcb_path:
        Path to the .kicad_pcb file (for dest-pad lookup).
    pads:
        All board pads from ``kicad_extract_pads``.
    geo:
        Project geometry dict.
    board_bbox, board_outline, drill_obstacles, drill_centers:
        Board geometry helpers.
    escape_net_names:
        Set of net names to assign (the QFN escape nets).
    fcu_only_nets:
        Subset that route entirely on F.Cu (no via needed).
    r_min, r_max:
        Ring band radii.
    slots_per_edge:
        Slots to generate per edge.

    Returns
    -------
    Dict matching ``extract_topo_classes`` shape:
    ``{net_name: {escape_via_xy, lane_y_mm, dest_xy, source_xy, order_key}}``
    """
    from tracewise.route.gridless.geom import detect_dense_components

    dense_comps = detect_dense_components(pads)
    u3 = next((d for d in dense_comps if d["ref"] == "U3"), None)
    if u3 is None:
        raise RuntimeError("U3 (RP2040 QFN) not found in detect_dense_components")

    u3x, u3y = u3["cx"], u3["cy"]
    dense_refs = {d["ref"] for d in dense_comps}

    # --- Collect source pads (QFN F.Cu pads) per net ---
    source_by_net: dict[str, tuple[float, float]] = {}
    for p in pads:
        if p.get("ref", "") not in dense_refs:
            continue
        if not (p.get("front") and not p.get("back")):
            continue
        nm = p.get("net", "")
        if nm and nm not in source_by_net:
            source_by_net[nm] = (p["x"], p["y"])

    # --- Collect dest pads (connector/passive pads) per net ---
    # Mirrors the logic from extract_topo_classes in _probe_tcr_e1.py:
    # prefer J3/J4 inner row, fall back to passives.
    connector_refs = {"J3", "J4", "J5"}
    passive_refs = {"Y1", "C17", "R9", "SW1"}
    J3_INNER_Y = 98.20
    J4_INNER_Y = 80.40

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
            best = min(plist, key=lambda p: abs(p["y"] - J3_INNER_Y))
        elif ref == "J4":
            best = min(plist, key=lambda p: abs(p["y"] - J4_INNER_Y))
        else:
            best = plist[0]
        dest_by_net[nm] = (best["x"], best["y"])

    for p in pads:
        if p.get("ref", "") in passive_refs:
            nm = p.get("net", "")
            if nm and nm not in dest_by_net and p.get("ref") not in {"U3"}:
                dest_by_net[nm] = (p["x"], p["y"])

    # --- Build escape_nets_info (no via assigned yet — Step B does that) ---
    escape_nets_info: dict[str, dict] = {}
    for net_name in escape_net_names:
        source_xy = source_by_net.get(net_name)
        dest_xy = dest_by_net.get(net_name)
        if dest_xy is not None:
            order_key = dest_xy[1] * 10000 + dest_xy[0]
        else:
            order_key = 9999.0 * 10000

        escape_nets_info[net_name] = {
            "escape_via_xy": None,  # to be filled by Step B
            "lane_y_mm": None,      # no witness lane — B.Cu run uses free routing
            "dest_xy": dest_xy,
            "source_xy": source_xy,
            "order_key": order_key,
        }

    # --- Step A: generate legal ring slots ---
    print(
        f"  [ring_slot] Generating slots: r=[{r_min},{r_max}]mm "
        f"K={slots_per_edge}/edge around U3=({u3x:.3f},{u3y:.3f})",
        flush=True,
    )
    ring_slots = generate_ring_slots(
        component_cx=u3x,
        component_cy=u3y,
        pads=pads,
        geo=geo,
        board_bbox=board_bbox,
        board_outline=board_outline,
        drill_obstacles=drill_obstacles,
        drill_centers=drill_centers,
        r_min=r_min,
        r_max=r_max,
        slots_per_edge=slots_per_edge,
    )
    for edge, slots in ring_slots.items():
        print(f"  [ring_slot]   edge {edge}: {len(slots)} legal slots", flush=True)

    total_slots = sum(len(v) for v in ring_slots.values())
    n_via_nets = len([n for n in escape_net_names if n not in fcu_only_nets])
    if total_slots < n_via_nets:
        raise RuntimeError(
            f"ring_slot: only {total_slots} legal slots for {n_via_nets} via nets — "
            "increase r_max or reduce slot density requirements"
        )

    # --- Step B: assign nets to slots ---
    assignment = assign_nets_to_slots(
        escape_nets_info=escape_nets_info,
        ring_slots=ring_slots,
        component_cx=u3x,
        component_cy=u3y,
        fcu_only_nets=fcu_only_nets,
    )

    # Verify all via-needing nets got a slot
    unassigned = [
        nm for nm in escape_net_names
        if nm not in fcu_only_nets
        and assignment.get(nm, {}).get("escape_via_xy") is None
    ]
    if unassigned:
        print(
            f"  [ring_slot] WARNING: {len(unassigned)} nets unassigned: {unassigned}",
            flush=True,
        )
    else:
        print(
            f"  [ring_slot] All {n_via_nets} via-nets assigned legal slots",
            flush=True,
        )

    return assignment


def verify_assignment(
    assignment: dict[str, dict],
    component_cx: float,
    component_cy: float,
    pads: list[dict],
    geo: dict,
    board_bbox: tuple[float, float, float, float],
    board_outline: object | None,
    drill_obstacles: list,
    drill_centers: list[tuple[float, float, float]],
    fcu_only_nets: set[str],
) -> dict:
    """Verify the assignment: slots legal, disjoint, crossing-free fan.

    Returns a summary dict for test assertions and reporting:
    {
        "all_legal": bool,
        "all_disjoint": bool,
        "crossing_free_fan": bool,
        "illegal_nets": [net_name, ...],
        "collision_pairs": [(net_a, net_b), ...],
        "crossing_pairs": [(net_a, net_b), ...],
        "run_swclk_legal": bool,
        "total_via_nets": int,
        "assigned_count": int,
    }
    """
    from tracewise.route.gridless.geom import (
        build_windowed_free_space,
        is_legal_via,
    )

    track_mm = geo["track_mm"]
    clearance_mm = geo["clearance_mm"]
    via_mm = geo.get("via_mm", 0.4)
    via_drill = geo.get("via_drill_mm", 0.2)
    hole_clearance = geo.get("hole_clearance_mm", 0.25)
    hole_to_hole = geo.get("hole_to_hole_mm", 0.25)
    bx1, by1, bx2, by2 = board_bbox

    # Build free-space for the ring window
    win_r = RING_R_MAX + via_mm / 2.0 + clearance_mm + 1.5
    wx1 = max(component_cx - win_r, bx1)
    wy1 = max(component_cy - win_r, by1)
    wx2 = min(component_cx + win_r, bx2)
    wy2 = min(component_cy + win_r, by2)
    window_bbox = (wx1, wy1, wx2, wy2)

    illegal_nets: list[str] = []
    via_slots: dict[str, tuple[float, float]] = {}

    for net_name, info in assignment.items():
        if net_name in fcu_only_nets:
            continue
        via_xy = info.get("escape_via_xy")
        if via_xy is None:
            # Skip nets that had no source pad on U3 (can't get ring-band slot)
            # vs those that should have had a slot but didn't get one.
            if info.get("source_xy") is not None:
                illegal_nets.append(net_name)  # had a source pad but no slot assigned
            continue

        # Per-net legality check with own-pad carve-out
        fs_F, _ = build_windowed_free_space(
            pads, net_name, clearance_mm, track_mm,
            [], window_bbox,
            board_outline=board_outline,
            drill_obstacles=drill_obstacles,
            layer=0,
        )
        fs_B, _ = build_windowed_free_space(
            pads, net_name, clearance_mm, track_mm,
            [], window_bbox,
            board_outline=board_outline,
            drill_obstacles=drill_obstacles,
            layer=1,
        )
        ok, reason = is_legal_via(
            via_xy, fs_F, fs_B, pads, net_name,
            via_mm, via_drill, clearance_mm, hole_clearance, hole_to_hole,
            drill_centers, window_bbox,
        )
        if not ok:
            illegal_nets.append(net_name)
        else:
            via_slots[net_name] = via_xy

    # Disjointness: no two nets share the same slot (within 0.05mm)
    collision_pairs: list[tuple[str, str]] = []
    slot_list = list(via_slots.items())
    for i in range(len(slot_list)):
        for j in range(i + 1, len(slot_list)):
            na, (ax, ay) = slot_list[i]
            nb, (bx, by_) = slot_list[j]
            dist = math.hypot(ax - bx, ay - by_)
            if dist < 0.05:
                collision_pairs.append((na, nb))

    # Crossing-free fan: two radial escape lines (centroid→via_i, centroid→via_j)
    # cross if and only if the angular RANK of slot i vs j is REVERSED relative
    # to the angular rank of source pad i vs j.
    # Correct check: build a mapping net→slot_rank (by slot angle) and
    # net→pad_rank (by pad angle). Count pairs where pad_rank_i < pad_rank_j
    # but slot_rank_i > slot_rank_j (a true inversion = crossing).
    nets_with_src_angle: list[tuple[float, str, tuple[float, float]]] = []
    for net_name, via_xy in via_slots.items():
        src = assignment[net_name].get("source_xy")
        if src is None:
            continue
        src_angle = math.degrees(
            math.atan2(src[1] - component_cy, src[0] - component_cx)
        ) % 360.0
        nets_with_src_angle.append((src_angle, net_name, via_xy))
    nets_with_src_angle.sort(key=lambda t: t[0])

    # Assign slot ranks by slot angle (sort slot positions by angle).
    nets_slot_angle = sorted(
        [(math.degrees(math.atan2(vxy[1] - component_cy, vxy[0] - component_cx)) % 360.0,
          nm)
         for _, nm, vxy in nets_with_src_angle],
        key=lambda t: t[0],
    )
    slot_rank: dict[str, int] = {nm: i for i, (_, nm) in enumerate(nets_slot_angle)}

    # Count inversions: i before j in pad order (pad_rank_i < pad_rank_j) but
    # slot_rank_i > slot_rank_j — a true crossing.
    crossing_pairs: list[tuple[str, str]] = []
    names = [nm for _, nm, _ in nets_with_src_angle]
    for i in range(len(names)):
        for j in range(i + 1, len(names)):
            ni, nj = names[i], names[j]
            if slot_rank[ni] > slot_rank[nj]:
                crossing_pairs.append((ni, nj))

    run_swclk_legal = (
        "/RUN" not in illegal_nets
        and "/SWCLK" not in illegal_nets
        and "/RUN" in via_slots
        and "/SWCLK" in via_slots
    )

    via_nets_in_assignment = [
        nm for nm in assignment if nm not in fcu_only_nets
    ]

    return {
        "all_legal": len(illegal_nets) == 0,
        "all_disjoint": len(collision_pairs) == 0,
        "crossing_free_fan": len(crossing_pairs) == 0,
        "illegal_nets": illegal_nets,
        "collision_pairs": collision_pairs,
        "crossing_pairs": crossing_pairs,
        "run_swclk_legal": run_swclk_legal,
        "total_via_nets": len(via_nets_in_assignment),
        "assigned_count": len(via_slots),
    }
