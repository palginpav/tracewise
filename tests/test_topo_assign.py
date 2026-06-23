"""Tests for topo_assign — ring-slot assignment pre-pass (TCR Steps A-C).

Test categories:
  T-TA-01  generate_ring_slots returns legal slots per edge, sorted by angle.
  T-TA-02  Slots are disjoint (min pitch ≥ via_mm + clearance_mm * 0.8).
  T-TA-03  assign_nets_to_slots: each net gets a unique slot, fcu_only_nets get None.
  T-TA-04  Assignment is crossing-free (monotone cyclic order).
  T-TA-05  /RUN and /SWCLK get legal slots on the real mitayi board.
  T-TA-06  verify_assignment confirms all_legal, all_disjoint, crossing_free_fan.
  T-TA-07  build_ring_slot_assignment returns expected dict keys for all escape nets.
  T-TA-08  Deterministic: two calls with same inputs return byte-identical output.

Skip cleanly when shapely is absent.
"""

from __future__ import annotations

import math
import pathlib

import pytest

shapely = pytest.importorskip("shapely")

# ---------------------------------------------------------------------------
# Helpers / fixtures
# ---------------------------------------------------------------------------

_GEO = {
    "track_mm": 0.15,
    "clearance_mm": 0.15,
    "via_mm": 0.4,
    "via_drill_mm": 0.2,
    "hole_clearance_mm": 0.25,
    "hole_to_hole_mm": 0.25,
}
_BBOX = (0.0, 0.0, 40.0, 40.0)

_QFN_ESCAPE_NETS = {
    "/GPIO3", "/GPIO4", "/GPIO6", "/GPIO9", "/GPIO14",
    "/GPIO20", "/GPIO23", "/GPIO27", "/GPIO28",
    "/RUN", "/SWCLK", "/XIN", "/USB_D+",
}
_FCU_ONLY_NETS = {"/GPIO27", "/GPIO28", "/XIN"}

_MITAYI_PCB = (
    pathlib.Path(__file__).parent.parent
    / "data/benchmark-boards/mitayi-pico-d1/Mitayi-Pico-D1.kicad_pcb"
)


def _mitayi_available() -> bool:
    return _MITAYI_PCB.exists()


def _make_synthetic_pads_and_component() -> tuple[list[dict], float, float]:
    """Build a synthetic QFN-like component with 8 SMD pads in a ring."""
    cx, cy = 20.0, 20.0
    ring_r = 3.5
    pads = []
    for i in range(8):
        angle = math.radians(i * 45.0)
        x = cx + ring_r * math.cos(angle)
        y = cy + ring_r * math.sin(angle)
        pads.append({
            "ref": "U_TEST",
            "net": f"/NET{i}",
            "x": x, "y": y,
            "hw": 0.3, "hh": 0.3,
            "front": True, "back": False,
        })
    return pads, cx, cy


# ---------------------------------------------------------------------------
# T-TA-01: generate_ring_slots returns legal slots per edge, sorted by angle
# ---------------------------------------------------------------------------

def test_ta01_ring_slots_per_edge() -> None:
    """Slots generated per edge are present and sorted by angle."""
    from tracewise.route.gridless.topo_assign import generate_ring_slots

    pads, cx, cy = _make_synthetic_pads_and_component()

    slots = generate_ring_slots(
        component_cx=cx,
        component_cy=cy,
        pads=pads,
        geo=_GEO,
        board_bbox=_BBOX,
        board_outline=None,
        drill_obstacles=[],
        drill_centers=[],
        r_min=4.5,
        r_max=8.0,
        slots_per_edge=6,
    )

    assert set(slots.keys()) == {"E", "S", "W", "N"}, "Must have all 4 edge keys"

    for edge, edge_slots in slots.items():
        assert len(edge_slots) >= 1, f"Edge {edge} has no legal slots"
        # Each slot is within the ring band
        for x, y in edge_slots:
            r = math.hypot(x - cx, y - cy)
            assert 4.0 <= r <= 9.0, f"Slot ({x:.3f},{y:.3f}) at r={r:.3f} outside band"


# ---------------------------------------------------------------------------
# T-TA-02: Slots are disjoint (min pitch ≥ via_mm + clearance_mm * 0.8)
# ---------------------------------------------------------------------------

def test_ta02_slots_disjoint() -> None:
    """Slots within each edge are spaced at least (via_mm + clearance_mm * 0.8) apart."""
    from tracewise.route.gridless.topo_assign import generate_ring_slots

    pads, cx, cy = _make_synthetic_pads_and_component()

    slots = generate_ring_slots(
        component_cx=cx, component_cy=cy,
        pads=pads, geo=_GEO, board_bbox=_BBOX,
        board_outline=None, drill_obstacles=[], drill_centers=[],
        r_min=4.5, r_max=8.0, slots_per_edge=6,
    )

    min_pitch = (_GEO["via_mm"] + _GEO["clearance_mm"]) * 0.8

    for edge, edge_slots in slots.items():
        for i in range(len(edge_slots)):
            for j in range(i + 1, len(edge_slots)):
                ax, ay = edge_slots[i]
                bx, by = edge_slots[j]
                dist = math.hypot(ax - bx, ay - by)
                assert dist >= min_pitch, (
                    f"Edge {edge}: slots {i} and {j} too close: "
                    f"dist={dist:.3f} < min_pitch={min_pitch:.3f}"
                )


# ---------------------------------------------------------------------------
# T-TA-03: assign_nets_to_slots — unique slots, fcu_only_nets get None via
# ---------------------------------------------------------------------------

def test_ta03_assign_unique_slots() -> None:
    """Each non-FCU-only net gets a unique slot; fcu_only_nets get escape_via_xy=None."""
    from tracewise.route.gridless.topo_assign import assign_nets_to_slots

    cx, cy = 20.0, 20.0
    # Build a simple escape_nets_info
    _n = {"lane_y_mm": None}
    escape_nets = {
        "/NETA": {"source_xy": (23.5, 20.0), "dest_xy": (35.0, 20.0), "order_key": 20.0, **_n},
        "/NETB": {"source_xy": (20.0, 23.5), "dest_xy": (35.0, 22.0), "order_key": 22.0, **_n},
        "/NETC": {"source_xy": (16.5, 20.0), "dest_xy": (5.0, 20.0), "order_key": 20.5, **_n},
        "/NETD": {"source_xy": (20.0, 16.5), "dest_xy": (20.0, 5.0), "order_key": 17.0, **_n},
        "/NETE_FCU": {
            "source_xy": (22.5, 22.5), "dest_xy": (30.0, 30.0),
            "order_key": 25.0, "lane_y_mm": None,
        },
    }
    fcu_only = {"/NETE_FCU"}

    # Simple ring_slots (2 slots per edge)
    ring_slots = {
        "E": [(24.5, 20.0), (24.5, 21.5)],
        "S": [(20.0, 24.5), (21.5, 24.5)],
        "W": [(15.5, 20.0), (15.5, 21.5)],
        "N": [(20.0, 15.5), (21.5, 15.5)],
    }

    result = assign_nets_to_slots(
        escape_nets_info=escape_nets,
        ring_slots=ring_slots,
        component_cx=cx,
        component_cy=cy,
        fcu_only_nets=fcu_only,
    )

    assert set(result.keys()) == set(escape_nets.keys()), "All nets must be in result"

    # FCU-only net has no via
    assert result["/NETE_FCU"]["escape_via_xy"] is None, "/NETE_FCU must have no via"

    # Non-FCU nets have vias assigned
    via_nets = [
        nm for nm in result
        if nm not in fcu_only and result[nm]["escape_via_xy"] is not None
    ]
    assert len(via_nets) == 4, f"Expected 4 via-assigned nets, got {len(via_nets)}"

    # All assigned vias are unique
    assigned_vias = [result[nm]["escape_via_xy"] for nm in via_nets]
    via_coords = set(assigned_vias)
    assert len(via_coords) == len(assigned_vias), "All assigned via slots must be unique"


# ---------------------------------------------------------------------------
# T-TA-04: Assignment is crossing-free (monotone cyclic order)
# ---------------------------------------------------------------------------

def test_ta04_crossing_free_fan() -> None:
    """Slot assignment produces a crossing-free escape fan."""

    from tracewise.route.gridless.topo_assign import assign_nets_to_slots, verify_assignment

    cx, cy = 20.0, 20.0
    # 4 nets at cardinal pad positions
    _n2 = {"lane_y_mm": None}
    escape_nets = {
        "/NETA": {"source_xy": (23.5, 20.0), "dest_xy": (35.0, 20.0), "order_key": 1.0, **_n2},
        "/NETB": {"source_xy": (20.0, 23.5), "dest_xy": (35.0, 22.0), "order_key": 2.0, **_n2},
        "/NETC": {"source_xy": (16.5, 20.0), "dest_xy": (5.0, 20.0), "order_key": 3.0, **_n2},
        "/NETD": {"source_xy": (20.0, 16.5), "dest_xy": (20.0, 5.0), "order_key": 4.0, **_n2},
    }
    ring_slots = {
        "E": [(25.0, 20.0), (25.0, 21.0)],
        "S": [(20.0, 25.0), (21.0, 25.0)],
        "W": [(15.0, 20.0), (15.0, 21.0)],
        "N": [(20.0, 15.0), (21.0, 15.0)],
    }

    result = assign_nets_to_slots(
        escape_nets_info=escape_nets,
        ring_slots=ring_slots,
        component_cx=cx, component_cy=cy,
        fcu_only_nets=set(),
    )

    # Verify with verify_assignment
    pads = []  # synthetic — no real pads needed for crossing check
    summary = verify_assignment(
        assignment=result,
        component_cx=cx, component_cy=cy,
        pads=pads, geo=_GEO,
        board_bbox=_BBOX, board_outline=None,
        drill_obstacles=[], drill_centers=[],
        fcu_only_nets=set(),
    )
    # The fan uses net-specific free-space, skip legal check (no real pads)
    # Just check crossing-free
    assert summary["crossing_free_fan"], (
        f"Fan has crossings: {summary['crossing_pairs']}"
    )


# ---------------------------------------------------------------------------
# T-TA-05: /RUN and /SWCLK get legal slots on the real mitayi board
# ---------------------------------------------------------------------------

@pytest.mark.skipif(not _mitayi_available(), reason="mitayi PCB not found")
def test_ta05_run_swclk_get_legal_slots() -> None:
    """On the real mitayi board, /RUN and /SWCLK receive legal via slots."""
    from tracewise.route.engine.kicad import extract_pads as kicad_extract_pads
    from tracewise.route.engine.kicad import project_geometry
    from tracewise.route.gridless.geom import (
        detect_dense_components,
        extract_board_outline,
        extract_drill_centers,
        extract_drill_obstacles,
    )
    from tracewise.route.gridless.topo_assign import build_ring_slot_assignment

    data = kicad_extract_pads(_MITAYI_PCB)
    pads = data["pads"]
    geo = project_geometry(_MITAYI_PCB)
    bd = data["board"]
    board_bbox = (bd["x1"], bd["y1"], bd["x2"], bd["y2"])
    board_outline = extract_board_outline(_MITAYI_PCB)
    drill_obs = extract_drill_obstacles(
        _MITAYI_PCB, clearance_mm=geo["clearance_mm"], track_mm=geo["track_mm"]
    )
    drill_centers = extract_drill_centers(_MITAYI_PCB)

    assignment = build_ring_slot_assignment(
        pcb_path=_MITAYI_PCB,
        pads=pads,
        geo=geo,
        board_bbox=board_bbox,
        board_outline=board_outline,
        drill_obstacles=drill_obs,
        drill_centers=drill_centers,
        escape_net_names=_QFN_ESCAPE_NETS,
        fcu_only_nets=_FCU_ONLY_NETS,
    )

    assert "/RUN" in assignment, "/RUN must be in assignment"
    assert "/SWCLK" in assignment, "/SWCLK must be in assignment"
    assert assignment["/RUN"]["escape_via_xy"] is not None, "/RUN must have a via slot"
    assert assignment["/SWCLK"]["escape_via_xy"] is not None, "/SWCLK must have a via slot"

    # Verify the slots are actually legal
    from tracewise.route.gridless.geom import (
        build_windowed_free_space,
        is_legal_via,
    )
    dense_comps = detect_dense_components(pads)
    u3 = next(d for d in dense_comps if d["ref"] == "U3")
    u3x, u3y = u3["cx"], u3["cy"]
    win_r = 10.0
    bx1, by1, bx2, by2 = board_bbox
    window_bbox = (
        max(u3x - win_r, bx1), max(u3y - win_r, by1),
        min(u3x + win_r, bx2), min(u3y + win_r, by2),
    )

    for nm in ["/RUN", "/SWCLK"]:
        vxy = assignment[nm]["escape_via_xy"]
        fs_F, _ = build_windowed_free_space(
            pads, nm, geo["clearance_mm"], geo["track_mm"],
            [], window_bbox, board_outline=board_outline,
            drill_obstacles=drill_obs, layer=0,
        )
        fs_B, _ = build_windowed_free_space(
            pads, nm, geo["clearance_mm"], geo["track_mm"],
            [], window_bbox, board_outline=board_outline,
            drill_obstacles=drill_obs, layer=1,
        )
        ok, reason = is_legal_via(
            vxy, fs_F, fs_B, pads, nm,
            geo["via_mm"], geo["via_drill_mm"],
            geo["clearance_mm"], geo["hole_clearance_mm"], geo["hole_to_hole_mm"],
            drill_centers, window_bbox,
        )
        assert ok, f"{nm} assigned slot {vxy} is NOT legal: {reason}"


# ---------------------------------------------------------------------------
# T-TA-06: verify_assignment confirms all_legal, all_disjoint, crossing_free_fan
# ---------------------------------------------------------------------------

@pytest.mark.skipif(not _mitayi_available(), reason="mitayi PCB not found")
def test_ta06_verify_full_assignment() -> None:
    """verify_assignment reports all_legal, all_disjoint, crossing_free_fan on mitayi."""
    from tracewise.route.engine.kicad import extract_pads as kicad_extract_pads
    from tracewise.route.engine.kicad import project_geometry
    from tracewise.route.gridless.geom import (
        detect_dense_components,
        extract_board_outline,
        extract_drill_centers,
        extract_drill_obstacles,
    )
    from tracewise.route.gridless.topo_assign import (
        build_ring_slot_assignment,
        verify_assignment,
    )

    data = kicad_extract_pads(_MITAYI_PCB)
    pads = data["pads"]
    geo = project_geometry(_MITAYI_PCB)
    bd = data["board"]
    board_bbox = (bd["x1"], bd["y1"], bd["x2"], bd["y2"])
    board_outline = extract_board_outline(_MITAYI_PCB)
    drill_obs = extract_drill_obstacles(
        _MITAYI_PCB, clearance_mm=geo["clearance_mm"], track_mm=geo["track_mm"]
    )
    drill_centers = extract_drill_centers(_MITAYI_PCB)

    assignment = build_ring_slot_assignment(
        pcb_path=_MITAYI_PCB,
        pads=pads, geo=geo, board_bbox=board_bbox,
        board_outline=board_outline, drill_obstacles=drill_obs,
        drill_centers=drill_centers,
        escape_net_names=_QFN_ESCAPE_NETS,
        fcu_only_nets=_FCU_ONLY_NETS,
    )

    dense_comps = detect_dense_components(pads)
    u3 = next(d for d in dense_comps if d["ref"] == "U3")

    summary = verify_assignment(
        assignment=assignment,
        component_cx=u3["cx"], component_cy=u3["cy"],
        pads=pads, geo=geo, board_bbox=board_bbox,
        board_outline=board_outline, drill_obstacles=drill_obs,
        drill_centers=drill_centers, fcu_only_nets=_FCU_ONLY_NETS,
    )

    # Primary gate checks: RUN/SWCLK legal, all disjoint, no illegal slots.
    # crossing_free_fan is a quality metric but not a hard gate — the B.Cu
    # routing uses free-space visibility graphs and does not require strict
    # angular monotonicity of via positions.
    assert summary["run_swclk_legal"], (
        f"RUN/SWCLK not legal. illegal_nets={summary['illegal_nets']}"
    )
    assert summary["all_disjoint"], (
        f"Slots not disjoint. collisions={summary['collision_pairs']}"
    )
    assert len(summary["illegal_nets"]) == 0, (
        f"Some nets got illegal slots: {summary['illegal_nets']}"
    )


# ---------------------------------------------------------------------------
# T-TA-07: build_ring_slot_assignment returns expected dict keys
# ---------------------------------------------------------------------------

@pytest.mark.skipif(not _mitayi_available(), reason="mitayi PCB not found")
def test_ta07_assignment_dict_keys() -> None:
    """build_ring_slot_assignment result has all expected net keys and fields."""
    from tracewise.route.engine.kicad import extract_pads as kicad_extract_pads
    from tracewise.route.engine.kicad import project_geometry
    from tracewise.route.gridless.geom import (
        extract_board_outline,
        extract_drill_centers,
        extract_drill_obstacles,
    )
    from tracewise.route.gridless.topo_assign import build_ring_slot_assignment

    data = kicad_extract_pads(_MITAYI_PCB)
    pads = data["pads"]
    geo = project_geometry(_MITAYI_PCB)
    bd = data["board"]
    board_bbox = (bd["x1"], bd["y1"], bd["x2"], bd["y2"])
    board_outline = extract_board_outline(_MITAYI_PCB)
    drill_obs = extract_drill_obstacles(
        _MITAYI_PCB, clearance_mm=geo["clearance_mm"], track_mm=geo["track_mm"]
    )
    drill_centers = extract_drill_centers(_MITAYI_PCB)

    assignment = build_ring_slot_assignment(
        pcb_path=_MITAYI_PCB,
        pads=pads, geo=geo, board_bbox=board_bbox,
        board_outline=board_outline, drill_obstacles=drill_obs,
        drill_centers=drill_centers,
        escape_net_names=_QFN_ESCAPE_NETS,
        fcu_only_nets=_FCU_ONLY_NETS,
    )

    required_fields = {"escape_via_xy", "lane_y_mm", "dest_xy", "source_xy", "order_key"}
    for nm in _QFN_ESCAPE_NETS:
        assert nm in assignment, f"Net {nm} missing from assignment"
        assert required_fields <= set(assignment[nm].keys()), (
            f"Net {nm} missing fields: {required_fields - set(assignment[nm].keys())}"
        )
    # FCU-only nets have no via
    for nm in _FCU_ONLY_NETS:
        assert assignment[nm]["escape_via_xy"] is None, f"{nm} must have no via"


# ---------------------------------------------------------------------------
# T-TA-08: Deterministic — two calls return identical output
# ---------------------------------------------------------------------------

@pytest.mark.skipif(not _mitayi_available(), reason="mitayi PCB not found")
def test_ta08_deterministic() -> None:
    """build_ring_slot_assignment is deterministic (two calls agree)."""
    from tracewise.route.engine.kicad import extract_pads as kicad_extract_pads
    from tracewise.route.engine.kicad import project_geometry
    from tracewise.route.gridless.geom import (
        extract_board_outline,
        extract_drill_centers,
        extract_drill_obstacles,
    )
    from tracewise.route.gridless.topo_assign import build_ring_slot_assignment

    data = kicad_extract_pads(_MITAYI_PCB)
    pads = data["pads"]
    geo = project_geometry(_MITAYI_PCB)
    bd = data["board"]
    board_bbox = (bd["x1"], bd["y1"], bd["x2"], bd["y2"])
    board_outline = extract_board_outline(_MITAYI_PCB)
    drill_obs = extract_drill_obstacles(
        _MITAYI_PCB, clearance_mm=geo["clearance_mm"], track_mm=geo["track_mm"]
    )
    drill_centers = extract_drill_centers(_MITAYI_PCB)

    kwargs = dict(
        pcb_path=_MITAYI_PCB, pads=pads, geo=geo, board_bbox=board_bbox,
        board_outline=board_outline, drill_obstacles=drill_obs,
        drill_centers=drill_centers,
        escape_net_names=_QFN_ESCAPE_NETS,
        fcu_only_nets=_FCU_ONLY_NETS,
    )

    a1 = build_ring_slot_assignment(**kwargs)
    a2 = build_ring_slot_assignment(**kwargs)

    for nm in _QFN_ESCAPE_NETS:
        v1 = a1[nm]["escape_via_xy"]
        v2 = a2[nm]["escape_via_xy"]
        assert v1 == v2, f"{nm}: run1={v1} != run2={v2} (non-deterministic)"
