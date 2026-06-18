"""Phase-C probe: quantify how many residual via-involved clearance DRC errors
are fixable by a hole-clearance-aware via nudge — BEFORE building the nudger.

This probe answers the "ceiling" question: of the ~56 via-involved residual
violations in the gated-routed board (mitayi, 88 errors), how many could a
*correct* (hole-aware, connectivity-preserving, bounded) nudge actually fix?

Usage:
    taskset -c 0-9 .venv/bin/python scripts/probe_via_nudge_holeaware.py \\
        /tmp/probe_gated_1/Mitayi-Pico-D1.kicad_pcb \\
        /tmp/probe_gated_1/Mitayi-Pico-D1.drc.json

Requirements: Python 3.10+, numpy only (no Shapely). Analysis only — does NOT
modify any board files.

Probe design:
  - 16 angles × 6 radii = 96 polar candidates per via position (plus origin)
  - Nudge max radius R = 0.3 mm
  - Connectivity constraint: nudged via stays within R of its current center,
    so all attached track endpoints that were within R+track_half_width are
    still reachable (same bounded-nudge assumption as the emitter).
  - Legality: candidate position passes ALL of:
      (a) via COPPER (r_via) to other-net copper (pads, other-net tracks,
          other-net vias): gap >= copper_clearance (~0.15 mm)
      (b) via DRILL (r_drill) to every OTHER hole (pad drill circles + other
          via drills, regardless of net): center-to-center - r_drill - r_other
          >= hole_clearance (~0.25 mm)
  - A position that fixes THIS violation but creates a new one is rejected
    (legality already checks all nearby obstacles, not just the triggering pair).
"""

from __future__ import annotations

import json
import math
import re
import sys
from pathlib import Path
from typing import NamedTuple

import numpy as np

# ---------------------------------------------------------------------------
# Polar search grid constants (fixed, documented — deterministic)
# ---------------------------------------------------------------------------

NUDGE_RADIUS = 0.3          # mm — max nudge distance
NUDGE_ANGLES = 16           # uniformly spaced in [0, 2pi)
_RADIUS_FRACTIONS = (1/6, 2/6, 3/6, 4/6, 5/6, 1.0)   # 6 radii
NEARBY_WINDOW = 1.0         # mm — obstacle collection window

POLAR_ANGLES: tuple[float, ...] = tuple(
    2.0 * math.pi * k / NUDGE_ANGLES for k in range(NUDGE_ANGLES)
)
POLAR_RADII: tuple[float, ...] = tuple(
    NUDGE_RADIUS * f for f in _RADIUS_FRACTIONS
)

# ---------------------------------------------------------------------------
# Geometry helpers (copied from probe_clearance_nudgeability.py for
# self-containment; same logic as in exact_geom.py)
# ---------------------------------------------------------------------------

def seg_point_dist(ax, ay, bx, by, px, py) -> float:
    """Distance from point P to segment AB."""
    dx, dy = bx - ax, by - ay
    len_sq = dx * dx + dy * dy
    if len_sq < 1e-18:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / len_sq))
    qx, qy = ax + t * dx, ay + t * dy
    return math.hypot(px - qx, py - qy)


def dist_point_to_rect(px, py, rx, ry, rhw, rhh) -> float:
    """Distance from point (px,py) to axis-aligned rect centered at (rx,ry)."""
    dx = max(0.0, abs(px - rx) - rhw)
    dy = max(0.0, abs(py - ry) - rhh)
    return math.hypot(dx, dy)


# ---------------------------------------------------------------------------
# Board data structures
# ---------------------------------------------------------------------------

class Via(NamedTuple):
    x: float; y: float
    copper_r: float    # size/2
    drill_r: float     # drill/2
    net: str
    uuid: str


class TrackSeg(NamedTuple):
    x1: float; y1: float; x2: float; y2: float
    half_width: float; layer: str; net: str; uuid: str


class DrillHole(NamedTuple):
    """A drilled hole — pad through-hole or via drill (net-agnostic for hole_clearance)."""
    x: float; y: float; radius: float
    source: str   # 'pad' or 'via'
    uuid: str     # for exclusion of self


class PadCopper(NamedTuple):
    """Pad copper rectangle for other-net clearance check."""
    x: float; y: float; hw: float; hh: float
    net: str; uuid: str   # uuid = footprint+pad idx combo


# ---------------------------------------------------------------------------
# Parsing helpers
# ---------------------------------------------------------------------------

def _nums(s: str) -> list[float]:
    return [float(x) for x in re.findall(r'-?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?', s)]


def parse_board(board_path: str):
    """Parse vias, tracks, pad-copper rectangles, and drill holes from .kicad_pcb.

    Returns (vias, tracks, pad_coppers, drill_holes).

    Approximations (documented):
    1. Pad copper treated as axis-aligned rectangle from (size w h) — ignores
       per-pad rotation.  Overapproximation; may slightly undercount clearance.
    2. Via copper radius = size/2.  Via drill radius = drill/2.
    3. Pad drill: circle of radius = drill/2 (assumes round drill; oval drill
       uses larger dimension/2 as conservative approximation).
    4. Footprint rotation applied to pad center, but NOT to pad copper
       orientation (see approximation 1).
    """
    text = Path(board_path).read_text(encoding='utf-8')

    # ------------------------------------------------------------------
    # Vias
    # ------------------------------------------------------------------
    vias: list[Via] = []
    VIA_RE = re.compile(
        r'\(via\s*'
        r'\(at\s+(-?[\d.]+)\s+(-?[\d.]+)\)\s*'
        r'\(size\s+(-?[\d.]+)\)\s*'
        r'\(drill\s+(-?[\d.]+)\)\s*'
        r'\(layers\s+"[^"]+"\s+"[^"]+"\)\s*'
        r'\(net\s+"?([^")\s]+)"?\)\s*'
        r'\(uuid\s+"([^"]+)"\)',
        re.DOTALL,
    )
    for m in VIA_RE.finditer(text):
        x, y, size, drill = float(m.group(1)), float(m.group(2)), float(m.group(3)), float(m.group(4))
        vias.append(Via(x, y, size / 2.0, drill / 2.0, m.group(5), m.group(6)))

    # ------------------------------------------------------------------
    # Tracks
    # ------------------------------------------------------------------
    tracks: list[TrackSeg] = []
    SEG_RE = re.compile(
        r'\(segment\s*'
        r'\(start\s+(-?[\d.]+)\s+(-?[\d.]+)\)\s*'
        r'\(end\s+(-?[\d.]+)\s+(-?[\d.]+)\)\s*'
        r'\(width\s+(-?[\d.]+)\)\s*'
        r'\(layer\s+"([^"]+)"\)\s*'
        r'\(net\s+"?([^")\s]+)"?\)\s*'
        r'\(uuid\s+"([^"]+)"\)',
        re.DOTALL,
    )
    for m in SEG_RE.finditer(text):
        tracks.append(TrackSeg(
            float(m.group(1)), float(m.group(2)),
            float(m.group(3)), float(m.group(4)),
            float(m.group(5)) / 2.0,
            m.group(6), m.group(7), m.group(8),
        ))

    # ------------------------------------------------------------------
    # Pads: footprint-relative positions + footprint (at) transform
    # ------------------------------------------------------------------
    pad_coppers: list[PadCopper] = []
    drill_holes: list[DrillHole] = []

    fp_starts = [m.start() for m in re.finditer(r'^\t\(footprint\s', text, re.MULTILINE)]
    fp_starts.append(len(text))

    FP_AT_RE = re.compile(
        r'^\s{2}\(at\s+(-?[\d.]+)\s+(-?[\d.]+)(?:\s+(-?[\d.]+))?\)',
        re.MULTILINE,
    )
    # Matches: (pad "N" <type> <shape>
    PAD_HDR_RE = re.compile(
        r'\(pad\s+"?[^"\s)]*"?\s+(\w+)\s+(\w+)\s*'  # type shape
        r'(?:.*?)\(at\s+(-?[\d.]+)\s+(-?[\d.]+)(?:\s+(-?[\d.]+))?\)\s*'
        r'\(size\s+(-?[\d.]+)\s+(-?[\d.]+)\)',
        re.DOTALL,
    )
    PAD_NET_RE = re.compile(r'\(net\s+"?([^")\s]+)"?\)')
    # Drill: (drill [oval] d [w h]) — first number after 'drill' is key
    PAD_DRILL_RE = re.compile(r'\(drill(?:\s+oval)?\s+([\d.]+)(?:\s+([\d.]+))?')
    PAD_LAYERS_RE = re.compile(r'\(layers\s+"([^"]+)"')

    for fi in range(len(fp_starts) - 1):
        fp_text = text[fp_starts[fi]:fp_starts[fi + 1]]
        at_m = FP_AT_RE.search(fp_text)
        if not at_m:
            continue
        fx, fy = float(at_m.group(1)), float(at_m.group(2))
        fp_rot_deg = float(at_m.group(3)) if at_m.group(3) else 0.0
        fp_rot = math.radians(fp_rot_deg)
        cos_r, sin_r = math.cos(fp_rot), math.sin(fp_rot)

        pad_idx = 0
        for pm in PAD_HDR_RE.finditer(fp_text):
            pad_type = pm.group(1)      # smd / thru_hole / np_thru_hole
            # pad_shape = pm.group(2)  # circle / rect / roundrect / oval …
            plx = float(pm.group(3))
            ply = float(pm.group(4))
            pw = float(pm.group(6))
            ph = float(pm.group(7))

            # Absolute center (apply footprint rotation + translation)
            pax = fx + cos_r * plx - sin_r * ply
            pay = fy + sin_r * plx + cos_r * ply

            # Extract net
            pad_block = fp_text[pm.start():pm.start() + 2000]
            net_m = PAD_NET_RE.search(pad_block)
            pad_net = net_m.group(1) if net_m else ''

            # Extract layer(s) for copper
            layers_m = PAD_LAYERS_RE.search(pad_block)
            layer_str = layers_m.group(1) if layers_m else ''

            uid = f"fp{fi}_p{pad_idx}"
            pad_idx += 1

            # SMD pads: copper on front or back only
            if pad_type == 'smd':
                pad_coppers.append(PadCopper(pax, pay, pw / 2.0, ph / 2.0, pad_net, uid))

            # Through-hole pads: copper on all layers + drill hole
            elif pad_type in ('thru_hole', 'np_thru_hole'):
                pad_coppers.append(PadCopper(pax, pay, pw / 2.0, ph / 2.0, pad_net, uid))
                drill_m = PAD_DRILL_RE.search(pad_block)
                if drill_m:
                    d1 = float(drill_m.group(1))
                    d2 = float(drill_m.group(2)) if drill_m.group(2) else d1
                    # Use max dim / 2 as conservative drill radius for oval drills
                    drill_r = max(d1, d2) / 2.0
                    drill_holes.append(DrillHole(pax, pay, drill_r, 'pad', uid))

    # Via drill holes (net-agnostic for hole_clearance)
    for v in vias:
        drill_holes.append(DrillHole(v.x, v.y, v.drill_r, 'via', v.uuid))

    return vias, tracks, pad_coppers, drill_holes


# ---------------------------------------------------------------------------
# DRC JSON parsing
# ---------------------------------------------------------------------------

def parse_via_violations(drc_path: str):
    """Return list of clearance/hole_clearance violations that involve a via."""
    with open(drc_path, encoding='utf-8') as f:
        data = json.load(f)

    result = []
    for v in data['violations']:
        if v['type'] not in ('clearance', 'hole_clearance'):
            continue
        if v['severity'] != 'error':
            continue
        items = v['items']
        item_types = [_classify_item(it['description']) for it in items]
        if 'via' in item_types:
            result.append(v)
    return result


def _classify_item(desc: str) -> str:
    d = desc.strip()
    if d.startswith('Перех.'):
        return 'via'
    if d.startswith('Дорожка') or d.startswith('Сегмент') or d.startswith('След'):
        return 'track'
    if (d.startswith('Конт. пл.') or d.startswith('PTH конт') or
            d.startswith('NPTH конт') or d.startswith('PTH') or d.startswith('NPTH')):
        return 'pad'
    if d.startswith('Зона') or d.startswith('Заливка') or d.startswith('Полиг'):
        return 'zone'
    return 'unknown'


def _parse_required_gap(description: str) -> float:
    """Extract the required clearance value from a violation description."""
    # Russian: "зазор X,XXXX mm" (hole_clearance format same structure)
    m = re.search(r'зазор\s+([\d,\.]+)\s*mm', description)
    if m:
        return float(m.group(1).replace(',', '.'))
    m = re.search(r'clearance\s+([\d.]+)\s*mm', description)
    if m:
        return float(m.group(1))
    return 0.25  # default fallback


def _classify_via_viol_subtype(viol: dict) -> str:
    """Return sub-type string: 'clearance_pad_via', 'clearance_track_via',
    'hole_clearance_pad_via', or 'other'."""
    vtype = viol['type']
    items = viol['items']
    item_types = [_classify_item(it['description']) for it in items]
    type_set = set(item_types)
    if 'via' not in type_set:
        return 'other'
    if vtype == 'clearance':
        if 'pad' in type_set:
            return 'clearance_pad_via'
        if 'track' in type_set:
            return 'clearance_track_via'
        return 'clearance_via_via'
    if vtype == 'hole_clearance':
        if 'pad' in type_set:
            return 'hole_clearance_pad_via'
        return 'hole_clearance_via_via'
    return 'other'


# ---------------------------------------------------------------------------
# Via-position lookup
# ---------------------------------------------------------------------------

def find_via(vias: list[Via], px: float, py: float, radius: float = 0.5):
    """Return the nearest via to (px, py) within *radius* mm, or None."""
    best_d = float('inf')
    best_v = None
    for v in vias:
        d = math.hypot(v.x - px, v.y - py)
        if d < best_d:
            best_d = d
            best_v = v
    return best_v if best_d <= radius else None


# ---------------------------------------------------------------------------
# Obstacle collection for the nudge search
# ---------------------------------------------------------------------------

def collect_obstacles_for_via(
    vx: float, vy: float,
    vias: list[Via],
    tracks: list[TrackSeg],
    pad_coppers: list[PadCopper],
    drill_holes: list[DrillHole],
    via: Via,
    window: float = NEARBY_WINDOW,
):
    """Collect all nearby copper obstacles (for copper clearance) and drill
    holes (for hole clearance) around position (vx, vy).

    Returns (copper_other_net, all_drill_holes) where:
      copper_other_net: list of (kind, *params) obstacles OTHER-NET only
      all_drill_holes:  list of DrillHole (all nets, excluding via's own hole)
    """
    copper_other = []    # (kind, ...) — for copper clearance test
    holes = []           # DrillHole — for hole clearance test

    via_net = via.net
    via_uuid = via.uuid

    # Other vias
    for ov in vias:
        if ov.uuid == via_uuid:
            continue
        d = math.hypot(ov.x - vx, ov.y - vy)
        if d > window + ov.copper_r:
            continue
        if ov.net != via_net:
            copper_other.append(('circle', ov.x, ov.y, ov.copper_r, ov.net))
        # Drill holes: ALL vias regardless of net
        holes.append(DrillHole(ov.x, ov.y, ov.drill_r, 'via', ov.uuid))

    # Tracks (other net only for copper clearance)
    for seg in tracks:
        if seg.net == via_net:
            continue
        d = seg_point_dist(seg.x1, seg.y1, seg.x2, seg.y2, vx, vy)
        if d > window + seg.half_width:
            continue
        copper_other.append(('segment', seg.x1, seg.y1, seg.x2, seg.y2, seg.half_width, seg.net))

    # Pad coppers (other net only) and pad drill holes (ALL nets)
    for pc in pad_coppers:
        dbox = dist_point_to_rect(vx, vy, pc.x, pc.y, pc.hw, pc.hh)
        if dbox > window:
            continue
        if pc.net != via_net:
            copper_other.append(('rect', pc.x, pc.y, pc.hw, pc.hh, pc.net))

    # Pad drill holes (already pre-computed, just filter by window)
    for dh in drill_holes:
        if dh.source == 'via' and dh.uuid == via_uuid:
            continue  # self
        d = math.hypot(dh.x - vx, dh.y - vy)
        if d <= window + dh.radius + 0.3:  # generous window for hole check
            holes.append(dh)

    return copper_other, holes


# ---------------------------------------------------------------------------
# Legality check at a candidate via position
# ---------------------------------------------------------------------------

def _copper_gap_to_via(qx, qy, via_copper_r: float, obs) -> float:
    """Edge-to-edge copper clearance from via at (qx,qy) to obstacle obs.
    Negative = overlap."""
    kind = obs[0]
    if kind == 'circle':
        _, cx, cy, r, _net = obs
        return math.hypot(qx - cx, qy - cy) - via_copper_r - r
    elif kind == 'rect':
        _, cx, cy, hw, hh, _net = obs
        return dist_point_to_rect(qx, qy, cx, cy, hw, hh) - via_copper_r
    elif kind == 'segment':
        _, ax, ay, bx, by, hw, _net = obs
        return seg_point_dist(ax, ay, bx, by, qx, qy) - via_copper_r - hw
    return float('inf')


def _hole_gap(qx, qy, via_drill_r: float, dh: DrillHole) -> float:
    """Center-to-center minus both radii (edge-to-edge for hole clearance).
    Negative = overlap."""
    return math.hypot(qx - dh.x, qy - dh.y) - via_drill_r - dh.radius


def is_position_legal(
    qx: float, qy: float,
    via: Via,
    copper_obs: list,
    drill_obs: list[DrillHole],
    copper_clearance: float,
    hole_clearance: float,
) -> bool:
    """Return True iff position (qx,qy) satisfies ALL copper + hole constraints."""
    # Copper: via copper to all other-net copper obstacles
    for obs in copper_obs:
        gap = _copper_gap_to_via(qx, qy, via.copper_r, obs)
        if gap < copper_clearance - 1e-6:
            return False

    # Hole: via drill to every other hole (net-agnostic)
    for dh in drill_obs:
        gap = _hole_gap(qx, qy, via.drill_r, dh)
        if gap < hole_clearance - 1e-6:
            return False

    return True


# ---------------------------------------------------------------------------
# Nudgeability test (polar grid search)
# ---------------------------------------------------------------------------

def can_nudge_via(
    via: Via,
    copper_obs: list,
    drill_obs: list[DrillHole],
    copper_clearance: float,
    hole_clearance: float,
) -> tuple[bool, str]:
    """Test whether a legal position exists within NUDGE_RADIUS of the via.

    Returns (nudgeable: bool, reason: str).
    reason is 'already_legal' | 'found_at_angle_r' | 'boxed_in'.
    """
    # Check original position first
    if is_position_legal(via.x, via.y, via, copper_obs, drill_obs,
                         copper_clearance, hole_clearance):
        return True, 'already_legal'

    # Polar grid search
    for r in POLAR_RADII:
        for angle in POLAR_ANGLES:
            qx = via.x + r * math.cos(angle)
            qy = via.y + r * math.sin(angle)
            if is_position_legal(qx, qy, via, copper_obs, drill_obs,
                                 copper_clearance, hole_clearance):
                return True, f'found_at_r={r:.3f}'

    return False, 'boxed_in'


# ---------------------------------------------------------------------------
# Main classification loop
# ---------------------------------------------------------------------------

COPPER_CLEARANCE_DEFAULT = 0.15
HOLE_CLEARANCE_DEFAULT   = 0.25


def classify_via_violation(
    viol: dict,
    vias: list[Via],
    tracks: list[TrackSeg],
    pad_coppers: list[PadCopper],
    drill_holes: list[DrillHole],
) -> dict:
    """Classify a single via-involved violation.

    Returns dict with keys:
      subtype, via_uuid, via_pos, bucket, reason, copper_clearance, hole_clearance
    buckets: 'via_nudgeable' | 'boxed_in' | 'not_via'
    """
    subtype = _classify_via_viol_subtype(viol)
    items = viol['items']
    description = viol.get('description', '')

    # Parse required clearance values
    required_gap = _parse_required_gap(description)
    if viol['type'] == 'clearance':
        copper_clr = required_gap
        hole_clr = HOLE_CLEARANCE_DEFAULT
    else:  # hole_clearance
        copper_clr = COPPER_CLEARANCE_DEFAULT
        hole_clr = required_gap

    # Find which item is the via
    via_item = None
    for it in items:
        if _classify_item(it['description']) == 'via':
            via_item = it
            break

    if via_item is None:
        return {'subtype': subtype, 'via_uuid': None, 'via_pos': None,
                'bucket': 'not_via', 'reason': 'no_via_item_found',
                'copper_clearance': copper_clr, 'hole_clearance': hole_clr}

    vp = via_item['pos']
    via = find_via(vias, vp['x'], vp['y'], radius=0.5)
    if via is None:
        return {'subtype': subtype, 'via_uuid': None, 'via_pos': (vp['x'], vp['y']),
                'bucket': 'not_via', 'reason': 'via_not_found_in_board',
                'copper_clearance': copper_clr, 'hole_clearance': hole_clr}

    # Collect obstacles around the via's CURRENT position
    copper_obs, drill_obs = collect_obstacles_for_via(
        via.x, via.y, vias, tracks, pad_coppers, drill_holes, via, window=NEARBY_WINDOW
    )

    nudgeable, reason = can_nudge_via(via, copper_obs, drill_obs, copper_clr, hole_clr)

    bucket = 'via_nudgeable' if nudgeable else 'boxed_in'
    return {
        'subtype': subtype,
        'via_uuid': via.uuid,
        'via_pos': (via.x, via.y),
        'bucket': bucket,
        'reason': reason,
        'copper_clearance': copper_clr,
        'hole_clearance': hole_clr,
    }


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    if len(sys.argv) < 3:
        print("Usage: probe_via_nudge_holeaware.py <board.kicad_pcb> <drc.json>")
        sys.exit(1)

    board_path = sys.argv[1]
    drc_path   = sys.argv[2]

    print("Phase-C probe: hole-clearance-aware via nudge ceiling")
    print(f"  Board: {board_path}")
    print(f"  DRC:   {drc_path}")
    print()

    # -----------------------------------------------------------------------
    # Parse board
    # -----------------------------------------------------------------------
    print("Parsing board geometry...")
    vias, tracks, pad_coppers, drill_holes = parse_board(board_path)
    print(f"  Vias:        {len(vias)}")
    print(f"  Tracks:      {len(tracks)}")
    print(f"  Pad coppers: {len(pad_coppers)}")
    print(f"  Drill holes: {len(drill_holes)}  "
          f"(pads={sum(1 for d in drill_holes if d.source=='pad')}, "
          f"vias={sum(1 for d in drill_holes if d.source=='via')})")
    print()

    # -----------------------------------------------------------------------
    # Parse via-involved DRC violations
    # -----------------------------------------------------------------------
    print("Parsing via-involved DRC violations...")
    viols = parse_via_violations(drc_path)
    print(f"  Via-involved clearance/hole_clearance errors: {len(viols)}")
    subtype_counts: dict[str, int] = {}
    for v in viols:
        st = _classify_via_viol_subtype(v)
        subtype_counts[st] = subtype_counts.get(st, 0) + 1
    for st, n in sorted(subtype_counts.items()):
        print(f"    {st}: {n}")
    print()

    # -----------------------------------------------------------------------
    # Classify each violation
    # -----------------------------------------------------------------------
    print("Classifying violations (deterministic order by description)...")
    sorted_viols = sorted(viols, key=lambda v: (v['type'], v.get('description', '')))

    results = []
    for v in sorted_viols:
        r = classify_via_violation(v, vias, tracks, pad_coppers, drill_holes)
        r['viol_type'] = v['type']
        results.append(r)

    # -----------------------------------------------------------------------
    # Tally buckets
    # -----------------------------------------------------------------------
    subtypes_of_interest = [
        'clearance_pad_via',
        'clearance_track_via',
        'hole_clearance_pad_via',
    ]

    def tally(bucket: str, subtype: str | None = None) -> int:
        return sum(
            1 for r in results
            if r['bucket'] == bucket
            and (subtype is None or r['subtype'] == subtype)
        )

    print()
    print("=" * 72)
    print("CLASSIFICATION RESULTS")
    print("=" * 72)
    print()
    hdr = f"{'Sub-type':<30} {'via_nudgeable':>14} {'boxed_in':>10} {'not_via':>8} {'TOTAL':>7}"
    print(hdr)
    print("-" * 72)

    grand_nudgeable = 0
    grand_boxed = 0
    grand_not_via = 0
    grand_total = 0

    for st in subtypes_of_interest:
        n_nudge = tally('via_nudgeable', st)
        n_box   = tally('boxed_in', st)
        n_nv    = tally('not_via', st)
        n_tot   = n_nudge + n_box + n_nv
        grand_nudgeable += n_nudge; grand_boxed += n_box
        grand_not_via   += n_nv;    grand_total  += n_tot
        print(f"  {st:<28} {n_nudge:>14} {n_box:>10} {n_nv:>8} {n_tot:>7}")

    # Any other subtypes
    other_results = [r for r in results if r['subtype'] not in subtypes_of_interest]
    if other_results:
        n_nudge = sum(1 for r in other_results if r['bucket'] == 'via_nudgeable')
        n_box   = sum(1 for r in other_results if r['bucket'] == 'boxed_in')
        n_nv    = sum(1 for r in other_results if r['bucket'] == 'not_via')
        n_tot   = len(other_results)
        grand_nudgeable += n_nudge; grand_boxed += n_box
        grand_not_via   += n_nv;    grand_total  += n_tot
        print(f"  {'other':<28} {n_nudge:>14} {n_box:>10} {n_nv:>8} {n_tot:>7}")

    print("-" * 72)
    print(f"  {'TOTAL':<28} {grand_nudgeable:>14} {grand_boxed:>10} {grand_not_via:>8} {grand_total:>7}")
    print()

    # -----------------------------------------------------------------------
    # Unique vias involved (fixing one via may resolve multiple violations)
    # -----------------------------------------------------------------------
    via_buckets: dict[str, set] = {'via_nudgeable': set(), 'boxed_in': set(), 'not_via': set()}
    for r in results:
        if r['via_uuid']:
            via_buckets[r['bucket']].add(r['via_uuid'])
        else:
            via_buckets[r['bucket']].add(r['via_pos'])

    n_unique_nudgeable = len(via_buckets['via_nudgeable'])
    n_unique_boxed     = len(via_buckets['boxed_in'])
    n_unique_all = len(set(r['via_uuid'] or r['via_pos'] for r in results))

    print(f"Unique vias involved:")
    print(f"  Total unique:          {n_unique_all}")
    print(f"  At least 1 nudgeable:  {n_unique_nudgeable}")
    print(f"  All violations boxed:  {n_unique_boxed}")
    print()

    # -----------------------------------------------------------------------
    # Ceiling estimate
    # -----------------------------------------------------------------------
    baseline_errors = 88   # mitayi after gated routing
    via_involved_total = grand_total
    implied_fixable = grand_nudgeable
    implied_after = baseline_errors - implied_fixable

    # Note: nudgeable here already passes full multi-constraint test
    print(f"Ceiling estimate (full multi-constraint legality check):")
    print(f"  Baseline errors (gated board):  {baseline_errors}")
    print(f"  Via-involved violations total:  {via_involved_total}")
    print(f"  Via-nudgeable (full test):       {implied_fixable}")
    print(f"  Boxed-in:                        {grand_boxed}")
    print(f"  Not-via (mis-match/fixed):       {grand_not_via}")
    print(f"  Implied ceiling: {baseline_errors} → ~{implied_after} errors")
    print(f"  Fraction fixable by via nudge:   "
          f"{implied_fixable}/{via_involved_total} = "
          f"{implied_fixable/max(via_involved_total,1)*100:.0f}%")
    print()

    # Verdict
    fraction = implied_fixable / max(via_involved_total, 1)
    if fraction >= 0.5:
        verdict = "HIGH-VALUE"
        verdict_reason = f"{implied_fixable}/{via_involved_total} ({fraction*100:.0f}%) are nudgeable — a hole-aware via nudge is worth building."
    else:
        verdict = "LOW-VALUE"
        verdict_reason = f"Only {implied_fixable}/{via_involved_total} ({fraction*100:.0f}%) are nudgeable — vias are congested; defer to gridless FAR build."
    print(f"VERDICT: {verdict}")
    print(f"  {verdict_reason}")
    print()

    # -----------------------------------------------------------------------
    # Per-violation detail
    # -----------------------------------------------------------------------
    print("Per-violation detail (via_nudgeable rows):")
    for r in results:
        if r['bucket'] == 'via_nudgeable':
            pos_str = f"({r['via_pos'][0]:.3f}, {r['via_pos'][1]:.3f})" if r['via_pos'] else "?"
            print(f"  [{r['subtype']}] via@{pos_str}  → {r['reason']}")
    print()
    print("Per-violation detail (boxed_in rows):")
    for r in results:
        if r['bucket'] == 'boxed_in':
            pos_str = f"({r['via_pos'][0]:.3f}, {r['via_pos'][1]:.3f})" if r['via_pos'] else "?"
            print(f"  [{r['subtype']}] via@{pos_str}  → {r['reason']}")
    print()

    # -----------------------------------------------------------------------
    # Approximation notes
    # -----------------------------------------------------------------------
    print("Approximations and caveats:")
    print("  1. Pad copper: approximated as axis-aligned rect from (size w h).")
    print("     Rotated footprints may have slightly wrong bounding-box orientation.")
    print("  2. Oval pad drills: conservative — uses max(d1, d2)/2 as radius.")
    print("  3. Connectivity: via stays within R=0.3 mm of current position,")
    print("     so all attached tracks are implicitly still reachable (same")
    print("     bounded-nudge assumption as the emitter).")
    print("  4. Polar grid: 16 angles × 6 radii + origin = 97 candidates.")
    print("  5. Legality test: full multi-constraint (copper + hole) for all")
    print("     nearby obstacles within 1.0 mm window — NOT just the triggering pair.")
    print("  6. Teardrops / zone copper not included in obstacle model (not in")
    print("     segment/via/pad parsing). May slightly overestimate nudgeability.")
    print()

    # -----------------------------------------------------------------------
    # Structured JSON result
    # -----------------------------------------------------------------------
    result = {
        "status": "success",
        "summary": (
            f"Via-involved violations: {via_involved_total}. "
            f"Via-nudgeable (full hole-aware test): {implied_fixable}. "
            f"Implied ceiling: {baseline_errors} → ~{implied_after}. "
            f"Verdict: {verdict}."
        ),
        "files_changed": [
            "scripts/probe_via_nudge_holeaware.py"
        ],
        "files_read": [board_path, drc_path],
        "via_involved_total": via_involved_total,
        "buckets": {
            "via_nudgeable": grand_nudgeable,
            "boxed_in": grand_boxed,
            "not_via": grand_not_via,
            "by_subtype": {
                st: {
                    "via_nudgeable": tally('via_nudgeable', st),
                    "boxed_in": tally('boxed_in', st),
                    "not_via": tally('not_via', st),
                }
                for st in subtypes_of_interest
            },
        },
        "unique_vias_involved": n_unique_all,
        "unique_vias_nudgeable": n_unique_nudgeable,
        "implied_fixable": implied_fixable,
        "implied_errors_after": implied_after,
        "verdict": "high-value" if verdict == "HIGH-VALUE" else "low-value",
        "caveats": [
            "Pad copper approximated as axis-aligned rect (no per-pad rotation correction)",
            "Oval pad drills: uses max dimension / 2 as conservative drill radius",
            "Connectivity: via stays within R=0.3mm (bounded-nudge assumption)",
            "Polar grid: 16 angles x 6 radii = 96 candidates + origin = 97 total",
            "Legality: full multi-constraint test (copper AND hole clearances, all nearby obstacles)",
            "Teardrop/zone copper excluded from obstacle model (may slightly overestimate nudgeability)",
        ],
        "issues": [],
        "assumptions": [
            "copper_clearance_default=0.15 mm (from violation description 'зазор 0.1500 mm')",
            "hole_clearance_default=0.25 mm (from violation description 'зазор 0.2500 mm')",
            "NUDGE_RADIUS=0.3 mm (same as emitter bound)",
            "NEARBY_WINDOW=1.0 mm for obstacle collection",
        ],
    }

    print("## Structured Result")
    print(json.dumps(result, indent=2, ensure_ascii=False))


if __name__ == '__main__':
    main()
