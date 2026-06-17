"""Pour-copper extraction, rasterization, and power-pour synthesis (F0 / F3).

Data source: the **pcbnew API** (not s-expression parsing).  The s-expr
``net_name`` field is unreliable on pcbnew-resaved boards (it can be absent or
carried only as a numeric net code), whereas ``zone.GetNetname()`` and
``pad.GetNetname()`` always return the canonical name regardless of save format.
This is the core ``source == resaved`` invariant verified by the F0 test suite.

Public surface
--------------
- :class:`PourPoly`            — one filled-polygon region on one layer
- :class:`IsolatedPad`         — a pad not electrically connected to its net's pour
- :func:`extract_pours`        — post-fill geometry per net, via pcbnew
- :func:`unconnected_pads`     — stitch worklist (pads isolated from their pour)
- :func:`rasterize_pour`       — bool mask ``[L, H, W]`` of pour-copper cells
- :func:`synthesize_power_pours` — add low-priority pours for high-fanout power nets (F3)
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import TYPE_CHECKING

import numpy as np

from tracewise.route.bridge import _run_pcbnew_script
from tracewise.route.engine.grid import Grid

if TYPE_CHECKING:
    pass

# ---------------------------------------------------------------------------
# Layer-ID constants (pcbnew internal values that never change across versions)
# ---------------------------------------------------------------------------
_F_CU_ID = 0   # pcbnew.F_Cu
_B_CU_ID = 2   # pcbnew.B_Cu  (NOT 1 — inner layers are 2-based)

# Map pcbnew copper-layer ID → grid layer index (F=0, B=1)
_LAYER_TO_GRID: dict[int, int] = {_F_CU_ID: 0, _B_CU_ID: 1}


# ---------------------------------------------------------------------------
# Data classes
# ---------------------------------------------------------------------------


@dataclass
class PourPoly:
    """One filled-copper polygon region for a net, on one layer.

    Data source: pcbnew ``ZONE.GetFilledPolysList(layer)`` (post-fill).
    Coordinates are in mm (pcbnew nm ÷ 1 000 000).  ``layer`` follows the
    grid convention: 0 = F.Cu, 1 = B.Cu.

    The source-vs-resaved invariant: ``extract_pours`` on the source board and
    on a ``pcbnew.SaveBoard``-resaved copy produce identical ``PourPoly`` lists
    (same net names, same polygon vertices) because both come from the same
    pcbnew fill data — not from s-expr ``net_name`` strings.

    ``island_id`` is the polygon index within ``GetFilledPolysList`` for the
    owning zone (0-based).  F2 can use it to prefer the island with the most
    already-connected pads when choosing a stitch target.
    """

    net: str
    layer: int                          # 0 = F.Cu, 1 = B.Cu
    pts: list[tuple[float, float]]      # outline vertices, mm
    island_id: int = field(default=0)   # polygon index within the zone's fill


@dataclass
class IsolatedPad:
    """A pad whose net has a copper pour but the pad is not electrically
    connected to it (e.g. isolated by a clearance moat or outside the zone).

    Data source: pcbnew connectivity — ``CONNECTIVITY_DATA.IsConnectedOnLayer``
    after ``BOARD.BuildConnectivity()``.  pcbnew's thermal-relief logic means
    a pad with thermal spokes *is* connected; ``IsConnectedOnLayer`` respects
    this, so we never need to re-derive it.

    The source-vs-resaved invariant matches ``extract_pours``: netnames come
    from ``pad.GetNetname()`` not from s-expr strings.
    """

    net: str
    ref: str            # footprint reference, e.g. "U1"
    pad: str            # pad number/name, e.g. "1"
    x: float            # pad centre, mm
    y: float
    layers: tuple[int, ...]   # grid layer indices the pad is on


# ---------------------------------------------------------------------------
# pcbnew script templates
# ---------------------------------------------------------------------------

_POURS_SCRIPT = """\
import wx; wx.DisableAsserts()
import pcbnew, json

IU = 1e6
F_CU = pcbnew.F_Cu   # 0
B_CU = pcbnew.B_Cu   # 2  (not 1; inner layers use 2-based IDs)
LAYER_MAP = {{F_CU: 0, B_CU: 1}}

b = pcbnew.LoadBoard({board!r})
b.BuildConnectivity()

pours = []
for z in sorted(b.Zones(), key=lambda z: (z.GetNetname(), z.GetLayer())):
    net_name = z.GetNetname()
    if not net_name:
        continue
    for layer_id in z.GetLayerSet().CuStack():
        if layer_id not in LAYER_MAP:
            continue
        if not z.HasFilledPolysForLayer(layer_id):
            continue
        ps = z.GetFilledPolysList(layer_id)
        grid_layer = LAYER_MAP[layer_id]
        for i in range(ps.OutlineCount()):
            outline = ps.Outline(i)
            n = outline.PointCount()
            pts = [[outline.CPoint(j).x / IU, outline.CPoint(j).y / IU]
                   for j in range(n)]
            pours.append({{
                "net": net_name,
                "layer": grid_layer,
                "pts": pts,
                "island_id": i,
            }})

print("TWJSON" + json.dumps(pours))
raise SystemExit(0)
"""

_UNCONNECTED_SCRIPT = """\
import wx; wx.DisableAsserts()
import pcbnew, json

IU = 1e6
F_CU = pcbnew.F_Cu
B_CU = pcbnew.B_Cu
LAYER_MAP = {{F_CU: 0, B_CU: 1}}

b = pcbnew.LoadBoard({board!r})
b.BuildConnectivity()
conn = b.GetConnectivity()
conn.RecalculateRatsnest()

# Collect nets that have a filled zone (the "has a pour" filter from spec)
pour_nets = set()
for z in b.Zones():
    net_name = z.GetNetname()
    if not net_name:
        continue
    for layer_id in z.GetLayerSet().CuStack():
        if layer_id in (F_CU, B_CU) and z.HasFilledPolysForLayer(layer_id):
            pour_nets.add(net_name)
            break

isolated = []
for fp in sorted(b.GetFootprints(), key=lambda f: f.GetReference()):
    for pad in sorted(fp.Pads(), key=lambda p: p.GetNumber()):
        net_name = pad.GetNetname()
        if net_name not in pour_nets:
            continue
        # A pad is isolated if it is NOT connected on any of its copper layers.
        # IsConnectedOnLayer respects zone fills (including thermal-relief spokes)
        # exactly as pcbnew's own DRC does — no re-implementation needed.
        connected = False
        for layer_id in pad.GetLayerSet().CuStack():
            if layer_id in (F_CU, B_CU):
                if conn.IsConnectedOnLayer(pad, layer_id):
                    connected = True
                    break
        if not connected:
            pos = pad.GetPosition()
            layers = [LAYER_MAP[l] for l in pad.GetLayerSet().CuStack()
                      if l in LAYER_MAP]
            isolated.append({{
                "net": net_name,
                "ref": fp.GetReference(),
                "pad": pad.GetNumber(),
                "x": pos.x / IU,
                "y": pos.y / IU,
                "layers": sorted(layers),
            }})

print("TWJSON" + json.dumps(isolated))
raise SystemExit(0)
"""


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------


def extract_pours(board: str | Path) -> dict[str, list[PourPoly]]:
    """Return post-fill pour geometry per net.

    Data source: pcbnew API (``ZONE.GetFilledPolysList``).  The board must
    have been zone-filled before calling (e.g. after ``refill_zones``).

    **Source-vs-resaved invariant**: results are identical whether ``board`` is
    the original source file or a ``pcbnew.SaveBoard``-resaved copy, because
    ``z.GetNetname()`` reads the canonical net name from pcbnew's internal data
    — unlike s-expr ``net_name`` which can be absent in resaved boards.

    Returns a dict mapping net name → sorted list of :class:`PourPoly`.  Sorted
    by ``(net, layer, island_id)`` for deterministic output.

    Uses the ``_run_pcbnew_script`` JSON pattern (same as ``board_metrics`` in
    ``bridge.py``).  Output is a single ``TWJSON`` prefixed line on stdout.
    """
    board = Path(board).resolve()
    out = _run_pcbnew_script(_POURS_SCRIPT.format(board=str(board)))
    for line in out.splitlines():
        if line.startswith("TWJSON"):
            raw: list[dict] = json.loads(line[len("TWJSON"):])
            result: dict[str, list[PourPoly]] = {}
            for item in raw:
                pp = PourPoly(
                    net=item["net"],
                    layer=item["layer"],
                    pts=[(x, y) for x, y in item["pts"]],
                    island_id=item["island_id"],
                )
                result.setdefault(item["net"], []).append(pp)
            # Deterministic order within each net
            for polys in result.values():
                polys.sort(key=lambda p: (p.layer, p.island_id))
            return result
    raise RuntimeError("extract_pours: pcbnew script produced no TWJSON line")


def unconnected_pads(board: str | Path) -> list[IsolatedPad]:
    """Return pads not electrically connected to their net's copper pour.

    Data source: pcbnew connectivity (``CONNECTIVITY_DATA.IsConnectedOnLayer``
    after ``BOARD.BuildConnectivity()``).  Thermal-relief pads that have spokes
    ARE counted as connected — pcbnew handles this internally.

    Only pads whose net has at least one filled zone are returned (the "stitch
    worklist" per the F0 spec).  Pads on nets with no pour are skipped.

    **Source-vs-resaved invariant**: identical results on source and resaved
    boards because net names come from ``pad.GetNetname()`` not s-expr strings.

    Returns a list of :class:`IsolatedPad` sorted by ``(net, ref, pad)`` for
    deterministic output.
    """
    board = Path(board).resolve()
    out = _run_pcbnew_script(_UNCONNECTED_SCRIPT.format(board=str(board)))
    for line in out.splitlines():
        if line.startswith("TWJSON"):
            raw: list[dict] = json.loads(line[len("TWJSON"):])
            return [
                IsolatedPad(
                    net=item["net"],
                    ref=item["ref"],
                    pad=item["pad"],
                    x=item["x"],
                    y=item["y"],
                    layers=tuple(item["layers"]),
                )
                for item in raw
            ]
    raise RuntimeError("unconnected_pads: pcbnew script produced no TWJSON line")


def rasterize_pour(grid: Grid, polys: list[PourPoly]) -> np.ndarray:
    """Return a bool mask ``[L, H, W]`` marking pour-copper cells in *grid*.

    Each cell whose centre falls inside any polygon in *polys* is set ``True``
    on the polygon's layer.  The result is a pure-numpy union of all polygons —
    it is NOT written into ``grid.cells`` or ``grid.hard`` (the pour is a
    routing GOAL for stitching, not an obstacle).

    Algorithm: the same vectorized Jordan ray-cast used in
    ``Grid.block_polygon``, applied over each polygon's cell bounding box.
    Off-by-one-safe: cells are tested at their **centre** world coordinates,
    which are strictly interior to a 1-cell-pitch margin.

    Coordinate transform: ``PourPoly.pts`` are in mm → ``grid.to_cell(x, y)``
    → ``(iy, ix)``.  Layer: ``PourPoly.layer`` directly indexes the mask.

    Only layers 0 and 1 (F.Cu / B.Cu) are handled; polygons on other layers
    are silently skipped.
    """
    mask = np.zeros((grid.layers, grid.ny, grid.nx), dtype=bool)

    for poly in polys:
        layer = poly.layer
        if layer < 0 or layer >= grid.layers:
            continue
        pts = poly.pts
        if len(pts) < 3:
            continue

        # Bounding box in cell coordinates
        xs = [px for px, _ in pts]
        ys = [py for _, py in pts]
        cy1, cx1 = grid.to_cell(min(xs), min(ys))
        cy2, cx2 = grid.to_cell(max(xs), max(ys))
        cy1, cx1 = max(0, cy1), max(0, cx1)
        cy2, cx2 = min(grid.ny - 1, cy2), min(grid.nx - 1, cx2)
        if cy1 > cy2 or cx1 > cx2:
            continue

        # Cell-centre world coordinates (same approach as Grid.block_polygon)
        yy, xx = np.mgrid[cy1 : cy2 + 1, cx1 : cx2 + 1]
        wx = grid.x0 + xx * grid.pitch
        wy = grid.y0 + yy * grid.pitch

        # Vectorized Jordan ray-cast point-in-polygon
        inside = np.zeros(wx.shape, dtype=bool)
        n = len(pts)
        for i in range(n):
            x1p, y1p = pts[i]
            x2p, y2p = pts[(i + 1) % n]
            cond = (y1p > wy) != (y2p > wy)
            cond &= wx < (x2p - x1p) * (wy - y1p) / ((y2p - y1p) or 1e-12) + x1p
            inside ^= cond

        mask[layer, cy1 : cy2 + 1, cx1 : cx2 + 1] |= inside

    return mask


# ---------------------------------------------------------------------------
# F3: power-pour synthesis
# ---------------------------------------------------------------------------

# Nets excluded from synthesized pours — they already have hand-authored zones.
_POUR_EXCLUDES = frozenset({"GND", "EMIGND"})

# Priority bump applied to all pre-existing zones so they win contested copper
# over the synthesized low-priority pours.
_PREEXISTING_PRIORITY_BUMP = 10

_SYNTH_SCRIPT = """\
import re, json
import pcbnew

POWER = re.compile(r"^\\+?(VCC|VDD|VBUS|VBAT|VSYS|GND|VSS|[0-9]+V[0-9]*)$", re.IGNORECASE)
EXCLUDES = {exclude_set!r}
MIN_PADS = {min_pads!r}
PRIORITY_BUMP = {priority_bump!r}
IU = 1e6
INSET = int(1.0 * IU)   # 1 mm inset from board edge for synthesized zones

b = pcbnew.LoadBoard({board!r})

# ---- 1. Count pads per net ------------------------------------------------
pad_counts = {{}}
for fp in b.GetFootprints():
    for pad in fp.Pads():
        n = pad.GetNetname()
        if n:
            pad_counts[n] = pad_counts.get(n, 0) + 1

# ---- 2. Find nets that already have a zone --------------------------------
nets_with_zone = set()
for z in b.Zones():
    nn = z.GetNetname()
    if nn:
        nets_with_zone.add(nn)

# ---- 3. Identify qualifying nets ------------------------------------------
qualifying = sorted(
    n for n, cnt in pad_counts.items()
    if POWER.match(n)
    and n not in EXCLUDES
    and cnt >= MIN_PADS
    and n not in nets_with_zone
)

if not qualifying:
    print("TWSYNTH" + json.dumps({{"added": 0, "nets": []}}))
    raise SystemExit(0)

# ---- 4. Find the template zone (largest existing copper zone by outline area).
# The template is used as a SETTINGS donor (clearance, thermal relief, etc.)
# via Duplicate(). Its outline is replaced with the full board bbox below.
template = None
best_area = -1.0
for z in b.Zones():
    if not z.GetNetname():
        continue
    outline = z.Outline()
    if outline.OutlineCount() == 0:
        continue
    bb = z.GetBoundingBox()
    area = bb.GetWidth() * bb.GetHeight()
    if area > best_area:
        best_area = area
        template = z

# ---- 4b. Build the board-wide bounding-box outline for synthesized zones ---
# Each synthesized zone spans the full board so the zone filler can fill
# copper anywhere on the board.  Pre-existing zones win contested cells via
# their higher priority (bumped in step 5).
edges_bb = b.GetBoardEdgesBoundingBox()
bb_pts = [
    (edges_bb.GetLeft()  + INSET, edges_bb.GetTop()    + INSET),
    (edges_bb.GetRight() - INSET, edges_bb.GetTop()    + INSET),
    (edges_bb.GetRight() - INSET, edges_bb.GetBottom() - INSET),
    (edges_bb.GetLeft()  + INSET, edges_bb.GetBottom() - INSET),
]

def _make_zone_with_board_outline(board, net):
    \"\"\"Clone template (for settings) then replace its outline with the full
    board bounding box.  Falls back to a plain new ZONE if no template.\"\"\"
    if template is not None:
        z = template.Duplicate()
    else:
        z = pcbnew.ZONE(board)
        z.SetLayer(pcbnew.F_Cu)
    z.SetNet(net)
    # Replace the outline with the board-wide bbox
    outline = z.Outline()
    outline.RemoveAllContours()
    outline.NewOutline()
    for px, py in bb_pts:
        outline.Append(int(px), int(py))
    return z

# ---- 5. Bump ALL pre-existing copper zones --------------------------------
for z in b.Zones():
    if z.GetNetname():
        z.SetAssignedPriority(z.GetAssignedPriority() + PRIORITY_BUMP)

# ---- 6. Add one low-priority pour per qualifying net ----------------------
added_nets = []
for net_name in qualifying:
    net = b.FindNet(net_name)
    if net is None:
        continue
    new_zone = _make_zone_with_board_outline(b, net)
    new_zone.SetAssignedPriority(0)
    b.Add(new_zone)
    added_nets.append(net_name)

# ---- 7. Fill and save ------------------------------------------------------
pcbnew.ZONE_FILLER(b).Fill(b.Zones())
pcbnew.SaveBoard({board!r}, b)

print("TWSYNTH" + json.dumps({{"added": len(added_nets), "nets": added_nets}}))
raise SystemExit(0)
"""


def synthesize_power_pours(board: str | Path, min_pads: int = 8) -> int:
    """Add a low-priority copper pour for every qualifying high-fanout power net.

    **Rationale** (the GND-wins-contested-copper invariant)
    --------------------------------------------------------
    High-fanout power nets such as ``+3V3`` or ``+5V`` often have dozens of
    pads spread across the board but no copper pour.  The trace router must
    connect them all, which is expensive and frequently leaves unconnected pads.

    By giving these nets a LOW-PRIORITY pour (priority 0), the existing GND /
    EMIGND zones keep their higher priority (bumped by
    ``_PREEXISTING_PRIORITY_BUMP``) and **win every contested copper cell**,
    while the synthesized power-net zones fill the residual copper around them.
    The result: the bulk of the power pads connect via the fill rather than via
    individual traces, dramatically reducing the unconnected count.

    **Algorithm**
    -------------
    1. Count pads per net via pcbnew.
    2. Nets that already have a zone are excluded (no duplicate pours).
    3. Qualifying nets: match the ``POWER`` regex, have ``>= min_pads`` pads,
       no pre-existing zone, and are NOT in the exclusion set (GND, EMIGND).
    4. Find the **largest** existing copper zone as a *settings donor* (clearance,
       thermal-relief mode, etc.).  Its outline is **not** reused — each
       synthesized zone always gets the full board ``Edge.Cuts`` bounding-box
       outline (1 mm inset) so it can fill copper anywhere on the board.
       Fallback when no existing zone exists: create a plain ``ZONE`` directly.
    5. Bump every pre-existing zone's priority by ``_PREEXISTING_PRIORITY_BUMP``
       so they all win over the synthesized priority-0 pours.
    6. Add one board-wide zone per qualifying net at priority 0.
    7. ``ZONE_FILLER.Fill`` + ``SaveBoard``.

    Nets are processed in **sorted (deterministic) order**.

    Parameters
    ----------
    board:
        Path to the ``.kicad_pcb`` file (modified in-place).
    min_pads:
        Minimum pad count a net must have to receive a synthesized pour.
        Default 8 — low enough for 10-pad nets but above single-resistor nets.

    Returns
    -------
    int
        Number of new pours added (0 if no qualifying nets).
    """
    board = Path(board).resolve()
    script = _SYNTH_SCRIPT.format(
        board=str(board),
        exclude_set=set(_POUR_EXCLUDES),
        min_pads=min_pads,
        priority_bump=_PREEXISTING_PRIORITY_BUMP,
    )
    out = _run_pcbnew_script(script)
    for line in out.splitlines():
        if line.startswith("TWSYNTH"):
            data = json.loads(line[len("TWSYNTH"):])
            return int(data["added"])
    raise RuntimeError("synthesize_power_pours: pcbnew script produced no TWSYNTH line")
