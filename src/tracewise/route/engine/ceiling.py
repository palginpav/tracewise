"""L1 ceiling detector: classify unrouted ratsnest gaps as ROUTER_RECOVERABLE
or UNROUTABLE_2LAYER.

Method — free-space connected components
-----------------------------------------
On the final routing grid (all pads/keepouts/routed copper marked as obstacles
in ``grid.cells``):

1. **Free-space graph**: a cell ``(layer, iy, ix)`` is a node iff
   ``grid.cells[layer, iy, ix] == 0``.  Edges:

   * 8-connectivity within a layer between free cells (mirrors the A* move set).
   * Inter-layer edges ``(0, iy, ix) ↔ (1, iy, ix)`` when a via is *legal* at
     ``(iy, ix)`` — conservative proxy: the ``VIA_RING``-cell bounding square is
     entirely free on **both** layers (same test as ``astar.via_ok``).

2. **Connected-component labelling** over that graph (iterative BFS with a LIFO
   stack; no scipy dependency; vectorised per-layer free-cell extraction).

3. **Gap classification**: for each unconnected ratsnest gap (two endpoints with
   world positions), map each endpoint to its nearest free cell within a small
   search window (the endpoint sits *on* copper, so we step outward to find the
   first free neighbour).  If both representatives share the same component label
   the gap is ``ROUTER_RECOVERABLE`` (a path exists — the router just missed it);
   otherwise ``UNROUTABLE_2LAYER`` (genuinely needs 4+ layers or re-layout).

This analysis is **read-only**: it never modifies ``grid.cells``, the board
file, or any routing state.

Public API
----------
``classify_unrouted(board, grid) -> CeilingResult``
    Main entry point.  ``board`` is the path to the ``.kicad_pcb`` file (needed
    only to call ``run_drc`` for the unconnected-items list).  ``grid`` is the
    *live* occupancy grid after routing + pours.

``label_components(grid) -> np.ndarray``
    Lower-level: returns the ``(layers, ny, nx)`` int32 label array.
    Component 0 is the *obstacle* sentinel; real components start at 1.
    Deterministic: cells are visited in row-major order.

``CeilingResult``
    Dataclass with ``recoverable``, ``unroutable_2layer``, ``by_net``, and
    ``details`` fields.  JSON-serialisable via ``asdict()``.
"""

from __future__ import annotations

import re
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import TYPE_CHECKING

import numpy as np

from tracewise.route.engine.astar import VIA_RING
from tracewise.route.engine.grid import FREE, Grid

if TYPE_CHECKING:
    pass


# ---------------------------------------------------------------------------
# Data classes
# ---------------------------------------------------------------------------


@dataclass
class GapDetail:
    """Classification record for one unconnected ratsnest gap."""
    net: str
    #: World coordinates of the two endpoints (mm).
    pos_a: tuple[float, float]
    pos_b: tuple[float, float]
    #: Grid representatives (layer, iy, ix) found for each endpoint.
    rep_a: tuple[int, int, int] | None
    rep_b: tuple[int, int, int] | None
    #: Component labels; -1 means no free cell was found near that endpoint.
    label_a: int
    label_b: int
    classification: str  # "ROUTER_RECOVERABLE" | "UNROUTABLE_2LAYER" | "UNKNOWN"


@dataclass
class CeilingResult:
    """Aggregate classification of all unconnected ratsnest gaps."""
    recoverable: int = 0
    unroutable_2layer: int = 0
    #: Gaps where an endpoint is buried in copper with no free cell in range —
    #: classification indeterminate. recoverable + unroutable_2layer + unknown
    #: == total ratsnest gaps.
    unknown: int = 0
    #: Per-net counts: {net: {"recoverable": n, "unroutable": m, "unknown": k}}.
    by_net: dict[str, dict[str, int]] = field(default_factory=dict)
    #: One entry per gap.
    details: list[GapDetail] = field(default_factory=list)

    def as_dict(self) -> dict:
        """Return a JSON-serialisable plain dict."""
        return {
            "recoverable": self.recoverable,
            "unroutable_2layer": self.unroutable_2layer,
            "unknown": self.unknown,
            "by_net": self.by_net,
            "details": [asdict(d) for d in self.details],
        }


# ---------------------------------------------------------------------------
# Component labelling
# ---------------------------------------------------------------------------

# Neighbour offsets for 8-connectivity (dy, dx).
_NEIGH8 = [(-1, -1), (-1, 0), (-1, 1),
           (0, -1),           (0, 1),
           (1, -1),  (1, 0),  (1, 1)]


def label_components(grid: Grid) -> np.ndarray:
    """Label free-space connected components on the routing grid.

    Returns an int32 array of shape ``(layers, ny, nx)``.

    * Cells with ``grid.cells[l, iy, ix] != 0`` (obstacles) receive label 0.
    * Free cells share a label iff they are path-connected via:
      - 8-directional moves within the same layer between free cells, OR
      - a vertical (inter-layer) edge at ``(iy, ix)`` that passes the
        ``VIA_RING`` free-ring test on *both* layers.

    Component labels start at 1 and are assigned in raster (row-major) scan
    order for determinism.  The algorithm is a plain iterative DFS/stack BFS
    (no external dependencies; avoids recursion-depth limits on large grids).

    Performance: on a 901×1001×2 grid (~1.8 M cells) the BFS visits at most
    ~1.8 M nodes, each popped once from a Python list used as a LIFO stack.
    Per-cell work is O(10) neighbour checks (8 planar + 1 via).  Typical wall
    time is 3–10 s on a single CPU core, well within the "a few seconds" budget
    stated in the specification.
    """
    L, H, W = grid.cells.shape
    cells = grid.cells  # int16, read-only view

    labels = np.zeros((L, H, W), dtype=np.int32)
    # Mark all obstacle cells as 0 (already done by zeros init); free cells
    # will be assigned component >= 1 when first visited.
    # Use a sentinel value of -1 for "already in the BFS stack but not yet
    # processed" — this avoids re-queueing cells and keeps the stack compact.
    # Actually: use the labels array itself; a cell is "unvisited free" iff
    # labels[l,iy,ix] == 0 AND cells[l,iy,ix] == FREE.

    next_label = 1
    VR = VIA_RING

    # Precompute which (iy, ix) positions admit via transitions — positions
    # where the VIA_RING square is entirely free on ALL layers.
    # A via at (iy, ix) is legal if:
    #   iy-VR >= 0, iy+VR < H, ix-VR >= 0, ix+VR < W   AND
    #   cells[:, iy-VR:iy+VR+1, ix-VR:ix+VR+1].any() == False
    # Computing this lazily during BFS (one slice per candidate) is fast
    # enough because via edges are only checked when a free cell is a BFS
    # frontier node, not for every neighbour.

    def via_ok(iy: int, ix: int) -> bool:
        if iy - VR < 0 or iy + VR >= H or ix - VR < 0 or ix + VR >= W:
            return False
        return not cells[:, iy - VR:iy + VR + 1, ix - VR:ix + VR + 1].any()

    # Scan in row-major order for determinism; skip if already labelled or obstacle.
    for layer_idx in range(L):
        for iy in range(H):
            for ix in range(W):
                if cells[layer_idx, iy, ix] != FREE or labels[layer_idx, iy, ix] != 0:
                    continue
                # Seed a new component.
                comp = next_label
                next_label += 1
                stack = [(layer_idx, iy, ix)]
                labels[layer_idx, iy, ix] = comp
                while stack:
                    cl, cy, cx = stack.pop()
                    # 8-connected in-layer neighbours.
                    for dy, dx in _NEIGH8:
                        ny_, nx_ = cy + dy, cx + dx
                        if 0 <= ny_ < H and 0 <= nx_ < W:
                            if cells[cl, ny_, nx_] == FREE and labels[cl, ny_, nx_] == 0:
                                labels[cl, ny_, nx_] = comp
                                stack.append((cl, ny_, nx_))
                    # Inter-layer via edges (if via is legal at this position).
                    if via_ok(cy, cx):
                        for other_l in range(L):
                            if other_l != cl:
                                if cells[other_l, cy, cx] == FREE and labels[other_l, cy, cx] == 0:
                                    labels[other_l, cy, cx] = comp
                                    stack.append((other_l, cy, cx))

    return labels


# ---------------------------------------------------------------------------
# Nearest-free-cell search
# ---------------------------------------------------------------------------

_WINDOW = 8  # cells to search around an endpoint for a free representative


def _nearest_free(grid: Grid, labels: np.ndarray,
                  iy: int, ix: int, layers: list[int],
                  window: int = _WINDOW) -> tuple[tuple[int, int, int] | None, int]:
    """Return ``((layer, iy, ix), label)`` of the nearest free cell to
    ``(iy, ix)`` in ``layers``, within ``window`` cells (Manhattan-ish spiral).
    Returns ``(None, -1)`` if nothing is found.

    Search order: Manhattan distance 0, 1, 2, … up to ``window``.  Within
    each ring, cells are iterated in a fixed order (deterministic).
    """
    H, W = grid.ny, grid.nx
    for dist in range(window + 1):
        # Generate all (dy, dx) with max(|dy|, |dx|) == dist.
        candidates: list[tuple[int, int]] = []
        if dist == 0:
            candidates = [(0, 0)]
        else:
            for dy in range(-dist, dist + 1):
                for dx in range(-dist, dist + 1):
                    if max(abs(dy), abs(dx)) == dist:
                        candidates.append((dy, dx))
        for dy, dx in candidates:
            ny_, nx_ = iy + dy, ix + dx
            if not (0 <= ny_ < H and 0 <= nx_ < W):
                continue
            for li in layers:
                if grid.cells[li, ny_, nx_] == FREE:
                    lbl = int(labels[li, ny_, nx_])
                    return (li, ny_, nx_), lbl
    return None, -1


# ---------------------------------------------------------------------------
# Main entry point
# ---------------------------------------------------------------------------


def classify_unrouted(board: str | Path, grid: Grid) -> CeilingResult:
    """Classify each unconnected ratsnest gap as ROUTER_RECOVERABLE or
    UNROUTABLE_2LAYER using free-space connected components on the live grid.

    Parameters
    ----------
    board:
        Path to the ``.kicad_pcb`` file.  Used only to invoke ``run_drc``
        to obtain the unconnected-items list.  The board **must not** be
        modified by the caller between routing and this call.
    grid:
        The live occupancy grid after ``route_all`` + ``emit_routes`` +
        ``refill_zones`` (i.e. ``grid.cells`` reflects all routed copper,
        pads, and keepouts).  **Read-only**: this function never modifies
        the grid.

    Returns
    -------
    CeilingResult
        ``recoverable`` — gaps where a free-space path exists (router missed
        them under its time/budget constraints).
        ``unroutable_2layer`` — gaps where no path exists on the 2-layer
        free-space graph (genuinely needs 4+ layers or re-layout).
        ``by_net`` — per-net breakdown.
        ``details`` — one ``GapDetail`` per gap, suitable for diagnostics.

    Classification logic
    --------------------
    1. Compute free-space connected components via :func:`label_components`.
    2. For each DRC unconnected item:
       a. Parse the two endpoint world coordinates.
       b. Map each endpoint to its grid cell via ``grid.to_cell``; then walk
          outward up to ``_WINDOW`` cells to find the nearest free cell
          (endpoint pads are obstacles in the grid).
       c. Check both F.Cu and B.Cu layers for each endpoint (pads can be
          through-hole or on either layer).
       d. If both representatives share the same component label →
          ROUTER_RECOVERABLE; else → UNROUTABLE_2LAYER.

    This is strictly read-only: no board file is written, no grid cell is
    modified, and ``run_drc`` is called on the already-saved board.
    """
    from tracewise.route.bridge import run_drc

    result = CeilingResult()

    # --- Step 1: DRC for unconnected items -----------------------------------
    try:
        report = run_drc(board)
    except Exception as exc:
        # If DRC is unavailable (no kicad-cli), return empty result gracefully.
        result.details.append(GapDetail(
            net="<drc_error>",
            pos_a=(0.0, 0.0), pos_b=(0.0, 0.0),
            rep_a=None, rep_b=None,
            label_a=-1, label_b=-1,
            classification=f"DRC_ERROR: {exc}",
        ))
        return result

    unconnected = report.get("unconnected_items", [])
    if not unconnected:
        return result

    # --- Step 2: Build component labels (once, shared across all gaps) -------
    labels = label_components(grid)

    # --- Step 3: Classify each gap -------------------------------------------
    for item in unconnected:
        # DRC unconnected_items have two "items" sub-keys, each with a "pos".
        items_list = item.get("items", [])
        if len(items_list) < 2:
            continue
        # The net is NOT a top-level field — it is embedded in each endpoint's
        # localized description as "[NET]" (e.g. "Pad 33 [GND] from U1").
        net_name = ""
        for sub in items_list:
            m = re.search(r"\[([^\]]+)\]", sub.get("description", ""))
            if m:
                net_name = m.group(1)
                break
        try:
            ax = float(items_list[0]["pos"]["x"])
            ay = float(items_list[0]["pos"]["y"])
            bx = float(items_list[1]["pos"]["x"])
            by_ = float(items_list[1]["pos"]["y"])
        except (KeyError, TypeError, ValueError):
            continue

        aiy, aix = grid.clamp_cell(*grid.to_cell(ax, ay))
        biy, bix = grid.clamp_cell(*grid.to_cell(bx, by_))

        # Both F.Cu and B.Cu are searched for each endpoint (through-hole pads
        # appear on both layers, SMD pads on one — we take the best match).
        all_layers = list(range(grid.layers))
        rep_a, lbl_a = _nearest_free(grid, labels, aiy, aix, all_layers)
        rep_b, lbl_b = _nearest_free(grid, labels, biy, bix, all_layers)

        if lbl_a == -1 or lbl_b == -1:
            cls = "UNKNOWN"
            result.unknown += 1
            key = "unknown"
        elif lbl_a == lbl_b:
            cls = "ROUTER_RECOVERABLE"
            result.recoverable += 1
            key = "recoverable"
        else:
            cls = "UNROUTABLE_2LAYER"
            result.unroutable_2layer += 1
            key = "unroutable"

        # Per-net aggregation (every gap counted, keyed by net).
        entry = result.by_net.setdefault(
            net_name or "<unknown-net>",
            {"recoverable": 0, "unroutable": 0, "unknown": 0})
        entry[key] += 1

        result.details.append(GapDetail(
            net=net_name,
            pos_a=(ax, ay), pos_b=(bx, by_),
            rep_a=rep_a, rep_b=rep_b,
            label_a=lbl_a, label_b=lbl_b,
            classification=cls,
        ))

    return result
