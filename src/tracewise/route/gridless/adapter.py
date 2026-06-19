"""adapter — GridlessNetRoute: IS-A NetRoute adapter for the FAR gridless router.

``GridlessNetRoute`` subclasses ``NetRoute`` and carries world-coordinate
centerlines (``world_paths``) alongside the rasterized grid fields (``cells``,
``via_sites``) that the engine's hot-loop — ``_mark``, ``_nearest_victim``,
``route_all``, salvage pass — reads uniformly.

The key contract is **grid-ledger rasterization**: ``rasterize_into_grid`` walks
each world-mm segment, converts it to grid cells, inflates by ``net.halfwidth_cells``
(track_hw + clearance, same as ``_mark`` uses), and writes those cells into
``self.cells``.  Grid-routed nets routed after a gridless net therefore see its
copper as occupied cells — making the two substrates **share a single occupancy
ledger**.

Design constraints (from ``FAR-gridless-router-arch.md`` §Decision 2):
  - IS-A ``NetRoute``: no special-casing in ``_mark``, rip-up logic, salvage, or
    summary loops.
  - ``world_paths`` carries the exact geometry for ``emit_routes``; emit must NOT
    re-snap those through ``grid.to_world``.
  - ``rasterize_into_grid`` uses the SAME inflation radius as ``_mark`` (the
    ``net.halfwidth_cells`` field set by ``build_problem``), so subsequent grid
    nets see gridless copper identically to grid copper.
  - Deterministic: cell set depends only on snapped world coordinates and the grid
    parameters, both fixed per routing problem.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from tracewise.route.engine.grid import Grid
from tracewise.route.engine.multi import Net, NetRoute


@dataclass
class GridlessNetRoute(NetRoute):
    """A ``NetRoute`` whose copper was routed by the visibility-graph gridless engine.

    Inherits all ``NetRoute`` fields so ``_mark``, ``route_all``, and every
    downstream consumer accept it without modification.

    Extra fields
    ------------
    world_paths:
        List of path segments.  Each segment is a list of ``(x_mm, y_mm)``
        waypoints for F.Cu (single-layer Phase 1; Phase 3 adds ``layer``).
        ``emit_routes`` writes these directly, bypassing ``grid.to_world``.
    """

    # World-mm centerlines: [ [(x, y), ...], ... ] per path segment
    world_paths: list[list[tuple[float, float]]] = field(default_factory=list)

    def rasterize_into_grid(self, grid: Grid) -> None:
        """Populate ``self.cells`` from ``self.world_paths``.

        Walk each segment of each world path, rasterize it to the grid cells it
        occupies (Bresenham-style: sample every ``pitch/2`` along the segment to
        ensure no cell is skipped), and collect the set.

        Inflation is **not** applied here — ``_mark`` applies the
        ``halfwidth_cells`` box inflation when writing to ``grid.cells``.  This
        method only populates the ``cells`` set (the centerline cells), which is
        what ``_mark`` iterates.

        Already-populated ``cells`` are replaced, not appended (idempotent call).
        """
        cells: set[tuple[int, int, int]] = set()
        layer = 0  # Phase 1: F.Cu only

        for path in self.world_paths:
            for (x0, y0), (x1, y1) in zip(path, path[1:], strict=False):
                # Sample along the segment at sub-pitch intervals so that no
                # grid cell is skipped.  pitch/2 guarantees coverage.
                seg_len = math.hypot(x1 - x0, y1 - y0)
                if seg_len < 1e-9:
                    iy, ix = grid.to_cell(x0, y0)
                    cells.add((layer, *grid.clamp_cell(iy, ix)))
                    continue
                step = grid.pitch / 2.0
                n_steps = max(1, int(math.ceil(seg_len / step)))
                for k in range(n_steps + 1):
                    t = k / n_steps
                    x = x0 + t * (x1 - x0)
                    y = y0 + t * (y1 - y0)
                    iy, ix = grid.to_cell(x, y)
                    cells.add((layer, *grid.clamp_cell(iy, ix)))

        self.cells = cells


def to_gridless_netroute(
    net: Net,
    world_paths: list[list[tuple[float, float]]],
    grid: Grid,
) -> GridlessNetRoute:
    """Build a ``GridlessNetRoute`` from a successfully-routed gridless result.

    Parameters
    ----------
    net:
        The ``Net`` object (provides ``halfwidth_cells``, ``via_halfwidth_cells``).
    world_paths:
        World-mm centerlines from ``GridlessRouteResult.world_paths``.
    grid:
        The shared occupancy grid (used for rasterization and coordinate mapping).

    Returns
    -------
    A ``GridlessNetRoute`` with ``ok=True``, ``world_paths`` set, and ``cells``
    populated by ``rasterize_into_grid``.  ``via_sites`` is empty (Phase 1:
    single-layer, no vias).
    """
    nr = GridlessNetRoute(net=net, ok=True, world_paths=world_paths)
    nr.rasterize_into_grid(grid)
    return nr
