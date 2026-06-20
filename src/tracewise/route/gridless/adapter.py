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
        List of path segments.  Each segment is a list of waypoints.  For
        single-layer routes: ``[(x_mm, y_mm), ...]`` (2-tuples).  For 2-layer
        routes (M3): ``[(x_mm, y_mm, layer), ...]`` (3-tuples, ``layer ∈ {0,1}``).
        ``emit_routes`` writes these directly, bypassing ``grid.to_world``.
    world_vias:
        Via centre positions for 2-layer routes: ``[(x_mm, y_mm), ...]``.
        Empty for single-layer routes.  ``emit_routes`` emits one KiCad ``via``
        node per entry.  ``rasterize_into_grid`` marks both layers around each
        via so subsequent grid-routed nets see the via copper.
    """

    # World-mm centerlines: [ [(x, y), ...] or [(x, y, layer), ...], ... ]
    world_paths: list[list[tuple]] = field(default_factory=list)
    # Via centres for 2-layer routes: [(x_mm, y_mm), ...]
    world_vias: list[tuple[float, float]] = field(default_factory=list)

    def rasterize_into_grid(self, grid: Grid) -> None:
        """Populate ``self.cells`` from ``self.world_paths`` and ``self.world_vias``.

        Walk each segment of each world path, rasterize it to the grid cells it
        occupies (Bresenham-style: sample every ``pitch/2`` along the segment to
        ensure no cell is skipped), and collect the set.  Handles both
        2-tuple ``(x, y)`` waypoints (single-layer) and 3-tuple ``(x, y, layer)``
        waypoints (2-layer M3 routes).

        Via sites are rasterized onto both layers (0 and 1) so subsequent
        grid-routed nets see the via copper as occupied cells.

        Inflation is **not** applied here — ``_mark`` applies the
        ``halfwidth_cells`` box inflation when writing to ``grid.cells``.  This
        method only populates the ``cells`` set (the centerline cells), which is
        what ``_mark`` iterates.

        Already-populated ``cells`` are replaced, not appended (idempotent call).
        """
        cells: set[tuple[int, int, int]] = set()

        for path in self.world_paths:
            for wa, wb in zip(path, path[1:], strict=False):
                # Support both 2-tuples (x, y) and 3-tuples (x, y, layer)
                if len(wa) == 3:
                    x0, y0, layer_a = wa
                else:
                    x0, y0 = wa
                    layer_a = 0
                if len(wb) == 3:
                    x1, y1, _layer_b = wb
                else:
                    x1, y1 = wb

                # Skip via-transition waypoints (same position, different layer)
                if abs(x0 - x1) < 1e-9 and abs(y0 - y1) < 1e-9:
                    continue

                layer = layer_a  # use the layer of the starting waypoint
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

        # Rasterize via sites onto BOTH layers so grid-routed nets avoid via copper
        for vx, vy in self.world_vias:
            iy, ix = grid.to_cell(vx, vy)
            for lyr in (0, 1):
                cells.add((lyr, *grid.clamp_cell(iy, ix)))

        self.cells = cells


def to_gridless_netroute(
    net: Net,
    world_paths: list[list[tuple]],
    grid: Grid,
    world_vias: list[tuple[float, float]] | None = None,
) -> GridlessNetRoute:
    """Build a ``GridlessNetRoute`` from a successfully-routed gridless result.

    Parameters
    ----------
    net:
        The ``Net`` object (provides ``halfwidth_cells``, ``via_halfwidth_cells``).
    world_paths:
        World-mm centerlines from ``GridlessRouteResult.world_paths``.  May
        contain 2-tuples ``(x, y)`` (single-layer) or 3-tuples ``(x, y, layer)``
        (2-layer M3 routes).
    grid:
        The shared occupancy grid (used for rasterization and coordinate mapping).
    world_vias:
        Via centre positions from ``GridlessRouteResult.world_vias``.  When
        provided, each via is rasterized onto both grid layers so subsequent
        grid-routed nets see the via copper.  Defaults to ``[]``.

    Returns
    -------
    A ``GridlessNetRoute`` with ``ok=True``, ``world_paths`` set, and ``cells``
    populated by ``rasterize_into_grid``.
    """
    nr = GridlessNetRoute(
        net=net,
        ok=True,
        world_paths=world_paths,
        world_vias=world_vias or [],
    )
    nr.rasterize_into_grid(grid)
    return nr
