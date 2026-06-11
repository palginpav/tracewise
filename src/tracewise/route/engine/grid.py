"""Occupancy grid: the routing world model.

An int16 *count* per cell per layer at a fixed pitch (default 0.1 mm): every
obstacle adds +1 over its clearance-inflated footprint, removal subtracts 1,
and a cell is free iff its count is zero. Counting (rather than boolean
marking) is what makes rip-up sound: unmarking one net cannot erase another
obstacle that overlapped the same cells — the bug class that produced
route-through-pad shorts in R2. A free cell is a legal track centerline by
construction.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

FREE = 0
BLOCKED = 1


@dataclass
class Grid:
    x0: float  # world origin (mm)
    y0: float
    width_mm: float
    height_mm: float
    pitch: float = 0.1
    layers: int = 2
    cells: np.ndarray = field(init=False)

    def __post_init__(self) -> None:
        self.nx = max(1, int(round(self.width_mm / self.pitch)))
        self.ny = max(1, int(round(self.height_mm / self.pitch)))
        self.cells = np.zeros((self.layers, self.ny, self.nx), dtype=np.int16)
        # hard copper only (no clearance halo): the escape allowance may relax
        # halos near route endpoints but must never cross actual copper
        self.hard = np.zeros((self.layers, self.ny, self.nx), dtype=np.int16)

    # --- transforms ---------------------------------------------------------

    def to_cell(self, x: float, y: float) -> tuple[int, int]:
        return (int(round((y - self.y0) / self.pitch)), int(round((x - self.x0) / self.pitch)))

    def clamp_cell(self, iy: int, ix: int) -> tuple[int, int]:
        return (min(max(iy, 0), self.ny - 1), min(max(ix, 0), self.nx - 1))

    def to_world(self, iy: int, ix: int) -> tuple[float, float]:
        return (self.x0 + ix * self.pitch, self.y0 + iy * self.pitch)

    def in_bounds(self, iy: int, ix: int) -> bool:
        return 0 <= iy < self.ny and 0 <= ix < self.nx

    # --- obstacle marking (inflation included by the caller's radius) --------

    def _disc(self, arr, layer: int, x: float, y: float, radius_mm: float, delta: int) -> None:
        cy, cx = self.to_cell(x, y)
        r = int(np.ceil(radius_mm / self.pitch))
        y1, y2 = max(0, cy - r), min(self.ny, cy + r + 1)
        x1, x2 = max(0, cx - r), min(self.nx, cx + r + 1)
        if y1 >= y2 or x1 >= x2:
            return
        yy, xx = np.ogrid[y1:y2, x1:x2]
        mask = (yy - cy) ** 2 + (xx - cx) ** 2 <= r * r
        arr[layer, y1:y2, x1:x2][mask] += delta

    def block_disc(self, layer: int, x: float, y: float, radius_mm: float,
                   delta: int = 1, hard_radius_mm: float | None = None) -> None:
        self._disc(self.cells, layer, x, y, radius_mm, delta)
        if hard_radius_mm is not None:
            self._disc(self.hard, layer, x, y, hard_radius_mm, delta)

    def block_rect(self, layer: int, x1: float, y1: float, x2: float, y2: float,
                   inflate_mm: float = 0.0) -> None:
        cy1, cx1 = self.to_cell(min(x1, x2) - inflate_mm, min(y1, y2) - inflate_mm)
        cy2, cx2 = self.to_cell(max(x1, x2) + inflate_mm, max(y1, y2) + inflate_mm)
        self.cells[layer, max(0, cy1):min(self.ny, cy2 + 1),
                   max(0, cx1):min(self.nx, cx2 + 1)] += 1

    def free(self, layer: int, iy: int, ix: int) -> bool:
        return self.in_bounds(iy, ix) and self.cells[layer, iy, ix] == FREE

    def halo_only(self, layer: int, iy: int, ix: int) -> bool:
        """Blocked by clearance halo but not by actual copper."""
        return (self.in_bounds(iy, ix) and self.cells[layer, iy, ix] != FREE
                and self.hard[layer, iy, ix] == 0)

    def free_fraction(self) -> float:
        return float((self.cells == FREE).mean())
