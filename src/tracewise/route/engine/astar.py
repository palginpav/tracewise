"""A* maze search over the occupancy grid.

Nodes are (layer, iy, ix). Moves: 8-directional in-layer (diagonal cost √2 —
KiCad's 45° routing idiom) plus layer change (a via) at via-legal cells.
Multi-goal: the search stops at the nearest goal cell, which is how a net's
connection tree grows pin by pin (route to the nearest already-connected
copper). A result is a complete path or an explicit failure — there is no
partial-path return, which is what makes dangling stubs inexpressible.
"""

from __future__ import annotations

import heapq
import math
import time
from dataclasses import dataclass

from tracewise.route.engine.grid import Grid

SQRT2 = math.sqrt(2.0)
VIA_RING = 2  # via copper outsticks track copper; demand extra free ring (cells)


def _via_ok(grid: Grid, iy: int, ix: int) -> bool:
    """Via placement needs more room than a track centerline: the barrel
    radius exceeds the track halfwidth the cell grid budgets for."""
    for layer in range(grid.layers):
        for dy in range(-VIA_RING, VIA_RING + 1):
            for dx in range(-VIA_RING, VIA_RING + 1):
                if not grid.free(layer, iy + dy, ix + dx):
                    return False
    return True
DIRS = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
        (-1, -1, SQRT2), (-1, 1, SQRT2), (1, -1, SQRT2), (1, 1, SQRT2)]


@dataclass
class RouteResult:
    ok: bool
    path: list[tuple[int, int, int]]  # [(layer, iy, ix)]
    cost: float
    reason: str = ""
    escaped: frozenset = frozenset()  # nodes that traversed clearance halos


def route(
    grid: Grid,
    start: tuple[int, int, int],
    goals: set[tuple[int, int, int]],
    via_cost: float = 10.0,
    max_expansions: int = 600_000,
    escape: int = 0,
    escape_penalty: float = 4.0,
    max_seconds: float = 45.0,
) -> RouteResult:
    """A* from start to the nearest of `goals`. Cells in `goals` need not be
    free (pads are blocked for other nets but are this net's targets)."""
    if not goals:
        return RouteResult(False, [], 0.0, "no goals")
    for g in goals:  # tripwire for a rare unexplained corruption (2 sightings)
        if not (isinstance(g, tuple) and len(g) == 3):
            raise AssertionError(f"malformed goal {g!r} of {type(g)}; "
                                 f"start={start!r} ngoals={len(goals)}")
    sl, sy, sx = start
    if not grid.in_bounds(sy, sx):
        return RouteResult(False, [], 0.0, "start out of bounds")

    def h(node):  # admissible: octile distance to nearest goal, via cost if needed
        nl, ny, nx = node
        best = math.inf
        for gl, gy, gx in goals:
            dy, dx = abs(gy - ny), abs(gx - nx)
            d = (dy + dx) + (SQRT2 - 2) * min(dy, dx)
            if gl != nl:
                d += via_cost
            best = min(best, d)
        return best

    open_q: list[tuple[float, float, tuple[int, int, int]]] = [(h(start), 0.0, start)]
    came: dict[tuple[int, int, int], tuple[int, int, int] | None] = {start: None}
    gscore: dict[tuple[int, int, int], float] = {start: 0.0}
    shaved: set[tuple[int, int, int]] = set()
    expansions = 0
    deadline = time.monotonic() + max_seconds

    while open_q:
        _, g, node = heapq.heappop(open_q)
        if node in goals:
            path = []
            cur: tuple[int, int, int] | None = node
            while cur is not None:
                path.append(cur)
                cur = came[cur]
            path.reverse()
            return RouteResult(True, path, g,
                               escaped=frozenset(n for n in path if n in shaved))
        if g > gscore.get(node, math.inf):
            continue
        expansions += 1
        if expansions > max_expansions:
            return RouteResult(False, [], 0.0, "expansion budget exceeded")
        # cheap per-route wall-clock (checked every 100k expansions): a single
        # net exploring a large unreachable region must not run for minutes
        if expansions % 100_000 == 0 and time.monotonic() > deadline:
            return RouteResult(False, [], 0.0, "route time budget exceeded")
        nl, ny, nx = node

        # escape window is GEOMETRIC distance from the endpoints — measuring
        # by accumulated cost closes the window after ~4 cells once penalties
        # stack, which strands fine-pitch (QFN) pads inside their halo fields
        near_end = escape and (
            max(abs(ny - sy), abs(nx - sx)) < escape or h(node) < escape
        )

        def passable(layer, iy, ix, _near=near_end):
            if grid.free(layer, iy, ix):
                return 0.0
            if _near and grid.halo_only(layer, iy, ix):
                return escape_penalty
            return None

        neighbors = []
        for dy, dx, c in DIRS:
            iy, ix = ny + dy, nx + dx
            nxt = (nl, iy, ix)
            if nxt in goals:
                neighbors.append((nxt, c))
                continue
            pen = passable(nl, iy, ix)
            if pen is None:
                continue
            if dy and dx:  # no corner cutting: a 45° segment between free cells
                # still clips a blocked corner cell's halo in continuous geometry
                if passable(nl, ny, ix) is None or passable(nl, iy, nx) is None:
                    continue
            neighbors.append((nxt, c + pen))
        for layer in range(grid.layers):  # via: change layer in place
            if layer != nl:
                nxt = (layer, ny, nx)
                if nxt in goals or (grid.free(layer, ny, nx) and _via_ok(grid, ny, nx)):
                    neighbors.append((nxt, via_cost))

        for nxt, c in neighbors:
            ng = g + c
            if ng < gscore.get(nxt, math.inf):
                gscore[nxt] = ng
                came[nxt] = node
                if c > SQRT2:  # carried an escape penalty
                    shaved.add(nxt)
                else:
                    shaved.discard(nxt)
                heapq.heappush(open_q, (ng + h(nxt), ng, nxt))

    return RouteResult(False, [], 0.0, "no path")


def simplify(path: list[tuple[int, int, int]]) -> list[list[tuple[int, int, int]]]:
    """Split at layer changes and merge collinear runs. Returns per-layer
    polyline segments: each item is a list of waypoints on one layer; vias are
    implied between consecutive items."""
    if not path:
        return []
    runs: list[list[tuple[int, int, int]]] = [[path[0]]]
    for node in path[1:]:
        if node[0] != runs[-1][-1][0]:
            runs.append([node])
        else:
            runs[-1].append(node)
    out = []
    for run in runs:
        if len(run) <= 2:
            out.append(run)
            continue
        pts = [run[0]]
        for i in range(1, len(run) - 1):
            d1 = (run[i][1] - pts[-1][1], run[i][2] - pts[-1][2])
            d2 = (run[i + 1][1] - run[i][1], run[i + 1][2] - run[i][2])
            # keep the point unless direction is unchanged
            n1 = max(abs(d1[0]), abs(d1[1])) or 1
            n2 = max(abs(d2[0]), abs(d2[1])) or 1
            if (d1[0] * n2, d1[1] * n2) != (d2[0] * n1, d2[1] * n1):
                pts.append(run[i])
        pts.append(run[-1])
        out.append(pts)
    return out
