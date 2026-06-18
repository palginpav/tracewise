"""exact_geom — Phase A: exact-geometry primitive layer for the #4 NEAR build.

This module implements the pure-numpy, deterministic geometry primitives used by
the Minkowski-endpoint exact-geometry emitter (Phase B, emit refactor) and the
future gridless router (FAR build).

All world coordinates are **(x, y) in mm** — x is horizontal, y is vertical,
matching the convention of Grid.to_world(iy, ix) → (x, y).  The Grid class
internally uses (iy, ix) = (row, col) for cell indexing, but that row/col
representation never enters this module; callers must convert beforehand.

Obstacle tagged-union (plain Python tuples for zero-dependency serialisation):
  ("circle",  cx, cy, r)           — via copper / round pad (half-size = r)
  ("rect",    x1, y1, x2, y2)      — rectangular pad, x1≤x2, y1≤y2
  ("segment", ax, ay, bx, by, hw)  — existing track copper; hw = half-width

For a "circle" obstacle the copper edge is the circle of radius r centred at
(cx, cy).  For a "rect" the copper edge is the rectangle boundary.  For a
"segment" the copper edge is the Minkowski sum of the segment with a disc of
radius hw (i.e. every point within hw of the centreline segment).

``nudge_endpoint`` algorithm
-----------------------------
1. Analytic push-out: find the closest obstacle that violates the required
   clearance and move the point directly away from it by the deficit plus a
   tiny epsilon so the new point is strictly legal.
2. Validation: re-check ``min_clearance`` after the analytic step; also check
   the anchor-distance constraint and the max-nudge radius.
3. Refinement on failure: if the analytic step does not yield a fully legal
   position (another obstacle still violated, or the constraints were breached),
   fall back to a fixed deterministic polar grid:
     POLAR_ANGLES × POLAR_RADII candidate offsets from ``endpoint``
   The candidates are tried in (radius, angle) order so the search is
   reproducible.  The first candidate that satisfies all constraints (clearance
   ≥ required, distance from endpoint ≤ max_nudge, and — if anchor given —
   distance from anchor ≤ original distance from anchor) is returned.
4. If no candidate is found, return (endpoint, False).

No randomness anywhere — all sampling uses fixed, documented constants.
"""

from __future__ import annotations

import math
from typing import Union

import numpy as np

# ---------------------------------------------------------------------------
# Polar refinement grid constants (fixed, documented)
# ---------------------------------------------------------------------------

# 16 uniformly spaced angles in [0, 2π)
POLAR_ANGLES: tuple[float, ...] = tuple(
    2.0 * math.pi * k / 16 for k in range(16)
)

# 6 radii from fine to max_nudge-aligned; caller may override max_nudge but
# the search always uses these fractions of max_nudge.
_POLAR_RADIUS_FRACTIONS: tuple[float, ...] = (
    1 / 6, 2 / 6, 3 / 6, 4 / 6, 5 / 6, 1.0
)

# ---------------------------------------------------------------------------
# Type aliases
# ---------------------------------------------------------------------------

Point = Union[tuple[float, float], "np.ndarray"]
Obstacle = tuple  # tagged union described in module docstring

# ---------------------------------------------------------------------------
# Distance primitives
# ---------------------------------------------------------------------------


def segment_point_distance(
    a: Point,
    b: Point,
    p: Point,
) -> float:
    """Minimum distance from point *p* to segment *a*–*b*.

    Parameters
    ----------
    a, b:
        Segment endpoints as (x, y).
    p:
        Query point as (x, y).

    Returns
    -------
    float
        Euclidean distance.  Always ≥ 0.  Degeneracy: if a == b the segment
        collapses to a point and the distance is simply dist(p, a).
    """
    ax, ay = float(a[0]), float(a[1])
    bx, by = float(b[0]), float(b[1])
    px, py = float(p[0]), float(p[1])

    dx = bx - ax
    dy = by - ay
    len_sq = dx * dx + dy * dy

    if len_sq < 1e-18:
        # Degenerate: segment is a point
        return math.hypot(px - ax, py - ay)

    # Parameter t ∈ [0, 1] of closest point on segment to p
    t = ((px - ax) * dx + (py - ay) * dy) / len_sq
    t = max(0.0, min(1.0, t))
    qx = ax + t * dx
    qy = ay + t * dy
    return math.hypot(px - qx, py - qy)


def segment_segment_distance(
    a: Point,
    b: Point,
    c: Point,
    d: Point,
) -> float:
    """Minimum distance between segment *a*–*b* and segment *c*–*d*.

    Returns 0 if the segments intersect or overlap.
    """
    ax, ay = float(a[0]), float(a[1])
    bx, by = float(b[0]), float(b[1])
    cx, cy = float(c[0]), float(c[1])
    dx, dy = float(d[0]), float(d[1])

    # Intersection test via cross product
    abx, aby = bx - ax, by - ay
    cdx, cdy = dx - cx, dy - cy
    acx, acy = cx - ax, cy - ay

    denom = abx * cdy - aby * cdx
    if abs(denom) > 1e-14:
        t = (acx * cdy - acy * cdx) / denom
        u = (acx * aby - acy * abx) / denom
        if 0.0 <= t <= 1.0 and 0.0 <= u <= 1.0:
            return 0.0  # segments intersect

    # Otherwise: minimum of four endpoint-to-segment distances
    d1 = segment_point_distance(a, b, c)
    d2 = segment_point_distance(a, b, d)
    d3 = segment_point_distance(c, d, a)
    d4 = segment_point_distance(c, d, b)
    return min(d1, d2, d3, d4)


def point_rect_distance(p: Point, rect: tuple[float, float, float, float]) -> float:
    """Minimum distance from point *p* to axis-aligned rectangle *rect*.

    Parameters
    ----------
    p:
        Query point (x, y).
    rect:
        (x1, y1, x2, y2) where x1 ≤ x2 and y1 ≤ y2.

    Returns
    -------
    float
        0 if *p* is inside or on the boundary of *rect*.
    """
    px, py = float(p[0]), float(p[1])
    x1, y1, x2, y2 = float(rect[0]), float(rect[1]), float(rect[2]), float(rect[3])
    # Normalise in case caller passes x1>x2 or y1>y2
    rx1, rx2 = min(x1, x2), max(x1, x2)
    ry1, ry2 = min(y1, y2), max(y1, y2)
    dx = max(rx1 - px, 0.0, px - rx2)
    dy = max(ry1 - py, 0.0, py - ry2)
    return math.hypot(dx, dy)


def segment_rect_distance(
    a: Point,
    b: Point,
    rect: tuple[float, float, float, float],
) -> float:
    """Minimum distance between segment *a*–*b* and axis-aligned rectangle *rect*.

    Returns 0 if the segment intersects or is inside the rectangle.
    """
    x1, y1, x2, y2 = float(rect[0]), float(rect[1]), float(rect[2]), float(rect[3])
    rx1, rx2 = min(x1, x2), max(x1, x2)
    ry1, ry2 = min(y1, y2), max(y1, y2)

    # If either endpoint is inside the rect → 0
    ax, ay = float(a[0]), float(a[1])
    bx, by = float(b[0]), float(b[1])
    if rx1 <= ax <= rx2 and ry1 <= ay <= ry2:
        return 0.0
    if rx1 <= bx <= rx2 and ry1 <= by <= ry2:
        return 0.0

    # Check segment vs each of the 4 rect edges
    corners = [
        ((rx1, ry1), (rx2, ry1)),  # bottom edge
        ((rx2, ry1), (rx2, ry2)),  # right edge
        ((rx2, ry2), (rx1, ry2)),  # top edge
        ((rx1, ry2), (rx1, ry1)),  # left edge
    ]
    best = min(segment_segment_distance(a, b, e0, e1) for e0, e1 in corners)
    if best == 0.0:
        return 0.0

    # Also consider endpoint-to-rect (handles segment entirely outside the rect
    # without crossing any edge)
    ep_a = point_rect_distance(a, (rx1, ry1, rx2, ry2))
    ep_b = point_rect_distance(b, (rx1, ry1, rx2, ry2))
    return min(best, ep_a, ep_b)


# ---------------------------------------------------------------------------
# Obstacle + clearance model
# ---------------------------------------------------------------------------


def _raw_distance_to_obstacle(point: Point, obstacle: Obstacle) -> float:
    """Distance from *point* (x, y) to the **copper edge** of *obstacle*.

    For "circle": distance from point to circle boundary = dist(point, centre) − r.
    For "rect":   point_rect_distance to the rectangle boundary.
    For "segment": segment_point_distance to the centreline (the copper edge is
                   hw away from the centreline, so copper distance = that − hw).

    The raw value is the edge-to-centre-of-track distance; subtract track_hw to
    get edge-to-edge clearance.  Negative means inside the copper (overlap).
    """
    kind = obstacle[0]
    px, py = float(point[0]), float(point[1])

    if kind == "circle":
        _, cx, cy, r = obstacle
        dist_centre = math.hypot(px - cx, py - cy)
        return dist_centre - r

    if kind == "rect":
        _, x1, y1, x2, y2 = obstacle
        return point_rect_distance(point, (x1, y1, x2, y2))

    if kind == "segment":
        _, ax, ay, bx, by, hw = obstacle
        dist_centre = segment_point_distance((ax, ay), (bx, by), point)
        return dist_centre - hw

    raise ValueError(f"Unknown obstacle kind: {kind!r}")


def clearance_to_obstacle(
    point: Point,
    obstacle: Obstacle,
    track_hw: float,
) -> float:
    """Edge-to-edge clearance from a track/via of half-width *track_hw* to *obstacle*.

    Returns the minimum gap between the track copper and the obstacle copper.
    Negative values indicate overlap (DRC violation).
    """
    return _raw_distance_to_obstacle(point, obstacle) - track_hw


def min_clearance(
    point: Point,
    obstacles: list[Obstacle],
    track_hw: float,
) -> float:
    """Minimum clearance over all *obstacles*.

    Returns +inf if *obstacles* is empty.
    """
    if not obstacles:
        return math.inf
    return min(clearance_to_obstacle(point, obs, track_hw) for obs in obstacles)


def is_legal(
    point: Point,
    obstacles: list[Obstacle],
    track_hw: float,
    required_clearance: float,
) -> bool:
    """True iff ``min_clearance(point, obstacles, track_hw) >= required_clearance``."""
    return min_clearance(point, obstacles, track_hw) >= required_clearance


# ---------------------------------------------------------------------------
# Minkowski endpoint placement
# ---------------------------------------------------------------------------

_ANALYTIC_EPSILON = 1e-4  # tiny push beyond the clearance deficit


def nudge_endpoint(
    endpoint: Point,
    anchor: Point | None,
    obstacles: list[Obstacle],
    required_clearance: float,
    track_hw: float,
    max_nudge: float = 0.3,
) -> tuple[tuple[float, float], bool]:
    """Find the nearest legal position to *endpoint* subject to clearance constraints.

    Algorithm (deterministic, no randomness):

    1. **Already legal?** If ``min_clearance(endpoint, …) ≥ required_clearance``
       AND any anchor constraint is satisfied, return ``(endpoint, True)``.

    2. **Analytic push-out.** Find the obstacle with the worst (most negative)
       clearance.  Push *endpoint* directly away from that obstacle's
       representative point (nearest point on the copper edge) by the clearance
       deficit + ``_ANALYTIC_EPSILON``.  Check all constraints on the result.
       If satisfied, return immediately.

    3. **Polar-grid refinement.** Scan a fixed deterministic grid:
       ``POLAR_ANGLES × (max_nudge × fraction for fraction in _POLAR_RADIUS_FRACTIONS)``
       offsets from *endpoint*.  First candidate that satisfies all constraints
       is returned.

    4. Failure → return ``(endpoint, False)``.

    Constraints:
    - ``min_clearance(pos, obstacles, track_hw) >= required_clearance``
    - ``dist(pos, endpoint) <= max_nudge``
    - If *anchor* is not None: ``dist(pos, anchor) <= dist(endpoint, anchor)``
      (the track still reaches the pad; it may not move farther from the pad than
      the original endpoint position).

    Parameters
    ----------
    endpoint:
        Original track/via endpoint (x, y).
    anchor:
        The pad/via centre this endpoint must remain attached to, or None.
    obstacles:
        List of obstacle tagged-tuples.
    required_clearance:
        Minimum legal edge-to-edge clearance (mm).
    track_hw:
        Half-width of the track/via being placed (mm).
    max_nudge:
        Maximum displacement from *endpoint* (mm).

    Returns
    -------
    (new_xy, ok)
        *new_xy* is ``endpoint`` if ``ok=False``.
    """
    ex, ey = float(endpoint[0]), float(endpoint[1])

    # Anchor distance bound
    anchor_dist_max: float = math.inf
    if anchor is not None:
        anchor_dist_max = math.hypot(ex - float(anchor[0]), ey - float(anchor[1]))

    def _satisfies(px: float, py: float) -> bool:
        if math.hypot(px - ex, py - ey) > max_nudge + 1e-9:
            return False
        if math.hypot(px - float(anchor[0]), py - float(anchor[1])) > anchor_dist_max + 1e-9 \
                if anchor is not None else False:
            return False
        return min_clearance((px, py), obstacles, track_hw) >= required_clearance - 1e-9

    # Step 1: already legal?
    if _satisfies(ex, ey):
        return ((ex, ey), True)

    # Step 2: analytic push-out from the worst-violating obstacle
    if obstacles:
        worst_clearance = math.inf
        worst_obs = None
        for obs in obstacles:
            cl = clearance_to_obstacle((ex, ey), obs, track_hw)
            if cl < worst_clearance:
                worst_clearance = cl
                worst_obs = obs

        if worst_obs is not None and worst_clearance < required_clearance:
            # Find the nearest point on the obstacle copper edge (the push direction)
            push_x, push_y = _nearest_copper_point(ex, ey, worst_obs)
            vec_x = ex - push_x
            vec_y = ey - push_y
            vec_len = math.hypot(vec_x, vec_y)
            if vec_len > 1e-12:
                # Deficit: how much more clearance we need (positive means violation)
                deficit = required_clearance - worst_clearance  # = required - current
                # Move endpoint away from obstacle copper by deficit + epsilon
                step = deficit + _ANALYTIC_EPSILON
                nx = ex + (vec_x / vec_len) * step
                ny = ey + (vec_y / vec_len) * step
                if _satisfies(nx, ny):
                    return ((nx, ny), True)

    # Step 3: polar-grid refinement
    for frac in _POLAR_RADIUS_FRACTIONS:
        r = max_nudge * frac
        for theta in POLAR_ANGLES:
            nx = ex + r * math.cos(theta)
            ny = ey + r * math.sin(theta)
            if _satisfies(nx, ny):
                return ((nx, ny), True)

    # Also try the original endpoint once more (satisfies-check is idempotent, but
    # the _satisfies helper is reachable even after the analytic step fails)
    return ((ex, ey), False)


# ---------------------------------------------------------------------------
# Internal helpers (not part of the public API)
# ---------------------------------------------------------------------------


def _nearest_copper_point(px: float, py: float, obstacle: Obstacle) -> tuple[float, float]:
    """Return the nearest point on the **copper edge** of *obstacle* to (px, py).

    Used to compute the push-out direction for the analytic nudge step.
    """
    kind = obstacle[0]

    if kind == "circle":
        _, cx, cy, r = obstacle
        dist = math.hypot(px - cx, py - cy)
        if dist < 1e-12:
            # Point is at the centre — arbitrary push direction (rightward)
            return (cx + r, cy)
        # Nearest point on circle boundary
        nx = cx + r * (px - cx) / dist
        ny = cy + r * (py - cy) / dist
        return (nx, ny)

    if kind == "rect":
        _, x1, y1, x2, y2 = obstacle
        rx1, rx2 = min(x1, x2), max(x1, x2)
        ry1, ry2 = min(y1, y2), max(y1, y2)
        # Clamp to rectangle boundary
        clamp_x = max(rx1, min(px, rx2))
        clamp_y = max(ry1, min(py, ry2))
        if rx1 <= px <= rx2 and ry1 <= py <= ry2:
            # Inside: find nearest edge
            d_left = px - rx1
            d_right = rx2 - px
            d_bottom = py - ry1
            d_top = ry2 - py
            mind = min(d_left, d_right, d_bottom, d_top)
            if mind == d_left:
                return (rx1, py)
            if mind == d_right:
                return (rx2, py)
            if mind == d_bottom:
                return (px, ry1)
            return (px, ry2)
        return (clamp_x, clamp_y)

    if kind == "segment":
        _, ax, ay, bx, by, hw = obstacle
        # Nearest point on the centreline segment
        dx = bx - ax
        dy = by - ay
        len_sq = dx * dx + dy * dy
        if len_sq < 1e-18:
            # Degenerate segment
            cx, cy = ax, ay
        else:
            t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / len_sq))
            cx = ax + t * dx
            cy = ay + t * dy
        # Nearest point on the copper edge = centre + hw in direction of (p - centre)
        dist = math.hypot(px - cx, py - cy)
        if dist < 1e-12:
            return (cx + hw, cy)
        nx = cx + hw * (px - cx) / dist
        ny = cy + hw * (py - cy) / dist
        return (nx, ny)

    raise ValueError(f"Unknown obstacle kind: {kind!r}")


# ---------------------------------------------------------------------------
# Region-constrained endpoint nudge (terminal-on-pad case)
# ---------------------------------------------------------------------------


def nudge_endpoint_in_region(
    start: Point,
    allowed_rect: tuple[float, float, float, float],
    obstacles: list[Obstacle],
    required_clearance: float,
    track_hw: float,
) -> tuple[tuple[float, float], bool]:
    """Find a position INSIDE allowed_rect that maximises clearance to obstacles.

    Connectivity model: the point represents a track terminal sitting on its own
    pad; it stays electrically connected as long as it remains inside allowed_rect
    (the pad copper). Within that rect it is free to move to gain clearance to
    OTHER-net copper.

    Algorithm (deterministic, mirrors nudge_endpoint):
      1. If start already legal (min_clearance(start) >= required) -> (start, True).
      2. Analytic push-out from the worst obstacle, THEN clamp the candidate back
         into allowed_rect (clamp x to [x1,x2], y to [y1,y2]); accept if legal.
      3. Polar-grid refinement: same POLAR_ANGLES x _POLAR_RADIUS_FRACTIONS grid
         centred on start, but each candidate is CLAMPED into allowed_rect before
         the legality test; first legal clamped candidate wins. (Clamping, not
         rejection, so a candidate that points outside the pad still contributes
         its in-pad projection — important for thin pads.)
      4. Failure -> (start, False).   start is always inside allowed_rect by
         construction, so the failure case is connectivity-safe.

    No max_nudge parameter: the pad rect IS the bound (a pad is small; the rect
    constrains displacement more tightly than 0.3mm would).
    """
    ex, ey = float(start[0]), float(start[1])
    x1, y1, x2, y2 = (float(allowed_rect[0]), float(allowed_rect[1]),
                       float(allowed_rect[2]), float(allowed_rect[3]))
    # Normalise rect in case caller passes inverted coords
    rx1, rx2 = min(x1, x2), max(x1, x2)
    ry1, ry2 = min(y1, y2), max(y1, y2)

    def _clamp(px: float, py: float) -> tuple[float, float]:
        return (max(rx1, min(px, rx2)), max(ry1, min(py, ry2)))

    def _satisfies(px: float, py: float) -> bool:
        return min_clearance((px, py), obstacles, track_hw) >= required_clearance - 1e-9

    # Step 1: already legal?
    if _satisfies(ex, ey):
        return ((ex, ey), True)

    # Step 2: analytic push-out from the worst-violating obstacle, then clamp
    if obstacles:
        worst_clearance = math.inf
        worst_obs = None
        for obs in obstacles:
            cl = clearance_to_obstacle((ex, ey), obs, track_hw)
            if cl < worst_clearance:
                worst_clearance = cl
                worst_obs = obs

        if worst_obs is not None and worst_clearance < required_clearance:
            push_x, push_y = _nearest_copper_point(ex, ey, worst_obs)
            vec_x = ex - push_x
            vec_y = ey - push_y
            vec_len = math.hypot(vec_x, vec_y)
            if vec_len > 1e-12:
                deficit = required_clearance - worst_clearance
                step = deficit + _ANALYTIC_EPSILON
                nx = ex + (vec_x / vec_len) * step
                ny = ey + (vec_y / vec_len) * step
                nx, ny = _clamp(nx, ny)
                if _satisfies(nx, ny):
                    return ((nx, ny), True)

    # Step 3: polar-grid refinement (clamp each candidate into rect before check)
    # Max displacement = largest half-size of the rect so the grid covers the whole pad
    half_w = (rx2 - rx1) / 2.0
    half_h = (ry2 - ry1) / 2.0
    max_r = math.hypot(half_w, half_h) if (half_w > 0 or half_h > 0) else 0.3
    for frac in _POLAR_RADIUS_FRACTIONS:
        r = max_r * frac
        for theta in POLAR_ANGLES:
            raw_x = ex + r * math.cos(theta)
            raw_y = ey + r * math.sin(theta)
            nx, ny = _clamp(raw_x, raw_y)
            if _satisfies(nx, ny):
                return ((nx, ny), True)

    # Step 4: failure — return start (clamped, for safety; start should already be in rect)
    sx, sy = _clamp(ex, ey)
    return ((sx, sy), False)
