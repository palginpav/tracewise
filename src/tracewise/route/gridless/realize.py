"""realize — path-to-centerline conversion with per-segment legality assertion.

Converts a raw waypoint list from the A* search into a clean world-mm centreline:
  1. Quantise every coordinate to PRECISION (1 nm) for determinism.
  2. Remove consecutive near-duplicate points (distance < 1e-9 mm).
  3. Assert each segment lies within the free-space polygon (Shapely contains oracle,
     FIX-1 from the M1 scale spike) before returning.

No funnel step is needed: the visibility-graph path already is the shortest centreline.
"""

from __future__ import annotations

import math

from tracewise.route.gridless.geom import HAVE_SHAPELY, PRECISION, _require_shapely

if HAVE_SHAPELY:
    from shapely.geometry import LineString, Point


# ---------------------------------------------------------------------------
# Quantise + dedup
# ---------------------------------------------------------------------------


def snap_waypoints(
    waypoints: list[tuple[float, float]],
) -> list[tuple[float, float]]:
    """Quantise waypoints to PRECISION and remove consecutive near-duplicates.

    Parameters
    ----------
    waypoints:
        Raw (x, y) waypoint list from A*.

    Returns
    -------
    Cleaned list with at least 2 points (or the input if it was already short).
    """
    # Quantise to PRECISION grid
    snapped = [
        (
            round(x / PRECISION) * PRECISION,
            round(y / PRECISION) * PRECISION,
        )
        for x, y in waypoints
    ]

    # Remove consecutive near-duplicates
    result: list[tuple[float, float]] = [snapped[0]]
    for p in snapped[1:]:
        if math.hypot(p[0] - result[-1][0], p[1] - result[-1][1]) > 1e-9:
            result.append(p)

    return result


# ---------------------------------------------------------------------------
# Segment legality assertion
# ---------------------------------------------------------------------------


def assert_legal_segment(
    wa: tuple[float, float],
    wb: tuple[float, float],
    free_space: object,
) -> None:
    """Raise ``ValueError`` if segment *wa*→*wb* is not contained in *free_space*.

    Uses a 1e-5 mm buffer around the free-space boundary to tolerate boundary-grazing
    paths (a centreline touching an inflated obstacle corner is legal).

    Parameters
    ----------
    wa, wb:
        Segment endpoints in world mm.
    free_space:
        Shapely Polygon / MultiPolygon built for this net's routing window.

    Raises
    ------
    ValueError
        If the segment has more than 1e-6 mm of centreline outside *free_space*.
    """
    _require_shapely()
    fs_buf = free_space.buffer(1e-5)  # type: ignore[union-attr]
    seg = LineString([wa, wb])
    outside = seg.difference(fs_buf)
    if not outside.is_empty and outside.length > 1e-6:
        raise ValueError(
            f"Illegal segment ({wa[0]:.6f},{wa[1]:.6f})"
            f"→({wb[0]:.6f},{wb[1]:.6f}): "
            f"{outside.length:.6f} mm outside free_space"
        )


def assert_legal_point(
    pt: tuple[float, float],
    free_space: object,
) -> None:
    """Raise ``ValueError`` if *pt* is not inside *free_space* (with 1e-5 mm tolerance)."""
    _require_shapely()
    if not free_space.buffer(1e-5).contains(Point(*pt)):  # type: ignore[union-attr]
        dist = free_space.distance(Point(*pt))  # type: ignore[union-attr]
        raise ValueError(
            f"Waypoint ({pt[0]:.6f},{pt[1]:.6f}) outside free_space "
            f"(dist={dist:.6f} mm)"
        )


# ---------------------------------------------------------------------------
# Realize centreline
# ---------------------------------------------------------------------------


def realize_centerline(
    path: list[tuple[float, float]],
    free_space: object,
) -> list[tuple[float, float]]:
    """Convert a raw A* path to a legal world-mm centreline.

    Steps:
      1. Snap waypoints to PRECISION; dedup consecutive near-duplicates.
      2. Assert each waypoint is inside *free_space*.
      3. Assert each segment lies within *free_space*.

    Parameters
    ----------
    path:
        Raw waypoints from A* (start → [corners] → goal).
    free_space:
        Shapely geometry for the routing window used to find this path.

    Returns
    -------
    List of ``(x_mm, y_mm)`` waypoints, ready to emit as track segments.

    Raises
    ------
    ValueError
        If any waypoint or segment is outside the free space.
    RuntimeError
        If *path* has fewer than 2 points after deduplication.
    """
    _require_shapely()

    waypoints = snap_waypoints(path)

    if len(waypoints) < 2:
        raise RuntimeError(
            f"realize_centerline: path has only {len(waypoints)} unique waypoint(s) "
            "after deduplication — cannot form a segment"
        )

    # Assert each waypoint is inside free space
    for pt in waypoints:
        assert_legal_point(pt, free_space)

    # Assert each segment
    for wa, wb in zip(waypoints, waypoints[1:], strict=False):
        assert_legal_segment(wa, wb, free_space)

    return waypoints
