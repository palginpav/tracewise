"""geom — Shapely-based free-space construction for the FAR gridless router.

Responsibilities:
  - Guard the Shapely import behind HAVE_SHAPELY so the package loads even without
    shapely installed; functions raise ImportError with an actionable message if called
    when shapely is absent.
  - Build inflated obstacle polygons from pad data within a routing window.
  - Apply ``set_precision(1e-6)`` (1 nm) to every geometry for run-to-run determinism.
  - Carve the routing net's OWN pad rects out of the accumulated obstacle union before
    computing the free-space difference (the "own-pad carve-out" requirement, FIX-2 from
    the M1 scale spike).

All coordinates are in mm (x, y).  The routing window is an (x1, y1, x2, y2) tuple.
"""

from __future__ import annotations

# ---------------------------------------------------------------------------
# Shapely import guard
# ---------------------------------------------------------------------------

_SHAPELY_REQUIRED_MSG = (
    "shapely>=2.0 required for the gridless router; pip install tracewise[gridless]"
)

try:
    import shapely
    from shapely import STRtree, set_precision  # noqa: F401 — re-exported for search.py
    from shapely.geometry import Point, box
    from shapely.ops import unary_union

    HAVE_SHAPELY: bool = shapely.geos_version >= (3, 8, 0)
except ImportError:
    HAVE_SHAPELY = False

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

PRECISION: float = 1e-6  # 1 nm — quantise all Shapely geometry for determinism


# ---------------------------------------------------------------------------
# Internal helpers
# ---------------------------------------------------------------------------


def _require_shapely() -> None:
    if not HAVE_SHAPELY:
        raise ImportError(_SHAPELY_REQUIRED_MSG)


def snap(geom: object) -> object:
    """Quantise *geom* to the PRECISION grid for determinism.

    Wraps ``shapely.set_precision(geom, PRECISION)``.
    """
    _require_shapely()
    return set_precision(geom, PRECISION)  # type: ignore[return-value]


# ---------------------------------------------------------------------------
# Free-space construction
# ---------------------------------------------------------------------------


def build_windowed_free_space(
    pads: list[dict],
    net_name: str,
    clearance_mm: float,
    track_mm: float,
    extra_obstacles: list,
    window_bbox: tuple[float, float, float, float],
) -> tuple[object, list]:
    """Build the free-space polygon within *window_bbox* for routing *net_name*.

    Algorithm
    ---------
    1. Collect all other-net F.Cu pad rectangles whose bounding boxes intersect
       the window.  Inflate each by ``clearance_mm + track_mm / 2`` (Minkowski
       half-width: keeps the track *centreline* at ≥ *clearance_mm* from any
       copper edge).
    2. Add any pre-inflated *extra_obstacles* (buffered centrelines of
       already-routed nets) that intersect the window.
    3. Compute the union of all inflated obstacles, then **carve the routing
       net's own pad rects out** of that union (own-pad carve-out, FIX-2): this
       ensures the start/goal pad centres remain inside free space even after
       adjacent nets' copper buffers overlap them.
    4. Subtract the trimmed union from the window polygon, snap to PRECISION.

    Parameters
    ----------
    pads:
        All board pads from ``extract_pads`` (any format with keys ``net``,
        ``x``, ``y``, ``hw``, ``hh``, ``front``).
    net_name:
        The net being routed; its pads are excluded from obstacles and carved
        back into free space.
    clearance_mm:
        Required copper-to-copper clearance.
    track_mm:
        Track width (the Minkowski inflate = clearance + track/2).
    extra_obstacles:
        Shapely polygons already inflated by ``track_mm + clearance_mm`` (the
        correct inflate-for-track-to-track formula, FIX-6) representing
        already-routed copper.  Caller is responsible for the inflation.
    window_bbox:
        ``(x1, y1, x2, y2)`` routing window.

    Returns
    -------
    ``(free_space, obstacle_polys)``
        *free_space* is a Shapely geometry (Polygon or MultiPolygon).
        *obstacle_polys* is the list of inflated obstacle polygons used to
        build the STRtree in ``search.py``.
    """
    _require_shapely()

    inflate = clearance_mm + track_mm / 2.0
    wx1, wy1, wx2, wy2 = window_bbox
    window_poly = snap(box(wx1, wy1, wx2, wy2))

    obstacle_polys: list = []
    for p in pads:
        if p["net"] == net_name:
            continue
        if not p.get("front"):
            continue
        # Reject pads that don't intersect the window at all
        px1 = p["x"] - p["hw"]
        py1 = p["y"] - p["hh"]
        px2 = p["x"] + p["hw"]
        py2 = p["y"] + p["hh"]
        if px2 < wx1 or px1 > wx2 or py2 < wy1 or py1 > wy2:
            continue
        rect = box(px1, py1, px2, py2)
        inflated = snap(rect.buffer(inflate, cap_style=3, join_style=2))
        obstacle_polys.append(inflated)

    # Add already-routed copper (pre-inflated by caller)
    for obs in extra_obstacles:
        if obs.intersects(window_poly):
            obstacle_polys.append(obs)

    if obstacle_polys:
        union = snap(unary_union(obstacle_polys))

        # Own-pad carve-out (FIX-2): routing net's own pads must remain reachable
        # even if adjacent routed copper overlaps them.
        own_pads = [p for p in pads if p["net"] == net_name and p.get("front")]
        if own_pads:
            own_pad_polys = [
                box(
                    p["x"] - p["hw"],
                    p["y"] - p["hh"],
                    p["x"] + p["hw"],
                    p["y"] + p["hh"],
                )
                for p in own_pads
            ]
            own_union = snap(unary_union(own_pad_polys))
            union = snap(union.difference(own_union))

        free_space = snap(window_poly.difference(union))
    else:
        obstacle_polys = []
        free_space = window_poly

    return free_space, obstacle_polys


def get_component_containing(free_space: object, pt: tuple[float, float]) -> object:
    """Return the free-space polygon component that contains *pt*.

    If *free_space* is a ``MultiPolygon``, returns the component for which
    ``contains(pt)`` is True, or the one closest to *pt* if none contains it.
    If *free_space* is a plain ``Polygon``, returns it directly.
    """
    _require_shapely()
    sp = Point(*pt)
    fs = free_space  # type: ignore[assignment]
    if fs.geom_type == "MultiPolygon":
        for comp in fs.geoms:
            if comp.contains(sp) or comp.distance(sp) < 1e-6:
                return comp
        # Fallback: largest component
        return max(fs.geoms, key=lambda g: g.area)
    return fs
