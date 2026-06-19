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
  - Board edge: require free space to be INSIDE the board outline shrunk inward by
    ``clearance_mm + track_mm/2`` so track centrelines stay legal-distance from edges.
  - Drill holes: circular obstacles around through-hole pad drills and via drills.

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
# Board outline + drill extraction helpers
# ---------------------------------------------------------------------------


def extract_board_outline(board_path: object) -> object | None:
    """Extract the board outline polygon from the Edge.Cuts layer.

    Parses ``board_path`` (a path-like or str) via the sexpr parser and
    collects all ``gr_line`` / ``gr_arc`` / ``gr_rect`` / ``gr_poly`` items on
    ``Edge.Cuts``.  For simple rectangular boards (the common case) this
    assembles the 4 line endpoints into a closed polygon.

    Returns a Shapely Polygon of the board interior, or ``None`` if extraction
    fails (e.g. complex curved outlines — caller falls back to board_bbox).
    """
    if not HAVE_SHAPELY:
        return None
    try:
        from shapely.geometry import Polygon

        from tracewise.sexpr import parse_file

        root = parse_file(board_path)

        # Collect line segments on Edge.Cuts
        segments: list[tuple[tuple[float, float], tuple[float, float]]] = []
        for item in root.find_all("gr_line"):
            layer_node = item.first("layer")
            if layer_node is None:
                continue
            layer_name = layer_node.arg() or ""
            if "Edge.Cuts" not in layer_name:
                continue
            start_node = item.first("start")
            end_node = item.first("end")
            if start_node is None or end_node is None:
                continue
            try:
                x1 = float(start_node.arg(1))
                y1 = float(start_node.arg(2))
                x2 = float(end_node.arg(1))
                y2 = float(end_node.arg(2))
            except (TypeError, ValueError):
                continue
            segments.append(((x1, y1), (x2, y2)))

        # Also handle gr_rect on Edge.Cuts
        for item in root.find_all("gr_rect"):
            layer_node = item.first("layer")
            if layer_node is None:
                continue
            layer_name = layer_node.arg() or ""
            if "Edge.Cuts" not in layer_name:
                continue
            start_node = item.first("start")
            end_node = item.first("end")
            if start_node is None or end_node is None:
                continue
            try:
                x1 = float(start_node.arg(1))
                y1 = float(start_node.arg(2))
                x2 = float(end_node.arg(1))
                y2 = float(end_node.arg(2))
            except (TypeError, ValueError):
                continue
            # Convert rect to 4 segments
            segments.append(((x1, y1), (x2, y1)))
            segments.append(((x2, y1), (x2, y2)))
            segments.append(((x2, y2), (x1, y2)))
            segments.append(((x1, y2), (x1, y1)))

        # Also handle gr_poly on Edge.Cuts
        for item in root.find_all("gr_poly"):
            layer_node = item.first("layer")
            if layer_node is None:
                continue
            layer_name = layer_node.arg() or ""
            if "Edge.Cuts" not in layer_name:
                continue
            poly_node = item.first("pts")
            if poly_node is None:
                poly_node = item  # some formats inline pts
            pts = []
            for xy in item.find_all("xy"):
                try:
                    px = float(xy.arg(1))
                    py = float(xy.arg(2))
                    pts.append((px, py))
                except (TypeError, ValueError):
                    pass
            if len(pts) >= 3:
                try:
                    poly = snap(Polygon(pts))
                    return poly
                except Exception:  # noqa: BLE001
                    pass

        if not segments:
            return None

        # Attempt to chain segments into a closed polygon
        # Build endpoint adjacency graph and walk a cycle
        def _approx_eq(p1: tuple, p2: tuple, tol: float = 0.01) -> bool:
            return abs(p1[0] - p2[0]) < tol and abs(p1[1] - p2[1]) < tol

        # Try simple rectangular case: collect unique vertices
        all_pts: list[tuple[float, float]] = []
        seen: set[tuple[float, float]] = set()
        for (p1, p2) in segments:
            for p in (p1, p2):
                rp = (round(p[0], 6), round(p[1], 6))
                if rp not in seen:
                    seen.add(rp)
                    all_pts.append(rp)

        # Sort segments into a walk
        if len(segments) < 3:
            return None

        walked: list[tuple[float, float]] = []
        remaining = list(segments)
        current_pt = remaining[0][0]
        walked.append(current_pt)
        remaining = remaining[1:]
        # Start from the first segment's first point
        current_pt = segments[0][1]
        walked.append(current_pt)
        remaining = list(segments[1:])

        for _ in range(len(segments) + 1):
            if not remaining:
                break
            found = False
            for i, (p1, p2) in enumerate(remaining):
                if _approx_eq(p1, current_pt):
                    current_pt = p2
                    walked.append(current_pt)
                    remaining.pop(i)
                    found = True
                    break
                elif _approx_eq(p2, current_pt):
                    current_pt = p1
                    walked.append(current_pt)
                    remaining.pop(i)
                    found = True
                    break
            if not found:
                break

        # Remove duplicate closing vertex if present
        if len(walked) > 1 and _approx_eq(walked[0], walked[-1]):
            walked = walked[:-1]

        if len(walked) < 3:
            return None

        try:
            poly = snap(Polygon(walked))
            if not poly.is_valid or poly.area < 1.0:  # sanity: at least 1 mm²
                return None
            return poly
        except Exception:  # noqa: BLE001
            return None

    except Exception:  # noqa: BLE001
        return None


def extract_drill_obstacles(
    board_path: object,
    clearance_mm: float,
    track_mm: float,
) -> list:
    """Extract circular obstacles for through-hole pad drills and via drills.

    Each drill hole is modelled as a circle with radius
    ``drill_r + clearance_mm + track_mm / 2`` so that track centrelines are
    kept at least ``clearance_mm`` from the drill-hole edge (copper ring
    annular zone).

    Returns a list of Shapely Polygons (circles), snapped to PRECISION.
    Returns [] if extraction fails or HAVE_SHAPELY is False.
    """
    if not HAVE_SHAPELY:
        return []
    try:
        from tracewise.sexpr import parse_file

        root = parse_file(board_path)
        inflate = clearance_mm + track_mm / 2.0
        obstacles: list = []

        # We use a set of (cx, cy, r) rounded to 3 decimal places to deduplicate
        seen: set[tuple[float, float, float]] = set()

        def _add_circle(cx: float, cy: float, drill_r: float) -> None:
            key = (round(cx, 3), round(cy, 3), round(drill_r, 3))
            if key in seen:
                return
            seen.add(key)
            total_r = drill_r + inflate
            if total_r <= 0:
                return
            circ = snap(Point(cx, cy).buffer(total_r, resolution=8))
            obstacles.append(circ)

        # Walk all pads looking for (drill ...) children
        for fp in root.find_all("footprint"):
            # footprint position
            at_node = fp.first("at")
            try:
                fp_x = float(at_node.arg(1)) if at_node else 0.0
                fp_y = float(at_node.arg(2)) if at_node else 0.0
            except (TypeError, ValueError):
                fp_x, fp_y = 0.0, 0.0

            for pad in fp.find_all("pad"):
                drill_node = pad.first("drill")
                if drill_node is None:
                    continue
                # pad position (relative to footprint)
                at_pad = pad.first("at")
                try:
                    pad_rel_x = float(at_pad.arg(1)) if at_pad else 0.0
                    pad_rel_y = float(at_pad.arg(2)) if at_pad else 0.0
                except (TypeError, ValueError):
                    pad_rel_x, pad_rel_y = 0.0, 0.0

                # Absolute pad position (ignore rotation for drill centre)
                pad_x = fp_x + pad_rel_x
                pad_y = fp_y + pad_rel_y

                # Parse drill diameter — may be "drill <d>" or "drill oval <d1> <d2>"
                drill_atoms = drill_node.atoms()
                try:
                    # drill_atoms[0] is "drill", then either a number or "oval"
                    if len(drill_atoms) >= 2 and drill_atoms[1].value == "oval":
                        # oval: use the SMALLER of the two diameters for the obstacle
                        d = min(float(drill_atoms[2].value), float(drill_atoms[3].value))
                    else:
                        d = float(drill_atoms[1].value)
                except (IndexError, ValueError):
                    continue

                _add_circle(pad_x, pad_y, d / 2.0)

        # Walk standalone vias (if any in the template/netlists)
        for via in root.find_all("via"):
            drill_node = via.first("drill")
            if drill_node is None:
                continue
            at_node = via.first("at")
            try:
                vx = float(at_node.arg(1)) if at_node else 0.0
                vy = float(at_node.arg(2)) if at_node else 0.0
            except (TypeError, ValueError):
                continue
            try:
                d = float(drill_node.arg())
            except (TypeError, ValueError):
                continue
            _add_circle(vx, vy, d / 2.0)

        return obstacles

    except Exception:  # noqa: BLE001
        return []


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
    board_outline: object | None = None,
    drill_obstacles: list | None = None,
) -> tuple[object, list]:
    """Build the free-space polygon within *window_bbox* for routing *net_name*.

    Algorithm
    ---------
    1. Start with the routing window polygon.  If *board_outline* is provided,
       intersect the window with the outline shrunk inward by
       ``clearance_mm + track_mm / 2`` (keeps track centrelines
       legal-distance from the board edge).
    2. Collect all other-net F.Cu pad rectangles whose bounding boxes intersect
       the window.  Inflate each by ``clearance_mm + track_mm / 2`` (Minkowski
       half-width: keeps the track *centreline* at ≥ *clearance_mm* from any
       copper edge).
    3. Add any *drill_obstacles* (pre-inflated circles for through-hole drills
       and via drills) that intersect the window.
    4. Add any pre-inflated *extra_obstacles* (buffered centrelines of
       already-routed nets) that intersect the window.
    5. Compute the union of all inflated obstacles, then **carve the routing
       net's own pad rects out** of that union (own-pad carve-out, FIX-2): this
       ensures the start/goal pad centres remain inside free space even after
       adjacent nets' copper buffers overlap them.
    6. Subtract the trimmed union from the (possibly board-clipped) window
       polygon, snap to PRECISION.

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
    board_outline:
        Optional Shapely Polygon of the board interior.  When provided, the
        free space is clipped to the outline shrunk inward by
        ``clearance_mm + track_mm / 2`` so track centrelines stay
        legal-distance from the board edge.
    drill_obstacles:
        Optional list of pre-inflated Shapely circle Polygons for drill holes
        (through-hole pads and vias).  Each circle is already inflated by
        ``clearance_mm + track_mm / 2``; just test window intersection and add.

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

    # --- Board edge clipping ---
    # Shrink the board outline inward by inflate so track centrelines stay
    # legal-distance from the physical board edge.
    if board_outline is not None:
        try:
            shrunk_outline = snap(board_outline.buffer(-inflate, join_style=2))
            if not shrunk_outline.is_empty and shrunk_outline.is_valid:
                window_poly = snap(window_poly.intersection(shrunk_outline))
        except Exception:  # noqa: BLE001
            pass  # If outline processing fails, fall back to plain window

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

    # Add drill-hole obstacles that intersect the window
    if drill_obstacles:
        for drill_obs in drill_obstacles:
            try:
                if drill_obs.intersects(window_poly):
                    obstacle_polys.append(drill_obs)
            except Exception:  # noqa: BLE001
                pass

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
