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
    layer: int = 0,
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
        ``x``, ``y``, ``hw``, ``hh``, ``front``, ``back``).
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
    layer:
        Routing layer: 0 = F.Cu (default), 1 = B.Cu.  Selects which pads are
        obstacles — ``p["front"]`` for layer 0, ``p["back"]`` for layer 1.
        Board-outline clip and drill_obstacles are shared (layer-independent).
        Default ``0`` preserves byte-identical behaviour for all existing callers.

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

    # --- Own-pad boundary re-carve (M3-P2 FIX: boundary pads always reachable) ---
    # Collect routing net's own pads (layer-filtered) BEFORE building obstacles.
    # We need these to re-carve them into free space after the board-edge shrink,
    # so that a through-hole pad at the board boundary is always inside free space
    # even when its centre is outside the shrunk window_poly.
    own_pads_for_layer = [
        p for p in pads
        if p["net"] == net_name and (p.get("front") if layer == 0 else p.get("back"))
    ]
    # Build own-pad union early (needed for both the obstacle carve-out AND the
    # window expansion below).
    own_pad_polys_pre: list = []
    for p in own_pads_for_layer:
        px1 = p["x"] - p["hw"]
        py1 = p["y"] - p["hh"]
        px2 = p["x"] + p["hw"]
        py2 = p["y"] + p["hh"]
        own_pad_polys_pre.append(box(px1, py1, px2, py2))

    if own_pad_polys_pre:
        own_union_pre = snap(unary_union(own_pad_polys_pre))
        # Expand window_poly to include any own pads that the board-edge shrink cut out.
        # This is the M3-P2 boundary-pad fix: the routing net's own start/goal pads
        # must ALWAYS be inside free space, even when at the physical board edge.
        try:
            window_poly = snap(window_poly.union(own_union_pre))
        except Exception:  # noqa: BLE001
            pass  # If union fails, keep the shrunken window (conservative fallback)
    else:
        own_union_pre = None

    obstacle_polys: list = []
    for p in pads:
        if p["net"] == net_name:
            continue
        # Layer-aware pad filter: layer 0 = F.Cu (front), layer 1 = B.Cu (back).
        # Through-hole pads have both front=True and back=True — they appear as
        # obstacles on both layers, which is correct (copper annulus on both).
        if layer == 0:
            if not p.get("front"):
                continue
        else:
            if not p.get("back"):
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

        # Own-pad carve-out (FIX-2 + M3-P2): routing net's own pads must remain
        # reachable even if adjacent routed copper overlaps them AND even after
        # the board-edge inward shrink (the new M3-P2 boundary-pad fix).
        # Re-use the own_union_pre built above (already snapped).
        if own_union_pre is not None:
            union = snap(union.difference(own_union_pre))

        free_space = snap(window_poly.difference(union))
    else:
        obstacle_polys = []
        free_space = window_poly

    return free_space, obstacle_polys


def net_routes_to_track_obstacles(
    results: dict,
    grid: object,
    track_mm: float,
    clearance_mm: float,
    layer: int | None = None,
) -> list:
    """Convert routed grid NetRoute paths to buffered Shapely obstacle polygons.

    Each routed grid net's path segments are converted to world-mm line segments,
    then buffered by ``track_mm / 2 + clearance_mm`` (Minkowski inflate for
    track-to-track clearance), snapped to PRECISION, and returned as a list of
    Shapely Polygons.

    These obstacles represent the grid copper that the gridless rescue router
    must avoid — the key new ingredient of the M2.1 grid-first rescue mode.

    Parameters
    ----------
    results:
        ``dict[str, NetRoute]`` from ``route_all`` — only entries with ``nr.ok``
        and at least one path segment are converted.  ``GridlessNetRoute`` entries
        (already gridless) are skipped — their copper is rasterized into the shared
        grid ledger via ``_mark`` and is visible to ``build_windowed_free_space``
        through the pad obstacle set.
    grid:
        The shared occupancy ``Grid`` object — used to convert ``(layer, iy, ix)``
        cell coordinates back to world mm via ``grid.to_world``.
    track_mm:
        Track width in mm.
    clearance_mm:
        Required copper-to-copper clearance in mm.
    layer:
        Optional layer filter.  ``None`` (default) → include copper from all
        layers (existing behaviour).  ``0`` → F.Cu only.  ``1`` → B.Cu only.
        Grid paths are split at layer transitions by ``simplify`` — this
        parameter selects which sub-runs to include.  For ``GridlessNetRoute``
        entries (3-tuple world paths), waypoints whose third element matches
        *layer* are included; 2-tuple paths (single-layer) are included only
        when ``layer`` is ``None`` or ``0`` (F.Cu default).

    Returns
    -------
    List of pre-inflated Shapely Polygons (one per track segment, merged where
    consecutive segments on the same net overlap).  Returns ``[]`` if Shapely is
    absent or ``results`` is empty.

    Notes
    -----
    The inflation uses ``track_mm / 2 + clearance_mm`` (the FIX-6 formula):
    this keeps a subsequent gridless track centreline at ``>= clearance_mm``
    from any existing grid track edge.

    GridlessNetRoute entries carry exact world_paths already and their copper is
    already in the grid obstacle ledger after ``_mark`` — no double-counting.
    """
    if not HAVE_SHAPELY:
        return []
    try:
        from shapely.geometry import LineString

        from tracewise.route.engine.astar import simplify
        from tracewise.route.gridless.adapter import GridlessNetRoute

        inflate = track_mm / 2.0 + clearance_mm
        obstacles: list = []

        for _name, nr in results.items():
            if not nr.ok:
                continue

            # GridlessNetRoute: world_paths already emitted; copper visible via
            # the grid ledger cells + _mark. Skip to avoid double-counting.
            if isinstance(nr, GridlessNetRoute):
                if nr.world_paths:
                    for wpath in nr.world_paths:
                        if len(wpath) < 2:
                            continue
                        if layer is None:
                            # All-layer: include as-is (strip layer coord)
                            pts_2d: list = [
                                (pt[0], pt[1]) for pt in wpath
                            ]
                            if len(pts_2d) >= 2:
                                ls = snap(
                                    LineString(pts_2d).buffer(inflate, cap_style=2)
                                )
                                obstacles.append(ls)
                        else:
                            # Layer-filtered: split into per-layer sub-segments
                            seg: list[tuple[float, float]] = []
                            for pt in wpath:
                                pt_layer = pt[2] if len(pt) == 3 else 0
                                if pt_layer == layer:
                                    seg.append((pt[0], pt[1]))
                                else:
                                    if len(seg) >= 2:
                                        ls = snap(
                                            LineString(seg).buffer(
                                                inflate, cap_style=2
                                            )
                                        )
                                        obstacles.append(ls)
                                    seg = []
                            if len(seg) >= 2:
                                ls = snap(
                                    LineString(seg).buffer(inflate, cap_style=2)
                                )
                                obstacles.append(ls)
                continue

            # Grid NetRoute: paths are lists of (layer, iy, ix) cell tuples.
            # simplify() splits at layer transitions, so each run has one layer.
            for path in nr.paths:
                runs = simplify(path)
                for run in runs:
                    if len(run) < 2:
                        continue
                    # Layer filter: skip runs whose layer doesn't match.
                    run_layer = run[0][0]
                    if layer is not None and run_layer != layer:
                        continue
                    world_pts = [grid.to_world(c[1], c[2]) for c in run]
                    if len(world_pts) >= 2:
                        ls = snap(
                            LineString(world_pts).buffer(inflate, cap_style=2)
                        )
                        obstacles.append(ls)

        return obstacles
    except Exception:  # noqa: BLE001 — Shapely absent or path conversion failed
        return []


def net_routes_to_via_obstacles(
    results: dict,
    grid: object,
    via_mm: float,
    clearance_mm: float,
    track_mm: float,
) -> list:
    """Convert grid-router-placed vias to buffered Shapely obstacle circles.

    Grid router vias (stored in ``nr.via_sites`` as ``(iy, ix)`` grid cells) are
    placed at routing time and are NOT written to the KiCad file until emit.
    They are therefore absent from ``extract_drill_obstacles``.  This function
    builds per-via Shapely Polygon circles so that subsequent gridless rescue
    runs can avoid crossing via annular rings.

    The inflate radius is ``via_mm / 2 + track_mm / 2``:
    keeps a rescue track centreline strictly outside the via copper edge so the
    rescue track does NOT overlap via copper (preventing ``shorting_items`` DRC
    violations).  This is intentionally less conservative than the full
    ``via_mm/2 + clearance_mm + track_mm/2`` formula: tracks may pass within
    ``clearance_mm`` of a via (``clearance`` DRC violations are expected and
    permitted by the rescue gate) but must never OVERLAP via copper (``short``
    violations fail the gate).  The narrower radius preserves B.Cu corridors
    between closely-spaced routing vias that would otherwise be completely
    closed off.

    Parameters
    ----------
    results:
        ``dict[str, NetRoute]`` from ``route_all``.  Only ``ok`` grid
        ``NetRoute`` entries (not ``GridlessNetRoute``) are processed because
        only they store routing-time vias in ``via_sites``.
    grid:
        Shared ``Grid`` — used to convert ``(iy, ix)`` back to world mm.
    via_mm:
        Via copper diameter in mm.
    clearance_mm:
        Required copper-to-copper clearance in mm.
    track_mm:
        Track width in mm (used to keep track centreline clear of via edge).

    Returns
    -------
    List of inflated Shapely Polygons (one circle per unique via position).
    Returns ``[]`` if Shapely is absent or no vias found.
    """
    if not HAVE_SHAPELY:
        return []
    try:
        from shapely.geometry import Point as _Point

        from tracewise.route.gridless.adapter import GridlessNetRoute

        # Inflate = via copper radius + track half-width: the track centerline
        # must stay strictly outside the via copper disc to prevent copper
        # overlap (short). Does NOT enforce the full clearance margin — that
        # would close narrow corridors between closely-spaced routing vias.
        inflate = via_mm / 2.0 + track_mm / 2.0
        obstacles: list = []

        for _name, nr in results.items():
            if not nr.ok:
                continue
            # GridlessNetRoute vias are world_vias (already in mm) — they are
            # accumulated into _rescue_bcu_obstacles during the rescue block so
            # we do NOT double-count them here.
            if isinstance(nr, GridlessNetRoute):
                continue
            for iy, ix in nr.via_sites:
                wx, wy = grid.to_world(iy, ix)  # type: ignore[union-attr]
                try:
                    circ = snap(
                        _Point(wx, wy).buffer(inflate, resolution=16)
                    )
                    obstacles.append(circ)
                except Exception:  # noqa: BLE001
                    pass

        return obstacles
    except Exception:  # noqa: BLE001
        return []


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


# ---------------------------------------------------------------------------
# M3: Drill-center extraction
# ---------------------------------------------------------------------------


def extract_drill_centers(board_path: object) -> list[tuple[float, float, float]]:
    """Extract drill centers and radii from the board.

    Returns a list of ``(cx, cy, drill_r)`` tuples for through-hole pad drills and
    via drills.  Used by the M3 via-legality predicate 3 (hole-to-hole check).

    Returns ``[]`` if extraction fails or HAVE_SHAPELY is False.
    """
    if not HAVE_SHAPELY:
        return []
    try:
        from tracewise.sexpr import parse_file

        root = parse_file(board_path)
        centers: list[tuple[float, float, float]] = []
        seen: set[tuple[float, float, float]] = set()

        for fp in root.find_all("footprint"):
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
                at_pad = pad.first("at")
                try:
                    px = float(at_pad.arg(1)) if at_pad else 0.0
                    py = float(at_pad.arg(2)) if at_pad else 0.0
                except (TypeError, ValueError):
                    px, py = 0.0, 0.0
                cx = fp_x + px
                cy = fp_y + py

                drill_atoms = drill_node.atoms()
                try:
                    if len(drill_atoms) >= 2 and drill_atoms[1].value == "oval":
                        d = min(float(drill_atoms[2].value), float(drill_atoms[3].value))
                    else:
                        d = float(drill_atoms[1].value)
                except (IndexError, ValueError):
                    continue

                key = (round(cx, 3), round(cy, 3), round(d / 2.0, 3))
                if key not in seen:
                    seen.add(key)
                    centers.append((cx, cy, d / 2.0))

        # Also collect vias already placed on the board
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
            key = (round(vx, 3), round(vy, 3), round(d / 2.0, 3))
            if key not in seen:
                seen.add(key)
                centers.append((vx, vy, d / 2.0))

        return centers
    except Exception:  # noqa: BLE001
        return []


# ---------------------------------------------------------------------------
# M3: Per-component exterior-ring corner collection (THE key fix from spike-M3)
# ---------------------------------------------------------------------------


def exterior_ring_corners(
    fs: object,
    start_xy: tuple[float, float],
    goal_xy: tuple[float, float],
    margin_mm: float,
) -> list[tuple[float, float]]:
    """Extract exterior-ring vertices from MultiPolygon (or Polygon) free space.

    When free space is fragmented by grid copper into a MultiPolygon, obstacles
    appear on the EXTERIOR boundaries of each polygon fragment (not as interior-ring
    holes).  This function collects those exterior vertices so the visibility graph
    has waypoints.  Interior-ring corners are also collected when present.

    Per-COMPONENT collection is critical: it cuts edge-build ~93% vs naively
    collecting all vertices across all components (the spike measured 964K→64K pairs
    on a typical post-grid-routing F.Cu fragmentation).  Only components that
    contain start/goal or legal via sites need their corners as waypoints.

    Parameters
    ----------
    fs:
        Free-space geometry (Polygon or MultiPolygon).
    start_xy, goal_xy:
        Route endpoints for the bounding-box filter.
    margin_mm:
        Window margin around start→goal bbox.

    Returns
    -------
    Sorted list of ``(round(x, 6), round(y, 6))`` corner coordinates.
    """
    _require_shapely()
    sx, sy = start_xy
    gx, gy = goal_xy
    x_lo = min(sx, gx) - margin_mm
    x_hi = max(sx, gx) + margin_mm
    y_lo = min(sy, gy) - margin_mm
    y_hi = max(sy, gy) + margin_mm

    pts: set[tuple[float, float]] = set()

    if fs.geom_type == "MultiPolygon":  # type: ignore[union-attr]
        components = list(fs.geoms)  # type: ignore[union-attr]
    else:
        components = [fs]

    for comp in components:
        # Interior ring vertices (holes = obstacles in single-polygon free space)
        for ring in comp.interiors:  # type: ignore[union-attr]
            for x, y in ring.coords:
                if x_lo <= x <= x_hi and y_lo <= y <= y_hi:
                    pts.add((round(x, 6), round(y, 6)))
        # Exterior ring vertices (obstacle boundaries in fragmented free space)
        for x, y in comp.exterior.coords:  # type: ignore[union-attr]
            if x_lo <= x <= x_hi and y_lo <= y <= y_hi:
                pts.add((round(x, 6), round(y, 6)))

    return sorted(pts)


# ---------------------------------------------------------------------------
# M3: Candidate via sites
# ---------------------------------------------------------------------------

def _round1nm_geom(v: float) -> float:
    """Snap to 1nm grid (used in M3 via-site generation)."""
    return round(v * 1e6) / 1e6


def candidate_via_sites(
    fs_F: object,
    fs_B: object,
    start_xy: tuple[float, float],
    goal_xy: tuple[float, float],
    window_bbox: tuple[float, float, float, float],
    via_mm: float,
    clearance_mm: float,
) -> list[tuple[float, float]]:
    """Collect candidate via sites for a 2-layer route.

    Three sources (all sorted for determinism):
    1. **Grid-sample** each F.Cu component that intersects B.Cu free space at
       pitch = ``via_mm + clearance_mm``.  Handles the typical post-grid-routing
       case where F.Cu is a fragmented MultiPolygon with disconnected islands.
    2. **Shared obstacle corners** — interior-ring vertices present on BOTH
       layers at equal 1 nm coords.
    3. **Straight lattice** every ``via_mm + clearance_mm`` along start→goal.

    Returns
    -------
    Sorted ``[(round(x, 6), round(y, 6)), ...]`` candidate via positions.
    """
    _require_shapely()
    import math as _math

    wx1, wy1, wx2, wy2 = window_bbox
    via_pitch = via_mm + clearance_mm

    all_sites: set[tuple[float, float]] = set()

    # 1. Grid-sample each F.Cu component that intersects B.Cu free space
    if fs_F.geom_type == "MultiPolygon":  # type: ignore[union-attr]
        f_components = list(fs_F.geoms)  # type: ignore[union-attr]
    else:
        f_components = [fs_F]

    for comp in f_components:
        if not comp.intersects(fs_B):  # type: ignore[union-attr]
            continue
        cx1, cy1, cx2, cy2 = comp.bounds
        x = cx1
        while x <= cx2 + 1e-9:
            y = cy1
            while y <= cy2 + 1e-9:
                site = (_round1nm_geom(x), _round1nm_geom(y))
                if wx1 <= site[0] <= wx2 and wy1 <= site[1] <= wy2:
                    sp = Point(site[0], site[1])
                    if comp.contains(sp) and fs_B.contains(sp):  # type: ignore[union-attr]
                        all_sites.add(site)
                y += via_pitch
            x += via_pitch

    # 2. Shared obstacle corners (interior-ring vertices on both layers)
    def _ring_corners(fs_geom: object) -> set[tuple[float, float]]:
        corners: set[tuple[float, float]] = set()
        if fs_geom.geom_type == "MultiPolygon":  # type: ignore[union-attr]
            comps = list(fs_geom.geoms)  # type: ignore[union-attr]
        else:
            comps = [fs_geom]
        for c in comps:
            for ring in c.interiors:  # type: ignore[union-attr]
                for x, y in ring.coords:
                    if wx1 <= x <= wx2 and wy1 <= y <= wy2:
                        corners.add((_round1nm_geom(x), _round1nm_geom(y)))
        return corners

    corners_F = _ring_corners(fs_F)
    corners_B = _ring_corners(fs_B)
    all_sites |= (corners_F & corners_B)

    # 3. Straight lattice along start→goal
    ax, ay = start_xy
    bx, by = goal_xy
    dist = _math.hypot(bx - ax, by - ay)
    if dist > 1e-9:
        n_steps = max(1, int(_math.ceil(dist / via_pitch)))
        for i in range(n_steps + 1):
            t = i / n_steps if n_steps > 0 else 0.5
            x = _round1nm_geom(ax + t * (bx - ax))
            y = _round1nm_geom(ay + t * (by - ay))
            if wx1 <= x <= wx2 and wy1 <= y <= wy2:
                all_sites.add((x, y))

    return sorted(all_sites, key=lambda p: (round(p[0], 6), round(p[1], 6)))


# ---------------------------------------------------------------------------
# M3: Three-predicate hole-aware via legality test
# ---------------------------------------------------------------------------


def is_legal_via(
    site: tuple[float, float],
    fs_F: object,
    fs_B: object,
    pads: list[dict],
    net_name: str,
    via_mm: float,
    via_drill: float,
    clearance_mm: float,
    hole_clearance: float,
    hole_to_hole: float,
    drill_centers: list[tuple[float, float, float]],
    window_bbox: tuple[float, float, float, float],
) -> tuple[bool, str]:
    """Test if a candidate via site passes all three DQ4 predicates.

    Returns ``(is_legal, fail_reason)``; ``fail_reason`` is ``""`` if legal.

    Predicates (all three must pass; short-circuit on first fail):

    1. **Copper-ring clearance, both layers** — a disc of radius
       ``via_mm/2 + clearance_mm`` centred at *site* must lie within the
       free space on both F.Cu and B.Cu.  Because the per-layer free space
       already has all other-net copper subtracted (inflated by
       ``clearance + track/2``), this is the via-copper-clearance check by
       construction — legality-by-construction, no post-hoc nudge.

    2. **Drill-to-copper clearance** (the LARGER ``hole_clearance`` — the #4/M2.1
       lesson) — a disc of radius ``via_drill/2 + hole_clearance`` must not
       intersect any other-net pad copper on either layer.  ``hole_clearance``
       (~0.25 mm) is larger than ``clearance_mm`` (~0.15–0.2 mm); honoring
       only copper clearance was the boxed-in-via bug.

    3. **Drill-to-drill spacing** — for every existing drill center *c*,
       ``dist(site, c) ≥ via_drill/2 + drill_r(c) + hole_to_hole``.
    """
    _require_shapely()
    import math as _math

    x, y = site
    sp = Point(x, y)

    # Predicate 1: copper ring disc within free space on both layers
    copper_disc = snap(sp.buffer(via_mm / 2.0 + clearance_mm, resolution=16))
    if not copper_disc.within(fs_F):  # type: ignore[union-attr]
        return False, "pred1_fcu_copper_ring"
    if not copper_disc.within(fs_B):  # type: ignore[union-attr]
        return False, "pred1_bcu_copper_ring"

    # Predicate 2: drill disc clearance to other-net copper (hole_clearance > clearance)
    drill_disc = snap(sp.buffer(via_drill / 2.0 + hole_clearance, resolution=16))
    wx1, wy1, wx2, wy2 = window_bbox
    for lyr_idx in [0, 1]:
        for p in pads:
            if p["net"] == net_name:
                continue
            if lyr_idx == 0 and not p.get("front"):
                continue
            if lyr_idx == 1 and not p.get("back"):
                continue
            px1 = p["x"] - p["hw"]
            py1 = p["y"] - p["hh"]
            px2 = p["x"] + p["hw"]
            py2 = p["y"] + p["hh"]
            if px2 < wx1 or px1 > wx2 or py2 < wy1 or py1 > wy2:
                continue
            # Check if drill disc intersects pad copper (raw copper, no inflation)
            pad_rect = box(px1, py1, px2, py2)
            if drill_disc.intersects(pad_rect):
                return False, f"pred2_drill_to_copper_layer{lyr_idx}"

    # Predicate 3: drill-to-drill spacing
    for cx, cy, drill_r in drill_centers:
        dist = _math.hypot(x - cx, y - cy)
        required = via_drill / 2.0 + drill_r + hole_to_hole
        if dist < required - 1e-6:
            return False, f"pred3_drill_to_drill (dist={dist:.4f}<{required:.4f})"

    return True, ""


# ---------------------------------------------------------------------------
# Fanout-escape: dense-component detection
# ---------------------------------------------------------------------------


def detect_dense_components(pads: list[dict]) -> list[dict]:
    """Detect "dense pad-forest" components where pads form a tight ring.

    A component qualifies if it has ≥8 pads AND the pads' distances from the
    centroid have a coefficient of variation (std/mean) < 0.20 (ring-like) AND
    the mean pad pitch to the nearest neighbour is ≤ 0.65 mm.

    Parameters
    ----------
    pads:
        All board pads from ``extract_pads``.  Each pad must have keys
        ``"ref"``, ``"x"``, ``"y"``.

    Returns
    -------
    List of ``{"ref": str, "cx": float, "cy": float, "ring_radius": float,
    "pads": list[dict]}`` dicts, one per dense component.  Sorted by ``ref``
    for determinism.
    """
    import statistics

    # Group pads by component reference
    by_ref: dict[str, list[dict]] = {}
    for p in pads:
        ref = p.get("ref", "")
        if not ref:
            continue
        by_ref.setdefault(ref, []).append(p)

    dense: list[dict] = []
    for ref in sorted(by_ref):
        all_comp_pads = sorted(
            by_ref[ref], key=lambda p: (round(p["x"], 6), round(p["y"], 6))
        )

        # Only consider SMD F.Cu pads (front=True AND back=False).
        # This excludes:
        #   - non-copper entries (courtyard, fab, paste-only pads):
        #     front=False, back=False
        #   - through-hole pads (thermal vias): front=True, back=True
        # QFN signal pads are exclusively SMD front-copper (SMD F.Cu), so this
        # filter preserves exactly the pads that form the ring while excluding
        # the thermal via array that would inflate the CV and break detection.
        comp_pads = [
            p for p in all_comp_pads
            if p.get("front") and not p.get("back")
        ]
        if len(comp_pads) < 8:
            continue

        # Centroid = mean of copper pad positions
        xs = [p["x"] for p in comp_pads]
        ys = [p["y"] for p in comp_pads]
        cx = statistics.mean(xs)
        cy = statistics.mean(ys)

        # Distances from centroid
        import math as _math
        dists = [_math.hypot(p["x"] - cx, p["y"] - cy) for p in comp_pads]
        mean_dist = statistics.mean(dists)
        if mean_dist < 1e-6:
            continue  # degenerate: all pads at same point

        std_dist = statistics.pstdev(dists)  # population std dev
        cv = std_dist / mean_dist

        # Ring criterion: CV < 0.20 (tight ring around centroid)
        if cv >= 0.20:
            continue

        # Pitch criterion: mean nearest-neighbour distance ≤ 0.65 mm
        pitches: list[float] = []
        for i, p in enumerate(comp_pads):
            min_d = min(
                _math.hypot(p["x"] - comp_pads[j]["x"], p["y"] - comp_pads[j]["y"])
                for j in range(len(comp_pads)) if j != i
            )
            pitches.append(min_d)
        mean_pitch = statistics.mean(pitches)
        if mean_pitch > 0.65:
            continue

        ring_radius = mean_dist
        dense.append({
            "ref": ref,
            "cx": cx,
            "cy": cy,
            "ring_radius": ring_radius,
            "pads": comp_pads,
        })

    return dense


# ---------------------------------------------------------------------------
# Fanout-escape: guided escape-via placement
# ---------------------------------------------------------------------------


def guided_escape_via(
    source_pad_xy: tuple[float, float],
    component_cx: float,
    component_cy: float,
    ring_radius: float,
    fs_F: object,
    fs_B: object,
    pads: list[dict],
    net_name: str,
    via_mm: float,
    via_drill: float,
    clearance_mm: float,
    hole_clearance: float,
    hole_to_hole: float,
    drill_centers: list[tuple[float, float, float]],
    window_bbox: tuple[float, float, float, float],
    margin_mm: float = 0.3,
    search_arc_deg: float = 60.0,
    search_steps: int = 24,
    radial_steps: int = 5,
    radial_step_mm: float = 0.2,
) -> tuple[float, float] | None:
    """Place an escape via on the ray from component centroid through the source pad.

    The via is placed just outside the pad ring:
    ``target_r = ring_radius + via_mm/2 + clearance_mm + margin_mm``

    If the initial position is not legal (3-predicate via legality test), searches
    nearby candidates along the ring perimeter (±``search_arc_deg/2``) and radially
    outward (``radial_steps × radial_step_mm``), returning the nearest legal site.

    Parameters
    ----------
    source_pad_xy:
        Centre of the source pad (the QFN SMD pad, F.Cu).
    component_cx, component_cy:
        Component centroid (centre of the pad ring).
    ring_radius:
        Mean distance from centroid to pad centres.
    fs_F, fs_B:
        Per-layer free-space polygons (from ``build_windowed_free_space``).
    pads:
        All board pads (for the ``is_legal_via`` drill-to-copper check).
    net_name:
        Net being routed (own pads excluded from the obstacle check).
    via_mm, via_drill, clearance_mm, hole_clearance, hole_to_hole:
        Via and clearance geometry parameters.
    drill_centers:
        ``(cx, cy, drill_r)`` tuples for hole-to-hole predicate.
    window_bbox:
        Search window ``(x1, y1, x2, y2)`` for the drill-to-copper check.
    margin_mm:
        Extra radial margin beyond ``ring_radius + via_mm/2 + clearance_mm``.
    search_arc_deg:
        Total arc (±half on each side) to search around the guided angle.
    search_steps:
        Number of angular steps in the search arc.
    radial_steps:
        Number of outward radial steps.
    radial_step_mm:
        Radial step size (mm).

    Returns
    -------
    ``(x, y)`` of the nearest legal via site, or ``None`` if no legal site found.
    """
    _require_shapely()
    import math as _math

    sx, sy = source_pad_xy
    dx = sx - component_cx
    dy = sy - component_cy
    dist_to_source = _math.hypot(dx, dy)
    if dist_to_source < 1e-6:
        return None

    ux = dx / dist_to_source
    uy = dy / dist_to_source

    # Initial guided position: on the ray, just outside the ring
    target_r = ring_radius + via_mm / 2.0 + clearance_mm + margin_mm
    init_x = round((component_cx + ux * target_r) * 1e6) / 1e6
    init_y = round((component_cy + uy * target_r) * 1e6) / 1e6
    initial_pos = (init_x, init_y)

    # Try the initial position first
    ok, _reason = is_legal_via(
        initial_pos, fs_F, fs_B, pads, net_name,
        via_mm, via_drill, clearance_mm, hole_clearance, hole_to_hole,
        drill_centers, window_bbox,
    )
    if ok:
        return initial_pos

    # Search arc: vary angle ± search_arc_deg/2 and radius outward
    init_dist = _math.hypot(init_x - component_cx, init_y - component_cy)
    init_angle_deg = _math.degrees(_math.atan2(init_y - component_cy, init_x - component_cx))

    candidates: list[tuple[float, float, int, int, tuple[float, float]]] = []
    half_arc = search_arc_deg / 2.0
    n_arc = max(1, search_steps // 2)
    for ai in range(-n_arc, n_arc + 1):
        angle_deg = init_angle_deg + ai * (half_arc / n_arc if n_arc > 0 else 0.0)
        angle_rad = _math.radians(angle_deg)
        for ri in range(radial_steps):
            r = init_dist + ri * radial_step_mm
            cx2 = round((component_cx + r * _math.cos(angle_rad)) * 1e6) / 1e6
            cy2 = round((component_cy + r * _math.sin(angle_rad)) * 1e6) / 1e6
            dist_from_start = _math.hypot(cx2 - init_x, cy2 - init_y)
            candidates.append((dist_from_start, abs(ai), ri, id((cx2, cy2)), (cx2, cy2)))

    # Sort: nearest to initial position first, then by angle offset, then radial
    candidates.sort(key=lambda t: (t[0], t[1], t[2]))

    for _d, _ai, _ri, _id, site in candidates:
        ok, _reason = is_legal_via(
            site, fs_F, fs_B, pads, net_name,
            via_mm, via_drill, clearance_mm, hole_clearance, hole_to_hole,
            drill_centers, window_bbox,
        )
        if ok:
            return site

    return None
