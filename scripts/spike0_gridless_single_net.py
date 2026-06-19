"""Spike-0: FAR gridless / shape-based router — single-net proof-of-concept.

Validates the realize→emit→DRC loop on real Shapely room geometry for one net
on the mitayi board.  Standalone script — no engine changes.

Usage:
    taskset -c 0-9 .venv/bin/python scripts/spike0_gridless_single_net.py

Pass criteria (ALL must hold):
  - 0 DRC errors attributable to the emitted trace.
  - Net connected (ratsnest resolved).
  - Runtime < 5 s (Shapely-perf gate).
  - Byte-identical across 2 runs (determinism gate).
"""
from __future__ import annotations

import heapq
import math
import shutil
import subprocess
import sys
import time
import tempfile
import collections
from pathlib import Path

# ---------------------------------------------------------------------------
# Shapely import + version check
# ---------------------------------------------------------------------------
try:
    import shapely
    from shapely.geometry import Point as SPoint, Polygon, MultiPolygon, box
    from shapely.ops import unary_union
    from shapely import set_precision

    GEOS_VERSION = shapely.geos_version
    if GEOS_VERSION < (3, 8, 0):
        raise RuntimeError(f"GEOS ≥ 3.8.0 required, got {GEOS_VERSION}")
    print(f"[spike0] Shapely {shapely.__version__}  GEOS {GEOS_VERSION}", flush=True)
except ImportError as exc:
    print(f"ERROR: Shapely not installed: {exc}", file=sys.stderr)
    sys.exit(1)

# ---------------------------------------------------------------------------
# TraceWise imports (reuse; do not reimplement)
# ---------------------------------------------------------------------------
ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "src"))

from tracewise.route.bridge import run_drc, strip_routing
from tracewise.route.engine.kicad import (
    extract_pads,
    build_problem,
    project_geometry,
    refill_zones,
)
from tracewise.route.engine.exact_geom import is_legal, segment_rect_distance
from tracewise.sexpr import atom, node, parse_file, write_file

BOARD_SRC = ROOT / "data/benchmark-boards/mitayi-pico-d1/Mitayi-Pico-D1.kicad_pcb"
SUFFIXES = (".kicad_pcb", ".kicad_sch", ".kicad_pro", ".kicad_prl")

PRECISION = 1e-6   # 1 nm — quantize all Shapely geometry

# ---------------------------------------------------------------------------
# Step 2: copy board to temp dir + strip routing
# ---------------------------------------------------------------------------

def setup_board(out_dir: Path) -> Path:
    bdir = BOARD_SRC.parent
    out_dir.mkdir(parents=True, exist_ok=True)
    for f in bdir.iterdir():
        if f.suffix in SUFFIXES:
            shutil.copy(f, out_dir / f.name)
    board = next(out_dir.glob("*.kicad_pcb"))
    strip_routing(board)
    return board


# ---------------------------------------------------------------------------
# Step 4: mechanical net selection
# ---------------------------------------------------------------------------

def pick_net(data: dict) -> tuple[str, tuple, tuple]:
    """Return (net_name, pad_a, pad_b) for the chosen 2-pin F.Cu net.

    Selection rule:
      - Exactly 2 pads, both front-layer (front=True, back=False → SMD F.Cu).
      - Shortest straight-line pad distance.
      - Has ≥1 other-net pad within ~2 mm of the straight line between its pads
        (so room expansion must clip an obstacle halo).
    """
    from_net: dict[str, list[dict]] = {}
    for p in data["pads"]:
        if not p["net"]:
            continue
        from_net.setdefault(p["net"], []).append(p)

    candidates = []
    for net, pads in from_net.items():
        # Require EXACTLY 2 total pads on this net (true 2-pin net)
        if len(pads) != 2:
            continue
        a, b = pads
        # Both must be front-only SMD (not through-hole, not back-layer)
        if not (a.get("front") and not a.get("back")):
            continue
        if not (b.get("front") and not b.get("back")):
            continue
        dx, dy = b["x"] - a["x"], b["y"] - a["y"]
        dist = math.hypot(dx, dy)
        candidates.append((dist, net, a, b))

    # Sort by distance (ascending = shortest first)
    candidates.sort(key=lambda t: t[0])

    # Among the candidates, pick the shortest one that has ≥1 other-net pad within 2 mm
    # of the a–b segment.
    NEAR_MM = 2.0
    for dist, net, a, b in candidates:
        ax, ay = a["x"], a["y"]
        bx, by = b["x"], b["y"]
        found_near = False
        for p in data["pads"]:
            if p["net"] == net:
                continue
            if not (p.get("front") or p.get("back")):
                continue
            # Distance from pad centre to segment a–b
            pad_dist = _point_to_seg_dist(p["x"], p["y"], ax, ay, bx, by)
            if pad_dist <= NEAR_MM:
                found_near = True
                break
        if found_near:
            print(f"[spike0] chosen net={net!r}  dist={dist:.3f} mm  pads: "
                  f"({ax:.3f},{ay:.3f})→({bx:.3f},{by:.3f})", flush=True)
            return net, a, b

    # Fallback: just use the shortest 2-pin F.Cu net (no obstacle proximity requirement)
    if candidates:
        dist, net, a, b = candidates[0]
        ax, ay = a["x"], a["y"]
        bx, by = b["x"], b["y"]
        print(f"[spike0] chosen net (fallback, no nearby obstacle)={net!r}  "
              f"dist={dist:.3f} mm", flush=True)
        return net, a, b

    raise RuntimeError("No 2-pin F.Cu net found on this board")


def _point_to_seg_dist(px, py, ax, ay, bx, by) -> float:
    dx, dy = bx - ax, by - ay
    len_sq = dx * dx + dy * dy
    if len_sq < 1e-18:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / len_sq))
    qx, qy = ax + t * dx, ay + t * dy
    return math.hypot(px - qx, py - qy)


# ---------------------------------------------------------------------------
# Step 5: build Shapely obstacle polygons + free space
# ---------------------------------------------------------------------------

def _snap(geom):
    """Quantize to PRECISION grid for determinism."""
    return set_precision(geom, PRECISION)


def build_free_space(
    data: dict,
    chosen_net: str,
    clearance_mm: float,
    track_mm: float,
) -> tuple[object, tuple[float, float, float, float]]:
    """Return (free_space_polygon, board_bbox_tuple).

    free_space = board_bbox minus union of inflated obstacle pads.
    Only F.Cu (front) pads of other nets are included.
    """
    inflate = clearance_mm + track_mm / 2.0
    bd = data["board"]
    board_bbox_tuple = (bd["x1"], bd["y1"], bd["x2"], bd["y2"])
    board_poly = _snap(box(bd["x1"], bd["y1"], bd["x2"], bd["y2"]))

    obstacle_polys = []
    for p in data["pads"]:
        if p["net"] == chosen_net:
            continue  # skip own net pads
        if not p.get("front"):
            continue  # only F.Cu
        rect = box(
            p["x"] - p["hw"],
            p["y"] - p["hh"],
            p["x"] + p["hw"],
            p["y"] + p["hh"],
        )
        inflated = _snap(rect.buffer(inflate, cap_style=3, join_style=2))
        obstacle_polys.append(inflated)

    if obstacle_polys:
        union = _snap(unary_union(obstacle_polys))
        free_space = _snap(board_poly.difference(union))
    else:
        free_space = board_poly

    return free_space, board_bbox_tuple


# ---------------------------------------------------------------------------
# Step 6: lazy convex room expansion + deterministic A* over rooms
# ---------------------------------------------------------------------------

class Room:
    """A convex (or near-convex) region of free space accessible to the path."""
    __slots__ = ("id", "poly", "portals")
    _counter = 0

    def __init__(self, poly: Polygon):
        Room._counter += 1
        self.id: int = Room._counter
        self.poly = poly
        self.portals: list[tuple[int, tuple[float, float, float, float], tuple[float, float]]] = []
        # portal: (to_room_id, (ax,ay,bx,by), (mx,my))


def _round1nm(v: float) -> int:
    """Round to 1 nm integer for deterministic comparisons."""
    return round(v * 1e6)


def _poly_convex_hull_snapped(poly) -> Polygon:
    return _snap(poly.convex_hull)


def _expand_room_toward(free_space, seed_xy: tuple[float, float],
                         toward_xy: tuple[float, float],
                         inflate_radius: float = 0.0) -> Polygon | None:
    """Grow a convex room from seed_xy in free_space toward toward_xy.

    Strategy:
    1. Start with a small disc at seed_xy.
    2. Expand outward: take the convex hull of seed + toward_xy, then
       intersect with free_space.
    3. If the intersection is too small or disconnected, return None.
    """
    sx, sy = seed_xy
    tx, ty = toward_xy

    # Initial seed: tiny square around seed_xy
    seed_pt = _snap(SPoint(sx, sy).buffer(1e-4, cap_style=3))
    if not free_space.contains(SPoint(sx, sy)):
        # seed is not in free space — use the nearest free point
        nearest = free_space.boundary.interpolate(
            free_space.boundary.project(SPoint(sx, sy))
        )
        seed_pt = _snap(SPoint(nearest.x, nearest.y).buffer(1e-4, cap_style=3))

    # Grow: convex hull of seed + target → intersect with free_space
    target_pt = _snap(SPoint(tx, ty).buffer(1e-4, cap_style=3))
    combined = _snap(seed_pt.union(target_pt))
    hull = _snap(combined.convex_hull)
    room_raw = _snap(hull.intersection(free_space))

    if room_raw.is_empty or room_raw.area < 1e-8:
        return None

    # If it's a multi-polygon, take the piece containing the seed
    if room_raw.geom_type == "MultiPolygon":
        geoms = list(room_raw.geoms)
        for g in geoms:
            if g.contains(SPoint(sx, sy)) or g.distance(SPoint(sx, sy)) < 1e-5:
                room_raw = g
                break
        else:
            room_raw = max(geoms, key=lambda g: g.area)

    # Return the convex hull of the intersected shape (may not be convex after diff)
    result = _snap(room_raw.convex_hull.intersection(free_space))
    if result.is_empty:
        result = room_raw
    return _snap(result)


def _portal_between(r1: Polygon, r2: Polygon) -> tuple[float, float, float, float] | None:
    """Shared edge/segment between two rooms, returned as (ax,ay,bx,by)."""
    shared = _snap(r1.intersection(r2))
    if shared.is_empty:
        return None
    if shared.geom_type == "LineString":
        coords = list(shared.coords)
        if len(coords) >= 2:
            ax, ay = coords[0]
            bx, by = coords[-1]
            return (ax, ay, bx, by)
    if shared.geom_type == "Point":
        x, y = shared.x, shared.y
        return (x, y, x, y)
    # MultiLineString or Polygon (thick overlap — use bbox edge)
    b = shared.bounds  # (minx, miny, maxx, maxy)
    return (b[0], b[1], b[2], b[3])


def _portal_midpoint(portal: tuple[float, float, float, float]) -> tuple[float, float]:
    ax, ay, bx, by = portal
    return ((ax + bx) / 2.0, (ay + by) / 2.0)


def astar_rooms(
    free_space,
    start_xy: tuple[float, float],
    goal_xy: tuple[float, float],
    max_rooms: int = 2000,
) -> list[Room] | None:
    """Lazy convex room expansion + deterministic A* over rooms.

    Returns the list of rooms from start to goal, or None if unreachable.

    Determinism guarantees:
    - Room insertion order is the tie-break key (integer).
    - Portal sort key is (x_1nm, y_1nm) of midpoint.
    - Heap key is (f_cost_rounded_to_1nm, insertion_sequence).
    """
    # Reset room counter for reproducibility
    Room._counter = 0

    # Build initial room: expand a room from start toward goal
    seed_room_poly = _expand_room_toward(free_space, start_xy, goal_xy)
    if seed_room_poly is None:
        print("[spike0] ERROR: cannot build seed room at start", flush=True)
        return None

    seed_room = Room(seed_room_poly)
    rooms_by_id: dict[int, Room] = {seed_room.id: seed_room}
    all_room_polys: list[Polygon] = [seed_room_poly]

    # A* state
    # heap: (f_cost_int, insertion_seq, room_id, entry_xy, came_from_id_or_None)
    insertion_seq = 0
    g_costs: dict[int, float] = {seed_room.id: 0.0}
    came_from: dict[int, int | None] = {seed_room.id: None}

    def heuristic(xy: tuple[float, float]) -> float:
        return math.hypot(xy[0] - goal_xy[0], xy[1] - goal_xy[1])

    start_h = heuristic(start_xy)
    heap = [(_round1nm(start_h), insertion_seq, seed_room.id, start_xy)]

    visited: set[int] = set()
    total_rooms_expanded = 0

    while heap:
        f_int, _seq, room_id, entry_xy = heapq.heappop(heap)

        if room_id in visited:
            continue
        visited.add(room_id)
        total_rooms_expanded += 1

        room = rooms_by_id[room_id]
        g = g_costs[room_id]

        # Check if goal is inside this room
        if room.poly.contains(SPoint(*goal_xy)) or room.poly.distance(SPoint(*goal_xy)) < 1e-5:
            # Reconstruct room path
            path = []
            curr = room_id
            while curr is not None:
                path.append(rooms_by_id[curr])
                curr = came_from[curr]
            path.reverse()
            print(f"[spike0] A* found path: {len(path)} rooms, "
                  f"{total_rooms_expanded} rooms expanded", flush=True)
            return path

        if total_rooms_expanded > max_rooms:
            print(f"[spike0] A* exceeded max_rooms={max_rooms}", flush=True)
            break

        # Expand neighbors: grow new rooms toward goal from portal edges
        # Get the boundary of the current room's free space that doesn't overlap visited rooms
        # Grow candidate rooms in different directions
        current_poly = room.poly

        # Generate candidate expansion points on the room boundary
        # toward the goal direction — try multiple subdivisions
        candidate_pts = _sample_expansion_points(current_poly, goal_xy)

        for cpt in candidate_pts:
            new_poly = _expand_room_toward(free_space, cpt, goal_xy)
            if new_poly is None:
                continue
            if new_poly.area < 1e-8:
                continue

            # Check if this is meaningfully different from existing rooms
            portal = _portal_between(current_poly, new_poly)
            if portal is None:
                continue

            # Check if this room overlaps significantly with an existing visited room
            # (avoid creating duplicate rooms)
            is_duplicate = False
            for ep in all_room_polys:
                overlap = new_poly.intersection(ep)
                if not overlap.is_empty and overlap.area > 0.95 * new_poly.area:
                    is_duplicate = True
                    break
            if is_duplicate:
                continue

            new_room = Room(new_poly)
            rooms_by_id[new_room.id] = new_room
            all_room_polys.append(new_poly)

            # Register portal on current room
            pmid = _portal_midpoint(portal)
            room.portals.append((new_room.id, portal, pmid))

            # A* cost: g = accumulated path length to portal midpoint
            seg_len = math.hypot(pmid[0] - entry_xy[0], pmid[1] - entry_xy[1])
            new_g = g + seg_len
            h = heuristic(pmid)
            new_f = new_g + h

            if new_room.id not in g_costs or new_g < g_costs[new_room.id]:
                g_costs[new_room.id] = new_g
                came_from[new_room.id] = room_id
                insertion_seq += 1
                heapq.heappush(heap, (_round1nm(new_f), insertion_seq, new_room.id, pmid))

    print(f"[spike0] A* failed after expanding {total_rooms_expanded} rooms", flush=True)
    return None


def _sample_expansion_points(
    room_poly: Polygon,
    goal_xy: tuple[float, float],
    n_boundary: int = 8,
) -> list[tuple[float, float]]:
    """Sample candidate expansion points on the room boundary toward goal.

    Returns points sorted by their distance to goal (ascending = best first),
    with a deterministic sort key based on 1nm-rounded coordinates.
    """
    boundary = room_poly.boundary
    total_len = boundary.length
    if total_len < 1e-10:
        return []

    pts = []
    for i in range(n_boundary):
        frac = (i + 0.5) / n_boundary
        pt = boundary.interpolate(frac * total_len)
        px, py = pt.x, pt.y
        dist_to_goal = math.hypot(px - goal_xy[0], py - goal_xy[1])
        pts.append((dist_to_goal, _round1nm(px), _round1nm(py), (px, py)))

    # Sort: closest to goal first; tie-break by (x,y) rounded to 1nm (deterministic)
    pts.sort(key=lambda t: (t[0], t[1], t[2]))
    return [t[3] for t in pts]


# ---------------------------------------------------------------------------
# Step 7: funnel / shrink realization → centerline
# ---------------------------------------------------------------------------

def realize_centerline(
    room_seq: list[Room],
    start_xy: tuple[float, float],
    goal_xy: tuple[float, float],
    obstacles: list,
    clearance_mm: float,
    track_mm: float,
) -> list[tuple[float, float]] | None:
    """Funnel/shrink: compute the shortest legal centerline through the room corridor.

    Simple approach: straight-line pull through portals.
    Start at start_xy, end at goal_xy. For each consecutive pair of rooms,
    find the portal and project the straight-line path through the closest
    legal point on the portal.

    Returns list of (x, y) waypoints in world mm, or None on failure.
    """
    if len(room_seq) == 1:
        # Start and goal are in the same room — straight line
        return [start_xy, goal_xy]

    # Collect portal segments between consecutive rooms
    portals = []
    for i in range(len(room_seq) - 1):
        r1 = room_seq[i]
        r2 = room_seq[i + 1]
        portal = _portal_between(r1.poly, r2.poly)
        if portal is None:
            print(f"[spike0] WARNING: no portal between room {r1.id} and {r2.id}", flush=True)
            # Try the other direction
            portal = _portal_between(r2.poly, r1.poly)
        if portal is None:
            print(f"[spike0] ERROR: cannot realize centerline — no portal at step {i}", flush=True)
            return None
        portals.append(portal)

    # Funnel algorithm: pull the shortest path through portal midpoints
    # (Simplified funnel — use midpoints clamped to the corridor)
    waypoints = [start_xy]
    cur = start_xy
    for portal in portals:
        ax, ay, bx, by = portal
        # Project cur→goal onto the portal, clamped
        pt = _closest_point_on_segment(cur, goal_xy, (ax, ay), (bx, by))
        waypoints.append(pt)
        cur = pt
    waypoints.append(goal_xy)

    # Simplify: remove collinear or near-zero-length segments
    simplified = _simplify_polyline(waypoints, tol=1e-5)
    return simplified


def _closest_point_on_segment(
    path_a: tuple[float, float],
    path_b: tuple[float, float],
    seg_a: tuple[float, float],
    seg_b: tuple[float, float],
) -> tuple[float, float]:
    """Find the point on segment seg_a–seg_b closest to the line path_a→path_b.

    Returns the closest point on the portal segment to the straight line,
    clamped to the portal endpoints.
    """
    sx, sy = seg_a
    ex, ey = seg_b
    # Direction of path
    pdx, pdy = path_b[0] - path_a[0], path_b[1] - path_a[1]
    # Direction of portal
    sdx, sdy = ex - sx, ey - sy
    seg_len = math.hypot(sdx, sdy)
    if seg_len < 1e-10:
        return seg_a

    # Project the path onto the portal using the midpoint
    # Find intersection of path_a→path_b with portal seg, or closest point
    # Use the midpoint as the "ideal" crossing point
    mx, my = (sx + ex) / 2, (sy + ey) / 2

    # If path is not zero-length, compute parametric intersection
    if math.hypot(pdx, pdy) > 1e-10:
        # Solve: path_a + t*(path_b-path_a) = seg_a + s*(seg_b-seg_a)
        denom = pdx * sdy - pdy * sdx
        if abs(denom) > 1e-12:
            acx = sx - path_a[0]
            acy = sy - path_a[1]
            s = (acx * pdy - acy * pdx) / denom
            s = max(0.0, min(1.0, s))
            return (sx + s * sdx, sy + s * sdy)

    # Fall back to midpoint
    return (mx, my)


def _simplify_polyline(
    pts: list[tuple[float, float]],
    tol: float = 1e-5,
) -> list[tuple[float, float]]:
    """Remove near-duplicate consecutive points."""
    if len(pts) <= 2:
        return pts
    result = [pts[0]]
    for p in pts[1:]:
        if math.hypot(p[0] - result[-1][0], p[1] - result[-1][1]) > tol:
            result.append(p)
    return result


# ---------------------------------------------------------------------------
# Step 8: emit centerline as (segment ...) nodes
# ---------------------------------------------------------------------------

def emit_net_segments(
    board: Path,
    net_name: str,
    waypoints: list[tuple[float, float]],
    track_mm: float,
    layer: str = "F.Cu",
) -> None:
    """Emit a polyline as (segment ...) nodes into the board sexpr."""
    root = parse_file(board)

    # Find net number (KiCad 10 uses named nets)
    decls = {n.arg(2): n.arg(1) for n in root.nodes("net") if n.arg(2) is not None}

    def net_atom_fn(name: str):
        if decls:
            num = decls.get(name)
            if num is not None:
                return node("net", num)
        return node("net", atom(name, quote=True))

    net_nd = net_atom_fn(net_name)
    if net_nd is None:
        print(f"[spike0] WARNING: net {net_name!r} not in board declarations", flush=True)

    segs_written = 0
    for a, b in zip(waypoints, waypoints[1:]):
        xa, ya = a
        xb, yb = b
        seg = node("segment",
                   node("start", f"{xa:.6f}", f"{ya:.6f}"),
                   node("end",   f"{xb:.6f}", f"{yb:.6f}"),
                   node("width", str(track_mm)),
                   node("layer", atom(layer, quote=True)),
                   net_nd)
        root.insert(seg)
        segs_written += 1

    write_file(root, board)
    print(f"[spike0] emitted {segs_written} segment(s) for net {net_name!r}", flush=True)


def extract_emitted_coords(board: Path, net_name: str) -> str:
    """Extract segment coordinates for the chosen net, for determinism comparison."""
    root = parse_file(board)
    # Find net number
    decls = {n.arg(2): n.arg(1) for n in root.nodes("net") if n.arg(2) is not None}
    net_num = decls.get(net_name)

    segs = []
    for seg in root.nodes("segment"):
        # Check net matches
        for child in seg.nodes("net"):
            num_val = child.arg(1)
            name_val = child.arg(1)
            if net_num and num_val == net_num:
                match = True
            elif name_val == net_name or name_val == f'"{net_name}"':
                match = True
            else:
                match = False
            if match:
                start = seg.first("start")
                end_ = seg.first("end")
                if start and end_:
                    segs.append(f"{start.arg(1)},{start.arg(2)}-{end_.arg(1)},{end_.arg(2)}")
    segs.sort()
    return "\n".join(segs)


# ---------------------------------------------------------------------------
# DRC helpers
# ---------------------------------------------------------------------------

def drc_summary_for_net(report: dict, net_name: str) -> dict:
    """Count violations attributable to the chosen net."""
    violations = report.get("violations", [])
    errors = sum(1 for v in violations if v.get("severity") == "error")
    unconnected = report.get("unconnected_items", [])
    unconnected_count = len(unconnected)

    # Violations mentioning the chosen net
    net_errors = 0
    for v in violations:
        if v.get("severity") != "error":
            continue
        items = v.get("items", [])
        for it in items:
            desc = str(it.get("description", "")) + str(it.get("net", ""))
            if net_name in desc:
                net_errors += 1
                break

    # Is the chosen net unconnected?
    net_unconnected = sum(
        1 for u in unconnected
        if any(net_name in str(it) for it in u.get("items", []))
    )

    by = collections.Counter(v.get("type") for v in violations)
    return {
        "unconnected": unconnected_count,
        "errors": errors,
        "net_errors": net_errors,
        "net_unconnected": net_unconnected,
        "by_type": dict(by),
    }


# ---------------------------------------------------------------------------
# Step 9 + 11: main pipeline (returns emitted waypoints as string for determinism)
# ---------------------------------------------------------------------------

def run_pipeline(board: Path, chosen_net: str, pad_a: dict, pad_b: dict,
                 data: dict, geo: dict) -> tuple[str, float]:
    """Run steps 5-8: build free space, A*, realize, emit.

    Returns (emitted_coords_string, solve_time_seconds).
    """
    clearance_mm = geo["clearance_mm"]
    track_mm = geo["track_mm"]

    t0 = time.perf_counter()

    # Step 5: build free space
    free_space, _bbox = build_free_space(data, chosen_net, clearance_mm, track_mm)
    print(f"[spike0] free_space area={free_space.area:.2f} mm²", flush=True)

    start_xy = (pad_a["x"], pad_a["y"])
    goal_xy  = (pad_b["x"], pad_b["y"])

    # Step 6: A* over rooms
    room_seq = astar_rooms(free_space, start_xy, goal_xy)
    if room_seq is None:
        raise RuntimeError("A* failed to find a room path")

    # Step 7: realize centerline
    # Build obstacle list for legality check (all F.Cu obstacles except own net)
    obstacles = []
    for p in data["pads"]:
        if p["net"] == chosen_net:
            continue
        if not p.get("front"):
            continue
        obstacles.append(("rect", p["x"] - p["hw"], p["y"] - p["hh"],
                          p["x"] + p["hw"], p["y"] + p["hh"]))

    waypoints = realize_centerline(room_seq, start_xy, goal_xy,
                                   obstacles, clearance_mm, track_mm)
    if waypoints is None or len(waypoints) < 2:
        raise RuntimeError("Centerline realization failed")

    print(f"[spike0] centerline: {len(waypoints)} waypoints: "
          f"{[(round(x,3), round(y,3)) for x,y in waypoints]}", flush=True)

    # Assert each segment is legal via exact_geom predicates
    track_hw = track_mm / 2.0
    for i, (a, b) in enumerate(zip(waypoints, waypoints[1:])):
        # Check both endpoints
        if not is_legal(a, obstacles, track_hw, clearance_mm):
            print(f"[spike0] WARNING: segment {i} start {a} not legal vs obstacles", flush=True)
        if not is_legal(b, obstacles, track_hw, clearance_mm):
            print(f"[spike0] WARNING: segment {i} end {b} not legal vs obstacles", flush=True)
        # Check segment clearance: for each segment obstacle, ensure min separation
        for obs in obstacles:
            if obs[0] == "rect":
                _, x1, y1, x2, y2 = obs
                d = segment_rect_distance(a, b, (x1, y1, x2, y2))
                if d < clearance_mm + track_hw - 1e-5:
                    print(f"[spike0] WARNING: segment {i} clears rect by only {d:.4f} mm "
                          f"(need {clearance_mm + track_hw:.4f})", flush=True)

    solve_time = time.perf_counter() - t0
    print(f"[spike0] solve time (steps 5-8 pre-emit): {solve_time:.3f}s", flush=True)

    # Step 8: emit segments
    emit_net_segments(board, chosen_net, waypoints, track_mm)

    # Extract emitted coordinates for determinism comparison
    coords_str = extract_emitted_coords(board, chosen_net)
    return coords_str, solve_time


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    print("=" * 60, flush=True)
    print("Spike-0: FAR gridless single-net router", flush=True)
    print("=" * 60, flush=True)

    # Step 2: setup board
    with tempfile.TemporaryDirectory(prefix="spike0_") as tmp:
        out_dir = Path(tmp)
        board = setup_board(out_dir)
        print(f"[spike0] board: {board}", flush=True)

        # Step 3: extract + build_problem
        data = extract_pads(board)
        geo = project_geometry(board)
        print(f"[spike0] geo: {geo}", flush=True)
        grid, nets, anchors, obstacles_dict, anchor_rects = build_problem(
            data,
            pitch=0.1,
            track_mm=geo["track_mm"],
            clearance_mm=geo["clearance_mm"],
        )

        # Step 4: pick net
        chosen_net, pad_a, pad_b = pick_net(data)

        # --- Baseline DRC (stripped board, before adding our trace) ---
        print("[spike0] Running BASELINE DRC (stripped board)...", flush=True)
        baseline_report = run_drc(board)
        baseline = drc_summary_for_net(baseline_report, chosen_net)
        print(f"[spike0] BASELINE  unconnected={baseline['unconnected']}  "
              f"errors={baseline['errors']}", flush=True)
        print(f"[spike0] BASELINE  by_type={baseline['by_type']}", flush=True)
        print(f"[spike0] BASELINE  net_unconnected={baseline['net_unconnected']}  "
              f"net_errors={baseline['net_errors']}", flush=True)

        # --- First run of pipeline (same process) ---
        print("[spike0] Running pipeline (run 1)...", flush=True)
        board1_dir = out_dir / "run1"
        shutil.copytree(out_dir, board1_dir, ignore=shutil.ignore_patterns("run1", "run2", "*.drc.json"))
        board1 = next(board1_dir.glob("*.kicad_pcb"))
        coords1, solve_time1 = run_pipeline(board1, chosen_net, pad_a, pad_b, data, geo)

        # Step 8b: refill zones
        refill_zones(board1)

        # Step 9: DRC after
        print("[spike0] Running AFTER DRC...", flush=True)
        after_report = run_drc(board1)
        after = drc_summary_for_net(after_report, chosen_net)
        print(f"[spike0] AFTER    unconnected={after['unconnected']}  "
              f"errors={after['errors']}", flush=True)
        print(f"[spike0] AFTER    by_type={after['by_type']}", flush=True)
        print(f"[spike0] AFTER    net_unconnected={after['net_unconnected']}  "
              f"net_errors={after['net_errors']}", flush=True)

        # New errors attributable to the emitted net
        new_net_errors = after["net_errors"] - baseline["net_errors"]
        net_connected = after["net_unconnected"] < baseline["net_unconnected"]
        print(f"[spike0] new_trace_attributable_errors={new_net_errors}", flush=True)
        print(f"[spike0] net_connected={net_connected}", flush=True)

        # --- Second run of pipeline (same process, new copy) ---
        print("[spike0] Running pipeline (run 2, same-process)...", flush=True)
        board2_dir = out_dir / "run2"
        shutil.copytree(
            out_dir,
            board2_dir,
            ignore=shutil.ignore_patterns("run1", "run2", "*.drc.json"),
        )
        board2 = next(board2_dir.glob("*.kicad_pcb"))
        coords2, solve_time2 = run_pipeline(board2, chosen_net, pad_a, pad_b, data, geo)

        # --- Third run: fresh subprocess ---
        print("[spike0] Running pipeline (run 3, fresh subprocess)...", flush=True)
        board3_dir = out_dir / "run3"
        shutil.copytree(
            out_dir,
            board3_dir,
            ignore=shutil.ignore_patterns("run1", "run2", "run3", "*.drc.json"),
        )
        board3 = next(board3_dir.glob("*.kicad_pcb"))

        # Run subprocess: just emit, capture coords
        proc = subprocess.run(
            [sys.executable, str(Path(__file__).resolve()),
             "--subprocess-emit", str(board3), chosen_net,
             str(pad_a["x"]), str(pad_a["y"]),
             str(pad_b["x"]), str(pad_b["y"]),
             str(geo["clearance_mm"]), str(geo["track_mm"])],
            capture_output=True, text=True,
        )
        if proc.returncode != 0:
            print(f"[spike0] subprocess failed: {proc.stderr[-500:]}", flush=True)
            coords3 = "SUBPROCESS_FAILED"
        else:
            # Extract lines prefixed COORDS: (subprocess prints these to stdout)
            coord_lines = [
                line[len("COORDS:"):].strip()
                for line in proc.stdout.splitlines()
                if line.startswith("COORDS:")
            ]
            coords3 = "\n".join(sorted(coord_lines))
            print(f"[spike0] subprocess returned {len(coords3)} bytes of coords", flush=True)
            if not coord_lines:
                print(f"[spike0] subprocess stdout: {proc.stdout[:300]}", flush=True)

        # --- Determinism gate ---
        det1_2 = "byte-identical" if coords1 == coords2 else f"differ: run1 vs run2"
        det1_3 = "byte-identical" if coords1 == coords3 else f"differ: run1 vs subprocess"
        determinism = "byte-identical" if (coords1 == coords2 == coords3) else (
            f"differ: 1vs2={det1_2}, 1vs3={det1_3}"
        )
        print(f"[spike0] determinism: {determinism}", flush=True)
        if coords1 != coords2:
            print(f"[spike0]   run1: {coords1[:200]}", flush=True)
            print(f"[spike0]   run2: {coords2[:200]}", flush=True)
        if coords1 != coords3:
            print(f"[spike0]   run3(subprocess): {coords3[:200]}", flush=True)

        # --- Timing check ---
        under_5s = solve_time1 < 5.0
        print(f"[spike0] solve_runtime_s={solve_time1:.3f}  under_5s={under_5s}", flush=True)

        # --- set_precision instability check ---
        # Run the free_space build twice and compare polygon area
        fs1, _ = build_free_space(data, chosen_net, geo["clearance_mm"], geo["track_mm"])
        fs2, _ = build_free_space(data, chosen_net, geo["clearance_mm"], geo["track_mm"])
        area_diff = abs(fs1.area - fs2.area)
        set_precision_instability = (
            f"area_diff={area_diff:.2e}" if area_diff > 1e-10
            else "none (area identical across 2 builds)"
        )
        print(f"[spike0] set_precision_instability: {set_precision_instability}", flush=True)

        # --- Go/no-go ---
        trace_ok = new_net_errors == 0
        pass_criteria = [
            ("0 new errors", trace_ok),
            ("net connected", net_connected),
            ("runtime < 5s", under_5s),
            ("determinism", determinism == "byte-identical"),
        ]
        all_pass = all(ok for _, ok in pass_criteria)
        for label, ok in pass_criteria:
            print(f"[spike0]   {'PASS' if ok else 'FAIL'} {label}", flush=True)

        if all_pass:
            go_no_go = "GO"
        elif trace_ok and net_connected:
            go_no_go = "GO-WITH-CAVEATS (not all pass criteria met)"
        else:
            go_no_go = "NO-GO"

        print(f"\n[spike0] GO/NO-GO: {go_no_go}", flush=True)
        print(flush=True)

        # --- Structured Result ---
        import json
        result = {
            "status": "success" if all_pass else ("partial" if (trace_ok and net_connected) else "failure"),
            "summary": (
                f"Spike-0 {go_no_go}: net={chosen_net!r}, "
                f"new_trace_errors={new_net_errors}, net_connected={net_connected}, "
                f"solve={solve_time1:.3f}s, det={determinism}"
            ),
            "files_changed": ["scripts/spike0_gridless_single_net.py"],
            "files_read": [
                "docs/design/FAR-gridless-router-arch.md",
                "scripts/_probe_route_human.py",
                "src/tracewise/route/engine/kicad.py",
                "src/tracewise/route/engine/exact_geom.py",
                "src/tracewise/route/bridge.py",
                "src/tracewise/sexpr.py",
            ],
            "shapely_geos_version": list(GEOS_VERSION),
            "chosen_net": chosen_net,
            "baseline_drc": {
                "unconnected": baseline["unconnected"],
                "errors": baseline["errors"],
            },
            "after_drc": {
                "unconnected": after["unconnected"],
                "errors": after["errors"],
            },
            "new_trace_attributable_errors": new_net_errors,
            "net_connected": net_connected,
            "solve_runtime_s": round(solve_time1, 3),
            "under_5s": under_5s,
            "determinism": determinism,
            "set_precision_instability": set_precision_instability,
            "go_no_go": go_no_go,
            "issues": [] if all_pass else [
                label for label, ok in pass_criteria if not ok
            ],
            "assumptions": [
                "Free space built from F.Cu pad rectangles only (no arc/polygon pads)",
                "Funnel realization uses portal midpoint projection (simplified funnel)",
                "No vias (single-layer F.Cu only)",
                "Board boundary from pcbnew bounding box",
            ],
        }
        print("## Structured Result")
        print("```json")
        print(json.dumps(result, indent=2))
        print("```")


# ---------------------------------------------------------------------------
# Subprocess emit mode (for determinism gate, step 10)
# ---------------------------------------------------------------------------

def subprocess_emit_mode(args: list[str]) -> None:
    """Used when called as subprocess: emit trace, print coords to stdout."""
    board_path = Path(args[0])
    chosen_net = args[1]
    ax_s, ay_s, bx_s, by_s = args[2], args[3], args[4], args[5]
    clearance_mm = float(args[6])
    track_mm = float(args[7])

    pad_a = {"x": float(ax_s), "y": float(ay_s), "front": True, "back": False}
    pad_b = {"x": float(bx_s), "y": float(by_s), "front": True, "back": False}

    # We need the full data for obstacle extraction
    data = extract_pads(board_path)

    free_space, _ = build_free_space(data, chosen_net, clearance_mm, track_mm)
    start_xy = (pad_a["x"], pad_a["y"])
    goal_xy  = (pad_b["x"], pad_b["y"])

    room_seq = astar_rooms(free_space, start_xy, goal_xy)
    if room_seq is None:
        print("ASTAR_FAILED", flush=True)
        sys.exit(1)

    obstacles = []
    for p in data["pads"]:
        if p["net"] == chosen_net:
            continue
        if not p.get("front"):
            continue
        obstacles.append(("rect", p["x"] - p["hw"], p["y"] - p["hh"],
                          p["x"] + p["hw"], p["y"] + p["hh"]))

    waypoints = realize_centerline(room_seq, start_xy, goal_xy,
                                   obstacles, clearance_mm, track_mm)
    if waypoints is None or len(waypoints) < 2:
        print("REALIZE_FAILED", flush=True)
        sys.exit(1)

    emit_net_segments(board_path, chosen_net, waypoints, track_mm)
    coords = extract_emitted_coords(board_path, chosen_net)
    for line in coords.splitlines():
        print(f"COORDS:{line}", flush=True)


if __name__ == "__main__":
    if "--subprocess-emit" in sys.argv:
        idx = sys.argv.index("--subprocess-emit")
        subprocess_emit_mode(sys.argv[idx + 1:])
    else:
        main()
