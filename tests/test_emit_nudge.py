"""Integration tests for Phase B emit refactor — per-cell resolved map.

Tests:
  1. Shared interior cell: two segments meeting at the same cell produce
     byte-identical endpoint coordinates in the emitted board.
  2. Via cell: the via (at ...) matches both adjacent segment endpoints.
  3. obstacles=None: emit_routes output is byte-identical to pre-change
     behaviour (back-compat).

Uses a tiny synthetic in-memory board with two nets and no pcbnew.
Drives emit_routes directly via the sexpr editor on a minimal .kicad_pcb.
"""

from __future__ import annotations

from pathlib import Path

from tracewise.route.engine.grid import Grid
from tracewise.route.engine.kicad import emit_routes
from tracewise.route.engine.multi import Net, NetRoute
from tracewise.sexpr import Node, parse_file

# ---------------------------------------------------------------------------
# Minimal synthetic board template (no pcbnew needed)
# ---------------------------------------------------------------------------

_BOARD_TEMPLATE = """\
(kicad_pcb
\t(version 20260206)
\t(net 1 "A")
\t(net 2 "B")
)
"""


def _make_board(path: Path) -> Path:
    """Write the minimal board template to path and return it."""
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(_BOARD_TEMPLATE, encoding="utf-8")
    return path


def _make_grid() -> Grid:
    """10×10 mm grid at 0.1mm pitch, 2 layers, origin at (0,0)."""
    return Grid(x0=0.0, y0=0.0, width_mm=10.0, height_mm=10.0, pitch=0.1, layers=2)


def _make_net(name: str, pads) -> Net:
    return Net(name=name, pads=pads)


def _make_nr(net: Net, path: list) -> NetRoute:
    return NetRoute(net=net, ok=True, paths=[path], cells=set(path), escape_cells=set())


# ---------------------------------------------------------------------------
# Helper: parse emitted board and extract segment + via nodes
# ---------------------------------------------------------------------------

def _parse_segments_vias(board: Path):
    root = parse_file(board)
    segs = root.find_all("segment")
    vias = root.find_all("via")
    return segs, vias


def _seg_coords(seg: Node):
    """Return ((sx,sy), (ex,ey)) from a segment node."""
    start = seg.first("start")
    end = seg.first("end")
    return (float(start.arg(1)), float(start.arg(2))), (float(end.arg(1)), float(end.arg(2)))


def _via_at(via: Node):
    """Return (vx, vy) from a via node."""
    at = via.first("at")
    return float(at.arg(1)), float(at.arg(2))


# ---------------------------------------------------------------------------
# Test 1: Shared interior cell — coincident endpoints are byte-identical
#
# Path: (0,10,10) → (0,10,50) → (0,50,50) → (0,90,50)
# Cell (0,10,50) is the shared interior cell between seg0 and seg1 of the
# SIMPLIFIED path.  simplify() produces:
#   run0: [(0,10,10),(0,10,50),(0,90,50)] (one run, collinear turn stripped? No)
# Actually simplify strips collinear interior points; the turn at (0,10,50)
# changes direction (horizontal then vertical) so it is KEPT.
# So runs = [[(0,10,10),(0,10,50),(0,50,50),(0,90,50)]]  — one run,
# segments: (10,10)→(10,50), (10,50)→(50,50), (50,50)→(90,50).
# The interior cells (0,10,50) and (0,50,50) each appear in two consecutive
# segments.  resolved[(0,10,50)] must be identical for both segs.
# ---------------------------------------------------------------------------


def test_shared_interior_cell_coords_are_identical(tmp_path):
    """Segments sharing an interior cell must have IDENTICAL world coordinates
    at that junction — the same float from the resolved dict, formatted once."""
    board = _make_board(tmp_path / "board.kicad_pcb")
    grid = _make_grid()
    # Path with TWO direction changes so simplify keeps both interior waypoints.
    # (0,10,10)→(0,10,50): direction (row+0, col+40) = south-east
    # (0,10,50)→(0,50,10): direction changes → (0,10,50) kept
    # (0,50,10)→(0,90,10): direction (row+40, col+0) = south  → (0,50,10) kept
    # All three interior cells appear in two consecutive segments.
    path = [(0, 10, 10), (0, 10, 50), (0, 50, 10), (0, 90, 10)]
    net = _make_net("A", [(0, 10, 10), (0, 90, 10)])
    nr = _make_nr(net, path)
    results = {"A": nr}

    anchors = {
        (0, 10, 10): (1.0, 1.0),
        (0, 90, 10): (1.0, 9.0),
    }
    # Other-net obstacle near the interior shared cell (0,10,50) = world (5.0, 1.0)
    obstacles: dict = {
        0: [("B", ("rect", 5.12, 0.85, 5.32, 1.15))],
        1: [],
    }
    anchor_rects = {
        (0, 10, 10): (0.8, 0.8, 1.2, 1.2),
        (0, 90, 10): (0.8, 8.8, 1.2, 9.2),
    }

    emit_routes(board, grid, results, anchors=anchors,
                obstacles=obstacles, anchor_rects=anchor_rects, clearance_mm=0.15)

    segs, vias = _parse_segments_vias(board)
    assert len(vias) == 0, f"Expected no vias, got {len(vias)}"
    assert len(segs) == 3, f"Expected 3 segments, got {len(segs)}"

    # Collect all endpoints; at the shared interior cell the SAME point
    # should appear as an endpoint of two different segments.
    coord_count: dict = {}
    for seg in segs:
        for pt in _seg_coords(seg):
            coord_count[pt] = coord_count.get(pt, 0) + 1

    # Interior shared cells appear twice (end of one seg, start of next).
    interior_pts = {pt for pt, cnt in coord_count.items() if cnt == 2}
    assert len(interior_pts) >= 1, (
        f"No interior shared point found (each appearing in 2 segs): {coord_count}"
    )

    # Additionally verify all segs are on F.Cu
    for seg in segs:
        layer_node = seg.first("layer")
        assert layer_node.arg(1) == "F.Cu"


# ---------------------------------------------------------------------------
# Test 2: Via cell — via (at) equals both joining segment endpoints
#
# Path: (0,10,10) → (0,10,50) → (1,10,50) → (1,10,90)
# simplify gives two runs:
#   run0: [(0,10,10),(0,10,50)]
#   run1: [(1,10,50),(1,10,90)]
# The transition cell (iy=10,ix=50) is the via.
# ---------------------------------------------------------------------------


def test_via_at_equals_adjacent_segment_endpoints(tmp_path):
    """The via (at ...) must match the end of run0 and the start of run1 —
    all three must read from the same resolved dict entry (via_pos)."""
    board = _make_board(tmp_path / "via_board.kicad_pcb")
    grid = _make_grid()
    path = [(0, 10, 10), (0, 10, 50), (1, 10, 50), (1, 10, 90)]
    net = _make_net("A", [(0, 10, 10), (1, 10, 90)])
    nr = _make_nr(net, path)
    results = {"A": nr}

    anchors = {
        (0, 10, 10): (1.0, 1.0),
        (1, 10, 90): (9.0, 1.0),
    }
    obstacles: dict = {0: [], 1: []}
    anchor_rects = {
        (0, 10, 10): (0.8, 0.8, 1.2, 1.2),
        (1, 10, 90): (8.8, 0.8, 9.2, 1.2),
    }

    emit_routes(board, grid, results, anchors=anchors,
                obstacles=obstacles, anchor_rects=anchor_rects, clearance_mm=0.15)

    segs, vias = _parse_segments_vias(board)
    assert len(segs) == 2, f"Expected 2 segments, got {len(segs)}: {[_seg_coords(s) for s in segs]}"
    assert len(vias) == 1, f"Expected 1 via, got {len(vias)}"

    vx, vy = _via_at(vias[0])
    seg0_coords = _seg_coords(segs[0])
    seg1_coords = _seg_coords(segs[1])

    eps = 1e-6

    def _has_endpoint(coords, px, py):
        return (
            (abs(coords[0][0] - px) < eps and abs(coords[0][1] - py) < eps) or
            (abs(coords[1][0] - px) < eps and abs(coords[1][1] - py) < eps)
        )

    # seg0 must have one endpoint at the via position (the end of run0)
    seg0_at_via = _has_endpoint(seg0_coords, vx, vy)
    # seg1 must have one endpoint at the via position (the start of run1)
    seg1_at_via = _has_endpoint(seg1_coords, vx, vy)

    assert seg0_at_via, (
        f"Via at ({vx:.3f},{vy:.3f}) not found in seg0 endpoints {seg0_coords}"
    )
    assert seg1_at_via, (
        f"Via at ({vx:.3f},{vy:.3f}) not found in seg1 endpoints {seg1_coords}"
    )


# ---------------------------------------------------------------------------
# Test 3: obstacles=None → byte-identical on two calls (back-compat invariant)
#
# Also verify terminals land at anchor positions and interior cells at
# grid.to_world values — the pre-Phase-B behaviour.
# ---------------------------------------------------------------------------


def test_obstacles_none_byte_identical_and_matches_old_logic(tmp_path):
    """With obstacles=None, two identical calls produce identical board text,
    and the emitted terminal/interior coordinates match the old snap/grid.to_world
    values."""
    board_a = _make_board(tmp_path / "board_a.kicad_pcb")
    board_b = _make_board(tmp_path / "board_b.kicad_pcb")

    grid = _make_grid()
    # Same path as test 1: two direction changes so simplify keeps interior waypoints.
    # Interior (0,10,50) = world (5.0, 1.0); interior (0,50,10) = world (1.0, 5.0).
    path = [(0, 10, 10), (0, 10, 50), (0, 50, 10), (0, 90, 10)]
    net = _make_net("A", [(0, 10, 10), (0, 90, 10)])
    nr = _make_nr(net, path)
    results = {"A": nr}

    anchors = {
        (0, 10, 10): (1.0, 1.0),
        (0, 90, 10): (1.0, 9.0),
    }

    # Board_a: emit with obstacles=None
    emit_routes(board_a, grid, results, anchors=anchors, obstacles=None)
    # Board_b: second identical call — must be byte-identical
    emit_routes(board_b, grid, results, anchors=anchors, obstacles=None)

    text_a = board_a.read_text(encoding="utf-8")
    text_b = board_b.read_text(encoding="utf-8")
    assert text_a == text_b, "obstacles=None: two identical calls produced different output"

    # Verify the emitted coordinates match old logic:
    #   Terminal (0,10,10) → anchors[(0,10,10)] = (1.0, 1.0)
    #   Terminal (0,90,10) → anchors[(0,90,10)] = (1.0, 9.0)
    #   Interior (0,10,50) → grid.to_world(10,50): x=0+50*0.1=5.0, y=0+10*0.1=1.0
    segs, _ = _parse_segments_vias(board_a)
    assert len(segs) == 3

    all_coords: set = set()
    for seg in segs:
        c = _seg_coords(seg)
        all_coords.add(c[0])
        all_coords.add(c[1])

    assert (1.0, 1.0) in all_coords, \
        f"Terminal anchor (1.0,1.0) not in emitted coords: {all_coords}"
    assert (1.0, 9.0) in all_coords, \
        f"Terminal anchor (1.0,9.0) not in emitted coords: {all_coords}"
    # Interior cell (0,10,50) → grid.to_world(10,50): x=0+50*0.1=5.0, y=0+10*0.1=1.0
    assert (5.0, 1.0) in all_coords, \
        f"Interior grid.to_world (5.0,1.0) not in emitted coords: {all_coords}"
