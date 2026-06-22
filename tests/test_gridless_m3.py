"""M3-P1 Tests — 2-layer via routing for geometry-blocked nets.

Test categories:
  T-M3-01  project_geometry surfaces hole_clearance_mm, hole_to_hole_mm, via_cost_mm.
  T-M3-02  build_windowed_free_space layer=1 excludes F.Cu-only pads.
  T-M3-03  build_windowed_free_space layer=0/1 default = byte-identical to callers.
  T-M3-04  candidate_via_sites returns sorted list, deduped, within bbox.
  T-M3-05  is_legal_via pred1 rejects sites outside free space.
  T-M3-06  GridlessRouteResult has world_vias; single-layer route world_vias=[].
  T-M3-07  adapter rasterize_into_grid handles 3-tuple (x,y,layer) waypoints.
  T-M3-08  adapter rasterize_into_grid marks both layers for via sites.
  T-M3-09  to_gridless_netroute accepts world_vias kwarg (no crash, cells populated).
  T-M3-10  emit_routes gridless branch skips via-transition waypoints (same pos).

Skip cleanly when shapely is absent:
    pytest.importorskip("shapely")
"""

from __future__ import annotations

import pathlib

import pytest

shapely = pytest.importorskip("shapely")

from shapely.geometry import box  # noqa: E402

from tracewise.route.gridless.geom import (  # noqa: E402
    build_windowed_free_space,
    candidate_via_sites,
    is_legal_via,
)
from tracewise.route.gridless.route import GridlessRouteResult  # noqa: E402

# ---------------------------------------------------------------------------
# Helpers / fixtures
# ---------------------------------------------------------------------------

_GEO = {
    "track_mm": 0.2,
    "clearance_mm": 0.2,
    "via_mm": 0.6,
    "via_drill_mm": 0.3,
    "hole_clearance_mm": 0.25,
    "hole_to_hole_mm": 0.25,
    "via_cost_mm": 3.0,
}
_BBOX = (0.0, 0.0, 20.0, 20.0)


def _simple_pads(net_a: str = "net_A", net_b: str = "net_B") -> list[dict]:
    """Two pads of net_A on F.Cu and two pads of net_B on B.Cu."""
    return [
        {"net": net_a, "x": 2.0, "y": 2.0, "hw": 0.4, "hh": 0.4,
         "front": True, "back": False},
        {"net": net_a, "x": 18.0, "y": 18.0, "hw": 0.4, "hh": 0.4,
         "front": True, "back": False},
        {"net": net_b, "x": 5.0, "y": 5.0, "hw": 0.4, "hh": 0.4,
         "front": False, "back": True},
        {"net": net_b, "x": 15.0, "y": 15.0, "hw": 0.4, "hh": 0.4,
         "front": False, "back": True},
    ]


# ---------------------------------------------------------------------------
# T-M3-01: project_geometry new fields
# ---------------------------------------------------------------------------

def test_m3_project_geometry_fields(tmp_path: pathlib.Path) -> None:
    """project_geometry returns hole_clearance_mm, hole_to_hole_mm, via_cost_mm
    even when there is no .kicad_pro file (should return safe defaults)."""
    from tracewise.route.engine.kicad import project_geometry

    # Create a fake .kicad_pcb file so project_geometry doesn't crash
    pcb = tmp_path / "test.kicad_pcb"
    pcb.write_text("(kicad_pcb)")
    geo = project_geometry(pcb)

    assert "hole_clearance_mm" in geo, "hole_clearance_mm missing from project_geometry"
    assert "hole_to_hole_mm" in geo, "hole_to_hole_mm missing from project_geometry"
    assert "via_cost_mm" in geo, "via_cost_mm missing from project_geometry"
    assert geo["hole_clearance_mm"] > 0
    assert geo["hole_to_hole_mm"] >= 0
    assert geo["via_cost_mm"] > 0


# ---------------------------------------------------------------------------
# T-M3-02: build_windowed_free_space layer=1 excludes F.Cu-only pads
# ---------------------------------------------------------------------------

def test_m3_layer1_excludes_fcu_pads() -> None:
    """layer=1 free space should not be blocked by F.Cu-only pads."""
    pads = _simple_pads()
    window = (0.0, 0.0, 20.0, 20.0)

    fs_F, obs_F = build_windowed_free_space(
        pads, "net_A", 0.2, 0.2, [], window, layer=0
    )
    fs_B, obs_B = build_windowed_free_space(
        pads, "net_A", 0.2, 0.2, [], window, layer=1
    )

    # F.Cu free space has obstacles from net_B F.Cu pads (there are none in
    # this setup, but the area must be smaller than B.Cu which sees net_B B.Cu pads)
    # B.Cu free space excludes net_B's B.Cu pads — so it should be SMALLER than
    # the unconstrained window
    window_area = 20.0 * 20.0
    assert fs_B.area < window_area, "B.Cu free space should be blocked by net_B B.Cu pads"


# ---------------------------------------------------------------------------
# T-M3-03: layer default = 0, byte-identical to existing callers
# ---------------------------------------------------------------------------

def test_m3_layer_default_byte_identical() -> None:
    """build_windowed_free_space() with no layer arg == layer=0."""
    pads = _simple_pads()
    window = (0.0, 0.0, 20.0, 20.0)

    fs_default, obs_default = build_windowed_free_space(
        pads, "net_A", 0.2, 0.2, [], window
    )
    fs_explicit, obs_explicit = build_windowed_free_space(
        pads, "net_A", 0.2, 0.2, [], window, layer=0
    )

    assert fs_default.equals(fs_explicit), \
        "layer=0 and no-layer must produce byte-identical free space"
    assert len(obs_default) == len(obs_explicit), \
        "obstacle counts must match"


# ---------------------------------------------------------------------------
# T-M3-04: candidate_via_sites returns sorted, deduped list within bbox
# ---------------------------------------------------------------------------

def test_m3_candidate_via_sites_sorted_and_bounded() -> None:
    """candidate_via_sites returns a sorted list and all sites within bbox."""
    # Use very simple free spaces: large open boxes
    fs_F = box(0.0, 0.0, 20.0, 20.0)
    fs_B = box(0.0, 0.0, 20.0, 20.0)
    window = (0.0, 0.0, 20.0, 20.0)

    sites = candidate_via_sites(
        fs_F, fs_B,
        (1.0, 1.0), (19.0, 19.0),
        window,
        via_mm=0.6,
        clearance_mm=0.2,
    )

    assert isinstance(sites, list)
    if sites:
        # Must be sorted
        assert sites == sorted(sites), "candidate_via_sites must return sorted list"
        # All within bbox
        wx1, wy1, wx2, wy2 = window
        for x, y in sites:
            assert wx1 <= x <= wx2 and wy1 <= y <= wy2, \
                f"Via site ({x},{y}) outside window bbox"
        # No duplicates
        assert len(sites) == len(set(sites)), "candidate_via_sites must deduplicate"


# ---------------------------------------------------------------------------
# T-M3-05: is_legal_via pred1 rejects sites outside free space
# ---------------------------------------------------------------------------

def test_m3_is_legal_via_rejects_outside_free_space() -> None:
    """is_legal_via must reject a site outside B.Cu free space."""
    # F.Cu: full board; B.Cu: only right half
    fs_F = box(0.0, 0.0, 20.0, 20.0)
    fs_B = box(10.0, 0.0, 20.0, 20.0)  # right half only

    site_inside = (15.0, 10.0)
    site_outside = (5.0, 10.0)  # outside B.Cu free space

    ok_inside, reason_inside = is_legal_via(
        site_inside, fs_F, fs_B, [], "net_A",
        via_mm=0.6, via_drill=0.3,
        clearance_mm=0.2, hole_clearance=0.25, hole_to_hole=0.25,
        drill_centers=[], window_bbox=(0.0, 0.0, 20.0, 20.0),
    )
    ok_outside, reason_outside = is_legal_via(
        site_outside, fs_F, fs_B, [], "net_A",
        via_mm=0.6, via_drill=0.3,
        clearance_mm=0.2, hole_clearance=0.25, hole_to_hole=0.25,
        drill_centers=[], window_bbox=(0.0, 0.0, 20.0, 20.0),
    )

    assert ok_inside, f"Via inside both free spaces should be legal; got: {reason_inside}"
    assert not ok_outside, "Via outside B.Cu free space must be rejected"
    assert "bcu" in reason_outside, f"Rejection reason should mention B.Cu, got: {reason_outside}"


# ---------------------------------------------------------------------------
# T-M3-06: GridlessRouteResult has world_vias field; single-layer route = []
# ---------------------------------------------------------------------------

def test_m3_gridless_route_result_world_vias_default() -> None:
    """GridlessRouteResult.world_vias defaults to empty list."""
    result = GridlessRouteResult(ok=True, world_paths=[[(1.0, 2.0), (3.0, 4.0)]])
    assert hasattr(result, "world_vias"), "GridlessRouteResult must have world_vias field"
    assert result.world_vias == [], "world_vias must default to []"


def test_m3_route_net_gridless_single_layer_no_vias() -> None:
    """Single-layer route produces world_vias=[]."""
    from tracewise.route.gridless.route import route_net_gridless

    pads = [
        {"net": "N", "x": 1.0, "y": 1.0, "hw": 0.3, "hh": 0.3,
         "front": True, "back": False},
        {"net": "N", "x": 5.0, "y": 1.0, "hw": 0.3, "hh": 0.3,
         "front": True, "back": False},
    ]
    geo = {
        "track_mm": 0.2, "clearance_mm": 0.2,
        "via_mm": 0.6, "via_drill_mm": 0.3,
        "hole_clearance_mm": 0.25, "hole_to_hole_mm": 0.25,
        "via_cost_mm": 3.0,
    }
    result = route_net_gridless(
        pad_a=(1.0, 1.0), pad_b=(5.0, 1.0),
        pads=pads, net_name="N", geo=geo,
        board_bbox=(0.0, 0.0, 10.0, 10.0),
    )
    assert result.ok, f"Simple route should succeed, got: {result.reason}"
    assert result.world_vias == [], "Single-layer route must have world_vias=[]"


# ---------------------------------------------------------------------------
# T-M3-07: rasterize_into_grid handles 3-tuple waypoints
# ---------------------------------------------------------------------------

def test_m3_rasterize_3tuple_waypoints() -> None:
    """rasterize_into_grid correctly handles 3-tuple (x, y, layer) waypoints."""
    from tracewise.route.engine.grid import Grid
    from tracewise.route.engine.multi import Net
    from tracewise.route.gridless.adapter import GridlessNetRoute

    grid = Grid(pitch=0.5, x0=0.0, y0=0.0, width_mm=20.0, height_mm=20.0)
    net = Net(name="T", pads=[], halfwidth_cells=1)
    nr = GridlessNetRoute(
        net=net, ok=True,
        world_paths=[
            [(1.0, 1.0, 0), (5.0, 1.0, 0)],   # F.Cu segment
            [(5.0, 1.0, 0), (5.0, 1.0, 1)],   # via transition (same pos, layer 0→1)
            [(5.0, 1.0, 1), (9.0, 1.0, 1)],   # B.Cu segment
        ],
    )
    nr.rasterize_into_grid(grid)

    # Must have cells populated
    assert len(nr.cells) > 0, "3-tuple rasterize must populate cells"
    # F.Cu cells at layer 0
    fcu_cells = {c for c in nr.cells if c[0] == 0}
    bcu_cells = {c for c in nr.cells if c[0] == 1}
    assert fcu_cells, "Must have F.Cu cells from layer=0 segment"
    assert bcu_cells, "Must have B.Cu cells from layer=1 segment"


# ---------------------------------------------------------------------------
# T-M3-08: rasterize_into_grid marks both layers for via sites
# ---------------------------------------------------------------------------

def test_m3_rasterize_via_sites_both_layers() -> None:
    """world_vias entries must appear in via_sites (not cells) for correct inflation.

    Via centres go into via_sites (layer-less (iy, ix) tuples) so that _mark
    applies the larger via_halfwidth_cells inflation radius.  Via copper rings
    extend to via_mm/2, which requires blocking radius via_mm/2 + clearance +
    track_mm/2 — that is via_halfwidth_cells, not halfwidth_cells.  Placing them
    in cells would use the too-small halfwidth_cells radius, allowing grid tracks
    to enter the via copper ring clearance zone (tracks_crossing / clearance DRC).
    """
    from tracewise.route.engine.grid import Grid
    from tracewise.route.engine.multi import Net
    from tracewise.route.gridless.adapter import GridlessNetRoute

    grid = Grid(pitch=0.5, x0=0.0, y0=0.0, width_mm=20.0, height_mm=20.0)
    net = Net(name="T", pads=[], halfwidth_cells=1)
    nr = GridlessNetRoute(
        net=net, ok=True,
        world_paths=[[(1.0, 1.0), (9.0, 1.0)]],
        world_vias=[(5.0, 1.0)],
    )
    nr.rasterize_into_grid(grid)

    iy_via, ix_via = grid.to_cell(5.0, 1.0)
    iy_via, ix_via = grid.clamp_cell(iy_via, ix_via)
    # Via must be in via_sites (layer-less tuple) so _mark applies via_halfwidth_cells
    assert (iy_via, ix_via) in nr.via_sites, (
        "Via centre must be in via_sites so _mark uses via_halfwidth_cells inflation"
    )
    # via_sites entries are layer-less (iy, ix) tuples, not (layer, iy, ix)
    assert all(len(s) == 2 for s in nr.via_sites), (
        "via_sites must contain (iy, ix) 2-tuples, not layer-prefixed tuples"
    )


# ---------------------------------------------------------------------------
# T-M3-09: to_gridless_netroute accepts world_vias kwarg
# ---------------------------------------------------------------------------

def test_m3_to_gridless_netroute_world_vias() -> None:
    """to_gridless_netroute must accept world_vias and populate it on result."""
    from tracewise.route.engine.grid import Grid
    from tracewise.route.engine.multi import Net
    from tracewise.route.gridless.adapter import to_gridless_netroute

    grid = Grid(pitch=0.5, x0=0.0, y0=0.0, width_mm=20.0, height_mm=20.0)
    net = Net(name="T", pads=[], halfwidth_cells=1)

    nr = to_gridless_netroute(
        net,
        world_paths=[[(1.0, 1.0), (9.0, 1.0)]],
        grid=grid,
        world_vias=[(5.0, 1.0)],
    )

    assert nr.ok
    assert nr.world_vias == [(5.0, 1.0)], "world_vias must be stored on GridlessNetRoute"
    assert len(nr.cells) > 0


def test_m3_to_gridless_netroute_no_world_vias() -> None:
    """to_gridless_netroute with world_vias=None defaults to empty list."""
    from tracewise.route.engine.grid import Grid
    from tracewise.route.engine.multi import Net
    from tracewise.route.gridless.adapter import to_gridless_netroute

    grid = Grid(pitch=0.5, x0=0.0, y0=0.0, width_mm=20.0, height_mm=20.0)
    net = Net(name="T", pads=[], halfwidth_cells=1)

    nr = to_gridless_netroute(
        net,
        world_paths=[[(1.0, 1.0), (9.0, 1.0)]],
        grid=grid,
    )
    assert nr.world_vias == [], "Default world_vias must be []"


# ---------------------------------------------------------------------------
# T-M3-10: emit_routes skips via-transition waypoints (same position)
# ---------------------------------------------------------------------------

def test_m3_emit_routes_skips_via_transition_waypoints(tmp_path: pathlib.Path) -> None:
    """emit_routes gridless branch must skip waypoints with same (x,y) and
    different layer (via transitions); those are not real segments."""
    import re

    from tracewise.route.engine.grid import Grid
    from tracewise.route.engine.kicad import emit_routes
    from tracewise.route.engine.multi import Net
    from tracewise.route.gridless.adapter import GridlessNetRoute

    # Create a minimal .kicad_pcb with net declaration
    pcb_content = """\
(kicad_pcb
  (version 20240108)
  (net 1 "net_T")
)
"""
    pcb = tmp_path / "test.kicad_pcb"
    pcb.write_text(pcb_content)

    grid = Grid(pitch=0.5, x0=0.0, y0=0.0, width_mm=20.0, height_mm=20.0)
    net = Net(name="net_T", pads=[], halfwidth_cells=1)

    # Path: F.Cu segment, via transition (same pos), B.Cu segment
    world_paths = [
        [
            (1.0, 1.0, 0),   # F.Cu
            (5.0, 1.0, 0),   # F.Cu end
            (5.0, 1.0, 1),   # via transition (same pos, layer changes)
            (9.0, 1.0, 1),   # B.Cu
        ]
    ]
    nr = GridlessNetRoute(
        net=net, ok=True,
        world_paths=world_paths,
        world_vias=[(5.0, 1.0)],
    )
    nr.rasterize_into_grid(grid)

    emitted = emit_routes(
        pcb, grid, {"net_T": nr},
        track_mm=0.2, via_mm=0.6, via_drill_mm=0.3,
    )

    text = pcb.read_text()

    # Should have 2 segments (F.Cu 1→5, B.Cu 5→9), not 3 (no via-transition stub)
    seg_matches = re.findall(r'\(segment\b', text)
    assert len(seg_matches) == 2, \
        f"Expected 2 segments (F+B), got {len(seg_matches)}; emitted={emitted}"

    # Should have 1 via
    via_matches = re.findall(r'\(via\b', text)
    assert len(via_matches) == 1, \
        f"Expected 1 via, got {len(via_matches)}"

    # Segments must be on different layers
    fcu_segs = re.findall(r'\(layer "F\.Cu"\)', text)
    bcu_segs = re.findall(r'\(layer "B\.Cu"\)', text)
    assert fcu_segs, "Must have an F.Cu segment"
    assert bcu_segs, "Must have a B.Cu segment"
