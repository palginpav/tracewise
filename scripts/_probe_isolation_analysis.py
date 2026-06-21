"""Quick isolation analysis for nodes=2, edges=0 nets.

Diagnose WHY /QSPI_SD2 and /QSPI_SCLK show nodes=2, edges=0 in both
A (hard-pour) and B (receding-pour) models. This means the two pad
centers are in completely DISCONNECTED free-space components.

Run after the board is already routed (board at /tmp/probe_pour_artifact/).
"""
from __future__ import annotations
import sys, json
from pathlib import Path
ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "src"))

import shapely
from shapely.geometry import Point, box as _box
from shapely.ops import unary_union
from shapely import set_precision

from tracewise.route.bridge import _run_pcbnew_script
from tracewise.route.engine.kicad import extract_pads, project_geometry
from tracewise.route.gridless.geom import (
    build_windowed_free_space,
    extract_board_outline,
    extract_drill_obstacles,
    extract_drill_centers,
    net_routes_to_track_obstacles,
    snap,
)
from tracewise.route.engine.multi import route_all
from tracewise.route.engine.kicad import build_problem
import math

BOARD_PATH = Path("/tmp/probe_pour_artifact/Mitayi-Pico-D1.kicad_pcb")
PROBES = ["/QSPI_SD2", "/QSPI_SCLK"]

def get_track_obstacles_from_board(board):
    """Extract track copper from the already-routed board."""
    script = f"""
import wx; wx.DisableAsserts()
import pcbnew, json
b = pcbnew.LoadBoard({str(board)!r})
IU = 1e6
tracks = []
for t in b.GetTracks():
    cls = t.GetClass()
    if cls in ('PCB_TRACK', 'PCB_ARC'):
        start = t.GetStart()
        end = t.GetEnd()
        tracks.append({{
            'net': t.GetNetname(),
            'x1': start.x/IU, 'y1': start.y/IU,
            'x2': end.x/IU, 'y2': end.y/IU,
            'width': t.GetWidth()/IU,
            'layer': t.GetLayerName(),
        }})
print("TWJSON" + json.dumps(tracks))
raise SystemExit(0)
"""
    out = _run_pcbnew_script(script)
    for line in out.splitlines():
        if line.startswith("TWJSON"):
            return json.loads(line[len("TWJSON"):])
    return []

def build_track_obstacles_shapely(tracks, geo):
    """Build Shapely obstacles from track list."""
    from shapely.geometry import LineString
    inflate = geo["track_mm"] / 2.0 + geo["clearance_mm"]
    obstacles = []
    for t in tracks:
        try:
            ls = snap(LineString([(t["x1"], t["y1"]), (t["x2"], t["y2"])]).buffer(
                inflate, cap_style=2))
            obstacles.append(ls)
        except Exception:
            pass
    return obstacles

def main():
    print(f"[iso] Board: {BOARD_PATH}", flush=True)
    board = BOARD_PATH

    data = extract_pads(board)
    geo = project_geometry(board)
    print(f"[iso] geo: track={geo['track_mm']} clearance={geo['clearance_mm']}", flush=True)

    bd = data["board"]
    board_bbox = (bd["x1"], bd["y1"], bd["x2"], bd["y2"])
    board_outline = extract_board_outline(board)
    drill_obstacles = extract_drill_obstacles(board, geo["clearance_mm"], geo["track_mm"])
    drill_centers = extract_drill_centers(board)

    # Get tracks from the already-routed board
    tracks = get_track_obstacles_from_board(board)
    print(f"[iso] Board tracks: {len(tracks)}", flush=True)
    track_obstacles = build_track_obstacles_shapely(tracks, geo)
    print(f"[iso] Track obstacles: {len(track_obstacles)}", flush=True)

    # Also build via obstacles
    via_obstacles = []
    script_vias = f"""
import wx; wx.DisableAsserts()
import pcbnew, json
b = pcbnew.LoadBoard({str(board)!r})
IU = 1e6
vias = []
for t in b.GetTracks():
    if t.GetClass() == 'PCB_VIA':
        pos = t.GetPosition()
        vias.append({{'x': pos.x/IU, 'y': pos.y/IU, 'size': t.GetWidth()/IU, 'drill': t.GetDrillValue()/IU}})
print("TWJSON" + json.dumps(vias))
raise SystemExit(0)
"""
    out = _run_pcbnew_script(script_vias)
    for line in out.splitlines():
        if line.startswith("TWJSON"):
            vias_list = json.loads(line[len("TWJSON"):])
            for v in vias_list:
                inflate = geo["via_mm"] / 2.0 + geo["clearance_mm"]
                try:
                    circ = snap(Point(v["x"], v["y"]).buffer(inflate, resolution=16))
                    via_obstacles.append(circ)
                except Exception:
                    pass
    print(f"[iso] Via obstacles: {len(via_obstacles)}", flush=True)

    all_obstacles = track_obstacles + via_obstacles

    for net_name in PROBES:
        pads = [p for p in data["pads"] if p.get("net") == net_name]
        print(f"\n[iso] ===== {net_name} ({len(pads)} pads) =====", flush=True)
        for p in pads:
            print(f"  Pad at ({p['x']:.3f}, {p['y']:.3f}) front={p.get('front')} back={p.get('back')}", flush=True)

        xs = [p["x"] for p in pads]
        ys = [p["y"] for p in pads]
        bx1, by1, bx2, by2 = board_bbox
        WINDOW = 10.0
        wx1 = max(min(xs) - WINDOW, bx1)
        wy1 = max(min(ys) - WINDOW, by1)
        wx2 = min(max(xs) + WINDOW, bx2)
        wy2 = min(max(ys) + WINDOW, by2)
        window_bbox = (wx1, wy1, wx2, wy2)
        print(f"  Window: {window_bbox}", flush=True)

        # Build free space (model B = no pours)
        fs, obs_polys = build_windowed_free_space(
            data["pads"], net_name, geo["clearance_mm"], geo["track_mm"],
            all_obstacles, window_bbox,
            board_outline=board_outline,
            drill_obstacles=drill_obstacles,
            layer=0,
        )

        print(f"  Free space type: {fs.geom_type}", flush=True)
        if fs.geom_type == "MultiPolygon":
            components = list(fs.geoms)
            print(f"  Free space components: {len(components)}", flush=True)
        else:
            components = [fs]
            print(f"  Free space: single polygon", flush=True)

        print(f"  Total free space area: {fs.area:.2f} mm2", flush=True)

        # Check which component each pad is in
        for i, p in enumerate(pads):
            pt = Point(p["x"], p["y"])
            found = False
            for j, comp in enumerate(components):
                if comp.contains(pt) or comp.distance(pt) < 0.01:
                    print(f"  Pad {i} ({p['x']:.3f},{p['y']:.3f}) in component {j} (area={comp.area:.3f})", flush=True)
                    found = True
                    break
            if not found:
                min_dist = min(comp.distance(pt) for comp in components)
                print(f"  Pad {i} ({p['x']:.3f},{p['y']:.3f}) NOT IN ANY COMPONENT! min_dist={min_dist:.4f}", flush=True)

        # Check if components containing pads are the SAME
        pad_components = []
        for p in pads:
            pt = Point(p["x"], p["y"])
            for j, comp in enumerate(components):
                if comp.contains(pt) or comp.distance(pt) < 0.01:
                    pad_components.append(j)
                    break
            else:
                pad_components.append(-1)

        all_same = len(set(pad_components)) == 1
        print(f"  Pad components: {pad_components} — {'SAME ISLAND (routing possible)' if all_same else 'DIFFERENT ISLANDS (topology impossible!)'}", flush=True)

        # Count obstacle types in window
        n_pads_obs = len(obs_polys)

        # Check what specifically blocks the path between pads
        if len(pads) == 2:
            from shapely.geometry import LineString
            corridor = snap(LineString([(pads[0]["x"], pads[0]["y"]),
                                         (pads[1]["x"], pads[1]["y"])]).buffer(
                geo["track_mm"] + geo["clearance_mm"], cap_style=2))
            print(f"  Direct corridor area: {corridor.area:.4f} mm2", flush=True)

            # Count how many track obstacles intersect the direct corridor
            blocking_tracks = sum(1 for obs in all_obstacles if obs.intersects(corridor))
            print(f"  Track obstacles in direct corridor: {blocking_tracks}", flush=True)

            # Free space in corridor
            try:
                fs_in_corridor = snap(fs.intersection(corridor))
                print(f"  Free space in corridor: {fs_in_corridor.area:.4f} mm2 ({fs_in_corridor.geom_type})", flush=True)
            except Exception as e:
                print(f"  Free space in corridor: ERROR {e}", flush=True)

    print("\n[iso] CONCLUSION:", flush=True)
    print("  If pad components differ → pads are in separate free-space islands", flush=True)
    print("  → tracks (not pours) fragment the free space", flush=True)
    print("  → TRUE WALL: no routing possible even with receding-pour model", flush=True)


if __name__ == "__main__":
    main()
