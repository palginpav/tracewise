"""R2: the engine meets real KiCad boards.

Extraction (pcbnew JSON, same pattern as place/extract.py): every pad with its
net, absolute position, layer reach (SMD front/back vs through-hole) and a
conservative disc radius. Grid build: other nets' pads become clearance-
inflated obstacles; the routed net's own pads are A* goals. Emission: cell
paths convert back to mm and land in the .kicad_pcb as (segment)/(via) nodes
written by the lossless sexpr editor — no SWIG on the write path. Net numbers
come from the board's own (net N "name") declarations, read with the same
parser.
"""

from __future__ import annotations

import json
import math
from pathlib import Path

from tracewise.route.bridge import _run_pcbnew_script
from tracewise.route.engine.astar import simplify
from tracewise.route.engine.grid import Grid
from tracewise.route.engine.multi import Net, NetRoute, route_all
from tracewise.sexpr import atom, node, parse_file, write_file

PAD_SCRIPT = """
import pcbnew, json
b = pcbnew.LoadBoard({board!r})
IU = 1e6
pads = []
for fp in b.GetFootprints():
    for p in fp.Pads():
        ls = p.GetLayerSet()
        front = ls.Contains(pcbnew.F_Cu)
        back = ls.Contains(pcbnew.B_Cu)
        bb = p.GetBoundingBox()
        pads.append({{
            "ref": fp.GetReference(),
            "net": p.GetNetname(),
            "x": p.GetPosition().x / IU, "y": p.GetPosition().y / IU,
            "front": bool(front), "back": bool(back),
            "hw": bb.GetWidth() / 2 / IU, "hh": bb.GetHeight() / 2 / IU,
        }})
edges = b.GetBoardEdgesBoundingBox()
print("TWJSON" + json.dumps({{
    "pads": pads,
    "board": {{"x1": edges.GetLeft()/IU, "y1": edges.GetTop()/IU,
               "x2": edges.GetRight()/IU, "y2": edges.GetBottom()/IU}},
}}))
"""


def extract_pads(board: str | Path) -> dict:
    out = _run_pcbnew_script(PAD_SCRIPT.format(board=str(Path(board).resolve())))
    for line in out.splitlines():
        if line.startswith("TWJSON"):
            data = json.loads(line[len("TWJSON"):])
            # keepouts come from a direct sexpr parse (lossless, no pcbnew round-trip)
            data["keepouts"] = extract_keepouts(board)
            return data
    raise RuntimeError("pad extraction produced no data")


def extract_keepouts(board: str | Path) -> list[dict]:
    """Keepout / rule-area zones that forbid tracks, parsed from the board
    s-expression. Each is {layers: set[int], pts: [(x,y)..]}. The router routes
    through these otherwise (measured: 35 'items_not_allowed' GND tracks in
    zuluscsi's keepout)."""
    from tracewise.sexpr import parse_file

    try:
        root = parse_file(board)
    except (OSError, ValueError):
        return []
    out = []
    for z in root.find_all("zone"):
        ko = z.first("keepout")
        if ko is None:
            continue
        tr = ko.first("tracks")
        if not (tr and tr.arg() == "not_allowed"):
            continue  # only zones that forbid tracks affect routing
        names = set()
        for ly in z.find_all("layer"):
            names.add(ly.arg() or "")
        lyrs = z.first("layers")
        if lyrs:
            names.update(a.value for a in lyrs.atoms()[1:])
        layers = set()
        for nm in names:
            if "F" in nm and ".Cu" in nm:
                layers.add(0)
            if "B" in nm and ".Cu" in nm:
                layers.add(1)
        if not layers:
            layers = {0, 1}  # F&B.Cu or unspecified -> both
        poly = z.first("polygon")
        pts = []
        if poly:
            for xy in poly.find_all("xy"):
                try:
                    pts.append((float(xy.arg(1)), float(xy.arg(2))))
                except (TypeError, ValueError):
                    pass
        if len(pts) >= 3:
            out.append({"layers": layers, "pts": pts})
    return out


def build_problem(
    data: dict, pitch: float = 0.1, track_mm: float = 0.2, clearance_mm: float = 0.2
) -> tuple[Grid, list[Net], dict]:
    bd = data["board"]
    grid = Grid(x0=bd["x1"], y0=bd["y1"], width_mm=bd["x2"] - bd["x1"],
                height_mm=bd["y2"] - bd["y1"], pitch=pitch, layers=2)
    for ko in data.get("keepouts", []):  # forbid routing through keepout areas
        for layer in ko["layers"]:
            grid.block_polygon(layer, ko["pts"])
    inflate = track_mm / 2 + clearance_mm
    by_net: dict[str, list[tuple[int, int, int]]] = {}
    carve: dict[str, list[tuple]] = {}
    anchors: dict[tuple[int, int, int], tuple[float, float]] = {}
    for p in data["pads"]:
        layers = ([0] if p["front"] else []) + ([1] if p["back"] else [])
        rect = (p["x"] - p["hw"], p["y"] - p["hh"], p["x"] + p["hw"], p["y"] + p["hh"])
        for layer in layers:
            if p["net"]:
                cell = grid.clamp_cell(*grid.to_cell(p["x"], p["y"]))
                by_net.setdefault(p["net"], []).append((layer, *cell))
                anchors[(layer, *cell)] = (p["x"], p["y"])
                carve.setdefault(p["net"], []).append((layer, *rect, inflate))
            grid.block_pad(layer, *rect, inflate_mm=inflate)
    half_cells = max(1, math.ceil((track_mm / 2 + clearance_mm) / pitch))
    nets = [Net(name, pads, halfwidth_cells=half_cells, carve=carve.get(name, []))
            for name, pads in by_net.items() if len(pads) >= 2]
    return grid, nets, anchors


def emit_routes(
    board: str | Path, grid: Grid, results: dict[str, NetRoute],
    track_mm: float = 0.2, via_mm: float = 0.6, via_drill_mm: float = 0.3,
    anchors: dict | None = None, neck_mm: float | None = None,
) -> dict:
    """Write routed nets into the board file via the sexpr editor."""
    root = parse_file(board)
    # KiCad 10 references nets by NAME in copper items; older boards use
    # numeric (net N) with top-level declarations. Support both.
    decls = {n.arg(2): n.arg(1) for n in root.nodes("net") if n.arg(2) is not None}
    def net_atom(name: str):
        if decls:
            return decls.get(name) and node("net", decls[name])
        return node("net", atom(name, quote=True))
    layer_name = {0: "F.Cu", 1: "B.Cu"}
    segs = vias = 0
    for name, nr in results.items():
        if not nr.ok:
            continue
        if net_atom(name) is None:
            continue
        net_vias: set[tuple[float, float]] = set()
        for path in nr.paths:
            # terminal nodes snap to exact pad coordinates: cell centers can
            # miss thin pad copper by up to pitch/2, which DRC reads as a
            # dangling track end (the R4-falsified guarantee, repaired here)
            snap = {}
            if anchors:
                for term in (path[0], path[-1]):
                    if term in anchors:
                        snap[term] = anchors[term]
            runs = simplify(path)
            # segments only for multi-node runs, but vias for EVERY layer
            # transition of the ORIGINAL run list: filtering single-node
            # terminal runs before via emission amputated the via-in-pad that
            # connected back-layer approaches to front-only pads (the R4
            # dangling class, root-caused by numeric geometry walk)
            for run in runs:
                if len(run) < 2:
                    continue
                layer = layer_name[run[0][0]]
                for a, b in zip(run, run[1:], strict=False):
                    xa, ya = snap.get(a) or grid.to_world(a[1], a[2])
                    xb, yb = snap.get(b) or grid.to_world(b[1], b[2])
                    # width-necking: segments that traversed clearance halos
                    # emit at the necked width (never below project minimum) —
                    # what a human does at fine-pitch escapes
                    w = track_mm
                    shaved = a in nr.escape_cells or b in nr.escape_cells
                    if neck_mm and neck_mm < track_mm and shaved:
                        w = neck_mm
                    root.insert(node("segment",
                        node("start", f"{xa:.3f}", f"{ya:.3f}"),
                        node("end", f"{xb:.3f}", f"{yb:.3f}"),
                        node("width", str(w)),
                        node("layer", atom(layer, quote=True)),
                        net_atom(name)))
                    segs += 1
            for i in range(len(runs) - 1):
                tn = runs[i][-1]
                other = runs[i + 1][0]
                # a through-hole pad spans the layers itself — transitions
                # INTO it need no via (via-in-hole = co-located drills)
                same_cell = tn[1:] == other[1:]
                if anchors and same_cell and tn in anchors and other in anchors:
                    continue
                pos = snap.get(tn) or snap.get(other)
                vx, vy = pos if pos else grid.to_world(tn[1], tn[2])
                if (vx, vy) in net_vias:
                    continue  # one barrel per position per net
                net_vias.add((vx, vy))
                root.insert(node("via",
                    node("at", f"{vx:.3f}", f"{vy:.3f}"),
                    node("size", str(via_mm)),
                    node("drill", str(via_drill_mm)),
                    node("layers", atom("F.Cu", quote=True), atom("B.Cu", quote=True)),
                    net_atom(name)))
                vias += 1
    write_file(root, board)
    return {"segments": segs, "vias": vias}


def project_geometry(board: str | Path) -> dict:
    """Track/clearance/via geometry from the project's own rules (fallbacks
    are conservative). Routing with fatter-than-project geometry costs real
    completion on dense boards — measured, not theoretical."""
    import json as _json

    geo = {"track_mm": 0.2, "clearance_mm": 0.2, "via_mm": 0.6, "via_drill_mm": 0.3,
           "min_track_mm": 0.1}
    pro = Path(board).with_suffix(".kicad_pro")
    if pro.exists():
        try:
            data = _json.loads(pro.read_text(encoding="utf-8"))
            rules = data.get("board", {}).get("design_settings", {}).get("rules", {})
            if rules.get("min_track_width"):
                geo["track_mm"] = max(float(rules["min_track_width"]), 0.1)
                geo["min_track_mm"] = geo["track_mm"]
            if rules.get("min_clearance"):
                geo["clearance_mm"] = max(float(rules["min_clearance"]), 0.1)
            for c in data.get("net_settings", {}).get("classes", []):
                if c.get("name") == "Default":
                    if c.get("via_diameter"):
                        geo["via_mm"] = float(c["via_diameter"])
                    if c.get("via_drill"):
                        geo["via_drill_mm"] = float(c["via_drill"])
        except (ValueError, OSError):
            pass
    return geo


def refill_zones(board: str | Path) -> None:
    """Re-pour all zones around the newly emitted copper. Pours are not
    routing obstacles — they refill with their own clearance after routing
    (skipping this leaves stale fills crossing the new tracks, which DRC
    reads as hundreds of clearance violations)."""
    board = Path(board).resolve()
    _run_pcbnew_script(
        "import pcbnew; "
        f"b = pcbnew.LoadBoard({str(board)!r}); "
        "pcbnew.ZONE_FILLER(b).Fill(b.Zones()); "
        f"pcbnew.SaveBoard({str(board)!r}, b)"
    )


def route_board_engine(board: str | Path, pitch: float = 0.1,
                       priority: dict[str, int] | None = None,
                       ripup_factor: int = 8, via_cost: float = 10.0) -> dict:
    """End-to-end: extract -> grid -> route_all -> emit. Returns a summary.

    `via_cost` is the A* penalty for a layer hop; lower it to make the router
    use the back layer as a bypass lane (F->B->F) past congestion."""
    data = extract_pads(board)
    geo = project_geometry(board)
    grid, nets, anchors = build_problem(data, pitch=pitch,
                                        track_mm=geo["track_mm"],
                                        clearance_mm=geo["clearance_mm"])
    # via blocking must hold the NEXT track's centerline at
    # via_r + clearance + track_halfwidth — omitting the halfwidth produced a
    # repeating 0.125mm-actual violation class (measured)
    via_half = max(1, math.ceil(
        (geo["via_mm"] / 2 + geo["clearance_mm"] + geo["track_mm"] / 2) / pitch))
    for n in nets:
        n.via_halfwidth_cells = via_half
    results = route_all(grid, nets, escape=12, priority=priority,
                        ripup_factor=ripup_factor, via_cost=via_cost)
    emitted = emit_routes(board, grid, results, track_mm=geo["track_mm"],
                          via_mm=geo["via_mm"], via_drill_mm=geo["via_drill_mm"],
                          anchors=anchors, neck_mm=geo["min_track_mm"])
    refill_zones(board)
    ok = sum(1 for r in results.values() if r.ok)
    return {
        "nets": len(nets), "routed": ok, "failed": len(nets) - ok,
        "failures": {n: r.reason for n, r in results.items() if not r.ok},
        **emitted,
    }
