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
            return json.loads(line[len("TWJSON"):])
    raise RuntimeError("pad extraction produced no data")


def build_problem(
    data: dict, pitch: float = 0.1, track_mm: float = 0.2, clearance_mm: float = 0.2
) -> tuple[Grid, list[Net]]:
    bd = data["board"]
    grid = Grid(x0=bd["x1"], y0=bd["y1"], width_mm=bd["x2"] - bd["x1"],
                height_mm=bd["y2"] - bd["y1"], pitch=pitch, layers=2)
    inflate = track_mm / 2 + clearance_mm
    by_net: dict[str, list[tuple[int, int, int]]] = {}
    carve: dict[str, list[tuple]] = {}
    for p in data["pads"]:
        layers = ([0] if p["front"] else []) + ([1] if p["back"] else [])
        rect = (p["x"] - p["hw"], p["y"] - p["hh"], p["x"] + p["hw"], p["y"] + p["hh"])
        for layer in layers:
            if p["net"]:
                cell = grid.clamp_cell(*grid.to_cell(p["x"], p["y"]))
                by_net.setdefault(p["net"], []).append((layer, *cell))
                carve.setdefault(p["net"], []).append((layer, *rect, inflate))
            grid.block_pad(layer, *rect, inflate_mm=inflate)
    half_cells = max(1, math.ceil((track_mm / 2 + clearance_mm) / pitch))
    nets = [Net(name, pads, halfwidth_cells=half_cells, carve=carve.get(name, []))
            for name, pads in by_net.items() if len(pads) >= 2]
    return grid, nets


def emit_routes(
    board: str | Path, grid: Grid, results: dict[str, NetRoute],
    track_mm: float = 0.2, via_mm: float = 0.6, via_drill_mm: float = 0.3,
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
        for path in nr.paths:
            runs = [r for r in simplify(path) if len(r) >= 2]  # no degenerate runs
            for i, run in enumerate(runs):
                layer = layer_name[run[0][0]]
                for a, b in zip(run, run[1:], strict=False):
                    xa, ya = grid.to_world(a[1], a[2])
                    xb, yb = grid.to_world(b[1], b[2])
                    root.insert(node("segment",
                        node("start", f"{xa:.3f}", f"{ya:.3f}"),
                        node("end", f"{xb:.3f}", f"{yb:.3f}"),
                        node("width", str(track_mm)),
                        node("layer", atom(layer, quote=True)),
                        net_atom(name)))
                    segs += 1
                if i + 1 < len(runs):  # via between layer runs
                    vx, vy = grid.to_world(run[-1][1], run[-1][2])
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

    geo = {"track_mm": 0.2, "clearance_mm": 0.2, "via_mm": 0.6, "via_drill_mm": 0.3}
    pro = Path(board).with_suffix(".kicad_pro")
    if pro.exists():
        try:
            data = _json.loads(pro.read_text(encoding="utf-8"))
            rules = data.get("board", {}).get("design_settings", {}).get("rules", {})
            if rules.get("min_track_width"):
                geo["track_mm"] = max(float(rules["min_track_width"]), 0.1)
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


def route_board_engine(board: str | Path, pitch: float = 0.1) -> dict:
    """End-to-end: extract -> grid -> route_all -> emit. Returns a summary."""
    data = extract_pads(board)
    geo = project_geometry(board)
    grid, nets = build_problem(data, pitch=pitch,
                               track_mm=geo["track_mm"], clearance_mm=geo["clearance_mm"])
    via_half = max(1, math.ceil((geo["via_mm"] / 2 + geo["clearance_mm"]) / pitch))
    for n in nets:
        n.via_halfwidth_cells = via_half
    results = route_all(grid, nets, escape=12)  # ~1.2mm endpoint escape
    emitted = emit_routes(board, grid, results, track_mm=geo["track_mm"],
                          via_mm=geo["via_mm"], via_drill_mm=geo["via_drill_mm"])
    refill_zones(board)
    ok = sum(1 for r in results.values() if r.ok)
    return {
        "nets": len(nets), "routed": ok, "failed": len(nets) - ok,
        "failures": {n: r.reason for n, r in results.items() if not r.ok},
        **emitted,
    }
