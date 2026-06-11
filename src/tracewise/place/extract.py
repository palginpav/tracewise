"""Extract placement geometry from a board via pcbnew (JSON over a script).

Per footprint: reference, position (mm), locked flag, courtyard-ish bounding
box (width/height, mm). Per pad: parent footprint, offset from the footprint
origin (mm), net name. Connectors and mechanically-locked parts should be
locked in KiCad (or via --lock-refs) — the optimizer never moves locked parts.
"""

from __future__ import annotations

import json
from pathlib import Path

from tracewise.route.bridge import _run_pcbnew_script

EXTRACT_SCRIPT = """
import pcbnew, json
b = pcbnew.LoadBoard({board!r})
IU = 1e6  # nm per mm
fps = []
for fp in b.GetFootprints():
    bbox = fp.GetBoundingBox(False)  # without text
    pads = []
    for p in fp.Pads():
        off = p.GetPosition() - fp.GetPosition()
        pads.append({{"net": p.GetNetname(), "dx": off.x / IU, "dy": off.y / IU}})
    fps.append({{
        "ref": fp.GetReference(),
        "x": fp.GetPosition().x / IU, "y": fp.GetPosition().y / IU,
        "w": bbox.GetWidth() / IU, "h": bbox.GetHeight() / IU,
        "locked": bool(fp.IsLocked()),
        "pads": pads,
    }})
edges = b.GetBoardEdgesBoundingBox()
out = {{
    "footprints": fps,
    "board": {{"x1": edges.GetLeft() / IU, "y1": edges.GetTop() / IU,
               "x2": edges.GetRight() / IU, "y2": edges.GetBottom() / IU}},
}}
print("TWJSON" + json.dumps(out))
"""


def extract(board: str | Path) -> dict:
    out = _run_pcbnew_script(EXTRACT_SCRIPT.format(board=str(Path(board).resolve())))
    for line in out.splitlines():
        if line.startswith("TWJSON"):
            return json.loads(line[len("TWJSON"):])
    raise RuntimeError("extraction produced no data")


APPLY_SCRIPT = """
import pcbnew, json
b = pcbnew.LoadBoard({board!r})
moves = json.loads({moves!r})
IU = 1e6
for fp in b.GetFootprints():
    r = fp.GetReference()
    if r in moves and not fp.IsLocked():
        x, y = moves[r]
        fp.SetPosition(pcbnew.VECTOR2I(int(x * IU), int(y * IU)))
pcbnew.SaveBoard({board!r}, b)
print("moved", sum(1 for fp in b.GetFootprints() if fp.GetReference() in moves))
"""


def apply_positions(board: str | Path, moves: dict[str, tuple[float, float]]) -> None:
    _run_pcbnew_script(
        APPLY_SCRIPT.format(board=str(Path(board).resolve()), moves=json.dumps(moves))
    )
