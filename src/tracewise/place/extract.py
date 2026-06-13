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
    # courtyard is the legality contract; bbox (incl. silk) only as fallback
    crt = fp.GetCourtyard(pcbnew.F_CrtYd)
    if crt.OutlineCount() == 0:
        crt = fp.GetCourtyard(pcbnew.B_CrtYd)
    if crt.OutlineCount() > 0:
        bb = crt.BBox()
    else:
        bb = fp.GetBoundingBox(False)
    w, h = bb.GetWidth() / IU, bb.GetHeight() / IU
    # footprint origin is NOT the box center (headers anchor at pin 1) —
    # carry the center offset or every box is placed wrong
    cx = bb.GetCenter().x / IU - fp.GetPosition().x / IU
    cy = bb.GetCenter().y / IU - fp.GetPosition().y / IU
    pads = []
    for p in fp.Pads():
        off = p.GetPosition() - fp.GetPosition()
        pads.append({{"net": p.GetNetname(), "dx": off.x / IU, "dy": off.y / IU}})
    fps.append({{
        "ref": fp.GetReference(),
        "x": fp.GetPosition().x / IU, "y": fp.GetPosition().y / IU,
        "w": w, "h": h, "cx": cx, "cy": cy,
        "locked": bool(fp.IsLocked()),
        "side": 1 if fp.IsFlipped() else 0,  # 0=front (F.Cu), 1=back (B.Cu)
        "rot": fp.GetOrientation().AsDegrees(),
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
        m = moves[r]
        pos = pcbnew.VECTOR2I(int(m[0] * IU), int(m[1] * IU))
        fp.SetPosition(pos)
        if len(m) > 2 and m[2]:
            fp.SetOrientation(fp.GetOrientation()
                              + pcbnew.EDA_ANGLE(float(m[2]), pcbnew.DEGREES_T))
        if len(m) > 3 and m[3]:  # flip to the other copper side, in place
            fp.Flip(pos, False)
pcbnew.SaveBoard({board!r}, b)
print("moved", sum(1 for fp in b.GetFootprints() if fp.GetReference() in moves))
"""


def apply_positions(board: str | Path, moves: dict[str, tuple[float, float]]) -> None:
    _run_pcbnew_script(
        APPLY_SCRIPT.format(board=str(Path(board).resolve()), moves=json.dumps(moves))
    )
