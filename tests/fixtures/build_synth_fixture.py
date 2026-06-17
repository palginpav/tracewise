"""Build the power-pour synthesis test fixture board (F3).

Generates:
  synth_source.kicad_pcb — a 2-layer 40×40 mm board with:
    - GND zone (existing pour) covering the TOP half: (1,1)→(39,18)
    - +5V net: 10 pads in the BOTTOM half (y > 22, outside GND zone)
      so the synthesized +5V fill can claim the bottom-half copper.
      No zone → qualifies (min_pads=8 default).
    - +3V3 net: 3 pads (no zone) → below min_pads, should be skipped
    - SIG net: 10 pads (no zone) → not a power net, should be skipped

Topology rationale
------------------
The GND zone occupies only the top half.  The synthesized +5V zone
(cloned from the GND outline) will cover the whole board, but GND
(higher priority) keeps the top half.  The bottom half — where all
+5V pads live — has no GND copper, so the +5V fill fills it and
connects those pads.

Run once to regenerate (needs pcbnew):
  python3 tests/fixtures/build_synth_fixture.py
"""

import os
import sys

try:
    import wx  # noqa: F401 — wx.DisableAsserts() must come before pcbnew
    wx.DisableAsserts()
except ImportError:
    pass  # pcbnew may embed wx without a top-level wx package

import pcbnew  # noqa: E402

IU = 1_000_000  # nm per mm


def _add_edge(board, x1, y1, x2, y2):
    s = pcbnew.PCB_SHAPE(board)
    s.SetShape(pcbnew.SHAPE_T_SEGMENT)
    s.SetStart(pcbnew.VECTOR2I(int(x1 * IU), int(y1 * IU)))
    s.SetEnd(pcbnew.VECTOR2I(int(x2 * IU), int(y2 * IU)))
    s.SetLayer(pcbnew.Edge_Cuts)
    s.SetWidth(int(0.1 * IU))
    board.Add(s)


def _add_tht_pad(board, ref, pad_num, x_mm, y_mm, net):
    fp = pcbnew.FOOTPRINT(board)
    fp.SetReference(ref)
    fp.SetPosition(pcbnew.VECTOR2I(int(x_mm * IU), int(y_mm * IU)))
    pad = pcbnew.PAD(fp)
    pad.SetShape(pcbnew.PAD_SHAPE_CIRCLE)
    pad.SetSize(pcbnew.VECTOR2I(int(1.0 * IU), int(1.0 * IU)))
    pad.SetDrillSize(pcbnew.VECTOR2I(int(0.5 * IU), int(0.5 * IU)))
    pad.SetLayerSet(pcbnew.LSET.AllCuMask())
    pad.SetNet(net)
    pad.SetNumber(str(pad_num))
    fp.Add(pad)
    board.Add(fp)


def build() -> None:
    here = os.path.dirname(os.path.abspath(__file__))
    path = os.path.join(here, "synth_source.kicad_pcb")

    b = pcbnew.CreateEmptyBoard()
    b.SetCopperLayerCount(2)

    # Board outline 40×40 mm
    for x1, y1, x2, y2 in [(0, 0, 40, 0), (40, 0, 40, 40), (40, 40, 0, 40), (0, 40, 0, 0)]:
        _add_edge(b, x1, y1, x2, y2)

    # ----------- GND net + zone (TOP half only: y 1→18) ---------------------
    net_gnd = pcbnew.NETINFO_ITEM(b, "GND", 1)
    b.Add(net_gnd)
    gnd = b.FindNet("GND")
    _add_tht_pad(b, "GND1", 1, 5.0, 5.0, gnd)
    _add_tht_pad(b, "GND2", 1, 35.0, 12.0, gnd)

    zone_gnd = pcbnew.ZONE(b)
    zone_gnd.SetLayer(pcbnew.F_Cu)
    zone_gnd.SetNet(gnd)
    zone_gnd.SetAssignedPriority(2)
    outline = zone_gnd.Outline()
    outline.NewOutline()
    # GND zone covers only the top half (y: 1 → 18)
    for px, py in [(1, 1), (39, 1), (39, 18), (1, 18)]:
        outline.Append(int(px * IU), int(py * IU))
    b.Add(zone_gnd)

    # ----------- +5V net: 10 pads in BOTTOM half (y > 22), no zone ----------
    # These pads are OUTSIDE the GND zone, so the synthesized +5V zone fill
    # (covering the full board) will connect them via the bottom-half copper.
    net_5v = pcbnew.NETINFO_ITEM(b, "+5V", 2)
    b.Add(net_5v)
    pv = b.FindNet("+5V")
    pad_positions_5v = [
        (5, 23), (10, 23), (15, 23), (20, 23), (25, 23),
        (5, 30), (10, 30), (15, 30), (20, 30), (25, 30),
    ]
    for i, (px, py) in enumerate(pad_positions_5v):
        _add_tht_pad(b, f"U{i+1}", 1, px, py, pv)

    # ----------- +3V3 net: 3 pads (no zone) — below min_pads ----------------
    net_3v3 = pcbnew.NETINFO_ITEM(b, "+3V3", 3)
    b.Add(net_3v3)
    pv3 = b.FindNet("+3V3")
    for i, (px, py) in enumerate([(32, 23), (32, 30), (32, 37)]):
        _add_tht_pad(b, f"R{i+1}", 1, px, py, pv3)

    # ----------- SIG net: 10 pads (no zone) — not a power net ---------------
    net_sig = pcbnew.NETINFO_ITEM(b, "SIG", 4)
    b.Add(net_sig)
    sig = b.FindNet("SIG")
    for i, (px, py) in enumerate([
        (5, 37), (8, 37), (11, 37), (14, 37), (17, 37),
        (5, 34), (8, 34), (11, 34), (14, 34), (17, 34),
    ]):
        _add_tht_pad(b, f"C{i+1}", 1, px, py, sig)

    # Save, reload, fill zones
    pcbnew.SaveBoard(path, b)
    b2 = pcbnew.LoadBoard(path)
    pcbnew.ZONE_FILLER(b2).Fill(b2.Zones())
    pcbnew.SaveBoard(path, b2)

    print(f"Written: {path}")


if __name__ == "__main__":
    build()
    sys.exit(0)
