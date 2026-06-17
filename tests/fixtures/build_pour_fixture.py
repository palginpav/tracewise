"""Build the pour-extraction test fixture boards.

Generates two files in this directory:
  pour_source.kicad_pcb  — hand-authored, zone-filled once
  pour_resaved.kicad_pcb — pcbnew round-trip of source (simulates format variance)

Fixture topology (F.Cu only, 2-layer, 30×30 mm board):
  GND zone: polygon (1,1)→(20,1)→(20,20)→(1,20)
  R1 pad at  (5, 5)  — inside zone → connected via fill
  R2 pad at (10,10)  — inside zone → connected via fill
  R3 pad at (25,25)  — outside zone / no copper path → ISOLATED

Run once to regenerate:
  python3 tests/fixtures/build_pour_fixture.py
"""

import os
import sys

import wx  # noqa: F401 — wx.DisableAsserts() must come before pcbnew
wx.DisableAsserts()

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


def _add_tht_pad(board, ref, x_mm, y_mm, net):
    fp = pcbnew.FOOTPRINT(board)
    fp.SetReference(ref)
    fp.SetPosition(pcbnew.VECTOR2I(int(x_mm * IU), int(y_mm * IU)))
    pad = pcbnew.PAD(fp)
    pad.SetShape(pcbnew.PAD_SHAPE_CIRCLE)
    pad.SetSize(pcbnew.VECTOR2I(int(1.0 * IU), int(1.0 * IU)))
    pad.SetDrillSize(pcbnew.VECTOR2I(int(0.5 * IU), int(0.5 * IU)))
    pad.SetLayerSet(pcbnew.LSET.AllCuMask())
    pad.SetNet(net)
    pad.SetNumber("1")
    fp.Add(pad)
    board.Add(fp)


def build() -> None:
    here = os.path.dirname(os.path.abspath(__file__))
    source_path = os.path.join(here, "pour_source.kicad_pcb")
    resaved_path = os.path.join(here, "pour_resaved.kicad_pcb")

    # -----------------------------------------------------------------
    # 1. Build the board
    # -----------------------------------------------------------------
    b = pcbnew.CreateEmptyBoard()
    b.SetCopperLayerCount(2)

    # Board outline 30×30 mm
    for x1, y1, x2, y2 in [(0, 0, 30, 0), (30, 0, 30, 30), (30, 30, 0, 30), (0, 30, 0, 0)]:
        _add_edge(b, x1, y1, x2, y2)

    # GND net
    net_gnd = pcbnew.NETINFO_ITEM(b, "GND", 1)
    b.Add(net_gnd)
    gnd = b.FindNet("GND")

    # 3 GND pads: R1 and R2 inside zone, R3 outside (isolated)
    _add_tht_pad(b, "R1", 5.0, 5.0, gnd)
    _add_tht_pad(b, "R2", 10.0, 10.0, gnd)
    _add_tht_pad(b, "R3", 25.0, 25.0, gnd)

    # GND zone covering (1,1)..(20,20) on F.Cu
    zone = pcbnew.ZONE(b)
    zone.SetLayer(pcbnew.F_Cu)
    zone.SetNet(gnd)
    outline = zone.Outline()
    outline.NewOutline()
    for px, py in [(1, 1), (20, 1), (20, 20), (1, 20)]:
        outline.Append(int(px * IU), int(py * IU))
    b.Add(zone)

    # Save un-filled first, then reload + fill (ZONE_FILLER needs a loaded board)
    pcbnew.SaveBoard(source_path, b)

    b2 = pcbnew.LoadBoard(source_path)
    filler = pcbnew.ZONE_FILLER(b2)
    filler.Fill(b2.Zones())
    pcbnew.SaveBoard(source_path, b2)

    # -----------------------------------------------------------------
    # 2. Resave = pcbnew round-trip (simulates format variance)
    # -----------------------------------------------------------------
    b3 = pcbnew.LoadBoard(source_path)
    pcbnew.SaveBoard(resaved_path, b3)

    print(f"Written: {source_path}")
    print(f"Written: {resaved_path}")


if __name__ == "__main__":
    build()
    sys.exit(0)
