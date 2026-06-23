#!/usr/bin/env python3
"""
_invent1_human_stats.py — INVENTOR-1 throwaway analysis.

Reverse-engineer the human's DECISION PROCEDURE from the shipped 0/0 board.
Extends probe_human_routing.py's parser with the statistics the invention needs:

  1. LAYER-BY-DIRECTION: for every track segment, its angle and layer. Is there a
     "F.Cu = horizontal, B.Cu = vertical" (or similar) directional layer convention?
  2. VIA RING geometry around U3 (the RP2040 QFN): each via's radius from U3 center
     and angular sector. Is fanout a ring? what radius band? are sectors used once?
  3. J3/J4 CONNECTOR channel structure: which layer do nets entering J3/J4 use on the
     last leg, and the spatial ordering of entries (a bus?).
  4. PER-NET layer-transition topology: how many layer changes, where vias sit
     relative to the source QFN pad vs the destination connector pad.

Throwaway — name prefixed `_invent1_`. Read-only. No production code touched.
"""

import math
import sys
from collections import defaultdict
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from probe_human_routing import (  # reuse the proven parser
    extract_nets, extract_segments, extract_vias, parse_pcb,
    segment_length, find_all, get_attr,
)

PCB = Path(__file__).parent.parent / "data/benchmark-boards/mitayi-pico-d1/Mitayi-Pico-D1.kicad_pcb"


def extract_pads_global(root):
    """Like probe's extract_pads but returns GLOBAL pad coords.

    KiCad pad `at` is in the footprint LOCAL frame (pre-rotation). Global =
    footprint_origin + Rotate(local, footprint_angle). Vias/segments are already
    global, so to compare via-to-pad geometry we MUST put pads in the same frame.
    """
    pads = []
    for fp in find_all(root, "footprint"):
        ref = None
        for prop in find_all(fp, "property"):
            if len(prop) >= 3 and prop[1] == "Reference":
                ref = prop[2]
        at_node = get_attr(fp, "at")
        fx = fy = 0.0
        fang = 0.0
        if at_node and len(at_node) >= 3:
            fx, fy = float(at_node[1]), float(at_node[2])
            if len(at_node) >= 4:
                try:
                    fang = float(at_node[3])
                except Exception:
                    fang = 0.0
        ca = math.cos(math.radians(fang)); sa = math.sin(math.radians(fang))
        for pad in find_all(fp, "pad"):
            d = {"ref": ref}
            if len(pad) >= 3:
                d["pad_num"] = pad[1]
                d["pad_type"] = pad[2]
            for child in pad[1:]:
                if isinstance(child, list) and child:
                    key = child[0]
                    if key == "net":
                        try:
                            d["net_num"] = int(child[1])
                            d["net_name"] = child[2] if len(child) > 2 else ""
                        except Exception:
                            pass
                    elif key == "at":
                        try:
                            lx, ly = float(child[1]), float(child[2])
                            # KiCad applies footprint rotation to local pad coords
                            gx = fx + (lx * ca - ly * sa)
                            gy = fy + (lx * sa + ly * ca)
                            d["at"] = (gx, gy)
                        except Exception:
                            pass
                    elif key == "drill":
                        d["has_drill"] = True
            pads.append(d)
    return pads


def seg_angle_deg(s):
    """Angle of a segment in [0,180) — undirected (a track has no direction)."""
    dx = s["end"][0] - s["start"][0]
    dy = s["end"][1] - s["start"][1]
    a = math.degrees(math.atan2(dy, dx)) % 180.0
    return a


def angle_bucket(a):
    """Classify undirected angle into H / V / diag."""
    # KiCad Y grows downward; H = along x, V = along y.
    if a < 22.5 or a >= 157.5:
        return "H"          # horizontal
    if 67.5 <= a < 112.5:
        return "V"          # vertical
    return "D"              # diagonal (45-ish)


def main():
    root = parse_pcb(PCB)
    nets = extract_nets(root)
    segs = extract_segments(root)
    vias = extract_vias(root)
    pads = extract_pads_global(root)
    for v in vias:
        v["net_name"] = nets.get(v.get("net_num"), f"net_{v.get('net_num')}")

    # ── component centroids (U3 QFN, J3, J4, J1, J5) from their pads ──
    comp_pads = defaultdict(list)
    for p in pads:
        ref = p.get("ref")
        if ref and "at" in p:
            comp_pads[ref].append(p["at"])

    def centroid(ref):
        pts = comp_pads.get(ref, [])
        if not pts:
            return None
        return (sum(x for x, _ in pts) / len(pts), sum(y for _, y in pts) / len(pts))

    def bbox(ref):
        pts = comp_pads.get(ref, [])
        if not pts:
            return None
        xs = [x for x, _ in pts]; ys = [y for _, y in pts]
        return (min(xs), min(ys), max(xs), max(ys))

    u3 = centroid("U3")
    u3bb = bbox("U3")
    print("=" * 72)
    print("COMPONENT GEOMETRY")
    print("=" * 72)
    print(f"  U3 (RP2040 QFN) center = {u3}, n_pads={len(comp_pads.get('U3', []))}")
    if u3bb:
        ring_r = max(u3bb[2] - u3[0], u3bb[3] - u3[1], u3[0] - u3bb[0], u3[1] - u3bb[1])
        print(f"  U3 pad bbox = {tuple(round(v,2) for v in u3bb)}  -> outer pad-ring radius ~= {ring_r:.2f}mm")
    for ref in ("J3", "J4", "J1", "J5", "J2", "J9"):
        c = centroid(ref); bb = bbox(ref)
        if c:
            print(f"  {ref} center = ({c[0]:.2f},{c[1]:.2f}) bbox={tuple(round(v,2) for v in bb)} n_pads={len(comp_pads.get(ref,[]))}")

    # ── (1) LAYER-BY-DIRECTION (the layer convention) ──
    print("\n" + "=" * 72)
    print("(1) LAYER x DIRECTION  (track length-weighted, signal+power)")
    print("=" * 72)
    # length-weighted: a layer convention is about where COPPER goes, weight by length
    grid = defaultdict(float)      # (layer, bucket) -> total length
    cnt = defaultdict(int)         # (layer, bucket) -> seg count
    for s in segs:
        if "start" not in s or "end" not in s or "layer" not in s:
            continue
        L = segment_length(s)
        if L < 1e-6:
            continue
        b = angle_bucket(seg_angle_deg(s))
        grid[(s["layer"], b)] += L
        cnt[(s["layer"], b)] += 1
    layers = ["F.Cu", "B.Cu"]
    buckets = ["H", "V", "D"]
    print(f"  {'layer':<7} {'H_len':>9} {'V_len':>9} {'D_len':>9}  | {'H%':>5} {'V%':>5} {'D%':>5} (of layer)")
    for lyr in layers:
        tot = sum(grid[(lyr, b)] for b in buckets) or 1.0
        hv = [grid[(lyr, b)] for b in buckets]
        pct = [100 * grid[(lyr, b)] / tot for b in buckets]
        print(f"  {lyr:<7} {hv[0]:>9.1f} {hv[1]:>9.1f} {hv[2]:>9.1f}  | {pct[0]:>4.0f}% {pct[1]:>4.0f}% {pct[2]:>4.0f}%")
    # the inverse view: of all H copper, what fraction is on F.Cu? (the convention test)
    print("\n  Directional layer preference (of all copper in a direction, % on each layer):")
    for b in buckets:
        f = grid[("F.Cu", b)]; bb = grid[("B.Cu", b)]; t = (f + bb) or 1.0
        print(f"    {b}: F.Cu {100*f/t:>4.0f}%  B.Cu {100*bb/t:>4.0f}%   (F={f:.1f} B={bb:.1f} mm)")

    # SIGNAL-ONLY view (exclude GND/+3V3 power which behave differently)
    print("\n  SIGNAL-ONLY (exclude GND/+3V3 — pours/power skew the board view):")
    sgrid = defaultdict(float)
    for s in segs:
        nn = nets.get(s.get("net_num"), "")
        if nn in ("GND", "+3V3", "/3.3V"):
            continue
        if "start" not in s or "end" not in s or "layer" not in s:
            continue
        L = segment_length(s)
        if L < 1e-6:
            continue
        sgrid[(s["layer"], angle_bucket(seg_angle_deg(s)))] += L
    for b in buckets:
        f = sgrid[("F.Cu", b)]; bb = sgrid[("B.Cu", b)]; t = (f + bb) or 1.0
        print(f"    {b}: F.Cu {100*f/t:>4.0f}%  B.Cu {100*bb/t:>4.0f}%   (F={f:.1f} B={bb:.1f} mm)")

    # ── (2) VIA RING around U3 (the fanout structure) ──
    print("\n" + "=" * 72)
    print("(2) VIA RING around U3  (fanout-escape geometry)")
    print("=" * 72)
    via_polar = []
    for v in vias:
        if "at" not in v:
            continue
        dx = v["at"][0] - u3[0]; dy = v["at"][1] - u3[1]
        r = math.hypot(dx, dy)
        th = math.degrees(math.atan2(dy, dx)) % 360.0
        via_polar.append((r, th, v.get("net_name", "?")))
    via_polar.sort()
    near = [v for v in via_polar if v[0] <= 12.0]       # vias plausibly in U3 fanout band
    print(f"  total vias={len(via_polar)}; within 12mm of U3 center={len(near)}")
    # radius histogram (1mm bins)
    rbin = defaultdict(int)
    for r, _, _ in via_polar:
        rbin[int(r)] += 1
    print("  radius histogram (mm bin -> count):")
    for k in sorted(rbin):
        bar = "#" * rbin[k]
        print(f"    {k:>2}-{k+1:<2}mm: {rbin[k]:>3} {bar}")
    # the fanout band: how many vias sit in the 4-8mm ring the human used?
    band = [v for v in via_polar if 3.5 <= v[0] <= 8.5]
    print(f"\n  vias in 3.5-8.5mm 'fanout band': {len(band)}")
    if band:
        rs = [r for r, _, _ in band]
        print(f"    radius range {min(rs):.2f}-{max(rs):.2f}mm, mean {sum(rs)/len(rs):.2f}mm")
    # angular sector occupancy of the fanout band (30-deg sectors) — are sectors shared?
    sect = defaultdict(list)
    for r, th, nn in band:
        sect[int(th // 30)].append((round(th, 1), nn))
    print("    angular sectors (30deg) of fanout-band vias  [sector: count]:")
    for s in sorted(sect):
        names = ",".join(n for _, n in sect[s])
        print(f"      {s*30:>3}-{s*30+30:<3}deg: {len(sect[s]):>2}  ({names})")

    # ── (3) J3/J4 CONNECTOR channel structure ──
    print("\n" + "=" * 72)
    print("(3) J3/J4 CONNECTOR ENTRY — last-leg layer + spatial order (channel/bus)")
    print("=" * 72)
    # which pads belong to J3/J4 and their net + position, sorted along the row
    for ref in ("J3", "J4"):
        rows = []
        for p in pads:
            if p.get("ref") == ref and "at" in p:
                rows.append((p["at"], p.get("net_name", ""), p.get("pad_num", "?")))
        # connector pads are a row: sort by the longer axis
        if not rows:
            continue
        xs = [a[0] for a, _, _ in rows]; ys = [a[1] for a, _, _ in rows]
        along_x = (max(xs) - min(xs)) >= (max(ys) - min(ys))
        rows.sort(key=lambda t: t[0][0] if along_x else t[0][1])
        print(f"\n  {ref} (row along {'X' if along_x else 'Y'}):")
        for (x, y), nn, pn in rows:
            # find the segment incident to this pad and report its layer
            inc_layer = "?"
            best = 1e9
            for s in segs:
                if "start" not in s:
                    continue
                for ep in (s["start"], s["end"]):
                    d = math.hypot(ep[0] - x, ep[1] - y)
                    if d < best and d < 0.3:
                        best = d; inc_layer = s.get("layer", "?")
            print(f"    pad{pn:<3} ({x:6.2f},{y:6.2f}) net={nn:<14} last-leg-layer={inc_layer}")

    # ── (4) PER-NET layer-transition topology for the 13 hard nets ──
    print("\n" + "=" * 72)
    print("(4) PER-NET via position: source(QFN) vs dest(connector) side")
    print("=" * 72)
    HARD = ["/GPIO3", "/GPIO4", "/GPIO6", "/GPIO9", "/GPIO14", "/GPIO20",
            "/GPIO23", "/RUN", "/SWCLK", "/USB_D+"]  # the 10 B.Cu-escape nets
    name2num = {v: k for k, v in nets.items()}
    for nn in HARD:
        num = name2num.get(nn)
        if num is None:
            continue
        nvias = [v for v in vias if v.get("net_num") == num]
        # source QFN pad of this net (the U3 SMD pad)
        src = None
        for p in pads:
            if p.get("net_name") == nn and p.get("ref") == "U3":
                src = p["at"]
        if src is None:
            continue
        parts = []
        for v in nvias:
            if "at" not in v:
                continue
            d_u3 = math.hypot(v["at"][0] - u3[0], v["at"][1] - u3[1])
            d_src = math.hypot(v["at"][0] - src[0], v["at"][1] - src[1])
            parts.append(f"via@{d_u3:.1f}mm-from-U3 ({d_src:.1f}mm-from-srcpad)")
        print(f"  {nn:<12} src_pad=({src[0]:.2f},{src[1]:.2f})  {len(nvias)} via(s): {'; '.join(parts) if parts else 'none'}")


if __name__ == "__main__":
    main()
