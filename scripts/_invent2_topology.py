#!/usr/bin/env python3
"""
_invent2_topology.py — INVENTOR-2 throwaway: extract TOPOLOGY + CAPACITY data from
the human witness board to ground a topological/flow routing proposal.

Reuses the s-expr parser from probe_human_routing.py.

Quantifies:
  1. VIA POSITIONS of the failing-net escapes (radius from U3 center) — where the
     human places layer transitions; the QFN escape ring.
  2. CHANNEL CAPACITY around J3/J4: how many signal tracks cross a cut line between
     the QFN and the connector rows; compare to a Euclidean capacity bound.
  3. HOMOTOPY proxy: per failing net, the SEQUENCE of layer (F/B) and via crossings
     pad->...->pad (the topological sketch the human chose).
  4. TRACK-CROSSING audit: do any two different-net F.Cu segments cross? (they must
     not on a legal board) — sanity that the witness is clean, vs our 41/73 best
     which had illegal crossings inflating connectivity.
"""
import math
import sys
from collections import defaultdict
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from probe_human_routing import (  # noqa: E402
    parse_pcb, extract_segments, extract_vias, extract_nets, extract_pads,
    segment_length, PCB_PATH,
)

FAILING_NETS = {
    "/GPIO3", "/GPIO4", "/GPIO6", "/GPIO9", "/GPIO14",
    "/GPIO20", "/GPIO23", "/GPIO27", "/GPIO28",
    "/RUN", "/SWCLK", "/XIN", "/USB_D+",
}


def seg_intersect(p1, p2, p3, p4):
    """Proper segment intersection test (excludes shared endpoints)."""
    def ccw(a, b, c):
        return (c[1] - a[1]) * (b[0] - a[0]) - (b[1] - a[1]) * (c[0] - a[0])
    # shared endpoint => not a crossing
    pts = [p1, p2, p3, p4]
    for i in range(2):
        for j in range(2, 4):
            if abs(pts[i][0] - pts[j][0]) < 1e-6 and abs(pts[i][1] - pts[j][1]) < 1e-6:
                return False
    d1 = ccw(p3, p4, p1)
    d2 = ccw(p3, p4, p2)
    d3 = ccw(p1, p2, p3)
    d4 = ccw(p1, p2, p4)
    if ((d1 > 0) != (d2 > 0)) and ((d3 > 0) != (d4 > 0)):
        return True
    return False


def main():
    root = parse_pcb(PCB_PATH)
    nets = extract_nets(root)
    segments = extract_segments(root)
    vias = extract_vias(root)
    pads = extract_pads(root)
    name2num = {v: k for k, v in nets.items()}

    # ---- U3 (RP2040 QFN) center ----
    u3_center = None
    for fp in [f for f in _find_fps(root)]:
        if fp["ref"] == "U3":
            u3_center = (fp["x"], fp["y"])
    print(f"U3 center: {u3_center}")

    # ---- 1. VIA RING: radius of every via from U3 center ----
    print("\n" + "=" * 70)
    print("1. VIA RING — layer-transition positions (QFN escape topology)")
    print("=" * 70)
    via_radii = []
    failing_via_radii = []
    for v in vias:
        if "at" not in v or u3_center is None:
            continue
        r = math.hypot(v["at"][0] - u3_center[0], v["at"][1] - u3_center[1])
        nm = nets.get(v.get("net_num"), "?")
        via_radii.append((r, nm))
        if nm in FAILING_NETS:
            failing_via_radii.append((r, nm))
    via_radii.sort()
    failing_via_radii.sort()
    print(f"  total vias: {len(via_radii)}")
    if via_radii:
        rs = [r for r, _ in via_radii]
        print(f"  via radius from U3: min={min(rs):.2f} max={max(rs):.2f} "
              f"median={sorted(rs)[len(rs)//2]:.2f}")
    # histogram of radii in 1mm bins
    bins = defaultdict(int)
    for r, _ in via_radii:
        bins[int(r)] += 1
    print("  radius histogram (1mm bins):")
    for b in sorted(bins):
        print(f"    {b:>2}-{b+1}mm: {'#'*bins[b]} ({bins[b]})")
    print(f"\n  FAILING-NET vias (the escapes we can't reproduce): {len(failing_via_radii)}")
    for r, nm in failing_via_radii:
        print(f"    {nm:<10} via @ r={r:.2f}mm from U3")

    # ---- 2. CHANNEL CAPACITY: cut-line crossing count ----
    # The QFN escape funnels signals OUTWARD then to J3 (left) / J4 (right) headers.
    # Build a capacity argument: count tracks crossing a vertical cut between U3 and
    # each header column, per layer, and compare to a width-based bound.
    print("\n" + "=" * 70)
    print("2. CHANNEL CAPACITY — tracks crossing cut lines (flow/min-cut probe)")
    print("=" * 70)
    # locate J3, J4 centers
    fp_centers = {f["ref"]: (f["x"], f["y"]) for f in _find_fps(root)}
    for ref in ("J3", "J4", "J1", "J5", "U3"):
        if ref in fp_centers:
            print(f"  {ref} @ {fp_centers[ref][0]:.2f},{fp_centers[ref][1]:.2f}")

    # For each header, define a cut at x = midpoint(U3, header) and count crossings.
    for ref in ("J3", "J4"):
        if ref not in fp_centers or u3_center is None:
            continue
        hx = fp_centers[ref][0]
        cut_x = (u3_center[0] + hx) / 2.0
        for layer in ("F.Cu", "B.Cu"):
            crossings = 0
            crossing_nets = set()
            widths = []
            ys = []
            for s in segments:
                if s.get("layer") != layer:
                    continue
                if "start" not in s or "end" not in s:
                    continue
                x1, x2 = s["start"][0], s["end"][0]
                if (x1 - cut_x) * (x2 - cut_x) < 0:  # crosses vertical line
                    crossings += 1
                    crossing_nets.add(nets.get(s.get("net_num"), "?"))
                    widths.append(s.get("width", 0.2))
                    # y at crossing
                    t = (cut_x - x1) / (x2 - x1) if x2 != x1 else 0
                    ys.append(s["start"][1] + t * (s["end"][1] - s["start"][1]))
            yspan = (max(ys) - min(ys)) if ys else 0
            occupied = sum(w + 0.15 for w in widths)  # track + clearance halo
            print(f"  cut x={cut_x:.1f} (U3<->{ref}) {layer}: "
                  f"{crossings} tracks, {len(crossing_nets)} nets, "
                  f"y-span={yspan:.1f}mm, occupied≈{occupied:.1f}mm")

    # ---- 3. HOMOTOPY SKETCH proxy per failing net ----
    print("\n" + "=" * 70)
    print("3. HOMOTOPY SKETCH — layer/via sequence per failing net (the topology)")
    print("=" * 70)
    for nm in sorted(FAILING_NETS):
        num = name2num.get(nm)
        if num is None:
            continue
        segs = [s for s in segments if s.get("net_num") == num]
        nvias = [v for v in vias if v.get("net_num") == num]
        # order segments into a path by endpoint chaining (approx)
        layer_seq = _chain_layers(segs)
        print(f"  {nm:<10} layers={layer_seq} vias={len(nvias)} "
              f"segs={len(segs)} len={sum(segment_length(s) for s in segs):.1f}mm")

    # ---- 4. TRACK-CROSSING AUDIT (witness legality sanity) ----
    print("\n" + "=" * 70)
    print("4. TRACK-CROSSING AUDIT — do different-net same-layer tracks cross?")
    print("=" * 70)
    for layer in ("F.Cu", "B.Cu"):
        ls = [s for s in segments
              if s.get("layer") == layer and "start" in s and "end" in s]
        crossings = 0
        examples = []
        for i in range(len(ls)):
            for j in range(i + 1, len(ls)):
                if ls[i].get("net_num") == ls[j].get("net_num"):
                    continue
                if seg_intersect(ls[i]["start"], ls[i]["end"],
                                 ls[j]["start"], ls[j]["end"]):
                    crossings += 1
                    if len(examples) < 5:
                        examples.append((nets.get(ls[i].get("net_num"), "?"),
                                         nets.get(ls[j].get("net_num"), "?")))
        print(f"  {layer}: {len(ls)} segs, {crossings} different-net crossings")
        for a, b in examples:
            print(f"      {a} x {b}")


def _find_fps(root):
    """Yield dicts {ref,x,y} for each footprint."""
    out = []

    def walk(node):
        if isinstance(node, list) and node and node[0] == "footprint":
            ref, x, y = None, 0.0, 0.0
            for child in node[1:]:
                if isinstance(child, list) and child:
                    if child[0] == "at" and len(child) >= 3:
                        try:
                            x, y = float(child[1]), float(child[2])
                        except Exception:
                            pass
                    if child[0] == "property" and len(child) >= 3 and child[1] == "Reference":
                        ref = child[2]
            out.append({"ref": ref, "x": x, "y": y})
        if isinstance(node, list):
            for c in node[1:]:
                walk(c)
    walk(root)
    return out


def _chain_layers(segs):
    """Approximate layer sequence along the net path by chaining endpoints."""
    if not segs:
        return []
    seq = []
    for s in segs:
        ly = "F" if s.get("layer") == "F.Cu" else "B"
        seq.append(ly)
    # collapse runs
    out = []
    for ly in seq:
        if not out or out[-1] != ly:
            out.append(ly)
    return "".join(out)


if __name__ == "__main__":
    main()
