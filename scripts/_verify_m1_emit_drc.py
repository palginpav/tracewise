"""M1 focused real-DRC check: route ONE 2-pin net via the PRODUCTION gridless
package, emit it through emit_routes' world-centerline branch, and run REAL
kicad-cli DRC — confirming the refactored package + Phase-2 emit produce
DRC-clean copper (closes the 'structural-legal vs real-DRC' gap fast, without a
full-board route).

    taskset -c 0-9 .venv/bin/python scripts/_verify_m1_emit_drc.py
"""
from __future__ import annotations

import collections
import re
import shutil
from pathlib import Path

from tracewise.route.bridge import run_drc, strip_routing
from tracewise.route.engine.kicad import (
    build_problem,
    emit_routes,
    extract_pads,
    project_geometry,
    refill_zones,
)
from tracewise.route.gridless import route_net_gridless
from tracewise.route.gridless.adapter import to_gridless_netroute

SUFFIXES = (".kicad_pcb", ".kicad_sch", ".kicad_pro", ".kicad_prl")
BDIR = Path("data/benchmark-boards/mitayi-pico-d1").resolve()
OUT = Path("/tmp/verify_m1_emit").resolve()


def _err_names(report: dict, types: tuple[str, ...]) -> collections.Counter:
    c: collections.Counter = collections.Counter()
    for v in report.get("violations", []):
        if v.get("severity") == "error" and v.get("type") in types:
            for it in v.get("items", []):
                for m in re.findall(r"\[([^\]]+)\]", it.get("description", "")):
                    c[(v["type"], m)] += 1
    return c


def main() -> None:
    shutil.rmtree(OUT, ignore_errors=True)
    OUT.mkdir(parents=True)
    for f in BDIR.iterdir():
        if f.suffix in SUFFIXES:
            shutil.copy(f, OUT / f.name)
    board = next(OUT.glob("*.kicad_pcb"))
    strip_routing(board)

    data = extract_pads(board)
    grid, nets, anchors, *_ = build_problem(data)
    geo = project_geometry(board)
    board_bbox = (grid.x0, grid.y0, grid.x0 + grid.width_mm, grid.y0 + grid.height_mm)

    err_types = ("clearance", "hole_clearance", "shorting_items", "hole_to_hole")
    baseline = _err_names(run_drc(board), err_types)

    routed = []
    for net in nets:
        if len(net.pads) != 2 or not all(p[0] == 0 for p in net.pads):
            continue
        if "QSPI" in net.name:
            continue
        pa = anchors.get(net.pads[0])
        pb = anchors.get(net.pads[1])
        if pa is None or pb is None:
            continue
        res = route_net_gridless(pa, pb, data["pads"], net.name, geo, board_bbox)
        if res.ok:
            routed.append((net, res.world_paths))
        if len(routed) >= 3:
            break

    print(f"routed {len(routed)} nets gridless: {[n.name for n, _ in routed]}", flush=True)
    results = {n.name: to_gridless_netroute(n, wp, grid) for n, wp in routed}
    emit_routes(board, grid, results, anchors=anchors)
    refill_zones(board)

    after = _err_names(run_drc(board), err_types)
    new = after - baseline
    routed_names = {n.name for n, _ in routed}
    new_on_routed = {k: v for k, v in new.items() if k[1] in routed_names}
    print(f"baseline routing-error count={sum(baseline.values())}  "
          f"after={sum(after.values())}", flush=True)
    print(f"NEW errors total={sum(new.values())}  "
          f"NEW errors on the gridless nets={sum(new_on_routed.values())}", flush=True)
    for k, v in list(new.items())[:20]:
        print(f"  +{v} {k}", flush=True)
    verdict = "PASS" if not new_on_routed else "FAIL"
    print(f"VERDICT (package emit real-DRC): {verdict}  "
          f"(0 new clearance/short/hole errors on the emitted gridless nets == PASS)", flush=True)


if __name__ == "__main__":
    main()
