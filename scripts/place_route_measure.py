"""Measure placement quality by ROUTING-IN-THE-LOOP (the only metric that counts —
local placement scores mislead, per docs/PLAN.md). Re-place a board from its current
positions with the analytical placer, then route and report unconnected + errors.

    .venv/bin/python scripts/place_route_measure.py <board_dir> [iters]
"""
import re
import shutil
import sys
import time
from collections import Counter
from pathlib import Path

from tracewise.place.core import build_problem, optimize
from tracewise.place.extract import apply_positions, extract
from tracewise.route.bridge import drc_summary, run_drc, strip_routing
from tracewise.route.engine.kicad import route_board_engine

bdir = Path(sys.argv[1]); iters = int(sys.argv[2]) if len(sys.argv) > 2 else 400
work = Path(f"/tmp/pr_{bdir.name}"); shutil.rmtree(work, ignore_errors=True); work.mkdir(parents=True)
for f in bdir.iterdir():
    if f.suffix in (".kicad_pcb",".kicad_sch",".kicad_pro",".kicad_prl"): shutil.copy(f, work/f.name)
b = next(work.glob("*.kicad_pcb"))

def measure(tag):
    rep = run_drc(b); unc = Counter()
    for u in rep.get("unconnected_items", []):
        names = {re.search(r"\[([^\]]+)\]", i["description"]).group(1) for i in u["items"] if re.search(r"\[([^\]]+)\]", i["description"])}
        unc[tuple(sorted(names))] += 1
    drc = drc_summary(rep)
    print(f"[{tag}] unc={sum(unc.values())} err={drc['by_severity'].get('error',0)}", flush=True)

# 1) route the HUMAN placement (baseline)
strip_routing(b); route_board_engine(b); measure("human-placement")
# 2) re-place from scratch with the analytical placer, then route
data = extract(b); prob = build_problem(data)
t = time.time(); r = optimize(prob, iters=iters)
apply_positions(b, r["positions"])
print(f"placer: {time.time()-t:.0f}s hpwl {r['hpwl_before']:.1f}->{r['hpwl_after']:.1f} overlap_after={r['overlap_after']:.1f}", flush=True)
strip_routing(b); route_board_engine(b); measure("placer-placement")
print("PR DONE", flush=True)
