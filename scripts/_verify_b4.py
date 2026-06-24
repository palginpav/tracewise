import collections, shutil
from pathlib import Path
from tracewise.route.bridge import run_drc, strip_routing
from tracewise.route.engine.kicad import route_board_engine
SUF=(".kicad_pcb",".kicad_sch",".kicad_pro",".kicad_prl")
for name,bdir in [("zuluscsi","data/benchmark-boards/zuluscsi-pico-oshw"),
                  ("mitayi","data/benchmark-boards/mitayi-pico-d1")]:
    out=Path(f"/tmp/vb4_{name}"); shutil.rmtree(out,ignore_errors=True); out.mkdir(parents=True)
    for f in Path(bdir).iterdir():
        if f.suffix in SUF: shutil.copy(f,out/f.name)
    b=next(out.glob("*.kicad_pcb")); strip_routing(b)
    route_board_engine(b,pitch=0.1)
    rep=run_drc(b); errs=[v for v in rep.get("violations",[]) if v.get("severity")=="error"]
    by=collections.Counter(v.get("type") for v in errs)
    print(f"[{name}] unc={len(rep.get('unconnected_items',[]))} err={len(errs)} short={by.get('shorting_items',0)} smb={by.get('solder_mask_bridge',0)} by={dict(by)}",flush=True)
