"""Routing ablation: naked Freerouting vs TraceWise-constrained.

For each benchmark project: strip all routing, then route the same board two
ways — (a) as-is ("naked"), (b) after `constrain` generated net classes and
rules ("constrained") — and compare DRC violations, unconnected items, via
count and routed length. This is the measured form of the project's central
claim: constraint generation is where an LLM-era tool improves a classical
router.

    .venv/bin/python scripts/ablation_route.py [--out docs/route-ablation.md]
"""

from __future__ import annotations

import shutil
from pathlib import Path

import typer

from tracewise.boardspec import BoardSpec
from tracewise.netlist import export_netlist, parse_netlist
from tracewise.route.bridge import board_metrics, drc_summary, route_board, run_drc, strip_routing
from tracewise.route.constraints import generate

app = typer.Typer(add_completion=False)

WORK = Path.home() / ".cache" / "tracewise" / "ablation"


def prepare(src_project: Path, mode: str) -> Path:
    dest = WORK / src_project.name / mode
    if dest.exists():
        shutil.rmtree(dest)
    dest.mkdir(parents=True)
    for f in src_project.iterdir():
        if f.suffix in (".kicad_pcb", ".kicad_sch", ".kicad_pro"):
            shutil.copy(f, dest / f.name)
    return dest


def run_mode(project: Path, mode: str) -> dict:  # noqa: C901
    work = prepare(project, mode)
    board = next(work.glob("*.kicad_pcb"))
    # root sheet = the one named like the board (hierarchical projects have sub-sheets)
    sch = next((s for s in work.glob("*.kicad_sch") if s.stem == board.stem), None) or next(
        work.glob("*.kicad_sch"), None
    )
    pro = next(work.glob("*.kicad_pro"), None)
    pro_original = pro.read_text(encoding="utf-8") if pro else None

    strip_routing(board)
    if mode in ("engine", "pathfinder"):
        from tracewise.route.engine.kicad import route_board_engine

        route_board_engine(board, engine="pathfinder" if mode == "pathfinder" else "ripup")
    else:
        if mode == "constrained" and sch is not None:
            nl = parse_netlist(export_netlist(sch))
            spec = BoardSpec.for_project(work)
            generate(nl, spec, work)
        route_board(board, workdir=work)

    # FAIR SCORING: constraints may influence routing, but both arms must be
    # judged against the identical rule set — restore the original project
    # rules and drop TraceWise's .kicad_dru before the scoring DRC.
    if pro is not None and pro_original is not None:
        pro.write_text(pro_original, encoding="utf-8")
    for dru in work.glob("*.kicad_dru"):
        dru.unlink()

    report = run_drc(board)
    drc = drc_summary(report)
    drc["dangling"] = sum(1 for v in report.get("violations", [])
                          if v["type"] in ("track_dangling", "via_dangling"))
    metrics = board_metrics(board)
    return {**drc, **metrics}


@app.command()
def main(
    projects: str = typer.Option("pic_programmer", "--projects", help="comma-separated"),
    modes: str = typer.Option("naked,constrained", "--modes", help="naked,constrained,engine"),
    src_root: Path = typer.Option(Path("data/demo-projects"), "--src"),
    out: Path = typer.Option(Path("docs/route-ablation.md"), "--out"),
) -> None:
    lines = [
        "# Routing ablation — naked vs TraceWise-constrained Freerouting",
        "",
        "Same board, all routing stripped, routed twice: with KiCad's stock settings "
        "(naked) and after `tracewise constrain` generated net classes + rules "
        "(constrained). Freerouting 2.2.4, default effort.",
        "",
        "| Board | Mode | unconnected | DRC violations | dangling | vias "
        "| F/B split | length (mm) |",
        "|---|---|---|---|---|---|---|---|",
    ]
    for name in projects.split(","):
        project = src_root / name.strip()
        for mode in modes.split(","):
            typer.echo(f"=== {name} [{mode}] ...")
            r = run_mode(project, mode)
            typer.echo(f"    {r}")
            lines.append(
                f"| {name} | {mode} | {r['unconnected']} | {r['violations']} "
                f"| {r['dangling']} | {r['vias']} | {r.get('f_cu', '?')}/{r.get('b_cu', '?')} "
                f"| {r['length_mm']} |"
            )
    lines.append("")
    out.write_text("\n".join(lines), encoding="utf-8")
    typer.echo(f"wrote {out}")


if __name__ == "__main__":
    app()
