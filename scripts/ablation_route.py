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


def run_mode(project: Path, mode: str) -> dict:
    work = prepare(project, mode)
    board = next(work.glob("*.kicad_pcb"))
    sch = next(work.glob("*.kicad_sch"), None)
    strip_routing(board)
    if mode == "constrained" and sch is not None:
        nl = parse_netlist(export_netlist(sch))
        spec = BoardSpec.for_project(work)
        generate(nl, spec, work)
    route_board(board, workdir=work)
    drc = drc_summary(run_drc(board))
    metrics = board_metrics(board)
    return {**drc, **metrics}


@app.command()
def main(
    projects: str = typer.Option("pic_programmer", "--projects", help="comma-separated"),
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
        "| Board | Mode | unconnected | DRC violations | vias | length (mm) |",
        "|---|---|---|---|---|---|",
    ]
    for name in projects.split(","):
        project = src_root / name.strip()
        for mode in ("naked", "constrained"):
            typer.echo(f"=== {name} [{mode}] ...")
            r = run_mode(project, mode)
            typer.echo(f"    {r}")
            lines.append(
                f"| {name} | {mode} | {r['unconnected']} | {r['violations']} "
                f"| {r['vias']} | {r['length_mm']} |"
            )
    lines.append("")
    out.write_text("\n".join(lines), encoding="utf-8")
    typer.echo(f"wrote {out}")


if __name__ == "__main__":
    app()
