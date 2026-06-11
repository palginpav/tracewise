"""TraceWise CLI.

    tracewise review <schematic.kicad_sch> [--llm/--no-llm] [--model qwen3:4b]
                                           [--json out.json] [--out report.md]
"""

from __future__ import annotations

from pathlib import Path

import typer

app = typer.Typer(add_completion=False)


@app.callback()
def main() -> None:
    """TraceWise: AI-assisted place & route for KiCad."""


@app.command()
def review(
    schematic: Path = typer.Argument(..., help="Path to .kicad_sch"),
    llm: bool = typer.Option(True, "--llm/--no-llm", help="Include the LLM pass"),
    model: str = typer.Option("qwen3:4b", "--model"),
    out: Path | None = typer.Option(None, "--out", help="Write markdown report here"),
    json_out: Path | None = typer.Option(None, "--json", help="Write JSON report here"),
) -> None:
    from tracewise.llm import OllamaClient
    from tracewise.review.engine import review_schematic

    client = OllamaClient(model=model) if llm else None
    if client is not None and not client.available():
        typer.echo("note: Ollama not reachable — running rules only", err=True)
        client = None
    report = review_schematic(schematic, client=client)
    md = report.to_markdown()
    if out:
        out.write_text(md, encoding="utf-8")
        typer.echo(f"wrote {out}")
    else:
        typer.echo(md)
    if json_out:
        json_out.write_text(report.to_json(), encoding="utf-8")
        typer.echo(f"wrote {json_out}")
    c = report.counts()
    raise typer.Exit(1 if c["error"] else 0)


@app.command()
def constrain(
    schematic: Path = typer.Argument(..., help="Path to .kicad_sch"),
    force: bool = typer.Option(False, "--force", help="Emit even without sensitive nets"),
) -> None:
    """Generate net classes + design rules from the netlist and tracewise.yaml."""
    from tracewise.boardspec import BoardSpec
    from tracewise.netlist import export_netlist, parse_netlist
    from tracewise.route.constraints import generate

    nl = parse_netlist(export_netlist(schematic))
    spec = BoardSpec.for_project(schematic.parent)
    summary = generate(nl, spec, schematic.parent, conditional=not force)
    typer.echo(f"classes: {summary['classes']}")
    if summary.get("skipped"):
        typer.echo(f"skipped: {summary['skipped']}")
    else:
        typer.echo(f"patched: {summary['kicad_pro']}")
        typer.echo(f"rules:   {summary['kicad_dru']}")


@app.command()
def route(
    board: Path = typer.Argument(..., help="Path to .kicad_pcb"),
    engine: str = typer.Option("freerouting", "--engine", help="freerouting | tracewise"),
) -> None:
    """Route the board (Freerouting bridge or the TraceWise engine) + DRC."""
    if engine == "tracewise":
        from tracewise.route.bridge import drc_summary, run_drc
        from tracewise.route.engine.kicad import route_board_engine

        s = route_board_engine(board)
        typer.echo(f"engine: {s['routed']}/{s['nets']} nets, "
                   f"{s['segments']} segments, {s['vias']} vias")
        for n, why in list(s["failures"].items())[:8]:
            typer.echo(f"  failed {n}: {why}")
        summary = {"drc": drc_summary(run_drc(board))}
    else:
        from tracewise.route.bridge import route_board

        summary = route_board(board)
    typer.echo(f"DRC: {summary['drc']}")
    raise typer.Exit(1 if summary["drc"]["violations"] else 0)


@app.command()
def place(
    board: Path = typer.Argument(..., help="Path to .kicad_pcb"),
    apply: bool = typer.Option(False, "--apply", help="Write optimized positions back"),
    iters: int = typer.Option(800, "--iters"),
    lock: str = typer.Option("", "--lock", help="comma-separated refs to lock"),
) -> None:
    """Analytical placement: optimize footprint positions (dry-run by default)."""
    from tracewise.place.core import build_problem, optimize
    from tracewise.place.extract import apply_positions, extract

    data = extract(board)
    prob = build_problem(data, lock_refs={r.strip() for r in lock.split(",") if r.strip()})
    result = optimize(prob, iters=iters)
    typer.echo(
        f"HPWL {result['hpwl_before']:.1f} -> {result['hpwl_after']:.1f} mm "
        f"({100 * (1 - result['hpwl_after'] / max(result['hpwl_before'], 1e-9)):.1f}% better) · "
        f"overlap {result['overlap_after']:.1f} mm² "
        f"(initial layout: {result['overlap_initial']:.1f})"
    )
    if apply:
        apply_positions(board, result["positions"])
        typer.echo(f"applied {len(result['positions'])} positions to {board}")


@app.command()
def auto(
    board: Path = typer.Argument(..., help="Path to .kicad_pcb (will be routed in place)"),
    iters: int = typer.Option(5, "--iters"),
) -> None:
    """Iterative route refinement: failed nets feed back as priority."""
    from tracewise.route.engine.auto import auto_route

    r = auto_route(board, max_iters=iters)
    for h in r["iterations"]:
        typer.echo(f"  iter {h['iter']}: routed {h['routed']}, "
                   f"unconnected {h['unconnected']}, errors {h['errors']}, "
                   f"boosted {h['boosted']}")
    typer.echo(f"best: {r['best_unconnected']} unconnected, {r['best_errors']} errors")
    raise typer.Exit(0 if r["best_unconnected"] == 0 and r["best_errors"] == 0 else 1)


if __name__ == "__main__":
    app()
