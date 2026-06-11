"""Seeded-error benchmark for the Reviewer.

Runs the review over benchmarks/seeded/*.net (synthetic netlists with known
faults and clean controls) and scores against expected.json:

- recall    — expected findings that were produced (category + evidence match)
- precision — produced findings that were expected (anything else on a clean
              board is a false positive)

Rules-only mode is deterministic and CI-runnable; --llm adds the model passes.

    .venv/bin/python scripts/benchmark_review.py [--llm] [--out docs/review-benchmark.md]
"""

from __future__ import annotations

import json
from pathlib import Path

import typer

from tracewise.netlist import parse_netlist
from tracewise.review.engine import review_netlist

app = typer.Typer(add_completion=False)


def matches(finding, exp) -> bool:
    return finding.category == exp["category"] and (
        exp["evidence"] in finding.nets or exp["evidence"] in finding.refs
    )


@app.command()
def main(
    suite: Path = typer.Option(Path("benchmarks/seeded"), "--suite"),
    llm: bool = typer.Option(False, "--llm/--no-llm"),
    model: str = typer.Option("qwen3:4b", "--model"),
    out: Path = typer.Option(Path("docs/review-benchmark.md"), "--out"),
) -> None:
    expected = json.loads((suite / "expected.json").read_text(encoding="utf-8"))
    expected.pop("_doc", None)

    client = None
    if llm:
        from tracewise.llm import OllamaClient

        client = OllamaClient(model=model)
        if not client.available():
            typer.echo("Ollama unreachable; running rules-only", err=True)
            client = None

    rows, tp = [], 0
    total_expected = total_found = 0
    for case, exps in sorted(expected.items()):
        nl = parse_netlist((suite / f"{case}.net").read_text(encoding="utf-8"))
        report = review_netlist(nl, project=case, client=client)
        found = report.findings
        hit = [e for e in exps if any(matches(f, e) for f in found)]
        false_pos = [f for f in found if not any(matches(f, e) for e in exps)]
        tp += len(hit)
        total_expected += len(exps)
        total_found += len(found)
        status = "✅" if len(hit) == len(exps) and not false_pos else "❌"
        rows.append(
            f"| {case} | {len(exps)} | {len(hit)} | {len(false_pos)} | {status} |"
        )
        for f in false_pos:
            rows.append(f"| ↳ FP: {f.title[:60]} | | | `{f.source}` | |")

    recall = tp / total_expected if total_expected else 1.0
    precision = tp / total_found if total_found else 1.0
    mode = f"rules + llm ({model})" if client else "rules-only"
    lines = [
        "# Reviewer benchmark — seeded errors",
        "",
        f"Mode: **{mode}** · {len(expected)} cases "
        f"({sum(1 for e in expected.values() if not e)} clean controls)",
        "",
        f"**Recall {recall:.2f} · Precision {precision:.2f}** "
        f"({tp}/{total_expected} expected found, {total_found - tp} false positives)",
        "",
        "| Case | expected | found | false-pos | |",
        "|---|---|---|---|---|",
        *rows,
        "",
    ]
    out.write_text("\n".join(lines), encoding="utf-8")
    typer.echo("\n".join(lines[2:6]))
    typer.echo(f"wrote {out}")
    raise typer.Exit(0 if recall == 1.0 and precision == 1.0 else 1)


if __name__ == "__main__":
    app()
