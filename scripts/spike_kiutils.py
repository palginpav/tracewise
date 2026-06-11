"""Spike 0: kiutils round-trip fidelity on current-format KiCad files.

kiutils' last release predates KiCad 10, so before building anything on it we
measure: can it parse current demo files, write them back, and preserve the
content? Three gates per file:

1. parse       — kiutils loads the file without error
2. reload      — the written file parses again (no corruption)
3. fidelity    — token-level s-expression diff between original and rewrite
                 (whitespace-insensitive; reports added/removed tokens)

Demo files come from KiCad's official GitLab (GPL — fetched, never committed).

    .venv/bin/python scripts/spike_kiutils.py [--branch 10.0]
"""

from __future__ import annotations

import re
from pathlib import Path

import httpx
import typer

app = typer.Typer(add_completion=False)

DEMOS = [
    # (project, files) — small + classic + hierarchical coverage
    ("flat_hierarchy", ["flat_hierarchy.kicad_pro", "flat_hierarchy.kicad_sch"]),
    ("pic_programmer", ["pic_programmer.kicad_sch", "pic_programmer.kicad_pcb"]),
    ("complex_hierarchy", ["complex_hierarchy.kicad_sch", "complex_hierarchy.kicad_pcb"]),
]
RAW = "https://gitlab.com/kicad/code/kicad/-/raw/{branch}/demos/{proj}/{file}"

_TOKEN = re.compile(r"\(|\)|\"(?:[^\"\\]|\\.)*\"|[^\s()\"]+")


def tokens(text: str) -> list[str]:
    return _TOKEN.findall(text)


def fetch(branch: str, dest: Path) -> list[Path]:
    out = []
    with httpx.Client(timeout=60, follow_redirects=True) as c:
        for proj, files in DEMOS:
            for f in files:
                p = dest / proj / f
                if p.exists():
                    out.append(p)
                    continue
                url = RAW.format(branch=branch, proj=proj, file=f)
                r = c.get(url)
                if r.status_code != 200:
                    typer.echo(f"  skip {proj}/{f}: HTTP {r.status_code}")
                    continue
                p.parent.mkdir(parents=True, exist_ok=True)
                p.write_bytes(r.content)
                out.append(p)
    return out


def roundtrip(path: Path) -> dict:
    from kiutils.board import Board
    from kiutils.schematic import Schematic

    cls = Schematic if path.suffix == ".kicad_sch" else Board
    result = {"file": str(path.name), "parse": False, "reload": False,
              "tok_orig": 0, "tok_rt": 0, "tok_missing": 0, "tok_added": 0, "error": ""}
    try:
        obj = cls.from_file(str(path))
        result["parse"] = True
    except Exception as e:
        result["error"] = f"parse: {type(e).__name__}: {e}"[:160]
        return result
    rt_path = path.with_suffix(path.suffix + ".rt")
    try:
        obj.to_file(str(rt_path))
        cls.from_file(str(rt_path))
        result["reload"] = True
    except Exception as e:
        result["error"] = f"write/reload: {type(e).__name__}: {e}"[:160]
        return result
    from collections import Counter

    orig = Counter(tokens(path.read_text(encoding="utf-8")))
    rt = Counter(tokens(rt_path.read_text(encoding="utf-8")))
    result["tok_orig"] = sum(orig.values())
    result["tok_rt"] = sum(rt.values())
    result["tok_missing"] = sum((orig - rt).values())
    result["tok_added"] = sum((rt - orig).values())
    return result


@app.command()
def main(
    branch: str = typer.Option("10.0", "--branch"),
    dest: Path = typer.Option(Path("data/demo-projects"), "--dest"),
) -> None:
    typer.echo(f"fetching demos from branch {branch}...")
    files = fetch(branch, dest)
    if not files:
        typer.echo("no files fetched — try --branch 9.0 or master", err=True)
        raise typer.Exit(1)
    kc_files = [f for f in files if f.suffix in (".kicad_sch", ".kicad_pcb")]
    typer.echo(f"round-tripping {len(kc_files)} files:\n")
    fails = 0
    for f in kc_files:
        r = roundtrip(f)
        ok = r["parse"] and r["reload"] and r["tok_missing"] == 0
        fails += 0 if ok else 1
        flag = "OK " if ok else "FAIL"
        typer.echo(
            f"  {flag} {r['file']}: parse={r['parse']} reload={r['reload']} "
            f"tokens {r['tok_orig']}→{r['tok_rt']} "
            f"(missing {r['tok_missing']}, added {r['tok_added']}) {r['error']}"
        )
    typer.echo(f"\nverdict: {len(kc_files) - fails}/{len(kc_files)} clean round-trips")
    raise typer.Exit(1 if fails else 0)


if __name__ == "__main__":
    app()
