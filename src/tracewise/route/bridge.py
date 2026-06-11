"""The external-router bridge: DSN export → Freerouting → SES import → DRC.

KiCad exposes no API to its interactive router, so this is the canonical
path (the same one KiCad's own Freerouting plugin uses):

1. export Specctra DSN from the board — via pcbnew's Python inside KiCad's
   own runtime (flatpak `--command=python3` or native python3 with pcbnew)
2. run the Freerouting JAR headless (Java 17+; the JAR is fetched once and
   cached under ~/.cache/tracewise)
3. import the resulting SES back into the board and refill zones
4. score the result with `kicad-cli pcb drc --format json`

Every step shells out — deliberately. The KiCad-side scripts are tiny and
run inside KiCad's environment, so SWIG version drift stays KiCad's problem,
not a linkage problem in our process.
"""

from __future__ import annotations

import json
import shutil
import subprocess
from pathlib import Path

import httpx

from tracewise.netlist import find_kicad_cli

FREEROUTING_VERSION = "2.2.4"
# the platform bundle ships its own JRE — the bare JAR needs a newer Java
# (class file 69 = Java 25) than most systems carry
FREEROUTING_URL = (
    "https://github.com/freerouting/freerouting/releases/download/"
    f"v{FREEROUTING_VERSION}/freerouting-{FREEROUTING_VERSION}-linux-x64.zip"
)
CACHE = Path.home() / ".cache" / "tracewise"


class BridgeError(RuntimeError):
    pass


# --- kicad python (pcbnew) discovery -----------------------------------------


def find_kicad_python() -> list[str] | None:
    """Command prefix for a python that can `import pcbnew`."""
    candidates = []
    if shutil.which("python3"):
        candidates.append(["python3"])
    if shutil.which("flatpak"):
        candidates.append(["flatpak", "run", "--command=python3", "org.kicad.KiCad"])
    for cmd in candidates:
        probe = subprocess.run(
            [*cmd, "-c", "import pcbnew"], capture_output=True, text=True
        )
        if probe.returncode == 0:
            return cmd
    return None


def _run_pcbnew_script(script: str, timeout: int = 300) -> str:
    py = find_kicad_python()
    if py is None:
        raise BridgeError("no python with pcbnew found (native or flatpak KiCad)")
    res = subprocess.run([*py, "-c", script], capture_output=True, text=True, timeout=timeout)
    if res.returncode != 0:
        raise BridgeError(f"pcbnew script failed: {res.stderr.strip()[:400]}")
    return res.stdout


HEAL_OUTLINE = """
import pcbnew
b = pcbnew.LoadBoard({board!r})
poly = pcbnew.SHAPE_POLY_SET()
if not b.GetBoardPolygonOutlines(poly, False):
    # strict outline is broken (micro-gaps are common on community boards and
    # are fatal to the DSN exporter); rebuild Edge.Cuts from the inferred poly
    assert b.GetBoardPolygonOutlines(poly, True), "outline not even inferable"
    for d in [d for d in b.GetDrawings() if d.GetLayer() == pcbnew.Edge_Cuts]:
        b.Remove(d)
    o = poly.Outline(0)
    n = o.PointCount()
    for i in range(n):
        a, c = o.CPoint(i), o.CPoint((i + 1) % n)
        s = pcbnew.PCB_SHAPE(b)
        s.SetShape(pcbnew.SHAPE_T_SEGMENT)
        s.SetStart(pcbnew.VECTOR2I(a.x, a.y))
        s.SetEnd(pcbnew.VECTOR2I(c.x, c.y))
        s.SetLayer(pcbnew.Edge_Cuts)
        s.SetWidth(int(0.1 * 1e6))
        b.Add(s)
    pcbnew.SaveBoard({board!r}, b)
    print("healed outline:", n, "segments")
# Community boards routinely contain footprints the DSN exporter cannot
# digest (logo marks with refs like "G***", label footprints, empty-ref screw
# holes) and the API reports only False. Self-heal: sanitize obviously bad
# refs, then if export still fails, auto-bisect the minimal set of offending
# footprints, drop them IN MEMORY ONLY (logos and holes carry no routed nets;
# the on-disk board keeps them, and the session file never references them),
# and disclose what was dropped.
import re as _re
for i, fp in enumerate(b.GetFootprints()):
    ref = fp.GetReference()
    if not ref or _re.search(r"[^A-Za-z0-9_.-]", ref):
        fp.SetReference("TWSAN%d" % i)
        print("sanitized ref for export:", repr(ref))

def _try():
    return pcbnew.ExportSpecctraDSN(b, {dsn!r})

if not _try():
    for _round in range(6):  # tolerate multiple independent culprits
        refs = [fp.GetReference() for fp in b.GetFootprints()]
        def _ok_with(keep):
            t = pcbnew.LoadBoard({board!r})
            byref = dict((f.GetReference() or "", f) for f in t.GetFootprints())
            for j, f in enumerate(list(t.GetFootprints())):
                r = f.GetReference()
                if not r or _re.search(r"[^A-Za-z0-9_.-]", r):
                    f.SetReference("TWSAN%d" % j)
            for f in list(t.GetFootprints()):
                if f.GetReference() not in keep:
                    t.Delete(f)
            return pcbnew.ExportSpecctraDSN(t, {dsn!r})
        suspects = [r for r in refs]
        if _ok_with(set(suspects)):
            break  # current in-memory board exports (culprits already dropped)
        while len(suspects) > 1:
            half = suspects[: len(suspects) // 2]
            if not _ok_with(set(half)):
                suspects = half
            else:
                suspects = suspects[len(suspects) // 2 :]
        culprit = suspects[0]
        for f in list(b.GetFootprints()):
            if f.GetReference() == culprit:
                print("dropped from export (DSN-incompatible):", culprit,
                      str(f.GetFPID().GetLibItemName()))
                b.Delete(f)
        if _try():
            break
raise SystemExit(0)
"""


def export_dsn(board: str | Path, dsn_out: str | Path) -> Path:
    """Specctra DSN export, healing unclosed Edge.Cuts outlines if needed.
    Paths must be visible to the (possibly sandboxed) KiCad runtime — keep
    them under $HOME. Outline healing mutates the board file (callers work on
    copies in routing flows)."""
    board, dsn_out = Path(board).resolve(), Path(dsn_out).resolve()
    _run_pcbnew_script(HEAL_OUTLINE.format(board=str(board), dsn=str(dsn_out)))
    if not dsn_out.exists():
        raise BridgeError("DSN export produced no file (outline unhealable?)")
    return dsn_out


def filter_ses(ses: str | Path) -> int:
    """Remove placement entries for export-sanitized refs (TWSANn) from a
    Specctra session so import matches the on-disk board, whose original
    references were never changed. Sessions are s-expressions — edited
    losslessly with our own core. Returns the number of entries removed."""
    from tracewise.sexpr import parse_file, write_file

    ses = Path(ses)
    root = parse_file(ses)
    removed = 0
    for comp in root.find_all("component"):
        for place in list(comp.nodes("place")):
            ref = place.arg(1)
            if ref and ref.startswith("TWSAN"):
                comp.remove(place)
                removed += 1
    if removed:
        # drop components left with no placements
        for parent in root.find_all("placement"):
            for comp in list(parent.nodes("component")):
                if not comp.nodes("place"):
                    parent.remove(comp)
        write_file(root, ses)
    return removed


def import_ses(board: str | Path, ses: str | Path) -> None:
    """Import a Specctra session back into the board and refill zones."""
    board, ses = Path(board).resolve(), Path(ses).resolve()
    _run_pcbnew_script(
        "import pcbnew; "
        f"b = pcbnew.LoadBoard({str(board)!r}); "
        f"ok = pcbnew.ImportSpecctraSES(b, {str(ses)!r}); "
        "filler = pcbnew.ZONE_FILLER(b); "
        "filler.Fill(b.Zones()); "
        f"pcbnew.SaveBoard({str(board)!r}, b); "
        "raise SystemExit(0 if ok else 1)"
    )


# --- freerouting ---------------------------------------------------------------


def fetch_freerouting() -> Path:
    """Fetch and unpack the self-contained Freerouting bundle; returns the launcher."""
    bundle = CACHE / "freerouting-bundle" / f"freerouting-{FREEROUTING_VERSION}-linux-x64"
    launcher = bundle / "bin" / "freerouting"
    if launcher.exists():
        return launcher
    CACHE.mkdir(parents=True, exist_ok=True)
    archive = CACHE / "freerouting.zip"
    with httpx.Client(timeout=300, follow_redirects=True) as c:
        r = c.get(FREEROUTING_URL)
        r.raise_for_status()
        archive.write_bytes(r.content)
    import zipfile

    with zipfile.ZipFile(archive) as z:
        z.extractall(CACHE / "freerouting-bundle")
    archive.unlink()
    launcher.chmod(0o755)
    if not launcher.exists():
        raise BridgeError("freerouting bundle missing its launcher after unpack")
    return launcher


def run_freerouting(dsn: str | Path, ses_out: str | Path, timeout_s: int = 1800) -> Path:
    launcher = fetch_freerouting()
    dsn, ses_out = Path(dsn).resolve(), Path(ses_out).resolve()
    res = subprocess.run(
        [str(launcher), "-de", str(dsn), "-do", str(ses_out), "-da"],
        capture_output=True, text=True, timeout=timeout_s,
    )
    if not ses_out.exists():
        raise BridgeError(f"freerouting produced no session: {res.stderr.strip()[:400]}")
    return ses_out


def strip_routing(board: str | Path) -> None:
    """Remove all tracks, arcs and vias — produce the unrouted starting point."""
    board = Path(board).resolve()
    _run_pcbnew_script(
        "import pcbnew; "
        f"b = pcbnew.LoadBoard({str(board)!r}); "
        "[b.Remove(t) for t in list(b.GetTracks())]; "
        f"pcbnew.SaveBoard({str(board)!r}, b)"
    )


def board_metrics(board: str | Path) -> dict:
    """Track/via counts and total routed length (mm) via pcbnew."""
    board = Path(board).resolve()
    out = _run_pcbnew_script(
        "import pcbnew, json; "
        f"b = pcbnew.LoadBoard({str(board)!r}); "
        "ts = list(b.GetTracks()); "
        "segs = [t for t in ts if t.GetClass() in ('PCB_TRACK','PCB_ARC')]; "
        "vias = [t for t in ts if t.GetClass()=='PCB_VIA']; "
        "print(json.dumps({'segments': len(segs), 'vias': len(vias), "
        "'length_mm': round(sum(t.GetLength() for t in segs)/1e6, 1)}))"
    )
    return json.loads(out.strip().splitlines()[-1])


# --- DRC ------------------------------------------------------------------------


def run_drc(board: str | Path) -> dict:
    """kicad-cli DRC, parsed JSON. Returns the report dict."""
    cli = find_kicad_cli()
    if cli is None:
        raise BridgeError("kicad-cli not found")
    board = Path(board).resolve()
    out = board.parent / f"{board.stem}.drc.json"
    res = subprocess.run(
        [*cli, "pcb", "drc", "--format", "json", "--severity-error", "--severity-warning",
         "-o", str(out), str(board)],
        capture_output=True, text=True,
    )
    if not out.exists():
        raise BridgeError(f"DRC produced no report: {res.stderr.strip()[:300]}")
    return json.loads(out.read_text(encoding="utf-8"))


def drc_summary(report: dict) -> dict:
    violations = report.get("violations", [])
    by_sev: dict[str, int] = {}
    for v in violations:
        by_sev[v.get("severity", "?")] = by_sev.get(v.get("severity", "?"), 0) + 1
    unconnected = report.get("unconnected_items", [])
    return {
        "violations": len(violations),
        "by_severity": by_sev,
        "unconnected": len(unconnected),
    }


# --- the loop --------------------------------------------------------------------


def route_board(board: str | Path, workdir: str | Path | None = None) -> dict:
    """DSN → Freerouting → SES → zones → DRC. Returns a summary dict."""
    board = Path(board).resolve()
    work = Path(workdir) if workdir else board.parent
    # Freerouting's launcher splits unquoted arguments on spaces — keep the
    # intermediate filenames space-free regardless of the board's name.
    stem = board.stem.replace(" ", "_")
    dsn = work / f"{stem}.dsn"
    ses = work / f"{stem}.ses"
    export_dsn(board, dsn)
    run_freerouting(dsn, ses)
    filter_ses(ses)
    import_ses(board, ses)
    report = run_drc(board)
    return {"board": str(board), "drc": drc_summary(report)}
