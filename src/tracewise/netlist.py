"""Netlist extraction and the compressed electrical representation.

The Reviewer's input pipeline. KiCad's netlist export is itself an
s-expression document, so our lossless parser reads it directly — no second
parser, no third-party dependency:

    kicad-cli sch export netlist <project>.kicad_sch -o out.net
    → parse (tracewise.sexpr) → Component/Net model → compressed text for LLMs

The compressed representation exists because raw KiCad files are ~98% UUIDs,
coordinates and graphics; the electrical content of a board fits in a few
thousand tokens. Format (one line per net, components header up top) is
designed to be unambiguous for a model and diff-friendly for humans:

    [components]
    R1 10k Resistor_SMD:R_0603_1608Metric
    U1 STM32C031C4 Package_QFP:LQFP-48_7x7mm_P0.5mm
    [nets]
    VCC: C1.1 C2.1 U1.7(VDD/power_in) R3.2
    SDA: U1.42(PB7/bidi) R5.1 J2.3

kicad-cli discovery handles native installs and the flatpak fallback.
"""

from __future__ import annotations

import shutil
import subprocess
import tempfile
from dataclasses import dataclass, field
from pathlib import Path

from tracewise.sexpr import Node, parse


class NetlistError(RuntimeError):
    pass


# --- kicad-cli discovery ------------------------------------------------------


def find_kicad_cli() -> list[str] | None:
    """Return the command prefix for kicad-cli, or None if unavailable."""
    if shutil.which("kicad-cli"):
        return ["kicad-cli"]
    if shutil.which("flatpak"):
        probe = subprocess.run(
            ["flatpak", "info", "org.kicad.KiCad"], capture_output=True, text=True
        )
        if probe.returncode == 0:
            return ["flatpak", "run", "--command=kicad-cli", "org.kicad.KiCad"]
    return None


def export_netlist(schematic: str | Path, cli: list[str] | None = None) -> str:
    """Run kicad-cli to export the s-expression netlist; returns netlist text.

    Temp output lives under $HOME, not /tmp: flatpak KiCad's sandbox grants
    ``filesystems=home`` but maps /tmp privately, so host-side /tmp paths
    silently receive nothing.
    """
    cli = cli or find_kicad_cli()
    if cli is None:
        raise NetlistError("kicad-cli not found (native or flatpak)")
    work = Path.home() / ".cache" / "tracewise" / "tmp"
    work.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(dir=work) as td:
        out = Path(td) / "out.net"
        res = subprocess.run(
            [*cli, "sch", "export", "netlist", str(schematic), "-o", str(out)],
            capture_output=True, text=True,
        )
        if res.returncode != 0 or not out.exists():
            raise NetlistError(f"kicad-cli failed: {res.stderr.strip()[:300]}")
        return out.read_text(encoding="utf-8")


# --- model --------------------------------------------------------------------


@dataclass
class Component:
    ref: str
    value: str = ""
    footprint: str = ""
    libsource: str = ""
    properties: dict[str, str] = field(default_factory=dict)


@dataclass
class NetNode:
    ref: str
    pin: str
    pinfunction: str = ""
    pintype: str = ""


@dataclass
class Net:
    name: str
    nodes: list[NetNode] = field(default_factory=list)


@dataclass
class Netlist:
    components: list[Component] = field(default_factory=list)
    nets: list[Net] = field(default_factory=list)

    def component(self, ref: str) -> Component | None:
        return next((c for c in self.components if c.ref == ref), None)

    def net_of(self, ref: str, pin: str) -> Net | None:
        for net in self.nets:
            if any(n.ref == ref and n.pin == pin for n in net.nodes):
                return net
        return None


def _fields_of(comp: Node) -> dict[str, str]:
    out: dict[str, str] = {}
    fields = comp.first("fields")
    if fields:
        for f in fields.nodes("field"):
            name = f.first("name")
            key = name.arg() if name else f.arg(1)
            val = f.atoms()[-1].value if len(f.atoms()) > 1 else ""
            if key:
                out[key] = val
    return out


def parse_netlist(text: str) -> Netlist:
    """Parse a KiCad s-expression netlist export into the model."""
    root = parse(text)
    if root.name != "export":
        raise NetlistError(f"not a KiCad netlist (top node {root.name!r})")
    nl = Netlist()

    comps = root.first("components")
    for c in comps.nodes("comp") if comps else []:
        ref = c.first("ref")
        nl.components.append(
            Component(
                ref=ref.arg() if ref else "?",
                value=(c.first("value").arg() if c.first("value") else "") or "",
                footprint=(c.first("footprint").arg() if c.first("footprint") else "") or "",
                libsource=(
                    c.first("libsource").first("part").arg()
                    if c.first("libsource") and c.first("libsource").first("part")
                    else ""
                ) or "",
                properties=_fields_of(c),
            )
        )

    nets = root.first("nets")
    for n in nets.nodes("net") if nets else []:
        name_node = n.first("name")
        net = Net(name=(name_node.arg() if name_node else "?") or "?")
        for node in n.nodes("node"):
            net.nodes.append(
                NetNode(
                    ref=(node.first("ref").arg() if node.first("ref") else "?") or "?",
                    pin=(node.first("pin").arg() if node.first("pin") else "?") or "?",
                    pinfunction=(
                        node.first("pinfunction").arg() if node.first("pinfunction") else ""
                    ) or "",
                    pintype=(node.first("pintype").arg() if node.first("pintype") else "") or "",
                )
            )
        nl.nets.append(net)
    return nl


# --- compressed representation -------------------------------------------------


def compress(nl: Netlist, include_pintype: bool = True) -> str:
    """Render the netlist as compact text for LLM consumption."""
    lines = ["[components]"]
    for c in sorted(nl.components, key=lambda c: c.ref):
        bits = [c.ref, c.value or "?"]
        if c.libsource and c.libsource.lower() != (c.value or "").lower():
            bits.append(f"({c.libsource})")
        if c.footprint:
            bits.append(c.footprint)
        lines.append(" ".join(bits))
    lines.append("[nets]")
    for net in sorted(nl.nets, key=lambda n: n.name):
        ends = []
        for nd in net.nodes:
            tag = ""
            if nd.pinfunction or (include_pintype and nd.pintype):
                inner = nd.pinfunction
                if include_pintype and nd.pintype:
                    inner = f"{inner}/{nd.pintype}" if inner else nd.pintype
                tag = f"({inner})"
            ends.append(f"{nd.ref}.{nd.pin}{tag}")
        lines.append(f"{net.name}: {' '.join(ends)}")
    return "\n".join(lines) + "\n"


def token_estimate(text: str) -> int:
    """Rough token count (chars/4) for budgeting model context."""
    return len(text) // 4
