"""Constraint generation: netlist semantics + board spec → net classes & rules.

The wedge this project exists for. Classification is deterministic first
(power rails, differential pairs, I²C/slow buses — mechanically recognizable),
with an optional LLM refinement pass for ambiguous signal nets. Output lands
in the two places KiCad actually reads:

- the project's ``.kicad_pro`` (JSON): net classes + per-class widths and
  pattern assignments — these flow into the board and the Specctra DSN export,
  which is how Freerouting sees them
- a ``.kicad_dru`` file: custom DRC rules for anything classes can't express

Width heuristics are deliberately conservative and documented; they are
starting constraints for an autorouter, not impedance engineering.
"""

from __future__ import annotations

import json
import re
from dataclasses import dataclass, field
from pathlib import Path

from tracewise.boardspec import BoardSpec
from tracewise.netlist import Netlist

POWER_NAME = re.compile(
    r"^\+?(VCC|VDD|VBUS|VBAT|VSYS|VIN|VOUT|PWR|[0-9]+V[0-9]*)$|^(GND|VSS|AGND|DGND|PGND)$",
    re.IGNORECASE,
)
DIFF_SUFFIX = re.compile(r"^(?P<stem>.+?)(?P<pol>[_-]?(P|N)|\+|-|_DP|_DM)$", re.IGNORECASE)
SLOW_BUS = re.compile(r"(^|[^A-Z])(SDA|SCL|TX|RX|CS|MOSI|MISO|SCK|EN|RST|NRST|IRQ)\d*$",
                      re.IGNORECASE)


@dataclass
class NetClass:
    name: str
    nets: list[str] = field(default_factory=list)
    track_mm: float = 0.2
    clearance_mm: float = 0.15
    via_dia_mm: float = 0.6
    via_drill_mm: float = 0.3
    description: str = ""


def find_diff_pairs(net_names: list[str]) -> list[tuple[str, str]]:
    """Pair nets like USB_DP/USB_DM, LVDS_P/LVDS_N, CAN+/CAN-."""
    stems: dict[str, dict[str, str]] = {}
    for name in net_names:
        m = DIFF_SUFFIX.match(name.rsplit("/", 1)[-1])
        if not m:
            continue
        pol = m.group("pol").upper()
        if pol not in ("+", "-"):  # bare +/- must not be stripped away
            pol = pol.lstrip("_-")
        pol = {"DP": "P", "DM": "N", "+": "P", "-": "N"}.get(pol, pol)
        stems.setdefault(m.group("stem").upper(), {})[pol] = name
    return [(d["P"], d["N"]) for d in stems.values() if "P" in d and "N" in d]


def classify(nl: Netlist, spec: BoardSpec) -> list[NetClass]:
    """Deterministic classification into Power / DiffPair / Slow / Default."""
    names = [n.name for n in nl.nets]
    power = NetClass(
        name="Power", track_mm=spec.power_track_mm, clearance_mm=spec.min_clearance_mm,
        via_dia_mm=spec.via.diameter_mm, via_drill_mm=spec.via.drill_mm,
        description="supply rails — wider copper for current and lower IR drop",
    )
    diff = NetClass(
        name="DiffPairs", track_mm=max(spec.min_track_mm, 0.2),
        clearance_mm=spec.min_clearance_mm, via_dia_mm=spec.via.diameter_mm,
        via_drill_mm=spec.via.drill_mm,
        description="differential pairs — route together, matched length",
    )
    slow = NetClass(
        name="SlowSignals", track_mm=max(spec.min_track_mm, 0.2),
        clearance_mm=spec.min_clearance_mm, via_dia_mm=spec.via.diameter_mm,
        via_drill_mm=spec.via.drill_mm,
        description="control/low-speed buses — relaxed routing priority",
    )

    paired = {n for pair in find_diff_pairs(names) for n in pair}
    for net in nl.nets:
        short = net.name.rsplit("/", 1)[-1]
        if POWER_NAME.match(short) or any(
            nd.pintype in ("power_in", "power_out") for nd in net.nodes
        ):
            power.nets.append(net.name)
        elif net.name in paired:
            diff.nets.append(net.name)
        elif SLOW_BUS.search(short):
            slow.nets.append(net.name)
    return [c for c in (power, diff, slow) if c.nets]


# --- emitters -----------------------------------------------------------------


def emit_dru(classes: list[NetClass], spec: BoardSpec) -> str:
    """KiCad custom design rules (.kicad_dru)."""
    lines = ["(version 1)", ""]
    for c in classes:
        lines += [
            f'(rule "tracewise_{c.name}"',
            f'\t(condition "A.NetClass == \'{c.name}\'")',
            f"\t(constraint track_width (min {c.track_mm}mm))",
            f"\t(constraint clearance (min {c.clearance_mm}mm))",
            ")",
            "",
        ]
    lines += [
        '(rule "tracewise_board_minimums"',
        f"\t(constraint track_width (min {spec.min_track_mm}mm))",
        f"\t(constraint clearance (min {spec.min_clearance_mm}mm))",
        ")",
        "",
    ]
    return "\n".join(lines)


def patch_kicad_pro(pro_path: str | Path, classes: list[NetClass]) -> None:
    """Write net classes + pattern assignments into the project JSON.

    Schema validated against KiCad 10 project files in the bridge integration;
    the Default class is preserved, tracewise-managed classes are replaced.
    """
    path = Path(pro_path)
    data = json.loads(path.read_text(encoding="utf-8"))
    ns = data.setdefault("net_settings", {})
    existing = [c for c in ns.get("classes", []) if c.get("name") == "Default"]
    for c in classes:
        existing.append(
            {
                "name": c.name,
                "track_width": c.track_mm,
                "clearance": c.clearance_mm,
                "via_diameter": c.via_dia_mm,
                "via_drill": c.via_drill_mm,
                "wire_width": 6, "bus_width": 12, "line_style": 0,
                "microvia_diameter": 0.3, "microvia_drill": 0.1,
                "diff_pair_width": c.track_mm, "diff_pair_gap": c.clearance_mm,
                "diff_pair_via_gap": 0.25,
                "pcb_color": "rgba(0, 0, 0, 0.000)", "schematic_color": "rgba(0, 0, 0, 0.000)",
            }
        )
    ns["classes"] = existing
    patterns = [p for p in ns.get("netclass_patterns", [])
                if not any(p.get("netclass") == c.name for c in classes)]
    for c in classes:
        for net in c.nets:
            patterns.append({"netclass": c.name, "pattern": net})
    ns["netclass_patterns"] = patterns
    path.write_text(json.dumps(data, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


def worth_constraining(classes: list[NetClass]) -> bool:
    """The first ablation's lesson: on boards with no constraint-sensitive nets
    (no diff pairs, no recognizable buses — just a bare Power class), emitted
    constraints add nothing and wider power tracks can *cost* DRC errors in
    tight legacy geometry. Emit only when the class set carries routing-
    meaningful structure beyond Power alone."""
    names = {c.name for c in classes}
    return bool(names - {"Power"})


def generate(
    nl: Netlist, spec: BoardSpec, project_dir: str | Path, conditional: bool = True
) -> dict:
    """Classify, write .kicad_dru, patch .kicad_pro. Returns a summary.

    With ``conditional`` (default), emission is skipped entirely when the
    board has nothing constraint-sensitive (see :func:`worth_constraining`)."""
    project_dir = Path(project_dir)
    classes = classify(nl, spec)
    if conditional and not worth_constraining(classes):
        return {
            "classes": {c.name: len(c.nets) for c in classes},
            "skipped": "no constraint-sensitive nets (use --force to emit anyway)",
            "kicad_pro": None,
            "kicad_dru": None,
        }
    pro = next(iter(project_dir.glob("*.kicad_pro")), None)
    dru_path = None
    if pro is not None:
        patch_kicad_pro(pro, classes)
        dru_path = pro.with_suffix(".kicad_dru")
        dru_path.write_text(emit_dru(classes, spec), encoding="utf-8")
    return {
        "classes": {c.name: len(c.nets) for c in classes},
        "kicad_pro": str(pro) if pro else None,
        "kicad_dru": str(dru_path) if dru_path else None,
    }
