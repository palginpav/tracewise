"""Deterministic review rules over the netlist model.

These checks are mechanically decidable — no model, no false creativity.
Each returns findings with source ``rule:<id>`` and confidence 1.0 (the rule
fired) or slightly lower where the heuristic has known blind spots, stated in
the rule's docstring.

Conventions used:
- a component is resistor-like if its ref starts with R (RN for networks),
  capacitor-like if C, inductor-like if L — the KiCad refdes convention
- power nets are recognized by name (VCC/VDD/3V3/5V/GND/VSS/...) or by
  carrying power_in/power_out pins
"""

from __future__ import annotations

import re

from tracewise.netlist import Net, Netlist
from tracewise.review.findings import Finding

I2C_NET = re.compile(r"(^|[^A-Z])(SDA|SCL)\d*$", re.IGNORECASE)
POWER_NAME = re.compile(
    r"^\+?(VCC|VDD|VBUS|VBAT|VSYS|VIN|VOUT|[0-9]+V[0-9]*|3V3|5V|12V)|^(GND|VSS|AGND|DGND)$",
    re.IGNORECASE,
)


def _is_resistor(ref: str) -> bool:
    return bool(re.match(r"^RN?\d", ref))


def _is_capacitor(ref: str) -> bool:
    return bool(re.match(r"^C\d", ref))


def _net_refs(net: Net) -> set[str]:
    return {n.ref for n in net.nodes}


def check_i2c_pullups(nl: Netlist) -> list[Finding]:
    """I²C lines are open-drain and need pull-ups. Flags SDA/SCL-named nets
    (or pins whose function names SDA/SCL) with no resistor-like part attached.
    Blind spot: pull-ups integrated in a module are invisible to the netlist —
    confidence 0.9."""
    out = []
    for net in nl.nets:
        name_hit = bool(I2C_NET.search(net.name.rsplit("/", 1)[-1]))
        pin_hit = any(
            n.pinfunction and re.search(r"\b(SDA|SCL)\b", n.pinfunction, re.IGNORECASE)
            for n in net.nodes
        )
        if not (name_hit or pin_hit):
            continue
        if not any(_is_resistor(r) for r in _net_refs(net)):
            out.append(
                Finding(
                    severity="warning",
                    category="pullup",
                    title=f"I²C net {net.name!r} has no pull-up resistor",
                    detail=(
                        "I²C is open-drain; both SDA and SCL require pull-up resistors "
                        "(typically 2.2–10 kΩ depending on bus speed and capacitance). "
                        "No resistor-like component is attached to this net."
                    ),
                    nets=[net.name],
                    refs=sorted(_net_refs(net)),
                    source="rule:i2c-pullup",
                    confidence=0.9,
                    suggested_fix="Add pull-up resistors from SDA/SCL to the I/O supply rail.",
                )
            )
    return out


def check_power_pin_decoupling(nl: Netlist) -> list[Finding]:
    """Every IC power-input pin's net should also feed at least one capacitor.
    Net-level heuristic (the netlist has no geometry): a cap anywhere on the
    rail satisfies it, so this catches only the total absence of decoupling —
    deliberately conservative, confidence 0.85."""
    out = []
    seen: set[str] = set()
    for net in nl.nets:
        power_pins = [
            n for n in net.nodes
            if n.pintype == "power_in" and not _is_capacitor(n.ref) and not _is_resistor(n.ref)
        ]
        if not power_pins or net.name in seen:
            continue
        if re.search(r"GND|VSS", net.name, re.IGNORECASE):
            continue  # ground side; decoupling is judged on the supply side
        if not any(_is_capacitor(r) for r in _net_refs(net)):
            seen.add(net.name)
            refs = sorted({n.ref for n in power_pins})
            out.append(
                Finding(
                    severity="warning",
                    category="decoupling",
                    title=f"Supply net {net.name!r} has no capacitor",
                    detail=(
                        f"Power-input pins ({', '.join(f'{n.ref}.{n.pin}' for n in power_pins)}) "
                        "are fed by a net with no capacitor of any kind — not even bulk. "
                        "Each IC supply pin normally wants a local 100 nF plus bulk per rail."
                    ),
                    nets=[net.name],
                    refs=refs,
                    source="rule:power-decoupling",
                    confidence=0.85,
                    suggested_fix="Add 100 nF decoupling at each supply pin and bulk per rail.",
                )
            )
    return out


def check_floating_inputs(nl: Netlist) -> list[Finding]:
    """Input-type pins on single-node nets are floating. KiCad's ERC catches
    explicit no-connects; this catches nets that exist but go nowhere."""
    out = []
    for net in nl.nets:
        if len(net.nodes) != 1:
            continue
        node = net.nodes[0]
        if node.pintype == "input":
            out.append(
                Finding(
                    severity="error",
                    category="floating",
                    title=f"Input pin {node.ref}.{node.pin} is floating",
                    detail=(
                        f"Net {net.name!r} connects only to input pin {node.ref}.{node.pin}"
                        + (f" ({node.pinfunction})" if node.pinfunction else "")
                        + ". Floating inputs cause undefined logic levels and excess current."
                    ),
                    nets=[net.name],
                    refs=[node.ref],
                    source="rule:floating-input",
                    confidence=1.0,
                    suggested_fix="Tie the pin to a defined level or drive it.",
                )
            )
    return out


ALL_RULES = [check_i2c_pullups, check_power_pin_decoupling, check_floating_inputs]


def run_rules(nl: Netlist) -> list[Finding]:
    out: list[Finding] = []
    for rule in ALL_RULES:
        out.extend(rule(nl))
    return out
