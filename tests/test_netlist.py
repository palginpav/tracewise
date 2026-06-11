"""Tests for netlist parsing and the compressed representation.

The unit fixture is a hand-written KiCad-format netlist (synthetic, ours).
The integration test exports a real netlist via kicad-cli from a fetched demo
project and self-skips when either piece is absent.
"""

from pathlib import Path

import pytest

from tracewise.netlist import (
    compress,
    export_netlist,
    find_kicad_cli,
    parse_netlist,
    token_estimate,
)

FIXTURE = """(export (version "E")
  (components
    (comp (ref "R1")
      (value "10k")
      (footprint "Resistor_SMD:R_0603_1608Metric")
      (libsource (lib "Device") (part "R") (description "Resistor")))
    (comp (ref "C1")
      (value "100n")
      (footprint "Capacitor_SMD:C_0402_1005Metric")
      (libsource (lib "Device") (part "C") (description "Capacitor")))
    (comp (ref "U1")
      (value "STM32C031C4")
      (footprint "Package_QFP:LQFP-48_7x7mm_P0.5mm")
      (libsource (lib "MCU_ST") (part "STM32C031C4Tx") (description "MCU"))
      (fields (field (name "Datasheet") "stm32c031c4.pdf"))))
  (nets
    (net (code "1") (name "VCC")
      (node (ref "C1") (pin "1") (pintype "passive"))
      (node (ref "U1") (pin "7") (pinfunction "VDD") (pintype "power_in")))
    (net (code "2") (name "GND")
      (node (ref "C1") (pin "2") (pintype "passive"))
      (node (ref "U1") (pin "8") (pinfunction "VSS") (pintype "power_in")))
    (net (code "3") (name "/SDA")
      (node (ref "R1") (pin "1") (pintype "passive"))
      (node (ref "U1") (pin "42") (pinfunction "PB7") (pintype "bidirectional")))))
"""


def test_parse_components():
    nl = parse_netlist(FIXTURE)
    assert [c.ref for c in nl.components] == ["R1", "C1", "U1"]
    u1 = nl.component("U1")
    assert u1.value == "STM32C031C4"
    assert u1.libsource == "STM32C031C4Tx"
    assert u1.properties.get("Datasheet") == "stm32c031c4.pdf"


def test_parse_nets_and_lookup():
    nl = parse_netlist(FIXTURE)
    assert len(nl.nets) == 3
    vcc = nl.net_of("U1", "7")
    assert vcc is not None and vcc.name == "VCC"
    node = next(n for n in vcc.nodes if n.ref == "U1")
    assert node.pinfunction == "VDD" and node.pintype == "power_in"


def test_rejects_non_netlist():
    with pytest.raises(Exception, match="not a KiCad netlist"):
        parse_netlist("(kicad_sch (version 1))")


def test_compress_format():
    nl = parse_netlist(FIXTURE)
    text = compress(nl)
    assert "[components]" in text and "[nets]" in text
    assert "U1 STM32C031C4 (STM32C031C4Tx) Package_QFP:LQFP-48_7x7mm_P0.5mm" in text
    assert "VCC: C1.1(passive) U1.7(VDD/power_in)" in text
    # nets sorted, components sorted
    assert text.index("C1 100n") < text.index("R1 10k")


def test_compress_is_compact():
    nl = parse_netlist(FIXTURE)
    compressed = compress(nl)
    assert len(compressed) < len(FIXTURE) / 2
    assert token_estimate(compressed) < token_estimate(FIXTURE) / 2


# --- integration (self-skipping) ---------------------------------------------

DEMO_SCH = Path("data/demo-projects/pic_programmer/pic_programmer.kicad_sch")


def test_export_and_parse_real_project():
    cli = find_kicad_cli()
    if cli is None:
        pytest.skip("kicad-cli not available")
    if not DEMO_SCH.exists():
        pytest.skip("demo project not fetched (run scripts/spike_kiutils.py)")
    text = export_netlist(DEMO_SCH, cli=cli)
    nl = parse_netlist(text)
    assert len(nl.components) > 10
    assert len(nl.nets) > 10
    compressed = compress(nl)
    assert "[nets]" in compressed
    # the win the Reviewer depends on: electrical content is far smaller than the schematic
    sch_tokens = token_estimate(DEMO_SCH.read_text(encoding="utf-8"))
    net_tokens = token_estimate(compressed)
    assert net_tokens < sch_tokens / 5
