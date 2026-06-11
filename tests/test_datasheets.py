"""Tests for datasheet retrieval and the per-part verification guards."""

from tracewise.datasheets import retrieve, windows
from tracewise.netlist import parse_netlist
from tracewise.review.partcheck import part_wiring

TEXT = """
7.3 Recommended Operating Conditions
Supply voltage VDD: 1.7 V to 5.5 V. Operating free-air temperature -40 to 125 C.

7.4 Decoupling
Place a 100 nF capacitor close to the VDD pin. A bulk capacitor of 1 uF is recommended per rail.

8.1 Pin Functions
Pin 5 (EN): active high enable. Do not leave floating. Internal pulldown is NOT provided.

9.2 Package thermal data
Theta-JA 90 C/W for the DGK package.
"""


def test_windows_split_paragraphs():
    w = windows(TEXT, size=120)
    assert len(w) >= 3
    assert any("Recommended Operating" in x for x in w)


def test_retrieve_finds_relevant_window():
    hits = retrieve(TEXT, "EN pin enable floating", k=2)
    assert hits and "active high enable" in hits[0]


def test_retrieve_empty_query():
    assert retrieve(TEXT, "", k=2) == []


def test_part_wiring_renders_connections():
    nl = parse_netlist("""(export (version "E")
      (components
        (comp (ref "U1") (value "TPS7A33") (libsource (lib "L") (part "TPS7A3301"))))
      (nets
        (net (code "1") (name "VIN")
          (node (ref "U1") (pin "1") (pinfunction "IN") (pintype "power_in")))
        (net (code "2") (name "Net-(U1-EN)")
          (node (ref "U1") (pin "3") (pinfunction "EN") (pintype "input")))))""")
    text = part_wiring(nl, "U1")
    assert "pin 1 (IN/power_in) -> net VIN" in text
    assert "floating" in text  # the single-node EN net
