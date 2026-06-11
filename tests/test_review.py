"""Tests for review rules, the LLM-pass guards, and report rendering."""

from tracewise.netlist import parse_netlist
from tracewise.review.engine import _dedupe, llm_findings, review_netlist
from tracewise.review.findings import Finding, Report
from tracewise.review.rules import (
    check_floating_inputs,
    check_i2c_pullups,
    check_power_pin_decoupling,
)


def make(nets: str, comps: str = "") -> str:
    comps = comps or """
    (comp (ref "U1") (value "MCU") (libsource (lib "X") (part "MCU")))
    (comp (ref "R1") (value "4k7") (libsource (lib "Device") (part "R")))
    (comp (ref "C1") (value "100n") (libsource (lib "Device") (part "C")))"""
    return f'(export (version "E") (components {comps}) (nets {nets}))'


# --- i2c pull-up rule -------------------------------------------------------


def test_i2c_without_pullup_flagged():
    nl = parse_netlist(make("""
      (net (code "1") (name "/SDA")
        (node (ref "U1") (pin "1") (pintype "bidirectional")))"""))
    found = check_i2c_pullups(nl)
    assert len(found) == 1
    assert found[0].category == "pullup" and "/SDA" in found[0].nets


def test_i2c_with_pullup_clean():
    nl = parse_netlist(make("""
      (net (code "1") (name "SCL1")
        (node (ref "U1") (pin "2") (pintype "bidirectional"))
        (node (ref "R1") (pin "1") (pintype "passive")))"""))
    assert check_i2c_pullups(nl) == []


def test_i2c_detected_by_pinfunction():
    nl = parse_netlist(make("""
      (net (code "1") (name "Net-(U1-Pad3)")
        (node (ref "U1") (pin "3") (pinfunction "SDA") (pintype "bidirectional")))"""))
    assert len(check_i2c_pullups(nl)) == 1


def test_sdcard_net_not_mistaken_for_i2c():
    nl = parse_netlist(make("""
      (net (code "1") (name "SD_DAT0")
        (node (ref "U1") (pin "4") (pintype "bidirectional")))"""))
    assert check_i2c_pullups(nl) == []


# --- decoupling rule --------------------------------------------------------


def test_power_pin_without_any_cap_flagged():
    nl = parse_netlist(make("""
      (net (code "1") (name "VCC")
        (node (ref "U1") (pin "7") (pinfunction "VDD") (pintype "power_in")))"""))
    found = check_power_pin_decoupling(nl)
    assert len(found) == 1 and found[0].category == "decoupling"


def test_power_pin_with_cap_clean():
    nl = parse_netlist(make("""
      (net (code "1") (name "VCC")
        (node (ref "U1") (pin "7") (pintype "power_in"))
        (node (ref "C1") (pin "1") (pintype "passive")))"""))
    assert check_power_pin_decoupling(nl) == []


def test_gnd_side_not_flagged():
    nl = parse_netlist(make("""
      (net (code "1") (name "GND")
        (node (ref "U1") (pin "8") (pintype "power_in")))"""))
    assert check_power_pin_decoupling(nl) == []


# --- floating input rule ----------------------------------------------------


def test_floating_input_flagged_as_error():
    nl = parse_netlist(make("""
      (net (code "1") (name "Net-(U1-EN)")
        (node (ref "U1") (pin "5") (pinfunction "EN") (pintype "input")))"""))
    found = check_floating_inputs(nl)
    assert len(found) == 1 and found[0].severity == "error"


def test_connected_input_clean():
    nl = parse_netlist(make("""
      (net (code "1") (name "EN")
        (node (ref "U1") (pin "5") (pintype "input"))
        (node (ref "R1") (pin "2") (pintype "passive")))"""))
    assert check_floating_inputs(nl) == []


# --- llm pass guards (stub client) -----------------------------------------


class StubLLM:
    model = "stub"

    def __init__(self, reply: str) -> None:
        self._reply = reply

    def available(self) -> bool:
        return True

    def chat(self, messages) -> str:
        return self._reply


NETLIST = parse_netlist(make("""
  (net (code "1") (name "VCC")
    (node (ref "U1") (pin "7") (pintype "power_in"))
    (node (ref "C1") (pin "1") (pintype "passive")))"""))


def test_llm_findings_parse_and_cap_confidence():
    reply = '[{"severity":"warning","category":"power","title":"VCC sag risk",' \
            '"detail":"...","nets":["VCC"],"refs":["U1"],"confidence":0.99}]'
    found = llm_findings(NETLIST, StubLLM(reply))
    assert len(found) == 1
    assert found[0].confidence == 0.8  # capped below deterministic rules


def test_llm_hallucinated_evidence_dropped():
    reply = '[{"severity":"error","category":"power","title":"Ghost net problem",' \
            '"detail":"...","nets":["NO_SUCH_NET"],"refs":["U99"],"confidence":0.9}]'
    assert llm_findings(NETLIST, StubLLM(reply)) == []


def test_llm_garbage_reply_yields_nothing():
    assert llm_findings(NETLIST, StubLLM("I think everything looks fine!")) == []
    assert llm_findings(NETLIST, StubLLM("<think>hmm</think>[]")) == []


def test_dedupe_prefers_first():
    a = Finding(severity="warning", category="pullup", title="rule version",
                nets=["SDA"], refs=["U1"], source="rule:i2c-pullup")
    b = Finding(severity="info", category="pullup", title="llm version",
                nets=["SDA"], refs=["U1"], source="llm:stub")
    out = _dedupe([a, b])
    assert len(out) == 1 and out[0].source == "rule:i2c-pullup"


# --- report -----------------------------------------------------------------


def test_review_netlist_merges_and_renders():
    nl = parse_netlist(make("""
      (net (code "1") (name "/SDA")
        (node (ref "U1") (pin "1") (pintype "bidirectional")))
      (net (code "2") (name "VCC")
        (node (ref "U1") (pin "7") (pintype "power_in")))"""))
    report = review_netlist(nl, project="t", client=None)
    assert report.counts()["warning"] == 2
    md = report.to_markdown()
    assert "I²C net" in md and "Supply net" in md and "rule:" in md


def test_empty_report_renders():
    r = Report(project="x", components=0, nets=0)
    assert "_No findings._" in r.to_markdown()
