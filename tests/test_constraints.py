"""Tests for board spec, net classification, and constraint emission."""

import json

from tracewise.boardspec import BoardSpec
from tracewise.netlist import parse_netlist
from tracewise.route.constraints import (
    classify,
    emit_dru,
    find_diff_pairs,
    generate,
    patch_kicad_pro,
)

NETLIST = parse_netlist("""(export (version "E")
  (components
    (comp (ref "U1") (value "MCU") (libsource (lib "L") (part "MCU"))))
  (nets
    (net (code "1") (name "VCC")
      (node (ref "U1") (pin "7") (pintype "power_in")))
    (net (code "2") (name "GND")
      (node (ref "U1") (pin "8") (pintype "power_in")))
    (net (code "3") (name "USB_DP")
      (node (ref "U1") (pin "11") (pintype "bidirectional")))
    (net (code "4") (name "USB_DM")
      (node (ref "U1") (pin "12") (pintype "bidirectional")))
    (net (code "5") (name "/SDA")
      (node (ref "U1") (pin "42") (pintype "bidirectional")))
    (net (code "6") (name "Net-(U1-PA0)")
      (node (ref "U1") (pin "10") (pintype "output")))))""")


def test_boardspec_defaults():
    spec = BoardSpec()
    assert spec.layers == 2 and spec.via.diameter_mm == 0.6


def test_boardspec_yaml(tmp_path):
    (tmp_path / "tracewise.yaml").write_text(
        "layers: 4\npower_track_mm: 0.8\nvia: {diameter_mm: 0.45, drill_mm: 0.2}\n"
    )
    spec = BoardSpec.for_project(tmp_path)
    assert spec.layers == 4 and spec.power_track_mm == 0.8 and spec.via.drill_mm == 0.2


def test_boardspec_missing_file_is_defaults(tmp_path):
    assert BoardSpec.for_project(tmp_path).layers == 2


def test_find_diff_pairs():
    pairs = find_diff_pairs(["USB_DP", "USB_DM", "CAN+", "CAN-", "LVDS_P", "LVDS_N", "VCC"])
    assert ("USB_DP", "USB_DM") in pairs
    assert ("CAN+", "CAN-") in pairs
    assert ("LVDS_P", "LVDS_N") in pairs
    assert len(pairs) == 3


def test_classify_buckets():
    classes = {c.name: c for c in classify(NETLIST, BoardSpec())}
    assert set(classes["Power"].nets) == {"VCC", "GND"}
    assert set(classes["DiffPairs"].nets) == {"USB_DP", "USB_DM"}
    assert classes["SlowSignals"].nets == ["/SDA"]
    # unclassified nets stay in KiCad's Default class
    all_classed = {n for c in classes.values() for n in c.nets}
    assert "Net-(U1-PA0)" not in all_classed


def test_power_class_width_from_spec():
    spec = BoardSpec(power_track_mm=0.9)
    classes = {c.name: c for c in classify(NETLIST, spec)}
    assert classes["Power"].track_mm == 0.9


def test_emit_dru_syntax():
    spec = BoardSpec()
    dru = emit_dru(classify(NETLIST, spec), spec)
    assert dru.startswith("(version 1)")
    assert '(rule "tracewise_Power"' in dru
    assert "(constraint track_width (min 0.5mm))" in dru
    assert '(condition "A.NetClass == \'Power\'")' in dru


def test_patch_kicad_pro(tmp_path):
    pro = tmp_path / "demo.kicad_pro"
    pro.write_text(json.dumps({
        "net_settings": {
            "classes": [{"name": "Default", "track_width": 0.2}],
            "netclass_patterns": [{"netclass": "Old", "pattern": "X"}],
        }
    }))
    patch_kicad_pro(pro, classify(NETLIST, BoardSpec()))
    data = json.loads(pro.read_text())
    names = [c["name"] for c in data["net_settings"]["classes"]]
    assert "Default" in names and "Power" in names and "DiffPairs" in names
    pats = data["net_settings"]["netclass_patterns"]
    assert {"netclass": "Power", "pattern": "VCC"} in pats
    assert {"netclass": "Old", "pattern": "X"} in pats  # unrelated patterns preserved


def test_generate_writes_files(tmp_path):
    pro = tmp_path / "demo.kicad_pro"
    pro.write_text(json.dumps({"net_settings": {"classes": []}}))
    summary = generate(NETLIST, BoardSpec(), tmp_path)
    assert summary["classes"]["Power"] == 2
    assert (tmp_path / "demo.kicad_dru").exists()


def test_conditional_skips_power_only(tmp_path):
    import json as _json

    from tracewise.route.constraints import worth_constraining
    power_only = parse_netlist("""(export (version "E")
      (components (comp (ref "U1") (value "X") (libsource (lib "L") (part "X"))))
      (nets (net (code "1") (name "VCC")
        (node (ref "U1") (pin "1") (pintype "power_in")))))""")
    assert not worth_constraining(classify(power_only, BoardSpec()))
    pro = tmp_path / "x.kicad_pro"
    pro.write_text(_json.dumps({"net_settings": {"classes": []}}))
    summary = generate(power_only, BoardSpec(), tmp_path)
    assert summary["skipped"] and summary["kicad_dru"] is None
    assert not (tmp_path / "x.kicad_dru").exists()


def test_conditional_emits_with_diff_pairs(tmp_path):
    import json as _json
    pro = tmp_path / "x.kicad_pro"
    pro.write_text(_json.dumps({"net_settings": {"classes": []}}))
    summary = generate(NETLIST, BoardSpec(), tmp_path)  # has USB_DP/DM + /SDA
    assert "skipped" not in summary or not summary.get("skipped")
    assert (tmp_path / "x.kicad_dru").exists()


def test_clamp_to_project_minimums(tmp_path):
    from tracewise.route.constraints import clamp_to_project, read_project_rules
    pro = tmp_path / "x.kicad_pro"
    pro.write_text(json.dumps({
        "board": {"design_settings": {"rules": {"min_clearance": 0.25, "min_track_width": 0.3}}},
        "net_settings": {"classes": []},
    }))
    rules = read_project_rules(pro)
    assert rules == {"min_clearance_mm": 0.25, "min_track_mm": 0.3}
    classes = classify(NETLIST, BoardSpec())  # defaults 0.15/0.2
    clamp_to_project(classes, rules)
    assert all(c.clearance_mm >= 0.25 and c.track_mm >= 0.3 for c in classes)
    # generate() applies the clamp end-to-end
    summary = generate(NETLIST, BoardSpec(), tmp_path)
    dru = (tmp_path / "x.kicad_dru").read_text()
    assert "(min 0.25mm)" in dru and summary["classes"]


def test_read_project_rules_absent(tmp_path):
    from tracewise.route.constraints import read_project_rules
    pro = tmp_path / "x.kicad_pro"
    pro.write_text(json.dumps({"net_settings": {}}))
    assert read_project_rules(pro) == {}


def test_zone_clearance_floor(tmp_path):
    from tracewise.route.constraints import read_zone_clearance
    pcb = tmp_path / "x.kicad_pcb"
    pcb.write_text("""(kicad_pcb (version 1) (generator "t")
      (zone (net 1) (net_name "GND") (clearance 0.18))
      (zone (net 2) (net_name "T") (clearance 0))
    )""")
    assert read_zone_clearance(pcb) == 0.18
    pro = tmp_path / "x.kicad_pro"
    pro.write_text(json.dumps({"net_settings": {"classes": []}}))
    summary = generate(NETLIST, BoardSpec(), tmp_path)
    assert summary["classes"]
    dru = (tmp_path / "x.kicad_dru").read_text()
    assert "(constraint clearance (min 0.18mm))" in dru
