"""Tests for the lossless s-expression layer.

The contract under test is absolute: write(parse(text)) == text for any valid
KiCad file, byte for byte. Real-file tests use the official KiCad demos
(fetched by scripts/spike_kiutils.py); they self-skip when the files are not
present so CI without network still passes the synthetic suite.
"""

from pathlib import Path

import pytest

from tracewise.sexpr import SexprError, atom, node, parse, write

SAMPLE = """(kicad_sch
\t(version 20250114)
\t(generator "eeschema")
\t(symbol
\t\t(lib_id "Device:R")
\t\t(at 100.33 50.8 90)
\t\t(property "Reference" "R1"
\t\t\t(at 102.87 49.53 90)
\t\t)
\t\t(pin "1"
\t\t\t(uuid "deadbeef-0000-4000-8000-000000000001")
\t\t)
\t)
)
"""


# --- round-trip fidelity ----------------------------------------------------


def test_roundtrip_byte_identical():
    assert write(parse(SAMPLE)) == SAMPLE


def test_roundtrip_preserves_odd_whitespace():
    text = '(a   (b  "x y"\r\n   )\n\n  (c 1.5))\n\n'
    assert write(parse(text)) == text


def test_roundtrip_preserves_escaped_quotes():
    text = '(a (name "say \\"hi\\" twice"))'
    assert write(parse(text)) == text


def test_roundtrip_unicode():
    text = '(a (label "Überspannungsschutz µF Ω"))\n'
    assert write(parse(text)) == text


# --- parsing errors ---------------------------------------------------------


def test_unterminated_string_raises():
    with pytest.raises(SexprError, match="unterminated"):
        parse('(a "oops)')


def test_trailing_garbage_raises():
    with pytest.raises(SexprError, match="trailing"):
        parse("(a) (b)")


def test_missing_close_raises():
    with pytest.raises(SexprError):
        parse("(a (b)")


# --- navigation -------------------------------------------------------------


def test_navigation():
    root = parse(SAMPLE)
    assert root.name == "kicad_sch"
    sym = root.first("symbol")
    assert sym is not None
    assert sym.first("lib_id").arg() == "Device:R"
    props = sym.nodes("property")
    assert props[0].arg(1) == "Reference" and props[0].arg(2) == "R1"
    assert len(root.find_all("at")) == 2  # nested at-nodes found depth-first


def test_atom_value_unquotes():
    root = parse('(a (gen "eeschema") (ver 42))')
    assert root.first("gen").arg() == "eeschema"
    assert root.first("ver").arg() == "42"


# --- surgical editing -------------------------------------------------------


def test_set_arg_changes_only_that_token():
    root = parse(SAMPLE)
    prop = root.first("symbol").nodes("property")[0]
    prop.set_arg(2, '"R99"')
    out = write(root)
    assert '"R99"' in out
    # everything else untouched: replacing back restores the original exactly
    assert out.replace('"R99"', '"R1"', 1) == SAMPLE


def test_insert_child_uses_inferred_indentation():
    root = parse(SAMPLE)
    sym = root.first("symbol")
    sym.insert(node("uuid", atom("cafe-1234", quote=True)))
    out = write(root)
    assert '\n\t\t(uuid "cafe-1234")' in out
    # the rest of the file is unchanged
    assert out.replace('\n\t\t(uuid "cafe-1234")', "", 1) == SAMPLE


def test_remove_child():
    root = parse(SAMPLE)
    sym = root.first("symbol")
    pin = sym.first("pin")
    sym.remove(pin)
    out = write(root)
    assert "deadbeef" not in out
    reparsed = parse(out)
    assert reparsed.first("symbol").first("pin") is None


def test_builders_produce_parseable_output():
    n = node("net_class", atom("HighSpeed", quote=True), node("clearance", "0.15"))
    text = n.write()
    assert text == '(net_class "HighSpeed" (clearance 0.15))'
    assert write(parse(text)) == text


# --- real KiCad files (self-skip when not fetched) ---------------------------

DEMO_DIRS = [Path("data/demo-projects"), Path("data/demo-projects-v9")]
SEXPR_SUFFIXES = {".kicad_sch", ".kicad_pcb", ".kicad_sym", ".kicad_dru", ".kicad_mod"}
real_files = sorted(
    {
        f
        for d in DEMO_DIRS
        if d.exists()
        for f in d.rglob("*.kicad_*")
        if f.suffix in SEXPR_SUFFIXES  # .kicad_pro is JSON, not s-expression
    }
)


@pytest.mark.parametrize("path", real_files, ids=lambda p: p.name)
def test_real_kicad_file_roundtrips_byte_identical(path):
    text = path.read_text(encoding="utf-8")
    assert write(parse(text)) == text


def test_real_files_present_locally_or_skipped():
    if not real_files:
        pytest.skip("KiCad demo files not fetched (run scripts/spike_kiutils.py)")
