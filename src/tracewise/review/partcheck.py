"""Per-part datasheet verification: the RAG-grounded LLM pass.

For each component whose datasheet is cached locally, build a focused prompt:
how the part is actually wired (its pins, nets, neighbors) plus retrieved
datasheet excerpts (power pins, recommended operating, application notes),
and ask one narrow question — does this usage contradict the datasheet?

Same evidence guards as the netlist-level LLM pass; findings additionally
carry the datasheet excerpt as their citation. Parts without a cached
datasheet are skipped silently — this pass only ever adds grounded findings.
"""

from __future__ import annotations

from tracewise.datasheets import DatasheetStore, extract_text, retrieve
from tracewise.llm import OllamaClient, extract_json_array
from tracewise.netlist import Netlist
from tracewise.review.findings import Finding

PART_SYSTEM = """You are an electronics design reviewer verifying ONE component against excerpts \
from its manufacturer datasheet.

You receive: the component, every pin connection it has in this design, and datasheet excerpts.
Identify only CONTRADICTIONS between the design and the datasheet: required pins left \
unconnected, pins tied wrong (e.g. an active-low reset tied low permanently), missing required \
externals stated by the datasheet, supply outside the stated range IF voltages are evident.

Reply with ONLY a JSON array (possibly empty). Each element:
{"severity": "error"|"warning"|"info",
 "category": "pin-use"|"power"|"topology"|"other",
 "title": "<short>",
 "detail": "<the contradiction, referencing the datasheet wording>",
 "citation": "<short quote from the excerpt that supports the finding>",
 "confidence": 0.0-1.0}

If the excerpts don't clearly support a problem, return []."""


def part_wiring(nl: Netlist, ref: str) -> str:
    """Render one component's connections for the prompt."""
    lines = []
    for net in nl.nets:
        for node in net.nodes:
            if node.ref != ref:
                continue
            others = [f"{n.ref}.{n.pin}" for n in net.nodes if n.ref != ref][:8]
            fn = f" ({node.pinfunction}/{node.pintype})" if node.pinfunction or node.pintype \
                else ""
            lines.append(f"pin {node.pin}{fn} -> net {net.name}: {' '.join(others) or 'floating'}")
    return "\n".join(sorted(lines, key=lambda s: s.split()[1]))


def check_part(
    nl: Netlist, ref: str, client: OllamaClient, store: DatasheetStore | None = None
) -> list[Finding]:
    store = store or DatasheetStore()
    comp = nl.component(ref)
    if comp is None:
        return []
    part_name = (comp.libsource or comp.value or "").upper()
    pdf = store.available(part_name) or store.available(comp.value.upper())
    if pdf is None:
        return []

    wiring = part_wiring(nl, ref)
    text = extract_text(pdf)
    query = f"{comp.value} pin functions recommended operating decoupling {wiring[:300]}"
    excerpts = retrieve(text, query, k=4)
    if not excerpts:
        return []

    user = (
        f"Component {ref} = {comp.value} ({comp.libsource})\n\nConnections:\n{wiring}\n\n"
        f"Datasheet excerpts:\n" + "\n---\n".join(excerpts)
    )
    raw = client.chat(
        [{"role": "system", "content": PART_SYSTEM}, {"role": "user", "content": user}]
    )
    items = extract_json_array(raw)
    if not items:
        return []
    out = []
    for item in items:
        if not isinstance(item, dict):
            continue
        try:
            out.append(
                Finding(
                    severity=item.get("severity", "info"),
                    category=item.get("category", "other"),
                    title=str(item.get("title", ""))[:200],
                    detail=str(item.get("detail", ""))[:1000],
                    refs=[ref],
                    source=f"llm-datasheet:{client.model}",
                    confidence=min(float(item.get("confidence", 0.5)), 0.8),
                    datasheet_citation=str(item.get("citation", ""))[:300],
                )
            )
        except (ValueError, TypeError):
            continue
    return out


def check_all_parts(
    nl: Netlist, client: OllamaClient, store: DatasheetStore | None = None
) -> list[Finding]:
    store = store or DatasheetStore()
    out: list[Finding] = []
    for comp in nl.components:
        part_name = (comp.libsource or comp.value or "").upper()
        if store.available(part_name) or store.available(comp.value.upper()):
            out.extend(check_part(nl, comp.ref, client, store))
    return out
