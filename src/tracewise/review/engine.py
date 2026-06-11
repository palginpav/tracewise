"""Review engine: deterministic rules + the LLM pass, merged into one report.

The LLM is constrained hard: it receives the compressed netlist and must
return a JSON array of findings referencing real nets/refs. Findings whose
evidence doesn't exist in the netlist are dropped (hallucination guard), and
everything the rules already flagged is deduplicated. The LLM's confidence is
capped below the deterministic rules' — a model's hunch never outranks a
mechanical check.
"""

from __future__ import annotations

from pathlib import Path

from pydantic import ValidationError

from tracewise.llm import OllamaClient, extract_json_array
from tracewise.netlist import Netlist, compress, export_netlist, parse_netlist
from tracewise.review.findings import Finding, Report
from tracewise.review.rules import run_rules

LLM_SYSTEM = """You are an experienced electronics design reviewer. You receive a compressed \
netlist: components (ref, value, part, footprint) and nets (each endpoint as ref.pin with \
pin function/type where known).

Identify REAL design mistakes only — wrong or missing connections, suspicious component values, \
misused pins, missing protection. Do not comment on style. Do not repeat these already-checked \
items: I2C pull-ups, missing supply capacitors, floating single-node inputs.

Reply with ONLY a JSON array (possibly empty). Each element:
{"severity": "error"|"warning"|"info",
 "category": "power"|"pin-use"|"polarity"|"topology"|"other",
 "title": "<short>",
 "detail": "<why this is a problem, specific to THIS netlist>",
 "nets": ["<net names involved>"],
 "refs": ["<component refs involved>"],
 "confidence": 0.0-1.0}

Only include findings where the evidence is visible in the netlist. If nothing is clearly \
wrong, return []."""


def llm_findings(nl: Netlist, client: OllamaClient) -> list[Finding]:
    raw = client.chat(
        [
            {"role": "system", "content": LLM_SYSTEM},
            {"role": "user", "content": compress(nl)},
        ]
    )
    items = extract_json_array(raw)
    if items is None:
        return []
    net_names = {n.name for n in nl.nets}
    refs = {c.ref for c in nl.components}
    out: list[Finding] = []
    for item in items:
        if not isinstance(item, dict):
            continue
        try:
            f = Finding(
                severity=item.get("severity", "info"),
                category=item.get("category", "other"),
                title=str(item.get("title", ""))[:200],
                detail=str(item.get("detail", ""))[:1000],
                nets=[n for n in item.get("nets", []) if n in net_names],
                refs=[r for r in item.get("refs", []) if r in refs],
                source=f"llm:{client.model}",
                confidence=min(float(item.get("confidence", 0.5)), 0.8),
            )
        except (ValidationError, TypeError, ValueError):
            continue
        # hallucination guard: a finding must cite at least one real net or ref
        if not f.nets and not f.refs:
            continue
        out.append(f)
    return out


def _dedupe(findings: list[Finding]) -> list[Finding]:
    seen: set[tuple] = set()
    out = []
    for f in findings:  # rules come first; their version of a finding wins
        key = (f.category, tuple(sorted(f.nets)), tuple(sorted(f.refs)))
        if key in seen:
            continue
        seen.add(key)
        out.append(f)
    return out


def review_netlist(nl: Netlist, project: str, client: OllamaClient | None = None) -> Report:
    findings = run_rules(nl)
    if client is not None and client.available():
        findings += llm_findings(nl, client)
    return Report(
        project=project,
        components=len(nl.components),
        nets=len(nl.nets),
        findings=_dedupe(findings),
    )


def review_schematic(
    schematic: str | Path, client: OllamaClient | None = None
) -> Report:
    nl = parse_netlist(export_netlist(schematic))
    return review_netlist(nl, project=Path(schematic).stem, client=client)
