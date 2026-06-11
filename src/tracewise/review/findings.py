"""Findings schema and report rendering.

A finding is only as useful as its evidence: every finding names the nets and
references it is about, states which pass produced it (rule id or LLM), and
carries a confidence so downstream consumers (the report, the future Fixer)
can filter. The schema is the contract between passes, benchmarks, and UI.
"""

from __future__ import annotations

import json
from typing import Literal

from pydantic import BaseModel, Field

Severity = Literal["error", "warning", "info"]
Category = Literal[
    "decoupling", "pullup", "power", "floating", "pin-use", "polarity", "topology", "other"
]


class Finding(BaseModel):
    severity: Severity
    category: Category
    title: str = Field(min_length=4)
    detail: str = ""
    nets: list[str] = Field(default_factory=list)
    refs: list[str] = Field(default_factory=list)
    source: str = Field(description="rule:<id> or llm:<model>")
    confidence: float = Field(default=1.0, ge=0.0, le=1.0)
    datasheet_citation: str = ""
    suggested_fix: str = ""


class Report(BaseModel):
    project: str
    components: int
    nets: int
    findings: list[Finding] = Field(default_factory=list)

    def counts(self) -> dict[str, int]:
        out = {"error": 0, "warning": 0, "info": 0}
        for f in self.findings:
            out[f.severity] += 1
        return out

    def to_json(self) -> str:
        return json.dumps(self.model_dump(mode="json"), indent=2, ensure_ascii=False)

    def to_markdown(self) -> str:
        c = self.counts()
        lines = [
            f"# TraceWise review — {self.project}",
            "",
            f"{self.components} components · {self.nets} nets · "
            f"**{c['error']} errors · {c['warning']} warnings · {c['info']} info**",
            "",
        ]
        icons = {"error": "🔴", "warning": "🟡", "info": "🔵"}
        order = {"error": 0, "warning": 1, "info": 2}
        for f in sorted(self.findings, key=lambda f: (order[f.severity], f.category)):
            lines.append(f"## {icons[f.severity]} {f.title}")
            meta = f"`{f.category}` · {f.source} · confidence {f.confidence:.2f}"
            if f.nets:
                meta += " · nets: " + ", ".join(f"`{n}`" for n in f.nets)
            if f.refs:
                meta += " · refs: " + ", ".join(f.refs)
            lines.append(meta)
            if f.detail:
                lines.append("")
                lines.append(f.detail)
            if f.suggested_fix:
                lines.append("")
                lines.append(f"**Suggested fix:** {f.suggested_fix}")
            if f.datasheet_citation:
                lines.append("")
                lines.append(f"*Datasheet: {f.datasheet_citation}*")
            lines.append("")
        if not self.findings:
            lines.append("_No findings._")
            lines.append("")
        return "\n".join(lines)
