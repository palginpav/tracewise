"""The Reviewer: deterministic rules + LLM analysis over the compressed netlist.

Two complementary passes, merged into one report:

- ``rules``  — checks that are mechanically decidable from the netlist
  (I²C without pull-ups, power pins without decoupling, floating inputs).
  Precise, free, no model required.
- ``engine`` — the LLM pass for what rules cannot see (wrong component
  choices, off-datasheet usage, topology mistakes), constrained to a
  structured findings schema with mandatory evidence.

Findings carry severity, evidence (nets/refs), and an optional datasheet
citation slot — populated once the RAG layer lands.
"""

from tracewise.review.findings import Finding, Report

__all__ = ["Finding", "Report"]
