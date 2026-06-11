# TraceWise

**AI-assisted place & route for KiCad** — schematic review with datasheet grounding, constraint
generation for autorouting, and analytical placement. Open source, local-first.

> Status: **v0.1 — the Reviewer works.** `python -m tracewise review board.kicad_sch` produces
> a findings report from deterministic rules + a local-LLM pass + per-part datasheet checks.
> Seeded-error benchmark: **1.00 recall / 1.00 precision** in both modes
> ([scorecards](docs/review-benchmark.md)). Design: [docs/DESIGN.md](docs/DESIGN.md) ·
> roadmap: [docs/PLAN.md](docs/PLAN.md) · why not kiutils: [docs/spike-0-kiutils.md](docs/spike-0-kiutils.md)

## The idea

Autorouters disappoint not because the solvers are weak, but because nobody feeds them the
constraints that make routing meaningful — net classes, differential pairs, length rules, fanout
strategy. Schematic review tools check syntax, not electronics. TraceWise applies LLMs exactly
where they help:

- **Reviewer** — reads your netlist, cross-checks parts against their datasheets (RAG), and flags
  real mistakes: missing decoupling, absent pull-ups, floating enables, off-datasheet pin usage —
  every finding cited to the datasheet
- **Fixer** — proposes mechanical corrections as reviewable patches to your schematic; you approve
  each one, nothing is ever edited silently
- **Placer** — analytical (gradient-descent) placement with electronics-aware cost terms:
  decoupling proximity, thermal spread, functional-block cohesion
- **Router** — generates formal constraints (`.kicad_dru`, Specctra rules) from netlist semantics
  and your stack-up, drives Freerouting through the proven DSN/SES bridge, and iterates against DRC

**What TraceWise does not pretend:** no autorouter in 2026 — commercial or otherwise — produces
sign-off-grade routing for high-speed designs, and neither does this. TraceWise makes the boring
80% disappear and shows its work on the rest. Signal-integrity judgment stays with you.

## Honest scope

Targets DRC-clean results on ~2–4 layer, sub-50 MHz-class digital and mixed-signal boards.
High-speed designs get constraint *suggestions*, not autonomous routing. Findings ship with
datasheet citations; metrics ship with every release (seeded-error benchmarks for the Reviewer,
constrained-vs-naked Freerouting ablations for the Router).

## Architecture (short)

A standalone Python engine (CLI-first) + a thin KiCad plugin (PCM). Schematic access via
`kicad-cli` + file-level parsing (KiCad has no live schematic API); board operations via the
official IPC API; routing via the Freerouting DSN/SES bridge; LLM runs locally (Ollama) with an
optional hosted-API mode. Details and platform constraints: [docs/DESIGN.md](docs/DESIGN.md).

## License

[Apache-2.0](LICENSE). KiCad demo files used in tests are fetched, not redistributed.
