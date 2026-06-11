# Build Plan

Versioned scope ladder — each release independently useful. Engineering standards throughout:
CLI-first engine, thin plugin; tests + CI on every push; no PDFs/models/secrets in git;
measured results per release.

## Spike 0 — platform de-risking (first!)

- [x] kiutils round-trip fidelity — **NO-GO** (0/9 clean round-trips on v9 AND v10: board files crash or corrupt, schematics silently drop 8–17% of tokens). Decision: own lossless s-expression layer (docs/spike-0-kiutils.md)
- [x] Demo-file fetcher (scripts/spike_kiutils.py; official KiCad demos, fetched not committed)
- [ ] kicad-cli availability strategy for CI (docker image) and for dev machines without KiCad
- [x] Lossless s-expression editor core (src/tracewise/sexpr.py): CST with verbatim tokens + trivia; byte-identical round-trip proven on all fetched KiCad v9/v10 demo files; surgical insert/set/remove with inferred indentation
- **Exit:** ✅ kiutils no-go decided; sexpr core is the v0.1 prerequisite

## v0.1 — Reviewer

- [ ] Netlist extraction (kicad-cli) + compressed electrical representation (token budget measured)
- [ ] Datasheet RAG corpus for benchmark boards (manifest + downloader pattern)
- [ ] LLM review pass: findings schema (severity, evidence, datasheet citation, confidence)
- [ ] Seeded-error benchmark suite + precision/recall scorecard (incl. clean-schematic FP rate)
- [ ] CLI: `tracewise review <project>` → markdown/JSON report
- **Exit:** measured precision/recall on the seeded benchmark; useful report on a real board

## v0.2 — Constraints + Router bridge

- [ ] Stack-up/board spec input (`tracewise.yaml`)
- [ ] LLM constraint generation → `.kicad_dru` + DSN rules (net classes, diff pairs, length groups)
- [ ] Freerouting bridge (DSN export → route → SES import) + zone refill
- [ ] DRC-iterate loop (kicad-cli JSON)
- [ ] Ablation: constrained vs naked Freerouting on the benchmark suite — the headline table
- **Exit:** measurable routing-quality delta from generated constraints

## v0.3 — Placer

- [ ] Analytical gradient-descent core (torch) with PCB cost terms
- [ ] LLM functional-block clustering pre-pass
- [ ] IPC API application into live KiCad session; lock-and-rerun
- [ ] Placement metrics vs human layouts on benchmark boards
- **Exit:** competitive wirelength + constraint satisfaction on benchmarks

## v0.4 — Fixer

- [ ] Patch generation for mechanical fixes (s-expression edits, grouped + labeled)
- [ ] Approval UI (plugin) + backups + diff log
- [ ] Fix-correctness benchmark (apply → re-run Reviewer → finding resolved, no new ERC errors)
- **Exit:** zero silent edits; measured fix correctness

## v1.0 — Full loop

- [ ] PCM packaging + submission
- [ ] End-to-end benchmark report (review → fix → place → route on suite boards)
- [ ] Docs site: honest scope, results, reproduction commands
