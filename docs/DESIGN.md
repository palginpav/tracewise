# TraceWise — Design

*Design date: June 2026. Grounded in a live survey of the AI-PCB landscape, KiCad's extensibility
surface, and the placement/routing state of the art.*

## 1. Problem and positioning

Professional consensus in 2026: no autorouter — classical or AI, including the best-funded
commercial attempts — is trusted for production boards with DDR/PCIe/RF or dense BGAs.
Auto-routers optimize geometry, not physics; return paths, impedance and skew need simulation or
human intent. Meanwhile the *documented* failure mode of classical autorouting is mundane:
engineers skip constraint entry (net classes, diff pairs, length rules, fanout strategy), so
solvers produce naive results.

TraceWise's bet: **LLMs are the right tool for the translation work** — netlist semantics +
datasheets + stack-up → formal constraints, review findings, placement intent — while **classical
solvers keep doing the geometry**. No end-to-end learned router, no sign-off claims.

The niche is empty: no open-source, KiCad-native tool combines schematic review, proposed fixes,
placement, and routing. Existing open projects cover fragments (kicad-happy: schematic checks;
KiCad MCP Server: LLM orchestration of stock tools; Freerouting: maze routing).

## 2. Modules

### M1 — Reviewer
Netlist (via `kicad-cli sch export netlist`) → compressed electrical representation (the
electrical content of a `.kicad_sch` is ~2% of the file; ~50k tokens per 100 components) →
LLM review grounded by **datasheet RAG**: per-part pin-use verification, missing decoupling,
missing pull-ups (I²C!), floating enables/resets, power-rail mistakes, polarity conventions.
Output: structured findings (severity, evidence, datasheet citation). Local model by default
(Ollama), hosted API opt-in. Stated ceiling: structural/connectivity errors — not analog margins,
not SI/PI.

### M2 — Fixer
For mechanically fixable findings (add 100 nF at pin, add pull-up, tie EN): generate a **patch to
the `.kicad_sch` file** (s-expression level), grouped and labeled, with a mandatory
review-and-approve workflow, automatic backups, and diffs. File-level patching is a platform
necessity: KiCad has no live schematic API (eeschema IPC is postponed indefinitely), so robust
file manipulation is a core competency, not a workaround.

### M3 — Placer
Analytical gradient-descent placement (the Cypress/ISPD-2025 class of method — consumer-GPU
feasible, unlike RL) with PCB cost terms: wirelength, decoupling-cap proximity, thermal spread,
edge/mechanical locks, courtyards/keepouts, and **functional-block cohesion** from an LLM
pre-pass that clusters the netlist (power / MCU / analog / connectors). Placements applied into a
live KiCad session via the official IPC API; user locks and re-runs incrementally.

### M4 — Router
LLM constraint generation: net classes (width/clearance from current and function),
differential pairs, length-match groups, impedance-informed widths from the stack-up, via
budgets — emitted as KiCad custom rules (`.kicad_dru`) + Specctra DSN rules. Routing by
**Freerouting through the DSN/SES bridge** (the canonical KiCad pattern), behind a pluggable
engine interface. Iterate loop: route → `kicad-cli pcb drc --format json` → adjust → re-route.
Rule-based BGA/fine-pitch fanout pre-pass (the documented hard case for maze routers).

## 3. Architecture

```
KiCad 10 (GUI)
 ├─ TraceWise plugin (PCM; thin wx panel; async thread + wx.CallAfter)
 │    └─ IPC API (official kicad-python): board ops
 └─ TraceWise engine (separate process; full CLI without KiCad running)
      ├─ schematic: kicad-cli (netlist/ERC) + s-expression file patching
      ├─ LLM: Ollama local | hosted API opt-in; prompts versioned
      ├─ RAG: datasheet corpus (manifest + downloader; PDFs never committed)
      ├─ placer: torch analytical core
      ├─ router bridge: DSN → Freerouting → SES; pluggable
      └─ verify: kicad-cli drc/erc JSON → iterate
```

### Platform constraints designed around (verified June 2026)
- SWIG `pcbnew` bindings deprecated (removal in KiCad 11) → target IPC API + `kicad-cli` only
- No schematic live API in any stable release → file-level patching with format-version checks
- No API to the push-and-shove router → DSN/SES external-engine bridge
- IPC API requires a running GUI in KiCad 9/10 → the engine's CLI mode works on files alone
- DRC results via `kicad-cli` JSON (no structured API); zones must be refilled after import

## 4. Evaluation discipline

- Benchmark suite of pinned open-hardware KiCad boards (2-layer simple → 4-layer MCU+analog)
- Reviewer: seeded-error benchmark (deliberately broken schematics) → precision/recall, with
  clean-schematic cases to measure false positives
- Router: completion %, DRC violations, via count, wirelength — **constrained vs naked
  Freerouting** on identical boards is the headline ablation
- Placer: wirelength + constraint satisfaction vs the boards' human placements
- Scorecards committed per release; negative results reported

## 5. Non-goals

Autonomous sign-off routing · SI/PI guarantees · RF/microwave · replacing engineering judgment.
