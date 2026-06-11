# Build Plan

Versioned scope ladder — each release independently useful. Engineering standards throughout:
CLI-first engine, thin plugin; tests + CI on every push; no PDFs/models/secrets in git;
measured results per release.

## Spike 0 — platform de-risking (first!)

- [x] kiutils round-trip fidelity — **NO-GO** (0/9 clean round-trips on v9 AND v10: board files crash or corrupt, schematics silently drop 8–17% of tokens). Decision: own lossless s-expression layer (docs/spike-0-kiutils.md)
- [x] Demo-file fetcher (scripts/spike_kiutils.py; official KiCad demos, fetched not committed)
- [x] kicad-cli strategy: native or flatpak auto-discovery (find_kicad_cli); flatpak sandbox quirk handled (temp under $HOME, not /tmp); kicad-cli-dependent tests self-skip; CI docker job deferred until Reviewer integration tests exist
- [x] Lossless s-expression editor core (src/tracewise/sexpr.py): CST with verbatim tokens + trivia; byte-identical round-trip proven on all fetched KiCad v9/v10 demo files; surgical insert/set/remove with inferred indentation
- **Exit:** ✅ kiutils no-go decided; sexpr core is the v0.1 prerequisite

## v0.1 — Reviewer

- [x] Netlist extraction (kicad-cli → our own sexpr parser) + compressed representation — measured on a real project: 57k → 1.6k tokens (2.8%)
- [x] Datasheet store (manifest + fetch, PDFs cached outside git) + keyword-window retrieval + per-part LLM verification pass (llm-datasheet source, citation field populated)
- [x] Findings schema + hybrid review: deterministic rules (i2c-pullup, power-decoupling, floating-input) + LLM pass with hallucination guard (evidence must exist in netlist), confidence capped below rules, dedupe
- [x] Seeded-error benchmark: 10 cases (4 clean controls) — rules-only AND rules+LLM both 1.00 recall / 1.00 precision; the LLM pass initially flagged incomplete fixtures (it was right) — fixtures fixed, scorecards committed
- [x] CLI: `python -m tracewise review <sch>` → markdown/JSON; exit code = error count; rules-only fallback when Ollama absent
- **Exit:** measured precision/recall on the seeded benchmark; useful report on a real board

## v0.2 — Constraints + Router bridge

- [x] Board spec input (`tracewise.yaml` → BoardSpec, sane defaults)
- [x] Constraint generation: deterministic classification (power by name+pintype, diff pairs by suffix matching incl. +/-, slow buses) → net classes patched into .kicad_pro + .kicad_dru rules; LLM refinement pass deferred to a later iteration
- [x] Freerouting bridge: DSN export + SES import via pcbnew inside KiCad's runtime (flatpak-aware), Freerouting 2.2.4 JAR auto-fetched/cached, zone refill on import
- [x] DRC scoring via kicad-cli JSON (summary: violations/severity/unconnected); iterate loop = ablation work
- [x] Ablation v2 on 3 real community boards (docs/route-ablation-v2.md): mixed — constraints helped the dense 2-layer (21 vs 27 unconnected), hurt the tight 4-layer; violation columns flagged non-comparable (asymmetric rule sets — fair-scoring fix queued); Freerouting's ceiling dominates → pluggable engines next
- [ ] Fair DRC scoring across arms (identical rule set) + suite growth
- **Exit:** measurable routing-quality delta from generated constraints

## Router strategy note (post-ablation)

Freerouting cannot meet the quality bar (designer-reviewed: dangling stubs, poor completion).
Pluggable engines stay the near-term path (Topola when ready), and **an own router is now an
acknowledged roadmap candidate** (v0.5+): the DSN/SES bridge, sexpr core, and DRC scoring
harness are router-agnostic and carry over. Scope honestly: grid/A* push-and-shove for
2-layer-class boards first, not a universal router. Decision deferred until after the placer —
better placement may move the routability needle more cheaply.

## v0.3 — Placer

- [x] Analytical gradient-descent core (torch, CPU): smooth-HPWL (logsumexp, annealed) + soft courtyard overlap (annealed) + boundary + decoupling-proximity terms; locked parts respected. Live: mitayi HPWL −23.7% in 7.5 s — with residual overlap honestly flagged (legalization pass = next)
- [ ] LLM functional-block clustering pre-pass
- [x] Apply path via pcbnew script (consistent with bridge); `tracewise place [--apply] [--lock refs]`
- [ ] Legalization pass (overlap-free final positions) — required before placement claims are comparable to human layouts
- [ ] Placement metrics vs human layouts on benchmark boards
- **Exit:** competitive wirelength + constraint satisfaction on benchmarks

## Router engine progress (R-milestones, docs/ROUTER-DESIGN.md)

- [x] R0: grid + A* + simplification (128ms corner-to-corner on 600k-node reference grid)
- [x] R1: multi-net, power-first ordering, bounded rip-up (80/80 synthetic nets in 29s)
- [~] R2: real boards end-to-end — pad extraction, own-pad carving, KiCad-10 name-based
  net emission via sexpr. First live mitayi run: **60/61 nets routed, unconnected 24 vs
  Freerouting's 89 (3.7× better completion)**. Open: via geometry in grid (vias modeled as
  points → ~800 hole/short/mask violations), dangling-via emit bug (68), zones unmodeled
  (→ 510 clearance hits in pours — that's R3 by design)
- [x] R2.1: via discs (all layers, full radius), via-barrel exclusion from goal trees,
  **counting occupancy** (block +1/unblock −1; boolean rip-up was erasing overlapped
  obstacles → route-through-pad shorts AND artificially easy routing), project-true
  geometry (track/clearance/via from .kicad_pro). Shorts 200→2, hole_to_hole 199→2,
  tracks_crossing 44→0. Honest completion reset: 107 unconnected on mitayi — the earlier
  24 was partly the unmark bug routing through erased obstacles. Remaining wall:
  castellated edge pads + unmodeled pours (R3) + escape routing.
- [~] R3: **zone refill stage landed — the biggest single win: 412→72 violations**
  (pours are not obstacles; they re-pour around tracks; stale fills were the phantom
  clearance class). Fine-pitch experiment (0.05mm): completion unchanged (~96 unconnected,
  castellated pads are sealed at any resolution), DRC worse (stair artifacts) — 0.1mm stays
  default, finding recorded. Escape allowance landed (two-tier grid: hard copper never
  crossable, clearance halos passable within ~1.2mm of endpoints at +2.0/cell penalty):
  unconnected 97→93 — modest; the castellated holdouts fail differently (clamped edge-pad
  centers likely landing inside neighbor hard copper — per-pad geometry investigation next).
  INVESTIGATION RESULT: disc pad model was the wall — castellated pads (1.6×3.2mm rects)
  modeled as r=1.6mm discs overlapped each other at 2.54mm pitch = phantom copper sealing the
  edge. Pads are now true rectangles (hard = exact rect, halo = inflated): **93→72 unconnected,
  remaining failures are inner pads.** Visual render check (kicad-cli pcb render + F.Cu PDF):
  routing fans out properly, pour refills clean. CLEARANCE TUNING (4 hypotheses, measured):
  directional halo rounding (kept — correct), geometric escape window (cost-based closed after
  ~4 cells, stranding QFN pads), no-corner-cutting rule (45° segments clipped blocked corners),
  via inflation must include the approaching track's halfwidth (the repeating 0.125mm class) +
  via placement ring (barrel outsticks track copper). Clearance 70→33. Known limit recorded:
  0.5mm-pitch QFN escapes shave clearance where humans neck track width — width-necking is the
  future fix. Pareto today: 72 unconn/107 err ↔ 80 unconn/58 err. R4 done (see
  route-ablation-r4.md). Snap-to-pad landed (terminal nodes emit exact pad coords) —
  correct but the dangling class survived unchanged (15): the gaps are TREE JUNCTIONS
  (branches meeting earlier paths), not pad terminals — REVISED by instrumented forensics:
  junction hypothesis dead too. All 15 dangling are SHORT B.Cu FRAGMENTS (0.28–2.3mm,
  2–4 diagonal cells) at final pad approaches (d 0.26–0.88mm from own F.Cu-only SMD pads):
  the terminal sequence B.Cu-run → via → F.Cu-run → snapped-pad-end loses connectivity in
  emit (run filtering / via placement / snap interplay). NUMERIC WALK FOUND IT: the
  degenerate-run filter amputated terminal vias-in-pad (B.Cu approach → via → F.Cu pad lost
  its via). Fixed: vias at every layer transition of the unfiltered run list, suppressed at
  through-pads (their barrel already spans layers), deduped per net. **Result: dangling 0,
  co-located 0, unconnected 63 (best; FR=89). Engine now beats Freerouting on completion by
  ~30% on the design-scope board.** Width-necking landed: A* tags halo-traversing nodes,
  emit necks those segments to project-min width (no-op where track==min, pays where
  headroom exists); escape-penalty sweep 2/4/6 → knee at 4.0 (63 unconn + clearance 44→34).
  Engine config frozen for this round: 82 err / 63 unconn / 0 dangling on mitayi (FR: 4/89/0).
  Remaining: via-near-hole spacing (~24), fine-pitch residual shaves. R4 re-run next.

## v0.4 — Fixer

- [ ] Patch generation for mechanical fixes (s-expression edits, grouped + labeled)
- [ ] Approval UI (plugin) + backups + diff log
- [ ] Fix-correctness benchmark (apply → re-run Reviewer → finding resolved, no new ERC errors)
- **Exit:** zero silent edits; measured fix correctness

## v1.0 — Full loop

- [ ] PCM packaging + submission
- [ ] End-to-end benchmark report (review → fix → place → route on suite boards)
- [ ] Docs site: honest scope, results, reproduction commands
