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

## Dev-machine note (2026-06-12)

A cluster of mystery crashes (phantom TypeError, "Fatal Python error: Executing a
cache", NULL-ip segfaults) was traced to a degraded CPU core on the dev machine
(all faults on the same core; clean on others; temps normal — Raptor-Lake-class
degradation signature). Workaround: `taskset -c 0-9` for long runs. None of the
affected runs' committed results are suspect (all verified results re-ran clean).

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
- [x] **ECCF full funnel in `tracewise auto`: 56 unconnected — first result below the
  human-placement plateau (63)**. T2 screens candidates (rotations + trust-region nudges
  of small parts on stubborn nets), top-3 verified by T3 pseudo-route (pour-class nets
  excluded — zone-connected; escape-aware), only dual-endorsed fixes applied, router
  judges, keep-best protects. Two T3-verified moves over 5 rounds → 63→61→56.
  Note: errors 88 at best state (vs 81) — score is lexicographic; error trend to watch.
  Deep run (10 iters, 12 cands, 8-dir nudges): converged at 56 — no further dual-endorsed
  fixes exist in the single-small-part move space. 3-lever expansion (multi-part combos,
  3mm tier, error-site candidates; data-level T2 scoring — extract once, patch pads in
  memory): best 63, NOT 56 — combos crowded the T3 quota out of the proven singles and
  the router rejected their predicted benefit (T3's no-rip-up blind spot on coordinated
  moves). Fix queued: separate T3 quotas (top-3 singles + top-1 combo). Candidate-mix
  sensitivity is now a measured property of the funnel.
- [x] Quota split (top-3 singles + top-1 combo) shipped — combos no longer crowd singles.
- [~] "Center of the storm" arm (layer-flip passives in the densest congestion cluster):
  built, T3-gated, stall-triggered (fires only when the normal arm reaches its floor, per
  the operator's spec). MEASURED INERT ON MITAYI: T2 rejects all flips (+15 delta — a
  flipped pad escapes WORSE because the back side is a GND pour), and T3 confirms no fail
  reduction (76 held; one flip shaved 53/763k cost = noise). Mechanism: the relief only
  works when the OTHER side has free routing area; mitayi routes single-sided over a back
  ground plane, so a flip relocates a pad INTO the pour, not out of congestion. Sound idea,
  wrong board — value pending a design with free back-side area (R4 cross-board test).
  Design fix banked regardless: flips bypass the T2 gate (T2 can't see corridor-freeing
  externalities; only T3 can) — correct for any board where they do help.
- [x] **Storm-flip VALIDATED on zuluscsi** (the board the back-area probe predicted: 88%
  free back, balanced zones, 2-layer). T3-probe: flip R125 (a resistor — the called-out
  class) reduced T3 fail 10->9 and cost ~10% (100135->90414), while T2 scored it +10.6
  (worse) — the externality blind spot, proving the T2-bypass design. mitayi (poured both
  sides) inert, zuluscsi (free back) wins: the relief works exactly where topology allows,
  as predicted. Cross-board precondition probe (back-side free area) is the cheap predictor
  of where the arm pays.
- [x] **Layer-aware placer SHIPPED** (PIPELINE-DESIGN build-order steps 1-2): extract carries
  side (fp.IsFlipped); overlap_penalty masks opposite-side pairs; legalize_tetris collides
  per-side. The placer was wrongly separating front/back parts at the same xy — the
  foundation the storm-flip arm needed. Backward-compatible (side=None -> all front).
- [x] **Robust pipeline hardening — six fixes the cross-board (zuluscsi) validation forced**,
  each exposing the next: (1) per-side overlap/legalize; (2) wall-clock cap on route_all;
  (3) bounded single route (expansion ceiling 2M->600k + 45s/route — a single unreachable
  net had overshot by 16 min); (4) explore-from-best, not a mutable baseline (a T3-good but
  globally-catastrophic move had poisoned the search: zuluscsi 46->230); (5) priority/err
  snapshotted with best (rejected iters no longer pollute route ordering); (6) explore from
  a STRIPPED placement + combined keep-best score unconnected*5+errors (re-routing the
  routed board piled copper -> errors 82->990; lexicographic had accepted 49/655 over
  56/88). HONEST FINDING: under the sound pipeline mitayi's best is ~63-64 (stable, errors
  bounded) — the earlier 56 was partly an artifact of unsound accept logic + gameable
  lexicographic score. mitayi is hard for the arm (poured back, flips inert); zuluscsi (free
  back, validated flip) is where it should pay. CONFIRMED STABLE: after two more fixes
  (7: constant rip-up budget — escalation routed each iter differently; 8: snapshot the
  priority that PRODUCED best, not the post-boost state — the loop couldn't reproduce its
  own best), zuluscsi holds 46/46/46 reproducibly, no wandering. arm-1 priority boosting
  is board-dependent (helps mitayi, hurts zuluscsi) and now effectively neutralized;
  improvement rests on the T3-verified placement/flip arm under keep-best. Engine
  baselines: mitayi 63, zuluscsi 46 (was 115 at R4). UNCONNECTED_WEIGHT=5 tunable. The
  pipeline is now correct/bounded/stable/reproducible — the real deliverable.

## Improving zuluscsi below 46 — ROUTER-THROUGHPUT-BOUND (2026-06-14)

Investigated whether placement/flips can beat zuluscsi's 46. Root cause: zuluscsi's grid is
901x1001x2 = 1.8M nodes (large board at 0.1mm pitch). A single full route at the 600s
route_all budget can only do so much in pure-Python A*; the 46-48 is what the router reaches
in the time budget, NOT a placement limit. Evidence: same board routes 48 (2M cap) vs 68
(600k cap) — both hitting 603s. So the FIXED 600k expansion cap (tuned for mitayi's 428k
grid) was REGRESSING zuluscsi; fixed via a grid-proportional cap (~2x node count; 45s/route
wall-clock is the real runaway guard). Bbox-search-bound shortcut tried and REVERTED — it
made dense boards worse (routes needing detours fail -> rip-up thrash -> 704s/84 unconnected).
NEXT LEVER (substantial): a genuinely faster router — coarsen-then-refine (0.2mm pass then
0.1mm, ~4x fewer cells), numpy-vectorized wavefront (eccf's build_field is the template), or
Rust. Placement tricks cannot beat a time-truncated route; speed is the gate.

UPDATE (timeout experiment, operator-suggested): raising route_all budget 600s->1800s DID
route more — unconnected 48->27 (confirms time-bound) — BUT errors ballooned 105->290, so the
combined score (unc*5+err) got WORSE (345->425). More time = the router forces more
connections through clearance violations (escape-shaving + rip-up). Coarser pitch (0.2/0.15mm)
also worse (123/103 — misses sub-pitch gaps). CONCLUSION: the real bottleneck is routing
QUALITY, not speed or timeout — the engine completes nets by shaving clearance rather than
finding legal paths; every knob (time, pitch, placement) hits this wall. NEXT LEVER (deep,
fresh session): legality-first routing — reduce escape-shaving reliance / make rip-up prefer
legal detours over clearance violations. No current knob beats the 48/105 balance net-net.

ERROR-BREAKDOWN DIAGNOSIS (2026-06-14): escape on/off IDENTICAL (60/103) — escape-shaving is
NOT the cause. zuluscsi's errors are a LONG TAIL, no single fixable bug: items_not_allowed 35
(router routes through KEEPOUT zones it does not model), solder_mask_bridge 21, shorting 20,
clearance/hole 22, footprint_type_mismatch+courtyards_overlap 5 (BOARD-INHERENT, not us),
tracks_crossing only on heavily-routed states (diagonal 45deg X-crossings between adjacent
cells). CONCRETE NEXT TASKS (each bounded): (1) keepout-zone awareness — parse keepout
polygons, mark as grid obstacles (~kills 35 items_not_allowed); (2) diagonal-crossing
prevention — edge-occupancy so two nets' 45deg segments can't cross a shared corner;
(3) legality-first rip-up cost; (4) faster router for completion. zuluscsi is hard in several
INDEPENDENT ways; no quick win.

KEEPOUT-AWARENESS SHIPPED (task 1): router parses keepout zones (extract_keepouts) and marks
their polygons as grid obstacles (Grid.block_polygon ray-cast fill; fixed a swapped Y-coord
unpack bug). zuluscsi: items_not_allowed 35->3 (3 boundary tracks remain — inflation cleanup
pending), errors 103->64, unconnected 60->81. The old 60 was CHEATING (copper through a
forbidden zone); respecting the keepout removes invalid solutions, so the combined-score
"worsening" (403->469) is an objective-weight artifact — correctness improved.

HEADLINE NEXT BUILD (from docs/FORMULATION.md — operator's formalize-it idea): the router is
"negotiated congestion MINUS the negotiation" — capacity is a HARD WALL, so when stuck it
forces connections through violations (the signature of every measured wall). Build a
PathFinder negotiated-congestion loop around the existing A* (price overuse, iterate) + the
diagonal X-crossing edge resource. Spike proved it: forbid-wall -> 3 shorts, PathFinder -> 0
in 3 iters. The principled path below 46, reusing the A* almost verbatim.
- [x] PathFinder router BUILT (src/.../pathfinder.py) + wired into route_board_engine
  (engine="pathfinder"). Synthetic tests pass: prices contention, splits two nets across two
  gaps to zero overlap, neg-congestion mechanism sound. Obstacle model reworked to mirror the
  rip-up router's hard/halo split (only grid.hard forbidden; fixed-pad clearance halos
  enterable at a constant escape premium) — without it every pad sealed by neighbour clearance
  was "no path" (first mitayi run: 0/61, 18 no-path).
- [!] MEASURED ON MITAYI — NEGATIVE: with the escape model the no-path class vanishes (0), but
  negotiation does NOT converge. 1/61 "ok", 60 "congested" after 24 iters / **1584s (26 min)**.
  Two verdicts: (a) the board is congestion-limited — a conflict-free 2-layer embedding of a
  dense single-sided layout may not exist, so "everyone vacates contested cells" has nowhere to
  vacate to; (b) per-iteration cost (full A* reroute of 60 nets on a ~1.8M-cell grid, no time
  cap) makes even an eventual convergence impractical (~hours). And PathFinder only EMITS
  clearance-legal nets, so its best effort lays an almost-empty board (8 segs) vs rip-up's
  60/61 @ ~82 DRC errors. Apples-to-oranges by construction: rip-up lays copper + reports
  violations; PathFinder discards anything it can't make legal.
  CONCLUSION: PathFinder as a drop-in zero-overlap router does NOT beat rip-up here. The
  negotiated-congestion PRICING is still worth salvaging as a heuristic INSIDE rip-up (order
  victims / bias A* by priced contention) rather than as a hard acceptance gate. Router-as-
  router parked; pricing-as-heuristic is the open lever.
- [x] PRICING SALVAGED INTO RIP-UP (history_factor): each rip-up deposits congestion history on
  the victim's cells; the A* step cost is scaled by (1 + history_factor*history[cell]) so later
  routes detour around chronically-contested regions. Default 0 = original pure rip-up.
  MEASURED ON MITAYI (route+DRC, taskset 0-9):
    hf=0.0  routed 35/61  unconn 63  viol 86  vias 74  len 608   combined 401
    hf=0.5  routed 38/61  unconn 62  viol 89  vias 55  len 517   combined 399
    hf=1.0  routed 39/61  unconn 63  viol 83  vias 58  len 572   combined 398
  READ: pricing lays MORE copper, CLEANER — routed nets 35→39 (+11%), vias 74→58 (−22%),
  shorter length, viol −3 at hf=1.0. But combined score barely moves (401→398, <1%) because the
  UNCONNECTED FLOOR is flat (63→62→63): those nets are blocked by genuine placement congestion,
  not router thrash. Confirms the standing diagnosis — mitayi is placement-limited; routing-side
  levers improve route QUALITY but cannot break the unconnected floor. hf=1.0 is the best
  point (most nets, fewest violations). Cross-board (zuluscsi) validation before defaulting on.
  CROSS-VALIDATED ON ZULUSCSI (the denser 1.8M-cell board) — clearer win:
    hf=0.0  routed 89/116  unconn 80  viol 524  vias 170  len 2382  combined 924
    hf=1.0  routed 95/116  unconn 64  viol 539  vias 224  len 2893  combined 859
  unconnected 80→64 (−20%), combined 924→859 (−7%), +6 nets. Cost: more vias/length (detours
  use layer changes to reach legal corridors). Improves on BOTH boards, regresses neither →
  history_factor DEFAULT FLIPPED TO 1.0 in route_board_engine (flows to the auto loop). The
  unconnected floor still ultimately placement-bound; this is the routing-side ceiling raised.
- [ ] via-sweep hang is now FIXED by (3) above (bounded route). Note kept for history.
- [ ] Global via_cost tuning negative on mitayi (cheaper vias -> early nets sprawl the back,
  starve later); targeted/per-net cheap-via for stubborn nets is the open alternative.
- [~] ECCF integration round 1 (superseded): T2-only candidate screening in the auto loop —
  measured insufficient (moves T2 approved still cost pad-completion elsewhere, 63 held
  by rollback; errors improved 82→59). Consistent with the funnel design: T2 is the
  SCREEN; the VERIFY stage (T3 pseudo-route over affected nets) is required to price
  displaced congestion before applying — that's the documented next step, not a new idea.
- [~] **Refinement loop (`tracewise auto`)** toward the 0/0 goal: iterate route rounds,
  failed nets gain ordering priority + escalating rip-up, keep-best-with-rollback from
  pristine each round. First measurement (mitayi, 5 iters): routed nets 35→40 but pad-level
  unconnected churned 63→64–69 → rollback kept 63. FINDING: ordering feedback alone is
  zero-sum at this density — corridors gained by boosted nets are lost by others. The
  binding constraint is PLACEMENT (operator's hypothesis, now measured). Arm 2 v1
  (re-place all failed-net components, weighted HPWL) measured NEGATIVE: moved 28-30
  parts/round incl. the MCU → 86-96 unconn, 270+ errors (overlap chaos); rollback held 63.
  Arm 2 v2 requirements from the wreckage: never move high-pin parts, displacement trust
  region (~2mm), one stubborn net at a time, and the placer itself needs congestion
  awareness before it can be trusted to nudge. Congestion term added (differentiable
  pin-density splat, annealed) — measured NO routability gain (101 unconn, same as
  HPWL-only; overlap 217 vs human 130). CONCLUSION across 3 placement experiments:
  residual overlap is the gate — overlapping courtyards block routing corridors directly.
  Dependency chain revised: scanline/Tetris legalizer FIRST, then congestion, then arm 2.
  Tetris legalizer landed (largest-first ring-search snap, locked-aware): routability
  101→89 unconn (+12 nets). Overlap floor hit: ~147 vs human 130 is irreducible RECT-BBOX
  overlap (nested/rotated parts, 101% density board) — next gap is ROTATION support
  (placer treats orientation as fixed; humans rotate to pack). Gap to human: 26 nets.
  Rotation v1 (box-fit-only 90° in tetris, center-anchored parts): NEGATIVE — 97 unconn
  vs 89; rotating on fit alone scrambles pad axes and invalidates optimized wirelength.
  Gated off by default (optimize(rotate=True) to enable). v2 (HPWL-veto with verified
  pad transform) measured IDENTICAL (97): every rotation is locally wirelength-neutral —
  the damage is ESCAPE DIRECTION (rotated pads face different corridors), invisible to any
  local model. Conclusion: rotation needs routability-aware scoring = router-in-the-loop
  (route trial per orientation), parked. Placer ladder stands at 89; remaining levers:
  arm-2 trust-region nudges, and accepting the human-placement use case (TraceWise routes
  EXISTING placements — its strongest mode anyway).

## v0.4 — Fixer

- [ ] Patch generation for mechanical fixes (s-expression edits, grouped + labeled)
- [ ] Approval UI (plugin) + backups + diff log
- [ ] Fix-correctness benchmark (apply → re-run Reviewer → finding resolved, no new ERC errors)
- **Exit:** zero silent edits; measured fix correctness

## v1.0 — Full loop

- [ ] PCM packaging + submission
- [ ] End-to-end benchmark report (review → fix → place → route on suite boards)
- [ ] Docs site: honest scope, results, reproduction commands
