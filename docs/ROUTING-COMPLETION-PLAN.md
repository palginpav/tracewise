# Routing-completion plan — breaking the zuluscsi unconnected floor

Status: active (2026-06-17). Owner: orchestrated feature engineering.

## Objective

Break zuluscsi's unconnected floor (84 DRC unconnected items, deterministic) without
regressing mitayi or pic_programmer. The floor is now attributed at net level, so each
feature targets a specific slice.

| net group | now | target | feature |
|---|---|---|---|
| +3V0 (no pour, 58-pad power) | 56 | ≤15 | F3 high-fanout power |
| GND (pour exists, pads isolated) | 22 | ≤5 | F2 pour-stitching |
| ~{SCSI_DB2}, ~{SCSI_DB4} | 6 | ≤2 | F4 stubborn-net rip-up |

Hard gate per feature: measured improvement on zuluscsi AND zero regression on mitayi +
pic_programmer, all routes deterministic (`taskset -c 0-9`, routes now complete under the
600s cap after the 2026-06-17 A* speedups).

## Background facts (load-bearing)

- Router is deterministic since the A* heuristic + neighbor-expansion vectorization
  (mitayi 290s→134s; zuluscsi completes at ~530s, byte-identical across runs).
- `history_factor=1.0` (negotiated-congestion pricing) is on by default in
  `route_board_engine`.
- GND has an 85-poly pour spanning F&B; `refill_zones` (pcbnew ZONE_FILLER) fills it, but 22
  GND pads are isolated from the fill. +3V0 has NO pour in the source board — it is meant to
  be trace-routed (the human does) and the router completes only 2/58.
- Geometry-extraction fragility: zone `net_name` parses on the source board but returns NONE
  on a pcbnew-resaved board. Any pour feature must read geometry/connectivity robustly.
- Key files: `src/tracewise/route/engine/{kicad,multi,astar,eccf,grid}.py`,
  `src/tracewise/route/bridge.py`. Harness: `scripts/ablation_route.py`.

## Features

### F0 — Robust pour/connectivity extraction (foundation, blocking)
Source of truth for (a) pour-copper geometry and (b) pads unconnected to their pour.
Likely pcbnew connectivity API over s-expr. Deliver: design doc + API spec + test plan.
Chain: architect → developer → tester (fixture: source AND resaved boards).
Accept: returns pour-cell mask + isolated-pad list, identical on both board formats.

### F2 — GND pour-stitching (depends on F0)
Rasterize pour into the routing grid; route a short stub from each isolated pour-net pad to
the nearest pour cell (vectorized heuristic handles the large pour goal set). Emit → refill.
Chain: architect → developer → reviewer → measure.
Accept: zuluscsi GND 22→≤5, no new clearance violations, mitayi no regression.

### F3 — +3V0 high-fanout power (dominant 67%, hardest)
Design question (research first): pour problem (auto-generate a partial +3V0 fill on copper
GND doesn't own) vs routing problem (trunk/star + stubs, route-first ordering). 2-layer area
is contended; likely hybrid.
Chain: researcher → architect → developer → reviewer → measure.
Accept: +3V0 56→≤15, no regression. May land in stages.

### F4 — SCSI stubborn nets (small)
Per-net rip-up escalation / congestion-pricing tuning for the 2 remaining signal nets.
Chain: developer → measure.

## Phasing

```
Phase 0:  architect (F0 design) || researcher (F3 survey)        [read/think only]
Phase 1:  developer + tester (F0)                                [blocks F2]
Phase 2:  F2 chain (GND stitch) || architect (F3 design)
Phase 3:  F3 chain (+3V0)
Phase 4:  F4 + full cross-board validation + ROI scorecard
```

## Measurement protocol (gate)

Per-net DRC-breakdown harness (extends `scripts/ablation_route.py`), run before/after each
feature on all three boards, deterministic. No feature merges unless its target net group
improves and the others hold.

## Risks

- F0 fragility → if the pcbnew API is awkward, F2 slips. Mitigation: F0 is its own gated phase.
- F3 may have a real 2-layer ceiling (pour-less 58-pad net); partial wins acceptable, documented.
- Geometry bugs (the Y-coord-swap class): every geometry change ships with a fixture unit test.

## STATUS UPDATE (2026-06-17): F0 shipped; F2 approach FALSIFIED, reverted; model corrected

- **F0 SHIPPED** (commit ecc9587): pcbnew-based pour geometry extraction (`extract_pours`,
  `rasterize_pour`) — sound, 12 tests, source-vs-resaved invariant holds. KEPT.
- **F2 (stub-stitch) FALSIFIED and reverted.** Two implementation rounds capped at zuluscsi
  GND 22→20 (only 2 pads stitched), mitayi no regression. Root cause found by direct diagnosis:
  the stitch worklist used `unconnected_pads` (pcbnew `IsConnectedOnLayer`), which reports only
  **2** isolated GND pads, while DRC ratsnest reports **22**. The gap: `IsConnectedOnLayer`
  calls a pad "connected" when it touches ANY pour copper — including a DISCONNECTED pour
  island. So the real problem was mis-modelled.
- **CORRECTED PROBLEM MODEL (decisive — the 22 GND ratsnest gaps):**
    - 10 gaps are **zone↔zone at 0.0mm** — the GND pour fragmented into ISLANDS separated by
      hairline clearance; they need island BRIDGING (a via/short bridge at the gap), not pad stubs.
    - 12 gaps are **far** (median 44.9mm, max 93.4mm) — real cross-board GND routing.
    - Almost none are "isolated pad near pour needing a short stub" — F2's entire premise.
- **Consequences for the plan:**
    - F0's `unconnected_pads` is the WRONG signal (IsConnectedOnLayer misses island
      fragmentation). A future feature must use the RATSNEST (DRC unconnected_items) as the
      worklist, not IsConnectedOnLayer.
    - The GND floor (22) really decomposes into: ~10 pour-island bridges + ~12 high-fanout GND
      routes. The +3V0 floor (56) is pour-less high-fanout. So the dominant remaining work is
      TWO new capabilities: (1) pour-island bridging (cheapest next lever — 0mm gaps), and
      (2) high-fanout power/ground net routing (the hard, dominant part). Stub-stitching is NOT
      the lever and is removed.
- **F2' (island bridging) also FALSIFIED — cheaply, by a probe before building.** Hypothesis:
  a GND via at each 0mm zone↔zone gap merges the islands. Probe (place 9 GND vias at the
  zero-gap positions on /tmp/g2_off, refill, re-DRC): GND stayed at 22 — NO change. The 0mm
  gaps are NOT hairline clearance slivers; the GND pour islands are separated by OTHER nets'
  traces crossing the GND region. You cannot bridge across another net's copper (short). The
  islands can only be joined by routing GND copper AROUND the crossing trace (a detour, or
  via→opposite-layer→around→via-back) — i.e. real routing, same as the far gaps.

## CONSOLIDATED CONCLUSION (2026-06-17): the floor is high-fanout routing; no pour-side trick

Three approaches tried and falsified by measurement:
  1. F2 stub-stitch (pad → nearest pour): GND 22→20. Wrong model (ratsnest fragmentation, not
     isolated pads).
  2. F2' island via-bridge (via at 0mm gaps): GND 22→22. Islands split by crossing traces, not
     clearance slivers.
  3. (prior) placement flips/nudges: moved nothing (floor isn't placement).
THE FLOOR IS A HIGH-FANOUT ROUTING PROBLEM. zuluscsi 84 = GND 22 (fragmented, needs GND routed
around obstacles / across board) + +3V0 56 (pour-less 58-pad net, needs routing) + SCSI 6. None
has a cheap pour-side or placement-side fix. The single remaining lever is genuinely routing
these high-fanout power/ground nets through a congested 2-layer board — hard, and possibly at a
real 2-layer ceiling for some connections.

What SHIPPED and stands: the router speedup (−54%, determinism), congestion pricing (default on),
and F0 pour extraction (reusable IF a future high-fanout-routing feature wants pour geometry).
What's CLOSED: stub-stitch, island via-bridge, placement refinement — all measured dead-ends for
this floor. The F3 survey's "synthesize a +3V0 pour" idea remains the one UNTRIED principled
angle, but note it reduces to the same routing problem once the pour fragments (as GND's does).

Process win: probe-before-build on F2' avoided a second wasted module — the F2 lesson applied.

## F3 SHIPPED (2026-06-17): synthesize pours for high-fanout power nets — first floor break

Probe-validated then built (commit 328400e). `synthesize_power_pours` adds a board-outline zone
at LOW priority for each power net (POWER regex, >=8 pads, no existing pour); the GND pour
(bumped to higher priority) wins contested copper, the power net fills the residual and connects
the bulk of its pads via the fill. Integrated as `route_board_engine(synth_power_pours=True)`.
MEASURED (cross-board gate):
  zuluscsi  84 -> 65  (+3V0 56 -> 37, GND 22 held, SCSI 6 held)  errors 108->108  viol 562->562
  mitayi    63 -> 63  (no change: back fully GND-poured, no residual copper for a +3V3 pour)
            viol 82 -> 80  — NO regression.
First measured break of the unconnected floor (zuluscsi −23%) with ZERO new violations. Works
where copper is available (zuluscsi), neutral where GND owns all copper (mitayi's poured back).
Residual zuluscsi 65 = +3V0 37 (fragmentation tail) + GND 22 + SCSI 6 — the hard high-fanout
routing tail remains, but the pour-synthesis lever is real and shipped.
Process: F0 (extract) + F3 (synthesize) are the durable wins from the pour line; F2/F2'
(stub/bridge) stay falsified. Probe-before-build worked a third time.

## F4 (ratsnest short-stub routing) — VALIDATED win but REVERTED on a quality gate (2026-06-17)

Probe-validated: 12/24 short (<6mm) +3V0 ratsnest gaps are routable as stubs, all via the
opposite layer. Built it (DRC-ratsnest worklist — the correct signal F2 lacked — via-enabled,
distance-bounded, fail-safe). MEASURED connectivity win:
  zuluscsi 65 -> 46-47 unconnected (+3V0 37->21/22), mitayi 63 -> 56.  Combined score improved
  on both boards (zuluscsi 433->353, mitayi 393->362).
BUT it introduced 3 net-new `shorting_items` (zuluscsi 31->34). Quality investigation:
  - escape=0 (legality-first, no clearance shaving): shorts UNCHANGED at 34 (so not from
    shaving) — they're grid-quantization shorts (0.1mm grid vs exact DRC) AND pour-interaction
    (the stub changes the fill, creating a short ELSEWHERE in a dense region ~132-135mm, not on
    the stub itself). escape=0 did reduce clearance (23->19) so it was kept while it mattered.
  - Post-emit short-rejection (capture shorts-before, emit, DRC, revert offending stubs):
    FAILED — the 3 shorts can't be attributed to a stub segment (they're pour-interaction, not
    direct overlap), so reversion leaves them. "3 residual new shorts remain after reversion."
DECISION: REVERTED. A feature that introduces unremovable SHORTS does not ship, even though the
project's blunt combined metric (unc*5+err, lumping shorts with clearance) would accept it. A
short is electrically catastrophic; a human engineer would not trade new shorts for connectivity.
The connectivity approach is sound and re-openable IF stubs get EXACT-geometry (sub-grid) clearance
validation or a pour-aware emit that doesn't fragment/bridge the fill into shorts. Deferred.

## FINAL STATE OF THE ROUTING-COMPLETION ARC

Shipped & default-on: router speedups (-54%, determinism), congestion pricing, F0 pour extract,
F3 power-pour synthesis. zuluscsi unconnected 84 -> 65 (clean, zero new violations); mitayi 63 ->
63 (no residual copper, no regression).
Explored & closed with evidence: placement flips/nudges, F2 stub-stitch, F2' island via-bridge,
F4 short-stub routing — each falsified or reverted by measurement, none a cheap win. The residual
zuluscsi 65 (+3V0 37, GND 22, SCSI 6) is genuine high-fanout routing on a congested 2-layer board,
at or near a real ceiling without exact-geometry routing or more layers. Every dead-end is
documented so no future effort repeats them.

## RESEARCH SYNTHESIS — next levers (2026-06-18, two parallel researchers, CONVERGENT)

Docs: docs/research/NEXT-exact-geometry-routing.md, docs/research/NEXT-high-fanout-completion.md.
Both researchers independently reached `recommend_existing` and the SAME #1 lever.

DECISION — staged plan:
- **L1 (ceiling detector, ~0.5d, non-destructive, do FIRST):** BFS reachability on the final grid
  per unconnected pair with cap=infinity → classify each residual as `ROUTER_RECOVERABLE` vs
  `UNROUTABLE_2LAYER`. Gives the HONEST denominator (est. 30-35 of 65 are a true 2-layer ceiling)
  and is a genuine product differentiator (report "this net needs 4 layers"). Sets the real target
  for L2. Plugs into route_board_engine as a post-route report; reuses the grid.
- **L2 (re-open F4 with PER-STUB transactional DRC gate, ~2-3d):** for each short ratsnest gap
  (shortest-first ordering): route stub → emit → refill → run_drc → KEEP only if no net-new short/
  error, else revert THAT stub. Per-stub (not batch) catches pour-interaction shorts by construction
  — the exact failure that got F4 reverted. Target: +3V0 37->~20, GND a few, zuluscsi 65->~48,
  ZERO new shorts. Runtime guard: skip stubs not intersecting a zone outline; bisection fallback.
- **L3 (deeper, later): exact-geometry segment emit** (Shapely Minkowski snap) to kill grid-
  quantization shorts at the source; and **FindIsolatedCopperIslands** for GND island bridging.
- **vNext: true gridless/shape-based router** = the long-term architecture (3-6 mo), behind the
  same route(board,spec)->RouteResult contract. Not now.

Honest reframe: success is no longer "65->0". It is "connect the ROUTER-RECOVERABLE residual
cleanly (no shorts) and HONESTLY REPORT the 2-layer ceiling." L1 quantifies that; L2 delivers it.

## L1 SHIPPED — 2-layer ceiling detector + HONEST DENOMINATOR (2026-06-18, commit 92bc81a)

`classify_unrouted` (src/.../ceiling.py): free-space connected components (8-conn per layer +
via-legal inter-layer edges); each residual ratsnest gap is ROUTER_RECOVERABLE (path exists) vs
UNROUTABLE_2LAYER (no path — needs 4 layers) vs unknown (endpoint buried in copper). Read-only,
route_board_engine `report_ceiling=True`. 18 tests. (Bugs caught in PM review before commit:
net was read from wrong field -> empty by_net; missing `unknown` category -> counts didn't sum.
Both fixed; full accounting now recoverable+unroutable+unknown == total.)

MEASURED on zuluscsi (65 residual):
  recoverable 43 | unroutable_2layer 14 | unknown 8
  +3V0: 25 rec / 10 unroutable / 2 unk   (of 37)
  GND:  16 rec /  2 unroutable / 4 unk   (of 22)
  SCSI: 2 rec / 2 unroutable / 2 unk     (of 6)

STRATEGIC REFRAME: ~43 of 65 are ROUTER-RECOVERABLE (a path physically exists; the router just
didn't commit it under budget/ordering) — far more headroom than the research's 30-35-ceiling
guess. Only 14 are a genuine 2-layer ceiling. So the product story is: connect the recoverable
~43 cleanly + honestly report the ~14 as "needs 4 layers". L2 (per-stub-gated F4 re-open +
ratsnest ordering) targets the recoverable set. Success metric updated: not "65->0" but
"recoverable->0, ceiling reported." Next: L2.

## L2 (per-stub-gated stub) — REVERTED; STUB-COMPLETION LINE CLOSED (2026-06-18)

L2 added a per-stub TRANSACTIONAL gate (emit one stub -> refill -> DRC -> revert that exact stub
if any net-new violation). The violation invariant HELD (zuluscsi shorts 31->31). But unconnected
REGRESSED 65->77. Added an unconnected-monotonicity guard (keep only if unconnected STRICTLY
drops) and re-measured: STILL 65->75. The guard cannot work because the failure is the REFILL
itself: every emit->refill (even for a stub that gets reverted) re-solves the whole power-pour
fill and reshuffles it, drifting connectivity upward globally. The per-stub gate triggers this
on every attempt, so it cannot beat the instability it causes.

ROOT CAUSE (convergent across F2 / F2' / F4 / L2 — the ENTIRE stub-completion line): you cannot
LOCALLY edit a power pour. Each refill_zones re-solves the global fill; small copper edits
fragment it unpredictably. Post-pour stub completion is architecturally unstable. CLOSED.

Consequence: the 43 "router-recoverable" residuals (paths that EXIST per L1) must be addressed
in the MAIN router (route power nets as real traces with better ordering/congestion BEFORE/with
the pour), NOT by post-hoc stubs. That is high-fanout routing — the hard, standing problem.

DURABLE WINS from the whole pour line: F0 (extract), F3 (synthesize power pours, zuluscsi 84->65),
L1 (ceiling detector: 43 recoverable / 14 true-ceiling / 8 unknown). Everything else falsified.

## PIVOT: functional placement grouping (operator lever, research done 2026-06-18)

docs/research/NEXT-functional-placement-grouping.md. The placer groups connected parts only
IMPLICITLY (HPWL); the decoupling term is crude (cap -> FIRST pad on the shared power net). Plan
(recommend_existing, ~2-3d): (A) fix decap->IC assignment (closest supply pad), (B) rule-based
sub-circuit group extraction (crystal+caps, regulator+caps, ESD) via ref+value patterns, (C)
differentiable cluster-centroid attraction term annealed in optimize(). VALIDATE by routing-in-
the-loop on MITAYI (deterministic; zuluscsi floor is pour-class-dominated, placement-insensitive).
This is the upstream lever on congestion — measured, not by placement score (which misleads).
