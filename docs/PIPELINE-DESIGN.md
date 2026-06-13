# Layer-Aware Place + Route Pipeline

*Design + runnable spike, June 2026. Closes the LAYER-BLIND gap in the placer
named here for the first time: the gradient placer ([place/core.py](../src/tracewise/place/core.py))
and Tetris legalizer carry no per-footprint copper side, so overlap and
legalization treat both sides as one 2D plane. This document specifies the
robust fix — per-side occupancy plus side assignment as a first-class placement
DECISION VARIABLE, co-optimized with (x,y)/rotation and scored by the validated
ECCF signal — and is grounded in every measured finding in
[PLAN.md](PLAN.md) and [PLACE-ROUTE-COUPLING.md](PLACE-ROUTE-COUPLING.md).
Prototype: [scripts/spike_layeraware.py](../scripts/spike_layeraware.py), output
reproduced verbatim in §8.*

## 1. The gap, stated precisely

The router half of the codebase is already layer-aware: `extract_pads`
([route/engine/kicad.py](../src/tracewise/route/engine/kicad.py)) reads
`p.GetLayerSet().Contains(F_Cu / B_Cu)` and carries `front`/`back` per pad; the
`Grid` ([route/engine/grid.py](../src/tracewise/route/engine/grid.py)) keeps two
occupancy planes (`cells[0]=F.Cu`, `cells[1]=B.Cu`); `patch_data` and `apply`
already flip pads to the opposite side. **The placement half is not.**
Confirmed in source:

| Component | Layer-blindness | Consequence (all confirmed) |
|---|---|---|
| [`place/extract.py`](../src/tracewise/place/extract.py) | the extract dict has `x,y,w,h,cx,cy,locked,rot,pads` — **no `side`** | the placer cannot know which copper side a part sits on |
| `place/core.py overlap_penalty` | one rectangle plane; `same_side` mask absent | a FRONT part and a BACK part at the same (x,y) score the full bounding-box area as overlap — they do not collide |
| `place/core.py legalize_tetris` | one `placed` rect list | a front part is pushed off a back part it never touches; back-side components are never correctly rearranged |
| `optimize()` move space | `{dx, dy, rot90}` only | side is **not** a decision variable; the placer can never CHOOSE a part's side |

The storm-flip arm ([route/engine/auto.py](../src/tracewise/route/engine/auto.py),
`eccf_candidates(include_flips=True)`) is the *only* place a side change is
modeled today, and it lives entirely in the router-side refinement loop: it
flips a passive on the board and lets T3 + the router judge. **Placement itself
has never represented a side change.** That is the gap this design closes.

## 2. The measured constraints this design must honor

Every line below is a finding already paid for in PLAN.md / the coupling doc;
the design is built around them, not against them.

| # | Finding | Design obligation |
|---|---|---|
| 1 | Storm-flip VALIDATED on zuluscsi (88% free back, balanced zones; R125 flip cut T3 fail 10→9 while T2 scored it +10.6 — worse). INERT on mitayi (poured both sides). | Side moves must be **gated by a free-back-area probe**; the externality is visible to **T3 only**, never T2. |
| 2 | Global via_cost tuning NEGATIVE on mitayi (cheap vias let early nets sprawl). | No global via_cost change. Side preference is **per-part / targeted**, priced by T3. |
| 3 | Best mitayi = 56 unconnected (human routes to 63; ladder 101→89→63→56). T2-only insufficient; T2+T3 funnel broke the plateau; candidate-mix sensitivity is real (combos crowded singles 56→63; fixed with quota split). | Side joins the existing **screen→verify→judge funnel under a quota**, not a new arm with its own budget. |
| 4 | Router models 2 layers; rip-up bounded; keep-best/rollback protects every move. | Every side move is reversible and judged by the full route; nothing bypasses keep-best. |
| 5 | Differentiable congestion gave NO routability gain; rotation v1/v2 negative (escape direction is invisible to local models). | Side assignment is **discrete**, scored by ECCF (the only signal that saw escape direction on real boards: V1 100%, V2 ρ=0.579), not by a smooth placer term. |
| 6 | Arm-2 v1 wreckage: never move high-pin parts; trust region ~2mm; one stubborn net at a time; legalize first. | Side moves inherit the same discipline; flippable set = passives / low-pin SMD only. |

## 3. Layer-aware placement data model

### 3.1 Carry side through extraction — `place/extract.py`

`fp.IsFlipped()` (true ⇒ the footprint is on B.Cu) is the one fact missing.
Add it to the extract script and the dict:

```python
# in EXTRACT_SCRIPT, per footprint:
"side": 1 if fp.IsFlipped() else 0,   # 0 = F.Cu (front), 1 = B.Cu (back)
```

This is purely additive; the router-side data model already encodes side per
pad, so the two halves finally agree. `apply_positions` already accepts a
4th move element `flip` and calls `fp.Flip(pos, False)` — **no apply change is
needed**; the flip flag flows straight through.

### 3.2 Two occupancy planes — `place/core.py`

`PlaceProblem` gains one tensor:

```python
side: torch.Tensor      # [N] long, 0=front 1=back (the decision variable's state)
```

**Overlap, per side.** The pairwise intersection area is masked to zero for
parts on opposite copper sides — the single conceptual change, proven in §8
Claim 1:

```python
same_side = (side[:, None] == side[None, :]).double()
return (area * same_side).sum() / 2
```

This is geometrically exact: two parts only contend for the same courtyard if
they are on the same side. (Through-hole parts occupy both sides; §6 handles
them as a `both` flag that contends with either side — a small extension of the
same mask, deferred until a benchmark needs it; today's reference boards are
SMD-dominant.)

**Legalization, per side.** `legalize_tetris` keeps **two** `placed` rect lists
indexed by side; a part legalizes only against the list for its current side.
Locked parts seed the list for their own side. The ring search and largest-first
order are unchanged. Result: a back part is never pushed by a front obstacle it
does not touch — back-side components are rearranged *correctly* for the first
time.

The gradient terms `smooth_hpwl`, `boundary`, `decap`, `congestion` are
side-agnostic by nature (HPWL and board fit do not depend on copper side; decap
proximity does not either) and need **no change**. Congestion, if used, splits
into two side-planes for parity with the router — the §8 metric demonstrates the
two-plane form.

## 4. Side assignment as a decision variable

Side becomes the fourth discrete move in a **single, principled move space** —
the antidote to bolt-on-arm sprawl:

```
move(part) ∈ {dx, dy, rot90, side}
```

All four are scored by the **same** ECCF funnel (T2 screen → T3 verify → router
judge → keep-best/rollback). This unifies three things that are currently
separate code paths — rotation v3 (planned in the coupling doc §5), arm-2
trust-region nudges, and the stall-gated storm-flip arm — into one candidate
generator with one scoring contract.

### 4.1 Why side is scored by T3, never T2 (measured)

A flip's benefit is an **externality**: it frees the *old* side's corridor for
OTHER nets while it occupies the new side. T2 scores a part's OWN escape only,
so it prices a flip as *worse* (the validated zuluscsi datum: T2 +10.6 while the
flip actually helped; the mitayi datum: T2 +15 rejecting all flips). **T3 is the
only tier that sees the externality** (it pseudo-routes the neighborhood net set
on the candidate grid with shared reservations). Therefore:

- `{dx, dy, rot90}` candidates: **T2 screens**, T3 verifies the shortlist
  (cheap; thousands of candidates need the 28 ms screen).
- `{side}` candidates: **bypass T2**, go straight to T3 (few candidates — only
  flippable passives in the storm cluster — so the screen buys nothing and
  would actively mislead). This is exactly the existing `flips` path in
  `auto.py`, now recognized as the general rule for any externality-dominated
  move, not a special case.

### 4.2 Where the side move sits in the loop

Inside the existing `auto.py` refinement loop, for stubborn nets:

1. **Precondition probe (gate).** Compute back-side free fraction near the storm
   cluster. If below threshold (the mitayi case), **emit no side candidates** —
   the relief cannot exist there (constraint #1; the storm arm was measured
   inert). This is a ~ms geometry probe (§8 demonstrates it as a cross-board
   predictor).
2. **Generate candidates** for the storm cluster's smallest/lowest-pin parts:
   `{dx, dy}` on a ±2 mm ring, `rot90`, and — if the probe passed — `{side}`.
3. **Screen** (`{dx,dy,rot90}` via T2) under the quota split that already exists
   (top-3 singles + top-1 combo); **side candidates skip the screen**.
4. **Verify** the shortlist plus the side candidates with `t3_verify` against
   the stubborn net set (constraint #3: this funnel is what broke the 63
   plateau to 56).
5. **Apply the single best T3-verified move**, route the full board, and let
   **keep-best/rollback** be the final judge (constraint #4). One part per
   round, so fields stay cheap to rebuild (coupling doc §6).

The storm-flip arm is **not removed — it is generalized**: its candidate
generator (`rank_by_storm`) and its T2-bypass become the `{side}` branch of the
unified move space, under the same stall gate it already has. No new arm, no new
budget — the discipline of constraint #3 is preserved.

## 5. End-to-end pipeline and control flow

```
                   ┌─────────────────────────────────────────────┐
 board (placed) ─▶ │  EXTRACT  (place/extract.py)                 │
                   │   per-fp: x,y,w,h,coff,locked,rot, SIDE      │  ← new field
                   └─────────────────────────────────────────────┘
                                      │
                   ┌─────────────────────────────────────────────┐
                   │  PLACE  (place/core.py optimize)             │
                   │   gradient: HPWL+boundary+decap (side-blind) │
                   │   overlap: PER-SIDE mask                     │  ← fix
                   └─────────────────────────────────────────────┘
                                      │
                   ┌─────────────────────────────────────────────┐
                   │  LEGALIZE PER SIDE (legalize_tetris)         │
                   │   two `placed` lists, one per copper side    │  ← fix
                   └─────────────────────────────────────────────┘
                                      │
                   ┌─────────────────────────────────────────────┐
                   │  ROUTE  (route_board_engine) → DRC → score   │
                   └─────────────────────────────────────────────┘
                                      │   stubborn nets, error sites
                   ┌─────────────────────────────────────────────┐
                   │  PROPOSE MOVES  {dx,dy,rot90, side}          │
                   │   side gated by back-free PROBE              │  ← gate
                   ├─────────────────────────────────────────────┤
                   │  SCREEN  T2 (dx,dy,rot90; quota split)       │
                   │          side bypasses T2 (externality)      │
                   │  VERIFY  T3 (shortlist + side, neighborhood) │
                   │  APPLY   single best, full route             │
                   │  JUDGE   keep-best / rollback from pristine  │
                   └─────────────────────────────────────────────┘
                                      │  loop until floor (§6)
```

The only structural changes to the loop are: the new `side` field flowing in,
the probe gate before side candidates, and side candidates joining the existing
T3 shortlist. Everything else (`auto.py` round structure, pristine baseline,
priority/persist maps, stall counter) is reused as-is.

## 6. Robustness

**Failure modes.**

- *Probe false-positive (free back, but the back is a pour, not routable).*
  The probe measures component-free area, not copper-free area. Mitigation: the
  probe is a **gate to even consider** side moves; T3 (which routes on the real
  clearance-inflated grid incl. pours) is what actually prices them, and
  keep-best is the final backstop. A flip into a pour scores no fail reduction
  in T3 and is rejected — exactly the mitayi behaviour measured. The probe only
  saves T3 budget; it never authorizes a bad move.
- *Through-hole / both-sides parts.* A part on both planes contends with either
  side. The `same_side` mask generalizes to `side_i == side_j OR either is
  both`. Connectors are locked anyway (operator policy); the residual is dual-
  footprint THT, handled by the `both` flag when a benchmark exercises it.
- *Flip changes pad escape direction AND side simultaneously.* `patch_data`
  already mirrors pads correctly (`front,back = back,front`); the verified math
  is shared with the router. No new geometry bug surface.
- *Field staleness after an accepted side move.* Same policy as every accepted
  move (coupling doc §6): one part per round, rebuild affected coarse fields.
  A side move changes which plane a net's pads escape into, so the affected
  net's field MUST be rebuilt — flagged here as the likeliest implementation bug
  for side moves specifically.
- *SEAL domination.* Unchanged from the coupling doc: cap per-pad contribution
  so one sealed pad does not mask gradient elsewhere.

**Candidate-mix / quota discipline.** Side candidates enter the *same* quota
discipline that fixed the 56→63 regression: a fixed small slot (≤2 flips, as in
today's `flips[:2]`) so they cannot crowd the proven single nudges out of the T3
budget. The lesson of constraint #3 is honored by construction.

**Run-to-run variance.** The full route is a high-variance oracle on hard inputs
(coupling doc §9: rip-up churn swings effort 48×). Mitigations already in place
carry over: lexicographic keep-best `(unconnected, errors)`, pristine baseline
each round, and `taskset -c 0-9` pinning for the degraded dev core. Side moves
add no new variance source — they are judged by the same route.

**When to STOP (floor detection).** The loop stops when N consecutive rounds
produce no T3-verified, keep-best-accepted improvement (today's `stall`
counter). Side moves extend the move space, so the floor is re-probed once with
side enabled before declaring convergence — but the stall rule is unchanged:
3 rounds without improvement ⇒ stop (coupling doc §7 V4 stop rule). On mitayi
the floor stays 56 (no free back); on a free-back board the side arm is expected
to push past the single-part-nudge floor.

**Avoiding bolt-on-arm sprawl.** The explicit design principle: ONE move space
`{dx, dy, rot90, side}`, ONE scoring contract (T2-screen where escape is
self-contained, T3-verify always, router judge always). Rotation v3, arm-2
nudges, and the storm-flip arm collapse into this one generator. Adding a future
move type means adding a candidate generator and declaring whether it is
externality-dominated (skip T2) — not a new arm with its own budget and gate.

## 7. Complexity / cost budget at our scale

60–130 nets, 2-layer, grid 510×210×2 ≈ 214k cells (coupling doc §4). Side
assignment adds:

| Operation | Cost | When |
|---|---|---|
| `side` field in extract | 0 (one attribute) | once |
| per-side overlap mask | one `[N,N]` bool multiply, N≈50–150 | per gradient step (negligible vs HPWL) |
| per-side legalize | two rect lists instead of one | once per round |
| back-free probe | one Gaussian splat over a 16² bin grid | ~ms, per round |
| side candidate T3 | ≤2 flips × T3 (45–150 ms each) | shortlist only, gated |

No new per-candidate cost beyond the existing T3 (side bypasses T2). The whole
addition is well under the existing ~10 s/pass budget. **Everything stays in
Python** (numpy + torch, CPU) — the spike (§8) runs the per-side overlap and a
14-part side search in single-digit milliseconds. No new dependency.

## 8. Spike results (verbatim)

`scripts/spike_layeraware.py` — proves the CORE fix with numbers, numpy+torch
only, importing the *shipped* `overlap_penalty` (not a re-implementation) so the
contrast is against real code. Run pinned to the healthy cores.

```
============================================================================
CLAIM 1 — per-side overlap (opposite sides do NOT collide)
============================================================================
  layer-BLIND  (shipped)            overlap =   4.000 mm^2
  layer-AWARE  same side (F,F)       overlap =   4.000 mm^2
  layer-AWARE  opposite side (F,B)   overlap =   0.000 mm^2
  same-side partial overlap: blind=2.000  aware=2.000  (must match)

  VERDICT 1: opposite-side false collision removed (4.0 -> 0.0), same-side preserved: True

============================================================================
CLAIM 2 — side assignment reduces per-side congestion (where topology allows)
============================================================================

  [zuluscsi-like]  packed front, free back
    precondition probe (back free area) = 0.98
    congestion  before = 3.0143   after = 1.6411   (6 flips, 6.4 ms)
      flip part #2: 3.0143 -> 2.5186
      flip part #10: 2.5186 -> 2.2285
      flip part #4: 2.2285 -> 2.0962
      flip part #0: 2.0962 -> 1.6932
      flip part #8: 1.6932 -> 1.6414
      flip part #7: 1.6414 -> 1.6411

  [mitayi-like]    both sides packed
    precondition probe (back free area) = 0.93
    congestion  before = 3.0188   after = 2.8301   (3 flips)

  VERDICT 2: side search helps the free-back board: True (3.014 -> 1.641); probe predicts it (z 0.98 > m 0.93): True

============================================================================
SPIKE RESULT: PASS — per-side overlap correct AND side-assignment search reduces per-side congestion where the back-free probe predicts.
============================================================================
```

Readings beyond the verdicts:

- **Claim 1 is the load-bearing fix.** The shipped penalty scores a full
  4.0 mm² collision for two parts on opposite sides at the same (x,y); the
  per-side mask correctly returns 0.0, while leaving the same-side full (4.0) and
  same-side partial (2.0) cases byte-identical. This is the bug that made the
  placer push back-side parts away from front-side parts they never touched.
- **Claim 2 confirms the topology dependence the operator named.** On the
  free-back board the greedy side search cuts the per-side congestion metric
  3.01→1.64 (−46%) in 6 flips / 6.4 ms; on the both-sides-packed board the same
  search barely moves it (3.02→2.83) — flips relocate demand *into* congestion,
  exactly the measured zuluscsi-wins / mitayi-inert split. The back-free probe
  ranks them in the right order (0.98 > 0.93), so it can gate the move cheaply.
- **Honest limit of the synthetic probe:** the probe gap (0.98 vs 0.93) is
  thinner than the congestion-reduction gap because the synthetic mitayi back
  still has geometric gaps a real pour would fill. On real boards the probe must
  measure copper-free (pour-aware) back area, not component-free — noted in §6
  as a fidelity requirement, mirroring the V2 net-carve fidelity fix.

## 9. Validation plan — measurable exit criteria

Reuses the V1/V2 gate style (cheap kill-switches first, real-board metrics
after) and the existing DRC harness. Negative results get recorded in PLAN.md
like every rung before them.

- **L0 — per-side overlap unit (done, §8).** Opposite-side parts at identical
  (x,y) score 0 overlap; same-side full and partial cases unchanged. **PASS.**
- **L1 — extraction round-trip (minutes, no routing).** On the human placements
  of the benchmark boards, every footprint's extracted `side` matches
  `fp.IsFlipped()`; legalization with two planes is byte-stable for an
  already-legal single-side board (no spurious moves). Kill: any side mismatch.
- **L2 — legalization regression (minutes).** Re-legalize mitayi (front-
  dominant): unconnected after route **must not regress vs 56**; the per-side
  planes must not *worsen* a board with little back-side content. Success: ≤ 56.
- **L3 — side-move rank correlation (V2-style).** On a free-back board
  (zuluscsi), 20 single-part flip perturbations; Spearman ρ between the T3 side
  score delta and routed-unconnected delta **≥ 0.5** (the V2 bar). Kill: ≤ 0.0
  (the externality signal does not track outcomes).
- **L4 — side arm on a 2-sided board (the payoff gate).** zuluscsi, side moves
  enabled under keep-best/rollback: **unconnected strictly below the side-
  disabled floor**, errors not regressing, ≤ 3-round stall stop. This is the
  cross-board test the storm-flip validation set up; success = the side arm pays
  where the probe says it should.
- **L5 — no-regression on mitayi (the guard).** Same loop with side enabled on
  mitayi (probe should suppress side candidates): unconnected stays **56**,
  runtime not materially worse. Confirms the gate prevents the inert-board waste
  the global-via_cost experiment warned about.

## 10. Honest self-assessment

**Is full side-assignment co-optimization worth it, or does the simpler path —
fix per-side overlap + keep the validated stall-gated storm-flip arm — dominate?**

Decomposed honestly:

- **Per-side overlap + legalization is not optional and not the question.** It
  is a *correctness* fix: the placer is wrong today (§8 Claim 1), it cannot
  rearrange back-side components, and any board with meaningful back-side
  content is mis-placed. This ships regardless. It is also cheap (one mask, one
  extra rect list) and low-risk (L1/L2 are byte-stable gates).

- **The storm-flip arm already exists, is validated, and is correctly designed**
  (T2-bypass, stall-gated, probe-predicted). It delivers the *router-loop*
  benefit of side changes today on free-back boards.

- **What "full co-optimization" would add beyond those two** is letting the
  *gradient placer* choose side during the initial place, not just the
  refinement loop flipping passives after routing fails. The measured evidence
  says this extra step is **low-yield at our scale**: rotation v1/v2 showed the
  placer's local models cannot score escape direction (the same lens that would
  judge an initial side choice), and side is *more* externality-dominated than
  rotation — the benefit is a corridor freed for OTHER nets, which the placer's
  per-part gradient fundamentally cannot see. A continuous/relaxed side variable
  would be the differentiable-congestion mistake again (constraint #5: no
  routability gain).

**Recommendation — the simpler path dominates, with one unification.** Do NOT
build a continuous side term in the gradient placer. DO:

1. **Ship per-side overlap + per-side legalization + the `side` field** (the
   correctness fix; L0–L2). This is the bulk of the value and the bulk of the
   risk reduction.
2. **Keep the storm-flip arm, but recognize it AS the `{side}` member of the
   unified discrete move space** `{dx,dy,rot90,side}` — same generator, same T3
   contract, same quota, same probe gate. This is a refactor that prevents
   bolt-on-arm sprawl, not new capability; it makes the next discrete move type
   a drop-in.
3. **Defer placer-stage side choice** until a benchmark shows the refinement-
   loop side arm is exhausted AND a board exists where initial side assignment
   (not post-route flips) is the binding constraint. No such board is in the
   suite today.

**Build order:**
1. `place/extract.py`: add `side` (additive, 0-risk). → L1.
2. `place/core.py`: `same_side` mask in `overlap_penalty`; two-plane
   `legalize_tetris`; `side` in `PlaceProblem`. → L0 (done), L2.
3. `route/engine/auto.py`: refactor the flip path into the unified
   `{dx,dy,rot90,side}` generator with the back-free probe gate (the storm arm
   becomes the `side` branch). → L3.
4. Cross-board validation on zuluscsi (L4) + mitayi guard (L5).

Steps 1–2 are correctness and ship first; 3 is a low-risk unification of code
that already works; 4 is the measured payoff gate. The expensive,
measured-low-yield idea (gradient-placer side choice) is explicitly parked.
