# Place–Route Coupling: Escape-Coupled Cost Fields (ECCF)

*Design + measured spike, June 2026. The missing link named in
[docs/PLAN.md](PLAN.md): placement decisions need router-grounded scoring, but a
full route per candidate (40–90 s) cannot serve a search that evaluates
hundreds-to-thousands of candidates. This document specifies a routability
signal that is grounded in the production router's own grid, costs ~30 ms per
candidate, and — unlike every placement-side model tried so far — scores
ESCAPE DIRECTION and corridor capacity. Prototype:
[scripts/spike_coupling.py](../scripts/spike_coupling.py), output reproduced in §8.*

## 1. The measured problem this must solve

The placer ladder (PLAN.md, v0.3 + refinement loop) established, by
measurement, what a coupling signal must and must not be:

| Finding | Consequence for the signal |
|---|---|
| Routability of a placement is the binding constraint; the router cannot fix geometry | the signal must predict the *router's* outcome, not a proxy |
| Ordering feedback in the route loop is zero-sum (corridors are conserved) | per-net scores summed independently are not enough; competition must be visible |
| Overlap was the first gate (Tetris legalizer: 101→89; human placement: 63) | the signal operates *after* legalization, on overlap-free candidates |
| Differentiable pin-density congestion: NO routability gain | smooth field proxies wash out the discrete geometry that matters |
| Rotation v1 (fit) and v2 (HPWL-veto): identical negative (97 vs 89). Every rotation is locally wirelength-neutral; the damage is escape direction | the killer case is *invisible to any local placement-side model* — the signal must see the actual corridor geometry around the pads |
| Full board route ≈ 40–90 s; placement explores thousands of candidates | budget: **≤ 50 ms per candidate evaluation** |

Scale: the reference board grid is **510 × 210 cells × 2 layers = 214 200
cells** at 0.1 mm pitch, 60–120 nets, 50–150 components.

## 2. Prior art, and what each misses for this case

- **RUDY** (Spindler & Johannes, DATE'07) — each net spreads its wire demand
  uniformly over its bounding box; cheap and near-differentiable. Misses:
  obstacle-blind (demand over blocked cells counts the same), and the net bbox
  is *rotation-invariant for exactly the reason HPWL is* — both orientations of
  the rotation-killer case score identically. Useless for escape.
- **Probabilistic congestion maps** (Lou et al.; Westra's L/Z-shape models) —
  per-net crossing probabilities over the bbox. Same two blind spots as RUDY;
  additionally, capacity is modeled as uniform supply, while on a dense 2-layer
  PCB the supply *holes* (pad fields, halos) are the entire story.
- **ePlace/RePlAce routability modes** (periodic global-router feedback, e.g.
  NCTUgr; cell inflation in congested GCells) — closest in spirit: run a cheap
  router, feed a map back. Misses: GCell-granularity global routing abstracts
  away exactly our failure scale — escape failures happen within 1–2 mm
  (10–20 cells), *below* GR resolution; ASIC standard cells have no
  escape-direction freedom, so that literature never needed to score it; and
  cell inflation needs area slack — the reference board is at 101 % density.
- **FLUTE / Steiner lookahead** — topology-accurate net length, obstacle-blind.
  Obstacle-avoiding variants (OARSMT) exist but are expensive per candidate and
  still score nets independently (competition-blind).
- **Pin-accessibility-driven detailed placement** (NTUplace-DR lineage,
  sub-10 nm M1-access work) — the right *problem* (local escape feasibility per
  cell per orientation), solved by precomputed per-library-cell access
  patterns. PCBs have no cell library: escape feasibility is a function of the
  actual, continuous neighborhood (the wall is the *neighbor's* halo field).
  The lesson carries (precompute escape per orientation); the lookup-table
  mechanism does not.
- **PathFinder negotiation** — history-cost fields that persist across routing
  iterations. It is a router-internal signal, and PLAN.md already measured that
  negotiation/ordering alone is zero-sum at our density. The reusable idea is
  different: *routing computations can persist and be reused as fields*.
- **Differential heuristics / ALT landmarks** (pathfinding literature) — exact
  cost-to-go fields reused as admissible A\* heuristics. Textbook; never, to
  our knowledge, used as a *placement* scoring primitive with a forced-boundary
  local correction (§3, T2), which is where the escape-direction sensitivity
  comes from.

Gap: nothing above is simultaneously (a) grounded in the real clearance-
inflated grid, (b) sensitive to which way pads *face*, (c) aware of corridor
competition, and (d) ~milliseconds per candidate. ECCF is the combination; the
components are deliberately textbook.

## 3. The mechanism

Three tiers over the **production router's own occupancy grid** (`grid.cells
== 0`, same per-net inflation `track/2 + clearance`, same octile metric, same
via cost) — not a re-implementation of routing reality, a *reuse* of it. The
tiers form a funnel; the existing full route with keep-best rollback
(`auto.py`) stays the final judge.

### T1 — cached per-net cost-to-go fields (built once per round)

For each net of interest, a multi-source min-plus wavefront (vectorized
Bellman over numpy, 8-connected octile steps, layer flips at via-legal cells
costing `via_cost`) from the net's terminals over the grid **without the
candidate part**. The result `CTG_n[layer, y, x]` is the true grid distance
from any cell to the net's nearest terminal, around every real obstacle.

Fields are *candidate-independent by construction*: one build serves every
orientation/position trial of the part. They can be built on an
**optimistically coarsened** grid (a coarse cell is free iff any fine cell in
its f×f block is free; measured f=3: 24× faster, ranking preserved — §8).
Optimistic coarsening is safe because tier T2 is exact at full resolution
where the candidate geometry lives, and a pessimistic coarse field could
manufacture false seals.

### T2 — escape stitch (the per-candidate score, ~30 ms)

To score *part P at orientation θ / position x*: apply P's pads to a copy of
the grid (numpy rect ops, the same `block_pad` the router uses), carve the
scored net's own pad exactly as `route_net` does, then run an **exact local
wavefront in a (2R+1)² window** around each pad (R = 30 cells = 3 mm) and
stitch onto the cached field at the window's **exit ring**:

```
score(pad) = min over ring cells r at Chebyshev distance R:
                 d_local(pad → r  | candidate geometry, full res)
               + CTG_net(r)        | cached, candidate-blind
score(candidate) = Σ_pads score(pad);   ring unreachable ⇒ SEAL penalty
```

**Why the exit ring is load-bearing** (a measured design lesson, not a
nicety): cost-to-go fields are 1-Lipschitz in the grid metric, so
`min over all visited cells of (g + CTG)` collapses to `CTG(pad)` — the local
term silently vanishes and the candidate's own geometry prices at zero. The
first spike version had exactly this bug. Forcing the hand-off at distance R
makes the pad pay the true local escape cost through its own body, its unused
pad rows, and the neighbors' halo fields before it may touch the optimistic
field. Any "cached field + local correction" scheme needs a forced boundary.

**Why this captures escape direction** — three distinct effects:

1. *Global detour*: the field already contains every wall and pocket; a pad
   ring-exit on the walled side reads a much higher `CTG` than on the open side.
2. *Local self-geometry*: `d_local` runs on the candidate grid at 0.1 mm — the
   rotated part's own copper (e.g. the unused second pad row of a header)
   and the exact neighbor halos price the first 3 mm of escape, which is where
   the measured failures live.
3. *Sealing*: if no ring cell is reachable, the pad is walled in — returned as
   a hard SEAL penalty. This is the rotation killer detected directly, including
   the 180° case where the bounding box is *identical* and no box- or
   HPWL-based model can even represent the difference.

The **coupling signal is the excess over the obstacle-free octile lower
bound** (per net). The lower bound itself is wirelength — HPWL already prices
it; subtracting it isolates what placement-side models cannot see.

### T3 — field-guided pseudo-route (capacity-aware verifier, ~tens of ms)

Independent per-net scores cannot see zero-sum corridor competition (the
measured ordering-feedback finding). T3 prices it: route the candidate's
attached nets **sequentially, one A\* attempt each, no rip-up**, using
`CTG_n` as the heuristic and splatting each routed path into a shared
reservation map (track halfwidth; via discs both layers) that later nets pay
to cross. Because reservations only *add* cost, the cached heuristic stays
admissible; because it is near-exact, A\* expands ≈ path-length cells when
uncontested and only grows where contention forces detours. A net that
exhausts its expansion cap scores as a failure.

T3 is router-in-the-loop made affordable: the cached field replaces the weak
octile heuristic (which degrades exactly when there are detours, i.e. exactly
when boards are interesting), and the net set is restricted to the candidate's
neighborhood.

### The funnel

```
candidates (10²–10⁴)  ──T2 screen (≈30 ms)──▶  shortlist (≈10)
   ──T3 verify (≈30–150 ms)──▶  best candidate applied
   ──existing auto.py full route, keep-best, rollback──▶  accepted or reverted
```

## 4. Complexity at scale (510 × 210 × 2 cells, 60–120 nets) — measured

| Operation | Cost (measured in the spike) | When paid |
|---|---|---|
| T1 field, full res | 293 ms/net | once per round, stubborn nets only |
| T1 field, coarse f=3 (2×70×170) | 12 ms/net | once per round, all candidate nets |
| T2 stitch, 6-pad part, R=30 | **28 ms/candidate** (≈5 ms/pad) | per candidate — the inner loop |
| T3 pseudo-route, 5–6 nets | 45–150 ms/set (cap-bound on fails) | shortlist only |
| Production full route (ground truth) | 1.3 s (clean) … 62 s (rip-up churn) | final judge only |

Worked example, rotation pass: 40 rotatable parts × 4 orientations × 28 ms ≈
4.5 s of T2, plus ~1–2 s of coarse field builds (fields only for nets attached
to rotatable parts, deduplicated), plus T3 on a shortlist ≈ **under 10 s per
pass**. Router-in-the-loop sampling of the same 160 candidates at 40–90 s each
is 1.8–4 h: a ~10³ gap. Memory: one float32 field per net = 0.86 MB full-res,
0.1 MB coarse — all 120 nets cached coarse ≈ 12 MB.

T2 scales linearly in pad count; high-pin parts are excluded by policy anyway
(the arm-2 v1 wreckage: never move high-pin parts).

## 5. Integration points

1. **Rotation v3 — `place/core.py optimize()`**: replace the HPWL-veto
   `rotatable` logic with ECCF. After `legalize_tetris`, for each rotation
   candidate score {0°, 90°, 180°, 270°} via T2 (pad transforms per the
   verified v2 math), accept `argmin` only if it beats the current orientation
   by a margin; re-legalize if the box swapped. Note: ECCF can rank 180°
   against 0° — box-identical, escape-different — which no previous version
   could even express.
2. **Arm 2 trust-region nudges — `route/engine/auto.py`**: for stubborn nets
   (the existing `persist` map), generate candidates for the net's smallest
   attached part on a ±2 mm ring × orientations, T2-screen, T3-verify the top
   few against the neighborhood net set, apply the best, and let the existing
   round structure (pristine baseline, keep-best, rollback) judge. This is the
   v2 requirement list from the wreckage — trust region, one stubborn net at a
   time, never move high-pin parts — finally with a scoring function.
3. **Not recommended: a differentiable placer term.** A finite-difference
   force from T2 is possible, but the congestion-term experiment already
   showed smooth proxies don't move routability, and the killer decision
   (orientation) is discrete. ECCF stays in the discrete arms.

## 6. Failure modes

- **Field staleness.** Fields are built without the candidate part; once a
  move is *accepted*, fields whose nets touch the changed region are stale.
  Policy: accept one part per round, rebuild affected coarse fields (~12 ms
  each). Bounded and cheap; forgetting it is the likeliest implementation bug.
- **Optimism beyond the window.** `CTG` is candidate-blind: effects of the
  moved part farther than R from its own pads (it may wall in a *neighbor's*
  escape) are not priced by T2. T3 partially catches this (neighborhood nets
  pseudo-route on the candidate grid); the trust region keeps displacement
  small; the full route catches the rest.
- **Multi-pin nets.** `CTG` from all terminals scores pad→nearest-terminal,
  under-pricing Steiner topology for high-fanout nets. Power nets are excluded
  from the score (routed first and pour-assisted anyway); residual risk is
  mid-fanout signal nets.
- **Pessimism vs the escape allowance.** The production A\* may traverse
  clearance halos near endpoints (escape=12, penalty 4.0); the spike's field
  and stitch do not, so ECCF can declare SEAL where the router would squeeze
  through. Mitigation if V1/V2 (§7) show it matters: mirror the two-tier
  halo-passability inside the T2 window — the `hard` plane already exists in
  `Grid`.
- **T3 is a ranking signal, not a completion oracle.** No rip-up, fixed
  ordering: in spike scenario B it predicted 3 failures where the production
  router (with rip-up churn) produced 4. Direction and magnitude were right;
  absolute counts are not the contract.
- **SEAL constant.** A single sealed pad dominates any sum; when comparing
  multi-part candidates, cap the per-pad contribution so one seal does not mask
  gradient information elsewhere.

## 7. Validation plan — measurable exit criteria vs the 89/63 ladder

- **V0 (done, §8):** synthetic separation, production-router ground truth.
  Criteria: T2 excess separates HPWL-neutral orientations ≥1.5×; T3 separates a
  capacity wall that T2 cannot; ground truth agrees. **PASS.**
- **V1 — human-orientation recovery (no routing needed, ~minutes):** on the
  benchmark boards' *human* placements, score all 4 orientations of every
  2-sided part; the human's choice must rank top-2 for **≥ 70 %** of parts.
  Kill criterion: ≤ 50 % ⇒ the signal is too weak; stop before integration.
- **V2 — rank correlation:** 20 random single-part perturbations of the
  legalized placement; Spearman ρ between ECCF delta and routed-unconnected
  delta **≥ 0.5**.
- **V3 — rotation v3:** placer ladder must not regress: **unconnected ≤ 89**
  with rotations enabled; success = **< 89**. (v1 and v2 both measured 97.)
- **V4 — arm-2 ECCF nudges:** target **≤ 84** on mitayi (+5 nets toward the
  human 63), errors not regressing, under the existing keep-best/rollback.
  Stop rule: 3 rounds without improvement.

Each gate uses the existing DRC harness; negative results get recorded in
PLAN.md like every other ladder rung.

## 8. Spike results (verbatim output)

`scripts/spike_coupling.py` — synthetic boards at production geometry (0.1 mm
pitch, 0.2/0.2 track/clearance, 2 layers), scored by T1/T2/T3 and judged by
the production `route_all` (escape=12, rip-up). Scenario A is the rotation
killer: a 6-signal header inside a connector pocket, 180° flip, HPWL delta
−1.1 %. Scenario B is a corridor-capacity wall.

```
============================================================================
Scenario A — escape direction (180° rotation, HPWL-near-neutral)
============================================================================
HPWL  good=144.7mm  bad=143.1mm  (delta -1.1% — what HPWL-veto rotation sees)
T1 field build (cached, once per round): full-res 293ms/net, coarse(f=3) 12ms/net  (grid (2, 210, 510))
[good] T2 excess=   40.9 (coarse    37.0, sealed 0,  28.0ms/cand) | T3 excess=  284.5 fails=0 (152.4ms) | GT routed 6/6, copper 1385 (1.3s)
[bad ] T2 excess=  201.1 (coarse   174.9, sealed 0,  28.5ms/cand) | T3 excess=  552.5 fails=0 (145.7ms) | GT routed 4/6, copper 1129 (62.6s)
VERDICT A: T2 separates True (excess 41 -> 201; coarse True), T3 separates True (excess 284 -> 553), ground truth confirms damage: True (router effort x48)

============================================================================
Scenario B — corridor capacity (independent fields vs pseudo-route)
============================================================================
[wide  ] T2 independent=  1124.0 | T3 pseudo=   1137.3 fails=0 ( 44.2ms for all 5) | GT routed 5/5
[narrow] T2 independent=  1172.9 | T3 pseudo=  30485.6 fails=3 (480.9ms for all 5) | GT routed 1/5
VERDICT B: T2 capacity-blind as predicted: True; T3 sees the capacity wall: True; ground truth drops: True

============================================================================
SPIKE RESULT: PASS — escape direction separated by T2/T3 and confirmed by the production router; capacity separated by T3 only (as designed).
```

Readings beyond the verdicts:

- The HPWL-neutral flip (−1.1 %, i.e. the *bad* orientation looks marginally
  better to the placer's only current lens) loses 2 of 6 nets and costs the
  router **48× the effort** even on the nets it does complete. ECCF's T2
  excess separates the orientations **4.9×** at 28 ms per candidate, and the
  24×-cheaper coarse field preserves the ranking.
- Scenario B confirms both halves of the tier design: independent fields are
  capacity-blind (+4 % where completion collapses 5/5 → 1/5), and the
  reservation pseudo-route sees the wall (cost ×27, 3 predicted failures).
- The 62 s ground-truth run is itself evidence for the whole program: bad
  placement doesn't just fail nets, it burns the route budget in rip-up churn.

## 9. Honest self-assessment

**Is this genuinely better than router-in-the-loop sampling?** Decomposed:

- *Against full-route-per-candidate*: yes, decisively — ~10³ cheaper per
  candidate, and the spike shows full routes are also a *noisy* judge at the
  per-candidate level (rip-up churn swings effort 48× and the engine's own
  completion varies run to run on hard inputs). Router-in-the-loop for
  thousands of candidates is not merely slow; it is a high-variance oracle.
- *Against reduced router-in-the-loop* (route only affected nets on the real
  grid): this is not actually a rival — it is **tier T3**. A plain reduced
  route with an octile heuristic explores far more cells exactly when the
  board is congested; the cached field heuristic is the thing that makes it
  affordable. The honest framing: ECCF doesn't *replace* router-in-the-loop,
  it makes a small router-in-the-loop cheap enough to be a scoring function,
  and adds a millisecond screen (T2) in front of it.
- *What would make the verdict "don't build it"*: V1 failing (humans' chosen
  orientations not recovered) would mean escape direction is not actually
  captured well enough outside constructed scenarios — that gate costs minutes
  and runs before any integration work. V2 failing would mean per-candidate
  deltas don't track router outcomes on real density. Both kill switches are
  scheduled first deliberately.
- *Known intellectual debt*: the components are textbook (distance fields,
  ALT-style heuristic reuse, reservation routing); the claim is the
  integration — same-grid grounding, the forced-boundary stitch (without which
  the local term provably vanishes; measured), the excess-over-octile
  decomposition that separates coupling from wirelength, and the
  screen/verify/judge funnel matched to the measured cost ladder. Novelty for
  its own sake was not the goal; closing the 89→63 gap is.

**Verdict: build it** — as the funnel in §3, gated by V1/V2 before any placer
integration, with rotation v3 (V3) as the first consumer because it is the
measured, isolated failure that no local model can express.
