# Phase 1 — Coarsen-then-Refine Negotiated Router (architecture)

Status: architecture (2026-06-18). Implements Stage 1 of GLOBAL-ROUTER-DESIGN.md (the connectivity
lever) made tractable. Built ENTIRELY on existing components + the Phase 0 corrected PathFinder.

## Problem this solves

The corrected PathFinder (commit 36b661b) converges correctly but is impractical at full 0.1mm
resolution (Phase 0 prototype: >40 min on the 428k-cell mitayi; zuluscsi 1.8M would be hours).
Fix: do the GLOBAL negotiation on a COARSE grid (fast), then REFINE each net's detailed path on
the fine grid CONSTRAINED to a corridor around its coarse route (small, bounded searches).

## Pipeline (3 stages, all reusing existing parts)

```
1. COARSE NEGOTIATION  (0.2mm grid)  -> per-net coarse corridor (congestion-free global plan)
2. CORRIDOR REFINEMENT (0.1mm grid)  -> per-net detailed route, A* constrained to its corridor
3. SALVAGE + emit + F3 pours + refill (existing)
```

### Stage 1 — coarse negotiation
- Build a coarse `Grid` at `coarse_pitch` (≈0.2mm = 2× fine). `build_problem(data, pitch=0.2)`
  already works (verified). Obstacles map directly (pads/keepouts inflated at coarse pitch).
- CRITICAL (Phase 0 lesson): preserve the hard/halo split + escape allowance on the coarse grid —
  the crude PathFinder died without escape. The coarse `Grid` already carries `hard`/`cells`;
  `route_all_pathfinder` already uses the escape (`fixed_pen`) model. No change needed.
- Run the corrected `route_all_pathfinder` (Phase 0 schedule: p_fac=0 iter0, p_init 0.5, growth
  1.3, cap 20, h_fac 1.0) on the coarse grid to a congestion-free state. ~16× fewer cells ⇒ the
  per-iteration A* and the dilation/history numpy ops are ~16× cheaper ⇒ minutes not hours.
- OUTPUT per net: the coarse path cells (`NetRoute.cells` from the coarse run) = its CORRIDOR
  spine.

### Stage 2 — corridor refinement (the one genuinely new component)
- For each net, build a fine-grid CORRIDOR MASK: fine cells within `corridor_radius` coarse-cells
  (start R=2) of the net's coarse spine (upsample coarse cells → fine block + dilate). A boolean
  `[L,H,W]` mask (reuse the `_dilate` pattern from pathfinder.py / rasterize_pour).
- Route the net's fine detailed tree with the EXISTING `astar.route`, but FORBID cells outside the
  corridor (pass the corridor as an extra constraint: cells outside → treated as hard). This
  bounds the fine A* to a thin region ⇒ fast, and it realizes the negotiated global plan at exact
  fine resolution with vias + escape.
- Nets route INDEPENDENTLY in Stage 2 (their corridors are mostly disjoint after the coarse
  negotiation made them congestion-free) — so Stage 2 is embarrassingly parallel and each search
  is small. Order is deterministic (sorted).

### Stage 3 — salvage + emit (existing, unchanged)
- A net whose corridor refinement fails (corridor too tight / coarse-fine capacity mismatch) falls
  back to the existing full-grid SALVAGE pass (incremental routing). This is the safety net.
- Then `emit_routes` + `refill_zones` + F3 `synthesize_power_pours` as today.

## Determinism

- Coarse negotiation COMPLETES (it's fast) — no wall-clock truncation.
- Fine refinement is per-net bounded (corridor) — small, completes.
- Deterministic ordering throughout. Re-runs are byte-identical (the property restored this
  session and required by the scorecard).

## Integration

- New `engine="negotiated"` branch in `route_board_engine`:
  `coarse_grid = build_problem(data, pitch=coarse_pitch)` → `route_all_pathfinder(coarse...)` →
  for each net: corridor mask from coarse cells → `astar.route(fine_grid, ..., corridor=mask)` →
  collect NetRoutes → salvage fallback → existing emit/refill/pours.
- `astar.route` gains an optional `corridor` mask param (cells outside the mask are forbidden);
  default None = unconstrained (no behavior change for existing callers).
- Reuses: Grid, corrected route_all_pathfinder, astar.route, _dilate, emit_routes, refill_zones,
  F3 pours, salvage. The NEW code is: coarse↔fine cell mapping, corridor-mask construction, and
  the `corridor` constraint in astar.route. Small surface.

## Risks & mitigations

- **Coarse-fine capacity mismatch** — a coarse-congestion-free plan may lack a fine-legal
  realization in the corridor. Mitigate: corridor_radius ≥ 2; coarse cell capacity calibrated to
  fine tracks-per-coarse-cell; salvage fallback catches the rest.
- **Coarse downsampling hides thin gaps** the fine router needs. Mitigate: conservative obstacle
  mapping (a coarse cell is free only if it has a fine-feasible passage) + fine refinement does the
  exact work; the coarse plan is a GUIDE, not the final geometry.
- **Vias across scales** — coarse via sites guide; fine refinement places exact vias (via_ok at
  fine res) within the corridor.
- **Legality** — clearance errors remain a fine-grid-quantization issue → Stage 2 of the overall
  design (exact-geometry pass), unchanged by this.

## Phase 2 build order (each gated by measurement)

1. `astar.route` corridor-mask param + unit test (forbids outside mask; unconstrained default
   unchanged). [smallest, no behavior change]
2. Coarse↔fine cell mapping + corridor-mask builder + unit tests (mask covers the upsampled spine).
3. `engine="negotiated"` wiring: coarse negotiate → corridor refine → salvage → emit. 
4. MEASURE vs human scorecard on mitayi/zuluscsi/rp2040. Gate: coarse negotiation completes in
   minutes; unconnected materially below 15/48/33 toward ~0; deterministic; no regression.

## Acceptance (Phase 1 → Phase 2 entry)

Coarse negotiation runtime confirmed tractable (probe pending: corrected PathFinder on a 0.2mm
mitayi grid should finish in minutes, vs >40 min at 0.1mm). If confirmed, Phase 2 builds the
corridor refinement; if the coarse run is still slow, drop to 0.3mm coarse or cap coarse iters.

## PHASE 1 PROBE FINDING — the coarse grid must be CAPACITY-based, not boolean (2026-06-18)

Probed the central assumption (coarse negotiation is fast) by running the corrected PathFinder on
a NAIVE coarse grid = `build_problem(mitayi, pitch=0.2)` (boolean fine-clearance inflation at 0.2mm).
RESULT: 53,760 cells, 583s (10 min), ok=0/61. Two revisions follow:

1. **The naive coarse grid is DEGENERATE for fine-pitch boards.** At 0.2mm pitch, the fine inflation
   (track/2 + clearance ≈ 0.3mm) exceeds the coarse pitch, so every pad blocks a multi-cell radius
   and the coarse grid is almost entirely "blocked" → ok=0, nothing routes. CONCLUSION: Stage 1's
   coarse grid CANNOT reuse build_problem at a coarse pitch. It must be a proper CAPACITY-BASED
   global-routing graph (FPGA-style): each coarse cell carries a CAPACITY = how many fine tracks
   fit through it (≈ coarse_pitch / (track+clearance)), and congestion = usage > capacity — NOT a
   boolean obstacle. This is the McMurchie-Ebeling capacity model the Phase 0 spec described;
   build_problem's boolean inflation was the wrong proxy. [Phase 2 must build this graph.]

2. **The negotiation does not CONVERGE at capacity** (ran all 40 iters). Expected: McMurchie-Ebeling
   reaches congestion-free only when capacity suffices; on a capacity-tight board it iterates to the
   cap. Needs: iteration cap + salvage fallback (already planned) + the capacity model (1) so
   "congestion" is real, not the degenerate boolean.

3. **DEEPER INSIGHT (revises the whole design):** mitayi is HUMAN-routable to 0, yet our grid model
   can't find the assignment. The 0.1mm grid quantization likely OVER-ESTIMATES congestion (the
   human packs tracks at sub-grid spacing the grid can't represent). So exact geometry may be needed
   for CONNECTIVITY on fine-pitch boards, not only legality — strengthening the case that the v2
   gridless/exact-geometry router (design Option C) is the true unlock for the last gap, with
   negotiated congestion layered on the exact-geometry capacity model rather than the 0.1mm grid.

REVISED Stage-1 plan: build a capacity-based coarse global-routing graph (not boolean), negotiate on
it (corrected schedule + iter cap), refine to fine corridors. If the capacity graph still can't
represent mitayi's fine-pitch packing, the connectivity gap converges with the legality gap onto
exact-geometry (Option C) as the unifying solution. Phase 2 entry gate: a capacity-based coarse
negotiation that completes in minutes AND reaches ~congestion-free on a SLACK board (zuluscsi)
before tackling capacity-tight mitayi.
