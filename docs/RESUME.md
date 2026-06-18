# RESUME — TraceWise router work (handoff as of 2026-06-18)

Single entry point to continue. Read this, then the design docs it points to, then do "NEXT STEP".

## Project in one paragraph
TraceWise = open-source AI-assisted place & route for KiCad. The benchmark goal: **match a human
router on 2-layer boards** — i.e. 0 unconnected, errors ≤ human — measured against the SHIPPED
human originals in `data/benchmark-boards/`. The detailed router is a grid A* (0.1mm, 2 layers,
octile + vias, counting occupancy) with bounded rip-up + negotiated-congestion pricing.

## Current scorecard (our router vs the shipped human boards, all 2-layer)
| board    | HUMAN unc/err | OURS unc/err | notes |
|----------|---------------|--------------|-------|
| zuluscsi | 0 / 5         | 15 / 150     | persistence breakthrough took it 84→15 |
| mitayi   | 0 / 0         | 48 / 104     | weakest on connectivity; 71% of errs are clearance-class |
| rp2040   | 3 / 65        | 33 / 152     | |
Reproduce: `taskset -c 0-9 .venv/bin/python scripts/place_route_measure.py data/benchmark-boards/<board>`
(routes human placement; the `--quality` finer-pitch mode helps, see below).

## The two remaining gaps (both proven to be GRID-QUANTIZATION artifacts — the key insight)
1. **Connectivity** — the 0.1mm grid OVER-ESTIMATES congestion. MEASURED: finer grid connects more
   (mitayi 0.1→48, 0.075→28, 0.05→27 unconnected). Exact geometry = the limit.
2. **Legality** — grid-legal tracks land sub-pitch from copper → clearance errors. mitayi's 104
   errors are 71% clearance-class (clearance 42 + hole_clearance 32) — see EXACT-GEOMETRY probe.
**Unifying solution: exact-geometry (gridless) routing eliminates BOTH.** Measurement-justified,
not assumed. The "needs 4 layers" idea was DISPROVEN — the human routes all boards on 2 layers.

## What shipped this session (all committed, full suite green ~150 tests)
- Router −54% faster + DETERMINISM restored (vectorized A* heuristic + neighbor expansion).
- Congestion pricing default on (`history_factor=1.0`).
- F0 pour extraction (pcbnew API), F3 power-pour synthesis (zuluscsi 84→65), L1 ceiling detector.
- Placement functional grouping (cross-validated: mitayi/rp2040 placer routability ↑).
- **Persistence breakthrough**: incremental + salvage-pass routing (route_net keeps the sub-tree on
  a blocked pad instead of discarding the whole net) — zuluscsi 65→15, mitayi 63→48. `allow_partial`
  default on in `route_board_engine`.
- `--quality` CLI flag: `tracewise route --engine tracewise --quality` → 0.075mm grid (mitayi
  48→28 at 1.5× runtime). Interim connectivity lever.
- PathFinder convergence fixes (correct McMurchie-Ebeling schedule + retry failed nets).

## Closed dead-ends (DO NOT retry — each falsified with a root cause; see ROUTING-COMPLETION-PLAN.md)
PathFinder-as-full-router (didn't converge full-res; runtime), F2 stub-stitch / F2' island-bridge /
F4 / L2 (power pours CANNOT be locally edited — every refill re-solves the global fill), placement
flips, escape-tuning for legality (errors are grid-quantization, not shaving), local/iterative levers
on mitayi (capacity is zero-sum redistributable, auto-loop stuck at 48), coarse-grid negotiation
(0.2mm boolean grid is DEGENERATE for fine-pitch).

## Design docs to read IN ORDER (the global-router chapter)
1. `docs/design/GLOBAL-ROUTER-DESIGN.md` — scope + the Phase-2 quantization PIVOT.
2. `docs/design/EXACT-GEOMETRY-ROUTER-ARCH.md` — the v2 blueprint + the #4-value probe. **START HERE for the build.**
3. `docs/research/PHASE0-negotiated-congestion-algorithm.md`, `docs/research/NEXT-exact-geometry-routing.md` — algorithm + approach detail.
4. `docs/PLAN.md` (session summary) + `docs/ROUTING-COMPLETION-PLAN.md` (full measured arc).

## NEXT STEP — exact-geometry "#4 NEAR build" (legality, ~3-5 days), measured target mitayi 104→~30
Build the Minkowski-snap exact-geometry EMITTER: replace `emit_routes` (in
`src/tracewise/route/engine/kicad.py`)'s `grid.to_world(iy,ix)` cell-center endpoint snapping with
exact-legal track/via endpoint placement (Minkowski-inflated obstacles). 71% of mitayi errors are
the clearance-class this fixes.
- Phase A (do FIRST): add `shapely` to pyproject deps (or numpy segment-distance fallback);
  implement an exact segment-clearance + Minkowski-endpoint helper with FIXTURE unit tests, BEFORE
  touching emit.py.
- Phase-A PROBE (cheapest first move, no new dep): on a routed board, use numpy exact segment-to-
  copper distance to confirm how many of the 74 clearance-class errors are endpoint-nudgeable vs
  mid-segment/pour-interaction (sets the realistic #4 ceiling).
- Then refactor emit; MEASURE on the scorecard (errors → toward human; no unconnected regression).
- FAR horizon (separate, months): full gridless router for the CONNECTIVITY gap.

## How to work here (methodology that worked — IMPORTANT)
- **Always `taskset -c 0-9`** (degraded CPU on this dev box).
- **Probe before building** — every big idea this session was validated/falsified by a cheap probe
  first. It repeatedly saved multi-day dead-ends.
- **Measure vs the HUMAN scorecard**, not internal scores (local placement/routing scores mislead).
- **Determinism is mandatory** — routes must COMPLETE within budget; never wall-clock-truncate.
- **Subagents**: the `architect`/`developer` roles are blocked here by an Orchestray WorktreeCreate
  hook (cwd isn't a git repo for the parent shell). Use `general-purpose` agents with crisp specs;
  the `researcher` role works for surveys. Give agents ABSOLUTE paths and the venv path.
- venv: `/home/palgin/Business_projects/tracewise/.venv`; lint `ruff`; tests `pytest -q`. KiCad/pcbnew
  available; `_run_pcbnew_script` in bridge.py is the pcbnew entry. Routes are slow (zuluscsi ~530s,
  mitayi ~176s) — run measurements in the background.
