# Global Router — design scope

Status: scoping (2026-06-18). Grounds every decision in this session's measured findings.
The goal is the one remaining frontier: close the gap to the SHIPPED human boards.

## 1. The measured problem

The current detailed router is a grid A* (0.1mm pitch, 2 layers, octile + vias, counting
occupancy) routed GREEDILY net-by-net with bounded rip-up. Two gaps to the human (proven by the
scorecard vs the shipped originals — all 2-layer):

| board    | human unc/err | ours unc/err | gap |
|----------|---------------|--------------|-----|
| zuluscsi | 0 / 5         | 15 / 150     | connectivity 15, legality ~145 |
| mitayi   | 0 / 0         | 48 / 104     | connectivity 48, legality 104 |
| rp2040   | 3 / 65        | 33 / 152     | connectivity 30, legality ~87 |

- **Connectivity gap** = CAPACITY CEILING. mitayi is exhaustively proven stuck at ~48 across
  every local AND iterative lever (single pass 48; ripup 8/16/32 → 48; ordering/skip-GND →
  zero-sum 43-52; auto-loop 6 iters → 48). Nets COMPETE for shared corridors; greedy+bounded
  rip-up cannot find the joint assignment the human finds. +3V3 routes FULLY ALONE on the empty
  grid — its 22 unconnected is 100% congestion. This is a GLOBAL-OPTIMIZATION gap.
- **Legality gap** = GRID QUANTIZATION. Clearance violations from grid-legal paths landing
  sub-pitch (<0.05mm) from copper. Proven NOT escape-tunable (errors ~151 at salvage_escape
  {0,4,12}). Needs EXACT geometry.

## 2. Goal / success metric

Match the human on the benchmark suite (2 layers): **unconnected → ≤ ~5 per board**, **errors → ≤
human (0-65)**, **deterministic** (byte-identical re-runs). Measured by the routing-in-the-loop
scorecard (`scripts/place_route_measure.py` + per-net DRC breakdown) against the shipped originals.

## 3. Constraints learned THIS session (every design choice must respect these)

1. **Determinism is mandatory and achievable** — restored via the vectorized A* heuristic; routes
   must COMPLETE within budget (no wall-clock truncation). Keep it.
2. **Incremental routing is decisive** — discarding a net over one blocked pad was the dominant
   failure (all-or-nothing → salvage took zuluscsi 84→15). The global router MUST treat a net as a
   set of independently-completable connections, never all-or-nothing.
3. **Negotiated-congestion PRICING works as a bias** (history_factor cross-validated: zuluscsi
   combined 924→859) BUT the crude full PathFinder router FAILED to converge (mitayi 0/61) — it
   lacked the escape/clearance model and a proper cost schedule. A real implementation MUST carry
   the hard-won escape allowance (neck through halo near endpoints) and the hard/halo grid split.
4. **Power pours cannot be locally edited** — every refill re-solves the global fill (F2/F2'/F4/L2
   all died here). Pours are PLANNED up front (F3 synthesis), not stitched post-fill.
5. **Grid quantization caps legality** — clean clearance needs exact geometry, not a finer grid
   alone (a finer grid trades connectivity and cost).

## 4. Design options (evidence-weighed)

- **A — Proper McMurchie-Ebeling PathFinder (negotiated congestion).** All nets share a capacity
  graph; each iteration rip-up-and-reroutes congested nets at a cost = base·(1+h·history)·
  (1+p·present_congestion), escalating until congestion-free. CONVERGENT by construction with the
  right schedule. Directly attacks the CAPACITY ceiling. Reuses: grid, escape model, incremental
  routing, history pricing. RISK: our crude attempt didn't converge — must get escape + schedule
  right; runtime (iterate all nets). This is the connectivity lever.
- **B — Grid + exact-geometry post-pass (push-and-shove / Minkowski snap).** Keep the router; add
  a geometric legality cleanup that nudges emitted tracks to exact clearance. Attacks LEGALITY
  (errors), NOT capacity. Smaller (Shapely). This is the legality lever.
- **C — Full gridless shape-based router.** Attacks both; 3-6 months; highest risk. Defer.
- **D — ILP/SAT global routing.** Optimal; scalability risk on 1.8M-node grids. Research-only.

## 5. Recommended architecture (staged)

- **Stage 1 (the big win): Option A — negotiated-congestion global router** for connectivity.
  Built on the existing grid + escape + incremental(salvage) + history learnings, with PROPER
  convergence (McMurchie-Ebeling cost schedule, all-nets-per-iteration, present+history costs).
  Deterministic; salvage as the non-convergent fallback. Behind `engine="negotiated"`. Target:
  unconnected → ~0.
- **Stage 2: Option B — exact-geometry legality pass** for the error gap. Shapely push/nudge of
  emitted segments to true clearance. Target: errors → ≤ human.
- **Defer Option C** unless Stages 1+2 plateau short of the human.

## 6. Reuse inventory (de-risks the build — most of the machine exists)

Grid (occupancy, hard/halo split, block_polygon); determinism (vectorized heuristic); escape
allowance; incremental/salvage routing; history congestion pricing; F0 pour extraction; F3 power-
pour synthesis; L1 ceiling detector (validation/triage); the routing-in-the-loop harness; the
human-baseline scorecard. The negotiated router is largely a NEW OUTER LOOP over EXISTING parts.

## 7. Risks & mitigations (from session evidence)

- **Convergence** (the crude PathFinder failed): use the canonical McMurchie-Ebeling schedule
  (present-cost grows within an iteration, history accumulates across iterations); integrate the
  escape model (the crude version's fatal omission); cap iterations with salvage fallback.
- **Runtime** (all-nets iteration on 1.8M grid): vectorized heuristic + history field; consider
  coarsen-then-refine (0.2→0.1mm); per-iteration time cap; MUST stay deterministic (complete, not
  truncate).
- **Legality**: Stage 2 exact geometry (grid alone can't).
- **Pour interaction**: plan pours up front (F3); never stitch post-fill.

## 8. Phased plan (each phase gated by the human-baseline scorecard)

- **Phase 0 — research** (researcher): McMurchie-Ebeling PathFinder + modern negotiated-congestion
  convergence (cost schedule, capacity model) adapted to 2-layer PCB; confirm why the crude
  attempt diverged and the exact fix. Output: algorithm spec.
- **Phase 1 — architect**: detailed design on the existing grid (capacity model, cost functions,
  iteration loop, escape integration, convergence + salvage fallback, determinism guarantees).
- **Phase 2 — developer + tester**: implement behind `engine="negotiated"`; unit tests; MEASURE vs
  human on all 3 boards. Gate: unconnected materially below 15/48/33 toward ~0, deterministic, no
  legality regression beyond Stage 2's remit.
- **Phase 3 — exact-geometry legality pass** (Stage 2): cut clearance errors toward human.
- **Phase 4 — full scorecard + ROI**; update docs; decide on Option C.

## 9. Definition of done

The scorecard shows unconnected ≤ ~5 and errors ≤ human on mitayi/zuluscsi/rp2040, deterministic,
with no board regressed. At that point TraceWise matches a human router on 2-layer boards — the
original portfolio claim, proven against shipped designs.

## Phase 0 COMPLETE — algorithm spec + prototype finding (2026-06-18)

Research: docs/research/PHASE0-negotiated-congestion-algorithm.md. Canonical McMurchie-Ebeling
cost = b(n)·(1+p_fac·p(n))·(1+h_fac·h(n)); iteration-0 free pass (p_fac=0); p_init 0.5, growth
1.3, cap ~20; h_fac 1.0 no decay; history updated once per iteration. Pinpointed the crude
pathfinder.py's 0/61 divergence to 3 root causes: RC1 too-aggressive schedule (0.5, ×1.8 → hard
wall by iter 5), RC2 stale reroute-selection snapshot, RC3 failed nets never re-queued (the
killer). Fixes applied to pathfinder.py (commit 36b661b); synthetic tests pass.

PROTOTYPE (corrected PathFinder, engine="pathfinder", on mitayi): the algorithm now iterates
CORRECTLY (no early hard-wall collapse) BUT did not finish in 40+ minutes on the 428k-cell mitayi
grid — zuluscsi (1.8M) would be many hours. DECISIVE for the architecture:
- The convergence fixes are right (foundation validated).
- RUNTIME is the binding Stage-1 constraint. COARSEN-THEN-REFINE is now MANDATORY (was "consider"):
  negotiate on a coarse ~0.2mm grid (16x fewer cells -> minutes), then refine the global routes to
  0.1mm locally. The architect (Phase 1) must design this as the core of Stage 1, plus a
  per-iteration expansion/time budget that preserves DETERMINISM (complete, never truncate).
NEXT: Phase 1 architect — design the coarsen-then-refine negotiated router on the existing grid,
with the corrected schedule, escape model, salvage fallback, and a runtime budget.
