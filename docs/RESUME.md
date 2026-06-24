# RESUME — TraceWise router (handoff, current as of 2026-06-24)

Single entry point. Read this, then the design docs it points to, then pick from "Next options".

## Project in one paragraph
TraceWise = open-source AI-assisted place & route for KiCad. Benchmark goal: **match a human router on
2-layer boards** — 0 unconnected, errors ≤ human — measured against the SHIPPED human originals in
`data/benchmark-boards/`. Detailed router is a grid A* (0.1mm, 2 layers, octile + vias, negotiated
congestion + bounded rip-up). Reproduce any board:
`taskset -c 0-9 .venv/bin/python scripts/_probe_route_human.py data/benchmark-boards/<board> /tmp/out`

## CURRENT SCORECARD (PM-verified 2026-06-24, default grid router + #4 emit + B4 castellated-pad fix)
| board    | OURS unc/err | HUMAN unc/err | notes |
|----------|--------------|---------------|-------|
| mitayi   | 48 / 88      | 0 / 0   | best = **41 / 73** via OPT-IN gridless_first (17-net set, mitayi-tuned) |
| zuluscsi | 13 / 40      | 0 / 5   | B4 fix cut errors 149→40 (−109); shorts 40→1 |
| rp2040   | 31 / 128     | 3 / 65  | B4 fix cut errors 152→128; ~71 of 128 are NON-routing (courtyards/footprint/silk) |
Default route is GRID; all gridless/coopt/etc. are OPT-IN flags (default None/False = this scorecard).

## WHAT'S DEFAULT (on for everyone, committed)
- Grid A* + bounded rip-up + negotiated congestion (deterministic — wall-clock truncation removed).
- **#4 exact-geometry track-endpoint emit** (clearance-class legality; mitayi 104→88).
- **B4 castellated-pad obstacle fix** (`bb.GetCenter()` for the obstacle rect) — −132 errors across
  boards, GENERAL (any Pico-module board). The biggest routing-quality win; see SCORECARD-ALL-BOARDS.md.

## WHAT'S OPT-IN / PROVEN-BUT-NOT-DEFAULT (committed behind flags; mitayi-tuned or partial)
- `gridless_nets`/`gridless_negotiate` (mitayi attempt-3 = 41/73): the gridless visibility-graph router
  (locality + super-cell congestion negotiation + 2-layer vias + multi-pin trees + QFN fanout-escape +
  ring-slot assignment). Mitayi-tuned; does NOT generalize (slow/no-op on other boards — see B3).
- `gridless_rescue` / `coopt`: grid-first rescue / cross-substrate co-opt. Region-validated; full-board
  is slow or net-neutral.
- `route_net_steered` + ring-slot assignment (`topo_assign.py`) + GCO min-cost-flow corridor allocation
  + CDR concurrent-channel infra: all built, the MECHANISMS validated, but none beat 41/73 (see below).

## THE mitayi 41→0 CEILING (mapped exhaustively — DO NOT re-grind without a new idea)
mitayi connectivity is at the architectural ceiling of staged routing. PROVEN: the gap is NOT capacity
(tightest cut ~56% F.Cu / <22% B.Cu; witness is crossing-free) — it is **global concurrent corridor
co-optimization**. The shared U3↔J3/J4 corridor is contended by +3V3 + escape nets; every approach maps
the SAME Pareto frontier, none dominating: power-first 30/113 | GCO 38/104 | escape 40/79 |
**attempt-3 41/73** | ring_slots 46/65. The CDR thesis (concurrent channel routing eliminates the per-net
`bcu_run_failed` wall) was CONFIRMED, but its spike regressed on a B.Cu-vs-F.Cu-jog implementation bug.
Closing 41→0 needs a COMPLETE concurrent detailed router (F.Cu jogs + jog-safe columns) — a real
multi-week chapter with an honest "may tie not beat" risk. See CONCURRENT-DETAILED-ROUTER.md.

## DESIGN DOC MAP (docs/design + docs/research)
- `research/SCORECARD-ALL-BOARDS.md` — the B3/B4 cross-board scorecard + error decomposition (READ FIRST).
- `research/HUMAN-ROUTING-TECHNIQUES.md` — surveyed human/pro techniques, ranked by impact.
- `research/INVENT-{human-mimicry,topological}-routability.md` — the two inventor proposals (gap = homotopy/ordering).
- `design/EXACT-GEOMETRY-ROUTER-ARCH.md` — #4 emit + the FAR gridless blueprint.
- `design/FAR-gridless-router-arch.md` — the full gridless arc (substrate→negotiate→vias→multi-pin→fanout).
- `design/TOPOLOGY-CLASS-ROUTING.md` — ring-slot/lane assignment + the 4-point frontier.
- `design/GLOBAL-CORRIDOR-OPTIMIZER.md` — GCO min-cost-flow (validated assignment).
- `design/CONCURRENT-DETAILED-ROUTER.md` — CDR channel router (thesis confirmed; the real next chapter).

## NEXT OPTIONS (ranked by EV)
1. **More error-axis wins (B4 follow-ups, HIGH EV / tractable):** rp2040 16 shorting_items + 20
   hole_clearance (via-at-goal-cell on QFN — the deferred "escape-before-via" / Fix-2 territory); mitayi
   32 hole_clearance; clearance (mitayi 27, zuluscsi 12). The error axis proved FAR more tractable than
   the connectivity tail (B4 = −132 errors from one fix).
2. **Complete the concurrent detailed router (CDR-2)** — the only path to mitayi 0/0, but multi-week +
   "may tie not beat" risk. Thesis validated; needs F.Cu jogs + jog-safe columns + engine integration.
3. **Placement co-design** — low for mitayi (placement already good); the lever for future boards.
4. **Bank** — current state is a strong, documented, all-pushed result.

## HOW TO WORK HERE (methodology that worked — IMPORTANT)
- **Probe before building** — every architecture this arc was spiked/validated before production; it
  repeatedly killed dead-ends in ~1 day at zero production cost.
- **Measure vs the HUMAN scorecard**, all 3 boards (B3 showed mitayi-only optimization missed the bigger
  cross-board error wins).
- **Decompose errors** board-inherent (courtyards/footprint/silk — router can't fix, also in human's
  counts) vs routing-introduced (shorts/clearance/hole) — attack only the latter.
- **Bounded discipline NON-NEGOTIABLE**: cap windows (≤12mm/8mm B.Cu), `_check_rss` 2GB abort, region-
  scope; the gridless fanout/rescue paths blow up (4–18GB / 25min) without caps.
- **PM independently re-verifies every subagent "win"** vs the correct baseline (NOT the 48 grid baseline
  — use attempt-3 41/73 for mitayi gridless claims); many subagent reports over-claimed (a false
  GATE-MET, multi-hour blowups) — independent re-measurement caught them.
- **Always `taskset -c 0-9`**; routes slow (mitayi ~165s, zuluscsi ~460s); run measurements in background.
- venv `/home/palgin/Business_projects/tracewise/.venv`; lint `ruff check .` (throwaway probe/spike/
  measure/invent scripts excluded in pyproject); tests `pytest -q`. networkx + Shapely installed.
- Commit per milestone; **push to GitHub periodically** (origin/main).
