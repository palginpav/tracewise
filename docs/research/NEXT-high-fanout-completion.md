# Research: NEXT — High-Fanout Power/Ground Completion on a Congested 2-Layer Board

**Status:** complete  
**Date:** 2026-06-18  
**Context:** Post-F3 state. zuluscsi residual = 65 unconnected (+3V0 37, GND 22, SCSI 6).  
**Supersedes / extends:** `docs/research/F3-high-fanout-power.md` (F3 shipped; that approach
  exhausted its first-order win; this survey covers the harder residual).

---

## 0. Situation Summary (what has been tried and why it matters)

The project has now shipped or falsified every "cheap" approach to the zuluscsi floor:

| Approach | Result |
|---|---|
| F3 synthesize +3V0 board-outline pour at low priority | SHIPPED. 84→65 (-23%). +3V0 56→37. GND 22 unchanged. |
| F2 stub-stitch pad→nearest pour | FALSIFIED. Only 2 GND pads isolated from pour; 20 are in disconnected islands. |
| F2' via-bridge at 0mm zone gaps | FALSIFIED. Islands separated by crossing traces, not clearance slivers. |
| F4 ratsnest-driven short-stub routing | Connectivity win (+18) but introduced 3 unremovable shorts (grid-quantization + pour interaction). REVERTED. |
| Placement flips/nudges | Zero gain; not a placement problem. |

**What remains:**
- **+3V0 37**: the +3V0 pour (F3) filled residual copper but fragmented into islands separated
  by GND traces. These 37 pads are NOT touching any +3V0 copper. They need routing or a new
  approach to extend the pour into those fragmented regions.
- **GND 22**: 10 are zero-gap zone↔zone island splits (islands separated by OTHER nets' crossing
  traces), 12 are far (median 44.9mm, max 93.4mm) cross-board connections. Neither is stub-stitchable.
- **SCSI 6**: 2 small signal nets; secondary priority.

**The honest diagnosis:** All 65 are routing problems requiring copper to route AROUND obstacles
on a board where GND copper occupies the majority of both layers. F4's short-stub approach proved
the routing model is correct and the connectivity gain is real — the blocker is shorts from
0.1mm grid quantization + pour interaction, not algorithmic correctness.

---

## 1. Problem Framing

**Goal:** Connect the residual 65 unconnected items on zuluscsi (2-layer, GND-poured both sides)
after all pour-side tricks are exhausted. The dominant sub-problem is routing copper AROUND obstacles
for high-fanout power/ground nets on a board where corridor space is brutally contended.

**Hard constraints:**
1. 2-layer board only (F.Cu + B.Cu).
2. Must not introduce new `shorting_items` — this is the gate F4 failed.
3. Must not regress mitayi (63 unconnected) or pic_programmer.
4. Deterministic (reproducible; zuluscsi runs ~530s of 600s cap).
5. Must work within TraceWise's pcbnew/Python architecture; no native-lib rewrites.

**Soft constraints:**
1. Prefer staged delivery: smallest-first lever first.
2. Reuse F0 extract_pours / pours.py infrastructure where possible.
3. Honest ceiling reporting preferred over silent partial wins.

**Known-not-wanted (pre-excluded):**
- Full inner power plane (4-layer requirement; out of scope for this feature).
- Redesigning the source board.
- F2/F2'/F4 approaches already measured as dead-ends.
- Route power last (already contra-indicated by multi.py order_nets rationale).

---

## 2. Five Candidate Approaches

### Approach A: Exact-geometry / sub-grid routing for the F4 stub problem

**What it is:** The F4 short-stub router was reverted solely because 0.1mm grid routing
produces stubs that are within DRC clearance of adjacent copper — the grid cell is 0.1mm
but DRC expects sub-grid exact geometry. Exact-geometry routing (sub-grid or gridless)
would place trace segments with exact floating-point coordinates, allowing the DRC clearance
to be satisfied precisely. This is the "gridless router" technique used by Freerouting
(shape-expansion) and commercial tools (Cadence Allegro "float" routing).

**What it solves for TraceWise:** The specific failure mode of F4 (3 unremovable shorts from
grid quantization). If the stub segments were emitted with exact coordinates (not snapped to
0.1mm grid) and verified against exact DRC geometry, the shorts would not appear. The
connectivity gain of +18 (from F4's measurement) is recoverable.

**Fit for TraceWise architecture:** Medium-High. The astar.py router walks grid cells but
emits track segments as grid-cell-center coordinates. The fix is at the emit layer: instead of
snapping segment endpoints to grid centers, emit the exact pad-to-track and track-to-via
coordinates from the KiCad footprint geometry. This is a targeted change to the segment-emit
code, NOT a full gridless router rewrite. The pour-interaction short (which appeared ~132-135mm
away, not on the stub) is harder — it requires knowing that placing a trace changes the zone
fill boundary, which causes a short elsewhere. This pour-interaction requires a post-emit refill
+ DRC check (already architected in the revert decision).

**Effort:** Medium. Two sub-problems: (1) sub-grid segment emit (1–2 days), (2) post-emit
refill+DRC check loop to catch pour-interaction shorts (1–2 days). Already understood from F4
investigation.

**Sources:**
- Altium Situs autorouter: topological routing at "any angle" rather than orthogonal-only
  (https://resources.altium.com/p/automated-pcb-routing-with-situs-topological-autorouter)
- Freerouting shape-expansion approach vs grid routing
  (https://github.com/freerouting/freerouting)

---

### Approach B: Pour-aware stub routing (emit → refill → DRC loop before commit)

**What it is:** The F4 short-stub approach with an additional safety gate: after emitting
each candidate stub, run `refill_zones` (pcbnew ZONE_FILLER) and re-run DRC on just the
affected region. If the refill introduces new shorts, reject that stub and mark it
permanently failed. This is the "post-emit short rejection" that was attempted in F4 but
failed because the 3 shorts could not be attributed to a specific stub (they were
pour-interaction, not direct overlap). The key insight: the rejection needs to be done
at the *single-stub* granularity, not at the batch level, and the trigger is *any* new
short anywhere on the board after refill.

**What it solves:** Catches pour-interaction shorts before they become permanent. Allows the
F4 connectivity win to ship without the quality regression.

**Fit for TraceWise architecture:** High. The `_run_pcbnew_script` pattern is already used
for fill and DRC. The loop is: for each candidate ratsnest pair → emit stub → refill → DRC →
if new shorts: un-emit stub → continue to next pair. This is a transactional routing pattern.

**Cost:** Runtime. Each candidate stub requires a `refill_zones` call (pcbnew), which is
significant (~seconds per call). With 37 +3V0 ratsnest pairs, this could add 1–3 minutes.
Given the 600s wall-clock cap, this might exhaust the budget. Mitigation: filter to
short ratsnest pairs only (< 8mm, the range that proved routable in F4's probe), reducing
candidates to ~12 calls.

**Effort:** Medium-Low. The F4 code (bridge/emit infrastructure) is already written and
understood. Adding a per-stub refill+DRC check loop is incremental. The hardest part is
ensuring the stub can be un-emitted atomically if DRC fails — requires board-state copy or
explicit segment removal by ID.

**Sources:**
- F4 investigation in ROUTING-COMPLETION-PLAN.md (empirical, not external)
- KiCad ZONE_FILLER API: `pcbnew.ZONE_FILLER(b).Fill(b.Zones())`
  (https://docs.kicad.org/8.0/en/pcbnew/pcbnew.html)
- KiCad DRC scripting: `pcbnew.DRC_ENGINE` (same pattern as existing `run_drc` in bridge.py)

---

### Approach C: Topology-aware ratsnest ordering (route shortest + easiest first)

**What it is:** The current F4 ratsnest-driven stub loop routes short ratsnest pairs in
arbitrary order. The improvement: sort pairs by a combined score:
(a) geometric distance (short pairs first — fewer grid cells, lower short risk),
(b) layer clearance (pairs where one layer has clearly more free corridor than the other),
(c) whether a stub can be routed on the layer opposite the GND pour (B.Cu is heavily poured
on zuluscsi; F.Cu has more free copper for +3V0 stubs).

This is the "ordering heuristic" used by commercial routers' cleanup passes and by the
Situs autorouter's final optimization pass, which selects pairs by score rather than
sequentially.

**What it solves:** Maximizes connections on the easy/short ratsnest pairs before corridor
space is consumed by harder, longer routes. On a board where 37 pairs have varying length,
the order determines how many can be connected before corridors close.

**Fit for TraceWise:** High and cheap. The ratsnest worklist already exists (F4 built it).
Sorting it by distance + estimated layer availability is a 20-line change. It doesn't
require architectural changes — just a better key function for the candidate list.

**Effort:** Low. One afternoon to implement the scoring key, 1 day to measure.

**Sources:**
- Situs autorouter: "prioritized" item selection strategy, scores calculated per round
  (https://resources.altium.com/p/automated-pcb-routing-with-situs-topological-autorouter)
- Freerouting prioritized selection (https://github.com/freerouting/freerouting)
- Big Mess o' Wires: "autorouters treat all paths equally" — the manual insight is to order
  power/ground paths first and shortest-connection-first
  (https://www.bigmessowires.com/2019/03/27/pcb-autorouting/)

---

### Approach D: FindIsolatedCopperIslands API for GND island diagnosis and targeted bridging

**What it is:** KiCad's `CONNECTIVITY_DATA.FindIsolatedCopperIslands()` returns the specific
filled-polygon islands that are electrically disconnected from the rest of their net's copper.
For GND, this would identify exactly which of the 85-polygon pour regions are disconnected from
the main GND net. Currently TraceWise uses `IsConnectedOnLayer` (which misidentified the GND
problem as F2 falsified) and DRC ratsnest (which correctly counts 22). `FindIsolatedCopperIslands`
sits between these: it identifies disconnected pour REGIONS (not pads), which is the correct
granularity for the GND 10 zero-gap island-split problem.

**What it solves for GND 22:**
- For the 10 zero-gap island splits: `FindIsolatedCopperIslands` would enumerate the isolated
  GND pour polygons. Each isolated polygon is a fill region cut off by a crossing trace. The
  solution is to route a GND BRIDGE: a trace segment on the layer where the crossing trace does
  NOT exist, connecting the two sides of the gap. This is the correct model (not a via at the
  gap, which F2' falsified).
- For the 12 far cross-board GND connections: these require actual routing (traces or vias),
  not just island identification. `FindIsolatedCopperIslands` identifies the islands but the
  solver is still the router.

**Fit for TraceWise:** High for the island identification; medium for the bridge routing.
The API is exposed in pcbnew's CONNECTIVITY_DATA class (confirmed in KiCad doxygen docs).
It returns island indices within a zone's filled poly list, matching the `PourPoly.island_id`
field already in `pours.py`.

**Effort:** Medium. Two sub-parts: (1) wrap `FindIsolatedCopperIslands` in a pcbnew script
(1 day, similar to F0 extract_pours pattern), (2) for each isolated island, find the nearest
main-net copper on the opposite layer and route a bridge trace (1–2 days). Harder than approach C.

**Sources:**
- KiCad `CONNECTIVITY_DATA.FindIsolatedCopperIslands` doxygen
  (https://docs.kicad.org/doxygen-python/classpcbnew_1_1CONNECTIVITY__DATA.html)
- KiCad PCB Editor docs §Zones — island removal options
  (https://docs.kicad.org/8.0/en/pcbnew/pcbnew.html)
- Birologue stitching layers between copper zones: via stitching to prevent floating planes
  (https://biro.wordpress.com/2011/11/25/stitching-layers-between-copper-zones-in-kicad/)

---

### Approach E: Honest 2-layer ceiling detection and "needs 4 layers" reporting

**What it is:** A ceiling detector that, for a given ratsnest pair, determines whether any
legal path exists between the two endpoints given the current copper occupancy. This is
equivalent to running A* with zero cost per step and checking reachability — not routing,
just connectivity. If no path exists on either layer (and via-layer-switch is also blocked),
the connection is provably unroutable on 2 layers. Report these as `UNROUTABLE_ON_2_LAYER`
in the DRC output.

**What it solves:** Product value: instead of silently leaving unconnected items, TraceWise
reports "this net requires additional copper layers." This is honest reporting and differentiates
the tool from dumb autorouters. It also tells the architect: these N connections are not bugs
in the router — they are physical layer-count constraints.

**How to implement:** After routing completes, for each remaining unconnected ratsnest pair:
run a BFS/Dijkstra on the current grid (including hard-blocked cells, NO escape allowance)
between the two pad cells with via-layer transitions. If no path is found → provably unreachable
on 2 layers → tag it. Runtime: BFS on a 2-layer grid is O(L×H×W) = O(2 × 800 × 800) ≈ 1.3M
cells per check; at 65 checks this is ~85M cell visits, negligible (<1s).

**Fit for TraceWise:** High. The grid infrastructure already supports exactly this check
(route() with cap=∞ and escape=0). The ceiling detector is a diagnostic run of the existing
router with modified parameters.

**Effort:** Low-Medium. 1 day to implement the reachability check; 1 day to add the
reporting output (DRC-style output in route_board_engine). Completely non-destructive —
it does not change any routing, only labels residual items.

**Sources:**
- Altium "Practical PCB Design Tips for an Unroutable Board": explicit acknowledgment that
  layer count is a ceiling factor (https://resources.altium.com/p/practical-pcb-design-tips-unroutable-board)
- 2-layer vs 4-layer design guidance: GND/power planes on inner layers free outer layers
  (https://www.nwengineeringllc.com/article/2-layer-vs-4-layer-pcbs-whats-the-right-choice.php)
- allpcb.com PDN best practices: "insufficient layers ... reasons a board may be unroutable"
  (https://www.allpcb.com/allelectrohub/power-distribution-network-design-in-double-layer-pcbs-best-practices)

---

## 3. Landscape Table

| # | Approach | What it solves | Fit for TraceWise (reuses existing infra?) | Effort | Fit Score | Verdict | Source |
|---|----------|---------------|-------------------------------------------|--------|-----------|---------|--------|
| A | Exact-geometry / sub-grid segment emit | F4 grid-quantization shorts (3 unremovable) — unlocks the +18 connectivity win | Medium-High: targeted change to emit layer in astar.py / bridge.py; gridless is NOT needed | Medium (3–4 days) | 4/5 | **Recommend** | Altium Situs, Freerouting |
| B | Pour-aware per-stub refill+DRC loop | F4 pour-interaction shorts — catches all short types before commit | High: uses existing _run_pcbnew_script pattern; incremental to F4 code | Medium-Low (2–3 days) | 4/5 | **Recommend** | KiCad ZONE_FILLER API |
| C | Topology-aware ratsnest ordering (shortest+layer-clear first) | Maximizes easy connections before corridors close; orthogonal to A+B | High: 20-line change to ratsnest worklist sort key | Low (1 day) | 4/5 | **Recommend** (do first) | Situs prioritized selection, Freerouting |
| D | FindIsolatedCopperIslands API for GND island bridging | GND 10 zero-gap island splits — identifies correct bridge targets | High: PourPoly.island_id already in pours.py; API confirmed in KiCad doxygen | Medium (2–3 days) | 3/5 | Consider | KiCad CONNECTIVITY_DATA doxygen |
| E | 2-layer ceiling detector + "needs 4 layers" reporting | Honest unroutability classification; product differentiator | High: reuses route() with cap=∞ as reachability check; no routing changes | Low-Medium (1–2 days) | 5/5 | **Recommend** (non-destructive) | Altium, allpcb.com, nwengineering |

---

## 4. Recommendations

### 4.1 Smallest-first lever: Approach C + B (ratsnest ordering + per-stub refill gate)

**Rationale:** F4 already proved the routing model is correct and the connectivity gain is
real (+18 on +3V0 37→~19). The only blocker was 3 unremovable shorts. The per-stub
refill+DRC loop (approach B) is the minimal fix: run the same F4 loop but transactionally
gate each stub. Approach C (sorted ratsnest order) is free since the ratsnest worklist
already exists.

**Concretely in the codebase:**
1. The F4 ratsnest-driven stub module (reverted but understood) is re-opened with two changes:
   a. Sort the candidate ratsnest pairs by `distance_mm ASC` then `layer_with_more_free_copper FIRST`.
   b. After emitting each stub: call `pcbnew.ZONE_FILLER(board).Fill(board.Zones())` and
      `run_drc(board)`, compare short count. If new shorts appear, call
      `board.RemoveNative(seg)` (or equivalent board-state rollback) and skip this pair.
2. The loop processes short pairs first (< 8mm, the range F4 proved routable via probe).
3. Wire into `route_board_engine` as a post-route pass, same slot F4 occupied.

**Expected gain from C+B alone:** +3V0 37 → ~19–22 (recovering most of F4's +18 without
the shorts). GND 22 unchanged (GND island problem is a separate lever, approach D).

**Risk:** Per-stub refill is runtime-costly. With ~12 short candidates (< 8mm), ~12 pcbnew
script invocations at ~2–5s each = 24–60s additional. This stays within the 600s cap (current
run ~530s). Mitigation: run the loop only on short pairs and bail early if time budget
nears 580s.

### 4.2 Bigger lever: Approach A (sub-grid emit)

**Rationale:** The root cause of the 3 shorts is 0.1mm grid snap. If segment endpoints are
emitted with exact coordinates (pad centroid to via center, not grid-snapped), the clearance
violations disappear. This is a more fundamental fix than per-stub DRC gating.

**Concretely in the codebase:**
- In the segment-emit path (bridge.py's track-add code or the F4 stub emitter), when placing
  a track segment from pad to via: compute the exact pad centroid (from pcbnew footprint
  geometry) and exact via center, and use those coordinates directly instead of grid-cell
  centers. The A* path provides the route topology (which cells, which layer, where to via);
  the emit translates cell coordinates to exact world coordinates.
- The pour-interaction short (appearing at a distant location after refill) requires approach B
  as a safety net regardless of A — exact emit alone does not prevent topological pour changes.

**Combined A+B:** eliminates both short types. Expected outcome: F4 can ship cleanly,
recovering +18 on +3V0 (37 → ~19) and potentially additional gain if more than 12 short
pairs become routable with exact geometry.

### 4.3 GND 22 lever: Approach D (FindIsolatedCopperIslands + bridge routing)

**Rationale:** The correct model for GND 22 is now known: 10 are pour-island splits (one
GND copper region cut off from the main GND fill by a crossing trace on the same layer);
12 are far cross-board connections. `FindIsolatedCopperIslands` identifies the 10 island
splits programmatically via pcbnew's own topology analysis. For each isolated island:
- Find the crossing trace on F.Cu that creates the split.
- Route a GND bridge on B.Cu connecting the two sides of the split (opposite layer to the
  crossing trace). This avoids the crossing and connects the islands.
- The 12 far connections require full routing with the main router (not a stub pass) — they
  are genuinely hard (44.9mm median, cross-board, through a congested grid).

**Concretely:** A new `find_gnd_islands(board)` function calls `FindIsolatedCopperIslands`
via the `_run_pcbnew_script` pattern, returns island indices + pour polygon geometries. For
each island, compute the nearest opposite-layer corridor and route a bridge. Expected gain:
GND 22 → ~12 (the 10 island-split bridges), with the 12 far connections remaining.

**Effort:** Medium. Not the smallest lever but addresses a real structural problem in GND.
Sequence after B: GND stitching is independent of the +3V0 stub work.

### 4.4 Product feature: Approach E (2-layer ceiling detector)

**Rationale:** This is non-destructive and adds product value. After routing completes,
run reachability (route with cap=∞, escape=0, no time limit) for each remaining unconnected
pair. Tag unreachable pairs as `UNROUTABLE_2LAYER`. Report in CLI output:
  "3 connections require additional copper layers (GND: 2, +3V0: 1)"

**This is a genuine product differentiator.** No open-source autorouter reports this
honestly. TraceWise can distinguish "router didn't find a path" from "no path exists on
2 layers."

**Implementation:** Add a `detect_ceiling(grid, unconnected_pairs)` function in multi.py
or a new ceiling.py. For each pair, call `route()` with `cap=None` (or a very large cap),
`escape=0`, `history_factor=0`. If `RouteResult.ok == False` after full expansion: provably
unroutable. Wire into `route_board_engine` as a final diagnostic pass.

---

## 5. Honest 2-Layer Ceiling for zuluscsi Residual 65

### Estimation methodology

The F4 probe (12/24 short ratsnest pairs routable via opposite layer) gives an empirical
partial answer: ~half of +3V0's short pairs are routable in principle (route exists),
and ~half are not (grid-blocked). Extrapolating:

| Net group | Total residual | Estimated routable | Estimated genuine ceiling |
|---|---|---|---|
| +3V0 37 | Short pairs (~24 at <8mm): ~12 routable (F4 probe). Long pairs (~13 at >8mm): harder; maybe 3–5 routable. | ~15–17 connectable | ~20–22 genuinely unroutable on 2 layers |
| GND 22 | 10 island splits: likely bridgeable via opposite layer (~8). 12 far: 3–4 may find a path; ~8 blocked by congestion. | ~11–12 connectable | ~10 genuinely unroutable |
| SCSI 6 | Small signal nets. Both 5-pad nets. Prior rip-up reached ~3 unconnected each. | ~2–3 connectable | ~3 genuinely unroutable |
| **Total** | **65** | **~28–32 connectable** | **~33–37 genuine ceiling** |

**Interpretation:** A realistic floor for zuluscsi on 2 layers is **~28–35 unconnected**,
not zero. The remaining 30–35 connections would require either (a) 4-layer board (GND/power
inner planes free both outer layers for signal routing) or (b) re-layout (moving components,
which TraceWise intentionally does not do on human placements).

**How to detect and report each category:**

1. **Provably unroutable (ceiling):** BFS reachability check on the final grid (approach E).
   If BFS finds no path even with cap=∞ → `UNROUTABLE_2LAYER`. These do NOT need 4-layer
   universally — some are bridgeable with re-routing earlier nets (rip-up can free corridors).

2. **Routing failure (not ceiling):** BFS finds a path but A* failed within its cap →
   `ROUTE_FAILED_CONGESTION`. These are a router quality issue, not a physical impossibility.
   Rip-up, ordering changes, or wider search could recover them.

3. **Pour-island structural (GND):** `FindIsolatedCopperIslands` identifies them →
   `POUR_ISLAND_REQUIRES_BRIDGE`. These are GND topology issues, solvable on 2 layers
   with bridge traces.

**Reporting in CLI output (suggested format):**
```
  Routing complete: 51/116 nets routed. Unconnected: 65.
    +3V0:  37 unconnected  (est. 20 genuine 2L ceiling, 17 router-recoverable)
    GND:   22 unconnected  (10 pour-island bridges needed, 12 congestion/ceiling)
    SCSI:   6 unconnected  (router congestion)
  2-layer ceiling: ~30–35 connections require 4+ layers or re-layout.
```

---

## 6. Where Each Approach Plugs Into the Codebase

| Approach | Entry point | Key functions touched |
|---|---|---|
| C (ordering) | `route_board_engine` → F4 stub loop pre-sort | New `_score_ratsnest_pair(pair, grid)` in the F4 stub module |
| B (per-stub DRC gate) | F4 stub loop: after each `board.Add(seg)` | New `_refill_and_check_shorts(board_path) → bool` via `_run_pcbnew_script` |
| A (sub-grid emit) | bridge.py track-emit / F4 segment emit | Replace grid-cell-center coords with exact pad/via centroid coords |
| D (GND island bridge) | New `find_gnd_islands(board)` function in pours.py | Wraps `CONNECTIVITY_DATA.FindIsolatedCopperIslands`; feeds bridge router |
| E (ceiling detector) | `route_board_engine` final pass | New `detect_2layer_ceiling(grid, ratsnest_pairs)` in multi.py or ceiling.py |

---

## 7. Honest Gaps

- **F4 source code state:** F4 was reverted (commit rollback). The bridge-route code and
  ratsnest-worklist infrastructure are understood from the investigation but may need to be
  re-implemented. The design is clear; re-implementation from the investigation notes should
  not require re-discovery.

- **Per-stub refill runtime (approach B):** Actual pcbnew ZONE_FILLER runtime per call on
  zuluscsi is not measured (only the total routing time is known). If each fill takes >5s,
  12 candidates = 60s, approaching the budget limit. A mitigation is to only run the fill
  check on candidates that change the fill boundary (i.e., stubs in or adjacent to a zone
  outline) — pure copper-area stubs that don't intersect zone outlines cannot cause
  pour-interaction shorts.

- **FindIsolatedCopperIslands exact API signature:** The doxygen docs confirm the method
  exists. The exact Python call signature (whether it modifies zones in-place or returns
  a result vector) is not verified by a live pcbnew script test. Pattern: add a probe script
  before implementing.

- **Situs/Altium source:** Situs is proprietary; only the architecture description is
  publicly available, not source code. The ratsnest ordering improvement (approach C) is
  derived from first principles consistent with what Situs describes, not a direct copy.

- **zuluscsi ceiling estimate:** The 30–35 figure is a projection from the F4 probe sample
  (12/24 short pairs routable). The actual ceiling can only be determined precisely by
  running approach E (ceiling detector) on the final grid. The estimate is a lower-bound
  confidence region; the actual figure could be as low as 25 (if long-pair routing yields
  more than expected) or as high as 40 (if GND's 12 far connections are all blocked).

---

## 8. Recommended Next Agent

**Developer** — the research is complete and the architecture is clear enough to implement
directly. No new design decisions are required. Suggested sequence:

1. **Re-implement F4 stub module** (or un-revert) with approach B's per-stub refill+DRC gate.
2. **Add approach C** sorting to the candidate list (30 minutes, no risk).
3. **Measure** on zuluscsi. Gate: zero new shorts, +3V0 < 30. If passes, ship.
4. **Add approach E** ceiling detector as a final diagnostic pass (non-destructive). Ship.
5. **Then** approach D (GND island bridges) as a separate feature, gated independently.
6. **Then** approach A (sub-grid emit) as a deeper quality improvement.

Approach C+B together is the smallest coherent unit that can recover F4's +18 win without
the quality regression. That is the next concrete step.

---

## 9. Sources

- [KiCad PCB Editor 8.0 Documentation — Zones](https://docs.kicad.org/8.0/en/pcbnew/pcbnew.html)
- [KiCad CONNECTIVITY_DATA doxygen — FindIsolatedCopperIslands](https://docs.kicad.org/doxygen-python/classpcbnew_1_1CONNECTIVITY__DATA.html)
- [Hackster.io — Two-Layer PCB Routing Strategies](https://www.hackster.io/news/pcb-friday-two-layer-pcb-routing-strategies-and-tips-1c6b8cfcf5d3)
- [Altium Situs Topological Autorouter](https://resources.altium.com/p/automated-pcb-routing-with-situs-topological-autorouter)
- [Freerouting GitHub](https://github.com/freerouting/freerouting)
- [Big Mess o' Wires — PCB Autorouting (completion rate, ordering insights)](https://www.bigmessowires.com/2019/03/27/pcb-autorouting/)
- [allpcb.com — PDN Design in Double Layer PCBs Best Practices](https://www.allpcb.com/allelectrohub/power-distribution-network-design-in-double-layer-pcbs-best-practices)
- [Altium — Practical PCB Design Tips for an Unroutable Board](https://resources.altium.com/p/practical-pcb-design-tips-unroutable-board)
- [NW Engineering — 2-Layer vs 4-Layer PCBs](https://www.nwengineeringllc.com/article/2-layer-vs-4-layer-pcbs-whats-the-right-choice.php)
- [Birologue — Stitching Layers Between Copper Zones in KiCad](https://biro.wordpress.com/2011/11/25/stitching-layers-between-copper-zones-in-kicad/)
- [Sierra Circuits — Autorouting in KiCad using FreeRouting Plugin](https://www.protoexpress.com/blog/how-to-autoroute-pcb-layout-in-kicad-using-freerouting-plugin/)
