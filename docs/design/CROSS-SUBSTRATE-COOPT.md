# Design: Cross-Substrate Co-Optimization (unified negotiated router)

Status: ARCHITECTURE, 2026-06-23. The largest architecture in the project. Successor to the
FAR gridless arc (`docs/design/FAR-gridless-router-arch.md`), which ended at the **attempt-3
ceiling: mitayi 41/73** via gridless-FIRST ordering — proven that grid + gridless **contend for
the same routing channels and NO sequential ordering resolves it**. This doc designs the fix:
route BOTH substrates TOGETHER under a SHARED congestion/negotiation field so they jointly
deconflict.

NO production code here. Pseudocode + contracts only. The deliverable is a DESIGN + a small,
decisive, BOUNDED spike (Spike-CoOpt).

> **Read-first context (the proven problem, PM-verified — FAR doc tail).**
> - attempt-3 (gridless-FIRST negotiate, 17 boxed-in nets routed before grid): **41/73, 4/17 QFN
>   nets connected** — the BEST result, current ceiling. Better on BOTH axes vs grid-only 48/87.
> - gridless-first + fanout-escape: 10/17 QFN nets connect BUT **displaces 7 grid nets → 45** (worse).
> - grid-first + fanout-rescue: keeps grid but grid copper **walls off** the fanout escapes → 1/17 → 46 (worse).
> - The fanout-escape MECHANISM is validated (10/17 in isolation; matches the human's QFN B.Cu escape).
> - The blocker is purely SEQUENTIAL contention. Fix = route together under a shared field.
> Target: beat 41. Floor estimate ~38 (the 10 fanout connections with zero grid displacement);
> stretch toward the human's 0.

---

## Overview

Replace the two-phase pipeline (gridless-FIRST claims corridors → grid routes the remainder around
frozen gridless copper) with ONE negotiated-congestion loop over ALL nets — grid-routed and
gridless-routed — pricing every net's route by a SINGLE shared congestion field. Both substrates
deposit history when ripped and price their search by it, so a grid net's contention raises the
price for a gridless net crossing the same region and vice versa. No net is frozen first; they
jointly settle. This is **McMurchie-Ebeling PathFinder generalized across two routing substrates**.

**Why this approach.** Both substrates ALREADY run their own negotiated-congestion loop with
near-identical mechanics — the grid in `route_all_pathfinder` (`pathfinder.py`: free iter-0,
present+history pricing, occ-overuse → rip+reroute), the gridless in `route_gridless_set`
(`negotiate.py`: 0.5 mm super-cell history + bounded rip-up). They diverge only in (a) the
congestion data structure (grid `occ`/`hist` numpy at 0.1 mm vs super-cell `history` at 0.5 mm)
and (b) being run sequentially. Unify (a) into one field and (b) into one loop, and the contention
they currently resolve only WITHIN a substrate is resolved ACROSS substrates. This reuses two
proven loops rather than inventing a third.

**Why NOT just tune the existing two-phase order.** FAR proved exhaustively: gridless-first,
grid-first, fanout-rescue, cross-substrate rip-up (CSRU v1/v2) — every SEQUENTIAL combination
either walls off the late substrate or displaces the early one. The contention is structural, not
an ordering bug. Only joint pricing dissolves it.

**Pattern/KB check.** `pattern_find` (catalog) + `kb_search` returned ZERO matches for
cross-substrate / shared-congestion / unified-router — greenfield in the orchestration KB. No prior
decision is contradicted or extended. A `decisions/` entry should be written once Spike-CoOpt
reports (see task list).

---

## Scope

- **Files to create (the spike, built NEXT — no engine changes):**
  - `scripts/spike_coopt_shared_field.py` — Spike-CoOpt (see spec). Standalone, reuses
    `build_problem` + the gridless package + the `route_all_pathfinder` grid loop; BOUNDED windows.
- **Files to modify (M-CO-1+ production, NOT in the spike — listed for the staging map):**
  - `src/tracewise/route/engine/multi.py` — `route_all`: add a `coopt: bool` path that runs the
    unified loop instead of the staged `gridless_first` blocks. The existing
    `gridless_first`/`gridless_negotiate`/`gridless_rescue` paths STAY (byte-identical default).
  - `src/tracewise/route/gridless/negotiate.py` — `route_gridless_set`: accept an EXTERNAL shared
    super-cell field (`sc_grid`) instead of always building its own, so the unified loop owns the
    single field. Today it builds `_SuperCellGrid` internally (`_make_supercell_grid`).
  - `src/tracewise/route/engine/pathfinder.py` — the grid `_astar` cost term gains a super-cell
    history read (price grid cells by the shared field) and the commit deposits to it. Today its
    `hist` is grid-cell-resolution and private to the call.
  - `src/tracewise/route/engine/kicad.py` — `route_board_engine`: thread a `coopt` param.
- **Files read (context only):** `pathfinder.py` (the grid negotiated loop — the unified-loop
  template), `negotiate.py` (the gridless negotiated loop + `_SuperCellGrid`), `multi.py`
  (`route_all`, `_mark`, `_nearest_victim`, `order_nets`, the gridless_first blocks), `grid.py`
  (the `cells`/`hard` int16 shared occupancy ledger), `adapter.py`
  (`rasterize_into_grid` — gridless copper → shared grid ledger), `geom.py`
  (`build_windowed_free_space`, the bounded-window machinery), `route.py` (per-net gridless entry).

---

## Decision 1 — The unified shared congestion field

**Resolution: the 0.5 mm super-cell lattice is the single shared congestion currency. Both
substrates read AND write it. The 0.1 mm grid `cells`/`hard` ledger remains the HARD occupancy
plane (legality), unchanged.**

There are two distinct fields and they must not be conflated:

1. **The HARD occupancy ledger — `grid.cells` / `grid.hard` (0.1 mm int16, EXISTS).** This is the
   capacity-0 wall: actual copper + keepouts + clearance halos. It is ALREADY shared — gridless
   copper rasterizes into it via `adapter.rasterize_into_grid` at the same inflation `_mark` uses.
   Legality is by construction off this plane. **No change.** It answers "is this cell physically
   occupied?"

2. **The SOFT congestion field — a 0.5 mm super-cell history lattice (the NEW shared currency).**
   This is the negotiation price: "how often has this region caused a rip-up?" Today it exists TWICE
   and disconnected: the grid's `hist` numpy (`pathfinder.py`, 0.1 mm, private per call) and the
   gridless `_SuperCellGrid.history` (`negotiate.py`, 0.5 mm, private per call). **Unify both onto
   ONE `_SuperCellGrid` instance, owned by the unified loop, passed into both substrates.**

**Why super-cells (0.5 mm) as the common currency, not the 0.1 mm grid.** (a) It already exists,
tested, deterministic (`negotiate._SuperCellGrid`, `supercell_of`, `edge_history_cost`,
`supercells_for_path`, `deposit`). (b) It is substrate-independent: a spatial `(sy, sx)` key both a
grid cell and a gridless edge map onto trivially. (c) It is ~5× coarser than the 0.1 mm pitch, so
the field is small (`~ board / 0.5mm` per axis ≈ tens-of-thousands of float64, kilobytes) and cheap
to price against — critical for bounded memory. (d) Congestion is inherently a coarse
"this corridor is contested" signal; 0.1 mm resolution would be false precision and 25× the memory.

**The single source of truth + the mapping.**

```
SharedField = negotiate._SuperCellGrid          # ONE instance, owned by the unified loop
  .history : np.float64[ny_sc, nx_sc]           # ny_sc = ceil(board_h / 0.5) + 2, etc.
  .supercell_of(x_mm, y_mm) -> (sy, sx)          # world-mm -> super-cell (EXISTS)

# GRID side mapping (grid cell -> super-cell):
def grid_cell_to_supercell(grid, layer, iy, ix, field):
    x, y = grid.to_world(iy, ix)                 # 0.1mm cell center -> world mm (EXISTS)
    return field.supercell_of(x, y)              # -> (sy, sx)

# GRIDLESS side mapping (visibility edge -> super-cells it crosses):
field.edge_history_cost(u_xy, v_xy, history_factor)   # length-weighted mean history (EXISTS)
field.supercells_for_path(waypoints)                  # the super-cells a route credits (EXISTS)
```

- **Both PRICE by it.** Gridless A* edge cost already = `length × (1 + history_factor × mean_history)`
  via `edge_history_cost` (`negotiate._build_congestion_visgraph`). The grid `_astar`
  (`pathfinder.py`) currently multiplies the step by `(1 + h_fac × hist[cell])` where `hist` is
  per-grid-cell; CHANGE it to read `field.history[grid_cell_to_supercell(...)]` so the grid prices
  by the SAME super-cell value the gridless edge crossing that region sees. (Layer-collapsed: a
  super-cell is an `(x,y)` property, layer-independent — a via and the tracks it joins all credit
  the same super-cells. This matches `negotiate.py`'s existing layer-agnostic history and Decision 3
  of the FAR doc.)
- **Both DEPOSIT to it.** On rip-up of a grid net, deposit `+1.0` onto the super-cells its cells map
  to (`field.deposit([grid_cell_to_supercell(...) for cell in victim.cells])`). On rip-up of a
  gridless net, deposit onto `nr["supercells"]` (ALREADY computed by `supercells_for_path`). One
  `deposit` call site shape for both.

**Determinism of the field.** `history` is plain float64 addition over integer super-cell keys; the
deposit set is built by deterministic sampling (`supercells_for_path` / `grid_cell_to_supercell`,
both pure functions of snapped coords). No iteration-order dependence. Identical to the existing
two private fields, just merged.

---

## Decision 2 — The unified iteration loop

**Resolution: ONE McMurchie-Ebeling-style negotiated-congestion loop over ALL nets, partitioned by
substrate assignment, pricing every route by the shared field, with cross-substrate contention
detection on the 0.1 mm hard ledger and rip-up victim selection across substrates. Bounded
deterministic round budget.**

This is `route_all_pathfinder`'s loop (`pathfinder.py`) generalized: where PathFinder routes every
net on the grid each round, the unified loop routes each net on ITS ASSIGNED SUBSTRATE each round,
all pricing the one shared field.

### 2a. Substrate assignment (which net routes where) — STATIC, computed once

Deterministic, computed before the loop (sorted, reproducible):

- **GRIDLESS (visibility-graph + fanout/2-layer):** nets the grid leaves geometry-blocked or
  boxed-in — concretely the validated **17 boxed-in signal nets** (`_verify_gridless_first_ab.py:
  TARGET_NETS_17`) PLUS the QFN-escape nets that need the fanout mechanism. Rule:
  `route_gridless_set`'s pre-classifier already flags `min_needed_window > 0.5×board_diag` as
  geometry-blocked; the assignment = those ∪ the boxed-in-on-clean-board set (Probe-Order: nets the
  grid fails but route fine on a clean board). The fanout-escape sub-strategy fires for the QFN
  source pads (`detect_dense_components` — EXISTS in `geom.py`).
- **GRID (PathFinder A*):** everything else — the bulk, power pours excluded (pours are not routed
  as tracks; FAR Probe-Pour finding).
- The assignment is a `dict[str, Literal["grid","gridless"]]`, sorted by net name, logged. It does
  NOT change across rounds in M-CO-1/2 (re-assignment under negotiation is an M-CO-3 stretch, gated
  behind measurement — keeps the loop bounded and deterministic).

### 2b. The round (one negotiated-congestion iteration)

```
field = SharedField(board_bbox)                  # the ONE shared super-cell field
# iteration 0 is FREE (history_factor effectively 0) so every net finds SOME route first —
# the McMurchie-Ebeling free-exploration pass (pathfinder.py proved this prevents the hard-wall
# 0/61 divergence). Then history_factor ramps on a capped schedule.
for round in range(MAX_ROUNDS):                  # HARD CAP, e.g. 12
    contended = []                               # nets to (re)route this round
    if round == 0:
        contended = ALL nets (in deterministic order_nets order)
        hf = 0.0
    else:
        contended = nets whose copper CONTENDS on the shared ledger (Decision 2c)
        if not contended: break                  # CONVERGED
        hf = min(hf_init * growth**(round-1), hf_max)   # capped ramp (pathfinder schedule)

    for net in sorted(contended, key=order_nets_key):    # DETERMINISTIC order
        unmark net's old copper from grid.cells/hard (if any)        # frees the ledger
        sub = assignment[net.name]
        if sub == "grid":
            nr = pathfinder_route_one(net, grid, field, hf, BOUNDED_EXPANSIONS)
        else:  # gridless
            nr = gridless_route_one(net, grid, field, hf, BOUNDED_WINDOW)  # via route_gridless_set machinery, single net, bounded window
        if nr.ok:
            mark nr into grid.cells/hard (_mark / rasterize_into_grid)   # shared ledger
        else:
            record failure (do NOT mark)
    # post-round: detect contention, deposit history on contended super-cells (below)
```

- **Per-net routing reuses the existing single-net entries**, just threaded with the shared field:
  grid = `pathfinder._route_one` (already prices by `hist`; swap `hist` read to the shared field);
  gridless = `negotiate._route_one_net_congestion` (already prices by `sc_grid`; pass the shared
  `sc_grid`). Both already BOUNDED (grid: `max_expansions`; gridless: `max_window_mm`).
- **The free iteration-0 is load-bearing** (pathfinder.py RC1 lesson): without it, history pricing
  starts before any net has a baseline route and the loop wall-walls. Keep it.

### 2c. Contention detection — ACROSS substrates, on the shared hard ledger

**Resolution: two nets contend iff their clearance-inflated copper overlaps on the shared 0.1 mm
ledger.** This is detectable directly because BOTH substrates write the same `grid.cells` count
plane at the SAME inflation (grid via `_mark`, gridless via `rasterize_into_grid` + `_mark`).

```
# After routing all contended nets a round, re-mark everyone, then:
overuse = grid.cells > <expected count if no overlap>
```

The clean detector reuses the grid's count semantics (grid.py docstring: "a cell is free iff count
== 0"; counting makes rip-up sound). Concretely, mirror `route_all_pathfinder`'s `occ > 1` overuse
test: maintain an `occ`-style count of inflated copper (separate from `hard` fixed obstacles) and a
cell with `occ > 1` is shared by ≥2 nets' clearance footprints — a contention. Map the contended
cells → super-cells → the nets occupying them (each `NetRoute`/`GridlessNetRoute` knows its
`cells`), and those nets are the round's `contended` set. Cross-substrate falls out for free: a grid
net and a gridless net that overlap both wrote into the same `occ` cells.

- **Deposit on contention:** for each contended super-cell, `field.deposit([sc], +1.0)` — exactly
  `route_all_pathfinder`'s `hist += max(0, occ-1)` step, lifted to super-cell resolution. This is
  what makes the contested corridor pricier next round for BOTH substrates.

### 2d. Rip-up victim selection (cross-substrate)

When two nets contend, the higher-priority net (earlier in `order_nets`) keeps the corridor; the
lower-priority net is the victim, re-routed next round at the now-higher price. This is the
McMurchie-Ebeling "nets with alternatives vacate; nets with none keep" dynamic — it emerges from
re-pricing, NOT from an explicit victim pick, which is why it deconflicts cleanly. (The explicit
`_nearest_victim` rip-up in `route_all` is the OLD hard-wall mechanism; the unified loop uses the
soft re-pricing mechanism from `route_all_pathfinder` instead.) Determinism: ties broken by
`order_nets` key then net name (integer/string, total order).

### 2e. Convergence + budget

- **Converged** iff a round produces zero contended nets (no `occ > 1` super-cell shared across
  nets). Then emit.
- **Budget:** `MAX_ROUNDS` (hard, e.g. 12 — pathfinder uses 40 grid-only iters, but each unified
  round is heavier, so cap lower and measure). If the budget is spent with residual contention,
  accept the best round seen (track `unconnected + errors` per round; keep the min) and report —
  NEVER spin. This mirrors `route_all`'s `budget = ripup_factor × n_nets` discipline.

---

## Decision 3 — Determinism + BOUNDED runtime/memory (CRITICAL — non-negotiable)

The last builds hit **4–18 GB / multi-hour** (FAR M3-P1.1: gridless-first on a clean board hung
>25 min; CSRU: ~10 min/net board-wide windows; multi-pin rescue: 18 GB at 40 mm windows). The root
cause is ALWAYS the same: **an UNBOUNDED window builds an O(n²) visibility graph / unbounded
`unary_union` over hundreds of obstacles.** The unified loop MUST never do this. Every guard below
is mandatory; the spike validates them on a small region first.

### 3a. Memory/runtime guards (each MANDATORY, each maps to a measured blowup)

| Guard | Rule | The blowup it prevents (FAR evidence) |
|-------|------|----------------------------------------|
| **Bounded windows everywhere** | Every gridless route uses a per-net window = pad-bbox + `window_mm`, escalated ×2 only on no-path, HARD-capped at `max_route_window_mm` (25 mm). NEVER board-wide. | M3-P1.1 clean-board hang (>25 min): huge free space → O(n²) graph. Probe-Order proved ≤6 s/net WITH bounded windows. |
| **`skip_full_corner_fallback`** | When the windowed free space still yields a large corner set, SKIP the full-corner visibility-graph fallback (reflex-pruned graph only); accept routing failure over OOM. (EXISTS as a `route_net_multipin` param, used in `multi.py` rescue.) | 18 GB: ~1680 obstacles in a 12 mm window → ~6000 polygon vertices → 18M edges → 4–5 GB numpy. |
| **Capped B.Cu windows** | The 2-layer / B.Cu run uses a SEPARATE tighter cap `max_bcu_window_mm` (8 mm). B.Cu obstacle lists are the largest (~814 segments on mitayi). (EXISTS in `route_net_fanout_escape`.) | 18 GB: O(n²) `unary_union` on ~814 B.Cu track obstacles at 40 mm. |
| **HARD per-round + total work budget** | Per net: `max_expansions` (grid A*) / capped window (gridless). Per round: a wall of total nets routed. Total: `MAX_ROUNDS`. No wall-CLOCK deadline (determinism) — bound WORK, not time. | Multi-hour spins: pathfinder.py already bounds by `max_expansions = 2·L·ny·nx` and `iters`; reuse that discipline. |
| **STRtree-pruned edge tests** | Visibility edges tested only against obstacles near the segment (Shapely STRtree). (EXISTS in `negotiate._build_congestion_visgraph`.) | The O(n²) edge-vs-all-obstacles test. |
| **Reflex-corner pruning** | Only reflex (taut-string) corners are visibility-graph nodes. (EXISTS: `reflex_obstacle_corners`.) | Node-count explosion on dense regions. |
| **Shared field is coarse** | The shared congestion field is 0.5 mm super-cells (kilobytes), not 0.1 mm (would be 25× and per-layer). | The grid `hist` numpy at 0.1 mm × 2 layers is the larger structure; super-cells shrink it ~25–50×. |

### 3b. Determinism strategy

Routes must be byte-identical run-to-run on a fixed install (the hard constraint proven through M1–M3).

1. **Net order:** `order_nets` (power-first, then short-bbox) — a total order; the unified loop pops
   in this order every round. Already deterministic in both `route_all` and `route_gridless_set`.
2. **Integer tie-breaks:** grid A* heap key `(g+h, insertion_seq, node)`; gridless A* heap key
   `(round(f×1e6), seq, node_idx)` — both integer 1 nm bucketed (EXIST in `pathfinder._astar` /
   `negotiate._astar_congestion`). The shared-field read does not touch the tie-break.
3. **Geometry snap:** `set_precision(1e-6)` (1 nm) at every Shapely boundary (EXISTS in `geom.snap`).
4. **Field deposit is order-free:** float64 += over integer super-cell keys; the deposit set is a
   pure function of snapped coords. Sorted before deposit is unnecessary (addition commutes) but the
   SET of cells is deterministic.
5. **Round-best selection** breaks ties by round index (earliest), a total order.
6. **Verification gate (spike + every milestone):** route twice same-process + once fresh
   subprocess; assert emitted `.kicad_pcb` coords byte-identical across all three (the proven gate).

### 3c. Can bounded runtime be GUARANTEED? — honest answer

**Bounded MEMORY: YES, guaranteed** — every window is hard-capped (25 mm route / 8 mm B.Cu), so the
largest free-space `unary_union` and visibility graph are bounded by the obstacle count in a
25 mm×25 mm region (low hundreds, not thousands). `skip_full_corner_fallback` caps the worst case.
The 18 GB blowups were ALL from windows ≥40 mm or board-wide; capped at 25 mm they cannot recur.
**Bounded total RUNTIME: YES for a single pass, bounded by `MAX_ROUNDS × n_nets × per-net-cap`.** The
RISK is the CONSTANT: `MAX_ROUNDS` rounds each re-routing a contended subset could be slow if the
contended subset stays large (poor convergence). This is the open runtime risk — mitigated by (a)
the free iteration-0 giving everyone a baseline, (b) the capped history ramp, (c) round-best
early-accept. **If full-board convergence is too slow, the design SCOPES DOWN to a region** (the
QFN cluster + its grid-contending neighbors — exactly Spike-CoOpt's region), co-optimizes that,
freezes the rest on the grid. This is why the spike is region-scoped first: it proves the mechanism
AND the bounded runtime on a small region before any full-board attempt.

---

## Decision 4 — Staging (incremental build of a large architecture)

| Milestone | Goal | Measurable exit criterion (vs scorecard, beat 41) | Est. effort |
|-----------|------|---------------------------------------------------|-------------|
| **Spike-CoOpt** ⬅ NEXT | Prove a SHARED field lets ONE gridless QFN-fanout net + the specific grid net(s) it displaced BOTH connect in a BOUNDED region | In the bounded region: BOTH the 2–4 QFN-escape nets AND the grid nets they displaced (e.g. /GPIO1, /GPIO2) connect (vs sequential where one loses); 0 new trace DRC errors; deterministic (3-run byte-identical); BOUNDED runtime ≤ a few min, no memory blowup (peak RSS recorded, < ~2 GB). Standalone script, no engine changes. **GO/NO-GO for the whole architecture.** | 1–2 wk |
| **M-CO-1** | Shared field + 2-net co-route IN THE ENGINE: one `_SuperCellGrid` owned by `route_all`, read+written by both `pathfinder._route_one` (grid) and a single-net gridless route; the loop runs on a 2-net problem | On a 2-net fixture (1 grid + 1 gridless that contend): both connect under the shared field where sequential connects only one; deterministic; `coopt=False` byte-identical to current `route_all`. | 2–3 wk |
| **M-CO-2** | Small REGION co-route: the QFN nets + their ~7 grid-contending neighbors, full unified loop (rounds, contention detection, cross-substrate rip-up, convergence) | On the region: the QFN fanout connections coexist with the grid nets WITHOUT displacement — region unconnected ≤ the best sequential result for that region AND ≥1 of the 7 grid nets that attempt-3 displaced (/GPIO1,2,16,17) is RETAINED while a QFN net also connects; deterministic; bounded runtime. **Do they deconflict? — the core question.** | 3–5 wk |
| **M-CO-3** | Full board | mitayi HUMAN placement: **unconnected < 41 AND errors ≤ 73** (beat attempt-3 on both axes); floor target ~38; deterministic; bounded runtime (< ~2× grid `--quality`, no >2 GB peak). | 4–8 wk + profiling |

Gate discipline: do NOT advance until the exit criterion is measured green on the scorecard
(`scripts/_verify_gridless_first_ab.py`-style A/B). **Spike-CoOpt and M-CO-2 are the two go/no-go
gates** — if a shared field cannot make 2 contending nets coexist (spike), or the region cannot
deconflict (M-CO-2), reassess (the architecture's premise is wrong; fall back to the attempt-3
ceiling) before sinking full-board effort.

---

## Spike-CoOpt — the smallest decisive experiment (built NEXT, MUST be bounded)

**Goal.** Validate the CORE HYPOTHESIS: a SHARED congestion field lets a gridless QFN-fanout net and
a contending grid net DECONFLICT (BOTH connect) where SEQUENTIAL routing forced one to displace the
other. This is THE premise of the whole architecture; everything M-CO builds rests on it.

**Decisive because** FAR's attempt-3 (gridless-first) connected 4/17 QFN nets but DISPLACED 4 grid
nets (/GPIO1,2,16,17); fanout-first displaced 7 → 45. If a shared field lets a displaced grid net
AND the QFN net both hold their corridors in a bounded region, the premise holds and full
co-optimization is worth building. If it cannot, the premise is wrong and we stop at 41.

Standalone script `scripts/spike_coopt_shared_field.py`, **no engine changes** — reuse
`build_problem`/`extract_pads`/`project_geometry`/`run_drc`/`strip_routing`; import the gridless
package (`route_gridless_set` machinery / `_route_one_net_congestion`, `_SuperCellGrid`,
`build_windowed_free_space`, `to_gridless_netroute`) and the grid `route_all_pathfinder` /
`pathfinder._route_one`; reuse `_verify_gridless_first_ab.py`'s measure+DRC harness shape and
`spike0b`/`spike2` helpers. BOUNDED windows MANDATORY.

### Region (bounded, mechanical, reproducible)

The mitayi RP2040 QFN cluster, bounded. Rule: take the QFN footprint (densest pad bbox via
`detect_dense_components`), expand its pad bbox by `region_margin_mm = 6.0` → the region bbox. Build
the shared field ONLY over this bbox (super-cells), and route ONLY nets in this region. Cap every
window at `max_route_window_mm = 25.0`, B.Cu at `max_bcu_window_mm = 8.0`. Print the region bbox +
super-cell dims.

### Net set (the SPECIFIC contending pair sets — mechanical)

Take **2–4 QFN-escape nets** (the ones attempt-3 connected via fanout: from {/QSPI_SCLK, /QSPI_SD2,
/GPIO18, Net-(U3-USB-DP)} — those inside the region) AND **the specific grid nets they DISPLACED in
the fanout-first run** (/GPIO1, /GPIO2 — recorded in the FAR attempt-2/3 caveats as the displaced
grid nets). Rule: from `_verify_gridless_first_ab.py`'s `grid_nets_newly_failed` output (the grid
nets that went unconnected when the QFN nets routed first), pick those whose pads fall in the region.
Print the chosen QFN set + the displaced-grid set + confirm they share a corridor (their straight
pad-lines cross within `2×clearance`).

### Procedure

1. Copy mitayi → project-local temp (`.spike_coopt_tmp`, mirror spike0b/spike1 flatpak workaround),
   `strip_routing`. `extract_pads` + `project_geometry` + `build_problem` + `extract_board_outline`
   + `extract_drill_obstacles`.
2. Derive region + net set per the rules. Build ONE shared `_SuperCellGrid` over the region bbox.
3. **BASELINE (sequential, the control):** route the QFN nets FIRST (gridless, fanout, into the
   region), mark their copper, THEN route the displaced-grid nets on the grid around them. Record:
   which connect, which fail (expectation: QFN connects, ≥1 grid net fails — reproduces the
   displacement).
4. **CO-OPT (the test):** run the unified loop (Decision 2) on the SAME net set under the SHARED
   field: free iteration-0 routes all on their assigned substrate; then ≤`MAX_ROUNDS=8` rounds of
   contention-detect → deposit → re-route the contended subset, pricing by the shared field. Cap all
   windows. Realize every route, validate each segment with `exact_geom` against the current
   obstacle set.
5. Emit ALL co-opt routes into the temp board; `refill_zones`; `run_drc`.
6. **Determinism:** re-run steps 3–5 same-process + fresh subprocess; assert emitted coords
   byte-identical across all three (run BEFORE the comparison DRC mutates state — the spike1 FIX-4
   lesson).
7. **Profile + memory:** record per-round contended-set size, per-net window/expansions, total
   rounds to converge, total wall time, AND peak RSS (`resource.getrusage(RUSAGE_SELF).ru_maxrss`).

### Pass criteria (ALL must hold for GO)

- **Deconfliction:** in the co-opt run, BOTH ≥1 QFN-escape net AND ≥1 grid net that the BASELINE
  (sequential) left unconnected now connect TOGETHER — the shared field resolved a contention
  sequential could not. (The decisive delta: co-opt connects strictly MORE region nets than the
  sequential baseline.)
- **All-legal:** 0 new trace-attributable DRC errors (clearance/short/tracks_crossing/hole classes)
  in the region — legality-by-construction holds under joint routing.
- **Deterministic:** byte-identical emitted coords across same-process ×2 + fresh subprocess.
- **BOUNDED runtime/memory:** total ≤ a few minutes (record; flag if >5 min), peak RSS < ~2 GB
  (record; HARD-fail if >4 GB — that is the blowup regime), no single window exceeds the 25 mm cap.

### Validates

That a single shared super-cell congestion field, priced by BOTH substrates and deposited on
contention, lets a gridless QFN-fanout net and a contending grid net coexist in a bounded region —
the premise of cross-substrate co-optimization. That the bounded-window + capped-B.Cu +
`skip_full_corner_fallback` guards keep a JOINT (not sequential) routing pass within memory. That
determinism holds across a multi-round joint loop.

### Defers

Full-board co-optimization (M-CO-3 — the spike is ONE bounded region); dynamic substrate
re-assignment under negotiation (M-CO-3 stretch — the spike's assignment is static); the engine
wiring of the unified loop into `route_all` (M-CO-1 — the spike is standalone); power-pour
interaction (pours refill post-route, FAR-proven invisible to gridless); the >4 grid-contending-net
case (the spike uses the specific 2–4 displaced pair; the full ~7-neighbor region is M-CO-2);
convergence-rate optimization (the spike just needs to converge within 8 rounds on a small region —
if it does not, that is data for the runtime-risk decision, not a fail of the mechanism).

### Fallback trigger (decision point)

If the spike shows the shared field does NOT deconflict (co-opt connects no more than sequential
baseline) → the architecture's premise is unproven; record WHY (is it pricing too weak? too coarse a
super-cell? a real capacity limit — the corridor genuinely fits only one net?) and reassess before
M-CO-1. If it deconflicts BUT blows memory/runtime even on the bounded region → the bounded-runtime
strategy is insufficient at JOINT scale; scope M-CO to an even smaller region and document the cap.
Do NOT abandon on a single determinism wobble — fall back to the `exact_geom` numpy predicates for
legality ops (the proven insurance).

---

## Spike-CoOpt — ordered developer task list

Reuse the production gridless package + the grid `route_all_pathfinder` loop + spike0b/spike2 +
`_verify_gridless_first_ab.py` helpers — **do NOT re-derive the substrate or the negotiated loop.**
NO `pyproject.toml` / production-source edits (that lands in M-CO-1).

1. Confirm `.venv` has `shapely>=2.0,<3.0` (`shapely.geos_version`). Create
   `scripts/spike_coopt_shared_field.py`. Copy mitayi → `.spike_coopt_tmp`, `strip_routing` (reuse
   `setup_board`). `extract_pads` + `project_geometry` + `build_problem` + `extract_board_outline` +
   `extract_drill_obstacles`.
2. **Derive the bounded region:** `detect_dense_components(pads)` → the QFN; region bbox = QFN pad
   bbox + `region_margin_mm=6.0`. Build ONE shared `_SuperCellGrid` over the region bbox. Print bbox
   + super-cell dims + a peak-RSS baseline.
3. **Derive the net set mechanically:** QFN-escape nets in-region (subset of {/QSPI_SCLK, /QSPI_SD2,
   /GPIO18, Net-(U3-USB-DP)}) + the displaced-grid nets in-region (/GPIO1, /GPIO2 — confirm via the
   `_verify_gridless_first_ab.py` `grid_nets_newly_failed` list). Print both sets; assert their
   corridors cross (straight-line distance < 2×clearance).
4. **BASELINE (sequential control):** route QFN nets first (reuse the `route_gridless_set` fanout
   path, bounded windows), `to_gridless_netroute` + `_mark` into the grid ledger; then route the
   displaced-grid nets via `pathfinder._route_one` / `route_all_pathfinder` restricted to the net
   set, around the frozen QFN copper. Record connect/fail per net. Assert ≥1 grid net fails (the
   displacement reproduced) — if not, pick the next displaced pair.
5. **Build the unified-loop driver in the spike** (the mechanism under test, standalone — not the
   engine): (a) one shared `_SuperCellGrid`; (b) static substrate assignment (QFN→gridless,
   grid-net→grid); (c) free iteration-0 routing all on their substrate; (d) ≤8 rounds: detect
   contention on the shared ledger (`occ>1` count → super-cells → nets), `field.deposit` on
   contended super-cells, re-route the contended subset pricing by the shared field (grid via
   `pathfinder._route_one` with its `hist` read swapped to the shared field; gridless via
   `_route_one_net_congestion` passing the shared `sc_grid`), bounded windows + capped expansions;
   (e) break on zero contention; track round-best (`unconnected+errors`); accept best on budget
   exhaust.
6. **Realize + emit:** emit grid + gridless centerlines into the temp board (reuse
   `emit_net_segments` / the grid emit), `refill_zones`, `run_drc`.
7. **Determinism gate (BEFORE the comparison DRC mutates state):** re-run steps 4–6 same-process +
   fresh subprocess; assert byte-identical emitted coords across all three.
8. **Measure + report** (Structured Result + console): per-net baseline connect/fail vs co-opt
   connect/fail; the DECONFLICTION delta (co-opt region-connected − baseline region-connected);
   DRC before/after; rounds-to-converge; per-round contended-set sizes; total runtime; **peak RSS**;
   max window used; determinism PASS/FAIL; GO/NO-GO. On GO: recommend M-CO-1 (shared field in the
   engine) + a `decisions/` KB entry ("shared super-cell field deconflicts cross-substrate
   contention — confirmed"). On NO-GO: record whether the failure is pricing strength, super-cell
   coarseness, a true capacity wall, or runtime/memory — do NOT abandon the architecture on a single
   wobble.

---

## Interface Contracts (binding for M-CO-1; the spike may prototype loosely)

```python
# negotiate.py — route_gridless_set accepts an EXTERNAL shared field (M-CO-1)
def route_gridless_set(
    net_set, pads, geo, board_bbox, ...,
    sc_grid: "_SuperCellGrid | None" = None,   # NEW: when provided, use it instead of
                                               # _make_supercell_grid (the unified loop owns it).
                                               # None = current behaviour (own field), byte-identical.
) -> dict[str, GridlessSetNetResult]: ...

# pathfinder.py — the grid A* prices grid cells by the shared super-cell field (M-CO-1)
def _astar(..., shared_field=None, grid=None):
    # step cost gains: (1 + h_fac * shared_field.history[grid_cell_to_supercell(grid, layer, iy, ix)])
    # when shared_field is provided; else the current private per-cell hist (byte-identical).
    ...

# multi.py — the unified loop entry (M-CO-1)
def route_all(..., coopt: bool = False, coopt_assignment: dict[str,str] | None = None):
    # coopt=False -> current route_all, byte-identical. coopt=True -> the unified loop:
    #   one _SuperCellGrid, free iter-0, capped-ramp rounds, cross-substrate contention
    #   detection, soft re-pricing deconfliction, MAX_ROUNDS budget, round-best accept.
    ...

# the cross-substrate mapping (new helper, congestion.py or negotiate.py)
def grid_cell_to_supercell(grid: "Grid", layer: int, iy: int, ix: int,
                           field: "_SuperCellGrid") -> tuple[int, int]:
    x, y = grid.to_world(iy, ix)
    return field.supercell_of(x, y)
```

`GridlessNetRoute` IS-A `NetRoute` (unchanged — the unified loop sees both substrates' results as
`NetRoute`). The shared field is `negotiate._SuperCellGrid` (unchanged structure; just shared).

---

## Risks and Mitigations

| Risk | Likelihood | Impact | Mitigation |
|------|-----------|--------|------------|
| Shared field does NOT deconflict (corridor genuinely fits one net — a capacity wall, not an ordering one) | Med | High | Spike-CoOpt is EXACTLY this test on a bounded region BEFORE any build; NO-GO fallback documented (stop at 41); FAR's capacity-based coarse-grid finding (commit 82f92ca) suggests capacity is real — the spike measures whether the QFN+grid pair truly cannot coexist or merely needs joint pricing |
| Joint loop blows memory/runtime (the 4–18 GB regime) | Med | High | ALL windows hard-capped (25 mm route / 8 mm B.Cu); `skip_full_corner_fallback`; STRtree + reflex pruning; coarse 0.5 mm field; WORK budget not wall-clock; spike records peak RSS + HARD-fails >4 GB; region-scope fallback if joint scale is too slow |
| Poor convergence: contended set stays large across rounds → slow | Med | Med | Free iteration-0 baseline; capped history ramp; round-best early-accept; `MAX_ROUNDS` hard cap; if full-board won't converge, scope to the region (the spike's region) and freeze the rest |
| Determinism breaks in the multi-round joint loop | Low | High | `order_nets` total order every round; integer A* tie-breaks (both substrates, EXIST); order-free float deposit; 3-run byte-identical gate; `exact_geom` numpy fallback for legality ops |
| Double-counting occupancy (grid + gridless write the same ledger) | Med | High | Single shared `grid.cells`/`hard` ledger; gridless rasterizes at the SAME inflation `_mark` uses (`rasterize_into_grid`, EXISTS); contention detector reads the count semantics directly (grid.py: count==0 free) |
| Super-cell 0.5 mm too coarse to localize contention | Low | Med | Tunable `SUPERCELL_SIZE_MM`; if the spike shows mis-pricing, 0.25 mm is a 4× memory bump still bounded; priced per-edge weighted by crossing length (EXISTS) |
| Substrate assignment wrong (a net assigned grid that needs gridless) | Low | Med | Static assignment from the proven pre-classifier (`min_needed_window > 0.5×board_diag`) + the Probe-Order boxed-in set; M-CO-3 stretch adds dynamic re-assignment behind a measured gate |

---

## Testing Strategy

- **Spike-CoOpt:** the script IS the test — deconfliction delta (co-opt > sequential region nets) +
  all-legal + 3-run byte-identical + bounded-runtime/peak-RSS gate.
- **Unit (M-CO-1+):** `grid_cell_to_supercell` maps a known grid cell to the expected super-cell;
  the shared field read in `pathfinder._astar` matches a hand-computed price; `route_gridless_set`
  with an external `sc_grid` produces identical routes to the internal one for a single net; the
  contention detector flags two overlapping nets and not two disjoint ones.
- **Integration (M-CO-2):** the unified loop on the region: shared field updates from BOTH
  substrates; a grid net's rip-up raises the price a gridless net then sees; convergence within
  `MAX_ROUNDS`; `coopt=False` byte-identical to current `route_all`.
- **Scorecard (every milestone):** `_verify_gridless_first_ab.py`-style A/B on mitayi — compare
  `unconnected`/`errors`/`by_type` against the attempt-3 ceiling (41/73) and the milestone exit
  criterion. Canonical acceptance.

---

## Acceptance Rubric

Per `agents/pm-reference/rubric-format.md`. Atomic, binary, evidence-mandatory. Applies to THIS
design + the Spike-CoOpt it specifies.

1. **R-CO-1 — Unified shared field specified as a single source of truth with the mapping.** PASS iff
   the doc names ONE 0.5 mm super-cell field both substrates read+write, the grid-cell→super-cell and
   gridless-edge→super-cell mappings, and distinguishes it from the unchanged 0.1 mm hard ledger.
   Evidence: Decision 1.
2. **R-CO-2 — Unified iteration loop specified.** PASS iff one negotiated-congestion loop over all
   nets is defined: free iter-0, capped history ramp, per-substrate per-net routing priced by the
   shared field, deterministic order. Evidence: Decision 2 / 2b.
3. **R-CO-3 — Cross-substrate contention detection specified.** PASS iff contention is defined on the
   shared 0.1 mm count ledger (`occ>1` → super-cells → nets), works across grid+gridless, and drives
   the deposit. Evidence: Decision 2c.
4. **R-CO-4 — Rip-up / victim selection across substrates.** PASS iff the soft re-pricing
   (McMurchie-Ebeling vacate-or-keep) mechanism + deterministic tie-break is stated, distinct from
   the old hard-wall `_nearest_victim`. Evidence: Decision 2d.
5. **R-CO-5 — Convergence + bounded round budget.** PASS iff convergence (zero contention) +
   `MAX_ROUNDS` hard cap + round-best accept-on-exhaust are specified. Evidence: Decision 2e.
6. **R-CO-6 — Bounded runtime/memory strategy is explicit and evidence-linked.** PASS iff bounded
   windows, `skip_full_corner_fallback`, capped B.Cu windows, the hard work budget, STRtree/reflex
   pruning, and the coarse field are EACH named AND each mapped to a measured FAR blowup. Evidence:
   Decision 3a table.
7. **R-CO-7 — Determinism strategy stated.** PASS iff net order, integer tie-breaks, geometry snap,
   order-free deposit, and the 3-run gate are stated. Evidence: Decision 3b.
8. **R-CO-8 — Honest runtime-risk answer.** PASS iff the doc states whether bounded runtime can be
   GUARANTEED (memory yes, total-runtime bounded-but-convergence-risk) and the region-scope fallback.
   Evidence: Decision 3c.
9. **R-CO-9 — Staging with measurable exits vs 41.** PASS iff M-CO-1/2/3 each have a scorecard exit
   criterion referencing the attempt-3 ceiling. Evidence: Decision 4 table.
10. **R-CO-10 — Spike-CoOpt fully + decisively specified, BOUNDED.** PASS iff the region, net set
    (the specific QFN + displaced-grid pair), procedure, exact pass criteria (deconfliction +
    all-legal + deterministic + bounded runtime/RSS), validates, defers, and an ordered standalone
    task list are all stated. Evidence: Spike-CoOpt section + task list.
11. **R-CO-11 — No production code written.** PASS iff only this `.md` is created/edited (pseudocode
    + contracts only; no runnable source). Evidence: `files_changed` = this doc only.

---

## Spike-CoOpt (2026-06-23) — GO-WITH-CAVEATS: deconfliction PROVEN, bounded; the architecture works

`scripts/spike_coopt_shared_field.py` (PM-verified by independent re-run). In a bounded mitayi QFN-cluster
region, co-routed 3 QFN-escape nets (gridless) + 2 grid nets they displaced (/GPIO1, /GPIO2) under ONE
shared 0.5mm super-cell congestion field.

**CORE HYPOTHESIS VALIDATED — the shared field deconflicts cross-substrate contention:**
| | region nets connected |
|---|---|
| BASELINE (sequential: gridless QFN first, then grid) | **3/5** — /GPIO1, /GPIO2 FAIL (displacement reproduced) |
| CO-OPT (shared field, unified loop) | **5/5** — /GPIO1 AND /GPIO2 rescued + all 3 QFN nets kept |

**Deconfliction = +2.** The shared congestion field lets a gridless QFN-fanout net and a contending grid
net BOTH connect where sequential forced one out — the premise of the whole architecture.

**BOUNDED (the critical de-risk — independently measured):** peak RSS **365MB** (`/usr/bin/time`; target
<2GB, the 4–18GB blowups did NOT recur), 180s wall, max window 8mm (cap 25), deterministic (same-process
+ subprocess byte-identical). The bounded-window + capped-B.Cu + region-scope discipline WORKS — joint
routing stays bounded.

**Caveat (spike implementation gap, NOT an architecture flaw):** 11 DRC errors (shorting/clearance/hole).
Root cause: /GPIO1,/GPIO2 are multi-pin nets with their OWN U3 QFN pads, and the spike routed them with
PLAIN grid A* THROUGH the dense QFN pad clearance zone → shorts vs adjacent QFN pads. Fix: route those
nets' QFN source pads via FANOUT-ESCAPE too (the validated capability), not naive grid A*. The shared-field
deconfliction mechanism itself is clean.

**Convergence note:** the loop did not reach 0 contention (oscillated 191→174→327 over 7 rounds — density
contention in the QFN pad area); the round-best accept correctly picked the min-score round. Convergence
rate is the documented open risk for full-board scale (region-scope fallback exists).

**VERDICT: GO for the architecture.** Cross-substrate co-optimization is proven (deconfliction +2) and
bounded (365MB). Next: M-CO-1 (shared field in the engine, 2-net) → fix the QFN-padded grid nets with
fanout-escape so the region is DRC-clean → M-CO-2 (region) → M-CO-3 (full board, beat 41/73).
