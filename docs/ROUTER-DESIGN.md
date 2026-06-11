# TraceWise Router — Design

*Design date: June 2026. The decision to build an own router is recorded in
[docs/PLAN.md](PLAN.md) ("Router strategy note") and grounded in the
Freerouting ablation evidence in [docs/route-ablation-v3.md](route-ablation-v3.md).
This document specifies the v1 engine: scope, algorithm, integration, testing,
module layout and milestones.*

## 1. Why an own router

Three measurement rounds plus a designer review (10+ years electronics design)
converged on a single conclusion: on real community boards Freerouting is the
bottleneck, not our constraints. The recorded failure modes are not subtle:

- **Dangling stubs dominate** — 35–48 `track_dangling` items on a single
  board, the largest single contributor to "this looks awful" and a defect
  raw violation counts under-weight.
- **Poor completion** — 21–89 unconnected nets on every arm of every board,
  with run-to-run variance (39 vs 21 on identical inputs), so even the
  completion number is not dependable.
- **Zone-clearance blindness** — copper pours carry their own clearance
  (a real bug we hit: the GND pour demanded 0.18 mm while tracks routed to
  0.15 mm, producing ~500 track-to-zone violations the router never saw).

These are not constraint problems. They are properties of the engine's output.
The constraint generator (`route/constraints.py`), the DSN/SES bridge plumbing
(`route/bridge.py`), the lossless s-expression editor (`sexpr.py`) and the DRC
scoring harness are all **router-agnostic and carry over unchanged**. What is
missing is an output stage we can hold to a quality bar by construction.

## 2. Scope ladder (honest scoping is the project's signature)

**v1 target:** 2-layer, sub-50 MHz digital and mixed-signal boards, routed on a
uniform grid. This is the boring 80% — the dense 2-layer class where the
ablation showed constraints *help* and where grid routing has a sixty-year
track record.

### In scope for v1

- 2 copper layers (top + bottom), through-hole and SMD pads, through-vias only.
- Orthogonal + 45° track segments on a uniform grid.
- Per-net-class track width / clearance / via geometry, sourced from
  `route/constraints.py`.
- Copper-zone (pour) awareness as a clearance obstacle, including zone-local
  clearance (the bug above, fixed by construction).
- 100 % completion **or** an explicit, per-net failure report. No silent
  partial routes, ever.

### Explicit non-goals for v1 (deferred or never)

- **High-speed / SI-PI**: no impedance control, no length matching enforcement,
  no return-path analysis. (Consistent with the project's standing non-goals.)
- **Differential-pair geometric coupling**: pairs are routed as two ordinary
  nets in v1; coupled-pair routing is a v2+ topic. (Constraint generation still
  *labels* them.)
- **4+ layers, blind/buried/micro-vias**: v1 is strictly 2-layer, through-via.
- **BGA / fine-pitch escape (fanout)**: the documented hard case for maze
  routers; out of v1 scope. A rule-based fanout pre-pass is a later module.
- **Curved / arbitrary-angle / length-tuned (accordion) routing.**
- **Autonomous sign-off** of anything. The engine produces DRC-clean copper for
  its class of board; engineering judgment stays with the user.

The non-goals are load-bearing: a router that *claims* only 2-layer sub-50 MHz
and delivers DRC-clean completion there is more useful than one that claims
everything and dangles stubs.

## 3. Quality bar (derived directly from the Freerouting failure analysis)

Each bar is a *construction* property, not a post-hoc check — the point is to
make the failure modes unrepresentable rather than to detect them afterward.

1. **Zero dangling stubs by construction.** A net is routed as a connected tree
   spanning all its pads, or it is reported as failed and *no* partial copper is
   emitted for it. The data structure is a per-net connection tree; a "stub" (a
   segment with a non-pad, non-junction free end) is not an expressible state.
2. **Clearance-aware by construction.** The grid's occupancy model inflates
   every obstacle (pads, other nets' copper, board edge, *and zones*) by
   `track_width/2 + clearance` for the net being routed. A cell that would
   violate clearance is simply not in the search space. Zone clearance uses the
   zone-local value (`read_zone_clearance` already extracts the per-board
   maximum; the engine consumes the per-zone value).
3. **100 % completion or explicit per-net failure.** The router reports, per
   net: routed / failed-with-reason (blocked, no-via-space, budget-exhausted).
   The CLI exit code reflects unrouted-net count. A partial board is never
   silently presented as done.
4. **External judge, always.** Every run ends in `kicad-cli pcb drc --format
   json` (via `route/bridge.run_drc`). The engine's internal clearance model
   and KiCad's DRC must agree; any disagreement is a bug in the engine, surfaced
   loudly. The harness records the same metrics the ablation uses
   (completion / violations / vias / length) **plus dangling-stub count**, which
   the designer review identified as the missing metric.

## 4. Algorithm choice

### Candidates evaluated

| Approach | Strengths | Why not for v1 |
|---|---|---|
| **Channel / switchbox routing** | Optimal in structured channels; classic for gate arrays | Assumes a global floorplan decomposition into channels; PCBs are not channel-structured, and channel definition on arbitrary placement is itself hard |
| **Topological (rubber-band / Topola-style)** | Layer-efficient, fewer vias, handles density well | Much higher implementation complexity; geometry of rubber-band sketches + later geometrization is a multi-month effort; this is the *v2+* direction, slotting behind the same I/O contract |
| **Grid A\*/Lee maze + rip-up-and-reroute + push-and-shove** | Simple, complete on the grid, clearance is trivially expressible by cell occupancy, dangling-stub-free by construction (tree search), sixty years of literature | Memory at fine grids; sequential net ordering needs rip-up to approach completeness | 

**Recommendation: grid A\* (Lee/maze family) with net ordering + bounded
rip-up-and-reroute, push-and-shove deferred to a sub-phase of R1/R2.** It is the
only candidate whose core invariants (clearance-by-occupancy, tree-by-search)
directly deliver the quality bar, and the only one a single engineer can land in
pure Python in the v1 milestone window. Topological is the acknowledged
successor; it reuses this engine's I/O contract verbatim.

A\* over Lee's pure breadth-first wavefront because A\* with an admissible
octile-distance heuristic visits dramatically fewer cells on sparse boards while
remaining complete and shortest-path-optimal on the grid.

### Complexity at PCB scale

Reference board: **50 × 60 mm**, **0.1 mm grid**.

- Grid: ⌈50/0.1⌉ × ⌈60/0.1⌉ = **500 × 600 = 300 000 cells per layer**.
- 2 layers ⇒ **600 000 nodes**. With 8-connectivity (orthogonal + 45°), ~8
  edges/node ⇒ ~4.8 M directed edges. Stored as a `numpy` occupancy array
  (one `uint8`/`int32` per cell per layer), the grid is **0.6–2.4 MB** — trivial.
- A single A\* search is `O(E log V)` in the worst case but in practice explores
  a small corridor between two pads; with a tight heuristic and a bounding box
  around the net's terminals (search is clipped to the net's bbox + margin), a
  typical net touches **10²–10⁴ cells**, not 6 × 10⁵.
- Board scale: **50–150 components, 60–120 nets**. Total routing work is
  `nets × (rip-up rounds) × per-net-search`. At ~10⁴ cells/search, 120 nets, and
  a rip-up budget averaging ~2 attempts/net, that is ~2.4 M cell expansions —
  **seconds, not minutes, in pure Python with numpy-backed grids**. Pure Python
  is the v1 commitment; we measure before optimizing (see §6 Rust note).

Memory and time both sit comfortably within pure-Python/numpy reach at the
target board class. Fine pitch or 4-layer would change this calculus — and they
are out of scope precisely because they would.

### Net ordering strategy

Ordering is the single biggest lever on completion for sequential maze routers.
v1 heuristic, applied in this priority:

1. **Power/ground last-but-widest is wrong for grids** — wide power tracks block
   the most cells, so route **power and ground first** (they want short, direct,
   low-impedance paths and benefit from claiming territory before signals
   fill in). This matches the displacement hypothesis from the ablation: wide
   power tracks shoved signal routing into the pour and caused violations.
2. **Then shortest-bounding-box-first** among signals (the classic "route the
   easy/constrained ones before the grid fills"). Short nets have the least
   freedom and the lowest rip-up cost when displaced.
3. **Criticality tie-break**: net-class priority (DiffPairs/Slow before
   Default) so labeled nets get first pick of the channel.

Ordering is a pluggable strategy object so R1 can A/B orderings against the
benchmark completion numbers rather than guessing.

### Rip-up-and-reroute budget

- A net that fails to route triggers **rip-up of the cheapest blocking nets**
  (lowest already-routed cost / lowest priority) which are torn up and re-queued.
- **Budget: global cap of `8 × net_count` reroute attempts** and a per-net cap
  of **10 attempts**, whichever binds first. On exhaustion the net is reported
  failed (quality bar #3) — never left as a stub.
- A **negotiated-congestion** flavor (PathFinder-style: cell cost rises with
  historical contention) is the R1 escalation if flat rip-up plateaus below
  100 % on the benchmark boards. Recorded as the planned escalation, not built
  speculatively.

**Push-and-shove** (sliding an existing track aside within its clearance rather
than ripping it up) is a refinement applied within rip-up rounds in R1/R2; it is
not on the R0 critical path.

## 5. Integration

### Input

- **Geometry**: the `place/extract.py` pcbnew pattern — a tiny script run inside
  KiCad's own runtime emits JSON (footprints, pads with net + offset,
  courtyards, board edges). The router needs the same plus: per-pad layer and
  shape, existing copper to preserve, and **zone polygons with their per-zone
  clearance**. This is an additive extension to the extract script, kept in the
  same JSON-over-pcbnew style so SWIG drift stays KiCad's problem.
- **Net classes**: `route/constraints.NetClass` (already carries `track_mm`,
  `clearance_mm`, `via_dia_mm`, `via_drill_mm`). The engine maps every net to a
  class and derives its occupancy inflation from the class. `clamp_to_project`
  and `read_zone_clearance` mean the widths/clearances the engine sees already
  respect the project minimums and pour floors.

### Output — recommendation: **our sexpr editor**, not a pcbnew script

The bridge writes routing back via pcbnew (`ImportSpecctraSES`). For our own
engine the recommendation is to **emit tracks and vias directly into the
`.kicad_pcb` with `sexpr.py`**, justified:

- **No SWIG dependency for the write path.** The output is a list of
  `(segment | via)` records; appending `(segment (start ..) (end ..) (width ..)
  (layer ..) (net ..))` and `(via (at ..) (size ..) (drill ..) (layers ..)
  (net ..))` nodes is exactly the surgical-insert the sexpr core was built for
  (byte-identical round-trip proven on all KiCad v9/v10 demo files).
- **Lossless and reviewable.** The board diff is a clean append of copper nodes;
  nothing else in the file moves. This is the same discipline the Fixer uses.
- **Deterministic and CI-friendly.** Writing copper needs no running KiCad, so
  the core round-trips in plain unit tests.

Caveat recorded: **zones must still be refilled after editing copper**, and zone
refill *requires* pcbnew (`ZONE_FILLER`), exactly as `bridge.import_ses` already
does. So the write path is: sexpr emits copper → a one-line pcbnew refill+save
(reuse `bridge`'s pattern) → DRC. The DSN/SES bridge stays available as a
fallback engine behind the same interface.

### The DRC iterate loop (kicad-cli as external judge)

```
extract(board) ──▶ build grid (occupancy from pads/zones/edges, per-net inflation)
                       │
            order nets ▼
        route + rip-up loop ──▶ per-net trees (or failure reports)
                       │
        emit copper (sexpr) ──▶ pcbnew zone refill + save
                       │
   run_drc(board)  ◀───┘   (route/bridge.run_drc → kicad-cli JSON)
                       │
          drc_summary ▼
   clean? ──no──▶ map violations back to nets, raise their cell cost, re-route those nets
     │                    (bounded by the rip-up budget)
     yes
      ▼
   report: completion / violations / vias / length / dangling-stubs (== 0 by construction)
```

The external DRC is authoritative. If the engine believes a route is clean and
DRC disagrees, that gap is the bug to fix — the loop surfaces it instead of
hiding it.

## 6. Module layout — `src/tracewise/route/engine/`

Python first, `numpy` allowed for the grid. Rough LOC are *targets*, not budgets.

| File | Responsibility | ~LOC |
|---|---|---|
| `__init__.py` | Public surface: `route(board, spec) -> RouteResult` | 20 |
| `extract.py` | Routing-specific pcbnew extraction (pads+layers, existing copper, **zone polygons + per-zone clearance**); JSON-over-pcbnew, mirrors `place/extract.py` | 140 |
| `grid.py` | `numpy` occupancy grid: world↔cell transform, per-net obstacle inflation (`width/2 + clearance`), zone rasterization, multi-layer stack | 220 |
| `astar.py` | A\* / Lee maze search on the grid, octile heuristic, 8-connectivity, bbox clipping, returns a cell path or `None` | 180 |
| `net.py` | `Net`, `NetClass`-binding, multi-pad ordering (route as a connected tree via successive nearest-terminal A\*); the data structure that makes stubs unrepresentable | 160 |
| `order.py` | Net-ordering strategies (power-first, shortest-bbox, criticality); pluggable | 90 |
| `ripup.py` | Rip-up-and-reroute loop, budget accounting, negotiated-congestion cost (R1 escalation), per-net failure reporting | 200 |
| `shove.py` | Push-and-shove: slide a routed segment within clearance instead of ripping it (R1/R2) | 160 |
| `vias.py` | Via insertion/cost model, layer-change legality (R2) | 120 |
| `emit.py` | Cell-path → board geometry → `sexpr` track/via nodes; orthogonal+45° segment simplification | 180 |
| `result.py` | `RouteResult` dataclass: per-net status, metrics, the report the CLI/harness consume | 80 |
| `engine.py` | Orchestrator: extract → grid → order → route/ripup → emit → refill → DRC iterate | 200 |

Total ≈ **1.75 kLOC** for v1.

### Where a Rust rewrite would slot — but not yet

The hot loops are `astar.py` (cell expansion) and `grid.py` (occupancy
inflation/rasterization). Both have narrow, numeric interfaces and no KiCad
coupling, so they are the natural `pyo3`/`maturin` extraction point **if and
only if** measurement on the benchmark boards shows pure Python missing the
time bar. **v1 is pure Python; we measure first.** The §4 estimate says we have
room; the Rust note is recorded so the boundary is designed for, not retrofitted.

## 7. Testability

### Unit (synthetic grids — no KiCad)

- `grid.py`: obstacle inflation produces the exact forbidden cell set for known
  pad/zone geometry; world↔cell transforms round-trip; zone clearance uses the
  per-zone value.
- `astar.py`: on hand-built grids — empty grid (straight path), single
  wall (detour), fully blocked (returns `None`), narrow corridor exactly one
  clearance wide (finds it), corridor one cell too narrow (rejects it).
- `net.py`: a 3-pad net yields a connected tree spanning all pads with **no free
  ends** (the stub-impossibility invariant, asserted directly).
- `order.py`/`ripup.py`: ordering determinism; budget accounting; a contrived
  congested grid that requires exactly one rip-up to complete.

These run in milliseconds with no pcbnew, on every push.

### Integration (the 3 benchmark boards in `data/benchmark-boards/`)

`mitayi-pico-d1`, `rp2040-dev-board`, `zuluscsi-pico-oshw`. For each: strip
routing, run the engine, refill zones, run `kicad-cli` DRC. Assertions:

- **dangling stubs == 0** (the headline; true by construction, verified by DRC
  `track_dangling` count),
- reported completion matches DRC unconnected count (engine vs external judge
  agree),
- zero track-to-zone clearance violations (the pour-clearance bug stays fixed).

These self-skip when no pcbnew/kicad-cli is present, exactly like the existing
kicad-cli-dependent tests.

### Benchmark vs Freerouting (reuse the ablation harness pattern)

Extend `scripts/ablation_route.py` from a 2-arm (naked/constrained Freerouting)
to a **3-arm** comparison: add a `tracewise` arm that swaps `route_board` for
the own engine, keeping the *identical fair-scoring DRC* (restore original
project rules, drop our `.kicad_dru` before scoring) so all arms are judged on
one rule set. Reported columns: **completion / DRC violations / vias / length /
dangling-stubs** — the last column is the metric the designer review proved the
old harness was missing. Multiple runs per board (Freerouting determinism can no
longer be assumed; the own engine is deterministic, which is itself a result).

## 8. Milestones

Each milestone is independently demonstrable and ships its own metrics, matching
the project's release discipline.

### R0 — single net on an empty grid
Build `grid.py` + `astar.py` + minimal `emit.py`. Route one 2-pad net on an
otherwise empty board, write copper via sexpr, refill, DRC-clean.
**Exit:** one net routed, DRC-clean, byte-identical board diff except the
appended copper; unit tests for grid + A\* green.

### R1 — multi-net, ordering, rip-up (single layer)
`net.py`, `order.py`, `ripup.py`. Route all single-layer-routable nets on a
benchmark board's bottom layer; multi-pad nets as connected trees; flat rip-up
with budget; per-net failure report.
**Exit:** ≥1 benchmark board routed to 100 % *on a single layer where feasible*,
or an honest per-net failure list; zero dangling stubs; ordering strategy
A/B numbers recorded. Negotiated-congestion escalation decided by the data.

### R2 — vias / 2-layer
`vias.py` + layer dimension in `grid.py`/`astar.py` + push-and-shove in
`shove.py`. Full 2-layer routing with through-vias; via cost discourages
gratuitous layer changes.
**Exit:** all 3 benchmark boards routed on 2 layers; via counts in a sane range
vs Freerouting's; DRC-clean except for documented residuals.

### R3 — zones / pours awareness
Zone polygons rasterized as clearance obstacles with **per-zone clearance**;
copper near a GND pour respects the pour's own clearance (the bug that motivated
`read_zone_clearance`).
**Exit:** zero track-to-zone clearance violations on all 3 boards after zone
refill — the specific failure that beat the constrained Freerouting arm on
mitayi is closed.

### R4 — benchmark parity vs Freerouting
3-arm ablation (naked FR / constrained FR / TraceWise) on all 3 boards, fair
scoring, multiple runs, full metric set incl. dangling-stubs.
**Exit:** on the 2-layer boards in the suite, the TraceWise arm shows **higher
completion and zero dangling stubs** vs both Freerouting arms, with via/length
within a defensible band. Negative or mixed results reported as honestly as the
ablations were — parity-or-better on the *target class*, not a universal win.

---

## Structured Result

```json
{
  "status": "complete",
  "summary": "Wrote the v1 PCB autorouter design doc for TraceWise. Scope: 2-layer sub-50MHz grid routing only, with explicit non-goals. Recommends grid A*/Lee maze with net ordering (power-first, then shortest-bbox, criticality tie-break) and bounded rip-up-and-reroute (8x net_count global, 10/net), push-and-shove and negotiated-congestion as recorded escalations. Quality bar derived from the Freerouting failure analysis: zero dangling stubs and clearance/zone-clearance compliance by construction, 100% completion or explicit per-net failure. Grid math for the 50x60mm/0.1mm reference: 500x600=300k cells/layer, 600k nodes over 2 layers, 0.6-2.4MB numpy grid, ~seconds in pure Python. Integration reuses extract/constraints/bridge/sexpr; output via sexpr editor (no SWIG on write path) + pcbnew zone refill, DRC iterate loop with kicad-cli as external judge. Module layout under route/engine/ (~1.75 kLOC) with Rust extraction point identified for astar/grid if measurement demands. Testability: synthetic-grid units, 3-board integration, 3-arm ablation extending scripts/ablation_route.py with a dangling-stub column. Milestones R0-R4 with exit criteria.",
  "files_changed": ["/home/palgin/Business_projects/tracewise/docs/ROUTER-DESIGN.md"],
  "files_read": [
    "/home/palgin/Business_projects/tracewise/README.md",
    "/home/palgin/Business_projects/tracewise/docs/DESIGN.md",
    "/home/palgin/Business_projects/tracewise/docs/PLAN.md",
    "/home/palgin/Business_projects/tracewise/docs/route-ablation-v3.md",
    "/home/palgin/Business_projects/tracewise/src/tracewise/route/constraints.py",
    "/home/palgin/Business_projects/tracewise/src/tracewise/route/bridge.py",
    "/home/palgin/Business_projects/tracewise/src/tracewise/place/extract.py",
    "/home/palgin/Business_projects/tracewise/src/tracewise/boardspec.py",
    "/home/palgin/Business_projects/tracewise/scripts/ablation_route.py",
    "/home/palgin/Business_projects/tracewise/src/tracewise/sexpr.py"
  ],
  "issues": [],
  "assumptions": [
    "Through-vias only and 2 copper layers for v1; blind/buried/micro-vias deferred.",
    "Differential pairs are routed as two independent nets in v1 (constraints still label them); coupled-pair geometry is v2+.",
    "0.1mm grid and 50x60mm board are the reference for the complexity analysis; actual benchmark boards may differ but stay within the same order of magnitude.",
    "The write path emits copper via sexpr then uses a pcbnew one-liner for zone refill, since ZONE_FILLER requires pcbnew; the DSN/SES bridge remains a fallback engine behind the same interface.",
    "Pure-Python/numpy meets the time bar at the target board class; Rust extraction is held until measurement proves otherwise."
  ]
}
```
