# Research: FAR-gridless-routing-survey
## Free-space area router substrate — prior art for TraceWise v2

**Researched:** 2026-06-19
**Decision trigger:** Two measured gaps vs. human router both trace to grid quantization:
connectivity (grid over-estimates congestion; exact geometry is the limit) and legality
(grid-quantized positions produce sub-pitch clearance errors; only 1/56 residual via
errors are nudgeable — they need rerouting to different positions). Budget: ~3-6 months
Python. Replaces only the SEARCH SUBSTRATE; reuses extract_pads/build_problem, congestion
history, DRC scorecard, 2-layer+vias.

---

## Problem Framing

**Goal:** Identify the best search substrate for a free-space (gridless) PCB router in
Python that eliminates quantization error for both connectivity and legality, while
plugging into the existing TraceWise pipeline.

**Hard constraints:**
1. Python implementation (pure Python + mainstream libs; Rust/C extension only if clearly
   necessary and via a Python API)
2. 3-6 month solo effort budget
3. Must reuse extract_pads/build_problem, congestion history pricing, DRC scorecard,
   2-layer + via insertion
4. Deterministic and work-bounded (the router was just hardened; new substrate must
   preserve these properties)
5. Must support incremental adoption: some nets routed gridless, others fall back to grid

**Soft constraints:**
1. Open-source prior art preferred (Python or translatable)
2. Numerical stability / exact predicates — geometry bugs are the dominant failure class
3. Congestion negotiation must be layerable on top (history-based pricing from existing
   multi.py)
4. Staged rollout preferred over big-bang replacement

**Known-not-wanted:**
- Freerouting re-integration (Java, documented ablation failures: completion/dangling-stub,
  zone-blindness; see NEXT-exact-geometry-routing.md)
- ML/NN routing (no tractable clearance guarantee in Python)
- Approaches requiring >2 layers or blind vias (out of v1 scope)

---

## Candidate Approaches

| # | Approach | Source / Provenance | Fit Score (0–5) | Fit Rationale | Known Risks | Cost / Complexity | Staged Rollout | Verdict |
|---|----------|---------------------|-----------------|---------------|-------------|-------------------|----------------|---------|
| 1 | **Convex expansion rooms (Freerouting-style)** | tinycomputers.io Freerouting analysis [1]; US patent 7937681 [2]; freerouting/freerouting GitHub [3] | 4/5 | Directly matches the gap: routes in continuous coords over Minkowski-inflated convex "expansion rooms"; legality by construction; A* over rooms plugs into existing cost model; congestion pricing layers onto room cost naturally | Room generation (lazy convex polygon expansion) is ~3-5 kLOC in Java; Python rewrite with Shapely is the dominant complexity; numerical robustness near room corners is fiddly | Med-High: ~3-4 months Python from scratch; Shapely for geometry; new A* node type (polygon room vs grid cell) | Good: can route a single net gridless, others use grid fallback; same `route()` interface | **Recommend** |
| 2 | **Topological / rubber-band routing (sketch → geometric realization)** | gEDA Toporouter (C, Anthony Blake 2008) [4]; Ruby-PCB-Router (Dayan thesis) [5]; SURF/rubber-band sketch (IEEE 979685) [6]; Altium Situs description [7] | 3/5 | Highest quality ceiling: routes topological sketch first (which side of each obstacle), then realizes as Euclidean shortest-path rubber-band; eliminates both quantization errors by separating topology from geometry | Highest implementation complexity: constrained Delaunay triangulation of the board, rubber-band sketch generation, and a separate geometric-realization pass — three distinct algorithmic components; gEDA Toporouter "results were poor" on dense 2-layer boards; multi-net congestion negotiation requires re-ordering sketches across nets (not a natural layering) | High: ~4-6 months; three sub-systems each with geometry bugs of their own | Partial: topological sketches are per-net, so can route individual nets gridless; but congestion interaction is harder to decouple | Consider |
| 3 | **Visibility graph + Dijkstra/A* (point-to-point geometric shortest path)** | pyvisgraph (MIT, TaipanRex, GitHub) [8]; extremitypathfinder (Python, readthedocs) [9]; Computational Geometry in Python (Toptal) [10] | 3/5 | Mathematically clean: Minkowski-inflate all obstacles, build visibility graph over inflated polygon vertices + source/dest, run Dijkstra; legal by construction; every path is a Euclidean shortest path in free space | O(n² log n) graph build; must be REBUILT each time any obstacle (previously-routed net) is added — for a 200-net board this is 200 × O(n²) incremental rebuilds; pyvisgraph took 554s to build for 4,335 obstacle points; incremental update not supported in any Python lib surveyed; memory also grows as O(n²) | Med: visibility graph build ~200-400 LOC; but performance is a hard constraint violation for multi-net boards | Poor: rebuilding the full graph per net is too expensive for dense boards; no practical staged rollout path | Consider (Spike-0 only for single-net validation) |
| 4 | **Constrained Delaunay triangulation / navigation mesh (CDT navmesh)** | CDT Wikipedia [11]; jdxdev RTS navmesh blog [12]; IJCAI 2017 compromise-free navmesh pathfinding [13]; PCB routing on unstructured meshes 2025 (Springer, paywalled) [14] | 3/5 | Build a CDT of free space between obstacles (treated as polygonal holes); A* over triangles; trace runs along triangle edges / funnel algorithm gives the shortest path through the mesh; dynamic CDT update (Delaunay edge flips) allows incremental obstacle insertion | Python CDT libraries (bmmeijers/tri, Delaunator-Python) are thin wrappers around C; no maintained Python CDT with hole support + incremental update surveyed; funnel-algorithm implementation is ~500-800 LOC; multi-net congestion pricing over triangles is non-trivial (which triangle "owns" a routed net?) | Med: ~2-3 months if a solid CDT library exists; currently no well-maintained Python CDT with holes found | Good: CDT can be rebuilt incrementally per routed net; each net is independent search | Consider |
| 5 | **Hybrid: grid global path + exact-geometry local resolution (Minkowski snap)** | US patent 6545288 (gridless using maze+line-probe) [15]; NEXT-exact-geometry-routing.md (internal, Approach #4) | 3/5 | Keep grid A* for global topology; replace emit.py cell-center snapping with Shapely Minkowski-aware endpoint resolver; addresses legality; partial connectivity improvement (finer effective resolution at emit time) | Does NOT eliminate the connectivity gap — grid still over-estimates congestion; only closes the legality (DRC) gap. Quantization error is moved from routing to emission but not eliminated. This is Approach #4 from the prior survey — correct for immediate fix, NOT a true FAR | Low-Med: 3-5 days (already analyzed in prior survey) | Excellent: purely additive to existing engine | Reject as FAR substrate (correct as immediate patch, wrong as v2 architecture) |

---

## Approach Details

### Approach 1: Convex Expansion Rooms (Freerouting-style)

**How it works:**
Free space is represented as a set of lazily-generated convex polygons ("expansion rooms"),
each bounded by the Minkowski-inflated clearance halos of nearby obstacles (pads, already-
routed traces, board edges). The A* search state is a (room, entry-edge) pair. Expanding a
state means growing the room outward, clipping against neighboring obstacle halos to find
adjacent rooms. A trace is legal by construction: it runs through the interior of a room,
which is guaranteed to be clearance-distance from all obstacles.

**Connectivity:** Eliminates grid over-estimation entirely. A net is unroutable only if the
actual geometric free space between its terminals is too narrow — the true limit.

**Legality:** Every routed segment is inside a Minkowski-inflated free-space room. Clearance
is structural, not checked post-hoc.

**Congestion negotiation:** The existing history-based pricing (cost += history[cell]) maps
naturally: cost += history[room_id] or a spatially-hashed version. Rip-up-and-reroute works
at the net level. The congestion signal is coarser (room vs. cell) but conceptually identical.

**Via / 2-layer:** A via transition is a room expansion that crosses to a second layer; the
via pad itself is an obstacle on both layers, creating a new room boundary.

**Python complexity:** Shapely handles buffer/offset (Minkowski inflation) and polygon
intersection. Room expansion = `obs.buffer(clearance).union(...)` + `free_space.difference(...)`.
The A* node type changes from `(layer, iy, ix)` to `(layer, room_polygon, entry_edge)`.
Estimated: ~2,000-3,500 LOC for the routing core (vs. Freerouting's ~10kLOC Java, which
has 3 geometry modes, interactive routing, and optimization passes).

**Numerical risks:** Room corner degeneracy when two inflated obstacle halos nearly touch.
Shapely / GEOS is deterministic but uses floating-point; near-zero-area rooms must be
filtered. Workaround: minimum room area threshold + fallback to grid for sub-threshold rooms.

**Open source prior art:** Freerouting (Java, GPL, ~10kLOC search core) is the canonical
reference. No Python port exists. The algorithm is well-described in [1] and implicitly in
the Freerouting source.

**Staged rollout:** Route individual nets through the gridless engine; keep `grid_astar` as
fallback. Same `NetRoute` result type. Works at the net granularity.

---

### Approach 2: Topological / Rubber-Band Routing

**How it works (3-phase):**
1. **Topological sketch** — route each net through a constrained Delaunay triangulation
   (CDT) of the board. The sketch says "go left of obstacle A, right of obstacle B" — no
   coordinates yet. The rubber-band representation is a sequence of CDT triangle edges
   (corridor).
2. **Geometric realization** — tighten the rubber band: the physical wire follows the
   Euclidean shortest path through the corridor. This is the "funnel algorithm" over the
   sequence of triangles.
3. **Layer assignment** — for multi-layer boards, decide which sketch crossings become
   vias.

**Connectivity:** Similar to Approach 1 — routes in exact geometry, so the true
connectivity limit is reachable.

**Legality:** Geometric realization can enforce clearance by inflating obstacles before
the funnel step. Legal by construction.

**Congestion:** The hard part. In topological routing, congestion is handled by re-ordering
the routing sequence and revising topological sketches. The history-based pricing from
TraceWise's multi.py does not map cleanly to the topological sketch phase (pricing over
CDT triangles is a different signal from history over grid cells). This is the main fit risk.

**Multi-layer / vias:** Layer assignment is phase 3 and is a known hard problem on its own.
gEDA Toporouter's known weakness: "poor results on dense 2-layer boards with tight
constraints" [4]. The reason: with only 2 layers, topological freedom is limited, and the
sketch may require many forced via crossings.

**Python complexity:** Three sub-systems:
- CDT of board (Python: scipy.spatial.Delaunay is unconstrained; bmmeijers/tri supports
  constrained CDT but is lightly maintained; triangle library via Python bindings is more
  robust)
- Rubber-band sketch generation (~500-800 LOC)
- Funnel/shortest-path realization (~300-500 LOC)
Total: ~1,500-2,500 LOC for a minimal implementation, but three independent geometry
subsystems means three independent classes of bugs.

**Open source prior art:** gEDA Toporouter (C, GPL, available in pcb repository [4]);
Ruby-PCB-Router (Ruby, implements Dayan thesis [5]); Situs in Altium (proprietary); TopoR
(proprietary). No Python implementation found.

**Staged rollout:** Partial — individual nets can be routed topologically, but the
congestion signal interaction makes net-by-net mixing with grid routing awkward.

---

### Approach 3: Visibility Graph

**How it works:**
Minkowski-inflate all obstacles; extract the vertices of the union boundary; add source and
destination pads; build a visibility graph (edges between all mutually-visible vertex pairs);
run Dijkstra/A* on the graph.

**Connectivity:** Optimal — finds the true Euclidean shortest path in free space.

**Legality:** Legal by construction (routes through the Minkowski-inflated free space).

**Congestion:** Multi-net negotiation is the critical weakness. The visibility graph must be
rebuilt (or at least augmented) each time a net is routed and becomes a new obstacle. For a
200-net board, this is ~200 graph rebuilds. pyvisgraph benchmark: 554s for one build at
4,335 obstacle points [8]. Even with incremental update, this is prohibitive for multi-net
PCB routing.

**Python complexity:** Low for single-net; high for multi-net. pyvisgraph (MIT, well-
maintained) handles the single-net case out of the box. Multi-net requires building an
incremental update mechanism not present in any surveyed Python library.

**Staged rollout:** Viable for a single-net Spike-0 validation only.

---

### Approach 4: CDT Navigation Mesh

**How it works:**
Build a constrained Delaunay triangulation of the free space between all obstacles. The
triangulation is a navigation mesh: each triangle is a node; A* runs over triangles; the
funnel algorithm computes the shortest path through the chosen triangle corridor. When a net
is routed, its trace becomes a new obstacle and the CDT is updated incrementally (Delaunay
edge flips).

**Connectivity:** Near-optimal (the funnel algorithm finds the true shortest path through
the chosen corridor, but the corridor choice by A* over triangles is only approximately
optimal for curved obstacles).

**Legality:** Legal by construction if obstacles are Minkowski-inflated before CDT
construction.

**Congestion:** Better fit than visibility graph: history cost can be attached to CDT
triangles (which form a spatial tiling). Congestion pricing over triangles is conceptually
close to pricing over grid cells.

**Multi-layer / vias:** Via transitions = a new polygon hole on each layer, triggering CDT
update on both layers. Manageable.

**Python:** scipy.spatial.Delaunay is unconstrained (cannot handle polygon holes). The
`triangle` library (Python bindings to J.R. Shewchuk's code, MIT) is the most robust
option for constrained CDT with holes in Python. Incremental update is not exposed in the
Python API — requires manual edge-flip implementation (~200-400 LOC).

**Open source prior art:** Game engine navmesh (e.g., Recast/Detour, C++); jdxdev RTS
navmesh blog [12]; no PCB-specific Python CDT navmesh found.

---

## Shortlist (Top 2)

### Top Pick: Convex Expansion Rooms (Approach 1)

**Why:** This is the direct architectural equivalent of what Freerouting does, adapted to
Python. It addresses BOTH the connectivity gap (no grid over-estimation) and the legality
gap (rooms are Minkowski-inflated free space; traces are legal by construction). The
congestion pricing layer maps cleanly: room cost = history-based price, same signal as the
existing `multi.py` negotiated-congestion pricing. Via insertion is a room transition
between layers. Staged rollout is natural: each net can be routed gridlessly or fall back
to grid. The algorithm is a single coherent system, not three independent sub-systems.

**Caveats:**
- Shapely is the critical dependency. For large boards with many obstacles, polygon union
  and difference operations are O(n log n) to O(n²). Each room expansion is a polygon
  operation. For a 500-obstacle board this is manageable; for a 5,000-obstacle board,
  profiling is required. Alternative: use CGAL via Python bindings for exact arithmetic
  if Shapely's float-based GEOS proves numerically unstable near degenerate rooms.
- The A* node type changes from a grid cell to a (layer, room_polygon, entry_edge) tuple.
  The existing `astar.py` heuristic (Manhattan/Euclidean distance to target) still applies
  directly; only the neighbor expansion function changes.
- Freerouting's Java implementation is the canonical reference, not a Python port. The
  developer must re-implement room expansion from the algorithm description in [1], not from
  a copy-paste port.
- Zone-blindness (Freerouting's documented gap): the expansion-room approach does not model
  copper pours. TraceWise's congestion pricing already partially compensates, but pour-aware
  room generation (using rasterized pour geometry as an additional obstacle layer) is a
  required extension.

**First-run cost signal:** Spike-0 (2-3 days) — route a single 2-pin net on the mitayi
board using Shapely polygon rooms. Validates room generation, A* over polygons, and trace
emission without the full multi-net complexity. If the spike routes a single net cleanly
with zero DRC errors, the approach is validated. Full implementation: ~3-4 months.

---

### Runner-up: CDT Navigation Mesh (Approach 4)

**Why:** The CDT navmesh has the second-best fit for TraceWise's requirements. It produces
a spatial tiling of free space (like the expansion room approach) but uses a globally-
computed triangulation rather than lazily-expanded convex polygons. The triangle library
in Python provides a solid constrained CDT foundation. History-based pricing over triangles
maps to the existing congestion model.

**Caveats:**
- Python CDT libraries with holes and incremental update are sparse. The `triangle` library
  (J.R. Shewchuk) is the most robust but requires calling a C extension; incremental updates
  require manual edge-flip logic not exposed in the Python API.
- The funnel algorithm for shortest path through a triangle corridor adds ~300 LOC.
- CDT quality degrades near very thin clearance gaps (slivers), requiring additional
  mesh-quality constraints that interact with routing rules.
- Congestion pricing over triangles (which vary in size) is less uniform than pricing over
  grid cells; a triangle covering a wide-open area has the same "cost unit" as one in a
  tight corridor.

**First-run cost signal:** 3-4 day spike — build a CDT of a simplified board outline with
3-4 obstacle polygons, run A* over triangles, apply funnel algorithm to get a trace.
Validates the Python CDT pipeline.

---

## Comparison Table: Approach × Key Dimensions

| Dimension | Expansion Rooms (#1) | Topological/RB (#2) | Visibility Graph (#3) | CDT Navmesh (#4) | Hybrid Grid+Exact (#5) |
|-----------|---------------------|---------------------|----------------------|-----------------|----------------------|
| **Connectivity gap** | Eliminates (exact geometry) | Eliminates (exact geometry) | Eliminates (optimal) | Near-eliminates | Partial (emit-level only) |
| **Legality gap** | Eliminates (by construction) | Eliminates (by construction) | Eliminates (by construction) | Eliminates (by construction) | Closes DRC gap; grid error remains |
| **Congestion pricing fit** | Direct (room cost = history price) | Awkward (sketch phase doesn't price geometry) | Very hard (full graph rebuild per net) | Good (triangle cost = history price) | Exact (grid cells) |
| **Via / 2-layer** | Natural (layer-crossing room) | Hard (explicit layer assignment phase) | Must model per-layer | Natural (per-layer CDT) | Exact (existing logic) |
| **Python LOC estimate** | ~2,000-3,500 | ~1,500-2,500 (3 subsystems) | ~200 (pyvisgraph); ~1,000+ incremental | ~1,500-2,000 + funnel | ~200-300 (emit.py only) |
| **Numerical robustness** | Medium (Shapely/GEOS floats; degenerate thin rooms) | Medium (CDT slivers + funnel edge cases) | High (pyvisgraph tested; point visibility is stable) | Medium (CDT slivers, thin triangle filtering) | High (only endpoint nudge) |
| **Staged rollout** | Excellent (per-net) | Partial (per-net but congestion mixing hard) | No (cost-prohibitive for multi-net) | Good (per-net CDT update) | Excellent (purely additive) |
| **Open source prior art** | Freerouting (Java GPL) [3] | gEDA Toporouter (C GPL) [4]; Ruby-PCB-Router [5] | pyvisgraph (Python MIT) [8] | triangle lib (Python, MIT bindings) [11] | US6545288 patent [15] |
| **Solo 3-6mo feasibility** | Yes (3-4 months) | Marginal (4-6 months, 3 subsystems) | No for multi-net | Yes (2-3 months if CDT lib works) | Yes (1 week; wrong level) |
| **Verdict** | **Recommend** | Consider | Consider (Spike-0 only) | Consider | Reject as FAR (correct as patch) |

---

## Recommended Spike-0

**Route a single 2-pin net on the mitayi board using Shapely expansion rooms.**

Concretely:
1. Load the mitayi board via extract_pads (existing).
2. For a single chosen net, collect all obstacle polygons (pads + board boundary).
3. Buffer each obstacle by its clearance rule → Minkowski-inflated obstacles.
4. Compute free-space polygon for a bounded bounding box: `bbox.difference(union(inflated_obstacles))`.
5. Run a simplified A* where each "step" expands a convex sub-polygon of the free space
   toward the destination (lazy room generation: start from source pad, grow a convex
   polygon until it hits an obstacle halo, register it as a room node, enqueue its border
   edges as next states).
6. Extract the centerline trace from the room sequence.
7. Emit the trace via the existing emit.py pipeline and run DRC.

**What this validates:**
- Shapely polygon operations are fast enough for a single net (pass/fail timing gate)
- Room expansion logic produces a connected room graph (connectivity validation)
- The resulting trace has zero DRC errors (legality-by-construction validation)
- The A* cost function (trace length + history pricing) is compatible with room nodes

**What it does NOT validate:**
- Multi-net congestion negotiation (deferred to Spike-1)
- Via insertion across layers (deferred to Spike-2)
- Performance at full board scale (deferred to profiling after Spike-0 passes)

**Expected effort:** 2-3 days. If the spike produces a single DRC-clean gridless trace,
the approach is validated and the architect can specify the full engine interface.

---

## Honest Gaps

- **Freerouting source LOC count not verified directly** — the "~10kLOC Java search core"
  estimate comes from the tinycomputers.io analysis [1] and a secondary search; the GitHub
  repo structure was not cloned and counted. The estimate is plausible but unverified.
- **Springer paper on PCB routing with unstructured meshes (2025) paywalled** — could not
  read; noted in table as [14] with the title from search result only.
- **ResearchGate non-uniform-grid paper returned HTTP 403** — could not read.
- **IEEE SURF rubber-band sketch paper (979685) returned empty page** — topological routing
  is covered via gEDA Toporouter wiki [4], Ruby-PCB-Router [5], and Altium Situs [7].
- **Dayan 1997 PhD thesis (Semantic Scholar)** — page returned empty. Covered indirectly
  via Toporouter wiki citation and Ruby-PCB-Router README.
- **Python CDT libraries:** scipy.spatial.Delaunay is unconstrained (no holes). The
  `triangle` Python package (Shewchuk) was found via PyPI search but not WebFetch-verified
  for API details. bmmeijers/tri was found on GitHub but is lightly maintained. This is an
  active gap: a production-quality Python CDT-with-holes library may not exist, which would
  push Approach 4's complexity up.
- **Known-not-wanted pre-excluded:** Freerouting re-integration (documented in ablation),
  ML routing (no clearance guarantee). Finer uniform grid (previously analyzed: 4× memory,
  hits 600s wall clock). These are noted, not re-scored.
- **WebFetch budget:** 8 of 10 calls used (2 returned paywalled/403). Survey is complete.

---

## Open Questions for the Architect

1. **Room generation granularity vs. performance:** The lazy convex-room expansion
   approach generates rooms on demand during A*. Should the architect target fully-lazy
   (rooms only on the A* frontier) or pre-computed (all rooms built once at start)? The
   tradeoff is memory vs. per-step polygon operation cost. What is the acceptable per-step
   latency budget for the A* loop?

2. **Shapely vs. exact-arithmetic geometry:** Shapely/GEOS uses double-precision floats.
   For boards with 0.05mm clearance and 0.1mm-pitch pads, near-degenerate room corners
   may produce unstable polygon operations. Is the architect prepared to swap Shapely for
   a CGAL Python binding (e.g., scikit-geometry or PyGEOS) if float instability is
   measured in Spike-0, and what is the license/dependency policy for C extensions?

3. **History-based congestion signal on rooms:** The existing `multi.py` pricing
   increments a cost on grid cells. For expansion rooms, the equivalent is pricing on
   a room identifier. But rooms are generated lazily and may not be stable across rip-up
   iterations (a room's shape changes when a neighboring net is ripped up). How should the
   architect handle room identity across reroute passes? Options: (a) price on spatial
   hash of room centroid, (b) pre-tile the board into fixed "super-cells" and price those,
   (c) use Euclidean distance from obstacles as a continuous congestion field.

4. **Via insertion decision point:** In the expansion-room approach, a via is routed when
   the A* search decides to transition from layer 1 to layer 2 (a via-room node). The
   decision is currently made by the grid router's layer-switching cost. For the gridless
   router, where should the via cost be parameterized, and should it reuse the existing
   `via_cost` parameter or be replaced by a geometric cost (e.g., minimum detour to reach
   a legal via pad)?

5. **Compatibility with pour-synthesis:** TraceWise's F0 rasterizes copper pours into the
   obstacle grid. The gridless engine needs an equivalent: either (a) pre-inflate pour
   boundaries as polygon obstacles before room generation, or (b) run a post-route pour
   update after each net and rebuild affected rooms. Which lifecycle is compatible with the
   existing `bridge.py` pour-synthesis pipeline?

6. **Interface contract for staged rollout:** The current `route_net()` returns a
   `NetRoute` with grid-cell paths. For the gridless engine to drop in per-net, it must
   return the same `NetRoute` type (or a compatible subtype with a `to_grid_path()`
   adapter). Does the architect want to define this adapter contract before Spike-0, or
   defer until Spike-0 proves the geometry pipeline works?

---

## Recommended Next Agent

**Architect** — design integration of Top Pick (expansion-room substrate) following
Spike-0 validation. The architect should specify:
- The `GridlessNetRoute` result type and its adapter to the existing pipeline
- The room expansion interface (`expand_room(room, direction) -> Room | None`)
- The congestion pricing signal contract for rooms
- The Shapely dependency policy and fallback to CGAL if needed

The Spike-0 (2-3 days, single-net gridless trace on mitayi) should precede the full
architect design to validate that Shapely polygon operations are fast enough and that
the room expansion logic produces clean geometry before committing to the full engine spec.

---

## Sources

[1] https://tinycomputers.io/posts/the-mathematics-of-pcb-trace-routing.html
[2] https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/7937681
[3] https://github.com/freerouting/freerouting
[4] https://github.com/bert/pcb/wiki/Autorouters:-gEDA-pcb-Toporouter
[5] https://github.com/StefanSalewski/Ruby-PCB-Router
[6] https://ieeexplore.ieee.org/document/979685
[7] https://resources.altium.com/p/automated-pcb-routing-with-situs-topological-autorouter
[8] https://github.com/TaipanRex/pyvisgraph
[9] https://extremitypathfinder.readthedocs.io/en/latest/3_about.html
[10] https://www.toptal.com/python/computational-geometry-in-python-from-theory-to-implementation
[11] https://en.wikipedia.org/wiki/Constrained_Delaunay_triangulation
[12] https://www.jdxdev.com/blog/2021/07/06/rts-pathfinding-2-dynamic-navmesh-with-constrained-delaunay-triangles/
[13] https://www.ijcai.org/proceedings/2017/0070.pdf
[14] https://link.springer.com/article/10.1007/s11227-025-07569-0 (paywalled; title only)
[15] https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/6545288

---

## Structured Result

```json
{
  "status": "success",
  "summary": "Surveyed 5 families of gridless/free-space PCB routing substrates. Top pick is Convex Expansion Rooms (Freerouting-style): addresses both the connectivity gap and legality gap by construction, maps cleanly to the existing congestion history pricing, supports staged rollout per-net, and is a 3-4 month Python solo effort. Runner-up is CDT Navigation Mesh: similar spatial-tiling approach with better incremental update characteristics but weaker Python library support. Visibility Graph is viable only for single-net Spike-0 validation, not multi-net boards. Topological/rubber-band routing has the highest quality ceiling but three independent geometry subsystems make it too complex for the budget.",
  "files_changed": ["docs/research/FAR-gridless-routing-survey.md"],
  "files_read": ["docs/research/NEXT-exact-geometry-routing.md"],
  "issues": [],
  "assumptions": [
    "Shapely/GEOS polygon operations are fast enough for single-net Spike-0; multi-net performance must be profiled after Spike-0.",
    "The existing NetRoute result type can be extended to carry polygon-room paths without a full API break.",
    "Pour geometry can be pre-inflated as polygon obstacles before room generation, reusing the existing rasterize_pour output.",
    "The triangle Python package (Shewchuk) provides adequate constrained CDT with holes for Approach 4, but this was not WebFetch-verified.",
    "Freerouting LOC estimate (~10kLOC Java search core) is plausible from secondary sources but not directly measured."
  ],
  "sources_cited": [
    "https://tinycomputers.io/posts/the-mathematics-of-pcb-trace-routing.html",
    "https://github.com/freerouting/freerouting",
    "https://github.com/bert/pcb/wiki/Autorouters:-gEDA-pcb-Toporouter",
    "https://github.com/StefanSalewski/Ruby-PCB-Router",
    "https://resources.altium.com/p/automated-pcb-routing-with-situs-topological-autorouter",
    "https://github.com/TaipanRex/pyvisgraph",
    "https://extremitypathfinder.readthedocs.io/en/latest/3_about.html",
    "https://en.wikipedia.org/wiki/Constrained_Delaunay_triangulation",
    "https://www.jdxdev.com/blog/2021/07/06/rts-pathfinding-2-dynamic-navmesh-with-constrained-delaunay-triangles/",
    "https://www.ijcai.org/proceedings/2017/0070.pdf",
    "https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/6545288",
    "https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/7937681"
  ],
  "research_summary": {
    "goal": "Identify the best search substrate for a free-space gridless PCB router in Python eliminating quantization error for both connectivity and legality",
    "candidates_surveyed": 5,
    "verdict": "recommend_existing",
    "top_pick": "Convex Expansion Rooms (Freerouting-style) — Shapely polygon rooms + A* over room nodes",
    "runner_up": "CDT Navigation Mesh (triangle library + funnel algorithm)",
    "key_risk": "Shapely polygon operation performance at full board scale (multi-net, 500+ obstacles); and Python CDT-with-holes library maturity for runner-up",
    "artifact_location": "docs/research/FAR-gridless-routing-survey.md",
    "next_agent_hint": "architect"
  },
  "recommended_spike": "Route a single 2-pin net on the mitayi board using Shapely expansion rooms (Approach 1): load obstacles, Minkowski-inflate by clearance, lazily expand convex free-space rooms from source to destination, run A* over rooms, emit centerline trace, run KiCad DRC. Pass criterion: zero DRC errors, runtime < 5s for single net. 2-3 day effort.",
  "open_questions": [
    "Room generation strategy: fully lazy (on A* frontier) vs pre-computed (all rooms at start) — tradeoff is memory vs per-step polygon op cost; architect to decide acceptable per-step latency budget",
    "Shapely vs. exact-arithmetic geometry (CGAL/scikit-geometry): what is the dependency policy for C extensions if float instability is measured in Spike-0?",
    "Congestion pricing signal on rooms: rooms are lazily generated and may change shape across rip-up iterations — architect to decide room identity strategy (spatial hash of centroid, super-cell pre-tiling, or continuous Euclidean congestion field)",
    "Via insertion decision point: gridless via cost parameterization — reuse existing via_cost or replace with geometric cost (minimum detour to reach legal via pad)?",
    "Pour-synthesis lifecycle: pre-inflate pour boundaries as polygon obstacles before room generation, or rebuild affected rooms after each net?",
    "Interface contract for staged rollout: define GridlessNetRoute adapter to existing NetRoute type before Spike-0, or defer until geometry pipeline is validated?"
  ],
  "next_agent_hint": "architect"
}
```
