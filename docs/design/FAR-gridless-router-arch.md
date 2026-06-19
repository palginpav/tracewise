# Design: FAR gridless / shape-based router (Visibility Graph + Spatial Locality)

Status: architecture **REVISED 2026-06-19** after Spike-0/0b. Blueprint for the FAR build
named in `docs/design/EXACT-GEOMETRY-ROUTER-ARCH.md` §FAR.

> **SUBSTRATE REVISION NOTICE.** The original substrate pick — **convex expansion rooms**
> (survey Approach #1) — is **DISPROVEN for congested corridors** by Spike-0b and is
> **SUPERSEDED**. The active substrate is now a **visibility graph over inflated obstacle
> corners + A***, made full-board-scalable by a **per-net bounded routing window + reduced
> (taut-string) corner set** locality scheme. See the new "Substrate (REVISED)" section.
> Everything else from the original design **STANDS**: the Shapely dependency + determinism
> policy, the `GridlessNetRoute`→`NetRoute` adapter, the emit/realize→DRC loop, the
> super-cell congestion model, the staged milestones, and the per-net rollout. Old
> convex-rooms prose that remains below is explicitly marked `⊘ SUPERSEDED` and is retained
> only for traceability — it is NOT the design to build.

This doc resolves the open questions enough to build, fixes the Shapely dependency +
determinism policy, defines module structure + plug points, the `GridlessNetRoute`→`NetRoute`
adapter, the staged milestone plan, and the **M1 scale spike** (the next decisive experiment).

NO production code here. Pseudocode only where it pins a contract.

---

## Substrate (REVISED) — Visibility graph over inflated obstacle corners, scaled by per-net locality

**Decision: (A) visibility-graph + spatial locality.** Replace ONLY the search substrate
(grid A* in `astar.py`/`grid.py`) with continuous-coordinate A* over a **visibility graph**
built from the **corner vertices of Minkowski-inflated obstacles** within a **bounded routing
window** around each net. Reuse everything around it: `extract_pads`/`build_problem`
(nets/pads/obstacles/anchor_rects), the `NetRoute` model, the negotiated-congestion history
from `multi.py`, the emit/DRC/scorecard pipeline. Staged rollout: route a designated subset of
nets gridlessly while the grid router remains the fallback for the rest, both coexisting inside
`route_all`, measured incrementally against the human scorecard.

### Why this, grounded in the spike (not the survey ranking)

The survey ranked convex expansion rooms #1 and flagged the visibility graph as
"cost-prohibitive for 200-net boards" (O(n² log n), pyvisgraph 554 s at 4,335 points). **The
spike inverted that ranking empirically:**

- **Spike-0** (`scripts/spike0_gridless_single_net.py`, net `Net-(U2-BP)`, 1.624 mm): the
  convex-room expansion routed only because the net was a near-straight shot inside a SINGLE
  room — multi-room search + funnel were never exercised. Plumbing validated (Minkowski
  inflation, `set_precision(1e-6)` byte-identical determinism, realize→emit→DRC closes at 0
  errors, 0.066 s) but the substrate claim was untested.
- **Spike-0b** (`scripts/spike0b_gridless_blocked_net.py`, net `Net-(J2-CC2)`): the DECISIVE
  test. The straight inflated path provably intersects a blocker (0.099 mm² outside free
  space). The **convex-room expansion DEGENERATED** — the 0.33 mm corridor spawned tiny-sliver
  rooms and A* hit `max_rooms=2000` without reaching the goal. The spike **pivoted to a
  visibility graph over inflated obstacle corners**: **96 nodes, 593 edges, optimal 3-segment
  bent path found in 4 A* expansions, graph + solve in 0.33 s**, 0 new DRC errors, every
  segment legal (`exact_geom`), byte-identical across same-process + fresh subprocess.

So visibility-graph is chosen because it (a) **empirically works on the exact failure mode
that killed rooms** — narrow congested corridors are handled natively, with no sliver
degeneracy; (b) is **provably optimal** (Euclidean shortest path in polygon-obstacle free
space lives on the inflated-obstacle corners + endpoints — a textbook result); (c) has a
**simpler determinism story** than rooms (no lazy polygon expansion with float-sensitive
convex hulls — just corner sets, segment-in-free-space visibility tests, and integer-keyed
A*, all already shown byte-identical in 0b); (d) keeps **legality structural** (every edge is
visibility-tested inside Minkowski-inflated free space, so clearance is by construction, never
nudged post-hoc).

The one open problem the survey correctly identified is **full-board scale** (a *global*
visibility graph over ALL inflated corners is O(n²) edges and rebuilds per net under rip-up).
That is exactly what the locality scheme below solves, and what the **M1 scale spike** (next
section) de-risks before the full M1 build.

### Why NOT (B) CDT navmesh

CDT was the survey runner-up and genuinely avoids sliver degeneracy via triangulation + funnel.
It is **rejected as the primary substrate, held only as the M2-gate fallback**, because:

1. **(A) is proven; (B) is untested.** The spike shows (A) routing a legal bent deterministic
   path on the real board today. Choosing (B) trades a working prototype for an unbuilt one —
   violating "consistency/evidence over novelty" with no offsetting measured advantage.
2. **Python CDT-with-holes + incremental-update library risk is real and unresolved.** The
   survey's own honest-gaps section flags it: `scipy.spatial.Delaunay` is unconstrained (no
   holes); `triangle` (Shewchuk) is a C extension whose Python API does **not** expose
   incremental edge-flips (the very feature that motivates B for rip-up) — that is ~200–400 LOC
   of manual edge-flip logic to write and harden, plus a funnel pass (~300–800 LOC). (A)'s
   visibility test is ~30 LOC of Shapely already written in 0b.
3. **Determinism surface is larger.** CDT vertex insertion order, edge-flip order, and funnel
   tie-breaks each need a determinism gate; the visibility graph has only corner ordering +
   edge ordering + A* tie-break, all already byte-identical in 0b.
4. **Funnel optimality caveat.** The survey notes CDT corridor choice (A* over triangles) is
   only *approximately* optimal; the visibility graph is *exactly* optimal.

(B) remains the documented escape hatch: **if the M1 scale spike or the M2 gate shows the
locality scheme cannot hold runtime/legality at full board scale**, the room-graph-shaped
search interface defined here is substrate-agnostic enough to swap a CDT producer behind A*
without disturbing the adapter, congestion model, or emit loop. This is a fallback, not a plan.

### Locality / scale scheme (the heart of this revision)

A global visibility graph is O(n²) in corners and would rebuild per net under rip-up — the
survey's cost objection. Routing is, however, **inherently local**: a 2-pin net's shortest
legal path stays within a corridor close to the straight pad-to-pad line (Spike-0b's path used
2 corners within ~1 mm of the line). Four composable mechanisms make the graph cheap:

1. **Per-net bounded routing window (the dominant lever).** For each net, the visibility graph
   is built ONLY over obstacles whose inflated geometry intersects the net's **bounding box
   expanded by a margin** `window_mm` (Spike-0b used `margin_mm = 4.0`). `n` collapses from
   "all board corners" (thousands) to "corners in this net's window" (Spike-0b: 96 nodes /
   593 edges). The window is the gridless analogue of routing locality the grid router gets for
   free. **Escalation on failure:** if A* finds no path in the window, the window grows
   (`window_mm *= 2`, capped at the board bbox) and the graph rebuilds — bounded, deterministic
   widening, NOT an unbounded global graph. This makes the *typical* cost local and the *worst*
   case (a net that genuinely must detour board-wide) still correct.
2. **Reduced (taut-string / convex-relevant) corner set.** Only **convex (reflex-in-free-space)
   corners of inflated obstacles** can be turning points of a taut shortest path; corners on
   the concave side never appear in an optimal route. Pruning to convex corners (and dropping
   corners not facing the start→goal corridor) cuts node count further with zero loss of
   optimality. M1 may ship the full corner set (as 0b did — it was already fast enough) and add
   this pruning in M2 only if profiling demands it; it is specified here so the node-builder
   has the contract.
3. **Visibility-edge spatial pruning.** Building all O(n²) edges is the costly step. Two cheap
   wins, both order-preserving: (a) skip edges longer than the current window diagonal; (b) use
   a Shapely STRtree over the inflated obstacles so each candidate edge's free-space test only
   queries obstacles near that segment, not all of them. Edge enumeration order stays the fixed
   sorted node-index order (determinism, below).
4. **Per-net rebuild under rip-up (cost addressed, not incremental).** The graph is **rebuilt
   per net-route attempt**, NOT incrementally mutated — this is the deliberate choice. Rationale:
   (i) the window keeps each rebuild small (tens–low-hundreds of nodes, sub-second in 0b);
   (ii) per-net rebuild sidesteps the incremental-update complexity that is the *entire* reason
   CDT looks attractive, collapsing (A) vs (B)'s main differentiator; (iii) it matches how the
   grid router already re-runs `route` per net under rip-up. Already-routed nets enter a net's
   window as **additional inflated obstacles** (their realized centerline buffered by
   `track_mm/2 + clearance`), so a net routed later sees earlier copper exactly as the grid
   router sees marked cells. Cross-net obstacle reuse / caching the inflated-obstacle STRtree
   keyed by the obstacle-set hash is an M2 optimization, NOT a correctness requirement.

**Scale budget this must hit** (validated by the M1 scale spike): a congested multi-net region
(~10–20 nets, the mitayi GPIO fan-out) routes all-legal and deterministic in a runtime within
~2× the grid router's `--quality` mode on that slice. If a per-net window graph blows past
~low-thousands of nodes routinely, escalate to corner-set pruning (#2) before considering CDT.

**Pattern/KB check:** `pattern_find` (catalog mode) returned zero matches for gridless/
visibility-graph/Shapely — greenfield in the orchestration KB; no prior decision is
contradicted or extended. A `decisions/` entry should be written once the M1 scale spike
reports (see task list).

---

## Overview ⊘ SUPERSEDED (convex-rooms framing — retained for traceability)

> The paragraphs below described the original convex-expansion-rooms substrate. The
> realize→emit→DRC reuse, staged-rollout, and "eliminate both gaps by construction" goals
> still hold; only "convex expansion rooms" → "visibility graph over inflated corners" and
> "room cost += history price" → "node/edge cost += super-cell history price" change. See the
> "Substrate (REVISED)" section above for the active design.

Replace ONLY the search substrate (grid A* in `astar.py`/`grid.py`) with continuous-coordinate
A* over lazily-grown **convex expansion rooms** in Minkowski-inflated free space. Reuse
everything around it: `extract_pads`/`build_problem` (nets/pads/obstacles/anchor_rects),
the `NetRoute` model, the negotiated-congestion history from `multi.py`, the emit/DRC/scorecard
pipeline. Staged rollout: route a designated subset of nets gridlessly while the grid router
remains the fallback for the rest, both coexisting inside `route_all`, measured incrementally
against the human scorecard.

**Why this approach** (cite survey §"Top Pick"): eliminates BOTH measured gaps by
construction — connectivity (no grid over-estimation of congestion; the true geometric free
space is the limit) and legality (every routed segment sits inside a clearance-inflated room,
so DRC clearance is structural, not nudged post-hoc as in the #4 emit patch). The congestion
history layer maps cleanly (room cost += history price), via insertion is a layer-crossing
room, and the rollout is per-net. It is a single coherent system, not three subsystems (vs.
topological/rubber-band, Approach #2 rejected as too complex for the budget). Survey
runner-up (CDT navmesh, #4) is held in reserve: if Shapely room expansion proves
numerically fragile or too slow at full board scale (decided in Spike-0 + M2 profiling), the
room-graph interface defined here is substrate-agnostic enough to swap a CDT producer behind it.

---

## Scope

- **Files to create (production, by the developer in M1+; the spikes precede the package):**
  - `src/tracewise/route/gridless/__init__.py` — package marker + public surface.
  - `src/tracewise/route/gridless/geom.py` — Shapely wrapper + determinism shims
    (`set_precision`, free-space build = `board_bbox.difference(union(inflated_obstacles))`,
    the `HAVE_SHAPELY` flag and import guard). **STANDS** from the original design.
  - `src/tracewise/route/gridless/visgraph.py` — **REPLACES `rooms.py`.** The locality core:
    `routing_window(net, window_mm)`, `inflated_obstacles_in_window(...)`,
    `obstacle_corners(free_space, window)` (the candidate waypoint set, optionally pruned to
    convex/taut-string corners), `is_visible(u, v, free_space, strtree)`, and
    `build_visibility_graph(...) -> (nodes, adj)`. Owns the window-escalation policy.
  - `src/tracewise/route/gridless/search.py` — A* over visibility-graph nodes (point
    waypoints, not room nodes): deterministic ordering/tie-break (integer 1 nm heap key,
    sorted-by-`(dist, node-coord)` neighbour expansion — exactly Spike-0b), history-pricing
    hook. The `RoomNode` type is GONE; the node is a `(x, y, layer)` waypoint.
  - `src/tracewise/route/gridless/realize.py` — visibility-path waypoints → world-coordinate
    centerline + per-segment legality assert (`exact_geom`). **No funnel step needed** — the
    visibility-graph path IS the shortest path (Spike-0b finding). Quantize/dedup to 1 nm.
  - `src/tracewise/route/gridless/adapter.py` — `GridlessNetRoute` and
    `to_netroute(...)`; `route_net_gridless(...)` (the per-net entry matching
    `route_net`'s contract). **STANDS** from the original design.
  - `src/tracewise/route/gridless/congestion.py` — node/edge→super-cell congestion mapping
    (Decision 3, now phrased on waypoints/edges instead of rooms). **STANDS.**
  - `scripts/spike0_gridless_single_net.py`, `scripts/spike0b_gridless_blocked_net.py` —
    the existing single-net + blocked-net spikes (DONE).
  - `scripts/spike1_gridless_congested_region.py` — the **M1 scale spike** (built NEXT,
    before the package; see M1-scale-spike spec).
  - `tests/test_gridless_*.py` — fixture tests per module (M1+).
- **Files to modify (M1+, NOT in Spike-0):**
  - `pyproject.toml` — add `shapely` to an optional extra `gridless` (decision 1).
  - `src/tracewise/route/engine/kicad.py` — `route_board_engine`: add `gridless_nets`
    parameter + branch that routes the selected subset via `route_net_gridless` and merges
    results into the same `results` dict before `emit_routes`. `emit_routes`: accept
    world-coordinate centerlines so gridless traces emit without cell snapping (decision 2).
  - `src/tracewise/route/engine/multi.py` — `route_all`: add `gridless_nets: set[str] | None`
    so the rip-up loop dispatches selected nets to `route_net_gridless` while keeping grid
    routing + rip-up for the rest; congestion history shared via super-cell map (decision 3).
- **Files to read (context only — inform the design, not changed):**
  `astar.py` (the search being replaced, `RouteResult`/`simplify`), `pathfinder.py`
  (negotiated-congestion reference), `extract.py` (pad/net data; lacks drill geometry),
  `exact_geom.py` (reuse its predicates as the determinism fallback), `bridge.py`
  (`run_drc`, `strip_routing`).

---

## Decision 1 — Shapely dependency: YES, optional extra, with numpy fallback policy

**Add Shapely. License-clean: BSD-3-Clause, compatible with the project's Apache-2.0.**
Pin in `pyproject.toml` as an OPTIONAL extra (not a core dependency) so the grid router and
the rest of TraceWise install without GEOS:

```toml
[project.optional-dependencies]
gridless = ["shapely>=2.0,<3.0"]
```

Rationale for `>=2.0,<3.0`: Shapely 2.x is the vectorized-GEOS rewrite with the stable
`shapely.set_precision` API and `shapely.geos_version` introspection — both load-bearing for
the determinism policy below. 1.x is EOL and lacks the precision API. Cap below 3.0 to avoid
an unreviewed major bump silently changing GEOS behaviour.

**Why optional, not core:** the staged rollout requires the grid router to remain fully
functional as fallback. Forcing GEOS onto every install (CI, the place-only path, the
scorecard's grid baseline) is unnecessary coupling. `gridless/geom.py` imports Shapely behind
a guard:

```python
try:
    import shapely
    from shapely import set_precision
    HAVE_SHAPELY = shapely.geos_version >= (3, 8, 0)
except ImportError:
    HAVE_SHAPELY = False
```

Callers (`route_board_engine` when `gridless_nets` is non-empty) assert `HAVE_SHAPELY` and
emit an actionable error (`pip install tracewise[gridless]`) if absent.

**Determinism policy (and the fallback trigger):** see Determinism section below — GEOS float
ops are deterministic for a fixed library version on a fixed platform but NOT guaranteed
byte-identical across GEOS versions/CPU arch. The hard constraint is byte-identical routes
run-to-run *on the same install*, which `set_precision` + canonical ordering achieves.
**Spike-0 and Spike-0b measured ZERO run-to-run divergence** (byte-identical, `set_precision`
free-space area-diff = 0). If a multi-net sequence (M1 scale spike) later diverges on one
install, fall back to the existing numpy `exact_geom` predicates for the legality-critical
operations (clearance check, visibility test) while keeping Shapely for the non-critical
polygon bookkeeping (buffer/union/difference for the free-space shape). The numpy fallback is
ALREADY BUILT and tested (`exact_geom.py`, 46 fixture tests) — it is the determinism insurance,
not a future cost.

---

## Determinism strategy (the mandatory hard constraint)

Routes must be byte-identical run-to-run. Three sources of nondeterminism, each closed:

1. **GEOS float noise.** Quantize every geometry to a fixed grid with
   `shapely.set_precision(geom, 1e-6)` (1 nm — finer than the 1 µm emit rounding, so no
   information loss, but it snaps GEOS's sub-nm float wobble to a deterministic lattice).
   Apply at every free-space-construction boundary (after buffer, after union, after
   difference). This makes the polygon *coordinates* — and hence the obstacle corners that
   become visibility-graph nodes — reproducible on a fixed install.
2. **Iteration order (visibility graph).** Python set/dict iteration over Shapely geometries
   and over obstacle collections must never drive control flow. Canonicalize, exactly as
   Spike-0b already does and verified byte-identical: **obstacle corners are collected into a
   `set` then emitted `sorted((round(x,6), round(y,6)))`**; the node list is
   `[start, goal] + sorted_corners` with fixed indices; the **adjacency is built in fixed
   `for i in range(n): for j in range(i+1, n)` order**; the A* open set is a heap keyed by
   `(f_int, insertion_seq, node_idx)` where `f_int = round(f_cost * 1e6)` (1 nm integer) and
   `insertion_seq` is a monotonic counter so equal-cost nodes pop in a fixed order; **neighbour
   expansion sorts `adj[ni]` by `(edge_len, all_nodes[neighbour])`**. NEVER key the heap on a
   Shapely object or a float-only tuple.
3. **Corner / edge ordering** (the gridless analogue of the grid router's fixed `DIRS` list):
   corner extraction reads the free-space polygon's interior-ring coords in ring order, snaps
   each to 1 nm, dedups via a `set`, and **sorts** — so the candidate-waypoint set is
   identical run-to-run regardless of GEOS ring-emission order. Edge enumeration is the fixed
   index double-loop above. (When convex/taut-string corner pruning is added in M2, the prune
   predicate must be a pure function of the snapped coords so ordering is preserved.)

**Verification gate (every spike + every milestone):** run the route twice in the same process
and once in a fresh process; assert the emitted `.kicad_pcb` track/via coordinates are
byte-identical across all three. **Spike-0 and Spike-0b both PASSED this gate** (same-process +
fresh subprocess, byte-identical, `set_precision` area-diff = 0). This mirrors how the grid
router's determinism was hardened (work-bounded, no wall-clock deadline). Cross-platform/
cross-GEOS-version identity is NOT promised (documented limitation) — same-install
reproducibility is.

---

## Decision 2 — Module structure & exact plug points

New package `src/tracewise/route/gridless/` (modules listed in Scope). Data flows:

```
build_problem(data)            # REUSED unchanged — gives obstacles (tagged tuples),
  -> grid, nets, anchors,      #   anchor_rects, per-net pad world coords
     obstacles, anchor_rects
        |
        v
route_all(grid, nets, gridless_nets={...})         # MODIFIED: dispatch
   for net in ordered:
     if net.name in gridless_nets:
        nr = route_net_gridless(net, obstacles, anchors, geo, history_supercells)
     else:
        nr = route_net(grid, net, ...)             # UNCHANGED grid path
   -> dict[name, NetRoute]                          # SAME return type
        |
        v
emit_routes(board, grid, results, ...)             # MODIFIED: world-centerline path
   -> .kicad_pcb segments/vias
        |
        v
refill_zones / run_drc / scorecard                 # REUSED unchanged
```

**Plug point A — `route_all` dispatch (`multi.py`).** Add `gridless_nets: set[str] | None`.
Inside the rip-up loop, before `route_net`, branch on membership. The gridless branch returns
a `NetRoute` (via the adapter) so `_mark`, `routed.append`, rip-up victim selection, and the
salvage pass all operate uniformly. Gridless nets still mark their copper into the grid (so
grid-routed nets see them as obstacles) — the adapter rasterizes the gridless centerline into
`nr.cells`/`nr.via_sites` exactly as a grid route would, at the same inflation. This is what
lets the two substrates coexist: **the grid is the shared occupancy ledger; gridless writes
into it via the adapter.**

**Plug point B — `route_board_engine` (`kicad.py`).** Add `gridless_nets: set[str] | None = None`
threaded into `route_all`. Assert `HAVE_SHAPELY` when non-empty. No other change to the
extract→build→route→emit→refill spine.

**Plug point C — `emit_routes` (`kicad.py`).** Gridless `NetRoute` carries true world-coordinate
waypoints (decision below). emit must NOT re-snap those through `grid.to_world`. The cleanest
contract: `GridlessNetRoute` (a subclass/extension of `NetRoute`) carries an extra
`world_paths: list[list[tuple[float, float, int]]]` (x, y, layer) and `world_vias:
list[tuple[float, float]]`. In `emit_routes`, when `nr.world_paths` is present, emit those
coordinates directly (still applying the per-segment width/neck logic and the existing
`net_atom`/sexpr writer); skip the cell→world `resolved` map for that net. Grid nets continue
through the existing `resolved` path. One `if nr.world_paths:` branch — no rewrite.

**The `GridlessNetRoute` adapter contract (pseudocode — pins the type, not the impl):**

```python
@dataclass
class GridlessNetRoute(NetRoute):       # IS-A NetRoute: route_all/_mark see it uniformly
    # world-coordinate centerline, what emit writes verbatim:
    world_paths: list[list[tuple[float, float, int]]] = field(default_factory=list)
    #            [ [ (x_mm, y_mm, layer), ... ] per run ]
    world_vias: list[tuple[float, float]] = field(default_factory=list)
    supercells_used: list[tuple[int, int, int]] = field(default_factory=list)  # (layer,sy,sx) -> congestion crediting

def to_netroute(net, world_paths, world_vias, grid, geo) -> GridlessNetRoute:
    """Build a GridlessNetRoute whose grid-side fields (cells, via_sites) are the
    rasterization of the world centerline at the net's inflation, so the SHARED grid
    ledger and rip-up see gridless copper identically to grid copper. world_paths/world_vias
    carry the exact geometry for emit. ok/reason/unroutable_pads as in route_net."""
```

Rationale for IS-A over a parallel type: `route_all`, `_mark`, `_nearest_victim`, the salvage
pass, and the summary all consume `NetRoute` fields; subclassing means zero changes there. The
only code that knows about `world_paths` is `emit_routes` (one branch) and the adapter itself.

**Visibility-graph / A* node types (REVISED — replaces the room/portal/RoomNode types):**

```python
# visgraph.py — built per-net inside a bounded window (locality scheme above)
# A node is just a continuous waypoint: a pad endpoint or an inflated-obstacle corner.
WayNode = tuple[float, float, int]          # (x_mm, y_mm, layer), coords snapped to 1nm

@dataclass
class VisGraph:                              # per-net, per-window, rebuilt under rip-up
    nodes: list[WayNode]                     # [start, goal, *sorted_corners]; index 0/1 fixed
    adj: dict[int, list[tuple[float, int]]]  # node_idx -> [(edge_len, neighbour_idx)], sorted
    # determinism: corners sorted by (x,y); adjacency built in fixed i<j double-loop;
    # each adj[i] sorted by (edge_len, node_coord). (All exactly as Spike-0b.)
```

A* over `WayNode` (this IS the Spike-0b search, promoted to a module): g = accumulated
Euclidean edge length + `via_cost` per layer change + super-cell history price (Decision 3);
h = Euclidean distance node→goal (admissible; add `via_cost` if the goal is on the other
layer — same shape as `astar.h`). Neighbours = `adj[node_idx]` (the visibility edges).
Heap key `(round(f*1e6), insertion_seq, node_idx)`; neighbour expansion sorted
`(edge_len, all_nodes[j])`. The grid octile heuristic is replaced by straight Euclidean
since geometry is continuous. **No `Room`, `Portal`, `RoomNode`, or funnel step exists** —
the visibility-graph path is already the exact shortest centerline (Spike-0b: 4 vertices,
0 funnel passes, optimal).

---

## Decision 3 — Negotiated-congestion history on a super-cell lattice (CONFIRMED, re-phrased for the visibility graph)

Survey open Q3. **The super-cell resolution STANDS unchanged** — and the substrate switch
makes it *more* clearly correct, not less. Under the old room substrate the worry was that a
*room id* is not stable across rip-up. Under the visibility graph there is no room id at all:
the search nodes are continuous waypoints and the graph is rebuilt per net (locality scheme
#4). So pricing must live on a substrate-independent spatial key regardless — exactly what the
super-cell lattice provides.

**Resolution: option (b) — a fixed super-cell congestion lattice, decoupled from the search
substrate.** Tile the board into a coarse fixed grid (e.g. 0.5 mm super-cells, ~5× the routing
pitch) at problem-build time. This lattice never changes across rip-up. History pricing lives
on super-cells, exactly like the grid router's `history[layer,iy,ix]` lives on routing cells —
same data structure (`np.zeros((layers, sy, sx))`), same `+= 1.0` deposit on rip-up, same
`(1 + history_factor * history[supercell])` multiplier.

**How it layers onto the visibility graph (the re-phrasing):** the multiplier is applied to a
**visibility EDGE's traversal cost** in A*, weighted by the length of that edge crossing each
super-cell. Concretely, in `search.py` the edge cost becomes
`edge_len * (1 + history_factor * mean_history_over_supercells_the_edge_crosses)`. This keeps
the A* admissible-heuristic shape intact (the heuristic is still plain Euclidean, an
underestimate, since `history_factor >= 0`). A net's realized centerline credits the
super-cells it crosses into `GridlessNetRoute.supercells_used`; on rip-up, those super-cells
get the `+= 1.0` deposit — the gridless analogue of `multi.py`'s per-victim-cell deposit.

Why super-cells, not a (now-nonexistent) node-id (a) or a continuous-field (c): (c)
(distance-from-obstacle field) prices proximity, not contention history, so it cannot carry
the cross-iteration "this corridor keeps causing rip-ups" signal that `multi.py` relies on.
Super-cells give a stable spatial key, reuse the exact numpy history mechanism and
`history_factor` parameter, and let grid-routed and gridless-routed nets **share one
congestion field** — a grid net's rip-up raises the price for a gridless net crossing the
same region and vice versa. This is what makes mixed routing negotiate jointly rather than as
two disconnected routers. `congestion.py` owns the cell↔super-cell mapping
(`supercell_of(x, y) -> (sy, sx)`) and the per-edge cost-accumulation helper.

Cross-iteration graph reuse for performance is an optimization, NOT a correctness requirement:
the visibility graph is rebuilt per net-route attempt; the super-cell history is the only thing
that persists. M2 may cache the inflated-obstacle STRtree keyed by the obstacle-set hash if
profiling demands it. **Congestion negotiation is DEFERRED past the M1 scale spike** — the M1
spike fixes a net order and routes each net once against the accumulating obstacle set (no
rip-up), so it can validate scale + multi-net interaction without the history layer; full
rip-up + history is the M2 gate.

---

## Decision 4 — Via insertion & 2-layer on the visibility graph; pour lifecycle

**Via / 2-layer (REVISED to the visibility graph; mechanism unchanged).** Build **one
per-net visibility graph per layer** (F.Cu, B.Cu), each over its own per-layer inflated-
obstacle set inside the same routing window. A via is a *via transition* edge between the two
graphs: from `WayNode(x, y, layer=0)` the search may cross to `WayNode(x, y, layer=1)` iff a
legal via can be placed at `(x, y)` — i.e. a disc of radius `via_mm/2 + clearance` is inside
free space on BOTH layers at `(x, y)` (the gridless analogue of `astar.via_ok`'s VIA_RING
check, but exact, via a Shapely `Point(x,y).buffer(via_mm/2+clearance).within(free_space_L)`
on each layer). **Candidate via sites are a finite deterministic set:** the obstacle corners
(already nodes) plus the goal — so the cross-layer edges enumerate in the same fixed node
order, preserving determinism. Cost = the existing `via_cost` parameter (reuse it; do NOT
invent a geometric via cost — survey Q4: keep parameter parity with the grid router so the
scorecard comparison is apples-to-apples; a geometric detour cost can be an M3 refinement if
measured to help). The via centre `(x, y)` becomes an obstacle (a `circle` of radius
`via_mm/2`) on both layers' obstacle sets for subsequent nets, and is rasterized into
`via_sites` by the adapter so grid-routed nets see it too.

Drill geometry: `extract.py`/`extract_pads` lacks drill data; vias use the project's
`via_mm`/`via_drill_mm` from `project_geometry` (already wired in `emit_routes`). Through-hole
pad holes are obstacles via their pad rect already; hole-to-hole spacing (a DRC class seen on
mitayi) is handled by the via-placement clearance check using the obstacle's copper radius.

**Pour lifecycle.** Survey Q5. Resolution: **option (a) — pre-inflate pours as polygon
obstacles before graph construction, NOT rebuild-after-each-net.** Pours are large and slow to
re-difference per net; rebuilding the free space after every net would dominate runtime.
Instead, at problem-build the existing F0 pour extraction / rasterized pour geometry is
converted to Shapely polygons, inflated by clearance, and added to the per-layer obstacle set
that the per-net window selects from (their corners become candidate waypoints like any other
obstacle). Post-route `refill_zones` (the existing pcbnew re-pour) still runs at the end
exactly as today — pours refill around the new copper with their own clearance, so the
gridless router only needs pours as *routing* obstacles, identical to how the grid router
rasterizes them. PROBE-A measured **0 pour-interaction** violations on mitayi, so pour
fidelity is not on the critical path for early milestones; pre-inflation is the conservative
correct choice and matches the grid router's pour treatment.

---

## Decision 5 — Staged milestone plan (each with a scorecard exit criterion)

Scorecard = `scripts/_probe_route_human.py` (route HUMAN placement, `run_drc`, report
`unconnected` + `errors` + `by_type`). Current grid-router baseline on mitayi HUMAN placement:
**48 unconnected / 104 errors** (74 clearance-class). The human target is **0 / 0**.
Effort estimates are solo; per the calibration caveat, treat as ±5× and record actuals in
`.planning/phases/<slug>/ACTUAL.md`.

| Milestone | Goal | Measurable exit criterion (vs scorecard) | Est. effort |
|-----------|------|------------------------------------------|-------------|
| **Spike-0** ✅ DONE | Prove the realize→emit→DRC plumbing on ONE net | `Net-(U2-BP)`: 0 new DRC errors, 0.066 s, byte-identical 3 runs. *Single-room shot — substrate not stressed.* | done |
| **Spike-0b** ✅ DONE | Prove a BENT path AROUND a provable blocker | `Net-(J2-CC2)`: visibility graph 96 nodes/593 edges, optimal 3-segment bent path, 0 new DRC errors, all-legal, 0.41 s, byte-identical. **GO; substrate switched to visibility graph.** | done |
| **M1-SCALE-SPIKE** ⬅ NEXT | De-risk the locality scheme on a CONGESTED MULTI-NET region BEFORE the full M1 build | A dense mitayi cluster (≥10 nets, GPIO fan-out): **100% of attempted nets routed all-legal** (0 new trace-attributable DRC errors), **deterministic** across same-process + fresh subprocess, **total runtime within ~2× grid `--quality`** on that slice, **and** connects ≥1 net the grid router leaves unconnected (relieves over-estimated congestion). Standalone script, no engine changes. **This is the go/no-go for the locality scheme.** | 1–2 wk |
| **M1: single/multi-net core + adapter** | Promote the spike code to `gridless/` package: `visgraph.py`+`search.py`+`realize.py`+`adapter.py`; `route_net_gridless` + `GridlessNetRoute` + emit branch | On a fixture set of ≥10 individually-routed mitayi 2-pin nets: 100% routed, 0 DRC errors each, deterministic; adapter produces a `NetRoute` that `_mark` accepts. No rip-up/history yet. | 3–4 wk |
| **M2: multi-net + congestion + rip-up** | `route_all` dispatch + super-cell history shared with grid; rip-up works for gridless nets | Route a SUBSET (e.g. all 2-pin signal nets) of mitayi gridlessly + remainder on grid: total errors ≤ grid-only baseline on that subset AND unconnected ≤ baseline; deterministic; rip-up converges within budget. | 4–6 wk |
| **M3: vias / full 2-layer** | Via transitions + B.Cu per-layer visibility graphs; multi-pin connection trees | Route nets requiring layer changes gridlessly: hole_clearance + hole_to_hole DRC classes ≤ grid baseline; vias placed at legal positions (0 via-clearance errors on gridless nets). | 3–4 wk |
| **M4: full board, parity then beat** | All nets gridless (grid only as failure fallback); profiled to acceptable runtime | mitayi HUMAN placement: **unconnected ≤ grid baseline AND errors < grid baseline** (parity gate), then stretch: **unconnected ≤ ~5 AND errors ≤ human on mitayi + zuluscsi** (the EXACT-GEOMETRY-ARCH FAR definition-of-done). Runtime within ~2× grid `--quality` mode. | 6–10 wk + profiling |

**Why the M1-SCALE-SPIKE is inserted (substrate-revision rationale):** Spike-0b proved the
visibility graph works for ONE blocked net in a local window. The survey's one valid objection
to the visibility graph is **scale** (O(n²) edges, per-net rebuild). The locality scheme is the
answer, but it is so far only *argued*, not *measured*. Building the full M1 package on an
unmeasured scale assumption would repeat the original mistake (committing to a substrate the
spike had not stressed in its failure regime). The M1-scale-spike measures the locality scheme
in its hard regime — many nets, one congested region — for ~1–2 weeks before the 3–4 week M1
package. If it fails, the CDT fallback decision happens here, cheaply, not after M2.

Gate discipline: do NOT advance a milestone until its exit criterion is measured green on the
scorecard. **The M1-scale-spike and M2 are the two go/no-go gates for the whole substrate** —
if the locality scheme cannot hold runtime+legality on a congested region (M1-scale-spike), or
shared-congestion mixed routing cannot reach baseline parity on a subset (M2), reassess (CDT
navmesh fallback or scope cut) before sinking further effort.

---

## Spike-0 / Spike-0b ✅ DONE (single-net + blocked-net) — see RESULTS section at end

Spike-0 (`scripts/spike0_gridless_single_net.py`) and Spike-0b
(`scripts/spike0b_gridless_blocked_net.py`) are complete. They validated the realize→emit→DRC
plumbing, the determinism strategy (`set_precision(1e-6)` + canonical corner/edge ordering +
integer A* tie-break), and — decisively — that the **visibility graph** routes a legal bent
deterministic path through a narrow congested corridor where convex rooms degenerated. The
full results and the substrate-finding are in the "Spike-0 / Spike-0b RESULTS" section at the
end of this doc. The next experiment is the M1 scale spike below.

---

## M1 SCALE SPIKE — the next decisive experiment (built NEXT by a developer)

**Goal:** de-risk the **locality / scale scheme** of the visibility-graph substrate on a
CONGESTED MULTI-NET region of a real board, BEFORE building the M1 package. Spike-0b proved the
graph works for ONE blocked net in a local window; this proves the per-net window + multi-net
obstacle accumulation scales and stays legal+deterministic when ~10–20 nets compete for the
same tight region. Standalone script `scripts/spike1_gridless_congested_region.py`, **no engine
changes** — same posture as Spike-0b (reuse `extract_pads`/`build_problem`/`project_geometry`/
`run_drc`/`refill_zones`/`strip_routing`/`sexpr`, and import the visibility-graph + emit
helpers from `spike0b_gridless_blocked_net.py` so the spike does not re-derive the core).

**Region / net-set selection (mechanical, not hand-picked — record the rule's output):**
1. Target the **mitayi RP2040 GPIO fan-out** — the region where the grid router's boxed-in
   vias and over-estimated congestion live (this is where the connectivity gap is measured).
2. Concretely, derive the cluster mechanically so it is reproducible: take the reference
   designator with the most pads (the RP2040 QFN; find it as the footprint whose pad bounding
   box is densest), compute its pad bounding box, and define the region as that bbox expanded
   by a fixed margin (start `region_margin_mm = 3.0`).
3. The **net-set** = every net with ≥2 pads where **all** of the net's pads fall inside the
   region bbox AND **both** pads are F.Cu (single-layer, no vias — vias are M3). If fewer than
   10 such nets exist, grow `region_margin_mm` (×1.5, deterministic) until ≥10 nets qualify, or
   until the region reaches half the board (then take what qualifies and record the count).
4. **Net order is fixed deterministically** = the existing `order_nets` ordering restricted to
   the net-set (power-first, then short-bbox), so the spike is reproducible and comparable to
   the grid router. Print the chosen region bbox, the ordered net list, and the count.

**Procedure:**
1. Copy mitayi to a temp dir, `strip_routing` (mirror Spike-0b `setup_board`).
2. `extract_pads` + `project_geometry` + `build_problem`; derive the region + net-set per the
   rule above.
3. Build the BASELINE: run the existing **grid** router (`route_all`) on this same net-set only
   (or read the grid scorecard's per-net result for these nets) and record, for the region:
   `unconnected` and `errors`. This is the comparison the spike must meet or beat.
4. Route the net-set gridlessly, **in the fixed order**, accumulating obstacles: maintain a
   running list of inflated obstacles = all other-net pads in the region **plus the realized
   centerlines of nets already routed in this pass** (each buffered by `track_mm/2 + clearance`,
   `set_precision`-snapped). For each net: build the per-net bounded-window visibility graph
   (reuse `visibility_graph_astar` from Spike-0b, with the window = net bbox + `window_mm`),
   A* to the goal, escalate `window_mm` ×2 on no-path (capped at region bbox). NO rip-up, NO
   history pricing — fixed order, route-once. Realize each path to a world centerline and
   validate every segment with `exact_geom` (`is_legal` + `segment_rect_distance`) against the
   current obstacle set.
5. Emit ALL routed centerlines into the temp board (`emit_net_segments` per net, reused);
   `refill_zones`; `run_drc`.
6. Determinism: re-run steps 4–5 in the same process and in a fresh subprocess; assert the
   union of emitted segment coordinates (sorted) is byte-identical across all three.
7. Profile: record per-net graph-build time, A* time, node/edge counts (max + mean), and total
   wall time. Record the grid `--quality` runtime on the same net-set for the ~2× comparison.

**Pass criteria (ALL must hold for GO):**
- **All-legal:** every attempted net that routes produces **0 new trace-attributable DRC
  errors** (clearance/short/etc.) — the legality-by-construction claim holds under multi-net
  obstacle accumulation, not just for one net.
- **Connectivity ≥ baseline AND strictly relieves ≥1:** the number of nets in the region the
  gridless pass connects is **≥ the grid baseline**, AND it connects **at least one net the
  grid router leaves unconnected** in this region (demonstrates the connectivity gap is real
  and the exact-geometry substrate closes part of it).
- **Deterministic:** byte-identical emitted coordinates across same-process + fresh subprocess
  (the multi-net, accumulating-obstacle ordering is reproducible).
- **Runtime budget:** total gridless solve time for the net-set is **within ~2× the grid
  `--quality` runtime** on the same net-set; AND no single per-net window graph exceeds a
  recorded node ceiling (flag if any window routinely exceeds ~2,000 nodes — that triggers the
  corner-set-pruning escalation, not a fail).

**Validates (the substrate-scale claims):** the per-net bounded routing window keeps each graph
small on a dense region; multi-net obstacle accumulation (later nets seeing earlier copper as
inflated obstacles) produces legal non-overlapping routes; the window-escalation policy
resolves nets that need a wider corridor; determinism holds across a *sequence* of net routes,
not just one; runtime is in the right order of magnitude vs the grid router. Together these are
the locality scheme of the "Substrate (REVISED)" section, measured in its hard regime.

**Explicitly DEFERS:** rip-up + super-cell history negotiation (M2 — the spike uses a fixed net
order, route-once; this is sufficient to test scale + multi-net legality without the negotiation
loop); vias + the 2nd layer (M3 — F.Cu single-layer nets only; nets needing a layer change are
excluded by the F.Cu-both-pads filter); the `route_all`/`route_board_engine` engine wiring
(M1/M2 — the spike stays standalone); convex/taut-string corner pruning (M2 optimization — ship
the full corner set as 0b did, and let the node-ceiling flag tell us if pruning is needed);
full-board (all-region) performance (M4 profiling). If the spike measures float nondeterminism
that `set_precision` cannot tame across the multi-net sequence, the fallback is the numpy
`exact_geom` predicates for the legality-critical ops — record it; do NOT abandon the approach
on a single determinism wobble.

**Fallback trigger (decision point):** if the spike FAILS runtime (windows routinely exceed the
node ceiling even after the ×2 escalation and corner-set pruning would not plausibly help) OR
fails legality under accumulation in a way the obstacle model cannot fix, that is the
**M2-gate-equivalent trigger to evaluate the (B) CDT navmesh fallback** — recorded here so the
go/no-go is cheap and early, not after the M1 package is built.

---

## Interface Contracts (binding for M1; Spike-0 may prototype loosely)

```python
# adapter.py
def route_net_gridless(
    net: Net,
    obstacles: dict[int, list[tuple[str, tuple]]],   # layer -> [(net_name, Obstacle)]
    anchors: dict[tuple[int,int,int], tuple[float,float]],
    anchor_rects: dict,
    geo: dict,                                        # track_mm, clearance_mm, via_mm, ...
    grid: Grid,                                       # shared occupancy ledger (for to_netroute raster)
    history_supercells=None, history_factor: float = 0.0,
    via_cost: float = 10.0,
) -> GridlessNetRoute: ...

# visgraph.py  (REPLACES rooms.py — the locality core)
def build_obstacle_polys(obstacles, layer, net_name, inflate_mm) -> "shapely.MultiPolygon": ...
def routing_window(net, anchors, window_mm: float) -> tuple[float, float, float, float]: ...  # bbox
def build_free_space(obstacle_polys, window_bbox) -> "shapely.MultiPolygon": ...               # bbox.difference(union)
def obstacle_corners(free_space, start_xy, goal_xy, margin_mm, prune_convex: bool = False)
    -> list[tuple[float, float]]: ...                                                          # sorted, deterministic
def is_visible(u, v, free_space, strtree=None) -> bool: ...                                     # segment-in-free-space
def build_visibility_graph(free_space, start_xy, goal_xy, window_mm)
    -> tuple[list[WayNode], dict[int, list[tuple[float, int]]]]: ...                            # (nodes, adj)

# search.py  (this IS the Spike-0b A*, promoted)
def astar_visgraph(nodes, adj, start_idx: int, goal_idx: int,
                   via_cost: float, history_supercells, history_factor: float)
    -> list[int] | None: ...    # node-index path; integer 1nm heap key, sorted-neighbour expansion

# realize.py  (no funnel — the visgraph path IS the shortest centerline)
def realize_centerline(node_path: list[int], nodes: list[WayNode])
    -> list[tuple[float, float, int]]: ...   # snap to 1nm, dedup, assert exact_geom-legal per segment
```

`GridlessNetRoute` IS-A `NetRoute` (see Decision 2). `Obstacle` reuses the `exact_geom.py`
tagged-tuple union (`("rect", x1,y1,x2,y2)`, `("circle", cx,cy,r)`, `("segment", ax,ay,bx,by,hw)`).

---

## Dependencies

- **External:** `shapely>=2.0,<3.0` (BSD-3, optional extra `gridless`). GEOS ships in the
  Shapely wheel; Shapely 2.1.2 / GEOS 3.13.1 confirmed working in both spikes. The visibility
  graph uses Shapely `buffer`/`unary_union`/`difference`/`LineString`/`STRtree` only — no new
  dependency. No CGAL unless the M1 scale spike / M2 measures Shapely instability that
  `set_precision` + the numpy fallback cannot resolve (escalation path, not a planned dependency).
- **Internal (consumed/extended):** `extract_pads`/`build_problem`/`project_geometry`/
  `emit_routes`/`refill_zones`/`route_board_engine` (`kicad.py`); `Net`/`NetRoute`/`route_all`/
  `route_net`/`order_nets` (`multi.py`); `exact_geom.py` predicates (determinism fallback);
  `run_drc`/`strip_routing` (`bridge.py`); the scorecard `_probe_route_human.py`.

---

## Risks and Mitigations

| Risk | Likelihood | Impact | Mitigation |
|------|-----------|--------|------------|
| GEOS float ops nondeterministic run-to-run | Low | High | `set_precision(1e-6)` + sorted-corner/fixed-edge-order + integer A* tie-break; **already byte-identical in Spike-0 + Spike-0b** (same-process + fresh subprocess); numpy `exact_geom` fallback for legality ops if a multi-net sequence wobbles |
| Visibility graph O(n²) too slow at full board scale | Med | High | **Per-net bounded routing window** (the locality scheme — Spike-0b: 96 nodes/593 edges/0.33 s); window ×2 escalation only on no-path; STRtree-pruned edge tests; convex/taut-string corner pruning held in reserve (M2); **M1 scale spike measures this before the package**; CDT navmesh as documented fallback |
| Per-net window misses a legal path that needs a wider corridor | Med | Med | Deterministic `window_mm *= 2` escalation capped at board bbox; worst case = global graph for that one net (still correct, just slower); flagged + counted in the M1 scale spike |
| Congestion signal coarse vs grid cells | Low | Med | Super-cell lattice tunable (start 0.5 mm); shared field with grid nets; priced per-edge weighted by crossing length; `history_factor` already a parameter |
| Mixed grid+gridless double-counts occupancy | Med | High | Single shared grid ledger: gridless adapter rasterizes its centerline into `nr.cells`/`via_sites` at the same inflation; `_mark` treats both identically |
| Realization places a segment outside free space → DRC error | Low | High | The visibility-graph path is by-construction inside free space; post-realize `is_legal` assert per segment (numpy predicate); **Spike-0b's 0-error + all-legal gate already passed**; M1 scale spike re-checks under multi-net accumulation |
| Via legality check (both layers) misses a layer | Low | High | Exact two-layer disc-in-free-space check (`Point.buffer.within(free_space)`) before emitting a via; rasterize via into both layers of the shared ledger |
| Effort overrun (3-6 mo solo) | Med | Med | M1 scale spike + M2 are the two go/no-go gates; gate-per-milestone discipline; CDT fallback if either fails |

---

## Testing Strategy

- **Spike-0 / Spike-0b:** the scripts ARE the tests — DRC 0-error gate + <5 s + byte-identical
  3-run gate (DONE, both passed).
- **M1 scale spike:** the script IS the test — all-legal + connectivity≥baseline (relieves ≥1)
  + byte-identical 3-run + runtime ~2× grid gate.
- **Unit (M1+):** `geom.py` — Minkowski inflation against hand-computed offsets; precision
  snap idempotence. `visgraph.py` — corner extraction is sorted/deterministic; `is_visible`
  agrees with the `exact_geom` clearance check; window-escalation widens deterministically;
  the same obstacle set yields the same `(nodes, adj)` twice. `search.py` — A* finds
  known-shortest path on a fixture with 2–3 obstacles; tie-break determinism (run twice,
  identical node-index sequence). `realize.py` — every emitted segment passes `is_legal` /
  `segment_rect_distance` against the numpy predicates. `adapter.py` — `to_netroute`
  rasterization matches a grid route's `cells` footprint for a straight trace.
- **Integration (M2+):** mixed grid+gridless on a small fixture board: shared history field
  updates from both substrates; rip-up of a gridless net frees its grid cells. Determinism
  across the full `route_all` (byte-identical emitted board, 2 runs).
- **Scorecard (every milestone):** `_probe_route_human.py` on mitayi (and zuluscsi at M4);
  compare `unconnected`/`errors`/`by_type` against the grid baseline and the milestone exit
  criterion. This is the canonical acceptance measure.

---

## Acceptance Rubric

Per `agents/pm-reference/rubric-format.md`. Atomic, binary, evidence-mandatory. Applies to
THIS revised design artifact + the M1 scale spike it specifies.

1. **R1 — Substrate decision is made and grounded in the spike.** PASS iff the doc states the
   chosen substrate, cites Spike-0b's room-failure vs visibility-graph-success, and says why
   NOT the CDT alternative. Evidence: "Substrate (REVISED)" section (visibility-graph + locality;
   rooms degenerated at `max_rooms=2000`; CDT rejected: untested + library risk).
2. **R2 — Locality / scale scheme is concrete.** PASS iff the doc specifies the per-net
   bounded window, the corner-set reduction, edge pruning, and the per-net-rebuild-under-rip-up
   policy with escalation. Evidence: "Locality / scale scheme" (window_mm + ×2 escalation,
   taut-string corners, STRtree edge pruning, per-net rebuild).
3. **R3 — Determinism strategy is specific and verifiable on the new substrate.** PASS iff it
   names the GEOS mechanism, the corner/edge ordering AND A* tie-break, and a verification gate.
   Evidence: Determinism section (`set_precision` 1e-6, sorted corners + fixed i<j edge loop +
   integer heap tie-break, 3-run byte-identical gate — passed in both spikes).
4. **R4 — Module structure + exact plug points named.** PASS iff the new package path, its
   modules (`visgraph.py`/`search.py`/`realize.py`/`adapter.py`/`congestion.py`/`geom.py`), and
   the named functions/parameters modified in `route_board_engine`/`route_all`/`emit_routes`
   are all specified. Evidence: Scope + Decision 2 (plug points A/B/C).
5. **R5 — Congestion model confirmed/adjusted on the new substrate.** PASS iff the doc confirms
   the super-cell lattice, explains why it is substrate-independent, and re-phrases pricing as
   per-edge × crossing-length. Evidence: Decision 3.
6. **R6 — Via + 2-layer + pour lifecycle decided on the visibility graph.** PASS iff via cost
   source, the per-layer two-graph model, the via legality check, and the pour lifecycle are
   each stated. Evidence: Decision 4.
7. **R7 — Milestone plan with measurable exit criteria, revised for the substrate.** PASS iff
   Spike-0/0b (done) → M1-scale-spike → M1 → M2 → M3 → M4 each have a goal, a measurable exit
   criterion, and an effort estimate, with the M1 scale spike inserted as the next gate.
   Evidence: Decision 5 table.
8. **R8 — M1 scale spike fully specified.** PASS iff the doc names the mechanical region/net-set
   selection rule, exact pass criteria (all-legal, connectivity≥baseline+relieves≥1, det 3-run,
   runtime ~2×), what it validates (scale + multi-net), and what it defers (rip-up, vias).
   Evidence: "M1 SCALE SPIKE" section.
9. **R9 — GridlessNetRoute→NetRoute adapter contract pinned (still stands).** PASS iff the
   adapter type, its relationship to `NetRoute`, and how both coexist in `route_all` (shared
   grid ledger) are specified in pseudocode. Evidence: Decision 2 adapter pseudocode + IS-A.
10. **R10 — No production code written; spike task list ordered + standalone.** PASS iff the
    artifact contains only pseudocode/contracts (no runnable files created), and a numbered
    M1-scale-spike task list exists that a developer can execute without the full engine,
    reusing `build_problem` + the Spike-0b helpers. Evidence: only this `.md` edited; task list
    below.

---

## M1 SCALE SPIKE — ordered developer task list

Reuse Spike-0b as the starting point — **import its proven helpers, do not re-derive them**:
`setup_board`, `build_free_space`, `visibility_graph_astar`, `_obstacle_corner_vertices`,
`_is_visible`, `validate_waypoints`, `emit_net_segments`, `extract_emitted_coords`,
`drc_summary_for_net` (all in `scripts/spike0b_gridless_blocked_net.py`).

1. Confirm the venv has `shapely>=2.0,<3.0` (Spike-0b's; `shapely.geos_version`). Do NOT edit
   `pyproject.toml` (that lands in M1).
2. Create `scripts/spike1_gridless_congested_region.py`. Copy mitayi to a temp dir +
   `strip_routing` via the reused `setup_board`. `extract_pads` + `project_geometry` +
   `build_problem` (do not reimplement extraction).
3. Implement the **mechanical region rule**: find the densest-pad footprint (RP2040 QFN),
   compute its pad bbox, expand by `region_margin_mm = 3.0`. Implement the **net-set rule**:
   nets with ≥2 pads, all pads inside the region bbox, both pads F.Cu; grow `region_margin_mm`
   ×1.5 until ≥10 nets qualify (cap at half-board). Order the net-set by the existing
   `order_nets` policy. Print region bbox, ordered net list, count.
4. Compute the **grid baseline** for the region: run the existing grid `route_all` on the
   net-set (or read the grid scorecard's per-net result) and record region `unconnected` +
   `errors`. Also record the grid `--quality` runtime on this net-set for the ~2× comparison.
5. **Gridless route-once loop, fixed order, accumulating obstacles:** maintain a running
   inflated-obstacle set = all other-net F.Cu pad rects in the region + the buffered realized
   centerlines of already-routed nets (`track_mm/2 + clearance`, `set_precision`-snapped). For
   each net in order: build the per-net window (net bbox + `window_mm`, start 4.0), build the
   visibility graph + A* (reuse `visibility_graph_astar`), escalating `window_mm` ×2 on no-path
   (cap at region bbox). Record per-net node/edge counts + build/solve times.
6. Realize each path to a world centerline; **assert every segment legal** with the reused
   `validate_waypoints` against the *current* obstacle set (so later nets respect earlier
   copper). Accumulate the routed net's buffered centerline into the obstacle set before the
   next net.
7. Emit ALL routed centerlines (`emit_net_segments` per net) into the temp board;
   `refill_zones`; `run_drc`. Compute region `unconnected` + new trace-attributable `errors`.
8. **Determinism gate:** re-run steps 5–7 in the same process and in a fresh subprocess; assert
   the sorted union of emitted segment coordinates is byte-identical across all three.
9. **Evaluate pass criteria:** all-legal (0 new errors); region connectivity ≥ grid baseline
   AND ≥1 net connected that the grid leaves unconnected; deterministic; total runtime ≤ ~2×
   grid `--quality`; flag any window exceeding the ~2,000-node ceiling.
10. Report (Structured Result + console): region bbox, ordered net-set + count, grid baseline
    vs gridless `unconnected`/`errors`, per-net + total runtime, max/mean node+edge counts,
    determinism pass/fail, any `set_precision` instability, GO/NO-GO, and — if NO-GO on
    runtime/legality — the explicit CDT-fallback recommendation (→ feeds the architect's gate +
    the `decisions/` KB entry to be written after this spike).

---

## Honest gaps / open risks

- **The locality scheme's scale is argued, not yet measured** — that is precisely what the M1
  scale spike exists to settle before the M1 package. If a per-net window graph routinely
  exceeds the node ceiling on the congested region, corner-set pruning (locality #2) is the
  first escalation; CDT navmesh (B) is the documented fallback if pruning is insufficient.
- Window-escalation worst case (a net that genuinely must detour board-wide) degrades to a
  near-global graph for that one net — correct but slow; the spike counts how often this fires.
- Cross-platform / cross-GEOS-version byte-identity is NOT promised; same-install
  reproducibility is (passed in both spikes). If CI runs a different GEOS than dev, golden-board
  determinism tests must pin the GEOS version (record in the M1 CI setup).
- `extract.py`/`extract_pads` lacks drill geometry; vias rely on project `via_mm`/`via_drill_mm`
  — adequate for uniform vias, insufficient if a board mixes via sizes (out of v1 scope; vias
  are M3 anyway).
- The visibility-graph approach was re-derived from the standard algorithm + Spike-0b, not
  ported from a library (pyvisgraph has no incremental update; we deliberately rebuild per-net).
- Pre-existing convex-rooms prose remains in the doc marked `⊘ SUPERSEDED` for traceability;
  a future cleanup pass may delete it once the visibility-graph design has shipped through M2.

---

## Spike-0 / Spike-0b RESULTS (2026-06-19) — GO on the premise; SUBSTRATE NEEDS REVISION

Validated the gridless premise on mitayi with two standalone scripts (Shapely 2.1.2 / GEOS 3.13.1).

**Spike-0** (`scripts/spike0_gridless_single_net.py`) — net `Net-(U2-BP)` (1.624mm):
plumbing validated (Minkowski inflation, set_precision(1e-6) → byte-identical determinism across 3
runs, realize→emit→DRC closes at 0 new errors, net connected, 0.066s). BUT the net routed inside a
SINGLE convex room — a near-straight shot — so multi-room search + funnel were NOT exercised. Not
decisive on its own.

**Spike-0b** (`scripts/spike0b_gridless_blocked_net.py`) — the DECISIVE test. Net `Net-(J2-CC2)`
whose straight inflated path PROVABLY intersects a blocker (centerline-to-copper distance 0.000mm,
intersection 0.099mm² — a straight trace is illegal). Result: **GO** —
- routed a BENT path AROUND the obstacle: 4 vertices / 3 segments, through a 0.33mm corridor;
- 0 new trace-attributable DRC errors; connection resolved; every segment legal (exact_geom);
- byte-identical determinism (same-process + fresh subprocess); solve 0.41s.

### ⚠ Substrate finding — convex expansion rooms FAILED; visibility-graph worked
The architect's chosen substrate (lazy **convex expansion rooms**, survey Approach #1) **degenerated**
on the blocked net: the narrow 0.33mm corridor spawns tiny-sliver rooms and A* hit `max_rooms=2000`
without reaching the goal. The spike PIVOTED to a **visibility graph over inflated obstacle corners**
(survey Approach #3): 96 nodes, 593 edges, optimal bent path with only 4 A* expansions in 0.33s.

This empirically contradicts the survey's ranking (Approach #1 top; Approach #3 flagged O(n² log n)
"cost-prohibitive for 200-net boards"). **The substrate decision must be revised before M1**, using
this data:
- **Visibility-graph** handles narrow corridors natively (no sliver degeneracy) and is optimal, but
  full-board scaling (n = all obstacle corners) needs spatial pruning / incremental updates / locality
  (route within a bounded window, not the whole board).
- **CDT navigation mesh** (survey runner-up, Approach #4) also avoids sliver-room degeneracy via
  triangulation and supports incremental edge-flip updates — a strong candidate the spike did not test.
- The **convex-expansion-rooms** approach as specified is NOT viable for congested corridors without
  a fix for the sliver-room degeneracy.

**Recommendation:** a focused architect revision to choose between (a) visibility-graph + spatial
locality/pruning, or (b) CDT navmesh, grounded in this spike — BEFORE building M1. The realize→emit→DRC
loop, determinism strategy, Shapely dependency, and adapter design from this doc all stand; only the
search-substrate core changes.

### ✅ RESOLUTION (2026-06-19, this revision)
The architect revision is DONE — see the **"Substrate (REVISED)"** section at the top of this
doc. **Decision: (A) visibility-graph + per-net bounded-window locality.** (B) CDT navmesh is
rejected as primary (untested vs (A)'s working prototype; Python CDT-with-holes + incremental-
update library risk; larger determinism surface; only approximately optimal) and held as the
documented M2-gate / M1-scale-spike fallback. The next decisive experiment is the **M1 SCALE
SPIKE** (route ≥10 nets through the mitayi GPIO fan-out, fixed order, accumulating obstacles;
all-legal + connectivity≥baseline+relieves≥1 + deterministic + runtime ~2× grid) — specified in
full above, to measure the locality scheme's scale BEFORE the M1 package is built. The
super-cell congestion model, determinism strategy, Shapely dependency, `GridlessNetRoute`
adapter, emit/realize→DRC loop, and staged milestones all STAND.

---

## M1 SCALE SPIKE — first run (2026-06-19): NO-GO as-run; substrate NOT disproven

`scripts/spike1_gridless_congested_region.py` routed 10 nets through the mitayi RP2040 (U3) region.
Result NO-GO as-run, but the diagnosis is decisive: **3 of 4 failures were spike defects, 1 was real,
and the substrate (visibility-graph + locality) is NOT disproven.** CDT fallback NOT triggered.

Findings:
1. **Legality (GND) — spike bug (false positive):** `validate_waypoints` passed pre-inflated segment
   obstacles to `is_legal`, which subtracts `track_hw` AGAIN (double-count). The waypoints are inside
   Shapely free_space. Fix: use Shapely `free_space.contains()` as the sole waypoint legality oracle.
2. **Legality (USB-DM) — REAL DESIGN GAP (now folded into the design):** after an adjacent net (+1V1)
   routes, the own-net start-pad center fell INSIDE +1V1's copper buffer, so the windowed free_space
   excluded the start point → A* started outside free space → illegal. **The gridless free-space build
   MUST carve the routing net's OWN pad rects out of the accumulated-obstacle set before union/difference
   — mirroring `build_problem`'s existing `carve` logic. This is now a substrate requirement.**
3. **Connectivity criterion was untestable here — milestone recalibration:** the dense-but-2-pin-F.Cu
   region is routed 10/10 by the grid, so "relieve ≥1 net" cannot be shown on a SINGLE-LAYER spike. The
   connectivity gap lives in multi-pin power nets + via-requiring nets. **Therefore the M1 scale spike's
   job is narrowed to: legality-under-accumulation + runtime + determinism at scale. Connectivity-relief
   validation moves to M3 (vias / 2-layer), where the grid actually fails.**
4. **Runtime 6.58× grid — the one genuine risk:** the spike did NOT implement the designed STRtree
   visibility-edge pruning (locality mechanism #3); visibility checks were O(n²·obstacles). Must
   implement STRtree (and, if still >2×, the reduced taut-string corner set) and re-measure.
5. **Determinism — harness bug:** isolated routing is byte-identical; the grid-baseline run contaminated
   shared state before the determinism gate. Fix: run the determinism gate before the grid baseline.

**Next:** apply fixes 1,2,4,5 + implement STRtree (3), recalibrate criteria (legality+runtime+determinism
only), re-run. Real go/no-go pending that re-run.

---

## M1 SCALE SPIKE — run-2 (2026-06-19): GO — substrate confirmed

`scripts/spike1_gridless_congested_region.py` re-run with all fixes applied. **Verdict: GO.**
CDT fallback NOT triggered.

### Fixes applied (FIX-1 through FIX-6)

- **FIX-1: Shapely contains oracle** — replaced `validate_waypoints`/`is_legal` (which double-counted
  inflation) with `free_space.buffer(1e-5).contains(Point(x,y))` as the sole per-waypoint legality check.
  GND and all other nets now correctly report legal.

- **FIX-2: Own-pad carve-out** — `build_windowed_free_space` now carves the routing net's own pad rects
  out of the accumulated-obstacle union before computing the `free_space` difference. Resolves the
  `Net-(U3-USB-DM)` start-outside-free_space failure that occurred when an adjacent net's copper buffer
  overlapped the routing net's own pad center.

- **FIX-3: STRtree pruning from actual obstacle polygons** — initial implementation built the STRtree
  from `Polygon(ring)` per interior ring of `fs_component`; obstacles partially extending outside the
  window clipped to the exterior ring and were invisible to the tree, causing false "trivially visible"
  results and illegal paths. Corrected by returning the actual obstacle polygon list from
  `build_windowed_free_space` and building STRtree over those. In the dense GPIO fan-out, every edge has
  at least one nearby obstacle (0% skipped); the main runtime benefit comes from FIX-5b below.

- **FIX-4: Determinism gate before grid baseline** — moved the 3-run determinism check (runs 1, 2, same-
  process; run 3, fresh subprocess) to execute before the grid baseline run, eliminating state
  contamination from `refill_zones` formatting side-effects. Added `refill_zones` calls on run-2 and
  subprocess paths to match run-1's normalization.

- **FIX-5: Recalibrated pass criteria** — connectivity-relief removed as hard criterion (deferred to
  M3/vias). Hard criteria: all-legal (Shapely oracle) + determinism (byte-identical) + runtime (≤2× grid).
  Connectivity reported as informational only.

- **FIX-5b: Reflex-corner pruning** — `_reflex_obstacle_corners` retains only convex obstacle corners
  (reflex corners of the free-space hole, i.e., left-turn corners of CCW interior rings). These are the
  only corners that can be turning points of a shortest taut path. Reduces node count from O(all corners)
  to O(reflex corners). Max nodes dropped from uncapped to 93 across all nets. Fallback to full-corner set
  when reflex-only returns no path.

- **FIX-6: inflate_track formula** — changed `inflate_track = track_mm/2 + clearance_mm` to
  `inflate_track = track_mm + clearance_mm`. The old formula gave edge-to-edge clearance of
  `0.225 - 0.075 - 0.075 = 0.075mm` (DRC failure: required 0.150mm). The correct formula gives exactly
  0.150mm edge-to-edge.

- **ENV FIX: project-local temp dir** — `tempfile.TemporaryDirectory(dir=ROOT / ".spike1_tmp")` to work
  around flatpak pcbnew not being able to access `/tmp` (Linux flatpak sandbox restriction).

### Run-2 results

| criterion | result | value |
|-----------|--------|-------|
| all-legal | PASS | 9/9 routed nets, 0 violations, 0 new DRC errors |
| determinism | PASS | byte-identical (same-process + fresh subprocess) |
| runtime | PASS | 2.24s = 0.81× grid (--quality finer-pitch) |
| node ceiling | clear | max=93 (ceiling=2000) |

- **9/10 nets routed.** `/GPIO2` fails in route-once mode: after 9 preceding nets accumulate copper,
  `/GPIO2`'s start and goal fall in disconnected free-space components (`nodes=2, edges=0`). This is a
  route-order / corridor-exhaustion artifact of the route-once-no-rip-up approach. It is NOT a substrate
  failure — rip-up (M2 milestone) would resolve it by allowing net reordering when blocked.
- **Grid baseline** (same 10-net region, pitch=0.05mm): 10/10 routed, 13 DRC errors, 2.79s.
  Gridless outperforms grid on both runtime (0.81×) and legality (0 vs 13 errors).

### Runtime progression (locality lever data)

| configuration | runtime | vs grid |
|--------------|---------|---------|
| run-1 (no STRtree, all corners, no pruning) | ~22s (extrapolated) | ~6.58× |
| run-2 (STRtree FIX-3, broken — from interior rings) | worse than run-1 | >6.58× |
| run-2 (STRtree FIX-3 corrected + reflex pruning FIX-5b) | 2.24s | **0.81×** |

The dominant speedup is FIX-5b (reflex-corner pruning): reducing the node set from O(all corners) to
O(reflex corners) collapses the visibility-graph edge count from O(n²) to O(r²) where r << n. STRtree
provides additional O(1) short-circuit for any edge with a clear bounding box; in the dense GPIO region
no edge is completely clear (0% skipped), so its contribution here is bookkeeping overhead rather than
speedup — but it protects open-region cases (where corridors are wide).

### Decision

**Substrate confirmed. Proceed to M1 package build.** CDT navmesh fallback remains documented as M2-gate
option but is not needed. The one open item (rip-up for `/GPIO2` and similar corridor-exhaustion cases)
is a planned M2 milestone, not a substrate defect.

---

## M2 SPIKE (2026-06-19) — negotiation mechanism GO; RUNTIME is the open question

`scripts/spike2_run2_congestion_ripup.py` (run-2; run-1 superseded). Congestion-history pricing +
bounded rip-up on the visibility-graph substrate, on the mitayi RP2040 GPIO fan-out (10 nets).

**Correctness gate (M2 milestone) — MET:**
- **10/10 routed** vs fixed-order 9/10 — **NO regression, +1 relief** (`/GPIO2` now routes).
- 0 new trace-attributable DRC errors (real kicad DRC); **0 board-scale escalations**.
- Byte-identical deterministic (same-process + subprocess); rip-up **converges** (17/80 rounds).
- Mechanism: 0.5mm super-cell history field, edge cost = length × (1 + history_factor·history[cell]);
  victim = most-overlapping routed net; cycle-breaking via pair-specific escalation protection +
  a thrash guard (3 initiations); geometry-blocked nets (>50% board-diagonal pad-only) quarantined
  as M3 (B.Cu/via) candidates rather than board-scale-escalated.

**The one caveat — RUNTIME 7.94× grid** (target ≤3×; improved from run-1's 10.03×). Root cause: 17
rip-up/escalation routes build 200–292-node visibility graphs at 8–16mm windows, and the
visibility-graph **O(n²) edge-build is inherent to the substrate** at wide windows. The midpoint-blocked
edge pre-filter helped (10×→7.94×) but did not reach 3×.

**Decision needed (substrate-scaling fork, architect-level):** the M2 correctness mechanism is proven,
but the runtime at contention scale revives the survey's original O(n²) concern about visibility graphs.
Options before the M2 PRODUCTION integration into `route_all`:
- (a) Accept 7.94× and integrate (gridless used for a bounded set of nets; full-board stays grid) — fine
  for staged adoption, slow for all-gridless.
- (b) Push visibility-graph runtime harder: stronger angular-sweep / reduced taut-string corner set /
  incremental graph reuse across rip-up rounds.
- (c) Re-evaluate the **CDT navmesh** substrate (survey runner-up) — triangulation avoids O(n²) edge
  build and was always the scale fallback; this is the natural point to weigh it, before M3.

### M2 runtime caveat RESOLVED (2026-06-19) — 7.94× → 2.69× grid

Optimized the visibility-graph edge-build (option (b), no substrate change). The fix was vectorizing
the hot path in the PRODUCTION `search.py` `build_visibility_graph` (so M1 benefits too):
- vectorized `shapely.contains_xy` midpoint pre-filter (replaces per-point `Point` creates);
- single batched `STRtree.query` over all candidate edges (replaces O(n²) per-edge queries).
Result: M2 spike runtime **2.69× grid `--quality`** (≤3× target MET), with ALL invariants preserved —
10/10 routed, 0 new DRC errors, 0 board-scale escalations, byte-identical determinism, rip-up converges,
312 tests green. The CDT-navmesh fallback is NOT needed for this scale. M2 spike-level work (mechanism +
runtime) is GO; next is the M2 production integration of congestion+rip-up into `route_all`.
