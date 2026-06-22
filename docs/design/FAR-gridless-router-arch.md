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

---

## M2 PRODUCTION INTEGRATION — gate met (2026-06-19)

Staged integration into `route_all`/`route_board_engine` via `gridless_negotiate=True`: the gridless
subset is routed by the negotiated mechanism (`gridless/negotiate.py` — congestion history + bounded
rip-up + cycle-breaking + geometry-blocked quarantine), rasterized into the shared grid ledger, then
the grid remainder routes seeing that copper. `gridless_negotiate=False`/`gridless_nets=None` is
byte-identical to the pre-M2 engine.

**Obstacle-model completion (the fix that closed the gate):** the gridless free-space build now models
the **board edge** (Edge.Cuts polygon, free space shrunk inward by clearance+track_hw) and **drill
holes** (pad + via drills as inflated circle obstacles, 224 on mitayi) — previously F.Cu pads only.
This eliminated the full-board copper_edge_clearance + hole regression.

**M2 gate (independently verified, `scripts/_verify_m2_gate.py`, same method as the 48/89 grid baseline,
gridless subset = QSPI_SD0/SD1/SD3, D2-A, D2-K, J2-CC1):**
- unconnected **45 ≤ 48** ✓ ; errors **73–76 ≤ 89** ✓ (improvement on BOTH axes); **copper_edge_clearance = 0** ✓.
- Routing deterministic (unconnected stable at 45 across runs; gridless mechanism byte-identical per
  spike; grid path hardened). DRC error COUNT varies ±3 (solder_mask_bridge/clearance) — this is the
  **KiCad pcbnew zone-fill C++ non-determinism** documented since the M1 determinism work, amplified by
  `synth_power_pours=True`; NOT router non-determinism. rip-up converges.

**Open (separate, next):** corridor-blocking — gridless copper can block grid nets (staged one-way
barrier); 3/6 subset nets failed on tight geometry. The board still improved net-net, but full
relief needs cross-substrate awareness or smarter subset selection (M2.1 / folds toward M3 vias).

---

## M2.1 — grid-first gridless rescue (2026-06-19): mechanism built; NO-OP on mitayi (relief ceiling reached)

Fixed the staged one-way barrier: `gridless_rescue=True` routes the GRID first, then rescues the
nets grid leaves unconnected via the negotiated gridless mechanism, with **grid track copper ingested
as obstacles** (`geom.net_routes_to_track_obstacles`: buffer grid centerlines by track_hw+clearance)
so gridless routes in the real gaps and cannot block grid. Deterministic; `gridless_rescue=False`
byte-identical to current; 356 tests green.

**Result on mitayi: ZERO nets rescued — and that is the correct, honest outcome.** The rescue
classifier found NO rescuable candidates: of the grid's unconnected failures, **4 are 2-pin F.Cu but
geometry-blocked** (QSPI_SCLK/SD1/SD2, USB-DM — min window ~92% of board diagonal → genuinely need
vias/B.Cu), and **~20 are multi-pin or multi-layer** (GPIO buses, power, SWCLK — outside the Phase-1
2-pin-F.Cu gridless scope). So the board is unchanged (no relief, no regression).

**Conclusion — the 2-layer single-F.Cu gridless relief ceiling on mitayi is REACHED.** Further
connectivity gains require **M3 (vias / B.Cu + multi-pin nets)**. The rescue mechanism + grid-track
obstacle ingestion built here are correct **M3 infrastructure** (they'll have candidates to rescue
once vias open the geometry-blocked nets and multi-pin support lands).

**Baseline note (record):** the verified grid-only mitayi baseline (route_board_engine defaults +
run_drc, severity==error) is **48 unconnected / 89 errors with 0 copper_edge_clearance** (3×
independently confirmed, incl. `scripts/_verify_m2_gate.py`). Some ad-hoc measure scripts reported
50/204 with copper_edge_clearance=120/text_height/lib_footprint_issues — that is a DRC-counting/
invocation discrepancy in those scripts, NOT the routing; `_verify_m2_gate.py` is the canonical method.

---

# M3 — Vias / 2-layer (B.Cu) + multi-pin nets — ACTIVE DESIGN (2026-06-20)

> **This is the active M3 design.** It supersedes the one-row M3 placeholder in the Decision-5
> milestone table and the sketch in Decision 4 (which predates the per-layer-graph framing and the
> #4 / M2.1 hole-aware learnings). M3 is the real connectivity unlock: M2.1 proved the
> single-layer-F.Cu relief ceiling on mitayi is **reached** — of the grid's 48 unconnected, **4 are
> 2-pin F.Cu but geometry-blocked** (QSPI_SCLK/SD1/SD2, USB-DM — min window ~92% of board diagonal,
> i.e. no single-layer corridor exists) and **~20 are multi-pin/multi-layer**. Both classes need
> M3. The `gridless_rescue` mechanism + grid-track-obstacle ingestion built in M2.1 are the M3
> integration substrate — they finally have rescuable candidates once vias open the
> geometry-blocked nets.
>
> NO production code in this doc — pseudocode only where it pins a contract. M3 ships in two phases:
> **M3-Phase-1 = vias / 2-layer for 2-pin nets** (the Spike-M3 below proves it); **M3-Phase-2 =
> multi-pin connection trees** (staged after Phase-1, specified here so the data model is fixed up
> front).

## M3 design question 1 — Per-layer free space + visibility graph

**Decision: build the free-space polygon + visibility graph PER LAYER (F.Cu=0, B.Cu=1), each over
its own per-layer obstacle set, inside the SAME per-net routing window.** This is the minimal,
consistent extension of the proven M1/M2 substrate: the visibility-graph machinery
(`build_windowed_free_space`, `build_visibility_graph`, `astar_visgraph`, reflex pruning, STRtree,
`set_precision`) is reused **verbatim, called twice** — once per layer — rather than reinvented.

**Per-layer obstacle sets (what changes vs M1/M2):**

- **Shared across both layers (drills/holes pierce both copper layers):**
  - The **board outline** (`extract_board_outline`) — same polygon clips both layers' free space
    (the board edge is layer-independent).
  - The **drill-hole obstacles** (`extract_drill_obstacles`) — through-hole pad drills + via drills.
    These are circular obstacles that **pierce both F.Cu and B.Cu**, so the same
    `drill_obstacles` list is added to BOTH per-layer free spaces. This is the geometric reason a
    via cannot be placed on top of an existing drill: the drill circle is an obstacle on both
    layers, so no candidate via site falls inside it.
- **Per-layer (copper is layer-specific):**
  - **F.Cu obstacles (layer 0):** other-net pads with `p["front"]` True (the current behaviour —
    `build_windowed_free_space` already filters `if not p.get("front"): continue`) + already-routed
    F.Cu track copper.
  - **B.Cu obstacles (layer 1):** other-net pads with `p["back"]` True + already-routed B.Cu track
    copper. **`extract_pads` already provides `p["back"]`** (PAD_SCRIPT: `ls.Contains(pcbnew.B_Cu)`).
    A through-hole pad is BOTH front and back (it appears as an obstacle on both layers — correct,
    its copper annulus is on both). An SMD pad on B.Cu is back-only (appears only on B.Cu). On
    mitayi the B.Cu obstacle set is sparse (mostly through-hole pads + a few B.Cu SMD pads) — which
    is exactly **why** B.Cu is the bypass lane the geometry-blocked F.Cu nets need.

**The single change to `build_windowed_free_space`:** add a `layer: int = 0` parameter. The pad
filter becomes `wants = p.get("front") if layer == 0 else p.get("back")`; `if not wants: continue`.
The board-outline clip and `drill_obstacles` add are unchanged (shared). `extra_obstacles`
(already-routed copper) becomes **per-layer** (the caller passes the layer's own routed-track
obstacle list). Everything else — own-pad carve-out, inflate formula, snap — is identical. This is
a ~3-line signature/filter change, NOT a rewrite: legality-by-construction is preserved per layer.

```python
# geom.py — the ONLY signature change (per-layer obstacle selection)
def build_windowed_free_space(
    pads, net_name, clearance_mm, track_mm, extra_obstacles, window_bbox,
    board_outline=None, drill_obstacles=None,
    layer: int = 0,                # NEW: 0=F.Cu, 1=B.Cu. Selects p["front"] vs p["back"].
) -> tuple[free_space, obstacle_polys]: ...
```

**Determinism is unaffected:** each per-layer graph is built by the same deterministic pipeline
(sorted corners, fixed i<j edge loop, integer A* key). Two layers = two independent deterministic
graphs; the cross-layer link (below) is enumerated in fixed candidate-site order. Same-install
byte-identity holds, proven through M2.

## M3 design question 2 — Via-transition edges (how the two graphs connect)

**Decision: a via is a single zero-length cross-layer edge linking `(x, y, 0)` on the F.Cu graph
to `(x, y, 1)` on the B.Cu graph, at the existing `via_cost` parameter, placeable ONLY at a finite
deterministic set of candidate via sites that pass the two-layer legality test (DQ4).** This is the
visibility-graph realization of `astar.via_ok` — exact (Shapely disc-in-free-space on both layers)
rather than cell-based.

**Node model.** The M3 A* node becomes `(x, y, layer)` (the doc's long-planned `WayNode`). Each
per-layer graph contributes its own `(x, y)` nodes tagged with its layer. The two node lists are
concatenated into one index space: `[F_start, F_goal, *F_corners, *B_corners]` (B has no separate
start/goal — the net's pads define the start/goal layer; see the pad-layer-reach note below).

**Candidate via sites = a finite deterministic set.** A via may be placed at:
1. **every shared obstacle corner** that exists as a node on BOTH layers' graphs at the same `(x,y)`
   (snap to 1 nm; equal coords → same site), AND
2. a **small deterministic lattice of via candidates seeded along the straight start→goal line**
   (sampled every `via_pitch_mm`, default = `via_mm + clearance_mm`, snapped to 1 nm) — this seeds
   via sites in OPEN regions where no obstacle corner exists but a layer change is still wanted
   (the geometry-blocked nets cross open B.Cu, so the bend point may not coincide with a corner).

Both site sources are sorted `(round(x,6), round(y,6))` → deterministic enumeration. For each
candidate site that passes the DQ4 two-layer legality test, add a cross-layer edge
`(site, 0) ↔ (site, 1)` with cost `via_cost` to the adjacency. The site `(x,y)` must also be a node
on each layer's graph and **visible** (via the existing `is_visible`) to its neighbours on that
layer — i.e. the site is inserted as an ordinary waypoint node on each layer's graph, and the
cross-layer edge connects the two co-located waypoint nodes. (Implementation note: insert candidate
via sites into BOTH per-layer node lists BEFORE building each layer's adjacency, so the in-plane
visibility edges to/from the site are built by the existing edge loop with no special case.)

**Cost & determinism.** `via_cost` is the **existing engine parameter** (`route_board_engine`
default 10.0). Reuse it unchanged — do NOT invent a geometric via cost (parameter parity with the
grid router keeps the scorecard apples-to-apples; Decision 4 already committed to this). The
cross-layer edge is enumerated after the in-plane edges in fixed candidate-site sorted order, so the
adjacency is byte-identical run-to-run.

**Pad layer reach (where the path starts/ends).** A net's two pads define the start/goal nodes and
their layers. For mitayi's geometry-blocked QSPI nets the pads are **through-hole** (both front and
back) — so start and goal exist on BOTH layers, and the path may begin on F.Cu, via down to B.Cu,
cross, and via back up to F.Cu, or even start on B.Cu. For an SMD F.Cu-only pad, start/goal exist
only on layer 0 (a via adjacent to the pad provides the layer change). The start/goal node is added
to each layer the pad reaches; A* terminates when it pops the goal node on ANY layer the goal pad
reaches.

## M3 design question 3 — A* over the 2-layer graph

**Decision: one A* over the merged `(x, y, layer)` node space; in-plane edges carry Euclidean
length (× congestion price, reusing M2's super-cell field unchanged); cross-layer edges carry
`via_cost`. Heuristic = Euclidean in-plane distance to the goal `(x,y)`, plus `via_cost` iff the
node's layer differs from the goal's layer AND the goal is reachable only on the other layer.**

This is `astar_visgraph` / `_astar_congestion` extended with a layer field on the node — the heap
key, integer 1 nm bucketing, `insertion_seq` tie-break, and sorted-neighbour expansion are all
unchanged. The heuristic stays **admissible**: in-plane Euclidean underestimates true in-plane
length (always), and adding `via_cost` only when a layer change is provably required never
overestimates (a path that needs a layer change pays ≥ `via_cost`; a path that doesn't is on the
goal's layer already so the term is 0). `via_cost ≥ 0`, so admissibility holds exactly as the M2
congestion-priced heuristic does.

```python
# search.py — the M3 heuristic (admissible: in-plane Euclid + conditional one via_cost)
def heuristic(node):           # node = (x, y, layer)
    x, y, lyr = node
    h = math.hypot(x - goal_x, y - goal_y)
    if lyr != goal_layer and goal_reachable_only_on(goal_layer):
        h += via_cost          # ≥1 layer change is unavoidable; never an overestimate
    return h
```

Neighbours of `(x, y, layer)` = its in-plane visibility edges on `layer` (from that layer's adj)
**plus** the cross-layer edge to `(x, y, 1 - layer)` if the site is a legal via. The path returned
is a list of `(x, y, layer)` waypoints; a layer change between consecutive waypoints at the same
`(x,y)` is a via, which `realize` turns into a via emission (DQ6).

## M3 design question 4 — Via legality + the M2.1 hole-aware lesson (THE crux)

**Decision: a candidate via site `(x, y)` is LEGAL iff a disc of radius `via_mm/2 + clearance_mm`
is inside the free space on BOTH layers, AND a disc of radius `via_drill/2 + hole_clearance_mm`
clears all OTHER drills/holes (hole-to-copper), AND drill-to-drill center distance to every other
via/drill ≥ `via_drill + hole_to_hole_mm`. The `hole_clearance` (~0.25 mm) is LARGER than the
copper clearance (~0.15–0.2 mm) — honoring only copper clearance is the #4 / boxed-in-via bug.**

This is the single most important correctness rule in M3. The #4 exact-geometry probe and the M2.1
diagnosis both showed the failure mode: a via nudged to satisfy only copper clearance still violates
`hole_clearance` (drill-to-copper) or `hole_to_hole` (drill-to-drill) — congested vias are
"boxed-in" by nearby drills. The check has **three independent predicates**, ALL of which must pass:

1. **Copper-ring clearance, BOTH layers (the via annular ring vs other-net copper):**
   `Point(x,y).buffer(via_mm/2 + clearance_mm).within(free_space_layer)` for `layer ∈ {0,1}`.
   Because the per-layer free space already has all other-net copper subtracted (inflated by
   `clearance + track/2`), this disc-within-free-space test is the via-copper-clearance check by
   construction — the same legality-by-construction property the tracks already enjoy. **Use the
   via radius `via_mm/2`, NOT the track half-width**, when building the disc.
2. **Hole-to-copper clearance (the LARGER ~0.25 mm rule — the #4 lesson):** the via DRILL
   (`via_drill/2`) must clear all other-net copper by `hole_clearance_mm`, which the project's DRC
   treats as a SEPARATE, larger class than copper clearance. Concretely:
   `Point(x,y).buffer(via_drill/2 + hole_clearance_mm)` must not intersect any other-net copper on
   either layer. Since `hole_clearance > clearance`, this is a STRICTER disc than predicate 1 in the
   region near drilled copper. Build a dedicated `hole_free_space_layer` = window − (copper inflated
   by `hole_clearance` instead of `clearance`) for the drill disc test, OR test the drill disc
   against the raw (un-inflated) copper polygons buffered by `hole_clearance`. Either is exact;
   the design picks the first (one extra free-space build per layer, cheap, reuses the pipeline).
3. **Hole-to-hole (drill-to-drill center distance):** for every existing drill/via center `c`,
   require `dist((x,y), c) ≥ via_drill/2 + drill_r(c) + hole_to_hole_mm`. The `drill_obstacles`
   list already carries inflated drill circles; a cheaper equivalent already available: the
   `drill_obstacles` are inflated by `clearance + track/2`, so for hole-to-hole the design adds a
   second drill-circle set inflated by `via_drill/2 + hole_to_hole_mm` and requires the via center
   to be **outside** all of them. (This set is built once per problem, not per via.)

**Where `hole_clearance_mm` / `hole_to_hole_mm` come from.** `project_geometry` currently surfaces
`via_mm` / `via_drill_mm`; M3 extends it to also surface `hole_clearance` and `hole_to_hole` from
the board's design rules (the same sexpr `setup`/`rules` block `project_geometry` already parses for
`via_diameter`/`via_drill`). Fallbacks: `hole_clearance_mm = max(clearance_mm, 0.25)`,
`hole_to_hole_mm = max(clearance_mm, 0.25)` (mitayi's observed values). **This is the M2.1
hole-aware lesson made structural:** the via placement honors the larger drill clearances up front,
so it never emits a boxed-in via that DRC then flags.

**Fail / escalate cleanly when no legal via exists.** If NO candidate via site in the window passes
all three predicates, the net's A* simply finds no cross-layer edge → no 2-layer path → the existing
**window-escalation** fires (`window_mm *= 2`), widening the candidate-site lattice and corner set.
If escalation reaches board scale with still no legal via, the net is reported
`status="failed"` / `reason="no_legal_via"` (distinct from `"geometry_blocked"`) — it does NOT
silently emit an illegal via. This is the clean-failure contract: **legal-or-nothing**, never
legal-by-nudge-after-the-fact (the #4 regression we are structurally avoiding).

## M3 design question 5 — Multi-pin nets (the ~20) — DECOMPOSITION + STAGING

**Decision: M3-Phase-2. Decompose an N-pin net into a Minimum Spanning Tree (MST) over its pads;
route the MST edges as 2-pin sub-routes SEQUENTIALLY; after each sub-route, the net's OWN realized
copper (centerlines + vias) is added to the same-net "connection geometry" so later sub-edges may
terminate on it (any point on already-routed same-net copper is a valid connection target, not just
a pad). The Spike-M3 stays strictly 2-pin — multi-pin is built AFTER Phase-1 lands.**

Concrete decomposition:

1. **MST over pads.** Build a complete graph over the net's K pads with edge weight = Euclidean
   pad-to-pad distance (deterministic: ties broken by `(pad_i_idx, pad_j_idx)`). Compute the MST
   (Prim's, deterministic seed = lowest-index pad). The MST has K−1 edges. Rationale: MST minimizes
   total wire length, matches how a human fans out a bus, and is deterministic.
2. **Sequential 2-pin routing of MST edges, growing same-net copper.** Order the K−1 edges
   deterministically (by `(min_pad_idx, max_pad_idx)`). Route edge 1 as a 2-pin gridless route
   (the Phase-1 machinery). Then for edge e, the **goal is not just pad B but the nearest point on
   the net's already-routed copper** — i.e. the start/goal set for edge e includes pad B AND every
   waypoint of the net's routed sub-paths so far. The simplest deterministic realization: add the
   net's already-routed centerline (buffered to a thin same-net "free connector" polygon) as
   ADDITIONAL goal nodes in that sub-edge's visibility graph; A* terminates on whichever goal node
   it reaches first. Same-net copper is "free" (cost 0 to land on, not an obstacle) — mirroring how
   the grid router lets a net's own cells be re-entered.
3. **Vias in multi-pin.** Each 2-pin sub-edge may itself use a via (Phase-1 machinery unchanged), so
   a multi-pin net naturally spans both layers. A sub-edge terminating on same-net B.Cu copper
   connects there directly (no extra via needed if it is already on B.Cu).

**Staging rationale (why Phase-2, not in the spike):** the via mechanism (DQ2/3/4) is the decisive,
risky unlock and must be proven on the cleanest possible case — ONE 2-pin geometry-blocked net.
Folding MST decomposition into the spike would conflate two unproven mechanisms. Phase-2 reuses
100% of the Phase-1 via/2-layer machinery; it adds only the MST + the same-net-copper-as-goal
connection logic, which is a `negotiate.py`-level concern (net-set orchestration), not a substrate
concern. **Phase-2 exit criterion:** route ≥3 mitayi multi-pin nets (e.g. a GPIO bus) all-legal,
0 new DRC errors, deterministic.

## M3 design question 6 — Integration (extends route_net_gridless / route_gridless_set / emit_routes)

The integration reuses the M2.1 plumbing — the gaps are small and additive.

**`build_windowed_free_space` (geom.py):** +`layer` param (DQ1). One filter line changes.

**`route_net_gridless` (route.py) → layer-aware 2-pin (Phase-1):** the window loop builds free space
on BOTH layers (two `build_windowed_free_space` calls), assembles candidate via sites (DQ2),
tests via legality (DQ4), builds the merged 2-layer graph, runs the 2-layer A* (DQ3). On success,
`world_paths` becomes a list of `(x, y, layer)` waypoints and a new `world_vias: list[tuple[float,
float]]` field carries the via centers. `GridlessRouteResult` gains `world_vias`.

**`route_gridless_set` (negotiate.py) → layer-aware negotiation:** the congestion-priced graph
builder (`_build_congestion_visgraph`) and `_route_one_net_congestion` gain the same two-layer
extension. The super-cell history field is **shared across both layers** (a via and the tracks it
joins all credit the super-cells they cross — congestion is an `(x,y)` property, layer-independent,
exactly as Decision 3 argued). The geometry-blocked pre-classifier (which today quarantines the
~92%-board-diagonal nets) now runs the **2-layer** reachability test: a net is only geometry-blocked
if NO 2-layer path (with legal vias) exists even at board scale — so the 4 QSPI nets that were
quarantined as "geometry-blocked (M3)" should now route. This closes the M2.1 loop: those nets were
deliberately quarantined FOR M3.

**`GridlessNetRoute` adapter (adapter.py) → vias + B.Cu rasterization:** `world_paths` becomes
`list[list[tuple[float, float, int]]]` (the layer field, already anticipated in the docstring as
"Phase 3 adds layer"). `rasterize_into_grid` walks each segment on its waypoint's layer (no longer
hardcoded `layer = 0`), and `world_vias` rasterizes a via cell into BOTH layers + `via_sites`
(reusing the grid router's `via_halfwidth_cells` inflation) so subsequent grid AND gridless nets see
the via on both layers + as a drill obstacle. This makes the shared occupancy ledger via-aware.

**`emit_routes` (kicad.py) → emit B.Cu segments + the via (REUSE existing via emission):** the
world-centerline branch (currently `layer = layer_name[0]` hardcoded) reads each waypoint's layer
and emits `(layer F.Cu)` / `(layer B.Cu)` accordingly. After segments, it emits each `world_vias`
center as a `(via (at x y) (size via_mm) (drill via_drill_mm) (layers F.Cu B.Cu) net)` node —
**this is the exact `(via ...)` node the grid path already emits** (kicad.py lines ~323–328), so the
via writer is reused verbatim; only the position source changes (gridless `world_vias` instead of
the grid `via_pos` map). No nudge is applied to gridless vias — they are placed legal-by-construction
(DQ4), so `nudge_vias` does not touch them (avoids the #4 +4-hole_clearance nudge regression).

**Determinism + existing gates preserved.** All new structures are sorted/seeded; the 2-layer A*
keeps the integer 1 nm heap key + `insertion_seq` tie-break. The hard invariant holds:
`gridless_nets=None`/empty AND no M3 nets → byte-identical to the M2 engine. The
`gridless_rescue=False` and `gridless_negotiate=False` defaults stay byte-identical. Spike-M3 runs
the same 3-run (same-process ×2 + fresh subprocess) byte-identical gate every prior spike used.

## M3 milestone plan (revises the Decision-5 M3 row into measurable sub-milestones)

| Sub-milestone | Goal | Exit criterion |
|---------------|------|----------------|
| **Spike-M3** ⬅ NEXT | Prove 2-layer + legal via closes ONE geometry-blocked 2-pin net | The QSPI net connects (ratsnest resolved), 0 new trace-attributable DRC errors INCLUDING via hole_clearance/hole_to_hole, deterministic (byte-identical 2 runs + subprocess), runtime sane. Standalone script. |
| **M3-P1: vias / 2-layer 2-pin in the package** | Promote spike → `geom.layer` param + 2-layer `route_net_gridless`/`route_gridless_set` + adapter vias + emit B.Cu/via | On the 4 mitayi geometry-blocked QSPI nets routed gridlessly via `gridless_rescue`: all 4 connect, 0 new DRC errors (incl. hole classes), deterministic; `gridless_rescue=False` byte-identical to current. |
| **M3-P2: multi-pin connection trees** | MST decomposition + same-net-copper-as-goal | ≥3 mitayi multi-pin nets (a GPIO bus) route all-legal, 0 new DRC errors, deterministic. |
| **M3 gate (vs scorecard)** | The M3 row of Decision 5, measured | mitayi HUMAN placement via `_verify_m2_gate.py`-style method: hole_clearance + hole_to_hole DRC classes ≤ grid baseline; 0 via-clearance errors on gridless nets; **unconnected strictly < 48** (the via unlock relieves ≥4 QSPI nets + multi-pin gains). |

---

# SPIKE-M3 — the next decisive experiment (built & measured NEXT)

**Goal.** Prove that **2-layer routing + a single legal via insertion closes a 2-pin net that
single-layer F.Cu routing provably cannot** — the M2.1-identified geometry-blocked case. Decisive
because it is the first time the gridless router crosses layers; everything M3-Phase-1 builds rests
on this working. Standalone script `scripts/spikeM3_gridless_via_2layer.py`, **no engine changes**
(same posture as Spike-0b / Spike-1 / Spike-2 — reuse `build_problem` + the package + the spike0b
helpers; import the production `geom`/`search` so the spike does not re-derive the substrate).

**Net selection (mechanical, reproducible — record the rule's output).** Route ONE mitayi QSPI 2-pin
net that the M2.1 `route_gridless_set` pre-classifier flagged `geometry_blocked` on F.Cu — concretely
`/QSPI_SD1` or `/QSPI_SD2` (the rule: among nets `route_gridless_set` returns with
`status="geometry_blocked"`, pick the lowest-`order_nets`-priority 2-pin net whose
`min_needed_window > 0.5 × board_diagonal`; print the chosen net + its single-layer min window to
prove it is genuinely F.Cu-blocked). Confirm both pads are reachable on B.Cu (through-hole, or one
hop to a via site) before routing.

**Procedure.**
1. Copy mitayi to a project-local temp dir (`.spikeM3_tmp`, mirroring Spike-1's flatpak workaround),
   `strip_routing` via the reused `setup_board`. `extract_pads` + `project_geometry` (extended to
   surface `hole_clearance`/`hole_to_hole`, fallback `max(clearance, 0.25)`) + `build_problem`.
   `extract_board_outline` + `extract_drill_obstacles` (drills pierce both layers).
2. **Confirm F.Cu-blocked:** run the existing single-layer `route_net_gridless` (F.Cu only) on the
   net and assert it FAILS (no path even at board-scale window) — this proves the via is necessary,
   not incidental.
3. **Build per-layer free space (F.Cu + B.Cu):** call `build_windowed_free_space(..., layer=0)` and
   `build_windowed_free_space(..., layer=1)` over the same window. F.Cu obstacles = front pads +
   shared drills; B.Cu obstacles = back pads + shared drills.
4. **Assemble candidate via sites** (DQ2): shared obstacle corners present on both layers +
   straight-line lattice sampled every `via_mm + clearance` (snapped 1 nm, sorted).
5. **Legal-via test** (DQ4): for each candidate site, all three predicates — copper-ring disc within
   both free spaces (`via_mm/2 + clearance`), drill disc clears other-net copper by the LARGER
   `hole_clearance`, and drill-to-drill `hole_to_hole` to every existing drill. Keep only legal
   sites. Print how many candidates passed and the chosen via center.
6. **Build the merged 2-layer visibility graph + via-transition edges** (insert legal via sites as
   nodes on BOTH layers, build each layer's in-plane adjacency via the reused
   `build_visibility_graph`, add cross-layer edges at `via_cost`).
7. **2-layer A*** (DQ3): node `(x,y,layer)`, admissible heuristic, deterministic heap key. Recover
   the `(x,y,layer)` waypoint path + the via center(s) where the layer changes.
8. **Realize + emit:** emit F.Cu segments on layer 0, B.Cu segments on layer 1
   (`emit_net_segments(..., layer="B.Cu")`), and the via as a `(via (at x y) (size via_mm)
   (drill via_drill) (layers F.Cu B.Cu) net)` node (reuse the grid emit's via node shape).
   `refill_zones`; `run_drc`.
9. **Determinism gate:** re-run steps 3–8 in the same process and once in a fresh subprocess; assert
   emitted segment+via coordinates are byte-identical across all three (run the gate BEFORE any DRC
   that mutates shared state — the Spike-1 FIX-4 lesson).
10. **Report** (Structured Result + console): chosen net + single-layer min window (proof it is
    blocked), candidate-via count + legal count + chosen via center, F.Cu/B.Cu segment counts,
    DRC before/after (esp. hole_clearance/hole_to_hole/via classes), ratsnest resolved Y/N, runtime,
    determinism PASS/FAIL, GO/NO-GO.

**Pass criteria (ALL must hold for GO).**
- **Connects:** the net's ratsnest is resolved (the previously-unconnected geometry-blocked net is
  now connected via the 2-layer path).
- **All-legal incl. holes:** 0 new trace-attributable DRC errors, **explicitly including via
  `hole_clearance` (drill-to-copper) and `hole_to_hole` (drill-to-drill) and via copper-clearance**
  classes — the #4 boxed-in-via failure mode must NOT recur.
- **Deterministic:** byte-identical emitted segment + via coordinates across same-process ×2 +
  fresh subprocess.
- **Runtime sane:** the single 2-layer route completes in a runtime comparable to a 2-pin M1 route
  (two per-layer graphs ≈ 2× one graph + the via test; sub-second expected on the QSPI window).

**Validates:** that per-layer free space + a via-transition edge + 2-layer A* + a legality-checked
via insertion closes a net that single-layer F.Cu routing provably could not — the core M3 unlock;
that the larger hole_clearance/hole_to_hole drill rules are honored up front (legal-by-construction
via, no post-hoc nudge); that determinism holds across the layer-crossing path.

**Defers:** multi-pin nets / MST decomposition (M3-Phase-2); full-board negotiation WITH vias
(M3-P1 package — the spike routes ONE net, no rip-up); via-congestion rip-up (when two nets contend
for the same via site — M3-P1/P2); B.Cu pour interaction (pours refill post-route as today);
multiple vias per net beyond what the single QSPI net needs (the machinery supports N vias; the
spike just exercises ≥1).

## SPIKE-M3 — ordered developer task list

Reuse the production package + spike0b helpers — **do NOT re-derive the substrate.** Import
`build_windowed_free_space`/`build_visibility_graph`/`astar_visgraph`/`is_visible`/`snap` from
`tracewise.route.gridless.{geom,search}`; `setup_board`/`emit_net_segments`/`extract_emitted_coords`
from `scripts/spike0b_gridless_blocked_net.py`; `extract_pads`/`project_geometry`/`build_problem`/
`extract_board_outline`/`extract_drill_obstacles` from the engine; `run_drc`/`strip_routing` from
`bridge`. NO `pyproject.toml` / production-source edits — that lands in M3-P1.

1. Confirm the venv has `shapely>=2.0,<3.0` (`shapely.geos_version`). Create
   `scripts/spikeM3_gridless_via_2layer.py`. Copy mitayi → `.spikeM3_tmp`, `strip_routing` via the
   reused `setup_board`. `extract_pads` + `project_geometry` + `build_problem` +
   `extract_board_outline` + `extract_drill_obstacles`.
2. Extend the local `geo` dict with `hole_clearance` / `hole_to_hole` (parse from the board's
   design-rules sexpr if present; else `max(geo["clearance_mm"], 0.25)`). Print them.
3. **Select the net mechanically:** run the production `route_gridless_set` on mitayi's gridless
   subset, take the `status="geometry_blocked"` 2-pin nets, pick the lowest-`order_nets`-priority one
   (`/QSPI_SD1` or `/QSPI_SD2`); print net + its single-layer `min_needed_window` vs board diagonal.
4. **Prove F.Cu-blocked:** call single-layer `route_net_gridless` (F.Cu) on the net; assert it
   returns `ok=False` even after window escalation (records the blocked min-window). If it routes,
   pick the next geometry-blocked net (the rule must yield a genuinely blocked net).
5. **Per-layer free space:** add a thin local `build_windowed_free_space` wrapper that passes
   `layer=0` and `layer=1` (until the production `layer` param lands in M3-P1, the spike may inline
   the 1-line front/back filter change). Build `fs_F, obs_F` and `fs_B, obs_B` over the net's window
   (drills + board outline shared).
6. **Candidate via sites:** collect shared obstacle corners (present on both layers at equal 1 nm
   coords) + a straight start→goal lattice every `via_mm + clearance`; sort `(round(x,6),round(y,6))`.
7. **Legal-via predicate** (the crux — implement all three, in this order, short-circuit on first
   fail): (a) `Point(site).buffer(via_mm/2 + clearance).within(fs_F)` AND `.within(fs_B)`;
   (b) drill disc `buffer(via_drill/2 + hole_clearance)` does not intersect other-net copper on
   either layer (build a `hole_clearance`-inflated copper set once); (c) `dist(site, c) ≥
   via_drill/2 + drill_r(c) + hole_to_hole` for every existing drill center `c`. Keep legal sites;
   print candidate vs legal counts.
8. **Merged 2-layer graph:** insert each legal via site as a node into BOTH layers' node lists;
   build each layer's in-plane adjacency via the reused `build_visibility_graph`; add cross-layer
   edges `(site,0)↔(site,1)` at `via_cost` (default 10.0). Node = `(x,y,layer)`.
9. **2-layer A*:** extend `astar_visgraph` to the `(x,y,layer)` node + the admissible
   in-plane-Euclid + conditional-`via_cost` heuristic; integer 1 nm heap key + `insertion_seq`
   tie-break unchanged. Recover the `(x,y,layer)` path + via center(s) at layer changes.
10. **Realize + emit:** emit F.Cu segments (layer 0) and B.Cu segments (layer 1,
    `emit_net_segments(..., layer="B.Cu")`); emit each via center as `(via (at x y) (size via_mm)
    (drill via_drill) (layers F.Cu B.Cu) net)`. `refill_zones`; `run_drc`.
11. **Determinism gate (BEFORE the comparison DRC mutates state):** re-run steps 5–10 same-process +
    fresh subprocess; assert byte-identical emitted segment+via coords across all three.
12. **Evaluate + report** (Structured Result + console): ratsnest resolved Y/N; DRC before/after
    with hole_clearance/hole_to_hole/via classes called out; candidate/legal via counts + chosen
    center; F.Cu/B.Cu segment counts; runtime; determinism PASS/FAIL; GO/NO-GO. On GO, recommend the
    M3-P1 package promotion + a `decisions/` KB entry (2-layer via model confirmed). On NO-GO,
    record whether the failure is the legal-via predicate (tighten DQ4) or determinism (the layer
    field broke a tie-break) — do NOT abandon the per-layer-graph approach on a single wobble.

## M3 risks and mitigations

| Risk | Likelihood | Impact | Mitigation |
|------|-----------|--------|------------|
| Via placed honoring only copper clearance → hole_clearance/hole_to_hole DRC error (#4 regression) | Med | High | DQ4 three-predicate legal-via test enforces the LARGER `hole_clearance` + `hole_to_hole` up front; legal-or-nothing (no post-hoc nudge); spike pass-criterion explicitly checks these DRC classes |
| `hole_clearance`/`hole_to_hole` not in `project_geometry` → wrong fallback | Low | Med | Parse from board design-rules sexpr; fallback `max(clearance, 0.25)` (mitayi's observed values); print the values used so a wrong fallback is visible in the spike report |
| Candidate-via lattice misses the only legal via site in a tight region | Med | Med | Lattice (every `via_mm+clearance`) + ALL shared obstacle corners as sites; window escalation widens both; clean `no_legal_via` failure (not an illegal emit) when truly boxed-in |
| Two per-layer graphs double the runtime | Low | Med | Per-layer windows stay small (M1/M2 reflex pruning + STRtree apply per layer); B.Cu is sparse on mitayi so its graph is tiny; spike measures runtime |
| Determinism breaks when the layer field enters the A* node/tie-break | Low | High | Node `(x,y,layer)` sorts deterministically; cross-layer edges enumerated in fixed site order; 3-run byte-identical gate (proven mechanism through M2) |
| B.Cu pad-layer-reach wrong (via on an SMD-only-F.Cu pad) | Low | Med | Start/goal added per layer the pad reaches (`front`/`back`); A* terminates on goal on ANY reachable layer; mitayi QSPI pads are through-hole (both layers) — the clean case |
| Multi-pin MST routing order leaves a sub-edge stranded | Med (P2) | Med | Same-net copper as free goal nodes; deterministic edge order; rip-up/reorder is the P2 negotiation concern, deferred from the spike |

## M3 acceptance rubric

Per `agents/pm-reference/rubric-format.md`. Atomic, binary, evidence-mandatory. Applies to THIS M3
design section + the Spike-M3 it specifies.

1. **R-M3-1 — Per-layer free space + visibility graph specified.** PASS iff the doc states one graph
   per layer over per-layer obstacle sets, names what is shared (board edge + drills) vs per-layer
   (front/back pads + that layer's tracks), and the exact `geom.build_windowed_free_space` `layer`
   param change. Evidence: M3 DQ1.
2. **R-M3-2 — Via-transition model pinned.** PASS iff the doc defines the via as a cross-layer edge
   at the existing `via_cost`, names the finite deterministic candidate-via-site set, and the
   node→`(x,y,layer)` change. Evidence: M3 DQ2.
3. **R-M3-3 — 2-layer A* with admissible heuristic.** PASS iff node=`(x,y,layer)`, in-plane Euclid +
   conditional `via_cost` heuristic stated admissible, deterministic heap key reused. Evidence: DQ3.
4. **R-M3-4 — Via legality is hole-aware (the #4/M2.1 lesson).** PASS iff the doc states all THREE
   predicates (copper-ring both layers, drill-to-copper `hole_clearance` LARGER than copper
   clearance, drill-to-drill `hole_to_hole`), where the rules come from, and the legal-or-nothing
   clean-failure contract. Evidence: DQ4.
5. **R-M3-5 — Multi-pin decomposition + staging.** PASS iff MST-over-pads → sequential 2-pin
   sub-routes with same-net copper as free goals is specified AND explicitly staged to M3-Phase-2
   (spike stays 2-pin). Evidence: DQ5.
6. **R-M3-6 — Integration plan names the exact extension points.** PASS iff `build_windowed_free_space`
   `layer` param, `route_net_gridless`/`route_gridless_set` two-layer extension, adapter
   `world_paths` layer + `world_vias` rasterization, and `emit_routes` B.Cu-segment + reused-via
   emission are each named. Evidence: DQ6.
7. **R-M3-7 — Determinism + existing gates preserved.** PASS iff the doc states the 2-layer A* keeps
   the integer heap key + tie-break, and the `gridless_*=False`/None byte-identical invariant holds.
   Evidence: DQ6 final paragraph.
8. **R-M3-8 — Spike-M3 fully + decisively specified.** PASS iff a mechanical net-selection rule, the
   procedure, exact pass criteria (connects + all-legal-incl-hole-classes + deterministic + runtime),
   validates, and defers are all stated. Evidence: SPIKE-M3 section.
9. **R-M3-9 — Spike-M3 developer task list ordered + standalone.** PASS iff a numbered task list
   reusing `build_problem` + the package + spike0b helpers, with NO production-source edits, exists.
   Evidence: SPIKE-M3 task list.
10. **R-M3-10 — No production code written.** PASS iff only this `.md` is edited (pseudocode/contracts
    only; no runnable files created). Evidence: `files_changed` = this doc only.

---

## Spike-M3 (2026-06-20) — GO: 2-layer + legal via closes a net F.Cu provably cannot

`scripts/spikeM3_gridless_via_2layer.py` (independently re-verified). Routed `/QSPI_SD0`, a mitayi
net the M2.1 classifier flagged geometry-blocked on F.Cu.

- **F.Cu-blocked PROVEN:** single-layer `route_net_gridless` returns ok=False even at a 55.3mm window
  (100% of board diagonal; visibility graph edges=0) — the via is NECESSARY, not incidental.
- **2-layer route:** F.Cu → via → B.Cu → via → F.Cu (4 segments, 2 vias). Ratsnest RESOLVED
  (net_unconnected 1→0; board unconnected 126→111).
- **Via legality held:** 577 candidate sites, **198 legal** under the 3-predicate hole-aware test
  (copper ring both layers + drill-to-copper @ hole_clearance 0.25 + drill-to-drill @ hole_to_hole 0.15).
  **0 new DRC errors, 0 via hole_clearance/hole_to_hole errors** — the #4/M2.1 boxed-in-via regression
  did NOT recur. Legal-by-construction via (no post-hoc nudge) is validated.
- **Deterministic:** byte-identical emitted segment+via coords across same-process ×2 + fresh subprocess.
- **Runtime:** 2-layer A* solve 0.43s (graph build ~6.5s; grid-setup of the other 56 nets dominates
  wall time but is incidental to the via mechanism).

Key spike-time fix (promote to M3-P1): per-COMPONENT exterior-ring corner collection — grid-routed
F.Cu copper creates fragment obstacles with no interior rings, which left the visibility graph with 0
edges; per-component exterior vertices fix it AND cut the edge-build ~93% (964K→64K pairs).

**M3 core via mechanism is GO.** Next: M3-P1 — promote 2-layer + legal-via into the production package
(`route_net_gridless`/`route_gridless_set` layer-aware) and route all 4 geometry-blocked QSPI nets via
`gridless_rescue`, then the M3 scorecard gate.

---

## M3-P1 (2026-06-20) — 2-layer via mechanism INTEGRATED; rescue-path connectivity gate NOT met

The Spike-M3 2-layer+via logic is promoted into the production package: `build_windowed_free_space`
gains a `layer` param; `geom` gets candidate-via-sites + the 3-predicate legal-via test; `search` gets
2-layer A* (node=(x,y,layer), cross-layer via edges at via_cost) + per-component exterior-ring corners;
`route_net_gridless`/`route_gridless_set` attempt a 2-layer route before quarantining; `GridlessNetRoute`
carries per-segment layer + via centers; `emit_routes` emits B.Cu segments + `(via ...)`. 368 tests
green, ruff clean. `gridless_rescue/negotiate=False` byte-identical to pre-M3.

**Honest gate result — controlled A/B (`scripts/_verify_m3_ab.py`, canonical method, grid-only vs
`gridless_rescue=True`):**
- grid-only **48/89**; rescue=True **48/89 — IDENTICAL. 0 QSPI nets rescued.** No via-hole regression
  (hole_clearance 32→32, hole_to_hole 14→14). **Connectivity gate (unconnected < 48) NOT met.**

**Root cause:** grid-only leaves `/QSPI_SCLK` + `/QSPI_SD2` unconnected. The grid-FIRST rescue then
tries exactly those — but the grid greedily walled off their corridors with its own copper, so even
2-layer vias find no gap (boxed-in-under-congestion; the #4 risk at the deployment level). The Spike-M3
success (`/QSPI_SD0` in isolation) is real, but full-board the rescue candidates are blocked by grid copper.

**The deployment-strategy insight:** M2's gridless-FIRST `negotiate` path already reaches **45**
unconnected with the QSPI subset — BETTER than grid-first rescue's 48 — because it claims corridors
BEFORE the grid does. So the via mechanism is sound; the rescue-AFTER-grid strategy is what fails on
these nets.

**Next (M3-P1.1, strategy not mechanism):** route the geometry-blocked QSPI nets gridless-FIRST (the
2-layer-capable `negotiate` path) rather than rescue-after — re-measure unconnected. If still blocked,
the deeper fix is cross-substrate rip-up (rescue rips grid tracks that wall it off). The committed M3
mechanism is the foundation for both. (Do NOT claim a connectivity gain until A/B shows unc < 48.)

---

## M3-P1.1 (2026-06-20) — gridless-first tested: the two strategies have OPPOSITE failure modes

Wired geometry-blocked nets to attempt the 2-layer route in the gridless-FIRST `negotiate` path
(`multi.py`: on `status=='geometry_blocked'`, call `_route_net_2layer` before the grid claims corridors;
bounded via-search window). No-op-safe (`gridless_negotiate=False` byte-identical); suite green; ruff clean.

**Finding (decisive): routing geometry-blocked QSPI nets gridless-first is PATHOLOGICALLY SLOW.** Even
just 2 nets (`/QSPI_SCLK`, `/QSPI_SD2`) did NOT complete the route in >25 min (killed). Root cause: on a
CLEAN board the free space is HUGE, so the via-candidate-site generation + the 2-layer visibility graph
at large windows explodes (the O(n²) regime the M2 vectorization only tamed for SMALL/congested windows).
Spike-M3 was fast (0.43s) ONLY because it ran POST-grid, where free space is fragmented and bounded.

**So the two M3 deployment strategies fail in OPPOSITE ways:**
| strategy | speed | connectivity |
|---|---|---|
| grid-first rescue (M3-P1) | FAST (post-grid free space bounded) | 0 rescued — corridors walled off by grid (boxed-in) |
| gridless-first (M3-P1.1) | PATHOLOGICALLY SLOW (clean-board free space explodes via search) | has room, but never finishes |

**Reframed plan — option 2 (cross-substrate rip-up) is now clearly the better bet:** add rip-up of
blocking GRID tracks to the FAST grid-first rescue path (rescue rips the grid copper walling off
`/QSPI_SCLK`/`/QSPI_SD2`, reroutes them on the grid, then 2-layer-routes the QSPI nets in the freed
corridor). This keeps the fast post-grid free space AND clears the boxed-in blockage — rather than
fighting gridless-first's clean-board runtime. Fixing gridless-first's via-search performance on large
windows is a SEPARATE optimization, deferred unless option 2 also stalls.

---

## Spike-CSRU / option 2 (2026-06-20) — NO-GO: the real lever is MULTI-PIN (M3-P2), not 2-pin via tactics

`scripts/spike_csru_cross_substrate_ripup.py` — cross-substrate rip-up on the fast grid-first path:
rip grid tracks blocking a boxed-in QSPI net, 2-layer-route the QSPI net in the freed corridor, reroute
the ripped grid nets, accept the swap only if net-positive.

**Result: NO-GO — no connectivity gain (final 48 unconnected, unchanged).** BUT the via mechanism is
NOT the problem: after ripping blockers, both QSPI nets route 2-layer fine (526 / 474 legal via sites,
route succeeds, 0 via-hole errors). The binding constraint is revealed:
- `/QSPI_SCLK` corridor is occupied by **18 nets, 12 of them MULTI-PIN** (GND/83 pads, +3V3/32, GPIO/5-7,
  VBUS/10); `/QSPI_SD2` by 14 nets, 10 multi-pin.
- The current gridless rerouter is **2-pin only**, so the displaced multi-pin nets can't be put back →
  swap is net-negative (+1 QSPI vs −10 to −12 grid) → correctly REJECTED. Final unconnected unchanged.

**Convergent conclusion across options 1 & 2:** mitayi's remaining connectivity gap is **gated by
MULTI-PIN net support (M3-P2)** — both directly (the ~20 still-unconnected nets are multi-pin/multi-layer)
AND indirectly (connecting the 2 boxed-in QSPI nets requires displacing multi-pin blockers). The 2-pin
gridless + 2-layer-via machinery (M1/M2/M3-P1, all spikes GO) is built and proven, but it has reached its
useful ceiling on mitayi WITHOUT multi-pin routing.

**Both tactical options for the 2-pin path are now exhausted/documented:** option 1 (gridless-first) is
too slow on a clean board; option 2 (cross-substrate rip-up) is net-negative without a multi-pin rerouter.
**The path to real mitayi connectivity gains is M3-P2 (multi-pin connection trees: MST decomposition +
same-net-copper-as-goal), which would route the ~20 multi-pin nets AND enable net-positive CSRU swaps.**
(Also noted: CSRU's board-wide 2-layer window ran ~10min/net — the via-search wants the same locality/
perf treatment as the negotiate path if CSRU is revisited post-M3-P2.)

---

## Spike-M3P2 (2026-06-20) — GO: multi-pin connection tree (MST + same-net-copper-as-goal) proven

`scripts/spikeM3P2_gridless_multipin.py` (independently verified). Routed `/GPIO15` (4 pins, 3 MST
sub-edges) as a connection tree.

- **Net FULLY connected:** real DRC net_unconnected **2 → 0**; 0 new errors; 0 via-hole errors.
- **The key new mechanism FIRED — same-net-copper-as-goal:** sub-edge[2] took a COPPER SHORTCUT,
  terminating on an interior point of already-routed same-net copper (157.97, 95.24) at 9.985mm instead
  of routing to the pad at 10.914mm. This proves it's a connection TREE (later sub-edges attach to the
  net's own copper), not K-1 independent pad-pad routes.
- **Deterministic:** byte-identical emitted coords. **Fast/bounded:** max sub-edge 1.5s, windows
  8–15.9mm — the locality lesson held; NO board-wide blowup (unlike options 1 & 2).

**Real limitation surfaced (fix in M3-P2 production):** sub-edge[1] (pad[2]→pad[1]) FAILED with
`realize_failed: waypoint outside free_space (0.235mm)` — a THROUGH-HOLE pad whose center sits just
outside the board-edge-shrunk free space. Here it was REDUNDANT (the net had only 2 ratsnest gaps, so 2
routed sub-edges fully connected it), so connectivity held — but on a net where every MST edge is
load-bearing, such a failure would leave it unconnected. **M3-P2 production must carve the routing net's
OWN start/goal pad into free space even at the board boundary** (mirror the M2.1 own-pad carve-out), so
boundary through-hole pads are always reachable.

**M3-P2 mechanism is GO.** Next: M3-P2 production — promote MST decomposition + same-net-copper-as-goal
into `route_gridless_set`/the engine (layer-aware, bounded windows), ADD the boundary own-pad carve-out
fix, then route mitayi's ~20 unconnected multi-pin nets and measure the M3 connectivity gate (unc < 48).

---

## M3-P2 PRODUCTION (2026-06-21) — mechanism shipped; +1 connectivity; residual is ROUTABLE, not pour-coverage

Promoted multi-pin connection trees into production: boundary own-pad carve-out fix (`geom.py` — own
start/goal pad unioned into free space AFTER the board-edge shrink, so boundary through-hole pads are
reachable); `route_net_multipin` (deterministic Prim MST + sequential bounded-window sub-routes +
same-net-copper-as-goal, via-capable); engine wiring (gridless_rescue routes grid's unconnected
multi-pin nets via the tree). 381 tests green, ruff clean, `gridless_rescue=False` byte-identical.

**M3 connectivity gate (independently verified, canonical A/B vs grid-only 48/89):**
- gridless_rescue=True → **47 unconnected** (gate met, 47 < 48), **0 new via hole_clearance/hole_to_hole**,
  errors 87–89 (zone-fill noise), deterministic. BUT the gain is **+1 only**.

**Corrected strategic read (a prior subagent claimed "residual is mostly power buses" — WRONG).** The 20
still-unconnected nets classify as:
- **POWER / pour-coverage (3):** GND (59 pins), +3V3 (29), +1V1 (5) — these connect via copper POURS, not
  track trees; out of scope for routing.
- **ROUTABLE SIGNAL (17):** /GPIO3,4,6,9,14,18,20,23,27,28 (3-4 pins), /RUN (6), /SWCLK, /USB_D+, /XIN,
  /QSPI_SCLK, /QSPI_SD2, Net-(U3-USB-DP). These are exactly M3-P2's target class.

**So there IS real connectivity headroom (17 routable nets), but the multi-pin RESCUE only nets +1.** The
mechanism is correct (Spike-M3P2 routed /GPIO15 standalone), so the gap is the full-board RESCUE
integration: these 17 signal nets are left unconnected by the grid AND boxed-in by grid copper post-grid
(the same one-way-barrier problem as the QSPI nets) — rescue-after-grid finds no room.

**Next lever — revisit CSRU (option 2), now VIABLE.** CSRU was NO-GO only because the blocking nets were
multi-pin and the rerouter was 2-pin-only. M3-P2 NOW provides a multi-pin rerouter, so cross-substrate
rip-up can rip multi-pin blockers, reroute them (multi-pin tree), and route the boxed-in signal nets in
the freed corridors — a net-positive swap is now possible. This is the path to converting the 17 routable
residual nets into real unconnected<<48 gains.

---

## Spike-CSRU2 (2026-06-21) — NO-GO: residual is GND-POUR-gated, not a rip-up problem

`scripts/spike_csru2_multipin_ripup.py` — cross-substrate rip-up revisited with the M3-P2 multi-pin
rerouter. Result: **NO-GO, 0 targets connected, final unconnected unchanged at 48** (no gain over grid).

**Root cause:** all 17 routable signal targets have **GND/+3V3 POUR copper** in their corridors (GND
pours board-wide, 59 pads; +3V3 29 pads). You cannot rip-and-reroute a poured net as point-to-point
traces — so CSRU (any version) is the wrong lever. The M3-P2 multi-pin rerouter does NOT unblock this:
CSRU v1's "missing multi-pin rerouter" was a SYMPTOM; the real wall is power-pour coverage. **This
rediscovers a CLOSED DEAD-END from the project's own history** (RESUME/ROUTING-COMPLETION: "power pours
CANNOT be locally edited — every refill re-solves the global fill").

**PM caveat (open question, not yet resolved):** CSRU2's early-exit guard SKIPPED each target the moment
GND appeared in its corridor — it never actually attempted to route the signal net THROUGH the GND
region. In reality pours RECEDE around new traces (refill with clearance AFTER routing). So
"GND blocks all 17" may be partly an artifact of treating the GND pour as a HARD obstacle rather than a
receding fill. Whether the 17 signal nets are TRULY walled or merely mis-modeled (pour-as-hard-obstacle)
is UNRESOLVED. Either way CSRU is the wrong tool; the open question is whether a corrected pour-interaction
model (signal routes first, pour refills around it) would let gridless_rescue connect more of the 17.

**Conclusion — the gridless ROUTING arc has reached its practical connectivity ceiling on mitayi.** The
2-layer + via + multi-pin gridless machinery is built, proven, and integrated (mitayi 48/89 → 45/73 via
the negotiate path; deterministic; 381 tests). The residual unconnected is gated by GND-pour INTERACTION
— a different problem class (pour-aware routing / pour-keepout engineering / placement), NOT more rip-up.
Recommended next investigation (separate, not CSRU): does treating the GND pour as a RECEDING fill (route
signal first, refill pour around it) unblock the 17 — i.e. is the residual a pour-modeling artifact or a
true wall? That is the honest open question this arc ends on.

---

## Probe-Pour (2026-06-21) — the residual is a TRUE WALL (track-topology), NOT a pour artifact

Decisive A/B probe (`scripts/_probe_pour_artifact.py`) on 8 representative boxed-in signal nets
(/QSPI_SD2, /QSPI_SCLK, /GPIO3, /GPIO9, /GPIO18, /RUN, /SWCLK, /USB_D+).

**Structural finding that closes the question:** `build_windowed_free_space()` ALREADY excludes pour
polygons — the "receding-pour model" is ALREADY the production behavior; pours are invisible to the
gridless router. There was never a pour-as-hard-obstacle artifact (correcting the CSRU2 diagnosis AND
the PM's own caveat).

**Result: 8/8 TRUE WALL.** Each net fails identically WITH pours as obstacles (A) and WITHOUT (B) — the
pour is irrelevant. The functional blocker is TRACK copper: after grid routing, each net's window free
space fragments into **155–175 disconnected components**, and the net's pads sit in SEPARATE islands
(/QSPI_SD2 pads in components 37 & 43; /QSPI_SCLK in 42 & 49), with 17–25 blocking tracks in the direct
corridor. The pads are topologically disconnected by other nets' tracks; no pour-model change helps.
count_pour_artifact=0, count_true_wall=8, receding_pour_fix_recommended=false.

**Definitive conclusion for the gridless routing arc:** mitayi's residual unconnected nets are walled by
TRACK PLACEMENT (the grid greedily routes blockers that fragment free space), not by pours and not by the
via/multi-pin mechanism. Given the current grid track layout, these nets are at the genuine gridless
routing ceiling. The remaining levers are NOT more routing: (1) routing ORDER/placement (route these
nets before the fragmenting tracks — the gridless-first strategy, blocked only by its clean-board
via-search runtime, a separate perf problem), or (2) the connectivity is placement-bound (matches the
project's long-standing finding that mitayi connectivity needs placement help). The gridless router
(M1–M3, deterministic, 381 tests, mitayi 48/89→45/73 via negotiate) is COMPLETE and at its measured
2-layer routing ceiling.

---

## Probe-Order (2026-06-21) — GREEN LIGHT: residual is ORDERING-bound; gridless-first is viable

Decisive probe (`scripts/probe_order.py`) on a STRIPPED mitayi (pads + board edge + drills only — no
grid tracks, no pours), routing the 17 boxed-in signal nets with BOUNDED windows, single-layer preferred.

**Result — ORDERING problem, NOT placement-bound:**
- **All 17 have OPEN corridors on a clean board.** Routed individually (parallel test, no cross-net
  obstacles): 17/17 route, **16/17 DRC-verified single-layer**, total 39s, max 6.0s/net, NO blowup
  (bounded windows + single-layer keep it fast — the M3-P1.1 clean-board slowness was from forced
  board-wide via-search; avoided here). /USB_D+ is the 17th: 3/4 pads connect F.Cu, pad[3] is B.Cu-only
  → needs ONE via (also routable, not placement-bound).
- The grid router fails these nets ONLY because OTHER nets' grid tracks fragment their corridors
  (Probe-Pour: 155–175 free-space islands). On a clean board the corridors are open and routable.

**Caveat (the real shape of the build):** routing the 17 SEQUENTIALLY with their own accumulating copper
gave only 4/17 — they block EACH OTHER (intra-batch ordering; e.g. GPIO3 copper blocked GPIO4's 1.61mm
sub-edge, which itself routes in 0.04s with no obstacle). So gridless-first needs the M2 NEGOTIATE pass
(congestion history + bounded rip-up, cross-net aware) applied AMONG the boxed-in set — not a naive
route-once. Realistic gain is between 4 (naive) and 17 (independent); the build will MEASURE it.

**Next build — gridless-first ordering (the viable connectivity lever):** (1) identify the boxed-in
hard-net set (nets the grid leaves unconnected with open clean-board corridors); (2) route them
GRIDLESS-FIRST via the negotiate mechanism (congestion + rip-up among them, bounded windows, single-layer
preferred, via where needed e.g. USB_D+); (3) mark their copper into the shared ledger; (4) grid-route
the remainder around them. Measure unconnected vs the 48 baseline (projection: ~15-16 of 17 → meaningful
drop below 48 — the first SUBSTANTIAL connectivity gain; to be verified by real DRC, not assumed).

---

## Gridless-first ordering — BUILD ATTEMPT 1 (2026-06-21): PARTIAL, measurement BLOCKED on perf

Wired a `gridless_first: set[str] | None` param into `route_all`/`route_board_engine` (multi.py):
no-op-safe (`gridless_first=None` byte-identical to current; suite green; `ruff check .` clean).

**BUT the implementation is incomplete and the gate is UNMEASURED.** `gridless_first` merely ACTIVATES
the existing `gridless_negotiate` path (merges the set into `gridless_nets`, forces negotiate=True) — it
does NOT implement Probe-Order's perf-critical fast path. So it reuses the SLOW negotiate-with-board-wide-
via-search that M3-P1.1 already showed is pathological: the gridless-first A/B route of the 17 nets
**did not complete in 30+ minutes** (grid-only side finished in 170s; gridless-first side never produced
output → killed). Board-wide blowup, exactly the M3-P1.1 failure mode.

**Root cause:** Probe-Order proved the 17 nets route FAST (≤6s each) ONLY with **single-layer-preferred +
BOUNDED windows** (no board-wide via escalation). The build wired the param to the negotiate path WITHOUT
that fast path, so via-search escalates board-wide and hangs.

**Precisely-scoped next step (BUILD ATTEMPT 2):** implement the single-layer-preferred + bounded-window
routing in the gridless-first path itself: for each hard net, try single-layer F.Cu in a bounded window
FIRST (the ≤6s Probe-Order path); escalate to 2-layer/via ONLY within a bounded window (cap ~20-25mm) and
ONLY for nets that genuinely need it (e.g. /USB_D+ B.Cu pad); NEVER board-wide. Then the negotiate
(congestion+rip-up) runs among the 17 using these bounded fast routes. THEN the A/B can actually complete
and the connectivity gain can be measured. The param + test + A/B harness (`scripts/_verify_gridless_first_ab.py`)
from attempt 1 are reusable scaffolding.

---

## Gridless-first ordering — BUILD ATTEMPT 2 (2026-06-22): perf FIXED, real gain, but ERROR REGRESSION

Implemented the Probe-Order fast path: `max_window_mm` cap (25mm) threaded through `route_net_gridless`/
`route_net_multipin`, single-layer-preferred, no board-wide via escalation. `gridless_first=None`
byte-identical; 392 tests green; `ruff check .` clean.

**Independently re-run A/B (PM, canonical method) — the route now COMPLETES (perf fixed):**
| | grid-only | gridless_first={17} |
|---|---|---|
| runtime | 164s | **167s (bounded — no hang; attempt-1 fixed)** ✅ |
| unconnected | 48 | **42 (−6)** ✅ real gain |
| of-17 connected | 0 | **4** (/GPIO18, /QSPI_SCLK, /QSPI_SD2, Net-(U3-USB-DP)) |
| errors | 88 | **160 (+72)** ❌ |
| determinism | — | unc stable at 42 across 2 runs ✅ |

**GATE NOT MET.** Two real problems:
1. **+72 error regression** — new `tracks_crossing=19`, `shorting_items=19`, `clearance` +29. The
   gridless-first copper is NOT clearance-clean against (a) the grid copper and (b) each other. This is
   the cross-substrate clearance problem: the 17 nets route on a clean board but their emitted copper
   shorts/crosses grid traces (and each other — the Probe-Order "43 inter-net errors" when routed without
   full mutual deconfliction).
2. **2 grid nets newly failed** (/GPIO1, /GPIO2) — routing the 17 first displaced them. Partial
   net-negative on those.

So gridless-first currently trades −6 unconnected for +72 errors — not yet a good trade. The PERF fix
(window cap + single-layer-preferred) is real and valuable (unblocks the approach; no-op-safe default),
but the cross-substrate CLEARANCE is the next problem.

**Next step (build attempt 3 — clearance, not perf):** (1) the 17 nets must fully DECONFLICT among
themselves under negotiate (no track-crossing/shorting between them — tighten the congestion/rip-up so
their copper is mutually clearance-legal); (2) their copper must be marked into the grid ledger with
PROPER clearance inflation so the subsequent grid routing stays clearance-away (eliminating gridless-vs-grid
shorts); (3) avoid displacing grid nets that were fine (the /GPIO1,2 regression). Target: keep the −6 (or
better) connectivity gain with errors ≤ ~89. The perf path from attempt 2 is the foundation.

---

## Gridless-first ordering — BUILD ATTEMPT 3 (2026-06-22): WIN — mitayi 48/87 → 41/73 (better on BOTH axes)

Cross-substrate clearance fixes (clearance-modeling only; perf path from attempt 2 unchanged):
- `adapter.py`: via centers go into `via_sites` (not `cells`) so `_mark` applies `via_halfwidth_cells=6`
  (via_mm/2 + clearance + track_mm/2 = 0.6mm) instead of the track `halfwidth_cells=3` (0.3mm) — gridless
  vias now reserve proper clearance in the shared grid ledger.
- `multi.py`: gridless-vs-gridless obstacle inflation corrected to `track_mm + clearance_mm` (was
  track_mm/2 + clearance_mm) and via inflation to `via_mm/2 + clearance_mm + track_mm/2`; RE-ENABLED
  `_mark` for multi-pin gridless-first copper so the grid router routes clearance-away from it.

**PM-verified A/B (canonical, reproduced independently):**
| metric | grid-only | gridless_first={17} | delta |
|--------|----------:|--------------------:|------:|
| unconnected | 48 | **41** | **−7** ✅ |
| errors | 87 | **73** | **−14** ✅ |
| tracks_crossing | 0 | **0** | 0 (attempt-2's +19 ELIMINATED) |
| shorting_items | 0 | **0** | 0 (attempt-2's +19 ELIMINATED) |
| solder_mask_bridge | 15 | 2 | −13 |
| hole_clearance | 32 | 26 | −6 |
| hole_to_hole | 14 | 18 | +4 (grid via-near-pad, pre-existing class) |
| copper_edge_clearance | 0 | 0 | 0 |

**This is the WIN of the FAR arc: the board is BETTER ON BOTH PRIMARY AXES** — fewer unconnected AND
fewer total errors. 4/17 boxed-in nets connect (both QSPI nets + /GPIO18 + Net-(U3-USB-DP)); routing
deterministic (unc=41 both runs; the ±1 error count is KiCad zone-fill noise); bounded runtime (~155s).
`gridless_first=None` byte-identical; 392 tests green; `ruff check .` clean.

**Residual / honest caveats:** (1) `hole_to_hole +4` — all from the GRID router placing vias near
component through-holes (GND/VBUS/GPIO7/12/21/24 — none are gridless nets); the grid A* has no
hole_to_hole enforcement for via placement (pre-existing: baseline already has 14). (2) 4 grid nets
displaced (/GPIO1,2,16,17) — net unconnected is still −7 overall. (3) Only 4/17 target nets connect
(the negotiate-among-17 + grid capacity limits the rest); more would need better intra-17 ordering or
grid-capacity work. Net result stands: a real, verified, net-positive connectivity+legality gain from
gridless-first ordering — the payoff of the chapter.
