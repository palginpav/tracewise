# Design: FAR gridless / shape-based router (Convex Expansion Rooms)

Status: architecture (2026-06-19). Blueprint for the FAR build named in
`docs/design/EXACT-GEOMETRY-ROUTER-ARCH.md` §FAR. Substrate decision is settled by
`docs/research/FAR-gridless-routing-survey.md` (Approach #1, Convex Expansion Rooms,
fit 4/5, **Recommend**). This doc resolves the 6 open questions enough to build, fixes
the Shapely dependency + determinism policy, defines module structure + plug points, the
`GridlessNetRoute`→`NetRoute` adapter, the staged milestone plan, and the full Spike-0 spec.

NO production code here. Pseudocode only where it pins a contract.

---

## Overview

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

**Pattern/KB check:** `pattern_find` + `kb_search` returned zero matches for gridless/rooms/
Shapely — this is greenfield in the orchestration KB; no prior decision is contradicted or
extended. A `decisions/` entry should be written once Spike-0 reports (see task list).

---

## Scope

- **Files to create (production, by the developer in M1+, NOT in Spike-0):**
  - `src/tracewise/route/gridless/__init__.py` — package marker + public surface.
  - `src/tracewise/route/gridless/geom.py` — Shapely wrapper + determinism shims
    (`set_precision`, deterministic polygon→ordered-vertex canonicalization, the
    `HAVE_SHAPELY` flag and import guard).
  - `src/tracewise/route/gridless/rooms.py` — `Room`, `RoomGraph`, Minkowski inflation of
    obstacles, lazy convex room expansion (`expand_room`), room↔room portal computation.
  - `src/tracewise/route/gridless/search.py` — continuous A* over room nodes
    (`RoomNode`), deterministic ordering/tie-break, history pricing hook.
  - `src/tracewise/route/gridless/realize.py` — room-sequence → centerline polyline
    (funnel/shrink within the room corridor), world-coordinate waypoints.
  - `src/tracewise/route/gridless/adapter.py` — `GridlessNetRoute` and
    `to_netroute(...)`; `route_net_gridless(...)` (the per-net entry matching
    `route_net`'s contract).
  - `src/tracewise/route/gridless/congestion.py` — room→super-cell congestion mapping
    (decision 3).
  - `scripts/spike0_gridless_single_net.py` — the standalone Spike-0 script (built FIRST,
    before the package; see Spike-0 spec).
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
run-to-run *on the same install*, which `set_precision` + canonical ordering achieves. If
Spike-0 measures any run-to-run divergence on one install, fall back to the existing numpy
`exact_geom` predicates for the legality-critical operations (clearance check, push-out) while
keeping Shapely for the non-critical polygon bookkeeping (union/difference for room shape). The
numpy fallback is ALREADY BUILT and tested (`exact_geom.py`, 46 fixture tests) — it is the
determinism insurance, not a future cost.

---

## Determinism strategy (the mandatory hard constraint)

Routes must be byte-identical run-to-run. Three sources of nondeterminism, each closed:

1. **GEOS float noise.** Quantize every geometry to a fixed grid with
   `shapely.set_precision(geom, 1e-6)` (1 nm — finer than the 1 µm emit rounding, so no
   information loss, but it snaps GEOS's sub-nm float wobble to a deterministic lattice).
   Apply at every room-construction boundary (after buffer, after difference, after
   intersection). This makes the polygon *coordinates* reproducible on a fixed install.
2. **Iteration order.** Python set/dict iteration over Shapely geometries and over obstacle
   collections must never drive control flow. Canonicalize: obstacles are processed in a
   fixed sorted order (by `(layer, x1, y1, kind)`); room vertices are stored in a canonical
   rotation (start at the lexicographically smallest vertex, fixed winding); the A* open set
   is a heap keyed by `(f_cost, tie_break)` where `tie_break` is a deterministic integer
   (room insertion sequence counter) so equal-cost nodes pop in a fixed order. NEVER key the
   heap on a Shapely object or a float-only tuple.
3. **Room expansion order.** Lazy expansion enqueues neighbor rooms; the enqueue order is
   the sorted portal order (by portal midpoint `(x, y)` rounded to 1 nm). Tie-break on portal
   index. This is the gridless analogue of the grid router's fixed `DIRS` list.

**Verification gate (Spike-0 and every milestone):** run the route twice in the same process
and once in a fresh process; assert the emitted `.kicad_pcb` track/via coordinates are
byte-identical across all three. This mirrors how the grid router's determinism was hardened
(work-bounded, no wall-clock deadline). Cross-platform/cross-GEOS-version identity is NOT
promised (documented limitation) — same-install reproducibility is.

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
    rooms_used: list[int] = field(default_factory=list)  # room ids -> congestion crediting

def to_netroute(net, world_paths, world_vias, grid, geo) -> GridlessNetRoute:
    """Build a GridlessNetRoute whose grid-side fields (cells, via_sites) are the
    rasterization of the world centerline at the net's inflation, so the SHARED grid
    ledger and rip-up see gridless copper identically to grid copper. world_paths/world_vias
    carry the exact geometry for emit. ok/reason/unroutable_pads as in route_net."""
```

Rationale for IS-A over a parallel type: `route_all`, `_mark`, `_nearest_victim`, the salvage
pass, and the summary all consume `NetRoute` fields; subclassing means zero changes there. The
only code that knows about `world_paths` is `emit_routes` (one branch) and the adapter itself.

**Room / A* node types (pseudocode):**

```python
@dataclass(frozen=True)
class Room:
    id: int                       # deterministic insertion-order id (tie-break key)
    layer: int
    poly: "shapely.Polygon"       # convex, clearance-inflated free space, precision-snapped
    portals: tuple["Portal", ...] # shared edges to neighbor rooms, canonical-sorted

@dataclass(frozen=True)
class Portal:
    to_room: int
    seg: tuple[tuple[float, float], tuple[float, float]]  # the shared free-space edge
    midpoint: tuple[float, float]                          # rounded 1nm, sort key

@dataclass(frozen=True)
class RoomNode:                   # A* state
    room_id: int
    entry_xy: tuple[float, float] # where the path enters this room (continuous)
    layer: int
```

A* over `RoomNode`: g = accumulated centerline length + via_cost per layer change +
history price; h = Euclidean distance entry_xy→target (admissible, plus via_cost if layer
differs — same shape as `astar.h`). Neighbors = the room's portals (cross to `to_room`,
new entry_xy = closest legal point on the portal segment to the target, à la funnel). The
existing octile heuristic is replaced by straight Euclidean since geometry is now continuous.

---

## Decision 3 — Negotiated-congestion history on rooms (room identity across rip-up)

Survey open Q3. Rooms are lazily generated and change shape when a neighbor net is ripped up,
so a room id is NOT stable across iterations — pricing on room id directly is unsound.

**Resolution: option (b) — a fixed super-cell congestion lattice, decoupled from room identity.**
Tile the board into a coarse fixed grid (e.g. 0.5 mm super-cells, ~5× the routing pitch) at
problem-build time. This lattice never changes across rip-up. History pricing lives on
super-cells, exactly like the grid router's `history[layer,iy,ix]` lives on routing cells —
same data structure (`np.zeros((layers, sy, sx))`), same `+= 1.0` deposit on rip-up, same
`(1 + history_factor * history[supercell])` multiplier applied to a room's traversal cost
(weighted by the centerline length crossing each super-cell).

Why super-cells, not room-id (a) or continuous-field (c): (a) is unstable as noted; (c)
(distance-from-obstacle field) prices proximity, not contention history, so it cannot carry
the cross-iteration "this corridor keeps causing rip-ups" signal that `multi.py` relies on.
Super-cells give a stable spatial key, reuse the exact numpy history mechanism and
`history_factor` parameter, and let grid-routed and gridless-routed nets **share one
congestion field** — a grid net's rip-up raises the price for a gridless net crossing the
same region and vice versa. This is what makes mixed routing negotiate jointly rather than as
two disconnected routers. `congestion.py` owns the cell↔super-cell mapping
(`supercell_of(x, y) -> (sy, sx)`) and the cost-accumulation helper.

Cross-iteration room reuse for performance is an optimization, NOT a correctness requirement:
rooms are rebuilt per net-route attempt; the super-cell history is the only thing that
persists. M2 may cache rooms keyed by the obstacle-set hash if profiling demands it.

---

## Decision 4 — Via insertion & 2-layer in the room model; pour lifecycle

**Via / 2-layer.** Two `RoomGraph`s, one per layer (F.Cu, B.Cu), each built over its own
obstacle set. A via is a special node transition: from `RoomNode(room, xy, layer=0)` the
search may emit a *via transition* to `(room', xy, layer=1)` iff a legal via can be placed at
`xy` — i.e. a disc of radius `via_mm/2 + clearance` is inside free space on BOTH layers at
`xy` (the gridless analogue of `astar.via_ok`'s VIA_RING check, but exact). Cost = the
existing `via_cost` parameter (reuse it; do NOT invent a geometric via cost — survey Q4: keep
parameter parity with the grid router so the scorecard comparison is apples-to-apples; a
geometric detour cost can be an M3 refinement if measured to help). The via centre `xy`
becomes an obstacle (a `circle` of radius `via_mm/2`) on both layers, splitting any room it
sits in — handled by re-inflating that point into the obstacle set for subsequent rooms in
the same net's search, and rasterized into `via_sites` by the adapter so other nets see it.

Drill geometry: `extract.py`/`extract_pads` lacks drill data; vias use the project's
`via_mm`/`via_drill_mm` from `project_geometry` (already wired in `emit_routes`). Through-hole
pad holes are obstacles via their pad rect already; hole-to-hole spacing (a DRC class seen on
mitayi) is handled by the via-placement clearance check using the obstacle's copper radius.

**Pour lifecycle.** Survey Q5. Resolution: **option (a) — pre-inflate pours as polygon
obstacles before room generation, NOT rebuild-after-each-net.** Pours are large and slow to
re-difference per net; rebuilding rooms after every net would dominate runtime. Instead, at
problem-build the existing F0 pour extraction / rasterized pour geometry is converted to
Shapely polygons, inflated by clearance, and added to the per-layer obstacle set used for
room construction. Post-route `refill_zones` (the existing pcbnew re-pour) still runs at the
end exactly as today — pours refill around the new copper with their own clearance, so the
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
| **Spike-0** | Prove the geometry pipeline end-to-end on ONE net | 1 chosen 2-pin net on mitayi: 0 DRC errors on the emitted trace, runtime < 5 s, byte-identical across 2 runs (1 same-process + 1 fresh-process). Standalone script, no engine changes. | 2–3 days |
| **M1: single-net robust + adapter** | `route_net_gridless` + `GridlessNetRoute` + emit branch; route ANY single 2-pin net (incl. one requiring a via) clean | On a fixture set of ≥10 individually-routed mitayi 2-pin nets: 100% routed, 0 DRC errors on each emitted trace, deterministic; adapter produces a `NetRoute` that `_mark` accepts. No multi-net interaction yet. | 3–4 wk |
| **M2: multi-net + congestion** | `route_all` dispatch + super-cell history shared with grid; rip-up works for gridless nets | Route a SUBSET (e.g. all 2-pin signal nets) of mitayi gridlessly + remainder on grid: total errors ≤ grid-only baseline on that subset AND unconnected ≤ baseline; deterministic; rip-up converges within budget. | 4–6 wk |
| **M3: vias / full 2-layer** | Via transitions + B.Cu rooms in the search; multi-pin connection trees | Route nets requiring layer changes gridlessly: hole_clearance + hole_to_hole DRC classes ≤ grid baseline; vias placed at legal positions (0 via-clearance errors on gridless nets). | 3–4 wk |
| **M4: full board, parity then beat** | All nets gridless (grid only as failure fallback); profiled to acceptable runtime | mitayi HUMAN placement: **unconnected ≤ grid baseline AND errors < grid baseline** (parity gate), then stretch: **unconnected ≤ ~5 AND errors ≤ human on mitayi + zuluscsi** (the EXACT-GEOMETRY-ARCH FAR definition-of-done). Runtime within ~2× grid `--quality` mode. | 6–10 wk + profiling |

Gate discipline: do NOT advance a milestone until its exit criterion is measured green on the
scorecard. M2 is the go/no-go for the whole substrate — if shared-congestion mixed routing
cannot reach baseline parity on a subset, reassess (CDT navmesh fallback or scope cut) before
sinking M3/M4 effort.

---

## Spike-0 — precise, decisive, cheap (built NEXT by a developer)

**Goal:** prove the realize→emit→DRC loop on real Shapely room geometry for one net, with
zero engine changes, so the substrate is validated before any package is written.

**Net / region (concrete, tractable):** pick the SHORTEST 2-pin net on mitayi whose two pads
are on the SAME layer (F.Cu) and clear of the worst fan-out (the QFN/connector escape
clusters). Selection is mechanical, not hand-picked: load `extract_pads(mitayi)`, build the
net→pads map (as `build_problem` does), filter to nets with exactly 2 pads both front-layer,
compute straight-line pad distance, and choose the **shortest** such net that has at least one
other-net pad within ~2 mm of its straight line (so the room expansion actually has to clip
against an obstacle halo — a trivial open-field net would not exercise the geometry). Record
the chosen net name in the script output and the design follow-up. Same layer + short + one
nearby obstacle = exercises Minkowski inflation, room growth, portal crossing, and centerline
realization WITHOUT vias/multi-net/rip-up.

**Procedure (standalone `scripts/spike0_gridless_single_net.py`, reusing existing inputs):**
1. Copy the mitayi board to a temp out dir and `strip_routing` (mirror `_probe_route_human.py`
   setup). Use `extract_pads` + `build_problem` for obstacles/anchors/geo — do NOT rebuild
   extraction.
2. Pick the net per the mechanical rule above. Collect obstacle polygons for F.Cu: every
   OTHER net's pad rect (from `obstacles[0]`) + the board boundary as the outer clip.
3. Minkowski-inflate each obstacle by `clearance_mm + track_mm/2` using Shapely `buffer`
   (`set_precision` to 1 nm after). Free space = `board_bbox.difference(unary_union(inflated))`.
4. Lazily grow convex rooms source→target: start a room at the source pad, expand a convex
   sub-polygon of free space toward the target until it hits an obstacle halo, register it,
   enqueue portal edges; A* over rooms (deterministic tie-break) to the target pad.
5. Realize the room sequence into a centerline polyline (funnel/shrink), in world mm.
6. Emit the trace into the temp board as `(segment …)` nodes via the existing sexpr writer
   (the script may call a minimal inline emitter or a one-net `emit_routes` with
   `world_paths`); `refill_zones`; `run_drc`.

**Pass criteria (ALL must hold):**
- `run_drc` reports **0 errors** attributable to the emitted trace (clearance, short, etc.).
- The net is connected (the emitted polyline joins both pads; ratsnest for it resolved).
- **Runtime < 5 s** for the single net (Shapely-perf gate).
- **Deterministic:** run the script twice — once in the same process, once fresh — and assert
  the emitted segment coordinates are byte-identical (determinism gate).

**Validates:** Shapely 2.x install + GEOS availability; Minkowski inflation correctness;
lazy convex room expansion produces a connected room graph; A* over rooms finds the corridor;
funnel realization yields a legal centerline; the realize→emit→DRC loop closes with 0 errors;
`set_precision` + canonical ordering deliver run-to-run determinism on this install.

**Explicitly DEFERS:** congestion/history pricing, vias + the 2nd layer, multi-net + rip-up,
the `GridlessNetRoute`/`route_all` adapter, pour pre-inflation fidelity, full-board Shapely
performance (profiled only after Spike-0 passes). If Spike-0 measures float nondeterminism
that `set_precision` cannot tame, the fallback is the numpy `exact_geom` predicates for the
legality-critical ops — note it in the Spike-0 report; do NOT abandon the approach on a single
determinism wobble.

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

# rooms.py
def build_obstacle_polys(obstacles, layer, net_name, inflate_mm) -> "shapely.MultiPolygon": ...
def expand_room(free_space, seed_xy, toward_xy) -> Room | None: ...

# search.py
def astar_rooms(graph, start_xy, goal_xys, layer, via_cost, history_supercells, history_factor)
    -> list[RoomNode] | None: ...

# realize.py
def realize_centerline(room_seq: list[RoomNode]) -> list[tuple[float, float, int]]: ...
```

`GridlessNetRoute` IS-A `NetRoute` (see Decision 2). `Obstacle` reuses the `exact_geom.py`
tagged-tuple union (`("rect", x1,y1,x2,y2)`, `("circle", cx,cy,r)`, `("segment", ax,ay,bx,by,hw)`).

---

## Dependencies

- **External:** `shapely>=2.0,<3.0` (BSD-3, optional extra `gridless`). GEOS ships in the
  Shapely wheel. No CGAL unless Spike-0/M2 measures Shapely instability that `set_precision`
  + the numpy fallback cannot resolve (escalation path, not a planned dependency).
- **Internal (consumed/extended):** `extract_pads`/`build_problem`/`project_geometry`/
  `emit_routes`/`refill_zones`/`route_board_engine` (`kicad.py`); `Net`/`NetRoute`/`route_all`/
  `route_net`/`order_nets` (`multi.py`); `exact_geom.py` predicates (determinism fallback);
  `run_drc`/`strip_routing` (`bridge.py`); the scorecard `_probe_route_human.py`.

---

## Risks and Mitigations

| Risk | Likelihood | Impact | Mitigation |
|------|-----------|--------|------------|
| GEOS float ops nondeterministic run-to-run | Med | High | `set_precision(1e-6)` + canonical vertex/portal ordering + integer A* tie-break; verify in Spike-0; numpy `exact_geom` fallback for legality ops |
| Shapely too slow at full board scale (union/difference per room) | Med | High | Lazy expansion (rooms only on A* frontier); per-net obstacle set, not whole board; profile after Spike-0; cache rooms by obstacle-hash in M2; CDT navmesh as substrate fallback |
| Degenerate / near-zero-area rooms near tight clearance | Med | Med | Minimum-room-area threshold; skip sub-threshold rooms; precision snap removes sub-nm slivers; fall back that net to grid router |
| Room congestion signal too coarse vs grid cells | Low | Med | Super-cell lattice tunable (start 0.5 mm); shared field with grid nets; `history_factor` already a parameter |
| Mixed grid+gridless double-counts occupancy | Med | High | Single shared grid ledger: gridless adapter rasterizes its centerline into `nr.cells`/`via_sites` at the same inflation; `_mark` treats both identically |
| Funnel/realization places centerline off-portal → DRC error | Med | High | Realize within the inflated corridor only; post-realize `is_legal` assert per segment (numpy predicate); Spike-0's 0-error gate catches it early |
| Via legality check (both layers) misses a layer | Low | High | Exact two-layer disc-in-free-space check before emitting a via; rasterize via into both layers of the shared ledger |
| Effort overrun (3-6 mo solo) | Med | Med | M2 is the go/no-go gate; gate-per-milestone discipline; CDT fallback if parity fails |

---

## Testing Strategy

- **Spike-0:** the script IS the test — DRC 0-error gate + <5 s + byte-identical 2-run gate.
- **Unit (M1+):** `geom.py` — Minkowski inflation against hand-computed offsets; precision
  snap idempotence. `rooms.py` — convexity of every produced room; portal symmetry (A↔B);
  determinism of expansion order. `search.py` — A* finds known-shortest corridor on a fixture
  with 2–3 obstacles; tie-break determinism (run twice, identical node sequence). `realize.py`
  — centerline legality (`is_legal` per segment against the numpy predicates). `adapter.py` —
  `to_netroute` rasterization matches a grid route's `cells` footprint for a straight trace.
- **Integration (M2+):** mixed grid+gridless on a small fixture board: shared history field
  updates from both substrates; rip-up of a gridless net frees its grid cells. Determinism
  across the full `route_all` (byte-identical emitted board, 2 runs).
- **Scorecard (every milestone):** `_probe_route_human.py` on mitayi (and zuluscsi at M4);
  compare `unconnected`/`errors`/`by_type` against the grid baseline and the milestone exit
  criterion. This is the canonical acceptance measure.

---

## Acceptance Rubric

Per `agents/pm-reference/rubric-format.md`. Atomic, binary, evidence-mandatory. Applies to
THIS design artifact + the Spike-0 it specifies.

1. **R1 — Shapely decision is concrete and pinned.** PASS iff the doc states yes/no, a
   pyproject version range, the dependency category (core vs optional extra), and the license
   compatibility. Evidence: Decision 1 (`shapely>=2.0,<3.0`, optional extra `gridless`, BSD-3).
2. **R2 — Determinism strategy is specific and verifiable.** PASS iff it names the GEOS
   determinism mechanism, the ordering/tie-break for room expansion AND A*, and a concrete
   verification gate. Evidence: Determinism section (`set_precision` 1e-6, canonical
   vertex/portal order, integer heap tie-break, 2-run byte-identical gate).
3. **R3 — Module structure + exact plug points named.** PASS iff the new package path, its
   modules, and the named functions/parameters modified in `route_board_engine`/`route_all`/
   `emit_routes` are all specified. Evidence: Scope + Decision 2 (plug points A/B/C).
4. **R4 — Congestion-on-rooms identity resolved.** PASS iff the doc picks one of survey Q3's
   options with rationale and explains room identity across rip-up. Evidence: Decision 3
   (super-cell lattice, option b, shared field).
5. **R5 — Via + 2-layer + pour lifecycle decided.** PASS iff via cost source, the two-layer
   room model, the via legality check, and the pour lifecycle (pre-inflate vs rebuild) are
   each stated. Evidence: Decision 4.
6. **R6 — 5-milestone plan with measurable scorecard exit criteria.** PASS iff Spike-0→M1→M2→
   M3→M4 each have a goal, a scorecard-referenced exit criterion, and an effort estimate.
   Evidence: Decision 5 table.
7. **R7 — Spike-0 fully specified.** PASS iff the doc names the concrete net-selection rule,
   the exact pass criteria (0 DRC errors, <5 s, deterministic 2-run), what it validates, and
   what it explicitly defers. Evidence: Spike-0 section.
8. **R8 — GridlessNetRoute→NetRoute adapter contract pinned.** PASS iff the adapter type,
   its relationship to `NetRoute`, and how both coexist in `route_all` (shared grid ledger)
   are specified in pseudocode. Evidence: Decision 2 adapter pseudocode + IS-A rationale.
9. **R9 — No production code written.** PASS iff the artifact contains only pseudocode/
   contracts, no runnable implementation files created. Evidence: only this `.md` written.
10. **R10 — Spike-0 developer task list is ordered and standalone.** PASS iff a numbered task
    list exists that a developer can execute without the full engine, reusing
    `_probe_route_human.py`/`build_problem`. Evidence: task list below.

---

## Spike-0 — ordered developer task list

1. Add `shapely>=2.0,<3.0` to a local venv (`pip install 'shapely>=2.0,<3.0'`); confirm
   `shapely.geos_version`. Do NOT yet edit `pyproject.toml` (that lands in M1).
2. Create `scripts/spike0_gridless_single_net.py`. Copy mitayi to a temp out dir and
   `strip_routing`, mirroring `scripts/_probe_route_human.py` setup (reuse its pattern).
3. Call `extract_pads` + `build_problem` + `project_geometry` to get obstacles, anchors,
   anchor_rects, and geo (track/clearance/via mm). Do not reimplement extraction.
4. Implement the mechanical net-selection rule: 2-pin nets, both pads F.Cu, shortest
   straight-line distance, with ≥1 other-net pad within ~2 mm of the line. Print the chosen
   net name.
5. Build the F.Cu obstacle polygons (other nets' pad rects + board boundary), Minkowski-
   inflate by `clearance_mm + track_mm/2` with Shapely `buffer`, `set_precision(geom, 1e-6)`,
   `unary_union`, and difference against the board bbox to get free space.
6. Implement lazy convex room expansion source→target + deterministic A* over rooms (sorted
   portal order, integer tie-break). Keep it minimal — one net, one layer, no vias.
7. Implement funnel/shrink realization → world-mm centerline polyline; assert each segment
   legal via the numpy `exact_geom` predicates before emitting.
8. Emit the centerline as `(segment …)` into the temp board via the sexpr writer (inline
   minimal emitter or a one-net `emit_routes` call with `world_paths`); `refill_zones`.
9. Run `run_drc`; print `unconnected`/`errors`/`by_type`; assert 0 errors on the emitted trace
   and the net connected.
10. Add the determinism gate: run steps 5–9 twice (same process + a fresh subprocess invocation
    of the script) and assert byte-identical emitted segment coordinates.
11. Time the run; assert < 5 s for the single net. Print elapsed.
12. Report: chosen net, DRC result, runtime, determinism pass/fail, and any `set_precision`
    instability observed (→ feeds the architect's go/no-go + the `decisions/` KB entry).

---

## Honest gaps / open risks

- Freerouting LOC reference unverified (survey-inherited); the room-expansion algorithm is
  re-implemented from description, not ported.
- Full-board Shapely performance is unmeasured until post-Spike-0 profiling — the M2 gate
  exists precisely to catch this before M3/M4 sink effort.
- Cross-platform / cross-GEOS-version byte-identity is NOT promised; same-install
  reproducibility is. If CI runs a different GEOS than dev, golden-board determinism tests
  must pin the GEOS version (record in the M1 CI setup).
- `extract.py`/`extract_pads` lacks drill geometry; vias rely on project `via_mm`/`via_drill_mm`
  — adequate for uniform vias, insufficient if a board mixes via sizes (out of v1 scope).

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
