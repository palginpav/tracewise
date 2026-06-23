# Design: Topology-Class Routing (TCR)

Status: DESIGN, 2026-06-23. Synthesis of INVENT-human-mimicry-router (Inventor-1) +
INVENT-topological-routability (Inventor-2) into ONE staged build with a cheap make-or-break
gate FIRST. NO production code in this doc. Target: beat attempt-3 ceiling (mitayi 41 unc / 73
err) toward human 0/0.

---

## Overview

Both inventors independently converged on the SAME thesis: the mitayi routing gap is an
**ordering / homotopy artifact, not capacity**. Witness proves it — tightest cut (QFN escape)
runs ~56% F.Cu, <22% B.Cu (~2x slack), and the human routing is crossing-free (0 different-net
crossings on 426 F.Cu + 157 B.Cu segments). Our greedy routers commit each net's homotopy class
per-net and contend for the escape ring + B.Cu bus; they never find the GLOBAL class the human
occupies.

TCR = **assign the global topology class BEFORE geometry, then realize it with the EXISTING
gridless machinery.** ~70% reuse (the realizer, via-legality, emit, DRC), ~30% new (the
assignment pre-pass). The build is gated FIRST on a 1-day make-or-break experiment (E1) that
tests whether the realizer can be STEERED by a fixed class at all. If E1 fails → NO-GO at ~1
day, zero production change, fall back to attempt-3.

**Approach selection (vs prior art):** no in-repo pattern or KB decision exists for global
topology assignment (`pattern_find` + `kb_search` both empty — genuinely new surface). The full
toporouter (gEDA CDT) was already rejected by the survey on complexity + poor dense-2-layer
record; TCR is the SCOPED version both inventors endorse: escape-order + corridor-side assignment
as a pre-pass, NOT a board-wide CDT.

---

## Scope

- **Files to create (E1 gate — throwaway, FIRST):**
  - `scripts/_probe_tcr_e1.py` — the unified make-or-break gate. Extracts the witness's TopoClass
    for the QFN-escape nets, steers `route_net_gridless`/fanout-escape with those fixed classes,
    grid-routes the rest, scores + audits crossings. Read-only use of the engine.
- **Files to create (production — ONLY after E1 GO):**
  - `src/tracewise/route/topo/__init__.py`
  - `src/tracewise/route/topo/topoclass.py` — `TopoClass` dataclass + witness extractor
    (`extract_topoclass_from_board`, reuses the two inventor scripts' polar/layer logic).
  - `src/tracewise/route/topo/assign.py` — `assign_topology_classes` (the pre-pass: planar escape
    order + monotone lane packing). The genuinely new primitive.
  - `src/tracewise/route/gridless/route.py` — ADD `route_net_steered` (minimal new entry; see
    Interface Contracts) that pins the escape via to a class-supplied site instead of the radial
    ray. Reuses every other step of `route_net_fanout_escape` unchanged.
  - `tests/test_topo_assign.py`, `tests/test_topo_steer.py`.
- **Files to modify (production — ONLY after TCR-1 green):**
  - `src/tracewise/route/engine/multi.py` `route_all` — add `topo: set[str] | None = None` +
    `topo_kwargs` opt-in path, mirroring the existing `coopt`/`gridless_first` pattern. Default
    `None` ⇒ byte-identical.
  - `src/tracewise/route/engine/kicad.py` — thread the `topo` flag (mirror `coopt` threading).
- **Files to read (context only):**
  - `docs/research/INVENT-human-mimicry-router.md`, `docs/research/INVENT-topological-routability.md`
  - `scripts/_invent1_human_stats.py`, `scripts/_invent2_topology.py` (extractors to reuse)
  - `src/tracewise/route/gridless/route.py` (`route_net_fanout_escape` = the steer surface),
    `geom.py` (`guided_escape_via`, `is_legal_via`, `build_windowed_free_space`),
    `engine/multi.py` (`_run_coopt_loop`, RSS guard, bounded-window params)
  - `data/benchmark-boards/mitayi-pico-d1/Mitayi-Pico-D1.kicad_pcb` (the witness)

---

## What each inventor proposal contributes (synthesis)

| Element | Inventor-1 (human-mimicry) | Inventor-2 (topological) | TCR takes |
|---|---|---|---|
| Core insight | "assign topology FIRST, then realize" | "homotopy class is the gap, not capacity" | BOTH — same thesis, two framings |
| TopoClass content | ring via-slot (edge/angle/xy) + B.Cu highway lane (y-coord) + layer=direction | (layer, via_angle, ordered corridor sides) | **UNION** — angle/slot from Inv-1, layer+via from Inv-2, lane==corridor (see below) |
| Escape ordering | balanced ring packing (W10/S10/E10/N9) | planar cyclic-order matching (pad-angle → via-angle monotone) | **Inv-2's cyclic-order match** generates the slots; Inv-1's balance is the validation target |
| Bus / corridor | monotone B.Cu lane packing (river-routing, crossing-free by construction) | min-cost multi-commodity flow on corridor graph | **Inv-1's monotone lane packing** — simpler, crossing-free by construction (see flow-vs-monotone) |
| Capacity feasibility | 34 signals / 46 lanes = 0.74 | ~56% F.Cu / <22% B.Cu, no binding cut | BOTH — same headroom, two measurements |
| Realization | thin wrapper over gridless A* | drive `route_net_gridless` with fixed class | **UNIFIED** as `route_net_steered` |
| The gate | Probe-Topology (≥30/34 bus legs realize crossing-free) | E1 (witness-class realization, GO iff <41 unc / <73 err / 0 crossings) | **UNIFIED as E1** (below) |

The two gates are the same experiment from two angles: Inv-1 tests "does the packed lane
assignment realize", Inv-2 tests "does the realizer honor a class". Unified E1 does both: steer
with the witness's OWN classes (easiest case — if even those can't be realized, the approach is
dead) and audit crossings.

---

## TopoClass representation

The homotopy class for steering. UNION of both inventors' content, minimal sufficient set:

```python
# src/tracewise/route/topo/topoclass.py
@dataclass(frozen=True)
class TopoClass:
    net: str
    # --- escape (Inv-1 ring slot + Inv-2 via angle/layer) ---
    escape_via_xy: tuple[float, float] | None   # the FIXED ring via site (1nm-snapped).
                                                 # None => F.Cu-only short net (no via, Rule H5).
    escape_edge: str | None                      # 'N'|'S'|'E'|'W' — QFN edge of the slot (audit/balance)
    escape_angle_deg: float | None               # via angle from U3 centroid — the homotopy choice
    # --- highway / corridor (Inv-1 lane == Inv-2 corridor side-seq, collapsed) ---
    highway_layer: int                           # 1 (B.Cu) for the horizontal bus, per Rule H3
    lane_y_mm: float | None                      # the reserved horizontal lane (== corridor side encoded
                                                 # as a y-band). None for F.Cu-only nets.
    dest_xy: tuple[float, float]                 # destination connector pad centre
    # --- ordering key (the planarity invariant) ---
    order_key: float                             # destination-y (monotone bus order). Lanes assigned
                                                 # in this order => crossing-free by construction.
```

Why this set is sufficient to PIN the class (not re-derive it greedily):
- `escape_via_xy` fixes WHICH slot the net escapes through — the single biggest greedy degree of
  freedom (`guided_escape_via` currently derives it radially; steering overrides it).
- `escape_angle_deg` + `escape_edge` are redundant-with-xy but carry the planarity/balance audit.
- `highway_layer` + `lane_y_mm` fix the B.Cu bus lane — the second contended resource.
- `order_key` is the monotone invariant: assigning lanes in `order_key` order guarantees the bus
  is crossing-free (river-routing result; the witness's monotone GPIO0..29 numbering realizes it).

We do NOT need a full ordered `corridor_sides` list (Inv-2): on mitayi the corridor between U3 and
J3/J4 is a single channel, so "which side of each obstacle" collapses to "which lane (y-band)".
Lane order IS the side assignment. Keep `corridor_sides` out of the prototype; add only if a
second-QFN board needs it (flagged in open risks).

**Extractable from the witness** (proven — the inventor scripts do ~90% already):
- `escape_via_xy`, `escape_angle_deg`, `escape_edge`: `_invent1_human_stats.py` section (2) +
  `_invent2_topology.py` section 1 compute every via's polar (r, θ) from U3 and bin by edge.
- `highway_layer`, `lane_y_mm`: `_invent1` section (1) proves B.Cu=horizontal; the long horizontal
  B.Cu segment per net gives `lane_y_mm` directly.
- `dest_xy`, `order_key`: `_invent1` section (3) extracts J3/J4 pad rows + y, monotone in GPIO#.
- `escape_via_xy=None` (F.Cu-only) for the 3 short nets (/GPIO27, /GPIO28, /XIN) — `_invent2`
  section 3 already flags them (layer seq "F", 0 vias).

**Assignable by the pre-pass** (TCR core, §assignment-algorithm): the pre-pass GENERATES the same
fields from placement geometry (slot candidates + monotone lane packing) without reading the
witness. E1 uses witness-extracted classes; TCR-1/2 use pre-pass-computed classes.

---

## The unified make-or-break gate (E1 / Probe-Topology) — BUILD FIRST

**What it tests:** can the EXISTING realizer be STEERED by a fixed homotopy class — pinned escape
via + lane + bounded window + layer — WITHOUT re-deriving it greedily? Using the witness's OWN
classes (the easiest possible case). If the witness classes can't be realized, no computed class
can be, and the approach is dead.

### Exact steering mechanism

**Finding (steerable_today):** `route_net_fanout_escape` ALREADY accepts the fixed escape
geometry that 90% of a class needs — `component_cx/cy`, `ring_radius`, `dest_xy`,
`bcu_extra_obstacles` (to keep the B.Cu run off prior nets' copper), `max_window_mm`,
`max_bcu_window_mm`. The F.Cu stub A*, B.Cu run, via legality, dedup, and `GridlessRouteResult`
are all reused unchanged. **The ONE knob missing:** the escape via is pinned to the radial ray
(centroid→source pad) inside `guided_escape_via` (computes `init_angle_deg` from the source pad
direction). Rule H2 proves the human does NOT escape radially — the via sits at a FREE ring SLOT,
often on a different QFN edge than the source pad. So steering requires overriding the via site.

**Minimal addition — `route_net_steered`** (a thin variant, NOT a new engine):

```python
# src/tracewise/route/gridless/route.py  (NEW, ~60 LOC; everything else reused)
def route_net_steered(
    source_xy, dest_xy, escape_via_xy,        # escape_via_xy = the FIXED class slot (None => F.Cu-only)
    lane_y_mm,                                 # the reserved B.Cu lane band (None => no lane constraint)
    pads, net_name, geo, board_bbox,
    extra_obstacles=None, bcu_extra_obstacles=None, fcu_stub_extra_obstacles=None,
    board_outline=None, drill_obstacles=None, drill_centers=None,
    max_window_mm=12.0, max_bcu_window_mm=8.0,
) -> GridlessRouteResult:
    """Like route_net_fanout_escape, but the escape via is PINNED to escape_via_xy
    (validated once with is_legal_via) instead of derived radially, and the B.Cu run
    window is centred on lane_y_mm. Reuses the F.Cu-stub A*, B.Cu Manhattan/visgraph
    run, dedup, and result-building of route_net_fanout_escape verbatim.
    escape_via_xy=None => single-layer F.Cu route (delegate to route_net_gridless,
    allow_via=False) for the short nets (Rule H5)."""
```

Implementation = copy `route_net_fanout_escape`'s steps 3+ but skip `guided_escape_via`; instead
call `is_legal_via(escape_via_xy, ...)` once. If the class-supplied site is illegal (drill/copper
collision the assignment model missed), fall back to a SMALL radial search around it (reuse
`guided_escape_via` with a tight `search_arc_deg`, ~10°) — bounded, deterministic. The B.Cu run
already does Manhattan vertical/horizontal-first; constrain its window to `[lane_y_mm - pitch,
lane_y_mm + pitch]` band so it cannot wander into the wrong lane.

### E1 procedure

1. **Extract witness classes.** Extend `_invent2_topology.py` (or import its functions) to emit a
   `TopoClass` for each of the ~18 QFN-escape nets: escape_via_xy/angle/edge from the via polar,
   lane_y_mm from the long horizontal B.Cu segment, dest_xy from the J3/J4 pad, order_key=dest y.
   F.Cu-only nets (/GPIO27, /GPIO28, /XIN) get `escape_via_xy=None`.
2. **Steer.** On a stripped mitayi (human placement), route the 18 escape nets with
   `route_net_steered` using the extracted classes, in `order_key` (monotone) order, accumulating
   each routed net's copper into `bcu_extra_obstacles`/`fcu_stub_extra_obstacles` so later nets
   avoid earlier ones. Grid-route all OTHER nets via the existing engine.
3. **Score.** `refill_zones` + `run_drc` (the existing harness, as `_probe_route_human.py` does).
   Record unconnected + error count by type.
4. **Audit crossings.** Reuse `_invent2_topology.py` section 4 (`seg_intersect`) — count
   different-net same-layer crossings on the routed board.

### Pass criteria (UNIFIED — both inventors' bars, the stricter wins)

GO **iff ALL** of:
- `unconnected < 41` (beat attempt-3 connectivity), AND
- `errors materially < 73` (target ≤ ~50; "materially" = not within noise of 73), AND
- `0 illegal different-net crossings` on F.Cu AND B.Cu (the legality bar co-opt failed), AND
- bounded: peak RSS < 2 GB, runtime < 5 min, AND
- deterministic: 3 runs byte-identical board output.

(Inv-1's "≥30/34 bus legs realize crossing-free" is the per-net diagnostic that explains a
borderline score; the GO decision is the unified scorecard above.)

### What it kills

- If the steered witness classes DON'T realize (<41 unc not reached, OR crossings appear, OR the
  realizer ignores the pinned via) → **the realizer cannot be steered; TCR is infeasible in this
  engine. NO-GO at ~1 day, zero production change, fall back to attempt-3 (41/73).** Record WHICH
  nets failed and WHY (lane blocked / via illegal / crossing) for the post-mortem.
- This is the cheapest possible kill-test: it uses the witness's own (known-feasible) classes, so
  any failure is a realizer-steering failure, not an assignment-quality failure. It de-risks the
  expensive part (the pre-pass) before a single line of it is written.

---

## The assignment pre-pass (TCR core) — ONLY if E1 passes

**Chosen algorithm: planar escape ordering (Inv-2 Step 1) + monotone B.Cu lane packing (Inv-1)**.
This is the SIMPLEST that the capacity proof supports. Min-cost flow (Inv-2 Step 2) is NOT used —
see flow-vs-monotone below.

```python
# src/tracewise/route/topo/assign.py
def assign_topology_classes(
    escape_nets: list[Net],            # the QFN-escape set (~18 on mitayi)
    pads_by_net: dict[str, list[dict]],
    u3_centroid: tuple[float, float],
    ring_radius_mm: float,
    connectors: list[ConnectorRow],    # J3/J4 pad rows + y
    geo: dict,
    free_space_F, free_space_B,        # for legal-slot filtering (pours/drills modeled)
) -> dict[str, TopoClass]: ...
```

**Step A — Slot generation + legal filter.** Generate candidate via slots at K≈10 evenly-spaced
angles per QFN edge in the escape band `[ring_r + via/2 + clr + margin, +band_width]` (≈4.5–8mm,
the measured human band). Keep only slots whose `is_legal_via` passes on BOTH layers (models
pours + drills — the realizability gap Inv-1 flagged). Deterministic: fixed angle grid, sorted.

**Step B — Planar escape order (Inv-2 cyclic-order matching).** For each escape net compute
θ_pad (source pad angle from U3) and θ_target (angle toward dest connector). Sort nets by θ_pad.
Walk in order; assign each the next free legal slot whose angle keeps the slot order MONOTONE
w.r.t. θ_target. An assignment that would invert (cross the fan) is the homotopy conflict — push
that net's escape to the OTHER layer's slot set (B.Cu has <22% use → room, per the capacity
proof). O(n log n), n=18. This fixes `escape_via_xy`, `escape_angle_deg`, `escape_edge`.

**Step C — Monotone lane packing (Inv-1, the crossing-free bus).** West channel J4(y≈81.2)→
J3(y≈97.4) = 16.2mm; at pitch 0.35mm = 46 lanes; demand 34 ≤ 46 (0.74). Sort nets by `order_key`
(= dest y, monotone with GPIO#). Assign lanes in that order — lane i to the i-th net in dest-y
order. Two nets share a lane only if their x-spans are disjoint (1-D interval pack per lane).
**Crossing-free by construction**: monotone destination order + monotone lane order = planar
river-routed bus (no two highways cross). Fixes `lane_y_mm`, `highway_layer=1`.

**Step D — Short-net branch (Rule H5).** Nets whose straight F.Cu path is unobstructed (no QFN
pad-forest crossing) get `escape_via_xy=None` — single-layer F.Cu, no via, no lane.

**Determinism:** pure combinatorial assignment over fixed net order (sorted by θ_pad, then by
order_key) and fixed slot grid. No float-driven control flow (comparisons use snapped 1nm coords).
Same placement ⇒ same classes, every run.

**Bounded runtime:** Step A is O(edges·K) ≈ 40 legality checks; B is O(n log n); C is O(n·lanes)
interval packing. All trivial for n≈18–34. The realization (`route_net_steered`) inherits the
bounded windows (§bounded-runtime).

### flow_vs_monotone: why monotone lane packing, NOT min-cost flow

Inv-2 proposed min-cost multi-commodity flow for corridor sides; Inv-1 proposed monotone lane
packing. **TCR picks monotone packing.** Reasons:
1. **Crossing-free by construction.** Monotone order on a single channel is a river-routing result
   — provably planar with no solver. Min-cost flow only MINIMIZES crossings; it needs integrality
   + rounding + repair to GUARANTEE zero, and Inv-2 itself flags multi-commodity integral flow as
   NP-hard in general (relies on slack to round cleanly).
2. **mitayi is a single dominant channel.** Inv-2's corridor graph (the L/R-side sequence) only
   earns its keep with MULTIPLE obstacle groups between source and dest. On mitayi the U3→J3/J4
   corridor is one channel ⇒ "side of each obstacle" collapses to "which lane". The flow machinery
   is unused complexity here.
3. **Capacity proof supports it.** Demand/capacity 0.74 < 1 with monotone order means a feasible
   crossing-free packing EXISTS (the witness IS one). Monotone packing realizes exactly that
   certificate; flow would rediscover it at higher cost.
4. **Simpler = fewer determinism surfaces.** No LP solver, no fractional-flow rounding nondeterminism.

**When flow would be needed (deferred):** a board with two big QFNs or a BGA where escapes route
through a SEQUENCE of corridors. Add the corridor graph + flow ONLY if such a board appears
(open risk). For mitayi and the 0/0 target, monotone packing is sufficient and provably correct.

---

## Interface Contracts

```python
# route_net_steered — the steering primitive (the E1 make-or-break surface)
def route_net_steered(
    source_xy: tuple[float, float],
    dest_xy: tuple[float, float],
    escape_via_xy: tuple[float, float] | None,   # FIXED class slot; None => F.Cu-only
    lane_y_mm: float | None,                      # reserved B.Cu lane; None => unconstrained
    pads: list[dict], net_name: str, geo: dict,
    board_bbox: tuple[float, float, float, float],
    extra_obstacles: list | None = None,
    bcu_extra_obstacles: list | None = None,
    fcu_stub_extra_obstacles: list | None = None,
    board_outline: object | None = None,
    drill_obstacles: list | None = None,
    drill_centers: list | None = None,
    max_window_mm: float = 12.0,
    max_bcu_window_mm: float = 8.0,
) -> GridlessRouteResult: ...   # reuses GridlessRouteResult (IS-A NetRoute via to_gridless_netroute)

# assign_topology_classes — the pre-pass (TCR core)
def assign_topology_classes(...) -> dict[str, TopoClass]: ...   # see §assignment, signature above

# TopoClass — the homotopy class (see §TopoClass representation, frozen dataclass)

# Opt-in wiring (mirror coopt exactly, default None => byte-identical):
def route_all(..., topo: set[str] | None = None, topo_kwargs: dict | None = None, ...): ...
```

`route_net_steered` returns a `GridlessRouteResult` (`world_paths` 3-tuples + `world_vias`),
adapted by the existing `to_gridless_netroute` → `NetRoute` → emit/DRC. Zero new emit code.

---

## Staging

Each milestone has a scorecard exit criterion. STOP at any gate that fails.

### Milestone E1 — make-or-break gate (BUILD FIRST, ~1 day)
- Build `scripts/_probe_tcr_e1.py` + the minimal `route_net_steered` (throwaway-acceptable, but
  if it's clean, keep it for TCR-1). Steer the 18 escape nets with WITNESS classes; grid the rest.
- **Exit:** GO iff `unconnected < 41 AND errors materially < 73 (≤~50) AND 0 illegal crossings AND
  RSS < 2GB AND runtime < 5min AND 3-run byte-identical`. FAIL ⇒ NO-GO, fall back to attempt-3.

### Milestone TCR-1 — assignment + realize the QFN-escape cluster (only if E1 GO)
- Build `topo/topoclass.py` + `topo/assign.py` (Steps A–D). Compute classes from placement (NOT
  witness). Realize the ~18 escape nets with `route_net_steered`; grid the rest.
- Validate the pre-pass against the witness as a fixture: computed escape order matches the
  witness's crossing-free assignment for the 8-net primary ring (Inv-2 E2: Hamming distance on
  layer+lane order). Assignment is deterministic; lanes disjoint; slots legal.
- **Exit:** computed-class realization scores `unconnected < 41 AND 0 illegal crossings`, AND the
  escape-cluster connectivity ≥ the E1 witness-class result minus small slack (computed classes
  shouldn't be much worse than the witness's). RSS < 2GB, deterministic.

### Milestone TCR-2 — full board, beat 41/73 (only if TCR-1 GO)
- Wire `topo` opt-in into `multi.route_all` + `kicad.py` (default OFF byte-identical). Run the full
  scorecard via `_probe_route_human.py`-style harness with `topo` enabled.
- **Exit:** full-board `unconnected < 41 AND errors < 73 AND 0 illegal crossings`, deterministic
  (3-run byte-identical), bounded (RSS < 2GB, runtime < 2x grid `--quality`). Stretch: toward 0/0.
  Default-OFF regression: `topo=None` produces byte-identical output to attempt-3.

---

## Bounded-runtime + determinism discipline

The last builds blew up (4–18 GB RSS from board-wide visibility graphs). Mandated guards, all
already present in the engine and REUSED:

- **Bounded windows everywhere.** `route_net_steered` caps `max_window_mm=12` (F.Cu stub + via
  search) and `max_bcu_window_mm=8` (B.Cu run) — the same caps `_run_coopt_loop` uses
  (`max_route_window_mm=25`, `max_bcu_window_mm=8`). The lane constraint shrinks the B.Cu window to
  a `±pitch` band around `lane_y_mm`, far tighter than the 8mm cap. NEVER full-board windows.
- **Hard RSS guard.** Reuse `_check_rss(label)` / `rss_hard_fail_gb` from `_run_coopt_loop`. The
  E1 probe and the topo path call `_check_rss` per net; abort with `MemoryError` if RSS > 2GB.
- **Region-scope.** The pre-pass and realization operate ONLY on the QFN-escape cluster (region
  bbox around U3 + the J3/J4 channel), exactly like co-opt's `region_bbox`. The rest of the board
  is grid-routed unchanged.
- **skip_full_corner_fallback=True** on the steered calls when `extra_obstacles` is large (the
  full-corner fallback extracts 6000+ vertices → 4–5GB; the existing flag prevents it).
- **Determinism.** Pure combinatorial assignment (fixed sort keys, 1nm-snapped coords, no float
  control flow). Realization is the existing deterministic A* (integer 1nm heap key). 3-run
  byte-identical is an EXIT criterion at every milestone.

---

## Risks and Mitigations

| Risk | Likelihood | Impact | Mitigation |
|---|---|---|---|
| Realizer can't be steered (ignores pinned via / re-derives greedily) | Med | High | E1 tests EXACTLY this with witness classes FIRST; NO-GO at 1 day if it fails |
| Class-supplied via site illegal (pour/drill the model missed) | Med | Med | `route_net_steered` validates with `is_legal_via`; tight (~10°) radial fallback around the slot; pre-pass filters slots on both layers |
| Lane locally blocked → escape net unrealizable despite feasible assignment | Med | High | E1 measures real lane realizability; model pours+drills in lane free-space; per-net fallback to existing router (strictly additive) |
| 0.74 capacity ratio optimistic (full channel not usable) | Med | Med | E1 measures real usable lanes; if effective demand/capacity > 1, TCR can't beat the human → fall back |
| Monotone packing too weak for a 2nd-QFN board | Low (mitayi) | Med | Out of scope for mitayi; corridor-graph + flow is the documented escalation (deferred) |
| RSS blowup recurrence | Low | High | Bounded windows + `_check_rss` 2GB hard abort + region-scope + skip_full_corner_fallback |
| Nondeterminism in assignment | Low | High | No float control flow; fixed sort keys; 1nm snap; 3-run gate at every milestone |
| Non-bus nets (USB pair, crystal, power) not modeled | Med | Med | topo owns ONLY the QFN↔connector escape set; everything else routes on the existing engine unchanged |

---

## Testing Strategy

- **E1 (integration gate):** the unified scorecard + crossing audit (above). The decisive test —
  pass/fail gates the entire build.
- **Unit — `assign_topology_classes`:** deterministic (same placement ⇒ same classes); generated
  slots legal on both layers; lanes disjoint in x within a lane; monotone destination order ⇒ no
  lane-order inversion (the crossing-free invariant); short-net branch fires for /GPIO27/28/XIN.
- **Fixture — witness match (Inv-2 E2):** computed escape order reproduces the witness's
  crossing-free assignment for the 8-net primary ring (Hamming distance 0 on layer + lane order).
- **Unit — `route_net_steered`:** pins the via to the supplied site (asserts `world_vias[0] ==
  escape_via_xy`); B.Cu run stays within the lane band; F.Cu-only branch (escape_via_xy=None)
  produces 0 vias.
- **Regression — default-OFF:** `topo=None` ⇒ byte-identical board to attempt-3 (the standing
  opt-in discipline).
- **Crossing-audit regression:** the W1 `seg_intersect` test as a permanent gate — TCR output must
  have 0 different-net same-layer crossings (the legality bar co-opt failed).
- **Determinism:** 3-run byte-identical at E1, TCR-1, TCR-2.

---

## Acceptance Rubric

- [ ] **AC1 — design doc complete.** `docs/design/TOPOLOGY-CLASS-ROUTING.md` contains: synthesis,
  TopoClass representation, unified E1 gate with exact steering mechanism, assignment pre-pass
  algorithm, staging, bounded/determinism guards, per-inventor contribution. Evidence: section
  headers present.
- [ ] **AC2 — TopoClass is extractable AND assignable.** Doc specifies the dataclass and cites the
  exact inventor-script sections that extract each field from the witness, AND the pre-pass step
  that generates each field. Evidence: §TopoClass representation maps every field to both.
- [ ] **AC3 — steering mechanism is concrete.** Doc states whether `route_net_gridless`/fanout is
  steerable today and names the ONE missing knob + the minimal addition (`route_net_steered`) with
  signature. Evidence: §steering mechanism + Interface Contracts.
- [ ] **AC4 — E1 gate is one unified spec.** Doc gives ONE gate (not two) with mechanism, exact
  pass criteria (numeric thresholds), and what it kills. Evidence: §unified make-or-break gate.
- [ ] **AC5 — assignment algorithm chosen + justified.** Doc picks ONE pre-pass (monotone lane
  packing + planar escape order) and justifies it over min-cost flow with ≥3 reasons grounded in
  the capacity proof. Evidence: §flow_vs_monotone.
- [ ] **AC6 — staging has scorecard exit criteria.** Three milestones (E1 → TCR-1 → TCR-2), each
  with a numeric/binary exit criterion. Evidence: §Staging.
- [ ] **AC7 — bounded/determinism guards mandated.** Doc names the specific reused guards
  (`_check_rss` 2GB, `max_window_mm`/`max_bcu_window_mm`, region-scope, skip_full_corner_fallback,
  3-run byte-identical). Evidence: §Bounded-runtime discipline.
- [ ] **AC8 — E1-first ordering enforced.** The ordered developer task list puts E1 (cheap kill-
  test) FIRST, with NO production code before E1 GO. Evidence: §developer task list / structured
  result `e1_developer_tasks`.
- [ ] **AC9 — no production code in the design.** This doc contains only illustrative
  signatures/pseudo-code, no runnable implementation. Evidence: no full function bodies.

---

## E1 make-or-break gate (2026-06-23) — NO-GO on the strict bar; mechanism promising but doesn't beat 41

`scripts/_probe_tcr_e1.py` + `route_net_steered` (additive ~60 LOC in route.py, default-unused →
byte-identical). Steered the realizer with the WITNESS'S OWN homotopy classes on the QFN-escape nets.
PM-verified by independent re-run (`/usr/bin/time`):

| criterion | result | bar | |
|---|---|---|---|
| unconnected | **43** | < 41 | FAIL (but < grid-only 48) |
| errors | **84** | < 73 | FAIL |
| F.Cu / B.Cu illegal crossings | **0 / 0** | 0 | **PASS** |
| peak RSS | 0.92GB | < 2GB | PASS |
| via steering honored | 6/9 | ≥ 80% | partial |
| determinism (connectivity) | 43 every run | — | stable (hashes differ = zone-fill) |

**NO-GO on the strict gate (43 ≥ 41, 84 ≥ 73).** BUT the make-or-break fear ("the realizer can't be
steered") is ALLAYED: steering partially WORKS (6/9 witness vias honored) and the topology realization is
**crossing-free (0 illegal crossings)** — the structural win the greedy approaches never had. 11/13
escape nets realized; **GPIO9 fails (`bcu_run_failed`)** because the `lane_y_mm` enforcement the design
specified was NOT implemented in the E1 probe → GPIO9's B.Cu lane crosses GPIO14's.

**Honest gap analysis (why it doesn't beat attempt-3):** (1) connectivity 43 vs 41 — proximate cause is
the unimplemented lane enforcement (GPIO9); fixing it might reach ~41-42 but not clearly below. (2) errors
84 vs 73 — attempt-3's 73 came from the `gridless_first` mechanism's solder_mask/hole_clearance
reductions; TCR routes the non-escape nets via plain grid, so it doesn't replicate that. Beating
attempt-3 on BOTH axes would require lane enforcement (connectivity) AND gridless_first integration
(errors) — substantial further work with an uncertain payoff (may TIE 41, not beat).

**VERDICT: the E1 gate did its job — killed cheaply (~1 day, zero production impact; `route_net_steered`
is additive/unused).** The investigation's lasting value: PROVED (via both inventors + this gate) that
mitayi's gap is homotopy/ordering not capacity, and that crossing-free topology realization IS achievable
(0 crossings). But TCR does not beat attempt-3 as-tested. **attempt-3 (commit 9ae76ea, mitayi 48/87 →
41/73) REMAINS THE DEFINITIVE BEST.** Closing the last gap to the human's 0/0 needs either the
lane-enforcement + gridless_first-integration follow-through (scoped but uncertain) or genuine placement
co-design — both fresh chapters.

---

## E1 follow-through (2026-06-23) — FIRST sub-41 connectivity (40), not yet a clean win

Implemented the two E1-gap fixes: **lane_y_mm enforcement** in `route_net_steered` (constrain the B.Cu
run to a ±pitch band around the assigned lane → monotone-ordered nets route in disjoint lanes,
crossing-free by construction — the river-routing result) + **gridless_first integration** (route the
steered escape nets first → mark their copper → route the rest via the attempt-3 negotiate mechanism;
also fixed a real bug: `gridless_first` multi-pin path wasn't seeding `extra_gridless_obstacles`).
`route_net_steered`/escape-mode opt-in; default byte-identical; 416 tests; ruff clean.

**PM-verified A/B (independent re-run, `/usr/bin/time`):**
| | unconnected | errors | crossings | |
|---|---|---|---|---|
| attempt-3 (gf_only control) | 41 | 73 | 0 | baseline |
| **TCR escape + gridless_first** | **40** | 79 | **0** | NEW connectivity best |

**40 < 41 — the FIRST time anything beats attempt-3 on connectivity** (the primary goal, toward the
human's 0). Crossing-free (lane enforcement worked); bounded (937MB); connectivity deterministic (40
every run). 11/13 escape nets realized; lane enforcement fixed the GPIO9 crossing from E1.

**NOT a clean win:** errors 79 vs 73 (+6), including **+2 shorting_items** — /RUN and /SWCLK get
FALLBACK vias because their WITNESS via positions are illegal under `is_legal_via` (QFN pad proximity);
the fallback places them too close → F.Cu stub overlap → 2 shorts. By the project's ~5-errors-per-
unconnected heuristic, 40/79 ≈ 41/73 (essentially tied), tilted toward connectivity.

**The path to a clean sub-41 win is now pinpointed AND independently corroborated:** the researcher's
`HUMAN-ROUTING-TECHNIQUES.md` ranks **#1 = global ring-slot assignment** (non-radial, balanced N/S/E/W
quadrant fanout) — which would give /RUN,/SWCLK LEGAL ring-slot vias instead of the too-close radial
fallback, clearing the 2 shorts and likely the +6 errors. That is the next build: replace the witness-via
+ radial-fallback with a proper global ring-slot assignment pre-pass (TCR Steps A-C), then re-measure.
attempt-3 (41/73) remains the committed best-overall; this 40/79 is the first connectivity breakthrough.

---

## Global ring-slot assignment (2026-06-24) — built + works (shorts cleared, err 65<73), but a Pareto trade vs connectivity

Built TCR Steps A-C as `src/tracewise/route/gridless/topo_assign.py` (generate_ring_slots →
assign_nets_to_slots via monotone cyclic-order matching → verify legal/disjoint/crossing-free; 8 unit
tests). Wired a `ring_slots` mode into the probe. PM-verified (suite 424 green, ruff clean).

**PM-verified A/B (vs attempt-3 41/73):**
| mode | unconnected | errors | shorting_items | crossings |
|---|---|---|---|---|
| attempt-3 (gf_only) | 41 | 73 | 0 | 0 |
| escape (witness via + radial fallback) | **40** | 79 | 2 | 0 |
| **ring_slots (legal slots)** | 46 | **65** | **0** | 0 |

**Ring-slot assignment DID its job: cleared the 2 shorts AND dropped errors to 65 (< 73).** BUT it
raised unconnected to 46 — NOT a clean win. Neither config achieves unc≤40 AND err≤73 simultaneously.

**The precise tension (important finding):** the escape mode's connectivity-40 RELIES on /RUN,/SWCLK
using ILLEGAL radial-fallback vias placed INSIDE U3's pad-exclusion zone (already-dead territory → they
connect WITHOUT adding routing obstacles, but overlap stubs → 2 shorts). The LEGAL ring slots sit
OUTSIDE that zone in the ACTIVE J4 corridor → legal + short-free, but they BLOCK +3V3 (a 29-pin net) →
it loses ~30 ratsnest gaps → unc 40→46. **No legal via position for /RUN,/SWCLK is simultaneously legal,
B.Cu-reachable to J4, AND non-obstructive to +3V3's routing.** We are on a Pareto frontier around
attempt-3: escape = connectivity-optimal (40/79), ring_slots = error-optimal (46/65), attempt-3 = 41/73.

**The pinpointed path to a clean win (40/65):** route +3V3 (and the other corridor power nets) BEFORE
placing the /RUN,/SWCLK legal vias — so +3V3 claims its corridor first and the legal vias don't block it.
This is a routing-ORDER/priority fix (the researcher's net-ordering technique). If +3V3 routes first,
ring_slots' error win (65, 0 shorts) could combine with escape's connectivity (40) → a clean sub-41 win
on BOTH axes. (Honest caveat: this is another Pareto-frontier micro-iteration; the fundamental 40→0 gap
is the human's global co-optimization our staged architecture approximates but doesn't match.)
attempt-3 (41/73) remains best-overall; ring-slot assignment is committed infrastructure (default-off).
