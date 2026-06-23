# Invention: Topology-First "Escape-Ring + Layer-Direction Highway" Router (human-mimicry)

Status: INVENTION PROPOSAL, 2026-06-23 (INVENTOR-1). Read-only reverse-engineering of the human
witness + a constructive routing procedure that mimics it. NO production code modified. The
throwaway analysis is `scripts/_invent1_human_stats.py` (every number below is real, parsed from
the shipped board).

> **One-line thesis.** The human does not *pathfind*; the human *assigns a global topology* — a
> via slot in an escape ring per QFN net, a layer per direction, a side per obstacle — and only
> then realizes it geometrically. Every TraceWise approach to date discovers that topology
> locally and piecemeal, so the nets fight for ring slots and B.Cu channels (ceiling 41/73). The
> invention is to **assign the topology FIRST, globally, as a combinatorial step, then realize it
> with the existing exact-geometry machinery.**

---

## Problem Statement

**One-sentence problem (technology-free):** Connect every signal between a dense component and two
connector rows across two stacked routing planes, when the per-signal corridors all compete for the
same narrow band of space, such that no two signals' copper overlap.

**Hard constraints:**
1. 2 copper layers only (F.Cu, B.Cu) — the human witness proves this is sufficient.
2. Track 0.2mm, clearance 0.15mm, via 0.4mm/0.2mm-drill (project values; same as human — geometry
   is NOT the gap, measured).
3. QFN (U3) pad pitch 0.4mm, pad gap 0.2mm — physically impossible to place a via *between* QFN
   pads; signals MUST escape outward to a ring.
4. Determinism: byte-identical routes run-to-run on a fixed install (the project's standing rule).
5. Bounded memory/runtime (the 4-18GB blowups from board-wide visibility graphs must not recur).

**Soft constraints:** match human via count (~111) order-of-magnitude; runtime within ~2x grid
`--quality`; reuse existing exact-geometry/emit/DRC machinery rather than a parallel engine.

**Success criteria:** mitayi HUMAN placement **unconnected < 41 AND errors ≤ 73** (beat the
attempt-3 ceiling on both axes); stretch toward human 0/0. Measured on the canonical scorecard.

---

## Landscape Analysis (what we already tried, why each ceilings at 41)

| Existing approach (all in-repo, all measured) | Strength | Limitation vs the constraints |
|---|---|---|
| Grid A* + bounded rip-up + negotiated congestion | Robust, deterministic, bounded | LOCAL greedy; 0.1mm grid over-estimates congestion; 48/87 |
| Gridless visibility-graph + per-net locality (FAR) | Exact geometry, legal-by-construction | Per-net windows route one net at a time; no global plan; contends |
| QFN fanout-escape (F.Cu stub → **radial** via → B.Cu) | Scales: 10/17 QFN nets connect | Via placed on radial ray through src pad → greedy B.Cu runs DISPLACE 7 grid nets → 45 |
| Grid-first fanout-rescue | Zero grid displacement | Grid copper WALLS OFF the escapes → 1/17 → 46 |
| Cross-substrate co-opt (shared 0.5mm congestion field) | Deconflicts at REGION scale (5/5, bounded 363MB) | Full board OVER-CONSTRAINS grid → 63 DRC-honest; worse than grid-only |

**Gap summary.** Every approach is a *search* (grid A*, visibility-graph A*, negotiated re-pricing)
that *discovers* the escape ring and the B.Cu highways one net at a time. Because the ring slots
and the horizontal B.Cu channels are a SHARED, near-saturated resource, sequential discovery makes
nets collide; negotiation only re-prices the collision, it does not *prevent* it by design. **None
of them assigns the global structure as a first-class combinatorial decision.** That is the
missing primitive — and it is exactly what the human board encodes.

---

## The reverse-engineered human DECISION PROCEDURE (measured, every claim cited)

All numbers from `scripts/_invent1_human_stats.py` on the shipped 0/0 board. U3 (RP2040 QFN)
center = (148.87, 88.98), pad-ring radius 3.44mm. Connectors: J3 (north pin row, y≈97.4), J4
(south pin row, y≈81.2), both x∈[74.5, 122.8] — WEST of U3. J5/J1 east of U3.

### Rule H1 — Escape ring: every dense-component net gets a via in a balanced ring just outside the pad ring.
- **39 vias** sit in a tight band **4.55–8.18mm** from U3 center (mean **6.03mm**), i.e. ~1.1–4.7mm
  outside the 3.44mm pad ring. **24 distinct nets.**
- The ring is **balanced across all four QFN edges**: W:10, S:10, E:10, N:9. Not ad-hoc — a
  deliberate, evenly-loaded escape ring.
- Interpretation: the human reserves a *ring of via slots* and assigns each escaping net one slot.

### Rule H2 — Escape via is placed at a FREE ring slot, NOT radially out from the source pad.
- For all 9 measured boxed nets, the via's QFN edge ≠ the source pad's edge (e.g. /GPIO3 src on the
  S/east region escapes to a via on the N side; /SWCLK src on the W edge escapes to a via on the E
  edge). The **F.Cu stub navigates AROUND the pad ring** (stub length **9–14mm**) to reach the
  assigned slot.
- This is the single biggest deviation from TraceWise's fanout-escape, which forces the via onto
  the radial ray through the source pad. The human treats ring-slot assignment as a *global packing*
  problem, not a per-pad radial projection.

### Rule H3 — Layer = direction. B.Cu is the horizontal highway layer; F.Cu carries vertical/local.
- Signal-only, length-weighted (excludes GND/+3V3 pours): of all **horizontal** copper, **67% is on
  B.Cu**; of all **vertical** copper, **80% is on F.Cu**. (Board-wide incl. power: H 61% B.Cu, V 80%
  F.Cu — same convention.)
- The boxed nets are **75–89% B.Cu by length** (e.g. /GPIO3 89%, /SWCLK 89%, /RUN 83%): a short F.Cu
  stub + via + a long B.Cu horizontal run. /GPIO3's B.Cu run is a single 13.36mm horizontal segment;
  /SWCLK's is 14.11mm horizontal.

### Rule H4 — B.Cu highways span the U3-ring → connector channel; the connector rows are the bus terminus.
- J3/J4 are vertical pin rows west of U3 (GPIO0..18 on J3, GPIO19..29 on J4, monotonic in y). The
  long horizontal B.Cu runs carry each escaped signal from its ring via westward to the connector
  pad's y-coordinate, then a short final leg into the pad. This is a **bus**: parallel horizontal
  B.Cu tracks stacked in the west channel.

### Rule H5 — Short nets stay single-layer F.Cu (no via).
- 3/13 hard nets (/GPIO27, /GPIO28, /XIN) are **F.Cu-only**, 0 vias, 6–9mm total — the human only
  pays the via + layer-change cost when the straight F.Cu path is blocked by the pad forest.

**Synthesis — the human procedure as a constructive plan (NOT a search):**
1. For each QFN-escaping net, reserve a via slot in the escape ring (balanced across edges).
2. Route the F.Cu stub from the pad, *around the pad ring*, to its assigned slot.
3. Drop a via; carry the signal as a horizontal B.Cu highway to the destination's y-band.
4. Final F.Cu (or B.Cu) leg into the connector pad. Short nets skip the via (Rule H5).
The human *commits the topology* (which slot, which layer, which channel track) before drawing a
single segment. The drawing is then a constraint-satisfaction realization, not a contention race.

---

## Solution Design

### Core idea
**Separate routing into two phases the way the human's mind does: (1) a global TOPOLOGY ASSIGNMENT
that is a small combinatorial optimization — assign each escaping net a ring via-slot and a B.Cu
highway track-slot, packed so no two collide; (2) a GEOMETRIC REALIZATION that uses the EXISTING
exact-geometry machinery to draw each net along its assigned topology, which by construction cannot
contend because the slots were packed disjoint in phase 1.** This is "global routing then detailed
routing" (the classical VLSI two-stage decomposition) but specialized to the escape-ring + layer-
direction structure the human witness reveals — which is what makes the global stage *small and
solvable* rather than a general NP-hard global router.

### Interface contract (the new primitive)

```python
# topology.py — the NEW combinatorial primitive (the invention's core)

@dataclass(frozen=True)
class RingSlot:
    edge: str          # 'N'|'S'|'E'|'W' — which QFN edge
    angle_deg: float   # slot center angle from component centroid
    radius_mm: float   # in the escape band (e.g. 6.0)
    xy: tuple[float, float]   # the legal via site (snapped 1nm), 3-predicate-checked

@dataclass(frozen=True)
class HighwayTrack:
    layer: int         # B.Cu for horizontal bus, per Rule H3
    coord_mm: float    # the y (for horizontal) the track occupies — its lane
    span: tuple[float, float]   # x-range the lane reserves

@dataclass(frozen=True)
class NetTopology:
    net: str
    ring_slot: RingSlot | None      # None for F.Cu-only short nets (Rule H5)
    highway: HighwayTrack | None
    dest_xy: tuple[float, float]

def assign_topology(
    escaping_nets: list[Net],          # nets exiting the dense component
    component: DenseComponent,         # centroid, pad-ring radius, per-edge pad order
    connectors: list[ConnectorRow],    # J3/J4/J5 pad rows + y-coords
    geo: dict,                         # track/clearance/via — for pitch + ring radius
    free_space_F, free_space_B,        # Shapely free space per layer (for legal-slot check)
) -> dict[str, NetTopology]: ...
    # The combinatorial core. Deterministic. Returns a COLLISION-FREE assignment or a
    # best-effort partial (nets it could not seat are flagged, routed by fallback).
```

Realization reuses the existing per-net gridless entry, *constrained* to the assigned topology:

```python
# realize_topology.py — thin wrapper over the EXISTING gridless machinery
def realize_net(top: NetTopology, net, free_space_F, free_space_B, geo) -> GridlessNetRoute:
    # 1. F.Cu stub: visibility-graph A* (build_visibility_graph, use_all_rings=True for QFN pad)
    #    from src pad -> top.ring_slot.xy, BOUNDED window = pad-to-slot bbox + margin.
    # 2. via at top.ring_slot.xy (existing 3-predicate via check).
    # 3. B.Cu highway: a near-straight run constrained to lane top.highway.coord_mm
    #    (visibility-graph A* on B.Cu, but the search is trivial — the lane is reserved free).
    # 4. final leg into dest. Returns the existing GridlessNetRoute (IS-A NetRoute) -> emit reuses.
```

### Key mechanism — the topology assignment (phase 1), the genuinely new part

This is a **bipartite/lane-packing assignment**, NOT a search over geometry:

**(a) Ring-slot generation.** Generate candidate via slots on K evenly-spaced angles per QFN edge in
the escape band [pad_ring_r + via/2 + clr + margin, +band_width] (≈4.5–8mm). Keep only slots whose
3-predicate via test passes on both layers (legal-by-construction). Measured human: 39 slots, ~10
per edge — so K≈10/edge is the right density.

**(b) Lane generation.** The west channel between J4 (y≈81.2) and J3 (y≈97.4) is 16.2mm tall. At
pitch = track+clearance = 0.35mm it holds **⌊16.2/0.35⌋ = 46 horizontal B.Cu lanes**. Each lane is a
reserved y-band for one signal's B.Cu highway.

**(c) Assignment as min-cost matching / lane packing.** Each escaping net must be matched to (ring
slot, lane) such that: its lane's y matches its destination connector pad's y (lanes near the
destination cost less — this is what keeps runs short and non-crossing); two nets in the same lane
must not overlap in x (a 1-D interval-packing per lane); the ring slot should be on the QFN edge
facing the net's escape direction to keep the F.Cu stub short. Solve as: sort nets by destination y
(the connector order is already monotonic — GPIO0..29), assign lanes in destination-y order (a
greedy that is provably crossing-free for a monotone bus, below), and assign each net the nearest
*free, legal* ring slot on the appropriate edge. Deterministic (fixed net order + fixed slot order);
no float-driven control flow.

**(d) Crossing-free guarantee for the bus.** If nets are assigned lanes in the SAME order as their
destination pads appear along the connector row (both monotone in y), and each net's escape via x is
ordered consistently, the horizontal runs form a *planar* bus — no two B.Cu highways cross (a
standard river-routing result: a monotone permutation routes without crossings in a channel of
sufficient height). The human's J3/J4 GPIO ordering IS monotone, which is why the human bus is
crossing-free.

### Trade-off analysis

| Dimension | This invention (topology-first) | Best existing alternative (cross-substrate co-opt) |
|---|---|---|
| Contention handling | PREVENTS collisions by disjoint slot/lane packing (phase 1) | DETECTS + re-prices collisions (negotiation) — over-constrains at full board |
| Global structure | Explicit (ring + lanes assigned up front) | Emergent from pricing (never globally consistent) |
| Runtime | Phase-1 packing is O(nets·slots) tiny; phase-2 realizations are bounded windows | Multi-round joint loop; convergence risk |
| Determinism | Pure combinatorial assignment + bounded realization — easy | Achievable but multi-round surface is larger |
| Reuses existing machinery | YES — phase 2 = existing gridless A* + emit + DRC | YES — but adds a whole unified loop |
| Risk | Phase-1 model may not capture all real obstacles (pours, drill holes) — realization can still fail a seated net | Proven bounded; proven NOT to beat 41 at full board |
| Novelty | The topology-assignment primitive does not exist in the repo | Co-opt already built, already NO-GO at full board |

---

## Prototype

**Location:** `scripts/_invent1_human_stats.py` (the reverse-engineering analysis — RUNS, all
numbers above are its output). A phase-1 *assignment* prototype is specified below but NOT yet
built as runnable code in this proposal (see Assessment — this is honest scoping: the decisive
prototype is the witness analysis, which already proves the structure is real, balanced, and
capacity-feasible; the assignment algorithm is the build, gated on the validation experiment).

**Demonstrates (runnable, verified):**
- The escape ring is real, balanced (W10/S10/E10/N9), tight (4.55–8.18mm), 24 nets — Rule H1/H2.
- Layer=direction convention quantified (B.Cu H 67%, F.Cu V 80%) — Rule H3.
- The west channel capacity (46 lanes) vs demand (34 signals) = ratio **0.74 < 1** — the feasibility
  headroom that makes a packed assignment possible (below).

**Omits (what the production build must add):** the `assign_topology` solver itself; the
`realize_net` wrapper around the existing gridless entry; pour/drill obstacles in the legal-slot
test; the F.Cu-only short-net branch (Rule H5).

**How to run:** `taskset -c 0-9 .venv/bin/python scripts/_invent1_human_stats.py`

---

## Feasibility argument (the math — honest, not a 0/0 guarantee)

**Claim (capacity feasibility, the constructive lower bound).** A topology-first router CAN seat the
escaping signals iff (i) the escape ring has ≥ N legal via slots and (ii) the highway channel has ≥ N
disjoint lanes, where N = number of escaping signals. The human witness proves both hold on mitayi:

- **Ring capacity:** the escape band has room for ~10 slots/edge × 4 edges = 40 (human used 39 for
  24 nets). N_escaping ≈ 24–34 ≤ 40. **Slots are not the binding constraint.**
- **Channel capacity:** west channel height 16.19mm / pitch 0.35mm = **46 horizontal B.Cu lanes**.
  Signal demand U3↔connector (excl. power) = **34 nets**. Demand/capacity = **34/46 = 0.74 < 1.**
  There is 26% headroom. **The bus is feasible with margin — this is precisely why the human reaches
  0/0 and why a lane-packed router can too.**

**Crossing-free guarantee (the planarity argument).** If escaping nets are assigned lanes in the
order their destinations appear along the connector row (monotone permutation), and ring-via x-order
is made consistent, the horizontal B.Cu runs form a planar river-routed bus — no two highways
cross. This is a classical channel-routing result (a monotone net ordering routes in a channel
without crossings). The human's monotone J3/J4 GPIO numbering realizes exactly this. Therefore a
router that *enforces* the monotone lane assignment inherits the crossing-free property *by
construction*, eliminating the tracks_crossing/shorting DRC class that sank co-opt (28 crossings,
27 shorts at full board).

**Honest limits of the argument (what it does NOT prove):**
- It proves *capacity and crossing-freedom for the U3↔connector bus*, the dominant contended
  resource. It does NOT prove every one of the 64 nets routes — power pours, the USB differential
  pair, crystal nets, and non-QFN nets have their own constraints not modeled here.
- The 0.74 ratio assumes the full 16.2mm channel is usable for horizontal B.Cu; real pours/keepouts
  reduce it. The true usable capacity must be measured in the validation experiment, not assumed.
- "Can seat" (phase-1 assignment exists) does NOT guarantee "realizes legally" (phase-2 geometry
  may still fail a seated net if the lane band is locally blocked). The realization re-checks every
  segment with `exact_geom`; a seated-but-unrealizable net falls back to the existing router.

So: **feasibility is argued with real capacity numbers and a planarity result, NOT asserted as a
0/0 guarantee.** The claim is "the binding resource (the bus) has 26% headroom and admits a
crossing-free packing, which is the necessary condition the prior approaches violated by routing
greedily."

---

## What is NOVEL vs the exhausted approaches

1. **Topology assignment is a first-class combinatorial step, executed BEFORE any geometry.** Every
   prior approach (grid A*, gridless A*, co-opt negotiation) *searches* and discovers the ring/bus
   structure incidentally and per-net. None *assigns* it globally up front. This is the new primitive.
2. **Collisions are PREVENTED by disjoint packing, not DETECTED+re-priced.** Co-opt's shared field
   reacts to contention; this packs slots/lanes disjoint so contention cannot arise in the bus. That
   is why it can avoid the 28-crossing/27-short regression that DRC-honestly sank co-opt to 63.
3. **The layer=direction convention is ENFORCED as a constraint, not emergent.** B.Cu is *declared*
   the horizontal highway layer (measured 67% in the witness); prior routers let the layer fall out
   of A* via-cost, producing the channel contention.
4. **Ring slot ≠ radial projection (Rule H2).** Prior fanout-escape forced the via radially out from
   the source pad; the witness shows the human packs slots and routes the F.Cu stub *around* the ring
   to a free slot. Modeling slot assignment as packing (not projection) is new and is what lets the
   ring stay balanced (W10/S10/E10/N9) instead of crowding the edge facing the connectors.

It REUSES (not reinvents): the exact-geometry visibility-graph A*, the `all_rings` corner mode (built
for QFN pads), the 3-predicate via check, the `GridlessNetRoute`→emit→DRC pipeline. The invention is
the *planning layer above* them.

---

## Smallest experiment to validate the core idea (do this BEFORE any build)

**Probe-Topology (1–2 days, standalone script, no engine changes).** Validate the load-bearing
assumption — that a packed lane assignment is realizable, not just combinatorially feasible:

1. Take the **34 U3↔connector signal nets** on mitayi (the bus).
2. Run phase-1 `assign_topology` *by hand/script*: assign each net a lane (destination-y order) and a
   ring slot (nearest free legal slot on the facing edge). Print the assignment; assert no two lanes
   overlap in their reserved x-span and no two ring slots coincide.
3. Realize **just the B.Cu highway legs** for these 34 nets using the EXISTING gridless B.Cu A*
   constrained to each assigned lane band (bounded windows), into a stripped mitayi temp board.
4. `refill_zones` + `run_drc`. **Pass iff: ≥30/34 highway legs realize all-legal (0 new
   tracks_crossing/shorting), deterministic (3-run byte-identical), bounded (<2GB, <5min).**

**Decision:** if ≥30/34 bus legs route crossing-free under the packed assignment → the topology-first
premise holds; build the full `assign_topology` + `realize_net` + the F.Cu-stub/via legs and measure
the full scorecard vs 41/73. If the packed assignment realizes <30/34 (lanes locally blocked by
pours/holes the model ignored) → the capacity model is too optimistic; record WHY (which lanes
blocked) and the invention reduces to "a better fanout-escape ordering," not a new architecture
(→ DO NOT RECOMMEND escalation; fall back to attempt-3).

This probe is decisive and cheap because it tests the *exact* failure mode (bus crossings/shorts)
that DRC-honestly sank co-opt, using only the existing realization machinery.

---

## Assessment (Inventor Phase 5 — Self-Assessment Gate)

**Verdict: RECOMMEND WITH CAVEATS — but gated strictly on Probe-Topology.**

**Is a NEW custom approach genuinely warranted, or is this "just tune what we have"?** Honestly: it
is **~70% recombination, ~30% genuinely new.** The realization machinery (gridless A*, via check,
fanout-escape, emit/DRC) all exists and is reused unchanged. What is genuinely new and absent from
the repo is the **global topology-assignment primitive** (`assign_topology`): a combinatorial
lane/slot packing executed before geometry, with an enforced layer=direction convention and a
crossing-free monotone-bus guarantee. That primitive is NOT a tuning of the existing routers — it is
a planning layer none of them has, and it directly attacks the one failure all of them share
(greedy per-net contention for the bus). So it clears the "new thing must exist" bar — but only if
Probe-Topology confirms the packed assignment is *realizable*, not merely *feasible on paper*. If the
probe fails, this collapses to "a smarter fanout ordering" = tuning, and should NOT be built.

**Maintenance burden: Medium.** `assign_topology` is ~200–400 LOC of combinatorial logic (slot gen,
lane gen, monotone matching) — self-contained, deterministic, testable in isolation against the
witness numbers. It adds a planning concept ("topology") that a new maintainer must learn, but it is
simpler to reason about than the multi-round co-opt negotiation loop it would partially replace.

**Adoption path:** opt-in flag (`topology_first=True`) exactly like `coopt`/`gridless_first`, default
OFF = byte-identical. The witness analysis script + this doc + the planarity argument are the
teaching material. Phase-2 reuses the contracts developers already know.

**Escape hatch:** if `assign_topology` cannot seat a net or its realization fails, that net falls
through to the EXISTING router (grid A* or gridless). The invention is strictly additive — worst case
it is a no-op on a net and the board scores no worse than attempt-3 on that net. If the whole
approach underperforms, delete the planning layer; the realization machinery is untouched.

**Why not just keep co-opt?** Co-opt is proven NO-GO at full board (63 DRC-honest). It detects
contention; it cannot prevent the bus crossings. This invention's planarity-by-construction is the
specific thing co-opt lacks. That is the justification for a new primitive over tuning co-opt.

---

## Handoff to Developer (if Probe-Topology passes)

**Files to create:**
- `scripts/_probe_topology_bus.py` — Probe-Topology (the validation experiment above). Build FIRST.
- `src/tracewise/route/topology/assign.py` — `assign_topology` (slot gen, lane gen, monotone
  matching). The new primitive.
- `src/tracewise/route/topology/realize.py` — `realize_net` thin wrapper over the existing gridless
  entry, constrained to the assigned `NetTopology`.
- `tests/test_topology_assign.py` — assignment is deterministic; lanes disjoint; ring slots legal;
  monotone destinations → crossing-free (assert against the witness numbers as a fixture).

**Files to modify (only after probe + tests green):** `multi.py` `route_all` add `topology_first`
opt-in path; `kicad.py` thread the flag. Default OFF byte-identical (the project's standing
opt-in discipline).

**Implementation notes / gotchas (from the reverse-engineering):**
- Pad `at` is in the footprint LOCAL frame (pre-rotation); convert to global with footprint origin +
  rotation BEFORE any ring/lane math (see `extract_pads_global` in `_invent1_human_stats.py`). This
  bit me — the first run had pads 175mm off because I used the probe's local-frame extractor.
- Ring slots must pass the 3-predicate via test on BOTH layers and be reflex/all_rings-visible from
  the source pad (the `all_rings` mode is REQUIRED for QFN source pads — already in the gridless pkg).
- Lane assignment MUST be monotone in destination y to get the crossing-free guarantee; do not sort
  lanes by source.
- Model pours + drill holes in the lane free-space, or the probe's 0.74 capacity headroom is
  optimistic.

**Test strategy:** unit-test `assign_topology` against the witness (24 nets, balanced ring,
crossing-free bus). Probe-Topology is the integration gate. Scorecard (`_probe_route_human.py` /
`_verify_gridless_first_ab.py`) vs 41/73 is the acceptance measure. Determinism: 3-run byte-identical.

**Documentation needs:** the topology concept (ring slots + lanes + layer=direction), the
crossing-free monotone-bus invariant, and the opt-in flag.

---

## Risks

| Risk | Likelihood | Impact | Mitigation |
|---|---|---|---|
| Packed assignment feasible on paper but lanes locally blocked by pours/holes → <30/34 realize | Med | High | Probe-Topology tests exactly this BEFORE any build; model pours/holes in lane free-space; per-net fallback to existing router |
| 0.74 capacity ratio optimistic (full 16.2mm channel not usable) | Med | Med | Probe measures real usable lanes; if ratio→>1 the invention cannot beat the human and degrades to fallback |
| Non-bus nets (USB pair, crystal, power) not modeled | Med | Med | Topology layer only owns the QFN↔connector bus; everything else routes on the existing engine unchanged |
| Determinism in the matching | Low | High | Pure combinatorial assignment over fixed net/slot order; no float control flow; 3-run gate |
| It is "just tuning" (NIH) | Med | Med | Phase-5 gate + Probe-Topology: if the packed bus does not realize crossing-free, DO NOT build — fall back to attempt-3 |
