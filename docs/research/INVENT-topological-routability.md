# INVENT — Topological / Constructive Routability for mitayi (INVENTOR-2)

Status: invention proposal (2026-06-23). Read-only analysis + math framing. NO production
code touched. Grounds every claim in real data extracted from the human witness board
(`scripts/_invent2_topology.py`, reusing `scripts/probe_human_routing.py` parsing).

Mission: exploit that the human's 0/0 routing is an EXISTENCE WITNESS — mitayi IS provably
2-layer-routable on this placement. Frame routing as a GLOBAL TOPOLOGICAL problem, not
per-net greedy pathfinding. Decide honestly whether a new approach is warranted.

---

## TL;DR (the honest verdict up front)

- **The contention our routers hit is NOT a capacity / min-cut wall. It is an ORDERING +
  HOMOTOPY artifact.** Measured from the witness: the tightest plausible bottleneck — the
  QFN radial escape cut around U3 — runs at **~50-56% utilization** (R=4mm F.Cu: 40 tracks
  of ~71 capacity; R=5mm: 45 of ~89). B.Cu is nearly empty at every ring. Every channel the
  human used has roughly **2x spare capacity**. There is no global cut violation, exactly as
  the witness guarantees.
- **The witness is provably legal: 0 different-net track crossings on F.Cu (426 segs) and 0
  on B.Cu (157 segs).** Contrast: the project's own best result ("41/136") was documented as
  *partly illusory — illegal grid crossings counted as connections* (CROSS-SUBSTRATE-COOPT.md
  2026-06-23). So our best number is inflated by DRC-illegal crossings; the human's is real.
- **Therefore the gap is precisely the one topological routing is designed to close:** our
  greedy, sequential, per-net pathfinding commits to a homotopy class (which side of each
  obstacle, which layer) net-by-net and gets trapped; the witness occupies a *globally
  consistent* homotopy class that no greedy ordering reaches. The repo's own Probe-Order
  already proved this ("ORDERING problem, NOT placement-bound; all 17 route on a clean board")
  but its fix (gridless-first) regressed errors +72 because it had no mechanism to keep the
  set in ONE mutually-legal topology class.
- **Invented approach: Topology-Class Routing via planarity-ordered escape + corridor
  assignment ("TCR").** Not a full gEDA-style toporouter. A *scoped* constructive method:
  (1) fix the QFN escape order by ANGULAR (planar) sort so the radial escape is crossing-free
  by construction; (2) assign each escaped net a homotopy class = (layer, ordered corridor
  side-sequence) via a global min-cost assignment, NOT greedy A*; (3) realize geometry by
  rubber-band/funnel inside the assigned corridor. The capacity proof guarantees a feasible
  assignment EXISTS.
- **Phase-5 honesty: a NEW *full* topological router is NOT warranted** (gEDA toporouter's
  documented poor dense-2-layer results + 3-subsystem complexity; the survey already rejected
  it). **But the CORE TOPOLOGICAL INSIGHT — escape ordering + global homotopy-class assignment
  as a PRE-PASS feeding the existing engine — IS warranted and is genuinely novel vs every
  exhausted greedy approach.** It is recombination of one new idea (planar escape order +
  global class assignment) with the existing realization machinery, not a from-scratch
  toporouter. Smallest experiment is cheap (~1-2 days) and directly falsifiable.

---

## Problem Statement

**One-sentence problem (technology-free):** Connect a fixed set of pin-pairs on a fixed
placement using two stacked conductive sheets joined by vertical links, such that no two
different connections' paths on the same sheet cross, given that a crossing-free assignment
is known to exist.

**Hard constraints:**
1. Same-sheet (same-layer) paths of different nets must not cross (DRC short).
2. Track + clearance halo geometry: 0.2mm track, 0.15mm clearance => 0.35mm effective pitch.
3. Layer transitions only via 0.4mm/0.2mm-drill vias, placeable only OUTSIDE the QFN pad
   field (gap between adjacent QFN pads = 0.2mm < any compliant via).
4. Two signal layers only (F.Cu, B.Cu). Limited topological freedom.
5. Must run in Python; reuse the existing nets/pads model, ledger, scorecard, DRC harness.

**Soft constraints:** short total length; deterministic; bounded runtime; fits the existing
congestion-pricing currency (super-cell history) where possible.

**Success criteria:** reproduce the witness topology classes geometrically and achieve
unconnected and errors BOTH at-or-below the current best (41 unc / 73 err) — ideally toward
the human's 0/0 — WITHOUT the illegal-crossing inflation that contaminated prior "wins."

---

## Witness data (real, extracted — the foundation)

All from `data/benchmark-boards/mitayi-pico-d1/Mitayi-Pico-D1.kicad_pcb` via
`scripts/_invent2_topology.py`. Board: 64 nets, 583 segments, 111 vias, U3 (RP2040 QFN)
center (148.87, 88.98).

### W1 — The witness is crossing-free (legality is real, ours is not)

| Layer | Segments | Different-net crossings |
|-------|----------|-------------------------|
| F.Cu  | 426      | **0** |
| B.Cu  | 157      | **0** |

The human routing occupies a *planar-per-layer* embedding. Our best result is on record as
inflated by illegal crossings counted as connections. **This is the central asymmetry: we are
not even comparing like-for-like.** A topological method is legal-by-construction on each
layer — it cannot produce the illegal crossings that contaminate our scorecard.

### W2 — Capacity is NOT the bottleneck (the witness disproves a min-cut)

QFN radial escape cut — every U3 signal must cross some circle of radius R around U3. This is
the *tightest* cut on the board (the famous QFN-escape congestion). Capacity = circumference /
0.35mm effective pitch.

| R (mm) | F.Cu tracks crossing | B.Cu tracks | circumference | capacity (tracks) | F.Cu utilization |
|--------|----------------------|-------------|---------------|-------------------|------------------|
| 2.5 | 4  | 1  | 15.7mm | ~44  | 9%  |
| 3.0 | 11 | 2  | 18.8mm | ~53  | 21% |
| 3.5 | 24 | 6  | 22.0mm | ~62  | 39% |
| 4.0 | 40 | 6  | 25.1mm | ~71  | **56%** |
| 5.0 | 45 | 14 | 31.4mm | ~89  | **51%** |
| 6.0 | 36 | 19 | 37.7mm | ~107 | 34% |

18 distinct nets have a segment originating within 4mm of U3 (the QFN escape set). Peak F.Cu
utilization is ~56%; B.Cu is < 22% everywhere. **The channel has ~2x headroom.** The
contention our routers report (channels near J3/J4, the QFN escape) cannot be a real capacity
limit — there is no saturated cut. It is an artifact of the ORDER in which greedy routing
claims the channel and the homotopy CLASS it locks in.

Inter-component cut (U3 <-> J3 header) corroborates: B.Cu carries 11 tracks across a 14.5mm
y-span, occupying ~3.9mm of copper+halo => **27% utilization.** Spare everywhere.

### W3 — The escape topology is a deliberate two-stage radial pattern

Failing-net vias (the 12 layer transitions we cannot reproduce) cluster in a tight ring:

| Net | via radius from U3 |
|-----|--------------------|
| /RUN, /SWCLK, /GPIO3, /GPIO4, /GPIO23, /GPIO6, /GPIO14, /GPIO20 | **5.2 – 6.7mm** (8 vias, the primary escape ring) |
| /GPIO14 (2nd), /GPIO9 | 8.2, 10.8mm |
| /RUN (2nd), /USB_D+ | 18.0, 25.1mm (far transitions near connectors) |

The human escapes radially on F.Cu out to a ~5-7mm ring, drops to B.Cu via a via on that
ring, then runs B.Cu to the (through-hole) connector. The ANGULAR position of each via on the
ring is exactly the homotopy choice — the order around U3 in which each net leaves the field.

### W4 — Homotopy sketch per failing net (the topology classes)

`layers` = collapsed layer sequence along the path; vias = transitions.

| Net | layer seq | vias | len mm | class |
|-----|-----------|------|--------|-------|
| /GPIO27, /GPIO28, /XIN | F | 0 | 6-9 | single-layer (F.Cu only) |
| /GPIO3,4,6,20,23, /SWCLK | FB | 1 | 13-21 | one escape via, F-then-B |
| /GPIO9, /USB_D+ | FB | 1 | 22-41 | long F + escape via |
| /GPIO14, /RUN | FB | 2 | 29-48 | two transitions |

3/13 failing nets are F.Cu single-layer (no via at all). 10/13 use exactly the FB escape
pattern. The human topology is SIMPLE and REGULAR — a layer-1 radial escape ordered around
U3, a via ring, a layer-2 fan to connectors. This regularity is what a global assignment can
capture and greedy per-net A* destroys.

---

## Landscape Analysis (what's been tried; what exists)

| Approach (in repo) | What it does | Why it ceilings at ~41 |
|--------------------|--------------|------------------------|
| Grid A* + rip-up + negotiated congestion (PathFinder) | per-net shortest path, history pricing, rip-up | greedy per-net; commits homotopy net-by-net; grid over-estimates congestion; "41" inflated by illegal crossings |
| Gridless visibility/expansion + locality + congestion | exact-geometry per-net | still sequential; same homotopy-commit trap (Probe-Order: 4/17 when routed sequentially) |
| Vias / multi-pin MST / QFN fanout-escape | escape mechanics | mechanics exist but routed greedily; at full scale fanout connects only 8-10 of 17 |
| Cross-substrate shared-congestion co-opt | joint pricing over a region | OVER-CONSTRAINS the grid at full scale (63 unc DRC-honest); region-valid, board-invalid |
| Gridless-first ordering (attempt 3 = best, 41/73) | route the boxed-in set first, negotiate | best number, but +72 errors when DRC-honest; no mechanism to keep the set in ONE legal topology class |

| External approach | Strength | Limitation vs our constraints |
|-------------------|----------|-------------------------------|
| gEDA Toporouter (CDT + rubber-band, C/GPL) | highest quality ceiling, legal-by-construction | "poor results on dense 2-layer boards with tight constraints" [survey ref 4]; 3 independent geometry subsystems; no Python impl |
| Freerouting expansion rooms (Java) | survey's TOP PICK for FAR | 3-4 month build; addresses substrate, not the ordering/homotopy insight directly |
| TopoR / Altium Situs (proprietary) | production topological routers | unavailable; confirm the family works on real dense boards |

**Gap summary.** Every exhausted approach is GREEDY/SEQUENTIAL in homotopy commitment, OR (the
co-opt) over-constrains by pre-routing a region. None computes a GLOBALLY-CONSISTENT homotopy
class for the contended set and then realizes it. The survey rejected *full* topological
routing on complexity grounds — correctly — but in doing so discarded the one idea (global
homotopy-class assignment) that the witness data says is the actual missing piece. The
capacity proof (W2) means we do NOT need the full toporouter's power; we need only to pick the
right CLASS, which a much smaller mechanism can do.

---

## Solution Design — Topology-Class Routing (TCR)

### Core idea

Separate the two things our greedy router conflates: **(a) which homotopy class each net
occupies** (which side of each obstacle, which layer, in what angular order it escapes U3) and
**(b) the geometric realization** within that class. Compute (a) GLOBALLY for the contended
set so it is crossing-free *by construction*, using the capacity proof to guarantee
feasibility. Then realize (b) with the EXISTING gridless funnel/rubber-band machinery.

The novelty vs the rejected full toporouter: we do NOT build a board-wide CDT and route every
net topologically. We exploit the *measured structure* of this problem — a radial QFN escape +
a via ring + a connector fan — and reduce the global topology problem to two tractable
sub-problems with closed-form feasibility:

1. **Planar escape ordering.** The 18 QFN escape nets must leave the U3 field through the
   ring. On a single layer, a crossing-free radial escape exists IFF the order of nets around
   the ring (by via angle) matches the order of their PAD angles around U3 — i.e. the escape
   is a planar "monotone" fan. This is a *cyclic-order matching* problem, solvable exactly.
   Where pad-order and target-order conflict (a net must cross the fan), that net is forced to
   the OTHER layer — and the capacity proof (B.Cu < 22% used) guarantees room for it.

2. **Global corridor-side assignment.** After escape, each net runs to its connector through a
   sequence of corridors (gaps between obstacle groups: U3, J3/J4, J1/J5, passives). The
   homotopy class is the ordered sequence of "which side of each obstacle." Assigning these to
   minimize crossings + length over ALL contended nets at once is a **min-cost flow / linear
   assignment** on a corridor graph, NOT greedy A*. Feasibility is guaranteed because the
   witness exhibits one such assignment.

### Interface contract

A PRE-PASS, opt-in, no-op-safe by default (matching the `gridless_first`/`coopt` pattern in
multi.py). Proposed signature, mirroring existing params:

```python
def assign_topology_classes(
    nets: list[Net],          # the contended set (e.g. the 18 QFN escape nets)
    obstacles: ObstacleSet,   # pads, footprints, board edge (existing model)
    escape_center: tuple[float, float],  # U3 center
    ring_radius_mm: float = 6.0,         # measured: primary via ring 5-7mm
) -> dict[str, TopoClass]:
    """Return, per net, a homotopy class:
       TopoClass = (layer: 'F'|'B',
                    via_angle_deg: float | None,   # angular slot on the escape ring
                    corridor_sides: list[Literal['L','R']])  # side of each obstacle group
    Guaranteed crossing-free per layer (planar fan + assignment). Deterministic."""
```

The class then drives the EXISTING realizer: `route_net_gridless` is invoked with the escape
direction + via position FIXED by the class and a bounded window, so it cannot wander into the
wrong homotopy. No new geometry subsystem — the funnel/rubber-band is already in
`src/tracewise/route/gridless/realize.py` + `route.py`.

### Key mechanism

**Step 1 — Planar escape order (cyclic-order matching).** For each escape net, compute the
angle of its U3 pad (theta_pad) and the angle toward its first off-board target (theta_target).
Sort nets by theta_pad. Walk the sorted order; a net is layer-F-escapable iff assigning it the
next free angular slot on the F.Cu ring keeps the slot order monotone w.r.t. theta_target
(no inversion => no crossing). Inversions are pushed to B.Cu. This is O(n log n) and exact for
a single escape ring; it is the discrete core that greedy A* never solves because A* commits
one net's slot before seeing the others'.

**Step 2 — Corridor graph + global side assignment.** Build a coarse corridor graph: nodes =
obstacle-group gaps (the channels the W2 cut measured), edges = adjacency. Each net's
escape->connector route is a path in this graph; the homotopy class is the L/R side at each
node. Formulate as min-cost multi-commodity flow with per-edge CAPACITY = (channel width /
0.35mm pitch) from W2 — capacities are real and proven slack (~2x). Because capacities are not
binding, the LP relaxation is integral-friendly and a min-cost flow solver (networkx, or a
small custom successive-shortest-path) yields a crossing-consistent assignment. The witness
provides a warm-start / feasibility certificate.

**Step 3 — Constrained realization.** Feed each net's (layer, via_angle, corridor_sides) to
the existing gridless realizer as HARD guidance: fixed escape heading, fixed via site on the
ring, bounded window. The realizer's funnel produces the rubber-band geometry; legality is by
construction (it already rasterizes into the hard ledger with clearance inflation).

### Trade-off analysis

| Dimension | TCR (proposed) | Best existing (gridless-first attempt-3) | Full toporouter (rejected) |
|-----------|----------------|-------------------------------------------|----------------------------|
| Homotopy commitment | GLOBAL, crossing-free by construction | greedy/sequential | global |
| Legality (same-layer crossings) | 0 by construction | +72 err regression (crossings) | 0 by construction |
| Capacity handling | uses proven slack as flow capacity | discovers contention greedily | implicit |
| New subsystems | 0 (reuses realizer) | 0 | 3 (CDT, sketch, funnel) |
| Python LOC (prototype->prod) | ~400-800 | (built) | ~1500-2500 |
| Determinism | yes (sorts + min-cost flow) | yes | larger surface |
| Risk | escape model is mitayi-shaped, may not generalize | known +72 err | poor dense-2-layer results |

TCR does BETTER on the exact failure mode (illegal crossings, ordering trap) and reuses the
existing geometry. It does WORSE on generality: the planar-escape model is tuned to a
single-QFN-dominated board; a board with two big QFNs or a BGA needs the corridor graph to
carry more of the load. Maintenance cost: medium — the corridor graph is new but small; the
min-cost flow is a standard library call.

---

## Prototype (smallest experiment to validate the core)

**The falsifiable claim:** if we EXTRACT the human's homotopy classes from the witness and
feed them to our existing realizer, we reproduce <= human errors. If we then COMPUTE the same
classes from scratch (Step 1+2) and they MATCH the witness classes, the invention is proven.

**Experiment E1 (1 day) — Witness-class realization (does the realizer honor a class?).**
1. From the witness, extract per escape-net: (layer of first run, via angle on the ring,
   ordered corridor sides). `scripts/_invent2_topology.py` already extracts via radii/angles
   and the layer sequence — extend it to emit `TopoClass` per net.
2. Drive `route_net_gridless` with those classes as fixed guidance on the contended set.
3. Score: unconnected + errors via the existing DRC harness. SUCCESS = errors near 0 and
   unconnected drops materially below 41 with NO illegal-crossing inflation (audit with the
   W1 crossing test).
   - If this FAILS, the realizer cannot honor a class => the invention is infeasible in our
     engine and we stop (honest NO-GO). This is the critical de-risking gate.

**Experiment E2 (1 day, only if E1 passes) — Computed-class match.**
1. Implement Step 1 (planar escape order) + a minimal Step 2 (corridor side assignment for the
   escape ring only; defer full multi-commodity flow).
2. Compare computed classes to witness classes (Hamming distance on layer + side sequences).
   SUCCESS = computed classes reproduce the witness's crossing-free assignment for >= the
   escape set (the 8-net primary ring). Mismatches localize exactly where the model is too
   weak.

Prototype lives in `scripts/_invent2_*.py` (analysis) and a throwaway driver; NO production
code is modified by this proposal. The class-realization driver reuses `route_net_gridless`
read-only via the existing public surface.

**Demonstrates:** that the gap is the homotopy class, not capacity or geometry.
**Omits:** full multi-commodity flow, multi-QFN generality, the production pre-pass wiring,
congestion pricing integration.
**How to run (analysis already runnable):** `python3 scripts/_invent2_topology.py`.

---

## Feasibility / Correctness argument (grounded in the witness)

1. **A crossing-free 2-layer routing EXISTS** — the witness, W1 (0 crossings both layers).
   So the assignment problems in Step 1/2 are FEASIBLE; we are searching a non-empty space.
2. **No cut is binding** — W2 (peak ~56% F.Cu, <22% B.Cu at the tightest QFN ring; 27% at the
   J3 channel). Min-cost flow with these capacities has slack, so the LP relaxation is
   well-behaved and an integral crossing-consistent assignment exists (the witness IS one).
   This is the rigorous content of "the contention is an ordering artifact, not a min-cut."
3. **The escape is planar-realizable** — 18 nets, ring capacity ~71 at R=4mm; the human used
   far fewer F.Cu escapes than capacity, spilling the rest to a near-empty B.Cu. The
   cyclic-order matching of Step 1 is exact for a single ring.
4. **The realizer is legal-by-construction** — it already rasterizes into the hard ledger with
   clearance inflation (CROSS-SUBSTRATE-COOPT.md), so honoring a class cannot reintroduce the
   +72 crossings that the greedy gridless-first produced.

What I do NOT claim: I cannot sketch a *proof* that Step-1+2 ALWAYS recovers a feasible class
for arbitrary boards (multi-commodity flow with integrality + planarity is NP-hard in general).
The claim is bounded: for THIS witness-shaped problem (single dominant QFN escape + slack
capacity) the decomposition is exact, and the witness certifies feasibility. Over-claiming a
general proof would be dishonest.

---

## What's NOVEL vs the exhausted approaches

- Every prior approach committed homotopy GREEDILY (per-net A*, sequential gridless, region
  pre-route). **TCR computes the homotopy class GLOBALLY before any geometry**, which is the
  one thing none of them did. Probe-Order proved the set is ordering-bound; TCR is the missing
  mechanism that keeps the whole set in ONE mutually-legal class instead of negotiating into
  +72 crossings.
- It turns the capacity finding (W2) into an ALGORITHM: channel slack becomes flow capacity,
  the witness becomes a feasibility certificate / warm start.
- It is legal-by-construction per layer (W1), so it structurally cannot produce the
  illegal-crossing inflation that made the "41/136" number partly illusory.
- It is NOT the full toporouter the survey rejected: no board-wide CDT, no 3 subsystems. It is
  a scoped pre-pass (planar escape order + min-cost flow side assignment) feeding the existing
  realizer.

---

## Tractability (honest, in Python, for mitayi)

- **Step 1 (planar escape order):** O(n log n) sort + linear scan, n=18. Trivial. ~100 LOC.
- **Step 2 (corridor side assignment):** corridor graph has ~10-20 nodes for mitayi; min-cost
  flow via networkx or a 150-LOC successive-shortest-path. Tractable. The general
  multi-commodity case is NP-hard, but mitayi's slack capacity (W2) and small size make a
  practical integral solution reachable; if the LP gives fractional flow, round + repair (the
  slack absorbs rounding). ~300 LOC.
- **Step 3 (realization):** reuses `route_net_gridless` — zero new geometry. The hard part
  (funnel, rasterization, ledger) already exists and is tested (381+ tests).
- **Total new code, prototype:** ~400-800 LOC, all standard-library + numpy + the existing
  engine. No Shapely required for the prototype (the realizer's numpy path suffices).
- **Honest risk:** the BENEFIT depends entirely on E1 — whether the realizer can be *steered*
  by a class without re-deriving it greedily. If `route_net_gridless` ignores or overrides the
  escape/via guidance, TCR collapses to the existing gridless-first and inherits its +72. E1
  is exactly the gate that tests this in ~1 day. **Do E1 before any build.**
- gEDA toporouter's poor dense-2-layer record is REAL and acknowledged. TCR sidesteps the
  reason it failed (full CDT topological freedom is thin on 2 layers) by NOT doing full
  topological routing — it only fixes escape order + corridor sides, where 2 layers is enough
  (the witness proves it).

---

## Phase 5 — Self-assessment (is a NEW approach warranted?)

**Verdict: RECOMMEND WITH CAVEATS — invent the SCOPED pre-pass (TCR Step 1+2), NOT a full
topological router. And gate it on the 1-day E1 experiment.**

- **Is invention necessary?** The greedy family is genuinely exhausted at ~41 and that number
  is inflated by illegal crossings. The witness proves capacity is fine and a legal class
  exists. No existing repo mechanism computes a global homotopy class. So YES, a new mechanism
  is warranted — but it is a *small* one (global class assignment), recombined with the
  existing realizer, not a from-scratch toporouter.
- **Recombination vs new?** ~70% recombination (realizer, ledger, scorecard, escape concept,
  super-cell pricing all reused), ~30% genuinely new (planar escape ordering + min-cost-flow
  corridor-side assignment as a pre-pass). The NEW 30% is precisely the missing piece every
  prior approach lacked.
- **Prior art honesty:** the project tried PathFinder negotiated congestion, a coarse-grid
  pivot, gridless-first, and cross-substrate co-opt. Topological routing is a KNOWN family
  (gEDA toporouter, TopoR, Freerouting) but UNATTEMPTED here, and the survey explicitly
  rejected the *full* version on complexity. TCR threads that needle: take the family's
  insight (separate topology from geometry, legal-by-construction) without its cost (3
  subsystems, board-wide CDT).
- **Maintenance burden:** Medium. The corridor graph + flow is new and mitayi-shaped; needs
  generalization tests before it touches other boards. Default-OFF / no-op-safe (proven
  pattern in multi.py) keeps the blast radius zero until validated.
- **Escape hatch:** opt-in flag, byte-identical when off; if E1 fails, the whole approach is
  abandoned at 1 day cost with no production change. Falls back to attempt-3 (41/73).
- **Why not just "fix the greedy ordering"?** That's what gridless-first attempt 1-3 tried;
  it lacks a mechanism to keep the set in one legal class and regresses errors. The global
  assignment IS the fix for the ordering, made principled.

---

## Validation experiment (the one number that decides it)

Run E1: extract the witness's per-net `TopoClass`, drive the existing realizer with it on the
18 escape nets + grid-route the rest, score with the DRC harness, and AUDIT crossings with the
W1 test. **GO iff unconnected < 41 AND errors materially < 73 AND 0 illegal crossings.** If E1
passes, do E2 (computed classes match witness). If E1 fails, NO-GO at 1 day — the realizer
cannot be steered and the invention is infeasible in this engine.

---

## Handoff notes (if E1/E2 pass)

- **Files (prototype, throwaway):** `scripts/_invent2_topology.py` (done — extend to emit
  `TopoClass`); `scripts/_invent2_e1_realize.py` (new — class-driven realization driver,
  read-only use of `route_net_gridless`).
- **Production (only after GO):** `src/tracewise/route/topo/assign.py` (Step 1+2), wired as an
  opt-in pre-pass in `multi.route_all` mirroring `gridless_first`/`coopt` (default None =>
  byte-identical). Reuse `route_net_gridless` for Step 3.
- **Test strategy:** determinism (same input => same classes); the W1 crossing-audit as a
  regression gate; E2 class-match as a fixture; no-op-safe default test.
- **Gotcha:** the realizer must accept and HONOR escape-direction + via-site guidance without
  re-deriving greedily — this is the make-or-break and is exactly what E1 measures first.
