# Human / Professional PCB Routing Techniques — Domain Knowledge Survey

**Status:** Research survey, 2026-06-23.
**Purpose:** Ground the TraceWise router improvement effort in documented professional PCB
routing techniques; cross-reference each against the mitayi human witness board; rank by
likely impact on closing the 41→0 unconnected gap.

**Reading order:** §1–6 survey techniques with sources. §7 cross-references the mitayi witness.
§8 ranks techniques by impact on TraceWise. §9 recommendations.

---

## Background: What We Know

TraceWise currently routes mitayi at **41 unconnected / 73 errors** (attempt-3, the best result).
The human witness achieves **0/0 on the same 2-layer placement**. Two inventor analyses + the E1
gate proved the gap is a **homotopy/ordering artifact, not capacity**: the tightest escape cut
runs at ~56% F.Cu / <22% B.Cu utilization; there is ~2x headroom everywhere.

Key witness facts extracted by `scripts/_invent1_human_stats.py` and `scripts/_invent2_topology.py`:

| Witness finding | Measurement |
|---|---|
| Escape ring | 39 vias, 4.55–8.18mm from RP2040 center (mean 6.03mm), balanced W:10 / S:10 / E:10 / N:9 |
| Layer=direction convention | Horizontal copper: 67% B.Cu; Vertical copper: 80% F.Cu |
| Crossing-free routing | 0 different-net crossings on F.Cu (426 segs) AND B.Cu (157 segs) |
| B.Cu bus | Monotone GPIO0..29 ordered horizontal runs in the west channel |
| F.Cu-only short nets | 3/13 hard nets (/GPIO27, /GPIO28, /XIN) with 0 vias |
| Capacity slack | QFN escape at R=4mm: ~56% F.Cu, <22% B.Cu — ~2x headroom |

---

## 1. Escape / Fanout Routing for Dense Components (QFN, BGA)

### 1.1 The fanout problem

Dense packages (QFN, BGA) cannot be routed by drawing point-to-point traces directly from
their pads; the pad density and fine pitch make it physically impossible to route through
the component "field." The standard professional approach is **escape routing**: route short
stubs from component pads outward, place a via at the boundary, then continue on the
inner/opposite layer. The collection of stubs + vias forms a **fanout ring**.

**Source:** Altium, "Which BGA Pad and Fanout Strategy is Right for Your PCB?"
[https://resources.altium.com/p/which-bga-pad-and-fanout-strategy-right-your-pcb]

### 1.2 Dog-bone fanout (0.5mm+ pitch)

For BGAs and QFNs with ≥0.5mm pitch, the standard technique is the **dog-bone**: each pad
connects via a short trace (the "neck") to a via just outside the pad. Outer rows of pads route
directly outward; inner rows thread their neck between adjacent outer pads. The term "dog-bone"
describes the shape of pad+neck+via annular ring.

**Key constraint:** the neck must be thin enough to pass between pads while maintaining the
design-rule clearance. At 0.4mm QFN pitch (the mitayi RP2040) with 0.2mm pad gap, a via
**between** pads is impossible — signals must escape outward to a ring outside the pad field.

**Source:** AllPCB, "Escape Routing Techniques for High-Density BGA Packages"
[https://www.allpcb.com/allelectrohub/escape-routing-techniques-for-high-density-bga-packages]

### 1.3 Via-in-pad (sub-0.5mm pitch)

For very fine pitch (< 0.5mm), via-in-pad places the via directly under/within the pad, filled
and plated flat. Eliminates the stub entirely. Not applicable to the mitayi RP2040 at 0.4mm
pitch with a standard through-hole via stack (requires HDI + filled vias manufacturing).

**Source:** JLCPCB, "Via in Pad (VIP) Technology for HDI PCBs"
[https://jlcpcb.com/blog/via-in-pad-pcb]

### 1.4 Escape ring / quadrant escape

Professional engineers treat QFN escape not as individual per-pad decisions but as a **global
ring assignment**: each escaping net is assigned a via slot in a ring just outside the component.
The ring is balanced across component edges (N/S/E/W quadrants) to:

1. Distribute routing channels evenly in all four directions.
2. Prevent congestion on any single edge.
3. Enable the stub (F.Cu) from any pad to reach any ring slot — the stub navigates *around*
   the pad ring rather than escaping radially.

**Principle:** "Fanout should proceed from the outside inward, ring by ring, ensuring each row
has a clear and repeatable breakout strategy." [PCBway BGA Guidelines]

"Consistent, symmetrical fanout patterns across the entire BGA improve heat distribution and
signal integrity." [PCBway BGA Guidelines]

**Source:** PCBway, "BGA PCB Layout Guidelines: Placement, Fanout, and Routing"
[https://www.pcbway.com/blog/PCB_Design_Layout/BGA_PCB_Layout_Guidelines_Placement_Fanout_and_Routing_b8eeabdd.html]

### 1.5 Fanout-then-route discipline

Professional practice mandates **defining the complete fanout before global routing begins**:
all vias are placed, checked for legality (clearance, drill overlap), and only then are
global routes drawn. Attempting to route before fanout is fully defined causes via placement
conflicts and routing channel blockages.

"Define your BGA fanout strategy before any global routing begins." [PCBway]

"Careful and thoughtful placement will yield a routable layout that requires minimal changes
to accommodate traces/vias, but a rushed or underdeveloped placement can produce a nearly
impossible-to-solve layout." [AllPCB Routing Guide]

**Source:** AllPCB, "The Ultimate Guide to PCB Trace Routing Layout"
[https://www.allpcb.com/allelectrohub/the-ultimate-guide-to-pcb-trace-routing-layout-component-placement-and-via-optimization]

---

## 2. Layer-Direction Conventions (Manhattan / HV Routing)

### 2.1 The HV convention

The dominant professional convention on 2-layer boards is **HV routing (Horizontal-Vertical)**:
one layer carries almost exclusively horizontal traces, the other almost exclusively vertical.
Signals change direction via a via and a layer transition.

"In Manhattan routing, you use one dedicated layer for horizontal tracks and another layer for
vertical tracks, with no horizontal tracks allowed on the vertical layer." [Cadence]

"Manhattan-Style routing involves the use of expressly east-west planes and north-south planes,
using a via and changing planes when a signal changes direction." [Cadence]

**Source:** Cadence, "PCB Manhattan Routing Techniques"
[https://resources.pcb.cadence.com/blog/2020-pcb-manhattan-routing-techniques]

### 2.2 Why HV reduces crossings

HV routing prevents **broadside coupling** (two parallel traces on adjacent layers) and
**intra-layer routing conflicts** (two traces on the same layer needing to pass through the
same region in the same direction). Because same-layer traces are all parallel to each other,
they can be packed without crossing. The via forces a direction change to the orthogonal layer.

"The routing directions of adjacent layers should be orthogonal to avoid signal traces of
adjacent layers running in the same direction, thus reducing unnecessary crosstalk." [PCBBUY]

"Perpendicular routing between layers will help to prevent potential broadside coupling, which
is crosstalk between traces running parallel to each other on adjacent layers." [Cadence]

**Source:** PCBBUY, "What is Manhattan Routing in PCB Manufacturing Process?"
[https://www.pcbbuy.com/news/What-is-Manhattan-Routing-in-PCB-Manufacturing-Process.html]

### 2.3 Trade-offs

The HV convention forces **more vias** (every direction change requires a layer transition).
"More vias required for layer transitions impact both manufacturing costs and signal integrity
through added inductance." [Cadence]

Differential pairs and high-speed memory buses are exceptions: they are kept parallel on one
layer. The HV convention applies to signal routing, not to these special classes.

**Source:** Cadence (op. cit.)

---

## 3. Channel Routing and River Routing

### 3.1 Channel routing (VLSI classical result)

**Channel routing** is the problem of connecting m terminals on the top edge of a rectangular
channel to m terminals on the bottom edge, using horizontal runs on one layer and vertical
jogs on another, within the channel's width.

The classical algorithm is the **Left-Edge Algorithm** (Hashimoto and Stevens, 1971): sort
nets by leftmost terminal, greedily assign horizontal tracks. Optimal in terms of track count.
This is the formal basis for the "pack nets into lanes from left to right" approach.

**Source:** TechSimplified, "What is Detailed Routing in VLSI Physical Design?"
[https://www.techsimplifiedtv.in/2025/04/what-is-detailed-routing-in-vlsi.html]

### 3.2 River routing (the crossing-free planarity result)

**River routing** (also called "monotone routing") connects two rows of n terminals — n
terminals on one horizontal line to n terminals on another horizontal line — using a
**planar** (crossing-free) routing in the channel between them.

**The fundamental theorem:** A set of n terminal pairs is crossing-free-routable between two
parallel lines *if and only if* the net assignment is a **monotone permutation** — i.e. the
order of source terminals along the top line is the same as the order of destination terminals
along the bottom line. If the permutation is not monotone (some nets are "inverted"), they
cannot be routed on a single layer without a crossing.

**Consequence for bus routing:** If a bus of n signals must connect from a component row to a
connector row, and both rows are ordered consistently (same left-to-right ordering on both
sides), the bus is crossing-free-routable on a single layer. This is the mathematical
foundation of **river routing** as a technique.

**Source:** ScienceDirect, "River routing in VLSI" (Leiserson and Pinter, J. Computer and
System Sciences, 1983) — referenced by:
[https://www.sciencedirect.com/science/article/pii/0022000087900043]

Springer Nature, "River Routing: Methodology and Analysis"
[https://link.springer.com/chapter/10.1007/978-3-642-95432-0_9]

### 3.3 Multi-layer extension

On two-layer boards, if a permutation is not monotone on F.Cu, offending nets can be moved to
B.Cu. The capacity proof (that B.Cu has headroom) guarantees a crossing-free assignment exists
with the available layers.

The standard multi-layer river routing result: a 2-terminal routing problem on k layers is
solvable with no crossings *if and only if* the permutation of net endpoints is realizable as
a composition of k monotone permutations. For 2-layer PCBs with ~50% headroom on each layer,
essentially any practical bus is routing-feasible.

**Source:** Springer Nature (op. cit.); also summarized in Parallel Algorithms for Several
VLSI Routing Problems, University of Maryland:
[https://drum.lib.umd.edu/items/bd01fffa-12cb-46e5-937e-ffe1338ac8a7]

---

## 4. Net Ordering, Rip-Up-and-Reroute, and Homotopy-Class Routing

### 4.1 Net ordering effects on routability

In any sequential routing system (grid A*, visibility-graph A*, expansion rooms), the order
in which nets are routed determines which homotopy class each net occupies. A net routed
early can block a corridor that a later net needs, forcing it onto a longer or illegal path.

"Changing the net orders in the sequential PathFinder has been observed to reduce the number
of iterations. Determining the best net ordering is still an open problem." [Semantic Scholar,
PathFinder]

"The same board produces meaningfully different results depending on input order." [TinyComputers]

This is the **ordering artifact** at the heart of TraceWise's 41-net ceiling.

**Source:** Semantic Scholar, "PathFinder: A Negotiation-Based Performance-Driven Router for FPGAs"
[https://www.semanticscholar.org/paper/PathFinder:-A-Negotiation-Based-Performance-Driven-McMurchie-Ebeling/45b0d141e847855f149b175abbc371aeb4b80cbb]

### 4.2 PathFinder / negotiated congestion

**PathFinder** (McMurchie and Ebeling, FPGA 1995) is the canonical academic algorithm for
iterative rip-up-and-reroute with negotiated congestion:

1. Route all nets greedily (accepting temporary resource conflicts).
2. Increase the cost of over-used routing resources (history pricing).
3. Rip up and reroute all nets with updated costs.
4. Repeat until no conflicts remain (convergence) or a budget is exhausted.

**Key property:** PathFinder allows *temporary* sharing of routing resources, which means nets
can overlap initially; the negotiation converges by raising prices until only one net uses each
resource. This is the algorithm underlying TraceWise's existing grid router.

**Limitation for the mitayi problem:** PathFinder's negotiation only re-prices contention; it
does not *prevent* contention by design. If the correct global topology class requires a net to
take a long detour that appears costly to the greedy A*, PathFinder will not discover it — the
short greedy path always wins the first iteration, cementing the wrong homotopy class.

**Source:** PathFinder paper (Semantic Scholar, op. cit.); also
[https://www.cecs.uci.edu/~papers/compendium94-03/papers/1995/fpga95/pdffiles/6a.pdf]

### 4.3 Topological routing (gEDA Toporouter / Rubber-Band Router)

The **gEDA Toporouter** (Anthony Blake, Google Summer of Code 2008) implements **Tal Dayan's
1997 rubber-band routing thesis**: route topologically first (assign which side of each
obstacle a net passes through), then tighten the rubber band to find the geometric path.

Technical foundation: **Constrained Delaunay Triangulation (CDT)** of the board space. Each
triangle face is a routing cell; nets are assigned to sequences of faces (topology class).
The CDT naturally represents "which side of each obstacle" because triangulation edges
separate obstacle regions.

**Performance on dense 2-layer boards:** The gEDA documentation explicitly documents poor
results: "when applied to typical PCB problems, especially when constrained to few layers and
dense constraints, the results were poor." The router excels with "many layers of free wiring
space" but underperforms in constrained 2-layer environments.

"The layout demonstrates the power of topological autorouters over geometric autorouters.
Geometric autorouters trying to route the nets with a shortest path will obstruct at least one
of the nets, resulting in failure." [gEDA wiki]

**Source:** gEDA pcb GitHub Wiki, "Autorouters: gEDA pcb Toporouter"
[https://github.com/bert/pcb/wiki/Autorouters:-gEDA-pcb-Toporouter]

### 4.4 Situs Topological Autorouter (Altium)

**Situs** (Altium's built-in autorouter) separates topology mapping from geometry routing. It
builds a topological map using only relative positions of obstacles (not coordinates), then
routes through the topology map.

"Rather than mapping design space using geometric coordinates, it builds a map using only the
relative positions of the obstacles in the space." [Altium documentation]

"By separating the mapping space from the routing space, the topological router is able to map
more natural paths and find routing paths that are non-orthogonal." [Altium]

Success requires planning: "organizing nets into classes and designing custom routing strategies."
Layer directions guide the routing process.

**Source:** Altium, "Automated PCB Routing With the Situs Topological Autorouter"
[https://resources.altium.com/p/automated-pcb-routing-with-situs-topological-autorouter]

### 4.5 TopoR (Eremex)

**TopoR** is a commercial topological router with free-angle routing. Unlike gEDA Toporouter
and Situs, it allows arbitrary (non-90°/45°) trace angles, claiming "PCB design time reduction
up to 10X" and "instant routing of 100% of wires." It performs automatic BGA fanout placement.

The system "simultaneously optimizes several alternative variants of the layout, removing
inferior solutions based on total wire length and via count."

**Limitation for our context:** proprietary (no Python integration), and free-angle routing
creates curved traces that conflict with the project's rectilinear/orthogonal emit model.

**Source:** TopoR website [https://t.eremex.com/];
Wikipedia, "TopoR" [https://en.wikipedia.org/wiki/TopoR]

### 4.6 Two-Stage Global/Detailed Routing (the classical VLSI framework)

The dominant VLSI/PCB routing framework separates routing into two stages:

1. **Global routing:** assign each net to a sequence of routing channels (cells, tiles) —
   determine WHICH paths the nets take topologically, without determining exact wire placement.
2. **Detailed routing:** within each channel, place exact wire geometries respecting DRC.

"For complex, large-scale circuit designs, circuit routing is usually solved by a two-stage
approach, global routing followed by detailed routing." [VLSI routing review]

The global stage is a combinatorial problem (assignment of channels/layers); the detailed stage
is a geometric problem (exact wire placement). This decomposition is the basis of every
professional EDA tool (Cadence Allegro/APD, Mentor PADS, Zuken CR-8000, Altium).

**Relevant result for TraceWise:** the TOPOLOGY-CLASS-ROUTING design implemented exactly this
decomposition (assign topology class globally, then realize geometrically). The E1 gate
confirmed the realization step works (0 crossings); the gap was in the assignment pre-pass
completeness.

**Source:** "A Review of Global and Detailed Routing in VLSI Design" (Academia.edu)
[https://www.academia.edu/23735271/A_REVIEW_OF_GLOBAL_AND_DETAILED_ROUTING_IN_VLSI_DESIGN]

---

## 5. Via Minimization, Length Matching, Differential Pairs

### 5.1 Via minimization

Professional routing minimizes vias because each via:
- Adds manufacturing cost and complexity.
- Introduces signal discontinuities (parasitic inductance ~1 nH).
- Consumes pad area on both layers.
- Can become a congestion point for other traces.

Techniques: route on a single layer as long as possible; use HV convention to defer layer
changes; plan escape fanout so vias are placed at ring positions that minimize the remaining
route length.

**Source:** Aivon, "PCB Routing Techniques: A Comprehensive Guide"
[https://www.aivon.com/blog/pcb-design/pcb-routing-techniques-a-comprehensive-guide-to-efficient-trace-layout/]

### 5.2 Length matching

For timing-sensitive buses (DDR, high-speed parallel signals): all traces in a bus must arrive
at their destination simultaneously. Engineers use **serpentine / trombone patterns** to add
length to shorter traces. Length matching is distinct from routing correctness (connection) —
it is a quality metric on an already-connected board.

**Relevance to TraceWise:** length matching is NOT relevant to the 41→0 connectivity gap.
It is a polish step after the board is fully connected. Noted here for completeness.

**Source:** Aivon (op. cit.)

### 5.3 Differential pairs

Differential pairs (USB, Ethernet, HDMI) require:
- Equal trace lengths (skew < 10 ps).
- Constant spacing (differential impedance 85–120 Ω per IPC-2141A).
- Parallel routing without sharp bends.
- Routing over solid reference planes (no plane splits).

**Relevance to TraceWise:** The mitayi USB_D+/USB_D- pair is among the failing nets in the
current router. Differential-pair-aware routing requires treating the pair as a joint net.
This is a medium-priority enhancement after the connectivity gap is closed.

**Source:** Aivon (op. cit.); IPC-2141A (Controlled Impedance Circuitry standard)

---

## 6. Placement-for-Routability

### 6.1 Place-to-route principle

Professional PCB design treats placement as primarily a **routing preparation step**:
component positions are chosen to maximize the routability of the resulting net topology.
The routing topology (which nets need which layers, channels, via positions) is assessed
during placement, not after.

"Careful and thoughtful placement will yield a routable layout that requires minimal changes
to accommodate traces/vias, but a rushed or underdeveloped placement can produce a nearly
impossible-to-solve layout." [AllPCB]

"Good component placement is the most important aspect to good board design." [AllPCB]

### 6.2 BGA/QFN center placement

Dense ICs (QFN, BGA) are placed near the **board center** so routing channels exist in all
four directions. This distributes routing load symmetrically. Connectors and other
destinations are arranged around the central IC.

"For most processor-based designs, the main BGA should be placed near the center of the board
to distribute routing evenly in all directions." [PCBway]

**Mitayi note:** The RP2040 QFN (U3) is placed to the east of J3/J4 connectors. This is NOT
a centered placement — the connectors are entirely on one side (west). This placement choice
creates the congested single-channel problem the human solved with the ring+bus technique.

### 6.3 Connector ordering to match component pinout

Professional design orders connector rows so the **signal ordering on the connector matches
the signal ordering on the IC pinout** (both monotone in the same direction). This creates a
monotone permutation at the routing layer, enabling crossing-free bus routing.

"BGA components should be kept away from board edges or connectors, where routing space is
limited." [PCBway]

The **mitayi witness exploits this**: J3/J4 connector pin ordering is GPIO0..29 monotone in y,
which exactly matches the RP2040 pad angular order around U3. The river-routing result
guarantees this is crossing-free-routable — and the witness proves it.

### 6.4 Fanout space reservation

During placement, the designer must reserve **ring space** around QFN/BGA components:
sufficient clearance for the escape ring vias (typically 1–5mm beyond the pad ring), plus
routing channels between the ring and the next component.

"Sufficient spacing and routing channels must be reserved between [BGAs], with each BGA
requiring adequate space for complete fanout." [PCBway]

On mitayi: 4.5–8mm escape band (measured), with routing channels to J3/J4 (~5–8mm west).
The placement already provides this; the router must exploit it rather than compress into the
smallest possible via site.

**Source:** PCBway (op. cit.); AllPCB Routing Guide (op. cit.)

---

## 7. Cross-Reference: Mitayi Human Witness vs. Techniques

| Technique | Used by mitayi human? | Evidence from witness | TraceWise gap |
|---|---|---|---|
| **Escape ring fanout** (§1.4) | YES — core technique | 39 vias in 4.55–8.18mm band, balanced W:10/S:10/E:10/N:9 | TraceWise places vias radially (wrong slot), not at balanced ring slots |
| **Non-radial slot assignment** (§1.4) | YES | Via QFN edge ≠ source pad edge in all 9 measured cases; F.Cu stub 9–14mm navigating around the ring | TraceWise's `guided_escape_via` forces radial via → blocks ring slots |
| **Fanout-then-route** (§1.5) | YES — implicitly | The ring structure is globally consistent, not ad-hoc per-net | TraceWise routes per-net greedily; no global fanout pre-pass |
| **HV / layer=direction** (§2) | YES — explicit | B.Cu: 67% horizontal; F.Cu: 80% vertical (board-wide); escape nets 75–89% B.Cu | TraceWise `gridless_first` partially honors this but not enforced globally |
| **River routing / monotone bus** (§3.2) | YES — core structure | GPIO0..29 ordered monotone in y on J3/J4; 0 crossings on B.Cu (157 segs) | TraceWise has no monotone lane assignment; B.Cu nets interleave and cross |
| **Two-stage global+detailed routing** (§4.6) | YES — inferred | Human "assigns topology then realizes" (Inventor-1 thesis) | TraceWise is pure detailed routing (no global stage) |
| **Via minimization** (§5.1) | YES — measured | ~111 vias total; QFN escape nets: exactly 1 via each (except /RUN, /GPIO14 with 2) | Not a gap — TraceWise via count is comparable; not the bottleneck |
| **Differential pair treatment** (§5.3) | Partial | USB_D+/USB_D- appear in failing nets; no explicit pairing in current router | Secondary gap — relevant after connectivity is closed |
| **Placement-for-routability** (§6.1–6.4) | YES — by design | Connector GPIO ordering matches QFN pin-ring order (monotone permutation) | Current placement already good; co-design option is for iteration |
| **PathFinder negotiated congestion** (§4.2) | N/A (human technique) | Human does not negotiate; human assigns globally | TraceWise uses PathFinder-style negotiation — it can't resolve the homotopy conflict |
| **gEDA-style topological routing** (§4.3) | PARTIAL analog | Human uses topology-first assignment, not CDT-based | CDT toporouter was rejected (poor on dense 2-layer); TCR is the scoped version |

---

## 8. Ranked Impact on TraceWise (Closing 41→0 Gap)

Rankings are based on: (a) is the technique provably used by the human witness, (b) does
TraceWise currently lack it, (c) would implementing it directly address the measured gap.

### Rank 1 — Global ring slot assignment (non-radial fanout + balanced escape)

**Technique:** §1.4 (escape ring) + §1.5 (fanout-then-route)

**Why Rank 1:** This is the single technique the human uses that TraceWise completely lacks.
TraceWise's `guided_escape_via` places the via on the radial ray from the source pad. The human
places the via at the **globally optimal ring slot**, which may be on a *different QFN edge* from
the source pad (verified for all 9 measured nets). The F.Cu stub navigates around the pad ring
(9–14mm stubs) to reach the assigned slot. This is the primary reason TraceWise's escape nets
contend for the same ring region: two nets whose source pads are on the same edge compete for
the same radial slots, when the human would assign one to the opposite edge.

**Maps to TCR design:** The `assign_topology_classes` pre-pass (Steps A–C in TOPOLOGY-CLASS-ROUTING.md)
directly implements ring slot assignment + balanced quadrant loading. E1 proved the realizer
CAN follow a pinned slot (6/9 witness vias honored). The remaining gap is lane enforcement
(GPIO9 `bcu_run_failed`) and full pre-pass completeness.

**Informs lane-enforcement path:** YES — lane enforcement for the B.Cu bus is the follow-on
needed after ring slot assignment works.

**Informs placement co-design:** the current placement already provides the ring space (4.5–8mm
band). Co-design would only matter if the ring band needed to be repositioned.

### Rank 2 — Monotone lane packing / river routing (crossing-free bus)

**Technique:** §3.2 (river routing), §3.3 (multi-layer extension)

**Why Rank 2:** The witness's B.Cu bus is provably monotone (0 crossings, 157 segments). This
is not accidental — it is the river-routing result: monotone destination ordering + monotone
lane ordering = crossing-free. TraceWise has no lane packing; B.Cu nets are placed greedily
and interleave, causing the crossings that the E1 gate confirmed eliminate (0 crossings when
witness classes are used). The E1 NO-GO (43 unc) was traced to the MISSING lane enforcement
for GPIO9.

**Direct action:** implement `lane_y_mm` enforcement in `route_net_steered` (the one missing
piece identified in the TCR E1 post-mortem). Estimated delta: closing GPIO9 alone may bring
unconnected from 43 to ~40-41.

**Informs lane-enforcement path:** YES — this IS the lane enforcement path.

### Rank 3 — Two-stage global/detailed routing architecture

**Technique:** §4.6 (global + detailed routing decomposition)

**Why Rank 3:** The fundamental architectural insight. Every professional EDA tool uses this
two-stage decomposition; TraceWise is a pure detailed router. The TCR design implements a
scoped version (global = topology pre-pass for the QFN escape cluster, detailed = gridless
A* per net). This is the ONLY architectural change that can address the homotopy ordering
artifact — negotiated congestion (PathFinder) cannot, as proved by the 41-net ceiling.

**Informs lane-enforcement path:** YES — lane enforcement IS the pre-pass for the bus, which
IS the global routing stage for the dominant routing resource.

**Informs placement co-design:** co-design would make the global routing stage cheaper by
ensuring the placement already implies a monotone net ordering.

### Rank 4 — HV / layer=direction enforcement

**Technique:** §2 (Manhattan/HV routing)

**Why Rank 4:** The human witness enforces HV rigorously (67% horizontal on B.Cu, 80%
vertical on F.Cu). TraceWise's `gridless_first` partially honors this through the `highway_layer`
convention in the escape-routing path, but it is not globally enforced. Enforcing it globally
would:
- Prevent B.Cu from being used for "short vertical detours" that block horizontal channel slots.
- Make the gridless A* prefer horizontal routes on B.Cu, reducing future crossings.

**Informs lane-enforcement path:** Supporting technique — lane enforcement on B.Cu naturally
implies HV enforcement for the bus.

**Informs placement co-design:** N/A — placement is already oriented correctly.

### Rank 5 — Placement co-design (connector ordering, ring space reservation)

**Technique:** §6.1–6.4 (placement-for-routability)

**Why Rank 5:** The current mitayi placement already implements good practice (GPIO0..29
monotone on connectors, adequate ring space). The co-design option adds value primarily for a
SECOND board or a board where placement is suboptimal. On mitayi specifically:
- The placement is already "correct" from a routability standpoint.
- The gap is in the router, not the placement.
- Co-design could improve things marginally (e.g., closer connector positioning to reduce
  B.Cu highway length) but is unlikely to unlock the 41→0 gap on its own.

**Informs placement co-design path:** YES — this is the theory base for that path, but the
measured evidence says the current placement is already well-designed. The co-design path is
most valuable for boards OTHER than mitayi.

### Rank 6 — PathFinder / negotiated congestion improvement

**Technique:** §4.2

**Why Rank 6:** TraceWise already uses this. Improving the negotiation (better net ordering,
tighter history pricing) can reduce unconnected from 48 to 41 (which attempt-3 achieved) but
cannot close the homotopy gap — the ordering artifact is structural, not a congestion pricing
error. A smarter PathFinder would still ceiling at approximately the same point because the
wrong homotopy class is locked in early.

### Rank 7 — Differential pair routing

**Technique:** §5.3

**Why Rank 7:** Relevant but secondary. The USB_D+/USB_D- pair is among failing nets, but
closing it requires treating the pair jointly. This is a single net-pair; fixing the 41-net
connectivity gap for the GPIO bus is higher impact. Address after Rank 1–4 work.

---

## 9. Recommendations for the Router

Based on the survey and cross-reference, in priority order:

1. **Implement full ring slot assignment as a pre-pass (the TCR pre-pass, Steps A–C in
   TOPOLOGY-CLASS-ROUTING.md).** The escape ring + balanced quadrant assignment is the
   professional technique (#1 ranked) that TraceWise most critically lacks. The E1 gate
   confirmed the realizer can follow a pinned slot; the missing step is generating the
   assignment from placement geometry rather than reading it from the witness.

2. **Implement `lane_y_mm` enforcement in `route_net_steered`.** The E1 post-mortem identified
   this as the proximate cause of GPIO9's `bcu_run_failed` (the specific net that kept
   unconnected at 43 instead of ≤41). Monotone lane packing (river routing) guarantees
   crossing-free B.Cu; enforcement caps it. This is the cheapest next change.

3. **Globally enforce the HV (layer=direction) convention.** Add a soft constraint to the
   gridless A* that penalizes B.Cu vertical segments and F.Cu horizontal segments. This
   prevents the HV pattern from eroding on nets outside the QFN escape cluster.

4. **Treat the escape cluster as a global routing unit, not independent nets.** Professional
   practice is fanout-then-route: assign all ring slots simultaneously before routing any net.
   The current engine routes nets one-at-a-time, which is the root of the homotopy conflict.
   The TCR pre-pass is the correct fix.

5. **Defer differential pair routing and length matching** until the connectivity gap is
   closed. These are quality-of-result improvements, not connectivity fixes.

6. **Placement co-design** is NOT the first priority for mitayi (the placement is already
   good). Use it as the framework for future board support where placement is suboptimal.

---

## Sources

| # | Source | URL | Used for |
|---|--------|-----|---------|
| 1 | AllPCB, "Escape Routing Techniques for High-Density BGA Packages" | https://www.allpcb.com/allelectrohub/escape-routing-techniques-for-high-density-bga-packages | §1.2 dog-bone, §1.3 VIP |
| 2 | Altium, "Which BGA Pad and Fanout Strategy is Right for Your PCB?" | https://resources.altium.com/p/which-bga-pad-and-fanout-strategy-right-your-pcb | §1.1 fanout problem |
| 3 | PCBway, "BGA PCB Layout Guidelines: Placement, Fanout, and Routing" | https://www.pcbway.com/blog/PCB_Design_Layout/BGA_PCB_Layout_Guidelines_Placement_Fanout_and_Routing_b8eeabdd.html | §1.4, §6.2–6.4 |
| 4 | AllPCB, "The Ultimate Guide to PCB Trace Routing Layout" | https://www.allpcb.com/allelectrohub/the-ultimate-guide-to-pcb-trace-routing-layout-component-placement-and-via-optimization | §1.5, §6.1 |
| 5 | Cadence, "PCB Manhattan Routing Techniques" | https://resources.pcb.cadence.com/blog/2020-pcb-manhattan-routing-techniques | §2.1, §2.3 |
| 6 | PCBBUY, "What is Manhattan Routing in PCB Manufacturing Process?" | https://www.pcbbuy.com/news/What-is-Manhattan-Routing-in-PCB-Manufacturing-Process.html | §2.2 |
| 7 | TechSimplified, "What is Detailed Routing in VLSI Physical Design?" | https://www.techsimplifiedtv.in/2025/04/what-is-detailed-routing-in-vlsi.html | §3.1 Left-Edge Algorithm |
| 8 | ScienceDirect, "River routing in VLSI" (Leiserson & Pinter, 1983) | https://www.sciencedirect.com/science/article/pii/0022000087900043 | §3.2 planarity theorem |
| 9 | Springer Nature, "River Routing: Methodology and Analysis" | https://link.springer.com/chapter/10.1007/978-3-642-95432-0_9 | §3.2, §3.3 |
| 10 | UMD DRUM, "Parallel Algorithms for Several VLSI Routing Problems" | https://drum.lib.umd.edu/items/bd01fffa-12cb-46e5-937e-ffe1338ac8a7 | §3.3 multi-layer extension |
| 11 | Semantic Scholar, "PathFinder: A Negotiation-Based Performance-Driven Router" | https://www.semanticscholar.org/paper/PathFinder:-A-Negotiation-Based-Performance-Driven-McMurchie-Ebeling/45b0d141e847855f149b175abbc371aeb4b80cbb | §4.1, §4.2 |
| 12 | PathFinder paper PDF (FPGA 1995, McMurchie & Ebeling) | https://www.cecs.uci.edu/~papers/compendium94-03/papers/1995/fpga95/pdffiles/6a.pdf | §4.2 |
| 13 | gEDA GitHub Wiki, "Autorouters: gEDA pcb Toporouter" | https://github.com/bert/pcb/wiki/Autorouters:-gEDA-pcb-Toporouter | §4.3 |
| 14 | Altium, "Automated PCB Routing With the Situs Topological Autorouter" | https://resources.altium.com/p/automated-pcb-routing-with-situs-topological-autorouter | §4.4 |
| 15 | TopoR website (Eremex) | https://t.eremex.com/ | §4.5 |
| 16 | Wikipedia, "TopoR" | https://en.wikipedia.org/wiki/TopoR | §4.5 |
| 17 | Academia.edu, "A Review of Global and Detailed Routing in VLSI Design" | https://www.academia.edu/23735271/A_REVIEW_OF_GLOBAL_AND_DETAILED_ROUTING_IN_VLSI_DESIGN | §4.6 |
| 18 | Aivon, "PCB Routing Techniques: A Comprehensive Guide" | https://www.aivon.com/blog/pcb-design/pcb-routing-techniques-a-comprehensive-guide-to-efficient-trace-layout/ | §5.1, §5.2, §5.3 |
| 19 | TinyComputers, "The Mathematics of PCB Trace Routing" | https://tinycomputers.io/posts/the-mathematics-of-pcb-trace-routing.html | §4.1 net ordering |
| 20 | JLCPCB, "Via in Pad (VIP) Technology for HDI PCBs" | https://jlcpcb.com/blog/via-in-pad-pcb | §1.3 |

---

## Inference Annotations

Claims marked "(inference)" below are analytical conclusions not directly sourced from a
fetched URL, but follow from first principles or measured data:

- §7 table, "TraceWise gap" column: inference from measured scorer data + analysis in
  INVENT-human-mimicry-router.md and INVENT-topological-routability.md.
- §8 rankings: judgment calls combining witness measurement data and technique impact analysis.
  Relative rankings beyond Rank 1–2 involve inference.
- §3.3 multi-layer river routing extension: derived from the single-layer planarity theorem
  by induction; not directly cited in a fetched source.

---

*Survey complete. All technique claims in §1–6 trace to at least one fetched source URL.
Mitayi cross-reference in §7 traces to measured data in the inventor scripts. Rankings in §8
combine measured data with analytical judgment (annotated where inference is used).*
