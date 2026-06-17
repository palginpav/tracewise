# Research: F3 — High-Fanout Power Net (+3V0) on a 2-Layer GND-Contended Board

**Status:** complete  
**Date:** 2026-06-17  
**Scope:** External prior-art survey for TraceWise F3 (zuluscsi +3V0, 58 pads, no source pour, 56 of 84 DRC unconnected)

---

## Problem Framing

**Goal:** Connect a ~58-pad, no-pour power net (+3V0) on a 2-layer board where GND already has
copper pours spanning both layers, using TraceWise's existing grid/A* engine with minimal new
machinery.

**Hard constraints:**
1. Must work on a 2-layer board (F.Cu + B.Cu only); no inner power planes available.
2. GND already holds pours on both layers — the competing fill owns the majority of free copper area.
3. Must not regress mitayi or pic_programmer routing results (TraceWise gate condition).
4. Must be deterministic (reproducible runs under `taskset -c 0-9`).
5. Improvement target: +3V0 56 → ≤15 unconnected.

**Soft constraints:**
1. Prefer reusing the existing zone machinery (`pcbnew` ZONE_FILLER / `eccf.py` `build_field`) over new code.
2. Prefer approaches with staged delivery (smallest first step first).
3. Low additional router time budget consumption (zuluscsi already runs ~530s of a 600s cap).

**Known-not-wanted:**
- Full inner power plane (requires ≥4 layers; out of scope).
- Redesigning the source KiCad board (TraceWise is an autorouter, not a redesign tool).

---

## Landscape of Approaches

| # | Approach | How It Works | Fit for 2-Layer + GND-Both-Sides | Effort | Source |
|---|----------|-------------|----------------------------------|--------|--------|
| 1 | **Auto-generated partial power pour (lower-priority zone)** | Add a +3V0 copper zone on one or both layers; KiCad zone-fill priority lets lower-priority zones occupy copper GND doesn't own. All pads touching the fill become connected. Zone arbitration is rule-based: higher-priority zone fills first, lower-priority keeps a DRC clearance to it. | HIGH — KiCad's zone fill engine already handles GND vs power arbitration natively via priority levels. Works even on both layers simultaneously. Gaps remain where GND fully occupies all free area (typical near dense GND pads). | Medium — must compute a convex-hull or board-outline polygon for +3V0, set priority below GND's zones, call `refill_zones`. F0 pour-geometry must be complete first. | [KiCad PCB Editor docs §Zones](https://docs.kicad.org/8.0/en/pcbnew/pcbnew.html); [KiCad zone priority issue](https://gitlab.com/kicad/code/kicad/-/work_items/23790) |
| 2 | **Power trunk/bus + short pad stubs (F-shape or spine routing)** | A wide trace ("trunk") runs from the power entry toward the centroid of pad clusters; thin branch stubs connect nearby pads. Human designers describe this as an "F-shape" for 3.3V on 2-layer boards. Route-power-first ordering ensures trunk claims preferred corridor before signal nets block it. | MEDIUM — avoids the pour-arbitration problem entirely; works in any free routing area. On a heavily GND-poured board the trunk must navigate GND islands but a single trunk needs only 1 clear path. Diminishing returns past ~30 pads because branch stubs must each find a route. | Low-Medium — TraceWise already routes +3V0 first (POWER regex match), but the current A* point-to-point tree does exactly this and only completes 2 of 58. The problem is likely congestion after the trunk uses up the corridor, leaving pads on the far side unreachable. | [Hackster PCB Friday — F-shape power bus](https://www.hackster.io/news/pcb-friday-two-layer-pcb-routing-strategies-and-tips-1c6b8cfcf5d3); [Big Mess o' Wires autorouting](https://www.bigmessowires.com/2019/03/27/pcb-autorouting/) |
| 3 | **Hybrid: partial pour covering a defined board sub-region, then stub-stitch outlier pads** | Draw a +3V0 zone outline over the board region that concentrates most +3V0 pads. Let zone fill claim that sub-region. Pads inside the fill connect automatically (pcbnew). Pads outside the region (outliers) are stitched by short traces routed after fill. | HIGH — this is the standard professional practice for "mixed" power on a 2-layer board. It concentrates pour where pads are dense, avoids fighting GND over the whole board, and leaves trace routing only for outlier pads. Mirrors the GND F2 approach (pour-stitch). | Medium — requires: (a) bounding-box or cluster analysis of +3V0 pads to define zone outlines; (b) fill call; (c) identify which pads are still isolated post-fill; (d) trace-stitch those pads. Steps (c)+(d) are exactly the F2 pour-stitching machinery. | [allpcb.com PDN 2-layer best practices](https://www.allpcb.com/allelectrohub/power-distribution-network-design-in-double-layer-pcbs-best-practices); [Altium Situs power+ground router](https://resources.altium.com/p/automated-pcb-routing-with-situs-topological-autorouter) |
| 4 | **High-fanout net decomposition (subnet partitioning)** | Split the 58-pad net into spatial subnets (clusters) and route each cluster's Steiner tree independently, then chain them. Classic academic approach for IC routing; reduces A* search space per sub-problem. | LOW-MEDIUM — the improvement from decomposition is wire-length / routability in dense IC layers. On a 2-layer PCB the bottleneck is corridor congestion and the GND pour blocking, not search-space size. Decomposition does not resolve the copper-availability problem. | High — significant algorithmic change to `route_net` / `route_all`; no existing hook. | [US Patent 6154874 — high fanout net partitioning](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/6154874); [Multicommodity flow routing](https://arxiv.org/pdf/cs/0508045) |
| 5 | **Route power last (minimize blockage from signal traces)** | Reverse ordering so signal nets route first and leave corridors clear for power trunks. Counter-intuitive but sometimes used when the power net is distributed and the signal corridors are the bottleneck. | LOW — TraceWise's POWER regex puts +3V0 first precisely because power nets need to block corridors. Routing power last on a GND-poured board would likely make it worse (signal traces block the remaining corridors). | Low (trivial to test) — but current PLAN.md and multi.py already reason about why power-first is correct. | [multi.py `order_nets` docstring](../../../src/tracewise/route/engine/multi.py) |
| 6 | **Expand-contract / shape-based router (e.g., Freerouting, Altium push-and-shove)** | Freerouting uses a shape-based approach that expands pad shapes and contracts copper outlines to avoid obstacles, better than grid A* at fitting wide traces in constrained corridors. | MEDIUM in theory — Freerouting (the reference external router TraceWise benchmarks against) outperformed by TraceWise on zuluscsi by ~30% on completion (per PLAN.md). However, Freerouting's advantage is signal nets; it does not expose a "power-first + zone-pour" pipeline. Importing this approach into TraceWise is a large architectural change. | Very High — replacing or augmenting the A* engine. Out of scope for F3 as a standalone fix. | [Freerouting GitHub](https://github.com/freerouting/freerouting); [PLAN.md A* vs FR comparison](../PLAN.md) |

---

## Shortlist (Top 2)

### Top Pick: Approach 3 — Hybrid Partial Pour + Stub-Stitch

**Why:**  
The 2-layer + GND-both-sides constraint makes approach 2 (trace-only) structurally difficult: the A*
router already routes +3V0 first and completes only 2/58. This failure is not ordering — it is copper
starvation. After GND pour rasterizes into the routing grid, most of both layers is already marked
hard-blocked. A +3V0 pour, defined over the board sub-region where +3V0 pads cluster, occupies
whatever copper GND doesn't own in that region. KiCad's zone fill priority mechanism handles the
arbitration automatically: give +3V0 a lower priority number than GND's zones and `refill_zones`
will fill GND first, then fill +3V0 into leftover copper. The result is that every pad inside or
adjacent to the +3V0 fill region connects to the fill without needing a trace route at all. The
remaining outlier pads (pads far from the fill's coverage) then become a small trace-stitch problem
— identical in structure to the F2 GND stitch already being built.

This hybrid is also the human designer's standard practice for a secondary power rail on a 2-layer
board with a GND pour: draw the power polygon, set priority, fill, then hand-route any stragglers.

**Caveats:**  
- Zone fill arbitration depends on the +3V0 pad cluster covering a region that GND doesn't fully
  saturate. If GND pour occupies 100% of copper on both layers in the pad cluster area, the +3V0
  zone will generate zero fill and the stubs will be as stuck as before. This is the honest ceiling
  (see Risks below).
- The zone polygon shape matters: a board-outline polygon is the broadest option; a convex hull
  of +3V0 pads is more targeted. A board-outline zone will fight GND more aggressively (more copper
  claimed), which is usually better for connectivity.
- F0 must deliver robust zone-geometry extraction before F3 can synthesize a new zone robustly.
- The zone fill call (`refill_zones` via pcbnew) is already in the codebase (F2 path); reuse it
  directly.

**First-run cost signal:** 2–4 day implementation spike after F0 lands. Core logic: enumerate +3V0
pads, compute board-outline zone polygon (or convex hull), add zone with priority below GND, call
refill, re-read connectivity, identify still-isolated pads, invoke F2-style stub routing on them.

---

### Runner-up: Approach 2 — Power Trunk/Bus Trace Routing (Improved Ordering / Trunk-First)

**Why:**  
If the partial pour is infeasible (e.g., the pad cluster area is fully GND-saturated and the fill
generates zero copper), a trace-only approach may still improve. The key insight from the "F-shape"
practice: route a single wide trunk trace first, from the power entry toward the dense pad cluster,
before marking it into the grid. All pads within reach of short stubs from that trunk can then
connect. This is exactly the current point-to-point tree the router builds — the issue is that after
the first few pads are connected, the trunk cells are marked as hard obstacles for via placement,
and the tree cannot grow into regions blocked by GND copper.

A targeted improvement: widen the +3V0 trunk trace allowance (treat its cells as passable by its
own subsequent stubs), or route +3V0 with an explicit two-pass strategy (backbone first, stubs
second). This requires modifying `route_net` to support a "trunk-then-branch" mode, which is more
invasive than adding a zone.

**Caveats:** More architectural change than approach 3, and does not resolve the copper-starvation
root cause. Better as a fallback or complement to the pour.

**First-run cost signal:** 3–5 days to prototype; uncertain gain unless pour approach ceiling is hit.

---

## Recommendation

**Verdict: (c) Hybrid — partial +3V0 pour on sub-region where GND doesn't dominate, then F2-style
stub stitch for pads outside the pour.**

Reasoning chain tied to the specific constraint:

1. The root cause of 56 unconnected is not routing heuristic quality — the router is already placing
   +3V0 first and succeeding at only 2/58. The grid cells are occupied by GND pour copper
   (hard-blocked by `grid.hard`) before +3V0 gets any A* expansion.
2. A pour bypasses A* entirely for covered pads. `refill_zones` connects any pad whose pad hole
   or thermal relief touches the fill; no trace routing needed, no congestion contention.
3. KiCad's zone priority mechanism provides exactly the arbitration needed: GND zones stay at their
   current priority; a new +3V0 zone gets a lower priority number (higher integer = lower precedence
   in KiCad's convention), and the fill engine leaves GND copper intact while claiming residual space
   for +3V0.
4. The stub-stitch residual (pads outside the fill's reach) mirrors F2's already-designed mechanism
   exactly. F3 can extend F2's pour-stitching to a second fill net rather than building from scratch.

---

## Staged Implementation Path

### Stage 1 (smallest step, highest leverage): Board-outline +3V0 zone + refill

**Pre-condition:** F0 robust zone extraction delivered.

**What to do:**
- Programmatically add a KiCad copper zone on F.Cu (and optionally B.Cu) with:
  - net = "+3V0"
  - outline = board bounding rectangle (or board edge polygon)
  - priority = 1 (below GND's default priority 0 — NOTE: check KiCad's exact convention; higher
    integer may mean higher OR lower priority depending on version; verify against KiCad 8 docs)
  - clearance = project clearance rule
  - thermal relief = solid (no spokes; power rail, not signal)
- Call `refill_zones` (pcbnew ZONE_FILLER).
- Re-read pad connectivity using F0's API.
- Report how many of 58 +3V0 pads are now connected via fill.
- Commit no routing changes yet — measure pure fill contribution.

**Expected outcome:** Depending on GND saturation, 15–40 of 58 pads may connect via fill alone.
This is the cheapest measurable step and its result determines whether the pour approach pays.

**Risk gate:** If fewer than 10 pads connect via fill, the board is GND-saturated in the +3V0
cluster and the pour approach ceiling is low. Fall back to Stage 2b.

### Stage 2a (if Stage 1 pays): Stub-stitch isolated outliers

Reuse F2's stub-routing logic: for each +3V0 pad still isolated post-fill, find nearest fill cell
(using the same vectorized-distance heuristic as F2) and route a short stub. This directly maps
to the F2 architecture with a different target net.

**Expected outcome:** Combined (fill + stubs) should reach ≤15 unconnected.

### Stage 2b (if Stage 1 ceiling is low): Trunk routing improvement

If the fill yields < 10 connections, investigate whether a +3V0 trunk trace on the layer with
lower GND occupancy (check F.Cu vs B.Cu fill density before deciding) plus explicit stub branching
gives better results. This is a routing-only fallback that does not depend on pour.

### Stage 3 (optional): Both-layer fill + via-stitching between layers

Add the +3V0 zone to the second layer as well, and stitch the two fills with vias at points where
neither layer is GND-saturated. This maximizes coverage on heavily-poured boards.

---

## Risks / Honest Ceiling

1. **GND full saturation in the +3V0 cluster area.** If GND pours cover nearly 100% of copper on
   both layers in the region where +3V0 pads concentrate, the lower-priority +3V0 zone will fill to
   nearly zero copper. Probability: moderate. The PLAN.md notes GND has "85-poly pour spanning F&B"
   — this is a large pour. Measurement after Stage 1 will reveal whether a ceiling exists.

2. **Zone arbitration ordering (priority convention).** KiCad's zone priority direction (higher
   integer = higher or lower precedence) differs across versions and is easy to set backwards.
   Setting +3V0 priority higher than GND would make +3V0 fill first and expel GND copper, causing
   DRC floods on GND. Must verify convention on KiCad 8 before writing code.

3. **Net name fragility (F0 known risk).** ROUTING-COMPLETION-PLAN.md notes that zone `net_name`
   parses correctly on source board but returns NONE on pcbnew-resaved board. Adding a zone
   programmatically via pcbnew API (not s-expr parsing) should avoid this — the API sets net by
   NETINFO object, not string. But F0 must confirm this.

4. **Pour geometry on the stub-stitch step.** The stub route must start from the isolated pad and
   reach a pour cell, not another pad. Using `eccf.py`'s `build_field` seeded from fill cells
   (already how F2 works) handles this correctly.

5. **Time budget.** The `refill_zones` call happens outside the A* routing loop (it is a pcbnew
   API call, not a Python A* iteration). It does not consume TraceWise's 600s route budget. This
   is a strict improvement to the pipeline timing.

6. **Real ceiling estimate.** Even with perfect fill + stub stitching, pads that are physically
   isolated (surrounded by GND copper on both layers with no clearance gap to route a stub through)
   cannot be connected without a design change. On a real board this is rare but possible for 1–3
   pads. A realistic floor is ≈5–8 still-unconnected, not zero.

---

## Honest Gaps

- **Freerouting power net source code**: the Freerouting GitHub README does not document how it
  handles power nets internally. A full code read was not within scope; the README only mentions
  `--ignore-net-classes`. Freerouting is not used as a reference for the recommended approach.
- **KiCad forum zone priority thread**: returned HTTP 403 on fetch. Relied on official KiCad 8
  PCB Editor docs instead.
- **Screaming Circuits "Route or Plane" article**: certificate error; excluded from table.
- **Academic EDA literature on power routing**: no directly applicable paper found that addresses
  the specific 2-layer + competing-pour scenario. The Steiner tree decomposition literature
  (patents 6154874, 6247167) addresses IC-level high-fanout but not PCB-level copper-pour
  interaction.
- **Known-not-wanted pre-excluded**: Approach 5 (route power last) included in table for
  completeness but is already contra-indicated by the existing `order_nets` logic.

---

## Recommended Next Agent

**Architect** — design the integration of the hybrid pour + stub approach into the TraceWise
pipeline. Specifically:
1. Design the zone-synthesis step (programmatic zone creation via pcbnew API, priority assignment).
2. Design how the isolated-pad identification post-fill feeds into the F2 stub-route machinery.
3. Produce interface contracts for the F3 module callable from `route_board_engine`.

Inject this landscape table into the architect delegation prompt under
`## Landscape Survey (from Researcher)`.

---

## Sources

- [KiCad PCB Editor 8.0 Documentation](https://docs.kicad.org/8.0/en/pcbnew/pcbnew.html)
- [KiCad zone overlap issue #23790](https://gitlab.com/kicad/code/kicad/-/work_items/23790)
- [Hackster.io — Two-Layer PCB Routing Strategies (F-shape power bus)](https://www.hackster.io/news/pcb-friday-two-layer-pcb-routing-strategies-and-tips-1c6b8cfcf5d3)
- [Big Mess o' Wires — PCB Autorouting (manual power pre-routing observation)](https://www.bigmessowires.com/2019/03/27/pcb-autorouting/)
- [allpcb.com — PDN Design in Double Layer PCBs Best Practices](https://www.allpcb.com/allelectrohub/power-distribution-network-design-in-double-layer-pcbs-best-practices)
- [Altium Situs Autorouter (power+ground router reference)](https://resources.altium.com/p/automated-pcb-routing-with-situs-topological-autorouter)
- [Freerouting GitHub](https://github.com/freerouting/freerouting)
- [US Patent 6154874 — High fanout net partitioning / subnet decomposition](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/6154874)
- [Altium Understanding Ground Planes on Two-Layer PCBs](https://resources.altium.com/p/understanding-ground-planes-your-two-layer-pcb)
