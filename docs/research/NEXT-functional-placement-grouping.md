# Research: Functional Placement Grouping for TraceWise's Analytical Placer

## Problem Framing

**Goal:** Identify and implement explicit functional/semantic clustering of related
components (IC + decoupling caps, crystal + load caps, regulator + output caps, connector
+ ESD parts) so the gradient-descent HPWL placer places them as tight sub-circuit units,
reducing routing congestion on the residual unconnected floor (43 of 65 zuluscsi residuals
are "router-recoverable" = congestion-blocked, not structurally impossible).

**Hard constraints:**
1. Must work in pure Python/PyTorch without native extensions (CPU-only env).
2. Must integrate into `build_problem` / `optimize` without breaking the existing
   gradient graph or locked-part semantics.
3. Must not require schematic hierarchy or LLM pass at runtime — netlist is the sole
   input (PCB JSON from extract.py).
4. Measurable benefit must be validated by routing-in-the-loop (not placement score alone),
   per the project's standing finding that local placement metrics do not predict routability.

**Soft constraints:**
1. Minimal added dependency (networkx is fine; scipy is borderline; no GPU deps).
2. Incremental: smallest-first step (fix decap assignment) then fuller clustering.
3. Must remain backward-compatible with the from-scratch and the human-placement
   refinement modes.

**Known-not-wanted:**
- Hard hierarchical partitioning with fixed cluster centroids (makes global HPWL
  optimization rigid and increases congestion at cluster boundaries — see §4).
- GPU-only or native-compiled approaches (DREAMPlace, ePlace) — out of scope for a
  small-board CPU placer.
- GNN-based learned placement (TransPlace etc.) — training data requirements not met.

---

## Candidate Approaches

| # | Approach | Source / Provenance | Fit Score (0–5) | Fit Rationale | Known Risks | Cost / Complexity | Verdict |
|---|----------|---------------------|-----------------|---------------|-------------|-------------------|---------|
| 1 | **Improved decap-to-IC assignment: closest-power-pin heuristic** | IPC-2221C rule + Altium/Sierra Circuits guidelines (protoexpress.com/blog/decoupling-capacitor-placement-guidelines-pcb-design) | 5/5 | Fixes the most concrete bug in `build_problem`: current code binds every C* on a non-GND net to `others[0]` (first part in iteration order, not the nearest IC power pin). Correct rule: for each C* on net N, find the non-cap part that has the most "power-ish" pad on N (pad named VCC/VDD/VBUS/3V3/5V or a supply pin); if multiple ICs share net N, bind to the IC whose supply pad is positionally closest to the cap's centroid in the initial layout. Fits entirely within `build_problem.decap` construction. | Positional proximity in `pos0` depends on initial placement; on a board with no prior placement the heuristic degrades to "first IC on the net". Mitigate: use pin connectivity degree as tie-breaker (IC with more pins on net N = likely the primary consumer). | Low — 15–30 lines replacing the `decap` loop in `build_problem`. | **Recommend** |
| 2 | **Soft cluster attraction via virtual net (centroid anchor)** | Timing-driven net weighting in DREAMPlace 4.0 (ieeexplore.ieee.org/document/10032162); analytical cluster cohesion in PPA-relevant clustering (vlsicad.ucsd.edu/Publications/Conferences/410/c410.pdf); virtual anchor pseudo-nets for preplaced blocks in NTUplace3 (cc.ee.ntu.edu.tw/~ywchang/Papers/tcad08-NTUplace.pdf) | 4/5 | Add a soft attraction term pulling each cluster member toward the cluster centroid. Concretely: for each identified functional group G = {anchor IC, decap1, decap2, crystal, load_cap1, …} compute a virtual centroid as the mean of initial positions, then add a penalty `w_cluster * sum((pos[i] - centroid_virtual) ** 2)` for i in G. Centroid can be re-computed each optimizer step from the movable members (not fixed) — this keeps it differentiable and avoids the rigid-centroid problem. Weight `w_cluster` follows the same annealing schedule as `w_decap`. Integrates in `optimize` as a new penalty term alongside `decap_penalty`. | Differentiable centroid re-computation adds O(num_clusters × cluster_size) work per step — negligible. Risk: over-attraction collapses clusters into overlapping blobs; mitigate by annealing `w_cluster` together with overlap weight (enforce spread before applying cluster attraction). | Low–Medium — ~40 lines for `cluster_penalty` + group construction in `build_problem`. | **Recommend** |
| 3 | **Louvain / modularity community detection on the net hypergraph** | python-louvain library (python-louvain.readthedocs.io); modularity-based clustering for placement (sciencedirect.com/science/article/abs/pii/S0167926019305711) — shows Louvain quality better than hMetis on placement clusters, 4.5× faster | 3/5 | Convert the netlist hypergraph to a 2-section weighted graph: for each net with k pins, create edges between all pairs with weight 1/(k−1) (clique expansion). Run Louvain community detection to produce clusters without any semantic labeling. Feed detected community IDs into `build_problem` as cluster group definitions. Works best for larger boards with clear sub-circuit topology. Requires `pip install python-louvain networkx`. | Community detection sees only connectivity, not electronics semantics (a decap shared between two ICs may land in the wrong community). Also: Louvain is stochastic (random tie-breaking), so different runs may produce different communities — problematic for reproducibility. Mitigate: fix random seed. Over-clustering of power nets (GND/VCC connect everything) is a real failure mode — must exclude GND/VSS/power-global nets from the 2-section graph (exactly as the current `decap` loop already excludes GND). | Medium — 60–80 lines for hypergraph construction + Louvain call + cluster extraction. Two new deps. | Consider |
| 4 | **Rule-based sub-circuit pattern matching (crystal, regulator, ESD)** | IPC-7711 assembly rules; Altium room automation docs (altium.com/documentation/knowledge-base/altium-designer/creating-user-defined-component-classes-and-rooms); Sierra Circuits placement guidelines (protoexpress.com/blog/how-to-place-components-kicad) | 4/5 | Identify canonical sub-circuit patterns directly from ref+net structure: (a) **crystal cluster** — Yx (crystal ref), any Cx sharing both XTAL nets with Yx, any Rx on those nets; (b) **regulator cluster** — any U* with pads named EN/FB/ADJ/SHDN + Cx sharing the output net; (c) **connector ESD cluster** — Dx sharing a net with a Jx connector pad; (d) **decap cluster** — Cx sharing a single non-GND power net with a Ux (already partially handled). Each pattern produces a group definition fed to approach #2. No ML, no new dep. | Patterns are brittle to non-standard ref conventions (e.g., crystal labeled X1 not Y1, or U vs IC prefix). Mitigate: match by KiCad component value keywords (e.g., "crystal", "LDO", "TVS") from footprint value field, which extract.py already has. Needs extend to extract.py to emit the `value` field. | Low–Medium — 50–70 lines for pattern matcher; +5 lines to emit `value` from extract.py. | **Recommend** |
| 5 | **Multilevel hierarchical partitioning (mPL / hMetis style)** | mPL6 (researchgate.net/publication/227258178_mPL6_Enhanced_Multilevel_Mixed-Size_Placement_with_Congestion_Control); NTUplace3 multilevel global placement (cc.ee.ntu.edu.tw/~ywchang/Papers/tcad08-NTUplace.pdf) | 2/5 | Partition netlist into coarse clusters using min-cut (hMetis / kahypar), place cluster centroids globally, then place parts within each cluster. Academic standard for large VLSI netlists (>100k cells). On a 50–200 part PCB the problem is already small; adding a coarsening hierarchy introduces complexity without proportional benefit. kahypar has C++ native deps violating hard constraint #1. hmetis is GPL + binary-only. | Violates HC#1 (native deps). Over-engineering for board-scale parts count. Cluster boundary artifacts increase congestion at borders. | High — native deps, multi-stage algorithm, significant refactor. | **Reject** |
| 6 | **Learned/GNN-based placement (TransPlace, DREAMPlace)** | TransPlace (arxiv.org/pdf/2501.05667); DREAMPlace 4.0 (github.com/limbo018/DREAMPlace) | 1/5 | State-of-the-art for large VLSI, but requires GPU, training data (thousands of boards), and significant native C++ ops. DREAMPlace does support per-net weighting (`.wts` files) which is analogous to approach #2, but the full framework is not portable to this torch CPU placer. | GPU required; training data unavailable; native build chain. Not portable as a drop-in. | Very High | **Reject** |

---

## Shortlist

### Top Pick: Approach #1 + #2 + #4 — Fix decap assignment → add rule-based groups → add soft cluster attraction

The correct sequencing is to combine three complementary steps, each adding incrementally:

**Step A — Fix decap-to-IC assignment (Approach #1)**

The `build_problem.decap` loop currently binds every cap on net N to `others[0]`, which is iteration-order-dependent and often wrong (e.g., a bulk cap for a regulator output also shares a net with multiple ICs and gets arbitrarily assigned). The correct heuristic:

1. Identify "power pin" pads: pad whose net name ends in VCC, VDD, VBUS, 3V3, 5V, AVCC,
   DVDD, VIN, VOUT, or whose net name is a single supply rail (no slash prefix).
2. Among all non-cap parts sharing net N, select the one whose supply pad is closest in
   initial `pos0` to the cap's initial position. Tie-break: highest count of pins on net N
   (the IC most tightly connected to that rail).
3. Attract the cap to that pad (as today), not to `others[0]`.

This is a pure fix in `build_problem` (~20 lines). It costs nothing in the optimizer and removes a known wrong pull.

**Step B — Rule-based sub-circuit group extraction (Approach #4)**

Extend `extract.py` to emit each footprint's `value` field (one line: `"value": fp.GetValue()`). Then add a `build_groups` function in `core.py` that identifies:
- Crystal clusters: ref matches `Y*` or value contains "MHz"/"kHz"/"crystal" + caps sharing both XTAL pins.
- Regulator clusters: value contains "LDO"/"regulator"/"AMS"/"LM3x"/"XC6" or ref pattern `VR*`/`PS*` + caps on output net.
- Connector ESD: ref matches `D*` sharing a net with a `J*` connector.
- Generic decap cluster: the corrected Step A assignment already handles this.

Each group is a list of part indices. Groups feed directly into Step C.

**Step C — Soft cluster attraction penalty (Approach #2)**

Add `cluster_penalty(pos, groups)` to `core.py`:
```python
# groups: list[list[int]] — indices of co-grouped parts
def cluster_penalty(pos, groups):
    total = pos.new_zeros(())
    for g in groups:
        if len(g) < 2: continue
        centroid = pos[g].mean(0)  # differentiable, moves with members
        total = total + ((pos[g] - centroid) ** 2).sum()
    return total
```

Wire into `optimize` with weight `w_cluster * t` (annealed on with overlap). Suggested
starting weight: `w_cluster = 0.05` (same order as `w_decap`). The annealing ensures
parts can still spread in the first half of optimization (overlap removal), then cluster
cohesion tightens in the second half.

**Why:** This three-step sequence is:
- Entirely within the existing `build_problem` + `optimize` structure.
- Backward-compatible (groups=[] → cluster_penalty = 0, original behavior).
- Independently measurable: Step A alone can be tested by comparing decap penalty final
  values; Steps B+C require routing-in-the-loop (see §Measurement).
- No new dependencies beyond what is already imported.

**Caveats:**
- The differentiable centroid (`pos[g].mean(0)`) re-computes every step — this is correct
  and keeps all members contributing to the pull, but it means the centroid "chases" the
  heaviest/most constrained member. If the anchor IC is locked (connector or locked ref),
  the centroid is fixed, which is ideal.
- Over-clustering can happen if the weight is too high or the group too large. Keep groups
  to the direct sub-circuit (IC + its immediate decaps/crystal-caps), not the whole board.
- The congestion penalty's current failure to improve routability (measured: adding it
  yielded no gain) suggests congestion is genuinely structural, not optimizer-responsive.
  Functional clustering attacks the structural cause (wrong part positions) rather than
  the penalty signal.

**First-run cost signal:** 2–3 day implementation (Step A: half day; Step B: 1 day including
extending extract.py; Step C: half day including integration and sweep of `w_cluster`).
Routing validation on mitayi + zuluscsi is 1–2 hours per test point.

---

### Runner-up: Approach #3 — Louvain community detection

**Why:** Handles any sub-circuit topology automatically without explicit pattern rules —
useful for designs where ref naming is non-standard. On a 50–200 part board, Louvain runs
in milliseconds. The 2-section graph construction is clean Python.

**Caveats:** Requires fixing the power-net exclusion problem (GND/VCC are "super-hubs" that
connect all parts, collapsing the community structure into one big cluster unless excluded).
Stochastic community boundaries reduce reproducibility — fix by seeding. Main weakness:
communities are connectivity-only; a cap on a net shared by three ICs may cluster with the
wrong IC. Approach #1 is more targeted for decoupling specifically.

**First-run cost signal:** "2-3 day integration" — mainly the hypergraph-to-graph conversion
and tuning the exclusion list for power nets and the resolution parameter.

---

## How Each Step Plugs Into core.py

```
build_problem():
  → decap loop: replace others[0] with closest-supply-pad heuristic      [Step A]
  → call build_groups(fps, refs, by_net) → returns List[List[int]]        [Step B]
  → store as PlaceProblem.groups: list[list[int]]                          [Step B]

optimize():
  → cost += w_cluster * t * cluster_penalty(pos, prob.groups)             [Step C]
```

`PlaceProblem` needs one new field: `groups: list[list[int]] = field(default_factory=list)`.

---

## Measurement Protocol

Per the project's standing finding (PLAN.md: HPWL-only routed WORSE than human; congestion
term added no routability gain; local metrics mislead), every grouping change MUST be
validated by routing-in-the-loop:

1. **Baseline:** current `tracewise place --from-scratch` on mitayi → route → record
   (unconnected, violations). Must be deterministic (mitayi IS deterministic per PLAN.md
   since router completes in 196s < 600s budget).
2. **Step A only:** apply the fixed decap assignment, re-place, re-route, compare.
   Expected signal: marginal (decap penalty weight is 0.02 — very small). Main value is
   correctness, not measurable gain.
3. **Steps A+B+C:** apply full grouping, sweep `w_cluster` ∈ {0.02, 0.05, 0.1, 0.2},
   re-route each. The correct weight is the one that maximally reduces unconnected without
   increasing violations.
4. **zuluscsi:** zuluscsi is now deterministic (512/536s, unc=84 is the floor). However,
   the floor is dominated by pour-class nets (GND 22, +3V0 56 per PLAN.md) — those are
   structurally unreachable by placement changes. Only the SCSI_DB2/DB4 (3 each) are
   potentially placement-sensitive. Expect small or no gain on zuluscsi from clustering alone.
5. **mitayi is the primary test board.** Its 63-unconnected floor is described as a
   "true placement floor" — 43 of those are "router-recoverable" (congestion-blocked, not
   structurally impossible). Functional clustering that reduces local congestion around ICs
   could unlock some of these.

**Expected locus of gain:** The from-scratch placer mode. In human-placement-refinement mode,
the human's original placement already has correct functional grouping (humans instinctively
place decaps next to ICs); the analytical placer only nudges. From-scratch mode lacks any
initial functional structure and is where the grouping gap is most severe.

**Measurement caveat:** the project notes (PLAN.md 2026-06-17) that even on deterministic
mitayi, the nudge/flip arm has been measured inert — the 63-floor survived all refinement
attempts. Functional clustering is a different lever (it affects initial global placement,
not local refinement) so it is not expected to be inert, but the prior measurements set a
cautious expectation.

---

## Honest Gaps

1. **ScienceDirect paper on modularity-based placement clustering** (sciencedirect.com
   S0167926019305711) returned HTTP 403 — could not verify the Louvain vs hMetis quality
   claim beyond the search result summary.
2. **NTUplace3 full paper** PDF was binary-encoded and unreadable by WebFetch. The
   cluster-attraction mechanisms were confirmed only through search result summaries, not
   direct paper reading.
3. **Double-layer IC-module placement paper** (arxiv.org/pdf/2502.14012) was also binary PDF
   — its specific two-stage PCB placement algorithm could not be fully surveyed.
4. **TransPlace GNN PCB placement** (arxiv.org/pdf/2501.05667) — similarly binary PDF; GNN
   feature encoding for functional grouping could not be directly extracted.
5. **KiCad PCB JSON does not include schematic hierarchy** — `extract.py` only sees the flat
   PCB. Approaches that rely on hierarchical sheet names (Altium "rooms" model) are not
   available without parsing `.kicad_sch`. This is accepted; the rule-based approach (Step B)
   works entirely from refs, values, and nets in the PCB JSON.
6. **The congestion proxy** (current `congestion_penalty` in core.py) was measured to add no
   routability gain. The cluster attraction penalty proposed here operates on a different level
   (structural sub-circuit proximity, not density histogram). Whether it converts to routing
   gain is genuinely uncertain — the routing-in-the-loop measurement is not optional.
7. **Cap-value parsing** for "smallest cap closest to pin" heuristic (a secondary placement
   refinement used in high-speed design: 100nF nearer to pin than 10µF) would require
   parsing the component `value` field numerically. Not included in the recommendation as a
   first step; it is a refinement if the basic clustering already shows routing gain.

---

## Recommended Next Agent

**Developer — all hard constraints satisfied, top pick defined, integration points specified.**

Instruction for Developer: The three steps are ordered by effort and confidence. Step A
(fix decap assignment in `build_problem`) is a pure bug-fix with zero risk, implement first.
Steps B+C (rule-based groups + cluster_penalty) are new features that require routing-in-the-
loop validation on mitayi (the deterministic board). Do NOT use placement metrics (HPWL, overlap)
as the quality signal — route and count unconnected. The implementation plan is in the
"Shortlist" section above; the exact plug-in points are in the "How each step plugs into
core.py" section.
