# Design: Simultaneous Global Corridor Optimizer (GCO)

Status: DESIGN, 2026-06-24. NO production code in this doc — illustrative signatures only.
Target: beat attempt-3 (mitayi 41 unc / 73 err) on BOTH axes toward the human 0/0, by
allocating the shared J4 corridor to ALL contending nets AT ONCE instead of sequentially.

---

## Overview

Four reordering experiments (TOPOLOGY-CLASS-ROUTING.md, frontier table) proved the residual
41→0 gap is NOT capacity and NOT homotopy-realizability — both are solved (0 crossings
achievable; ~2x channel slack). The gap is that our router is **sequential**: it picks an
ORDER for the contending nets in the shared J4 corridor (+3V3 power spine vs the QFN-escape
signal nets), and every order trades one axis for the other. The Pareto frontier is mapped and
NONE of its points dominates attempt-3:

| config | unc | err | character |
|---|---|---|---|
| ring_slots + power-first | 30 | 113 | connectivity-extreme |
| escape (illegal-fallback vias) | 40 | 79 | connectivity, dirty |
| **attempt-3 (gridless-first negotiate)** | **41** | **73** | **BEST BALANCED** |
| ring_slots (legal vias) | 46 | 65 | error-extreme |

Each AXIS is individually reachable (30 unc; 65 err) but never together. The human routes
+3V3 AND the escape nets through the corridor SIMULTANEOUSLY. The missing primitive: a SINGLE
global multi-commodity allocation of the corridor's B.Cu lanes × layers to all contending nets
at once. This VINDICATES Inventor-2's min-cost multi-commodity flow proposal — deferred when we
believed mitayi was single-commodity (TCR §flow_vs_monotone picked monotone packing because
"mitayi is a single dominant channel"). The frontier proves that was wrong: +3V3 is a SECOND
commodity contending for the SAME channel. The single-commodity monotone packer cannot model
the +3V3↔escape contention, which is exactly why every sequential order trades axes.

**Approach selection vs prior art:** `pattern_find`/`kb_search` on global topology assignment
both returned empty (genuinely new surface — confirmed in TCR). vs the in-repo TCR monotone
packer: GCO EXTENDS it — it keeps TCR's ring-slot assignment (topo_assign.py, committed) and
its `route_net_steered` realizer, and ADDS the multi-commodity lane/layer flow that TCR
explicitly deferred. vs Inventor-2's proposal: GCO APPLIES the min-cost multi-commodity flow
Inventor-2 proposed, now with the +3V3 spine as a first-class commodity (Inventor-2 modeled
only escape nets). vs the rejected gEDA full toporouter: still NOT a board-wide CDT — the flow
graph is the single J4 corridor (~46 lanes × 2 layers), combinatorially small (~34 commodities).

**Reuse / new split:** ~75% reuse (topo_assign ring slots, route_net_steered lane realizer,
the probe harness, refill_zones/run_drc, _check_rss bounded guards), ~25% new (the corridor
flow graph builder + the min-cost-flow solve + the spine-as-commodity model).

---

## Scope

- **Files to create (GCO-spike — standalone, throwaway-acceptable):**
  - `scripts/_probe_gco_spike.py` — the decisive spike. Builds the corridor flow graph for the
    contending set (+3V3 + ~8 J4-corridor escape nets), solves the simultaneous lane/layer
    assignment ONCE via networkx min-cost-flow, realizes via `route_net_steered` (escape) +
    `route_all(gridless_first=...)` (the +3V3 spine taps), grids the rest, scores vs attempt-3.
    Reuses `topo_assign`, `route_net_steered`, and the `_probe_tcr_e1.py` scoring/harness.
- **Files to create (production — ONLY after GCO-spike GO):**
  - `src/tracewise/route/gridless/corridor_flow.py` — the new primitive:
    `build_corridor_graph`, `solve_corridor_assignment` (min-cost multi-commodity flow on the
    lane×layer graph, spine + signal commodities), `corridor_assignment_to_classes` (flow →
    per-net `escape_via_xy`/`lane_y_mm`/spine-band, the dict shape topo_assign already emits).
  - `tests/test_corridor_flow.py`.
- **Files to modify (production — ONLY after GCO-1 green):**
  - `src/tracewise/route/engine/multi.py` `route_all` — add `corridor: dict | None = None`
    opt-in path mirroring the existing `gridless_first`/`coopt` pattern. Default `None` ⇒
    byte-identical.
  - `src/tracewise/route/engine/kicad.py` — thread the `corridor` flag (mirror `coopt`).
- **Files to read (context only):**
  - `docs/design/TOPOLOGY-CLASS-ROUTING.md` (frontier + ring-slot + lane infra)
  - `docs/research/INVENT-topological-routability.md` (the now-vindicated min-cost-flow proposal)
  - `docs/research/HUMAN-ROUTING-TECHNIQUES.md` (river/channel/left-edge theory)
  - `src/tracewise/route/gridless/topo_assign.py` (ring slots — reused verbatim)
  - `src/tracewise/route/gridless/route.py` `route_net_steered` (lines 1282+, the realizer)
  - `src/tracewise/route/engine/multi.py` `_run_coopt_loop`/`_check_rss` (lines 290–550, guards)
  - `scripts/_probe_tcr_e1.py` (the harness + CORRIDOR_POWER_NETS + scoring to reuse)
  - `data/benchmark-boards/mitayi-pico-d1/Mitayi-Pico-D1.kicad_pcb` (the witness/feasibility cert)

---

## 1. Corridor model

The shared J4 corridor = the west channel U3 → J3/J4 (TCR Rule H4). Modeled as a discrete
**lane × layer grid with capacity**, exactly the resource the frontier shows is contended:

- **Lanes (B.Cu horizontal tracks).** West channel J4 (y≈81.2) → J3 (y≈97.4) = 16.2mm tall;
  at pitch `track_mm + clearance_mm = 0.35mm` → `⌊16.2/0.35⌋ = 46 lanes` (Inventor-1 measured,
  INVENT-human-mimicry §c). Each lane = a reserved horizontal y-band `lane_y_mm ± pitch/2`.
  This is the same `lane_y_mm` `route_net_steered` already enforces (route.py Option L,
  river-routing, crossing-free by construction).
- **Layers.** 2 (F.Cu=0, B.Cu=1). Per Rule H3 the horizontal bus is B.Cu (67% horizontal
  copper); F.Cu carries the vertical escape stubs + short F.Cu-only nets. Modeled as 2 parallel
  lane-banks; a lane on B.Cu and the same y-band on F.Cu are DISTINCT capacity slots.
- **Capacity per lane = 1 net's horizontal run.** Two nets may share a lane only if their
  x-spans are disjoint (1-D interval; deferred to GCO-1 — the spike treats lanes as unit-cap).

### Commodities

Two commodity CLASSES contend for the corridor, modeled differently because their topology
differs fundamentally:

**(a) Signal escape nets (single source → single dest).** Each QFN-escape net is one commodity:
source = QFN F.Cu pad, dest = J3/J4 connector pad. Its corridor DEMAND = one B.Cu lane (the
horizontal run from escape via to the dest y-band) + one ring via slot (F.Cu→B.Cu transition).
~8 nets share the J4 corridor specifically (/RUN, /SWCLK, /GPIO9, /GPIO20, /GPIO3, /GPIO4,
/GPIO6, /GPIO23 — measured in INVENT-topological W3, the 5.2–6.7mm primary ring + GPIO9 at
8.2/10.8mm). Source/dest already extracted by topo_assign `build_ring_slot_assignment`.

**(b) The +3V3 power spine (multi-pin, NOT source→dest).** See §2.

---

## 2. Multi-pin power handling

+3V3 is a **29–30-pad net** (verified: 30 pad-refs in the .kicad_pcb), +1V1 = 6 pads. These are
NOT a single source→dest commodity — they are a **spine/comb**: a power trunk with many taps to
loads. Modeling +3V3 as one flow commodity with 30 sinks would be wrong (it would minimize total
flow length, not respect the comb structure) and the frontier shows routing it as a single
gridless MST first (probe Phase 0) blew errors to 113.

**Model: reserved spine-band + taps.**

- The +3V3 spine occupies a **contiguous BAND of adjacent lanes** (not one lane) — a power
  trunk is wide and runs the length of the corridor. The band is modeled as a single commodity
  in the flow with **band-width demand B_spine** (number of adjacent lanes the trunk needs;
  start B_spine = 2, the trunk + return room — calibrate in the spike). The flow allocates the
  band as a contiguous lane-interval at one END of the lane stack (top or bottom), so the
  signal commodities pack into the remaining lanes monotonically — this is the left-edge /
  channel-routing discipline (HUMAN-ROUTING §3.1): the power trunk is the widest "net," seated
  first at the channel edge, signals fill inward.
- **Taps** (the 29 load pads) are NOT routed by the flow. They are short local stubs from the
  reserved spine band to each load pad, realized AFTER the band is fixed, via the existing
  `route_all(gridless_first={+3V3})` path with the spine band's lanes pre-marked as +3V3 copper
  (so the taps attach to the trunk, not re-route the trunk). The flow's job is ONLY to reserve
  the band so the signal lanes and the spine band are DISJOINT by construction — which is the
  exact disjointness the sequential orders never achieved simultaneously.
- **Why a band, not a lane:** the frontier's "power-first" config (30/113) proves +3V3 NEEDS
  corridor copper (its blocked-count dropped 30→2 when routed first) but consuming it
  sequentially starves the escapes. Reserving a contiguous band SIMULTANEOUSLY with the signal
  lanes lets both fit iff `B_spine + N_signals_in_corridor ≤ 46` — and 2 + 8 = 10 ≪ 46, so the
  capacity proof says a disjoint co-allocation EXISTS (the witness is the certificate, §7).

---

## 3. The simultaneous global assignment (the core)

**Formulation: min-cost multi-commodity flow on the corridor lane×layer graph, solved ONCE for
all contending commodities (spine + signals) together.** (assignment_formulation = flow.)

### Why flow, not ILP, not the TCR monotone packer

- **vs TCR monotone packer (single-commodity):** the packer assumes one commodity class
  (signals) and a free channel. It CANNOT represent the spine as a competing demand — which is
  precisely the modeling gap the frontier exposed. Flow models both commodity classes on one
  shared capacitated graph. This is the specific extension the frontier mandates.
- **vs ILP:** an ILP (binary net→lane assignment) is more expressive but needs a solver (pulp /
  scipy.milp) — NEITHER is installed (verified: no scipy, no pulp). Adding an ILP solver
  violates Principle 2 (stdlib over deps). Min-cost flow is available in **networkx 3.6.1
  (verified installed)** — zero new dependency. And the corridor problem is a TRANSPORTATION
  problem (commodities → lane slots with per-slot capacity 1 and per-assignment cost), which is
  exactly min-cost flow's sweet spot; it does NOT need ILP's full expressiveness.
- **vs general multi-commodity flow (NP-hard):** true integral multi-commodity flow is NP-hard
  (Inventor-2 flagged this). We SIDESTEP it: the corridor is a SINGLE channel, so each signal
  commodity's path is trivial (escape via → its lane → dest); the only decision is WHICH
  (lane, layer) slot each commodity gets. That collapses the multi-commodity flow to a
  **single-commodity min-cost flow on a bipartite assignment graph** (commodities ↔ lane slots),
  which IS polynomial and integral (the constraint matrix is a transportation polytope — totally
  unimodular → LP optimum is integral, no rounding). The spine band is one "fat" commodity
  consuming B_spine adjacent slots, handled by a contiguity gadget (below).

### The graph

```
SOURCE ──(cap=1, cost=0)──> commodity_node[net]  for each signal net
SOURCE ──(cap=B_spine, cost=0)──> spine_node       the +3V3 band commodity

commodity_node[net] ──(cap=1, cost=C(net,slot))──> slot_node[lane,layer]
                                                    for each legal (lane,layer)
spine_node          ──(cap=1, cost=C_spine(slot))──> slot_node[lane,layer]
                                                    restricted to a contiguous edge band

slot_node[lane,layer] ──(cap=1, cost=0)──> SINK     each lane slot holds ≤1 net
```

- **Cost `C(net, slot)`** = (a) horizontal-run length proxy `|lane_y - dest_y(net)|` (lanes near
  the dest cost less → short runs, the river-routing monotone preference) + (b) layer penalty
  (B.Cu cheap for horizontal per Rule H3; F.Cu penalized) + (c) escape-stub length to the ring
  slot serving that lane. Integer costs (×1000, snapped) for determinism.
- **Spine contiguity gadget:** the spine must take B_spine ADJACENT lanes at one channel edge.
  Model as B_spine unit-edges from `spine_node` to the slot_nodes of a candidate edge-band, and
  enumerate the (small, ~4) candidate bands (top-2-lanes, bottom-2-lanes, …) as parallel
  alternatives — pick the min-cost feasible one. With only ~4 candidate bands × B_spine≈2 this
  is trivial; no general contiguity ILP needed.
- **Crossing-freedom** is INHERITED, not solved: assigning each signal a distinct lane and
  realizing via `route_net_steered` Option L (horizontal run strictly at `lane_y_mm`) makes the
  bus planar by construction (river-routing) — TCR already proved 0 crossings. The flow only
  decides WHICH lane; the realizer guarantees no-cross.

### Objective

Minimize total assignment cost subject to: every commodity served (all contending nets get a
lane+slot → connectivity), each lane slot used ≤ once (disjoint → 0 shorts/crossings), spine
band contiguous (power trunk realizable). Minimizing cost ⇒ short runs ⇒ fewer clearance/length
errors. The witness certifies the served-all-disjoint constraint set is FEASIBLE (§7).

---

## 4. Solver + tractability/bounded argument

**Solver: `networkx.algorithms.flow.min_cost_flow` (network simplex), built-in, no new dep.**
(verified networkx 3.6.1 in the venv.)

- **Tractability.** Graph size: ~34 commodity nodes + (46 lanes × 2 layers = 92 slot nodes) + 2
  terminals ≈ 130 nodes, ~34×92 + spine edges ≈ 3200 edges. networkx network-simplex on a graph
  this small solves in **< 50ms**. Integral by total-unimodularity (transportation polytope) →
  no rounding, no repair. This is the "combinatorially-small" claim: ~34 commodities, ~92 slots,
  one solve.
- **Determinism.** Integer costs (snapped ×1000). networkx network-simplex is deterministic for
  a fixed graph; we further sort all node/edge insertions by a fixed key (net name, then
  (lane_idx, layer)) so the graph build is order-stable. Tie-break costs by a deterministic
  epsilon on (lane_idx, layer) so no two edges have equal cost → unique optimum. 3-run
  byte-identical assignment is an EXIT criterion.
- **Bounded — if the solve could be slow/large.** It cannot blow up (130-node graph), but the
  guard is explicit anyway: wrap the solve in a wall-clock budget (`SOLVE_TIMEOUT_S = 10`); on
  timeout fall back to the TCR monotone packer's assignment (best-so-far, already committed
  infra) — NEVER unbounded. The graph is CAPPED at the corridor (no full-board nodes). Peak RSS
  of the solve itself is < 50MB (a 130-node networkx graph). See §5 for the realization bound,
  which is where prior builds actually blew up.

---

## 5. Bounded-runtime + determinism strategy (NON-NEGOTIABLE)

Prior builds blew to 4–18GB from BOARD-WIDE visibility graphs. GCO's blowup surface is the
REALIZATION, not the solve. Every guard below is ALREADY in the engine and REUSED. Each maps to
a specific prior blowup:

| Guard | Reused from | Prior blowup it prevents |
|---|---|---|
| `_check_rss(label)` per net, abort `MemoryError` if RSS > 2GB | `_run_coopt_loop` (multi.py:333, `rss_hard_fail_gb`) | the 4–18GB board-wide visgraph blowups |
| `max_window_mm=12` (F.Cu stub + via), `max_bcu_window_mm=8` (B.Cu run) | `route_net_steered` defaults (route.py:1297) | unbounded per-net window → O(n²) visgraph |
| Lane band `lane_y_mm ± pitch` shrinks the B.Cu window | `route_net_steered` Option L (route.py:1533) | wandering B.Cu runs that cross lanes |
| `skip_full_corner_fallback=True` when extra_obstacles large | `route_net_gridless` (route.py:98), already set in steered F.Cu branch | the 6000-vertex full-corner fallback → 4–5GB |
| Region-scope: graph + realization ONLY on the J4 corridor + U3 ring | `_run_coopt_loop` `region_bbox` (multi.py:357) | board-wide search; rest is grid-routed unchanged |
| `SOLVE_TIMEOUT_S=10` → fall back to monotone packer | NEW (this design) | a pathological flow solve |

- **Determinism.** Flow assignment is integer-cost + fixed insertion order + unique-optimum
  tie-break (§4). Realization is the existing deterministic visgraph A* (1nm-snapped). 3-run
  byte-identical (connectivity AND board hash modulo zone-fill) at every milestone.

---

## 6. Realization

The flow assignment → fixed per-net `(escape_via_xy, lane_y_mm)` + the spine band → realize via
the EXISTING machinery, legal-by-construction:

1. **Reserve the spine band.** Mark the B_spine contiguous lanes the flow assigned to +3V3 as
   reserved y-bands. Route +3V3 (trunk + taps) via `route_all(gridless_first={+3V3,+1V1})` with
   the band lanes available and the signal lanes pre-marked as obstacles → the trunk runs in its
   band, taps attach locally. (Reuses the probe's Phase-0 power path, but now the band is
   CO-ASSIGNED with the signals, not greedily grabbed.)
2. **Realize the signal escapes.** For each signal commodity, take its flow-assigned ring slot
   (`escape_via_xy`, from topo_assign's legal slots) + lane (`lane_y_mm`) and call
   `route_net_steered(source_xy, dest_xy, escape_via_xy, lane_y_mm, …)`. Option L routes the
   horizontal run strictly at `lane_y_mm` → disjoint lanes → 0 crossings. Accumulate each net's
   copper into `bcu_extra_obstacles`/`fcu_stub_extra_obstacles` (probe already does this).
3. **Grid/gridless_first the rest.** All non-corridor nets route via the existing
   `route_all(gridless_first=…)` (the attempt-3 mechanism that produced the 73-error baseline —
   so GCO inherits attempt-3's error profile on the non-corridor nets, then IMPROVES the
   corridor nets via the simultaneous allocation).

Legal-by-construction: disjoint lanes (flow capacity 1/slot) + legal ring slots (topo_assign
`is_legal_via` filter) + spine band disjoint from signal lanes (flow). The `route_net_steered`
fallback path (route.py:1400, tight 10° radial) handles any slot the model missed.

---

## Interface Contracts

```python
# src/tracewise/route/gridless/corridor_flow.py  (NEW — the GCO primitive)

@dataclass(frozen=True)
class CorridorSlot:
    lane_idx: int           # 0..N_lanes-1, monotone in y
    lane_y_mm: float        # the reserved horizontal lane y
    layer: int              # 0=F.Cu, 1=B.Cu

def build_corridor_graph(
    contending_signals: dict[str, dict],   # net -> {source_xy, dest_xy, order_key, ...}
                                            #   (the topo_assign escape_nets_info shape)
    spine_nets: dict[str, int],            # {"+3V3": B_spine, "+1V1": 1} band-width demand
    ring_slots: dict,                       # from topo_assign.generate_ring_slots
    channel_y_range: tuple[float, float],  # (J4_y, J3_y) = lane stack extent
    geo: dict,
) -> "networkx.DiGraph":
    """Build the capacitated lane x layer flow graph (SOURCE -> commodities ->
    slot_nodes -> SINK).  Integer costs (x1000, snapped).  Deterministic node/edge
    insertion order (sorted by net name, then (lane_idx, layer))."""

def solve_corridor_assignment(
    graph: "networkx.DiGraph",
    solve_timeout_s: float = 10.0,
) -> dict[str, "CorridorSlot"]:
    """Run networkx min_cost_flow ONCE; decode flow -> per-commodity slot.
    On timeout -> raise CorridorTimeout (caller falls back to topo_assign monotone packer).
    Deterministic: integer costs + unique-optimum tie-break."""

def corridor_assignment_to_classes(
    assignment: dict[str, "CorridorSlot"],
    ring_slots: dict,
    contending_signals: dict[str, dict],
    spine_band_lanes: dict[str, list[float]],   # +3V3 -> [lane_y, ...] reserved band
) -> dict[str, dict]:
    """Map the flow assignment to the dict shape topo_assign/route_net_steered consume:
    {net: {escape_via_xy, lane_y_mm, dest_xy, source_xy, order_key}} for signals,
    plus the spine band as a reserved-lane list for the power realization step."""

# Opt-in wiring (mirror coopt/gridless_first exactly; default None => byte-identical):
def route_all(..., corridor: dict | None = None, ...): ...
```

REUSED verbatim: `topo_assign.generate_ring_slots`, `topo_assign.build_ring_slot_assignment`
(slot legality), `route_net_steered` (signal realization), `route_all(gridless_first=…)` (spine
+ rest), `_check_rss` (guard), `refill_zones`/`run_drc` (scoring).

---

## 7. Feasibility vs the witness

**YES — the witness certifies a clean simultaneous assignment EXISTS.** The human board
(Mitayi-Pico-D1.kicad_pcb) routes +3V3 AND the J4-corridor escape nets in the SAME corridor with
0 crossings, 0 shorts (INVENT-topological W1: 0 different-net crossings on 426 F.Cu + 157 B.Cu
segs). That IS a feasible co-allocation of the spine band + the signal lanes — the exact object
the flow searches for. Capacity: B_spine(≈2) + N_signals_in_J4(≈8) = 10 ≪ 46 lanes → the
disjoint-co-allocation polytope is non-empty with large slack. The flow cannot fail to find a
served-all-disjoint assignment when one provably exists and capacity has 4x headroom.

**Honest limit:** the witness certifies a FEASIBLE assignment exists; it does NOT certify the
flow's MIN-COST assignment realizes to ≤73 errors on the non-corridor nets (those route via the
unchanged attempt-3 mechanism). GCO's claim is bounded: it makes the corridor nets connect AND
stay short-free SIMULTANEOUSLY (the frontier's two halves), inheriting attempt-3's profile
elsewhere. Whether that nets out below 41/73 is what the spike MEASURES — not asserted.

---

## Staging

Each milestone has a scorecard exit criterion. STOP at any gate that fails.

### GCO-spike — the decisive experiment (BUILD FIRST)
- `scripts/_probe_gco_spike.py`: build corridor graph for the contending set (+3V3 + ~8 J4
  escape nets), solve the simultaneous assignment ONCE, realize (spine band → +3V3 via
  gridless_first; signals via route_net_steered), grid the rest, score vs attempt-3.
- **Exit (GO):** `unconnected ≤ 40 AND errors ≤ 73 AND 0 shorts AND 0 illegal crossings AND
  deterministic (3-run identical connectivity) AND bounded (peak RSS < 2GB, no blowup)`.
  FAIL ⇒ NO-GO; attempt-3 (41/73) remains best; record WHICH commodity failed and WHY.

### GCO-1 — engine integration (only if spike GO)
- Build `corridor_flow.py` production (graph + solve + decode). Wire `corridor` opt-in into
  `multi.route_all` (default None ⇒ byte-identical). Add lane-sharing (1-D interval pack, two
  signals per lane if x-disjoint) + the spine band-width calibration.
- **Exit:** corridor-cluster realization scores `unconnected ≤ 40 AND errors ≤ 73 AND 0 shorts
  AND 0 crossings`, deterministic, RSS < 2GB. Default-OFF regression byte-identical to attempt-3.

### GCO-2 — full board, beat 41/73 (only if GCO-1 GO)
- Run the full `_probe_route_human.py`-style scorecard with `corridor` enabled across the whole
  board (extend the contending set to all corridor-sharing nets if measurement warrants).
- **Exit:** full-board `unconnected < 41 AND errors < 73 AND 0 illegal crossings`, deterministic
  (3-run byte-identical), bounded (RSS < 2GB, runtime < 2× grid `--quality`). Stretch: toward
  0/0. Default-OFF (`corridor=None`) byte-identical to attempt-3.

---

## GCO-spike spec (SMALL + decisive + BOUNDED)

**The smallest experiment validating that SIMULTANEOUS corridor allocation combines the two
frontier halves (connectivity-40 of `escape` + errors-65 of `ring_slots`) into one config.**

- **Contending-set selection.** The J4-corridor sharers:
  `+3V3` (29-pin spine) + `+1V1` (small spine) + the ~8 J4-escape signals
  `{/RUN, /SWCLK, /GPIO9, /GPIO20, /GPIO3, /GPIO4, /GPIO6, /GPIO23}` (INVENT-topological W3
  primary ring + GPIO9). Selection rule, deterministic: a net is "contending" iff its dest pad
  is on J4 (y ≈ 80.4 inner row) OR it is a CORRIDOR_POWER_NET (the probe's existing set). Everything
  else routes via the unchanged attempt-3 path.
- **Solver invocation.** `build_corridor_graph(contending_signals, {"+3V3":2,"+1V1":1},
  ring_slots, (J4_y, J3_y), geo)` → `solve_corridor_assignment(graph, solve_timeout_s=10)` →
  `corridor_assignment_to_classes(...)`. ONE solve, all commodities together. networkx
  min_cost_flow. Assert the assignment is disjoint (no two signals share a lane; spine band
  disjoint from signal lanes) before realizing.
- **Pass criteria (ALL):** `unconnected ≤ 40` AND `errors ≤ 73` AND `0 shorting_items` AND
  `0 illegal different-net crossings (F.Cu + B.Cu)` AND deterministic (3-run identical
  connectivity) AND bounded (peak RSS < 2GB via `/usr/bin/time`, no blowup, solve < 10s).
- **Validates:** that a SINGLE simultaneous allocation seats the +3V3 spine band AND the J4
  escape lanes DISJOINTLY → connectivity AND short-freedom together, which no sequential order
  achieved. I.e. that the flone-vs-frontier thesis (the gap is sequential ordering) is correct
  and the multi-commodity allocation closes it.
- **Defers:** lane-sharing (1-D interval pack — spike uses unit-cap lanes); spine band-width
  auto-calibration (spike hardcodes B_spine=2, tune if it fails); full-board contending set
  (spike is J4-corridor only); engine integration / opt-in wiring; non-J4 corridors (J5/J1 east).
- **Standalone:** reuses `topo_assign` (slots) + `route_net_steered` (signals) +
  `_probe_tcr_e1` scoring/harness + the Phase-0 power path. New code = the corridor graph + solve
  (~150 LOC). BOUNDED windows mandatory (max_window_mm=12, max_bcu_window_mm=8, _check_rss 2GB).

---

## Risks and Mitigations

| Risk | Likelihood | Impact | Mitigation |
|---|---|---|---|
| Flow assignment feasible but a lane LOCALLY blocked (pour/drill) → escape unrealizable | Med | High | spike MEASURES real realizability; topo_assign filters slots on both layers; per-net route_net_steered radial fallback; `_check_rss` |
| Spine band-width B_spine=2 too narrow/wide (+3V3 trunk needs more/less) | Med | Med | spike tries B_spine ∈ {2,3}; band is the calibration knob; frontier shows +3V3 needs corridor copper, magnitude is the unknown |
| Min-cost assignment short but non-corridor nets still 73 err (no net win) | Med | High | spike measures full scorecard; GCO only claims to combine the two corridor halves, not improve elsewhere — honest bound (§7) |
| networkx min_cost_flow non-integral / non-deterministic | Low | High | transportation polytope is totally unimodular → integral; integer costs + unique-optimum tie-break; 3-run gate |
| Multi-commodity NP-hardness recurrence | Low | Med | collapsed to single-channel bipartite assignment (§3); NOT general multi-commodity; spine = contiguity gadget over ~4 candidate bands |
| RSS blowup recurrence in realization | Low | High | bounded windows + `_check_rss` 2GB hard abort + region-scope + skip_full_corner_fallback (§5) |
| Tap realization (29 +3V3 pads) slow/incorrect | Med | Med | taps via existing gridless_first power path (probe Phase-0, proven bounded 937MB); band pre-reserved so taps are local |
| GCO is a 5th Pareto point, not a dominating win | Med | High | this is THE risk the spike decides; if it ties not beats, NO-GO, attempt-3 stays — the simultaneous formulation is the only untried lever |

---

## Testing Strategy

- **GCO-spike (integration gate):** the full scorecard + crossing audit (reuse `_invent2_topology`
  §4 `seg_intersect`). Pass/fail gates the build.
- **Unit — `build_corridor_graph`:** deterministic graph (same input ⇒ identical edge set/costs);
  integer costs; spine band edges restricted to edge-bands; signal commodities have one edge per
  legal slot.
- **Unit — `solve_corridor_assignment`:** assignment is disjoint (no two commodities → same
  slot); spine band contiguous; integral flow; timeout → raises (caller falls back); 3-run
  identical assignment.
- **Fixture — witness feasibility:** the witness's +3V3-band + J4-escape lanes form a feasible
  point in the flow polytope (assert the constraint set is satisfiable on witness geometry).
- **Regression — default-OFF:** `corridor=None` ⇒ byte-identical board to attempt-3.
- **Crossing-audit regression:** GCO output 0 different-net same-layer crossings (permanent gate).
- **Determinism:** 3-run byte-identical connectivity at spike, GCO-1, GCO-2.

---

## Acceptance Rubric

- [ ] **AC1 — design doc complete.** `docs/design/GLOBAL-CORRIDOR-OPTIMIZER.md` contains:
  corridor model, multi-pin-power handling, simultaneous-assignment formulation + solver
  (tractability/determinism/bounded), realization, staging, GCO-spike spec, feasibility-vs-witness.
  Evidence: section headers §1–§7 + staging + spike spec present.
- [ ] **AC2 — corridor model is lanes×layers with capacity.** Doc specifies 46 B.Cu lanes at
  0.35mm pitch × 2 layers, per-lane capacity, and how signals vs spine map to demands.
  Evidence: §1.
- [ ] **AC3 — multi-pin power handled as spine band + taps, NOT a source→dest commodity.** Doc
  specifies the contiguous reserved lane-band (B_spine demand) co-assigned with signals, taps
  realized separately. Evidence: §2.
- [ ] **AC4 — assignment is ONE simultaneous solve over all commodities.** Doc gives the flow
  graph (SOURCE→commodities→slots→SINK), the spine contiguity gadget, the objective, and states
  it solves spine+signals TOGETHER. Evidence: §3.
- [ ] **AC5 — solver chosen + tractability/determinism/bounded argued.** Doc picks networkx
  min_cost_flow (no new dep, verified installed), argues TU-integrality + ~130-node tractability
  + integer-cost determinism + 10s timeout fallback. Evidence: §4.
- [ ] **AC6 — bounded/determinism guards mapped to prior blowups.** Doc names `_check_rss` 2GB,
  `max_window_mm`/`max_bcu_window_mm`, lane band, skip_full_corner_fallback, region-scope, solve
  timeout — each mapped to a specific prior blowup. Evidence: §5 table.
- [ ] **AC7 — realization reuses route_net_steered + gridless_first, legal-by-construction.** Doc
  specifies spine band → +3V3 gridless_first, signals → route_net_steered Option L (disjoint
  lanes), rest → attempt-3 path. Evidence: §6.
- [ ] **AC8 — feasibility vs witness stated honestly.** Doc states the witness certifies a
  feasible co-allocation EXISTS (cap 10≪46) but does NOT certify the full ≤73-err result.
  Evidence: §7.
- [ ] **AC9 — staging has scorecard exit criteria.** Three milestones (spike → GCO-1 → GCO-2),
  each with numeric/binary exit. Evidence: §Staging.
- [ ] **AC10 — GCO-spike-first, no production code before GO.** The developer task list puts the
  standalone spike FIRST, no production `corridor_flow.py`/wiring before spike GO. Evidence:
  structured result `gco_spike_developer_tasks`.
- [ ] **AC11 — no production code in the design.** Only illustrative signatures/pseudo-code.
  Evidence: no runnable function bodies.

---

## GCO-spike (2026-06-24) — flow MATH validated; realization broke it; NO-GO (the persistent wall)

`scripts/_probe_gco_spike.py` (PM-verified; probe-only, suite 424 green, ruff clean).

**Flow phase: SUCCESS (the core hypothesis is mathematically validated).** `min_cost_flow` found a
DISJOINT corridor assignment (+3V3 spine band lanes 1-2, +1V1 lane 3, /GPIO20 lane 0, /RUN lane 4,
/GPIO23 lane 5) in **0.002s** (107 nodes, 417 edges), bounded (664MB). Simultaneous allocation CAN seat
the spine + signal lanes disjointly — exactly what no sequential order did.

**Realization phase: BROKE IT → NO-GO.** Result: unc=38 (better than 41!) / **err=104** (vs 73) / **4
shorts** / 0 crossings. 2 of 3 J4 signals (/RUN, /GPIO20) failed `bcu_run_failed` — the Phase-0 power
routing had ALREADY placed copper in the lane-y bands the flow assigned them, so `route_net_steered`
couldn't reach the assigned lane. Two fixable gaps: (a) the flow assigns lanes in an idealized model
BEFORE Phase-0 power copper exists → must derive lane_y AFTER power placement (or model power copper in
the flow); (b) the J4-only contending filter caught just 3 signals (the J3 corridor nets were excluded).

**THE PERSISTENT WALL (across all ~24 iterations — the decisive lesson):** whether the assignment is
sequential (escape/ring_slots/power-first) or simultaneous-on-paper (GCO flow), the per-net GEOMETRIC
REALIZATION in the dense shared corridor keeps breaking — a net can't always reach its assigned lane
when other copper is present, and the fallback produces shorts/errors OR the net fails (unc). The
assignment computes cleanly every time; the REALIZER (place one net at a time into an exact lane) lacks
the mutual geometric awareness to honor all assignments simultaneously. The human routes all corridor
nets together with full concurrent geometric awareness — that is the gap, and it is a CONCURRENT
DETAILED ROUTER for the corridor (route all assigned nets' geometry simultaneously, not one-at-a-time),
a much larger undertaking that keeps hitting this same realization wall.

**5th frontier point: GCO = 38/104.** Frontier: power-first 30/113 | GCO 38/104 | escape 40/79 |
attempt-3 41/73 | ring_slots 46/65. Still NONE dominates attempt-3. **attempt-3 (commit 9ae76ea, 41/73)
REMAINS THE DEFINITIVE BEST.** The GCO flow infrastructure (min-cost corridor allocation, validated +
bounded) is committed for a future concurrent realizer.
