# Design: Concurrent Detailed Corridor Router (CDR)

Status: DESIGN, 2026-06-24. NO production code in this doc — illustrative signatures only.
Target: beat attempt-3 (mitayi 41 unc / 73 err) on BOTH axes toward the human 0/0, by
**realizing ALL shared-corridor nets' geometry SIMULTANEOUSLY** instead of one-at-a-time.

---

## Overview

~24 iterations / 5 frontier points (power-first 30/113, GCO flow 38/104, escape 40/79,
attempt-3 41/73, ring_slots 46/65) PROVED one thing: the lane/slot **assignment** always
computes cleanly — the GCO `min_cost_flow` found a DISJOINT corridor assignment in **0.002s**
(GLOBAL-CORRIDOR-OPTIMIZER.md, "flow MATH validated") — but **per-net GEOMETRIC REALIZATION in
the dense shared corridor keeps breaking**. The realizer (`route_net_steered`, route.py:1282)
places ONE net at a time into its assigned lane; its B.Cu run `_seg_in_free` check (route.py:1568,
Option L) tests against `bcu_extra_obstacles` ACCUMULATED from already-placed nets (the GCO spike
accumulates `bcu_extra_obs` per net, _probe_gco_spike.py:882-893). When prior copper (Phase-0
+3V3, or an earlier escape net) occupies the assigned lane band, the net can't reach its lane →
`bcu_run_failed` (route.py:1638) → unconnected; or the fallback overlaps → shorts. In the GCO
spike, 2 of 3 J4 signals failed exactly this way (38/104, 4 shorts).

**The human routes ALL corridor nets CONCURRENTLY with full mutual geometric awareness.** This is
classic VLSI **constraint-graph channel routing** (HUMAN-ROUTING-TECHNIQUES.md §3.1, §4.6): the
shared U3↔J3/J4 corridor IS a channel — terminals on two sides (U3 escape vias feed in; J3/J4
connector pads on the far edge), nets crossing it. The channel router assigns ALL nets to
horizontal tracks SIMULTANEOUSLY, ordered by the vertical + horizontal constraint graphs, with
doglegs to break cycles. Track y-coordinates are computed JOINTLY so the runs are **disjoint and
spaced by construction** — then emitted directly as segments+vias. **No per-net windowed A*. No
`_seg_in_free` against accumulated copper. No `bcu_run_failed`.** That replacement of the per-net
realizer with concurrent direct track emission is the entire thesis.

**Approach selection vs prior art:** `pattern_find`/`kb_search` empty (genuinely new surface,
consistent with TCR + GCO). vs the GCO min-cost flow: CDR **combines** with it — the flow gives a
starting track ORDER (its 0.002s disjoint lane assignment); the constraint-graph channel router
realizes that order CONCURRENTLY with doglegs + layer assignment, replacing the realization step
the flow's `route_net_steered` broke. vs the TCR monotone packer: CDR generalizes it — monotone
lane packing IS the no-vertical-constraint special case; CDR adds the vertical constraint graph +
doglegs that handle the non-monotone subset (the GPIO9-type inversions that broke E1). vs gEDA
CDT toporouter: still NOT a board-wide triangulation — CDR is the single J4/J3 channel, ~8-34 nets,
O(n²) constraint graph, polynomial track assignment, deterministic + bounded by construction.

**Reuse / new split:** ~70% reuse (the witness-extraction `_invent2` polar/segment logic;
`topo_assign.generate_ring_slots` for via slots; the GCO `build_corridor_graph`/`solve_corridor_assignment`
for the seed track order; `to_gridless_netroute` + `emit_routes` for emission; `refill_zones`/`run_drc`/
`audit_crossings` for scoring; the `route_all(gridless_first=…)` attempt-3 path for the rest;
`_check_rss` bounded guards). ~30% new (the constraint-graph channel router: VCG + HCG build,
left-edge track assignment, dogleg cycle-break, layer assignment, direct track→segment emission).

---

## Scope

- **Files to create (CDR-spike — standalone, throwaway-acceptable, BUILD FIRST):**
  - `scripts/_probe_cdr_spike.py` — the decisive spike. Selects the FULL corridor contending set
    (+3V3 spine + J3 AND J4 escape signals, ~8-12 nets — fixing GCO-spike's too-narrow J4-only
    filter), runs the constraint-graph channel router ONCE over all of them, emits the
    tracks+doglegs+vias DIRECTLY (no `route_net_steered`), grids/`gridless_first` the rest, scores
    vs attempt-3. Reuses `_invent2` extraction, `topo_assign` slots, the GCO flow for the seed
    order, `to_gridless_netroute`+`emit_routes`, and the `_probe_gco_spike` scoring harness
    (`audit_crossings`, `refill_zones`, `run_drc`, `_check_rss`, `board_hash`).
- **Files to create (production — ONLY after CDR-spike GO):**
  - `src/tracewise/route/gridless/channel_router.py` — the new primitive:
    `build_channel_instance`, `build_constraint_graphs` (VCG + HCG), `assign_tracks_left_edge`,
    `break_cycles_with_doglegs`, `assign_layers`, `realize_channel` (track/dogleg/via →
    `world_paths`/`world_vias` per net, the dict shape `to_gridless_netroute` consumes).
  - `tests/test_channel_router.py`.
- **Files to modify (production — ONLY after CDR-1 green):**
  - `src/tracewise/route/engine/multi.py` `route_all` — add `channel: dict | None = None` opt-in
    path mirroring the existing `gridless_first`/`coopt`/`corridor` pattern. Default `None` ⇒
    byte-identical.
  - `src/tracewise/route/engine/kicad.py` — thread the `channel` flag (mirror `coopt`).
- **Files to read (context only):**
  - `docs/design/GLOBAL-CORRIDOR-OPTIMIZER.md` (the validated flow + the realization-broke-it wall)
  - `docs/design/TOPOLOGY-CLASS-ROUTING.md` (frontier, ring-slot infra, lane enforcement post-mortem)
  - `docs/research/HUMAN-ROUTING-TECHNIQUES.md` (§3 channel/river/left-edge; §4.6 global+detailed)
  - `scripts/_invent2_topology.py` (witness extraction — vias polar, `seg_intersect`, channel cut)
  - `scripts/_probe_gco_spike.py` (the harness + flow + scoring to reuse; the realizer being replaced)
  - `src/tracewise/route/gridless/topo_assign.py` (`generate_ring_slots` — via slots reused)
  - `src/tracewise/route/gridless/route.py` `route_net_steered` (the per-net realizer being REPLACED
    for the corridor — Option L:1568, `bcu_run_failed`:1638 = the wall)
  - `src/tracewise/route/gridless/adapter.py` (`to_gridless_netroute` — direct-emit path)
  - `src/tracewise/route/engine/multi.py` `_run_coopt_loop`/`_check_rss` (bounded guards)
  - `data/benchmark-boards/mitayi-pico-d1/Mitayi-Pico-D1.kicad_pcb` (the witness = a concurrent
    channel-route; the feasibility certificate)

---

## 1. Channel model

Map the U3↔J3/J4 corridor to a 2-layer channel-routing instance (HUMAN-ROUTING §3.1).

### Geometry

The west corridor is a **vertical channel**: signals escape U3 (east of the channel) through the
ring vias, run HORIZONTALLY across the channel on B.Cu, and terminate at the J3/J4 connector pads
on the far (west) edge. In channel-routing terms (rotated 90° from the textbook horizontal channel):

- **Channel axis = y** (the channel runs vertically from J4 at y≈80.4 to J3 at y≈98.2, 17.8mm).
- **Tracks = horizontal B.Cu runs at distinct y-coordinates** (the resource the frontier contends).
  Track pitch = `track_mm + clearance_mm = 0.35mm`; the 16.2-17.8mm extent yields **~46-50 lanes**
  (Inventor-1 measured 46; GCO `N_lanes`). Each track = a reserved horizontal y-band `track_y ± pitch/2`.
- **Terminals (two sides):**
  - **TOP side (the U3 ring):** each escape net's terminal = its ring via exit point
    (`escape_via_xy` from `topo_assign.generate_ring_slots`, filtered legal on both layers). The
    via x-coordinate is the net's "top-of-channel column position."
  - **BOTTOM side (the connectors):** each net's terminal = its J3/J4 connector pad
    (`dest_xy`). J4 dest pads sit at y≈80.4 (inner row), J3 at y≈98.2. The dest x is the net's
    "bottom-of-channel column position."
- **Net = one (top-terminal, bottom-terminal) pair** crossing the channel — exactly the classic
  channel-routing net. The net needs: a vertical jog down its top column → a horizontal run at its
  assigned track y → a vertical jog up/down its bottom column to the dest.

### Two layers + via transitions (the 2-layer channel / switchbox)

The classic channel router uses 2 layers: horizontal segments on ONE layer, vertical jogs on the
OTHER, with a via at every horizontal↔vertical transition. mitayi's witness uses exactly this HV
convention (HUMAN-ROUTING §2; 67% B.Cu horizontal, 80% F.Cu vertical):

- **B.Cu (layer 1) = horizontal tracks** (the channel runs). Per Rule H3.
- **F.Cu (layer 0) = vertical jogs** (the short stubs from a track to a terminal column) AND the
  escape stub from the QFN pad to the ring via.
- **Via transitions:** the escape via (QFN pad F.Cu → ring slot → B.Cu track) is the TOP-side via;
  a SECOND via where the B.Cu track jogs to the F.Cu vertical to reach the dest pad is the
  BOTTOM-side via (or the dest pad is reachable on B.Cu directly — model both, prefer the witness's
  single-via-per-net profile, _invent2 §3 shows most escapes = 1 via).
- Modeled as **2 parallel track-banks**: a track at y on B.Cu and the same y on F.Cu are DISTINCT
  capacity slots. The HCG (below) treats same-layer track overlap; the VCG forces vertical ordering.
  The +3V3 spine occupies a **reserved track BAND** at one channel edge (GCO §2 — a contiguous
  interval of adjacent tracks the power trunk needs; B_spine≈2), so the signal nets pack into the
  remaining tracks. The band is a fixed input to the channel instance (the channel router routes
  signals AROUND the reserved band), realized separately via the +3V3 gridless_first path.

### Representation

```python
# Illustrative — channel_router.py
@dataclass(frozen=True)
class ChannelTerminal:
    net: str
    x_mm: float          # column position of the terminal (top via.x or bottom dest.x)
    side: str            # 'TOP' (ring via) | 'BOT' (connector pad)

@dataclass(frozen=True)
class ChannelNet:
    net: str
    top: ChannelTerminal          # escape via column
    bot: ChannelTerminal          # dest connector column
    escape_via_xy: tuple[float, float]   # the TOP via (legal ring slot)
    x_span: tuple[float, float]   # (min, max) of {top.x, bot.x} — horizontal extent on its track

@dataclass(frozen=True)
class ChannelInstance:
    nets: list[ChannelNet]        # all contending corridor nets, sorted by net name (deterministic)
    track_ys: list[float]         # candidate track y-coordinates, monotone (the lane stack)
    spine_band: tuple[float, float]  # (y_lo, y_hi) reserved for +3V3 — signals route outside it
    pitch_mm: float               # 0.35
```

---

## 2. The concurrent routing algorithm (the core)

**Constraint-graph channel router**, the classic VLSI dogleg router (Deutsch / YACR lineage,
HUMAN-ROUTING §3.1, §4.6). Routes ALL corridor nets at ONCE. Five deterministic, polynomial steps:

### Step 1 — Build the Vertical Constraint Graph (VCG)

The VCG encodes which net MUST be on a track above which other net, from terminal columns. For
two nets i, j that share a vertical column (their terminal x's overlap within `pitch`): if net i's
TOP terminal and net j's BOT terminal are in the same column, then i must be ABOVE j on the tracks
(else the vertical jogs collide). Edge i→j means "i's track y must be < j's track y."

- Build: O(n²) pairwise column-overlap test over the n contending nets. n ≤ ~34 → ≤ ~600 pairs.
- A **cycle** in the VCG (i above j AND j above i, from interleaved terminals) is the
  non-monotone/inversion case — exactly the GPIO9-vs-GPIO14 conflict that broke TCR E1
  (`bcu_run_failed` because both wanted overlapping lanes). Step 4 breaks cycles with doglegs.

### Step 2 — Build the Horizontal Constraint Graph (HCG)

The HCG encodes which nets' horizontal runs OVERLAP in x (so they CANNOT share one track). Edge
i—j (undirected) iff `x_span(i)` ∩ `x_span(j)` ≠ ∅. Two nets may share a track ONLY if their
x-spans are disjoint (1-D interval — the lane-sharing the TCR/GCO designs deferred; CDR does it
here because it's free once you have all spans concurrently). HCG = the interval-overlap graph.

- Build: O(n²) interval-overlap test. n ≤ 34.

### Step 3 — Left-edge track assignment (Hashimoto-Stevens 1971, HUMAN-ROUTING §3.1)

Assign nets to tracks honoring BOTH constraint graphs, simultaneously for all nets:

1. Topologically order nets by the VCG (sources-first = nets that must be highest). Ties broken by
   `order_key` (= dest y, the monotone river-routing key, deterministic).
2. Sweep tracks from the channel edge inward (the +3V3 band is seated at one edge first per GCO §2,
   then signals fill inward — the left-edge discipline: widest/most-constrained net first).
3. For each track, greedily pack the next VCG-ready nets whose x-spans are disjoint (HCG-compatible)
   onto that track. A net goes on a track iff: (a) all its VCG predecessors are already on
   higher tracks, AND (b) its x-span doesn't overlap any net already on this track, AND (c) the
   track is outside the spine band.
4. Result: each net → a track y-coordinate. Provably the minimum track count when the VCG is
   acyclic (Hashimoto-Stevens optimality). With 8-12 contending nets ≪ 46 tracks, capacity has
   huge slack (the witness certifies feasibility, §7).

- O(n · tracks) interval packing. Deterministic (fixed topological order + `order_key` tie-break).

### Step 4 — Dogleg to break VCG cycles

When the VCG has a cycle (the inversion that the single-track-per-net monotone packer cannot
realize — the GPIO9 failure), introduce a **dogleg**: split the offending net's horizontal run into
TWO track segments joined by a short vertical jog (a column dogleg). This breaks the cyclic vertical
constraint by letting the net occupy a HIGHER track on the left part of the channel and a LOWER
track on the right part (or vice versa), so it can pass both conflicting neighbors. (HUMAN-ROUTING
§3.1 "doglegs to break cycles" — the standard dogleg-router result.)

- Cycle detection: Tarjan SCC on the VCG, O(V+E). For each cycle, pick the net with the widest
  x-span (most dogleg room) as the dogleg net; split at the column where its constraint flips.
- After splitting, re-run Step 3 on the augmented net set (each dogleg net is now two sub-nets with
  a vertical-jog link). Cycles are guaranteed breakable on 2 layers given the <22% B.Cu utilization
  headroom (the capacity proof; multi-layer river-routing §3.3 — any permutation realizes as a
  composition of monotone permutations with layer headroom).
- BOUNDED: at most n dogleg splits (each net split ≤ once in the spike; defer multi-split to CDR-1).
  Deterministic: SCC order + widest-span tie-break by net name.

### Step 5 — Layer assignment

Assign each horizontal run to B.Cu (the default, Rule H3) and each vertical jog/dogleg link to
F.Cu, with a via at each transition. If two nets' horizontal runs are forced onto the SAME track
y AND their x-spans overlap (an HCG conflict the left-edge packer couldn't avoid because tracks ran
out — won't happen on mitayi given the slack, but guard anyway), move one run to the F.Cu track-bank
at the same y (the 2-layer channel's second layer). The escape via (TOP) is `escape_via_xy` from the
ring-slot assignment; the dest-side transition is a via at the dest column where the B.Cu track jogs
to F.Cu (or omitted if the dest pad is B.Cu-reachable — match witness's 1-via profile).

- O(n) per-run layer pick. Deterministic.

### How CDR combines with the GCO flow (uses_gco_flow = COMBINE)

The GCO `min_cost_flow` (`build_corridor_graph`/`solve_corridor_assignment`, validated 0.002s
disjoint) provides the **seed track ORDER**: its lane-index assignment gives the initial
VCG-topological order + the +3V3 band placement. The channel router then REALIZES that order
concurrently — Step 3 left-edge packing seeds from the flow's lane indices, Step 4 doglegs handle
any inversion the flow's idealized model assigned to overlapping lanes (the exact case that broke
the flow's `route_net_steered` realization). The flow does ASSIGNMENT; CDR does concurrent
REALIZATION. The flow is NOT replaced — it is the warm-start; CDR replaces only the per-net
realizer that consumed the flow's output one-net-at-a-time. If the flow seed is unavailable
(timeout fallback), Step 3 seeds from `order_key` (dest-y monotone) directly — CDR is
self-sufficient.

---

## 3. Determinism + BOUNDED (NON-NEGOTIABLE)

CDR is **combinatorial on ~8-34 corridor nets (small)** — it ASSIGNS tracks, it does NOT pathfind
per-net. No geometric search blowup. Every step is deterministic + bounded by construction.

### Determinism

- All net iteration sorted by net name; all tie-breaks by `order_key` (= dest_y×10000 + dest_x,
  1nm-snapped, integer-keyed). No float-driven control flow (column-overlap + interval tests use
  snapped 1nm coords).
- VCG/HCG build = deterministic pairwise tests. Left-edge = deterministic topological order. SCC =
  Tarjan (deterministic given fixed adjacency order). Layer pick = fixed rule (B.Cu default).
- 3-run byte-identical (connectivity AND board hash modulo zone-fill) is an EXIT criterion at every
  milestone (reuse `board_hash`, _probe_gco_spike.py:1136).

### Bounded — guards mapped to prior 4-18GB blowups

The prior blowups came from BOARD-WIDE / per-net visibility graphs. CDR's realization is **direct
track emission** — no visgraph, no A*, no per-net window. The blowup surface is ELIMINATED, not
merely guarded. Guards retained for defense-in-depth, each mapped to a specific prior blowup:

| Guard | Source | Prior blowup it prevents |
|---|---|---|
| **NO per-net visgraph A*** — track emission is `[(via.x,via.y),(via.x,track_y),(dest.x,track_y),(dest.x,dest.y)]` direct geometry | NEW (the CDR thesis) | the per-net `route_net_steered` visgraph fallback (route.py:1606) that grew O(n²) and hit `bcu_run_failed` |
| **NO accumulated `bcu_extra_obstacles`** — tracks disjoint by construction (left-edge + HCG), so no `_seg_in_free` against prior copper | NEW | the GCO spike's per-net accumulation (gco_spike:882) that blocked later nets' lanes |
| `_check_rss(label)` per phase, abort `MemoryError` if RSS > 2GB | `_run_coopt_loop` (multi.py:333, `rss_hard_fail_gb`); spike `RSS_HARD_FAIL_GB=2.0` | the 4-18GB board-wide visgraph blowups |
| Channel instance CAPPED at the corridor (~34 nets × ~50 tracks); rest grid-routed | `_run_coopt_loop` `region_bbox` (multi.py:357) | board-wide search |
| Constraint-graph build O(n²), n≤34 → ≤600 pairs; left-edge O(n·tracks); SCC O(V+E) | NEW (combinatorially small) | a pathological geometric search |
| `CHANNEL_TIMEOUT_S=10` wall-clock on the channel solve → fall back to attempt-3 path | NEW (mirrors GCO `SOLVE_TIMEOUT_S=10`) | a pathological combinatorial blowup (cannot occur at n≤34, but guarded) |
| Remaining (non-corridor) nets via the UNCHANGED attempt-3 `route_all(gridless_first=…)` path | _probe_gco_spike.py:1056 | inherits attempt-3's bounded profile elsewhere |

Peak RSS of the channel solve itself: < 50MB (a 34-node constraint graph + interval lists). The
spike runs under `/usr/bin/time` with RSS < 2GB asserted.

---

## 4. Realization — direct track emission (replaces the per-net realizer)

**This is the key. The channel router OUTPUT (track-y per net + doglegs + vias) is emitted DIRECTLY
as `world_paths`/`world_vias`, NOT routed per-net.** (replaces_per_net_realizer.)

For each corridor net, after Steps 1-5 fix its `track_y`, doglegs, and via columns, the geometry is
KNOWN and LEGAL-by-construction:

```
world_path (per net, 3-tuples (x,y,layer)):
  [ (pad.x, pad.y, 0),            # QFN F.Cu pad (escape stub start)
    (via.x, via.y, 0),            # F.Cu stub to ring via
    (via.x, via.y, 1),            # via transition F.Cu→B.Cu  (world_vias += [(via.x,via.y)])
    (via.x, track_y, 1),          # B.Cu vertical jog into the track  (if via.y != track_y)
    (dogleg.x, track_y, 1),       # B.Cu horizontal run at the assigned track  (+ dogleg jog if any)
    (dest.x, track_y2, 1),        # (track_y2 = post-dogleg track, else == track_y)
    (dest.x, dest.y, 1) ]         # B.Cu jog to dest column→pad  (or F.Cu via if 2-via net)
```

- Each net's path is built from its FIXED track_y — the horizontal runs at distinct track y's
  NEVER cross (left-edge + HCG disjointness = river-routing planar by construction, the witness's
  0-crossing B.Cu bus). No `_seg_in_free` check against accumulated copper is needed because the
  tracks were assigned CONCURRENTLY to be disjoint. **This is why `bcu_run_failed` cannot occur**:
  there is no per-net search that can fail — the geometry is a direct consequence of the track
  assignment.
- Convert each net's `world_paths`/`world_vias` via the existing `to_gridless_netroute(net_obj,
  world_paths, grid, world_vias=…)` (adapter.py:135) → `GridlessNetRoute` → `emit_routes` (the
  zero-new-emit-code path the GCO spike already uses, gco_spike:876, 1075). `_mark(grid, nr, 1)` so
  grid-routed remaining nets see the corridor copper.
- The **spine band** (+3V3/+1V1) is realized SEPARATELY (it's a multi-pin comb, not a 2-terminal
  channel net — GCO §2): mark the reserved band tracks, route +3V3 trunk+taps via
  `route_all(gridless_first={+3V3,+1V1})` with the band tracks available and the signal tracks
  pre-marked as obstacles. The band is co-assigned with the signals (Step 3 seats it first), so the
  trunk and the signal runs are disjoint — the disjointness no sequential order achieved.
- **Remaining nets** (non-corridor) route via the UNCHANGED attempt-3 `route_all(gridless_first=…)`
  path → CDR inherits attempt-3's 73-error profile elsewhere, improving ONLY the corridor.

Legal-by-construction: distinct track y's (left-edge) + disjoint x-spans per track (HCG) + legal
ring slots (`topo_assign` `is_legal_via` filter) + spine band disjoint from signal tracks (Step 3).
The ONE residual risk is a track-y band locally blocked by a non-corridor obstacle the channel model
didn't see (a stray pad/drill in the corridor) — guarded by validating each emitted track segment
against the static board obstacles ONCE (not against accumulated copper); if a segment is blocked,
nudge the net to the next free track (bounded, deterministic) or fall it back to the grid (strictly
additive). This is NOT the per-net A* — it's a one-shot static-obstacle check on known geometry.

---

## Interface Contracts

```python
# src/tracewise/route/gridless/channel_router.py  (NEW — the CDR primitive)

@dataclass(frozen=True)
class ChannelNet:
    net: str
    top_x: float                  # ring-via column
    bot_x: float                  # dest connector column
    escape_via_xy: tuple[float, float]
    dest_xy: tuple[float, float]
    order_key: float              # dest_y*10000 + dest_x — monotone river key, deterministic

@dataclass(frozen=True)
class ChannelResult:
    net: str
    track_ys: list[float]         # one y per horizontal sub-run (>1 only if doglegged)
    dogleg_x: float | None        # column where the run jogs between track_ys (None = single track)
    layer_of_run: list[int]       # 1=B.Cu default per sub-run
    via_xys: list[tuple[float, float]]   # transition vias (escape via + any dest-side via)

def build_channel_instance(
    contending_nets: dict[str, dict],     # net -> {escape_via_xy, dest_xy, source_xy, order_key}
    spine_band: tuple[float, float],      # (y_lo, y_hi) reserved for +3V3
    channel_y_range: tuple[float, float], # (J4_y, J3_y)
    pitch_mm: float,
    flow_seed_order: list[str] | None = None,   # GCO flow lane order (warm-start) or None
) -> "ChannelInstance":
    """Deterministic: nets sorted by name; track_ys = monotone grid at pitch outside spine_band."""

def build_constraint_graphs(
    instance: "ChannelInstance",
) -> tuple["networkx.DiGraph", "networkx.Graph"]:
    """Return (VCG, HCG). VCG edge i->j = i must be above j; HCG edge i-j = x-spans overlap.
    O(n^2), n<=34. Deterministic node/edge insertion order (sorted by net name)."""

def assign_tracks_left_edge(
    instance, vcg, hcg,
) -> dict[str, "ChannelResult"]:
    """Hashimoto-Stevens left-edge: topo-order by VCG (order_key tie-break), pack disjoint
    x-spans per track from the channel edge inward, skip spine_band. Doglegs applied for VCG
    cycles (break_cycles_with_doglegs). Layer assignment (assign_layers). Returns per-net result."""

def realize_channel(
    results: dict[str, "ChannelResult"],
    contending_nets: dict[str, dict],
    geo: dict,
) -> dict[str, tuple[list, list]]:
    """Each ChannelResult -> (world_paths, world_vias) DIRECT geometry (no A*, no visgraph).
    Caller feeds to to_gridless_netroute -> emit_routes. Legal-by-construction."""

# Opt-in wiring (mirror coopt/corridor exactly; default None => byte-identical):
def route_all(..., channel: dict | None = None, ...): ...
```

REUSED verbatim: `_invent2_topology` extraction (vias polar, dest pads, `seg_intersect`),
`topo_assign.generate_ring_slots`/`build_ring_slot_assignment` (escape via slots),
GCO `build_corridor_graph`/`solve_corridor_assignment` (seed order), `to_gridless_netroute`
(adapter.py:135), `emit_routes`, `refill_zones`/`run_drc`/`audit_crossings` (scoring),
`route_all(gridless_first=…)` (the rest), `_check_rss` (guard).

---

## 5. Feasibility vs the witness

**YES — the witness IS a concurrent channel-route; it certifies the CDR output exists.** The human
board (Mitayi-Pico-D1.kicad_pcb) routes +3V3 AND the J3/J4-corridor escape nets through the SAME
corridor with **0 different-net crossings** (INVENT-topological W1, _invent2 §4: 0 crossings on 426
F.Cu + 157 B.Cu segs) and **0 shorts**. The witness's B.Cu bus is the MONOTONE GPIO0..29 ordered
horizontal-run structure (HUMAN-ROUTING §7, "B.Cu bus: Monotone GPIO0..29 ordered horizontal runs
in the west channel") — i.e. **the witness IS a left-edge channel-route with a reserved power band**.
That is precisely the object Steps 1-5 construct: distinct horizontal tracks (the GPIO runs at
distinct y's), ordered by the connector pinout (the monotone `order_key`), with the +3V3 trunk as a
band. Capacity: B_spine(≈2) + N_signals_in_corridor(≈8-12) ≪ 46 tracks → the channel has ~3-4×
headroom → the left-edge assignment cannot run out of tracks, and the dogleg cycle-break is
guaranteed feasible on 2 layers (multi-layer river-routing §3.3).

**Honest limit:** the witness certifies that a clean concurrent channel-route of the corridor EXISTS
(0 crossings, 0 shorts, feasible track assignment). It does NOT certify that CDR's track assignment +
the UNCHANGED attempt-3 routing of the non-corridor nets nets out below 41/73 — the non-corridor
profile is inherited, not improved. CDR's claim is bounded: it makes ALL corridor nets connect AND
stay short-free SIMULTANEOUSLY (closing the frontier's two halves that no sequential realizer
combined), inheriting attempt-3 elsewhere. Whether that beats 41/73 is what the spike MEASURES.

---

## Staging

Each milestone has a scorecard exit criterion. STOP at any gate that fails.

### CDR-spike — the decisive experiment (BUILD FIRST)
- `scripts/_probe_cdr_spike.py`: select the FULL corridor contending set (+3V3 + J3 AND J4 escape
  signals), run the constraint-graph channel router ONCE over all of them, emit tracks+doglegs+vias
  DIRECTLY (no `route_net_steered`), grid/`gridless_first` the rest, score vs attempt-3.
- **Exit (GO):** `unconnected ≤ 40 AND errors ≤ 73 AND 0 shorting_items AND 0 illegal different-net
  crossings (F.Cu + B.Cu) AND deterministic (3-run identical connectivity) AND bounded (peak RSS <
  2GB via /usr/bin/time, channel solve < 10s, no blowup)`. FAIL ⇒ NO-GO; attempt-3 (41/73) remains
  best; record WHICH net/track failed and WHY.

### CDR-1 — engine integration (only if spike GO)
- Build `channel_router.py` production (instance + VCG/HCG + left-edge + dogleg + layer + realize).
  Wire `channel` opt-in into `multi.route_all` (default None ⇒ byte-identical). Add multi-split
  doglegs + spine-band-width calibration + the static-obstacle track-nudge.
- **Exit:** corridor-cluster realization scores `unconnected ≤ 40 AND errors ≤ 73 AND 0 shorts AND
  0 crossings`, deterministic, RSS < 2GB. Default-OFF regression byte-identical to attempt-3.

### CDR-2 — full board, beat 41/73 (only if CDR-1 GO)
- Run the full `_probe_route_human.py`-style scorecard with `channel` enabled board-wide (extend the
  contending set to all corridor-sharing nets if measurement warrants; add the east J5/J1 channel).
- **Exit:** full-board `unconnected < 41 AND errors < 73 AND 0 illegal crossings`, deterministic
  (3-run byte-identical), bounded (RSS < 2GB, runtime < 2× grid `--quality`). Stretch: toward 0/0.
  Default-OFF (`channel=None`) byte-identical to attempt-3.

---

## CDR-spike spec (SMALL + decisive + BOUNDED)

**The smallest experiment validating that CONCURRENT geometric realization of all corridor nets
closes the frontier — eliminating the per-net `bcu_run_failed` + shorts that every prior realizer
hit.**

- **Contending-set selection (FIX the GCO too-narrow J4-only filter — full set this time).** The
  corridor sharers, deterministic rule: a net is "contending" iff its dest pad is on J3 (y≈98.2±1.5)
  OR J4 (y≈80.4±1.5) — i.e. it crosses the U3↔J3/J4 channel — OR it is a CORRIDOR_POWER_NET. This
  catches the FULL ~8-12-net set `{+3V3, +1V1}` + `{/RUN, /SWCLK, /GPIO3, /GPIO4, /GPIO6, /GPIO9,
  /GPIO20, /GPIO23, …}` (both J3 and J4 escapes), NOT just the 3 J4-only nets the GCO spike caught.
  Everything else routes via the unchanged attempt-3 path. (Reuse `QFN_ESCAPE_NETS`,
  `CORRIDOR_POWER_NETS`, `FCU_ONLY_NETS` from `_probe_gco_spike.py`; widen the dest filter to J3∪J4.)
- **Channel-router invocation.** `build_channel_instance(contending_nets, spine_band=(band_lo,
  band_hi), channel_y_range=(J4_y, J3_y), pitch_mm=0.35, flow_seed_order=<GCO flow lane order>)` →
  `build_constraint_graphs(instance)` → `assign_tracks_left_edge(instance, vcg, hcg)` →
  `realize_channel(results, …)`. ONE channel solve, all corridor nets together. Assert the
  assignment is disjoint (no two nets' horizontal runs overlap on the same track; spine band
  disjoint from signal tracks) BEFORE emitting. Emit via `to_gridless_netroute`+`emit_routes`.
  Realize +3V3/+1V1 separately via the Phase-0 gridless_first power path with the band reserved.
- **Pass criteria (ALL):** `unconnected ≤ 40` AND `errors ≤ 73` AND `0 shorting_items` AND
  `0 illegal different-net crossings (F.Cu + B.Cu)` AND deterministic (3-run identical connectivity)
  AND bounded (peak RSS < 2GB via `/usr/bin/time`, channel solve < 10s, no blowup).
- **Validates:** that CONCURRENT realization — all corridor nets' track geometry computed jointly,
  emitted directly with no per-net visgraph and no accumulated-copper `_seg_in_free` — eliminates
  the `bcu_run_failed` (no net "can't reach its lane" because tracks are disjoint by construction)
  AND the shorts (no overlapping fallback) that broke every prior realizer. I.e. the thesis (the
  wall is the per-net realizer's lack of mutual geometric awareness) is correct and concurrent
  channel routing closes it.
- **Defers:** multi-split doglegs (spike allows ≤1 split per net); spine band-width auto-calibration
  (spike hardcodes B_spine=2, tries {2,3} if it fails); the east J5/J1 channel (spike = west
  U3↔J3/J4 only); engine integration / opt-in wiring; F.Cu second-layer track-bank for HCG conflicts
  (spike relies on the slack to avoid same-track x-overlap; add only if a conflict appears).
- **Standalone:** reuses `_invent2` extraction + `topo_assign` slots + GCO flow seed + the
  `_probe_gco_spike` scoring/harness + the Phase-0 power path. New code = the channel router
  (instance + VCG/HCG + left-edge + dogleg + realize, ~250-350 LOC). **Do NOT reuse the failing
  per-net `route_net_steered` for the corridor** — emit tracks directly. BOUNDED guards mandatory
  (`_check_rss` 2GB, `CHANNEL_TIMEOUT_S=10`, no per-net visgraph).

---

## Risks and Mitigations

| Risk | Likelihood | Impact | Mitigation |
|---|---|---|---|
| Track-y band LOCALLY blocked by a non-corridor obstacle the channel model missed | Med | Med | one-shot static-obstacle check per emitted segment (NOT per-net A*); nudge to next free track (bounded) or fall back to grid (additive) |
| VCG has a cycle the single dogleg can't break (deep inversion) | Low (mitayi monotone) | Med | spike allows ≤1 split/net; CDR-1 adds multi-split; 2-layer headroom guarantees breakability (§3.3); if not, fall back to attempt-3 path |
| Concurrent realization fixes corridor but non-corridor nets still 73 err (no net win) | Med | High | spike measures FULL scorecard; CDR only claims to close the corridor's two halves, not improve elsewhere — honest bound (§5) |
| Spine band-width B_spine=2 too narrow/wide for +3V3 trunk | Med | Med | spike tries B_spine∈{2,3}; band is the calibration knob (GCO §2 same finding) |
| Dest-side via vs single-via profile mismatch → extra vias inflate errors | Low | Med | match witness's 1-via-per-net profile (_invent2 §3); prefer B.Cu-reachable dest, add dest via only when forced |
| RSS blowup recurrence | Low | High | NO per-net visgraph (the blowup surface is eliminated) + `_check_rss` 2GB + region-scope + channel solve timeout (§3) |
| Nondeterminism in track assignment | Low | High | no float control flow; net-name sort + order_key tie-break; 1nm snap; 3-run gate |
| CDR is a 6th Pareto point, not a dominating win | Med | High | THE risk the spike decides; concurrent realization is the only untried lever (all 5 prior points used a sequential realizer); if it ties not beats, NO-GO, attempt-3 stays |

---

## Testing Strategy

- **CDR-spike (integration gate):** the full scorecard + crossing audit (reuse
  `_probe_gco_spike.audit_crossings`). Pass/fail gates the build.
- **Unit — `build_constraint_graphs`:** VCG edge i→j iff i's top + j's bot share a column; HCG edge
  i—j iff x-spans overlap; deterministic edge set; cycle on a constructed inversion fixture.
- **Unit — `assign_tracks_left_edge`:** distinct track y per net (or disjoint x-span if shared);
  VCG order honored (no net below a predecessor); spine band skipped; minimum track count on an
  acyclic VCG; dogleg fires on a cyclic VCG fixture; 3-run identical.
- **Unit — `realize_channel`:** world_paths are direct geometry (horizontal run strictly at
  track_y); 0 different-net crossings on the emitted set (`seg_intersect` over all corridor runs);
  via count matches the witness 1-via profile for the primary ring.
- **Fixture — witness as a channel-route:** the witness's B.Cu GPIO bus track-y's + +3V3 band form a
  valid `ChannelInstance` whose left-edge assignment reproduces the witness's track order (Hamming
  distance 0 on track rank for the monotone primary ring).
- **Regression — default-OFF:** `channel=None` ⇒ byte-identical board to attempt-3.
- **Crossing-audit regression:** CDR output 0 different-net same-layer crossings (permanent gate).
- **Determinism:** 3-run byte-identical connectivity at spike, CDR-1, CDR-2.

---

## Acceptance Rubric

- [ ] **AC1 — design doc complete.** `docs/design/CONCURRENT-DETAILED-ROUTER.md` contains: channel
  model, constraint-graph concurrent algorithm (VCG+HCG+left-edge+dogleg+layers), determinism+bounded
  argument, direct-emission realization, staging, CDR-spike spec, feasibility-vs-witness.
  Evidence: §1-§5 + staging + spike spec present.
- [ ] **AC2 — corridor mapped to a 2-layer channel instance.** Doc specifies terminals (TOP ring
  vias + BOT connector pads), tracks (~46 B.Cu horizontal lanes × 2 layers), via transitions, and
  the +3V3 reserved band. Evidence: §1.
- [ ] **AC3 — concurrent algorithm is the constraint-graph channel router, specified concretely.**
  Doc gives VCG (vertical ordering from terminals), HCG (track-sharing from x-spans), left-edge
  track assignment, dogleg cycle-break, layer assignment — routing ALL corridor nets at once.
  Evidence: §2 Steps 1-5.
- [ ] **AC4 — replaces the per-net realizer via direct track emission.** Doc states the channel
  output → direct `world_paths`/`world_vias` (no `route_net_steered`, no per-net visgraph, no
  `_seg_in_free` against accumulated copper) → `to_gridless_netroute`+`emit_routes`, and explains
  why `bcu_run_failed` cannot occur. Evidence: §4.
- [ ] **AC5 — determinism + bounded argued, guards mapped to prior blowups.** Doc argues net-name
  sort + order_key tie-break + 1nm snap (determinism); and the eliminated visgraph surface +
  `_check_rss` 2GB + region-scope + 10s timeout, each mapped to a prior blowup. Evidence: §3 table.
- [ ] **AC6 — uses the GCO flow as warm-start (combine, not replace).** Doc states the flow's
  validated disjoint lane order seeds the VCG topological order + band placement; CDR replaces only
  the realization step. Evidence: §2 "How CDR combines with the GCO flow".
- [ ] **AC7 — feasibility vs witness: the witness IS a concurrent channel-route.** Doc states the
  witness's monotone B.Cu bus + +3V3 band IS a left-edge channel-route (0 crossings, capacity 3-4×
  slack), certifying the CDR output exists; honest about the non-corridor bound. Evidence: §5.
- [ ] **AC8 — staging has scorecard exit criteria.** Three milestones (spike → CDR-1 → CDR-2), each
  with numeric/binary exit (beat 41 AND ≤73). Evidence: §Staging.
- [ ] **AC9 — CDR-spike-first, full contending set, no production code before GO.** Spike FIRST,
  contending set = J3∪J4 (fixing GCO's J4-only), no production `channel_router.py`/wiring before
  spike GO. Evidence: §CDR-spike spec + structured result `cdr_spike_developer_tasks`.
- [ ] **AC10 — does NOT reuse the failing per-net realizer for the corridor.** Doc + spike spec
  explicitly emit tracks directly and forbid `route_net_steered` for corridor nets. Evidence: §4 +
  spike spec "do NOT reuse".
- [ ] **AC11 — no production code in the design.** Only illustrative signatures/pseudo-code.
  Evidence: no runnable function bodies.

---

## CDR-spike (2026-06-24) — THESIS CONFIRMED (bcu_run_failed eliminated) but spike regressed via B.Cu-jog bug

`scripts/_probe_cdr_spike.py` (PM-verified; probe-only, suite 424 green, ruff clean).

**THE THESIS IS CONFIRMED — concurrent realization breaks the per-net wall:**
- **bcu_run_failed ELIMINATED (0 occurrences)** — no net failed to reach its track. The wall that capped
  all 5 prior approaches is genuinely broken by computing all track-y's jointly + direct emission.
- Disjoint assignment passed; 0 illegal different-net crossings in the audit; channel solve 0.006s
  (107-node graph, 0 VCG cycles — mitayi is monotone); bounded (667MB peak).

**BUT the spike REGRESSED (44 unc / 159 err / 20 shorts) due to an IMPLEMENTATION bug, not the thesis:**
the spike routed the vertical jogs (via_y→track_y) on **B.Cu**, but the design §1 mandates **F.Cu jogs**
(B.Cu = horizontal tracks only). Adjacent ring-slot jogs at near-identical x-columns (/GPIO6 x=153.289,
/GPIO14 x=153.370 — 0.081mm apart, << the 0.45mm needed) overlap → 15-22 shorts. The agent correctly
diagnosed: "This spike used B.Cu for the jog, which is the architectural mistake."

**Verdict: NO-GO (regression), but the concurrent thesis HELD.** The fix per the design: jogs on F.Cu
(2-via-per-net, with ring-slot assignment reserving jog-safe columns). That is another iteration with
more geometric machinery (jog-safe column reservation), and the honest open risk persists: even with the
corridor short-free, the NON-corridor nets inherit attempt-3's error profile, so CDR may TIE not dominate.

**6th experiment, still no clean win. attempt-3 (commit 9ae76ea, 41/73) REMAINS THE DEFINITIVE BEST.**
PATTERN (honest, decisive): each architecture validates its CORE mechanism (escape nudge, ring-slot
legality, GCO disjoint flow, CDR concurrent emission) but each spike reveals the NEXT geometric
realization subtlety (lane crossing → via legality → corridor ordering → flow-vs-placement order →
B.Cu-jog overlap). We are in the long tail of matching a human full-board 0/0; marginal iterations
produce frontier/regression points + successive geometric details, not convergence to a clean win. The
CDR concurrent-channel infrastructure (constraint graphs, direct emission, validated bcu_run_failed
elimination) is committed for a future complete implementation (F.Cu jogs + jog-safe columns).
