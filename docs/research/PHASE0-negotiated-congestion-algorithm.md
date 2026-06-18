# Phase 0 — Negotiated-Congestion Algorithm Spec
## Canonical McMurchie-Ebeling PathFinder adapted for TraceWise 2-layer PCB

**Status:** research complete (2026-06-18). Decision input for Phase 1 (architect).
**Scope:** algorithm spec for a convergent negotiated-congestion global router replacing
the crude `pathfinder.py` attempt. Precise enough for an architect to design from.

---

## 1. Problem Framing

**Goal (technology-free):** given a fixed set of nets competing for shared routing
resources on a 2-layer PCB grid, find a jointly-legal, clearance-respecting route
assignment that maximises the number of connected nets (target: match human, ≤5
unconnected).

**Hard constraints for the new design:**
1. Must carry the existing `grid.hard` / `grid.cells` hard-copper/clearance-halo split;
   pads must be escapable via the escape-allowance mechanism.
2. Must converge (monotonically reduce congestion in expectation) or detect non-convergence
   and fall back gracefully rather than returning 0 routed nets.
3. Must be deterministic (byte-identical re-runs); no wall-clock truncation of nets.
4. Must handle a ~1.8 M-cell, 2-layer, octile grid with vias.
5. Must treat each net's connections incrementally (no all-or-nothing tree failure on one
   blocked pad).

**Known-not-wanted:** the crude `pathfinder.py` outer loop as-is (documented failure
causes below); ILP/SAT global router (scalability risk); full gridless router (Phase 4+).

---

## 2. The Canonical McMurchie-Ebeling PathFinder

### 2.1 Canonical cost function

For a routing resource (node or edge) `n`, the traversal cost in A* is:

```
cost(n)  =  b(n) · (1 + p_fac · p(n)) · (1 + h_fac · h(n))
```

where:

| Symbol | Meaning |
|--------|---------|
| `b(n)` | **Base cost.** Physical length (or delay) of the resource; for a grid cell this is the step length (1.0 for H/V, √2 for diagonal, `via_cost` for a layer change). |
| `p(n)` | **Present congestion.** Number of nets currently sharing resource `n` above its capacity: `p(n) = max(0, usage(n) − cap(n))`. Measures WITHIN-ITERATION overuse. |
| `p_fac` | **Present-cost scaling factor.** Starts near 0 and grows multiplicatively after each iteration. Forces routing alternatives when a resource is overused. |
| `h(n)` | **History.** Accumulated overuse of `n` ACROSS all past iterations: `h(n) += max(0, usage(n) − cap(n))` after each iteration ends. Permanently increases the cost of chronically contested resources so nets remember what they fought over. |
| `h_fac` | **History scaling factor.** Fixed coefficient (not ramped); typically 1.0. Controls how strongly history biases future routing. |

**Why this shape (Lagrangian interpretation):** `p_fac · p(n)` is a Lagrange multiplier
on the capacity constraint of resource `n`; raising `p_fac` performs subgradient ascent
on the dual. `h(n)` is a permanent memory of dual pressure — it prevents the algorithm
from cycling back to resources that repeatedly caused congestion. Together they implement
a convergent dual ascent on the multicommodity-flow capacity LP.
Source: McMurchie & Ebeling, FPGA 1995; formulation confirmed by Zha & Li, "Revisiting
PathFinder", 2022 (Semantic Scholar).

### 2.2 Canonical iteration loop (pseudocode)

```
# Initialise
h[all resources] = 0
p_fac = 0.0          # first iteration: ignore present congestion (free exploration)
usage[all resources] = 0

# ITERATION 0: greedy, uncongested route — allow overuse, establish history baseline
for net in all_nets (by priority order):
    path = astar(net, cost=base_only)
    commit(net, path)          # usage[n] += 1 for each resource n in path

# OUTER NEGOTIATION LOOP
for iter in 1 .. MAX_ITERS:
    # 1. History update (ACROSS iterations): accumulate overuse into h[]
    for n in all_resources:
        h[n] += max(0, usage[n] - cap[n])

    # 2. Present-cost ramp (grows BETWEEN iterations, constant WITHIN one)
    p_fac *= PRES_FAC_GROWTH      # e.g. 1.3 per iteration; p_fac = p_fac0 * 1.3^iter
    p_fac = min(p_fac, PRES_FAC_MAX)   # cap to prevent overflow; typical max 8-1000

    # 3. Select nets to rip up and reroute
    #    CANONICAL: rip up ALL nets that touch any over-capacity resource
    #    (routing all nets every iteration is also correct and simpler; selective
    #    rip-up is an optimisation).
    to_reroute = nets_touching_congested_resources(usage, cap)
    if not to_reroute:
        break   # CONVERGENCE: no resource is over capacity

    # 4. Rip up
    for net in to_reroute:
        release(net)    # usage[n] -= 1 for each resource in net's old path

    # 5. Reroute in fixed order (order affects quality, not convergence)
    for net in to_reroute (by priority or original order):
        path = astar(net, cost = b(n)*(1+p_fac*p(n))*(1+h_fac*h(n)))
        if path is not None:
            commit(net, path)
        else:
            mark_net_failed(net)   # genuinely unroutable given current costs

# POST-LOOP
over_capacity = [n for n in resources if usage[n] > cap[n]]
failed_nets   = [net for net in all_nets if net has no valid path]
# Residual over-capacity = board-inherent infeasibility or insufficient iterations
```

**Key timing invariants:**
- `h[n]` updates ONLY once per iteration (after all nets have been rerouted for that
  iteration). It NEVER changes mid-iteration. This gives history its "memory across
  rounds" semantics.
- `p_fac` is fixed for the entire duration of one iteration; it changes only between
  iterations. This makes the cost surface stable while A* searches.
- `p(n)` (present usage) changes dynamically as nets commit during the iteration. This
  is intentional: a net routed early in iteration `k` raises the cost for nets routed
  later in `k`, which is the within-iteration negotiation signal.

Sources: McMurchie & Ebeling FPGA 1995; VPR/VTR open-source implementation
(pres_fac_mult=1.3, acc_fac=1.0, initial_pres_fac=0.5 per VPR docs); OrthoRoute
(pres_fac = 1.15^iteration, capped at 8.0 for PCB).

---

## 3. Cost Schedule: Parameters That Converge vs Oscillate

### 3.1 Parameter summary (concrete ranges)

| Parameter | VPR default | OrthoRoute PCB | Recommended range for TraceWise |
|-----------|-------------|----------------|----------------------------------|
| `p_fac` initial (iter 0) | 0.0 | 0.0 | **0.0** (allow free exploration first) |
| `p_fac` at iter 1 | 0.5 | 1.0 | **0.5 – 1.0** |
| `PRES_FAC_GROWTH` | **1.3×** per iter | 1.15× per iter | **1.3×** (VPR) or 1.15–1.5× (PCB) |
| `PRES_FAC_MAX` | 1000.0 | **8.0** | **8–50**: cap needed on PCBs to prevent history being drowned |
| `h_fac` (acc_fac) | **1.0** | tuned per board | **1.0** (do NOT decay) |
| History decay | none | none (decay was a bug) | **None.** Decay causes 3× congestion growth |
| `MAX_ITERS` | 50 | board-adaptive | **30–60** for PCB; salvage fallback beyond |

### 3.2 Convergence argument

PathFinder converges when the dual ascent drives every over-capacity resource's price
high enough that only one net chooses it. The argument (informal):

1. `p_fac · p(n)` grows without bound (if uncapped). For any finite set of nets and
   resources, there exists a threshold `p_fac*` beyond which the cheapest path for each
   net no longer shares any resource with another net (each net has at least one
   alternative path with finite cost).
2. `h(n)` monotonically increases for resources that remain over-capacity, making those
   resources progressively less attractive even when `p(n)` temporarily drops (after a
   net reroutes away).
3. Together, `p_fac · p(n) + h_fac · h(n)` provides both short-term pressure
   (drive nets off hotspots this iteration) and long-term memory (don't re-use resources
   that were repeatedly congested).

**The convergence guarantee breaks when:**
- `PRES_FAC_MAX` is too low (history and present costs balance poorly; routes oscillate
  between alternatives without settling).
- History decay is applied (nets forget chronic hotspots; congestion can grow by 3× per
  iteration — measured by OrthoRoute).
- `PRES_FAC_GROWTH` is too large (costs spike before routes can find alternatives;
  produces 0-routed solutions in early iterations when ALL alternatives become
  expensive).
- Some resource has capacity 0 (a net is structurally trapped; price escalation cannot
  help; the router must have a genuine "no path" exit rather than looping infinitely at
  maximum cost).

### 3.3 Oscillation failure mode

Classic oscillation: nets A and B both want resource R. At iteration k, A gets R; B
reroutes to R' (its only alternative). At k+1, B's history on R' is low; A reroutes back
to an uncongested path; B returns to R. Loop repeats without convergence.

Fix: monotonically-increasing `h(n)` with no decay. After A occupies R for two
iterations, `h[R]` accumulates and both A's and B's cost for R rises enough that the
cheaper alternative wins and stays. This is why VPR removes history decay and sets
`acc_fac=1`.

Source: OrthoRoute empirical finding; VPR parameter documentation.

---

## 4. Two-Layer PCB Capacity and Escape Model

### 4.1 Capacity definition on a PCB grid

FPGA PathFinder runs on a routing resource graph (RRG) where each wire segment has an
explicit capacity (typically 1 per track per channel). A PCB occupancy grid is different:

- **Each cell (layer, iy, ix)** is a potential track centerline. Capacity = **1**: at most
  one net's centerline may pass through a cell such that its clearance halo does not
  overlap another net's halo.
- **Hard-copper cells** (`grid.hard > 0`): capacity = **0**. Fixed copper (pads, keepouts,
  pours). Entering them is forbidden regardless of price.
- **Clearance-halo cells** (`grid.cells > 0` but `grid.hard == 0`): these represent the
  clearance region around existing copper. Capacity semantics depend on the router's
  occupation model (see §4.2).

The TraceWise grid already separates these two populations. The crude `pathfinder.py`
also separates them. The critical issue is how `usage(n)` and `cap(n)` are defined
for halo cells.

### 4.2 The correct halo occupation model

**The correct model:** a net "occupies" a resource when its **inflated footprint** (track
centerline expanded by `halfwidth + clearance`) covers that resource cell. Two nets'
halo footprints cannot overlap without a clearance violation.

Implementation in terms of existing grid fields:

```
# When net A commits path P_A:
for cell in halo_cells(P_A, halfwidth=net_A.halfwidth_cells):
    usage[cell] += 1
# cap[cell] = 1 for all routing cells
# p(cell) = max(0, usage[cell] - 1)
```

This is exactly what `pathfinder.py` does with `r.halo` and the `commit()` function —
correct in principle. The congestion signal `p(n)` then fires when two nets' clearance
halos overlap, which is the right definition of a clearance violation.

**The dilated congestion pricing** in `pathfinder.py` (`_dilate(occ, hw)`) is also
correct: when evaluating candidate centerlines during A*, the cost uses the
MAX-dilated `occ` so that a centerline whose footprint would touch an occupied halo pays
the congestion price even if its own centerline cell has `occ=0`. This prevents two nets
from each routing a "free centerline" that physically overlaps.

### 4.3 The hard/halo split and the escape allowance

This is the critical difference between FPGA PathFinder and a PCB router:

| Resource type | FPGA equivalent | PCB (`grid.py`) | Routing treatment |
|---------------|-----------------|-----------------|-------------------|
| Hard copper (pads, keepouts, planes) | Used wire segment | `grid.hard > 0` | **Forbidden** (capacity 0) |
| Clearance halo (spacing around existing copper) | No equivalent (channel routing doesn't need it) | `grid.cells > 0` and `grid.hard == 0` | **Priced with fixed_pen**, not forbidden — this IS the escape allowance |
| Free routing space | Unused wire segment | `grid.cells == 0` and `grid.hard == 0` | **Free** (priced only by congestion) |

The **escape allowance** is the mechanism by which a net can leave its own pad: the pad's
clearance halo would normally block routing; the router temporarily relaxes this for the
net's own pads using `carve()`, which clears both `base_hard` and `fixed` in the pad
region so the net can route out.

**Without escape allowance:** a pad surrounded by other nets' clearance halos is
permanently sealed. The router returns "no path" immediately. This was the failure mode
of the very first crude PathFinder variant (ROUTING-COMPLETION-PLAN.md "collapsed hard
copper vs clearance-halo into one wall so pads couldn't ESCAPE"). The current
`pathfinder.py` DOES implement `carve()` correctly — this is no longer the failure cause.

### 4.4 Via model

Vias introduce a layer change at cost `via_cost`. On a 2-layer board:
- Via cell (iy, ix) requires a clear ring of radius `VIA_RING = 2` cells on BOTH layers
  (astar.py `via_ok()`).
- Via congestion: a via site can be occupied by at most 1 net; the halo extends `vhw`
  cells on all layers (PathFinder `_halo_cells`, `vhw = net.via_halfwidth_cells`).
- This is already correctly modelled in `pathfinder.py`.

### 4.5 Per-cell capacity summary

```
cap(layer, iy, ix) =
    0    if grid.hard[layer, iy, ix] > 0          # permanent copper: forbidden
    0    if sum of neighbour hard cells seals cell  # physically unreachable
    1    otherwise                                  # one net's halo footprint allowed
```

Clearance halo cells (`grid.cells > 0`, `grid.hard == 0`) have **soft capacity 1 with
a fixed base penalty** (`fixed_pen`), not hard capacity 0. This is the distinction that
enables pad escape.

---

## 5. Reconciliation: Why the Crude `pathfinder.py` Diverged

Reading `pathfinder.py` and `ROUTING-COMPLETION-PLAN.md` together, the code has three
structural divergence causes. The first (escape model) was the PARK entry's stated
reason; the second and third are visible in the code and explain why even after the
escape model was added, the router failed on dense boards like mitayi (0/61 routed).

### Root Cause 1: Present-cost GROWTH factor starts too high and lacks an iteration-0 free pass

**Evidence in code:**

```python
p_fac = 0.5          # line 229: initial value
# ...
p_fac *= p_growth    # line 259: grown EACH iteration, including the first
```

With `p_growth=1.8` (line 188, default), the factor schedule is:
- Before iteration 0: `p_fac = 0.5`
- After iteration 0: `p_fac = 0.5 × 1.8 = 0.9`
- After iteration 1: `p_fac = 0.9 × 1.8 = 1.62`
- After iteration 4: `p_fac ≈ 8.5`
- After iteration 8: `p_fac ≈ 72`

**The canonical algorithm initialises `p_fac = 0.0` for iteration 0** (the free
exploration pass that lets all nets find ANY path regardless of overuse). Starting at
0.5 is not catastrophic, but the growth rate 1.8× is much more aggressive than VPR's
1.3× or OrthoRoute's 1.15×. By iteration 5–6, present costs dominate history so strongly
that paths through ANY previously-used cell become extremely expensive — the router
effectively reverts to hard-wall behaviour (very high price ≈ forbidden).

On a dense board like mitayi (where competing nets have few alternatives), this means:
after 5 iterations, the cost surface becomes so steep that A* cannot find paths for
most nets (effectively "no path" even though paths exist at a lower price). The router
returns `None` (line 252: `state[net.name] = _Net(net.name)` — empty, failed) and nets
accumulate as failed with no committed path. Those empty-state nets no longer release
their (non-existent) halo; but because they were never committed, `occ` remains at the
over-capacity state from competing nets — causing a cascade where more nets fail.

**Fix required:** Set `p_fac = 0.0` for iteration 0 (free-exploration pass). Use
`PRES_FAC_GROWTH = 1.3` (VPR) or `1.15` (OrthoRoute). Cap `p_fac` at 8–50 to prevent
the cost surface from becoming a hard wall again.

### Root Cause 2: History is updated before ALL nets have been rerouted in an iteration (timing mismatch)

**Evidence in code:**

```python
for iter in range(iters):                    # outer loop
    over = occ > 1
    to_route = [n for n in routed            # select congested nets
                if n.name not in state
                or any(over[c] for c in state[n.name].halo)]
    if not to_route:
        break
    for net in to_route:                     # reroute each selected net
        if net.name in state:
            commit(net, state[net.name], -1)
            del state[net.name]
        # ... route ...
        commit(net, r, 1)
        state[net.name] = r
    hist += np.maximum(0.0, occ.astype(np.float64) - 1.0)  # line 258: AFTER all nets in iteration
```

Actually, the history update IS at the end of the iteration loop (line 258). This is
correct timing. However, the `p_fac` growth (line 259: `p_fac *= p_growth`) happens
ALSO at the end of each iteration — meaning iteration 0 ALREADY starts at `p_fac=0.5`
(not 0.0), and by the end of iteration 0, `p_fac` has already grown to 0.9.

The deeper issue is that `to_route` is computed at the START of each iteration using the
`over` snapshot, but the `over` mask does NOT update during the iteration as nets commit.
This means: if net A's routing in iteration k removes congestion from a cell that net B
is queued to avoid, net B still avoids it (because it was in `to_route` based on the
stale `over` snapshot). This is minor. More critically, if net A creates NEW congestion
in a cell that net B was relying on, net B is not re-queued — it commits its old path
on top of A's new path, increasing congestion without triggering re-rerouting. This is
a missed negotiation step that can lock in congestion.

**Fix required:** After each net commits during an iteration, update the congestion
check so subsequent nets in the same iteration respond to the new state. Or, use the
canonical "rip up and reroute ALL congested nets" definition where each net sees the
CURRENT `occ` when it routes (not a stale snapshot). The current code does use the
current `occ_cost` at route time (it re-dilates dynamically), but the `to_route` set is
frozen at iteration start, preventing late-committers from being added to the reroute
queue within the same iteration.

### Root Cause 3: Selective rip-up without a full-nets fallback causes starvation on dense boards

**Evidence in code:**

```python
to_route = [n for n in routed
            if n.name not in state              # never routed
            or any(over[c] for c in state[n.name].halo)]  # currently congested
```

The `any(over[c] ...)` check uses a frozen `over` snapshot. On a dense board like mitayi,
after the first few aggressive-`p_fac` iterations, many nets fail (return `None`) and
are stored as empty `_Net` objects with no cells and no halo. The condition
`n.name not in state` only triggers for nets that were NEVER routed. Failed nets (stored
as empty `_Net`) REMAIN in `state` with an empty halo, so `any(over[c] ...)` is vacuously
False (empty set has no element matching `over`). These nets are NEVER rerouted after
their first failure — they are permanently abandoned.

The canonical algorithm explicitly handles this: at the start of each iteration, rip up
ALL nets that share ANY over-capacity resource (not just those with non-empty halos). A
net with an empty halo (failed in a prior iteration) should be retried because conditions
may have changed (other nets rerouted around their hotspots).

**Fix required:** Add a condition: `or not state[n.name].paths` (has no routed path —
was previously failed) to the `to_route` selection. Equivalently, reroute all nets in
the first iteration (canonical free-exploration), then reroute only congested+failed nets
in subsequent iterations.

### Summary reconciliation table

| Root Cause | Evidence in `pathfinder.py` | Effect on mitayi | Required Fix |
|------------|-----------------------------|--------------------|--------------|
| RC1: `p_fac` starts at 0.5, grows 1.8× | lines 229, 259; `p_growth=1.8` default | By iter 5, cost surface ≈ hard wall; A* returns None for most nets | `p_fac_0=0.0`; `PRES_FAC_GROWTH=1.3`; `PRES_FAC_MAX=8–50` |
| RC2: `to_route` frozen at iteration start, new congestion mid-iteration missed | lines 231–233, stale `over` snapshot | Late-committing nets worsen congestion not seen by subsequent nets in same iter | Recompute `over` live, or use canonical rip-up-all-congested with current state |
| RC3: failed nets (empty halo) never re-queued | lines 231–233; empty `_Net` has `halo=set()` | Once a net fails it is abandoned for all remaining iterations | Add `or not state[n.name].paths` to reroute condition; retry failed nets |

Note: the escape model (the PARK entry's stated cause) was FIXED in the current
`pathfinder.py` via `carve()`. RC1–RC3 are the remaining live causes of the 0/61
divergence on mitayi.

---

## 6. Correct Algorithm Design for TraceWise

### 6.1 The outer negotiation loop (corrected)

```python
# PHASE 0: Initialise
h    = np.zeros((L, H, W), float)   # history: per resource, accumulates across iters
occ  = np.zeros((L, H, W), int)     # present usage: recomputed each iter commit
p_fac = 0.0                          # FIX RC1: start at 0; iteration 0 = free exploration
state: dict[str, _Net] = {}

# PHASE 1: Iteration 0 — free exploration (p_fac=0, h=0)
for net in all_nets_by_priority():
    carve(net, on=True)
    r = route_net_soft(net, occ, h, h_fac=H_FAC, p_fac=p_fac)
    carve(net, on=False)
    if r is not None:
        commit(net, r, +1)
        state[net.name] = r
    else:
        state[net.name] = _empty_net()   # FIX RC3: store empty, not missing

# PHASE 2: Negotiation iterations
for iteration in range(1, MAX_ITERS):
    # History update (ACROSS iterations, once per iteration, after all nets done)
    h += np.maximum(0.0, occ.astype(float) - 1.0)

    # Present-cost ramp (BETWEEN iterations, fixed WITHIN)
    if iteration == 1:
        p_fac = PRES_FAC_INIT    # e.g. 0.5
    else:
        p_fac = min(p_fac * PRES_FAC_GROWTH, PRES_FAC_MAX)

    # Check convergence
    if not np.any(occ > 1):
        break

    # Select nets to reroute: congested OR previously failed (FIX RC2+RC3)
    to_reroute = []
    for net in all_nets:
        r = state.get(net.name)
        is_congested = (r is not None and r.halo and
                        any(occ[c] > 1 for c in r.halo))
        is_failed    = (r is None or not r.paths)   # FIX RC3
        if is_congested or is_failed:
            to_reroute.append(net)

    # Rip up selected nets
    for net in to_reroute:
        if net.name in state and state[net.name].halo:
            for c in state[net.name].halo:
                occ[c] -= 1
            del state[net.name]

    # Reroute: p_fac is FIXED for this entire iteration; occ updates live (FIX RC2)
    for net in to_reroute:
        carve(net, on=True)
        hw = net.halfwidth_cells
        occ_priced = dilate(occ, hw)
        h_priced   = dilate(h, hw)
        r = route_net_soft(net, occ_priced, h_priced, h_fac=H_FAC, p_fac=p_fac)
        carve(net, on=False)
        if r is not None:
            commit(net, r, +1)
            state[net.name] = r
        else:
            state[net.name] = _empty_net()

# PHASE 3: Extract results
over = occ > 1
results = {}
for name, r in state.items():
    ok = bool(r.paths) and not any(over[c] for c in r.halo)
    results[name] = NetRoute(net=..., paths=r.paths, ok=ok, ...)
```

### 6.2 Per-net cost function in A* (unchanged shape, corrected parameters)

```python
# Inside A* expansion, cost of taking step to neighbour cell nxt=(layer, iy, ix):
step = base_move_cost            # 1.0 H/V, √2 diagonal, via_cost for layer-change
if nxt is a hard-copper cell and not own-pad:
    skip(nxt)                    # hard capacity = 0: forbidden
if nxt is a clearance-halo cell (fixed):
    step *= (1.0 + FIXED_PEN * 1)   # constant escape penalty, not escalated by p_fac
# Congestion cost (present): only on routing space (not hard cells):
p_congestion = occ_dilated[layer, iy, ix]
h_congestion = h_dilated[layer, iy, ix]
step *= (1.0 + p_fac * p_congestion)
step *= (1.0 + H_FAC  * h_congestion)
```

The `FIXED_PEN` for clearance halos must NOT be multiplied by `p_fac`. It is a fixed
base premium (typically 4.0) that allows pad escape at a known cost but does not
escalate with the negotiation. This is what makes pad escape stable across all
iterations — the very distinction the earliest crude attempt lacked.

### 6.3 Recommended parameter set for TraceWise

| Parameter | Value | Rationale |
|-----------|-------|-----------|
| `p_fac_0` | `0.0` | Free exploration on iteration 0 |
| `PRES_FAC_INIT` | `0.5` | VPR default; enough signal on iteration 1 |
| `PRES_FAC_GROWTH` | `1.3` | VPR validated; PCB may need 1.15–1.5 |
| `PRES_FAC_MAX` | `20.0` | PCB tuning: cap before history is drowned; OrthoRoute uses 8.0; allow headroom |
| `H_FAC` | `1.0` | VPR default; do not decay |
| `FIXED_PEN` | `4.0` | Existing value from `pathfinder.py`; preserves pad escape |
| `VIA_COST` | `10.0` | Existing; tunable |
| `MAX_ITERS` | `40` | 30–60 for PCBs; salvage fallback at cap |
| `History decay` | `None` | VPR and OrthoRoute both found decay causes divergence |

**Tuning sensitivity:** the single most critical parameter is `PRES_FAC_GROWTH`. Too
large (> 1.8) → cost surface becomes a hard wall early, causing RC1 failure. Too small
(< 1.1) → slow convergence, many iterations needed. 1.3 is the validated sweet spot for
FPGA (VPR); PCB is denser per unit area so 1.15–1.3 is the recommended range. Measure
convergence on mitayi as the acceptance criterion (denser than zuluscsi).

---

## 7. Determinism and Runtime

### 7.1 Determinism requirements

The existing A* achieves determinism via:
1. The vectorized octile heuristic (eliminates Python float rounding non-determinism
   from repeated `min` calls over large goal sets).
2. Deterministic net ordering (`order_nets` in `multi.py`).

The new outer loop adds: `h` and `occ` numpy arrays — deterministic if net ordering is
deterministic; no random tie-breaking. The only hazard is iteration order within
`to_reroute`; this must use the same `order_nets` ordering, not a dict insertion order.

### 7.2 No wall-clock truncation of nets

The existing `astar.py` `max_seconds` guard truncates INDIVIDUAL net routing (returns
"time budget exceeded"). This is acceptable for the inner A* search. The OUTER
negotiation loop must not truncate nets — if a net cannot be routed in the current
iteration, store it as a failed net and retry in subsequent iterations (RC3 fix).

The outer loop terminates via `MAX_ITERS` (not wall-clock). After `MAX_ITERS`, apply the
existing salvage-pass fallback for any still-unconnected nets.

### 7.3 Runtime on 1.8M-cell grid

Each iteration reroutes O(K) congested nets where K is the number of still-contested
nets (shrinks toward 0 at convergence). Each net's A* search costs O(N log N) in the
worst case. The vectorized heuristic in `astar.py` is already the dominant speedup.

Additional leverage:
- The `_dilate(occ, hw)` operation is O(H·W) per net per iteration; pre-compute once per
  iteration for each distinct `hw` value among the rerouted nets (or skip dilation for
  `hw=1` where centerline occupancy is a close enough proxy).
- For early iterations (p_fac=0), skip dilation entirely (occ is all zeros after
  iteration 0 → dilated occ is also zero → no congestion yet).
- The convergence check `np.any(occ > 1)` is O(1) with numpy and should be the loop
  guard, not a Python iteration.

Rough runtime estimate at 40 iterations, 60 nets rerouted on average per iteration, 1.8M
cells: 40 × 60 × (single-net A* ~2s avg on 2-layer 1.8M grid) = ~4800 s worst case.
The vectorized A* (zuluscsi reported ~530s for full single-pass route) suggests a more
realistic single-net avg of 530/116_nets ≈ 4.5s → 40×60×4.5 ≈ 10,800s (3 hours).

**This is unacceptable.** Runtime mitigation is essential:

1. **Coarsen-then-refine (0.2mm → 0.1mm):** global negotiation at 0.2mm pitch
   (¼ the cells, much faster convergence), then refine on the 0.1mm grid guided by the
   coarse result. Standard technique in commercial global routers.
2. **Skip uncongested nets entirely:** reroute only `to_reroute` (already done in
   `pathfinder.py`), not all nets. By iteration 10–15, only 10–20 nets may remain
   contested.
3. **Shared history field reuse:** the `_dilate` operation is the bottleneck for many
   nets with the same `halfwidth_cells`. Group nets by width and dilate once.
4. **Expansion-cap calibration:** each net's A* has `max_expansions = 2 × L × H × W`.
   A per-iteration cap of `4 × L × H × W` (global budget shared across all nets in the
   iteration) may be more appropriate as an iteration-level cost guard.

The architect should decide whether to implement a coarse-then-refine pass or accept
longer runtime behind a time-gated flag (users can pre-empt with the salvage result).

---

## 8. Relation to KiCad / Freerouting

**KiCad's built-in router (pcbnew)** uses push-and-shove with look-ahead; it does NOT
implement negotiated congestion. It is a single-pass interactive tool, not a batch
optimizer. Not relevant to this design.

**Freerouting** implements a simplified rip-up-and-reroute with some history penalty, but
does not use PathFinder's present-cost/history factored cost or the outer negotiation
loop as described above. Its convergence is not guaranteed.

**OrthoRoute** is the closest reference: PCB-adapted PathFinder on GPU, confirms the
algorithm works on PCB grids, confirms the key parameter issues (pres_fac cap, no history
decay, fixed hotset). Its escape planner targets SMD-only; TraceWise's `carve()` mechanism
is a more general equivalent.

Source: OrthoRoute project page (bbenchoff.github.io/pages/OrthoRoute.html).

---

## 9. Honest Gaps

1. The original 1995 McMurchie-Ebeling PDF could not be decoded (LZW compression). All
   canonical cost function details are sourced from secondary references (VPR docs, the
   "Revisiting PathFinder" paper, OrthoRoute) and are consistent across those sources.
2. The "Revisiting PathFinder" paper (Zha & Li 2022) full text returned HTTP 403. The
   Semantic Scholar abstract was accessible.
3. Runtime projections are Fermi estimates based on existing profiling data (530s for
   full zuluscsi single-pass). Actual negotiated-router runtime requires measurement.
4. OrthoRoute's parameter "pres_fac_max=8.0" was chosen for GPU-parallel PCB routing;
   the optimal cap for a serial Python implementation may differ.
5. The `_dilate` call in `pathfinder.py` is not referenced in any external source; it
   is a TraceWise-specific optimization that appears correct but has no external
   validation. The architect should evaluate whether halo-dilated congestion pricing
   introduces asymmetric pressures between narrow and wide nets.

---

## 10. Recommended Next Agent

**Architect** — design integration of the corrected PathFinder into the existing
TraceWise routing engine. Key decisions for architect:
- Whether to implement coarsen-then-refine (0.2mm global → 0.1mm detail) or accept
  longer runtime with a time-gated flag.
- Whether to extend `pathfinder.py` in-place (targeted fixes for RC1–RC3) or rewrite
  the outer loop as a standalone module behind `engine="negotiated"`.
- How to integrate the salvage pass as a post-convergence fallback (already exists in
  `multi.py`).
- The exact `to_reroute` selection policy (rip-up ALL nets every iteration for
  simplicity, vs. selective rip-up for runtime).
- Whether to carry the `_dilate` halo-aware pricing or simplify to centerline-only
  congestion (tradeoff: accuracy vs. speed).

---

## Structured Result

```json
{
  "status": "success",
  "summary": "Surveyed the canonical McMurchie-Ebeling PathFinder algorithm (FPGA 1995) and its PCB adaptation (OrthoRoute), confirmed by VPR/VTR parameter documentation. Identified three concrete root causes (RC1: p_fac growth too aggressive/no free-exploration pass; RC2: frozen to_reroute snapshot misses mid-iteration congestion; RC3: failed nets with empty halos never re-queued) that explain why pathfinder.py produces 0/61 on mitayi. Specified the corrected algorithm with concrete parameter ranges and convergence argument. Determinism and runtime constraints documented with mitigation strategies.",
  "files_changed": ["/home/palgin/Business_projects/tracewise/docs/research/PHASE0-negotiated-congestion-algorithm.md"],
  "files_read": [
    "/home/palgin/Business_projects/tracewise/docs/design/GLOBAL-ROUTER-DESIGN.md",
    "/home/palgin/Business_projects/tracewise/src/tracewise/route/engine/pathfinder.py",
    "/home/palgin/Business_projects/tracewise/docs/FORMULATION.md",
    "/home/palgin/Business_projects/tracewise/docs/ROUTING-COMPLETION-PLAN.md",
    "/home/palgin/Business_projects/tracewise/src/tracewise/route/engine/grid.py",
    "/home/palgin/Business_projects/tracewise/src/tracewise/route/engine/astar.py"
  ],
  "issues": [
    "Runtime on 1.8M-cell grid at 40 iterations may be prohibitive (estimate 3-10 hours serial); coarsen-then-refine or per-iteration budget cap required before deployment.",
    "PRES_FAC_MAX needs empirical calibration on mitayi/zuluscsi; the 8.0 value from OrthoRoute was tuned for GPU-parallel routing and may differ for serial execution.",
    "The _dilate halo-aware congestion pricing in pathfinder.py has no external validation; architect should assess whether it creates asymmetric pressure between nets of different widths."
  ],
  "assumptions": [
    "The existing carve() escape mechanism in pathfinder.py correctly handles pad escape; this was verified by reading the code and is no longer a root cause.",
    "The vectorized A* heuristic in astar.py is the correct inner search to reuse; the cost function extension is the outer-loop change only.",
    "Salvage-pass fallback (existing multi.py) serves as the non-convergence fallback after MAX_ITERS."
  ],
  "sources_cited": [
    "https://www.cecs.uci.edu/~papers/compendium94-03/papers/1995/fpga95/pdffiles/6a.pdf",
    "https://bbenchoff.github.io/pages/OrthoRoute.html",
    "https://docs.verilogtorouting.org/en/latest/vpr/command_line_usage/",
    "https://www.semanticscholar.org/paper/Revisiting-PathFinder-Routing-Algorithm-Zha-Li/04fd9d7b7514dda3ccdfd3413059ff7e81988b2f",
    "https://dl.acm.org/doi/10.1145/201310.201328",
    "https://github.com/verilog-to-routing/vtr-verilog-to-routing",
    "https://ieeexplore.ieee.org/document/1377269/"
  ],
  "research_summary": {
    "goal": "Specify the correct McMurchie-Ebeling PathFinder algorithm for a 2-layer PCB grid, explain why the crude pathfinder.py diverged, and provide a corrected design an architect can implement from.",
    "candidates_surveyed": 3,
    "verdict": "recommend_existing",
    "top_pick": "Corrected McMurchie-Ebeling PathFinder with RC1+RC2+RC3 fixes",
    "artifact_location": "/home/palgin/Business_projects/tracewise/docs/research/PHASE0-negotiated-congestion-algorithm.md",
    "next_agent_hint": "architect"
  }
}
```
