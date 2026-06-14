# TraceWise — Mathematical Formulation of Combined Placement + Routing

**Status of this document.** This is a *first-principles* mathematical model of the
TraceWise place-and-route problem. It is written to match the system described in
`KiCad-AI-Plugin-Design.md` (the M3 analytical placer + M4 router/iterate loop) and the
SOTA review in `research/46-ai-place-route-sota.md`.

> **Note on numbers.** The engine described here is built (`src/tracewise/{place,route}/`)
> and the figures are MEASURED, not targets: zuluscsi ~48 unconnected / ~103 DRC errors
> (1.8M-cell 0.1 mm grid, 116 nets); mitayi ~63 unconnected. (This document was first
> drafted from the design doc + SOTA survey, hence any residual `[TARGET]` tags — read them
> as measured reality.) The mathematics and the algorithmic recommendation stand on the
> model, independent of the exact magnitudes.

---

## 0. Notation and the two views

We model one board. Two views are used throughout and must be kept distinct:

- **Continuous view (placement-native).** Geometry lives in `R^2` (millimetres), copper is
  a union of polygons, clearance is Euclidean distance between polygon sets. The placer
  (`place/core.py`, gradient descent) lives here.
- **Discretized view (routing-native).** Geometry lives on a uniform grid
  `G = {0,...,W-1} × {0,...,H-1} × L` where `L = {F, B}` are the two copper layers (front /
  back), with pitch `δ = 0.1 mm`. The router (`route/engine/{grid,astar,multi}.py`) lives
  here. For zuluscsi, `W·H·|L| ≈ 1.8×10^6` cells `[TARGET]`.

The *coupling* problem is that placement is solved in the continuous view but its quality is
only realized after projection into the discrete view and routing. This projection is lossy
(sub-`δ` gaps vanish), which is itself one of the measured walls (coarser pitch misses
sub-pitch gaps).

Symbols:

| Symbol | Meaning |
|---|---|
| `C` | set of components (footprints) |
| `N` | set of nets; net `n` has terminal pin-set `P_n` |
| `xc, yc ∈ R` | continuous position of component `c`'s origin |
| `θc ∈ Θ = {0,90,180,270}` | orientation (rot90) |
| `sc ∈ {F,B}` | board side |
| `K` | set of keep-out polygons (incl. hard-copper plane regions, mounting, mechanical) |
| `Cl(n,m)` | required min clearance between copper of nets `n,m` (by net class) |
| `w_n` | required track width of net `n` |
| `Π` | the placement decision `(xc,yc,θc,sc)_{c∈C}` |
| `R` | the routing decision (per-net realized copper) |

---

## 1. Decision variables

### 1.1 Placement (continuous–discrete hybrid)

For each component `c ∈ C`:

```
xc, yc ∈ R            position (continuous)
θc     ∈ {0,90,180,270}   orientation (discrete, "rotation gated" in core.py)
sc     ∈ {F, B}            side (discrete)
```

`θc, sc` together select a rigid transform `T_c = T(xc,yc,θc,sc)` mapping each pin/courtyard
polygon of `c` from local to board coordinates. Side `sc` mirrors the x-axis and remaps the
copper layer of each pin. The full placement vector is `Π ∈ R^{2|C|} × Θ^{|C|} × {F,B}^{|C|}`
— a mixed continuous/combinatorial space. This is exactly why `core.py` uses **gradient
descent on the continuous block** `(xc,yc)` with the discrete block `(θc,sc)` *gated* (held
fixed inside a smooth pass, flipped by discrete moves between passes): the objective is not
differentiable in `θ, s`.

### 1.2 Routing — continuous (geometric) view

For each net `n`, a routing is a connected set of copper geometry `r_n` realizing the
required topology over `P_n`: a union of track segments (width `w_n`) and vias such that the
copper of `r_n` together with the pins `T(P_n)` is connected (one electrical node). The
collection `R = {r_n}` is the routing.

### 1.3 Routing — discretized (grid) view

This is what the A* router actually optimizes. Build a routing graph
`Γ = (V, E)`:

- **Vertices** `V = G = {(i,j,ℓ)}`, one per grid cell per layer.
- **Planar edges** `E_planar`: octile connectivity (the A* uses an octile heuristic), i.e.
  each cell connects to its 8 in-layer neighbours. Horizontal/vertical edges have length `δ`,
  diagonal edges length `δ√2`.
- **Via edges** `E_via`: `(i,j,F)—(i,j,B)` with a via cost.

A routed net `n` is a **Steiner-connected subgraph** `t_n ⊆ Γ` spanning the cells that
contain the pins of `P_n`. `multi.py` builds this as a *connection tree* (sequential
two-terminal A* segments stitched into a tree), not a true optimal rectilinear Steiner tree —
this is an approximation we will return to in §5.

**Occupancy.** Each cell `(i,j,ℓ)` holds a count `u(i,j,ℓ) = Σ_n 1[(i,j,ℓ) ∈ t_n]`
(`grid.py` is described as "counting occupancy"). A *capacity* `cap(i,j,ℓ) = 1` encodes
"one net per cell" (with clearance baked into the discretization). **Overuse** is
`max(0, u − cap)`. The whole negotiated-congestion idea in §5 hinges on the fact that the
grid *can represent* `u > cap` even though a legal board cannot *ship* with `u > cap`.

---

## 2. Objective

### 2.1 The scalar in use

The brief fixes the operative objective (the one `auto.py`'s keep-best loop minimizes):

```
J(Π, R) = 5 · U(Π,R)  +  1 · D(Π,R)
```

where

- `U` = number of **unconnected** nets/airwires (failures of the connectivity constraint,
  counted not enforced), and
- `D` = number of **DRC errors** (clearance/short, keepout, tracks-crossing, etc.).

The goal is the point `(U, D) = (0, 0)`. Both terms are *defects*, not costs; the ideal is
not "small `J`" but `J = 0`.

### 2.2 Why it is this shape (hierarchical / weighted), and why `5`

This is a **scalarization of a two-objective lexicographic-ish program**. The "true" intent is
roughly lexicographic — *first* make it connected, *then* make it clean — but pure lexicographic
optimization is brittle for a stochastic heuristic (it cannot trade a hopeless net for ten
fixed violations). The weight `5` is a **soft lexicographic** device: it makes one unconnected
net "worth" five DRC errors, biasing search toward connectivity while still letting the loop
escape local traps. The brief notes this weight was forced by "lexicographic gaming" — i.e. a
pure `U`-first rule let the optimizer farm trivial connections while exploding `D`.

The deeper objective, including the placer's continuous terms, is:

```
minimize   Φ(Π,R) = λ_U·U + λ_D·D + λ_W·WL(Π,R) + (placement regularizers)
```

with `λ_U = 5, λ_D = 1` and `WL` the total wirelength. The placer's *smooth surrogate*
(`core.py`) for `WL` is the **log-sum-exp HPWL** (a standard differentiable approximation of
half-perimeter wirelength):

```
WL_smooth(Π) = Σ_n γ·( log Σ_{p∈P_n} e^{x_p/γ} + log Σ_{p∈P_n} e^{-x_p/γ}
                        + log Σ_{p∈P_n} e^{y_p/γ} + log Σ_{p∈P_n} e^{-y_p/γ} )
```

`x_p, y_p` are pin coordinates induced by `Π`; `γ→0` recovers exact HPWL. The placer also adds
**soft-overlap** (a density/penalty term ≈ `Σ overlap_area`), **boundary** (keep inside the
board), **decap proximity**, and **congestion** (pin-density) penalties. Each is a
differentiable surrogate for a constraint that becomes hard only after legalization (Tetris).

**Key structural observation.** `U` and `D` are *not* differentiable in `Π` and are not even
defined without running the router. The placer therefore optimizes a **surrogate**
(`WL_smooth` + density + congestion) whose only justification is that it *correlates* with the
real objective `J(Π, route(Π))`. The validated ECCF routability signal (T1 cost-to-go fields,
T2 escape-stitch, T3 pseudo-route) is precisely an attempt to make that surrogate *more
faithful* — T3 is the only tier that estimates the routability **externality** a placement move
imposes on *other* nets. In optimization terms, ECCF-T3 is a cheap evaluation of
`∂J/∂Π` that internalizes a cross-net externality the HPWL surrogate is blind to. Hold this
thought; it is the hook for §5(d).

---

## 3. Constraints

We separate **solver-controllable constraints** (the model can satisfy them) from
**board-inherent defects** (outside the solver's reach — e.g. footprint pads that violate
clearance by construction, or nets that are unroutable on 2 layers given fixed connectors).
Counting a board-inherent defect in `D` and then "optimizing" it is a category error and a
source of the measured long tail.

### 3.1 Clearance (min inter-net spacing) — continuous

For every pair of distinct nets `n ≠ m` and every piece of their copper:

```
dist( copper(n), copper(m) ) ≥ Cl(n,m)            (C-CLEAR)
```

`dist` is Euclidean distance between closed polygon sets. In the grid view this becomes a
**packing/spacing** condition: distinct nets may not occupy cells closer than `⌈Cl/δ⌉`. The
"shorting / clearance (track-track at <clearance)" tail is exactly violations of (C-CLEAR)
that the discretization let slip — the grid forbids *same-cell* sharing (`cap=1`) but
**does not forbid two nets in adjacent cells whose actual copper edges are <`Cl` apart**, nor
diagonal near-touches (see §3.6). Whenever the router is "forced to make a connection," it
shaves the (C-CLEAR) margin: this is the measured *more-time → more-connections-but-more-
violations* tradeoff made formal.

### 3.2 Keepout (no copper in polygons)

For every keepout polygon `k ∈ K` (mechanical keepout, hard-copper plane region, mounting
holes) and every net `n`:

```
copper(n) ∩ interior(k) = ∅                         (C-KEEPOUT)
```

Grid view: cells whose footprint intersects any `k` are removed from `V` (blocked).
"keepout-violations (now being fixed)" = cells that should have been blocked but were not, i.e.
a discretization/rasterization gap in `grid.py` rather than a search failure. Correct fix is in
the *graph construction*, not the search.

### 3.3 Layer / side legality

```
sc ∈ {F,B}; pins of c live only on layer(sc).
A net's copper on a layer may only connect across layers through a via edge.
SMD pads are reachable only from their own side; through-hole pins from both.   (C-LAYER)
```

This is enforced structurally by which vertices/edges exist in `Γ` and by which layer a pin
projects to under `T_c`.

### 3.4 Per-net connectivity (counted, not always enforced)

```
For each net n: T(P_n) ∪ r_n is connected (single component) in the copper graph.  (C-CONN)
```

The number of nets violating (C-CONN) is `U`. The router *tries* to enforce it but the brief's
objective *counts* the residual — connectivity is a soft/relaxed constraint promoted into the
objective. This is the standard move when a constraint is infeasible-or-expensive: dualize it.

### 3.5 Per-side courtyard non-overlap (placement)

For components on the same side:

```
∀ c≠c' with sc=sc':  interior(courtyard(T_c)) ∩ interior(courtyard(T_{c'})) = ∅   (C-OVL)
```

`core.py` enforces this *softly* during gradient descent (the soft-overlap penalty, evaluated
**per side**) and *hard* at the end via the **Tetris legalizer**, which snaps components to a
non-overlapping arrangement. Formally: soft-overlap is a penalty relaxation of (C-OVL); Tetris
is a projection back onto the feasible (non-overlapping) set. Components on opposite sides do
not interact through (C-OVL) — hence "per-side overlap."

### 3.6 The diagonal-crossing EDGE constraint (the subtle one)

**Why cell-occupancy is insufficient.** Octile connectivity admits diagonal edges. Consider
the unit square of cells `(i,j), (i+1,j), (i,j+1), (i+1,j+1)`. Net `a` uses the edge
`(i,j)→(i+1,j+1)` (a "/" or "\" diagonal); net `b` uses the *crossing* diagonal
`(i+1,j)→(i,j+1)`. **No cell is shared** — `u ≤ cap` everywhere — yet the two 45° tracks
physically cross (an X) in the middle of the square. This is the measured **`tracks_crossing`**
tail: "45° diagonal X-crossings between adjacent cells — cell-occupancy doesn't forbid them."

Cell occupancy is a **vertex** capacity. The crossing is an **edge** conflict. A vertex model
is structurally incapable of expressing it. The correct formalization is an **edge-conflict
(edge-disjointness) constraint** on the two diagonal edges of each grid square:

```
For each grid square q with the two diagonals d+(q)={(i,j)-(i+1,j+1)} and
d-(q)={(i+1,j)-(i,j+1)}:
    (edge d+(q) used by some net)  AND  (edge d-(q) used by some net)  is FORBIDDEN.   (C-XCROSS)
```

Equivalently, introduce a **conflict resource** `χ(q)` per square with capacity 1, where
*both* diagonal edges consume `χ(q)`. Then (C-XCROSS) is just `usage(χ(q)) ≤ 1` — i.e. it
slots into the *same* shared-resource machinery as cell capacity, but the resource is the
square's crossing-slot, not a cell. This is the precise statement that the right model is a
**capacitated routing graph with both vertex and edge (conflict) resources**, not vertices
alone.

**Cheapness.** Enforcing (C-XCROSS) requires only that, when A* expands a diagonal edge
`d±(q)`, it checks whether the opposite diagonal `d∓(q)` is occupied (one extra lookup keyed
on the square id `q = (min(i,i'), min(j,j'))`). It is O(1) per diagonal expansion and needs no
new data structure beyond a per-square "diagonal-used" map parallel to the occupancy grid.

### 3.7 Board-inherent defects (NOT in the solver's feasible set)

Some DRC errors are properties of the *input* (footprint internal clearance violations,
physically-unroutable nets given fixed connector positions on 2 layers, pads inside a
mandated keepout). These are **infeasibilities of the instance**, not failures of the search.
They must be *partitioned out* of `D`:

```
D = D_solver + D_inherent
```

and only `D_solver` belongs in the objective. Reporting `D_inherent` separately (and to the
user) is both more honest and removes a chunk of the "long tail" that no amount of routing
effort can move. Failing to partition is, formally, optimizing over points outside the
feasible region — guaranteed to waste effort and to make `J` look stuck.

---

## 4. Problem class & complexity

### 4.1 What it is

TraceWise couples two hard problems:

1. **Placement** = a *mixed continuous–combinatorial packing* problem: place rigid polygons
   (with discrete orientation/side) without overlap to minimize a wirelength-plus-routability
   objective. The continuous relaxation is the **analytical-placement** program used by
   ePlace / RePlAce / DREAMPlace / Cypress: minimize `WL_smooth(Π) + λ·Density(Π)` by gradient
   descent, where `Density` is a (electrostatics-inspired) penalty driving overlap to zero.
   This is a smooth nonconvex program; it is what `place/core.py` is. Adding the discrete
   `θ, s` and exact non-overlap makes the *full* placement problem NP-hard (it contains
   geometric packing / quadratic assignment).

2. **Routing** = **geometric disjoint paths / min-cost multicommodity flow with conflicts**.
   Each net is a commodity that must be connected (a Steiner tree per net) on the shared
   capacitated graph `Γ`; nets compete for cell/edge resources (C-CLEAR, C-XCROSS). Two
   classical hardness results bracket it:
   - **Edge-disjoint paths** on a grid is NP-hard (Kramer–van Leeuwen; the diagonal version is
     our (C-XCROSS)). Even deciding routability of `k` two-terminal nets disjointly is NP-hard.
     `[CITE: Kramer & van Leeuwen 1984, "The complexity of wire-routing..."]`
   - **Rectilinear Steiner minimal tree** (single net, optimal topology) is NP-hard.
     `[CITE: Garey & Johnson 1977]`
   So routing alone — even *ignoring* placement — is NP-hard, and the *joint* place+route is at
   least as hard. There is no polynomial exact algorithm; everything real is a heuristic or a
   relaxation.

### 4.2 The standard relaxations (cited honestly)

- **Analytical placement as a Lagrangian / quadratic+density program.** ePlace/RePlAce cast
  placement as minimizing `WL + λ·(density penalty)`, the density term being a Poisson /
  electrostatic field; `λ` is ramped (a continuation / Lagrangian-style schedule). DREAMPlace
  is the GPU realization; Cypress (ISPD'25) adapts it to PCB. TraceWise's `core.py` is squarely
  in this family (log-sum-exp HPWL + soft-overlap density). `[CITE: Lu et al. ePlace 2015;
  Cheng et al. RePlAce 2019; Lin et al. DREAMPlace 2019; Cypress ISPD 2025]`

- **Routing as negotiated congestion (PathFinder) = Lagrangian dual on capacity.** PathFinder
  (Ebeling/McMurchie, FPGA routing) routes each net by shortest path on a graph whose vertex
  costs *rise with overuse*, iterating: nets initially share resources (infeasible), and a
  *history* + *present-congestion* price on each shared resource is increased each iteration
  until contention resolves. This is exactly a **subgradient ascent on the Lagrange multipliers
  of the capacity constraints** of the multicommodity-flow LP. The capacity constraints are
  *dualized into prices* rather than enforced as hard walls. `[CITE: McMurchie & Ebeling,
  PathFinder, FPGA 1995; this is also what OrthoRoute adapts to PCB on GPU — see research/46]`

- **The diagonal conflict as an edge-disjoint constraint** — §3.6 above; standard in detailed
  routing as a "conflict graph" / "design-rule conflict" edge in the routing resource graph.

### 4.3 Where TraceWise sits relative to these

The A* + connection-tree + bounded rip-up + power-first ordering router (`astar.py`,
`multi.py`) is a **sequential negotiated-congestion router *without the negotiation*** — it
rips up and reroutes (the "negotiation" of resources) but it treats capacity as a **hard wall**
(`cap=1`, blocked cell) rather than a **price that rises with overuse**. That single
difference — *forbid vs. price* — is the crux of §5.

---

## 5. THE ACTIONABLE PART

### 5.1 Mapping formal elements to what TraceWise already does

| Formal element | TraceWise component | Verdict |
|---|---|---|
| Continuous placement relaxation (`WL_smooth+density`, Lagrangian λ ramp) | gradient placer `core.py` (log-sum-exp HPWL, soft-overlap, congestion) | **Already best-practice.** This *is* ePlace/RePlAce/Cypress. Little headroom. |
| Projection back to feasible packing | Tetris legalizer | Standard. Fine. |
| Shortest-path / cost-to-go relaxation of routing | ECCF T1 cost-to-go fields | Good — a per-net relaxation. |
| Cross-net routability externality (∂J/∂Π for *other* nets) | ECCF T3 pseudo-route | **The right idea, under-exploited (see 5.4).** |
| Decomposition place→route→verify→refine | the "funnel" + `auto.py` keep-best loop | A coordinate-descent / decomposition heuristic. |
| Weighted defect objective | `5·U + D` in `auto.py` | A scalarized bi-objective; weight set by hand (see 5.5). |
| Routing search | A* octile, via moves, rip-up, power-first | **Negotiated-congestion *without* negotiation — the gap.** |
| Vertex capacity (`cap=1`) | counting occupancy in `grid.py` | Correct but **incomplete**: missing the edge/X-crossing resource (5.3). |

### 5.2 The candidates, evaluated (not just listed)

The brief lists four candidates. Verdict on each, then the build order.

**(a) PathFinder-style negotiated-congestion router — HIGH VALUE, the headline.**
*What it changes:* instead of forbidding a cell once used (`cap=1` wall), let nets *share*
resources at a *price*. Each resource `e` (cell, via, and crucially the X-crossing slot `χ(q)`
from §3.6) carries a cost
```
cost(e) = base(e) · (1 + h(e)·h_fac) · (1 + p(e)·p_fac)
```
where `p(e) = max(0, usage(e) − cap(e))` is **present congestion** and `h(e)` is **history**
(accumulated overuse across iterations). Route all nets by shortest path (the existing A*!)
using these costs; then **raise** `p_fac` and add overuse to `h(e)`; repeat. Early iterations
allow shorts (cheap to overlap); rising prices push nets onto alternative routes until no
resource is over capacity → a legal, short-free board, or a *certified* minimal residual set
of genuinely-contended resources.
*Why it directly attacks the measured walls:*
- It **prices conflicts instead of forbidding them**, so it never "forces a connection through
  clearance shaving" blindly — the price *tells* it which net should yield. This is the formal
  cure for the measured *more-time→more-violations* pathology: more iterations now *reduce*
  overuse monotonically in expectation (subgradient ascent), instead of trading `U` for `D`.
- It is **resource-agnostic**: putting the X-crossing slot `χ(q)` and a soft-clearance halo
  into the same price machinery makes shorting/clearance *and* `tracks_crossing` first-class,
  negotiable resources rather than post-hoc DRC errors.
- It **reuses the existing A*** — the only change is the edge-cost function and an outer
  iteration loop. This is a *bounded* change, not a rewrite. OrthoRoute proves PathFinder works
  for PCB (research/46), so this is de-risked.
*Predicted effect `[TARGET, fermi]`:* on zuluscsi, the *unconnected* count is dominated by
nets that lose every contention under a hard wall; negotiation lets them claim a resource and
push the loser to reroute, so `U` should fall substantially (rough estimate: ~48 → ~10–20,
since a large fraction of the 48 are contention losers, not topologically impossible). DRC
*shorting/clearance* errors are exactly the priced resources, so at convergence they go to the
*irreducible* set (board-inherent + genuinely over-capacity), plausibly `D_solver` from ~tens
→ single digits, with `D_inherent` reported separately. mitayi `U` ~63 → ~20–30 by the same
mechanism. These are order-of-magnitude predictions, to be replaced by measurement.

**(b) The diagonal-conflict edge constraint — HIGH VALUE, CHEAPEST, do it first regardless.**
§3.6. O(1) per diagonal expansion, kills the entire `tracks_crossing` error class by
construction. It is independently correct *and* it is a prerequisite for (a) (the X-crossing
slot must exist as a resource before you can price it). *Predicted effect:* removes
`tracks_crossing` from `D` essentially entirely `[TARGET]`; mild reduction in completion if the
router was relying on illegal crossings (which is why it belongs *inside* the negotiated
router, where the loser just reroutes).

**(c) Global net-ordering from a min-cost-flow relaxation — MEDIUM, mostly subsumed.**
Power-first ordering is a greedy proxy for "route the constrained/important nets first." A
min-cost multicommodity-flow LP relaxation would give a principled *fractional* ordering/route
prior. **But** negotiated congestion (a) *is* the iterative dual of that very LP, and it
removes most of the order-sensitivity (every net gets re-priced every round), so ordering
matters far less once (a) is in. Verdict: **do (a) first; revisit ordering only if a residual
order-sensitivity is measured.** Not worth a separate build now.

**(d) The combined objective as a true Lagrangian to set λ_U principally — MEDIUM, elegant.**
The weight `λ_U = 5` is a hand-tuned dual variable on the connectivity constraint (C-CONN).
Formally, the right `λ_U` is the **shadow price** of connectivity: increase `λ_U` only while it
buys net connections cheaper than the DRC errors it costs; stop when the marginal connection
costs more than 5 errors. This can be *automated* by treating `auto.py`'s loop as subgradient
updates on `λ_U` (raise it when `U>0` stalls, lower it when `D` balloons from forced
connections). It pairs naturally with (a) (same dual machinery). Verdict: **a cheap,
principled upgrade to `auto.py` to do *after* (a)+(b)**; turns a magic constant into a
self-tuning multiplier and removes the "lexicographic gaming" failure at its root (gaming is a
symptom of a *fixed* λ that the optimizer can exploit; a *responsive* λ closes the loophole).

### 5.3 The single most promising thing, with algorithm sketch

**Build a negotiated-congestion router (a) that prices a resource set including the X-crossing
slot (b), and let `auto.py` update `λ_U` as a dual (d).** Concretely, the minimal first build is
**(b) folded into (a)**: a PathFinder outer loop wrapping the existing A*.

```
# Resources: cells E_cell, via slots E_via, X-crossing slots χ(q) [§3.6],
#            and (optionally) clearance-halo slots around each used cell.
init  h(e) = 0 for all e;  p_fac = small
route every net once by A* on current costs (allow overuse)        # iteration 0
repeat:
    for e in resources:  h(e) += max(0, usage(e) - cap(e))         # history update
    p_fac *= growth        (e.g. ×1.3 per iteration)               # present-cost ramp
    rip up the nets that *touch any over-capacity resource* only   # not all nets
    re-route those nets by A* with cost(e)=base·(1+h·h_fac)·(1+p·p_fac)
until  (no resource over capacity)  or  (iteration budget hit)
report residual over-capacity resources as D_solver candidates;
       classify board-inherent → D_inherent (do not count against the solver)
# outer: if U>0 and stalled, auto.py raises λ_U; if D spikes from forced nets, lowers it.
```

The shortest-path inner solve is *exactly today's A*, with one swapped cost function and one
extra O(1) diagonal check.* Wall-clock is controlled by the same 45 s / expansion-cap budget,
now spent on *converging prices* instead of *blind rip-up*. This is the smallest change that
turns "forbid" into "price," which is the formal root cause of every measured routing wall.

### 5.4 A second, independent win the formal view exposes (ECCF-T3 as a gradient)

§2.2 showed ECCF-T3 is a cheap estimate of the cross-net routability externality
`∂J/∂Π`. Today placement moves "rarely beat the plain route (large global blast radius)"
because the placer optimizes `WL_smooth`, whose gradient is *blind to congestion the move
creates elsewhere*. The fix is not more placement search; it is **adding the T3 externality as
a term in the placer's gradient** (a congestion-aware wirelength à la RePlAce's routability
mode): `∇Φ ← ∇WL_smooth + ∇Density + λ_R·∇(T3 congestion estimate)`. This shrinks the blast
radius by making the placer *see* the routing cost it is about to inflict, locally. This is
also bounded (one extra gradient term, T3 already exists) and is the natural §3 placement
analogue of pricing in §5.3 routing. Recommended **after** the router work, because a better
router raises the bar a placement move must beat.

### 5.5 What is NOT worth building

- A from-scratch RL router (research/46 §D: infeasible on the target hardware; OrthoRoute
  needed an A100). Negotiated congestion gets PathFinder's benefits with none of the training.
- A true optimal-Steiner per-net router. NP-hard, and the connection-tree approximation is not
  the dominant error source; contention (cap-as-wall) is.
- Replacing the analytical placer. It is already the SOTA relaxation; the headroom is in
  *coupling* it to routability (5.4), not in the placer itself.

---

## 6. Honest verdict

**Is formalizing worth a build? Yes — and it points to one specific, bounded thing.**

The formal view does *not* say "your heuristic is silly." The placer is already the textbook
analytical relaxation (ePlace/Cypress); the keep-best loop is a sensible decomposition. The
one place where TraceWise diverges from the relaxation frontier is the router: it implements
**rip-up-reroute with capacity as a hard wall**, which is *negotiated congestion minus the
negotiation.* Every measured routing wall — more-time→more-violations, forced clearance
shaving, escape-on/off identical, no dominant lever — is the signature of a router that
*forbids* contention instead of *pricing* it. The formalization's payoff is that it identifies
this as a single root cause and names the cure (PathFinder / Lagrangian dual on capacity),
which is *known to work on PCB* (OrthoRoute) and *reuses the existing A* almost verbatim.*

**Build first, in this order:**

1. **(b) The X-crossing edge resource** — O(1), kills `tracks_crossing`, and is a prerequisite
   for pricing. *Smallest possible change; do it this week.*
2. **(a) Wrap the A* in a PathFinder negotiated-congestion outer loop** — prices cells, vias,
   X-slots, and a clearance halo; rip-up restricted to over-capacity nets. *This is the
   headline lever the project has not tried.*
3. **(d) Make `λ_U` a self-tuning dual in `auto.py`** — removes the magic constant and the
   lexicographic-gaming loophole.
4. **(5.4) Add the ECCF-T3 externality to the placer gradient** — only after the router
   improves, to shrink the placement blast radius.

If, after (b)+(a), the residual `U`/`D` is dominated by `D_inherent` (board-inherent footprint
and 2-layer-infeasible nets) rather than `D_solver`, then **the heuristic *is* at the
achievable frontier for those instances**, and the honest move is to *report the partition*
(§3.7) and stop optimizing the un-optimizable — not to chase the long tail. The first
measurement after building (b)+(a) is exactly the experiment that decides this, and the
formulation tells you which number to look at: `D_solver` (priced resources still over capacity
at convergence) vs `D_inherent` (infeasibilities of the instance).

---

## 7. References (as cited inline; verify against research/46)

- McMurchie, Ebeling. *PathFinder: A Negotiation-Based Performance-Driven Router for FPGAs.*
  FPGA 1995. (negotiated congestion = Lagrangian dual on capacity)
- OrthoRoute (bbenchoff) — PathFinder adapted to PCB on GPU. research/46 ref [2,3].
- Lu et al. *ePlace.* 2015; Cheng et al. *RePlAce.* TCAD 2019; Lin et al. *DREAMPlace.* DAC
  2019; *Cypress*, ISPD 2025 (research/46 ref [16,17]). (analytical placement + density)
- Kramer, van Leeuwen. *The complexity of wire routing and finding minimum area layouts.* 1984.
  (edge-disjoint paths NP-hard)
- Garey, Johnson. *The rectilinear Steiner tree problem is NP-complete.* SIAM 1977.

See `scripts/spike_formulation.py` for a numerical demonstration that negotiated congestion
resolves a 2-net conflict that a greedy/forbid-wall router shorts.
