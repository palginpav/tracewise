# Design: Phase B — Exact-geometry nudge integration into the route EMITTER

Status: design (2026-06-18). Implements the "#4 NEAR build" emit refactor described in
`docs/design/EXACT-GEOMETRY-ROUTER-ARCH.md` §"NEAR — Minkowski-snap emitter" and
§"Phase-A status". Phase A (the `exact_geom` primitive layer + 46 fixture tests) is DONE.
Phase B wires it into `emit_routes`.

## Overview

Today `emit_routes` (kicad.py) places every track endpoint and via at a 0.1 mm grid-cell
center via `grid.to_world(iy, ix)` (terminals are exception-snapped to their pad center).
PROBE-A measured that ~62 of 74 clearance-class DRC errors on mitayi are caused by these
quantized positions landing sub-pitch (<0.05 mm) from other-net copper, and that a bounded
(≤0.3 mm) nudge restores legal clearance. This design replaces the cell-center → world
computation with a **per-unique-cell nudge resolver** that places each emitted point at a
legal-clearance position while preserving connectivity (terminals stay on their own pad,
shared interior endpoints stay coincident, vias stay co-located with the run-ends they join).

Approach chosen: **resolve each unique cell to ONE world position before emitting any
segment/via**, keyed by cell, so coincident endpoints can never diverge. This is the single
most important decision — it is what prevents the refactor from manufacturing new
dangling-end / unconnected errors (the dominant risk).

Target (PROBE-A, conservative): mitayi 104 → ~42 errors, unconnected NOT worse than 48,
deterministic.

## Scope

- **Files to create:** none (extends an existing module + an existing test file).
- **Files to modify:**
  - `src/tracewise/route/engine/exact_geom.py` — add ONE new function
    `nudge_endpoint_in_region` (terminal-on-pad case; see §"exact_geom extension").
  - `src/tracewise/route/engine/kicad.py` — `build_problem` (assemble + return an
    `obstacles` structure + `anchor_rects`), `emit_routes` (consume it; per-cell nudge
    resolution), `route_board_engine` (thread `obstacles`/`anchor_rects` from build_problem
    into emit_routes, pass `clearance_mm`).
  - `tests/test_exact_geom.py` — fixture tests for `nudge_endpoint_in_region`.
- **Files to read (context only):**
  - `src/tracewise/route/engine/grid.py` — `to_world(iy,ix)`, `to_cell(x,y)` conventions.
  - `src/tracewise/route/engine/multi.py` — `NetRoute.paths`, `.escape_cells`, `.ok`;
    `Net` shape.
  - `src/tracewise/route/engine/astar.py` — `simplify(path) -> list[run]` (runs are
    per-layer node lists; `run[i][-1] == run[i+1][0]` in cell terms at via transitions).
  - `src/tracewise/place/extract.py` / the `PAD_SCRIPT` in kicad.py — pad fields available
    (`net`, `x`, `y`, `hw`, `hh`, `front`, `back`).
  - `scripts/probe_clearance_nudgeability.py` — the obstacle-building + clearance reference
    the runtime resolver must agree with.
  - `scripts/_probe_route_human.py`, `scripts/place_route_measure.py` — measurement harness.

## Prior-art / KB check

- `pattern_find` returned no matches (novel surface). `kb_search` unavailable this session.
- Prior decision consulted: EXACT-GEOMETRY-ROUTER-ARCH.md. This design **applies** the
  documented NEAR plug point (`emit_routes`, the `grid.to_world` lines) verbatim and the
  PROBE-A success target (→~42, no unconnected regression). It does NOT contradict any prior
  decision. It does NOT add Shapely (Phase A chose the numpy fallback; we stay numpy-only).

---

## Hard question 1 — Obstacle threading

### Where copper geometry lives today

`build_problem` already loops every pad in `data["pads"]`, has each pad's `net`, layer reach
(`front`/`back`), and computes the exact rect `(x-hw, y-hh, x+hw, y+hh)`. It currently uses
this only to (a) build A* goals/anchors and (b) inflate the grid. We add a third use: emit an
**obstacle catalog** keyed by layer, carried out of `build_problem` and into `emit_routes`.

### Data shape (new return value from `build_problem`)

`build_problem` gains a fourth return element `obstacles`, a structure that lets the emitter
cheaply ask "what OTHER-net copper is near point P on layer L?":

```
obstacles: dict[int, list[tuple[str, Obstacle]]]
    # key   = layer index (0 = F.Cu, 1 = B.Cu)
    # value = list of (net_name, exact_geom Obstacle tuple)
    # Obstacle for a pad = ("rect", x1, y1, x2, y2)   (the same rect already computed)
```

Net identity is carried as the first element of each pair (`net_name`). The emitter, when
nudging net N's endpoint on layer L, filters this to **other-net** copper:
`[obs for (m, obs) in obstacles[L] if m != N]`. This realises the rule "clearance applies
BETWEEN DIFFERENT NETS; same-net copper may touch" — same-net pads are excluded so the
track is never pushed away from a pad it is allowed to touch.

Build site (inside the existing `for p in data["pads"]` loop, where `rect` and `layers` are
already in scope):

```
for layer in layers:
    obstacles.setdefault(layer, []).append((p["net"], ("rect", *rect)))
```

Pads with empty `net` ("") are still emitted as obstacles (unnamed copper such as mounting
pads still needs clearance); they are filtered out only when they would match the routed net
(empty != any real net name, so they always remain obstacles — correct).

### What is intentionally NOT in the obstacle set (scope discipline, matches PROBE-A)

- **Already-emitted tracks/vias of the SAME route pass are not added.** PROBE-A measured 0
  pour-interaction and treated each violation against the *fixed* copper (pads). The dominant
  fixable class is endpoint-vs-pad and via-vs-pad. Adding live track-track obstacles would
  make the resolver order-dependent and not deterministic-by-construction (a later net would
  see earlier nets' nudged tracks). We keep the obstacle set = **fixed pad copper only** for
  Phase B. Track-track and via-via residue (mid-segment, tight-corridor, hole_to_hole) is the
  documented residual that PROBE-A excluded from the ~62 ceiling and that the FAR gridless
  build addresses. This is a deliberate scope boundary, stated as a risk below.
- **Pours/zones are not obstacles** — they refill *after* emit (`refill_zones`) with their
  own clearance; PROBE-A measured 0 pour interactions.

### Why not pass raw `data["pads"]` to the emitter instead

We could, but `build_problem` already computes the rect and knows the layer mapping; passing a
pre-built `obstacles` keeps the emitter free of pad-geometry knowledge and keeps a single
source of truth for the rect math (avoids a second, divergent `x±hw` computation).

---

## Hard question 2 — Connectivity invariant (the anchor trap)

### The trap

`nudge_endpoint(endpoint, anchor, …)` enforces `dist(pos, anchor) <= dist(endpoint, anchor)`.
A terminal endpoint is `snap`-ed exactly onto its own pad CENTER, so `endpoint == anchor` and
`dist == 0`. The anchor constraint then forbids ANY movement → terminals can never be nudged.
That is wrong: a terminal may move anywhere **within its own pad's copper rectangle** and
remain electrically connected (the track still lands inside the pad). We must free the terminal
to roam its own pad rect while maximising clearance to OTHER-net neighbours.

Decision: **(a) extend `exact_geom` with a region-constrained nudge.** Reusing
`nudge_endpoint` with different anchor semantics cannot express "stay inside this rectangle";
the anchor model is a radial bound, not a region. A pad rect is the natural allowed region and
gives the terminal full freedom inside its own copper. This is allowed — Phase A is done and
tested; extending it with a new, separately-tested function is in-scope.

### Three cases the emitter distinguishes (the core of question 2)

For each UNIQUE cell that appears in a net's runs, classify it and resolve a world position:

1. **Terminal endpoint** (cell ∈ `anchors` AND is the first/last node of the path; it sits on
   its OWN pad). Allowed region = that pad's copper rect. Use
   `nudge_endpoint_in_region(start_xy, pad_rect, other_net_obstacles, required, track_hw)`.
   `start_xy` = the snapped pad-center world coords (`anchors[cell]`). The result stays inside
   the pad (connected) and as far as possible from other-net copper. If no legal point exists
   inside the pad rect → keep `start_xy` (do not break connectivity).

2. **Intermediate endpoint** (an interior node shared by two adjacent segments of the same
   polyline, OR a non-terminal node that is not on a pad). The connectivity constraint here is
   NOT "stay near a pad", it is "the two segments meeting here move TOGETHER" (handled by
   continuity, question 3). So an intermediate cell uses `nudge_endpoint(cell_center_xy,
   anchor=None, …, max_nudge=0.3)` — the polyline stays continuous because BOTH segments read
   the same resolved position (question 3), and the bound `max_nudge` (=0.3, PROBE-A's window)
   keeps the nudged polyline from deviating enough to self-collide or overshoot. NOTE the
   anchor trap would bite here too if we passed `anchor=cell_center`; we deliberately pass
   `anchor=None` because the continuity guarantee, not a pad, is what keeps an intermediate
   node connected.

3. **Via position** (a cell that is a layer-transition point: `run[i][-1]` cell == `run[i+1][0]`
   cell, different layers). The via barrel is a single point shared by: the end of run *i*, the
   start of run *i+1*, and the via `(at …)`. All THREE must use the SAME resolved world
   position. Resolve the via cell ONCE with `anchor=None`, `track_hw = via_mm/2`, against
   other-net obstacles on BOTH layers it spans (union of `obstacles[0]` and `obstacles[1]`,
   filtered to other-net) — because a via is copper on both F.Cu and B.Cu and must clear
   neighbours on both. `required` = `clearance_mm` (hole_clearance is via-copper-to-copper at
   the same clearance rule; PROBE-A: 30/32 hole_clearance nudgeable). The resolved position is
   stored in the per-cell map so the joining segment endpoints AND the via all read it.

A cell that is simultaneously a terminal AND a via (via-in-pad, the R4 dangling class) keeps
the existing `emit_routes` short-circuit: `same_cell and tn in anchors and other in anchors`
→ no via emitted, endpoint resolved by the terminal rule. Do not change that branch.

### Determinism of classification

Classification is a pure function of the path/run structure and the `anchors` dict — both
deterministic. No ordering ambiguity.

---

## Hard question 3 — Polyline continuity (resolve-each-cell-once)

`simplify(path)` yields runs where `run[i]` end cell == `run[i+1]` start cell, and within a run
consecutive segments share endpoint cells. If we nudged each segment's endpoints independently,
a shared cell would get two different world positions and the track would visibly break →
DRC dangling-end / unconnected regression. This is THE risk the design exists to prevent.

### Algorithm: per-cell resolution map, computed ONCE per net-path

For each net, for each `path` in `nr.paths`:

```
runs = simplify(path)

# 1. Collect every unique cell that will be emitted, and classify it.
#    cell = (layer, iy, ix). Two nodes with the same (iy,ix) but different
#    layers are DIFFERENT cells for segment purposes but the SAME (iy,ix) for
#    a via — handle vias by (iy,ix) key, segments by full (layer,iy,ix) key.

resolved: dict[cell, (x, y)] = {}     # full (layer,iy,ix) -> world xy
terminals = {path[0], path[-1]}       # the two pad-anchored endpoints

for each unique cell c in all runs:
    others = other_net(obstacles[c.layer], net)
    if c in anchors and c in terminals:
        rect = anchor_rects[c]                 # OWN-net pad rect
        xy, ok = nudge_endpoint_in_region(anchors[c], rect, others, clr, track_hw)
        resolved[c] = xy                       # ok=False -> xy == anchors[c] (unchanged)
    else:                                       # intermediate
        cx = grid.to_world(c.iy, c.ix)
        xy, ok = nudge_endpoint(cx, None, others, clr, track_hw, max_nudge=0.3)
        resolved[c] = xy                       # ok=False -> xy == cx (unchanged)

# 2. Resolve via cells (by (iy,ix)) ONCE, overriding the per-layer entries so
#    both layers' coincident endpoints + the via all share one position.
for i in range(len(runs) - 1):
    tn, other = runs[i][-1], runs[i+1][0]
    if tn[1:] == other[1:]:                     # same (iy,ix): a real via cell
        cx = grid.to_world(tn[1], tn[2])
        others = other_net(obstacles[0], net) + other_net(obstacles[1], net)
        vxy, ok = nudge_endpoint(cx, None, others, clr, via_hw, max_nudge=0.3)
        resolved[tn] = vxy                       # end of run i (layer A)
        resolved[other] = vxy                    # start of run i+1 (layer B)
        via_pos[(tn[1], tn[2])] = vxy            # the via barrel

# 3. Emit segments by LOOKING UP resolved[cell] — never recompute.
for run in runs:
    for a, b in zip(run, run[1:]):
        xa, ya = resolved[a]
        xb, yb = resolved[b]
        ...emit segment...

# 4. Emit vias from via_pos.
```

Because every segment endpoint reads `resolved[cell]` and the via reads `via_pos[(iy,ix)]`
which was written into `resolved` for both joining nodes, **coincident endpoints are
coincident by construction** — they are literally the same dict value. There is no second
code path that can recompute and diverge. This is the mechanical guarantee against new
dangling ends.

### Replacing the existing snap logic

The current `snap` dict and the inline `snap.get(a) or grid.to_world(...)` are SUPERSEDED by
`resolved[a]`. Terminal snapping is now subsumed: a terminal's `resolved` entry starts at the
pad center (= old snap value) and is then nudged within the pad rect. If `obstacles` is None /
empty (back-compat path, e.g. a caller that does not pass obstacles), `resolved[c]` falls back
to `anchors.get(c) or grid.to_world(...)` with no nudge — byte-identical to today's behaviour.

---

## Hard question 4 — Vias

Covered above (case 3 + step 2). Specifics:

- Via half-width for clearance = `via_mm / 2` (the copper annulus radius). track_hw for the
  joining segments stays `track_mm / 2`.
- A via clears OTHER-net copper on BOTH layers → obstacle set is the union over layers 0 and 1,
  filtered to other-net.
- The via, the end of the incoming run, and the start of the outgoing run all read the SAME
  resolved position (they were the same `(iy,ix)` cell). This keeps the via co-located with
  both track runs it joins — no dangling barrel.
- The existing `net_vias` dedupe ("one barrel per position per net") still applies, now keyed
  by the resolved `(vx, vy)`.
- Via-in-pad (terminal that is also a transition) keeps the existing no-via short-circuit.

---

## Hard question 5 — Determinism & no-regression

- `nudge_endpoint` and `nudge_endpoint_in_region` are deterministic (fixed analytic step +
  fixed polar grid; no randomness). Obstacle lists are built in pad-iteration order (stable).
  `simplify` and `anchors` are deterministic. Therefore emit is deterministic: same board in →
  byte-identical board out.
- `required` clearance = `project_geometry`'s `clearance_mm` (mitayi: 0.15). `track_hw =
  track_mm / 2` (mitayi 0.2/2 = 0.1). `via_hw = via_mm / 2`. These are threaded from
  `route_board_engine` (which already calls `project_geometry`) into `emit_routes`.
- `max_nudge = 0.3` (PROBE-A's measured window; the conservative bucket assumed exactly this).
- **No-regression mechanism:** partial-failure (a fully-boxed pad or via with no legal point in
  range) returns `ok=False`; the resolver then keeps the ORIGINAL position (`anchors[c]` for
  terminals, `grid.to_world` for the rest). Leaving a point as-is can never CREATE an
  unconnected error — it reproduces today's geometry for that point. Continuity is preserved
  because the unchanged point is still written once into `resolved`.

### Measurement plan (BEFORE / AFTER, copy from the probe harness)

Route mitayi human placement before and after the change and diff the scorecard.

1. BEFORE (capture baseline on current `main`):
   ```
   taskset -c 0-9 .venv/bin/python scripts/_probe_route_human.py \
       data/benchmark-boards/mitayi-pico-d1 /tmp/phaseB_before
   ```
   Run in background (~176s). Then DRC + scorecard the routed board:
   ```
   .venv/bin/python scripts/probe_clearance_nudgeability.py \
       /tmp/phaseB_before/<board>.kicad_pcb /tmp/phaseB_before/<board>.drc.json
   ```
   Record: unconnected, total errors, errors by-type (clearance / hole_clearance /
   solder_mask_bridge / hole_to_hole). Expected baseline: 48 unc / 104 err / 74 clearance-class.
2. AFTER (same command into `/tmp/phaseB_after` with the branch checked out). Run in background.
3. Compare. **Success criteria (PROBE-A):**
   - errors materially down toward ~42 (clearance + hole_clearance shrink; the ~62 conservative
     ceiling minus residual). A drop to ≤ ~62 is a clear win; ~42 is the target.
   - unconnected ≤ 48 (NOT worse). A single new unconnected = continuity bug → block/rollback.
   - deterministic: run AFTER twice; the two routed boards must be byte-identical.

### Rollback plan

The change is gated by the presence of `obstacles`. If AFTER shows unconnected > 48 or errors
do not improve, the developer can (a) revert the `route_board_engine` line that passes
`obstacles=` (emit falls back to today's exact behaviour) for a zero-risk disable, or (b)
`git revert` the kicad.py commit; `exact_geom.py` + tests can stay (additive, harmless).

---

## exact_geom extension (signature + tests)

### New public function

```python
def nudge_endpoint_in_region(
    start: Point,                      # current point (pad-center world xy)
    allowed_rect: tuple[float, float, float, float],  # (x1,y1,x2,y2) the OWN pad copper
    obstacles: list[Obstacle],         # OTHER-net copper only (caller filters)
    required_clearance: float,
    track_hw: float,
) -> tuple[tuple[float, float], bool]:
    """Find a position INSIDE allowed_rect that maximises clearance to obstacles.

    Connectivity model: the point represents a track terminal sitting on its own
    pad; it stays electrically connected as long as it remains inside allowed_rect
    (the pad copper). Within that rect it is free to move to gain clearance to
    OTHER-net copper.

    Algorithm (deterministic, mirrors nudge_endpoint):
      1. If start already legal (min_clearance(start) >= required) -> (start, True).
      2. Analytic push-out from the worst obstacle, THEN clamp the candidate back
         into allowed_rect (clamp x to [x1,x2], y to [y1,y2]); accept if legal.
      3. Polar-grid refinement: same POLAR_ANGLES x _POLAR_RADIUS_FRACTIONS grid
         centred on start, but each candidate is CLAMPED into allowed_rect before
         the legality test; first legal clamped candidate wins. (Clamping, not
         rejection, so a candidate that points outside the pad still contributes
         its in-pad projection — important for thin pads.)
      4. Failure -> (start, False).   start is always inside allowed_rect by
         construction, so the failure case is connectivity-safe.

    No max_nudge parameter: the pad rect IS the bound (a pad is small; the rect
    constrains displacement more tightly than 0.3mm would).
    """
```

Rationale for clamp-into-rect (vs reject-outside): a thin pad may be narrower than the
push-out distance; rejecting outside-rect candidates would fail every nudge on thin pads,
whereas clamping projects the candidate onto the nearest legal in-pad point and frequently
succeeds. The pad rect is the hard connectivity boundary, so clamping is always safe.

### New fixture tests (add to `tests/test_exact_geom.py`, new class `TestNudgeEndpointInRegion`)

1. `test_start_already_legal_inside_rect_unchanged` — start far from obstacles → returns
   start, True.
2. `test_nudges_within_rect_to_gain_clearance` — start at pad center 0.0998 mm from an
   other-net pad (the mitayi case), required 0.15, pad rect large enough → returns a point
   INSIDE the rect with clearance ≥ required; assert `x1<=x<=x2 and y1<=y<=y2` AND
   `min_clearance >= required`.
3. `test_result_always_inside_allowed_rect` — even on failure, assert the returned point is
   within `allowed_rect` (connectivity invariant).
4. `test_thin_pad_clamps_not_rejects` — narrow rect (e.g. 0.1 mm wide) where the unclamped
   push-out would land outside; assert result is inside rect and (if legal exists) legal.
5. `test_hopelessly_boxed_returns_start_false` — obstacles surround the rect so no in-rect
   point is legal → returns (start, False) and start unchanged.
6. `test_determinism` — identical inputs twice → identical output.
7. `test_asymmetric_coords_anti_swap` — rect and obstacle with x≠y to catch an x/y swap
   (mirror the existing anti-swap test style).

All assert exact, hand-checked numbers where feasible (consistent with the existing 46-test
style). Run `.venv/bin/python -m pytest tests/test_exact_geom.py -q` and `ruff check`.

---

## emit_routes change sketch (pseudocode, not implementation)

```python
def emit_routes(board, grid, results, track_mm=0.2, via_mm=0.6, via_drill_mm=0.3,
                anchors=None, neck_mm=None,
                obstacles=None, anchor_rects=None, clearance_mm=0.2):   # NEW params
    ...
    track_hw = track_mm / 2.0
    via_hw = via_mm / 2.0
    for name, nr in results.items():
        if not nr.ok or net_atom(name) is None:
            continue
        net_vias = set()
        for path in nr.paths:
            runs = simplify(path)
            terminals = {path[0], path[-1]}
            resolved = {}        # (layer,iy,ix) -> (x,y)
            via_pos = {}         # (iy,ix) -> (x,y)

            # --- resolve every unique cell ONCE ---
            for c in unique_cells(runs):
                if obstacles is None:                      # back-compat: today's behaviour
                    resolved[c] = (anchors.get(c) if anchors else None) \
                                   or grid.to_world(c[1], c[2])
                    continue
                others = [o for (m, o) in obstacles.get(c[0], []) if m != name]
                if anchors and c in anchors and c in terminals:
                    rect = anchor_rects[c]                 # OWN-net pad rect
                    xy, _ = nudge_endpoint_in_region(anchors[c], rect, others,
                                                     clearance_mm, track_hw)
                else:
                    cx = grid.to_world(c[1], c[2])
                    xy, _ = nudge_endpoint(cx, None, others, clearance_mm, track_hw, 0.3)
                resolved[c] = xy

            # --- resolve via cells once, override both joining endpoints ---
            for i in range(len(runs) - 1):
                tn, other = runs[i][-1], runs[i+1][0]
                if tn[1:] == other[1:]:                    # real via (same iy,ix)
                    if obstacles is None:
                        v = (anchors.get(tn) if anchors else None) or grid.to_world(tn[1], tn[2])
                    else:
                        others = ([o for (m, o) in obstacles.get(0, []) if m != name]
                                  + [o for (m, o) in obstacles.get(1, []) if m != name])
                        v, _ = nudge_endpoint(grid.to_world(tn[1], tn[2]), None,
                                              others, clearance_mm, via_hw, 0.3)
                    resolved[tn] = v
                    resolved[other] = v
                    via_pos[(tn[1], tn[2])] = v

            # --- emit segments from resolved (necking logic unchanged) ---
            for run in runs:
                if len(run) < 2: continue
                layer = layer_name[run[0][0]]
                for a, b in zip(run, run[1:]):
                    xa, ya = resolved[a]; xb, yb = resolved[b]
                    w = neck_width(a, b, nr, track_mm, neck_mm)   # unchanged
                    root.insert(node("segment", ...))
                    segs += 1

            # --- emit vias from via_pos (dedupe + via-in-pad short-circuit unchanged) ---
            for i in range(len(runs) - 1):
                tn, other = runs[i][-1], runs[i+1][0]
                if not (tn[1:] == other[1:]): continue
                if anchors and tn in anchors and other in anchors:
                    continue                                # via-in-pad: no barrel
                vx, vy = via_pos[(tn[1], tn[2])]
                if (vx, vy) in net_vias: continue
                net_vias.add((vx, vy))
                root.insert(node("via", node("at", f"{vx:.3f}", f"{vy:.3f}"), ...))
                vias += 1
    write_file(root, board)
    return {"segments": segs, "vias": vias}
```

`anchor_rects[c]` resolves the OWN-net pad rect for a terminal cell. Source: `build_problem`
exposes `anchor_rects: dict[cell, rect]` alongside `anchors` (same loop, one extra dict write:
`anchor_rects[(layer,*cell)] = rect`). This keeps the terminal rect lookup O(1) and exact, and
avoids a second divergent `x±hw` computation in the emitter.

`route_board_engine` change: capture the new returns and pass them through:
```
grid, nets, anchors, obstacles, anchor_rects = build_problem(...)   # was: grid, nets, anchors
...
emit_routes(board, grid, results, track_mm=geo["track_mm"], via_mm=geo["via_mm"],
            via_drill_mm=geo["via_drill_mm"], anchors=anchors, neck_mm=geo["min_track_mm"],
            obstacles=obstacles, anchor_rects=anchor_rects, clearance_mm=geo["clearance_mm"])
```
**`build_problem` caller audit (DONE — grep `build_problem(` in src/scripts/tests):** the
3-tuple → 5-tuple change breaks every site that unpacks `grid, nets, anchors` or `grid, _, _`:
- `src/tracewise/route/engine/kicad.py:300` (production, `route_board_engine`) — update to
  5-tuple, the point of this change.
- `src/tracewise/route/engine/eccf.py:120` and `:273` — both do `grid, _, _ = build_problem(...)`;
  must become `grid, _, _, _, _ = build_problem(...)`.
- `scripts/validate_eccf.py:33` (`grid, nets, anchors`) and `scripts/validate_eccf_v2.py:47`
  (`grid, _, _`) — update unpack arity.
- NOT affected: `src/tracewise/__main__.py:112`, `scripts/place_route_measure.py:59`,
  `tests/test_place.py` — these call the PLACEMENT `tracewise.place.core.build_problem`, a
  different function. Do not touch them.

To avoid the multi-site unpack churn AND the risk of a missed caller, the developer MAY instead
return a small dataclass/named structure or append the two new fields only when needed — but the
simplest, explicitly-audited path is the 5-tuple with the four edits above. The
`obstacles`/`anchor_rects`/`clearance_mm` params on `emit_routes` default to `None`/`0.2` so
emit's own old call sites keep working.

---

## Interface Contracts

```python
# build_problem now returns a 5-tuple (was 3):
def build_problem(data, pitch=0.1, track_mm=0.2, clearance_mm=0.2
) -> tuple[Grid, list[Net],
           dict[tuple[int,int,int], tuple[float,float]],          # anchors (unchanged)
           dict[int, list[tuple[str, tuple]]],                    # obstacles: layer -> [(net, Obstacle)]
           dict[tuple[int,int,int], tuple[float,float,float,float]]]:  # anchor_rects: cell -> pad rect

# emit_routes gains keyword-defaulted params (back-compat preserved):
def emit_routes(board, grid, results, track_mm=0.2, via_mm=0.6, via_drill_mm=0.3,
                anchors=None, neck_mm=None,
                obstacles=None,        # dict[int, list[(net, Obstacle)]] | None
                anchor_rects=None,     # dict[cell, rect] | None
                clearance_mm=0.2,      # required clearance (mm)
) -> dict: ...   # unchanged {"segments": int, "vias": int}

# exact_geom new function (see full contract above):
def nudge_endpoint_in_region(start, allowed_rect, obstacles, required_clearance, track_hw
) -> tuple[tuple[float,float], bool]: ...
```

Obstacle tuple shapes are EXACTLY the existing `exact_geom` tagged union: pads emit
`("rect", x1, y1, x2, y2)`. No new obstacle kind is introduced.

---

## Risks and Mitigations

| Risk | Likelihood | Impact | Mitigation |
|------|-----------|--------|------------|
| Nudging a shared interior endpoint independently breaks the polyline → NEW dangling-end / unconnected errors | Med | High | The per-cell `resolved` map: every coincident endpoint reads the SAME dict value; there is no second code path that recomputes. Measurement gate: unconnected must be ≤ 48 or rollback. |
| Endpoint can't escape a fully-boxed pad/corridor (PROBE-A's 6 tight-corridor + 3 inherent cases) | High | Low | `nudge_*` returns `ok=False`; resolver keeps the ORIGINAL position (leave-as-is). Reproduces today's geometry for that point — never creates a new error, never breaks connectivity. |
| Via nudged off the track runs it joins | Low | High | Via cell resolved ONCE; both joining run-endpoints AND the via barrel read the same `via_pos`. Via-in-pad short-circuit unchanged. |
| Resolver disagrees with DRC's clearance math (rotated pads approximated as axis-aligned rects) | Med | Low | Same approximation PROBE-A used to derive ~62; accept it for Phase B (matches the measured ceiling). Residual rotated-pad cases fall into the "leave-as-is" bucket, not a regression. |
| Mid-segment clearance unaddressed (a long grid-routed segment still violates mid-path) | Med | Low | By design (PROBE-A: only 3 mid-segment of 74). Documented residual for the FAR build; not a regression vs today. |
| `build_problem` 5-tuple change breaks a caller that unpacks 3 values | Med | Med | Audited (see caller audit above): `eccf.py:120,273` and `scripts/validate_eccf*.py` unpack `grid, _, _` and MUST be widened to 5 (else `ValueError: too many values to unpack` at import/run). `place.core.build_problem` callers are a different function — untouched. |
| Non-determinism from including live tracks as obstacles | Low (excluded) | High | Obstacle set is FIXED pad copper only — no live-route feedback. Stated scope boundary. Determinism test: run AFTER twice, byte-identical. |

---

## Testing Strategy

- **Unit (exact_geom):** the 7 new `nudge_endpoint_in_region` fixtures above (in-rect result
  always, mitayi case legal, thin-pad clamp, boxed-in failure-safe, determinism, anti-swap).
  Run full `tests/test_exact_geom.py` (53 tests) — all green; `ruff check` clean.
- **Integration (emit_routes):** a small synthetic test — a 2-run path with a shared interior
  cell and a via cell, plus one other-net pad obstacle; assert (a) the two segments meeting at
  the shared cell have IDENTICAL coincident coords in the emitted board, (b) the via `(at)`
  equals both adjacent segment endpoints, (c) with `obstacles=None` the output is byte-
  identical to pre-change emit (back-compat). Use a tiny in-memory `Grid` + hand-built
  `NetRoute` (no pcbnew needed; `parse_file`/`write_file` on a temp board fixture).
- **Manual / measurement:** the BEFORE/AFTER mitayi scorecard run (question 5). The pass bar:
  errors materially down toward ~42, unconnected ≤ 48, AFTER deterministic across two runs.

---

## Developer task list (ordered)

1. Add `nudge_endpoint_in_region` to `exact_geom.py` per the contract above (analytic
   push-out + clamp-into-rect + polar refinement; deterministic; no `max_nudge`).
2. Add the 7 `TestNudgeEndpointInRegion` fixtures to `tests/test_exact_geom.py`; run
   `pytest tests/test_exact_geom.py -q` and `ruff check` until green.
3. In `build_problem` (route.engine.kicad): build `obstacles: dict[int, list[(net,
   ("rect",*rect))]]` and `anchor_rects: dict[cell, rect]` inside the existing pad loop; return
   the 5-tuple. Widen the four audited unpack sites: `eccf.py:120`, `eccf.py:273`,
   `scripts/validate_eccf.py:33`, `scripts/validate_eccf_v2.py:47` (all become 5-element
   unpacks). Do NOT touch the `place.core.build_problem` callers (`__main__.py`,
   `place_route_measure.py`, `test_place.py`) — different function.
4. In `emit_routes`: add `obstacles`, `anchor_rects`, `clearance_mm` params (defaulted for
   back-compat); replace the `snap`/`grid.to_world` endpoint+via computation with the per-cell
   `resolved` map + `via_pos` resolution; emit segments/vias from the lookups. Keep necking,
   `net_vias` dedupe, and via-in-pad short-circuit unchanged.
5. In `route_board_engine`: unpack the 5-tuple; pass `obstacles`, `anchor_rects`,
   `clearance_mm=geo["clearance_mm"]` to `emit_routes`.
6. Add the integration test (shared-cell coincidence, via coincidence, `obstacles=None`
   byte-identical). Run the full suite + `ruff check`.
7. MEASURE: run the BEFORE baseline (current main) and AFTER (this branch) mitayi human-route
   in the background per question 5; capture scorecards; run AFTER twice for determinism.
   Confirm errors ↓ toward ~42 and unconnected ≤ 48. If not, apply the rollback gate.

## Acceptance Rubric

- [ ] AR1 — `exact_geom.nudge_endpoint_in_region(start, allowed_rect, obstacles,
  required_clearance, track_hw)` exists with the documented signature and returns
  `((x,y), bool)`. Evidence: function def + a passing import in the test module.
- [ ] AR2 — On failure (`ok=False`) AND on success, the returned point is always inside
  `allowed_rect`. Evidence: `test_result_always_inside_allowed_rect` passes.
- [ ] AR3 — `nudge_endpoint_in_region` is deterministic: identical inputs → identical output.
  Evidence: `test_determinism` passes.
- [ ] AR4 — `build_problem` returns `obstacles` keyed by layer with `(net_name, Obstacle)`
  pairs and `anchor_rects` keyed by cell. Evidence: unpacking the 5-tuple + a test asserting an
  other-net pad appears in `obstacles[layer]` with its net name.
- [ ] AR5 — Every coincident endpoint (shared interior cell, via-joined run ends) is emitted at
  ONE identical world position. Evidence: integration test asserts byte-equal coords for the
  shared cell and the via `(at)`.
- [ ] AR6 — With `obstacles=None`, `emit_routes` output is byte-identical to pre-change
  behaviour. Evidence: back-compat integration test passes.
- [ ] AR7 — Partial failure (boxed-in pad/via) leaves the point at its original position;
  no new unconnected is introduced. Evidence: measurement shows unconnected ≤ 48.
- [ ] AR8 — Emit is deterministic end-to-end: routing mitayi AFTER twice yields byte-identical
  boards. Evidence: diff of the two AFTER `.kicad_pcb` files is empty.
- [ ] AR9 — mitayi human-placement errors drop materially toward ~42 (clearance-class shrinks)
  vs the 104 baseline. Evidence: BEFORE/AFTER scorecard diff.
- [ ] AR10 — Full test suite green and `ruff check` clean after all changes. Evidence: pytest +
  ruff output.

---

## Phase-B RESULT (measured 2026-06-18) — PARTIAL WIN + clear diagnosis

Implemented and measured on mitayi human placement (BEFORE = `/tmp/probe_mitayi_human`,
AFTER = `/tmp/probe_mitayi_after`). Full suite green (208 tests), ruff clean, `obstacles=None`
byte-identical to pre-change emit.

Scorecard: errors **104 → ~91** (run1 92 / run2 90), unconnected **48 → 48** (no regression).
Clearance-class **74 → ~62**. Breakdown by counterpart type (the key insight):

| counterpart        | type           | BEFORE | AFTER | Δ |
|--------------------|----------------|-------:|------:|---:|
| pad ↔ track        | clearance      | 18 | **3** | **−15 ✅** |
| pad ↔ via          | clearance      | 16 | 18 | +2 ❌ |
| pad ↔ via          | hole_clearance | 32 | 34 | +2 ❌ |
| track ↔ via        | clearance      |  8 |  7 | −1 |

**What worked:** the terminal track-endpoint → pad nudge (`nudge_endpoint_in_region`) is the core
#4 mechanism and it nailed its target: **pad↔track clearance 18 → 3** (the 3 residual are the
mid-segment / boxed-in cases PROBE-A predicted). This is robust across both routes.

**What didn't (3 root causes — these reconcile the −14 actual vs PROBE-A's optimistic −62):**
1. **Via nudging is NET-NEGATIVE (+4).** It optimizes COPPER clearance (track_hw, 0.15mm) but
   ignores DRILL `hole_clearance` (0.25mm drill-to-hole), so it pushes vias into positions that
   satisfy copper clearance while *worsening* hole_clearance. Fix: via nudge must respect the
   larger hole-clearance against drill/hole geometry (and hole-to-hole), or be gated off (keeping
   the clean track-endpoint win). PROBE-A counted these as nudgeable; they are not, under a
   copper-only objective.
2. **Obstacle set is PADS-ONLY (by design, for determinism).** track↔via (7-8) and any track↔track
   cases cannot be nudged because other-net tracks/vias are not in the obstacle set. PROBE-A's
   exact-distance ceiling included them. Reaching the ~42 target requires expanding obstacles to
   emitted tracks/vias (reintroduces ordering/determinism work — closer to the FAR gridless build).
3. **Router non-determinism observed** (run1 92 vs run2 90, coords differ). `multi.py` has a
   wall-clock `time_budget_s=600` deadline; mitayi routes in ~176s so the deadline isn't the cause —
   source is elsewhere and UNVERIFIED whether pre-existing (RESUME claimed determinism restored) or
   Phase-B-introduced (nudge fns are unit-test-deterministic). MUST be confirmed before trusting
   ±2 deltas. OPEN.

**Recommended next step (focused):** (a) gate via nudging OFF or fix it to honor hole_clearance +
hole-to-hole drill geometry — this removes the +4 regression and isolates the clean ~−15 track win;
(b) settle the determinism question (route the SAME code twice, compare); (c) only then consider
expanding the obstacle set to tracks/vias (bigger, determinism-sensitive — likely folds into the FAR
gridless build). The pad↔track result confirms the exact-geometry approach is sound where its
obstacle model is complete.

---

## Deferred: hole_clearance-aware via nudge

**Status:** gated off (`nudge_vias=False` default in `emit_routes` / `route_board_engine`).
Phase-B measured that copper-only via nudging is NET-NEGATIVE (+4 hole_clearance regression).
This section documents what a correct via nudge requires before it can be re-enabled.

### What a correct via nudge needs

1. **Drill/hole geometry in the obstacle set.** Via drill holes (not copper annuli) are governed
   by `hole_clearance` (≈ 0.25 mm on mitayi, vs. the copper `clearance_mm` ≈ 0.15 mm). The
   current nudge objective is copper-only (`via_hw = via_mm / 2`), so it optimizes edge-to-edge
   copper distance while ignoring the drill-to-copper and drill-to-drill constraints that KiCad
   DRC actually checks under `hole_clearance` and `hole_to_hole`.

2. **Correct half-width for via nudge = hole radius, not copper radius.**
   The nudge call for a via must use `hw = via_drill_mm / 2` (drill radius, typically 0.15 mm)
   when computing distance to pad copper obstacles, and `required_clearance = hole_clearance`
   (typically 0.25 mm). This makes the nudger respect the drill-to-pad-copper distance that
   KiCad's `hole_clearance` rule measures, not the copper-annulus-to-pad-copper distance.

3. **Drill-to-drill (hole_to_hole) spacing.** Other vias' drills must appear as circle obstacles
   with radius `via_drill_mm / 2`, and the required clearance for the nudge must be
   `hole_to_hole_clearance` (typically 0.25 mm). The current obstacle set contains only pad
   copper rectangles — no via drill circles. Adding via drill obstacles requires either
   (a) a pre-pass that collects all placed vias' positions before nudging starts (ordering-
   sensitive, non-trivial), or (b) a post-routing obstacle set built from the emitted board.

4. **Through-hole pad drills as obstacles.** Through-hole pads have drill holes too; their
   drill circles (`(drill_diameter/2)` radius) must appear in the obstacle set for `hole_clearance`
   checks. Currently only pad copper rects are in `obstacles`; drill geometry is absent.

### Summary: re-enable condition

`nudge_vias=True` is safe to re-enable ONLY when the via nudge:
- uses `hw = via_drill_mm / 2` and `required_clearance = hole_clearance` (from project rules)
- has an obstacle set that includes pad drill circles (for `hole_clearance` vs. pads)
- has an obstacle set that includes other vias' drill circles (for `hole_to_hole` spacing)
- has been measured to NOT regress `hole_clearance` or `hole_to_hole` counts vs. the gated baseline

Until then the track-endpoint nudge (`nudge_endpoint_in_region`) remains active and delivers
the measured −15 pad↔track clearance win, while via positions revert to the grid.to_world
snap (pre-Phase-B behaviour for vias — no regression).

---

## Phase-B FINAL (via-nudge GATED OFF, 2026-06-18) — clean deterministic win

Applied recommendation #1: `emit_routes(..., nudge_vias=False)` is the new default (threaded through
`route_board_engine`). Terminal track-endpoint → pad nudge stays ON; via nudging is gated OFF
(preserved behind `nudge_vias=True` for a future hole_clearance-aware fix — see "Deferred" section).

Measured mitayi human placement, TWO runs, **bit-identical (88/88)**:

| metric          | BEFORE (#4 off) | Phase-B via-on | **Phase-B GATED (final)** |
|-----------------|----------------:|---------------:|--------------------------:|
| unconnected     | 48              | 48             | **48** (no regression)    |
| errors          | 104             | ~91            | **88 (−16)**              |
| clearance       | 42              | 27-28          | **27** (pad↔track 18→~3)  |
| hole_clearance  | 32              | 34 ❌          | **32** (regression gone)  |
| solder_mask_brdg| 16              | 16             | 15                        |
| hole_to_hole    | 14              | 14             | 14                        |

Full suite green (208), ruff clean, `obstacles=None` byte-identical to pre-#4 emit.

**Determinism — RESOLVED:** the two clean gated routes are bit-identical → emit is deterministic by
construction AND mitayi routing is deterministic (it completes well under the wall-clock deadlines).
The earlier "92 vs 90" was a MEASUREMENT ARTIFACT (a background route + an interactive route
colliding on the same output dir without an intervening strip — also produced a spurious 197-error
run). The wall-clock deadlines in `astar.py` (per-net `max_seconds`) and `multi.py` (`time_budget_s`)
remain a LATENT non-determinism risk for *denser* boards that actually hit them — a separate,
out-of-#4 router hardening task (replace wall-clock truncation with deterministic expansion/iteration
caps, per the "never wall-clock-truncate" rule). Probe scripts should also guard against re-routing a
non-stripped board (the artifact above).

**#4 NEAR build outcome:** mitayi **104 → 88**, deterministic, no unconnected regression. The
exact-geometry track-endpoint emit is the validated, shipped win. Remaining legality gap (vias,
track↔track, mid-segment) needs either a hole_clearance-aware via nudge (bounded next step) or the
obstacle-set expansion to tracks/vias — which folds into the FAR gridless router (the connectivity
unlock). Recommendation #3 (obstacle expansion) is intentionally deferred to that chapter.

---

## Phase-C PROBE (2026-06-18) — hole-aware via nudge is LOW-VALUE; #4 is at its ceiling

Before building the deferred hole_clearance-aware via nudge, probed it (`scripts/probe_via_nudge_holeaware.py`,
numpy-only, reuses exact_geom) on the gated board `/tmp/probe_gated_1` (88 errors). For each of the
56 via-involved residual violations, searched via positions within R=0.3mm (the emitter's bound) for
a position satisfying BOTH copper clearance (~0.15) to other-net copper AND hole_clearance (~0.25)
drill-to-hole to ALL nearby holes (the full multi-constraint test the previous via nudge lacked):

| sub-type                | via-nudgeable | boxed-in | stale |
|-------------------------|--------------:|---------:|------:|
| clearance pad↔via       | 0  | 14 | 1 |
| clearance track↔via     | 1  |  8 | 0 |
| hole_clearance pad↔via  | 0  | 30 | 2 |
| **total (of 56)**       | **1** | **52** | **3** |

**Ceiling: ~1 of 56 → mitayi 88 → ~87 (one error).** 17 distinct vias are involved; 15 are fully
boxed-in. Root cause: these vias sit in the RP2040 GPIO fan-out at ~0.8mm pitch against 0.4mm-spaced
U3 pads — a via drill (r 0.1) needs 0.45mm center-to-center from each pad hole (r ~0.175), and there
is simply no legal position within a bounded nudge. **These need REROUTING to different positions,
not endpoint nudging.**

**Conclusion — #4 NEAR build is COMPLETE at its measured ceiling (mitayi 104 → 88).** The clean win
was the track-endpoint → pad nudge (clearance 42 → 27). Via-involved legality (52 errors) and the
connectivity gap are BOTH congestion/quantization problems that endpoint nudging cannot reach — they
require the FAR gridless shape-based router (continuous via/track positions over free-space polygons),
which is the documented connectivity unlock. The hole-aware via nudge is hereby SUPERSEDED by that
chapter, not deferred-pending — probe-disproven as a standalone lever.
