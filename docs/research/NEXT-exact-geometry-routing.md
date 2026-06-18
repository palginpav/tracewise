# Research: NEXT-exact-geometry-routing
## Eliminating quantization shorts in TraceWise's grid-A* router

**Researched:** 2026-06-18
**Decision trigger:** F4 short-stub feature validated +18 connectivity but introduced 3 unremovable
grid-quantization shorts. Quality gate reverted it. Exact-geometry clearance is now the blocking
problem.

---

## Problem Framing

**Goal:** Identify the smallest architectural change that eliminates grid-quantization shorts in
TraceWise's 0.1mm grid A* router, and assess the fuller alternatives if that minimum change cannot
close the gap.

**Hard constraints:**
1. Must work with or evolve from the existing Python/numpy grid stack
   (`engine/grid.py`, `astar.py`, `multi.py`).
2. Must produce output verifiable by KiCad's own DRC (`kicad-cli pcb drc`) — the external
   judge is the standard.
3. Must not increase wall-clock time beyond what the 600s cap allows (currently mitayi at ~134s,
   zuluscsi at ~530s).
4. Python-first (Rust extraction allowed if measured necessary, but the approach must be
   expressible in Python first).

**Soft constraints:**
1. Retrofit onto existing ECCF + rip-up + pour-synthesis pipeline — not a full rewrite unless
   clearly justified.
2. Completion rate must not regress (zuluscsi 65, mitayi 63 are the floors).
3. Prefer approaches with open-source prior art in PCB or VLSI routing.

**Known-not-wanted:**
- Freerouting re-integration (documented failure in ablation; completion/dangling-stub failures
  are engine properties, not constraint problems).
- Approaches requiring 4+ layers or blind vias (out of v1 scope).
- ML-based routing (no tractable path from current Python codebase; no verifiable clearance
  guarantee).

---

## The quantization short anatomy (from F4 post-mortem)

The 0.1mm grid guarantees clearance only at grid-cell centers. A track center placed at cell
`(c_x, c_y)` with halfwidth `w` and a neighbor track at `(c_x + k, c_y)` occupies world-space
`[c_x - w, c_x + w]` exactly. But KiCad's DRC measures the minimum distance between the exact
Minkowski-inflated geometries of two tracks. Sub-pitch positional errors and diagonal-segment
endpoint snapping can produce actual clearance `< required` even when the cell count says
"clear."

Two distinct failure modes observed in F4:
- **Direct grid-quantization short:** a diagonal stub segment endpoint snaps to a grid cell whose
  clearance halo does not perfectly tile with an adjacent inflated obstacle at exact geometry.
- **Pour-interaction short:** emitting the stub triggers a zone refill that fragments or
  re-bridges copper elsewhere at ~132–135mm, creating a short outside the stub itself.

These are different problems with different fixes.

---

## Candidate Approaches

| # | Approach | Source / Provenance | Fit Score (0–5) | Fit Rationale | Known Risks | Cost / Complexity | Verdict |
|---|----------|---------------------|-----------------|---------------|-------------|-------------------|---------|
| 1 | **Exact-geometry post-pass: DRC-in-the-loop per-stub validation** | TraceWise F4 post-mortem (internal); OrthoRoute escape-planner lesson (https://bbenchoff.github.io/pages/OrthoRoute.html) | 4/5 | Plugs into existing emit pipeline. Each candidate track/via gets validated against KiCad exact DRC *before* commit. Blocks direct shorts; partial help on pour-interaction shorts. | Pour-interaction shorts (mode 2) survive per-stub DRC because the short materializes during zone refill, not in the stub itself. Reversion logic needs the DRC diff to be attributable. | Low-Med: ~100–200 LOC in `multi.py` + `bridge.py`; 1–2 day integration. | **Recommend** |
| 2 | **Finer grid (0.05mm or 0.025mm) — uniform reduction** | ROUTER-DESIGN.md §4 complexity analysis (internal); blog.autorouting.com grid tradeoffs | 2/5 | Halving the pitch squares the grid (4× cells per layer → 2.4 M cells/layer). mitayi 134s would likely hit 600s cap before routing completes. Reduces but does not eliminate quantization error. | Memory/time: 4× cells = grid becomes 18–40 MB, A* expansion count grows super-linearly in congested areas. Diagonal endpoints still have sub-pitch error at √2 * pitch / 2. | High for 0.05mm, prohibitive for 0.025mm. | Reject |
| 3 | **Adaptive / quadtree grid (fine only near obstacles)** | Multi-Scale Cell Decomposition for Path Planning (https://arxiv.org/abs/2408.02786); Nature article 3D LineExplore (https://www.nature.com/articles/s41598-026-36925-0) | 3/5 | Refines cells near pad/obstacle edges to sub-0.1mm, keeps bulk grid coarse. Reduces quantization error near the geometrically tight spots. Fits the "clearance inflation by caller" design in grid.py. | Quadtree neighbor lookup breaks the current flat numpy array design; significant grid.py rewrite. Clearance boundary is now at the fine-cell level but still discrete. Does not eliminate the problem, just shifts the floor. | Med-High: grid.py rewrite, A* node expansion over non-uniform cells, 1–2 week effort. | Consider |
| 4 | **Hybrid: grid A* path + exact-geometry segment emitter (Minkowski snap)** | Gridless router using maze+line-probe (US6545288B1, https://patents.google.com/patent/US6545288B1/en); tinycomputers.io Freerouting internals (https://tinycomputers.io/posts/the-mathematics-of-pcb-trace-routing.html) | 4/5 | Keep grid A* for global path; replace emit.py's "cell center → track endpoint" snapping with an exact-geometry resolver that Minkowski-expands each obstacle and computes the tightest-fitting legal endpoint. No grid.py or astar.py changes. Eliminates mode-1 shorts at the emit boundary. | Mode-2 (pour-interaction) shorts survive. Requires a Minkowski expansion helper and a KiCad-compatible segment endpoint solver; non-trivial geometry but tractable in Python/shapely. | Med: emit.py refactor + shapely dependency + geometry unit tests, ~3–5 days. | **Recommend** |
| 5 | **KiCad PNS-style push-and-shove post-pass (force propagation, exact coords)** | KiCad PNS source doxygen (https://docs.kicad.org/doxygen/classPNS_1_1NODE.html); FOSDEM 2015 PCB routing talk (https://archive.fosdem.org/2015/schedule/event/pcb_routing/); DeepWiki KiCad interactive router (https://deepwiki.com/KiCad/kicad-source-mirror/2.4-interactive-router) | 3/5 | PNS represents all geometry in continuous coordinates; its `SHAPE_LINE_CHAIN` + spatial index can slide segments until exact clearance holds. A post-pass that calls PNS via pcbnew scripting could clean up grid-routed tracks. | KiCad PNS is C++ / SWIG; calling it from Python scripting is fragile and version-specific. PNS operates interactively, not batch-over-a-board. Invoking it for 100+ stubs is untested. The pcbnew scripting API for PNS is not stable. | High: requires pcbnew scripting + PNS invocation per-track; brittle SWIG path; 1–2 weeks + ongoing breakage risk. | Consider |
| 6 | **Shape-based gridless replacement (Freerouting-style expansion rooms)** | tinycomputers.io Freerouting internals (https://tinycomputers.io/posts/the-mathematics-of-pcb-trace-routing.html); Freerouting GitHub (https://github.com/freerouting/freerouting) | 2/5 | Eliminates quantization error by construction: routing space is convex polygonal "expansion rooms" bounded by Minkowski-inflated obstacles. Every routed trace is legal by construction. | Complete engine replacement (Java, ~10 kLOC for the search alone); the ablation documented Freerouting's own failures (dangling stubs, poor completion, zone blindness). Building the equivalent from scratch in Python is a 2–4 month effort. | Very High: ~3–6 months, replaces grid.py + astar.py + multi.py entirely. | Reject (for now; see §Honest Ceiling) |
| 7 | **Topological / rubber-band routing (topology-first, then geometric realization)** | Topola (https://topola.dev/); Semantic Scholar rubber-band router (https://www.semanticscholar.org/paper/RUBBER-BAND-BASED-TOPOLOGICAL-ROUTER-Dayan-Cruz/91ce7726d0b103db47ab5db433ed75b538e6e7f8); Altium Situs description (https://resources.altium.com/p/automated-pcb-routing-with-situs-topological-autorouter); ROUTER-DESIGN.md §4 (internal) | 2/5 | Topology-first means the geometric realization step can use exact-coordinate push-and-shove; clearance is enforced during geometry, not on a grid. Highest quality ceiling. Already identified in ROUTER-DESIGN.md as the v2+ direction. | Highest implementation complexity of all candidates. Requires: constrained Delaunay triangulation of the board, rubber-band sketch generation, and a separate geometric realization pass. Topola is still WIP (its own description). gEDA's Toporouter is in C. 2–4 month effort. | Very High: reuses I/O contract but replaces search core entirely. | Reject (for now; see §Honest Ceiling) |

---

## Shortlist (Top Picks)

### Top Pick: Exact-geometry post-pass — DRC-in-the-loop per-segment validation (#1)

**Why:** This is the minimal, lowest-risk change that directly targets the F4 revert. The
existing multi.py + bridge.py already run DRC after each full board emission. What is missing
is a *per-candidate-path* DRC check before commit. The implementation is:

1. In `multi.py` `route_board_engine`, before marking a candidate `NetRoute` as `ok=True`
   and calling `_mark()`, emit its copper to a *scratch copy* of the board, run a targeted
   DRC (or use pcbnew's `Board.TestConnectivity()` + `Board.RunDRCJob()`), diff the
   shorting_items count.
2. If `shorting_items` increases: discard the candidate path, mark the net `FAILED`, log
   reason `"exact-geometry short"`. Do not commit.
3. If `shorting_items` is unchanged: commit normally.

This directly catches mode-1 shorts (direct clearance violation between track and
neighbor). It is inherently slower (N × DRC calls instead of 1), so it should be gated
behind a `validate_exact=True` flag and applied only to the stub-routing inner loop in F4,
not the main A* loop (which already uses the counting grid as its correctness model).

**Caveats:**
- Mode-2 shorts (pour-interaction) will likely survive this check unless the scratch
  board also runs `refill_zones` before each DRC. Zone refill via pcbnew adds ~2–10s per
  call, making it impractical per-stub. A practical compromise: validate before zone refill
  (catches mode-1), then run one final DRC+refill after all stubs are committed, and
  reject the entire stub batch if new shorts appear (binary fallback to zero stubs).
- The scratch-board DRC approach requires pcbnew access, which is already a dependency.
  The implementation must copy the board object, not mutate the live board.

**First-run cost signal:** 1–2 day integration into the existing `multi.py` + `kicad.py`
pipeline. No new dependencies. Moderate testing effort (need a fixture board with known
quantization shorts to verify detection).

---

### Runner-up: Hybrid exact-geometry segment emitter (#4 — Minkowski snap in emit.py)

**Why:** Addresses the root cause at the point of origin rather than detecting it afterward.
The current `emit.py` converts cell-center paths to track endpoints by multiplying by pitch
— a direct source of sub-pitch error on diagonal segments. Replacing this with a
Minkowski-aware endpoint resolver means each segment endpoint is computed as the farthest
legal position away from all neighboring obstacles, using exact 2D geometry.

The approach: for each diagonal segment `(layer, iy0, ix0) → (layer, iy1, ix1)`, compute
the exact world coordinates of the endpoints, then for each nearby obstacle (pad,
existing track, pour) compute the minimum separation using Shapely's `distance()` or a
manual polygon test, and nudge the endpoint outward if separation < required clearance.

**Caveats:**
- Requires Shapely (pure Python 2D geometry library; Apache-2 licensed; well-maintained).
  Already a natural dependency for Python PCB tooling.
- Endpoint nudging can create a track that no longer connects to its grid-path neighbors —
  the emitter must re-check connectivity after nudging.
- Pour-interaction shorts are not addressed (those happen during zone refill, not emit).
- Medium implementation complexity: ~200–300 LOC in emit.py + geometry utilities.

**First-run cost signal:** 3–5 day effort. Cleaner long-term fix than post-pass validation,
but higher initial risk (geometry bugs in endpoint solvers are subtle).

---

## Addressing the F4 Short-Stub Feature Specifically

The question: *could an exact-geometry post-pass make the reverted F4 short-stub feature
shippable?*

**Yes, with conditions.** The F4 stubs created two distinct short types:

**Mode 1 — direct grid-quantization short** (track clearance violation between the new
stub and an existing neighbor): *fully catchable* by per-stub DRC validation (approach #1)
if zone refill is included in the validation step. With `validate_exact=True`, the stub
emitter would detect these before commit and skip the offending stub. The -18 connectivity
win would be reduced by however many stubs are geometry-illegal, but the remainder would
be clean.

**Mode 2 — pour-interaction short** (zone refill triggered by stub emission fragments or
bridges copper elsewhere, ~132–135mm from the stub): *not directly catchable* per-stub
without running a full refill per stub (impractical). The mitigation strategy:

1. Emit ALL valid stubs (those passing mode-1 check) in a batch.
2. Run a single zone refill on the batch.
3. Run DRC. If `shorting_items` increased: binary fallback — reject the entire batch OR
   bisect the stub list (half at a time) to identify which specific stub triggers the
   pour-interaction short.
4. The bisect strategy is tractable: with 12–24 stubs, O(log n) refill calls =
   4–5 refill+DRC calls to isolate the culprit stub.

This would make F4 shippable: the -18 connectivity win becomes -15 or so (a few stubs
discarded as geometry-unsafe), with zero new shorts.

---

## Honest Ceiling: Can the Floor Be Crossed Without True Gridlessness?

**The partial answer: yes, for mode-1 shorts; no, for mode-2.**

Mode-1 (direct track clearance): a per-stub DRC gate or Minkowski-snap emitter completely
eliminates this class. No gridless engine required.

Mode-2 (pour-interaction): this is not a routing algorithm problem — it is a zone-refill
side-effect problem. Its fix is a pour-aware stub emitter that predicts whether emitting a
stub will fragment a fill into a short. This requires modeling the zone-fill geometry
(already partially available via F0's `rasterize_pour`). It is harder than mode-1 but
still within the grid-A* framework.

**The true quantization floor for the main routing loop (not just stubs):**

The main A* loop on a 0.1mm grid produces at most `pitch * √2 / 2 ≈ 0.071mm` positional
error on diagonal segments. With a 0.15mm clearance rule, this is a ~47% relative error —
large enough to cause shorts on fine-pitch boards but *empirically absent* on the current
benchmark boards outside the specific F4 stub scenario. This suggests the main loop's
occupancy-counting model is conservative enough (the inflation radius rounds up) that
direct shorts in the main loop are rare in practice.

**Where the grid floor is real:**
If TraceWise expands to 0.05mm pitch boards, fine-pitch BGAs, or tighter clearance rules
(0.1mm class), the 0.071mm diagonal error becomes ~71% of the clearance — a systematic
source of shorts that can only be reliably closed by either a finer grid or a true
shape-based engine. That is the architectural horizon for v2.

**Scope of a true gridless/shape-based engine as a separate effort:**
The Freerouting approach (expansion rooms, Minkowski-inflated obstacles, lazy convex room
generation, A* over free-space polygons) is a ~3–6 month effort in Python if built from
scratch. Freerouting itself is Java (~10 kLOC for the search), has the zone-blindness
problem documented in the ablation, and is not a drop-in. Topola is WIP. The right posture:
implement approaches #1 and #4 now (close the practical gap); design the gridless v2 engine
behind the same `route(board, spec) -> RouteResult` contract specified in ROUTER-DESIGN.md
§4, so it slots in when warranted.

---

## Recommended Architecture — Concrete Plug-In Points

### Immediate (closes F4, ~2–4 days total)

**Step A: Per-stub exact-geometry validation gate in `multi.py`**

Location: the stub-routing loop introduced by F4, inside `route_board_engine()`. After
`astar.route()` returns a candidate path for a stub:

```python
# pseudo-code — exact API TBD
if validate_exact:
    scratch = copy_board(board)
    emit_path_to_board(scratch, candidate_path)
    drc_result = run_drc(scratch)   # bridge.run_drc — existing function
    if drc_result.shorts > baseline_shorts:
        continue   # skip this stub
```

No changes to `grid.py` or `astar.py`. New parameter: `validate_exact: bool = False`
on `route_board_engine`. The scratch-board copy + DRC is expensive; use only for stubs,
not the main A* pass.

**Step B: Bisect gate for pour-interaction shorts**

Location: in `kicad.py` or a new `validate.py`. After all passing stubs are emitted and
zones refilled, if `shorts > baseline`:
- Bisect the stub list, refill+DRC each half, isolate culprit, reject only that stub.
- Maximum 5 refill+DRC calls for ≤24 stubs (log₂(24) ≈ 5).

### Medium-term (closes systematic quantization error, ~3–5 days)

**Step C: Minkowski-aware endpoint resolver in `emit.py`**

Replace the cell-center → world-coordinate multiplication with an exact-geometry endpoint
solver using Shapely. Each segment endpoint is nudged to the nearest legal position.
Location: `emit.py` `path_to_segments()` function. Adds a Shapely dependency.

### Longer-term (v2 architectural direction, ~3–6 months)

Shape-based gridless engine behind the same `route(board, spec) -> RouteResult` interface.
Build against ROUTER-DESIGN.md §4's I/O contract. Start from Freerouting's expansion-room
concept but with pour-fill awareness (the documented Freerouting gap). The grid A* engine
stays as the v1 fallback.

---

## Honest Gaps

- The Semantic Scholar rubber-band router paper (Dayan 1997) returned an empty page; its
  content was not directly fetchable. Topological routing is covered via Topola, Altium Situs
  description, and the ROUTER-DESIGN.md prior analysis.
- The arxiv multi-scale quadtree paper (2408.02786) exceeded the content limit; the quadtree
  approach is covered via the Nature article search result summary.
- KiCad PNS source is C++; the doxygen API was readable but no actual algorithm walkthrough
  was fetched. The FOSDEM 2015 talk abstract confirmed octagonal primitives + force propagation
  but no line-by-line detail.
- Known-not-wanted pre-excluded: Freerouting (documented ablation failures), ML routing (no
  verifiable clearance guarantee).
- OrthoRoute (GPU PathFinder for PCB) was surveyed: it has the same DRC-gap problem as
  TraceWise — its author spent days manually cleaning DRC violations after GPU routing.
  Confirms that the grid-quantization gap is a universal grid-router problem, not a
  TraceWise-specific bug.

---

## Recommended Next Agent

**Developer** — implement Step A (per-stub exact-geometry DRC gate in multi.py) as the
immediate unblock for F4. Scope is crisp: ~100–150 LOC in multi.py + kicad.py, gated
behind `validate_exact=True`, with a fixture test using the zuluscsi board's known
quantization shorts.

If Step A ships clean and mode-2 shorts persist, escalate to Step B (bisect gate).
Step C (Minkowski emitter) is the correct medium-term fix and can be assigned to
developer or architect depending on whether Shapely integration needs a design decision.

---

## Structured Result

```json
{
  "status": "success",
  "summary": "Surveyed 7 approaches for eliminating grid-quantization shorts in TraceWise's 0.1mm grid-A* router. Two are recommended: (1) a per-stub exact-geometry DRC validation gate in multi.py (lowest risk, directly unblocks the reverted F4 feature, ~1-2 days), and (2) a Minkowski-aware endpoint resolver in emit.py (addresses root cause, ~3-5 days). True gridless/shape-based routing (Freerouting expansion-room approach or topological rubber-band routing) is the correct v2 architectural direction but is a 3-6 month effort; the current gap can be closed within the grid-A* framework. Mode-1 shorts (direct clearance) are eliminable now; mode-2 shorts (pour-interaction side-effects) require a bisect-and-reject batch strategy.",
  "files_changed": ["/home/palgin/Business_projects/tracewise/docs/research/NEXT-exact-geometry-routing.md"],
  "files_read": [
    "/home/palgin/Business_projects/tracewise/docs/PLAN.md",
    "/home/palgin/Business_projects/tracewise/docs/ROUTING-COMPLETION-PLAN.md",
    "/home/palgin/Business_projects/tracewise/docs/ROUTER-DESIGN.md",
    "/home/palgin/Business_projects/tracewise/src/tracewise/route/engine/grid.py",
    "/home/palgin/Business_projects/tracewise/src/tracewise/route/engine/astar.py",
    "/home/palgin/Business_projects/tracewise/src/tracewise/route/engine/multi.py"
  ],
  "issues": [],
  "assumptions": [
    "The F4 short-stub loop is a distinct code path from the main A* routing loop; the DRC gate can be applied selectively.",
    "pcbnew board copy + DRC call takes ~2-10s; acceptable for a stub-count of ~24 but not for the main routing loop.",
    "Mode-2 pour-interaction shorts are caused by zone refill side-effects, not by track geometry; per-stub DRC before refill will not catch them unless refill is included.",
    "Shapely is an acceptable Python dependency (Apache-2, widely used in geospatial/EDA tooling).",
    "The existing `bridge.run_drc` function can be reused for scratch-board validation."
  ],
  "sources_cited": [
    "https://tinycomputers.io/posts/the-mathematics-of-pcb-trace-routing.html",
    "https://deepwiki.com/KiCad/kicad-source-mirror/2.4-interactive-router",
    "https://archive.fosdem.org/2015/schedule/event/pcb_routing/",
    "https://patents.google.com/patent/US6545288B1/en",
    "https://topola.dev/",
    "https://resources.altium.com/p/automated-pcb-routing-with-situs-topological-autorouter",
    "https://bbenchoff.github.io/pages/OrthoRoute.html",
    "https://github.com/freerouting/freerouting",
    "https://docs.kicad.org/doxygen/classPNS_1_1NODE.html",
    "https://www.semanticscholar.org/paper/RUBBER-BAND-BASED-TOPOLOGICAL-ROUTER-Dayan-Cruz/91ce7726d0b103db47ab5db433ed75b538e6e7f8",
    "https://blog.autorouting.com/p/building-a-grid-based-pcb-autorouter",
    "https://www.nature.com/articles/s41598-026-36925-0"
  ],
  "research_summary": {
    "goal": "Identify the smallest architectural change that eliminates grid-quantization shorts in TraceWise's 0.1mm grid-A* router, and assess fuller alternatives",
    "candidates_surveyed": 7,
    "verdict": "recommend_existing",
    "top_pick": "Exact-geometry post-pass: per-stub DRC-in-the-loop validation gate in multi.py",
    "runner_up": "Hybrid exact-geometry segment emitter (Minkowski snap) in emit.py",
    "artifact_location": "/home/palgin/Business_projects/tracewise/docs/research/NEXT-exact-geometry-routing.md",
    "next_agent_hint": "developer"
  }
}
```
