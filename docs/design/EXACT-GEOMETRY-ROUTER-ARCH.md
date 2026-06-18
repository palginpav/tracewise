# Exact-Geometry Router — v2 architecture

Status: architecture (2026-06-18). The measurement-justified unlock for the LAST gap (both
connectivity and legality). Grounded in docs/research/NEXT-exact-geometry-routing.md + this
session's findings. The BUILD is a multi-day (near) to multi-month (full) chapter; this is its
blueprint.

## Why exact geometry (measured, not assumed)

Both remaining gaps to the human are grid-quantization artifacts:
- **Legality** — grid-legal track centerlines (0.1mm cell centers) land sub-pitch (<0.05mm) from
  copper → clearance violations (mitayi 104, zuluscsi 150 errors). Proven not escape-tunable.
- **Connectivity** — the 0.1mm grid OVER-ESTIMATES congestion. Measured: finer grid connects more
  (mitayi 0.1mm→48, 0.075→28, 0.05→27 unconnected). Exact geometry = the limit (zero quantization).

Exact geometry eliminates BOTH by construction: copper is exact polygons, clearance is exact
distance, routes are placed at true-legal positions — no cell to over-/under-estimate.

## Two horizons (honest staging)

### NEAR — Minkowski-snap exact-geometry EMITTER (research #4; ~3-5 days; LEGALITY only)
Keep the grid A* for the global path; replace `emit.py`'s "cell-center → track endpoint" snapping
with an EXACT-geometry resolver: each emitted segment endpoint is placed at the farthest-legal
position w.r.t. Minkowski-inflated neighboring obstacles. Eliminates emit-boundary (mode-1)
clearance violations. Does NOT touch connectivity (routing still on the grid) and does NOT fix
mode-2 pour-interaction shorts. No grid.py/astar.py changes.
- Dependency: add **Shapely** (Apache-2, standard EDA geometry; currently NOT installed). A numpy
  fallback (segment-segment / segment-rect exact distance) covers the clearance CHECK without
  Shapely, but Shapely is cleaner for Minkowski inflation + polygon ops — recommend adding it.
- Plug point: `emit_routes` in kicad.py, the `grid.to_world(iy,ix)` endpoint computation.

### FAR — full gridless shape-based router (research #6/#7; 3-6 months; CONNECTIVITY + LEGALITY)
Route in continuous coordinates over free-space polygons (Minkowski-inflated obstacles ⇒ convex
"expansion rooms"; A* / line-probe over rooms; or topology-first rubber-band then geometric
realization). Eliminates quantization entirely ⇒ no false congestion (connectivity) AND no
sub-pitch clearance (legality). This is the true unlock for the connectivity gap (the finer-grid
result extrapolates to: exact = best connectivity). Replaces grid.py/astar.py's search core;
keeps the I/O contract, nets/pads model, congestion pricing (history) layered on, and the
scorecard. Effort: ~3-6 months in Python (Freerouting is ~10kLOC Java for the search alone).

## Recommended sequencing

1. **Interim (shipped):** `--quality` finer-pitch mode — connectivity now (mitayi 48→28) at a
   runtime cost. Bridges until the gridless router.
2. **NEAR build (#4):** Minkowski emit — cut the legality/clearance errors toward the human (5-65).
   Bounded, ships value, builds the exact-geometry tooling (Shapely + Minkowski helpers) that the
   FAR build reuses.
3. **FAR build (gridless):** the connectivity unlock. The big chapter; de-risked by the NEAR
   tooling + the measurement mandate.

## Reuse inventory

Nets/pads model (extract_pads, build_problem); the escape concept; congestion pricing
(history_factor) — layers onto any geometry; F0 pour extraction; F3 pour synthesis; L1 ceiling
detector; the routing-in-the-loop scorecard; the DRC harness. The exact-geometry work replaces the
SEARCH SUBSTRATE (grid → polygons), not the surrounding pipeline.

## Risks & mitigations

- **Shapely dependency** — small, standard, Apache-2; or numpy fallback for the distance check.
- **#4 doesn't fix connectivity OR pour-interaction shorts** — by design; it's the legality stopgap.
  The connectivity unlock is the FAR build. Set expectations: #4 targets the error gap only.
- **Mid-segment clearance** — #4 snaps ENDPOINTS; a long grid-routed segment can still violate
  mid-path. Mitigate: also validate/split long segments, or accept that #4 is partial and the FAR
  build is the complete fix.
- **FAR build scale** — 3-6 months; the documented horizon, not this session. Robustness (the
  geometry-bug class, e.g. the Y-swap) demands heavy fixture testing.

## Definition of done (per horizon)

- NEAR (#4): scorecard errors materially toward the human (mitayi→0, zuluscsi→~5), no unconnected
  regression, deterministic.
- FAR (gridless): scorecard unconnected ≤ ~5 AND errors ≤ human on all boards — TraceWise matches a
  human router on 2 layers, the original claim, proven against shipped designs.

## Immediate next step

~~Phase A of the NEAR build~~ **DONE (2026-06-18) — see "Phase-A status" below.**
~~Next is the emit refactor (Phase B)~~ **Phase B DONE (2026-06-18): mitayi 104 → 88 errors,
deterministic, 0 unconnected regression** (see `docs/design/PHASE-B-emit-integration.md`). Terminal
track-endpoint → pad nudge is the shipped win (clearance 42→27, pad↔track 18→~3). Via nudge gated
off (was net-negative on hole_clearance). The ~42 PROBE-A target was optimistic: it counted
via-involved + track↔track cases that the pad-only obstacle model can't reach. Remaining legality
gap (vias, track↔track, mid-segment) → hole_clearance-aware via nudge (bounded) or obstacle-set
expansion, the latter folding into the FAR gridless build.

### Phase-A status (2026-06-18) — exact-geometry primitive layer BUILT + TESTED
- Committed the numpy distance fallback (NO Shapely — not installed). New module
  `src/tracewise/route/engine/exact_geom.py`: `segment_point_distance`, `segment_segment_distance`,
  `point_rect_distance`, `segment_rect_distance`, `clearance_to_obstacle`, `min_clearance`,
  `is_legal`, and the core `nudge_endpoint(endpoint, anchor, obstacles, required_clearance,
  track_hw, max_nudge=0.3)` (Minkowski push-out + bounded deterministic polar refinement;
  anchor-constrained so the endpoint stays attached to its pad).
- `tests/test_exact_geom.py`: 46 fixture tests — hand-computed distances with asymmetric coords,
  explicit anti-(x/y)-swap test, determinism test, and the real mitayi case (endpoint 0.0998mm from
  a pad, required 0.15mm → legal nudge found, clearance satisfaction ASSERTED on the returned point).
  All 46 pass; full suite green; ruff clean. Obstacle model: circle (via/hole/round-pad), rect
  (pad), segment+hw (track). Probe before this: PROBE-A above set the realistic ceiling at ~62/74.

## PROBE — #4 (Minkowski emit) is HIGH-value: legality is clearance-dominated (2026-06-18)

Error-type breakdown of a current routed mitayi (104 errors):
  clearance 42 + hole_clearance 32 = 74 (71%)  | solder_mask_bridge 16 | hole_to_hole 14
71% are CLEARANCE-CLASS — exactly what the exact-geometry emitter (#4) addresses by placing tracks
AND VIAS at true-legal positions (not 0.1mm cell centers / via-ring quantization). So #4 is NOT a
minor stopgap: it could cut mitayi errors ~104 -> ~30 (the clearance-class), a major legality win,
in the ~3-5 day NEAR build. (Caveat: #4 must nudge VIA positions too — hole_clearance is via-to-
copper; hole_to_hole is via-via spacing, partly #4-addressable.) solder_mask_bridge + some
hole_to_hole are partly board-inherent / need separate handling.
REVISED NEAR target: #4 exact-geometry emit (tracks + vias) -> clearance-class errors toward 0.
Combined with the connectivity lever (finer pitch now, gridless later), this is the credible path
to the human (0/0): #4 for legality (measured 71% addressable), gridless for connectivity.

## PROBE-A — endpoint-nudgeability of the 74 clearance-class errors (2026-06-18)

Numpy exact segment/via-to-copper distance analysis on a freshly-routed mitayi HUMAN placement
(0.1mm grid; 48 unc / 104 err / 74 clearance-class — matches the scorecard). Probe script:
`scripts/probe_clearance_nudgeability.py` (numpy-only, no Shapely; deterministic — reproduces
exactly). Each clearance-class error assigned to one bucket:

| bucket               | clearance | hole_clearance | TOTAL |
|----------------------|-----------|----------------|-------|
| endpoint-nudgeable (optimistic)   | 38 | 30 | **68** |
| endpoint-nudgeable (conservative) | 32 | 30 | **62** |
| mid-segment          | 3  | 0  | 3  |
| pour-interaction     | 0  | 0  | **0** |
| inherent/other       | 1  | 2  | 3  |
| **TOTAL**            | 42 | 32 | **74** (sanity: sum = 74 ✓) |

- **Optimistic** = violation point is a via or near a track endpoint, and the other party is discrete
  (pad/track/via). **Conservative** = a legal position also EXISTS within a 0.3mm nudge that restores
  the required clearance to all nearby copper (6 candidates fail: tight corridors needing reroute).
- **0 pour-interaction** — the documented #4 pour caveat is a NON-ISSUE on this board. All 74 are
  routed-copper collisions (track/via vs pad/track/via).
- **REALISTIC #4 CEILING: ~62 of 74 fixable → mitayi 104 → ~42 errors** (conservative). The earlier
  "~30" arch estimate was optimistic by ~12; 62 is the measurement-backed number.
- Caveats: pads approximated as axis-aligned rects (no rotation); vias always optimistic; nudge
  strong-test is a polar grid (12 angles × 6 radii). See script docstring.

Verdict: #4 (Minkowski-snap endpoint+via emit) remains HIGH-value and well-targeted — proceed to
Phase-A build (Shapely/numpy clearance helper + fixture tests, then refactor emit). Set the success
target at mitayi err → ~42, not 0; the residual (3 mid-segment + 3 inherent + 6 tight-corridor) needs
reroute/gridless, not endpoint nudging.
