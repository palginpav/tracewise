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

Phase A of the NEAR build: add Shapely (or commit to the numpy distance fallback), implement the
exact segment-clearance + Minkowski-endpoint helper with FIXTURE unit tests (a known segment near a
known pad → exact legal endpoint), BEFORE touching emit.py. Probe first: on a routed board, confirm
exact-geometry analysis identifies the DRC clearance violations our grid emits and that endpoint
nudging resolves a measurable fraction (some are mid-segment / pour-interaction and won't be).

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
