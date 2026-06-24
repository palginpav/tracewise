# Full-board scorecard (B3, 2026-06-24, PM-measured)

The entire gridless/negotiate/fanout/ring-slot/CDR arc optimized mitayi ONLY. B3 re-measures the
current router on ALL 3 benchmark boards vs the human targets. Canonical method (route HUMAN placement +
run_drc, severity==error, unconnected=len(unconnected_items)).

## Default grid router (the committed default — the practical config)
| board | OURS unc/err | HUMAN unc/err |
|---|---|---|
| mitayi | 48 / 88 | 0 / 0 |
| zuluscsi | 15 / 149 | 0 / 5 |
| rp2040 | 33 / 152 | 3 / 65 |
(mitayi best = 41/73 with the OPT-IN attempt-3 gridless_first 17-net set; default grid = 48/88.)

## gridless_rescue (the generic auto path) — DOES NOT GENERALIZE
- mitayi: ABORT (route took 1505s / ~25min, RSS 1.17GB — the QFN fanout path is pathologically slow).
- rp2040: same slow QFN fanout path (aborted/killed at the 600s guard).
- zuluscsi: completed 455s but 15/150 — NO connectivity improvement vs default (15/149), +1 error.
=> The mitayi-tuned gridless work is either pathologically slow (boards with a QFN fanout) or a no-op
(zuluscsi). It is NOT a practical general default. The grid router is the practical cross-board config.

## ERROR-CLASS DECOMPOSITION (the key B4 insight: much of the "error gap" is NON-routing)
Per board, errors split into BOARD-INHERENT (placement/footprint/board-setup — the router CANNOT fix;
the human's own error counts include these) vs ROUTING-INTRODUCED (our tracks/vias):
- **rp2040 (152):** NON-routing = courtyards_overlap 33 + footprint_type_mismatch 32 + copper_edge_clearance 6 = **71**. Routing-introduced = shorting_items 23 + hole_clearance 27 + clearance 4 + hole_to_hole 2 (+ solder_mask_bridge 25, partly board). The human's 65 rp2040 errors are largely the SAME board-inherent classes → the true ROUTING gap is far smaller than 152−65.
- **zuluscsi (149):** NON-routing = footprint_type_mismatch 3 + courtyards_overlap 2 + copper_edge_clearance 2 + items_not_allowed 1 = 8. Routing-introduced = **shorting_items 40** + clearance 24 + hole_clearance 15 + hole_to_hole 21 (+ solder_mask_bridge 41, partly board).
- **mitayi (88):** all routing-class (clearance 26, hole_clearance 32, hole_to_hole 14, solder_mask_bridge 16).

## Biggest ROUTING-FIXABLE error opportunities (→ B4)
1. **shorting_items: zuluscsi 40, rp2040 23, mitayi 0** — the router is creating real different-net shorts on zuluscsi/rp2040. THE biggest routing-quality bug (not board-inherent, not architectural ceiling).
2. **hole_clearance: mitayi 32, rp2040 27, zuluscsi 15** — vias/track-ends too close to drill holes.
3. **clearance: mitayi 26, zuluscsi 24** — tracks too close to other-net copper.
4. solder_mask_bridge (zuluscsi 41, rp2040 25, mitayi 16) — partly board-inherent (mask slivers between close pads), partly routing.

## Conclusion / next (B4)
The mitayi CONNECTIVITY tail (41→0) is the architectural ceiling (mapped exhaustively). But the ERROR
axis — especially the routing-introduced shorting_items (zuluscsi 40!) and hole_clearance — is a
DIFFERENT, likely-tractable opportunity that was never targeted (the whole arc chased mitayi
connectivity). B4: decompose errors board-inherent vs routing-introduced, then attack the
routing-introduced shorts + hole_clearance, measuring error reduction with NO connectivity regression.

---

## B4 RESULT (2026-06-24) — castellated-pad obstacle fix: −132 errors across boards (PM-verified)

ROOT CAUSE (diagnosed): castellated/hybrid Pico-module pads (zuluscsi U1, rp2040 U201) have their copper
bounding-box CENTER offset ~0.9mm from `pcbnew GetPosition()` (the drill center). `build_problem` blocked
the obstacle rect at GetPosition → left ~0.9mm of REAL copper unblocked → the grid router routed tracks
THROUGH actual copper → `shorting_items`. FIX: `PAD_SCRIPT` exports `bb.GetCenter()`; `build_problem`
uses bb_center for the obstacle rect (fallback to position for normal pads → normal boards unchanged).

**PM-verified A/B (independent re-route, vs B3 baseline):**
| board | B3 (before) | B4 (after) | Δerr | connectivity |
|---|---|---|---|---|
| zuluscsi | 15 / 149 | **13 / 40** | **−109** | 15→13 (improved) |
| rp2040 | 33 / 152 | 31 / 128 | −24 | 33→31 (improved) |
| mitayi | 48 / 88 | 48 / 89 | +1 (noise; no castellated pads) | unchanged |
zuluscsi shorting_items 40→1, solder_mask_bridge 41→0. Total errors 389→257 (**−132**). **NO connectivity
regression on any board.** Suite 424 green, ruff clean, bounded (536MB).

This is the best routing-quality improvement of the whole arc AND it's GENERAL (a DEFAULT grid-router fix
helping any board with Pico-module castellated pads — not a mitayi-only opt-in). The error-axis (B4) was
far more tractable than the mitayi connectivity tail: a single principled obstacle-model fix cleared most
of the routing-introduced shorts. (Rejected Fix-2 via-at-goal-cell hard-check: regressed mitayi 88→121 /
15 nets fail — needs escape-before-via architecture; correctly deferred.)

Residual routing-introduced opportunities: rp2040 16 shorting_items + 20 hole_clearance (via-at-goal-cell
on QFN pads — the deferred Fix-2 territory); mitayi 32 hole_clearance; clearance (mitayi 27, zuluscsi 12).
