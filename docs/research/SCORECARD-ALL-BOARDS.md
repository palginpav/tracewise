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
