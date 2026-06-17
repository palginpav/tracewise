# Routing-completion plan — breaking the zuluscsi unconnected floor

Status: active (2026-06-17). Owner: orchestrated feature engineering.

## Objective

Break zuluscsi's unconnected floor (84 DRC unconnected items, deterministic) without
regressing mitayi or pic_programmer. The floor is now attributed at net level, so each
feature targets a specific slice.

| net group | now | target | feature |
|---|---|---|---|
| +3V0 (no pour, 58-pad power) | 56 | ≤15 | F3 high-fanout power |
| GND (pour exists, pads isolated) | 22 | ≤5 | F2 pour-stitching |
| ~{SCSI_DB2}, ~{SCSI_DB4} | 6 | ≤2 | F4 stubborn-net rip-up |

Hard gate per feature: measured improvement on zuluscsi AND zero regression on mitayi +
pic_programmer, all routes deterministic (`taskset -c 0-9`, routes now complete under the
600s cap after the 2026-06-17 A* speedups).

## Background facts (load-bearing)

- Router is deterministic since the A* heuristic + neighbor-expansion vectorization
  (mitayi 290s→134s; zuluscsi completes at ~530s, byte-identical across runs).
- `history_factor=1.0` (negotiated-congestion pricing) is on by default in
  `route_board_engine`.
- GND has an 85-poly pour spanning F&B; `refill_zones` (pcbnew ZONE_FILLER) fills it, but 22
  GND pads are isolated from the fill. +3V0 has NO pour in the source board — it is meant to
  be trace-routed (the human does) and the router completes only 2/58.
- Geometry-extraction fragility: zone `net_name` parses on the source board but returns NONE
  on a pcbnew-resaved board. Any pour feature must read geometry/connectivity robustly.
- Key files: `src/tracewise/route/engine/{kicad,multi,astar,eccf,grid}.py`,
  `src/tracewise/route/bridge.py`. Harness: `scripts/ablation_route.py`.

## Features

### F0 — Robust pour/connectivity extraction (foundation, blocking)
Source of truth for (a) pour-copper geometry and (b) pads unconnected to their pour.
Likely pcbnew connectivity API over s-expr. Deliver: design doc + API spec + test plan.
Chain: architect → developer → tester (fixture: source AND resaved boards).
Accept: returns pour-cell mask + isolated-pad list, identical on both board formats.

### F2 — GND pour-stitching (depends on F0)
Rasterize pour into the routing grid; route a short stub from each isolated pour-net pad to
the nearest pour cell (vectorized heuristic handles the large pour goal set). Emit → refill.
Chain: architect → developer → reviewer → measure.
Accept: zuluscsi GND 22→≤5, no new clearance violations, mitayi no regression.

### F3 — +3V0 high-fanout power (dominant 67%, hardest)
Design question (research first): pour problem (auto-generate a partial +3V0 fill on copper
GND doesn't own) vs routing problem (trunk/star + stubs, route-first ordering). 2-layer area
is contended; likely hybrid.
Chain: researcher → architect → developer → reviewer → measure.
Accept: +3V0 56→≤15, no regression. May land in stages.

### F4 — SCSI stubborn nets (small)
Per-net rip-up escalation / congestion-pricing tuning for the 2 remaining signal nets.
Chain: developer → measure.

## Phasing

```
Phase 0:  architect (F0 design) || researcher (F3 survey)        [read/think only]
Phase 1:  developer + tester (F0)                                [blocks F2]
Phase 2:  F2 chain (GND stitch) || architect (F3 design)
Phase 3:  F3 chain (+3V0)
Phase 4:  F4 + full cross-board validation + ROI scorecard
```

## Measurement protocol (gate)

Per-net DRC-breakdown harness (extends `scripts/ablation_route.py`), run before/after each
feature on all three boards, deterministic. No feature merges unless its target net group
improves and the others hold.

## Risks

- F0 fragility → if the pcbnew API is awkward, F2 slips. Mitigation: F0 is its own gated phase.
- F3 may have a real 2-layer ceiling (pour-less 58-pad net); partial wins acceptable, documented.
- Geometry bugs (the Y-coord-swap class): every geometry change ships with a fixture unit test.

## STATUS UPDATE (2026-06-17): F0 shipped; F2 approach FALSIFIED, reverted; model corrected

- **F0 SHIPPED** (commit ecc9587): pcbnew-based pour geometry extraction (`extract_pours`,
  `rasterize_pour`) — sound, 12 tests, source-vs-resaved invariant holds. KEPT.
- **F2 (stub-stitch) FALSIFIED and reverted.** Two implementation rounds capped at zuluscsi
  GND 22→20 (only 2 pads stitched), mitayi no regression. Root cause found by direct diagnosis:
  the stitch worklist used `unconnected_pads` (pcbnew `IsConnectedOnLayer`), which reports only
  **2** isolated GND pads, while DRC ratsnest reports **22**. The gap: `IsConnectedOnLayer`
  calls a pad "connected" when it touches ANY pour copper — including a DISCONNECTED pour
  island. So the real problem was mis-modelled.
- **CORRECTED PROBLEM MODEL (decisive — the 22 GND ratsnest gaps):**
    - 10 gaps are **zone↔zone at 0.0mm** — the GND pour fragmented into ISLANDS separated by
      hairline clearance; they need island BRIDGING (a via/short bridge at the gap), not pad stubs.
    - 12 gaps are **far** (median 44.9mm, max 93.4mm) — real cross-board GND routing.
    - Almost none are "isolated pad near pour needing a short stub" — F2's entire premise.
- **Consequences for the plan:**
    - F0's `unconnected_pads` is the WRONG signal (IsConnectedOnLayer misses island
      fragmentation). A future feature must use the RATSNEST (DRC unconnected_items) as the
      worklist, not IsConnectedOnLayer.
    - The GND floor (22) really decomposes into: ~10 pour-island bridges + ~12 high-fanout GND
      routes. The +3V0 floor (56) is pour-less high-fanout. So the dominant remaining work is
      TWO new capabilities: (1) pour-island bridging (cheapest next lever — 0mm gaps), and
      (2) high-fanout power/ground net routing (the hard, dominant part). Stub-stitching is NOT
      the lever and is removed.
- **NEXT (re-scoped):** F2' = pour-island bridging driven off the DRC ratsnest (target the 10
  zone↔zone 0mm GND gaps; likely a via/short bridge where two same-net islands abut). Measure;
  if it cleanly fixes the island gaps, generalise. The 12 far GND + 56 +3V0 remain the
  high-fanout routing problem (F3-class), genuinely hard on 2 layers.
