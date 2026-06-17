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
