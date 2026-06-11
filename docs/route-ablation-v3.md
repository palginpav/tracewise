# Routing ablation — naked vs TraceWise-constrained Freerouting

Same board, all routing stripped, routed twice: with KiCad's stock settings (naked) and after `tracewise constrain` generated net classes + rules (constrained). Freerouting 2.2.4, default effort.

| Board | Mode | unconnected | DRC violations | vias | length (mm) |
|---|---|---|---|---|---|
| rp2040-dev-board | naked | 23 | 197 | 26 | 838.7 |
| rp2040-dev-board | constrained | 30 | 195 | 32 | 730.2 |
| mitayi-pico-d1 | naked | 89 | 4 | 26 | 226.9 |
| mitayi-pico-d1 | constrained | 89 | 503 | 26 | 213.9 |
| zuluscsi-pico-oshw | naked | 27 | 486 | 94 | 4031.6 |
| zuluscsi-pico-oshw | constrained | 39 | 982 | 90 | 3852.8 |

## Analysis (v3 — fair scoring + clamped constraints)

**What the fixes verifiably fixed:** rp2040's violation columns equalized (197 naked vs 195
constrained — the scoring asymmetry is gone there), and generated classes now respect both the
project minimums and zone-local clearances (the constrained DSN carries 0.18 rules where the
GND pour demands it; verified in the exported rules).

**What remains open — recorded, not hand-waved:** mitayi's constrained arm still shows ~500
track-to-zone clearance errors that the naked arm avoids. The violating nets are *Default-class*
GPIOs (identical 0.15 rules in both arms), so the class rules are not the direct cause; the
leading hypothesis is displacement — wider Power tracks push GPIO routing into the GND pour's
0.18 clearance band. zuluscsi also showed run-to-run variance this time (39 vs 21 unconnected on
an earlier identical run), so Freerouting determinism can no longer be assumed in comparisons —
repeated runs belong in the methodology. **Next step is visual: open the naked/constrained board
pairs and look at the copper** (`~/.cache/tracewise/ablation/<board>/{naked,constrained}/`).

**Standing conclusions, unchanged:** constraints are board-dependent (helped dense 2-layer,
hurt tight 4-layer), Freerouting's completion ceiling dominates everything (23–89 unconnected
on every arm), and each measurement round so far has improved the *tool* (scoring fairness,
project-minimum clamps, zone-clearance floors) — which is what the harness is for.
