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

## Designer review (the data point that settles it)

The project author (10+ years electronics design) opened both mitayi arms in KiCad and ran
full GUI DRC. Verdict: **both are poor; naked is the better of the two.** His reports add what
the harness metrics missed:

- **`track_dangling` dominates** — 48 stubs (one arm) and 35 (other): Freerouting leaves
  dangling track stubs all over, the single biggest contributor to "this looks awful" that
  violation counts under-weight
- GUI DRC counted ~104–108 unconnected items (higher than the CLI's 89 — counting/severity
  configuration differs between paths; worth unifying in the harness)
- The arm with *fewer* counted violations was judged *worse* by eye — **DRC counts are not a
  routing-quality metric**, they're a manufacturability gate

## Consequence for the roadmap

Three measurement rounds and a designer review converge: **the router is the bottleneck, not
the constraints.** Constraint micro-tuning on top of Freerouting has hit its ceiling on these
boards. Actions:

1. **Stop tuning constraints for Freerouting** — keep `constrain` (correct, clamped, conditional)
   as-is; it helps where boards have constraint-sensitive structure and the solver has room
2. **Post-route cleanup pass** — deleting dangling stubs is a mechanical pcbnew operation and
   directly attacks the dominant visual defect (quick win, queued)
3. **Pluggable engine priority up** — Topola (topological, in development) and any future
   engine slot in behind the same bridge; the DSN/SES plumbing and its battle-hardening carry over
4. **v0.3 placer becomes the main line** — placement quality constrains routability more than
   rules do; that was the design doc's thesis and the evidence now backs it

## Route-after-place experiment (v0.3 placer, center-offset-corrected)

Does TraceWise placement improve Freerouting's completion? **No — it currently hurts it.**
mitayi, routing stripped, placed by `tracewise place` (HPWL −11%, overlap 215 mm² vs the
human's 130), then routed: **101 unconnected / 179 violations** vs the human placement's
**89 / 4**. Wirelength is not routability: the HPWL+overlap objective doesn't model
congestion, and residual overlap directly blocks routing channels. The human layout encodes
routability knowledge (functional clustering, channel discipline) our cost function doesn't
have yet. Documented gaps for the placer: congestion-aware cost term, rotation support,
overlap-free legalization. The placement thesis stays unproven until those land — recorded,
not spun.
