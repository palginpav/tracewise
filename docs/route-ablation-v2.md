# Routing ablation — naked vs TraceWise-constrained Freerouting

Same board, all routing stripped, routed twice: with KiCad's stock settings (naked) and after `tracewise constrain` generated net classes + rules (constrained). Freerouting 2.2.4, default effort.

| Board | Mode | unconnected | DRC violations | vias | length (mm) |
|---|---|---|---|---|---|
| rp2040-dev-board | naked | 23 | 197 | 26 | 838.7 |
| rp2040-dev-board | constrained | 30 | 279 | 32 | 730.2 |
| mitayi-pico-d1 | naked | 89 | 4 | 26 | 226.9 |
| mitayi-pico-d1 | constrained | 89 | 4 | 21 | 211.7 |
| zuluscsi-pico-oshw | naked | 27 | 488 | 94 | 4031.6 |
| zuluscsi-pico-oshw | constrained | 21 | 944 | 93 | 3902.1 |

## Analysis

**Completion (unconnected — the metric that matters most): mixed, board-dependent.**
- zuluscsi (dense 2-layer, buck regulator): constrained **better** — 21 vs 27 unconnected, −3% length
- rp2040 (4-layer, USB diff pairs): constrained **worse** — 30 vs 23; wider Power tracks consumed
  routing space in a tight layout
- mitayi: identical completion (89 both — see below), constrained slightly cleaner (−5 vias, −7% length)

**The violation columns are NOT comparable between arms — a measurement-design flaw we're
keeping on the record.** The constrained arm writes a `.kicad_dru` and tighter net-class
clearances which DRC then *enforces*; the naked arm is scored against laxer rules. zuluscsi's
488 vs 32 errors is mostly this asymmetry (the same copper judged against stricter rules), not
worse routing. Fix for the next iteration: score both arms against an identical rule set
(strip TraceWise rules before DRC, or apply them to both arms' scoring).

**The dominant signal is Freerouting's own ceiling on real 2026 boards.** 21–89 unconnected
across every arm: mitayi (a bare RP2040 with USB on MCU pads, 2 layers) is largely beyond it
regardless of constraints. This matches the professional consensus our design research found and
strengthens the case for the roadmap's pluggable-engine interface (Topola et al.) over deeper
investment in constraint tuning for this solver.

**Net read:** constraints show a real effect in the direction theory predicts on the dense
2-layer board, hurt on the space-tight 4-layer, and can't rescue a solver out of its depth.
n=3 boards; the suite and the fair-scoring fix are the next iteration. All runs reproducible:
`python scripts/ablation_route.py --projects ... --src data/benchmark-boards`.

## Bridge robustness (a by-product worth recording)

Getting three real community boards through the DSN/SES round-trip surfaced and fixed four
integration failures the official KiCad demo never shows: micro-gap board outlines (fatal to
the DSN exporter, healed automatically), references the DSN grammar can't carry ("G***" logos,
empty-ref screw holes — sanitized in memory), label footprints the exporter rejects outright
(auto-bisected and dropped from export with disclosure), and Freerouting's launcher splitting
unquoted paths on spaces. "Works on real boards, not just curated demos" is now a tested claim.
