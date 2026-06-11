# Routing ablation — naked vs TraceWise-constrained Freerouting

Same board, all routing stripped, routed twice: with KiCad's stock settings (naked) and after `tracewise constrain` generated net classes + rules (constrained). Freerouting 2.2.4, default effort.

| Board | Mode | unconnected | DRC violations | vias | length (mm) |
|---|---|---|---|---|---|
| pic_programmer | naked | 3 | 66 | 0 | 1505.3 |
| pic_programmer | constrained | 3 | 68 | 2 | 1493.2 |
## Analysis (honest): no effect on this board — and why that's informative

On pic_programmer, constraints made no measurable difference (3 unconnected both ways;
68 vs 66 violations, the two extra errors plausibly from the wider 0.5 mm power tracks in
tight THT geometry). The explanation is structural, not a surprise in hindsight:

1. **This board has nothing for constraints to constrain.** It's a classic through-hole
   design: no differential pairs, no high-current SMD rails, no length-sensitive nets.
   The classifier produced exactly one class (Power, 2 nets). The constraint wedge
   targets boards whose nets *need* net classes — modern SMD designs with USB/RF pairs,
   switching regulators, dense fanout.
2. **Both arms hit the same wall:** 3 unconnected items in both runs is Freerouting's
   documented copper-pour/GND limitation, which no constraint set fixes.

**Takeaways** (which define the next iteration):
- Build the benchmark suite from boards with constraint-sensitive content (USB diff
  pairs, buck converters, mixed-signal) — open-hardware candidates, not 1980s demos.
- Constraint generation must be *conditional*: on boards with no constraint-sensitive
  nets, emit nothing rather than well-meaning width bumps that cost DRC errors.
- Track Freerouting effort settings as a variable; default effort may underuse vias
  (0–2 vias on a 2-layer board suggests minimal layer switching).

A null result published with its mechanism beats an unmeasured claim. The pipeline that
produced this table is reproducible: `python scripts/ablation_route.py`.
