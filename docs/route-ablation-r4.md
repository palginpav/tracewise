# Routing ablation — naked vs TraceWise-constrained Freerouting

Same board, all routing stripped, routed twice: with KiCad's stock settings (naked) and after `tracewise constrain` generated net classes + rules (constrained). Freerouting 2.2.4, default effort.

| Board | Mode | unconnected | DRC violations | dangling | vias | F/B split | length (mm) |
|---|---|---|---|---|---|---|---|
| mitayi-pico-d1 | naked | 89 | 4 | 0 | 26 | 161/57 | 226.9 |
| mitayi-pico-d1 | engine | 80 | 78 | 15 | 36 | 380/174 | 550.9 |
| rp2040-dev-board | naked | 23 | 197 | 0 | 26 | 418/90 | 838.7 |
| rp2040-dev-board | engine | 74 | 216 | 12 | 19 | 380/87 | 512.1 |
| zuluscsi-pico-oshw | naked | 27 | 486 | 0 | 94 | 0/1362 | 4031.6 |
| zuluscsi-pico-oshw | engine | 115 | 610 | 47 | 298 | 0/2543 | 4461.7 |

## Analysis (R4 — first engine-vs-incumbent benchmark)

**On the design-scope board (mitayi, dense 2-layer):** the engine routes *more* of the board
than Freerouting (80 vs 89 unconnected) at the cost of more violations (78 vs 4) — the
fine-pitch shaving and emission gaps documented in PLAN. Competitive, not winning.

**Out of scope, measured anyway (rp2040, 4-layer):** engine 74 vs FR 23 — the engine models
2 layers by design (v1 scope), so it routes this board with half its routing resources. The
gap is the price of the scope ladder, now quantified: 4-layer support is the single biggest
completion lever for v2.

**On the big board (zuluscsi, 125 components):** engine 115 vs 27 — rip-up thrash at scale
plus single-sided congestion (note both routers put ~everything on B.Cu: the component side
is pad-saturated). Scale behavior is a real gap, not noise.

**A falsified guarantee, kept on the record:** "zero dangling stubs by construction" held
topologically (connection trees admit no partial nets) but DRC counts 12–47 dangling items —
the *emission* layer lands track ends on grid cells that don't always touch pad copper.
The guarantee was about graph structure; DRC measures geometry. Fix: snap terminal segments
to pad anchors at emit. (Freerouting scored 0 dangling on these from-scratch runs — its
35–48-stub behavior appeared in different conditions; both observations stand.)

**What R4 established:** a reproducible 3-board, 8-column benchmark
(`scripts/ablation_route.py --modes naked,engine`) that the engine can be improved against,
run by run. v2 priorities, in measured order: terminal-segment landing, 4-layer support,
rip-up thrash at scale, width-necking for fine pitch.
