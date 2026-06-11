# Routing ablation — naked vs TraceWise-constrained Freerouting

Same board, all routing stripped, routed twice: with KiCad's stock settings (naked) and after `tracewise constrain` generated net classes + rules (constrained). Freerouting 2.2.4, default effort.

| Board | Mode | unconnected | DRC violations | vias | length (mm) |
|---|---|---|---|---|---|
| rp2040-dev-board | naked | 23 | 197 | 26 | 838.7 |
| rp2040-dev-board | constrained | 30 | 195 | 32 | 730.2 |
| mitayi-pico-d1 | naked | 89 | 4 | 26 | 226.9 |
| mitayi-pico-d1 | constrained | 89 | 503 | 21 | 211.7 |
| zuluscsi-pico-oshw | naked | 27 | 488 | 94 | 4031.6 |
| zuluscsi-pico-oshw | constrained | 21 | 986 | 93 | 3902.1 |
