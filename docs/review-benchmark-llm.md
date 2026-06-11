# Reviewer benchmark — seeded errors

Mode: **rules + llm (qwen3:4b)** · 10 cases (4 clean controls)

**Recall 1.00 · Precision 1.00** (9/9 expected found, 0 false positives)

| Case | expected | found | false-pos | |
|---|---|---|---|---|
| clean_mcu | 0 | 0 | 0 | ✅ |
| floating_enable | 1 | 1 | 0 | ✅ |
| floating_irq | 1 | 1 | 0 | ✅ |
| gnd_only_ok | 0 | 0 | 0 | ✅ |
| i2c_no_pullup | 2 | 2 | 0 | ✅ |
| i2c_pinfunction | 1 | 1 | 0 | ✅ |
| multi_fault | 3 | 3 | 0 | ✅ |
| no_decoupling | 1 | 1 | 0 | ✅ |
| sd_lines_ok | 0 | 0 | 0 | ✅ |
| unused_output_ok | 0 | 0 | 0 | ✅ |
