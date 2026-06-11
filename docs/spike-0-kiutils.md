# Spike 0 — kiutils round-trip fidelity: NO-GO

**Date:** 2026-06-11 · **Harness:** `scripts/spike_kiutils.py` (fetch official KiCad demos →
parse → write → re-parse → token-level diff)

## Results

| File | Format | parse | reload | tokens orig→rt | missing |
|---|---|---|---|---|---|
| pic_programmer.kicad_sch | v10 | ✅ | ✅ | 41,977 → 35,521 | **7,001 (16.7%)** |
| pic_programmer.kicad_pcb | v10 | ❌ IndexError | — | — | — |
| complex_hierarchy.kicad_sch | v10 | ✅ | ✅ | 10,798 → 10,049 | **884 (8.2%)** |
| complex_hierarchy.kicad_pcb | v10 | ✅ | ❌ corrupt output | — | — |
| flat_hierarchy.kicad_sch | v9 | ✅ | ✅ | 298 → 258 | 44 (14.8%) |
| pic_programmer.kicad_sch | v9 | ✅ | ✅ | 35,882 → 33,642 | 2,785 (7.8%) |
| *.kicad_pcb | v9 | parse ok | ❌ corrupt output | — | — |

**0/9 clean round-trips across KiCad 9 and 10 formats.**

## Verdict

kiutils (last release Feb 2024) cannot back TraceWise's file layer:
- board files either fail to parse (v10) or produce corrupt output on write (v9 and v10)
- schematic files round-trip **with silent loss of 8–17% of tokens** — the worst failure class
  for a tool that patches user files

This confirms the risk identified in the design review and is precisely why this spike ran first.

## Decision — own lossless s-expression layer

TraceWise's actual file needs are narrow:
1. **Reads** come mostly from `kicad-cli` exports (netlist, ERC/DRC JSON) — not from parsing
   project files semantically
2. **Writes** (the Fixer) are *surgical insertions* into `.kicad_sch` — they must preserve
   everything else byte-for-byte

So instead of a full semantic model of every format node (kiutils' approach — large surface,
breaks on format evolution), we build a small **lossless s-expression editor**: parse to a
concrete syntax tree that retains whitespace/ordering/comments, support targeted node
insertion/modification, and write back byte-identical except for the edit. Version-agnostic by
construction; testable with exactly this spike's token-fidelity harness (which becomes the
regression suite).

`scripts/spike_kiutils.py` stays in the repo as the evaluation harness for any future
third-party parser (e.g., a revived kiutils or KiCad's own future libraries).
