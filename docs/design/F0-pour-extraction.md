# F0 — Robust pour / connectivity extraction (design)

Status: design complete (2026-06-17). Foundation for F2 (GND stitch) and F3 (+3V0 pour).
Authored by PM (architect subagent blocked by an unrelated worktree hook; PM has full
file-level context from the determinism/diagnosis work).

## 1. Decision: data source — pcbnew API, not s-expr

Two things must be extracted from a routed board:
(a) pour-copper GEOMETRY per (net, layer) — to rasterize into the routing grid as stitch goals;
(b) the PADS not electrically connected to their net's pour — the stitch worklist.

**Use the pcbnew API via the existing `_run_pcbnew_script` JSON pattern (bridge.py).** Reasons:
- The s-expr `net_name` parse is already known to break: it reads on the source board but
  returns `None` on a pcbnew-resaved board (format variance — net carried as numeric `(net N)`
  with the name in a top-level declaration, or reordered). Building the foundation on this is
  fragile. pcbnew exposes the name directly: `zone.GetNetname()` / `pad.GetNetname()`.
- pcbnew OWNS the fill and the connectivity graph. `ZONE.GetFilledPolysList(layer)` returns the
  EXACT filled geometry (post-`refill_zones`), and `BOARD.GetConnectivity()` reports unconnected
  items with thermal-relief awareness — exactly what DRC uses. Re-deriving either from s-expr
  would duplicate pcbnew's logic and drift.
- Precedent in-repo: `board_metrics`, `strip_routing`, `refill_zones` already drive pcbnew via
  `_run_pcbnew_script(...)` and parse a single JSON line from stdout. F0 follows the same shape.

s-expr stays the source of truth ONLY for the static problem build (`extract_pads`,
`build_problem`) which already works; F0 adds pcbnew-side extraction for post-fill geometry +
connectivity.

## 2. API spec

New module: `src/tracewise/route/engine/pours.py` (net-agnostic; reused by F2 GND and F3 +3V0).

```python
# A filled pour region on one layer (mm coordinates, pcbnew → ÷1e6).
@dataclass
class PourPoly:
    net: str          # e.g. "GND"
    layer: int        # 0 = F.Cu, 1 = B.Cu
    pts: list[tuple[float, float]]   # outline vertices, mm

@dataclass
class IsolatedPad:
    net: str
    ref: str          # footprint ref, e.g. "U1"
    pad: str          # pad name/number
    x: float; y: float   # pad centre, mm
    layers: tuple[int, ...]   # copper layers the pad is on

def extract_pours(board: str | Path) -> dict[str, list[PourPoly]]:
    """Post-fill pour geometry per net. pcbnew: for z in b.Zones(): for layer in
    z.GetLayerSet().CuStack(): GetFilledPolysList(layer) → outline points (nm→mm).
    Emits one JSON line; parsed like board_metrics."""

def unconnected_pads(board: str | Path) -> list[IsolatedPad]:
    """Pads not connected to their net (thermal-relief aware). pcbnew connectivity:
    BOARD.GetConnectivity() / GetRatsnestForNet, OR the same source DRC uses for
    unconnected_items. Returns only pads whose net HAS a pour (filter against
    extract_pours keys) — the stitch worklist."""

def rasterize_pour(grid: Grid, polys: list[PourPoly]) -> "np.ndarray":
    """bool[L,H,W] mask of pour-copper cells, for use as A* stitch GOALS. Ray-cast
    point-in-polygon over each poly's cell bbox (reuse the vectorized algorithm in
    Grid.block_polygon — note its Y-unpack bug is already fixed). Union across polys.
    mm→cell via grid.to_cell; layer from PourPoly.layer."""
```

## 3. Grid integration

- `rasterize_pour` produces a SEPARATE bool mask, NOT written into `grid.cells`. The pour is a
  routing GOAL for the stitch, not an obstacle. (Obstacles stay in `grid.cells`/`grid.hard` as
  built by `build_problem`.)
- Coordinate transform: pcbnew returns nm → mm (`/1e6`) → `grid.to_cell(x_mm, y_mm)` → (iy, ix).
  Layer: `F.Cu→0`, `B.Cu→1` (matches `layer_name` in `emit_routes`).
- F2 consumes this: for each `IsolatedPad`, run the existing A* (`astar.route`) from the pad cell
  to the set of `True` cells in the mask as multi-goal. The vectorized heuristic shipped
  2026-06-17 makes a large pour goal set tractable; bound the search with a radius window so a
  stitch stays short.

## 4. Edge cases

- **Fragmented islands.** `GetFilledPolysList` returns many polygons; the mask is their union.
  RISK: stitching a pad to island A only helps if A is electrically the net's main body.
  Mitigation (hand to F2): identify the "main" pour component (the island with the most
  already-connected pads, or via pcbnew connectivity netcode) and prefer it as the goal; or
  stitch + re-run connectivity and iterate.
- **Multi-layer pour (F&B).** Two masks, one per layer; the stitch A* may use a via to reach the
  pour on the opposite layer (the router already prices vias). Target whichever layer is nearer.
- **Pad partially in pour / thermal relief.** pcbnew connectivity already counts these as
  connected, so `unconnected_pads` correctly excludes them — do NOT re-implement this in Python.
- **Net with no pour.** `extract_pours` returns no entry for the net → F2 skips it; F3 SYNTHESISES
  a pour first (its own step) then calls F0 on the result.

## 5. Test plan (anti-fragility gate)

Fixture: a tiny 2-layer board, one GND zone, 3 GND pads — 2 inside the pour (connected via fill),
1 isolated by a clearance moat (unconnected). Commit BOTH a source-format copy and a
pcbnew-resaved copy (`fixtures/pour_source.kicad_pcb`, `fixtures/pour_resaved.kicad_pcb`).

- `test_extract_pours_identical_across_formats`: `extract_pours` on source vs resaved → same net
  keys ("GND"), same polygon count and total area within tolerance. **This is the core gate that
  defeats the net_name fragility.**
- `test_unconnected_pads_finds_only_isolated`: returns exactly the 1 isolated pad, correct
  ref/coords; identical on both formats.
- `test_rasterize_pour_marks_interior`: a known rectangle pour → interior cells True, exterior
  False, correct layer, off-by-one-safe at the bbox edges.
- Skip-if-no-kicad guard (mirror existing pcbnew-dependent tests) so CI without kicad-cli is green.

## 6. Risks & open questions for the implementer

- pcbnew API surface varies by KiCad version (`GetFilledPolysList` arg, layer iteration). Pin to
  the installed version (the repo already runs pcbnew via `_run_pcbnew_script`); verify the exact
  calls against it before writing the JSON emitter.
- Island-connectivity (which island IS the net) is the main open question — belongs to F2's
  design, but F0 should expose enough (per-polygon, and ideally pcbnew netcode/component id) for
  F2 to pick the right target. Recommend `PourPoly` carry an optional `island_id` from pcbnew.
- Performance: rasterizing many polygons on zuluscsi's 1.8M-cell grid — reuse the bbox-bounded
  vectorized ray cast from `block_polygon`; one-time cost, acceptable.
- Determinism: extraction must be order-stable (sort zones/pads by a stable key) so F2/F3 measure
  cleanly against the deterministic router.
