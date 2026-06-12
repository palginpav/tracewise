"""Sequential place-route refinement toward 0 errors / 0 unconnected.

The operator's insight this implements: unconnected items are not only router
defects — they are a routability signal about the placement and the net
ordering. Each iteration feeds the previous round's failures back:

- arm 1 (this version): failed nets gain ordering priority (they route FIRST
  next round, claiming corridors before easy nets consume them) and the
  rip-up budget escalates
- arm 2 (planned): components of persistently-failing nets get re-placed
  with weighted wirelength before the next routing round

Every iteration starts from the same pristine (stripped) board; the best
result by (unconnected, errors) is kept — a worse iteration rolls back.
"""

from __future__ import annotations

from pathlib import Path

from tracewise.route.bridge import drc_summary, run_drc
from tracewise.route.engine.kicad import route_board_engine


def eccf_candidates(board: Path, failed_nets: set[str], max_cands: int = 8):
    """Candidate single-part fixes for failing nets: 90-degree rotations and
    trust-region nudges of small parts, screened by validated T2 scoring
    (docs/PLACE-ROUTE-COUPLING.md gates: V1 100%, V2 rho 0.579)."""
    from tracewise.place.extract import extract
    from tracewise.route.engine.eccf import t2_score

    data = extract(board)
    parts = []
    for fp in data["footprints"]:
        if fp["locked"] or len(fp["pads"]) > 4:
            continue
        if any(p["net"] in failed_nets for p in fp["pads"]):
            parts.append(fp)
    parts = parts[:3]  # few parts, several moves each
    cands = []
    for fp in parts:
        ref = fp["ref"]
        cands.append((ref, fp["x"], fp["y"], 90.0))  # rotation
        for dx, dy in ((1.5, 0), (-1.5, 0), (0, 1.5), (0, -1.5)):
            cands.append((ref, fp["x"] + dx, fp["y"] + dy, 0.0))
    cands = cands[:max_cands]

    pristine = board.read_bytes()
    refs = {c[0] for c in cands}
    base_scores = {r: t2_score(board, {r}) for r in refs}
    scored = []
    from tracewise.place.extract import apply_positions
    for ref, nx, ny, rot in cands:
        apply_positions(board, {ref: (nx, ny, rot)})
        delta = t2_score(board, {ref}) - base_scores[ref]
        board.write_bytes(pristine)
        scored.append((delta, ref, nx, ny, rot))
    scored.sort()
    return scored


def nudge_placement(board: Path, failed_nets: set[str], weight: float = 4.0) -> int:
    """Arm 2: re-place ONLY the components touching persistently-failing nets,
    with those nets' wirelength weighted up — everything else locked. Returns
    the number of components moved."""
    from tracewise.place.core import build_problem as place_problem
    from tracewise.place.core import optimize
    from tracewise.place.extract import apply_positions, extract

    data = extract(board)
    movable = set()
    for fp in data["footprints"]:
        if fp["locked"]:
            continue
        if any(p["net"] in failed_nets for p in fp["pads"]):
            movable.add(fp["ref"])
    if not movable:
        return 0
    lock = {fp["ref"] for fp in data["footprints"]} - movable
    weights = {n: weight for n in failed_nets}
    prob = place_problem(data, lock_refs=lock, net_weights=weights)
    r = optimize(prob, iters=400)
    apply_positions(board, r["positions"])
    return len(r["positions"])


def auto_route(board: str | Path, max_iters: int = 5, placement_arm: bool = True) -> dict:
    board = Path(board)
    pristine = board.read_bytes()  # caller provides the stripped board
    priority: dict[str, int] = {}
    persist: dict[str, int] = {}
    best: tuple[int, int] | None = None
    best_bytes: bytes | None = None
    history = []
    moved = 0

    for it in range(max_iters):
        board.write_bytes(pristine)
        if placement_arm and it > 0:
            # arm 2 v2: ECCF-screened single-part fixes for stubborn nets
            stubborn = {n for n, c in persist.items() if c >= it}
            if stubborn:
                from tracewise.place.extract import apply_positions
                from tracewise.route.engine.eccf import t3_verify

                scored = eccf_candidates(board, stubborn)
                screened = [c for c in scored if c[0] < -1.0][:3]  # T2 screen
                if screened:
                    base_cost, base_fail = t3_verify(board, stubborn)
                    pick = None
                    for _delta, ref, nx, ny, rot in screened:
                        apply_positions(board, {ref: (nx, ny, rot)})
                        cost, fail = t3_verify(board, stubborn)
                        board.write_bytes(pristine)
                        key = (fail, cost)
                        improves = key < (base_fail, base_cost)
                        if improves and (pick is None or key < pick[0]):
                            pick = (key, ref, nx, ny, rot)
                    if pick:  # T3-verified improvement only
                        _, ref, nx, ny, rot = pick
                        apply_positions(board, {ref: (nx, ny, rot)})
                        moved = 1
                        pristine = board.read_bytes()
        summary = route_board_engine(board, priority=priority,
                                     ripup_factor=8 + 4 * it)
        drc = drc_summary(run_drc(board))
        score = (drc["unconnected"], drc["by_severity"].get("error", 0))
        history.append({"iter": it, "routed": f"{summary['routed']}/{summary['nets']}",
                        "unconnected": score[0], "errors": score[1],
                        "boosted": sum(1 for v in priority.values() if v),
                        "moved": moved})
        moved = 0
        if best is None or score < best:
            best, best_bytes = score, board.read_bytes()
        if score == (0, 0):
            break
        for name in summary["failures"]:
            priority[name] = priority.get(name, 0) + 1
            persist[name] = persist.get(name, 0) + 1

    if best_bytes is not None:
        board.write_bytes(best_bytes)
    return {"best_unconnected": best[0], "best_errors": best[1],
            "iterations": history}
