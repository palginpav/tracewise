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

from tracewise.route.bridge import drc_summary, run_drc, strip_routing
from tracewise.route.engine.kicad import route_board_engine


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
            # arm 2: components of nets that failed in ALL prior rounds get
            # re-placed with weighted wirelength before this round routes
            stubborn = {n for n, c in persist.items() if c >= it}
            if stubborn:
                moved = nudge_placement(board, stubborn)
                strip_routing(board)  # placement nudge leaves no copper edits,
                # but normalize through pcbnew for a consistent baseline
                pristine = board.read_bytes()  # nudged placement becomes baseline
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
