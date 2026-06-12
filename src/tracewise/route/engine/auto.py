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


def eccf_candidates(board: Path, failed_nets: set[str],
                    error_sites: list | None = None):
    """Candidate fixes screened by validated T2 (data-level: extract once,
    patch pad coords per candidate — no board writes). Sources: small parts on
    failing nets AND parts near DRC violation sites; menu: rotation + 1.5mm
    8-dir + 3.0mm 4-dir nudges. Returns sorted (delta, [(ref,x,y,rot),...])."""
    from tracewise.place.extract import extract
    from tracewise.route.engine.eccf import patch_data, t2_score
    from tracewise.route.engine.kicad import extract_pads, project_geometry

    data = extract(board)
    rdata = extract_pads(board)
    geo = project_geometry(board)
    parts, seen = [], set()
    for fp in data["footprints"]:
        if fp["locked"] or len(fp["pads"]) > 4 or fp["ref"] in seen:
            continue
        near_err = error_sites and any(
            abs(fp["x"] - ex) < 1.5 and abs(fp["y"] - ey) < 1.5
            for ex, ey in error_sites)
        if near_err or any(p["net"] in failed_nets for p in fp["pads"]):
            parts.append(fp)
            seen.add(fp["ref"])
    parts = parts[:6]
    cands = []
    for fp in parts:
        ref = fp["ref"]
        cands.append((ref, fp["x"], fp["y"], 90.0))
        for d, dirs in ((1.5, ((1, 0), (-1, 0), (0, 1), (0, -1),
                               (1, 1), (-1, 1), (1, -1), (-1, -1))),
                        (3.0, ((1, 0), (-1, 0), (0, 1), (0, -1)))):
            for dx, dy in dirs:
                cands.append((ref, fp["x"] + dx * d, fp["y"] + dy * d, 0.0))

    refs = {c[0] for c in cands}
    base = {r: t2_score(board, {r}, data=rdata, geo=geo) for r in refs}
    scored = []
    for ref, nx, ny, rot in cands:
        d2 = patch_data(rdata, ref, nx, ny, rot)
        delta = t2_score(board, {ref}, data=d2, geo=geo) - base[ref]
        scored.append((delta, [(ref, nx, ny, rot)]))
    scored.sort(key=lambda s: s[0])
    # multi-part combos: best two single moves on DIFFERENT parts, applied together
    singles = [s for s in scored if s[0] < -1.0]
    combos = []
    for i, (da, ma) in enumerate(singles[:4]):
        for db, mb in singles[i + 1:5]:
            if ma[0][0] != mb[0][0]:
                combos.append((da + db, ma + mb))
                break
    return sorted(scored + combos, key=lambda s: s[0])


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
    err_sites: list = []
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

                scored = eccf_candidates(board, stubborn, error_sites=err_sites)
                screened = [c for c in scored if c[0] < -1.0][:4]  # T2 screen
                if screened:
                    base_cost, base_fail = t3_verify(board, stubborn)
                    pick = None
                    for _delta, moves_list in screened:
                        apply_positions(board, {r: (x, y, ro)
                                                for r, x, y, ro in moves_list})
                        cost, fail = t3_verify(board, stubborn)
                        board.write_bytes(pristine)
                        key = (fail, cost)
                        improves = key < (base_fail, base_cost)
                        if improves and (pick is None or key < pick[0]):
                            pick = (key, moves_list)
                    if pick:  # T3-verified improvement only
                        _, moves_list = pick
                        apply_positions(board, {r: (x, y, ro)
                                                for r, x, y, ro in moves_list})
                        moved = len(moves_list)
                        pristine = board.read_bytes()
        summary = route_board_engine(board, priority=priority,
                                     ripup_factor=8 + 4 * it)
        report = run_drc(board)
        drc = drc_summary(report)
        err_sites = [(v["items"][0]["pos"]["x"], v["items"][0]["pos"]["y"])
                     for v in report.get("violations", [])
                     if v["type"] in ("clearance", "hole_clearance")
                     and v.get("items")][:20]
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
