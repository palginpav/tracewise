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
                    error_sites: list | None = None, include_flips: bool = False):
    """Candidate fixes screened by validated T2 (data-level: extract once,
    patch pad coords per candidate — no board writes). Move tuples are
    (ref, x, y, rot, flip). Sources: nudges/rotation of small parts on failing
    nets or near DRC sites, PLUS layer-FLIP of passives in the 'center of the
    storm' (densest congestion) — the human relief move for a packed top side.
    Returns sorted (delta, [(ref,x,y,rot,flip),...])."""
    from tracewise.place.extract import extract
    from tracewise.route.engine.eccf import patch_data, rank_by_storm, t2_score
    from tracewise.route.engine.kicad import extract_pads, project_geometry

    data = extract(board)
    rdata = extract_pads(board)
    geo = project_geometry(board)
    error_sites = error_sites or []
    parts, seen = [], set()
    for fp in data["footprints"]:
        if fp["locked"] or len(fp["pads"]) > 4 or fp["ref"] in seen:
            continue
        near_err = any(abs(fp["x"] - ex) < 1.5 and abs(fp["y"] - ey) < 1.5
                       for ex, ey in error_sites)
        if near_err or any(p["net"] in failed_nets for p in fp["pads"]):
            parts.append(fp)
            seen.add(fp["ref"])
    parts = parts[:6]
    cands = []
    for fp in parts:
        ref = fp["ref"]
        cands.append((ref, fp["x"], fp["y"], 90.0, False))
        for d, dirs in ((1.5, ((1, 0), (-1, 0), (0, 1), (0, -1),
                               (1, 1), (-1, 1), (1, -1), (-1, -1))),
                        (3.0, ((1, 0), (-1, 0), (0, 1), (0, -1)))):
            for dx, dy in dirs:
                cands.append((ref, fp["x"] + dx * d, fp["y"] + dy * d, 0.0, False))

    # "center of the storm" escalation (only when the normal arm has stalled):
    # flip passives where failing-net pads + error sites cluster densest, to
    # the other copper side. T2 cannot value a flip (its benefit is freeing a
    # corridor for OTHER nets — an externality), so the loop routes flips
    # straight to T3; here we just emit them with their (usually positive) T2
    # delta so the loop can find them by the flip flag.
    if include_flips:
        hot = list(error_sites)
        for fp in data["footprints"]:
            for p in fp["pads"]:
                if p["net"] in failed_nets:
                    hot.append((fp["x"] + p.get("dx", 0.0),
                                fp["y"] + p.get("dy", 0.0)))
        passives = [fp for fp in data["footprints"]
                    if not fp["locked"] and fp["ref"][:1] in ("C", "R")
                    and len(fp["pads"]) <= 2]
        for _n, fp in rank_by_storm(passives, hot)[:4]:
            cands.append((fp["ref"], fp["x"], fp["y"], 0.0, True))  # flip in place

    refs = {c[0] for c in cands}
    base = {r: t2_score(board, {r}, data=rdata, geo=geo) for r in refs}
    scored = []
    for ref, nx, ny, rot, flip in cands:
        d2 = patch_data(rdata, ref, nx, ny, rot, flip=flip)
        delta = t2_score(board, {ref}, data=d2, geo=geo) - base[ref]
        scored.append((delta, [(ref, nx, ny, rot, flip)]))
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
    stall = 0  # rounds since the last improvement; gates the flip escalation

    for it in range(max_iters):
        board.write_bytes(pristine)
        if placement_arm and it > 0:
            # arm 2 v2: ECCF-screened single-part fixes for stubborn nets
            stubborn = {n for n, c in persist.items() if c >= it}
            if stubborn:
                from tracewise.place.extract import apply_positions
                from tracewise.route.engine.eccf import t3_verify

                # flip escalation only after the normal arm stalls (the
                # "reaching the unconnected floor" trigger)
                flip_now = stall >= 2
                scored = eccf_candidates(board, stubborn, error_sites=err_sites,
                                         include_flips=flip_now)
                # split quota so combos cannot crowd the proven single moves out
                # of the T3 budget (measured regression: 56 -> 63 when they did)
                impr = [c for c in scored if c[0] < -1.0]
                singles = [c for c in impr
                           if len(c[1]) == 1 and not c[1][0][4]][:3]
                combos = [c for c in impr if len(c[1]) > 1][:1]
                # flips bypass the T2 gate (T2 cannot see their externality);
                # least self-harm first, T3 is the arbiter
                flips = [c for c in scored if len(c[1]) == 1 and c[1][0][4]][:2]
                screened = singles + combos + flips  # T2 screen, quota-split
                if screened:
                    base_cost, base_fail = t3_verify(board, stubborn)
                    pick = None
                    for _delta, moves_list in screened:
                        apply_positions(board, {r: (x, y, ro, fl)
                                                for r, x, y, ro, fl in moves_list})
                        cost, fail = t3_verify(board, stubborn)
                        board.write_bytes(pristine)
                        key = (fail, cost)
                        improves = key < (base_fail, base_cost)
                        if improves and (pick is None or key < pick[0]):
                            pick = (key, moves_list)
                    if pick:  # T3-verified improvement only
                        _, moves_list = pick
                        apply_positions(board, {r: (x, y, ro, fl)
                                                for r, x, y, ro, fl in moves_list})
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
            stall = 0
        else:
            stall += 1
        if score == (0, 0):
            break
        for name in summary["failures"]:
            priority[name] = priority.get(name, 0) + 1
            persist[name] = persist.get(name, 0) + 1

    if best_bytes is not None:
        board.write_bytes(best_bytes)
    return {"best_unconnected": best[0], "best_errors": best[1],
            "iterations": history}
