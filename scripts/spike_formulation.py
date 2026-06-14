#!/usr/bin/env python3
"""
spike_formulation.py -- numerical demo of ONE formal insight from docs/FORMULATION.md:

    "Negotiated congestion (PathFinder) prices a shared resource instead of
     forbidding it, and so resolves a 2-net conflict that a greedy/forbid-wall
     router is forced to SHORT (overlap = DRC error)."

This is the §5.2(a)/§6 claim made numerical on a tiny synthetic instance. numpy only.

Run:  taskset -c 0-9 python3 scripts/spike_formulation.py

The instance (a shared-bottleneck gadget):
  A vertical wall splits the board left<->right with TWO crossings:
    - GAP_ROW (center, row 3): the cheap crossing both nets' shortest paths want.
    - DET_ROW (top, row 0): a second crossing that exists but is strictly LONGER to use
      (a net must climb to row 0 and back, costing extra over the direct gap route).
  cap(each cell) = 1.

  GREEDY-FORBID (the current TraceWise model): route net0 (it takes the gap), then net1
  finds the gap forbidden. A legal detour exists, but it costs more than `force_threshold`
  over the direct route, so the budgeted router "forces the connection" through the gap
  -- modelling the measured 'shave clearance / push through to connect' pathology. Result:
  OVERUSE at the gap == a SHORT (DRC error).

  PATHFINDER: price the gap up over a few iterations until taking the detour for ONE net
  is cheaper than sharing the gap. Result: both nets connected, zero overuse -- the §6
  claim ("pricing contention beats forbidding it") in miniature.
"""

import heapq

import numpy as np

# ----------------------------------------------------------------------------- grid
# 4-connected grid, H x W. base[c] = traversal base-cost of a cell; +inf = blocked.

H, W = 7, 9
INF = float("inf")

base = np.ones((H, W), dtype=float)

# vertical wall at column 4, blocking ALL rows except two crossings:
#   GAP_ROW (center, row 3) -- the cheap, contended crossing both nets prefer
#   DET_ROW (top,    row 0) -- the detour crossing (longer to reach)
WALL_COL = 4
GAP_ROW = 3
DET_ROW = 0
for r in range(H):
    if r not in (GAP_ROW, DET_ROW):
        base[r, WALL_COL] = INF

cap = np.ones((H, W), dtype=float)  # one net per cell

# Two nets routed STRAIGHT across their own rows on each side, but BOTH must funnel
# through column 4. Net0 lives on row 3 (the gap row -> direct). Net1 lives on row 2
# (left) and row 4 (right); to cross it must use the gap (row3) or climb to the
# detour (row0). Sources/sinks are kept off each other's rows so the only structural
# contention is the WALL CROSSING itself -- a clean single-bottleneck gadget.
NETS = [
    {"name": "net0", "src": (3, 0), "snk": (3, 8)},   # wants the gap, straight line
    {"name": "net1", "src": (2, 0), "snk": (2, 8)},   # also wants the gap (row2->row3->row2)
]

NEI = [(-1, 0), (1, 0), (0, -1), (0, 1)]


def neighbors(r, c):
    for dr, dc in NEI:
        nr, nc = r + dr, c + dc
        if 0 <= nr < H and 0 <= nc < W:
            yield nr, nc


def dijkstra(src, snk, cellcost):
    """Shortest path src->snk where entering cell x costs cellcost[x].
    Returns (path_list, total_cost) or (None, inf) if unreachable."""
    (sr, sc), (tr, tc) = src, snk
    dist = np.full((H, W), INF)
    prev = {}
    dist[sr, sc] = cellcost[sr, sc]
    pq = [(dist[sr, sc], sr, sc)]
    while pq:
        d, r, c = heapq.heappop(pq)
        if d > dist[r, c]:
            continue
        if (r, c) == (tr, tc):
            break
        for nr, nc in neighbors(r, c):
            cc = cellcost[nr, nc]
            if cc == INF:
                continue
            nd = d + cc
            if nd < dist[nr, nc]:
                dist[nr, nc] = nd
                prev[(nr, nc)] = (r, c)
                heapq.heappush(pq, (nd, nr, nc))
    if dist[tr, tc] == INF:
        return None, INF
    # reconstruct
    path = [(tr, tc)]
    while path[-1] != (sr, sc):
        path.append(prev[path[-1]])
    path.reverse()
    return path, dist[tr, tc]


def usage_of(paths):
    u = np.zeros((H, W), dtype=int)
    for p in paths:
        if p is None:
            continue
        for (r, c) in p:
            u[r, c] += 1
    return u


def report(tag, paths):
    u = usage_of(paths)
    overuse = np.maximum(0, u - cap.astype(int))
    shorts = int(overuse.sum())
    connected = sum(p is not None for p in paths)
    over_cells = [(int(r), int(c), int(u[r, c]))
                  for r in range(H) for c in range(W) if overuse[r, c] > 0]
    print(f"  [{tag}] connected={connected}/{len(paths)}  "
          f"shorts(overuse cells*count)={shorts}  over_cells={over_cells}")
    return connected, shorts


# ----------------------------------------------------------------------------- greedy
def route_greedy_forbid(force_threshold=1.0):
    """Greedy sequential, capacity as a HARD WALL (the current TraceWise model).

    Route net0 freely (it takes the cheap center gap). Then net1 must avoid net0's
    cells (forbid). A forbid-respecting path EXISTS (the top detour), but it is much
    longer. We model the measured TraceWise behavior under its bounded budget: when
    the legal detour costs more than `force_threshold` over the blocked-but-direct
    path, the router 'forces the connection' through the contended gap -> OVERUSE ->
    a SHORT. This is the formal statement of the measured
    'more-time -> more-connections-but-more-violations / shave clearance to connect'.
    """
    print("GREEDY (capacity = hard wall; forces connection when the legal detour is costly):")
    occupied = np.zeros((H, W), dtype=bool)
    paths = []
    for net in NETS:
        forbid = base.copy()
        forbid[occupied] = INF
        forbid[net["src"]] = base[net["src"]]
        forbid[net["snk"]] = base[net["snk"]]
        p_legal, c_legal = dijkstra(net["src"], net["snk"], forbid)
        # cost if we ignored occupancy (the direct/contended route)
        p_direct, c_direct = dijkstra(net["src"], net["snk"], base.copy())
        if p_legal is None or (c_legal - c_direct) > force_threshold:
            # budgeted router gives up on the long detour -> forces the direct route -> short
            p = p_direct
        else:
            p = p_legal
        for (r, c) in p:
            occupied[r, c] = True
        paths.append(p)
    return report("greedy", paths)


# --------------------------------------------------------------------- negotiated
def route_pathfinder(iters=12, h_fac=0.5, p_growth=1.6):
    """PathFinder negotiated congestion: present-congestion + history pricing,
    re-route over-capacity nets each iteration. (docs/FORMULATION.md §5.3)"""
    print("PATHFINDER (negotiated congestion: price overuse, iterate):")
    hist = np.zeros((H, W), dtype=float)
    p_fac = 0.5
    paths = [None] * len(NETS)
    for it in range(iters):  # noqa: B007 (it is used after the loop)
        u = usage_of(paths)
        over = (u - cap) > 0
        # nets touching an over-capacity cell (or not yet routed) must be rerouted
        to_route = [i for i, p in enumerate(paths)
                    if p is None or any(over[r, c] for (r, c) in p)]
        if not to_route:
            break
        # Canonical PathFinder: reroute SEQUENTIALLY, removing the net's own usage and
        # recomputing the present-congestion price from the *current* paths after each
        # net. This breaks the symmetry that makes a simultaneous reroute oscillate:
        # the second net sees the congestion the first just (re)committed.
        for i in to_route:
            paths[i] = None  # rip up this net
            u_now = usage_of(paths)  # usage of everyone ELSE currently
            present = np.maximum(0.0, u_now + 1.0 - cap)  # cost of ADDING this net here
            cost = base * (1.0 + hist * h_fac) * (1.0 + present * p_fac)
            cost[base == INF] = INF
            net = NETS[i]
            cost[net["src"]] = base[net["src"]]
            cost[net["snk"]] = base[net["snk"]]
            paths[i], _ = dijkstra(net["src"], net["snk"], cost)
        # history: accumulate end-of-iteration overuse; ramp present-cost weight
        u2 = usage_of(paths)
        hist += np.maximum(0.0, u2 - cap)
        p_fac *= p_growth
    return report("pathfinder", paths), it + 1


# ----------------------------------------------------------------------------- main
def main():
    np.set_printoptions(linewidth=120)
    print("=" * 70)
    print("FORMULATION SPIKE: forbid-wall router SHORTS a 2-net conflict that")
    print("negotiated-congestion (PathFinder) resolves cleanly. (docs/FORMULATION.md §5.2a)")
    print("=" * 70)
    print(f"grid {H}x{W}; wall at col {WALL_COL} with single gap at row {GAP_ROW}; "
          f"both nets' geodesics cross the gap.\n")

    g_conn, g_short = route_greedy_forbid()
    print()
    (p_conn, p_short), p_iters = route_pathfinder()
    print()
    print("-" * 70)
    print(f"RESULT: greedy   -> connected={g_conn}/2, shorts={g_short}")
    print(f"        pathfinder -> connected={p_conn}/2, shorts={p_short} "
          f"(converged in {p_iters} iters)")
    ok = (p_conn == 2 and p_short == 0 and g_short > 0)
    print()
    if ok:
        print("PASS: negotiation eliminated the short the forbid-wall router was forced into.")
        print("      This is the §6 claim in miniature: pricing contention > forbidding it.")
    else:
        print("UNEXPECTED: instance did not exhibit the contrast; tune the gadget.")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
