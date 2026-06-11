"""The TraceWise routing engine (see docs/ROUTER-DESIGN.md).

R0 milestone: occupancy grid + A* + path simplification — route a single net
on an empty board. Quality bar by construction: a path either fully connects
its endpoints or is reported failed; clearance is baked into the grid as
obstacle inflation before search begins.
"""
