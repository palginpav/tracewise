"""Analytical placement: gradient-descent optimization of footprint positions.

The Cypress-class approach (ISPD 2025) adapted to PCB: positions are continuous
variables, the cost is differentiable (smooth half-perimeter wirelength +
soft courtyard-overlap + boundary containment + decoupling proximity), and
Adam does the rest. No RL, no training — runs on CPU for hundreds of
components. Geometry in/out goes through pcbnew (extract.py / apply via the
same script pattern as the routing bridge).
"""
