"""tracewise.route.gridless — FAR gridless router (M1 Phase 1 + Phase 2 adapter + M2 negotiate).

Public API
----------
route_net_gridless
    Route a 2-pin net via a visibility-graph A* in Minkowski-inflated free space.
GridlessRouteResult
    Lightweight result dataclass (world_paths, ok, stats, reason).
GridlessNetRoute
    IS-A NetRoute adapter carrying world_paths + rasterized grid cells.
to_gridless_netroute
    Build a GridlessNetRoute from a routed result.
HAVE_SHAPELY
    True if shapely>=2.0 / GEOS>=3.8.0 is installed; False otherwise.
route_gridless_set
    M2: route a set of gridless nets with congestion negotiation + bounded rip-up.
GridlessSetNetResult
    M2: per-net result from route_gridless_set.
"""

from tracewise.route.gridless.adapter import GridlessNetRoute, to_gridless_netroute
from tracewise.route.gridless.geom import HAVE_SHAPELY
from tracewise.route.gridless.negotiate import GridlessSetNetResult, route_gridless_set
from tracewise.route.gridless.route import GridlessRouteResult, route_net_gridless

__all__ = [
    "route_net_gridless",
    "GridlessRouteResult",
    "GridlessNetRoute",
    "to_gridless_netroute",
    "HAVE_SHAPELY",
    "route_gridless_set",
    "GridlessSetNetResult",
]
