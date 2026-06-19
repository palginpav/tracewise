"""tracewise.route.gridless — FAR gridless router (M1 Phase 1).

Public API
----------
route_net_gridless
    Route a 2-pin net via a visibility-graph A* in Minkowski-inflated free space.
GridlessRouteResult
    Lightweight result dataclass (world_paths, ok, stats, reason).
HAVE_SHAPELY
    True if shapely>=2.0 / GEOS>=3.8.0 is installed; False otherwise.

Phase 2 will add the GridlessNetRoute → NetRoute adapter and engine wiring.
"""

from tracewise.route.gridless.geom import HAVE_SHAPELY
from tracewise.route.gridless.route import GridlessRouteResult, route_net_gridless

__all__ = ["route_net_gridless", "GridlessRouteResult", "HAVE_SHAPELY"]
