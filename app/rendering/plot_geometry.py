"""将道路和扫掠多边形转为保留内环的 Matplotlib 路径。"""

from matplotlib.patches import PathPatch
from matplotlib.path import Path
from shapely.geometry.polygon import orient


def polygon_patches(geometry, **style):
    if geometry is None or geometry.is_empty:
        return
    polygons = (geometry,) if geometry.geom_type == "Polygon" else getattr(geometry, "geoms", ())
    for polygon in polygons:
        if polygon.geom_type != "Polygon":
            continue
        polygon = orient(polygon)
        rings = [polygon.exterior, *polygon.interiors]
        vertices = [point for ring in rings for point in ring.coords]
        codes = [code for ring in rings for code in
                 [Path.MOVETO] + [Path.LINETO] * (len(ring.coords) - 2) + [Path.CLOSEPOLY]]
        yield PathPatch(Path(vertices, codes), **style)
