"""几何包络线生成：把仿真帧多边形求并，得到扫掠区域。

对圆弧弯道，还提供"稳态圆转"的径向内外半径测量（R_o / R_i）。
"""
from __future__ import annotations
import math

from shapely.ops import unary_union


def polylist_to_multipoly(frames_geoms, simplify_tol=0.02):
    """把 [{x,y,geom}...] 逐帧滚动求并后合并。

    geom 为有效 Polygon/MultiPolygon（由 build_trajectory_geoms 保证）。
    """
    polys = []
    chunk = []
    for _x, _y, geom, *_rest in frames_geoms:
        if geom is None or geom.is_empty:
            continue
        chunk.append(geom)
        if len(chunk) >= 40:
            polys.append(unary_union(chunk))
            chunk = []
    if chunk:
        polys.append(unary_union(chunk))
    if not polys:
        return None
    merged = unary_union(polys)
    if simplify_tol and merged is not None and merged.geom_type in (
            "Polygon", "MultiPolygon") and hasattr(merged, "simplify"):
        merged = merged.simplify(simplify_tol)
    return merged


def measure_steady_turn(kin, turn_steps, v=1.5, dt=0.05, center=(0, 0)):
    """在固定转向角下以 center 为转动圆心转圈，返回 (R_o, R_i)。

    center 为车辆转动圆心；从该圆心量取整车所有包络点的径向距离，
    最大=R_o(扫空外径)、最小=R_i(内径)。
    """
    from itertools import repeat

    delta = kin.delta if kin.delta else kin.max_delta * 0.7
    cx, cy = center
    kin.v = v
    rmax, rmin = 0.0, 1e18
    for _ in repeat(None, turn_steps):
        kin.update_step(delta, dt)
        pts = []
        kin.footprint_poly(pts)
        for px, py in pts:
            r = math.hypot(px - cx, py - cy)
            if r > rmax:
                rmax = r
            if r < rmin:
                rmin = r
    return rmax, rmin