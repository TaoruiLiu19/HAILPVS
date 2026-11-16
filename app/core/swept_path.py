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


def measure_steady_turn(kin, turn_steps, v=1.5, dt=0.05, center=(0, 0),
                        settle_frac=0.5):
    """在固定转向角下以 center 为转动圆心转圈，返回**稳态** (R_o, R_i)。

    center 为车辆转动圆心；从该圆心量取整车所有包络点的径向距离，
    最大=R_o(扫空外径)、最小=R_i(内径)。

    只统计后段（(1−settle_frac) 的行程）：挂车从"直线拖尾"初始姿态进入定常
    圆周时有一段过渡期，过渡期内车尾仍在入场直道上（离圆心比稳态更远），
    若把它并入统计，R_o 就被**入场姿态**而非稳态扫空带定义，且量值随"仿真
    从哪一点起算"漂移（实测 15 m 车板/12 m 货、R=22 m：含过渡 R_o=27.68、
    稳态 23.40，扫空宽度虚高 48%；起点再后退 6 m 会长到 31.3）。故量测只取
    稳态段——与"转弯通道宽度"的定义（定常圆周转弯时扫空环带的宽度）一致。
    """
    from itertools import repeat

    delta = kin.delta if kin.delta else kin.max_delta * 0.7
    cx, cy = center
    kin.v = v
    rmax, rmin = 0.0, 1e18
    settle = int(max(0.0, min(settle_frac, 0.9)) * turn_steps)
    for i, _ in enumerate(repeat(None, turn_steps)):
        kin.update_step(delta, dt)
        if i < settle:
            continue
        pts = []
        kin.footprint_poly(pts)
        for px, py in pts:
            r = math.hypot(px - cx, py - cy)
            if r > rmax:
                rmax = r
            if r < rmin:
                rmin = r
    if rmin > 1e17:                     # 窗口内无采样（turn_steps 过小）
        return measure_steady_turn(kin, max(turn_steps, 8), v=v, dt=dt,
                                   center=center, settle_frac=0.0)
    return rmax, rmin