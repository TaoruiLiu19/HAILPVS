"""横向空间宽度 / 路面宽度计算（JTG/T 2213—2023 4.2.2 / 4.3.1）。

替代旧代码中“路面净宽 = lane − 2σ”的错误口径：规范 4.3.1 第2款要求
  1) 转弯通道宽度 < 路面宽度；
  2) 扫空空间宽度 < 横向空间宽度（有圆曲线加宽时含加宽）。
横向空间宽度按 4.2.2 优先取同向路缘带、硬路肩、行车道之和，划线分隔时可
借用对向行车道。σ 仅用于外廓边缘与护栏等障碍物之间的预留，不再参与车道
宽度双侧扣除。
"""
from __future__ import annotations

from .datamodel import RoadParams


def road_width(road: RoadParams, lane: float) -> float:
    """路面宽度：行驶方向行车道 + 硬路肩。用于 4.3.1 转弯通道宽度比较。"""
    return lane + road.hard_shoulder_w


def transverse_space_width(road: RoadParams, lane: float) -> float:
    """横向空间宽度（4.2.2）。
    依次计入同向路缘带、硬路肩、行车道；划线分隔时借用对向行车道；
    有圆曲线加宽时计入加宽（4.3.1 第2款）。
    """
    w = lane
    w += road.left_edgeline_w          # 左侧路缘带
    w += road.hard_shoulder_w          # 硬路肩
    if road.median_type != "物理隔离":
        w += lane                      # 划线分隔可借用对向行车道
    if road.has_curve_widening:
        w += road.curve_widening_w     # 圆曲线加宽
    return w