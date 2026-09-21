"""平曲线超高路段横向稳定性验算法（JTG/T 2213—2023 附录 B.2）。

实现侧滑/侧翻临界速度与停车超高验算：
  B.2.1   V ≤ V_hmax
  B.2.2   V_hmax = min(Vch, Vcv)
              Vch = √( g·R·(μ+i_h) / (1 − μ·i_h) )   —— 侧滑临界速度（km/h）
              Vcv = √( 127·R·(b+2·h·i_h) / (2·h − b·i_h) ) —— 侧翻临界速度（km/h）
  B.2.3   i_h < b / (2·h)   —— 停车横向稳定性

式中 R 取最内侧转弯半径；i_h 为超高值；b 为轮距；h 为车货总体重心高度；
μ 为横向力系数（此处以衬垫摩擦系数作为保守估算来源）。
"""
from __future__ import annotations
import math

from config import g as _G


def side_slip_critical_v(r: float, mu: float, ih: float) -> float:
    """侧滑临界速度 Vch（km/h），B.2.2。r 为最内侧转弯半径(m)。"""
    denom = 1.0 - mu * ih
    if denom <= 1e-9 or (mu + ih) <= 0.0:
        return float("inf")
    return 3.6 * math.sqrt(_G * r * (mu + ih) / denom)


def overturn_critical_v(r: float, b: float, h: float, ih: float) -> float:
    """侧翻临界速度 Vcv（km/h），B.2.2。b 轮距、h 重心高度、i_h 超高。"""
    denom = 2.0 * h - b * ih
    if denom <= 1e-9:
        return 0.0
    return math.sqrt(127.0 * r * (b + 2.0 * h * ih) / denom)


def max_stable_v(r: float, mu: float, b: float, h: float, ih: float) -> float:
    """B.2.1 满足横向行驶稳定性的最大速度 = min(Vch, Vcv)。"""
    return min(side_slip_critical_v(r, mu, ih), overturn_critical_v(r, b, h, ih))


def parking_superelevation_ok(b: float, h: float, ih: float) -> bool:
    """B.2.3 停车横向稳定性：i_h < b/(2h)。"""
    if h <= 1e-9:
        return True
    return ih < b / (2.0 * h)


def lateral_stability(
        r: float, v_kmh: float, truck_width: float, cg_height: float,
        superelevation: float, friction_mu: float):
    """封装横向稳定性判定。

    参数：r 最内侧转弯半径(m)；v_kmh 行驶速度；truck_width 轮距（或两侧轮距）;
          cg_height 车货总体重心高度(m)；superelevation 超高 i_h；
          friction_mu 横向力系数 μ。
    返回 (pass_ok, V_hmax, Vch, Vcv, parking_ok, msg)。
    """
    vch = side_slip_critical_v(r, friction_mu, superelevation)
    vcv = overturn_critical_v(r, truck_width, cg_height, superelevation)
    vh = min(vch, vcv)
    p_ok = parking_superelevation_ok(truck_width, cg_height, superelevation)
    pass_ok = v_kmh <= vh + 1e-9 and p_ok
    msg = (f"侧滑临界速度 Vch = {vch:.3f} km/h；侧翻临界速度 Vcv = {vcv:.3f} km/h；"
           f"最大稳定速度 Vhmax = {vh:.3f} km/h。")
    if not p_ok:
        msg += " 停车超高不满足 B.2.3（i_h 需 < 轮距/(2×重心高)）。"
    caption = "横向稳定性：通过" if (v_kmh <= vh + 1e-9 and p_ok) else "横向稳定性：不通过"
    return pass_ok, vh, vch, vcv, p_ok, msg + " " + caption