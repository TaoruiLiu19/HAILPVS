"""回归与整治护栏（M10）。

覆盖修改方案 M1–M8 的关键判据：
  M1 安全宽度 0.5m 翻转判定；M2 内缘/预留净空；M3 公路等级→σ；
  M4 直角弯 R_m/W/ΔB 自洽；M5 分级；M6 液压挂车闭式解；M7 垂直净空；M8 横向稳定。
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import pytest

from app.core.datamodel import (InputBundle, CargoParams, TrailerParams,
                                TractorParams, RoadParams, ArcCorner,
                                RightAngleCorner)
from app.core.passability import run_scene
from app.core.grade import classify, classify_bundle, TABLE_4_3_1_RMIN
from app.core.kinematics import hydraulic_turn_radii
from app.core.lateral import (side_slip_critical_v, overturn_critical_v,
                              parking_superelevation_ok)


def _tractor():
    return TractorParams(wheelbase=4.0, max_steering_angle_deg=45,
                         length=7.0, width=2.5, height=3.0, fifth_wheel_pos=1.0)


def _cargo():
    return CargoParams(weight_t=60, length=10, width=3.0, height=3.0,
                       cg_height=1.2, cargo_bearing="干燥的木板")


def _trailer(lowbed=True, axle_count=3, axle_spacing=1.5, track=2.9,
             length=12.0, width=3.0):
    return TrailerParams(
        trailer_type="重型机械挂车" if lowbed else "液压轴线挂车",
        length=length, width=width, deck_height=1.2,
        axle_count=axle_count, axle_spacing=axle_spacing,
        track_width=track)


def _arc_bundle(lowbed=True, outer=30.0, lane=7.0, grade="三级公路",
                lateral_clearance=None, vertical=7.5, **kw):
    return InputBundle(
        cargo=_cargo(), trailer=_trailer(lowbed=lowbed, **kw),
        tractor=_tractor(),
        road=RoadParams(road_type="圆弧弯道",
                        arc=ArcCorner(outer_radius=outer, lane_width=lane),
                        vertical_clearance=vertical, highway_grade=grade,
                        lateral_clearance=lateral_clearance),
        max_speed_kmh=10.0, load_factor=1.0)


# ---- M1: 0.5m 安全宽度 ----
def test_m1_safety_clearance_flips_judgement():
    # 液压挂车 B.1.2 使扫空宽度 = 轴距 d = 5.0，转弯通道 = 5.0+0.5 = 5.5
    b = _arc_bundle(lowbed=False, outer=30.0, lane=5.2,
                    lateral_clearance=0.0, axle_spacing=5.0)
    res = run_scene(b)
    swept = res.metrics["W_needed"]
    assert swept >= 5.0 - 1e-6
    assert abs(res.metrics["W"] - (swept + 0.5)) < 1e-6          # 计入安全宽度
    assert res.metrics["turn_channel_w"] > res.metrics["road_lane"]  # 5.5 > 5.2
    assert res.pass_ok is False                                    # 判定为不通过


def test_m1_same_vehicle_passes_with_wider_lane():
    b = _arc_bundle(lowbed=False, outer=40.0, lane=9.0,
                    lateral_clearance=0.0, axle_spacing=5.0)
    res = run_scene(b)
    assert res.pass_ok is True


# ---- M2/M3: 内缘约束与横向预留 ----
def test_m2_inner_clearance_rejected():
    # 窄车道 → 内缘净空不足以容纳预留 σ → 不通过
    b = _arc_bundle(lowbed=True, outer=22.0, lane=3.5, grade="三级公路")
    res = run_scene(b)
    assert res.pass_ok is False
    assert res.metrics["clearance_inner"] < res.metrics["sigma"] - 1e-6


def test_m3_highway_grade_stricter():
    high = run_scene(_arc_bundle(lowbed=True, outer=30.0, lane=7.0,
                                 grade="高速公路"))
    low = run_scene(_arc_bundle(lowbed=True, outer=30.0, lane=7.0,
                                grade="三级公路"))
    assert low.metrics["sigma"] == pytest.approx(0.25)
    assert high.metrics["sigma"] == pytest.approx(0.5)
    assert high.metrics["lane_avail"] < low.metrics["lane_avail"]


def test_m3_explicit_sigma_overrides_grade():
    b = _arc_bundle(lowbed=True, outer=30.0, lane=7.0, grade="高速公路",
                    lateral_clearance=0.1)
    res = run_scene(b)
    assert res.metrics["sigma"] == pytest.approx(0.1)


# ---- M4: 直角弯 R_m/W/ΔB 自洽 ----
def test_m4_right_angle_self_consistent():
    b = InputBundle(
        cargo=_cargo(), trailer=_trailer(), tractor=_tractor(),
        road=RoadParams(road_type="直角弯道",
                        right_angle=RightAngleCorner(entry_width=10.0,
                                                     exit_width=10.0,
                                                     corner_radius=0.0,
                                                     intersection_angle_deg=90.0),
                        vertical_clearance=7.5),
        max_speed_kmh=10.0)
    res = run_scene(b)
    m = res.metrics
    sigma = m["sigma"]
    assert m["R_m"] > 0 and m["W"] > 0
    assert m["deltaB"] >= 0
    # 外摆值 ΔB = Dmax − (R_m + B_t) → ΔB = W − B_t
    assert m["deltaB"] == pytest.approx(max(0.0, m["W"] - m["B_t"]), abs=1e-6)
    right_ok = ((m["entry_width"] - 2 * sigma) >= (m["B_t"] + m["deltaB"]) - 1e-6) \
        and ((m["exit_width"] - 2 * sigma) >= m["W"] - 1e-6)
    assert res.pass_ok == right_ok
    assert ("外摆值" in "\n".join(res.verdict_lines)) and ("ΔB" in "\n".join(res.verdict_lines))


def test_m4_right_angle_too_narrow_rejected():
    b = InputBundle(
        cargo=_cargo(), trailer=_trailer(), tractor=_tractor(),
        road=RoadParams(road_type="直角弯道",
                        right_angle=RightAngleCorner(entry_width=1.0,
                                                     exit_width=1.0,
                                                     corner_radius=0.0,
                                                     intersection_angle_deg=90.0),
                        vertical_clearance=7.5),
        max_speed_kmh=10.0)
    res = run_scene(b)
    assert res.pass_ok is False


# ---- M5: 分级 ----
def test_m5_classify_bounds():
    # 宽度/长度/高度各区间 → 级（高度 A/B/C 共区 (4.00,4.50]，取最高）
    assert classify(10, 2.8, 3.9, False) == "A"
    assert classify(10, 2.8, 4.2, False) == "C"   # 高度 4.2 → A/B/C 取最高 C
    assert classify(20, 3.2, 4.2, False) == "C"   # 长 B、宽 B、高 C → C
    assert classify(26, 3.6, 4.4, False) == "C"   # 宽 C
    assert classify(33, 4.0, 4.8, False) == "D"
    assert classify(40, 5.0, 5.5, False) == "E"
    # 取最高级
    assert classify(50, 2.8, 4.2, False) == "E"   # 长 E → E
    assert classify(10, 2.8, 5.5, False) == "E"   # 高 E → E


def test_m5_table_radius():
    assert TABLE_4_3_1_RMIN["A"] == 8.6
    assert TABLE_4_3_1_RMIN["E"] is None   # 无表数据 → 回退验算/仿真


# ---- M6: 液压挂车闭式解 ----
def test_m6_hydraulic_radius_grows_with_axles():
    radii = [hydraulic_turn_radii(_arc_bundle(lowbed=False, axle_count=na))
             for na in range(2, 9)]
    rin = [r["R_in"] for r in radii]
    rout = [r["R_out"] for r in radii]
    assert all(rin[i] < rin[i + 1] for i in range(len(rin) - 1))    # 单调不减
    assert all(rout[i] < rout[i + 1] for i in range(len(rout) - 1))
    # B.1.2-6 扫空宽度 = 轴距 d
    b = _arc_bundle(lowbed=False, axle_count=4, axle_spacing=1.5)
    assert hydraulic_turn_radii(b)["W"] == pytest.approx(1.5)


# ---- M7: 垂直净空 ----
def test_m7_vertical_clearance():
    ok = run_scene(_arc_bundle(lowbed=True, outer=30.0, lane=7.0, vertical=7.5))
    assert ok.metrics["vert_clearance_ok"] is True
    bad = run_scene(_arc_bundle(lowbed=True, outer=30.0, lane=7.0,
                                vertical=2.0))  # 总高 4.2 > 净空
    assert bad.metrics["vert_clearance_ok"] is False
    assert bad.pass_ok is False


# ---- M8: 横向稳定性 ----
def test_m8_lateral_stability_textbook():
    # 侧翻临界速度随重心升高而下降
    lo = overturn_critical_v(20, 2.9, 1.0, 0.02)
    hi = overturn_critical_v(20, 2.9, 3.0, 0.02)
    assert lo > hi
    # 侧滑临界速度随摩擦系数/超高增大而增大
    assert side_slip_critical_v(20, 0.4, 0.02) < \
        side_slip_critical_v(20, 0.7, 0.02)
    # 停车超高 B.2.3
    assert parking_superelevation_ok(2.9, 1.0, 0.02) is True
    assert parking_superelevation_ok(2.0, 2.0, 0.5) is False


def test_m8_scene_lateral_runs():
    res = run_scene(_arc_bundle(lowbed=True, outer=30.0, lane=7.0))
    assert "Vhmax" in res.metrics
    assert isinstance(res.metrics["ok"], bool)


# ---- 基准回归 ----
def test_regression_arc_lowbed_runs_and_fields():
    res = run_scene(_arc_bundle(lowbed=True, outer=30.0, lane=7.0))
    for key in ("R_o", "R_i", "W_needed", "W", "sigma", "size_grade",
                "Vch", "Vcv", "total_height", "vert_clearance_ok"):
        assert key in res.metrics


def test_regression_metricturn_safe_in_config():
    from config import TURN_SAFE_CLEARANCE
    assert TURN_SAFE_CLEARANCE == 0.5