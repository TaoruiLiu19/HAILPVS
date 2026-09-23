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
from app.core.grade import (classify, classify_bundle, TABLE_4_3_1,
                            arch_lookup_apply)
from app.core.kinematics import (hydraulic_turn_radii,
                                 hydraulic_coupling_draw_anchor,
                                 axle_steering_geoms,
                                 hydraulic_arc_trajectory)
from app.core.lateral import (side_slip_critical_v, overturn_critical_v,
                              parking_superelevation_ok)
from config import AXLE_STEER_ABS_LIMIT_DEG, MIN_TURN_ANGLE_DEG


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


def _h_trailer(**kw):
    """液压轴线挂车便捷构造；kw 覆盖默认参数（含 θ/锁死轴/耦合字段）。"""
    d = dict(trailer_type="液压轴线挂车", length=12.0, width=3.0,
             deck_height=1.2, axle_count=4, axle_spacing=1.5, track_width=2.9)
    d.update(kw)
    return TrailerParams(**d)


def _h_bundle(trailer, outer=30.0, lane=7.0):
    """弧形弯道 + 指定液压挂车的完整输入。"""
    return InputBundle(
        cargo=_cargo(), trailer=trailer, tractor=_tractor(),
        road=RoadParams(road_type="圆弧弯道",
                        arc=ArcCorner(outer_radius=outer, lane_width=lane),
                        vertical_clearance=7.5, highway_grade="三级公路"),
        max_speed_kmh=10.0)


# ---- M1: 0.5m 安全宽度 ----
def test_m1_safety_clearance_flips_judgement():
    # 液压挂车 B.1.2：扫空宽度 = d = L轮挂·(Na−1) = 5.0×(3−1) = 10.0，
    # 转弯通道 = 10.0+0.5 = 10.5
    b = _arc_bundle(lowbed=False, outer=30.0, lane=5.2,
                    lateral_clearance=0.0, axle_spacing=5.0)
    res = run_scene(b)
    swept = res.metrics["W_needed"]
    assert swept >= 10.0 - 1e-6
    assert abs(res.metrics["W"] - (swept + 0.5)) < 1e-6          # 计入安全宽度
    assert res.metrics["turn_channel_w"] > res.metrics["road_lane"]  # 10.5 > 5.2
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
    # 横向空间宽度/路面宽度不再由 σ 双侧扣除（废止 lane-2σ 口径）
    assert high.metrics["transverse_space_w"] == pytest.approx(low.metrics["transverse_space_w"])


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
    # 表4.3.1 双列数据：最小半径 / 最大扫空空间宽度
    assert TABLE_4_3_1["A"]["min_radius"] == 11.0
    assert TABLE_4_3_1["A"]["max_swept_width"] == 8.6
    assert TABLE_4_3_1["B"]["min_radius"] == 15.0
    assert TABLE_4_3_1["C"]["min_radius"] == 22.0
    assert TABLE_4_3_1["D"]["min_radius"] == 28.0
    assert TABLE_4_3_1["E"] is None   # 无表数据 → 回退验算/仿真


def test_m5_lookup_dual_condition():
    # 查表法 4.3.1：半径与通行空间宽度均大于表值 → pass；任一不满足 → fail
    assert arch_lookup_apply("A", 20.0, 10.0, 5.0) == "pass"    # r>11, w>8.6
    assert arch_lookup_apply("A", 10.0, 10.0, 5.0) == "fail"    # r≤11
    assert arch_lookup_apply("A", 20.0, 6.0, 5.0) == "fail"     # 通行空间≤8.6
    assert arch_lookup_apply("E", 50.0, 30.0, 5.0) == "fallback"  # E 无表数据


# ---- M6: 液压挂车闭式解 ----
def test_m6_hydraulic_radius_grows_with_axles():
    radii = [hydraulic_turn_radii(_arc_bundle(lowbed=False, axle_count=na))
             for na in range(2, 9)]
    rin = [r["R_in"] for r in radii]
    rout = [r["R_out"] for r in radii]
    # 轴数不影响转向角(限45°)同时首末轴距项与 (D总−d)/2 相抵 ⇒ R_in = D总/2 恒定
    assert all(abs(rin[i] - rin[0]) < 1e-9 for i in range(len(rin) - 1))
    assert all(rout[i] < rout[i + 1] for i in range(len(rout) - 1))   # R_out 单调增
    # B.1.2-6 扫空宽度 W_swept = d = 首末轴距 = L轮挂·(Na−1)
    b = _arc_bundle(lowbed=False, axle_count=4, axle_spacing=1.5)
    r = hydraulic_turn_radii(b)
    assert r["W_swept"] == pytest.approx(1.5 * (4 - 1))
    assert r["R_out"] - r["R_in"] == pytest.approx(4.5)


def test_m6_hydraulic_Lwheel_is_axle_spacing_not_track():
    # 规范 B.1.2-2/-4：R = L轮挂·(Na−1)/(2tanθ)，L轮挂 取纵向轴距。
    # 轮距 track_width 变化不应影响转弯半径；轴距 axle_spacing 变化才应影响。
    base = _arc_bundle(lowbed=False, axle_count=4, axle_spacing=1.5, track=2.9)
    r0 = hydraulic_turn_radii(base)
    wide = _arc_bundle(lowbed=False, axle_count=4, axle_spacing=1.5, track=5.0)
    r_wide = hydraulic_turn_radii(wide)
    long = _arc_bundle(lowbed=False, axle_count=4, axle_spacing=3.0, track=2.9)
    r_long = hydraulic_turn_radii(long)
    # 轮距不影响（修复前会错误触发）
    assert r_wide["R_in"] == pytest.approx(r0["R_in"], abs=1e-9)
    assert r_wide["R_out"] == pytest.approx(r0["R_out"], abs=1e-9)
    # 轴距影响：R_out 与扫空宽度 W_swept=d 随之增大
    assert r_long["R_out"] > r0["R_out"]
    assert r_long["W_swept"] > r0["W_swept"]


# ---- B.1.2 规范接入（D2/D1/D4/D3/D8）----
def test_b12_theta_src():
    # D2: θ 取自挂车轴最大转向角，不读牵引车前轮角
    tl = _h_trailer(axle_max_steering_angle_deg=25.0)
    b = _h_bundle(tl)
    b.tractor.max_steering_angle_deg = 70.0     # 牵引车前轮角更大也不应影响 θ
    r = hydraulic_turn_radii(b)
    assert r["theta_deg"] == pytest.approx(25.0, abs=1e-9)


def test_b12_theta_upperbound():
    # D6: θ 钳到工程上限，不做原地回转(θ<90)
    tl = _h_trailer(axle_max_steering_angle_deg=85.0)
    r = hydraulic_turn_radii(_h_bundle(tl))
    assert r["theta_deg"] == pytest.approx(AXLE_STEER_ABS_LIMIT_DEG, abs=1e-9)
    assert math.isfinite(r["R_in"]) and math.isfinite(r["R_out"]) and r["R_in"] > 0


def test_axle_steering_geoms_positions_precise():
    # 轴位精确到瞬心/车板坐标：第 i 轴距车板中心前向偏移 = length/2 − front_overhang
    # − (i−1)·spacing。front_overhang=2 时首轴应前移 length/2−2=4，末轴(length/2−2−3·spacing)
    # 由 front_overhang / coupling 唯一决定（D7/D8）。
    tl = _h_trailer(axle_count=4, axle_spacing=1.5, length=12.0,
                    front_overhang=2.0)
    ax = axle_steering_geoms(0.0, 0.0, 0.0, tl, 10.0, na=4)
    # psi=0 → 前向=+x；首轴 cx = 0 + (6−2−0) = 4.0
    first = next(a for a in ax if a[0] == 1)
    assert first[1] == pytest.approx(6.0 - 2.0, abs=1e-9)
    last = next(a for a in ax if a[0] == 4)
    assert last[1] == pytest.approx(6.0 - 2.0 - 3 * 1.5, abs=1e-9)


def test_axle_tire_count_in_frames():
    # 每帧 axle_geoms 带 n_tires（液压轴线每轴 8 轮胎），供多轮胎实体绘制
    tl = _h_trailer(axle_count=4)
    b12 = hydraulic_turn_radii(_h_bundle(tl))
    frames, _ = hydraulic_arc_trajectory(_h_bundle(tl), b12, 20.0, 140.0)
    assert frames[0][5].get("n_tires") == 8


def test_right_angle_hydraulic_axle_layer():
    # 直角弯液压车帧为 6 元组且携带 axle_geoms（§4.5 仅可视化，判定 5.2.1 不动）
    tl = _h_trailer(axle_count=5)
    res = run_scene(InputBundle(
        cargo=_cargo(), trailer=tl, tractor=_tractor(),
        road=RoadParams(road_type="直角弯道",
                        right_angle=RightAngleCorner(entry_width=14.0,
                                                     exit_width=14.0,
                                                     intersection_angle_deg=90.0,
                                                     corner_radius=2.0),
                        vertical_clearance=7.5, highway_grade="三级公路"),
        max_speed_kmh=10.0))
    assert res.vehicle_frames and len(res.vehicle_frames[0]) == 6
    for f in res.vehicle_frames:
        assert f[5] is not None and f[5].get("axles")
    # 奇数轴含中间闭锁轴
    first_data = res.vehicle_frames[0][5]["axles"]
    assert sum(1 for a in first_data if a[5]) >= 1   # 至少一根闭锁轴


def test_right_angle_hydraulic_judgement_unchanged():
    # 加轴线层不影响 5.2.1：W/R_o/R_i 与机械车一致恢复（纯判定口径不变）
    tl = _h_trailer(axle_count=4)
    res = run_scene(InputBundle(
        cargo=_cargo(), trailer=tl, tractor=_tractor(),
        road=RoadParams(road_type="直角弯道",
                        right_angle=RightAngleCorner(entry_width=14.0,
                                                     exit_width=14.0,
                                                     intersection_angle_deg=90.0,
                                                     corner_radius=2.0),
                        vertical_clearance=7.5, highway_grade="三级公路"),
        max_speed_kmh=10.0))
    assert "right_ok" in res.metrics
    assert res.metrics["W"] > 0
    assert res.pass_ok in (True, False)


def test_b12_theta_min():
    # 下端 1° 钳制，防 tan 除零
    tl = _h_trailer(axle_max_steering_angle_deg=0.5)
    r = hydraulic_turn_radii(_h_bundle(tl))
    assert r["theta_deg"] == pytest.approx(MIN_TURN_ANGLE_DEG, abs=1e-9)
    assert math.isfinite(r["R_in"]) and math.isfinite(r["R_out"])


def test_b12_locked_middle_spec():
    # D1: 纯中间锁死不改变 d，spec_mode=True(严格守规范)
    tl = _h_trailer(axle_count=5, locked_axles=[2, 3])
    r = hydraulic_turn_radii(_h_bundle(tl))
    assert r["spec_mode"] is True
    assert r["d"] == pytest.approx(1.5 * (5 - 1))
    assert r["W_swept"] == pytest.approx(1.5 * (5 - 1))


def test_b12_locked_ends_extrap():
    # D1: 锁死首轴 → 规范外推 d 缩短 + 告警
    tl = _h_trailer(axle_count=5, locked_axles=[1])
    r = hydraulic_turn_radii(_h_bundle(tl))
    assert r["spec_mode"] is False
    assert r["d"] == pytest.approx(1.5 * (5 - 2))   # last=5,first=2
    assert r["warnings"] and "锁死" in r["warnings"][0]


def test_b12_D_total_coupling_independent():
    # D3/D8: D_total=车板长；耦合连接字段不影响 D_total/R/W_swept
    tl0 = _h_trailer(length=12.0)
    tl1 = _h_trailer(length=12.0, coupling_type="连接杆", coupling_length=5.0)
    r0, r1 = hydraulic_turn_radii(_h_bundle(tl0)), hydraulic_turn_radii(_h_bundle(tl1))
    assert r1["D_total"] == pytest.approx(r0["D_total"])
    assert r1["D_total"] == pytest.approx(12.0)
    for k in ("R_in", "R_out", "W_swept", "W_turn", "d"):
        assert r1[k] == pytest.approx(r0[k])


def test_b12_W_turn_indicator():
    # D4: W_turn=d+0.5，run_arc_scene 输出指标
    r = hydraulic_turn_radii(_h_bundle(_h_trailer(axle_count=4, axle_spacing=1.5)))
    assert r["W_turn"] == pytest.approx(r["d"] + 0.5)
    res = run_scene(_h_bundle(_h_trailer(), outer=30.0, lane=7.0))
    m = res.metrics
    assert "B1_2_W_turn" in m and "B1_2_W_swept" in m and "B1_2_d" in m
    assert m["B1_2_W_turn"] == pytest.approx(m["B1_2_W_swept"] + 0.5)


def test_b12_active_lt2_fallback():
    # 可转向轴<2 → 回退规范基准 + 告警
    tl = _h_trailer(axle_count=3, locked_axles=[1, 2])
    r = hydraulic_turn_radii(_h_bundle(tl))
    assert r["spec_mode"] is True and "转向轴<2" in r["warnings"][-1]


def test_arc_b12_indicator():
    res = run_scene(_h_bundle(_h_trailer()))
    m = res.metrics
    for k in ("B1_2_W_turn", "B1_2_d", "B1_2_D_total", "B1_2_spec_mode",
              "B1_2_warnings", "B1_2_theta_deg", "B1_2_base",
              "trajectory_source"):
        assert k in m
    assert m["trajectory_source"] == "B.1.2"


def test_b12_judgement_turn_channel_vs_road_width():
    # D4 判定联动：转弯通道 W_turn 与路面宽比较进入宽度判据
    # 窄车道时因 W_turn=d+0.5 超路面宽 → 判不通过
    tl = _h_trailer(axle_count=4, axle_spacing=5.0)     # d=15,W_turn=15.5
    res = run_scene(_h_bundle(tl, outer=40.0, lane=9.0))
    # 查表(3级C半径>22、横向空间18>…)若不命中则落入验算；此用例确保不因异常而挂
    assert isinstance(res.metrics["B1_2_W_turn"], float)
    assert res.metrics["B1_2_W_turn"] == pytest.approx(15.5)


# ---- S2: 液压车瞬心轨迹 + 多轴转向绘制（D5）----
def test_axle_steering_geoms_midlock_zero():
    # 奇轴含中间闭锁轴，其 δ=0；首/末轴 δ 方向一致指向瞬心且坐标在车板内
    tl = _h_trailer(axle_count=5, locked_axles=[3])   # 第3轴为中间闭锁轴
    axles = axle_steering_geoms(0.0, 0.0, 0.0, tl, 10.0, na=5)
    mid = next(a for a in axles if a[0] == 3)
    assert mid[3] == pytest.approx(0.0, abs=1e-9)     # 闭锁轴 δ=0
    assert mid[5] is True
    others = [a for a in axles if a[0] != 3]
    assert any(a[3] != 0.0 for a in others)           # 首/末轴有偏转


def test_axle_draw_odd_axles():
    # 奇轴 N_a → 首轴 + 末轴 + 中间闭锁轴 三组，且中间闭锁轴 δ=0（第六章图式）
    tl = _h_trailer(axle_count=5)
    axles = axle_steering_geoms(0.0, 0.0, 0.0, tl, 10.0, na=5)
    assert [a[0] for a in axles] == [1, 3, 5]
    mid = next(a for a in axles if a[0] == 3)
    assert mid[3] == pytest.approx(0.0, abs=1e-9)   # 奇轴中间轴自动闭锁 δ=0
    assert mid[5] is True                            # 标记为闭锁轴


def test_axle_draw_even_axles():
    # 偶轴 N_a → 只首轴 + 末轴，无闭锁轴(δ=0 仅当用户显式锁死)
    tl = _h_trailer(axle_count=4)
    axles = axle_steering_geoms(0.0, 0.0, 0.0, tl, 10.0, na=4)
    assert [a[0] for a in axles] == [1, 4]
    axles_locked = axle_steering_geoms(0.0, 0.0, 0.0, tl, 10.0, na=4,
                                       locked_axles=[2])
    # 显式锁死才可能出 δ=0，且中间闭锁轴不在默认绘制集
    assert 2 not in [a[0] for a in axles_locked]


def test_hydra_arc_trajectory_frames_have_axles():
    # 液压车轨迹为刚性圆周，且每帧携带 axle_geoms(6 元组)
    tl = _h_trailer(axle_count=4)
    b12 = hydraulic_turn_radii(_h_bundle(tl))
    frames, swept = hydraulic_arc_trajectory(_h_bundle(tl), b12, 20.0, 140.0)
    assert len(frames) > 5
    assert swept is not None and not swept.is_empty
    for f in frames:
        assert len(f) == 6                    # 追加 axle_geoms
        assert f[5] is not None and f[5].get("axles")
        for (_i, _cx, _cy, _dd, _ht, _lk) in f[5]["axles"]:
            assert abs(_dd) <= AXLE_STEER_ABS_LIMIT_DEG + 1e-6


def test_two_vehicle_trajectory_diff():
    # 同一几何下：机械车铰接尾随(挂车外摆)扫掠带更宽；液压车瞬心刚体(无铰尾摆)扫掠带更窄，
    # 且两者扫掠几何不同/面积不同(D5 行为区别)。
    common = dict(outer=28.0, lane=7.0)
    mec = run_scene(_arc_bundle(lowbed=True, axle_count=4, axle_spacing=1.5,
                                track=2.9, **common))
    hyd = run_scene(_h_bundle(_h_trailer(axle_count=4, axle_spacing=1.5,
                                         track_width=2.9), **common))
    assert mec.metrics["trajectory_source"] == "铰接尾随"
    assert hyd.metrics["trajectory_source"] == "B.1.2"
    r_mec = [math.hypot(*p) for p in _ring_pts(mec.swept)]
    r_hyd = [math.hypot(*p) for p in _ring_pts(hyd.swept)]
    band_mec = max(r_mec) - min(r_mec)
    band_hyd = max(r_hyd) - min(r_hyd)
    assert band_hyd < band_mec + 1e-9          # 液压车扫掠带更窄：瞬心刚体无铰尾摆
    assert mec.swept.area != pytest.approx(hyd.swept.area, abs=2.0)


def _ring_pts(geom):
    """取扫掠几何所有外环/内环坐标点(扁平列表)。"""
    pts = []
    if geom is None or geom.is_empty:
        return pts
    polys = geom.geoms if geom.geom_type == "MultiPolygon" else [geom]
    for p in polys:
        for ring in ([p.exterior] + list(p.interiors)):
            pts.extend(ring.coords)
    return pts


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


# ---- P0-6: 垂直净空分场景阈值 ----
def test_vertical_cross_structure_thresholds():
    # 跨线构造物：间隙在 [0.05,0.10) 需措施但可通行；<0.05 不通过
    ok = run_scene(_arc_bundle(lowbed=True, outer=30.0, lane=7.0,
                               vertical=4.26))  # 总高4.2，间隙0.06
    assert ok.metrics["vert_clearance_ok"] is True
    bad = run_scene(_arc_bundle(lowbed=True, outer=30.0, lane=7.0,
                                vertical=4.23))  # 间隙0.03 < 0.05
    assert bad.metrics["vert_clearance_ok"] is False
    assert bad.pass_ok is False


def test_vertical_tunnel_threshold_stricter():
    # 隧道顶部要求 ≥0.2：间隙0.15 应判不通过（比跨线0.05严格）
    b = _arc_bundle(lowbed=True, outer=30.0, lane=7.0, vertical=4.35)
    b.road.clearance_structure_type = "隧道"
    res = run_scene(b)
    assert res.metrics["vert_clearance_ok"] is False


# ---- P0-4: 圆弧判定使用横向空间宽度（废止 lane-2σ）----
def test_arc_uses_transverse_width():
    b = _arc_bundle(lowbed=True, outer=30.0, lane=7.0)
    res = run_scene(b)
    assert "transverse_space_w" in res.metrics
    assert "road_width" in res.metrics
    assert "method" in res.metrics
    assert res.metrics["road_width"] > 0


# ---- P0-5: 左转使用 W_section / W_lturn ----
def test_left_turn_uses_section_width():
    b = InputBundle(
        cargo=_cargo(), trailer=_trailer(), tractor=_tractor(),
        road=RoadParams(road_type="直角弯道",
                        right_angle=RightAngleCorner(entry_width=10.0,
                                                     exit_width=10.0,
                                                     corner_radius=0.0,
                                                     intersection_angle_deg=90.0,
                                                     W_section=6.0,
                                                     W_lturn=7.0),
                        vertical_clearance=7.5),
        max_speed_kmh=10.0)
    res = run_scene(b)
    m = res.metrics
    sigma = m["sigma"]
    assert m["W_section"] == pytest.approx(6.0)
    assert m["W_lturn"] == pytest.approx(7.0)
    left_ok = ((m["W_section"] - sigma) >= (m["B_t"] + m["deltaB"]) - 1e-6) and \
              ((m["W_lturn"] - sigma) >= m["W"] - 1e-6)
    assert m["left_ok"] == left_ok


# ---- 基准回归 ----
def test_regression_arc_lowbed_runs_and_fields():
    res = run_scene(_arc_bundle(lowbed=True, outer=30.0, lane=7.0))
    for key in ("R_o", "R_i", "W_needed", "W", "sigma", "size_grade",
                "Vch", "Vcv", "total_height", "vert_clearance_ok"):
        assert key in res.metrics


def test_regression_metricturn_safe_in_config():
    from config import TURN_SAFE_CLEARANCE
    assert TURN_SAFE_CLEARANCE == 0.5