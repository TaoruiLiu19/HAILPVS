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
from app.core.grade import (classify, TABLE_4_3_1, arch_lookup_apply)
from app.core.kinematics import (hydraulic_turn_radii,
                                 axle_steering_geoms,
                                 hydraulic_arc_trajectory,
                                 hydraulic_rigid_sweep_radii)
from app.core.lateral import (side_slip_critical_v, overturn_critical_v,
                              parking_superelevation_ok)
from config import (AXLE_STEER_ABS_LIMIT_DEG, MIN_TURN_ANGLE_DEG,
                    TURN_SAFE_CLEARANCE_B12)


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
def test_m1_safety_clearance_enters_width_criteria():
    # 液压挂车 B.1.2：扫空宽度 Wd = Rmax−Rmin+σ（B.1.2-6），转弯通道宽度
    # Wz = R外(轮迹)−R内(轮迹)+σ（B.1.2-1），二者都必须含 σ=0.5。
    # 构造工况使"扣掉 0.5 m 后即可通过路面宽度判据"——证明这 0.5 m 是
    # 决定宽度判据的余量（不是装饰项）。
    b = _arc_bundle(lowbed=False, outer=30.0, lane=5.2,
                    lateral_clearance=0.0, axle_spacing=7.5, axle_count=4)
    res = run_scene(b)
    m = res.metrics
    A = TURN_SAFE_CLEARANCE_B12
    assert m["B1_2_Wd"] == pytest.approx(m["W_needed"])               # 扫空宽度 = 判定 Wd
    assert m["W_needed"] == pytest.approx(
        m["B1_2_Rout"] - m["B1_2_Rin"] + A)                           # B.1.2-6 含 σ
    assert m["W"] == pytest.approx(
        m["B1_2_R_vehicle_out"] - m["B1_2_R_vehicle_in"] + A)         # B.1.2-1 含 σ
    assert m["W"] > m["road_width"] > m["W"] - A                      # 0.5 m 决定该判据
    assert m["B1_2_warnings"] == []                                  # 无 R_min 取负告警


def test_m1_same_vehicle_passes_with_wider_lane():
    b = _arc_bundle(lowbed=False, outer=40.0, lane=9.0,
                    lateral_clearance=0.0, axle_spacing=5.0)
    res = run_scene(b)
    assert res.pass_ok is True


# ---- D9/D10: θ 口径 / La / 仅板车刚体扫空（液压车口径整治）----
def test_b12_theta_from_actual_turn_radius():
    # D9: 给定实际弯道（车板中心走行）半径 → θ = atan(s/(2·(R−d/2)))，
    # base 为**内侧轮迹线**偏距，故 base = R − 轮距/2
    tl = _h_trailer(axle_count=4, axle_spacing=1.5)          # s = 4.5，轮距 2.9
    r = hydraulic_turn_radii(_h_bundle(tl), actual_turn_radius=26.5)
    assert r["theta_source"] == "turn_radius"
    # 返回字典中 theta_deg 按 4 位小数取整，故此处容差取 1e-4
    assert r["theta_deg"] == pytest.approx(
        math.degrees(math.atan(4.5 / (2 * (26.5 - 1.45)))), abs=1e-4)
    assert r["base"] == pytest.approx(26.5 - 2.9 / 2, abs=1e-9)   # base = R − d/2
    assert r["turn_radius"] == pytest.approx(26.5)
    # 车货总宽 3.0、轮距 2.9 → 内缘 = base − (3.0−2.9)/2，与外缘一起构成
    # "车体绕圆心刚体圆周"的初等几何（见函数文档的自洽性校验）
    assert r["R_in"] == pytest.approx(26.5 - 3.0 / 2, abs=1e-9)
    assert r["warnings"] == []


def test_b12_theta_falls_back_to_axle_max_without_radius():
    # 未给半径 → 回退结构最大转角（固有最不利口径），保持直接调用向后兼容
    tl = _h_trailer(axle_max_steering_angle_deg=25.0)
    r = hydraulic_turn_radii(_h_bundle(tl))
    assert r["theta_source"] == "axle_max"
    assert r["theta_deg"] == pytest.approx(25.0, abs=1e-9)


def test_b12_theta_clamped_to_axle_max():
    # 半径小到所需转角超过结构最大转角 → θ 钳到 θ_max，base 大于该半径
    tl = _h_trailer(axle_count=4, axle_spacing=1.5,
                    axle_max_steering_angle_deg=3.0)          # d = 4.5
    r = hydraulic_turn_radii(_h_bundle(tl), actual_turn_radius=26.5)
    assert r["theta_deg"] == pytest.approx(3.0, abs=1e-9)
    assert r["base"] > 26.5


def test_b12_la_is_track_width():
    # D10: B.1.2-3 的 La 取轮距（与第六章自走式挂车最小转弯半径式吻合），
    # 不再暂取 0——否则 R外(轮迹) 与转弯通道宽度 W_z 被低估（偏不安全）
    tl = _h_trailer(axle_count=4, axle_spacing=1.5, track_width=2.9)
    b = _h_bundle(tl)
    r = hydraulic_turn_radii(b, actual_turn_radius=26.5)
    assert r["La"] == pytest.approx(2.9)
    d_half = 1.5 * (4 - 1) / 2.0                             # l = 轴组跨度/2
    assert r["R_vehicle_out"] == pytest.approx(
        math.hypot(r["base"] + 2.9, d_half))
    assert r["W_turn"] == pytest.approx(
        r["R_vehicle_out"] - r["R_vehicle_in"] + TURN_SAFE_CLEARANCE_B12)
    # 第六章 R_min = √((l/tanα_max + b)² + l²) + (B−b)/2 与 La=轮距时逐项一致
    # （l/tanα_max + b 即 B.1.2-2 的 base + La；B−b 为车宽相对轮距的悬伸）
    r6 = math.hypot(r["base"] + 2.9, d_half) \
        + (tl.width - tl.track_width) / 2.0
    assert r["R_vehicle_out"] == pytest.approx(
        r6 - (tl.width - tl.track_width) / 2.0, abs=1e-9)


def test_hydraulic_rigid_sweep_radii_excludes_tractor():
    # 仅板车+货物刚体扫空：R_o=√((R+w/2)²+(L/2)²)、R_i=R−w/2，与牵引车无关
    b = _h_bundle(_h_trailer(axle_count=4, axle_spacing=1.5))
    r_o, r_i = hydraulic_rigid_sweep_radii(b, 26.5)
    assert r_o == pytest.approx(math.hypot(26.5 + 1.5, 6.0))   # 包络 12 m × 3.0 m
    assert r_i == pytest.approx(26.5 - 1.5)
    # 换更大牵引车不改变板车扫空半径（B.1.2 条文说明：牵引车基本不增加转弯半径）
    b2 = _h_bundle(_h_trailer(axle_count=4, axle_spacing=1.5))
    b2.tractor.width, b2.tractor.length = 3.5, 12.0
    assert hydraulic_rigid_sweep_radii(b2, 26.5) == (r_o, r_i)


def test_hydraulic_arc_clearance_uses_trailer_only():
    # 圆弧弯道液压车的 R_o/R_i 取板车刚体几何（不含车头），内/外缘预留转正
    b = _h_bundle(_h_trailer(axle_count=4, axle_spacing=1.5),
                  outer=30.0, lane=7.0)
    res = run_scene(b)
    m = res.metrics
    r_o, r_i = hydraulic_rigid_sweep_radii(b, m["trailer_turn_radius"])
    assert m["R_o"] == pytest.approx(r_o, abs=1e-9)
    assert m["R_i"] == pytest.approx(r_i, abs=1e-9)
    assert m["clearance_outer"] > 0 and m["clearance_inner"] > 0
    assert "板车刚体扫空量测" in "\n".join(res.verdict_lines)


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


# ---- 查表法不豁免 4.2.1 横向预留 σ ----
def test_lookup_pass_does_not_waive_lateral_clearance():
    # 查表法（4.3.1-1）只覆盖"转弯通道宽度/扫空空间宽度"两项**宽度**判据；
    # 4.2.1 的横向预留空间 σ 属路线层面的独立要求，查表命中也要校核。
    # 该工况查表命中，但车货外廓已越过路面内缘（内缘净空为负）→ 应判不通过。
    # （15 m 板车在 R=30/7 m 的路面上，扫掠内径 R_i≈20.3 m 已切进内缘 23.0 m 内侧）
    res = run_scene(_arc_bundle(lowbed=True, outer=30.0, lane=7.0, length=15.0))
    m = res.metrics
    assert m["lookup"] == "pass"
    assert m["clearance_inner"] < 0
    assert res.pass_ok is False
    assert "横向预留" in m["method"]
    assert "横向预留" in "\n".join(res.verdict_lines)


def test_lookup_pass_with_adequate_clearance_passes():
    # 同一查表命中的口径下，4.2.1 内/外缘预留满足时仍判通过（示例2"刚好通过"工况）
    res = run_scene(_arc_bundle(lowbed=True, outer=30.0, lane=9.8))
    m = res.metrics
    assert m["lookup"] == "pass"
    assert m["clearance_inner"] >= m["sigma"] - 1e-6
    assert res.pass_ok is True


def test_clearance_metrics_keep_negative_sign():
    # 净空指标不再 max(0,·) 截断，与结论文本同符号、可判断越界量
    res = run_scene(_arc_bundle(lowbed=True, outer=30.0, lane=7.0))
    m = res.metrics
    assert m["clearance_outer"] == pytest.approx(
        m["road_outer"] - m["R_o"], abs=1e-9)
    assert m["clearance_inner"] == pytest.approx(
        m["R_i"] - (m["road_outer"] - m["road_lane"]), abs=1e-9)
    assert m["clearance_inner"] < 0


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
    # 综合结论 = 5.2.1 宽度判据 且 路口扫空校核（V1.5.0 起对所有工况生效）
    assert res.pass_ok == (right_ok and m["split_vehicle_ok"] and m["split_cargo_ok"])
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
    # 宽度/长度/高度各区间 → 级。规范 3.2.4 高度 A/B/C 共区 (4.00,4.50]，
    # 高度只在跨越 4.5/5.0 边界时抬到 D/E；区间内子级由长/宽维度决定（不再抢档）。
    assert classify(10, 2.8, 3.9, False) == "A"
    assert classify(10, 2.8, 4.2, False) == "A"   # 高 4.2 → 高度不抬级，长/宽皆 A
    assert classify(20, 3.2, 4.2, False) == "B"   # 长 B、宽 B、高 4.2 不抬级 → B（修正前误判 C）
    assert classify(26, 3.6, 4.4, False) == "C"   # 宽 C
    assert classify(33, 4.0, 4.8, False) == "D"   # 高 4.8 → D
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
    r_out = [r["R_out"] for r in radii]
    # 轴数 → 首末有效轴距 d 增大，B.1.2-5 外缘半径 Rmax 随轴数单调增
    assert all(r_out[i] < r_out[i + 1] for i in range(len(r_out) - 1))
    # 每个半径均有限且内缘扫空半径非负（车板过长时 B.1.2-4 可能取负，已钳到 0.5）
    for r in radii:
        assert math.isfinite(r["R_in"]) and math.isfinite(r["R_out"]) and r["R_in"] >= 0
    # B.1.2-6 扫空宽度 Wd = Rmax − Rmin + σ（含安全宽度预留）
    b = _arc_bundle(lowbed=False, axle_count=4, axle_spacing=1.5)
    r = hydraulic_turn_radii(b)
    assert r["W_swept"] == pytest.approx(
        r["R_out"] - r["R_in"] + TURN_SAFE_CLEARANCE_B12)
    # B.1.2-1 转弯通道宽度 = 轮迹 R外(轮迹) − R内(轮迹) + σ（不再复用扫空基数 d）
    assert r["W_turn"] == pytest.approx(
        r["R_vehicle_out"] - r["R_vehicle_in"] + TURN_SAFE_CLEARANCE_B12)


def test_m6_base_independent_of_track():
    # 规范 B.1.2-2：base = L轮挂·(Na−1)/(2tanθ) 只取纵向轴距与 θ，
    # 与轮距无关；轮距只作为车体相对轮迹线的横向偏距 d/2 进入 B.1.2-4/-5。
    base = _arc_bundle(lowbed=False, axle_count=4, axle_spacing=1.5, track=2.9)
    r0 = hydraulic_turn_radii(base)
    wide = _arc_bundle(lowbed=False, axle_count=4, axle_spacing=1.5, track=5.0)
    r_wide = hydraulic_turn_radii(wide)
    long = _arc_bundle(lowbed=False, axle_count=4, axle_spacing=3.0, track=2.9)
    r_long = hydraulic_turn_radii(long)
    # 轮距不改变 base / θ
    assert r_wide["base"] == pytest.approx(r0["base"], abs=1e-9)
    assert r_wide["theta_deg"] == pytest.approx(r0["theta_deg"], abs=1e-9)
    # 轮距加大 → 车体相对轮迹线的悬伸变小 → 内缘半径上抬、外缘半径外扩
    assert r_wide["R_in"] > r0["R_in"]
    assert r_wide["R_out"] > r0["R_out"]
    assert r_wide["W_swept"] == pytest.approx(
        r_wide["R_out"] - r_wide["R_in"] + TURN_SAFE_CLEARANCE_B12)
    # 轴距影响：轴组跨度增大 → base 与扫空宽度随之增大
    assert r_long["base"] > r0["base"]
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
    # 轴位精确到瞬心/车板坐标：三组代表轴（前/中/后）分铺车板；
    # front_overhang 可把首组整体后移（D7/D8），否则默认各内收 1m。
    tl = _h_trailer(axle_count=4, axle_spacing=1.5, length=12.0,
                    front_overhang=2.0)
    ax = axle_steering_geoms(0.0, 0.0, 0.0, tl, 10.0, na=4)
    # psi=0 → 前向=+x；首轴 cx = 6−2 = 4.0；中轴居中；末轴 = −(6 − 内收1) = −5.0
    first = next(a for a in ax if a[0] == 1)
    assert first[1] == pytest.approx(6.0 - 2.0, abs=1e-9)
    mid = next(a for a in ax if a[0] == 2)
    assert mid[1] == pytest.approx((6.0 - 2.0 - 5.0) / 2.0, abs=1e-9)
    last = next(a for a in ax if a[0] == 4)
    assert last[1] == pytest.approx(-5.0, abs=1e-9)


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
    # 加轴线层不影响 5.2.1：W/R_o/R_i 仍为正值且 right_ok 可判
    # （液压车自 V1.4.0 起 R_m/W/ΔB 改按"仅板车"刚体几何，见下方新用例）
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


# ---- 圆弧弯道：入弯前 / 弯道内×2 / 出弯后 共 4 辆车 ----
def test_arc_draws_four_poses_entry_curve_exit():
    # 顺序固定 [入弯前, 弯道内①, 弯道内②, 出弯后]，下标 0 用于车/货尺寸标注
    for lowbed in (True, False):
        res = run_scene(_arc_bundle(lowbed=lowbed, outer=30.0, lane=7.0))
        assert len(res.vehicle_frames) == 4

        def ang(f):
            return math.degrees(math.atan2(f[1], f[0]))
        a = [ang(f) for f in res.vehicle_frames]
        assert a[0] < 0.0, f"第 0 辆应为入弯前直行，实际 {a[0]:.1f}°"
        assert 0.0 <= a[1] <= 140.0 + 1e-6 and 0.0 <= a[2] <= 140.0 + 1e-6
        assert a[2] - a[1] > 30.0, "弯道内两辆应分开"
        assert a[3] > 140.0, f"第 3 辆应为完全出弯后直行，实际 {a[3]:.1f}°"
        # 入/出弯两辆在同一走行半径的直道上，参考点到圆心距离 > 走行半径
        r_veh = res.metrics["trailer_turn_radius"]
        assert math.hypot(*res.vehicle_frames[0][:2]) > r_veh
        assert math.hypot(*res.vehicle_frames[3][:2]) > r_veh


# ---- 直角弯液压车：仅板车（规范 5.2.1 口径）+ 可选 ΔW₂ ----
def _wide_cargo(width=5.0):
    return CargoParams(weight_t=80, length=10, width=width, height=3.0,
                       cg_height=1.2, cargo_bearing="干燥的木板", is_overhang=True)


def _ra_bundle(hyd=True, entry=18.5, exit_w=14.0, cargo_width=5.0):
    """直角弯工况：货宽 5.0 > 板宽 3.0（触发 5.2.4 分体扫空校核）。"""
    return InputBundle(
        cargo=_wide_cargo(cargo_width),
        trailer=(_h_trailer(axle_count=4, axle_spacing=1.5) if hyd
                 else _trailer()),
        tractor=_tractor(),
        road=RoadParams(road_type="直角弯道",
                        right_angle=RightAngleCorner(
                            entry_width=entry, exit_width=exit_w,
                            corner_radius=6.0, intersection_angle_deg=90.0),
                        vertical_clearance=7.5, highway_grade="三级公路"),
        max_speed_kmh=10.0)


def test_right_angle_hydraulic_uses_trailer_only():
    # 直角弯液压车 R_m/R_o/W 按"仅板车+货物"刚体几何解析量测，不含牵引车
    res = run_scene(_ra_bundle(hyd=True))
    m = res.metrics
    r, env_w, env_l = m["r_path"], max(3.0, 5.0), max(12.0, 10.0)
    assert m["hydraulic_trailer_only"] is True
    assert m["R_m"] == pytest.approx(r - env_w / 2.0, abs=1e-9)
    assert m["R_o"] == pytest.approx(
        math.hypot(r + env_w / 2.0, env_l / 2.0), abs=1e-9)
    assert m["W"] == pytest.approx(m["R_o"] - m["R_m"], abs=1e-9)
    # 换更大的牵引车不改变板车扫空参数
    b2 = _ra_bundle(hyd=True)
    b2.tractor.width, b2.tractor.length = 3.5, 12.0
    m2 = run_scene(b2).metrics
    for k in ("R_m", "R_o", "W"):
        assert m2[k] == pytest.approx(m[k], abs=1e-9)
    # 机械车仍走含牵引车的铰接仿真
    assert run_scene(_ra_bundle(hyd=False)).metrics[
        "hydraulic_trailer_only"] is False


def test_right_angle_spec_scope_and_optional_conservatism():
    # 规范口径（默认）：req_w = B_t + ΔB（= W），ΔW₂ = 0；出口略窄也可通过
    m = run_scene(_ra_bundle(hyd=True, entry=18.5, exit_w=7.0)).metrics
    assert m["tractor_swing_widening"] == 0.0
    assert m["required_w"] == pytest.approx(m["B_t"] + m["deltaB"], abs=1e-9)
    assert m["required_w"] == pytest.approx(m["W"], abs=1e-9)
    # 走行半径经寻优（取可行弧中最平缓者）后 W≈5.8，出口净宽 6.0 刚好够
    assert m["exit_net_w"] >= m["required_w"] and m["entry_ok"] and m["exit_ok"]
    assert run_scene(_ra_bundle(hyd=True, entry=18.5, exit_w=7.0)).pass_ok is True
    # 机械车同样 ΔW₂ = 0（包络本就含牵引车）
    assert run_scene(_ra_bundle(hyd=False)).metrics[
        "tractor_swing_widening"] == 0.0


def test_right_angle_optional_tractor_swing_widening(monkeypatch):
    # ΔW₂ 是规范之外的附加余量：开启后所需宽度加大，出口略窄的工况翻为不通过
    import app.core.passability as P
    monkeypatch.setattr(P, "TRACTOR_SWING_EXIT_FACTOR", 0.5)

    m = run_scene(_ra_bundle(hyd=True, entry=18.5, exit_w=7.0)).metrics
    assert m["tractor_swing_widening"] == pytest.approx(0.5 * 2.5, abs=1e-9)
    assert m["required_w"] == pytest.approx(m["W"] + 1.25, abs=1e-9)
    res = run_scene(_ra_bundle(hyd=True, entry=18.5, exit_w=7.0))
    assert res.metrics["exit_ok"] is False and res.pass_ok is False
    assert "附加安全余量" in "\n".join(res.verdict_lines)
    # 机械车不受该系数影响
    assert run_scene(_ra_bundle(hyd=False)).metrics[
        "tractor_swing_widening"] == 0.0


def test_right_angle_requires_both_entry_and_exit():
    # 规范两式等价于"W 须同时 ≤ 入口净宽与出口净宽"——只有一侧够宽不算通过
    wide = run_scene(_ra_bundle(hyd=True, entry=18.5, exit_w=14.0)).metrics
    assert wide["entry_ok"] and wide["exit_ok"]

    # 仅入口过窄：req_w = W 也随之增大，入口净宽不够 → 不通过
    res_entry = run_scene(_ra_bundle(hyd=True, entry=5.0, exit_w=14.0))
    em = res_entry.metrics
    assert em["entry_ok"] is False and em["exit_ok"] is True
    assert em["entry_net_w"] < em["required_w"]
    assert res_entry.pass_ok is False

    # 仅出口过窄到低于 req_w → 不通过（此处 req_w = 6.03，出口净宽 0.5）
    res_exit = run_scene(_ra_bundle(hyd=True, entry=18.5, exit_w=2.5))
    xm = res_exit.metrics
    assert xm["entry_ok"] is True and xm["exit_ok"] is False
    assert res_exit.pass_ok is False

    # 机械车 ΔW₂=0：req_w = W，入口/出口同样须同时满足
    # （机械车走行半径受"车板长 > 走行半径则尾随模型失效"约束，W 比液压车更宽）
    mm = run_scene(_ra_bundle(hyd=False, entry=18.5, exit_w=14.0)).metrics
    assert mm["required_w"] == pytest.approx(mm["W"], abs=1e-9)
    assert (mm["entry_ok"] and mm["exit_ok"]) == (
        mm["entry_net_w"] >= mm["required_w"] - 1e-6
        and mm["exit_net_w"] >= mm["required_w"] - 1e-6)


# ---- 直角弯：走行半径寻优 + 通用扫空校核（V1.5.0）----
def test_right_angle_radius_is_optimized_not_magic():
    """走行半径不再取 max(入口,出口,1.2×车板长,10) 的魔数，而是在路口内寻优：
    存在不越界的半径时取**最平缓**者；否则取切角最轻者并判不通过。"""
    res = run_scene(_ra_bundle(hyd=True, entry=18.5, exit_w=14.0))
    m = res.metrics
    # 18.5/14 + 12 m 板车：存在若干可行半径 → 取最大者，扫空不越界
    assert m["vehicle_outside"] <= 0.10 and m["cargo_outside"] <= 0.10
    assert m["overrun_best"] <= 0.10
    assert m["r_path"] > 0.5 * max(18.5, 14.0)
    # 圆心 = 路口角点 −(r, r)：弧与两条直道中心线相切
    px, py = m["corner_point"]
    assert m["turn_center"] == pytest.approx((px - m["r_path"], py - m["r_path"]))


def test_right_angle_swept_check_is_universal():
    """扫空越界校核对所有工况生效（不再只在"货物超宽"时触发）——
    修复前普通工况跳过该校核，图上包络已画出路面、结论却写"通过"。"""
    # 窄路口 + 长车板（机械挂车，铰接尾随）：无论怎么走都会切内角
    b = InputBundle(
        cargo=_cargo(),
        trailer=_trailer(lowbed=True, length=15.0, axle_count=6),
        tractor=_tractor(),
        road=RoadParams(road_type="直角弯道",
                        right_angle=RightAngleCorner(entry_width=7.0,
                                                     exit_width=7.0,
                                                     corner_radius=0.0),
                        vertical_clearance=7.5, highway_grade="三级公路"),
        max_speed_kmh=10.0)
    res = run_scene(b)
    m = res.metrics
    assert m["split_active"] is True
    assert m["vehicle_outside"] > 0.10
    assert m["vehicle_depth"] > 0.10          # 真实切角，不是多边形毛刺
    assert res.pass_ok is False
    assert "路口扫空" in "\n".join(res.verdict_lines)


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
    # Wd = Rmax − Rmin + σ（B.1.2-6），不再等切 d
    assert r["W_swept"] == pytest.approx(
        r["R_out"] - r["R_in"] + TURN_SAFE_CLEARANCE_B12)


def test_b12_locked_ends_extrap():
    # D1: 锁死首轴 → 规范外推 d 缩短 + 告警
    tl = _h_trailer(axle_count=5, locked_axles=[1])
    r = hydraulic_turn_radii(_h_bundle(tl))
    assert r["spec_mode"] is False
    assert r["d"] == pytest.approx(1.5 * (5 - 2))   # last=5,first=2
    assert r["warnings"] and "锁死" in r["warnings"][0]


def test_b12_D_total_is_total_width_and_coupling_independent():
    # D3/D8: B.1.2 的 D总 为**车货总宽**（车板端部横向量取），不是车板长；
    # 耦合/连接杆字段只影响绘图，不影响 D总/R/W_swept。
    tl0 = _h_trailer(length=12.0)
    tl1 = _h_trailer(length=12.0, coupling_type="连接杆", coupling_length=5.0)
    r0, r1 = hydraulic_turn_radii(_h_bundle(tl0)), hydraulic_turn_radii(_h_bundle(tl1))
    assert r1["D_total"] == pytest.approx(r0["D_total"])
    assert r1["D_total"] == pytest.approx(max(tl0.width, 3.0))   # 车货总宽
    # 车板长（改为 20 m）不再改变 D总 与扫空宽度——纵向尺寸只经轴组跨度进公式
    r2 = hydraulic_turn_radii(_h_bundle(_h_trailer(length=20.0)))
    assert r2["D_total"] == pytest.approx(r0["D_total"])
    assert r2["W_swept"] == pytest.approx(r0["W_swept"])
    for k in ("R_in", "R_out", "W_swept", "W_turn", "d"):
        assert r1[k] == pytest.approx(r0[k])


def test_b12_W_turn_indicator():
    # D4: W_turn = 轮迹 R外−R内+σ（B.1.2-1），run_arc_scene 输出指标
    r = hydraulic_turn_radii(_h_bundle(_h_trailer(axle_count=4, axle_spacing=1.5)))
    assert r["W_turn"] == pytest.approx(
        r["R_vehicle_out"] - r["R_vehicle_in"] + TURN_SAFE_CLEARANCE_B12)
    res = run_scene(_h_bundle(_h_trailer(), outer=30.0, lane=7.0))
    m = res.metrics
    assert "B1_2_W_turn" in m and "B1_2_W_swept" in m and "B1_2_d" in m
    assert m["B1_2_W_turn"] == pytest.approx(
        m["B1_2_R_vehicle_out"] - m["B1_2_R_vehicle_in"] + TURN_SAFE_CLEARANCE_B12)


def test_b12_active_lt2_fallback():
    # 可转向轴<2 → 回退规范基准 + 告警
    tl = _h_trailer(axle_count=3, locked_axles=[1, 2])
    r = hydraulic_turn_radii(_h_bundle(tl))
    assert r["spec_mode"] is True and any("转向轴<2" in w for w in r["warnings"])


def test_arc_b12_indicator():
    res = run_scene(_h_bundle(_h_trailer()))
    m = res.metrics
    for k in ("B1_2_W_turn", "B1_2_d", "B1_2_D_total", "B1_2_spec_mode",
              "B1_2_warnings", "B1_2_theta_deg", "B1_2_base",
              "trajectory_source"):
        assert k in m
    assert m["trajectory_source"] == "B.1.2"


def test_b12_judgement_turn_channel_vs_road_width():
    # D4 判定联动：转弯通道 W_turn（B.1.2-1 轮迹）作为展示指标，扫空宽度 Wd 进入
    # “≤横向空间宽度”判据。此用例确保窄车道依托 Wd 判定不异常挂起。
    tl = _h_trailer(axle_count=4, axle_spacing=5.0)
    res = run_scene(_h_bundle(tl, outer=40.0, lane=9.0))
    # 查表(3级C半径>22、横向空间18>…)若不命中则落入验算；此用例确保不因异常而挂
    assert isinstance(res.metrics["B1_2_W_turn"], float)
    assert res.metrics["B1_2_W_turn"] == pytest.approx(
        res.metrics["B1_2_R_vehicle_out"] - res.metrics["B1_2_R_vehicle_in"]
        + TURN_SAFE_CLEARANCE_B12)


# ---- 物理合理性护栏（V1.5.0：防止"公式自洽但物理错"再次发生）----
def test_hydraulic_swept_width_decreases_with_radius():
    """扫空宽度必须随弯道半径增大单调减小，并在大半径时收敛到车宽量级。

    旧口径把 B.1.2 的 D总 当成车板长，W_d 几乎与半径无关（R=16→15.87、
    R=100→15.59），且大于车体对角线——该护栏用于锁死这类"量纲放错"的回归。
    """
    tl = _h_trailer(axle_count=6, axle_spacing=1.5, length=15.0, width=3.0)
    b = _h_bundle(tl)
    ws = [hydraulic_turn_radii(b, actual_turn_radius=R)["W_swept"]
          for R in (12, 16, 20, 30, 50, 100)]
    assert all(ws[i] > ws[i + 1] for i in range(len(ws) - 1))
    assert ws[-1] < 2 * max(tl.width, 3.0)          # 大半径下趋近车宽
    # 刚体扫空带同样单调，且两者量级一致（差异仅来自 B.1.2-5 的纵向半长口径）
    bands = []
    for R in (12, 16, 20, 30, 50, 100):
        ro, ri = hydraulic_rigid_sweep_radii(b, R)
        bands.append(ro - ri)
    assert all(bands[i] > bands[i + 1] for i in range(len(bands) - 1))
    for w, band in zip(ws, bands):
        assert abs(w - band) < 1.5


def test_hydraulic_swept_width_below_body_diagonal():
    """单车刚体的扫空带宽度不可能超过车体对角线（旧口径 15.87 m > 15.30 m）。"""
    for length, width, na in ((15.0, 3.0, 6), (20.0, 4.5, 8), (12.0, 3.0, 4)):
        tl = _h_trailer(axle_count=na, axle_spacing=1.5,
                        length=length, width=width)
        diag = math.hypot(length, width)
        for R in (10, 20, 40):
            r = hydraulic_turn_radii(_h_bundle(tl), actual_turn_radius=R)
            assert r["W_swept"] < diag, (length, width, R, r["W_swept"])


def test_mechanical_steady_measurement_independent_of_entry_pose():
    """机械车 R_o/R_i 不得取决于仿真起点（入弯直道长度）。

    修复前 measure_steady_turn 把入场过渡段一起统计：起点后退 6 m，R_o 就从
    27.68 涨到 31.3，扫空宽度随"从哪一点开始算"漂移。
    """
    from app.core.kinematics import TractorTrailerKinematics
    from app.core.swept_path import measure_steady_turn
    from config import SIM_DT, SIM_TURN_SPEED

    b = _arc_bundle(lowbed=True, outer=24.0, lane=4.0, length=15.0)
    out = []
    for y0 in (0.0, -6.0, -12.0):
        kin = TractorTrailerKinematics(b)
        R = 22.0
        delta = min(kin.max_delta, math.atan(kin.wheelbase / R))
        r_path = kin.wheelbase / math.tan(delta)
        kin.reset_pose(r_path, y0, math.radians(90))
        kin.delta = delta
        ang = SIM_TURN_SPEED * math.tan(delta) / kin.wheelbase
        steps = int(4 * math.pi / (ang * SIM_DT))
        # 起点后退 y0 时转动圆心随之平移到 (0, y0)
        out.append(measure_steady_turn(kin, steps, v=SIM_TURN_SPEED, dt=SIM_DT,
                                       center=(0.0, y0)))
    for ro, ri in out[1:]:
        assert ro == pytest.approx(out[0][0], abs=0.05)
        assert ri == pytest.approx(out[0][1], abs=0.05)


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
    # 偶轴 N_a → 仍是 前/中/后 三组代表轴（第六章图式）；中间组为枢轴中心 δ=0
    tl = _h_trailer(axle_count=4)
    axles = axle_steering_geoms(0.0, 0.0, 0.0, tl, 10.0, na=4)
    assert [a[0] for a in axles] == [1, 2, 4]
    mid = next(a for a in axles if a[0] == 2)
    assert mid[3] == 0.0 and mid[5] is False   # 中间=枢轴，δ=0，非闭锁
    axles_locked = axle_steering_geoms(0.0, 0.0, 0.0, tl, 10.0, na=4,
                                       locked_axles=[2])
    # 用户显式锁死中间轴 → 中间组标记为锁死且 δ=0
    assert 2 in [a[0] for a in axles_locked]
    m2 = next(a for a in axles_locked if a[0] == 2)
    assert m2[3] == 0.0 and m2[5] is True


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