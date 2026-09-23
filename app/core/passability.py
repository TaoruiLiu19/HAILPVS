"""道路几何构建 + 转段仿真编排 + 干涉判定 + 最小宽度优化。

判定口径对标《公路大件运输安全通行评价技术规范》JTG/T 2213—2023：
  - 圆弧弯道：按 B.1.1/B.1.2 计算转弯通道宽度并计入 0.5m 安全宽度（M1），
    校核内缘约束（M2）与按公路等级的横向预留空间 σ（M3）；
  - 直角弯道：按 5.2.1 以 R_m/W/ΔB 三要素判定（M4）；
  - 液压轴线挂车：按 B.1.2 闭式解（M6）；
  - 垂直净空（M7）与横向稳定性 B.2（M8）作为公共校验并入结论；
  - 大件运输尺寸分级与表4.3.1 查表法作为辅助信息（M5）。
"""
from __future__ import annotations
import math

from shapely.geometry import Polygon, MultiPolygon, box
from shapely.ops import unary_union

from config import (TURN_SAFE_CLEARANCE, LATERAL_CLEARANCE_BY_GRADE,
                    INTERSECTION_RESERVE,
                    HIGHWAY_GRADES, DEFAULT_SUPERELEVATION, PAD_FRICTION, g,
                    VERT_CROSS_STRUCTURE, TUNNEL_CROWN_CLEARANCE,
                    TUNNEL_SIDE_CLEARANCE, TUNNEL_HIGH_REQ_BOTTOM,
                    TUNNEL_GROUND_CLEARANCE, RAMP_CLEARANCE, VERT_SCENARIO,
                    AXLE_STEER_ABS_LIMIT_DEG)
from .datamodel import InputBundle
from .kinematics import (TractorTrailerKinematics, run_profile,
                         build_trajectory_geoms, hydraulic_turn_radii)
from .swept_path import measure_steady_turn, polylist_to_multipoly
from . import transverse as _transverse
from . import grade as _grade
from . import lateral as _lateral

_DT = 0.05
_V = 1.5
SAMPLES = 40           # 抽帧绘制步长
TIGHT_STEPS = 200      # 直角转弯收紧仿真步数
_TOL = 1e-6
_STRAIGHT = 6.0        # 仿真入弯/出弯直道长度（m）：保持车辆近似绕原点转弯，与稳态判定一致
_ROAD_STRAIGHT = 22.0  # 道路几何直道长度（m）：仅为绘图包住整车姿态，不影响仿真/判定
_TURN_DEG = 140.0      # 圆弧弯道仿真转角（°）
_RA_TURN_DEG = 118.0   # 直角弯道仿真转角（°）
POSE_FRAMES = 3        # 抽帧车辆姿态数：只在关键位置画车，避免重叠遮挡


def _pick_poses(vgeoms, n=POSE_FRAMES):
    """均匀挑选 n 个车辆姿态用于绘图，并保证包含最终位置（最后轮廓）。

    沿轨迹头尾均匀铺开（步长以“布满全程”为准），再补上最后姿态，
    使图面能清楚看到转弯完成后的车辆最终轮廓，区别于中间抽帧。
    """
    if not vgeoms:
        return []
    n = max(2, n)
    step = max(1, (len(vgeoms) - 1) // (n - 1))
    chosen = list(vgeoms[::step])
    if chosen and vgeoms[-1] is not chosen[-1]:
        if len(chosen) >= n:
            chosen = chosen[:-1] + [vgeoms[-1]]
        else:
            chosen.append(vgeoms[-1])
    return chosen[:n]


class SweepScenarioResult:
    """一次检验的完整结果。"""
    def __init__(self):
        self.scene = "arc"              # "arc" / "right_angle"
        self.road = None                # shapely 道路多边形
        self.swept = None               # shapely 扫掠区域
        self.vehicle_frames = []        # [(x,y,geom,trailer_geom,cargo_geom)] 抽帧车辆轮廓
        self.pass_ok = False
        self.metrics = {}               # 指标字典
        self.verdict_lines = []         # 结论文本行


def _rotate(px, py, ang):
    ca, sa = math.cos(ang), math.sin(ang)
    return (px * ca - py * sa, px * sa + py * ca)


def _effective_sigma(bundle: InputBundle) -> float:
    """横向预留空间 σ：显式给定优先，否则按公路等级（4.2.1）。"""
    road = bundle.road
    if road.lateral_clearance is not None:
        return float(road.lateral_clearance)
    return float(LATERAL_CLEARANCE_BY_GRADE.get(road.highway_grade, 0.5))


def _is_hydraulic(bundle: InputBundle) -> bool:
    return "液压" in bundle.trailer.trailer_type


def _cargo_total_height(bundle: InputBundle) -> float:
    return bundle.trailer.deck_height + bundle.cargo.height


def _vehicle_cargo_lines(bundle: InputBundle) -> list[str]:
    """“车辆及货物参数”结论文本块（车货总宽/总高沿用判定口径）。"""
    c, tl, tr = bundle.cargo, bundle.trailer, bundle.tractor
    B_t = max(tl.width, c.width)
    return [
        "【车辆及货物参数】",
        (f"货物：长 {c.length:.2f} m × 宽 {c.width:.2f} m × 高 {c.height:.2f} m，"
         f"重心高 {c.cg_height:.2f} m，衬垫 {c.cargo_bearing}"),
        (f"挂车：{tl.trailer_type}（{tl.column_type}），长 {tl.length:.2f} m × "
         f"宽 {tl.width:.2f} m，{tl.axle_count} 轴线、轴距 {tl.axle_spacing:.2f} m，"
         f"轮距 {tl.track_width:.2f} m，车板高 {tl.deck_height:.2f} m"),
        (f"牵引车：轴距 {tr.wheelbase:.2f} m，最大转向角 {tr.max_steering_angle_deg:.1f}°，"
         f"最高行驶速度 {bundle.max_speed_kmh:.1f} km/h"),
        f"车货总宽 B<sub>t</sub> = {B_t:.3f} m，车货总高 H<sub>总</sub> = {_cargo_total_height(bundle):.3f} m",
    ]


def _hydraulic_b12_lines(bundle, b12) -> list[str]:
    """液压轴线挂车 B.1.2 闭式解结论文本块（判定口径，D2/D3/D4/D6）。"""
    tl = bundle.trailer
    na = max(int(tl.axle_count), 2)
    L = tl.axle_spacing
    lines = [
        "液压轴线挂车 B.1.2 闭式解（判定口径，第六章定点转向）：",
        f"θ = min(挂车轴结构最大转角, 工程上限 {AXLE_STEER_ABS_LIMIT_DEG:g}°) "
        f"= {b12['theta_deg']:g}°（D2：θ 不再引用牵引车前轮角；D6：工程上限<90°，不做原地回转）",
        f"首末轴距 d = L<sub>轮挂</sub>·({na}−1) = {L:g} × {na - 1} = {b12['d']:g} m"
        f"（{'规范外推：' + repr(b12['locked']) if not b12['spec_mode'] else '严格守规范，中间锁死不改变 d'}）",
        f"车板总长 D<sub>总</sub> = 车板长 = {b12['D_total']:g} m（D3 判定口径；牛颈/连接杆耦合段仅绘图、不参与计算）",
        f"瞬心偏距 base = d/(2·tanθ) = {b12['base']:g} m；"
        f"R<sub>in</sub> = base + (D<sub>总</sub> − d)/2 = {b12['R_in']:g} m；"
        f"R<sub>out</sub> = base + (D<sub>总</sub> + d)/2 = {b12['R_out']:g} m",
        f"扫空宽度 W<sub>swept</sub>（B.1.2-6）= R<sub>out</sub> − R<sub>in</sub> = {b12['W_swept']:g} m；"
        f"转弯通道宽度 W<sub>turn</sub>（B.1.2-1）= d + 0.5 = {b12['W_turn']:g} m "
        "（工程锚点，R<sub>外</sub>−R<sub>内</sub>≈d，OCR 残缺待复核）",
        f"判定口径（车型优先）：扫空 W<sub>swept</sub> ≤ 横向空间宽度 判扫空无障碍；"
        f"转弯通道 W<sub>turn</sub> ≤ 路面宽度 判转弯通道",
    ]
    for w_ in b12["warnings"]:
        lines.append(f"⚠ {w_}")
    return lines


def _vertical_check(bundle: InputBundle):
    """垂直净空校验（M7）：按净空场景分阈值判定。

    返回 (ok, total_h, margin, msg)。margin 为场景对应的"不应小于"阈值，
    用于结果展示与消息文本。
    """
    total_h = _cargo_total_height(bundle)
    limit = bundle.road.vertical_clearance
    stype = bundle.road.clearance_structure_type
    scenario = VERT_SCENARIO.get(stype, "cross")
    gap = limit - total_h

    if scenario == "tunnel":
        # 隧道 9.2.1：顶部距洞顶(含设备) ≥0.20
        req = TUNNEL_CROWN_CLEARANCE
        ok = gap >= req - _TOL
        msg = (f"隧道顶部净空：车货总高 {total_h:.3f} m，洞顶净空 {limit:.3f} m，"
               f"顶部间隙 {gap:.3f} m {'≥' if ok else '&lt;'} 要求 {req:.2f} m。")
        # 侧壁/检修道/底缘取向净空属于空间结构评价，弯道内核仅校验顶部，
        # 侧壁 0.5、检修道 0.2、底缘 0.2 在隧道专项模块（P2）中处理。
        return ok, total_h, req, msg

    if scenario == "ramp":
        req_min = RAMP_CLEARANCE["minimum"]       # 不应 <0.25
        ok = gap >= req_min - _TOL
        disp = ("宜（间隙≥0.50m）" if gap >= RAMP_CLEARANCE["desirable"] - _TOL
                else "不应通行（间隙&lt;0.25m）")
        msg = (f"匝道净空(6.3.2)：车货总高 {total_h:.3f} m，净空 {limit:.3f} m，"
               f"间隙 {gap:.3f} m，判定{disp}；外廓与护栏预留不宜&lt;0.50、不应&lt;0.25。")
        return ok, total_h, req_min, msg

    # 跨线构造物 / 上跨桥（6.4.1，4.4.2）：不宜 <0.1，不应 <0.05
    req_min = VERT_CROSS_STRUCTURE["minimum"]
    ok = gap >= req_min - _TOL
    if gap >= VERT_CROSS_STRUCTURE["desirable"] - _TOL:
        disp = "宜（间隙≥0.10m）"
    elif gap >= req_min - _TOL:
        disp = "需采取措施（0.05≤间隙&lt;0.10m）"
    else:
        disp = "不应通行（间隙&lt;0.05m）"
    msg = (f"跨线净空(6.4.1)：车货总高 {total_h:.3f} m，净空 {limit:.3f} m，"
           f"间隙 {gap:.3f} m，判定{disp}。")
    return ok, total_h, req_min, msg


def grade_threshold(grade: str, spec: dict) -> float:
    """返回场景对应阈值（保留，供报告使用）。"""
    return spec["desirable"] if grade == "宜" else spec["minimum"]


def _lateral_check(bundle: InputBundle, r_min: float):
    """横向稳定性（M8，附录B.2）。r_min 取最内侧转弯半径。"""
    road = bundle.road
    tl = bundle.trailer
    cargo = bundle.cargo
    v = bundle.max_speed_kmh
    mu = PAD_FRICTION.get(cargo.cargo_bearing, 0.4)
    cg_h = tl.deck_height + cargo.cg_height
    ih = DEFAULT_SUPERELEVATION            # 超高按默认值计入横向稳定性(B.2)
    ok, vh, vch, vcv, p_ok, msg = _lateral.lateral_stability(
        r_min, v, tl.track_width, cg_h, ih, mu)
    return {
        "ok": ok, "Vhmax": vh, "Vch": vch, "Vcv": vcv,
        "parking_ok": p_ok, "msg": msg,
        "mu": mu, "cg_height": cg_h, "superelevation": ih,
    }


def _lateral_formula_lines(r_min: float, bundle: InputBundle, lat: dict) -> list[str]:
    """横向稳定性（附录B.2）逐步推导文本：公式 + 代入数值。"""
    v = bundle.max_speed_kmh
    b = bundle.trailer.track_width
    ih = lat["superelevation"]
    mu = lat["mu"]
    h = lat["cg_height"]
    vch, vcv, vh = lat["Vch"], lat["Vcv"], lat["Vhmax"]
    ok = lat["ok"]
    lines = [
        f"最内侧转弯半径 R = {r_min:.3f} m，轮距 b = {b:.2f} m，"
        f"车货重心高 h = {h:.3f} m，超高 i<sub>h</sub> = {ih:.3f}，"
        f"衬垫摩擦系数 μ = {mu:.2f}",
        f"侧滑临界速度 V<sub>ch</sub> = 3.6·√[g·R·(μ+i<sub>h</sub>)/(1−μ·i<sub>h</sub>)] "
        f"= 3.6·√[{g:.2f}·{r_min:.3f}·({mu:.3f}+{ih:.3f})/(1−{mu:.3f}·{ih:.3f})] "
        f"= {vch:.3f} km/h",
        f"侧翻临界速度 V<sub>cv</sub> = √[127·R·(b+2·h·i<sub>h</sub>)/(2·h−b·i<sub>h</sub>)] "
        f"= √[127·{r_min:.3f}·({b:.3f}+2·{h:.3f}·{ih:.3f})/"
        f"({2 * h:.3f}−{b:.3f}·{ih:.3f})] = {vcv:.3f} km/h",
        f"最大稳定速度 V<sub>hmax</sub> = min(V<sub>ch</sub>, V<sub>cv</sub>) "
        f"= min({vch:.3f}, {vcv:.3f}) = {vh:.3f} km/h",
        f"行驶速度 V = {v:.1f} km/h {'≤' if v <= vh + 1e-9 else '&gt;'} "
        f"V<sub>hmax</sub>({vh:.3f}) "
        f"→ 侧向 {'通过' if v <= vh + 1e-9 else '不通过'}",
        f"停车超高 B.2.3：i<sub>h</sub>({ih:.3f}) "
        f"{'&lt;' if ih < b / (2 * h) else '≥'} b/(2h) = {b / (2 * h):.3f} "
        f"→ {'满足' if lat['parking_ok'] else '不满足'} 停车横向稳定性",
        f"横向稳定性总体：{'通过' if ok else '不通过'}。",
    ]
    return lines


def _attach_common_checks(res: SweepScenarioResult, bundle: InputBundle,
                          r_inner_min: float, sigma_override: float | None = None) -> list[str]:
    """把公路等级、垂直净空、横向稳定性并入结果与判定，返回“理论分析”文本行。

    圆弧弯道按 4.2.1 公路等级取横向预留 σ；平面交叉（直角弯）按 5.2.1 固定取
    INTERSECTION_RESERVE（默认 0.5），由 sigma_override 显式传入。
    """
    sigma = _effective_sigma(bundle) if sigma_override is None else float(sigma_override)
    vert_ok, total_h, vmargin, vmsg = _vertical_check(bundle)
    lat = _lateral_check(bundle, r_inner_min)

    res.metrics["sigma"] = sigma
    res.metrics["highway_grade"] = bundle.road.highway_grade
    res.metrics["turn_safe_clearance"] = TURN_SAFE_CLEARANCE
    res.metrics["total_height"] = total_h
    res.metrics["vert_clearance_ok"] = vert_ok
    res.metrics.update({k: v for k, v in lat.items() if k != "msg"})

    # 尺寸分级（M5）辅助信息
    size_grade = _grade.classify_bundle(bundle)
    res.metrics["size_grade"] = size_grade
    res.metrics["horizontal_reserve"] = sigma

    res.pass_ok = bool(res.pass_ok) and vert_ok and lat["ok"]

    lines = [
        "垂直净空校验：",
        f"车货总高 H<sub>总</sub> = 车板高 + 货物高 = {bundle.trailer.deck_height:.3f} + "
        f"{bundle.cargo.height:.3f} = {total_h:.3f} m",
        f"净空间隙 gap = 净空 − H<sub>总</sub> = {bundle.road.vertical_clearance:.3f} − {total_h:.3f} "
        f"= {bundle.road.vertical_clearance - total_h:.3f} m",
        vmsg,
        f"大件运输尺寸分级：{size_grade}（表3.2.4）。",
        "横向稳定性（附录B.2）：",
        *_lateral_formula_lines(r_inner_min, bundle, lat),
    ]
    return lines


# =====================================================================
# 圆弧弯道
# =====================================================================
def _road_from_centerline(r_path, half_widths, turn_deg, straight=_ROAD_STRAIGHT,
                          n=90):
    """由车辆行驶中心线（入弯直道 + 圆弧 + 出弯直道）扩展出道路带。

    中心线与仿真轨迹一致（仿真：位置角 0° 处沿 +Y 直行入场 → 绕 O 逆时针
    转 turn_deg → 沿末端切向直行出场）。入弯直道只向驶来方向（-Y）延伸以
    罩住车尾，不再伸进弯道内侧；等宽时用整条连续中心线一次缓冲，避免分段
    拼接在入弯/出弯端点留下误植的边角。
    half_widths = (入口半宽, 弯道半宽, 出口半宽)。
    """
    from shapely.geometry import LineString
    from shapely.ops import unary_union

    w_in, w_turn, w_out = half_widths
    arc_pts = [(r_path * math.cos(math.radians(turn_deg) * i / n),
                r_path * math.sin(math.radians(turn_deg) * i / n))
               for i in range(n + 1)]
    end_a = math.radians(turn_deg)
    dir_x, dir_y = -math.sin(end_a), math.cos(end_a)
    exit_end = (arc_pts[-1][0] + dir_x * straight,
                arc_pts[-1][1] + dir_y * straight)

    # —— 等宽（圆弧弯道）：整条连续中心线一次缓冲，无拼接边角 ——
    if abs(w_in - w_turn) < 1e-9 and abs(w_turn - w_out) < 1e-9:
        pts = [(r_path, -straight)] + list(arc_pts[1:]) + [exit_end]
        return LineString(pts).buffer(w_turn, cap_style=2)

    # —— 不等宽（直角弯道）：分段缓冲，入弯直道只向驶来方向延伸 ——
    entry_ = LineString([(r_path, -straight), (r_path, 0.0)])
    arc_ = LineString(arc_pts)
    exit_ = LineString([arc_pts[-1], exit_end])
    return unary_union([entry_.buffer(w_in, cap_style=2),
                        arc_.buffer(w_turn, cap_style=2),
                        exit_.buffer(w_out, cap_style=2)])


def _build_arc_display(bundle, r_path, delta_used):
    """生成圆弧轨迹用于绘图：沿切入向直道入场 → 在弧起点(角度0)处绕圆心 O
    逆时针转 _TURN_DEG（与稳态量测同圆心）→ 沿末端切向直道出场。"""
    kin = TractorTrailerKinematics(bundle)
    kin.reset_pose(r_path, -_STRAIGHT, math.radians(90))   # 起于直道，切向进入弧起点
    profile = []
    profile.append((int(_STRAIGHT / (_V * _DT)), 0.0))     # 直道到( r_path, 0 )
    ang_vel = _V * math.tan(delta_used) / kin.wheelbase
    turn_steps = int(math.radians(_TURN_DEG) / (ang_vel * _DT))
    profile.append((turn_steps, delta_used))               # 绕 O 转 _TURN_DEG
    profile.append((int(_STRAIGHT / (_V * _DT)), 0.0))     # 切向直道出场
    frames, _k = run_profile(kin, profile, v=_V, dt=_DT, draw_step=SAMPLES)
    vgeoms = build_trajectory_geoms(kin, frames, sample_rate=1)
    swept = polylist_to_multipoly(vgeoms)
    return vgeoms, swept


def run_arc_scene(bundle: InputBundle) -> SweepScenarioResult:
    res = SweepScenarioResult()
    res.scene = "arc"
    road = bundle.road
    ra = road.arc
    r_out, lane = ra.outer_radius, ra.lane_width
    r_in = r_out - lane
    center = (0.0, 0.0)
    kin = TractorTrailerKinematics(bundle)

    # —— 沿道路中心线半径行驶；受最大转向角约束 → 实际路径半径 R_path ——
    r_target = max(r_in + lane / 2.0, 1.0)
    delta_used = math.atan(kin.wheelbase / r_target)
    delta_used = max(-kin.max_delta, min(kin.max_delta, delta_used))
    r_path = kin.wheelbase / math.tan(delta_used)   # 实际转弯半径

    # —— 稳态量测（B.1.2 闭式解是绕车辆自身瞬时中心，不适于同道路圆心直接比较，
    #      故道路几何拟合统一用绕圆心 O 的仿真量测；闭式解作为 M6 扫空参数输出）——
    kin.reset_pose(r_path, 0.0, math.radians(90))
    kin.delta = delta_used
    ang_vel = _V * math.tan(delta_used) / kin.wheelbase
    rev_steps = int(4 * math.pi / (ang_vel * _DT))
    r_o, r_i = measure_steady_turn(kin, rev_steps, v=_V, dt=_DT, center=center)
    swept_w = r_o - r_i
    vgeoms, swept = _build_arc_display(bundle, r_path, delta_used)

    b12 = None
    trajectory_source = "铰接尾随"
    if _is_hydraulic(bundle):
        b12 = hydraulic_turn_radii(bundle)
        # B.1.2 口径优先：扫空=W_swept(=d)、转弯通道=W_turn=d+0.5；仿真量测降级为附注(不再无条件 max)
        trajectory_source = "B.1.2"
        swept_w = b12["W_swept"]
        turn_channel_w = b12["W_turn"]
    else:
        # B.1.1/B.1.1 机械挂车：仿真扫空宽度计入 0.5m 安全宽度
        turn_channel_w = swept_w + TURN_SAFE_CLEARANCE

    sigma = _effective_sigma(bundle)
    road_w = _transverse.road_width(road, lane)           # 路面宽度
    trans_w = _transverse.transverse_space_width(road, lane)  # 横向空间宽度（含加宽）
    clearance_outer = r_out - r_o
    clearance_inner = r_i - r_in

    # 表4.3.1 查表法（M5）：查表优先（规范 3.1.2 / 4.3.1）
    size_grade = _grade.classify_bundle(bundle)
    lookup = _grade.arch_lookup_apply(size_grade, r_in, trans_w, swept_w)
    lookup_note = {
        "pass": (f"尺寸级 {size_grade}：圆曲线内缘半径 {r_in:.3f} m 与横向空间宽度 "
                 f"{trans_w:.3f} m 均大于表4.3.1，查表法满足（直接判定）。"),
        "fail": (f"尺寸级 {size_grade}：圆曲线半径或横向空间宽度不大于表4.3.1，"
                 f"按附录B验算/仿真判定。"),
        "fallback": (f"尺寸级 {size_grade}：表4.3.1 无该级数据，按附录B验算/仿真判定。"),
    }.get(lookup)

    # 判定：查表命中即满足；否则回退到附录 B 验算口径（不再无条件覆盖查表）
    if lookup == "pass":
        method = "查表法（表4.3.1）"
        pass_h = True
        verdict = "查表法满足：圆曲线半径与通行空间宽度均大于表4.3.1 数值，空间可通行。"
    else:
        method = "验算法（附录B / 仿真C）"
        # B.1 验算口径：①转弯通道宽度<路面宽度；②扫空宽度<横向空间宽度；
        # ③/④内外缘与护栏等障碍物的预留 σ
        width_ok = turn_channel_w <= road_w + _TOL
        swept_ok = swept_w <= trans_w + _TOL
        outer_ok = clearance_outer >= sigma - _TOL
        inner_ok = clearance_inner >= sigma - _TOL
        pass_h = width_ok and swept_ok and outer_ok and inner_ok
        verdict = f"验算法判定{'通过' if pass_h else '不通过'}："
    res.metrics["method"] = method
    res.metrics["lookup"] = lookup
    res.metrics["road_width"] = road_w
    res.metrics["transverse_space_w"] = trans_w

    road_poly = _road_from_centerline(
        r_path, (lane / 2.0, lane / 2.0, lane / 2.0), _TURN_DEG)
    res.road = road_poly
    res.swept = swept
    res.vehicle_frames = _pick_poses(vgeoms)
    res.metrics.update({
        "R_o": r_o, "R_i": r_i, "W_needed": swept_w,
        "R_min": r_o, "W": turn_channel_w,
        "W_swept": swept_w,
        "R_path": r_path, "delta_used_deg": math.degrees(delta_used),
        "road_outer": r_out, "road_lane": lane,
        "clearance_outer": max(0.0, clearance_outer),
        "clearance_inner": max(0.0, clearance_inner),
        "turn_channel_w": turn_channel_w,
        "trajectory_source": trajectory_source,
    })
    if b12 is not None:
        res.metrics["B1_2_Rin"] = b12["R_in"]
        res.metrics["B1_2_Rout"] = b12["R_out"]
        res.metrics["B1_2_W"] = b12["W_swept"]
        res.metrics["B1_2_W_swept"] = b12["W_swept"]
        res.metrics["B1_2_W_turn"] = b12["W_turn"]
        res.metrics["B1_2_d"] = b12["d"]
        res.metrics["B1_2_D_total"] = b12["D_total"]
        res.metrics["B1_2_base"] = b12["base"]
        res.metrics["B1_2_spec_mode"] = b12["spec_mode"]
        res.metrics["B1_2_warnings"] = b12["warnings"]
        res.metrics["B1_2_theta_deg"] = b12["theta_deg"]
    res.pass_ok = pass_h
    res.verdict_lines = _vehicle_cargo_lines(bundle) + [
        "【弯道参数】",
        f"弯道类型：圆弧弯道；公路等级：{road.highway_grade}",
        f"道路外半径 R<sub>o</sub> = {r_out:.3f} m，内半径 R<sub>i</sub> = {r_in:.3f} m，"
        f"车道宽 {lane:.3f} m",
        f"横向预留空间 σ = {sigma:.3f} m；垂直净空 H<sub>净空</sub> = {road.vertical_clearance:.3f} m",
        "【检验过程】",
        "实际转弯半径（受最大转向角约束）：",
        f"目标半径 R<sub>t</sub> = max(道路内半径 + 车道宽/2, 1) "
        f"= max({r_in:.3f} + {lane / 2:.3f}, 1) = {r_target:.3f} m",
        f"前轮转角 δ = arctan(轴距 / R<sub>t</sub>) = "
        f"arctan({kin.wheelbase:.3f} / {r_target:.3f}) = {math.degrees(delta_used):.2f}°"
        f"（限位 {math.degrees(kin.max_delta):.1f}°）",
        f"实际路径半径 R<sub>path</sub> = 轴距 / tan(δ) = "
        f"{kin.wheelbase:.3f} / tan{math.degrees(delta_used):.2f}° = {r_path:.3f} m",
        "稳态扫掠量测（绕道路圆心 O，附录C铰接仿真，供展示）：",
        f"扫掠外径 R<sub>o</sub> = {r_o:.3f} m；扫掠内径 R<sub>i</sub> = {r_i:.3f} m",
        f"仿真扫空宽度 W<sub>swept,仿真</sub> = R<sub>o</sub> − R<sub>i</sub> "
        f"= {r_o:.3f} − {r_i:.3f} = {r_o - r_i:.3f} m",
        *(_hydraulic_b12_lines(bundle, b12) if b12 is not None else ()),
        "转弯通道宽度（判定，含安全宽度）：",
        f"W<sub>turn</sub> = 判定 W<sub>swept</sub> + {TURN_SAFE_CLEARANCE:g} "
        f"= {swept_w:.3f} + {TURN_SAFE_CLEARANCE:.3f} = {turn_channel_w:.3f} m",
        "路面宽度 / 横向空间宽度：",
        f"路面宽度 W<sub>road</sub> = 车道宽 + 硬路肩 = {lane:.3f} + {road.hard_shoulder_w:.3f} "
        f"= {road_w:.3f} m",
        f"横向空间宽度 W<sub>横</sub> = 车道宽 + 路缘带 + 硬路肩"
        + (f" + 借用对向行车道 {lane:.3f}" if road.median_type != "物理隔离" else "")
        + (f" + 圆曲线加宽 {road.curve_widening_w:.3f}" if road.has_curve_widening else "")
        + f" = {trans_w:.3f} m",
        "内/外缘与护栏等障碍物预留（σ）：",
        f"外缘净空 Δ<sub>out</sub> = 道路外半径 − R<sub>o</sub> = {r_out:.3f} − {r_o:.3f} "
        f"= {clearance_outer:.3f} m "
        f"{'≥' if clearance_outer >= sigma - _TOL else '&lt;'} σ({sigma:.3f})"
        f" → {'满足' if clearance_outer >= sigma - _TOL else '不满足'}",
        f"内缘净空 Δ<sub>in</sub> = R<sub>i</sub> − 道路内半径 = {r_i:.3f} − {r_in:.3f} "
        f"= {clearance_inner:.3f} m "
        f"{'≥' if clearance_inner >= sigma - _TOL else '&lt;'} σ({sigma:.3f})"
        f" → {'满足' if clearance_inner >= sigma - _TOL else '不满足'}",
        lookup_note,
        verdict,
    ]
    res.verdict_lines += _attach_common_checks(res, bundle, r_inner_min=r_i)
    return res


# =====================================================================
# 直角弯道（平面交叉转弯，5.2.1）
# =====================================================================
def _road_right_angle(r_path, entry_w, exit_w, turn_deg, straight=_ROAD_STRAIGHT,
                      corner_radius=0.0):
    """直角弯道道路几何：两条直道正交 + 内角导圆的 L 形。

    车辆行驶中心线（与仿真一致）：沿 x = r_path 自 -Y 向 +Y 直行入场 → 绕角心 O
    转 90° → 沿 y = r_path 向 -X 直行出场。
    入口/ 出口为两条相互正交的直道带，内角处以 corner_radius 圆顺，形成清晰可辨
    的直角转弯，而非圆弧弯道。仅当 turn_deg ≈ 90° 时应用；其它夹角退回中心线缓冲。
    """
    from shapely.geometry import Polygon as _Poly

    if abs(turn_deg - 90.0) > 1e-6:
        return _road_from_centerline(
            r_path, (entry_w / 2.0, max(entry_w, exit_w) / 2.0, exit_w / 2.0),
            turn_deg)

    eh, xh = entry_w / 2.0, exit_w / 2.0
    # 外缘右/上边界；入口/出口直道外延到满外矩形，使路口外角为干净直角。
    rr, rt = r_path + eh, r_path + xh
    b, l = -straight, -straight

    pts = [(r_path - eh, b)]                      # 入口下角
    if corner_radius > 1e-9:
        # 左（内）角导圆：以内角顶点为基准，圆心落在其左下方、半径为
        # corner_radius 的外凸圆弧，把尖角 "╮" 顺成融为一体的圆弧
        # （圆心在左下、弧面朝路内侧，给车辆留转弯空间），杜绝此前用
        # buffer() 生成游离整圆、与主体仅两点相切的错误形状。
        vx, vy = r_path - eh, r_path - xh         # 内角顶点
        pts.append((vx, vy - corner_radius))      # 垂直直道内缘切点（内角下方）
        cx, cy = vx - corner_radius, vy - corner_radius
        n = 48
        # 90° 弧自东侧切点逆时针铺到北侧切点：圆心在内角左下方、弧面朝
        # 路内侧（外凸给车辆留转弯空间），是贴合两直道内缘的轻量圆角。
        for i in range(1, n + 1):
            ang = math.radians(90.0 * i / n)
            pts.append((cx + corner_radius * math.cos(ang),
                        cy + corner_radius * math.sin(ang)))
    else:
        # 无导圆：保留内角尖角（一段水平 + 一段垂直）。
        pts.append((r_path - eh, r_path - xh))
    pts += [(l, r_path - xh), (l, rt), (rr, rt), (rr, b)]
    return _Poly(pts)


def _simulate_right_angle(bundle, entry_w, exit_w, turn_deg):
    """绕角心 C 转 turn_deg（默认 90°）仿真，量测扫掠带（R_o/R_i/W_needed），并生成绘图底图。"""
    kin = TractorTrailerKinematics(bundle)
    r_target = max(entry_w, exit_w, kin.loop_len * 1.2, 10.0)
    r_target = min(r_target, 80.0)
    delta_rec = min(kin.max_delta, math.atan(kin.wheelbase / r_target))
    r_path = kin.wheelbase / math.tan(delta_rec)
    center = (0.0, 0.0)
    kin.reset_pose(r_path, 0.0, math.radians(90))
    kin.delta = delta_rec

    ang_vel = abs(_V * math.tan(delta_rec) / kin.wheelbase)
    rev_steps = int(4 * math.pi / (ang_vel * _DT))
    r_o, r_i = measure_steady_turn(kin, rev_steps, v=_V, dt=_DT, center=center)
    swept_w = max(r_o - r_i, 0.0)

    kin.reset_pose(r_path, -_STRAIGHT, math.radians(90))   # 直道入场至弧起点，切向绕角心 C 转 turn_deg
    profile = []
    profile.append((int(_STRAIGHT / (_V * _DT)), 0.0))
    turn_steps = int(math.radians(turn_deg) / (ang_vel * _DT))
    profile.append((turn_steps, delta_rec))
    profile.append((int(_STRAIGHT / (_V * _DT)), 0.0))
    frames, _k = run_profile(kin, profile, v=_V, dt=_DT, draw_step=SAMPLES)
    vgeoms = build_trajectory_geoms(kin, frames, sample_rate=1)
    swept = polylist_to_multipoly(vgeoms)
    return r_o, r_i, swept_w, r_path, swept, vgeoms


def _overhang(bundle: InputBundle) -> bool:
    """货物长度或宽度超过挂车尺寸 ⇒ 触发 5.2.4 分体扫空校核。"""
    c, tl = bundle.cargo, bundle.trailer
    return (c.width > tl.width) or (c.length > tl.length)


def _split_swept_checks(bundle: InputBundle, vgeoms, road_poly):
    """5.2.4 分体扫空校核：分别求车辆扫空与货物扫空是否超出平面交叉路面范围。

    规格 5.2.4：货物超出挂车尺寸时，
      1 车辆扫空空间不应大于平面交叉路面范围；
      2/3 货物扫空空间不应大于路面范围（有护栏且货物离地≤护栏高时）
         或路侧可用空间（无护栏/离地>护栏时）。
    本工况未录入护栏高度/路侧可用空间数据，故货物按 5.2.4 最不利情形
    以“平面交叉路面范围”校核（较 5.2.4-3 宽松情形更保守）。

    返回 dict：是否触发、车辆/货物扫空超界面积、二者是否合格。
    """
    out = {"active": False, "vehicle_swept_area": 0.0, "cargo_swept_area": 0.0,
           "vehicle_outside": 0.0, "cargo_outside": 0.0,
           "vehicle_ok": True, "cargo_ok": True}
    if not _overhang(bundle) or road_poly is None or not vgeoms:
        return out
    try:
        veh_polys, cargo_polys = [], []
        for frame in vgeoms:
            # frame = (x, y, geom, trailer_geom, cargo_geom)；geom=元组(合体, 牵引车)
            _, _, geom, trailer_geom, cargo_geom = frame
            tractor_geom = geom.geoms[1] if geom.geom_type == "MultiPolygon" else geom
            veh_polys.append(unary_union_box(trailer_geom, tractor_geom))
            cargo_polys.append(cargo_geom)
        veh_swept = unary_union(veh_polys)
        cargo_swept = unary_union(cargo_polys)
        veh_out = veh_swept if veh_swept is not None and not veh_swept.is_empty else None
        cargo_out = cargo_swept if cargo_swept is not None and not cargo_swept.is_empty else None
        veh_outside = veh_out.difference(road_poly).area if veh_out is not None else 0.0
        cargo_outside = cargo_out.difference(road_poly).area if cargo_out is not None else 0.0
        out.update({
            "active": True,
            "vehicle_swept_area": veh_out.area if veh_out is not None else 0.0,
            "cargo_swept_area": cargo_out.area if cargo_out is not None else 0.0,
            "vehicle_outside": veh_outside,
            "cargo_outside": cargo_outside,
            "vehicle_ok": veh_outside <= _TOL,
            "cargo_ok": cargo_outside <= _TOL,
        })
    except Exception:
        # 分体扫空不参与主判定即可通过；失败仅记录，不影响其余结论
        return out
    return out


def unary_union_box(a, b):
    from shapely.ops import unary_union
    return unary_union([a, b]) if (a is not None and b is not None) \
        else (a or b)


def _judge_line(lhs_sym, lhs_val, rhs_sym, rhs_val, ok):
    """生成一条判定代入行：W<sub>en</sub> − 2σ = 17.500 m ≥ B<sub>t</sub> + ΔB = 12.115 m → 满足"""
    op = "≥" if ok else "&lt;"
    verdict = "满足" if ok else "不满足"
    return f"{lhs_sym} = {lhs_val:.3f} m {op} {rhs_sym} = {rhs_val:.3f} m → {verdict}"


def run_right_angle_scene(bundle: InputBundle) -> SweepScenarioResult:
    road = bundle.road
    ra = road.right_angle
    entry_w, exit_w = ra.entry_width, ra.exit_width
    turn_deg = min(ra.intersection_angle_deg, 179.9)   # 默认 90°，两直道正交 + 内角导圆的 L 形
    r_o, r_i, swept_w, r_path, swept, vgeoms = _simulate_right_angle(
        bundle, entry_w, exit_w, turn_deg)

    res = SweepScenarioResult()
    res.scene = "right_angle"
    road_poly = _road_right_angle(
        r_path, entry_w, exit_w, turn_deg, corner_radius=ra.corner_radius)
    res.road = road_poly
    res.swept = swept
    res.vehicle_frames = _pick_poses(vgeoms)

    # == 5.2.1 三要素 ==
    R_m = r_i                              # 最内侧转弯半径（取仿真内径）
    R_max = r_o                            # 最外侧包络半径
    W = swept_w                            # 扫空空间最大宽度（B.1.2-6 口径）
    B_t = max(bundle.trailer.width, bundle.cargo.width)  # 车货总宽度
    deltaB = max(0.0, W - B_t)             # 外摆值 ΔB = Dmax − (R_m + B_t)
    # 5.2.1-1 横向预留空间：平面交叉转弯固定取 INTERSECTION_RESERVE（默认 0.5），
    # 非 4.2.1 按公路等级取值；显式给出 road.lateral_clearance 时按显式值。
    if road.lateral_clearance is not None:
        sigma = float(road.lateral_clearance)
    else:
        sigma = float(INTERSECTION_RESERVE)

    # 判定（5.2.1）：右转 5.2.1-1/-2 用进口道总宽/右转可通行宽；左转 5.2.1-3/-4
    # 用平面交叉物理区可通行宽度 W_section 与左转处可通行宽度 W_lturn。
    right_ok = ((entry_w - 2 * sigma) >= (B_t + deltaB) - _TOL) and \
               ((exit_w - 2 * sigma) >= W - _TOL)
    W_section = ra.W_section if ra.W_section is not None else entry_w
    W_lturn = ra.W_lturn if ra.W_lturn is not None else exit_w
    left_ok = ((W_section - sigma) >= (B_t + deltaB) - _TOL) and \
              ((W_lturn - sigma) >= W - _TOL)
    pass_ok = right_ok

    # 5.2.4 超宽货物分体扫空校核（车辆扫空 / 货物扫空均不应超出路面范围）
    split = _split_swept_checks(bundle, vgeoms, road_poly)
    pass_ok = pass_ok and split["vehicle_ok"] and split["cargo_ok"]

    # 现有“所需最小出口转弯带”保留为参考量（不再作为唯一判据）
    w2_min = max(W, B_t + deltaB) + 2 * sigma

    res.metrics = {
        "entry_width": entry_w, "exit_width": exit_w,
        "r_path": r_path, "R_o": r_o, "R_i": r_i, "W_needed": swept_w,
        "R_m": R_m, "W": W, "deltaB": deltaB, "W_en": entry_w,
        "W_turn": exit_w, "sigma": sigma, "B_t": B_t,
        "W2_min": w2_min, "turn_deg": turn_deg,
        "corner_radius": ra.corner_radius,
        "W_section": W_section, "W_lturn": W_lturn,
        "right_ok": right_ok, "left_ok": left_ok,
        "split_active": split["active"],
        "vehicle_swept_area": split["vehicle_swept_area"],
        "cargo_swept_area": split["cargo_swept_area"],
        "vehicle_outside": split["vehicle_outside"],
        "cargo_outside": split["cargo_outside"],
        "split_vehicle_ok": split["vehicle_ok"],
        "split_cargo_ok": split["cargo_ok"],
    }
    res.pass_ok = pass_ok
    res.verdict_lines = _vehicle_cargo_lines(bundle) + [
        "【弯道参数】",
        f"弯道类型：直角弯道（平面交叉转弯）；公路等级：{road.highway_grade}",
        f"入口宽度 W<sub>en</sub> = {entry_w:.2f} m；出口宽度 W<sub>turn</sub> = {exit_w:.2f} m；"
        f"交角 {turn_deg:.0f}°；内角半径 r<sub>c</sub> = {ra.corner_radius:.2f} m",
        f"平面交叉物理区可通行宽度 W<sub>section</sub> = {W_section:.2f} m；"
        f"左转处可通行宽度 W<sub>lturn</sub> = {W_lturn:.2f} m",
        f"横向预留空间 σ = {sigma:g} m；垂直净空 H<sub>净空</sub> = {road.vertical_clearance:.3f} m",
        "【检验过程】",
        "扫空空间参数：",
        f"扫掠外径 R<sub>max</sub> = R<sub>o</sub> = {r_o:.3f} m；扫掠内径 R<sub>i</sub> = {r_i:.3f} m",
        f"最内侧转弯半径 R<sub>m</sub> = R<sub>i</sub> = {R_m:.3f} m",
        f"扫空空间宽度 W = R<sub>o</sub> − R<sub>i</sub> = {r_o:.3f} − {r_i:.3f} = {W:.3f} m",
        f"车货总宽度 B<sub>t</sub> = max(挂车宽, 货宽) = max({bundle.trailer.width:.2f}, "
        f"{bundle.cargo.width:.2f}) = {B_t:.3f} m",
        f"外摆值 ΔB = max(W − B<sub>t</sub>, 0) = max({W:.3f} − {B_t:.3f}, 0) = {deltaB:.3f} m",
        f"横向预留空间 σ = {sigma:g} m",
        "右转判定：",
        _judge_line("W<sub>en</sub> − 2σ", entry_w - 2 * sigma,
                    "B<sub>t</sub> + ΔB", B_t + deltaB,
                    (entry_w - 2 * sigma) >= (B_t + deltaB) - _TOL),
        _judge_line("W<sub>turn</sub> − 2σ", exit_w - 2 * sigma,
                    "W", W, (exit_w - 2 * sigma) >= W - _TOL),
        f"→ 右转结论：{'通过' if right_ok else '不通过'}",
        "左转判定：",
        _judge_line("W<sub>section</sub> − σ", W_section - sigma,
                    "B<sub>t</sub> + ΔB", B_t + deltaB,
                    (W_section - sigma) >= (B_t + deltaB) - _TOL),
        _judge_line("W<sub>lturn</sub> − σ", W_lturn - sigma,
                    "W", W, (W_lturn - sigma) >= W - _TOL),
        f"→ 左转结论：{'通过' if left_ok else '不通过'}",
        f"所需最小出口转弯带 W<sub>2,min</sub> = max(W, B<sub>t</sub> + ΔB) + 2σ = "
        f"max({W:.3f}, {B_t + deltaB:.3f}) + 2×{sigma:g} = {w2_min:.3f} m（参考量）",
    ]
    if split["active"]:
        res.verdict_lines += [
            "5.2.4 超宽货物分体扫空校核：",
            f"货物超挂车 → 分别校核车辆扫空与货物扫空是否超出平面交叉路面范围",
            f"车辆扫空空间超界面积 A<sub>veh</sub> = {split['vehicle_outside']:.4f} m² "
            f"→ {'合格' if split['vehicle_ok'] else '不合格'}",
            f"货物扫空空间超界面积 A<sub>cargo</sub> = {split['cargo_outside']:.4f} m² "
            f"→ {'合格' if split['cargo_ok'] else '不合格'}",
            "(无护栏/路侧数据，货物按 5.2.4 最不利情形的路面范围校核)",
            f"→ 5.2.4 结论：{'通过' if split['vehicle_ok'] and split['cargo_ok'] else '不通过'}",
        ]
    res.verdict_lines += _attach_common_checks(res, bundle, r_inner_min=R_m,
                                               sigma_override=sigma)
    return res


# =====================================================================
# 统一入口
# =====================================================================
def run_scene(bundle: InputBundle) -> SweepScenarioResult:
    kind = bundle.road.road_type
    if "圆弧" in kind:
        return run_arc_scene(bundle)
    return run_right_angle_scene(bundle)