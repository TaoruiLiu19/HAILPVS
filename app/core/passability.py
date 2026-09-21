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
                    VERT_CLEARANCE_MARGIN, INTERSECTION_RESERVE,
                    HIGHWAY_GRADES, DEFAULT_SUPERELEVATION, PAD_FRICTION)
from .datamodel import InputBundle
from .kinematics import (TractorTrailerKinematics, run_profile,
                         build_trajectory_geoms, hydraulic_turn_radii)
from .swept_path import measure_steady_turn, polylist_to_multipoly
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
    """均匀挑选 n 个车辆姿态用于绘图（沿轨迹首尾+中间，减少残留重叠）。"""
    step = max(1, len(vgeoms) // max(n, 1))
    chosen = vgeoms[::step]
    # 保证总帧数 ≤ n（不足 n 则全取）
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
        f"车货总宽 B_t = {B_t:.3f} m，车货总高 = {_cargo_total_height(bundle):.3f} m",
    ]


def _vertical_check(bundle: InputBundle):
    """垂直净空校验（M7）：车货总高 + 附加余量 ≤ 净空高度。"""
    total_h = _cargo_total_height(bundle)
    limit = bundle.road.vertical_clearance
    margin = max(0.0, VERT_CLEARANCE_MARGIN)
    ok = (total_h + margin) <= limit + _TOL
    msg = (f"垂直净空：车货总高 {total_h:.3f} m + 余量 {margin:.3f} m "
           f"{'≤' if ok else '>'} 净空 {limit:.3f} m。")
    return ok, total_h, margin, msg


def _lateral_check(bundle: InputBundle, r_min: float):
    """横向稳定性（M8，附录B.2）。r_min 取最内侧转弯半径。"""
    road = bundle.road
    tl = bundle.trailer
    cargo = bundle.cargo
    v = bundle.max_speed_kmh
    mu = PAD_FRICTION.get(cargo.cargo_bearing, 0.4)
    cg_h = tl.deck_height + cargo.cg_height
    ih = road.superelevation
    ok, vh, vch, vcv, p_ok, msg = _lateral.lateral_stability(
        r_min, v, tl.track_width, cg_h, ih, mu)
    return {
        "ok": ok, "Vhmax": vh, "Vch": vch, "Vcv": vcv,
        "parking_ok": p_ok, "msg": msg,
        "mu": mu, "cg_height": cg_h, "superelevation": ih,
    }


def _attach_common_checks(res: SweepScenarioResult, bundle: InputBundle,
                          r_inner_min: float) -> list[str]:
    """把公路等级、垂直净空、横向稳定性并入结果与判定，返回“理论分析”文本行。"""
    sigma = _effective_sigma(bundle)
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
    return [vmsg, f"大件运输尺寸分级：{size_grade}（表3.2.4）。", lat["msg"]]


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
    if _is_hydraulic(bundle):
        b12 = hydraulic_turn_radii(bundle)
        swept_w = max(swept_w, b12["W"])     # 扫空宽度取 B.1.2-6 与仿真的较大值

    sigma = _effective_sigma(bundle)
    # M1: 转弯通道宽度计入 0.5m 安全宽度；M3: 判据扣除两侧预留空间 σ
    turn_channel_w = swept_w + TURN_SAFE_CLEARANCE       # B.1.1-1/B.1.2-1 口径
    lane_avail = lane - 2.0 * sigma                      # 路面净宽（扣除两侧预留）
    trans_width = lane                                   # 横向空间宽度（无加宽）
    clearance_outer = r_out - r_o
    clearance_inner = r_i - r_in

    # M2: 内缘约束 + M3: 双侧预留
    outer_ok = clearance_outer >= sigma - _TOL
    inner_ok = clearance_inner >= sigma - _TOL
    width_ok = turn_channel_w <= lane_avail + _TOL       # 4.3.1-1）
    swept_ok = swept_w <= trans_width - 2.0 * sigma + _TOL  # 4.3.1-2）

    # 表4.3.1 查表法半径判据（M5 辅助）
    size_grade = _grade.classify_bundle(bundle)
    lookup = _grade.arch_lookup_apply(size_grade, r_in)
    lookup_note = ""
    if lookup is True:
        lookup_note = f"尺寸级 {size_grade}：圆曲线内缘半径 {r_in:.3f} m > 表4.3.1 最小半径，查表法满足。"
    elif lookup is False:
        lookup_note = (f"尺寸级 {size_grade}：圆曲线内缘半径 {r_in:.3f} m 不大于表4.3.1 "
                       f"最小半径，需按附录B验算/仿真（本系统已按验算口径判定）。")
    else:
        lookup_note = f"尺寸级 {size_grade}：表4.3.1 无半径数据，按附录B验算判定。"

    pass_ok = outer_ok and inner_ok and width_ok and swept_ok

    road_poly = _road_from_centerline(
        r_path, (lane / 2.0, lane / 2.0, lane / 2.0), _TURN_DEG)
    res.road = road_poly
    res.swept = swept
    res.vehicle_frames = _pick_poses(vgeoms)
    res.metrics = {
        "R_o": r_o, "R_i": r_i, "W_needed": swept_w,
        "R_min": r_o, "W": turn_channel_w,
        "W_swept": swept_w,
        "R_path": r_path, "delta_used_deg": math.degrees(delta_used),
        "road_outer": r_out, "road_lane": lane,
        "clearance_outer": max(0.0, clearance_outer),
        "clearance_inner": max(0.0, clearance_inner),
        "turn_channel_w": turn_channel_w,
        "lane_avail": lane_avail,
    }
    if b12 is not None:
        res.metrics["B1_2_Rin"] = b12["R_in"]
        res.metrics["B1_2_Rout"] = b12["R_out"]
        res.metrics["B1_2_W"] = b12["W"]
        res.metrics["B1_2_theta_deg"] = b12["theta_deg"]
    res.pass_ok = pass_ok
    res.verdict_lines = _vehicle_cargo_lines(bundle) + [
        "【弯道参数】",
        f"弯道类型：圆弧弯道；公路等级：{road.highway_grade}",
        f"道路外半径 R_o = {r_out:.3f} m，内半径 R_i = {r_in:.3f} m，车道宽 {lane:.3f} m",
        f"横向预留空间 σ = {sigma:.3f} m；垂直净空 {road.vertical_clearance:.3f} m；"
        f"超高 {road.superelevation:.3f}",
        "【理论分析】",
        f"车辆所需最小弯道外半径 R_min = {r_o:.3f} m（道路外半径 {r_out:.3f} m）",
        f"车辆扫掠外径 R_o = {r_o:.3f} m，内径 R_i = {r_i:.3f} m",
        f"转弯通道宽度（含 {TURN_SAFE_CLEARANCE:g} m 安全宽度）W = {turn_channel_w:.3f} m"
        f"（扫空宽度 {swept_w:.3f} m；路面净宽 {lane_avail:.3f} m）",
        f"外缘净空 = {clearance_outer:.3f} m，内缘净空 = {clearance_inner:.3f} m"
        f"（两侧预留 σ = {sigma:.3f} m）",
        lookup_note,
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
    sigma = INTERSECTION_RESERVE           # 5.2.1 可取 0.5m

    # 判定（默认右转弯，5.2.1-1/-2；左转弯 5.2.1-3/-4 一并计算供参考）
    right_ok = ((entry_w - 2 * sigma) >= (B_t + deltaB) - _TOL) and \
               ((exit_w - 2 * sigma) >= W - _TOL)
    left_ok = ((entry_w - sigma) >= (B_t + deltaB) - _TOL) and \
              ((exit_w - sigma) >= W - _TOL)
    pass_ok = right_ok

    # 现有“所需最小出口转弯带”保留为参考量（不再作为唯一判据）
    w2_min = max(W, B_t + deltaB) + 2 * sigma

    res.metrics = {
        "entry_width": entry_w, "exit_width": exit_w,
        "r_path": r_path, "R_o": r_o, "R_i": r_i, "W_needed": swept_w,
        "R_m": R_m, "W": W, "deltaB": deltaB, "W_en": entry_w,
        "W_turn": exit_w, "sigma": sigma, "B_t": B_t,
        "W2_min": w2_min, "turn_deg": turn_deg,
    }
    res.pass_ok = pass_ok
    res.verdict_lines = _vehicle_cargo_lines(bundle) + [
        "【弯道参数】",
        f"弯道类型：直角弯道（平面交叉转弯）；公路等级：{road.highway_grade}",
        f"入口宽度 W_en = {entry_w:.2f} m，出口宽度 W_turn = {exit_w:.2f} m，"
        f"内角半径 = {ra.corner_radius:.2f} m",
        f"横向预留空间 σ = {sigma:g} m；垂直净空 {road.vertical_clearance:.3f} m；"
        f"超高 {road.superelevation:.3f}",
        "【理论分析】",
        f"最内侧转弯半径 R_m = {R_m:.3f} m，扫空空间最大宽度 W = {W:.3f} m",
        f"车货总宽度 B_t = {B_t:.3f} m，外摆值 ΔB = {deltaB:.3f} m",
        f"右转判定(5.2.1)：W_en−2σ({entry_w - 2 * sigma:.3f}) ≥ B_t+ΔB"
        f"({B_t + deltaB:.3f}) 且 W_turn−2σ({exit_w - 2 * sigma:.3f}) ≥ W({W:.3f})",
        f"所需最小出口转弯带 W2_min ≈ {w2_min:.3f} m（参考量）",
    ]
    res.verdict_lines += _attach_common_checks(res, bundle, r_inner_min=R_m)
    return res


# =====================================================================
# 统一入口
# =====================================================================
def run_scene(bundle: InputBundle) -> SweepScenarioResult:
    kind = bundle.road.road_type
    if "圆弧" in kind:
        return run_arc_scene(bundle)
    return run_right_angle_scene(bundle)