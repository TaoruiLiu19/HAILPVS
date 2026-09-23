"""多刚体运动学模型：牵引车(单车模型) + 挂车(铰接约束)。

采用经典铰接单车模型：
  牵引车后轴中心为状态点 (x, y, θ)，转向角 δ。
  挂车与牵引车在鞍座(第五轮)处铰接，挂车航向 ψ 由铰接点速度约束更新：
    ψ̇ = v · sin(θ - ψ) / L_trailer_loop
  挂车参考后点 = 鞍座 - L_loop·(cosψ, sinψ)。
"""
from __future__ import annotations
import itertools
import math

from config import (MIN_TURN_ANGLE_DEG, AXLE_STEER_ABS_LIMIT_DEG,
                    TURN_SAFE_CLEARANCE_B12)
from .datamodel import InputBundle


def _rect_into(cx, cy, angle, length, width, pts_out):
    """把以 (cx,cy) 为中心、orientation=angle、长为 length、宽为 width 的
    矩形四角(世界坐标)追加到 pts_out。"""
    ca, sa = math.cos(angle), math.sin(angle)
    hx, hy = length / 2.0, width / 2.0
    corners = [(-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy)]
    for lx, ly in corners:
        wx = cx + lx * ca - ly * sa
        wy = cy + lx * sa + ly * ca
        pts_out.append((wx, wy))


class TractorTrailerKinematics:
    """牵引车(单车模型) + 挂车(尾随点追逐)。

    挂车参考点(车尾/尾随点)与鞍座间为刚性连杆，尾随点速度 = 牵引销速度
    沿连杆方向的投影(无横向滑动)。该模型天然收敛，不会像纯 psi 积分那样
    在定常转弯中振荡。
    """
    def __init__(self, bundle: InputBundle):
        self.b = bundle
        tr = bundle.tractor
        tl = bundle.trailer
        self.wheelbase = tr.wheelbase
        self.max_delta = math.radians(tr.max_steering_angle_deg)
        self.fifth_wheel = tr.fifth_wheel_pos
        self.loop_len = max(tl.length, 1e-6)
        self.env_w = max(tl.width, bundle.cargo.width)
        self.body_len = max(tl.length, bundle.cargo.length)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.tx = 0.0      # 挂车尾随点
        self.ty = 0.0
        self.v = 0.0
        self.delta = 0.0

    def reset_pose(self, x=0.0, y=0.0, theta=0.0, psi=None):
        self.x, self.y, self.theta = x, y, theta
        if psi is None:
            psi = theta
        # 尾随点在连杆延长线上，初始与牵引车同向
        self.tx, self.ty = self._tail_along(psi)
        self.delta = 0.0

    def _tail_along(self, psi):
        kx, ky = self.kingpin()
        return (kx - self.loop_len * math.cos(psi),
                ky - self.loop_len * math.sin(psi))

    def kingpin(self):
        return (self.x + self.fifth_wheel * math.cos(self.theta),
                self.y + self.fifth_wheel * math.sin(self.theta))

    def trailer_tail(self):
        return (self.tx, self.ty)

    def psi(self):
        """挂车航向：由尾随点→鞍座方向确定。"""
        kx, ky = self.kingpin()
        return math.atan2(ky - self.ty, kx - self.tx)

    def footprint_poly(self, pts_out):
        """把整车(牵引车+挂车/货物)当前外轮廓矩形(4~8个顶点)追加进 pts_out。"""
        kx, ky = self.kingpin()
        px, py = self.trailer_tail()
        ps = self.psi()
        _rect_into((kx + px) / 2, (ky + py) / 2, ps, self.body_len,
                   self.env_w, pts_out)
        _rect_into(self.x, self.y, self.theta, self.wheelbase + 1.0,
                   self.b.tractor.width, pts_out)

    def update_step(self, delta, dt):
        self.delta = max(-self.max_delta, min(self.max_delta, delta))
        v = self.v
        if abs(self.delta) < 1e-9:
            self.x += v * math.cos(self.theta) * dt
            self.y += v * math.sin(self.theta) * dt
        else:
            ang_vel = v * math.tan(self.delta) / self.wheelbase
            self.theta += ang_vel * dt
            self.x += v * math.cos(self.theta) * dt
            self.y += v * math.sin(self.theta) * dt
        # 尾随点追逐：速度 = 牵引销速度沿连杆方向投影(无横向滑动)
        kx, ky = self.kingpin()
        dx, dy = kx - self.tx, ky - self.ty
        dist = math.hypot(dx, dy)
        if dist > 1e-12:
            ux, uy = dx / dist, dy / dist
            vx = v * math.cos(self.theta)
            vy = v * math.sin(self.theta)
            proj = vx * ux + vy * uy
            self.tx += proj * ux * dt
            self.ty += proj * uy * dt
            # 刚性长度约束：重投影到链路长度
            dx, dy = kx - self.tx, ky - self.ty
            d = math.hypot(dx, dy)
            if d > 1e-12:
                self.tx = kx - self.loop_len * dx / d
                self.ty = ky - self.loop_len * dy / d
        return True


def run_profile(kin: TractorTrailerKinematics, profile, v=2.0, dt=0.05,
                draw_step=1, max_steps=40000):
    """按转向角时序推演(profile: [(count, delta_rad), ...])。

    返回 (frames, kin)。frames 为采样帧，每帧 (x,y,theta,tx,ty)。
    """
    frames = []
    kin.v = v
    step = 0
    for count, delta in profile:
        for _ in itertools.repeat(None, count):
            if step >= max_steps:
                return frames, kin
            kin.update_step(delta, dt)
            if step % draw_step == 0:
                frames.append((kin.x, kin.y, kin.theta, kin.tx, kin.ty))
            step += 1
    return frames, kin


def build_trajectory_geoms(kin: TractorTrailerKinematics, frames,
                           sample_rate=1):
    """把帧序列恢复为完整位姿，逐帧构建外轮廓几何。

    返回 [(x, y, geom, trailer_geom, cargo_geom), ...]：
      x/y            帧参考点（牵引车状态点，抽帧绘制定位）；
      geom           整车扫掠几何（挂车+货物合并矩形 ∪ 牵引车矩形，用于包络计算）；
      trailer_geom   挂车车板独立矩形（绘制：蓝色车架）；
      cargo_geom     货物独立矩形（绘制：超宽/超长时黑色轮廓凸出车板可见）。
    """
    import copy

    from shapely.geometry import Polygon, MultiPolygon

    polys = []
    k = copy.copy(kin)
    for i, (x0, y0, th0, tx0, ty0) in enumerate(frames):
        if i % sample_rate != 0:
            continue
        k.x, k.y, k.theta = x0, y0, th0
        k.tx, k.ty = tx0, ty0
        # 牵引车与挂车各成一个矩形，避免帧内自交
        from . import kinematics as _km
        kx, ky = k.kingpin()
        px, py = k.trailer_tail()
        ps = k.psi()
        cx, cy = (kx + px) / 2, (ky + py) / 2
        env_pts, tr_pts, cg_pts, tb_pts = [], [], [], []
        _km._rect_into(cx, cy, ps, k.body_len, k.env_w, env_pts)          # 挂车+货物合并
        _km._rect_into(cx, cy, ps, k.b.trailer.length, k.b.trailer.width,
                       tr_pts)                                            # 挂车车板
        _km._rect_into(cx, cy, ps, k.b.cargo.length, k.b.cargo.width,
                       cg_pts)                                            # 货物
        _km._rect_into(k.x, k.y, k.theta, k.wheelbase + 1.0,
                       k.b.tractor.width, tb_pts)                         # 牵引车
        geom = MultiPolygon([Polygon(env_pts), Polygon(tb_pts)])
        polys.append((x0, y0, geom, Polygon(tr_pts), Polygon(cg_pts)))
    return polys


# =====================================================================
# B.1.2 液压轴线挂车（多轴多轮液压悬挂挂车）闭式解
# =====================================================================
def hydraulic_turn_radii(bundle: InputBundle, turn_angle_theta=None):
    """按规范附录 B.1.2 闭式解求液压悬挂挂车的转弯半径、扫空宽度与转弯通道宽度。

    中间一轴（或中间两轴）锁死不转向、首尾轴全轮强制转向，各轮绕同一瞬时
    转动中心做圆周运动：
      base  = d/(2·tanθ)                           瞬心横向偏距(≈瞬心半径)
      R_in  = base + (D总−d)/2         式(B.1.2-4)
      R_out = base + (D总+d)/2         式(B.1.2-5)
      W_swept = R_out − R_in = d       式(B.1.2-6)  车货总体扫空空间宽度
      W_turn  = d + 0.5                式(B.1.2-1)  转弯通道宽度(工程锚点)
    式中 d = L轮挂·(断末有效轴距)，L轮挂 为相邻轴线间距(axle_spacing)；
    D总 = 车板长(trailer.length, D3)。θ = 第一轴线内侧轮胎转向角(D2)，
    来源为挂车轴结构最大转向角 axle_max_steering_angle_deg，受工程上限
    AXLE_STEER_ABS_LIMIT_DEG 钳制(D6)，下端受 MIN_TURN_ANGLE_DEG 防除零。

    D1 locked_axles 双口径：默认严格守规范(中间锁死不改变 d)；仅当锁死轴
    含首/末轴时走规范外推，d=L(last_active−first_active) 并告警。
    返回字典：R_in/R_out、W_swept、W_turn、base、theta_deg、d、D_total、
    spec_mode、locked、sweep_center_dist(=base,D5 瞬心偏距)、warnings。
    """
    tl = bundle.trailer
    na = max(int(tl.axle_count), 2)
    L = tl.axle_spacing                       # L轮挂：相邻轴线间距
    # D3/D8: D_total = 车板长(length)，不含牛颈/连接杆耦合段(耦合仅绘图、不入判定)
    d_total = max(float(tl.length), L * (na - 1))

    # D1：首/末轴锁死 → 规范外推；纯中间锁死 → 严格守规范(d 不变)
    locked = sorted({int(a) for a in tl.locked_axles if 1 <= int(a) <= na})
    first, last, spec_mode = 1, na, True
    if (1 in locked) or (na in locked):
        active = [i for i in range(1, na + 1) if i not in locked]
        if active:
            first, last = min(active), max(active)
        spec_mode = False
    d = L * (last - first)
    if d <= 0:                                # active 转向轴<2 等退化：回退规范基准
        d = L * (na - 1)
        spec_mode = True

    # D2/D6：θ 取自挂车轴结构最大转向角，受工程上限与下端 1° 钳制
    theta_max_deg = min(float(tl.axle_max_steering_angle_deg),
                        AXLE_STEER_ABS_LIMIT_DEG)
    theta_deg = (turn_angle_theta if turn_angle_theta is not None
                 else theta_max_deg)
    theta_deg = max(MIN_TURN_ANGLE_DEG, min(theta_deg, AXLE_STEER_ABS_LIMIT_DEG))
    theta = math.radians(theta_deg)

    base = d / (2.0 * math.tan(theta))        # = d/(2·tanθ)，瞬心偏距
    r_in = base + (d_total - d) / 2.0          # B.1.2-4
    r_out = base + (d_total + d) / 2.0         # B.1.2-5
    w_swept = r_out - r_in                     # B.1.2-6 = d
    w_turn = d + TURN_SAFE_CLEARANCE_B12       # B.1.2-1 工程锚点，OCR 待复核

    warnings_ = []
    if not spec_mode:
        warnings_.append(
            f"首/末轴锁死({locked}):d 按 active 首末={d:.3f}m 外推,属规范外推、保守")
    active_count = len([i for i in range(1, na + 1) if i not in locked])
    if active_count < 2:
        warnings_.append("active 转向轴<2,回退规范基准")

    return {
        "R_in": r_in, "R_out": r_out,
        "W_swept": w_swept, "W_turn": w_turn,
        "base": base, "theta_deg": round(theta_deg, 4),
        "d": d, "D_total": d_total,
        "spec_mode": spec_mode, "locked": locked,
        "sweep_center_dist": base,      # D5 瞬心横向偏距(轨迹层用)
        "warnings": warnings_,
    }


def hydraulic_coupling_draw_anchor(bundle: InputBundle, tractor_rear_x,
                                   tractor_rear_y, psi):
    """D8: 仅绘图。返回挂车车板前端锚点(相对牵引车后沿偏移 coupling_length)。

    牵引车后边沿 + 连接部分长度·挂车航向 → 挂车车板前端；车板自前端向后延伸
    length 得到车尾。不参与 D_total/R/W_swept/W_turn 计算，仅影响绘图定位。
    """
    tl = bundle.trailer
    off = float(tl.coupling_length or 0.0)
    return (tractor_rear_x + off * math.cos(psi),
            tractor_rear_y + off * math.sin(psi))