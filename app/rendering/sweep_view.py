"""扫掠轨迹工程图绘制（CAD 线框风格）。

对标参考工程图：
  - 白底、纯线框、无填充、无网格；
  - 道路边界 = 黑色粗实线（主结构）；
  - 车辆轮廓 = 蓝色中实线（构件）；
  - 扫掠包络与半径构造圆弧 = 蓝色细虚线（构造/测量几何）；
  - 转弯圆心 = 红色十字（关键参考点）；
  - 扫掠半径 R_o / R_i 带引出线标注（仿 R13122 / R4465）。
沿用 LasingSystem2.0 输出约定：Agg 后端、等比例、300dpi。
"""
import math

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Arc

from config import PLOT_DPI
from ..core.passability import SweepScenarioResult

# —— CAD 风格配色：白底 / 黑主结构 / 蓝构件与构造线 / 红关键点 ——
ROAD_COLOR = "#111111"        # 道路边界：黑色粗实线
VEHICLE_COLOR = "#111111"     # 车辆轮廓：黑色中实线
CARGO_COLOR = "#2e9e44"       # 货物轮廓：绿色实线（与车辆/道路区分）
SWEPT_COLOR = "#4d8dd0"       # 扫掠包络（整车占道范围）：蓝色细虚线
SWEPT_ARC_COLOR = "#90959a"   # 扫掠曲线（R_o / R_i 两条半径圆弧）：灰色，区别于标注线
DIM_COLOR = "#0d47a1"         # 尺寸/半径/交角等所有标注线统一色：深蓝色（区别于曲线，避开红色）
CENTER_COLOR = "#e02020"      # 关键参考点：红色十字
TEXT_COLOR = "#0d47a1"        # 标注文字：与标注线同色（深蓝）
SPINE_COLOR = "#b8c0c8"       # 坐标轴细线：浅灰
AXLE_COLOR = "#7b3fe4"        # 液压车轴段/转向轮：紫色（区别于机械车黑色轮廓）
CARGO_SWEPT_COLOR = "#2f9e44"  # 货物扫空曲线（R_sf/R_sa/r_s，货物外伸时）：绿色短划线，区别于车辆扫掠灰线

# 构造弧(扫掠 R_o/R_i)相对道路边界的向内留隙(m)：避免半径弧贴近/压过内缘线
_SWEPT_ARC_INSET = 0.5


def _iter_geometries(geom):
    """把 Polygon/MultiPolygon 展开为 (环点表) 迭代器（含内部环）。"""
    if geom is None or geom.is_empty:
        return
    if geom.geom_type == "Polygon":
        yield list(geom.exterior.coords)
        for interior in geom.interiors:
            yield list(interior.coords)
    elif geom.geom_type == "MultiPolygon":
        for poly in geom.geoms:
            yield list(poly.exterior.coords)
            for interior in poly.interiors:
                yield list(interior.coords)


def _extents(*geoms):
    xs, ys = [], []
    for g in geoms:
        if g is None or g.is_empty:
            continue
        xs0, ys0, xs1, ys1 = g.bounds
        xs += [xs0, xs1]
        ys += [ys0, ys1]
    if not xs:
        return (-10, 10, -10, 10)
    return (min(xs), max(xs), min(ys), max(ys))


def _view_extents(ax, *geoms):
    """视图范围 = 几何 bounds ∪ 已绘制折线数据点 ∪ 文字/标注锚点。

    尺寸线与半径/宽度标注都画在车辆轮廓之外（`_annotate_rect_dims` 最多外移
    3.2+1.7 / 6.6+1.9 m），只用 road/swept 的 bounds 会把它们排除在 xlim/ylim
    之外，最终图只能依赖 `bbox_inches="tight"` 兜底，留白与比例不稳定。
    """
    minx, maxx, miny, maxy = _extents(*geoms)

    def _acc(x, y):
        nonlocal minx, maxx, miny, maxy
        if math.isfinite(x) and math.isfinite(y):
            minx, maxx = min(minx, x), max(maxx, x)
            miny, maxy = min(miny, y), max(maxy, y)

    for ln in ax.lines:                      # 道路/扫掠/车辆/货物/尺寸线等全部折线
        for x, y in zip(ln.get_xdata(), ln.get_ydata()):
            _acc(float(x), float(y))
    for t in ax.texts:                       # 含 ax.annotate 生成的对象（xy/xyann）
        for p in (getattr(t, "xy", None), getattr(t, "xyann", None)):
            if p is not None:
                _acc(float(p[0]), float(p[1]))
        pos = t.get_position()
        _acc(float(pos[0]), float(pos[1]))
    return minx, maxx, miny, maxy


def _plot_ring(ax, coords, color, lw, ls="-"):
    """闭合折线绘制（首尾相连），供道路/车辆/包络环共用。"""
    xs = [c[0] for c in coords] + [coords[0][0]]
    ys = [c[1] for c in coords] + [coords[0][1]]
    ax.plot(xs, ys, color=color, linewidth=lw, linestyle=ls)


def _crosshair(ax, x, y, arm=1.8):
    """红色十字：关键参考点（转弯圆心）。"""
    ax.plot([x - arm, x + arm], [y, y], color=CENTER_COLOR, linewidth=1.2, zorder=5)
    ax.plot([x, x], [y - arm, y + arm], color=CENTER_COLOR, linewidth=1.2, zorder=5)


def _rotated_rect(cx, cy, ang, half_len, half_wid):
    """以 (cx,cy) 为中心、航向 ang(弧度)、长 2·half_len、宽 2·half_wid 的矩形四角。

    局部偏移 (lx,ly) 按航向旋转： (lx·cos−ly·sin, lx·sin+ly·cos)。
    """
    ca, sa = math.cos(ang), math.sin(ang)
    return [(cx + lx * ca - ly * sa, cy + lx * sa + ly * ca)
            for lx, ly in ((-half_len, -half_wid), (half_len, -half_wid),
                           (half_len, half_wid), (-half_len, half_wid))]


def _plot_steered_axle(ax, cx, cy, psi, delta_deg, half_track, n_tires=8,
                       color=AXLE_COLOR):
    """绘制液压车一根轴：轴段 + 两侧按 δ 偏转的多轮胎实体（第六章图式）。

    cx/cy 为轴中点；psi 为车板航向；delta_deg 为该轴转向角（首/末朝瞬心偏转、
    闭锁轴保持 ψ 平行即 δ=0，故无需单独的锁死标志）。half_track 为轮距半宽。
    n_tires 为该轴轮胎数，沿轴横向均布于轮距上，每个轮胎沿偏转方向画小矩形。
    """
    tire_len = 1.1                        # 轮胎实体长度（m，沿滚动方向示意）
    ca, sa = math.cos(psi), math.sin(psi)
    nx, ny = -sa, ca                       # 横向(轴)方向
    # 轴段：垂直于车板纵轴
    ax.plot([cx - nx * half_track, cx + nx * half_track],
            [cy - ny * half_track, cy + ny * half_track],
            color=color, linewidth=1.0, zorder=4)
    da = math.radians(delta_deg)
    # 每侧轮胎数；沿轴横向均布以 fit 轮距
    per_side = int(n_tires) // 2
    if per_side < 1:
        per_side = 1
    for sgn in (-1.0, 1.0):
        for k in range(per_side):
            # 该侧第 k 个轮胎的横向位置（自轴中点向该侧，1..per_side 等分拟合半轮距）
            frac = (k + 0.5) / per_side
            wx = cx + sgn * nx * half_track * frac
            wy = cy + sgn * ny * half_track * frac
            tire_wid = 2.0 * half_track / per_side * 0.85   # 轮胎宽占比该排间距
            # 轮胎矩形航向 = 车板航向 + 该轴转向角（此前手写四角有 tsa 符号错误，
            # 使矩形随航向畸变、ψ=90/270° 时退化为零面积线段）
            corners = _rotated_rect(wx, wy, psi + da, tire_len / 2.0,
                                    tire_wid / 2.0)
            _plot_ring(ax, corners, color, 1.0)


def _annotate_radius(ax, center, radius, span_deg, label, side, road=None,
                     ls="-", label_ang=None, color=DIM_COLOR):
    """构造圆弧 + 引出线半径标注（仿 R13122 / R4465）。

    span_deg 为圆弧张角（度）：从弯道入口(0°)铺到弯道转角，使构造圆弧
    贴合弯道实际转弯段、不伸进出入弯直道。side 取 'out'/'in'，决定文字
    落在弧线外侧（开阔区）还是内侧（圆心岛区）。label_ang 可另行指定
    引出线/文字所在角度，避免与其余标注重叠。color 允许扫掠曲线
    （R_o / R_i）用灰色，道路曲线半径等标注沿用深蓝。

    传入 road 几何时，圆弧只画出落在道路范围内的连续段：这样内半径圆弧
    落在直角弯内角空腔（道路L形切掉的口袋）里的部分会被裁掉，避免图上
    出现“半径标注圆越过道路边界”的误导。
    """
    cxx, cyy = center
    a = math.radians(span_deg if label_ang is None else label_ang)
    if road is not None:
        from shapely.geometry import Point
        n = 240
        run_x, run_y = [], []
        for i in range(n + 1):
            d = span_deg * i / n
            px = cxx + radius * math.cos(math.radians(d))
            py = cyy + radius * math.sin(math.radians(d))
            if road.covers(Point(px, py)):
                run_x.append(px)
                run_y.append(py)
            else:
                if len(run_x) >= 2:
                    ax.plot(run_x, run_y, color=color,
                            linewidth=0.9, linestyle=ls, zorder=3)
                run_x, run_y = [], []
        if len(run_x) >= 2:
            ax.plot(run_x, run_y, color=color, linewidth=0.9,
                    linestyle=ls, zorder=3)
    else:
        ax.add_patch(Arc((cxx, cyy), 2 * radius, 2 * radius,
                         theta1=0.0, theta2=span_deg,
                         color=color, linewidth=0.9,
                         linestyle=ls, zorder=3))
    ex = cxx + radius * math.cos(a)
    ey = cyy + radius * math.sin(a)
    offset = 2.4 if side == "out" else -2.4
    if side == "in" and radius + offset < 0.5:
        # 内径小于 2.4 m 时标注会落到圆心另一侧（半径为负）→ 翻到外侧
        offset = 2.4
    tx = cxx + (radius + offset) * math.cos(a)
    ty = cyy + (radius + offset) * math.sin(a)
    ax.plot([tx, ex], [ty, ey], color=color, linewidth=0.7, zorder=3)
    ax.text(tx, ty, label, color=color, fontsize=9,
            ha="center", va="center", family="Microsoft YaHei",
            bbox=dict(facecolor="white", edgecolor="none", pad=1.5),
            zorder=4)


def _annotate_band(ax, center, r_in, r_out, ang_deg, label):
    """径向尺寸线：在 ang_deg 上从 r_in 到 r_out 画双向箭头，标注扫空/转弯通道宽度 W。"""
    cxx, cyy = center
    a = math.radians(ang_deg)

    def _pt(rr):
        return (cxx + rr * math.cos(a), cyy + rr * math.sin(a))
    p0, p1 = _pt(r_in), _pt(r_out)
    ax.annotate("", xy=p1, xytext=p0,
                arrowprops=dict(arrowstyle="<->", color=DIM_COLOR, lw=1.0,
                                shrinkA=0, shrinkB=0), zorder=3)
    mx, my = _pt((r_in + r_out) / 2.0)
    lx, ly = _pt(r_out + 2.6)
    ax.plot([mx, lx], [my, ly], color=DIM_COLOR, linewidth=0.7, zorder=3)
    ax.text(lx, ly, label, color=DIM_COLOR, fontsize=9,
            ha="center", va="center", family="Microsoft YaHei",
            bbox=dict(facecolor="white", edgecolor="none", pad=1.5), zorder=4)


def _annotate_angle(ax, center, ang_deg, radius, label, start_deg=0.0):
    """CAD 交角标注：以 center 为顶点画一段张角弧，在弧中点外穿出引出线+文字。"""
    cxx, cyy = center
    ax.add_patch(Arc((cxx, cyy), 2 * radius, 2 * radius,
                     theta1=start_deg, theta2=start_deg + ang_deg,
                     color=DIM_COLOR, linewidth=0.9, zorder=3))
    a = math.radians(start_deg + ang_deg * 0.5)
    ex = cxx + radius * math.cos(a)
    ey = cyy + radius * math.sin(a)
    lx = cxx + (radius + 2.2) * math.cos(a)
    ly = cyy + (radius + 2.2) * math.sin(a)
    ax.plot([lx, ex], [ly, ey], color=DIM_COLOR, linewidth=0.7, zorder=3)
    ax.text(lx, ly, label, color=TEXT_COLOR, fontsize=9,
            ha="center", va="center", family="Microsoft YaHei",
            bbox=dict(facecolor="white", edgecolor="none", pad=1.2), zorder=4)


def _annotate_rect_dims(ax, geom, prefix, offset, color=DIM_COLOR, text_off=1.7,
                        rear_anchor=None):
    """CAD 长/宽尺寸标注：长度沿最长边、宽度沿“尾部”短边。

    长度尺寸线沿矩形最长边外移 offset；宽度尺寸线移到车尾侧的短边（即离
    rear_anchor=牵引车形心 最远的一条短边），避免宽度标注压到车头/牵引车。
    offset 为尺寸线相对轮廓的垂直偏移；prefix 区分“车”“货”；数字不带单位。
    """
    pts = list(geom.exterior.coords)[:4]
    cx = sum(p[0] for p in pts) / 4.0
    cy = sum(p[1] for p in pts) / 4.0
    E = []
    for i in range(4):
        a, b = pts[i], pts[(i + 1) % 4]
        dx, dy = b[0] - a[0], b[1] - a[1]
        L = math.hypot(dx, dy)
        ux, uy = dx / L, dy / L
        nx, ny = -uy, ux
        mx, my = (a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0
        if nx * (mx - cx) + ny * (my - cy) < 0:   # 取背离形心的外法向
            nx, ny = -nx, -ny
        E.append([a, b, ux, uy, nx, ny, L, mx, my])

    E.sort(key=lambda e: e[6], reverse=True)        # 长度降序
    len_e = E[0]                                   # 最长边 = 尺寸“长”
    short_edges = [e for e in E[1:] if e[6] < len_e[6] - 1e-6]
    if not short_edges:
        width_e = len_e                            # 正方形：长宽同边
    elif rear_anchor is not None:
        # 尾部短边 = 离车头锚点最远的那条短边
        width_e = max(short_edges, key=lambda e:
                      math.hypot(e[7] - rear_anchor[0], e[8] - rear_anchor[1]))
    else:
        width_e = short_edges[0]

    for idx, (e, tag) in enumerate(((len_e, "长"), (width_e, "宽"))):
        if width_e is len_e and idx == 1:
            continue                               # 正方形长宽同边，仅标一条
        a, b, _ux, _uy, nx, ny, L, _mx, _my = e
        oa = (a[0] + nx * offset, a[1] + ny * offset)
        ob = (b[0] + nx * offset, b[1] + ny * offset)
        ax.plot([a[0], oa[0]], [a[1], oa[1]], color=color, linewidth=0.6, zorder=3)
        ax.plot([b[0], ob[0]], [b[1], ob[1]], color=color, linewidth=0.6, zorder=3)
        ax.annotate("", xy=ob, xytext=oa,
                    arrowprops=dict(arrowstyle="<->", color=color, lw=0.8,
                                    shrinkA=0, shrinkB=0), zorder=3)
        mx, my = (oa[0] + ob[0]) / 2.0, (oa[1] + ob[1]) / 2.0
        tx, ty = mx + nx * text_off, my + ny * text_off
        ax.text(tx, ty, f"{prefix}{tag} {L:.2f}", color=TEXT_COLOR, fontsize=8,
                ha="center", va="center", family="Microsoft YaHei",
                bbox=dict(facecolor="white", edgecolor="none", pad=1.0), zorder=4)


def _hydraulic_front_anchor(trailer_geom, frame0, frame1):
    """液压车（无牵引车图形）的“车头前端”锚点，使宽度标注稳定落向车尾。

    用首帧行驶方向(后一帧位置−首帧位置)作为车板纵向前向，把锚点放在车板
    前端短边外侧；_annotate_rect_dims 取距它最远的短边 = 车尾短边。
    """
    ux, uy = 0.0, 1.0
    if frame1 is not None:
        dx, dy = frame1[0] - frame0[0], frame1[1] - frame0[1]
        dl = math.hypot(dx, dy)
        if dl > 1e-6:
            ux, uy = dx / dl, dy / dl
    pts = list(trailer_geom.exterior.coords)[:4]
    half_len = max(math.hypot(pts[i][0] - pts[(i + 1) % 4][0],
                              pts[i][1] - pts[(i + 1) % 4][1])
                  for i in range(4)) / 2.0
    c = trailer_geom.centroid
    return (c.x + ux * (half_len + 0.5), c.y + uy * (half_len + 0.5))


def build_figure(res: SweepScenarioResult, figsize=(12, 10)):
    """构建扫掠工程图的 (fig, ax)，不保存、不关闭（供测试检视图元）。

    `draw_sweep_view` 只是本函数的"保存并关闭"封装。
    """
    fig, ax = plt.subplots(figsize=figsize)
    ax.set_facecolor("white")
    scene = getattr(res, "scene", "arc")
    pass_ok = bool(res.pass_ok)

    # Layer 1: 扫掠包络 —— 蓝色细虚线（构造/测量几何）
    if res.swept is not None:
        for ring in _iter_geometries(res.swept):
            _plot_ring(ax, ring, SWEPT_COLOR, 0.9, ls=(0, (5, 3)))

    # Layer 2: 道路边界 —— 黑色粗实线（主结构）
    if res.road is not None:
        for ring in _iter_geometries(res.road):
            _plot_ring(ax, ring, ROAD_COLOR, 1.9)

    # Layer 3: 抽帧车辆轮廓 —— 挂车/牵引车蓝色中实线；货物绿色细线
    #           （无论超宽与否都单独画出货物轮廓，直观反映车货相对关系）
    for frame in res.vehicle_frames:
        _x, _y, geom, trailer_geom, cargo_geom = frame[:5]
        axle_geoms = frame[5] if len(frame) > 5 else None
        geoms = geom.geoms if geom.geom_type == "MultiPolygon" else [geom]
        # 牵引车（合并几何最后一个组件）
        if len(geoms) >= 2:
            _plot_ring(ax, list(geoms[-1].exterior.coords), VEHICLE_COLOR, 1.1)
        # 挂车车板
        _plot_ring(ax, list(trailer_geom.exterior.coords), VEHICLE_COLOR, 1.1)
        # 液压车轴线层（仅当此帧携带 axle_geoms）
        if axle_geoms and axle_geoms.get("axles"):
            psi = axle_geoms.get("psi", 0.0)
            n_tires = int(axle_geoms.get("n_tires") or 8)
            for (aidx, axc, axy, dd, halft, _locked) in axle_geoms["axles"]:
                _plot_steered_axle(ax, axc, axy, psi, dd, halft,
                                   n_tires=n_tires)
        # 货物轮廓（始终显示）
        _plot_ring(ax, list(cargo_geom.exterior.coords), CARGO_COLOR, 1.2)
        ax.plot([_x], [_y], "o", color=VEHICLE_COLOR, markersize=3, zorder=4)

    # —— 车辆/货物尺寸标注：沿“入弯前第一辆车”的外沿画 CAD 长/宽尺寸线 ——
    if res.vehicle_frames:
        _first = res.vehicle_frames[0]       # 第一辆车 = 入弯前的初始轮廓
        _fg = _first[2].geoms if _first[2].geom_type == "MultiPolygon" else [_first[2]]
        if len(_fg) >= 2:
            # 机械车：以牵引车形心为“车头”锚点，宽度标注落在车尾侧、避开车头
            rear_anchor = (_fg[-1].centroid.x, _fg[-1].centroid.y)
        else:
            # 液压车（无车头图形）：按首帧行驶方向合成车头前端锚点，宽度线落向车尾
            _nf = res.vehicle_frames[1] if len(res.vehicle_frames) > 1 else None
            rear_anchor = _hydraulic_front_anchor(_first[3], _first, _nf)
        _annotate_rect_dims(ax, _first[3], "车", 3.2, color=DIM_COLOR,
                            rear_anchor=rear_anchor)
        _annotate_rect_dims(ax, _first[4], "货", 6.6, color=DIM_COLOR,
                            text_off=1.9, rear_anchor=rear_anchor)

    # Layer 4: 转弯圆心红色十字 + 扫掠半径 / 道路曲线半径 标注（数字无单位，统一为 m）
    metrics = res.metrics
    # 转弯圆心与仿真转角均由计算层给出，渲染层不再硬编码 (0,0)、也不再跨模块
    # 引用 passability 的私有常量（_TURN_DEG / _RA_TURN_DEG）
    _ctr = metrics.get("turn_center") or (0.0, 0.0)
    center = (float(_ctr[0]), float(_ctr[1]))
    _crosshair(ax, center[0], center[1])
    turn_deg = float(metrics.get("turn_deg") or (140.0 if scene == "arc" else 90.0))
    r_o = float(metrics.get("R_o", 0.0) or 0.0)
    r_i = float(metrics.get("R_i", 0.0) or 0.0)
    # 扫掠曲线 = 外/内两条半径圆弧 R_o / R_i，用灰色（区别于深蓝标注线）。
    # 圆弧弯道：道路即整条弯曲带，构造圆弧完整铺满；直角弯道：道路为 L 形带，
    # 扫掠内弧 R_m 会在中段伸进内角空腔而越出路面，故按道路 footprint 裁剪，
    # 使扫掠曲线只落在弯道（道路带）内。
    clip_road = (res.road.buffer(-_SWEPT_ARC_INSET)
                 if (scene == "right_angle" and res.road is not None
                     and not res.road.is_empty) else None)
    if r_o > 0.5:
        _annotate_radius(ax, center, r_o, turn_deg, f"扫掠外径 R_o = {r_o:.3f}",
                         "out", color=SWEPT_ARC_COLOR, road=clip_road)
    if r_i > 0.5:
        r_i_label = (f"最内侧转弯 R_m = {r_i:.3f}" if scene == "right_angle"
                     else f"扫掠内径 R_i = {r_i:.3f}")
        _annotate_radius(ax, center, r_i, turn_deg, r_i_label, "in",
                         color=SWEPT_ARC_COLOR, road=clip_road)

    # —— 货物前/后外侧扫空曲线 + 内侧扫空曲线（第六章公式；仅在货物外伸时绘制）——
    cargo_swept = res.metrics.get("cargo_swept")
    if scene == "arc" and isinstance(cargo_swept, dict) and cargo_swept.get("active"):
        csw_ls = (0, (3, 2))                       # 绿色短划线，区别于车辆扫掠(灰)
        # 前/后外扫空给不同引出线角度，避免对称外伸时两条同半径弧标注叠在一起
        out_arcs = (
            (cargo_swept["R_sf"], 50.0, f"货物前外扫空 R_sf = {cargo_swept['R_sf']:.2f}"),
            (cargo_swept["R_sa"], 30.0, f"货物后外扫空 R_sa = {cargo_swept['R_sa']:.2f}"),
        )
        for radius, l_ang, label_ in out_arcs:
            if radius > 0.5 and radius >= r_o - 1e-9:
                _annotate_radius(ax, center, radius, turn_deg, label_, "out",
                                 color=CARGO_SWEPT_COLOR, ls=csw_ls, label_ang=l_ang)
        r_s = cargo_swept["r_s"]
        if r_s > 0.3 and r_s < r_i + 1e-9:
            _annotate_radius(ax, center, r_s, turn_deg,
                             f"货物内扫空 r_s = {r_s:.2f}", "in",
                             color=CARGO_SWEPT_COLOR, ls=csw_ls, label_ang=40.0)

    # —— 圆弧弯道：补画“弯道部分”道路曲线内/外缘半径（区别于整车扫掠半径）——
    if scene == "arc":
        road_outer = float(metrics.get("road_outer", 0.0) or 0.0)
        road_lane = float(metrics.get("road_lane", 0.0) or 0.0)
        road_inner = road_outer - road_lane
        # 曲线外/内缘构造圆弧完整铺满转弯段，但引出文字错开到中途角度，
        # 避免与末端 R_o / R_i 标注重叠。
        if road_outer > 0.5:
            _annotate_radius(ax, center, road_outer, turn_deg,
                             f"道路曲线外缘 = {road_outer:.2f}", "out",
                             ls=(0, (6, 2)), label_ang=38.0)
        if road_inner > 0.5:
            _annotate_radius(ax, center, road_inner, turn_deg,
                             f"道路曲线内缘 = {road_inner:.2f}", "in",
                             ls=(0, (6, 2)), label_ang=38.0)

    # —— 转弯通道宽度标注（扫掠宽度 W）——
    mid_ang = turn_deg * 0.5
    if scene == "arc":
        swept_w = float(metrics.get("W_swept", 0.0) or 0.0)
        band_w = max(r_o - r_i, 0.0)
        if r_i > 0.5 and r_o > 0.5 and band_w > 0:
            # 环带标注须与**所画环带**一致，取几何带宽 R_o−R_i；液压轴线挂车的判定
            # 量是 B.1.2-6 特征宽度 W_d，其值与几何带宽不同（B.1.2-5 的 R_max 偏大），
            # 故另附注，避免"标注值 ≠ 图中环带宽度"的误读。
            band_label = f"扫掠环带 R_o−R_i = {band_w:.2f}"
            if abs(swept_w - band_w) > 0.02:
                band_label += f"（判定用 W_d = {swept_w:.2f}）"
            _annotate_band(ax, center, r_i, r_o, mid_ang, band_label)
    else:  # right_angle
        w = float(metrics.get("W", 0.0) or 0.0)
        if r_i > 0.5 and r_o > 0.5 and w > 0:
            _annotate_band(ax, center, r_i, r_o, mid_ang,
                           f"扫掠宽度 W = {w:.2f}")

        # —— 直角弯：交角 + 弯道（内角导圆）半径 标注 ——
        r_path = float(metrics.get("r_path", 0.0) or 0.0)
        if abs(turn_deg - 90.0) < 1e-6:
            eh = float(metrics.get("entry_width", 0.0) or 0.0) / 2.0
            xh = float(metrics.get("exit_width", 0.0) or 0.0) / 2.0
            # 交角：顶点锚在 L 形弯路的外角肘点（两条车道外缘线的交会点，
            # 即外角轴点），张角弧开口朝向弯道内侧，表示入口/出口两直道
            # 在转角处的夹角（区别于内角导圆那个"圆"）。
            _annotate_angle(ax, (r_path + eh, r_path + xh), 90.0, 4.5,
                            f"交角 {turn_deg:.0f}°", start_deg=180.0)
            # 弯道内半径：内角导圆的圆弧半径（圆心在内角顶点左下方）
            cr = float(metrics.get("corner_radius", 0.0) or 0.0)
            if cr > 1e-9:
                cxx, cyy = r_path - eh - cr, r_path - xh - cr
                ax.add_patch(Arc((cxx, cyy), 2 * cr, 2 * cr,
                                 theta1=0.0, theta2=90.0,
                                 color=DIM_COLOR, linewidth=0.9,
                                 linestyle=(0, (6, 2)), zorder=3))
                a = math.radians(45.0)
                ex = cxx + cr * math.cos(a)
                ey = cyy + cr * math.sin(a)
                lx = cxx + (cr + 2.4) * math.cos(a)
                ly = cyy + (cr + 2.4) * math.sin(a)
                ax.plot([lx, ex], [ly, ey], color=DIM_COLOR, linewidth=0.7, zorder=3)
                ax.text(lx, ly, f"内角半径 = {cr:.2f}", color=TEXT_COLOR, fontsize=9,
                        ha="center", va="center", family="Microsoft YaHei",
                        bbox=dict(facecolor="white", edgecolor="none", pad=1.2),
                        zorder=4)
        else:
            # 非正交交角：绕转弯圆心标一段“入口→出口”的转角弧
            r_ang = (min(r_i, r_path) * 0.5 if r_i > 0.5 else r_path * 0.5)
            _annotate_angle(ax, center, turn_deg, max(r_ang, 1.5),
                            f"交角 {turn_deg:.0f}°", start_deg=90.0)

    # Layer 5: 检验结论 + 图例（左下角）
    from matplotlib.lines import Line2D
    verdict = "通过" if pass_ok else "不通过"
    vc = "#2f855a" if pass_ok else "#c53030"
    sigma = float(metrics.get("sigma", 0.0) or 0.0)
    info_lines = [f"检验结论：{verdict}"]
    if scene == "arc":
        # 每个"所需宽度"必须与其对应的"可供宽度"配对，不能混用（旧版把扫空空间
        # 宽度 W_d 与**车道宽度**相比，导致通过工况显示成"12.57 / 7.00"而看起来超限）：
        #   转弯通道宽度 ≤ 路面宽度(行车道+硬路肩) ／ 扫空空间宽度 ≤ 横向空间宽度
        # 查表法命中时两项宽度判据由 4.3.1-1 覆盖，此时不再逐条列出，只列真正生效的
        # 4.2.1 内/外缘预留。
        lookup = metrics.get("lookup")
        w_turn = float(metrics.get("turn_channel_w")
                       or metrics.get("W", 0.0) or 0.0)
        road_w = float(metrics.get("road_width", 0.0) or 0.0)
        w_swept = float(metrics.get("W_swept", 0.0) or 0.0)
        trans_w = float(metrics.get("transverse_space_w", 0.0) or 0.0)
        if lookup == "pass":
            info_lines.append("4.3.1-1 查表法：两项宽度判据满足")
        else:
            info_lines.append(
                f"转弯通道 {w_turn:.2f} "
                f"{'≤' if w_turn <= road_w + 1e-6 else '>'} 路面 {road_w:.2f}")
            info_lines.append(
                f"扫空 {w_swept:.2f} "
                f"{'≤' if w_swept <= trans_w + 1e-6 else '>'} 横向空间 {trans_w:.2f}")
        clr_o = metrics.get("clearance_outer")
        clr_i = metrics.get("clearance_inner")
        if clr_o is not None and clr_i is not None:
            clr_ok = min(float(clr_i), float(clr_o)) >= sigma - 1e-6
            info_lines.append(
                f"4.2.1 内/外缘净空 {float(clr_i):.2f} / {float(clr_o):.2f} "
                f"{'≥' if clr_ok else '<'} σ {sigma:.2f}")
    elif "entry_width" in metrics:
        d_w2 = float(metrics.get("tractor_swing_widening", 0.0) or 0.0)
        info_lines.append(
            f"入口 {metrics.get('entry_width', 0):.2f} / "
            f"出口 {metrics.get('exit_width', 0):.2f}")
        req = (f"扫空 W = {metrics.get('W', 0):.2f}"
               # 图内用 ASCII "ΔW2"：Microsoft YaHei 无 U+2082(₂) 字形，
               # 直接用下标字符会渲染成缺字方块
               + (f" + 牵引车加宽 ΔW2 {d_w2:.2f} = "
                  f"{metrics.get('exit_required_w', 0):.2f}" if d_w2 else ""))
        info_lines.append(req)
    info = "\n".join(info_lines)
    ax.text(0.015, 0.97, info, transform=ax.transAxes, verticalalignment="top",
            horizontalalignment="left", fontsize=10.0, family="Microsoft YaHei",
            fontweight="bold", color=vc, linespacing=1.35,
            bbox=dict(facecolor="white", alpha=0.9, edgecolor=vc, linewidth=1.0,
                      boxstyle="round,pad=0.4"))

    handles = [
        Line2D([0], [0], color=ROAD_COLOR, lw=1.9, label="道路边界"),
        Line2D([0], [0], color=SWEPT_COLOR, lw=1.0, ls=(0, (5, 3)),
               label="扫掠包络 - 整车全程占道范围"),
        Line2D([0], [0], color=SWEPT_ARC_COLOR, lw=1.0, label="扫掠曲线（R_o / R_i）"),
    ]
    # 仅列入本图实际绘制的图元（货物扫空曲线仅在货物外伸时绘制；
    # 道路曲线内/外缘半径、交角构造线仅圆弧弯道绘制）
    _cs = metrics.get("cargo_swept")
    if scene == "arc" and isinstance(_cs, dict) and _cs.get("active"):
        handles.append(Line2D([0], [0], color=CARGO_SWEPT_COLOR, lw=1.0,
                              ls=(0, (3, 2)),
                              label="货物扫空曲线（R_sf / R_sa / r_s）"))
    handles += [
        Line2D([0], [0], color=VEHICLE_COLOR, lw=1.1, label="车辆轮廓（挂车 / 牵引车）"),
        Line2D([0], [0], color=CARGO_COLOR, lw=1.2, label="货物轮廓"),
    ]
    if any(len(f) > 5 and f[5] for f in res.vehicle_frames):
        handles.append(Line2D([0], [0], color=AXLE_COLOR, lw=1.1,
                              label="液压车轴线/转向轮（δ 朝瞬心）"))
    if scene == "arc":
        handles.append(Line2D([0], [0], color=DIM_COLOR, lw=0.9,
                              label="道路曲线内 / 外缘半径"))
    else:
        handles.append(Line2D([0], [0], color=DIM_COLOR, lw=0.9, ls=(0, (6, 2)),
                              label="交角 / 内角半径 构造线"))
    handles += [
        Line2D([0], [0], color=CENTER_COLOR, ls='none', marker='+', markersize=8,
               label="转弯圆心（车辆绕其转弯）"),
        Line2D([0], [0], color=DIM_COLOR, lw=1.0, ls="-",
               label="尺寸标注（扫掠宽度 W / 车·货长宽）"),
    ]
    from matplotlib.font_manager import FontProperties
    ax.legend(handles=handles, loc="lower left", title="图　例",
              framealpha=0.94, edgecolor="#444444", labelspacing=0.45,
              prop=dict(family="Microsoft YaHei", size=8),
              title_fontproperties=FontProperties(family="Microsoft YaHei", size=9))

    minx, maxx, miny, maxy = _view_extents(ax, res.road, res.swept)
    xpad = max((maxx - minx) * 0.12, 2.0)
    ypad = max((maxy - miny) * 0.12, 2.0)
    ax.set_xlim(minx - xpad, maxx + xpad)
    ax.set_ylim(miny - ypad, maxy + ypad)
    ax.set_aspect("equal", "box")
    for spine in ax.spines.values():
        spine.set_color(SPINE_COLOR)
        spine.set_linewidth(0.8)
    ax.tick_params(colors=SPINE_COLOR, labelsize=8, length=3)
    ax.grid(False)
    ax.set_title("大件运输弯道扫掠通过性检验图", fontsize=13,
                 family="Microsoft YaHei", color="#000000")
    ax.set_xlabel("X (m)", fontsize=10, color="#666666")
    ax.set_ylabel("Y (m)", fontsize=10, color="#666666")
    return fig, ax


def draw_sweep_view(res: SweepScenarioResult, out_path, dpi=PLOT_DPI,
                    figsize=(12, 10)):
    """绘制 CAD 线框风格扫掠工程图并保存。返回图路径。"""
    fig, ax = build_figure(res, figsize)
    plt.tight_layout()
    plt.savefig(out_path, dpi=dpi, bbox_inches="tight")
    plt.close(fig)
    return out_path
