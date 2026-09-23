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
LOCKED_TEXT_COLOR = "#c53030" # 闭锁轴文字：红色

# 构造弧(扫掠 R_o/R_i)相对道路边界的向内留隙(m)：避免半径弧贴近/压过内缘线
_SWEPT_ARC_INSET = 0.5


def _iter_geometries(geom):
    """把 Polygon/MultiPolygon 展开为 (环点表) 迭代器（含内部环）。"""
    from shapely.geometry import Polygon, MultiPolygon
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


def _plot_ring(ax, coords, color, lw, ls="-"):
    """闭合折线绘制（首尾相连），供道路/车辆/包络环共用。"""
    xs = [c[0] for c in coords] + [coords[0][0]]
    ys = [c[1] for c in coords] + [coords[0][1]]
    ax.plot(xs, ys, color=color, linewidth=lw, linestyle=ls)


def _crosshair(ax, x, y, arm=1.8):
    """红色十字：关键参考点（转弯圆心）。"""
    ax.plot([x - arm, x + arm], [y, y], color=CENTER_COLOR, linewidth=1.2, zorder=5)
    ax.plot([x, x], [y - arm, y + arm], color=CENTER_COLOR, linewidth=1.2, zorder=5)


def _plot_steered_axle(ax, cx, cy, psi, delta_deg, half_track, locked,
                       color=AXLE_COLOR):
    """绘制液压车一根轴：轴段 + 两侧按 δ 偏转的轮胎实体（第六章图式）。

    cx/cy 为轴中点；psi 为车板航向；delta_deg 为该轴转向角（首/末朝瞬心偏转、
    闭锁轴保持 ψ 平行即 δ=0）。half_track 为轮距半宽。轮胎沿偏转方向画小矩形。
    """
    tire_len, tire_wid = 1.1, 0.34           # 轮胎实体尺寸（m，示意）
    ca, sa = math.cos(psi), math.sin(psi)
    # 轴段：垂直于车板纵轴
    nx, ny = -sa, ca
    ax.plot([cx - nx * half_track, cx + nx * half_track],
            [cy - ny * half_track, cy + ny * half_track],
            color=color, linewidth=1.0, zorder=4)
    da = math.radians(delta_deg)
    tca, tsa = math.cos(psi + da), math.sin(psi + da)    # 轮胎方向(受 δ 偏转)
    for sgn in (-1.0, 1.0):
        wx = cx + sgn * nx * half_track
        wy = cy + sgn * ny * half_track
        hx, hy = tire_len / 2.0, tire_wid / 2.0
        corners = [(wx + (-hx * tca - -hy * -tsa), wy + (-hx * tsa + -hy * tca)),
                   (wx + (hx * tca - -hy * -tsa), wy + (hx * tsa + -hy * tca)),
                   (wx + (hx * tca - hy * tsa), wy + (hx * tsa + hy * tca)),
                   (wx + (-hx * tca - hy * tsa), wy + (-hx * tsa + hy * tca))]
        _plot_ring(ax, corners, color, 1.0)
    if locked:
        ax.text(cx + 2 * nx * half_track, cy + 2 * ny * half_track, "闭锁",
                color=LOCKED_TEXT_COLOR, fontsize=7, ha="center", va="center",
                family="Microsoft YaHei", zorder=5)


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


def _annotate_rect_dims(ax, geom, prefix, offset, color=DIM_COLOR, text_off=1.7):
    """CAD 长/宽尺寸标注：沿矩形几何相邻两条临边外沿偏移画带箭头的尺寸线。

    offset 为尺寸线相对轮廓的垂直偏移（越大越远离车体/货物，避开图形主体）；
    prefix 用于区分“车”“货”，数字不带单位（统一视为 m）。
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
        E.append((a, b, ux, uy, nx, ny, L))

    is_long0 = E[0][6] >= E[1][6]
    for idx, tag in ((0, "长" if is_long0 else "宽"),
                     (1, "宽" if is_long0 else "长")):
        a, b, _ux, _uy, nx, ny, L = E[idx]
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


def draw_sweep_view(res: SweepScenarioResult, out_path, dpi=PLOT_DPI,
                    figsize=(12, 10)):
    """绘制 CAD 线框风格扫掠工程图。返回图路径。"""
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
            for (aidx, axc, axy, dd, halft, locked) in axle_geoms["axles"]:
                _plot_steered_axle(ax, axc, axy, psi, dd, halft, locked)
        # 货物轮廓（始终显示）
        _plot_ring(ax, list(cargo_geom.exterior.coords), CARGO_COLOR, 1.2)
        ax.plot([_x], [_y], "o", color=VEHICLE_COLOR, markersize=3, zorder=4)

    # —— 车辆/货物尺寸标注：沿“入弯前第一辆车”的外沿画 CAD 长/宽尺寸线 ——
    if res.vehicle_frames:
        _first = res.vehicle_frames[0]       # 第一辆车 = 入弯前的初始轮廓
        _annotate_rect_dims(ax, _first[3], "车", 3.2, color=DIM_COLOR)
        _annotate_rect_dims(ax, _first[4], "货", 6.6, color=DIM_COLOR, text_off=1.9)

    # Layer 4: 转弯圆心红色十字 + 扫掠半径 / 道路曲线半径 标注（数字无单位，统一为 m）
    metrics = res.metrics
    center = (0.0, 0.0)
    _crosshair(ax, center[0], center[1])
    from app.core.passability import _TURN_DEG, _RA_TURN_DEG
    turn_deg = _TURN_DEG if scene == "arc" \
        else float(metrics.get("turn_deg", _RA_TURN_DEG))
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
        if r_i > 0.5 and r_o > 0.5 and swept_w > 0:
            _annotate_band(ax, center, r_i, r_o, mid_ang,
                           f"扫掠宽度 W_s = {swept_w:.2f}")
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
    info = f"检验结论：{verdict}"
    if "road_lane" in metrics:
        w_req = metrics.get("W", 0)
        lane = metrics.get("road_lane", 0)
        info += f"　所需宽度 W = {w_req:.2f} / 实际 {lane:.2f}"
    elif "entry_width" in metrics:
        info += (f"　入口 {metrics.get('entry_width', 0):.2f} / "
                 f"出口 {metrics.get('exit_width', 0):.2f}，"
                 f"所需扫掠宽 W = {metrics.get('W', 0):.2f}")
    ax.text(0.015, 0.97, info, transform=ax.transAxes, verticalalignment="top",
            horizontalalignment="left", fontsize=10.5, family="Microsoft YaHei",
            fontweight="bold", color=vc,
            bbox=dict(facecolor="white", alpha=0.9, edgecolor=vc, linewidth=1.0,
                      boxstyle="round,pad=0.4"))

    handles = [
        Line2D([0], [0], color=ROAD_COLOR, lw=1.9, label="道路边界"),
        Line2D([0], [0], color=SWEPT_COLOR, lw=1.0, ls=(0, (5, 3)),
               label="扫掠包络 - 整车全程占道范围"),
        Line2D([0], [0], color=SWEPT_ARC_COLOR, lw=1.0, label="扫掠曲线（R_o / R_i）"),
        Line2D([0], [0], color=VEHICLE_COLOR, lw=1.1, label="车辆轮廓（挂车 / 牵引车）"),
        Line2D([0], [0], color=CARGO_COLOR, lw=1.2, label="货物轮廓"),
        Line2D([0], [0], color=AXLE_COLOR, lw=1.1, ls="-",
               label="液压车轴线/转向轮（仅液压车，δ朝瞬心）"),
        Line2D([0], [0], color=DIM_COLOR, lw=0.9, label="道路曲线内 / 外缘半径"),
        Line2D([0], [0], color=DIM_COLOR, lw=0.9, ls=(0, (6, 2)),
               label="交角 / 内角半径 构造线"),
        Line2D([0], [0], color=CENTER_COLOR, ls='none', marker='+', markersize=8,
               label="转弯圆心（车辆绕其转弯）"),
        Line2D([0], [0], color=DIM_COLOR, lw=1.0, ls="-",
               label="尺寸标注（扫掠宽度 W / 车·货长宽）"),
        Line2D([0], [0], color=DIM_COLOR, lw=0.9, ls="-.",
               label="车辆·货物长宽尺寸标注（首辆入弯前轮廓）"),
    ]
    from matplotlib.font_manager import FontProperties
    ax.legend(handles=handles, loc="lower left", title="图　例",
              framealpha=0.94, edgecolor="#444444", labelspacing=0.45,
              prop=dict(family="Microsoft YaHei", size=8),
              title_fontproperties=FontProperties(family="Microsoft YaHei", size=9))

    minx, maxx, miny, maxy = _extents(res.road, res.swept)
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

    plt.tight_layout()
    plt.savefig(out_path, dpi=dpi, bbox_inches="tight")
    plt.close(fig)
    return out_path
