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
VEHICLE_COLOR = "#1a5fb4"     # 车辆轮廓：蓝色中实线
CARGO_COLOR = "#2e9e44"       # 货物轮廓：绿色实线（与车辆/道路区分）
SWEPT_COLOR = "#4d8dd0"       # 扫掠包络：蓝色细虚线
DIM_COLOR = "#7a7a7a"         # 尺寸/半径标注（构造圆弧）：中灰，仿CAD尺寸线
CENTER_COLOR = "#e02020"      # 关键参考点：红色十字
TEXT_COLOR = "#111111"        # 标注文字：黑色
SPINE_COLOR = "#b8c0c8"       # 坐标轴细线：浅灰


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


def _annotate_radius(ax, center, radius, span_deg, label, side):
    """构造圆弧 + 引出线半径标注（仿 R13122 / R4465）。

    span_deg 为圆弧张角（度）：从弯道入口(0°)铺到弯道转角，使构造圆弧
    贴合弯道实际转弯段、不伸进出入弯直道。side 取 'out'/'in'，决定文字
    落在弧线外侧（开阔区）还是内侧（圆心岛区）。
    """
    cxx, cyy = center
    a = math.radians(span_deg)
    ax.add_patch(Arc((cxx, cyy), 2 * radius, 2 * radius,
                     theta1=0.0, theta2=span_deg,
                     color=DIM_COLOR, linewidth=0.9, zorder=3))
    ex = cxx + radius * math.cos(a)
    ey = cyy + radius * math.sin(a)
    offset = 2.4 if side == "out" else -2.4
    tx = cxx + (radius + offset) * math.cos(a)
    ty = cyy + (radius + offset) * math.sin(a)
    ax.plot([tx, ex], [ty, ey], color=DIM_COLOR, linewidth=0.7, zorder=3)
    ax.text(tx, ty, label, color=DIM_COLOR, fontsize=9,
            ha="center", va="center", family="Microsoft YaHei",
            bbox=dict(facecolor="white", edgecolor="none", pad=1.5),
            zorder=4)


def _rect_len_width(poly):
    """返回矩形 (长, 宽)（由顶点边长求得，与旋转无关）。"""
    xs = list(poly.exterior.coords)
    d1 = math.hypot(xs[1][0] - xs[0][0], xs[1][1] - xs[0][1])
    d2 = math.hypot(xs[2][0] - xs[1][0], xs[2][1] - xs[1][1])
    return (max(d1, d2), min(d1, d2))


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

    # Layer 3: 抽帧车辆轮廓 —— 挂车/牵引车蓝色中实线；货物黑色细线
    #           （货物超宽/超长时黑色轮廓凸出车板，直观可见）
    cargo_overhang = False
    for _x, _y, geom, trailer_geom, cargo_geom in res.vehicle_frames:
        from shapely.geometry import Polygon as _Poly, MultiPolygon as _MP
        geoms = geom.geoms if geom.geom_type == "MultiPolygon" else [geom]
        # 牵引车（合并几何最后一个组件）
        if len(geoms) >= 2:
            _plot_ring(ax, list(geoms[-1].exterior.coords), VEHICLE_COLOR, 1.1)
        # 挂车车板
        _plot_ring(ax, list(trailer_geom.exterior.coords), VEHICLE_COLOR, 1.1)
        # 货物：超宽/超长时以黑色轮廓单独绘制
        if not cargo_overhang:
            tl_, tw_ = _rect_len_width(trailer_geom)
            cl_, cw_ = _rect_len_width(cargo_geom)
            cargo_overhang = (cl_ > tl_ + 1e-6) or (cw_ > tw_ + 1e-6)
        if cargo_overhang:
            _plot_ring(ax, list(cargo_geom.exterior.coords), CARGO_COLOR, 1.2)
        ax.plot([_x], [_y], "o", color=VEHICLE_COLOR, markersize=3, zorder=4)

    # Layer 4: 转弯圆心红色十字 + 扫掠半径 R_o / R_i 灰色构造圆弧标注
    metrics = res.metrics
    center = (0.0, 0.0)
    _crosshair(ax, center[0], center[1])
    from app.core.passability import _TURN_DEG, _RA_TURN_DEG
    turn_deg = _TURN_DEG if scene == "arc" \
        else float(metrics.get("turn_deg", _RA_TURN_DEG))
    r_o = float(metrics.get("R_o", 0.0) or 0.0)
    r_i = float(metrics.get("R_i", 0.0) or 0.0)
    if r_o > 0.5:
        _annotate_radius(ax, center, r_o, turn_deg, f"R_o = {r_o:.3f} m", "out")
    if r_i > 0.5:
        _annotate_radius(ax, center, r_i, turn_deg, f"R_i = {r_i:.3f} m", "in")

    # Layer 5: 检验结论 + 图例（左下角）
    from matplotlib.lines import Line2D
    verdict = "通过" if pass_ok else "不通过"
    vc = "#2f855a" if pass_ok else "#c53030"
    info = f"检验结论：{verdict}"
    if "road_lane" in metrics:
        w_req = metrics.get("W", 0)
        lane = metrics.get("road_lane", 0)
        info += f"　所需宽度 W = {w_req:.2f} m / 实际 {lane:.2f} m"
    elif "entry_width" in metrics:
        info += (f"　入口 {metrics.get('entry_width', 0):.2f} / "
                 f"出口 {metrics.get('exit_width', 0):.2f} m，"
                 f"所需扫掠宽 W = {metrics.get('W', 0):.2f} m")
    ax.text(0.015, 0.97, info, transform=ax.transAxes, verticalalignment="top",
            horizontalalignment="left", fontsize=10.5, family="Microsoft YaHei",
            fontweight="bold", color=vc,
            bbox=dict(facecolor="white", alpha=0.9, edgecolor=vc, linewidth=1.0,
                      boxstyle="round,pad=0.4"))

    handles = [
        Line2D([0], [0], color=ROAD_COLOR, lw=1.9, label="道路边界"),
        Line2D([0], [0], color=SWEPT_COLOR, lw=1.0, ls=(0, (5, 3)),
               label="扫掠包络：整车全程占道范围"),
        Line2D([0], [0], color=VEHICLE_COLOR, lw=1.1, label="车辆轮廓（挂车 / 牵引车）"),
        Line2D([0], [0], color=CARGO_COLOR, lw=1.2, label="货物轮廓（超宽 / 超长凸出）"),
        Line2D([0], [0], color=DIM_COLOR, lw=0.9, label="转弯扫掠半径"),
        Line2D([0], [0], marker='+', color=CENTER_COLOR, ls='none', markersize=8,
               label="转弯圆心（车辆绕其转弯）"),
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
                 family="Microsoft YaHei", color=TEXT_COLOR)
    ax.set_xlabel("X (m)", fontsize=10, color="#666666")
    ax.set_ylabel("Y (m)", fontsize=10, color="#666666")

    plt.tight_layout()
    plt.savefig(out_path, dpi=dpi, bbox_inches="tight")
    plt.close(fig)
    return out_path
