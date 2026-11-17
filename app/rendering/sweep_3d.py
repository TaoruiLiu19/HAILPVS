"""三维查看平面扫掠与车货外廓，所有坐标以米计。"""

import math

from matplotlib.figure import Figure
from matplotlib.lines import Line2D
from matplotlib.patches import Patch
from matplotlib.ticker import FuncFormatter, MaxNLocator
from mpl_toolkits.mplot3d import art3d

from .plot_geometry import polygon_patches

OBLIQUE_VIEW = (36, -62)
TOP_VIEW = (90, -90)
SIDE_VIEW = (0, -90)

_FONT = "Microsoft YaHei"
_INK = "#19334f"
_MUTED = "#526981"
_ROAD = "#e9eef3"
_ROAD_EDGE = "#73869b"
_SWEEP = "#b9d8f5"
_SWEEP_EDGE = "#477eaf"
_DECK = "#487aa3"
_CARGO = "#79b9a5"
_CARGO_EDGE = "#215d55"
_AXLE = "#713fbe"
_EXCESS = "#b03637"
_TOLERANCE = "#92724c"


def _plane(ax, geometry, z=0, **style):
    for patch in polygon_patches(geometry, **style):
        ax.add_patch(patch)
        art3d.pathpatch_2d_to_3d(patch, z=z)


def _prism_faces(footprint, z_base, z_top):
    ring = list(footprint.exterior.coords)[:-1]
    bottom = [(x, y, z_base) for x, y in ring]
    top = [(x, y, z_top) for x, y in ring]
    walls = [[bottom[i - 1], bottom[i], top[i], top[i - 1]] for i in range(len(ring))]
    return [*walls, top]


def _steering_lines(info):
    psi = info["psi"]
    lateral_x, lateral_y = -math.sin(psi), math.cos(psi)
    for _, x, y, delta_deg, half_track, _ in info["axles"]:
        ends = [(x + sign * lateral_x * half_track, y + sign * lateral_y * half_track)
                for sign in (-1, 1)]
        yield [(wheel_x, wheel_y, 0) for wheel_x, wheel_y in ends]
        angle = psi + math.radians(delta_deg)
        dx, dy = 0.45 * math.cos(angle), 0.45 * math.sin(angle)
        for wheel_x, wheel_y in ends:
            yield [(wheel_x - dx, wheel_y - dy, 0), (wheel_x + dx, wheel_y + dy, 0)]


def _draw_vehicles(ax, res):
    deck_z = res.metrics["deck_height"]
    cargo_top = res.metrics["total_height"]
    tractor_z = res.metrics["tractor_height"]
    # 将所有三维面交给同一个 collection，旋转时按观察方向排序。
    faces, fills, edges, steering = [], [], [], []
    for frame in res.vehicle_frames:
        _, _, body, deck, cargo = frame[:5]
        plate = _prism_faces(deck, deck_z - min(0.12, deck_z * 0.1), deck_z)
        load = _prism_faces(cargo, deck_z, cargo_top)
        faces.extend(plate + load)
        fills.extend([_DECK] * len(plate) + ["#62a38d"] * (len(load) - 1) + [_CARGO])
        edges.extend([_INK] * len(plate) + [_CARGO_EDGE] * len(load))
        if res.hydraulic and len(frame) > 5 and frame[5]:
            steering.extend(_steering_lines(frame[5]))
        if not res.hydraulic and body.geom_type == "MultiPolygon":
            tractor = body.geoms[-1]
            if tractor_z > 0:
                cab = _prism_faces(tractor, 0, tractor_z)
                faces.extend(cab)
                fills.extend([_INK] * len(cab))
                edges.extend([_INK] * len(cab))
            else:
                _plane(ax, tractor, facecolor="#d5e1ee", edgecolor=_INK,
                       linestyle="--", linewidth=1.1, zorder=5)
    if steering:
        ax.add_collection3d(art3d.Line3DCollection(steering, colors=_AXLE, linewidths=1.45,
                                                  zorder=5))
    if faces:
        ax.add_collection3d(art3d.Poly3DCollection(faces, facecolors=fills, edgecolors=edges,
                                                  linewidths=0.85, zorder=6))


def _draw_ground(ax, res):
    _plane(ax, res.road, facecolor=_ROAD, edgecolor=_ROAD_EDGE, linewidth=1.2, zorder=1)
    _plane(ax, res.swept, facecolor=_SWEEP, edgecolor=_SWEEP_EDGE, linewidth=0.85, zorder=2)
    _plane(ax, res.cargo_sweep, facecolor="none", edgecolor=_CARGO_EDGE,
           linestyle="--", linewidth=1.2, zorder=3)
    overrun_legend = {}
    for body, geometry in (("vehicle", res.vehicle_overrun), ("cargo", res.cargo_overrun)):
        if geometry is None or geometry.is_empty:
            continue
        ok = res.metrics[f"split_{body}_ok"]
        color = _TOLERANCE if ok else _EXCESS
        fill = "#f6e1c3" if ok else "#edb3b2"
        _plane(ax, geometry, facecolor=fill, edgecolor=color, linewidth=1.2, zorder=4)
        label = "容差内侵入" if ok else "超限区域"
        overrun_legend[label] = Patch(fc=fill, ec=color, label=label)
    path = res.travel_path
    if len(path) > 1:
        xs, ys = zip(*path)
        ax.plot(xs, ys, zs=0, color=_INK, lw=0.95, ls="--", zorder=4)
        for fraction in (0.20, 0.50, 0.82):
            end = min(len(path) - 1, max(1, round((len(path) - 1) * fraction)))
            start = max(0, end - max(1, round(len(path) * 0.035)))
            x, y = path[start]
            dx, dy = path[end][0] - x, path[end][1] - y
            ax.quiver(x, y, 0, dx, dy, 0, color=_INK, linewidth=1.5,
                      arrow_length_ratio=0.45, zorder=4)
    return list(overrun_legend.values())


def _set_view_limits(ax, res):
    bounds = [g.bounds for g in (res.road, res.swept) if g is not None and not g.is_empty]
    x0 = min(b[0] for b in bounds)
    y0 = min(b[1] for b in bounds)
    x1 = max(b[2] for b in bounds)
    y1 = max(b[3] for b in bounds)
    z1 = max(res.metrics["total_height"], res.metrics["tractor_height"]) * 1.35
    pad = 0.04 * max(x1 - x0, y1 - y0)
    xlim = (x0 - pad, x1 + pad)
    if res.metrics.get("turn_direction") == "右转":
        xlim = xlim[::-1]
        pivot_x = res.metrics["corner_point"][0]
        ax.xaxis.set_major_formatter(FuncFormatter(lambda x, _: f"{2 * pivot_x - x:g}"))
    ax.set(xlim=xlim, ylim=(y0 - pad, y1 + pad), zlim=(0, z1))
    ax.set_box_aspect((abs(xlim[1] - xlim[0]), y1 - y0 + 2 * pad, z1))


def build_3d_figure(res, figsize=(12, 10)):
    """同一三维图供 Qt 交互窗口、主窗口预览和报告出图使用。"""
    fig = Figure(figsize=figsize, facecolor="white")
    ax = fig.add_axes((.04, .152, .92, .742), projection="3d", proj_type="ortho",
                      computed_zorder=False)
    ax.view_init(*OBLIQUE_VIEW)
    extras = _draw_ground(ax, res)
    _draw_vehicles(ax, res)
    _set_view_limits(ax, res)
    ax.grid(False)
    for axis in (ax.xaxis, ax.yaxis, ax.zaxis):
        axis.set_major_locator(MaxNLocator(nbins=4))
        axis.pane.set_facecolor("#f5f7fa")
        axis.pane.set_edgecolor("#e2e8ef")
    ax.zaxis.set_major_locator(MaxNLocator(nbins=3))
    ax.set(xlabel="X (m)", ylabel="Y (m)", zlabel="Z (m)")
    ax.tick_params(colors=_MUTED, labelsize=10, pad=1)

    m = res.metrics
    corner = "圆弧弯道" if res.scene == "arc" else f"直角弯道 · {m['turn_direction']}"
    vehicle_type = "液压轴线板车全轮转向" if res.hydraulic else "机械挂车铰接尾随"
    fig.text(.055, .953, f"{corner} / 三维扫掠", ha="left", va="top",
             family=_FONT, color=_INK, weight="bold", fontsize=23)
    fig.text(.055, .900, f"{vehicle_type}  ·  平面扫掠投影与车货外廓", ha="left", va="top",
             family=_FONT, color=_MUTED, fontsize=13)
    status_color = "#80551b" if res.pending else "#23664d" if res.pass_ok else _EXCESS
    fig.text(.945, .944, res.verdict, ha="right", va="top", family=_FONT,
             color=status_color, weight="bold", fontsize=15)

    handles = [
        Patch(fc=_ROAD, ec=_ROAD_EDGE, label="道路"),
        Patch(fc=_SWEEP, ec=_SWEEP_EDGE, label="板车及货物扫掠" if res.hydraulic else "车货扫掠"),
        Patch(fc=_DECK, label="车板"),
        Patch(fc=_CARGO, ec=_CARGO_EDGE, label="货物"),
        Line2D([0], [0], color=_INK, ls="--", marker=">", label="行驶方向"),
    ]
    if res.cargo_sweep is not None and not res.cargo_sweep.is_empty:
        handles.append(Line2D([0], [0], color=_CARGO_EDGE, ls="--", label="外伸货物扫掠"))
    if res.hydraulic:
        handles.append(Line2D([0], [0], color=_AXLE, lw=1.4, label="转向轴及轮向"))
    elif m["tractor_height"] > 0:
        handles.append(Patch(fc=_INK, label="牵引车"))
    else:
        handles.append(Patch(fc="#d5e1ee", ec=_INK, ls="--", label="牵引车平面轮廓"))
    fig.legend(handles=handles + extras, loc="lower left", bbox_to_anchor=(.049, .104),
               ncol=5, frameon=False, prop=dict(family=_FONT, size=11.5))

    gap = m["vertical_clearance_gap"]
    height = f"车板 {m['deck_height']:.2f} m    车货总高 {m['total_height']:.2f} m"
    clearance = f"顶部间隙 {gap:.3f} m" if gap is not None else f"顶部净空 {m['vertical_clearance_status']}"
    fig.text(.055, .075, height, family=_FONT, fontsize=13, color=_INK)
    fig.text(.945, .075, clearance, ha="right", family=_FONT, fontsize=13,
             color=_EXCESS if m["vert_clearance_ok"] is False else _INK)
    return fig, ax
