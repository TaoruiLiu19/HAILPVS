"""渲染层回归（M11）：图形几何正确性与报告着色一致性。

覆盖：
  - `_rotated_rect` 轮胎/矩形四角随航向不畸变（原手写四角 tsa 符号错误，
    ψ=90°/270° 时退化为零面积线段）；
  - 结论文本判定词着色：同一行同时含"满足"与"不通过"时须逐词着色，
    Word 报告不得把"不通过"染成绿色。
"""
import math
import os
import sys
import warnings

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import pytest

from app.core.datamodel import (InputBundle, CargoParams, TrailerParams,
                                TractorParams, RoadParams, ArcCorner)
from app.core.passability import run_scene
from app.rendering.sweep_view import _rotated_rect
from app.rendering.verdict_style import (verdict_color, split_verdict_segments,
                                         COLOR_GREEN, COLOR_RED)


def _poly_area(pts):
    s = 0.0
    n = len(pts)
    for i in range(n):
        x1, y1 = pts[i]
        x2, y2 = pts[(i + 1) % n]
        s += x1 * y2 - x2 * y1
    return abs(s) / 2.0


# ---- 轮胎/矩形四角几何 ----
def test_rotated_rect_area_constant_across_headings():
    # 矩形面积应与航向无关（恒定 = 长 × 宽）
    half_len, half_wid = 1.1 / 2.0, 0.616 / 2.0
    want = 1.1 * 0.616
    for deg in range(0, 360, 5):
        pts = _rotated_rect(0.0, 0.0, math.radians(deg), half_len, half_wid)
        assert len(pts) == 4
        assert _poly_area(pts) == pytest.approx(want, abs=1e-9), f"deg={deg}"


def test_rotated_rect_is_rectangle_not_bowtie():
    # 四角须构成真矩形：对边等长、邻边垂直、对角线相等
    half_len, half_wid = 0.55, 0.308
    for deg in (0, 30, 45, 90, 135, 230, 270, 315):
        p = _rotated_rect(3.0, -2.0, math.radians(deg), half_len, half_wid)
        sides = [math.dist(p[i], p[(i + 1) % 4]) for i in range(4)]
        assert sides[0] == pytest.approx(sides[2], abs=1e-9)
        assert sides[1] == pytest.approx(sides[3], abs=1e-9)
        assert {round(sides[0], 9), round(sides[1], 9)} == \
            {round(2 * half_len, 9), round(2 * half_wid, 9)}
        # 邻边垂直
        for i in range(4):
            a = (p[(i + 1) % 4][0] - p[i][0], p[(i + 1) % 4][1] - p[i][1])
            b = (p[(i + 2) % 4][0] - p[(i + 1) % 4][0],
                 p[(i + 2) % 4][1] - p[(i + 1) % 4][1])
            assert a[0] * b[0] + a[1] * b[1] == pytest.approx(0.0, abs=1e-9)


def test_rotated_rect_matches_manual_rotation():
    # 局部角点 (lx,ly) 旋转后须与手算一致（航向 90°：长边沿 y）
    p = _rotated_rect(0.0, 0.0, math.radians(90.0), 3.0, 1.5)
    xs = sorted({round(x, 9) for x, _y in p})
    ys = sorted({round(y, 9) for _x, y in p})
    assert xs == [-1.5, 1.5]
    assert ys == [-3.0, 3.0]


# ---- 判定词着色 ----
def test_verdict_color_none_on_polarity_conflict():
    line = "查表法满足（4.3.1-1），但 4.2.1 内/外缘横向预留不足 → 不通过。"
    assert verdict_color(line) is None          # 极性冲突 → 不整段上色
    assert verdict_color("W = 3.49 m ≤ 路面 7.00 m → 满足") == COLOR_GREEN
    assert verdict_color("W = 8.46 m > 路面 7.00 m → 不满足") == COLOR_RED
    assert verdict_color("弯道类型：圆弧弯道") is None


def test_split_verdict_segments_colors_per_word():
    line = "查表法满足（4.3.1-1），但 4.2.1 内/外缘横向预留不足 → 不通过。"
    segs = split_verdict_segments(line)
    assert "".join(s for s, _c in segs) == line          # 无损切分
    colored = {s: c for s, c in segs if c}
    assert colored["满足"] == COLOR_GREEN
    assert colored["不通过"] == COLOR_RED


def _failing_lookup_bundle():
    """示例1 工况：查表法命中但 4.2.1 内/外缘净空为负 → 行内极性冲突。"""
    return InputBundle(
        cargo=CargoParams(weight_t=60, length=10, width=3.0, height=3.0,
                          cg_height=1.2),
        trailer=TrailerParams(trailer_type="重型机械挂车", length=12.0, width=3.0,
                              deck_height=1.2, axle_count=3, axle_spacing=1.5,
                              track_width=2.9),
        tractor=TractorParams(wheelbase=4.0, max_steering_angle_deg=45,
                              length=7.0, width=2.5, fifth_wheel_pos=1.0),
        road=RoadParams(road_type="圆弧弯道",
                        arc=ArcCorner(outer_radius=30.0, lane_width=7.0),
                        vertical_clearance=7.5, highway_grade="三级公路"),
        max_speed_kmh=10.0)


def test_word_report_no_run_mixes_verdict_polarity():
    # Word 报告的每个 run 不得同时含"满足"与"不通过"，且含"不通过"的 run 不得为绿色
    docx = pytest.importorskip("docx")
    from app.rendering.report_assembler import assemble_and_save

    res = run_scene(_failing_lookup_bundle())
    assert res.pass_ok is False
    assert any("不通过" in ln and "满足" in ln for ln in res.verdict_lines)

    import tempfile
    path = os.path.join(tempfile.mkdtemp(), "r.docx")
    assemble_and_save(res, path)
    doc = docx.Document(path)

    green_rgb = docx.shared.RGBColor(0x2f, 0x85, 0x5a)
    for p in doc.paragraphs:
        for r in p.runs:
            t = r.text or ""
            assert not ("满足" in t and "不通过" in t), f"极性混染: {t!r}"
            if "不通过" in t:
                assert r.font.color.rgb != green_rgb, f"'不通过'被染绿: {t!r}"


# ---- 视图范围须覆盖标注（否则最终图靠 bbox_inches 兜底、比例不稳）----
def _hydraulic_arc_bundle():
    from app.core.datamodel import ArcCorner
    return InputBundle(
        cargo=CargoParams(weight_t=60, length=10, width=3.0, height=3.0,
                          cg_height=1.2),
        trailer=TrailerParams(trailer_type="液压轴线挂车", length=12.0, width=3.0,
                              deck_height=1.2, axle_count=4, axle_spacing=1.5,
                              track_width=2.9),
        tractor=TractorParams(wheelbase=4.0, max_steering_angle_deg=45,
                              length=7.0, width=2.5, fifth_wheel_pos=1.0),
        road=RoadParams(road_type="圆弧弯道",
                        arc=ArcCorner(outer_radius=30.0, lane_width=7.0),
                        vertical_clearance=7.5, highway_grade="三级公路"),
        max_speed_kmh=10.0)


def _right_angle_bundle():
    from app.core.datamodel import RightAngleCorner
    return InputBundle(
        cargo=CargoParams(weight_t=80, length=10, width=5.0, height=3.0,
                          cg_height=1.2, is_overhang=True),
        trailer=TrailerParams(trailer_type="液压轴线挂车", length=12.0, width=3.0,
                              deck_height=1.2, axle_count=4, axle_spacing=1.5,
                              track_width=2.9),
        tractor=TractorParams(wheelbase=4.0, max_steering_angle_deg=45,
                              length=7.0, width=2.5, fifth_wheel_pos=1.0),
        road=RoadParams(road_type="直角弯道",
                        right_angle=RightAngleCorner(entry_width=18.5,
                                                     exit_width=14.0,
                                                     corner_radius=6.0,
                                                     intersection_angle_deg=90.0),
                        vertical_clearance=7.5, highway_grade="三级公路"),
        max_speed_kmh=10.0)


def _anchors(ax):
    out = []
    for t in ax.texts:
        for p in (getattr(t, "xy", None), getattr(t, "xyann", None)):
            if p is not None:
                out.append((float(p[0]), float(p[1])))
        pos = t.get_position()
        out.append((float(pos[0]), float(pos[1])))
    return out


@pytest.mark.parametrize("factory", [_failing_lookup_bundle,
                                     _hydraulic_arc_bundle,
                                     _right_angle_bundle])
def test_all_annotations_inside_view_limits(factory):
    matplotlib = pytest.importorskip("matplotlib")
    matplotlib.use("Agg")
    from app.rendering.sweep_view import build_figure, _extents

    res = run_scene(factory())
    fig, ax = build_figure(res)
    try:
        x0, x1 = ax.get_xlim()
        y0, y1 = ax.get_ylim()
        # 确有标注画在"道路 ∪ 扫掠"之外（尺寸线外移），否则本用例无意义
        bx0, bx1, by0, by1 = _extents(res.road, res.swept)
        anchors = _anchors(ax)
        assert any(not (bx0 <= x <= bx1 and by0 <= y <= by1)
                   for x, y in anchors), "本工况无界外标注，用例失去意义"
        # 而它们必须全部落在视图范围内
        for x, y in anchors:
            assert x0 <= x <= x1 and y0 <= y <= y1, f"标注越出视图: ({x:.2f}, {y:.2f})"
    finally:
        matplotlib.pyplot.close(fig)


def test_view_metrics_come_from_result_not_hardcoded():
    # 转弯圆心与仿真转角由计算层 metrics 提供，渲染层不再硬编码/引用私有常量
    res = run_scene(_hydraulic_arc_bundle())
    assert res.metrics["turn_center"] == (0.0, 0.0)
    assert res.metrics["turn_deg"] == pytest.approx(140.0)
    ra = run_scene(_right_angle_bundle()).metrics
    assert ra["turn_center"] == (0.0, 0.0)
    assert ra["turn_deg"] == pytest.approx(90.0)


@pytest.mark.parametrize("factory", [_failing_lookup_bundle,
                                     _hydraulic_arc_bundle,
                                     _right_angle_bundle])
def test_no_missing_glyph_warnings(factory, tmp_path):
    # 图内文字不得使用 Microsoft YaHei 缺字形的字符（如 U+2082 下标₂、
    # U+2081 等 Unicode 下标），否则会渲染成缺字方块。
    # 图内统一改用 ASCII（如 ΔW2）；需要真下标时只在报告富文本里用 <sub>。
    matplotlib = pytest.importorskip("matplotlib")
    matplotlib.use("Agg")
    from app.rendering.sweep_view import draw_sweep_view

    res = run_scene(factory())
    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always")
        draw_sweep_view(res, str(tmp_path / "sweep.png"))
    bad = [str(w.message) for w in caught if "missing from font" in str(w.message)]
    assert not bad, f"图内出现缺字形字符：{bad}"
