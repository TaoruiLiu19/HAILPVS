"""大件运输尺寸分级与查表法（JTG/T 2213—2023）。

分级依据规范第 3.2.4 条“基于车货总体外廓尺寸的大件运输分级”（表3.2.4），
按车货总体总长度、总宽度、总高度分别确定级别，取最高级。

查表法依据第 4.3.1 条（圆曲线可通行性，表4.3.1）。表内"最小半径"与
"最大扫空空间宽度"双列数据均已按规范复原；当分级数据缺失（E 尺寸级）或
圆曲线半径 / 通行空间宽度小于等于表值时，调用方应回退到附录 B 验算法 /
附录 C 仿真法（查表优先，非无条件覆盖）。
"""
from __future__ import annotations

from .datamodel import InputBundle

GRADES = ["A", "B", "C", "D", "E"]
_ORDER = {g: i for i, g in enumerate(GRADES)}


def _band_grade(value, table):
    """区间表 table = [(lo, hi, grade), ...]，返回 value 所在级的字符。"""
    for lo, hi, g in table:
        if lo < value <= hi:
            return g
    return GRADES[-1]


# 表3.2.4 —— 总长度分级 (m)。低平板半挂车 / 多轴多轮液压悬挂挂车 两套区间。
_LENGTH_LOWBED = [
    (float("-inf"), 17.0, "A"),
    (17.0, 22.0, "B"),
    (22.0, 30.0, "C"),
    (30.0, 35.0, "D"),
    (35.0, float("inf"), "E"),
]
_LENGTH_HYDRAULIC = [
    (float("-inf"), 22.0, "A"),
    (22.0, 31.0, "B"),
    (31.0, 41.0, "C"),
    (41.0, 45.0, "D"),
    (45.0, float("inf"), "E"),
]
# 总宽度 (m)：A 下限 2.55（≤2.55 视为非超限车辆，取 A）
_WIDTH_BANDS = [
    (float("-inf"), 2.55, "A"),
    (2.55, 3.00, "A"),
    (3.00, 3.50, "B"),
    (3.50, 3.75, "C"),
    (3.75, 4.50, "D"),
    (4.50, float("inf"), "E"),
]
# 总高度 (m)：规范 3.2.4 中 A/B/C 三行高度同区为 (4.00,4.50]，区间内子级由
# 长/宽维度决定。高度维度只在跨越 4.5 / 5.0 边界时抬到 D/E，不再抢档
# （修正前把 (4.00,4.50] 固定记 C，导致长宽均 A、总高 4.3 m 的车被误判为 C）。
_HEIGHT_BANDS = [
    (float("-inf"), 4.50, "A"),       # ≤4.50 → 高度不抬级，A/B/C 由长宽决定
    (4.50, 5.00, "D"),
    (5.00, float("inf"), "E"),
]

# 表4.3.1 圆曲线可通行性（双列）：min_radius = 最小半径(m)，max_swept_width = 最大扫空空间宽度(m)。
# 规范 4.3.1 第1款：圆曲线半径与通行空间宽度**均大于**表值才判满足；任一不满足或 E 级回退验算/仿真。
TABLE_4_3_1 = {
    "A": {"min_radius": 11.0, "max_swept_width": 8.6},
    "B": {"min_radius": 15.0, "max_swept_width": 11.2},
    "C": {"min_radius": 22.0, "max_swept_width": 17.7},
    "D": {"min_radius": 28.0, "max_swept_width": 20.2},
    "E": None,   # 表4.3.1 无 E 级数据 → 回退验算/仿真
}


def classify(l: float, b: float, h: float, hydraulic: bool) -> str:
    """按表3.2.4 综合长/宽/高得出 A–E 尺寸级，取最高（3.2.4 注 1）。"""
    length_table = _LENGTH_HYDRAULIC if hydraulic else _LENGTH_LOWBED
    grades = {_band_grade(l, length_table),
              _band_grade(b, _WIDTH_BANDS),
              _band_grade(h, _HEIGHT_BANDS)}
    return max(grades, key=lambda g: _ORDER[g])


def classify_bundle(bundle: InputBundle) -> str:
    """由输入 Bundle 求尺寸级。总长为车货总体（挂车/货物取大）+ 牵引车车身长
    （表3.2.4 “总长度”为牵引车+半挂车整车总长；trailer.length 为车板长，
    故补加 bundle.tractor.length）。液压轴线挂车按表3.2.4 液压悬挂挂车长区间分级。
    """
    tl = bundle.trailer
    cargo = bundle.cargo
    total_l = max(tl.length, cargo.length) + bundle.tractor.length
    total_b = max(tl.width, cargo.width)
    total_h = tl.deck_height + cargo.height
    hydraulic = "液压" in tl.trailer_type
    return classify(total_l, total_b, total_h, hydraulic)


def arch_lookup_min_radius(grade: str):
    """表4.3.1 某级的最小半径。无数据（E）返回 None。"""
    row = TABLE_4_3_1.get(grade)
    return row["min_radius"] if row else None


def arch_lookup_min_swept_width(grade: str):
    """表4.3.1 某级的最大扫空空间宽度。无数据（E）返回 None。"""
    row = TABLE_4_3_1.get(grade)
    return row["max_swept_width"] if row else None


def arch_lookup_apply(grade: str, curve_radius: float,
                      transverse_space_w: float, swept_w: float) -> str:
    """查表法 4.3.1 第1款：圆曲线半径与通行空间宽度**均大于**表值才判满足。

    返回 "pass"（查表命中，半径与通行空间宽度双超表值，直接满足）
        / "fail"（任一不满足，按附录B验算/附录C仿真）
        / "fallback"（该级无表数据，如 E 级，回退验算/仿真）。
    """
    row = TABLE_4_3_1.get(grade)
    if row is None:
        return "fallback"
    r_ok = curve_radius > row["min_radius"] + 1e-9
    w_ok = (transverse_space_w > row["max_swept_width"] + 1e-9
            and swept_w < transverse_space_w)
    if r_ok and w_ok:
        return "pass"
    return "fail"