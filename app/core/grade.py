"""大件运输尺寸分级与查表法（JTG/T 2213—2023）。

分级依据规范第 3.2.4 条“基于车货总体外廓尺寸的大件运输分级”（表3.2.4），
按车货总体总长度、总宽度、总高度分别确定级别，取最高级。

查表法依据第 4.3.1 条（圆曲线可通行性，表4.3.1）。表内数据为 OCR 复核后
的最小半径值；当分级数据缺失（如 E 尺寸级最大扫空空间宽度）或圆曲线半径
小于表值时，调用方应回退到附录 B 验算法 / 附录 C 仿真法。
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
# 总宽度 (m)
_WIDTH_BANDS = [
    (2.55, 3.00, "A"),
    (3.00, 3.50, "B"),
    (3.50, 3.75, "C"),
    (3.75, 4.50, "D"),
    (4.50, float("inf"), "E"),
]
# 总高度 (m)：A/B/C 上限 4.50，D 上限 5.00，E >5.00
_HEIGHT_BANDS = [
    (float("-inf"), 4.00, "A"),
    (4.00, 4.50, "C"),
    (4.50, 5.00, "D"),
    (5.00, float("inf"), "E"),
]

# 表4.3.1 圆曲线可通行性：最小半径 R_min (m)。
# 最大扫空空间宽度列受 OCR 完整度限制未全复原，缺失项用 None（回退验算/仿真）。
TABLE_4_3_1_RMIN = {
    "A": 8.6,
    "B": 11.2,
    "C": 17.7,
    "D": 20.2,
    "E": None,
}


def classify(l: float, b: float, h: float, hydraulic: bool) -> str:
    """按表3.2.4 综合长/宽/高得出 A–E 尺寸级，取最高（3.2.4 注 1）。"""
    length_table = _LENGTH_HYDRAULIC if hydraulic else _LENGTH_LOWBED
    grades = {_band_grade(l, length_table),
              _band_grade(b, _WIDTH_BANDS),
              _band_grade(h, _HEIGHT_BANDS)}
    return max(grades, key=lambda g: _ORDER[g])


def classify_bundle(bundle: InputBundle) -> str:
    """由输入 Bundle 求尺寸级。总长为车货总体（挂车/货物取大），
    液压轴线挂车按表3.2.4 液压悬挂挂车长区间分级。
    """
    tl = bundle.trailer
    cargo = bundle.cargo
    total_l = max(tl.length, cargo.length)
    total_b = max(tl.width, cargo.width)
    total_h = tl.deck_height + cargo.height
    hydraulic = "液压" in tl.trailer_type
    return classify(total_l, total_b, total_h, hydraulic)


def arch_lookup_min_radius(grade: str):
    """表4.3.1 某级的最小半径。无数据（E）返回 None。"""
    return TABLE_4_3_1_RMIN.get(grade)


def arch_lookup_apply(grade: str, curve_radius: float) -> bool | None:
    """查表法 4.3.1.1：圆曲线半径 ≥ 表值即满足最小半径要求。

    返回 True（满足）/ False（不满足）/ None（该级无表数据，需回退验算）。
    """
    r_min = arch_lookup_min_radius(grade)
    if r_min is None:
        return None
    return curve_radius > r_min + 1e-9