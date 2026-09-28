# -*- coding: utf-8 -*-
"""结论文本（检验过程及结论）中判定词的着色规范。

界面富文本用 <span style="color:..."> 直接着色；Word 报告由
report_assembler 依据判定词把对应 run 染成红/绿。判定词是搭在结论文本里
的，与 <sub>/<sup>/&lt; 等标记不冲突，可安全地在整行字符串上匹配。
"""
import re

COLOR_GREEN = "#2f855a"      # 通过 / 满足 / 合格
COLOR_RED = "#c53030"        # 不通过 / 不满足 / 不合格

# “不通过/不满足/不合格”在前，避免被“通过/满足/合格”拆成两半
_VERDICT_RE = re.compile(r"不通过|不满足|不合格|通过|满足|合格")
# 同一表达式带捕获组，用于把整行切成 片段/判定词 交替序列（逐词着色）
_VERDICT_SPLIT_RE = re.compile(r"(不通过|不满足|不合格|通过|满足|合格)")


def _is_fail(word: str) -> bool:
    return word.startswith("不")


def verdict_color(text: str):
    """返回 text 的整体判定颜色：COLOR_RED / COLOR_GREEN / None。

    **同一文本内判定词极性冲突时返回 None**——整段着色的渲染器无法同时表达
    "满足"与"不通过"两种极性，此前按"首个判定词"上色会把
    「查表法满足…→ 不通过。」整行染成绿色。需要逐词着色请用
    `split_verdict_segments`。
    """
    polars = {w.startswith("不") for w in _VERDICT_RE.findall(text)}
    if len(polars) != 1:
        return None
    return COLOR_RED if polars.pop() else COLOR_GREEN


def split_verdict_segments(text: str):
    """把 text 切成 [(片段, 颜色或 None), ...]，供 Word 等逐词着色的渲染器使用。

    判定词片段给出红/绿，其余片段为 None（沿用默认字色）。
    """
    out = []
    for seg in _VERDICT_SPLIT_RE.split(text):
        if not seg:
            continue
        is_word = _VERDICT_SPLIT_RE.fullmatch(seg) is not None
        out.append((seg, (COLOR_RED if _is_fail(seg) else COLOR_GREEN)
                    if is_word else None))
    return out


def colorize_verdict_html(text: str) -> str:
    """把整行文本中的判定词用 <span style="color:..."> 包裹（Qt 富文本渲染）。"""
    def _repl(m):
        w = m.group(0)
        color = COLOR_RED if _is_fail(w) else COLOR_GREEN
        return f'<span style="color:{color};">{w}</span>'
    return _VERDICT_RE.sub(_repl, text)