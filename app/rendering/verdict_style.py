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


def _is_fail(word: str) -> bool:
    return word.startswith("不")


def verdict_color(text: str):
    """返回 text 中出现的判定词颜色：'red' / 'green' / None。"""
    for m in _VERDICT_RE.finditer(text):
        return COLOR_RED if _is_fail(m.group(0)) else COLOR_GREEN
    return None


def colorize_verdict_html(text: str) -> str:
    """把整行文本中的判定词用 <span style="color:..."> 包裹（Qt 富文本渲染）。"""
    def _repl(m):
        w = m.group(0)
        color = COLOR_RED if _is_fail(w) else COLOR_GREEN
        return f'<span style="color:{color};">{w}</span>'
    return _VERDICT_RE.sub(_repl, text)