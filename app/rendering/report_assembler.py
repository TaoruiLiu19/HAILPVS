"""Word 弯道通过性检验报告生成。

纯文档构造逻辑，不依赖 Qt 控件；文件选择与保存由界面层负责。
参照 LasingSystem2.0 报告排版惯例：楷体_GB2312、黑色、紧凑表格。
"""
import datetime
import os
import re
import tempfile

from docx import Document
from docx.shared import Pt, Inches, RGBColor
from docx.oxml.ns import qn
from docx.enum.text import WD_ALIGN_PARAGRAPH

from ..core.passability import SweepScenarioResult
from ..rendering.sweep_view import draw_sweep_view
from ..rendering.verdict_style import split_verdict_segments


# ===== 字体工具（复用参考项目 set_font_kai）=====
def set_font_kai(run, font_size=12, bold=False):
    run.font.name = "楷体_GB2312"
    run._element.rPr.rFonts.set(qn("w:eastAsia"), "楷体_GB2312")
    run.font.size = Pt(font_size)
    run.font.color.rgb = RGBColor(0, 0, 0)
    run.font.bold = bold


def _hex_rgb(hexstr: str) -> RGBColor:
    h = hexstr.lstrip("#")
    return RGBColor(int(h[0:2], 16), int(h[2:4], 16), int(h[4:6], 16))


def _add_param_table(doc, rows, title_text):
    h = doc.add_heading("", level=1)
    hr = h.add_run(title_text)
    set_font_kai(hr, 14, bold=True)
    tbl = doc.add_table(1, 2)
    hdr = tbl.rows[0].cells
    a = hdr[0].paragraphs[0].add_run("参数名称"); set_font_kai(a, 12, bold=True)
    b = hdr[1].paragraphs[0].add_run("数值"); set_font_kai(b, 12, bold=True)
    for name, val in rows:
        c = tbl.add_row().cells
        rn = c[0].paragraphs[0].add_run(str(name)); set_font_kai(rn, 12)
        rv = c[1].paragraphs[0].add_run(str(val)); set_font_kai(rv, 12)
    return tbl


def _add_para(doc, text, size=12, bold=False):
    p = doc.add_paragraph()
    r = p.add_run(text)
    set_font_kai(r, size, bold=bold)
    return p


_SUBSUP_TAGS = re.compile(r'(<sub>|</sub>|<sup>|</sup>|<b>|</b>|<i>|</i>)')


def _add_formula_para(doc, text, size=12, bold=False):
    """按 <sub>/<sup>/<b>/<i> 分段写入，生成真正带上下标/加粗的 Word 段落。

    与界面富文本共用同一套轻量 HTML 标记；其余实体（&lt; 等）在此解码。
    """
    p = doc.add_paragraph()
    sub = sup = italic = False
    inner_bold = bold
    for tok in _SUBSUP_TAGS.split(text):
        if not tok:
            continue
        if tok == "<sub>":
            sub, sup = True, False
            continue
        if tok == "<sup>":
            sup, sub = True, False
            continue
        if tok == "</sub>" or tok == "</sup>":
            sub = sup = False
            continue
        if tok == "<i>" or tok == "</i>":
            italic = tok == "<i>"
            continue
        if tok == "<b>" or tok == "</b>":
            inner_bold = inner_bold if tok == "</b>" else True
            continue
        tok = tok.replace("&lt;", "<").replace("&gt;", ">").replace("&amp;", "&")
        if not tok:
            continue
        # 逐判定词着色（不是整段一色）：同一行可能同时出现"满足"与"不通过"，
        # 整段上色会把"不通过"染成绿色。
        for seg, seg_color in split_verdict_segments(tok):
            r = p.add_run(seg)
            set_font_kai(r, size, bold=inner_bold)
            r.font.italic = italic
            if sub:
                r.font.subscript = True
            if sup:
                r.font.superscript = True
            if seg_color:
                r.font.color.rgb = _hex_rgb(seg_color)
    return p


def assemble_and_save(res: SweepScenarioResult, out_path):
    """按检验结果生成 Word 报告并保存。"""
    doc = Document()

    title = doc.add_heading("", level=0)
    t_run = title.add_run("大件运输弯道通过性检验报告")
    set_font_kai(t_run, 18, bold=True)
    title.alignment = WD_ALIGN_PARAGRAPH.CENTER

    tp = doc.add_paragraph()
    tr = tp.add_run(f"报告生成时间：{datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    set_font_kai(tr, 12)

    # —— 一、检验方法依据 ——
    from config import ACCEPTANCE_NOTE
    h1 = doc.add_heading("", level=1)
    hr = h1.add_run("一、检验方法依据")
    set_font_kai(hr, 14, bold=True)
    _add_para(doc, ACCEPTANCE_NOTE, 12)

    # —— 二、弯道输入参数 ——
    meta = _assemble_meta(res)
    _add_param_table(doc, meta.get("params", []), "二、弯道输入参数")

    # —— 三、车道与车辆扫掠结果 ——
    rows = []
    m = res.metrics
    rows.append(("公路等级", m.get('highway_grade', '')))
    rows.append(("横向预留空间 σ (m)", f"{m.get('sigma', 0):.3f}"))
    rows.append(("大件运输尺寸级", m.get('size_grade', '')))
    rows.append(("判定方法", m.get('method', '')))
    if res.scene == "arc":
        rows.append(("弯道类型", "圆弧弯道"))
        if m.get('trajectory_source') == "B.1.2":
            rows.append(("扫空量测口径", "仅板车+货物刚体几何（不含牵引车，B.1.2 条文说明）"))
        else:
            rows.append(("扫空量测口径", "铰接尾随仿真（包络含牵引车，B.1.1）"))
        # R_o 即"所需最小弯道外半径"（metrics['R_min'] 与 R_o 同值），合并为一行
        rows.append(("车辆扫掠外径 R_o（=所需最小弯道外半径）(m)", f"{m.get('R_o',0):.3f}"))
        rows.append(("车辆扫掠内径 R_i (m)", f"{m.get('R_i',0):.3f}"))
        rows.append(("扫空空间宽度 W_d / W_swept (m)", f"{m.get('W_needed',0):.3f}"))
        # metrics['W'] 与 'turn_channel_w' 同值，合并为一行
        rows.append(("转弯通道宽度 W_z（含0.5m安全宽度）(m)", f"{m.get('W',0):.3f}"))
        rows.append(("转弯通道宽度口径",
                     "液压：B.1.2-1 轮迹 R外−R内+σ（La=轮距）；机械：扫空宽度+σ"
                     if m.get('trajectory_source') == "B.1.2"
                     else "机械挂车：仿真扫空宽度 + 0.5m 安全宽度"))
        if m.get('trailer_turn_radius') is not None:
            rows.append(("车辆/板车实际走行半径 (m)", f"{m.get('trailer_turn_radius',0):.3f}"))
        rows.append(("路面宽度 (m)", f"{m.get('road_width',0):.3f}"))
        rows.append(("横向空间宽度 (m)", f"{m.get('transverse_space_w',0):.3f}"))
        rows.append(("外缘净空 (m)", f"{m.get('clearance_outer',0):.3f}"))
        rows.append(("内缘净空 (m)", f"{m.get('clearance_inner',0):.3f}"))
        rows.append(("内/外缘预留 σ 校核（4.2.1）",
                     "满足" if (m.get('clearance_outer', 0) >= m.get('sigma', 0)
                                and m.get('clearance_inner', 0) >= m.get('sigma', 0))
                     else "不满足"))
        if "B1_2_Rin" in m:
            rows.append(("液压挂车最内转弯半径 B.1.2 (m)", f"{m.get('B1_2_Rin',0):.3f}"))
            rows.append(("液压挂车最外转弯半径 B.1.2 (m)", f"{m.get('B1_2_Rout',0):.3f}"))
            rows.append(("液压挂车扫空宽度 Wd=Rmax−Rmin+σ (m)", f"{m.get('B1_2_Wd', m.get('B1_2_W',0)):.3f}"))
            rows.append(("液压挂车转弯通道宽度 W_z=R外(轮迹)−R内(轮迹)+σ (m)",
                         f"{m.get('B1_2_W_turn',0):.3f}"))
            rows.append(("液压 B.1.2-3 横项 La（取轮距）(m)", f"{m.get('B1_2_La',0):.3f}"))
            rows.append(("液压 θ 取值来源",
                         {"turn_radius": "按实际弯道半径反算 tanθ=d/(2R)",
                          "axle_max": "挂车轴结构最大转角（固有最不利）",
                          "explicit": "显式给定"}.get(m.get('B1_2_theta_source'), '')))
            rows.append(("液压动力机组长度 C (m)", f"{m.get('B1_2_power_unit_len',0):.3f}"))
            rows.append(("液压 θ(首轴转向角) B.1.2 (°)", f"{m.get('B1_2_theta_deg',0):.3f}"))
            rows.append(("液压车板总长 D_总 (m)", f"{m.get('B1_2_D_total',0):.3f}"))
            rows.append(("B.1.2 口径",
                         "严格守规范" if m.get('B1_2_spec_mode', True)
                         else "规范外推(首/末轴锁死)"))
            rows.append(("轨迹/扫空来源", m.get('trajectory_source', '')))
    else:
        rows.append(("弯道类型", "直角弯道(平面交叉转弯)"))
        rows.append(("R_m/W/ΔB 口径",
                     "仅板车+货物刚体（液压，不含牵引车）"
                     if m.get('hydraulic_trailer_only') else "铰接尾随仿真（含牵引车）"))
        rows.append(("入口宽度 W_en (m)", f"{m.get('entry_width',0):.3f}"))
        rows.append(("出口宽度 W_turn (m)", f"{m.get('exit_width',0):.3f}"))
        rows.append(("最内侧转弯半径 R_m (m)", f"{m.get('R_m',0):.3f}"))
        rows.append(("扫空空间最大宽度 W (m)", f"{m.get('W',0):.3f}"))
        rows.append(("车货总宽度 B_t (m)", f"{m.get('B_t',0):.3f}"))
        rows.append(("外摆值 ΔB (m)", f"{m.get('deltaB',0):.3f}"))
        rows.append(("牵引车出口加宽 ΔW2 (m)", f"{m.get('tractor_swing_widening',0):.3f}"))
        rows.append(("出口判定所需宽度 W+ΔW2 (m)", f"{m.get('exit_required_w',0):.3f}"))
        rows.append(("右转判定（5.2.1-1/-2）", "满足" if m.get('right_ok') else "不满足"))
        rows.append(("左转判定（5.2.1-3/-4，仅参考）", "满足" if m.get('left_ok') else "不满足"))
        rows.append(("所需最小出口转弯带 W2_min (m，参考)", f"{m.get('W2_min',0):.3f}"))
    rows.append(("车货总高 (m)", f"{m.get('total_height',0):.3f}"))
    rows.append(("垂直净空校验", "满足" if m.get('vert_clearance_ok') else "不满足"))
    rows.append(("侧滑/侧翻临界速度 (km/h)",
                 f"{m.get('Vch',0):.2f} / {m.get('Vcv',0):.2f}"))
    rows.append(("横向稳定性", "满足" if m.get('ok') else "不满足"))
    _add_param_table(doc, rows, "三、车道与车辆扫掠结果")

    # —— 四、扫掠轨迹图 ——
    h4 = doc.add_heading("", level=1)
    h4r = h4.add_run("四、弯道扫掠轨迹图")
    set_font_kai(h4r, 14, bold=True)
    img_path = _render_img(res)
    if img_path:
        try:
            doc.add_picture(img_path, width=Inches(6.2))
        finally:
            try:
                os.unlink(img_path)          # 临时 PNG 用完即删，避免 %TEMP% 累积
            except OSError:
                pass
    else:
        # 渲染失败不再静默省略，正文留痕便于排查
        _add_para(doc, "（扫掠轨迹图渲染失败，报告未附图；请检查图形依赖后重新导出。）",
                  12, bold=True)

    # —— 五、检验结论 ——
    h5 = doc.add_heading("", level=1)
    h5r = h5.add_run("五、检验结论")
    set_font_kai(h5r, 14, bold=True)
    for line in res.verdict_lines:
        _add_formula_para(doc, line, 12)
    verdict = "通过" if res.pass_ok else "不通过"
    _add_para(doc, f"综合评定：该运输车辆（牵引车-挂车-货物）在该弯道工况下可通行性判定为【{verdict}】。", 12, bold=True)

    doc.save(out_path)
    return out_path


def _assemble_meta(res: SweepScenarioResult):
    """汇总输入参数到 Word 表。由 MainWindow 在构造时传入轻量元数据。

    此处保持与界面对齐：若未提供 bundle 元数据，则仅回填结果内可得信息。
    """
    # 界面在 export 前已把输入元数据挂到 res.user_meta（dict）
    meta = getattr(res, "user_meta", None) or {}
    params = []
    mapping = [
        ("货物长度(m)", "c_length"),
        ("货物宽度(m)", "c_width"),
        ("货物高度(m)", "c_height"),
        ("重心高度(m)", "c_cg_height"),
        ("挂车长度(m)", "t_length"),
        ("挂车宽度(m)", "t_width"),
        ("车板上表面离地高度(m)", "t_deck_height"),
        ("挂车轴数", "t_axle_count"),
        ("挂车轴距(m)", "t_axle_spacing"),
        ("轮距(m)", "t_track_width"),
        ("牵引车轴距(m)", "tr_wheelbase"),
        ("最大转向角(°)", "tr_max_steering_angle_deg"),
        ("最高行驶速度(km/h)", "max_speed_kmh"),
    ]
    for name, key in mapping:
        if key in meta:
            params.append((name, meta[key]))
    if not params:
        params.append(("弯道类型", "圆弧弯道" if res.scene == "arc" else "直角弯道"))
    return {"params": params}


def _render_img(res):
    """把扫掠图渲染为临时 PNG 供 Word 内嵌。

    仅捕获"渲染不可用"类异常（缺依赖/后端问题）；程序性错误（如 KeyError）
    不吞，便于暴露真实缺陷。失败返回 None，由调用方在正文留痕。
    """
    try:
        fd = tempfile.NamedTemporaryFile(suffix=".png", delete=False)
        path = fd.name
        fd.close()
        draw_sweep_view(res, path)
        return path
    except (ImportError, OSError, RuntimeError, ValueError) as exc:
        print(f"[报告] 扫掠图渲染失败：{type(exc).__name__}: {exc}")
        return None