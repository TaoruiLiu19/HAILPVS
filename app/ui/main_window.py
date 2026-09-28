"""主窗口：左栏三页签输入表单 + 右栏扫掠图与结论。

字段表单与 Pydantic 数据模型(InputBundle)一一对应；点击“运行检验”后：
  采集表单 → 构建 InputBundle → run_scene 仿真 → draw_sweep_view 出图
  → 右侧展示扫掠图 + 通过性结论；并支持模型保存/打开(JSON 持久化)。
"""
import os
import datetime

from PyQt5.QtWidgets import (QMainWindow, QLabel, QPushButton, QFileDialog,
                             QVBoxLayout, QHBoxLayout, QTabWidget,
                             QScrollArea, QFrame, QSplitter, QTextBrowser, QWidget,
                             QMessageBox)
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QPixmap

from config import (APP_NAME, APP_VERSION, ROAD_TYPES, TRAILER_TYPES,
                    ACCEPTANCE_NOTE, EXPORT_PWD, MODEL_SUFFIX,
                    HIGHWAY_GRADES, HYDRAULIC_AXLE_DEFAULT_STEER_DEG)
try:
    from config import PAD_ITEMS
except ImportError:
    PAD_ITEMS = ["干燥的木板", "干燥的橡胶垫", "潮湿的木板", "潮湿的橡胶垫"]

from app.core.datamodel import (InputBundle, CargoParams, TrailerParams, TractorParams,
                                RoadParams, ArcCorner, RightAngleCorner)
from app.core.passability import run_scene
from app.rendering.sweep_view import draw_sweep_view
from app.rendering.verdict_style import colorize_verdict_html
from app.model.project import save_model, load_model, ensure_suffix
from .qtutil import APP_QSS, _app_font, _primary_screen_geometry
from .widgets import (_make_form_group, _row_label, _new_combo, _new_edit,
                      ClickableImageLabel)

# 左栏页签用小一号字(继承参考项目排版：FORM_FONT_SIZE 与整体 QSS 分离)
_LEFT_PANEL_QSS = """
QWidget { font-size: 9pt; }
QGroupBox { font-size: 10pt; font-weight: bold; }
QLabel { font-size: 9pt; }
QLineEdit, QComboBox { font-size: 9pt; }
"""

_TMP_IMG = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
                        "temp_sweep.png")

# ===== 窗体左侧输入表单的字段定义：控件键 → (标签, Pydantic 字段) =====
# 键前缀区分货物(c_)/挂车(t_)/牵引车(tr_)，避免 length/width 等重名覆盖。
# 仅保留参与几何/判定运算的参数；仅存档展示项不再作为输入。
_CARGO_FIELDS = [
    ("货物长度(m)", "c_length", "length", True),
    ("货物宽度(m)", "c_width", "width", True),
    ("货物高度(m)", "c_height", "height", True),
    ("重心高度(m)", "c_cg_height", "cg_height", True),
]
_TRAILER_FIELDS = [
    ("挂车长度(m)", "t_length", "length", True),
    ("挂车宽度(m)", "t_width", "width", True),
    ("车板上表面离地高度(m)", "t_deck_height", "deck_height", True),
    ("挂车轴数", "t_axle_count", "axle_count", True),
    ("挂车轴距(m)", "t_axle_spacing", "axle_spacing", True),
    ("轮距(m)", "t_track_width", "track_width", True),
]
# 仅液压轴线挂车显示的参数（选挂车类型=液压后整组可见）；标签不带“(液压)”后缀。
# 连接方式/连接部分长度为纯绘图参数且当前渲染路径未使用，不再提供输入（模型默认 0/鹅颈）。
_HYDRAULIC_TRAILER_FIELDS = [
    ("每轴最大转向角(°)", "t_axle_steer_deg", "axle_max_steering_angle_deg", True),
]
_TRACTOR_FIELDS = [
    ("牵引车轴距(m)", "tr_wheelbase", "wheelbase", True),
    ("最大转向角(°)", "tr_max_steering_angle_deg", "max_steering_angle_deg", True),
    ("牵引车宽度(m)", "tr_width", "width", True),
    ("鞍座至后轴距离(m)", "tr_fifth_wheel_pos", "fifth_wheel_pos", True),
]
_ARC_FIELDS = [
    ("道路外半径(m)", "outer_radius", True),
    ("车道宽度(m)", "lane_width", True),
]
_RIGHT_FIELDS = [
    ("入口宽度(m)", "entry_width", True),
    ("出口宽度(m)", "exit_width", True),
    ("弯道内半径(m)", "corner_radius", True),
    ("左转物理区可通行宽 W_section(m,可留空)", "W_section", False),
    ("左转处可通行宽 W_lturn(m,可留空)", "W_lturn", False),
    # 交角固定为 90°（直角弯道），不再提供输入
]


def _esc_html(s: str) -> str:
    return (s.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;"))


class MainWindow(QMainWindow):
    _STATUS_STYLE = {
        "none": ("未运算", "#d9d9d9", "#000000"),
        "ok": ("通过", "#2f855a", "#ffffff"),
        "ng": ("不通过", "#c53030", "#ffffff"),
    }

    def __init__(self):
        super().__init__()
        self.setWindowTitle(f"{APP_NAME}{APP_VERSION}")
        geo = _primary_screen_geometry()
        self.resize(min(1600, int(geo.width() * 0.94)), min(1000, int(geo.height() * 0.92)))
        self.setStyleSheet(APP_QSS)
        self.setFont(_app_font(10))
        self.init_data()
        self.init_menu()
        self.init_ui()

    # ================= 数据状态 =================
    def init_data(self):
        self.edits = {}          # 数值编辑框：键见上面 *_FIELDS
        self.checks = {}         # 复选框
        self.combos = {}         # 下拉框：由控件引用持有
        self.cur_result = None   # 最近一次 run_scene 结果
        self.current_model_path = ""
        self._orig_pixmap = None

    # ================= 菜单 =================
    def init_menu(self):
        bar = self.menuBar()
        m_file = bar.addMenu("文件(&F)")
        m_file.addAction("新建模型", self.action_new)
        m_file.addAction("打开模型", self.action_open)
        m_file.addSeparator()
        m_file.addAction("保存", self.action_save)
        m_file.addAction("另存为", self.action_saveas)
        m_file.addSeparator()
        m_file.addAction("退出系统", self.close)
        m_tool = bar.addMenu("工具(&T)")
        m_tool.addAction("清空检验结果", self.action_clear_result)
        m_tool.addSeparator()
        m_tool.addAction("检验方法说明", self.action_show_acceptance)

    # ================= 界面 =================
    def init_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        splitter = QSplitter(Qt.Horizontal)
        left_w = QWidget()
        left_w.setMaximumWidth(720)
        left_v = QVBoxLayout(left_w)

        tab = QTabWidget()
        tab.setStyleSheet(_LEFT_PANEL_QSS)
        tab.tabBar().setExpanding(False)
        tab.tabBar().setUsesScrollButtons(False)
        tab.tabBar().setElideMode(Qt.ElideNone)

        tab.addTab(self._build_input_tab(), "运输及车辆参数")
        tab.addTab(self._build_road_tab(), "弯道参数")
        left_v.addWidget(tab)

        btn_calc = QPushButton("运行弯道检验并出图")
        btn_calc.setMinimumHeight(46)
        btn_calc.setFont(_app_font(13, bold=True))
        btn_calc.setStyleSheet("padding:8px;border-radius:6px;background:#2b6cb0;color:white;")
        btn_calc.clicked.connect(self.run_calc)
        btn_export = QPushButton("一键生成Word报告")
        btn_export.setMinimumHeight(46)
        btn_export.setFont(_app_font(13, bold=True))
        btn_export.setStyleSheet("padding:8px;border-radius:6px;background:#2f855a;color:white;")
        btn_export.clicked.connect(self.export_word)
        btn_h = QHBoxLayout()
        btn_h.setSpacing(10)
        btn_h.addWidget(btn_calc, 1)
        btn_h.addWidget(btn_export, 1)
        left_v.addLayout(btn_h)
        splitter.addWidget(left_w)

        right_w = QWidget()
        right_v = QVBoxLayout(right_w)
        img_title = QHBoxLayout()
        img_title.addWidget(QLabel("弯道扫掠检验图（点击图片可放大预览）"))
        img_title.addStretch(1)
        self.btn_save_img = QPushButton("保存扫掠图")
        self.btn_save_img.setMinimumHeight(34)
        self.btn_save_img.setFont(_app_font(12, bold=True))
        self.btn_save_img.setStyleSheet("padding:6px 18px;border-radius:6px;background:#3182ce;color:white;")
        self.btn_save_img.clicked.connect(self.save_sweep_img)
        img_title.addWidget(self.btn_save_img)
        right_v.addLayout(img_title)

        self.img_label = ClickableImageLabel("")
        self.img_label.setAlignment(Qt.AlignCenter)
        self.img_label.setMinimumHeight(430)
        self.img_label.clicked.connect(self.open_image_viewer)
        right_v.addWidget(self.img_label)

        res_title = QHBoxLayout()
        res_title.setSpacing(10)
        res_title.addWidget(QLabel("检验过程及结论"))
        res_title.addStretch(1)
        self.lbl_status = QLabel()
        self.lbl_status.setAlignment(Qt.AlignCenter)
        self.lbl_status.setFixedSize(92, 30)
        res_title.addWidget(self.lbl_status)
        right_v.addLayout(res_title)
        self.set_status_badge("none")

        self.text_result = QTextBrowser()
        self.text_result.setOpenExternalLinks(False)
        self.text_result.document().setDocumentMargin(6)
        right_v.addWidget(self.text_result)
        splitter.addWidget(right_w)
        splitter.setStretchFactor(0, 10)
        splitter.setStretchFactor(1, 22)
        splitter.setSizes([700, max(760, self.width() - 700)])

        layout = QVBoxLayout(central)
        layout.addWidget(splitter)

    # ---- 三个页签 ----
    def _build_input_tab(self):
        tab = QWidget()
        outer = QVBoxLayout(tab)
        outer.setContentsMargins(0, 0, 0, 0)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        inner = QWidget()
        v = QVBoxLayout(inner)
        v.setContentsMargins(10, 10, 10, 10)
        v.setSpacing(12)
        v.setSizeConstraint(QVBoxLayout().SetMinimumSize)

        # 1. 货物参数
        g_c, f_c = _make_form_group("1. 装载货物参数")
        for label, key, field, numeric in _CARGO_FIELDS:
            ed = _new_edit("0.0")
            self.edits[key] = ed
            f_c.addRow(_row_label(f"{label}："), ed)
        self.cmb_pad = _new_combo(PAD_ITEMS, PAD_ITEMS[0] if PAD_ITEMS else "干燥的木板")
        f_c.addRow(_row_label("货物衬垫："), self.cmb_pad)
        v.addWidget(g_c)

        # 2. 挂车参数
        g_t, f_t = _make_form_group("2. 承载挂车参数")
        self.cmb_trailer_type = _new_combo(TRAILER_TYPES, TRAILER_TYPES[0] if TRAILER_TYPES else "")
        f_t.addRow(_row_label("挂车类型："), self.cmb_trailer_type)
        for label, key, field, numeric in _TRAILER_FIELDS:
            ed = _new_edit("0.0")
            self.edits[key] = ed
            f_t.addRow(_row_label(f"{label}："), ed)
        # 液压轴线挂车参数整组：仅选“液压轴线挂车”后可见（标签不带“(液压)”后缀）
        g_hyd, f_hyd = _make_form_group("液压轴线挂车参数")
        for label, key, field, numeric in _HYDRAULIC_TRAILER_FIELDS:
            ed = _new_edit("0.0")
            self.edits[key] = ed
            f_hyd.addRow(_row_label(f"{label}："), ed)
        # 每轴最大转向角默认取第六章典型值（B.1.2 D2，界面可调）
        if "t_axle_steer_deg" in self.edits:
            self.edits["t_axle_steer_deg"].setText(str(HYDRAULIC_AXLE_DEFAULT_STEER_DEG))
        v.addWidget(g_t)
        v.addWidget(g_hyd)
        self.g_hyd = g_hyd
        self.cmb_trailer_type.currentTextChanged.connect(self._sync_hydraulic_fields)
        self._sync_hydraulic_fields()

        # 3. 牵引车参数
        g_car, f_car = _make_form_group("3. 牵引车参数")
        for label, key, field, numeric in _TRACTOR_FIELDS:
            ed = _new_edit("0.0")
            self.edits[key] = ed
            f_car.addRow(_row_label(f"{label}："), ed)
        v.addWidget(g_car)

        v.addStretch(1)
        scroll.setWidget(inner)
        outer.addWidget(scroll)
        return tab

    def _build_road_tab(self):
        tab = QWidget()
        outer = QVBoxLayout(tab)
        outer.setContentsMargins(0, 0, 0, 0)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        inner = QWidget()
        v = QVBoxLayout(inner)
        v.setContentsMargins(10, 10, 10, 10)
        v.setSpacing(12)
        v.setSizeConstraint(QVBoxLayout().SetMinimumSize)

        g_top, f_top = _make_form_group("1. 弯道方式")
        self.cmb_road = _new_combo(ROAD_TYPES, ROAD_TYPES[0] if ROAD_TYPES else "")
        self.cmb_road.currentTextChanged.connect(self._sync_road_fields)
        f_top.addRow(_row_label("弯道类型："), self.cmb_road)
        ed = _new_edit("7.5")
        self.edits["vertical_clearance"] = ed
        f_top.addRow(_row_label("垂直净空高度(m)："), ed)
        v.addWidget(g_top)

        # 圆弧弯道
        self.g_arc, f_arc = _make_form_group("2. 圆弧弯道参数")
        self._arc_edits = {}
        for name, key, numeric in _ARC_FIELDS:
            ed = _new_edit("0.0")
            self._arc_edits[key] = ed
            f_arc.addRow(_row_label(f"{name}："), ed)
        v.addWidget(self.g_arc)

        # 直角弯道
        self.g_right, f_right = _make_form_group("2. 直角弯道参数")
        self._right_edits = {}
        for name, key, numeric in _RIGHT_FIELDS:
            ed = _new_edit("0.0" if numeric else "")
            self._right_edits[key] = ed
            f_right.addRow(_row_label(f"{name}："), ed)
        v.addWidget(self.g_right)

        # 满载工况
        g_cond, f_cond = _make_form_group("3. 运行工况与道路等级")
        self.cmb_grade = _new_combo(HIGHWAY_GRADES, "三级公路")
        f_cond.addRow(_row_label("公路等级："), self.cmb_grade)
        ed = _new_edit("10.0")
        self.edits["max_speed_kmh"] = ed
        f_cond.addRow(_row_label("最高行驶速度(km/h)："), ed)
        v.addWidget(g_cond)

        v.addStretch(1)
        scroll.setWidget(inner)
        outer.addWidget(scroll)
        self._sync_road_fields()
        return tab

    def _sync_road_fields(self):
        mode = self.cmb_road.currentText()
        is_arc = mode == "圆弧弯道"
        self.g_arc.setVisible(is_arc)
        self.g_right.setVisible(not is_arc)

    def _sync_hydraulic_fields(self):
        # 液压轴线挂车参数整组仅在选择“液压轴线挂车”后显示
        self.g_hyd.setVisible(self.cmb_trailer_type.currentText() == "液压轴线挂车")

    # ================= 状态徽章 =================
    def set_status_badge(self, state):
        text, bg, fg = self._STATUS_STYLE.get(state, self._STATUS_STYLE["none"])
        self.lbl_status.setText(text)
        self.lbl_status.setStyleSheet(
            "QLabel {"
            f"background-color: {bg}; color: {fg}; border-radius: 6px;"
            "font-size: 11pt; font-weight: normal;}")

    # ================= 表单采集 → InputBundle =================
    def _f(self, key, default="0.0"):
        ed = self.edits.get(key)
        return float(ed.text()) if ed else float(default)

    def _f_arc(self, key):
        return float(self._arc_edits[key].text())

    def _f_right(self, key):
        return float(self._right_edits[key].text())

    def _f_right_opt(self, key):
        """可选输入：留空/非数字 → None（回退到入口/出口宽度口径），填数则用于 5.2.1 左转校核。"""
        txt = self._right_edits.get(key).text().strip()
        try:
            return None if txt == "" else float(txt)
        except ValueError:
            return None

    def collect_bundle(self) -> InputBundle:
        cargo_w = self._f("c_width")
        cargo_l = self._f("c_length")
        trailer_w = self._f("t_width")
        trailer_l = self._f("t_length")
        # 货物超出车板（宽度/长度）时按“超宽/超长装载”处理，允许生成包络
        overhang = (cargo_w > trailer_w) or (cargo_l > trailer_l)
        cargo = CargoParams(
            length=cargo_l,
            width=cargo_w,
            height=self._f("c_height"),
            cg_height=self._f("c_cg_height"),
            is_overhang=overhang,
            cargo_bearing=self.cmb_pad.currentText(),
        )
        trailer = TrailerParams(
            trailer_type=self.cmb_trailer_type.currentText(),
            column_type="单纵列",
            length=self._f("t_length"), width=self._f("t_width"),
            deck_height=self._f("t_deck_height"),
            axle_count=int(self._f("t_axle_count")),
            axle_spacing=self._f("t_axle_spacing"),
            track_width=self._f("t_track_width"),
            steering_type="全轮强制转向",
            axle_max_steering_angle_deg=self._f("t_axle_steer_deg"),
        )
        tractor = TractorParams(
            wheelbase=self._f("tr_wheelbase"),
            max_steering_angle_deg=self._f("tr_max_steering_angle_deg"),
            width=self._f("tr_width"),
            fifth_wheel_pos=self._f("tr_fifth_wheel_pos"),
        )
        road_type = self.cmb_road.currentText()
        grade = self.cmb_grade.currentText()
        if road_type == "圆弧弯道":
            road = RoadParams(road_type=road_type,
                              arc=ArcCorner(outer_radius=self._f_arc("outer_radius"),
                                            lane_width=self._f_arc("lane_width")),
                              vertical_clearance=self._f("vertical_clearance"),
                              highway_grade=grade)
        else:
            road = RoadParams(road_type=road_type,
                              right_angle=RightAngleCorner(
                                  entry_width=self._f_right("entry_width"),
                                  exit_width=self._f_right("exit_width"),
                                  corner_radius=self._f_right("corner_radius"),
                                  intersection_angle_deg=90.0,
                                  W_section=self._f_right_opt("W_section"),
                                  W_lturn=self._f_right_opt("W_lturn")),
                              vertical_clearance=self._f("vertical_clearance"),
                              highway_grade=grade)
        return InputBundle(cargo=cargo, trailer=trailer, tractor=tractor, road=road,
                           max_speed_kmh=self._f("max_speed_kmh"))

    def run_calc(self):
        try:
            bundle = self.collect_bundle()
        except Exception as e:
            QMessageBox.warning(self, "输入错误", f"参数采集/校验失败：\n{e}")
            return
        try:
            self.cur_result = run_scene(bundle)
        except Exception as e:
            QMessageBox.critical(self, "仿真失败", f"弯道通过性仿真出错：\n{e}")
            self.set_status_badge("ng")
            return
        res = self.cur_result
        try:
            draw_sweep_view(res, _TMP_IMG)
        except Exception as e:
            QMessageBox.warning(self, "出图失败", f"扫掠图生成失败：{e}")

        if os.path.exists(_TMP_IMG):
            pm = QPixmap(_TMP_IMG)
            if not pm.isNull():
                self._orig_pixmap = pm
                self.img_label.setPixmap(pm.scaled(self.img_label.size(),
                                                   Qt.KeepAspectRatio, Qt.SmoothTransformation))
        self.set_status_badge("ok" if res.pass_ok else "ng")
        self._render_result(res)

    def _render_result(self, res):
        # verdict_lines 已含 HTML 标记（<sub>/<sup> 上下标）与判定词，先对判定词
        # 上色（通过/满足/合格=绿，不通过/不满足/不合格=红），再以富文本渲染
        lines = [colorize_verdict_html(ln) for ln in res.verdict_lines]
        verdict = "通过" if res.pass_ok else "不通过"
        color = "#2f855a" if res.pass_ok else "#c53030"
        lines.append(f"<b style='color:{color};font-size:12pt;'>【结论】检验结论：{verdict}</b>")
        self.text_result.setHtml(
            "<div style='font-size:11pt;line-height:1.6;'>"
            + "<br>".join(lines) + "</div>")

    # ================= 图片操作 =================
    def save_sweep_img(self):
        if not os.path.exists(_TMP_IMG):
            QMessageBox.information(self, "提示", "请先运行检验生成扫掠图。")
            return
        path, _ = QFileDialog.getSaveFileName(self, "保存扫掠图", "sweep.png",
                                              "PNG 图片 (*.png)")
        if path:
            pm = QPixmap(_TMP_IMG)
            pm.save(path if path.endswith(".png") else path + ".png")

    def open_image_viewer(self):
        if self._orig_pixmap is None:
            return
        dlg = ImageViewerDialog(self._orig_pixmap, self)
        dlg.exec_()

    # ================= 模型持久化 =================
    def pack_model_data(self):
        d = {}
        for k, ed in self.edits.items():
            d[k] = ed.text()
        d["pad"] = self.cmb_pad.currentText()
        d["trailer_type"] = self.cmb_trailer_type.currentText()
        d["road_type"] = self.cmb_road.currentText()
        d["highway_grade"] = self.cmb_grade.currentText()
        d["vertical_clearance"] = self._f("vertical_clearance")
        for k, ed in self._arc_edits.items():
            d["arc_" + k] = ed.text()
        for k, ed in self._right_edits.items():
            d["right_" + k] = ed.text()
        d["saved_at"] = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        return d

    def unpack_model_data(self, d):
        def setv(key, value):
            if key in self.edits and isinstance(value, (int, float, str)):
                self.edits[key].setText(str(value))
        for k, ed in self.edits.items():
            if k in d:
                ed.setText(str(d[k]))
        if "pad" in d:
            self.cmb_pad.setCurrentText(str(d["pad"]))
        if "trailer_type" in d:
            self.cmb_trailer_type.setCurrentText(str(d["trailer_type"]))
        if "road_type" in d:
            self.cmb_road.setCurrentText(str(d["road_type"]))
        if "highway_grade" in d:
            self.cmb_grade.setCurrentText(str(d["highway_grade"]))
        for k, ed in self._arc_edits.items():
            if "arc_" + k in d:
                ed.setText(str(d["arc_" + k]))
        for k, ed in self._right_edits.items():
            if "right_" + k in d:
                ed.setText(str(d["right_" + k]))
        self._sync_road_fields()

    def action_new(self):
        for ed in self.edits.values():
            ed.setText("0.0")
        self.cmb_road.setCurrentText(ROAD_TYPES[0])
        self._sync_road_fields()
        self.cur_result = None
        self.set_status_badge("none")
        self.text_result.clear()
        self._orig_pixmap = None
        self.img_label.clear()
        self.current_model_path = ""

    def action_open(self):
        from PyQt5.QtWidgets import QFileDialog
        path, _ = QFileDialog.getOpenFileName(self, "打开模型", "",
                                              f"模型文件 (*{MODEL_SUFFIX});;所有文件 (*)")
        if path:
            try:
                d = load_model(path)
            except Exception as e:
                QMessageBox.warning(self, "打开失败", f"{e}")
                return
            self.unpack_model_data(d)
            self.current_model_path = path
            QMessageBox.information(self, "已打开", f"模型已加载：{os.path.basename(path)}")

    def action_save(self):
        if self.current_model_path:
            try:
                save_model(self.pack_model_data(), ensure_suffix(self.current_model_path))
                QMessageBox.information(self, "保存成功", "模型已保存。")
            except Exception as e:
                QMessageBox.warning(self, "保存失败", f"{e}")
        else:
            self.action_saveas()

    def action_saveas(self):
        from PyQt5.QtWidgets import QFileDialog
        path, _ = QFileDialog.getSaveFileName(self, "另存为模型", "model" + MODEL_SUFFIX,
                                              f"模型文件 (*{MODEL_SUFFIX})")
        if path:
            path = ensure_suffix(path)
            try:
                save_model(self.pack_model_data(), path)
                self.current_model_path = path
                QMessageBox.information(self, "保存成功", "模型已保存。")
            except Exception as e:
                QMessageBox.warning(self, "保存失败", f"{e}")

    def action_clear_result(self):
        self.cur_result = None
        self.set_status_badge("none")
        self.text_result.clear()
        self._orig_pixmap = None
        self.img_label.clear()

    def action_show_acceptance(self):
        QMessageBox.information(self, "检验方法依据", ACCEPTANCE_NOTE)

    # ================= Word 导出（口令保护：EXPORT_PWD）=================
    def export_word(self):
        if self.cur_result is None:
            QMessageBox.information(self, "提示", "请先运行弯道检验，再生成报告。")
            return
        try:
            from app.rendering.report_assembler import assemble_and_save
        except ImportError as e:
            QMessageBox.warning(self, "报告模块不可用",
                                f"报告依赖加载失败：{e}\n"
                                "请确认已安装 python-docx / matplotlib。")
            return
        # 导出口令保护（口令见 config.EXPORT_PWD）
        from PyQt5.QtWidgets import QInputDialog, QLineEdit
        pwd, ok = QInputDialog.getText(self, "导出口令", "请输入导出口令：",
                                       QLineEdit.Password)
        if not ok:
            return
        if pwd != EXPORT_PWD:
            QMessageBox.warning(self, "口令错误", "导出口令不正确，已取消导出。")
            return
        # 把当前表单输入挂到结果上，供报告“弯道输入参数”表读取
        self.cur_result.user_meta = self.pack_model_data()
        from PyQt5.QtWidgets import QFileDialog
        path, _ = QFileDialog.getSaveFileName(self, "导出Word报告", "弯道检验报告.docx",
                                              "Word 文档 (*.docx)")
        if path:
            try:
                assemble_and_save(self.cur_result, path)
                QMessageBox.information(self, "导出成功", f"报告已生成：{path}")
            except Exception as e:
                QMessageBox.warning(self, "导出失败", f"{e}")


# 延迟导入，避免 import 循环
from app.ui.widgets import ImageViewerDialog