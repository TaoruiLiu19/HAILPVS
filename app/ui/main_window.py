"""主窗口：工况配置、扫掠图和检验结论。

字段表单与 Pydantic 数据模型(InputBundle)一一对应；点击“运行检验”后：
  采集表单 → 构建 InputBundle → run_scene 仿真 → draw_sweep_view 出图
  → 右侧展示扫掠图 + 通过性结论；并支持模型保存/打开(JSON 持久化)。
"""
import os
import datetime
from io import BytesIO
from pathlib import Path

from PyQt5.QtWidgets import (QApplication, QMainWindow, QLabel, QPushButton, QFileDialog,
                             QVBoxLayout, QHBoxLayout, QGridLayout, QTabWidget, QListWidget,
                             QScrollArea, QFrame, QTextBrowser, QWidget,
                             QMessageBox)
from PyQt5.QtCore import Qt, QUrl
from PyQt5.QtGui import QKeySequence, QPixmap

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
from .qtutil import _app_font, _primary_screen_geometry
from .workspace_style import workspace_stylesheet
from .widgets import (_make_form_group, _row_label, _new_combo, _new_edit,
                      ClickableImageLabel)

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
    ("牵引销至转动中心轴 (m)", "t_kingpin_to_axle", "kingpin_to_axle", False),
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
    ("牵引车前轮轮距 (m)", "tr_track_width", "track_width", False),
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
    ("左转物理区可通行宽 (m)", "W_section", False),
    ("左转处可通行宽 (m)", "W_lturn", False),
    # 交角固定为 90°（直角弯道），不再提供输入
]
_OPTIONAL_INPUTS = ("t_kingpin_to_axle", "tr_track_width", "superelevation", "combined_cg_height")
_EMPTY_PREVIEW = "运行弯道检验后，扫掠图将在此显示"
_DOCS = Path(__file__).resolve().parents[2] / "docs"
_STANDARD_URL = "https://xxgk.mot.gov.cn/jigou/glj/202310/t20231008_3924626.html"


class ParameterGrid(QWidget):
    """按可用宽度在两列和三列之间切换，保留所有表单控件状态。"""

    def __init__(self, columns):
        super().__init__()
        self.columns = columns
        self.grid = QGridLayout(self)
        self.grid.setContentsMargins(2, 2, 9, 3)
        self.grid.setSpacing(16)
        self._wide = None
        self._reflow(False)

    def _reflow(self, wide):
        if wide == self._wide:
            return
        self._wide = wide
        for column in self.columns:
            self.grid.removeWidget(column)
        for index, column in enumerate(self.columns):
            if not wide and index == 2:
                self.grid.addWidget(column, 1, 0, 1, 2)
            else:
                self.grid.addWidget(column, 0, index)
        self.grid.setColumnStretch(0, 1)
        self.grid.setColumnStretch(1, 1)
        self.grid.setColumnStretch(2, 1 if wide else 0)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._reflow(event.size().width() >= 1010)


class MainWindow(QMainWindow):
    _STATUS_STYLE = {
        "none": ("未运算", "#edf2f7", "#40596e"),
        "running": ("检验中", "#e0f1f8", "#12617e"),
        "ok": ("通过", "#e2f5ec", "#126047"),
        "ng": ("不通过", "#fce9ea", "#a12435"),
    }

    def __init__(self):
        super().__init__()
        self.setWindowTitle(f"{APP_NAME}{APP_VERSION}")
        geo = _primary_screen_geometry()
        self.resize(min(1600, int(geo.width() * 0.94)), min(1000, int(geo.height() * 0.92)))
        self.setStyleSheet(workspace_stylesheet())
        self.setFont(_app_font(10))
        self.init_data()
        self.init_menu()
        self.init_ui()
        for edits in (self.edits, self._arc_edits, self._right_edits):
            for edit in edits.values():
                edit.textChanged.connect(self.action_clear_result)
        for combo in (self.cmb_pad, self.cmb_trailer_type, self.cmb_road, self.cmb_grade,
                      self.cmb_turn_direction):
            combo.currentTextChanged.connect(self.action_clear_result)

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
        m_file.addAction("新建模型", self.action_new, QKeySequence.New)
        m_file.addAction("打开模型", self.action_open, QKeySequence.Open)
        m_file.addSeparator()
        m_file.addAction("保存", self.action_save, QKeySequence.Save)
        m_file.addAction("另存为", self.action_saveas, QKeySequence.SaveAs)
        m_file.addSeparator()
        m_file.addAction("退出系统", self.close)
        m_tool = bar.addMenu("工具(&T)")
        m_tool.addAction("清空检验结果", self.action_clear_result)
        m_tool.addSeparator()
        m_tool.addAction("检验方法说明", self.action_show_acceptance)

    # ================= 界面 =================
    def init_ui(self):
        central = QWidget()
        central.setObjectName("Workspace")
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)
        layout.setContentsMargins(22, 12, 22, 18)
        layout.setSpacing(14)
        layout.addWidget(self._build_header())

        self.pages = QTabWidget()
        self.pages.setObjectName("WorkspacePages")
        self.inputs_page = self._build_inputs()
        self.graph_page = self._build_results()
        self.process_page = self._build_process()
        self.methods_page = self._build_methods()
        self.pages.addTab(self.inputs_page, "工况参数")
        self.pages.addTab(self.graph_page, "扫掠图")
        self.pages.addTab(self.process_page, "计算过程")
        self.pages.addTab(self.methods_page, "方法参考")
        layout.addWidget(self.pages, 1)
        self.action_clear_result()

    @staticmethod
    def _heading(text, style="PanelTitle"):
        label = QLabel(text)
        label.setObjectName(style)
        return label

    def _build_header(self):
        header = QFrame()
        header.setObjectName("WorkspaceHeader")
        row = QHBoxLayout(header)
        row.setContentsMargins(0, 7, 0, 10)
        row.setSpacing(8)
        titles = QVBoxLayout()
        titles.setSpacing(2)
        titles.addWidget(self._heading("弯道通过性检验", "WorkspaceTitle"))
        self.model_label = self._heading("未命名模型", "ModelName")
        titles.addWidget(self.model_label)
        row.addLayout(titles)
        row.addStretch(1)
        for label, action in (("打开模型", self.action_open), ("保存模型", self.action_save)):
            button = QPushButton(label)
            button.setMinimumHeight(34)
            button.clicked.connect(action)
            row.addWidget(button)
        self.btn_export = QPushButton("导出 Word")
        self.btn_export.setMinimumHeight(34)
        self.btn_export.clicked.connect(self.export_word)
        row.addWidget(self.btn_export)
        return header

    def _build_inputs(self):
        panel = QWidget()
        layout = self._page_layout(panel)
        road = self._build_road_column()
        vehicles, trailer = self._build_vehicle_columns()
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll.setWidget(ParameterGrid((road, vehicles, trailer)))
        layout.addWidget(scroll, 1)
        footer = QHBoxLayout()
        footer.addStretch(1)
        self.btn_calc = QPushButton("运行弯道检验")
        self.btn_calc.setObjectName("RunButton")
        self.btn_calc.setMinimumHeight(41)
        self.btn_calc.setMinimumWidth(160)
        self.btn_calc.clicked.connect(self.run_calc)
        footer.addWidget(self.btn_calc)
        layout.addLayout(footer)
        return panel

    @staticmethod
    def _page_layout(page):
        layout = QVBoxLayout(page)
        layout.setContentsMargins(0, 16, 0, 0)
        layout.setSpacing(12)
        return layout

    def _build_results(self):
        panel = QWidget()
        layout = self._page_layout(panel)
        body = QHBoxLayout()
        body.setSpacing(14)
        canvas = QFrame()
        canvas.setObjectName("Canvas")
        graph_layout = QVBoxLayout(canvas)
        graph_layout.setContentsMargins(12, 10, 12, 12)
        graph_layout.setSpacing(8)
        graph_actions = QHBoxLayout()
        graph_actions.addWidget(self._heading("弯道扫掠工程图"))
        graph_actions.addStretch(1)
        self.btn_preview = QPushButton("放大")
        self.btn_preview.clicked.connect(self.open_image_viewer)
        self.btn_save_img = QPushButton("保存 PNG")
        self.btn_save_img.clicked.connect(self.save_sweep_img)
        graph_actions.addWidget(self.btn_preview)
        graph_actions.addWidget(self.btn_save_img)
        graph_layout.addLayout(graph_actions)
        self.img_label = ClickableImageLabel()
        self.img_label.setObjectName("SweepPreview")
        self.img_label.setMinimumHeight(160)
        self.img_label.clicked.connect(self.open_image_viewer)
        graph_layout.addWidget(self.img_label, 1)
        body.addWidget(canvas, 1)

        summary = QFrame()
        summary.setObjectName("Inspector")
        summary.setFixedWidth(248)
        inspector = QVBoxLayout(summary)
        inspector.setContentsMargins(14, 13, 14, 13)
        inspector.setSpacing(9)
        title_row = QHBoxLayout()
        title_row.addWidget(self._heading("检验结果"))
        title_row.addStretch(1)
        self.lbl_status = QLabel()
        self.lbl_status.setAlignment(Qt.AlignCenter)
        self.lbl_status.setMinimumWidth(65)
        title_row.addWidget(self.lbl_status)
        inspector.addLayout(title_row)
        self.scenario_label = self._heading("", "ScenarioLabel")
        self.scenario_label.setWordWrap(True)
        inspector.addWidget(self.scenario_label)

        self.metric_labels = []
        for _ in range(3):
            card = QFrame()
            card.setObjectName("MetricCard")
            card_layout = QVBoxLayout(card)
            card_layout.setContentsMargins(10, 6, 10, 7)
            card_layout.setSpacing(2)
            name = self._heading("", "MetricTitle")
            value = self._heading("", "MetricValue")
            card_layout.addWidget(name)
            card_layout.addWidget(value)
            self.metric_labels.append((name, value))
            inspector.addWidget(card)
        self.btn_process = QPushButton("查看公式与代入值")
        self.btn_process.clicked.connect(lambda: self.pages.setCurrentWidget(self.process_page))
        inspector.addWidget(self.btn_process)
        inspector.addStretch(1)
        body.addWidget(summary)
        layout.addLayout(body, 1)
        return panel

    def _build_process(self):
        panel = QWidget()
        layout = self._page_layout(panel)
        body = QHBoxLayout()
        body.setSpacing(14)
        self.process_sections = QListWidget()
        self.process_sections.setObjectName("ProcessIndex")
        self.process_sections.setAccessibleName("计算过程目录")
        self.process_sections.setFixedWidth(213)
        self.process_sections.currentRowChanged.connect(self._jump_to_step)
        body.addWidget(self.process_sections)
        self.text_result = QTextBrowser()
        self.text_result.setOpenExternalLinks(False)
        self.text_result.setDocumentTitle("本工况计算过程")
        self.text_result.setPlaceholderText("运行检验后，可查看本工况的公式、代入值和逐项判定。")
        self.text_result.document().setDocumentMargin(24)
        body.addWidget(self.text_result, 1)
        layout.addLayout(body, 1)
        return panel

    def _build_methods(self):
        panel = QWidget()
        layout = self._page_layout(panel)
        self.methods_document = QTextBrowser()
        self.methods_document.setOpenExternalLinks(True)
        self.methods_document.document().setDocumentMargin(24)
        self.methods_document.document().setBaseUrl(QUrl.fromLocalFile(str(_DOCS) + os.sep))
        try:
            reference = (_DOCS / "检验方法参考.md").read_text(encoding="utf-8")
        except OSError:
            reference = f"# 检验方法参考\n\n{ACCEPTANCE_NOTE}\n\n[交通运输部发布公告]({_STANDARD_URL})"
        self.methods_document.setMarkdown(reference)
        layout.addWidget(self.methods_document, 1)
        return panel

    # ---- 工况表单 ----
    def _form_column(self, title):
        column = QWidget()
        layout = QVBoxLayout(column)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(10)
        layout.addWidget(self._heading(title))
        return column, layout

    def _build_vehicle_columns(self):
        vehicle_column, vehicle = self._form_column("车货外廓")
        trailer_column, trailer = self._form_column("挂车配置")

        g_c, f_c = _make_form_group("货物")
        for label, key, field, numeric in _CARGO_FIELDS:
            ed = _new_edit("0.0")
            self.edits[key] = ed
            f_c.addRow(_row_label(f"{label}："), ed)
        self.cmb_pad = _new_combo(PAD_ITEMS, PAD_ITEMS[0] if PAD_ITEMS else "干燥的木板")
        f_c.addRow(_row_label("货物衬垫："), self.cmb_pad)
        vehicle.addWidget(g_c)

        g_t, f_t = _make_form_group("板车与轴组")
        self.cmb_trailer_type = _new_combo(TRAILER_TYPES, TRAILER_TYPES[0] if TRAILER_TYPES else "")
        f_t.addRow(_row_label("挂车类型："), self.cmb_trailer_type)
        for label, key, field, numeric in _TRAILER_FIELDS:
            ed = _new_edit("0.0" if numeric else "")
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
        trailer.addWidget(g_t)
        trailer.addWidget(g_hyd)
        self.g_hyd = g_hyd
        self.cmb_trailer_type.currentTextChanged.connect(self._sync_hydraulic_fields)
        self._sync_hydraulic_fields()

        g_car, f_car = _make_form_group("牵引车")
        for label, key, field, numeric in _TRACTOR_FIELDS:
            ed = _new_edit("0.0" if numeric else "")
            self.edits[key] = ed
            f_car.addRow(_row_label(f"{label}："), ed)
        vehicle.addWidget(g_car)
        vehicle.addStretch(1)
        trailer.addStretch(1)
        return vehicle_column, trailer_column

    def _build_road_column(self):
        column, v = self._form_column("道路与工况")
        g_top, f_top = _make_form_group("道路条件")
        self.cmb_road = _new_combo(ROAD_TYPES, ROAD_TYPES[0] if ROAD_TYPES else "")
        self.cmb_road.currentTextChanged.connect(self._sync_road_fields)
        f_top.addRow(_row_label("弯道类型："), self.cmb_road)
        ed = _new_edit("7.5")
        self.edits["vertical_clearance"] = ed
        f_top.addRow(_row_label("垂直净空高度(m)："), ed)
        v.addWidget(g_top)

        # 圆弧弯道
        self.g_arc, f_arc = _make_form_group("圆弧几何")
        self._arc_edits = {}
        for name, key, numeric in _ARC_FIELDS:
            ed = _new_edit("0.0")
            self._arc_edits[key] = ed
            f_arc.addRow(_row_label(f"{name}："), ed)
        v.addWidget(self.g_arc)

        # 直角弯道
        self.g_right, f_right = _make_form_group("路口几何")
        self.cmb_turn_direction = _new_combo(["右转", "左转"], "右转")
        f_right.addRow(_row_label("实际转向："), self.cmb_turn_direction)
        self._right_edits = {}
        for name, key, numeric in _RIGHT_FIELDS:
            ed = _new_edit("0.0" if numeric else "")
            self._right_edits[key] = ed
            f_right.addRow(_row_label(f"{name}："), ed)
        v.addWidget(self.g_right)

        # 满载工况
        g_cond, f_cond = _make_form_group("运行工况")
        self.cmb_grade = _new_combo(HIGHWAY_GRADES, "三级公路")
        f_cond.addRow(_row_label("公路等级："), self.cmb_grade)
        ed = _new_edit("10.0")
        self.edits["max_speed_kmh"] = ed
        f_cond.addRow(_row_label("最高行驶速度(km/h)："), ed)
        for label, key, hint in (("弯道超高率", "superelevation", "选填，如 0.02"),
                                 ("车货总体重心离地高 (m)", "combined_cg_height", "选填")):
            ed = _new_edit("")
            ed.setPlaceholderText(hint)
            self.edits[key] = ed
            f_cond.addRow(_row_label(f"{label}："), ed)
        v.addWidget(g_cond)

        v.addStretch(1)
        self._sync_road_fields()
        return column

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
            f"background-color: {bg}; color: {fg}; border-radius: 7px;"
            "padding: 4px 8px; font-size: 9pt; font-weight: 700;}")

    def _refresh_metrics(self):
        road_type = self.cmb_road.currentText()
        is_right_angle = road_type == "直角弯道"
        grade = self.cmb_grade.currentText()
        direction = f" · {self.cmb_turn_direction.currentText()}" if is_right_angle else ""
        self.scenario_label.setText(f"{road_type}{direction} · {grade}")
        names = ("所需宽度", "最大侵入深度", "顶部净空间隙") if is_right_angle else (
            "转弯通道宽度", "最小横向净空", "顶部净空间隙")
        values = (None, None, None)
        if self.cur_result is not None:
            metrics = self.cur_result.metrics
            if is_right_angle:
                width = metrics["required_w"]
                horizontal = None if metrics["split_error"] else max(
                    metrics["vehicle_depth"], metrics["cargo_depth"])
            else:
                width = metrics["turn_channel_w"]
                horizontal = min(metrics["clearance_inner"], metrics["clearance_outer"])
                if metrics["lookup"] == "pass":
                    names = ("通道宽度（参考）", *names[1:])
            vertical = self.cur_result.user_meta["vertical_clearance"] - metrics["total_height"]
            values = (width, horizontal, vertical)
        for (title, output), name, value in zip(self.metric_labels, names, values):
            title.setText(name)
            output.setText("—" if value is None else f"{value:.3f} m")

    def _set_model_path(self, path):
        self.current_model_path = path
        name = os.path.basename(path) if path else "未命名模型"
        self.model_label.setText(self.model_label.fontMetrics().elidedText(name, Qt.ElideMiddle, 290))
        self.model_label.setToolTip(path)

    # ================= 表单采集 → InputBundle =================
    def _f(self, key, default="0.0"):
        ed = self.edits.get(key)
        return float(ed.text()) if ed else float(default)

    def _f_arc(self, key):
        return float(self._arc_edits[key].text())

    def _f_right(self, key):
        return float(self._right_edits[key].text())

    @staticmethod
    def _optional_number(edit):
        text = edit.text().strip()
        return float(text) if text else None

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
            kingpin_to_axle=self._optional_number(self.edits["t_kingpin_to_axle"]),
            steering_type="全轮强制转向",
            axle_max_steering_angle_deg=self._f("t_axle_steer_deg"),
        )
        tractor = TractorParams(
            wheelbase=self._f("tr_wheelbase"),
            max_steering_angle_deg=self._f("tr_max_steering_angle_deg"),
            width=self._f("tr_width"),
            track_width=self._optional_number(self.edits["tr_track_width"]),
            fifth_wheel_pos=self._f("tr_fifth_wheel_pos"),
        )
        road_type = self.cmb_road.currentText()
        grade = self.cmb_grade.currentText()
        superelevation = self._optional_number(self.edits["superelevation"])
        if road_type == "圆弧弯道":
            road = RoadParams(road_type=road_type,
                              arc=ArcCorner(outer_radius=self._f_arc("outer_radius"),
                                            lane_width=self._f_arc("lane_width")),
                              vertical_clearance=self._f("vertical_clearance"),
                              superelevation=superelevation,
                              highway_grade=grade)
        else:
            road = RoadParams(road_type=road_type,
                              right_angle=RightAngleCorner(
                                  entry_width=self._f_right("entry_width"),
                                  exit_width=self._f_right("exit_width"),
                                  corner_radius=self._f_right("corner_radius"),
                                  intersection_angle_deg=90.0,
                                  turn_direction=self.cmb_turn_direction.currentText(),
                                  W_section=self._optional_number(self._right_edits["W_section"]),
                                  W_lturn=self._optional_number(self._right_edits["W_lturn"])),
                              vertical_clearance=self._f("vertical_clearance"),
                              superelevation=superelevation,
                              highway_grade=grade)
        return InputBundle(cargo=cargo, trailer=trailer, tractor=tractor, road=road,
                           max_speed_kmh=self._f("max_speed_kmh"),
                           combined_cg_height=self._optional_number(self.edits["combined_cg_height"]))

    def run_calc(self):
        self.action_clear_result()
        try:
            bundle = self.collect_bundle()
            model_data = self.pack_model_data()
        except Exception as e:
            QMessageBox.warning(self, "输入错误", f"参数采集/校验失败：\n{e}")
            return
        self.set_status_badge("running")
        self.lbl_status.repaint()
        self.btn_calc.setEnabled(False)
        QApplication.setOverrideCursor(Qt.WaitCursor)
        try:
            res = run_scene(bundle)
        except Exception as e:
            self.set_status_badge("none")
            QMessageBox.critical(self, "仿真失败", f"弯道通过性仿真出错：\n{e}")
            return
        finally:
            self.btn_calc.setEnabled(True)
            QApplication.restoreOverrideCursor()
        res.user_meta = model_data
        self.cur_result = res
        try:
            with BytesIO() as image:
                draw_sweep_view(res, image)
                pm = QPixmap()
                if not pm.loadFromData(image.getvalue(), "PNG"):
                    raise ValueError("无法读取生成的扫掠图")
        except Exception as e:
            self.img_label.set_preview(placeholder="扫掠图生成失败，可查看计算过程")
            QMessageBox.warning(self, "出图失败", f"扫掠图生成失败：{e}")
            self.pages.setCurrentWidget(self.process_page)
        else:
            self._orig_pixmap = pm
            self.img_label.set_preview(pm)
            self.btn_preview.setEnabled(True)
            self.btn_save_img.setEnabled(True)
            self.pages.setCurrentWidget(self.graph_page)
        self.btn_export.setEnabled(True)
        self.btn_process.setEnabled(True)
        self.set_status_badge("ok" if res.pass_ok else "ng")
        self._refresh_metrics()
        self._render_result(res)

    def _render_result(self, res):
        self.process_sections.clear()
        html = []
        verdict = "通过" if res.pass_ok else "不通过"
        for line in [*res.verdict_lines, "【综合结论】", f"弯道检验结论：{verdict}"]:
            if line.startswith("【") and line.endswith("】"):
                title = line[1:-1]
                anchor = f"step-{self.process_sections.count()}"
                self.process_sections.addItem(title)
                html.append(f"<h2 style='margin-top:22px;color:#18524d;'>"
                            f"<a name='{anchor}'>{title}</a></h2>")
            else:
                html.append(f"<p style='margin:0 0 6px;'>{colorize_verdict_html(line)}</p>")
        self.text_result.setHtml(
            "<div style='font-size:11pt;color:#283d3a;line-height:1.5;'>"
            + "".join(html) + "</div>")
        self.process_sections.setCurrentRow(0)

    def _jump_to_step(self, row):
        if row >= 0:
            self.text_result.scrollToAnchor(f"step-{row}")

    # ================= 图片操作 =================
    def save_sweep_img(self):
        if self._orig_pixmap is None:
            QMessageBox.information(self, "提示", "请先运行检验生成扫掠图。")
            return
        path, _ = QFileDialog.getSaveFileName(self, "保存扫掠图", "sweep.png",
                                              "PNG 图片 (*.png)")
        if path:
            if not self._orig_pixmap.save(path if path.lower().endswith(".png") else path + ".png"):
                QMessageBox.warning(self, "保存失败", "无法保存扫掠图。")

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
        d["turn_direction"] = self.cmb_turn_direction.currentText()
        d["highway_grade"] = self.cmb_grade.currentText()
        d["vertical_clearance"] = self._f("vertical_clearance")
        for k, ed in self._arc_edits.items():
            d["arc_" + k] = ed.text()
        for k, ed in self._right_edits.items():
            d["right_" + k] = ed.text()
        d["saved_at"] = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        return d

    def unpack_model_data(self, d):
        self.action_clear_result()
        for k, ed in self.edits.items():
            if k in d:
                ed.setText("" if k in _OPTIONAL_INPUTS and d[k] is None else str(d[k]))
            elif k in _OPTIONAL_INPUTS:
                ed.clear()
        direction = d.get("turn_direction")
        self.cmb_turn_direction.setCurrentText("左转" if direction == "左转" else "右转")
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
                value = d["right_" + k]
                ed.setText("" if value is None else str(value))
            elif k in ("W_section", "W_lturn"):
                ed.clear()
        self._sync_road_fields()

    def action_new(self):
        for key, ed in self.edits.items():
            ed.setText("" if key in _OPTIONAL_INPUTS else "0.0")
        self.edits["t_axle_steer_deg"].setText(str(HYDRAULIC_AXLE_DEFAULT_STEER_DEG))
        self.cmb_turn_direction.setCurrentText("右转")
        for key, edit in self._right_edits.items():
            edit.setText("" if key in ("W_section", "W_lturn") else "0.0")
        self.cmb_road.setCurrentText(ROAD_TYPES[0])
        self._sync_road_fields()
        self.action_clear_result()
        self._set_model_path("")
        self.pages.setCurrentWidget(self.inputs_page)

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
            self._set_model_path(path)
            self.pages.setCurrentWidget(self.inputs_page)
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
                self._set_model_path(path)
                QMessageBox.information(self, "保存成功", "模型已保存。")
            except Exception as e:
                QMessageBox.warning(self, "保存失败", f"{e}")

    def action_clear_result(self):
        self.cur_result = None
        self.set_status_badge("none")
        self.text_result.clear()
        self.process_sections.clear()
        self._orig_pixmap = None
        self.img_label.set_preview(placeholder=_EMPTY_PREVIEW)
        self.btn_preview.setEnabled(False)
        self.btn_save_img.setEnabled(False)
        self.btn_export.setEnabled(False)
        self.btn_process.setEnabled(False)
        self._refresh_metrics()

    def action_show_acceptance(self):
        self.pages.setCurrentWidget(self.methods_page)

    # ================= Word 导出（口令保护：EXPORT_PWD）=================
    def export_word(self):
        res = self.cur_result
        if res is None:
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
        from PyQt5.QtWidgets import QFileDialog
        path, _ = QFileDialog.getSaveFileName(self, "导出Word报告", "弯道检验报告.docx",
                                              "Word 文档 (*.docx)")
        if path:
            try:
                assemble_and_save(res, path)
                QMessageBox.information(self, "导出成功", f"报告已生成：{path}")
            except Exception as e:
                QMessageBox.warning(self, "导出失败", f"{e}")


# 延迟导入，避免 import 循环
from app.ui.widgets import ImageViewerDialog
