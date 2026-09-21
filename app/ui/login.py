"""用户登录弹窗 - 深海军蓝渐变背景 + 白色横版卡片（苹果极简风）。

沿用于 LasingSystem2.0 的 login.py：布局、渐变、阴影、圆角、DPI 自适应标题
一律保留，仅替换应用名（来自 config.APP_NAME）与公司名文案。
"""
from PyQt5.QtWidgets import (QDialog, QLabel, QLineEdit, QPushButton, QMessageBox,
                             QVBoxLayout, QHBoxLayout, QFrame, QGraphicsDropShadowEffect,
                             QDesktopWidget)
from PyQt5.QtGui import QLinearGradient, QPalette, QBrush, QColor, QFont, QFontMetrics
from PyQt5.QtCore import Qt

from config import LOGIN_USERS, APP_NAME, APP_VERSION
from .qtutil import _app_font, APP_QSS

# ================= 尺寸 =================
DIALOG_W, DIALOG_H = 1060, 740
CARD_W, CARD_H = 700, 560
FIELD_W, FIELD_H = 400, 52
CARD_RADIUS = 20
FIELD_RADIUS = 12
VER_RATIO = 0.6
VER_FLOOR = 9
VER_GAP = 8
# ========================================

COLOR_BG_START = "#0F2C59"
COLOR_BG_END = "#2B6CB0"
COLOR_CARD = "#FFFFFF"
COLOR_TITLE = "#1A365D"
COLOR_COMPANY = "#718096"
COLOR_FOOT = "#A0AEC0"
COLOR_BLUE = "#2B6CB0"
COLOR_BLUE_HOVER = "#3182CE"
COLOR_BLUE_PRESSED = "#2C5282"
COLOR_FOCUS = "#3182CE"
COLOR_FIELD_BG = "#F5F5F7"
COLOR_FIELD_HOVER = "#EFF1F4"
COLOR_PLACEHOLDER = "#A0AEC0"

COMPANY_NAME = "昆明中远海运物流有限公司"


class LoginDialog(QDialog):
    def __init__(self):
        super().__init__()
        self.setWindowTitle(f"{APP_NAME}{APP_VERSION} - 用户登录")
        self.setFixedSize(DIALOG_W, DIALOG_H)
        self._center_window()
        self.setStyleSheet(APP_QSS)
        self.setFont(_app_font(10))
        self._apply_gradient()
        self._build_ui()
        self.edt_user.setFocus()
        self.edt_user.returnPressed.connect(self.edt_pwd.setFocus)
        self.edt_pwd.returnPressed.connect(self.check_login)

    def _center_window(self):
        screen = QDesktopWidget().screenGeometry()
        x = (screen.width() - DIALOG_W) // 2
        y = (screen.height() - DIALOG_H) // 2
        self.move(x, y)

    def _fit_title_pt(self, prefer=15, floor=11):
        limit = CARD_W - 48
        for pt in range(prefer, floor - 1, -1):
            if self._title_width(pt) <= limit:
                return pt
        return floor

    @staticmethod
    def _ver_pt(title_pt):
        return max(VER_FLOOR, round(title_pt * VER_RATIO))

    @classmethod
    def _title_width(cls, pt):
        fm_t = QFontMetrics(QFont("Microsoft YaHei", pt, QFont.Bold))
        fm_v = QFontMetrics(QFont("Microsoft YaHei", cls._ver_pt(pt)))
        return (fm_t.horizontalAdvance(APP_NAME) + VER_GAP
                + fm_v.horizontalAdvance(APP_VERSION))

    def _build_title_row(self):
        title_pt = self._fit_title_pt()
        ver_pt = self._ver_pt(title_pt)
        title = QLabel(APP_NAME)
        title.setFont(QFont("Microsoft YaHei", title_pt, QFont.Bold))
        title.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        title.setStyleSheet(
            "border: none;"
            f" color: {COLOR_TITLE};"
            f" font-family: 'Microsoft YaHei'; font-size: {title_pt}pt;"
            " font-weight: bold;")
        ver = QLabel(APP_VERSION)
        ver.setFont(QFont("Microsoft YaHei", ver_pt))
        ver.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        ver.setStyleSheet(
            "border: none;"
            f" color: {COLOR_COMPANY};"
            f" font-family: 'Microsoft YaHei'; font-size: {ver_pt}pt;"
            " font-weight: normal;")
        lift = max(0, QFontMetrics(title.font()).descent()
                      - QFontMetrics(ver.font()).descent())
        ver.setContentsMargins(0, 0, 0, lift)
        row = QHBoxLayout()
        row.setContentsMargins(0, 0, 0, 0)
        row.setSpacing(0)
        row.addSpacing(max(0, (CARD_W - title.sizeHint().width()) // 2))
        row.addWidget(title)
        row.addSpacing(VER_GAP)
        row.addWidget(ver, alignment=Qt.AlignBottom)
        row.addStretch(1)
        return row

    def _apply_gradient(self):
        c0, c1 = QColor(COLOR_BG_START), QColor(COLOR_BG_END)
        gradient = QLinearGradient(0, 0, DIALOG_W, DIALOG_H)
        for t in (0.0, 0.25, 0.5, 0.75, 1.0):
            s = t * t * (3.0 - 2.0 * t)
            gradient.setColorAt(t, QColor(
                round(c0.red() + (c1.red() - c0.red()) * s),
                round(c0.green() + (c1.green() - c0.green()) * s),
                round(c0.blue() + (c1.blue() - c0.blue()) * s)))
        pal = self.palette()
        pal.setBrush(QPalette.Window, QBrush(gradient))
        self.setPalette(pal)
        self.setAutoFillBackground(True)

    def _build_ui(self):
        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        card = QFrame()
        card.setObjectName("loginCard")
        card.setFixedSize(CARD_W, CARD_H)
        card.setStyleSheet(
            "QFrame#loginCard {"
            f" background-color: {COLOR_CARD};"
            " border: none;"
            f" border-radius: {CARD_RADIUS}px;"
            " }")
        shadow = QGraphicsDropShadowEffect(card)
        shadow.setBlurRadius(64)
        shadow.setColor(QColor(0, 0, 0, 45))
        shadow.setOffset(0, 18)
        card.setGraphicsEffect(shadow)
        card_layout = QVBoxLayout(card)
        card_layout.setContentsMargins(0, 44, 0, 28)
        card_layout.setSpacing(0)
        card_layout.addStretch(1)

        company = QLabel(COMPANY_NAME)
        company.setAlignment(Qt.AlignCenter)
        company.setStyleSheet(
            "border: none;"
            f" color: {COLOR_FOOT};"
            " font-family: 'Microsoft YaHei'; font-size: 9pt;")
        card_layout.addWidget(company)
        card_layout.addSpacing(20)
        card_layout.addLayout(self._build_title_row())
        card_layout.addSpacing(52)

        self.edt_user = self._create_field("请输入用户名")
        card_layout.addWidget(self.edt_user, alignment=Qt.AlignCenter)
        card_layout.addSpacing(18)
        self.edt_pwd = self._create_field("请输入密码", password=True)
        card_layout.addWidget(self.edt_pwd, alignment=Qt.AlignCenter)
        card_layout.addSpacing(52)

        self.btn_login = QPushButton("登 录")
        self.btn_login.setFixedSize(FIELD_W, FIELD_H)
        self.btn_login.setCursor(Qt.PointingHandCursor)
        self.btn_login.setStyleSheet(f"""
            QPushButton {{
                border: none;
                background-color: {COLOR_BLUE};
                color: #ffffff;
                border-radius: {FIELD_RADIUS}px;
                font-family: 'Microsoft YaHei';
                font-size: 14pt;
                font-weight: bold;
            }}
            QPushButton:hover {{ background-color: {COLOR_BLUE_HOVER}; }}
            QPushButton:pressed {{ background-color: {COLOR_BLUE_PRESSED}; }}
        """)
        self.btn_login.clicked.connect(self.check_login)
        card_layout.addWidget(self.btn_login, alignment=Qt.AlignCenter)
        card_layout.addStretch(1)
        root.addStretch(1)
        root.addWidget(card, alignment=Qt.AlignCenter)
        root.addStretch(1)

    def _create_field(self, placeholder, password=False):
        edit = QLineEdit()
        edit.setFixedSize(FIELD_W, FIELD_H)
        edit.setPlaceholderText(placeholder)
        if password:
            edit.setEchoMode(QLineEdit.Password)
        edit.setStyleSheet(f"""
            QLineEdit {{
                background: {COLOR_FIELD_BG};
                border: 1px solid {COLOR_FIELD_BG};
                border-radius: {FIELD_RADIUS}px;
                padding: 0 18px;
                font-family: 'Microsoft YaHei';
                font-size: 16px;
                color: #1A2B4C;
                selection-background-color: {COLOR_FOCUS};
                selection-color: #ffffff;
            }}
            QLineEdit:hover {{
                background: {COLOR_FIELD_HOVER};
                border: 1px solid {COLOR_FIELD_HOVER};
            }}
            QLineEdit:focus {{
                background: {COLOR_CARD};
                border: 1px solid {COLOR_FOCUS};
            }}
        """)
        role = getattr(QPalette, "PlaceholderText", None)
        if role is not None:
            pal = edit.palette()
            pal.setColor(role, QColor(COLOR_PLACEHOLDER))
            edit.setPalette(pal)
        return edit

    def check_login(self):
        u = self.edt_user.text().strip()
        p = self.edt_pwd.text().strip()
        if u in LOGIN_USERS and LOGIN_USERS[u] == p:
            self.accept()
        else:
            QMessageBox.warning(self, "验证失败", "用户名或密码输入错误！")
            self.edt_pwd.clear()
            self.edt_pwd.setFocus()