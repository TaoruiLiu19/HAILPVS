"""Qt 运行环境与全局样式工具。

负责在 QApplication 创建前修复 Qt 平台插件路径，并提供统一的
字体选择、屏幕可用区域与全局样式表。
"""
import os

# ===== 修复“找不到 Qt 平台插件 windows”问题 =====
# 当系统 PATH 混入独立 Qt6 安装、或 PyQt5 装在用户目录时，
# Qt 的插件路径可能被解析成空，导致 QApplication 启动即崩溃。
# 必须在创建 QApplication 之前显式指向 PyQt5 自带的插件目录。
def _fix_qt_plugins():
    try:
        import PyQt5.QtCore as _qtcore
        base = os.path.dirname(os.path.abspath(_qtcore.__file__))
        plugins = os.path.join(base, "Qt5", "plugins")
        platforms = os.path.join(plugins, "platforms")
        if os.path.isdir(platforms):
            os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"] = platforms
            os.environ["QT_PLUGIN_PATH"] = plugins
    except Exception:
        pass


# 全局字体：显式使用系统自带“微软雅黑”，按可用性回退，避免乱码/符号
_FONT_CANDIDATES = ["Microsoft YaHei", "微软雅黑", "SimSun", "宋体", "SimHei", "黑体", "KaiTi", "楷体", ""]
APP_FONT_FAMILY = "Microsoft YaHei"


def _pick_font_family():
    from PyQt5.QtGui import QFontDatabase
    try:
        fams = set(QFontDatabase.families())
    except Exception:
        fams = set()
    for cand in _FONT_CANDIDATES:
        if not cand:
            return ""
        if cand in fams:
            return cand
    return ""


def _app_font(size=10, bold=False):
    from PyQt5.QtGui import QFont
    f = QFont(APP_FONT_FAMILY)
    f.setPointSize(size)
    f.setBold(bold)
    return f


# 获取主屏幕可用几何区域，替代已废弃的 QApplication.desktop()
def _primary_screen_geometry():
    from PyQt5.QtGui import QGuiApplication
    from PyQt5.QtCore import QRect
    scr = QGuiApplication.primaryScreen()
    if scr is not None:
        return scr.availableGeometry()
    # 极端兜底：无屏幕信息时返回一个合理尺寸
    return QRect(0, 0, 1600, 900)


# ===== 动态获取可用字体族，避免硬编码导致回退到难看的默认字体 =====
_FONT_FAMILY = _pick_font_family()
_FONT_FAMILY_QSS = f'"{_FONT_FAMILY}"' if _FONT_FAMILY else 'sans-serif'

# 全局样式表：统一样式，防止控件字体被系统默认字体覆盖
APP_QSS = f"""
QWidget {{ font-family: {_FONT_FAMILY_QSS}; }}
QGroupBox {{
    font-family: {_FONT_FAMILY_QSS};
    font-size: 11pt;
    font-weight: bold;
    margin-top: 6px;
    border: 1px solid #c8c8c8;
    border-radius: 6px;
    padding-top: 14px;
}}
QGroupBox::title {{ subcontrol-origin: margin; left: 10px; padding: 0 4px; }}
QLabel {{ font-family: {_FONT_FAMILY_QSS}; font-size: 10pt; }}
QLineEdit, QComboBox {{
    font-family: {_FONT_FAMILY_QSS};
    font-size: 10pt;
    padding: 2px 6px;
    border: 1px solid #b0b0b0;
    border-radius: 4px;
}}
QPushButton {{ font-family: {_FONT_FAMILY_QSS}; font-size: 11pt; }}
QTabWidget::pane {{ border: 1px solid #c8c8c8; }}
QTabBar::tab {{ font-family: {_FONT_FAMILY_QSS}; font-size: 10pt; padding: 8px 12px; min-width: 210px; }}
QTableWidget, QHeaderView {{ font-family: {_FONT_FAMILY_QSS}; font-size: 10pt; }}

/* 将 QTextEdit 改为同时适用于 QTextBrowser */
QTextEdit, QTextBrowser {{
    font-family: {_FONT_FAMILY_QSS};
    font-size: 10pt;
}}

/* 专门针对结果区 QTextBrowser 的清理：去掉内边距，让 HTML 里的布局自己控制 */
QTextBrowser {{
    padding: 0px;
    border: 1px solid #c8c8c8;
    border-radius: 4px;
    background-color: #ffffff;
}}

/* 美化滚动条：让结果区看起来更专业 */
QScrollBar:vertical {{
    border: none;
    background: #f1f1f1;
    width: 10px;
    margin: 0px 0px 0px 0px;
}}
QScrollBar::handle:vertical {{
    background: #c1c1c1;
    min-height: 20px;
    border-radius: 5px;
}}
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {{
    border: none;
    background: none;
}}
"""