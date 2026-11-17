"""主窗口样式，与应用全局样式和登录对话框独立。"""

from PyQt5.QtGui import QFontDatabase


def workspace_stylesheet():
    families = set(QFontDatabase().families())
    for family in ("Microsoft YaHei UI", "Microsoft YaHei", "微软雅黑", "Noto Sans SC"):
        if family in families:
            return f'QWidget {{ font-family: "{family}"; }}\n' + WORKSPACE_QSS
    return WORKSPACE_QSS


WORKSPACE_QSS = """
QWidget#Workspace {
    background: #eef3f9;
    color: #172b48;
}
QMenuBar { background: #f7f9fd; color: #3b5375; padding: 3px 8px; }
QMenuBar::item { padding: 4px 10px; border-radius: 4px; }
QMenuBar::item:selected, QMenu::item:selected { background: #dce8f9; color: #172b48; }
QMenu { background: #ffffff; color: #172b48; border: 1px solid #cbd9eb; padding: 4px; }
QMenu::item { padding: 5px 20px; }

QFrame#WorkspaceHeader {
    background: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #102c52, stop:1 #194780);
    border-radius: 10px;
}
QLabel#WorkspaceTitle { color: #ffffff; font-size: 17pt; font-weight: 700; }
QLabel#ModelName { color: #c4d8f3; font-size: 9pt; }
QFrame#Canvas, QFrame#InputPanel, QFrame#ProcessPanel, QTextBrowser {
    background: #ffffff;
    border: 1px solid #d5e1f0;
    border-radius: 12px;
}
QSplitter#WorkspaceSplitter::handle { background: transparent; }
QSplitter#WorkspaceSplitter::handle:hover { background: #a8c4ea; border-radius: 4px; }
QLabel#PanelTitle { color: #1b3760; font-size: 13pt; font-weight: 700; }
QLabel#ScenarioLabel { color: #506887; font-size: 9pt; }
QFrame#MetricCard { background: #edf4ff; border: 1px solid #dae7f8; border-radius: 8px; }
QLabel#MetricTitle { color: #506887; font-size: 9pt; }
QLabel#MetricValue { color: #174785; font-size: 13pt; font-weight: 700; }

QPushButton {
    background: #ffffff;
    color: #244874;
    border: 1px solid #bacdea;
    border-radius: 7px;
    font-size: 9pt;
    font-weight: 600;
    padding: 5px 12px;
}
QPushButton:hover { background: #e9f1ff; border-color: #8caee2; }
QPushButton:pressed { background: #dbe8ff; }
QPushButton:focus { border-color: #2e6fd1; }
QPushButton:disabled { background: #eff3f8; color: #607489; border-color: #dce4ee; }
QPushButton#HeaderButton { background: #24528a; color: #f2f7ff; border-color: #4270a5; }
QPushButton#HeaderButton:hover { background: #33669f; border-color: #86afe0; }
QPushButton#HeaderButton:pressed { background: #163b6d; }
QPushButton#HeaderButton:focus { border-color: #b7d6fe; }
QPushButton#HeaderButton:disabled { background: #204573; color: #b1c3da; border-color: #365d8a; }
QPushButton#RunButton {
    background: #1c5cc5;
    color: #ffffff;
    border: 1px solid #1c5cc5;
    font-size: 11pt;
}
QPushButton#RunButton:hover { background: #174eaa; border-color: #174eaa; }
QPushButton#RunButton:pressed { background: #123e88; }
QPushButton#RunButton:focus { border: 2px solid #9cc6ff; }
QPushButton#RunButton:disabled { background: #526f97; border-color: #526f97; }

QScrollArea#FormScroll, QWidget#ConditionForm { background: #ffffff; border: 0; }
QGroupBox {
    background: #f5f8fd;
    border: 1px solid #dce6f4;
    border-radius: 8px;
    color: #2d4d78;
    font-size: 10pt;
    font-weight: 700;
    margin-top: 10px;
    padding-top: 6px;
}
QGroupBox::title { subcontrol-origin: margin; left: 9px; padding: 0 5px; }
QLabel#FormLabel { color: #365174; font-size: 9pt; }
QLineEdit, QComboBox {
    background: #ffffff;
    color: #172b48;
    border: 1px solid #c6d6ed;
    border-radius: 5px;
    padding: 3px 7px;
    font-size: 9pt;
    selection-background-color: #1d62c7;
    selection-color: #ffffff;
}
QLineEdit:focus, QComboBox:focus { border-color: #2e6fd1; }
QComboBox#ProcessSections { background: #edf4ff; color: #284f80; padding-left: 10px; }
QComboBox#ProcessSections:disabled { color: #506887; border-color: #dce6f4; }
QComboBox QAbstractItemView {
    background: #ffffff;
    color: #172b48;
    border: 1px solid #c6d6ed;
    selection-background-color: #dce8f9;
    selection-color: #193b69;
}
QLabel#SweepPreview {
    background: #f3f7fc;
    border: 1px solid #dce7f5;
    border-radius: 8px;
    color: #506887;
    font-size: 10pt;
}
QTextBrowser { color: #283d59; font-size: 11pt; }
QTextBrowser#CalculationText { border: 0; border-radius: 0; }
QScrollBar:vertical { border: 0; background: #eaf0f8; width: 8px; }
QScrollBar::handle:vertical { background: #aec2df; min-height: 24px; border-radius: 4px; }
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0; }
"""
