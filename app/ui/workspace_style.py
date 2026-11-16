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

QFrame#WorkspaceHeader { border-bottom: 1px solid #ccd9eb; }
QLabel#WorkspaceTitle { color: #132d55; font-size: 22pt; font-weight: 700; }
QLabel#ModelName { color: #526987; font-size: 9pt; }
QFrame#Canvas, QFrame#Inspector, QTextBrowser, QListWidget#ProcessIndex {
    background: #ffffff;
    border: 1px solid #d5e1f0;
    border-radius: 12px;
}
QLabel#PanelTitle { color: #1b3760; font-size: 13pt; font-weight: 700; }
QLabel#ScenarioLabel { color: #506887; font-size: 9pt; }
QFrame#MetricCard { background: #edf4ff; border: 1px solid #dae7f8; border-radius: 8px; }
QLabel#MetricTitle { color: #506887; font-size: 9pt; }
QLabel#MetricValue { color: #174785; font-size: 17pt; font-weight: 700; }

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

QTabWidget::pane { border: 0; }
QTabBar::tab {
    background: #e0eaf8;
    color: #3f597d;
    border: 1px solid transparent;
    border-radius: 8px;
    padding: 9px 18px;
    margin-right: 7px;
    min-width: 74px;
    font-size: 10pt;
}
QTabBar::tab:hover { background: #d4e4fb; color: #16467e; }
QTabBar::tab:selected { background: #123565; color: #ffffff; font-weight: 700; }
QTabBar::tab:focus { border-color: #64a5fb; }
QScrollArea, QScrollArea > QWidget > QWidget { background: #eef3f9; border: 0; }
QGroupBox {
    background: #ffffff;
    border: 1px solid #d5e1f0;
    border-radius: 10px;
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
QListWidget#ProcessIndex { padding: 7px; color: #3f587b; font-size: 10pt; }
QListWidget#ProcessIndex::item { padding: 9px 9px; margin-bottom: 3px; border-radius: 6px; }
QListWidget#ProcessIndex::item:hover { background: #edf4ff; }
QListWidget#ProcessIndex::item:selected { background: #dceaff; color: #184780; }
QScrollBar:vertical { border: 0; background: #eaf0f8; width: 8px; }
QScrollBar::handle:vertical { background: #aec2df; min-height: 24px; border-radius: 4px; }
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0; }
"""
