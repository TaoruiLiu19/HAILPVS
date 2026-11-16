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
    background: #eaf0f5;
    color: #13273b;
}
QMenuBar { background: #f6f9fc; color: #32475b; padding: 3px 8px; }
QMenuBar::item { padding: 4px 10px; border-radius: 4px; }
QMenuBar::item:selected, QMenu::item:selected { background: #dbe8f3; color: #13273b; }
QMenu { background: #ffffff; color: #13273b; border: 1px solid #c8d6e2; padding: 4px; }
QMenu::item { padding: 5px 20px; }

QFrame#WorkspaceHeader { background: #12273c; border-radius: 12px; }
QLabel#WorkspaceTitle { color: #ffffff; font-size: 17pt; font-weight: 700; }
QLabel#ModelName { color: #b9cee0; font-size: 9pt; }
QWidget#InputPanel, QWidget#ResultPanel {
    background: #ffffff;
    border: 1px solid #d5e0e9;
    border-radius: 12px;
}
QLabel#PanelTitle { color: #152e46; font-size: 13pt; font-weight: 700; }
QLabel#ScenarioLabel { color: #4d657b; font-size: 9pt; }
QFrame#MetricCard { background: #f2f7fb; border: 1px solid #dce7f0; border-radius: 8px; }
QLabel#MetricTitle { color: #476176; font-size: 9pt; }
QLabel#MetricValue { color: #123c5e; font-size: 15pt; font-weight: 700; }

QPushButton {
    background: #ffffff;
    color: #245473;
    border: 1px solid #c4d5e2;
    border-radius: 7px;
    font-size: 9pt;
    font-weight: 600;
    padding: 5px 12px;
}
QPushButton:hover { background: #eaf4fa; border-color: #8eb3cc; }
QPushButton:pressed { background: #dbeaf4; }
QPushButton:focus { border-color: #267aa5; }
QPushButton:disabled { background: #f2f5f7; color: #667b8e; border-color: #e0e7ee; }
QPushButton#RunButton {
    background: #096b86;
    color: #ffffff;
    border: 1px solid #096b86;
    font-size: 11pt;
}
QPushButton#RunButton:hover { background: #075872; border-color: #075872; }
QPushButton#RunButton:pressed { background: #08485e; }
QPushButton#RunButton:focus { border: 2px solid #83d5e6; }
QPushButton#RunButton:disabled { background: #456c7d; border-color: #456c7d; }
QPushButton#HeaderAction { background: #203d55; color: #f3f8fc; border-color: #48657c; }
QPushButton#HeaderAction:hover, QPushButton#HeaderAction:focus { background: #315672; }

QTabWidget::pane { border: 0; border-top: 1px solid #e0e8ef; }
QTabBar::tab {
    background: transparent;
    color: #4b647b;
    border: 0;
    border-bottom: 2px solid transparent;
    padding: 7px 11px;
    margin-right: 4px;
    min-width: 0;
    font-size: 9pt;
}
QTabBar::tab:hover { color: #075b76; background: #f3f8fb; }
QTabBar::tab:selected { color: #075b76; border-bottom-color: #0783a0; font-weight: 700; }
QTabBar::tab:focus { background: #eef6fb; }
QScrollArea, QWidget#FormSurface { background: #ffffff; border: 0; }
QGroupBox {
    background: #f7fafd;
    border: 1px solid #dce7ef;
    border-radius: 8px;
    color: #26445d;
    font-size: 10pt;
    font-weight: 700;
    margin-top: 10px;
    padding-top: 6px;
}
QGroupBox::title { subcontrol-origin: margin; left: 9px; padding: 0 5px; }
QLabel#FormLabel { color: #314b62; font-size: 9pt; }
QLineEdit, QComboBox {
    background: #ffffff;
    color: #152e46;
    border: 1px solid #cbd9e4;
    border-radius: 5px;
    padding: 3px 7px;
    font-size: 9pt;
    selection-background-color: #0e708c;
    selection-color: #ffffff;
}
QLineEdit:focus, QComboBox:focus { border-color: #087f9b; }
QComboBox QAbstractItemView {
    background: #ffffff;
    color: #152e46;
    border: 1px solid #cbd9e4;
    selection-background-color: #dbeef5;
    selection-color: #123d53;
}
QLabel#SweepPreview {
    background: #f2f7fb;
    border: 1px solid #dce7f0;
    border-radius: 8px;
    color: #4d657b;
    font-size: 10pt;
}
QTextBrowser { background: #ffffff; color: #223d54; border: 0; }
QSplitter::handle { background: #eaf0f5; }
QScrollBar:vertical { border: 0; background: #eef3f7; width: 8px; }
QScrollBar::handle:vertical { background: #b6c8d6; min-height: 24px; border-radius: 4px; }
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0; }
"""
