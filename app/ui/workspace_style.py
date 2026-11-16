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
    background: #edf2ef;
    color: #172e2b;
}
QMenuBar { background: #f8faf8; color: #36544e; padding: 3px 8px; }
QMenuBar::item { padding: 4px 10px; border-radius: 4px; }
QMenuBar::item:selected, QMenu::item:selected { background: #dcebe4; color: #172e2b; }
QMenu { background: #ffffff; color: #172e2b; border: 1px solid #cbdad4; padding: 4px; }
QMenu::item { padding: 5px 20px; }

QFrame#WorkspaceHeader { border-bottom: 1px solid #ccdcd5; }
QLabel#WorkspaceTitle { color: #163630; font-size: 22pt; font-weight: 700; }
QLabel#ModelName { color: #526c66; font-size: 9pt; }
QFrame#Canvas, QFrame#Inspector, QTextBrowser, QListWidget#ProcessIndex {
    background: #ffffff;
    border: 1px solid #d4e0da;
    border-radius: 12px;
}
QLabel#PanelTitle { color: #183a34; font-size: 13pt; font-weight: 700; }
QLabel#ScenarioLabel { color: #536e68; font-size: 9pt; }
QFrame#MetricCard { background: #eef6f2; border: 1px solid #dce9e2; border-radius: 8px; }
QLabel#MetricTitle { color: #536e68; font-size: 9pt; }
QLabel#MetricValue { color: #124e43; font-size: 17pt; font-weight: 700; }

QPushButton {
    background: #ffffff;
    color: #244e45;
    border: 1px solid #bdd1c8;
    border-radius: 7px;
    font-size: 9pt;
    font-weight: 600;
    padding: 5px 12px;
}
QPushButton:hover { background: #e7f2ed; border-color: #91b6a7; }
QPushButton:pressed { background: #dcebe5; }
QPushButton:focus { border-color: #27816b; }
QPushButton:disabled { background: #f0f4f1; color: #6a7f77; border-color: #dae4de; }
QPushButton#RunButton {
    background: #116753;
    color: #ffffff;
    border: 1px solid #116753;
    font-size: 11pt;
}
QPushButton#RunButton:hover { background: #0b5846; border-color: #0b5846; }
QPushButton#RunButton:pressed { background: #0c473a; }
QPushButton#RunButton:focus { border: 2px solid #93d3bb; }
QPushButton#RunButton:disabled { background: #50786b; border-color: #50786b; }

QTabWidget::pane { border: 0; }
QTabBar::tab {
    background: #e1ebe5;
    color: #3f5d55;
    border: 1px solid transparent;
    border-radius: 8px;
    padding: 9px 18px;
    margin-right: 7px;
    min-width: 74px;
    font-size: 10pt;
}
QTabBar::tab:hover { background: #d6e8df; color: #134f41; }
QTabBar::tab:selected { background: #153e35; color: #ffffff; font-weight: 700; }
QTabBar::tab:focus { border-color: #399b81; }
QScrollArea, QScrollArea > QWidget > QWidget { background: #edf2ef; border: 0; }
QGroupBox {
    background: #ffffff;
    border: 1px solid #d4e0da;
    border-radius: 10px;
    color: #2b5047;
    font-size: 10pt;
    font-weight: 700;
    margin-top: 10px;
    padding-top: 6px;
}
QGroupBox::title { subcontrol-origin: margin; left: 9px; padding: 0 5px; }
QLabel#FormLabel { color: #36544d; font-size: 9pt; }
QLineEdit, QComboBox {
    background: #ffffff;
    color: #172e2b;
    border: 1px solid #c9d9d1;
    border-radius: 5px;
    padding: 3px 7px;
    font-size: 9pt;
    selection-background-color: #15745e;
    selection-color: #ffffff;
}
QLineEdit:focus, QComboBox:focus { border-color: #27816b; }
QComboBox QAbstractItemView {
    background: #ffffff;
    color: #172e2b;
    border: 1px solid #c9d9d1;
    selection-background-color: #dcebe4;
    selection-color: #173c34;
}
QLabel#SweepPreview {
    background: #f2f6f4;
    border: 1px solid #dce8e1;
    border-radius: 8px;
    color: #536e68;
    font-size: 10pt;
}
QTextBrowser { color: #283d3a; font-size: 11pt; }
QListWidget#ProcessIndex { padding: 7px; color: #3c5a52; font-size: 10pt; }
QListWidget#ProcessIndex::item { padding: 9px 9px; margin-bottom: 3px; border-radius: 6px; }
QListWidget#ProcessIndex::item:hover { background: #edf5f0; }
QListWidget#ProcessIndex::item:selected { background: #d9eee3; color: #154a3e; }
QScrollBar:vertical { border: 0; background: #eef3ef; width: 8px; }
QScrollBar::handle:vertical { background: #b3c7bd; min-height: 24px; border-radius: 4px; }
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0; }
"""
