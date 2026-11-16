"""可复用控件与表单构建工具。"""
from PyQt5.QtWidgets import (QLabel, QComboBox, QLineEdit, QGroupBox, QFormLayout,
                             QDialog, QVBoxLayout, QHBoxLayout, QGraphicsView,
                             QGraphicsScene, QGraphicsPixmapItem, QSizePolicy)
from PyQt5.QtCore import Qt, pyqtSignal, QTimer
from PyQt5.QtGui import QPainter

from .qtutil import _app_font, _primary_screen_geometry

# ===== “基础信息输入 / 绑扎数据输入”排版常量：统一字号、行距与标签列宽 =====
FORM_FONT_SIZE = 9      # 标签/控件的字体(pt)
FORM_INPUT_H = 29       # 所有输入控件统一高度，保证行排列整齐
FORM_LABEL_W = 156      # 长字段标签在窄窗口自动换行
FORM_INPUT_MAX_W = 200  # 输入控件最大宽度（避免长数字拥挤）
FORM_V_SPACING = 7      # 行与行之间的垂直间距（行距均匀）


def _row_label(text):
    lab = QLabel(text)
    lab.setObjectName("FormLabel")
    lab.setFixedWidth(FORM_LABEL_W)
    lab.setWordWrap(True)
    lab.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
    lab.setFont(_app_font(FORM_FONT_SIZE))
    return lab


def _new_combo(items, current=""):
    cb = QComboBox()
    cb.addItems(items)
    if current:
        cb.setCurrentText(current)
    cb.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
    cb.setMaximumWidth(FORM_INPUT_MAX_W)
    cb.setMinimumHeight(FORM_INPUT_H)
    cb.setFont(_app_font(FORM_FONT_SIZE))
    return cb


def _new_edit(text="0.0"):
    ed = QLineEdit(text)
    ed.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
    ed.setMaximumWidth(FORM_INPUT_MAX_W)
    ed.setMinimumHeight(FORM_INPUT_H)
    ed.setFont(_app_font(FORM_FONT_SIZE))
    if not text:
        ed.setPlaceholderText("选填")
    return ed


def _make_form_group(title):
    g = QGroupBox(title)
    g.setFont(_app_font(FORM_FONT_SIZE, bold=True))
    form = QFormLayout(g)
    form.setLabelAlignment(Qt.AlignLeft | Qt.AlignVCenter)
    form.setHorizontalSpacing(12)
    form.setVerticalSpacing(FORM_V_SPACING)
    form.setContentsMargins(12, 12, 12, 12)
    form.setFieldGrowthPolicy(QFormLayout.AllNonFixedFieldsGrow)
    form.setRowWrapPolicy(QFormLayout.DontWrapRows)
    return g, form


class ClickableImageLabel(QLabel):
    """适配视口大小的缩略图，点击时弹出原图。"""
    clicked = pyqtSignal()

    def __init__(self, text="", parent=None):
        super().__init__(text, parent)
        self._source = None
        self.setAlignment(Qt.AlignCenter)
        self.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Ignored)

    def set_preview(self, pixmap=None, placeholder=""):
        self._source = pixmap
        if pixmap is None:
            self.setText(placeholder)
            self.unsetCursor()
        else:
            self.setCursor(Qt.PointingHandCursor)
            self._resize_preview()

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._resize_preview()

    def _resize_preview(self):
        if self._source is not None:
            self.setPixmap(self._source.scaled(self.contentsRect().size(),
                                               Qt.KeepAspectRatio, Qt.SmoothTransformation))

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton and self._source is not None:
            self.clicked.emit()
        super().mousePressEvent(event)


class ZoomableGraphicsView(QGraphicsView):
    """支持滚轮缩放、拖拽平移的图片视图。"""
    zoomChanged = pyqtSignal(float)

    def __init__(self, scene, parent=None):
        super().__init__(scene, parent)
        self.setDragMode(QGraphicsView.ScrollHandDrag)
        self.setRenderHints(QPainter.Antialiasing | QPainter.SmoothPixmapTransform)
        self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        self.setResizeAnchor(QGraphicsView.AnchorViewCenter)
        self._zoom = 1.0

    def wheelEvent(self, event):
        factor = 1.25 if event.angleDelta().y() > 0 else 0.8
        nz = self._zoom * factor
        if 0.05 <= nz <= 20.0:
            self.scale(factor, factor)
            self._zoom = nz
            self.zoomChanged.emit(nz)
        event.accept()

    def fit_image(self, item):
        self.resetTransform()
        self.fitInView(item, Qt.KeepAspectRatio)
        self._zoom = 1.0
        self.zoomChanged.emit(1.0)


class ImageViewerDialog(QDialog):
    """大图预览窗口：滚轮缩放、鼠标拖拽平移、双击关闭。"""
    def __init__(self, pixmap, parent=None):
        super().__init__(parent)
        self.setWindowTitle("扫掠图预览（滚轮缩放 · 拖拽平移）")
        geo = _primary_screen_geometry()
        self.resize(int(geo.width() * 0.92), int(geo.height() * 0.9))
        lay = QVBoxLayout(self)
        self.scene = QGraphicsScene(self)
        self.item = QGraphicsPixmapItem(pixmap)
        self.item.setTransformationMode(Qt.SmoothTransformation)
        self.scene.addItem(self.item)
        self.view = ZoomableGraphicsView(self.scene, self)
        self.view.zoomChanged.connect(self._on_zoom)
        lay.addWidget(self.view, 1)
        bar = QHBoxLayout()
        bar.addStretch(1)
        self.lab_zoom = QLabel("100%")
        self.lab_zoom.setFont(_app_font(10))
        bar.addWidget(self.lab_zoom)
        bar.addStretch(1)
        lay.addLayout(bar)
        QTimer.singleShot(0, lambda: self.view.fit_image(self.item))

    def _on_zoom(self, z):
        self.lab_zoom.setText(f"{z * 100:.0f}%")

    def mouseDoubleClickEvent(self, event):
        self.accept()
