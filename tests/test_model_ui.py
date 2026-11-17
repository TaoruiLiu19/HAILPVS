from pathlib import Path

import pytest
from PyQt5.QtCore import QSettings
from PyQt5.QtWidgets import QApplication, QFileDialog, QMessageBox

from app.model.project import load_model
from app.ui import main_window


def test_non_object_model_rejected(tmp_path):
    path = tmp_path / "invalid.model"
    path.write_text("[]", encoding="utf-8")
    with pytest.raises(ValueError, match="JSON 对象"):
        load_model(path)


def test_input_change_and_failed_render_cannot_reuse_old_result(monkeypatch):
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    app = QApplication.instance() or QApplication([])
    window = main_window.MainWindow()
    example = next(Path(__file__).resolve().parents[1].glob("示例2_*.model"))
    monkeypatch.setattr(QMessageBox, "warning", lambda *args: None)
    try:
        window.show()
        window.unpack_model_data(load_model(example))
        window.run_calc()
        original = window.cur_result
        assert original.user_meta["c_height"] == "3.0"
        assert not {"t_axle_count", "turn_direction", "right_entry_width"} & original.user_meta.keys()
        assert window._orig_pixmap is not None
        assert window.btn_export.isEnabled()
        assert window.btn_preview.isEnabled() and window.btn_save_img.isEnabled()
        assert window.metric_labels[0][1].text().endswith(" m")
        assert all(panel.isVisible() for panel in
                   (window.inputs_panel, window.graph_panel, window.process_panel))
        width_section = window.process_sections.findText("宽度与横向净空")
        assert window.process_sections.currentIndex() == width_section >= 0
        width_position = window.text_result.verticalScrollBar().value()
        assert width_position > 0
        window.process_sections.setCurrentIndex(window.process_sections.findText("车辆及货物参数"))
        assert window.text_result.verticalScrollBar().value() < width_position
        detail = window.text_result.toPlainText()
        assert f"{original.metrics['W']:.3f} m" in detail
        assert "= 4.350 − 4.200 = 0.150 m" in detail
        window.action_show_acceptance()
        assert window._methods_dialog.isVisible() and not window._methods_dialog.isModal()
        assert "JTG/T 2213—2023" in window.methods_document.toPlainText()
        assert "计算方法说明.md" in window.methods_document.toMarkdown()
        window._methods_dialog.close()

        window.edits["c_height"].setText("3.5")
        assert window.cur_result is None
        assert window._orig_pixmap is None
        assert window.text_result.toPlainText() == ""
        assert window.lbl_status.text() == "未运算"
        assert not window.btn_export.isEnabled()
        assert not window.btn_preview.isEnabled() and not window.btn_save_img.isEnabled()
        assert all(value.text() == "—" for _, value in window.metric_labels)

        def fail_to_draw(*args, **kwargs):
            raise OSError("render failed")

        window.edits["vertical_clearance"].setText("4.0")
        monkeypatch.setattr(main_window, "draw_sweep_view", fail_to_draw)
        window.run_calc()
        assert window.cur_result.user_meta["c_height"] == "3.5"
        assert original.user_meta["c_height"] == "3.0"
        assert window._orig_pixmap is None
        assert window.btn_export.isEnabled() and not window.btn_save_img.isEnabled()
        assert window.process_panel.isVisible() and window.text_result.toPlainText()
        assert window.process_sections.currentText() == "垂直净空"
    finally:
        window.close()
        app.processEvents()


def test_turn_and_stability_inputs_survive_model_round_trip(monkeypatch):
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    app = QApplication.instance() or QApplication([])
    window = main_window.MainWindow()
    example = next(Path(__file__).resolve().parents[1].glob("示例4_*.model"))
    old_model = load_model(example)
    try:
        window.unpack_model_data(old_model)
        right_turn = window.collect_bundle()
        assert right_turn.road.right_angle.turn_direction == "右转"
        assert right_turn.road.right_angle.W_section is None
        assert window.cmb_grade.isHidden()
        assert window._right_edits["W_section"].isHidden()
        assert window.edits["t_axle_count"].isHidden()
        window.cmb_turn_direction.setCurrentText("左转")
        assert not window._right_edits["W_section"].isHidden()
        window.edits["superelevation"].setText("-0.04")
        window.edits["combined_cg_height"].setText("3.5")
        assert window.edits["c_cg_height"].isHidden()
        saved = window.pack_model_data()
        window.action_new()
        window.unpack_model_data(saved)
        bundle = window.collect_bundle()
        assert bundle.road.right_angle.turn_direction == "左转"
        assert bundle.road.right_angle.W_section == pytest.approx(18.5)
        assert bundle.road.superelevation == pytest.approx(-0.04)
        assert bundle.combined_cg_height == pytest.approx(3.5)
        window.unpack_model_data(old_model)
        assert window.collect_bundle().combined_cg_height is None
        assert not window.edits["c_cg_height"].isHidden()

        window.cmb_trailer_type.setCurrentText("液压轴线挂车")
        assert window.g_tractor.isHidden()
        assert not window.edits["t_axle_count"].isHidden()
        window.edits["tr_wheelbase"].setText("unused")
        assert window.collect_bundle().trailer.axle_count == 3
        assert window.pack_model_data()["tr_wheelbase"] == "unused"
    finally:
        window.close()
        app.processEvents()


def test_clearance_states_and_legacy_model(monkeypatch):
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    app = QApplication.instance() or QApplication([])
    window = main_window.MainWindow()
    example = next(Path(__file__).resolve().parents[1].glob("示例2_*.model"))
    legacy_model = load_model(example)
    legacy_model.pop("vertical_clearance_mode")
    legacy_model["vertical_clearance"] = 7.5
    try:
        window.unpack_model_data(legacy_model)
        height = window.edits["vertical_clearance"]
        assert height.isHidden() and height.text() == "7.5"
        assert window.cmb_clearance_structure.isHidden()
        assert window.collect_bundle().road.vertical_clearance is None
        window.run_calc()
        assert window.lbl_status.text() == "待核实"
        assert not window.cur_result.pass_ok
        assert window._orig_pixmap is not None
        assert window.process_sections.currentText() == "垂直净空"
        assert "vertical_clearance" not in window.cur_result.user_meta

        window.cmb_clearance_mode.setCurrentText("上方开敞")
        height.setText("unused")
        window.run_calc()
        assert window.cur_result.pass_ok
        assert window.metric_labels[2][1].text() == "上方开敞"
        assert window.cur_result.metrics["vertical_clearance_gap"] is None

        window.cmb_clearance_mode.setCurrentText("有顶部障碍")
        assert not height.isHidden() and not window.cmb_clearance_structure.isHidden()
        height.clear()
        with pytest.raises(ValueError, match="最小顶部净空"):
            window.collect_bundle()
        height.setText("4.35")
        window.cmb_clearance_structure.setCurrentText("隧道")
        saved = window.pack_model_data()
        window.action_new()
        window.unpack_model_data(saved)
        window.run_calc()
        assert window.metric_labels[2][1].text() == "0.150 m"
        assert window.cur_result.verdict == "不通过"
        assert window.cur_result.metrics["vert_clearance_ok"] is False
        assert window.cur_result.user_meta["clearance_structure_type"] == "隧道"
    finally:
        window.close()
        app.processEvents()


def test_plot_style_switch_refreshes_the_png_and_persists(monkeypatch, tmp_path):
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    settings_path = str(tmp_path / "drawing.ini")
    monkeypatch.setattr(main_window, "QSettings", lambda *_: QSettings(settings_path, QSettings.IniFormat))
    app = QApplication.instance() or QApplication([])
    window = main_window.MainWindow()
    example = next(Path(__file__).resolve().parents[1].glob("示例1_*.model"))
    png_path = tmp_path / "selected-style.png"
    monkeypatch.setattr(QFileDialog, "getSaveFileName", lambda *args: (str(png_path), ""))
    try:
        window.unpack_model_data(load_model(example))
        window.run_calc()
        result = window.cur_result
        window.save_sweep_img()
        overview_png = png_path.read_bytes()
        classic = next(a for a in window._style_actions.actions() if a.data() == "classic")
        classic.trigger()
        assert window.cur_result is result and classic.isChecked()
        window.save_sweep_img()
        assert png_path.read_bytes() != overview_png
        window._settings.sync()

        reopened = main_window.MainWindow()
        try:
            assert reopened._style_actions.checkedAction().data() == "classic"
        finally:
            reopened.close()
    finally:
        window.close()
        app.processEvents()
