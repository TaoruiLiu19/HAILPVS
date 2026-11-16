from pathlib import Path

import pytest
from PyQt5.QtWidgets import QApplication, QMessageBox

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
        window.unpack_model_data(load_model(example))
        window.run_calc()
        original = window.cur_result
        assert original.user_meta["c_height"] == "3.0"
        assert window._orig_pixmap is not None

        window.edits["c_height"].setText("3.5")
        assert window.cur_result is None
        assert window._orig_pixmap is None
        assert window.text_result.toPlainText() == ""
        assert window.lbl_status.text() == "未运算"

        def fail_to_draw(*args):
            raise OSError("render failed")

        monkeypatch.setattr(main_window, "draw_sweep_view", fail_to_draw)
        window.run_calc()
        assert window.cur_result.user_meta["c_height"] == "3.5"
        assert original.user_meta["c_height"] == "3.0"
        assert window._orig_pixmap is None
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
        assert window.collect_bundle().road.right_angle.turn_direction == "右转"
        window.cmb_turn_direction.setCurrentText("左转")
        window.edits["superelevation"].setText("-0.04")
        window.edits["combined_cg_height"].setText("3.5")
        saved = window.pack_model_data()
        window.action_new()
        window.unpack_model_data(saved)
        bundle = window.collect_bundle()
        assert bundle.road.right_angle.turn_direction == "左转"
        assert bundle.road.superelevation == pytest.approx(-0.04)
        assert bundle.combined_cg_height == pytest.approx(3.5)
        window.unpack_model_data(old_model)
        assert window.collect_bundle().combined_cg_height is None
    finally:
        window.close()
        app.processEvents()
