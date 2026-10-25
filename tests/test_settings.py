"""Settings page: picking robot + end effector, lab edits to the overlay (never the shared file), teaching the
safety box by corners, and when Save & restart is allowed. Uses a copy of the fake robot config in tmp_path."""

import pytest
import yaml
from helpers import DH_TOOL, ROBOTS

from botswain.app import build
from botswain.core.config import load_robot_config, load_tool_config, overlay_path
from botswain.core.types import Connect, Enable, JogStep, Mode


@pytest.fixture
def rig(qtbot, tmp_path):
    data = yaml.safe_load((ROBOTS / "fake.yaml").read_text(encoding="utf-8"))
    data["urdf"] = str(load_robot_config(ROBOTS / "fake.yaml").model_urdf)
    path = tmp_path / "fake.yaml"
    path.write_text(yaml.safe_dump(data), encoding="utf-8")
    worker, w = build(load_robot_config(path), view_3d=False, tool_cfg=load_tool_config(DH_TOOL))
    qtbot.addWidget(w)
    loads = []
    w.settings_page._switch = lambda robot, tool: loads.append((robot, tool))
    yield worker, w, w.settings_page, path, loads
    w.close()


def test_lists_configs_and_shows_what_is_running(rig):
    _, _, page, _, _ = rig
    robots = [page.robot_box.itemText(i) for i in range(page.robot_box.count())]
    assert {"Dobot CR5", "Dobot CR5 (fake controller)", "Fake CR5"} <= set(robots)
    assert page.tool_box.itemText(0) == "None" and page.tool_box.currentText() == "DH PGE-50-26"
    assert "dh_modbus" in page.tool_details.text() and "[ ] not filled in" in page.tool_details.text()


def test_load_only_while_disconnected(qtbot, rig):
    worker, _, page, _, loads = rig
    page.robot_box.setCurrentIndex(page.robot_box.findText("Dobot CR5"))
    qtbot.waitUntil(page.restart_btn.isEnabled, timeout=1000)
    worker.submit(Connect())
    qtbot.waitUntil(lambda: not page.restart_btn.isEnabled(), timeout=2000)  # connected: no switching
    assert "Disconnect first" in page.apply_note.text()
    page.load_selected()
    assert loads == []


def test_incompatible_end_effector_is_refused(rig):
    from dataclasses import replace

    _, _, page, _, _ = rig
    path, cfg, err = page.robot_box.currentData()
    page.robot_box.setItemData(page.robot_box.currentIndex(), (path, replace(cfg, tool_port=None), err))
    page._update_details()
    assert "needs an rs485 tool port" in page.apply_note.text() and not page.restart_btn.isEnabled()


def test_switching_to_no_end_effector_loads_with_none(qtbot, rig):
    _, _, page, path, loads = rig
    page.tool_box.setCurrentIndex(0)
    qtbot.waitUntil(page.restart_btn.isEnabled, timeout=1000)
    page.restart_btn.click()
    assert loads == [(path, None)]


def test_lab_edits_go_to_the_overlay_and_bad_ones_are_undone(rig):
    _, _, page, path, _ = rig
    page.cap_box.setValue(60)
    page.save_speed()
    shared = yaml.safe_load(path.read_text(encoding="utf-8"))
    assert shared["limits"]["speed_cap_pct"] == 100  # the shared config is untouched
    assert load_robot_config(path).limits.speed_cap_pct == 60
    before = overlay_path(path).read_text(encoding="utf-8")
    page.box_fields["x"][1].setValue(page.box_fields["x"][0].value() + 5)  # under the 20 mm minimum
    page.save_box()
    assert overlay_path(path).read_text(encoding="utf-8") == before


def test_teach_box_in_the_robot_view_applies_and_saves(qtbot, rig):
    """Robot view > Box: corner 1, move (here only X and Z, so Y stays flat), corner 2, Apply."""
    worker, w, page, path, _ = rig
    jog = w.jog_page
    worker.submit(Connect())
    worker.submit(Enable())
    qtbot.waitUntil(lambda: worker.latest_state().mode is Mode.IDLE, timeout=2000)
    qtbot.waitUntil(lambda: jog._state is not None and jog._state.link_ok, timeout=2000)

    def gui_caught_up():  # corners come from the Jog page's copy of the state (published at 30 Hz)
        return all(abs(a - b) < 0.01 for a, b in zip(jog._state.tcp.xyz, worker.latest_state().tcp.xyz, strict=True))

    qtbot.waitUntil(gui_caught_up, timeout=2000)
    jog.robot_view.corner_requested.emit(0)
    x0, _, z0 = jog._state.tcp.xyz
    for i, (axis, d) in ((0, ("X", 60.0)), (2, ("Z", -40.0))):
        start = worker.latest_state().tcp.xyz[i]
        worker.submit(JogStep(axis, d, "base"))
        qtbot.waitUntil(lambda i=i, start=start, d=d: abs(worker.latest_state().tcp.xyz[i] - start - d) < 0.05
                        and worker.latest_state().mode is Mode.IDLE, timeout=5000)
    qtbot.waitUntil(gui_caught_up, timeout=2000)
    jog.robot_view.corner_requested.emit(1)
    old_y = worker.gate.limits.box_mm[1]
    jog.robot_view.box_apply_requested.emit()
    (xl, xh), y, (zl, zh) = worker.gate.limits.box_mm
    assert xl == pytest.approx(x0, abs=0.2) and xh == pytest.approx(x0 + 60, abs=0.2)
    assert zl == pytest.approx(z0 - 40, abs=0.2) and zh == pytest.approx(z0, abs=0.2)
    assert y == old_y  # flat axis: kept from the current box
    assert worker.gate.box_enabled and jog.robot_view.box_chip.isChecked()
    assert load_robot_config(path).limits.box_mm == worker.gate.limits.box_mm  # saved to the overlay
    assert page.box_fields["x"][1].value() == pytest.approx(x0 + 60, abs=0.2)  # Settings shows it
    assert jog.box_corners == []
    qtbot.waitUntil(lambda: "60×" in w.status.box.text(), timeout=1000)



def test_load_swaps_the_window_in_the_same_app(qtbot, tmp_path, monkeypatch):
    """The real app path: Load builds the new window, the old one closes, the app keeps running."""
    from PySide6.QtWidgets import QApplication

    import botswain.app as app_mod
    from botswain.core import config as config_mod

    monkeypatch.setattr(config_mod, "SELECTION", tmp_path / "selection.yaml")
    monkeypatch.setattr(app_mod, "write_selection", lambda robot, tool: None)
    windows = []
    real_build = app_mod.build
    def build(cfg, view_3d=True, tool_cfg=None):
        worker, window = real_build(cfg, False, tool_cfg)  # no 3D view in tests
        windows.append(window)
        return worker, window

    monkeypatch.setattr(app_mod, "build", build)
    monkeypatch.setattr(QApplication, "exec", lambda self: 0)  # run() returns right after the first window
    monkeypatch.setattr(app_mod, "QApplication", lambda argv: QApplication.instance())
    app_mod.run(load_robot_config(ROBOTS / "fake.yaml"), load_tool_config(DH_TOOL))
    first = windows[0]
    first.switch_requested.emit(ROBOTS / "dobot_cr5_fake_controller.yaml", None)
    qtbot.waitUntil(lambda: len(windows) == 2 and not first.isVisible(), timeout=5000)
    second = windows[1]
    assert second.isVisible() and second.cfg.name == "Dobot CR5 (fake controller)"
    assert second.tool_cfg is None and not second.jog_page.gripper_open_btn.isVisibleTo(second)
    second.close()


def test_switching_the_box_off_and_on_survives_a_restart(qtbot, rig):
    """MS: once a box is saved and applied, the next start must have the same box, switched the same way."""
    worker, w, page, path, _ = rig
    page.save_box_values(((-500.0, 400.0), (-300.0, 350.0), (100.0, 600.0)), enabled=True)
    w.jog_page.robot_view.box_chip.setChecked(False)  # switched off in the Robot view (or the VR button)
    assert not worker.gate.box_enabled and not page.box_on.isChecked()
    after = load_robot_config(path)  # what the next start loads
    assert after.limits.box_mm == ((-500.0, 400.0), (-300.0, 350.0), (100.0, 600.0)) and after.limits.box_enabled is False
    w.jog_page.robot_view.box_chip.setChecked(True)
    assert load_robot_config(path).limits.box_enabled is True
    worker2, w2 = build(load_robot_config(path), view_3d=False)  # a fresh start
    qtbot.addWidget(w2)
    assert worker2.gate.box_enabled and worker2.gate.limits.box_mm == after.limits.box_mm
    assert w2.jog_page.robot_view.box_chip.isChecked()
    w2.close()
