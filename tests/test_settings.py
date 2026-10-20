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
    restarts = []
    w.settings_page._restart = lambda robot, tool: restarts.append((robot, tool))
    yield worker, w, w.settings_page, path, restarts
    w.close()


def test_lists_configs_and_shows_what_is_running(rig):
    _, _, page, _, _ = rig
    robots = [page.robot_box.itemText(i) for i in range(page.robot_box.count())]
    assert {"Dobot CR5", "Dobot CR5 (fake controller)", "Fake CR5"} <= set(robots)
    assert page.tool_box.itemText(0) == "None" and page.tool_box.currentText() == "DH PGE-50-26"
    assert "dh_modbus" in page.tool_details.text() and "[ ] not filled in" in page.tool_details.text()


def test_restart_only_for_a_real_change_and_while_disconnected(qtbot, rig):
    worker, _, page, _, restarts = rig
    page.robot_box.setCurrentIndex(page.robot_box.findText("Dobot CR5"))
    qtbot.waitUntil(page.restart_btn.isEnabled, timeout=1000)
    worker.submit(Connect())
    qtbot.waitUntil(lambda: not page.restart_btn.isEnabled(), timeout=2000)  # connected: no switching
    assert "Disconnect first" in page.apply_note.text()
    page.save_and_restart()
    assert restarts == []


def test_incompatible_end_effector_is_refused(rig):
    from dataclasses import replace

    _, _, page, _, _ = rig
    path, cfg, err = page.robot_box.currentData()
    page.robot_box.setItemData(page.robot_box.currentIndex(), (path, replace(cfg, tool_port=None), err))
    page._update_details()
    assert "needs an rs485 tool port" in page.apply_note.text() and not page.restart_btn.isEnabled()


def test_switching_to_no_end_effector_restarts_with_none(qtbot, rig):
    _, _, page, path, restarts = rig
    page.tool_box.setCurrentIndex(0)
    qtbot.waitUntil(page.restart_btn.isEnabled, timeout=1000)
    page.restart_btn.click()
    assert restarts == [(path, None)]


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


def test_teach_box_by_corners_applies_at_once(qtbot, rig):
    worker, w, page, path, _ = rig
    worker.submit(Connect())
    worker.submit(Enable())
    qtbot.waitUntil(lambda: worker.latest_state().mode is Mode.IDLE, timeout=2000)
    qtbot.waitUntil(page.add_corner_btn.isEnabled, timeout=1000)
    page.add_corner()
    x0, y0, z0 = worker.latest_state().tcp.xyz
    for i, (axis, d) in enumerate((("X", 60.0), ("Y", -40.0), ("Z", 30.0))):
        start = worker.latest_state().tcp.xyz[i]
        worker.submit(JogStep(axis, d, "base"))
        qtbot.waitUntil(lambda i=i, start=start, d=d: abs(worker.latest_state().tcp.xyz[i] - start - d) < 0.05
                        and worker.latest_state().mode is Mode.IDLE, timeout=5000)
    page.add_corner()
    page.margin_box.setValue(10.0)
    page.fit_box()
    page.box_on.setChecked(True)
    page.save_box()
    (xl, xh), (yl, yh), (zl, zh) = worker.gate.limits.box_mm
    assert xl == pytest.approx(x0 - 10, abs=0.2) and xh == pytest.approx(x0 + 70, abs=0.2)
    assert yl == pytest.approx(y0 - 50, abs=0.2) and yh == pytest.approx(y0 + 10, abs=0.2)
    assert zl == pytest.approx(z0 - 10, abs=0.2) and zh == pytest.approx(z0 + 40, abs=0.2)
    assert worker.gate.box_enabled and w.jog_page.robot_view.box_chip.isChecked()
    assert load_robot_config(path).limits.box_mm == worker.gate.limits.box_mm  # saved for next time
    qtbot.waitUntil(lambda: "80×" in w.status.box.text(), timeout=1000)
