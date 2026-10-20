"""Jog page, STOP keys, points and home against the fake robot. Commands are recorded on their way to the worker."""

import time

import pytest
from helpers import DH_TOOL, ROBOTS
from PySide6.QtCore import Qt

from botswain.app import build
from botswain.core.config import load_robot_config, load_tool_config
from botswain.core.types import (
    Connect,
    Enable,
    Gripper,
    JogHold,
    JogRelease,
    JogStep,
    JogTick,
    Joints,
    Mode,
    MoveTo,
    Pose,
    SetSpeed,
)


@pytest.fixture
def rig(qtbot, tmp_path):
    worker, w = build(load_robot_config(ROBOTS / "fake.yaml"), view_3d=False,
                     tool_cfg=load_tool_config(DH_TOOL))
    qtbot.addWidget(w)
    w.points.path = tmp_path / "points.json"  # keep test points out of robots/
    w.points._points.clear()  # ignore points saved by hand in robots/fake.points.json
    sent, stops = [], []
    real_submit, real_stop = worker.submit, worker.request_stop
    worker.submit = lambda c: (sent.append(c), real_submit(c))
    worker.request_stop = lambda: (stops.append(time.perf_counter()), real_stop())
    yield worker, w, sent, stops
    w.close()


def ready(qtbot, worker, w):
    worker.submit(Connect())
    worker.submit(Enable())
    qtbot.waitUntil(lambda: worker.latest_state().mode is Mode.IDLE, timeout=2000)
    qtbot.waitUntil(lambda: w.jog_page.jog_pad.keys[("X", 1)].isEnabled(), timeout=1000)


def of(sent, kind):
    return [c for c in sent if isinstance(c, kind)]


def test_step_jog_button_submits_jogstep(qtbot, rig):
    worker, w, sent, _ = rig
    ready(qtbot, worker, w)
    w.jog_page.jog_pad.keys[("X", 1)].click()
    w.jog_page.step_selector.buttons[5.0].click()
    qtbot.waitUntil(lambda: worker.latest_state().mode is Mode.IDLE, timeout=2000)
    qtbot.waitUntil(lambda: w.jog_page.jog_pad.keys[("J2", -1)].isEnabled(), timeout=1000)
    w.jog_page.jog_pad.keys[("J2", -1)].click()
    assert of(sent, JogStep) == [JogStep("X", 1.0, "base"), JogStep("J2", -5.0, "base")]


def test_tool_frame_reaches_cartesian_steps(qtbot, rig):
    worker, w, sent, _ = rig
    ready(qtbot, worker, w)
    w.jog_page.frame_box.setCurrentIndex(w.jog_page.frame_box.findData("tool"))
    w.jog_page.jog_pad.keys[("Z", -1)].click()
    assert of(sent, JogStep) == [JogStep("Z", -1.0, "tool")]


def test_hold_mode_sends_hold_ticks_release(qtbot, rig):
    worker, w, sent, _ = rig
    ready(qtbot, worker, w)
    w.jog_page.step_selector.buttons[None].click()
    key = w.jog_page.jog_pad.keys[("Y", -1)]
    qtbot.mousePress(key, Qt.MouseButton.LeftButton)
    qtbot.wait(300)
    qtbot.mouseRelease(key, Qt.MouseButton.LeftButton)
    assert of(sent, JogHold) == [JogHold("Y", -1, "base")]
    assert len(of(sent, JogTick)) >= 4
    assert of(sent, JogRelease)
    qtbot.waitUntil(lambda: worker.latest_state().mode is Mode.IDLE, timeout=1000)


def test_jog_disabled_when_not_idle(qtbot, rig):
    worker, w, sent, _ = rig
    worker.submit(Connect())
    qtbot.waitUntil(lambda: w.toolbar.connect_btn.text() == "Disconnect", timeout=2000)
    key = w.jog_page.jog_pad.keys[("X", 1)]
    assert not key.isEnabled() and key.toolTip() == "robot not enabled"
    key.click()
    assert of(sent, JogStep) == []


def test_space_stops_even_with_jog_focus(qtbot, rig):
    worker, w, sent, stops = rig
    ready(qtbot, worker, w)
    key = w.jog_page.jog_pad.keys[("X", 1)]
    key.setFocus()  # forced: its policy is NoFocus, this is the worst case
    qtbot.keyClick(key, Qt.Key.Key_Space)
    assert stops and of(sent, JogStep) == []


def test_space_stops_while_typing_a_target(qtbot, rig):
    worker, w, _, stops = rig
    ready(qtbot, worker, w)
    w.jog_page.edit_btn.click()
    field = w.jog_page.pose_fields.pose_edits["X"]
    field.setFocus()
    qtbot.keyClick(field, Qt.Key.Key_Space)
    assert stops and " " not in field.text()


def test_esc_stops(qtbot, rig):
    _, w, _, stops = rig
    qtbot.keyClick(w, Qt.Key.Key_Escape)
    assert stops


def test_stop_button_stops_on_press(qtbot, rig):
    _, w, _, stops = rig
    qtbot.mousePress(w.toolbar.stop_button, Qt.MouseButton.LeftButton)
    assert stops


def test_gripper_buttons_send_gripper_cmd(qtbot, rig):
    worker, w, sent, _ = rig
    ready(qtbot, worker, w)
    w.jog_page.gripper_close_btn.click()
    w.jog_page.gripper_open_btn.click()
    assert of(sent, Gripper) == [Gripper(False), Gripper(True)]


def test_speed_buttons_send_set_speed(qtbot, rig):
    worker, w, sent, _ = rig
    ready(qtbot, worker, w)
    qtbot.waitUntil(lambda: w.jog_page.speed_control.slider.value() == 10, timeout=1000)
    minus_one = next(b for b in w.jog_page.speed_control.findChildren(type(w.jog_page.move_btn)) if b.text() == "−1")
    minus_one.click()
    assert of(sent, SetSpeed) == [SetSpeed(9)]


def test_move_to_target_only_on_button(qtbot, rig):
    worker, w, sent, _ = rig
    ready(qtbot, worker, w)
    page = w.jog_page
    page.edit_btn.click()
    page.copy_btn.click()
    z = page.pose_fields.pose_edits["Z"]
    z.setText("500.0")
    z.textEdited.emit("500.0")
    qtbot.wait(100)
    assert of(sent, MoveTo) == []
    page.move_btn.click()
    moves = of(sent, MoveTo)
    assert len(moves) == 1 and moves[0].kind == "linear" and isinstance(moves[0].target, Pose)
    assert moves[0].target.z == 500.0


def test_joint_target_moves_with_joint_move(qtbot, rig):
    worker, w, sent, _ = rig
    ready(qtbot, worker, w)
    page = w.jog_page
    page.edit_btn.click()
    page.copy_btn.click()
    page.pose_fields.joint_edits["J1"].textEdited.emit("5")
    page.move_btn.click()
    move = of(sent, MoveTo)[0]
    assert move.kind == "joint" and isinstance(move.target, Joints)


def test_save_point_and_go_to_through_worker(qtbot, rig):
    worker, w, sent, _ = rig
    ready(qtbot, worker, w)
    w.toolbar.save_point_btn.click()
    assert [p.name for p in w.points.all()] == ["P1"]
    assert w.points_page.table.rowCount() == 1 and w.points_dock_view.list.count() == 1
    w.points_page.table.selectRow(0)
    w.points_page.go_btn.click()
    assert of(sent, MoveTo) == [MoveTo(w.points.get("P1").joints, "joint")]


def test_home_moves_to_yaml_joints(qtbot, rig):
    worker, w, sent, _ = rig
    ready(qtbot, worker, w)
    qtbot.waitUntil(w.toolbar.home_btn.isEnabled, timeout=1000)
    w.toolbar.home_btn.click()
    assert of(sent, MoveTo) == [MoveTo(Joints((0, 0, 90, 0, -90, 0)), "joint")]


def test_hold_jog_released_at_box_wall(qtbot, rig):
    worker, w, _, _ = rig
    ready(qtbot, worker, w)
    w.jog_page.robot_view.box_chip.setChecked(True)  # box is off by default
    assert worker.gate.box_enabled
    worker.submit(SetSpeed(20))  # 50 mm/s on the fake
    w.jog_page.step_selector.buttons[None].click()
    start = worker.latest_state().tcp
    worker.driver.place_at(start.with_axis("X", -340.0))  # 8 mm from the X+ wall
    qtbot.waitUntil(lambda: abs(worker.latest_state().tcp.x + 340.0) < 1e-3, timeout=500)
    key = w.jog_page.jog_pad.keys[("X", 1)]
    qtbot.mousePress(key, Qt.MouseButton.LeftButton)
    qtbot.waitUntil(lambda: worker.latest_state().mode is Mode.JOGGING, timeout=1000)
    qtbot.waitUntil(lambda: worker.latest_state().mode is Mode.IDLE, timeout=2000)  # released by the gate
    qtbot.mouseRelease(key, Qt.MouseButton.LeftButton)
    # one or two worker ticks of overshoot at 50 mm/s when the test thread competes for the GIL
    # predictive stop: releases 1 mm + 60 ms of travel before the wall (about 4 mm at 50 mm/s)
    assert -332.0 - 6.0 <= worker.latest_state().tcp.x <= -332.0 + 2.0
    qtbot.waitUntil(lambda: "released (box)" in w.log_dock.log_view.toPlainText(), timeout=1000)


def test_space_types_in_task_field_but_esc_still_stops(qtbot, rig):
    _, w, _, stops = rig
    w.show()
    field = w.episode_bar.task_edit
    field.setFocus()
    qtbot.keyClicks(field, "pick")
    qtbot.keyClick(field, Qt.Key.Key_Space)
    qtbot.keyClicks(field, "cube")
    assert field.text() == "pick cube" and not stops
    qtbot.keyClick(field, Qt.Key.Key_Escape)
    assert stops


def test_episode_bar_lives_in_episodes_tab(qtbot, rig):
    _, w, _, _ = rig
    tabs = w.log_dock.tabs
    assert tabs.tabText(2) == "Episodes" and tabs.widget(2) is w.episode_bar
    if w.episode_bar.session is None:
        assert not w.episode_bar.start_btn.isEnabled()
    else:
        assert w.episode_bar.start_btn.isEnabled() and not w.episode_bar.save_btn.isEnabled()


def test_fields_go_live_after_move_to_target(qtbot, rig):
    # Lab: after typing home angles and moving, the fields stayed frozen on the typed numbers.
    worker, w, _, _ = rig
    ready(qtbot, worker, w)
    page = w.jog_page
    page.edit_btn.click()
    page.copy_btn.click()
    page.pose_fields.joint_edits["J1"].setText("8")
    page.pose_fields.joint_edits["J1"].textEdited.emit("8")
    page.move_btn.click()
    assert not page.edit_btn.isChecked() and not page.pose_fields.editing
    qtbot.waitUntil(lambda: abs(float(page.pose_fields.joint_edits["J1"].text()) - 8.0) < 0.01, timeout=3000)


def test_hold_survives_a_running_mode(qtbot, rig):
    # Older Dobot firmware may report a MoveJog as mode 7 (running): the hold must not be dropped for it.
    from dataclasses import replace

    worker, w, sent, _ = rig
    ready(qtbot, worker, w)
    page = w.jog_page
    page.step_selector.buttons[None].click()
    key = page.jog_pad.keys[("J1", 1)]
    qtbot.mousePress(key, Qt.MouseButton.LeftButton)
    page.set_state(replace(worker.latest_state(), mode=Mode.MOVING))
    assert page.jog_pad.holding and key.isEnabled() and of(sent, JogRelease) == []
    page.set_state(replace(worker.latest_state(), mode=Mode.ERROR))  # a real problem still lets go
    assert not page.jog_pad.holding and of(sent, JogRelease)
    qtbot.mouseRelease(key, Qt.MouseButton.LeftButton)


def test_jog_page_home_button(qtbot, rig):
    worker, w, sent, _ = rig
    ready(qtbot, worker, w)
    btn = w.jog_page.home_btn
    qtbot.waitUntil(btn.isEnabled, timeout=1000)
    btn.click()
    assert of(sent, MoveTo) == [MoveTo(Joints((0, 0, 90, 0, -90, 0)), "joint")]



def test_gripper_close_uses_the_force_box(qtbot, rig):
    from botswain.core.types import SetGripperForce

    worker, w, sent, _ = rig
    ready(qtbot, worker, w)
    page = w.jog_page
    page.gripper_close_btn.click()  # first close: the 50 % shown in the box goes first
    page.force_box.setValue(80)  # a change is sent at once, for gamepad and phone closes too
    page.gripper_close_btn.click()
    grip = [c for c in sent if isinstance(c, SetGripperForce | Gripper)]
    assert grip == [SetGripperForce(50), Gripper(False), SetGripperForce(80), Gripper(False)]
    assert wait_for_state(qtbot, lambda: "at position" in page.gripper_status.text())


def wait_for_state(qtbot, cond):
    qtbot.waitUntil(cond, timeout=2000)
    return True
