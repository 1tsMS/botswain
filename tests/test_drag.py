"""Drag teach: fake robot, gate, Dobot protocol (STOP ends drag), toolbar toggle, and a recorded drag episode
whose action is the next measured pose."""

import time
from importlib.util import find_spec

import numpy as np
import pytest
from helpers import DH_TOOL, ROBOTS, dobot_cfg, make_state, wait_for

from botswain.control.safety import Allowed, Rejected, SafetyGate
from botswain.control.worker import RobotWorker
from botswain.core.config import load_robot_config, load_tool_config
from botswain.core.types import Connect, Drag, Enable, Mode
from botswain.drivers.base import DriverError
from botswain.drivers.dobot.driver import DobotDriver
from botswain.drivers.dobot.fake_controller import FakeDobotController
from botswain.drivers.fake import FakeDriver


@pytest.fixture
def worker():
    cfg = load_robot_config(ROBOTS / "fake.yaml")
    w = RobotWorker(FakeDriver(cfg), SafetyGate(cfg.limits, cfg.stream.period_s))
    w.start()
    w.submit(Connect())
    w.submit(Enable())
    assert wait_for(lambda: w.latest_state().mode is Mode.IDLE)
    yield w
    w.shutdown()


def test_fake_drag_follows_the_hand_and_stop_ends_it(worker):
    worker.submit(Drag(True))
    assert wait_for(lambda: worker.latest_state().mode is Mode.DRAG)
    j1 = worker.latest_state().joints.values[0]
    worker.driver.push((5, 0, 0, 0, 0, 0))
    assert wait_for(lambda: worker.latest_state().joints.values[0] == pytest.approx(j1 + 5))
    worker.request_stop()
    assert wait_for(lambda: worker.latest_state().mode is Mode.IDLE)
    worker.driver.push((5, 0, 0, 0, 0, 0))  # not in drag: the arm does not give way
    time.sleep(0.05)
    assert worker.latest_state().joints.values[0] == pytest.approx(j1 + 5)


def test_gate_starts_drag_only_when_idle():
    cfg = load_robot_config(ROBOTS / "fake.yaml")
    gate = SafetyGate(cfg.limits, cfg.stream.period_s)
    assert isinstance(gate.check(Drag(True), make_state(Mode.IDLE)), Allowed)
    assert isinstance(gate.check(Drag(True), make_state(Mode.DISABLED)), Rejected)
    assert isinstance(gate.check(Drag(True), make_state(Mode.STREAMING)), Rejected)
    assert isinstance(gate.check(Drag(False), make_state(Mode.DRAG)), Allowed)
    assert isinstance(gate.check(Drag(False), make_state(Mode.DISCONNECTED, link_ok=False)), Rejected)


def test_teleop_and_jogs_are_refused_during_drag(worker):
    from botswain.core.clock import now_ns
    from botswain.core.types import JogStep, StreamTarget

    worker.submit(Drag(True))
    assert wait_for(lambda: worker.latest_state().mode is Mode.DRAG)
    s = worker.latest_state()
    assert isinstance(worker.gate.check(JogStep("X", 1.0, "base"), s), Rejected)
    assert isinstance(worker.gate.check(StreamTarget(s.tcp, now_ns(), "vr", True), s), Rejected)


def test_dobot_drag_commands_and_stop():
    ctrl = FakeDobotController()
    ctrl.start()
    d = DobotDriver(dobot_cfg(ctrl))
    try:
        d.connect()
        assert d.capabilities.drag_mode
        with pytest.raises(DriverError):
            d.drag(True)  # disabled: the controller refuses
        d.enable()
        assert wait_for(lambda: d.read_state().mode is Mode.IDLE)
        d.drag(True)
        assert wait_for(lambda: d.read_state().mode is Mode.DRAG)
        before = len(ctrl.commands)
        d.stop()  # STOP must end drag explicitly, first
        assert ctrl.commands[before] == "StopDrag()"
        assert wait_for(lambda: d.read_state().mode is Mode.IDLE)
    finally:
        d.disconnect()
        ctrl.stop()


def test_toolbar_toggles_drag(qtbot):
    from botswain.app import build

    worker, w = build(load_robot_config(ROBOTS / "fake.yaml"), view_3d=False,
                     tool_cfg=load_tool_config(DH_TOOL))
    qtbot.addWidget(w)
    try:
        worker.submit(Connect())
        worker.submit(Enable())
        btn = w.toolbar.drag_btn
        qtbot.waitUntil(btn.isEnabled, timeout=2000)
        btn.click()
        qtbot.waitUntil(lambda: worker.latest_state().mode is Mode.DRAG and btn.text() == "End drag", timeout=2000)
        btn.click()
        qtbot.waitUntil(lambda: worker.latest_state().mode is Mode.IDLE and btn.text() == "Drag mode", timeout=2000)
    finally:
        w.close()


@pytest.mark.skipif(find_spec("lerobot") is None, reason="LeRobot not installed")
def test_recorded_drag_action_is_the_next_measured_pose(worker, tmp_path):
    import pandas as pd

    from botswain.record.check import check_dataset
    from botswain.record.session import RecordingSession, lerobot_writer_factory

    session = RecordingSession(worker, lerobot_writer_factory("fake"))
    worker.submit(Drag(True))
    assert wait_for(lambda: worker.latest_state().mode is Mode.DRAG)
    session.start(tmp_path / "drag", "botswain/drag", "drag: sweep J1", 30, {})
    for _ in range(40):  # a hand sweeping J1 at about 1 deg per frame
        worker.driver.push((1.0, 0, 0, 0, 0, 0))
        time.sleep(1 / 30)
    assert session.save() > 30
    session.close()

    d = pd.concat(pd.read_parquet(f) for f in (tmp_path / "drag" / "data").glob("chunk-*/*.parquet"))
    state = np.stack(d["observation.state"].to_numpy())
    action = np.stack(d["action"].to_numpy())
    tcp = state[:, 6:12]
    assert (d["observation.input_source"] == 5).all()
    np.testing.assert_allclose(action[:-1, :6], tcp[1:], atol=1e-3)  # action t = measured pose t+1
    np.testing.assert_allclose(action[-1, :6], tcp[-1], atol=1e-3)  # last frame: it stayed there
    assert np.linalg.norm(tcp[-1, :3] - tcp[0, :3]) > 50  # the hand really moved the arm
    report = check_dataset(tmp_path / "drag")
    assert report.episodes[0].main_source == "drag" and not report.problems
