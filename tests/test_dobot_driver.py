"""The real DobotDriver against the fake controller on random localhost ports."""

import time

import pytest
from helpers import dobot_cfg, wait_for

from botswain.core.types import Joints, Mode
from botswain.drivers.base import DriverError
from botswain.drivers.dobot.dashboard import NO_LINK, NOT_TCP
from botswain.drivers.dobot.driver import DobotDriver
from botswain.drivers.dobot.fake_controller import FakeDobotController
from botswain.drivers.registry import make_driver


@pytest.fixture
def ctrl():
    c = FakeDobotController()
    c.start()
    yield c
    c.stop()


@pytest.fixture
def drv(ctrl):
    d = DobotDriver(dobot_cfg(ctrl))
    yield d
    d.disconnect()


def connected(d):
    d.connect()
    assert wait_for(lambda: d.read_state().link_ok)
    return d


def enabled(d):
    connected(d)
    d.enable()
    assert wait_for(lambda: d.read_state().mode is Mode.IDLE)
    return d


def test_registry_builds_dobot(ctrl):
    assert isinstance(make_driver(dobot_cfg(ctrl)), DobotDriver)


def test_connect_reads_state_without_enabling(ctrl, drv):
    connected(drv)
    s = drv.read_state()
    assert s.mode is Mode.DISABLED and s.raw_mode == 4
    assert s.joints == Joints((0, 0, 90, 0, -90, 0))
    assert not any(c.startswith("EnableRobot") for c in ctrl.commands)
    assert "SpeedFactor(10)" in ctrl.commands


def test_enable_disable_modes(drv):
    enabled(drv)
    drv.disable()
    assert wait_for(lambda: drv.read_state().mode is Mode.DISABLED)


def test_alarm_maps_to_error_and_clear(ctrl, drv):
    enabled(drv)
    ctrl.inject_alarm(22)
    assert wait_for(lambda: drv.read_state().mode is Mode.ERROR)
    with pytest.raises(DriverError) as e:
        drv.jog_step("X", 1.0, "base")
    assert e.value.code == -2
    assert drv.read_state().errors == (22,)
    drv.clear_error()
    assert wait_for(lambda: drv.read_state().mode is Mode.IDLE)


def test_not_tcp_gives_hint(ctrl, drv):
    ctrl.tcp_mode = False
    with pytest.raises(DriverError) as e:
        drv.connect()
    assert e.value.code == NOT_TCP and "TCP/IP" in e.value.hint
    assert drv.read_state().mode is Mode.DISCONNECTED


def test_feedback_rate_about_125hz(drv):
    connected(drv)
    time.sleep(1.2)
    assert 100 <= drv.read_state().feedback_hz <= 140


def test_split_and_garbage_feedback_still_parses(ctrl, drv):
    ctrl.split_packets = True
    ctrl.garbage_every = 5
    connected(drv)
    time.sleep(0.5)
    s = drv.read_state()
    assert s.link_ok and s.joints == Joints((0, 0, 90, 0, -90, 0))
    assert drv._feedback.bad_frames >= 1


def test_link_lost_when_feedback_stops(ctrl, drv):
    connected(drv)
    t0 = time.perf_counter()
    ctrl.feedback_paused = True
    assert wait_for(lambda: not drv.read_state().link_ok, timeout=0.5)
    assert time.perf_counter() - t0 < 0.5
    assert drv.read_state().feedback_hz == 0.0
    ctrl.feedback_paused = False
    assert wait_for(lambda: drv.read_state().link_ok)


def test_disconnect_sends_stop_first(ctrl, drv):
    enabled(drv)
    drv.jog_hold("X", 1, "base")
    assert wait_for(lambda: drv.read_state().mode is Mode.JOGGING)
    drv.disconnect()
    assert ctrl.commands[-2:] == ["MoveJog()", "Stop()"]
    assert drv.read_state().mode is Mode.DISCONNECTED


def test_move_to_linear_reaches_target(drv):
    enabled(drv)
    drv.set_speed(100)
    p = drv.read_state().tcp
    target = p.with_axis("Z", p.z + 20)
    drv.move_to(target, "linear")
    assert wait_for(lambda: drv.read_state().mode is Mode.MOVING, timeout=0.5)
    assert wait_for(lambda: drv.read_state().mode is Mode.IDLE, timeout=2.0)
    assert drv.read_state().tcp.z == pytest.approx(target.z, abs=1e-6)


def test_jog_commands_per_frame(ctrl, drv):
    enabled(drv)
    drv.jog_step("X", 1.0, "base")
    drv.jog_step("Z", -1.0, "tool")
    drv.jog_step("J6", 2.0, "base")
    sent = [c for c in ctrl.commands if c.startswith("Rel")]
    assert sent[0].startswith("RelMovLUser(1.000000") and sent[0].endswith("user=0)")
    assert sent[1].startswith("RelMovLTool(0.000000,0.000000,-1.000000")
    assert sent[2].startswith("RelJointMovJ(")


def test_hold_jog_coordtypes(ctrl, drv):
    enabled(drv)
    drv.jog_hold("J2", -1, "base")
    drv.jog_release()
    drv.jog_hold("Ry", 1, "tool")
    drv.jog_release()
    assert [c for c in ctrl.commands if c.startswith("MoveJog")] == [
        "MoveJog(J2-,coordtype=0)", "MoveJog()", "MoveJog(Ry+,coordtype=2)", "MoveJog()"]


def test_stream_target_uses_servo_p(ctrl, drv):
    enabled(drv)
    p = drv.read_state().tcp
    drv.stream_target(p.with_axis("Y", p.y + 1))
    assert any(c.startswith("ServoP(") and "t=0.030" in c for c in ctrl.commands)
    assert wait_for(lambda: drv.read_state().mode is Mode.STREAMING, timeout=0.5)
    drv.stop()
    assert drv.read_state().mode is not Mode.STREAMING


def test_gripper_frames_reach_controller(ctrl, drv):
    connected(drv)
    drv.gripper(True)
    drv.gripper(False)
    writes = [f for f in ctrl.gripper_frames if f[1] == 0x06]  # reads (03) are the status poll
    assert writes == [[1, 6, 1, 0, 0, 1, 73, 246], [1, 6, 1, 3, 3, 232, 120, 136], [1, 6, 1, 3, 0, 0, 120, 54]]
    assert drv.read_state().gripper == 0.0


def test_stop_raises_when_link_is_dead(ctrl, drv):
    connected(drv)
    ctrl.stop()
    with pytest.raises(DriverError) as e:
        drv.stop()
    assert e.value.code == NO_LINK and "e-stop" in e.value.hint


def test_motion_port_can_be_separate(ctrl):
    c2 = FakeDobotController(motion_port=0)
    c2.start()
    try:
        d = DobotDriver(dobot_cfg(c2, motion_port=c2.ports["motion"]))
        enabled(d)
        d.jog_step("X", 1.0, "base")
        assert any(c.startswith("RelMovLUser") for c in c2.commands)
        d.disconnect()
    finally:
        c2.stop()


def test_speed_control_scales_to_the_controller_max():
    # Dobot's SpeedFactor 10 % jogs far faster than the gamepad's 10 %; the YAML sets what 100 % means.
    from dataclasses import replace

    ctrl = FakeDobotController()
    ctrl.start()
    cfg = dobot_cfg(ctrl)
    d = DobotDriver(replace(cfg, limits=replace(cfg.limits, controller_speed_max_pct=20)))
    try:
        d.connect()
        d.set_speed(10)
        d.set_speed(100)
        d.set_speed(1)
        assert [c for c in ctrl.commands if c.startswith("SpeedFactor")] == [
            "SpeedFactor(2)", "SpeedFactor(2)", "SpeedFactor(20)", "SpeedFactor(1)"]
        assert d.read_state().speed_pct == 1  # the GUI keeps showing its own %
    finally:
        d.disconnect()
        ctrl.stop()
