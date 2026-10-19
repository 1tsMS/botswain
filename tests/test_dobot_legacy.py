"""Older controller firmware (lab CR5, 2026-10-19): keyword forms refused with -30001/-30002,
positional forms accepted. The driver falls back once, remembers, and never retries other errors."""

import pytest
from helpers import dobot_cfg, wait_for

from botswain.core.types import Joints, Mode
from botswain.drivers.base import DriverError
from botswain.drivers.dobot.driver import DobotDriver
from botswain.drivers.dobot.fake_controller import FakeDobotController


@pytest.fixture(params=[True, False], ids=["old firmware", "new firmware"])
def setup(request):
    ctrl = FakeDobotController()
    ctrl.legacy = request.param
    ctrl.start()
    d = DobotDriver(dobot_cfg(ctrl))
    d.connect()
    assert wait_for(lambda: d.read_state().link_ok)
    d.enable()
    assert wait_for(lambda: d.read_state().mode is Mode.IDLE)
    d.set_speed(100)
    yield ctrl, d, request.param
    d.disconnect()
    ctrl.stop()


def moves(ctrl):
    return [c for c in ctrl.commands if c.split("(")[0] in ("MovJ", "MovL", "JointMovJ", "MoveJog")]


def test_joint_move_reaches_target(setup):
    ctrl, d, legacy = setup
    d.move_to(Joints((10, 0, 90, 0, -90, 0)), "joint")
    assert wait_for(lambda: d.read_state().mode is Mode.IDLE and d.read_state().joints.values[0] > 9.99, timeout=3)
    sent = moves(ctrl)
    if legacy:
        assert sent[0].startswith("MovJ(joint=") and sent[1].startswith("JointMovJ(10.000000")
        assert d.legacy_syntax is True
    else:
        assert sent == [sent[0]] and sent[0].startswith("MovJ(joint=") and d.legacy_syntax is False


def test_dialect_is_remembered(setup):
    ctrl, d, legacy = setup
    p = d.read_state().tcp
    d.move_to(p.with_axis("Z", p.z + 5), "linear")
    assert wait_for(lambda: d.read_state().mode is Mode.IDLE, timeout=3)
    before = len(moves(ctrl))
    d.move_to(p, "linear")
    after = moves(ctrl)[before:]
    assert len(after) == 1  # no second try once the dialect is known
    assert after[0].startswith("MovL(-" if legacy else "MovL(pose=")


def test_hold_jog_both_dialects(setup):
    ctrl, d, legacy = setup
    d.jog_hold("X", 1, "base")
    assert wait_for(lambda: d.read_state().mode is Mode.JOGGING)
    d.jog_release()
    jogs = [c for c in ctrl.commands if c.startswith("MoveJog(X")]
    assert jogs[-1] == ("MoveJog(X+)" if legacy else "MoveJog(X+,coordtype=1)")


def test_joint_and_cartesian_jogs_learn_their_own_form(setup):
    # Lab CR5: joints took MoveJog(J1+); the Cartesian jog then reused that form's index and sent the refused
    # MoveJog(Y+,1) forever.
    ctrl, d, legacy = setup
    d.jog_hold("J1", 1, "base")
    assert wait_for(lambda: d.read_state().mode is Mode.JOGGING)
    d.jog_release()
    d.jog_hold("Y", 1, "base")
    assert wait_for(lambda: d.read_state().mode is Mode.JOGGING)
    d.jog_release()
    assert wait_for(lambda: d.read_state().mode is Mode.IDLE)
    before = len(ctrl.commands)
    d.jog_hold("Y", -1, "base")  # remembered: one command, no probing
    d.jog_release()
    assert ctrl.commands[before:] == ["MoveJog(Y-)" if legacy else "MoveJog(Y-,coordtype=1)", "MoveJog()"]


def test_refused_jog_leaves_the_link_in_sync(setup):
    _ctrl, d, legacy = setup
    if not legacy:
        pytest.skip("only the old firmware refuses a tool-frame jog in the fake")
    with pytest.raises(DriverError) as e:
        d.jog_hold("Z", 1, "tool")  # every tool form refused, the last with the split reply
    assert e.value.code == -40001
    assert d.read_state().mode is Mode.IDLE
    d.jog_release()  # must read its own reply, not the stray echo
    d.move_to(Joints((5, 0, 90, 0, -90, 0)), "joint")
    assert wait_for(lambda: d.read_state().joints.values[0] > 4.99, timeout=3)


def test_other_refusals_are_not_retried(setup):
    ctrl, d, _ = setup
    d.move_to(Joints((0, 0, 90, 0, -90, 0)), "joint")  # learn the dialect first
    assert wait_for(lambda: d.read_state().mode is Mode.IDLE, timeout=3)
    ctrl.inject_alarm(22)
    assert wait_for(lambda: d.read_state().mode is Mode.ERROR)
    before = len(moves(ctrl))
    with pytest.raises(DriverError) as e:
        d.move_to(Joints((0, 0, 90, 0, -90, 0)), "joint")
    assert e.value.code == -2 and len(moves(ctrl)) - before == 1  # one try only


def test_doctor_reports_the_syntax(setup):
    _ctrl, d, legacy = setup
    d.move_to(Joints((0, 0, 90, 0, -90, 0)), "joint")
    line = next(c for c in d.diagnose() if c.name == "Command syntax")
    assert ("older" in line.value) == legacy


def test_stop_reaches_the_controller_in_both_dialects(setup):
    ctrl, d, legacy = setup
    p = d.read_state().tcp
    d.set_speed(1)
    d.move_to(p.with_axis("Z", p.z + 100), "linear")
    assert wait_for(lambda: d.read_state().mode is Mode.MOVING)
    d.stop()
    assert wait_for(lambda: d.read_state().mode is Mode.IDLE, timeout=1.0)  # the queue really was cleared
    assert ("ResetRobot()" in ctrl.commands) == legacy
    before = len(ctrl.commands)
    d.stop()  # the working stop form is remembered: no unknown-command round trip
    assert ctrl.commands[before:] == ["MoveJog()", "ResetRobot()" if legacy else "Stop()"]


def test_servo_p_falls_back_to_pose_only(setup):
    ctrl, d, legacy = setup
    p = d.read_state().tcp
    d.stream_target(p.with_axis("Y", p.y + 50))  # long enough to see it moving at 100 % speed
    sent = [c for c in ctrl.commands if c.startswith("ServoP(")]
    assert ("t=" not in sent[-1]) == legacy
    assert wait_for(lambda: d.read_state().mode is Mode.STREAMING, timeout=0.5)


def test_stop_without_any_known_command_fails_loudly():
    from botswain.drivers.dobot import fake_controller as fc

    ctrl = FakeDobotController()
    ctrl.legacy = True
    ctrl.start()
    original = ctrl._do
    ctrl._do = lambda name, pos, kw: (-10000, "") if name == "resetrobot" else original(name, pos, kw)
    try:
        d = DobotDriver(dobot_cfg(ctrl))
        d.connect()
        with pytest.raises(DriverError) as e:
            d.stop()
        assert "e-stop" in e.value.hint
        d.disconnect()
    finally:
        ctrl.stop()
    assert fc  # module imported for the patch above


def test_point_with_negative_zero_joint_moves(setup):
    # Lab CR5: go to point P1 (J1 saved as -0.0) was refused every time with -30001.
    _ctrl, d, _ = setup
    d.move_to(Joints((-0.0, -0.41, 91.69, -6.8e-06, -90.6, 0.82)), "joint")
    d.move_to(Joints((-0.0, 0.0, 90.0, 0.0, -90.0, 0.0)), "joint")  # second time: the remembered form
    assert wait_for(lambda: abs(d.read_state().joints.values[2] - 90.0) < 0.01, timeout=3)
