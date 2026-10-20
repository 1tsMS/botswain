"""Dobot V3 (controller 3.5.x, the lab CR5) and V4 (4.x) command sets, each against a fake controller of the
same version. The commands are explicit per version: nothing is guessed, nothing falls back."""

import pytest
from helpers import dobot_cfg, wait_for

from botswain.core.types import Joints, Mode
from botswain.drivers.base import DriverError
from botswain.drivers.dobot.driver import DobotV3Driver, DobotV4Driver
from botswain.drivers.dobot.fake_controller import FakeDobotController


@pytest.fixture(params=["v3", "v4"])
def setup(request):
    ctrl = FakeDobotController()
    ctrl.v3 = request.param == "v3"
    ctrl.start()
    d = (DobotV3Driver if ctrl.v3 else DobotV4Driver)(dobot_cfg(ctrl))
    d.connect()
    assert wait_for(lambda: d.read_state().link_ok)
    d.enable()
    assert wait_for(lambda: d.read_state().mode is Mode.IDLE)
    d.set_speed(100)
    yield ctrl, d, ctrl.v3
    d.disconnect()
    ctrl.stop()


def sent_since(ctrl, n):
    return ctrl.commands[n:]


def test_joint_move_uses_the_versions_own_form(setup):
    ctrl, d, v3 = setup
    n = len(ctrl.commands)
    d.move_to(Joints((10, 0, 90, 0, -90, 0)), "joint")
    assert wait_for(lambda: d.read_state().mode is Mode.IDLE and d.read_state().joints.values[0] > 9.99, timeout=3)
    first = sent_since(ctrl, n)[0]
    assert first.startswith("JointMovJ(10.000000" if v3 else "MovJ(joint={10.000000")


def test_hold_jog_reads_as_jogging_not_collision(setup):
    # Lab CR5: V3 reports jogging as mode 11, which the V4 table calls collision. Botswain then released every
    # hold-jog after one frame.
    ctrl, d, v3 = setup
    for axis in ("J1", "X"):
        d.jog_hold(axis, 1, "base")
        assert wait_for(lambda: d.read_state().mode is Mode.JOGGING)
        assert d.read_state().raw_mode == (11 if v3 else 8)
        d.jog_release()
        assert wait_for(lambda: d.read_state().mode is Mode.IDLE)
    jogs = [c for c in ctrl.commands if c.startswith("MoveJog(") and c != "MoveJog()"]
    assert jogs == (["MoveJog(J1+)", "MoveJog(X+,CoordType=0,User=0,Tool=0)"] if v3
                    else ["MoveJog(J1+,coordtype=0)", "MoveJog(X+,coordtype=1)"])


def test_v3_mode_11_is_never_a_collision():
    from botswain.drivers.dobot.dialects import V3, V4

    assert V3.modes[11] is Mode.JOGGING and V4.modes[11] is Mode.COLLISION
    assert V3.modes[8] is Mode.DRAG and V4.modes[8] is Mode.JOGGING


def test_tool_frame_hold_jog_is_off_on_v3_until_tested(setup):
    ctrl, d, v3 = setup
    n = len(ctrl.commands)
    if v3:
        with pytest.raises(DriverError, match="not verified"):
            d.jog_hold("Z", 1, "tool")
        assert sent_since(ctrl, n) == []  # nothing reached the arm
    else:
        d.jog_hold("Z", 1, "tool")
        assert sent_since(ctrl, n) == ["MoveJog(Z+,coordtype=2)"]
        d.jog_release()


def test_stop_sends_the_versions_stop(setup):
    ctrl, d, v3 = setup
    n = len(ctrl.commands)
    d.stop()
    assert sent_since(ctrl, n) == ["MoveJog()", "ResetRobot()" if v3 else "Stop()"]


def test_stop_ends_drag_first(setup):
    ctrl, d, _ = setup
    d.drag(True)
    assert wait_for(lambda: d.read_state().mode is Mode.DRAG)
    n = len(ctrl.commands)
    d.stop()
    assert sent_since(ctrl, n)[0] == "StopDrag()"
    assert wait_for(lambda: d.read_state().mode is Mode.IDLE)


def test_servo_p_form(setup):
    ctrl, d, v3 = setup
    p = d.read_state().tcp
    d.stream_target(p.with_axis("Y", p.y + 50))
    sent = [c for c in ctrl.commands if c.startswith("ServoP(")][-1]
    assert ("t=" not in sent) == v3
    assert wait_for(lambda: d.read_state().mode is Mode.STREAMING, timeout=0.5)


def test_point_with_negative_zero_joint_moves(setup):
    # Lab CR5: go to point P1 (J1 saved as -0.0) was refused every time with -30001.
    _ctrl, d, _ = setup
    d.move_to(Joints((-0.0, -0.41, 91.69, -6.8e-06, -90.6, 0.82)), "joint")
    assert wait_for(lambda: abs(d.read_state().joints.values[2] - 91.69) < 0.01, timeout=3)


def test_take_control_only_on_v4(setup):
    _ctrl, d, v3 = setup
    assert d.capabilities.request_control is not v3
    if v3:
        with pytest.raises(DriverError, match="no RequestControl"):
            d.request_control()


def test_recover_reconnects_a_silent_controller(setup):
    # Lab CR5 09:25: after pressing into the table the dashboard stopped answering; only DobotStudio's mode
    # switch freed it. Recover must reconnect, stop and clear, and say so.
    ctrl, d, _ = setup
    d._dash.timeout_s = 0.3
    ctrl.silent = True
    with pytest.raises(DriverError):
        d.clear_error()
    assert "not answering" in d.read_state().fault
    steps = d.recover()
    assert steps[0] == "dashboard 29999 reconnected" and "ClearError() accepted" in steps
    assert d.read_state().fault == ""
    d.move_to(Joints((5, 0, 90, 0, -90, 0)), "joint")  # it works again
    assert wait_for(lambda: d.read_state().joints.values[0] > 4.99, timeout=3)


def test_recover_clears_an_alarm(setup):
    ctrl, d, _ = setup
    ctrl.inject_alarm(22)
    assert wait_for(lambda: d.read_state().mode is Mode.ERROR)
    with pytest.raises(DriverError):
        d.move_to(Joints((5, 0, 90, 0, -90, 0)), "joint")
    assert d.read_state().fault.startswith("alarm: 22")
    assert "alarms now: none" in d.recover()
    assert wait_for(lambda: d.read_state().mode in (Mode.IDLE, Mode.DISABLED))


def test_doctor_names_the_protocol(setup):
    _ctrl, d, v3 = setup
    line = next(c for c in d.diagnose() if c.name == "Protocol")
    assert ("V3" in line.value) == v3
