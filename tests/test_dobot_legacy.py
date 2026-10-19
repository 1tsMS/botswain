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
    assert jogs[-1] == ("MoveJog(X+,CoordType=1,User=0,Tool=0)" if legacy else "MoveJog(X+,coordtype=1)")


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
