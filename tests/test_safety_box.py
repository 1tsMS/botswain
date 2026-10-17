"""Workspace box and joint-limit rules of the SafetyGate (T4)."""

import pytest
from helpers import ROBOTS, make_state

from botswain.control.safety import Allowed, SafetyGate
from botswain.core.config import load_robot_config
from botswain.core.types import JogHold, JogStep, Joints, Mode, MoveTo, Pose


@pytest.fixture
def gate():
    cfg = load_robot_config(ROBOTS / "dobot_cr5.yaml")  # box x [-632, -332], y [-286, 14], z [334, 634]
    g = SafetyGate(cfg.limits, cfg.stream.period_s)
    g.box_enabled = True
    return g


def at(x=-482.0, y=-136.0, z=484.0, rpy=(180.0, 0.0, 0.0), joints=(0, 0, 90, 0, -90, 0)):
    return make_state(Mode.IDLE, tcp=Pose(x, y, z, *rpy), joints=joints)


def test_jog_step_clamped_at_box_wall(gate):
    v = gate.check(JogStep("X", 5.0), at(x=-334.0))
    assert v.clamped == ("box",) and v.cmd.delta == pytest.approx(2.0)


def test_jog_step_at_wall_rejected_but_away_allowed(gate):
    assert gate.check(JogStep("X", 1.0), at(x=-332.0)).rule == "box"
    assert gate.check(JogStep("X", -1.0), at(x=-332.0)) == Allowed(JogStep("X", -1.0))


def test_tool_frame_step_is_rotated_before_clamping(gate):
    # Rx = 180: tool Z points down, so tool Z+ moves base Z-. The TCP is 1 mm above the box floor.
    v = gate.check(JogStep("Z", 5.0, "tool"), at(z=335.0))
    assert v.clamped == ("box",) and v.cmd.delta == pytest.approx(1.0)


def test_rotation_steps_are_not_box_limited(gate):
    assert gate.check(JogStep("Rz", 10.0), at(x=-332.0)) == Allowed(JogStep("Rz", 10.0))


def test_outside_box_only_moves_back_in(gate):
    assert isinstance(gate.check(JogStep("X", -1.0), at(x=-320.0)), Allowed)
    assert gate.check(JogStep("X", 1.0), at(x=-320.0)).rule == "box"


def test_move_to_outside_box_rejected(gate):
    v = gate.check(MoveTo(Pose(-482, -136, 700, 180, 0, 0), "linear"), at())
    assert v.rule == "box" and "Z 700.0" in v.reason
    assert isinstance(gate.check(MoveTo(Pose(-482, -136, 500, 180, 0, 0), "linear"), at()), Allowed)


def test_joint_jog_clamped_at_limit(gate):
    v = gate.check(JogStep("J3", 5.0), at(joints=(0, 0, 158, 0, -90, 0)))
    assert v.clamped == ("joint_limit",) and v.cmd.delta == pytest.approx(2.0)
    assert gate.check(JogStep("J3", 1.0), at(joints=(0, 0, 160, 0, -90, 0))).rule == "joint_limit"


def test_joint_move_outside_rejected(gate):
    v = gate.check(MoveTo(Joints((0, 0, 170, 0, -90, 0)), "joint"), at())
    assert v.rule == "joint_limit" and "J3" in v.reason


def test_hold_jog_refused_at_wall(gate):
    assert gate.check(JogHold("Y", 1), at(y=14.0)).rule == "box"
    assert isinstance(gate.check(JogHold("Y", -1), at(y=14.0)), Allowed)
    assert gate.check(JogHold("J3", 1), at(joints=(0, 0, 160, 0, -90, 0))).rule == "joint_limit"


def test_hold_violation_uses_margin_while_running(gate):
    assert gate.hold_violation(JogHold("X", 1), at(x=-332.5)) is not None  # within 1 mm of the wall
    assert gate.hold_violation(JogHold("X", 1), at(x=-340.0)) is None
    assert gate.hold_violation(JogHold("X", 1, "tool"), at(x=-300.0))[0] == "box"


def test_box_off_lets_everything_through(gate):
    gate.box_enabled = False
    assert gate.check(JogStep("X", 5.0), at(x=-332.0)) == Allowed(JogStep("X", 5.0))
    assert isinstance(gate.check(MoveTo(Pose(-482, -136, 700, 180, 0, 0), "linear"), at()), Allowed)
    assert gate.check(JogStep("J3", 5.0), at(joints=(0, 0, 160, 0, -90, 0))).rule == "joint_limit"  # always on
