"""Streaming (teleop) rules of the SafetyGate."""

import itertools

import pytest
from helpers import ROBOTS, make_state

from botswain.control.safety import Allowed, Rejected, SafetyGate, clamp_step
from botswain.core.clock import now_ns
from botswain.core.config import load_robot_config
from botswain.core.types import Mode, Pose, StreamTarget

START = Pose(-482, -136, 484, 180, 0, 0)


@pytest.fixture
def gate():
    cfg = load_robot_config(ROBOTS / "dobot_cr5.yaml")  # 150 mm/s * 0.03 s = 4.5 mm per tick, teleport 80 mm
    g = SafetyGate(cfg.limits, cfg.stream.period_s)
    g.reset_stream(START)
    return g


def target(pose, hold=True, age_ms=0.0):
    return StreamTarget(pose, now_ns() - int(age_ms * 1e6), "test", hold)


def state():
    return make_state(Mode.STREAMING, tcp=START)


def test_fast_move_is_clamped_not_dropped(gate):
    goal = START.with_axis("X", START.x + 50)
    xs = []
    for _ in range(12):
        v = gate.check(target(goal), state())
        assert isinstance(v, Allowed)
        xs.append(v.cmd.pose.x)
    assert xs[0] - START.x == pytest.approx(4.5)
    assert v.clamped == () and xs[-1] == pytest.approx(goal.x)  # kept advancing until it got there
    assert all(b >= a for a, b in itertools.pairwise(xs))


def test_teleport_rejected_and_reanchors(gate):
    assert isinstance(gate.check(target(START.with_axis("Y", START.y + 2)), state()), Allowed)
    v = gate.check(target(START.with_axis("Y", START.y + 200)), state())
    assert isinstance(v, Rejected) and v.rule == "teleport"
    # after the glitch the next sample is a fresh reference again (the mapping re-anchors)
    assert isinstance(gate.check(target(START.with_axis("Y", START.y + 1)), state()), Allowed)


def test_stale_sample_rejected(gate):
    v = gate.check(target(START, age_ms=500), state())
    assert v.rule == "stale"


def test_release_hold_rejected(gate):
    assert gate.check(target(START, hold=False), state()).rule == "hold"


def test_first_target_seeded_no_jump(gate):
    gate.reset_stream(None)
    measured = START.with_axis("Z", 400)
    v = gate.check(target(START.with_axis("Z", 600)), make_state(Mode.IDLE, tcp=measured))
    assert v.cmd.pose.z == pytest.approx(400 + 4.5)  # from the measured pose, not from the phone


def test_stream_clamped_to_box(gate):
    near = START.with_axis("X", -334.0)
    gate.reset_stream(near)
    v = gate.check(target(near.with_axis("X", -330.0)), make_state(Mode.STREAMING, tcp=near))
    assert v.cmd.pose.x == -332.0 and "box" in v.clamped


def test_rotation_step_clamped_the_short_way_round():
    last = Pose(0, 0, 0, 179.0, 0, 0)
    pose, was = clamp_step(last, Pose(0, 0, 0, -179.0, 0, 0), 10.0, 2.0)
    assert was is False and pose.rx == -179.0  # 2 deg across the +-180 seam, not 358 the long way
    pose, was = clamp_step(last, Pose(0, 0, 0, -170.0, 0, 0), 10.0, 2.0)
    assert was and pose.rx == pytest.approx(181.0)
