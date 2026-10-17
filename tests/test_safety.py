import pytest
from helpers import ROBOTS, make_state

from botswain.control.safety import Allowed, Rejected, SafetyGate
from botswain.core.clock import now_ns
from botswain.core.config import load_robot_config
from botswain.core.types import (
    ClearError,
    Connect,
    Disable,
    Disconnect,
    Enable,
    Gripper,
    JogHold,
    JogRelease,
    JogStep,
    JogTick,
    Mode,
    MoveTo,
    SetSpeed,
    Stop,
    StreamTarget,
)


@pytest.fixture
def gate():
    cfg = load_robot_config(ROBOTS / "dobot_cr5.yaml")
    return SafetyGate(cfg.limits, cfg.stream.period_s)


@pytest.mark.parametrize("mode", [Mode.DISABLED, Mode.ERROR, Mode.COLLISION, Mode.MOVING, Mode.JOGGING,
                                  Mode.DISCONNECTED, Mode.POWERED_OFF, Mode.UNKNOWN])
def test_motion_rejected_when_not_idle(gate, mode):
    v = gate.check(JogStep("X", 1.0), make_state(mode))
    assert isinstance(v, Rejected) and v.rule == "ready"


def test_motion_rejected_when_disabled(gate):
    v = gate.check(MoveTo(make_state().tcp, "linear"), make_state(Mode.DISABLED))
    assert v == Rejected("ready", "robot not enabled")


def test_motion_rejected_without_live_feedback(gate):
    v = gate.check(JogHold("X", 1), make_state(Mode.IDLE, link_ok=False))
    assert isinstance(v, Rejected) and "feedback" in v.reason


def test_motion_allowed_when_idle(gate):
    cmd = JogStep("J1", 1.0)
    assert gate.check(cmd, make_state(Mode.IDLE)) == Allowed(cmd)


def test_set_speed_clamped_to_cap(gate):
    assert gate.check(SetSpeed(80), make_state()) == Allowed(SetSpeed(20), ("speed_cap",))


def test_set_speed_inside_cap_unchanged(gate):
    assert gate.check(SetSpeed(5), make_state()) == Allowed(SetSpeed(5))
    assert gate.check(SetSpeed(0), make_state()) == Allowed(SetSpeed(1), ("speed_min",))


@pytest.mark.parametrize("cmd", [Enable(), Disable(), ClearError(), Stop(), JogTick(), JogRelease(), Connect(),
                                 Disconnect()])
def test_non_motion_always_allowed(gate, cmd):
    for mode in (Mode.DISCONNECTED, Mode.ERROR, Mode.MOVING):
        assert gate.check(cmd, make_state(mode, link_ok=False)) == Allowed(cmd)


def test_gripper_needs_link_not_enable(gate):
    assert isinstance(gate.check(Gripper(True), make_state(Mode.DISABLED)), Allowed)
    assert isinstance(gate.check(Gripper(True), make_state(Mode.DISCONNECTED, link_ok=False)), Rejected)


def test_stream_allowed_while_streaming(gate):
    cmd = StreamTarget(make_state().tcp, now_ns(), "test", True)
    assert isinstance(gate.check(cmd, make_state(Mode.STREAMING)), Allowed)


def test_unknown_command_rejected(gate):
    v = gate.check("MoveSomewhere", make_state())  # type: ignore[arg-type]
    assert isinstance(v, Rejected) and v.rule == "unknown"


def test_counts_track_rejects_and_clamps(gate):
    gate.check(JogStep("X", 1.0), make_state(Mode.DISABLED))
    gate.check(JogStep("X", 1.0), make_state(Mode.DISABLED))
    gate.check(SetSpeed(99), make_state())
    assert gate.counts == {"ready": 2, "speed_cap": 1}
