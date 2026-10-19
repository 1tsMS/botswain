"""Gamepad teleop: deadzone, deadman, rate integration, lead cap, buttons, and the fake robot following."""

import time

import pytest
from helpers import ROBOTS, wait_for

from botswain.control.safety import SafetyGate
from botswain.control.worker import RobotWorker
from botswain.core.config import load_robot_config
from botswain.core.types import Connect, Enable, Gripper, Mode
from botswain.drivers.fake import FakeDriver
from botswain.teleop.gamepad import MAX_LEAD_MM, GamepadTeleop, PadState, _stick, _trigger


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


def pad(*buttons, **axes):
    return PadState(True, buttons=frozenset(buttons), **axes)


def test_deadzones():
    assert _stick(3000, -2000) == (0.0, 0.0)  # cheap-pad drift is ignored
    x, y = _stick(32767, 0)
    assert x == pytest.approx(1.0) and y == 0.0
    assert _trigger(10) == 0.0 and _trigger(255) == pytest.approx(1.0)


def test_nothing_moves_without_the_deadman(worker):
    t = GamepadTeleop(worker, lambda: PadState(False))
    assert t.step(pad(ly=1.0), 0.02) is None
    assert not t.active


def test_deadman_starts_at_the_measured_pose_then_integrates(worker):
    t = GamepadTeleop(worker, lambda: PadState(False))
    t.speed_scale = 1.0
    start = worker.latest_state().tcp
    first = t.step(pad("lb", ly=1.0), 0.02)
    assert first.hold and first.pose.as_list() == pytest.approx(start.as_list())
    nxt = t.step(pad("lb", ly=1.0), 0.02)  # full stick forward: 100 mm/s for 20 ms
    assert nxt.pose.x - start.x == pytest.approx(2.0, abs=1e-6)
    slow = t.step(pad("lb", "rb", ly=1.0), 0.02)  # RB: quarter speed
    assert slow.pose.x - nxt.pose.x == pytest.approx(0.5, abs=1e-6)
    up = t.step(pad("lb", rt=1.0), 0.02)
    assert up.pose.z - slow.pose.z == pytest.approx(2.0, abs=1e-6)


def test_release_stops_and_lead_is_capped(worker):
    t = GamepadTeleop(worker, lambda: PadState(False))
    t.speed_scale = 1.0
    start = worker.latest_state().tcp
    t.step(pad("lb"), 0.02)
    far = t.step(pad("lb", ly=1.0), 10.0)  # a huge dt would mean 1 m; the target may lead by 30 mm only
    assert far.pose.x - start.x == pytest.approx(MAX_LEAD_MM, abs=1e-6)
    release = t.step(pad(ly=1.0), 0.02)
    assert release is not None and not release.hold and not t.active


def test_buttons(worker):
    t = GamepadTeleop(worker, lambda: PadState(False))
    assert t.speed_scale == pytest.approx(0.1)
    actions = []
    t.on_action = actions.append
    t.step(pad("a"), 0.02)  # unknown state: opens first
    assert wait_for(lambda: worker.latest_state().gripper == 1.0)
    t.step(pad("a"), 0.02)  # still held: no second toggle
    t.step(pad(), 0.02)
    t.step(pad("a"), 0.02)  # next press closes
    assert wait_for(lambda: worker.latest_state().gripper == 0.0)
    t.step(pad("b"), 0.02)  # B no longer does anything
    t.step(pad(), 0.02)
    for name in ("up", "up", "down"):
        t.step(pad(name), 0.02)
        t.step(pad(), 0.02)
    assert t.speed_scale == pytest.approx(0.2)  # starts at 0.1 (10 %)
    for name in ("start", "back", "x"):
        t.step(pad(name), 0.02)
        t.step(pad(name), 0.02)  # held: only one action per press
        t.step(pad(), 0.02)
    assert actions == ["start", "back", "x"]
    assert worker.last_command.raw == Gripper(False)


def test_fake_robot_follows_the_stick(worker):
    state = {"pad": PadState(False)}
    t = GamepadTeleop(worker, lambda: state["pad"])
    t.speed_scale = 1.0
    t.start()
    try:
        x0 = worker.latest_state().tcp.x
        state["pad"] = pad("lb", ly=1.0)
        assert wait_for(lambda: worker.latest_state().tcp.x > x0 + 20, timeout=2.0)
        assert worker.latest_state().mode is Mode.STREAMING
        state["pad"] = pad()  # let go of LB
        assert wait_for(lambda: worker.latest_state().mode is Mode.IDLE, timeout=1.0)
        x1 = worker.latest_state().tcp.x
        time.sleep(0.2)
        assert worker.latest_state().tcp.x == pytest.approx(x1, abs=0.5)
    finally:
        t.stop()


def test_unplugged_pad_releases(worker):
    t = GamepadTeleop(worker, lambda: PadState(False))
    t.step(pad("lb"), 0.02)
    out = t.step(PadState(False), 0.02)
    assert out is not None and not out.hold
