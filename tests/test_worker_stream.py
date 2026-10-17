"""Teleop streaming through the worker: newest wins, release stops, silence stops, STOP latches."""

import time

import pytest
from helpers import ROBOTS, SpyDriver, wait_for

from botswain.control.safety import SafetyGate
from botswain.control.worker import RobotWorker
from botswain.core.clock import now_ns
from botswain.core.config import load_robot_config
from botswain.core.types import Connect, Enable, Mode, StreamTarget
from botswain.teleop.mapping import TeleopMapper
from botswain.teleop.phone import PhoneSample
from botswain.teleop.session import TeleopSession


@pytest.fixture
def rig():
    cfg = load_robot_config(ROBOTS / "fake.yaml")
    driver = SpyDriver(cfg, lin_speed_mm_s=1000.0)
    worker = RobotWorker(driver, SafetyGate(cfg.limits, cfg.stream.period_s))
    logs = []
    worker.on_log = lambda level, text: logs.append(text)
    worker.start()
    worker.submit(Connect())
    worker.submit(Enable())
    assert wait_for(lambda: worker.latest_state().mode is Mode.IDLE)
    yield driver, worker, logs
    worker.shutdown()


def stream(worker, dx, hold=True):
    p = worker.latest_state().tcp
    worker.stream(StreamTarget(p.with_axis("X", p.x + dx), now_ns(), "test", hold))


def count(driver, name):
    return driver.names().count(name)


def test_newest_wins_one_target_per_period(rig):
    driver, worker, _ = rig
    for _ in range(20):  # a burst far faster than the 30 ms stream period
        stream(worker, 1.0)
    time.sleep(0.1)
    assert 1 <= count(driver, "stream_target") <= 2
    assert worker.latest_state().mode is Mode.STREAMING


def test_release_stops_stream(rig):
    driver, worker, _ = rig
    stream(worker, 1.0)
    assert wait_for(lambda: worker.latest_state().mode is Mode.STREAMING)
    stream(worker, 1.0, hold=False)
    assert wait_for(lambda: worker.latest_state().mode is Mode.IDLE)
    assert "stop" in driver.names()


def test_silent_input_stops_stream(rig):
    _driver, worker, logs = rig
    stream(worker, 1.0)
    assert wait_for(lambda: worker.latest_state().mode is Mode.STREAMING)
    assert wait_for(lambda: worker.latest_state().mode is Mode.IDLE, timeout=0.5)  # stale_ms = 200
    assert any("went silent" in t for t in logs)


def test_stop_latches_until_hold_released(rig):
    driver, worker, _logs = rig
    stream(worker, 1.0)
    assert wait_for(lambda: worker.latest_state().mode is Mode.STREAMING)
    worker.request_stop()
    assert wait_for(lambda: worker.latest_state().mode is Mode.IDLE)
    before = count(driver, "stream_target")
    for _ in range(5):  # the phone keeps sending hold=true after STOP
        stream(worker, 1.0)
        time.sleep(0.04)
    assert count(driver, "stream_target") == before
    stream(worker, 0.0, hold=False)
    time.sleep(0.05)
    stream(worker, 1.0)
    assert wait_for(lambda: count(driver, "stream_target") > before)


def test_session_maps_phone_to_stream(rig):
    _driver, worker, _ = rig
    session = TeleopSession(worker, TeleopMapper(scale_pos=1.0))
    x0 = worker.latest_state().tcp.x

    def sample(z, hold=True):
        return PhoneSample(now_ns(), (0.0, 0.0, z), (1.0, 0.0, 0.0, 0.0), hold, False, 1.0)

    for i in range(10):
        session.on_sample(sample(-0.001 * i))  # 1 mm forward per sample
        time.sleep(0.035)
    assert wait_for(lambda: worker.latest_state().tcp.x > x0 + 5)
    session.on_sample(sample(0.0, hold=False))
    assert wait_for(lambda: worker.latest_state().mode is Mode.IDLE)
