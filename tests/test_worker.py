import time

import pytest
from helpers import ROBOTS, SpyDriver, wait_for

from botswain.control.safety import SafetyGate
from botswain.control.worker import RobotWorker
from botswain.core.config import load_robot_config
from botswain.core.types import (
    Connect,
    Disconnect,
    Enable,
    JogHold,
    JogRelease,
    JogStep,
    JogTick,
    Mode,
    MoveTo,
    SetSpeed,
)
from botswain.drivers.base import DriverError


@pytest.fixture
def cfg():
    return load_robot_config(ROBOTS / "fake.yaml")


@pytest.fixture
def rig(cfg):
    driver = SpyDriver(cfg, lin_speed_mm_s=1000.0, joint_speed_deg_s=1000.0)
    worker = RobotWorker(driver, SafetyGate(cfg.limits, cfg.stream.period_s))
    logs: list[tuple[str, str]] = []
    worker.on_log = lambda level, text: logs.append((level, text))
    yield driver, worker, logs
    worker.shutdown()


def ready(worker):
    worker.start()
    worker.submit(Connect())
    worker.submit(Enable())
    assert wait_for(lambda: worker.latest_state().mode is Mode.IDLE)


def test_worker_enable_and_jog(rig):
    driver, worker, _ = rig
    ready(worker)
    x0 = worker.latest_state().tcp.x
    worker.submit(JogStep("X", 1.0))
    assert wait_for(lambda: abs(worker.latest_state().tcp.x - (x0 + 1.0)) < 1e-4)
    assert "jog_step" in driver.names()


def test_connect_never_enables(rig):
    driver, worker, _ = rig
    worker.start()
    worker.submit(Connect())
    assert wait_for(lambda: worker.latest_state().mode is Mode.DISABLED)
    time.sleep(0.05)
    assert "enable" not in driver.names()


def test_stop_clears_queue(rig):
    driver, worker, logs = rig
    # Queue everything before the thread runs: STOP is checked first, so none of it may execute.
    worker.submit(Connect())
    worker.submit(Enable())
    for i in range(5):
        worker.submit(MoveTo(driver.read_state().tcp.with_axis("X", -400 - i), "linear"))
    worker.request_stop()
    worker.start()
    assert wait_for(lambda: "stop" in driver.names())
    time.sleep(0.05)
    assert worker.pending == 0
    assert not {"connect", "enable", "move_to"} & set(driver.names())
    assert any("7 queued" in text for _, text in logs)


def test_stop_mid_move(rig):
    _, worker, _ = rig
    ready(worker)
    worker.submit(SetSpeed(1))  # 10 mm/s on this fake: a 100 mm move takes 10 s
    p = worker.latest_state().tcp
    worker.submit(MoveTo(p.with_axis("X", p.x + 100), "linear"))
    assert wait_for(lambda: worker.latest_state().mode is Mode.MOVING)
    t0 = time.perf_counter()
    worker.request_stop()
    assert wait_for(lambda: worker.latest_state().mode is not Mode.MOVING, timeout=0.1)
    assert time.perf_counter() - t0 < 0.1


def test_hold_jog_watchdog_releases(rig):
    driver, worker, logs = rig
    ready(worker)
    worker.submit(JogHold("X", 1))
    assert wait_for(lambda: "jog_release" in driver.names(), timeout=1.0)
    dt_ms = (driver.call_time("jog_release") - driver.call_time("jog_hold")) / 1e6
    assert 150 <= dt_ms <= 250
    assert any("watchdog" in text for _, text in logs)
    assert wait_for(lambda: worker.latest_state().mode is Mode.IDLE)


def test_hold_jog_ticks_keep_it_alive(rig):
    driver, worker, _ = rig
    ready(worker)
    worker.submit(JogHold("Z", -1))
    for _ in range(8):  # 400 ms of "still held"
        time.sleep(0.05)
        worker.submit(JogTick())
    assert "jog_release" not in driver.names()
    assert worker.latest_state().mode is Mode.JOGGING
    worker.submit(JogRelease())
    assert wait_for(lambda: "jog_release" in driver.names())


def test_driver_exception_logged_loop_survives(rig):
    driver, worker, logs = rig
    ready(worker)
    driver.fail_next["jog_step"] = DriverError(-1, "boom", "try again")
    x0 = worker.latest_state().tcp.x
    worker.submit(JogStep("X", 1.0))
    assert wait_for(lambda: any(level == "error" and "boom" in text for level, text in logs))
    assert any("try again" in text for _, text in logs)
    worker.submit(JogStep("X", 1.0))
    assert wait_for(lambda: abs(worker.latest_state().tcp.x - (x0 + 1.0)) < 1e-4)


def test_driver_bug_does_not_kill_worker(rig):
    driver, worker, logs = rig
    ready(worker)
    driver.fail_next["set_speed"] = ZeroDivisionError("oops")
    worker.submit(SetSpeed(10))
    assert wait_for(lambda: any("crashed" in text for _, text in logs))
    worker.submit(SetSpeed(10))
    assert wait_for(lambda: worker.latest_state().speed_pct == 10)
    assert worker.running


def test_state_published_at_most_30hz(rig):
    _, worker, _ = rig
    count = []
    worker.on_state = lambda s: count.append(s)
    worker.start()
    time.sleep(1.0)
    assert 25 <= len(count) <= 31


def test_last_command_records_raw_and_sent(rig):
    _, worker, _ = rig
    ready(worker)
    worker.submit(SetSpeed(150))
    assert wait_for(lambda: worker.last_command is not None and worker.last_command.raw == SetSpeed(150))
    rec = worker.last_command
    assert rec.sent == SetSpeed(100) and rec.rule == "speed_cap"


def test_rejected_command_recorded_and_not_sent(rig):
    driver, worker, logs = rig
    worker.start()
    worker.submit(Connect())
    worker.submit(JogStep("X", 1.0))  # still disabled
    assert wait_for(lambda: worker.last_command is not None and worker.last_command.rule == "ready")
    assert worker.last_command.sent is None
    assert "jog_step" not in driver.names()
    assert any("not enabled" in text for _, text in logs)


def test_disconnect_sends_stop_first(rig):
    driver, worker, _ = rig
    ready(worker)
    worker.submit(JogHold("X", 1))
    assert wait_for(lambda: worker.latest_state().mode is Mode.JOGGING)
    worker.submit(Disconnect())
    assert wait_for(lambda: "disconnect" in driver.names())
    names = driver.names()
    assert names.index("stop") < names.index("disconnect")


def test_shutdown_stops_then_disconnects(rig):
    driver, worker, _ = rig
    ready(worker)
    worker.shutdown()
    names = driver.names()
    assert names[names.index("enable") + 1:][:2] == ["stop", "disconnect"]
    assert not worker.running
