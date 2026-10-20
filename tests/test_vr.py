"""VR teleop: message parsing, clutch + trigger session, one input at a time, the paired server end to end."""

import json
import socket
import ssl
import time
import urllib.request

import pytest
import websocket
from helpers import ROBOTS, SpyDriver, wait_for

from botswain.control.safety import SafetyGate
from botswain.control.worker import RobotWorker
from botswain.core.clock import now_ns
from botswain.core.config import load_robot_config
from botswain.core.types import Connect, Enable, Mode, StreamTarget
from botswain.teleop.mapping import TeleopMapper
from botswain.teleop.vr import VrServer, VrSession, parse_vr

SSL = {"cert_reqs": ssl.CERT_NONE}


def hand(x=0.0, y=1.0, z=0.0, grip=0.0, trigger=0.0, tracked=True):
    if not tracked:
        return {"tracked": False}
    return {"tracked": True, "pos": [x, y, z], "quat": [0, 0, 0, 1], "grip": grip, "trigger": trigger,
            "a": False, "b": False, "stick": [0, 0]}


def msg(right):
    return {"type": "vr", "right": right, "left": {"tracked": False}, "head": {"pos": [0, 1.6, 0], "quat": [0, 0, 0, 1]}}


# ---- parsing ----

def test_parse_normalises_and_rejects_bad_numbers():
    s = parse_vr({**msg(hand(0.1, 1.2, -0.3, grip=1.5)), "right": {**hand(), "quat": [0, 0, 0, 2]}}, 1)
    assert s.right.quat_wxyz == (1.0, 0.0, 0.0, 0.0) and s.left is None and s.head_m == (0.0, 1.6, 0.0)
    assert parse_vr(msg(hand(grip=1.5)), 1).right.grip == 1.0  # clamped
    with pytest.raises(ValueError):
        parse_vr(msg({**hand(), "pos": [float("nan"), 0, 0]}), 1)
    with pytest.raises(ValueError):
        parse_vr(msg({**hand(), "quat": [0, 0, 0, 0]}), 1)
    assert parse_vr(msg(hand(tracked=False)), 1).right is None


# ---- session through a real worker ----

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
    session = VrSession(worker, TeleopMapper(scale_pos=1.0))
    yield driver, worker, session, logs
    worker.shutdown()


def feed(session, right, seconds=0.15):
    end = time.perf_counter() + seconds
    while time.perf_counter() < end:
        session.on_sample(parse_vr(msg(right), now_ns()))
        time.sleep(0.012)  # ~80 Hz like the headset


def test_clutch_moves_and_release_stops(rig):
    _, worker, session, _ = rig
    x0 = worker.latest_state().tcp.x
    feed(session, hand(z=0.0, grip=0.9))  # squeeze: anchor here
    feed(session, hand(z=-0.05, grip=0.9), 0.6)  # hand 5 cm forward (WebXR -z) -> robot +X 50 mm
    assert wait_for(lambda: worker.latest_state().tcp.x > x0 + 40, timeout=2)
    assert worker.commanded.source == "vr"
    feed(session, hand(z=-0.05, grip=0.1))
    assert wait_for(lambda: worker.latest_state().mode is Mode.IDLE)


def test_clutch_hysteresis_does_not_chatter(rig):
    _, _, session, _ = rig
    feed(session, hand(grip=0.5), 0.05)
    assert not session.clutch  # 0.5 is below the 0.6 needed to engage
    feed(session, hand(grip=0.7), 0.05)
    feed(session, hand(grip=0.5), 0.05)
    assert session.clutch  # and above the 0.4 needed to let go


def test_lost_tracking_stops_the_arm(rig):
    _, worker, session, _ = rig
    feed(session, hand(grip=0.9))
    feed(session, hand(z=-0.05, grip=0.9), 0.3)
    assert wait_for(lambda: worker.latest_state().mode is Mode.STREAMING)
    feed(session, hand(tracked=False), 0.1)
    assert not session.clutch
    assert wait_for(lambda: worker.latest_state().mode is Mode.IDLE)


def test_trigger_toggles_the_gripper_once_per_press(rig):
    _, worker, session, _ = rig
    feed(session, hand(trigger=0.9), 0.1)  # held for many frames: one toggle
    assert wait_for(lambda: worker.latest_state().gripper == 1.0)  # unknown -> open first
    feed(session, hand(trigger=0.5), 0.05)  # between the thresholds: still "down"
    feed(session, hand(trigger=0.9), 0.05)
    assert worker.latest_state().gripper == 1.0
    feed(session, hand(trigger=0.0), 0.05)
    feed(session, hand(trigger=0.9), 0.05)
    assert wait_for(lambda: worker.latest_state().gripper == 0.0)


def test_one_input_owns_the_arm(rig):
    _, worker, session, logs = rig
    feed(session, hand(grip=0.9))
    feed(session, hand(z=-0.03, grip=0.9), 0.2)
    assert wait_for(lambda: worker.latest_state().mode is Mode.STREAMING)
    p = worker.latest_state().tcp
    worker.stream(StreamTarget(p.with_axis("Y", p.y + 30), now_ns(), "phone", True))  # second hand on the arm
    assert wait_for(lambda: any("phone ignored: vr is moving the arm" in t for t in logs))
    feed(session, hand(z=-0.03, grip=0.0))  # VR lets go: the phone may take over now
    assert wait_for(lambda: worker.latest_state().mode is Mode.IDLE)
    for _ in range(10):
        p = worker.latest_state().tcp
        worker.stream(StreamTarget(p.with_axis("Y", p.y + 2), now_ns(), "phone", True))
        time.sleep(0.04)
    assert worker.commanded.source == "phone"


# ---- server end to end ----

def free_port():
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


def test_server_pairs_streams_and_releases_on_disconnect(rig):
    _, worker, session, _ = rig
    srv = VrServer(session, port=free_port(), host="127.0.0.1")
    srv.start()
    try:
        ctx = ssl.create_default_context()
        ctx.check_hostname, ctx.verify_mode = False, ssl.CERT_NONE
        with urllib.request.urlopen(f"https://127.0.0.1:{srv.port}/?k={srv.token}", context=ctx, timeout=3) as r:
            assert b"Botswain" in r.read()
            cookie = r.headers["set-cookie"].split(";")[0]
        ws = websocket.create_connection(f"wss://127.0.0.1:{srv.port}/ws", sslopt=SSL, cookie=cookie)
        ws.send(json.dumps(msg(hand(grip=0.9))))
        for i in range(25):
            ws.send(json.dumps(msg(hand(z=-0.002 * i, grip=0.9))))
            time.sleep(0.012)
        assert wait_for(lambda: worker.latest_state().mode is Mode.STREAMING)
        ws.send("not json")
        ws.close()  # the headset vanished with the clutch held
        # Faster than the 200 ms stale timeout: the disconnect itself released the clutch.
        assert wait_for(lambda: worker.latest_state().mode is Mode.IDLE, timeout=0.15)
        assert srv.bad_messages == 1 and srv.samples >= 26
    finally:
        srv.stop()
