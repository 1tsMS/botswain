"""Security and robustness: phone pairing key, NaN inputs, YAML host validation, take control."""

import json
import socket
import ssl
import urllib.error
import urllib.request

import pytest
import websocket
import yaml
from helpers import ROBOTS, dobot_cfg, make_state, wait_for

from botswain.control.safety import Rejected, SafetyGate
from botswain.core.clock import now_ns
from botswain.core.config import ConfigError, load_robot_config
from botswain.core.types import JogStep, Joints, Mode, MoveTo, Pose, StreamTarget
from botswain.drivers.dobot.driver import DobotDriver
from botswain.drivers.dobot.fake_controller import FakeDobotController
from botswain.teleop.phone import PhoneServer, parse_message

SSL = {"cert_reqs": ssl.CERT_NONE}


def free_port():
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


@pytest.fixture
def server():
    got = []
    srv = PhoneServer(got.append, port=free_port(), host="127.0.0.1")
    srv.start()
    yield srv, got
    srv.stop()


def https_get(url):
    ctx = ssl.create_default_context()
    ctx.check_hostname = False
    ctx.verify_mode = ssl.CERT_NONE
    return urllib.request.urlopen(url, context=ctx, timeout=3)


def test_page_needs_the_pairing_key(server):
    srv, _ = server
    with pytest.raises(urllib.error.HTTPError) as e:
        https_get(f"https://127.0.0.1:{srv.port}/")
    assert e.value.code == 403
    with pytest.raises(urllib.error.HTTPError):
        https_get(f"https://127.0.0.1:{srv.port}/?k=guess")
    assert srv.token in srv.urls[0]


def test_websocket_without_key_is_refused(server):
    srv, got = server
    with pytest.raises(websocket.WebSocketException):
        ws = websocket.create_connection(f"wss://127.0.0.1:{srv.port}/ws", sslopt=SSL)
        ws.send(json.dumps({"type": "pose", "data": {}}))
        ws.recv()
    with pytest.raises(websocket.WebSocketException):
        websocket.create_connection(f"wss://127.0.0.1:{srv.port}/ws", sslopt=SSL, cookie="botswain_key=wrong").recv()
    assert wait_for(lambda: srv.refused >= 2) and got == []


def test_page_sets_cookie_that_opens_the_websocket(server):
    srv, _ = server
    with https_get(srv.urls[0].replace(srv.urls[0].split("/")[2].split(":")[0], "127.0.0.1")) as r:
        cookie = r.headers["set-cookie"]
    assert "httponly" in cookie.lower() and "secure" in cookie.lower() and srv.token in cookie
    ws = websocket.create_connection(f"wss://127.0.0.1:{srv.port}/ws", sslopt=SSL, cookie=cookie.split(";")[0])
    assert wait_for(lambda: srv.clients == 1)
    ws.close()


@pytest.mark.parametrize("bad", ["NaN", "Infinity", "-Infinity"])
def test_phone_nan_rejected(bad):
    data = {"position": {"x": float(bad), "y": 0, "z": 0}, "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}}
    with pytest.raises(ValueError):
        parse_message(data, 0)


def test_gate_refuses_nan_even_with_box_off():
    cfg = load_robot_config(ROBOTS / "dobot_cr5.yaml")
    gate = SafetyGate(cfg.limits, cfg.stream.period_s)
    assert gate.box_enabled is False
    nan_pose = Pose(float("nan"), 0, 400, 180, 0, 0)
    state = make_state(Mode.IDLE)
    for cmd in (MoveTo(nan_pose, "linear"), JogStep("X", float("inf")), MoveTo(Joints((0, float("nan"), 0, 0, 0, 0)), "joint")):
        v = gate.check(cmd, state)
        assert isinstance(v, Rejected) and v.rule == "invalid"
    stream = gate.check(StreamTarget(nan_pose, now_ns(), "t", True), make_state(Mode.STREAMING))
    assert stream.rule == "invalid"


@pytest.mark.parametrize("ip", ["-t 10.0.0.1", "1.2.3.4 && calc", "", "a" * 300])
def test_bad_ip_in_yaml_rejected(tmp_path, ip):
    data = yaml.safe_load((ROBOTS / "dobot_cr5.yaml").read_text(encoding="utf-8"))
    data["connection"]["ip"] = ip
    path = tmp_path / "r.yaml"
    path.write_text(yaml.safe_dump(data), encoding="utf-8")
    with pytest.raises(ConfigError) as e:
        load_robot_config(path)
    assert e.value.field == "connection.ip"


def test_host_names_still_allowed(tmp_path):
    data = yaml.safe_load((ROBOTS / "dobot_cr5.yaml").read_text(encoding="utf-8"))
    data["connection"]["ip"] = "cr5-lab.local"
    path = tmp_path / "r.yaml"
    path.write_text(yaml.safe_dump(data), encoding="utf-8")
    assert load_robot_config(path).connection.ip == "cr5-lab.local"


def test_take_control_before_connect():
    ctrl = FakeDobotController()
    ctrl.tcp_mode = False
    ctrl.start()
    try:
        d = DobotDriver(dobot_cfg(ctrl))
        d.request_control()  # works while not connected: that is the point
        assert ctrl.tcp_mode and "RequestControl()" in ctrl.commands
        d.connect()
        assert wait_for(lambda: d.read_state().link_ok)
        d.disconnect()
    finally:
        ctrl.stop()
