import json
import socket
import ssl
import time

import pytest
import websocket
from helpers import wait_for

from botswain.teleop.phone import PhoneServer, local_ips, parse_message

SAMPLE = {"position": {"x": 0.1, "y": 1.2, "z": -0.3}, "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
          "move": True, "gripper": "close", "scale": 0.5, "reservedButtonA": False, "fps": 60, "device": "test"}


def test_parse_message_from_spes_format():
    s = parse_message(SAMPLE, 123)
    assert s.t_ns == 123 and s.position_m == (0.1, 1.2, -0.3) and s.quat_wxyz == (1.0, 0.0, 0.0, 0.0)
    assert s.hold and s.gripper_closed and s.scale == 0.5


def test_parse_message_defaults():
    s = parse_message({"position": SAMPLE["position"], "orientation": SAMPLE["orientation"]}, 0)
    assert not s.hold and not s.gripper_closed and s.scale == 1.0


def test_local_ips_are_private():
    for ip in local_ips():
        assert ip.startswith(("192.168.", "10.", "172."))


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


def test_phone_server_end_to_end(server):
    srv, got = server
    ws = websocket.create_connection(f"wss://127.0.0.1:{srv.port}/ws", sslopt={"cert_reqs": ssl.CERT_NONE})
    try:
        assert wait_for(lambda: srv.clients == 1)
        ws.send(json.dumps({"type": "pose", "data": SAMPLE}))
        ws.send("not json")
        ws.send(json.dumps({"type": "log", "data": "hello"}))
        assert wait_for(lambda: len(got) == 1 and srv.bad_messages == 1)
        assert got[0].hold and got[0].position_m == (0.1, 1.2, -0.3)
    finally:
        ws.close()
    assert wait_for(lambda: srv.clients == 0)


def test_phone_page_served_over_https(server):
    import urllib.request
    srv, _ = server
    ctx = ssl.create_default_context()
    ctx.check_hostname = False
    ctx.verify_mode = ssl.CERT_NONE
    with urllib.request.urlopen(f"https://127.0.0.1:{srv.port}/", context=ctx, timeout=3) as r:
        assert r.status == 200 and b"<html" in r.read().lower()


def test_server_stops_and_frees_the_port():
    port = free_port()
    srv = PhoneServer(lambda s: None, port=port, host="127.0.0.1")
    srv.start()
    srv.stop()
    time.sleep(0.1)
    s = socket.socket()
    s.bind(("127.0.0.1", port))  # would raise if the server still held it
    s.close()
