import socket
import threading
import time

import pytest

from botswain.drivers.base import DriverError
from botswain.drivers.dobot.dashboard import NOT_TCP, TIMEOUT, DashboardClient, describe, parse_reply


def test_parse_reply_values():
    r = parse_reply("0,{5},RobotMode();")
    assert (r.error_id, r.values, r.echo) == (0, "5", "RobotMode()")
    assert r.numbers() == [5.0]


def test_parse_reply_error():
    r = parse_reply("-2,{},MovJ(pose={1,2,3,4,5,6});")
    assert r.error_id == -2 and r.values == "" and r.echo.startswith("MovJ(")


def test_parse_reply_nested_values():
    r = parse_reply("0,{[[22,23],[],[]]},GetErrorID();")
    assert r.values == "[[22,23],[],[]]" and r.numbers() == [22.0, 23.0]


def test_parse_reply_garbage():
    with pytest.raises(ValueError):
        parse_reply("hello")


def test_describe_param_errors():
    assert "parameter 2" in describe(-40002)[0]
    assert describe(-2)[0] == "robot in alarm"


class ScriptedServer:
    """Accepts one client; for each received line sends the next scripted list of (bytes, delay_s)."""

    def __init__(self, script):
        self.script = list(script)
        self.sock = socket.socket()
        self.sock.bind(("127.0.0.1", 0))
        self.sock.listen(1)
        self.port = self.sock.getsockname()[1]
        threading.Thread(target=self._serve, daemon=True).start()

    def _serve(self):
        conn, _ = self.sock.accept()
        buf = b""
        while self.script:
            data = conn.recv(1024)
            if not data:
                return
            buf += data
            while b"\n" in buf and self.script:
                _, buf = buf.split(b"\n", 1)
                for chunk, delay in self.script.pop(0):
                    time.sleep(delay)
                    conn.sendall(chunk)
        time.sleep(0.5)
        conn.close()


def client_for(script, timeout_s=1.0):
    server = ScriptedServer(script)
    c = DashboardClient("127.0.0.1", server.port, timeout_s=timeout_s)
    c.connect()
    return c


def test_reply_split_and_merged():
    c = client_for([
        [(b"0,{5},Robo", 0), (b"tMode();", 0.05)],  # one reply in two reads
        [(b"0,{},MovJ(joint={0,0,0,0,0,0});0,{10,20,30,0,0,0},GetPose();", 0)],  # stale reply + ours in one read
        [(b"0,{4},RobotMode();", 0)],
    ])
    assert c.call("RobotMode()").values == "5"
    pose = c.call("GetPose()")
    assert pose.echo == "GetPose()" and pose.numbers()[:3] == [10.0, 20.0, 30.0]
    assert c.call("RobotMode()").values == "4"


def test_timeout_then_late_reply_is_not_mixed_up():
    c = client_for([
        [(b"0,{5},RobotMode();", 0.4)],  # answer comes after the client gave up
        [(b"0,{1,2,3,4,5,6},GetAngle();", 0)],
    ], timeout_s=0.2)
    with pytest.raises(DriverError) as e:
        c.call("RobotMode()")
    assert e.value.code == TIMEOUT
    time.sleep(0.3)
    assert c.call("GetAngle()").numbers() == [1, 2, 3, 4, 5, 6]


def test_not_tcp_reply():
    c = client_for([[(b"Not Tcp", 0)], [(b"Not Tcp", 0)]])
    assert c.call("RobotMode()").error_id == NOT_TCP
    with pytest.raises(DriverError) as e:
        c.call_ok("EnableRobot()")
    assert e.value.code == NOT_TCP and "TCP/IP" in e.value.hint


def test_call_ok_raises_with_meaning():
    c = client_for([[(b"-2,{},EnableRobot();", 0)]])
    with pytest.raises(DriverError) as e:
        c.call_ok("EnableRobot()")
    assert e.value.code == -2 and "alarm" in e.value.message and e.value.hint
