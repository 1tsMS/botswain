import socket
from dataclasses import replace

import pytest
from helpers import ROBOTS, dobot_cfg, wait_for

from botswain.control.doctor import network_checks, offline_checks
from botswain.core.config import load_robot_config
from botswain.drivers.dobot.driver import DobotDriver
from botswain.drivers.dobot.fake_controller import FakeDobotController


@pytest.fixture
def ctrl():
    c = FakeDobotController()
    c.start()
    yield c
    c.stop()


def by_name(checks, part):
    return next(c for c in checks if part in c.name)


def closed_port():
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


def test_doctor_all_ok_on_fake(ctrl):
    cfg = dobot_cfg(ctrl)
    d = DobotDriver(cfg)
    d.connect()
    try:
        assert wait_for(lambda: d.read_state().link_ok)
        wait_for(lambda: d.read_state().feedback_hz > 0, timeout=1.0)
        checks = [c for c in network_checks(cfg, connected=True) + d.diagnose() if "fallback" not in c.name]
        assert [c for c in checks if c.status != "ok"] == []
        assert {"Ping 127.0.0.1", "TCP/IP mode", "Robot mode", "Alarms", "Pose agreement"} <= {c.name for c in checks}
    finally:
        d.disconnect()


def test_doctor_before_connect_probes_itself(ctrl):
    checks = network_checks(dobot_cfg(ctrl), connected=False) + offline_checks(dobot_cfg(ctrl), sample_s=0.5)
    assert by_name(checks, "Feedback rate").status == "ok"
    assert by_name(checks, "TCP/IP mode").status == "ok"
    assert by_name(checks, "Dashboard port").status == "ok"


def test_doctor_port_closed_fails_with_hint(ctrl):
    cfg = dobot_cfg(ctrl)
    cfg = replace(cfg, connection=replace(cfg.connection, feedback_port=closed_port()))
    check = by_name(network_checks(cfg, connected=False), "Feedback port")
    assert check.status == "fail" and "192.168.5" in check.hint


def test_doctor_not_tcp(ctrl):
    ctrl.tcp_mode = False
    check = by_name(offline_checks(dobot_cfg(ctrl), sample_s=0.2), "TCP/IP mode")
    assert check.status == "fail" and "DobotStudio" in check.hint


def test_doctor_pose_mismatch_warns(ctrl):
    ctrl.pose_offset_mm = 2.0
    d = DobotDriver(dobot_cfg(ctrl))
    d.connect()
    try:
        assert wait_for(lambda: d.read_state().link_ok)
        check = by_name(d.diagnose(), "Pose agreement")
        assert check.status == "warn" and "2.00 mm" in check.value
    finally:
        d.disconnect()


def test_doctor_alarm_listed(ctrl):
    ctrl.inject_alarm(22)
    d = DobotDriver(dobot_cfg(ctrl))
    d.connect()
    try:
        assert wait_for(lambda: d.read_state().link_ok)
        checks = d.diagnose()
        assert by_name(checks, "Alarms").status == "fail" and "22" in by_name(checks, "Alarms").value
        assert by_name(checks, "Robot mode").status == "fail"
    finally:
        d.disconnect()


def test_doctor_fake_robot_has_nothing_to_check():
    checks = network_checks(load_robot_config(ROBOTS / "fake.yaml"), connected=False)
    assert [c.status for c in checks] == ["ok"]
