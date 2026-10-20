"""The DH gripper as its own end-effector driver: Modbus frames, the tool driver on a pretend bus, the full path
through a Dobot's RS485 tool port (HTTP 22000), and the worker that owns it."""

import pytest
from helpers import ROBOTS, dh_tool, dobot_cfg, wait_for

from botswain.control.safety import SafetyGate
from botswain.control.worker import RobotWorker
from botswain.core.config import load_robot_config
from botswain.core.types import Connect, Gripper, Mode, SetGripperForce
from botswain.drivers.base import DriverError
from botswain.drivers.dobot.driver import DobotV3Driver
from botswain.drivers.dobot.fake_controller import FakeDobotController
from botswain.drivers.fake import FakeDriver
from botswain.tools.dh_modbus import (
    crc16_modbus,
    parse_read_reply,
    read_register_frame,
    write_register_frame,
)
from botswain.tools.fake_dh import FakeDhBus

# ---- frames ----


def test_crc_matches_phone2act():
    assert write_register_frame(1, 0x0103, 0) == [1, 6, 1, 3, 0, 0, 120, 54]  # close
    assert write_register_frame(1, 0x0103, 1000) == [1, 6, 1, 3, 3, 232, 120, 136]  # open
    assert write_register_frame(1, 0x0100, 1) == [1, 6, 1, 0, 0, 1, 73, 246]  # init


def test_read_frame_and_reply():
    frame = read_register_frame(1, 0x0201)
    assert frame[:6] == [1, 3, 2, 1, 0, 1] and crc16_modbus(bytes(frame)) == 0
    body = [1, 3, 2, 0x01, 0x2C]  # 300
    crc = crc16_modbus(bytes(body))
    assert parse_read_reply([*body, crc & 0xFF, crc >> 8], 1) == 300
    assert parse_read_reply([*body, crc & 0xFF, (crc >> 8) ^ 1], 1) is None  # bad CRC
    assert parse_read_reply([1, 6, 1, 3, 0, 0, 120, 54], 1) is None  # a write echo is not a read reply
    assert parse_read_reply([2, *body[1:], crc & 0xFF, crc >> 8], 1) is None  # other slave


# ---- the tool driver on a pretend bus ----


def writes_to(bus, reg):
    return [f for f in bus.frames if f[1] == 0x06 and (f[2] << 8 | f[3]) == reg]


@pytest.fixture
def bus_tool():
    bus, tool = FakeDhBus(), dh_tool()
    tool.connect(bus)
    yield bus, tool
    tool.disconnect()


def test_force_waits_for_init_then_close_stops_on_the_object(bus_tool):
    bus, tool = bus_tool
    assert wait_for(lambda: tool.status().text.startswith("not initialised"))
    assert writes_to(bus, 0x0100) == []  # the readout never initialises (init moves the jaws)
    tool.set_force(40)
    assert writes_to(bus, 0x0101) == []  # not initialised yet: kept for the first close
    bus.object_permille = 300
    tool.set_open(False)
    assert [f[3] for f in bus.frames if f[1] == 0x06] == [0x00, 0x01, 0x03]  # init, force, then close
    assert writes_to(bus, 0x0101)[0][4:6] == [0, 40]
    assert wait_for(lambda: tool.status().text == "object caught")
    assert tool.status().opening == pytest.approx(0.3) and tool.status().commanded == 0.0
    tool.set_force(70)  # initialised now: written at once
    assert writes_to(bus, 0x0101)[-1][4:6] == [0, 70]
    assert tool.diagnose()[0].value.startswith("register 0x0101 = 70 %")


def test_already_initialised_gripper_is_not_initialised_again(bus_tool):
    bus, tool = bus_tool
    bus.regs[0x0200] = 1  # e.g. DobotStudio did it
    assert wait_for(lambda: tool.status().text == "at position")
    tool.set_open(True)
    assert writes_to(bus, 0x0100) == [] and tool.status().commanded == 1.0


def test_force_out_of_range_is_refused(bus_tool):
    _, tool = bus_tool
    with pytest.raises(DriverError):
        tool.set_force(10)


def test_silent_bus_reads_as_no_reading():
    class Silent:
        kind = "rs485"

        def exchange(self, frame):
            return []

    tool = dh_tool()
    tool.connect(Silent())
    try:
        assert wait_for(lambda: tool.status().text == "no reading")
    finally:
        tool.disconnect()


# ---- through a Dobot's tool port ----


def test_gripper_through_the_dobot_tool_port():
    ctrl = FakeDobotController()
    ctrl.v3 = True
    ctrl.start()
    d = DobotV3Driver(dobot_cfg(ctrl))
    tool = dh_tool()
    try:
        d.connect()
        port = d.tool_port()
        assert port is not None and port.kind == "rs485"
        tool.connect(port)
        tool.set_open(True)
        tool.set_open(False)
        writes = [f for f in ctrl.tool_bus.frames if f[1] == 0x06]  # reads (03) are the status poll
        assert writes == [[1, 6, 1, 0, 0, 1, 73, 246], [1, 6, 1, 3, 3, 232, 120, 136], [1, 6, 1, 3, 0, 0, 120, 54]]
        assert wait_for(lambda: tool.status().opening == 0.0)
    finally:
        tool.disconnect()
        d.disconnect()
        ctrl.stop()


# ---- owned by the worker ----


@pytest.fixture
def worker():
    cfg = load_robot_config(ROBOTS / "fake.yaml")
    w = RobotWorker(FakeDriver(cfg), SafetyGate(cfg.limits, cfg.stream.period_s), tool=dh_tool())
    w.start()
    w.submit(Connect())
    assert wait_for(lambda: w.latest_state().mode is Mode.DISABLED)
    yield w
    w.shutdown()


def test_worker_routes_gripper_commands_and_merges_status(worker):
    assert worker.capabilities.gripper and worker.capabilities.gripper_force
    assert not worker.driver.capabilities.gripper  # the arm itself has no gripper
    worker.submit(SetGripperForce(60))
    worker.submit(Gripper(False))
    assert wait_for(lambda: worker.latest_state().gripper == 0.0)
    assert wait_for(lambda: worker.latest_state().gripper_status == "at position")
    assert writes_to(worker.driver.tool_bus, 0x0101)[-1][4:6] == [0, 60]


def test_worker_without_end_effector_says_so():
    cfg = load_robot_config(ROBOTS / "fake.yaml")
    w = RobotWorker(FakeDriver(cfg), SafetyGate(cfg.limits, cfg.stream.period_s))
    logs = []
    w.on_log = lambda level, text: logs.append(text)
    w.start()
    try:
        w.submit(Connect())
        assert wait_for(lambda: w.latest_state().mode is Mode.DISABLED)
        assert not w.capabilities.gripper
        w.submit(Gripper(True))
        assert wait_for(lambda: any("no end effector" in t for t in logs))
    finally:
        w.shutdown()
