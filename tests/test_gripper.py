import pytest
from helpers import dobot_cfg, wait_for

from botswain.drivers.base import DriverError
from botswain.drivers.dobot.driver import DobotDriver
from botswain.drivers.dobot.fake_controller import FakeDobotController
from botswain.drivers.dobot.gripper import crc16_modbus, write_register_frame


def test_crc_matches_phone2act():
    assert write_register_frame(1, 0x0103, 0) == [1, 6, 1, 3, 0, 0, 120, 54]  # close
    assert write_register_frame(1, 0x0103, 1000) == [1, 6, 1, 3, 3, 232, 120, 136]  # open


def test_init_frame():
    assert write_register_frame(1, 0x0100, 1) == [1, 6, 1, 0, 0, 1, 73, 246]


def test_crc_of_frame_with_crc_is_zero():
    frame = write_register_frame(1, 0x0103, 500)
    assert crc16_modbus(bytes(frame)) == 0


def test_read_frame_and_reply():
    from botswain.drivers.dobot.gripper import parse_read_reply, read_register_frame

    frame = read_register_frame(1, 0x0201)
    assert frame[:6] == [1, 3, 2, 1, 0, 1] and crc16_modbus(bytes(frame)) == 0
    body = [1, 3, 2, 0x01, 0x2C]  # 300
    crc = crc16_modbus(bytes(body))
    assert parse_read_reply([*body, crc & 0xFF, crc >> 8], 1) == 300
    assert parse_read_reply([*body, crc & 0xFF, (crc >> 8) ^ 1], 1) is None  # bad CRC
    assert parse_read_reply([1, 6, 1, 3, 0, 0, 120, 54], 1) is None  # a write echo is not a read reply
    assert parse_read_reply([2, *body[1:], crc & 0xFF, crc >> 8], 1) is None  # other slave


@pytest.fixture
def dobot():
    ctrl = FakeDobotController()
    ctrl.start()
    d = DobotDriver(dobot_cfg(ctrl))
    d.connect()
    yield ctrl, d
    d.disconnect()
    ctrl.stop()


def frames_to(ctrl, reg):
    return [f for f in ctrl.gripper_frames if f[1] == 0x06 and (f[2] << 8 | f[3]) == reg]


def test_force_waits_for_init_then_close_stops_on_the_object(dobot):
    ctrl, d = dobot
    assert wait_for(lambda: d.read_state().gripper_status.startswith("not initialised"))
    assert frames_to(ctrl, 0x0100) == []  # the readout never initialises (init moves the jaws)
    d.set_gripper_force(40)
    assert frames_to(ctrl, 0x0101) == []  # not initialised yet: kept for the first close
    ctrl.gripper_object_permille = 300
    d.gripper(False)
    order = [f[3] for f in ctrl.gripper_frames if f[1] == 0x06]
    assert order == [0x00, 0x01, 0x03]  # init, force, then close
    assert frames_to(ctrl, 0x0101)[0][4:6] == [0, 40]
    assert wait_for(lambda: d.read_state().gripper_status == "object caught")
    assert d.read_state().gripper_pos == pytest.approx(0.3)
    d.set_gripper_force(70)  # initialised now: written at once
    assert frames_to(ctrl, 0x0101)[-1][4:6] == [0, 70]


def test_already_initialised_gripper_is_not_initialised_again(dobot):
    ctrl, d = dobot
    ctrl.gripper_regs[0x0200] = 1  # e.g. DobotStudio did it
    assert wait_for(lambda: d.read_state().gripper_status == "at position")
    d.gripper(True)
    assert frames_to(ctrl, 0x0100) == [] and d.read_state().gripper == 1.0


def test_force_out_of_range_is_refused(dobot):
    _, d = dobot
    with pytest.raises(DriverError):
        d.set_gripper_force(10)
