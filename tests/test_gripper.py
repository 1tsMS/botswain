from botswain.drivers.dobot.gripper import crc16_modbus, write_register_frame


def test_crc_matches_phone2act():
    assert write_register_frame(1, 0x0103, 0) == [1, 6, 1, 3, 0, 0, 120, 54]  # close
    assert write_register_frame(1, 0x0103, 1000) == [1, 6, 1, 3, 3, 232, 120, 136]  # open


def test_init_frame():
    assert write_register_frame(1, 0x0100, 1) == [1, 6, 1, 0, 0, 1, 73, 246]


def test_crc_of_frame_with_crc_is_zero():
    frame = write_register_frame(1, 0x0103, 500)
    assert crc16_modbus(bytes(frame)) == 0
