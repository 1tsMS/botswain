import struct

from botswain.drivers.dobot.feedback import (
    FEEDBACK_SIZE,
    TEST_VALUE,
    FrameReader,
    build_packet,
    parse_packet,
)

Q = (1.5, -2.0, 90.0, 0.25, -90.0, 7.0)
TCP = (-482.0, -136.0, 484.0, 180.0, 0.0, 90.0)


def packet(mode=5, **kw):
    return build_packet(mode=mode, q_actual=Q, tcp_actual=TCP, q_target=Q, tcp_target=TCP, **kw)


def test_build_parse_roundtrip():
    p = parse_packet(packet(mode=7, enable=1, command_id=42, speed_scaling=0.3))
    assert p.mode == 7 and p.q_actual == Q and p.tcp_actual == TCP and p.enable == 1 and p.command_id == 42


def test_offsets_match_research_notes():
    b = packet(mode=9)
    assert len(b) == FEEDBACK_SIZE == struct.unpack_from("<H", b, 0)[0]
    assert struct.unpack_from("<Q", b, 24)[0] == 9
    assert struct.unpack_from("<Q", b, 48)[0] == TEST_VALUE
    assert struct.unpack_from("<6d", b, 432) == Q
    assert struct.unpack_from("<6d", b, 624) == TCP


def test_parse_rejects_bad_test_value():
    b = bytearray(packet())
    b[50] ^= 0xFF
    assert parse_packet(bytes(b)) is None


def test_frame_reader_split_packet():
    r = FrameReader()
    b = packet()
    assert r.feed(b[:100]) == [] and r.feed(b[100:1000]) == []
    out = r.feed(b[1000:])
    assert len(out) == 1 and out[0].q_actual == Q and r.bad_frames == 0


def test_frame_reader_two_in_one():
    out = FrameReader().feed(packet(mode=4) + packet(mode=5))
    assert [p.mode for p in out] == [4, 5]


def test_frame_reader_resyncs_after_garbage():
    r = FrameReader()
    out = r.feed(bytes(range(17)) + packet(mode=4) + packet(mode=5))
    assert [p.mode for p in out] == [4, 5]
    assert r.bad_frames >= 1


def test_frame_reader_resyncs_mid_stream():
    r = FrameReader()
    b = packet()
    out = r.feed(b[700:] + b + b)  # starts mid-packet, like the SDK bug
    assert len(out) == 2 and r.bad_frames >= 1
