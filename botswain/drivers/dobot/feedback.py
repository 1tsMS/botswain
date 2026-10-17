"""Real-time feedback port (30004): one 1440-byte little-endian packet every 8 ms.

Offsets come from the SDK's numpy dtype (RESEARCH_NOTES section 1). The SDK reader can parse packets that
start mid-way, because TCP does not keep packet borders. FrameReader keeps a byte buffer, cuts exact
1440-byte frames, checks the length field and TestValue, and resyncs on the next TestValue if they fail.
"""

from __future__ import annotations

import logging
import socket
import struct
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass

from botswain.core.clock import now_ns
from botswain.drivers.base import DriverError

log = logging.getLogger(__name__)

FEEDBACK_SIZE = 1440
TEST_VALUE = 0x0123456789ABCDEF
TEST_BYTES = struct.pack("<Q", TEST_VALUE)

OFFSETS = {
    "len": 0, "robot_mode": 24, "timestamp": 32, "test_value": 48, "speed_scaling": 64,
    "q_target": 192, "q_actual": 432, "tool_vector_actual": 624, "tool_vector_target": 768,
    "enable": 1026, "error": 1029, "collision": 1038, "command_id": 1112,
}
STALE_NS = 300_000_000  # no packet for this long -> the link counts as lost


@dataclass(frozen=True)
class FeedbackPacket:
    mode: int
    timestamp_ms: int
    q_actual: tuple[float, ...]
    q_target: tuple[float, ...]
    tcp_actual: tuple[float, ...]
    tcp_target: tuple[float, ...]
    speed_scaling: float
    enable: int
    error: int
    collision: int
    command_id: int


def _valid_header(b: bytes | bytearray, at: int = 0) -> bool:
    return (struct.unpack_from("<H", b, at)[0] == FEEDBACK_SIZE
            and b[at + OFFSETS["test_value"]:at + OFFSETS["test_value"] + 8] == TEST_BYTES)


def parse_packet(b: bytes | bytearray) -> FeedbackPacket | None:
    if len(b) != FEEDBACK_SIZE or not _valid_header(b):
        return None

    def six(name: str) -> tuple[float, ...]:
        return struct.unpack_from("<6d", b, OFFSETS[name])

    return FeedbackPacket(
        mode=struct.unpack_from("<Q", b, OFFSETS["robot_mode"])[0],
        timestamp_ms=struct.unpack_from("<Q", b, OFFSETS["timestamp"])[0],
        q_actual=six("q_actual"), q_target=six("q_target"),
        tcp_actual=six("tool_vector_actual"), tcp_target=six("tool_vector_target"),
        speed_scaling=struct.unpack_from("<d", b, OFFSETS["speed_scaling"])[0],
        enable=struct.unpack_from("<b", b, OFFSETS["enable"])[0],
        error=struct.unpack_from("<b", b, OFFSETS["error"])[0],
        collision=struct.unpack_from("<b", b, OFFSETS["collision"])[0],
        command_id=struct.unpack_from("<Q", b, OFFSETS["command_id"])[0],
    )


def build_packet(mode: int = 4, timestamp_ms: int = 0, q_actual=(0.0,) * 6, q_target=(0.0,) * 6,
                 tcp_actual=(0.0,) * 6, tcp_target=(0.0,) * 6, speed_scaling: float = 0.0, enable: int = 0,
                 error: int = 0, collision: int = 0, command_id: int = 0) -> bytes:
    """A valid packet (used by the fake controller and the tests)."""
    b = bytearray(FEEDBACK_SIZE)
    struct.pack_into("<H", b, OFFSETS["len"], FEEDBACK_SIZE)
    struct.pack_into("<Q", b, OFFSETS["test_value"], TEST_VALUE)
    struct.pack_into("<Q", b, OFFSETS["robot_mode"], mode)
    struct.pack_into("<Q", b, OFFSETS["timestamp"], timestamp_ms)
    struct.pack_into("<d", b, OFFSETS["speed_scaling"], speed_scaling)
    for name, values in (("q_actual", q_actual), ("q_target", q_target), ("tool_vector_actual", tcp_actual),
                         ("tool_vector_target", tcp_target)):
        struct.pack_into("<6d", b, OFFSETS[name], *values)
    for name, value in (("enable", enable), ("error", error), ("collision", collision)):
        struct.pack_into("<b", b, OFFSETS[name], value)
    struct.pack_into("<Q", b, OFFSETS["command_id"], command_id)
    return bytes(b)


class FrameReader:
    def __init__(self) -> None:
        self._buf = bytearray()
        self.bad_frames = 0

    def feed(self, data: bytes) -> list[FeedbackPacket]:
        self._buf += data
        out: list[FeedbackPacket] = []
        while len(self._buf) >= FEEDBACK_SIZE:
            if _valid_header(self._buf):
                packet = parse_packet(self._buf[:FEEDBACK_SIZE])
                if packet is not None:
                    out.append(packet)
                del self._buf[:FEEDBACK_SIZE]
                continue
            # Lost the frame border: jump to the next TestValue (48 bytes into a packet).
            self.bad_frames += 1
            idx = self._buf.find(TEST_BYTES, OFFSETS["test_value"] + 1)
            if idx == -1:
                del self._buf[:len(self._buf) - (OFFSETS["test_value"] + 7)]  # keep a possible partial header
                break
            del self._buf[:idx - OFFSETS["test_value"]]
        return out


class FeedbackThread:
    """Reads 30004 in the background and hands over only the newest packet, stamped with its arrival time."""

    def __init__(self, ip: str, port: int, on_packet: Callable[[FeedbackPacket, int], None],
                 connect_timeout_s: float = 1.0) -> None:
        self.ip = ip
        self.port = port
        self.on_packet = on_packet
        self.connect_timeout_s = connect_timeout_s
        self.reader = FrameReader()
        self.last_packet_ns = 0
        self._sock: socket.socket | None = None
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="dobot-feedback", daemon=True)
        self._count = 0
        self._window_ns = now_ns()
        self._hz = 0.0

    @property
    def hz(self) -> float:
        return self._hz if now_ns() - self.last_packet_ns < STALE_NS else 0.0

    @property
    def bad_frames(self) -> int:
        return self.reader.bad_frames

    def start(self) -> None:
        """Connect first (so a closed port raises right here), then read in the background."""
        self._sock = self._open()
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._close()
        if self._thread.is_alive():
            self._thread.join(1.0)

    def _open(self) -> socket.socket:
        try:
            sock = socket.create_connection((self.ip, self.port), timeout=self.connect_timeout_s)
        except OSError as e:
            raise DriverError(-101, f"cannot open feedback port {self.ip}:{self.port} ({e})",
                              "check the controller is on and port 30004 is reachable") from e
        sock.settimeout(0.2)
        return sock

    def _close(self) -> None:
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None

    def _run(self) -> None:
        while not self._stop.is_set():
            if self._sock is None:
                try:
                    self._sock = self._open()
                    self.reader = FrameReader()
                    log.info("feedback reconnected")
                except DriverError:
                    time.sleep(0.5)
                    continue
            try:
                data = self._sock.recv(65536)
            except TimeoutError:
                continue
            except OSError:
                data = b""
            if not data:
                if not self._stop.is_set():
                    log.warning("feedback connection dropped, retrying")
                self._close()
                continue
            packets = self.reader.feed(data)
            if packets:
                t = now_ns()
                self.last_packet_ns = t
                self._count_rate(len(packets), t)
                self.on_packet(packets[-1], t)  # newest wins; older ones in this read are stale already

    def _count_rate(self, n: int, t: int) -> None:
        self._count += n
        elapsed = t - self._window_ns
        if elapsed >= 500_000_000:
            self._hz = self._count * 1e9 / elapsed
            self._count = 0
            self._window_ns = t
