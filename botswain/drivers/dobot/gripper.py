"""DH-Robotics gripper on the tool RS485 port, driven through the controller's HTTP server (port 22000).

The controller forwards raw Modbus RTU frames: POST /interface/toolDataExchange {"value": [bytes]}.
Registers (DH, checked against DH-Robotics/dh_gripper_ros dh_modbus_gripper.cpp): 0x0100 init (write 1, needed
once after power-up), 0x0101 closing force %, 0x0103 target position in permille (0 closed, 1000 open), 0x0200
init state (1 = done), 0x0201 grip state, 0x0202 current position. Phone2Act's frames decoded exactly to these.

The PGE has no force sensor: it limits motor current. Closing stops when the set force is reached, and the grip
state then says "object caught". So "close until this force" is the force setting plus an ordinary close.
"""

from __future__ import annotations

import json
import threading
import urllib.error
import urllib.request

from botswain.drivers.base import DriverError

REG_INIT = 0x0100
REG_FORCE = 0x0101
REG_POSITION = 0x0103
REG_INIT_STATE = 0x0200
REG_GRIP_STATE = 0x0201
REG_CURRENT_POS = 0x0202
FORCE_RANGE = (20, 100)  # % (DH PGE manual)
GRIP_STATES = {0: "moving", 1: "at position", 2: "object caught", 3: "object dropped"}


def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def write_register_frame(slave: int, reg: int, value: int) -> list[int]:
    """Modbus function 06 (write one register), CRC low byte first."""
    body = [slave, 0x06, reg >> 8, reg & 0xFF, value >> 8, value & 0xFF]
    crc = crc16_modbus(bytes(body))
    return [*body, crc & 0xFF, crc >> 8]


def read_register_frame(slave: int, reg: int) -> list[int]:
    """Modbus function 03 (read holding registers), one register."""
    body = [slave, 0x03, reg >> 8, reg & 0xFF, 0x00, 0x01]
    crc = crc16_modbus(bytes(body))
    return [*body, crc & 0xFF, crc >> 8]


def parse_read_reply(frame: list[int], slave: int) -> int | None:
    """The register value from a function-03 reply [slave, 3, 2, hi, lo, crc_lo, crc_hi], or None if it is not one."""
    if len(frame) != 7 or frame[0] != slave or frame[1] != 0x03 or frame[2] != 2:
        return None
    if crc16_modbus(bytes(frame[:5])) != frame[5] | (frame[6] << 8):
        return None
    return (frame[3] << 8) | frame[4]


class DhGripper:
    def __init__(self, base_url: str, slave: int = 1, timeout_s: float = 1.0) -> None:
        self.base_url = base_url.rstrip("/")
        self.slave = slave
        self.timeout_s = timeout_s
        self._lock = threading.Lock()  # one frame at a time on the tool RS485 bus (commands vs the status poll)

    def init(self) -> str:
        return self._send(write_register_frame(self.slave, REG_INIT, 1))

    def set_force(self, pct: int) -> str:
        lo, hi = FORCE_RANGE
        if not lo <= pct <= hi:
            raise ValueError(f"gripper force must be {lo}..{hi} %, got {pct}")
        return self._send(write_register_frame(self.slave, REG_FORCE, int(pct)))

    def read(self, reg: int) -> int:
        """One register. Raises DriverError if the controller does not hand back a valid gripper reply."""
        body = self._send(read_register_frame(self.slave, reg))
        try:
            frame = json.loads(body)["data"]["value"]
            value = parse_read_reply([int(b) for b in frame], self.slave)
        except (ValueError, KeyError, TypeError):
            value = None
        if value is None:
            raise DriverError(-201, f"gripper read 0x{reg:04X}: no valid reply ({body[:120]})",
                              "check the gripper's RS485 settings and slave id")
        return value

    def set_position(self, permille: int) -> str:
        if not 0 <= permille <= 1000:
            raise ValueError(f"gripper position must be 0..1000, got {permille}")
        return self._send(write_register_frame(self.slave, REG_POSITION, permille))

    def open(self) -> str:
        return self.set_position(1000)

    def close(self) -> str:
        return self.set_position(0)

    def _send(self, frame: list[int]) -> str:
        """Returns the controller's HTTP body, so the log shows what it said."""
        req = urllib.request.Request(
            f"{self.base_url}/interface/toolDataExchange", data=json.dumps({"value": frame}).encode(),
            headers={"Content-Type": "application/json"}, method="POST")
        try:
            with self._lock, urllib.request.urlopen(req, timeout=self.timeout_s) as resp:
                return resp.read().decode("utf-8", "replace")
        except (urllib.error.URLError, TimeoutError, OSError) as e:
            raise DriverError(-200, f"gripper HTTP request failed ({e})",
                              "check tool power and RS485 settings (115200 8N1) in DobotStudio") from e
