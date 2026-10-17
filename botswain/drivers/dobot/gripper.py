"""DH-Robotics gripper on the tool RS485 port, driven through the controller's HTTP server (port 22000).

The controller forwards raw Modbus RTU frames: POST /interface/toolDataExchange {"value": [bytes]}.
Registers (DH): 0x0100 = init (write 1, needed once after power-up), 0x0103 = position in permille
(0 closed, 1000 open). Phone2Act's frames decoded exactly to these, CRC included.
"""

from __future__ import annotations

import json
import urllib.error
import urllib.request

from botswain.drivers.base import DriverError

REG_INIT = 0x0100
REG_POSITION = 0x0103


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


class DhGripper:
    def __init__(self, base_url: str, slave: int = 1, timeout_s: float = 1.0) -> None:
        self.base_url = base_url.rstrip("/")
        self.slave = slave
        self.timeout_s = timeout_s

    def init(self) -> str:
        return self._send(write_register_frame(self.slave, REG_INIT, 1))

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
            with urllib.request.urlopen(req, timeout=self.timeout_s) as resp:
                return resp.read().decode("utf-8", "replace")
        except (urllib.error.URLError, TimeoutError, OSError) as e:
            raise DriverError(-200, f"gripper HTTP request failed ({e})",
                              "check tool power and RS485 settings (115200 8N1) in DobotStudio") from e
