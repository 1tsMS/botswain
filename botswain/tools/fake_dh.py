"""A pretend DH gripper on an RS485 bus: the same registers and replies as the real one, no hardware.

Used behind the fake Dobot controller's HTTP port and as the fake robot's tool port, so the real DhModbusTool
code runs in every test.
"""

from __future__ import annotations

import threading

from botswain.tools.dh_modbus import crc16_modbus


class FakeDhBus:
    kind = "rs485"

    def __init__(self) -> None:
        self.regs: dict[int, int] = {0x0200: 0, 0x0201: 1, 0x0202: 1000}  # not initialised, open
        self.object_permille: int | None = None  # set: a close stops here with "object caught"
        self.frames: list[list[int]] = []
        self._lock = threading.Lock()

    def exchange(self, frame: list[int]) -> list[int]:
        """Write (06) echoes and sets the register, read (03) answers it. A bad CRC gets no answer."""
        with self._lock:
            self.frames.append(list(frame))
            if len(frame) < 8 or crc16_modbus(bytes(frame[:-2])) != frame[-2] | (frame[-1] << 8):
                return []
            reg, value = (frame[2] << 8) | frame[3], (frame[4] << 8) | frame[5]
            if frame[1] == 0x06:
                self.regs[reg] = value
                if reg == 0x0100:
                    self.regs[0x0200] = 1
                if reg == 0x0103:
                    obj = self.object_permille
                    caught = obj is not None and value < obj
                    self.regs[0x0202] = obj if caught else value
                    self.regs[0x0201] = 2 if caught else 1
                return list(frame)
            if frame[1] == 0x03:
                v = self.regs.get(reg, 0)
                body = [frame[0], 0x03, 2, v >> 8, v & 0xFF]
                crc = crc16_modbus(bytes(body))
                return [*body, crc & 0xFF, crc >> 8]
            return []
