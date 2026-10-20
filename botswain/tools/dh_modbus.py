"""DH-Robotics electric grippers (PGE, PGI, AG ...) over Modbus RTU on an RS485 tool port.

Registers (checked against DH-Robotics/dh_gripper_ros dh_modbus_gripper.cpp): 0x0100 init (write 1, needed once
after power-up; it moves the jaws), 0x0101 closing force %, 0x0103 target position in permille (0 closed,
1000 open), 0x0200 init state (1 = done), 0x0201 grip state, 0x0202 current position.
A PGE has no force sensor: it limits motor current. Closing stops when the set force is reached and the grip
state then says "object caught", so "close until this force" is the force setting plus an ordinary close.

Proportional openings (VR trigger) are written by the poll thread, newest value only: a slow RS485 round trip
then never holds up the worker thread, which also streams the arm's servo targets.
"""

from __future__ import annotations

import logging
import threading
import time

from botswain.drivers.base import Check, DriverError
from botswain.tools.base import Tool, ToolCaps, ToolPort, ToolStatus

log = logging.getLogger(__name__)

REG_INIT = 0x0100
REG_FORCE = 0x0101
REG_POSITION = 0x0103
REG_INIT_STATE = 0x0200
REG_GRIP_STATE = 0x0201
REG_CURRENT_POS = 0x0202
GRIP_STATES = {0: "moving", 1: "at position", 2: "object caught", 3: "object dropped"}
POLL_S = 0.25  # status readout period
RETRY_S = 2.0  # after a failed read, try again this much later


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


class DhModbusTool(Tool):
    port_kind = "rs485"

    def __init__(self, slave: int = 1, force_range_pct: tuple[int, int] = (20, 100)) -> None:
        self.slave = slave
        self.caps = ToolCaps(open_close=True, force=True, force_range_pct=force_range_pct, proportional=True)
        self._port: ToolPort | None = None
        self._lock = threading.Lock()  # one frame at a time on the bus (commands vs the status poll)
        self._prepare_lock = threading.Lock()  # init + force once, whichever thread writes first
        self._ready = False  # initialised (by us, or found initialised)
        self._commanded: float | None = None
        self._opening_wanted: float | None = None  # newest proportional target, written by the poll thread
        self._wanted_lock = threading.Lock()
        self._force: int | None = None  # what the force register holds, as far as we know
        self._force_wanted: int | None = None  # set before init: written right after it
        self._meas: tuple[float | None, str] = (None, "")
        self._poll: threading.Thread | None = None
        self._stop = threading.Event()
        self._wake = threading.Event()  # a new opening to write, or stop

    # ---- the Tool contract ----

    def connect(self, port: ToolPort) -> None:
        self._port = port
        self._ready = False
        self._force = None
        self._opening_wanted = None
        self._stop.clear()
        self._poll = threading.Thread(target=self._poll_loop, name="tool-poll", daemon=True)
        self._poll.start()

    def disconnect(self) -> None:
        self._stop.set()
        self._wake.set()
        if self._poll is not None:
            self._poll.join(2.0)
            self._poll = None
        self._port = None
        self._meas = (None, "")

    def set_open(self, open: bool) -> None:
        self._take_opening()  # an open / close overrides a proportional target not written yet
        self._prepare()
        log.info("gripper %s reply: %s", "open" if open else "close", self._write(REG_POSITION, 1000 if open else 0))
        self._commanded = 1.0 if open else 0.0

    def set_opening(self, opening: float) -> None:
        if not 0.0 <= opening <= 1.0:
            raise DriverError(-1, f"gripper opening must be 0..1, got {opening}")
        if self._port is None:
            raise DriverError(-1, "gripper not connected", "press Connect")
        with self._wanted_lock:
            self._opening_wanted = float(opening)
        self._commanded = float(opening)
        self._wake.set()

    def set_force(self, pct: int) -> None:
        lo, hi = self.caps.force_range_pct
        if not lo <= pct <= hi:
            raise DriverError(-1, f"gripper force must be {lo}..{hi} %, got {pct}")
        self._force_wanted = int(pct)
        if self._ready:
            self._write_force(int(pct))

    def status(self) -> ToolStatus:
        opening, text = self._meas
        return ToolStatus(self._commanded, opening, text)

    def diagnose(self) -> list[Check]:
        if not self._ready:
            return [Check("Gripper force", "ok", "read after the first Open or Close (gripper not initialised)")]
        try:
            value = self.read(REG_FORCE)
        except DriverError as e:
            return [Check("Gripper force", "warn", e.message, e.hint)]
        sent = "" if self._force is None else f" (last set {self._force} %)"
        ok = self._force is None or value == self._force
        return [Check("Gripper force", "ok" if ok else "warn", f"register 0x0101 = {value} %{sent}",
                      "" if ok else "the gripper did not keep the force we set")]

    # ---- protocol ----

    def read(self, reg: int) -> int:
        """One register. Raises DriverError if the port does not hand back a valid reply."""
        reply = self._exchange(read_register_frame(self.slave, reg))
        value = parse_read_reply(reply, self.slave)
        if value is None:
            raise DriverError(-201, f"gripper read 0x{reg:04X}: no valid reply ({reply[:12]})",
                              "check the gripper's RS485 settings and slave id")
        return value

    def _write(self, reg: int, value: int) -> list[int]:
        return self._exchange(write_register_frame(self.slave, reg, value))

    def _take_opening(self) -> float | None:
        with self._wanted_lock:
            target, self._opening_wanted = self._opening_wanted, None
        return target

    def _prepare(self) -> None:
        """Initialise once (DH init moves the jaws) and write a force set before init."""
        with self._prepare_lock:
            if not self._ready:
                log.info("gripper init reply: %s", self._write(REG_INIT, 1))
                self._ready = True
            if self._force_wanted is not None and self._force_wanted != self._force:
                self._write_force(self._force_wanted)

    def _write_force(self, pct: int) -> None:
        log.info("gripper force %d %% reply: %s", pct, self._write(REG_FORCE, pct))
        self._force = pct

    def _exchange(self, frame: list[int]) -> list[int]:
        port = self._port
        if port is None:
            raise DriverError(-1, "gripper not connected", "press Connect")
        with self._lock:
            return port.exchange(frame)

    def _poll_loop(self) -> None:
        """Writes the newest proportional opening, and reads the status for the GUI every POLL_S.

        The status read never initialises (DH init moves the jaws); only a commanded opening does.
        """
        failed, next_read = False, 0.0
        while not self._stop.is_set():
            self._wake.wait(max(0.0, next_read - time.monotonic()))
            self._wake.clear()
            if self._stop.is_set():
                return
            target = self._take_opening()
            try:
                if target is not None:
                    self._prepare()
                    self._write(REG_POSITION, round(target * 1000))
                if time.monotonic() < next_read:
                    continue
                next_read = time.monotonic() + POLL_S
                if not self._ready:
                    if self.read(REG_INIT_STATE) != 1:
                        self._meas = (None, "not initialised: press Open or Close once")
                        continue
                    self._ready = True  # already initialised (e.g. by DobotStudio): no init motion needed
                state, pos = self.read(REG_GRIP_STATE), self.read(REG_CURRENT_POS)
                self._meas = (min(max(pos / 1000.0, 0.0), 1.0), GRIP_STATES.get(state, f"state {state}"))
                if failed:
                    log.info("gripper readout working again")
                failed = False
            except DriverError as e:
                if not failed:
                    log.warning("gripper readout failed (retrying every %.0f s): %s", RETRY_S, e.message)
                self._meas = (None, "no reading")
                failed, next_read = True, time.monotonic() + RETRY_S
