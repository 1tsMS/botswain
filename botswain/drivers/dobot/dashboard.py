"""Dashboard port (29999) client: one request, one reply, ASCII.

Reply format: `ErrorID,{values},Echo(...);`. ErrorID 0 means "accepted", not "done".
TCP is a byte stream, so a reply can arrive in pieces or two replies can arrive in one read. We keep a byte
buffer, cut at ';', and check the echoed command name: a late reply to an earlier (timed-out) command is
logged and dropped, never handed back as the answer to the current one.
"""

from __future__ import annotations

import logging
import re
import socket
import threading
import time
from dataclasses import dataclass

from botswain.drivers.base import DriverError

log = logging.getLogger(__name__)

NOT_TCP = -100  # controller answered "Not Tcp": it is not in TCP/IP control mode
NO_LINK = -101  # socket not connected or dropped
TIMEOUT = -102  # no reply in time
BAD_REPLY = -103  # reply we could not parse

HINT_TCP = ("switch the controller to TCP/IP mode in DobotStudio Pro and close DobotStudio, "
            "or use RequestControl() on firmware >= 4.6")

ERROR_TEXT: dict[int, tuple[str, str]] = {
    -1: ("command failed", "check the robot state and the log"),
    -2: ("robot in alarm", "Clear error, then check the alarm list in the Doctor"),
    -3: ("emergency stop pressed", "release the e-stop, then Clear error"),
    -4: ("robot powered off", "power the arm on from the pendant or DobotStudio"),
    -5: ("a script is running", "stop the script in DobotStudio first"),
    -6: ("MoveJog axis does not match coordtype", "joint axes need coordtype 0, Cartesian 1 or 2"),
    -7: ("script paused", "stop or resume the script in DobotStudio"),
    -10000: ("unknown command", "the firmware may be older than this command"),
    -20000: ("wrong number of parameters", "this is a Botswain bug, please report it"),
    NOT_TCP: ("controller is not in TCP/IP mode", HINT_TCP),
}


def describe(code: int) -> tuple[str, str]:
    """Error text + fix hint. -3000X/-4000X/-5000X/-6000X are type/range errors on parameter X."""
    if code in ERROR_TEXT:
        return ERROR_TEXT[code]
    if -69999 <= code <= -30000:
        return (f"parameter {abs(code) % 10} has a wrong type or is out of range", "Botswain bug, please report it")
    return (f"error {code}", "see the Dobot TCP/IP protocol doc")


@dataclass(frozen=True)
class Reply:
    error_id: int
    values: str
    echo: str

    def numbers(self) -> list[float]:
        return [float(x) for x in re.findall(r"-?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?", self.values)]


# ErrorID, then '{'. The lab CR5 (V3 firmware) refuses some commands with an extra number before the braces
# and an empty echo: '-40001,0,{},;' followed by the echo on its own, 'MoveJog(Y+,1);'.
_HEAD = re.compile(r"\s*(-?\d+)\s*,(?:\s*-?\d+\s*,)*\s*\{")
# A bare echo with no ErrorID: the second half of the split reply above. Never a reply on its own.
_BARE_ECHO = re.compile(r"\s*[A-Za-z]\w*\(.*\)\s*;?\s*$", re.DOTALL)


def parse_reply(text: str) -> Reply:
    t = text.strip()
    t = t.removesuffix(";")
    m = _HEAD.match(t)
    if m is None:
        raise ValueError(f"not a dashboard reply: {text!r}")
    start = m.end() - 1
    depth = 0
    for i in range(start, len(t)):  # values can nest: GetErrorID() returns {[[22],[],...]}
        if t[i] == "{":
            depth += 1
        elif t[i] == "}":
            depth -= 1
            if depth == 0:
                return Reply(int(m.group(1)), t[start + 1:i], t[i + 1:].lstrip(" ,").strip())
    raise ValueError(f"unbalanced braces in reply: {text!r}")


def command_name(cmd: str) -> str:
    return cmd.split("(", 1)[0].strip().lower()


class DashboardClient:
    def __init__(self, ip: str, port: int, timeout_s: float = 2.0) -> None:
        self.ip = ip
        self.port = port
        self.timeout_s = timeout_s
        self.last_rtt_ms: float | None = None
        self._sock: socket.socket | None = None
        self._buf = b""
        self._lock = threading.Lock()

    @property
    def connected(self) -> bool:
        return self._sock is not None

    def connect(self) -> None:
        try:
            sock = socket.create_connection((self.ip, self.port), timeout=self.timeout_s)
        except OSError as e:
            raise DriverError(NO_LINK, f"cannot connect to {self.ip}:{self.port} ({e})",
                              "is the controller on, the cable in, and the laptop on 192.168.5.x?") from e
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self._sock = sock
        self._buf = b""

    def close(self) -> None:
        with self._lock:
            self._drop()

    def call(self, cmd: str, timeout_s: float | None = None) -> Reply:
        """Send one command and return its reply (any ErrorID). Raises DriverError on link problems only."""
        name = command_name(cmd)
        with self._lock:
            if self._sock is None:
                raise DriverError(NO_LINK, f"{name}: dashboard not connected", "press Connect")
            t0 = time.perf_counter()
            deadline = t0 + (timeout_s or self.timeout_s)
            try:
                self._sock.sendall((cmd + "\n").encode("ascii"))
            except OSError as e:
                self._drop()
                raise DriverError(NO_LINK, f"connection lost while sending {name}", "check the Ethernet cable") from e
            while True:
                text = self._take_reply()
                if text is not None:
                    reply = self._to_reply(text, name)
                    if reply is not None:
                        self.last_rtt_ms = (time.perf_counter() - t0) * 1000.0
                        return reply
                    continue
                remaining = deadline - time.perf_counter()
                if remaining <= 0:
                    raise DriverError(TIMEOUT, f"no reply to {name} within {timeout_s or self.timeout_s:.1f} s",
                                      "controller busy, or the link is down")
                self._recv(remaining)

    def call_ok(self, cmd: str, timeout_s: float | None = None) -> Reply:
        """Like call(), but any ErrorID other than 0 raises DriverError with the meaning and a hint."""
        reply = self.call(cmd, timeout_s)
        if reply.error_id != 0:
            text, hint = describe(reply.error_id)
            raise DriverError(reply.error_id, f"{command_name(cmd)} refused: {text}", hint)
        return reply

    # ---- internals (lock held) ----

    def _recv(self, timeout: float) -> None:
        assert self._sock is not None
        self._sock.settimeout(timeout)
        try:
            chunk = self._sock.recv(4096)
        except TimeoutError:
            return
        except OSError as e:
            self._drop()
            raise DriverError(NO_LINK, f"connection lost ({e})", "check the Ethernet cable") from e
        if not chunk:
            self._drop()
            raise DriverError(NO_LINK, "controller closed the dashboard connection", "reconnect")
        self._buf += chunk

    def _take_reply(self) -> str | None:
        """Cut the first complete reply off the buffer, or None if it is not complete yet."""
        semi = self._buf.find(b";")
        not_tcp = self._buf.lower().find(b"not tcp")
        if not_tcp != -1 and (semi == -1 or not_tcp < semi):
            # A controller in local mode answers "Not Tcp", possibly without the ';' ending.
            cut = semi + 1 if semi != -1 else not_tcp + len(b"not tcp")
        elif semi != -1:
            cut = semi + 1
        else:
            return None
        text, self._buf = self._buf[:cut], self._buf[cut:]
        return text.decode("ascii", "replace").strip()

    def _to_reply(self, text: str, name: str) -> Reply | None:
        if "not tcp" in text.lower():
            return Reply(NOT_TCP, "", text)
        if _BARE_ECHO.match(text):
            log.debug("dropped a bare echo %r while waiting for %s", text, name)
            return None
        try:
            reply = parse_reply(text)
        except ValueError as e:
            raise DriverError(BAD_REPLY, f"{name}: {e}", "this is a Botswain bug or a firmware change") from e
        if reply.echo and command_name(reply.echo) != name:
            log.warning("dropped a late reply %r while waiting for %s", text, name)
            return None
        return reply

    def _drop(self) -> None:
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
        self._sock = None
        self._buf = b""
