"""Game controller teleop (Xbox-style pads in XInput mode) as rate control.

Reading uses Windows' own xinput1_4.dll through ctypes, so there is no extra dependency. Pads with an
X-input / D-input switch (Cosmic Byte and similar) must be in X-input mode.

Mapping (robot base frame, turned by the same "facing" setting as the phone):
  hold LB      deadman: the arm moves only while it is held; release = stop
  left stick   X / Y          triggers    RT up, LT down (Z)
  right stick  Rz (x), Ry (y)  RB          slow mode (x0.25)
  A / B        gripper open / close        D-pad up/down   speed +/-10 %
  Start        record: start or save episode   Back   discard episode   X   save point

Sticks set a velocity; the target pose is integrated from the measured pose and goes to worker.stream(),
so the safety gate's teleop rules (step clamp, box, stale input, STOP latch) apply exactly as for the phone.
"""

from __future__ import annotations

import ctypes
import logging
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass, field

import numpy as np

from botswain.control.worker import RobotWorker
from botswain.core.clock import now_ns
from botswain.core.kinematics import axis_rotation, matrix_to_pose, pose_to_matrix
from botswain.core.types import Gripper, StreamTarget

log = logging.getLogger(__name__)

BUTTONS = {0x0001: "up", 0x0002: "down", 0x0004: "left", 0x0008: "right", 0x0010: "start", 0x0020: "back",
           0x0040: "ls", 0x0080: "rs", 0x0100: "lb", 0x0200: "rb", 0x1000: "a", 0x2000: "b", 0x4000: "x",
           0x8000: "y"}
STICK_DEADZONE = 0.15  # fraction of full travel ignored around the centre (cheap pads drift)
TRIGGER_DEADZONE = 0.08
MAX_LEAD_MM = 30.0  # the integrated target never runs further than this ahead of the measured pose
ACTIONS = ("start", "back", "x")  # buttons the GUI handles (recording, save point)


@dataclass(frozen=True)
class PadState:
    connected: bool
    lx: float = 0.0  # -1..1, right positive
    ly: float = 0.0  # -1..1, up positive
    rx: float = 0.0
    ry: float = 0.0
    lt: float = 0.0  # 0..1
    rt: float = 0.0
    buttons: frozenset[str] = field(default_factory=frozenset)


class _Pad(ctypes.Structure):
    _fields_ = [("wButtons", ctypes.c_ushort), ("bLeftTrigger", ctypes.c_ubyte), ("bRightTrigger", ctypes.c_ubyte),
                ("sThumbLX", ctypes.c_short), ("sThumbLY", ctypes.c_short), ("sThumbRX", ctypes.c_short),
                ("sThumbRY", ctypes.c_short)]


class _State(ctypes.Structure):
    _fields_ = [("dwPacketNumber", ctypes.c_uint), ("Gamepad", _Pad)]


def _stick(x: int, y: int) -> tuple[float, float]:
    """Radial deadzone, rescaled so the output still reaches 1.0 at full deflection."""
    v = np.array([x, y], dtype=float) / 32767.0
    mag = float(np.linalg.norm(v))
    if mag <= STICK_DEADZONE:
        return 0.0, 0.0
    v *= min(1.0, (mag - STICK_DEADZONE) / (1.0 - STICK_DEADZONE)) / mag
    return float(np.clip(v[0], -1, 1)), float(np.clip(v[1], -1, 1))


def _trigger(t: int) -> float:
    t = t / 255.0
    return 0.0 if t <= TRIGGER_DEADZONE else (t - TRIGGER_DEADZONE) / (1.0 - TRIGGER_DEADZONE)


class XInputReader:
    """Reads one XInput slot (0..3). find() picks the first connected pad."""

    def __init__(self, index: int | None = None) -> None:
        self._dll = ctypes.windll.xinput1_4 if hasattr(ctypes, "windll") else None
        self.index = index if index is not None else (self.find() or 0)

    def find(self) -> int | None:
        for i in range(4):
            if self._raw(i) is not None:
                return i
        return None

    def _raw(self, i: int) -> _State | None:
        if self._dll is None:
            return None
        s = _State()
        return s if self._dll.XInputGetState(i, ctypes.byref(s)) == 0 else None

    def __call__(self) -> PadState:
        s = self._raw(self.index)
        if s is None:
            return PadState(False)
        g = s.Gamepad
        lx, ly = _stick(g.sThumbLX, g.sThumbLY)
        rx, ry = _stick(g.sThumbRX, g.sThumbRY)
        pressed = frozenset(name for bit, name in BUTTONS.items() if g.wButtons & bit)
        return PadState(True, lx, ly, rx, ry, _trigger(g.bLeftTrigger), _trigger(g.bRightTrigger), pressed)


class GamepadTeleop:
    def __init__(self, worker: RobotWorker, reader: Callable[[], PadState], rate_hz: float = 50.0) -> None:
        self.worker = worker
        self.reader = reader
        self.period = 1.0 / rate_hz
        self.max_lin_mm_s = 100.0
        self.max_rot_deg_s = 30.0
        self.speed_scale = 0.5  # D-pad changes it, 0.1..1.0
        self.remap = np.eye(3)
        self.on_action: Callable[[str], None] | None = None  # "start" / "back" / "x", called on the pad thread
        self.pad = PadState(False)
        self.active = False  # deadman held and streaming
        self._prev = frozenset()
        self._target: np.ndarray | None = None
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    @property
    def running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def start(self) -> None:
        if self.running:
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="gamepad", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(1.0)
        if self.active:  # never leave the arm streaming
            self.worker.stream(self._release())

    def _run(self) -> None:
        last = time.perf_counter()
        while not self._stop.is_set():
            now = time.perf_counter()
            cmd = self.step(self.reader(), now - last)
            last = now
            if cmd is not None:
                self.worker.stream(cmd)
            time.sleep(max(0.0, self.period - (time.perf_counter() - now)))

    def step(self, pad: PadState, dt: float) -> StreamTarget | None:
        """One control tick: buttons, then the deadman and the velocity integration."""
        self.pad = pad
        pressed = pad.buttons - self._prev
        self._prev = pad.buttons
        for name in pressed:
            self._button(name)
        if not (pad.connected and "lb" in pad.buttons):
            return self._release() if self.active else None
        measured = pose_to_matrix(self.worker.latest_state().tcp)
        if not self.active:  # deadman just pressed: start exactly where the arm is
            self.active = True
            self._target = measured
            return StreamTarget(matrix_to_pose(self._target), now_ns(), "gamepad", True)
        gain = self.speed_scale * (0.25 if "rb" in pad.buttons else 1.0)
        v = self.remap @ np.array([pad.ly, -pad.lx, pad.rt - pad.lt]) * self.max_lin_mm_s * gain
        t = self._target.copy()
        t[:3, 3] += v * dt
        lead = t[:3, 3] - measured[:3, 3]
        dist = float(np.linalg.norm(lead))
        if dist > MAX_LEAD_MM:  # the arm is slower than the stick: wait for it instead of running away
            t[:3, 3] = measured[:3, 3] + lead * (MAX_LEAD_MM / dist)
        w = np.radians(self.max_rot_deg_s * gain) * dt
        t[:3, :3] = axis_rotation(np.array([0.0, 0.0, 1.0]), -pad.rx * w) @ axis_rotation(
            np.array([0.0, 1.0, 0.0]), pad.ry * w) @ t[:3, :3]
        self._target = t
        return StreamTarget(matrix_to_pose(t), now_ns(), "gamepad", True)

    def _release(self) -> StreamTarget:
        self.active = False
        self._target = None
        return StreamTarget(self.worker.latest_state().tcp, now_ns(), "gamepad", False)

    def _button(self, name: str) -> None:
        if name == "a":
            self.worker.submit(Gripper(True))
        elif name == "b":
            self.worker.submit(Gripper(False))
        elif name == "up":
            self.speed_scale = min(1.0, round(self.speed_scale + 0.1, 2))
        elif name == "down":
            self.speed_scale = max(0.1, round(self.speed_scale - 0.1, 2))
        elif name in ACTIONS and self.on_action is not None:
            self.on_action(name)
