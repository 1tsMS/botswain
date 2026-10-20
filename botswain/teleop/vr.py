"""VR teleop (Meta Quest Browser, WebXR): our own page, the right controller drives the arm.

The headset tracks the controllers (IR LEDs + IMU fused by the runtime) and the page sends each frame's
grip-space pose (the palm, not the pointing ray) with the buttons over the paired websocket:
    {"type": "vr", "right": hand, "left": hand, "head": {"pos", "quat"} | null}
    hand = {"tracked": false} or {"tracked": true, "pos": [x,y,z] m, "quat": [x,y,z,w], "grip": 0..1,
            "trigger": 0..1, "a": bool, "b": bool, "stick": [x, y]}
Positions are WebXR local-floor (x right, y up, z back), like the phone, so TeleopMapper is reused as is.

Controls (right hand): squeeze grip = clutch (hold to move; let go, move your hand, squeeze again to re-centre),
trigger press = gripper open/close toggle. Lost tracking or a closed page = clutch released at once.
"""

from __future__ import annotations

import json
import logging
import math
from dataclasses import dataclass
from pathlib import Path

from botswain.control.worker import RobotWorker
from botswain.core.clock import now_ns
from botswain.core.types import Gripper, StreamTarget
from botswain.teleop.mapping import TeleopMapper
from botswain.teleop.server import PairedServer

log = logging.getLogger(__name__)

PAGE = Path(__file__).resolve().parents[2] / "assets" / "vr" / "index.html"
PORT = 4444
SOURCE = "vr"
# Hysteresis, so an analog value hovering at one threshold can't chatter. The page uses the same clutch numbers
# only for its haptic tick; this side decides.
CLUTCH_ON, CLUTCH_OFF = 0.6, 0.4
TRIGGER_ON, TRIGGER_OFF = 0.7, 0.3


@dataclass(frozen=True)
class Hand:
    position_m: tuple[float, float, float]
    quat_wxyz: tuple[float, float, float, float]
    grip: float
    trigger: float
    a: bool
    b: bool
    stick: tuple[float, float]


@dataclass(frozen=True)
class VrSample:
    t_ns: int
    right: Hand | None  # None = not tracked (or no controller): never moves the arm
    left: Hand | None
    head_m: tuple[float, float, float] | None


def _finite(values: list[float], what: str) -> list[float]:
    out = [float(v) for v in values]
    if not all(math.isfinite(v) for v in out):
        raise ValueError(f"{what} contains NaN or infinity")
    return out


def parse_hand(d: dict | None) -> Hand | None:
    if not d or not d.get("tracked"):
        return None
    pos = _finite(d["pos"], "pos")
    qx, qy, qz, qw = _finite(d["quat"], "quat")
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if len(pos) != 3 or norm < 1e-6:
        raise ValueError("bad pose")
    grip, trigger, sx, sy = _finite([d.get("grip", 0.0), d.get("trigger", 0.0), *d.get("stick", (0.0, 0.0))[:2]],
                                    "buttons")
    return Hand(position_m=(pos[0], pos[1], pos[2]), quat_wxyz=(qw / norm, qx / norm, qy / norm, qz / norm),
                grip=min(max(grip, 0.0), 1.0), trigger=min(max(trigger, 0.0), 1.0),
                a=bool(d.get("a", False)), b=bool(d.get("b", False)), stick=(sx, sy))


def parse_vr(msg: dict, t_ns: int) -> VrSample:
    """Raises KeyError/ValueError/TypeError on a malformed message."""
    head = msg.get("head")
    head_m = None
    if head:
        hx, hy, hz = _finite(head["pos"], "head")
        head_m = (hx, hy, hz)
    return VrSample(t_ns, parse_hand(msg.get("right")), parse_hand(msg.get("left")), head_m)


class VrSession:
    """VR samples -> clutch + mapping -> worker.stream(); trigger -> gripper. Runs on the VR server's thread."""

    def __init__(self, worker: RobotWorker, mapper: TeleopMapper) -> None:
        self.worker = worker
        self.mapper = mapper
        self.clutch = False
        self.last_sample: VrSample | None = None
        self._trigger_down = False
        worker.teleport_handlers[SOURCE] = mapper.request_reanchor

    def on_sample(self, s: VrSample) -> None:
        self.last_sample = s
        hand = s.right
        if hand is None:
            self.clutch = False  # tracking lost: let go at once, never move on an IMU-only guess
        else:
            self.clutch = hand.grip > (CLUTCH_OFF if self.clutch else CLUTCH_ON)
        robot = self.worker.latest_state().tcp
        if hand is None:
            target = self.mapper.update((0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0), False, 1.0, robot)
        else:
            target = self.mapper.update(hand.position_m, hand.quat_wxyz, self.clutch, 1.0, robot)
        # clutch=False still goes to the worker: that is how a release stops the stream.
        self.worker.stream(StreamTarget(target or robot, s.t_ns, SOURCE, self.clutch and target is not None))
        if hand is not None:
            pressed = hand.trigger > (TRIGGER_OFF if self._trigger_down else TRIGGER_ON)
            if pressed and not self._trigger_down:
                g = self.worker.latest_state().gripper  # toggle from the robot's state, like the gamepad's A
                self.worker.submit(Gripper(g is None or g < 0.5))
            self._trigger_down = pressed

    def on_disconnect(self) -> None:
        """The page closed or the headset slept: release now instead of waiting for the stale timeout."""
        self.clutch = False
        self._trigger_down = False
        self.worker.stream(StreamTarget(self.worker.latest_state().tcp, now_ns(), SOURCE, False))


class VrServer(PairedServer):
    def __init__(self, session: VrSession, port: int = PORT, host: str = "0.0.0.0") -> None:
        self.session = session
        self.samples = 0
        self.bad_messages = 0
        super().__init__(PAGE.read_text(encoding="utf-8"), self._handle, port, host, path="/", tab="VR",
                         on_disconnect=session.on_disconnect)

    def _handle(self, text: str) -> None:
        try:
            msg = json.loads(text)
            if msg.get("type") != "vr":
                return
            sample = parse_vr(msg, now_ns())
        except (ValueError, KeyError, TypeError):
            self.bad_messages += 1
            return
        self.samples += 1
        self.session.on_sample(sample)

