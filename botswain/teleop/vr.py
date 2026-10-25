"""VR teleop (Meta Quest Browser, WebXR, passthrough): our own page, the right controller drives the arm.

The headset tracks the controllers (IR LEDs + IMU fused by the runtime) and the page sends each frame's
grip-space pose (the palm, not the pointing ray) with the buttons over the websocket:
    {"type": "vr", "right": hand, "left": hand, "head": {"pos", "quat"} | null}
    hand = {"tracked": false} or {"tracked": true, "pos": [x,y,z] m, "quat": [x,y,z,w], "grip": 0..1,
            "trigger": 0..1, "a": bool, "b": bool, "stick": [x, y]}
    {"type": "calib", "op": "add", "pos": [x,y,z]}                    left controller tip touches the tool tip
    {"type": "calib", "op": "place", "pos": [x,y,z], "quat": [x,y,z,w]}  tool tip here, base where it points
    {"type": "calib", "op": "set", "matrix": [16 floats, row-major]}   the pose the left hand dragged it to
    {"type": "calib", "op": "reset"}
    {"type": "log", "text": "...", "level": "error" | "info"}           written to Botswain's log
    {"type": "ui", "action": "home", "held": bool}                    controls window HOME (hold = go, let go = stop)
    {"type": "ui", "action": "box"}                                   controls window SAFETY BOX on / off
    {"type": "ui", "action": "record"}                                controls window RECORD: start, or save the episode
    {"type": "ui", "action": "discard"}                               controls window DISCARD (held 1 s on the page)
Placing (left trigger held): the page first asks for "place" (a starting pose) if there is none, then drags the
outline with the controller live and sends "set" when the trigger is let go.
Positions are WebXR local-floor (x right, y up, z back), like the phone, so TeleopMapper is reused as is.
The server sends a status 30 times a second (see VrServer.status): robot mode, link poses for the overlay, the
safety box, the calibration and the camera names. Camera frames are separate JPEG requests (/cam/0.jpg).

Controls (right hand): squeeze grip = clutch (hold to move; let go, move your hand, squeeze again to re-centre),
the arm follows the hand's X Y Z and, with rotation following on, the wrist's tilt about X and Y; thumbstick
left/right = turn the tool about Z (while the clutch is held); trigger = gripper, proportional (pull = close,
let go = open); A / B = motion scale down / up; stick click = wrist rotation following on / off. Lost tracking
or a closed page = clutch released at once, and after a reconnect the grip must be let go once before it moves
the arm again.
"""

from __future__ import annotations

import json
import logging
import math
from collections.abc import Callable, Sequence
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from fastapi.responses import FileResponse, JSONResponse, Response
from transforms3d.euler import euler2mat
from transforms3d.quaternions import quat2mat

from botswain.control.worker import RobotWorker
from botswain.core.clock import now_ns
from botswain.core.types import Gripper, GripperOpening, Pose, StreamTarget
from botswain.teleop.mapping import TeleopMapper, remap_about_z
from botswain.teleop.server import PairedServer

log = logging.getLogger(__name__)

ASSETS = Path(__file__).resolve().parents[2] / "assets" / "vr"
PAGE = ASSETS / "index.html"
VENDOR = {"three.module.min.js", "STLLoader.js"}  # three.js r170, MIT (assets/vr/vendor/THREE_LICENSE)
FONTS = {"IBMPlexSans-SemiBold.ttf", "IBMPlexMono-Medium.ttf"}  # assets/fonts, OFL
PORT = 4444
SOURCE = "vr"
# Hysteresis, so an analog value hovering at one threshold can't chatter. The page uses the same clutch numbers
# only for its haptic tick; this side decides.
CLUTCH_ON, CLUTCH_OFF = 0.6, 0.4
TRIGGER_ON, TRIGGER_OFF = 0.7, 0.3  # open / close only grippers
STICK_DEADBAND = 0.2
YAW_RATE_DEG_S = 45.0  # thumbstick fully over
MAX_DT_S = 0.05  # a gap in the samples must not turn into one big yaw step
GRIP_END = 0.05  # trigger below this = fully open, above 1 - GRIP_END = fully closed (a resting finger is not a grip)
GRIP_ARM_AT = 0.1  # per connection the gripper follows the trigger only once it was pulled: connecting never opens it
GRIP_STEP = 0.05  # smaller trigger changes are not sent (the jaws would only twitch)
GRIP_PERIOD_NS = 100_000_000  # at most 10 gripper targets a second
SCALE_STEP_PCT, SCALE_MIN_PCT, SCALE_MAX_PCT = 10, 10, 100
STATUS_HZ = 30.0
CAM_WIDTH = 480  # camera frames for the headset are scaled down to this width
MAX_CAMS = 4
MIN_CALIB_SPREAD_M = 0.10  # calibration points closer than this (sideways) give no usable heading

# WebXR is y-up (x right, z back). FLOOR is the same room with z up: x right, y forward, z up.
XR_TO_FLOOR = np.array([[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]])


@dataclass(frozen=True)
class Hand:
    position_m: tuple[float, float, float]
    quat_wxyz: tuple[float, float, float, float]
    grip: float
    trigger: float
    a: bool
    b: bool
    stick: tuple[float, float]
    stick_press: bool = False


@dataclass(frozen=True)
class VrSample:
    t_ns: int
    right: Hand | None  # None = not tracked (or no controller): never moves the arm
    left: Hand | None
    head_m: tuple[float, float, float] | None


def _finite(values: Sequence[float], what: str) -> list[float]:
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
                a=bool(d.get("a", False)), b=bool(d.get("b", False)),
                stick=(min(max(sx, -1.0), 1.0), min(max(sy, -1.0), 1.0)),
                stick_press=bool(d.get("stick_press", False)))


def parse_vr(msg: dict, t_ns: int) -> VrSample:
    """Raises KeyError/ValueError/TypeError on a malformed message."""
    head = msg.get("head")
    head_m = None
    if head:
        hx, hy, hz = _finite(head["pos"], "head")
        head_m = (hx, hy, hz)
    return VrSample(t_ns, parse_hand(msg.get("right")), parse_hand(msg.get("left")), head_m)


class VrSession:
    """VR samples -> clutch + mapping -> worker.stream(); trigger -> gripper; A/B -> motion scale.

    Runs on the VR server's thread.
    """

    def __init__(self, worker: RobotWorker, mapper: TeleopMapper) -> None:
        self.worker = worker
        self.mapper = mapper
        # Placed or calibrated: hand directions come from where the robot really stands in the room (hand toward
        # the robot = tool toward the robot, as the overlay shows), not from the GUI's facing setting.
        self.room_remap: np.ndarray | None = None
        self._manual_remap: np.ndarray | None = None  # the GUI's facing, given back when the calibration is reset
        self.clutch = False
        self.need_release = False  # after a reconnect: the grip must be let go once before it moves the arm
        self.last_sample: VrSample | None = None
        self._last_t_ns: int | None = None
        self._ab = (False, False)
        self._stick_was = False
        self._trigger_down = False  # open / close only grippers
        self._grip_armed = False
        self._grip_sent: float | None = None
        self._grip_sent_ns = 0
        worker.teleport_handlers[SOURCE] = mapper.request_reanchor

    @property
    def scale_pct(self) -> int:
        return round(self.mapper.scale_pos * 100)

    def set_room(self, base_to_xr: np.ndarray | None) -> None:
        """The calibration changed (None = reset). Takes effect on the next sample, re-anchored: no jump."""
        if base_to_xr is None:
            self.room_remap = None
            return
        if self.room_remap is None:
            self._manual_remap = self.mapper.remap
        self.room_remap = room_remap(base_to_xr)

    def on_sample(self, s: VrSample) -> None:
        self._apply_room()
        dt = 0.0 if self._last_t_ns is None else min(max((s.t_ns - self._last_t_ns) / 1e9, 0.0), MAX_DT_S)
        self._last_t_ns = s.t_ns
        self.last_sample = s
        hand = s.right
        self._update_clutch(hand)
        robot = self.worker.latest_state().tcp
        target = None
        if self.clutch:
            if abs(hand.stick[0]) > STICK_DEADBAND:  # stick right = clockwise seen from above = -Rz
                self.mapper.nudge_yaw(-hand.stick[0] * YAW_RATE_DEG_S * dt)
            target = self.mapper.update(hand.position_m, hand.quat_wxyz, True, 1.0, robot)
        else:
            self.mapper.update((0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0), False, 1.0, robot)  # drops the anchor
        # clutch=False still goes to the worker: that is how a release stops the stream.
        self.worker.stream(StreamTarget(target or robot, s.t_ns, SOURCE, self.clutch and target is not None))
        if hand is not None:
            self._speed_buttons(hand)
            self._gripper(hand.trigger, s.t_ns)

    def on_disconnect(self) -> None:
        """The page closed or the headset slept: release now instead of waiting for the stale timeout."""
        self.clutch = False
        self.need_release = True
        self._last_t_ns = None
        self._ab = (False, False)
        self._stick_was = False
        self._trigger_down = False
        self._grip_armed = False
        self._grip_sent = None
        self.worker.stream(StreamTarget(self.worker.latest_state().tcp, now_ns(), SOURCE, False))

    def _apply_room(self) -> None:
        remap = self.room_remap
        if remap is not None and self.mapper.remap is not remap:  # also undoes a GUI facing change meanwhile
            self.mapper.remap = remap
            self.mapper.request_reanchor()
        elif remap is None and self._manual_remap is not None:
            self.mapper.remap, self._manual_remap = self._manual_remap, None
            self.mapper.request_reanchor()

    def _update_clutch(self, hand: Hand | None) -> None:
        if hand is None:
            self.clutch = False  # tracking lost: let go at once, never move on an IMU-only guess
        elif self.need_release:
            self.need_release = hand.grip >= CLUTCH_OFF
            self.clutch = False
        else:
            self.clutch = hand.grip > (CLUTCH_OFF if self.clutch else CLUTCH_ON)

    def _speed_buttons(self, hand: Hand) -> None:
        """A / B step the motion scale; the stick click turns wrist rotation following on or off."""
        a_was, b_was = self._ab
        self._ab = (hand.a, hand.b)
        if hand.stick_press and not self._stick_was:
            self.mapper.scale_rot = 0.0 if self.mapper.scale_rot > 0.0 else 1.0
            self.mapper.request_reanchor()  # the next sample is the new reference: no sudden turn
        self._stick_was = hand.stick_press
        step = (-SCALE_STEP_PCT if hand.a and not a_was else 0) + (SCALE_STEP_PCT if hand.b and not b_was else 0)
        pct = min(max(self.scale_pct + step, SCALE_MIN_PCT), SCALE_MAX_PCT)
        if step and pct != self.scale_pct:
            self.mapper.scale_pos = pct / 100.0
            self.mapper.request_reanchor()  # a new scale must not make the target jump

    def _gripper(self, trigger: float, t_ns: int) -> None:
        tool = self.worker.tool
        if tool is None:
            return
        if not self._grip_armed:
            if trigger < GRIP_ARM_AT:
                return
            self._grip_armed = True
        if not tool.caps.proportional:
            pressed = trigger > (TRIGGER_OFF if self._trigger_down else TRIGGER_ON)
            if pressed != self._trigger_down:
                self.worker.submit(Gripper(not pressed))
            self._trigger_down = pressed
            return
        opening = 1.0 - min(max((trigger - GRIP_END) / (1.0 - 2 * GRIP_END), 0.0), 1.0)
        last = self._grip_sent
        if last is not None:
            if opening == last:
                return
            if abs(opening - last) < GRIP_STEP and opening not in (0.0, 1.0):
                return
            if t_ns - self._grip_sent_ns < GRIP_PERIOD_NS:
                return  # the next sample (~72 Hz) tries again
        self.worker.submit(GripperOpening(round(opening, 3)))
        self._grip_sent, self._grip_sent_ns = opening, t_ns


# ---- calibration: where the robot stands in the headset's room ----


def _yaw(theta: float) -> np.ndarray:
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _xr_from_base(rz: np.ndarray, t_floor: np.ndarray) -> np.ndarray:
    """4x4 that takes robot base coordinates (m, z up) into WebXR local-floor coordinates."""
    t = np.eye(4)
    t[:3, :3] = XR_TO_FLOOR.T @ rz
    t[:3, 3] = XR_TO_FLOOR.T @ t_floor
    return t


def fit_base_to_xr(base_m: Sequence[Sequence[float]], xr_m: Sequence[Sequence[float]]) -> tuple[np.ndarray, float]:
    """Robot base pose in the room from touched points: heading + position (4 DOF), least squares.

    Both frames are level (the headset's floor is gravity-aligned and the arm stands on a level table), so only
    the turn about the vertical and the offset are fitted: 2 points are enough and noise can't tilt the overlay.
    Returns (4x4 base -> XR, rms error in m). Raises ValueError if the points do not pin down the heading.
    """
    p = np.asarray(base_m, dtype=float)
    q = np.asarray(xr_m, dtype=float) @ XR_TO_FLOOR.T
    if len(p) < 2 or len(p) != len(q):
        raise ValueError("need at least 2 points")
    pc, qc = p - p.mean(axis=0), q - q.mean(axis=0)
    if np.max(np.linalg.norm(pc[:, :2], axis=1)) * 2 < MIN_CALIB_SPREAD_M:
        raise ValueError(f"move the arm at least {MIN_CALIB_SPREAD_M * 100:.0f} cm sideways between points")
    theta = math.atan2(float(np.sum(pc[:, 0] * qc[:, 1] - pc[:, 1] * qc[:, 0])),
                       float(np.sum(pc[:, 0] * qc[:, 0] + pc[:, 1] * qc[:, 1])))
    rz = _yaw(theta)
    t = q.mean(axis=0) - rz @ p.mean(axis=0)
    rms = float(np.sqrt(np.mean(np.sum((p @ rz.T + t - q) ** 2, axis=1))))
    return _xr_from_base(rz, t), rms


def room_remap(base_to_xr: np.ndarray) -> np.ndarray:
    """TeleopMapper.remap that moves the tool the way the hand moves in the room, for this robot placement.

    The mapper turns WebXR motion into FLU (x forward, y left, z up); FLOOR is (x right, y forward, z up), so
    FLOOR = Rz(90) FLU, and base = Rz(-heading) FLOOR: remap = Rz(90 - heading).
    """
    rz = XR_TO_FLOOR @ np.asarray(base_to_xr, dtype=float)[:3, :3]
    heading = math.degrees(math.atan2(rz[1, 0], rz[0, 0]))
    return remap_about_z(90.0 - heading)


def place_base(pos_xr: Sequence[float], quat_xyzw: Sequence[float], tip_base_m: Sequence[float]) -> np.ndarray:
    """Quick placement from one touch: the controller's dot is on the tool tip (wherever the arm is now, e.g. home),
    and it points from there toward the robot's base. So the base lands behind the tip, reaching back at you.

    The base centre itself can't be touched on a real arm, the tool tip can. Level only (heading + position);
    touch calibration (X) refines it.
    """
    x, y, z, w = (float(v) for v in quat_xyzw)
    forward = XR_TO_FLOOR @ quat2mat((w, x, y, z)) @ np.array([0.0, 0.0, -1.0])  # the ray points along -z
    if math.hypot(forward[0], forward[1]) < 1e-3:
        raise ValueError("point the controller at the robot's base, not straight up or down")
    tip = np.asarray(tip_base_m, dtype=float)
    if math.hypot(tip[0], tip[1]) < 0.05:  # tip right above the base: no tip-to-base direction to line up
        raise ValueError("move the tool away from above the base first")
    aim = math.atan2(forward[1], forward[0])  # where the controller points, in the room
    tip_to_base = math.atan2(-tip[1], -tip[0])  # the same direction, in the robot's frame
    rz = _yaw(aim - tip_to_base)
    return _xr_from_base(rz, XR_TO_FLOOR @ np.asarray(pos_xr, dtype=float) - rz @ tip)


def tool_tip_m(tcp: Pose, offset_mm: Sequence[float] | None) -> np.ndarray:
    """The touched point in the base frame, in m: the TCP the controller reports (the flange with Tool 0) plus the
    end effector's tcp_offset_mm when it is set."""
    p = np.array(tcp.xyz, dtype=float)
    if offset_mm is not None:
        p = p + euler2mat(*np.radians(tcp.rpy), axes="sxyz") @ np.asarray(offset_mm, dtype=float)
    return p / 1000.0


class Calibration:
    """Lives in the server, not the page: the fit is plain numpy and tested here. Lost when the server stops, and
    stale when the headset re-centres (its local-floor origin moves): Y on the left controller resets it."""

    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self.base_points: list[np.ndarray] = []
        self.xr_points: list[np.ndarray] = []
        self.matrix: np.ndarray | None = None  # base -> XR
        self.rms_mm: float | None = None
        self.note = "not calibrated"

    def add(self, base_m: np.ndarray, xr_m: Sequence[float]) -> None:
        self.base_points.append(np.asarray(base_m, dtype=float))
        self.xr_points.append(np.asarray(_finite(xr_m, "pos"), dtype=float))
        if len(self.base_points) < 2:
            self.note = "point 1 set: move the arm, touch again"
            return
        try:
            self.matrix, rms = fit_base_to_xr(self.base_points, self.xr_points)
        except ValueError as e:
            self.note = str(e)
            return
        self.rms_mm = rms * 1000.0
        self.note = f"{len(self.base_points)} points, error {self.rms_mm:.0f} mm"

    def place(self, pos_xr: Sequence[float], quat_xyzw: Sequence[float], tip_base_m: Sequence[float]) -> None:
        try:
            matrix = place_base(_finite(pos_xr, "pos"), _finite(quat_xyzw, "quat"), tip_base_m)
        except ValueError as e:
            self.note = str(e)
            return
        self.reset()
        self.matrix = matrix
        self.note = "placed at the tip (X touches refine it)"

    def set_matrix(self, rows: Sequence[float]) -> None:
        """A pose dragged into place by hand. Must be a rigid, level transform (robot +Z = room up)."""
        m = np.asarray(_finite(rows, "matrix"), dtype=float).reshape(4, 4)
        r = m[:3, :3]
        if not np.allclose(r.T @ r, np.eye(3), atol=1e-3) or np.linalg.det(r) < 0.0 \
                or not np.allclose(m[3], (0.0, 0.0, 0.0, 1.0)):
            raise ValueError("not a rigid transform")
        if r[1, 2] < 0.999:  # column z, its WebXR y (up) component
            raise ValueError("robot must stand upright")
        self.reset()
        self.matrix = m
        self.note = "placed by hand (X touches refine it)"


# ---- server ----


def _rows(m: np.ndarray) -> list[float]:
    """Row-major, for THREE.Matrix4.set()."""
    return [round(float(v), 5) for v in m.reshape(-1)]


class VrServer(PairedServer):
    """No pairing key (typed in a headset it is painful); one headset at a time, the newest wins."""

    def __init__(self, session: VrSession, port: int = PORT, host: str = "0.0.0.0", urdf: Path | None = None,
                 tool_offset_mm: Sequence[float] | None = None,
                 cameras: Callable[[], list] = list, on_ui: Callable[[str, bool], None] | None = None,
                 home_ready: bool = False, recording: Callable[[], dict | None] | None = None) -> None:
        """on_ui(action, held): the controls window's buttons. Called on the server thread: the GUI side must hop to
        its own thread (TeleopPage does, with a queued signal). home_ready: the robot YAML has a confirmed home."""
        self.session = session
        self.on_ui = on_ui
        self.home_ready = home_ready
        self.recording = recording  # -> {"on", "frames", "saved"} for the headset, None = recording unavailable
        self.worker = session.worker
        self.samples = 0
        self.bad_messages = 0
        self.calibration = Calibration()
        self._page_log_ns = 0
        self.tool_offset_mm = tool_offset_mm
        self.cameras = cameras
        self._model = None
        self._meshes: list[bytes] = []
        self._base_mesh: list[bool] = []
        self._links_for: tuple[float, ...] | None = None
        self._links: list[list[float]] = []
        if urdf is not None and urdf.exists():
            self._load_model(urdf)
        super().__init__(PAGE.read_text(encoding="utf-8"), self._handle, port, host, path="/", tab="VR",
                         on_disconnect=self._disconnected, require_key=False, one_client=True,
                         outgoing=self.status_text, outgoing_hz=STATUS_HZ)
        self._add_routes()

    def _load_model(self, urdf: Path) -> None:
        from trimesh.exchange.stl import export_stl

        from botswain.core.robot_model import RobotModel

        try:
            self._model = RobotModel(urdf)  # own instance: the Jog page's one lives on the GUI thread
            for node, mesh in self._model.meshes().items():
                self._meshes.append(export_stl(mesh))
                self._base_mesh.append(node.startswith("base"))
        except Exception:  # no overlay is better than no VR
            log.exception("VR: robot model failed, the headset shows no robot outline")
            self._model, self._meshes, self._base_mesh = None, [], []

    def _add_routes(self) -> None:
        app = self.app

        @app.get("/vendor/{name}")
        def vendor(name: str) -> Response:
            if name not in VENDOR:
                return Response(status_code=404)
            return FileResponse(ASSETS / "vendor" / name, media_type="text/javascript")

        @app.get("/font/{name}")
        def font(name: str) -> Response:
            if name not in FONTS:
                return Response(status_code=404)
            return FileResponse(ASSETS.parents[0] / "fonts" / name, media_type="font/ttf")

        @app.get("/robot.json")
        def robot() -> JSONResponse:
            return JSONResponse({"meshes": [{"url": f"/mesh/{i}.stl", "base": b} for i, b in enumerate(self._base_mesh)],
                                 "tip": "gripper tip" if self.tool_offset_mm is not None else "flange centre"})

        @app.get("/mesh/{i}.stl")
        def mesh(i: int) -> Response:
            if not 0 <= i < len(self._meshes):
                return Response(status_code=404)
            return Response(self._meshes[i], media_type="model/stl", headers={"Cache-Control": "max-age=3600"})

        @app.get("/cam/{i}.jpg")
        def cam(i: int) -> Response:
            jpeg = self.camera_jpeg(i)
            if jpeg is None:
                return Response(status_code=404)
            return Response(jpeg, media_type="image/jpeg", headers={"Cache-Control": "no-store"})

    def camera_jpeg(self, i: int) -> bytes | None:
        import cv2

        cams = self.cameras()[:MAX_CAMS]
        latest = cams[i].latest() if 0 <= i < len(cams) else None
        if latest is None:
            return None
        frame = latest[0]
        if frame.shape[1] > CAM_WIDTH:
            frame = cv2.resize(frame, (CAM_WIDTH, round(frame.shape[0] * CAM_WIDTH / frame.shape[1])),
                               interpolation=cv2.INTER_AREA)
        ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
        return buf.tobytes() if ok else None

    def _handle(self, text: str) -> None:
        try:
            msg = json.loads(text)
            kind = msg.get("type")
            if kind == "calib":
                self._calib(msg)
                return
            if kind == "log":
                self._page_log(str(msg.get("text", "")), msg.get("level") == "info")
                return
            if kind == "ui":
                self._ui(msg)
                return
            if kind != "vr":
                return
            sample = parse_vr(msg, now_ns())
        except (ValueError, KeyError, TypeError, AttributeError):
            self.bad_messages += 1
            return
        self.samples += 1
        self.session.on_sample(sample)

    def _page_log(self, text: str, info: bool = False) -> None:
        """From the headset page (it has no console you can read). Errors at most one a second; notes (re-centres,
        anchor corrections) are rare events the page throttles itself."""
        if info:
            log.info("VR page: %s", text[:400])
            return
        t = now_ns()
        if t - self._page_log_ns >= 1_000_000_000:
            self._page_log_ns = t
            log.warning("VR page error: %s", text[:800])

    def _ui(self, msg: dict) -> None:
        action = msg.get("action")
        if action not in ("home", "box", "record", "discard"):
            raise ValueError(f"unknown ui action {action!r}")
        held = bool(msg.get("held", True))
        log.info("VR controls window: %s%s", action, "" if action != "home" else (" pressed" if held else " released"))
        if self.on_ui is not None:
            self.on_ui(action, held)

    def _disconnected(self) -> None:
        self.session.on_disconnect()
        if self.on_ui is not None:
            self.on_ui("home", False)  # a HOME hold ends with the page: the arm must not keep driving home

    def _calib(self, msg: dict) -> None:
        try:
            self._calib_op(msg)
        except (ValueError, KeyError, TypeError) as e:
            log.warning("VR calibration %r refused: %s", msg.get("op"), e)
            raise

    def _calib_op(self, msg: dict) -> None:
        op = msg.get("op")
        if op == "reset":
            self.calibration.reset()
        elif op == "add":
            self.calibration.add(tool_tip_m(self.worker.latest_state().tcp, self.tool_offset_mm), msg["pos"])
        elif op == "set":
            self.calibration.set_matrix(msg["matrix"])
        elif op == "place":
            self.calibration.place(msg["pos"], msg["quat"],
                                   tool_tip_m(self.worker.latest_state().tcp, self.tool_offset_mm))
        else:
            raise ValueError(f"unknown calibration op {op!r}")
        self.session.set_room(self.calibration.matrix)
        log.info("VR calibration: %s", self.calibration.note)

    def status(self) -> dict:
        st = self.worker.latest_state()
        gate = self.worker.gate
        cal = self.calibration
        return {
            "type": "status",
            "mode": st.mode.value, "link": st.link_ok, "fault": st.fault,
            "owner": self.worker.stream_owner, "clutch": self.session.clutch,
            "need_release": self.session.need_release, "scale": self.session.scale_pct,
            "rotation": self.session.mapper.scale_rot > 0.0,
            "room_directions": self.session.room_remap is not None,
            "grip_cmd": st.gripper, "grip_pos": st.gripper_pos, "grip_text": st.gripper_status,
            "links": self._link_rows(st.joints.values),
            "tip": [round(float(v), 4) for v in tool_tip_m(st.tcp, self.tool_offset_mm)],
            "box": [[lo / 1000.0, hi / 1000.0] for lo, hi in gate.limits.box_mm], "box_on": gate.box_enabled,
            "calib": {"matrix": None if cal.matrix is None else _rows(cal.matrix), "points": len(cal.base_points),
                      "note": cal.note},
            "cams": [c.name for c in self.cameras()[:MAX_CAMS]],
            "home_ready": self.home_ready,
            "rec": self.recording() if self.recording is not None else None,
        }

    def status_text(self) -> str | None:
        try:
            return json.dumps(self.status(), allow_nan=False)
        except ValueError:  # a NaN somewhere (no reading yet): skip this one
            return None

    def _link_rows(self, joints: tuple[float, ...]) -> list[list[float]]:
        if self._model is None:
            return []
        if joints != self._links_for:  # FK only when the arm moved
            transforms = self._model.transforms(joints)
            self._links = [_rows(transforms[node]) for node in self._model.nodes]
            self._links_for = joints
        return self._links
