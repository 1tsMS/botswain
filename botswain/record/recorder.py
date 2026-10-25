"""Fixed-rate recorder on the one clock (SPEC 5.12).

LeRobot computes timestamp = frame_index / fps, so we sample on a fixed grid and store the real arrival
times as extra features. Frame k always stands for time start + k / fps: if the loop is ever late it fills the
slots it missed with the newest sample (counted in late_frames) instead of skipping them, because a skipped
slot squeezes time and the dataset would play faster than reality (MS's first lab sets: 287 s stored as 169 s).
Per frame:
- observation.state       measured joints + TCP pose + gripper
- action                  what the arm was told to do, pose + gripper:
                            phone / gamepad / vr  the target sent AFTER the gate
                            drag                  the next measured pose (see below)
                            gui (home, points)    the controller's own target (also covers joint moves)
                            nothing (idle)        the measured pose: stay here
                          A stale teleop target is never used for a frame no input drives.
- observation.command_raw the input BEFORE the gate (shows clamping); the action itself when no teleop drives
- observation.controller_target  the controller's own target (Dobot ToolVectorTarget)
- observation.t_state_ns / t_command_ns / t_cam_<name>_ns  real arrival times (float64)
- observation.images.<name>  newest camera frame
Drag teach has no command: a drag frame's action (and command_raw) is the NEXT frame's measured TCP pose,
"go where the hand took the arm". So every frame is written one tick late; the last drag frame of an episode
gets its own pose (it stayed there).
- observation.input_source  which input is driving the arm at that frame, as a number (legend in the feature's
  names, so it travels with the dataset): 0 none (idle), 1 gui, 2 phone, 3 gamepad, 4 vr, 5 drag (by hand)
Angles (Rx Ry Rz in state, action, command_raw, controller_target) are unwrapped: no 360 deg jump between frames
(a tool pointing down sits at Rx = +-180 and the controller flips between the two). The first frame of every
episode takes the branch nearest the dataset's first frame, so all episodes agree. Values can leave -180..180;
wrap them back ((a + 180) % 360 - 180) before sending a policy's output to the robot.
"""

from __future__ import annotations

import logging
import threading
import time
from collections.abc import Callable
from typing import Protocol

import numpy as np

from botswain.control.worker import RobotWorker
from botswain.core.types import AXES_CART, RobotState

log = logging.getLogger(__name__)

SOURCE_CODES = {"": 0, "gui": 1, "phone": 2, "gamepad": 3, "vr": 4, "drag": 5}  # never renumber
SOURCE_NAMES = {v: k or "none" for k, v in SOURCE_CODES.items()}
SOURCE_LEGEND = "source (" + ", ".join(f"{v} {k or 'none'}" for k, v in SOURCE_CODES.items()) + ")"
NOT_STREAMING = ("", "gui", "drag")  # active_source() values that are not a teleop input streaming targets
ROT = slice(9, 12)  # Rx Ry Rz in observation.state (after 6 joints and X Y Z)
POSE_ROT = slice(3, 6)  # Rx Ry Rz in action / command_raw / controller_target


def unwrap_near(angles: np.ndarray, reference: np.ndarray) -> np.ndarray:
    """The same angles (deg) shifted by whole turns to lie within 180 deg of the reference."""
    return angles + 360.0 * np.round((reference - angles) / 360.0)


class Writer(Protocol):
    def add(self, frame: dict) -> None: ...
    def save(self) -> None: ...
    def discard(self) -> None: ...
    def finalize(self) -> None: ...


class FrameSource(Protocol):
    def latest(self) -> tuple[np.ndarray, int] | None: ...


def build_features(dof: int, cameras: dict[str, tuple[int, int]]) -> dict:
    joints = [f"J{i + 1}" for i in range(dof)]
    pose = list(AXES_CART)

    def vec(names: list[str], dtype: str = "float32") -> dict:
        return {"dtype": dtype, "shape": (len(names),), "names": names}

    features = {
        "observation.state": vec([*joints, *pose, "gripper"]),
        "action": vec([*pose, "gripper"]),
        "observation.command_raw": vec([*pose, "gripper"]),
        "observation.controller_target": vec(pose),
        "observation.t_state_ns": vec(["t"], "float64"),
        "observation.t_command_ns": vec(["t"], "float64"),
        "observation.input_source": vec([SOURCE_LEGEND]),
    }
    for name, (h, w) in cameras.items():
        features[f"observation.t_cam_{name}_ns"] = vec(["t"], "float64")
        features[f"observation.images.{name}"] = {"dtype": "video", "shape": (h, w, 3),
                                                  "names": ["height", "width", "channels"]}
    return features


class Recorder:
    def __init__(self, worker: RobotWorker, cameras: dict[str, FrameSource], fps: int, writer: Writer,
                 clock: Callable[[], float] = time.perf_counter) -> None:
        self.worker = worker
        self.cameras = cameras
        self.fps = fps
        self.writer = writer
        self._clock = clock
        self.frames = 0
        self.source_frames: dict[str, int] = {}  # input name -> frames in this episode
        self.task = ""
        self.error: str | None = None
        self._pending: tuple[dict, np.ndarray] | None = None  # (frame not written yet, its measured TCP pose)
        self.late_frames = 0  # grid slots filled with a repeated sample because the loop was late (this episode)
        self._rot_anchor: np.ndarray | None = None  # the dataset's first Rx Ry Rz: every episode's branch
        self._rot_prev: np.ndarray | None = None  # the previous written frame's unwrapped Rx Ry Rz
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    @property
    def recording(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def start_episode(self, task: str) -> None:
        if self.recording:
            raise RuntimeError("an episode is already being recorded")
        self.task = task
        self.frames = 0
        self.source_frames = {}
        self.late_frames = 0
        self._rot_prev = self._rot_anchor  # a new episode starts on the dataset's branch
        self.error = None
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="recorder", daemon=True)
        self._thread.start()

    def save_episode(self) -> int:
        self._end()
        self._flush()
        n = self.frames
        if n:
            self.writer.save()
        return n

    def discard_episode(self) -> None:
        self._end()
        self._pending = None
        self.writer.discard()
        self.frames = 0

    def close(self) -> None:
        if self.recording:
            self.save_episode()
        self.writer.finalize()  # LeRobot: without this the parquet footers are missing

    def _end(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(2.0)
            self._thread = None

    def _run(self) -> None:
        try:
            self._loop()
        except Exception as e:
            log.exception("recorder stopped")
            self.error = f"{type(e).__name__}: {e}"

    def _loop(self) -> None:
        period = 1.0 / self.fps
        start = self._clock()
        k = 0
        while not self._stop.is_set():
            target = start + k * period
            delay = target - self._clock()
            if delay > 0:
                time.sleep(delay)
            if self._stop.is_set():
                break
            s = self.worker.latest_state()
            frame, tcp = self._sample(s), np.array(s.tcp.as_list(), dtype=np.float32)
            self._emit(frame, tcp)
            k += 1
            behind = int((self._clock() - start) / period) - k
            if behind > 0:  # late: fill the slots we missed (time stays true) instead of skipping them
                for _ in range(behind):
                    self._emit({key: value.copy() if isinstance(value, np.ndarray) else value
                                for key, value in frame.items()}, tcp)
                self.late_frames += behind
                k += behind
                if self.late_frames == behind or self.late_frames // self.fps != (self.late_frames - behind) // self.fps:
                    log.warning("recorder late: %d slot(s) filled with a repeated sample (%d this episode)",
                                behind, self.late_frames)

    def _emit(self, frame: dict, tcp: np.ndarray) -> None:
        """Write the previous frame now that the next pose is known; keep this one back."""
        if self._pending is not None:
            self._write(self._pending[0], next_tcp=tcp)
        self._pending = (frame, tcp)

    def _flush(self) -> None:
        if self._pending is not None:
            frame, tcp = self._pending
            self._write(frame, next_tcp=tcp)  # last frame: no next pose, the arm stays where it is
            self._pending = None

    def _write(self, frame: dict, next_tcp: np.ndarray) -> None:
        if frame["observation.input_source"][0] == SOURCE_CODES["drag"]:
            for key in ("action", "observation.command_raw"):
                frame[key] = frame[key].copy()
                frame[key][:6] = next_tcp
        self._unwrap(frame)
        self.writer.add(frame)
        self.frames += 1
        name = SOURCE_NAMES.get(int(frame["observation.input_source"][0]), "none")
        self.source_frames[name] = self.source_frames.get(name, 0) + 1

    def _unwrap(self, frame: dict) -> None:
        """No 360 deg jumps: the state's angles follow the previous frame, the commanded ones follow the state."""
        state = frame["observation.state"] = frame["observation.state"].astype(np.float64)
        if self._rot_anchor is None:
            self._rot_anchor = self._rot_prev = state[ROT].copy()
        state[ROT] = unwrap_near(state[ROT], self._rot_prev)
        self._rot_prev = state[ROT].copy()
        for key in ("action", "observation.command_raw", "observation.controller_target"):
            v = frame[key] = frame[key].astype(np.float64)
            v[POSE_ROT] = unwrap_near(v[POSE_ROT], state[ROT])
            frame[key] = v.astype(np.float32)
        frame["observation.state"] = state.astype(np.float32)

    @property
    def main_source(self) -> str | None:
        """The input that drove this episode most, or None if nothing drove the arm."""
        driven = {k: v for k, v in self.source_frames.items() if k != "none"}
        return max(driven, key=driven.get) if driven else None

    def sample(self) -> dict:
        """One frame as it is now (drag frames still carry the placeholder action)."""
        return self._sample(self.worker.latest_state())

    def _sample(self, s: RobotState) -> dict:
        c = self.worker.commanded
        grip = s.gripper_pos if s.gripper_pos is not None else s.gripper  # measured when the gripper reports it
        grip_meas = grip if grip is not None else np.nan
        grip_cmd = np.nan if c.gripper is None else c.gripper
        source = self.worker.active_source(s)
        if source not in NOT_STREAMING and c.sent is not None and c.source == source:  # phone, gamepad, vr, ...
            sent, raw = c.sent, c.raw or c.sent  # the target this input is streaming, after / before the gate
        elif source == "gui":
            sent = raw = s.tcp_target  # the controller's own target: right for joint moves too (home)
        else:
            sent = raw = s.tcp  # idle (drag is replaced by the next measured pose in _write): stay here
        frame = {
            "observation.state": np.array([*s.joints.values, *s.tcp.as_list(), grip_meas], dtype=np.float32),
            "action": np.array([*sent.as_list(), grip_cmd], dtype=np.float32),
            "observation.command_raw": np.array([*raw.as_list(), grip_cmd], dtype=np.float32),
            "observation.controller_target": np.array(s.tcp_target.as_list(), dtype=np.float32),
            "observation.t_state_ns": np.array([s.t_ns], dtype=np.float64),
            "observation.t_command_ns": np.array([c.t_ns], dtype=np.float64),
            "observation.input_source": np.array([SOURCE_CODES.get(source, 0)], dtype=np.float32),
            "task": self.task,
        }
        for name, cam in self.cameras.items():
            latest = cam.latest()
            if latest is None:
                raise RuntimeError(f"camera {name} has no frame yet")
            image, t = latest
            frame[f"observation.images.{name}"] = image[:, :, ::-1]  # OpenCV BGR -> RGB
            frame[f"observation.t_cam_{name}_ns"] = np.array([t], dtype=np.float64)
        return frame
