"""Fixed-rate recorder on the one clock (SPEC 5.12).

LeRobot computes timestamp = frame_index / fps, so we sample on a fixed grid and store the real arrival
times as extra features. Per frame:
- observation.state       measured joints + TCP pose + gripper
- action                  the commanded target AFTER the gate (pose + gripper): what we really sent
- observation.command_raw the input BEFORE the gate (shows clamping)
- observation.controller_target  the controller's own target (Dobot ToolVectorTarget)
- observation.t_state_ns / t_command_ns / t_cam_<name>_ns  real arrival times (float64)
- observation.images.<name>  newest camera frame
Drag teach has no command: a drag frame's action (and command_raw) is the NEXT frame's measured TCP pose,
"go where the hand took the arm". So every frame is written one tick late; the last drag frame of an episode
gets its own pose (it stayed there).
- observation.input_source  which input is driving the arm at that frame, as a number (legend in the feature's
  names, so it travels with the dataset): 0 none (idle), 1 gui, 2 phone, 3 gamepad, 4 vr, 5 drag (by hand)
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
            self._emit(self._sample(s), np.array(s.tcp.as_list(), dtype=np.float32))
            k += 1
            behind = int((self._clock() - start) / period) - k
            if behind > 0:  # a slow frame: skip grid slots instead of bunching frames together
                log.warning("recorder fell %d frame(s) behind", behind)
                k += behind

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
        self.writer.add(frame)
        self.frames += 1
        name = SOURCE_NAMES.get(int(frame["observation.input_source"][0]), "none")
        self.source_frames[name] = self.source_frames.get(name, 0) + 1

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
        sent = c.sent or s.tcp_target  # nothing commanded yet: the controller's own target
        raw = c.raw or sent
        frame = {
            "observation.state": np.array([*s.joints.values, *s.tcp.as_list(), grip_meas], dtype=np.float32),
            "action": np.array([*sent.as_list(), grip_cmd], dtype=np.float32),
            "observation.command_raw": np.array([*raw.as_list(), grip_cmd], dtype=np.float32),
            "observation.controller_target": np.array(s.tcp_target.as_list(), dtype=np.float32),
            "observation.t_state_ns": np.array([s.t_ns], dtype=np.float64),
            "observation.t_command_ns": np.array([c.t_ns], dtype=np.float64),
            "observation.input_source": np.array([SOURCE_CODES.get(self.worker.active_source(s), 0)],
                                                 dtype=np.float32),
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
