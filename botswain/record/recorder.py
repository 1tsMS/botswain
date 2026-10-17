"""Fixed-rate recorder on the one clock (SPEC 5.12).

LeRobot computes timestamp = frame_index / fps, so we sample on a fixed grid and store the real arrival
times as extra features. Per frame:
- observation.state       measured joints + TCP pose + gripper
- action                  the commanded target AFTER the gate (pose + gripper): what we really sent
- observation.command_raw the input BEFORE the gate (shows clamping)
- observation.controller_target  the controller's own target (Dobot ToolVectorTarget)
- observation.t_state_ns / t_command_ns / t_cam_<name>_ns  real arrival times (float64)
- observation.images.<name>  newest camera frame
"""

from __future__ import annotations

import logging
import threading
import time
from collections.abc import Callable
from typing import Protocol

import numpy as np

from botswain.control.worker import RobotWorker
from botswain.core.types import AXES_CART

log = logging.getLogger(__name__)


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
        self.task = ""
        self.error: str | None = None
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
        self.error = None
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="recorder", daemon=True)
        self._thread.start()

    def save_episode(self) -> int:
        self._end()
        n = self.frames
        if n:
            self.writer.save()
        return n

    def discard_episode(self) -> None:
        self._end()
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
            self.writer.add(self.sample())
            self.frames += 1
            k += 1
            behind = int((self._clock() - start) / period) - k
            if behind > 0:  # a slow frame: skip grid slots instead of bunching frames together
                log.warning("recorder fell %d frame(s) behind", behind)
                k += behind

    def sample(self) -> dict:
        s = self.worker.latest_state()
        c = self.worker.commanded
        grip_meas = s.gripper if s.gripper is not None else np.nan
        grip_cmd = np.nan if c.gripper_open is None else float(c.gripper_open)
        sent = c.sent or s.tcp_target  # nothing commanded yet: the controller's own target
        raw = c.raw or sent
        frame = {
            "observation.state": np.array([*s.joints.values, *s.tcp.as_list(), grip_meas], dtype=np.float32),
            "action": np.array([*sent.as_list(), grip_cmd], dtype=np.float32),
            "observation.command_raw": np.array([*raw.as_list(), grip_cmd], dtype=np.float32),
            "observation.controller_target": np.array(s.tcp_target.as_list(), dtype=np.float32),
            "observation.t_state_ns": np.array([s.t_ns], dtype=np.float64),
            "observation.t_command_ns": np.array([c.t_ns], dtype=np.float64),
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
