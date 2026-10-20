"""A recording session: one dataset, many episodes. Glue between the GUI and Recorder + LeRobotWriter.

The dataset's features (cameras, image sizes, fps) are fixed when its first episode starts, because LeRobot
fixes them at create(). Changing cameras or fps needs a new dataset (Close dataset, new name).
"""

from __future__ import annotations

import logging
from collections.abc import Callable
from pathlib import Path

from botswain.control.worker import RobotWorker
from botswain.record.recorder import FrameSource, Recorder, Writer, build_features

log = logging.getLogger(__name__)

WriterFactory = Callable[[Path, str, int, dict], Writer]


def lerobot_available() -> bool:
    from importlib.util import find_spec

    return find_spec("lerobot") is not None


def lerobot_writer_factory(robot_type: str) -> WriterFactory:
    def make(root: Path, repo_id: str, fps: int, features: dict) -> Writer:
        from botswain.record.lerobot_writer import LeRobotWriter  # torch loads here, not at app start

        return LeRobotWriter(root, repo_id, fps, features, robot_type)

    return make


class RecordingSession:
    def __init__(self, worker: RobotWorker, writer_factory: WriterFactory) -> None:
        self.worker = worker
        self.writer_factory = writer_factory
        self.recorder: Recorder | None = None
        self.root: Path | None = None
        self.episodes_saved = 0
        self.dataset_source: str | None = None  # the input of the first saved episode that moved the arm
        self._key: tuple | None = None

    @property
    def recording(self) -> bool:
        return self.recorder is not None and self.recorder.recording

    @property
    def frames(self) -> int:
        return self.recorder.frames if self.recorder else 0

    def start(self, root: Path, repo_id: str, task: str, fps: int, cameras: dict[str, FrameSource]) -> None:
        if self.recording:
            raise RuntimeError("already recording an episode")
        if not task.strip():
            raise ValueError("type a task description first (it is stored with every frame)")
        if not self.worker.latest_state().link_ok:
            raise ValueError("robot not connected: there is no state to record")
        shapes = {}
        for name, cam in cameras.items():
            latest = cam.latest()
            if latest is None:
                raise ValueError(f"camera {name} has no picture yet")
            shapes[name] = tuple(latest[0].shape[:2])
        key = (root.resolve(), fps, tuple(sorted(shapes.items())))
        if self.recorder is None:
            if root.exists() and any(root.iterdir()):
                raise ValueError(f"{root} already holds data: pick a new dataset name")
            features = build_features(self.worker.driver.capabilities.dof, shapes)
            writer = self.writer_factory(root, repo_id, fps, features)
            self.recorder = Recorder(self.worker, cameras, fps, writer)
            self.root, self._key = root, key
        elif key != self._key:
            raise ValueError("cameras, image size or fps changed: Close dataset first, then start a new one")
        self.recorder.cameras = cameras
        self.recorder.start_episode(task.strip())

    def mismatch(self) -> str | None:
        """Why the running episode does not belong in this dataset, or None. One dataset per input: training
        normalises with the dataset's own stats, so a gamepad episode in a phone dataset mixes the conditions."""
        source = self.recorder.main_source if self.recorder else None
        if source is None or self.dataset_source is None or source == self.dataset_source:
            return None
        return f"this episode was driven by {source}, but this dataset holds {self.dataset_source} episodes"

    def save(self) -> int:
        """Ends the episode and writes it (encodes video: takes a few seconds). Returns the frame count."""
        if self.recorder is None:
            return 0
        source = self.recorder.main_source
        n = self.recorder.save_episode()
        if n:
            self.episodes_saved += 1
            if self.dataset_source is None:
                self.dataset_source = source
        return n

    def discard(self) -> None:
        if self.recorder is not None:
            self.recorder.discard_episode()

    def close(self) -> None:
        """Finalize the dataset (LeRobot needs this, or the files are unreadable)."""
        if self.recorder is not None:
            self.recorder.close()
        self.recorder, self._key = None, None
        self.dataset_source = None
