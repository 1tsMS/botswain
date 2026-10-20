"""USB cameras: one thread per camera keeps the newest frame and its arrival time (clock.now_ns).

DirectShow backend on Windows (opens much faster than MSMF). The app runs fine with zero cameras.
"""

from __future__ import annotations

import logging
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass

import cv2
import numpy as np

from botswain.core.clock import now_ns

log = logging.getLogger(__name__)


def open_capture(index: int):
    return cv2.VideoCapture(index, cv2.CAP_DSHOW)


@dataclass(frozen=True)
class CameraSource:
    name: str
    index: int


def list_sources() -> list[CameraSource]:
    """Every DirectShow video source by name: USB webcams, DroidCam, OBS Virtual Camera...

    Fast (reads the device list, opens nothing). Falls back to probing indexes if the enumerator is missing.
    """
    try:
        from cv2_enumerate_cameras import enumerate_cameras
    except ImportError:
        return [CameraSource(f"Camera {i}", i) for i in list_cameras()]
    return [CameraSource(c.name, c.index) for c in enumerate_cameras(cv2.CAP_DSHOW)]


def list_cameras(max_index: int = 4, opener: Callable[[int], object] = open_capture) -> list[int]:
    """Indexes that open. Slow (about a second per index), so only run it when the operator asks."""
    found = []
    for i in range(max_index):
        cap = opener(i)
        try:
            if cap.isOpened():
                found.append(i)
        finally:
            cap.release()
    return found


class CameraThread:
    def __init__(self, index: int, name: str, width: int = 640, height: int = 480,
                 opener: Callable[[int], object] = open_capture) -> None:
        self.index = index
        self.name = name
        self.width = width
        self.height = height
        self._opener = opener
        self._latest: tuple[np.ndarray, int] | None = None
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name=f"camera-{index}", daemon=True)
        self._cap = None
        self.fps = 0.0
        self.failures = 0
        self.error: str | None = None  # set by the camera's thread if the device does not open

    def start(self) -> None:
        """Returns at once: opening a DirectShow device (DroidCam, OBS) can take seconds, so the camera's own
        thread does it. Until the first frame, latest() is None; if it fails, error says why."""
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread.is_alive():
            self._thread.join(1.0)

    def latest(self) -> tuple[np.ndarray, int] | None:
        """(BGR frame, arrival ns) of the newest frame, or None before the first one."""
        return self._latest

    def _run(self) -> None:
        cap = self._opener(self.index)
        if not cap.isOpened():
            self.error = f"{self.name} did not open (in use by another app?)"
            log.warning("camera %s (%d) did not open", self.name, self.index)
            cap.release()
            return
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self._cap = cap
        try:
            self._read_loop()
        finally:
            cap.release()

    def _read_loop(self) -> None:
        count, window = 0, now_ns()
        while not self._stop.is_set():
            ok, frame = self._cap.read()
            t = now_ns()
            if not ok or frame is None:
                self.failures += 1
                time.sleep(0.05)
                continue
            self._latest = (frame, t)
            count += 1
            if t - window >= 1_000_000_000:
                self.fps = count * 1e9 / (t - window)
                count, window = 0, t
