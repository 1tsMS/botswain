"""USB cameras: one thread per camera keeps the newest frame and its arrival time (clock.now_ns).

DirectShow backend on Windows (opens much faster than MSMF). The app runs fine with zero cameras.
"""

from __future__ import annotations

import logging
import threading
import time
from collections.abc import Callable

import cv2
import numpy as np

from botswain.core.clock import now_ns

log = logging.getLogger(__name__)


def open_capture(index: int):
    return cv2.VideoCapture(index, cv2.CAP_DSHOW)


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

    def start(self) -> None:
        self._cap = self._opener(self.index)
        if not self._cap.isOpened():
            raise RuntimeError(f"camera {self.index} did not open")
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread.is_alive():
            self._thread.join(1.0)
        if self._cap is not None:
            self._cap.release()

    def latest(self) -> tuple[np.ndarray, int] | None:
        """(BGR frame, arrival ns) of the newest frame, or None before the first one."""
        return self._latest

    def _run(self) -> None:
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
