"""Camera tiles: newest frame of each CameraThread, redrawn at 15 Hz on the GUI thread."""

from __future__ import annotations

import cv2
from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QImage, QPixmap
from PySide6.QtWidgets import QGridLayout, QLabel, QWidget

from botswain.sensors.cameras import CameraThread
from botswain.ui.theme import set_role


class CameraTile(QLabel):
    def __init__(self, cam: CameraThread, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.cam = cam
        self.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.setMinimumSize(160, 120)
        self.setStyleSheet("background: #1f2023; color: #8d9097; border-radius: 4px;")
        self.setText(f"{cam.name} · opening…")
        self._last_t = 0

    def refresh(self) -> None:
        if self.cam.error and not self._last_t:
            self.setText(self.cam.error)
            return
        latest = self.cam.latest()
        if latest is None or latest[1] == self._last_t:
            return
        frame, self._last_t = latest
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w, _ = rgb.shape
        image = QImage(rgb.data, w, h, 3 * w, QImage.Format.Format_RGB888)
        self.setPixmap(QPixmap.fromImage(image).scaled(self.size(), Qt.AspectRatioMode.KeepAspectRatio,
                                                       Qt.TransformationMode.SmoothTransformation))


class CameraGrid(QWidget):
    def __init__(self, empty_text: str = "No cameras yet", parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._grid = QGridLayout(self)
        self._grid.setContentsMargins(6, 6, 6, 6)
        self._grid.setSpacing(6)
        self._empty = set_role(QLabel(empty_text), "dockEmpty")
        self._empty.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._grid.addWidget(self._empty, 0, 0)
        self.tiles: list[CameraTile] = []
        self._timer = QTimer(self)
        self._timer.setInterval(66)
        self._timer.timeout.connect(lambda: [t.refresh() for t in self.tiles])
        self._timer.start()

    def set_cameras(self, cams: list[CameraThread]) -> None:
        for t in self.tiles:
            t.deleteLater()
        self.tiles = [CameraTile(c) for c in cams]
        self._empty.setVisible(not cams)
        for i, tile in enumerate(self.tiles):
            self._grid.addWidget(tile, i // 2, i % 2)
