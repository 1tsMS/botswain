"""Cameras page: find USB cameras on request, start one thread each, show them big here and small in the dock."""

from __future__ import annotations

from collections.abc import Callable

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import QHBoxLayout, QPushButton, QVBoxLayout, QWidget

from botswain.sensors.cameras import CameraThread, list_cameras
from botswain.ui.theme import set_role
from botswain.ui.widgets.camera_tile import CameraGrid
from botswain.ui.widgets.panel import Panel


class CamerasPage(QWidget):
    cameras_changed = Signal(list)

    def __init__(self, log: Callable[[str, str], None], parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._log = log
        self.cameras: list[CameraThread] = []
        lay = QVBoxLayout(self)
        lay.setContentsMargins(8, 8, 8, 8)
        panel = Panel("Cameras", "USB, DirectShow")
        row = QHBoxLayout()
        self.find_btn = set_role(QPushButton("Find cameras"), "primary")
        self.stop_btn = set_role(QPushButton("Stop cameras"), "btn")
        for b in (self.find_btn, self.stop_btn):
            b.setFocusPolicy(Qt.FocusPolicy.NoFocus)
            row.addWidget(b)
        row.addStretch(1)
        panel.body_layout.addLayout(row)
        self.grid = CameraGrid("No cameras running. Plug them in and press Find cameras.")
        panel.body_layout.addWidget(self.grid, 1)
        lay.addWidget(panel)
        self.find_btn.clicked.connect(self.find)
        self.stop_btn.clicked.connect(self.stop_all)

    def find(self) -> None:
        self.stop_all()
        indexes = list_cameras()
        for i in indexes:
            cam = CameraThread(i, f"cam {i}")
            try:
                cam.start()
                self.cameras.append(cam)
            except RuntimeError as e:
                self._log("warn", str(e))
        self._log("info", f"cameras: {len(self.cameras)} running ({indexes or 'none found'})")
        self._changed()

    def stop_all(self) -> None:
        for cam in self.cameras:
            cam.stop()
        self.cameras = []
        self._changed()

    def _changed(self) -> None:
        self.grid.set_cameras(self.cameras)
        self.cameras_changed.emit(self.cameras)
