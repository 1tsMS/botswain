"""Cameras page: scan for USB cameras, pick one (or all) from the dropdown, show it here and in the dock."""

from __future__ import annotations

from collections.abc import Callable

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import QComboBox, QHBoxLayout, QPushButton, QVBoxLayout, QWidget

from botswain.sensors.cameras import CameraThread, list_cameras
from botswain.ui.theme import set_role
from botswain.ui.widgets.camera_tile import CameraGrid
from botswain.ui.widgets.panel import Panel

ALL = -1


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
        self.scan_btn = self._button("Scan", "btn", self.scan)
        self.choice = QComboBox()
        self.choice.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.choice.setMinimumWidth(180)
        self.choice.addItem("press Scan first", None)
        self.show_btn = self._button("Show", "primary", self.show_selected)
        self.stop_btn = self._button("Stop", "btn", self.stop_all)
        for w in (self.scan_btn, self.choice, self.show_btn, self.stop_btn):
            row.addWidget(w)
        row.addStretch(1)
        panel.body_layout.addLayout(row)
        self.grid = CameraGrid("No camera running. Plug it in, press Scan, pick it, press Show.")
        panel.body_layout.addWidget(self.grid, 1)
        lay.addWidget(panel)
        self.show_btn.setEnabled(False)

    def _button(self, text: str, role: str, slot) -> QPushButton:
        b = set_role(QPushButton(text), role)
        b.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        b.clicked.connect(slot)
        return b

    def scan(self, indexes: list[int] | None = None) -> None:
        """Probing is slow (about a second per index), so it only runs on request."""
        self.stop_all()
        found = list_cameras() if indexes is None else indexes
        self.choice.clear()
        for i in found:
            self.choice.addItem(f"Camera {i}", i)
        if len(found) > 1:
            self.choice.addItem("All cameras", ALL)
        if not found:
            self.choice.addItem("no cameras found", None)
        self.show_btn.setEnabled(bool(found))
        self._log("info", f"cameras found: {found or 'none'}")

    def show_selected(self) -> None:
        choice = self.choice.currentData()
        if choice is None:
            return
        self.stop_all()
        indexes = [self.choice.itemData(i) for i in range(self.choice.count())
                   if self.choice.itemData(i) not in (None, ALL)] if choice == ALL else [choice]
        for i in indexes:
            cam = CameraThread(i, f"cam {i}")
            try:
                cam.start()
                self.cameras.append(cam)
            except RuntimeError as e:
                self._log("warn", str(e))
        self._changed()

    def stop_all(self) -> None:
        for cam in self.cameras:
            cam.stop()
        self.cameras = []
        self._changed()

    def _changed(self) -> None:
        self.grid.set_cameras(self.cameras)
        self.cameras_changed.emit(self.cameras)
