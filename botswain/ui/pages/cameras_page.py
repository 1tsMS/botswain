"""Cameras page: scan for USB cameras, pick one from the dropdown and Show it; Show again to add more (or pick
All sources). Stop stops them all. Every running camera shows here, in the dock, in VR and in recordings."""

from __future__ import annotations

import threading
from collections.abc import Callable

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import QComboBox, QHBoxLayout, QPushButton, QVBoxLayout, QWidget

from botswain.sensors.cameras import CameraSource, CameraThread, list_sources
from botswain.ui.theme import set_role
from botswain.ui.widgets.camera_tile import CameraGrid
from botswain.ui.widgets.panel import Panel

ALL = -1


class CamerasPage(QWidget):
    cameras_changed = Signal(list)
    _scanned = Signal(list)  # from the scan thread

    def __init__(self, log: Callable[[str, str], None], parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._log = log
        self.cameras: list[CameraThread] = []
        self._names: dict[int, str] = {}
        lay = QVBoxLayout(self)
        lay.setContentsMargins(8, 8, 8, 8)
        panel = Panel("Cameras", "any video source: webcam, DroidCam, OBS")
        row = QHBoxLayout()
        # Not self.scan directly: clicked(bool) would pass False as `sources`, and Scan then crashed in silence.
        self.scan_btn = self._button("Scan", "btn", lambda: self.scan())
        self.choice = QComboBox()
        self.choice.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.choice.setMinimumWidth(240)
        self.choice.addItem("press Scan first", None)
        self.show_btn = self._button("Show", "primary", self.show_selected)
        self.stop_btn = self._button("Stop", "btn", self.stop_all)
        for w in (self.scan_btn, self.choice, self.show_btn, self.stop_btn):
            row.addWidget(w)
        row.addStretch(1)
        panel.body_layout.addLayout(row)
        self.grid = CameraGrid("No camera running. Plug it in, press Scan, pick it, press Show. "
                               "Pick another and Show again to add it.")
        panel.body_layout.addWidget(self.grid, 1)
        lay.addWidget(panel)
        self.show_btn.setEnabled(False)
        self._scanned.connect(self._fill)

    def _button(self, text: str, role: str, slot) -> QPushButton:
        b = set_role(QPushButton(text), role)
        b.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        b.clicked.connect(slot)
        return b

    def scan(self, sources: list[CameraSource] | None = None) -> None:
        """Lists every video source Windows knows: USB webcams, DroidCam, OBS Virtual Camera...
        The listing runs on a thread (a driver can take a while to answer), so the window never freezes."""
        self.stop_all()
        if sources is not None:
            self._fill(sources)
            return
        self.scan_btn.setEnabled(False)
        self.choice.clear()
        self.choice.addItem("scanning…", None)
        self.show_btn.setEnabled(False)
        threading.Thread(target=lambda: self._scanned.emit(list_sources()), name="camera-scan", daemon=True).start()

    def _fill(self, found: list[CameraSource]) -> None:
        self.scan_btn.setEnabled(True)
        self._names = {s.index: s.name for s in found}
        self.choice.clear()
        for s in found:
            self.choice.addItem(s.name, s.index)
        if len(found) > 1:
            self.choice.addItem("All sources", ALL)
        if not found:
            self.choice.addItem("no video sources found", None)
        self.show_btn.setEnabled(bool(found))
        self._log("info", f"video sources: {', '.join(s.name for s in found) or 'none'}")

    def show_selected(self) -> None:
        choice = self.choice.currentData()
        if choice is None:
            return
        indexes = [self.choice.itemData(i) for i in range(self.choice.count())
                   if self.choice.itemData(i) not in (None, ALL)] if choice == ALL else [choice]
        running = {cam.index for cam in self.cameras}
        if all(i in running for i in indexes):
            self._log("info", "that camera is already running")
            return
        for i in indexes:
            if i in running:
                continue  # adds to what runs; Stop stops them all
            cam = CameraThread(i, self._names.get(i, f"cam {i}"))
            cam.start()  # opens on its own thread; a failure shows on the tile
            self.cameras.append(cam)
        self._changed()

    def stop_all(self) -> None:
        for cam in self.cameras:
            cam.stop()
        self.cameras = []
        self._changed()

    def _changed(self) -> None:
        self.grid.set_cameras(self.cameras)
        self.cameras_changed.emit(self.cameras)
