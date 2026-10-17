"""Episode bar (Episodes dock tab): dataset name, task, Start / Save / Discard / Close dataset.

Saving encodes video and finalizing writes the parquet footers; both run off the GUI thread.
"""

from __future__ import annotations

import threading
import time
from collections.abc import Callable
from pathlib import Path

from PySide6.QtCore import Qt, QTimer, Signal
from PySide6.QtWidgets import QGridLayout, QHBoxLayout, QLabel, QLineEdit, QPushButton, QWidget

from botswain.record.session import RecordingSession
from botswain.sensors.cameras import CameraThread
from botswain.ui.theme import set_role

DATA_DIR = Path(__file__).resolve().parents[3] / "data"
FPS = 30


class EpisodeBar(QWidget):
    busy_done = Signal(str, str)  # level, message from a background save/close

    def __init__(self, session: RecordingSession | None, cameras: Callable[[], list[CameraThread]],
                 log: Callable[[str, str], None], parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.session = session
        self._cameras = cameras
        self._log = log
        self._busy = False
        self._t0 = 0.0
        set_role(self, "dock")
        self.setAttribute(Qt.WidgetAttribute.WA_StyledBackground)
        grid = QGridLayout(self)
        grid.setContentsMargins(10, 8, 10, 8)
        grid.setHorizontalSpacing(8)
        self.name_edit = self._field(time.strftime("botswain_%Y%m%d_%H%M"))
        self.task_edit = self._field("")
        self.task_edit.setPlaceholderText("what the operator does, e.g. pick the red cube")
        grid.addWidget(set_role(QLabel("Dataset"), "dockMeta"), 0, 0)
        grid.addWidget(self.name_edit, 0, 1)
        grid.addWidget(set_role(QLabel("Task"), "dockMeta"), 1, 0)
        grid.addWidget(self.task_edit, 1, 1)
        row = QHBoxLayout()
        self.start_btn = self._button("Start", "primary", self.start)
        self.save_btn = self._button("Save", "btn", self.save)
        self.discard_btn = self._button("Discard", "btn", self.discard)
        self.close_btn = self._button("Close dataset", "btn", self.close_dataset)
        for b in (self.start_btn, self.save_btn, self.discard_btn, self.close_btn):
            row.addWidget(b)
        row.addStretch(1)
        grid.addLayout(row, 2, 0, 1, 2)
        self.status = set_role(QLabel(""), "dockMeta")
        self.status.setWordWrap(True)
        grid.addWidget(self.status, 3, 0, 1, 2)
        grid.setColumnStretch(1, 1)
        self.busy_done.connect(self._busy_finished)
        self._timer = QTimer(self)
        self._timer.setInterval(250)
        self._timer.timeout.connect(self.refresh)
        self._timer.start()
        self.refresh()

    def _field(self, text: str) -> QLineEdit:
        edit = set_role(QLineEdit(text), "field")
        edit.setProperty("free_text", True)  # Space types here; Esc still stops the robot
        return edit

    def _button(self, text: str, role: str, slot) -> QPushButton:
        b = set_role(QPushButton(text), role)
        b.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        b.clicked.connect(slot)
        return b

    @property
    def dataset_root(self) -> Path:
        name = "".join(c for c in self.name_edit.text().strip() if c.isalnum() or c in "-_") or "dataset"
        return DATA_DIR / name

    @property
    def recording(self) -> bool:
        return self.session is not None and self.session.recording

    def start(self) -> None:
        if self.session is None or self._busy:
            return
        cams = {c.name.replace(" ", "_").lower(): c for c in self._cameras()}
        try:
            self.session.start(self.dataset_root, f"botswain/{self.dataset_root.name}", self.task_edit.text(),
                               FPS, cams)
        except (ValueError, RuntimeError, OSError) as e:
            self._log("error", f"Record: {e}")
            return
        self._t0 = time.perf_counter()
        self._log("info", f"recording episode {self.session.episodes_saved + 1} to {self.dataset_root} "
                          f"({len(cams)} camera(s), {FPS} fps)")

    def save(self) -> None:
        if self.recording:
            self._in_background("Saving episode (encoding video)...", self._do_save)

    def discard(self) -> None:
        if self.recording:
            self.session.discard()
            self._log("warn", "episode discarded")

    def close_dataset(self) -> None:
        if self.session is not None and self.session.recorder is not None:
            self._in_background("Finalizing dataset...", self._do_close)

    def _do_save(self) -> tuple[str, str]:
        n = self.session.save()
        return "info", f"saved episode {self.session.episodes_saved}: {n} frames ({n / FPS:.1f} s)"

    def _do_close(self) -> tuple[str, str]:
        root = self.session.root
        self.session.close()
        return "info", f"dataset finalized: {root}"

    def _in_background(self, text: str, job: Callable[[], tuple[str, str]]) -> None:
        self._busy = True
        self.status.setText(text)

        def run() -> None:
            try:
                level, msg = job()
            except Exception as e:  # noqa: BLE001 -- shown to the operator; the dataset may need a look
                level, msg = "error", f"Record: {type(e).__name__}: {e}"
            self.busy_done.emit(level, msg)

        threading.Thread(target=run, name="record-io", daemon=True).start()

    def _busy_finished(self, level: str, msg: str) -> None:
        self._busy = False
        self._log(level, msg)
        self.refresh()

    def refresh(self) -> None:
        if self.session is None:
            self.status.setText("Recording needs LeRobot: pip install -e .[record]")
            for b in (self.start_btn, self.save_btn, self.discard_btn, self.close_btn):
                b.setEnabled(False)
            return
        rec, busy = self.recording, self._busy
        self.start_btn.setEnabled(not rec and not busy)
        self.save_btn.setEnabled(rec and not busy)
        self.discard_btn.setEnabled(rec and not busy)
        self.close_btn.setEnabled(not rec and not busy and self.session.recorder is not None)
        self.name_edit.setEnabled(self.session.recorder is None)
        if busy:
            return
        error = self.session.recorder.error if self.session.recorder else None
        if error:
            self.status.setText(f"recording stopped: {error}")
        elif rec:
            self.status.setText(f"● recording  {time.perf_counter() - self._t0:5.1f} s  {self.session.frames} frames")
        else:
            self.status.setText(f"{self.session.episodes_saved} episode(s) saved · folder {self.dataset_root}")
