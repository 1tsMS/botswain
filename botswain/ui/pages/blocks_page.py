"""Blocks page: a Blockly editor in a Qt web view, with Run / Stop / New / Save / Open.

The editor only builds the program. Run reads it as JSON and hands it to ProgramRunner (Python), which
sends every step through the worker and the safety gate. The web view is created the first time the page
is shown, so the app starts fast and tests never load a browser engine.
"""

from __future__ import annotations

import json
import threading
from collections.abc import Callable
from pathlib import Path

from PySide6.QtCore import Qt, QUrl, Signal
from PySide6.QtWidgets import QFileDialog, QHBoxLayout, QLabel, QPushButton, QVBoxLayout, QWidget

from botswain.control.program import ProgramError, ProgramRunner
from botswain.ui.theme import set_role

PAGE = Path(__file__).resolve().parents[3] / "assets" / "blockly" / "botswain.html"
PROGRAMS = Path(__file__).resolve().parents[3] / "programs"


class BlocksPage(QWidget):
    block_changed = Signal(object)  # block id or None, from the runner thread
    finished = Signal(str, str)  # level, message

    def __init__(self, runner: ProgramRunner, point_names: Callable[[], list[str]],
                 log: Callable[[str, str], None], parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.runner = runner
        self._point_names = point_names
        self._log = log
        self.view = None
        self._thread: threading.Thread | None = None
        lay = QVBoxLayout(self)
        lay.setContentsMargins(8, 8, 8, 8)
        bar = QHBoxLayout()
        self.run_btn = self._button("Run", "primary", self.run_program)
        self.stop_btn = self._button("Stop", "btn", self.stop_program)
        self.new_btn = self._button("New", "btn", lambda: self._js("botswain.clear()"))
        self.save_btn = self._button("Save program", "btn", self.save_program)
        self.open_btn = self._button("Open program", "btn", self.open_program)
        for b in (self.run_btn, self.stop_btn, self.new_btn, self.save_btn, self.open_btn):
            bar.addWidget(b)
        self.status = set_role(QLabel("Build a program inside 'when Run is pressed', then Run. STOP aborts it."), "lab")
        bar.addWidget(self.status, 1)
        lay.addLayout(bar)
        self._host = QVBoxLayout()
        lay.addLayout(self._host, 1)
        self.stop_btn.setEnabled(False)
        self.block_changed.connect(lambda block_id: self._js(f"botswain.highlight({json.dumps(block_id)})"))
        self.finished.connect(self._finished)

    def _button(self, text: str, role: str, slot) -> QPushButton:
        b = set_role(QPushButton(text), role)
        b.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        b.clicked.connect(slot)
        return b

    # ---- the editor ----

    def showEvent(self, event) -> None:
        if self.view is None:
            from PySide6.QtWebEngineWidgets import QWebEngineView  # heavy: load only when needed

            self.view = QWebEngineView(self)
            self.view.setFocusPolicy(Qt.FocusPolicy.ClickFocus)
            self.view.loadFinished.connect(lambda _ok: self.refresh_points())
            self.view.load(QUrl.fromLocalFile(str(PAGE)))
            self._host.addWidget(self.view)
        else:
            self.refresh_points()
        super().showEvent(event)

    def refresh_points(self) -> None:
        self._js(f"botswain.setPoints({json.dumps(self._point_names())})")

    def _js(self, script: str, callback: Callable | None = None) -> None:
        if self.view is None:
            return
        guarded = f"(window.botswain ? ({script}) : null)"
        if callback is None:
            self.view.page().runJavaScript(guarded)
        else:
            self.view.page().runJavaScript(guarded, 0, callback)

    # ---- running ----

    @property
    def running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def run_program(self) -> None:
        if not self.running:
            self._js("botswain.save()", self._start)

    def _start(self, text: str | None) -> None:
        if not text:
            self._log("error", "Blocks: the editor is not ready yet")
            return
        program = json.loads(text)
        self.run_btn.setEnabled(False)
        self.stop_btn.setEnabled(True)
        self.status.setText("running...")
        self._log("info", "program started")

        def work() -> None:
            self.runner.on_block = self.block_changed.emit
            try:
                self.runner.run(program)
                self.finished.emit("info", "program finished")
            except ProgramError as e:
                self.finished.emit("warn" if str(e) == "stopped" else "error", f"program stopped: {e}")

        self._thread = threading.Thread(target=work, name="program", daemon=True)
        self._thread.start()

    def stop_program(self) -> None:
        self.runner.abort()

    def _finished(self, level: str, message: str) -> None:
        self._log(level, message)
        self.status.setText(message)
        self.run_btn.setEnabled(True)
        self.stop_btn.setEnabled(False)

    # ---- files ----

    def save_program(self) -> None:
        PROGRAMS.mkdir(exist_ok=True)
        path, _ = QFileDialog.getSaveFileName(self, "Save program", str(PROGRAMS / "program.json"), "Program (*.json)")
        if path:
            self._js("botswain.save()", lambda text: self._write(Path(path), text))

    def _write(self, path: Path, text: str | None) -> None:
        if text:
            path.write_text(json.dumps(json.loads(text), indent=1), encoding="utf-8")
            self._log("info", f"program saved: {path.name}")

    def open_program(self) -> None:
        PROGRAMS.mkdir(exist_ok=True)
        path, _ = QFileDialog.getOpenFileName(self, "Open program", str(PROGRAMS), "Program (*.json)")
        if not path:
            return
        try:
            text = json.dumps(json.loads(Path(path).read_text(encoding="utf-8")))
        except (OSError, ValueError) as e:
            self._log("error", f"cannot open {Path(path).name}: {e}")
            return
        self._js(f"botswain.load({json.dumps(text)})")
        self._log("info", f"program opened: {Path(path).name}")
