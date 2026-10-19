"""Gamepad panel (Teleop page): enable the controller, see it live, the button map."""

from __future__ import annotations

from collections.abc import Callable

from PySide6.QtCore import Qt, QTimer, Signal
from PySide6.QtWidgets import QHBoxLayout, QLabel, QPushButton, QWidget

from botswain.control.worker import RobotWorker
from botswain.teleop.gamepad import GamepadTeleop, PadState, XInputReader
from botswain.ui.theme import mono_font, set_role
from botswain.ui.widgets.panel import Panel

MAP_TEXT = ("Hold LB to move (let go = stop) · left stick X/Y · RT up / LT down · right stick Rz / Ry · "
            "RB slow · A gripper open/close · D-pad speed · Start record/save episode · Back discard · "
            "X save point")


class GamepadPanel(Panel):
    action = Signal(str)  # "start" / "back" / "x", delivered on the GUI thread

    def __init__(self, worker: RobotWorker, log: Callable[[str, str], None],
                 reader: Callable[[], PadState] | None = None, parent: QWidget | None = None) -> None:
        super().__init__("Game controller", "Xbox-style, X-input mode", parent)
        self._log = log
        self._reader = reader
        self.teleop = GamepadTeleop(worker, reader or (lambda: PadState(False)))
        self.teleop.on_action = self.action.emit  # Qt queues it from the pad thread to the GUI thread
        row = QHBoxLayout()
        self.enable_btn = set_role(QPushButton("Use game controller"), "btn")
        self.enable_btn.setCheckable(True)
        self.enable_btn.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.enable_btn.toggled.connect(self._toggled)
        row.addWidget(self.enable_btn)
        self.state_label = set_role(QLabel("off"), "lab")
        row.addWidget(self.state_label, 1)
        self.body_layout.addLayout(row)
        self.live = QLabel("")
        self.live.setFont(mono_font(12))
        self.body_layout.addWidget(self.live)
        legend = set_role(QLabel(MAP_TEXT), "lab")
        legend.setWordWrap(True)
        self.body_layout.addWidget(legend)
        self._timer = QTimer(self)
        self._timer.setInterval(100)
        self._timer.timeout.connect(self.refresh)
        self._timer.start()

    def _toggled(self, on: bool) -> None:
        if on:
            if self._reader is None:
                reader = XInputReader()
                if reader.find() is None:
                    self._log("warn", "no game controller found: plug it in and switch it to X-input mode")
                self.teleop.reader = reader
            self.teleop.start()
            self._log("info", "game controller on: hold LB to move")
        else:
            self.teleop.stop()
            self._log("info", "game controller off")

    def shutdown(self) -> None:
        if self.teleop.running:
            self.teleop.stop()

    def refresh(self) -> None:
        t, pad = self.teleop, self.teleop.pad
        if not t.running:
            self.state_label.setText("off")
            self.live.setText("")
            return
        if not pad.connected:
            self.state_label.setText("not connected (plug in, X-input mode)")
            self.live.setText("")
            return
        moving = "MOVING (LB held)" if t.active else "holding still (hold LB to move)"
        self.state_label.setText(f"connected · speed {t.speed_scale * 100:.0f} % · {moving}")
        buttons = " ".join(sorted(pad.buttons)) or "-"
        self.live.setText(f"L {pad.lx:+.2f} {pad.ly:+.2f}  R {pad.rx:+.2f} {pad.ry:+.2f}  "
                          f"LT {pad.lt:.2f} RT {pad.rt:.2f}  [{buttons}]")
