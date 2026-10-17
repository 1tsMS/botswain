"""Phone teleop page: start the phone server, show its address, tune the mapping, watch the safety counters."""

from __future__ import annotations

from collections.abc import Callable

from PySide6.QtCore import Qt, QTimer
from PySide6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSlider,
    QWidget,
)

from botswain.control.worker import RobotWorker
from botswain.core.types import Mode
from botswain.teleop.mapping import TeleopMapper, remap_about_z
from botswain.teleop.phone import PhoneServer
from botswain.teleop.session import TeleopSession
from botswain.ui.theme import mono_font, set_role
from botswain.ui.widgets.panel import Panel

PORT = 4443
STREAM_RULES = ("hold", "stale", "teleport", "step", "box")


class TeleopPage(QWidget):
    def __init__(self, worker: RobotWorker, log: Callable[[str, str], None], parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.worker = worker
        self._log = log
        self.server: PhoneServer | None = None
        self.mapper = TeleopMapper(scale_pos=0.3, scale_rot=0.0)
        self.session = TeleopSession(worker, self.mapper)

        grid = QGridLayout(self)
        grid.setContentsMargins(8, 8, 8, 8)
        grid.setSpacing(8)
        phone = Panel("Phone (WebXR)", f"HTTPS port {PORT}")
        self.start_btn = set_role(QPushButton("Start phone server"), "primary")
        self.start_btn.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.start_btn.clicked.connect(self.toggle_server)
        self.url_label = QLabel("Server off")
        self.url_label.setFont(mono_font(18))
        self.url_label.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        self.status_label = set_role(QLabel(""), "lab")
        notes = set_role(QLabel(
            "Android Chrome with ARCore (Oppo K13 works). Same WiFi as this laptop. Accept the certificate warning, "
            "then start AR. Hold the on-screen button to move; let go to stop. Windows Firewall must allow inbound "
            f"TCP {PORT} (add the rule yourself)."), "lab")
        notes.setWordWrap(True)
        for w in (self.start_btn, self.url_label, self.status_label, notes):
            phone.body_layout.addWidget(w)
        phone.body_layout.addStretch(1)

        mapping = Panel("Mapping", "first runs: small and slow")
        row = QHBoxLayout()
        row.addWidget(set_role(QLabel("Motion scale"), "lab"))
        self.scale_slider = QSlider(Qt.Orientation.Horizontal)
        self.scale_slider.setRange(10, 100)
        self.scale_slider.setValue(30)
        self.scale_slider.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.scale_value = QLabel("0.30")
        self.scale_value.setFont(mono_font(13))
        row.addWidget(self.scale_slider, 1)
        row.addWidget(self.scale_value)
        mapping.body_layout.addLayout(row)
        self.rotation_box = QCheckBox("Follow phone rotation (off for first runs)")
        self.rotation_box.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        mapping.body_layout.addWidget(self.rotation_box)
        face = QHBoxLayout()
        face.addWidget(set_role(QLabel("Phone forward is robot"), "lab"))
        self.facing = QComboBox()
        self.facing.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        for text, deg in (("+X", 0), ("+Y", 90), ("-X", 180), ("-Y", 270)):
            self.facing.addItem(text, deg)
        face.addWidget(self.facing)
        face.addStretch(1)
        mapping.body_layout.addLayout(face)
        self.counters = set_role(QLabel(""), "lab")
        mapping.body_layout.addWidget(self.counters)
        mapping.body_layout.addStretch(1)

        grid.addWidget(phone, 0, 0)
        grid.addWidget(mapping, 0, 1)
        grid.setColumnStretch(0, 3)
        grid.setColumnStretch(1, 2)

        self.scale_slider.valueChanged.connect(self._mapping_changed)
        self.rotation_box.toggled.connect(self._mapping_changed)
        self.facing.currentIndexChanged.connect(self._mapping_changed)
        self._timer = QTimer(self)
        self._timer.setInterval(200)
        self._timer.timeout.connect(self._refresh)
        self._timer.start()
        self._refresh()

    def _mapping_changed(self) -> None:
        self.mapper.scale_pos = self.scale_slider.value() / 100.0
        self.mapper.scale_rot = 1.0 if self.rotation_box.isChecked() else 0.0
        self.mapper.remap = remap_about_z(self.facing.currentData())
        self.mapper.request_reanchor()  # never let a settings change make the target jump
        self.scale_value.setText(f"{self.mapper.scale_pos:.2f}")

    def toggle_server(self) -> None:
        if self.server is None:
            server = PhoneServer(self.session.on_sample, port=PORT)
            try:
                server.start()
            except RuntimeError as e:
                self._log("error", f"{e}. Is another program using port {PORT}?")
                return
            self.server = server
            note = " (shared package certificate)" if server.cert_is_shared else ""
            self._log("info", f"phone server running: {', '.join(server.urls)}{note}")
        else:
            self.shutdown()

    def shutdown(self) -> None:
        if self.server is None:
            return
        self.server.stop()
        self.server = None
        if self.worker.latest_state().mode is Mode.STREAMING:
            self.worker.request_stop()
        self._log("info", "phone server stopped")

    def _refresh(self) -> None:
        running = self.server is not None
        self.start_btn.setText("Stop phone server" if running else "Start phone server")
        self.url_label.setText("\n".join(self.server.urls) if running else "Server off")
        if running:
            s = self.session.last_sample
            hold = "holding" if s and s.hold else "not holding"
            self.status_label.setText(f"phones connected: {self.server.clients} · {self.server.hz:.0f} Hz · {hold}")
        else:
            self.status_label.setText("")
        counts = self.worker.gate.counts
        self.counters.setText("Gate: " + " · ".join(f"{r} {counts.get(r, 0)}" for r in STREAM_RULES))
