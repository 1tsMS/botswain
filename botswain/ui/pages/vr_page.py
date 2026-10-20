"""VR page: start the VR server for Meta Quest Browser, show how to pair, tune the mapping, watch the hand live."""

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
from botswain.teleop.vr import PORT, VrServer, VrSession
from botswain.ui.theme import mono_font, set_role
from botswain.ui.widgets.pairing_link import PairingLink
from botswain.ui.widgets.panel import Panel

SETUP = (
    "1. Windows Settings > Network > Mobile hotspot: on (no internet needed). "
    "2. Quest: join that Wi-Fi, open Meta Quest Browser, go to the https address below and type the key. "
    "3. Accept the certificate warning, press Enter, put the controllers in view. "
    f"Windows Firewall must allow inbound TCP {PORT} (add the rule yourself)."
)


class VrPage(QWidget):
    def __init__(self, worker: RobotWorker, log: Callable[[str, str], None], parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.worker = worker
        self._log = log
        self.server: VrServer | None = None
        self.mapper = TeleopMapper(scale_pos=0.5, scale_rot=0.0)
        self.session = VrSession(worker, self.mapper)

        grid = QGridLayout(self)
        grid.setContentsMargins(8, 8, 8, 8)
        grid.setSpacing(8)

        headset = Panel("VR headset (WebXR)", f"Meta Quest Browser · HTTPS port {PORT}")
        self.start_btn = set_role(QPushButton("Start VR server"), "primary")
        self.start_btn.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.start_btn.clicked.connect(self.toggle_server)
        self.link = PairingLink()
        self.status_label = set_role(QLabel(""), "lab")
        setup = set_role(QLabel(SETUP), "lab")
        setup.setWordWrap(True)
        for w in (self.start_btn, self.link, self.status_label, setup):
            headset.body_layout.addWidget(w)
        headset.body_layout.addStretch(1)

        hand = Panel("Right hand", "live")
        self.hand_label = QLabel("no headset")
        self.hand_label.setFont(mono_font(14))
        controls = set_role(QLabel("Squeeze grip = clutch (hold to move; let go and squeeze again to re-centre). "
                                   "Trigger = gripper open/close. Lost tracking or a closed page stops the arm."),
                            "lab")
        controls.setWordWrap(True)
        hand.body_layout.addWidget(self.hand_label)
        hand.body_layout.addWidget(controls)
        hand.body_layout.addStretch(1)

        mapping = Panel("Mapping", "first runs: small and slow")
        row = QHBoxLayout()
        row.addWidget(set_role(QLabel("Motion scale"), "lab"))
        self.scale_slider = QSlider(Qt.Orientation.Horizontal)
        self.scale_slider.setRange(10, 100)
        self.scale_slider.setValue(50)
        self.scale_slider.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.scale_value = QLabel("0.50")
        self.scale_value.setFont(mono_font(13))
        row.addWidget(self.scale_slider, 1)
        row.addWidget(self.scale_value)
        mapping.body_layout.addLayout(row)
        self.rotation_box = QCheckBox("Follow hand rotation (off for first runs)")
        self.rotation_box.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        mapping.body_layout.addWidget(self.rotation_box)
        face = QHBoxLayout()
        face.addWidget(set_role(QLabel("Your forward is robot"), "lab"))
        self.facing = QComboBox()
        self.facing.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        for text, deg in (("+X", 0), ("+Y", 90), ("-X", 180), ("-Y", 270)):
            self.facing.addItem(text, deg)
        face.addWidget(self.facing)
        face.addStretch(1)
        mapping.body_layout.addLayout(face)
        mapping.body_layout.addStretch(1)

        grid.addWidget(headset, 0, 0, 2, 1)
        grid.addWidget(hand, 0, 1)
        grid.addWidget(mapping, 1, 1)
        grid.setRowStretch(2, 1)
        grid.setColumnStretch(0, 3)
        grid.setColumnStretch(1, 2)

        self.scale_slider.valueChanged.connect(self._mapping_changed)
        self.rotation_box.toggled.connect(self._mapping_changed)
        self.facing.currentIndexChanged.connect(self._mapping_changed)
        self._timer = QTimer(self)
        self._timer.setInterval(100)
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
            server = VrServer(self.session)
            try:
                server.start()
            except RuntimeError as e:
                self._log("error", f"{e}. Is another program using port {PORT}?")
                return
            self.server = server
            self.link.show_server(server)
            note = " (shared package certificate)" if server.cert_is_shared else ""
            self._log("info", f"VR server running: {', '.join(server.urls)}{note}")
        else:
            self.shutdown()

    def shutdown(self) -> None:
        if self.server is None:
            return
        self.server.stop()
        self.server = None
        self.link.show_server(None)
        self.session.on_disconnect()
        if self.worker.latest_state().mode is Mode.STREAMING:
            self.worker.request_stop()
        self._log("info", "VR server stopped")

    def _refresh(self) -> None:
        running = self.server is not None
        self.start_btn.setText("Stop VR server" if running else "Start VR server")
        if running:
            self.status_label.setText(f"headsets connected: {self.server.clients} · {self.server.hz:.0f} Hz · "
                                      f"bad messages {self.server.bad_messages}")
        else:
            self.status_label.setText("")
        self.hand_label.setText(hand_text(self.session, running and self.server.clients > 0))


def hand_text(session: VrSession, connected: bool) -> str:
    s = session.last_sample
    if not connected or s is None:
        return "no headset"
    if s.right is None:
        return "right controller NOT tracked (arm held)"
    h = s.right
    state = "clutch HELD" if session.clutch else "clutch released"
    return f"{state}\ngrip {h.grip * 100:3.0f} %   trigger {h.trigger * 100:3.0f} %"
