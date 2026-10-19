"""Phone teleop page: start the phone server, show its address, tune the mapping, watch the safety counters."""

from __future__ import annotations

import io
from collections.abc import Callable

import segno
from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QPixmap
from PySide6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSlider,
    QVBoxLayout,
    QWidget,
)

from botswain.control.worker import RobotWorker
from botswain.core.types import Mode
from botswain.teleop.mapping import TeleopMapper, remap_about_z
from botswain.teleop.phone import CERT_DIR, PhoneServer, make_certificate
from botswain.teleop.session import TeleopSession
from botswain.ui.theme import mono_font, set_role
from botswain.ui.widgets.gamepad_panel import GamepadPanel
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
        # Scan the QR code with the phone camera, or open https://<ip>:4443 and type the key.
        self.qr_label = QLabel()
        self.qr_label.setVisible(False)
        self.key_label = QLabel("")
        self.key_label.setFont(mono_font(28))
        self.key_label.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        self.url_label = QLabel("Server off")
        self.url_label.setFont(mono_font(14))
        self.url_label.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        link_row = QHBoxLayout()
        link_row.addWidget(self.qr_label, 0, Qt.AlignmentFlag.AlignTop)
        link_text = QVBoxLayout()
        link_text.addWidget(self.key_label)
        link_text.addWidget(self.url_label)
        link_text.addStretch(1)
        link_row.addLayout(link_text, 1)
        self.status_label = set_role(QLabel(""), "lab")
        notes = set_role(QLabel(
            "Android Chrome on a phone with ARCore. Same WiFi as this laptop. Accept the certificate warning, "
            "then start AR. Hold the on-screen button to move; let go to stop. Windows Firewall must allow inbound "
            f"TCP {PORT} (add the rule yourself)."), "lab")
        notes.setWordWrap(True)
        self.cert_btn = set_role(QPushButton("Make my own HTTPS certificate"), "btn")
        self.cert_btn.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.cert_btn.clicked.connect(self.make_cert)
        self.cert_label = set_role(QLabel(""), "lab")
        self.cert_label.setWordWrap(True)
        phone.body_layout.addWidget(self.start_btn)
        phone.body_layout.addLayout(link_row)
        for w in (self.status_label, notes, self.cert_btn, self.cert_label):
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
        self.gamepad = GamepadPanel(worker, log)
        grid.addWidget(self.gamepad, 1, 0, 1, 2)
        grid.setRowStretch(2, 1)  # spare height goes below the panels, not into them
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
        self.gamepad.teleop.remap = self.mapper.remap  # one 'facing' setting for every teleop input
        self.mapper.request_reanchor()  # never let a settings change make the target jump
        self.scale_value.setText(f"{self.mapper.scale_pos:.2f}")

    def make_cert(self) -> None:
        try:
            path = make_certificate()
        except (RuntimeError, OSError) as e:
            self._log("error", f"certificate: {e}")
            return
        except Exception as e:  # noqa: BLE001 -- openssl failed: show its message
            self._log("error", f"certificate: openssl failed ({e})")
            return
        self._log("info", f"own certificate written to {path}; restart the phone server to use it")

    def toggle_server(self) -> None:
        if self.server is None:
            server = PhoneServer(self.session.on_sample, port=PORT)
            try:
                server.start()
            except RuntimeError as e:
                self._log("error", f"{e}. Is another program using port {PORT}?")
                return
            self.server = server
            self._show_link(server)
            note = " (shared package certificate)" if server.cert_is_shared else ""
            self._log("info", f"phone server running: {', '.join(server.urls)}{note}")
        else:
            self.shutdown()

    def shutdown(self) -> None:
        self.gamepad.shutdown()
        if self.server is None:
            return
        self.server.stop()
        self.server = None
        self.qr_label.setVisible(False)
        self.key_label.setText("")
        if self.worker.latest_state().mode is Mode.STREAMING:
            self.worker.request_stop()
        self._log("info", "phone server stopped")

    def _show_link(self, server: PhoneServer) -> None:
        """QR code of the first address (Wi-Fi first, robot LAN last) and the key to type by hand."""
        buf = io.BytesIO()
        segno.make(server.urls[0], error="m").save(buf, kind="png", scale=5, border=2)
        pixmap = QPixmap()
        pixmap.loadFromData(buf.getvalue(), "PNG")
        self.qr_label.setPixmap(pixmap)
        self.qr_label.setVisible(True)
        self.key_label.setText(f"Key {server.token}")

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
        own = (CERT_DIR / "cert.pem").exists()
        self.cert_label.setText("Certificate: this laptop's own (assets/certs)." if own else
                                "Certificate: the teleop package's shared one (its private key is public). "
                                "Fine on a lab network; better: make your own (needs Git for Windows).")
        self.cert_btn.setEnabled(self.server is None)
        counts = self.worker.gate.counts
        self.counters.setText("Gate: " + " · ".join(f"{r} {counts.get(r, 0)}" for r in STREAM_RULES))
