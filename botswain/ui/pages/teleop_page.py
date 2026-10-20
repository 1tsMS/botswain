"""Teleop page: one input at a time on screen (Phone | Game controller | VR headset), each laid out the same way:
connect and status on the left, mapping and the controls on the right. Every input streams through the same
worker and safety gate; only one moves the arm at a time (the worker's stream owner)."""

from __future__ import annotations

from collections.abc import Callable

from PySide6.QtCore import Qt, QTimer
from PySide6.QtWidgets import (
    QButtonGroup,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QStackedWidget,
    QVBoxLayout,
    QWidget,
)

from botswain.control.worker import RobotWorker
from botswain.core.types import Mode
from botswain.teleop.mapping import TeleopMapper
from botswain.teleop.phone import CERT_DIR, PhoneServer, make_certificate
from botswain.teleop.session import TeleopSession
from botswain.teleop.vr import PORT as VR_PORT
from botswain.teleop.vr import VrServer, VrSession
from botswain.ui.theme import mono_font, set_role
from botswain.ui.widgets.gamepad_panel import GamepadPanel
from botswain.ui.widgets.pairing_link import PairingLink
from botswain.ui.widgets.panel import Panel
from botswain.ui.widgets.teleop_parts import MappingControls, key_table

PORT = 4443
STREAM_RULES = ("hold", "stale", "teleport", "step", "box")
INPUTS = ("Phone", "Game controller", "VR headset")

PHONE_KEYS = [("Hold button", "move the arm (let go = stop)"), ("Gripper button", "gripper open / close"),
              ("Scale slider", "the phone's own motion scale")]
PAD_KEYS = [("LB (hold)", "move the arm (let go = stop)"), ("Left stick", "X / Y"), ("RT / LT", "Z up / down"),
            ("Right stick", "Rz / Ry"), ("RB", "slow mode (quarter speed)"), ("A", "gripper open / close"),
            ("D-pad up / down", "speed +/- 10 %"), ("Start", "recording: start, or save the episode"),
            ("Back", "discard the episode"), ("X", "save the current pose as a point")]
VR_KEYS = [("Right grip (squeeze)", "clutch: hold to move; let go, move your hand, squeeze again to re-centre"),
           ("Right trigger", "gripper open / close"),
           ("Lost tracking / page closed", "the arm stops at once")]


def _button(text: str, role: str, slot) -> QPushButton:
    b = set_role(QPushButton(text), role)
    b.setFocusPolicy(Qt.FocusPolicy.NoFocus)
    b.clicked.connect(lambda: slot())
    return b


def _row(*widgets: QWidget) -> QHBoxLayout:
    """Buttons at their natural size, left-aligned (not stretched across the panel)."""
    row = QHBoxLayout()
    for w in widgets:
        row.addWidget(w)
    row.addStretch(1)
    return row


def _note(text: str) -> QLabel:
    label = set_role(QLabel(text), "lab")
    label.setWordWrap(True)
    return label


def _two_columns(left: list[QWidget], right: list[QWidget]) -> QWidget:
    page = QWidget()
    grid = QGridLayout(page)
    grid.setContentsMargins(0, 0, 0, 0)
    grid.setSpacing(8)
    for col, widgets in enumerate((left, right)):
        column = QVBoxLayout()
        column.setSpacing(8)
        for w in widgets:
            column.addWidget(w)
        column.addStretch(1)
        grid.addLayout(column, 0, col)
    grid.setColumnStretch(0, 3)
    grid.setColumnStretch(1, 2)
    return page


class TeleopPage(QWidget):
    def __init__(self, worker: RobotWorker, log: Callable[[str, str], None], parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.worker = worker
        self._log = log
        self.server: PhoneServer | None = None
        self.mapper = TeleopMapper(scale_pos=0.3, scale_rot=0.0)
        self.session = TeleopSession(worker, self.mapper)
        self.vr_server: VrServer | None = None
        self.vr_mapper = TeleopMapper(scale_pos=0.5, scale_rot=0.0)
        self.vr_session = VrSession(worker, self.vr_mapper)

        lay = QVBoxLayout(self)
        lay.setContentsMargins(8, 8, 8, 8)
        lay.setSpacing(8)
        lay.addLayout(self._input_bar())
        self.stack = QStackedWidget()
        self.stack.addWidget(self._phone_view())
        self.stack.addWidget(self._gamepad_view())
        self.stack.addWidget(self._vr_view())
        lay.addWidget(self.stack, 1)
        self.counters = set_role(QLabel(""), "lab")
        lay.addWidget(self.counters)

        self._timer = QTimer(self)
        self._timer.setInterval(150)
        self._timer.timeout.connect(self._refresh)
        self._timer.start()
        self._refresh()

    # ---- input selector ----

    def _input_bar(self) -> QHBoxLayout:
        bar = QHBoxLayout()
        bar.addWidget(set_role(QLabel("Input"), "lab"))
        self._group = QButtonGroup(self)
        self.input_buttons: list[QPushButton] = []
        seg = QHBoxLayout()
        seg.setSpacing(0)
        for i, name in enumerate(INPUTS):
            b = set_role(QPushButton(name), "seg")
            b.setProperty("pos", "first" if i == 0 else "last" if i == len(INPUTS) - 1 else "mid")
            b.setCheckable(True)
            b.setFocusPolicy(Qt.FocusPolicy.NoFocus)
            b.clicked.connect(lambda _=False, i=i: self.show_input(i))
            self._group.addButton(b)
            self.input_buttons.append(b)
            seg.addWidget(b)
        self.input_buttons[0].setChecked(True)
        bar.addLayout(seg)
        bar.addStretch(1)
        self.owner_label = QLabel("")
        self.owner_label.setFont(mono_font(12))
        bar.addWidget(self.owner_label)
        return bar

    def show_input(self, i: int) -> None:
        self.input_buttons[i].setChecked(True)
        self.stack.setCurrentIndex(i)

    # ---- phone ----

    def _phone_view(self) -> QWidget:
        phone = Panel("Phone (WebXR)", f"HTTPS port {PORT}")
        self.start_btn = _button("Start phone server", "primary", self.toggle_server)
        phone.body_layout.addLayout(_row(self.start_btn))
        self.link = PairingLink()  # scan the QR code, or open https://<ip>:4443 and type the key
        phone.body_layout.addWidget(self.link)
        self.status_label = set_role(QLabel(""), "lab")
        phone.body_layout.addWidget(self.status_label)
        phone.body_layout.addWidget(_note(
            "Android Chrome on a phone with ARCore, on the same Wi-Fi as this laptop. Accept the certificate "
            f"warning, then start AR. Windows Firewall must allow inbound TCP {PORT} (add the rule yourself)."))

        cert = Panel("HTTPS certificate")
        self.cert_btn = _button("Make this laptop's certificate", "btn", self.make_cert)
        cert.body_layout.addLayout(_row(self.cert_btn))
        cert.body_layout.addWidget(_note(
            "The phone needs HTTPS. Without your own certificate Botswain uses the teleop package's, whose private "
            "key is public. This makes a key that only this laptop has (needs Git for Windows, one time)."))
        self.cert_label = set_role(QLabel(""), "lab")
        cert.body_layout.addWidget(self.cert_label)

        mapping = Panel("Mapping", "first runs: small and slow")
        mapping.body_layout.addWidget(MappingControls(self.mapper, scale_pct=30, what="phone"))
        controls = Panel("Controls", "on the phone page")
        controls.body_layout.addWidget(key_table(PHONE_KEYS))
        return _two_columns([phone, cert], [mapping, controls])

    def make_cert(self) -> None:
        try:
            path = make_certificate()
        except (RuntimeError, OSError) as e:
            self._log("error", f"certificate: {e}")
            return
        except Exception as e:  # noqa: BLE001 -- openssl failed: show its message
            self._log("error", f"certificate: openssl failed ({e})")
            return
        self._log("info", f"own certificate written to {path}; restart the servers to use it")

    def toggle_server(self) -> None:
        if self.server is None:
            server = PhoneServer(self.session.on_sample, port=PORT)
            try:
                server.start()
            except RuntimeError as e:
                self._log("error", f"{e}. Is another program using port {PORT}?")
                return
            self.server = server
            self.link.show_server(server)
            note = " (shared package certificate)" if server.cert_is_shared else ""
            self._log("info", f"phone server running: {', '.join(server.urls)}{note}")
        else:
            self.server.stop()
            self.server = None
            self.link.show_server(None)
            self._stop_if_streaming()
            self._log("info", "phone server stopped")

    # ---- game controller ----

    def _gamepad_view(self) -> QWidget:
        self.gamepad = GamepadPanel(self.worker, self._log)
        mapping = Panel("Mapping")
        mapping.body_layout.addWidget(MappingControls(
            None, what="you", on_remap=lambda m: setattr(self.gamepad.teleop, "remap", m)))
        controls = Panel("Controls", "Xbox-style, X-input mode")
        controls.body_layout.addWidget(key_table(PAD_KEYS))
        return _two_columns([self.gamepad], [mapping, controls])

    # ---- VR headset ----

    def _vr_view(self) -> QWidget:
        headset = Panel("VR headset (WebXR)", f"Meta Quest Browser · HTTPS port {VR_PORT}")
        self.vr_btn = _button("Start VR server", "primary", self.toggle_vr)
        headset.body_layout.addLayout(_row(self.vr_btn))
        self.vr_link = PairingLink()
        headset.body_layout.addWidget(self.vr_link)
        self.vr_status = set_role(QLabel(""), "lab")
        headset.body_layout.addWidget(self.vr_status)
        headset.body_layout.addWidget(_note(
            "1. Windows Settings > Network > Mobile hotspot: on (no internet needed). 2. Quest: join that Wi-Fi, "
            "open Meta Quest Browser, go to the https address above and type the key. 3. Accept the certificate "
            f"warning, press Enter. Windows Firewall must allow inbound TCP {VR_PORT}."))
        hand = Panel("Right hand", "live")
        self.hand_label = QLabel("no headset")
        self.hand_label.setFont(mono_font(14))
        hand.body_layout.addWidget(self.hand_label)
        mapping = Panel("Mapping", "first runs: small and slow")
        mapping.body_layout.addWidget(MappingControls(self.vr_mapper, scale_pct=50, what="hand"))
        controls = Panel("Controls", "right controller")
        controls.body_layout.addWidget(key_table(VR_KEYS))
        return _two_columns([headset, hand], [mapping, controls])

    def toggle_vr(self) -> None:
        if self.vr_server is None:
            server = VrServer(self.vr_session)
            try:
                server.start()
            except RuntimeError as e:
                self._log("error", f"{e}. Is another program using port {VR_PORT}?")
                return
            self.vr_server = server
            self.vr_link.show_server(server)
            self._log("info", f"VR server running: {', '.join(server.urls)}")
        else:
            self.vr_server.stop()
            self.vr_server = None
            self.vr_link.show_server(None)
            self.vr_session.on_disconnect()
            self._stop_if_streaming()
            self._log("info", "VR server stopped")

    # ---- shared ----

    def _stop_if_streaming(self) -> None:
        if self.worker.latest_state().mode is Mode.STREAMING:
            self.worker.request_stop()

    def shutdown(self) -> None:
        self.gamepad.shutdown()
        if self.server is not None:
            self.toggle_server()
        if self.vr_server is not None:
            self.toggle_vr()

    def _refresh(self) -> None:
        running = self.server is not None
        self.start_btn.setText("Stop phone server" if running else "Start phone server")
        if running:
            s = self.session.last_sample
            hold = "holding" if s and s.hold else "not holding"
            self.status_label.setText(f"phones connected: {self.server.clients} · {self.server.hz:.0f} Hz · {hold}")
        else:
            self.status_label.setText("")
        own = (CERT_DIR / "cert.pem").exists()
        self.cert_label.setText("This laptop has its own certificate (assets/certs)." if own else
                                "Using the teleop package's shared certificate.")
        self.cert_btn.setEnabled(self.server is None and self.vr_server is None)

        vr = self.vr_server is not None
        self.vr_btn.setText("Stop VR server" if vr else "Start VR server")
        self.vr_status.setText(f"headsets connected: {self.vr_server.clients} · {self.vr_server.hz:.0f} Hz · "
                               f"bad messages {self.vr_server.bad_messages}" if vr else "")
        self.hand_label.setText(hand_text(self.vr_session, vr and self.vr_server.clients > 0))

        owner = self.worker.stream_owner
        self.owner_label.setText(f"moving the arm: {owner}" if owner else "")
        counts = self.worker.gate.counts
        self.counters.setText("Safety gate: " + " · ".join(f"{r} {counts.get(r, 0)}" for r in STREAM_RULES))


def hand_text(session: VrSession, connected: bool) -> str:
    s = session.last_sample
    if not connected or s is None:
        return "no headset"
    if s.right is None:
        return "right controller NOT tracked (arm held)"
    h = s.right
    state = "clutch HELD" if session.clutch else "clutch released"
    return f"{state}\ngrip {h.grip * 100:3.0f} %   trigger {h.trigger * 100:3.0f} %"
