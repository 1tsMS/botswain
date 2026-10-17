"""Top toolbar, Fusion style: grouped icon-over-label tools, robot state and the big STOP on the right."""

from __future__ import annotations

from PySide6.QtCore import QSize, Qt, Signal
from PySide6.QtWidgets import (
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSizePolicy,
    QToolBar,
    QToolButton,
    QVBoxLayout,
    QWidget,
)

from botswain.core.config import RobotConfig
from botswain.core.types import Capabilities, Mode, RobotState
from botswain.ui.icons import icon
from botswain.ui.theme import C, set_flag, set_role

ENABLED_MODES = {Mode.IDLE, Mode.MOVING, Mode.JOGGING, Mode.STREAMING, Mode.DRAG, Mode.PAUSED}


class TopToolbar(QToolBar):
    connect_clicked = Signal()
    doctor_clicked = Signal()
    enable_toggled = Signal(bool)  # True = operator wants it enabled
    clear_clicked = Signal()
    home_clicked = Signal()
    drag_clicked = Signal()
    record_clicked = Signal()
    save_point_clicked = Signal()

    def __init__(self, cfg: RobotConfig, caps: Capabilities, parent: QWidget | None = None) -> None:
        super().__init__("Main toolbar", parent)
        self.setObjectName("top")
        self.setMovable(False)
        self.setFloatable(False)
        self.cfg = cfg
        self._enabled = False

        self.connect_btn = self._tool("Connect", "connect", self.connect_clicked.emit)
        self.doctor_btn = self._tool("Doctor", "doctor", self.doctor_clicked.emit)
        self._group("Connection", [self.connect_btn, self.doctor_btn])

        self.enable_btn = self._tool("Enable", "locked", lambda: self.enable_toggled.emit(not self._enabled))
        self.clear_btn = self._tool("Clear error", "clear", self.clear_clicked.emit)
        self.home_btn = self._tool("Home", "home", self.home_clicked.emit)
        robot_tools = [self.enable_btn, self.clear_btn, self.home_btn]
        self.drag_btn = self._tool("Drag mode", "drag", self.drag_clicked.emit)
        if caps.drag_mode:
            robot_tools.append(self.drag_btn)
        self._group("Robot", robot_tools)

        self.record_btn = self._tool("Record", "record", self.record_clicked.emit)
        self.save_point_btn = self._tool("Save point", "save_point", self.save_point_clicked.emit)
        self._group("Data", [self.record_btn, self.save_point_btn])

        spacer = QWidget()
        spacer.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)
        self.addWidget(spacer)
        self.addWidget(self._state_widget(cfg))
        self.stop_button = self._stop_button()
        self.addWidget(self.stop_button)
        self.record_btn.setEnabled(False)
        self.set_state(RobotState.disconnected(caps.dof), connected=False)

    # ---- building ----

    def _tool(self, text: str, icon_name: str, slot) -> QToolButton:
        b = set_role(QToolButton(), "tool")
        b.setText(text)
        b.setIcon(icon(icon_name, C["chrome_text"], C["chrome_off"]))
        b.setIconSize(QSize(20, 20))
        b.setToolButtonStyle(Qt.ToolButtonStyle.ToolButtonTextUnderIcon)
        b.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        b.setSizePolicy(QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Expanding)
        b.clicked.connect(slot)
        return b

    def _group(self, name: str, tools: list[QToolButton]) -> None:
        g = set_role(QWidget(), "group")
        g.setAttribute(Qt.WidgetAttribute.WA_StyledBackground)
        v = QVBoxLayout(g)
        v.setContentsMargins(6, 6, 6, 4)
        v.setSpacing(2)
        row = QHBoxLayout()
        row.setSpacing(2)
        for t in tools:
            row.addWidget(t)
        v.addLayout(row, 1)
        label = set_role(QLabel(name.upper()), "groupName")
        label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        v.addWidget(label)
        g.setFixedHeight(76)
        self.addWidget(g)

    def _state_widget(self, cfg: RobotConfig) -> QWidget:
        w = QWidget()
        v = QVBoxLayout(w)
        v.setContentsMargins(8, 0, 12, 0)
        v.setSpacing(2)
        self.state_label = QLabel()
        self.state_label.setObjectName("stateText")
        where = cfg.connection.ip if cfg.connection else "no network"
        sub = QLabel(f"{cfg.name} · {where}")
        sub.setObjectName("stateSub")
        for label in (self.state_label, sub):
            label.setAlignment(Qt.AlignmentFlag.AlignRight)
            v.addWidget(label)
        return w

    def _stop_button(self) -> QPushButton:
        b = QPushButton()
        b.setObjectName("stop")
        b.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        b.setToolTip("Stop all motion (Space or Esc, works from anywhere in the app)")
        v = QVBoxLayout(b)
        v.setContentsMargins(0, 8, 0, 8)
        v.setSpacing(2)
        for text, name in (("STOP", "stopText"), ("Space / Esc", "stopKey")):
            label = QLabel(text)
            label.setObjectName(name)
            label.setAlignment(Qt.AlignmentFlag.AlignCenter)
            label.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents)
            v.addWidget(label)
        return b

    # ---- state ----

    def set_state(self, state: RobotState, connected: bool) -> None:
        self.connect_btn.setText("Disconnect" if connected else "Connect")
        self._enabled = state.mode in ENABLED_MODES
        self.enable_btn.setText("Disable" if self._enabled else "Enable")
        self.enable_btn.setIcon(icon("unlocked" if self._enabled else "locked", C["chrome_text"], C["chrome_off"]))
        set_flag(self.enable_btn, "on", self._enabled)
        live = connected and state.link_ok
        self.enable_btn.setEnabled(live and state.mode not in (Mode.ERROR, Mode.COLLISION) or self._enabled)
        self.clear_btn.setEnabled(live)
        self.save_point_btn.setEnabled(live)
        home_ok = self.cfg.home_confirmed and self.cfg.home_joints_deg is not None
        self.home_btn.setEnabled(home_ok and live and state.mode is Mode.IDLE)
        self.home_btn.setToolTip("" if home_ok else "Home pose not confirmed in the robot YAML (home_confirmed)")
        self.state_label.setText(state_text(state, connected))


def state_text(state: RobotState, connected: bool) -> str:
    """'Enabled · idle', 'Disabled', 'Error · 2 alarms', ... with a coloured dot."""
    if not connected or state.mode is Mode.DISCONNECTED:
        dot, text = C["chrome_off"], "Disconnected"
    elif not state.link_ok:
        dot, text = C["red"], "No feedback"
    elif state.mode in (Mode.ERROR, Mode.COLLISION):
        n = len(state.errors)
        count = f" · {n} alarm{'s' if n != 1 else ''}" if n else ""
        dot, text = C["red"], f"{state.mode.value.capitalize()}{count}"
    elif state.mode in ENABLED_MODES:
        dot, text = C["ok"], f"Enabled · {state.mode.value}"
    else:
        dot, text = C["warn"], state.mode.value.capitalize()
    return f'<span style="color:{dot}">●</span>&nbsp; {text}'
