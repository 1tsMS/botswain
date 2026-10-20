"""Jog page, old app layout: Orientation | Control over Speed control | Robot view."""

from __future__ import annotations

from collections.abc import Callable

from PySide6.QtCore import Qt, QTimer, Signal
from PySide6.QtWidgets import QComboBox, QGridLayout, QHBoxLayout, QLabel, QPushButton, QSpinBox, QWidget

from botswain.control.box_teach import box_from_corners, flat_axes
from botswain.control.safety import not_ready_reason
from botswain.core.config import RobotConfig
from botswain.core.types import (
    Capabilities,
    Command,
    Gripper,
    JogHold,
    JogRelease,
    JogStep,
    JogTick,
    Joints,
    Mode,
    MoveTo,
    Pose,
    RobotState,
    SetGripperForce,
    SetSpeed,
    is_joint_axis,
)
from botswain.ui.theme import mono_font, set_role
from botswain.ui.widgets.jog_pad import JogPad, StepSelector
from botswain.ui.widgets.panel import Panel
from botswain.ui.widgets.pose_fields import PoseFields
from botswain.ui.widgets.robot_view import RobotView
from botswain.ui.widgets.speed_control import SpeedControl

JOG_MODES = frozenset({Mode.IDLE, Mode.JOGGING})  # keys stay live during a hold-jog, greyed otherwise
# While a key is held, "moving" is expected, not a reason to let go: older Dobot firmware may report a running
# MoveJog as mode 7 (running) instead of 8 (jog), and greying the keys then released the jog after one frame.
HOLD_MODES = JOG_MODES | {Mode.MOVING}
TICK_MS = 50  # "still held" heartbeat; the worker releases the jog after 150 ms without one


class JogPage(QWidget):
    box_taught = Signal(object)  # the box from the taught corners, to apply and save
    def __init__(self, cfg: RobotConfig, caps: Capabilities, submit: Callable[[Command], None],
                 log: Callable[[str, str], None], view_3d: bool = True, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._submit = submit
        self._log = log
        self._state: RobotState | None = None
        grid = QGridLayout(self)
        grid.setContentsMargins(8, 8, 8, 8)
        grid.setSpacing(8)
        for col, stretch in enumerate((20, 21, 20)):
            grid.setColumnStretch(col, stretch)
        grid.setRowStretch(0, 1)
        grid.setRowStretch(1, 0)

        # Orientation: frame, live pose + joints, target buttons.
        self.orientation = Panel("Orientation", "live")
        frame_row = QHBoxLayout()
        frame_row.addWidget(set_role(QLabel("Frame"), "lab"))
        self.frame_box = QComboBox()
        self.frame_box.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        for frame in caps.frames:
            self.frame_box.addItem(frame.capitalize(), frame)
        frame_row.addWidget(self.frame_box)
        frame_row.addStretch(1)
        # Same move as the toolbar's Home: a joint move to home_joints_deg, only once the YAML confirms it.
        self._home = cfg.home_joints_deg if cfg.home_confirmed else None
        self.home_btn = self._button("Home", "btn")
        self.home_btn.setToolTip("" if self._home else "Home pose not confirmed in the robot YAML (home_confirmed)")
        frame_row.addWidget(self.home_btn)
        self.orientation.body_layout.addLayout(frame_row)
        self.pose_fields = PoseFields(caps.dof)
        self.pose_fields.set_limits(cfg.limits.joints_deg, cfg.limits.box_mm)
        self.pose_fields_joint_limits = cfg.limits.joints_deg
        self.orientation.body_layout.addWidget(self.pose_fields)
        self.orientation.body_layout.addStretch(1)
        buttons = QHBoxLayout()
        self.edit_btn = self._button("Edit target", "btn", checkable=True)
        self.copy_btn = self._button("Copy current", "btn")
        self.move_btn = self._button("Move to target", "primary")
        for b in (self.edit_btn, self.copy_btn, self.move_btn):
            buttons.addWidget(b)
        self.orientation.body_layout.addLayout(buttons)
        self.edit_btn.toggled.connect(self._edit_toggled)
        self._edit_toggled(False)

        # Control: step selector in the header, jog keys, gripper.
        self.control = Panel("Control", "step")
        self.step_selector = StepSelector(hold_allowed=caps.hold_jog)
        self.control.add_head_widget(self.step_selector)
        self.jog_pad = JogPad(caps.dof)
        self.step_selector.changed.connect(self.jog_pad.set_step_size)
        self.control.body_layout.addWidget(self.jog_pad)
        gripper_row = QHBoxLayout()
        self.gripper_open_btn = self._button("Gripper open", "btn")
        self.gripper_close_btn = self._button("Gripper close", "btn")
        gripper_row.addWidget(self.gripper_open_btn)
        gripper_row.addWidget(self.gripper_close_btn)
        self.control.body_layout.addLayout(gripper_row)
        # Readout (what the gripper itself reports) and the closing force. A DH PGE has no force sensor: it closes
        # until its motor reaches the set force, then holds and reports "object caught".
        self.gripper_status = QLabel("")
        self.gripper_status.setFont(mono_font(12))
        force_row = QHBoxLayout()
        self.force_label = set_role(QLabel("Close force"), "lab")
        self.force_box = QSpinBox()
        self.force_box.setRange(20, 100)
        self.force_box.setSingleStep(10)
        self.force_box.setValue(50)
        self.force_box.setSuffix(" %")
        self.force_box.setKeyboardTracking(False)  # typing "80" sends 80, not 8 then 80
        self.force_box.setToolTip("Closing force for every close (buttons, gamepad, phone). The gripper stops when "
                                  "it reaches it. DH PGE-50-26: 20-100 % covers about 15-50 N per jaw.")
        force_row.addWidget(self.force_label)
        force_row.addWidget(self.force_box)
        force_row.addStretch(1)
        self.control.body_layout.addWidget(self.gripper_status)
        self.control.body_layout.addLayout(force_row)
        self.control.body_layout.addStretch(1)
        self._force_sent: int | None = None
        for w in (self.gripper_open_btn, self.gripper_close_btn, self.gripper_status):
            w.setVisible(caps.gripper)
        for w in (self.force_label, self.force_box):
            w.setVisible(caps.gripper_force)
        self._has_force = caps.gripper_force

        self.speed = Panel("Speed control")
        self.speed_control = SpeedControl(cfg.limits.speed_cap_pct)
        self.speed.body_layout.addWidget(self.speed_control)

        urdf = cfg.model_urdf
        self.view = Panel("Robot view", urdf.name if urdf else "")
        self.robot_view = RobotView(urdf if view_3d else None, cfg.limits.box_mm, gripper=caps.gripper,
                                    box_on=cfg.limits.box_enabled)
        self.view.body_layout.setContentsMargins(6, 6, 6, 6)
        self.view.body_layout.addWidget(self.robot_view, 1)
        self.current_box = cfg.limits.box_mm  # the box in use (Settings or a taught one may replace it)
        self.box_corners: list[tuple[float, float, float]] = []
        self.robot_view.corner_requested.connect(self._set_corner)
        self.robot_view.corners_cleared.connect(self._clear_corners)
        self.robot_view.box_apply_requested.connect(self._apply_corners)

        self._hold_timer = QTimer(self)
        self._hold_timer.setInterval(TICK_MS)
        self._hold_timer.timeout.connect(lambda: self._submit(JogTick()))
        self.jog_pad.step.connect(self._step)
        self.jog_pad.hold_start.connect(self._hold_start)
        self.jog_pad.hold_end.connect(self._hold_end)
        self.speed_control.changed.connect(lambda pct: self._submit(SetSpeed(pct)))
        self.gripper_open_btn.clicked.connect(lambda: self._submit(Gripper(True)))
        self.gripper_close_btn.clicked.connect(self._close_gripper)
        self.force_box.valueChanged.connect(self._send_force)
        self.copy_btn.clicked.connect(self._copy_current)
        self.move_btn.clicked.connect(self._move_to_target)
        self.home_btn.clicked.connect(self._go_home)

        grid.addWidget(self.orientation, 0, 0, 2, 1)
        grid.addWidget(self.control, 0, 1)
        grid.addWidget(self.speed, 1, 1)
        grid.addWidget(self.view, 0, 2, 2, 1)

    def _button(self, text: str, role: str, checkable: bool = False) -> QPushButton:
        b = set_role(QPushButton(text), role)
        b.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        b.setCheckable(checkable)
        return b

    def _edit_toggled(self, on: bool) -> None:
        self.pose_fields.set_editing(on)
        self.copy_btn.setEnabled(on)
        self.orientation.set_meta("editing target" if on else "live")

    @property
    def frame(self) -> str:
        return self.frame_box.currentData()

    # ---- operator actions -> commands (all go through the worker and the gate) ----

    def _frame_for(self, axis: str) -> str:
        return "base" if is_joint_axis(axis) else self.frame

    def _step(self, axis: str, sign: int) -> None:
        self._submit(JogStep(axis, sign * self.jog_pad.step_size, self._frame_for(axis)))

    def _hold_start(self, axis: str, sign: int) -> None:
        self._submit(JogHold(axis, sign, self._frame_for(axis)))
        self._hold_timer.start()

    def _hold_end(self) -> None:
        self._hold_timer.stop()
        self._submit(JogRelease())

    def _copy_current(self) -> None:
        if self._state is not None:
            self.pose_fields.copy_from(self._state)

    def _move_to_target(self) -> None:
        try:
            target = self.pose_fields.target()
        except ValueError as e:
            self._log("error", f"Move to target: {e}")
            return
        self._submit(MoveTo(target, "linear" if isinstance(target, Pose) else "joint"))
        self.edit_btn.setChecked(False)  # back to live: the fields follow the arm through the move

    def _send_force(self, pct: int) -> None:
        self._force_sent = pct
        self._submit(SetGripperForce(pct))

    def _close_gripper(self) -> None:
        if self._has_force and self._force_sent != self.force_box.value():
            self._send_force(self.force_box.value())  # the box shows 50 % from the start: make that true
        self._submit(Gripper(False))

    # ---- teaching the safety box (Robot view > Box) ----

    def _set_corner(self, index: int) -> None:
        if self._state is None or not self._state.link_ok:
            self._log("warn", "Box: needs a connected robot with live feedback")
            return
        tcp = self._state.tcp.xyz
        if index < 0 or index >= len(self.box_corners):
            self.box_corners.append(tcp)
        else:
            self.box_corners[index] = tcp
        n = len(self.box_corners)
        self._log("info", f"Box corner {n if index < 0 else index + 1} at X {tcp[0]:.1f} Y {tcp[1]:.1f} "
                          f"Z {tcp[2]:.1f} ({n} corner{'s' if n != 1 else ''})")
        self._preview()

    def _clear_corners(self) -> None:
        self.box_corners.clear()
        self.robot_view.set_preview_box(None)
        self._log("info", "Box corners cleared")

    def _preview(self) -> None:
        if len(self.box_corners) < 2:
            return
        box = box_from_corners(self.box_corners, self.current_box)
        self.robot_view.set_preview_box(box)
        flat = flat_axes(self.box_corners)
        size = " x ".join(f"{hi - lo:.0f}" for lo, hi in box)
        note = f"; {', '.join(a.upper() for a in flat)} kept from the current box" if flat else ""
        self._log("info", f"Box preview {size} mm{note}. Box > Apply & save to use it")

    def _apply_corners(self) -> None:
        if len(self.box_corners) < 2:
            self._log("warn", "Box: set at least two corners first")
            return
        self.box_taught.emit(box_from_corners(self.box_corners, self.current_box))

    def show_box(self, box_mm: tuple) -> None:
        """A new box is in use (taught here or typed in Settings)."""
        self.current_box = box_mm
        self.box_corners.clear()
        self.robot_view.set_preview_box(None)
        self.robot_view.set_box(box_mm)
        self.pose_fields.set_limits(self.pose_fields_joint_limits, box_mm)

    def _go_home(self) -> None:
        if self._home is not None:
            self._submit(MoveTo(Joints(self._home), "joint"))

    def set_state(self, state: RobotState) -> None:
        self._state = state
        holding = self.jog_pad.holding
        reason = not_ready_reason(state, HOLD_MODES if holding else JOG_MODES)
        if holding and reason is not None:
            self._log("warn", f"hold-jog released: {reason}")
        self.jog_pad.set_blocked(reason)
        move_reason = not_ready_reason(state, frozenset({Mode.IDLE}))
        self.move_btn.setEnabled(move_reason is None)
        self.move_btn.setToolTip(move_reason or "")
        self.home_btn.setEnabled(self._home is not None and move_reason is None)
        if self._home is not None:
            self.home_btn.setToolTip(move_reason or f"joint move to {', '.join(f'{v:g}' for v in self._home)}")
        live = state.link_ok and state.mode is not Mode.DISCONNECTED
        for w in (self.gripper_open_btn, self.gripper_close_btn, self.force_box):
            w.setEnabled(live)
        self.gripper_status.setText(_gripper_text(state) if live else "")
        self.pose_fields.set_state(state)
        self.speed_control.set_value(state.speed_pct)
        self.robot_view.set_joints(state.joints.values)
        self.robot_view.set_gripper(state.gripper)
        self.robot_view.set_target(state.tcp_target if state.tcp_target != state.tcp else None)


def _gripper_text(state: RobotState) -> str:
    if state.gripper_pos is None:
        return f"Gripper: {state.gripper_status}" if state.gripper_status else "Gripper: no reading"
    return f"Gripper: {state.gripper_status} · {state.gripper_pos * 100:.0f} % open"
