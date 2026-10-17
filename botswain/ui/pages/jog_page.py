"""Jog page, old app layout: Orientation | Control over Speed control | Robot view."""

from __future__ import annotations

from collections.abc import Callable

from PySide6.QtCore import Qt, QTimer
from PySide6.QtWidgets import QComboBox, QGridLayout, QHBoxLayout, QLabel, QPushButton, QWidget

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
    Mode,
    MoveTo,
    Pose,
    RobotState,
    SetSpeed,
    is_joint_axis,
)
from botswain.ui.theme import set_role
from botswain.ui.widgets.jog_pad import JogPad, StepSelector
from botswain.ui.widgets.panel import Panel
from botswain.ui.widgets.pose_fields import PoseFields
from botswain.ui.widgets.robot_view import RobotView
from botswain.ui.widgets.speed_control import SpeedControl

JOG_MODES = frozenset({Mode.IDLE, Mode.JOGGING})  # keys stay live during a hold-jog, greyed otherwise
TICK_MS = 50  # "still held" heartbeat; the worker releases the jog after 150 ms without one


class JogPage(QWidget):
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
        self.orientation.body_layout.addLayout(frame_row)
        self.pose_fields = PoseFields(caps.dof)
        self.pose_fields.set_limits(cfg.limits.joints_deg, cfg.limits.box_mm)
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
        self.control.body_layout.addStretch(1)
        for b in (self.gripper_open_btn, self.gripper_close_btn):
            b.setVisible(caps.gripper)

        self.speed = Panel("Speed control")
        self.speed_control = SpeedControl(cfg.limits.speed_cap_pct)
        self.speed.body_layout.addWidget(self.speed_control)

        urdf = cfg.model_urdf
        self.view = Panel("Robot view", urdf.name if urdf else "")
        self.robot_view = RobotView(urdf if view_3d else None, cfg.limits.box_mm, gripper=caps.gripper,
                                    box_on=cfg.limits.box_enabled)
        self.view.body_layout.setContentsMargins(6, 6, 6, 6)
        self.view.body_layout.addWidget(self.robot_view, 1)

        self._hold_timer = QTimer(self)
        self._hold_timer.setInterval(TICK_MS)
        self._hold_timer.timeout.connect(lambda: self._submit(JogTick()))
        self.jog_pad.step.connect(self._step)
        self.jog_pad.hold_start.connect(self._hold_start)
        self.jog_pad.hold_end.connect(self._hold_end)
        self.speed_control.changed.connect(lambda pct: self._submit(SetSpeed(pct)))
        self.gripper_open_btn.clicked.connect(lambda: self._submit(Gripper(True)))
        self.gripper_close_btn.clicked.connect(lambda: self._submit(Gripper(False)))
        self.copy_btn.clicked.connect(self._copy_current)
        self.move_btn.clicked.connect(self._move_to_target)

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

    def set_state(self, state: RobotState) -> None:
        self._state = state
        reason = not_ready_reason(state, JOG_MODES)
        self.jog_pad.set_blocked(reason)
        move_reason = not_ready_reason(state, frozenset({Mode.IDLE}))
        self.move_btn.setEnabled(move_reason is None)
        self.move_btn.setToolTip(move_reason or "")
        live = state.link_ok and state.mode is not Mode.DISCONNECTED
        for b in (self.gripper_open_btn, self.gripper_close_btn):
            b.setEnabled(live)
        self.pose_fields.set_state(state)
        self.speed_control.set_value(state.speed_pct)
        self.robot_view.set_joints(state.joints.values)
        self.robot_view.set_gripper(state.gripper)
        self.robot_view.set_target(state.tcp_target if state.tcp_target != state.tcp else None)
