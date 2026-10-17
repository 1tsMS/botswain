"""Jog page, old app layout: Orientation | Control over Speed control | Robot view."""

from __future__ import annotations

from PySide6.QtCore import Qt
from PySide6.QtWidgets import QComboBox, QGridLayout, QHBoxLayout, QLabel, QPushButton, QWidget

from botswain.core.config import RobotConfig
from botswain.core.types import Capabilities, RobotState
from botswain.ui.theme import set_role
from botswain.ui.widgets.jog_pad import JogPad, StepSelector
from botswain.ui.widgets.panel import Panel
from botswain.ui.widgets.pose_fields import PoseFields
from botswain.ui.widgets.robot_view import RobotView
from botswain.ui.widgets.speed_control import SpeedControl


class JogPage(QWidget):
    def __init__(self, cfg: RobotConfig, caps: Capabilities, view_3d: bool = True,
                 parent: QWidget | None = None) -> None:
        super().__init__(parent)
        grid = QGridLayout(self)
        grid.setContentsMargins(8, 8, 8, 8)
        grid.setSpacing(8)
        for col, stretch in enumerate((20, 21, 20)):
            grid.setColumnStretch(col, stretch)
        grid.setRowStretch(0, 1)

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
        self.control.body_layout.addStretch(1)
        gripper_row = QHBoxLayout()
        self.gripper_open_btn = self._button("Gripper open", "btn")
        self.gripper_close_btn = self._button("Gripper close", "btn")
        gripper_row.addWidget(self.gripper_open_btn)
        gripper_row.addWidget(self.gripper_close_btn)
        self.control.body_layout.addLayout(gripper_row)
        for b in (self.gripper_open_btn, self.gripper_close_btn):
            b.setVisible(caps.gripper)

        self.speed = Panel("Speed control")
        self.speed_control = SpeedControl(cfg.limits.speed_cap_pct)
        self.speed.body_layout.addWidget(self.speed_control)

        urdf = cfg.model_urdf
        self.view = Panel("Robot view", urdf.name if urdf else "")
        self.robot_view = RobotView(urdf if view_3d else None, cfg.limits.box_mm)
        self.view.body_layout.setContentsMargins(6, 6, 6, 6)
        self.view.body_layout.addWidget(self.robot_view, 1)

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

    def set_state(self, state: RobotState) -> None:
        self.pose_fields.set_state(state)
        self.speed_control.set_value(state.speed_pct)
        self.robot_view.set_joints(state.joints.values)
        self.robot_view.set_target(state.tcp_target if state.tcp_target != state.tcp else None)
