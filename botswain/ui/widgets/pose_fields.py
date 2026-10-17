"""Live TCP pose and joint fields.

Read-only and following the robot until "Edit target" is switched on. Typing never moves anything:
target() only reads the numbers, and only the Move to target button sends them (through the gate).
Fields turn red near a real limit from the robot YAML (5 deg from a joint limit, 5 mm from a box wall).
"""

from __future__ import annotations

from PySide6.QtCore import Qt
from PySide6.QtGui import QDoubleValidator
from PySide6.QtWidgets import QFrame, QGridLayout, QLabel, QLineEdit, QVBoxLayout, QWidget

from botswain.core.types import AXES_CART, Joints, Pose, RobotState
from botswain.ui.theme import set_flag, set_role

NEAR_DEG = 5.0
NEAR_MM = 5.0


class PoseFields(QWidget):
    def __init__(self, dof: int = 6, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.dof = dof
        self._editing = False
        self._edited = "pose"  # which group the operator typed in last
        self._joint_limits: tuple[tuple[float, float], ...] | None = None
        self._box: tuple[tuple[float, float], ...] | None = None

        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(8)
        lay.addWidget(set_role(QLabel("Tool pose (mm / deg)"), "lab"))
        self.pose_edits: dict[str, QLineEdit] = {}
        lay.addLayout(self._grid(list(AXES_CART), self.pose_edits, "pose"))
        lay.addWidget(set_role(QFrame(), "hr"))
        lay.addWidget(set_role(QLabel("Joints (deg)"), "lab"))
        self.joint_edits: dict[str, QLineEdit] = {}
        lay.addLayout(self._grid([f"J{i + 1}" for i in range(dof)], self.joint_edits, "joints"))
        self.set_editing(False)

    def _grid(self, names: list[str], store: dict[str, QLineEdit], group: str) -> QGridLayout:
        """Two columns like the mockup: first half on the left, second half on the right."""
        grid = QGridLayout()
        grid.setHorizontalSpacing(8)
        grid.setVerticalSpacing(6)
        half = (len(names) + 1) // 2
        for i, name in enumerate(names):
            row, col = i % half, (i // half) * 2
            edit = set_role(QLineEdit(), "field")
            edit.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
            edit.setValidator(QDoubleValidator(-100000.0, 100000.0, 3))
            edit.textEdited.connect(lambda _text, g=group: setattr(self, "_edited", g))
            label = set_role(QLabel(name), "axis")
            label.setFixedWidth(22)
            grid.addWidget(label, row, col)
            grid.addWidget(edit, row, col + 1)
            store[name] = edit
        return grid

    @property
    def editing(self) -> bool:
        return self._editing

    def set_editing(self, on: bool) -> None:
        self._editing = on
        for edit in (*self.pose_edits.values(), *self.joint_edits.values()):
            edit.setReadOnly(not on)
            edit.setFocusPolicy(Qt.FocusPolicy.StrongFocus if on else Qt.FocusPolicy.NoFocus)

    def set_limits(self, joints_deg: tuple[tuple[float, float], ...], box_mm: tuple[tuple[float, float], ...]) -> None:
        self._joint_limits = joints_deg
        self._box = box_mm

    def set_state(self, state: RobotState) -> None:
        if self._editing:
            return  # the operator is typing a target; don't overwrite it
        self.copy_from(state)
        for i, axis in enumerate(AXES_CART[:3]):
            set_flag(self.pose_edits[axis], "near", self._near(state.tcp.axis(axis), self._box, i, NEAR_MM))
        for i, (name, edit) in enumerate(self.joint_edits.items()):
            set_flag(edit, "near", self._near(state.joints.axis(name), self._joint_limits, i, NEAR_DEG))

    def copy_from(self, state: RobotState) -> None:
        for axis, edit in self.pose_edits.items():
            edit.setText(f"{state.tcp.axis(axis):.1f}")
        for name, edit in self.joint_edits.items():
            edit.setText(f"{state.joints.axis(name):.2f}")

    def target(self) -> Pose | Joints:
        """The numbers in the group the operator edited last. Raises ValueError on an empty or bad field."""
        if self._edited == "joints":
            return Joints(tuple(_number(e) for e in self.joint_edits.values()))
        return Pose(*(_number(e) for e in self.pose_edits.values()))

    @staticmethod
    def _near(value: float, limits: tuple[tuple[float, float], ...] | None, i: int, margin: float) -> bool:
        if limits is None or i >= len(limits):
            return False
        lo, hi = limits[i]
        return value - lo < margin or hi - value < margin


def _number(edit: QLineEdit) -> float:
    try:
        return float(edit.text())
    except ValueError:
        raise ValueError(f"not a number: {edit.text()!r}") from None
