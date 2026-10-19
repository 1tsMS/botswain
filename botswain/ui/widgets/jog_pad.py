"""Jog keys (X..Rz left, J1..J6 right) and the step selector.

Every key is Qt.NoFocus: in Qt, Space "clicks" the focused button, so a focused jog key could jog when the
operator presses Space to STOP. With no focus, Space always reaches the STOP filter.
"""

from __future__ import annotations

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import QButtonGroup, QGridLayout, QHBoxLayout, QPushButton, QWidget

from botswain.core.types import AXES_CART
from botswain.ui.theme import set_role

STEPS: tuple[float | None, ...] = (0.5, 1.0, 5.0, 10.0, 50.0, None)  # None = Hold (continuous jog)


class StepSelector(QWidget):
    changed = Signal(object)  # float step, or None for Hold

    def __init__(self, hold_allowed: bool = True, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        lay = QHBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(0)
        self._group = QButtonGroup(self)
        self.buttons: dict[float | None, QPushButton] = {}
        steps = [s for s in STEPS if s is not None or hold_allowed]
        for i, step in enumerate(steps):
            b = set_role(QPushButton("Hold" if step is None else f"{step:g}"), "seg")
            b.setProperty("pos", "first" if i == 0 else "last" if i == len(steps) - 1 else "mid")
            b.setCheckable(True)
            b.setFocusPolicy(Qt.FocusPolicy.NoFocus)
            b.clicked.connect(lambda _=False, s=step: self.changed.emit(s))
            self._group.addButton(b)
            self.buttons[step] = b
            lay.addWidget(b)
        self.buttons[1.0].setChecked(True)


class JogPad(QWidget):
    step = Signal(str, int)  # axis, sign
    hold_start = Signal(str, int)
    hold_end = Signal()

    def __init__(self, dof: int = 6, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.step_size: float | None = 1.0
        self._holding = False
        grid = QGridLayout(self)
        grid.setContentsMargins(0, 0, 0, 0)
        grid.setHorizontalSpacing(8)
        grid.setVerticalSpacing(8)
        grid.setColumnMinimumWidth(2, 16)
        self.keys: dict[tuple[str, int], QPushButton] = {}
        joints = [f"J{i + 1}" for i in range(dof)]
        for row in range(max(len(AXES_CART), dof)):
            if row < len(AXES_CART):
                self._add_pair(grid, AXES_CART[row], row, 0)
            if row < dof:
                self._add_pair(grid, joints[row], row, 3)

    def _add_pair(self, grid: QGridLayout, axis: str, row: int, col: int) -> None:
        for i, sign in enumerate((-1, 1)):
            b = set_role(QPushButton(f"{axis} {'−' if sign < 0 else '+'}"), "key")
            b.setFocusPolicy(Qt.FocusPolicy.NoFocus)
            b.setMinimumHeight(36)
            b.pressed.connect(lambda a=axis, s=sign: self._pressed(a, s))
            b.released.connect(self._released)
            grid.addWidget(b, row, col + i)
            self.keys[(axis, sign)] = b

    @property
    def holding(self) -> bool:
        return self._holding

    def set_step_size(self, step: float | None) -> None:
        if self._holding:
            self._released()
        self.step_size = step

    def set_blocked(self, reason: str | None) -> None:
        """Grey out every key with the reason as tooltip, or enable them all (reason None)."""
        if reason is not None and self._holding:
            self._released()
        for b in self.keys.values():
            b.setEnabled(reason is None)
            b.setToolTip(reason or "")

    def _pressed(self, axis: str, sign: int) -> None:
        if self.step_size is None:
            self._holding = True
            self.hold_start.emit(axis, sign)
        else:
            self.step.emit(axis, sign)

    def _released(self) -> None:
        if self._holding:
            self._holding = False
            self.hold_end.emit()
