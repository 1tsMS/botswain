"""Speed control like the old app: -1/-5/-15/-25 left, slider in the middle, +1..+25 right.

The slider only goes up to the YAML speed cap, so what you see is what the robot can get.
"""

from __future__ import annotations

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import QGridLayout, QHBoxLayout, QLabel, QPushButton, QSlider, QVBoxLayout, QWidget

from botswain.ui.theme import set_role

DELTAS = (1, 5, 15, 25)


class SpeedControl(QWidget):
    changed = Signal(int)

    def __init__(self, cap_pct: int, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.cap_pct = cap_pct
        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(10)

        top = QHBoxLayout()
        top.addWidget(set_role(QLabel(f"of max speed (cap {cap_pct}%)"), "lab"))
        top.addStretch(1)
        self.value_label = QLabel("–")
        self.value_label.setObjectName("speedValue")
        self.value_label.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        top.addWidget(self.value_label)
        lay.addLayout(top)

        grid = QGridLayout()
        grid.setHorizontalSpacing(8)
        grid.setVerticalSpacing(4)
        grid.setColumnStretch(1, 1)
        self.slider = QSlider(Qt.Orientation.Horizontal)
        self.slider.setRange(1, cap_pct)
        self.slider.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.slider.sliderReleased.connect(lambda: self._emit(self.slider.value()))
        self.slider.valueChanged.connect(lambda v: self.value_label.setText(f"{v}%"))
        grid.addWidget(self.slider, 0, 1, len(DELTAS), 1)
        for row, d in enumerate(DELTAS):
            for col, sign in ((0, -1), (2, 1)):
                b = set_role(QPushButton(f"{'−' if sign < 0 else '+'}{d}"), "key")
                b.setFocusPolicy(Qt.FocusPolicy.NoFocus)
                b.setFixedWidth(52)
                b.setMinimumHeight(26)
                b.clicked.connect(lambda _=False, delta=sign * d: self._emit(self.slider.value() + delta))
                grid.addWidget(b, row, col)
        lay.addLayout(grid)

    def set_value(self, pct: int) -> None:
        """Show the robot's speed. Ignored while the operator drags the slider."""
        if self.slider.isSliderDown():
            return
        self.slider.blockSignals(True)
        self.slider.setValue(max(1, min(pct, self.cap_pct)))
        self.slider.blockSignals(False)
        self.value_label.setText(f"{pct}%")

    def _emit(self, pct: int) -> None:
        pct = max(1, min(pct, self.cap_pct))
        self.slider.setValue(pct)
        self.changed.emit(pct)
