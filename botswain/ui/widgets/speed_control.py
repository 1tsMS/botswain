"""Speed control, compact: -25 -5 -1 [slider] +1 +5 +25 and the value. Range 1..speed cap (YAML)."""

from __future__ import annotations

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import QHBoxLayout, QLabel, QPushButton, QSlider, QVBoxLayout, QWidget

from botswain.ui.theme import set_role

DELTAS = (25, 5, 1)


class SpeedControl(QWidget):
    changed = Signal(int)

    def __init__(self, cap_pct: int, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.cap_pct = cap_pct
        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(8)

        top = QHBoxLayout()
        cap = "" if cap_pct >= 100 else f" (cap {cap_pct}%)"
        top.addWidget(set_role(QLabel(f"of max speed{cap}"), "lab"))
        top.addStretch(1)
        self.value_label = QLabel("–")
        self.value_label.setObjectName("speedValue")
        self.value_label.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        top.addWidget(self.value_label)
        lay.addLayout(top)

        row = QHBoxLayout()
        row.setSpacing(4)
        self.slider = QSlider(Qt.Orientation.Horizontal)
        self.slider.setRange(1, cap_pct)
        self.slider.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.slider.sliderReleased.connect(lambda: self._emit(self.slider.value()))
        self.slider.valueChanged.connect(lambda v: self.value_label.setText(f"{v}%"))
        for d in DELTAS:
            row.addWidget(self._button(-d))
        row.addSpacing(6)
        row.addWidget(self.slider, 1)
        row.addSpacing(6)
        for d in reversed(DELTAS):
            row.addWidget(self._button(d))
        lay.addLayout(row)

    def _button(self, delta: int) -> QPushButton:
        b = set_role(QPushButton(f"{'−' if delta < 0 else '+'}{abs(delta)}"), "key")
        b.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        b.setFixedWidth(44)
        b.clicked.connect(lambda _=False: self._emit(self.slider.value() + delta))
        return b

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
