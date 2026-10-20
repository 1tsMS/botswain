"""Small pieces shared by the Teleop inputs: a key -> action table and the mapping controls."""

from __future__ import annotations

from collections.abc import Callable

import numpy as np
from PySide6.QtCore import Qt
from PySide6.QtWidgets import QCheckBox, QComboBox, QGridLayout, QHBoxLayout, QLabel, QSlider, QWidget

from botswain.teleop.mapping import TeleopMapper, remap_about_z
from botswain.ui.theme import mono_font, set_role

FACINGS = (("+X", 0), ("+Y", 90), ("-X", 180), ("-Y", 270))


def key_table(rows: list[tuple[str, str]]) -> QWidget:
    """Two columns: the control (monospace) and what it does."""
    w = QWidget()
    grid = QGridLayout(w)
    grid.setContentsMargins(0, 0, 0, 0)
    grid.setHorizontalSpacing(16)
    grid.setVerticalSpacing(6)
    for i, (key, action) in enumerate(rows):
        k = QLabel(key)
        k.setFont(mono_font(12))
        a = set_role(QLabel(action), "lab")
        a.setWordWrap(True)
        grid.addWidget(k, i, 0, Qt.AlignmentFlag.AlignTop)
        grid.addWidget(a, i, 1)
    grid.setColumnStretch(1, 1)
    return w


class MappingControls(QWidget):
    """Motion scale, follow rotation, and which way the operator faces. With mapper=None only the facing is shown
    (the game controller has no position to scale); on_remap then receives the new facing matrix."""

    def __init__(self, mapper: TeleopMapper | None, scale_pct: int = 30, what: str = "phone",
                 on_remap: Callable[[np.ndarray], None] | None = None, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.mapper = mapper
        self._on_remap = on_remap
        grid = QGridLayout(self)
        grid.setContentsMargins(0, 0, 0, 0)
        grid.setHorizontalSpacing(8)
        grid.setVerticalSpacing(10)
        row = 0
        if mapper is not None:
            grid.addWidget(set_role(QLabel("Motion scale"), "lab"), row, 0)
            scale_row = QHBoxLayout()
            self.scale_slider = QSlider(Qt.Orientation.Horizontal)
            self.scale_slider.setRange(10, 100)
            self.scale_slider.setValue(scale_pct)
            self.scale_slider.setFocusPolicy(Qt.FocusPolicy.NoFocus)
            self.scale_value = QLabel(f"{scale_pct / 100:.2f}")
            self.scale_value.setFont(mono_font(13))
            scale_row.addWidget(self.scale_slider, 1)
            scale_row.addWidget(self.scale_value)
            grid.addLayout(scale_row, row, 1)
            row += 1
            self.rotation_box = QCheckBox(f"Follow {what} rotation (off for first runs)")
            self.rotation_box.setFocusPolicy(Qt.FocusPolicy.NoFocus)
            grid.addWidget(self.rotation_box, row, 0, 1, 2)
            row += 1
            self.scale_slider.valueChanged.connect(self._changed)
            self.rotation_box.toggled.connect(self._changed)
        forward = "Your forward" if what == "you" else f"{what.capitalize()} forward"
        grid.addWidget(set_role(QLabel(f"{forward} is robot"), "lab"), row, 0)
        self.facing = QComboBox()
        self.facing.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        for text, deg in FACINGS:
            self.facing.addItem(text, deg)
        facing_row = QHBoxLayout()
        facing_row.addWidget(self.facing)
        facing_row.addStretch(1)
        grid.addLayout(facing_row, row, 1)
        grid.setColumnStretch(1, 1)
        self.facing.currentIndexChanged.connect(self._changed)

    def sync_from_mapper(self) -> None:
        """Show a scale or rotation switch changed elsewhere (the headset's buttons) without treating it as a new
        setting."""
        if self.mapper is None:
            return
        rotating = self.mapper.scale_rot > 0.0
        if rotating != self.rotation_box.isChecked():
            self.rotation_box.blockSignals(True)
            self.rotation_box.setChecked(rotating)
            self.rotation_box.blockSignals(False)
        pct = round(self.mapper.scale_pos * 100)
        if pct != self.scale_slider.value():
            self.scale_slider.blockSignals(True)
            self.scale_slider.setValue(pct)
            self.scale_slider.blockSignals(False)
            self.scale_value.setText(f"{self.mapper.scale_pos:.2f}")

    def _changed(self) -> None:
        remap = remap_about_z(self.facing.currentData())
        if self.mapper is not None:
            self.mapper.scale_pos = self.scale_slider.value() / 100.0
            self.mapper.scale_rot = 1.0 if self.rotation_box.isChecked() else 0.0
            self.mapper.remap = remap
            self.mapper.request_reanchor()  # never let a settings change make the target jump
            self.scale_value.setText(f"{self.mapper.scale_pos:.2f}")
        if self._on_remap is not None:
            self._on_remap(remap)
