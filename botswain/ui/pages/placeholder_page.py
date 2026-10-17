"""Pages that come later (Script, Blocks...): a plain label so the rail is complete."""

from __future__ import annotations

from PySide6.QtCore import Qt
from PySide6.QtWidgets import QLabel, QVBoxLayout, QWidget

from botswain.ui.theme import set_role


class PlaceholderPage(QWidget):
    def __init__(self, name: str, note: str = "coming later", parent: QWidget | None = None) -> None:
        super().__init__(parent)
        lay = QVBoxLayout(self)
        label = set_role(QLabel(f"{name}: {note}"), "placeholder")
        label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        lay.addWidget(label)
