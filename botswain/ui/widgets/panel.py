"""Blender-style panel: a header with a collapse caret, title and meta text, and a body below."""

from __future__ import annotations

from PySide6.QtCore import Qt
from PySide6.QtWidgets import QFrame, QHBoxLayout, QLabel, QToolButton, QVBoxLayout, QWidget

from botswain.ui.theme import set_role


class Panel(QFrame):
    def __init__(self, title: str, meta: str = "", parent: QWidget | None = None) -> None:
        super().__init__(parent)
        set_role(self, "panel")
        self.title = title
        outer = QVBoxLayout(self)
        outer.setContentsMargins(1, 1, 1, 1)
        outer.setSpacing(0)

        head = set_role(QWidget(), "panelHead")
        head.setAttribute(Qt.WidgetAttribute.WA_StyledBackground)
        head.setFixedHeight(30)
        self.head_layout = QHBoxLayout(head)
        self.head_layout.setContentsMargins(6, 0, 10, 0)
        self.head_layout.setSpacing(6)
        self._title_btn = set_role(QToolButton(), "panelTitle")
        self._title_btn.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self._title_btn.clicked.connect(lambda: self.set_collapsed(not self.collapsed))
        self.meta_label = set_role(QLabel(meta), "panelMeta")
        self.head_layout.addWidget(self._title_btn)
        self.head_layout.addStretch(1)
        self.head_layout.addWidget(self.meta_label)
        outer.addWidget(head)

        self.body = QWidget()
        self.body_layout = QVBoxLayout(self.body)
        self.body_layout.setContentsMargins(12, 12, 12, 12)
        self.body_layout.setSpacing(12)
        outer.addWidget(self.body, 1)
        self.set_collapsed(False)

    @property
    def collapsed(self) -> bool:
        return not self.body.isVisibleTo(self)

    def set_collapsed(self, collapsed: bool) -> None:
        self.body.setVisible(not collapsed)
        policy = self.sizePolicy()
        policy.setVerticalPolicy(policy.Policy.Maximum if collapsed else policy.Policy.Preferred)
        self.setSizePolicy(policy)
        self._title_btn.setText(("▶  " if collapsed else "▼  ") + self.title)

    def set_meta(self, text: str) -> None:
        self.meta_label.setText(text)

    def add_head_widget(self, w: QWidget) -> None:
        self.head_layout.addWidget(w)
