"""Saved points list with Go to / Rename / Delete. Used as the Points page and as the Points dock tab."""

from __future__ import annotations

from collections.abc import Callable

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QHBoxLayout,
    QInputDialog,
    QLabel,
    QListWidget,
    QListWidgetItem,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

from botswain.control.points import Point, PointStore
from botswain.core.types import Command, MoveTo
from botswain.ui.theme import mono_font, set_role


class PointsView(QWidget):
    changed = Signal()

    def __init__(self, store: PointStore, submit: Callable[[Command], None], log: Callable[[str, str], None],
                 save: Callable[[], None] | None = None, dark: bool = False, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.store = store
        self._submit = submit
        self._log = log
        lay = QVBoxLayout(self)
        margin = 6 if dark else 8
        lay.setContentsMargins(margin, margin, margin, margin)
        self.list = QListWidget()
        self.list.setObjectName("pointsDark" if dark else "points")
        self.list.setFont(mono_font(12))
        self.list.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.list.itemDoubleClicked.connect(lambda _item: self.go_to())
        self.list.currentRowChanged.connect(lambda _row: self._update_buttons())
        self.empty = QLabel("No saved points yet. Connect, enable, move the arm, then press Save current pose "
                            "(or Save point in the toolbar). Double-click a point to go there.")
        self.empty.setWordWrap(True)
        self.empty.setProperty("role", "dockEmpty" if dark else "lab")
        lay.addWidget(self.empty)
        lay.addWidget(self.list, 1)
        row = QHBoxLayout()
        self.save_btn = self._button("Save current pose", "btn", lambda: save() if save else None)
        self.save_btn.setVisible(save is not None)
        row.addWidget(self.save_btn)
        self.go_btn = self._button("Go to", "primary", self.go_to)
        self.rename_btn = self._button("Rename", "btn", self.rename)
        self.delete_btn = self._button("Delete", "btn", self.delete)
        for b in (self.go_btn, self.rename_btn, self.delete_btn):
            row.addWidget(b)
        row.addStretch(1)
        lay.addLayout(row)
        self.refresh()

    def _button(self, text: str, role: str, slot) -> QPushButton:
        b = set_role(QPushButton(text), role)
        b.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        b.clicked.connect(slot)
        return b

    def refresh(self) -> None:
        current = self.selected_name()
        self.list.clear()
        for p in self.store.all():
            x, y, z = p.pose.xyz
            joints = " ".join(f"{v:7.2f}" for v in p.joints.values)
            item = QListWidgetItem(f"{p.name:<10} X{x:8.1f} Y{y:8.1f} Z{z:8.1f}   J {joints}")
            item.setData(Qt.ItemDataRole.UserRole, p.name)
            self.list.addItem(item)
            if p.name == current:
                self.list.setCurrentItem(item)
        self.empty.setVisible(self.list.count() == 0)
        self.list.setVisible(self.list.count() > 0)
        self._update_buttons()

    def _update_buttons(self) -> None:
        has = self.list.currentItem() is not None
        for b in (self.go_btn, self.rename_btn, self.delete_btn):
            b.setEnabled(has)

    def selected_name(self) -> str | None:
        item = self.list.currentItem()
        return item.data(Qt.ItemDataRole.UserRole) if item else None

    def selected(self) -> Point | None:
        name = self.selected_name()
        return self.store.get(name) if name else None

    def go_to(self) -> None:
        """Joint move to the saved joint angles: same arm configuration as when it was saved. Goes through the gate."""
        point = self.selected()
        if point is None:
            return
        self._submit(MoveTo(point.joints, "joint"))
        self._log("info", f"go to point {point.name}")

    def rename(self) -> None:
        point = self.selected()
        if point is None:
            return
        new, ok = QInputDialog.getText(self, "Rename point", "Name (no spaces: Space is STOP):", text=point.name)
        if ok and new.strip():
            renamed = self.store.rename(point.name, new)
            self._log("info", f"renamed point {point.name} -> {renamed.name}")
            self.changed.emit()

    def delete(self) -> None:
        name = self.selected_name()
        if name:
            self.store.remove(name)
            self._log("info", f"deleted point {name}")
            self.changed.emit()
