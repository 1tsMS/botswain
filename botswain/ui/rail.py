"""Left icon rail with labels: the pages in the order they are used. Settings sits at the bottom."""

from __future__ import annotations

from PySide6.QtCore import QSize, Qt, Signal
from PySide6.QtGui import QActionGroup
from PySide6.QtWidgets import QSizePolicy, QToolBar, QToolButton, QWidget

from botswain.ui.icons import icon
from botswain.ui.theme import C, set_role

PAGES = ("Jog", "Teleop", "Cameras", "Points", "Blocks", "Settings")


class Rail(QToolBar):
    page_selected = Signal(str)

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__("Pages", parent)
        self.setObjectName("rail")
        self.setOrientation(Qt.Orientation.Vertical)
        self.setMovable(False)
        self.setFloatable(False)
        self._group = QActionGroup(self)
        self._group.setExclusive(True)
        self.buttons: dict[str, QToolButton] = {}
        for name in PAGES:
            if name == "Settings":
                spacer = QWidget()
                spacer.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Expanding)
                self.addWidget(spacer)
            ic = icon(name.lower(), C["chrome_dim"])
            ic.addPixmap(icon(name.lower(), C["chrome_text"]).pixmap(40, 40), ic.Mode.Normal, ic.State.On)
            action = self.addAction(ic, name)
            action.setCheckable(True)
            action.triggered.connect(lambda _=False, n=name: self.page_selected.emit(n))
            self._group.addAction(action)
            b = self.widgetForAction(action)
            set_role(b, "rail")
            b.setToolButtonStyle(Qt.ToolButtonStyle.ToolButtonTextUnderIcon)
            b.setIconSize(QSize(20, 20))
            b.setFocusPolicy(Qt.FocusPolicy.NoFocus)
            self.buttons[name] = b

    def select(self, name: str) -> None:
        self.buttons[name].defaultAction().setChecked(True)
        self.page_selected.emit(name)

    def current(self) -> str | None:
        checked = self._group.checkedAction()
        return checked.text() if checked else None
