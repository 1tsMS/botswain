"""OBS-style bottom docks: Log / Points / Episodes tabs, and Cameras. The headers are the tabs themselves."""

from __future__ import annotations

import html
import time

from PySide6.QtCore import Qt
from PySide6.QtWidgets import QDockWidget, QLabel, QPlainTextEdit, QTabWidget, QVBoxLayout, QWidget

from botswain.ui.theme import C, set_role

LEVEL_COLOR = {"warn": C["log_warn"], "error": C["log_error"]}


class LogView(QPlainTextEdit):
    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setObjectName("log")
        self.setReadOnly(True)
        self.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.setMaximumBlockCount(2000)
        self.entries = 0

    def append_line(self, level: str, text: str) -> None:
        stamp = time.strftime("%H:%M:%S")
        color = LEVEL_COLOR.get(level, C["log_text"])
        self.appendHtml(f'<span style="color:{C["log_time"]}">{stamp}</span>&nbsp;&nbsp;'
                        f'<span style="color:{color}">{html.escape(text)}</span>')
        self.entries += 1


def _bare_dock(title: str, content: QWidget) -> QDockWidget:
    dock = QDockWidget(title)
    dock.setObjectName(title.lower())
    dock.setTitleBarWidget(QWidget())  # hide the title bar: the tabs are the header, like the mockup
    dock.setFeatures(QDockWidget.DockWidgetFeature.DockWidgetClosable)
    dock.setWidget(content)
    return dock


def _empty(text: str) -> QLabel:
    label = set_role(QLabel(text), "dockEmpty")
    label.setAlignment(Qt.AlignmentFlag.AlignCenter)
    return label


class LogDock(QDockWidget):
    """Log, Points and Episodes tabs. Points and Episodes get filled in later tasks."""

    def __init__(self) -> None:
        super().__init__("Log")
        self.setObjectName("log_dock")
        self.setTitleBarWidget(QWidget())
        self.setFeatures(QDockWidget.DockWidgetFeature.DockWidgetClosable)
        self.tabs = QTabWidget()
        self.tabs.setObjectName("dockTabs")
        self.tabs.setDocumentMode(True)
        self.tabs.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.log_view = LogView()
        self.tabs.addTab(self.log_view, "Log")
        self.tabs.addTab(_empty("Saved points show here"), "Points")
        self.tabs.addTab(_empty("Recorded episodes show here"), "Episodes")
        self.meta = set_role(QLabel("0 entries"), "dockMeta")
        self.tabs.setCornerWidget(self.meta, Qt.Corner.TopRightCorner)
        self.setWidget(self.tabs)

    def append(self, level: str, text: str) -> None:
        self.log_view.append_line(level, text)
        self.meta.setText(f"{self.log_view.entries} entries")


def camera_dock(content: QWidget) -> QDockWidget:
    box = set_role(QWidget(), "dock")
    box.setAttribute(Qt.WidgetAttribute.WA_StyledBackground)
    v = QVBoxLayout(box)
    v.setContentsMargins(0, 0, 0, 0)
    v.setSpacing(0)
    v.addWidget(set_role(QLabel("Cameras"), "dockHead"))
    v.addWidget(content, 1)
    return _bare_dock("Cameras", box)
