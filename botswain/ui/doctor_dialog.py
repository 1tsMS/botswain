"""Doctor dialog: runs the no-motion checks and shows check / status / value / fix in a table."""

from __future__ import annotations

import threading

from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QColor, QGuiApplication
from PySide6.QtWidgets import (
    QDialog,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QPushButton,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

from botswain.control.doctor import model_check, network_checks, offline_checks
from botswain.control.worker import RobotWorker
from botswain.core.config import RobotConfig
from botswain.core.types import Diagnose, Mode
from botswain.drivers.base import Check
from botswain.ui.theme import C, set_role

STATUS_COLOR = {"ok": C["ok"], "warn": C["warn"], "fail": C["red"]}


class DoctorDialog(QDialog):
    results = Signal(list)

    def __init__(self, cfg: RobotConfig, worker: RobotWorker, model=None, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.cfg = cfg
        self.worker = worker
        self.model = model
        self.setWindowTitle("Doctor")
        self.resize(820, 420)
        lay = QVBoxLayout(self)
        self.summary = set_role(QLabel(), "lab")
        lay.addWidget(self.summary)
        self.table = QTableWidget(0, 4)
        self.table.setHorizontalHeaderLabels(["Check", "Status", "Value", "Fix"])
        self.table.verticalHeader().setVisible(False)
        self.table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        header = self.table.horizontalHeader()
        for col, mode in enumerate((QHeaderView.ResizeMode.ResizeToContents,) * 3 + (QHeaderView.ResizeMode.Stretch,)):
            header.setSectionResizeMode(col, mode)
        lay.addWidget(self.table, 1)
        row = QHBoxLayout()
        row.addStretch(1)
        for text, slot in (("Run again", self.run), ("Copy", self.copy), ("Close", self.close)):
            b = set_role(QPushButton(text), "btn")
            b.setFocusPolicy(Qt.FocusPolicy.NoFocus)
            b.clicked.connect(slot)
            row.addWidget(b)
        lay.addLayout(row)
        self.results.connect(self._add_rows)
        self.run()

    def run(self) -> None:
        self.table.setRowCount(0)
        self.summary.setText("Running checks (nothing moves)...")
        state = self.worker.latest_state()
        connected = state.mode is not Mode.DISCONNECTED
        threading.Thread(target=self._network, args=(connected,), daemon=True).start()
        if connected:
            self.worker.submit(Diagnose(reply=self._emit))
            if self.model is not None and state.link_ok and self.cfg.connection is not None:
                self._add_rows([model_check(self.model, state)])

    def _network(self, connected: bool) -> None:
        checks = network_checks(self.cfg, connected)
        if not connected:
            checks += offline_checks(self.cfg)
        self._emit(checks)

    def _emit(self, checks: list[Check]) -> None:
        try:
            self.results.emit(checks)
        except RuntimeError:
            pass  # dialog closed before the checks finished

    def _add_rows(self, checks: list[Check]) -> None:
        for c in checks:
            r = self.table.rowCount()
            self.table.insertRow(r)
            status = QTableWidgetItem(c.status.upper())
            status.setForeground(QColor(STATUS_COLOR[c.status]))
            for col, item in enumerate((QTableWidgetItem(c.name), status, QTableWidgetItem(c.value),
                                        QTableWidgetItem(c.hint))):
                self.table.setItem(r, col, item)
        fails = sum(self.table.item(r, 1).text() == "FAIL" for r in range(self.table.rowCount()))
        warns = sum(self.table.item(r, 1).text() == "WARN" for r in range(self.table.rowCount()))
        self.summary.setText(f"{self.table.rowCount()} checks: {fails} failed, {warns} warnings")

    def rows(self) -> list[tuple[str, str, str, str]]:
        return [tuple(self.table.item(r, c).text() for c in range(4)) for r in range(self.table.rowCount())]

    def copy(self) -> None:
        lines = [f"{name}\t{status}\t{value}\t{fix}" for name, status, value, fix in self.rows()]
        QGuiApplication.clipboard().setText("\n".join(lines))
