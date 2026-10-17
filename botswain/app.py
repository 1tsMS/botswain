"""Builds config -> driver -> gate -> worker -> window and wires the worker callbacks to Qt signals."""

from __future__ import annotations

import sys
from pathlib import Path

from PySide6.QtCore import QObject, Signal
from PySide6.QtWidgets import QApplication

from botswain.control.safety import SafetyGate
from botswain.control.worker import RobotWorker
from botswain.core.config import RobotConfig
from botswain.core.log import setup_logging
from botswain.drivers.registry import make_driver
from botswain.ui import theme
from botswain.ui.main_window import MainWindow

LOG_DIR = Path(__file__).resolve().parents[1] / "logs"


class StateBridge(QObject):
    """Worker callbacks run on the worker thread; emitting a signal queues them onto the GUI thread."""

    state = Signal(object)
    log = Signal(str, str)


def build(cfg: RobotConfig, view_3d: bool = True) -> tuple[RobotWorker, MainWindow]:
    driver = make_driver(cfg)
    worker = RobotWorker(driver, SafetyGate(cfg.limits, cfg.stream.period_s))
    bridge = StateBridge()
    worker.on_state = bridge.state.emit
    worker.on_log = bridge.log.emit
    window = MainWindow(cfg, worker, bridge, view_3d=view_3d)
    worker.start()
    return worker, window


def run(cfg: RobotConfig) -> int:
    log_file = setup_logging(LOG_DIR)
    app = QApplication(sys.argv)
    app.setApplicationName("Botswain")
    theme.apply(app)
    worker, window = build(cfg)
    window.log_dock.append("info", f"loaded {cfg.path.name} ({cfg.name}, driver {cfg.driver}), log file {log_file}")
    window.show()
    code = app.exec()
    worker.shutdown()
    return code
