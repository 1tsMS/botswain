"""Builds configs -> drivers (arm + end effector) -> gate -> worker -> window, and wires the worker to Qt."""

from __future__ import annotations

import sys
from pathlib import Path

from PySide6.QtCore import QCoreApplication, QObject, Qt, Signal
from PySide6.QtQuick import QQuickWindow, QSGRendererInterface
from PySide6.QtWidgets import QApplication

from botswain.control.safety import SafetyGate
from botswain.control.worker import RobotWorker
from botswain.core.config import (
    ConfigError,
    RobotConfig,
    ToolConfig,
    load_robot_config,
    load_tool_config,
    write_selection,
)
from botswain.core.log import setup_logging
from botswain.drivers.registry import make_driver, make_tool
from botswain.ui import theme
from botswain.ui.main_window import MainWindow

LOG_DIR = Path(__file__).resolve().parents[1] / "logs"


class StateBridge(QObject):
    """Worker callbacks run on the worker thread; emitting a signal queues them onto the GUI thread."""

    state = Signal(object)
    log = Signal(str, str)


def build(cfg: RobotConfig, view_3d: bool = True, tool_cfg: ToolConfig | None = None) -> tuple[RobotWorker, MainWindow]:
    driver = make_driver(cfg)
    tool = make_tool(tool_cfg, cfg) if tool_cfg is not None else None
    worker = RobotWorker(driver, SafetyGate(cfg.limits, cfg.stream.period_s), tool=tool)
    bridge = StateBridge()
    worker.on_state = bridge.state.emit
    worker.on_log = bridge.log.emit
    window = MainWindow(cfg, worker, bridge, view_3d=view_3d, tool_cfg=tool_cfg)
    worker.start()
    return worker, window


def run(cfg: RobotConfig, tool_cfg: ToolConfig | None = None) -> int:
    log_file = setup_logging(LOG_DIR)
    # The Blocks page (Qt web engine, drawn through Qt Quick) and the 3D robot view (VTK in a QOpenGLWidget) share
    # one window. Qt Quick defaults to Direct3D on Windows while the window composites with OpenGL because of
    # the 3D view, and the web view then showed black. Both on OpenGL, with shared contexts, set before the app.
    QQuickWindow.setGraphicsApi(QSGRendererInterface.GraphicsApi.OpenGL)
    QCoreApplication.setAttribute(Qt.ApplicationAttribute.AA_ShareOpenGLContexts)
    app = QApplication(sys.argv)
    app.setApplicationName("Botswain")
    theme.apply(app)
    workers: list[RobotWorker] = []

    def open_window(cfg: RobotConfig, tool_cfg: ToolConfig | None, geometry=None) -> None:
        worker, window = build(cfg, tool_cfg=tool_cfg)
        workers.append(worker)
        tool = f", end effector {tool_cfg.name}" if tool_cfg else ", no end effector"
        window.log_dock.append("info", f"loaded {cfg.path.name} ({cfg.name}, protocol {cfg.protocol}{tool}), "
                                       f"log file {log_file}")
        window.switch_requested.connect(lambda robot, tool_path: switch(window, robot, tool_path))
        if geometry is not None:
            window.restoreGeometry(geometry)
        window.show()

    def switch(old: MainWindow, robot: Path, tool_path: Path | None) -> None:
        """Load another robot / end effector in this same app: the new window opens, then the old one closes
        (its close stops the robot, the servers and the cameras, as on exit)."""
        try:
            new_cfg = load_robot_config(robot)
            new_tool = load_tool_config(tool_path) if tool_path else None
        except ConfigError as e:
            old.log_dock.append("error", f"cannot load: {e}")
            return
        write_selection(robot, tool_path)
        open_window(new_cfg, new_tool, old.saveGeometry())  # first: closing the last window would quit the app
        old.close()
        old.deleteLater()

    open_window(cfg, tool_cfg)
    code = app.exec()
    for worker in workers:
        worker.shutdown()  # already stopped by each window's close; harmless twice
    return code
