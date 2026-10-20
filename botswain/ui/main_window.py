"""Main window: menu, toolbar, rail, pages, docks, status bar. Talks to the robot only through the worker."""

from __future__ import annotations

import logging
import time

from PySide6.QtCore import QEvent, QObject, Qt
from PySide6.QtGui import QAction, QKeyEvent
from PySide6.QtWidgets import QApplication, QFrame, QLabel, QMainWindow, QScrollArea, QStackedWidget, QWidget

from botswain.control.points import PointStore
from botswain.control.program import ProgramRunner
from botswain.control.worker import RobotWorker
from botswain.core.config import RobotConfig
from botswain.core.log import LEVELS
from botswain.core.types import (
    ClearError,
    Connect,
    Disable,
    Disconnect,
    Drag,
    Enable,
    Joints,
    Mode,
    MoveTo,
    RequestControl,
    RobotState,
)
from botswain.record.session import RecordingSession, lerobot_available, lerobot_writer_factory
from botswain.ui.docks import LogDock, camera_dock
from botswain.ui.doctor_dialog import DoctorDialog
from botswain.ui.pages.blocks_page import BlocksPage
from botswain.ui.pages.cameras_page import CamerasPage
from botswain.ui.pages.jog_page import JogPage
from botswain.ui.pages.placeholder_page import PlaceholderPage
from botswain.ui.pages.points_page import PointsView
from botswain.ui.pages.record_bar import EpisodeBar
from botswain.ui.pages.teleop_page import TeleopPage
from botswain.ui.pages.vr_page import VrPage
from botswain.ui.rail import PAGES, Rail
from botswain.ui.status_bar import StatusBar
from botswain.ui.toolbar import TopToolbar
from botswain.ui.widgets.camera_tile import CameraGrid

log = logging.getLogger(__name__)
STOP_KEYS = (Qt.Key.Key_Space, Qt.Key.Key_Escape)


class StopKeyFilter(QObject):
    """App-wide STOP on Space and Esc, whatever widget has focus.

    A QShortcut is not enough: a focused QLineEdit takes Space for itself and a focused button "clicks" on
    Space. This filter sees every key press before any widget does, stops, and eats the key.
    """

    def __init__(self, stop) -> None:
        super().__init__()
        self._stop = stop

    def eventFilter(self, obj: QObject, event: QEvent) -> bool:
        if event.type() == QEvent.Type.KeyPress and isinstance(event, QKeyEvent) and event.key() in STOP_KEYS:
            # The widget receiving the key decides (the app's focus widget can be empty when the window
            # is not the active OS window).
            if event.key() == Qt.Key.Key_Space and isinstance(obj, QWidget) and obj.property("free_text"):
                return False  # typing a task or dataset name: Space is a space here, Esc still stops
            if not event.isAutoRepeat():
                self._stop()
            return True
        return False


class MainWindow(QMainWindow):
    def __init__(self, cfg: RobotConfig, worker: RobotWorker, bridge: QObject, view_3d: bool = True,
                 parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.cfg = cfg
        self.worker = worker
        self.bridge = bridge  # keep the signal bridge alive as long as the window
        caps = worker.driver.capabilities
        self.setWindowTitle(f"Botswain · {cfg.name}")
        self.resize(1440, 900)

        self.toolbar = TopToolbar(cfg, caps)
        self.addToolBar(Qt.ToolBarArea.TopToolBarArea, self.toolbar)
        self.rail = Rail()
        self.addToolBar(Qt.ToolBarArea.LeftToolBarArea, self.rail)

        self.stack = QStackedWidget()
        self.stack.setObjectName("workspace")
        self.jog_page = JogPage(cfg, caps, self._submit, self._log, view_3d=view_3d)
        self.points = self._load_points(cfg)
        self.points_page = PointsView(self.points, self._submit, self._log, save=self._save_point)
        self.teleop_page = TeleopPage(worker, self._log)
        self.vr_page = VrPage(worker, self._log)
        self.cameras_page = CamerasPage(self._log)
        home = cfg.home_joints_deg if cfg.home_confirmed else None
        self.runner = ProgramRunner(worker, self.points, home, log_cb=self._log)
        self.blocks_page = BlocksPage(self.runner, lambda: [p.name for p in self.points.all()], self._log)
        notes = {"Script": "coming later",
                 "Teleop": "phone teleop comes in T5", "Cameras": "cameras come in T5",
                 "Settings": f"robot settings live in {cfg.path.name}"}
        self.pages: dict[str, QWidget] = {}
        self._stack_items: dict[str, QWidget] = {}
        self.jog_page.setMinimumSize(1000, 520)  # below this the page scrolls instead of squashing
        for name in PAGES:
            built = {"Jog": self.jog_page, "Points": self.points_page, "Teleop": self.teleop_page,
                     "VR": self.vr_page, "Cameras": self.cameras_page, "Blocks": self.blocks_page}
            page = built.get(name) or PlaceholderPage(name, notes[name])
            self.pages[name] = page
            self._stack_items[name] = self._scrollable(page)
            self.stack.addWidget(self._stack_items[name])
        self.setCentralWidget(self.stack)
        self.rail.page_selected.connect(lambda n: self.stack.setCurrentWidget(self._stack_items[n]))
        self.rail.select("Jog")

        self.log_dock = LogDock()
        self.points_dock_view = PointsView(self.points, self._submit, self._log, save=self._save_point, dark=True)
        self.log_dock.tabs.removeTab(1)
        self.log_dock.tabs.insertTab(1, self.points_dock_view, "Points")
        session = (RecordingSession(worker, lerobot_writer_factory(cfg.name.replace(" ", "_").lower()))
                   if lerobot_available() else None)
        self.episode_bar = EpisodeBar(session, lambda: self.cameras_page.cameras, self._log)
        self.log_dock.tabs.removeTab(2)
        self.log_dock.tabs.insertTab(2, self.episode_bar, "Episodes")
        for view in (self.points_page, self.points_dock_view):
            view.changed.connect(self._refresh_points)
        self.camera_grid = CameraGrid()
        self.camera_dock = camera_dock(self.camera_grid)
        self.cameras_page.cameras_changed.connect(self.camera_grid.set_cameras)
        self.addDockWidget(Qt.DockWidgetArea.BottomDockWidgetArea, self.log_dock)
        self.splitDockWidget(self.log_dock, self.camera_dock, Qt.Orientation.Horizontal)
        self.resizeDocks([self.log_dock, self.camera_dock], [8, 5], Qt.Orientation.Horizontal)
        self.resizeDocks([self.log_dock], [150], Qt.Orientation.Vertical)

        for dock in (self.log_dock, self.camera_dock):
            dock.setMinimumHeight(90)
        self.jog_page.robot_view.box_toggled.connect(self._box_toggled)
        self.status = StatusBar(cfg)
        self.setStatusBar(self.status)
        self._build_menu()

        self._connected = False
        self._state = RobotState.disconnected(caps.dof)
        bridge.state.connect(self.on_state)
        bridge.log.connect(self.log_dock.append)
        self._wire_toolbar()
        if getattr(self, "log_dock_pending", None):
            self._log("error", self.log_dock_pending)
        self.stop_filter = StopKeyFilter(self.stop_all)
        self.teleop_page.gamepad.action.connect(self._gamepad_action)
        QApplication.instance().installEventFilter(self.stop_filter)

    def _build_menu(self) -> None:
        bar = self.menuBar()
        file_menu = bar.addMenu("File")
        file_menu.addAction("Quit", self.close)
        robot = bar.addMenu("Robot")
        robot.addAction("Connect / Disconnect", self._toggle_connect)
        robot.addAction("Clear error", lambda: self.worker.submit(ClearError()))
        robot.addAction("Take control (RequestControl, Dobot firmware 4.6+)",
                        lambda: self.worker.submit(RequestControl()))
        stop = QAction("STOP", self)
        stop.setShortcutVisibleInContextMenu(True)
        stop.triggered.connect(self.stop_all)
        robot.addAction(stop)
        view = bar.addMenu("View")
        view.addAction(self.log_dock.toggleViewAction())
        view.addAction(self.camera_dock.toggleViewAction())
        tools = bar.addMenu("Tools")
        tools.addAction("Doctor", self.toolbar.doctor_clicked.emit)
        tools.addSeparator()
        tools.addAction("Simulate GUI hang (2 s)", self._simulate_hang)
        help_menu = bar.addMenu("Help")
        help_menu.addAction("About Botswain", lambda: self.log_dock.append(
            "info", "Botswain: connect, check, jog, teleop and record datasets on cobots. Apache-2.0."))
        path = QLabel(_short_path(self.cfg))
        path.setObjectName("robotPath")
        bar.setCornerWidget(path, Qt.Corner.TopRightCorner)

    def _wire_toolbar(self) -> None:
        tb = self.toolbar
        tb.connect_clicked.connect(self._toggle_connect)
        tb.doctor_clicked.connect(self.open_doctor)
        tb.enable_toggled.connect(lambda on: self.worker.submit(Enable() if on else Disable()))
        tb.drag_clicked.connect(
            lambda: self.worker.submit(Drag(self.worker.latest_state().mode is not Mode.DRAG)))
        tb.clear_clicked.connect(lambda: self.worker.submit(ClearError()))
        tb.stop_button.pressed.connect(self.stop_all)  # on press, not release
        tb.home_clicked.connect(self._home)
        tb.save_point_clicked.connect(self._save_point)
        tb.record_clicked.connect(self._record_clicked)

    @staticmethod
    def _scrollable(page: QWidget) -> QScrollArea:
        """Pages scroll instead of squashing, so the bottom docks can always be dragged taller."""
        area = QScrollArea()
        area.setWidgetResizable(True)
        area.setFrameShape(QFrame.Shape.NoFrame)
        area.setWidget(page)
        return area

    def _box_toggled(self, on: bool) -> None:
        self.worker.gate.box_enabled = on
        self._log("info" if on else "warn", "safety box ON: steps clamp at the walls, moves outside are refused"
                  if on else "safety box OFF: only joint limits and the speed cap protect the arm")

    def _submit(self, cmd) -> None:
        self.worker.submit(cmd)

    def _log(self, level: str, text: str) -> None:
        """GUI-side messages (pages, cameras, programs) go to the log file too, like the worker's do."""
        log.log(LEVELS.get(level, logging.INFO), text)
        self.log_dock.append(level, text)

    def _load_points(self, cfg: RobotConfig) -> PointStore:
        path = cfg.path.with_suffix(".points.json")
        try:
            return PointStore(path)
        except ValueError as e:
            self.log_dock_pending = f"{e}; points are kept in memory only until the file is fixed"
            return PointStore(None)

    def _refresh_points(self) -> None:
        self.points_page.refresh()
        self.blocks_page.refresh_points()
        self.points_dock_view.refresh()

    def _save_point(self) -> None:
        state = self.worker.latest_state()
        if not state.link_ok:
            self._log("warn", "Save point: no live robot state")
            return
        point = self.points.add(self.points.next_name(), state.tcp, state.joints)
        self._refresh_points()
        self._log("info", f"saved point {point.name}")

    def _home(self) -> None:
        if not self.cfg.home_confirmed or self.cfg.home_joints_deg is None:
            self._log("warn", "Home is disabled until home_confirmed: true in the robot YAML")
            return
        self._submit(MoveTo(Joints(self.cfg.home_joints_deg), "joint"))

    def stop_all(self) -> None:
        """STOP: abort a running Blocks program first (so it sends nothing more), then stop the robot."""
        self.runner.abort()
        self.worker.request_stop()

    def _gamepad_action(self, name: str) -> None:
        if name == "start":
            self._record_clicked()
        elif name == "back":
            self.episode_bar.discard()
        elif name == "x":
            self._save_point()

    def _record_clicked(self) -> None:
        """Toolbar Record: start an episode, or save the running one. Details live in the Episodes tab."""
        self.log_dock.tabs.setCurrentWidget(self.episode_bar)
        if self.episode_bar.recording:
            self.episode_bar.save()
        else:
            self.episode_bar.start()

    def _simulate_hang(self) -> None:
        """Lab test for the hold-jog watchdog: the GUI stops sending ticks, the worker must release the jog."""
        self._log("warn", "simulating a 2 s GUI hang")
        time.sleep(2.0)

    def _toggle_connect(self) -> None:
        self.worker.submit(Disconnect() if self._connected else Connect())

    def on_state(self, state: RobotState) -> None:
        self._state = state
        self._connected = state.mode is not Mode.DISCONNECTED
        self.toolbar.set_state(state, self._connected)
        self.jog_page.set_state(state)
        latency = getattr(self.worker.driver, "latency_ms", None) if self._connected else None
        self.status.set_state(state, self.worker.gate.counts, self.jog_page.frame, latency)
        bar = self.episode_bar
        can_record = bar.session is not None and self._connected and state.link_ok
        self.toolbar.record_btn.setEnabled(can_record or bar.recording)
        self.toolbar.record_btn.setText("Save ep." if bar.recording else "Record")
        self.toolbar.record_btn.setToolTip("" if bar.session else "Recording needs LeRobot: pip install -e .[record]")
        self.status.set_dataset(str(bar.dataset_root) if bar.session and bar.session.recorder else None)

    def open_doctor(self) -> DoctorDialog:
        dialog = DoctorDialog(self.cfg, self.worker, self.jog_page.robot_view.model, self)
        dialog.show()
        return dialog

    def closeEvent(self, event) -> None:
        QApplication.instance().removeEventFilter(self.stop_filter)
        self.runner.abort()
        self.teleop_page.shutdown()
        self.vr_page.shutdown()
        self.worker.shutdown()  # sends stop, then disconnects
        if self.episode_bar.session is not None and self.episode_bar.session.recorder is not None:
            self.episode_bar.session.close()  # finalize, or LeRobot leaves an unreadable dataset
        self.cameras_page.stop_all()
        self.jog_page.robot_view.close_view()
        super().closeEvent(event)


def _short_path(cfg: RobotConfig) -> str:
    parts = cfg.path.parts
    return "/".join(parts[-2:]) if len(parts) >= 2 else str(cfg.path)
