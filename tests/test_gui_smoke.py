import pytest
from helpers import DH_TOOL, ROBOTS

from botswain.app import build
from botswain.core.config import load_robot_config, load_tool_config
from botswain.core.types import Connect, Enable, JogStep, Mode
from botswain.ui.rail import PAGES


@pytest.fixture
def make_window(qtbot):
    built = []

    def make(cfg=None, view_3d=False, tool=True):
        cfg = cfg or load_robot_config(ROBOTS / "fake.yaml")
        worker, window = build(cfg, view_3d=view_3d, tool_cfg=load_tool_config(DH_TOOL) if tool else None)
        qtbot.addWidget(window)
        built.append(window)
        return worker, window

    yield make
    for w in built:
        w.close()


def ready(qtbot, worker):
    worker.submit(Connect())
    worker.submit(Enable())
    qtbot.waitUntil(lambda: worker.latest_state().mode is Mode.IDLE, timeout=2000)


def test_window_opens_with_mockup_sections(make_window):
    _, w = make_window()
    groups = [lbl.text() for lbl in w.toolbar.findChildren(type(w.toolbar.state_label))
              if lbl.property("role") == "groupName"]
    assert groups == ["CONNECTION", "ROBOT", "DATA"]
    assert list(w.rail.buttons) == list(PAGES)
    assert w.rail.current() == "Jog"
    page = w.jog_page
    assert [p.title for p in (page.orientation, page.control, page.speed, page.view)] == [
        "Orientation", "Control", "Speed control", "Robot view"]
    assert w.log_dock.tabs.tabText(0) == "Log" and w.camera_dock.windowTitle() == "Cameras"
    assert w.toolbar.stop_button is not None


def test_connect_button_connects_without_enabling(qtbot, make_window):
    worker, w = make_window()
    w.toolbar.connect_btn.click()
    qtbot.waitUntil(lambda: w.toolbar.connect_btn.text() == "Disconnect", timeout=2000)
    assert worker.latest_state().mode is Mode.DISABLED
    assert w.toolbar.enable_btn.text() == "Enable"


def test_enable_button_toggles(qtbot, make_window):
    worker, w = make_window()
    worker.submit(Connect())
    qtbot.waitUntil(lambda: w.toolbar.enable_btn.isEnabled(), timeout=2000)
    w.toolbar.enable_btn.click()
    qtbot.waitUntil(lambda: w.toolbar.enable_btn.text() == "Disable", timeout=2000)
    w.toolbar.enable_btn.click()
    qtbot.waitUntil(lambda: worker.latest_state().mode is Mode.DISABLED, timeout=2000)


def test_pose_fields_follow_fake_state(qtbot, make_window):
    worker, w = make_window()
    ready(qtbot, worker)
    field = w.jog_page.pose_fields.pose_edits["X"]
    qtbot.waitUntil(lambda: field.text() != "", timeout=1000)
    before = field.text()
    worker.submit(JogStep("X", 5.0))
    qtbot.waitUntil(lambda: field.text() != before, timeout=500)


def test_fields_read_only_until_edit_target(qtbot, make_window):
    _, w = make_window()
    page = w.jog_page
    assert page.pose_fields.pose_edits["X"].isReadOnly()
    page.edit_btn.click()
    assert not page.pose_fields.pose_edits["X"].isReadOnly()
    assert page.orientation.meta_label.text() == "editing target"


def test_hidden_when_not_capable(make_window):
    _, w = make_window(tool=False)  # no end effector selected
    page = w.jog_page
    assert page.gripper_open_btn.isHidden() and page.gripper_close_btn.isHidden() and page.force_box.isHidden()
    _, w2 = make_window()
    assert not w2.jog_page.gripper_open_btn.isHidden() and not w2.jog_page.force_box.isHidden()


def test_home_disabled_until_confirmed(qtbot, make_window):
    worker, w = make_window(load_robot_config(ROBOTS / "dobot_cr5.yaml").__class__(
        **{**load_robot_config(ROBOTS / "fake.yaml").__dict__, "home_confirmed": False}))
    ready(qtbot, worker)
    qtbot.wait(100)
    assert not w.toolbar.home_btn.isEnabled()
    assert "home_confirmed" in w.toolbar.home_btn.toolTip()


def test_status_bar_shows_hz(qtbot, make_window):
    worker, w = make_window()
    worker.submit(Connect())
    qtbot.waitUntil(lambda: "0 Hz" not in w.status.hz.text(), timeout=3000)
    assert "Link" in w.status.link.text()


def test_log_dock_shows_worker_messages(qtbot, make_window):
    worker, w = make_window()
    worker.submit(Connect())
    worker.submit(JogStep("X", 1.0))  # disabled -> rejected by the gate, logged
    qtbot.waitUntil(lambda: "not enabled" in w.log_dock.log_view.toPlainText(), timeout=2000)


def test_3d_view_builds(qtbot, make_window):
    worker, w = make_window(view_3d=True)
    view = w.jog_page.robot_view
    assert view.model is not None and view.plotter is not None
    ready(qtbot, worker)
    worker.submit(JogStep("J1", 2.0))  # fake joints move 3 deg/s at 10 % speed
    qtbot.waitUntil(lambda: view._last_joints is not None and abs(view._last_joints[0] - 2.0) < 1e-6,
                    timeout=3000)


def test_doctor_dialog_fills_table(qtbot, make_window):
    _, w = make_window()
    dialog = w.open_doctor()
    qtbot.addWidget(dialog)
    qtbot.waitUntil(lambda: dialog.table.rowCount() >= 1, timeout=3000)  # fake, offline: one check
    assert all(row[1] == "OK" for row in dialog.rows())


def test_app_drives_fake_dobot_controller(qtbot, make_window):
    from helpers import dobot_cfg

    from botswain.drivers.dobot.fake_controller import FakeDobotController

    ctrl = FakeDobotController()
    ctrl.v3 = True  # like the lab CR5 (dobot_cr5_fake.yaml speaks dobot_v3)
    ctrl.start()
    try:
        worker, w = make_window(dobot_cfg(ctrl))
        w.toolbar.connect_btn.click()
        qtbot.waitUntil(lambda: worker.latest_state().link_ok, timeout=3000)
        assert worker.latest_state().mode is Mode.DISABLED
        qtbot.waitUntil(w.toolbar.enable_btn.isEnabled, timeout=1000)  # GUI catches up at 30 Hz
        w.toolbar.enable_btn.click()
        qtbot.waitUntil(lambda: worker.latest_state().mode is Mode.IDLE, timeout=3000)
        x0 = worker.latest_state().tcp.x
        worker.submit(JogStep("X", 2.0))
        qtbot.waitUntil(lambda: abs(worker.latest_state().tcp.x - x0 - 2.0) < 1e-3, timeout=3000)
        dialog = w.open_doctor()
        qtbot.addWidget(dialog)
        qtbot.waitUntil(lambda: dialog.table.rowCount() >= 9, timeout=5000)
        assert "Disconnect" == w.toolbar.connect_btn.text()
        w.close()
        assert ctrl.commands[-2:] == ["MoveJog()", "ResetRobot()"]
    finally:
        ctrl.stop()


def test_teleop_page_starts_and_stops_server(qtbot, make_window, monkeypatch):
    import socket

    from botswain.ui.pages import teleop_page

    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    monkeypatch.setattr(teleop_page, "PORT", port)
    _, w = make_window()
    page = w.teleop_page
    page.toggle_server()
    assert page.server is not None
    qtbot.waitUntil(lambda: str(port) in page.link.url_label.text(), timeout=1000)
    assert page.server.token in page.link.key_label.text()  # typeable key next to the QR code
    assert not page.link.qr_label.isHidden() and not page.link.qr_label.pixmap().isNull()
    page.toggle_server()
    assert page.server is None and page.link.qr_label.isHidden()


def test_camera_dock_follows_cameras_page(qtbot, make_window):
    from botswain.sensors.cameras import CameraThread

    _, w = make_window()
    cam = CameraThread(0, "front")
    w.cameras_page.cameras = [cam]
    w.cameras_page._changed()
    assert len(w.camera_grid.tiles) == 1 and len(w.cameras_page.grid.tiles) == 1


def test_scan_button_lists_every_source(qtbot, make_window, monkeypatch):
    # Lab: pressing Scan listed nothing, because clicked(bool) passed False in as the source list.
    from botswain.sensors.cameras import CameraSource
    from botswain.ui.pages import cameras_page

    found = [CameraSource("USB2.0 HD UVC WebCam", 0), CameraSource("DroidCam Source 3", 1),
             CameraSource("OBS Virtual Camera", 3)]
    monkeypatch.setattr(cameras_page, "list_sources", lambda: found)
    _, w = make_window()
    page = w.cameras_page
    page.scan_btn.click()  # lists on a thread: the window stays responsive
    items = lambda: [page.choice.itemText(i) for i in range(page.choice.count())]
    qtbot.waitUntil(lambda: items() == [s.name for s in found] + ["All sources"], timeout=2000)
    assert page.show_btn.isEnabled() and page.scan_btn.isEnabled()
