import numpy as np
from helpers import wait_for

from botswain.sensors.cameras import CameraThread, list_cameras


class FakeCapture:
    """Stands in for cv2.VideoCapture: numbered grey frames, or closed."""

    def __init__(self, opened=True):
        self.opened = opened
        self.n = 0

    def isOpened(self):
        return self.opened

    def set(self, prop, value):
        return True

    def read(self):
        self.n += 1
        return True, np.full((48, 64, 3), self.n % 255, dtype=np.uint8)

    def release(self):
        self.opened = False


def test_camera_thread_with_fake_source():
    cam = CameraThread(0, "front", opener=lambda i: FakeCapture())
    cam.start()
    try:
        assert wait_for(lambda: cam.latest() is not None)
        f1, t1 = cam.latest()
        assert f1.shape == (48, 64, 3)
        assert wait_for(lambda: cam.latest()[1] > t1)
    finally:
        cam.stop()


def test_zero_cameras_found():
    assert list_cameras(opener=lambda i: FakeCapture(opened=False)) == []
    assert list_cameras(max_index=3, opener=lambda i: FakeCapture(opened=i == 1)) == [1]


def test_camera_that_does_not_open_reports_instead_of_freezing():
    # Opening runs on the camera's thread (DirectShow can take seconds): start() returns at once.
    cam = CameraThread(5, "x", opener=lambda i: FakeCapture(opened=False))
    cam.start()
    assert wait_for(lambda: cam.error is not None) and "did not open" in cam.error
    assert cam.latest() is None
    cam.stop()


def test_show_adds_cameras_instead_of_replacing(qtbot, monkeypatch):
    from botswain.sensors.cameras import CameraSource
    from botswain.ui.pages import cameras_page

    class FakeThread:
        def __init__(self, index, name):
            self.index, self.name, self.stopped = index, name, False

        def start(self):
            pass

        def stop(self):
            self.stopped = True

        def latest(self):
            return None

    monkeypatch.setattr(cameras_page, "CameraThread", FakeThread)
    monkeypatch.setattr(cameras_page.CameraGrid, "set_cameras", lambda self, cams: None)
    page = cameras_page.CamerasPage(lambda level, text: None)
    qtbot.addWidget(page)
    page.scan([CameraSource("top", 0), CameraSource("wrist", 1), CameraSource("side", 2)])
    page.choice.setCurrentIndex(0)
    page.show_selected()
    page.choice.setCurrentIndex(2)
    page.show_selected()
    page.show_selected()  # already running: not opened twice
    assert [c.name for c in page.cameras] == ["top", "side"]
    page.choice.setCurrentIndex(page.choice.findData(cameras_page.ALL))
    page.show_selected()
    assert sorted(c.index for c in page.cameras) == [0, 1, 2]
    page.stop_all()
    assert page.cameras == []
