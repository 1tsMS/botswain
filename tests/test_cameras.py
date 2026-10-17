import numpy as np
import pytest
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


def test_camera_that_does_not_open_raises():
    with pytest.raises(RuntimeError):
        CameraThread(5, "x", opener=lambda i: FakeCapture(opened=False)).start()
