import numpy as np
import pytest
from helpers import ROOT

from botswain.ui.widgets.robot_view import RobotModel

URDF = ROOT / "assets" / "dobot_cr5" / "cr5_robot.urdf"


@pytest.fixture(scope="module")
def model():
    return RobotModel(URDF)


def test_robot_view_loads_cr5_and_moves(model):
    assert len(model.meshes()) == 7
    assert model.joint_names == [f"joint{i}" for i in range(1, 7)]
    zero = model.link_transform("Link6", [0, 0, 0, 0, 0, 0])
    bent = model.link_transform("Link6", [0, 0, 90, 0, -90, 0])
    assert not np.allclose(zero, bent)


def test_joint1_rotates_about_vertical(model):
    a = model.link_transform("Link6", [0, 0, 90, 0, -90, 0])[:3, 3]
    b = model.link_transform("Link6", [90, 0, 90, 0, -90, 0])[:3, 3]
    assert a[2] == pytest.approx(b[2], abs=1e-9)  # height unchanged
    assert np.hypot(a[0], a[1]) == pytest.approx(np.hypot(b[0], b[1]), abs=1e-9)  # same radius


def test_transforms_cover_every_mesh(model):
    t = model.transforms([0, 0, 90, 0, -90, 0])
    assert set(t) == set(model.meshes())
    assert all(m.shape == (4, 4) for m in t.values())
