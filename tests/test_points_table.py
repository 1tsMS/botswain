"""Points table: limited edits with pose/joints kept in step by the model, rename, drag to reorder, Run all."""

import numpy as np
import pytest
from helpers import ROBOTS, wait_for

from botswain.app import build
from botswain.control.points import EDIT_LIMIT_DEG, EDIT_LIMIT_MM
from botswain.core.config import load_robot_config
from botswain.core.kinematics import Kinematics, matrix_to_pose
from botswain.core.types import Connect, Enable, Joints, Mode, MoveTo
from botswain.ui.pages.points_table import NAME, POSE0


@pytest.fixture
def rig(qtbot, tmp_path):
    worker, w = build(load_robot_config(ROBOTS / "fake.yaml"), view_3d=False)
    qtbot.addWidget(w)
    w.points.path = tmp_path / "points.json"
    w.points._points.clear()
    worker.submit(Connect())
    worker.submit(Enable())
    assert wait_for(lambda: worker.latest_state().mode is Mode.IDLE, timeout=2)
    sent = []
    real_submit = worker.submit
    worker.submit = lambda c: (sent.append(c), real_submit(c))
    page = w.points_page
    kin = Kinematics(load_robot_config(ROBOTS / "fake.yaml").model_urdf)
    for name, q in (("A", (0, 0, 90, 0, -90, 0)), ("B", (10, 5, 85, 0, -90, 0)), ("C", (-10, 0, 95, 0, -90, 0))):

        w.points.add(name, matrix_to_pose(kin.fk(q)), Joints(q))
    page.refresh()
    yield worker, w, page, kin, sent
    w.close()

def names(page):
    return [page.table.item(r, NAME).text() for r in range(page.table.rowCount())]

def test_table_has_named_columns_and_rows(rig):
    _, _, page, _, _ = rig
    headers = [page.table.horizontalHeaderItem(c).text() for c in range(page.table.columnCount())]
    assert headers == ["Name", "Move", "Wait s", "X", "Y", "Z", "Rx", "Ry", "Rz", "J1", "J2", "J3", "J4", "J5", "J6"]
    assert names(page) == ["A", "B", "C"] and page.table.verticalHeaderItem(2).text() == "3"

def test_joint_edit_is_limited_and_recomputes_the_pose(rig):
    _, w, page, kin, _ = rig
    col = POSE0 + 6  # J1
    lo, hi, _ = page.edit_range(0, col)
    assert (lo, hi) == (-EDIT_LIMIT_DEG, EDIT_LIMIT_DEG)  # A was taught at J1 = 0
    page.edit(0, col, 5.0)
    a = w.points.get("A")
    assert a.joints.values[0] == 5.0
    np.testing.assert_allclose(a.pose.as_list()[:3], kin.fk(a.joints.values)[:3, 3], atol=0.01)
    page.edit(0, col, 25.0)  # past the limit: refused, nothing changes
    assert w.points.get("A").joints.values[0] == 5.0
    assert page.edit_range(0, col)[:2] == (-EDIT_LIMIT_DEG, EDIT_LIMIT_DEG)  # still measured from what was taught

def test_pose_edit_recomputes_the_joints(rig):
    _, w, page, kin, _ = rig
    b = w.points.get("B")
    page.edit(1, POSE0 + 2, b.pose.z + 6.0)  # Z up 6 mm
    b2 = w.points.get("B")
    assert b2.pose.z == pytest.approx(b.pose.z + 6.0)
    np.testing.assert_allclose(kin.fk(b2.joints.values)[:3, 3], b2.pose.xyz, atol=0.05)
    assert page.edit_range(1, POSE0)[1] == pytest.approx(b.origin_pose.x + EDIT_LIMIT_MM)

def test_rename_move_kind_and_wait(rig):
    _, w, page, _, _ = rig
    page.edit(0, NAME, "pick")
    page.edit(0, 1, "linear")
    page.edit(0, 2, 1.5)
    p = w.points.get("pick")
    assert (p.move, p.wait_s) == ("linear", 1.5)
    page.edit(0, NAME, "has space")  # Space is STOP: refused
    assert names(page)[0] == "pick"

def test_drag_a_row_to_reorder(rig):
    _, w, page, _, _ = rig
    page.table.verticalHeader().moveSection(0, 2)  # drag row 1 (A) to the end
    assert [p.name for p in w.points.all()] == ["B", "C", "A"] == names(page)

def test_run_all_visits_points_in_order_and_stops_on_stop(qtbot, rig):
    _, w, page, _, sent = rig
    page.edit(1, 1, "linear")  # B by a straight line
    page.run_btn.click()
    qtbot.waitUntil(lambda: not page.runner.busy, timeout=15000)
    moves = [c for c in sent if isinstance(c, MoveTo)]
    a, b, c = (w.points.get(n) for n in ("A", "B", "C"))
    assert moves == [MoveTo(a.joints, "joint"), MoveTo(b.pose, "linear"), MoveTo(c.joints, "joint")]
    assert "all points done" in page.status.text()
    sent.clear()
    page.edit(0, 2, 5.0)  # a long wait on A, then STOP during it
    page.run_btn.click()
    # STOP during A's wait: A's move was sent and finished, B and C must never go out.
    at_a = lambda: np.allclose(w.worker.latest_state().joints.values, a.joints.values, atol=0.01)
    qtbot.waitUntil(lambda: page._running_at == "A" and any(isinstance(c, MoveTo) for c in sent) and at_a()
                    and w.worker.latest_state().mode is Mode.IDLE, timeout=8000)
    w.stop_all()
    qtbot.waitUntil(lambda: not page.runner.busy, timeout=5000)
    assert [c for c in sent if isinstance(c, MoveTo)] == [MoveTo(a.joints, "joint")]  # B and C never sent
    assert "stopped" in page.status.text()
