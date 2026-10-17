import pytest

from botswain.control.points import PointStore
from botswain.core.types import Joints, Pose

P = Pose(-482, -136, 484, 180, 0, 0)
J = Joints((0, 0, 90, 0, -90, 0))


def test_points_roundtrip_json(tmp_path):
    path = tmp_path / "robot.points.json"
    store = PointStore(path)
    store.add("pick", P, J)
    store.add(store.next_name(), P.with_axis("Z", 500), J)
    again = PointStore(path)
    assert [p.name for p in again.all()] == ["pick", "P2"]
    assert again.get("P2").pose.z == 500 and again.get("pick").joints == J


def test_duplicate_name_gets_suffix(tmp_path):
    store = PointStore(tmp_path / "p.json")
    assert store.add("P1", P, J).name == "P1"
    assert store.add("P1", P, J).name == "P1 (2)"
    assert store.add("P1", P, J).name == "P1 (3)"


def test_rename_and_remove(tmp_path):
    store = PointStore(tmp_path / "p.json")
    store.add("a", P, J)
    store.add("b", P, J)
    assert store.rename("a", "b").name == "b (2)"
    store.remove("b")
    assert [p.name for p in PointStore(tmp_path / "p.json").all()] == ["b (2)"]


def test_bad_file_raises_instead_of_dropping_points(tmp_path):
    path = tmp_path / "p.json"
    path.write_text("{not json", encoding="utf-8")
    with pytest.raises(ValueError):
        PointStore(path)


def test_memory_only_store():
    store = PointStore(None)
    store.add("x", P, J)
    assert len(store.all()) == 1
