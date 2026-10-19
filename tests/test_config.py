import pytest
import yaml
from helpers import ROBOTS

from botswain.core.config import LIMIT_KEYS, ConfigError, load_robot_config


def write_variant(tmp_path, change):
    """Copy dobot_cr5.yaml, let `change` edit the dict, write it to tmp_path."""
    data = yaml.safe_load((ROBOTS / "dobot_cr5.yaml").read_text(encoding="utf-8"))
    change(data)
    path = tmp_path / "robot.yaml"
    path.write_text(yaml.safe_dump(data), encoding="utf-8")
    return path


def test_config_loads_cr5():
    cfg = load_robot_config(ROBOTS / "dobot_cr5.yaml")
    assert cfg.limits.box_mm[0] == (-632, -332)
    assert cfg.limits.speed_cap_pct == 100 and cfg.limits.box_enabled is False
    assert cfg.dof == 6
    assert cfg.connection.ip == "192.168.5.1"
    assert cfg.model_urdf.is_absolute() and cfg.model_urdf.name == "cr5_robot.urdf"
    assert cfg.model_urdf.parent.name == "dobot_cr5"
    assert cfg.home_confirmed is True and cfg.home_joints_deg == (0, 0, 90, 0, -90, 0)  # confirmed in the lab
    assert cfg.limits.controller_speed_max_pct == 20


def test_config_loads_fake():
    cfg = load_robot_config(ROBOTS / "fake.yaml")
    assert cfg.driver == "fake" and cfg.connection is None


@pytest.mark.parametrize("key", LIMIT_KEYS)
def test_config_missing_limit_fails(tmp_path, key):
    path = write_variant(tmp_path, lambda d: d["limits"].pop(key))
    with pytest.raises(ConfigError) as e:
        load_robot_config(path)
    assert e.value.field == f"limits.{key}"


def test_config_misspelt_limit_names_the_typo(tmp_path):
    def typo(d):
        d["limits"]["max_stp_deg"] = d["limits"].pop("max_step_deg")

    with pytest.raises(ConfigError) as e:
        load_robot_config(write_variant(tmp_path, typo))
    assert e.value.field == "limits.max_step_deg"
    assert "max_stp_deg" in e.value.message


def test_config_bad_box_order_fails(tmp_path):
    path = write_variant(tmp_path, lambda d: d["limits"]["box_mm"].update(x=[-332, -632]))
    with pytest.raises(ConfigError) as e:
        load_robot_config(path)
    assert e.value.field == "limits.box_mm.x"


def test_config_speed_cap_range(tmp_path):
    path = write_variant(tmp_path, lambda d: d["limits"].update(speed_cap_pct=150))
    with pytest.raises(ConfigError) as e:
        load_robot_config(path)
    assert e.value.field == "limits.speed_cap_pct"


def test_config_real_driver_needs_connection(tmp_path):
    path = write_variant(tmp_path, lambda d: d.pop("connection"))
    with pytest.raises(ConfigError) as e:
        load_robot_config(path)
    assert e.value.field == "connection"


def test_config_home_outside_joint_limits_fails(tmp_path):
    path = write_variant(tmp_path, lambda d: d.update(home_joints_deg=[0, 0, 170, 0, -90, 0]))
    with pytest.raises(ConfigError) as e:
        load_robot_config(path)
    assert e.value.field == "home_joints_deg[2]"


def test_config_unknown_top_level_key_fails(tmp_path):
    path = write_variant(tmp_path, lambda d: d.update(limit={}))
    with pytest.raises(ConfigError) as e:
        load_robot_config(path)
    assert e.value.field == "limit"
