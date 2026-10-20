import pytest
import yaml
from helpers import DH_TOOL, ROBOTS

from botswain.core import config as config_mod
from botswain.core.config import (
    LIMIT_KEYS,
    ConfigError,
    discover,
    load_robot_config,
    load_tool_config,
    save_overlay,
)


def write_variant(tmp_path, change):
    """Copy dobot_cr5.yaml, let `change` edit the dict, write it to tmp_path."""
    data = yaml.safe_load((ROBOTS / "dobot_cr5.yaml").read_text(encoding="utf-8"))
    data["urdf"] = str(load_robot_config(ROBOTS / "dobot_cr5.yaml").model_urdf)  # still found from tmp_path
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
    assert cfg.protocol == "fake" and cfg.connection is None and cfg.tool_port == "rs485"


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
    path = write_variant(tmp_path, lambda d: d["home"].update(joints_deg=[0, 0, 170, 0, -90, 0]))
    with pytest.raises(ConfigError) as e:
        load_robot_config(path)
    assert e.value.field == "home.joints_deg[2]"


def test_config_unknown_top_level_key_fails(tmp_path):
    path = write_variant(tmp_path, lambda d: d.update(limit={}))
    with pytest.raises(ConfigError) as e:
        load_robot_config(path)
    assert e.value.field == "limit"



def test_cr5_speaks_v3_and_names_its_ports():
    cfg = load_robot_config(ROBOTS / "dobot_cr5.yaml")
    assert cfg.protocol == "dobot_v3" and cfg.vendor == "Dobot" and cfg.model == "CR5"
    assert cfg.connection.port("dashboard") == 29999 and cfg.connection.port("http") == 22000
    assert cfg.connection.port("not there", 7) == 7


def test_missing_urdf_is_caught(tmp_path):
    path = write_variant(tmp_path, lambda d: d.update(urdf="nope/robot.urdf"))
    with pytest.raises(ConfigError) as e:
        load_robot_config(path)
    assert e.value.field == "urdf"


def test_bad_port_number_is_caught(tmp_path):
    path = write_variant(tmp_path, lambda d: d["connection"]["ports"].update(feedback=70000))
    with pytest.raises(ConfigError) as e:
        load_robot_config(path)
    assert e.value.field == "connection.ports.feedback"


def test_tool_config_loads_the_lab_gripper():
    t = load_tool_config(DH_TOOL)
    assert (t.driver, t.port, t.slave, t.force_range_pct) == ("dh_modbus", "rs485", 1, (20, 100))
    assert t.mass_kg is None and t.tcp_offset_mm is None  # not filled in yet: unknown stays unknown


def test_overlay_changes_a_copy_never_the_shared_file(tmp_path):
    path = write_variant(tmp_path, lambda d: None)
    shared = path.read_text(encoding="utf-8")
    local = save_overlay(path, {"connection": {"ip": "192.168.5.7"}, "limits": {"box_enabled": True}})
    assert local.name == "robot.local.yaml" and path.read_text(encoding="utf-8") == shared
    cfg = load_robot_config(path)
    assert cfg.connection.ip == "192.168.5.7" and cfg.limits.box_enabled
    assert cfg.connection.port("dashboard") == 29999  # the rest of the section is kept
    save_overlay(path, {"limits": {"speed_cap_pct": 50}})  # a later edit keeps the earlier ones
    cfg = load_robot_config(path)
    assert cfg.limits.speed_cap_pct == 50 and cfg.connection.ip == "192.168.5.7"


def test_discover_lists_good_and_broken_configs(tmp_path):
    write_variant(tmp_path, lambda d: None)
    (tmp_path / "broken.yaml").write_text("name: x\n", encoding="utf-8")
    save_overlay(tmp_path / "robot.yaml", {"limits": {"speed_cap_pct": 40}})
    found = {p.name: (cfg, err) for p, cfg, err in discover(tmp_path, load_robot_config)}
    assert set(found) == {"robot.yaml", "broken.yaml"}  # the overlay is not a config of its own
    assert found["robot.yaml"][0].limits.speed_cap_pct == 40 and "protocol" in found["broken.yaml"][1]


def test_selection_round_trip(tmp_path, monkeypatch):
    robots, tools = tmp_path / "robots", tmp_path / "tools"
    robots.mkdir()
    tools.mkdir()
    (robots / "a.yaml").write_text("x", encoding="utf-8")
    (tools / "t.yaml").write_text("x", encoding="utf-8")
    monkeypatch.setattr(config_mod, "ROBOT_DIR", robots)
    monkeypatch.setattr(config_mod, "TOOL_DIR", tools)
    monkeypatch.setattr(config_mod, "SELECTION", tmp_path / "selection.yaml")
    assert config_mod.read_selection() == (None, None)
    config_mod.write_selection(robots / "a.yaml", tools / "t.yaml")
    assert config_mod.read_selection() == (robots / "a.yaml", tools / "t.yaml")
    (tools / "t.yaml").unlink()  # a deleted config is simply not picked any more
    assert config_mod.read_selection() == (robots / "a.yaml", None)


def test_tool_needs_a_matching_port():
    from dataclasses import replace

    from botswain.drivers.registry import make_tool

    robot = load_robot_config(ROBOTS / "fake.yaml")
    with pytest.raises(ConfigError, match="needs an rs485 tool port"):
        make_tool(load_tool_config(DH_TOOL), replace(robot, tool_port=None))
