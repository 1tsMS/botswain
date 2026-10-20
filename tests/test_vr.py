"""VR teleop: message parsing, clutch / stick / trigger / A-B session, one input at a time, calibration math,
and the keyless server end to end (status push, newest headset wins, camera and mesh endpoints)."""

import json
import math
import socket
import ssl
import time
import urllib.error
import urllib.request

import numpy as np
import pytest
import websocket
from helpers import ROBOTS, SpyDriver, dh_tool, wait_for

from botswain.control.safety import SafetyGate
from botswain.control.worker import RobotWorker
from botswain.core.clock import now_ns
from botswain.core.config import load_robot_config
from botswain.core.types import Connect, Enable, Mode, Pose, StreamTarget
from botswain.teleop.mapping import TeleopMapper
from botswain.teleop.vr import (
    GRIP_PERIOD_NS,
    VrServer,
    VrSession,
    fit_base_to_xr,
    parse_vr,
    place_base,
    tool_tip_m,
)

SSL = {"cert_reqs": ssl.CERT_NONE}


def hand(x=0.0, y=1.0, z=0.0, grip=0.0, trigger=0.0, tracked=True, a=False, b=False, stick=(0, 0)):
    if not tracked:
        return {"tracked": False}
    return {"tracked": True, "pos": [x, y, z], "quat": [0, 0, 0, 1], "grip": grip, "trigger": trigger,
            "a": a, "b": b, "stick": list(stick)}


def msg(right):
    return {"type": "vr", "right": right, "left": {"tracked": False}, "head": {"pos": [0, 1.6, 0], "quat": [0, 0, 0, 1]}}


# ---- parsing ----

def test_parse_normalises_and_rejects_bad_numbers():
    s = parse_vr({**msg(hand(0.1, 1.2, -0.3, grip=1.5)), "right": {**hand(), "quat": [0, 0, 0, 2]}}, 1)
    assert s.right.quat_wxyz == (1.0, 0.0, 0.0, 0.0) and s.left is None and s.head_m == (0.0, 1.6, 0.0)
    assert parse_vr(msg(hand(grip=1.5)), 1).right.grip == 1.0  # clamped
    with pytest.raises(ValueError):
        parse_vr(msg({**hand(), "pos": [float("nan"), 0, 0]}), 1)
    with pytest.raises(ValueError):
        parse_vr(msg({**hand(), "quat": [0, 0, 0, 0]}), 1)
    assert parse_vr(msg(hand(tracked=False)), 1).right is None


# ---- session through a real worker ----

@pytest.fixture
def rig():
    cfg = load_robot_config(ROBOTS / "fake.yaml")
    driver = SpyDriver(cfg, lin_speed_mm_s=1000.0)
    worker = RobotWorker(driver, SafetyGate(cfg.limits, cfg.stream.period_s), tool=dh_tool())
    logs = []
    worker.on_log = lambda level, text: logs.append(text)
    worker.start()
    worker.submit(Connect())
    worker.submit(Enable())
    assert wait_for(lambda: worker.latest_state().mode is Mode.IDLE)
    session = VrSession(worker, TeleopMapper(scale_pos=1.0))
    yield driver, worker, session, logs
    worker.shutdown()


def feed(session, right, seconds=0.15):
    end = time.perf_counter() + seconds
    while time.perf_counter() < end:
        session.on_sample(parse_vr(msg(right), now_ns()))
        time.sleep(0.012)  # ~80 Hz like the headset


def test_clutch_moves_and_release_stops(rig):
    _, worker, session, _ = rig
    x0 = worker.latest_state().tcp.x
    feed(session, hand(z=0.0, grip=0.9))  # squeeze: anchor here
    feed(session, hand(z=-0.05, grip=0.9), 0.6)  # hand 5 cm forward (WebXR -z) -> robot +X 50 mm
    assert wait_for(lambda: worker.latest_state().tcp.x > x0 + 40, timeout=2)
    assert worker.commanded.source == "vr"
    feed(session, hand(z=-0.05, grip=0.1))
    assert wait_for(lambda: worker.latest_state().mode is Mode.IDLE)


def test_clutch_hysteresis_does_not_chatter(rig):
    _, _, session, _ = rig
    feed(session, hand(grip=0.5), 0.05)
    assert not session.clutch  # 0.5 is below the 0.6 needed to engage
    feed(session, hand(grip=0.7), 0.05)
    feed(session, hand(grip=0.5), 0.05)
    assert session.clutch  # and above the 0.4 needed to let go


def test_lost_tracking_stops_the_arm(rig):
    _, worker, session, _ = rig
    feed(session, hand(grip=0.9))
    feed(session, hand(z=-0.05, grip=0.9), 0.3)
    assert wait_for(lambda: worker.latest_state().mode is Mode.STREAMING)
    feed(session, hand(tracked=False), 0.1)
    assert not session.clutch
    assert wait_for(lambda: worker.latest_state().mode is Mode.IDLE)


def test_trigger_is_a_proportional_gripper(rig):
    _, worker, session, _ = rig
    feed(session, hand(trigger=0.0), 0.1)
    assert worker.latest_state().gripper is None  # connecting with the finger off the trigger never moves the jaws
    feed(session, hand(trigger=1.0), 0.2)
    assert wait_for(lambda: worker.latest_state().gripper == 0.0)  # pulled = closed
    assert wait_for(lambda: worker.latest_state().gripper_pos == pytest.approx(0.0, abs=0.01))
    feed(session, hand(trigger=0.5), 0.2)
    assert wait_for(lambda: worker.latest_state().gripper == pytest.approx(0.5, abs=0.01))
    assert wait_for(lambda: worker.latest_state().gripper_pos == pytest.approx(0.5, abs=0.01))  # the jaws went there
    feed(session, hand(trigger=0.0), 0.2)
    assert wait_for(lambda: worker.latest_state().gripper == 1.0)  # let go = open
    assert worker.commanded.gripper == 1.0  # what the recorder logs as the action


def test_gripper_targets_are_rate_limited_and_small_changes_skipped(rig):
    _, worker, session, _ = rig
    sent = []
    worker.submit = lambda cmd: sent.append(cmd)
    t = 1_000_000_000
    for i in range(40):  # trigger swept 0.2 -> 0.6 at 1 ms per sample: far faster than 10 Hz
        session.on_sample(parse_vr(msg(hand(trigger=0.2 + 0.01 * i)), t + i * 1_000_000))
    assert len(sent) == 1  # the first, then the rate limit holds the rest back
    session.on_sample(parse_vr(msg(hand(trigger=0.6)), t + GRIP_PERIOD_NS + 1))
    assert len(sent) == 2
    session.on_sample(parse_vr(msg(hand(trigger=0.62)), t + 3 * GRIP_PERIOD_NS))
    assert len(sent) == 2  # 2 % change: not worth a write
    session.on_sample(parse_vr(msg(hand(trigger=0.0)), t + 4 * GRIP_PERIOD_NS))
    assert sent[-1].opening == 1.0  # the ends always go out exactly


def test_thumbstick_turns_the_tool_about_z_while_held(rig):
    _, worker, session, _ = rig
    rz0 = worker.latest_state().tcp.rz
    feed(session, hand(grip=0.9), 0.05)
    feed(session, hand(grip=0.9, stick=(-1.0, 0.0)), 0.4)  # stick left = counter-clockwise from above = +Rz
    assert wait_for(lambda: worker.commanded.raw is not None and worker.commanded.raw.rz > rz0 + 5)
    session.on_disconnect()
    feed(session, hand(grip=0.0, stick=(-1.0, 0.0)), 0.1)  # no clutch: the stick does nothing
    assert not session.clutch


def test_a_and_b_step_the_motion_scale(rig):
    _, _, session, _ = rig
    assert session.scale_pct == 100
    feed(session, hand(a=True), 0.05)  # held for many frames: one step
    assert session.scale_pct == 90
    feed(session, hand(), 0.02)
    feed(session, hand(b=True), 0.02)
    feed(session, hand(), 0.02)
    feed(session, hand(b=True), 0.02)
    assert session.scale_pct == 100  # capped
    for _ in range(12):
        feed(session, hand(a=True), 0.02)
        feed(session, hand(), 0.02)
    assert session.scale_pct == 10  # floor


def test_after_a_reconnect_the_grip_must_be_let_go_once(rig):
    _, _, session, _ = rig
    session.on_disconnect()
    feed(session, hand(grip=0.9), 0.1)  # the page came back with the grip still squeezed
    assert not session.clutch and session.need_release
    feed(session, hand(grip=0.1), 0.05)
    feed(session, hand(grip=0.9), 0.05)
    assert session.clutch


def test_one_input_owns_the_arm(rig):
    _, worker, session, logs = rig
    feed(session, hand(grip=0.9))
    feed(session, hand(z=-0.03, grip=0.9), 0.2)
    assert wait_for(lambda: worker.latest_state().mode is Mode.STREAMING)
    p = worker.latest_state().tcp
    worker.stream(StreamTarget(p.with_axis("Y", p.y + 30), now_ns(), "phone", True))  # second hand on the arm
    assert wait_for(lambda: any("phone ignored: vr is moving the arm" in t for t in logs))
    feed(session, hand(z=-0.03, grip=0.0))  # VR lets go: the phone may take over now
    assert wait_for(lambda: worker.latest_state().mode is Mode.IDLE)
    for _ in range(10):
        p = worker.latest_state().tcp
        worker.stream(StreamTarget(p.with_axis("Y", p.y + 2), now_ns(), "phone", True))
        time.sleep(0.04)
    assert worker.commanded.source == "phone"


# ---- server end to end ----

def free_port():
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


def https_get(srv, path):
    ctx = ssl.create_default_context()
    ctx.check_hostname, ctx.verify_mode = False, ssl.CERT_NONE
    with urllib.request.urlopen(f"https://127.0.0.1:{srv.port}{path}", context=ctx, timeout=5) as r:
        return r.status, r.headers.get("content-type", ""), r.read()


def test_server_needs_no_key_streams_and_releases_on_disconnect(rig):
    _, worker, session, _ = rig
    srv = VrServer(session, port=free_port(), host="127.0.0.1")
    srv.start()
    try:
        assert srv.urls[0].endswith(f":{srv.port}/")  # nothing to type but the address
        status, _, body = https_get(srv, "/")
        assert status == 200 and b"Botswain" in body
        ws = websocket.create_connection(f"wss://127.0.0.1:{srv.port}/ws", sslopt=SSL)  # no cookie, no key
        ws.send(json.dumps(msg(hand(grip=0.9))))
        for i in range(25):
            ws.send(json.dumps(msg(hand(z=-0.002 * i, grip=0.9))))
            time.sleep(0.012)
        assert wait_for(lambda: worker.latest_state().mode is Mode.STREAMING)
        ws.send("not json")
        ws.close()  # the headset vanished with the clutch held
        # Faster than the 200 ms stale timeout: the disconnect itself released the clutch.
        assert wait_for(lambda: worker.latest_state().mode is Mode.IDLE, timeout=0.15)
        assert srv.bad_messages == 1 and srv.samples >= 26
    finally:
        srv.stop()


def test_server_pushes_status_and_the_newest_headset_wins(rig):
    _, _, session, _ = rig
    cfg = load_robot_config(ROBOTS / "dobot_cr5.yaml")
    srv = VrServer(session, port=free_port(), host="127.0.0.1", urdf=cfg.model_urdf, tool_offset_mm=(0, 0, 100))
    srv.start()
    try:
        first = websocket.create_connection(f"wss://127.0.0.1:{srv.port}/ws", sslopt=SSL)
        status = json.loads(first.recv())
        assert status["type"] == "status" and status["mode"] == "idle" and status["link"] is True
        assert len(status["links"]) == len(srv._meshes) > 0 and len(status["links"][0]) == 16
        assert status["calib"]["matrix"] is None and status["scale"] == 100
        second = websocket.create_connection(f"wss://127.0.0.1:{srv.port}/ws", sslopt=SSL)  # headset reconnected
        assert wait_for(lambda: srv.clients == 1)
        with pytest.raises(websocket.WebSocketConnectionClosedException):
            for _ in range(100):
                first.recv()  # the old socket was closed by the server
        assert json.loads(second.recv())["type"] == "status"
        second.close()
    finally:
        srv.stop()


def test_server_serves_meshes_three_js_and_camera_frames(rig):
    _, _, session, _ = rig

    class Cam:
        name = "top"

        def latest(self):
            return np.full((720, 1280, 3), 128, dtype=np.uint8), 0

    cfg = load_robot_config(ROBOTS / "dobot_cr5.yaml")
    srv = VrServer(session, port=free_port(), host="127.0.0.1", urdf=cfg.model_urdf, cameras=lambda: [Cam()])
    srv.start()
    try:
        _, kind, body = https_get(srv, "/robot.json")
        robot = json.loads(body)
        assert robot["tip"] == "flange centre" and len(robot["meshes"]) == 7
        _, _, stl = https_get(srv, robot["meshes"][0]["url"])
        assert len(stl) > 84  # binary STL: 80 byte header + count + triangles
        _, kind, js = https_get(srv, "/vendor/three.module.min.js")
        assert "javascript" in kind and len(js) > 100_000
        with pytest.raises(urllib.error.HTTPError):
            https_get(srv, "/vendor/..%2F..%2Fbotswain%2Fapp.py")  # only the two vendored files
        _, kind, jpeg = https_get(srv, "/cam/0.jpg")
        import cv2
        frame = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
        assert kind == "image/jpeg" and frame.shape[1] == 480  # scaled down for the headset
        with pytest.raises(urllib.error.HTTPError):
            https_get(srv, "/cam/1.jpg")
        assert srv.status()["cams"] == ["top"]
    finally:
        srv.stop()


# ---- calibration math ----

def _xr_of(matrix, base_point):
    return (matrix @ np.array([*base_point, 1.0]))[:3]


def test_fit_recovers_heading_and_offset():
    # Robot base 1.2 m in front of the headset's origin, on a 0.8 m table, robot +X pointing to the user's left.
    theta = math.radians(90)
    truth = np.eye(4)
    rz = np.array([[math.cos(theta), -math.sin(theta), 0], [math.sin(theta), math.cos(theta), 0], [0, 0, 1]])
    floor_to_xr = np.array([[1.0, 0, 0], [0, 0, 1.0], [0, -1.0, 0]])  # x right, y forward, z up -> WebXR
    truth[:3, :3] = floor_to_xr @ rz
    truth[:3, 3] = floor_to_xr @ np.array([0.0, 1.2, 0.8])
    base = [(-0.5, -0.1, 0.4), (-0.3, 0.2, 0.3), (-0.45, 0.15, 0.6)]
    xr = [_xr_of(truth, p) + np.random.default_rng(i).normal(0, 0.002, 3) for i, p in enumerate(base)]
    matrix, rms = fit_base_to_xr(base, xr)
    assert rms < 0.005
    assert np.allclose(matrix[:3, :3], truth[:3, :3], atol=0.02) and np.allclose(matrix[:3, 3], truth[:3, 3], atol=0.01)
    with pytest.raises(ValueError):
        fit_base_to_xr([(-0.5, 0, 0.4), (-0.5, 0.01, 0.6)], xr[:2])  # only moved up: no heading


AHEAD = (0.0, 0.0, 0.0, 1.0)  # controller held normally, pointing away from you (WebXR -z)
HOME_TIP = (-0.47, -0.14, 0.47)  # roughly the CR5's home: tool on the -X side of the base


def test_place_puts_the_tip_at_the_dot_and_the_base_behind_it():
    dot = (0.2, 0.9, -0.5)
    matrix = place_base(dot, AHEAD, HOME_TIP)
    assert np.allclose(_xr_of(matrix, HOME_TIP), dot)  # the tool tip is exactly at the white dot
    base = _xr_of(matrix, (0, 0, 0))
    assert base[2] < dot[2] - 0.4  # the base is further away from you, where the controller points
    assert base[0] == pytest.approx(dot[0], abs=1e-9)  # straight ahead, not off to a side
    assert base[1] == pytest.approx(0.9 - 0.47)  # and at the right height: the tip is 0.47 m above the base
    assert np.allclose(matrix[:3, 2], (0, 1, 0), atol=1e-9)  # level: robot +Z is up
    with pytest.raises(ValueError):
        place_base(dot, AHEAD, (0.01, 0.0, 0.8))  # tool right above the base: no direction to line up


def test_tool_tip_adds_the_offset_along_the_tool():
    down = Pose(-400, 100, 300, 180, 0, 0)  # flange pointing down
    assert np.allclose(tool_tip_m(down, None), (-0.4, 0.1, 0.3))
    assert np.allclose(tool_tip_m(down, (0, 0, 150)), (-0.4, 0.1, 0.15))  # 150 mm further down


def test_placed_robot_drives_in_room_directions(rig):
    # MS's case: placed from the tip at home, the arm reaches back at you. Hand toward your chest must bring the
    # tool toward you (robot -X here), hand away sends it away (+X).
    _, worker, session, _ = rig
    session.set_room(place_base((0.0, 0.9, -0.5), AHEAD, (-0.5, 0.0, 0.4)))
    x0 = worker.latest_state().tcp.x
    feed(session, hand(z=0.0, grip=0.9))
    feed(session, hand(z=0.05, grip=0.9), 0.6)  # hand 5 cm toward your chest
    assert wait_for(lambda: worker.latest_state().tcp.x < x0 - 40, timeout=2)
    feed(session, hand(z=0.05, grip=0.0))
    session.set_room(None)  # reset: the GUI's facing (identity here) is back
    feed(session, hand(z=0.05, grip=0.0), 0.05)
    assert np.allclose(session.mapper.remap, np.eye(3))


def test_room_remap_matches_the_overlay():
    from botswain.teleop.vr import room_remap
    # Tool toward you, base away: robot +X points away from you, which is the GUI's default facing.
    assert np.allclose(room_remap(place_base((0, 0.9, -0.5), AHEAD, (-0.5, 0.0, 0.4))), np.eye(3), atol=1e-9)


def test_right_stick_click_toggles_wrist_rotation(rig):
    _, _, session, _ = rig
    assert session.mapper.scale_rot == 0.0
    feed(session, {**hand(), "stick_press": True}, 0.05)  # held for many frames: one toggle
    assert session.mapper.scale_rot == 1.0
    feed(session, hand(), 0.02)
    feed(session, {**hand(), "stick_press": True}, 0.02)
    assert session.mapper.scale_rot == 0.0


def test_a_hand_dragged_pose_is_accepted_only_if_rigid_and_upright():
    from botswain.teleop.vr import Calibration
    cal = Calibration()
    good = place_base((0.0, 0.9, -0.5), AHEAD, HOME_TIP)
    cal.set_matrix(good.reshape(-1).tolist())  # row-major, like the page sends it
    assert np.allclose(cal.matrix, good) and cal.note.startswith("placed by hand")
    tilted = good.copy()
    tilted[:3, :3] = tilted[:3, :3] @ np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]])  # robot lying on its side
    with pytest.raises(ValueError):
        cal.set_matrix(tilted.reshape(-1).tolist())
    with pytest.raises(ValueError):
        cal.set_matrix((good * 2).reshape(-1).tolist())  # scaled: not rigid
    assert np.allclose(cal.matrix, good)  # a refused pose leaves the placement alone


def test_page_errors_and_refused_placements_reach_the_log(rig, caplog):
    _, _, session, _ = rig
    srv = VrServer(session, port=free_port(), host="127.0.0.1")
    srv._handle(json.dumps({"type": "log", "text": "TypeError: x is undefined"}))
    srv._handle(json.dumps({"type": "log", "text": "flood"}))  # same second: not logged again
    lying = np.eye(4)
    srv._handle(json.dumps({"type": "calib", "op": "set", "matrix": lying.reshape(-1).tolist()}))  # robot z = room -z
    text = caplog.text
    assert "VR page error: TypeError: x is undefined" in text and "flood" not in text
    assert "VR calibration 'set' refused: robot must stand upright" in text
    assert srv.calibration.matrix is None and srv.bad_messages == 1
