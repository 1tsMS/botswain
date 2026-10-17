# Botswain spec

Version 1, 2026-10-17. Living document: change this file first, then the code.
Background facts and sources: [RESEARCH_NOTES.md](RESEARCH_NOTES.md). Build order: [PLAN.md](PLAN.md).

## 1. Objective

One Windows desktop app to **connect, check, jog, teleop (phone) and record datasets** for different cobots.
First robot: the lab **Dobot CR5** (V4 controller, 192.168.5.1). Next: Addverb Syncro.

Who uses it: MS (and later other lab people) running experiments for the cobot research:
- P1: compare teleop interfaces (needs clean teleop + recordings).
- P2: commanded vs measured action labels (needs the recorder to log what we commanded AND what the robot did, on one clock).

Open-source second: other people add a driver file for their robot; nothing else changes.

What "done" means for the weekend build: on Monday the app connects to the real CR5, shows live state,
passes the Doctor checks, and jogs the arm safely (Must). Phone teleop and recording are Should/Nice.

## 2. Tech stack

| What | Choice | Why |
|---|---|---|
| OS | **Windows 10/11 only** (for now) | MS's laptop and the lab PC are Windows |
| Language | Python 3.12 | vendor SDKs, LeRobot are Python; LeRobot needs >= 3.12 |
| GUI | PySide6 (Qt 6, LGPL) | PyQt is GPL or paid |
| Numbers | numpy 2.x | LeRobot needs numpy >= 2, pick it from day one so T6 doesn't break things |
| Config | PyYAML | one YAML per robot |
| 3D view | yourdfpy + pyvista + pyvistaqt | urdfpy is dead |
| Cameras | opencv-python (DirectShow backend on Windows) | |
| Rotations | transforms3d | already a dependency of `teleop` |
| Phone teleop | `teleop` (SpesRobotics, Apache-2.0) | WebXR phone page + HTTPS server |
| Dataset | `lerobot` (v0.6.x), installed only at T6 | pulls torch, keep it out of T1-T5 |
| Tests | pytest, pytest-qt | |
| Lint | ruff | |

No ROS. No C++ unless a vendor forces it inside one driver.

## 3. Commands

All installs stay on D: (C: is nearly full). Run from `D:\College\projects\botswain`.

```bash
py -3.12 -m venv .venv
```
```bash
set PIP_CACHE_DIR=D:\Softwares\MS_agent\Claude\cache\pip && .venv\Scripts\python -m pip install -e .[dev]
```
```bash
.venv\Scripts\python -m botswain                      # start the app (opens with the last robot config)
.venv\Scripts\python -m botswain --robot robots/fake.yaml
.venv\Scripts\python -m botswain.drivers.dobot.fake_controller   # pretend Dobot on localhost, for testing the real driver
.venv\Scripts\python -m pytest -q
.venv\Scripts\python -m ruff check .
```

## 4. Project structure

Organized by job, many small files, no folder deeper than needed.

```
botswain/                         repo root
  pyproject.toml                  deps + entry point
  README.md  SPEC.md  PLAN.md  RESEARCH_NOTES.md
  robots/                         one YAML per robot (+ saved points)
    fake.yaml
    dobot_cr5.yaml
    dobot_cr5_fake.yaml           real Dobot driver pointed at the fake controller on localhost
    dobot_cr5.points.json
  assets/
    dobot_cr5/                    cr5_robot.urdf + meshes/*.STL + LICENSE (MIT, from Dobot)
  botswain/                       the Python package
    __main__.py                   python -m botswain
    app.py                        builds config -> driver -> worker -> window, wires signals
    core/
      types.py                    Pose, Joints, RobotState, Mode, Capabilities, command messages
      clock.py                    the one clock (monotonic ns)
      config.py                   load + validate robot YAML into dataclasses
      log.py                      logging setup (file + GUI log dock)
    drivers/
      base.py                     Driver contract (the "buttons") + capability card
      registry.py                 "dobot_v4" -> DobotDriver, "fake" -> FakeDriver
      fake.py                     fake robot (pure Python, no network)
      dobot/
        commands.py               build exact command strings (pure functions)
        dashboard.py              29999 client: send, read until ';', parse ErrorID
        feedback.py               30004 reader: framing, parse 1440-byte packet
        gripper.py                DH gripper over HTTP 22000 (Modbus RTU frames + CRC)
        driver.py                 DobotDriver: implements the contract using the 4 files above
        fake_controller.py        pretend Dobot TCP server (29999 + 30004 + 22000)
    control/
      worker.py                   background thread that owns the driver; command queue; STOP lane
      safety.py                   SafetyGate: box, step clamp, speed cap, hold-to-move, ready check
      doctor.py                   no-motion health checks -> list of results
      points.py                   saved points (load/save JSON)
    teleop/
      phone.py                    runs the `teleop` HTTPS server in a thread, turns messages into PhoneSample
      mapping.py                  phone motion -> robot target (anchor, scale, axis remap)
    sensors/
      cameras.py                  one thread per USB camera, newest frame + timestamp
    record/
      recorder.py                 fixed-rate sampler on the one clock
      lerobot_writer.py           writes LeRobotDataset (lazy import)
    ui/
      theme.py                    colors, fonts (IBM Plex), QSS
      main_window.py              menu, toolbar, rail, pages, docks, status bar
      toolbar.py  rail.py  status_bar.py  docks.py
      widgets/                    panel (collapsible header), pose_fields, jog_pad, speed_control, robot_view, camera_tile
      pages/                      jog_page, points_page, teleop_page, cameras_page, settings_page
      doctor_dialog.py
  tests/                          one test file per module, e.g. test_safety.py, test_feedback.py
```

`mockup_final.html` and `mockup_c_session.html` stay in the root as the UI reference.

## 5. Architecture (how data flows)

```
 inputs                       worker thread (owns the driver)                robot
 ------                       -------------------------------                -----
 GUI jog pad  ─┐
 Move to target├─> command queue ─> SafetyGate ─> Driver ─> socket ─────────> CR5 :29999
 Phone (T5)   ─┘        ^              │ clamp/reject + reason
 STOP button ───────────┘ (STOP lane: jumps the queue, clears it)
                                                     Driver feedback thread <── CR5 :30004 (8 ms)
                         worker reads state every tick ──> Qt signal (30 Hz) ──> GUI telemetry, 3D view
                                                       └─> recorder (T6), with the commanded target
 cameras (T5) ── own threads, newest frame + timestamp ──> GUI docks + recorder
```

Rules (non-negotiable):
1. **GUI never talks to the robot.** It only puts commands in the worker queue and shows state from signals.
2. **Every motion goes through the SafetyGate.** Drivers' motion methods are called only by the worker, after the gate.
3. **One clock.** Every sample (robot state, camera frame, phone pose, command) is stamped with `clock.now_ns()`
   (`time.perf_counter_ns()`) the moment it arrives in our process. The session start also stores wall time once, to map back.
4. **Connect never enables.** Enable is its own button. (Phone2Act enabled on start; we don't.)
5. **STOP is a real stop:** clears our queue, ends any stream or jog, sends the driver's stop. It never toggles anything back on.
6. Test on the fake robot, then the fake Dobot controller, then the real CR5. Only then call it working.

### 5.1 Core types (core/types.py)

Units inside the app: **mm and degrees**, same as Dobot and the GUI. Frozen dataclasses.

- `Pose(x, y, z, rx, ry, rz)`; `Joints(j: tuple[float, ...])` (length = dof).
- `Mode` enum: `DISCONNECTED, POWERED_OFF, DISABLED, IDLE, MOVING, JOGGING, STREAMING, DRAG, PAUSED, ERROR, COLLISION, UNKNOWN`.
  Each driver maps its own codes (Dobot: 4 -> DISABLED, 5 -> IDLE, 7 -> MOVING, 8 -> JOGGING, 9 -> ERROR, 11 -> COLLISION...).
- `RobotState`: `t_ns, mode, joints, tcp, joints_target, tcp_target, speed_pct, errors: tuple[int,...], gripper: float | None, raw_mode: int, link_ok: bool, feedback_hz: float`.
- Commands (what goes in the queue): `Enable, Disable, ClearError, Stop, SetSpeed(pct), JogStep(axis, delta, frame)`,
  `JogHold(axis, direction, frame)` / `JogRelease`, `MoveTo(target: Pose | Joints, kind: "joint"|"linear")`,
  `JogTick` (the "still held" tick), `StreamTarget(pose, t_ns, source, hold)`, `Gripper(open: bool)`.

### 5.2 Driver contract (drivers/base.py)

A driver is a "travel adapter": every brand answers the same buttons.

```python
class Driver(ABC):
    capabilities: Capabilities              # the "what I can do" card

    def connect(self) -> None: ...          # open links, start feedback; never enables
    def disconnect(self) -> None: ...       # stop motion first, then close
    def read_state(self) -> RobotState: ... # newest state, must not block (> 1 ms is a bug)
    def enable(self) -> None: ...
    def disable(self) -> None: ...
    def clear_error(self) -> None: ...
    def stop(self) -> None: ...             # must work in any mode, never raises for "already stopped"
    def set_speed(self, pct: int) -> None: ...
    def jog_step(self, axis: str, delta: float, frame: str) -> None: ...   # "X".."Rz" mm/deg or "J1".."J6" deg
    def move_to(self, target: Pose | Joints, kind: str) -> None: ...      # returns when accepted, not when done
    # optional, only if the card says so:
    def jog_hold(self, axis: str, direction: int, frame: str) -> None: ...
    def jog_release(self) -> None: ...
    def stream_target(self, pose: Pose) -> None: ...                      # one teleop point
    def gripper(self, open: bool) -> None: ...
    def diagnose(self) -> list[Check]: ...  # driver-specific Doctor checks (ports, packet rate...)
```

```python
@dataclass(frozen=True)
class Capabilities:
    name: str; dof: int
    step_jog: bool; hold_jog: bool; streaming: bool; stream_rate_hz: float
    gripper: bool; drag_mode: bool; frames: tuple[str, ...]   # ("base", "tool")
    max_tcp_speed_mm_s: float
```

The GUI reads the card and hides what the robot can't do (no gripper -> no gripper buttons).
Driver errors raise `DriverError(code, message, hint)`; the worker catches them and logs them. Nothing is swallowed.

### 5.3 Fake robot (drivers/fake.py)

Pure Python, no network, used for GUI work and most tests.
- Keeps joints and TCP pose. Moves toward its target at a capped speed each `read_state` (simple first-order step).
  Joint and Cartesian state are **not** kinematically linked (no IK); it is a stand-in, the 3D view uses joints.
- Has the same mode machine as Dobot: starts DISABLED, enable -> IDLE, move -> MOVING -> IDLE, error injection -> ERROR.
- Test hooks: `inject_error(code)`, `drop_link()`, `set_latency(ms)`.

### 5.4 Fake Dobot controller (drivers/dobot/fake_controller.py)

A pretend CR5 that speaks the real protocol on localhost, so the **real** DobotDriver runs end to end at home.
- 29999: parses the commands we use, replies `ErrorID,{...},Cmd(...);` with the real error codes
  (-2 in alarm, -6 bad MoveJog coordtype, -10000 unknown). Mode machine matches section 1 of the research notes.
  Optional "not in TCP mode" switch that answers like a controller in local mode.
- 30004: sends valid 1440-byte packets every 8 ms (len, TestValue, RobotMode, QActual, ToolVectorActual,
  QTarget, ToolVectorTarget at the right offsets). Test switch to **split packets across sends** and to
  send garbage bytes, to prove our framing.
- 22000: `GET /protocol/getAlarm`, `POST /interface/toolDataExchange` (logs gripper frames, checks CRC).
- Moves its pose for MovJ/MovL/RelMovL*/ServoP/MoveJog so jog and teleop can be watched in the GUI.
- Ports configurable (tests use random free ports).

### 5.5 Dobot driver (drivers/dobot/)

- Commands on **29999** (dashboard). `motion_port` option in YAML (default 29999, can be 30003).
- Reader thread on 30004: byte buffer, cut 1440-byte frames, check len + TestValue, resync on failure,
  keep only the newest parsed state. Counts packet rate (Hz) and bad frames.
- Every reply parsed; ErrorID != 0 -> `DriverError` with the meaning from the error table.
- `stop()` = `MoveJog()` then `Stop()`. `disconnect()` calls `stop()` first.
- `stream_target()` = `ServoP(..., t=<period>, aheadtime, gain)`, period from YAML (start 0.03 s).
- Step jog = `RelMovLUser` / `RelMovLTool` / `RelJointMovJ`. Hold jog = `MoveJog(axis,coordtype=..)` + `MoveJog()`.
- Gripper = DH Modbus RTU frames sent through HTTP `toolDataExchange` (init once, then position); log the HTTP reply.
- Never calls `EnableRobot` on connect.

### 5.6 Worker (control/worker.py)

- A plain `threading.Thread` that owns the driver. Loop at a fixed tick (default 125 Hz = 8 ms).
- Each tick: read state; check STOP flag; take queued commands; run them through the gate; call driver;
  handle the jog-hold watchdog; publish state to the GUI at most 30 Hz through a `QObject` bridge signal
  (Qt queues it to the GUI thread).
- STOP lane: `request_stop()` sets a `threading.Event` the loop checks first; it clears the queue and calls `driver.stop()`.
  It can be called from any thread.
- Hold-jog watchdog: while a hold-jog is active, the GUI sends a "still held" tick every 50 ms;
  no tick for 150 ms -> `jog_release()`.
- Driver exceptions are caught per command, logged with the hint, shown in the log dock. The loop keeps running.
- Disconnect / app close: `stop()`, then `disconnect()`, then thread join.

### 5.7 Safety gate (control/safety.py)

Sits in the worker. Pure function style: `gate.check(cmd, state) -> Allowed(cmd') | Rejected(reason)`.
Limits come from the robot YAML. First-run values are small on purpose.

| Rule | Applies to | Behaviour |
|---|---|---|
| Ready check | all motion | Only when `mode` is IDLE (or STREAMING/JOGGING for their own commands). Otherwise reject with "robot not enabled / in error". |
| Workspace box | MoveTo | target outside the box -> **reject** with a message (never silently go somewhere else) |
| Workspace box | JogStep, StreamTarget | **clamp** the target onto the box (the arm stops at the wall) |
| Step clamp | StreamTarget | limit the distance from the last sent target to `max_step_mm` (and `max_step_deg`) per tick. **Clamp, never drop.** The arm keeps following, just slower. |
| Teleport guard | StreamTarget | jump bigger than `teleport_mm` (tracking glitch) -> hold position and ask the mapping to **re-anchor**; nothing is sent for that sample |
| Speed cap | all | speed % clamped to `speed_cap_pct`; ServoP step limited by `max_tcp_speed_mm_s * period` |
| Hold-to-move | StreamTarget | accepted only while the phone's hold button is down AND the sample is younger than `stale_ms` (200). Release or stale -> `stop()` |
| Joint limits | joint jog / MoveTo joints | soft limits from YAML: clamp jog, reject MoveTo; warn in the log when within 5 deg |
| First target | StreamTarget | the first target after hold-down is the current measured pose (seeded), so there is no jump |

Every clamp/reject is counted and shown (status bar + log, rate-limited to one line per second per rule).
The gate keeps the last sent target; the recorder logs both the input (before gate) and the output (after gate).

### 5.8 Robot YAML (robots/dobot_cr5.yaml)

```yaml
name: Dobot CR5
driver: dobot_v4
connection: {ip: 192.168.5.1, dashboard_port: 29999, motion_port: 29999, feedback_port: 30004, http_port: 22000}
model: {urdf: ../assets/dobot_cr5/cr5_robot.urdf}   # relative paths are relative to this YAML file
home_confirmed: false   # Home stays disabled until the pose is checked in the cell
limits:
  speed_cap_pct: 20
  box_mm: {x: [-632, -332], y: [-286, 14], z: [334, 634]}   # +-150 around the Phone2Act start pose; widen step by step
  joints_deg: [[-360, 360], [-360, 360], [-160, 160], [-360, 360], [-360, 360], [-360, 360]]
  max_tcp_speed_mm_s: 150
  max_step_deg: 2.0
  teleport_mm: 80
  stale_ms: 200
stream: {period_s: 0.03, aheadtime: 50, gain: 500}
gripper: {type: dh_modbus, slave: 1, baud: 115200}
home_joints_deg: [0, 0, 90, 0, -90, 0]   # [ ] confirm with the lab before using Home
```

### 5.9 Doctor (control/doctor.py)

No-motion checks, run on demand from the toolbar. Each result: name, ok/warn/fail, value, fix hint.
1. Ping the IP (Windows `ping -n 1 -w 500`).
2. TCP connect to each port (29999, 30004, motion port, 22000).
3. Feedback: first valid packet within 1 s, measured rate (expect ~125 Hz), bad-frame count.
4. TCP control mode: the dashboard answers (not `Not Tcp`); if not, hint "switch to TCP in DobotStudio Pro or press Take control (fw >= 4.6)".
5. RobotMode with its meaning; GetErrorID + alarm text from HTTP 22000.
6. Pose agreement: `GetPose()` vs feedback ToolVectorActual (within 0.5 mm / 0.5 deg).
7. Model check (after T2): URDF FK of QActual vs ToolVectorActual (warn only, tool offset may differ).

### 5.10 Teleop (teleop/)

- `phone.py` runs the `teleop` package server (HTTPS, port 4443) in a thread and turns each message into
  `PhoneSample(t_ns, position_m, quat, hold, gripper, scale)`. Uses our own self-signed cert if present.
- `mapping.py` (ours, testable): on hold-down store phone anchor + robot anchor (measured pose);
  target = robot anchor + R_remap * (phone - phone anchor) * scale_pos. Rotation off by default (`scale_rot: 0`)
  for first runs, like Phone2Act's first-run config. Re-anchor on teleport guard.
- Worker streams the newest target each tick through the gate (not every phone message; newest wins).

### 5.11 Cameras (sensors/cameras.py)

One thread per camera (`cv2.VideoCapture(index, cv2.CAP_DSHOW)`), keeps newest frame + `t_ns`. The app runs fine with zero cameras.

### 5.12 Recorder (record/)

- Samples on a fixed grid (`fps`, default 30) using the one clock; LeRobot computes `timestamp = frame_index / fps`,
  so real times go in extra features.
- Per frame:
  - `observation.state` = measured joints (6) + TCP pose (6) + gripper (1)
  - `action` = **commanded** target after the gate (pose 6 + gripper 1), the thing we actually sent
  - `observation.command_raw` = input before the gate (shows clamping)
  - `observation.controller_target` = Dobot ToolVectorTarget (6)
  - `observation.t_state_ns`, `observation.t_command_ns`, `observation.t_cam_<name>_ns` (real arrival times, as float64)
  - `observation.images.<name>` per camera
- Episode bar: Start (R), Save (S), Discard (D). Discard = `clear_episode_buffer()`. Always `finalize()` on close.
- Dataset path under D: (default `D:\data\<task>`).

### 5.13 UI (per mockup_final.html)

Style (locked): black chrome (menu bar, toolbar, left rail, bottom docks, status bar) around a light workspace
(#e9e9e6, panels #f7f7f5). IBM Plex Sans + IBM Plex Mono with tabular numbers (bundle the TTFs, load with
`QFontDatabase`). Red #e5322d only for STOP / record / danger, blue #1f6feb only for selection/focus.
No animation except a press effect on buttons; nothing on jog keys.

Structure:
- Menu bar: File, Robot, View, Tools, Help + current robot YAML path on the right.
- Toolbar (icon over label, grouped): Connection (Connect/Disconnect, Doctor) | Robot (Enable/Disable, Clear error,
  Home, Drag mode if the card says so) | Data (Record, Save point) | right side: robot state text + big red STOP.
- Left rail: Script, Blocks (both "coming later" placeholders), Points, Jog, Teleop, Cameras, Settings.
- Jog page: Orientation panel (frame select, live pose + joint fields, Copy current / Move to target) |
  Control panel (X..Rz pairs left, J1..J6 right, step selector 0.5/1/5/10/Hold, gripper open/close) + Speed control
  (-1/-5/-15/-25, slider, +1..+25, shows %) | Robot view (3D, chips: Safety box, Target).
- Bottom docks: Log / Points / Episodes tabs, Cameras.
- Status bar: link, feedback Hz, latency, frame, safety box size + clamp count, dataset path.

Fixes vs the mockup (details MS allowed us to change):
- "Unlocked" button becomes a clear **Enable / Disable** toggle with state color.
- Pose/joint fields are live and read-only until you click "Edit target"; typing never moves the arm. Only "Move to target" moves.
- Jog buttons are disabled (greyed) unless the robot is IDLE/JOGGING; reason shown on hover.
- **STOP shortcut: Space and Esc as application-wide shortcuts. All jog/move buttons use `Qt.NoFocus`**, because in Qt
  Space "clicks" the focused button; without this, Space could press a jog key instead of stopping.
- Near-limit fields turn red from real limits in the YAML (not hardcoded).

## 6. Code style

- Type hints everywhere, dataclasses for data, no globals, no `eval`. Small functions, one job per file.
- Errors are raised with a hint and logged once; never `except: pass`.
- Comments explain *why*, not what. Plain English names.
- ruff default rules, line length 110.

```python
def clamp_step(last: Pose, target: Pose, max_mm: float) -> tuple[Pose, bool]:
    """Move from `last` toward `target` by at most `max_mm`. Returns (pose, was_clamped).

    Clamp, never drop: a fast phone move makes the arm follow slower, not freeze.
    """
    d = np.array(target.xyz) - np.array(last.xyz)
    dist = float(np.linalg.norm(d))
    if dist <= max_mm:
        return target, False
    xyz = np.array(last.xyz) + d * (max_mm / dist)
    return replace(target, x=xyz[0], y=xyz[1], z=xyz[2]), True
```

## 7. Testing strategy

- pytest, tests in `tests/`, one file per module. Run before every commit.
- Pure logic (types, commands, feedback parser, safety gate, mapping, points, recorder timing): unit tests, no network.
- Dobot driver: integration tests against `fake_controller` on random localhost ports (split packets, garbage, alarms, Not Tcp).
- Worker: tests with the fake robot (STOP clears queue, watchdog releases hold jog, exceptions don't kill the loop).
- GUI: pytest-qt smoke tests (window opens, buttons hidden per capability card, Space triggers STOP not a jog key).
- Real robot: only the Monday checklist below. Never in automated tests.
- Coverage target: safety.py, feedback.py, commands.py, mapping.py at 100% of branches; the rest "what matters".

## 8. Boundaries

- **Always:** route motion through the gate; run tests before commit; keep installs and data on D:; parse every robot reply;
  log every clamp/reject; commit at the end of each task.
- **Ask MS first:** adding a dependency not listed in section 2; widening default limits in a YAML; anything that sends
  `EnableRobot`, `PowerOn` or `RequestControl` automatically; changing the locked UI style; pushing to GitHub.
- **Never:** write to C:; bypass or disable the gate (no "debug mode" that skips it); enable on connect; swallow exceptions;
  put the STOP behind a confirm dialog; commit `.venv`, datasets or `.research_tmp/`.

## 9. Success criteria per task

| Task | Done when |
|---|---|
| T1 core + driver base + fake robot + worker | `pytest` green; a script drives FakeDriver through the worker: enable, jog step, move_to, STOP clears the queue mid-move; hold-jog watchdog releases after 150 ms with no tick; a driver exception is logged and the loop keeps running. |
| T2 GUI shell + telemetry + 3D | `python -m botswain --robot robots/fake.yaml` opens the mockup layout; live pose/joint fields update at ~30 Hz from the fake robot; CR5 URDF renders and follows fake joints; status bar shows Hz; pytest-qt smoke tests green; MS looks at it and says the style matches. |
| T3 Dobot driver + fake controller + Doctor | DobotDriver passes integration tests against the fake controller (connect, state, enable, clear, stop, error codes, split/garbage packets, Not Tcp); app connects to `dobot_cr5_fake.yaml` and shows moving state; Doctor shows all checks green on the fake and a clear red with hint when a port is closed. |
| T4 jog + gripper + points | step jog and hold jog work on fake robot and fake controller; jog buttons disabled when not IDLE; Space/Esc STOP works with focus anywhere; joint and box clamps visible; gripper frames reach the fake controller with correct CRC; points save, list, go-to (through the gate), persist in JSON. |
| T5 cameras + phone teleop + gate | USB cams show in docks (zero cams also fine); phone opens the HTTPS page and drives the fake controller with hold-to-move; step clamp, teleport re-anchor, stale stop and box clamp have unit tests and visible counters. |
| T6 recording | an episode recorded against the fake controller loads back with `LeRobotDataset(root=...)`; frame count = duration x fps (+-1); `action` (commanded) and `observation.state` (measured) both present and differ when clamping happened; videos play. |

## 10. Monday lab checklist (2026-10-20)

Serious rules for the whole session: someone's hand on the physical e-stop during every motion step, nobody inside
the arm's reach, speed cap 10 % for the first motion, stop at the first surprise and write it down.

**Stage 0, at home before leaving**
- [ ] `pytest` green, app runs on `fake.yaml` and `dobot_cr5_fake.yaml`.
- [ ] Laptop Ethernet adapter name known; plan static IP 192.168.5.10 / 255.255.255.0.

**Stage 1, network only (no enable, no motion)**
- [ ] Set laptop static IP, cable to controller LAN1, `ping 192.168.5.1`.
- [ ] Note controller firmware version (DobotStudio Pro > About). >= 4.6.0 means `RequestControl()` exists.
- [ ] Robot in TCP/IP mode (DobotStudio Pro), then close DobotStudio (it can hold the control).
- [ ] Botswain Connect -> Doctor: ports, feedback ~125 Hz, mode 4 (disabled), no alarms, GetPose agrees with feedback.
- [ ] Also check port 30003 open (for the motion_port fallback).

**Stage 2, read-only telemetry**
- [ ] Joint angles and TCP pose in Botswain match the teach pendant / DobotStudio.
- [ ] 3D view pose matches the real arm. Move the arm by hand in drag mode **using the pendant**, watch it follow.
- [ ] Record the start pose; check the safety box in the YAML is around it (update the YAML if not).

**Stage 3, state buttons, still no motion**
- [ ] Enable -> mode 5. Disable -> mode 4. Enable again.
- [ ] Clear error with no alarm (should be harmless). STOP while idle (no error).

**Stage 4, first motion (10 % speed, e-stop hand ready)**
- [ ] Step jog X+ 1 mm: arm moves +X in the base frame (direction matches the 3D view). Repeat for each axis, both signs.
- [ ] Joint step jog J6 +1 deg, then J1.
- [ ] Hold jog X+ and release: stops on release. Hold again and press Space: stops.
- [ ] Watchdog test: hold jog, then freeze the GUI (debug menu "simulate GUI hang") -> stops within 150 ms.
- [ ] Unplug test only with e-stop ready and lowest speed: hold jog and pull the Ethernet cable; write down whether the controller stops on its own. If it does not, hold jog stays disabled until fixed.
- [ ] Box clamp: jog toward a box wall, arm stops at the wall, counter goes up.

**Stage 5, moves and points**
- [ ] Move to target 20 mm from current pose (MovL), then back. Then a joint move of 5 deg.
- [ ] Save point, move away, go to point.

**Stage 6, gripper**
- [ ] Check tool power + RS485 settings in DobotStudio (baud, slave id) vs our YAML.
- [ ] Gripper init, open, close; read the HTTP response in the log.

**Stage 7, phone teleop (if T5 is done)**
- [ ] Windows Firewall inbound rule for TCP 4443 (MS adds it).
- [ ] Oppo K13 on the same WiFi, open `https://<laptop-ip>:4443`, accept the cert, start AR.
- [ ] scale 0.3, rotation off, box +-150 mm, hold-to-move. Small moves first. Fast move: arm follows slower (clamp), never freezes.
- [ ] Note ServoP lag with `t=0.03`; try motion_port 30003 if lag is bad.

**Stage 8, recording (if T6 is done)**
- [ ] One 20 s episode with 1-2 cameras; reload it on the laptop.

## 11. Build notes (2026-10-17, what changed from the plan while building)

- Doctor: network checks run in their own thread; driver checks run on the worker through a `Diagnose` command,
  because the worker owns the sockets. Before Connect the Doctor opens its own short connections instead.
- Connect sets SpeedFactor to 10 % (or the cap if lower) so the first move after Enable is slow. It never enables.
- Dashboard replies are matched by the echoed command name: a late reply to a timed-out command is dropped.
- `stop()` raises only when the link is dead (then the hint says: use the physical e-stop).
- Feedback counts as lost after 300 ms without packets (review focus 1 asks for < 0.5 s).
- Teleop uses our own uvicorn server with the `teleop` package's phone page; `Teleop.run()` cannot be stopped and
  always uses the package's shared certificate. Own cert: put `cert.pem` + `key.pem` in `assets/certs/`.
- Teleport guard compares two consecutive input samples, not input vs last sent target (that would trigger
  whenever the step clamp makes the arm lag behind a fast hand).
- STOP during teleop latches the stream until the phone's hold button is released.
- Points: Go to is a joint move to the saved joints (same arm configuration). The gate checks joint limits only
  for joint targets; the box is checked for pose targets.
- Tool-frame steps are rotated with Rx/Ry/Rz as static XYZ Euler ('sxyz'). **Verify on the CR5.**

## 12. Open questions (need MS)

1. Controller firmware version on our CR5 (decides `RequestControl()` vs DobotStudio for TCP mode).
2. Home pose: which joint angles are safe in our cell? (YAML has a placeholder; Home button stays disabled until confirmed.)
3. Gripper model on the arm (I think DH-Robotics from the Modbus bytes; the AG/PGE register maps may differ).
4. Is there a second person for the Monday e-stop?
5. Licence for Botswain itself: MIT or Apache-2.0? (Apache-2.0 matches `teleop`; MIT matches Dobot files.)
6. Blockly and Script pages: placeholders only this weekend (assumed yes).
