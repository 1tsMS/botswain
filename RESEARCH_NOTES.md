# Botswain research notes

Code-reading pass, 2026-10-17. Read from source and GitHub issues, nothing run on hardware yet.
Each item says where it came from. "Verify" means we still have to check it on the real robot.

---

## 1. Dobot V4 TCP/IP (our CR5)

Sources: `Dobot-Arm/TCP-IP-Python-V4` (MIT, `dobot_sdk/` package, 2026 rewrite) and the official
protocol doc shipped inside it (`assets/手臂二开md文档/`, doc version V4.6.6, 2026-04-10).

### Ports

| Port | What | Notes |
|---|---|---|
| 29999 | Dashboard: every command (enable, clear, jog, MovJ, ServoP, IO, queries) | Request/reply, ASCII. Official V4 doc lists only this one for commands. |
| 30003 | Old "motion" port | NOT in the V4 doc, and the SDK's connection class refuses it (`port not in [29999, 30004, 30005, 30006]`). But Phone2Act sent ServoP to 30003 on our CR5 and it moved, so our firmware still accepts it. |
| 30004 | Real-time feedback, 1440-byte packet every 8 ms (125 Hz) | Push only, no requests. |
| 30005 / 30006 | Same packet at 200 ms / 1000 ms | Not needed. |
| 22000 | Controller HTTP server | `GET /protocol/getAlarm` (alarm text), `POST /interface/toolDataExchange` (tool RS485 bytes, used for the gripper). |

**Decision:** send everything on 29999. Keep `motion_port` as a config option (default 29999, can set 30003)
in case ServoP over 29999 behaves badly. Doctor checks both ports.

### Message format

- Send: `Name(p1,p2,...)` + `\n`. Not case sensitive.
- Reply: `ErrorID,{values},Name(p1,...);` Example: `0,{},MovL(-500,100,200,150,0,90);`
- `ErrorID` 0 = accepted (not "finished"!). Main codes: -1 failed, -2 robot in alarm, -3 e-stop,
  -4 powered off, -5 script running, -6 MoveJog axis/coordtype mismatch, -7 script paused,
  -10000 unknown command, -20000 wrong number of params, -3000X / -4000X / -5000X / -6000X type or range error on param X.
- If the robot is not in TCP mode, the reply contains `Not Tcp` (seen in issue #6 user code). Doctor should look for this.
- Queued commands (MovJ, MovL, ServoP, ServoJ, RelMov*) wait in the motion queue; immediate commands
  (EnableRobot, ClearError, Stop, MoveJog, RobotMode, GetPose) run at once.

### Commands we need (exact strings, from `dobot_sdk/api/*.py`)

| Button | String | Type |
|---|---|---|
| Take control (fw >= 4.6.0) | `RequestControl()` | immediate. Only allowed when powered off or disabled. Replaces the DobotStudio Pro mode switch on new firmware. **Verify our firmware version.** |
| Power on | `PowerOn()` | ~10 s before enable is allowed |
| Enable | `EnableRobot()` or `EnableRobot(load)` or `EnableRobot(load,cx,cy,cz)` | immediate |
| Disable | `DisableRobot()` | immediate |
| Clear error | `ClearError()` | immediate. Some alarms need the cause fixed or a reboot. Check `RobotMode()` after. |
| Stop | `Stop()` | immediate. Stops queue, jog, scripts. |
| E-stop (software) | `EmergencyStop(1)` / `EmergencyStop(0)` | disables + alarms. We do NOT map our STOP button to this (old app bug: e-stop that re-enables). |
| Mode | `RobotMode()` -> `0,{5},RobotMode();` | see table below |
| Errors | `GetErrorID()` -> `0,{[[id,...],...]},GetErrorID();` | text via HTTP 22000 |
| Pose | `GetPose()` / `GetAngle()` | backup if 30004 fails |
| Speed | `SpeedFactor(1..100)` | global %, does NOT affect ServoP/ServoJ |
| Joint move | `MovJ(joint={j1,...,j6},v=20)` or `MovJ(pose={x,y,z,rx,ry,rz})` | queued; returns at once |
| Line move | `MovL(pose={x,y,z,rx,ry,rz},v=20)` | queued |
| Step jog (tool/user frame) | `RelMovLUser(1,0,0,0,0,0,user=0,v=20)` (mm, deg), `RelMovLTool(...)` same shape, `RelJointMovJ(d1,...,d6,v=20)` | queued |
| Continuous jog | `MoveJog(X+,coordtype=1,user=0,tool=0)` ... `MoveJog()` to stop | immediate. **Keeps moving until `MoveJog()` arrives.** Joint axes `J1+`.. need coordtype 0; Cartesian axes need 1 (user) or 2 (tool), 0 gives -6. |
| Stream (teleop) | `ServoP(x,y,z,rx,ry,rz,t=0.03,aheadtime=50,gain=500)` | queued. Doc: call at ~33 Hz (every 30 ms). `t` 0.004..3600 s, aheadtime 20..100 (like D), gain 200..1000 (like P). Not limited by SpeedFactor, only by the robot's own speed limit. |
| Tool power / RS485 | `SetToolPower(1)`, `SetTool485(baud,parity,stop)`, `SetToolMode(...)` | for gripper setup |

Units: mm and degrees. Rx/Ry/Rz are degrees in the current user/tool frame (default 0/0).

### RobotMode values (doc V4.6.6)

1 init, 2 brake open, 3 powered off, 4 disabled, 5 enabled + idle, 6 drag mode, 7 running (queue),
8 single move (jog, RunTo), 9 **error (highest priority, always 9 if any alarm)**, 10 paused, 11 **collision**.
SDK issue #11: the SDK enum calls 11 "JOG", the doc says collision. Trust the doc: treat 11 as a fault.

### Feedback packet (30004), little-endian, 1440 bytes

Offsets computed from the SDK's numpy dtype (`dobot_sdk/protocol/data_types.py`, itemsize checked = 1440):

| Field | Offset | Type |
|---|---|---|
| len | 0 | uint16 (= 1440) |
| DigitalInputs / Outputs | 8 / 16 | uint64 |
| RobotMode | 24 | uint64 |
| TimeStamp | 32 | uint64 (ms) |
| TestValue | 48 | uint64, must be `0x0123456789ABCDEF` |
| SpeedScaling | 64 | float64 |
| QTarget (6 joints, deg) | 192 | 6 x float64 |
| QActual (6 joints, deg) | 432 | 6 x float64 |
| QDActual | 480 | 6 x float64 |
| TCPSpeedActual | 672 | 6 x float64 |
| ToolVectorActual (x,y,z,rx,ry,rz) | 624 | 6 x float64 |
| ToolVectorTarget | 768 | 6 x float64 |
| User / Tool index | 1012 / 1013 | int8 |
| EnableStatus, DragStatus, RunningStatus, ErrorStatus, JogStatusCR | 1026..1030 | int8 each |
| CollisionState | 1038 | int8 |
| CurrentCommandId | 1112 | uint64 |
| ActualQuaternion (w,x,y,z) | 1384 | 4 x float64 |
| SafetyState | 1420 | int8 |

`QTarget` and `ToolVectorTarget` are what the controller is aiming at. Useful for P2: we can log
our command, the controller's target and the measured value.

### Bugs and gotchas found

1. **Feedback framing bug in the SDK.** `_feedback_loop` calls `recv(144000)`, and if it got more than
   1440 bytes it throws that data away and reads again; then it takes the first 1440 bytes of whatever came.
   TCP does not keep packet borders, so this can parse a packet that starts mid-way (TestValue check then
   drops it, so you lose frames quietly). **Our reader: keep a byte buffer, cut exact 1440-byte frames,
   check `len == 1440` and TestValue, and resync by scanning for TestValue at offset 48 if it fails.
   Always use the newest frame, drop old ones.**
2. Dashboard socket is request/reply. One command at a time, wait for the `;` ending the reply.
   The SDK uses a lock around send+recv; we do the same (only the worker thread talks to it anyway).
   `recv(1024)` once is not enough for long replies; read until `;`.
3. ErrorID 0 means "accepted", not "done". To know a queued move finished, watch RobotMode go back to 5
   (or CurrentCommandId). Issue #5: people waiting on command IDs got timeouts.
4. MoveJog runs until told to stop. If our app freezes or the network drops mid-jog, the arm keeps going.
   Fix: hold-to-jog sends `MoveJog(...)` on press and `MoveJog()` on release, plus a watchdog in the worker
   that sends `MoveJog()` if no "still held" tick for 150 ms, plus `Stop()` on disconnect.
   Prefer step jog (RelMovL*) as the default mode, continuous jog only in "Hold" mode.
5. ServoP is queued with `t=0.1` default. Phone2Act sends it without `t`, so each point plans for 100 ms while
   points arrive every ~22 ms: the queue can fill and lag builds up. Set `t` close to our send period
   (e.g. 0.03 s at 33 Hz) and **verify on the robot** what happens to lag.
6. `Stop()` also clears the ServoP queue. Use it on clutch release / teleop stop.
7. TCP mode: needs DobotStudio Pro (Windows) on older firmware, or `RequestControl()` on fw >= 4.6.0.
   Issue #4/#6/#9: "does nothing" reports were almost always "not in TCP mode".
8. Issue #10: SDK `InverseKin` uses a wrong param name (`JointNear`). We don't use IK from the controller yet.
9. Issue #5: on a Nova 5, MovJ always went to x = 0 for one user. Unresolved. Watch for it in the lab
   (Move to target must be a small test move first).
10. Old app lesson repeated here: don't treat mode 7 or 8 as idle. Idle = 5 only.

---

## 2. Phone2Act (what worked on our CR5, 2026-10-15)

Source: `OmMandhane/Phone2Act` `dobot_hardware_bridge.py` + our run notes.

- Motion: raw socket to `:30003`, `TCP_NODELAY`, timeout 10 ms, sends `ServoP(x,y,z,rx,ry,rz)\r\n`
  (3 decimals, no `t`). Before each send it does one `recv(1024)` to throw away old replies. Replies are
  never parsed, so errors are invisible. **We parse every reply.**
- Orientation: phone quaternion -> `euler_from_quaternion` (ROS default 'sxyz') -> degrees -> Rx,Ry,Rz.
  This matched the CR5 well enough in the run (axis directions were right). Keep the same convention and
  test it in the Doctor with a pose read-back.
- Enable: bridge calls ClearError then EnableRobot on start (it ENABLES the robot as soon as it runs).
  **We never enable on connect; enable is its own button.**
- **Gripper bytes decoded.** The HTTP body `{"value": [1,6,1,3,0,0,120,54]}` is a raw Modbus RTU frame:
  slave 1, function 06 (write one register), register 0x0103, value 0, CRC 0x3678 (CRC checked by us).
  Open = value 0x03E8 (1000). That is the DH-Robotics gripper "position" register (0 = closed, 1000 = open,
  in 0.1 %). DH grippers need an **init command first** after power-up: register 0x0100 = 1,
  frame `[1,6,1,0,0,1,73,246]` (CRC computed by us; register from a UFactory guide for the DH AG-160-95).
  **Likely reason our gripper did nothing: no init, or tool power/RS485 baud (DH default 115200) not set.**
  Verify in the lab: `SetToolPower(1)`, `SetTool485(115200,"N",1)`, send init, then position, and LOG the HTTP response.
- Planner lessons (from our run): drop-on-jump froze the arm -> we clamp. First target must be seeded from
  the robot pose on clutch-in. Clutch must be hold-to-move, not a toggle.

---

## 3. SpesRobotics `teleop` (phone WebXR)

Source: `SpesRobotics/teleop` v0.1.5 (Apache-2.0, pushed 2026-09). Pure Python: FastAPI + uvicorn + transforms3d.

- **HTTPS:** uvicorn with `ssl_keyfile`/`ssl_certfile` pointing at `key.pem`/`cert.pem` **shipped inside
  the package** (same self-signed cert for everyone). Phone opens `https://<laptop-ip>:4443`, taps through
  the warning, WebXR works (Chrome treats it as a secure context after that). Issue #13: iPhone Firefox
  refuses self-signed. For us: Android Chrome only. Better: we generate our own cert per laptop on first
  run (the shared key is public, fine for a lab LAN but not great).
- **Phone needs ARCore** (Google Play Services for AR). Issue #6: "NotSupportedError" = no ARCore or no
  HTTPS. Same device list as Phone2Act: our Oppo K13 5G is on it, the Redmi/OnePlus are not.
- **Messages:** websocket `/ws`, JSON `{"type":"pose","data":{...}}`. `data` has
  `position{x,y,z}` (m), `orientation{x,y,z,w}`, `move` (bool, the "hold to move" button), `gripper`
  ('open'/'close'), `scale` (0.2..1.0), `reservedButtonA/B`, `fps`, `device`.
- **Rate:** sent from the XR frame loop when more than 10 ms passed since the last send, so roughly the
  phone's AR frame rate (30-60 Hz). No filtering.
- **Mapping:** WebXR frame is RUB (right, up, back). It converts to FLU with `TF_RUB2FLU`, applies a
  "natural phone pose" (default pitch -45°), then: on `move` going true it stores phone start + robot
  start; target = robot start + (phone now - phone start) * scale. Orientation = relative rotation applied
  to the start orientation. `move` false -> resets.
- **Jump guard:** if the new phone pose is more than 5 cm or 35° from the last one, it logs "Pose jump",
  resets the start pose and skips that message. That's re-anchoring, not freezing. Good, keep that idea.
- Has `transform_limiter.py`: velocity + acceleration clamp on SE(3) twists. Same idea as our safety-gate clamp.
- **Windows:** README says CI-tested on Windows. Gotchas: `get_local_ip()` uses a UDP connect to 8.8.8.8, so on
  a hotspot with no internet it prints an error (issue #22). Windows Firewall must allow inbound TCP 4443
  (MS adds the rule himself, it's a system setting). `Teleop.run()` blocks; run it in a thread.
- **How we use it:** import the package for the server + phone page, subscribe a callback, and turn the
  callback into our own `TargetPose` message. Phone -> safety gate -> driver. Our own relative mapping
  (so the scale and axis remap are ours and testable), using their message format.

---

## 4. LeRobot plugin templates (Spes)

Sources: `SpesRobotics/lerobot-robot-xarm` 0.0.4, `SpesRobotics/lerobot-teleoperator-teleop`.

- Plugin = pip package named `lerobot_robot_<x>` with a `@RobotConfig.register_subclass("lerobot_robot_xarm")`
  dataclass config + a `Robot` subclass with `connect / disconnect / get_observation / send_action /
  observation_features / action_features / is_connected / calibrate / configure`.
- xArm plugin: actions are joint positions (`joint1.pos`..), poses get turned into joints with a Jacobian IK
  helper. It hardcodes the IP inside `connect()` and ignores `config.ip` (bug). Gripper state is the
  last command, not measured (same flaw Phone2Act has).
- Teleoperator: gives `pose_from_initial` (4x4) + `gripper` + `home`; `move` false -> no pose in the action.
- **For Botswain:** we are not a LeRobot plugin first. Our driver contract is ours. Later (after T6) a thin
  `lerobot_robot_botswain` adapter can wrap any Botswain driver so `lerobot-record` works too. Put it on the
  "later" list.

---

## 5. LeRobot dataset writer

Source: `huggingface/lerobot` tag v0.6.1 (2026-08-03): `datasets/lerobot_dataset.py`, `dataset_writer.py`,
`video_utils.py`, `configs/video.py`, `pyproject.toml`.

- API:
  ```python
  ds = LeRobotDataset.create(repo_id="ms/pick_cube", fps=30, features=FEATURES,
                             root="D:/data/pick_cube", robot_type="dobot_cr5", use_videos=True)
  ds.add_frame({"observation.state": np.float32[...], "action": ..., "observation.images.front": HxWx3 uint8,
                "task": "pick the cube"})   # per frame
  ds.save_episode()                       # per episode (encodes video)
  ds.clear_episode_buffer()               # discard a bad episode
  ds.finalize()                           # MUST call at the end, or parquet footers are missing -> broken dataset
  ```
- Features dict: `{"name": {"dtype": "float32", "shape": (n,), "names": [...]}}`, images `{"dtype": "video",
  "shape": (h, w, 3), "names": ["height","width","channels"]}`.
- **Big gotcha: `timestamp = frame_index / fps`.** LeRobot computes timestamps, you can't pass your own.
  So the recorder must sample on a fixed grid (one clock), and we store the real capture times as extra
  float features (`observation.t_robot`, `observation.t_cam_front`, ...) so P2 can check the delay.
- Requirements: **Python >= 3.12** (laptop has 3.12.3, ok), `torch>=2.7` (hard dependency, big install),
  `numpy>=2`, `av` 15.x (PyAV, bundles FFmpeg), `torchcodec` on Windows only with torch >= 2.8 (decode only).
- Encoding uses PyAV. Default codec `libsvtav1`, `crf 30`, `g 2`. **Verify** `libsvtav1` is in the Windows
  PyAV wheel (`"libsvtav1" in av.codecs_available`); if not, set `vcodec="h264"`.
- `streaming_encoding=True` encodes while recording (no PNG temp files). Try it, fall back to default.
- Decision: install LeRobot in its own venv on D: only when we reach T6. Before that, the recorder writes
  our own raw log (see SPEC) so T1-T5 never depend on torch.

---

## 6. CR5 3D model

Source: `Dobot-Arm/DOBOT_6Axis_ROS2_V4` (MIT).

- No xacro needed. `cr5_moveit/config/cr5_robot.urdf.xacro` only includes `dobot_rviz/urdf/cr5_robot.urdf`,
  which is a **plain URDF** (0 xacro tags). Meshes: `dobot_rviz/meshes/cr5/{base_link,J1..J6}.STL` (~2 MB total),
  referenced as `package://dobot_rviz/meshes/cr5/...`. We copy URDF + STLs into `assets/robots/dobot_cr5/`
  and rewrite the paths to relative `meshes/...` (keep the MIT notice).
- Joints `joint1..joint6`: J1 origin z 0.147, J3 offset -0.427, J4 -0.357 (z 0.141), J5 y -0.116, J6 y 0.105.
  URDF limits: ±6.28 rad except J3 ±2.792 rad (±160°). These are loose; real limits go in our YAML.
- Load with `yourdfpy.URDF.load(path, filename_handler=...)`, get link transforms from joint angles, show
  meshes with PyVista inside Qt via `pyvistaqt.QtInteractor`. `pip xacro` only needed for Addverb Syncro later.
- Verify: the URDF zero pose matches the controller's joint zero (compare FK of `QActual` with
  `ToolVectorActual` in the Doctor; flange offset may differ by the tool).

---

## 7. Other unified APIs (ideas only)

- `armctl` (MIT): 7 abstract methods (`move_joints`, `get_joint_positions`, `move_cartesian`,
  `get_cartesian_position`, `stop_motion`, `get_robot_state`, `sleep`), all blocking, no streaming, no
  capability info. Too small for teleop. Take-away: keep our contract small too, but add streaming and a capability card.
- RIO (CMU): node + middleware, Ubuntu only. RoboDK drivers: tiny text protocol per robot. Both say the
  same thing: isolate each driver behind a tiny contract.

## Still to verify on the real robot (goes into the Monday checklist)

- [ ] Controller firmware version (>= 4.6.0 means `RequestControl()` works).
- [ ] ServoP over 29999 vs 30003: both accepted? Which has less lag?
- [ ] Best `t` for ServoP at our rate.
- [ ] Gripper: tool power, RS485 115200, init frame, HTTP response body.
- [ ] Euler convention of Rx/Ry/Rz vs our quaternion conversion (read back pose).
- [ ] URDF FK vs controller TCP pose.
- [ ] Windows Firewall rule for 4443 (phone) and PyAV `libsvtav1`.
