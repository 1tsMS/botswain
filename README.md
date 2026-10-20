# Botswain

New here? Read [How Botswain works](docs/how-it-works.md) first.

A Windows desktop app to **connect, check, jog, teleoperate and record datasets** on collaborative robots.
First robot: Dobot CR5 (V4 controller). Built for teleop and imitation-learning research, open for other
robots: one driver file per brand.

| Works | Where it is tested |
|---|---|
| Connect, health check (Doctor), enable / disable, Recover, STOP | fake robot, fake Dobot controller |
| Jog: steps or hold, base / tool frame, joints | fake robot, fake Dobot controller |
| Live pose, joints and 3D model (CR5 URDF) | fake robot |
| Safety gate: speed cap, workspace box, joint limits, hold-jog watchdog | unit tests |
| Saved points, gripper (DH over RS485) | fake robot, fake Dobot controller |
| Game controller teleop (Xbox-style, hold LB to move) and recording from the pad | unit tests with a simulated pad |
| Blocks: drag-and-drop programs run through the safety gate | program runner tests on the fake robot |
| Phone teleop (WebXR, hold to move) | unit and server tests; on a phone: see [Phone teleop](#phone-teleop) |
| Cameras: any video source (webcam, DroidCam, OBS Virtual Camera) | this laptop |
| Dataset recording in LeRobot format (commanded **and** measured actions) | LeRobot round-trip test |

**Not yet tested on the real CR5.** Follow the lab checklist in [SPEC.md section 10](SPEC.md) the first time.

---

## Quick start

Needs Windows and Python 3.12.

```
py -3.12 -m venv .venv
.venv\Scripts\python -m pip install -e .[dev]
.venv\Scripts\python -m botswain
```

The first time it opens the **fake robot** (no hardware). Press **Connect**, then **Enable**, then jog. Pick the
real robot and its end effector on the **Settings** page; Botswain remembers the choice.

**From VS Code:** open the folder, go to Run and Debug (Ctrl+Shift+D), pick a configuration and press F5:

| Configuration | What it runs |
|---|---|
| Botswain: fake robot | the app with the built-in fake robot |
| Botswain + fake Dobot controller | a pretend Dobot controller and the app talking to it with the real driver |
| Botswain: lab CR5 (192.168.5.1) | the app on the real robot |

**Recording datasets** needs LeRobot (adds about 0.3 GB download, 1 GB on disk, CPU torch):

```
.venv\Scripts\python -m pip install -e .[record]
```

---

## Using it

### 1. Pick the robot and the end effector
**Settings** page: a **Robot** dropdown and an **End effector** dropdown, then **Save & restart** (only while
disconnected; every page is rebuilt for the new robot). **Load … config** adds a config file from anywhere.

Each robot and each end effector is a config file (YAML), its "ID card":

| Robot (`configs/robots/`) | Talks to |
|---|---|
| `fake.yaml` | a fake robot inside the app (real CR5 kinematics, a pretend DH gripper, no network) |
| `dobot_cr5_fake_controller.yaml` | the fake Dobot controller on this laptop (start it first, see [Development](#development)) |
| `dobot_cr5.yaml` | the lab CR5 at 192.168.5.1 (controller 3.5.x: protocol `dobot_v3`) |

| End effector (`configs/tools/`) | |
|---|---|
| `dh_pge_50_26.yaml` | DH-Robotics PGE-50-26 on the arm's RS485 tool port (driver `dh_modbus`) |

A robot config says which **protocol** talks to the arm, its address and named ports, its model file (URDF),
which **tool port** it offers, its limits and home. An end-effector config says which **driver** talks to it and
which kind of port it needs; Settings refuses a gripper the robot has no port for.

The right half of Settings edits **this lab's** values for the running robot: IP and ports, speed cap,
controller speed, and the **safety box**. They are saved to `<robot>.local.yaml` next to the config (the shared,
commented config file is never rewritten; delete the `.local.yaml` to go back).

**Teach the safety box:** move the tool to one corner of the table area (jog, drag or teleop), **Add corner**, then
the opposite corner, **Add corner**, set a margin, **Fit box**, **Apply & save box**. It applies at once. The box
limits the tool point, not the elbow.

From a terminal, `--robot <config>` and `--tool <config>` pick for one run without changing the saved choice.

### 2. Connect, check, enable
1. **Connect** opens the links and reads the state. It never enables the robot.
2. **Doctor** runs health checks without moving anything: ping, ports, feedback rate, TCP mode, robot mode,
   alarms, pose agreement, 3D model. Every failed check comes with a fix.
3. **Enable** powers the motors. **Disable** turns them off.
4. **Recover** gets a stuck robot back: it reconnects if the controller stopped answering (seen after the arm
   pressed into the table), stops drag, jog and the motion queue, clears the alarm and shows what is left.
   It never enables: press Enable when the cell is clear. It works even while the feedback is down.

The status (top right) says what is wrong in words: "alarm: collision", "controller not answering",
"feedback lost". Dobot controllers come in two command sets, picked in the robot config: `dobot_v3`
(controller 3.5.x, CR series, the lab CR5) and `dobot_v4` (4.x: CRA, Nova, E6). The Doctor shows which.

On a Dobot that answers "Not Tcp": switch it to TCP/IP mode in DobotStudio Pro, then close DobotStudio.
V4 controllers (4.6 and newer) can also use **Robot > Take control**; V3 has no such command.

### 3. STOP
**Space** or **Esc** from anywhere in the app, or the red STOP button. STOP clears every queued command,
ends any jog or teleop stream and sends the robot's stop. It never re-enables anything.
In text fields (dataset name, task) Space types a space; **Esc still stops**.

### 4. Jog
- **Step sizes** 0.5 / 1 / 5 / 10 / 50 mm (or degrees), or **Hold** to move while the key is held.
- **Frame:** Base or Tool for X..Rz. J1..J6 always move single joints.
- **Speed:** -25 / -5 / -1, slider, +1 / +5 / +25. Range 1..100 % (cap in the robot YAML).
- **Hold jog is safe by design:** the app sends a "still held" tick every 50 ms; if the app freezes or the
  key is released, the robot stops within 150 ms. It also stops early before a limit.

### 5. Move to a target
The pose and joint fields show the robot live and cannot be edited, so typing never moves the arm.
1. **Edit target** freezes the fields.
2. **Copy current** fills them with the current position. Change the numbers you want.
3. **Move to target** sends it: a straight line if you edited the pose, a joint move if you edited joints.

### 6. Safety gate
Every motion command passes the gate before the robot sees it.

| Rule | What it does |
|---|---|
| Ready check | motion only when enabled and idle (or jogging / streaming for its own commands) |
| Speed cap | speed above the YAML cap is lowered to the cap |
| Joint limits (always on) | steps stop at the limit; moves outside are refused |
| Safety box (toggle in the Robot view, off by default) | steps stop at the wall; moves outside are refused |
| Teleop | hold-to-move, stale input stops, jumps are ignored, fast moves are slowed (never dropped) |
| Bad numbers | NaN or infinity is always refused |

Every clamp and refusal is written to the log and counted in the status bar.

### 7. Saved points
**Save point** (toolbar) or **Save current pose** (Points page) stores the pose and joints in
`configs/robots/<robot>.points.json`.

The **Points** page is a table: Name, Move (joint / linear), Wait (s), X..Rz, J1..J6.
- Double-click a cell to edit it. Names have no spaces (Space is STOP).
- Numbers stay within **±10 mm / ±10°** of what was taught, and inside the joint limits: the editor will not take
  more, so many small edits can never walk a point away. Editing a joint recomputes the pose with the robot model
  (URDF); editing the pose recomputes the joints, and a pose the model cannot reach is refused.
- Drag a row by its **number** to reorder.
- Editing never moves the arm. **Go to** moves to the selected point (its own move kind); **Run all in order** visits
  every point, waits its Wait, and stops on STOP, a gate refusal or a robot error.
- The **Points** tab in the bottom dock is the quick list (Go to, Rename, Delete).

**Home** (toolbar, or the Jog page next to Frame) is a joint move to the robot config's `home.joints_deg`; it stays
greyed out until the config says `home: {confirmed: true}`.

**Gripper** (DH, Jog page): under Open / Close the gripper's own status (moving, at position, object caught,
object dropped) and its measured opening. **Close force** (20-100 %) is kept in the gripper and used by every close,
from the buttons, the gamepad or the phone. A DH PGE has no force sensor: it closes until its motor reaches that
force, then holds and reports *object caught*.

### 8. Robot view
3D model following the robot. Drag to orbit (it stays upright), scroll to zoom, **Reset view** to go back.
Chips: **Safety box** (shows and enforces the box), **Target**. The gripper drawn on the flange is a simple
stand-in: the CR5 URDF has no gripper.

### Teleop
One **Teleop** page for every input: pick **Phone**, **Game controller** or **VR headset** at the top. Each shows how
to connect on the left, and its mapping and controls on the right. Only one input moves the arm at a time (the one
that started first, until it lets go); the top right says which.

#### Phone
Needs an Android phone with ARCore, in Google Chrome, on the same WiFi as the laptop.

1. One time, in an admin PowerShell (Windows firewall):
   `New-NetFirewallRule -DisplayName "Botswain phone" -Direction Inbound -Protocol TCP -LocalPort 4443 -Action Allow -Profile Private`
2. In the app: Connect, Enable, **Teleop** page, **Start phone server**.
3. On the phone open the **full** address shown (it ends with `?k=...`, a pairing key that changes every
   start). Accept the certificate warning. Press **Start**, allow the camera.
4. Hold the move button and move the phone. Let go and the robot stops.

Start with motion scale 0.3 and rotation off. If Start does nothing, open the same address with `/check`
instead of `/?k=...`: it tells you what the phone is missing. **Make my own HTTPS certificate** replaces the
teleop package's shared certificate (needs Git for Windows).

#### Game controller
Any Xbox-style pad in **X-input mode** (Cosmic Byte and similar have an X-input / D-input switch).
Teleop > Game controller > **Use game controller**. The arm moves only while **LB** is held.

| Control | Does |
|---|---|
| hold **LB** | move (let go = stop) |
| left stick | X / Y |
| RT / LT | Z up / down |
| right stick | Rz / Ry |
| RB | slow mode (quarter speed) |
| A | gripper: open if closed, close if open |
| D-pad up / down | speed +/- 10 % |
| **Start** | recording: start an episode, or save the running one |
| **Back** | discard the running episode |
| X | save the current pose as a point |

The sticks set a speed; the target goes through the same teleop safety rules as the phone. "Phone forward is
robot" on the Mapping panel also turns the controller's axes.

#### VR headset (Meta Quest)
Teleop > VR headset > **Start VR server** (HTTPS port 4444). No lab network needed: turn on the Windows mobile hotspot,
join the Quest to it, open the https address in **Meta Quest Browser**, type the key, accept the certificate
warning, press **Enter**. Passthrough shows the real room (greyscale on Quest 2).

| Right controller | Does |
|---|---|
| squeeze grip | clutch: hold to move. Let go, move your hand, squeeze again to re-centre |
| trigger | gripper open / close |

The pose is the controller's grip (palm) pose as tracked by the headset, so the headset must be on and see the
controllers. Lost tracking, leaving VR or closing the page releases the clutch at once. Only one input moves the
arm at a time: while the headset streams, phone and gamepad targets are ignored (and the other way round).
Recordings store which input drives the arm at each frame in `observation.input_source`.

### Drag teach (hand-guiding)
**Drag mode** (toolbar, Robot group) puts an enabled, idle arm in hand-guiding: push it where you want, click
**End drag** to hold it there. STOP also ends drag. The arm's own drag button works too; Botswain sees the mode
either way. Before the first drag, check the payload (tool mass) in DobotStudio: a wrong payload makes the arm sag
or drift. Jogging and teleop are refused while dragging; the gripper still works (Jog page buttons, gamepad A).

Recording a drag episode: start drag, press Record, guide the arm, Save. Drag frames are tagged `drag`, and as
there is no command, their `action` is the **next** frame's measured pose: "go where the hand took the arm".

### Blocks
Drag-and-drop programs. Put blocks inside **when Run is pressed** and press **Run**. Categories:
- **Motion**: move to a saved point (joint or in a line), move relative, to a pose, to joint angles, home, speed.
- **Gripper**: open / close, close force; values *holds an object* and *opening %*.
- **Robot state**: the tool's X..Rz and J1..J6 now, as values (e.g. *if tool Z < 300*).
- **Logic**, **Loops** (repeat, while / until, count from-to), **Math**, **Variables**, **Wait & log**.

Distances, waits and speeds take values, so variables and math work there. The running block is highlighted.
Loops that never end are stopped after 100 000 rounds. There are no I/O blocks yet (the driver contract has no
digital I/O). **STOP** (or Space / Esc) aborts the program, and so does any command the safety gate refuses
or a robot error. Programs save as JSON in `programs/`. The editor never runs generated code: Botswain reads
the blocks and sends each step through the safety gate, like a button press.

### Cameras
**Cameras** page: **Scan** lists every video source Windows knows (USB webcams, DroidCam, OBS Virtual
Camera...), pick one or "All sources", **Show**. DroidCam and OBS show a picture only while they run.

### Recording datasets
1. Start the cameras you want and Connect the robot.
2. **Episodes** tab (bottom dock): dataset name and task ("pick the red cube").
3. **Start** (or **Record** in the toolbar), do the task, **Save** (or **Discard** a bad take). Repeat.
4. **Close dataset** when done (also happens when the app closes). Datasets are saved in `data/<name>/`.

Every frame (30 per second) holds:

| Key | Content |
|---|---|
| `observation.state` | measured joints, TCP pose, gripper |
| `observation.input_source` | which input drives the arm at that frame: 0 none (idle), 1 gui, 2 phone, 3 gamepad, 4 vr, 5 drag |
| `action` | the target actually sent, after the safety gate |
| `observation.command_raw` | the input before the gate (shows clamping) |
| `observation.controller_target` | the controller's own target |
| `observation.t_*_ns` | real arrival times of state, command and each camera frame |
| `observation.images.<camera>` | camera video |

Load it with `LeRobotDataset("botswain/<name>", root="data/<name>", video_backend="pyav")`.

---

## How it works

```
GUI buttons, jog keys ─┐
Move to target, points ├─> command queue ─> SAFETY GATE ─> DRIVER ─> robot (TCP/IP)
Phone (WebXR)  ────────┘    (worker thread)      │              │
STOP (Space/Esc) ── jumps the queue              │              └─ feedback thread (125 Hz) ─> state
                                                 └─ clamp / refuse + reason in the log
state ─> GUI (30 Hz), 3D view, recorder (with cameras, on one clock)
```

- One **worker thread** owns the robot driver. The GUI and every input only put commands in its queue.
- The **driver** hides each brand behind the same small set of buttons (connect, read state, enable, jog,
  move, stream, stop...) plus a capability card; the GUI hides what a robot can't do.
- The **fake robot** uses the URDF for forward and inverse kinematics, so it behaves like the real arm.

```
botswain/
  core/       types, clock, robot + end-effector config loading, kinematics, logging
  drivers/    arm driver contract, registry, fake robot, dobot/ (dialects V3/V4, dashboard, feedback,
              tool port, fake controller)
  tools/      end-effector drivers: contract, DH Modbus gripper, pretend DH bus
  control/    worker thread, safety gate, Doctor, saved points, Blocks program runner
  teleop/     phone server, phone-to-robot mapping, game controller
  sensors/    cameras
  record/     recorder, LeRobot writer, recording session
  ui/         main window, toolbar, pages, widgets, theme
configs/      robots/ and tools/: one config per robot and per end effector (+ .local.yaml lab edits, points)
assets/       CR5 URDF + meshes, fonts
tests/        one test file per module (pytest)
```

More detail: [docs/how-it-works.md](docs/how-it-works.md) (plain-language tour: fake robot, Dobot driver,
recording), [SPEC.md](SPEC.md) (design and decisions), [RESEARCH_NOTES.md](RESEARCH_NOTES.md) (protocol facts).

---

## Development

```
.venv\Scripts\python -m pytest -q
.venv\Scripts\python -m ruff check .
```

**Fake Dobot controller:** a pretend CR5 on this laptop that speaks the real protocol, so the real driver can
be tested at home:

```
.venv\Scripts\python -m botswain.drivers.dobot.fake_controller
```

Then pick **Dobot CR5 (fake controller)** in Settings (or run with
`--robot configs/robots/dobot_cr5_fake_controller.yaml`). It acts as a V3 controller like the lab CR5
(`--v4` for a V4 one). In its window, type a key and Enter: `p` pause feedback,
`a` alarm, `s` split packets, `g` garbage bytes, `t` leave TCP mode.

**Adding a robot:**
- Same protocol as one Botswain knows (any Dobot CR / CRA / Nova): copy a config in `configs/robots/`, change the
  name, address, URDF and limits. Nothing else.
- A new vendor: write one driver (`botswain/drivers/<brand>/`) implementing `botswain/drivers/base.py`, add one
  line to `PROTOCOLS` in `drivers/registry.py`, then a config. Every robot of that vendor is a config after that.
- If the arm carries end effectors, its driver returns a tool port from `tool_port()` (e.g. RS485 bytes).

**Adding an end effector:** same driver (another DH model): a config in `configs/tools/`. A new kind: a driver in
`botswain/tools/` implementing `botswain/tools/base.py`, one line in `TOOLS` (`botswain/tools/__init__.py`).

---

## Troubleshooting

| Problem | Fix |
|---|---|
| Connect fails: "Not Tcp" | TCP/IP mode in DobotStudio Pro (then close DobotStudio), or Robot > Take control (V4 4.6+) |
| Stuck after hitting something, "controller not answering" | **Recover**; if it still does not answer, power-cycle the controller |
| Hold-jog stops after a split second | the robot config's protocol does not match the controller (`dobot_v3` for 3.5.x) |
| Doctor: ports closed | cable, controller power, laptop on a static IP in 192.168.5.x |
| Jog keys grey | hover them: not connected, not enabled, in error, or busy |
| Phone page: 403 | open the full address with `?k=...` from the Teleop page (the key changes each start) |
| Phone: Start does nothing | open `/check`; use Chrome; install "Google Play Services for AR" |
| Phone server won't start | another program uses port 4443 |
| Camera tile black | DroidCam / OBS virtual camera not running, or the camera is used by another app |
| Controller "not connected" | plug it in, switch it to X-input mode (mode button / switch on the pad) |
| Record button grey | install LeRobot (`pip install -e .[record]`) and Connect |

Logs: `logs/botswain.log`.

## License

Apache-2.0. CR5 model in `assets/dobot_cr5/`: MIT (Dobot). IBM Plex fonts: OFL. Phone page: from the
`teleop` package (SpesRobotics, Apache-2.0).
Block editor: Blockly (Google, Apache-2.0), in `assets/blockly/`.
