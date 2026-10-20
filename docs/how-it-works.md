# How Botswain works

A plain-language tour: what happens when you press a button, what each file does, how the fake robot and
the Dobot driver work, and what a recording contains.

- [The big picture](#the-big-picture)
- [What each file does](#what-each-file-does)
- [The fake robot](#the-fake-robot)
- [The Dobot driver](#the-dobot-driver)
- [Recording datasets](#recording-datasets)

---

## The big picture

Think of a restaurant kitchen.

| In the kitchen | In Botswain | File |
|---|---|---|
| The customer writes an order slip | the GUI turns a button press into a command ("jog X +5 mm") | `ui/` |
| One waiter is allowed in the kitchen | the **worker** thread takes commands from a queue, 125 times a second | `control/worker.py` |
| The head chef checks every slip | the **safety gate** allows, changes ("+50 mm leaves the box, make it +12") or refuses each command, with a reason in the log | `control/safety.py` |
| The cook speaks the robot's language | the **driver** turns the command into what this robot brand understands | `drivers/` |
| Someone shouts "stop!" | **STOP** skips the queue: queued commands are thrown away and the stop is sent at once | `control/worker.py` |

The robot's state (position, joints, mode) flows back the other way: driver → worker → screen, 30 times a
second, and to the recorder.

```
GUI buttons, jog keys ─┐
Move to target, points ├─> command queue ─> SAFETY GATE ─> DRIVER ─> robot
Phone (WebXR)  ────────┘    (worker thread)      │
STOP (Space/Esc) ── skips the queue              └─ allow / clamp / refuse, reason in the log

robot state ─> worker ─> screen (30 Hz), 3D view, recorder
```

Rules that never change:
- The GUI never talks to the robot directly. Only the worker does.
- Every motion command goes through the safety gate.
- Connect never enables the robot. Enable is its own button.
- Everything is timestamped with one clock, so recordings line up.

---

## What each file does

| File / folder | Job |
|---|---|
| `__main__.py`, `app.py` | Start the app: read the robot YAML, build driver, gate, worker and window |
| `core/types.py` | The shared vocabulary: Pose, Joints, robot Mode and every command (JogStep, MoveTo, Stop...) |
| `core/config.py` | Read `robots/*.yaml`; refuse anything missing or wrong, never fall back to wide defaults |
| `core/kinematics.py` | Math from the URDF: joint angles to tool pose (forward kinematics) and back (inverse kinematics) |
| `core/clock.py` | The single stopwatch every sample is stamped with |
| `control/worker.py` | The waiter: command queue, STOP lane, hold-jog watchdog, teleop streaming |
| `control/safety.py` | The head chef: speed cap, joint limits, safety box, teleop rules, refuses NaN |
| `control/doctor.py` | Health checks that never move the robot |
| `control/points.py` | Saved points, stored as JSON next to the robot YAML |
| `drivers/base.py` | The contract: the list of "buttons" every robot driver must have |
| `drivers/registry.py` | Maps the `driver:` name in the YAML to the driver class |
| `drivers/fake.py` | The fake robot |
| `drivers/dobot/` | The real Dobot driver (below) |
| `teleop/` | Phone server, phone motion to robot target, game controller (`gamepad.py`) |
| `control/program.py` | Runs Blocks programs: reads the saved blocks and sends each step through the worker |
| `sensors/cameras.py` | One background thread per camera, keeps the newest picture and its time |
| `record/` | Recording datasets |
| `ui/` | Everything on screen: window, toolbar, pages, 3D view, theme |
| `robots/` | One YAML per robot: address and safety limits |
| `assets/` | CR5 3D model (URDF + meshes) and fonts |
| `tests/` | Automatic checks, one file per module |

---

## The fake robot

`drivers/fake.py` pretends to be a CR5, entirely inside Python. No network, no hardware.

- It stores **6 joint angles**. That is its whole body.
- The tool position is always **calculated** from those angles with the CR5's URDF (the same model the 3D
  view shows), so everything it reports is physically consistent.
- **Joint jog / joint move:** it changes the angles a little every tick until they reach the target.
- **Straight-line move** (X..Rz jogs, Move to target, teleop): every 8 ms it takes a small step along the
  line, then solves which joint angles put the tool there (inverse kinematics).
- It behaves like the Dobot: starts disabled; Enable makes it idle; moving shows "moving"; an alarm puts it
  in error until Recover (or Clear error).

That makes it possible to test the GUI, the safety gate, teleop and recording at home.

---

## The Dobot driver

### There is no Dobot library in this project

Botswain uses **no Dobot SDK and no DLL**. The "controller" is the physical box next to the robot, running
Dobot's firmware. That box already listens on the network and understands **plain text commands over TCP**:

```
we send:     EnableRobot()
it replies:  0,{},EnableRobot();          0 means "accepted"
we send:     MovL(pose={-480,-140,470,180,0,90})
it replies:  0,{},MovL(pose={-480,-140,470,180,0,90});
```

So the driver is Python opening network connections to that box, on three ports:

| Port | What travels on it |
|---|---|
| 29999 | our commands and the controller's replies (text) |
| 30004 | the robot's state, pushed 125 times a second (1440-byte binary packets: joints, pose, mode...) |
| 22000 | a small web server in the controller, used for the gripper |

### Where the commands come from

From Dobot's **official TCP/IP protocol document (V4.6.6)**, shipped in Dobot's GitHub repository
`Dobot-Arm/TCP-IP-Python-V4`, plus reading that repository's Python code. Everything used is written down
in [RESEARCH_NOTES.md](../RESEARCH_NOTES.md), section 1: each command and its exact format, the error codes,
the robot mode numbers, and the byte offsets inside the 1440-byte packet.

Example from the document: continuous jog is `MoveJog(axis,coordtype=...)` where coordtype 0 means joints,
1 the user frame and 2 the tool frame; `MoveJog()` with nothing inside stops it.

The gripper is not in that document. The bytes Phone2Act sent, `[1,6,1,3,0,0,120,54]`, decode to a standard
Modbus "write one register" message for the DH-Robotics gripper (register 0x0103 = position), with a valid
checksum. The controller forwards those bytes to the gripper over the tool's RS485 port.

### The wrapper, layer by layer

`botswain/drivers/dobot/`:

| File | Job |
|---|---|
| `commands.py` | Builds the exact text. `servo_p(pose, ...)` gives `"ServoP(1.000000,...,t=0.030,...)"`. Values out of the documented range are refused before they become a command |
| `dashboard.py` | Sends a command on 29999 and waits for its reply. Turns error codes into words plus a fix hint. Matches each reply to its command, so a late reply is never mistaken for the next one |
| `feedback.py` | Reads the 125 Hz packets on 30004 and cuts them cleanly even when the network splits or joins them |
| `gripper.py` | Builds the Modbus bytes for the gripper and posts them to port 22000 |
| `driver.py` | **The wrapper.** Implements the common contract (`connect`, `enable`, `jog_step`, `move_to`, `stop`...) using the files above. `jog_step("X", 5)` sends `RelMovLUser(5,0,0,0,0,0,user=0)` |
| `fake_controller.py` | A pretend Dobot controller on your laptop that speaks the same protocol, to test the real driver without the robot |

Because every robot answers the same contract, adding another brand means writing one new driver file;
the GUI, safety gate, teleop and recorder stay the same.

---

## Game controller and Blocks

**Game controller** (`teleop/gamepad.py`): read 50 times a second from Windows' built-in XInput, no extra
library. While LB is held, stick deflection is a speed; the target pose is moved by speed x time, starting
from where the arm really is, and never more than 30 mm ahead of it. That target goes into the same teleop
stream as the phone, so the safety gate treats both the same way. Start / Back / X drive recording and
saved points, so a whole data-collection session can run from the pad.

**Blocks** (`ui/pages/blocks_page.py`, `control/program.py`): the Blockly editor (Google's drag-and-drop
block library, stored in `assets/blockly/`) runs in a web view inside the app. It only *builds* the program.
On Run, the program is read as JSON and a small Python interpreter walks it block by block: it submits each
step to the worker, waits for the move to finish, and stops at once on STOP, a refused command or a robot
error. Unknown blocks are refused instead of guessed at.

---

## Recording datasets

### How it runs

1. Start the cameras you want, Connect the robot, type a dataset name and a task.
2. **Start**: a recorder thread wakes up exactly 30 times a second and takes a snapshot (`record/recorder.py`).
3. **Save** writes the episode (video encoding takes a few seconds). **Discard** drops a bad take.
4. **Close dataset** finishes the files (`record/session.py`, `record/lerobot_writer.py`). Closing the app
   does this too.

### What each frame holds

| Field | Meaning |
|---|---|
| `observation.state` | **Measured**: 6 joint angles, tool pose (x, y, z, rx, ry, rz), gripper |
| `action` | **What was actually sent** to the robot, after the safety gate (pose + gripper) |
| `observation.command_raw` | What the operator asked for, **before** the gate. Differs from `action` when something was clamped |
| `observation.controller_target` | Where the robot's own controller was aiming |
| `observation.t_state_ns`, `observation.t_command_ns`, `observation.t_cam_<name>_ns` | The real arrival time of each of those, on one clock |
| `observation.images.<camera>` | The camera picture |
| `task` | The task text |

Having the commanded action, the measured state, the raw input and the real timestamps side by side makes it
possible to study the gap between what was commanded and what the robot did.

### What it exports

A **LeRobot dataset** (the format the Hugging Face LeRobot tools read) in `data/<dataset name>/`:

| Folder | Content |
|---|---|
| `meta/` | dataset description: fps, robot, list of fields, tasks, episodes, statistics |
| `data/` | tables (Parquet files) with every frame's numbers |
| `videos/` | one MP4 per camera (AV1) |

Load it in Python:

```python
from lerobot.datasets.lerobot_dataset import LeRobotDataset

ds = LeRobotDataset("botswain/<name>", root="data/<name>", video_backend="pyav")
frame = ds[0]   # dict with every field above; images as tensors
```

It can go straight into LeRobot training (ACT, Diffusion Policy, SmolVLA...). A test records from the running
fake robot, reloads the dataset, and checks the frame count, a decoded video frame and the arm's motion.
