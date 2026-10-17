# Botswain

One desktop app to connect, check, jog, teleop and record datasets on cobots.
First robot: Dobot CR5 (V4 controller). Windows only for now. Work in progress.

## Install

Python 3.12.

```
py -3.12 -m venv .venv
.venv\Scripts\python -m pip install -e .[dev]
```

Recording datasets needs LeRobot, which pulls in torch (large): `pip install -e .[record]`.

## Run

```
.venv\Scripts\python -m botswain --robot robots/fake.yaml
```

| Robot file | What it talks to |
|---|---|
| `robots/fake.yaml` | a pretend robot inside the app, no network |
| `robots/dobot_cr5_fake.yaml` | the real Dobot driver against a pretend controller on this laptop |
| `robots/dobot_cr5.yaml` | the lab CR5 at 192.168.5.1 |

For `dobot_cr5_fake.yaml`, start the pretend controller first in another terminal:

```
.venv\Scripts\python -m botswain.drivers.dobot.fake_controller
```

It listens on 29999 / 30003 / 30004 / 22000 and takes keys for testing: `p` pause feedback, `a` alarm,
`s` split packets, `g` garbage bytes, `t` leave TCP mode.

Each robot YAML holds the connection and the safety limits (workspace box, joint limits, speed cap).
Every motion command goes through the safety gate; Space or Esc stops from anywhere in the app.
Connect never enables the robot. [SPEC.md](SPEC.md) explains how the pieces fit together.

## Tests

```
.venv\Scripts\python -m pytest -q
.venv\Scripts\python -m ruff check .
```

## License

Apache-2.0. The CR5 model in `assets/dobot_cr5/` is MIT (Dobot). IBM Plex fonts are OFL.
The phone page comes from the `teleop` package (SpesRobotics, Apache-2.0).
