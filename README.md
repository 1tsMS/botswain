# Botswain

One desktop app to connect, check, jog, teleop and record datasets on cobots.
First robot: Dobot CR5 (V4 controller). Windows only for now. Work in progress.

## Install

Python 3.12.

```
py -3.12 -m venv .venv
.venv\Scripts\python -m pip install -e .[dev]
```

## Run

```
.venv\Scripts\python -m botswain --robot robots/fake.yaml
.venv\Scripts\python -m pytest -q
```

`robots/fake.yaml` is a pretend robot, no hardware needed. Each robot has one YAML in `robots/` with its
connection settings and safety limits. [SPEC.md](SPEC.md) explains how the pieces fit together.

## License

Apache-2.0. The CR5 model in `assets/dobot_cr5/` is MIT (Dobot).
