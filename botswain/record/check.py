"""Check a recorded dataset: did every episode really record what we think it did?

    python -m botswain.record.check data/<dataset>

Reads the LeRobot v3 files directly (meta/info.json, data/*.parquet, meta/tasks.parquet) with pandas, so it
needs no torch. Per episode: frames, length, how often the robot state really changed, which input drove the
arm, how far the TCP travelled, the largest gap between commanded and measured position, and bad numbers.
Plus the three faults found in the first lab datasets: time warp (real time vs frames / fps), stale actions on
frames no input drives, and 360 deg angle jumps. Tips (idle at the ends, too few episodes) don't fail a check.
"""

from __future__ import annotations

import json
import re
import sys
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import pandas as pd

FPS_TOLERANCE = 0.12  # state updates may run this much off the dataset fps before it is a problem
TIME_TOLERANCE = 0.05  # the episode's real duration may differ this much from frames / fps
STALE_ACTION_MM = 5.0  # on a frame no input drives, the action must be (about) where the arm is
ANGLE_JUMP_DEG = 90.0  # no real motion turns this much in one frame: it is a +-180 wrap
IDLE_TIP_S = 3.0
EPISODES_TIP = 20  # ACT-style policies want about 50 demonstrations per task


class CheckError(Exception):
    """The dataset can't be checked at all (wrong folder, still open, broken files)."""


@dataclass
class EpisodeReport:
    index: int
    task: str
    frames: int
    seconds: float
    real_fps: float  # how often a NEW robot state arrived between frames (from t_state_ns)
    state_repeats: int  # frames whose state is the same sample as the frame before (feedback slower than fps)
    sources: dict[str, int]  # input name -> frames
    main_source: str  # the input that drove the arm most ("none" if nothing moved it)
    travel_mm: float  # measured TCP path length
    max_gap_mm: float  # largest |commanded - measured| TCP distance while an input drove the arm
    bad_values: int  # NaN / inf in a pose or joint value (gripper NaN = no gripper reading / command yet)
    real_seconds: float = 0.0  # first to last state arrival: what the frames really span
    idle_gap_mm: float = 0.0  # largest |action - measured| on frames no input drives (should be ~0)
    angle_jump_deg: float = 0.0  # largest frame-to-frame change of Rx Ry Rz (state or action)
    idle_start_s: float = 0.0
    idle_end_s: float = 0.0


@dataclass
class DatasetReport:
    root: Path
    fps: int
    cameras: list[str]
    episodes: list[EpisodeReport] = field(default_factory=list)
    problems: list[str] = field(default_factory=list)
    tips: list[str] = field(default_factory=list)


def _legend(info: dict) -> dict[int, str]:
    """{code: name} from the input_source feature's name, e.g. 'source (0 none, 1 gui, ...)'."""
    names = info["features"].get("observation.input_source", {}).get("names") or [""]
    return {int(code): name for code, name in re.findall(r"(\d+) (\w+)", names[0])}


def check_dataset(root: Path | str) -> DatasetReport:
    root = Path(root)
    if not (root / "meta" / "info.json").exists():
        found = sorted(p.name for p in root.parent.glob("*") if (p / "meta" / "info.json").exists())
        raise CheckError(f"{root} is not a dataset folder. Datasets in {root.parent}: {', '.join(found) or 'none'}")
    info = json.loads((root / "meta" / "info.json").read_text(encoding="utf-8"))
    fps = int(info["fps"])
    cameras = [k.removeprefix("observation.images.") for k in info["features"] if k.startswith("observation.images.")]
    report = DatasetReport(root, fps, cameras)
    files = sorted((root / "data").glob("chunk-*/file-*.parquet"))
    if not files:
        report.problems.append("no data files: was the dataset closed (Close dataset) after recording?")
        return report
    if not (root / "meta" / "episodes").exists():
        raise CheckError("the dataset is still open: press Close dataset in Botswain (Episodes tab), then check again. "
                         "LeRobot writes the end of its files only then.")
    try:
        data = pd.concat([pd.read_parquet(f) for f in files], ignore_index=True)
    except (OSError, ValueError) as e:
        raise CheckError(f"cannot read the data files ({e}). Was Botswain closed while recording?") from e
    tasks = pd.read_parquet(root / "meta" / "tasks.parquet")
    task_of = {int(i): str(t) for t, i in zip(tasks.index, tasks["task_index"], strict=True)}
    legend = _legend(info)
    if not legend:
        report.problems.append("no observation.input_source: recorded before input tagging existed")

    for ep, d in data.groupby("episode_index", sort=True):
        e = _episode(int(ep), d.sort_values("frame_index"), fps, legend, task_of)
        report.episodes.append(e)
        report.problems += [f"episode {e.index} ({e.task}): {p}" for p in _problems(e, fps, legend)]
        report.tips += [f"episode {e.index}: {t}" for t in _tips(e)]
    if len(report.episodes) < EPISODES_TIP:
        report.tips.append(f"{len(report.episodes)} episode(s): fine to check recording, but train on about 50 "
                           f"per task (and per input method)")
    return report


def _episode(ep: int, d: pd.DataFrame, fps: int, legend: dict[int, str], task_of: dict[int, str]) -> EpisodeReport:
    state = np.stack(d["observation.state"].to_numpy())
    action = np.stack(d["action"].to_numpy())
    t_state = d["observation.t_state_ns"].to_numpy()
    tcp = state[:, -7:-4]  # X, Y, Z (state = joints, X Y Z Rx Ry Rz, gripper)
    diffs = np.diff(t_state)
    fresh = diffs[diffs > 0]
    if "observation.input_source" in d:
        codes = d["observation.input_source"].to_numpy().astype(int)
    else:
        codes = np.zeros(len(d), dtype=int)
    sources: dict[str, int] = {}
    for code in codes:
        name = legend.get(int(code), f"code {code}")
        sources[name] = sources.get(name, 0) + 1
    driven = {k: v for k, v in sources.items() if k != "none"}
    driving = codes != 0
    gaps = np.linalg.norm(action[:, :3] - tcp, axis=1)
    rot = np.concatenate([state[:, -4:-1], action[:, 3:6]], axis=1)  # Rx Ry Rz of state and of action
    jumps = np.abs(np.diff(rot, axis=0))
    moved = np.flatnonzero(driving)
    return EpisodeReport(
        index=ep, task=task_of.get(int(d["task_index"].iloc[0]), "?"), frames=len(d), seconds=len(d) / fps,
        real_fps=float(1e9 / np.median(fresh)) if len(fresh) else 0.0,
        state_repeats=int(np.sum(diffs == 0)), sources=sources,
        main_source=max(driven, key=driven.get) if driven else "none",
        travel_mm=float(np.sum(np.linalg.norm(np.diff(tcp, axis=0), axis=1))),
        max_gap_mm=float(gaps[driving].max()) if driving.any() else 0.0,
        bad_values=int(np.sum(~np.isfinite(state[:, :-1])) + np.sum(~np.isfinite(action[:, :-1]))),
        real_seconds=float((t_state[-1] - t_state[0]) / 1e9) if len(d) > 1 else 0.0,
        idle_gap_mm=float(gaps[~driving].max()) if (~driving).any() else 0.0,
        angle_jump_deg=float(np.nanmax(jumps)) if len(jumps) else 0.0,
        idle_start_s=(moved[0] if len(moved) else len(d)) / fps,
        idle_end_s=(len(d) - 1 - moved[-1] if len(moved) else 0) / fps,
    )


def _problems(e: EpisodeReport, fps: int, legend: dict[int, str]) -> list[str]:
    out = []
    if e.bad_values:
        out.append(f"{e.bad_values} NaN/inf values in state or action")
    if e.frames > 1 and e.state_repeats > 0.1 * e.frames:
        out.append(f"robot state did not change in {e.state_repeats} of {e.frames} frames (feedback stalled?)")
    if e.real_fps and abs(e.real_fps - fps) > FPS_TOLERANCE * fps:
        out.append(f"state arrived at {e.real_fps:.1f} Hz, dataset says {fps} fps")
    if e.travel_mm < 1.0:
        out.append("the arm did not move (under 1 mm)")
    if legend and e.main_source == "none" and e.travel_mm >= 1.0:
        out.append("the arm moved but no input is tagged as driving it")
    span = (e.frames - 1) / fps
    if e.frames > fps and abs(e.real_seconds - span) > TIME_TOLERANCE * span:
        out.append(f"time warp: frames say {span:.1f} s but they really span {e.real_seconds:.1f} s "
                   f"(the recorder fell behind and skipped slots)")
    if e.idle_gap_mm > STALE_ACTION_MM:
        out.append(f"stale action: {e.idle_gap_mm:.0f} mm between action and arm on frames no input drives")
    if e.angle_jump_deg > ANGLE_JUMP_DEG:
        out.append(f"angle jump of {e.angle_jump_deg:.0f} deg between two frames (a +-180 wrap)")
    return out


def _tips(e: EpisodeReport) -> list[str]:
    out = []
    if e.idle_start_s > IDLE_TIP_S:
        out.append(f"{e.idle_start_s:.0f} s before the arm first moves: start recording when you are ready")
    if e.idle_end_s > IDLE_TIP_S:
        out.append(f"{e.idle_end_s:.0f} s after the last motion: save right after the task is done")
    return out


def format_report(r: DatasetReport) -> str:
    lines = [f"{r.root} | {r.fps} fps | cameras: {', '.join(r.cameras) or 'none'} | {len(r.episodes)} episode(s)",
             ""]
    lines.append(f"{'ep':>3}  {'frames':>6}  {'sec':>5}  {'state Hz':>8}  {'travel':>8}  {'max gap':>8}  "
                 f"{'driven by':<22}  task")
    for e in r.episodes:
        by = ", ".join(f"{k} {v}" for k, v in sorted(e.sources.items(), key=lambda kv: -kv[1]))
        lines.append(f"{e.index:>3}  {e.frames:>6}  {e.seconds:>5.1f}  {e.real_fps:>8.1f}  {e.travel_mm:>6.1f}mm  "
                     f"{e.max_gap_mm:>6.1f}mm  {by:<22}  {e.task}")
    lines += ["", "problems:" if r.problems else "no problems found"]
    lines += [f"  - {p}" for p in r.problems]
    if r.tips:
        lines += ["", "tips:"] + [f"  - {t}" for t in r.tips]
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    args = sys.argv[1:] if argv is None else argv
    if len(args) != 1:
        print("usage: python -m botswain.record.check data/<dataset>")
        return 2
    try:
        report = check_dataset(args[0])
    except CheckError as e:
        print(f"cannot check: {e}")
        return 2
    print(format_report(report))
    return 1 if report.problems else 0


if __name__ == "__main__":
    sys.exit(main())
