"""Check a recorded dataset: did every episode really record what we think it did?

    python -m botswain.record.check data/<dataset>

Reads the LeRobot v3 files directly (meta/info.json, data/*.parquet, meta/tasks.parquet) with pandas, so it
needs no torch. Per episode: frames, length, how often the robot state really changed, which input drove the
arm, how far the TCP travelled, the largest gap between commanded and measured position, and bad numbers.
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


@dataclass
class DatasetReport:
    root: Path
    fps: int
    cameras: list[str]
    episodes: list[EpisodeReport] = field(default_factory=list)
    problems: list[str] = field(default_factory=list)


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
    return EpisodeReport(
        index=ep, task=task_of.get(int(d["task_index"].iloc[0]), "?"), frames=len(d), seconds=len(d) / fps,
        real_fps=float(1e9 / np.median(fresh)) if len(fresh) else 0.0,
        state_repeats=int(np.sum(diffs == 0)), sources=sources,
        main_source=max(driven, key=driven.get) if driven else "none",
        travel_mm=float(np.sum(np.linalg.norm(np.diff(tcp, axis=0), axis=1))),
        max_gap_mm=float(gaps[driving].max()) if driving.any() else 0.0,
        bad_values=int(np.sum(~np.isfinite(state[:, :-1])) + np.sum(~np.isfinite(action[:, :-1]))),
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
