"""Teaching the safety box by moving the tool to its corners.

The box is the bounding box of the taught tool points. Two opposite corners give the whole box. If the corners
leave an axis flat (e.g. both at the same Y, a rectangle), that axis keeps the current box's extent; one more
corner further along it stretches the box.
"""

from __future__ import annotations

from collections.abc import Sequence

MIN_SPAN_MM = 20.0  # corners closer than this on an axis count as flat on that axis

Box = tuple[tuple[float, float], tuple[float, float], tuple[float, float]]


def box_from_corners(corners: Sequence[Sequence[float]], current: Box) -> Box:
    """corners: taught tool positions (x, y, z) in mm. current: the box in use, for flat axes."""
    if len(corners) < 2:
        raise ValueError("teach at least two corners")
    box = []
    for axis in range(3):
        values = [float(c[axis]) for c in corners]
        lo, hi = min(values), max(values)
        box.append(current[axis] if hi - lo < MIN_SPAN_MM else (lo, hi))
    return tuple(box)  # type: ignore[return-value]


def flat_axes(corners: Sequence[Sequence[float]]) -> list[str]:
    """Axes the corners do not span yet (they keep the current box's extent)."""
    if len(corners) < 2:
        return ["x", "y", "z"]
    return [name for axis, name in enumerate("xyz")
            if max(c[axis] for c in corners) - min(c[axis] for c in corners) < MIN_SPAN_MM]
