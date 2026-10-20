"""Safety box from taught corners: opposite corners give the box; a flat axis keeps the current extent."""

import pytest

from botswain.control.box_teach import box_from_corners, flat_axes

CURRENT = ((-600.0, -300.0), (-300.0, 0.0), (300.0, 600.0))


def test_two_opposite_corners_make_the_box():
    box = box_from_corners([(-550, -250, 350), (-400, -50, 520)], CURRENT)
    assert box == ((-550, -400), (-250, -50), (350, 520))
    assert flat_axes([(-550, -250, 350), (-400, -50, 520)]) == []


def test_flat_axis_keeps_the_current_extent_until_another_corner():
    # MS's case: top corner, then the same side lower and further back in X (Y unchanged): a rectangle.
    rect = [(-550, -120, 520), (-420, -120, 360)]
    assert flat_axes(rect) == ["y"]
    assert box_from_corners(rect, CURRENT) == ((-550, -420), (-300.0, 0.0), (360, 520))
    stretched = box_from_corners([*rect, (-500, -40, 400)], CURRENT)
    assert stretched[1] == (-120, -40)  # the third corner gives Y its own extent


def test_needs_two_corners():
    with pytest.raises(ValueError):
        box_from_corners([(0, 0, 0)], CURRENT)
