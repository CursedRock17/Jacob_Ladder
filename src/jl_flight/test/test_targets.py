from __future__ import annotations

import math

import pytest

from jl_flight.core import VehicleState
from jl_flight.perception import CameraMount, OpticalTarget, ranging_to_optical

LEVEL = (1.0, 0.0, 0.0, 0.0)
YAW_90 = (math.cos(math.pi / 4), 0.0, 0.0, math.sin(math.pi / 4))


def at(position=(0.0, 0.0, -1.0), attitude=LEVEL, stamp=0.0):
    return VehicleState(position_ned=position, attitude=attitude, stamp=stamp)


def aruco(camera="front", max_age=0.5, offset=(0.0, 0.0, 0.0)):
    return OpticalTarget(CameraMount(*offset, facing=camera), max_age)


def yolo(camera_pitch_deg=0.0, max_age=0.5):
    """The YOLO pose node's detections: ranging frame, camera pitch on the mount."""
    return OpticalTarget(
        CameraMount(pitch_deg=camera_pitch_deg), max_age, frame="ranging"
    )


def test_a_front_tag_straight_ahead_is_in_front_of_the_drone():
    tag = aruco()
    tag.observe((0.0, 0.0, 3.0), at())
    assert tag.estimate(at()) == pytest.approx((3.0, 0.0, -1.0))


def test_front_camera_right_and_down_become_east_and_down():
    tag = aruco()
    tag.observe((0.5, 0.2, 3.0), at())
    assert tag.estimate(at()) == pytest.approx((3.0, 0.5, -0.8))


def test_the_drone_heading_rotates_the_detection():
    tag = aruco()
    tag.observe((0.0, 0.0, 3.0), at(attitude=YAW_90))
    assert tag.estimate(at()) == pytest.approx((0.0, 3.0, -1.0))


def test_down_camera_axes():
    tag = aruco(camera="down")
    tag.observe((0.5, 0.2, 2.0), at())
    assert tag.estimate(at()) == pytest.approx((-0.2, 0.5, 1.0))


def test_a_detection_counts_for_max_age_seconds():
    tag = aruco()
    tag.observe((0.0, 0.0, 3.0), at(stamp=10.0))
    assert tag.estimate(at(stamp=10.4)) is not None
    assert tag.estimate(at(stamp=10.6)) is None


def test_the_estimate_keeps_where_the_tag_was_when_seen():
    tag = aruco()
    tag.observe((0.0, 0.0, 3.0), at(position=(0.0, 0.0, -1.0)))
    moved = at(position=(1.0, 0.0, -1.0), stamp=0.1)
    assert tag.estimate(moved) == pytest.approx((3.0, 0.0, -1.0))


def test_bad_detections_and_unknown_attitude_are_ignored():
    tag = aruco()
    tag.observe((math.nan, 0.0, 3.0), at())
    tag.observe((0.0, 0.0, 3.0), at(attitude=(0.0, 0.0, 0.0, 0.0)))
    tag.observe((0.0, 0.0, 3.0), at(position=(math.nan, 0.0, -1.0)))
    assert tag.estimate(at()) is None


def test_reset_forgets_the_last_detection():
    tag = aruco()
    tag.observe((0.0, 0.0, 3.0), at())
    tag.reset()
    assert tag.estimate(at()) is None


def test_an_unknown_facing_is_rejected():
    with pytest.raises(ValueError, match="facing must be one of"):
        CameraMount(facing="side")


def test_max_age_must_be_positive():
    with pytest.raises(ValueError, match="max_age"):
        aruco(max_age=0)


def test_the_cameras_lever_arm_is_part_of_the_estimate():
    # The camera is 0.13 m ahead of and 0.05 m above the vehicle origin, and sees
    # the tag 3 m ahead: the tag is 3.13 m ahead of the origin.
    tag = aruco(offset=(0.13, 0.0, -0.05))
    tag.observe((0.0, 0.0, 3.0), at())
    assert tag.estimate(at()) == pytest.approx((3.13, 0.0, -1.05))


def test_the_lever_arm_turns_with_the_vehicle():
    tag = aruco(offset=(0.13, 0.0, 0.0))
    tag.observe((0.0, 0.0, 3.0), at(attitude=YAW_90))  # facing east
    assert tag.estimate(at()) == pytest.approx((0.0, 3.13, -1.0))


def test_yolo_ranging_frame():
    drogue = yolo()
    drogue.observe((0.5, 0.2, 3.0), at())  # 0.5 m left, 0.2 m up, 3 m ahead
    assert drogue.estimate(at()) == pytest.approx((3.0, -0.5, -1.2))


def test_yolo_camera_pitch_tilts_the_ray_down():
    drogue = yolo(camera_pitch_deg=30.0)
    drogue.observe((0.0, 0.0, 2.0), at())
    assert drogue.estimate(at()) == pytest.approx(
        (2.0 * math.cos(math.radians(30)), 0.0, -1.0 + 2.0 * math.sin(math.radians(30)))
    )


def test_ranging_is_optical_with_x_and_y_flipped():
    assert ranging_to_optical((0.5, 0.2, 3.0)) == (-0.5, -0.2, 3.0)


def test_pitch_applies_to_optical_detections_too():
    tag = OpticalTarget(CameraMount(pitch_deg=30.0), 0.5)
    tag.observe((0.0, 0.0, 2.0), at())
    assert tag.estimate(at()) == pytest.approx(
        (2.0 * math.cos(math.radians(30)), 0.0, -1.0 + 2.0 * math.sin(math.radians(30)))
    )


def test_pitch_is_for_front_cameras_only():
    with pytest.raises(ValueError, match="front cameras only"):
        CameraMount(facing="down", pitch_deg=10.0)


def test_an_unknown_frame_is_rejected():
    with pytest.raises(ValueError, match="frame must be one of"):
        OpticalTarget(CameraMount(), 0.5, frame="sideways")
