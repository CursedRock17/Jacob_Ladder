from __future__ import annotations

import math

import pytest

from jl_blocks import library  # noqa: F401  (registers the shipped blocks)
from jl_blocks.core import MissionError, VehicleState, parse_mission
from jl_blocks.library.aruco_tag import ArucoTag
from jl_blocks.library.yolo_drogue import YoloDrogue

LEVEL = (1.0, 0.0, 0.0, 0.0)
YAW_90 = (math.cos(math.pi / 4), 0.0, 0.0, math.sin(math.pi / 4))


def at(position=(0.0, 0.0, -1.0), attitude=LEVEL, stamp=0.0):
    return VehicleState(position_ned=position, attitude=attitude, stamp=stamp)


def aruco(**params):
    target = ArucoTag(ArucoTag.Params(**params))
    target.reset()
    return target


def yolo(**params):
    target = YoloDrogue(YoloDrogue.Params(**params))
    target.reset()
    return target


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


def test_the_camera_picks_the_topic():
    assert aruco().topic() == "/front/target_pose"
    assert aruco(camera="down").topic() == "/target_pose"
    assert yolo().topic() == "/tag_detections"


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


def test_an_unknown_camera_is_rejected():
    with pytest.raises(MissionError, match="camera must be 'front' or 'down'"):
        parse_mission(
            "name: M\ntarget: {aruco_tag: {camera: side}}\nsteps:\n  - hold: {}\n"
        )


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
