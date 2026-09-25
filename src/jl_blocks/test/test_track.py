from __future__ import annotations

import math

import pytest

from jl_blocks import library  # noqa: F401  (registers the shipped blocks)
from jl_blocks.core import (
    MissionError,
    Setpoint,
    Status,
    StepContext,
    VehicleState,
    parse_mission,
)
from jl_blocks.library.position import Position
from jl_blocks.library.track import Track

TARGET = (10.0, 0.0, -2.0)  # standoff 3 m back along +x: (7, 0, -2)


def ctx(position, target=TARGET, elapsed=0.0, yaw=0.0):
    return StepContext(
        vehicle=VehicleState(position_ned=position, yaw=yaw),
        target=None,
        target_position=target,
        controller=Position(),
        elapsed=elapsed,
        dt=0.02,
    )


def make(**params):
    track = Track(Track.Params(**params))
    track.reset()
    return track


def test_approach_leads_toward_the_standoff_at_max_speed():
    track = make()
    out = track.step(ctx((0.0, 0.0, -1.0)))
    d = math.sqrt(50.0)  # distance to (7, 0, -2)
    assert out.position == pytest.approx((0.5 * 7 / d, 0.0, -1.0 - 0.5 / d))
    assert out.velocity == pytest.approx((0.5 * 7 / d, 0.0, -0.5 / d))
    assert out.yaw == pytest.approx(0.0)


def test_speed_tapers_inside_the_lead_distance():
    out = make().step(ctx((6.6, 0.0, -2.0)))
    assert out.position == pytest.approx((7.0, 0.0, -2.0))
    assert out.velocity == pytest.approx((0.4, 0.0, 0.0))


def test_inside_tolerance_it_holds_the_standoff_and_is_done():
    track = make()
    c = ctx((6.9, 0.0, -2.0))
    out = track.step(c)
    assert out.position == pytest.approx((7.0, 0.0, -2.0))
    assert out.velocity is None
    assert out.yaw == pytest.approx(0.0)
    assert track.status(c) is Status.DONE


def test_hover_has_hysteresis():
    track = make()
    track.step(ctx((6.9, 0.0, -2.0)))  # arrived
    still_close = track.step(ctx((6.6, 0.0, -2.0)))  # 0.4 m out: still holding
    assert still_close.velocity is None
    too_far = track.step(ctx((6.4, 0.0, -2.0)))  # 0.6 m out: approach again
    assert too_far.velocity is not None


def test_too_close_it_backs_away():
    out = make().step(ctx((8.5, 0.0, -2.0)))
    assert out.position[0] < 8.5
    assert out.velocity == pytest.approx((-0.5, 0.0, 0.0))


def test_a_lost_target_holds_where_it_was_lost_then_fails():
    track = make()
    track.step(ctx((5.0, 0.0, -2.0), elapsed=0.0))
    lost = ctx((5.2, 0.0, -2.0), target=None, elapsed=1.0, yaw=0.3)
    assert track.step(lost) == Setpoint(position=(5.2, 0.0, -2.0), yaw=0.3)
    assert track.status(lost) is Status.RUNNING
    later = ctx((5.4, 0.0, -2.0), target=None, elapsed=3.0)
    assert track.step(later).position == (5.2, 0.0, -2.0)
    assert track.status(later) is Status.FAILED


def test_seeing_the_target_again_clears_the_loss():
    track = make()
    track.step(ctx((5.0, 0.0, -2.0), elapsed=0.0))
    track.step(ctx((5.0, 0.0, -2.0), target=None, elapsed=2.5))
    seen = ctx((5.0, 0.0, -2.0), elapsed=2.9)
    track.step(seen)
    assert (
        track.status(ctx((5.0, 0.0, -2.0), target=None, elapsed=4.0)) is Status.RUNNING
    )


def test_a_far_target_never_commands_a_big_jump():
    here = (0.0, 0.0, -1.0)
    out = make().step(ctx(here, target=(100.0, 50.0, -2.0)))
    assert math.dist(out.position, here) <= 0.5 + 1e-9


def test_directly_below_the_target_keeps_the_current_heading():
    out = make().step(ctx((0.0, 0.0, -1.0), target=(0.0, 0.0, -5.0), yaw=1.1))
    assert out.yaw == 1.1


def test_standoff_must_be_positive():
    with pytest.raises(MissionError, match="standoff"):
        parse_mission(
            "name: M\ntarget: {aruco_tag: {}}\nsteps:\n  - track: {standoff: 0}\n"
        )


def test_track_needs_a_target():
    with pytest.raises(MissionError, match="track needs a target"):
        parse_mission("name: M\nsteps:\n  - track: {}\n")
