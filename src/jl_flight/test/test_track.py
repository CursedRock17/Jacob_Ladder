"""Track: the standoff geometry, a speed-limited reference, and arriving only once settled."""

from __future__ import annotations

import math

import pytest

from jl_flight.core import Setpoint, SkillContext, Status, VehicleState
from jl_flight.skills import Track

TARGET = (10.0, 0.0, -2.0)  # standoff 3 m back along +x: (7, 0, -2)
STANDOFF = (7.0, 0.0, -2.0)


def ctx(
    position, target=TARGET, elapsed=0.0, yaw=0.0, dt=0.02, velocity=(0.0, 0.0, 0.0)
):
    return SkillContext(
        vehicle=VehicleState(position_ned=position, velocity_ned=velocity, yaw=yaw),
        target=None,
        target_position=target,
        elapsed=elapsed,
        dt=dt,
    )


def make(**params):
    return Track(**params)


def test_the_setpoint_starts_at_the_drone_and_moves_at_max_speed():
    track = make()
    first = track.step(ctx((0.0, 0.0, -2.0), dt=0.0))
    assert first.position == pytest.approx((0.0, 0.0, -2.0))
    second = track.step(ctx((0.0, 0.0, -2.0), dt=0.1))
    assert second.position == pytest.approx((0.05, 0.0, -2.0))  # 0.5 m/s * 0.1 s
    assert second.velocity == pytest.approx((0.5, 0.0, 0.0))
    assert second.yaw == pytest.approx(0.0)


def test_the_setpoint_stops_at_the_standoff_point():
    track = make()
    out = None
    for _ in range(200):  # 4 s at 0.5 m/s covers the 1 m to go
        out = track.step(ctx((6.0, 0.0, -2.0)))
    assert out == Setpoint(position=STANDOFF, yaw=0.0)


def test_the_setpoint_never_jumps_however_far_the_target_moves():
    track = make()
    track.step(ctx((0.0, 0.0, -1.0), dt=0.0))
    out = track.step(ctx((0.0, 0.0, -1.0), target=(100.0, 50.0, -30.0), dt=0.02))
    assert math.dist(out.position, (0.0, 0.0, -1.0)) <= 0.5 * 0.02 + 1e-9


def test_too_close_it_backs_away():
    track = make()
    track.step(ctx((8.5, 0.0, -2.0), dt=0.0))
    out = track.step(ctx((8.5, 0.0, -2.0), dt=0.1))
    assert out.position == pytest.approx((8.45, 0.0, -2.0))
    assert out.velocity == pytest.approx((-0.5, 0.0, 0.0))


def settled_at_standoff(track, start=0.0, seconds=1.0, velocity=(0.0, 0.0, 0.0)):
    """Feed ticks with the drone sitting at the standoff; return the last status."""
    status = None
    for i in range(int(seconds / 0.02) + 1):
        c = ctx(STANDOFF, elapsed=start + i * 0.02, velocity=velocity)
        track.step(c)
        status = track.status(c)
    return status


def test_done_only_once_settled_at_the_standoff():
    track = make(settle=0.5)
    assert settled_at_standoff(track, seconds=0.3) is Status.RUNNING
    assert settled_at_standoff(track, start=0.32, seconds=0.3) is Status.DONE


def test_passing_through_the_standoff_at_speed_is_not_arriving():
    track = make()
    assert (
        settled_at_standoff(track, seconds=2.0, velocity=(0.9, 0.0, 0.0))
        is Status.RUNNING
    )


def test_keep_station_never_finishes_but_still_fails_when_lost():
    track = make(keep_station=True)
    assert settled_at_standoff(track, seconds=2.0) is Status.RUNNING
    lost = ctx(STANDOFF, target=None, elapsed=5.5)
    track.step(lost)
    assert track.status(lost) is Status.FAILED


def test_a_lost_target_holds_where_it_was_lost_then_fails():
    track = make()
    track.step(ctx((5.0, 0.0, -2.0), elapsed=0.0))
    lost = ctx((5.2, 0.0, -2.0), target=None, elapsed=1.0, yaw=0.3)
    assert track.step(lost) == Setpoint(position=(5.2, 0.0, -2.0), yaw=0.3)
    assert track.status(lost) is Status.RUNNING
    later = ctx((5.3, 0.0, -2.0), target=None, elapsed=3.0)
    assert track.step(later).position == (5.2, 0.0, -2.0)
    assert track.status(later) is Status.FAILED


def test_seeing_the_target_again_restarts_from_the_drone():
    track = make()
    track.step(ctx((5.0, 0.0, -2.0), elapsed=0.0))
    track.step(ctx((5.2, 0.0, -2.0), target=None, elapsed=2.0))
    again = track.step(ctx((5.2, 0.0, -2.0), elapsed=2.5, dt=0.1))
    assert again.position == pytest.approx((5.25, 0.0, -2.0))
    assert track.status(ctx((5.2, 0.0, -2.0), elapsed=2.5)) is Status.RUNNING


def test_directly_below_the_target_keeps_the_current_heading():
    out = make().step(ctx((0.0, 0.0, -1.0), target=(0.0, 0.0, -5.0), yaw=1.1))
    assert out.yaw == 1.1


def test_it_faces_the_target():
    out = make().step(ctx((0.0, 0.0, -2.0), target=(3.0, 3.0, -2.0)))
    assert out.yaw == pytest.approx(math.pi / 4)


def test_standoff_must_be_positive():
    with pytest.raises(ValueError, match="standoff"):
        Track(standoff=0)


def test_speed_and_tolerance_must_be_positive():
    with pytest.raises(ValueError, match="must be > 0"):
        Track(max_speed=0)


def test_track_needs_a_target():
    assert Track.needs_target
