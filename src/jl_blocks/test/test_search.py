from __future__ import annotations

import math

import pytest

from jl_blocks import library  # noqa: F401  (registers the shipped blocks)
from jl_blocks.core import (
    MissionError,
    Setpoint,
    StepContext,
    VehicleState,
    parse_mission,
)
from jl_blocks.library.position import Position
from jl_blocks.library.search import Search


def ctx(vehicle):
    return StepContext(
        vehicle=vehicle,
        target=None,
        target_position=None,
        controller=Position(),
        elapsed=0.0,
        dt=0.02,
    )


def make(**params):
    search = Search(Search.Params(**params))
    search.reset()
    return search


def test_hold_pattern_keeps_the_start_position_and_heading():
    search = make()
    start = VehicleState(position_ned=(1.0, 2.0, -1.5), yaw=0.7)
    expected = Setpoint(position=(1.0, 2.0, -1.5), yaw=0.7)
    assert search.step(ctx(start)) == expected
    drifted = VehicleState(position_ned=(1.4, 2.0, -1.5), yaw=0.9)
    assert search.step(ctx(drifted)) == expected


def test_spiral_is_centred_on_where_the_step_started_not_the_origin():
    search = make(pattern="spiral")
    search.step(ctx(VehicleState(position_ned=(20.0, -30.0, -2.0))))
    assert search.waypoints[0] == (20.0, -30.0, -2.0)
    for x, y, z in search.waypoints:
        assert math.hypot(x - 20.0, y + 30.0) <= 2.0 + 1e-9
        assert z == -2.0


def test_spiral_waypoints_are_close_together():
    search = make(pattern="spiral")
    search.step(ctx(VehicleState(position_ned=(0.0, 0.0, -2.0))))
    points = list(search.waypoints)
    for a, b in zip(points, points[1:] + points[:1]):
        assert math.dist(a, b) < 1.0  # far inside jl_mission's 5 m max_step_m


def test_spiral_goes_out_then_back_in():
    search = make(pattern="spiral", points=4, radius=1.0)
    search.step(ctx(VehicleState(position_ned=(0.0, 0.0, -2.0))))
    radii = [round(math.hypot(x, y), 3) for x, y, _ in search.waypoints]
    assert radii == [0.0, 0.25, 0.5, 0.75, 1.0, 0.75, 0.5, 0.25]


def test_spiral_moves_on_only_once_the_drone_is_there_and_slow():
    search = make(pattern="spiral")
    search.step(ctx(VehicleState(position_ned=(0.0, 0.0, -2.0))))
    second = search.waypoints[1]
    moving = VehicleState(position_ned=second, velocity_ned=(0.5, 0.0, 0.0))
    assert search.step(ctx(moving)).position == second
    still = VehicleState(position_ned=second)
    assert search.step(ctx(still)).position == search.waypoints[2]


def test_spiral_repeats_forever():
    search = make(pattern="spiral", points=4, radius=1.0)
    here = (0.0, 0.0, -2.0)
    visited = []
    for _ in range(9):
        here = search.step(ctx(VehicleState(position_ned=here))).position
        visited.append(here)
    assert visited[7] == search.waypoints[0]
    assert visited[8] == search.waypoints[1]


def test_an_unknown_pattern_is_rejected_with_the_choices():
    with pytest.raises(MissionError, match="pattern must be 'hold' or 'spiral'"):
        parse_mission("name: M\nsteps:\n  - search: {pattern: zigzag}\n")
