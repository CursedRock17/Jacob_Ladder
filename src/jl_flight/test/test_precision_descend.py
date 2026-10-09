from __future__ import annotations

import pytest

from jl_flight.core import Setpoint, SkillContext, Status, VehicleState
from jl_flight.skills import PrecisionDescend

PAD = (0.0, 0.0, 0.0)


def ctx(position, target=PAD, elapsed=0.0, landed=False, yaw=0.4):
    return SkillContext(
        vehicle=VehicleState(position_ned=position, yaw=yaw, landed=landed),
        target=None,
        target_position=target,
        elapsed=elapsed,
        dt=0.02,
    )


def make(**params):
    descend = PrecisionDescend(**params)
    descend.reset()
    return descend


def test_descends_while_steering_over_the_target():
    out = make().step(ctx((0.2, -0.1, -2.0)))
    assert out.position is None
    assert out.velocity == pytest.approx((-0.34, 0.17, 0.6))
    assert out.yaw == 0.4


def test_horizontal_speed_is_clamped_per_axis():
    out = make().step(ctx((2.0, 0.0, -2.0)))
    assert out.velocity[0] == pytest.approx(-1.0)


def test_heading_stays_where_the_step_started():
    descend = make()
    descend.step(ctx((0.2, 0.0, -2.0), yaw=0.4))
    assert descend.step(ctx((0.2, 0.0, -1.9), yaw=0.9)).yaw == 0.4


def test_done_once_landed():
    descend = make()
    c = ctx((0.0, 0.0, 0.0), landed=True)
    descend.step(c)
    assert descend.status(c) is Status.DONE


def test_a_lost_target_holds_then_fails():
    descend = make()
    descend.step(ctx((0.0, 0.0, -1.0), elapsed=0.0))
    lost = ctx((0.1, 0.0, -0.9), target=None, elapsed=1.0)
    assert descend.step(lost) == Setpoint(position=(0.1, 0.0, -0.9), yaw=0.4)
    assert descend.status(lost) is Status.RUNNING
    assert (
        descend.status(ctx((0.1, 0.0, -0.9), target=None, elapsed=3.0)) is Status.FAILED
    )


def test_needs_a_target():
    assert PrecisionDescend.needs_target
