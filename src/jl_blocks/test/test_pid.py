from __future__ import annotations

import math

import pytest

from jl_blocks.core import Setpoint, VehicleState
from jl_blocks.library.pid import PID

DT = 0.02
AT = VehicleState(position_ned=(0.0, 0.0, -1.0))


def pid(**params):
    controller = PID(PID.Params(**params))
    controller.reset()
    return controller


def test_a_setpoint_without_a_position_passes_through():
    velocity_only = Setpoint(velocity=(0.1, 0.0, 0.0), yaw=0.3)
    assert pid().command(AT, velocity_only, DT) == velocity_only


def test_proportional_term_drives_toward_the_target():
    out = pid().command(AT, Setpoint(position=(1.0, 0.0, -1.0)), DT)
    assert out.position is None
    assert out.velocity == pytest.approx((0.8, 0.0, 0.0))


def test_horizontal_speed_is_capped_at_max_speed():
    out = pid().command(AT, Setpoint(position=(10.0, 10.0, -1.0)), DT)
    assert math.hypot(out.velocity[0], out.velocity[1]) == pytest.approx(1.0)


def test_vertical_speed_has_its_own_gain_and_cap():
    one_metre_up = pid().command(AT, Setpoint(position=(0.0, 0.0, -2.0)), DT)
    assert one_metre_up.velocity[2] == pytest.approx(-0.1)
    ten_cm_up = pid().command(AT, Setpoint(position=(0.0, 0.0, -1.1)), DT)
    assert ten_cm_up.velocity[2] == pytest.approx(-0.06)


def test_integral_is_limited_and_cleared_by_reset():
    controller = pid(kp=0.0, ki=1.0, kd=0.0, integral_limit=0.5)
    for _ in range(100):
        out = controller.command(AT, Setpoint(position=(1.0, 0.0, -1.0)), DT)
    assert out.velocity[0] == pytest.approx(0.5)
    controller.reset()
    out = controller.command(AT, Setpoint(position=(1.0, 0.0, -1.0)), DT)
    assert out.velocity[0] == pytest.approx(0.02)


def test_derivative_uses_the_change_in_error():
    controller = pid(kp=0.0, ki=0.0, kd=0.2, max_speed=5.0)
    first = controller.command(AT, Setpoint(position=(1.0, 0.0, -1.0)), DT)
    assert first.velocity[0] == 0.0
    second = controller.command(AT, Setpoint(position=(1.1, 0.0, -1.0)), DT)
    assert second.velocity[0] == pytest.approx(0.2 * 0.1 / DT)


def test_yaw_passes_through():
    out = pid().command(AT, Setpoint(position=(1.0, 0.0, -1.0), yaw=1.2), DT)
    assert out.yaw == 1.2
