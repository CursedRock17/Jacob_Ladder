from __future__ import annotations

from jl_blocks.core import Setpoint, Status, VehicleState


def test_vehicle_state_defaults_to_still_and_level():
    v = VehicleState(position_ned=(1.0, 2.0, -3.0))
    assert v.velocity_ned == (0.0, 0.0, 0.0)
    assert v.attitude == (1.0, 0.0, 0.0, 0.0)
    assert not v.landed


def test_setpoint_fields_default_to_not_controlled():
    assert Setpoint() == Setpoint(position=None, velocity=None, yaw=None)


def test_status_has_three_values():
    assert [s.value for s in Status] == ["running", "done", "failed"]
