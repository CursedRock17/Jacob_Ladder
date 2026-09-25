"""position: pass the mission's setpoint straight to PX4, whose own controller closes the loop."""

from __future__ import annotations

from ..core import Controller, Setpoint, VehicleState, block


@block("position")
class Position(Controller):
    def command(self, vehicle: VehicleState, desired: Setpoint, dt: float) -> Setpoint:
        return desired
