"""track: fly to a standoff point in front of the target and keep station there.

Port of DroneSmoothPlanner's Approach and Hover states. The standoff point is
`standoff` metres back from the target along the drone -> target bearing, at
the target's altitude, so the path to it never crosses the target and drifting
too close puts it behind the drone (the same setpoint then backs away).
Approach flies a carrot `max_speed * lead_time` ahead; within `tolerance` on
every axis it holds the standoff itself, until the error passes 2x tolerance.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from ..core import Mission, Setpoint, Status, StepContext, Target, Vector3, block
from ..core.geometry import add, norm, scale, sub


def standoff_point(position: Vector3, target: Vector3, standoff: float) -> Vector3:
    dx, dy = target[0] - position[0], target[1] - position[1]
    distance = math.hypot(dx, dy)
    if distance < 1e-3:  # right above or below: no bearing to back off along
        return (position[0], position[1], target[2])
    return (
        target[0] - standoff * dx / distance,
        target[1] - standoff * dy / distance,
        target[2],
    )


@block("track")
class Track(Mission):
    needs_target = True

    @dataclass
    class Params:
        standoff: float = 3.0  # m kept back from the target
        lead_time: float = 1.0  # s the setpoint leads the drone
        max_speed: float = 0.5  # m/s approach speed
        tolerance: float = 0.25  # m per NED axis counted as arrived

        def __post_init__(self) -> None:
            if self.standoff <= 0:
                raise ValueError(
                    "standoff must be > 0, or the drone flies into the target"
                )
            if min(self.lead_time, self.max_speed, self.tolerance) <= 0:
                raise ValueError("lead_time, max_speed and tolerance must be > 0")

    def reset(self) -> None:
        self._arrived = False
        self._last_seen = 0.0  # ctx.elapsed of the last estimate
        self._hold: Vector3 | None = None
        self._hold_yaw = 0.0

    def step(self, ctx: StepContext) -> Setpoint:
        here = ctx.vehicle.position_ned
        target = ctx.target_position
        if target is None:
            if self._hold is None:
                self._hold, self._hold_yaw = here, ctx.vehicle.yaw
            self._arrived = False
            return Setpoint(position=self._hold, yaw=self._hold_yaw)

        self._last_seen = ctx.elapsed
        self._hold = None
        standoff = standoff_point(here, target, self.params.standoff)
        error = sub(standoff, here)
        if math.hypot(target[0] - here[0], target[1] - here[1]) < 1e-3:
            yaw = ctx.vehicle.yaw
        else:
            yaw = math.atan2(target[1] - here[1], target[0] - here[0])

        band = self.params.tolerance * (2.0 if self._arrived else 1.0)
        self._arrived = all(abs(e) <= band for e in error)
        if self._arrived:
            return Setpoint(position=standoff, yaw=yaw)

        distance = norm(error)
        direction = scale(error, 1.0 / distance)
        full_lead = self.params.max_speed * self.params.lead_time
        lead = min(full_lead, distance)
        speed = self.params.max_speed * min(1.0, distance / full_lead)
        return Setpoint(
            position=add(here, scale(direction, lead)),
            velocity=scale(direction, speed),
            yaw=yaw,
        )

    def status(self, ctx: StepContext) -> Status:
        lost_after = ctx.target.lost_after if ctx.target else Target.lost_after
        if ctx.elapsed - self._last_seen >= lost_after:
            return Status.FAILED
        return Status.DONE if self._arrived else Status.RUNNING
