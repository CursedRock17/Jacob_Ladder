"""search: wait for the target, holding still or flying a spiral.

`hold` is the Search state of DroneSmoothPlanner / FrontApproach. `spiral` is
PrecisionLand's search pattern, centred on where this step started (the
original spirals around the NED origin, which only works when the drone took
off above the pad). Both keep the heading the step started with.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from ..core import Mission, Setpoint, StepContext, Vector3, block
from ..core.geometry import norm, sub

PATTERNS = ("hold", "spiral")


@block("search")
class Search(Mission):
    @dataclass
    class Params:
        pattern: str = "hold"  # hold or spiral
        radius: float = 2.0  # m, spiral only
        points: int = 16  # waypoints per spiral, out and in each
        reach: float = 0.2  # m and m/s: a waypoint counts as reached

        def __post_init__(self) -> None:
            if self.pattern not in PATTERNS:
                raise ValueError(
                    f"pattern must be 'hold' or 'spiral', got {self.pattern!r}"
                )
            if self.radius <= 0 or self.reach <= 0 or self.points < 2:
                raise ValueError("radius and reach must be > 0, and points >= 2")

    def reset(self) -> None:
        self._waypoints: list[Vector3] = []
        self._index = 0
        self._yaw = 0.0

    @property
    def waypoints(self) -> tuple[Vector3, ...]:
        return tuple(self._waypoints)

    def step(self, ctx: StepContext) -> Setpoint:
        vehicle = ctx.vehicle
        if not self._waypoints:
            self._waypoints = self._plan(vehicle.position_ned)
            self._yaw = vehicle.yaw
        here = self._waypoints[self._index]
        reached = norm(sub(here, vehicle.position_ned)) < self.params.reach
        slow = norm(vehicle.velocity_ned) < self.params.reach
        if len(self._waypoints) > 1 and reached and slow:
            self._index = (self._index + 1) % len(self._waypoints)
        return Setpoint(position=self._waypoints[self._index], yaw=self._yaw)

    def _plan(self, start: Vector3) -> list[Vector3]:
        if self.params.pattern == "hold":
            return [start]
        n = self.params.points
        out = []
        for k in range(n + 1):
            r = self.params.radius * k / n
            angle = 2.0 * math.pi * k / n
            out.append(
                (
                    start[0] + r * math.cos(angle),
                    start[1] + r * math.sin(angle),
                    start[2],
                )
            )
        # Out to the full radius, then back in, without repeating either end.
        return out + out[-2:0:-1]
