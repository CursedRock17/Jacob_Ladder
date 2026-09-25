"""pid: fly toward the mission's position with a PID on the error, as velocity.

Port of FrontApproach's approach controller: PID in XY (speed capped by
magnitude, integral clamped per axis), P-only in Z, yaw passed through. A
setpoint that has no position is passed through unchanged.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from ..core import Controller, Setpoint, VehicleState, block


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


@block("pid")
class PID(Controller):
    @dataclass
    class Params:
        kp: float = 0.8  # 1/s
        ki: float = 0.0  # 1/s^2
        kd: float = 0.2  # unitless
        max_speed: float = 1.0  # m/s, horizontal
        integral_limit: float = 0.5  # m*s, per axis
        kp_z: float = 0.6  # 1/s
        max_speed_z: float = 0.1  # m/s, vertical

        def __post_init__(self) -> None:
            if min(self.kp, self.ki, self.kd, self.kp_z, self.integral_limit) < 0:
                raise ValueError("gains and integral_limit must not be negative")
            if self.max_speed <= 0 or self.max_speed_z <= 0:
                raise ValueError("max_speed and max_speed_z must be > 0")

    def reset(self) -> None:
        self._integral = (0.0, 0.0)
        self._previous: tuple[float, float] | None = None

    def command(self, vehicle: VehicleState, desired: Setpoint, dt: float) -> Setpoint:
        if desired.position is None:
            return desired
        p = self.params
        ex = desired.position[0] - vehicle.position_ned[0]
        ey = desired.position[1] - vehicle.position_ned[1]
        ix = _clamp(self._integral[0] + ex * dt, p.integral_limit)
        iy = _clamp(self._integral[1] + ey * dt, p.integral_limit)
        self._integral = (ix, iy)
        dx = dy = 0.0
        if self._previous is not None and dt > 1e-3:
            dx = (ex - self._previous[0]) / dt
            dy = (ey - self._previous[1]) / dt
        self._previous = (ex, ey)

        vx = p.kp * ex + p.ki * ix + p.kd * dx
        vy = p.kp * ey + p.ki * iy + p.kd * dy
        speed = math.hypot(vx, vy)
        if speed > p.max_speed:
            vx, vy = vx * p.max_speed / speed, vy * p.max_speed / speed
        ez = desired.position[2] - vehicle.position_ned[2]
        vz = _clamp(p.kp_z * ez, p.max_speed_z)
        return Setpoint(velocity=(vx, vy, vz), yaw=desired.yaw)
