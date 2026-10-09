"""PID: fly toward the skill's position with a PID on the error, as velocity.

Port of FrontApproach's approach controller: PID in XY (speed capped by
magnitude, integral clamped per axis), P-only in Z, yaw passed through. A
setpoint that has no position is passed through unchanged.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from ..core import Controller, Setpoint, VehicleState


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


@dataclass
class PID(Controller):
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
        self.reset()

    def reset(self) -> None:
        self._integral = (0.0, 0.0)
        self._previous: tuple[float, float] | None = None

    def command(self, vehicle: VehicleState, desired: Setpoint, dt: float) -> Setpoint:
        if desired.position is None:
            return desired
        ex = desired.position[0] - vehicle.position_ned[0]
        ey = desired.position[1] - vehicle.position_ned[1]
        ix = _clamp(self._integral[0] + ex * dt, self.integral_limit)
        iy = _clamp(self._integral[1] + ey * dt, self.integral_limit)
        self._integral = (ix, iy)
        dx = dy = 0.0
        if self._previous is not None and dt > 1e-3:
            dx = (ex - self._previous[0]) / dt
            dy = (ey - self._previous[1]) / dt
        self._previous = (ex, ey)

        vx = self.kp * ex + self.ki * ix + self.kd * dx
        vy = self.kp * ey + self.ki * iy + self.kd * dy
        speed = math.hypot(vx, vy)
        if speed > self.max_speed:
            vx, vy = vx * self.max_speed / speed, vy * self.max_speed / speed
        ez = desired.position[2] - vehicle.position_ned[2]
        vz = _clamp(self.kp_z * ez, self.max_speed_z)
        return Setpoint(velocity=(vx, vy, vz), yaw=desired.yaw)
