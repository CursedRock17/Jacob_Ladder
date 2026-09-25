"""precision_descend: descend onto the target, steering to stay over it.

Port of PrecisionLand's Descend state (a downward camera). Differences from the
original: max_speed defaults to 1.0 m/s (jl_mission clamps to that anyway; the
flown value was 3.0), the integral uses error*dt (the original adds the raw
error per tick; identical with the flown ki = 0), and yaw holds the heading at
step start (the original yaws to the tag's orientation, which the target
blocks don't report).
"""

from __future__ import annotations

from dataclasses import dataclass

from ..core import Mission, Setpoint, Status, StepContext, Target, Vector3, block


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


@block("precision_descend")
class PrecisionDescend(Mission):
    needs_target = True

    @dataclass
    class Params:
        descent_speed: float = 0.6  # m/s down
        kp: float = 1.7  # 1/s
        ki: float = 0.0  # 1/s^2
        max_speed: float = 1.0  # m/s per horizontal axis

        def __post_init__(self) -> None:
            if self.descent_speed <= 0 or self.max_speed <= 0:
                raise ValueError("descent_speed and max_speed must be > 0")
            if self.kp < 0 or self.ki < 0:
                raise ValueError("kp and ki must not be negative")

    def reset(self) -> None:
        self._integral = (0.0, 0.0)
        self._last_seen = 0.0
        self._yaw: float | None = None
        self._hold: Vector3 | None = None

    def step(self, ctx: StepContext) -> Setpoint:
        vehicle = ctx.vehicle
        if self._yaw is None:
            self._yaw = vehicle.yaw
        target = ctx.target_position
        if target is None:
            if self._hold is None:
                self._hold = vehicle.position_ned
            return Setpoint(position=self._hold, yaw=self._yaw)

        self._last_seen = ctx.elapsed
        self._hold = None
        p = self.params
        ex = vehicle.position_ned[0] - target[0]
        ey = vehicle.position_ned[1] - target[1]
        ix = _clamp(self._integral[0] + ex * ctx.dt, p.max_speed)
        iy = _clamp(self._integral[1] + ey * ctx.dt, p.max_speed)
        self._integral = (ix, iy)
        vx = _clamp(-(p.kp * ex + p.ki * ix), p.max_speed)
        vy = _clamp(-(p.kp * ey + p.ki * iy), p.max_speed)
        return Setpoint(velocity=(vx, vy, p.descent_speed), yaw=self._yaw)

    def status(self, ctx: StepContext) -> Status:
        if ctx.vehicle.landed:
            return Status.DONE
        lost_after = ctx.target.lost_after if ctx.target else Target.lost_after
        if ctx.elapsed - self._last_seen >= lost_after:
            return Status.FAILED
        return Status.RUNNING
