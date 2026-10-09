"""Track: fly to a standoff point in front of the target and keep station there.

After DroneSmoothPlanner's Approach and Hover states. The standoff point is
`standoff` metres back from the target along the drone -> target bearing, at
the target's altitude, so the path to it never crosses the target and drifting
too close puts it behind the drone (the same setpoint then backs away). The
drone faces the target throughout.

The setpoint is a reference point that starts at the drone and moves toward
the standoff at `max_speed`, with that speed as velocity feed-forward, so the
drone really flies at `max_speed` and a target that jumps never causes a jump
in the setpoint. (DroneSmoothPlanner instead put the setpoint a fixed lead
ahead of the drone *and* fed the full speed forward; PX4 adds the two, so it
flew at about twice its max_velocity. Its Approach/Hover hysteresis is not
needed here: the reference itself is speed-limited.)

Done once the drone has settled at the standoff, within `tolerance` on every
axis and nearly still for `settle` seconds, so the final standoff comes from
detections taken at rest, which camera latency does not bias. With
`keep_station` it never finishes, and keeps re-tracking a moving target.
Failed once the target has been lost for `lost_after`; while it is lost the
drone holds where it is.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from ..core import Setpoint, Skill, SkillContext, Status, Target, Vector3
from ..core.geometry import add, norm, scale, sub

ARRIVE_SPEED = 0.2  # m/s: slower than this counts as settled


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


@dataclass(eq=False)
class Track(Skill):
    needs_target = True

    standoff: float = 3.0  # m kept back from the target
    max_speed: float = 0.5  # m/s the setpoint moves at
    tolerance: float = 0.25  # m per NED axis counted as arrived
    settle: float = 0.5  # s the drone must stay arrived and nearly still
    keep_station: bool = (
        False  # True: never finish; hold the standoff as the target moves
    )

    def __post_init__(self) -> None:
        if self.standoff <= 0:
            raise ValueError("standoff must be > 0, or the drone flies into the target")
        if min(self.max_speed, self.tolerance) <= 0 or self.settle < 0:
            raise ValueError("max_speed and tolerance must be > 0, and settle >= 0")
        self.reset()

    def reset(self) -> None:
        self._ref: Vector3 | None = (
            None  # the commanded point, moving toward the standoff
        )
        self._standoff: Vector3 | None = (
            None  # this tick's standoff point, None if unseen
        )
        self._last_seen = 0.0  # ctx.elapsed of the last estimate
        self._hold: Vector3 | None = None
        self._hold_yaw = 0.0
        self._since: float | None = None  # when the drone first counted as arrived

    def step(self, ctx: SkillContext) -> Setpoint:
        here = ctx.vehicle.position_ned
        target = ctx.target_position
        if target is None:
            if self._hold is None:
                self._hold, self._hold_yaw = here, ctx.vehicle.yaw
            # When it is seen again, the reference restarts from where the drone is.
            self._ref, self._standoff = self._hold, None
            return Setpoint(position=self._hold, yaw=self._hold_yaw)

        self._last_seen = ctx.elapsed
        self._hold = None
        self._standoff = standoff_point(here, target, self.standoff)
        if math.hypot(target[0] - here[0], target[1] - here[1]) < 1e-3:
            yaw = ctx.vehicle.yaw
        else:
            yaw = math.atan2(target[1] - here[1], target[0] - here[0])

        ref = here if self._ref is None else self._ref
        to_go = sub(self._standoff, ref)
        distance = norm(to_go)
        step = self.max_speed * ctx.dt
        if distance <= step:
            self._ref = self._standoff
            return Setpoint(position=self._standoff, yaw=yaw)
        direction = scale(to_go, 1.0 / distance)
        self._ref = add(ref, scale(direction, step))
        return Setpoint(
            position=self._ref, velocity=scale(direction, self.max_speed), yaw=yaw
        )

    def status(self, ctx: SkillContext) -> Status:
        lost_after = ctx.target.lost_after if ctx.target else Target.lost_after
        if ctx.elapsed - self._last_seen >= lost_after:
            return Status.FAILED
        standoff, v = self._standoff, ctx.vehicle
        arrived = (
            standoff is not None
            and self._ref == standoff
            and all(
                abs(s - p) <= self.tolerance for s, p in zip(standoff, v.position_ned)
            )
            and norm(v.velocity_ned) < ARRIVE_SPEED
        )
        if not arrived:
            self._since = None
            return Status.RUNNING
        self._since = ctx.elapsed if self._since is None else self._since
        settled = ctx.elapsed - self._since >= self.settle
        return Status.DONE if settled and not self.keep_station else Status.RUNNING
