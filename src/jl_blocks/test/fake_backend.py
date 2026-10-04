"""A simulated drone for testing jl_blocks.fly without ROS or PX4.

The clock only moves when the Drone sleeps. The vehicle chases the last setpoint
it was sent at 1 m/s (like jl_mission's speed limit) and turns at 90 degrees/s.
"""

from __future__ import annotations

import math

from jl_blocks.core import Setpoint, VehicleState


class FakeBackend:
    def __init__(self, active: bool = True) -> None:
        self.t = 0.0
        self.position = [0.0, 0.0, 0.0]  # NED
        self.velocity = [0.0, 0.0, 0.0]
        self.yaw = 0.0
        self.landed = True
        self.active = active
        self.stale = False  # True: vehicle() reports no data
        self.setpoint: Setpoint | None = None
        self.published: list[tuple[float, Setpoint]] = []
        self.calls: list[str] = []
        self.fail: dict[str, str] = {}  # call name -> the failure message to return
        self.closed = False

    # ── Backend ──

    def connect(self) -> None:
        self.calls.append("connect")

    def close(self) -> None:
        self.closed = True

    def now(self) -> float:
        return self.t

    def sleep(self, seconds: float) -> None:
        self._advance(seconds)

    def vehicle(self) -> VehicleState | None:
        if self.stale:
            return None
        return VehicleState(
            position_ned=(self.position[0], self.position[1], self.position[2]),
            velocity_ned=(self.velocity[0], self.velocity[1], self.velocity[2]),
            yaw=self.yaw,
            landed=self.landed,
            stamp=self.t,
        )

    def is_active(self) -> bool:
        return self.active

    def publish(self, setpoint: Setpoint) -> None:
        self.setpoint = setpoint
        self.published.append((self.t, setpoint))

    def arm(self) -> tuple[bool, str]:
        return self._call("arm")

    def takeoff(self, height: float) -> tuple[bool, str]:
        ok, message = self._call(f"takeoff {height:g}")
        if ok:
            self.position[2] = -height
            self.landed = False
        return ok, message

    def land(self) -> tuple[bool, str]:
        ok, message = self._call("land")
        if ok:
            self.position[2] = 0.0
            self.landed = True
        return ok, message

    # ── the simulated vehicle ──

    def _call(self, call: str) -> tuple[bool, str]:
        self.calls.append(call)
        name = call.split()[0]
        if name in self.fail:
            return False, self.fail[name]
        return True, "ok"

    def _advance(self, dt: float) -> None:
        sp = self.setpoint
        self.velocity = [0.0, 0.0, 0.0]
        if sp is not None and sp.position is not None:
            delta = [g - p for g, p in zip(sp.position, self.position, strict=True)]
            dist = math.hypot(*delta)
            step = min(dist, 1.0 * dt)
            if dist > 0:
                self.velocity = [d / dist * step / dt for d in delta]
                self.position = [
                    p + d / dist * step
                    for p, d in zip(self.position, delta, strict=True)
                ]
        elif sp is not None and sp.velocity is not None:
            self.velocity = list(sp.velocity)
            self.position = [
                p + v * dt for p, v in zip(self.position, self.velocity, strict=True)
            ]
        if sp is not None and sp.yaw is not None:
            error = math.remainder(sp.yaw - self.yaw, 2 * math.pi)
            turn = math.copysign(min(abs(error), math.radians(90) * dt), error)
            self.yaw = math.remainder(self.yaw + turn, 2 * math.pi)
        self.t += dt
