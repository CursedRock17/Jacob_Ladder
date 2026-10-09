"""A simulated drone for testing jl_flight without ROS or PX4.

The clock only moves when the Drone sleeps. The vehicle chases the last setpoint
it was sent at 1 m/s (like jl_mission's speed limit) and turns at 90 degrees/s.
It can also carry tags: put_tag() places one in the world, and while it is in
view of the front camera the fake publishes detections for it on every step,
exactly as aruco_tracker would (OpenCV optical frame, camera offset included).
"""

from __future__ import annotations

import math
from collections.abc import Callable

from jl_flight.core import Setpoint, VehicleState

Vector3 = tuple[float, float, float]


class FakeBackend:
    def __init__(
        self, active: bool = True, camera_offset: Vector3 = (0.0, 0.0, 0.0)
    ) -> None:
        self.t = 0.0
        self.position = [0.0, 0.0, 0.0]  # NED
        self.velocity = [0.0, 0.0, 0.0]
        self.yaw = 0.0
        self.landed = True
        self.active = active
        self.selection_count = 1 if active else 0
        self.stale = False  # True: vehicle() reports no data
        self.setpoint: Setpoint | None = None
        self.published: list[tuple[float, Setpoint]] = []
        self.calls: list[str] = []
        self.events: list[str] = []
        self.fail: dict[str, str] = {}  # call name -> the failure message to return
        self.closed = False
        self.camera_offset = (
            camera_offset  # forward, right, down from the vehicle origin
        )
        self._tags: dict[str, tuple[Vector3, str] | None] = {}
        self._listeners: dict[str, list[Callable[[Vector3], None]]] = {}

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
            attitude=(math.cos(self.yaw / 2), 0.0, 0.0, math.sin(self.yaw / 2)),
            landed=self.landed,
            stamp=self.t,
        )

    def is_active(self) -> bool:
        return self.active

    def selections(self) -> int:
        return self.selection_count

    def select(self) -> None:
        """The pilot selects the mode."""
        self.active = True
        self.selection_count += 1

    def publish(self, setpoint: Setpoint) -> None:
        self.setpoint = setpoint
        self.published.append((self.t, setpoint))

    def subscribe_pose(self, topic: str, callback: Callable[[Vector3], None]) -> None:
        self._listeners.setdefault(topic, []).append(callback)

    def event(self, text: str) -> None:
        self.events.append(text)

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

    # ── tags ──

    def put_tag(self, topic: str, ned: Vector3 | None, frame: str = "optical") -> None:
        """Place a target in the world (None takes it away). Detections are
        published in `frame`: "optical" like aruco_tracker, "ranging" like the
        YOLO pose node (+x left, +y up, +z forward)."""
        self._tags[topic] = None if ned is None else (ned, frame)

    def _detections(self) -> None:
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        for topic, tag in self._tags.items():
            if tag is None or topic not in self._listeners:
                continue
            ned, frame = tag
            dn, de, dd = (ned[i] - self.position[i] for i in range(3))
            forward, right, down = dn * c + de * s, -dn * s + de * c, dd
            f, r, d = self.camera_offset
            x, y, z = (
                right - r,
                down - d,
                forward - f,
            )  # OpenCV: right, down, out of the lens
            if z > 0.3:  # in front of the camera
                seen = (-x, -y, z) if frame == "ranging" else (x, y, z)
                for callback in self._listeners[topic]:
                    callback(seen)

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
        self._detections()
