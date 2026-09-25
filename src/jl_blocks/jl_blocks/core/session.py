"""The mission_runner's decisions, with no ROS: when to start, what to send, when to land.

The ROS node (jl_blocks.ros.mission_runner) only moves data between topics and
this class, so everything here is tested with plain pytest. See the rules in
docs/superpowers/plans/2026-09-24-mission-blocks-phase3-runner-and-blocks.md,
Task 9.
"""

from __future__ import annotations

import traceback
from dataclasses import dataclass

from .blocks import REGISTRY, Registry
from .engine import Engine
from .geometry import is_finite
from .loader import MissionSpec
from .types import Action, Setpoint, Vector3, VehicleState

NAN = float("nan")


@dataclass(frozen=True)
class Request:
    """An executor call for the runner to make; pass `token` back to action_done()."""

    token: int
    action: Action


@dataclass(frozen=True)
class Output:
    setpoint: Setpoint | None = None
    requests: tuple[Request, ...] = ()


def trajectory_fields(setpoint: Setpoint) -> tuple[list[float], list[float], float]:
    """Position, velocity and yaw for px4_msgs/TrajectorySetpoint (NaN = not controlled)."""
    position = list(setpoint.position) if setpoint.position is not None else [NAN] * 3
    velocity = list(setpoint.velocity) if setpoint.velocity is not None else [NAN] * 3
    yaw = setpoint.yaw if setpoint.yaw is not None else NAN
    return position, velocity, yaw


class Session:
    def __init__(
        self,
        spec: MissionSpec,
        registry: Registry = REGISTRY,
        abort_hold_s: float = 2.0,
        vehicle_timeout_s: float = 0.5,
        abort_land_timeout_s: float = 5.0,
    ) -> None:
        self.spec = spec
        self._registry = registry
        self._abort_hold_s = abort_hold_s
        self._vehicle_timeout_s = vehicle_timeout_s
        self._abort_land_timeout_s = abort_land_timeout_s
        self.vehicle: VehicleState | None = None
        self.engine: Engine | None = None
        self._next_token = 0
        # token -> the engine step entry it belongs to, or None for the abort landing
        self._requests: dict[int, int | None] = {}
        self._aborted_at: float | None = None
        self._abort_land_sent = False
        # When the abort land request was sent, and its token, so a missing or
        # failed reply can make the runner go quiet instead of holding forever.
        self._abort_land_sent_at: float | None = None
        self._abort_land_token: int | None = None
        self._abort_quiet = False
        self._seen_active = False
        self._events: list[str] = []
        self._errors: list[str] = []

    @property
    def state(self) -> str:
        return self.engine.state if self.engine is not None else "inactive"

    def topics(self) -> list[str]:
        """Every topic a target in this mission listens on."""
        engine = Engine(self.spec, self._registry)
        return sorted({t for target in engine.targets() if (t := target.topic())})

    def set_active(self, active: bool, now: float) -> None:
        # Every "true" is a new activation, even without a "false" in between:
        # /jl/NAME/active is latched with depth 1, so a quick false/true can
        # arrive as a single true. The mission starts again from step 1.
        #
        # But the very first /active message this Session ever sees, if it's
        # already true, means the topic was latched true before this runner
        # existed (e.g. systemd restarted a dead runner mid-flight). Spec
        # §10: no resume. Ignore it; the pilot re-selects the mission in QGC
        # to fly it, which republishes a fresh true.
        if not self._seen_active:
            self._seen_active = True
            if active:
                self._events.append(
                    "ignored: the mission was already active when the runner "
                    "started; select it again in QGC to fly it"
                )
                return
        if self.engine is not None:
            self._events.extend(self.engine.drain_events())
        self._requests.clear()
        self._aborted_at = None
        self._abort_land_sent = False
        self._abort_land_sent_at = None
        self._abort_land_token = None
        self._abort_quiet = False
        if active:
            self._events.append("activated: starting from step 1")
            self.engine = Engine(self.spec, self._registry)
            self.engine.start(now)
            self._collect_error()
        else:
            if self.engine is not None:
                self._events.append("deactivated")
            self.engine = None

    def update_vehicle(self, vehicle: VehicleState) -> None:
        finite = (
            is_finite(vehicle.position_ned)
            and is_finite(vehicle.velocity_ned)
            and is_finite(vehicle.attitude)
        )
        self.vehicle = vehicle if finite else None

    def observe(self, topic: str, position: Vector3) -> None:
        if self.engine is None or self.vehicle is None:
            return  # nothing to place it with, or nobody listening
        for target in self.engine.targets():
            if target.topic() != topic:
                continue
            try:
                target.observe(position, self.vehicle)
            except Exception:  # a custom target's bug must not stop the runner
                self._errors.append(traceback.format_exc())

    def tick(self, now: float, dt: float) -> Output:
        engine, vehicle = self.engine, self.vehicle
        if engine is None or vehicle is None:
            return Output()
        if now - vehicle.stamp > self._vehicle_timeout_s:
            # The vehicle topic went quiet: count it as unknown again so the
            # runner goes quiet too, and jl_mission holds then lands on silence.
            return Output()
        setpoint = engine.tick(vehicle, now, dt)
        self._collect_error()

        requests = []
        taken = engine.take_action()
        if taken is not None:
            entry, action = taken
            requests.append(self._request(action, entry))
        if engine.aborted:
            if self._aborted_at is None:
                self._aborted_at = now
            waited = now - self._aborted_at
            if not self._abort_land_sent and waited >= self._abort_hold_s:
                self._abort_land_sent = True
                self._abort_land_sent_at = now
                self._events.append("aborted: held, now landing")
                request = self._request(Action("land"), None)
                self._abort_land_token = request.token
                requests.append(request)
            elif (
                self._abort_land_sent_at is not None
                and not self._abort_quiet
                and self._abort_land_token in self._requests  # no reply yet
                and now - self._abort_land_sent_at >= self._abort_land_timeout_s
            ):
                self._abort_quiet = True
                self._events.append(
                    f"aborted: no land reply after {self._abort_land_timeout_s:g} s; "
                    "going quiet (jl_mission lands on silence, or is already landing)"
                )

        empty = setpoint is not None and (
            setpoint.position is None and setpoint.velocity is None
        )
        if vehicle.landed or empty or self._abort_quiet:
            setpoint = None
        return Output(setpoint, tuple(requests))

    def action_done(self, token: int, success: bool, message: str) -> None:
        if token not in self._requests or self.engine is None:
            return  # from an earlier activation
        entry = self._requests.pop(token)
        if entry is None:
            outcome = "ok" if success else "failed"
            self._events.append(f"abort landing {outcome} ({message})")
            if not success:
                self._abort_quiet = True
                self._events.append(
                    "aborted: land failed, going quiet so jl_mission lands on silence"
                )
            return
        self.engine.action_done(entry, success, message)

    def drain_events(self) -> list[str]:
        events, self._events = self._events, []
        if self.engine is not None:
            events.extend(self.engine.drain_events())
        return events

    def drain_errors(self) -> list[str]:
        errors, self._errors = self._errors, []
        return errors

    def _request(self, action: Action, entry: int | None) -> Request:
        self._next_token += 1
        self._requests[self._next_token] = entry
        return Request(self._next_token, action)

    def _collect_error(self) -> None:
        if self.engine is not None and self.engine.last_error is not None:
            self._errors.append(self.engine.last_error)
            self.engine.last_error = None
