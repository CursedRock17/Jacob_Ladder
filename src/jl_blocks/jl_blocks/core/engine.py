"""Runs a MissionSpec one tick at a time.

Each tick: ask the step's target where the thing is, ask the step's mission
block what it wants, pass that through the controller, then decide whether the
step is done, failed, or still running. Time is always passed in, never read
from a clock, so every transition can be tested without sleeping.
"""

from __future__ import annotations

import copy
import traceback
from dataclasses import dataclass
from typing import TypeVar

from .blocks import REGISTRY, Block, Controller, Mission, Registry, StepContext, Target
from .loader import BlockSpec, MissionSpec, StepSpec
from .types import Action, ActionStatus, Setpoint, Status, VehicleState

T = TypeVar("T", bound=Block)


@dataclass
class _Step:
    spec: StepSpec
    mission: Mission
    target: Target | None
    controller: Controller
    # True when this step's target is its own override, not the mission's shared
    # target -- only an override is reset when the step is (re-)entered.
    own_target: bool = False


class Engine:
    def __init__(self, spec: MissionSpec, registry: Registry = REGISTRY) -> None:
        self.spec = spec
        self._registry = registry
        self._shared_target = self._build(spec.target, Target) if spec.target else None
        shared_controller = self._build(spec.controller, Controller)
        self._steps = [
            _Step(
                spec=s,
                mission=self._build(s.block, Mission),
                target=self._build(s.target, Target)
                if s.target
                else self._shared_target,
                controller=self._build(s.controller, Controller)
                if s.controller
                else shared_controller,
                own_target=s.target is not None,
            )
            for s in spec.steps
        ]
        self._index = {s.spec.name: i for i, s in enumerate(self._steps)}
        self._i = 0
        self._t0 = 0.0
        self._last_seen = 0.0
        self._hold_position: tuple[float, float, float] | None = None
        self.started = False
        self.finished = False
        # True when a step failed with no on_fail: the runner should hold, then land.
        self.aborted = False
        self.events: list[str] = []
        # traceback.format_exc() from the most recent block exception the engine
        # caught (a bad tick() or a bad reset()), for the runner to log.
        self.last_error: str | None = None
        # Bumped on every step entry, so a late executor reply for a step that
        # has already ended can be recognised and ignored.
        self._entry = 0
        self._action: Action | None = None
        self._action_sent = False
        self._action_status = ActionStatus.NONE
        self._action_message = ""

    def _build(self, spec: BlockSpec, base: type[T]) -> T:
        cls = self._registry.get(spec.name)
        if cls is None or not issubclass(cls, base):
            raise ValueError(
                f"'{spec.name}' is not a registered {base.__name__.lower()}"
            )
        return cls(copy.deepcopy(spec.params))

    @property
    def state(self) -> str:
        """One word for /jl/NAME/state: the current step name, 'finished' or 'aborted'."""
        if self.aborted:
            return "aborted"
        if self.finished:
            return "finished"
        return self._steps[self._i].spec.name if self.started else "idle"

    def start(self, now: float) -> None:
        self.started = True
        self.finished = False
        self.aborted = False
        self.last_error = None
        if self._reset_all_targets():
            self._enter(0, now)

    def take_action(self) -> tuple[int, Action] | None:
        """The current step's executor request, handed out once, with its step entry."""
        if self._action is None or self._action_sent or self.finished or self.aborted:
            return None
        self._action_sent = True
        return self._entry, self._action

    def action_done(self, entry: int, success: bool, message: str) -> None:
        """The executor's reply to the request take_action() handed out for `entry`."""
        if (
            entry != self._entry
            or self._action is None
            or self._action_status is not ActionStatus.PENDING
            or self.finished
            or self.aborted
        ):
            return  # the step it was for has already ended
        self._action_status = ActionStatus.SUCCEEDED if success else ActionStatus.FAILED
        self._action_message = message
        outcome = "ok" if success else "failed"
        self.events.append(
            f"{self._steps[self._i].spec.name}: {self._action.kind} {outcome} ({message})"
        )

    def targets(self) -> list[Target]:
        """Every distinct target instance: the shared one and each step override."""
        found: dict[int, Target] = {}
        if self._shared_target is not None:
            found[id(self._shared_target)] = self._shared_target
        for step in self._steps:
            if step.target is not None:
                found[id(step.target)] = step.target
        return list(found.values())

    def drain_events(self) -> list[str]:
        """The events since the last call, each returned once."""
        events, self.events = self.events, []
        return events

    def tick(self, vehicle: VehicleState, now: float, dt: float) -> Setpoint | None:
        if not self.started:
            return None
        if self.finished or self.aborted:
            return Setpoint(position=self._hold_position)

        step = self._steps[self._i]
        try:
            estimate = step.target.estimate(vehicle) if step.target else None
            if estimate is not None:
                self._last_seen = now
            ctx = StepContext(
                vehicle=vehicle,
                target=step.target,
                target_position=estimate,
                controller=step.controller,
                elapsed=now - self._t0,
                dt=dt,
                action=self._action_status,
                action_message=self._action_message,
            )
            result = step.mission.step(ctx)
            status = step.mission.status(ctx)
            if isinstance(result, Action):
                if self._action_status is ActionStatus.NONE:
                    self._action = result
                    self._action_status = ActionStatus.PENDING
                    self.events.append(f"{step.spec.name}: requested {result.kind}")
                setpoint = None
            elif result is not None:
                setpoint = step.controller.command(vehicle, result, dt)
            else:
                setpoint = None
        except Exception as err:  # a block bug must fail the step, not crash the runner
            self.last_error = traceback.format_exc()
            self._fail(step, f"{type(err).__name__}: {err}", vehicle, now)
            return Setpoint(position=vehicle.position_ned)

        if status is Status.FAILED:
            reason = "block reported failure"
            if self._action is not None and self._action_status is ActionStatus.FAILED:
                reason = f"{self._action.kind} failed: {self._action_message}"
            self._fail(step, reason, vehicle, now)
        elif step.spec.timeout is not None and now - self._t0 >= step.spec.timeout:
            self._fail(step, f"timeout after {step.spec.timeout:g} s", vehicle, now)
        elif self._until_met(step, status, estimate, now):
            self._advance(vehicle, now)
        return setpoint

    def _until_met(
        self, step: _Step, status: Status, estimate: object, now: float
    ) -> bool:
        until = step.spec.until
        if until == "done":
            return status is Status.DONE
        if until == "never":
            return False
        if until == "target_seen":
            return estimate is not None
        if until == "target_lost":
            lost_after = step.target.lost_after if step.target else Target.lost_after
            return now - self._last_seen >= lost_after
        return now - self._t0 >= float(until)

    def _reset_all_targets(self) -> bool:
        """Reset every distinct Target instance once, before entering step 0.

        Returns False (and aborts) if any of them raises. The abort names the
        first step that uses the failing target.
        """
        owners: dict[int, tuple[_Step, Target]] = {}
        for step in self._steps:
            if step.target is not None and id(step.target) not in owners:
                owners[id(step.target)] = (step, step.target)
        for step, target in owners.values():
            try:
                target.reset()
            except Exception as err:
                self._abort_from_reset(step, err, None)
                return False
        return True

    def _abort_from_reset(
        self, step: _Step, err: Exception, vehicle: VehicleState | None
    ) -> None:
        """A reset() raised while entering `step`: abort (no on_fail is followed)."""
        self.last_error = traceback.format_exc()
        self.aborted = True
        self._hold_position = vehicle.position_ned if vehicle is not None else None
        self.events.append(
            f"{step.spec.name}: failed (reset: {type(err).__name__}: {err}); "
            "no on_fail -> hold, then land"
        )

    def _enter(
        self, index: int, now: float, vehicle: VehicleState | None = None
    ) -> bool:
        self._entry += 1
        self._action = None
        self._action_sent = False
        self._action_status = ActionStatus.NONE
        self._action_message = ""
        self._i = index
        self._t0 = now
        self._last_seen = now
        step = self._steps[index]
        try:
            step.mission.reset()
            step.controller.reset()
            if step.own_target and step.target is not None:
                step.target.reset()
        except Exception as err:
            self._abort_from_reset(step, err, vehicle)
            return False
        self.events.append(f"{step.spec.name}: started")
        return True

    def _advance(self, vehicle: VehicleState, now: float) -> None:
        self.events.append(f"{self._steps[self._i].spec.name}: done")
        if self._i + 1 < len(self._steps):
            self._enter(self._i + 1, now, vehicle)
        else:
            self.finished = True
            self._hold_position = vehicle.position_ned
            self.events.append("mission finished")

    def _fail(
        self, step: _Step, reason: str, vehicle: VehicleState, now: float
    ) -> None:
        on_fail = step.spec.on_fail
        if on_fail is not None:
            self.events.append(f"{step.spec.name}: failed ({reason}) -> {on_fail}")
            self._enter(self._index[on_fail], now, vehicle)
        else:
            self.events.append(
                f"{step.spec.name}: failed ({reason}); no on_fail -> hold, then land"
            )
            self.aborted = True
            self._hold_position = vehicle.position_ned
