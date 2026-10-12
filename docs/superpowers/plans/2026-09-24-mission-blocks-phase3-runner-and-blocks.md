# Mission Blocks Phase 3: `mission_runner` and the shipped blocks — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make a mission file fly. This adds the `mission_runner` ROS node that drives `jl_blocks.core.Engine` against `jl_mission`, plus the first-release blocks from spec §3, each ported from code that already flies and each with a unit test.

**Architecture:**
- **Action channel (spec §11 open item):** a mission block's `step()` may return an `Action` (`takeoff` or `land`) instead of a `Setpoint`. The engine hands it out once through `take_action()`, and the runner calls the `jl_mission` service. The reply comes back through `action_done()` and appears in the block's next `StepContext.action`. A reply for a step that has already ended is ignored.
- **`Session`** (in `jl_blocks.core`, no ROS) holds every runner decision:
  - every `/active` true starts the mission again from step 1;
  - the vehicle state must be known before anything is sent;
  - it never sends setpoints while landed;
  - after an abort, it holds for 2 s, then asks to land.

  The ROS node `jl_blocks.ros.mission_runner` only moves data between topics and `Session`, so almost everything is covered by plain `pytest` in `make check`.
- **Target blocks** receive camera detections through `observe()`. The detection is converted to NED immediately, using the vehicle state at the moment it arrived (like `FrontApproach`), and `estimate()` returns it while it is fresh.

**Tech Stack:** Python 3.10, `jl_blocks.core` (pure Python, pyyaml only), pytest 9.1.1, ruff 0.15.20, ty 0.0.55 (all via `make check`), ROS 2 Humble `rclpy` for the node only, `px4_msgs`, `jl_mission_interfaces`, PX4 v1.16.0 SITL in the `jacob_ladder_sim` container.

**Spec:** `docs/superpowers/specs/2026-09-21-mission-blocks-design.md`. This plan implements phase 3 of §9: the §3 shipped blocks, the §4 mission-file rules that were still missing (name length), and the §11 action-channel open item. It also adds a SITL smoke flight so the runner is proven end to end before Phase 4 builds the general `test/sitl_mission.sh`.

## Global Constraints

- **No flight-code changes.** Do NOT modify `src/precision_land/`, `src/drogue_flight/`, `src/aruco_tracker/` or `src/ros2_yolo_image_processing/`. The blocks are *ports*: they copy the logic into new Python files.
- Do not modify `src/jl_mission/` C++ either. The Phase 2 contract is done. The only allowed `jl_mission` change is Task 12's move of shared shell functions out of `test/sitl_jl_mission.sh`.
- **Do not commit.** The maintainer makes all commits. Each task ends with a checkpoint: stop and report the diff. Never run `git add/commit/stash/checkout/reset/restore`.
- Work only on `main`, and never add yourself to commit history.
- `jl_blocks.core` and `jl_blocks.library` must not import ROS, numpy, or anything beyond the standard library and `yaml`. Only `jl_blocks/ros/` and `launch/` may import ROS.
- `make check` must pass after every task (ruff check, ruff format --check, ty, pytest, `jl_blocks check missions/*.yaml`). Run it from the repo root on the host.
- Topics and services per mission `NAME`:
  - from Phase 2:
    - `/jl/NAME/setpoint` (`px4_msgs/TrajectorySetpoint`, sensor-data QoS, NaN = not controlled)
    - `/jl/NAME/active` (`std_msgs/Bool`, transient-local, depth 1)
    - `/jl/NAME/mode_state` (jl_mission's own state)
    - `/jl/NAME/takeoff` (`jl_mission_interfaces/srv/Takeoff`, `float32 height`)
    - `/jl/NAME/land` (`std_srvs/srv/Trigger`)
  - new here, owned by the runner:
    - `/jl/NAME/state` (`std_msgs/String`, transient-local): the current step name, `idle`, `finished`, `aborted` or `inactive`;
    - `/jl/NAME/events` (`std_msgs/String`): one line per step event, the "why it ended" from spec §5.
- The `jl_mission` safety limits the blocks must live within (spec §5 defaults):
  - `max_step_m` 5.0: a position setpoint more than 5 m from the vehicle is rejected;
  - `max_speed` 1.0 m/s;
  - `silence_hold_s` 0.5, `silence_land_s` 5.0.
- Mission names: 1–24 characters, `^[A-Za-z][A-Za-z0-9_]{0,23}$` (the PX4 external-mode name limit; jl_mission enforces the same rule).
- Block and parameter names follow spec §3/§4 where the spec names them (`takeoff: {height}`, `search: {pattern}`, `track: {standoff}`, `pid: {kp, ki, kd, max_speed}`, `aruco_tag: {camera}`). Units go in a trailing comment on the `Params` field.
- Style: `from __future__ import annotations` at the top of every new module, and double quotes. The code below is already formatted; if `ruff format` changes something, keep its version.
- Build and test ROS pieces inside the sim container:
  `docker exec --user user -w /hdd/Documents/Electrical/Matrix_Lab/jacob_drone_ws/src/Jacob_Ladder jacob_ladder_sim bash -c 'source /opt/ros/humble/setup.bash && ...'`
  If the container isn't running, start it with `docker start jacob_ladder_sim`.

## Review Focus

These are the inputs a real user will hit that the spec implies but doesn't spell out. Each one has a test in the owning task.

1. **The pilot re-selects the mission quickly in QGC.** `/active` is latched with depth 1, so false→true can arrive as a single `true`. Every `true` must restart the mission from step 1, never continue a half-finished one. Owner: Task 9, `test_every_activation_starts_again_from_step_one`.
2. **A takeoff/land reply arrives after its step has already ended** (step timeout, `on_fail`, or a re-activation). It must be ignored and must never advance or fail the new step. Owners: Task 1, `test_a_reply_for_a_step_that_already_ended_is_ignored`; Task 9, `test_a_reply_from_before_a_reactivation_is_ignored`.
3. **A camera detection arrives before the vehicle's position/attitude are known, or contains NaN.** It must be dropped. It must never be placed using a default identity attitude. Owners: Task 6, `test_bad_detections_and_unknown_attitude_are_ignored`; Task 9, `test_a_detection_before_the_vehicle_state_is_known_is_dropped`.
4. **A search or track starts far from the NED origin, or the target is far away.** Every position setpoint must stay well inside `jl_mission`'s 5 m `max_step_m`, or the mode rejects it and holds. Owners: Task 4, `test_spiral_is_centred_on_where_the_step_started_not_the_origin` and `test_spiral_waypoints_are_close_together`; Task 7, `test_a_far_target_never_commands_a_big_jump`.
5. **A mission file that would only fail in the air:** a 25-character name (PX4 refuses to register), `.nan`/`.inf` numbers, or a `track` with no target. `jl_blocks check` must catch each one. Owner: Task 2.

---

## File Structure

```
src/jl_blocks/
├── jl_blocks/
│   ├── core/
│   │   ├── types.py        + VehicleState.stamp, Action, ActionStatus
│   │   ├── blocks.py       + StepContext.action/action_message, Mission.needs_target,
│   │   │                     Target.topic()/observe(), Mission.step may return Action
│   │   ├── engine.py       + action channel, targets(), drain_events(), params copy,
│   │   │                     reset-failure names the owning step
│   │   ├── loader.py       + 24-char names, finite numbers, needs_target check
│   │   ├── geometry.py     NEW: tuple vector helpers, quaternion rotate
│   │   ├── session.py      NEW: Session, Request, Output, trajectory_fields
│   │   ├── plugins.py      NEW: load_block_files() for researchers' own blocks
│   │   └── __init__.py     + new exports
│   ├── library/
│   │   ├── takeoff.py, land.py        NEW: Action-based mission blocks
│   │   ├── search.py                  NEW: hold / spiral (PrecisionLand search)
│   │   ├── track.py                   NEW: standoff carrot (DroneSmoothPlanner)
│   │   ├── precision_descend.py       NEW: PrecisionLand descend
│   │   ├── pid.py                     NEW: FrontApproach PID
│   │   ├── camera.py                  NEW: CameraTarget base (detections -> NED)
│   │   ├── aruco_tag.py, yolo_drogue.py  NEW: target blocks
│   │   └── __init__.py                + imports
│   ├── ros/
│   │   ├── __init__.py
│   │   └── mission_runner.py          NEW: the thin ROS node
│   └── cli.py              + --blocks
├── launch/mission.launch.py           NEW: jl_mission + mission_runner for one file
├── package.xml, setup.py              + ROS deps, launch, entry point
└── test/
    ├── conftest.py                    + asker, mutator, badtarget, seeker, grumpy fakes
    ├── test_actions.py                NEW (Task 1)
    ├── test_loader.py                 + Task 2 tests
    ├── test_takeoff_land.py, test_search.py, test_pid.py, test_targets.py,
    │   test_track.py, test_precision_descend.py, test_session.py, test_plugins.py
    └── test_geometry.py
missions/
├── takeoff_hold_land.yaml             NEW: flown by the SITL smoke test
└── track_moving_aruco.yaml            NEW: the spec §4 example
test/
├── sitl_common.sh                     NEW: start_px4 / kill_flight, shared
└── sitl_runner_smoke.sh               NEW: fly takeoff_hold_land.yaml headless
src/jl_mission/test/sitl_jl_mission.sh sources test/sitl_common.sh (no behaviour change)
Makefile                               ty excludes ros/; sitl-test runs the smoke flight too
docs/superpowers/specs/...design.md    §4 example, §11 items closed
```

---

### Task 1: Engine action channel and the Phase 1 carry-overs

**Files:**
- Modify: `src/jl_blocks/jl_blocks/core/types.py`, `src/jl_blocks/jl_blocks/core/blocks.py`, `src/jl_blocks/jl_blocks/core/engine.py`, `src/jl_blocks/jl_blocks/core/__init__.py`, `src/jl_blocks/test/conftest.py`
- Test: `src/jl_blocks/test/test_actions.py` (new)

**Interfaces:**
- Consumes: the Phase 1 core (`Engine`, `StepContext`, `Registry`, `parse_mission`).
- Produces:
  - `Action(kind: str, height: float = 0.0)`, a frozen dataclass; `kind` is `"takeoff"` or `"land"`, anything else raises `ValueError`.
  - `ActionStatus` enum: `NONE`, `PENDING`, `SUCCEEDED`, `FAILED`.
  - `VehicleState.stamp: float = 0.0`: seconds, on the same clock as the engine's `now`.
  - `StepContext.action: ActionStatus = ActionStatus.NONE` and `StepContext.action_message: str = ""`.
  - `Mission.step(ctx) -> Setpoint | Action | None`.
  - `Mission.needs_target: ClassVar[bool] = False`.
  - `Target.topic(self) -> str | None` (returns `None`) and `Target.observe(self, position: Vector3, vehicle: VehicleState) -> None` (does nothing). Both are declared here so later tasks only override them.
  - `Engine.take_action() -> tuple[int, Action] | None`.
  - `Engine.action_done(entry: int, success: bool, message: str) -> None`.
  - `Engine.targets() -> list[Target]`.
  - `Engine.drain_events() -> list[str]`.
- Event strings: `"<step>: requested <kind>"`, `"<step>: <kind> ok (<message>)"`, `"<step>: <kind> failed (<message>)"`. A failed action fails the step with the reason `"<kind> failed: <message>"`.

This also closes these Phase 1 deferred minors (from `.superpowers/sdd/2026-09-21-mission-blocks-phase1-core/progress.md`):
- each `Engine` now gets its own copy of every block's params (`copy.deepcopy`), which matters now that the runner rebuilds the engine on every activation;
- events can be drained, so they don't grow forever over a long flight;
- a target `reset()` failure in `start()` names the step that owns the target, not step 0.

- [ ] **Step 1: Add the fake blocks to the test registry**

In `src/jl_blocks/test/conftest.py`:
- add `Action` and `ActionStatus` to the `from jl_blocks.core import (...)` list;
- add these classes inside `make_registry()`, after `Counter`:

```python
    class Asker(Mission):
        """Asks the executor for `kind` once, then reports the reply."""

        @dataclass
        class Params:
            kind: str = "takeoff"

        def step(self, ctx: StepContext) -> Action | None:
            if ctx.action is ActionStatus.NONE:
                return Action(self.params.kind, 1.5)
            return None

        def status(self, ctx: StepContext) -> Status:
            if ctx.action is ActionStatus.SUCCEEDED:
                return Status.DONE
            if ctx.action is ActionStatus.FAILED:
                return Status.FAILED
            return Status.RUNNING

    class Mutator(Mission):
        """Changes its own params, to prove each engine gets its own copy."""

        @dataclass
        class Params:
            x: float = 1.0

        def step(self, ctx: StepContext) -> Setpoint:
            self.params.x += 1.0
            return Setpoint(position=(self.params.x, 0.0, -1.0))

    class BadTarget(Target):
        def reset(self) -> None:
            raise RuntimeError("target reset boom")

        def estimate(self, vehicle: VehicleState) -> tuple[float, float, float] | None:
            return None

    class Seeker(Mission):
        """Needs a target, so the loader must insist on one."""

        needs_target = True

        def step(self, ctx: StepContext) -> Setpoint:
            return Setpoint(position=(0.0, 0.0, -1.0))

    class Grumpy(Target):
        """Listens on a topic and raises on every detection."""

        def topic(self) -> str | None:
            return "/grumpy"

        def observe(self, position: tuple[float, float, float], vehicle: VehicleState) -> None:
            raise RuntimeError("grumpy")

        def estimate(self, vehicle: VehicleState) -> tuple[float, float, float] | None:
            return None
```

Then add them to the registration list:

```python
        ("asker", Asker),
        ("mutator", Mutator),
        ("badtarget", BadTarget),
        ("seeker", Seeker),
        ("grumpy", Grumpy),
```

- [ ] **Step 2: Write the failing tests**

Create `src/jl_blocks/test/test_actions.py`:

```python
"""The action channel: a mission block asks the executor to take off or land."""

from __future__ import annotations

import pytest

from jl_blocks.core import Action, Engine, VehicleState, parse_mission

DT = 0.02
HERE = VehicleState(position_ned=(0.0, 0.0, -1.0))


def started(text, registry):
    engine = Engine(parse_mission(text, registry), registry)
    engine.start(0.0)
    return engine


def test_an_action_is_handed_out_once(registry):
    engine = started("name: M\nsteps:\n  - asker: {}\n", registry)
    assert engine.tick(HERE, DT, DT) is None
    taken = engine.take_action()
    assert taken is not None
    assert taken[1] == Action("takeoff", 1.5)
    engine.tick(HERE, 2 * DT, DT)
    assert engine.take_action() is None


def test_success_finishes_the_step(registry):
    engine = started("name: M\nsteps:\n  - asker: {}\n  - goto: {}\n", registry)
    engine.tick(HERE, DT, DT)
    entry, _ = engine.take_action()
    engine.action_done(entry, True, "reached 1.50 m")
    engine.tick(HERE, 2 * DT, DT)
    assert engine.state == "goto"
    assert "asker: takeoff ok (reached 1.50 m)" in engine.events


def test_failure_fails_the_step_with_the_executor_message(registry):
    text = (
        "name: M\nsteps:\n  - asker: {}\n    on_fail: rescue\n"
        "  - goto: {}\n    name: rescue\n"
    )
    engine = started(text, registry)
    engine.tick(HERE, DT, DT)
    entry, _ = engine.take_action()
    engine.action_done(entry, False, "arming denied")
    engine.tick(HERE, 2 * DT, DT)
    assert engine.state == "rescue"
    assert "asker: failed (takeoff failed: arming denied) -> rescue" in engine.events


def test_a_reply_for_a_step_that_already_ended_is_ignored(registry):
    text = (
        "name: M\nsteps:\n  - asker: {}\n    timeout: 1\n    on_fail: again\n"
        "  - asker: {}\n    name: again\n"
    )
    engine = started(text, registry)
    engine.tick(HERE, DT, DT)
    old_entry, _ = engine.take_action()
    engine.tick(HERE, 1.0, DT)  # the timeout moves on to "again"
    assert engine.state == "again"
    engine.action_done(old_entry, True, "late")
    engine.tick(HERE, 1.02, DT)
    assert engine.state == "again"
    new_entry, _ = engine.take_action()
    assert new_entry != old_entry


def test_nothing_is_requested_after_the_mission_ends(registry):
    engine = started("name: M\nsteps:\n  - goto: {ticks: 1}\n", registry)
    engine.tick(HERE, DT, DT)
    assert engine.finished
    assert engine.take_action() is None


def test_each_engine_gets_its_own_copy_of_the_params(registry):
    spec = parse_mission("name: M\nsteps:\n  - mutator: {}\n", registry)
    first = Engine(spec, registry)
    first.start(0.0)
    first.tick(HERE, DT, DT)
    first.tick(HERE, 2 * DT, DT)
    second = Engine(spec, registry)
    second.start(0.0)
    assert second.tick(HERE, DT, DT).position == (2.0, 0.0, -1.0)


def test_drain_events_returns_each_event_once(registry):
    engine = started("name: M\nsteps:\n  - goto: {ticks: 1}\n", registry)
    engine.tick(HERE, DT, DT)
    assert engine.drain_events() == ["goto: started", "goto: done", "mission finished"]
    assert engine.drain_events() == []


def test_a_target_reset_failure_names_the_step_that_owns_the_target(registry):
    text = (
        "name: M\nsteps:\n  - goto: {}\n"
        "  - goto: {}\n    name: second\n    target: {badtarget: {}}\n"
    )
    engine = started(text, registry)
    assert engine.aborted
    assert engine.events[-1].startswith(
        "second: failed (reset: RuntimeError: target reset boom)"
    )


def test_targets_lists_each_distinct_target_once(registry):
    text = (
        "name: M\ntarget: {beacon: {}}\nsteps:\n  - goto: {}\n"
        "  - goto: {}\n    name: b\n"
        "  - goto: {}\n    name: c\n    target: {counter: {}}\n"
    )
    engine = Engine(parse_mission(text, registry), registry)
    assert len(engine.targets()) == 2


def test_vehicle_state_carries_a_timestamp():
    assert VehicleState(position_ned=(0.0, 0.0, 0.0), stamp=3.5).stamp == 3.5


def test_an_unknown_action_is_rejected():
    with pytest.raises(ValueError, match="takeoff or land"):
        Action("fly")
```

- [ ] **Step 3: Run the tests and watch them fail**

Run: `make check`
Expected: FAIL. The import of `Action` from `jl_blocks.core` fails, so conftest can't load.

- [ ] **Step 4: Add the types**

In `src/jl_blocks/jl_blocks/core/types.py`, add a field at the end of `VehicleState`:

```python
    # When this state was sampled, in seconds on the same clock as the engine's
    # `now`, so a target can tell how old a detection is.
    stamp: float = 0.0
```

Append to the same file:

```python
ACTION_KINDS = ("takeoff", "land")


@dataclass(frozen=True)
class Action:
    """A request for the executor instead of a setpoint: PX4's own takeoff or land.

    A mission block returns one from step(); the runner calls jl_mission and the
    reply shows up in the next StepContext.action.
    """

    kind: str
    height: float = 0.0  # m above the ground; takeoff only

    def __post_init__(self) -> None:
        if self.kind not in ACTION_KINDS:
            raise ValueError(f"an Action is takeoff or land, got {self.kind!r}")


class ActionStatus(enum.Enum):
    NONE = "none"  # this step hasn't asked for anything
    PENDING = "pending"  # asked, no reply yet
    SUCCEEDED = "succeeded"
    FAILED = "failed"
```

- [ ] **Step 5: Extend the block base classes**

In `src/jl_blocks/jl_blocks/core/blocks.py`:

Change the types import to:

```python
from .types import Action, ActionStatus, Setpoint, Status, Vector3, VehicleState
```

In `Target`, after `estimate`, add:

```python
    def topic(self) -> str | None:
        """The geometry_msgs/PoseStamped topic the runner feeds to observe(), or None."""
        return None

    def observe(self, position: Vector3, vehicle: VehicleState) -> None:
        """One detection, in the camera's own frame, and the vehicle state when it arrived."""
```

Add two fields at the end of `StepContext`:

```python
    # This step's executor request so far (see Action), and the executor's reply.
    action: ActionStatus = ActionStatus.NONE
    action_message: str = ""
```

In `Mission`, add the class attribute and change `step`'s return type:

```python
    kind = "mission"
    # True for blocks that make no sense without a target (track, precision_descend);
    # the loader then insists on one.
    needs_target: ClassVar[bool] = False

    def step(self, ctx: StepContext) -> Setpoint | Action | None:
        raise NotImplementedError
```

- [ ] **Step 6: The engine**

In `src/jl_blocks/jl_blocks/core/engine.py`:

Change the imports:

```python
import copy
import traceback
from dataclasses import dataclass
from typing import TypeVar

from .blocks import REGISTRY, Block, Controller, Mission, Registry, StepContext, Target
from .loader import BlockSpec, MissionSpec, StepSpec
from .types import Action, ActionStatus, Setpoint, Status, VehicleState
```

In `_build`, give every block its own params copy:

```python
        return cls(copy.deepcopy(spec.params))
```

In `__init__`, after `self.last_error = None`, add:

```python
        # Bumped on every step entry, so a late executor reply for a step that
        # has already ended can be recognised and ignored.
        self._entry = 0
        self._action: Action | None = None
        self._action_sent = False
        self._action_status = ActionStatus.NONE
        self._action_message = ""
```

Add these public methods after `start()`:

```python
    def take_action(self) -> tuple[int, Action] | None:
        """The current step's executor request, handed out once, with its step entry."""
        if (
            self._action is None
            or self._action_sent
            or self.finished
            or self.aborted
        ):
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
        self._action_status = (
            ActionStatus.SUCCEEDED if success else ActionStatus.FAILED
        )
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
```

In `tick()`, build the context with the action state, and handle an `Action` result. Replace the body of the `try:` block with:

```python
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
```

In the same method, replace the `if status is Status.FAILED:` branch with:

```python
        if status is Status.FAILED:
            reason = "block reported failure"
            if self._action is not None and self._action_status is ActionStatus.FAILED:
                reason = f"{self._action.kind} failed: {self._action_message}"
            self._fail(step, reason, vehicle, now)
```

Replace `_reset_all_targets` so a failure names the owning step:

```python
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
```

At the top of `_enter`, reset the action state:

```python
        self._entry += 1
        self._action = None
        self._action_sent = False
        self._action_status = ActionStatus.NONE
        self._action_message = ""
```

- [ ] **Step 7: Export the new names**

In `src/jl_blocks/jl_blocks/core/__init__.py`:
- change the types import to `from .types import Action, ActionStatus, Setpoint, Status, Vector3, VehicleState`;
- add `"Action"` and `"ActionStatus"` to `__all__`, keeping it sorted.

- [ ] **Step 8: Run the tests and watch them pass**

Run: `make check`
Expected: every stage passes, including all the Phase 1 tests and the 11 new ones in `test_actions.py`.

- [ ] **Step 9: Checkpoint**

Stop and report the diff (`git status --short` plus the new test count) to the maintainer. Do not commit.

---

### Task 2: Loader: 24-character names, finite numbers, blocks that need a target

**Files:**
- Modify: `src/jl_blocks/jl_blocks/core/loader.py`
- Test: `src/jl_blocks/test/test_loader.py`

**Interfaces:**
- Consumes: `Mission.needs_target` (Task 1), the fake `seeker` block (Task 1 conftest).
- Produces: loader error messages that later tasks' tests match:
  - name rule: starts `name must start with a letter` and contains `at most 24 characters`;
  - blocks needing a target: `<block> needs a target (at the top or on this step)`.

- [ ] **Step 1: Write the failing tests**

Append to `src/jl_blocks/test/test_loader.py` (add `import pytest` and `from jl_blocks.core import MissionError, parse_mission` at the top only if they're not there already):

```python
def errors_of(text, registry):
    with pytest.raises(MissionError) as caught:
        parse_mission(text, registry)
    return caught.value.errors


def test_a_name_longer_than_24_characters_is_rejected(registry):
    errs = errors_of("name: " + "A" * 25 + "\nsteps:\n  - goto: {}\n", registry)
    assert "at most 24 characters" in errs[0]


def test_a_name_of_exactly_24_characters_is_fine(registry):
    spec = parse_mission("name: " + "A" * 24 + "\nsteps:\n  - goto: {}\n", registry)
    assert len(spec.name) == 24


def test_nan_is_not_a_number(registry):
    errs = errors_of("name: M\nsteps:\n  - goto: {x: .nan}\n", registry)
    assert "param 'x' for goto should be float, got nan" in errs[0]


def test_infinity_is_not_a_timeout(registry):
    errs = errors_of("name: M\nsteps:\n  - goto: {}\n    timeout: .inf\n", registry)
    assert "timeout must be a positive number" in errs[0]


def test_a_block_that_needs_a_target_says_so(registry):
    errs = errors_of("name: M\nsteps:\n  - seeker: {}\n", registry)
    assert "seeker needs a target (at the top or on this step)" in errs[0]


def test_a_top_level_target_satisfies_needs_target(registry):
    parse_mission("name: M\ntarget: {beacon: {}}\nsteps:\n  - seeker: {}\n", registry)
```

- [ ] **Step 2: Run the tests and watch them fail**

Run: `make check`
Expected: FAIL. Four tests fail: the 25-character name, nan, infinity and needs-target cases.

- [ ] **Step 3: Implement**

In `src/jl_blocks/jl_blocks/core/loader.py`:
- add `import math` to the imports;
- change the pattern to `NAME_PATTERN = re.compile(r"^[A-Za-z][A-Za-z0-9_]{0,23}$")`;
- replace `_is_number` with:

```python
def _is_number(value: object) -> bool:
    return (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(value)
    )
```

In `_Parser.mission`, replace the name error message with:

```python
            self.error(
                "name",
                "name must start with a letter, use only letters, digits and _, "
                "and be at most 24 characters (PX4's limit for a mode name) "
                f"(got {name!r})",
            )
```

In the same method, inside the `for step in steps:` loop, after the `until` target check, add:

```python
            block_cls = self.registry.get(step.block.name)
            if (
                getattr(block_cls, "needs_target", False)
                and step.target is None
                and target is None
            ):
                self.error(
                    f"step '{step.name}'",
                    f"{step.block.name} needs a target (at the top or on this step)",
                )
```

- [ ] **Step 4: Run the tests and watch them pass**

Run: `make check`
Expected: all pass. The existing `assert "name must start with a letter" in errs[0]` test still passes, because the message keeps that prefix.

- [ ] **Step 5: Checkpoint**

Stop and report the diff to the maintainer. Do not commit.

---

### Task 3: `takeoff` and `land` blocks

**Files:**
- Create: `src/jl_blocks/jl_blocks/library/takeoff.py`, `src/jl_blocks/jl_blocks/library/land.py`
- Modify: `src/jl_blocks/jl_blocks/library/__init__.py`
- Test: `src/jl_blocks/test/test_takeoff_land.py`

**Interfaces:**
- Consumes: `Action`, `ActionStatus`, `StepContext.action` (Task 1).
- Produces:
  - block `takeoff` with `Params.height: float = 1.5` (must be > 0 and ≤ 10). It returns `Action("takeoff", height)` until the executor replies, then is DONE on success or FAILED on failure.
  - block `land` (no params): the same, with `Action("land")`.

These port the arm → `takeoff()` and `land()` calls of the `TakeoffLand`/`TakeoffHold` executors. `jl_mission`'s executor makes the actual PX4 calls. A takeoff while already airborne is answered as success by `jl_mission` ("already airborne; relaying"), so re-selecting a mission in the air works.

- [ ] **Step 1: Write the failing tests**

Create `src/jl_blocks/test/test_takeoff_land.py`:

```python
from __future__ import annotations

import pytest

from jl_blocks import library  # noqa: F401  (registers the shipped blocks)
from jl_blocks.core import Action, Engine, MissionError, VehicleState, parse_mission

DT = 0.02
GROUND = VehicleState(position_ned=(0.0, 0.0, 0.0), landed=True)
AIR = VehicleState(position_ned=(0.0, 0.0, -2.0))


def started(text):
    engine = Engine(parse_mission(text))
    engine.start(0.0)
    return engine


def test_takeoff_asks_for_the_height_then_moves_on_when_it_is_reached():
    engine = started("name: M\nsteps:\n  - takeoff: {height: 2.0}\n  - hold: {}\n")
    assert engine.tick(GROUND, DT, DT) is None
    entry, action = engine.take_action()
    assert action == Action("takeoff", 2.0)
    engine.action_done(entry, True, "reached 2.00 m")
    engine.tick(AIR, 2 * DT, DT)
    assert engine.state == "hold"


def test_takeoff_default_height_is_one_and_a_half_metres():
    engine = started("name: M\nsteps:\n  - takeoff: {}\n")
    engine.tick(GROUND, DT, DT)
    assert engine.take_action()[1] == Action("takeoff", 1.5)


def test_a_failed_takeoff_fails_the_step():
    engine = started("name: M\nsteps:\n  - takeoff: {}\n")
    engine.tick(GROUND, DT, DT)
    entry, _ = engine.take_action()
    engine.action_done(entry, False, "arming denied")
    engine.tick(GROUND, 2 * DT, DT)
    assert engine.aborted
    assert "takeoff: failed (takeoff failed: arming denied)" in engine.events[-1]


@pytest.mark.parametrize("height", [0, -1, 10.5])
def test_takeoff_height_must_be_sensible(height):
    with pytest.raises(MissionError, match="height"):
        parse_mission(f"name: M\nsteps:\n  - takeoff: {{height: {height}}}\n")


def test_land_asks_to_land_and_ends_the_mission():
    engine = started("name: M\nsteps:\n  - land: {}\n")
    engine.tick(AIR, DT, DT)
    entry, action = engine.take_action()
    assert action == Action("land")
    engine.action_done(entry, True, "landed")
    engine.tick(GROUND, 2 * DT, DT)
    assert engine.finished
```

- [ ] **Step 2: Run the tests and watch them fail**

Run: `make check`
Expected: FAIL: `unknown mission 'takeoff'`.

- [ ] **Step 3: Implement**

Create `src/jl_blocks/jl_blocks/library/takeoff.py`:

```python
"""takeoff: arm and take off with PX4's own takeoff, to `height` above the ground.

The step is done once jl_mission reports the drone is at that height. Port of
the arm -> takeoff() calls in the TakeoffLand / TakeoffHold executors.
"""

from __future__ import annotations

from dataclasses import dataclass

from ..core import Action, ActionStatus, Mission, Status, StepContext, block


@block("takeoff")
class Takeoff(Mission):
    @dataclass
    class Params:
        height: float = 1.5  # m above the ground

        def __post_init__(self) -> None:
            if not 0 < self.height <= 10:
                raise ValueError(
                    f"height must be more than 0 and at most 10 m (got {self.height})"
                )

    def step(self, ctx: StepContext) -> Action | None:
        if ctx.action is ActionStatus.NONE:
            return Action("takeoff", self.params.height)
        return None

    def status(self, ctx: StepContext) -> Status:
        if ctx.action is ActionStatus.SUCCEEDED:
            return Status.DONE
        if ctx.action is ActionStatus.FAILED:
            return Status.FAILED
        return Status.RUNNING
```

Create `src/jl_blocks/jl_blocks/library/land.py`:

```python
"""land: land with PX4's own land mode. Done once the drone is on the ground."""

from __future__ import annotations

from ..core import Action, ActionStatus, Mission, Status, StepContext, block


@block("land")
class Land(Mission):
    def step(self, ctx: StepContext) -> Action | None:
        if ctx.action is ActionStatus.NONE:
            return Action("land")
        return None

    def status(self, ctx: StepContext) -> Status:
        if ctx.action is ActionStatus.SUCCEEDED:
            return Status.DONE
        if ctx.action is ActionStatus.FAILED:
            return Status.FAILED
        return Status.RUNNING
```

Replace `src/jl_blocks/jl_blocks/library/__init__.py` with:

```python
"""Blocks shipped with Jacob's Ladder. Importing this package registers them."""

from . import hold, land, position, takeoff

__all__ = ["hold", "land", "position", "takeoff"]
```

- [ ] **Step 4: Run the tests and watch them pass**

Run: `make check`
Expected: all pass.

- [ ] **Step 5: Checkpoint**

Stop and report the diff to the maintainer. Do not commit.

---

### Task 4: Geometry helpers and the `search` block

**Files:**
- Create: `src/jl_blocks/jl_blocks/core/geometry.py`, `src/jl_blocks/jl_blocks/library/search.py`
- Modify: `src/jl_blocks/jl_blocks/library/__init__.py`
- Test: `src/jl_blocks/test/test_geometry.py`, `src/jl_blocks/test/test_search.py`

**Interfaces:**
- Consumes: the core types.
- Produces:
  - `jl_blocks.core.geometry`, imported by later tasks as `from ..core.geometry import ...`:
    - `Quaternion = tuple[float, float, float, float]` (w, x, y, z);
    - `add(a, b)`, `sub(a, b)`, `scale(a, k) -> Vector3`;
    - `norm(a) -> float`;
    - `is_finite(values) -> bool`;
    - `valid_quaternion(q) -> bool`;
    - `rotate(q, v) -> Vector3` (rotates `v` by the unit quaternion `q`; body FRD → NED when `q` is PX4's attitude).
  - Block `search`:
    - `Params`: `pattern: str = "hold"` (`hold` or `spiral`), `radius: float = 2.0`, `points: int = 16`, `reach: float = 0.2`;
    - a read-only `waypoints: tuple[Vector3, ...]` property, filled on the first `step()`;
    - status is always RUNNING, so a mission ends it with `until: target_seen` and/or `timeout`.

`pattern: hold` ports DroneSmoothPlanner/FrontApproach's Search, which holds and waits. `pattern: spiral` ports `PrecisionLand::generateSearchWaypoints` (spiral out to 2 m in 16 points, then back in) with **one deliberate change**: the spiral is centred on where the step started, not on the NED origin. The original only works because PrecisionLand takes off above its pad. Far from the origin, the first waypoint could be more than `jl_mission`'s 5 m `max_step_m` away and would be rejected. Both patterns hold the heading the step started with, so a front camera keeps looking the same way.

- [ ] **Step 1: Write the failing tests**

Create `src/jl_blocks/test/test_geometry.py`:

```python
from __future__ import annotations

import math

import pytest

from jl_blocks.core.geometry import (
    add,
    is_finite,
    norm,
    rotate,
    scale,
    sub,
    valid_quaternion,
)

LEVEL = (1.0, 0.0, 0.0, 0.0)
YAW_90 = (math.cos(math.pi / 4), 0.0, 0.0, math.sin(math.pi / 4))


def test_vector_arithmetic():
    assert add((1.0, 2.0, 3.0), (1.0, 1.0, 1.0)) == (2.0, 3.0, 4.0)
    assert sub((1.0, 2.0, 3.0), (1.0, 1.0, 1.0)) == (0.0, 1.0, 2.0)
    assert scale((1.0, 2.0, 3.0), 2.0) == (2.0, 4.0, 6.0)
    assert norm((3.0, 4.0, 0.0)) == 5.0


def test_level_attitude_leaves_a_vector_alone():
    assert rotate(LEVEL, (1.0, 2.0, 3.0)) == pytest.approx((1.0, 2.0, 3.0))


def test_facing_east_turns_forward_into_east():
    assert rotate(YAW_90, (3.0, 0.0, 0.0)) == pytest.approx((0.0, 3.0, 0.0))


def test_an_unnormalised_quaternion_still_rotates_correctly():
    doubled = tuple(2.0 * c for c in YAW_90)
    assert rotate(doubled, (3.0, 0.0, 0.0)) == pytest.approx((0.0, 3.0, 0.0))


def test_finite_and_quaternion_checks():
    assert is_finite((1.0, 2.0, 3.0))
    assert not is_finite((1.0, math.nan, 3.0))
    assert not is_finite((math.inf, 0.0, 0.0))
    assert valid_quaternion(LEVEL)
    assert not valid_quaternion((0.0, 0.0, 0.0, 0.0))
    assert not valid_quaternion((math.nan, 0.0, 0.0, 1.0))
```

Create `src/jl_blocks/test/test_search.py`:

```python
from __future__ import annotations

import math

import pytest

from jl_blocks import library  # noqa: F401  (registers the shipped blocks)
from jl_blocks.core import (
    MissionError,
    Setpoint,
    StepContext,
    VehicleState,
    parse_mission,
)
from jl_blocks.library.position import Position
from jl_blocks.library.search import Search


def ctx(vehicle):
    return StepContext(
        vehicle=vehicle,
        target=None,
        target_position=None,
        controller=Position(),
        elapsed=0.0,
        dt=0.02,
    )


def make(**params):
    search = Search(Search.Params(**params))
    search.reset()
    return search


def test_hold_pattern_keeps_the_start_position_and_heading():
    search = make()
    start = VehicleState(position_ned=(1.0, 2.0, -1.5), yaw=0.7)
    expected = Setpoint(position=(1.0, 2.0, -1.5), yaw=0.7)
    assert search.step(ctx(start)) == expected
    drifted = VehicleState(position_ned=(1.4, 2.0, -1.5), yaw=0.9)
    assert search.step(ctx(drifted)) == expected


def test_spiral_is_centred_on_where_the_step_started_not_the_origin():
    search = make(pattern="spiral")
    search.step(ctx(VehicleState(position_ned=(20.0, -30.0, -2.0))))
    assert search.waypoints[0] == (20.0, -30.0, -2.0)
    for x, y, z in search.waypoints:
        assert math.hypot(x - 20.0, y + 30.0) <= 2.0 + 1e-9
        assert z == -2.0


def test_spiral_waypoints_are_close_together():
    search = make(pattern="spiral")
    search.step(ctx(VehicleState(position_ned=(0.0, 0.0, -2.0))))
    points = list(search.waypoints)
    for a, b in zip(points, points[1:] + points[:1]):
        assert math.dist(a, b) < 1.0  # far inside jl_mission's 5 m max_step_m


def test_spiral_goes_out_then_back_in():
    search = make(pattern="spiral", points=4, radius=1.0)
    search.step(ctx(VehicleState(position_ned=(0.0, 0.0, -2.0))))
    radii = [round(math.hypot(x, y), 3) for x, y, _ in search.waypoints]
    assert radii == [0.0, 0.25, 0.5, 0.75, 1.0, 0.75, 0.5, 0.25]


def test_spiral_moves_on_only_once_the_drone_is_there_and_slow():
    search = make(pattern="spiral")
    search.step(ctx(VehicleState(position_ned=(0.0, 0.0, -2.0))))
    second = search.waypoints[1]
    moving = VehicleState(position_ned=second, velocity_ned=(0.5, 0.0, 0.0))
    assert search.step(ctx(moving)).position == second
    still = VehicleState(position_ned=second)
    assert search.step(ctx(still)).position == search.waypoints[2]


def test_spiral_repeats_forever():
    search = make(pattern="spiral", points=4, radius=1.0)
    here = (0.0, 0.0, -2.0)
    visited = []
    for _ in range(9):
        here = search.step(ctx(VehicleState(position_ned=here))).position
        visited.append(here)
    assert visited[7] == search.waypoints[0]
    assert visited[8] == search.waypoints[1]


def test_an_unknown_pattern_is_rejected_with_the_choices():
    with pytest.raises(MissionError, match="pattern must be 'hold' or 'spiral'"):
        parse_mission("name: M\nsteps:\n  - search: {pattern: zigzag}\n")
```

- [ ] **Step 2: Run the tests and watch them fail**

Run: `make check`
Expected: FAIL: `No module named 'jl_blocks.core.geometry'`.

- [ ] **Step 3: Implement the geometry helpers**

Create `src/jl_blocks/jl_blocks/core/geometry.py`:

```python
"""Small vector helpers for blocks: plain tuples, no numpy, so core stays light."""

from __future__ import annotations

import math
from collections.abc import Iterable

from .types import Vector3

# w, x, y, z, as PX4 reports attitude (body FRD -> NED)
Quaternion = tuple[float, float, float, float]


def add(a: Vector3, b: Vector3) -> Vector3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def sub(a: Vector3, b: Vector3) -> Vector3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def scale(a: Vector3, k: float) -> Vector3:
    return (a[0] * k, a[1] * k, a[2] * k)


def norm(a: Vector3) -> float:
    return math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2])


def is_finite(values: Iterable[float]) -> bool:
    return all(math.isfinite(v) for v in values)


def valid_quaternion(q: Quaternion) -> bool:
    """False for NaN or near-zero quaternions (e.g. attitude not received yet)."""
    return is_finite(q) and math.sqrt(sum(c * c for c in q)) >= 0.1


def rotate(q: Quaternion, v: Vector3) -> Vector3:
    """Rotate v by the (normalised) quaternion q."""
    length = math.sqrt(sum(c * c for c in q))
    w, x, y, z = (c / length for c in q)
    # v' = v + w*t + u x t, where u = (x, y, z) and t = 2 * (u x v)
    tx = 2.0 * (y * v[2] - z * v[1])
    ty = 2.0 * (z * v[0] - x * v[2])
    tz = 2.0 * (x * v[1] - y * v[0])
    return (
        v[0] + w * tx + (y * tz - z * ty),
        v[1] + w * ty + (z * tx - x * tz),
        v[2] + w * tz + (x * ty - y * tx),
    )
```

- [ ] **Step 4: Implement `search`**

Create `src/jl_blocks/jl_blocks/library/search.py`:

```python
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
                (start[0] + r * math.cos(angle), start[1] + r * math.sin(angle), start[2])
            )
        # Out to the full radius, then back in, without repeating either end.
        return out + out[-2:0:-1]
```

Add `search` to `src/jl_blocks/jl_blocks/library/__init__.py` (both the import and `__all__`, kept sorted).

- [ ] **Step 5: Run the tests and watch them pass**

Run: `make check`
Expected: all pass.

- [ ] **Step 6: Checkpoint**

Stop and report the diff to the maintainer. Do not commit.

---

### Task 5: `pid` controller

**Files:**
- Create: `src/jl_blocks/jl_blocks/library/pid.py`
- Modify: `src/jl_blocks/jl_blocks/library/__init__.py`
- Test: `src/jl_blocks/test/test_pid.py`

**Interfaces:**
- Consumes: `Controller`, `Setpoint`, `VehicleState`.
- Produces: block `pid`, `Params`:
  - `kp: float = 0.8`, `ki: float = 0.0`, `kd: float = 0.2`, `max_speed: float = 1.0`: the spec §3 defaults;
  - `integral_limit: float = 0.5`, `kp_z: float = 0.6`, `max_speed_z: float = 0.1`: FrontApproach's flown values from `src/precision_land/cfg/front_approach_params.yaml`.

  It turns a position setpoint into a velocity-only setpoint; one without a position passes through unchanged.

Port of `FrontApproach::updateSetpoint`'s Approach PID:
- the XY PID has an integral clamped per axis and a derivative of the error, and its XY speed is capped by magnitude;
- Z is P-only, clamped;
- yaw passes through.

The engine calls `reset()` at each step entry, which clears the integral like FrontApproach's `resetController()`.

- [ ] **Step 1: Write the failing tests**

Create `src/jl_blocks/test/test_pid.py`:

```python
from __future__ import annotations

import math

import pytest

from jl_blocks.core import Setpoint, VehicleState
from jl_blocks.library.pid import PID

DT = 0.02
AT = VehicleState(position_ned=(0.0, 0.0, -1.0))


def pid(**params):
    controller = PID(PID.Params(**params))
    controller.reset()
    return controller


def test_a_setpoint_without_a_position_passes_through():
    velocity_only = Setpoint(velocity=(0.1, 0.0, 0.0), yaw=0.3)
    assert pid().command(AT, velocity_only, DT) == velocity_only


def test_proportional_term_drives_toward_the_target():
    out = pid().command(AT, Setpoint(position=(1.0, 0.0, -1.0)), DT)
    assert out.position is None
    assert out.velocity == pytest.approx((0.8, 0.0, 0.0))


def test_horizontal_speed_is_capped_at_max_speed():
    out = pid().command(AT, Setpoint(position=(10.0, 10.0, -1.0)), DT)
    assert math.hypot(out.velocity[0], out.velocity[1]) == pytest.approx(1.0)


def test_vertical_speed_has_its_own_gain_and_cap():
    one_metre_up = pid().command(AT, Setpoint(position=(0.0, 0.0, -2.0)), DT)
    assert one_metre_up.velocity[2] == pytest.approx(-0.1)
    ten_cm_up = pid().command(AT, Setpoint(position=(0.0, 0.0, -1.1)), DT)
    assert ten_cm_up.velocity[2] == pytest.approx(-0.06)


def test_integral_is_limited_and_cleared_by_reset():
    controller = pid(kp=0.0, ki=1.0, kd=0.0, integral_limit=0.5)
    for _ in range(100):
        out = controller.command(AT, Setpoint(position=(1.0, 0.0, -1.0)), DT)
    assert out.velocity[0] == pytest.approx(0.5)
    controller.reset()
    out = controller.command(AT, Setpoint(position=(1.0, 0.0, -1.0)), DT)
    assert out.velocity[0] == pytest.approx(0.02)


def test_derivative_uses_the_change_in_error():
    controller = pid(kp=0.0, ki=0.0, kd=0.2, max_speed=5.0)
    first = controller.command(AT, Setpoint(position=(1.0, 0.0, -1.0)), DT)
    assert first.velocity[0] == 0.0
    second = controller.command(AT, Setpoint(position=(1.1, 0.0, -1.0)), DT)
    assert second.velocity[0] == pytest.approx(0.2 * 0.1 / DT)


def test_yaw_passes_through():
    out = pid().command(AT, Setpoint(position=(1.0, 0.0, -1.0), yaw=1.2), DT)
    assert out.yaw == 1.2
```

- [ ] **Step 2: Run the tests and watch them fail**

Run: `make check`
Expected: FAIL: `No module named 'jl_blocks.library.pid'`.

- [ ] **Step 3: Implement**

Create `src/jl_blocks/jl_blocks/library/pid.py`:

```python
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
```

Add `pid` to `library/__init__.py` (import and `__all__`, sorted).

- [ ] **Step 4: Run the tests and watch them pass**

Run: `make check`
Expected: all pass.

- [ ] **Step 5: Checkpoint**

Stop and report the diff to the maintainer. Do not commit.

---

### Task 6: Camera targets: `aruco_tag` and `yolo_drogue`

**Files:**
- Create: `src/jl_blocks/jl_blocks/library/camera.py`, `src/jl_blocks/jl_blocks/library/aruco_tag.py`, `src/jl_blocks/jl_blocks/library/yolo_drogue.py`
- Modify: `src/jl_blocks/jl_blocks/library/__init__.py`
- Test: `src/jl_blocks/test/test_targets.py`

**Interfaces:**
- Consumes:
  - `Target.topic()` / `Target.observe()` and `VehicleState.stamp` (Task 1);
  - `rotate`, `add`, `is_finite`, `valid_quaternion` (Task 4).
- Produces:
  - `CameraTarget(Target)` base:
    - `observe(position, vehicle)` stores `vehicle.position_ned + rotate(vehicle.attitude, to_body(position))` and the time `vehicle.stamp`;
    - `estimate(vehicle)` returns that NED point while `vehicle.stamp - seen_at <= params.max_age`, else `None`;
    - subclasses implement `to_body(position) -> Vector3` and `topic()`.
  - Block `aruco_tag`:
    - `Params`: `camera: str = "front"` (`front` or `down`), `max_age: float = 0.5`;
    - topic `/front/target_pose` (front) or `/target_pose` (down), as the `aruco_tracker` launch files remap them.
  - Block `yolo_drogue`:
    - `Params`: `camera_pitch_deg: float = 0.0`, `max_age: float = 0.5`;
    - topic `/tag_detections` (`pose_estimation_node`).

The camera → body transforms are ports:
- **front ArUco (OpenCV optical: +x right, +y down, +z forward):** body = (z, x, y), from the `_front_optical_to_body` matrix in `FrontApproach.cpp`;
- **down ArUco:** body = (−y, x, z), from the `R` matrix in `PrecisionLand::getTagWorld`;
- **YOLO ranging frame (+x left, +y up, +z forward):** body = (z, −x, −y), then tilted by `camera_pitch_deg` about body y, from `DroneSmoothPlanner::drogueTargetNed`.

**Two rulings to confirm with the maintainer at review:**
1. The spec §4 example gives `aruco_tag: {id: 0, camera: front, size_m: 0.15}`. The tag id and size are `aruco_tracker`'s own parameters, and the block can't act on them. A block param that silently does nothing is a trap for a beginner, so they are left out. Task 12 updates the spec example.
2. Detections are placed in NED when they arrive (FrontApproach's approach), not re-rotated with a later attitude (DroneSmoothPlanner's). A stored detection then stays correct while the drone turns. `max_age` (0.5 s) is how long one detection counts as "seen". The existing `Target.lost_after` (3 s) still decides `target_lost`.

- [ ] **Step 1: Write the failing tests**

Create `src/jl_blocks/test/test_targets.py`:

```python
from __future__ import annotations

import math

import pytest

from jl_blocks import library  # noqa: F401  (registers the shipped blocks)
from jl_blocks.core import MissionError, VehicleState, parse_mission
from jl_blocks.library.aruco_tag import ArucoTag
from jl_blocks.library.yolo_drogue import YoloDrogue

LEVEL = (1.0, 0.0, 0.0, 0.0)
YAW_90 = (math.cos(math.pi / 4), 0.0, 0.0, math.sin(math.pi / 4))


def at(position=(0.0, 0.0, -1.0), attitude=LEVEL, stamp=0.0):
    return VehicleState(position_ned=position, attitude=attitude, stamp=stamp)


def aruco(**params):
    target = ArucoTag(ArucoTag.Params(**params))
    target.reset()
    return target


def yolo(**params):
    target = YoloDrogue(YoloDrogue.Params(**params))
    target.reset()
    return target


def test_a_front_tag_straight_ahead_is_in_front_of_the_drone():
    tag = aruco()
    tag.observe((0.0, 0.0, 3.0), at())
    assert tag.estimate(at()) == pytest.approx((3.0, 0.0, -1.0))


def test_front_camera_right_and_down_become_east_and_down():
    tag = aruco()
    tag.observe((0.5, 0.2, 3.0), at())
    assert tag.estimate(at()) == pytest.approx((3.0, 0.5, -0.8))


def test_the_drone_heading_rotates_the_detection():
    tag = aruco()
    tag.observe((0.0, 0.0, 3.0), at(attitude=YAW_90))
    assert tag.estimate(at()) == pytest.approx((0.0, 3.0, -1.0))


def test_down_camera_axes():
    tag = aruco(camera="down")
    tag.observe((0.5, 0.2, 2.0), at())
    assert tag.estimate(at()) == pytest.approx((-0.2, 0.5, 1.0))


def test_the_camera_picks_the_topic():
    assert aruco().topic() == "/front/target_pose"
    assert aruco(camera="down").topic() == "/target_pose"
    assert yolo().topic() == "/tag_detections"


def test_a_detection_counts_for_max_age_seconds():
    tag = aruco()
    tag.observe((0.0, 0.0, 3.0), at(stamp=10.0))
    assert tag.estimate(at(stamp=10.4)) is not None
    assert tag.estimate(at(stamp=10.6)) is None


def test_the_estimate_keeps_where_the_tag_was_when_seen():
    tag = aruco()
    tag.observe((0.0, 0.0, 3.0), at(position=(0.0, 0.0, -1.0)))
    moved = at(position=(1.0, 0.0, -1.0), stamp=0.1)
    assert tag.estimate(moved) == pytest.approx((3.0, 0.0, -1.0))


def test_bad_detections_and_unknown_attitude_are_ignored():
    tag = aruco()
    tag.observe((math.nan, 0.0, 3.0), at())
    tag.observe((0.0, 0.0, 3.0), at(attitude=(0.0, 0.0, 0.0, 0.0)))
    tag.observe((0.0, 0.0, 3.0), at(position=(math.nan, 0.0, -1.0)))
    assert tag.estimate(at()) is None


def test_reset_forgets_the_last_detection():
    tag = aruco()
    tag.observe((0.0, 0.0, 3.0), at())
    tag.reset()
    assert tag.estimate(at()) is None


def test_an_unknown_camera_is_rejected():
    with pytest.raises(MissionError, match="camera must be 'front' or 'down'"):
        parse_mission(
            "name: M\ntarget: {aruco_tag: {camera: side}}\nsteps:\n  - hold: {}\n"
        )


def test_yolo_ranging_frame():
    drogue = yolo()
    drogue.observe((0.5, 0.2, 3.0), at())  # 0.5 m left, 0.2 m up, 3 m ahead
    assert drogue.estimate(at()) == pytest.approx((3.0, -0.5, -1.2))


def test_yolo_camera_pitch_tilts_the_ray_down():
    drogue = yolo(camera_pitch_deg=30.0)
    drogue.observe((0.0, 0.0, 2.0), at())
    assert drogue.estimate(at()) == pytest.approx(
        (2.0 * math.cos(math.radians(30)), 0.0, -1.0 + 2.0 * math.sin(math.radians(30)))
    )
```

- [ ] **Step 2: Run the tests and watch them fail**

Run: `make check`
Expected: FAIL: `No module named 'jl_blocks.library.aruco_tag'`.

- [ ] **Step 3: Implement**

Create `src/jl_blocks/jl_blocks/library/camera.py`:

```python
"""CameraTarget: the shared part of every camera-based target block.

Each detection arrives in the camera's own frame. It is turned into an NED
point immediately, using the vehicle state at that moment, and counts as
"seen" for max_age seconds. Detections with NaN, or before the vehicle's
position and attitude are known, are dropped.
"""

from __future__ import annotations

import math

from ..core import Target, Vector3, VehicleState
from ..core.geometry import add, is_finite, rotate, valid_quaternion


class CameraTarget(Target):
    def __init__(self, params: object | None = None) -> None:
        super().__init__(params)
        self.reset()

    def reset(self) -> None:
        self._ned: Vector3 | None = None
        self._seen_at = -math.inf

    def to_body(self, position: Vector3) -> Vector3:
        """Camera frame -> body FRD (forward, right, down)."""
        raise NotImplementedError

    def observe(self, position: Vector3, vehicle: VehicleState) -> None:
        if not (
            is_finite(position)
            and is_finite(vehicle.position_ned)
            and valid_quaternion(vehicle.attitude)
        ):
            return
        offset = rotate(vehicle.attitude, self.to_body(position))
        self._ned = add(vehicle.position_ned, offset)
        self._seen_at = vehicle.stamp

    def estimate(self, vehicle: VehicleState) -> Vector3 | None:
        if self._ned is None or vehicle.stamp - self._seen_at > self.params.max_age:
            return None
        return self._ned
```

Create `src/jl_blocks/jl_blocks/library/aruco_tag.py`:

```python
"""aruco_tag: an ArUco tag seen by aruco_tracker, on the front or down camera.

aruco_tracker reports the tag in OpenCV's optical frame (+x right, +y down,
+z out of the lens). The tag id and size are aruco_tracker's own parameters.
Front camera -> body: FrontApproach's _front_optical_to_body. Down camera ->
body: PrecisionLand::getTagWorld's R.
"""

from __future__ import annotations

from dataclasses import dataclass

from ..core import Vector3, block
from .camera import CameraTarget

TOPICS = {"front": "/front/target_pose", "down": "/target_pose"}


@block("aruco_tag")
class ArucoTag(CameraTarget):
    @dataclass
    class Params:
        camera: str = "front"  # front or down
        max_age: float = 0.5  # s a detection counts as "seen"

        def __post_init__(self) -> None:
            if self.camera not in TOPICS:
                raise ValueError(
                    f"camera must be 'front' or 'down', got {self.camera!r}"
                )
            if self.max_age <= 0:
                raise ValueError("max_age must be > 0")

    def topic(self) -> str | None:
        return TOPICS[self.params.camera]

    def to_body(self, position: Vector3) -> Vector3:
        x, y, z = position
        if self.params.camera == "front":
            return (z, x, y)
        return (-y, x, z)
```

Create `src/jl_blocks/jl_blocks/library/yolo_drogue.py`:

```python
"""yolo_drogue: the drogue from the YOLO pipeline's pose_estimation_node.

It publishes the drogue on /tag_detections in a ranging frame (+x left,
+y up, +z forward). Port of DroneSmoothPlanner::drogueTargetNed: ranging ->
body FRD is (z, -x, -y), then tilted by the camera's mount pitch.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from ..core import Vector3, block
from .camera import CameraTarget


@block("yolo_drogue")
class YoloDrogue(CameraTarget):
    @dataclass
    class Params:
        camera_pitch_deg: float = 0.0  # deg, positive = tilted down
        max_age: float = 0.5  # s a detection counts as "seen"

        def __post_init__(self) -> None:
            if self.max_age <= 0:
                raise ValueError("max_age must be > 0")

    def topic(self) -> str | None:
        return "/tag_detections"

    def to_body(self, position: Vector3) -> Vector3:
        x_left, y_up, z_forward = position
        fx, fy, fz = z_forward, -x_left, -y_up
        pitch = math.radians(self.params.camera_pitch_deg)
        return (
            math.cos(pitch) * fx - math.sin(pitch) * fz,
            fy,
            math.sin(pitch) * fx + math.cos(pitch) * fz,
        )
```

Add `aruco_tag` and `yolo_drogue` to `library/__init__.py` (import and `__all__`, sorted). Don't add `camera`: it registers nothing.

- [ ] **Step 4: Run the tests and watch them pass**

Run: `make check`
Expected: all pass.

- [ ] **Step 5: Checkpoint**

Stop and report the diff and the two rulings above to the maintainer. Do not commit.

---

### Task 7: `track` block (standoff carrot)

**Files:**
- Create: `src/jl_blocks/jl_blocks/library/track.py`
- Modify: `src/jl_blocks/jl_blocks/library/__init__.py`
- Test: `src/jl_blocks/test/test_track.py`

**Interfaces:**
- Consumes: `StepContext.target_position`, `Target.lost_after`, `Mission.needs_target` (Task 1), and the geometry helpers (Task 4).
- Produces: block `track`, `needs_target = True`. `Params`:
  - `standoff: float = 3.0` (> 0);
  - `lead_time: float = 1.0`;
  - `max_speed: float = 0.5`;
  - `tolerance: float = 0.25`.

  Status:
  - DONE while holding station within tolerance;
  - FAILED once the target has been unseen for `lost_after` s;
  - RUNNING otherwise.

  A mission that should keep tracking uses `until: never`.

Port of `DroneSmoothPlanner`'s Approach and Hover states (its defaults are from `src/drogue_flight/config/drone_smooth_planner.yaml`):
- The standoff point is `standoff` back from the target along the drone→target bearing, at the target's altitude.
- **Approach:** the setpoint is a carrot `min(max_speed·lead_time, distance)` ahead, with a velocity feed-forward tapered inside the lead distance and yaw facing the target.
- **Hover:** when every NED axis is within `tolerance`, command the standoff itself. Go back to Approach past 2× tolerance (hysteresis).
- **Target lost:** hold where it was lost, facing the same way. The original falls back to Search; here the step fails after `lost_after`, and the mission's `on_fail` (e.g. `search`) takes over.

- [ ] **Step 1: Write the failing tests**

Create `src/jl_blocks/test/test_track.py`:

```python
from __future__ import annotations

import math

import pytest

from jl_blocks import library  # noqa: F401  (registers the shipped blocks)
from jl_blocks.core import (
    MissionError,
    Setpoint,
    Status,
    StepContext,
    VehicleState,
    parse_mission,
)
from jl_blocks.library.position import Position
from jl_blocks.library.track import Track

TARGET = (10.0, 0.0, -2.0)  # standoff 3 m back along +x: (7, 0, -2)


def ctx(position, target=TARGET, elapsed=0.0, yaw=0.0):
    return StepContext(
        vehicle=VehicleState(position_ned=position, yaw=yaw),
        target=None,
        target_position=target,
        controller=Position(),
        elapsed=elapsed,
        dt=0.02,
    )


def make(**params):
    track = Track(Track.Params(**params))
    track.reset()
    return track


def test_approach_leads_toward_the_standoff_at_max_speed():
    track = make()
    out = track.step(ctx((0.0, 0.0, -1.0)))
    d = math.sqrt(50.0)  # distance to (7, 0, -2)
    assert out.position == pytest.approx((0.5 * 7 / d, 0.0, -1.0 - 0.5 / d))
    assert out.velocity == pytest.approx((0.5 * 7 / d, 0.0, -0.5 / d))
    assert out.yaw == pytest.approx(0.0)


def test_speed_tapers_inside_the_lead_distance():
    out = make().step(ctx((6.6, 0.0, -2.0)))
    assert out.position == pytest.approx((7.0, 0.0, -2.0))
    assert out.velocity == pytest.approx((0.4, 0.0, 0.0))


def test_inside_tolerance_it_holds_the_standoff_and_is_done():
    track = make()
    c = ctx((6.9, 0.0, -2.0))
    assert track.step(c) == Setpoint(position=(7.0, 0.0, -2.0), yaw=0.0)
    assert track.status(c) is Status.DONE


def test_hover_has_hysteresis():
    track = make()
    track.step(ctx((6.9, 0.0, -2.0)))  # arrived
    still_close = track.step(ctx((6.6, 0.0, -2.0)))  # 0.4 m out: still holding
    assert still_close.velocity is None
    too_far = track.step(ctx((6.4, 0.0, -2.0)))  # 0.6 m out: approach again
    assert too_far.velocity is not None


def test_too_close_it_backs_away():
    out = make().step(ctx((8.5, 0.0, -2.0)))
    assert out.position[0] < 8.5
    assert out.velocity == pytest.approx((-0.5, 0.0, 0.0))


def test_a_lost_target_holds_where_it_was_lost_then_fails():
    track = make()
    track.step(ctx((5.0, 0.0, -2.0), elapsed=0.0))
    lost = ctx((5.2, 0.0, -2.0), target=None, elapsed=1.0, yaw=0.3)
    assert track.step(lost) == Setpoint(position=(5.2, 0.0, -2.0), yaw=0.3)
    assert track.status(lost) is Status.RUNNING
    later = ctx((5.4, 0.0, -2.0), target=None, elapsed=3.0)
    assert track.step(later).position == (5.2, 0.0, -2.0)
    assert track.status(later) is Status.FAILED


def test_seeing_the_target_again_clears_the_loss():
    track = make()
    track.step(ctx((5.0, 0.0, -2.0), elapsed=0.0))
    track.step(ctx((5.0, 0.0, -2.0), target=None, elapsed=2.5))
    seen = ctx((5.0, 0.0, -2.0), elapsed=2.9)
    track.step(seen)
    assert track.status(ctx((5.0, 0.0, -2.0), target=None, elapsed=4.0)) is Status.RUNNING


def test_a_far_target_never_commands_a_big_jump():
    here = (0.0, 0.0, -1.0)
    out = make().step(ctx(here, target=(100.0, 50.0, -2.0)))
    assert math.dist(out.position, here) <= 0.5 + 1e-9


def test_directly_below_the_target_keeps_the_current_heading():
    out = make().step(ctx((0.0, 0.0, -1.0), target=(0.0, 0.0, -5.0), yaw=1.1))
    assert out.yaw == 1.1


def test_standoff_must_be_positive():
    with pytest.raises(MissionError, match="standoff"):
        parse_mission(
            "name: M\ntarget: {aruco_tag: {}}\nsteps:\n  - track: {standoff: 0}\n"
        )


def test_track_needs_a_target():
    with pytest.raises(MissionError, match="track needs a target"):
        parse_mission("name: M\nsteps:\n  - track: {}\n")
```

- [ ] **Step 2: Run the tests and watch them fail**

Run: `make check`
Expected: FAIL: `No module named 'jl_blocks.library.track'`.

- [ ] **Step 3: Implement**

Create `src/jl_blocks/jl_blocks/library/track.py`:

```python
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
```

Add `track` to `library/__init__.py` (import and `__all__`, sorted).

- [ ] **Step 4: Run the tests and watch them pass**

Run: `make check`
Expected: all pass.

- [ ] **Step 5: Checkpoint**

Stop and report the diff to the maintainer. Do not commit.

---

### Task 8: `precision_descend` block

**Files:**
- Create: `src/jl_blocks/jl_blocks/library/precision_descend.py`
- Modify: `src/jl_blocks/jl_blocks/library/__init__.py`
- Test: `src/jl_blocks/test/test_precision_descend.py`

**Interfaces:**
- Consumes: `StepContext.target_position`, `VehicleState.landed`, `Target.lost_after`.
- Produces: block `precision_descend`, `needs_target = True`. `Params`:
  - `descent_speed: float = 0.6`;
  - `kp: float = 1.7`;
  - `ki: float = 0.0`;
  - `max_speed: float = 1.0`.

  It sends velocity-only setpoints and holds its starting heading. Status:
  - DONE once `vehicle.landed`;
  - FAILED once the target has been unseen for `lost_after` s.

Port of `PrecisionLand`'s Descend state (`calculateVelocitySetpointXY`), with the flown values from `src/precision_land/cfg/params.yaml` (`descent_vel: 0.6`, `vel_p_gain: 1.7`, `vel_i_gain: 0.0`). Three differences, each noted in the file:
- `max_speed` defaults to 1.0, not the flown 3.0, because `jl_mission` clamps every velocity to its 1 m/s `max_speed` anyway.
- The integral uses `error·dt`; the original adds the raw error every tick. This is identical with the flown `ki = 0`.
- Yaw holds the heading at step start. The original yaws to the tag's orientation, and the target blocks report position only.

This is a downward-camera block. The real drone has a single front camera, so it is shipped for SITL and future airframes, and nothing here depends on it.

- [ ] **Step 1: Write the failing tests**

Create `src/jl_blocks/test/test_precision_descend.py`:

```python
from __future__ import annotations

import pytest

from jl_blocks import library  # noqa: F401  (registers the shipped blocks)
from jl_blocks.core import (
    MissionError,
    Setpoint,
    Status,
    StepContext,
    VehicleState,
    parse_mission,
)
from jl_blocks.library.position import Position
from jl_blocks.library.precision_descend import PrecisionDescend

PAD = (0.0, 0.0, 0.0)


def ctx(position, target=PAD, elapsed=0.0, landed=False, yaw=0.4):
    return StepContext(
        vehicle=VehicleState(position_ned=position, yaw=yaw, landed=landed),
        target=None,
        target_position=target,
        controller=Position(),
        elapsed=elapsed,
        dt=0.02,
    )


def make(**params):
    descend = PrecisionDescend(PrecisionDescend.Params(**params))
    descend.reset()
    return descend


def test_descends_while_steering_over_the_target():
    out = make().step(ctx((0.2, -0.1, -2.0)))
    assert out.position is None
    assert out.velocity == pytest.approx((-0.34, 0.17, 0.6))
    assert out.yaw == 0.4


def test_horizontal_speed_is_clamped_per_axis():
    out = make().step(ctx((2.0, 0.0, -2.0)))
    assert out.velocity[0] == pytest.approx(-1.0)


def test_heading_stays_where_the_step_started():
    descend = make()
    descend.step(ctx((0.2, 0.0, -2.0), yaw=0.4))
    assert descend.step(ctx((0.2, 0.0, -1.9), yaw=0.9)).yaw == 0.4


def test_done_once_landed():
    descend = make()
    c = ctx((0.0, 0.0, 0.0), landed=True)
    descend.step(c)
    assert descend.status(c) is Status.DONE


def test_a_lost_target_holds_then_fails():
    descend = make()
    descend.step(ctx((0.0, 0.0, -1.0), elapsed=0.0))
    lost = ctx((0.1, 0.0, -0.9), target=None, elapsed=1.0)
    assert descend.step(lost) == Setpoint(position=(0.1, 0.0, -0.9), yaw=0.4)
    assert descend.status(lost) is Status.RUNNING
    assert descend.status(ctx((0.1, 0.0, -0.9), target=None, elapsed=3.0)) is Status.FAILED


def test_needs_a_target():
    with pytest.raises(MissionError, match="precision_descend needs a target"):
        parse_mission("name: M\nsteps:\n  - precision_descend: {}\n")
```

- [ ] **Step 2: Run the tests and watch them fail**

Run: `make check`
Expected: FAIL: `No module named 'jl_blocks.library.precision_descend'`.

- [ ] **Step 3: Implement**

Create `src/jl_blocks/jl_blocks/library/precision_descend.py`:

```python
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
```

Add `precision_descend` to `library/__init__.py` (import and `__all__`, sorted).

- [ ] **Step 4: Run the tests and watch them pass**

Run: `make check`
Expected: all pass.

- [ ] **Step 5: Checkpoint**

Stop and report the diff to the maintainer. Do not commit.

---

### Task 9: `Session`: the runner's decisions, without ROS

**Files:**
- Create: `src/jl_blocks/jl_blocks/core/session.py`
- Modify: `src/jl_blocks/jl_blocks/core/__init__.py`
- Test: `src/jl_blocks/test/test_session.py`

**Interfaces:**
- Consumes (Task 1):
  - `Engine.take_action()`, `action_done()`, `targets()`, `drain_events()`;
  - `Engine.last_error`;
  - `Target.topic()` and `Target.observe()`.
- Produces (all exported from `jl_blocks.core`):
  - `Request(token: int, action: Action)`: frozen.
  - `Output(setpoint: Setpoint | None = None, requests: tuple[Request, ...] = ())`: frozen.
  - `trajectory_fields(setpoint) -> tuple[list[float], list[float], float]`: position, velocity and yaw with NaN for "not controlled".
  - `Session(spec: MissionSpec, registry: Registry = REGISTRY, abort_hold_s: float = 2.0)`, with:
    - `.vehicle: VehicleState | None`;
    - `.engine: Engine | None`;
    - `.state -> str` (`inactive` or the engine's state);
    - `.topics() -> list[str]`;
    - `.set_active(active: bool, now: float)`;
    - `.update_vehicle(vehicle: VehicleState)`;
    - `.observe(topic: str, position: Vector3)`;
    - `.tick(now: float, dt: float) -> Output`;
    - `.action_done(token: int, success: bool, message: str)`;
    - `.drain_events() -> list[str]`;
    - `.drain_errors() -> list[str]` (tracebacks).

Rules, and the reason for each:
- **Every `set_active(True)` builds a fresh `Engine` and starts from step 1, even without a `False` in between.** `/jl/NAME/active` is latched with depth 1, so a quick false/true reaches the runner as one `true`. This also resolves the Phase 2 "active merge" note without changing `jl_mission`.
- **Replies are matched by token.** Tokens are forgotten on every `set_active`, so a reply from before a re-activation is ignored.
- **Nothing is sent until the vehicle state is known**, and `update_vehicle` with a non-finite position or velocity makes it unknown again. `jl_mission` then holds, then lands on silence, which is the safe default.
- **No setpoints while `vehicle.landed`.** The takeoff step sends none anyway. After a landing, or when a mission touches down, the runner goes quiet.
- **Abort** (a failed step with no `on_fail`): keep sending the engine's hold setpoint for `abort_hold_s`, then request one `land`. This is spec §4's "hold, then land".
- **Never send an empty setpoint.** One with neither a position nor a velocity would be rejected by `jl_mission` as invalid; send nothing instead.
- **A target that raises in `observe()`** has the traceback collected in `drain_errors()`, so a custom target's bug can't crash the runner.

- [ ] **Step 1: Write the failing tests**

Create `src/jl_blocks/test/test_session.py`:

```python
from __future__ import annotations

import math

from jl_blocks import library  # noqa: F401  (registers the shipped blocks)
from jl_blocks.core import (
    Action,
    Output,
    Session,
    Setpoint,
    VehicleState,
    parse_mission,
    trajectory_fields,
)

DT = 0.02
MISSION = (
    "name: M\nsteps:\n  - takeoff: {height: 1.5}\n  - hold: {duration: 1}\n"
    "  - land: {}\n"
)
TAG_MISSION = (
    "name: M\ntarget: {aruco_tag: {camera: front}}\n"
    "steps:\n  - search: {}\n    until: target_seen\n  - hold: {}\n"
)
GROUND = VehicleState(position_ned=(0.0, 0.0, 0.0), landed=True)
AIR = VehicleState(position_ned=(0.0, 0.0, -1.5))


def session(text=MISSION, **kwargs):
    return Session(parse_mission(text), **kwargs)


def fly_to_hold(s):
    """Activate on the ground and answer the takeoff; ends in the hold step."""
    s.update_vehicle(GROUND)
    s.set_active(True, 0.0)
    (request,) = s.tick(DT, DT).requests
    s.action_done(request.token, True, "reached 1.50 m")
    s.update_vehicle(AIR)
    s.tick(2 * DT, DT)


def test_nothing_happens_until_the_mission_is_active():
    s = session()
    s.update_vehicle(GROUND)
    assert s.tick(DT, DT) == Output()
    assert s.state == "inactive"


def test_nothing_is_sent_before_the_vehicle_state_is_known():
    s = session()
    s.set_active(True, 0.0)
    assert s.tick(DT, DT) == Output()


def test_activation_asks_for_takeoff_once():
    s = session()
    s.update_vehicle(GROUND)
    s.set_active(True, 0.0)
    out = s.tick(DT, DT)
    assert out.setpoint is None
    assert [r.action for r in out.requests] == [Action("takeoff", 1.5)]
    assert s.tick(2 * DT, DT).requests == ()
    assert s.state == "takeoff"


def test_after_takeoff_the_next_step_sends_setpoints():
    s = session()
    fly_to_hold(s)
    assert s.state == "hold"
    assert s.tick(3 * DT, DT).setpoint == Setpoint(position=(0.0, 0.0, -1.5))


def test_every_activation_starts_again_from_step_one():
    s = session()
    fly_to_hold(s)
    s.set_active(True, 1.0)  # a second "true", with no "false" in between
    assert s.state == "takeoff"
    assert [r.action.kind for r in s.tick(1.0 + DT, DT).requests] == ["takeoff"]


def test_a_reply_from_before_a_reactivation_is_ignored():
    s = session()
    s.update_vehicle(GROUND)
    s.set_active(True, 0.0)
    (old,) = s.tick(DT, DT).requests
    s.set_active(False, 0.5)
    s.set_active(True, 1.0)
    (new,) = s.tick(1.0 + DT, DT).requests
    s.action_done(old.token, True, "late")
    s.tick(1.0 + 2 * DT, DT)
    assert s.state == "takeoff"
    s.action_done(new.token, True, "reached 1.50 m")
    s.update_vehicle(AIR)
    s.tick(1.0 + 3 * DT, DT)
    assert s.state == "hold"


def test_deactivation_stops_all_output():
    s = session()
    fly_to_hold(s)
    s.set_active(False, 0.1)
    assert s.tick(0.1 + DT, DT) == Output()
    assert s.state == "inactive"


def test_no_setpoints_while_landed():
    s = session("name: M\nsteps:\n  - hold: {}\n")
    s.update_vehicle(GROUND)
    s.set_active(True, 0.0)
    assert s.tick(DT, DT).setpoint is None
    s.update_vehicle(AIR)
    assert s.tick(2 * DT, DT).setpoint is not None


def test_a_failed_step_with_no_on_fail_holds_then_lands():
    s = session(abort_hold_s=2.0)
    s.update_vehicle(AIR)  # activated in the air
    s.set_active(True, 0.0)
    (request,) = s.tick(DT, DT).requests
    s.action_done(request.token, False, "landing in progress")
    s.tick(2 * DT, DT)  # the step fails here, at t = 0.04
    assert s.state == "aborted"
    out = s.tick(3 * DT, DT)
    assert out.setpoint == Setpoint(position=(0.0, 0.0, -1.5))
    assert out.requests == ()
    assert s.tick(2.03, DT).requests == ()  # held 1.99 s so far
    (land,) = s.tick(2.05, DT).requests
    assert land.action == Action("land")
    assert s.tick(2.07, DT).requests == ()


def test_an_empty_setpoint_is_never_sent(registry):
    # A target reset failure aborts before the vehicle is known: no hold position.
    text = "name: M\ntarget: {badtarget: {}}\nsteps:\n  - goto: {}\n"
    s = Session(parse_mission(text, registry), registry)
    s.set_active(True, 0.0)
    s.update_vehicle(AIR)
    assert s.tick(DT, DT).setpoint is None
    assert s.drain_errors()  # the reset traceback is reported


def test_a_non_finite_vehicle_state_counts_as_unknown():
    s = session()
    s.set_active(True, 0.0)
    s.update_vehicle(VehicleState(position_ned=(math.nan, 0.0, 0.0)))
    assert s.vehicle is None
    assert s.tick(DT, DT) == Output()


def test_detections_reach_the_targets_listening_on_that_topic():
    s = session(TAG_MISSION)
    assert s.topics() == ["/front/target_pose"]
    s.update_vehicle(VehicleState(position_ned=(0.0, 0.0, -1.0)))
    s.set_active(True, 0.0)
    s.observe("/target_pose", (0.0, 0.0, 3.0))  # the down camera's topic
    s.tick(DT, DT)
    assert s.state == "search"
    s.observe("/front/target_pose", (0.0, 0.0, 3.0))
    s.tick(2 * DT, DT)
    assert s.state == "hold"


def test_a_detection_before_the_vehicle_state_is_known_is_dropped():
    s = session(TAG_MISSION)
    s.set_active(True, 0.0)
    s.observe("/front/target_pose", (0.0, 0.0, 3.0))  # no attitude to place it
    s.update_vehicle(VehicleState(position_ned=(0.0, 0.0, -1.0)))
    s.tick(DT, DT)
    assert s.state == "search"


def test_a_target_that_raises_in_observe_is_reported_not_raised(registry):
    text = "name: M\ntarget: {grumpy: {}}\nsteps:\n  - goto: {}\n"
    s = Session(parse_mission(text, registry), registry)
    s.update_vehicle(AIR)
    s.set_active(True, 0.0)
    s.observe("/grumpy", (0.0, 0.0, 1.0))
    (error,) = s.drain_errors()
    assert "RuntimeError: grumpy" in error


def test_events_are_reported_once():
    s = session()
    s.update_vehicle(GROUND)
    s.set_active(True, 0.0)
    assert s.drain_events() == ["activated: starting from step 1", "takeoff: started"]
    s.tick(DT, DT)
    assert s.drain_events() == ["takeoff: requested takeoff"]
    assert s.drain_events() == []


def test_trajectory_fields_use_nan_for_uncontrolled_axes():
    position, velocity, yaw = trajectory_fields(Setpoint(velocity=(0.1, 0.2, 0.3)))
    assert all(math.isnan(v) for v in position)
    assert velocity == [0.1, 0.2, 0.3]
    assert math.isnan(yaw)
```

- [ ] **Step 2: Run the tests and watch them fail**

Run: `make check`
Expected: FAIL: `cannot import name 'Session' from 'jl_blocks.core'`.

- [ ] **Step 3: Implement**

Create `src/jl_blocks/jl_blocks/core/session.py`:

```python
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
        self, spec: MissionSpec, registry: Registry = REGISTRY, abort_hold_s: float = 2.0
    ) -> None:
        self.spec = spec
        self._registry = registry
        self._abort_hold_s = abort_hold_s
        self.vehicle: VehicleState | None = None
        self.engine: Engine | None = None
        self._next_token = 0
        # token -> the engine step entry it belongs to, or None for the abort landing
        self._requests: dict[int, int | None] = {}
        self._aborted_at: float | None = None
        self._abort_land_sent = False
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
        if self.engine is not None:
            self._events.extend(self.engine.drain_events())
        self._requests.clear()
        self._aborted_at = None
        self._abort_land_sent = False
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
        finite = is_finite(vehicle.position_ned) and is_finite(vehicle.velocity_ned)
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
                self._events.append("aborted: held, now landing")
                requests.append(self._request(Action("land"), None))

        empty = setpoint is not None and (
            setpoint.position is None and setpoint.velocity is None
        )
        if vehicle.landed or empty:
            setpoint = None
        return Output(setpoint, tuple(requests))

    def action_done(self, token: int, success: bool, message: str) -> None:
        if token not in self._requests or self.engine is None:
            return  # from an earlier activation
        entry = self._requests.pop(token)
        if entry is None:
            outcome = "ok" if success else "failed"
            self._events.append(f"abort landing {outcome} ({message})")
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
```

In `src/jl_blocks/jl_blocks/core/__init__.py`, add
`from .session import Output, Request, Session, trajectory_fields`
and add the four names to `__all__` (sorted).

- [ ] **Step 4: Run the tests and watch them pass**

Run: `make check`
Expected: all pass.

- [ ] **Step 5: Checkpoint**

Stop and report the diff to the maintainer. Do not commit.

---

### Task 10: Researchers' own blocks: `load_block_files` and `jl_blocks check --blocks`

**Files:**
- Create: `src/jl_blocks/jl_blocks/core/plugins.py`
- Modify: `src/jl_blocks/jl_blocks/core/__init__.py`, `src/jl_blocks/jl_blocks/cli.py`
- Test: `src/jl_blocks/test/test_plugins.py`

**Interfaces:**
- Consumes: `REGISTRY` and the `@block` decorator.
- Produces:
  - `load_block_files(paths: list[str]) -> list[str]`: imports every `.py` file in each path (a file, or a folder's top level), so its `@block` classes register. It returns one readable error per file that failed. It never raises.
  - `jl_blocks check --blocks PATH [--blocks PATH ...] FILE...`.
  - The runner (Task 11) takes the same paths as its `blocks` parameter.

Spec §2 shows `my_blocks/*.py (optional)` feeding the block library. Without this, a researcher's own block could never be checked or flown.

- [ ] **Step 1: Write the failing tests**

Create `src/jl_blocks/test/test_plugins.py`:

```python
from __future__ import annotations

from jl_blocks import cli
from jl_blocks import library  # noqa: F401  (registers the shipped blocks)
from jl_blocks.core import REGISTRY, load_block_files

BLOCK = '''
from jl_blocks.core import Mission, Setpoint, StepContext, block


@block("{name}")
class Wiggle(Mission):
    def step(self, ctx: StepContext) -> Setpoint:
        return Setpoint(position=ctx.vehicle.position_ned)
'''


def test_blocks_in_a_folder_are_registered(tmp_path):
    (tmp_path / "wiggle.py").write_text(BLOCK.format(name="plugin_folder_wiggle"))
    assert load_block_files([str(tmp_path)]) == []
    assert REGISTRY.get("plugin_folder_wiggle") is not None


def test_a_single_file_works_too(tmp_path):
    path = tmp_path / "wiggle.py"
    path.write_text(BLOCK.format(name="plugin_file_wiggle"))
    assert load_block_files([str(path)]) == []
    assert REGISTRY.get("plugin_file_wiggle") is not None


def test_a_broken_file_is_reported_not_raised(tmp_path):
    (tmp_path / "broken.py").write_text("def oops(:\n")
    errors = load_block_files([str(tmp_path)])
    assert len(errors) == 1
    assert "broken.py" in errors[0]
    assert "SyntaxError" in errors[0]


def test_reusing_a_shipped_name_is_reported(tmp_path):
    (tmp_path / "hold2.py").write_text(BLOCK.format(name="hold"))
    (error,) = load_block_files([str(tmp_path)])
    assert "block 'hold' is already registered" in error


def test_a_missing_path_is_reported(tmp_path):
    (error,) = load_block_files([str(tmp_path / "nope")])
    assert "no such file or folder" in error


def test_check_uses_your_own_blocks(tmp_path, capsys):
    blocks = tmp_path / "my_blocks"
    blocks.mkdir()
    (blocks / "wiggle.py").write_text(BLOCK.format(name="plugin_check_wiggle"))
    mission = tmp_path / "m.yaml"
    mission.write_text("name: M\nsteps:\n  - plugin_check_wiggle: {}\n")
    assert cli.main(["check", "--blocks", str(blocks), str(mission)]) == 0


def test_check_fails_on_a_broken_block_file(tmp_path, capsys):
    (tmp_path / "broken.py").write_text("def oops(:\n")
    mission = tmp_path / "m.yaml"
    mission.write_text("name: M\nsteps:\n  - hold: {}\n")
    assert cli.main(["check", "--blocks", str(tmp_path), str(mission)]) == 1
    assert "FAIL blocks" in capsys.readouterr().out
```

- [ ] **Step 2: Run the tests and watch them fail**

Run: `make check`
Expected: FAIL: `cannot import name 'load_block_files'`.

- [ ] **Step 3: Implement**

Create `src/jl_blocks/jl_blocks/core/plugins.py`:

```python
"""Load researchers' own blocks from .py files, so their @block classes register."""

from __future__ import annotations

import hashlib
import importlib.util
import sys
from pathlib import Path


def load_block_files(paths: list[str]) -> list[str]:
    """Import each .py file in `paths` (files, or the top level of folders).

    Returns one readable error per file or path that failed; never raises, so
    `jl_blocks check` and the runner can print every problem at once.
    """
    errors: list[str] = []
    files: list[Path] = []
    for raw in paths:
        path = Path(raw)
        if path.is_dir():
            files.extend(sorted(path.glob("*.py")))
        elif path.is_file():
            files.append(path)
        else:
            errors.append(f"{path}: no such file or folder")
    for file in files:
        digest = hashlib.sha1(str(file.resolve()).encode()).hexdigest()[:12]
        module_name = f"_jl_user_blocks_{file.stem}_{digest}"
        spec = importlib.util.spec_from_file_location(module_name, file)
        if spec is None or spec.loader is None:
            errors.append(f"{file}: cannot be imported")
            continue
        module = importlib.util.module_from_spec(spec)
        sys.modules[module_name] = module
        try:
            spec.loader.exec_module(module)
        except Exception as err:
            sys.modules.pop(module_name, None)
            errors.append(f"{file}: {type(err).__name__}: {err}")
    return errors
```

In `src/jl_blocks/jl_blocks/core/__init__.py`, add `from .plugins import load_block_files` and add `"load_block_files"` to `__all__` (sorted).

In `src/jl_blocks/jl_blocks/cli.py`:
- change the core import to `from .core import MissionError, load_block_files, load_mission`;
- change `check` to take the block paths first:

```python
def check(paths: list[str], blocks: list[str] | None = None) -> int:
    block_errors = load_block_files(blocks or [])
    if block_errors:
        print("FAIL blocks")
        for line in block_errors:
            print(f"  {line}")
        return 1
    failed = 0
```

  Keep the rest of `check` unchanged.
- in `main`, add the option and pass it through:

```python
    check_cmd.add_argument(
        "--blocks",
        action="append",
        default=[],
        metavar="PATH",
        help="a .py file or folder of your own blocks (repeat for more)",
    )
    check_cmd.add_argument("files", nargs="+", help="mission YAML files")
    args = parser.parse_args(argv)
    return check(args.files, args.blocks)
```

- [ ] **Step 4: Run the tests and watch them pass**

Run: `make check`
Expected: all pass.

- [ ] **Step 5: Checkpoint**

Stop and report the diff to the maintainer. Do not commit.

---

### Task 11: The `mission_runner` ROS node, launch file, packaging and the shipped missions

**Files:**
- Create:
  - `src/jl_blocks/jl_blocks/ros/__init__.py`, `src/jl_blocks/jl_blocks/ros/mission_runner.py`;
  - `src/jl_blocks/launch/mission.launch.py`;
  - `missions/takeoff_hold_land.yaml`, `missions/track_moving_aruco.yaml`.
- Modify: `src/jl_blocks/setup.py`, `src/jl_blocks/package.xml`, `Makefile` (the `ty` line).

**Interfaces:**
- Consumes (Tasks 9–10):
  - `Session`, `Request`, `trajectory_fields`;
  - `load_block_files`, `load_mission`, `MissionError`.
- Produces:
  - Executable `ros2 run jl_blocks mission_runner`. Its parameters:
    - `mission_file` (string, required);
    - `blocks` (string: comma-separated paths, default `""`);
    - `rate_hz` (50.0);
    - `local_position_topic`, `attitude_topic`, `land_detected_topic`.
  - Launch file `ros2 launch jl_blocks mission.launch.py mission_file:=PATH [blocks:=PATHS]`. It starts `jl_mission` (with `mission_name` read from the file) and the runner.
  - The runner logs every event as `event: <text>` and publishes it on `/jl/NAME/events`. `/jl/NAME/state` is latched. Task 12's smoke test greps `event: `.

**Safety note for the reviewer:** every runner failure mode ends in silence, never in a stale or wrong setpoint. Examples:
- a bad mission file: the process exits 1 before registering anything;
- an exception in `tick`: logged, nothing published;
- a wrong vehicle topic: `landed` stays true, so nothing is sent, and a warning is logged after 5 s.

Silence is what `jl_mission`'s watchdog handles: hold at 0.5 s, land at 5 s.

- [ ] **Step 1: Confirm the PX4 topic names in SITL**

The runner's vehicle topics must match what PX4 v1.16 publishes through the agent. In the container, start SITL the way `sitl_jl_mission.sh`'s `start_px4` does:

```bash
docker exec --user user -w /hdd/Documents/Electrical/Matrix_Lab/jacob_drone_ws/src/Jacob_Ladder jacob_ladder_sim bash -c '
source jl_env.sh && source /opt/ros/humble/setup.bash && source install/setup.bash
B=$JL_PX4_DIR/build/px4_sitl_default; L=$(mktemp -d); mkdir -p $L/rootfs && cp $B/rootfs/gz_env.sh $L/rootfs/
export GZ_PARTITION=topic_probe
HEADLESS=1 PX4_SIM_MODEL=gz_x500 GZ_IP=127.0.0.1 $B/bin/px4 -d -w $L/rootfs $B/etc > $L/px4.log 2>&1 &
MicroXRCEAgent udp4 -p 8888 > $L/agent.log 2>&1 &
sleep 25; ros2 topic list | grep -E "vehicle_(local_position|attitude|land_detected)"
pkill -f "gz sim"; pkill -f bin/px4; pkill -f MicroXRCEAgent'
```

Expected: three topic names. The defaults below assume `/fmu/out/vehicle_local_position_v1`, `/fmu/out/vehicle_attitude` and `/fmu/out/vehicle_land_detected`. If the list differs, use the listed names as the parameter defaults in Step 2, and say so in the report.

- [ ] **Step 2: Write the node**

Create `src/jl_blocks/jl_blocks/ros/__init__.py`:

```python
"""ROS 2 adapters. The only part of jl_blocks that imports rclpy."""
```

Create `src/jl_blocks/jl_blocks/ros/mission_runner.py`:

```python
"""mission_runner: flies one mission file through jl_mission.

A thin ROS adapter around jl_blocks.core.Session. It turns PX4 topics into a
VehicleState, feeds camera detections to the mission's targets, publishes the
session's setpoints on /jl/NAME/setpoint, and calls /jl/NAME/takeoff and
/jl/NAME/land when a block asks. Every decision lives in Session, which is
tested without ROS. Whenever something goes wrong here, the runner goes quiet
and jl_mission's watchdog holds, then lands.
"""

from __future__ import annotations

import sys
import traceback

import rclpy
from geometry_msgs.msg import PoseStamped
from jl_mission_interfaces.srv import Takeoff
from px4_msgs.msg import (
    TrajectorySetpoint,
    VehicleAttitude,
    VehicleLandDetected,
    VehicleLocalPosition,
)
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from .. import library  # noqa: F401  (registers the shipped blocks)
from ..core import (
    MissionError,
    Request,
    Session,
    Setpoint,
    VehicleState,
    load_block_files,
    load_mission,
    trajectory_fields,
)

NAN = float("nan")
LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class MissionRunner(Node):
    def __init__(self) -> None:
        super().__init__("mission_runner")
        mission_file = self.declare_parameter("mission_file", "").value
        blocks = self.declare_parameter("blocks", "").value
        rate_hz = self.declare_parameter("rate_hz", 50.0).value
        local_topic = self.declare_parameter(
            "local_position_topic", "/fmu/out/vehicle_local_position_v1"
        ).value
        attitude_topic = self.declare_parameter(
            "attitude_topic", "/fmu/out/vehicle_attitude"
        ).value
        land_topic = self.declare_parameter(
            "land_detected_topic", "/fmu/out/vehicle_land_detected"
        ).value

        errors = load_block_files([p for p in str(blocks).split(",") if p.strip()])
        if errors:
            raise MissionError(errors)
        if not mission_file:
            raise MissionError(["mission_file parameter is required"])
        spec = load_mission(str(mission_file))
        self.session = Session(spec)
        self.name = spec.name
        prefix = f"/jl/{self.name}"

        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, f"{prefix}/setpoint", qos_profile_sensor_data
        )
        self.state_pub = self.create_publisher(String, f"{prefix}/state", LATCHED)
        self.events_pub = self.create_publisher(String, f"{prefix}/events", 10)
        self.executor_clients = {
            "takeoff": self.create_client(Takeoff, f"{prefix}/takeoff"),
            "land": self.create_client(Trigger, f"{prefix}/land"),
        }
        self.create_subscription(Bool, f"{prefix}/active", self.on_active, LATCHED)
        self.create_subscription(
            VehicleLocalPosition, local_topic, self.on_local, qos_profile_sensor_data
        )
        self.create_subscription(
            VehicleAttitude, attitude_topic, self.on_attitude, qos_profile_sensor_data
        )
        self.create_subscription(
            VehicleLandDetected, land_topic, self.on_land, qos_profile_sensor_data
        )
        for topic in self.session.topics():
            self.create_subscription(
                PoseStamped,
                topic,
                lambda msg, t=topic: self.on_detection(t, msg),
                qos_profile_sensor_data,
            )

        self._local: VehicleLocalPosition | None = None
        self._attitude: VehicleAttitude | None = None
        self._landed: bool | None = None  # None until the first message
        self._started = self.now()
        self._warned = False
        self._last_tick: float | None = None
        self._last_state: str | None = None
        self.create_timer(1.0 / float(rate_hz), self.tick)
        self.publish_state()
        self.get_logger().info(
            f"flying {mission_file} as '{self.name}'; waiting for {prefix}/active"
        )

    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # ── inputs ──

    def on_active(self, msg: Bool) -> None:
        self.session.set_active(msg.data, self.now())
        self.flush()

    def on_local(self, msg: VehicleLocalPosition) -> None:
        self._local = msg
        self.update_vehicle()

    def on_attitude(self, msg: VehicleAttitude) -> None:
        self._attitude = msg
        self.update_vehicle()

    def on_land(self, msg: VehicleLandDetected) -> None:
        self._landed = bool(msg.landed)
        self.update_vehicle()

    def update_vehicle(self) -> None:
        local, attitude = self._local, self._attitude
        if local is None or attitude is None:
            return
        q = attitude.q
        self.session.update_vehicle(
            VehicleState(
                position_ned=(float(local.x), float(local.y), float(local.z)),
                velocity_ned=(float(local.vx), float(local.vy), float(local.vz)),
                yaw=float(local.heading),
                attitude=(float(q[0]), float(q[1]), float(q[2]), float(q[3])),
                # Until PX4 says otherwise, assume on the ground: send nothing.
                landed=self._landed is not False,
                stamp=self.now(),
            )
        )

    def on_detection(self, topic: str, msg: PoseStamped) -> None:
        p = msg.pose.position
        self.session.observe(topic, (float(p.x), float(p.y), float(p.z)))

    # ── the control loop ──

    def tick(self) -> None:
        now = self.now()
        dt = 0.0 if self._last_tick is None else now - self._last_tick
        self._last_tick = now
        self.warn_if_vehicle_topics_missing(now)
        try:
            out = self.session.tick(now, dt)
        except Exception:
            # Publish nothing: jl_mission holds, then lands on silence.
            self.get_logger().error(traceback.format_exc())
            return
        if out.setpoint is not None:
            self.publish_setpoint(out.setpoint)
        for request in out.requests:
            self.send(request)
        self.flush()

    def publish_setpoint(self, setpoint: Setpoint) -> None:
        position, velocity, yaw = trajectory_fields(setpoint)
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = position
        msg.velocity = velocity
        msg.acceleration = [NAN, NAN, NAN]
        msg.jerk = [NAN, NAN, NAN]
        msg.yaw = yaw
        msg.yawspeed = NAN
        self.setpoint_pub.publish(msg)

    def send(self, request: Request) -> None:
        kind = request.action.kind
        client = self.executor_clients[kind]
        if not client.service_is_ready():
            self.session.action_done(
                request.token,
                False,
                f"/jl/{self.name}/{kind} is not available (is jl_mission running?)",
            )
            return
        if kind == "takeoff":
            call = Takeoff.Request()
            call.height = float(request.action.height)
        else:
            call = Trigger.Request()
        future = client.call_async(call)
        future.add_done_callback(
            lambda f, token=request.token: self.on_reply(token, f)
        )

    def on_reply(self, token: int, future) -> None:
        error = future.exception()
        if error is not None:
            self.session.action_done(token, False, f"service call failed: {error}")
        else:
            reply = future.result()
            self.session.action_done(token, bool(reply.success), str(reply.message))
        self.flush()

    # ── outputs for people ──

    def flush(self) -> None:
        for event in self.session.drain_events():
            self.get_logger().info(f"event: {event}")
            self.events_pub.publish(String(data=event))
        for error in self.session.drain_errors():
            self.get_logger().error(error)
        self.publish_state()

    def publish_state(self) -> None:
        state = self.session.state
        if state != self._last_state:
            self._last_state = state
            self.state_pub.publish(String(data=state))

    def warn_if_vehicle_topics_missing(self, now: float) -> None:
        if self._warned or now - self._started < 5.0:
            return
        missing = [
            name
            for name, value in (
                ("local_position_topic", self._local),
                ("attitude_topic", self._attitude),
                ("land_detected_topic", self._landed),
            )
            if value is None
        ]
        if missing:
            self._warned = True
            self.get_logger().warn(
                "no data yet on " + ", ".join(missing) + ": the mission sends "
                "nothing until they arrive (check the topic names)"
            )


def main() -> None:
    rclpy.init()
    try:
        node = MissionRunner()
    except MissionError as err:
        for line in err.errors:
            print(f"mission_runner: {line}", file=sys.stderr)
        rclpy.try_shutdown()
        sys.exit(1)
    except OSError as err:
        print(f"mission_runner: cannot read the mission file: {err}", file=sys.stderr)
        rclpy.try_shutdown()
        sys.exit(1)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
```

- [ ] **Step 3: The launch file**

Create `src/jl_blocks/launch/mission.launch.py`:

```python
"""Fly one mission file: its jl_mission mode (the name shown in QGC) plus the runner.

ros2 launch jl_blocks mission.launch.py mission_file:=missions/takeoff_hold_land.yaml
ros2 launch jl_blocks mission.launch.py mission_file:=m.yaml blocks:=my_blocks
"""

import os

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _nodes(context):
    path = os.path.abspath(LaunchConfiguration("mission_file").perform(context))
    blocks = LaunchConfiguration("blocks").perform(context)
    with open(path, encoding="utf-8") as f:
        doc = yaml.safe_load(f)
    name = doc.get("name") if isinstance(doc, dict) else None
    if not isinstance(name, str):
        raise RuntimeError(f"{path} has no mission name; run: jl_blocks check {path}")
    block_paths = ",".join(os.path.abspath(p) for p in blocks.split(",") if p)
    suffix = name.lower()
    return [
        Node(
            package="jl_mission",
            executable="jl_mission",
            name=f"jl_mission_{suffix}",
            output="screen",
            parameters=[{"mission_name": name}],
        ),
        Node(
            package="jl_blocks",
            executable="mission_runner",
            name=f"mission_runner_{suffix}",
            output="screen",
            parameters=[{"mission_file": path, "blocks": block_paths}],
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("mission_file", description="mission YAML file"),
            DeclareLaunchArgument(
                "blocks",
                default_value="",
                description="comma-separated .py files or folders of your own blocks",
            ),
            OpaqueFunction(function=_nodes),
        ]
    )
```

- [ ] **Step 4: Packaging**

In `src/jl_blocks/setup.py`:
- add the launch file to `data_files`:

```python
        ("share/" + package_name + "/launch", ["launch/mission.launch.py"]),
```

- add the node to `console_scripts`:

```python
            "mission_runner = jl_blocks.ros.mission_runner:main",
```

In `src/jl_blocks/package.xml`, add after the existing `python3-yaml` exec_depend:

```xml
  <exec_depend>rclpy</exec_depend>
  <exec_depend>launch</exec_depend>
  <exec_depend>launch_ros</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>std_srvs</exec_depend>
  <exec_depend>px4_msgs</exec_depend>
  <exec_depend>jl_mission_interfaces</exec_depend>
  <exec_depend>jl_mission</exec_depend>
```

In the `Makefile`, the `ty` line in `check` gets the ROS folder excluded, because the check venv has no ROS:

```make
	$(CHECK_BIN)/ty check --python $(CHECK_VENV) --exclude 'src/jl_blocks/jl_blocks/ros/' $(JL_BLOCKS)/jl_blocks $(JL_BLOCKS)/test
```

`ruff` still lints and format-checks `ros/` and `launch/`.

- [ ] **Step 5: The shipped mission files**

Create `missions/takeoff_hold_land.yaml`:

```yaml
# The smallest mission: take off, hover for 5 s, land.
# Flown headless by test/sitl_runner_smoke.sh (make sitl-test).
name: TakeoffHoldLand
steps:
  - takeoff: {height: 1.5}
  - hold: {duration: 5}
  - land: {}
```

Create `missions/track_moving_aruco.yaml`:

```yaml
# Spec section 4's example: find an ArUco tag with the front camera and keep
# 2 m in front of it. The tag id and size are aruco_tracker's own parameters.
name: TrackMovingAruco
target:
  aruco_tag: {camera: front}
controller:
  pid: {kp: 0.8, ki: 0.0, kd: 0.2, max_speed: 1.0}

steps:
  - takeoff: {height: 1.5}
  - search: {pattern: hold}
    until: target_seen
    timeout: 30
    on_fail: land
  - track: {standoff: 2.0}
    until: never
    on_fail: search
  - land: {}
```

- [ ] **Step 6: Verify lint, types and the mission files on the host**

Run: `make check`
Expected: every stage passes. The last stage prints `ok   missions/takeoff_hold_land.yaml  (TakeoffHoldLand, 3 steps)` and `ok   missions/track_moving_aruco.yaml  (TrackMovingAruco, 4 steps)`.

- [ ] **Step 7: Build and run the node's failure path in the container**

```bash
docker exec --user user -w /hdd/Documents/Electrical/Matrix_Lab/jacob_drone_ws/src/Jacob_Ladder jacob_ladder_sim bash -c '
source /opt/ros/humble/setup.bash && colcon build --packages-select jl_blocks && source install/setup.bash
printf "name: Bad\nsteps:\n  - holdd: {}\n" > /tmp/bad.yaml
ros2 run jl_blocks mission_runner --ros-args -p mission_file:=/tmp/bad.yaml; echo "exit=$?"
ros2 launch jl_blocks mission.launch.py --show-args'
```

Expected:
- the build succeeds;
- the runner prints `mission_runner: /tmp/bad.yaml: steps[0] (holdd): unknown mission 'holdd'; did you mean 'hold'?` and then `exit=1`;
- `--show-args` lists `mission_file` and `blocks`.

- [ ] **Step 8: Checkpoint**

Stop and report the diff, the topic names from Step 1, and the Step 7 output to the maintainer. Do not commit.

---

### Task 12: SITL smoke flight, shared harness functions, and spec updates

**Files:**
- Create: `test/sitl_common.sh`, `test/sitl_runner_smoke.sh`
- Modify:
  - `src/jl_mission/test/sitl_jl_mission.sh`: sources the common file; no behaviour change;
  - `Makefile`: `sitl-test`;
  - `docs/superpowers/specs/2026-09-21-mission-blocks-design.md`: §4 example, §11.

**Interfaces:**
- Consumes: `ros2 launch jl_blocks mission.launch.py` and the `event: ` log lines (Task 11), plus `missions/takeoff_hold_land.yaml`.
- Produces:
  - `test/sitl_common.sh` (`start_px4 LOGDIR`, `kill_flight`, `wait_for_no_px4`, `$PX4_BUILD`), which Phase 4's `test/sitl_mission.sh` will reuse;
  - `make sitl-test` now also runs the smoke flight.

- [ ] **Step 1: Move the shared shell functions**

Create `test/sitl_common.sh` with the three functions moved verbatim from `src/jl_mission/test/sitl_jl_mission.sh`. The only change is two extra `pkill` lines for the runner:

```bash
# Shared by the headless SITL checks: start PX4 + agent + translation node in
# a fresh rootfs, and kill everything between flights. Source it, don't run it.
# Each script sets its own GZ_PARTITION before calling start_px4.
source "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/../jl_env.sh"
source /opt/ros/humble/setup.bash
source "$JL_WS_ROOT/install/setup.bash"
PX4_BUILD="$JL_PX4_DIR/build/px4_sitl_default"

wait_for_no_px4() {
  for _ in $(seq 30); do
    pgrep -f "$PX4_BUILD/bin/px4" >/dev/null || return 0
    sleep 1
  done
  echo "warning: a bin/px4 process is still running after 30s" >&2
}

# Kill everything from the previous flight before starting the next one.
kill_flight() {
  kill $(jobs -p) 2>/dev/null
  pkill -f "gz sim" 2>/dev/null
  pkill -f "$PX4_BUILD/bin/px4" 2>/dev/null
  pkill -f "MicroXRCEAgent" 2>/dev/null
  pkill -f "translation_node_bin" 2>/dev/null
  pkill -f "ros2 run jl_mission jl_mission" 2>/dev/null
  pkill -f "lib/jl_mission/jl_mission" 2>/dev/null
  pkill -f "ros2 launch jl_blocks" 2>/dev/null
  pkill -f "lib/jl_blocks/mission_runner" 2>/dev/null
  pkill -f "fake_runner.py" 2>/dev/null
  pkill -f "fake_pilot.py" 2>/dev/null
  wait_for_no_px4
}

# Start PX4 + agent + translation node in a fresh rootfs, and wait until PX4
# is up. $1: log dir.
start_px4() {
  local logs="$1"
  mkdir -p "$logs/rootfs" && cp "$PX4_BUILD/rootfs/gz_env.sh" "$logs/rootfs/"
  HEADLESS=1 PX4_SIM_MODEL=gz_x500 GZ_IP=127.0.0.1 \
    "$PX4_BUILD/bin/px4" -d -w "$logs/rootfs" "$PX4_BUILD/etc" > "$logs/px4.log" 2>&1 &
  MicroXRCEAgent udp4 -p 8888 > "$logs/agent.log" 2>&1 &
  ros2 run translation_node translation_node_bin > "$logs/translation.log" 2>&1 &
  for _ in $(seq 120); do grep -q "synchronized with time offset" "$logs/px4.log" && break; sleep 1; done
  (cd "$logs/rootfs" && "$PX4_BUILD/bin/px4-param" set NAV_DLL_ACT 0) >> "$logs/commander.log" 2>&1  # no GCS in this test
}
```

In `src/jl_mission/test/sitl_jl_mission.sh`, make these edits and nothing else:
- replace the three `source` lines (`jl_env.sh`, `/opt/ros/humble/setup.bash`, `install/setup.bash`) with
  `source "$HERE/../../../test/sitl_common.sh"`;
- delete the `PX4_BUILD=...` line;
- delete the definitions of `wait_for_no_px4`, `kill_flight` and `start_px4`.

Keep `trap kill_flight EXIT`, and keep everything else.

- [ ] **Step 2: Confirm the refactor changed nothing**

Run: `make sitl-test` (from the host; the Makefile still runs only the jl_mission script at this point).
Expected: the same three flights all-PASS as before (20/20 checks), exit code 0.

- [ ] **Step 3: Write the smoke flight**

Create `test/sitl_runner_smoke.sh` and make it executable (`chmod +x`):

```bash
#!/bin/bash
# SITL smoke flight for the Phase 3 mission_runner, headless: fly
# missions/takeoff_hold_land.yaml through jl_mission with
# `ros2 launch jl_blocks mission.launch.py`, and check the runner's step
# events arrive in order. Phase 4 generalizes this into test/sitl_mission.sh.
# Run inside the sim container after colcon build.
HERE="$(dirname "$(readlink -f "$0")")"
source "$HERE/sitl_common.sh"
export GZ_PARTITION=sitl_runner_smoke
NAME=TakeoffHoldLand
MISSION="$JL_WS_ROOT/missions/takeoff_hold_land.yaml"
trap kill_flight EXIT

echo "== flight: runner smoke ($NAME) =="
logs=$(mktemp -d)
start_px4 "$logs"
ros2 launch jl_blocks mission.launch.py mission_file:="$MISSION" > "$logs/launch.log" 2>&1 &
for _ in $(seq 60); do grep -q "Registered '$NAME'" "$logs/launch.log" && break; sleep 1; done
sleep 2
(cd "$logs/rootfs" && "$PX4_BUILD/bin/px4-commander" mode ext1) >> "$logs/commander.log" 2>&1

# Takeoff (~10 s) + 5 s hold + land (~10 s)
for _ in $(seq 90); do grep -q "event: mission finished" "$logs/launch.log" && break; sleep 1; done
grep "event:" "$logs/launch.log"

rc=0
last=0
for event in "takeoff: started" "takeoff: takeoff ok" "takeoff: done" \
             "hold: started" "hold: done" \
             "land: started" "land: land ok" "land: done" "mission finished"; do
  line=$(grep -n "event: $event" "$logs/launch.log" | head -1 | cut -d: -f1)
  if [ -n "$line" ] && [ "$line" -gt "$last" ]; then
    echo "PASS $event"; last=$line
  else
    echo "FAIL $event (missing or out of order)"; rc=1
  fi
done
if grep -qE "Traceback|Assertion|terminate called" "$logs/launch.log"; then
  echo "FAIL no crash in the runner or the mode"; rc=1
else
  echo "PASS no crash in the runner or the mode"
fi
state=$(timeout 10 ros2 topic echo --once --qos-durability transient_local \
  "/jl/$NAME/state" std_msgs/msg/String 2>/dev/null | grep -o "data: .*")
if [ "$state" = "data: finished" ]; then
  echo "PASS /jl/$NAME/state is finished"
else
  echo "FAIL /jl/$NAME/state is '$state', want 'data: finished'"; rc=1
fi
[ $rc -ne 0 ] && echo "logs: $logs"
exit $rc
```

- [ ] **Step 4: Run it and watch it pass**

Rebuild in the container (`colcon build --packages-select jl_blocks`), then:

```bash
docker restart jacob_ladder_sim && sleep 3
docker exec --user user -w /hdd/Documents/Electrical/Matrix_Lab/jacob_drone_ws/src/Jacob_Ladder jacob_ladder_sim bash -c 'source /opt/ros/humble/setup.bash && source install/setup.bash && test/sitl_runner_smoke.sh'
```

Expected: 11 PASS lines (9 events, no crash, state `finished`) and exit code 0.

If an event is missing, read `$logs/launch.log` before changing anything:
- **No `takeoff: started`:** the runner never saw `/active` true.
- **`takeoff: started` but no `takeoff ok`:** check the service reply in `launch.log`.
- **`takeoff ok` but the hold never finishes:** check that `landed` flipped to false. A wrong `land_detected_topic` keeps it true, and the 5 s warning says so.

Fix the cause in `jl_blocks` (never in `jl_mission`, `precision_land` or `drogue_flight`), and report what it was.

- [ ] **Step 5: Add it to `make sitl-test`**

In the `Makefile`, change the `sitl-test` recipe's `docker exec` line to run both scripts:

```make
	docker exec --user user -w $(CURDIR) $(SIM_CONTAINER) bash -c 'source /opt/ros/humble/setup.bash && source install/setup.bash && src/jl_mission/test/sitl_jl_mission.sh && test/sitl_runner_smoke.sh'
```

And extend the comment above `SIM_CONTAINER` to:

```make
# L3: fly the jl_mission safety contract, then a mission file through the
# mission_runner, in headless SITL (spec section 8).
# Needs the jacob_ladder_sim container (./docker/run_sim_container.sh) with
# jl_mission_interfaces, jl_mission and jl_blocks built by colcon inside it.
```

Run: `make sitl-test` twice.
Expected: the three jl_mission flights all-PASS, then the smoke flight's 11 PASS lines, with exit code 0, both times.

- [ ] **Step 6: Spec updates**

In `docs/superpowers/specs/2026-09-21-mission-blocks-design.md`:

In §4, change the example's target line from `aruco_tag: {id: 0, camera: front, size_m: 0.15}` to:

```yaml
  aruco_tag: {camera: front}      # tag id and size: aruco_tracker's own params
```

In §3's shipped-blocks table, change the Mission row `` `track` / `approach` (`standoff`) `` to `` `track` (`standoff`) ``.

In §11:
- replace the bullet that starts `` - `takeoff` and `land` are mission blocks, but `` with:

```markdown
- Resolved in Phase 3: a mission block's `step()` may return an `Action` (`takeoff` / `land`); the engine hands it out once (`Engine.take_action()`), the runner calls `jl_mission`, and the reply arrives in `StepContext.action`. A finished mission holds; a mission that should land ends with a `land` step.
```

- replace the bullet that starts `` - `jl_mission` needs `mission_name` of at most 24 characters `` with:

```markdown
- Resolved in Phase 3: the loader enforces jl_mission's name rule (at most 24 characters), so `jl_blocks check` catches it.
```

- [ ] **Step 7: Final check**

Run: `make check`
Expected: every stage passes. Report the pytest count.

- [ ] **Step 8: Checkpoint**

Stop and report the diff, both `make sitl-test` outputs, and the pytest count to the maintainer. Do not commit.

---

## Maintainer notes (found while planning; not tasks)

- **Rulings to confirm at review:**
  - `aruco_tag` drops the spec example's `id`/`size_m` (Task 6);
  - detections are placed in NED on arrival (Task 6);
  - the `search` spiral is centred on the step start (Task 4);
  - `track` fails on target loss instead of searching in place (Task 7);
  - `precision_descend` has three small deviations (Task 8);
  - `/jl/NAME/events` carries the "why it ended" lines, and `/jl/NAME/state` stays one word (Global Constraints).
- **Not in this phase:**
  - Phase 4 flies `track_moving_aruco`-style missions in SITL against the moving ArUco world and checks the 3 m standoff within ±0.25 m;
  - Phase 5 adds `config/flight.yaml`, `jl_mission@.service.in` and `deploy.sh`.
- **Real drone:** nothing here changes what the drone runs today. The runner has only flown in SITL. Before a real flight, confirm the three PX4 topic names on the Jetson (Task 11 Step 1 checks SITL only), since a wrong name keeps the runner silent.
- **Still-deferred Phase 1 minors** (not picked up here; none affect flight):
  - the `_type_ok` pass-through for non-scalar params;
  - the Makefile tool-pin stamp;
  - CI permissions and the double run;
  - docstring line lengths;
  - the cascade-error message.
