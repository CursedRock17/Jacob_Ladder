# Mission Blocks Phase 1: `jl_blocks` core, checker CLI and CI — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Ship the ROS-free core of the mission-blocks system (block base classes, registry, mission-file loader and checker, step engine), the `hold` and `position` blocks, the `jl_blocks check` CLI, and a `make check` gate that also runs in GitHub Actions.

**Architecture:** A new `ament_python` package, `src/jl_blocks`. Everything under `jl_blocks/core/` is plain Python with no ROS imports, so it's unit-tested with `pytest` on any machine and in CI. Mission YAML is parsed into frozen `MissionSpec` dataclasses, and every error is reported at once with a "did you mean" suggestion. The `Engine` runs a spec one tick at a time, with time passed in so every transition is testable without sleeping.

**Tech Stack:** Python 3.10, PyYAML 6.0.3, pytest 9.1.1, ruff 0.15.20, ty 0.0.55, uv, GNU make, colcon/ament_python (ROS 2 Humble), GitHub Actions.

**Spec:** `docs/superpowers/specs/2026-09-21-mission-blocks-design.md`. This plan implements phase 1 of section 9, plus the L0 and L1 rows of section 8.

## Global Constraints

- Python target is 3.10 (`requires-python = ">=3.10, <3.11"`); use `from __future__ import annotations` in every module.
- `jl_blocks/core/` must not import `rclpy` or any ROS package, directly or indirectly (spec §2).
- Pinned tool versions, used everywhere: `ruff==0.15.20`, `ty==0.0.55`, `pytest==9.1.1`, `pyyaml==6.0.3`.
- Formatting is `ruff format` with the repo's `pyproject.toml` (default 88-column line length). Code blocks below are already formatted.
- Package license is `BSD-3-Clause` (the repo `LICENSE`).
- **Do not commit.** The maintainer makes all commits. Each task ends with a checkpoint: stop and report the diff.
- Work only on `main`; never add yourself to commit history.
- Error message text in the loader is user-facing. Tests assert it exactly, so change a message only together with its test.
- `.check-venv/` is a build artifact; never commit it.

## File Structure

```
src/jl_blocks/
├── package.xml                  ament manifest (python3-yaml exec dep)
├── setup.py / setup.cfg         ament_python install, `jl_blocks` console script
├── resource/jl_blocks           ament index marker (empty file)
├── jl_blocks/
│   ├── __init__.py
│   ├── cli.py                   `jl_blocks check FILE...`
│   ├── core/                    ROS-free; everything below is unit-tested
│   │   ├── __init__.py          public API re-exports
│   │   ├── types.py             Vector3, VehicleState, Setpoint, Status
│   │   ├── blocks.py            Block/Target/Controller/Mission, StepContext, Registry, @block
│   │   ├── loader.py            YAML → MissionSpec, MissionError with all problems
│   │   └── engine.py            Engine: ticks a MissionSpec, until/timeout/on_fail
│   └── library/                 shipped blocks (importing registers them)
│       ├── __init__.py
│       ├── hold.py              mission `hold`
│       └── position.py          controller `position`
└── test/
    ├── conftest.py              fake blocks in a private Registry
    ├── test_types.py
    ├── test_blocks.py
    ├── test_loader.py
    ├── test_engine.py
    └── test_library_and_cli.py
Makefile                         + `check` target and .check-venv bootstrap
pyproject.toml                   + ty extra-path; + pytest, pyyaml deps
.gitignore                       + .check-venv/
.github/workflows/check.yml      runs `make check` on push and PR
README.md                        + "Checking your work" pointer
```

---

### Task 1: Package scaffold, data types and the `make check` gate

**Files:**
- Create: `src/jl_blocks/package.xml`, `src/jl_blocks/setup.py`, `src/jl_blocks/setup.cfg`, `src/jl_blocks/resource/jl_blocks` (empty), `src/jl_blocks/jl_blocks/__init__.py`, `src/jl_blocks/jl_blocks/core/__init__.py`, `src/jl_blocks/jl_blocks/core/types.py`
- Modify: `Makefile` (append a target, update `.PHONY` on line 44), `pyproject.toml` (`[tool.ty.environment] extra-paths`), `.gitignore`
- Test: `src/jl_blocks/test/test_types.py`

**Interfaces:**
- Consumes: nothing.
- Produces: `jl_blocks.core.Vector3 = tuple[float, float, float]`; `VehicleState(position_ned, velocity_ned=(0,0,0), yaw=0.0, attitude=(1,0,0,0), landed=False)` (frozen); `Setpoint(position=None, velocity=None, yaw=None)` (frozen); `Status.RUNNING/DONE/FAILED` with values `"running"/"done"/"failed"`; the `make check` target.

- [ ] **Step 1: Write the failing test**

Create `src/jl_blocks/test/test_types.py`:

```python
from __future__ import annotations

from jl_blocks.core import Setpoint, Status, VehicleState


def test_vehicle_state_defaults_to_still_and_level():
    v = VehicleState(position_ned=(1.0, 2.0, -3.0))
    assert v.velocity_ned == (0.0, 0.0, 0.0)
    assert v.attitude == (1.0, 0.0, 0.0, 0.0)
    assert not v.landed


def test_setpoint_fields_default_to_not_controlled():
    assert Setpoint() == Setpoint(position=None, velocity=None, yaw=None)


def test_status_has_three_values():
    assert [s.value for s in Status] == ["running", "done", "failed"]
```

- [ ] **Step 2: Add the `check` target and tool config**

Append to `Makefile` (after the `clean:` recipe, before `.PHONY`):

```make
# Lint, type-check and unit-test the mission blocks (no ROS needed). The tools
# are pinned and installed once into .check-venv, so the same versions run here
# and in CI (.github/workflows/check.yml). uv must be installed.
CHECK_VENV := .check-venv
CHECK_BIN := $(CHECK_VENV)/bin
JL_BLOCKS := src/jl_blocks
MISSIONS := $(wildcard missions/*.yaml)

$(CHECK_BIN)/pytest:
	uv venv --python 3.10 $(CHECK_VENV)
	uv pip install --python $(CHECK_VENV) ruff==0.15.20 ty==0.0.55 pytest==9.1.1 pyyaml==6.0.3

check: $(CHECK_BIN)/pytest
	$(CHECK_BIN)/ruff check $(JL_BLOCKS)
	$(CHECK_BIN)/ruff format --check $(JL_BLOCKS)
	$(CHECK_BIN)/ty check --python $(CHECK_VENV) $(JL_BLOCKS)/jl_blocks $(JL_BLOCKS)/test
	PYTHONPATH=$(JL_BLOCKS) $(CHECK_BIN)/pytest -q $(JL_BLOCKS)/test
ifneq ($(MISSIONS),)
	PYTHONPATH=$(JL_BLOCKS) $(CHECK_BIN)/python -m jl_blocks.cli check $(MISSIONS)
endif
```

Change the last line of `Makefile` from `.PHONY: all format build clean` to:

```make
.PHONY: all format build clean check
```

In `pyproject.toml`, under `[tool.ty.environment]`, add `"src/jl_blocks"` as the last entry of `extra-paths`:

```toml
extra-paths = [
    "src/oak_d_visual_odometry",
    "src/drogue_flight",
    "src/ros2_yolo_image_processing",
    "packages/jetson_cudss_bootstrap",
    "src/jl_blocks",
]
```

Append to `.gitignore`:

```
# Pinned lint/test tools for `make check`
.check-venv/
```

Notes: `make check` deliberately covers only `src/jl_blocks`. The rest of the workspace has pre-existing ruff findings (see "Maintainer notes" at the end), and fixing them isn't part of this plan. `ty` is pointed at the package and its tests, not at `setup.py`, which imports setuptools that the check venv doesn't need.

- [ ] **Step 3: Run the check to verify it fails**

Run: `make check`
Expected: the venv is created, `ruff` passes on the one test file, then `ty` or `pytest` fails with `unresolved-import` / `ModuleNotFoundError: No module named 'jl_blocks'`.

- [ ] **Step 4: Write the package scaffold and types**

Create `src/jl_blocks/package.xml`:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>jl_blocks</name>
  <version>0.1.0</version>
  <description>Compose autonomous missions from reusable Python blocks (targets, controllers, missions)</description>
  <maintainer email="mtglucas1@gmail.com">Lucas Wendland</maintainer>
  <license>BSD-3-Clause</license>

  <exec_depend>python3-yaml</exec_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

Create `src/jl_blocks/setup.py`:

```python
from setuptools import find_packages, setup

package_name = "jl_blocks"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "pyyaml"],
    zip_safe=True,
    maintainer="Lucas Wendland",
    maintainer_email="mtglucas1@gmail.com",
    description="Compose autonomous missions from reusable Python blocks.",
    license="BSD-3-Clause",
    extras_require={
        "test": ["pytest"],
    },
    entry_points={
        "console_scripts": [
            "jl_blocks = jl_blocks.cli:main",
        ],
    },
)
```

Create `src/jl_blocks/setup.cfg`:

```ini
[develop]
script_dir=$base/lib/jl_blocks
[install]
install_scripts=$base/lib/jl_blocks
```

Create the empty ament marker: `touch src/jl_blocks/resource/jl_blocks`

Create `src/jl_blocks/jl_blocks/__init__.py`:

```python
"""Jacob's Ladder mission blocks."""
```

Create `src/jl_blocks/jl_blocks/core/types.py`:

```python
"""Plain data passed between the step engine and blocks. No ROS types here."""

from __future__ import annotations

import enum
from dataclasses import dataclass

# North, east, down, in metres (PX4's local frame).
Vector3 = tuple[float, float, float]


@dataclass(frozen=True)
class VehicleState:
    position_ned: Vector3
    velocity_ned: Vector3 = (0.0, 0.0, 0.0)
    yaw: float = 0.0
    # w, x, y, z; body FRD -> NED, as PX4 reports it
    attitude: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)
    landed: bool = False


@dataclass(frozen=True)
class Setpoint:
    """What to send PX4 this tick. None on a field means "not controlled"."""

    position: Vector3 | None = None
    velocity: Vector3 | None = None
    yaw: float | None = None


class Status(enum.Enum):
    RUNNING = "running"
    DONE = "done"
    FAILED = "failed"
```

Create `src/jl_blocks/jl_blocks/core/__init__.py` (it grows in Tasks 2–4):

```python
"""ROS-free core: block base classes, registry, mission loader and step engine."""

from .types import Setpoint, Status, Vector3, VehicleState

__all__ = [
    "Setpoint",
    "Status",
    "Vector3",
    "VehicleState",
]
```

- [ ] **Step 5: Run the check to verify it passes**

Run: `make check`
Expected: `ruff` "All checks passed!", "N files already formatted", `ty` "All checks passed!", `3 passed`.

- [ ] **Step 6: Checkpoint**

Stop and report the diff (`git status --short`, `git diff --stat`) to the maintainer. Do not commit.

---

### Task 2: Block base classes and the registry

**Files:**
- Create: `src/jl_blocks/jl_blocks/core/blocks.py`, `src/jl_blocks/test/conftest.py`
- Modify: `src/jl_blocks/jl_blocks/core/__init__.py`
- Test: `src/jl_blocks/test/test_blocks.py`

**Interfaces:**
- Consumes: `Setpoint`, `Status`, `Vector3`, `VehicleState` from Task 1.
- Produces:
  - `Block` with `kind: ClassVar[str]`, `name: ClassVar[str]`, nested `@dataclass class Params`, `__init__(self, params: object | None = None)` setting `self.params: Any`, and `reset(self) -> None`.
  - `Target(Block)`: `kind = "target"`, `lost_after: ClassVar[float] = 3.0`, `estimate(self, vehicle: VehicleState) -> Vector3 | None`.
  - `Controller(Block)`: `kind = "controller"`, `command(self, vehicle: VehicleState, desired: Setpoint, dt: float) -> Setpoint`.
  - `Mission(Block)`: `kind = "mission"`, `step(self, ctx: StepContext) -> Setpoint | None`, `status(self, ctx: StepContext) -> Status` (default `RUNNING`).
  - `StepContext(vehicle, target, target_position, controller, elapsed, dt)` (frozen).
  - `Registry` with `register(name, cls)` (sets `cls.name`, raises `ValueError` on duplicates), `get(name) -> type[Block] | None` and `names(kind=None) -> list[str]` (sorted).
  - `REGISTRY` (the default registry) and the `block(name, registry=REGISTRY)` class decorator.
  - Test fixtures `registry` (a private `Registry` holding the fakes `beacon`, `position`, `doubler`, `goto`, `broken`, `quitter`, `needy`) and `vehicle` (at NED `(0, 0, -1)`).

- [ ] **Step 1: Write the fixtures and the failing test**

Create `src/jl_blocks/test/conftest.py`:

```python
"""Fake blocks registered in a private Registry, so tests never depend on the shipped library."""

from __future__ import annotations

from dataclasses import dataclass

import pytest

from jl_blocks.core import (
    Controller,
    Mission,
    Registry,
    Setpoint,
    Status,
    StepContext,
    Target,
    VehicleState,
)


def make_registry() -> Registry:
    reg = Registry()

    class Beacon(Target):
        """Visible whenever the class-level switch says so."""

        visible = True

        @dataclass
        class Params:
            id: int = 0

        def estimate(self, vehicle: VehicleState) -> tuple[float, float, float] | None:
            return (5.0, 0.0, -1.0) if Beacon.visible else None

    class Passthrough(Controller):
        def command(
            self, vehicle: VehicleState, desired: Setpoint, dt: float
        ) -> Setpoint:
            return desired

    class Doubler(Controller):
        """Doubles x, so a test can tell which controller ran."""

        def command(
            self, vehicle: VehicleState, desired: Setpoint, dt: float
        ) -> Setpoint:
            assert desired.position is not None
            x, y, z = desired.position
            return Setpoint(position=(2 * x, y, z))

    class Goto(Mission):
        """Flies to a fixed point; DONE after `ticks` ticks."""

        @dataclass
        class Params:
            x: float = 1.0
            ticks: int = 0  # 0 = never done

        def reset(self) -> None:
            self.count = 0

        def step(self, ctx: StepContext) -> Setpoint:
            self.count += 1
            return Setpoint(position=(self.params.x, 0.0, -1.0))

        def status(self, ctx: StepContext) -> Status:
            return (
                Status.DONE
                if self.params.ticks and self.count >= self.params.ticks
                else Status.RUNNING
            )

    class Broken(Mission):
        def step(self, ctx: StepContext) -> Setpoint:
            raise RuntimeError("boom")

    class Quitter(Mission):
        def step(self, ctx: StepContext) -> Setpoint:
            return Setpoint(position=(0.0, 0.0, -1.0))

        def status(self, ctx: StepContext) -> Status:
            return Status.FAILED

    class Needy(Mission):
        @dataclass
        class Params:
            height: float  # required: no default

        def step(self, ctx: StepContext) -> Setpoint | None:
            return None

    for name, cls in [
        ("beacon", Beacon),
        ("position", Passthrough),
        ("doubler", Doubler),
        ("goto", Goto),
        ("broken", Broken),
        ("quitter", Quitter),
        ("needy", Needy),
    ]:
        reg.register(name, cls)
    Beacon.visible = True
    return reg


@pytest.fixture
def registry() -> Registry:
    return make_registry()


@pytest.fixture
def vehicle() -> VehicleState:
    return VehicleState(position_ned=(0.0, 0.0, -1.0))
```

Create `src/jl_blocks/test/test_blocks.py`:

```python
from __future__ import annotations

import pytest

from jl_blocks.core import Controller, Mission, Registry, Target, block


def test_decorator_registers_block_under_its_name():
    reg = Registry()

    @block("spin", registry=reg)
    class Spin(Mission):
        pass

    assert reg.get("spin") is Spin
    assert Spin.name == "spin"


def test_registering_the_same_name_twice_is_an_error():
    reg = Registry()
    reg.register("a", Mission)
    with pytest.raises(ValueError, match="already registered"):
        reg.register("a", Controller)


def test_names_can_be_filtered_by_kind(registry):
    assert registry.names("target") == ["beacon"]
    assert "goto" in registry.names("mission")
    assert "goto" not in registry.names("controller")


def test_block_uses_default_params_when_none_given(registry):
    goto = registry.get("goto")
    assert goto is not None
    assert goto().params.x == 1.0


def test_target_default_lost_after_is_three_seconds():
    assert Target.lost_after == 3.0
```

- [ ] **Step 2: Run the check to verify it fails**

Run: `make check`
Expected: FAIL. `ty` reports that `Registry`, `Controller` and the others can't be imported from `jl_blocks.core`, and pytest errors on conftest with `ImportError: cannot import name 'Controller'`.

- [ ] **Step 3: Write `blocks.py` and export it**

Create `src/jl_blocks/jl_blocks/core/blocks.py`:

```python
"""The three kinds of block, and the registry that names them.

A block is one small class. Its parameters are a nested ``Params`` dataclass,
so the mission loader can check a mission file's params against it.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from typing import Any, ClassVar, TypeVar

from .types import Setpoint, Status, Vector3, VehicleState


class Block:
    kind: ClassVar[str] = "block"
    name: ClassVar[str] = ""

    @dataclass
    class Params:
        pass

    def __init__(self, params: object | None = None) -> None:
        # Any: each block's Params dataclass is different, and YAML fills it in.
        self.params: Any = params if params is not None else self.Params()

    def reset(self) -> None:
        """Called when a step using this block starts."""


class Target(Block):
    """Answers "where is the thing?" in NED metres, or None if not seen."""

    kind = "target"
    # Seconds without an estimate before the target counts as lost.
    lost_after: ClassVar[float] = 3.0

    def estimate(self, vehicle: VehicleState) -> Vector3 | None:
        raise NotImplementedError


class Controller(Block):
    """Turns what the mission wants into the setpoint sent to PX4."""

    kind = "controller"

    def command(self, vehicle: VehicleState, desired: Setpoint, dt: float) -> Setpoint:
        raise NotImplementedError


@dataclass(frozen=True)
class StepContext:
    vehicle: VehicleState
    target: Target | None
    # This tick's target estimate, so missions don't call estimate() again.
    target_position: Vector3 | None
    controller: Controller
    elapsed: float
    dt: float


class Mission(Block):
    """What the drone is trying to do in one step, and whether it's done."""

    kind = "mission"

    def step(self, ctx: StepContext) -> Setpoint | None:
        raise NotImplementedError

    def status(self, ctx: StepContext) -> Status:
        return Status.RUNNING


B = TypeVar("B", bound=type[Block])


class Registry:
    def __init__(self) -> None:
        self._blocks: dict[str, type[Block]] = {}

    def register(self, name: str, cls: type[Block]) -> None:
        if name in self._blocks:
            raise ValueError(f"block '{name}' is already registered")
        cls.name = name
        self._blocks[name] = cls

    def get(self, name: str) -> type[Block] | None:
        return self._blocks.get(name)

    def names(self, kind: str | None = None) -> list[str]:
        return sorted(
            n for n, c in self._blocks.items() if kind is None or c.kind == kind
        )


REGISTRY = Registry()


def block(name: str, registry: Registry = REGISTRY) -> Callable[[B], B]:
    """Class decorator: ``@block("pid")`` makes the class usable as ``pid:`` in a mission file."""

    def register(cls: B) -> B:
        registry.register(name, cls)
        return cls

    return register
```

Replace `src/jl_blocks/jl_blocks/core/__init__.py` with:

```python
"""ROS-free core: block base classes, registry, mission loader and step engine."""

from .blocks import (
    REGISTRY,
    Block,
    Controller,
    Mission,
    Registry,
    StepContext,
    Target,
    block,
)
from .types import Setpoint, Status, Vector3, VehicleState

__all__ = [
    "REGISTRY",
    "Block",
    "Controller",
    "Mission",
    "Registry",
    "Setpoint",
    "Status",
    "StepContext",
    "Target",
    "Vector3",
    "VehicleState",
    "block",
]
```

- [ ] **Step 4: Run the check to verify it passes**

Run: `make check`
Expected: all linters pass; `8 passed`.

- [ ] **Step 5: Checkpoint**

Stop and report the diff to the maintainer. Do not commit.

---

### Task 3: Mission loader and checker

**Files:**
- Create: `src/jl_blocks/jl_blocks/core/loader.py`
- Modify: `src/jl_blocks/jl_blocks/core/__init__.py`
- Test: `src/jl_blocks/test/test_loader.py`

**Interfaces:**
- Consumes: `REGISTRY`, `Registry` from Task 2 (and `Registry.get`, `Registry.names(kind)`, `Block.kind`, `Block.Params`).
- Produces:
  - `BlockSpec(name: str, params: Any)` (frozen), where `params` is an instance of that block's `Params`.
  - `StepSpec(name, block: BlockSpec, until: str | float = "done", timeout: float | None = None, on_fail: str | None = None, target: BlockSpec | None = None, controller: BlockSpec | None = None)` (frozen).
  - `MissionSpec(name, target: BlockSpec | None, controller: BlockSpec, steps: tuple[StepSpec, ...])` (frozen).
  - `MissionError(Exception)` with `.errors: list[str]`, each formatted `"<source>: <where>: <message>"`.
  - `parse_mission(text: str, registry=REGISTRY, source="<mission>") -> MissionSpec`.
  - `load_mission(path: str | Path, registry=REGISTRY) -> MissionSpec`, which raises `FileNotFoundError` for a missing file.
  - Rules: if no controller is given, it defaults to the block named `position`. `until` is one of `done`, `target_seen`, `target_lost`, `never`, or seconds > 0.

- [ ] **Step 1: Write the failing test**

Create `src/jl_blocks/test/test_loader.py`:

```python
from __future__ import annotations

import pytest

from jl_blocks.core import MissionError, parse_mission

GOOD = """
name: TrackBeacon
target:
  beacon: {id: 3}
controller:
  doubler: {}
steps:
  - goto: {x: 2.0}
    until: target_seen
    timeout: 30
    on_fail: goto_again
  - goto: {x: 4}
    name: goto_again
    until: never
"""


def errors(text, registry):
    with pytest.raises(MissionError) as caught:
        parse_mission(text, registry, source="m.yaml")
    return caught.value.errors


def test_good_mission_parses_into_a_spec(registry):
    spec = parse_mission(GOOD, registry)
    assert spec.name == "TrackBeacon"
    assert spec.target is not None and spec.target.params.id == 3
    assert spec.controller.name == "doubler"
    assert [s.name for s in spec.steps] == ["goto", "goto_again"]
    first = spec.steps[0]
    assert (first.until, first.timeout, first.on_fail) == (
        "target_seen",
        30,
        "goto_again",
    )
    assert spec.steps[1].block.params.x == 4


def test_controller_defaults_to_position(registry):
    spec = parse_mission("name: M\nsteps:\n  - goto: {}\n", registry)
    assert spec.controller.name == "position"


def test_unknown_param_suggests_the_close_one(registry):
    errs = errors("name: M\nsteps:\n  - goto: {xx: 1}\n", registry)
    assert errs == [
        "m.yaml: steps[0] (goto): unknown param 'xx' for goto; did you mean 'x'?"
    ]


def test_unknown_block_suggests_a_block_of_the_same_kind(registry):
    errs = errors("name: M\nsteps:\n  - gotoo: {}\n", registry)
    assert errs == [
        "m.yaml: steps[0] (gotoo): unknown mission 'gotoo'; did you mean 'goto'?"
    ]


def test_block_of_the_wrong_kind_is_named(registry):
    errs = errors("name: M\ncontroller:\n  goto: {}\nsteps:\n  - goto: {}\n", registry)
    assert errs == ["m.yaml: controller: 'goto' is a mission, not a controller"]


def test_param_type_is_checked_and_ints_count_as_floats(registry):
    errs = errors("name: M\nsteps:\n  - goto: {x: fast, ticks: 2.5}\n", registry)
    assert "param 'x' for goto should be float, got 'fast'" in errs[0]
    assert "param 'ticks' for goto should be int, got 2.5" in errs[1]


def test_missing_required_param_is_reported(registry):
    errs = errors("name: M\nsteps:\n  - needy: {}\n", registry)
    assert errs == [
        "m.yaml: steps[0] (needy): missing required param 'height' for needy"
    ]


def test_all_problems_are_reported_together(registry):
    text = "name: 9bad\nsteps:\n  - gotoo: {}\n  - goto: {xx: 1}\n"
    assert len(errors(text, registry)) == 3


def test_step_must_have_exactly_one_block(registry):
    errs = errors("name: M\nsteps:\n  - goto: {}\n    quitter: {}\n", registry)
    assert errs == [
        "m.yaml: steps[0]: each step needs exactly one mission block (found: goto, quitter)"
    ]


def test_until_accepts_words_and_seconds_only(registry):
    parse_mission("name: M\nsteps:\n  - goto: {}\n    until: 5\n", registry)
    errs = errors("name: M\nsteps:\n  - goto: {}\n    until: soon\n", registry)
    assert "until must be one of done, target_seen, target_lost, never" in errs[0]


def test_on_fail_must_name_an_existing_step(registry):
    errs = errors("name: M\nsteps:\n  - goto: {}\n    on_fail: gotoo\n", registry)
    assert errs == [
        "m.yaml: step 'goto': on_fail 'gotoo' is not a step; did you mean 'goto'?"
    ]


def test_duplicate_step_names_need_a_name_field(registry):
    errs = errors("name: M\nsteps:\n  - goto: {}\n  - goto: {}\n", registry)
    assert errs == [
        "m.yaml: step 'goto': duplicate step name; add 'name:' to one of them"
    ]


def test_target_conditions_need_a_target(registry):
    errs = errors("name: M\nsteps:\n  - goto: {}\n    until: target_seen\n", registry)
    assert errs == [
        "m.yaml: step 'goto': until: target_seen needs a target (at the top or on this step)"
    ]


def test_step_can_override_target_and_controller(registry):
    text = "name: M\nsteps:\n  - goto: {}\n    target: {beacon: {id: 7}}\n    controller: {doubler: {}}\n    until: target_seen\n"
    step = parse_mission(text, registry).steps[0]
    assert step.target is not None and step.target.params.id == 7
    assert step.controller is not None and step.controller.name == "doubler"


def test_unknown_top_level_key_is_reported(registry):
    errs = errors("name: M\nstep:\n  - goto: {}\n", registry)
    assert "m.yaml: top level: unknown key 'step'; did you mean 'steps'?" in errs


def test_invalid_yaml_is_reported_not_raised_raw(registry):
    errs = errors("name: [unclosed\n", registry)
    assert errs[0].startswith("m.yaml: invalid YAML:")


def test_mission_name_must_be_usable_in_qgc_and_ros(registry):
    errs = errors("name: my mission\nsteps:\n  - goto: {}\n", registry)
    assert "name must start with a letter" in errs[0]
```

- [ ] **Step 2: Run the check to verify it fails**

Run: `make check`
Expected: FAIL with `ImportError: cannot import name 'MissionError' from 'jl_blocks.core'` (`ty` reports the same import).

- [ ] **Step 3: Write `loader.py` and export it**

Create `src/jl_blocks/jl_blocks/core/loader.py`:

```python
"""Read a mission YAML file into a checked MissionSpec.

Every problem found is collected and reported together, each with where it is
and, for typos, a suggestion, so a beginner fixes a file in one pass instead
of one error at a time.
"""

from __future__ import annotations

import dataclasses
import difflib
import re
import typing
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

from .blocks import REGISTRY, Registry

TOP_KEYS = ("name", "target", "controller", "steps")
STEP_KEYS = ("until", "timeout", "on_fail", "name", "target", "controller")
UNTIL_WORDS = ("done", "target_seen", "target_lost", "never")
DEFAULT_CONTROLLER = "position"
NAME_PATTERN = re.compile(r"^[A-Za-z][A-Za-z0-9_]*$")


@dataclass(frozen=True)
class BlockSpec:
    name: str
    params: Any


@dataclass(frozen=True)
class StepSpec:
    name: str
    block: BlockSpec
    until: str | float = "done"
    timeout: float | None = None
    on_fail: str | None = None
    target: BlockSpec | None = None
    controller: BlockSpec | None = None


@dataclass(frozen=True)
class MissionSpec:
    name: str
    target: BlockSpec | None
    controller: BlockSpec
    steps: tuple[StepSpec, ...]


class MissionError(Exception):
    def __init__(self, errors: list[str]) -> None:
        self.errors = errors
        super().__init__("\n".join(errors))


def _suggest(word: str, choices: list[str]) -> str:
    close = difflib.get_close_matches(word, choices, n=1, cutoff=0.6)
    return f"; did you mean '{close[0]}'?" if close else ""


def _is_number(value: object) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool)


class _Parser:
    def __init__(self, registry: Registry, source: str) -> None:
        self.registry = registry
        self.source = source
        self.errors: list[str] = []
        self.step_names: list[str] = []

    def error(self, where: str, message: str) -> None:
        self.errors.append(f"{self.source}: {where}: {message}")

    def block_ref(self, node: Any, kind: str, where: str) -> BlockSpec | None:
        """Parse ``{name: {params}}`` into a BlockSpec of the given kind."""
        if not isinstance(node, dict) or len(node) != 1:
            self.error(
                where, f"expected one {kind} block, like 'name: {{param: value}}'"
            )
            return None
        ((name, params),) = node.items()
        return self.block(str(name), params, kind, where)

    def block(self, name: str, params: Any, kind: str, where: str) -> BlockSpec | None:
        cls = self.registry.get(name)
        if cls is None:
            self.error(
                where,
                f"unknown {kind} '{name}'{_suggest(name, self.registry.names(kind))}",
            )
            return None
        if cls.kind != kind:
            self.error(where, f"'{name}' is a {cls.kind}, not a {kind}")
            return None
        if params is None:
            params = {}
        if not isinstance(params, dict):
            self.error(
                where, f"params for {name} must be a mapping like {{param: value}}"
            )
            return None
        fields = {f.name: f for f in dataclasses.fields(cls.Params)}
        hints = typing.get_type_hints(cls.Params)
        ok = True
        for key, value in params.items():
            if key not in fields:
                self.error(
                    where,
                    f"unknown param '{key}' for {name}{_suggest(str(key), list(fields))}",
                )
                ok = False
            elif not _type_ok(hints.get(key), value):
                self.error(
                    where,
                    f"param '{key}' for {name} should be {_type_name(hints.get(key))}, got {value!r}",
                )
                ok = False
        for field in fields.values():
            no_default = (
                field.default is dataclasses.MISSING
                and field.default_factory is dataclasses.MISSING
            )
            if no_default and field.name not in params:
                self.error(where, f"missing required param '{field.name}' for {name}")
                ok = False
        if not ok:
            return None
        return BlockSpec(name, cls.Params(**params))

    def step(self, index: int, node: Any) -> StepSpec | None:
        where = f"steps[{index}]"
        if not isinstance(node, dict):
            self.error(where, "each step must be a mapping like '- hold: {}'")
            return None
        block_keys = [k for k in node if k not in STEP_KEYS]
        if len(block_keys) != 1:
            found = ", ".join(map(str, block_keys)) or "none"
            self.error(
                where, f"each step needs exactly one mission block (found: {found})"
            )
            return None
        block_name = str(block_keys[0])
        where = f"steps[{index}] ({block_name})"
        spec = self.block(block_name, node[block_name], "mission", where)

        until = node.get("until", "done")
        if not (until in UNTIL_WORDS or (_is_number(until) and until > 0)):
            self.error(
                where,
                f"until must be one of {', '.join(UNTIL_WORDS)} or a number of seconds, got {until!r}",
            )
        timeout = node.get("timeout")
        if timeout is not None and not (_is_number(timeout) and timeout > 0):
            self.error(
                where, f"timeout must be a positive number of seconds, got {timeout!r}"
            )
        on_fail = node.get("on_fail")
        if on_fail is not None and not isinstance(on_fail, str):
            self.error(where, f"on_fail must be a step name, got {on_fail!r}")
        name = node.get("name", block_name)
        if not isinstance(name, str):
            self.error(where, f"name must be text, got {name!r}")
        self.step_names.append(str(name))
        target = (
            self.block_ref(node["target"], "target", f"{where} target")
            if "target" in node
            else None
        )
        controller = (
            self.block_ref(node["controller"], "controller", f"{where} controller")
            if "controller" in node
            else None
        )
        if spec is None:
            return None
        return StepSpec(
            name=str(name),
            block=spec,
            until=until,
            timeout=timeout,
            on_fail=on_fail,
            target=target,
            controller=controller,
        )

    def mission(self, doc: Any) -> MissionSpec | None:
        if not isinstance(doc, dict):
            self.error(
                "file",
                "a mission file must be a mapping with name, steps and optionally target and controller",
            )
            return None
        for key in doc:
            if key not in TOP_KEYS:
                self.error(
                    "top level",
                    f"unknown key '{key}'{_suggest(str(key), list(TOP_KEYS))}",
                )

        name = doc.get("name")
        if not isinstance(name, str) or not NAME_PATTERN.match(name):
            self.error(
                "name",
                f"name must start with a letter and use only letters, digits and _ (got {name!r})",
            )

        target = (
            self.block_ref(doc["target"], "target", "target")
            if "target" in doc
            else None
        )
        if "controller" in doc:
            controller = self.block_ref(doc["controller"], "controller", "controller")
        else:
            controller = self.block(
                DEFAULT_CONTROLLER, {}, "controller", "controller (default)"
            )

        raw_steps = doc.get("steps")
        if not isinstance(raw_steps, list) or not raw_steps:
            self.error("steps", "a mission needs a non-empty list of steps")
            return None
        steps = [
            s
            for s in (self.step(i, node) for i, node in enumerate(raw_steps))
            if s is not None
        ]

        seen: set[str] = set()
        for step_name in self.step_names:
            if step_name in seen:
                self.error(
                    f"step '{step_name}'",
                    "duplicate step name; add 'name:' to one of them",
                )
            seen.add(step_name)
        for step in steps:
            if step.on_fail is not None and step.on_fail not in seen:
                self.error(
                    f"step '{step.name}'",
                    f"on_fail '{step.on_fail}' is not a step{_suggest(step.on_fail, sorted(seen))}",
                )
            if (
                step.until in ("target_seen", "target_lost")
                and step.target is None
                and target is None
            ):
                self.error(
                    f"step '{step.name}'",
                    f"until: {step.until} needs a target (at the top or on this step)",
                )

        if self.errors or controller is None or not isinstance(name, str):
            return None
        return MissionSpec(
            name=name, target=target, controller=controller, steps=tuple(steps)
        )


def _type_ok(expected: object, value: object) -> bool:
    if expected is float:
        return _is_number(value)
    if expected is int:
        return isinstance(value, int) and not isinstance(value, bool)
    if expected in (bool, str):
        return isinstance(value, expected)  # ty: ignore[invalid-argument-type]
    return True


def _type_name(expected: object) -> str:
    return getattr(expected, "__name__", str(expected))


def parse_mission(
    text: str, registry: Registry = REGISTRY, source: str = "<mission>"
) -> MissionSpec:
    try:
        doc = yaml.safe_load(text)
    except yaml.YAMLError as err:
        raise MissionError([f"{source}: invalid YAML: {err}"]) from err
    parser = _Parser(registry, source)
    spec = parser.mission(doc)
    if spec is None:
        raise MissionError(parser.errors)
    return spec


def load_mission(path: str | Path, registry: Registry = REGISTRY) -> MissionSpec:
    return parse_mission(Path(path).read_text(), registry, source=str(path))
```

Replace `src/jl_blocks/jl_blocks/core/__init__.py` with:

```python
"""ROS-free core: block base classes, registry, mission loader and step engine."""

from .blocks import (
    REGISTRY,
    Block,
    Controller,
    Mission,
    Registry,
    StepContext,
    Target,
    block,
)
from .loader import (
    BlockSpec,
    MissionError,
    MissionSpec,
    StepSpec,
    load_mission,
    parse_mission,
)
from .types import Setpoint, Status, Vector3, VehicleState

__all__ = [
    "REGISTRY",
    "Block",
    "BlockSpec",
    "Controller",
    "Mission",
    "MissionError",
    "MissionSpec",
    "Registry",
    "Setpoint",
    "Status",
    "StepContext",
    "StepSpec",
    "Target",
    "Vector3",
    "VehicleState",
    "block",
    "load_mission",
    "parse_mission",
]
```

- [ ] **Step 4: Run the check to verify it passes**

Run: `make check`
Expected: all linters pass; `25 passed`.

- [ ] **Step 5: Checkpoint**

Stop and report the diff to the maintainer. Do not commit.

---

### Task 4: Step engine

**Files:**
- Create: `src/jl_blocks/jl_blocks/core/engine.py`
- Modify: `src/jl_blocks/jl_blocks/core/__init__.py`
- Test: `src/jl_blocks/test/test_engine.py`

**Interfaces:**
- Consumes: `MissionSpec`, `StepSpec` and `BlockSpec` (Task 3); `Registry`, `REGISTRY`, `Block`, `Target`, `Controller`, `Mission` and `StepContext` (Task 2); `Setpoint`, `Status` and `VehicleState` (Task 1).
- Produces: `Engine(spec: MissionSpec, registry: Registry = REGISTRY)` with:
  - `start(now: float) -> None`
  - `tick(vehicle: VehicleState, now: float, dt: float) -> Setpoint | None`. This returns `None` before `start()` or when the mission block returns `None`. Once finished or aborted, it returns `Setpoint(position=<where it ended>)`.
  - Properties and attributes: `state -> str` (the current step name, or `"idle"`, `"finished"` or `"aborted"`), `started`, `finished`, `aborted` (all `bool`), and `events` (a `list[str]`).
  - Semantics:
    - `Status.FAILED` or a timeout fails the step.
    - A block exception fails the step too, and that tick returns the vehicle's current position.
    - A failed step jumps to its `on_fail` step, re-entering it with `mission.reset()` and `controller.reset()`. With no `on_fail`, the engine sets `aborted` and holds, which tells the runner to land.
    - `target_lost` means no estimate for `Target.lost_after` seconds, counted from the last estimate or from the step's start.

- [ ] **Step 1: Write the failing test**

Create `src/jl_blocks/test/test_engine.py`:

```python
from __future__ import annotations

from jl_blocks.core import Engine, VehicleState, parse_mission

DT = 0.02


def engine_for(text, registry):
    engine = Engine(parse_mission(text, registry), registry)
    engine.start(now=0.0)
    return engine


def run(engine, vehicle, seconds, start=0.0):
    """Tick at 50 Hz from `start` for `seconds`; return the last setpoint."""
    setpoint = None
    t = start
    while t < start + seconds - 1e-9:
        t += DT
        setpoint = engine.tick(vehicle, t, DT)
    return setpoint


def test_nothing_happens_before_start(registry, vehicle):
    engine = Engine(
        parse_mission("name: M\nsteps:\n  - goto: {}\n", registry), registry
    )
    assert engine.tick(vehicle, 1.0, DT) is None
    assert engine.state == "idle"


def test_step_output_goes_through_the_controller(registry, vehicle):
    engine = engine_for(
        "name: M\ncontroller: {doubler: {}}\nsteps:\n  - goto: {x: 1.5}\n", registry
    )
    assert engine.tick(vehicle, DT, DT).position == (3.0, 0.0, -1.0)


def test_done_moves_to_the_next_step_and_resets_it(registry, vehicle):
    text = "name: M\nsteps:\n  - goto: {ticks: 3}\n  - goto: {x: 9}\n    name: second\n"
    engine = engine_for(text, registry)
    for i in range(3):
        engine.tick(vehicle, (i + 1) * DT, DT)
    assert engine.state == "second"
    assert engine.tick(vehicle, 4 * DT, DT).position == (9.0, 0.0, -1.0)


def test_last_step_done_finishes_and_holds_where_it_ended(registry):
    engine = engine_for("name: M\nsteps:\n  - goto: {ticks: 1}\n", registry)
    here = VehicleState(position_ned=(3.0, 4.0, -2.0))
    engine.tick(here, DT, DT)
    assert engine.finished and engine.state == "finished"
    assert engine.tick(
        VehicleState(position_ned=(0.0, 0.0, 0.0)), 2 * DT, DT
    ).position == (3.0, 4.0, -2.0)


def test_until_seconds(registry, vehicle):
    engine = engine_for("name: M\nsteps:\n  - goto: {}\n    until: 1\n", registry)
    run(engine, vehicle, 0.9)
    assert not engine.finished
    run(engine, vehicle, 0.2, start=0.9)
    assert engine.finished


def test_until_never_ignores_the_block_saying_done(registry, vehicle):
    engine = engine_for(
        "name: M\nsteps:\n  - goto: {ticks: 1}\n    until: never\n", registry
    )
    run(engine, vehicle, 5)
    assert engine.state == "goto"


def test_until_target_seen(registry, vehicle):
    beacon = registry.get("beacon")
    beacon.visible = False
    text = (
        "name: M\ntarget: {beacon: {}}\nsteps:\n  - goto: {}\n    until: target_seen\n"
    )
    engine = engine_for(text, registry)
    run(engine, vehicle, 2)
    assert engine.state == "goto"
    beacon.visible = True
    engine.tick(vehicle, 2.1, DT)
    assert engine.finished


def test_until_target_lost_waits_lost_after_seconds(registry, vehicle):
    beacon = registry.get("beacon")
    text = (
        "name: M\ntarget: {beacon: {}}\nsteps:\n  - goto: {}\n    until: target_lost\n"
    )
    engine = engine_for(text, registry)
    run(engine, vehicle, 1)
    beacon.visible = False
    run(engine, vehicle, 2.9, start=1)
    assert engine.state == "goto"
    run(engine, vehicle, 0.2, start=3.9)
    assert engine.finished


def test_timeout_jumps_to_on_fail(registry, vehicle):
    text = "name: M\nsteps:\n  - goto: {}\n    timeout: 2\n    on_fail: rescue\n  - goto: {x: 7}\n    name: rescue\n"
    engine = engine_for(text, registry)
    run(engine, vehicle, 2.1)
    assert engine.state == "rescue"
    assert "goto: failed (timeout after 2 s) -> rescue" in engine.events


def test_failure_without_on_fail_aborts_and_holds(registry):
    engine = engine_for("name: M\nsteps:\n  - quitter: {}\n", registry)
    here = VehicleState(position_ned=(1.0, 2.0, -3.0))
    engine.tick(here, DT, DT)
    assert engine.aborted and engine.state == "aborted"
    assert engine.tick(here, 2 * DT, DT).position == (1.0, 2.0, -3.0)
    assert (
        engine.events[-1]
        == "quitter: failed (block reported failure); no on_fail -> hold, then land"
    )


def test_block_exception_fails_the_step_instead_of_crashing(registry, vehicle):
    text = "name: M\nsteps:\n  - broken: {}\n    on_fail: goto\n  - goto: {}\n"
    engine = engine_for(text, registry)
    setpoint = engine.tick(vehicle, DT, DT)
    assert setpoint.position == vehicle.position_ned
    assert engine.state == "goto"
    assert "broken: failed (RuntimeError: boom) -> goto" in engine.events


def test_on_fail_can_loop_back_and_reset_the_step(registry, vehicle):
    text = "name: M\nsteps:\n  - goto: {}\n    name: search\n    until: 0.1\n  - quitter: {}\n    on_fail: search\n"
    engine = engine_for(text, registry)
    run(engine, vehicle, 0.2)
    assert engine.state == "search"


def test_step_override_controller_applies_only_to_that_step(registry, vehicle):
    text = "name: M\nsteps:\n  - goto: {x: 1}\n    controller: {doubler: {}}\n    until: 0.05\n  - goto: {x: 1}\n    name: plain\n"
    engine = engine_for(text, registry)
    assert engine.tick(vehicle, DT, DT).position == (2.0, 0.0, -1.0)
    run(engine, vehicle, 0.1, start=DT)
    assert engine.state == "plain"
    assert engine.tick(vehicle, 1.0, DT).position == (1.0, 0.0, -1.0)
```

- [ ] **Step 2: Run the check to verify it fails**

Run: `make check`
Expected: FAIL with `ImportError: cannot import name 'Engine' from 'jl_blocks.core'`.

- [ ] **Step 3: Write `engine.py` and export it**

Create `src/jl_blocks/jl_blocks/core/engine.py`:

```python
"""Runs a MissionSpec one tick at a time.

Each tick: ask the step's target where the thing is, ask the step's mission
block what it wants, pass that through the controller, then decide whether the
step is done, failed, or still running. Time is always passed in, never read
from a clock, so every transition can be tested without sleeping.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TypeVar

from .blocks import REGISTRY, Block, Controller, Mission, Registry, StepContext, Target
from .loader import BlockSpec, MissionSpec, StepSpec
from .types import Setpoint, Status, VehicleState

T = TypeVar("T", bound=Block)


@dataclass
class _Step:
    spec: StepSpec
    mission: Mission
    target: Target | None
    controller: Controller


class Engine:
    def __init__(self, spec: MissionSpec, registry: Registry = REGISTRY) -> None:
        self.spec = spec
        self._registry = registry
        shared_target = self._build(spec.target, Target) if spec.target else None
        shared_controller = self._build(spec.controller, Controller)
        self._steps = [
            _Step(
                spec=s,
                mission=self._build(s.block, Mission),
                target=self._build(s.target, Target) if s.target else shared_target,
                controller=self._build(s.controller, Controller)
                if s.controller
                else shared_controller,
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

    def _build(self, spec: BlockSpec, base: type[T]) -> T:
        cls = self._registry.get(spec.name)
        if cls is None or not issubclass(cls, base):
            raise ValueError(
                f"'{spec.name}' is not a registered {base.__name__.lower()}"
            )
        return cls(spec.params)

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
        self._enter(0, now)

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
            )
            desired = step.mission.step(ctx)
            status = step.mission.status(ctx)
            setpoint = (
                step.controller.command(vehicle, desired, dt)
                if desired is not None
                else None
            )
        except Exception as err:  # a block bug must fail the step, not crash the runner
            self._fail(step, f"{type(err).__name__}: {err}", vehicle, now)
            return Setpoint(position=vehicle.position_ned)

        if status is Status.FAILED:
            self._fail(step, "block reported failure", vehicle, now)
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

    def _enter(self, index: int, now: float) -> None:
        self._i = index
        self._t0 = now
        self._last_seen = now
        step = self._steps[index]
        step.mission.reset()
        step.controller.reset()
        self.events.append(f"{step.spec.name}: started")

    def _advance(self, vehicle: VehicleState, now: float) -> None:
        self.events.append(f"{self._steps[self._i].spec.name}: done")
        if self._i + 1 < len(self._steps):
            self._enter(self._i + 1, now)
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
            self._enter(self._index[on_fail], now)
        else:
            self.events.append(
                f"{step.spec.name}: failed ({reason}); no on_fail -> hold, then land"
            )
            self.aborted = True
            self._hold_position = vehicle.position_ned
```

Replace `src/jl_blocks/jl_blocks/core/__init__.py` with the final version:

```python
"""ROS-free core: block base classes, registry, mission loader and step engine."""

from .blocks import (
    REGISTRY,
    Block,
    Controller,
    Mission,
    Registry,
    StepContext,
    Target,
    block,
)
from .engine import Engine
from .loader import (
    BlockSpec,
    MissionError,
    MissionSpec,
    StepSpec,
    load_mission,
    parse_mission,
)
from .types import Setpoint, Status, Vector3, VehicleState

__all__ = [
    "REGISTRY",
    "Block",
    "BlockSpec",
    "Controller",
    "Engine",
    "Mission",
    "MissionError",
    "MissionSpec",
    "Registry",
    "Setpoint",
    "Status",
    "StepContext",
    "StepSpec",
    "Target",
    "Vector3",
    "VehicleState",
    "block",
    "load_mission",
    "parse_mission",
]
```

- [ ] **Step 4: Run the check to verify it passes**

Run: `make check`
Expected: all linters pass; `38 passed`.

- [ ] **Step 5: Checkpoint**

Stop and report the diff to the maintainer. Do not commit.

---

### Task 5: The `hold` and `position` blocks, and the `jl_blocks check` CLI

**Files:**
- Create: `src/jl_blocks/jl_blocks/library/__init__.py`, `src/jl_blocks/jl_blocks/library/hold.py`, `src/jl_blocks/jl_blocks/library/position.py`, `src/jl_blocks/jl_blocks/cli.py`
- Test: `src/jl_blocks/test/test_library_and_cli.py`

**Interfaces:**
- Consumes: `block`, `Mission`, `Controller`, `StepContext`, `Setpoint`, `Status`, `Vector3`, `VehicleState`, `Engine`, `REGISTRY`, `load_mission` and `MissionError` from `jl_blocks.core`.
- Produces:
  - The mission block `hold` (`Params.duration: float = 0.0`; with 0 it never finishes on its own). It holds the position recorded on its first tick after `reset()`.
  - The controller `position` (a passthrough), registered in `REGISTRY` when `jl_blocks.library` is imported.
  - `jl_blocks.cli.main(argv: list[str] | None = None) -> int` and `check(paths: list[str]) -> int`. These return 0 if every file is OK and 1 otherwise. They print `ok   <path>  (<Name>, <n> steps)`, or `FAIL <path>` followed by indented errors.
  - The `jl_blocks` console script (`ros2 run jl_blocks jl_blocks check ...`).

- [ ] **Step 1: Write the failing test**

Create `src/jl_blocks/test/test_library_and_cli.py`:

```python
from __future__ import annotations

from jl_blocks import cli
from jl_blocks.core import REGISTRY, Engine, Setpoint, VehicleState, parse_mission


def test_shipped_blocks_are_registered():
    assert REGISTRY.get("hold") is not None
    assert REGISTRY.get("position") is not None


def test_hold_keeps_the_position_from_when_the_step_started():
    engine = Engine(parse_mission("name: M\nsteps:\n  - hold: {duration: 1}\n"))
    engine.start(0.0)
    first = engine.tick(VehicleState(position_ned=(1.0, 1.0, -2.0)), 0.02, 0.02)
    drifted = engine.tick(VehicleState(position_ned=(1.3, 1.0, -2.0)), 0.04, 0.02)
    assert first == drifted == Setpoint(position=(1.0, 1.0, -2.0))


def test_hold_with_duration_finishes():
    engine = Engine(parse_mission("name: M\nsteps:\n  - hold: {duration: 0.5}\n"))
    engine.start(0.0)
    here = VehicleState(position_ned=(0.0, 0.0, -1.0))
    engine.tick(here, 0.4, 0.02)
    assert not engine.finished
    engine.tick(here, 0.6, 0.02)
    assert engine.finished


def test_check_passes_a_good_file(tmp_path, capsys):
    path = tmp_path / "hover.yaml"
    path.write_text("name: Hover\nsteps:\n  - hold: {duration: 10}\n")
    assert cli.main(["check", str(path)]) == 0
    assert f"ok   {path}  (Hover, 1 steps)" in capsys.readouterr().out


def test_check_fails_and_explains_a_bad_file(tmp_path, capsys):
    path = tmp_path / "bad.yaml"
    path.write_text("name: Bad\nsteps:\n  - hold: {duraton: 10}\n")
    assert cli.main(["check", str(path)]) == 1
    out = capsys.readouterr().out
    assert f"FAIL {path}" in out
    assert "unknown param 'duraton' for hold; did you mean 'duration'?" in out


def test_check_reports_a_missing_file(tmp_path, capsys):
    assert cli.main(["check", str(tmp_path / "nope.yaml")]) == 1
    assert "file not found" in capsys.readouterr().out
```

- [ ] **Step 2: Run the check to verify it fails**

Run: `make check`
Expected: FAIL with `ImportError: cannot import name 'cli' from 'jl_blocks'`.

- [ ] **Step 3: Write the blocks and the CLI**

Create `src/jl_blocks/jl_blocks/library/__init__.py`:

```python
"""Blocks shipped with Jacob's Ladder. Importing this package registers them."""

from . import hold, position

__all__ = ["hold", "position"]
```

Create `src/jl_blocks/jl_blocks/library/hold.py`:

```python
"""hold: stay where the drone was when the step started."""

from __future__ import annotations

from dataclasses import dataclass

from ..core import Mission, Setpoint, Status, StepContext, Vector3, block


@block("hold")
class Hold(Mission):
    @dataclass
    class Params:
        duration: float = 0.0  # s; 0 means hold until the step's until/timeout ends it

    def reset(self) -> None:
        self._position: Vector3 | None = None

    def step(self, ctx: StepContext) -> Setpoint:
        if self._position is None:
            self._position = ctx.vehicle.position_ned
        return Setpoint(position=self._position)

    def status(self, ctx: StepContext) -> Status:
        if self.params.duration > 0 and ctx.elapsed >= self.params.duration:
            return Status.DONE
        return Status.RUNNING
```

Create `src/jl_blocks/jl_blocks/library/position.py`:

```python
"""position: pass the mission's setpoint straight to PX4, whose own controller closes the loop."""

from __future__ import annotations

from ..core import Controller, Setpoint, VehicleState, block


@block("position")
class Position(Controller):
    def command(self, vehicle: VehicleState, desired: Setpoint, dt: float) -> Setpoint:
        return desired
```

Create `src/jl_blocks/jl_blocks/cli.py`:

```python
"""Command line: ``jl_blocks check missions/*.yaml``."""

from __future__ import annotations

import argparse
import sys

from . import library  # noqa: F401  (registers the shipped blocks)
from .core import MissionError, load_mission


def check(paths: list[str]) -> int:
    failed = 0
    for path in paths:
        try:
            spec = load_mission(path)
        except FileNotFoundError:
            print(f"FAIL {path}\n  file not found")
            failed += 1
        except MissionError as err:
            print(f"FAIL {path}")
            for line in err.errors:
                print(f"  {line}")
            failed += 1
        else:
            print(f"ok   {path}  ({spec.name}, {len(spec.steps)} steps)")
    return 1 if failed else 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        prog="jl_blocks", description="Jacob's Ladder mission blocks"
    )
    commands = parser.add_subparsers(dest="command", required=True)
    check_cmd = commands.add_parser(
        "check", help="check mission files without flying them"
    )
    check_cmd.add_argument("files", nargs="+", help="mission YAML files")
    args = parser.parse_args(argv)
    return check(args.files)


if __name__ == "__main__":
    sys.exit(main())
```

- [ ] **Step 4: Run the check to verify it passes**

Run: `make check`
Expected: all linters pass; `44 passed`.

- [ ] **Step 5: Verify it builds and runs as a ROS package**

The sim container from `docker/run_sim_container.sh` must be running (`docker ps` lists `jacob_ladder_sim`). Run:

```bash
docker exec --user user -w "$PWD" jacob_ladder_sim bash -c '
  source /opt/ros/humble/setup.bash &&
  colcon build --packages-select jl_blocks &&
  source install/setup.bash &&
  printf "name: Hover\nsteps:\n  - hold: {duration: 10}\n" > /tmp/good.yaml &&
  printf "name: Hover\nsteps:\n  - hold: {duraton: 10}\n" > /tmp/bad.yaml &&
  ros2 run jl_blocks jl_blocks check /tmp/good.yaml /tmp/bad.yaml; echo "exit=$?"'
```

Expected output (the last lines):

```
ok   /tmp/good.yaml  (Hover, 1 steps)
FAIL /tmp/bad.yaml
  /tmp/bad.yaml: steps[0] (hold): unknown param 'duraton' for hold; did you mean 'duration'?
exit=1
```

- [ ] **Step 6: Checkpoint**

Stop and report the diff to the maintainer. Do not commit.

---

### Task 6: CI workflow, dependencies and README pointer

**Files:**
- Create: `.github/workflows/check.yml`
- Modify: `pyproject.toml` (`[project] dependencies`), `uv.lock` (regenerated), `README.md` (the "Features in the repository" section, after the "Flight Parameters" subsection)

**Interfaces:**
- Consumes: the `make check` target from Task 1.
- Produces: a GitHub Actions job named `check` that runs on every push and pull request.

- [ ] **Step 1: Add the workflow**

Create `.github/workflows/check.yml`:

```yaml
# Lint, type-check and unit-test the mission blocks on every push and PR.
# Runs the same `make check` developers run locally; the tool versions are
# pinned in the Makefile. No ROS is needed: jl_blocks.core is plain Python.
name: check

on:
  push:
  pull_request:

jobs:
  check:
    runs-on: ubuntu-22.04
    steps:
      - uses: actions/checkout@v4
      - uses: astral-sh/setup-uv@v6
      - run: make check
```

- [ ] **Step 2: Add pytest and PyYAML to the workspace dependencies**

In `pyproject.toml`, in `[project] dependencies`, insert these two lines directly above `"ruff",`:

```toml
    "pytest",
    "pyyaml",
```

Then run: `uv lock`
Expected: exits 0 and reports `Added pytest v9.1.1`. PyYAML 6.0.3 is already in the lock as a transitive dependency, so listing it only makes it direct. The command also reports `Added pyrealsense2 …`, because the current `uv.lock` is already stale: `pyrealsense2` is in `pyproject.toml` but not in the lock. Leave that in and mention it in the checkpoint report.

- [ ] **Step 3: Add the README pointer**

In `README.md`, after the "Flight Parameters" subsection (it ends with the sentence about the `config/params` directory) and before `### Before You Fly`, insert:

```markdown
### Checking your work
`make check` lints, type-checks and unit-tests the mission blocks in
[`src/jl_blocks`](src/jl_blocks), and checks every mission file in `missions/`.
It needs only [uv](https://docs.astral.sh/uv/) (no ROS), takes a few seconds, and
runs automatically on every push through GitHub Actions.
```

- [ ] **Step 4: Verify**

Run: `make check`
Expected: passes, `44 passed`.

Run: `.check-venv/bin/python -c "import yaml; d = yaml.safe_load(open('.github/workflows/check.yml')); print(sorted(d[True]), d['jobs']['check']['runs-on'])"`
Expected: `['pull_request', 'push'] ubuntu-22.04`. PyYAML reads the bare `on:` key as `True`.

The workflow itself runs only on GitHub. Its first real result shows up on the maintainer's next push, so say in the report that this remains unverified until then.

- [ ] **Step 5: Checkpoint**

Stop and report the diff to the maintainer. Do not commit. Include:
- the `pyrealsense2` lock drift from Step 2
- that the CI workflow is unverified until the first push

---

## Maintainer notes (not tasks; found while planning)

- **Pre-existing ruff findings outside `jl_blocks`:** with the locked `ruff` 0.15.20, `ruff check .` reports 23 errors. They are in `src/drogue_flight/launch/drogue_approach.launch.py` (21: undefined names and unused imports) and `installation_scripts/setup_system.py` (2), plus duplicates under `.claude/worktrees/`. `ruff check` also descends into `build/` and `install/` unless they're excluded. That's why `make check` is scoped to `src/jl_blocks` for now.
- **`uv.lock` is stale:** it's missing `pyrealsense2` (see Task 6, Step 2).
- **`make format`** points at an `astylerc` that isn't in the repo (spec §11).

## Next plans

Each is written once the one before it has landed, so that its interfaces come from real code:

1. **Phase 2:** `jl_mission` (C++ relay mode and executor) and `jl_mission_interfaces`, with gtests for the watchdog, validation and limits (spec §5).
2. **Phase 3:** the `mission_runner` node plus ports of `takeoff`, `land`, `search`, `track`, `precision_descend`, `aruco_tag`, `yolo_drogue` and `pid` (spec §3).
3. **Phase 4:** `test/sitl_mission.sh` and the ArUco acceptance mission (spec §8, L3).
4. **Phase 5:** `config/flight.yaml`, `jl_mission@.service.in`, `deploy.sh` and the readiness report, plus the Jetson timing check (spec §7).
5. **Phase 6:** the beginner tutorial (spec §9.6).
