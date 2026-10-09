"""A mission is a Python function that flies a Drone.

    @mission("LandBeforeAprilTag", needs=[TAG])
    def land_before_apriltag(drone):
        ...

`name` is what appears in QGC (1-24 letters, digits and underscores, starting
with a letter: PX4's rule for external modes). `needs` lists the perception it
uses, so whoever runs the mission (the simulator, the tests, deploy) can start
the right trackers. Importing the file defines the mission without flying it.
"""

from __future__ import annotations

import importlib.util
import re
import sys
from collections.abc import Callable, Sequence
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from .airframes import Airframe, get_airframe
from .backend import Backend
from .drone import Drone
from .perception import Helper, Perception

NAME_PATTERN = re.compile(r"^[A-Za-z][A-Za-z0-9_]{0,23}$")
# The Drone options a mission may set; the rest are the runner's business.
DRONE_OPTIONS = ("default_height", "velocity", "select_timeout")


class MissionError(Exception):
    """A mission file that cannot be loaded or is not valid."""


@dataclass(frozen=True)
class MissionSpec:
    name: str
    fn: Callable[[Drone], None]
    needs: tuple[Perception, ...] = ()
    options: dict[str, Any] = field(default_factory=dict)
    path: str | None = None

    def __call__(self, drone: Drone) -> None:
        self.fn(drone)

    def helpers(self, airframe: Airframe | str | None = None) -> list[Helper]:
        """The processes this mission needs running, de-duplicated."""
        airframe = (
            airframe if isinstance(airframe, Airframe) else get_airframe(airframe)
        )
        found: dict[str, Helper] = {}
        for need in self.needs:
            for helper in need.helpers(airframe):
                found.setdefault(helper.name, helper)
        return list(found.values())

    def check(self, airframe: Airframe | str | None = None) -> None:
        """Raise MissionError if the mission cannot run on this airframe."""
        airframe = (
            airframe if isinstance(airframe, Airframe) else get_airframe(airframe)
        )
        for need in self.needs:
            try:
                need.validate(airframe)
            except KeyError as err:
                raise MissionError(f"{self.name}: {err.args[0]}") from None


def mission(
    name: str, *, needs: Sequence[Perception] = (), **options: Any
) -> Callable[[Callable[[Drone], None]], MissionSpec]:
    if not NAME_PATTERN.match(name):
        raise MissionError(
            f"mission name '{name}' must be 1-24 letters, digits or underscores, "
            "starting with a letter (PX4 limits external mode names)"
        )
    unknown = sorted(set(options) - set(DRONE_OPTIONS))
    if unknown:
        raise MissionError(
            f"unknown mission option(s) {', '.join(unknown)} (known: {', '.join(DRONE_OPTIONS)})"
        )
    for need in needs:
        if not isinstance(need, Perception):
            raise MissionError(
                f"needs= takes perception declarations like AprilTag(...), got {need!r}"
            )

    def define(fn: Callable[[Drone], None]) -> MissionSpec:
        return MissionSpec(name, fn, tuple(needs), dict(options))

    return define


def load(path: str | Path) -> MissionSpec:
    """Import a mission file and return the one mission it defines."""
    path = Path(path).resolve()
    if not path.is_file():
        raise MissionError(f"{path}: file not found")
    module_name = f"jl_mission_{path.stem}"
    spec = importlib.util.spec_from_file_location(module_name, path)
    if spec is None or spec.loader is None:
        raise MissionError(f"{path}: not a Python file")
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    try:
        spec.loader.exec_module(module)
    except MissionError:
        raise
    except Exception as err:
        raise MissionError(f"{path}: {type(err).__name__}: {err}") from err
    found = [v for v in vars(module).values() if isinstance(v, MissionSpec)]
    if not found:
        raise MissionError(
            f"{path}: defines no mission (decorate a function with @mission)"
        )
    if len(found) > 1:
        names = ", ".join(sorted(m.name for m in found))
        raise MissionError(
            f"{path}: defines {len(found)} missions ({names}); one file, one mission"
        )
    return MissionSpec(
        found[0].name, found[0].fn, found[0].needs, found[0].options, str(path)
    )


def run(
    spec: MissionSpec,
    *,
    airframe: Airframe | str | None = None,
    backend: Backend | None = None,
    fresh_selection: bool = False,
    report_every: float | None = None,
    log: Callable[[str], None] = print,
) -> None:
    """Wait for the mission to be selected, fly it, and land (even after an error).

    report_every: print what is being commanded (setpoint, drone, targets) at
    most this often, in seconds. None: never.
    """
    spec.check(airframe)
    with Drone(
        spec.name,
        backend=backend,
        airframe=airframe,
        needs=spec.needs,
        take_off_on_enter=False,
        fresh_selection=fresh_selection,
        report_every=report_every,
        log=log,
        **spec.options,
    ) as drone:
        spec.fn(drone)
