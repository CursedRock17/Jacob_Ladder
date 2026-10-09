"""config/flight.yaml: what this aircraft registers at boot. Read by deploy.sh
through `python3 -m jl_flight flight ...`. No ROS here.

    airframe: x500_rgbd            # see jl_flight/airframes.py
    missions:                      # each becomes a jl_mission@<file> boot service
      - missions/land_before_apriltag.py
    helpers: [vio, battery_monitor]  # boot services the aircraft needs besides missions

The trackers a mission declares in `needs=` are not helpers: each mission's
service starts its own (mission.launch.py), so they come and go with it.
"""

from __future__ import annotations

import difflib
from dataclasses import dataclass
from pathlib import Path

import yaml

from .airframes import AIRFRAMES, get_airframe
from .missions import MissionError, MissionSpec, load

# Helper name -> colcon packages it needs built. The unit has the same name.
HELPERS: dict[str, tuple[str, ...]] = {
    "vio": ("oak_d_visual_odometry",),
    # Runs from the separate tracktor-beam workspace (start_aruco_tracker.sh), for
    # the C++ precision_land modes. jl_flight missions start their own trackers.
    "aruco_tracker": (),
    # Plain Python module run by run_battery_monitor.sh; nothing to build.
    "battery_monitor": (),
}
# Mode names other boot services already register with PX4.
RESERVED_NAMES = frozenset(
    {
        "BlankMode",
        "DroneSmoothPlanner",
        "FrontApproach",
        "FrontToPrecisionLand",
        "MyModeCustom",
        "PrecisionLandAutoCustom",
        "PrecisionLandCustom",
        "TakeoffHold",
        "TakeoffLand",
    }
)
TOP_KEYS = ("airframe", "missions", "helpers")


@dataclass(frozen=True)
class Flight:
    airframe: str
    missions: tuple[Path, ...]
    helpers: tuple[str, ...]


def _suggest(word: str, choices: list[str]) -> str:
    close = difflib.get_close_matches(word, choices, n=1, cutoff=0.6)
    return f"; did you mean '{close[0]}'?" if close else ""


def load_flight(path: str | Path, root: str | Path) -> Flight:
    source = str(path)
    root = Path(root).resolve()
    doc = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    errors: list[str] = []
    if not isinstance(doc, dict):
        raise MissionError(
            f"{source}: must be a mapping with airframe, missions and helpers"
        )
    for key in doc:
        if key not in TOP_KEYS:
            errors.append(
                f"{source}: unknown key '{key}'{_suggest(str(key), list(TOP_KEYS))}"
            )

    airframe = str(doc.get("airframe") or "generic")
    if airframe not in AIRFRAMES:
        errors.append(
            f"{source}: unknown airframe '{airframe}'{_suggest(airframe, list(AIRFRAMES))}"
            f" (known: {', '.join(sorted(AIRFRAMES))})"
        )

    missions: list[Path] = []
    raw = doc.get("missions") or []
    if not isinstance(raw, list) or not raw:
        errors.append(f"{source}: missions must be a non-empty list of mission files")
        raw = []
    for entry in raw:
        mission = (root / str(entry)).resolve()
        if mission.parent != root / "missions" or mission.suffix != ".py":
            errors.append(
                f"{source}: {entry} must be a file in missions/ ending in .py"
            )
        elif not mission.is_file():
            errors.append(f"{source}: {entry}: file not found")
        else:
            missions.append(mission)

    helpers: list[str] = []
    for helper in doc.get("helpers") or []:
        if helper not in HELPERS:
            errors.append(
                f"{source}: unknown helper '{helper}'{_suggest(str(helper), list(HELPERS))}"
                f" (known: {', '.join(HELPERS)})"
            )
        elif helper not in helpers:
            helpers.append(helper)

    if errors:
        raise MissionError("\n".join(errors))
    return Flight(airframe, tuple(missions), tuple(helpers))


def load_missions(flight: Flight) -> tuple[list[MissionSpec], list[str]]:
    """Every listed mission that loads and checks out, and the errors from the rest."""
    specs: list[MissionSpec] = []
    errors: list[str] = []
    names: dict[str, Path] = {}
    for path in flight.missions:
        try:
            spec = load(path)
            spec.check(flight.airframe)
        except MissionError as err:
            errors.append(str(err))
            continue
        if spec.name in RESERVED_NAMES:
            errors.append(
                f"{path}: '{spec.name}' is already registered by another boot service; "
                "pick another name"
            )
        elif spec.name in names:
            errors.append(
                f"{path}: two missions are named '{spec.name}' "
                f"(also {names[spec.name].name}); names must be unique"
            )
        else:
            names[spec.name] = path
            specs.append(spec)
    return specs, errors


def units(flight: Flight) -> list[str]:
    return [f"jl_mission@{m.stem}" for m in flight.missions] + list(flight.helpers)


def packages(flight: Flight, specs: list[MissionSpec]) -> list[str]:
    """What colcon must build: the flight stack, the helpers', and every package a
    mission's perception runs from."""
    out = ["jl_flight", "jl_mission"]
    wanted = [p for h in flight.helpers for p in HELPERS[h]]
    wanted += [p for spec in specs for need in spec.needs for p in need.packages]
    for package in wanted:
        if package not in out:
            out.append(package)
    return out


def topics(specs: list[MissionSpec]) -> list[str]:
    """Every perception topic the missions read, for the readiness report."""
    found: list[str] = []
    for spec in specs:
        for need in spec.needs:
            if need.topic not in found:
                found.append(need.topic)
    return found


def airframe_name(flight: Flight) -> str:
    return get_airframe(flight.airframe).name
