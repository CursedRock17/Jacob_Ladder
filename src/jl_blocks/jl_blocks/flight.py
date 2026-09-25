"""config/flight.yaml: which missions this airframe registers at boot, and the
helper services they need. Read by deploy.sh through `jl_blocks flight ...`.
No ROS here, like jl_blocks.core.
"""

from __future__ import annotations

import difflib
from dataclasses import dataclass
from pathlib import Path

import yaml

from . import library  # noqa: F401  (registers the shipped blocks)
from .core import MissionError, load_block_files, load_mission

# Helper name -> colcon packages it needs built. The unit has the same name.
HELPERS: dict[str, tuple[str, ...]] = {
    "vio": ("oak_d_visual_odometry",),
    # Runs from the separate tracktor-beam workspace (start_aruco_tracker.sh).
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
TOP_KEYS = ("missions", "helpers", "blocks")


@dataclass(frozen=True)
class Flight:
    missions: tuple[Path, ...]
    helpers: tuple[str, ...]
    blocks: tuple[Path, ...]


def _suggest(word: str, choices: list[str]) -> str:
    close = difflib.get_close_matches(word, choices, n=1, cutoff=0.6)
    return f"; did you mean '{close[0]}'?" if close else ""


def load_flight(path: str | Path, root: str | Path) -> Flight:
    source = str(path)
    root = Path(root).resolve()
    doc = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    errors: list[str] = []
    if not isinstance(doc, dict):
        raise MissionError([f"{source}: must be a mapping with missions and helpers"])
    for key in doc:
        if key not in TOP_KEYS:
            errors.append(
                f"{source}: unknown key '{key}'{_suggest(str(key), list(TOP_KEYS))}"
            )

    missions: list[Path] = []
    raw = doc.get("missions") or []
    if not isinstance(raw, list) or not raw:
        errors.append(f"{source}: missions must be a non-empty list of mission files")
        raw = []
    for entry in raw:
        mission = (root / str(entry)).resolve()
        if mission.parent != root / "missions" or mission.suffix != ".yaml":
            errors.append(
                f"{source}: {entry} must be a file in missions/ ending in .yaml"
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

    blocks = tuple((root / str(b)).resolve() for b in doc.get("blocks") or [])
    if errors:
        raise MissionError(errors)
    return Flight(tuple(missions), tuple(helpers), blocks)


def check_flight(flight: Flight) -> list[str]:
    errors = load_block_files([str(b) for b in flight.blocks])
    names: dict[str, Path] = {}
    for mission in flight.missions:
        try:
            spec = load_mission(mission)
        except MissionError as err:
            errors.extend(err.errors)
            continue
        if spec.name in RESERVED_NAMES:
            errors.append(
                f"{mission}: '{spec.name}' is already registered by another boot service; "
                "pick another name"
            )
        if spec.name in names:
            errors.append(
                f"{mission}: two missions are named '{spec.name}' "
                f"(also {names[spec.name].name}); names must be unique"
            )
        names[spec.name] = mission
    return errors


def units(flight: Flight) -> list[str]:
    return [f"jl_mission@{m.stem}" for m in flight.missions] + list(flight.helpers)


def packages(flight: Flight) -> list[str]:
    out = ["jl_blocks", "jl_mission"]
    for helper in flight.helpers:
        out.extend(p for p in HELPERS[helper] if p not in out)
    return out
