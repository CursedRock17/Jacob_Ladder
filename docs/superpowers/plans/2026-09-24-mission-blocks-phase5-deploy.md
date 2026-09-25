# Mission Blocks Phase 5: `config/flight.yaml`, `jl_mission@.service`, `deploy.sh` and the readiness report — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Meet spec §1's success criterion. A researcher adds one line to `config/flight.yaml` and runs `./deploy.sh` on the drone. The mission then appears in QGC after every boot, and the command reports whether the vehicle is ready.

**Architecture:**
- **What the airframe flies** lives in `config/flight.yaml`. A ROS-free module, `jl_blocks.flight`, reads it and checks it; the `jl_blocks flight …` CLI exposes that.
- **One systemd template unit**, `jl_mission@<mission>.service`, runs `mission.launch.py` for one mission file. The runner is respawned if it dies, and it won't resume a flight (Phase 3).
- **`deploy.sh`** runs spec §7's five steps in order, stopping at the first failure: pull, check, build, enable/disable units, readiness report.
- **The readiness report** is a small ROS node, `readiness`, plus journal checks. Its pass/fail logic is plain Python with unit tests.
- **`setpoint_timing`** measures setpoint inter-arrival, to repeat spec §6's spike measurement on the Jetson.

**Tech Stack:** bash, systemd, Python 3.10, rclpy (ROS 2 Humble), pytest 9.1.1, ruff 0.15.20, ty 0.0.55, PX4 v1.16.0 SITL in `jacob_ladder_sim`.

**Spec:** `docs/superpowers/specs/2026-09-21-mission-blocks-design.md`. This plan implements phase 5 of §9: §7 Deployment in full, and "repeat the spike's timing measurement on the Jetson" (§6), as a tool plus a SITL baseline. The Jetson run itself needs the maintainer and the drone.

## Global Constraints

- **No flight-code changes.** Do NOT modify `src/precision_land/`, `src/drogue_flight/`, `src/aruco_tracker/`, `src/ros2_yolo_image_processing/`, the `jl_mission` C++, or `gazebo/`.
- **Existing boot units stay as they are** (spec §7: "The existing boot units (`dds_agent`, `translation_node`, `vio`) are unchanged"). Do not edit any existing `services/*.service.in` or `services/run_*.sh`. `deploy.sh` never enables, disables or restarts `dds_agent` or `translation_node`. The one allowed change to `services/install_services.sh` is Task 2's "skip template units when enabling".
- **Do not commit.** The maintainer makes all commits. Each task ends with a checkpoint: stop and report the diff. Never run `git add/commit/stash/checkout/reset/restore`, and never run `deploy.sh` for real on this machine: only `--dry-run` and the stubbed tests.
- Never run `sudo`, `systemctl` or `install_services.sh` without `--dry-run` on the development machine. Those belong to the drone.
- Work only on `main`; never add yourself to commit history.
- `jl_blocks.flight` is ROS-free (standard library + yaml), like `jl_blocks.core`. Only `jl_blocks/ros/` imports rclpy.
- `make check` must pass after every task.
- Spec §7, verbatim, for `deploy.sh`, run on the Jetson in this order and stopping at the first failure:
  1. "`git pull`, refusing if there are uncommitted local changes"
  2. "`jl_blocks check` on every listed mission. A bad file stops here, and nothing already running is touched."
  3. "`colcon build --packages-up-to jl_blocks jl_mission` plus the helpers' packages, and nothing else."
  4. "Enable and start `jl_mission@<name>` (relay + executor + runner) for each listed mission and each listed helper; disable those not listed."
  5. "Readiness report, one line per check"

  `./deploy.sh --check` runs only step 5.
- The spec §7 readiness lines are: DDS agent session established (from the journal), FC data arriving (`/fmu/out/vehicle_status`), VIO publishing (`/fmu/in/vehicle_visual_odometry`, with a rate), Registered in PX4 (the mission names), and `aruco_tracker` (detections; "fine if no tag in view").
- Mission files listed in `config/flight.yaml` must live in `missions/`. The unit instance is the file's stem, e.g. `missions/takeoff_hold_land.yaml` → `jl_mission@takeoff_hold_land`.
- Tests run on the host as part of `make check`. That includes `test/test_deploy.py`, which runs `deploy.sh` against stub commands.

## Rulings (confirm at review)

1. **"Disable those not listed" applies only to things `flight.yaml` can list:** `jl_mission@*` instances and the known helpers (`vio`, `aruco_tracker`, `battery_monitor`). The core link (`dds_agent`, `translation_node`) and the existing `takeoff_hold` service are never touched. So a `flight.yaml` without `vio` stops VIO, and the shipped `flight.yaml` lists it.
2. **Deploy refuses while the vehicle is armed.** Restarting units mid-flight would pull the active mode out from under the pilot. The readiness node's `--armed` probe answers this, and a timeout counts as "don't know". Deploy then refuses unless `--force-unknown-arm-state` is given.
3. **The runner is respawned by launch, not by systemd.** `mission.launch.py` gains `respawn_runner:=true` for the service. `jl_mission` stays up the whole time, so its silence watchdog still holds, then lands (spec §5). A respawned runner ignores the latched `/active` (Phase 3), so it never resumes mid-flight.
4. **A mission name that another boot service already registers** (`TakeoffHold` from `takeoff_hold.service`, `DroneSmoothPlanner`, and the precision_land modes) is rejected by `jl_blocks flight check`. Two modes with one name would confuse PX4 and QGC.

## Review Focus

1. **Deploying with uncommitted local changes** must refuse before touching anything: no pull, build or unit change, and exit 1. Owner: Task 4, `test_a_dirty_tree_refuses_before_anything`.
2. **A bad mission file in `flight.yaml`** must stop before the build, and running units must be left untouched. Owner: Task 4, `test_a_bad_mission_stops_before_build_and_touches_no_unit`.
3. **A mission removed from `flight.yaml`** must have its `jl_mission@` instance disabled and stopped, and the others must be left enabled. Owner: Task 4, `test_a_removed_mission_is_disabled`.
4. **Two listed missions with the same `name:`, or a name another boot mode already uses,** must be rejected by the check. Owner: Task 1, `test_duplicate_names_are_rejected` and `test_a_name_taken_by_an_existing_boot_mode_is_rejected`.
5. **Deploying while the vehicle is armed** must refuse. Owner: Task 4, `test_an_armed_vehicle_refuses`.

---

## File Structure

```
config/flight.yaml                     NEW: missions + helpers for this airframe
src/jl_blocks/jl_blocks/
├── flight.py                          NEW: load/check flight.yaml (no ROS)
├── readiness.py                       NEW: readiness verdicts + formatting (no ROS)
├── timing.py                          NEW: inter-arrival stats (no ROS)
├── cli.py                             + `jl_blocks flight check|units|packages|blocks`
└── ros/
    ├── readiness_probe.py             NEW: samples topics, prints readiness lines / arm state
    └── setpoint_timing.py             NEW: measures /jl/NAME/setpoint timing
src/jl_blocks/launch/mission.launch.py + respawn_runner arg
src/jl_blocks/setup.py                 + two entry points
src/jl_blocks/test/test_flight.py, test_readiness.py, test_timing.py   NEW
services/jl_mission@.service.in        NEW
services/run_jl_mission.sh             NEW
services/install_services.sh           skip template units on --enable
deploy.sh                              NEW (repo root)
test/test_deploy.py                    NEW: deploy.sh against stub commands
Makefile                               check also runs test/test_deploy.py
README.md                              "Deploying a mission" section
docs/superpowers/specs/...design.md    §6 Jetson row placeholder, §11 notes
```

---

### Task 1: `config/flight.yaml` and `jl_blocks.flight`

**Files:**
- Create: `config/flight.yaml`, `src/jl_blocks/jl_blocks/flight.py`, `src/jl_blocks/test/test_flight.py`
- Modify: `src/jl_blocks/jl_blocks/cli.py`

**Interfaces:**
- Consumes: `load_mission`, `load_block_files`, `MissionError` (`jl_blocks.core`); `library` (registers the shipped blocks).
- Produces:
  - `HELPERS: dict[str, tuple[str, ...]]`: helper name → the colcon packages it needs built. `vio` → `("oak_d_visual_odometry",)`, `aruco_tracker` → `()`, `battery_monitor` → `()`.
  - `RESERVED_NAMES: frozenset[str]`: mode names other boot services register.
  - `Flight` (frozen dataclass): `missions: tuple[Path, ...]`, `helpers: tuple[str, ...]`, `blocks: tuple[Path, ...]`.
  - `load_flight(path: str | Path, root: str | Path) -> Flight`: raises `MissionError` listing every problem. `root` is the repo root, and relative paths resolve against it.
  - `check_flight(flight: Flight) -> list[str]`: loads the block files and every mission, and returns one error line per problem. An empty list means OK.
  - `units(flight) -> list[str]`: e.g. `["jl_mission@takeoff_hold_land", "vio"]`.
  - `packages(flight) -> list[str]`: `["jl_blocks", "jl_mission", …helper packages]`, de-duplicated, in order.
  - CLI: `jl_blocks flight check|units|packages|blocks FLIGHT_YAML [--root DIR]`.
    - `check` prints `ok …` lines, or `FAIL …` lines and exits 1.
    - `units` and `packages` print one name per line.
    - `blocks` prints the comma-joined absolute block paths (empty if none).

- [ ] **Step 1: Write the failing tests**

Create `src/jl_blocks/test/test_flight.py`:

```python
from __future__ import annotations

import pytest

from jl_blocks import cli
from jl_blocks.core import MissionError
from jl_blocks.flight import check_flight, load_flight, packages, units

GOOD = "name: {name}\nsteps:\n  - takeoff: {{height: 1.5}}\n  - land: {{}}\n"


def repo(tmp_path, flight_text, missions=None):
    (tmp_path / "missions").mkdir()
    for stem, name in (missions or {"a": "MissionA"}).items():
        (tmp_path / "missions" / f"{stem}.yaml").write_text(GOOD.format(name=name))
    (tmp_path / "config").mkdir()
    path = tmp_path / "config" / "flight.yaml"
    path.write_text(flight_text)
    return path


def test_a_good_flight_file_loads(tmp_path):
    path = repo(tmp_path, "missions:\n  - missions/a.yaml\nhelpers: [vio]\n")
    flight = load_flight(path, tmp_path)
    assert flight.missions == ((tmp_path / "missions" / "a.yaml").resolve(),)
    assert flight.helpers == ("vio",)
    assert check_flight(flight) == []


def test_units_and_packages(tmp_path):
    path = repo(
        tmp_path,
        "missions:\n  - missions/a.yaml\n  - missions/b.yaml\nhelpers: [vio, battery_monitor]\n",
        {"a": "MissionA", "b": "MissionB"},
    )
    flight = load_flight(path, tmp_path)
    assert units(flight) == [
        "jl_mission@a", "jl_mission@b", "vio", "battery_monitor",
    ]
    assert packages(flight) == ["jl_blocks", "jl_mission", "oak_d_visual_odometry"]


def test_unknown_helper_is_rejected_with_the_choices(tmp_path):
    path = repo(tmp_path, "missions:\n  - missions/a.yaml\nhelpers: [vi0]\n")
    with pytest.raises(MissionError, match="unknown helper 'vi0'; did you mean 'vio'"):
        load_flight(path, tmp_path)


def test_a_mission_outside_missions_folder_is_rejected(tmp_path):
    path = repo(tmp_path, "missions:\n  - elsewhere/a.yaml\n")
    with pytest.raises(MissionError, match="must be a file in missions/"):
        load_flight(path, tmp_path)


def test_a_missing_mission_file_is_rejected(tmp_path):
    path = repo(tmp_path, "missions:\n  - missions/nope.yaml\n")
    with pytest.raises(MissionError, match="missions/nope.yaml: file not found"):
        load_flight(path, tmp_path)


def test_unknown_top_level_key_is_rejected(tmp_path):
    path = repo(tmp_path, "missions:\n  - missions/a.yaml\nhelper: [vio]\n")
    with pytest.raises(MissionError, match="unknown key 'helper'; did you mean 'helpers'"):
        load_flight(path, tmp_path)


def test_duplicate_names_are_rejected(tmp_path):
    path = repo(
        tmp_path,
        "missions:\n  - missions/a.yaml\n  - missions/b.yaml\n",
        {"a": "Same", "b": "Same"},
    )
    errors = check_flight(load_flight(path, tmp_path))
    assert any("two missions are named 'Same'" in e for e in errors)


def test_a_name_taken_by_an_existing_boot_mode_is_rejected(tmp_path):
    path = repo(tmp_path, "missions:\n  - missions/a.yaml\n", {"a": "TakeoffHold"})
    errors = check_flight(load_flight(path, tmp_path))
    assert any("'TakeoffHold' is already registered by another boot service" in e for e in errors)


def test_a_bad_mission_file_is_reported(tmp_path):
    path = repo(tmp_path, "missions:\n  - missions/a.yaml\n")
    (tmp_path / "missions" / "a.yaml").write_text("name: A\nsteps:\n  - holdd: {}\n")
    errors = check_flight(load_flight(path, tmp_path))
    assert any("did you mean 'hold'" in e for e in errors)


def test_cli_check_and_units(tmp_path, capsys):
    path = repo(tmp_path, "missions:\n  - missions/a.yaml\nhelpers: [vio]\n")
    assert cli.main(["flight", "check", str(path), "--root", str(tmp_path)]) == 0
    assert "ok" in capsys.readouterr().out
    assert cli.main(["flight", "units", str(path), "--root", str(tmp_path)]) == 0
    assert capsys.readouterr().out.split() == ["jl_mission@a", "vio"]


def test_cli_check_fails_on_a_bad_file(tmp_path, capsys):
    path = repo(tmp_path, "missions:\n  - missions/nope.yaml\n")
    assert cli.main(["flight", "check", str(path), "--root", str(tmp_path)]) == 1
    assert "FAIL" in capsys.readouterr().out


def test_the_shipped_flight_file_checks_out():
    from pathlib import Path

    root = Path(__file__).resolve().parents[3]
    assert check_flight(load_flight(root / "config" / "flight.yaml", root)) == []
```

- [ ] **Step 2: Run the tests and watch them fail**

Run: `make check`
Expected: FAIL: `No module named 'jl_blocks.flight'`.

- [ ] **Step 3: Implement**

Create `src/jl_blocks/jl_blocks/flight.py`:

```python
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
        "TakeoffHold",
        "DroneSmoothPlanner",
        "PrecisionLandCustom",
        "FrontApproach",
        "FrontToPrecisionLand",
        "PrecisionLandAuto",
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
            errors.append(f"{source}: unknown key '{key}'{_suggest(str(key), list(TOP_KEYS))}")

    missions: list[Path] = []
    raw = doc.get("missions") or []
    if not isinstance(raw, list) or not raw:
        errors.append(f"{source}: missions must be a non-empty list of mission files")
        raw = []
    for entry in raw:
        mission = (root / str(entry)).resolve()
        if mission.parent != root / "missions" or mission.suffix != ".yaml":
            errors.append(f"{source}: {entry} must be a file in missions/ ending in .yaml")
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
```

In `src/jl_blocks/jl_blocks/cli.py`, add a `flight` subcommand. After the `check_cmd` block in `main`:

```python
    flight_cmd = commands.add_parser(
        "flight", help="check or read config/flight.yaml (used by deploy.sh)"
    )
    flight_cmd.add_argument("action", choices=["check", "units", "packages", "blocks"])
    flight_cmd.add_argument("file", help="the flight YAML, e.g. config/flight.yaml")
    flight_cmd.add_argument(
        "--root", default=".", help="repo root that mission paths are relative to"
    )
```

and dispatch on `args.command`:

```python
    args = parser.parse_args(argv)
    if args.command == "flight":
        return flight(args.action, args.file, args.root)
    return check(args.files, args.blocks)
```

with this function above `main`:

```python
def flight(action: str, path: str, root: str) -> int:
    from .flight import check_flight, load_flight, packages, units

    try:
        config = load_flight(path, root)
    except (OSError, MissionError) as err:
        lines = err.errors if isinstance(err, MissionError) else [str(err)]
        print(f"FAIL {path}")
        for line in lines:
            print(f"  {line}")
        return 1
    if action == "check":
        errors = check_flight(config)
        if errors:
            print(f"FAIL {path}")
            for line in errors:
                print(f"  {line}")
            return 1
        print(f"ok   {path}  ({len(config.missions)} missions, helpers: "
              f"{', '.join(config.helpers) or 'none'})")
    elif action == "units":
        print("\n".join(units(config)))
    elif action == "packages":
        print("\n".join(packages(config)))
    else:
        print(",".join(str(b) for b in config.blocks))
    return 0
```

Create `config/flight.yaml`:

```yaml
# What this airframe registers at boot (spec section 7). After editing, run
# ./deploy.sh on the drone: it checks, builds, and enables one
# jl_mission@<file> service per mission below, then reports readiness.
#
# missions: files in missions/ -- each appears in QGC under its `name:`.
# helpers:  boot services the missions need: vio, aruco_tracker, battery_monitor.
#           A helper left out here is stopped and disabled by deploy.sh.
# blocks:   optional folders or .py files with your own blocks.
missions:
  - missions/takeoff_hold_land.yaml
helpers: [vio, battery_monitor]
```

- [ ] **Step 4: Run the tests and watch them pass**

Run: `make check`
Expected: every stage passes, with 12 new tests.

- [ ] **Step 5: Checkpoint**

Stop and report the diff to the maintainer. Do not commit.

---

### Task 2: `jl_mission@.service`, the run script, launch respawn, and `install_services.sh`

**Files:**
- Create: `services/jl_mission@.service.in`, `services/run_jl_mission.sh` (executable)
- Modify: `src/jl_blocks/launch/mission.launch.py`, `services/install_services.sh`

**Interfaces:**
- Consumes: `jl_blocks flight blocks` (Task 1), `mission.launch.py`.
- Produces:
  - unit template `jl_mission@.service`: `%i` is the mission file's stem, and `ExecStart` runs `run_jl_mission.sh %i`;
  - launch argument `respawn_runner` (default `false`);
  - `install_services.sh --enable` skips template units, i.e. names ending in `@`.

- [ ] **Step 1: Launch respawn**

In `src/jl_blocks/launch/mission.launch.py`:
- declare `DeclareLaunchArgument("respawn_runner", default_value="false", description="restart mission_runner if it exits (the boot service sets true)")`;
- in `_nodes`, read it with `respawn = LaunchConfiguration("respawn_runner").perform(context).lower() == "true"`;
- pass `respawn=respawn, respawn_delay=2.0` to the `mission_runner` Node.

Keep the existing OnProcessExit `LogInfo`. Update the module docstring to say that with `respawn_runner:=true` the runner comes back after 2 s and ignores a mission that was already active (so it never resumes mid-flight), while jl_mission holds, then lands.

- [ ] **Step 2: The unit and its run script**

Create `services/jl_mission@.service.in`:

```ini
[Unit]
Description=Mission %i (jl_mission + mission_runner, from missions/%i.yaml)
Documentation=file://@JL_WS_ROOT@/config/flight.yaml
# Needs the DDS link to reach PX4, but not the network: the airframe flies
# with no WiFi. Same ordering as takeoff_hold.service.
After=dds_agent.service translation_node.service
Wants=dds_agent.service
# Registration retries until the FMU answers; never give up.
StartLimitIntervalSec=0

[Service]
Type=simple
User=@JL_USER@
Group=@JL_GROUP@
WorkingDirectory=@JL_WS_ROOT@
ExecStart=/bin/bash @JL_WS_ROOT@/services/run_jl_mission.sh %i
# always: `ros2 launch` can exit 0 when its nodes die.
Restart=always
RestartSec=5s
StandardOutput=journal
StandardError=journal
SyslogIdentifier=jl_mission_%i

[Install]
WantedBy=multi-user.target
```

Create `services/run_jl_mission.sh` (`chmod +x`):

```bash
#!/bin/bash
# One mission from config/flight.yaml, as a boot service: jl_mission (the mode
# shown in QGC) plus the mission_runner, via mission.launch.py.
#   $1: the mission file's stem, e.g. takeoff_hold_land for missions/takeoff_hold_land.yaml
# The runner is respawned if it dies; it will not resume a flight that was
# already active (jl_mission holds, then lands, meanwhile).
source "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/../jl_env.sh"
cd "$JL_WS_ROOT" || exit 1
jl_source_ros

mission="$JL_WS_ROOT/missions/$1.yaml"
if [ ! -f "$mission" ]; then
    echo "run_jl_mission: $mission not found (check config/flight.yaml, then ./deploy.sh)" >&2
    exit 1
fi
blocks="$(jl_blocks flight blocks "$JL_WS_ROOT/config/flight.yaml" --root "$JL_WS_ROOT")"

exec ros2 launch jl_blocks mission.launch.py \
    mission_file:="$mission" blocks:="$blocks" respawn_runner:=true
```

- [ ] **Step 3: `install_services.sh` skips templates when enabling**

In the `--enable` block of `services/install_services.sh`, filter out template units before `systemctl enable`:

```bash
if [ "$do_enable" -eq 1 ]; then
    # A template (name@) can only be enabled per instance, e.g.
    # jl_mission@takeoff_hold_land -- deploy.sh does that.
    enable_units=()
    for unit in "${units[@]}"; do
        [[ "$unit" == *@ ]] || enable_units+=("$unit")
    done
    echo
    sudo systemctl enable --now "${enable_units[@]}"
    echo
    for unit in "${enable_units[@]}"; do
        printf '%-20s %s\n' "$unit" "$(systemctl is-active "$unit" 2>&1)"
    done
```

Keep the `else` branch as it is.

- [ ] **Step 4: Verify on the host and in the container**

On the host:

```bash
./services/install_services.sh --dry-run jl_mission@ | sed -n '1,30p'
bash -n services/run_jl_mission.sh && echo syntax-ok
make check
```

Expected:
- the rendered unit, with this checkout's path in place of `@JL_WS_ROOT@` and your user in place of `@JL_USER@`;
- `syntax-ok`;
- `make check` green.

In the container (restart it first), run the service's script by hand against SITL, as a unit would. Use `test/sitl_common.sh`'s `start_px4`, and set `GZ_PARTITION=verify_unit`. Then:

```bash
services/run_jl_mission.sh takeoff_hold_land > /tmp/unit.log 2>&1 &
sleep 20; grep -c "Registered 'TakeoffHoldLand'" /tmp/unit.log
pkill -9 -f lib/jl_blocks/mission_runner; sleep 5
grep -c "process started" /tmp/unit.log   # the runner came back
services/run_jl_mission.sh nope; echo "exit=$?"
```

Expected:
- `1` registration line;
- two or more `process started` lines for `mission_runner` (the original, then the respawn);
- for the missing mission: `run_jl_mission: …/missions/nope.yaml not found …` and `exit=1`.

Kill everything afterwards.

- [ ] **Step 5: Checkpoint**

Stop and report the diff and the Step 4 output to the maintainer. Do not commit.

---

### Task 3: The readiness report

**Files:**
- Create: `src/jl_blocks/jl_blocks/readiness.py`, `src/jl_blocks/jl_blocks/ros/readiness_probe.py`, `src/jl_blocks/test/test_readiness.py`
- Modify: `src/jl_blocks/setup.py` (entry point `readiness`)

**Interfaces:**
- Consumes: mission names (from each file's `name:`), and helper names (Task 1).
- Produces:
  - `readiness.Check(name: str, ok: bool | None, detail: str)`: `None` means "could not tell";
  - `readiness.rate_hz(stamps: list[float]) -> float`;
  - `readiness.format_report(checks) -> list[str]`: `✓`, `✗` or `?`, then name padded to 34, then detail;
  - `readiness.all_ok(checks) -> bool`: `ok is not False` for every check, except those listed in `advisory`;
  - `ros2 run jl_blocks readiness --names NAME [NAME…] [--helpers H…] [--seconds 5]` prints the report and exits 0 if all_ok, else 1;
  - `ros2 run jl_blocks readiness --armed` prints `armed`, `disarmed` or `unknown`, and exits 0, 3 or 4.

The report lines, following spec §7:

| Check | ok when |
|---|---|
| DDS agent session established | the `dds_agent` journal since boot contains `session established` (`?` if journalctl is unavailable, e.g. in a container) |
| FC data arriving | a `/fmu/out/vehicle_status` message within `--seconds` |
| VIO publishing | ≥ 20 Hz on `/fmu/in/vehicle_visual_odometry`, only if `vio` is a helper |
| Registered in PX4 | for each name, `/jl/NAME/active` has a publisher and the `jl_mission@` journal (or, without journald, nothing) shows `Registered 'NAME'` |
| aruco_tracker | only if `aruco_tracker` is a helper: a detection on `/front/target_pose` within `--seconds`. Advisory: ✗ here doesn't fail the report ("fine if no tag in view") |

- [ ] **Step 1: Write the failing tests**

Create `src/jl_blocks/test/test_readiness.py`:

```python
from __future__ import annotations

from jl_blocks.readiness import Check, all_ok, format_report, rate_hz


def test_rate_from_stamps():
    assert rate_hz([0.0, 0.1, 0.2, 0.3, 0.4]) == 10.0
    assert rate_hz([1.0]) == 0.0
    assert rate_hz([]) == 0.0


def test_format_uses_marks_and_aligns():
    lines = format_report(
        [
            Check("FC data arriving", True, "/fmu/out/vehicle_status"),
            Check("VIO publishing", False, "/fmu/in/vehicle_visual_odometry  3 Hz"),
            Check("DDS agent session established", None, "journalctl unavailable"),
        ]
    )
    assert lines[0] == f"✓ {'FC data arriving':<34}/fmu/out/vehicle_status"
    assert lines[1].startswith("✗ VIO publishing")
    assert lines[2].startswith("? DDS agent session established")


def test_unknown_is_not_a_failure_but_false_is():
    assert all_ok([Check("a", True, ""), Check("b", None, "")])
    assert not all_ok([Check("a", True, ""), Check("b", False, "")])


def test_advisory_checks_never_fail_the_report():
    checks = [Check("FC data arriving", True, ""), Check("aruco_tracker", False, "")]
    assert all_ok(checks, advisory={"aruco_tracker"})
```

- [ ] **Step 2: Run the tests and watch them fail**

Run: `make check`
Expected: FAIL: `No module named 'jl_blocks.readiness'`.

- [ ] **Step 3: Implement the pure part**

Create `src/jl_blocks/jl_blocks/readiness.py`:

```python
"""The readiness report (spec section 7): one line per check. No ROS here; the
probe node in jl_blocks.ros.readiness_probe gathers the evidence."""

from __future__ import annotations

from collections.abc import Iterable
from dataclasses import dataclass

MARKS = {True: "✓", False: "✗", None: "?"}


@dataclass(frozen=True)
class Check:
    name: str
    ok: bool | None  # None = could not tell (e.g. no journal in a container)
    detail: str


def rate_hz(stamps: list[float]) -> float:
    if len(stamps) < 2 or stamps[-1] <= stamps[0]:
        return 0.0
    return round((len(stamps) - 1) / (stamps[-1] - stamps[0]), 1)


def format_report(checks: Iterable[Check]) -> list[str]:
    return [f"{MARKS[c.ok]} {c.name:<34}{c.detail}" for c in checks]


def all_ok(checks: Iterable[Check], advisory: set[str] | None = None) -> bool:
    skip = advisory or set()
    return all(c.ok is not False for c in checks if c.name not in skip)
```

- [ ] **Step 4: The probe node**

Create `src/jl_blocks/jl_blocks/ros/readiness_probe.py`:

```python
"""readiness: the deploy.sh readiness report (spec section 7), or --armed.

Samples the topics for a few seconds, reads the journal where there is one,
and prints one line per check. Exit 0 = ready (advisory checks aside).
"""

from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import VehicleOdometry, VehicleStatus
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from ..readiness import Check, all_ok, format_report, rate_hz

VIO_TOPIC = "/fmu/in/vehicle_visual_odometry"
TAG_TOPIC = "/front/target_pose"


def journal_has(unit: str, text: str) -> bool | None:
    if shutil.which("journalctl") is None:
        return None
    result = subprocess.run(
        ["journalctl", "-b", "-u", unit, "--no-pager", "-o", "cat"],
        capture_output=True,
        text=True,
        check=False,
    )
    if result.returncode != 0 and not result.stdout:
        return None
    return text in result.stdout


class Probe(Node):
    def __init__(self) -> None:
        super().__init__("readiness_probe")
        self.status: VehicleStatus | None = None
        self.vio: list[float] = []
        self.tags = 0
        self.create_subscription(
            VehicleStatus, "/fmu/out/vehicle_status", self.on_status, qos_profile_sensor_data
        )
        self.create_subscription(
            VehicleOdometry, VIO_TOPIC, self.on_vio, qos_profile_sensor_data
        )
        self.create_subscription(PoseStamped, TAG_TOPIC, self.on_tag, qos_profile_sensor_data)

    def on_status(self, msg: VehicleStatus) -> None:
        self.status = msg

    def on_vio(self, msg: VehicleOdometry) -> None:
        self.vio.append(time.monotonic())

    def on_tag(self, msg: PoseStamped) -> None:
        self.tags += 1

    def sample(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)


def report(probe: Probe, names: list[str], helpers: list[str]) -> list[Check]:
    checks = []
    dds = journal_has("dds_agent", "session established")
    checks.append(
        Check("DDS agent session established", dds, "dds_agent journal" if dds is not None
              else "journalctl unavailable")
    )
    checks.append(
        Check("FC data arriving", probe.status is not None, "/fmu/out/vehicle_status")
    )
    if "vio" in helpers:
        hz = rate_hz(probe.vio)
        checks.append(Check("VIO publishing", hz >= 20.0, f"{VIO_TOPIC}  {hz:g} Hz"))
    missing = []
    for name in names:
        has_pub = probe.count_publishers(f"/jl/{name}/active") > 0
        logged = journal_has("jl_mission@*", f"Registered '{name}'")
        if not has_pub or logged is False:
            missing.append(name)
    checks.append(
        Check(
            "Registered in PX4",
            not missing,
            ", ".join(names) if not missing else f"missing: {', '.join(missing)}",
        )
    )
    if "aruco_tracker" in helpers:
        checks.append(
            Check(
                "aruco_tracker",
                probe.tags > 0,
                f"{TAG_TOPIC}  {probe.tags} detections"
                + ("" if probe.tags else " (fine if no tag in view)"),
            )
        )
    return checks


def main() -> None:
    parser = argparse.ArgumentParser(prog="readiness")
    parser.add_argument("--names", nargs="*", default=[])
    parser.add_argument("--helpers", nargs="*", default=[])
    parser.add_argument("--seconds", type=float, default=5.0)
    parser.add_argument("--armed", action="store_true", help="only print the arm state")
    args = parser.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    rclpy.init()
    probe = Probe()
    try:
        probe.sample(args.seconds)
        if args.armed:
            if probe.status is None:
                print("unknown")
                code = 4
            elif probe.status.arming_state == VehicleStatus.ARMING_STATE_ARMED:
                print("armed")
                code = 3
            else:
                print("disarmed")
                code = 0
        else:
            checks = report(probe, args.names, args.helpers)
            for line in format_report(checks):
                print(line)
            code = 0 if all_ok(checks, advisory={"aruco_tracker"}) else 1
    finally:
        probe.destroy_node()
        rclpy.try_shutdown()
    sys.exit(code)


if __name__ == "__main__":
    main()
```

In `setup.py` add `"readiness = jl_blocks.ros.readiness_probe:main",`.

- [ ] **Step 5: Run the unit tests, then try it against SITL**

Run: `make check` (4 new tests).

In the container, restarted: build `jl_blocks`, `start_px4` (GZ_PARTITION=verify_ready), launch `missions/takeoff_hold_land.yaml` with `mission.launch.py`, and wait for `Registered`. Then:

```bash
ros2 run jl_blocks readiness --names TakeoffHoldLand --helpers vio; echo "exit=$?"
ros2 run jl_blocks readiness --names Nope; echo "exit=$?"
ros2 run jl_blocks readiness --armed; echo "exit=$?"
```

Expected:
- **First run:** `? DDS agent session established … journalctl unavailable`, `✓ FC data arriving`, and `✗ VIO publishing … 0 Hz`, because SITL has no VIO. Then `✓ Registered in PX4 TakeoffHoldLand`, and `exit=1` because of VIO.
- **Second run:** `✗ Registered in PX4 missing: Nope`, `exit=1`.
- **Third run:** `disarmed`, `exit=0`.

Kill everything afterwards.

- [ ] **Step 6: Checkpoint**

Stop and report the diff and the Step 5 output to the maintainer. Do not commit.

---

### Task 4: `deploy.sh`

**Files:**
- Create: `deploy.sh` (repo root, executable), `test/test_deploy.py`
- Modify: `Makefile` (`check` also runs `test/test_deploy.py`), `README.md` (a "Deploying a mission" section)

**Interfaces:**
- Consumes:
  - `jl_blocks flight check|units|packages` (Task 1), run from source as `python3 -m jl_blocks.cli` with `PYTHONPATH=src/jl_blocks`, so it works before any build;
  - `services/install_services.sh` (Task 2);
  - `ros2 run jl_blocks readiness` (Task 3).
- Produces: `./deploy.sh [--check] [--dry-run] [--force-unknown-arm-state]`. Exit 0 = deployed and ready; 1 = a step failed; 2 = usage.

Every external command goes through one `run` function, so `--dry-run` prints instead of running. The tests put stub `git`, `colcon`, `systemctl`, `sudo`, `ros2` and `journalctl` executables first on `PATH`. Each stub appends its argv to a log and exits with a code taken from an environment variable. Nothing real runs.

- [ ] **Step 1: Write the failing tests**

Create `test/test_deploy.py`:

```python
"""deploy.sh against stub commands: nothing real is pulled, built or enabled."""

from __future__ import annotations

import os
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
STUBS = ("git", "colcon", "systemctl", "sudo", "ros2", "journalctl")


@pytest.fixture
def env(tmp_path):
    bin_dir = tmp_path / "bin"
    bin_dir.mkdir()
    log = tmp_path / "calls.log"
    for name in STUBS:
        stub = bin_dir / name
        stub.write_text(
            "#!/bin/bash\n"
            f'echo "{name} $*" >> {log}\n'
            # sudo runs its command, so `sudo systemctl ...` is logged too
            + ('exec "$@"\n' if name == "sudo" else "")
            + f'var="STUB_{name.upper()}_OUT"; [ -n "${{!var}}" ] && echo "${{!var}}"\n'
            + f'var="STUB_{name.upper()}_RC"; exit "${{!var:-0}}"\n'
        )
        stub.chmod(0o755)
    installer = bin_dir / "install_services_stub"
    installer.write_text(f'#!/bin/bash\necho "install_services $*" >> {log}\n')
    installer.chmod(0o755)
    e = dict(os.environ)
    # The check venv's python (with PyYAML) must be the python3 deploy.sh finds.
    e["PATH"] = f"{bin_dir}:{Path(sys.executable).parent}:{e['PATH']}"
    e["JL_DEPLOY_ROOT"] = str(ROOT)
    e["JL_INSTALL_SERVICES"] = str(installer)
    e["JL_DEPLOY_NO_SOURCE"] = "1"
    e["JL_DEPLOY_SETTLE_S"] = "0"
    e["STUB_ROS2_OUT"] = "disarmed"
    e["STUB_SYSTEMCTL_OUT"] = ""
    return e, log


def deploy(env_log, *args, **overrides):
    e, log = env_log
    e = dict(e, **overrides)
    result = subprocess.run(
        ["bash", str(ROOT / "deploy.sh"), *args],
        env=e, capture_output=True, text=True, timeout=60, check=False,
    )
    calls = log.read_text().splitlines() if log.exists() else []
    return result, calls


def test_a_dirty_tree_refuses_before_anything(env):
    result, calls = deploy(env, STUB_GIT_OUT=" M src/x.py")
    assert result.returncode == 1
    assert "uncommitted local changes" in result.stdout
    assert not any(c.startswith(("colcon", "systemctl", "git pull")) for c in calls)


def test_a_bad_mission_stops_before_build_and_touches_no_unit(env, tmp_path):
    bad = tmp_path / "flight.yaml"
    bad.write_text("missions:\n  - missions/nope.yaml\n")
    result, calls = deploy(env, JL_FLIGHT_FILE=str(bad))
    assert result.returncode == 1
    assert "FAIL" in result.stdout
    assert not any(c.startswith(("colcon", "systemctl")) for c in calls)


def test_an_armed_vehicle_refuses(env):
    result, calls = deploy(env, STUB_ROS2_OUT="armed", STUB_ROS2_RC="3")
    assert result.returncode == 1
    assert "armed" in result.stdout
    assert not any(c.startswith(("colcon", "systemctl", "git pull")) for c in calls)


def test_unknown_arm_state_refuses_unless_forced(env):
    result, _ = deploy(env, STUB_ROS2_OUT="unknown", STUB_ROS2_RC="4")
    assert result.returncode == 1
    assert "--force-unknown-arm-state" in result.stdout


def test_a_full_deploy_runs_the_steps_in_order(env):
    result, calls = deploy(env)
    assert result.returncode == 0, result.stdout + result.stderr
    order = [
        next(i for i, c in enumerate(calls) if c.startswith(prefix))
        for prefix in ("git pull", "colcon build", "systemctl enable", "ros2 run jl_blocks readiness --names")
    ]
    assert order == sorted(order)
    build = next(c for c in calls if c.startswith("colcon build"))
    assert "--packages-up-to jl_blocks jl_mission oak_d_visual_odometry" in build
    enable = next(c for c in calls if c.startswith("systemctl enable"))
    assert "jl_mission@takeoff_hold_land" in enable and "vio" in enable
    assert not any("dds_agent" in c or "translation_node" in c for c in calls if c.startswith("systemctl"))


def test_a_removed_mission_is_disabled(env):
    running = "jl_mission@old_mission.service loaded active running Mission old_mission"
    result, calls = deploy(env, STUB_SYSTEMCTL_OUT=running)
    assert result.returncode == 0, result.stdout
    assert any(c.startswith("systemctl disable --now") and "jl_mission@old_mission" in c for c in calls)
    assert not any(c.startswith("systemctl disable") and "takeoff_hold_land" in c for c in calls)


def test_check_only_runs_readiness(env):
    result, calls = deploy(env, "--check")
    assert result.returncode == 0
    assert not any(c.startswith(("git", "colcon", "systemctl")) for c in calls)
    assert any(c.startswith("ros2 run jl_blocks readiness --names TakeoffHoldLand") for c in calls)


def test_dry_run_changes_nothing(env):
    result, calls = deploy(env, "--dry-run")
    assert result.returncode == 0
    assert "would run: colcon build" in result.stdout
    assert not any(c.startswith(("git pull", "colcon", "systemctl enable")) for c in calls)


def test_bad_option_is_a_usage_error(env):
    result, _ = deploy(env, "--nope")
    assert result.returncode == 2
    assert "usage" in result.stdout.lower()


@pytest.mark.skipif(shutil.which("bash") is None, reason="needs bash")
def test_script_is_executable():
    assert os.access(ROOT / "deploy.sh", os.X_OK)
```

- [ ] **Step 2: Run the tests and watch them fail**

In the `Makefile` `check` recipe, change the pytest line to cover both folders:

```make
	PYTHONPATH=$(JL_BLOCKS) $(CHECK_BIN)/pytest -q $(JL_BLOCKS)/test test/test_deploy.py
```

Run: `make check`
Expected: FAIL. `deploy.sh` doesn't exist, so the subprocess fails.

- [ ] **Step 3: Write `deploy.sh`**

```bash
#!/bin/bash
# Deploy this checkout to the drone it runs on (spec section 7). Run on the Jetson:
#
#   ./deploy.sh            pull, check, build, enable the missions in
#                          config/flight.yaml, then report readiness
#   ./deploy.sh --check    only the readiness report (the preflight command)
#   ./deploy.sh --dry-run  print every command instead of running it
#   --force-unknown-arm-state   deploy even if the arm state can't be read
#
# Stops at the first failure. Refuses while the vehicle is armed, with
# uncommitted changes, or with a mission file that doesn't check out.
set -u
ROOT="${JL_DEPLOY_ROOT:-$(dirname "$(readlink -f "$0")")}"
FLIGHT="${JL_FLIGHT_FILE:-$ROOT/config/flight.yaml}"
cd "$ROOT" || exit 1

usage() { sed -n '2,12p' "$0" | sed 's/^# \{0,1\}//'; exit 2; }
CHECK_ONLY=0; DRY=0; FORCE_ARM=0
for arg in "$@"; do
  case "$arg" in
    --check) CHECK_ONLY=1 ;;
    --dry-run) DRY=1 ;;
    --force-unknown-arm-state) FORCE_ARM=1 ;;
    -h|--help) usage ;;
    *) echo "unknown option: $arg"; usage ;;
  esac
done

run() {
  if [ "$DRY" -eq 1 ]; then echo "would run: $*"; return 0; fi
  "$@"
}
step() { echo; echo "== $*"; }
fail() { echo "deploy: $*"; exit 1; }

# jl_blocks from source: works before anything is built.
flight() { PYTHONPATH="$ROOT/src/jl_blocks" python3 -m jl_blocks.cli flight "$@" "$FLIGHT" --root "$ROOT"; }
mission_names() {
  for f in $(flight units | grep '^jl_mission@' | sed 's/^jl_mission@//'); do
    python3 -c "import sys, yaml; print(yaml.safe_load(open(sys.argv[1]))['name'])" "$ROOT/missions/$f.yaml"
  done
}

readiness() {
  step "5/5 readiness"
  # (the tests skip this so their stub ros2 stays first on PATH)
  if [ "${JL_DEPLOY_NO_SOURCE:-0}" != 1 ] && [ -f "$ROOT/install/setup.bash" ]; then
    source "$ROOT/install/setup.bash" 2>/dev/null
  fi
  # shellcheck disable=SC2046
  run ros2 run jl_blocks readiness --names $(mission_names) \
    --helpers $(flight units | grep -v '^jl_mission@' | tr '\n' ' ')
}

if [ "$CHECK_ONLY" -eq 1 ]; then
  readiness; exit $?
fi

step "0/5 arm state"
arm=$(ros2 run jl_blocks readiness --armed --seconds 3 2>/dev/null); arm_rc=$?
case "$arm_rc" in
  0) echo "disarmed" ;;
  3) fail "the vehicle is armed; land and disarm before deploying" ;;
  *) [ "$FORCE_ARM" -eq 1 ] && echo "arm state unknown ($arm); continuing (--force-unknown-arm-state)" \
       || fail "can't read the arm state ($arm); is the FC link up? Re-run with --force-unknown-arm-state to deploy anyway" ;;
esac

step "1/5 git pull"
[ -z "$(git status --porcelain)" ] || fail "uncommitted local changes; commit or discard them so the drone matches a commit"
run git pull --ff-only || fail "git pull failed"

step "2/5 check $FLIGHT"
flight check || fail "fix the files above; nothing running was touched"

step "3/5 build"
# shellcheck disable=SC2046
run colcon build --packages-up-to $(flight packages | tr '\n' ' ') || fail "build failed; nothing running was touched"

step "4/5 services"
wanted=$(flight units | tr '\n' ' ')
run "${JL_INSTALL_SERVICES:-./services/install_services.sh}" jl_mission@ || fail "installing jl_mission@.service failed"
# Disable what flight.yaml no longer lists: old jl_mission@ instances and
# known helpers. dds_agent and translation_node are never touched.
for unit in $(systemctl list-units --all --plain --no-legend 'jl_mission@*' | awk '{print $1}' | sed 's/\.service$//') \
            vio aruco_tracker battery_monitor; do
  case " $wanted " in *" $unit "*) ;; *) run sudo systemctl disable --now "$unit" 2>/dev/null ;; esac
done
# shellcheck disable=SC2086
run sudo systemctl enable $wanted || fail "enabling units failed"
# shellcheck disable=SC2086
run sudo systemctl restart $wanted || fail "starting units failed"
sleep "${JL_DEPLOY_SETTLE_S:-15}"

readiness
```

`chmod +x deploy.sh`.

Test-harness notes. The fixture sets:
- `JL_DEPLOY_SETTLE_S=0`, so there's no wait;
- `JL_INSTALL_SERVICES` to a logging stub, so the real installer, which writes under `/etc`, never runs;
- `JL_DEPLOY_NO_SOURCE=1`, so the stub `ros2` stays first on `PATH`;
- `PATH` to include the check venv's python, which has PyYAML.
- If `git status --porcelain` needs its own stub output, the `STUB_GIT_OUT` variable covers it.
- The armed tests expect a failure before `git pull`.

- [ ] **Step 4: Run the tests and watch them pass**

Run: `make check`
Expected: every stage passes, including the 10 deploy tests.

If a stub interaction differs from what the test expects, fix `deploy.sh` or the stub fixture, and keep each test's intent. Examples: `systemctl list-units` output parsing, or `sudo` passing the command through.

- [ ] **Step 5: README section**

In `README.md`, add a "Deploying a mission" section after the START HERE package table:

````markdown
## Deploying a mission

On the drone (over its hotspot):

1. Add the mission file to `config/flight.yaml` (it must live in `missions/`), commit, push.
2. On the Jetson: `./deploy.sh`. It refuses while armed or with local changes,
   checks every mission, builds, enables one `jl_mission@<file>` service per
   mission, and ends with the readiness report.
3. Before each flight: `./deploy.sh --check`.

```
✓ DDS agent session established    dds_agent journal
✓ FC data arriving                 /fmu/out/vehicle_status
✓ VIO publishing                   /fmu/in/vehicle_visual_odometry  30 Hz
✓ Registered in PX4                TakeoffHoldLand
```

Try it anywhere first with `./deploy.sh --dry-run`, and in SITL with `make sitl-mission`.
````

- [ ] **Step 6: Checkpoint**

Stop and report the diff to the maintainer. Do not commit.

---

### Task 5: `setpoint_timing`, a SITL baseline, and the spec

**Files:**
- Create: `src/jl_blocks/jl_blocks/timing.py`, `src/jl_blocks/jl_blocks/ros/setpoint_timing.py`, `src/jl_blocks/test/test_timing.py`
- Modify: `src/jl_blocks/setup.py` (entry point `setpoint_timing`), `docs/superpowers/specs/2026-09-21-mission-blocks-design.md`

**Interfaces:**
- Produces:
  - `timing.summary(stamps: list[float]) -> dict[str, float]`, with keys `count`, `p50_ms`, `p99_ms`, `max_ms`;
  - `ros2 run jl_blocks setpoint_timing --name NAME --seconds 60`, which prints `setpoint inter-arrival over N messages: p50 X ms, p99 Y ms, max Z ms` and exits 1 if fewer than 10 messages arrived.

- [ ] **Step 1: Failing tests**

Create `src/jl_blocks/test/test_timing.py`:

```python
from __future__ import annotations

import pytest

from jl_blocks.timing import summary


def test_regular_20_ms_stream():
    stamps = [i * 0.02 for i in range(101)]
    s = summary(stamps)
    assert s["count"] == 101
    assert s["p50_ms"] == pytest.approx(20.0)
    assert s["max_ms"] == pytest.approx(20.0)


def test_one_late_message_shows_in_max_and_p99():
    stamps = [i * 0.02 for i in range(100)] + [1.98 + 0.05]
    s = summary(stamps)
    assert s["max_ms"] == pytest.approx(50.0)
    assert s["p99_ms"] >= 20.0


def test_too_few_messages():
    assert summary([0.0])["count"] == 1
    assert summary([0.0])["max_ms"] == 0.0
```

- [ ] **Step 2: Watch them fail, then implement**

`make check` fails with `No module named 'jl_blocks.timing'`. Create `src/jl_blocks/jl_blocks/timing.py`:

```python
"""Setpoint inter-arrival statistics, as in spec section 6's spike table."""

from __future__ import annotations


def _percentile(sorted_values: list[float], q: float) -> float:
    if not sorted_values:
        return 0.0
    index = min(len(sorted_values) - 1, round(q * (len(sorted_values) - 1)))
    return sorted_values[index]


def summary(stamps: list[float]) -> dict[str, float]:
    gaps = sorted((b - a) * 1000.0 for a, b in zip(stamps, stamps[1:]))
    return {
        "count": float(len(stamps)),
        "p50_ms": round(_percentile(gaps, 0.50), 1),
        "p99_ms": round(_percentile(gaps, 0.99), 1),
        "max_ms": round(gaps[-1], 1) if gaps else 0.0,
    }
```

Create `src/jl_blocks/jl_blocks/ros/setpoint_timing.py`:

```python
"""setpoint_timing: repeat spec section 6's timing measurement on this machine.

Listens to /jl/NAME/setpoint (the runner's output) for --seconds and prints the
inter-arrival p50 / p99 / max. Run it on the Jetson during a flight with VIO
and YOLO up, and compare with the spec's desktop numbers (p99 22.8 ms, max 27.6 ms).
"""

from __future__ import annotations

import argparse
import sys
import time

import rclpy
from px4_msgs.msg import TrajectorySetpoint
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from ..timing import summary


def main() -> None:
    parser = argparse.ArgumentParser(prog="setpoint_timing")
    parser.add_argument("--name", required=True, help="the mission name (QGC name)")
    parser.add_argument("--seconds", type=float, default=60.0)
    args = parser.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    rclpy.init()
    node = Node("setpoint_timing")
    stamps: list[float] = []
    node.create_subscription(
        TrajectorySetpoint,
        f"/jl/{args.name}/setpoint",
        lambda _msg: stamps.append(time.monotonic()),
        qos_profile_sensor_data,
    )
    end = time.monotonic() + args.seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.try_shutdown()
    s = summary(stamps)
    print(
        f"setpoint inter-arrival over {int(s['count'])} messages: "
        f"p50 {s['p50_ms']:g} ms, p99 {s['p99_ms']:g} ms, max {s['max_ms']:g} ms"
    )
    sys.exit(0 if s["count"] >= 10 else 1)


if __name__ == "__main__":
    main()
```

Add `"setpoint_timing = jl_blocks.ros.setpoint_timing:main",` to `setup.py`. Run `make check`: 3 new tests pass.

- [ ] **Step 3: SITL baseline**

In the container, restarted, fly a 60 s hold:
- write `/tmp/hold60.yaml` with `name: Hold60`, then `takeoff: {height: 1.5}`, then `hold: {duration: 60}`;
- run `test/sitl_mission.sh /tmp/hold60.yaml --expect "takeoff hold" --timeout 150 &`;
- once `event: hold: started` shows in the newest `/tmp/tmp.*/watch.log`, run `ros2 run jl_blocks setpoint_timing --name Hold60 --seconds 30`.

Report the printed line. For reference, spec §6's spike measured p50 20.0 ms, p99 22.8 ms and max 27.6 ms on this desktop.

- [ ] **Step 4: Spec updates**

In `docs/superpowers/specs/2026-09-21-mission-blocks-design.md`:

§6, after the table, add:

```markdown
Phase 5 added `ros2 run jl_blocks setpoint_timing --name NAME` to repeat this
measurement anywhere. SITL baseline with the full runner (Phase 5): <the Step 3 line>.
**Still to do on the Jetson** under flight load (VIO + YOLO): run it during a
hold and record the result here before the first real mission flight.
```

(Paste the actual Step 3 line in place of the `<…>` text.)

§11, add:

```markdown
- Phase 5 deploy decisions: "disable those not listed" applies to `jl_mission@*` and the known helpers only (dds_agent/translation_node/takeoff_hold are never touched); deploy refuses while armed; the runner is respawned by launch inside `jl_mission@` (jl_mission stays up so its watchdog holds, then lands); mission names used by other boot services are rejected.
- `deploy.sh` has only run against stubs and `--dry-run`. Its first real run should be on the drone with props off, watching `./deploy.sh --check` afterwards.
```

- [ ] **Step 5: Final check**

Run: `make check`, then `make sitl-test` once from the host.
Expected: both green. If `make sitl-test` hits the known one-in-eight ArUco takeoff flake (see the Phase 4 ledger), re-run it once and report both results.

- [ ] **Step 6: Checkpoint**

Stop and report the diff, the Step 3 line, and the check results to the maintainer. Do not commit.

---

## Maintainer notes (found while planning; not tasks)

- **Needs the drone:**
  - `deploy.sh`'s first real run, props off;
  - `./deploy.sh --check` on the Jetson (journal checks, VIO rate);
  - the Jetson timing run (spec §6).

  Nothing in this plan runs `sudo`/`systemctl` on the development machine.
- **The shipped `config/flight.yaml`** lists `takeoff_hold_land.yaml` with `vio` and `battery_monitor`, so a first deploy keeps VIO and the battery alarm running. `TakeoffHold` (the existing precision_land service) is left alone.
- **After Phase 5:** Phase 6 (the beginner tutorial) can point at `make sitl-mission` and this README section.
