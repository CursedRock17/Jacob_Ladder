# Mission Blocks Phase 4: `test/sitl_mission.sh` and the ArUco acceptance mission — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build one command that flies any mission file headless in SITL and checks what it did. Then use it for the spec's L3 acceptance test: the DroneSmoothPlanner ArUco demo, rewritten as a mission file, must hold a 3 m standoff within ±0.25 m per axis.

**Architecture:**
- **`test/sitl_mission.sh MISSION.yaml --expect "…" [options]`** starts a headless world and PX4, flies the file through `ros2 launch jl_blocks mission.launch.py`, and lets a watcher decide pass or fail. Two worlds are available:
  - PX4's plain `gz_x500`;
  - the repo's `aruco_dual_ids` world with `x500_dual_cam` and the front `aruco_tracker`.
- **The watcher** is `mission_watch`, a small ROS node in `jl_blocks`. It listens to the runner's `/jl/NAME/events` and, optionally, the front camera's `/front/target_pose`. The checks themselves are `jl_blocks.testing.expect`: pure Python, run by `pytest` in `make check`.
- **`make sitl-test`** runs every L3 flight through one script, `test/sitl_all.sh`:
  - the three jl_mission contract flights;
  - takeoff→hold→land;
  - the runner-dies flight;
  - the ArUco acceptance.

**Tech Stack:** Python 3.10, rclpy (ROS 2 Humble), bash, Gazebo Harmonic (`gz sim -s --headless-rendering`), PX4 v1.16.0 SITL (standalone mode for the ArUco world), `aruco_tracker`, pytest 9.1.1, ruff 0.15.20, ty 0.0.55.

**Spec:** `docs/superpowers/specs/2026-09-21-mission-blocks-design.md`. This plan implements phase 4 of §9: the L3 row and the "SITL harness requirements" of §8, and the first L3 acceptance test (the ArUco demo from `src/drogue_flight/docs/aruco_sitl.md` as a mission file).

## Global Constraints

- **No flight-code changes.** Do NOT modify:
  - `src/precision_land/`, `src/drogue_flight/`;
  - `src/aruco_tracker/` (its launch files are used as they are);
  - `src/ros2_yolo_image_processing/`;
  - the `jl_mission` C++;
  - `gazebo/` (world and models are used as they are).
- **Do not commit.** The maintainer makes all commits. Each task ends with a checkpoint: stop and report the diff. Never run `git add/commit/stash/checkout/reset/restore`.
- Work only on `main`; never add yourself to commit history.
- `jl_blocks.testing` is ROS-free (standard library only), like `jl_blocks.core`. Only `jl_blocks/ros/` imports rclpy.
- `make check` must pass after every task. Run it from the repo root on the host.
- SITL harness requirements, verbatim from spec §8. The L3 harness must:
  - "start PX4 with `-d` (no interactive shell…) and `-w <fresh dir>`";
  - "pin `GZ_PARTITION`";
  - "set `NAV_DLL_ACT 0` in that fresh parameter set";
  - "retry mode selection slowly (≥ 10 s apart)";
  - "run headless".
- Acceptance numbers, verbatim from spec §8: the ArUco mission "must reproduce the 3 m standoff within ±0.25 m per axis". This plan measures it in the front camera's optical frame (+x right, +y down, +z forward): the tag at (0, 0, 3) ± 0.25 m per axis, held for 5 s. See Ruling 1.
- Rendering headless was verified in the container while planning. `gz sim -s -r --headless-rendering gazebo/worlds/aruco_dual_ids.sdf`, with `x500_dual_cam` spawned, publishes 1280×960 RGB frames on the front camera topic without a display.
- Run SITL pieces inside the sim container. From the host:
  `docker exec --user user -w /hdd/Documents/Electrical/Matrix_Lab/jacob_drone_ws/src/Jacob_Ladder jacob_ladder_sim bash -c 'source /opt/ros/humble/setup.bash && source install/setup.bash && …'`
  Restart it first (`docker restart jacob_ladder_sim && sleep 3`) so no earlier flight is still running.
- In this container, some processes ignore SIGTERM: px4, MicroXRCEAgent, translation_node_bin, and jl_mission while it waits for the FMU. The container restart clears them, and zombies are harmless.

## Rulings (confirm at review)

1. **The standoff is measured by the front camera, not against Gazebo ground truth.** The tag's local-frame height depends on where PX4's EKF puts z = 0 and on the camera's mount offset. The demo in `aruco_sitl.md` shows both of these moving the numbers by ~0.2 m, which is most of the tolerance. The camera frame is what "3 m in front of the tag" means to the drone, and it is exactly what the DroneSmoothPlanner GIF shows ("Z = forward distance, settling at 3.00"). What passes: the tag straight ahead, level, 3 m away.
2. **The acceptance mission uses the default `position` controller**, like DroneSmoothPlanner, whose carrot + feed-forward it ports. `missions/track_moving_aruco.yaml` (with `pid`) is flown once by hand in Task 4 and reported, but does not gate `make sitl-test`. The repo has no moving front-camera target yet: the moving platform in `gazebo/models/` is a downward-camera world.
3. **Steps are checked on `/jl/NAME/events`, not `/jl/NAME/state`.** State is latched and only shows the latest step, so a fast step can be missed; events arrive in order. Spec §8 names `/jl/NAME/state` loosely.

## Review Focus

1. **A mission that aborts early** (e.g. the search times out and it lands) must FAIL at once, naming the steps that never started. It must not wait out the full timeout, and it must never PASS. Owner: Task 1, `test_an_abort_is_a_failure_naming_the_missing_steps`.
2. **A tracker that sees the tag only in bursts.** Frames drop headless under load. A gap longer than 0.5 s must restart the 5 s hold, so two good samples 5 s apart never pass. Owner: Task 1, `test_a_gap_in_detections_restarts_the_hold`.
3. **A mission that loops through `on_fail`** (track → search → track) must still pass the expected order on the first occurrences, and repeats must not confuse it. Owner: Task 1, `test_repeated_steps_still_match_the_first_time_through`.
4. **The headless world or PX4 never comes up** (no `/dev/dri`, a bad model path). The harness must stop with a `FAIL … did not start (see <log>)` line and exit 1, not hang. Owner: Task 4, Step 3.
5. **A bad mission file, or a wrong option,** must fail in seconds, before PX4 starts. Bad options exit 2 with usage; a bad file exits 1 with `jl_blocks check`'s message. Owner: Task 3, Step 4.

---

## File Structure

```
src/jl_blocks/
├── jl_blocks/testing/
│   ├── __init__.py
│   └── expect.py            NEW: StepOrder, BandHold, report()  (no ROS)
├── jl_blocks/ros/
│   └── mission_watch.py     NEW: the watcher node
├── setup.py                 + mission_watch entry point
└── test/test_expect.py      NEW
missions/aruco_standoff.yaml NEW: the acceptance mission
test/
├── sitl_common.sh           + start_px4_aruco, kill_flight additions
├── sitl_mission.sh          NEW: fly + check any mission file
├── sitl_runner_smoke.sh     keeps only the runner-dies flight
└── sitl_all.sh              NEW: every L3 flight, in order
Makefile                     sitl-test runs test/sitl_all.sh; new sitl-mission
docs/superpowers/specs/...design.md  §8 L3 row, §11 note
```

---

### Task 1: `jl_blocks.testing.expect`: the checks, without ROS

**Files:**
- Create: `src/jl_blocks/jl_blocks/testing/__init__.py`, `src/jl_blocks/jl_blocks/testing/expect.py`
- Test: `src/jl_blocks/test/test_expect.py`

**Interfaces:**
- Consumes: the runner's event strings. They look like:
  - `"<step>: started"`, `"<step>: done"`, `"<step>: requested takeoff"`;
  - `"<step>: failed (<why>) -> <other>"`;
  - `"<step>: failed (<why>); no on_fail -> hold, then land"`;
  - `"mission finished"`;
  - `"aborted: held, now landing"`;
  - `"activated: starting from step 1"`, `"deactivated"`.
- Produces:
  - `StepOrder(expected: list[str])`, with:
    - `.feed(event: str) -> None`;
    - `.ok -> bool` (every expected step started, in order, as a subsequence of first starts);
    - `.missing() -> list[str]`;
    - `.finished -> bool`;
    - `.aborted -> bool`;
    - `.expected -> tuple[str, ...]`.
  - `BandHold(center: Vector3, tolerance: float, hold_s: float, max_gap_s: float = 0.5)`, with:
    - `.feed(position: Vector3, t: float) -> None`;
    - `.held -> bool` (latched);
    - `.last -> Vector3 | None`;
    - `.center`, `.tolerance`, `.hold_s`.
  - `report(order: StepOrder, need_finished: bool, band: BandHold | None) -> tuple[list[str], bool]`: one `PASS …`/`FAIL …` line per check, and whether all passed.

- [ ] **Step 1: Write the failing tests**

Create `src/jl_blocks/test/test_expect.py`:

```python
from __future__ import annotations

from jl_blocks.testing.expect import BandHold, StepOrder, report


def fed(expected, events):
    order = StepOrder(expected)
    for event in events:
        order.feed(event)
    return order


def test_steps_started_in_order_pass():
    order = fed(
        ["takeoff", "search", "track"],
        ["activated: starting from step 1", "takeoff: started",
         "takeoff: requested takeoff", "takeoff: done", "search: started",
         "search: done", "track: started"],
    )
    assert order.ok
    assert order.missing() == []


def test_out_of_order_steps_do_not_pass():
    order = fed(
        ["takeoff", "search", "track"],
        ["takeoff: started", "track: started", "search: started"],
    )
    assert not order.ok
    assert order.missing() == ["track"]


def test_only_started_events_count():
    order = fed(["takeoff"], ["takeoff: requested takeoff", "takeoff: done"])
    assert not order.ok


def test_repeated_steps_still_match_the_first_time_through():
    order = fed(
        ["takeoff", "search", "track"],
        ["takeoff: started", "search: started", "track: started",
         "track: failed (block reported failure) -> search",
         "search: started", "track: started"],
    )
    assert order.ok


def test_finished_is_recorded():
    order = fed(["hold"], ["hold: started", "hold: done", "mission finished"])
    assert order.finished


def test_an_abort_is_a_failure_naming_the_missing_steps():
    order = fed(
        ["takeoff", "search", "track"],
        ["takeoff: started", "search: started",
         "search: failed (timeout after 30 s); no on_fail -> hold, then land",
         "aborted: held, now landing"],
    )
    assert order.aborted
    lines, ok = report(order, need_finished=False, band=None)
    assert not ok
    assert "FAIL steps started in order: takeoff search track (missing: track)" in lines
    assert "FAIL mission did not abort" in lines


def band(**kwargs):
    return BandHold((0.0, 0.0, 3.0), 0.25, 5.0, **kwargs)


def feed_steady(hold, position, start, seconds, step=0.1):
    t = start
    while t <= start + seconds + 1e-9:
        hold.feed(position, t)
        t += step
    return t


def test_holding_inside_the_band_for_hold_s_passes():
    hold = band()
    feed_steady(hold, (0.1, -0.1, 3.2), 0.0, 4.9)
    assert not hold.held
    feed_steady(hold, (0.1, -0.1, 3.2), 5.0, 0.0)
    assert hold.held


def test_one_axis_outside_the_band_is_outside():
    hold = band()
    feed_steady(hold, (0.3, 0.0, 3.0), 0.0, 6.0)
    assert not hold.held
    assert hold.last == (0.3, 0.0, 3.0)


def test_leaving_the_band_restarts_the_hold():
    hold = band()
    feed_steady(hold, (0.0, 0.0, 3.0), 0.0, 3.0)
    hold.feed((0.0, 0.0, 3.5), 3.1)
    feed_steady(hold, (0.0, 0.0, 3.0), 3.2, 4.9)
    assert not hold.held
    feed_steady(hold, (0.0, 0.0, 3.0), 8.2, 0.0)
    assert hold.held


def test_a_gap_in_detections_restarts_the_hold():
    hold = band()
    hold.feed((0.0, 0.0, 3.0), 0.0)
    hold.feed((0.0, 0.0, 3.0), 5.0)  # nothing in between
    assert not hold.held
    feed_steady(hold, (0.0, 0.0, 3.0), 5.1, 5.0)
    assert hold.held


def test_held_stays_true_once_reached():
    hold = band()
    feed_steady(hold, (0.0, 0.0, 3.0), 0.0, 5.0)
    hold.feed((5.0, 5.0, 5.0), 5.2)
    assert hold.held


def test_report_all_passing():
    order = fed(["hold"], ["hold: started", "mission finished"])
    hold = band()
    feed_steady(hold, (0.0, 0.0, 3.0), 0.0, 5.0)
    lines, ok = report(order, need_finished=True, band=hold)
    assert ok
    assert lines == [
        "PASS steps started in order: hold",
        "PASS mission did not abort",
        "PASS mission finished",
        "PASS camera saw the target within 0.25 m of (0.00, 0.00, 3.00) for 5 s",
    ]


def test_report_never_seen_and_not_finished():
    order = fed(["hold"], ["hold: started"])
    lines, ok = report(order, need_finished=True, band=band())
    assert not ok
    assert "FAIL mission finished" in lines
    assert (
        "FAIL camera saw the target within 0.25 m of (0.00, 0.00, 3.00) for 5 s "
        "(never saw it)"
    ) in lines


def test_report_last_seen_position():
    hold = band()
    hold.feed((0.5, 0.0, 2.0), 1.0)
    lines, _ = report(fed(["hold"], ["hold: started"]), False, hold)
    assert lines[-1].endswith("(last seen at (0.50, 0.00, 2.00))")
```

- [ ] **Step 2: Run the tests and watch them fail**

Run: `make check`
Expected: FAIL: `No module named 'jl_blocks.testing'`.

- [ ] **Step 3: Implement**

Create `src/jl_blocks/jl_blocks/testing/__init__.py`:

```python
"""Checks for missions flown in simulation. No ROS here, like jl_blocks.core."""
```

Create `src/jl_blocks/jl_blocks/testing/expect.py`:

```python
"""What test/sitl_mission.sh checks, as plain Python so it is tested without ROS.

StepOrder follows the runner's /jl/NAME/events; BandHold follows where the
camera sees the target. report() turns both into PASS/FAIL lines.
"""

from __future__ import annotations

from ..core import Vector3

STARTED = ": started"
ABORT_MARKERS = ("no on_fail -> hold, then land", "aborted: held, now landing")


class StepOrder:
    """Did the expected steps start, in this order? Later repeats are ignored."""

    def __init__(self, expected: list[str]) -> None:
        self.expected = tuple(expected)
        self._next = 0
        self.finished = False
        self.aborted = False

    def feed(self, event: str) -> None:
        if event == "mission finished":
            self.finished = True
        if any(marker in event for marker in ABORT_MARKERS):
            self.aborted = True
        if not event.endswith(STARTED):
            return
        step = event[: -len(STARTED)]
        if self._next < len(self.expected) and step == self.expected[self._next]:
            self._next += 1

    @property
    def ok(self) -> bool:
        return self._next == len(self.expected)

    def missing(self) -> list[str]:
        return list(self.expected[self._next :])


class BandHold:
    """Was a position inside center +/- tolerance (per axis) for hold_s seconds?

    A gap of more than max_gap_s between samples restarts the clock, so a
    target seen only now and then never counts as held. Once held, it stays held.
    """

    def __init__(
        self, center: Vector3, tolerance: float, hold_s: float, max_gap_s: float = 0.5
    ) -> None:
        self.center = center
        self.tolerance = tolerance
        self.hold_s = hold_s
        self.max_gap_s = max_gap_s
        self.held = False
        self.last: Vector3 | None = None
        self._since: float | None = None
        self._last_t: float | None = None

    def feed(self, position: Vector3, t: float) -> None:
        inside = all(
            abs(p - c) <= self.tolerance for p, c in zip(position, self.center)
        )
        gap = self._last_t is not None and t - self._last_t > self.max_gap_s
        if not inside:
            self._since = None
        elif self._since is None or gap:
            self._since = t
        self._last_t = t
        self.last = position
        if self._since is not None and t - self._since >= self.hold_s - 1e-9:
            self.held = True


def _fmt(v: Vector3) -> str:
    return ", ".join(f"{x:.2f}" for x in v)


def report(
    order: StepOrder, need_finished: bool, band: BandHold | None
) -> tuple[list[str], bool]:
    lines: list[str] = []
    ok = True
    wanted = " ".join(order.expected)
    if order.ok:
        lines.append(f"PASS steps started in order: {wanted}")
    else:
        lines.append(
            f"FAIL steps started in order: {wanted} "
            f"(missing: {' '.join(order.missing())})"
        )
        ok = False
    if order.aborted:
        lines.append("FAIL mission did not abort")
        ok = False
    else:
        lines.append("PASS mission did not abort")
    if need_finished:
        lines.append(f"{'PASS' if order.finished else 'FAIL'} mission finished")
        ok = ok and order.finished
    if band is not None:
        what = (
            f"camera saw the target within {band.tolerance:g} m of "
            f"({_fmt(band.center)}) for {band.hold_s:g} s"
        )
        if band.held:
            lines.append(f"PASS {what}")
        elif band.last is None:
            lines.append(f"FAIL {what} (never saw it)")
            ok = False
        else:
            lines.append(f"FAIL {what} (last seen at ({_fmt(band.last)}))")
            ok = False
    return lines, ok
```

- [ ] **Step 4: Run the tests and watch them pass**

Run: `make check`
Expected: every stage passes, with 14 new tests.

- [ ] **Step 5: Checkpoint**

Stop and report the diff to the maintainer. Do not commit.

---

### Task 2: The `mission_watch` node

**Files:**
- Create: `src/jl_blocks/jl_blocks/ros/mission_watch.py`
- Modify: `src/jl_blocks/setup.py` (`console_scripts`)

**Interfaces:**
- Consumes: `StepOrder`, `BandHold`, `report` (Task 1). The runner publishes `/jl/NAME/events` as `std_msgs/String`, reliable, depth 10. `aruco_tracker` publishes `geometry_msgs/PoseStamped` on `/front/target_pose`.
- Produces: `ros2 run jl_blocks mission_watch --ros-args -p …`. Its parameters:
  - `mission_name` (string);
  - `expect` (string, space-separated step names);
  - `finished` (bool, false);
  - `camera_topic` (string, `/front/target_pose`);
  - `camera_target` (string `"X Y Z"`; empty = no camera check);
  - `tolerance`, `hold_s`, `timeout_s`: numbers, 0.25 / 5.0 / 120.0, and ints are accepted too.

  It prints each `event: …` as it arrives, then the PASS/FAIL lines. It exits 0 when everything has passed. It exits 1 on an abort, or at the timeout if anything still fails.

- [ ] **Step 1: Write the node**

Create `src/jl_blocks/jl_blocks/ros/mission_watch.py`:

```python
"""mission_watch: check a mission flown in SITL (used by test/sitl_mission.sh).

Listens to the runner's /jl/NAME/events and, if asked, to where a camera sees
the target. Exits 0 as soon as every check passes, 1 on an abort or a timeout.
The checks themselves are jl_blocks.testing.expect, tested without ROS.
"""

from __future__ import annotations

import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String

from ..testing.expect import BandHold, StepOrder, report

# Numbers may be written as 5 or 5.0 on the command line.
ANY_NUMBER = ParameterDescriptor(dynamic_typing=True)


class MissionWatch(Node):
    def __init__(self) -> None:
        super().__init__("mission_watch")
        name = str(self.declare_parameter("mission_name", "").value)
        expect = str(self.declare_parameter("expect", "").value).split()
        self.need_finished = bool(self.declare_parameter("finished", False).value)
        camera_topic = str(
            self.declare_parameter("camera_topic", "/front/target_pose").value
        )
        target = str(self.declare_parameter("camera_target", "").value).split()
        tolerance = float(self.declare_parameter("tolerance", 0.25, ANY_NUMBER).value)
        hold_s = float(self.declare_parameter("hold_s", 5.0, ANY_NUMBER).value)
        self.timeout_s = float(
            self.declare_parameter("timeout_s", 120.0, ANY_NUMBER).value
        )

        self.order = StepOrder(expect)
        self.band: BandHold | None = None
        if target:
            x, y, z = (float(v) for v in target)
            self.band = BandHold((x, y, z), tolerance, hold_s)
            self.create_subscription(
                PoseStamped, camera_topic, self.on_pose, qos_profile_sensor_data
            )
        self.create_subscription(String, f"/jl/{name}/events", self.on_event, 10)
        self.start = time.monotonic()
        self.result: int | None = None
        self.create_timer(0.1, self.check)

    def on_event(self, msg: String) -> None:
        print(f"event: {msg.data}", flush=True)
        self.order.feed(msg.data)

    def on_pose(self, msg: PoseStamped) -> None:
        assert self.band is not None
        p = msg.pose.position
        self.band.feed((float(p.x), float(p.y), float(p.z)), time.monotonic())

    def check(self) -> None:
        if self.result is not None:
            return
        passed = (
            self.order.ok
            and (self.order.finished or not self.need_finished)
            and (self.band is None or self.band.held)
        )
        timed_out = time.monotonic() - self.start > self.timeout_s
        if not (passed or timed_out or self.order.aborted):
            return
        lines, ok = report(self.order, self.need_finished, self.band)
        if timed_out and not ok:
            lines.append(f"FAIL all checks passed within {self.timeout_s:g} s")
        for line in lines:
            print(line, flush=True)
        self.result = 0 if ok else 1


def main() -> None:
    rclpy.init()
    node = MissionWatch()
    try:
        while rclpy.ok() and node.result is None:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.result = 1
    finally:
        result = 1 if node.result is None else node.result
        node.destroy_node()
        rclpy.try_shutdown()
    sys.exit(result)


if __name__ == "__main__":
    main()
```

(`rclpy.try_shutdown` exists in this Humble install: the Phase 3 runner uses it.)

In `src/jl_blocks/setup.py`, add to `console_scripts`:

```python
            "mission_watch = jl_blocks.ros.mission_watch:main",
```

- [ ] **Step 2: Lint on the host**

Run: `make check`
Expected: every stage passes. `ty` skips `jl_blocks/ros/` as before, and ruff checks it.

- [ ] **Step 3: Exercise it in the container without SITL**

```bash
docker restart jacob_ladder_sim && sleep 3
docker exec --user user -w /hdd/Documents/Electrical/Matrix_Lab/jacob_drone_ws/src/Jacob_Ladder jacob_ladder_sim bash -c '
source /opt/ros/humble/setup.bash && colcon build --packages-select jl_blocks && source install/setup.bash
echo "--- timeout case"
ros2 run jl_blocks mission_watch --ros-args -p mission_name:=W -p expect:="takeoff hold" -p timeout_s:=3; echo "exit=$?"
echo "--- pass case"
ros2 run jl_blocks mission_watch --ros-args -p mission_name:=W -p expect:="takeoff hold" -p finished:=true -p timeout_s:=20 &
sleep 3
for e in "takeoff: started" "hold: started" "mission finished"; do
  ros2 topic pub --once /jl/W/events std_msgs/msg/String "{data: \"$e\"}" > /dev/null
done
wait $!; echo "exit=$?"
echo "--- abort case"
ros2 run jl_blocks mission_watch --ros-args -p mission_name:=W -p expect:="takeoff hold" -p timeout_s:=20 &
sleep 3
ros2 topic pub --once /jl/W/events std_msgs/msg/String "{data: \"takeoff: failed (x); no on_fail -> hold, then land\"}" > /dev/null
wait $!; echo "exit=$?"'
```

Expected:
- **Timeout case:** `FAIL steps started in order: takeoff hold (missing: takeoff hold)`, `PASS mission did not abort`, `FAIL all checks passed within 3 s`, `exit=1`.
- **Pass case:** the three `event:` lines, three PASS lines, `exit=0`.
- **Abort case:** `FAIL mission did not abort`, `exit=1`, well before 20 s.

- [ ] **Step 4: Checkpoint**

Stop and report the diff and the Step 3 output to the maintainer. Do not commit.

---

### Task 3: `test/sitl_mission.sh` on the default world, and `test/sitl_all.sh`

**Files:**
- Create: `test/sitl_mission.sh` (executable), `test/sitl_all.sh` (executable)
- Modify:
  - `test/sitl_common.sh` (`kill_flight`);
  - `test/sitl_runner_smoke.sh` (drop the first flight, which `sitl_mission.sh` now covers);
  - `Makefile` (`sitl-test`, new `sitl-mission`).

**Interfaces:**
- Consumes:
  - `mission_watch` (Task 2);
  - `start_px4 LOGDIR`, `kill_flight`, `$PX4_BUILD` from `test/sitl_common.sh`;
  - `ros2 launch jl_blocks mission.launch.py mission_file:=…`;
  - `ros2 run jl_blocks jl_blocks check FILE`.
- Produces: `test/sitl_mission.sh MISSION.yaml --expect "…" [--finished] [--world default|aruco] [--camera-standoff "X Y Z"] [--tolerance M] [--hold S] [--timeout S]`. It exits 0 when every check passed, 1 on a failed check, and 2 on a usage error. `--world aruco` calls `start_px4_aruco`, which Task 4 adds.

- [ ] **Step 1: `kill_flight` knows the new processes**

In `test/sitl_common.sh`, add these lines to `kill_flight`, after the `mission_runner` line:

```bash
  pkill -f "lib/jl_blocks/mission_watch" 2>/dev/null
  pkill -f "aruco_tracker" 2>/dev/null
  pkill -f "parameter_bridge" 2>/dev/null
```

- [ ] **Step 2: Write `test/sitl_mission.sh`**

```bash
#!/bin/bash
# Fly one mission file headless in SITL and check what it did (spec section 8, L3).
#
#   test/sitl_mission.sh MISSION.yaml --expect "takeoff hold land" [--finished]
#       [--world default|aruco] [--camera-standoff "X Y Z"] [--tolerance M]
#       [--hold S] [--timeout S]
#
#   --expect           step names that must start, in this order (required)
#   --finished         the mission must also finish ("mission finished")
#   --world            default: PX4's gz_x500 in an empty world.
#                      aruco: gazebo/worlds/aruco_dual_ids.sdf, x500_dual_cam,
#                      and the front aruco_tracker (tag id 0)
#   --camera-standoff  the front camera must see the tag at X Y Z metres
#                      (optical frame: +x right, +y down, +z forward), within
#                      --tolerance (0.25) per axis, for --hold (5) seconds
#   --timeout          seconds from selecting the mission to giving up (120)
#
# Run inside the sim container after colcon build. Exit 0 = every check passed.
HERE="$(dirname "$(readlink -f "$0")")"
source "$HERE/sitl_common.sh"

usage() { sed -n '2,18p' "$0" | sed 's/^# \{0,1\}//'; exit 2; }
[ $# -ge 1 ] || usage
MISSION="$(readlink -f "$1")"; shift
EXPECT=""; FINISHED=false; WORLD=default; CAMERA=""; TOL=0.25; HOLD=5; TIMEOUT=120
while [ $# -gt 0 ]; do
  case "$1" in
    --expect) EXPECT="$2"; shift 2 ;;
    --finished) FINISHED=true; shift ;;
    --world) WORLD="$2"; shift 2 ;;
    --camera-standoff) CAMERA="$2"; shift 2 ;;
    --tolerance) TOL="$2"; shift 2 ;;
    --hold) HOLD="$2"; shift 2 ;;
    --timeout) TIMEOUT="$2"; shift 2 ;;
    *) echo "unknown option: $1"; usage ;;
  esac
done
[ -n "$EXPECT" ] || { echo "--expect is required"; usage; }
case "$WORLD" in default|aruco) ;; *) echo "unknown world: $WORLD"; usage ;; esac

# A bad mission file fails here, in seconds, before anything starts.
ros2 run jl_blocks jl_blocks check "$MISSION" || exit 1
NAME=$(python3 -c "import sys, yaml; print(yaml.safe_load(open(sys.argv[1]))['name'])" "$MISSION")
export GZ_PARTITION="sitl_mission_$NAME"
trap kill_flight EXIT

echo "== flight: $NAME ($WORLD world) =="
logs=$(mktemp -d)
if [ "$WORLD" = aruco ]; then
  start_px4_aruco "$logs" || { echo "logs: $logs"; exit 1; }
else
  start_px4 "$logs"
fi

ros2 launch jl_blocks mission.launch.py mission_file:="$MISSION" > "$logs/launch.log" 2>&1 &
for _ in $(seq 60); do grep -q "Registered '$NAME'" "$logs/launch.log" && break; sleep 1; done
if ! grep -q "Registered '$NAME'" "$logs/launch.log"; then
  echo "FAIL $NAME registered with PX4 (see $logs/launch.log)"; echo "logs: $logs"; exit 1
fi

# The watcher must be listening before the mission starts, because events are
# not latched. Its timeout counts from here.
watch_args=(-p mission_name:="$NAME" -p expect:="$EXPECT" -p finished:=$FINISHED
            -p tolerance:=$TOL -p hold_s:=$HOLD -p timeout_s:=$TIMEOUT)
[ -n "$CAMERA" ] && watch_args+=(-p camera_target:="$CAMERA")
ros2 run jl_blocks mission_watch --ros-args "${watch_args[@]}" > "$logs/watch.log" 2>&1 &
watch=$!
sleep 3
# Select the mission; retry slowly (spec section 8: >= 10 s apart, since
# re-selecting mid-takeoff restarts the executor) only if it never activated.
for attempt in 1 2 3; do
  (cd "$logs/rootfs" && "$PX4_BUILD/bin/px4-commander" mode ext1) >> "$logs/commander.log" 2>&1
  for _ in $(seq 12); do grep -q "event: activated" "$logs/watch.log" && break 2; sleep 1; done
  echo "mission not active after selection attempt $attempt; selecting again" >> "$logs/commander.log"
done

wait $watch; rc=$?
cat "$logs/watch.log"
if grep -qE "Traceback|Assertion|terminate called" "$logs/launch.log"; then
  echo "FAIL no crash in the runner or the mode"; rc=1
else
  echo "PASS no crash in the runner or the mode"
fi
[ $rc -ne 0 ] && echo "logs: $logs"
exit $rc
```

`chmod +x test/sitl_mission.sh`.

- [ ] **Step 3: Trim `test/sitl_runner_smoke.sh` and add `test/sitl_all.sh`**

In `test/sitl_runner_smoke.sh`:
- delete the first flight: everything from `echo "== flight: runner smoke ($NAME) =="` through the `kill_flight` after it, plus the `NAME=TakeoffHoldLand` and `MISSION=…` lines;
- initialise `rc=0` before the remaining runner-dies flight;
- update the header comment to say it checks that a dying runner leaves jl_mission to hold, then land. The takeoff→hold→land flight now runs through `test/sitl_mission.sh`.

Everything in the runner-dies flight stays as it is.

Create `test/sitl_all.sh` (`chmod +x`):

```bash
#!/bin/bash
# Every L3 flight (spec section 8), each from a fresh PX4 start. Stops at the
# first script that fails. Run inside the sim container after colcon build.
HERE="$(dirname "$(readlink -f "$0")")"
ROOT="$HERE/.."
set -e
"$ROOT/src/jl_mission/test/sitl_jl_mission.sh"
"$HERE/sitl_mission.sh" "$ROOT/missions/takeoff_hold_land.yaml" \
  --expect "takeoff hold land" --finished
"$HERE/sitl_runner_smoke.sh"
echo "== all L3 flights passed =="
```

(Task 4 adds the ArUco acceptance flight before the final `echo`.)

- [ ] **Step 4: Check the fast failure paths (Review Focus 5)**

In the container:

```bash
test/sitl_mission.sh missions/takeoff_hold_land.yaml; echo "exit=$?"
test/sitl_mission.sh missions/takeoff_hold_land.yaml --expect "takeoff" --world moon; echo "exit=$?"
printf "name: Bad\nsteps:\n  - holdd: {}\n" > /tmp/bad.yaml
test/sitl_mission.sh /tmp/bad.yaml --expect "hold"; echo "exit=$?"
```

Expected, each in a few seconds with no PX4 started:
- `--expect is required`, then the usage text, `exit=2`;
- `unknown world: moon`, usage, `exit=2`;
- `jl_blocks check`'s `FAIL /tmp/bad.yaml … did you mean 'hold'?`, `exit=1`.

- [ ] **Step 5: Makefile**

Replace the `sitl-test` recipe's `docker exec` line with:

```make
	docker exec --user user -w $(CURDIR) $(SIM_CONTAINER) bash -c 'source /opt/ros/humble/setup.bash && source install/setup.bash && test/sitl_all.sh'
```

After the `sitl-test` recipe, add:

```make
# Fly one mission file headless and check it, e.g.
#   make sitl-mission MISSION=missions/takeoff_hold_land.yaml ARGS='--expect "takeoff hold land" --finished'
# See test/sitl_mission.sh for the options. Also restarts the container.
sitl-mission:
	docker restart $(SIM_CONTAINER) > /dev/null
	sleep 3
	docker exec --user user -w $(CURDIR) $(SIM_CONTAINER) bash -c 'source /opt/ros/humble/setup.bash && source install/setup.bash && test/sitl_mission.sh $(MISSION) $(ARGS)'
```

Add `sitl-mission` to `.PHONY`.

- [ ] **Step 6: Run the L3 suite**

Run: `make sitl-test` from the host.
Expected, then exit code 0:
- the three jl_mission flights all-PASS;
- `== flight: TakeoffHoldLand (default world) ==` with its `event:` lines, then these four lines:
  - `PASS steps started in order: takeoff hold land`
  - `PASS mission did not abort`
  - `PASS mission finished`
  - `PASS no crash in the runner or the mode`
- the runner-dies flight all-PASS;
- `== all L3 flights passed ==`.

- [ ] **Step 7: Checkpoint**

Stop and report the diff and the Step 4 and Step 6 output to the maintainer. Do not commit.

---

### Task 4: The ArUco world and the acceptance mission

**Files:**
- Create: `missions/aruco_standoff.yaml`
- Modify:
  - `test/sitl_common.sh` (new `start_px4_aruco`);
  - `test/sitl_all.sh` (the acceptance flight);
  - `docs/superpowers/specs/2026-09-21-mission-blocks-design.md` (§8, §11).

**Interfaces:**
- Consumes: `test/sitl_mission.sh --world aruco --camera-standoff` (Task 3); `src/aruco_tracker/launch/front_camera_aruco.launch.py`, used as it is (tag id 0, 0.15 m, remapped to `/front/target_pose`).
- Produces: `start_px4_aruco LOGDIR` (returns 1, after a `FAIL … did not start` line, if Gazebo or PX4 never comes up), and the acceptance flight in `make sitl-test`.

**The world** is `gazebo/worlds/aruco_dual_ids.sdf`:
- The vertical tag (id 0, 0.15 m) is at Gazebo ENU (1, 0, 1), facing the drone.
- `x500_dual_cam` is spawned at the origin facing +x (east), so its front camera sees the tag 1 m ahead and 1 m up.
- In PX4's NED frame the tag is at about (0, 1, −1).

The mission climbs to the tag's height, sees it, and `track` backs off to 3 m along its own line of approach, just as `aruco_sitl.md` shows DroneSmoothPlanner doing. The camera then sees the tag at about (0, 0, 3).

- [ ] **Step 1: The acceptance mission**

Create `missions/aruco_standoff.yaml`:

```yaml
# L3 acceptance (spec section 8): the DroneSmoothPlanner ArUco demo
# (src/drogue_flight/docs/aruco_sitl.md) as a mission file. The front camera
# finds the vertical tag, and the drone keeps 3 m in front of it.
# Flown by test/sitl_all.sh in gazebo/worlds/aruco_dual_ids.sdf.
name: ArucoStandoff
target:
  aruco_tag: {camera: front}

steps:
  - takeoff: {height: 1.0}
  - search: {pattern: hold}
    until: target_seen
    timeout: 30
    on_fail: land
  - track: {standoff: 3.0}
    until: never
    on_fail: land
  - land: {}
```

Run `make check`. Expected: the mission-lint stage prints `ok   missions/aruco_standoff.yaml  (ArucoStandoff, 4 steps)`.

- [ ] **Step 2: `start_px4_aruco`**

Append to `test/sitl_common.sh`:

```bash
# Start the ArUco world headless: Gazebo with rendering (the cameras need it)
# loading gazebo/worlds/aruco_dual_ids.sdf, the x500_dual_cam vehicle, PX4
# attached to it in standalone mode, the agent, the translation node, and the
# front aruco_tracker. The same setup as launch_scripts/aruco_smooth_planner.sh,
# without the GUI. $1: log dir. Returns 1 if Gazebo or PX4 never comes up.
start_px4_aruco() {
  local logs="$1" world=aruco_dual_ids
  mkdir -p "$logs/rootfs" && cp "$PX4_BUILD/rootfs/gz_env.sh" "$logs/rootfs/"
  source "$PX4_BUILD/rootfs/gz_env.sh"
  export GZ_SIM_RESOURCE_PATH="$GZ_SIM_RESOURCE_PATH:$JL_WS_ROOT/gazebo/models:$JL_WS_ROOT/gazebo/worlds"
  export GZ_IP=127.0.0.1
  gz sim -s -r --headless-rendering "$JL_WS_ROOT/gazebo/worlds/$world.sdf" > "$logs/gz.log" 2>&1 &
  for _ in $(seq 60); do
    gz service -l 2>/dev/null | grep -q "/world/$world/create" && break; sleep 1
  done
  if ! gz service -l 2>/dev/null | grep -q "/world/$world/create"; then
    echo "FAIL gazebo world $world did not start (see $logs/gz.log)"; return 1
  fi
  gz service -s "/world/$world/create" --reqtype gz.msgs.EntityFactory \
    --reptype gz.msgs.Boolean --timeout 5000 \
    --req "sdf_filename: \"$JL_WS_ROOT/gazebo/models/x500_dual_cam/model.sdf\", name: \"x500_dual_cam_0\"" \
    >> "$logs/gz.log" 2>&1
  PX4_GZ_STANDALONE=1 PX4_GZ_WORLD=$world PX4_GZ_MODEL_NAME=x500_dual_cam_0 \
    PX4_SIM_MODEL=gz_x500_dual_cam \
    "$PX4_BUILD/bin/px4" -d -w "$logs/rootfs" "$PX4_BUILD/etc" > "$logs/px4.log" 2>&1 &
  MicroXRCEAgent udp4 -p 8888 > "$logs/agent.log" 2>&1 &
  ros2 run translation_node translation_node_bin > "$logs/translation.log" 2>&1 &
  for _ in $(seq 120); do grep -q "synchronized with time offset" "$logs/px4.log" && break; sleep 1; done
  if ! grep -q "synchronized with time offset" "$logs/px4.log"; then
    echo "FAIL PX4 did not start (see $logs/px4.log)"; return 1
  fi
  (cd "$logs/rootfs" && "$PX4_BUILD/bin/px4-param" set NAV_DLL_ACT 0) >> "$logs/commander.log" 2>&1  # no GCS in this test
  ros2 launch aruco_tracker front_camera_aruco.launch.py > "$logs/tracker.log" 2>&1 &
}
```

- [ ] **Step 3: Check the "did not start" path (Review Focus 4)**

In the container, point Gazebo at a world that doesn't exist, by temporarily running the function with a bad `JL_WS_ROOT`:

```bash
bash -c 'source test/sitl_common.sh; JL_WS_ROOT=/nonexistent start_px4_aruco "$(mktemp -d)"; echo "rc=$?"; kill_flight'
```

Expected: after about 60 s, `FAIL gazebo world aruco_dual_ids did not start (see …/gz.log)` and `rc=1`, with nothing left running. Report the output.

- [ ] **Step 4: Fly the acceptance mission and watch it pass**

```bash
docker restart jacob_ladder_sim && sleep 3
docker exec --user user -w /hdd/Documents/Electrical/Matrix_Lab/jacob_drone_ws/src/Jacob_Ladder jacob_ladder_sim bash -c '
source /opt/ros/humble/setup.bash && source install/setup.bash &&
test/sitl_mission.sh missions/aruco_standoff.yaml --world aruco \
  --expect "takeoff search track" --camera-standoff "0 0 3"'; echo "exit=$?"
```

Expected, then `exit=0`:
- `event:` lines for takeoff, search and track;
- `PASS steps started in order: takeoff search track`
- `PASS mission did not abort`
- `PASS camera saw the target within 0.25 m of (0.00, 0.00, 3.00) for 5 s`
- `PASS no crash in the runner or the mode`

If it fails, read the logs in the printed folder before changing anything:
- `tracker.log`: did it detect id 0?
- `gz.log`: did the vehicle spawn?
- `px4.log`: did PX4 attach to `x500_dual_cam_0`?
- `launch.log`: the runner's events and jl_mission's states.
- `watch.log`: the last camera position.

Fixes may go only in `test/` scripts or in `missions/aruco_standoff.yaml`. In that file only the takeoff height may change, and only if the logs show the tag outside the camera's view. Never loosen the 0.25 m / 5 s check, and never change `jl_blocks`, `jl_mission`, `gazebo/`, `aruco_tracker` or the flight packages. If the cause lies in one of those, stop and report BLOCKED with the evidence.

- [ ] **Step 5: Add it to the L3 suite**

In `test/sitl_all.sh`, before the final `echo`, add:

```bash
"$HERE/sitl_mission.sh" "$ROOT/missions/aruco_standoff.yaml" --world aruco \
  --expect "takeoff search track" --camera-standoff "0 0 3"
```

Run `make sitl-test` from the host, **twice**. Expected both times: every flight all-PASS, `== all L3 flights passed ==`, exit code 0. Report the PASS/FAIL lines of both runs.

- [ ] **Step 6: Fly the spec §4 example once, by hand (report only, not gating)**

```bash
docker restart jacob_ladder_sim && sleep 3
docker exec --user user -w /hdd/Documents/Electrical/Matrix_Lab/jacob_drone_ws/src/Jacob_Ladder jacob_ladder_sim bash -c '
source /opt/ros/humble/setup.bash && source install/setup.bash &&
test/sitl_mission.sh missions/track_moving_aruco.yaml --world aruco \
  --expect "takeoff search track" --camera-standoff "0 0 2" --timeout 180'; echo "exit=$?"
```

Report the result either way, with the last camera position if it failed. Do not change anything to make it pass. It uses the `pid` controller, whose flown vertical speed limit is 0.1 m/s, so it may just be slow.

- [ ] **Step 7: Spec updates**

In `docs/superpowers/specs/2026-09-21-mission-blocks-design.md`, §8's table, replace the L3 row with:

```markdown
| L3 SITL | a mission flies headless and its step events on `/jl/NAME/events` arrive in the expected order (e.g. `takeoff → search → track`), plus optional checks: finished, and where the front camera sees the target | `test/sitl_mission.sh <mission.yaml> --expect "…"`; every L3 flight: `test/sitl_all.sh` | `jacob_ladder_sim` |
```

In §11, append:

```markdown
- Phase 4's ArUco acceptance measures the 3 m standoff in the front camera's frame (the tag at (0, 0, 3) ± 0.25 m for 5 s, as `aruco_tracker` reports it), not against Gazebo ground truth: the tag's local-frame height depends on the EKF origin and the camera mount, which move it by about 0.2 m.
```

- [ ] **Step 8: Final check**

Run: `make check`
Expected: every stage passes. Report the pytest count.

- [ ] **Step 9: Checkpoint**

Stop and report to the maintainer:
- the diff;
- the Step 3, 4, 5 and 6 output;
- the pytest count.

Do not commit.

---

## Maintainer notes (found while planning; not tasks)

- **Rulings to confirm:** the camera-frame measurement (Ruling 1), the `position` controller for the acceptance flight with `track_moving_aruco.yaml` as report-only (Ruling 2), and events over state (Ruling 3).
- **`make sitl-test` gets longer:** roughly 3.5 min for the jl_mission flights, 1 min for takeoff→hold→land, 1.5 min for the runner-dies flight and 1.5 min for the ArUco flight.
- **Not in this phase:** Phase 5 (`config/flight.yaml`, `deploy.sh`, the Jetson timing measurement) and Phase 6 (the beginner tutorial, which can point at `make sitl-mission`).
- **Real drone:** nothing here changes what the drone runs. The acceptance test proves the front-camera chain in SITL only: `aruco_tracker` → `aruco_tag` → `track` → `jl_mission` → PX4.
