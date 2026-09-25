# Handoff: finish Mission Blocks Phase 5 (for a coding agent)

You are picking up **Phase 5** of the mission-blocks work in this repo, part-way through. Read this whole file before touching anything.

## 1. What to read, in order

1. This file.
2. The plan you are executing: `docs/superpowers/plans/2026-09-24-mission-blocks-phase5-deploy.md`. It has exact code, tests and expected output for every task. Treat it as your requirements, **except where section 4 below corrects it**.
3. The spec it implements: `docs/superpowers/specs/2026-09-21-mission-blocks-design.md`. §7 Deployment matters most, then §5 and §6.
4. The progress ledger: `.superpowers/sdd/2026-09-24-mission-blocks-phase5-deploy/progress.md`. It lists every decision already made. Append to it as you go.

## 2. House rules (these override your defaults)

- **Never commit, and never run `git add/commit/stash/checkout/reset/restore`.** The maintainer commits. Leave all changes in the working tree. Work on `main`.
- **Never modify flight code:** `src/precision_land/`, `src/drogue_flight/`, `src/aruco_tracker/`, `src/ros2_yolo_image_processing/`, the `src/jl_mission` C++, or `gazebo/`. The real drone flies this code today.
- **Never edit existing boot units:** no existing `services/*.service.in` or `services/run_*.sh`. New files are fine.
- **Never run `sudo`, `systemctl`, `services/install_services.sh` without `--dry-run`, or `deploy.sh` without `--dry-run` on this machine.** Those belong to the drone.
- `jl_blocks.core`, `jl_blocks.flight`, `jl_blocks.readiness`, `jl_blocks.timing` and `jl_blocks.testing` must stay **ROS-free** (standard library + yaml only). Only `src/jl_blocks/jl_blocks/ros/` may import `rclpy`/`px4_msgs`.
- **TDD:** write the plan's tests first, run them and see them fail, implement, then see them pass.
- Keep code like the surrounding code: brief English comments, small functions, `from __future__ import annotations`, double quotes. If `ruff format --check` fails, run `.check-venv/bin/ruff format src/jl_blocks test` and keep ruff's version.

## 3. Commands

- **Host checks, after every task:** `make check` from the repo root. It runs ruff, ruff format, ty (skipping `jl_blocks/ros/`), pytest, and mission-file lint. It is currently green: **197 passed**.
- **ROS / SITL runs inside the Docker container `jacob_ladder_sim`:**
  ```bash
  docker restart jacob_ladder_sim && sleep 3        # always first: clears leftovers
  docker exec --user user -w /hdd/Documents/Electrical/Matrix_Lab/jacob_drone_ws/src/Jacob_Ladder jacob_ladder_sim \
    bash -c 'source /opt/ros/humble/setup.bash && colcon build --packages-select jl_blocks && source install/setup.bash && <your command>'
  ```
  - Headless SITL helpers are in `test/sitl_common.sh`. `source` it, set `GZ_PARTITION=<something unique>`, and call `start_px4 "$(mktemp -d)"` or `start_px4_aruco "$(mktemp -d)"`. `kill_flight` cleans up.
  - In that container, px4, MicroXRCEAgent, translation_node_bin, and jl_mission while it waits for the FMU all **ignore SIGTERM**. Use `kill -9`, or just `docker restart` the container.
- **Full L3 suite:** `make sitl-test` from the host. It restarts the container itself and takes about 8 min. It is known to hit a **one-in-eight ArUco takeoff flake**: PX4's native takeoff doesn't finish within jl_mission's 30 s. If only that flight fails, re-run it once, and report both results.

## 4. Lessons already learned: the plan text is wrong in these places

- **Console scripts are not on PATH.** `setup.cfg` installs them to `lib/jl_blocks`. From shell scripts, call the CLI as `python3 -m jl_blocks.cli …`, or use `ros2 run jl_blocks <exe>`. Never call a bare `jl_blocks`. `services/run_jl_mission.sh` already does this.
- **`ros2 launch` rejects an empty `name:=` argument.** Only pass an argument when it has a value. See the bash-array pattern in `services/run_jl_mission.sh`.
- **Versioned PX4 topics.** `translation_node` republishes versioned messages under a `_v1` suffix in this workspace's message layout. The runner uses `/fmu/out/vehicle_local_position_v1` for that reason.
  - Task 3 found that `/fmu/out/vehicle_status` produced RTPS payload-size warnings and no messages in SITL. That is the same kind of mismatch. Check `ros2 topic list | grep vehicle_status` with `translation_node` running (`start_px4` starts it). Use the versioned name if there is one, e.g. `/fmu/out/vehicle_status_v1`, both in the readiness probe and in the `--armed` check, with a comment saying why. Do the same check for `VehicleOdometry` / `/fmu/in/vehicle_visual_odometry`.
- **Subagents / reviewers must be read-only.** If you use any, a reviewer that edits files is a defect.

## 5. Where things stand

| Task (plan numbering) | State |
|---|---|
| 1 `config/flight.yaml` + `jl_blocks.flight` + `jl_blocks flight …` CLI | **Done and reviewed.** `RESERVED_NAMES` was corrected to the real mode names, and a test checks them against the flight code's `k*ModeName` constants. |
| 2 `jl_mission@.service.in`, `run_jl_mission.sh`, `respawn_runner`, `install_services.sh` skip-templates | **Done and reviewed.** Verified in SITL: registers, respawns the runner after `kill -9`, and a missing mission exits 1. |
| 3 Readiness report (`jl_blocks.readiness`, `ros/readiness_probe.py`, `readiness` entry point) | **Code written, unit tests pass, NOT finished.** The SITL check (plan Task 3 Step 5) failed on `vehicle_status`; see section 4. No task report was written. |
| 4 `deploy.sh` + `test/test_deploy.py` + Makefile + README section | **Not started.** The plan text was already fixed for the stub installer, `JL_DEPLOY_NO_SOURCE` and the venv python. |
| 5 `setpoint_timing` + SITL baseline + spec notes | **Not started.** |
| Final whole-branch review | **Not started.** |

## 6. What to do

1. **Finish Task 3.**
   - Read `src/jl_blocks/jl_blocks/readiness.py`, `src/jl_blocks/jl_blocks/ros/readiness_probe.py` and `src/jl_blocks/test/test_readiness.py` against plan Task 3.
   - Fix the topic names (section 4) and confirm `setup.py` has the `readiness` entry point.
   - Then run plan Task 3 Step 5 in the container and check the three expected results:
     - first probe: `✓ FC data arriving`, `✗ VIO … 0 Hz`, `✓ Registered in PX4 TakeoffHoldLand`, exit 1;
     - `Nope`: `✗ Registered in PX4 missing: Nope`, exit 1;
     - `--armed`: `disarmed`, exit 0.
   - Write `.superpowers/sdd/2026-09-24-mission-blocks-phase5-deploy/task-3-report.md` with the output.
2. **Task 4 (`deploy.sh`)**, exactly as the plan gives it.
   - All 10 tests in `test/test_deploy.py` must pass under `make check` (the Makefile pytest line gains `test/test_deploy.py`).
   - If a stub interaction differs, fix `deploy.sh` or the fixture, and keep each test's intent.
   - Only ever run `deploy.sh` with `--dry-run` or through the tests.
   - Write `task-4-report.md`.
3. **Task 5 (`setpoint_timing`)**, as in the plan.
   - Run the SITL baseline (plan Task 5 Step 3) and paste the printed line into spec §6 where the plan says.
   - Add the §11 notes.
   - Write `task-5-report.md`.
4. **Final check.**
   - `make check`, then `make sitl-test` once. Re-run it once if only the ArUco flight fails.
   - Then review the whole Phase 5 diff against the plan's Global Constraints, Rulings and Review Focus lists. `git diff --stat HEAD` shows everything uncommitted, including earlier phases, so focus on the Phase 5 files listed in the plan's File Structure.
   - Confirm `git diff --stat HEAD -- src/precision_land src/drogue_flight src/aruco_tracker gazebo services/*.service.in services/run_takeoff_hold.sh services/run_vio.sh` shows nothing new beyond what the ledger already describes.
5. **Report to the maintainer.** Append a "Phase 5 done" summary to the ledger with:
   - what changed (files);
   - the `make check` count;
   - the `make sitl-test` result;
   - any decision you made and what it costs if wrong;
   - what still needs the real drone: the first real `./deploy.sh` with props off, `./deploy.sh --check` on the Jetson, and the Jetson `setpoint_timing` run.

## 7. Decisions already made (don't reopen them)

- `deploy.sh` disables only `jl_mission@*` instances and the known helpers (`vio`, `aruco_tracker`, `battery_monitor`) that `config/flight.yaml` doesn't list. It never touches `dds_agent`, `translation_node` or `takeoff_hold`.
- `deploy.sh` refuses while the vehicle is armed. An unknown arm state also refuses, unless `--force-unknown-arm-state` is given.
- The runner is respawned by the launch file (`respawn_runner:=true`), not by systemd, so jl_mission's hold-then-land watchdog is never interrupted. A respawned runner ignores an `/active` that was already latched, so it never resumes mid-flight.
- `jl_blocks flight check` rejects mission names that other boot services already register.
