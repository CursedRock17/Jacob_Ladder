#!/bin/bash
# SITL check of the jl_mission safety contract (spec section 5), headless.
# Three flights, each from a fresh PX4 start:
#   silence:  takeoff to the exact height, relay, reject an invalid setpoint
#             (hold), and land after 5 s of silence.
#   land:     takeoff, relay, then the runner calls /jl/NAME/land: the mode
#             must not re-enter land() (finding C1) and must answer the land
#             request exactly once (finding I2), and must never publish
#             "relay" again after "landing" (F4).
#   takeover: takeoff, relay, then the pilot switches to posctl mid-air: the
#             mission must not fight for control (F5).
# Run inside the sim container after colcon build.
#   src/jl_mission/test/sitl_jl_mission.sh
HERE="$(dirname "$(readlink -f "$0")")"
source "$HERE/../../../test/sitl_common.sh"
NAME=JlSitlCheck
export GZ_PARTITION=sitl_jl_mission

overall_rc=0
fail_logs=()

trap kill_flight EXIT

expect_in() {
  local logs="$1" needle="$2" what="$3"
  if grep -q "$needle" "$logs/runner.log"; then echo "PASS $what"; else echo "FAIL $what"; rc=1; fi
}

# Flight 1: takeoff, relay, reject an invalid setpoint, land after silence.
run_silence_flight() {
  echo "== flight: silence =="
  local logs; logs=$(mktemp -d)
  start_px4 "$logs"

  ros2 run jl_mission jl_mission --ros-args -p mission_name:=$NAME > "$logs/mode.log" 2>&1 &
  for _ in $(seq 60); do grep -q "Registered '$NAME'" "$logs/mode.log" && break; sleep 1; done
  python3 "$HERE/fake_runner.py" $NAME 1.5 silence > "$logs/runner.log" 2>&1 &
  sleep 2
  (cd "$logs/rootfs" && "$PX4_BUILD/bin/px4-commander" mode ext1) >> "$logs/commander.log" 2>&1

  # The whole flight takes about 45 s from selecting the mode to landed
  for _ in $(seq 90); do grep -q "STATE landed" "$logs/runner.log" && break; sleep 1; done
  cat "$logs/runner.log"

  local rc=0
  expect_in "$logs" "TAKEOFF success=True" "takeoff reported success"
  local ground z
  ground=$(grep -o "TAKEOFF success=True .* ground_z=-\?[0-9.]*" "$logs/runner.log" | grep -o "ground_z=-\?[0-9.]*" | cut -d= -f2)
  z=$(grep -o "TAKEOFF success=True .* z=-\?[0-9.]*" "$logs/runner.log" | grep -o " z=-\?[0-9.]*" | cut -d= -f2)
  if python3 -c "import sys; sys.exit(0 if abs((float('${ground:-0}') - float('${z:-0}')) - 1.5) <= 0.15 else 1)"; then
    echo "PASS takeoff height (ground_z=${ground} z=${z}, want 1.5 +/- 0.15)"
  else
    echo "FAIL takeoff height (ground_z=${ground:-none} z=${z:-none}, want 1.5 +/- 0.15)"; rc=1
  fi
  expect_in "$logs" "STATE relay" "relayed the runner's setpoints"
  grep -q "Rejected setpoint: position contains infinity" "$logs/mode.log" && echo "PASS invalid setpoint rejected" || { echo "FAIL invalid setpoint rejected"; rc=1; }
  expect_in "$logs" "STATE hold (no valid setpoint)" "held on silence"
  expect_in "$logs" "STATE landing (no valid setpoint)" "landed after prolonged silence"
  expect_in "$logs" "STATE landed" "reached the ground"

  kill_flight
  if [ $rc -ne 0 ]; then echo "logs: $logs"; fail_logs+=("$logs"); overall_rc=1; fi
}

# Flight 2: takeoff, relay, then the runner requests a land mid-mission.
run_land_flight() {
  echo "== flight: land =="
  local logs; logs=$(mktemp -d)
  start_px4 "$logs"

  ros2 run jl_mission jl_mission --ros-args -p mission_name:=$NAME > "$logs/mode.log" 2>&1 &
  for _ in $(seq 60); do grep -q "Registered '$NAME'" "$logs/mode.log" && break; sleep 1; done
  python3 "$HERE/fake_runner.py" $NAME 1.5 land > "$logs/runner.log" 2>&1 &
  sleep 2
  (cd "$logs/rootfs" && "$PX4_BUILD/bin/px4-commander" mode ext1) >> "$logs/commander.log" 2>&1

  # Takeoff (~10 s) + relay/velocity/freeze phases to t=17 s + land: about 40 s to landed
  for _ in $(seq 90); do grep -q "STATE landed" "$logs/runner.log" && break; sleep 1; done
  cat "$logs/runner.log"

  local rc=0
  expect_in "$logs" "TAKEOFF success=True" "takeoff reported success"
  expect_in "$logs" "STATE relay" "relayed the runner's setpoints"
  expect_in "$logs" "LAND success=True message='landed'" "land reported success"
  expect_in "$logs" "STATE landed" "reached the ground"
  local maxerr
  maxerr=$(grep -o "MAXERR [0-9.]*" "$logs/runner.log" | tail -1 | cut -d' ' -f2)
  if [ -n "$maxerr" ] && python3 -c "import sys; sys.exit(0 if float('$maxerr') <= 0.3 else 1)"; then
    echo "PASS F1: tracking error stayed <= 0.3 m after a velocity-only relay (MAXERR $maxerr)"
  else
    echo "FAIL F1: tracking error exceeded 0.3 m after a velocity-only relay (MAXERR ${maxerr:-none})"
    rc=1
  fi
  local last_relay_line first_landing_line
  last_relay_line=$(grep -n "STATE relay" "$logs/runner.log" | tail -1 | cut -d: -f1)
  first_landing_line=$(grep -n "STATE landing" "$logs/runner.log" | head -1 | cut -d: -f1)
  if [ -n "$last_relay_line" ] && [ -n "$first_landing_line" ] && \
     [ "$last_relay_line" -lt "$first_landing_line" ]; then
    echo "PASS no STATE relay after STATE landing"
  else
    echo "FAIL no STATE relay after STATE landing (last relay line $last_relay_line, first landing line $first_landing_line)"
    rc=1
  fi
  if grep -qE "Assertion|terminate called" "$logs/mode.log"; then
    echo "FAIL no assert/terminate in mode.log"; rc=1
  else
    echo "PASS no assert/terminate in mode.log"
  fi

  kill_flight
  if [ $rc -ne 0 ]; then echo "logs: $logs"; fail_logs+=("$logs"); overall_rc=1; fi
}

# Flight 3: takeoff, relay, then the pilot switches to posctl mid-air (F5):
# the mission must not fight for control.
run_takeover_flight() {
  echo "== flight: takeover =="
  local logs; logs=$(mktemp -d)
  start_px4 "$logs"

  ros2 run jl_mission jl_mission --ros-args -p mission_name:=$NAME > "$logs/mode.log" 2>&1 &
  for _ in $(seq 60); do grep -q "Registered '$NAME'" "$logs/mode.log" && break; sleep 1; done
  python3 "$HERE/fake_runner.py" $NAME 1.5 silence > "$logs/runner.log" 2>&1 &
  sleep 2
  (cd "$logs/rootfs" && "$PX4_BUILD/bin/px4-commander" mode ext1) >> "$logs/commander.log" 2>&1

  for _ in $(seq 60); do grep -q "TAKEOFF success=True" "$logs/runner.log" && break; sleep 1; done
  # Headless SITL has no RC/joystick: PX4's posctl arming check silently
  # denies the mode switch without one (see fake_pilot.py). Start the fake
  # pilot's stick input before requesting the switch, as a real RC would
  # already be live.
  if [ "${JL_SKIP_TAKEOVER:-0}" != "1" ]; then
    python3 "$HERE/fake_pilot.py" > "$logs/pilot.log" 2>&1 &
  fi
  sleep 6
  if [ "${JL_SKIP_TAKEOVER:-0}" != "1" ]; then
    (cd "$logs/rootfs" && "$PX4_BUILD/bin/px4-commander" mode posctl) >> "$logs/commander.log" 2>&1
  fi
  sleep 2
  (cd "$logs/rootfs" && "$PX4_BUILD/bin/px4-commander" status) >> "$logs/commander.log" 2>&1
  sleep 6
  for _ in $(seq 20); do grep -q "LAND_AFTER_TAKEOVER" "$logs/runner.log" && break; sleep 1; done
  cat "$logs/runner.log"
  echo "px4 mode after the switch:"
  grep -i "nav_state\|Current mode" "$logs/commander.log" | tail -5

  local rc=0
  expect_in "$logs" "TAKEOFF success=True" "takeoff reported success"

  local first_active_true_line first_active_false_line
  first_active_true_line=$(grep -n "ACTIVE true" "$logs/runner.log" | head -1 | cut -d: -f1)
  first_active_false_line=$(grep -n "ACTIVE false" "$logs/runner.log" | tail -1 | cut -d: -f1)
  if [ -n "$first_active_true_line" ] && [ -n "$first_active_false_line" ] && \
     [ "$first_active_false_line" -gt "$first_active_true_line" ]; then
    echo "PASS active went false AFTER going true (posctl takeover proven, true@$first_active_true_line false@$first_active_false_line)"
  else
    echo "FAIL active did not go false after going true (true@${first_active_true_line:-none} false@${first_active_false_line:-none})"
    rc=1
  fi

  if grep -q "executor — deactivated" "$logs/mode.log"; then
    echo "PASS mode.log shows the executor deactivated after the switch"
  else
    echo "FAIL mode.log does not show the executor deactivating"; rc=1
  fi

  if grep -q "LAND_AFTER_TAKEOVER success=False message='mission is not active'" "$logs/runner.log"; then
    echo "PASS land request after takeover was refused (mission is not active)"
  else
    echo "FAIL land request after takeover was not refused as expected"; rc=1
  fi

  if grep -q "Landing:" "$logs/mode.log"; then
    echo "FAIL no Landing: line after the switch"; rc=1
  else
    echo "PASS no Landing: line after the switch"
  fi
  local ground z
  ground=$(grep -o "TAKEOFF success=True .* ground_z=-\?[0-9.]*" "$logs/runner.log" | grep -o "ground_z=-\?[0-9.]*" | cut -d= -f2)
  z=$(grep -o "^\[.*\] z=-\?[0-9.]*" "$logs/runner.log" | tail -1 | grep -o "z=-\?[0-9.]*" | cut -d= -f2)
  if python3 -c "import sys; sys.exit(0 if abs(float('${z:-0}') - float('${ground:-0}')) > 1.0 else 1)"; then
    echo "PASS still airborne 8 s after the switch (ground_z=${ground} z=${z})"
  else
    echo "FAIL still airborne 8 s after the switch (ground_z=${ground:-none} z=${z:-none})"; rc=1
  fi
  if grep -qE "Assertion|terminate called" "$logs/mode.log"; then
    echo "FAIL no assert/terminate in mode.log"; rc=1
  else
    echo "PASS no assert/terminate in mode.log"
  fi

  kill_flight
  if [ $rc -ne 0 ]; then echo "logs: $logs"; fail_logs+=("$logs"); overall_rc=1; fi
}

run_silence_flight
run_land_flight
run_takeover_flight

if [ ${#fail_logs[@]} -ne 0 ]; then
  echo "failed flight logs: ${fail_logs[*]}"
fi
exit $overall_rc
