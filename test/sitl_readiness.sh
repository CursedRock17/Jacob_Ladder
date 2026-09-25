#!/bin/bash
# Check readiness against an already running, disarmed SITL instance with
# TakeoffHoldLand registered and no VIO publisher (Phase 5 Task 3).
# Run inside jacob_ladder_sim after sourcing ROS and install/setup.bash:
#   bash test/sitl_readiness.sh
# This only probes the vehicle; it does not arm it or select a flight mode.

failed=0
probe() {
    local expected="$1" output rc pattern
    shift
    output=$(ros2 run jl_blocks readiness "$@" 2>&1)
    rc=$?
    printf '%s\nexit=%s\n' "$output" "$rc"
    if [ "$rc" -ne "$expected" ]; then
        echo "FAIL readiness exit: expected $expected, got $rc"
        failed=1
    fi
    while IFS= read -r pattern; do
        if ! grep -Eq "$pattern" <<< "$output"; then
            echo "FAIL readiness output: missing $pattern"
            failed=1
        fi
    done
}

probe 1 --names TakeoffHoldLand --helpers vio <<'EXPECTED'
^\? DDS agent session established +journalctl unavailable$
^✓ FC data arriving +/fmu/out/vehicle_status_v1$
^✗ VIO publishing +/fmu/in/vehicle_visual_odometry +0 Hz$
^✓ Registered in PX4 +TakeoffHoldLand$
EXPECTED

probe 1 --names Nope <<'EXPECTED'
^✗ Registered in PX4 +missing: Nope$
EXPECTED

probe 0 --armed <<'EXPECTED'
^disarmed$
EXPECTED

if [ "$failed" -eq 0 ]; then
    echo "PASS all three readiness probes"
fi
exit "$failed"
