#!/bin/bash
# Deploy on the Jetson: pull, validate missions, build, configure services,
# then report readiness. Stops at the first failure.
# --check runs readiness only; --dry-run previews commands without running them.
# --force-unknown-arm-state bypasses an unavailable arm probe, never an armed FC.
set -u
ROOT="${JL_DEPLOY_ROOT:-$(dirname "$(readlink -f "$0")")}"
FLIGHT="${JL_FLIGHT_FILE:-$ROOT/config/flight.yaml}"
cd "$ROOT" || exit 1

usage() {
    echo "Usage: ./deploy.sh [--check] [--dry-run] [--force-unknown-arm-state]"
    exit 2
}
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

# Load from source so validation works before the first colcon build.
flight() {
    PYTHONPATH="$ROOT/src/jl_blocks" python3 -m jl_blocks.cli flight "$@" "$FLIGHT" --root "$ROOT"
}
source_ros() {
    if [ "$DRY" -eq 0 ] && [ "${JL_DEPLOY_NO_SOURCE:-0}" != 1 ]; then
        # shellcheck source=jl_env.sh
        source "$ROOT/jl_env.sh" || fail "cannot load jl_env.sh"
        # This helper relaxes nounset while ROS setup files read unset variables.
        jl_source_ros || fail "cannot source the ROS workspace"
    fi
}

# Validate first, then keep command arguments in arrays without shell splitting.
load_config() {
    flight check || fail "fix the files above; nothing running was touched"
    local unit_text unit name
    unit_text=$(flight units) || fail "cannot read configured units"
    mapfile -t wanted <<< "$unit_text"
    names=(); helpers=()
    for unit in "${wanted[@]}"; do
        case "$unit" in
            jl_mission@*)
                name=$(python3 -c 'import sys, yaml; print(yaml.safe_load(open(sys.argv[1]))["name"])' \
                    "$ROOT/missions/${unit#jl_mission@}.yaml") || fail "cannot read mission name"
                names+=("$name") ;;
            *) helpers+=("$unit") ;;
        esac
    done
}
readiness() {
    step "5/5 readiness"
    source_ros
    run ros2 run jl_blocks readiness --names "${names[@]}" --helpers "${helpers[@]}" \
        || fail "readiness check failed"
}

if [ "$CHECK_ONLY" -eq 1 ]; then
    load_config
    readiness
    exit 0
fi

# Refuse local changes even when the FC link is unavailable.
if [ "$DRY" -eq 1 ]; then
    run git status --porcelain
else
    status=$(git status --porcelain) || fail "cannot read git status"
    [ -z "$status" ] || fail "uncommitted local changes; commit or discard them so the drone matches a commit"
fi

step "0/5 arm state"
source_ros
if [ "$DRY" -eq 1 ]; then
    run ros2 run jl_blocks readiness --armed --seconds 3
else
    arm=$(ros2 run jl_blocks readiness --armed --seconds 3 2>/dev/null); arm_rc=$?
    case "$arm_rc" in
        0) echo "disarmed" ;;
        3) fail "the vehicle is armed; land and disarm before deploying" ;;
        *)
            if [ "$FORCE_ARM" -eq 1 ]; then
                echo "arm state unknown ($arm); continuing (--force-unknown-arm-state)"
            else
                fail "can't read the arm state ($arm); is the FC link up? Re-run with --force-unknown-arm-state to deploy anyway"
            fi ;;
    esac
fi

step "1/5 git pull"
run git pull --ff-only || fail "git pull failed"
step "2/5 check $FLIGHT"
load_config
step "3/5 build"
package_text=$(flight packages) || fail "cannot read build packages"
mapfile -t packages <<< "$package_text"
run colcon build --packages-up-to "${packages[@]}" || fail "build failed; nothing running was touched"

step "4/5 services"
run "${JL_INSTALL_SERVICES:-./services/install_services.sh}" jl_mission@ || fail "installing jl_mission@.service failed"
previous=()
if [ "$DRY" -eq 1 ]; then
    run systemctl list-units --all --plain --no-legend 'jl_mission@*'
    run systemctl list-unit-files --plain --no-legend 'jl_mission@*'
    echo "would disable --now any unlisted jl_mission@ instances found above"
else
    # Include enabled instances that are not currently loaded, so they cannot
    # reappear on the next boot after being removed from flight.yaml.
    loaded=$(systemctl list-units --all --plain --no-legend 'jl_mission@*') || fail "cannot list mission units"
    installed=$(systemctl list-unit-files --plain --no-legend 'jl_mission@*') || fail "cannot list installed mission units"
    mapfile -t previous < <(printf '%s\n%s\n' "$loaded" "$installed" | awk '{print $1}' | sort -u)
fi

# Only mission instances and known helpers belong to flight.yaml.
# dds_agent, translation_node and takeoff_hold are never changed.
for unit in "${previous[@]}" vio aruco_tracker battery_monitor; do
    unit="${unit%.service}"
    case "$unit" in
        jl_mission@) continue ;;
        jl_mission@*|vio|aruco_tracker|battery_monitor) ;;
        *) continue ;;
    esac
    case " ${wanted[*]} " in
        *" $unit "*) ;;
        *) run sudo systemctl disable --now "$unit" || fail "disabling $unit failed" ;;
    esac
done
run sudo systemctl enable "${wanted[@]}" || fail "enabling units failed"
run sudo systemctl restart "${wanted[@]}" || fail "starting units failed"
if [ "$DRY" -eq 0 ]; then
    sleep "${JL_DEPLOY_SETTLE_S:-15}" || fail "waiting for readiness failed"
fi
readiness
