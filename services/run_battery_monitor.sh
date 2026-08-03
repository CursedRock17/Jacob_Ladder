#!/bin/bash
# Source our Environment (paths resolved from this script's location — see jl_env.sh)
source "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/../jl_env.sh"
cd "$JL_WS_ROOT" || exit 1

# No ROS here: the monitor talks to the INA238 over I2C directly and must keep
# alerting even when the ROS graph or the DDS agent is down — that is exactly
# when nobody is watching the voltage.
#
# Deliberately do NOT source the workspace venv unconditionally, unlike the
# other services here. The venv ships jetson_cudss_bootstrap.pth, which
# preloads the cuDSS CUDA runtime at interpreter startup — that costs ~120 MB
# RSS (138 MB vs 19 MB measured) for a service whose entire job is reading
# four I2C registers, and this box also has to hold YOLO, torch and cuVSLAM.
#
# The monitor needs only smbus2 and PyYAML, both present in system
# site-packages on the ARK JetPack image. Fall back to the venv only if they
# are genuinely missing, so a machine without them still works.
if ! python3 -c 'import smbus2, yaml' >/dev/null 2>&1; then
    echo "run_battery_monitor: smbus2/PyYAML not in system python, using $JL_VENV" >&2
    jl_source_venv
fi

# systemd's StandardOutput=journal is a pipe, and Python block-buffers stdout
# on a pipe. Without this, a LAND alert can sit in a 4 KB buffer instead of
# reaching the journal — the log would lag the emergency it is reporting.
export PYTHONUNBUFFERED=1

exec python3 -m battery_monitor --journal "$@"
