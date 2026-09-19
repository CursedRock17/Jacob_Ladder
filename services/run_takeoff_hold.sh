#!/bin/bash
# Source our Environment (paths resolved from this script's location — see jl_env.sh)
source "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/../jl_env.sh"
cd "$JL_WS_ROOT" || exit 1
jl_source_ros

# Autonomous takeoff-and-hold external mode, registered with PX4 as
# "TakeoffHold" and selectable from QGC.
#
# This has to be a service, not a tmux window in super_real.sh: with no WiFi on
# the airframe there is no shell to launch it from, so a mode that is not
# started at boot simply does not exist as far as QGC is concerned.
#
# Safe to run unattended. The executor is Activation::ActivateAlways, which
# permits it to arm the vehicle *once the pilot selects the mode*; it is not
# ActivateImmediately, so registering does not arm anything.
exec ros2 launch precision_land takeoff_hold.launch.py
