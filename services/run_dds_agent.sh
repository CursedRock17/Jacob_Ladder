#!/bin/bash
# Source our Environment (paths resolved from this script's location — see jl_env.sh)
source "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/../jl_env.sh"
cd "$JL_WS_ROOT" || exit 1
jl_source_ros

# Run the Micro XRCE DDS Agent.
# XRCE_DEV selects the serial link to the flight controller:
#   /dev/ttyUSB0  — USB-TTL adapter (default)
#   /dev/ttyTHS1  — Jetson 40-pin header UART: pins 8 (TX) / 10 (RX) / 6 (GND)
# Override via the environment (e.g. a `systemctl edit dds_agent` drop-in with
# `Environment=XRCE_DEV=/dev/ttyTHS1`); see DRONE_SETUP.md.
MicroXRCEAgent serial --dev "${XRCE_DEV:-/dev/ttyTHS1}" -b "${XRCE_BAUD:-921600}"
