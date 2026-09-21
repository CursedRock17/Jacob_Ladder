#!/bin/bash
# Source our Environment (paths resolved from this script's location — see jl_env.sh)
source "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/../jl_env.sh"
cd "$JL_WS_ROOT" || exit 1
jl_source_ros

# Run the Micro XRCE DDS Agent.
# XRCE_DEV selects the serial link to the flight controller:
#   /dev/ttyTHS3  — ARK Just a Jetson UART0, the 6-pin JST-SH wired to the
#                   OrangeCube+ TELEM2 (default; session established 2026-09-01)
#   /dev/ttyTHS1  — ARK UART1 / Jetson 40-pin header: pins 8 (TX) / 10 (RX) / 6 (GND)
#   /dev/ttyUSB0  — USB-TTL adapter
# ttyTHS2 is the reserved Linux serial console -- do not use it.
# Baud must match PX4's SER_TEL2_BAUD (921600); UXRCE_DDS_CFG must select
# TELEM2 and no MAV_n_CONFIG may claim the same port, or the client never
# starts and the agent sits at `running... | fd: 3` forever.
# Override via the environment (e.g. a `systemctl edit dds_agent` drop-in with
# `Environment=XRCE_DEV=/dev/ttyTHS1`); see DRONE_SETUP.md.
MicroXRCEAgent serial --dev "${XRCE_DEV:-/dev/ttyTHS3}" -b "${XRCE_BAUD:-921600}"
