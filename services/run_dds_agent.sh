#!/bin/bash
# Source our Environment
source /opt/ros/humble/setup.bash
cd /home/usmsm/Jacob_Ladder
source install/setup.bash

# Run the Micro XRCE DDS Agent.
# XRCE_DEV selects the serial link to the flight controller:
#   /dev/ttyUSB0  — USB-TTL adapter (default)
#   /dev/ttyTHS1  — Jetson 40-pin header UART: pins 8 (TX) / 10 (RX) / 6 (GND)
# Override via the environment (e.g. a `systemctl edit dds_agent` drop-in with
# `Environment=XRCE_DEV=/dev/ttyTHS1`); see DRONE_SETUP.md.
MicroXRCEAgent serial --dev "${XRCE_DEV:-/dev/ttyUSB0}" -b "${XRCE_BAUD:-921600}"
