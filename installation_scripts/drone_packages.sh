#!/usr/bin/env bash
# =============================================================================
# 04_ros_drone_packages.sh
# Source: Dockerfile (main / top-level)
#
# Installs drone-relevant ROS 2 packages and system utilities.
# Foxglove Studio desktop .deb is OMITTED (was amd64-only in the Dockerfile).
# Foxglove bridge already installed in 02_ros2_humble.sh.
# =============================================================================
set -euo pipefail

export DEBIAN_FRONTEND=noninteractive
ROS_DISTRO=humble

# Source ROS so ${ROS_DISTRO} variable resolves correctly in apt package names.
# ROS setup files reference unset variables, so relax `set -u` while sourcing.
if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then
    set +u
    # shellcheck disable=SC1091
    source /opt/ros/${ROS_DISTRO}/setup.bash
    set -u
fi

echo ">>> [04] Installing drone ROS 2 packages..."
apt-get update && apt-get -y --quiet --no-install-recommends install \
    xdg-utils \
    libnotify4 \
    ros-${ROS_DISTRO}-v4l2-camera \
    ros-${ROS_DISTRO}-vision-msgs \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-rosbag2-storage-mcap \
    python3-pip \
    && apt-get -y autoremove \
    && apt-get clean autoclean \
    && rm -rf /var/lib/apt/lists/*

echo ">>> [04] Drone ROS packages installed."
