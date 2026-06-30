#!/usr/bin/env bash
# =============================================================================
# 02_ros2_humble.sh
# Source: Dockerfile_ros2-humble / Dockerfile_humble-arm64 (Step 3)
#
# Installs ROS 2 Humble desktop + colcon/vcstool build tooling.
# Gazebo Harmonic and ros-gzharmonic bridge are OMITTED (simulation only).
# Foxglove agent (arm64) and foxglove-bridge are included for drone use.
# =============================================================================
set -euo pipefail

export DEBIAN_FRONTEND=noninteractive
export ROS_DISTRO=humble

echo ">>> [02] Setting up locale..."
apt-get update && apt-get install -y --no-install-recommends \
    curl gnupg2 lsb-release locales \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

echo ">>> [02] Adding ROS 2 apt repository..."
ROS_APT_SOURCE_VERSION="$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
    | grep -F "tag_name" | awk -F'"' '{print $4}')"
curl -L -o /tmp/ros2-apt-source.deb \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo "$VERSION_CODENAME")_all.deb"
dpkg -i /tmp/ros2-apt-source.deb && rm /tmp/ros2-apt-source.deb

echo ">>> [02] Installing ROS 2 Humble desktop..."
apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-desktop \
    && rm -rf /var/lib/apt/lists/*

# Auto-source ROS in every bash session
if ! grep -q "source /opt/ros/${ROS_DISTRO}/setup.bash" /etc/bash.bashrc; then
    echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /etc/bash.bashrc
fi

echo ">>> [02] Installing colcon, vcstool..."
apt-get update && apt-get install -y --no-install-recommends \
    python3-vcstool \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

echo ">>> [02] Installing Foxglove agent (arm64) and ROS bridge..."
curl -L -o /tmp/foxglove-agent_1.4.0_arm64.deb \
    "https://github.com/foxglove/agent/releases/download/1.4.0/foxglove-agent_1.4.0-1_arm64.deb"
dpkg -i /tmp/foxglove-agent_1.4.0_arm64.deb && rm /tmp/foxglove-agent_1.4.0_arm64.deb
apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-foxglove-bridge \
    && rm -rf /var/lib/apt/lists/*

echo ">>> [02] ROS 2 Humble installed. Re-source your shell or open a new terminal."
