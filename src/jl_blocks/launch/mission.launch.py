"""Fly one mission file: its jl_mission mode (the name shown in QGC) plus the runner.

ros2 launch jl_blocks mission.launch.py mission_file:=missions/takeoff_hold_land.yaml
ros2 launch jl_blocks mission.launch.py mission_file:=m.yaml blocks:=my_blocks

With respawn_runner:=true (the boot service sets this) the mission_runner is
restarted 2 s after it exits. A respawned runner ignores a mission that was
already active, so it never resumes mid-flight; jl_mission holds, then lands,
in the meantime.
"""

from __future__ import annotations

import os

import yaml
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _nodes(context):
    path = os.path.abspath(LaunchConfiguration("mission_file").perform(context))
    blocks = LaunchConfiguration("blocks").perform(context)
    respawn = LaunchConfiguration("respawn_runner").perform(context).lower() == "true"
    with open(path, encoding="utf-8") as f:
        doc = yaml.safe_load(f)
    name = doc.get("name") if isinstance(doc, dict) else None
    if not isinstance(name, str):
        raise RuntimeError(f"{path} has no mission name; run: jl_blocks check {path}")
    block_paths = ",".join(
        os.path.abspath(p.strip()) for p in blocks.split(",") if p.strip()
    )
    suffix = name.lower()
    mission = Node(
        package="jl_mission",
        executable="jl_mission",
        name=f"jl_mission_{suffix}",
        output="screen",
        parameters=[{"mission_name": name}],
    )
    runner = Node(
        package="jl_blocks",
        executable="mission_runner",
        name=f"mission_runner_{suffix}",
        output="screen",
        parameters=[{"mission_file": path, "blocks": block_paths}],
        respawn=respawn,
        respawn_delay=2.0,
    )
    return [
        mission,
        runner,
        # If the runner dies, jl_mission must stay up: its own watchdog holds
        # on silence and then lands (spec §5). Shutting the whole launch down
        # here would kill jl_mission mid-hold with nothing left to land the
        # vehicle. Just log it so it's visible in the launch output.
        RegisterEventHandler(
            OnProcessExit(
                target_action=runner,
                on_exit=[
                    LogInfo(
                        msg=(
                            "mission_runner exited; jl_mission stays up so its "
                            "watchdog holds, then lands"
                        )
                    )
                ],
            )
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("mission_file", description="mission YAML file"),
            DeclareLaunchArgument(
                "blocks",
                default_value="",
                description="comma-separated .py files or folders of your own blocks",
            ),
            DeclareLaunchArgument(
                "respawn_runner",
                default_value="false",
                description="restart mission_runner if it exits (the boot service sets true)",
            ),
            OpaqueFunction(function=_nodes),
        ]
    )
