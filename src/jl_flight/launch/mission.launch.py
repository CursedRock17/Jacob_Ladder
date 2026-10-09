"""Fly one mission file: its jl_mission mode (the name shown in QGC), the
perception helpers it declares, and the mission itself.

    ros2 launch jl_flight mission.launch.py mission:=missions/land_before_apriltag.py
    ros2 launch jl_flight mission.launch.py mission:=... airframe:=x500_rgbd respawn:=true
    ros2 launch jl_flight mission.launch.py mission:=... report:=0.5     # fly, and print setpoints

The mission waits for its mode to be selected, flies, and exits. With
respawn:=true (the boot service) it restarts afterwards and waits for the
mode to be selected *again*, so it never flies twice on one selection.
jl_mission stays up throughout, so its watchdog holds, then lands, if the
mission process dies mid-flight.
"""

from __future__ import annotations

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from jl_flight.airframes import get_airframe
from jl_flight.missions import load


def _nodes(context):
    path = os.path.abspath(LaunchConfiguration("mission").perform(context))
    airframe = get_airframe(LaunchConfiguration("airframe").perform(context) or None)
    respawn = LaunchConfiguration("respawn").perform(context).lower() == "true"
    report = LaunchConfiguration("report").perform(context)
    spec = load(path)
    spec.check(airframe)

    runner = ["python3", "-m", "jl_flight", "run", path, "--airframe", airframe.name]
    if respawn:
        runner.append("--fresh-selection")
    if report:
        runner += ["--report", report]
    actions = [
        Node(
            package="jl_mission",
            executable="jl_mission",
            name=f"jl_mission_{spec.name.lower()}",
            output="screen",
            parameters=[{"mission_name": spec.name}],
        ),
        ExecuteProcess(
            cmd=runner,
            name=f"mission_{spec.name.lower()}",
            output="screen",
            respawn=respawn,
            respawn_delay=2.0,
            # Unbuffered, so the mission's own prints reach the log as they happen.
            additional_env={"PYTHONUNBUFFERED": "1"},
        ),
    ]
    for helper in spec.helpers(airframe):
        actions.append(
            ExecuteProcess(cmd=list(helper.argv), name=helper.name, output="screen")
        )
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("mission", description="a mission .py file"),
            DeclareLaunchArgument(
                "airframe",
                default_value=os.environ.get("JL_AIRFRAME", ""),
                description="airframe name (default: $JL_AIRFRAME, else generic)",
            ),
            DeclareLaunchArgument(
                "report",
                default_value="",
                description="print the setpoint, drone and targets every this many seconds",
            ),
            DeclareLaunchArgument(
                "respawn",
                default_value="false",
                description="restart the mission when it ends (boot services set true)",
            ),
            OpaqueFunction(function=_nodes),
        ]
    )
