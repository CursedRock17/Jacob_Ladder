"""Start one jl_mission instance: the mode that appears in QGC as MISSION_NAME.

ros2 launch jl_mission jl_mission.launch.py mission_name:=TrackMovingAruco
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    name = LaunchConfiguration("mission_name")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mission_name", description="QGC mode name, 1-24 characters"
            ),
            Node(
                package="jl_mission",
                executable="jl_mission",
                # One node per mission, so several missions can be registered at once
                name=PythonExpression(["'jl_mission_' + '", name, "'.lower()"]),
                output="screen",
                parameters=[{"mission_name": name}],
            ),
        ]
    )
