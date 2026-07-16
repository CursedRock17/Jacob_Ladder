from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # All tunable numbers live in the config file (see config/drone_smooth_planner.yaml)
    params = PathJoinSubstitution(
        [FindPackageShare("drogue_flight"), "config", "drone_smooth_planner.yaml"]
    )

    drone_smooth_planner_node = Node(
        package="drogue_flight",
        executable="drone_smooth_planner",
        name="drone_smooth_planner",
        output="screen",
        parameters=[params],
    )

    return LaunchDescription([drone_smooth_planner_node])
