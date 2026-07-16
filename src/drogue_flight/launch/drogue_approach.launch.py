import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource



def generate_launch_description():
    params = PathJoinSubstitution(
        [FindPackageShare("drogue_flight"), "cfg", "drogue_approach_params.yaml"]
    )

    qos_overrides = PathJoinSubstitution(
        [FindPackageShare("drogue_flight"), "config", "rosbag_qos_override.yaml"]
    )
    bag_dir = "rosbags/" + f"flight_bag_{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}"
    ros2bag_node = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "record",
            "-a",
            "-o",
            bag_dir,
            "--qos-profile-overrides-path",
            qos_overrides,
        ],
        output="log",
    )
    
    # ROS 2 Node that runs the autonomous landing code
    drone_smooth_planner_node = Node(
        package="drogue_flight",
        executable="drone_smooth_planner",
        parameters=[
            {
                "num_waypoints": num_waypoints,
                "setpoint_rate_hz": setpoint_rate_hz,
                "s_curve_steepness": s_curve_steepness,
                "waypoint_tolerance_m": waypoint_tolerance_m,
                "namespace": namespace,
                "max_velocity": max_velocity,
                "max_acceleration": max_acceleration,
            }
        ],
        name="drone_smooth_planner",
        output="screen",
    )

    return LaunchDescription(
        [
            Node(
                package="drogue_flight",
                executable="drogue_approach",
                name="drogue_approach",
                output="screen",
                parameters=[params],
            ),
            ros2bag_node,
        ]
    )
