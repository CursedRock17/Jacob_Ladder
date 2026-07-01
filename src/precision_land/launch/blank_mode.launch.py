from datetime import datetime

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    qos_overrides = PathJoinSubstitution(
        [FindPackageShare("precision_land"), "cfg", "rosbag_qos_override.yaml"]
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
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            Node(
                package="precision_land",
                executable="blank_mode",
                name="blank_mode",
                output="screen",
            ),
            ros2bag_node,
        ]
    )
