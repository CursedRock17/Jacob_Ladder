from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = PathJoinSubstitution([
        FindPackageShare("oak_d_visual_odometry"),
        "config",
        "cuvslam_params.yaml",
    ])

    # Start a Foxglove bridge alongside the node so every topic (odometry, the
    # /features/image overlay, /rgb/image, IMU, TF) is exposed to Foxglove
    # Studio at ws://<host>:8765 without a separate launch.
    start_bridge = LaunchConfiguration("foxglove_bridge")
    bridge_port = LaunchConfiguration("foxglove_port")

    return LaunchDescription([
        DeclareLaunchArgument(
            "foxglove_bridge", default_value="false",
            description="Start foxglove_bridge to expose topics to Foxglove Studio. "
                        "Leave false if you already run a bridge (default port 8765 "
                        "will otherwise fail with a Bind Error)."),
        DeclareLaunchArgument(
            "foxglove_port", default_value="8765",
            description="WebSocket port for foxglove_bridge"),

        Node(
            package="oak_d_visual_odometry",
            executable="cuvslam_publisher_px4_node",
            name="oak_d_cuvslam_px4_publisher",
            output="screen",
            parameters=[params_file],
        ),

        Node(
            condition=IfCondition(start_bridge),
            package="foxglove_bridge",
            executable="foxglove_bridge",
            name="foxglove_bridge",
            output="screen",
            parameters=[{"port": ParameterValue(bridge_port, value_type=int)}],
        ),
    ])
