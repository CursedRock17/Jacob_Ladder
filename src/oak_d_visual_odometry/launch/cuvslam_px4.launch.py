from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Which camera drives cuVSLAM. Both backends live in the same node
    # (camera_type parameter) and claim their device exclusively, so exactly
    # one runs at a time; this argument only picks which params file is
    # loaded, and each of those sets camera_type to match.
    #
    #   ros2 launch oak_d_visual_odometry cuvslam_px4.launch.py            # D435i
    #   ros2 launch oak_d_visual_odometry cuvslam_px4.launch.py camera:=oak
    camera = LaunchConfiguration("camera")

    # The two configs are not named alike (cuvslam_params.yaml is the
    # flight-proven OAK-D file and is referenced by name throughout the repo
    # docs and config/params/README.md), so map rather than string-concatenate.
    params_file = PathJoinSubstitution(
        [
            FindPackageShare("oak_d_visual_odometry"),
            "config",
            PythonExpression(
                [
                    "'realsense_params.yaml' if '",
                    camera,
                    "' == 'realsense' else 'cuvslam_params.yaml'",
                ]
            ),
        ]
    )

    # Start a Foxglove bridge alongside the node so every topic (odometry, the
    # /features/image overlay, /rgb/image, IMU, TF) is exposed to Foxglove
    # Studio at ws://<host>:8765 without a separate launch.
    start_bridge = LaunchConfiguration("foxglove_bridge")
    bridge_port = LaunchConfiguration("foxglove_port")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "camera",
                default_value="realsense",
                choices=["realsense", "oak"],
                description="Camera driving cuVSLAM: 'realsense' loads "
                "config/realsense_params.yaml (Intel D435i), 'oak' loads "
                "config/cuvslam_params.yaml (OAK-D S2). Run one at a time.",
            ),
            DeclareLaunchArgument(
                "foxglove_bridge",
                default_value="false",
                description="Start foxglove_bridge to expose topics to Foxglove Studio. "
                "Leave false if you already run a bridge (default port 8765 "
                "will otherwise fail with a Bind Error).",
            ),
            DeclareLaunchArgument(
                "foxglove_port",
                default_value="8765",
                description="WebSocket port for foxglove_bridge",
            ),
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
        ]
    )
