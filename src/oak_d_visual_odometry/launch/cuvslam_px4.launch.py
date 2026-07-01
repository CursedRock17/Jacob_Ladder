from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = PathJoinSubstitution([
        FindPackageShare("oak_d_visual_odometry"),
        "config",
        "cuvslam_params.yaml",
    ])

    return LaunchDescription([
        Node(
            package="oak_d_visual_odometry",
            executable="cuvslam_publisher_px4_node",
            name="oak_d_cuvslam_px4_publisher",
            output="screen",
            parameters=[params_file],
        ),
    ])
