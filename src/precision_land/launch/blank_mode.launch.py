from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='precision_land',
            executable='blank_mode',
            name='blank_mode',
            output='screen',
        )
    ])
