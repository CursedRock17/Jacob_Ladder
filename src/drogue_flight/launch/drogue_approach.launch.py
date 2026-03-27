from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params = PathJoinSubstitution([
        FindPackageShare('voxl_offboard_ship_land'),
        'cfg',
        'drogue_approach_params.yaml'
    ])

    return LaunchDescription([
        Node(
            package='voxl_offboard_ship_land',
            executable='drogue_approach',
            name='drogue_approach',
            output='screen',
            parameters=[params]
        )
    ])
