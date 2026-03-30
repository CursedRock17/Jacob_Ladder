from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params = PathJoinSubstitution([
        FindPackageShare('precision_land'),
        'cfg',
        'front_to_precision_params.yaml'
    ])
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('precision_land'),
        'cfg',
        'drone.rviz'
    ])

    return LaunchDescription([
        Node(
            package='precision_land',
            executable='front_to_precision_land',
            name='front_to_precision_land',
            output='screen',
            parameters=[params]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz_node',
            output='screen',
            arguments=['-d', rviz_config_file]
        ),
        Node(
            package='precision_land',
            executable='visualizer',
            name='visualizer',
            output='screen',
        ),
    ])
