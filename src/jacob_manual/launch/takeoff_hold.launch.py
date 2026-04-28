from datetime import datetime

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params = PathJoinSubstitution([
        FindPackageShare('jacob_manual'),
        'cfg',
        'takeoff_hold_params.yaml'
    ])
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('jacob_manual'),
        'cfg',
        'drone.rviz'
    ])

    bag_dir = 'rosbags/' + f"flight_bag_{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}"
    ros2bag_node = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '-a',
            '-s', 'mcap',
            '-o', bag_dir,
        ],
        output='screen'
    )

    return LaunchDescription([
        Node(
            package='jacob_manual',
            executable='takeoff_hold',
            name='takeoff_hold',
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
            package='jacob_manual',
            executable='visualizer',
            name='visualizer',
            output='screen',
        ),
        #ros2bag_node,
    ])
