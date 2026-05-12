"""
Generic simulation launch file for jacob_manual modes.

Usage:
    ros2 launch jacob_manual sim.launch.py executable:=<name> params_file:=<yaml>

Examples:
    ros2 launch jacob_manual sim.launch.py executable:=takeoff_land params_file:=takeoff_land_params.yaml
    ros2 launch jacob_manual sim.launch.py executable:=precision_land params_file:=params.yaml
    ros2 launch jacob_manual sim.launch.py executable:=front_approach params_file:=front_approach_params.yaml
    ros2 launch jacob_manual sim.launch.py executable:=precision_land_auto params_file:=params.yaml
"""
from datetime import datetime

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    executable_arg = DeclareLaunchArgument(
        'executable',
        description='jacob_manual executable to launch (e.g. takeoff_land, precision_land)'
    )
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        description='Params YAML filename inside jacob_manual/cfg/ (e.g. params.yaml)'
    )

    executable = LaunchConfiguration('executable')
    params = PathJoinSubstitution([
        FindPackageShare('jacob_manual'), 'cfg', LaunchConfiguration('params_file')
    ])
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('jacob_manual'), 'cfg', 'drone.rviz'
    ])

    bag_dir = 'rosbags/' + f"flight_bag_{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}"
    ros2bag_node = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '-a',
            '--exclude', '/.*image_raw|/.*compressed|/.*theora|/.*h264|/.*depth|/.*color/image',
            '-s', 'mcap',
            '-o', bag_dir,
        ],
        output='screen'
    )

    return LaunchDescription([
        executable_arg,
        params_file_arg,
        #ros2bag_node,
        Node(
            package='jacob_manual',
            executable=executable,
            name=executable,
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
    ])
