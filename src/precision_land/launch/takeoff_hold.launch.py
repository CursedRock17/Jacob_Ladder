import os
import yaml
from datetime import datetime

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


# Used to load parameters for composable nodes from a standard param file
def dump_params(param_file_path, node_name):
    with open(param_file_path, 'r') as file:
        return [yaml.safe_load(file)[node_name]['ros__parameters']]


def generate_launch_description():
    params = PathJoinSubstitution([
        FindPackageShare('precision_land'),
        'cfg',
        'takeoff_hold_params.yaml'
    ])
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('precision_land'),
        'cfg',
        'drone.rviz'
    ])

    # Create a composable node to have cheap, easy recording of data
    #ros2bag_node = ComposableNodeContainer(
    #    name='ros2bag_recorder_container',
    #    namespace='',
    #    package='rclcpp_components',
    #    executable='component_container_mt',
    #    output='screen',
    #    emulate_tty=True,
    #    composable_node_descriptions=[
    #        ComposableNode(
    #            package='rosbag2_transport',
    #            plugin='rosbag2_transport::Recorder',
    #            name='recorder',
    #            parameters=[
    #                os.path.join(
    #                    '/home/cursedrock17/Documents/Electrical/Matrix_Lab/jacob_drone_ws/src/Jacob_Ladder/src/precision_land/cfg/',
    #                    'recorder_params.yaml'
    #                )
    #            ],
    #            extra_arguments=[{'use_intra_process_comms': True}],
    #        ),
    #    ]
    #)

    bag_dir = 'rosbags/' + f"flight_bag_{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}"
    ros2bag_node = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '-a',
            '-o', bag_dir,
        ],
        output='screen'
    )

    return LaunchDescription([
        Node(
            package='precision_land',
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
            package='precision_land',
            executable='visualizer',
            name='visualizer',
            output='screen',
        ),
        ros2bag_node,
    ])
