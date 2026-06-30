from datetime import datetime

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    qos_overrides = PathJoinSubstitution([
        FindPackageShare('precision_land'), 'cfg', 'rosbag_qos_override.yaml'
    ])
    bag_dir = 'rosbags/' + f"flight_bag_{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}"
    ros2bag_node = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '-a',
            '-o', bag_dir,
            '--qos-profile-overrides-path', qos_overrides
        ],
        output='log'
    )

    return LaunchDescription([
        Node(
            package='precision_land',
            executable='precision_land',
            name='precision_land',
            output='screen',
            parameters=[
                PathJoinSubstitution([FindPackageShare('precision_land'), 'cfg', 'params.yaml'])
            ]
        ),
        ros2bag_node,
    ])
