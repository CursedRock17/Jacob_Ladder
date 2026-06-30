import os
from datetime import datetime

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

from launch_ros.actions import Node


# General Call to Create the Launch File
def generate_launch_description():
    # Add in all separate packages
    mpa_ros2_bridge = get_package_share_directory("voxl_mpa_to_ros2")

    # Include the MPA To ROS 2 connection launch file.
    mpa_to_ros2 = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(mpa_ros2_bridge, "launch", "voxl_map_to_ros2.launch"))
    )

    # ROS 2 Node that runs the autonomous landing code
    drogue_flight_agro = Node(
        package='drogue_flight',
        executable='drogue_flight_agro',
        name="drogue_flight_auto",
        output='screen',
    )

    # ROS 2 Node that runs the ranging algorithm to the drogue/coupler
    drogue_ranging = Node(
        package='drogue_flight',
        executable='drogue_range_ros_node',
        name="drogue_ranging",
        output='screen',
    )

    drogue_share = get_package_share_directory("drogue_flight")
    qos_overrides = os.path.join(drogue_share, 'config', 'rosbag_qos_override.yaml')
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

    # Add in all our separate commands into one general launch command
    return LaunchDescription([
        #mpa_to_ros2,
        #drogue_flight_agro,
        drogue_ranging,
        #ros2bag_node,
    ])
