import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

from launch_ros.actions import Node


# General Call to Create the Launch File
def generate_launch_description():
    # Add in all separate packages
    voxl_mpa_ros2_bridge = get_package_share_directory("voxl_mpa_to_ros2")

    # Include the Voxl MPA To ROS 2 connection launch file.
    voxl_mpa_to_ros2 = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(voxl_mpa_ros2_bridge, "launch", "voxl_map_to_ros2.launch"))
    )

    # ROS 2 Node that runs the autonomous landing code
    voxl_ship_land_agro = Node(
        package='voxl_offboard_ship_land',
        executable='voxl_offboard_ship_land_agro',
        name="voxl_ship_land_auto",
        output='screen',
    )

    # ROS 2 Node that runs the ranging algorithm to the drogue/coupler
    drogue_ranging = Node(
        package='voxl_offboard_ship_land',
        executable='drogue_range_ros_node',
        name="drogue_ranging",
        output='screen',
    )

    # Add in all our separate commands into one general launch command
    return LaunchDescription([
        voxl_mpa_to_ros2,
        voxl_ship_land_agro,
        drogue_ranging
    ])
