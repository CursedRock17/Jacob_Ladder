import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'ros2_yolo_image_processing'

    # Add in any additional libraries
    # foxglove_bridge = get_package_share_directory("foxglove_bridge")

    # In case you need to namespace:
    # Arguments set from the command line or uses a default
    DeclareLaunchArgument('namespace', default_value='/'),
    DeclareLaunchArgument('min_score_threshold', default_value='0.25'),
    DeclareLaunchArgument('min_iou_threshold', default_value='0.45'),

    drogue_detection_node = Node(
        package=package_name,
        executable='drogue_detection_node',
        parameters=[{
            'min_score_thresh': LaunchConfiguration('min_score_threshold'),
            'iou_thresh': LaunchConfiguration('min_iou_threshold')
        }],
        name='drogue_detection_node',
    )

    pose_estimation_node = Node(
        package=package_name,
        executable='pose_estimation_node',
        name='pose_estimation_node',
    )

    # Visualize Data with Foxglove
    # Connect our Foxglove Bridge to hear all of our ROS 2 topics
    foxglove = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(foxglove_bridge, "launch", "foxglove_bridge_launch.xml"))
    )

    # Add in all our separate commands into one general launch command
    return LaunchDescription([
        ExecuteProcess(cmd=[['foxglove-studio']]),
        foxglove,
        drogue_detection_node,
        pose_estimation_node,
    ])
