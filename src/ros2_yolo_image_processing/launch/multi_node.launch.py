#!/usr/bin/env python3

import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource


# Used to load parameters for composable nodes from a standard param file
def dump_params(param_file_path, node_name):
    with open(param_file_path, 'r') as file:
        return [yaml.safe_load(file)[node_name]['ros__parameters']]


def generate_launch_description():
    package_name = 'ros2_yolo_image_processing'

    # Add in any additional libraries
    #foxglove_bridge = get_package_share_directory("foxglove_bridge")
    config_dir = os.path.join(get_package_share_directory(package_name), 'config')

    # In case you need to namespace:
    # Arguments set from the command line or uses a default
    namespace_launch = DeclareLaunchArgument('namespace', default_value='/')
    min_score_threshold_launch = DeclareLaunchArgument('min_score_threshold', default_value='0.25')
    min_iou_threshold_launch = DeclareLaunchArgument('min_iou_threshold', default_value='0.45')

    # Arguments to swap in as parameters
    namespace = LaunchConfiguration('namespace')
    min_score_threshold = LaunchConfiguration('min_score_threshold')
    min_iou_threshold = LaunchConfiguration('min_iou_threshold')

    drogue_detection_node = Node(
        package=package_name,
        executable='drogue_detection_node',
        parameters=[{
            'min_score_threshold': min_score_threshold,
            'min_iou_threshold': min_iou_threshold,
        }],
        name='drogue_detection_node',
    )

    pose_estimation_node = Node(
        package=package_name,
        executable='pose_estimation_node',
        name='pose_estimation_node',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_node',
        output='screen'
    )


    # Visualize Data with Foxglove
    # Connect our Foxglove Bridge to hear all of our ROS 2 topics
    #foxglove = IncludeLaunchDescription(
    #    XMLLaunchDescriptionSource(
    #        os.path.join(foxglove_bridge, "launch", "foxglove_bridge_launch.xml"))
    #)

    # Create a composable node to have cheap, easy recording of data
    ros2bag = ComposableNodeContainer(
            name='ros2bag_recorder_container',
            package='rclcpp_components',
            executable='component_container',
            namespace=namespace,
            composable_node_descriptions=[
                ComposableNode(
                    package='rosbag2_transport',
                    plugin='rosbag2_transport::Recorder',
                    name='recorder',
                    parameters=dump_params(os.path.join(config_dir, 'recorder_params.yaml'), "recorder"),
                    extra_arguments=[{'use_intra_process_comms': True}]
                ),
            ]
    )

    # Add in all our separate commands into one general launch command
    return LaunchDescription([
        namespace_launch,
        min_score_threshold_launch,
        min_iou_threshold_launch,
        rviz,
        #ExecuteProcess(cmd=[['foxglove-studio']]),
        #foxglove,
        drogue_detection_node,
        pose_estimation_node,
        ros2bag
    ])
