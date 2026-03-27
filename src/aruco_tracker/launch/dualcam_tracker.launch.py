import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource


WORLD_PREFIX = '/world/aruco_dual_ids/model/x500_dual_cam_0'


def _bridge(source_topic: str, target_topic: str, msg_type: str) -> ExecuteProcess:
    return ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
            f'{source_topic}@{msg_type}',
            '--ros-args', '--remap', f'{source_topic}:={target_topic}'
        ],
        output='screen'
    )


# Used to load parameters for composable nodes from a standard param file
def dump_params(param_file_path, node_name):
    with open(param_file_path, 'r') as file:
        return [yaml.safe_load(file)[node_name]['ros__parameters']]


def generate_launch_description():
    # Add in any additional libraries
    # foxglove_bridge = get_package_share_directory("foxglove_bridge")

    # Front Image Topics
    front_image = f'{WORLD_PREFIX}/link/front_camera_link/sensor/front_camera/image'
    front_info = f'{WORLD_PREFIX}/link/front_camera_link/sensor/front_camera/camera_info'

    # Down Image Topics
    down_image = f'{WORLD_PREFIX}/link/down_camera_link/sensor/down_camera/image'
    down_info = f'{WORLD_PREFIX}/link/down_camera_link/sensor/down_camera/camera_info'

    # ROS2 -> Gazebo Remapping
    bridges = [
        _bridge(front_image, '/front/camera/image_raw', 'sensor_msgs/msg/Image@gz.msgs.Image'),
        _bridge(front_info, '/front/camera/camera_info', 'sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'),
        _bridge(down_image, '/downward/camera/image_raw', 'sensor_msgs/msg/Image@gz.msgs.Image'),
        _bridge(down_info, '/downward/camera/camera_info', 'sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'),
    ]

    # Create the Front Aruco Tracker Node
    front_tracker_node = Node(
        package='aruco_tracker',
        executable='aruco_tracker',
        name='front_aruco_tracker',
        output='screen',
        parameters=[{
            'aruco_id': 1,
            'dictionary': 2,
            'marker_size': 0.5,
        }],
        remappings=[
            ('/camera', '/front/camera/image_raw'),
            ('/camera_info', '/front/camera/camera_info'),
            ('/target_pose', '/front/target_pose'),
            ('/image_proc', '/front/image_proc'),
        ]
    )

    # Create the Downward Aruco Tracker Node
    down_tracker_node = Node(
        package='aruco_tracker',
        executable='aruco_tracker',
        name='downward_aruco_tracker',
        output='screen',
        parameters=[{
            'aruco_id': 0,
            'dictionary': 2,
            'marker_size': 0.5,
        }],
        remappings=[
            ('/camera', '/downward/camera/image_raw'),
            ('/camera_info', '/downward/camera/camera_info'),
            ('/target_pose', '/target_pose'),
            ('/image_proc', '/downward/image_proc'),
        ]
    )

    # Visualize Data with Foxglove
    # Connect our Foxglove Bridge to hear all of our ROS 2 topics
    # foxglove = IncludeLaunchDescription(
        # XMLLaunchDescriptionSource(
         #   os.path.join(foxglove_bridge, "launch", "foxglove_bridge_launch.xml"))
    #)

    # Create a composable node to have cheap, easy recording of data
    ros2bag = ComposableNodeContainer(
            name='ros2bag_recorder_container',
            package='rclcpp_components',
            executable='component_container',
            namespace="",
            composable_node_descriptions=[
                ComposableNode(
                    package='rosbag2_transport',
                    plugin='rosbag2_transport::Recorder',
                    name='recorder',
                    parameters=dump_params(os.path.join('./src/aruco_tracker/cfg/', 'recorder_params.yaml'), "recorder"),
                    extra_arguments=[{'use_intra_process_comms': True}]
                ),
            ]
    )

    return LaunchDescription(bridges + [
        # ExecuteProcess(cmd=[['foxglove-studio']]),
        # foxglove,
        ros2bag,
        front_tracker_node,
        down_tracker_node,
    ])
