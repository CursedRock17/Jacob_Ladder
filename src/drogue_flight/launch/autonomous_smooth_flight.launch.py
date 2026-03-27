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

from launch_ros.actions import Node


# Used to load parameters for composable nodes from a standard param file
def dump_params(param_file_path, node_name):
    with open(param_file_path, 'r') as file:
        return [yaml.safe_load(file)[node_name]['ros__parameters']]


# General Call to Create the Launch File
def generate_launch_description():
    # Add in all separate packages
    voxl_mpa_ros2_bridge = get_package_share_directory("voxl_mpa_to_ros2")
    foxglove_bridge = get_package_share_directory("foxglove_bridge")

    # In case you need to namespace:
    # Arguments set from the command line or uses a default
    namespace_launch = DeclareLaunchArgument('namespace', default_value='/'),
    num_waypoints_launch = DeclareLaunchArgument('num_waypoints', default_value='10')
    setpoint_rate_hz_launch = DeclareLaunchArgument('setpoint_rate_hz', default_value='20',)
    s_curve_steepness_launch = DeclareLaunchArgument('s_curve_steepness', default_value='4.0')
    waypoint_tolerance_m_launch = DeclareLaunchArgument('waypoint_tolerance_m', default_value='0.15')
    max_velocity_launch = DeclareLaunchArgument('max_velocity', default_value='0.50')
    max_acceleration_launch = DeclareLaunchArgument('max_acceleration', default_value='0.35')

    # Each of the parameters to pass
    num_waypoints = LaunchConfiguration('num_waypoints'),
    setpoint_rate_hz = LaunchConfiguration('setpoint_rate_hz'),
    s_curve_steepness = LaunchConfiguration('s_curve_steepness'),
    waypoint_tolerance_m = LaunchConfiguration('waypoint_tolerance_m'),
    namespace = LaunchConfiguration('namespace'),
    max_velocity = LaunchConfiguration('max_velocity'),
    max_acceleration = LaunchConfiguration('max_acceleration'),

    # Include the Voxl MPA To ROS 2 connection launch file.
    voxl_mpa_to_ros2 = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(voxl_mpa_ros2_bridge, "launch", "voxl_map_to_ros2.launch"))
    )

    # ROS 2 Node that runs the autonomous landing code
    drone_smooth_planner_node = Node(
        package='voxl_offboard_ship_land',
        executable='drone_smooth_planner',
        parameters=[{
            'num_waypoints': num_waypoints,
            'setpoint_rate_hz': setpoint_rate_hz,
            's_curve_steepness': s_curve_steepness,
            'waypoint_tolerance_m': waypoint_tolerance_m,
            'namespace': namespace,
            'max_velocity': max_velocity,
            'max_acceleration': max_acceleration,
        }],
        name="drone_smooth_planner",
        output='screen',
    )

    # ROS 2 Node that runs the ranging algorithm to the drogue/coupler
    drogue_ranging_node = Node(
        package='voxl_offboard_ship_land',
        executable='drogue_range_ros_node',
        name="drogue_ranging",
        output='screen',
    )

    # Visualize Data with Foxglove
    # Connect our Foxglove Bridge to hear all of our ROS 2 topics
    foxglove = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(foxglove_bridge, "launch", "foxglove_bridge_launch.xml"))
    )

    # Create a composable node to have cheap, easy recording of data
    ros2bag_node = ComposableNodeContainer(
            name='ros2bag_recorder_container',
            package='rclcpp_components',
            executable='component_container',
            namespace=namespace,
            composable_node_descriptions=[
                ComposableNode(
                    package='rosbag2_transport',
                    plugin='rosbag2_transport::Recorder',
                    name='recorder',
                    parameters=dump_params(os.path.join('./voxl_offboard_ship_land/config/', 'recorder_params.yaml'), "recorder"),
                    extra_arguments=[{'use_intra_process_comms': True}]
                ),
            ]
    )

    # Add in all our separate commands into one general launch command
    return LaunchDescription([
        namespace_launch,
        num_waypoints_launch,
        setpoint_rate_hz_launch,
        s_curve_steepness_launch,
        waypoint_tolerance_m_launch,
        max_velocity_launch,
        max_acceleration_launch,
        ExecuteProcess(cmd=[['foxglove-studio']]),
        foxglove,
        voxl_mpa_to_ros2,
        drone_smooth_planner_node,
        drogue_ranging_node,
        ros2bag_node,
    ])
