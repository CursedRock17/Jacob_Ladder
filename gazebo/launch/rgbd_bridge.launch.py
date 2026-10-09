"""Bridge the simulated RGB-D camera (x500_rgbd) into ROS 2.

    ros2 launch gazebo/launch/rgbd_bridge.launch.py [depth:=false] [points:=true]

The topics use the names of the real realsense2_camera driver, so code written
against the simulator runs on the drone's camera unchanged:

    /camera/color/image_raw                      sensor_msgs/Image        rgb8
    /camera/color/camera_info                    sensor_msgs/CameraInfo
    /camera/aligned_depth_to_color/image_raw     sensor_msgs/Image        32FC1, metres
    /camera/depth/color/points                   sensor_msgs/PointCloud2  (points:=true only)

The point cloud is off by default. It is about 5 MB a frame at 30 Hz, and bridging it
in the same process as the images stalled them by about a second, which made anything
steering by the camera (like a tag tracker) act on stale readings. Each stream has its own
bridge process, so one cannot stall another, but the cloud is still expensive:
only ask for it if you use it.

Two differences from the real driver: the depth image here is 32-bit float in
metres (the driver publishes 16UC1 millimetres), and its intrinsics are the
colour camera's, because Gazebo renders both from one viewpoint.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

IMAGE, INFO, DEPTH, POINTS = (
    "sensor_msgs/msg/Image",
    "sensor_msgs/msg/CameraInfo",
    "sensor_msgs/msg/Image",
    "sensor_msgs/msg/PointCloud2",
)
GZ_TYPES = {
    IMAGE: "gz.msgs.Image",
    INFO: "gz.msgs.CameraInfo",
    POINTS: "gz.msgs.PointCloudPacked",
}

# One bridge process per stream, so a heavy one cannot stall a light one.
# name -> [(Gazebo topic, ROS type, ROS topic)]
COLOUR = [
    ("/realsense/image", IMAGE, "/camera/color/image_raw"),
    ("/realsense/camera_info", INFO, "/camera/color/camera_info"),
]
DEPTH_STREAM = [
    ("/realsense/depth_image", DEPTH, "/camera/aligned_depth_to_color/image_raw"),
]
POINTS_STREAM = [("/realsense/points", POINTS, "/camera/depth/color/points")]


def _bridge(name: str, streams, condition=None) -> Node:
    return Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name=name,
        output="screen",
        # `[` = one way, Gazebo -> ROS: nothing is ever sent back to the sim.
        arguments=[f"{gz}@{ros}[{GZ_TYPES[ros]}" for gz, ros, _ in streams],
        remappings=[(gz, target) for gz, _, target in streams],
        condition=condition,
    )


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("depth", default_value="true"),
            DeclareLaunchArgument("points", default_value="false"),
            _bridge("rgbd_colour_bridge", COLOUR),
            _bridge(
                "rgbd_depth_bridge",
                DEPTH_STREAM,
                IfCondition(LaunchConfiguration("depth")),
            ),
            _bridge(
                "rgbd_points_bridge",
                POINTS_STREAM,
                IfCondition(LaunchConfiguration("points")),
            ),
        ]
    )
