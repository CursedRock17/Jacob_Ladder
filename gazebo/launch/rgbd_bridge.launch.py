"""Bridge the simulated RGB-D camera (x500_rgbd) into ROS 2.

    ros2 launch gazebo/launch/rgbd_bridge.launch.py

The topics use the names of the real realsense2_camera driver, so code written
against the simulator runs on the drone's camera unchanged:

    /camera/color/image_raw                      sensor_msgs/Image        rgb8
    /camera/color/camera_info                    sensor_msgs/CameraInfo
    /camera/aligned_depth_to_color/image_raw     sensor_msgs/Image        32FC1, metres
    /camera/depth/color/points                   sensor_msgs/PointCloud2

Two differences from the real driver: the depth image here is 32-bit float in
metres (the driver publishes 16UC1 millimetres), and its intrinsics are the
colour camera's, because Gazebo renders both from one viewpoint.
"""

from launch import LaunchDescription
from launch_ros.actions import Node

# Gazebo topic -> (ROS type, ROS topic)
TOPICS = {
    "/realsense/image": ("sensor_msgs/msg/Image", "/camera/color/image_raw"),
    "/realsense/camera_info": (
        "sensor_msgs/msg/CameraInfo",
        "/camera/color/camera_info",
    ),
    "/realsense/depth_image": (
        "sensor_msgs/msg/Image",
        "/camera/aligned_depth_to_color/image_raw",
    ),
    "/realsense/points": (
        "sensor_msgs/msg/PointCloud2",
        "/camera/depth/color/points",
    ),
}
GZ_TYPES = {
    "sensor_msgs/msg/Image": "gz.msgs.Image",
    "sensor_msgs/msg/CameraInfo": "gz.msgs.CameraInfo",
    "sensor_msgs/msg/PointCloud2": "gz.msgs.PointCloudPacked",
}


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="rgbd_bridge",
                output="screen",
                # `[` = one way, Gazebo -> ROS: nothing is ever sent back to the sim.
                arguments=[
                    f"{gz}@{ros_type}[{GZ_TYPES[ros_type]}"
                    for gz, (ros_type, _) in TOPICS.items()
                ],
                remappings=[(gz, ros) for gz, (_, ros) in TOPICS.items()],
            )
        ]
    )
