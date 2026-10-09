"""Bridge the x500_dual_cam drone's two cameras into ROS 2.

    ros2 launch gazebo/launch/dualcam_bridge.launch.py world:=aruco_dual_ids

    /front/camera/image_raw, /front/camera/camera_info          the front camera
    /downward/camera/image_raw, /downward/camera/camera_info    the downward camera

Only images: the trackers a mission declares read these (see jl_flight's
x500_dual_cam airframe). One bridge process per camera, so one cannot stall the other.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

CAMERAS = {"front": "front_camera", "downward": "down_camera"}


def _bridges(context):
    world = LaunchConfiguration("world").perform(context)
    model = LaunchConfiguration("model").perform(context)
    nodes = []
    for ros_name, sensor in CAMERAS.items():
        prefix = f"/world/{world}/model/{model}/link/{sensor}_link/sensor/{sensor}"
        image, info = f"{prefix}/image", f"{prefix}/camera_info"
        nodes.append(
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name=f"{ros_name}_camera_bridge",
                output="screen",
                # `[` = one way, Gazebo -> ROS.
                arguments=[
                    f"{image}@sensor_msgs/msg/Image[gz.msgs.Image",
                    f"{info}@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
                ],
                remappings=[
                    (image, f"/{ros_name}/camera/image_raw"),
                    (info, f"/{ros_name}/camera/camera_info"),
                ],
            )
        )
    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("world", default_value="aruco_dual_ids"),
            DeclareLaunchArgument("model", default_value="x500_dual_cam_0"),
            OpaqueFunction(function=_bridges),
        ]
    )
