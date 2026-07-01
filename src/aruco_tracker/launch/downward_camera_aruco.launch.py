from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


WORLD_PREFIX = "/world/aruco_dual_ids/model/x500_dual_cam_0"


def _bridge(source_topic: str, target_topic: str, msg_type: str) -> ExecuteProcess:
    return ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "ros_gz_bridge",
            "parameter_bridge",
            f"{source_topic}@{msg_type}",
            "--ros-args",
            "--remap",
            f"{source_topic}:={target_topic}",
        ],
        output="screen",
    )


def generate_launch_description():
    down_image = f"{WORLD_PREFIX}/link/down_camera_link/sensor/down_camera/image"
    down_info = f"{WORLD_PREFIX}/link/down_camera_link/sensor/down_camera/camera_info"

    bridges = [
        _bridge(
            down_image,
            "/downward/camera/image_raw",
            "sensor_msgs/msg/Image@gz.msgs.Image",
        ),
        _bridge(
            down_info,
            "/downward/camera/camera_info",
            "sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
        ),
    ]

    tracker = Node(
        package="aruco_tracker",
        executable="aruco_tracker",
        name="downward_aruco_tracker",
        output="screen",
        parameters=[
            {
                "aruco_id": 0,
                "dictionary": 2,
                "marker_size": 0.5,
            }
        ],
        remappings=[
            ("/camera", "/downward/camera/image_raw"),
            ("/camera_info", "/downward/camera/camera_info"),
            ("/target_pose", "/target_pose"),
            ("/image_proc", "/downward/image_proc"),
        ],
    )

    return LaunchDescription(bridges + [tracker])
