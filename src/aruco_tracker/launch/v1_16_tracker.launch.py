# use this launch file when using px4 v1.16 and remember to update submodules to latest main
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            # Bridge Gazebo camera image to ROS2 topic /camera
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "run",
                    "ros_gz_bridge",
                    "parameter_bridge",
                    "/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/image"
                    + "@sensor_msgs/msg/Image@gz.msgs.Image",
                    "--ros-args",
                    "--remap",
                    "/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/image:=/camera",
                ],
                name="image_bridge_process",
                output="screen",
            ),
            # Bridge Gazebo camera info to ROS2 topic /image_proc (remapped)
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "run",
                    "ros_gz_bridge",
                    "parameter_bridge",
                    "/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/camera_info"
                    + "@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
                    "--ros-args",
                    "--remap",
                    "/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/camera_info:=/camera_info",
                ],
                name="camera_info_bridge_process",
                output="screen",
            ),
            # Aruco tracker node
            Node(
                package="aruco_tracker",
                executable="aruco_tracker",
                name="aruco_tracker",
                output="screen",
                parameters=[
                    PathJoinSubstitution(
                        [FindPackageShare("aruco_tracker"), "cfg", "params.yaml"]
                    )
                ],
            ),
        ]
    )
