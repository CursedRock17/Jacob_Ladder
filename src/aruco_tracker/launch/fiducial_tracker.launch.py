"""Track one ArUco or AprilTag marker on one camera, publishing its pose on pose_topic.

    ros2 launch aruco_tracker fiducial_tracker.launch.py tag_id:=0 marker_size:=0.30 \\
        dictionary:=20 image_topic:=/camera/color/image_raw \\
        camera_info_topic:=/camera/color/camera_info \\
        pose_topic:=/jl/perception/front/apriltag_36h11_0

jl_flight's perception declarations (AprilTag, ArucoTag) start this with the
right arguments, so a mission never has to. dictionary is OpenCV's
cv::aruco::PredefinedDictionaryType (2 = DICT_4X4_250, 20 = DICT_APRILTAG_36h11).
marker_size is the side of the black square in metres; it scales the pose.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    arg = LaunchConfiguration
    return LaunchDescription(
        [
            DeclareLaunchArgument("tag_id", default_value="0"),
            DeclareLaunchArgument("marker_size"),
            DeclareLaunchArgument("dictionary"),
            DeclareLaunchArgument("image_topic"),
            DeclareLaunchArgument("camera_info_topic"),
            DeclareLaunchArgument("pose_topic"),
            DeclareLaunchArgument("node_name", default_value="fiducial_tracker"),
            Node(
                package="aruco_tracker",
                executable="aruco_tracker",
                name=arg("node_name"),
                output="screen",
                parameters=[
                    {
                        "aruco_id": ParameterValue(arg("tag_id"), value_type=int),
                        "dictionary": ParameterValue(arg("dictionary"), value_type=int),
                        "marker_size": ParameterValue(
                            arg("marker_size"), value_type=float
                        ),
                    }
                ],
                remappings=[
                    ("/camera", arg("image_topic")),
                    ("/camera_info", arg("camera_info_topic")),
                    ("/target_pose", arg("pose_topic")),
                    ("/image_proc", [arg("pose_topic"), "/image"]),
                ],
            ),
        ]
    )
