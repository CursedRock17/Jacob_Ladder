"""Launch the example autonomous mode.

Deliberately minimal: it starts the mode node and nothing else. The modes in
`jacob_manual` also bring up RViz, a visualizer node, and a rosbag recorder,
but those are separate concerns -- add them once your own mode needs them.

PX4 and the DDS agent must already be running; see the repository README.
"""

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params = PathJoinSubstitution(
        [
            FindPackageShare("example_autonomous_mode"),
            "cfg",
            "example_autonomous_mode_params.yaml",
        ]
    )

    return LaunchDescription(
        [
            Node(
                package="example_autonomous_mode",
                executable="example_autonomous_mode",
                # Must match the top-level key in the params YAML
                name="example_autonomous_mode",
                output="screen",
                parameters=[params],
            ),
        ]
    )
