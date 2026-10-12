"""Launch the example autonomous mode and executor workflow.

Deliberately minimal: it starts the combined executor/mode node and nothing
else. The executor is ready while disarmed; selecting the registered mode starts
its arm and takeoff sequence. The modes in `precision_land` also bring up RViz,
a visualizer node, and a rosbag recorder. Add those once your mode needs them.

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
