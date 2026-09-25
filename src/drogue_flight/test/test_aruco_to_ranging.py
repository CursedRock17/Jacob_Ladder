"""The ArUco -> ranging-frame flip that lets DroneSmoothPlanner chase a tag in SITL."""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "drogue_flight"))

from aruco_to_ranging import optical_to_ranging  # noqa: E402


def test_tag_up_and_to_the_right_lands_in_ranging_frame():
    # OpenCV optical: +x right, +y down, +z forward.  Tag 0.5 m right, 0.2 m up, 3 m ahead.
    # Ranging (what the YOLO pose node publishes): +x left, +y up, +z forward.
    assert optical_to_ranging(0.5, -0.2, 3.0) == (-0.5, 0.2, 3.0)


def test_straight_ahead_is_unchanged():
    assert optical_to_ranging(0.0, 0.0, 2.0) == (0.0, 0.0, 2.0)
