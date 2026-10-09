"""Perception declarations: what they start, where they publish, what they reject."""

from __future__ import annotations

import pytest

from jl_flight import AprilTag, ArucoTag, YoloDrogue
from jl_flight.airframes import X500_RGBD


def args(helper) -> dict[str, str]:
    return dict(a.split(":=", 1) for a in helper.argv if ":=" in a)


def test_an_apriltag_starts_a_tracker_on_its_camera() -> None:
    tag = AprilTag(id=3, size=0.16, family="25h9")
    (helper,) = tag.helpers(X500_RGBD)
    a = args(helper)
    assert helper.argv[:4] == (
        "ros2",
        "launch",
        "aruco_tracker",
        "fiducial_tracker.launch.py",
    )
    assert a["tag_id"] == "3" and a["marker_size"] == "0.16" and a["dictionary"] == "18"
    assert a["image_topic"] == "/camera/color/image_raw"
    assert a["pose_topic"] == tag.topic == "/jl/perception/front/apriltag_25h9_3"


def test_aruco_dictionaries_map_to_opencv() -> None:
    assert ArucoTag(id=0, size=0.1, dictionary_name="4x4_250").dictionary == 2
    assert ArucoTag(id=0, size=0.1, dictionary_name="6x6_1000").dictionary == 11
    assert ArucoTag(id=0, size=0.1, dictionary_name="original").dictionary == 16


def test_the_drogue_starts_the_yolo_pipeline_wired_together() -> None:
    drogue = YoloDrogue(min_score=0.4)
    detector, ranging = drogue.helpers(X500_RGBD)
    d, r = " ".join(detector.argv), " ".join(ranging.argv)
    boxes = f"{drogue.topic}/detections"
    assert "image_raw:=/camera/color/image_raw" in d and f"detections:={boxes}" in d
    assert "min_score_thresh:=0.4" in d  # the name the node reads
    assert f"detections:={boxes}" in r and f"tag_detections:={drogue.topic}" in r
    assert drogue.frame == "ranging"


def test_different_tags_do_not_share_a_topic() -> None:
    topics = {
        AprilTag(id=0, size=0.3).topic,
        AprilTag(id=1, size=0.3).topic,
        ArucoTag(id=0, size=0.3).topic,
        YoloDrogue().topic,
    }
    assert len(topics) == 4


@pytest.mark.parametrize(
    ("make", "match"),
    [
        (lambda: AprilTag(id=-1, size=0.3), "id must not be negative"),
        (lambda: AprilTag(id=0, size=0), "size must be > 0"),
        (lambda: AprilTag(id=0, size=0.3, family="99h9"), "unknown AprilTag family"),
        (
            lambda: ArucoTag(id=0, size=0.3, dictionary_name="3x3"),
            "unknown ArUco dictionary",
        ),
        (lambda: YoloDrogue(min_score=1.5), "between 0 and 1"),
        (lambda: AprilTag(id=0, size=0.3, max_age=0), "max_age"),
    ],
)
def test_bad_declarations_are_rejected(make, match: str) -> None:
    with pytest.raises(ValueError, match=match):
        make()


def test_declarations_are_values() -> None:
    assert AprilTag(id=0, size=0.3) == AprilTag(id=0, size=0.3)
    assert len({AprilTag(id=0, size=0.3), AprilTag(id=0, size=0.3)}) == 1
