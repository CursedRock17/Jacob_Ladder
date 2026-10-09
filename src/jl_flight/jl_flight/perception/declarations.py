"""Things a mission can see, declared up front.

    TAG = AprilTag(id=0, size=0.30)

    @mission("LandBeforeAprilTag", needs=[TAG])
    def fly(drone):
        drone.approach(TAG, standoff=1.5)

A declaration says everything needed to start perception (which detector, its
settings, which camera) and to read it (the topic, the frame its detections are
in). The camera's topics and mount come from the airframe. The sim, the tests
and deploy all start the helper processes it describes, so a mission can only
use what it declared, and nothing is configured twice.

Declarations are interchangeable wherever a target is wanted: a mission swaps
an AprilTag for the YOLO drogue by changing the declaration, nothing else.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, ClassVar

from .targets import OpticalTarget

if TYPE_CHECKING:  # airframes imports this package, so only for type hints
    from ..airframes import Airframe

# cv::aruco::PredefinedDictionaryType values.
APRILTAG_FAMILIES = {"16h5": 17, "25h9": 18, "30h10": 19, "36h11": 20}
ARUCO_DICTIONARIES = {
    f"{n}x{n}_{count}": 4 * i + j
    for i, n in enumerate((4, 5, 6, 7))
    for j, count in enumerate((50, 100, 250, 1000))
} | {"original": 16}


@dataclass(frozen=True)
class Helper:
    """A process that has to run alongside the mission."""

    name: str
    argv: tuple[str, ...]


@dataclass(frozen=True)
class Perception:
    """Base for declarations: something seen by one of the airframe's cameras,
    published as geometry_msgs/PoseStamped detections on `topic`."""

    camera: str = "front"
    max_age: float = 0.5  # s a detection counts as "seen"

    # The frame detections arrive in: "optical" (OpenCV) or "ranging" (YOLO pose node).
    frame: ClassVar[str] = "optical"
    # The ROS packages its helpers run from, so deploy builds them.
    packages: ClassVar[tuple[str, ...]] = ()

    def __post_init__(self) -> None:
        if self.max_age <= 0:
            raise ValueError("max_age must be > 0")

    @property
    def key(self) -> str:
        """A short name, unique among declarations, used in topic and process names."""
        raise NotImplementedError

    @property
    def topic(self) -> str:
        return f"/jl/perception/{self.camera}/{self.key}"

    def helpers(self, airframe: Airframe) -> list[Helper]:
        """The processes that produce `topic`."""
        return []

    def validate(self, airframe: Airframe) -> None:
        """Raise KeyError if the airframe cannot provide this."""
        airframe.camera(self.camera)

    def make_target(self, airframe: Airframe) -> OpticalTarget:
        return OpticalTarget(
            airframe.camera(self.camera).mount, self.max_age, self.frame
        )


@dataclass(frozen=True, kw_only=True)
class _Fiducial(Perception):
    id: int
    size: float  # m: the side of the tag's black square

    kind: ClassVar[str] = "fiducial"
    packages: ClassVar[tuple[str, ...]] = ("aruco_tracker",)

    def __post_init__(self) -> None:
        super().__post_init__()
        if self.id < 0:
            raise ValueError("id must not be negative")
        if self.size <= 0:
            raise ValueError("size must be > 0 (metres: the side of the black square)")

    @property
    def label(self) -> str:
        """The dictionary's short name, e.g. 36h11."""
        raise NotImplementedError

    @property
    def dictionary(self) -> int:
        raise NotImplementedError

    @property
    def key(self) -> str:
        return f"{self.kind}_{self.label}_{self.id}"

    def helpers(self, airframe: Airframe) -> list[Helper]:
        camera = airframe.camera(self.camera)
        name = f"{self.key}_{self.camera}"
        return [
            Helper(
                name,
                (
                    "ros2",
                    "launch",
                    "aruco_tracker",
                    "fiducial_tracker.launch.py",
                    f"tag_id:={self.id}",
                    f"marker_size:={self.size}",
                    f"dictionary:={self.dictionary}",
                    f"image_topic:={camera.image_topic}",
                    f"camera_info_topic:={camera.info_topic}",
                    f"pose_topic:={self.topic}",
                    f"node_name:={name}",
                ),
            )
        ]


@dataclass(frozen=True, kw_only=True)
class AprilTag(_Fiducial):
    family: str = "36h11"

    kind: ClassVar[str] = "apriltag"

    def __post_init__(self) -> None:
        super().__post_init__()
        if self.family not in APRILTAG_FAMILIES:
            raise ValueError(
                f"unknown AprilTag family '{self.family}' "
                f"(known: {', '.join(APRILTAG_FAMILIES)})"
            )

    @property
    def label(self) -> str:
        return self.family

    @property
    def dictionary(self) -> int:
        return APRILTAG_FAMILIES[self.family]


@dataclass(frozen=True, kw_only=True)
class ArucoTag(_Fiducial):
    dictionary_name: str = "4x4_250"

    kind: ClassVar[str] = "aruco"

    def __post_init__(self) -> None:
        super().__post_init__()
        if self.dictionary_name not in ARUCO_DICTIONARIES:
            raise ValueError(
                f"unknown ArUco dictionary '{self.dictionary_name}' "
                f"(known: {', '.join(ARUCO_DICTIONARIES)})"
            )

    @property
    def label(self) -> str:
        return self.dictionary_name

    @property
    def dictionary(self) -> int:
        return ARUCO_DICTIONARIES[self.dictionary_name]


@dataclass(frozen=True, kw_only=True)
class YoloDrogue(Perception):
    """The refuelling drogue, found by YOLO and ranged by the pose node
    (ros2_yolo_image_processing): drogue_detection_node draws boxes on the
    camera image, pose_estimation_node turns the drogue's box into a position
    in its ranging frame. Both run on the declared camera. The camera's pitch
    and offset come from the airframe's mount."""

    min_score: float = 0.25  # YOLO confidence a detection needs
    iou: float = 0.45  # YOLO non-maximum-suppression overlap

    frame: ClassVar[str] = "ranging"
    packages: ClassVar[tuple[str, ...]] = ("ros2_yolo_image_processing",)

    def __post_init__(self) -> None:
        super().__post_init__()
        if not 0 < self.min_score < 1 or not 0 < self.iou < 1:
            raise ValueError("min_score and iou must be between 0 and 1")

    @property
    def key(self) -> str:
        return "yolo_drogue"

    def helpers(self, airframe: Airframe) -> list[Helper]:
        camera = airframe.camera(self.camera)
        boxes = f"{self.topic}/detections"
        return [
            Helper(
                f"{self.key}_{self.camera}_detector",
                (
                    "ros2",
                    "run",
                    "ros2_yolo_image_processing",
                    "drogue_detection_node",
                    "--ros-args",
                    "-r",
                    f"__node:{self.key}_{self.camera}_detector",
                    "-r",
                    f"image_raw:={camera.image_topic}",
                    "-r",
                    f"detections:={boxes}",
                    "-p",
                    f"min_score_thresh:={self.min_score}",
                    "-p",
                    f"iou_thresh:={self.iou}",
                ),
            ),
            Helper(
                f"{self.key}_{self.camera}_ranging",
                (
                    "ros2",
                    "run",
                    "ros2_yolo_image_processing",
                    "pose_estimation_node",
                    "--ros-args",
                    "-r",
                    f"__node:{self.key}_{self.camera}_ranging",
                    "-r",
                    f"detections:={boxes}",
                    "-r",
                    f"camera_info:={camera.info_topic}",
                    "-r",
                    f"tag_detections:={self.topic}",
                ),
            ),
        ]
