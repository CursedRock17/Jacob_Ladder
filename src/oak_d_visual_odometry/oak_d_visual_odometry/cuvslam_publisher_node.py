"""OAK-D PyCuVSLAM publisher.

This node runs NVIDIA cuVSLAM on the host from synchronized OAK-D stereo
frames:

  /slam/odometry  geometry_msgs/PoseStamped  rig-in-cuvslam-world
  /tf             tf2_msgs/TFMessage         cuvslam_world -> oak_camera
  /imu/data       sensor_msgs/Imu            raw OAK IMU passthrough
  /rgb/image      sensor_msgs/Image          optional CAM_A color frames
  /features/image sensor_msgs/Image          left frame with tracked feature
                                             points drawn on it (Foxglove)

The rig frame follows NVIDIA's OAK-D sample: CAM_A/RGB is the rig origin and
CAM_B/CAM_C stereo images are expressed relative to it. cuVSLAM uses OpenCV
camera coordinates: X right, Y down, Z forward.

All topic names above are parameters; the shipped config
(config/cuvslam_params.yaml) publishes CAM_A on /front/camera/image_raw so
the same stream feeds the YOLO drogue detector.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from datetime import timedelta
import math
import re
import threading
import time
from typing import Any

import cv2
import depthai as dai
import numpy as np

from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import CameraInfo, Image, Imu
from tf2_ros import TransformBroadcaster

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.exceptions import ParameterUninitializedException
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from . import frames

try:
    import cuvslam as vslam

    CUVSLAM_IMPORT_ERROR = None
except ImportError as exc:
    # Optional dependency: fall back to None on hosts without the wheel.
    vslam = None  # ty: ignore[invalid-assignment]
    CUVSLAM_IMPORT_ERROR = exc

try:
    from px4_msgs.msg import VehicleOdometry

    PX4_AVAILABLE = True
except ImportError:
    VehicleOdometry = None  # ty: ignore[invalid-assignment]
    PX4_AVAILABLE = False


PUBLISHER_QUEUE_SIZE = 10

# uXRCE-DDS uses BEST_EFFORT + KEEP_LAST(5) for PX4 uORB topics.
PX4_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)


@dataclass
class ImuSample:
    timestamp_ns: int
    linear_acceleration: np.ndarray
    angular_velocity: np.ndarray


class CuVslamPublisherNode(Node):
    def __init__(self, publish_px4: bool = False):
        node_name = (
            "oak_d_cuvslam_px4_publisher" if publish_px4 else "oak_d_cuvslam_publisher"
        )
        super().__init__(node_name)

        if vslam is None:
            self.get_logger().error(
                "Could not import cuvslam. Install NVIDIA's PyCuVSLAM wheel on "
                f"the target Jetson/Ubuntu system before running this node: {CUVSLAM_IMPORT_ERROR}"
            )
            raise RuntimeError("cuvslam Python package is not available")

        self._publish_px4_requested = publish_px4
        self._declare_params()
        self._load_params()

        self._create_publishers()

        self.bridge = CvBridge()
        self._rgb_camera_info = None
        self._left_camera_info = None
        self._right_camera_info = None

        # Persistent per-track colors so a feature keeps the same color across
        # frames, mirroring the rerun visualizer.
        self._feature_track_colors: dict[int, tuple[int, int, int]] = {}
        self._feature_color_rng = np.random.default_rng(0)

        self._pending_imu: list[ImuSample] = []
        self._last_cuvslam_timestamp_ns = None
        # Rolling (timestamp_ns, NED position) window for the PX4 velocity
        # fit and the empirical position variance; cleared on tracking loss
        # or a stream gap so neither is computed across a pose jump.
        self._px4_pose_window: deque[tuple[int, np.ndarray]] = deque(
            maxlen=self._velocity_window_frames
        )

        # Tracking-health state reported to PX4. reset_counter tells EKF2 the
        # pose stream is discontinuous (tracking loss or a long frame gap) so
        # it re-references instead of fusing the jump as motion.
        self._tracking_lost = False
        self._reset_counter = 0
        self._stable_frames = 0
        self._last_landmarks_3d: int | None = None
        self._warned_sample_clock = False
        self._last_world_position = None
        self._last_world_timestamp_ns = None
        self._track_stats_start = time.monotonic()
        self._track_stats = {"n": 0, "sum_ms": 0.0, "max_ms": 0.0, "slow": 0, "gaps": 0}

        self._stop = threading.Event()
        self._worker = threading.Thread(target=self._pipeline_loop, daemon=True)
        self._worker.start()

        mode = "stereo+IMU" if self._enable_imu_fusion else "stereo"
        self.get_logger().info(
            f"{node_name} started; opening OAK-D DepthAI pipeline for cuVSLAM ({mode})"
        )

    def _create_publishers(self):
        """Create every ROS publisher; optional ones stay None when disabled."""
        self.odometry_publisher = self.create_publisher(
            PoseStamped, self._odom_topic, PUBLISHER_QUEUE_SIZE
        )
        self.tf_broadcaster = TransformBroadcaster(self)

        self.imu_publisher = None
        if self._publish_imu:
            self.imu_publisher = self.create_publisher(
                Imu, self._imu_topic, PUBLISHER_QUEUE_SIZE
            )

        self.rgb_publisher = None
        self.rgb_camera_info_publisher = None
        if self._publish_rgb:
            self.rgb_publisher = self.create_publisher(
                Image, self._rgb_topic, PUBLISHER_QUEUE_SIZE
            )
            self.rgb_camera_info_publisher = self.create_publisher(
                CameraInfo, self._rgb_camera_info_topic, PUBLISHER_QUEUE_SIZE
            )

        self.left_image_publisher = None
        self.right_image_publisher = None
        self.left_camera_info_publisher = None
        self.right_camera_info_publisher = None
        if self._publish_stereo_images:
            self.left_image_publisher = self.create_publisher(
                Image, self._left_image_topic, PUBLISHER_QUEUE_SIZE
            )
            self.right_image_publisher = self.create_publisher(
                Image, self._right_image_topic, PUBLISHER_QUEUE_SIZE
            )
            self.left_camera_info_publisher = self.create_publisher(
                CameraInfo, self._left_camera_info_topic, PUBLISHER_QUEUE_SIZE
            )
            self.right_camera_info_publisher = self.create_publisher(
                CameraInfo, self._right_camera_info_topic, PUBLISHER_QUEUE_SIZE
            )

        self.features_image_publisher = None
        self.features_camera_info_publisher = None
        if self._publish_features:
            self.features_image_publisher = self.create_publisher(
                Image, self._features_image_topic, PUBLISHER_QUEUE_SIZE
            )
            self.features_camera_info_publisher = self.create_publisher(
                CameraInfo, self._features_camera_info_topic, PUBLISHER_QUEUE_SIZE
            )

        if self._publish_px4_requested and PX4_AVAILABLE:
            self.px4_publisher = self.create_publisher(
                VehicleOdometry, self._px4_topic, PX4_QOS
            )
            self.get_logger().info(
                f"PX4 VehicleOdometry publisher enabled on {self._px4_topic}"
            )
        elif self._publish_px4_requested:
            self.px4_publisher = None
            self.get_logger().warn(
                "px4_msgs not importable; /fmu/in/vehicle_visual_odometry disabled"
            )
        else:
            self.px4_publisher = None

    # ---------- parameters ----------

    def _declare_params(self):
        # -- DepthAI device & streams --
        # Pin to a specific OAK device by DepthAI deviceId (MXID). Empty means
        # first available — unsafe when a second OAK is also plugged in.
        self.declare_parameter("device_id", "")
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("camera_fps", 30.0)
        self.declare_parameter("rgb_fps", 30.0)
        self.declare_parameter("sync_threshold_ms", -1.0)
        self.declare_parameter("warmup_frames", 30)

        # -- optional output streams --
        self.declare_parameter("publish_rgb", True)
        self.declare_parameter("publish_stereo_images", False)
        self.declare_parameter("publish_features", True)
        self.declare_parameter("feature_point_radius", 3)
        self.declare_parameter("publish_imu", True)
        self.declare_parameter("enable_imu_fusion", False)
        self.declare_parameter("imu_rate_hz", 200)

        # -- cuVSLAM engine --
        self.declare_parameter("rectified_stereo_camera", False)
        self.declare_parameter("use_gpu", True)
        self.declare_parameter("async_sba", False)
        self.declare_parameter("enable_final_landmarks_export", False)
        self.declare_parameter("enable_observations_export", False)
        self.declare_parameter("verbosity", 2)
        # cuVSLAM troubleshooting knobs (see cuVSLAM TROUBLESHOOTING.md).
        # debug_dump_directory makes cuVSLAM write an EDEX scene file + TGA
        # images so a failing run can be replayed offline with NVIDIA's
        # tracker/result_visualizer tools (including shuttle-mode
        # forward/backward consistency checks). Empty disables the dump.
        self.declare_parameter("debug_dump_directory", "")
        self.declare_parameter("use_denoising", False)
        self.declare_parameter("use_motion_model", True)
        # cuVSLAM treats a frame-to-frame timestamp delta above this as a
        # broken stream. The library default (1.0 s) invites it to LK-track
        # across the huge appearance change after a stall; Isaac ROS sets
        # ~one frame period. <= 0 selects 3 frame intervals from camera_fps.
        self.declare_parameter("max_frame_delta_s", -1.0)
        # Multicamera engine trade-off: "moderate", "performance", or
        # "precision". Empty keeps the PyCuVSLAM default (Precision in 16.0;
        # Isaac ROS ships Performance) — drop down if track() times grow.
        self.declare_parameter("multicam_mode", "")
        # NVIDIA's IMU-extrinsics check (cuVSLAM TROUBLESHOOTING.md step 9):
        # disables visual tracking and locks translation so the reported
        # rotation comes from the IMU alone; physically rotate the rig and
        # compare. Requires enable_imu_fusion. Bench-only — never fly.
        self.declare_parameter("debug_imu_mode", False)

        self.declare_parameter("left_border_mask", [0, 0, 0, 0])
        self.declare_parameter("right_border_mask", [0, 0, 0, 0])

        # -- frame ids & topic names --
        self.declare_parameter("world_frame_id", "cuvslam_world")
        self.declare_parameter("rig_frame_id", "oak_camera")
        self.declare_parameter("left_frame_id", "oak_left_camera")
        self.declare_parameter("right_frame_id", "oak_right_camera")
        self.declare_parameter("imu_frame_id", "oak_imu")

        self.declare_parameter("odom_topic", "/slam/odometry")
        self.declare_parameter("imu_topic", "/imu/data")
        self.declare_parameter("rgb_topic", "/rgb/image")
        self.declare_parameter("rgb_camera_info_topic", "/rgb/camera_info")
        self.declare_parameter("left_image_topic", "/left/image")
        self.declare_parameter("right_image_topic", "/right/image")
        self.declare_parameter("left_camera_info_topic", "/left/camera_info")
        self.declare_parameter("right_camera_info_topic", "/right/camera_info")
        self.declare_parameter("features_image_topic", "/features/image")
        self.declare_parameter("features_camera_info_topic", "/features/camera_info")
        self.declare_parameter("px4_topic", "/fmu/in/vehicle_visual_odometry")

        # -- PX4 output: mounting, frames, covariance --
        self.declare_parameter("init_yaw_offset_deg", 0.0)
        # How the camera is bolted to the airframe: "forward", "down" (nadir),
        # or "down<degrees>" (angle of depression of the boresight below
        # body-horizontal, e.g. "down30"). Resolved in _preset_r_body_cam.
        self.declare_parameter("camera_mounting", "forward")
        self.declare_parameter("t_body_cam", [0.0, 0.0, 0.0])
        # Optional 3x3 (row-major, 9 values) camera-optical -> body FRD override.
        # Declared with dynamic typing and an empty default so it is always
        # initialized: a bare DOUBLE_ARRAY with a YAML "[]" cannot infer its
        # type and would stay uninitialized, which breaks the get_parameters
        # service used by Foxglove/rqt.
        self.declare_parameter(
            "R_body_cam_override", [], ParameterDescriptor(dynamic_typing=True)
        )
        # position/orientation variances act as floors: when
        # use_cuvslam_covariance is true the per-sample tracker covariance is
        # used wherever it exceeds these values, so degraded tracking is
        # reported to EKF2 instead of a constant (over-)confidence.
        self.declare_parameter("position_variance", [0.01, 0.01, 0.02])
        self.declare_parameter("orientation_variance", [0.01, 0.01, 0.02])
        self.declare_parameter("velocity_variance", [0.1, 0.1, 0.1])
        self.declare_parameter("use_cuvslam_covariance", True)
        # PX4 velocity is the least-squares slope of the last
        # velocity_window_frames NED positions (Isaac ROS pose-cache style)
        # instead of a two-frame difference: at 30 Hz a two-frame diff turns
        # ~5 mm pose noise into ~0.3 m/s velocity noise, which is exactly
        # the signal a position-hold controller damps with. The residual
        # scatter about the same fit is an empirical position variance that,
        # unlike cuVSLAM's covariance, actually grows when tracking degrades;
        # use_empirical_covariance reports it to EKF2 above the floors.
        self.declare_parameter("velocity_window_frames", 10)
        self.declare_parameter("use_empirical_covariance", True)

        # -- PX4 output: tracking-health gates & instrumentation --
        # Instrumentation/health thresholds: warn when a single track() call
        # blocks longer than slow_track_warn_ms, and treat a stereo-frame gap
        # longer than velocity_reset_gap_s as a stream discontinuity (skip
        # velocity differentiation across it and bump reset_counter).
        self.declare_parameter("slow_track_warn_ms", 60.0)
        self.declare_parameter("velocity_reset_gap_s", 0.5)

        # Sanity gate on the tracker output: frame-to-frame motion faster than
        # this is treated as tracking loss (mistracking or inertial-mode
        # dead-reckoning runaway) and withheld from PX4 until the pose
        # stabilizes again, at which point reset_counter is bumped. The
        # tracker's own covariance cannot be relied on for this: it stayed at
        # its nominal value through a 10 m/s divergence. Set above the
        # vehicle's real flight envelope.
        self.declare_parameter("max_vio_speed_mps", 4.0)
        # After a tracking loss, this many consecutive sane frames are
        # required before publishing to PX4 resumes. Prevents thrashing
        # between lost/re-acquired when the tracker alternates garbage and
        # plausible poses frame-to-frame.
        self.declare_parameter("reacquire_stable_frames", 6)
        # Scene-quality gate: a pose constrained by fewer triangulated 3D
        # landmarks than this is withheld from PX4 and treated as tracking
        # loss. Covered/dark lenses still yield hundreds of 2D "tracks" on
        # noise with landmarks3d=0, and runs of them are self-consistent
        # enough to pass both the speed gate and the reacquire hysteresis
        # (2026-07-13 bench cover test: 163 m teleports reached PX4 that
        # way). Healthy scenes on this rig run 24-110 landmarks.
        self.declare_parameter("min_landmarks_3d", 10)

        # -- IMU fusion (only used when enable_imu_fusion is true) --
        # Defaults are placeholders
        # for development; tune them from the target IMU datasheet/calibration.
        self.declare_parameter("imu_gyro_noise_density", 0.001)
        self.declare_parameter("imu_gyro_random_walk", 0.00001)
        self.declare_parameter("imu_accel_noise_density", 0.02)
        self.declare_parameter("imu_accel_random_walk", 0.0001)
        self.declare_parameter("imu_frequency_hz", 200.0)
        # Where the IMU->rig (CAM_A) extrinsics come from: "calibration" reads
        # them from the device EEPROM (with the parameters below as fallback),
        # "params" always uses rig_from_imu_translation/rotation_xyzw. The
        # identity default is a placeholder, not a usable OAK-D value: the OAK
        # IMU axes are permuted relative to the camera optical frame, so IMU
        # fusion is refused rather than run with the placeholder.
        self.declare_parameter("imu_extrinsics_source", "calibration")
        self.declare_parameter("rig_from_imu_translation", [0.0, 0.0, 0.0])
        self.declare_parameter("rig_from_imu_rotation_xyzw", [0.0, 0.0, 0.0, 1.0])

    def _load_params(self):
        self._width = int(self.get_parameter("width").value)
        self._height = int(self.get_parameter("height").value)
        self._camera_fps = float(self.get_parameter("camera_fps").value)
        self._rgb_fps = float(self.get_parameter("rgb_fps").value)
        sync_threshold_ms = float(self.get_parameter("sync_threshold_ms").value)
        if sync_threshold_ms < 0.0:
            sync_threshold_ms = 500.0 / max(self._camera_fps, 1.0)
        self._sync_threshold = timedelta(milliseconds=sync_threshold_ms)
        self._warmup_frames = int(self.get_parameter("warmup_frames").value)

        self._publish_rgb = bool(self.get_parameter("publish_rgb").value)
        self._publish_stereo_images = bool(
            self.get_parameter("publish_stereo_images").value
        )
        self._publish_features = bool(self.get_parameter("publish_features").value)
        self._feature_point_radius = max(
            1, int(self.get_parameter("feature_point_radius").value)
        )
        self._publish_imu = bool(self.get_parameter("publish_imu").value)
        self._enable_imu_fusion = bool(self.get_parameter("enable_imu_fusion").value)
        self._imu_rate_hz = int(self.get_parameter("imu_rate_hz").value)

        self._rectified_stereo_camera = bool(
            self.get_parameter("rectified_stereo_camera").value
        )
        self._use_gpu = bool(self.get_parameter("use_gpu").value)
        self._async_sba = bool(self.get_parameter("async_sba").value)
        self._enable_final_landmarks_export = bool(
            self.get_parameter("enable_final_landmarks_export").value
        )
        self._enable_observations_export = bool(
            self.get_parameter("enable_observations_export").value
        )
        self._verbosity = int(self.get_parameter("verbosity").value)
        self._debug_dump_directory = str(
            self.get_parameter("debug_dump_directory").value
        ).strip()
        self._use_denoising = bool(self.get_parameter("use_denoising").value)
        self._use_motion_model = bool(self.get_parameter("use_motion_model").value)
        self._max_frame_delta_s = float(
            self.get_parameter("max_frame_delta_s").value
        )
        if self._max_frame_delta_s <= 0.0:
            self._max_frame_delta_s = 3.0 / max(self._camera_fps, 1.0)
        self._multicam_mode = (
            str(self.get_parameter("multicam_mode").value).strip().lower()
        )
        self._debug_imu_mode = bool(self.get_parameter("debug_imu_mode").value)

        self._left_border_mask = self._list_param("left_border_mask", 4, int)
        self._right_border_mask = self._list_param("right_border_mask", 4, int)

        self._world_frame_id = str(self.get_parameter("world_frame_id").value)
        self._rig_frame_id = str(self.get_parameter("rig_frame_id").value)
        self._left_frame_id = str(self.get_parameter("left_frame_id").value)
        self._right_frame_id = str(self.get_parameter("right_frame_id").value)
        self._imu_frame_id = str(self.get_parameter("imu_frame_id").value)

        self._odom_topic = str(self.get_parameter("odom_topic").value)
        self._imu_topic = str(self.get_parameter("imu_topic").value)
        self._rgb_topic = str(self.get_parameter("rgb_topic").value)
        self._rgb_camera_info_topic = str(
            self.get_parameter("rgb_camera_info_topic").value
        )
        self._left_image_topic = str(self.get_parameter("left_image_topic").value)
        self._right_image_topic = str(self.get_parameter("right_image_topic").value)
        self._left_camera_info_topic = str(
            self.get_parameter("left_camera_info_topic").value
        )
        self._right_camera_info_topic = str(
            self.get_parameter("right_camera_info_topic").value
        )
        self._features_image_topic = str(
            self.get_parameter("features_image_topic").value
        )
        self._features_camera_info_topic = str(
            self.get_parameter("features_camera_info_topic").value
        )
        self._px4_topic = str(self.get_parameter("px4_topic").value)

        self._t_body_cam = np.array(
            self._list_param("t_body_cam", 3, float), dtype=np.float64
        )

        # The camera mounting (optical -> body FRD) must be resolved before the
        # world->NED rotation, because cuVSLAM's world frame starts aligned with
        # the camera and the two rotations have to describe the same mounting.
        try:
            override_value = self.get_parameter("R_body_cam_override").value
            override = list(override_value) if override_value else []
        except ParameterUninitializedException:
            override = []
        if len(override) == 9:
            self._R_body_cam = np.array(override, dtype=np.float64).reshape(3, 3)
        else:
            self._R_body_cam = self._preset_r_body_cam()

        yaw_deg = float(self.get_parameter("init_yaw_offset_deg").value)
        self._R_ned_world = frames.r_ned_from_cuvslam_world(
            math.radians(yaw_deg), self._R_body_cam
        )

        self._pos_var = np.array(
            self._list_param("position_variance", 3, float), dtype=np.float32
        )
        self._ori_var = np.array(
            self._list_param("orientation_variance", 3, float), dtype=np.float32
        )
        self._vel_var = np.array(
            self._list_param("velocity_variance", 3, float), dtype=np.float32
        )
        self._use_cuvslam_covariance = bool(
            self.get_parameter("use_cuvslam_covariance").value
        )
        self._velocity_window_frames = max(
            2, int(self.get_parameter("velocity_window_frames").value)
        )
        self._use_empirical_covariance = bool(
            self.get_parameter("use_empirical_covariance").value
        )
        self._slow_track_warn_ms = float(self.get_parameter("slow_track_warn_ms").value)
        self._velocity_reset_gap_s = float(
            self.get_parameter("velocity_reset_gap_s").value
        )
        self._max_vio_speed_mps = float(self.get_parameter("max_vio_speed_mps").value)
        self._reacquire_stable_frames = int(
            self.get_parameter("reacquire_stable_frames").value
        )
        self._min_landmarks_3d = int(self.get_parameter("min_landmarks_3d").value)
        self._imu_extrinsics_source = (
            str(self.get_parameter("imu_extrinsics_source").value).strip().lower()
        )

    def _list_param(self, name: str, expected_len: int, item_type):
        value = list(self.get_parameter(name).value)
        if len(value) != expected_len:
            self.get_logger().warn(
                f"{name} must have {expected_len} elements; using zeros"
            )
            return [item_type(0.0)] * expected_len
        return [item_type(v) for v in value]

    def _preset_r_body_cam(self) -> np.ndarray:
        """Camera-optical -> body FRD rotation from the `camera_mounting` preset.

        Overridden by `R_body_cam_override` when a 9-element matrix is given.
        """
        mounting = str(self.get_parameter("camera_mounting").value).strip().lower()
        presets = {
            "forward": frames.R_BODY_FROM_CAM_OPTICAL,
            "down": frames.R_BODY_FROM_CAM_OPTICAL_DOWN,
        }
        preset = presets.get(mounting)
        if preset is None and re.fullmatch(r"down(\d+(?:\.\d+)?)", mounting):
            depression_deg = float(mounting[len("down"):])
            if 0.0 <= depression_deg <= 90.0:
                preset = frames.r_body_from_cam_depression(
                    math.radians(depression_deg)
                )
        if preset is None:
            self.get_logger().warn(
                f"Unknown camera_mounting '{mounting}'; expected 'forward', "
                "'down', or 'down<degrees>' (angle of depression, 0-90, e.g. "
                "'down30'). Falling back to 'forward'."
            )
            preset = frames.R_BODY_FROM_CAM_OPTICAL
        else:
            self.get_logger().info(f"Using '{mounting}' camera mounting")
        return preset.copy()

    # ---------- DepthAI + cuVSLAM pipeline ----------

    def _pipeline_loop(self):
        device = None
        try:
            try:
                # Pay the CUDA/cusolver init cost before frames start
                # streaming instead of inside the first track() call, where
                # it shows up as a slow track with frames dropping behind it.
                vslam.warm_up_gpu()
            except Exception as exc:
                # Non-fatal here: if CUDA is truly broken, Tracker() below
                # fails with a clearer error.
                print(f"[cuvslam] warm_up_gpu failed: {exc!r}", flush=True)
            device_id = str(self.get_parameter("device_id").value or "")
            if device_id:
                print(f"[cuvslam] opening OAK-D device {device_id}", flush=True)
                device = dai.Device(dai.DeviceInfo(device_id))
            else:
                print("[cuvslam] opening first available OAK-D device", flush=True)
                device = dai.Device()
            print(
                f"[cuvslam] opened {device.getDeviceName()} "
                f"(id {device.getDeviceId()})",
                flush=True,
            )
            calibration = device.readCalibration()

            print("[cuvslam] creating cuVSLAM tracker", flush=True)
            tracker = self._create_tracker(calibration)

            print("[cuvslam] creating DepthAI pipeline", flush=True)
            with dai.Pipeline(device) as pipeline:
                left, right = self._create_stereo_cameras(pipeline)
                sync_q = self._create_stereo_sync_queue(pipeline, left, right)
                rgb_q = self._create_rgb_queue(pipeline)
                imu_q = self._create_imu_queue(pipeline)

                print("[cuvslam] starting DepthAI pipeline", flush=True)
                pipeline.start()
                print("[cuvslam] pipeline running", flush=True)

                self._camera_info_from_calibration(calibration)
                self._warmup(sync_q)

                n_stereo = 0
                n_rgb = 0
                while pipeline.isRunning() and not self._stop.is_set():
                    self._drain_imu_queue(imu_q)

                    message_group = sync_q.tryGet()
                    if message_group is not None:
                        self._handle_stereo_group(tracker, message_group)
                        n_stereo += 1

                    if rgb_q is not None:
                        rgb_frame = rgb_q.tryGet()
                        if rgb_frame is not None:
                            self._publish_rgb_frame(rgb_frame)
                            n_rgb += 1

                    total = n_stereo + n_rgb
                    if total > 0 and total % 300 == 0:
                        print(
                            f"[cuvslam] counts: stereo={n_stereo} rgb={n_rgb}",
                            flush=True,
                        )

                    time.sleep(0.001)
        except Exception as exc:
            print(f"[cuvslam] pipeline EXCEPTION: {exc!r}", flush=True)
            self.get_logger().error(f"cuVSLAM pipeline failed: {exc}")
        finally:
            if device is not None:
                try:
                    device.close()
                except Exception:
                    pass

    def _create_stereo_cameras(self, pipeline):
        print("[cuvslam] creating CAM_B (left mono)", flush=True)
        left = pipeline.create(dai.node.Camera).build(
            dai.CameraBoardSocket.CAM_B, sensorFps=self._camera_fps
        )
        print("[cuvslam] creating CAM_C (right mono)", flush=True)
        right = pipeline.create(dai.node.Camera).build(
            dai.CameraBoardSocket.CAM_C, sensorFps=self._camera_fps
        )
        return left, right

    def _create_stereo_sync_queue(self, pipeline, left, right):
        sync = pipeline.create(dai.node.Sync)
        sync.setSyncThreshold(self._sync_threshold)

        print("[cuvslam] linking left/right mono cameras to Sync", flush=True)
        left.requestOutput(
            (self._width, self._height), type=dai.ImgFrame.Type.GRAY8
        ).link(sync.inputs["left"])
        right.requestOutput(
            (self._width, self._height), type=dai.ImgFrame.Type.GRAY8
        ).link(sync.inputs["right"])

        return sync.out.createOutputQueue(maxSize=4, blocking=False)

    def _create_rgb_queue(self, pipeline):
        if not self._publish_rgb:
            return None

        print(f"[cuvslam] creating CAM_A (color) at {self._rgb_fps}fps", flush=True)
        color = pipeline.create(dai.node.Camera).build(
            dai.CameraBoardSocket.CAM_A, sensorFps=self._rgb_fps
        )
        return color.requestOutput((self._width, self._height)).createOutputQueue(
            maxSize=4, blocking=False
        )

    def _create_imu_queue(self, pipeline):
        if not (self._publish_imu or self._enable_imu_fusion):
            return None

        print("[cuvslam] creating IMU", flush=True)
        imu = pipeline.create(dai.node.IMU)
        imu.enableIMUSensor(
            [dai.IMUSensor.ACCELEROMETER_RAW, dai.IMUSensor.GYROSCOPE_RAW],
            self._imu_rate_hz,
        )
        imu.setBatchReportThreshold(1)
        imu.setMaxBatchReports(10)
        # The queue must ride out the longest track() stall without dropping
        # samples: at 200 Hz a 30-deep queue holds only ~0.15 s, and losing
        # IMU coverage during a stall is exactly what sends cuVSLAM's
        # inertial mode into dead-reckoning divergence. 600 packets ≈ 3 s.
        return imu.out.createOutputQueue(maxSize=600, blocking=False)

    def _create_tracker(self, calibration):
        cameras = [
            self._create_vslam_camera(
                calibration,
                dai.CameraBoardSocket.CAM_B,
                dai.CameraBoardSocket.CAM_A,
                self._left_border_mask,
            ),
            self._create_vslam_camera(
                calibration,
                dai.CameraBoardSocket.CAM_C,
                dai.CameraBoardSocket.CAM_A,
                self._right_border_mask,
            ),
        ]

        rig = vslam.Rig(cameras)
        if self._enable_imu_fusion:
            imu_calibration = self._create_vslam_imu(calibration)
            if imu_calibration is None:
                self.get_logger().error(
                    "IMU fusion requested but no usable rig_from_imu extrinsics "
                    "(device calibration invalid and rig_from_imu_* parameters "
                    "are the identity placeholder); falling back to stereo-only "
                    "odometry"
                )
                self._enable_imu_fusion = False
            else:
                rig.imus = [imu_calibration]

        odometry_mode = (
            vslam.Tracker.OdometryMode.Inertial
            if self._enable_imu_fusion
            else vslam.Tracker.OdometryMode.Multicamera
        )
        if self._debug_imu_mode and not self._enable_imu_fusion:
            self.get_logger().error(
                "debug_imu_mode requires enable_imu_fusion with valid IMU "
                "extrinsics; ignoring it"
            )
            self._debug_imu_mode = False
        if self._debug_imu_mode:
            self.get_logger().warn(
                "cuVSLAM debug_imu_mode: visual tracking disabled, translation "
                "locked, rotation from the IMU alone — bench verification only, "
                "never fly with this on"
            )
        # Feature-point visualization needs the tracker to export observations.
        export_observations = self._enable_observations_export or self._publish_features
        odom_config = vslam.Tracker.OdometryConfig(
            async_sba=self._async_sba,
            odometry_mode=odometry_mode,
            enable_final_landmarks_export=self._enable_final_landmarks_export,
            enable_observations_export=export_observations,
            rectified_stereo_camera=self._rectified_stereo_camera,
            use_gpu=self._use_gpu,
            use_denoising=self._use_denoising,
            use_motion_model=self._use_motion_model,
            max_frame_delta_s=self._max_frame_delta_s,
            debug_dump_directory=self._debug_dump_directory,
            debug_imu_mode=self._debug_imu_mode,
        )
        multicam_mode = self._resolve_multicam_mode()
        if multicam_mode is not None:
            odom_config.multicam_mode = multicam_mode
        if self._debug_dump_directory:
            self.get_logger().warn(
                "cuVSLAM EDEX debug dump enabled -> "
                f"{self._debug_dump_directory} (writes TGA images continuously; "
                "disable for normal flights)"
            )

        try:
            vslam.set_verbosity(self._verbosity)
        except AttributeError:
            pass
        tracker = vslam.Tracker(rig, odom_config)
        return tracker

    def _resolve_multicam_mode(self):
        """Map the multicam_mode parameter to the cuVSLAM enum, or None."""
        if not self._multicam_mode:
            return None
        modes = {
            "moderate": vslam.Tracker.MulticameraMode.Moderate,
            "performance": vslam.Tracker.MulticameraMode.Performance,
            "precision": vslam.Tracker.MulticameraMode.Precision,
        }
        mode = modes.get(self._multicam_mode)
        if mode is None:
            self.get_logger().warn(
                f"Unknown multicam_mode '{self._multicam_mode}'; expected "
                "'moderate', 'performance', or 'precision'. Using the library "
                "default."
            )
        return mode

    def _create_vslam_camera(self, calibration, camera_socket, rig_socket, border_mask):
        intrinsics = calibration.getCameraIntrinsics(
            camera_socket, self._width, self._height
        )
        distortion = list(calibration.getDistortionCoefficients(camera_socket))[:8]
        if len(distortion) < 8:
            distortion += [0.0] * (8 - len(distortion))
        extrinsics = np.array(
            calibration.getCameraExtrinsics(camera_socket, rig_socket),
            dtype=np.float64,
        )

        camera = vslam.Camera()
        camera.distortion = vslam.Distortion(
            vslam.Distortion.Model.Polynomial, distortion
        )
        camera.focal = np.array([intrinsics[0][0], intrinsics[1][1]], dtype=np.float64)
        camera.principal = np.array(
            [intrinsics[0][2], intrinsics[1][2]], dtype=np.float64
        )
        camera.size = np.array([self._width, self._height], dtype=np.int64)
        camera.rig_from_camera = self._pose_from_depthai_extrinsics(extrinsics)
        camera.border_top = border_mask[0]
        camera.border_bottom = border_mask[1]
        camera.border_left = border_mask[2]
        camera.border_right = border_mask[3]
        return camera

    def _create_vslam_imu(self, calibration):
        rig_from_imu = self._resolve_rig_from_imu(calibration)
        if rig_from_imu is None:
            return None
        rotation, translation_m = rig_from_imu

        imu = vslam.ImuCalibration()
        imu.gyroscope_noise_density = float(
            self.get_parameter("imu_gyro_noise_density").value
        )
        imu.gyroscope_random_walk = float(
            self.get_parameter("imu_gyro_random_walk").value
        )
        imu.accelerometer_noise_density = float(
            self.get_parameter("imu_accel_noise_density").value
        )
        imu.accelerometer_random_walk = float(
            self.get_parameter("imu_accel_random_walk").value
        )
        imu.frequency = float(self.get_parameter("imu_frequency_hz").value)
        imu.rig_from_imu = vslam.Pose(
            rotation=[float(v) for v in frames.rot_to_quat_xyzw(rotation)],
            translation=[float(v) for v in translation_m],
        )
        return imu

    def _resolve_rig_from_imu(self, calibration):
        """IMU -> rig (CAM_A optical) extrinsics as (R, t_meters), or None.

        Prefers the device EEPROM calibration; falls back to the
        rig_from_imu_* parameters. The identity-parameter placeholder is
        treated as "unset" because it is never correct for an OAK-D.
        """
        if self._imu_extrinsics_source == "calibration":
            try:
                extrinsics = np.array(
                    calibration.getImuToCameraExtrinsics(dai.CameraBoardSocket.CAM_A),
                    dtype=np.float64,
                )
                rotation = extrinsics[:3, :3]
                translation_m = extrinsics[:3, 3] * 0.01
                if self._is_valid_rotation(rotation):
                    self.get_logger().info(
                        "rig_from_imu from device calibration: "
                        f"R={rotation.tolist()} t={translation_m.tolist()} m"
                    )
                    return rotation, translation_m
                self.get_logger().warn(
                    "Device calibration has no valid IMU extrinsics "
                    f"(R={rotation.tolist()}); falling back to rig_from_imu_* "
                    "parameters"
                )
            except Exception as exc:
                self.get_logger().warn(
                    f"Could not read IMU extrinsics from device calibration: {exc}; "
                    "falling back to rig_from_imu_* parameters"
                )

        q = self._list_param("rig_from_imu_rotation_xyzw", 4, float)
        t = np.array(self._list_param("rig_from_imu_translation", 3, float))
        if np.allclose(q, [0.0, 0.0, 0.0, 1.0]) and np.allclose(t, 0.0):
            return None
        rotation = frames.quat_to_rot(q[0], q[1], q[2], q[3])
        self.get_logger().info(
            f"rig_from_imu from parameters: q_xyzw={q} t={t.tolist()} m"
        )
        return rotation, t

    @staticmethod
    def _is_valid_rotation(rotation: np.ndarray) -> bool:
        return abs(np.linalg.det(rotation) - 1.0) < 1e-2 and np.allclose(
            rotation @ rotation.T, np.eye(3), atol=1e-2
        )

    def _pose_from_depthai_extrinsics(self, extrinsics: np.ndarray):
        rotation = extrinsics[:3, :3]
        translation_m = extrinsics[:3, 3] * 0.01
        return vslam.Pose(
            rotation=frames.rot_to_quat_xyzw(rotation).tolist(),
            translation=translation_m.tolist(),
        )

    def _camera_info_from_calibration(self, calibration):
        self._left_camera_info = self._make_camera_info(
            calibration, dai.CameraBoardSocket.CAM_B, self._left_frame_id
        )
        self._right_camera_info = self._make_camera_info(
            calibration, dai.CameraBoardSocket.CAM_C, self._right_frame_id
        )
        self._rgb_camera_info = self._make_camera_info(
            calibration, dai.CameraBoardSocket.CAM_A, self._rig_frame_id
        )

    def _make_camera_info(self, calibration, camera_socket, frame_id):
        msg = CameraInfo()
        msg.width = self._width
        msg.height = self._height
        msg.header.frame_id = frame_id
        try:
            intrinsics = calibration.getCameraIntrinsics(
                camera_socket, self._width, self._height
            )
            distortion = list(calibration.getDistortionCoefficients(camera_socket))
            msg.distortion_model = (
                "rational_polynomial" if len(distortion) > 5 else "plumb_bob"
            )
            msg.d = distortion
            msg.k = [
                intrinsics[0][0],
                intrinsics[0][1],
                intrinsics[0][2],
                intrinsics[1][0],
                intrinsics[1][1],
                intrinsics[1][2],
                intrinsics[2][0],
                intrinsics[2][1],
                intrinsics[2][2],
            ]
            msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            msg.p = [
                intrinsics[0][0],
                intrinsics[0][1],
                intrinsics[0][2],
                0.0,
                intrinsics[1][0],
                intrinsics[1][1],
                intrinsics[1][2],
                0.0,
                intrinsics[2][0],
                intrinsics[2][1],
                intrinsics[2][2],
                0.0,
            ]
        except Exception as exc:
            self.get_logger().warn(
                f"Could not read camera calibration for {frame_id}: {exc}"
            )
        return msg

    def _warmup(self, sync_q):
        if self._warmup_frames <= 0:
            return

        print(
            f"[cuvslam] dropping {self._warmup_frames} warmup stereo frames", flush=True
        )
        dropped = 0
        while dropped < self._warmup_frames and not self._stop.is_set():
            frame = sync_q.tryGet()
            if frame is not None:
                dropped += 1
            else:
                time.sleep(0.005)

    def _handle_stereo_group(self, tracker, message_group):
        left_frame = message_group["left"]
        right_frame = message_group["right"]
        timestamp_ns = self._timestamp_to_ns(message_group)

        if (
            self._last_cuvslam_timestamp_ns is not None
            and timestamp_ns <= self._last_cuvslam_timestamp_ns
        ):
            self.get_logger().warn(
                "Dropping non-monotonic stereo frame timestamp from DepthAI"
            )
            return

        self._check_frame_gap(timestamp_ns)
        self._register_imu_until(tracker, timestamp_ns)

        track_start = time.perf_counter()
        try:
            pose_estimate, _ = tracker.track(
                timestamp_ns,
                [left_frame.getCvFrame(), right_frame.getCvFrame()],
            )
        except Exception as exc:
            self.get_logger().error(f"cuVSLAM track() failed: {exc}")
            self._on_tracking_lost(f"track() raised: {exc}")
            return
        self._record_track_time((time.perf_counter() - track_start) * 1000.0, tracker)

        self._last_cuvslam_timestamp_ns = timestamp_ns
        self._publish_stereo_debug_images(left_frame, right_frame)
        self._publish_feature_image(tracker, left_frame)

        pose, covariance = self._extract_world_from_rig_pose(pose_estimate)
        if pose is None:
            self._on_tracking_lost("tracker returned an invalid pose")
            return

        # Pose and TF always flow (they are the debugging view); only the PX4
        # stream is gated on tracking health.
        self._publish_pose(
            pose,
            covariance,
            timestamp_ns,
            publish_px4=self._pose_health_gate(tracker, pose, timestamp_ns),
            host_capture_ns=self._host_timestamp_to_ns(message_group),
        )

    def _pose_health_gate(self, tracker, pose: Any, timestamp_ns: int) -> bool:
        """Decide whether this pose is trustworthy enough to send to PX4.

        Three gates, each of which marks tracking as lost and blocks PX4:
          1. Scene quality: too few triangulated 3D landmarks means the pose
             is unconstrained (a covered/dark lens still yields hundreds of
             2D "tracks" on noise).
          2. Speed sanity: frame-to-frame motion above the flight envelope is
             mistracking or a dead-reckoning runaway.
          3. Reacquire hysteresis: after a loss, a run of consecutive sane
             frames is required before PX4 publishing resumes — a single
             plausible frame between two garbage ones is not a recovery.
             A gate failure resets the run, so a blackout never counts as
             "stable". Recovery bumps reset_counter so EKF2 re-references.
        """
        landmarks_3d = self._landmark_count(tracker)
        self._last_landmarks_3d = landmarks_3d
        if landmarks_3d is not None and landmarks_3d < self._min_landmarks_3d:
            self._on_tracking_lost(
                f"only {landmarks_3d} 3D landmarks "
                f"(< {self._min_landmarks_3d}); pose is unconstrained"
            )
            return False

        speed = self._world_speed(pose, timestamp_ns)
        if speed is not None and speed > self._max_vio_speed_mps:
            self._on_tracking_lost(
                f"implausible VIO speed {speed:.1f} m/s "
                f"(> {self._max_vio_speed_mps:.1f} m/s)"
            )
            return False

        if self._tracking_lost:
            self._stable_frames += 1
            if self._stable_frames < self._reacquire_stable_frames:
                return False
            self._reset_counter += 1
            self._tracking_lost = False
            self.get_logger().info(
                f"cuVSLAM tracking re-acquired after {self._stable_frames} stable "
                f"frames; reset_counter={self._reset_counter}"
            )
        return True

    def _landmark_count(self, tracker) -> int | None:
        """Current triangulated 3D landmark count, or None if unreadable."""
        try:
            return len(tracker.get_last_landmarks())
        except Exception:
            return None

    def _world_speed(self, pose: Any, timestamp_ns: int) -> float | None:
        """Frame-to-frame speed of the rig in the cuVSLAM world frame.

        Returns None on the first frame or across a stream gap longer than
        velocity_reset_gap_s (a jump across a gap is handled by the gap's own
        reset_counter bump, not the speed gate).
        """
        position = np.array(pose.translation, dtype=np.float64)
        last_position = self._last_world_position
        last_timestamp_ns = self._last_world_timestamp_ns
        self._last_world_position = position
        self._last_world_timestamp_ns = timestamp_ns

        if last_position is None or last_timestamp_ns is None:
            return None
        dt = (timestamp_ns - last_timestamp_ns) * 1e-9
        if dt <= 1e-4 or dt > self._velocity_reset_gap_s:
            return None
        return float(np.linalg.norm(position - last_position) / dt)

    def _extract_world_from_rig_pose(self, pose_estimate):
        world_from_rig = getattr(pose_estimate, "world_from_rig", None)
        if world_from_rig is None:
            return None, None
        pose = getattr(world_from_rig, "pose", world_from_rig)
        return pose, getattr(world_from_rig, "covariance", None)

    def _on_tracking_lost(self, reason: str):
        # Drop the velocity/variance window so the position jump at
        # re-acquisition is not reported to PX4 as a huge velocity.
        self._px4_pose_window.clear()
        self._stable_frames = 0
        if not self._tracking_lost:
            self._tracking_lost = True
            self.get_logger().warn(f"cuVSLAM tracking lost: {reason}")

    def _check_frame_gap(self, timestamp_ns: int):
        """Detect stereo-frame stream stalls (dropped frames, blocked loop)."""
        if self._last_cuvslam_timestamp_ns is None:
            return
        dt = (timestamp_ns - self._last_cuvslam_timestamp_ns) * 1e-9
        expected = 1.0 / max(self._camera_fps, 1.0)
        if dt > 3.0 * expected:
            self._track_stats["gaps"] += 1
            self.get_logger().warn(
                f"stereo frame gap of {dt:.2f}s "
                f"(~{max(0, round(dt / expected) - 1)} frames dropped); "
                "pipeline stalled or sync queue overflowed"
            )
        if dt > self._velocity_reset_gap_s:
            # The pose may jump across the gap: don't fit velocity across it
            # and tell EKF2 the stream is discontinuous.
            self._px4_pose_window.clear()
            self._reset_counter += 1

    def _record_track_time(self, duration_ms: float, tracker=None):
        stats = self._track_stats
        stats["n"] += 1
        stats["sum_ms"] += duration_ms
        stats["max_ms"] = max(stats["max_ms"], duration_ms)
        if duration_ms > self._slow_track_warn_ms:
            stats["slow"] += 1
            self.get_logger().warn(
                f"slow cuVSLAM track(): {duration_ms:.0f}ms "
                f"(> {self._slow_track_warn_ms:.0f}ms); frames are being "
                "dropped while it blocks",
                throttle_duration_sec=5.0,
            )

        now = time.monotonic()
        if now - self._track_stats_start >= 10.0 and stats["n"] > 0:
            self.get_logger().info(
                f"track() stats over {now - self._track_stats_start:.0f}s: "
                f"n={stats['n']} mean={stats['sum_ms'] / stats['n']:.1f}ms "
                f"max={stats['max_ms']:.1f}ms slow={stats['slow']} "
                f"frame_gaps={stats['gaps']}{self._stereo_health(tracker)}"
            )
            self._track_stats_start = now
            self._track_stats = {
                "n": 0,
                "sum_ms": 0.0,
                "max_ms": 0.0,
                "slow": 0,
                "gaps": 0,
            }

    def _stereo_health(self, tracker) -> str:
        """2D-track vs triangulated-landmark counts for the stats log.

        Many 2D tracks with few 3D landmarks means left-right stereo matching
        is failing (e.g. the scene is closer than the LK matcher's
        zero-disparity init can converge from) and pose/scale rests on only a
        handful of points, even when the feature overlay looks healthy.
        """
        if tracker is None:
            return ""
        try:
            landmarks = tracker.get_last_landmarks()
            tracks2d = len(tracker.get_last_observations(0))
        except Exception:
            return ""
        zs = sorted(lm.coords[2] for lm in landmarks)
        median_z = zs[len(zs) // 2] if zs else float("nan")
        return (
            f" tracks2d={tracks2d} landmarks3d={len(landmarks)} "
            f"median_z={median_z:.2f}m"
        )

    # ---------- IMU ----------

    def _drain_imu_queue(self, imu_q):
        if imu_q is None:
            return

        while True:
            packet = imu_q.tryGet()
            if packet is None:
                break
            for imu_packet in packet.packets:
                sample = self._imu_sample_from_depthai(imu_packet)
                if sample is None:
                    continue
                if self._publish_imu and self.imu_publisher is not None:
                    self._publish_imu_sample(sample)
                if self._enable_imu_fusion:
                    self._pending_imu.append(sample)

    def _imu_sample_from_depthai(self, imu_packet):
        accel = imu_packet.acceleroMeter
        gyro = imu_packet.gyroscope
        timestamp_ns = self._timestamp_to_ns(accel)
        return ImuSample(
            timestamp_ns=timestamp_ns,
            linear_acceleration=np.array([accel.x, accel.y, accel.z], dtype=np.float64),
            angular_velocity=np.array([gyro.x, gyro.y, gyro.z], dtype=np.float64),
        )

    def _register_imu_until(self, tracker, image_timestamp_ns: int):
        if not self._enable_imu_fusion or not self._pending_imu:
            return

        if self._last_cuvslam_timestamp_ns is None:
            self._pending_imu = [
                sample
                for sample in self._pending_imu
                if sample.timestamp_ns > image_timestamp_ns
            ]
            return

        keep = []
        for sample in self._pending_imu:
            if sample.timestamp_ns <= self._last_cuvslam_timestamp_ns:
                continue
            if sample.timestamp_ns >= image_timestamp_ns:
                keep.append(sample)
                continue
            try:
                measurement = vslam.ImuMeasurement()
                measurement.timestamp_ns = sample.timestamp_ns
                measurement.linear_accelerations = sample.linear_acceleration.tolist()
                measurement.angular_velocities = sample.angular_velocity.tolist()
                tracker.register_imu_measurement(0, measurement)
            except Exception as exc:
                self.get_logger().warn(f"Failed to register cuVSLAM IMU sample: {exc}")
        self._pending_imu = keep

    def _publish_imu_sample(self, sample: ImuSample):
        if self.imu_publisher is None:
            return
        msg = Imu()
        msg.header.stamp = self._now()
        msg.header.frame_id = self._imu_frame_id
        msg.linear_acceleration.x = float(sample.linear_acceleration[0])
        msg.linear_acceleration.y = float(sample.linear_acceleration[1])
        msg.linear_acceleration.z = float(sample.linear_acceleration[2])
        msg.angular_velocity.x = float(sample.angular_velocity[0])
        msg.angular_velocity.y = float(sample.angular_velocity[1])
        msg.angular_velocity.z = float(sample.angular_velocity[2])
        msg.orientation_covariance[0] = -1.0
        self.imu_publisher.publish(msg)

    # ---------- publishers ----------

    def _publish_pose(
        self,
        pose: Any,
        covariance: Any,
        sample_timestamp_ns: int,
        publish_px4: bool = True,
        host_capture_ns: int | None = None,
    ):
        translation = np.array(pose.translation, dtype=np.float64)
        rotation = np.array(pose.rotation, dtype=np.float64)
        now = self.get_clock().now()
        stamp = now.to_msg()

        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = self._world_frame_id
        pose_msg.pose.position.x = float(translation[0])
        pose_msg.pose.position.y = float(translation[1])
        pose_msg.pose.position.z = float(translation[2])
        pose_msg.pose.orientation.x = float(rotation[0])
        pose_msg.pose.orientation.y = float(rotation[1])
        pose_msg.pose.orientation.z = float(rotation[2])
        pose_msg.pose.orientation.w = float(rotation[3])
        self.odometry_publisher.publish(pose_msg)

        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = self._world_frame_id
        tf_msg.child_frame_id = self._rig_frame_id
        tf_msg.transform.translation.x = float(translation[0])
        tf_msg.transform.translation.y = float(translation[1])
        tf_msg.transform.translation.z = float(translation[2])
        tf_msg.transform.rotation.x = float(rotation[0])
        tf_msg.transform.rotation.y = float(rotation[1])
        tf_msg.transform.rotation.z = float(rotation[2])
        tf_msg.transform.rotation.w = float(rotation[3])
        self.tf_broadcaster.sendTransform(tf_msg)

        if publish_px4 and self.px4_publisher is not None:
            self._publish_px4(
                translation,
                rotation,
                covariance,
                sample_timestamp_ns,
                now.nanoseconds,
                host_capture_ns,
            )

    def _publish_px4(
        self,
        p_world_rig: np.ndarray,
        q_world_rig_xyzw: np.ndarray,
        covariance: Any,
        sample_timestamp_ns: int,
        ros_timestamp_ns: int,
        host_capture_ns: int | None = None,
    ):
        if self.px4_publisher is None:
            return
        R_world_rig = frames.quat_to_rot(
            q_world_rig_xyzw[0],
            q_world_rig_xyzw[1],
            q_world_rig_xyzw[2],
            q_world_rig_xyzw[3],
        )
        p_ned_body, R_ned_body = frames.transform_pose(
            p_world_rig,
            R_world_rig,
            self._R_body_cam,
            self._t_body_cam,
            self._R_ned_world,
        )

        self._px4_pose_window.append((sample_timestamp_ns, p_ned_body))
        v_ned, pos_var_empirical = self._fit_pose_window()
        if v_ned is not None:
            velocity_frame = VehicleOdometry.VELOCITY_FRAME_NED
        else:
            v_ned = np.array([float("nan")] * 3)
            velocity_frame = VehicleOdometry.VELOCITY_FRAME_UNKNOWN

        pos_var, ori_var = self._px4_variances(
            covariance, R_world_rig, pos_var_empirical
        )

        msg = VehicleOdometry()
        msg.timestamp = int(ros_timestamp_ns // 1000)
        msg.timestamp_sample = self._px4_timestamp_sample_us(
            ros_timestamp_ns, host_capture_ns
        )
        msg.pose_frame = VehicleOdometry.POSE_FRAME_NED
        msg.position = p_ned_body.astype(np.float32).tolist()
        msg.q = frames.rot_to_px4_quat(R_ned_body).tolist()
        msg.velocity_frame = velocity_frame
        msg.velocity = v_ned.astype(np.float32).tolist()
        msg.angular_velocity = [float("nan")] * 3
        msg.position_variance = pos_var.tolist()
        msg.orientation_variance = ori_var.tolist()
        msg.velocity_variance = self._vel_var.tolist()
        msg.reset_counter = int(self._reset_counter % 256)
        msg.quality = self._px4_quality()
        self.px4_publisher.publish(msg)

    def _px4_timestamp_sample_us(
        self, ros_timestamp_ns: int, host_capture_ns: int | None
    ) -> int:
        """VehicleOdometry.timestamp_sample: the camera capture time, in us.

        It must be the capture time, not the publish time, so EKF2 fuses the
        pose against the vehicle state at exposure (EKF2_EV_DELAY then only
        covers residual, unmodeled lag). DepthAI stamps frames in the host
        steady clock while PX4 expects the ROS epoch domain (the uXRCE agent
        timesyncs both timestamp fields), so bridge the domains by
        subtracting the measured capture->publish latency from ROS time.
        Falls back to the publish time when the host-clock capture time is
        missing or implies an implausible (>500 ms or negative) latency.
        """
        latency_ns = (
            time.monotonic_ns() - host_capture_ns if host_capture_ns is not None else -1
        )
        if 0 <= latency_ns < 500_000_000:
            return int((ros_timestamp_ns - latency_ns) // 1000)
        if not self._warned_sample_clock:
            self._warned_sample_clock = True
            self.get_logger().warning(
                f"Host-clock capture time unavailable or implausible "
                f"(apparent capture->publish latency {latency_ns * 1e-6:.0f} ms); "
                "falling back to timestamp_sample = publish time"
            )
        return int(ros_timestamp_ns // 1000)

    def _px4_quality(self) -> int:
        """VehicleOdometry.quality scaled by the 3D landmark count.

        2x landmarks capped at 100: ~50+ landmarks (a good scene on the
        2026-07-13 bench) saturates, the min_landmarks_3d gate floor (10)
        maps to 20, so EKF2_EV_QMIN can de-weight marginal scenes. 100 is
        the no-information fallback when the landmark API is unavailable.
        """
        if self._last_landmarks_3d is None:
            return 100
        return int(np.clip(2 * self._last_landmarks_3d, 1, 100))

    def _fit_pose_window(self) -> tuple[np.ndarray | None, np.ndarray | None]:
        """Least-squares NED velocity and residual variance over the window.

        Fits position = mean + v * t per axis across the rolling pose window,
        so the velocity reported to EKF2 averages pose noise over the whole
        window instead of amplifying it by the frame rate the way a two-frame
        difference does. The per-axis variance of the residuals about that
        constant-velocity fit is an empirical tracking-jitter measure —
        cuVSLAM's own covariance stays nominal even through runaways. Returns
        (None, None) with fewer than 2 samples; the variance additionally
        needs 4+ samples to mean anything with two fitted parameters.
        """
        window = self._px4_pose_window
        if len(window) < 2:
            return None, None
        t = np.array([sample[0] for sample in window], dtype=np.float64)
        p = np.stack([sample[1] for sample in window])
        ts = (t - t.mean()) * 1e-9
        denom = float(np.dot(ts, ts))
        if denom < 1e-9:
            return None, None
        p_centered = p - p.mean(axis=0)
        v_ned = ts @ p_centered / denom
        if len(window) < 4:
            return v_ned, None
        residuals = p_centered - np.outer(ts, v_ned)
        return v_ned, (residuals**2).mean(axis=0)

    def _px4_variances(
        self,
        covariance: Any,
        R_world_rig: np.ndarray,
        pos_var_empirical: np.ndarray | None,
    ) -> tuple[np.ndarray, np.ndarray]:
        """Per-sample variances for PX4, floored by the configured values.

        cuVSLAM's world_from_rig covariance is a 6x6 in the world frame,
        ordered (rotation about x/y/z as fixed axes, then translation x/y/z).
        The translation block is rotated into NED and the rotation block into
        body FRD to match VehicleOdometry's position/orientation variance
        conventions. The empirical window variance (already NED, see
        _fit_pose_window) raises the position variance further wherever the
        recent pose scatter exceeds both.
        """
        pos_var = self._pos_var.astype(np.float64)
        ori_var = self._ori_var.astype(np.float64)

        if self._use_empirical_covariance and pos_var_empirical is not None:
            pos_var = np.maximum(pos_var, pos_var_empirical)

        cov = None
        if self._use_cuvslam_covariance and covariance is not None:
            cov = np.asarray(covariance, dtype=np.float64)
            if cov.size != 36 or not np.all(np.isfinite(cov)):
                cov = None
            elif np.array_equal(cov.reshape(6, 6), np.eye(6)):
                # cuVSLAM returns an exact identity covariance as a "no
                # estimate" sentinel for a few frames around keyframe/SBA
                # updates (1 m^2 / 1 rad^2 — bench-measured at ~4% of
                # samples). Forwarding it would randomly de-weight vision in
                # EKF2; fall back to the floors + empirical variance instead.
                cov = None
        if cov is not None:
            cov = cov.reshape(6, 6)
            cov_rot_world = cov[:3, :3]
            cov_trans_world = cov[3:, 3:]
            pos_var = np.maximum(
                pos_var,
                np.diag(self._R_ned_world @ cov_trans_world @ self._R_ned_world.T),
            )
            R_body_world = self._R_body_cam @ R_world_rig.T
            ori_var = np.maximum(
                ori_var, np.diag(R_body_world @ cov_rot_world @ R_body_world.T)
            )

        return pos_var.astype(np.float32), ori_var.astype(np.float32)

    def _feature_color(self, track_id: int) -> tuple[int, int, int]:
        """Return a stable BGR color for a feature track id."""
        color = self._feature_track_colors.get(track_id)
        if color is None:
            rgb = self._feature_color_rng.integers(64, 256, size=3)
            # OpenCV/cv_bridge bgr8 expects BGR ordering.
            color = (int(rgb[2]), int(rgb[1]), int(rgb[0]))
            self._feature_track_colors[track_id] = color
        return color

    def _publish_feature_image(self, tracker, left_frame):
        """Draw the left camera's tracked features and publish for Foxglove.

        Mirrors the rerun example's 2D observation overlay: each tracked
        feature is drawn as a circle whose color is stable across frames so a
        single Foxglove Image panel on ``features_image_topic`` shows the
        features being tracked.
        """
        if self.features_image_publisher is None:
            return

        # Always publish the frame (even with zero features or a read error) so
        # the topic keeps streaming and stays discoverable in Foxglove, and so
        # the live downward view is available for debugging tracking dropouts.
        try:
            observations = tracker.get_last_observations(0)
        except Exception as exc:
            observations = []
            self.get_logger().warn(
                f"Could not read cuVSLAM observations: {exc}", throttle_duration_sec=5.0
            )

        overlay = cv2.cvtColor(left_frame.getCvFrame(), cv2.COLOR_GRAY2BGR)
        for obs in observations:
            center = (int(round(obs.u)), int(round(obs.v)))
            cv2.circle(
                overlay,
                center,
                self._feature_point_radius,
                self._feature_color(obs.id),
                thickness=-1,
                lineType=cv2.LINE_AA,
            )

        stamp = self._now()
        msg = self.bridge.cv2_to_imgmsg(overlay, encoding="bgr8")
        msg.header.stamp = stamp
        msg.header.frame_id = self._left_frame_id
        self.features_image_publisher.publish(msg)

        if (
            self._left_camera_info is not None
            and self.features_camera_info_publisher is not None
        ):
            self._left_camera_info.header = msg.header
            self.features_camera_info_publisher.publish(self._left_camera_info)

    def _publish_stereo_debug_images(self, left_frame, right_frame):
        if self.left_image_publisher is None or self.right_image_publisher is None:
            return

        stamp = self._now()
        left_msg = self.bridge.cv2_to_imgmsg(left_frame.getCvFrame(), encoding="mono8")
        left_msg.header.stamp = stamp
        left_msg.header.frame_id = self._left_frame_id
        self.left_image_publisher.publish(left_msg)

        right_msg = self.bridge.cv2_to_imgmsg(
            right_frame.getCvFrame(), encoding="mono8"
        )
        right_msg.header.stamp = stamp
        right_msg.header.frame_id = self._right_frame_id
        self.right_image_publisher.publish(right_msg)

        if (
            self._left_camera_info is not None
            and self.left_camera_info_publisher is not None
        ):
            self._left_camera_info.header = left_msg.header
            self.left_camera_info_publisher.publish(self._left_camera_info)
        if (
            self._right_camera_info is not None
            and self.right_camera_info_publisher is not None
        ):
            self._right_camera_info.header = right_msg.header
            self.right_camera_info_publisher.publish(self._right_camera_info)

    _logged_rgb_shape = False

    def _publish_rgb_frame(self, frame):
        if self.rgb_publisher is None:
            return
        cv_img = frame.getCvFrame()
        if not CuVslamPublisherNode._logged_rgb_shape:
            print(
                f"[cuvslam] rgb cv_img shape={cv_img.shape} dtype={cv_img.dtype}",
                flush=True,
            )
            CuVslamPublisherNode._logged_rgb_shape = True

        msg = self.bridge.cv2_to_imgmsg(cv_img, encoding="bgr8")
        msg.header.stamp = self._now()
        msg.header.frame_id = self._rig_frame_id
        self.rgb_publisher.publish(msg)

        if (
            self._rgb_camera_info is not None
            and self.rgb_camera_info_publisher is not None
        ):
            self._rgb_camera_info.header = msg.header
            self.rgb_camera_info_publisher.publish(self._rgb_camera_info)

    def _host_timestamp_to_ns(self, packet) -> int | None:
        """Capture time in the host steady clock, or None if unavailable.

        cuVSLAM is fed device-clock timestamps (_timestamp_to_ns) so frames
        and IMU share one drift-free clock; this host-synced variant exists
        only to measure the capture->publish latency for PX4's
        timestamp_sample.
        """
        if not hasattr(packet, "getTimestamp"):
            return None
        timestamp = packet.getTimestamp()
        if hasattr(timestamp, "total_seconds"):
            return int(timestamp.total_seconds() * 1e9)
        return int(float(timestamp) * 1e9)

    def _timestamp_to_ns(self, packet) -> int:
        timestamp = None
        for method in ("getTimestampDevice", "getTimestamp"):
            if hasattr(packet, method):
                timestamp = getattr(packet, method)()
                break
        if timestamp is None:
            return self.get_clock().now().nanoseconds
        if hasattr(timestamp, "total_seconds"):
            return int(timestamp.total_seconds() * 1e9)
        return int(float(timestamp) * 1e9)

    def _now(self):
        return self.get_clock().now().to_msg()

    def destroy_node(self):
        # Ask the DepthAI worker to stop and give it time to tear the pipeline
        # and device down cleanly. It is a daemon thread, so if we returned
        # while it was still mid-call the interpreter would kill it during a
        # native DepthAI call and the C++ runtime would abort().
        self._stop.set()
        if self._worker.is_alive():
            self._worker.join(timeout=10.0)
            if self._worker.is_alive():
                self.get_logger().warn(
                    "DepthAI worker did not stop within 10s; shutting down anyway"
                )
        super().destroy_node()


def main(args=None, publish_px4: bool = False):
    rclpy.init(args=args)
    node = None
    try:
        node = CuVslamPublisherNode(publish_px4=publish_px4)
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # Ctrl-C: launch's signal handler may have already shut the context
        # down. Fall through to clean up the node and shut down idempotently.
        pass
    finally:
        if node is not None:
            node.destroy_node()
        # try_shutdown() is a no-op if the context is already shut down, so it
        # never raises "rcl_shutdown already called" the way shutdown() does.
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
