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
"""

from __future__ import annotations

from dataclasses import dataclass
from datetime import timedelta
import math
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
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from . import frames

try:
    import cuvslam as vslam
    CUVSLAM_IMPORT_ERROR = None
except ImportError as exc:
    vslam = None
    CUVSLAM_IMPORT_ERROR = exc

try:
    from px4_msgs.msg import VehicleOdometry
    PX4_AVAILABLE = True
except ImportError:
    VehicleOdometry = None
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
        node_name = "oak_d_cuvslam_px4_publisher" if publish_px4 else "oak_d_cuvslam_publisher"
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

        self.odometry_publisher = self.create_publisher(
            PoseStamped, self._odom_topic, PUBLISHER_QUEUE_SIZE)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.imu_publisher = None
        if self._publish_imu:
            self.imu_publisher = self.create_publisher(
                Imu, self._imu_topic, PUBLISHER_QUEUE_SIZE)

        self.rgb_publisher = None
        self.rgb_camera_info_publisher = None
        if self._publish_rgb:
            self.rgb_publisher = self.create_publisher(
                Image, self._rgb_topic, PUBLISHER_QUEUE_SIZE)
            self.rgb_camera_info_publisher = self.create_publisher(
                CameraInfo, self._rgb_camera_info_topic, PUBLISHER_QUEUE_SIZE)

        self.left_image_publisher = None
        self.right_image_publisher = None
        self.left_camera_info_publisher = None
        self.right_camera_info_publisher = None
        if self._publish_stereo_images:
            self.left_image_publisher = self.create_publisher(
                Image, self._left_image_topic, PUBLISHER_QUEUE_SIZE)
            self.right_image_publisher = self.create_publisher(
                Image, self._right_image_topic, PUBLISHER_QUEUE_SIZE)
            self.left_camera_info_publisher = self.create_publisher(
                CameraInfo, self._left_camera_info_topic, PUBLISHER_QUEUE_SIZE)
            self.right_camera_info_publisher = self.create_publisher(
                CameraInfo, self._right_camera_info_topic, PUBLISHER_QUEUE_SIZE)

        self.features_image_publisher = None
        self.features_camera_info_publisher = None
        if self._publish_features:
            self.features_image_publisher = self.create_publisher(
                Image, self._features_image_topic, PUBLISHER_QUEUE_SIZE)
            self.features_camera_info_publisher = self.create_publisher(
                CameraInfo, self._features_camera_info_topic, PUBLISHER_QUEUE_SIZE)

        if self._publish_px4_requested and PX4_AVAILABLE:
            self.px4_publisher = self.create_publisher(
                VehicleOdometry, self._px4_topic, PX4_QOS)
            self.get_logger().info(f"PX4 VehicleOdometry publisher enabled on {self._px4_topic}")
        elif self._publish_px4_requested:
            self.px4_publisher = None
            self.get_logger().warn(
                "px4_msgs not importable; /fmu/in/vehicle_visual_odometry disabled")
        else:
            self.px4_publisher = None

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
        self._last_px4_position_ned = None
        self._last_px4_timestamp_ns = None

        self._stop = threading.Event()
        self._worker = threading.Thread(target=self._pipeline_loop, daemon=True)
        self._worker.start()

        mode = "stereo+IMU" if self._enable_imu_fusion else "stereo"
        self.get_logger().info(
            f"{node_name} started; opening OAK-D DepthAI pipeline for cuVSLAM ({mode})")

    # ---------- parameters ----------

    def _declare_params(self):
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 400)
        self.declare_parameter("camera_fps", 30.0)
        self.declare_parameter("rgb_fps", 15.0)
        self.declare_parameter("sync_threshold_ms", -1.0)
        self.declare_parameter("warmup_frames", 30)

        self.declare_parameter("publish_rgb", True)
        self.declare_parameter("publish_stereo_images", False)
        self.declare_parameter("publish_features", True)
        self.declare_parameter("feature_point_radius", 3)
        self.declare_parameter("publish_imu", True)
        self.declare_parameter("enable_imu_fusion", False)
        self.declare_parameter("imu_rate_hz", 200)

        self.declare_parameter("rectified_stereo_camera", False)
        self.declare_parameter("use_gpu", True)
        self.declare_parameter("async_sba", False)
        self.declare_parameter("enable_final_landmarks_export", False)
        self.declare_parameter("enable_observations_export", False)
        self.declare_parameter("verbosity", 2)

        self.declare_parameter("left_border_mask", [0, 0, 0, 0])
        self.declare_parameter("right_border_mask", [0, 0, 0, 0])

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

        self.declare_parameter("init_yaw_offset_deg", 0.0)
        self.declare_parameter("camera_mounting", "forward")
        self.declare_parameter("t_body_cam", [0.0, 0.0, 0.0])
        # Optional 3x3 (row-major, 9 values) camera-optical -> body FRD override.
        # Declared with dynamic typing and an empty default so it is always
        # initialized: a bare DOUBLE_ARRAY with a YAML "[]" cannot infer its
        # type and would stay uninitialized, which breaks the get_parameters
        # service used by Foxglove/rqt.
        self.declare_parameter(
            "R_body_cam_override", [], ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter("position_variance", [0.01, 0.01, 0.02])
        self.declare_parameter("orientation_variance", [0.01, 0.01, 0.02])
        self.declare_parameter("velocity_variance", [0.1, 0.1, 0.1])

        # Only used when enable_imu_fusion is true. Defaults are placeholders
        # for development; tune them from the target IMU datasheet/calibration.
        self.declare_parameter("imu_gyro_noise_density", 0.001)
        self.declare_parameter("imu_gyro_random_walk", 0.00001)
        self.declare_parameter("imu_accel_noise_density", 0.02)
        self.declare_parameter("imu_accel_random_walk", 0.0001)
        self.declare_parameter("imu_frequency_hz", 200.0)
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
        self._publish_stereo_images = bool(self.get_parameter("publish_stereo_images").value)
        self._publish_features = bool(self.get_parameter("publish_features").value)
        self._feature_point_radius = max(1, int(self.get_parameter("feature_point_radius").value))
        self._publish_imu = bool(self.get_parameter("publish_imu").value)
        self._enable_imu_fusion = bool(self.get_parameter("enable_imu_fusion").value)
        self._imu_rate_hz = int(self.get_parameter("imu_rate_hz").value)

        self._rectified_stereo_camera = bool(
            self.get_parameter("rectified_stereo_camera").value)
        self._use_gpu = bool(self.get_parameter("use_gpu").value)
        self._async_sba = bool(self.get_parameter("async_sba").value)
        self._enable_final_landmarks_export = bool(
            self.get_parameter("enable_final_landmarks_export").value)
        self._enable_observations_export = bool(
            self.get_parameter("enable_observations_export").value)
        self._verbosity = int(self.get_parameter("verbosity").value)

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
            self.get_parameter("rgb_camera_info_topic").value)
        self._left_image_topic = str(self.get_parameter("left_image_topic").value)
        self._right_image_topic = str(self.get_parameter("right_image_topic").value)
        self._left_camera_info_topic = str(
            self.get_parameter("left_camera_info_topic").value)
        self._right_camera_info_topic = str(
            self.get_parameter("right_camera_info_topic").value)
        self._features_image_topic = str(
            self.get_parameter("features_image_topic").value)
        self._features_camera_info_topic = str(
            self.get_parameter("features_camera_info_topic").value)
        self._px4_topic = str(self.get_parameter("px4_topic").value)

        self._t_body_cam = np.array(self._list_param("t_body_cam", 3, float), dtype=np.float64)

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
            math.radians(yaw_deg), self._R_body_cam)

        self._pos_var = np.array(
            self._list_param("position_variance", 3, float), dtype=np.float32)
        self._ori_var = np.array(
            self._list_param("orientation_variance", 3, float), dtype=np.float32)
        self._vel_var = np.array(
            self._list_param("velocity_variance", 3, float), dtype=np.float32)

    def _list_param(self, name: str, expected_len: int, item_type):
        value = list(self.get_parameter(name).value)
        if len(value) != expected_len:
            self.get_logger().warn(
                f"{name} must have {expected_len} elements; using zeros")
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
        if preset is None:
            self.get_logger().warn(
                f"Unknown camera_mounting '{mounting}'; expected one of "
                f"{sorted(presets)}. Falling back to 'forward'.")
            preset = frames.R_BODY_FROM_CAM_OPTICAL
        else:
            self.get_logger().info(f"Using '{mounting}' camera mounting")
        return preset.copy()

    # ---------- DepthAI + cuVSLAM pipeline ----------

    def _pipeline_loop(self):
        device = None
        try:
            print("[cuvslam] opening OAK-D device", flush=True)
            device = dai.Device()
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
                        print(f"[cuvslam] counts: stereo={n_stereo} rgb={n_rgb}",
                              flush=True)

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
            dai.CameraBoardSocket.CAM_B, sensorFps=self._camera_fps)
        print("[cuvslam] creating CAM_C (right mono)", flush=True)
        right = pipeline.create(dai.node.Camera).build(
            dai.CameraBoardSocket.CAM_C, sensorFps=self._camera_fps)
        return left, right

    def _create_stereo_sync_queue(self, pipeline, left, right):
        sync = pipeline.create(dai.node.Sync)
        sync.setSyncThreshold(self._sync_threshold)

        print("[cuvslam] linking left/right mono cameras to Sync", flush=True)
        left.requestOutput((self._width, self._height), type=dai.ImgFrame.Type.GRAY8).link(
            sync.inputs["left"])
        right.requestOutput((self._width, self._height), type=dai.ImgFrame.Type.GRAY8).link(
            sync.inputs["right"])

        return sync.out.createOutputQueue(maxSize=4, blocking=False)

    def _create_rgb_queue(self, pipeline):
        if not self._publish_rgb:
            return None

        print(f"[cuvslam] creating CAM_A (color) at {self._rgb_fps}fps", flush=True)
        color = pipeline.create(dai.node.Camera).build(
            dai.CameraBoardSocket.CAM_A, sensorFps=self._rgb_fps)
        return color.requestOutput((self._width, self._height)).createOutputQueue(
            maxSize=4, blocking=False)

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
        return imu.out.createOutputQueue(maxSize=30, blocking=False)

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
            rig.imus = [self._create_vslam_imu()]

        odometry_mode = (
            vslam.Tracker.OdometryMode.Inertial
            if self._enable_imu_fusion
            else vslam.Tracker.OdometryMode.Multicamera
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
        )

        try:
            vslam.set_verbosity(self._verbosity)
        except AttributeError:
            pass
        tracker = vslam.Tracker(rig, odom_config)
        return tracker

    def _create_vslam_camera(self, calibration, camera_socket, rig_socket, border_mask):
        intrinsics = calibration.getCameraIntrinsics(
            camera_socket, self._width, self._height)
        distortion = list(calibration.getDistortionCoefficients(camera_socket))[:8]
        if len(distortion) < 8:
            distortion += [0.0] * (8 - len(distortion))
        extrinsics = np.array(
            calibration.getCameraExtrinsics(camera_socket, rig_socket),
            dtype=np.float64,
        )

        camera = vslam.Camera()
        camera.distortion = vslam.Distortion(
            vslam.Distortion.Model.Polynomial, distortion)
        camera.focal = np.array([intrinsics[0][0], intrinsics[1][1]], dtype=np.float64)
        camera.principal = np.array([intrinsics[0][2], intrinsics[1][2]], dtype=np.float64)
        camera.size = np.array([self._width, self._height], dtype=np.int64)
        camera.rig_from_camera = self._pose_from_depthai_extrinsics(extrinsics)
        camera.border_top = border_mask[0]
        camera.border_bottom = border_mask[1]
        camera.border_left = border_mask[2]
        camera.border_right = border_mask[3]
        return camera

    def _create_vslam_imu(self):
        imu = vslam.ImuCalibration()
        imu.gyroscope_noise_density = float(
            self.get_parameter("imu_gyro_noise_density").value)
        imu.gyroscope_random_walk = float(
            self.get_parameter("imu_gyro_random_walk").value)
        imu.accelerometer_noise_density = float(
            self.get_parameter("imu_accel_noise_density").value)
        imu.accelerometer_random_walk = float(
            self.get_parameter("imu_accel_random_walk").value)
        imu.frequency = float(self.get_parameter("imu_frequency_hz").value)
        imu.rig_from_imu = vslam.Pose(
            rotation=np.array(
                self._list_param("rig_from_imu_rotation_xyzw", 4, float),
                dtype=np.float64,
            ),
            translation=np.array(
                self._list_param("rig_from_imu_translation", 3, float),
                dtype=np.float64,
            ),
        )
        return imu

    def _pose_from_depthai_extrinsics(self, extrinsics: np.ndarray):
        rotation = extrinsics[:3, :3]
        translation_m = extrinsics[:3, 3] * 0.01
        return vslam.Pose(
            rotation=frames.rot_to_quat_xyzw(rotation),
            translation=translation_m,
        )

    def _camera_info_from_calibration(self, calibration):
        self._left_camera_info = self._make_camera_info(
            calibration, dai.CameraBoardSocket.CAM_B, self._left_frame_id)
        self._right_camera_info = self._make_camera_info(
            calibration, dai.CameraBoardSocket.CAM_C, self._right_frame_id)
        self._rgb_camera_info = self._make_camera_info(
            calibration, dai.CameraBoardSocket.CAM_A, self._rig_frame_id)

    def _make_camera_info(self, calibration, camera_socket, frame_id):
        msg = CameraInfo()
        msg.width = self._width
        msg.height = self._height
        msg.header.frame_id = frame_id
        try:
            intrinsics = calibration.getCameraIntrinsics(
                camera_socket, self._width, self._height)
            distortion = list(calibration.getDistortionCoefficients(camera_socket))
            msg.distortion_model = "rational_polynomial" if len(distortion) > 5 else "plumb_bob"
            msg.d = distortion
            msg.k = [
                intrinsics[0][0], intrinsics[0][1], intrinsics[0][2],
                intrinsics[1][0], intrinsics[1][1], intrinsics[1][2],
                intrinsics[2][0], intrinsics[2][1], intrinsics[2][2],
            ]
            msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            msg.p = [
                intrinsics[0][0], intrinsics[0][1], intrinsics[0][2], 0.0,
                intrinsics[1][0], intrinsics[1][1], intrinsics[1][2], 0.0,
                intrinsics[2][0], intrinsics[2][1], intrinsics[2][2], 0.0,
            ]
        except Exception as exc:
            self.get_logger().warn(
                f"Could not read camera calibration for {frame_id}: {exc}")
        return msg

    def _warmup(self, sync_q):
        if self._warmup_frames <= 0:
            return

        print(f"[cuvslam] dropping {self._warmup_frames} warmup stereo frames", flush=True)
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

        if (self._last_cuvslam_timestamp_ns is not None
                and timestamp_ns <= self._last_cuvslam_timestamp_ns):
            self.get_logger().warn(
                "Dropping non-monotonic stereo frame timestamp from DepthAI")
            return

        self._register_imu_until(tracker, timestamp_ns)

        try:
            pose_estimate, _ = tracker.track(
                timestamp_ns,
                [left_frame.getCvFrame(), right_frame.getCvFrame()],
            )
        except Exception as exc:
            self.get_logger().error(f"cuVSLAM track() failed: {exc}")
            return

        self._last_cuvslam_timestamp_ns = timestamp_ns
        self._publish_stereo_debug_images(left_frame, right_frame)
        self._publish_feature_image(tracker, left_frame)

        if pose_estimate is None:
            return

        pose = self._extract_world_from_rig_pose(pose_estimate)
        if pose is None:
            return

        self._publish_pose(pose, timestamp_ns)

    def _extract_world_from_rig_pose(self, pose_estimate):
        world_from_rig = getattr(pose_estimate, "world_from_rig", None)
        if world_from_rig is None:
            return None
        return getattr(world_from_rig, "pose", world_from_rig)

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
                sample for sample in self._pending_imu
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
                measurement.linear_accelerations = sample.linear_acceleration
                measurement.angular_velocities = sample.angular_velocity
                tracker.register_imu_measurement(0, measurement)
            except Exception as exc:
                self.get_logger().warn(f"Failed to register cuVSLAM IMU sample: {exc}")
        self._pending_imu = keep

    def _publish_imu_sample(self, sample: ImuSample):
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

    def _publish_pose(self, pose: Any, sample_timestamp_ns: int):
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

        if self.px4_publisher is not None:
            self._publish_px4(translation, rotation, sample_timestamp_ns, now.nanoseconds)

    def _publish_px4(
        self,
        p_world_rig: np.ndarray,
        q_world_rig_xyzw: np.ndarray,
        sample_timestamp_ns: int,
        ros_timestamp_ns: int,
    ):
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

        if self._last_px4_position_ned is not None and self._last_px4_timestamp_ns is not None:
            dt = (sample_timestamp_ns - self._last_px4_timestamp_ns) * 1e-9
            if dt > 1e-4:
                v_ned = (p_ned_body - self._last_px4_position_ned) / dt
                velocity_frame = VehicleOdometry.VELOCITY_FRAME_NED
            else:
                v_ned = np.array([float("nan")] * 3)
                velocity_frame = VehicleOdometry.VELOCITY_FRAME_UNKNOWN
        else:
            v_ned = np.array([float("nan")] * 3)
            velocity_frame = VehicleOdometry.VELOCITY_FRAME_UNKNOWN

        self._last_px4_position_ned = p_ned_body
        self._last_px4_timestamp_ns = sample_timestamp_ns

        msg = VehicleOdometry()
        timestamp_us = ros_timestamp_ns // 1000
        msg.timestamp = int(timestamp_us)
        msg.timestamp_sample = int(timestamp_us)
        msg.pose_frame = VehicleOdometry.POSE_FRAME_NED
        msg.position = p_ned_body.astype(np.float32).tolist()
        msg.q = frames.rot_to_px4_quat(R_ned_body).tolist()
        msg.velocity_frame = velocity_frame
        msg.velocity = v_ned.astype(np.float32).tolist()
        msg.angular_velocity = [float("nan")] * 3
        msg.position_variance = self._pos_var.tolist()
        msg.orientation_variance = self._ori_var.tolist()
        msg.velocity_variance = self._vel_var.tolist()
        msg.reset_counter = 0
        msg.quality = 100
        self.px4_publisher.publish(msg)

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
                f"Could not read cuVSLAM observations: {exc}",
                throttle_duration_sec=5.0)

        overlay = cv2.cvtColor(left_frame.getCvFrame(), cv2.COLOR_GRAY2BGR)
        for obs in observations:
            center = (int(round(obs.u)), int(round(obs.v)))
            cv2.circle(
                overlay, center, self._feature_point_radius,
                self._feature_color(obs.id), thickness=-1, lineType=cv2.LINE_AA)

        stamp = self._now()
        msg = self.bridge.cv2_to_imgmsg(overlay, encoding="bgr8")
        msg.header.stamp = stamp
        msg.header.frame_id = self._left_frame_id
        self.features_image_publisher.publish(msg)

        if (self._left_camera_info is not None
                and self.features_camera_info_publisher is not None):
            self._left_camera_info.header = msg.header
            self.features_camera_info_publisher.publish(self._left_camera_info)

    def _publish_stereo_debug_images(self, left_frame, right_frame):
        if not self._publish_stereo_images:
            return

        stamp = self._now()
        left_msg = self.bridge.cv2_to_imgmsg(left_frame.getCvFrame(), encoding="mono8")
        left_msg.header.stamp = stamp
        left_msg.header.frame_id = self._left_frame_id
        self.left_image_publisher.publish(left_msg)

        right_msg = self.bridge.cv2_to_imgmsg(right_frame.getCvFrame(), encoding="mono8")
        right_msg.header.stamp = stamp
        right_msg.header.frame_id = self._right_frame_id
        self.right_image_publisher.publish(right_msg)

        if self._left_camera_info is not None:
            self._left_camera_info.header = left_msg.header
            self.left_camera_info_publisher.publish(self._left_camera_info)
        if self._right_camera_info is not None:
            self._right_camera_info.header = right_msg.header
            self.right_camera_info_publisher.publish(self._right_camera_info)

    _logged_rgb_shape = False

    def _publish_rgb_frame(self, frame):
        cv_img = frame.getCvFrame()
        if not CuVslamPublisherNode._logged_rgb_shape:
            print(f"[cuvslam] rgb cv_img shape={cv_img.shape} dtype={cv_img.dtype}",
                  flush=True)
            CuVslamPublisherNode._logged_rgb_shape = True

        msg = self.bridge.cv2_to_imgmsg(cv_img, encoding="bgr8")
        msg.header.stamp = self._now()
        msg.header.frame_id = self._rig_frame_id
        self.rgb_publisher.publish(msg)

        if self._rgb_camera_info is not None:
            self._rgb_camera_info.header = msg.header
            self.rgb_camera_info_publisher.publish(self._rgb_camera_info)

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
                    "DepthAI worker did not stop within 10s; shutting down anyway")
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
