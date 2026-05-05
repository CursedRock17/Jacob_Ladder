"""OAK-D BasaltVIO publisher with PX4 visual odometry output.

Same as vo_publisher_node, plus publishes
  /fmu/in/vehicle_visual_odometry  px4_msgs/VehicleOdometry
for PX4 EKF2 fusion over uXRCE-DDS.

Frame chain:
  BasaltVIO world (FLU: X-fwd, Y-left, Z-up at startup)
    -- init_yaw_offset_deg -->
  NED world (X-N, Y-E, Z-D)

  Camera optical (X-right, Y-down, Z-fwd, standard CV)
    -- R_body_cam (default = canonical optical->FRD swap) -->
  Body FRD (X-fwd, Y-right, Z-down)

  Camera origin --> body origin via lever arm t_body_cam.

px4_msgs is imported conditionally; if it's not in the workspace overlay
this node still publishes /slam/odometry, /tf, /imu/data, /rgb/image and
just skips the PX4 message.
"""

import math
import threading
import time

import depthai as dai
import numpy as np

from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import CameraInfo, Image, Imu
from tf2_ros import TransformBroadcaster

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from . import frames

try:
    from px4_msgs.msg import VehicleOdometry
    PX4_AVAILABLE = True
except ImportError:
    PX4_AVAILABLE = False

PUBLISHER_QUEUE_SIZE = 10

# uXRCE-DDS uses BEST_EFFORT + KEEP_LAST(5) for PX4 uORB topics.
PX4_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)


class VoPublisherPx4Node(Node):
    def __init__(self):
        super().__init__('oak_d_vo_px4_publisher')

        self._declare_params()
        self._load_params()

        self.odometry_publisher = self.create_publisher(
            PoseStamped, "/slam/odometry", PUBLISHER_QUEUE_SIZE)
        self.imu_publisher = self.create_publisher(
            Imu, "/imu/data", PUBLISHER_QUEUE_SIZE)
        self.image_publisher = self.create_publisher(
            Image, "/rgb/image", PUBLISHER_QUEUE_SIZE)
        self.camera_info_publisher = self.create_publisher(
            CameraInfo, "/rgb/camera_info", PUBLISHER_QUEUE_SIZE)
        self.tf_broadcaster = TransformBroadcaster(self)

        if PX4_AVAILABLE:
            self.px4_publisher = self.create_publisher(
                VehicleOdometry, "/fmu/in/vehicle_visual_odometry", PX4_QOS)
            self.get_logger().info("PX4 VehicleOdometry publisher enabled")
        else:
            self.px4_publisher = None
            self.get_logger().warn(
                "px4_msgs not importable; /fmu/in/vehicle_visual_odometry disabled")

        self.bridge = CvBridge()
        self._camera_info = None

        # State for finite-difference velocity in NED.
        self._prev_p_ned = None
        self._prev_t_ns = None

        self._stop = threading.Event()
        self._worker = threading.Thread(target=self._pipeline_loop, daemon=True)
        self._worker.start()
        self.get_logger().info('oak_d_vo_px4_publisher started; opening DepthAI pipeline')

    # ---------- parameters ----------

    def _declare_params(self):
        self.declare_parameter('init_yaw_offset_deg', 0.0)
        self.declare_parameter('t_body_cam', [0.0, 0.0, 0.0])
        self.declare_parameter('R_body_cam_override', [])
        self.declare_parameter('position_variance', [0.01, 0.01, 0.02])
        self.declare_parameter('orientation_variance', [0.01, 0.01, 0.02])
        self.declare_parameter('velocity_variance', [0.1, 0.1, 0.1])

    def _load_params(self):
        yaw_deg = self.get_parameter('init_yaw_offset_deg').get_parameter_value().double_value
        self._R_ned_world = frames.r_ned_from_basalt_world(math.radians(yaw_deg))

        self._t_body_cam = np.array(
            self.get_parameter('t_body_cam').get_parameter_value().double_array_value,
            dtype=np.float64)
        if self._t_body_cam.size != 3:
            self.get_logger().warn('t_body_cam must have 3 elements; defaulting to zero')
            self._t_body_cam = np.zeros(3)

        override = list(self.get_parameter('R_body_cam_override')
                        .get_parameter_value().double_array_value)
        if len(override) == 9:
            self._R_body_cam = np.array(override, dtype=np.float64).reshape(3, 3)
        else:
            self._R_body_cam = frames.R_BODY_FROM_CAM_OPTICAL.copy()

        self._pos_var = np.array(
            self.get_parameter('position_variance').get_parameter_value().double_array_value,
            dtype=np.float32)
        self._ori_var = np.array(
            self.get_parameter('orientation_variance').get_parameter_value().double_array_value,
            dtype=np.float32)
        self._vel_var = np.array(
            self.get_parameter('velocity_variance').get_parameter_value().double_array_value,
            dtype=np.float32)

    # ---------- DepthAI pipeline ----------

    def _pipeline_loop(self):
        try:
            print("[vo_px4] entering dai.Pipeline()", flush=True)
            with dai.Pipeline() as p:
                fps = 60
                width = 640
                height = 400

                print("[vo_px4] creating CAM_B (left mono)", flush=True)
                left = p.create(dai.node.Camera).build(
                    dai.CameraBoardSocket.CAM_B, sensorFps=fps)
                print("[vo_px4] creating CAM_C (right mono)", flush=True)
                right = p.create(dai.node.Camera).build(
                    dai.CameraBoardSocket.CAM_C, sensorFps=fps)
                color_fps = 30
                print(f"[vo_px4] creating CAM_A (color) at {color_fps}fps", flush=True)
                color = p.create(dai.node.Camera).build(
                    dai.CameraBoardSocket.CAM_A, sensorFps=color_fps)
                print("[vo_px4] creating IMU", flush=True)
                imu = p.create(dai.node.IMU)
                print("[vo_px4] creating BasaltVIO", flush=True)
                odom = p.create(dai.node.BasaltVIO)

                imu.enableIMUSensor(
                    [dai.IMUSensor.ACCELEROMETER_RAW, dai.IMUSensor.GYROSCOPE_RAW], 200)
                imu.setBatchReportThreshold(1)
                imu.setMaxBatchReports(10)

                print("[vo_px4] linking left/right/imu -> BasaltVIO", flush=True)
                left.requestOutput((width, height)).link(odom.left)
                right.requestOutput((width, height)).link(odom.right)
                imu.out.link(odom.imu)

                print("[vo_px4] creating output queues", flush=True)
                imu_q = imu.out.createOutputQueue(maxSize=20, blocking=False)
                transform_q = odom.transform.createOutputQueue(maxSize=10, blocking=False)
                rgb_q = color.requestOutput((width, height)).createOutputQueue(
                    maxSize=4, blocking=False)

                print("[vo_px4] calling p.start()", flush=True)
                p.start()
                print("[vo_px4] pipeline running, entering drain loop", flush=True)

                self._camera_info = self._read_camera_info(p, width, height)

                n_imu = 0
                n_pose = 0
                n_rgb = 0
                while p.isRunning() and not self._stop.is_set():
                    imu_packet = imu_q.tryGet()
                    if imu_packet is not None:
                        self._publish_imu(imu_packet)
                        n_imu += 1

                    trans = transform_q.tryGet()
                    if trans is not None:
                        self._publish_pose(trans)
                        n_pose += 1

                    frame = rgb_q.tryGet()
                    if frame is not None:
                        self._publish_image(frame)
                        n_rgb += 1

                    total = n_imu + n_pose + n_rgb
                    if total > 0 and total % 500 == 0:
                        print(f"[vo_px4] counts: imu={n_imu} pose={n_pose} rgb={n_rgb}",
                              flush=True)

                    time.sleep(0.001)
        except Exception as e:
            print(f"[vo_px4] pipeline EXCEPTION: {e!r}", flush=True)
            self.get_logger().error(f"DepthAI pipeline failed: {e}")

    # ---------- publishers ----------

    def _read_camera_info(self, pipeline, width, height):
        try:
            calib = pipeline.getDefaultDevice().readCalibration()
            K = calib.getCameraIntrinsics(dai.CameraBoardSocket.CAM_A, width, height)
            d = list(calib.getDistortionCoefficients(dai.CameraBoardSocket.CAM_A))
            msg = CameraInfo()
            msg.width = width
            msg.height = height
            msg.distortion_model = "rational_polynomial" if len(d) > 5 else "plumb_bob"
            msg.d = d
            msg.k = [K[0][0], K[0][1], K[0][2],
                     K[1][0], K[1][1], K[1][2],
                     K[2][0], K[2][1], K[2][2]]
            msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            msg.p = [K[0][0], K[0][1], K[0][2], 0.0,
                     K[1][0], K[1][1], K[1][2], 0.0,
                     K[2][0], K[2][1], K[2][2], 0.0]
            print(f"[vo_px4] calibration: "
                  f"fx={K[0][0]:.1f} fy={K[1][1]:.1f} "
                  f"cx={K[0][2]:.1f} cy={K[1][2]:.1f} "
                  f"d={len(d)} coeffs", flush=True)
            return msg
        except Exception as e:
            print(f"[vo_px4] WARNING: calibration read failed ({e!r}); "
                  f"camera_info will have zero intrinsics", flush=True)
            msg = CameraInfo()
            msg.width = width
            msg.height = height
            return msg

    def _now(self):
        return self.get_clock().now().to_msg()

    def _publish_imu(self, imu_data):
        for packet in imu_data.packets:
            accel = packet.acceleroMeter
            gyro = packet.gyroscope
            msg = Imu()
            msg.header.stamp = self._now()
            msg.header.frame_id = "oak_imu"
            msg.linear_acceleration.x = float(accel.x)
            msg.linear_acceleration.y = float(accel.y)
            msg.linear_acceleration.z = float(accel.z)
            msg.angular_velocity.x = float(gyro.x)
            msg.angular_velocity.y = float(gyro.y)
            msg.angular_velocity.z = float(gyro.z)
            msg.orientation_covariance[0] = -1.0
            self.imu_publisher.publish(msg)

    def _publish_pose(self, transform_data):
        t = transform_data.getTranslation()
        q = transform_data.getQuaternion()
        now = self.get_clock().now()
        stamp = now.to_msg()
        stamp_ns = now.nanoseconds

        # ----- visualization: PoseStamped in BasaltVIO world frame -----
        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = "world"
        pose_msg.pose.position.x = float(t.x)
        pose_msg.pose.position.y = float(t.y)
        pose_msg.pose.position.z = float(t.z)
        pose_msg.pose.orientation.x = float(q.qx)
        pose_msg.pose.orientation.y = float(q.qy)
        pose_msg.pose.orientation.z = float(q.qz)
        pose_msg.pose.orientation.w = float(q.qw)
        self.odometry_publisher.publish(pose_msg)

        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = "world"
        tf_msg.child_frame_id = "oak_camera"
        tf_msg.transform.translation.x = float(t.x)
        tf_msg.transform.translation.y = float(t.y)
        tf_msg.transform.translation.z = float(t.z)
        tf_msg.transform.rotation.x = float(q.qx)
        tf_msg.transform.rotation.y = float(q.qy)
        tf_msg.transform.rotation.z = float(q.qz)
        tf_msg.transform.rotation.w = float(q.qw)
        self.tf_broadcaster.sendTransform(tf_msg)

        # ----- PX4 VehicleOdometry in NED body frame -----
        if self.px4_publisher is not None:
            self._publish_px4(t, q, stamp_ns)

    def _publish_px4(self, t, q, stamp_ns):
        p_world_cam = np.array([t.x, t.y, t.z], dtype=np.float64)
        R_world_cam = frames.quat_to_rot(q.qx, q.qy, q.qz, q.qw)

        p_ned_body, R_ned_body = frames.transform_pose(
            p_world_cam, R_world_cam,
            self._R_body_cam, self._t_body_cam,
            self._R_ned_world,
        )

        # Finite-difference velocity in NED. Set velocity_variance high enough
        # in params to tell EKF2 not to over-trust this.
        if self._prev_p_ned is not None and self._prev_t_ns is not None:
            dt = (stamp_ns - self._prev_t_ns) * 1e-9
            if dt > 1e-4:
                v_ned = (p_ned_body - self._prev_p_ned) / dt
                vel_frame = VehicleOdometry.VELOCITY_FRAME_NED
            else:
                v_ned = np.array([float('nan')] * 3)
                vel_frame = VehicleOdometry.VELOCITY_FRAME_UNKNOWN
        else:
            v_ned = np.array([float('nan')] * 3)
            vel_frame = VehicleOdometry.VELOCITY_FRAME_UNKNOWN
        self._prev_p_ned = p_ned_body
        self._prev_t_ns = stamp_ns

        msg = VehicleOdometry()
        # PX4 expects microsecond timestamps. timestamp_sample is when the
        # measurement was taken; without device timestamp we use host time
        # for both and rely on EKF2_EV_DELAY for any constant offset.
        ts_us = stamp_ns // 1000
        msg.timestamp = int(ts_us)
        msg.timestamp_sample = int(ts_us)
        msg.pose_frame = VehicleOdometry.POSE_FRAME_NED
        msg.position = p_ned_body.astype(np.float32).tolist()
        msg.q = frames.rot_to_px4_quat(R_ned_body).tolist()
        msg.velocity_frame = vel_frame
        msg.velocity = v_ned.astype(np.float32).tolist()
        # BasaltVIO doesn't expose body-frame angular velocity; report NaN so
        # EKF2 falls back to its IMU-based estimate.
        msg.angular_velocity = [float('nan')] * 3
        msg.position_variance = self._pos_var.tolist()
        msg.orientation_variance = self._ori_var.tolist()
        msg.velocity_variance = self._vel_var.tolist()
        msg.reset_counter = 0
        msg.quality = 100
        self.px4_publisher.publish(msg)

    _logged_image_shape = False

    def _publish_image(self, frame):
        cv_img = frame.getCvFrame()
        if not VoPublisherPx4Node._logged_image_shape:
            print(f"[vo_px4] cv_img shape={cv_img.shape} dtype={cv_img.dtype}",
                  flush=True)
            VoPublisherPx4Node._logged_image_shape = True
        msg = self.bridge.cv2_to_imgmsg(cv_img, encoding="bgr8")
        msg.header.stamp = self._now()
        msg.header.frame_id = "oak_camera"
        self.image_publisher.publish(msg)

        if self._camera_info is not None:
            self._camera_info.header = msg.header
            self.camera_info_publisher.publish(self._camera_info)

    def destroy_node(self):
        self._stop.set()
        if self._worker.is_alive():
            self._worker.join(timeout=2.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VoPublisherPx4Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
