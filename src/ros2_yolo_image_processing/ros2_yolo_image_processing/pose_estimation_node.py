import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default, qos_profile_sensor_data
from rclpy.executors import MultiThreadedExecutor

# MSG Libraries
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo

# Math Library
import numpy as np

# PX4 attitude is optional: only needed when use_attitude_correction is on.
try:
    from px4_msgs.msg import VehicleAttitude

    HAVE_PX4_MSGS = True
except ImportError:
    HAVE_PX4_MSGS = False


class PoseEstimationNode(Node):
    """
    ROS 2 Node which uses a YoloV8n DNN to find a drogue within a Camera Image ROS Topic.
    The monocular camera is the DFK 23UM021 which can do pose estimation from the camera
    To the middle of the coupler in the KC-130.
    """

    def __init__(self):
        """
        Default constructor where we set up the dual subscriber/publisher.
        :params:
            None
        :returns:
            None
        """
        super().__init__("dnn_ranging_node")

        # Camera intrinsics. Defaults come from the DFK 23UM021 calibration;
        # they are overridden by the first CameraInfo message if one arrives,
        # so with the OAK-D Lite driver publishing camera_info these are moot.
        self.fx = self.declare_parameter("fx", 721.0000).value
        self.fy = self.declare_parameter("fy", 722.3546).value
        self.cx = self.declare_parameter("cx", 499.2184).value
        self.cy = self.declare_parameter("cy", 387.2306).value
        self._intrinsics_from_camera_info = False

        # Physical widths of objects, in meters (REP-103). The published pose
        # is therefore in meters — the old node reported inches.
        self.OBJECT_WIDTHS_M = {
            "Coupler": 10.8 * 0.0254,
            "Drogue": 26.5 * 0.0254,
        }

        # Global PC Offset (adjust if needed)
        self.PC = np.array([0.0, 0.0, 0.0], dtype=float)

        # YOLO class names (must match detection node)
        self.CLASS_COUPLER = "Coupler"
        self.CLASS_DROGUE = "Drogue"

        # Optional roll/pitch/yaw correction from PX4: rotates the camera-frame
        # estimate into a level, north-referenced frame using
        # /fmu/out/vehicle_attitude. Off by default (pure camera frame).
        self.use_attitude_correction = self.declare_parameter(
            "use_attitude_correction", False
        ).value
        self.roll_rad = 0.0
        self.pitch_rad = 0.0
        self.yaw_rad = 0.0
        self.have_attitude = False

        # Create a valid Quality of Service
        default_qos = qos_profile_system_default
        # Create our Subscription to the array of Bounding Boxes
        self.bbox_subscriber = self.create_subscription(
            Detection2DArray,
            "detections",
            self.detections_callback,
            qos_profile=default_qos,
        )
        # Real intrinsics from the camera driver override the defaults above
        self.camera_info_subscriber = self.create_subscription(
            CameraInfo,
            "camera_info",
            self.camera_info_callback,
            qos_profile=qos_profile_sensor_data,
        )
        if self.use_attitude_correction:
            if HAVE_PX4_MSGS:
                self.attitude_subscriber = self.create_subscription(
                    VehicleAttitude,
                    "/fmu/out/vehicle_attitude",
                    self.vehicle_attitude_callback,
                    qos_profile_sensor_data,
                )
            else:
                self.use_attitude_correction = False
                self.get_logger().warn(
                    "use_attitude_correction requested but px4_msgs is not "
                    "importable; publishing uncorrected camera-frame poses"
                )
        # Create our Publisher for the pose
        self.pose_publisher = self.create_publisher(
            PoseStamped, "tag_detections", qos_profile=default_qos
        )

        # Log it
        self.get_logger().info("Created Pose Estimation Node (units: meters)")

    def camera_info_callback(self, msg):
        """
        Adopt the camera driver's calibrated intrinsics (once).
        """
        if self._intrinsics_from_camera_info:
            return
        k = msg.k
        if k[0] <= 0.0:
            return
        self.fx, self.fy = float(k[0]), float(k[4])
        self.cx, self.cy = float(k[2]), float(k[5])
        self._intrinsics_from_camera_info = True
        self.get_logger().info(
            f"Intrinsics from camera_info: fx={self.fx:.1f} fy={self.fy:.1f} "
            f"cx={self.cx:.1f} cy={self.cy:.1f} ({msg.width}x{msg.height})"
        )

    def vehicle_attitude_callback(self, msg):
        """
        Convert the PX4 attitude quaternion (w, x, y, z) to roll/pitch/yaw.
        """
        q0, q1, q2, q3 = msg.q

        sinr_cosp = 2.0 * (q0 * q1 + q2 * q3)
        cosr_cosp = 1.0 - 2.0 * (q1 * q1 + q2 * q2)
        self.roll_rad = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (q0 * q2 - q3 * q1)
        if abs(sinp) >= 1.0:
            self.pitch_rad = math.copysign(math.pi / 2.0, sinp)
        else:
            self.pitch_rad = math.asin(sinp)

        siny_cosp = 2.0 * (q0 * q3 + q1 * q2)
        cosy_cosp = 1.0 - 2.0 * (q2 * q2 + q3 * q3)
        self.yaw_rad = math.atan2(siny_cosp, cosy_cosp)

        self.have_attitude = True

    def apply_attitude_correction(self, rng4):
        """
        Rotate a ranging-frame vector (+x left, +y up, +z forward) through the
        vehicle attitude into a level NED-aligned frame, then express it back
        in the same ranging-frame convention.
        """
        x_left, y_up, z_forward = rng4[:3]
        # Ranging frame -> body FRD (+x forward, +y right, +z down)
        p_body = np.array([z_forward, -x_left, -y_up], dtype=float)

        cr, sr = np.cos(self.roll_rad), np.sin(self.roll_rad)
        cp, sp = np.cos(self.pitch_rad), np.sin(self.pitch_rad)
        cy, sy = np.cos(self.yaw_rad), np.sin(self.yaw_rad)
        rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
        ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
        rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
        p_ned = rz @ ry @ rx @ p_body

        north, east, down = p_ned
        return [
            float(-east),
            float(-down),
            float(north),
            float(np.linalg.norm(p_ned)),
        ]

    def ranging_function(self, x1c, y1c, x2c, y2c, class_name):
        """
        Uses bounding box size and camera intrinsics to estimate range.

        Output frame (meters):
            +x = left
            +y = up
            +z = forward
            [3] = 3D line-of-sight distance
        """
        object_width = self.OBJECT_WIDTHS_M.get(class_name, 10.8 * 0.0254)

        xd = abs(x2c - x1c)
        yd = abs(y2c - y1c)
        if xd <= 1 or yd <= 1:
            return [0.0, 0.0, 0.0, 0.0]

        object_center_x = x1c + xd / 2.0
        object_center_y = y1c + yd / 2.0

        # Offsets from the calibrated principal point, not the frame center
        CenterErrorx = self.cx - object_center_x
        CenterErrory = self.cy - object_center_y

        StraightDistancex = (self.fx / xd) * object_width
        StraightDistancey = (self.fy / yd) * object_width

        xErrorWeight = abs(CenterErrorx) / self.cx
        yErrorWeight = abs(CenterErrory) / self.cy

        totalError = xErrorWeight + yErrorWeight
        if totalError == 0:
            # Dead-center target: both axes equally trustworthy
            WeightedDistance = (StraightDistancex + StraightDistancey) / 2.0
        else:
            maxErrorSide = max(xErrorWeight, yErrorWeight)
            ErrorWeightRatio = maxErrorSide / totalError

            if xErrorWeight > yErrorWeight:
                WeightDistancex = StraightDistancex * (1.0 - ErrorWeightRatio)
                WeightDistancey = StraightDistancey * ErrorWeightRatio
            elif xErrorWeight < yErrorWeight:
                WeightDistancex = StraightDistancex * ErrorWeightRatio
                WeightDistancey = StraightDistancey * (1.0 - ErrorWeightRatio)
            else:
                WeightDistancex = StraightDistancex * 0.5
                WeightDistancey = StraightDistancey * 0.5

            WeightedDistance = WeightDistancex + WeightDistancey

        OffsetDistancex = WeightedDistance * CenterErrorx / self.fx
        OffsetDistancey = WeightedDistance * CenterErrory / self.fy

        CombinedOffset = np.hypot(OffsetDistancex, OffsetDistancey)
        FinalDistance = np.hypot(WeightedDistance, CombinedOffset)

        return [
            float(OffsetDistancex),
            float(OffsetDistancey),
            float(WeightedDistance),
            float(FinalDistance),
        ]

    def apply_pc_and_norm(self, rng4, pc=None):
        # Adjust if needed
        if pc is None:
            pc = self.PC
        out = list(rng4)
        v = np.array(out[:3], dtype=float) + pc
        out[:3] = v.tolist()
        out[3] = float(np.linalg.norm(v))
        return out

    def pick_best_by_class(self, detections, class_name):
        """
        Find the highest-confidence detection of a specific class.

        :params:
            detections: List of Detection2D messages.
            class_name: Class name to search for.
        :returns:
            Detection2D or None
        """
        best_det = None
        best_score = -1.0

        for det in detections:
            if len(det.results) > 0:
                hypothesis = det.results[0].hypothesis
                if hypothesis.class_id == class_name and hypothesis.score > best_score:
                    best_score = hypothesis.score
                    best_det = det

        return best_det

    def extract_bbox_corners(self, detection):
        """
        Extract corner coordinates from Detection2D bbox.

        :params:
            detection: Detection2D message
        :returns:
            (x1, y1, x2, y2)
        """
        cx = detection.bbox.center.position.x
        cy = detection.bbox.center.position.y
        w = detection.bbox.size_x
        h = detection.bbox.size_y

        x1 = cx - w / 2
        y1 = cy - h / 2
        x2 = cx + w / 2
        y2 = cy + h / 2

        return x1, y1, x2, y2

    def detections_callback(self, detections_msg):
        """
        Process detections and publish pose estimate.

        :param detections_msg: Detection2DArray message
        """
        # Find best coupler and drogue using pick_best_by_class
        coupler_det = self.pick_best_by_class(
            detections_msg.detections, self.CLASS_COUPLER
        )
        drogue_det = self.pick_best_by_class(
            detections_msg.detections, self.CLASS_DROGUE
        )

        # Initialize outputs
        coupler_out = [0.0, 0.0, 0.0, 0.0]
        drogue_out = [0.0, 0.0, 0.0, 0.0]

        # Calculate ranging for coupler using ranging_function
        if coupler_det is not None:
            x1c, y1c, x2c, y2c = self.extract_bbox_corners(coupler_det)
            coupler_out = self.ranging_function(
                x1c, y1c, x2c, y2c, self.CLASS_COUPLER
            )
            if self.use_attitude_correction and self.have_attitude:
                coupler_out = self.apply_attitude_correction(coupler_out)
            coupler_out = self.apply_pc_and_norm(coupler_out)
            self.get_logger().debug(
                f"Coupler: x={coupler_out[0]:.2f}, y={coupler_out[1]:.2f}, "
                f"z={coupler_out[2]:.2f}, dist={coupler_out[3]:.2f}"
            )

        # Calculate ranging for drogue using ranging_function
        if drogue_det is not None:
            x1d, y1d, x2d, y2d = self.extract_bbox_corners(drogue_det)
            drogue_out = self.ranging_function(
                x1d, y1d, x2d, y2d, self.CLASS_DROGUE
            )
            if self.use_attitude_correction and self.have_attitude:
                drogue_out = self.apply_attitude_correction(drogue_out)
            drogue_out = self.apply_pc_and_norm(drogue_out)
            self.get_logger().debug(
                f"Drogue: x={drogue_out[0]:.2f}, y={drogue_out[1]:.2f}, "
                f"z={drogue_out[2]:.2f}, dist={drogue_out[3]:.2f}"
            )

        # Same output shape as your original Snapshot(): 8 values
        snapshot_out = coupler_out + drogue_out
        self.get_logger().debug(f"RANGE: {snapshot_out}")

        # Decide which to use for pose (prefer drogue if available)
        if drogue_det is not None:
            pose_data = drogue_out
            target = "drogue"
        elif coupler_det is not None:
            pose_data = coupler_out
            target = "coupler"
        else:
            self.get_logger().warn("No drogue or coupler detected")
            return

        # Create and publish pose message
        pose_msg = PoseStamped()
        pose_msg.header = detections_msg.header

        # Position (offset_x, offset_y, weighted_distance)
        pose_msg.pose.position.x = pose_data[0]
        pose_msg.pose.position.y = pose_data[1]
        pose_msg.pose.position.z = pose_data[2]

        # TODO: Orientation if you need IMU fusing
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = 0.0
        pose_msg.pose.orientation.w = 1.0

        self.pose_publisher.publish(pose_msg)

        self.get_logger().debug(
            f"Published pose to {target}: "
            f"[{pose_data[0]:.2f}, {pose_data[1]:.2f}, {pose_data[2]:.2f}] "
            f"dist={pose_data[3]:.2f}"
        )


def main(args=None):
    rclpy.init(args=args)
    # Create our Node
    node = PoseEstimationNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    # Continously Run Our Node
    executor.spin()

    # Clean up
    rclpy.shutdown()


if __name__ == "__main__":
    main()
