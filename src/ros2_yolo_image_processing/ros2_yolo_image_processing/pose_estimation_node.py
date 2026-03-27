import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_default
from rclpy.executors import MultiThreadedExecutor

# MSG Libraries
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import PoseStamped

# Math Library
import numpy as np


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
        super().__init__('dnn_ranging_node')

        self.FRAME_W = 1024
        self.FRAME_H = 768
        self.FOCAL_LENGTH = 721.7273

        # Physical widths (inches) of objects
        self.COUPLER_WIDTH = 10.8
        self.DROGUE_WIDTH = 26.5

        # Global PC Offset (adjust if needed)
        self.PC = np.array([0.0, 0.0, 0.0], dtype=float)

        # YOLO class names (must match detection node)
        self.CLASS_COUPLER = "Coupler"
        self.CLASS_DROGUE = "Drogue"

        # Create a valid Quality of Service
        default_qos = qos_profile_default
        # Create our Subscription to the array of Bounding Boxes
        self.bbox_subscriber = self.create_subscription(
            Detection2DArray, 'detections', self.detections_callback, qos_profile=default_qos)
        # Create our Publisher for the pose
        self.pose_publisher = self.create_publisher(
            PoseStamped, 'tag_dectections', qos_profile=default_qos)

        # Log it
        self.get_logger().info("Created Pose Estimation Node")

    def ranging_function(self, x1c, y1c, x2c, y2c, n, frame_w=None, frame_h=None):
        """
        Returns [OffsetDistancex, OffsetDistancey, WeightedDistance, FinalDistance]
        """
        if frame_w is None:
            frame_w = self.FRAME_W
        if frame_h is None:
            frame_h = self.FRAME_H

        FocalLength = 721.7273
        CouplerWidths = {1: 10.8, 2: 26.5}  # n=1 coupler, n=2 drogue
        CouplerWidth = CouplerWidths.get(n, 10.8)

        xd = abs(x2c - x1c)
        yd = abs(y2c - y1c)
        if xd <= 1 or yd <= 1:
            return [0.0, 0.0, 0.0, 0.0]

        droguex = x1c + xd / 2.0
        droguey = y1c + yd / 2.0

        vidcenterx = frame_w / 2.0
        vidcentery = frame_h / 2.0

        StraightDistancex = (FocalLength / xd) * CouplerWidth
        StraightDistancey = (FocalLength / yd) * CouplerWidth

        CenterErrorx = vidcenterx - droguex
        CenterErrory = vidcentery - droguey

        xErrorWeight = abs(CenterErrorx) / vidcenterx
        yErrorWeight = abs(CenterErrory) / vidcentery

        totalError = xErrorWeight + yErrorWeight
        if totalError == 0:
            return [0.0, 0.0, 0.0, 0.0]

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

        if abs(CenterErrorx) < 1e-9 or abs(CenterErrory) < 1e-9:
            return [0.0, 0.0, float(WeightedDistance), float(WeightedDistance)]

        OffsetDistancex = WeightedDistance / (FocalLength / CenterErrorx)
        OffsetDistancey = WeightedDistance / (FocalLength / CenterErrory)

        CombinedOffset = np.hypot(OffsetDistancex, OffsetDistancey)
        FinalDistance = np.hypot(WeightedDistance, CombinedOffset)

        return [float(OffsetDistancex), float(OffsetDistancey), float(WeightedDistance), float(FinalDistance)]

    def apply_pc_and_norm(self, rng4, pc=None):
        # Adjust if needed
        if pc is None:
            pc = self.pc
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
            detections_msg.detections, self.CLASS_COUPLER)
        drogue_det = self.pick_best_by_class(
            detections_msg.detections, self.CLASS_DROGUE)

        # Initialize outputs
        coupler_out = [0.0, 0.0, 0.0, 0.0]
        drogue_out = [0.0, 0.0, 0.0, 0.0]

        # Calculate ranging for coupler using ranging_function
        if coupler_det is not None:
            x1c, y1c, x2c, y2c = self.extract_bbox_corners(coupler_det)
            coupler_out = self.ranging_function(x1c, y1c, x2c, y2c, n=1)
            coupler_out = self.apply_pc_and_norm(coupler_out)
            self.get_logger().debug(
                f"Coupler: x={coupler_out[0]:.2f}, y={coupler_out[1]:.2f}, "
                f"z={coupler_out[2]:.2f}, dist={coupler_out[3]:.2f}")

        # Calculate ranging for drogue using ranging_function
        if drogue_det is not None:
            x1d, y1d, x2d, y2d = self.extract_bbox_corners(drogue_det)
            drogue_out = self.ranging_function(x1d, y1d, x2d, y2d, n=2)
            drogue_out = self.apply_pc_and_norm(drogue_out)
            self.get_logger().debug(
                f"Drogue: x={drogue_out[0]:.2f}, y={drogue_out[1]:.2f}, "
                f"z={drogue_out[2]:.2f}, dist={drogue_out[3]:.2f}")

        # Same output shape as your original Snapshot(): 8 values
        snapshot_out = coupler_out + drogue_out
        self.get_logger().info(f"RANGE: {snapshot_out}")

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

        self.get_logger().info(
            f"Published pose to {target}: "
            f"[{pose_data[0]:.2f}, {pose_data[1]:.2f}, {pose_data[2]:.2f}] "
            f"dist={pose_data[3]:.2f}")


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


if __name__ == '__main__':
    main()
