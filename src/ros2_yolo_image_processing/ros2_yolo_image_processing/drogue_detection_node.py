import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_default, qos_profile_sensor_data

# MSG Libraries
from sensor_msgs.msg import Image
from vision_msgs.msg import VisionInfo, Detection2D, Detection2DArray, ObjectHypothesisWithPose

# OpenCV/Image Processing Libraries
import numpy as np
import cv2
from cv_bridge import CvBridge
from ultralytics import YOLO

# GLOBALS ##
# Your stream/frame size (used by ranging center math)
FRAME_W = 1024
FRAME_H = 768
# Global PC offset (adjust if needed)
PC = np.array([0.0, 0.0, 0.0], dtype=float)


class DrogueDetectionNode(Node):
    """
    ROS 2 Node which uses a DNN to find a Drogue within a Camera Image ROS Topic
    and publish bounding box to the drogue (if one).
    """

    def __init__(self):
        """
        Default constructor where we set up the dual subscriber/publisher.
        :params:
            None
        :returns:
            None
        """
        super().__init__('drogue_detection_node')

        # Find our local Pytorch Neural Network, then establish the tensors
        self.MODEL_FILEPATH = './models/ranging_DNN_v8n.pt'
        self.IMGSZ = 416
        self.CONF = 0.25
        self.IOU = 0.45
        self.DEVICE = "cpu"  # set to "0" if GPU works

        # Frame Info (Timing) + Logger
        self.frame_idx = 0
        self.t_last = self.get_clock().now()
        self.get_logger().info(f"Loading Model from {self.MODEL_FILEPATH}")

        # IMPORTANT: set these to match your YOLO class IDs
        self.CLASS_COUPLER = 0
        self.CLASS_DROGUE = 1
        # For Viewing Purposes
        self.CLASS_NAMES = {0: "Coupler", 1: "Drogue"}

        # Establish our Model
        self.dnn_model = YOLO(self.MODEL_FILEPATH)

        # Create a valid Quality of Service
        sensor_qos = qos_profile_sensor_data
        default_qos = qos_profile_default

        # Create ROS 2 Parameters to handle some requirements on finding the drogue
        self.confidence_param = self.declare_parameter('min_score_thresh', self.CONF)
        self.iou_thresh_param = self.declare_parameter('iou_thresh', self.IOU)

        # Create our Subscription to the camera feed.
        self.img_subscriber = self.create_subscription(
            Image, 'image_raw', self.image_callback, qos_profile=sensor_qos)
        self.opencv_bridge = CvBridge()

        # Create a Publisher for Information about our CV Pipeline
        self.vision_info_pub = self.create_publisher(
            VisionInfo, 'vision_info', qos_profile=default_qos)
        # Create a publisher for the detected drogue on screen.
        self.detection_publisher = self.create_publisher(
            Detection2DArray, 'detections', qos_profile=default_qos)
        # Create a publisher which provides a new image with a bounding box around it.
        self.detected_image_publisher = self.create_publisher(
            Image, 'detections_image', qos_profile=default_qos)

        # Adverstise our CV Info
        self.publish_network_info(model_description="dnn", model_path=self.MODEL_FILEPATH)
        self.get_logger().info("Created Drogue Detection Node")

    def publish_network_info(self, model_description, model_path):
        """
        Simple publish function which locally advertises information about
        our CV Pipeline. *It's good practice
        :params:
            model_description: Formatting of our DNN Model.
            model_path: Local Filepath of our DNN Model.
        :returns:
            None
        """
        vision_info_msg = VisionInfo()
        vision_info_msg.method = model_description
        vision_info_msg.database_location = model_path
        self.vision_info_pub.publish(vision_info_msg)

    def process_image(self, img_msg):
        """
        Runs the DNN onboard to get the correct detection output
        :params:
            img_msg: ROS 2 Image MSG from the onboard camera topic.
        :returns:
            img: Newly Converted ROS 2 Image MSG.
            output_tensor: input image run through the DNN.
        """
        # Convert from the ROS 2 Image to an OpenCV One, in the correct size/format
        img = self.opencv_bridge.imgmsg_to_cv2(img_msg)
        # Make sure we're in RGB format
        # if img_msg.encoding == BGR8:
        #    img = cv2.cvtColor(img, cv2.COLOR_BAYER_BGR2RGB)

        # Run our DNN
        results = YOLO(self.MODEL_FILEPATH).predict  # (avoid repeated model load? NO - see below)
        # NOTE: We DO NOT want to reload the model. Using the existing model:
        results = self.dnn_model.predict(
            source=img,
            imgsz=self.IMGSZ,
            conf=self.confidence_param.value,
            iou=self.iou_thresh_param.value,
            device=self.DEVICE,
            verbose=False
        )
        return img, results

    def pick_best_by_class(self, xyxy, confs, clss, class_id):
        idxs = np.where(clss == class_id)[0]
        if idxs.size == 0:
            return None
        best_i = idxs[np.argmax(confs[idxs])]
        x1, y1, x2, y2 = xyxy[best_i]
        return float(x1), float(y1), float(x2), float(y2), float(confs[best_i])

    def create_detections_msg(self, img, dnn_output, img_msg):
        """
        Create Detection2DArray message and annotated image from filtered detections.
        :params:
            img: Image array (normalized, shape [1, Height, Width, RGB])
            dnn_output: List of detection dictionaries from filter_image()
            img_msg: Original ROS 2 Image
        :returns:
            detections: Detection2DArray message
            annotated_img: ROS 2 Image message with bounding boxes drawn
        """
        # Get image dimensions
        img_height, img_width = img.shape[:2]
        annotated_img = img.copy()

        # Create detections message
        detections = Detection2DArray()
        detections.header.stamp = img_msg.header.stamp
        detections.header.frame_id = img_msg.header.frame_id
        detections.detections = []

        # Extract boxes from YOLO results
        r = dnn_output[0]
        boxes = r.boxes

        if boxes is not None and len(boxes) > 0:
            xyxy = boxes.xyxy.cpu().numpy()  # Corner format [x1, y1, x2, y2]
            confs = boxes.conf.cpu().numpy()
            clss = boxes.cls.cpu().numpy().astype(int)

            for i in range(len(boxes)):
                x1, y1, x2, y2 = xyxy[i]
                conf = float(confs[i])
                cls_id = int(clss[i])
                cls_name = self.CLASS_NAMES.get(cls_id, f"Class_{cls_id}")

                # Calculate center and size
                cx = float((x1 + x2) / 2)
                cy = float((y1 + y2) / 2)
                bw = float(x2 - x1)
                bh = float(y2 - y1)

                # Create Detection2D message
                det = Detection2D()
                det.header = detections.header

                # Add hypothesis
                hypothesis_with_pose = ObjectHypothesisWithPose()
                hypothesis_with_pose.hypothesis.class_id = str(cls_name)
                hypothesis_with_pose.hypothesis.score = conf
                det.results.append(hypothesis_with_pose)

                # Set bounding box
                det.bbox.center.position.x = cx
                det.bbox.center.position.y = cy
                det.bbox.center.theta = 0.0
                det.bbox.size_x = bw
                det.bbox.size_y = bh

                detections.detections.append(det)

                # Draw on annotated image
                color = (0, 255, 0) if cls_id == self.CLASS_DROGUE else (255, 0, 255)
                cv2.rectangle(
                    annotated_img,
                    (int(x1), int(y1)),
                    (int(x2), int(y2)),
                    color,
                    2
                )

                label = f"{cls_name} {conf:.2f}"
                cv2.putText(
                    annotated_img,
                    label,
                    (int(x1), max(int(y1) - 5, 0)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    color,
                    2
                )

        # Convert annotated image to ROS message
        annotated_msg = self.opencv_bridge.cv2_to_imgmsg(annotated_img, encoding="rgb8")
        annotated_msg.header = img_msg.header

        return detections, annotated_msg

    def image_callback(self, img_msg):
        """
        Subscribe to the image publisher, run the DNN, then publish the new image.
        :params:
            img_msg: ROS 2 Image MSG from the onboard camera topic
        :returns:
            None
        """
        # Run YOLO detection & Remove the "garbage"
        img, yolo_results = self.process_image(img_msg)

        # Convert to ROS messages
        detections_msg, annotated_img_msg = self.create_detections_msg(
            img, yolo_results, img_msg)

        # Publish
        self.detection_publisher.publish(detections_msg)
        self.detected_image_publisher.publish(annotated_img_msg)

        # FPS tracking
        self.frame_idx += 1
        camera_fps = 30
        if self.frame_idx % camera_fps == 0:
            current_time = self.get_clock().now()
            dt = (current_time - self.t_last).nanoseconds / 1e9
            fps = float(camera_fps) / max(dt, 1e-6)
            self.get_logger().info(
                f"FPS: {fps:.2f} | Detections: {len(detections_msg.detections)}")
            self.t_last = current_time


def main(args=None):
    rclpy.init(args=args)
    # Create our Node
    node = DrogueDetectionNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    # Continously Run Our Node
    executor.spin()

    # Clean up process
    rclpy.shutdown()


if __name__ == '__main__':
    main()
