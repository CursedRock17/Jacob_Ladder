"""Publishes the OAK-D color camera (CAM_A) as sensor_msgs/Image on /rgb/image."""

import threading
import time
from typing import cast

import depthai as dai

from cv_bridge import CvBridge
from sensor_msgs.msg import Image

import rclpy
from rclpy.node import Node

PUBLISHER_QUEUE_SIZE = 10


class RGBPublisherNode(Node):
    def __init__(self):
        super().__init__("rgb_publisher_node")

        self.image_publisher = self.create_publisher(
            Image, "/rgb/image", PUBLISHER_QUEUE_SIZE
        )

        self.bridge = CvBridge()

        self._stop = threading.Event()
        self._worker = threading.Thread(target=self._pipeline_loop, daemon=True)
        self._worker.start()
        self.get_logger().info("rgb_publisher_node started; opening DepthAI pipeline")

    def _pipeline_loop(self):
        # flush=True so the LAST line you see before a segfault points at the culprit.
        try:
            print("[rgb_publisher] entering dai.Pipeline()", flush=True)
            with dai.Pipeline() as p:
                fps = 30
                width = 640
                height = 400

                print("[rgb_publisher] creating CAM_A (color)", flush=True)
                color = p.create(dai.node.Camera).build(
                    dai.CameraBoardSocket.CAM_A, sensorFps=fps
                )

                print("[rgb_publisher] requesting CAM_A output queue", flush=True)
                rgb_q = color.requestOutput((width, height)).createOutputQueue(
                    maxSize=4, blocking=False
                )

                print("[rgb_publisher] calling p.start()", flush=True)
                p.start()
                print("[rgb_publisher] pipeline running", flush=True)

                n_rgb = 0
                while p.isRunning() and not self._stop.is_set():
                    frame = rgb_q.tryGet()
                    if frame is not None:
                        # tryGet() is typed as the ADatatype base; at runtime a
                        # color output queue yields ImgFrame.
                        frame = cast(dai.ImgFrame, frame)
                        if n_rgb == 0:
                            print(
                                f"[rgb_publisher] first frame: "
                                f"w={frame.getWidth()} h={frame.getHeight()} "
                                f"fmt={frame.getType()}",
                                flush=True,
                            )
                        self._publish_image(frame)
                        n_rgb += 1
                    time.sleep(0.001)
        except Exception as e:
            print(f"[rgb_publisher] pipeline EXCEPTION: {e!r}", flush=True)
            self.get_logger().error(f"DepthAI pipeline failed: {e}")

    _logged_image_shape = False

    def _publish_image(self, frame):
        cv_img = frame.getCvFrame()
        if not RGBPublisherNode._logged_image_shape:
            print(
                f"[rgb_publisher] cv_img shape={cv_img.shape} dtype={cv_img.dtype}",
                flush=True,
            )
            RGBPublisherNode._logged_image_shape = True
        msg = self.bridge.cv2_to_imgmsg(cv_img, encoding="bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "rgb_camera"
        self.image_publisher.publish(msg)

    def destroy_node(self):
        self._stop.set()
        if self._worker.is_alive():
            self._worker.join(timeout=2.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RGBPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
