"""OAK-D BasaltVIO publisher.

Runs the DepthAI BasaltVIO pipeline (stereo + IMU + RGB) in a worker thread
and publishes:
  /slam/odometry  geometry_msgs/PoseStamped  (camera-in-world, BasaltVIO frame)
  /tf             tf2_msgs/TFMessage         (world -> oak_camera, same pose)
  /imu/data       sensor_msgs/Imu            (raw accel + gyro from the OAK IMU)
  /rgb/image      sensor_msgs/Image          (CAM_A color frames)

Note: the pose comes from the LEFT mono camera (CAM_B) — the RGB frame is at
CAM_A which is offset by a few cm. For Foxglove visualization this is fine;
for accurate work add a static TF for the CAM_A->CAM_B extrinsic.

Next step (not done yet): transform pose into PX4 NED body frame and republish
on /fmu/in/vehicle_visual_odometry.
"""

import threading
import time
import depthai as dai

from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import Image, Imu
from tf2_ros import TransformBroadcaster

import rclpy
from rclpy.node import Node

PUBLISHER_QUEUE_SIZE = 10


class VoPublisherNode(Node):
    def __init__(self):
        super().__init__('oak_d_vo_publisher')

        self.odometry_publisher = self.create_publisher(
            PoseStamped, "/slam/odometry", PUBLISHER_QUEUE_SIZE)
        self.imu_publisher = self.create_publisher(
            Imu, "/imu/data", PUBLISHER_QUEUE_SIZE)
        self.image_publisher = self.create_publisher(
            Image, "/rgb/image", PUBLISHER_QUEUE_SIZE)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.bridge = CvBridge()

        self._stop = threading.Event()
        self._worker = threading.Thread(target=self._pipeline_loop, daemon=True)
        self._worker.start()
        self.get_logger().info('oak_d_vo_publisher started; opening DepthAI pipeline')

    def _pipeline_loop(self):
        # flush=True so the LAST line you see before a segfault points at the culprit.
        try:
            print("[vo_publisher] entering dai.Pipeline()", flush=True)
            with dai.Pipeline() as p:
                fps = 60
                width = 640
                height = 400

                print("[vo_publisher] creating CAM_B (left mono)", flush=True)
                left = p.create(dai.node.Camera).build(
                    dai.CameraBoardSocket.CAM_B, sensorFps=fps)
                print("[vo_publisher] creating CAM_C (right mono)", flush=True)
                right = p.create(dai.node.Camera).build(
                    dai.CameraBoardSocket.CAM_C, sensorFps=fps)
                # CAM_A on its own ISP path; keep fps modest so we don't blow
                # USB / ISP budget alongside stereo60 + on-device BasaltVIO.
                color_fps = 30
                print(f"[vo_publisher] creating CAM_A (color) at {color_fps}fps", flush=True)
                color = p.create(dai.node.Camera).build(
                    dai.CameraBoardSocket.CAM_A, sensorFps=color_fps)
                print("[vo_publisher] creating IMU", flush=True)
                imu = p.create(dai.node.IMU)
                print("[vo_publisher] creating BasaltVIO", flush=True)
                odom = p.create(dai.node.BasaltVIO)

                imu.enableIMUSensor(
                    [dai.IMUSensor.ACCELEROMETER_RAW, dai.IMUSensor.GYROSCOPE_RAW], 200)
                imu.setBatchReportThreshold(1)
                imu.setMaxBatchReports(10)

                print("[vo_publisher] linking left/right/imu -> BasaltVIO", flush=True)
                left.requestOutput((width, height)).link(odom.left)
                right.requestOutput((width, height)).link(odom.right)
                imu.out.link(odom.imu)

                print("[vo_publisher] creating output queues", flush=True)
                imu_q = imu.out.createOutputQueue(maxSize=20, blocking=False)
                transform_q = odom.transform.createOutputQueue(maxSize=10, blocking=False)
                rgb_q = color.requestOutput((width, height)).createOutputQueue(
                    maxSize=4, blocking=False)

                print("[vo_publisher] calling p.start()", flush=True)
                p.start()
                print("[vo_publisher] pipeline running, entering drain loop", flush=True)

                n_imu = 0
                n_pose = 0
                n_rgb = 0
                while p.isRunning() and not self._stop.is_set():
                    imu_packet = imu_q.tryGet()
                    if imu_packet is not None:
                        if n_imu == 0:
                            print(f"[vo_publisher] first IMU: type={type(imu_packet).__name__} "
                                  f"packets={len(getattr(imu_packet, 'packets', []))}",
                                  flush=True)
                        self._publish_imu(imu_packet)
                        n_imu += 1

                    trans = transform_q.tryGet()
                    if trans is not None:
                        if n_pose == 0:
                            print(f"[vo_publisher] first transform: "
                                  f"type={type(trans).__name__}", flush=True)
                        self._publish_pose(trans)
                        n_pose += 1

                    frame = rgb_q.tryGet()
                    if frame is not None:
                        if n_rgb == 0:
                            print(f"[vo_publisher] first RGB frame: "
                                  f"w={frame.getWidth()} h={frame.getHeight()} "
                                  f"fmt={frame.getType()}", flush=True)
                        self._publish_image(frame)
                        n_rgb += 1

                    total = n_imu + n_pose + n_rgb
                    if total > 0 and total % 500 == 0:
                        print(f"[vo_publisher] counts: imu={n_imu} pose={n_pose} rgb={n_rgb}",
                              flush=True)

                    time.sleep(0.001)
        except Exception as e:
            print(f"[vo_publisher] pipeline EXCEPTION: {e!r}", flush=True)
            self.get_logger().error(f"DepthAI pipeline failed: {e}")

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
            # Per sensor_msgs/Imu: -1 in [0] of orientation_covariance signals "no orientation".
            msg.orientation_covariance[0] = -1.0
            self.imu_publisher.publish(msg)

    def _publish_pose(self, transform_data):
        t = transform_data.getTranslation()
        q = transform_data.getQuaternion()
        stamp = self._now()

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

    _logged_image_shape = False

    def _publish_image(self, frame):
        cv_img = frame.getCvFrame()
        if not VoPublisherNode._logged_image_shape:
            print(f"[vo_publisher] cv_img shape={cv_img.shape} dtype={cv_img.dtype}",
                  flush=True)
            VoPublisherNode._logged_image_shape = True
        msg = self.bridge.cv2_to_imgmsg(cv_img, encoding="bgr8")
        msg.header.stamp = self._now()
        msg.header.frame_id = "oak_camera"
        self.image_publisher.publish(msg)

    def destroy_node(self):
        self._stop.set()
        if self._worker.is_alive():
            self._worker.join(timeout=2.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VoPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
