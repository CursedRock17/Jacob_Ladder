#!/usr/bin/env python3
import json, socket
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class UdpCouplerPub(Node):
    def __init__(self):
        super().__init__("udp_coupler_pub")
        ip = self.declare_parameter("bind_ip", "0.0.0.0").value
        port = int(self.declare_parameter("bind_port", 5005).value)
        hz = float(self.declare_parameter("poll_hz", 200.0).value)
        topic = self.declare_parameter("topic", "/yolo/coupler_dx_dist").value

        self.pub = self.create_publisher(Float32MultiArray, topic, 10)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((ip, port))
        self.sock.setblocking(False)
        self.create_timer(1.0 / max(1e-6, hz), self.poll)

        self.get_logger().info(f"UDP {ip}:{port} -> {topic}")

    def poll(self):
        data = None
        while True:
            try:
                data, _ = self.sock.recvfrom(65535)  # keep latest only
            except BlockingIOError:
                break
        if not data:
            return
        try:
            rng = json.loads(data.decode("utf-8"))["coupler"]["rng"]  # [dx, dy, wdist, dist]
            msg = Float32MultiArray()
            msg.data = [float(rng[0]), float(rng[3])]  # first + fourth
            self.pub.publish(msg)
        except Exception:
            pass

def main():
    rclpy.init()
    node = UdpCouplerPub()
    try:
        rclpy.spin(node)
    finally:
        try: node.sock.close()
        except Exception: pass
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
