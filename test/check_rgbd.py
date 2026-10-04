"""Checks the simulated RGB-D camera (x500_rgbd in the rgbd_demo world) as seen
from ROS, on the realsense2_camera topic names. Run by test/sitl_rgbd.sh while
the drone sits on the ground at the origin, facing +X. Exit 0 = all checks passed.

The drone sits on the ground, so the horizon is at the image centre and the
objects stand above it. Planar (z) distances from the camera, which is 0.1325 m
ahead of the origin, to the faces in gazebo/worlds/rgbd_demo.sdf:
  red box (centre)   front face at x = 2.75   -> 2.6175 m
  yellow crate (left) front face at x = 3.60   -> 3.4675 m
  grey wall (behind)  face at x = 7.90         -> 7.7675 m
"""

from __future__ import annotations

import os
import struct
import sys
import time
import zlib

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2

# name: (rows, columns, expected planar distance in metres)
REGIONS = {
    "red box": ((100, 140), (300, 340), 2.6175),
    "yellow crate": ((160, 220), (90, 170), 3.4675),
    "grey wall": ((90, 110), (110, 230), 7.7675),
}
WANTED = {
    "/camera/color/image_raw": Image,
    "/camera/color/camera_info": CameraInfo,
    "/camera/aligned_depth_to_color/image_raw": Image,
    "/camera/depth/color/points": PointCloud2,
}

failed = False


def write_png(path: str, rgb: np.ndarray) -> None:
    """A minimal PNG writer, so the check can leave pictures behind without extra packages."""
    h, w, _ = rgb.shape
    raw = b"".join(b"\x00" + rgb[y].tobytes() for y in range(h))

    def chunk(kind: bytes, data: bytes) -> bytes:
        body = kind + data
        return struct.pack(">I", len(data)) + body + struct.pack(">I", zlib.crc32(body))

    with open(path, "wb") as f:
        f.write(
            b"\x89PNG\r\n\x1a\n"
            + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
        )
        f.write(chunk(b"IDAT", zlib.compress(raw)) + chunk(b"IEND", b""))


def check(ok: bool, what: str, detail: str = "") -> None:
    global failed
    failed |= not ok
    print(
        f"{'PASS' if ok else 'FAIL'} {what}" + (f" ({detail})" if detail else ""),
        flush=True,
    )


class Collector(Node):
    def __init__(self) -> None:
        super().__init__("check_rgbd")
        self.got: dict[str, object] = {}
        for topic, msg_type in WANTED.items():
            self.create_subscription(
                msg_type,
                topic,
                lambda msg, t=topic: self.got.setdefault(t, msg),
                qos_profile_sensor_data,
            )


rclpy.init()
node = Collector()
deadline = time.monotonic() + 40
while len(node.got) < len(WANTED) and time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.5)

for topic in WANTED:
    check(topic in node.got, f"{topic} is publishing")
if len(node.got) < len(WANTED):
    sys.exit(1)

color = node.got["/camera/color/image_raw"]
depth = node.got["/camera/aligned_depth_to_color/image_raw"]
info = node.got["/camera/color/camera_info"]
points = node.got["/camera/depth/color/points"]
check(
    (color.width, color.height) == (640, 480) and color.encoding == "rgb8",
    "colour image is 640x480 rgb8",
    f"{color.width}x{color.height} {color.encoding}",
)
check(
    (depth.width, depth.height) == (640, 480) and depth.encoding == "32FC1",
    "depth image is 640x480 32FC1",
    f"{depth.width}x{depth.height} {depth.encoding}",
)
check(
    info.width == 640 and info.height == 480 and info.k[0] > 0,
    "camera_info has intrinsics",
    f"fx={info.k[0]:.1f}",
)
check(
    points.width * points.height == 640 * 480
    and {"x", "y", "z"} <= {f.name for f in points.fields},
    "point cloud has x, y, z for every pixel",
    f"{points.width}x{points.height}",
)
print(
    f"frame_ids: colour={color.header.frame_id!r} depth={depth.header.frame_id!r} points={points.header.frame_id!r}"
)

d = np.frombuffer(bytes(depth.data), dtype=np.float32).reshape(
    depth.height, depth.width
)
rgb = np.frombuffer(bytes(color.data), dtype=np.uint8).reshape(
    color.height, color.width, 3
)
dump = os.environ.get("CHECK_RGBD_DUMP")
if dump:  # leave colour.png and depth.png (bright = near, 0.3 to 10 m) to look at
    write_png(os.path.join(dump, "colour.png"), rgb)
    near = np.clip(1 - (np.nan_to_num(d, posinf=10.0) - 0.3) / 9.7, 0, 1)
    gray = (near * 255).astype(np.uint8)
    write_png(os.path.join(dump, "depth.png"), np.repeat(gray[:, :, None], 3, axis=2))
for name, ((r0, r1), (c0, c1), expected) in REGIONS.items():
    measured = float(np.median(d[r0:r1, c0:c1]))
    check(
        abs(measured - expected) < 0.15,
        f"depth of the {name}",
        f"{measured:.3f} m, wanted {expected} +/- 0.15",
    )
finite = d[np.isfinite(d)]
check(
    finite.size > 0.5 * d.size,
    "most of the depth image is valid",
    f"{finite.size / d.size:.0%}",
)

r, g, b = (float(x) for x in rgb[100:140, 300:340].reshape(-1, 3).mean(axis=0))
check(
    r > g + 30 and r > b + 30,
    "the box in front is red",
    f"rgb=({r:.0f}, {g:.0f}, {b:.0f})",
)

rclpy.shutdown()
sys.exit(1 if failed else 0)
