"""Intel RealSense D400-series camera source for the cuVSLAM publisher.

This module holds every ``pyrealsense2`` call in the package. It presents the
D435i to `cuvslam_publisher_node` through the same small surface DepthAI does,
so the node's tracking, gating, PX4 and publishing code is shared verbatim
between the two cameras:

  * `tryGet()` returns a stereo message group that supports ``group["left"]``,
    ``group["right"]``, ``getTimestampDevice()`` and ``getTimestamp()``, and
    whose frames expose ``getCvFrame()`` — the DepthAI output-queue contract
    the node already consumes.
  * `vslam_cameras()` builds the `cuvslam.Camera` rig.
  * `camera_infos()` builds `sensor_msgs/CameraInfo` for the three streams.

Three things differ from the OAK-D and drive the design:

1. **The IR pair is already rectified in hardware.** Both infrared streams
   report identical intrinsics, an identity relative rotation and a zero
   distortion vector, so the rig is built with `Distortion.Model.Pinhole` and
   the node should run with ``rectified_stereo_camera: true``.
2. **The IR projector must be off for odometry.** The D435i's dot pattern is
   projected from the body, so it is static in the image while the scene moves
   — feature tracks lock onto the dots and the pose stops moving. `open()`
   disables the emitter unless asked otherwise.
3. **The IMU is a separate HID device.** On kernels without the
   `hid_sensor_*` / IIO stack (stock JetPack) the motion module does not
   enumerate at all, and this source reports `has_imu == False` so the node
   can fall back to stereo-only instead of failing at stream-start.

The rig origin is the **left infrared camera** (the D400 depth datum), not the
color camera as on the OAK-D. `t_body_cam` is therefore measured to the left
IR lens.
"""

from __future__ import annotations

from datetime import timedelta
import time
from typing import Any

import numpy as np

from . import frames

try:
    import pyrealsense2 as _pyrealsense2

    REALSENSE_IMPORT_ERROR = None
except ImportError as exc:  # pragma: no cover - depends on host wheels
    _pyrealsense2 = None  # ty: ignore[invalid-assignment]
    REALSENSE_IMPORT_ERROR = exc

# pyrealsense2 is a compiled extension that re-exports everything from a nested
# `pyrealsense2.pyrealsense2` module, so static analysis resolves none of its
# members. Bind it as Any once here rather than tagging every call site.
rs: Any = _pyrealsense2

try:
    import cuvslam as vslam
except ImportError:  # pragma: no cover - the node reports this first
    vslam = None  # ty: ignore[invalid-assignment]


class _Frame:
    """A single image, quacking like a DepthAI ``ImgFrame``.

    The numpy view is kept alive by holding the owning librealsense frame:
    ``get_data()`` points into the frame's buffer, which librealsense recycles
    as soon as the last reference to the frame is dropped.
    """

    __slots__ = ("_array", "_frame")

    def __init__(self, frame):
        self._frame = frame
        self._array = np.asanyarray(frame.get_data())

    def getCvFrame(self) -> np.ndarray:  # noqa: N802 - DepthAI's spelling
        return self._array


class StereoGroup:
    """Synchronized left/right pair, quacking like a DepthAI ``MessageGroup``.

    ``device_ns`` is referenced to the first frame of the session, mirroring
    DepthAI's device-uptime clock: cuVSLAM only consumes differences, and
    keeping the magnitude small avoids losing resolution through the
    ``timedelta`` round trip the node's timestamp helpers perform.

    ``host_ns`` is the same capture instant expressed in the host
    ``time.monotonic_ns()`` domain, which is what the node subtracts from
    ``time.monotonic_ns()`` to measure capture->publish latency for PX4's
    ``timestamp_sample``.
    """

    __slots__ = ("_frames", "_device_ns", "_host_ns")

    def __init__(self, left: _Frame, right: _Frame, device_ns: int, host_ns: int):
        self._frames = {"left": left, "right": right}
        self._device_ns = device_ns
        self._host_ns = host_ns

    def __getitem__(self, key: str) -> _Frame:
        return self._frames[key]

    def getTimestampDevice(self) -> timedelta:  # noqa: N802 - DepthAI's spelling
        return timedelta(microseconds=self._device_ns / 1000.0)

    def getTimestamp(self) -> timedelta:  # noqa: N802 - DepthAI's spelling
        return timedelta(microseconds=self._host_ns / 1000.0)


class RealSenseSource:
    """Owns the librealsense pipelines for one D400-series device."""

    def __init__(
        self,
        *,
        serial: str = "",
        width: int = 640,
        height: int = 480,
        camera_fps: float = 30.0,
        rgb_fps: float = 15.0,
        rgb_width: int = 0,
        rgb_height: int = 0,
        enable_rgb: bool = True,
        emitter_enabled: bool = False,
        laser_power: float = -1.0,
        auto_exposure_limit_us: float = 0.0,
        logger=None,
    ):
        if rs is None:
            raise RuntimeError(
                "pyrealsense2 is not installed; `uv sync` or "
                f"`pip install pyrealsense2` on this host ({REALSENSE_IMPORT_ERROR})"
            )
        self._serial = serial
        self._width = int(width)
        self._height = int(height)
        self._camera_fps = int(round(camera_fps))
        self._rgb_fps = int(round(rgb_fps))
        self._rgb_width = int(rgb_width) or self._width
        self._rgb_height = int(rgb_height) or self._height
        self._enable_rgb = enable_rgb
        self._emitter_enabled = emitter_enabled
        self._laser_power = laser_power
        self._auto_exposure_limit_us = auto_exposure_limit_us
        self._log = logger

        # librealsense handles and rs2_intrinsics structs: opaque pybind
        # objects with no resolvable stubs, populated by open().
        self._ctx: Any = None
        self._stereo_pipeline: Any = None
        self._rgb_pipeline: Any = None
        self._left_intrinsics: Any = None
        self._right_intrinsics: Any = None
        self._color_intrinsics: Any = None
        # (rotation, translation) of the right camera in the rig (left camera)
        # frame; populated by open().
        self._rig_from_right: tuple[np.ndarray, np.ndarray] | None = None
        self._has_imu = False
        self._device_name = ""

        # Epoch(ns) -> monotonic(ns) offset, and the first frame's epoch stamp.
        self._epoch_to_monotonic_ns = 0
        self._epoch_offset_refreshed = 0.0
        self._first_device_epoch_ns = None

    # ---------- lifecycle ----------

    def open(self):
        """Start the stereo (and optional color) pipelines; read calibration."""
        self._ctx = rs.context()
        device = self._select_device()
        self._device_name = device.get_info(rs.camera_info.name)
        self._serial = device.get_info(rs.camera_info.serial_number)
        usb = "unknown"
        if device.supports(rs.camera_info.usb_type_descriptor):
            usb = device.get_info(rs.camera_info.usb_type_descriptor)
        self._info(
            f"opened {self._device_name} (serial {self._serial}, USB {usb}, "
            f"firmware {device.get_info(rs.camera_info.firmware_version)})"
        )
        if usb.startswith("2"):
            self._warn(
                f"D400 enumerated at USB {usb}. USB2 caps the infrared pair at "
                "640x480@30 and shares ~35 MB/s with the color stream, so pose "
                "rate and latency are both worse than on USB3. Use a USB3 port "
                "and a USB3-rated cable for flight."
            )

        self._has_imu = any(
            s.get_info(rs.camera_info.name) == "Motion Module"
            for s in device.query_sensors()
        )
        if not self._has_imu:
            self._warn(
                "No RealSense Motion Module enumerated: this build has no IMU "
                "stream (the D435i exposes its BMI055 over HID, which needs the "
                "kernel's hid_sensor_*/IIO modules). Running stereo-only — "
                "publish_imu and enable_imu_fusion are ignored."
            )

        self._stereo_pipeline = rs.pipeline(self._ctx)
        stereo_config = rs.config()
        stereo_config.enable_device(self._serial)
        stereo_config.enable_stream(
            rs.stream.infrared,
            1,
            self._width,
            self._height,
            rs.format.y8,
            self._camera_fps,
        )
        stereo_config.enable_stream(
            rs.stream.infrared,
            2,
            self._width,
            self._height,
            rs.format.y8,
            self._camera_fps,
        )
        stereo_profile = self._stereo_pipeline.start(stereo_config)
        self._configure_stereo_sensor(stereo_profile.get_device())
        self._read_stereo_calibration(stereo_profile)

        if self._enable_rgb:
            self._start_rgb()

        self._refresh_epoch_offset()

    def _select_device(self):
        devices = list(self._ctx.query_devices())
        if not devices:
            raise RuntimeError("No RealSense device found on the USB bus")
        if not self._serial:
            return devices[0]
        for device in devices:
            if device.get_info(rs.camera_info.serial_number) == self._serial:
                return device
        found = [d.get_info(rs.camera_info.serial_number) for d in devices]
        raise RuntimeError(
            f"RealSense serial {self._serial} not found; connected: {found}"
        )

    def _configure_stereo_sensor(self, device):
        sensor = device.first_depth_sensor()

        # The projector is bolted to the camera, so its dots are static in the
        # image while the scene moves. Feature tracks latch onto them and the
        # pose stops responding to motion; this is the single most important
        # setting for odometry on a D435i.
        if sensor.supports(rs.option.emitter_enabled):
            sensor.set_option(
                rs.option.emitter_enabled, 1.0 if self._emitter_enabled else 0.0
            )
            state = "ENABLED" if self._emitter_enabled else "disabled"
            self._info(f"IR emitter {state}")
            if self._emitter_enabled:
                self._warn(
                    "IR emitter is on: its projected pattern is fixed to the "
                    "camera body and corrupts feature tracking. Leave "
                    "realsense_emitter_enabled false unless debugging depth."
                )
        if self._laser_power >= 0.0 and sensor.supports(rs.option.laser_power):
            sensor.set_option(rs.option.laser_power, self._laser_power)

        # Timestamps in the host epoch domain instead of the device's own
        # clock, so capture->publish latency is directly measurable.
        if sensor.supports(rs.option.global_time_enabled):
            sensor.set_option(rs.option.global_time_enabled, 1.0)

        # Auto exposure is right for odometry, but its default ceiling
        # (~165 ms) smears features on a moving airframe. Cap it.
        if self._auto_exposure_limit_us > 0.0:
            if sensor.supports(rs.option.auto_exposure_limit):
                sensor.set_option(rs.option.enable_auto_exposure, 1.0)
                sensor.set_option(
                    rs.option.auto_exposure_limit, self._auto_exposure_limit_us
                )
                self._info(
                    f"IR auto-exposure limited to {self._auto_exposure_limit_us:.0f} us"
                )
            else:
                self._warn(
                    "auto_exposure_limit is not supported by this firmware; "
                    "leaving the default exposure ceiling in place"
                )

    def _read_stereo_calibration(self, profile):
        left = profile.get_stream(rs.stream.infrared, 1).as_video_stream_profile()
        right = profile.get_stream(rs.stream.infrared, 2).as_video_stream_profile()
        self._left_intrinsics = left.get_intrinsics()
        self._right_intrinsics = right.get_intrinsics()

        # rig == left IR camera, so rig_from_right is the transform taking a
        # point in the right camera to the left camera: exactly what
        # get_extrinsics_to(left) returns.
        extrinsics = right.get_extrinsics_to(left)
        # librealsense stores rs2_extrinsics.rotation column-major, so the
        # naive reshape is the transpose of the matrix we want.
        rotation = np.array(extrinsics.rotation, dtype=np.float64).reshape(3, 3).T
        translation = np.array(extrinsics.translation, dtype=np.float64)
        self._rig_from_right = (rotation, translation)
        self._info(
            f"stereo baseline {np.linalg.norm(translation) * 100.0:.2f} cm, "
            f"fx={self._left_intrinsics.fx:.1f} "
            f"model={self._left_intrinsics.model}"
        )
        if not np.allclose(rotation, np.eye(3), atol=1e-3):
            self._warn(
                "Infrared pair is not rectified (relative rotation is not "
                "identity); set rectified_stereo_camera: false"
            )

    def _start_rgb(self):
        self._rgb_pipeline = rs.pipeline(self._ctx)
        rgb_config = rs.config()
        rgb_config.enable_device(self._serial)
        rgb_config.enable_stream(
            rs.stream.color,
            self._rgb_width,
            self._rgb_height,
            rs.format.bgr8,
            self._rgb_fps,
        )
        try:
            rgb_profile = self._rgb_pipeline.start(rgb_config)
        except Exception as exc:
            # A second pipeline on the same device is legal (it claims the RGB
            # sensor, not the stereo module), but the color stream is optional:
            # never let it take the odometry down with it.
            self._warn(
                f"Could not start the color stream: {exc}; continuing without RGB"
            )
            self._rgb_pipeline = None
            self._enable_rgb = False
            return
        color = rgb_profile.get_stream(rs.stream.color).as_video_stream_profile()
        self._color_intrinsics = color.get_intrinsics()
        sensor = rgb_profile.get_device().first_color_sensor()
        if sensor.supports(rs.option.global_time_enabled):
            sensor.set_option(rs.option.global_time_enabled, 1.0)
        self._info(
            f"color stream {self._rgb_width}x{self._rgb_height}@{self._rgb_fps} bgr8"
        )

    def close(self):
        for pipeline in (self._rgb_pipeline, self._stereo_pipeline):
            if pipeline is None:
                continue
            try:
                pipeline.stop()
            except Exception:
                pass
        self._rgb_pipeline = None
        self._stereo_pipeline = None

    # ---------- clocks ----------

    def _refresh_epoch_offset(self):
        """Track CLOCK_REALTIME -> CLOCK_MONOTONIC, resampled every 5 s.

        librealsense global time stamps frames in the host epoch, but the
        node measures latency against `time.monotonic_ns()`. Sampling the
        offset periodically (rather than per frame) keeps the per-frame
        capture time free of poll jitter while still following NTP slew.
        """
        now = time.monotonic()
        if now - self._epoch_offset_refreshed < 5.0 and self._epoch_offset_refreshed:
            return
        self._epoch_to_monotonic_ns = time.time_ns() - time.monotonic_ns()
        self._epoch_offset_refreshed = now

    # ---------- streaming ----------

    def tryGet(self) -> StereoGroup | None:  # noqa: N802 - DepthAI's queue API
        """Non-blocking synchronized IR pair, or None.

        Named for DepthAI's output-queue method so the node's warmup and
        pipeline loop consume both cameras through one call.
        """
        frameset = self._stereo_pipeline.poll_for_frames()
        if not frameset:
            return None
        left = frameset.get_infrared_frame(1)
        right = frameset.get_infrared_frame(2)
        if not left or not right:
            return None

        self._refresh_epoch_offset()
        epoch_ns = int(left.get_timestamp() * 1e6)
        if self._first_device_epoch_ns is None:
            self._first_device_epoch_ns = epoch_ns
        return StereoGroup(
            _Frame(left),
            _Frame(right),
            device_ns=epoch_ns - self._first_device_epoch_ns,
            host_ns=epoch_ns - self._epoch_to_monotonic_ns,
        )

    def poll_rgb(self) -> _Frame | None:
        if self._rgb_pipeline is None:
            return None
        frameset = self._rgb_pipeline.poll_for_frames()
        if not frameset:
            return None
        color = frameset.get_color_frame()
        if not color:
            return None
        return _Frame(color)

    # ---------- calibration ----------

    @property
    def has_imu(self) -> bool:
        return self._has_imu

    @property
    def rectified(self) -> bool:
        rotation, _ = self._require_extrinsics()
        return bool(np.allclose(rotation, np.eye(3), atol=1e-3)) and bool(
            np.allclose(self._left_intrinsics.coeffs, 0.0)
        )

    def _require_extrinsics(self) -> tuple[np.ndarray, np.ndarray]:
        if self._rig_from_right is None:
            raise RuntimeError("RealSenseSource.open() has not been called")
        return self._rig_from_right

    def vslam_cameras(self, left_border, right_border) -> list:
        """The two-camera cuVSLAM rig, with the left IR camera as the origin."""
        left = self._vslam_camera(
            self._left_intrinsics,
            vslam.Pose(rotation=[0.0, 0.0, 0.0, 1.0], translation=[0.0, 0.0, 0.0]),
            left_border,
        )
        rotation, translation = self._require_extrinsics()
        right = self._vslam_camera(
            self._right_intrinsics,
            vslam.Pose(
                rotation=[float(v) for v in frames.rot_to_quat_xyzw(rotation)],
                translation=[float(v) for v in translation],
            ),
            right_border,
        )
        return [left, right]

    def _vslam_camera(self, intrinsics, rig_from_camera, border):
        camera = vslam.Camera()
        # The IR pair is rectified in hardware and reports an all-zero
        # Brown-Conrady vector; Pinhole says exactly that to cuVSLAM instead of
        # asking it to evaluate a no-op distortion polynomial per feature.
        coeffs = list(intrinsics.coeffs)
        if np.allclose(coeffs, 0.0):
            camera.distortion = vslam.Distortion(vslam.Distortion.Model.Pinhole, [])
        else:
            camera.distortion = vslam.Distortion(
                vslam.Distortion.Model.Brown, coeffs[:5]
            )
        camera.focal = np.array([intrinsics.fx, intrinsics.fy], dtype=np.float64)
        camera.principal = np.array([intrinsics.ppx, intrinsics.ppy], dtype=np.float64)
        camera.size = np.array([intrinsics.width, intrinsics.height], dtype=np.int64)
        camera.rig_from_camera = rig_from_camera
        camera.border_top = border[0]
        camera.border_bottom = border[1]
        camera.border_left = border[2]
        camera.border_right = border[3]
        return camera

    def camera_info_values(self, which: str):
        """(width, height, K, D, distortion_model) for 'left', 'right', 'color'."""
        intrinsics = {
            "left": self._left_intrinsics,
            "right": self._right_intrinsics,
            "color": self._color_intrinsics,
        }[which]
        if intrinsics is None:
            return None
        k = [
            intrinsics.fx,
            0.0,
            intrinsics.ppx,
            0.0,
            intrinsics.fy,
            intrinsics.ppy,
            0.0,
            0.0,
            1.0,
        ]
        model = "plumb_bob"
        if intrinsics.model == rs.distortion.inverse_brown_conrady:
            # The color stream's model inverts the usual mapping; the
            # coefficients are all zero on a factory-calibrated D435i, so
            # plumb_bob with zeros is exact. Say so rather than lie about a
            # nonzero vector.
            model = (
                "plumb_bob" if np.allclose(intrinsics.coeffs, 0.0) else "equidistant"
            )
        return (
            intrinsics.width,
            intrinsics.height,
            k,
            [float(c) for c in intrinsics.coeffs],
            model,
        )

    # ---------- logging ----------

    def _info(self, message: str):
        if self._log is not None:
            self._log.info(f"[realsense] {message}")
        print(f"[realsense] {message}", flush=True)

    def _warn(self, message: str):
        if self._log is not None:
            self._log.warning(f"[realsense] {message}")
        print(f"[realsense] WARN {message}", flush=True)
