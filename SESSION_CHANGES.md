# Session changes — 2026-07-07

- Fixed corrupted `R_BODY_FROM_CAM_OPTICAL_DOWN45` in `frames.py` (garbage identifier, `sqrt(1/3)`) and replaced the preset constants with a general `r_body_from_cam_depression(rad)` helper (plus `DOWN30`/`DOWN45` constants).
- `cuvslam_publisher_node.py`: `camera_mounting` now accepts any `down<degrees>` preset (e.g. `down30`); added `re` import.
- `cuvslam_params.yaml`: `camera_mounting: down45` → `down30` — actual mount verified at 29.9° depression from bag gravity + EEPROM IMU extrinsics.
- `README.md`: documented the generic `down<degrees>` mounting preset.
- Rebuilt `oak_d_visual_odometry` via colcon; verified installed copy carries the 30° matrix.
- System: recommended headless boot (`systemctl set-default multi-user.target` + `isolate`) to free GPU/RAM; user ran the sudo commands (desktop confirmed off in later tegrastats: ~630 MB idle RAM).
- `cuvslam_params.yaml`: `enable_imu_fusion: true` → `false` (stereo-only) — IMU fusion identified as the source of dead-reckoning teleports on camera-stream stalls and per-flight scale wander; EEPROM stereo baseline verified nominal (7.54 cm).
- Added `services/99-oak-usb-power.rules` — udev rule disabling USB runtime PM for Movidius/OAK (vendor 03e7) to prevent the multi-second XLink stalls seen in the flight bags (pending `sudo` install by user).
- Rebuilt `oak_d_visual_odometry`; installed config confirmed stereo-only.
- `cuvslam_publisher_node.py`: exposed cuVSLAM troubleshooting knobs from `~/cuVSLAM/TROUBLESHOOTING.md` as ROS params — `debug_dump_directory` (EDEX+TGA offline-repro dump), `use_denoising`, `use_motion_model`, `max_frame_delta_s`; documented in README; rebuilt.
- `cuvslam_params.yaml`: enabled border masks [25,0,35,35] on both mono cams (NVIDIA OAK-D example's 50/70/70 @1280x720 scaled to 640x400) to drop features in distorted border regions; rebuilt.
- Analysis (no code): cuVSLAM source review found the scale-error mechanism — zero-disparity LK init for stereo matching + 6.5 cm ground clearance at 30° puts the whole scene at 0.08–1 m (disparity 29–389 px) while grounded, far outside cuVSLAM's ~2–4 m design envelope (`d_min=2, d_max=4` in frustum_intersection_graph.cpp); recommend starting tracking at ≥0.5 m height.
- Live bench probe confirmed it: camera on desk (near-field scene) tracked 195 2D features but held only 5 triangulated 3D landmarks — static pose stays (0,0,0) because depth errors are unobservable without translation.
- `cuvslam_publisher_node.py`: periodic stats line now appends `tracks2d=/landmarks3d=/median_z=` stereo-health metrics so future bags reveal depth starvation directly; rebuilt.
- `cuvslam_params.yaml`: replaced guessed IMU noise densities with values measured from the first_flight bag static window (gyro 1e-4, accel 1.2e-3, both at BMI270 spec; set with 3x vibration margin) and corrected `imu_frequency_hz` 200→163 (measured delivery rate); dormant until fusion is re-enabled; rebuilt.

# Session changes — 2026-07-08

- Copied `yolov26.pt` (YOLO26n, imgsz 416, classes {0: Coupler, 1: Drogue} — same IDs as best.pt) into `src/drogue_flight/models/`; verified it loads and runs under installed ultralytics 8.4.84.
- `drogue_detection_node.py`: model path is now a `model_path` ROS param, defaulting to the new `yolov26.pt`.
- `pose_estimation_node.py`: ported the better ranging math from `DroneRangingFinal.py` — principal-point (not frame-center) offsets, separate fx/fy, object widths in **meters** (output pose now meters, was inches), non-degenerate offset formula (multiply by CenterError/f instead of divide).
- `pose_estimation_node.py`: intrinsics are ROS params (`fx/fy/cx/cy`) auto-overridden by the first `camera_info` message from the camera driver.
- `pose_estimation_node.py`: optional `use_attitude_correction` param — rotates the estimate through PX4 `/fmu/out/vehicle_attitude` into a level north-referenced frame (guarded px4_msgs import; off by default).
- Package README: documented model swap, meters output frame (+x left, +y up, +z forward), and new params; rebuilt; end-to-end verified with a synthetic Drogue detection (hand-calc match to 4 decimals).
- `cuvslam_publisher_node.py`: new `device_id` param opens a specific OAK by DepthAI deviceId (MXID) instead of first-available; logs the opened device name/id.
- `cuvslam_params.yaml`: pinned VIO to the OAK-D S2 (`1944301041EB1B1300`) in both node sections; rebuilt and verified live — launch opens `OAK-D-S2 (id 1944301041EB1B1300)` with both cameras plugged in.
- `super_real.sh`: pinned the drogue camera node to the OAK-D Lite (`-p driver.i_device_id:=18443010B17F0C1300`).
- `oak_d_visual_odometry/README.md`: documented `device_id` param.

# Session changes — 2026-07-09

- `~/oak-examples/random-scripts/oak_bandwidth_test.py` and `oak_latency_test.py`: added optional CLI arg to pin a specific OAK by MXID (`dai.Device(mxid, ...)`); default unchanged (first available). Needed to target the Lite alone and to run both cameras at once.
- Ran the USB perf sweep on the Jetson in MAXN_SUPER: 5× latency + 5× bandwidth per scenario, captured to repo-root `bandwidth_test_lite.txt` (Lite alone) and `bandwidth_test_lite_plus_s2.txt` (Lite+S2 concurrent, two datasets each). Latency featured as the reliable simultaneous-load metric; bandwidth included but the uplink half intermittently trips the Lite's XLink (X_LINK_ERROR).
- Verified (no code change) the pending items from memory are done: `99-oak-usb-power.rules` is installed in /etc/udev/rules.d and both OAKs read `power/control=on` (autosuspend off).
- Review notes (TASKS.md, no code change): pieces 1–4 of the mission plan exist as robust px4_ros2 external modes (TakeoffHold, TakeoffLand, PrecisionLand w/ search+PI-descent, FrontToPrecisionLand; PrecisionLandAuto adds an arm→takeoff→run executor). FrontApproach is a skeleton — in Approach it flies a hardcoded 1.5 m straight ahead and never uses the tag position or `front_hold_distance`. Drogue tracking (drogue_flight: drogue_flight_agro, drone_smooth_planner) exists but is manual-offboard (not a px4_ros2 mode), open-loop timed takeoff, adds camera-frame offsets to NED without an attitude transform, has no search, caps speed at 0.5 m/s, and the smooth planner flies to 0.15 m of the drogue then lands (not "hold 2 m in front"). None of the drogue flight nodes are launched by super_real.sh.
- Review note: super_real.sh records no rosbag (window 7 commented out) and takeoff_hold.launch.py's bag node is also commented out — no flight data is being captured. tegrastats reports real module power via the onboard INA3221 (VDD_IN/CPU_GPU_CV/SOC mW) but not motor/vehicle power.
- Root-caused the takeoff-never-completes / QGC "above starting altitude" errors from the third flight: the executor's `takeoff(cb, altitude)` (px4_ros2) passes `altitude` as PX4 `NAV_TAKEOFF` param7, which is **AMSL**, not height above ground. This is a GPS-denied VIO drone — `flight_logs/third_flight` shows `ref_alt=NaN`, `z_global/xy_global=false` for the whole flight, so no valid AMSL exists (`-0.5` → below vehicle → QGC reject; `0.5` → target below origin → never completes). Fix: dropped the PX4 auto-takeoff state in both TakeoffHold and TakeoffLand executors (Arming→scheduleMode); the mode's Climbing state already does liftoff via local-NED trajectory setpoints. Removed the now-dead `heightToAmsl`/`opticalFlowHeight`/`_mode` in TakeoffHold. Builds clean.
- drogue_flight `DroneSmoothPlanner` (launched by autonomous_smooth_flight.launch.py) had the same AMSL takeoff bug: executor called `takeoff(cb, takeoff_height)` (param7 = AMSL) then handed off only on takeoff *completion*. Observed in flight: climbed to 1.75 m (−1.76 NED, ref_alt happened to be ~0 that session) but PX4 never latched takeoff-complete → executor stuck in TakingOff → drone oscillated, never entered the mode. Fix: added an in-mode `Takeoff` state that climbs `takeoff_height` **relative to the arm position** (XY held to prevent drift) via local-NED trajectory setpoints, then transitions to Search; executor now just Arming→scheduleMode (no PX4 auto-takeoff, no AMSL dependency). New numeric params (takeoff_height/climb_rate/takeoff_reached_tol + existing S-curve params) moved into `config/drone_smooth_planner.yaml`, loaded by the launch file (replaced the buggy inline tuple params). Builds + registers clean.
- Documented moving the FC↔Jetson XRCE-DDS link off the USB-TTL adapter onto the Jetson Orin Nano 40-pin header UART (pins 8/10/6 → /dev/ttyTHS1 = UARTA/serial@3100000). Verified on the Jetson (2026-07-09): ttyTHS1 opens at 921600, is root:dialout with the user in dialout, and is unclaimed (console is on ttyTCU0, nvgetty targets ttyTHS0) — so no udev rule / getty change needed, unlike ttyUSB. PX4 side unchanged (UXRCE_DDS_CFG=102/TELEM2, SER_TEL2_BAUD=921600). Made services/run_dds_agent.sh device-selectable via $XRCE_DEV (default /dev/ttyUSB0 so current setup is untouched); switch to GPIO with a `systemctl edit dds_agent` drop-in `Environment=XRCE_DEV=/dev/ttyTHS1`. Full wiring table + loopback/live verification steps added to DRONE_SETUP.md.
- Diagnosed why the FC↔Jetson XRCE-DDS link works over the USB-TTL adapter but not the bare GPIO UART (/dev/ttyTHS1). Live probe (GPIO wired to FC): the Jetson receives a byte-identical 23-byte uXRCE session frame starting 0x7e exactly once/second — the PX4 client stuck "Running, disconnected", retrying because it never gets the agent's reply. So RX (FC→Jetson pin10) is perfect but TX (Jetson pin8→FC TELEM2 RX/pin3) is dead → one-way link, handshake never completes, no topics. jetson-io shows UART configured and the agent opens ttyTHS1 fine, so it's not the port/baud/ground (clean 1Hz RX proves ground good). Culprit is the pin-8 TX leg: either the pin8→TELEM2-pin3 wire (loose/wrong/open) or pin 8 not muxed as UART1_TX. Discriminator = loopback jumper pin8↔pin10 via flight_logs/nv_gpio_uart_test.py (already 921600). Full troubleshooting write-up added to DRONE_SETUP.md.
- Refined the ttyTHS1 TX diagnosis after noting the same 3 leads worked on the USB-TTL adapter: the wire isn't the suspect — the endpoints differ. The FTDI adapter's TX is an always-on driver; Jetson pin 8 is a shared SoC pad that only drives UART TX if pinmux says so, and RX (pin 10) working doesn't prove TX (pin 8) is muxed (separate pads/registers). Reordered DRONE_SETUP.md troubleshooting to lead with pin-8 output/pinmux (multimeter idle-3.3V check → loopback → jetson-io UART1, incl. the Arducam custom-DTB gotcha where jetson-io may patch the default DTB not the booted one). Camera-overlay-claims-pin-8 ruled out (no [used]/consumer on UART1 pads, nothing in dmesg).
- Read the live pinmux (sudo, read-only): UART1_TX_PR2 (pin110) and UART1_RX_PR3 (pin111) are in identical state (MUX/GPIO UNCLAIMED, no GPIO consumer) — configured as a pair by the boot pinmux, and RX works, so pin 8 is almost certainly muxed as TX too. No software asymmetry → pinmux is NOT the likely cause. Flips the top suspect to a physical pin-8 connection (TX lead off-by-one-hole / unseated / marginal socket contact — the only endpoint that changed vs the USB-TTL adapter). Loopback (pin8↔10) is the decisive test: pass → physical pin-8↔FC link; nothing → boot-pinmux/DTB issue in the custom Arducam -super.dtb. DRONE_SETUP.md troubleshooting updated accordingly.
- Root-caused the DroneSmoothPlanner "self-deactivation" from the bench run: NOT our code. PX4 auto-disarmed at the 10 s mark (params show COM_DISARM_PRFLT=10 = auto-disarm if no takeoff detected within 10 s of arming). Props-off + hand-held never registered a real takeoff, so the timer expired; disarm → px4_ros2 deactivates the mode (mode is active only while id()==nav_state && armed). Node was healthy to the last ms. Bench artifact — set COM_DISARM_PRFLT=-1 for bench, restore to 10 for flight; won't trigger with props + real takeoff. (takeoff_hold survived 28 s because its clean ~1.1 m lift latched the land-detector "in air", cancelling the timer.) Also flagged a separate real bug: DroneSmoothPlanner Approach computes end_pos = vehicle_pos + drogue_pose, adding the camera-frame drogue vector (z=forward ~9.7 m) straight into NED, so targets land ~9 m underground — needs a camera→NED transform.
- Fixed #1 (takeoff_land landed mid-climb): removed the executor's blind arm-time hold timer. TakeoffLand mode now owns hold_duration — after reaching target it hovers hold_duration then calls ModeBase::completed(Success); the executor lands on that completion (only if result==Success, so a failsafe/disarm doesn't trigger a land). Added executor safety_timeout (default 30 s) that lands anyway if the mode never completes, preserving guaranteed-land. Moved hold_duration to the mode + added safety_timeout to cfg/takeoff_land_params.yaml. Builds + registers clean (no param double-declare).
- Fixed drogue_flight `DroneSmoothPlanner` Approach (three related fixes; sixth_flight bench log showed it flying up/away and thrashing 110 replans). (1) **Frame transform**: `/tag_detections` is published by pose_estimation_node in a camera ranging frame (+x left, +y up, +z forward) and the mode was doing `end_pos = vehicle_pos + drogue_pose`, dumping the ~6 m forward range straight into NED-down → targets underground. Added `drogueTargetNed()` which maps ranging→body-FRD `(z,-x,-y)` then FRD→NED via `px4_ros2::OdometryAttitude` (same convention as the pose node's apply_attitude_correction), used for both the plan endpoint and the replan check; guards against a not-yet-ready attitude quaternion. (2) **Replan gating**: the old check replanned whenever `vehicle_pos + drogue_pose` drifted, so hand-carry motion against a *stale* drogue pose triggered ~5 Hz replans (each resetting path_index). Now replans only on a fresh detection (`_drogue_timestamp > _last_plan_time`), rate-limited by new `replan_min_interval` (0.5 s), and only if the absolute NED target moved past `replan_threshold` (raised 0.10→0.35 m). (3) Kept `drogue_timeout` (3 s) as the single Search↔Approach debounce — the sixth_flight bounce was a real 3 s YOLO dropout, not chatter. New params in config/drone_smooth_planner.yaml. Builds + registers clean. NOTE: assumes the forward YOLO cam (OAK-D Lite) optical axis ≈ body forward (no separate mount offset), matching the pose node; only the S2/VIO cam's 30° tilt is calibrated.
- Added `camera_pitch_deg` param (default 0.0) to DroneSmoothPlanner: rotates the drogue FRD vector by the forward YOLO cam's fixed mount pitch (+ve = tilted down) before the vehicle-attitude → NED rotation. Default 0 assumes the OAK-D Lite optical axis = body forward (unchanged behavior); dial in if bench logs show a consistent vertical bias.
- Investigated "camera frame dropout" concern (sixth/seventh flight bags + cuvslam logs). Finding: the VIO (S2) camera is NOT dropping frames — cuvslam reads the OAK directly over XLink (dai.Device/sync_q.tryGet, not a ROS topic) and logs frame_gaps=0 at a steady 30 fps (n≈300/10s, track() 4-6 ms) for the entirety of both flights, with no XLink/device errors. IMU fusion is disabled (enable_imu_fusion:false), so cuVSLAM never dead-reckons on the IMU regardless. The only VIO interruption was one hand-carry motion artifact ("implausible VIO speed 5.7 m/s" → reset, re-acquired in 6 frames). The low image rates in the bags (/driver/stereo/image_raw ~8.5 Hz, /rgb/image ~5.9 Hz) are a rosbag2/disk under-recording artifact — small PX4 telemetry records at full ~43 Hz in the same bag — not a live camera/estimator dropout. The genuinely intermittent stream is the forward Lite→YOLO drogue path (/detections ~2 Hz, 3 s /tag_detections gaps = inference rate + FOV during hand-carry + possible USB contention), which is separate from VIO.
- oak_d_visual_odometry: PX4 velocity is now the least-squares slope over a rolling `velocity_window_frames` (10) NED pose window instead of a two-frame diff (~5x less noise on bench-level jitter); the residual scatter of the same fit is reported to EKF2 as empirical position variance (`use_empirical_covariance`) since cuVSLAM's own covariance stays nominal through runaways. Window clears on tracking loss/stream gaps. (Ideas pulled from isaac_ros_visual_slam pose_cache.)
- oak_d_visual_odometry: `max_frame_delta_s` now auto-derives to 3 frame intervals (0.1 s @ 30 fps) when <= 0 so cuVSLAM itself treats stalls as discontinuities (library default 1.0 s; Isaac ROS uses ~1 frame period); exposed `multicam_mode` (PyCuVSLAM 16.0 default is already precision) and `debug_imu_mode` (bench IMU-extrinsics check for the down45 mount, requires enable_imu_fusion); `warm_up_gpu()` called before first track(); border masks mirrored per eye (outer non-overlap edge cut to 64 px per TROUBLESHOOTING.md step 8).
- oak_d_visual_odometry: treat cuVSLAM exact-identity covariance (keyframe/SBA "no estimate" sentinel, ~4% of samples on bench) as invalid in _px4_variances instead of forwarding 1.0 m^2 to EKF2; found during 2026-07-13 down45 static bench test (rest velocity RMS ~1 mm/s, gravity-verified 45.2 deg mount).
- oak_d_visual_odometry: add min_landmarks_3d scene-quality gate (default 10) after the 2026-07-13 cover test showed covered lenses produce ~450 self-consistent 2D noise tracks with landmarks3d=0 that beat the speed gate + reacquire hysteresis and pushed 163 m teleports to PX4 mid-blackout; gate treats low-landmark poses as tracking loss so re-acquire is impossible until real 3D structure returns.
- Fixed the 3 Hz Foxglove-over-hotspot problem (onboard drogue tracking is 15-20 Hz). Root cause: raw BGR images over the websocket (768 KB/frame = ~123 Mbps at 20 Hz vs ~15-30 Mbps hotspot goodput), 10 MB bridge send buffer queueing stale frames, WiFi power-save on, 208 KB kernel UDP ceiling. New `launch_scripts/foxglove_wifi.sh` (now used by super_real.sh window 7) runs JPEG republishers for /detections_image and /front/camera/image_raw (~5 Mbps at 20 Hz), whitelists the bridge to compressed images + telemetry, and caps send_buffer_limit at 1 MB; new `config/fastdds_wifi_profile.xml` (separate from the XRCE agent profile) gives ROS nodes a 16 MB SHM segment (default 512 KB is smaller than one frame, pushing onboard image traffic to UDP loopback), 1400-byte RTPS fragments (no IP fragmentation over WiFi), 4 MB non-blocking UDP buffers, and 60 s discovery lease; `services/99-udp-buffers.conf` (sysctl) and `services/wifi-powersave-off.conf` (NetworkManager) need one-time sudo installs. Point Foxglove image panels at the /compressed topics. Full diagnosis + verification: general_docs/comms_latency.md. Smoke-tested: bridge + both republishers up, only whitelisted channels advertised.
- oak_d_visual_odometry: acted on NVIDIA forum feedback — VehicleOdometry.timestamp_sample was the publish time, not the capture time, so EKF2 fused every pose ~1-2 frames late with EKF2_EV_DELAY unable to model it. Now set to ROS publish time minus the measured capture->publish latency (DepthAI host-synced getTimestamp vs time.monotonic_ns; cuVSLAM still gets device-clock stamps so frames+IMU stay on one drift-free clock). Implausible latency (<0 or >0.5 s) falls back to publish time with a one-shot warning. Also replaced the hardcoded quality=100 with 2x the triangulated 3D landmark count capped at 100 (gate floor 10 -> 20, good bench scenes 50+ -> 100) so EKF2_EV_QMIN can de-weight marginal scenes. With this fix EKF2_EV_DELAY should be ~0.
- foxglove_wifi.sh: added /tag_detections (drogue PoseStamped) to the bridge topic whitelist for the tracking demo.
- Added config/drogue_demo_foxglove.json: Foxglove demo layout (compressed detections image, /tag_detections 3D pose arrow, range/offset + confidence plots, raw XYZ readout).

## VIO camera moved to forward mount (2026-07-16)
- OAK-D S2 (VIO) physically swapped to the forward mount after 30/45-degree
  tilted mounts produced scale errors in flight; forward ground tests read
  near-perfect on /slam/odometry.
- `src/oak_d_visual_odometry/config/cuvslam_params.yaml`: `camera_mounting:
  down45` -> `forward`, comment updated. Rebuilt so the installed copy the
  launch file loads matches.
- `t_body_cam` left at [0,0,0]; measure the camera lever arm from the FC and
  set it before precision work (small effect for position-only EV fusion).

## OAK-D Lite removed; S2 CAM_A serves as the front camera (2026-07-16)
- cuvslam_params.yaml: rgb_topic/rgb_camera_info_topic now publish on
  /front/camera/image_raw + /front/camera/camera_info (both node sections),
  so drogue_detection_node, pose_estimation_node, and foxglove_wifi.sh run
  unchanged against the S2's color stream. Rebuilt.
- super_real.sh window 4 (depthai driver for the Lite) retired/commented.
- CAM_A runs at rgb_fps (15) — raise in cuvslam_params.yaml if detection
  needs more input frames. Ranging auto-adapts via the published camera_info.
- rgb_fps raised 15 -> 30 in both node sections (S2 CAM_A handles 30 Hz fine
  alongside the stereo pair), restoring the input rate the Lite provided.
- foxglove_wifi.sh: fixed duplicate node name on the features republisher
  (republish_front_camera x2 -> republish_features_image); dropped the dead
  /rgb/camera_info whitelist entry (topic renamed to /front/camera/camera_info).

## Docs cleanup (2026-07-16)
- Removed general_docs/comms_latency.md; the parts that actually mattered
  (foxglove_wifi.sh + JPEG republishing + whitelist + 1 MB send buffer) are
  now a short "Watching live flight data over WiFi" section in
  general_docs/reviewing_flight_data.md.
- Removed services/99-udp-buffers.conf and services/wifi-powersave-off.conf
  (no observed effect on the link). If they were ever installed, uninstall
  with: sudo rm /etc/sysctl.d/99-udp-buffers.conf
  /etc/NetworkManager/conf.d/wifi-powersave-off.conf && sudo sysctl --system
  && sudo systemctl restart NetworkManager.
- config/fastdds_wifi_profile.xml kept: foxglove_wifi.sh exports it and the
  16 MB SHM segment affects the onboard camera->YOLO path, not just WiFi.

## cuvslam_publisher_node.py readability pass + README refresh (2026-07-16)
- Behavior-preserving restructure: publisher wiring moved to
  _create_publishers(); the three PX4 gates + reacquire hysteresis collapsed
  into _pose_health_gate() with one _publish_pose call; the timestamp-domain
  bridge and landmark-scaled quality extracted to _px4_timestamp_sample_us()
  / _px4_quality(); grouping banners added in _declare_params. Verified with
  ruff + ty + colcon build (no runtime test — needs the camera).
- README: intro corrected (S2, not Lite), Quick Start section, single-camera
  note (CAM_A -> /front/camera at 30 fps), new Mounting section with presets
  + change checklist, parameter-table defaults fixed (height 480, rgb_fps 30).

## Flight-state tracker for precision_land modes (2026-07-16)
- New StatePublisher.hpp: latched std_msgs/String on <node>/state,
  re-published at 1 Hz so bags started mid-flight still contain it.
- TakeoffHold publishes /TakeoffHold/state: Arming -> Armed ->
  OpticalFlowInit -> Climbing -> Holding (-> Deactivated / Failed).
- TakeoffLand publishes /TakeoffLand/state: Arming -> TakingOff -> Hover ->
  HoverComplete -> Landing -> Landed -> Disarmed (-> Failed). Executor ctor
  narrowed to TakeoffLandMode& so mode+executor share one timeline.
- std_msgs added to package.xml/CMakeLists; '.*/state$' added to the
  foxglove_wifi.sh whitelist. View with a State Transitions panel on
  "/TakeoffLand/state.data" (works live and in bag replay).

## First successful VIO flight — configuration locked in (2026-07-16)
- Flight result: position lock, ~5 cm drift, takeoff_land mode flew (1 m,
  10 s hold, hard-ish landing). EKF2_EV_CTRL=15 required (velocity fusion
  kills the 10-20 m drift; yaw fusion needed to arm in Position mode).
- config/params/v1_16_0_oakd_vio.params: EKF2_EV_CTRL 13 -> 15 so reloading
  the snapshot can't regress the working config.
- New config/params/README.md: file index + flight-proven VIO parameter
  rationale + open tuning list (X/Y hold oscillation, MPC_LAND_SPEED).
- oak_d_visual_odometry README PX4 Notes: flight-proven EKF2 paragraph.

## TakeoffLand: slow descent before land() (2026-07-16)
- New Descending state: after the timed hover, the mode ramps its own
  setpoints down at descent_rate (0.3 m/s) to land_handoff_height (0.3 m
  above estimated ground), THEN signals completion so the executor's native
  land() only covers the final touchdown (PX4 land detector still owns it).
  Motivation: MPC_LAND_SPEED has a 0.6 m/s floor, too hard for this quad.
- New params in cfg/takeoff_land_params.yaml: descent_rate,
  land_handoff_height. State topic now shows ... Hover -> Descending ->
  DescentComplete -> Landing ...

## EV latency measured across all bags — EV_DELAY question answered (2026-07-17)
- Cross-correlated gyro vs /slam/odometry angular rate in every flight_logs
  bag: current pipeline capture->publish latency is ~380-450 ms (node's
  timestamp_sample claim of ~435 ms independently CONFIRMED honest).
  fifth_flight (July 9, OAK-D Lite era, old code) measured 43 ms — the
  regression came with the current config (suspects: CAM_A 30 fps through
  the VIO node, features publishing, detector subscription load).
- EKF2_EV_DELAY should stay ~5 ms (timestamp_sample already carries the
  latency; tuning it was never the fix). EKF2_DELAY_MAX=200 ms is SMALLER
  than the 435 ms sample age — fusion mis-registration during motion.
  Stopgap: raise EKF2_DELAY_MAX to 500. Real fix: find the ~13-frame
  backlog. Node stats line now prints ev_lat=<ms> for live measurement.
- Onboard-latency pass on the cuVSLAM node:
  - `/features/image` overlay is now skipped when nobody is subscribed and
    otherwise rate-capped by the new `features_max_fps` param (default 10).
    It cost a colorspace convert + per-feature draws + a ~768 KB publish on
    every frame, all inside the tracking loop.
  - Stereo sync output queue maxSize 4 -> 2. The queue is non-blocking, so
    when track() is slower than the camera it sits full and each extra slot
    is pure added pose age (4 slots at 30 fps = 133 ms of the 435).
  - `camera_fps` 45 -> 40 and `rgb_fps` 30 -> 10, from a measured sweep
    (2026-07-20). ROOT CAUSE OF THE 435 ms LATENCY FOUND: the OAK-D S2 has a
    device-wide budget of ~50 delivered frames/s SHARED between the stereo
    pair and CAM_A. It was never the sensors (OV7251 does 640x480 @ 117 fps)
    and never host compute (track() means 4-6 ms = ~200 Hz ceiling). The old
    45+30=75 request oversubscribed the device, so the sync queue backlogged
    and poses arrived ~400 ms stale.
      60+30=90 -> 22.4 Hz / 405 ms      45+10=55 -> 44.0 Hz / 242 ms
      60+10=70 -> 38.4 Hz / 245 ms      40+10=50 -> 40.8 Hz /  60 ms  <-- set
      50+10=60 -> 42.5 Hz / 222 ms      30+10=40 -> 30.0 Hz /  39 ms
    40/10 gives the full requested stereo rate, frame_gaps=0, and ev_lat back
    down to the 52+-27 ms fifth_flight baseline. Latency was preferred over
    the marginally higher 44 Hz at 45+10 because ev_lat is what
    EKF2_DELAY_MAX must cover.
  - Measured non-findings, recorded so they are not re-litigated:
    `publish_features` and `publish_imu` cost essentially nothing (55.3 and
    48.4 Hz with rgb off, vs 54.6 Hz with all three off). The feature-overlay
    throttle above is still worth keeping but it was NOT the bottleneck --
    RGB alone accounted for the entire 55 Hz -> 22 Hz collapse.
  - README gained a "Frame rate" section with the measurement table and the
    verification procedure (n/10 must equal camera_fps, frame_gaps 0,
    ev_lat < ~100 ms).
  - Confirmed on the shipped config: n=400-408/10s (exactly camera_fps=40),
    frame_gaps=0, and ZERO DepthAI "Delta between frames" warnings (was 281
    in 45 s at 60/30). ev_lat settles to 33-45 ms, but only after ~20 s --
    the first two stats windows still read ~220 ms. Let the node settle
    before arming.
  - BUILD GOTCHA (hit and documented in the package README): build with
    `python3 -m colcon build ... --symlink-install`, NOT `/usr/bin/colcon`.
    The system colcon's `#!/usr/bin/python3` shebang propagates into the
    generated console script, which then cannot import depthai/cuvslam (venv
    only) -> ModuleNotFoundError at launch. Also do NOT `source
    .venv/bin/activate` first; that writes metadata the launch interpreter
    cannot find (PackageNotFoundError). The venv is on PATH from the profile.
- Node now publishes measured rates on `~/diagnostics`
  (`diagnostic_msgs/DiagnosticArray`, 1 Hz) — added because Foxglove showed
  "~10 Hz" while the node was genuinely at 40. Counted at the publish() call,
  upstream of any transport, so it is immune to bridge throttling. Keys:
  tracked_fps, camera_fps_requested, odometry_fps, px4_fps, rgb_fps,
  features_fps, ev_latency_ms, landmarks_3d, reset_counter. Status goes ERROR
  on >20% stereo shortfall vs camera_fps, WARN on ev_latency_ms > 150.
  Added `diagnostic_msgs` to package.xml and `.*/diagnostics$` to the
  foxglove_wifi.sh whitelist.
- Verified true rates with `ros2 topic hz` (2026-07-20, 40/10 config):
  /slam/odometry 40.0 Hz, /fmu/in/vehicle_visual_odometry 40.2 Hz,
  /front/camera/image_raw 7.7 Hz, /features/image 7.3 Hz. The "10 Hz" the
  operator saw was an image topic and/or bridge throttling, NOT the pose
  stream. Documented in the README "Reading the real rate" section.
