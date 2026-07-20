# PX4 Parameter Sets

QGroundControl parameter snapshots for tested drone configurations. Load the
one matching your PX4 version and sensor setup (QGC → Vehicle Setup →
Parameters → Tools → Load from file).

| File | Configuration |
|---|---|
| `1_15_OF.params` | PX4 v1.15, optical flow |
| `sim_1_16.params` | PX4 v1.16 SITL |
| `v1.16.0_arkflow.params` | PX4 v1.16, ARK Flow optical flow |
| `v1_16_0_oakd_vio.params` | PX4 v1.16, OAK-D S2 cuVSLAM VIO (forward mount) |

## VIO external-vision parameters (flight-proven 2026-07-16)

First successful VIO position lock: forward-mounted OAK-D S2, cuVSLAM
stereo-only, ~5 cm odometry drift while holding. These settings are the
result of that flight — do not "simplify" them without re-flying:

- **`EKF2_EV_CTRL = 15`** (horizontal pos + vertical pos + velocity + yaw).
  This is the full-fusion value and it is REQUIRED, not optional:
  - Without velocity fusion the odometry drifted 10–20 m.
  - Without yaw fusion PX4 refuses to arm in Position mode (horizontal
    position estimate never reaches "stable").
  - With all four: ~5 cm drift.
- `EKF2_GPS_CTRL = 0`, `EKF2_HGT_REF = 2` (vision height) — GPS-denied.
- `EKF2_EV_DELAY`: the cuVSLAM node stamps `timestamp_sample` with the true
  camera capture time (since 2026-07-16), so this only needs to cover
  residual transport lag — keep it small (0–10 ms), not the 175 ms-style
  values vision setups historically needed.
- `EKF2_EV_POS_X/Y/Z` must stay 0: the camera→body lever arm belongs in
  `t_body_cam` in `cuvslam_params.yaml` (the node transforms the pose to the
  body frame itself). Setting both double-counts the offset.

Known-open tuning (safe to fly, not yet ideal):

- **X/Y oscillation in hold**. As-flown on 2026-07-16: `EKF2_EV_DELAY=50`,
  `MPC_XY_P=0.25`, `MPC_XY_VEL_P_ACC=1.8`, `MPC_XY_VEL_I_ACC=0.4`,
  `t_body_cam` unset. Fixes to try, in order:
  1. `t_body_cam = [0.125, 0.0, -0.025]` — set in cuvslam_params.yaml
     (2026-07-16); attitude jitter was being reported as X/Y motion.
  2. `EKF2_EV_DELAY` 50 -> 5: with capture-time stamping, 50 ms is pure
     artificial mis-registration between the pose and the IMU history —
     phase error that shows up exactly as hover oscillation.
  3. MPC_XY_P at 0.25 is far below the 0.95 default — it was likely
     lowered chasing this same oscillation. Once 1+2 are in, walk it back
     up (0.5 -> 0.8) or the hold will feel mushy and wander.
  The velocity-loop gains (P_ACC 1.8, I_ACC 0.4) are PX4 defaults; leave
  them until the estimation-side fixes are flown.
- **Hard landing**: `MPC_LAND_SPEED` lowered 0.7 -> 0.6 (the parameter's
  minimum). `MPC_LAND_CRWL` can't help — it only engages with a distance
  sensor, which this airframe doesn't have. If 0.6 m/s still lands hard,
  the remaining lever is mode-side: have the external mode descend on its
  own trajectory setpoints (e.g. 0.3 m/s) to just above the ground before
  calling the native land().
