# Stage 2 runbook: IMU and LiDAR sensor fusion

Part of the [overall plan](plan.md). Prerequisites:
- **Stage 1:** the repo running on our hardware;
- **the [paper reproduction](paper_runbook.md):** our baseline numbers.

**Goal:** a rover mapping system built from **the D435, an IMU and a LiDAR**, with this
repository's ORB-SLAM3 as the visual(-inertial) core. It must be evaluated, with ablations,
against a **reference map of the test environment made with a standard 3D LiDAR**.

**The gimbal is out of the project.** It was dropped because of weight-balancing problems.

**The rule from Stage 1 still holds: the ORB-SLAM3 library isn't modified.** Everything here
lives outside the core:
- our own driver programs (their own CMake project, like `repro/cpp`);
- ROS 2 packages;
- calibration files;
- evaluation scripts.

Every interaction with ORB-SLAM3 goes through its public API: `TrackStereo(imL, imR, t, vImu)`,
`GetTrackingState()`, the `Save*` functions, and run-time switches.

---

## S2-0. Decisions to make first

| # | Decision | Recommendation | Why |
|---|---|---|---|
| D1 | **Which IMU** | **Swap the D435 for a D435i** if at all possible. Otherwise an external IMU rigidly bolted to the D435 (S2-2) | The D435i's IMU sits in the camera body with **hardware timestamps on the same clock as the images** and a factory camera-IMU calibration. The repository already ships the driver for it (`Examples/Stereo-Inertial/stereo_inertial_realsense_D435i.cc`) and its settings template, so IMU fusion needs **no new code**. An external IMU adds two hard problems: time synchronisation and calibration |
| D2 | **Which LiDAR on the rover** | **RPLidar A1 (2D)**, which we already have, for fusion | Indoors on flat floors, 2D LiDAR SLAM is robust where vision struggles (dark, blank walls) |
| D3 | **The "standard 3D LiDAR" for the reference map** | Borrow or arrange a 3D LiDAR (e.g. Ouster/Velodyne/Livox Mid-360, or a terrestrial laser scanner) to map the test environment once | Gives an independent, more accurate reference for both map quality and trajectories |
| D4 | **Ground-truth trajectories** | Preferred: the same 3D LiDAR **carried on the rover** during evaluation runs, localised in the reference map. Fallback: surveyed floor markers | Ablations need a reference trajectory, not only a reference map (S2-6) |
| D5 | **Middleware** | **ROS 2 Humble** (matches JetPack 6 / Ubuntu 22.04) | Standard drivers (IMU, RPLidar, RealSense), time-stamped recording (`rosbag2`), `slam_toolbox` and `robot_localization` |

Record the decisions in `plan/progress_log.md` before buying or building anything.

---

## S2-1. Target architecture

```
                   ┌───────────── Jetson Orin Nano (ROS 2 Humble) ─────────────┐
 D435 IR L/R ─────►│ realsense-ros ──┐                                         │
 IMU ─────────────►│ imu driver ─────┼─► orbslam3_ros (ours) ──► /orb/odom     │
                   │                 │    TrackStereo(imL, imR, t, vImu)        │   rosbag2:
 RPLidar A1 ──────►│ sllidar_ros2 ───┼─► slam_toolbox ──► /map, map→odom       │   every sensor
                   │                 └─► robot_localization EKF                 │   topic, every run
                   │                       (ORB VIO + (wheels later)) ─► odom→base_link
                   └────────────────────────────────────────────────────────────┘
 Evaluation (PC): trajectories + maps  vs  3D-LiDAR reference map / ground-truth trajectory
```

**Two kinds of fusion:**
1. **IMU fusion is tight, inside ORB-SLAM3.** The repository already implements stereo-inertial
   SLAM (`IMU_STEREO`); the paper's main configuration is stereo-inertial. We only have to deliver
   correctly calibrated, time-aligned IMU samples with each stereo pair. This is the standard
   ORB-SLAM3 path and needs no library change.
2. **LiDAR fusion is loose, outside ORB-SLAM3.**
   - ORB-SLAM3 has no LiDAR input, and adding one would mean changing the library.
   - Instead, ORB-SLAM3's visual-inertial pose goes to an EKF (`robot_localization`) as relative
     motion. The EKF's output is the odometry for `slam_toolbox`, the 2D LiDAR SLAM.
   - `slam_toolbox` does LiDAR scan matching and LiDAR loop closure, and owns the global `map`
     frame.
   - Result: vision and IMU give smooth, metric local motion, and the LiDAR gives global
     consistency and robustness where vision fails.

**What the rover outputs:**
- a 6-DoF trajectory: x, y and yaw from the fused estimate; z, roll and pitch from ORB-SLAM3;
- a 2D occupancy map from the LiDAR;
- a 3D point cloud: ORB-SLAM3's sparse map, plus a dense cloud built from D435 depth frames
  placed at the fused poses (S2-5).

---

## S2-2. IMU hardware

**Option A, recommended: D435i.** It's a mechanical swap, with no other hardware work.
- Its IMU is a BMI055: accelerometer up to 250 Hz, gyro up to 400 Hz.
- librealsense time-stamps it on the same clock as the images.
- Keep the same camera mount (Stage 1), and re-run Stage 1, Part E for the new unit's intrinsics.

**Option B: an external IMU, if a D435i is impossible.**
- **Choice:** an industrial IMU with a ROS 2 driver, **or** a MEMS IMU (Bosch BMI088 or TDK
  ICM-42688-P) on a microcontroller (Teensy 4.x, RP2040 or ESP32).
  - Data rate ≥ 200 Hz; ORB-SLAM3's examples use 200 Hz.
  - Gyro noise density ≲ 0.01 °/s/√Hz.
- **Mounting:** **rigidly** on the same bracket as the D435, within a few cm of it, with no
  dampers *between* camera and IMU. Dampers go between the bracket and the rover, never between
  the two sensors.
- **Time synchronisation**, the critical part. ORB-SLAM3 doesn't estimate a camera-IMU time
  offset online, so it has to be right at the input. In order of preference:
  1. **Hardware.** Use the D435's sync connector in master mode (`inter_cam_sync_mode = 1`) to
     output a per-frame pulse. The microcontroller time-stamps that pulse with the same clock as
     the IMU samples, so frames and IMU samples share one time base.
  2. **Software, with a calibrated offset.** Both streams are stamped on arrival on the Jetson.
     Kalibr estimates the constant offset (S2-3), and the driver adds it to every IMU timestamp.
     Repeat the calibration until the offset is stable to ±1 ms.
- **Microcontroller firmware** (if used): it sends `t_us, gx, gy, gz, ax, ay, az` over USB serial
  at 200–400 Hz, with timestamps taken at the sensor's data-ready interrupt. A small ROS 2 node
  republishes it as `sensor_msgs/Imu` with the timestamp converted to host time (offset plus
  drift, fitted continuously).

**Done when:**
- the IMU publishes at a steady rate (check the rate and jitter with `ros2 topic hz`);
- gravity reads ≈ 9.81 m/s² when level;
- the gyro reads ≈ 0 at rest.

---

## S2-3. Calibration

Everything in this section goes into **one new settings file**:
`fusion/config/RealSense_D435_IMU.yaml`. It's a copy of `Examples/Stereo-Inertial/RealSense_D435i.yaml`
with our values, and the original template stays untouched.

**1. Camera intrinsics and baseline.** As in Stage 1, Part E: the factory values, which the
driver prints at start-up. For a D435i, read them from the new unit.

**2. IMU noise (Allan variance).**
- Record the IMU **perfectly still** for ≥ 3 h, in the same thermal state as during runs:
  `ros2 bag record /imu` on the bench.
- Compute the Allan deviation, e.g. with `allan_variance_ros`. You get four values:

  | Settings key | Quantity | Unit |
  |---|---|---|
  | `IMU.NoiseGyro` | gyro noise density | rad/s/√Hz |
  | `IMU.NoiseAcc` | accel noise density | m/s²/√Hz |
  | `IMU.GyroWalk` | gyro random walk | rad/s²/√Hz |
  | `IMU.AccWalk` | accel random walk | m/s³/√Hz |

- These are **continuous-time** values, which is what ORB-SLAM3 expects (compare
  `Examples/Stereo-Inertial/EuRoC.yaml`).
- Multiply them by **2–10** before use. Real motion, vibration and temperature make things
  noisier than a bench test, and this is standard practice. Start with ×5, and keep the factor in
  the file's comments.
- Set `IMU.Frequency` to the measured rate.

**3. Camera-IMU extrinsic and time offset (Kalibr).**
- **Target:** an AprilGrid printed large and flat. The repo has a definition at
  `Examples/Calibration/recorder_empty/april_6x6_80x80cm_larues.yaml`.
- **Recording:** the IR projector **off** (the dots ruin target detection), good light, and a
  short exposure. Move for 60–90 s, exciting **all six axes** (three rotations, three
  translations) while keeping the target in view. Avoid motion blur.
- **Run:** `kalibr_calibrate_imu_camera` with the camera's intrinsics fixed and the IMU noise from
  step 2. Kalibr runs in its own Docker/ROS 1 environment on the PC; export the recording from
  the bag.
- **Accept the result only if:** the reprojection error is ≲ 0.3 px, the gyro and accel residuals
  are white-noise-like, and **3 independent calibrations agree** to within ~5 mm, ~0.5° and ~1 ms.
- **Convert it correctly. This is the usual mistake.** ORB-SLAM3's `IMU.T_b_c1` maps
  **camera-frame points into the IMU (body) frame**. That's the pose of the left camera in the IMU
  frame. Checked: `Examples/Stereo-Inertial/EuRoC.yaml` holds exactly EuRoC's published cam0
  `T_BS`. So:
  - write **Kalibr's `T_ic` (cam0 → imu0)** into `IMU.T_b_c1`, **not** `T_ci`;
  - ignore the comment in `RealSense_D435i.yaml` that says "body-frame (imu) to left camera".
    It's misleading.
- **The time offset** (Kalibr's `timeshift_cam_imu`, where t_imu = t_cam + shift) is applied in
  **our driver**, to every IMU timestamp, before the samples go to `TrackStereo`.

**4. Camera-LiDAR and base frames.**
- **Mechanical:** measure the positions of the D435 left IR camera, the IMU and the LiDAR centre
  relative to `base_link` (the rover centre on the floor) from CAD or with a tape measure.
  Write them into a URDF for `robot_state_publisher`.
- **Refine the LiDAR-to-camera yaw**, the angle that matters most in 2D, by aligning trajectories.
  Drive a loop, compute the ORB-SLAM3 and `slam_toolbox` trajectories, and fit the rotation
  between them with planar hand-eye alignment. Then update the URDF.
- **Check:** project the LiDAR scan into the D435 image; walls must line up with the image.

**Done when:** the settings file and URDF are committed (via the PC, as in Stage 1), and the
calibration reports are archived under `fusion/config/calib_reports/`.

---

## S2-4. Visual-inertial ORB-SLAM3 on our sensors

**1. Driver.**
- **Option A (D435i):** use the repository's own `stereo_inertial_realsense_D435i` with our
  settings file. No code is needed.
- **Option B (external IMU):** our own program, `fusion/cpp/stereo_inertial_d435_extimu.cc`, built
  by its own CMake project.
  - Modelled on the repository's `stereo_inertial_realsense_D435i.cc`: it collects the IMU
    samples between consecutive stereo frames, applies the time offset, interpolates the
    accelerometer to the gyro timestamps (the same `interpolateMeasure` approach), and calls
    `TrackStereo(imL, imR, t, vImu)`.
  - It adds the Stage 1 rover driver's features: clean `q`/Ctrl-C stop, saved trajectories, and
    the events log.

**2. Recording format.**
- Every run is recorded so it can be replayed under every configuration on identical data.
- **For the ORB-SLAM3-only ablations:** extend `repro/cpp/d435_record_euroc` (ours) to also write
  `mav0/imu0/data.csv` (`ts,wx,wy,wz,ax,ay,az`). The repository's **unmodified**
  `stereo_inertial_euroc` then replays our recordings in stereo-inertial mode, and `stereo_euroc`
  in stereo-only mode, from the same folder.
- **For the fused system:** `rosbag2` with every sensor topic (S2-5).

**3. Bring-up checks.**
- **Static start:** after start-up, the log shows the IMU initialisation steps
  (`start VIBA 1` … `end VIBA 2`, as in our EuRoC runs).
- **Gravity:** the map's vertical axis is aligned with gravity. Check this in the viewer: the
  floor points should lie horizontal.
- **Handheld loop with fast turns:** stereo-inertial keeps tracking where stereo-only lost
  tracking in Stage 1.
- **Scale:** a measured 5 m line comes out as 5 m ± 2 %.

**4. First ablation** (repeat each 3 times). Recorded sequences, replayed with `repro/run_matrix.py`
using two new modes:
- `d435imu_si`: `stereo_inertial_euroc` + our settings;
- `d435imu_s`: `stereo_euroc`, the same recording with the IMU ignored.

Compare stereo against stereo-inertial on: tracking losses, maps created, loop closures, scale
error on the measured line, and tracking time (the IMU adds preintegration cost).

**Done when:** stereo-inertial tracks the Stage 1 routes, plus a fast-turn route, without losses,
and is at least as accurate as stereo-only against the measured references.

---

## S2-5. LiDAR fusion (ROS 2)

**1. Software on the Jetson.**
- `ros-humble-ros-base`, `ros-humble-robot-localization`, `ros-humble-slam-toolbox`,
  `ros-humble-robot-state-publisher`, `ros-humble-foxglove-bridge`.
- `sllidar_ros2`, Slamtec's current ROS 2 driver; the A1 launch file publishes `/scan`.
- `realsense-ros` **built from source against our librealsense 2.55.1** (the matching 4.55.x tag),
  with the IR streams on, depth/colour off, and the projector off.

**2. `orbslam3_ros` (ours): a colcon package linking `libORB_SLAM3.so`, public API only.**
- **Subscribes** to the IR left/right images (exact-time synchronised) and the IMU.
- **Each frame:** calls `TrackStereo(imL, imR, t, vImu)`, as in S2-4.
- **Publishes** `/orb/odom` (`nav_msgs/Odometry`) and `/orb/state`.
- **Frame conversion:** ORB-SLAM3 reports the IMU (body) pose in stereo-inertial mode. Convert it
  to `base_link` with the URDF transform, and from the optical convention to ROS axes.
- **Only publish while tracking is OK.** On a map reset, or a jump larger than 0.2 m or 10°
  between frames (from a loop correction or a new map), **re-anchor**: keep the published pose
  continuous. The EKF uses the pose as *relative* motion (`differential: true`), so jumps must
  never reach it.
- **Build:** use the same include paths and flags as the repository (`-O3 -march=native`, C++14
  for the library; ROS 2 needs C++17, and the combination links fine). Set `LD_LIBRARY_PATH`/RPATH
  to the repo's `lib/` and `Thirdparty/*/lib`.

**3. EKF (`robot_localization`, `two_d_mode: true`).**
- `odom0` = `/orb/odom`, with x, y and yaw as **differential** inputs.
- Later, wheel odometry can be added as `odom1` (vx, vyaw).
- It publishes `odom → base_link`.

**4. `slam_toolbox`, online async mapping, configured for the A1:**
- `max_laser_range: 12.0`, `resolution: 0.05`;
- `minimum_travel_distance: 0.2`, `minimum_travel_heading: 0.2`;
- `do_loop_closing: true`.

It publishes `map → odom`.

**5. Dense 3D map** (for comparison with the 3D-LiDAR reference).
- Stream D435 depth as well, aligned to IR left.
- **The projector problem:** ORB-SLAM3 needs the projector **off**, but depth is best with it
  **on**. Two ways to handle it:
  - (a) passive depth with the projector off. Fine in textured rooms.
  - (b) `RS2_OPTION_EMITTER_ON_OFF`: the projector alternates every frame. Projector-off frames
    go to ORB-SLAM3, projector-on frames give depth, each at 15 FPS.
- **Offline:** integrate the depth frames at the final fused poses into a TSDF (Open3D), and
  extract a point cloud.

**6. Recording, every run:**
```
ros2 bag record /camera/.../infra1/image_rect_raw /camera/.../infra2/image_rect_raw \
  /camera/.../infra1/camera_info /imu /scan /tf_static [depth topics]
```
Check the real topic names with `ros2 topic list`. Replaying the bag (`--clock`,
`use_sim_time: true`) re-runs any configuration on identical data.

**Done when:**
- a lit indoor loop gives a consistent 2D map, and the fused trajectory has no jumps;
- in the visual-failure tests (lights off, blank wall) the fused system keeps tracking while
  ORB-SLAM3 alone loses track.

---

## S2-6. Reference: the environment mapped with a 3D LiDAR

**1. Reference map.**
- Map the test area once with the 3D LiDAR. A terrestrial scanner is ideal; otherwise a 3D LiDAR
  plus a LiDAR-inertial SLAM with loop closure, such as FAST-LIO2 + loop closure, or LIO-SAM.
- Export a point cloud in metres: `reference_map.pcd`.
- **Validate it independently:** tape-measure 5–10 distances (room sizes, door spacing). The map
  must agree to ≲ 1–2 cm. Archive the measurements.

**2. Ground-truth trajectory of the rover** (decision D4).
- **Preferred:** mount the same 3D LiDAR on the rover, rigidly, with its extrinsic measured to
  `base_link`.
  - Record it in the same bag as the other sensors.
  - Offline, localise each scan in `reference_map.pcd` with scan-to-map registration (e.g. GICP
    or NDT, starting from the LiDAR-inertial odometry).
  - This gives `base_link` poses accurate to a few cm: the **ground-truth trajectory**, which
    plays the role EuRoC's ground truth played in the paper.
- **Fallback, without a 3D LiDAR on the rover:**
  - place 8–12 **surveyed floor markers** whose positions are read from the reference map, and
    stop the rover on each, logging the time;
  - **AprilTags** on the walls, surveyed in the reference map, observed by the D435 (detected
    offline, so the SLAM is unaffected);
  - plus the end-point error of closed loops.

**3. Common frame.** Align each estimate to the ground truth with Umeyama SE(3) over the whole run
(trajectory metrics). For maps, the estimated map is transformed with the same alignment, then
refined with ICP.

---

## S2-7. Ablations and metrics

**Configurations,** all replayed from the **same bags**, routes, operator and lighting:

| ID | Configuration | Shows |
|---|---|---|
| C0 | Stereo only (Stage 1 system) | Baseline |
| C1 | Stereo-inertial (D435 + IMU) | What the IMU adds |
| C2 | LiDAR only (`slam_toolbox` + LiDAR scan-matching odometry, e.g. `rf2o_laser_odometry`) | LiDAR baseline |
| C3 | Stereo + LiDAR (EKF with ORB stereo) | LiDAR fusion without the IMU |
| C4 | **Stereo-inertial + LiDAR**, the full system | The final system |
| C1-cpu / C4-cpu | As C1 / C4 with `CPU_ORB=1` | The paper's GPU ≡ CPU claim on our full system |
| C1-pipe | C1 with `PIPELINE_FE=1` | Real-time margin |
| C4-nolc | C4 with ORB-SLAM3 loop closing off | What visual loop closure adds when LiDAR closes loops too |

**Routes:**
- R-A: lit closed loop;
- R-B: out-and-back corridor;
- R-C: low texture and lights partly off;
- R-D: long uniform corridor, where 2D LiDAR alone is weak;
- R-E: fast turns.

**3 runs** per configuration and route.

**Metrics:**

| Metric | Tool | Notes |
|---|---|---|
| ATE (SE(3), cm) and RPE against the ground-truth trajectory | `repro/evaluate.py` | Extended with a loader for TUM-format ground truth (S2-8) |
| Map accuracy: cloud-to-reference distance (mean, RMSE, % within 5 / 10 cm) | `fusion/eval/map_compare.py` (Open3D, on the PC) | After the S2-6 alignment and ICP refinement |
| Map completeness: % of reference points within 10 cm of the estimate | same | Penalises maps that cover only part of the area |
| Robustness: tracking losses, maps created, loop closures, EKF jumps | console and events logs | As in Stage 1 |
| Runtime: tracking time, FPS, CPU/GPU load, power | `tegrastats`, logs | As in the paper reproduction |

**Report:** one table per route, configurations against metrics, as median over runs with the
spread. Plus the GPU ≡ CPU check (C1 vs C1-cpu, C4 vs C4-cpu) in the paper's format.

---

## S2-8. Code we'll add (outside the core)

| Path | What it does |
|---|---|
| `fusion/config/RealSense_D435_IMU.yaml` | Calibrated settings (S2-3) |
| `fusion/config/urdf/rover.urdf` | `base_link` → camera, IMU, LiDAR |
| `fusion/cpp/stereo_inertial_d435_extimu.cc` (+ CMake) | Option B driver (S2-4) |
| `repro/cpp/d435_record_euroc.cc` | Add an IMU stream → `mav0/imu0/data.csv` (S2-4) |
| `repro/common.py`, `run_matrix.py` | Modes `d435imu_si` / `d435imu_s` (S2-4) |
| `fusion/ros2/orbslam3_ros/` | ROS 2 wrapper node (S2-5) |
| `fusion/ros2/rover_bringup/` | Launch files, EKF and `slam_toolbox` configs |
| `fusion/eval/gt_from_lidar.py` | Scan-to-map localisation → ground-truth trajectory (S2-6) |
| `fusion/eval/map_compare.py` | Cloud-to-reference metrics (S2-7) |
| `repro/evaluate.py` | TUM-format ground-truth loader; batch scoring of ROS-bag runs |

---

## S2-9. Order of work and time

| Step | Content | Rough time |
|---|---|---|
| S2-0 | Decisions D1–D5 | 1 day (procurement may take longer) |
| S2-2 | IMU hardware and timing | 1 day (D435i) / 1–2 weeks (external) |
| S2-3 | Calibration (Allan 3 h + Kalibr ×3) | 2–4 days |
| S2-4 | Visual-inertial bring-up and first ablation | 3–5 days |
| S2-5 | ROS 2 stack, wrapper, EKF, `slam_toolbox` | 1–2 weeks |
| S2-6 | 3D reference map and ground truth | 3–7 days (depends on LiDAR access) |
| S2-7 | Recording campaign and ablations | 1–2 weeks |
| Report | Tables, plots, conclusions | 3–5 days |

## Main risks

| Risk | Mitigation |
|---|---|
| External IMU time offset not constant | Hardware sync (S2-2, option 1). Otherwise a D435i |
| Wrong `IMU.T_b_c1` direction | Use Kalibr `T_ic` (S2-3). The bring-up gravity check (S2-4) shows an error immediately |
| IMU initialisation fails on a slow rover | ORB-SLAM3 needs motion to initialise. Start each run with a few seconds of handheld-style excitation, or drive a short S-curve. Check for `end VIBA 2` in the log |
| No 3D LiDAR available on the rover | Fallback ground truth with surveyed markers and AprilTags (S2-6) |
| ORB jumps corrupt the EKF | Re-anchoring in `orbslam3_ros` (S2-5) |
| Poor light (seen in Stage 1) | Lit routes for C0/C1; dark routes (R-C) deliberately test the LiDAR fusion |
| CPU load: ROS 2 + ORB-SLAM3 + `slam_toolbox` on 6 cores | Measure early; use `PIPELINE_FE`; drop the viewer; dense mapping offline only |
