# Stage 3 (future): wheel encoders + RPLidar A1, and fusion analysis

Part of the [overall plan](plan.md). **Not started, and not needed for the core demonstration.**
Pick it up after [Stage 1](stage1.md). [Stage 2](stage2.md) is optional before it.

**Status: not planned yet.** The current scope is Stage 1 only (see [plan.md](plan.md)).

**Goal:** add the two sensors together, **wheel encoders** and the **RPLidar A1** 2D LiDAR. Then
measure what fusing them with ORB-SLAM3 gains over each on its own.

The rule stays the same: **the ORB-SLAM3 library is not modified.** Fusion is **loose**:
ORB-SLAM3 runs as it does in Stage 1, and a separate filter combines its output with the other
sensors.

**Why encoders and LiDAR together:** 2D LiDAR SLAM works much better with odometry. The A1 scans
slowly (~5.5–10 turns/s). Between scans the rover may have moved noticeably, and scan matching
without a motion estimate to start from fails more easily. In long uniform corridors, scan
matching can't tell how far along the corridor the rover is. Wheel odometry fills both gaps. It
also gives a third, independent motion source for the analysis.

**Questions this stage answers:**
1. Does fusing ORB-SLAM3 into the LiDAR system reduce drift compared with LiDAR + wheels alone?
2. Where vision fails (darkness, blank walls, direct sun), does the fused system keep going?
3. Where 2D LiDAR struggles (long uniform corridors, glass), does ORB-SLAM3 help?
4. How does ORB-SLAM3 alone (the Stage 1 system) compare with the fused system?

---

## 1. Open decisions (answer before starting)

| Decision | Why it matters |
|---|---|
| **Rover drive type**: differential (2 driven wheels plus a caster), skid-steer (4 wheels), or tracks | Sets the odometry model. Skid-steer and tracks slip in turns, so their effective track width must be calibrated (section 4.3) |
| **Motor controller**: does it already report encoder counts (e.g. RoboClaw, ODrive, a hoverboard board)? | If yes, read counts from it. If not, add encoders and a microcontroller |
| **Encoders**: are there motors with built-in quadrature encoders, or do encoder discs need adding? | Hardware purchase and mounting |
| **LiDAR position**: a mast above the camera, or at the front below it | The scan plane must see 360° (or a known sector) without hitting the rover, gimbal or camera |
| **Viewing tool on the PC** | **Fedora PC** (ours): ROS 2 isn't packaged for Fedora, so use **Foxglove Studio** with `foxglove_bridge` on the Jetson, or `rviz2` on the Jetson over VNC (Stage 1, runbook Part V). **Ubuntu 22.04 PC:** install ROS 2 **Humble**, the same version as the Jetson, and run `rviz2` directly on the PC (`sudo apt install -y ros-humble-desktop` after adding the ROS 2 apt repository). Both machines need the same `ROS_DOMAIN_ID` on the same network. **Other Ubuntu versions:** use Foxglove, because mixing ROS 2 versions across machines isn't reliable |

---

## 2. Hardware

### 2.1 RPLidar A1
| Property | Value |
|---|---|
| Type | 2D, 360° single-plane laser scanner |
| Range | ≈ 0.15–12 m |
| Sample rate / scan rate | ≈ 8000 samples/s / ≈ 5.5 Hz typical (adjustable up to ~10 Hz) |
| Angular resolution | ≈ 1° or better |
| Interface | UART through its USB adapter board; appears as `/dev/ttyUSB*`; 115200 baud |
| Power | From the USB port (the motor and laser draw a few hundred mA) |

**Mounting:**
- **The scan plane must be horizontal and clear.** Nothing belonging to the rover may cut the
  plane: no gimbal, camera, mast or cables. Anything that does appears as a fake obstacle at a
  fixed distance, and has to be masked out.
- The best spot is usually on top, above the camera and gimbal, on a small mast. If the camera is
  higher, mount the LiDAR at the front below it, with the blocked rear sector masked.
- Rigidly mounted (no dampers that let it tilt), with its forward arrow along the rover's x axis.
  Measure the offset from `base_link` (section 4.1).
- Give it a udev symlink `/dev/rplidar`, the same way as the STorM32 in Stage 2.

### 2.2 Wheel encoders
- Quadrature encoders on **each driven side**, either built into the motors or added on the
  wheel shafts.
- If the motor controller doesn't report counts: a small **microcontroller** (Arduino Nano,
  ESP32 or Teensy) counts encoder edges using interrupts. It sends
  `<millis>,<ticks_left>,<ticks_right>\n` over USB serial at 50 Hz, as cumulative counts, so a
  lost line never loses distance.
- Check the logic levels: 5 V encoders on a 3.3 V microcontroller need level shifting.
- Record the **ticks per wheel revolution** (after quadrature ×4) and the **wheel radius**.

### 2.3 Gimbal during Stage 3 runs
Fusion needs a **known camera-to-rover transform**. A camera turned by the gimbal breaks that
unless the angles are tracked.

For the fusion experiments:
- **Lock the yaw axis rigid** (yaw motor off, or mechanically locked facing forward). Keep pitch
  and roll stabilised.
- **No coverage or recovery sweeps during fusion runs.** If the gimbal is used anyway, the ORB
  input to the filter is paused during sweeps (section 3.3).

The remaining error: when the rover tilts, the stabilised camera stays level, so its pitch and
roll differ slightly from the rover's. The filter runs in 2D mode (x, y, yaw only), so this only
matters on steep slopes.

---

## 3. Software architecture (ROS 2 Humble)

JetPack 6 is Ubuntu 22.04, which matches **ROS 2 Humble**.

### 3.1 Packages

| Package | Source | Role |
|---|---|---|
| `ros-humble-ros-base`, `ros-humble-tf2-ros`, `ros-humble-robot-state-publisher` | apt | Core |
| `realsense2_camera` (realsense-ros) | **built from source against our librealsense**; use the release matching its version | Publishes the IR stereo pair (`.../infra1/image_rect_raw`, `.../infra2/image_rect_raw`). The prefix depends on version and namespace: realsense-ros 4.x defaults to `/camera/camera/`. Check with `ros2 topic list` |
| `rplidar_ros` | apt (`ros-humble-rplidar-ros`) or source | Publishes `/scan` from the A1 (`rplidar_a1_launch.py`) |
| `slam_toolbox` | apt | 2D LiDAR SLAM; publishes `map`, and `map → odom` |
| `robot_localization` | apt | EKF combining wheel and visual odometry; publishes `odom → base_link` |
| `foxglove_bridge` | apt | Live view in Foxglove Studio on the PC |
| **`orbslam3_ros`** | **ours** | Runs ORB-SLAM3 on the IR topics; publishes its pose as odometry |
| **`wheel_odom`** | **ours** | Encoder counts → `/wheel/odom` |
| **`rover_bringup`** | **ours** | URDF (fixed transforms), launch files, parameter files |

**Why the camera goes through `realsense2_camera`** instead of librealsense inside our node (as in
the Stage 1 driver): the images become ROS topics, so they can be **recorded in a bag and
replayed**. Every configuration in section 5 can then run on **exactly the same data**.

### 3.2 Frames (TF tree)
```
map ──(slam_toolbox)──► odom ──(robot_localization EKF)──► base_link ─┬─(static)─► laser
                                                                      └─(static)─► camera_link ─(static)─► camera_infra1_optical_frame
```
- `base_link`: the rover's centre on the ground plane; x forward, y left, z up (ROS convention).
- Static transforms come from a **URDF** in `rover_bringup`, published by
  `robot_state_publisher`, with offsets measured in section 4.1.
- **Only one node publishes each transform.** ORB-SLAM3's pose is published as a **topic**
  (`/orb/odom`), not as a TF, so it never conflicts with the EKF or slam_toolbox.

### 3.3 `orbslam3_ros`: the ORB-SLAM3 wrapper (public API only)

**Inputs:** the IR stereo topics, synchronised with `message_filters` (exact time; the D435 stamps
both IR frames identically).

**Core loop:** for each stereo pair, `Tcw = SLAM.TrackStereo(imL, imR, t)`, then read
`GetTrackingState()`, as in the Stage 1 rover driver. It also writes the same `events.csv`,
`console.log` and trajectory files, so Stage 1's `summarize_run.py` works unchanged.

**Converting ORB-SLAM3's pose to rover motion:**
- ORB-SLAM3's world frame is the **optical frame of the first camera pose**: x right, y down,
  z forward.
- The camera pose in that world is `Twc = Tcw.inverse()`.
- Rover motion relative to where it started:
  ```
  T_start_base(t) = T_base_cam · T_link_opt · Twc(t) · T_opt_link · T_cam_base
  ```
  - `T_link_opt` is the fixed rotation from the optical frame to the ROS camera frame (x forward,
    y left, z up): rpy(−π/2, 0, −π/2);
  - `T_base_cam` is the measured camera mount transform (URDF); `T_cam_base` is its inverse.

  At the first frame, `Twc` is the identity, so this is the identity: the rover starts at the
  origin.
- Published as `nav_msgs/Odometry` on `/orb/odom`: frame `odom`, child `base_link`, pose only,
  with a fixed covariance tuned in section 4.4.

**Keeping `/orb/odom` a smooth motion signal.** The EKF uses it as *relative* motion
(`differential: true`), so jumps must never reach it:
- **Only publish when the state is OK (2).**
- **Re-anchor on discontinuities.** The wrapper keeps an offset `T_off` and publishes
  `T_off · T_start_base(t)`. On any discontinuity, it recomputes `T_off` so the next published
  pose equals the last one published. Discontinuities are:
  - a new map starting (the state went through `LOST`: coordinates reset);
  - a jump of more than **0.2 m or 10°** between consecutive frames (a loop closure or merge
    shifted the current pose);
  - the end of a gimbal sweep, if sweeps were used.
- **Why:** in this design ORB-SLAM3 provides high-quality *local motion* (visual odometry) to the
  filter. The *global* corrections come from slam_toolbox's LiDAR loop closure. ORB-SLAM3's own
  loop closures still happen inside its map. They're evaluated separately as configuration C1
  (section 5).

**Building the wrapper against the repo** (a `colcon` package linking `libORB_SLAM3.so`):
- **Include directories**, the same as the repo's `CMakeLists.txt`: the repo root (for
  `Thirdparty/DBoW2/...` and `Thirdparty/g2o/...`), `include/`, `include/CameraModels/`,
  `Thirdparty/Sophus/`, Eigen, Pangolin (`System.h` includes the viewer headers), OpenCV and CUDA.
- **Link:** `<repo>/lib/libORB_SLAM3.so`, `<repo>/Thirdparty/DBoW2/lib/libDBoW2.so`,
  `<repo>/Thirdparty/g2o/lib/libg2o.so`, Pangolin, OpenCV, `boost_serialization`.
- **Compile flags:** match the library, with `-O3 -march=native` and
  `add_definitions(-DCOMPILEDWITHC11)`. Mismatched CPU flags between a program and an Eigen-based
  library can cause hard-to-trace crashes. ROS 2 Humble needs C++17; the library itself is built
  as C++14, and that combination links fine.
- **Runtime:** `LD_LIBRARY_PATH` must include `<repo>/lib` and the two Thirdparty `lib` folders
  (or set an RPATH in the package).

**realsense-ros settings:**
- the two IR streams on, at 640×480 @ 30;
- depth and colour off;
- **projector off** (`emitter_enabled` 0);
- auto-exposure limit 5 ms, matching the Stage 1 driver.

Parameter names differ between realsense-ros versions; check them with `ros2 param list` on the
running node.

### 3.4 `wheel_odom`: encoders → odometry
- Reads the microcontroller's lines (or the motor controller's counts).
- Per update: `dL = 2πr·ΔticksL/TPR`, `dR = 2πr·ΔticksR/TPR`; forward distance `d = (dL+dR)/2`;
  turn `dθ = (dR−dL)/B_eff`. Integrate x, y, θ.
- Publishes `/wheel/odom` (`nav_msgs/Odometry`, frame `odom`, child `base_link`) with the
  **twist** (vx, ωz) and its covariance. It does **not** publish a TF; the EKF does that.

### 3.5 `slam_toolbox` settings (A1-specific)
Start from `online_async` mode, with these parameters:
```yaml
slam_toolbox:
  ros__parameters:
    mode: mapping
    map_frame: map
    odom_frame: odom
    base_frame: base_link
    scan_topic: /scan
    max_laser_range: 12.0          # A1 range
    resolution: 0.05
    minimum_travel_distance: 0.2
    minimum_travel_heading: 0.2    # rad
    do_loop_closing: true
```

### 3.6 `robot_localization` EKF settings
```yaml
ekf_filter_node:
  ros__parameters:
    frequency: 30.0
    two_d_mode: true
    publish_tf: true
    map_frame: map
    odom_frame: odom
    base_link_frame: base_link
    world_frame: odom
    odom0: /wheel/odom                 # wheel twist: vx and yaw rate
    odom0_config: [false, false, false, false, false, false,
                   true,  false, false, false, false, true,
                   false, false, false]
    odom1: /orb/odom                   # ORB-SLAM3 pose, used as relative motion
    odom1_config: [true,  true,  false, false, false, true,
                   false, false, false, false, false, false,
                   false, false, false]
    odom1_differential: true
```
The config vectors are in `robot_localization`'s order: x, y, z, roll, pitch, yaw, vx, vy, vz,
vroll, vpitch, vyaw, ax, ay, az.

---

## 4. Calibration

### 4.1 Fixed transforms (URDF)
- Measure, with a tape and square, the positions of `laser` and `camera_link` relative to
  `base_link`: x, y, z in metres, plus the mounting yaw and pitch.
- For the camera, `camera_link` is at the centre of the D435's front face. The left IR imager
  (`infra1`) sits a few centimetres off-centre; realsense-ros publishes its exact offset.
- **Verification:** drive route A. Align ORB-SLAM3's trajectory (C1) with slam_toolbox's (C3)
  using `evo_traj --align`. A consistent leftover rotation or offset means a measurement error in
  the URDF: fix it and re-check.

### 4.2 LiDAR check
- In Foxglove, look at `/scan` with the rover in a rectangular room. The walls must be straight,
  and at their true distances. There must be no blobs from rover parts.
- If the rover blocks a sector, mask it (`rplidar_ros` angle limits or a laser filter).

### 4.3 Wheel odometry
1. **Wheel radius (scale):** drive straight 5 m, measured with a tape. Set
   `r_new = r_old × (5 / odometry distance)`. Repeat 3 times and average.
2. **Effective track width (turning):** turn in place 5 full turns (1800°). Set
   `B_eff_new = B_eff_old × (odometry angle / 1800°)`. Skid-steer and tracked rovers typically get
   a `B_eff` well above the physical wheel spacing, because of slip.
3. Check on route A: the wheel-only end-point error gives the baseline for C2.

### 4.4 Covariances
- Start with **wheels:** σ(vx) ≈ 0.05 m/s and σ(ωz) ≈ 0.05 rad/s; and **ORB pose:** σ(x,y) ≈
  0.02 m and σ(yaw) ≈ 0.01 rad.
- Tune on route A: the fused path should follow ORB-SLAM3 while it tracks, and fall back on the
  wheels when it's lost.
- Record the final values in `rover_bringup/config/`. They're part of the result.

---

## 5. Experiments

### 5.1 Configurations
All run on the **same recorded bags** (section 5.2):

| ID | Configuration | What it tests |
|---|---|---|
| **C1** | ORB-SLAM3 alone (its own map and loop closure), i.e. the Stage 1 system | The repo by itself |
| **C2** | Wheel odometry alone | Dead-reckoning baseline |
| **C3** | slam_toolbox + wheel odometry | Standard 2D LiDAR SLAM |
| **C4** | slam_toolbox + EKF(wheels + ORB-SLAM3) | **The fusion** |
| **C5** | EKF(wheels + ORB-SLAM3), no LiDAR | Visual + wheel odometry, without a global map |

### 5.2 Recording protocol
- For every run, record a bag: `ros2 bag record <ns>/infra1/image_rect_raw
  <ns>/infra2/image_rect_raw <ns>/infra1/camera_info /scan /wheel/odom /tf_static`. Here `<ns>`
  is the camera prefix from `ros2 topic list`, e.g. `/camera/camera`.
- IR stereo is ~18 MB/s (≈1 GB/min), which is fine on the NVMe.
- Then replay each bag with `ros2 bag play --clock`, once per configuration. Every configuration
  sees identical input.
- Keep the Stage 1 conventions: an X on the floor for the start mark, the same operator and
  lighting, **3 runs per route**.

### 5.3 Routes

| Route | Description | Tests question |
|---|---|---|
| A. Closed loop (Stage 1) | ~30–50 m loop, ends at the start mark | Overall drift |
| B. Out-and-back (Stage 1) | Corridor out and back | Loop closure from the opposite direction |
| C. Blank wall (Stage 1) | Vision loses tracking | Q2: does fusion keep going? |
| **D. Dark** | Part of the route with the lights off | Q2: IR stereo with the projector off fails in darkness; the LiDAR doesn't |
| **E. Long uniform corridor** | ≥ 20 m of plain parallel walls | Q3: 2D LiDAR can't tell position along the corridor |
| **F. Glass / reflective** | Glass doors, mirrors, shiny floors | Q3: laser returns fail or mislead |

### 5.4 Ground truth and metrics
We have no motion-capture system, so we use three checks:
1. **End-point error** on closed routes (A, and D if it's a loop): the distance between start and
   end pose, as a % of path length. Same definition as Stage 1.
2. **Surveyed markers:** tape 5–10 floor markers at positions measured with a tape. Stop on each,
   and press a key that logs a `/marker` event (a small node, or a `ros2 topic pub` alias).
   Compare each configuration's position at those times with the measured positions, using the
   error in metres.
3. **Agreement between configurations:** `evo_ape` / `evo_rpe` between them, with `--align`, to
   show where they diverge (e.g. C1 vs C4 on route D).

Tools:
- `pip install evo` works on the Jetson or the PC. `evo_traj tum ... --plot` for trajectories;
  `evo_ape` for the numbers.
- Export each configuration's path in TUM format: ORB-SLAM3 files from the wrapper; the EKF and
  slam_toolbox paths with a small recorder node subscribing to TF `map → base_link`.

### 5.5 Results table (per route, 3 runs each)

| Config | end-point % | marker error (m) | tracking/scan failures | notes |
|---|---|---|---|---|
| C1 ORB-SLAM3 alone | | | lost_events / new_maps | |
| C2 wheels alone | | | – | |
| C3 LiDAR + wheels | | | scan-match failures | |
| C4 fusion | | | | |
| C5 wheels + ORB | | | | |

**The expected story:**
- C4 ≈ C3 or better on A and B;
- C4 ≫ C1 on C and D (vision failures);
- C4 > C3 on E (the corridor), if the camera sees texture there.

Report whatever the data shows, including where fusion *doesn't* help.

---

## 6. Work breakdown

| Step | Content | Done when |
|---|---|---|
| 3a | ROS 2 Humble, realsense-ros from source, `orbslam3_ros` wrapper | Route A through the wrapper gives the same results as the Stage 1 driver (same loops, similar end-point error) |
| 3b | Encoders, microcontroller, `wheel_odom`, calibration | Wheel-only straight-line error < 2 %, turning error < 5° per turn |
| 3c | RPLidar A1 mount, `rplidar_ros`, slam_toolbox with wheel odometry (C3) | Clean map of a room; loop closes on route A |
| 3d | URDF, EKF (C4, C5), re-anchoring logic | Fused path smooth with no jumps; follows ORB while tracking, holds when it's lost |
| 3e | Recording protocol, routes A–F × 3, replays C1–C5 | All bags recorded, all configurations replayed |
| 3f | Analysis | Results tables, plots, and a written conclusion for questions 1–4 |

Rough time: **3–5 weeks**. Most of it goes on 3b (hardware-dependent) and 3e (the number of runs).

## 7. Risks

| Risk | Mitigation |
|---|---|
| The rover has no accessible encoders | Add encoder discs and hall sensors, or motors with encoders. Without them, C2, C3 and C5 are weakened, and slam_toolbox must rely on scan matching alone |
| Parts of the rover in the scan plane | Mount position (2.1); mask the angles |
| realsense-ros doesn't match our librealsense | Build realsense-ros at the tag matching the installed librealsense version |
| The wrapper crashes on Eigen alignment | Same compile flags as the library (3.3) |
| Gimbal yaw moves the camera relative to the rover | Yaw locked for fusion runs (2.3); ORB input paused during any sweep |
| Jumps in `/orb/odom` corrupt the EKF | Only publish when OK; re-anchor on new maps, jumps, sweeps (3.3) |
| Replay timing differs from live | Use `--clock` and `use_sim_time: true` on all nodes during replays |
| Fusion doesn't help on some routes | That is a valid result. Report it |
