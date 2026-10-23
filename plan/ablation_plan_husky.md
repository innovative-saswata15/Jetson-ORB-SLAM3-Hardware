# Ablation study: every sensor in the Husky bag

**Goal:** use everything in the other team's bag (`husky_20261023_062755_0`, 355 s, 2026-10-23) to
measure **what each sensor and each feature of our Jetson ORB-SLAM3 build contributes**: accuracy,
robustness and compute cost.

This plan extends [evaluation_plan_stage_1.md](evaluation_plan_stage_1.md). Stage 1 evaluated stereo
only. This plan:
- adds the IMU, depth and colour streams;
- adds the repository's run-time features;
- uses non-visual baselines from the same bag.

Conventions are the same as the other runbooks: **PC** is the Fedora machine with the venv
`~/venvs/ext` (Python 3.12, `rosbags`, `kiss-icp`). **Jetson** is the Orin Nano.

---

## 0. What the bag contains (from `metadata.yaml`)

| Stream | Topic | Rate | Use in this study |
|---|---|---|---|
| IR left / right (rectified, uncompressed) | `/camera/infra{1,2}/image_rect_raw` + `camera_info` | 15 Hz | mono and stereo visual SLAM |
| Depth (in the IR-left frame) | `/camera/depth/image_rect_raw` + `camera_info` | 15 Hz | RGB-D mode with IR + depth |
| Colour (rolling shutter) | `/camera/color/image_raw` + `camera_info` | 15 Hz | RGB-D mode with colour + aligned depth |
| Depth aligned to colour | `/camera/aligned_depth_to_color/image_raw` | 15 Hz | (same) |
| RealSense extrinsics | `/camera/extrinsics/depth_to_{infra1,infra2,color}` | 1 msg | frame checks |
| Husky IMU | `/imu/data` (+ `/imu/mag`, `/imu/rpy`) | ~96 Hz | mono-inertial and stereo-inertial modes |
| 3D lidar | `/lidar_points` | 10 Hz | **reference trajectory** and lidar baselines |
| Wheel odometry | `/platform/odom` | ~28 Hz | baseline |
| Wheel + IMU filter | `/platform/odom/filtered` | ~48 Hz | baseline |
| TF | `/tf_static` (2), `/tf` (22 055) | n/a | camera ↔ IMU ↔ lidar ↔ base transforms |

The camera has no IMU of its own (no `/camera/accel` or `/camera/gyro`), so the inertial modes use
the **Husky's IMU**. That has three consequences, all handled in §3: the IMU sits away from the camera,
it may run on a different driver clock, and its noise parameters are unknown.

---

## 1. Rules of the study

1. **The reference is independent of what is measured.** It comes from the lidar (with loop
   closure, §2). No ORB-SLAM3 configuration uses the lidar, so no arm is scored against itself. The
   lidar baselines (KISS-ICP) are compared with the reference only to judge *the reference's* quality,
   and the comparison is labelled as correlated.
2. **Change one factor at a time around a fixed baseline.** Group A changes the sensors; groups B–D
   each change one setting on a fixed sensor set. A full factorial design would be about 400 runs,
   which is too many.
3. **Repeat every configuration 3 times** (5 for the baseline). ORB-SLAM3 is multithreaded and not
   deterministic, so a difference counts only if it is **larger than the run-to-run spread** of both
   configurations. The equivalence rule is the same as in the paper.
4. **Keep the platform state the same for every Jetson run:** MAXN_SUPER plus `jetson_clocks`, the
   same commit, nothing else running, and interleaved run order (repeats outermost, as `run_matrix`
   already does).
5. **Compute cost is measured only on the Jetson.** Lidar and wheel baselines run on the PC and get
   accuracy numbers only.

---

## 2. The reference trajectory (PC)

| Tier | Method | Role |
|---|---|---|
| **R*** | **KISS-SLAM** (`pip install kiss-slam`: KISS-ICP odometry + scan-context loop closure + pose-graph optimisation) on `/lidar_points` | **The reference** for all accuracy numbers |
| R1 | KISS-ICP (odometry only, `processing/lidar_odometry.py`) | Checks R*; also a lidar-only baseline |
| R2 | `/platform/odom/filtered` (wheels + IMU) | Checks R*; also a baseline |

### 2.1 Quality checks on R* (report all of them)
- **Agreement with R1:** the SE(3)-aligned RMS difference. A large difference that is concentrated
  around the loop closures is expected; elsewhere it should be small.
- **Map sharpness:** accumulate the deskewed scans along R*. Walls should be crisp, with no doubled
  surfaces at revisited places. Check the rendered top-view PNG by eye.
- **Loop residual:** the position jump each accepted loop closure applies. It bounds the drift R1
  accumulated.
- **Ground robot sanity:** z should stay within a few cm of flat, and roll and pitch should be small.
- **Error floor:** we can't resolve ATE differences smaller than ~2–5 cm, the typical lidar-SLAM
  accuracy indoors. Treat differences below this as "equal".

### 2.2 Conversion to each camera frame
For every visual configuration: `T_world_cam(t) = T_world_lidar(t) · T_lidar_cam`. The transform is
chained from `/tf_static`, and the poses are interpolated at that configuration's image timestamps
(`processing/make_gt.py`, which already does this). The camera frame is the infra1 optical frame
for IR, IR + depth and inertial runs, and the colour optical frame for the colour RGB-D run.

**Do the inertial modes need a different reference?** ORB-SLAM3 inertial modes output the **IMU
body** pose, not the camera pose (as on EuRoC). So for the inertial modes the reference is
`T_world_lidar · T_lidar_imu`. The evaluation picks the right `gt_*.csv` per mode.

---

## 3. Calibration items that the extra sensors need

| Item | How | Needed by |
|---|---|---|
| **`T_imu_cam` (ORB-SLAM3 `IMU.T_b_c1`)** | chain `/tf_static` from the `/imu/data` frame to `camera_infra1_optical_frame`. If the chain uses a dynamic `/tf` link, stop and check. | inertial modes |
| **IMU units and gravity** | at rest, ‖acc‖ should be ≈ 9.81 m/s² and gyro ≈ 0. If the driver removes gravity (‖acc‖ ≈ 0), the inertial modes can't be run: **drop them and record why**. | inertial modes |
| **IMU noise** (`IMU.NoiseGyro`, `NoiseAcc`, `GyroWalk`, `AccWalk`) | from a stationary segment at the start of the bag: the white-noise densities come from the standard deviation × √(Δt). The random walks use the datasheet values for the IMU model, if it can be identified from the frame name or driver; otherwise the EuRoC values. **Inflate ×5–10**, as is usual for uncalibrated IMUs. | inertial modes |
| `IMU.Frequency` | measured rate (~96 Hz) | inertial modes |
| **Camera-to-IMU time offset** | compare the gyro rate with the visual yaw rate from a stereo (B0) run, and find the shift that best correlates them. ORB-SLAM3 does not estimate it. If \|offset\| > 5 ms, shift the IMU timestamps in the conversion. | inertial modes |
| Depth frame = IR-left frame | `depth_to_infra1` should be identity, and the depth and infra1 resolutions should be equal | IR + depth RGB-D |
| Depth units | 16UC1 in mm, so `RGBD.DepthMapFactor: 1000.0` | both RGB-D modes |
| Colour intrinsics | `/camera/color/camera_info` (check the distortion; the D435 colour stream is usually Brown-Conrady with small coefficients, so copy them into the YAML) | colour RGB-D |

---

## 4. Ablation matrix (Jetson)

**Baseline B0:** stereo IR · GPU ORB · DBoW2 loop closure · 15 FPS · 1250 features · our D435 ORB
settings.

### Group A: sensors (GPU ORB, DBoW2, otherwise B0)
| ID | Configuration | ORB-SLAM3 program | Question |
|---|---|---|---|
| A1 | Mono IR | `Monocular/mono_euroc` | What does the second camera buy? (Scale is unobservable, so score with Sim(3).) |
| **A2 = B0** | Stereo IR | `Stereo/stereo_euroc` | the baseline |
| A3 | Mono IR + IMU | `Monocular-Inertial/mono_inertial_euroc` | Does the IMU recover scale and robustness for mono? |
| A4 | Stereo IR + IMU | `Stereo-Inertial/stereo_inertial_euroc` | **The paper's main configuration.** Does the IMU help stereo? |
| A5 | IR-left + depth (RGB-D) | `RGB-D/rgbd_tum` | Is the camera's depth a substitute for stereo matching, in accuracy and in speed? |
| A6 | Colour + aligned depth (RGB-D) | `RGB-D/rgbd_tum` | Does colour with a rolling shutter hurt? Comparing it with A5 isolates the image source. |

### Group B: compute front-end
| ID | Applied to | Setting | Question |
|---|---|---|---|
| B1 | A2 and A4 | `CPU_ORB=1` | GPU ≡ CPU accuracy with real ground truth; the speed difference at their resolution |
| B2 | A2 and A4 | `PIPELINE_FE=1` | Pipelining speed-up, and its accuracy cost (pipe was noisier on our data) |
| B3 | A4 | `ADAPTIVE_FE=1` (the repository's IMU-driven economy extractor) | Speed versus accuracy of adaptive feature extraction |

### Group C: loop closure
| ID | Applied to | Setting | Question |
|---|---|---|---|
| C1 | A2 and A4 | no loop closing (`loopClosing: 0`) | How much drift does loop closure remove on this route? |
| C2 | A2 and A4 | DBoW2 + CosPlace CNN (TensorRT engine) | Does the CNN find extra or better loops here? (The paper's Table 10 on new data.) |
| C3 | A4 | `ADAPTIVE_LC=1` (the repository's relaxed tilt guard for inertial loops) | Does the adaptive guard accept useful loops, or harmful ones? |

### Group D: input stress (on A2 and A4)
| ID | Setting | Question |
|---|---|---|
| D1 | 7.5 FPS (every second frame of `times.txt`) | Robustness at a lower frame rate; how much the IMU helps when frames are sparse |
| D2 | 800 features instead of 1250 | Speed versus accuracy of a smaller feature budget |

### Group E: non-visual baselines (PC, accuracy only)
| ID | Source | Note |
|---|---|---|
| E1 | `/platform/odom` (wheels) | pure dead reckoning |
| E2 | `/platform/odom/filtered` (wheels + IMU) | what the robot uses today |
| E3 | KISS-ICP (lidar odometry, no loop closure) | correlated with the reference (§1.1), so label it |

### Run count and time
| Group | Cells | Runs |
|---|---|---|
| A | 6 (B0 × 5 runs, others × 3) | 20 |
| B | 5 | 15 |
| C | 5 | 15 |
| D | 4 | 12 |
| **Total (Jetson)** | **20** | **62** |

A run replays 355 s at up to 15 FPS, plus load time, so about **6.5–8 min**. The whole matrix is
**~7–8 h unattended**, which fits in one night. If a mode is slower than 15 FPS, its runs take longer.
Measure that in the smoke test (§6.2).

---

## 5. Data preparation (PC)

One folder per sequence, a superset of the stage-1 layout:
```
~/datasets/ext/husky_20261023/
├── mav0/cam0/data/<t>.png        IR left            (stereo, mono, inertial, IR-D)
├── mav0/cam1/data/<t>.png        IR right
├── mav0/imu0/data.csv            '#timestamp [ns],w_x,w_y,w_z,a_x,a_y,a_z' (EuRoC), IMU frame
├── depth/<t>.png                 16-bit depth in the IR-left frame
├── color/<t>.png, depth_c/<t>.png    colour + depth aligned to colour
├── times.txt                     IR stereo times      (times_7p5.txt for D1)
├── assoc_ird.txt, assoc_rgbd.txt     rgbd_tum association files ('t rgb t depth')
├── settings_{mono,stereo,mono_inertial,stereo_inertial,ird,rgbd}.yaml  (+ _800feat for D2)
├── gt_cam.csv, gt_imu.csv, gt_color.csv    reference in each output frame (§2.2)
├── calib.json, tf_static.json, imu_calib.json, source_info.txt
```

| Step | Tool (`processing/`) | Status |
|---|---|---|
| IR stereo, `times.txt`, stereo settings | `bag_to_euroc.py` | written |
| Read-only inventory | `inspect_bag.py` | written; to be extended with IMU, depth and colour checks |
| IMU → `imu0/data.csv`, rest statistics, noise estimate | `bag_imu.py` | **to write** |
| Depth and colour → PNG + association files | `bag_rgbd.py` | **to write** |
| All 6 settings files (+ D2 variants), from one template per mode | `make_settings.py` | **to write** (`bag_to_euroc.py` already covers stereo) |
| Reference trajectories: R* (KISS-SLAM), R1 (KISS-ICP) | `lidar_odometry.py` (R1) + `lidar_slam.py` (R*) | R1 written; **R* to write** |
| Reference in each frame | `make_gt.py` | written; add `--frame imu/color` |
| Camera-to-IMU time offset | `imu_time_offset.py` | **to write** (needs a B0 run first) |
| Baselines E1, E2 → TUM files | `bag_odom.py` | **to write** |

---

## 6. Jetson side

### 6.1 Pipeline support (`repro/common.py`, `repro/run_matrix.py`)
- **New modes, each with its program, settings file, output and reference:**

  | Mode | Program | Settings | Output | Reference |
  |---|---|---|---|---|
  | `ext_m` | `mono_euroc` | `settings_mono.yaml` | camera | `gt_cam.csv` |
  | `ext_s` | `stereo_euroc` | `settings_stereo.yaml` | camera | `gt_cam.csv` |
  | `ext_mi` | `mono_inertial_euroc` | `settings_mono_inertial.yaml` | body | `gt_imu.csv` |
  | `ext_si` | `stereo_inertial_euroc` | `settings_stereo_inertial.yaml` | body | `gt_imu.csv` |
  | `ext_ird` | `rgbd_tum` | `settings_ird.yaml`, `assoc_ird.txt` | camera | `gt_cam.csv` |
  | `ext_rgbd` | `rgbd_tum` | `settings_rgbd.yaml`, `assoc_rgbd.txt` | camera | `gt_color.csv` |

  `rgbd_tum` writes `CameraTrajectory.txt` in TUM format with seconds, so the evaluator reads it.
- **New arms (env switches only, no rebuild):** `adaptive_fe` (`ADAPTIVE_FE=1`), `adaptive_lc`
  (`ADAPTIVE_LC=1`), `fps7p5` (`times_7p5.txt`), `feat800` (the `_800feat` settings). `nolc` and
  `cnn` are extended to the `ext_*` modes; `settings_for` gets the per-sequence settings file.
- **Suites:** `abl-A`, `abl-B`, `abl-C`, `abl-D`. Each is a list of (mode, arm, runs) cells from §4,
  so the matrix can be run and resumed group by group.
- **Evaluation:** `score_euroc_like` already gives the SE(3) and Sim(3) ATE, scale and t_rel. Add
  for each run:
  - **initialisation time** (first pose − first image; the inertial modes need IMU initialisation);
  - **coverage** (share of images with a pose);
  - **per-segment error** (1, 5 and 10 m).

### 6.2 Smoke tests (1 run per mode, before the night run)
For each of the 6 modes, check:
- a trajectory is written;
- coverage is above 80%;
- scale is 0.97–1.03 (all but mono; for mono, report it only);
- the ATE is plausible (centimetres to decimetres);
- the FPS is at least 15. If not, the run takes longer, so note it.

**Inertial modes only:** the console shows `IMU initialized` within ~20 s, and there are no
`not enough acceleration` loops. A Husky driving straight at constant speed may initialise slowly.
That is a result in its own right: report the initialisation time.

### 6.3 The run
```bash
sudo nvpmodel -m 2 && sudo jetson_clocks && sudo nvpmodel -q | tee ~/abl_power_mode.txt
cd ~/Jetson-ORB-SLAM3-Hardware
for g in A B C D; do
  python3 repro/run_matrix.py abl-$g --engine cosplace_r50_512.fp16.trt --results ~/repro_results_abl
done 2>&1 | tee ~/abl_matrix.log
```
(Only C2 links the engine. Every other cell runs DBoW2 only, which the suite definitions enforce.)

---

## 7. Metrics and how they are reported

| Category | Metrics | Per |
|---|---|---|
| **Accuracy** | ATE SE(3) (Sim(3) and scale for mono); RPE translation and rotation over 1, 5 and 10 m; final drift before loop closure (C1) | cell: median, min–max over runs |
| **Robustness** | coverage %, initialisation time, maps created, tracking-lost events, aborts (NaN and others) | cell |
| **Loop closure** | loops accepted, merges, `[GuardEval]` accept/reject counts (C3) | cell |
| **Compute** | FPS and ms per frame, VDD_IN mean and peak, energy per frame, CPU %, GPU %, RAM peak, Tj | cell (Jetson only) |
| **Efficiency** | ATE versus energy per frame, and ATE versus FPS (Pareto plot over all cells) | all cells |

**Main deliverables:**
1. The **ablation table**: rows are the cells, columns the categories above, and a "Δ vs B0" column
   marked significant only if it exceeds both run-to-run spreads.
2. A **sensor contribution chart**: the ATE of A1–A6 and E1–E3 on one axis, with the ~2–5 cm error
   floor of the reference drawn on it.
3. A **Pareto plot**: accuracy against energy per frame, to answer "what is the best configuration
   for the robot".
4. **Trajectory overlays:** the reference against B0, A4, the best and the worst configuration, plus
   error coloured along the path.

The report goes in `jetson-orbslam3-results-ablation/ABLATION_REPORT.md`, with the same structure as
the campaign reports. It also covers the reference's quality (§2.1), the calibration values used (§3),
the platform state and the limitations.

---

## 8. Order of work

| # | Step | Where | Blocks |
|---|---|---|---|
| 1 | Download finishes → `inspect_bag.py` (extended: IMU at rest, depth and colour frames, TF chains) | PC | everything |
| 2 | Decide feasibility: is gravity in the IMU data? Is depth in the IR frame? Is there a TF chain from the IMU and lidar to the camera? Drop any infeasible cells and record why | PC | §4 |
| 3 | Reference R* and R1, quality checks (§2.1) | PC | all scoring |
| 4 | Converters and settings (§5), and baselines E1–E3 | PC | Jetson runs |
| 5 | Pipeline support (§6.1), dry run | PC | Jetson runs |
| 6 | Copy to the Jetson, smoke tests for the 6 modes (§6.2) | Jetson | night run |
| 7 | Camera-to-IMU time offset from the B0 smoke run; regenerate the IMU file if needed; re-smoke A3 and A4 | PC + Jetson | inertial cells |
| 8 | The night run (§6.3) | Jetson | report |
| 9 | Evaluation, plots, report (§7) | PC | n/a |

## 9. Risks and fallbacks

| Risk | Effect | Fallback |
|---|---|---|
| The IMU driver removes gravity or reports in an unknown frame | A3, A4, B3, C3 and part of D are infeasible | Run the visual-only cells and report the reason. Still use `/imu/data` in E2. |
| No static TF path from the IMU or lidar to the camera | No lever arm, so the reference is wrong in turns | Hand-eye estimate from the R* and B0 trajectories, plus a tape check on photos of the robot |
| KISS-SLAM finds no loops, or bad ones | A weaker reference | Use R1 (no loop closure) as the reference and widen the error floor, or tune the KISS-SLAM loop thresholds |
| The projector was on | All IR cells are worse, except possibly IR + depth | Keep it as the condition of the dataset, and state it on every table |
| Inertial modes don't initialise (straight, constant-speed driving) | No inertial results | Report the initialisation failure as a result; try `IMU.InsertKFsWhenLost`, or start the replay at a turn |
| A mode runs below 15 FPS | Longer runs | Time is budgeted per measured FPS; the matrix is resumable |
| The 355 s sequence is the only one we have | No variety across scenes | State it as a limitation; split it into 2–3 segments for segment-level error statistics only |
