# Plan: Jetson-ORB-SLAM3 on our hardware – reproduction, then IMU + LiDAR fusion

This is the overview. Each phase has its own detailed document:

| Phase | Document | Status |
|---|---|---|
| **Stage 1. The repo on our hardware**: D435 + Jetson, handheld then on the rover | [stage1.md](stage1.md), step by step in [stage1_runbook.md](stage1_runbook.md) | Milestone 1 done; Milestone 2 run A1 passed |
| **Paper reproduction**: recreate the paper's results on our Jetson, then on our D435 | [paper_reproduction_runbook.md](paper_reproduction_runbook.md) (code in `repro/`) | **Current** |
| **Stage 2. IMU + LiDAR fusion**: D435 + IMU + LiDAR, evaluated against a 3D-LiDAR-mapped environment | [stage2_runbook.md](stage2_runbook.md) | After the reproduction |

**What has actually been done**, with real outputs and everything we learned along the way:
[progress_log.md](progress_log.md).

**Change of direction (2026-10-03):**
- The STorM32 **gimbal has been dropped** (it couldn't be weight-balanced), along with the earlier
  gimbal and encoder plans.
- New sequence: **recreate the paper's results on our hardware first**, then build the final
  system from the **D435, an IMU and a LiDAR**, with ablations against a standard 3D-LiDAR map of
  the test environment.

---

## Goals

1. **Deploy the repository, as it is, on our hardware.** Done in Stage 1: this repository's
   GPU-accelerated ORB-SLAM3 running live on a Jetson Orin Nano with an Intel RealSense D435, on a
   manually driven rover.
2. **Reproduce the paper** (arXiv:2608.17874) on our Jetson:
   - the EuRoC, TUM-VI and KITTI accuracy, GPU against CPU;
   - throughput, per-stage timing and power;
   - the loop-closure ablation and the feature-level equivalence;

   then the same equivalence and speed checks on **our own D435 recordings**.
3. **The final system:** stereo-inertial ORB-SLAM3 (D435 + IMU), loosely fused with LiDAR SLAM.
   Evaluated with ablations against a reference map (and ground-truth trajectories) of the test
   environment, made with a 3D LiDAR.

## Ground rules

1. **The ORB-SLAM3 library is never modified**: nothing under `src/`, `include/`, `Thirdparty/`,
   or the repository's `Examples/` programs. Our work consists of:
   - config files;
   - our own programs in **separate CMake projects** (`repro/cpp`, later `fusion/`);
   - Python tools (`repro/`, `tools/`);
   - later, ROS 2 packages.

   The Stage 1 rover driver and its `CMakeLists.txt` target are the one earlier exception: an
   added file, with the original driver untouched.
2. **Interact with ORB-SLAM3 only through its public API and run-time switches:**
   `TrackStereo`, `GetTrackingState`, the `Save*` functions, `CPU_ORB`, `PIPELINE_FE`, and the
   settings keys.
3. **Every comparison runs on identical input:** public datasets, or our own recordings replayed
   under each configuration. Results come from one evaluator, `repro/evaluate.py`.
4. **Measure, don't tune to match.** Differences from the paper get explained, not adjusted away.

## Decisions log

| Topic | Decision | Why |
|---|---|---|
| Core demo | D435 + Jetson, handheld first, then on the rover | Shows the repo working on our hardware |
| Stereo, no IMU (Stage 1) | Stereo visual SLAM on the D435's IR pair | The D435 has no IMU; stereo gives metric scale |
| Gimbal | **Dropped** (2026-10-03) | Weight-balancing not feasible |
| Wheel encoders | Not in the final system | The final sensors are the D435, an IMU and a LiDAR |
| Next phase | Reproduce the paper's results on our Jetson, then on our D435 | Validates the hardware stack against published numbers before building on it |
| Reproduction CPU arm | `CPU_ORB=1` (the repo's run-time switch) | The paper uses it for KITTI/TUM-VI; it swaps the front end only. An upstream ORB-SLAM3 build is optional |
| Desktop rows of Table 2 | Not reproduced | Building on x86 would need changes to the core `CMakeLists.txt` |
| Per-stage timing build | A separate git worktree with `-DREGISTER_TIMES` | The define changes class layouts; no source edits |
| IMU (Stage 2) | Recommend a D435i swap; an external IMU only if unavoidable | Hardware-synchronised timestamps, a factory calibration, and the repo's own driver |
| IMU fusion | Tight, inside ORB-SLAM3 (stereo-inertial mode, public API) | Already implemented by the library; the paper's main mode |
| LiDAR fusion | Loose: ORB-SLAM3 VIO → `robot_localization` EKF → `slam_toolbox` (2D, RPLidar A1) | ORB-SLAM3 has no LiDAR input, and adding one would change the library |
| Ground truth for the final evaluation | A 3D-LiDAR reference map, plus a 3D LiDAR on the rover localised in it (fallback: surveyed markers) | "Standard 3D LiDAR mapped environment" |
| Middleware | ROS 2 Humble | Matches JetPack 6 / Ubuntu 22.04; standard drivers and `rosbag2` |
| JetPack install | 6.2.3 flashed to NVMe with SDK Manager; components via `apt install nvidia-jetpack` | SDK Manager's component step failed over USB |
| Camera firmware | Keep 5.17.3.10 | Newer than librealsense 2.55.1's recommendation; works |
| Display | No monitor; a virtual screen over VNC (display `:99`, port 5910) | The viewer and Stop button need a display |
| Git workflow | Commit on the PC, `git pull --ff-only` on the Jetson | Pulling needs no login |
| `apt upgrade` | Not run during measurement campaigns | Results are tied to the JetPack version |

---

## The foundation: what ORB-SLAM3 gives us

From camera images alone (and IMU samples, in inertial modes), live, it produces:
- the camera's (or IMU body's) 6-DoF pose at every frame;
- a sparse 3D map of points and keyframes.

Loop closure corrects drift when a place is recognised. This repository's contribution, which
the paper claims and we now reproduce, is running ORB feature extraction on the Jetson's GPU
**without changing the results** (GPU ≡ CPU), plus optional CNN place recognition through
TensorRT.

**Messages we count** (details in [stage1.md, section 1](stage1.md#1-what-we-use-from-the-repo)):

| Message | Meaning |
|---|---|
| `*Loop detected` | Loop closure |
| `*Merge detected` | Two maps joined |
| `Relocalized!!` | Tracking recovered |
| `Stored map with ID` | Tracking gave up and a new map started |
| `median/mean tracking time` | Printed by the dataset programs |

---

## Phase summaries

### Stage 1: the repo on our hardware ([details](stage1.md))
- **Milestone 1** ✅: EuRoC reproduced with the prebuilt binary (3.72 cm) and from source
  (4.06 cm); the D435 calibration file; a live handheld loop closed.
- **Milestone 2:** route A on the rover. Run A1 passed (loop closed, visual only). A2 and A3
  remain, for repeatability.

### Paper reproduction ([runbook](paper_reproduction_runbook.md))
- **Datasets:** `repro/fetch_datasets.py` for EuRoC (11 sequences) and TUM-VI (rooms 1–6); KITTI
  odometry downloaded by hand.
- **Experiment matrix:** `repro/run_matrix.py` runs every paper table:
  - EuRoC accuracy, GPU/CPU × 5 runs;
  - throughput, Base/Pipe;
  - loop-closure ablation;
  - TUM-VI and KITTI;
  - instrumented timing;
  - power.
- **Tools:** `repro/cpp/feature_equivalence` for the feature-level equivalence;
  `repro/cpp/d435_record_euroc` and the `d435` suite for our own hardware.
- **Report:** `repro/evaluate.py` then `repro/make_report.py` give one table per paper result,
  paper value next to ours.
- About 24 h of compute, unattended.

### Stage 2: IMU + LiDAR fusion ([runbook](stage2_runbook.md))
- **Decisions:** IMU choice, LiDAR, the 3D reference LiDAR, the ground-truth method.
- **Calibration:** Allan variance, Kalibr camera-IMU, camera-LiDAR-base URDF.
- **Visual-inertial ORB-SLAM3** on our sensors.
- **ROS 2 stack:** our ORB-SLAM3 wrapper, an EKF, `slam_toolbox`, and a dense D435 depth map.
- **Reference:** a 3D-LiDAR map of the environment and a ground-truth trajectory.
- **Ablations C0–C4:** stereo, stereo-inertial, LiDAR only, stereo + LiDAR, full system. Plus the
  GPU/CPU and pipelining variants, on routes R-A to R-E. Metrics: ATE/RPE, map
  accuracy/completeness, robustness, runtime and power.

---

## Current status (2026-10-03)

| Item | Status |
|---|---|
| JetPack 6.2.3 (L4T 36.5.2) on NVMe, `nvidia-jetpack` installed | ✅ CUDA 12.6.68, TensorRT 10.3.0.30, OpenCV 4.8.0 |
| Repo on the Jetson, git workflow | ✅ `~/Jetson-ORB-SLAM3-Hardware` |
| EuRoC MH01 with the prebuilt binary / from source | ✅ ATE 3.72 cm / 4.06 cm (paper: 4.09 cm GPU, 3.60 cm CPU, median of 5) |
| D435 on USB 3, calibration file | ✅ serial 827312071682; fx = fy = 385.4061, cx = 318.2860, cy = 238.9506, b = 0.0499 m |
| Live tracking speed | ≈15 fps effective (every other 30 fps frame skipped) |
| Virtual screen over VNC | ✅ display `:99`, port 5910 |
| Milestone 1 (handheld loop) | ✅ passed on attempt 2 |
| Milestone 2 (rover, route A) | ✅ A1 passed; ⏳ A2, A3 |
| Reproduction pipeline (`repro/`) | ✅ written and tested on the PC (evaluator math against known answers, end-to-end with stand-in programs, C++ syntax-checked). ⏳ Not yet run on the Jetson |
| Stage 2 | Planned ([stage2_runbook.md](stage2_runbook.md)); decisions D1–D5 pending |

**Next action:** the paper reproduction runbook, R1–R3, then the R4 smoke test.

## Conventions

- **Run folders:**
  - reproduction: `~/repro_results/<suite>/<mode>/<seq>/<arm>/run<k>/`;
  - rover driver runs: `~/runs/<date>_<label>/`.
- **3 runs per route and condition** for live tests; **5 runs** for the EuRoC accuracy tables, as
  in the paper.
- **Aggregation, as in the paper:** per-sequence median over runs, then the mean over sequences.
- **Driving / walking:** ≤ 0.5 m/s, slow turns. At the end, pass over the start mark and continue
  2–3 m before stopping, so loop closure can confirm. Good lighting.

## Main risks

| Risk | Phase | Mitigation |
|---|---|---|
| ETH download rate limit (HTTP 429) | Reproduction | Wait and retry once; PC download as a fallback |
| KITTI needs registration | Reproduction | Manual download; `fetch_datasets.py kitti-check` |
| Instrumented (`REGISTER_TIMES`) build doesn't compile | Reproduction | Report Table 6 from uninstrumented totals; note it |
| CosPlace model not in the repo | Reproduction | Export on the PC (`repro/tools/export_cosplace_onnx.py`), build the engine on the Jetson |
| JetPack 6.2.3 vs the paper's 6.2 | Reproduction | Record versions; explain differences, don't tune |
| Viewer drawn in software under xvfb (KITTI, TUM-VI, D435 programs) | Reproduction | Same on both arms; noted in the report |
| External IMU timing / calibration | Stage 2 | D435i swap, or hardware sync; Kalibr ×3 |
| No 3D LiDAR on the rover for ground truth | Stage 2 | Surveyed markers / AprilTags fallback |
| Poor lighting | All | Lit routes; dark routes deliberately test the LiDAR fusion |
