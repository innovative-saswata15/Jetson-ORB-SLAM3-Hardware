# Plan: Jetson-ORB-SLAM3 on a rover, with a gimbal add-on

This is the overview. Each stage has its own detailed document:

| Stage | Document | Status |
|---|---|---|
| **1. The repo on our hardware**: D435 + Jetson, handheld then on the rover | [stage1.md](stage1.md), and the step-by-step [stage1_runbook.md](stage1_runbook.md) | **Current** |
| **2. Gimbal add-on**: stability, coverage sweeps, recovery sweeps | [stage2.md](stage2.md) | After Stage 1 |
| **3. Wheel encoders + RPLidar A1**, fusion analysis | [stage3.md](stage3.md) | Future |

---

## What this project demonstrates

1. **The repo, as it is, running on our hardware.** This repository's GPU-accelerated ORB-SLAM3
   runs live on a Jetson Orin Nano, with an Intel RealSense D435, on a manually driven rover.
2. **Our own add-on, built on top without touching the library.** A gimbal controller uses the
   STorM32 gimbal three ways to make that SLAM more robust:
   1. **stability**;
   2. **coverage sweeps** at stops, so places are recognised later from any direction and loops
      close;
   3. **recovery sweeps** when tracking fails, so ORB-SLAM3 relocalises instead of starting a new
      map.

   Each feature is measured against the plain Stage 1 setup.
3. **Future:** add wheel encoders and the RPLidar A1 together, and analyse what loosely fusing
   them with ORB-SLAM3 gains.

## Ground rules

1. **The ORB-SLAM3 library is never modified**: nothing under `src/`, `include/` or
   `Thirdparty/`. Our work consists of:
   - config files;
   - **new** driver programs in `Examples/Stereo/`; the originals stay untouched;
   - `addons/` and `tools/` folders;
   - later, separate ROS 2 packages;
   - new build targets in `CMakeLists.txt`.
2. **ORB-SLAM3 doesn't know the gimbal exists.** It sees images, copes with the camera moving, and
   reports its state through the public API. **Every decision to move the gimbal is ours.**
3. **Mode: stereo visual SLAM** on the D435's two infrared cameras, with the projector off. **No
   IMU is needed.** The stereo pair gives metric depth.
4. **The rover is driven by hand.** Nothing makes it autonomous.

## Decisions log

| Topic | Decision | Why |
|---|---|---|
| Core demo | D435 + Jetson, handheld first, then on the rover | Shows the repo working on our hardware |
| IMU / visual-inertial mode | Not used | Stereo gives metric scale; the D435 has no IMU |
| Gimbal | Stage 2 add-on: stability, coverage sweeps, recovery sweeps | Our own contribution on top of the repo |
| Feeding gimbal motion into ORB-SLAM3's tracker | Rejected | It would modify the library |
| Wheel encoders + RPLidar A1 | Stage 3 (future), added together, loose fusion | 2D LiDAR SLAM needs odometry; best analysed as one experiment |
| Rover autonomy | Out of scope | The rover is driven by hand |
| Plan layout | One overview plus one detailed document per stage | Each detail lives in exactly one place |

---

## The foundation: what ORB-SLAM3 gives us

From camera images alone, live, it produces:
- **the camera's position and orientation** at every frame;
- a **sparse 3D map**: thousands of 3D points plus **keyframes**, stored snapshots used to
  recognise places.

On recognising a place it has seen before, it **closes the loop** and corrects the drift. This
repo's contribution is running the feature-extraction step on the Jetson's GPU without changing
the results.

**Where we see it:**
- the **Map Viewer**, a 3D window: points, blue keyframes, green current camera. It shows only the
  active map;
- the **Current Frame** window: the image, with tracked features;
- trajectory files, saved by our rover driver;
- four console messages we count as measurements:
  - `*Loop detected`: loop closure;
  - `*Merge detected`: two maps joined;
  - `Relocalized!!`: tracking recovered in the same map;
  - `Stored map with ID`: tracking gave up and a new map started.

  The details are in [stage1.md, section 1](stage1.md#1-what-we-use-from-the-repo).

**How the pieces fit:**
```
   INPUT SIDE (Stage 2 add-on)          THE REPO (unchanged)                OUTPUT
   G1 stabilisation ───────┐
   G3 coverage sweeps ─────┼──► D435 stereo ──► ORB-SLAM3 on Jetson ──┬──► 3D viewer
   G4 recovery sweeps ─────┘      images         (GPU ORB)            ├──► trajectory files, events log
          ▲                                          │                └──► (Stage 3) ROS 2 pose → fusion
          └──── tracking state, tracked points, pose ┘
```

---

## Stage summaries

### Stage 1: the repo on our hardware ([details](stage1.md))
- **Milestone 1, handheld (1–2 days):**
  1. reproduce the repo's EuRoC result (ATE ≈ 2 cm);
  2. build from source with librealsense and Pangolin;
  3. write our D435's calibration file;
  4. walk a loop with the original live driver, and check that it closes.
- **Milestone 2, rover (3–5 days):**
  - rigid mount and power;
  - a new **rover driver**: a copy of the original that exits cleanly, saves trajectories and
    logs tracking events;
  - `tools/summarize_run.py`;
  - **baseline numbers** on three routes, 3 runs each:
    - A: closed loop;
    - B: out-and-back;
    - C: blank wall.
- **Done when:** route A closes reliably on the rover, and all baseline numbers are recorded.

### Stage 2: the gimbal add-on ([details](stage2.md))
- **G1 stability:** assemble, balance and tune the STorM32, with yaw in follow mode. Re-run routes
  A–C.
- **G2 control link:** the confirmed STorM32 serial protocol. Discovery tests T1–T8 decide the
  exact command sequence *before* any sweep code is written; this is the main risk.
- **G3 coverage sweeps** (key `s`, only when still): ±170° at 30°/s, ≈23 s. Measured on route B:
  the return trip should now close its loop.
- **G4 recovery sweeps** (`--auto-recover`): an early warning on few tracked points. On losing
  tracking, turn the camera back by the angle the gimbal's IMU says it has turned since tracking
  was last OK (inside ORB-SLAM3's 3 s window); after a new map, sweep so the maps can merge.
  Measured on route C.
- **Done when:** the protocol is documented, and G1/G3/G4 run and are measured (3 runs per
  condition).

### Stage 3: encoders + LiDAR + fusion ([details](stage3.md)), future
- ROS 2 Humble.
- An ORB-SLAM3 wrapper node using only the public API; its pose goes to the filter as *relative*
  motion.
- Wheel odometry and the RPLidar A1 with `slam_toolbox`.
- A `robot_localization` EKF.
- Five configurations (C1–C5) replayed from the **same recorded bags**, on routes A–F.
- **Open decisions first:** the rover's drive type, its motor controller and encoders, and where
  the LiDAR mounts.

---

## Current status (2026-09-27)

| Item | Status |
|---|---|
| JetPack 6.2.3 (L4T 36.5.2) on NVMe, `nvidia-jetpack` installed | ✅ done |
| CUDA 12.6 / TensorRT 10.3 / OpenCV 4.8.0 / numpy | ✅ checked |
| The repo's prebuilt Orin binary runs on this JetPack | ✅ checked |
| `xvfb`, `tmux` | ❌ install ([stage1.md, section 2](stage1.md#2-housekeeping-on-the-jetson-15-min)) |
| EuRoC MH01 on the Jetson | ⏳ downloading through `run_euroc.sh` (the earlier HTTP 429 block has lifted) |
| Two copies of the repo on the Jetson | ⚠️ keep the one with remote `innovative-saswata15/Jetson-ORB-SLAM3-Hardware` |
| Rover driver, `tools/summarize_run.py`, `tools/rover_run.sh`, CMake target | ✅ written on the PC; syntax-checked and summary logic tested. Not yet built on the Jetson. Moved to the Jetson with git push and pull ([runbook, Part B](stage1_runbook.md#part-b-get-our-new-files-onto-the-jetson-with-git)) |

**Next action:** follow [stage1_runbook.md](stage1_runbook.md) from Part A.

## Conventions used in all stages

- **Run folders:** `~/runs/<YYYYmmdd-HHMMSS>_<route><n>_<condition>`, e.g.
  `20261003-101500_B2_base`, `..._B2_g3`. Each holds `console.log`, `events.csv`, `frames.txt`,
  `keyframes.txt` and `run_info.txt`.
- **3 runs per route and condition.** Same operator, lighting and start mark.
- **Numbers come from `tools/summarize_run.py`,** so every comparison is computed the same way.
- **Driving:** ≤ 0.5 m/s and turns ≤ 30°/s. Stop on the start mark and wait ~5 s before ending a
  run.

## Main risks (details in each stage document)

| Risk | Stage | Mitigation |
|---|---|---|
| EuRoC download rate-limited (HTTP 429) | 1 | Wait and retry once; PC download as a fallback |
| Pangolin v0.6 or librealsense build issues on JetPack 6.2.3 | 1 | `-j2`; known small fixes |
| D435 on USB 2 | 1 | Direct connection, short USB 3 cable |
| STorM32 firmware won't accept angle commands | 2 | Discovery first (T1–T5); operator-assisted fallback |
| 3 s relocalisation window too short | 2 | Early warning; phase B aims for a merge |
| Rover has no usable encoders | 3 | Decide before Stage 3; add encoders |
