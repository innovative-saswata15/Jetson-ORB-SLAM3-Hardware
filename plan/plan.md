# Plan: Jetson-ORB-SLAM3 on a rover, with a gimbal add-on

This is the overview. Each stage has its own detailed document:

| Stage | Document | Status |
|---|---|---|
| **1. The repo on our hardware**: D435 + Jetson, handheld then on the rover | [stage1.md](stage1.md), and the step-by-step [stage1_runbook.md](stage1_runbook.md) | **Current**, the only stage planned now |
| **2. Gimbal add-on**: stability, coverage sweeps, recovery sweeps | [stage2.md](stage2.md) | Not planned yet |
| **3. Wheel encoders + RPLidar A1**, fusion analysis | [stage3.md](stage3.md) | Not planned yet |

**What has actually been done**, with real outputs and everything we learned along the way:
[progress_log.md](progress_log.md).

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
| Plan layout | One overview plus one detailed document per stage, a runbook, and a progress log | Each detail lives in exactly one place |
| Current scope | **Stage 1 only** | Stages 2 and 3 later, if at all |
| Milestone 2 routes | **Route A only** (3 runs). B and C only if Stage 2 starts | B and C exist as references for Stage 2's gimbal features |
| Rover driver | Optional: only for Milestone 2 **with measurements** | The SLAM is identical. The original driver over VNC is enough for a visual demo |
| Display | **No monitor, ever.** A virtual screen over VNC (Xvfb + openbox + x11vnc on port 5910, SSH tunnel, TigerVNC); programs with windows run with `DISPLAY=:1` | The original driver needs a display for its viewer and Stop button. Verified working: the camera feed shows in VNC and the VNC window can be recorded |
| JetPack install | 6.2.3 flashed to NVMe with SDK Manager (native Ubuntu); components through `apt install nvidia-jetpack` | SDK Manager's component step failed over USB; apt does the same job |
| Camera firmware | Keep 5.17.3.10 (librealsense 2.55.1 recommends 5.16.0.1) | Newer firmware works; "updating" would be a downgrade |
| Git workflow | Commit on the PC, push, `git pull --ff-only` on the Jetson; Jetson-made files go to the PC by `scp` | Pulling needs no login, pushing would |
| `apt upgrade` | Not run during this work | Keeps the JetPack base stable |

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
- **Milestone 1, handheld:**
  1. ✅ reproduce the repo's EuRoC result: **3.72 cm** ATE with the prebuilt binary (paper:
     3.6 cm);
  2. ✅ build from source with RealSense support: **4.06 cm** ATE;
  3. ✅ write our D435's calibration file;
  4. ✅ walk a loop with the original live driver, and check that it closes: **passed on attempt
     2**, over the virtual screen.
- **Milestone 2, rover:**
  - rigid mount and power;
  - **route A** (closed loop), 3 runs;
  - either **visual only** (the original driver over VNC, plus a screen recording), or **with
    measurements** (the rover driver: saved trajectories, and `tools/summarize_run.py` numbers).
- **Done when:** route A closes its loop on the rover in all 3 runs.

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
| JetPack 6.2.3 (L4T 36.5.2) on NVMe, `nvidia-jetpack` installed | ✅ CUDA 12.6.68, TensorRT 10.3.0.30, OpenCV 4.8.0 |
| Repo on the Jetson, git workflow | ✅ `~/Jetson-ORB-SLAM3-Hardware`; PC and Jetson both on `d1821b9` |
| EuRoC MH01 with the prebuilt binary (runbook C1) | ✅ ATE **3.72 cm**, 3682 frames in one map, tracking ~75 ms per frame |
| Source build with RealSense support (runbook D) | ✅ ATE **4.06 cm**; librealsense v2.55.1 and Pangolin were already installed |
| D435 on USB 3 | ✅ serial 827312071682, firmware 5.17.3.10, USB 3.2 via a USB 3 C-to-C cable |
| Calibration file (runbook E) | ✅ fx = fy = 385.4061, cx = 318.2860, cy = 238.9506, b = 0.0499 m, committed `d1821b9` |
| Rover driver | ✅ built and working (E1 run). Optional from here on |
| Live tracking speed | ≈15 fps effective (every other 30 fps frame skipped). Fine at slow speeds |
| Virtual screen over VNC (runbook V) | ✅ working: the driver's windows and live camera image show in VNC; recording the VNC window works |
| First live SLAM walk (runbook F), **Milestone 1** | ✅ **passed on attempt 2**: `*Loop detected`, no tracking loss, one map. Attempt 1 failed (tracking lost near the end, and the walk ended standing still); the procedure was corrected |
| Milestone 2 (rover, route A) | not started |

**Next action: Milestone 2** on the rover (runbook Parts H and I; Part G only for the "with
measurements" option).

## Conventions used in all stages

- **Run folders** (with the rover driver): `~/runs/<YYYYmmdd-HHMMSS>_<route><n>_<condition>`,
  e.g. `20261003-101500_A2_base`. Each holds `console.log`, `events.csv`, `frames.txt`,
  `keyframes.txt` and `run_info.txt`.
- **3 runs per route and condition.** Same operator, lighting and start mark.
- **Numbers come from `tools/summarize_run.py`,** so every comparison is computed the same way.
- **Driving / walking:** ≤ 0.5 m/s, and turns ≤ 30°/s, turned slowly. At the end, pass over the
  start mark and **continue 2–3 m along the start of the route** before stopping. ORB-SLAM3
  confirms a loop only after 3 matches with the old place, which needs several keyframes over
  the start area, which in turn needs the camera moving.

## Main risks (details in each stage document)

| Risk | Stage | Mitigation |
|---|---|---|
| EuRoC download rate-limited (HTTP 429) | 1 | Happened once. Wait and retry once; PC download as a fallback. **Resolved** |
| Pangolin or librealsense build issues on JetPack 6.2.3 | 1 | **Resolved:** both were already installed; the repo built cleanly |
| D435 on USB 2 | 1 | **Resolved:** USB 3.2 with a USB 3 C-to-C cable. Re-check `5000M` after mounting on the rover |
| Tracking slower than 30 fps (≈15 fps effective, ~75 ms per frame on EuRoC) | 1 | Move and drive slowly. If tracking suffers: run the camera at 15 fps, or compare `PIPELINE_FE=1` / `CPU_ORB=1` (runbook C2) |
| Virtual-screen viewer costs CPU (software drawing) | 1 | Watch `dropped frs`. If it matters: drag the windows smaller, or run with measurements headless (`--no-viewer`) |
| STorM32 firmware won't accept angle commands | 2 | Discovery first (T1–T5); operator-assisted fallback |
| 3 s relocalisation window too short | 2 | Early warning; phase B aims for a merge |
| Rover has no usable encoders | 3 | Decide before Stage 3; add encoders |
