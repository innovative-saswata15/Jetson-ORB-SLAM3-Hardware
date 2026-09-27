# Progress log

A chronological record of what was actually done, the real outputs, and what we learned along the
way. The other documents describe the plan ([plan.md](plan.md)) and the procedure
([stage1_runbook.md](stage1_runbook.md)). This file records what happened.

**Machines**

| | Details |
|---|---|
| **Jetson** | Orin Nano Developer Kit, user `orb-slam3`, IP on our network `192.168.1.5`. Repo at `~/Jetson-ORB-SLAM3-Hardware` |
| **PC** | Fedora 44 (x86-64), user `shiv`, IP `192.168.1.4`. Repo at `~/Desktop/code/Jetson-ORB-SLAM3-Hardware` |
| **Camera** | Intel RealSense **D435** (no IMU), serial `827312071682` |
| **Other hardware** (not used yet) | STorM32 BGC v1.3 3-axis gimbal (with its own IMU), RPLidar A1 (2D), a manually driven rover (no wheel encoders) |

---

## 1. Understanding the repo

What this repo is: ORB-SLAM3 (stereo, mono, RGB-D, with or without IMU) with its ORB
feature-extraction step moved onto the Jetson's GPU. The CUDA kernels reproduce the CPU
implementation, so accuracy matches the original. An optional CNN (CosPlace via TensorRT) can
propose loop-closure candidates.

Findings from reading the code (library unchanged; these only matter for how we use it):

| Finding | Where |
|---|---|
| The GPU path replaces only four stages: FAST corners, orientation, Gaussian blur, descriptors. The pyramid and keypoint distribution stay on the CPU, which is what keeps results identical | [src/ORBextractor.cc:1239-1330](../src/ORBextractor.cc#L1239-L1330) |
| `CPU_ORB=1` switches the same binary to the reference CPU extractor | [src/Tracking.cc:605](../src/Tracking.cc#L605) |
| `PIPELINE_FE=1` builds its worker extractors as **CPU** extractors, so pipelining turns GPU ORB off | [src/Tracking.cc:626-628](../src/Tracking.cc#L626-L628) |
| Left and right extraction run one after the other, because the GPU code uses shared static buffers | [src/Frame.cc:128](../src/Frame.cc#L128) |
| Much of `src/cuda/` (the Jetson-SLAM `ORB_GPU` path, the matcher and frustum kernels) is compiled but never called | `if(false)` at [src/ORBextractor.cc:1121](../src/ORBextractor.cc#L1121) |
| The live stereo driver `stereo_realsense_D435i` opens **only the two IR streams**, with the projector off, so it works on a plain D435 | [Examples/Stereo/stereo_realsense_D435i.cc](../Examples/Stereo/stereo_realsense_D435i.cc) |
| That driver can only be stopped with the viewer's **Stop** button (Ctrl-C only prints "Finishing session"), and it **never saves a trajectory** | same file |
| Stereo mode needs **no IMU**: the driver uses `System::STEREO`, `TrackStereo`'s IMU argument is optional, and IMU settings are read only for inertial modes | [src/Settings.cc:162-163](../src/Settings.cc#L162-L163) |
| ORB-SLAM3 sets its own verbosity to quiet at start-up, so only a few messages are printed (see [stage1.md, section 1](stage1.md#1-what-we-use-from-the-repo)) | [src/System.cc:244](../src/System.cc#L244) |
| In stereo mode, after losing tracking it tries to relocalise for **3 s** (only if the map has more than 10 keyframes), then starts a new map | [src/Tracking.cc](../src/Tracking.cc), `RECENTLY_LOST` handling |

## 2. Scope decisions

Decided in discussion, and recorded in [plan.md](plan.md):
- **Run the repo as-is.** The library (`src/`, `include/`, `Thirdparty/`) is never modified;
  everything we add goes on top.
- **Stereo visual SLAM** on the D435's IR pair, with **no IMU**. Stereo gives metric scale.
- **Three stages:**
  1. D435 + Jetson + rover;
  2. gimbal add-on (stability, coverage sweeps, recovery sweeps);
  3. wheel encoders + LiDAR + loose fusion.

  **Only Stage 1 is being done now.**
- Feeding gimbal rotation into ORB-SLAM3's tracker was **rejected**, because it would modify the
  library.
- LiDAR fusion is parked. 2D LiDAR SLAM needs odometry, and there are no encoders yet.
- The rover is driven by hand.
- ORB-SLAM3 doesn't need the camera to rotate. The gimbal only helps ORB-SLAM3 through
  stability; commanded sweeps are Stage 2 add-ons.

## 3. Setting up the Jetson (JetPack)

| Step | What happened / what we learned |
|---|---|
| Which JetPack | JetPack **6.2.3** = Jetson Linux **L4T 36.5.2**. The repo's prebuilt binaries were built on JetPack 6.2 (L4T 36.4.3), but they depend only on the CUDA 12, OpenCV 4.8 and TensorRT 10 major versions |
| Install options explored | There's **no ISO or USB-stick installer for JetPack 6** (NVIDIA's "Jetson ISO" exists only for JetPack 7.2+). The official SD-card path is the 6.2.1 image plus an APT upgrade to 6.2.3. A USB drive can be a flash *target*, not an installer. SDK Manager under Windows (WSL2) **can't flash external storage** (NVMe/USB), so a native Ubuntu host is needed |
| Firmware check | UEFI screen (press Esc at boot) showed **36.5.2**, already JetPack-6 firmware, so the JetPack 5.1.3 "bridge" update was **not** needed |
| microSD slot | On the dev kit it's on the underside of the compute module, hidden under the heatsink. Not used: we boot from NVMe |
| Flash | **SDK Manager on a native Ubuntu PC → JetPack 6.2.3 onto the NVMe SSD.** Jetson Linux flashed fine; the SDK-components step then failed with *"The connected Jetson device is not ready for flash … non-optimal USB connection"* |
| Fix | On the Jetson: `sudo apt install nvidia-jetpack`, which installs the same components (CUDA, cuDNN, TensorRT, VPI, OpenCV) from NVIDIA's apt repo. No reflash needed |
| Verified | `R36 REVISION 5.2`; `nvidia-jetpack 6.2.3+b81`; CUDA **12.6.68**; TensorRT **10.3.0.30**; OpenCV **4.8.0**; root on `/dev/nvme0n1p1`, 915 GB with 849 GB free; RAM 7.4 GB plus 3.7 GB swap; numpy 1.21.5 |
| Power profile | Left at stock. The README forbids `nvpmodel` and `jetson_clocks` |
| Updates | Ubuntu reports ~300 upgradable packages. **We deliberately don't run `apt upgrade`** during this work, so the JetPack base doesn't change underneath us |

## 4. Repo on the Jetson and the git workflow

- The repo is at `~/Jetson-ORB-SLAM3-Hardware`. An older copy, `~/Jetson-ORB-SLAM3`, also existed;
  only the `-Hardware` clone is used.
- **The prebuilt binary runs on JetPack 6.2.3:** `ldd` found every library, and it printed its
  `Usage` line.
- **Workflow:** edit and commit **on the PC**, `git push origin main`, then on the Jetson
  `git pull --ff-only origin main`.
  - Pulling needs **no GitHub login**.
  - Pushing from the Jetson would need one, so files created on the Jetson (the calibration
    file) are copied to the PC with `scp` and committed there.
- Commits so far:

| Commit | Content |
|---|---|
| `2aec1b1` added plans | plan documents, rover driver, tools, CMake target. It also accidentally included `.claude/settings.local.json`: a local Claude Code permission list, **no secrets** |
| `2613f4d` edited gitignore | ignore `.claude/`, `.vscode/`, `output/`, `*.trt`; `.claude/settings.local.json` untracked (kept on disk) |
| `f055bf3`, `e11acf4` | runbook: GitHub login optional, calibration committed via the PC; minor edits |
| `1402940` | rover driver no longer counts start-up frames as dropped; runbook fixes |
| `d1821b9` | calibration file for our D435 (serial 827312071682) |

## 5. EuRoC check with the prebuilt binary (runbook Part C)

- **Download:** `run_euroc.sh` downloads EuRoC MH01 from ETH's research collection.
  - The first attempt was interrupted (Ctrl-C at 27 %). Retries then got **`HTTP Error 429: Too
    Many Requests`**: the server rate-limits the requesting network.
  - Ctrl-C is safe: the script deletes its temporary file.
  - After waiting, the download worked on the Jetson directly: 1.57 GB into
    `~/datasets/MH_01_easy`.
  - **Fallback**, if it's blocked again: extract the downloader from `run_euroc.sh` and run it on
    the PC, then `rsync` the result over. See runbook C1.
- **Run** (`./run_euroc.sh MH01`, prebuilt binary, GPU ORB):

| Result | Value |
|---|---|
| Frames tracked | **3682**, all in **one map** (no tracking failures) |
| Keyframes | **124** |
| **ATE RMSE** | **3.72 cm** (SE(3) aligned, 118 poses) |
| Tracking time | median **74.7 ms**, mean 75.3 ms per frame (~13 fps) |
| `tegrastats` during the run | GPU (`GR3D_FREQ`) pulsing 0–42 %, two CPU cores 50–90 %, RAM ~2.0 of 7.6 GB, no swap, ~47 °C, ~6 W |

- **Reading the ATE:** the README's example run gave 2.14 cm (with 312 keyframes); the ORB-SLAM3
  paper reports **3.6 cm** for stereo-inertial on MH01. Repeat runs vary by up to about a
  centimetre (ORB-SLAM3 is multi-threaded). So 3.72 cm **matches the original ORB-SLAM3**.
- The EuRoC example processes frames at the recording's real speed, so the 3-minute sequence
  takes a few minutes.
- `GPU ORB enabled` is printed 4 times: one per extractor (left, right, and two "eco" extractors
  the repo also creates).
- `[CNN] No TensorRT engine; loop closure uses DBoW2 only` is expected: the optional CNN model
  isn't installed.
- **C2 (`CPU_ORB=1` / `PIPELINE_FE=1` comparison) was skipped.** It's optional. It stays
  available if live tracking speed becomes a problem.

## 6. Source build with RealSense support (runbook Part D)

| Step | What happened |
|---|---|
| D1 packages | Most were already installed (including librealsense's dependencies). Newly installed: Boost serialization 1.74, GLEW 2.2, Eigen **3.4.0**. CMake **3.22.1** |
| D2 librealsense | **Already installed** from an earlier `~/librealsense` checkout: **v2.55.1**, with `realsense2Config.cmake` in `/usr/local/lib/cmake/realsense2/` and `rs-enumerate-devices` 2.55.1. The udev rules were present (`99-realsense-libusb.rules`, plus `99-realsense-d4xx-mipi-dfu.rules`, which is irrelevant to the D435). **No rebuild needed** |
| D2d camera | `Intel RealSense D435`, serial 827312071682, **USB 3.2**, interfaces at **5000M**. Connected with a **USB 3 C-to-C cable**: NVIDIA documents host mode on the dev kit's USB-C port, and it gave full USB 3 |
| Firmware | Camera **5.17.3.10**; librealsense 2.55.1 recommends 5.16.0.1. **Kept 5.17.3.10**: newer firmware works with older librealsense, and "updating" would be a downgrade |
| D3 Pangolin | Found by the repo's CMake without any build, so it was already installed |
| D4 repo build | `BUILD=1 JOBS=2 ./run_euroc.sh MH01` built everything (~45 min of CUDA compiling) and re-ran EuRoC: **ATE 4.06 cm** (124 poses), within ~0.3 cm of the prebuilt run. `lib/libORB_SLAM3.so` (6.2 MB), `stereo_realsense_D435i` and our `stereo_realsense_D435_rover` all built. No warnings from our driver |
| Header warnings | `-Wreorder` warnings from `include/CameraModels/KannalaBrandt8.h` appear for every program including the library's headers. Harmless, and part of the unmodified library |
| Rebuilding one program | `cmake --build build -j2 --target stereo_realsense_D435_rover` takes about **1 minute** (one file). The 45-minute build only comes back if library code changes |

## 7. Our additions (on top of the repo)

| File | What it is | Status |
|---|---|---|
| `Examples/Stereo/stereo_realsense_D435_rover.cc` + CMake target | A copy of the live D435 driver, with the **same SLAM calls and camera settings**. It adds: `q`/Ctrl-C stop, `--no-viewer`, **saved trajectories**, an events log (state, tracked points, position, `track_ms`), frame copying and a wait timeout | Built and working (E1) |
| `tools/rover_run.sh` | One command per run: output folder, version info, `tegrastats`, console log, summary | Not used yet |
| `tools/summarize_run.py` | Turns a run folder into numbers (loops, merges, relocalisations, new maps, split, end-point error, tracking time, dropped frames) | Tested on a synthetic run |
| `Examples/Stereo/RealSense_D435.yaml` | **Our camera's calibration** | Committed `d1821b9` |

Two bugs we found and fixed during development:
- **Ctrl-C with `tee` could kill the driver before it saved.** The launcher uses `tee -i`, and the
  driver ignores `SIGPIPE`.
- **The ~10 s vocabulary load was counted as ~310 "dropped frames".** The count now resets when
  tracking starts (commit `1402940`).

**Is the rover driver needed?** Not for Milestone 1: Part F uses the **original** driver. On the
rover it's needed only to get **saved trajectories and numbers**. For a visual demo, the original
driver over VNC is enough. Someone running the repo on this hardware needs only librealsense, a
source build and (for correct scale) our calibration file; none of our tooling.

## 8. Camera calibration (runbook Part E)

- **E1:** the rover driver's start-up printout (a run with the template settings, stopped with
  `q`) reported:

| | Our D435 | Template (another unit) |
|---|---|---|
| fx, fy | **385.4061** | 382.613 |
| cx | **318.2860** | 320.183 |
| cy | **238.9506** | 236.455 |
| baseline | **0.0499 m** | 0.0499585 |

- That was also the rover driver's **first real run**: it streamed both IR cameras, built a map
  (`New Map created with 243 points`), stopped with `q`, waited 5 s, and saved `frames.txt` and
  `keyframes.txt`.
- **E2:** wrote `Examples/Stereo/RealSense_D435.yaml` (template, with our five values). `ThDepth
  40.0`, 640×480 and 30 fps unchanged.
- **E3:** copied to the PC with `scp`, and committed as `d1821b9` ("Calibration for our D435
  (serial 827312071682)"). Pulled on the Jetson.
- **Timing seen live:** `1 dropped frs` after almost every frame. Tracking takes longer than 33 ms,
  so every other camera frame is skipped: **≈15 fps effective**. That's consistent with the
  EuRoC timing. It's fine for slow movement; running the camera at 15 fps, or trying
  `PIPELINE_FE`, stay options if it proves a problem.

## 9. Display: a virtual screen over VNC (runbook Part V)

We don't use a monitor for the Jetson (the dev kit has only DisplayPort out). The
original driver needs a display for its viewer and Stop button, so we set up a **virtual screen**:
- **Xvfb** display `:1`;
- **openbox**, so windows can be moved;
- **x11vnc** on port **5910**, localhost only;
- reached through an **SSH tunnel** from the PC with TigerVNC.

Problems hit on the way, and their fixes:

| Problem | Cause | Fix |
|---|---|---|
| `bind [127.0.0.1]:5901: Address already in use` on the PC | An old tunnel still held port 5901 | Kill old tunnels (`pkill -f "ssh .*-L 59"`); use another port |
| `vncviewer` never opened | Several commands were pasted into one terminal. After `ssh` starts, the rest isn't run on the PC | Tunnel and viewer in **separate terminals**; `ssh -N` for a tunnel-only session |
| Nothing visible via the tunnel | `x11vnc` reported `listen6: bind: Address already in use`: something else holds port 5900 on IPv6, and `localhost` could reach that instead of our x11vnc | Use port **5910** (`-rfbport 5910`) and tunnel to **`127.0.0.1`** explicitly |
| Viewer syntax | TigerVNC reads `host:N` as display number N | Use `vncviewer 127.0.0.1::5910` (a double colon means "port") |

The final procedure is in runbook **Part V**.

**Status: working.** The original driver, started over SSH with `DISPLAY=:1`, showed its windows
with the **live camera image** in the VNC viewer on the PC, and a screen recording of the VNC
window worked.

**Decision:** no monitor from now on. Every runbook command that opens a window uses
`DISPLAY=:1`, viewed through VNC.

---

## Current position

- ✅ Parts A–E of the runbook.
- ✅ Part V: the virtual screen works (camera feed visible in VNC, recording works).
- ⏳ **Next: Part F.** Tape the X start mark, then do the walk (F1, F3) and the checks (F4). This
  is the first live handheld SLAM run, and completes Milestone 1.
- Then Milestone 2 on the rover: route A only, since Stage 2 isn't planned now.
