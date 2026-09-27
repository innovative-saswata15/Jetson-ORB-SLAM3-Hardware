# Stage 1: the repo running on our hardware

Part of the [overall plan](plan.md). Stage 2 ([stage2.md](stage2.md)) builds on everything here.

**Goal:** run this repository's GPU-accelerated ORB-SLAM3, **unmodified**, live on:
- a Jetson Orin Nano;
- an Intel RealSense D435;
- a manually driven rover.

**Current scope: Stage 1 only.** Stage 2 and Stage 3 aren't planned yet, so Milestone 2 needs
only route A (section 4.6). The exact steps are in [stage1_runbook.md](stage1_runbook.md), and what
actually happened, with real outputs, is in [progress_log.md](progress_log.md).

**Two milestones:**

| | Proves | Rough time | Status |
|---|---|---|---|
| **Milestone 1**: handheld | The repo runs on our Jetson with our camera, live | 1–2 days | ✅ **done** (Step 4 passed on attempt 2) |
| **Milestone 2**: on the rover | The same on the target vehicle | 2–4 days | not started |

**What we add to the repo in this stage** (the library under `src/`, `include/` and `Thirdparty/`
stays untouched):

| File | Purpose | Needed for |
|---|---|---|
| `Examples/Stereo/RealSense_D435.yaml` | Calibration for our D435 unit (committed `d1821b9`) | **Everything.** It's the only addition needed to run the plain repo accurately on our camera |
| `Examples/Stereo/stereo_realsense_D435_rover.cc` + `CMakeLists.txt` target | A copy of the original live driver, with the **same SLAM calls and camera settings**, that stops with `q`, runs headless, saves trajectories and logs events | Milestone 2 **with measurements** only |
| `tools/rover_run.sh`, `tools/summarize_run.py` | One command per rover run; turns a run into numbers | Milestone 2 with measurements only |

**Display: a virtual screen over VNC. No monitor is used.** The original driver needs a display
(its viewer and Stop button). The Jetson runs a **virtual screen** (Xvfb + openbox + x11vnc), which
the PC shows through an SSH tunnel with TigerVNC. Every program with a window is started over SSH
with `DISPLAY=:1` in front. See [runbook Part V](stage1_runbook.md#part-v-virtual-screen-over-vnc-our-display).

---

## 1. What we use from the repo

| Piece | What it is | Used in |
|---|---|---|
| `Vocabulary/ORBvoc.txt.tar.gz` | The bag-of-words vocabulary for place recognition. `run_euroc.sh` unpacks it once to `Vocabulary/ORBvoc.txt` | Every run |
| `run_euroc.sh` | One-command EuRoC run. It uses the prebuilt binary if it runs on this board, otherwise builds from source (`BUILD=1` forces the build). Also downloads the data and prints ATE | Step 1, and as the build script in Step 2 |
| `prebuilt/orin-jp6/` | The repo already compiled for JetPack 6. **It doesn't include the RealSense drivers** | Step 1 only |
| [Examples/Stereo/stereo_realsense_D435i.cc](../Examples/Stereo/stereo_realsense_D435i.cc) | Live stereo driver. Opens **only the two IR streams** (640×480 @ 30, projector off), so it works on a plain D435. Creates `System(..., STEREO, true, ...)` | Step 4, and the base for the rover driver |
| [Examples/Stereo/RealSense_D435i.yaml](../Examples/Stereo/RealSense_D435i.yaml) | Settings template for that driver | Step 3 |
| `CPU_ORB=1` environment variable | Makes the same binary use the reference CPU feature extractor instead of the GPU one ([src/Tracking.cc:605](../src/Tracking.cc#L605)) | Optional GPU vs CPU comparison |

**The public `ORB_SLAM3::System` calls we use** (from [include/System.h](../include/System.h)):

| Call | Returns / does |
|---|---|
| `TrackStereo(imL, imR, t)` | Processes one stereo pair. Returns `Tcw`, the world-to-camera pose. The camera's position is `Tcw.inverse().translation()` |
| `GetTrackingState()` | `2` = OK, `3` = RECENTLY_LOST, `4` = LOST, `1` = NOT_INITIALIZED |
| `GetTrackedMapPoints()` | One entry per image feature. Non-null entries are the map points tracked in this frame, with outliers already removed |
| `MapChanged()` | `true` once after a loop closure or merge **in the current map** |
| `Shutdown()`, `isShutDown()` | Stop the system / check whether it has stopped |
| `SaveTrajectoryTUM(f)`, `SaveKeyFrameTrajectoryTUM(f)` | Write trajectories in TUM format (`t x y z qx qy qz qw`). Must be called **after** `Shutdown()` |

**What ORB-SLAM3 prints.** The library switches its own verbosity to "quiet" right after it
starts ([src/System.cc:244](../src/System.cc#L244)), so only these messages appear. They are our
measurements:

| Message | Meaning |
|---|---|
| `*Loop detected` | Loop closure: drift corrected |
| `*Merge detected` | Two maps joined into one |
| `Relocalized!!` | Tracking recovered within the same map |
| `Stored map with ID: N` | Tracking gave up; the old map was stored and a new map started |
| `Creation of new map with id: N` | Printed at every map start, **including once at start-up**, so it isn't a failure count by itself |

Messages such as `Track Lost...` or `Active map Reseting` are **not printed** at this verbosity.
Tracking losses, and map resets (a lost map with 10 or fewer keyframes is discarded silently),
are therefore caught by our driver's state log (section 4.5).

---

## 2. Housekeeping on the Jetson (≈15 min)

1. **Keep one copy of the repo.** There are two: `~/Jetson-ORB-SLAM3` and
   `~/Jetson-ORB-SLAM3-Hardware`. Keep the one whose `git remote -v` shows
   `innovative-saswata15/Jetson-ORB-SLAM3-Hardware` and whose `git log --oneline -1` shows
   `09de0ae`. Rename the other one (e.g. to `~/old-orbslam3-copy`) so nothing runs from it by
   mistake. All paths below use `~/Jetson-ORB-SLAM3-Hardware`.
2. **Put CUDA on the PATH.** librealsense's CUDA build needs `nvcc`:
   ```bash
   echo 'export PATH=/usr/local/cuda/bin:$PATH' >> ~/.bashrc
   echo 'export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH' >> ~/.bashrc
   source ~/.bashrc && nvcc --version | tail -1          # release 12.6
   ```
3. **Tools:**
   ```bash
   sudo apt install -y xvfb tmux htop
   ```
   - `xvfb` gives `run_euroc.sh` a virtual display over SSH, because its viewer is always
     created.
   - `tmux` keeps long jobs alive if SSH drops.
4. **Power profile: leave it alone.** The README says not to run `nvpmodel` or
   `jetson_clocks`; all published numbers use the stock profile.

---

## 3. Milestone 1: the repo live with the D435, handheld

### Step 1: reproduce the repo's own EuRoC result (prebuilt binary)

**Why:** this is the authors' own test on the authors' dataset. A pass shows the GPU pipeline
runs correctly on *our* board before any camera or build is involved. If anything later goes
wrong, we know the fault is in our part.

**1a. Get MH01 onto the Jetson.** `run_euroc.sh` downloads it itself on the first run
(~1.5 GB, from ETH's research collection), so step 1b is normally all that's needed.

The ETH server rate-limits repeated requests (`HTTP Error 429`). If that happens, wait at least
an hour and retry once. If it's still blocked, download on another machine with the repo's own
download code, extracted from `run_euroc.sh`, and copy it over:
```bash
# on the PC (~10–20 min, ~5 GB of temporary space)
cd ~/Desktop/code/Jetson-ORB-SLAM3-Hardware
sed -n "/python3 - <<'PY'/,/^PY$/p" run_euroc.sh | sed '1d;$d' > /tmp/euroc_dl.py
mkdir -p ~/datasets/MH_01_easy
BUNDLE=machine_hall CANON=MH_01_easy DEST=$HOME/datasets/MH_01_easy python3 /tmp/euroc_dl.py
ssh orb-slam3@<jetson-ip> "mkdir -p ~/datasets"
rsync -a --info=progress2 ~/datasets/MH_01_easy orb-slam3@<jetson-ip>:datasets/
```
`run_euroc.sh` looks for `~/datasets/**/MH_01_easy/mav0/cam0/data` and skips the download when
it's there.

**1b. Run it (Jetson):**
```bash
tmux new -s euroc
cd ~/Jetson-ORB-SLAM3-Hardware
./run_euroc.sh MH01 2>&1 | tee ~/euroc_gpu.log
```
The script unpacks the vocabulary (once, ~30 s), confirms the prebuilt binary runs, and runs
stereo-inertial SLAM on MH01, which takes a few minutes. Then it prints the ATE.

**Pass:**
```bash
grep -E "Using the prebuilt|GPU ORB enabled|ATE RMSE" ~/euroc_gpu.log
# ==> Using the prebuilt Orin binary ...
# GPU ORB enabled (new CUDA kernels)
#     ATE RMSE:     a few cm (README example 2.14 cm; the ORB-SLAM3 paper reports 3.6 cm for MH01)
```
- `[CNN] No TensorRT engine; loop closure uses DBoW2 only` is expected. The optional CNN model
  isn't installed.
- This run uses the **dataset's recorded IMU**. It doesn't need an IMU on our hardware.

**1c. Optional GPU vs CPU comparison** (~5 min; the data is already there):
```bash
CPU_ORB=1 ./run_euroc.sh MH01 2>&1 | tee ~/euroc_cpu.log
grep "ATE RMSE" ~/euroc_gpu.log ~/euroc_cpu.log        # should be close
```
This is the repo's central claim: GPU and CPU give the same accuracy.

### Step 2: build from source with RealSense support

**Why:** the prebuilt binaries have no D435 driver. The repo's `CMakeLists.txt` builds every
RealSense driver automatically **if** `find_package(realsense2)` succeeds
([CMakeLists.txt:44](../CMakeLists.txt#L44), [CMakeLists.txt:166](../CMakeLists.txt#L166)).
That requires librealsense to be installed first.

**2a. System packages:**
```bash
sudo apt install -y build-essential cmake git pkg-config libeigen3-dev \
    libboost-serialization-dev libglew-dev libssl-dev libusb-1.0-0-dev \
    libgtk-3-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev libudev-dev
```
Eigen from apt (3.4) satisfies `find_package(Eigen3 3.1.0 REQUIRED)`. OpenCV 4.8 from JetPack
satisfies `find_package(OpenCV 4.4)`.

**2b. librealsense.** There is already a `~/librealsense` folder; check what it is first.
```bash
cd ~/librealsense && git status | head -3 && git describe --tags     # which version?
```
- Use a recent release tag (v2.55.1 or newer). If the folder is an old or half-built checkout,
  fetch and check out a release tag, then remove any old `build/` folder.
- **RSUSB backend (`FORCE_RSUSB_BACKEND=ON`):** librealsense talks to the camera through
  libusb, so JetPack's kernel needs no patching. This is the standard approach on Jetson.
- **`-j2`:** more parallel jobs run the 8 GB board out of memory.
```bash
cd ~/librealsense
./scripts/setup_udev_rules.sh                     # lets a normal user open the camera
mkdir -p build && cd build
cmake .. -DFORCE_RSUSB_BACKEND=ON -DBUILD_WITH_CUDA=ON -DCMAKE_BUILD_TYPE=Release \
         -DBUILD_EXAMPLES=ON -DBUILD_GRAPHICAL_EXAMPLES=ON
make -j2                                          # ~30–60 min
sudo make install && sudo ldconfig
```
**Verify** (camera plugged **directly** into a Jetson USB port):
```bash
rs-enumerate-devices | grep -E "Name|Serial Number|Firmware Version|Usb Type Descriptor"
# Name: Intel RealSense D435 ; Usb Type Descriptor: 3.2   <- must be 3.x, not 2.1
DISPLAY=:1 realsense-viewer      # watched in VNC (runbook Part V): Infrared 1 + 2, 640x480, 30 fps
```
**D435 firmware:** each librealsense release lists a recommended camera firmware in its release
notes. If `rs-enumerate-devices` shows an older one, download that firmware image and run
`rs-fw-update -f <file>.bin`. **Don't unplug the camera during the update.**

**2c. Pangolin v0.6** (the viewer library; JetPack doesn't ship it):
```bash
cd ~ && git clone --branch v0.6 --depth 1 https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin && mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=OFF -DBUILD_TESTS=OFF
make -j2 && sudo make install && sudo ldconfig
```
v0.6 predates Ubuntu 22.04's compiler. A common failure is `'numeric_limits' is not a member of
'std'`, fixed by adding `#include <limits>` at the top of the header the error names. Send the
exact error if a different one appears.

**2d. Build this repo.** `run_euroc.sh` with `BUILD=1` runs exactly the repo's build steps
(Thirdparty DBoW2, g2o and Sophus first, then the main project into `build/`), then repeats the
EuRoC check on the fresh build:
```bash
cd ~/Jetson-ORB-SLAM3-Hardware
BUILD=1 JOBS=2 ./run_euroc.sh MH01 2>&1 | tee ~/euroc_src.log     # ~45 min of CUDA compiling
grep -iE "realsense|ATE RMSE" ~/euroc_src.log
ls -l Examples/Stereo/stereo_realsense_D435i       # must exist
```

**Pass:**
- the ATE is within about 1 cm of Step 1 (ours: 4.06 cm from source, against 3.72 cm prebuilt);
- `stereo_realsense_D435i` exists.

If it's missing, CMake didn't find librealsense. Check that `/usr/local/lib/cmake/realsense2/`
exists, then rebuild.

**Later rebuilds**, after adding our own files, only need the build tree:
`cmake --build ~/Jetson-ORB-SLAM3-Hardware/build -j2`.

### Step 3: camera configuration for our D435

**Why:** ORB-SLAM3 turns pixel positions into metres using the focal length, the image centre
and the stereo baseline. The template holds another unit's values. Ours differ slightly, and
wrong values bend the scale of the whole map.

**3a. Read our unit's values.** Either source works:
- `rs-enumerate-devices -c`: find the **Infrared 1, 640×480** block (`Fx`, `Fy`, `PPX`, `PPY`)
  and the **Extrinsic from "Infrared 2" to "Infrared 1"** translation;
- **or** start the original driver once (Step 4). It prints ` fx = ...`, ` cx = ...` and the
  `Tlr` matrix at start-up.

The baseline is the absolute value of the x translation, in **metres** (≈ 0.050).

**3b. Write the file:**
```bash
cd ~/Jetson-ORB-SLAM3-Hardware
cp Examples/Stereo/RealSense_D435i.yaml Examples/Stereo/RealSense_D435.yaml
```
Edit only these lines:

| Key | Set to | Notes |
|---|---|---|
| `Camera1.fx`, `Camera1.fy` | our Fx, Fy | template: 382.613 |
| `Camera1.cx`, `Camera1.cy` | our PPX, PPY | template: 320.183 / 236.455 |
| `Stereo.b` | our baseline in metres | template: 0.0499585 |

Keep everything else as it is, and here's why:
- `Camera.type: "Rectified"`: the D435 delivers already-rectified IR images.
- `Camera.width/height: 640/480` and `Camera.fps: 30` match what the driver requests.
- `Stereo.ThDepth: 40.0`: a point is "close" (reliable stereo depth) up to 40 × baseline ≈
  **2 m**.
- The ORB settings (`nFeatures: 1250`, 8 levels, FAST 20/7) are the repo's tested values.

Changing them would mean we're no longer testing the repo as-is.

### Step 4: live run, handheld (original driver)

**Why:** the first end-to-end run of the repo with our camera and our Jetson. We use the
**unmodified** driver, so this result is purely the repo's.

**Setup:**
- the virtual screen running and the VNC viewer open on the PC (runbook Part V). The command
  below runs over SSH with `DISPLAY=:1`, and its windows appear in VNC;
- the D435 plugged directly into a Jetson USB port;
- check `lsusb -t` shows **5000M** for the camera, or `rs-enumerate-devices` shows USB 3.x;
- hold the camera by hand, or tape it to a board. It must not wobble relative to your hand.

**Run** (over SSH):
```bash
mkdir -p ~/evidence/m1 && cd ~/Jetson-ORB-SLAM3-Hardware
tegrastats --interval 1000 > ~/evidence/m1/tegrastats.log &        # GPU/CPU/RAM record
DISPLAY=:1 ./Examples/Stereo/stereo_realsense_D435i Vocabulary/ORBvoc.txt \
    Examples/Stereo/RealSense_D435.yaml 2>&1 | tee ~/evidence/m1/live_m1.log
```

**What you'll see:**
- **"ORB-SLAM3: Current Frame"**: the IR image, with tracked features marked in green.
- **"ORB-SLAM3: Map Viewer"**, a 3D view:
  - black points: the map;
  - red points: the local map around you;
  - blue camera shapes: keyframes;
  - green: the camera now.

Stereo needs no initialisation motion: the map starts from the first frame.

**Procedure:**
1. Start facing a textured area: shelves, posters, furniture. Avoid a blank wall.
2. Walk slowly (≤ 0.5 m/s) and turn gently. Keep the camera level and pointing forward.
3. Walk a loop of ~10–20 m around the room, come back over the start point facing the same way,
   and **keep walking slowly along the first 2–3 m of the route**. ORB-SLAM3 confirms a loop only
   after 3 matches with the old place, collected either at once from neighbouring keyframes or
   one at a time over new keyframes ([src/LoopClosing.cc](../src/LoopClosing.cc), `DetectCommonRegionsFromBoW` and line 444). Both need several keyframes covering
   the start area, and keyframes are only made while the camera moves, so stopping on the start
   point isn't enough.
4. Hold the camera with both hands, and turn very slowly. Most tracking losses happen in turns.
5. Press **Stop** in the viewer menu. **Ctrl-C doesn't stop this driver**: it prints "Finishing
   session" and carries on, because its main loop only checks `SLAM.isShutDown()`.
6. `kill %1` to stop `tegrastats`.

(Our first attempt stood still on the start point at the end, and also lost tracking during the
final turn; no loop closed. The second attempt, with this procedure, closed the loop with no
tracking loss. See [progress_log.md](progress_log.md), sections 10–11.)

**Pass:**
- `GPU ORB enabled` in the log;
- tracking holds at walking speed: few or no `Stored map with ID` lines;
- the map looks metric: a 1 m object spans about 1 m of points;
- `*Loop detected` appears when you return to the start, and the trajectory visibly snaps into
  place.

**Evidence to keep:**
- on the Jetson, in `~/evidence/m1/`: copies of `~/euroc_gpu.log` and `~/euroc_src.log` (and
  `~/euroc_cpu.log` if run), plus `live_m1.log` and `tegrastats.log` from this run;
- on the PC: a **screen recording of the VNC window** during the loop (Fedora's GNOME recorder,
  Ctrl+Shift+Alt+R). The original driver saves no trajectory, so this video is the M1 evidence.

**Milestone 1 is done** when Steps 1–4 pass.

---

## 4. Milestone 2: on the rover

### 4.1 Camera mount

**Why a rigid bracket and no gimbal yet:** this stage produces the *baseline*. Stage 2 then adds
the gimbal and compares against it. If the baseline already used the gimbal, the comparison
would be meaningless.

1. **Bracket:** mount the D435 by its two **M3 holes on the back**, or its **1/4"-20 tripod
   thread** underneath plus a second anti-rotation point. A single tripod screw lets it twist
   under vibration. Use rubber dampers between the bracket and the chassis.
2. **Position:** high enough that the view isn't mostly floor, and near the rover's centre line.
3. **Orientation:** facing forward, level. The two IR lenses must be horizontal.
4. **Field of view check:** at 640×480, the IR view is ≈ **80° wide × 64° tall**, from
   `fx ≈ 383`: 2·atan(320/383) ≈ 80°. Put the rover on the floor, open `realsense-viewer`, and
   confirm **no part of the rover** (wheels, antenna, cables) appears in either IR image. Parts
   of the rover in view become fake "features" that move with the camera and corrupt tracking.
5. **USB cable:** short (≤ 1 m), strain-relieved at both ends, tied to the chassis. After
   mounting, `lsusb -t` must still show **5000M**.

### 4.2 Power

1. The Jetson dev kit takes DC through its barrel jack. On the rover, feed it from a
   **regulated DC-DC converter giving 19 V at ≥ 45 W**, with an inline fuse. Never connect the
   battery straight to the jack: battery voltage sags and spikes.
2. **Keep motor current off the Jetson's supply.** Rover motors starting or stalling cause
   voltage dips that can reboot the Jetson. Use a separate regulator branch from the battery.
3. **Check:** measure the voltage at the Jetson's plug while driving hard (starts, stops, turns).
   It must stay steady.
4. The D435 is powered from the Jetson's USB port (up to ~0.7 A). No external power is needed.
   Don't put an unpowered hub in between.

### 4.3 Remote operation

The rover moves, so we work over the network. **SSH** over Wi-Fi starts runs, inside `tmux` so a
Wi-Fi drop doesn't kill the run. Then choose one of two ways:

| | **Visual only** | **With measurements** |
|---|---|---|
| Program | The original `stereo_realsense_D435i`, unmodified | The rover driver (4.4), via `tools/rover_run.sh` |
| Viewing and stopping | The virtual screen over VNC (runbook Part V); click **Stop** in VNC | SSH only (`--no-viewer`); press `q` |
| Evidence | A screen recording of the loop closing | Saved trajectories and a summary with numbers (loops, end-point error, tracking time), and optionally a recording |

Either way, end each run by driving 2–3 m past the start mark along the start of the route (so
loop closure can confirm), then stop the rover, and only then stop the run.

### 4.4 The rover driver

**Only needed for Milestone 2 *with measurements*** (4.3). The SLAM is identical: same library,
same camera settings, same `TrackStereo()` calls. Its additions only affect running on a rover
and collecting data.

**Why it exists.** The original driver has three limitations for rover runs:
1. It can only be stopped with the viewer's **Stop** button, which needs a display.
2. It **never saves a trajectory**, so drift can't be measured.
3. It records nothing about tracking health: losses, resets, tracked-point counts.

A new file fixes all three. **The original driver and the library stay untouched.** This driver
is also the program the Stage 2 gimbal add-on plugs into.

**Implemented:** [Examples/Stereo/stereo_realsense_D435_rover.cc](../Examples/Stereo/stereo_realsense_D435_rover.cc),
its build target in `CMakeLists.txt`, and the launcher
[tools/rover_run.sh](../tools/rover_run.sh). The rest of 4.4 documents what the code does and
why. The step-by-step commands are in [stage1_runbook.md](stage1_runbook.md).

**4.4.1 Create the file and build target:**
```bash
cd ~/Jetson-ORB-SLAM3-Hardware
cp Examples/Stereo/stereo_realsense_D435i.cc Examples/Stereo/stereo_realsense_D435_rover.cc
```
In `CMakeLists.txt`, inside the existing `if(realsense2_FOUND)` block of the **Stereo examples**
section (right after the `stereo_realsense_D435i` target), add:
```cmake
    add_executable(stereo_realsense_D435_rover
            Examples/Stereo/stereo_realsense_D435_rover.cc)
    target_link_libraries(stereo_realsense_D435_rover ${PROJECT_NAME})
```
The Stereo section already sets `CMAKE_RUNTIME_OUTPUT_DIRECTORY` to `Examples/Stereo`, so the
binary lands next to the original. Build it:
```bash
cmake --build build -j2 --target stereo_realsense_D435_rover
```

**4.4.2 Command line:**
```
stereo_realsense_D435_rover <vocabulary> <settings> [--out DIR] [--no-viewer]
```
- `--out DIR`: output folder. The default is `~/runs/<YYYYmmdd-HHMMSS>/`, created at start.
- `--no-viewer`: passes `bUseViewer=false` to the `System` constructor, so it runs without any
  display.

Replace the original `argc` check (`argc < 3 || argc > 4`) with this parser. The original's
optional 4th argument was only passed to `System` as a sequence label; pass the run folder's name
there instead.

**4.4.3 Changes inside the copy**, in the order they appear in the file:

1. **Includes:** add `<atomic>`, `<thread>`, `<termios.h>`, `<unistd.h>`, `<poll.h>`,
   `<sys/stat.h>`, `<iomanip>`.
2. **Signal flag:** change `bool b_continue_session;` to `volatile sig_atomic_t
   b_continue_session;`. It's written from the Ctrl-C handler, so it must be safe to use inside
   a signal handler. `exit_loop_handler` itself stays as it is.
3. **Keyboard thread** (new; `q` quits; Stage 2 adds more keys). It uses "cbreak" terminal mode:
   single keypresses without Enter, no echo, and **Ctrl-C still works**, because `ISIG` stays on.
   ```cpp
   std::atomic<bool> g_quit{false};
   std::atomic<char> g_key{0};              // last key for Stage 2; 0 = none

   void keyboard_thread() {
       termios old{}, cb{};
       if (tcgetattr(STDIN_FILENO, &old) != 0) return;      // not a terminal (e.g. systemd)
       cb = old; cb.c_lflag &= ~(ICANON | ECHO);            // keep ISIG: Ctrl-C still works
       tcsetattr(STDIN_FILENO, TCSANOW, &cb);
       while (!g_quit && b_continue_session) {
           pollfd p{STDIN_FILENO, POLLIN, 0};
           if (poll(&p, 1, 100) > 0) {
               char c;
               if (read(STDIN_FILENO, &c, 1) == 1) { if (c == 'q') g_quit = true; else g_key = c; }
           }
       }
       tcsetattr(STDIN_FILENO, TCSANOW, &old);              // restore the terminal
   }
   ```
   Start it after the `System` is created (`std::thread kb(keyboard_thread);`), and `join()` it
   at the end.
4. **Main loop condition:** `while (!SLAM.isShutDown())` becomes
   `while (b_continue_session && !g_quit && !SLAM.isShutDown())`. Now `q`, Ctrl-C and the
   viewer's Stop button all end the loop.
5. **Per-frame bookkeeping**, directly after the existing `SLAM.TrackStereo(im, imRight,
   timestamp);`. Keep its return value:
   ```cpp
   Sophus::SE3f Tcw = SLAM.TrackStereo(im, imRight, timestamp);
   int state = SLAM.GetTrackingState();
   if (state != last_state) { ev.log(timestamp, "state", std::to_string(state)); last_state = state; }
   if (SLAM.MapChanged())      ev.log(timestamp, "map_changed", "");   // hint only, see below
   const auto mps = SLAM.GetTrackedMapPoints();
   int n_tracked = std::count_if(mps.begin(), mps.end(), [](ORB_SLAM3::MapPoint* p){ return p != nullptr; });
   if (timestamp - last_stat_t >= 1.0) {                               // once per second
       ev.log(timestamp, "tracked", std::to_string(n_tracked));
       if (state == 2) { Eigen::Vector3f c = Tcw.inverse().translation();
                         ev.log(timestamp, "pos", fmt3(c)); }           // camera centre, metres
       last_stat_t = timestamp;
   }
   ```
   - `ev` is a small events logger: it opens `out/events.csv` with the header
     `time,event,detail`, and each `log()` writes one line and flushes.
   - `fmt3` formats `x y z`.
   - Only use `Tcw` when `state == 2` (OK). While lost, the returned pose means nothing.
   - **About `MapChanged()`:** its counter belongs to the current map, so after a new map starts
     it can miss events. It's logged as a live hint only. The exact counts come from
     `console.log` (section 4.5).
6. **Shutdown and saving**, after the loop:
   ```cpp
   if (!SLAM.isShutDown()) SLAM.Shutdown();         // Stop button may already have done it
   std::this_thread::sleep_for(std::chrono::seconds(5));
   SLAM.SaveTrajectoryTUM(out + "/frames.txt");
   SLAM.SaveKeyFrameTrajectoryTUM(out + "/keyframes.txt");
   g_quit = true; kb.join();
   ```
   - **Why the 5 s wait:** `Shutdown()` asks the mapping and loop-closing threads to finish, but
     **doesn't wait for them**. The waiting code is commented out in
     [src/System.cc:519-566](../src/System.cc#L519-L566). A loop correction or global bundle
     adjustment may still be running, and saving too early would write an uncorrected path.
   - **Which map each file describes:**
     - `frames.txt` holds every frame of the **biggest** map;
     - `keyframes.txt` holds the keyframes of the **current (last)** map, after all loop
       corrections, which makes it the most accurate path.

     If the run ends in one map, they describe the same map. If it ended "split" (new maps that
     never merged back), each file holds a different piece, and the coordinates of different
     maps can't be compared.
7. **Run info:** at start, the driver writes `out/run_info.txt`, holding:
   - the command line, settings file and date;
   - whether `CPU_ORB` is set;
   - the camera's serial number, firmware and USB type;
   - the intrinsics and baseline it reports.

   At the end it adds the frame and dropped-frame counts. The launcher adds `versions.txt` (git
   commit, uncommitted files, L4T version) and `tegrastats.log`. Together they make every run
   traceable later.
8. **Tracking time:** each second the driver also logs `track_ms` (mean and max time spent in
   `TrackStereo`, in milliseconds). **Real time at 30 fps needs a mean below ~33 ms.**
9. **Robustness fixes over the original**, in our copy only:
   - images are copied inside the librealsense callback (the original kept pointers into the
     camera's buffers);
   - the image size comes from each frame (the original used a size that was set only after
     streaming began);
   - the frame wait times out after 1 s, so `q` works even if the camera stops.

**4.4.4 Running it**, through the launcher:
```bash
cd ~/Jetson-ORB-SLAM3-Hardware
tools/rover_run.sh A1_base --no-viewer
# drive ... stop the rover ... wait ~5 s ... press q
```
The launcher:
- creates `~/runs/<date-time>_A1_base/`;
- records `versions.txt` and `tegrastats.log`;
- runs the driver with `--out` pointing there, saving the output to `console.log`;
- prints the summary line, also saved as `summary.txt`.

Each run folder holds `console.log`, `events.csv`, `frames.txt`, `keyframes.txt`,
`run_info.txt`, `versions.txt`, `tegrastats.log` and `summary.txt`.

**4.4.5 Bench test before the rover:** run it handheld with the viewer, walk the Step 4 room
loop, and press `q`. Check that:
- it exits within ~6 s, and the terminal still echoes typing afterwards (it was restored);
- all files exist, and `keyframes.txt` has one line per keyframe;
- `events.csv` has `state` changes, and a `tracked` and `pos` line every second;
- Ctrl-C also exits cleanly and saves.

### 4.5 Run summary script: `tools/summarize_run.py`

It turns one run folder into one row of numbers, so every comparison in Stages 1 and 2 is
computed the same way.

**Implemented:** [tools/summarize_run.py](../tools/summarize_run.py). It takes several run
folders at once, and adds `median_tracked`, `mean_track_ms` and `dropped_frames` to the fields
below. Its core logic is:
```python
#!/usr/bin/env python3
"""Summarise one rover run folder: event counts, split detection, end-point error."""
import csv, math, sys
from pathlib import Path

run = Path(sys.argv[1])
log = (run / "console.log").read_text(errors="replace")
loops, merges = log.count("*Loop detected"), log.count("*Merge detected")
relocs, new_maps = log.count("Relocalized!!"), log.count("Stored map with ID")

states = [int(r["detail"]) for r in csv.DictReader(open(run / "events.csv")) if r["event"] == "state"]
losses = sum(1 for a, b in zip([None] + states, states) if b == 4 and a != 4)   # entries into LOST
split = new_maps > merges            # a new map that never merged back leaves the run split

kf = [list(map(float, l.split())) for l in open(run / "keyframes.txt") if l.strip()]
steps = [math.dist(p[1:4], q[1:4]) for p, q in zip(kf, kf[1:])]
path = sum(steps)
# end point vs the nearest keyframe from the first third of the route
early, walked = [kf[0]], 0.0
for k, d in zip(kf[1:], steps):
    walked += d
    if walked > path / 3:
        break
    early.append(k)
end_err = min(math.dist(kf[-1][1:4], e[1:4]) for e in early)

print(f"run={run.name} loops={loops} merges={merges} relocalized={relocs} "
      f"new_maps={new_maps} lost_events={losses} split={split} keyframes={len(kf)} "
      f"path_m={path:.2f} endpoint_m={end_err:.3f} "
      + (f"endpoint_pct={100*end_err/path:.2f}" if path > 0 and not split else "endpoint_pct=n/a"))
```
- **End-point error:** the run starts on the mark, goes round the loop, passes the mark again,
  and ends 2–3 m further along the start of the route. The error is the distance from the **last
  keyframe** to the **nearest keyframe in the first third of the route**. After a correct loop
  closure the two passes lie on top of each other, so this is the drift left after the loop
  correction. It's meaningful only if the run isn't split. (Tested on a synthetic 42 m loop:
  ending 2 m past the start, 3 cm off the first pass, gives 3 cm.)
- **Path length** is summed between keyframes, which slightly underestimates the distance
  driven. That's fine, because we compare runs with each other, not with an external standard.

### 4.6 Baseline drive tests

**Now: route A only.** It shows the repo working on the rover. Routes B and C exist to be compared
against Stage 2's gimbal features (G3, G4), so they're recorded **only if Stage 2 is started**.

**Driving rules, for every run:**
- speed ≤ 0.5 m/s, and turns ≤ 30°/s, like the handheld walk;
- the same operator, the same time of day and lighting, and the same start mark: tape an X on
  the floor, plus a line for the heading;
- at the end, drive **past the X and continue 2–3 m along the start of the route**, in the original direction, then stop and end the run (`q`, or **Stop** in VNC);
- turn slowly: tracking losses mostly happen in turns.

**Routes:**

| Route | Description | What it shows | Needed |
|---|---|---|---|
| **A. Closed loop** | ~30–50 m around rooms or a building block. Comes back over the start mark **facing the same heading**, and ends 2–3 m further along the start of the route | Normal loop closure and end-point error: "does the repo work on a rover" | **Now** (Milestone 2) |
| **B. Out-and-back** | Drive ~20–30 m along a corridor, **turn around**, drive back to the start mark | The weak case: the return trip sees everything from the other side, so a loop closure is unlikely and drift stays | Only for Stage 2 (reference for G3) |
| **C. Blank wall** | Drive normally for ~10 m (so the map has more than 10 keyframes), approach a featureless wall until tracking is lost, stop, then turn away and continue back to the start | Tracking loss: relocalisation vs new map | Only for Stage 2 (reference for G4) |

**Repeat each route 3 times.** Live runs vary, and one run can't show a difference.

**Folder naming** (with measurements): `~/runs/<date>_<route><n>_base`, e.g.
`20261003-101500_A2_base`.

**Results table** (with measurements), from `summarize_run.py`, kept as `~/runs/route_A.txt`:

| Run | loops | merges | relocalized | new_maps | lost_events | split | path_m | endpoint_pct |
|---|---|---|---|---|---|---|---|---|
| A1 … A3 | | | | | | | | |

If Stage 2 is started later, first add B1–B3 and C1–C3. Also note the median `tracked` count
while tracking is OK, and how low it drops just before a loss on route C: Stage 2's early-warning
threshold (G4) is set from those.

**Milestone 2 is done** when route A closes its loop in all 3 runs and doesn't end split. With
measurements, the end-point error should also be a few % of the path or less. **Stage 1 is then
complete.**

---

## 5. Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| `HTTP Error 429` during the EuRoC download | The ETH server is rate-limiting | Wait an hour, retry once; otherwise download on the PC (Step 1a) |
| `no DISPLAY and no xvfb-run` | Running `run_euroc.sh` over SSH | `sudo apt install xvfb` |
| Build killed or the Jetson freezes | Out of memory | `JOBS=2` (or `1`); close the browser and other apps |
| `stereo_realsense_D435i` missing after the build | librealsense not found by CMake | Check `sudo make install` in librealsense; `/usr/local/lib/cmake/realsense2` exists; rebuild |
| `No device connected, please connect a RealSense device` | Camera not visible to librealsense | Replug directly into the Jetson; `rs-enumerate-devices`; check the udev rules |
| Camera shows USB 2.x / `480M` | Cable, port, or plug not fully seated | Short USB 3 cable, flip the USB-C plug, another port |
| Many `dropped frs` messages | Tracking slower than 30 fps | Compare the GPU run with a `CPU_ORB=1` run; move slower. If it persists, consider lower resolution/fps (a config change, so record it) |
| Tracking lost often | Low texture, blur, fast turns | Textured areas, slower driving; check the exposure (the driver caps auto-exposure at 5 ms) |
| Tracking fails in sunlight | IR sensors saturate | Test indoors first; this is a known limit of IR stereo |
| Jetson reboots while the rover drives | Supply voltage dips | Separate regulator for the Jetson (section 4.2) |
| Terminal doesn't echo after a crash | The keyboard thread didn't restore the terminal | Type `reset` and press Enter |
