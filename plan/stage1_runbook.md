# Stage 1 runbook: exact steps on the Jetson

The step-by-step companion to [stage1.md](stage1.md), which explains the *why*. Work through it
top to bottom. Every step ends with a **✅ Check**. Don't move on until it passes. At each
**📋 Send** checkpoint, paste the output back for review.

- **Where commands run:** 🖥️ **PC** means your Fedora PC, in `~/Desktop/code/Jetson-ORB-SLAM3-Hardware`.
  🤖 **Jetson** means the Jetson, over SSH or at its own desktop.
- **User and paths on the Jetson:** user `orb-slam3`; the repo at `~/Jetson-ORB-SLAM3-Hardware`.
- Replace `<jetson-ip>` with the Jetson's address (`hostname -I` on the Jetson).

| Part | Content | Time |
|---|---|---|
| A | Jetson housekeeping | 20 min |
| B | Get our new files onto the Jetson with git | 10 min |
| C | EuRoC check with the prebuilt binary (downloads MH01) | 30 min + download |
| D | Build from source with RealSense support | 2–3 h (mostly waiting) |
| E | Our D435's calibration file | 15 min |
| F | Live handheld run (original driver): **Milestone 1** | 30 min |
| G | Rover driver bench tests | 30 min |
| H | Mount, power, first rover drive | ½–1 day |
| I | Baseline runs A/B/C × 3: **Milestone 2** | ½–1 day |

---

## Part A: Jetson housekeeping

### A1. Pick the one repo copy to use 🤖
```bash
for d in ~/Jetson-ORB-SLAM3 ~/Jetson-ORB-SLAM3-Hardware; do
  echo "== $d"; git -C "$d" remote -v | head -1; git -C "$d" log --oneline -1
done
```
**✅ Check:** `~/Jetson-ORB-SLAM3-Hardware` shows the remote
`innovative-saswata15/Jetson-ORB-SLAM3-Hardware` and the commit `09de0ae Add citation section to
README`.

Rename the other copy so nothing runs from it by mistake:
```bash
mv ~/Jetson-ORB-SLAM3 ~/old-orbslam3-copy
```
If it's `~/Jetson-ORB-SLAM3-Hardware` that's wrong or missing: stop and send the output.

### A2. Check the repo has no local edits 🤖
Step B pulls new commits, which only fast-forward cleanly if nothing tracked was edited here:
```bash
cd ~/Jetson-ORB-SLAM3-Hardware
git status --short | grep -v '^??' || echo "CLEAN: no modified tracked files"
```
**✅ Check:** prints `CLEAN: no modified tracked files`. Untracked files (lines starting `??`,
such as `Vocabulary/ORBvoc.txt` or `output/`) are fine.

### A3. Put CUDA on the PATH 🤖
```bash
grep -q '/usr/local/cuda/bin' ~/.bashrc || cat >> ~/.bashrc <<'EOF'
export PATH=/usr/local/cuda/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH
EOF
source ~/.bashrc
nvcc --version | tail -2
```
**✅ Check:** `Cuda compilation tools, release 12.6`.

### A4. Tools 🤖
```bash
sudo apt update
sudo apt install -y xvfb tmux htop git rsync python3-numpy
command -v xvfb-run tmux rsync
```
**✅ Check:** three paths are printed.

### A5. Resources 🤖
```bash
df -h ~ | tail -1        # need >= 20 GB free
free -h | head -2        # 7.4 Gi RAM, 3.7 Gi swap
```
**✅ Check:** at least 20 GB free. You have ~849 GB, so this is fine.

**Rule for the whole runbook:** don't run `nvpmodel` or `jetson_clocks` (the repo README
forbids it).

---

## Part B: get our new files onto the Jetson with git

The new files are:
- the plan documents (`plan/`);
- the rover driver (`Examples/Stereo/stereo_realsense_D435_rover.cc`);
- its build target (`CMakeLists.txt`);
- the tools (`tools/`).

They go into git on the PC and are pulled on the Jetson. From then on, every run records the
exact commit it used.

### B1. Commit and push 🖥️
```bash
cd ~/Desktop/code/Jetson-ORB-SLAM3-Hardware
git status --short
```
Expect:
- `M CMakeLists.txt`;
- `?? Examples/Stereo/stereo_realsense_D435_rover.cc`, `?? plan/` and `?? tools/`.

Local tool and editor folders (`.claude/`, `.vscode/`) and build outputs are excluded by
`.gitignore`, so they don't appear here.

```bash
git add CMakeLists.txt Examples/Stereo/stereo_realsense_D435_rover.cc plan tools
git status --short          # the four paths now staged
git commit -m "Stage 1: rover driver, run tools and plan documents"
git push origin main
git log --oneline -1        # note this commit id
```
**✅ Check:** the push ends with `main -> main`.

If it's rejected with `403` or `permission denied`, the PC's GitHub account has no write access to
`innovative-saswata15/Jetson-ORB-SLAM3-Hardware`. The repo owner must add it as a collaborator,
or push from the owner's account.

### B2. GitHub login on the Jetson: only if git asks 🤖
Pulling works without any login, as the first pull showed, and the runbook never pushes from the
Jetson (see E3). **Skip this step** unless a `git pull` ever stops to ask for a username.

If it does:
```bash
sudo apt install -y gh
gh auth login      # GitHub.com -> HTTPS -> Yes, authenticate Git -> Login with a web browser
gh auth status     # "Logged in to github.com as <account>"
```

### B3. Pull 🤖
```bash
cd ~/Jetson-ORB-SLAM3-Hardware
git status --short | grep -v '^??' || echo "CLEAN: no modified tracked files"     # as in A2
git pull --ff-only origin main
git log --oneline -1
ls plan/ tools/ Examples/Stereo/stereo_realsense_D435_rover.cc
grep -n stereo_realsense_D435_rover CMakeLists.txt
ls -l tools/*.sh tools/*.py
```
**✅ Check:**
- `CLEAN`;
- the pull says `Fast-forward`;
- `git log` shows **the same commit id as on the PC**;
- `plan/` lists `plan.md stage1.md stage1_runbook.md stage2.md stage3.md`;
- two `grep` matches;
- both tools are executable (`-rwxr-xr-x`).

If the pull stops with `untracked working tree files would be overwritten`, copies of these files
already exist on the Jetson. Move the listed files aside (e.g. `mv plan plan.old`) and pull again.

**Rule from now on:**
- **Edit tracked files only on the PC**, then commit, push, and `git pull --ff-only` on the
  Jetson. Then pulls always fast-forward.
- After a pull that changes C++ code, rebuild: `cmake --build ~/Jetson-ORB-SLAM3-Hardware/build -j2`.
- Files created *on the Jetson*, like the calibration file in Part E, are copied to the PC and
  committed there (E3).

---

## Part C: EuRoC check with the prebuilt binary (Milestone 1, Step 1)

### C1. Download and run 🤖
`run_euroc.sh` downloads MH01 itself (~1.5 GB; needs ~5 GB free while unpacking), then runs SLAM
and prints the ATE:
```bash
tmux new -s euroc                          # detach: Ctrl-b then d; reattach: tmux attach -t euroc
cd ~/Jetson-ORB-SLAM3-Hardware
./run_euroc.sh MH01 2>&1 | tee ~/euroc_gpu.log
```
It takes ~10–20 min for the download (only the first time) and a few minutes of SLAM.

**If a download is already running** (started before this runbook, perhaps outside tmux), let it
finish. Then:
- **If it stopped with `no DISPLAY and no xvfb-run`:** that's the SSH case without `xvfb`. Run
  `sudo apt install -y xvfb` (A4), then the three commands above. The dataset is already in
  `~/datasets`, so there's no second download.
- **If it completed:** run the three commands above anyway, to get the log file `~/euroc_gpu.log`.
  It takes a few minutes, with no download.

**✅ Check the dataset** 🤖:
```bash
D=$(dirname $(dirname $(find ~/datasets -maxdepth 4 -type d -path '*MH_01*/mav0/cam0' | head -1)))
echo $D
ls $D/mav0                                                  # cam0 cam1 imu0 state_groundtruth_estimate0
ls $D/mav0/cam0/data | wc -l; ls $D/mav0/cam1/data | wc -l   # same number, in the thousands
wc -l $D/mav0/state_groundtruth_estimate0/data.csv          # thousands of lines
```

**✅ Check the run** 🤖:
```bash
grep -E "Using the prebuilt|GPU ORB enabled|ATE RMSE" ~/euroc_gpu.log
ls -l Vocabulary/ORBvoc.txt
```
- `==> Using the prebuilt Orin binary ...`
- `GPU ORB enabled (new CUDA kernels)`
- `ATE RMSE:     X.XX cm`, with **X.XX between about 2 and 4**. For reference, the README's example run gave 2.14 cm, and the ORB-SLAM3 paper reports 3.6 cm for stereo-inertial on MH01; runs vary a little because ORB-SLAM3 is multi-threaded. Our first run gave 3.72 cm with 124 keyframes, in one map.
- Also note the `median tracking time` line (seconds per frame). Our first run gave **0.075 s (~13 fps)**; compare it with C2.
- `ORBvoc.txt` exists (~140 MB)

The line `[CNN] No TensorRT engine; loop closure uses DBoW2 only` is expected.

**Fallback, if `HTTP Error 429` comes back** (the ETH server rate-limits repeated requests): wait
at least an hour and retry **once**. Don't retry in a loop, because that extends the block. If
it's still blocked, download on the PC and copy it over:
```bash
# 🖥️ PC
cd ~/Desktop/code/Jetson-ORB-SLAM3-Hardware
sed -n "/python3 - <<'PY'/,/^PY$/p" run_euroc.sh | sed '1d;$d' > /tmp/euroc_dl.py
mkdir -p ~/datasets/MH_01_easy
BUNDLE=machine_hall CANON=MH_01_easy DEST=$HOME/datasets/MH_01_easy python3 /tmp/euroc_dl.py
ssh orb-slam3@<jetson-ip> "mkdir -p ~/datasets"
rsync -a --info=progress2 ~/datasets/MH_01_easy orb-slam3@<jetson-ip>:datasets/
```
Then run C1 on the Jetson again. It finds the dataset and skips the download.

### C2. GPU vs CPU comparison (optional, ~5 min) 🤖
```bash
CPU_ORB=1 ./run_euroc.sh MH01 2>&1 | tee ~/euroc_cpu.log
grep -E "CPU_ORB set|ATE RMSE" ~/euroc_cpu.log
```
**✅ Check:** `[ORB] CPU_ORB set: using reference CPU extractor`, and an ATE close to the GPU run.

Also run the README's pipelined mode, and compare tracking times:
```bash
PIPELINE_FE=1 ./run_euroc.sh MH01 2>&1 | tee ~/euroc_pipe.log
grep -E "median tracking time|mean tracking time|Map 0 has|ATE RMSE" ~/euroc_gpu.log ~/euroc_cpu.log ~/euroc_pipe.log
```
The live D435 runs at 30 fps, so it needs about **33 ms per frame** to keep up. These numbers
show how close each mode gets, and decide the plan for live runs.

**📋 Send:** the output of the C1 run check and the C2 `grep` (all three logs).

---

## Part D: build from source with RealSense support (Milestone 1, Step 2)

Use `tmux` for every long build.

### D1. System packages 🤖
```bash
sudo apt install -y build-essential cmake git pkg-config libeigen3-dev \
    libboost-serialization-dev libglew-dev libssl-dev libusb-1.0-0-dev \
    libgtk-3-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev libudev-dev
cmake --version | head -1; pkg-config --modversion eigen3
```
**✅ Check:** CMake ≥ 3.16 (22.04 has 3.22); Eigen 3.4.x.

### D2. librealsense 🤖

**D2a. What's in the existing folder, and is it already installed?**
```bash
git -C ~/librealsense describe --tags 2>/dev/null || echo "not a git checkout"
ls /usr/local/lib/cmake/realsense2/realsense2Config.cmake 2>/dev/null && echo "INSTALLED" || echo "NOT INSTALLED"
command -v rs-enumerate-devices && rs-enumerate-devices --version 2>/dev/null
```
- If it prints **`INSTALLED`**, and with the D435 plugged in `rs-enumerate-devices` lists it with
  USB 3.x (D2d), **skip to D2d**. It's usable.
- Otherwise, build it (D2b and D2c).

**D2b. Choose the version** (skip if it's already on a recent release tag):
```bash
cd ~/librealsense
git fetch --tags
git tag -l 'v2.*' --sort=-v:refname | grep -v -E 'rc|beta' | head -5     # newest releases first
git checkout <newest tag from that list, e.g. v2.5x.y>
rm -rf build
```
If `~/librealsense` isn't a git checkout, replace it:
```bash
mv ~/librealsense ~/librealsense.old
git clone https://github.com/IntelRealSense/librealsense.git ~/librealsense
```
Then run the three tag commands above again.

**D2c. Build and install** (~40–60 min):
```bash
cd ~/librealsense
./scripts/setup_udev_rules.sh
mkdir -p build && cd build
cmake .. -DFORCE_RSUSB_BACKEND=ON -DBUILD_WITH_CUDA=ON -DCMAKE_BUILD_TYPE=Release \
         -DBUILD_EXAMPLES=ON -DBUILD_GRAPHICAL_EXAMPLES=ON 2>&1 | tee ~/rs_cmake.log
make -j2 2>&1 | tee ~/rs_build.log
sudo make install && sudo ldconfig
```
**✅ Check**, after `cmake`:
```bash
grep -iE "cuda|rsusb|error" ~/rs_cmake.log | head
```
It should mention CUDA and show no `CMake Error`. After install:
`ls /usr/local/lib/cmake/realsense2/realsense2Config.cmake` must exist.

If `make` gets killed (out of memory), rerun it with `make -j1`.

**D2d. Camera check.** Plug the D435 **directly** into a Jetson USB port, not through a hub:
```bash
rs-enumerate-devices | grep -E "Name|Serial Number|Firmware Version|Usb Type Descriptor"
lsusb -t | grep -B1 -A1 -i "5000M"
```
**✅ Check:** `Name : Intel RealSense D435`, and `Usb Type Descriptor : 3.2` (any **3.x**).

`2.1`, or no `5000M`, means the USB 3 link failed. Flip the USB-C plug, try another port, use a
short USB 3 cable, and repeat. **Don't continue on USB 2.**

**D2e. Live image and firmware** (at the Jetson's desktop):
```bash
realsense-viewer
```
1. Turn on **Stereo Module**. Enable **Infrared 1** and **Infrared 2**, set them to
   **640×480, 30 fps**, and check both images move live.
2. If the viewer offers a **firmware update** (recommended version for this librealsense), accept
   it. **Don't unplug during the update.** Replug afterwards and repeat D2d.

**✅ Check:** both IR images are live at 30 fps, and the firmware is up to date.

### D3. Pangolin v0.6 🤖
```bash
ls /usr/local/lib/cmake/Pangolin/PangolinConfig.cmake 2>/dev/null && echo "ALREADY INSTALLED"
```
If it's not installed:
```bash
cd ~ && git clone --branch v0.6 --depth 1 https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin && mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=OFF -DBUILD_TESTS=OFF \
         -DBUILD_PANGOLIN_PYTHON=OFF -DBUILD_PANGOLIN_FFMPEG=OFF 2>&1 | tee ~/pangolin_cmake.log
make -j2 2>&1 | tee ~/pangolin_build.log
sudo make install && sudo ldconfig
```
- Options your Pangolin version doesn't know only produce a CMake warning; that's harmless.
- **Common failure:** `'numeric_limits' is not a member of 'std'` in some header. Add
  `#include <limits>` below the other `#include` lines of the header named in the error, then run
  `make -j2` again.
- For any other error, **📋 Send** the last 30 lines of `~/pangolin_build.log`.

**✅ Check:** `ls /usr/local/lib/cmake/Pangolin/PangolinConfig.cmake /usr/local/lib/libpangolin.so`
(both exist).

### D4. Build the repo (~45 min of CUDA compiling) 🤖
This runs the repo's own build steps (Thirdparty first, then the main project into `build/`),
then the EuRoC check on the fresh build:
```bash
tmux new -s build
cd ~/Jetson-ORB-SLAM3-Hardware
BUILD=1 JOBS=2 ./run_euroc.sh MH01 2>&1 | tee ~/euroc_src.log
```
**✅ Check** 🤖:
```bash
grep -E "Using the prebuilt|Building|GPU ORB enabled|ATE RMSE" ~/euroc_src.log
ls -l lib/libORB_SLAM3.so Examples/Stereo/stereo_realsense_D435i Examples/Stereo/stereo_realsense_D435_rover
```
- `==> Building -- first time only ...` is present, and `Using the prebuilt` is **absent**.
- `GPU ORB enabled`, and an ATE close to C1 (≈ 2 cm).
- All three files exist.

**If the two `stereo_realsense_*` binaries are missing,** CMake didn't find librealsense. Check
that `/usr/local/lib/cmake/realsense2/realsense2Config.cmake` exists, then:
```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j2
```

**If the build gets killed** (out of memory): `cmake --build build -j1`.

**Later rebuilds** (only after changing our files): `cmake --build ~/Jetson-ORB-SLAM3-Hardware/build -j2`.
`BUILD=1` rebuilds only if the binary is missing.

**📋 Send:** the D4 `grep` and `ls` output.

---

## Part E: our D435's calibration file (Milestone 1, Step 3)

### E1. Read our camera's values 🤖
The rover driver prints them at start-up, from the same stream ORB-SLAM3 will use. It can start
with the template settings for this:
```bash
cd ~/Jetson-ORB-SLAM3-Hardware
./Examples/Stereo/stereo_realsense_D435_rover Vocabulary/ORBvoc.txt \
    Examples/Stereo/RealSense_D435i.yaml --out /tmp/calib_probe --no-viewer 2>&1 | tee /tmp/calib_probe.log
```
Wait for `[ROVER] running.`, then press **q**. It saves and exits after ~6 s.
```bash
grep -E "^\[ROVER\] (camera|Infrared 1|stereo baseline|WARNING)" /tmp/calib_probe.log
```
You'll get something like:
```
[ROVER] camera: Intel RealSense D435  serial 12345...  firmware 5.xx  USB 3.2
[ROVER] Infrared 1 intrinsics: fx=382.1234 fy=382.1234 cx=319.8765 cy=238.4321  (640x480)
[ROVER] stereo baseline |tx| = 0.0500 m
```
**✅ Check:**
- no `WARNING` line;
- `640x480`;
- fx ≈ 380–390;
- baseline ≈ 0.049–0.051.

This was also the rover driver's first test: it started, and `q` worked.

For a more precise baseline (the driver rounds to 4 decimals), read the translation's x value in
`rs-enumerate-devices -c | grep -A6 'Extrinsic from "Infrared 2"'`.

### E2. Write `RealSense_D435.yaml` 🤖
Put **your** numbers from E1 into the variables. Keep the decimal points: OpenCV reads `383` and
`383.0` differently.
```bash
cd ~/Jetson-ORB-SLAM3-Hardware
FX=382.1234; FY=382.1234; CX=319.8765; CY=238.4321; B=0.0500      # <-- your values
cp Examples/Stereo/RealSense_D435i.yaml Examples/Stereo/RealSense_D435.yaml
sed -i -e "s/^Camera1.fx: .*/Camera1.fx: $FX/" -e "s/^Camera1.fy: .*/Camera1.fy: $FY/" \
       -e "s/^Camera1.cx: .*/Camera1.cx: $CX/" -e "s/^Camera1.cy: .*/Camera1.cy: $CY/" \
       -e "s/^Stereo.b: .*/Stereo.b: $B/" Examples/Stereo/RealSense_D435.yaml
grep -E '^(Camera1\.(fx|fy|cx|cy)|Stereo\.b|Stereo\.ThDepth|Camera\.(width|height|fps)):' Examples/Stereo/RealSense_D435.yaml
```
**✅ Check:**
- the five values are yours;
- `Stereo.ThDepth: 40.0`, `Camera.width: 640`, `Camera.height: 480` and `Camera.fps: 30` are
  unchanged.

### E3. Commit the calibration file (via the PC)
The file belongs to this camera unit, so it's versioned like everything else. Pushing needs a
GitHub login, and we keep all commits on the PC, so copy the file there first:
```bash
# 🖥️ PC
cd ~/Desktop/code/Jetson-ORB-SLAM3-Hardware
scp orb-slam3@<jetson-ip>:Jetson-ORB-SLAM3-Hardware/Examples/Stereo/RealSense_D435.yaml Examples/Stereo/
git add Examples/Stereo/RealSense_D435.yaml
git commit -m "Calibration for our D435 (serial <your serial from E1>)"
git push origin main
```
```bash
# 🤖 Jetson: its untracked copy is identical, so remove it and pull the committed one
cd ~/Jetson-ORB-SLAM3-Hardware
rm Examples/Stereo/RealSense_D435.yaml
git pull --ff-only origin main
git log --oneline -1
```
**✅ Check:**
- `git log --oneline -1` shows the same commit on both machines;
- the E2 `grep` on the Jetson still shows your values.

**📋 Send:** the E1 and E2 `grep` output.

---

## Part F: live handheld run with the original driver (**Milestone 1**, Step 4)

This must run **at the Jetson's desktop**, with a monitor, keyboard and mouse. The original driver
always opens its viewer, and its **Stop** button is the only way to end it.

### F1. Prepare
- The D435 is plugged directly into the Jetson; D2d passes (USB 3.x).
- A room with texture: furniture, shelves, posters. Normal lighting, no direct sun into the
  camera.
- Tape an **X** on the floor as the start mark, with an arrow for the heading.
- Start a screen recording: GNOME's built-in recorder (Ctrl+Shift+Alt+R), or any recorder.

### F2. Run 🤖 (terminal on the Jetson desktop)
```bash
mkdir -p ~/evidence/m1 && cd ~/Jetson-ORB-SLAM3-Hardware
tegrastats --interval 1000 > ~/evidence/m1/tegrastats.log &
./Examples/Stereo/stereo_realsense_D435i Vocabulary/ORBvoc.txt \
    Examples/Stereo/RealSense_D435.yaml 2>&1 | tee ~/evidence/m1/live_m1.log
```
Two windows open: **"ORB-SLAM3: Current Frame"** (IR image with green features) and
**"ORB-SLAM3: Map Viewer"** (3D).

### F3. Walk
1. Stand on the X, facing the arrow. Tracking starts immediately; stereo needs no special motion.
2. Walk **slowly** (≤ 0.5 m/s), turning gently, with the camera level and facing forward. Go
   round a loop of ~10–20 m.
3. Come back to the X, **facing the arrow again**. Hold still for ~5 s.
4. Click **Stop** in the Map Viewer menu, then close the windows. Ctrl-C does *not* stop this
   driver.
5. Stop tegrastats and the screen recording: `kill %1`.

### F4. Verify 🤖
```bash
grep -cE "\*Loop detected" ~/evidence/m1/live_m1.log            # >= 1
grep -cE "Stored map with ID" ~/evidence/m1/live_m1.log          # ideally 0
grep -E "GPU ORB enabled" ~/evidence/m1/live_m1.log | head -1
grep -c "dropped frs" ~/evidence/m1/live_m1.log                  # small
cp ~/euroc_gpu.log ~/euroc_src.log ~/evidence/m1/ ; ls ~/evidence/m1
```
**✅ Check (Milestone 1 pass):**
- **`*Loop detected` ≥ 1**, and in the recording the trajectory visibly snaps into place when you
  returned;
- `GPU ORB enabled`;
- the map looks metric: a 1 m wide table spans about 1 m of points;
- `Stored map with ID` is 0, or rare;
- few `dropped frs`.

**If no loop closes:** walk slower, return along the same final few metres, check the room has
texture, and try again.

**📋 Send:** the F4 output. Milestone 1 is then complete; save the screen recording in
`~/evidence/m1/`.

---

## Part G: rover driver bench tests (before any mounting)

These tests check that the rover driver is trustworthy: it exits cleanly and saves correct files
every time.

### G1. Handheld with the viewer 🤖 (Jetson desktop)
```bash
cd ~/Jetson-ORB-SLAM3-Hardware
tools/rover_run.sh bench1
```
Walk the same room loop as in F3, return to the X, and hold still ~5 s. Then press **q** in the
terminal, not in the viewer.

**✅ Check:**
- within ~6 s it prints `[ROVER] waiting 5 s ...` and `[ROVER] saved to ...`, then a
  `run=..._bench1 loops=...` summary line;
- your terminal echoes typing again afterwards (if not, type `reset` and press Enter);
- then:
  ```bash
  R=$(ls -d ~/runs/*_bench1 | tail -1); ls $R
  head -3 $R/keyframes.txt; wc -l $R/keyframes.txt
  grep -c ',tracked,' $R/events.csv; grep ',track_ms,' $R/events.csv | tail -3
  cat $R/summary.txt
  ```
  - the folder holds `console.log events.csv frames.txt keyframes.txt run_info.txt summary.txt
    tegrastats.log versions.txt`;
  - `keyframes.txt` has one line per keyframe (`timestamp x y z qx qy qz qw`);
  - `tracked` lines ≈ run length in seconds;
  - the summary shows `loops>=1`, `split=False`, and a small `endpoint_pct`;
  - **`mean_track_ms` below ~33.** That's real time at 30 fps. If it's higher, note it; Stage 1,
    section 5 has the options.

### G2. Headless over SSH 🤖
From an SSH session, with no monitor needed:
```bash
cd ~/Jetson-ORB-SLAM3-Hardware
tools/rover_run.sh bench2 --no-viewer
```
Walk a short loop carrying the Jetson and camera (or just move the camera around the desk), then
press **q**.

**✅ Check:** same files as G1, and `viewer: off` in `run_info.txt`.

### G3. Ctrl-C path 🤖
```bash
tools/rover_run.sh bench3 --no-viewer
```
Move the camera a little, then press **Ctrl-C** (not q).

**✅ Check:**
- the output still shows `[ROVER] waiting 5 s ...`, `[ROVER] saved to ...` and the summary line;
- `tail -1 $(ls -d ~/runs/*_bench3 | tail -1)/events.csv` shows `stop,sigint`.

**📋 Send:** `cat ~/runs/*_bench*/summary.txt`.

---

## Part H: mount, power, first rover drive (Milestone 2, 4.1–4.3)

### H1. Mount (rigid bracket; **no gimbal yet**, this is the baseline)
- [ ] D435 held by its **two M3 holes** on the back (or the tripod thread plus an anti-rotation
  point), on rubber dampers.
- [ ] Facing forward and level. The two IR lenses are horizontal.
- [ ] USB cable ≤ 1 m, strain-relieved at both ends, tied to the chassis.
- [ ] **Field of view check** at the Jetson desktop, rover on the floor: `realsense-viewer`,
  Infrared 1 and 2. **No part of the rover** (wheels, antenna, cables) is visible in either image.
- [ ] `lsusb -t` still shows `5000M` for the camera with everything mounted.

### H2. Power
- [ ] The Jetson is fed by a **regulated 19 V, ≥ 45 W DC-DC converter** with an inline fuse, and
  **never** straight from the battery.
- [ ] Rover motors are on a separate branch, not sharing the Jetson's regulator output.
- [ ] Measured at the Jetson's plug with a multimeter while driving hard (starts, stops, turns):
  steady.
- [ ] The Jetson doesn't reboot during a 5-minute drive with `tegrastats` running.

### H3. Remote operation
- [ ] The Jetson joins Wi-Fi and SSH works from the PC while the rover is away from the desk.
- [ ] Every run starts inside `tmux`, so a Wi-Fi drop doesn't kill it.

### H4. First drive: a slow straight line 🤖
```bash
tmux new -s rover
cd ~/Jetson-ORB-SLAM3-Hardware
tools/rover_run.sh smoke1_line --no-viewer
```
Drive ~10 m straight at walking pace, stop, wait ~5 s, and press **q**.

**✅ Check:**
- summary `split=False`, `lost_events=0`;
- `path_m` ≈ the distance driven (±10 %);
- `mean_track_ms` < 33;
- `grep -c "dropped frs" $(ls -d ~/runs/*_smoke1_line | tail -1)/console.log` is small.

If tracking is lost while driving, go slower first. Then check for vibration: if the image is
blurry in `realsense-viewer` while driving, improve the dampers.

**📋 Send:** the smoke run's `summary.txt`.

---

## Part I: baseline runs (**Milestone 2**, 4.6)

### I1. Prepare the routes (once)
- **Start mark:** an X plus a heading arrow, taped on the floor for each route.
- **Route A, closed loop:** ~30–50 m around rooms or corridors, ending on the X **facing the
  arrow**.
- **Route B, out and back:** ~20–30 m along a corridor, turn around, and come back to the X.
- **Route C, blank wall:** ~10 m of normal driving (so the map has more than 10 keyframes), then
  approach a plain wall until tracking is lost. Stop, turn away, and drive back to the X.
- Write each route down, with a sketch, so every run follows the same path.

### I2. Run each route 3 times 🤖
The rules for every run:
- speed ≤ 0.5 m/s, turns ≤ 30°/s;
- the same operator, time of day and lighting;
- at the end, stop exactly on the X, wait ~5 s, then press **q**.
```bash
cd ~/Jetson-ORB-SLAM3-Hardware
tools/rover_run.sh A1_base --no-viewer      # then A2_base, A3_base
tools/rover_run.sh B1_base --no-viewer      # then B2_base, B3_base
tools/rover_run.sh C1_base --no-viewer      # then C2_base, C3_base
```
**✅ Check after each run:** the summary line was printed, and the folder has all 8 files. If a
run was disturbed (someone walked in front, you drove off the route), **repeat it** and delete the
bad folder.

### I3. Collect the baseline 🤖
```bash
python3 tools/summarize_run.py ~/runs/*_base | tee ~/runs/baseline.txt
```
**✅ Check (Milestone 2 pass):**
- **all three A runs:** `loops ≥ 1`, `split=False`, `endpoint_pct` of a few % or less;
- three B and three C runs recorded; their numbers are whatever they are, since they're the
  reference for Stage 2;
- note the **median of `median_tracked`** across the A runs. It sets Stage 2's weak-tracking
  threshold (about a third of it).

**📋 Send:** `~/runs/baseline.txt`. **Stage 1 is then complete.**

### I4. Back up the evidence 🖥️
```bash
rsync -a orb-slam3@<jetson-ip>:runs orb-slam3@<jetson-ip>:evidence ~/Desktop/jetson-orbslam3-results/
```

---

## Quick fixes

| Symptom | Fix |
|---|---|
| `HTTP Error 429` | The ETH server is rate-limiting. Wait an hour and retry once; else use the PC fallback in C1 |
| `git pull` asks for a username/password | B2 (`gh auth login`); also check the remote URL is spelled correctly (`git remote -v`) |
| `git pull` refuses: not a fast-forward | A tracked file was edited on the Jetson. `git status`; move the edit to the PC, then `git checkout -- <file>` and pull |
| `no DISPLAY and no xvfb-run` | `sudo apt install xvfb` (A4) |
| Build killed / Jetson freezes | `JOBS=1`, or `make -j1` / `cmake --build build -j1` |
| `No device connected, please connect a RealSense device` | Replug directly; `rs-enumerate-devices`; re-run `~/librealsense/scripts/setup_udev_rules.sh` and replug |
| `WARNING: camera is not on USB 3` | Cable, port or plug orientation (D2d) |
| `WARNING: no camera frames for 2 s` | USB dropout. Check the cable strain relief; watch `sudo dmesg -w` |
| Many `dropped frs`, `mean_track_ms` > 33 | Drive slower. Compare with a `CPU_ORB=1` run. Stage 1, section 5 |
| Terminal doesn't echo after a crash | Type `reset` and press Enter |
| Anything else | Stage 1, section 5; or send the last 30 lines of the relevant log |
