# Stage 1 runbook: exact steps on the Jetson

The step-by-step companion to [stage1.md](stage1.md), which explains the *why*. Work through it
top to bottom. Every step ends with a **✅ Check**. Don't move on until it passes. At each
**📋 Send** checkpoint, paste the output back for review.

- **Where commands run:** 🖥️ **PC** means your Fedora PC, in `~/Desktop/code/Jetson-ORB-SLAM3-Hardware`.
  🤖 **Jetson** means the Jetson, **over SSH**. No monitor is used. Programs with windows run on
  the Jetson's **virtual screen** (`DISPLAY=:1`), which you watch from the PC through VNC
  (**Part V**). Start Part V before any step that opens a window.
- **User and paths on the Jetson:** user `orb-slam3`; the repo at `~/Jetson-ORB-SLAM3-Hardware`.
- Replace `<jetson-ip>` with the Jetson's address (`hostname -I` on the Jetson). Ours is
  `192.168.1.5`.
- **What actually happened at each step**, with the real outputs:
  [progress_log.md](progress_log.md).
- **Scope right now: Stage 1 only.** Stage 2 (gimbal) and Stage 3 (LiDAR) aren't planned yet, so
  Milestone 2 needs only route A (Part I).

| Part | Content | Time | Status |
|---|---|---|---|
| A | Jetson housekeeping | 20 min | ✅ done |
| B | Get our new files onto the Jetson with git | 10 min | ✅ done |
| C | EuRoC check with the prebuilt binary (downloads MH01) | 30 min + download | ✅ C1 done: ATE 3.72 cm. C2 skipped (optional) |
| D | Build from source with RealSense support | 2–3 h (mostly waiting) | ✅ done: ATE 4.06 cm. librealsense and Pangolin were already installed |
| E | Our D435's calibration file | 15 min | ✅ done: commit `d1821b9` |
| V | Virtual screen over VNC: **our only display** | 15 min | ✅ done: the driver's windows and live camera image show in VNC, and recording the VNC window works |
| F | Live handheld run (original driver): **Milestone 1** | 30 min | ⏳ |
| G | Rover driver bench tests: only if Milestone 2 should produce saved trajectories and numbers | 30 min | optional |
| H | Mount, power, first rover drive | ½–1 day | |
| I | Route A closed-loop runs: **Milestone 2** | ½ day | |

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
- `ATE RMSE:     X.XX cm`, **a few centimetres** (roughly 2–5). For reference, the README's example run gave 2.14 cm, and the ORB-SLAM3 paper reports 3.6 cm for stereo-inertial on MH01. Repeat runs differ by up to about a centimetre, because ORB-SLAM3 is multi-threaded. Our runs: 3.72 cm (prebuilt) and 4.06 cm (source build), each in one map.
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

**Ours:** already installed, **v2.55.1**, with its udev rules present
(`99-realsense-libusb.rules`). D2b and D2c were skipped.

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

**Ours:** `Intel RealSense D435`, serial `827312071682`, firmware 5.17.3.10, USB 3.2 and `5000M`.
Connected with a **USB 3 C-to-C cable**; the dev kit's USB-C port supports host mode (NVIDIA's
hardware guide). A USB-C cable must be USB 3 data-rated: many C-to-C cables are USB 2 or
charge-only.

**D2e. Live image and firmware** (on the virtual screen: Part V running, VNC open on the PC):
```bash
DISPLAY=:1 realsense-viewer
```
1. Turn on **Stereo Module**. Enable **Infrared 1** and **Infrared 2**, set them to
   **640×480, 30 fps**, and check both images move live.
2. **Firmware:** compare `Firmware Version` with `Recommended Firmware Version` in D2d's output.
   - **Older than recommended:** accept the viewer's update offer. **Don't unplug during the
     update.** Replug afterwards and repeat D2d.
   - **Same or newer:** keep it, and **decline** any offer, which would be a downgrade. Our camera
     has 5.17.3.10 against a recommended 5.16.0.1 (librealsense 2.55.1); newer D400 firmware
     works with older librealsense. Revisit only if streams misbehave.

**✅ Check:** both IR images are live at 30 fps, and the firmware is the recommended version or
newer.

If `realsense-viewer` isn't installed, skip D2e. Step E1 streams both IR cameras with the rover
driver over SSH, and checks the same thing.

### D3. Pangolin v0.6 🤖
```bash
ls /usr/local/lib/cmake/Pangolin/PangolinConfig.cmake 2>/dev/null && echo "ALREADY INSTALLED"
```
**Ours:** already installed. The repo's CMake found it in D4 without any build.

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
cd ~/Jetson-ORB-SLAM3-Hardware      # the paths below are relative to the repo
ls -l lib/libORB_SLAM3.so Examples/Stereo/stereo_realsense_D435i Examples/Stereo/stereo_realsense_D435_rover
grep -n "warning" ~/euroc_src.log | grep -i rover || echo "no warnings from the rover driver"
```
- `==> Building -- first time only ...` is present, and `Using the prebuilt` is **absent**.
- `GPU ORB enabled`, and an ATE **within about 1 cm of the C1 result**. This shows our source build behaves like the authors' prebuilt binary. Ours: 4.06 cm, against 3.72 cm.
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

**Ours:** fx = fy = 385.4061, cx = 318.2860, cy = 238.9506, baseline 0.0499 m, USB 3.2, no
warning. The run also showed `1 dropped frs` after almost every frame: tracking takes longer than
the camera's 33 ms, so it runs at **≈15 fps effective**. That's fine at slow speeds.

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

## Part V: virtual screen over VNC (our display)

**We don't use a monitor.** The original driver always opens its viewer, and its **Stop** button
is the only way to end it, so it needs a display. Instead, the Jetson gets a **virtual screen**
that we view from the PC. **Start it (V2–V4) at the beginning of every session** that opens a
window: Parts D2e, F, G1, H1 and the visual-only runs in H and I.

How it works:
- **Xvfb** creates virtual display `:1`. It was already installed in A4; it's the same tool
  `run_euroc.sh` uses over SSH.
- **openbox** is a tiny window manager, so windows can be moved.
- **x11vnc** shares display `:1`, on port **5910**, reachable **only** from the Jetson itself.
- An **SSH tunnel** carries it to the PC, and **TigerVNC** shows it.

The viewer is drawn in software (a virtual screen has no GPU). That costs some CPU, so watch the
`dropped frs` count.

### V1. Install (once)
```bash
# 🤖 Jetson
sudo apt install -y x11vnc openbox
# 🖥️ PC (Fedora)
sudo dnf install -y tigervnc
```

### V2. Clean up anything old
```bash
# 🤖 Jetson
pkill x11vnc; pkill openbox; pkill Xvfb
sleep 1; pgrep -a "Xvfb|x11vnc|openbox" || echo "all stopped"
# 🖥️ PC: close any VNC viewer windows first
pkill -f "ssh .*-L 59"
```

### V3. Start the virtual screen 🤖
In an SSH session, inside tmux, so it survives an SSH drop:
```bash
tmux new -s vscreen
Xvfb :1 -screen 0 1920x1080x24 &
sleep 2
DISPLAY=:1 openbox &
x11vnc -display :1 -localhost -rfbport 5910 -nopw -forever -bg -noxdamage -o ~/x11vnc.log
ss -ltn | grep 5910
```
**✅ Check:** the last line shows `LISTEN ... 127.0.0.1:5910`. The Openbox message about a missing
menu file is harmless. Detach from tmux with **Ctrl-b, then d**.

### V4. Connect from the PC 🖥️
Use **two separate terminal windows**. Don't paste both commands into one: once `ssh` starts, the
rest isn't run on the PC.

**Terminal 1**, the tunnel. After the password it shows nothing; leave it open:
```bash
ssh -N -L 5910:127.0.0.1:5910 orb-slam3@<jetson-ip>
```
**Terminal 2**, the viewer. The **double colon** means "port" to TigerVNC:
```bash
vncviewer 127.0.0.1::5910
```
**✅ Check:** a window with a plain **black or grey screen** opens. That's the empty virtual
display, so it's correct. If it warns that the connection is unencrypted, accept: the SSH tunnel
already encrypts it.

### V5. Run programs on the virtual screen
Prefix the command with `DISPLAY=:1` (Part F shows it). Windows appear in the VNC viewer about
10 s later, while the vocabulary loads; drag them apart. If a window looks frozen or black, press
**Left Alt three times** in the VNC window to repaint.

### V6. Shut down afterwards
```bash
pkill x11vnc; pkill openbox; pkill Xvfb      # 🤖 Jetson
```
On the PC, close the viewer and press Ctrl-C in the tunnel terminal. **Next time: V2–V4 again.**

**Why these choices** (problems we actually hit):
- **Port 5910, not 5900:** something else on the Jetson already listens on 5900 over IPv6.
- **The tunnel targets `127.0.0.1`, not `localhost`:** `localhost` may resolve to that other IPv6
  listener.
- **`pkill -f "ssh .*-L 59"` first:** an old tunnel on the PC held port 5901.

---

## Part F: live handheld run with the original driver (**Milestone 1**, Step 4)

**Before starting:** the virtual screen is running and the VNC viewer is open on the PC (Part V,
V2–V4). The original driver's windows appear there; its **Stop** button, clicked in VNC, is the
only way to end it.

### F1. Prepare
- The D435 is plugged directly into the Jetson; D2d passes (USB 3.x).
- A room with texture: furniture, shelves, posters. Normal lighting, no direct sun into the
  camera.
- Tape an **X** on the floor as the start mark, with an arrow for the heading.
- Start a screen recording of the **VNC window on the PC**. Fedora's GNOME recorder works
  (Ctrl+Shift+Alt+R). The recording stays on the PC and is the Milestone 1 evidence.
- The camera cable must reach while you walk a small loop around the room. The Jetson stays on
  the desk.

### F2. Run 🤖 (over SSH)
```bash
mkdir -p ~/evidence/m1 && cd ~/Jetson-ORB-SLAM3-Hardware
tegrastats --interval 1000 > ~/evidence/m1/tegrastats.log &
DISPLAY=:1 ./Examples/Stereo/stereo_realsense_D435i Vocabulary/ORBvoc.txt \
    Examples/Stereo/RealSense_D435.yaml 2>&1 | tee ~/evidence/m1/live_m1.log
```
About 10 s later, two windows appear in VNC: **"ORB-SLAM3: Current Frame"** (the IR image with
green features) and **"ORB-SLAM3: Map Viewer"** (3D). Drag them apart so both are visible in the
recording.

### F3. Walk
1. Stand on the X, facing the arrow. Tracking starts immediately; stereo needs no special motion.
2. Walk **slowly** (≤ 0.5 m/s), turning gently, with the camera level and facing forward. Go
   round a loop of ~10–20 m. Tracking runs at about 15 fps (every other frame is skipped, see
   E1), so slow, smooth movement matters.
3. Hold the camera **with both hands**, steady and level. Turn **very slowly**: most tracking
   losses happen in turns. Keep the camera on textured things, never close to a plain wall.
4. Come back to the X facing the arrow, and **don't stop there: carry on slowly along the first
   2–3 m of your original route**, the same way you started. `*Loop detected` should appear during
   this part.
   - **Why:** ORB-SLAM3 confirms a loop only after recognising the place in **3 consecutive new
     keyframes** ([src/LoopClosing.cc:444](../src/LoopClosing.cc#L444)), and new keyframes are
     only made while the camera moves. Standing still at the X gives it at most one chance.
5. Stop walking, and click **Stop** in the Map Viewer menu, in the VNC window. Ctrl-C does *not*
   stop this driver.
6. Stop tegrastats with `kill %1` in the SSH session, and stop the screen recording on the PC.

### F4. Verify 🤖
```bash
grep -cE "\*Loop detected" ~/evidence/m1/live_m1.log            # >= 1
grep -cE "Stored map with ID" ~/evidence/m1/live_m1.log          # ideally 0
grep -E "GPU ORB enabled" ~/evidence/m1/live_m1.log | head -1
grep -c "dropped frs" ~/evidence/m1/live_m1.log                  # ~1 per processed frame at ~15 fps
grep -c "PR: Loop detected with Reffine Sim3" ~/evidence/m1/live_m1.log   # partial recognitions (need 3 in a row)
cp ~/euroc_gpu.log ~/euroc_src.log ~/evidence/m1/ ; ls ~/evidence/m1
```
**✅ Check (Milestone 1 pass):**
- **`*Loop detected` ≥ 1**, and in the recording the trajectory visibly snaps into place when you
  returned;
- `GPU ORB enabled`;
- the map looks metric: a 1 m wide table spans about 1 m of points;
- `Stored map with ID` is 0, or rare;
- few `dropped frs`.

**If no loop closes:**
- **`Stored map with ID` ≥ 1:** tracking was lost, and everything after is a new map. Find
  where: `grep -n "New Map created\|Stored map" ~/evidence/m1/live_m1.log`. Watch the recording
  at that moment (a turn? a plain wall?) and redo the walk slower there.
- **No `Stored map`, but no loop:** you probably didn't walk far enough over the start area. A
  non-zero `PR: Loop detected` count means it was recognising the place. Continue further along
  the start of the route next time.
- In any case, check the room has texture along the whole loop.

**Our attempt 1** (2026-09-27) failed this way. Tracking held for most of the ~60 s walk and was
lost near the end, during the return to the X: `Stored map with ID: 0` came at log line 1289 of
~1330. The walk ended standing on the X (the old instruction), so no loop could be confirmed.
Processing was a steady ≈15 fps (1290 × "1 dropped", 29 × "2 dropped"), so the virtual screen
isn't slowing it.

**📋 Send:** the F4 output. Milestone 1 is then complete. Keep the screen recording on the PC,
e.g. in `~/Desktop/jetson-orbslam3-results/m1/`.

---

## Part G: rover driver bench tests (before any mounting)

**Optional.** You only need Part G if Milestone 2 should produce **saved trajectories and numbers**
(loop closures, end-point error, tracking time). For a visual-only Milestone 2 (the original
driver over VNC plus a screen recording, see H3), skip it.

These tests check that the rover driver is trustworthy: it exits cleanly and saves correct files
every time.

### G1. Handheld with the viewer 🤖 (virtual screen running, VNC open)
```bash
cd ~/Jetson-ORB-SLAM3-Hardware
DISPLAY=:1 tools/rover_run.sh bench1
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
  - note `mean_track_ms`. **Below ~33 would be full real time at 30 fps.** We expect roughly
    50–75 ms, because tracking skips every other frame (≈15 fps, see E1). That's acceptable at
    slow speeds; Stage 1, section 5 has options if tracking suffers.

### G2. Headless over SSH 🤖
From an SSH session, with no viewer at all:
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
- [ ] **Field of view check**, rover on the floor: `DISPLAY=:1 realsense-viewer` (watched in VNC),
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

Choose one of two ways to run on the rover:

| | **Visual only** | **With measurements** |
|---|---|---|
| Program | The original `stereo_realsense_D435i` | `tools/rover_run.sh` (rover driver) |
| How to view and stop | Part V's virtual screen over Wi-Fi; click **Stop** in VNC | SSH only (`--no-viewer`); press **q** |
| Evidence | A screen recording of the loop closing | The recording (optional), plus saved trajectories and a summary line with numbers |
| Needs Part G | no | yes |

With the visual-only way, run the H4 and I2 drives with the F2 command instead of
`tools/rover_run.sh`, and record the VNC window.

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
- note `mean_track_ms` (≈50–75 ms expected, see G1);
- `grep -c "dropped frs" $(ls -d ~/runs/*_smoke1_line | tail -1)/console.log` is small.

If tracking is lost while driving, go slower first. Then check for vibration: if the image is
blurry in `DISPLAY=:1 realsense-viewer` (in VNC) while driving, improve the dampers.

**📋 Send:** the smoke run's `summary.txt`.

---

## Part I: route A runs (**Milestone 2**, 4.6)

Only **route A** (closed loop) is needed now: it shows the repo working on the rover. Routes B
(out-and-back) and C (blank wall) exist to be compared against Stage 2's gimbal features. Record
them **only if Stage 2 is started**; they're described in [stage1.md, 4.6](stage1.md#46-baseline-drive-tests).

### I1. Prepare route A (once)
- **Start mark:** an X plus a heading arrow, taped on the floor.
- **Route A:** a ~30–50 m closed loop around rooms or corridors that comes back over the X
  facing the arrow, and **continues 2–3 m along the start of the route**, where the run ends.
- Write it down, with a sketch, so every run follows the same path.

### I2. Drive route A 3 times 🤖
The rules for every run:
- speed ≤ 0.5 m/s, turns ≤ 30°/s;
- the same operator, time of day and lighting;
- at the end, drive **past the X and continue 2–3 m along the start of the route**, in the original direction, then stop and end the run. The end-point error is measured against the first pass over that stretch
  (4.5 in stage1.md);
- turn slowly: tracking losses mostly happen in turns.

**With measurements** (rover driver):
```bash
cd ~/Jetson-ORB-SLAM3-Hardware
tools/rover_run.sh A1_base --no-viewer      # then A2_base, A3_base; press q at the end
```
**Visual only** (original driver over VNC, see H3): first run `mkdir -p ~/evidence/m2`. Then use
the F2 command with `tee ~/evidence/m2/live_A1.log` (and A2, A3), recording the VNC window on the
PC. End each run with **Stop** in VNC.

**✅ Check after each run:** with measurements, the summary line was printed and the folder has
all 8 files; visual only, the log and recording exist. If a run was disturbed (someone walked in
front, you drove off the route), **repeat it** and delete the bad run.

### I3. Collect the results 🤖
**With measurements:**
```bash
python3 tools/summarize_run.py ~/runs/*_A?_base | tee ~/runs/route_A.txt
```
**Visual only:**
```bash
grep -c "\*Loop detected" ~/evidence/m2/live_A*.log; grep -c "Stored map with ID" ~/evidence/m2/live_A*.log
```
**✅ Check (Milestone 2 pass): all three A runs** close the loop (`loops ≥ 1` / `*Loop detected`),
and don't end split (`split=False` / no unmerged `Stored map` lines). With measurements, also an
`endpoint_pct` of a few % or less.

**📋 Send:** `~/runs/route_A.txt`, or the visual-only `grep` output. **Stage 1 is then complete.**

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
| `bind ... Address already in use` when starting the tunnel | An old tunnel holds the port: `pkill -f "ssh .*-L 59"` on the PC (V2) |
| VNC viewer doesn't open, or opens but stays empty | The tunnel and viewer must be in **separate** terminals (V4). Check `ss -ltn \| grep 5910` on the Jetson (V3). Use `vncviewer 127.0.0.1::5910` (double colon) |
| VNC shows a black screen after the driver starts | Press Left Alt three times in the VNC window; check the driver was started with `DISPLAY=:1` |
| `x11vnc`: `listen6: bind: Address already in use` | Something else holds 5900 on IPv6; that's why Part V uses port 5910 |
| Anything else | Stage 1, section 5; or send the last 30 lines of the relevant log |
