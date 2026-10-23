# Husky stage 1 runbook: our Jetson ORB-SLAM3 (visual only) vs the fused ground truth

**Goal:** run our camera-only pipeline (stereo IR: GPU / CPU / pipe) **on the Jetson**, on the other
team's Husky recording, and generate a **visual SLAM vs ground truth** report.

**The work is split between the two machines:**

| Where | What | Status |
|---|---|---|
| **PC** | IR stereo extracted from the bag (`~/datasets/ext/husky_20261023/`), and the ground truth from every non-camera sensor: a GTSAM fusion of lidar odometry, loop closures, the IMU and the wheels (`processing/fuse_reference.py`) | **Done.** The ground truth is in the repository: [`jetson-orbslam3-results-husky/reference/`](../jetson-orbslam3-results-husky/reference/README.md) |
| **Jetson** | Copy the extracted sequence from the PC, run ORB-SLAM3, write the report | This runbook |

The ground truth: `jetson-orbslam3-results-husky/reference/sequence/gt.csv`.
- **What it holds:** the left IR camera's pose at 5324 of the 5326 image times. It was built from the
  lidar, IMU and wheels (corrected for a 180° lidar-yaw error in their TF, and a 4.4% wheel scale
  error). The camera wasn't used, so the reference is independent of what it scores.
- **Uncertainty:** about 10–25 cm. The details are in the reference README.

The ablation studies come later ([ablation_plan_husky.md](ablation_plan_husky.md)).

Conventions:
- The PC repository is `~/Desktop/code/Jetson-ORB-SLAM3-Hardware`. The Jetson's is
  `~/Jetson-ORB-SLAM3-Hardware` (user `orb-slam3`, IP `172.31.53.140`).
- The Jetson never needs the bag. It only needs the extracted sequence (step 2).

---

## 0. PC: commit and push

```bash
cd ~/Desktop/code/Jetson-ORB-SLAM3-Hardware
git add processing jetson-orbslam3-results-husky repro/common.py repro/run_matrix.py \
        plan/husky_stage1_runbook.md plan/evaluation_plan_stage_1.md plan/ablation_plan_husky.md
git commit -m "Husky evaluation: fused ground truth (PC), ext suite, Jetson prepare + report"
git push
```
The commit holds about 7 MB: the reference, plus the code that builds and evaluates it. It contains
no images and no bag.

## 1. Jetson: code (once)

```bash
cd ~/Jetson-ORB-SLAM3-Hardware && git pull
ls jetson-orbslam3-results-husky/reference/sequence/gt.csv        # the ground truth arrived
df -h ~                                                           # ~2 GB needed for the IR images
```
The Python environment for the report is set up later, in step 4, so it doesn't compete with the runs.

## 2. Jetson: copy the IR stereo sequence from the PC

The PC already extracted the IR stereo from the bag into `~/datasets/ext/husky_20261023/` (1.3 GB):
images, `times.txt`, `settings.yaml` and `gt.csv`. Copy that folder as it is. Run this on the
**Jetson**, in tmux:
```bash
tmux new -s xfer
mkdir -p ~/datasets/ext
rsync -a --partial --info=progress2 shiv@172.31.2.87:datasets/ext/husky_20261023 ~/datasets/ext/
# PC password once, then Ctrl-b d; tmux attach -t xfer to check. If it drops, run it again.
```

Check that the ground truth in it matches the repository's:
```bash
cd ~/Jetson-ORB-SLAM3-Hardware
cmp ~/datasets/ext/husky_20261023/gt.csv jetson-orbslam3-results-husky/reference/sequence/gt.csv && echo "gt.csv ok"
```
It must print `gt.csv ok`.

## 3. Jetson: run visual SLAM (in tmux, ~1.5–2 h + ~20 min timing)

```bash
sudo nvpmodel -m 2 && sudo jetson_clocks          # MAXN_SUPER (0 = 15W, 1 = 25W on this board)
sudo nvpmodel -q | tee ~/husky_power_mode.txt
tmux new -s husky
cd ~/Jetson-ORB-SLAM3-Hardware
python3 repro/run_matrix.py ext --dry-run          # 15 runs: gpu, cpu, gpu_pipe × 5
python3 repro/run_matrix.py ext --engine /nonexistent --results ~/repro_results_ext 2>&1 | tee ~/husky_matrix.log
# Ctrl-b d to detach; tmux attach -t husky to look
```
- This is **visual only**: `Examples/Stereo/stereo_euroc` on the IR pair, with no IMU.
  `--engine /nonexistent` keeps loop closure DBoW2-only, which is our original pipeline.
- The replay is paced to the camera (15 FPS, 355 s per run, plus about 10 s of vocabulary loading).
- Keep the Jetson otherwise idle. Re-running the command skips finished runs.
- In the dry run, `settings:default` means "the sequence's own `settings.yaml`" (visible in the
  command line).

**Then the per-stage timing runs (~20 min, 3 runs).** Because the replay is paced, wall time can't
show how much faster than real time an arm is. The instrumented build (`~/Jetson-ORB-SLAM3-timing`,
built earlier with `repro/setup_timing_build.sh`) records the per-stage cost of every frame:
```bash
python3 repro/run_matrix.py ext --runs 1 --engine /nonexistent \
    --build-root ~/Jetson-ORB-SLAM3-timing --results ~/repro_results_ext_timing
ls ~/repro_results_ext_timing/ext/ext_s/husky_20261023/*/run1/ExecMean.txt   # 3 files: gpu, cpu, gpu_pipe
```
If one `ExecMean.txt` is missing, that arm shows "–" in the report's per-stage table.

## 4. Jetson: score and write the report (~5 min the first time, then ~1 min)

**After all runs have finished**, set up the light Python environment once. pip would compete with
ORB-SLAM3 for CPU if it ran during the runs.
```bash
cd ~/Jetson-ORB-SLAM3-Hardware
git pull                                # the latest runbook and report script
sudo apt install -y python3.10-venv     # once: Ubuntu's python3 can't create venvs without it
processing/setup_venv.sh --light        # must end with "ok" for rosbags, numpy, scipy, cv2, matplotlib
```
`--light` installs only `rosbags`, `numpy`, `scipy`, `opencv-python-headless` and `matplotlib` into
`~/venvs/ext`. The fusion stack (kiss-icp, gtsam) isn't needed on the Jetson. The system Python is
untouched: `run_matrix.py` and `evaluate.py` keep using it.

Then score the runs and write the report. `evaluate.py` takes ~1 min per run on the Jetson (it
scores 5,300 poses in pure Python): **let it finish**. It ends with `scored N run(s)`. If it is
interrupted, re-running it skips runs that are already scored.
```bash
python3 repro/evaluate.py ~/repro_results_ext
python3 repro/evaluate.py ~/repro_results_ext_timing
~/venvs/ext/bin/python processing/husky_report.py \
    --results ~/repro_results_ext --timing ~/repro_results_ext_timing \
    --seq ~/datasets/ext/husky_20261023 \
    --reference jetson-orbslam3-results-husky/reference \
    --out ~/husky_report
```
`~/husky_report/HUSKY_REPORT.md` contains:

| Section | Contents |
|---|---|
| 1. Data and reference | the sequence, how the ground truth was built, the inputs it excludes, its uncertainty |
| 2. Accuracy per arm (median, min–max over 5 runs) | **ATE SE(3), keyframes** (the paper's Table 1 metric); ATE for all frames; **horizontal ATE**; height error; Sim(3) ATE and scale; max error; RPE over 1 / 5 / 10 m; coverage; time to first pose; loops; maps; tracking-loss events. Also the robot's own wheel and EKF odometry scored the same way, for context. |
| 3. GPU ≡ CPU | Δ ATE against the run-to-run spread and the reference uncertainty; the reference-free trajectory-distance check (as in R6) |
| 4. Compute | power, energy per frame, CPU and GPU load, RAM, Tj, real-time yes/no; **per-stage ms** (ORB extraction, stereo matching, local-map tracking, total tracking, local mapping) and true FPS from the timing runs |
| 5. Run health | the outcome of every run (clean, teardown crash, NaN abort) |
| 6. Plots | top view against the reference; error over time; ATE per arm |

## 5. Bring the results to the PC

```bash
# on the PC
cd ~/Desktop/code/Jetson-ORB-SLAM3-Hardware
rsync -a orb-slam3@172.31.53.140:husky_report :repro_results_ext :repro_results_ext_timing \
    :husky_matrix.log :husky_power_mode.txt jetson-orbslam3-results-husky/
```
Each path is listed separately (`:path` = same host). rsync ≥ 3.2.4 treats a quoted, space-separated
list as one filename. If `husky_power_mode.txt` doesn't exist, rsync reports that file and copies
the rest.

After that, `jetson-orbslam3-results-husky/` holds the reference, the raw runs and the report together.

---

## Checklist
- [ ] 0 pushed · 1 pulled, `gt.csv` present in the repository
- [ ] 2 sequence copied; `gt.csv ok`
- [ ] 3 15 visual-SLAM runs done; 3 timing runs done (3 × `ExecMean.txt`)
- [ ] 4 light venv OK (after the runs); `HUSKY_REPORT.md` generated · 5 results on the PC

## Troubleshooting
| Symptom | Fix |
|---|---|
| run_matrix: `nothing to run` | `~/datasets/ext/husky_20261023/` is missing or incomplete. Re-run the step 2 rsync. |
| `cannot start: missing program …stereo_euroc` | Build the repository (Stage 1, Part D). |
| `ensurepip is not available` / report: `No module named 'numpy'` | `sudo apt install -y python3.10-venv`, `rm -rf ~/venvs/ext`, re-run `processing/setup_venv.sh --light` |
| Report: no accuracy for some runs | `gt.csv` was missing when `evaluate.py` ran. Copy it from `jetson-orbslam3-results-husky/reference/sequence/` into `~/datasets/ext/husky_20261023/`, then `python3 repro/evaluate.py ~/repro_results_ext --force`. |
| A run aborts with `Sophus SO3::exp` | The known ORB-SLAM3 NaN bug. The report counts it as a failed run. Don't re-run to hide it. |
| `Segmentation fault` / `Bus error` at the end of a run | The known teardown crash, after the trajectories are written. It's harmless. |

## PC preparation: what was done to build the dataset and the ground truth (complete)

This all ran **on the PC**, once, before the Jetson steps above. It is recorded here so it can be
repeated or audited. Its outputs are the sequence copied in step 2 and the reference in the repository.

### P1. The bag
- Downloaded from the other team's Drive to `~/Downloads/`: `husky_20261023_062755_0.db3` (20.7 GB)
  and `metadata.yaml`. SHA-256 `2877dab42281c4a5c4fa1205f5c3dddca66236801865f7adbea54e373f851bcd`.
- Integrity: SQLite `quick_check` ok, and all 218 073 messages are present.
- Linked read-only (symlinks, no copy) into the folder layout the scripts expect:
  ```bash
  mkdir -p ~/datasets/external_raw/husky_20261023
  ln -s ~/Downloads/husky_20261023_062755_0.db3 ~/Downloads/metadata.yaml ~/datasets/external_raw/husky_20261023/
  ```

### P2. Python environment (full set)
```bash
PYTHON=python3.12 processing/setup_venv.sh     # ~/venvs/ext: rosbags numpy scipy opencv matplotlib + kiss-icp gtsam kiss-slam
```
Python 3.12, because kiss-icp has no wheel for the PC's default Python 3.14.

### P3. Dataset and ground truth (`processing/prepare_husky.sh`, ~3 min)
```bash
processing/prepare_husky.sh                    # bag ~/datasets/external_raw/husky_20261023 -> see below
```
| Step | Script | Output |
|---|---|---|
| 1 inventory | `inspect_bag.py` | `~/datasets/ext_work/husky_20261023/inventory.txt`, sample IR pairs |
| 2 **IR stereo → EuRoC layout** | `bag_to_euroc.py` | `~/datasets/ext/husky_20261023/`: 5326 left + 5326 right PNG, `times.txt`, `settings.yaml` (their calibration with our R6 ORB settings), `calib.json`, `tf_static.json` |
| 3 robot odometry (validation) | `bag_odom.py` | `…/ext_work/…/odom/` |
| 4 **fused ground truth** | `fuse_reference.py` | `…/ext_work/…/fused/` (`fused_poses.txt`, reports, plots) |
| 5 sensitivity: the fusion without the IMU / wheels / loops | `fuse_reference.py --no-…` | `…/fused_no-imu/`, `…/fused_no-wheel/`, `…/fused_no-loops/` |
| 6 lidar-only cross-checks | `lidar_odometry.py`, `lidar_slam.py`, `reference_check.py` | `…/kiss_icp/`, `…/kiss_slam/`, `…/fused/reference_check.md` |
| 7 **`gt.csv`** | `make_gt.py` | `~/datasets/ext/husky_20261023/gt.csv`: the reference as left-IR-camera poses at every image time |

What `fuse_reference.py` found and fixed in the dataset (it checks every one automatically):
1. **The lidar yaw in their `/tf_static` is off by 179.6°.** It is measured from the data (lidar vs
   wheel direction of travel) and corrected.
2. **The IMU driver's orientation quaternion is unusable** (84.5° away from gravity). It is excluded;
   the raw gyro and accelerometer are used.
3. **The wheels over-report distance by 4.4%.** The wheel scale (0.956) is calibrated against the lidar.
4. **The magnetometer is distorted** (\|B\| varies 28%). It is excluded.
5. **The camera mounting in the TF is correct.** It was checked against the visual motion direction
   in the IR images. That check reads images for verification only; nothing from the camera enters
   the ground truth.

Result:
- a 65.6 m loop, with two lidar loop closures;
- removing any one input changes the reference by only 3–11 cm;
- uncertainty about 10–25 cm.

The details are in [`jetson-orbslam3-results-husky/reference/README.md`](../jetson-orbslam3-results-husky/reference/README.md).

### P4. Into the repository
The small outputs (no images) were copied into `jetson-orbslam3-results-husky/reference/` (7 MB),
then committed and pushed in step 0:
```bash
O=jetson-orbslam3-results-husky/reference S=~/datasets/ext/husky_20261023 W=~/datasets/ext_work/husky_20261023
mkdir -p $O/sequence $O/fusion $O/crosschecks
cp $S/{gt.csv,times.txt,settings.yaml,calib.json,tf_static.json,source_info.txt,reference_info.json} $O/sequence/
cp $W/fused/* $O/fusion/ && cp $W/inventory.txt $O/ && cp ~/datasets/ext/husky_20261023.prepare.log $O/prepare.log
for d in fused_no-imu fused_no-wheel fused_no-loops kiss_icp kiss_slam odom; do
  mkdir -p $O/crosschecks/$d; cp $W/$d/*.txt $W/$d/*.json $O/crosschecks/$d/ 2>/dev/null; done
```
The image folder `~/datasets/ext/husky_20261023/` (1.3 GB, with `gt.csv` installed in it) stays on
the PC. That is what step 2 copies to the Jetson.

**To rebuild from scratch:** delete `~/datasets/ext/husky_20261023` and
`~/datasets/ext_work/husky_20261023`, re-run P3 and P4, commit, and repeat steps 0–2.
