# Husky stage 1 runbook: our Jetson ORB-SLAM3 (visual only) vs the fused ground truth

**Goal:** run our camera-only pipeline (stereo IR: GPU / CPU / pipe) **on the Jetson**, on the other
team's raw Husky bag, and generate a **visual SLAM vs ground truth** report.

**The work is split between the two machines:**

| Where | What | Status |
|---|---|---|
| **PC** | Ground truth from every non-camera sensor: a GTSAM fusion of lidar odometry, loop closures, the IMU and the wheels (`processing/fuse_reference.py`) | **Done.** It's in the repository: [`jetson-orbslam3-results-husky/reference/`](../jetson-orbslam3-results-husky/reference/README.md) |
| **Jetson** | Extract the IR images from the raw bag, install the ground truth, run ORB-SLAM3, write the report | This runbook |

The ground truth: `jetson-orbslam3-results-husky/reference/sequence/gt.csv`.
- **What it holds:** the left IR camera's pose at 5324 of the 5326 image times. It was built from the
  lidar, IMU and wheels (corrected for a 180° lidar-yaw error in their TF, and a 4.4% wheel scale
  error). The camera wasn't used, so the reference is independent of what it scores.
- **Uncertainty:** about 10–25 cm. The details are in the reference README.

The ablation studies come later ([ablation_plan_husky.md](ablation_plan_husky.md)).

Conventions:
- The PC repository is `~/Desktop/code/Jetson-ORB-SLAM3-Hardware`. The Jetson's is
  `~/Jetson-ORB-SLAM3-Hardware` (user `orb-slam3`, IP `172.31.53.140`).
- The bag on the Jetson is at `~/datasets/external_raw/husky_20261023/`
  (`husky_20261023_062755_0.db3` + `metadata.yaml`, SHA-256 `2877dab4…f851bcd`).

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

## 1. Jetson: code and a light Python environment (once)

```bash
cd ~/Jetson-ORB-SLAM3-Hardware && git pull
ls jetson-orbslam3-results-husky/reference/sequence/gt.csv        # the ground truth arrived
df -h ~                                                           # ~2 GB needed for the IR images
processing/setup_venv.sh --light
```
`--light` installs only what the Jetson needs into `~/venvs/ext`: `rosbags`, `numpy`, `scipy`,
`opencv-python-headless` and `matplotlib`. It must end with `ok` for all five. The report (step 4)
needs it even if you copy the images (option A below).
- The fusion stack (kiss-icp, gtsam) is **not** needed on the Jetson.
- The system Python is untouched: `run_matrix.py` and `evaluate.py` keep using it.

## 2. Jetson: get the IR stereo and install the ground truth

The Jetson needs the IR stereo sequence in `~/datasets/ext/husky_20261023/`. There are two ways to get it.

**Option A (recommended): copy the sequence the PC already extracted** (1.3 GB, about 10.7k files).
This is the exact image set the ground truth was built for. Run on the **Jetson**, in tmux, so
closing your terminal doesn't stop it:
```bash
tmux new -s xfer
mkdir -p ~/datasets/ext
rsync -a --partial --info=progress2 shiv@172.31.2.87:datasets/ext/husky_20261023 ~/datasets/ext/
# PC password once, then Ctrl-b d; tmux attach -t xfer to check (Wi-Fi: ~5-15 min)
```
`rsync` verifies every file it copies. If the connection drops, run the same command again.

**Option B: extract from the raw bag on the Jetson.** This needs the 20.7 GB bag at
`~/datasets/external_raw/husky_20261023/` (`husky_20261023_062755_0.db3` + `metadata.yaml`,
SHA-256 `2877dab42281c4a5c4fa1205f5c3dddca66236801865f7adbea54e373f851bcd`).
`jetson_prepare.sh` extracts it automatically when the sequence folder is missing.

**Then, either way:**
```bash
cd ~/Jetson-ORB-SLAM3-Hardware
processing/jetson_prepare.sh
```
It does three things, and stops with `ERROR` if a check fails:
1. **Uses** the copied sequence, or **extracts** it from the bag if it is absent.
2. **Checks** that `times.txt` and `settings.yaml` are byte-identical to the ones the ground truth
   was built for (stored in the repository), and that there are 5326 left + 5326 right images.
3. **Installs** the repository's `gt.csv` and `reference_info.json` into the sequence folder.

It must end with `Ready.`.

## 3. Jetson: run visual SLAM (in tmux, ~1.5–2 h)

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

**Optional, recommended (~20 min): per-stage timing.** Because the replay is paced, wall time can't
show how much faster than real time an arm is. The instrumented build (`~/Jetson-ORB-SLAM3-timing`,
built earlier with `repro/setup_timing_build.sh`) can:
```bash
python3 repro/run_matrix.py ext --runs 1 --engine /nonexistent \
    --build-root ~/Jetson-ORB-SLAM3-timing --results ~/repro_results_ext_timing
```

## 4. Jetson: score and write the report (~1 min)

```bash
cd ~/Jetson-ORB-SLAM3-Hardware
python3 repro/evaluate.py ~/repro_results_ext
python3 repro/evaluate.py ~/repro_results_ext_timing                   # only if step 3's timing runs were done
~/venvs/ext/bin/python processing/husky_report.py \
    --results ~/repro_results_ext --timing ~/repro_results_ext_timing \
    --seq ~/datasets/ext/husky_20261023 \
    --reference jetson-orbslam3-results-husky/reference \
    --out ~/husky_report
```
Leave out `--timing …` if you skipped the timing runs.

`~/husky_report/HUSKY_REPORT.md` contains:

| Section | Contents |
|---|---|
| 1. Data and reference | the sequence, how the ground truth was built, the inputs it excludes, its uncertainty |
| 2. Accuracy per arm (median, min–max over 5 runs) | **ATE SE(3), keyframes** (the paper's Table 1 metric); ATE for all frames; **horizontal ATE**; height error; Sim(3) ATE and scale; max error; RPE over 1 / 5 / 10 m; coverage; time to first pose; loops; maps; tracking-loss events. Also the robot's own wheel and EKF odometry scored the same way, for context. |
| 3. GPU ≡ CPU | Δ ATE against the run-to-run spread and the reference uncertainty; the reference-free trajectory-distance check (as in R6) |
| 4. Compute | power, energy per frame, CPU and GPU load, RAM, Tj, real-time yes/no; per-stage ms if the timing runs were done |
| 5. Run health | the outcome of every run (clean, teardown crash, NaN abort) |
| 6. Plots | top view against the reference; error over time; ATE per arm |

## 5. Bring the results to the PC

```bash
# on the PC
cd ~/Desktop/code/Jetson-ORB-SLAM3-Hardware
rsync -a orb-slam3@172.31.53.140:'husky_report repro_results_ext repro_results_ext_timing husky_matrix.log husky_power_mode.txt' \
    jetson-orbslam3-results-husky/
```
After that, `jetson-orbslam3-results-husky/` holds the reference, the raw runs and the report together.

---

## Checklist
- [ ] 0 pushed · 1 pulled, `gt.csv` present, light venv OK
- [ ] 2 sequence copied (option A) or bag present (option B); `jetson_prepare.sh` ended with `Ready.`
- [ ] 3 15 visual-SLAM runs done (+ 3 timing runs)
- [ ] 4 `HUSKY_REPORT.md` generated · 5 results on the PC

## Troubleshooting
| Symptom | Fix |
|---|---|
| `run processing/setup_venv.sh --light first` | The light venv is missing a package. Re-run it and read the MISSING lines. |
| `ERROR: times.txt differs from the reference's` | The images come from a different bag or tool, or the copy is incomplete. Delete `~/datasets/ext/husky_20261023` and redo step 2 (option A: re-run the rsync). |
| `ERROR: no extracted sequence … and no bag` | Neither option A nor B was done. Copy the sequence from the PC (option A). |
| `ERROR: settings.yaml differs` | The calibration or ORB settings changed since the reference was built. Use the repository version of `processing/bag_to_euroc.py` and `Examples/Stereo/RealSense_D435.yaml`. |
| run_matrix: `nothing to run` | `~/datasets/ext/husky_20261023/` lacks `times.txt` or `settings.yaml`. Re-run step 2. |
| `cannot start: missing program …stereo_euroc` | Build the repository (Stage 1, Part D). |
| Report: no accuracy for some runs | `gt.csv` was missing when `evaluate.py` ran. Re-run step 2, then `python3 repro/evaluate.py ~/repro_results_ext --force`. |
| A run aborts with `Sophus SO3::exp` | The known ORB-SLAM3 NaN bug. The report counts it as a failed run. Don't re-run to hide it. |
| `Segmentation fault` / `Bus error` at the end of a run | The known teardown crash, after the trajectories are written. It's harmless. |

## Rebuilding the ground truth (PC only, not needed now)
```bash
PYTHON=python3.12 processing/setup_venv.sh        # full set: + kiss-icp, gtsam, kiss-slam
processing/prepare_husky.sh                       # ~3 min; then copy into jetson-orbslam3-results-husky/reference/
```
