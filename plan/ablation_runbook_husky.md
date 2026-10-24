# Ablation runbook: Husky bag, one factor at a time

**Design:** [ablation_plan_husky.md](ablation_plan_husky.md). **Prerequisite:** stage 1 is done
([husky_stage1_runbook.md](husky_stage1_runbook.md)), and its runs are on the PC in
`jetson-orbslam3-results-husky/repro_results_ext/`. They are cells B0, B1 and B2.

| Where | What | Status |
|---|---|---|
| PC | Extract IMU, depth and colour; build the 12 variant folders with their own settings and ground truth | **Done** (step P) |
| Jetson | 54 runs (~6 h) | steps 0–3 |
| PC | Score and write the report | step 4 |

The cells (B0 = stereo IR, GPU ORB, DBoW2, 15 FPS, 1250 features):

| Cell | Variant folder | Arm | Question |
|---|---|---|---|
| A1 | `__mono` | gpu | What does the second camera buy? |
| A3 | `__mi` | gpu | Does the IMU rescue mono? |
| A4 | `__si` | gpu | Does the IMU help stereo, and does it fix the 6.4% scale error? |
| A5 | `__ird` | gpu | Is the camera's depth a substitute for stereo matching? |
| A6 | `__rgbd` | gpu | Colour (rolling shutter) + depth instead of IR |
| B1 / B2 | (base) | cpu / gpu_pipe | stage 1 |
| B1i / B2i / B3 | `__si` | cpu / gpu_pipe / adaptive_fe | compute arms with the IMU (`ADAPTIVE_FE` only works stereo-inertial) |
| C1 / C1i | `__nolc` / `__si_nolc` | gpu | What does loop closure add? |
| C2 / C2i | (base) / `__si` | cnn | DBoW2 + CosPlace CNN |
| C3 | `__si` | adaptive_lc | `ADAPTIVE_LC` (inertial loops only) |
| D1 / D1i | `__7p5` / `__si_7p5` | gpu | 7.5 FPS input |
| D2 / D2i | `__f800` / `__si_f800` | gpu | 800 features |
| D3 | `__bscale` | gpu | Stereo baseline × 1.068: if this removes the scale error, it is a calibration problem |
| E1–E3 | none | none | wheels, robot EKF, lidar only (scored on the PC, no runs) |

---

## P. PC preparation (done; recorded so it can be repeated)

```bash
cd ~/Desktop/code/Jetson-ORB-SLAM3-Hardware
~/venvs/ext/bin/python -I processing/ablation_prepare.py      # ~5 min; needs the bag, the stage-1 runs and the reference
```
What it did:
- **Wrote into the base sequence `~/datasets/ext/husky_20261023/`:**
  - `mav0/imu0/data.csv`: the Husky IMU at 83.4 Hz, with the **camera-IMU time offset (−24.0 ms,
    correlation 0.97) applied**. The offset was measured from gyro vs stage-1 camera rotation.
  - `depth/`, `color/` and `depth_c/`: 5324 images each, about 3 GB in total.
- **Built the 12 variant folders `~/datasets/ext/husky_20261023__<variant>/`.** Each holds:
  - symlinks to the base data;
  - its own `settings.yaml`, `times.txt` and `mode.txt`;
  - its own `gt.csv` in the frame that variant's program reports. That's the camera for stereo,
    mono and IR + depth, the **IMU body** for the inertial variants, and the **colour camera** for A6.
- **The IMU calibration it used** is in `ablation_prepare.json`:
  - `T_imu_cam` comes from `/tf_static`, whose IMU axes were already verified (yaw-rate correlation 0.99);
  - noise is measured at rest and ×5: gyro 6.4e-4, acc 3.1e-3; walks 1e-5 / 1e-3.

## 0. PC: commit and push

```bash
cd ~/Desktop/code/Jetson-ORB-SLAM3-Hardware
git add processing repro/common.py repro/run_matrix.py repro/evaluate.py plan/ablation_runbook_husky.md
git commit -m "Ablation: variants (PC), abl suite + new modes/arms, ablation report"
git push
```

## 1. Jetson: code and data (~10–20 min over Wi-Fi)

```bash
cd ~/Jetson-ORB-SLAM3-Hardware && git pull
df -h ~                                   # ~4 GB more needed
tmux new -s xfer
rsync -a --partial --info=progress2 shiv@172.31.2.87:datasets/ext/ ~/datasets/ext/
# PC password once, then Ctrl-b d. If it drops, run it again: only missing files are copied.
```
This copies only what's new: `imu0/`, `depth/`, `color/`, `depth_c/` and the 12 variant folders.
Their symlinks are copied as symlinks.

Check:
```bash
ls -d ~/datasets/ext/husky_20261023__*            # 12 folders
ls ~/datasets/ext/husky_20261023/mav0/imu0/data.csv
python3 repro/run_matrix.py abl --seqs husky_20261023__si husky_20261023__ird --runs 1 --dry-run | grep -E "inertial|rgbd_tum"
```
The dry run must show `stereo_inertial_euroc` for `__si` and `rgbd_tum` for `__ird`.

## 2. Jetson: the runs (in tmux, ~6 h, 54 runs)

```bash
sudo nvpmodel -m 2 && sudo jetson_clocks          # MAXN_SUPER (0 = 15W, 1 = 25W on this board)
sudo nvpmodel -q | tee ~/abl_power_mode.txt
tmux new -s abl
cd ~/Jetson-ORB-SLAM3-Hardware
R=~/repro_results_abl
NO="--engine /nonexistent"                         # DBoW2 only for every arm except cnn
E="--engine $HOME/Jetson-ORB-SLAM3-Hardware/cosplace_r50_512.fp16.trt"

# 1. sensors: A1 A3 A4 A5 A6                               (15 runs)
python3 repro/run_matrix.py abl $NO --results $R --arms gpu \
    --seqs husky_20261023__mono husky_20261023__mi husky_20261023__si husky_20261023__ird husky_20261023__rgbd \
    2>&1 | tee -a ~/abl_matrix.log
# 2. stereo variants: C1 D1 D2 D3                          (12 runs)
python3 repro/run_matrix.py abl $NO --results $R --arms gpu \
    --seqs husky_20261023__nolc husky_20261023__7p5 husky_20261023__f800 husky_20261023__bscale \
    2>&1 | tee -a ~/abl_matrix.log
# 3. stereo + IMU variants: C1i D1i D2i                    (9 runs)
python3 repro/run_matrix.py abl $NO --results $R --arms gpu \
    --seqs husky_20261023__si_nolc husky_20261023__si_7p5 husky_20261023__si_f800 \
    2>&1 | tee -a ~/abl_matrix.log
# 4. stereo + IMU arms: B1i B2i B3 C3                      (12 runs)
python3 repro/run_matrix.py abl $NO --results $R --arms cpu gpu_pipe adaptive_fe adaptive_lc \
    --seqs husky_20261023__si 2>&1 | tee -a ~/abl_matrix.log
# 5. CNN loop closure: C2 C2i                              (6 runs; the only command with the engine)
python3 repro/run_matrix.py abl $E --results $R --arms cnn \
    --seqs husky_20261023 husky_20261023__si 2>&1 | tee -a ~/abl_matrix.log
# Ctrl-b d to detach; tmux attach -t abl to look
```
- The blocks are independent. Each command can be re-run, and finished runs are skipped. You can stop
  after any block and continue later.
- **Never** pass the engine (`$E`) to blocks 1–4. With an engine present, every 'auto' arm turns on
  CNN loop closure, and those cells would stop being one-factor changes.
- **Every block line except block 5** must start with `no TensorRT engine at /nonexistent`. Block 5
  must show `engine:linked`.
- Inertial runs print `start VIBA 1` / `end VIBA 1` once the IMU has initialised. The Husky turns at
  most ~7°/s, so initialisation may be slow or may reset (`Not enough motion for initializing`). That
  is counted in the report: it's a result, not a failure to fix.
- Expected harmless ends: `Segmentation fault` / `Bus error` / X error at teardown. `Sophus SO3::exp`
  is the known NaN abort, and the report counts it as a failed run.

Check when done:
```bash
grep "==> done" ~/abl_matrix.log                  # 5 lines, ideally "0 run(s) without a trajectory"
find ~/repro_results_abl/abl -name meta.json | wc -l    # 54
```

## 3. Bring the results to the PC

```bash
# on the PC
cd ~/Desktop/code/Jetson-ORB-SLAM3-Hardware
mkdir -p jetson-orbslam3-results-husky/ablation
rsync -a orb-slam3@172.31.53.140:repro_results_abl :abl_matrix.log :abl_power_mode.txt jetson-orbslam3-results-husky/ablation/
```
You don't need to run `evaluate.py` on the Jetson: the report scores every run itself, on the PC,
in seconds.

## 4. PC: the ablation report

```bash
cd ~/Desktop/code/Jetson-ORB-SLAM3-Hardware
~/venvs/ext/bin/python -I processing/ablation_report.py \
    --stage1 jetson-orbslam3-results-husky/repro_results_ext \
    --abl jetson-orbslam3-results-husky/ablation/repro_results_abl \
    --data ~/datasets/ext \
    --reference jetson-orbslam3-results-husky/reference \
    --out jetson-orbslam3-results-husky/ablation/report
```
`jetson-orbslam3-results-husky/ablation/report/ABLATION_REPORT.md` contains:
1. **All cells:**
   - ATE SE(3) (includes the scale error) and ATE Sim(3) (shape only; the primary metric for mono);
   - scale, RPE over 10 m, coverage, loops, maps, NaN aborts;
   - **Δ vs B0**, with a verdict: within run-to-run spread, below the reference uncertainty, or
     better / worse;
   - IMU initialisation counts for the inertial cells.
2. **Compute per cell:** CPU, GPU, power, energy per frame.
3. **Non-visual baselines E1–E3**, scored the same way.
4. **Plots:** ATE per cell, and accuracy vs energy (the Pareto view for picking the robot's configuration).

Then tell me, and I'll go through it with you.

---

## Checklist
- [ ] P done (PC) · 0 pushed
- [ ] 1 pulled, data copied (12 variant folders, `imu0/data.csv`), dry run shows the right programs
- [ ] 2 blocks 1–5 done; 54 `meta.json`
- [ ] 3 results on the PC · 4 `ABLATION_REPORT.md` generated

## Troubleshooting
| Symptom | Fix |
|---|---|
| `nothing to run` | The `--seqs` folder is missing on the Jetson. Re-run the step 1 rsync. |
| An inertial run exits at once with `*Error with the IMU parameters*` | `settings.yaml` of that variant is broken. Re-run step P on the PC and step 1's rsync. |
| `rgbd_tum` aborts: `Failed to load image` | `depth/`, `color/` or `depth_c/` didn't arrive. Re-run the step 1 rsync. |
| Every inertial run has no trajectory | Read one `console.log`. If IMU initialisation never completes, that is the finding. |
| `arm 'cnn' needs the TensorRT engine` | Block 5 must use `$E`, and `cosplace_r50_512.fp16.trt` must be in the repository root on the Jetson. |
| The report shows "no sequence" for a cell | `--data` doesn't point at the folder holding the 12 variants. |
