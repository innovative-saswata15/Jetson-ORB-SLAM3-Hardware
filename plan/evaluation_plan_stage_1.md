# Evaluation plan, stage 1: our ORB-SLAM3 against the other team's lidar ground truth

**Goal:** measure the absolute accuracy of our Jetson ORB-SLAM3 build (ATE, RPE, scale) on another
team's D435 recordings. Their Husky's 3D-lidar SLAM trajectory is the reference. This is "experiment 1"
from our discussion: their camera, their robot, **our software on our Jetson**. It's the first time we
get real accuracy numbers on D435 data (the R6 recordings have no ground truth).

**What we asked them for (email of 2026-10-14):**
1. Left and right IR images (uncompressed), with the D435 calibration
2. Their lidar ground-truth trajectory
3. The camera-to-lidar transform

We expect all of this in a rosbag in our Drive folder.

**What stage 1 produces:** for each of their sequences and for each arm (GPU / CPU / pipe):
- ATE in SE(3) and Sim(3);
- relative errors;
- tracking coverage;
- throughput and power;
- a GPU-vs-CPU equivalence check;

all of it in a report in `jetson-orbslam3-results-external/`.

Conventions are the same as [paper_runbook.md](paper_runbook.md):
- **"PC"** is the Fedora machine (`~/Desktop/code/Jetson-ORB-SLAM3-Hardware`).
- **"Jetson"** is the Orin Nano (`~/Jetson-ORB-SLAM3-Hardware`, user `orb-slam3`).
- Replace `<jetson-ip>` with the Jetson's address.

---

## E0. Overview

```
 Drive (their bag)                                  PC                                     Jetson
 ─────────────────   download   ┌───────────────────────────────────┐   rsync   ┌──────────────────────────┐
 seq.bag / seq/  ─────────────▶ │ E1 inspect  → E2 convert          │ ────────▶ │ E4 smoke → E5 full matrix │
                                │   images → EuRoC folder           │           │  (run_matrix.py ext)      │
                                │   camera_info → settings.yaml     │           └────────────┬─────────────┘
                                │   lidar poses + TF → gt.csv       │                        │ rsync back
                                │ E3 sanity checks + plots          │ ◀──────────────────────┘
                                │ E6 evaluate → E7 diagnostics      │
                                │ E8 report                         │
                                └───────────────────────────────────┘
```

| Phase | Where | Time |
|---|---|---|
| E1 Receive and inspect | PC | 30 min |
| E2 Convert (one-time tool build, then per sequence) | PC | 1 day for the tools, then 10 min per sequence |
| E3 Sanity checks | PC | 30 min per sequence |
| E4 Jetson prep and smoke run | Jetson | 1 h |
| E5 Full run matrix | Jetson | ~(sequence length × 1.5) × 9 per sequence, unattended |
| E6 Evaluate | PC | 30 min |
| E7 Diagnostics | PC | 1–2 h |
| E8 Report and archive | PC | 2 h |

**Code to write before E2** (none of it touches `src/`, `include/` or `Examples/`, only `repro/`):

| File | Purpose |
|---|---|
| `processing/inspect_bag.py` | List the topics, rates, counts, time spans, encodings and frames in their bag |
| `processing/bag_to_euroc.py` | Write the IR pairs to the EuRoC layout we already use (`mav0/cam0`, `mav0/cam1`, `times.txt`) |
| `processing/make_settings.py` | Build an ORB-SLAM3 stereo YAML from the two `camera_info` messages |
| `processing/make_gt.py` | Turn the lidar trajectory into camera-frame poses at the image timestamps → `gt.csv` |
| `processing/plot_check.py` | Plots for E3 (path, timestamps, sample frames) and E6 (overlays) |
| `repro/common.py` + `repro/run_matrix.py` | A new `ext_s` mode and an `ext` suite (see E4.1) |

---

## E1. Receive and inspect the data (PC)

### E1.1 Download
```bash
mkdir -p ~/datasets/external_raw && cd ~/datasets/external_raw
# download the Drive folder (browser, or: pip install gdown && gdown --folder <drive-url>)
ls -lh
```

### E1.2 Install the bag reader
The PC has no ROS. The `rosbags` library reads ROS 1 `.bag` files and ROS 2 `.db3`/`.mcap` folders
without needing ROS.
```bash
python3 -m venv ~/venvs/ext && source ~/venvs/ext/bin/activate
pip install rosbags numpy opencv-python-headless matplotlib
```

### E1.3 Inventory
```bash
python3 processing/inspect_bag.py ~/datasets/external_raw/<bag> | tee ~/datasets/external_raw/<bag>.inventory.txt
```
Fill in this table for **each** bag before going further:

| Check | Expected | If not |
|---|---|---|
| Left IR topic (e.g. `/camera/infra1/image_rect_raw`) | present, `mono8`, uncompressed | `…/compressed`: PNG is OK; **JPEG is not**, so ask them for raw |
| Right IR topic (`/camera/infra2/image_rect_raw`) | present, the **same count** as the left (±1) | big mismatch → frames were dropped; pairs get matched by timestamp in E2 |
| Image rate | 15, 30 or 60 Hz, steady | throttled (<10 Hz) → tracking will suffer; note it in the report |
| Resolution | 640×480 or 848×480 (or 1280×720) | anything works; the YAML comes from `camera_info` |
| `camera_info` for infra1 and infra2 | present | missing → ask for it, or for the camera serial (factory calibration) |
| Ground-truth poses | an odometry/pose topic, or a separate file | see E2.3 for each format |
| GT time span | covers the image time span | partial → only the overlap is evaluated |
| `/tf_static` | contains a chain from the lidar/`base_link` to `camera_infra1_optical_frame` | missing → E7.3 (estimate the transform) |
| IMU topics | optional | if present, keep them for stage 2 (stereo-inertial) |
| Emitter (projector) | **off** preferred | on → dots visible in the IR; run anyway and note it (E3.3) |

Also write down from their reply: the lidar-SLAM method used (LIO-SAM / FAST-LIO / …), whether it
closed loops, and its expected accuracy. That sets the floor of what we can resolve (E7.4).

---

## E2. Convert to our format (PC)

Target layout, one folder per sequence (the same as our R6 D435 recordings plus two files):
```
~/datasets/ext/<seq>/
├── mav0/cam0/data/<t_ns>.png      left IR
├── mav0/cam1/data/<t_ns>.png      right IR
├── times.txt                      one <t_ns> per stereo pair
├── settings.yaml                  ORB-SLAM3 stereo settings built from their calibration
├── gt.csv                         reference poses of the LEFT IR optical frame (EuRoC GT format)
└── source_info.txt                bag name, topics, counts, transform used, conversion date
```
Name sequences `ext_<place>_<n>` (e.g. `ext_lab_1`).

### E2.1 Images → EuRoC layout
```bash
python3 processing/bag_to_euroc.py <bag> ~/datasets/ext/<seq> \
    --left /camera/infra1/image_rect_raw --right /camera/infra2/image_rect_raw
```
What the tool does:
- **Pairs the left and right images by header stamp.** The two IR images of a D435 frame share a
  stamp. It allows a 1 ms tolerance and drops unmatched images, reporting how many.
- **Stamp source:** uses the image header stamp, not the bag receive time. The receive time includes
  transport delay.
- **Output format:** writes 8-bit PNGs (lossless), named `<t_ns>.png`, and `times.txt`.
- **Prints a summary:** pairs written, unmatched left/right, the median Δt, the largest gap (dropped
  frames) and the duration.

### E2.2 Calibration → `settings.yaml`
```bash
python3 processing/make_settings.py <bag> ~/datasets/ext/<seq>/settings.yaml \
    --left-info /camera/infra1/camera_info --right-info /camera/infra2/camera_info
```
- It copies [Examples/Stereo/RealSense_D435.yaml](../Examples/Stereo/RealSense_D435.yaml) and replaces
  only the camera block, so the ORB settings stay identical to R6 (1250 features, 8 levels,
  scale 1.2, FAST 20/7). Results then stay comparable to campaign 1 and 2.
- `Camera1.fx/fy/cx/cy` come from the left `P` (or `K`) matrix.
- **`Stereo.b` = −P_right[0,3] / fx_right.** It should be **≈ 0.050 m** for a D435. Abort if it is
  outside 0.045–0.055.
- `Camera.width/height` come from `camera_info`. `Camera.fps` is the measured image rate, rounded.
- `Camera.type: "Rectified"`. D435 `image_rect_raw` IR streams are already rectified with zero
  distortion. If `camera_info.D` is non-zero, stop and check which topic was recorded.

### E2.3 Lidar trajectory + transform → `gt.csv`
```bash
python3 processing/make_gt.py <bag> ~/datasets/ext/<seq>/gt.csv \
    --pose-topic <their odometry topic>  --cam-frame camera_infra1_optical_frame \
    --times ~/datasets/ext/<seq>/times.txt
```
**Why the transform matters.** Their poses describe the lidar (or `base_link`). ORB-SLAM3 reports the
**left IR optical frame**. The SE(3) alignment in the evaluation removes the difference between
world frames, but **not the lever arm** between the lidar and the camera. With a 0.3 m offset, every
turn would show up as several cm of false error. So:

```
T_world_cam(t) = T_world_lidar(t) · T_lidar_cam          (T_lidar_cam from /tf_static, chained)
```

What the tool does:
1. Reads the poses from the topic (`nav_msgs/Odometry`, `geometry_msgs/PoseStamped`), or from a file
   (`--pose-file`, TUM format `t x y z qx qy qz qw`).
2. Looks up `T_<pose frame>_<camera_infra1_optical_frame>` by chaining `/tf_static`. It prints the
   chain and the translation, which should be plausible: about 0.1–0.5 m for a camera mounted near
   the lidar.
3. **Interpolates the GT to every image timestamp:** linear for position, SLERP for rotation. Lidar
   SLAM poses are usually 10–20 Hz, but `repro/evaluate.py` matches estimate and GT within 20 ms, so
   the GT is written at the camera rate.
   - It never extrapolates.
   - Image times with no GT pose within 0.2 s on both sides are skipped and counted.
4. Writes the EuRoC GT format that `repro/evaluate.py` (`load_gt_csv`) already reads:
   `#timestamp[ns],p_x,p_y,p_z,q_w,q_x,q_y,q_z`.
5. Writes the transform used, the counts and the time spans to `source_info.txt`.

**Which pose topic to use:** prefer their **optimised / loop-closed** trajectory (for example a saved
`.txt` from the mapping back-end) over the live odometry topic. Live LIO odometry drifts. If they only
gave live odometry, note it in the report.

---

## E3. Sanity checks (PC, per sequence). Don't skip these.

```bash
python3 processing/plot_check.py ~/datasets/ext/<seq>      # writes check_*.png into the folder
```

| # | Check | Pass criterion |
|---|---|---|
| 1 | `times.txt` is strictly increasing; frame interval histogram | one peak at 1/fps; gaps under 3× the interval are rare |
| 2 | Left/right pair count | under 1% unmatched |
| 3 | **Look at 5 random frames** (left and right side by side) | rectified (the same feature on the same row in both images), no projector dots, exposure OK, little blur |
| 4 | Baseline from `settings.yaml` | 0.045–0.055 m |
| 5 | GT path plot (top view + height over time) | a smooth, plausible route, no jumps; the height is roughly constant for a ground robot |
| 6 | GT coverage | ≥ 95% of the image timestamps have a GT pose |
| 7 | GT speed over time (from `gt.csv`) | under ~2 m/s for a Husky indoors; spikes mean bad timestamps or a lidar-SLAM glitch |
| 8 | Lever-arm magnitude in `source_info.txt` | matches the physical mounting (ask them, or look at photos) |

If check 3 shows projector dots, run the sequence anyway, but flag every number from it in the report.
If they have a run with the projector off, prefer that one.

---

## E4. Jetson preparation and smoke run

### E4.1 Pipeline support (one-time code change in `repro/`, done on the PC and committed)
The R6 mode `d435_s` uses one fixed settings file and has no ground truth. Add a mode for external
sequences:

| Change | Detail |
|---|---|
| `repro/common.py` `MODES["ext_s"]` | binary `Examples/Stereo/stereo_euroc`; **settings = `<seq>/settings.yaml`**; `viewer=True` (xvfb); `dataset="ext"`; `gt_frame="camera"`; `output="euroc"` |
| `ext_dir(seq)`, `list_ext_sequences()` | `~/datasets/ext/<seq>`, the sequences that have `times.txt` + `gt.csv` |
| `mode_command("ext_s", …)` | `[binary, voc, <seq>/settings.yaml, <seq>, <seq>/times.txt, tag]` |
| `ground_truth("ext_s", seq)` | `<seq>/gt.csv`. The existing `score_euroc_like` then computes ATE SE(3)/Sim(3), scale and t_rel/r_rel with no further changes. |
| `repro/run_matrix.py` `SUITES["ext"]` | `modes=["ext_s"], arms=["gpu","cpu","gpu_pipe"], runs=5, seqs=None` (5 runs, as the paper uses) |
| `repro/make_report.py` | an "External sequences (lidar GT)" section: the E6.1 table, plus the equivalence table from R6 |

Test it with `python3 repro/run_matrix.py ext --dry-run` on the PC (it prints the commands without
running them).

### E4.2 Copy to the Jetson
```bash
# on the PC
rsync -a --info=progress2 ~/datasets/ext/ orb-slam3@<jetson-ip>:datasets/ext/
ssh orb-slam3@<jetson-ip> 'cd ~/Jetson-ORB-SLAM3-Hardware && git pull'
```

### E4.3 Fix the platform state (Jetson). The same for every run, and recorded.
```bash
sudo nvpmodel -m 2                     # MAXN_SUPER (0 = 15W, 1 = 25W on our board!)
sudo nvpmodel -q | tee ~/ext_power_mode.txt
sudo jetson_clocks && sudo jetson_clocks --show | tee ~/ext_clocks.txt    # CPU 1728 MHz, GPU ~1020 MHz
```
**Decide on CNN loop closure before starting, and keep it fixed:**
- **Default:** DBoW2 only (`--engine /nonexistent`). It's the configuration of campaign 1 and the
  cleanest one.
- **Optional:** a second pass with the CosPlace engine (`--arms cnn dbow2`). That reproduces the
  paper's Table 10 ablation on real GT. Do this only after E5 succeeds.

### E4.4 Smoke run: 1 sequence, 1 run, GPU arm
```bash
cd ~/Jetson-ORB-SLAM3-Hardware
python3 repro/run_matrix.py ext --seqs <seq> --arms gpu --runs 1 --engine /nonexistent \
    --results ~/repro_results_ext_smoke
python3 repro/evaluate.py ~/repro_results_ext_smoke
cat ~/repro_results_ext_smoke/ext/ext_s/<seq>/gpu/run1/eval.json
```
Pass if:
- `kf_run.txt` exists;
- `kf_poses` is > 90% of `kf_total` (the estimate overlaps the GT);
- **`kf_sim3_scale` is between 0.97 and 1.03** (stereo scale is metric). If not, the baseline in
  `settings.yaml` or the GT units are wrong;
- the ATE is plausible (expect a few cm to a few dm). Over 1 m means the extrinsic or time
  alignment is wrong, so go to E7 before E5.

---

## E5. Full run matrix (Jetson, unattended)

```bash
cd ~/Jetson-ORB-SLAM3-Hardware
nohup python3 repro/run_matrix.py ext --engine /nonexistent --results ~/repro_results_ext \
    > ~/ext_matrix.log 2>&1 &
tail -f ~/ext_matrix.log
```
- 3 arms × 5 runs × N sequences. Each run takes about 1.3–2× the sequence length (CPU arm ~1.7×, pipe
  ~1.3× at MAXN).
- Interrupted runs: run the same command again. Finished runs are skipped.
- Keep the Jetson otherwise idle: no builds, no VNC viewer, no other SSH jobs.
- **Optional, after it finishes:**
  - an idle baseline:
    `sudo tegrastats --interval 250 --logfile ~/repro_results_ext/idle.log & sleep 60; sudo tegrastats --stop`
  - the CNN ablation:
    `python3 repro/run_matrix.py ext --arms cnn dbow2 --results ~/repro_results_ext_lc`

Bring the results back:
```bash
# on the PC
rsync -a orb-slam3@<jetson-ip>:repro_results_ext ~/Desktop/code/Jetson-ORB-SLAM3-Hardware/jetson-orbslam3-results-external/
scp orb-slam3@<jetson-ip>:'~/ext_power_mode.txt ~/ext_clocks.txt ~/ext_matrix.log' \
    ~/Desktop/code/Jetson-ORB-SLAM3-Hardware/jetson-orbslam3-results-external/
```

---

## E6. Evaluate (PC)

```bash
cd ~/Desktop/code/Jetson-ORB-SLAM3-Hardware
R=jetson-orbslam3-results-external/repro_results_ext
python3 repro/evaluate.py $R && python3 repro/make_report.py $R
python3 jetson-orbslam3-results/analyze_d435.py $R/ext_perf.json     # performance (point its root at $R/ext/ext_s)
python3 processing/plot_check.py --overlay $R ~/datasets/ext      # trajectory overlays
```

### E6.1 Accuracy metrics (per sequence × arm, median over the 5 runs, and min/max)

| Metric | From | Meaning |
|---|---|---|
| **ATE SE(3), keyframes (cm)** | `kf_ate_se3_cm` | **The headline number.** The same definition as the paper's Table 1. |
| ATE SE(3), all frames (cm) | `frames_ate_se3_cm` | Includes every tracked frame, so it's stricter |
| ATE Sim(3) (cm) and **scale** | `*_ate_sim3_cm`, `*_sim3_scale` | The scale should be ≈1. The gap between SE(3) and Sim(3) shows the scale error. |
| t_rel (%) and r_rel (°/100 m) | `trel_5_40_pct`, `rrel_5_40_deg100m` | Drift over 5–40 m windows (the paper's Table 5 estimator) |
| Tracking coverage (%) | `frames_poses / frames in times.txt` | Share of the sequence that was localised |
| Maps, loops, merges, "Fail to track" | `console` | Robustness |

### E6.2 Equivalence (the paper's claim, now with GT)
- Δ = |median ATE GPU − median ATE CPU| for each sequence. Compare it with the **run-to-run spread**
  (max − min) within each arm. The claim holds if Δ is within that spread.
- Also give the pairwise trajectory distance tables from R6 (GPU vs CPU, GPU vs GPU, CPU vs CPU,
  pipe vs pipe).

### E6.3 Performance (the same as the campaign reports)
For each arm:
- FPS = frames ÷ tracking window, and ms per frame;
- VDD_IN mean and peak, energy per frame;
- CPU and GPU load, RAM peak, Tj max.

The real-time factor is relative to **their** frame rate.

### E6.4 Plots
- A top view of the GT path against the aligned estimate (one panel per arm, median run).
- Position error over time and over distance travelled.
- ATE box plots for each arm and sequence.

---

## E7. Diagnostics: run these if anything looks off, and for the error budget anyway

### E7.1 Time offset
Their image stamps and GT stamps may be offset (different machines, or lidar-SLAM latency). Shift
the GT by δ ∈ [−200, +200] ms in 5 ms steps and recompute the SE(3) ATE of one GPU run.
- A clear minimum at |δ| > 10 ms means there's an offset. Regenerate `gt.csv` with `--time-offset δ`
  and re-evaluate, and report δ.
- Without a clear minimum, the clocks were fine.

### E7.2 Lever-arm check
Recompute the ATE using the uncorrected lidar poses (no `T_lidar_cam`). The corrected ATE should be
**lower**. If it's higher, the TF chain is wrong: check the direction (inverse) and that the frame is
the *optical* one (z forward), not `camera_link` (x forward).

### E7.3 If `/tf_static` was not provided
Estimate `T_lidar_cam` from the two trajectories by **hand-eye calibration**: the AX = XB problem on
relative motions, using a GPU run that has good coverage.
- It needs sufficient rotation about at least two axes. A ground robot rotates mainly in yaw, so the
  vertical offset is poorly observable. Fix z from a tape measurement.
- Report the estimated transform, and evaluate with it. Mark the results as *"extrinsic estimated"*.

### E7.4 Error budget
We can't resolve errors smaller than the reference's own error. State the lidar-SLAM accuracy
(from them, or typically 2–5 cm indoors for LIO with loop closure). Treat ATE differences below it as
"not resolvable".

### E7.5 Failure triage
| Symptom | Likely cause | Action |
|---|---|---|
| Sim(3) scale far from 1 | wrong baseline sign or units in `settings.yaml`; GT in mm | Recheck E2.2 and E3.4 |
| Large ATE, but the shape matches after Sim(3) | scale issue (above) | Same as above |
| ATE grows linearly with each turn | missing or inverted lever arm | E7.2 |
| A good shape, but a constant lag along the path | time offset | E7.1 |
| Frequent "Fail to track", new maps | low texture, motion blur, projector on, throttled frames | Check E3.3 and the frame rate. Note it in the report; it's not a bug. |
| `Sophus SO3::exp` NaN abort | the known bug (campaign 1 and 2) | Count it as a failed run. Don't rerun to hide it. |
| Run has no trajectory | the `console.log` tail | paper_runbook troubleshooting |

---

## E8. Report and archive

Write `jetson-orbslam3-results-external/EXTERNAL_GT_EVALUATION_REPORT.md` (the same structure as the
campaign reports):
1. **Data:** the source (team, robot, place, date), sequences with length / duration / frame rate,
   the projector state, the lidar-SLAM method, the transform used, and the time offset found.
2. **Platform state:** power mode, clocks, commit, CNN on or off.
3. **Accuracy:** the E6.1 table, and the comparison with the paper (EuRoC Orin GPU 3.32 cm / CPU
   3.40 cm). Different data, so present it as context, not as a pass/fail.
4. **Equivalence:** the E6.2 tables and a verdict.
5. **Performance:** the E6.3 tables, and a comparison with campaign 2.
6. **Robustness:** coverage, maps, crashes.
7. **Error budget and limitations:** E7.4, plus anything flagged in E3.
8. **Plots** (E6.4).
9. **Next steps:** experiment 2 (our rig on the Husky) and experiment 3 (our rover with checkpoints).

Archive:
- Keep `~/datasets/ext/` and the raw bags. Converted sequences are reproducible from the bags with the
  E2 commands.
- Commit the `processing/` tools and the report, **not the data**.
- Their data is shared with us: don't redistribute it, and credit them in the report. Send them the
  report, as offered in the email.

---

## Acceptance checklist

- [ ] E1: inventory done for every bag; all three requested items present (or the gaps handled)
- [ ] E2: every sequence converted; baseline 0.045–0.055 m; GT coverage ≥ 95%
- [ ] E3: all 8 checks passed (or flagged) per sequence; sample frames looked at
- [ ] E4: `ext_s` mode added; dry run OK; MAXN_SUPER + `jetson_clocks` recorded; smoke run scale 0.97–1.03
- [ ] E5: 3 arms × 5 runs per sequence completed; results copied to the PC
- [ ] E6: accuracy, equivalence and performance tables generated; plots made
- [ ] E7: time-offset sweep and lever-arm check done; error budget stated
- [ ] E8: report written; tools committed; report sent to the other team and the professor

## If the data differs from what we asked for

| Received | Plan change |
|---|---|
| Separate image folders + pose file, not a bag | Skip E2.1 (copy and rename into the EuRoC layout); `make_gt.py --pose-file` |
| Compressed images (JPEG) | Run anyway, flag it, and ask for raw. ORB features on JPEG differ from raw. |
| Projector on | Run anyway, flag it, and ask whether a projector-off run exists |
| Only live LIO odometry, no optimised trajectory | Use it; widen the error budget (E7.4); prefer the shorter sequences |
| No transform | E7.3 hand-eye estimate + tape measurement |
| D435i with IMU | Stage 1 unchanged. Keep the IMU for a stereo-inertial stage 2 (needs IMU noise parameters and `Tbc`). |
| 848×480 or 15 FPS | No change. `settings.yaml` handles it; report FPS relative to their rate. |
