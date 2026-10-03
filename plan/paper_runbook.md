# Paper reproduction runbook: recreate the paper's results on our hardware

Goal: re-measure the results of **"Jetson-ORB-SLAM3: Accuracy-Preserving GPU Implementation for
Edge Computing Devices"** (arXiv:2608.17874, `2608.17874v1.pdf` in the repo root) on **our**
Jetson Orin Nano, with the repository **unmodified**. Then extend the same measurements to our
own D435 camera.

Everything we add lives in `repro/`. It calls the repository's own dataset programs (`Examples/...`)
and changes nothing in `src/`, `include/`, `Thirdparty/`, `Examples/` or `CMakeLists.txt`.

The second PDF in the root (*Data Flow ORB-SLAM*, IROS 2019, Jetson TX2) is the paper's
related-work reference [1]. It isn't a reproduction target.

---

## R0. What the paper claims, and how each claim is reproduced

| Paper result | What it measures | Our suite (`repro/run_matrix.py`) | Runs | Report section |
|---|---|---|---|---|
| Table 1 | EuRoC stereo-inertial ATE, GPU vs CPU, 11 sequences | `euroc-accuracy` | 11 × 2 × 5 | Table 1 |
| Table 2 | Mean EuRoC ATE, SE(3) and scaled | (same runs) | – | Table 2 |
| Table 3 | TUM-VI rooms, GPU vs CPU | `tumvi` | 6 × 2 × 1 | Table 3 |
| Table 4 | KITTI t_rel, r_rel, ATE, GPU vs CPU | `kitti` | 11 × 2 × 1 | Table 4 |
| Table 5 | Cross-dataset t_rel (KITTI estimator) | (from the three above) | – | Table 5 |
| Table 6 | Per-stage timing (instrumented build) | `instrumented` | 2 | Table 6 |
| Table 7 | Throughput, mono- and stereo-inertial, Base vs Pipe | `euroc-throughput` | 11 × 4 × 1 | Table 7 |
| Table 8 | Mean tracking time, GPU vs CPU | (from accuracy, TUM-VI, KITTI runs) | – | Table 8 |
| Table 9 | CosPlace latency, TensorRT FP16 | `trtexec` (Part R3, step 4) | – | Table 9 |
| Table 10 | Loop closure: DBoW2 + CNN / DBoW2 / none | `lc-ablation` | 11 × 3 × 5 | Table 10 |
| Sec. 4.1 | Power: 6.3 W mean, 6.9 W peak, 4.7 W idle | `power` | idle + 1 | Power |
| Sec. 3.1.5 | 94.7 % identical keypoints, 99.9 % descriptor bits | `feature_equivalence` (C++) | 205 frames | Features |
| *(new)* | Same equivalence and speed checks on **our D435** | `d435` | seqs × 3 × 3 | Our hardware |

**What we can't reproduce exactly, and why:**
- **The "Desktop" rows of Table 2.** The repository's `CMakeLists.txt` requires CUDA and links
  TensorRT from the Jetson's own library path, so it doesn't build on our Fedora PC without
  editing core files. We run the Orin rows only.
- **The Jetson-SLAM and published-ORB-SLAM3 columns** are other systems' numbers. They're shown
  for reference only.
- **The EuRoC "CPU" arm.** The paper used a separately built upstream ORB-SLAM3 binary there. We
  use the repository's own run-time switch `CPU_ORB=1`, which the paper itself uses for KITTI and
  TUM-VI. It swaps the front end only and keeps everything else fixed, and the paper reports both
  forms agree to 0.019 pp. An optional upstream build is described in R7.
- **JetPack.** Ours is 6.2.3 (L4T 36.5.2); the paper's is 6.2. The paper (Sec. 4.6 iv) notes that
  the software environment can shift results, so expect small differences. Record the version
  with every result (the runner does).

---

## R1. Prerequisites

- **Stage 1 done:** the repository built from source on the Jetson (Stage 1 runbook, Part D).
  The `euroc-accuracy` suite needs `Examples/Stereo-Inertial/stereo_inertial_euroc` built.
- **Tools:** `sudo apt install -y xvfb tmux python3-numpy`. KITTI, TUM-VI and the D435 runs use
  programs that always open a viewer, so they need `xvfb`.
- **Disk:** about 90 GB free: EuRoC ~25 GB, TUM-VI ~15 GB, KITTI ~45 GB while unpacking.
- **Power and thermals:** the stock power profile. **Don't run `nvpmodel` or `jetson_clocks`**
  (repo README). Fan on. Nothing else running during measurements: close the VNC session and the
  browser, and stop the virtual screen (Stage 1 runbook, Part V6).
- **Don't run `apt upgrade`** during the campaign. Every result is tied to the JetPack it was
  measured on.
- **Run everything inside `tmux`.** The full campaign is about 24 h of compute (R8).

Paths (override with environment variables if needed):

| | Default | Variable |
|---|---|---|
| Datasets | `~/datasets` | `REPRO_DATA` |
| Results | `~/repro_results` | `REPRO_RESULTS` |

---

## R2. Datasets

```bash
cd ~/Jetson-ORB-SLAM3-Hardware
python3 repro/fetch_datasets.py euroc            # all 11 sequences, ~1.5 GB download each
python3 repro/fetch_datasets.py tumvi            # rooms 1-6 (512x512 EuRoC export)
python3 repro/fetch_datasets.py status
```

- **EuRoC** uses the repository's own download method (the same code as `run_euroc.sh`). Finished
  sequences are skipped, so re-run it after an interruption. The ETH server rate-limits repeated
  requests (HTTP 429). The script waits 20 min and retries **once**; if a sequence still fails,
  re-run later. **Fallback:** download on the PC and `rsync` the folder into `~/datasets/euroc/`.
- **TUM-VI:** the script downloads `dataset-roomN_512_16.tar` and writes the two text files the
  repository's TUM-VI program needs (`orbslam_times.txt`, `orbslam_imu.txt`). The README says the
  upstream copies were dropped from this branch; ours are generated from the dataset itself, in
  the exact format the program reads. If the download host has moved, download the archives
  manually into `~/datasets/tumvi/` and run `python3 repro/fetch_datasets.py tumvi-prepare`.
- **KITTI odometry** requires a free registration at cvlibs.net, so download it by hand:
  `data_odometry_gray.zip` (~22 GB) and `data_odometry_poses.zip`. Unpack both into
  `~/datasets/kitti/`, so that `sequences/00/{image_0,image_1,times.txt}` and `poses/00.txt` exist.
  Then check the layout:
  ```bash
  python3 repro/fetch_datasets.py kitti-check     # every sequence must say OK
  ```

**Done when:** `status` shows all EuRoC and TUM-VI sequences OK, and `kitti-check` shows 00–10 OK.

---

## R3. Builds and assets

**1. Our measurement tools** (separate CMake project; links the already-built `lib/libORB_SLAM3.so`):
```bash
cmake -S repro/cpp -B repro/cpp/build -DCMAKE_BUILD_TYPE=Release
cmake --build repro/cpp/build -j2
ls repro/cpp/build/feature_equivalence repro/cpp/build/d435_record_euroc
```
If `d435_record_euroc` is missing, CMake didn't find librealsense; it's installed in
`/usr/local` from Stage 1.

**2. Instrumented build for Table 6** (~45 min). This creates an independent checkout of the same
commit at `../Jetson-ORB-SLAM3-timing` and builds it with `REGISTER_TIMES`. The define changes
class layouts, so it can't share this checkout's `lib/`. No source file is edited.
```bash
JOBS=2 repro/setup_timing_build.sh
```
It ends with `OK: instrumented build ready`, after checking that `Tracking::PrintTimeStats` exists
in the new library. If the build fails, the fork's timing code doesn't compile on our toolchain:
record the error. Table 6 can then only be reported from uninstrumented totals (Table 7's mean
tracking times), and the report will say so.

**3. CNN engine for loop closure** (needed for Table 10's "DBoW2 + CNN" arm, and for Table 9).
The repository doesn't include the CosPlace model.
1. **On the PC** (needs PyTorch): `pip install torch torchvision onnx`, then
   `python3 repro/tools/export_cosplace_onnx.py cosplace_r50_512.onnx`.
2. Copy the `.onnx` to the Jetson's repo root: `scp cosplace_r50_512.onnx orb-slam3@<jetson-ip>:Jetson-ORB-SLAM3-Hardware/`.
3. **On the Jetson**, build the engine. TensorRT engines only work on the board and version that
   built them; it takes about 30 s.
   ```bash
   cd ~/Jetson-ORB-SLAM3-Hardware
   /usr/src/tensorrt/bin/trtexec --onnx=cosplace_r50_512.onnx --saveEngine=cosplace_r50_512.fp16.trt --fp16
   ```
   The library looks for `cosplace_r50_512.fp16.trt` in the **working directory**. The runner
   links it into each run folder only for the arms that should use it. Both `.onnx` and `.trt`
   files are kept out of git by `.gitignore`.
4. **Table 9 latency:**
   ```bash
   mkdir -p ~/repro_results/cnn_latency
   /usr/src/tensorrt/bin/trtexec --loadEngine=cosplace_r50_512.fp16.trt --iterations=500 \
       2>&1 | tee ~/repro_results/cnn_latency/tensorrt_fp16.log
   ```
   The report reads the mean `GPU Compute Time` from that log. The paper reports 2.2 ms. The
   ONNX-Runtime rows of Table 9 are the paper's negative result (the EP fails to initialise); we
   don't re-run them.

**Without the engine:** the runner prints `no TensorRT engine ... DBoW2-only` and runs Tables 1–8
with DBoW2 loop closure, the repository's default. The `cnn` arm of `lc-ablation` refuses to start.

---

## R4. Run the experiments

Order: short suites first, so problems show up early. Each command can be re-run after an
interruption, because finished runs are skipped. Use `--dry-run` to see the commands without
running anything.

```bash
tmux new -s repro
cd ~/Jetson-ORB-SLAM3-Hardware

# 0. a 2-run smoke test of the whole chain (~10 min), then delete it
python3 repro/run_matrix.py euroc-accuracy --seqs MH01 --runs 1 --results ~/repro_smoke
python3 repro/evaluate.py ~/repro_smoke && python3 repro/make_report.py ~/repro_smoke
head -30 ~/repro_smoke/REPORT.md        # MH01 GPU and CPU must both have an ATE of a few cm

# 1. feature-level equivalence, 205 MH01 frames (~5 min)
mkdir -p ~/repro_results/features
repro/cpp/build/feature_equivalence ~/datasets/euroc/MH_01_easy/mav0/cam0/data \
    ~/repro_results/features/features.json

# 2. power: 60 s idle baseline, then MH01, tegrastats at 250 ms (~6 min)
python3 repro/run_matrix.py power

# 3. per-stage timing with the instrumented build (~10 min)
python3 repro/run_matrix.py instrumented --build-root ../Jetson-ORB-SLAM3-timing

# 4. throughput, Base vs Pipe, mono- and stereo-inertial (~2.5 h)
python3 repro/run_matrix.py euroc-throughput

# 5. accuracy, GPU vs CPU, 5 runs each (~7.5 h)
python3 repro/run_matrix.py euroc-accuracy

# 6. TUM-VI and KITTI, GPU vs CPU (~0.7 h and ~1.5 h)
python3 repro/run_matrix.py tumvi
python3 repro/run_matrix.py kitti

# 7. loop-closure ablation, 3 arms x 5 runs (~11 h; needs the engine for the cnn arm)
python3 repro/run_matrix.py lc-ablation
```

**What the runner does for each run:**
- Makes its own folder: `~/repro_results/<suite>/<mode>/<seq>/<arm>/run<k>/`.
- Starts the repository's program with the switches for that arm:

  | Arm | Front end | Pipelining | Loop closure |
  |---|---|---|---|
  | `gpu` | GPU | – | default |
  | `cpu` | `CPU_ORB=1` | – | default |
  | `gpu_pipe` | GPU | `PIPELINE_FE=1` | default |
  | `cnn` | GPU | – | engine linked into the folder |
  | `dbow2` | GPU | – | no engine |
  | `nolc` | GPU | – | `Examples/Stereo-Inertial/EuRoC_noloop.yaml` (`loopClosing: 0`) |

  "Default" means the CNN is used if the engine exists, otherwise DBoW2.
- Runs `tegrastats` alongside, and writes `console.log`, the trajectories and `meta.json`
  (command, switches, git commit, L4T version, exit code, wall time).
- Runs are ordered **run number first**, then sequence, then arm. A partial campaign then still
  has every sequence and arm equally covered, and GPU/CPU runs alternate so thermal drift affects
  both equally. There is a 10 s pause between runs.

**Don't interact with the Jetson during runs.** Any extra load changes timing and power numbers,
and can even change trajectories (ORB-SLAM3 is multi-threaded).

---

## R5. Evaluate and build the report

```bash
python3 repro/evaluate.py ~/repro_results          # scores new runs (eval.json next to each)
python3 repro/make_report.py ~/repro_results       # writes ~/repro_results/REPORT.md
```

**How the numbers are computed** (`repro/evaluate.py`, following the paper's Sec. 4.2):
- **ATE:** the estimate is matched to ground truth by nearest timestamp within 20 ms, aligned with
  Umeyama's method, and the RMS of the position errors is taken. Both **SE(3)** (scale fixed;
  the paper's main metric) and **Sim(3)** (scale fitted; the "scaled" column) are reported.
  - The SE(3) computation is checked against the repository's own scorer in `run_euroc.sh` and
    gives identical numbers.
  - EuRoC and TUM-VI inertial runs write their poses in the **IMU body frame**, the frame of the
    ground truth, as the paper requires.
  - The primary ATE uses the **keyframe** trajectory, as `run_euroc.sh` does. The every-frame ATE
    is listed as a secondary table.
- **Relative errors:** KITTI's official estimator. Segments start every 10 poses and are 100–800 m
  long (5–40 m for EuRoC and TUM-VI, as the paper scales them). t_rel is in %, r_rel in °/100 m.
- **Aggregation, as in the paper:** per-sequence **median over runs**, then the mean over
  sequences. Δ (abs) is the per-sequence GPU-vs-CPU gap. Every run is kept, including failures,
  which are listed under "Run health".
- **Throughput** is 1 / mean tracking time, as the dataset programs print it ("mean tracking
  time"). **Power** is the VDD_IN rail from tegrastats.

**What counts as "reproduced":**

| Claim | Reproduced if |
|---|---|
| GPU ≡ CPU on EuRoC (Table 1) | Mean per-sequence Δ (abs) ≲ 0.3 cm, and each sequence's gap within that sequence's run-to-run spread |
| Accuracy level (Tables 1–2) | Mean SE(3) ATE ≈ 3.3 cm (± ~0.5 cm). V203 and MH04–05 vary most between runs (paper, Sec. 4.6) |
| TUM-VI (Table 3) | Sub-centimetre on most rooms; GPU-CPU gap ≈ 0.2 cm |
| KITTI (Table 4) | t_rel < 1 % on 9 of 11 sequences; GPU-CPU gap ≲ 0.05 pp |
| Throughput (Table 7) | Stereo-inertial Base ≈ 14 FPS, Pipe ≳ 20 FPS (camera rate); mono-inertial ≈ 32 FPS |
| Timing split (Tables 6, 8) | Within ~10 %; on EuRoC the CPU extractor is faster than the GPU (the paper's own finding) |
| Power (Sec. 4.1) | Within ~0.5 W of 6.3 W mean / 6.9 W peak / 4.7 W idle |
| Features (Sec. 3.1.5) | ≈ 95 % identical keypoints, ≈ 99.9 % descriptor-bit agreement |
| Loop closure (Table 10) | CNN ≈ DBoW2 (accuracy-neutral); loop closing helps V202 most |

Differences beyond these need an explanation in the report, such as the JetPack version or the
viewer cost under xvfb. Don't tune anything to match the paper.

---

## R6. Our hardware: the same checks on our D435

The paper's numbers come from public datasets. To show the same properties on **our** camera
(GPU ≡ CPU, real-time throughput, power), we record the D435 once and replay the identical
recording under every configuration.

**1. Record** (with the virtual screen not running; images go straight to disk):
```bash
cd ~/Jetson-ORB-SLAM3-Hardware
repro/cpp/build/d435_record_euroc ~/datasets/d435/room_loop1 --seconds 120
```
- It records the IR left and right images in the EuRoC layout (`mav0/cam0`, `mav0/cam1`,
  `times.txt`), with the same camera settings as the repository's live driver: 640×480 @ 30,
  projector off, 5 ms exposure cap.
- Stop early with `q` then Enter, or Ctrl-C. Check `recording_info.txt`: **dropped must be 0**.
- Record 3–4 sequences, following the Stage 1 walking rules (both hands, slow turns, pass the
  start mark and keep going 2–3 m):
  - `room_loop1` and `room_loop2`: handheld room loops;
  - `rover_loopA`: route A on the rover;
  - `corridor`: an out-and-back corridor run.

**2. Run** (the program opens a viewer, so the runner wraps it in `xvfb-run`):
```bash
python3 repro/run_matrix.py d435          # every recorded sequence x {gpu, cpu, gpu_pipe} x 3 runs
python3 repro/evaluate.py ~/repro_results && python3 repro/make_report.py ~/repro_results
```

**3. Read the "Our hardware" section of the report.** There's no ground truth, so equivalence is
shown the way the paper argues it: **the GPU-vs-CPU distance between trajectories must be no larger
than the run-to-run distance** of the GPU arm alone, or of the CPU arm alone. The section also
lists the tracking time and FPS per arm, how often the loop closed, and how many maps were made.

---

## R7. Optional extras

- **The upstream CPU reference for EuRoC** (the paper's exact EuRoC CPU arm). Clone
  `UZ-SLAMLab/ORB_SLAM3` into a **separate** folder (not this repo) and build it there. It needs
  C++14 on JetPack 6 (set it in *that* clone's `CMakeLists.txt`). Then:
  ```bash
  python3 repro/run_matrix.py euroc-accuracy --arms gpu --build-root ~/ORB_SLAM3_upstream \
      --results ~/repro_results_upstream
  ```
  Here `gpu` is just a label: upstream has no GPU path. Report it as "CPU (upstream)".
- **A desktop arm:** possible only with an NVIDIA desktop GPU, plus a separate copy whose
  `CMakeLists.txt` points TensorRT at the desktop path. That's out of scope for "unmodified".

---

## R8. Time budget and checklist

| Step | Time |
|---|---|
| R2 datasets | 2–4 h of downloads, plus KITTI by hand |
| R3 builds | ~1 h (the timing build) + 15 min |
| R4 suites | ~24 h of compute: accuracy 7.5 h, ablation 11 h, throughput 2.5 h, KITTI 1.5 h, TUM-VI 0.7 h, the rest < 0.5 h |
| R6 D435 | ~1 h recording + ~3 h of runs |

- [ ] R2: all datasets OK
- [ ] R3: tools built; timing build OK; CNN engine built; Table 9 latency logged
- [ ] R4: smoke test passed; every suite finished; "Run health" reviewed
- [ ] R5: `REPORT.md` generated; every "reproduced if" row checked; differences explained
- [ ] R6: D435 recordings (0 dropped) and the `d435` suite done
- [ ] Results archived: `rsync -a orb-slam3@<jetson-ip>:repro_results ~/Desktop/jetson-orbslam3-results/`

## Troubleshooting

| Symptom | Fix |
|---|---|
| `cannot start: missing program ...` | Build the repository (Stage 1, Part D); for `instrumented`, run `repro/setup_timing_build.sh` |
| `missing data ...` | `python3 repro/fetch_datasets.py status`, then fetch what's missing |
| `HTTP Error 429` | ETH rate limit. The script waits and retries once; otherwise download on the PC |
| Run folder has no trajectory | Read its `console.log`. A viewer program without `xvfb` aborts at start (`apt install xvfb`) |
| KITTI `pose count != ground truth` | Frames were dropped (tracking was lost at the start), so poses can't be matched by line number. Re-run that sequence |
| Timing numbers noisy | Something else was running. Re-run with `--redo`, nothing else active |
| Instrumented run has no `ExecMean.txt` | The build doesn't have `REGISTER_TIMES`. Re-run `setup_timing_build.sh` and check its final line |
