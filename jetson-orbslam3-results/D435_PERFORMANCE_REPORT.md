# Jetson ORB-SLAM3 on our D435: performance and results analysis (runbook R6)

**Platform:** Jetson Orin Nano 8 GB, L4T R36.5.2 (JetPack 6), build `f06cee0`
**Input:** 5 D435 IR-stereo recordings (640×480 @ 30 FPS, projector off), each replayed offline
**Matrix:** 5 sequences × 3 arms × 3 runs = **45 runs**, recorded on 2026-10-06
**Raw data:** [repro_results/](repro_results/) (auto-generated tables in [REPORT.md](repro_results/REPORT.md))
**Re-derive every number below:** `python3 analyze_d435.py out.json` ([analyze_d435.py](analyze_d435.py))

R2–R5 (EuRoC / TUM-VI / KITTI) were skipped on purpose. For those we rely on the paper's published
numbers. This report covers only our own hardware and data.

---

## 1. TL;DR

| Question | Answer |
|---|---|
| Does the GPU front-end give the same trajectories as the CPU reference? | **Yes.** On all 5 sequences, the GPU-vs-CPU distance between trajectories is within the run-to-run noise of a single arm. |
| Is GPU ORB faster than CPU ORB for 640×480 stereo? | **No.** Both run at **14.4 FPS (69.5 ms/frame)**. This matches the paper's finding for EuRoC stereo (GPU 69.7 ms, CPU 62.1 ms). |
| What is fastest? | **PIPELINE_FE: 21.2 FPS (47.1 ms/frame), 1.47× faster.** Despite the arm's name (`gpu_pipe`), it extracts ORB **on the CPU** (see §4.3). |
| Real-time at 30 FPS? | **No arm reaches it.** The best arm (pipe) is at 71% of the camera rate. |
| Power | 6.36–6.65 W mean and 7.7 W peak (VDD_IN). The board is **never** close to its power or thermal limits. |
| Energy per frame | GPU 0.45 J, CPU 0.44 J, **pipe 0.31 J (−30%)** |
| RAM | ~1.8 GB peak of 7.6 GB. Swap stays at about 0. |
| Thermals | Tj max 54.4 °C. No throttling. |
| Stability | **Only 9 of 45 runs exited cleanly.** 5 runs (11%) aborted mid-run with a Sophus NaN. The others crashed during teardown, after the trajectories were already written (§6). |

---

## 2. Test setup

| Item | Value |
|---|---|
| Program | `Examples/Stereo/stereo_euroc` (pure stereo, no IMU) under `xvfb-run` (the Pangolin viewer is drawn in **software** on the CPU) |
| Settings | `RealSense_D435.yaml`: 1250 features, 8 levels, scale 1.2, FAST 20/7, baseline 49.9 mm |
| Arms | `gpu`: CUDA ORB extractor · `cpu`: `CPU_ORB=1` (reference extractor) · `gpu_pipe`: `PIPELINE_FE=1` |
| Loop closure | DBoW2 only (the CosPlace TRT engine was not built, so `[CNN] No TensorRT engine`) |
| Telemetry | `tegrastats` at **1 s** intervals (≈250 samples per run) |
| Sequences | `room_loop1` and `room_loop2` (handheld room loops, 120 s), `rover_loopA` and `rover_loopB` (rover, 120 s), `corridor` (out-and-back) |

### How throughput was measured
`stereo_euroc` only sleeps when a frame finishes faster than the 33 ms inter-frame gap. No arm was
that fast, so the replay never slept, and **wall-clock time over the tracking phase measures
throughput directly**:

```
FPS_eff = frames_in_sequence / tracking_window_s
tracking_window = tegrastats samples from the first one with ≥2 busy cores (or GPU>5%) to the last one
```

This deliberately includes the costs of a real deployment: the Local Mapping and Loop Closing
threads, and the software-rendered viewer competing for the same 6 cores. It does **not** use the
instrumented `REGISTER_TIMES` build, so it is end-to-end throughput, not per-stage timing. The paper's
Table 7 FPS figures come from the instrumented build.

- Runs that aborted mid-sequence (the Sophus crashes) are excluded from the performance statistics.
  That leaves n = 13 / 14 / 13 runs for gpu / cpu / pipe.
- The `corridor` length is assumed to be 120 s (3600 frames), like the other recordings. Its saved
  trajectory covers only the final map, so the length can't be read from the output.

---

## 3. Headline performance (mean ± std over runs, tracking phase only)

| Metric | GPU | CPU (`CPU_ORB=1`) | Pipe (`PIPELINE_FE=1`) |
|---|---|---|---|
| **Throughput (FPS)** | 14.39 ± 0.65 | 14.39 ± 0.69 | **21.22 ± 0.38** |
| **Time per frame (ms)** | 69.5 | 69.5 | **47.1** |
| Real-time factor vs 30 FPS camera | 0.48× | 0.48× | 0.71× |
| **VDD_IN mean (W)** | 6.44 ± 0.12 | 6.36 ± 0.15 | 6.65 ± 0.13 |
| VDD_IN peak (W) | 7.03 | 6.94 | 7.31 (max 7.78) |
| VDD_CPU_GPU_CV mean (W) | 1.77 | 1.78 | 2.01 |
| VDD_SOC mean (W) | 1.66 | 1.62 | 1.64 |
| **Energy per frame (J)** | 0.449 | 0.443 | **0.313** |
| CPU load, sum of 6 cores (%) | 305 | 343 | 389 |
| CPU time per frame (core-ms) | 212 | 238 | 183 |
| GPU load, GR3D mean / p95 (%) | **21.9 / 51** | 0 / 0 | **0 / 0** |
| RAM peak (MB of 7607) | 1830 | 1850 | 1817 |
| RAM growth during tracking (MB) | 351 | 314 | 284 |
| Swap peak (MB) | 5 | 5 | 4 |
| Tj max (°C) | 52.2 | 52.1 | 52.7 |
| CPU freq, cluster 0 / 1 (MHz, mean) | 1188 / 1106 | 1116 / 1239 | 1281 / 1150 |

### Comparison with the paper (arXiv 2608.17874)

| Quantity | Paper (EuRoC, Orin Nano) | Ours (D435, Orin Nano) | Comment |
|---|---|---|---|
| Stereo tracking time, GPU vs CPU | 69.7 vs 62.1 ms (Table 8) | 69.5 vs 69.5 ms | Same conclusion: no GPU speedup at about VGA resolution |
| Stereo baseline FPS | 14.4 (Table 7, stereo-inertial) | 14.4 | Practically identical |
| Pipelined FPS | 28.0 (1.94×) | 21.2 (1.47×) | Ours is lower. Likely causes: viewer + xvfb on the CPU, CPU clocks held at ~1344 MHz by the governor, and 1250 vs 1200 features (§7) |
| Tracking power mean / peak | 6.3 / 6.9 W (MH01) | 6.4 / 7.0 W (GPU arm) | Reproduced within 0.1 W |

---

## 4. Analysis

### 4.1 Throughput: the GPU extractor does not help at 640×480

GPU and CPU arms land on **exactly the same FPS** (14.39 vs 14.39). On a per-sequence basis they are
within ±0.4 FPS of each other, which is inside the run-to-run noise:

| Sequence | GPU FPS | CPU FPS | Pipe FPS | Pipe speedup |
|---|---|---|---|---|
| room_loop1 | 13.98 | 13.78 | 21.45 | 1.53× |
| room_loop2 | 15.04 | 14.98 | 21.27 | 1.42× |
| rover_loopA | 14.45 | 14.34 | 20.88 | 1.45× |
| rover_loopB | 14.89 | 15.25 | 21.46 | 1.44× |
| corridor | 13.22 | 13.58 | 20.69 | 1.55× |

Why the GPU gains nothing:
- At 640×480 with 8 pyramid levels, ORB extraction is a small part of the per-frame cost. Most of the
  69 ms is serial tracking work on the CPU: stereo matching, local-map search and pose optimisation.
- The GPU is used in bursts only (mean GR3D 22%, p95 51%). It is idle most of each frame while the CPU
  thread runs. The kernel launches and host↔device copies cancel the extraction speedup.
- The GPU still **frees CPU time**: 212 vs 238 core-ms per frame (−11%), and the CPU arm leaves one
  core busier (core 5 at 77%, against ~52% for the GPU arm). That headroom is the GPU arm's real
  benefit, for example for running other robot software alongside.

`corridor` is the slowest sequence on every arm (~13.2–13.6 FPS). It has ~185 tracking failures and
3 map re-initialisations per run (§5), and relocalisation plus map creation are expensive.

### 4.2 The pipelined front-end is the real speedup

Pipe overlaps extraction of frame *t* with tracking of frame *t−1* and gains **+47% FPS**. It costs
**+0.21–0.29 W** and **+84 percentage points of total CPU load**. Total energy per frame still drops
**30%**, because the board finishes its work in fewer seconds.

Its CPU profile is the most even of the three arms: about 66% on every core of cluster 0. That shows
the extra parallelism is being used. No core reaches 100%, so the limit is still the critical path of
the serial tracking thread, not the total compute available.

### 4.3 "gpu_pipe" uses no GPU at all

GR3D_FREQ is **0% throughout all 15 gpu_pipe runs**, although the console prints `GPU ORB enabled`.
This follows from how it is implemented ([include/PrefetchFE.h](../include/PrefetchFE.h),
[src/Tracking.cc:626](../src/Tracking.cc#L626)): `PIPELINE_FE` builds its own worker pair of extractors
with `useGpu=false` ("CPU; own pyramids, no GPU contention"). Those workers replace the GPU extractors
for every frame. The CUDA extractors are built but never called.

Implications:
- In R6 the "GPU + PIPELINE_FE" arm is really **"CPU + PIPELINE_FE"**. The table labels in
  `REPORT.md` should say so.
- The paper's pipelined numbers (Table 7) should be read the same way, unless their build differs.
- Untested option: a GPU-backed pipe worker could combine the GPU's lower CPU use with pipelining.

### 4.4 Power

| Phase | VDD_IN |
|---|---|
| Vocabulary loading (~10 s, one core at 100%, GPU idle) | **5.22 W** (an upper bound on idle; a true idle was not recorded) |
| Tracking, GPU arm | 6.44 W (+1.22 W) |
| Tracking, CPU arm | 6.36 W (+1.14 W) |
| Tracking, pipe arm | 6.65 W (+1.43 W) |

- The GPU arm uses **~80 mW more** than the CPU arm on average (VDD_SOC +36 mW, plus the GPU share of
  CPU_GPU_CV), for the same throughput. That difference is negligible.
- The SLAM workload itself adds only about 1.1–1.4 W on top of a ~5.2 W floor. Most of the board's
  power is the SoC, memory and USB baseline, so changes to the algorithm move total power very little.
- The peak was 7.78 W (rover_loopA pipe). That is far from the Orin Nano's 15 W / 25 W budgets.

### 4.5 Memory

- **Vocabulary load:** ~+460 MB, during the first ~10 s (RAM goes from ~1.06 GB to ~1.52 GB).
- **Map growth:** 280–350 MB per 2-minute sequence on average. It scales with keyframes: the largest
  maps (corridor, 2 maps and about 520 KFs) peak at **2.2 GB**, and the room loops (about 80 KFs) at
  **1.67 GB**.
- Swap use stays at 0–9 MB. At ~150 MB per minute of mapping, an 8 GB Orin Nano could map for roughly
  **35–40 minutes** before memory pressure. That is a rough linear estimate. Map culling and loop
  closure slow the growth in revisited areas.

### 4.6 Thermals and clocks

- Tj stays between 42 and 54.4 °C in every run, with no throttling. The workload is thermally trivial
  for this board.
- **The CPU clusters ran well below their maximum.** Cluster 0 sat mostly at 1344 MHz (7283
  samples) and cluster 1 often at 729 MHz. The board was in **25W mode** (`nvpmodel -q` showed mode 1),
  which allows up to 1728 MHz. The `schedutil` governor never raised the clocks under this bursty
  load, and `jetson_clocks` was not run. Because the tracking thread is latency-bound, **pinning the
  clocks with MAXN_SUPER + `jetson_clocks` is the cheapest lever for more FPS** (§7).
  Caution: on the Orin Nano Super, `nvpmodel -m 0` is **15W**, not MAXN. Check the IDs with
  `nvpmodel -p --verbose`; MAXN_SUPER is usually `-m 2`.

---

## 5. SLAM outcomes (no ground truth)

### 5.1 GPU ≡ CPU equivalence (the main claim of the paper, on our data)

The table gives the RMS distance between SE(3)-aligned trajectories in cm. The GPU-vs-CPU distance
must be no larger than the run-to-run distance of a single arm.

| Sequence | GPU vs CPU | GPU vs GPU | CPU vs CPU | Verdict |
|---|---|---|---|---|
| room_loop1 | 0.48 | 0.47 | 0.51 | ✅ equivalent |
| room_loop2 | 0.90 | 0.98 | 0.98 | ✅ equivalent |
| rover_loopB | 2.65 | 2.38 | 3.28 | ✅ equivalent (between the two arms' own spreads) |
| rover_loopA | 5.42 | 5.68 | 3.66 | ✅ within GPU spread (only 2 valid runs per arm) |
| corridor | 4.52 | 5.56 | 5.77 | ✅ equivalent (tracking loss dominates) |

On clean sequences (the room loops), the 0.5–1 cm disagreement is **sub-centimetre over a 32–37 m
path, below 0.03%**. Switching the extractor changes the result no more than ORB-SLAM3's own
multithreaded non-determinism does. **This reproduces the paper's equivalence result on our camera.**

### 5.2 Consistency of the reconstructed path

| Sequence | Path length GPU / CPU / Pipe (m) | Loops | KFs (final map) | Tracking failures / run |
|---|---|---|---|---|
| room_loop1 | 31.8 / 31.7 / 31.8 | 1 (all 9 runs) | 79–93 | 0 |
| room_loop2 | 36.7 / 37.0 / 36.6 | 2 (all 9 runs) | 64–96 | 0 |
| rover_loopA | 21.0 / 20.4 / 21.7 | 0–1 | 300–337 | 0–91 |
| rover_loopB | 27.9 / 28.1 / 28.2 | 1–3 | 202–227 | 0–91 |
| corridor | 24.0 / 23.9 / 23.6 (last map only) | 0 | 2 maps, ≈520 total | **~185 (every run)** |

- **Room loops are clean.** Every run of every arm closed the expected loop(s), built one map, and
  agreed on the path length to within ±0.5 m (±1.5%).
- **Rover sequences produce about 3–6× more keyframes per metre** (~16 KF/m on rover_loopA, against
  ~2.6 KF/m in the rooms). A camera near the floor sees large parallax and fast-changing views, which
  keeps triggering keyframe insertion. That costs memory (§4.5) and Local Mapping CPU time.
- **Rover tracking is intermittently fragile.** Some runs show a 91-failure burst at the same spot,
  then a new map and a merge. Others pass through cleanly. The difference is run-to-run
  non-determinism, not the arm: all three arms are affected.
- **Corridor loses tracking deterministically.** Every run, on every arm, has ~185 "Fail to track
  local map", 3 map creations and 1 merge, and ends with 2 maps. That points at the recording itself
  (a low-texture stretch or a fast turn), not at the extractor. The return leg was never re-joined to
  the outbound map, so 0 loops were closed. Re-record it with slower turns and more texture.

---

## 6. Stability and run health ⚠️

| Exit | Count | When | Data usable? |
|---|---|---|---|
| Clean (0) | **9** | n/a | yes |
| Segfault (139) | 17 | after `Saving keyframe trajectory`, during teardown | yes: `kf_run.txt` is complete (KF counts match the console) |
| Bus error (135) | 8 | same place | yes |
| X error, xvfb (1) | 4 | viewer teardown (`gdk_x_error`) | yes |
| `pure virtual method called` (134) | 2 | thread teardown | yes |
| **Sophus `SO3::exp` NaN (134)** | **5** | **mid-sequence, at 83–174 s** | **no trajectory** |

Per arm: the GPU arm had 1 clean exit out of 15, the CPU arm 5 out of 15, and pipe 3 out of 15. Each arm
had 1–2 of the NaN aborts. Takeaways:

1. **Teardown crashes (31 of 45) are harmless for the results but ugly.** They come from destroying
   threads while Local Mapping, Loop Closing or the Viewer are still active, which is a known upstream
   ORB-SLAM3 problem. The GPU arm is hit slightly more often (12 of 15, against 9 for CPU and 10 for
   pipe), so CUDA teardown order may contribute, but the sample is too small to be sure. For a robot
   this matters, because shutdown should be clean.
2. **The NaN aborts are a real robustness bug.** All 5 happened on the two sequences with tracking loss
   (corridor, rover_loopA), right after map re-initialisation or merge. A degenerate pose feeds NaN into
   `SO3::exp`, and Sophus' `ensure` aborts the whole process. In a live system that would kill
   localisation. It affects all arms, so it is not GPU-specific. Fix: validate poses (`isfinite`)
   before `SO3::exp` in the merge and relocalisation paths, and reject the update instead of aborting.

---

## 7. Recommendations, ranked by expected impact on FPS

1. **Max clocks:** switch to MAXN_SUPER (`sudo nvpmodel -m 2` on the Orin Nano Super; check the ID
   first) and run `sudo jetson_clocks` before benchmarking. The CPU mostly ran at 1344 MHz, and tracking is CPU-latency-bound. Expect a
   +10–25% gain at a small power cost.
2. **Use `PIPELINE_FE=1` in deployment.** It is +47% FPS and −30% energy per frame. It is the only arm
   that comes close to real time.
3. **Drop the viewer** for headless robots, or compare against a viewer-less run. xvfb renders in
   software and takes CPU time from tracking. That likely explains part of the gap to the paper's
   28 FPS.
4. **Reduce the frame budget** if 30 FPS live is required: drop to 15 FPS input (both GPU and CPU
   already manage that), or lower `nFeatures` from 1250 to 1000 and the pyramid from 8 to 6 levels.
   Expect 30 FPS only with pipelining plus these changes.
5. **Fix the NaN abort** (§6) before any field use, and fix teardown order for clean exits.
6. **Re-record `corridor`** with slower motion, and add an extra rover loop with deliberate revisits.
7. **Complete the measurements:** a true 60 s idle baseline (`run_matrix.py power` or plain
   `tegrastats`), the `instrumented` suite on one D435 sequence for per-stage ms (ORB vs stereo
   matching vs local map), and the CosPlace TRT engine for CNN loop closure.

---

## 8. Known limitations of this analysis

- **No ground truth**, so the ATE cannot be computed. Accuracy is argued through equivalence (§5.1) and
  cross-arm consistency (§5.2), the same way the paper argues it.
- **No per-stage timing.** The standard build has no `REGISTER_TIMES`, and `stereo_euroc` does not print
  tracking times. FPS is end-to-end wall-clock throughput (§2).
- **tegrastats at 1 s** misses millisecond spikes, so the peak power is a lower bound. GR3D is a
  utilisation percentage, not a frequency.
- **No idle baseline.** The 5.22 W "load phase" figure has one core busy.
- Small n: 2–3 valid runs per cell, and only 1 for corridor/pipe.
