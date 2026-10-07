# Jetson ORB-SLAM3 on our D435: second campaign (MAXN_SUPER + CNN loop closure)

**Platform:** Jetson Orin Nano 8 GB "Super", L4T R36.5.2, build `f06cee0` (same commit as campaign 1)
**Input:** the same 5 D435 IR-stereo recordings (640×480 @ 30 FPS), replayed offline
**Matrix:** 5 sequences × 3 arms (`gpu`, `cpu`, `gpu_pipe`) × 3 runs = **45 runs**, 2026-10-07 00:06–~03:00
**Raw data:** [repro_results_maxn/](repro_results_maxn/)
**Baseline for comparison:** campaign 1 in [../jetson-orbslam3-results/](../jetson-orbslam3-results/D435_PERFORMANCE_REPORT.md) (25W mode, DBoW2 loop closure)

The method is the same as in campaign 1: tegrastats at 1 s intervals, console events, and trajectory
files. Throughput is end-to-end wall-clock FPS over the tracking phase (frames ÷ seconds of tracking).
The replay never sleeps, because no arm keeps up with 30 FPS.

---

## 0. What changed between the campaigns, and what didn't

| | Campaign 1 | Campaign 2 (this report) |
|---|---|---|
| Power mode | 25W (mode 1) | **MAXN_SUPER** (mode 2) |
| `jetson_clocks` | off | **effectively off.** The CPU clocks still scale: 1728 MHz is the most common value (30 932 samples), but 729–1190 MHz also appears more than 15 000 times. Pinned clocks would show a constant 1728. |
| Loop closure | DBoW2 only (no engine) | **DBoW2 + CosPlace CNN.** `engine_linked: true` in all 45 runs, and the console shows `[CNN] EP: TensorRT FP16` |
| Code, settings, recordings | `f06cee0`, `RealSense_D435.yaml` | identical |
| Idle baseline | none | the `idle/tegrastats.log` file is **not a valid idle**. See §7. |
| Instrumented (per-stage) runs | none | none in this folder |

⚠️ **Two variables changed at once: the clocks and CNN loop closure.** The CNN runs once per keyframe
on the GPU, so it costs at most a few ms per second of tracking (§4.3). The throughput change can
therefore be attributed almost entirely to the clocks. Memory and power changes, however, are partly
the CNN's (§4.4).

---

## 1. TL;DR

| Question | Answer |
|---|---|
| Did MAXN_SUPER make it faster? | **Yes. Every arm, every sequence.** GPU +12.5%, CPU **+23.8%**, pipe +6.7% |
| Best throughput | **Pipe: 22.6 FPS (44.2 ms/frame)**, 75% of real time. It's still not 30 FPS. |
| Surprise | **The CPU arm now beats the GPU arm: 17.8 vs 16.2 FPS** (+10%). The CUDA extractor gains less from CPU clocks than the CPU extractor does. |
| GPU ≡ CPU trajectories? | **Still yes.** GPU-vs-CPU stays within the run-to-run noise on all 5 sequences. |
| Did CNN loop closure change the trajectories? | **No measurable change.** New-vs-old distances are within the run-to-run noise, and the loop counts are the same. |
| Power | Mean +0.8 W on every arm (7.18–7.42 W). Peak 9.09 W. |
| Energy per frame | CPU 0.40 J (−9%), GPU 0.45 J (unchanged), pipe 0.33 J (+5%) |
| RAM | **About +450 MB baseline** (TensorRT/CUDA context for the CNN engine), peak 2.4 GB on average and 3.3 GB at most. Swap 60 MB. |
| Thermals | Tj max 57.1 °C (was 54.4). Still no throttling. |
| Stability | 5 clean exits out of 45 (was 9). **Sophus NaN aborts: 2** (was 5). Teardown crashes are as before. |

---

## 2. Headline performance (mean ± std over runs, tracking phase; Sophus-aborted runs excluded)

n = 14 / 15 / 14 valid runs for gpu / cpu / pipe.

| Metric | GPU | CPU (`CPU_ORB=1`) | Pipe (`PIPELINE_FE=1`) |
|---|---|---|---|
| **Throughput (FPS)** | 16.19 ± 0.76 | **17.82 ± 0.87** | **22.63 ± 0.18** |
| **Time per frame (ms)** | 61.8 | 56.1 | 44.2 |
| Real-time factor vs 30 FPS | 0.54× | 0.59× | 0.75× |
| **VDD_IN mean (W)** | 7.26 ± 0.22 | 7.18 ± 0.24 | 7.42 ± 0.29 |
| VDD_IN peak (W), mean of runs | 8.08 | 8.04 | 8.36 (max 9.09) |
| VDD_CPU_GPU_CV mean (W) | 2.53 | 2.57 | 2.76 |
| VDD_SOC mean (W) | 1.65 | 1.60 | 1.63 |
| **Energy per frame (J)** | 0.450 | **0.404** | **0.328** |
| CPU load, sum of 6 cores (%) | 300 | 332 | 367 |
| CPU time per frame (core-ms) | 186 | 187 | 162 |
| GR3D mean / p95 (%) | 23.2 / 47 | 0.9 / 4.6 | 1.0 / 6.2 |
| Share of samples with GPU busy | 82% | 3.4% | 4.0% |
| RAM at start of tracking (MB) | 1994 | 1931 | 1954 |
| RAM peak (MB of 7607) | 2414 | 2370 | 2392 |
| RAM growth while tracking (MB) | 420 | 439 | 438 |
| Swap peak (MB) | 61 | 63 | 61 |
| Tj max (°C), mean of runs | 55.5 | 55.4 | 55.7 |
| CPU freq cluster 0 / 1 (MHz, mean) | 1486 / 1339 | 1454 / 1387 | 1581 / 1356 |

---

## 3. Campaign 1 → campaign 2

### 3.1 Throughput

| Sequence | GPU FPS | CPU FPS | Pipe FPS |
|---|---|---|---|
| room_loop1 | 13.98 → **15.92** | 13.78 → **17.16** | 21.45 → **22.86** |
| room_loop2 | 15.04 → **16.88** | 14.98 → **18.78** | 21.27 → **22.78** |
| rover_loopA | 14.45 → **16.32** | 14.34 → **17.96** | 20.88 → **22.49** |
| rover_loopB | 14.89 → **16.92** | 15.25 → **18.64** | 21.46 → **22.58** |
| corridor | 13.22 → **14.96** | 13.58 → **16.54** | 20.69 → **22.41** |
| **Mean** | 14.39 → **16.19 (+12.5%)** | 14.39 → **17.82 (+23.8%)** | 21.22 → **22.63 (+6.7%)** |

Mean wall time per run: GPU 244 → 233 s, CPU 251 → 212 s, pipe 178 → 169 s.

### 3.2 Everything else (arm means)

| Metric | GPU old → new | CPU old → new | Pipe old → new |
|---|---|---|---|
| ms/frame | 69.5 → 61.8 | 69.5 → 56.1 | 47.1 → 44.2 |
| VDD_IN mean (W) | 6.44 → 7.26 | 6.36 → 7.18 | 6.65 → 7.42 |
| VDD_IN peak (W) | 7.03 → 8.08 | 6.94 → 8.04 | 7.31 → 8.36 |
| VDD_CPU_GPU_CV (W) | 1.77 → 2.53 | 1.78 → 2.57 | 2.01 → 2.76 |
| VDD_SOC (W) | 1.66 → 1.65 | 1.62 → 1.60 | 1.64 → 1.63 |
| Energy/frame (J) | 0.449 → 0.450 | 0.443 → **0.404** | 0.313 → 0.328 |
| Energy per run (J) | 1614 → 1619 | 1595 → **1456** | 1128 → 1180 |
| CPU load (%) | 305 → 300 | 343 → 332 | 389 → 367 |
| CPU core-ms per frame | 213 → 186 | 239 → 187 | 183 → 162 |
| Cluster 0 freq (MHz) | 1188 → 1486 | 1116 → 1454 | 1281 → 1581 |
| RAM peak (MB) | 1830 → 2414 | 1850 → 2370 | 1817 → 2392 |
| Swap peak (MB) | 5 → 61 | 5 → 63 | 4 → 61 |
| Tj max (°C) | 52.2 → 55.5 | 52.1 → 55.4 | 52.7 → 55.7 |
| Vocabulary-load phase (W, one core busy) | 5.21 → 5.79 | 5.22 → 5.78 | 5.22 → 5.79 |
| Vocabulary-load duration (s) | ~10 → ~8 | | |

---

## 4. Analysis

### 4.1 The CPU extractor now outruns the GPU extractor

In campaign 1 the two arms were tied at 14.39 FPS. Now the CPU arm is **10% faster on every sequence**:
- room_loop2: 18.78 vs 16.88
- corridor: 16.54 vs 14.96

Why the GPU arm gained only half as much:
- **The CPU arm scales almost linearly with clock.** Its cluster-0 clock went up 30% (1116 → 1454 MHz)
  and its FPS went up 24%. Its whole per-frame path is CPU code.
- **The GPU arm has a part that doesn't scale with CPU clock.** It launches kernels, copies the image
  pyramid between host and device, and synchronises before tracking can continue. Its CPU clock went up
  25%, but its FPS only 12.5%. Its GPU is busy in 82% of samples (up from 72%) at a lower p95 (47%),
  so the GPU is now a relatively larger share of each frame. The GPU clock itself isn't logged
  (tegrastats ran without sudo, so it shows no GR3D frequency). Without `jetson_clocks` it may not
  have been at the MAXN_SUPER maximum (1020 MHz).
- **The GPU arm no longer saves CPU time.** Per frame it costs 186 core-ms, against 187 for the CPU arm
  (in campaign 1: 213 vs 239). It still uses less total CPU (300% vs 332%), but only because it
  processes fewer frames per second.

**Implication:** at 640×480 on this board, the CUDA ORB extractor is now a net loss in throughput and
gives no CPU saving per frame. Its remaining argument is lower total CPU occupancy when frame rate is
capped, for example at a fixed 15 FPS input (not measured).

### 4.2 Pipe barely moved, and it is now the most consistent arm

Pipe gained only +6.7%, and its FPS sits in a very narrow band: **σ = 0.18 FPS** across all 14 runs and
5 sequences (22.41–22.86). That is a sign of a hard limit independent of the content. The candidates:
- **The serial tracking stage.** With extraction overlapped, the frame time becomes
  max(extraction, tracking), and the tracking half gained less from clocks.
- **CPU contention.** Pipe uses 367% of 600%, while Local Mapping, Loop Closing, the CNN and the
  software-rendered viewer share the same cores. Cluster 1 still averages only ~1356 MHz.
- The ~23 FPS ceiling is consistent with the paper's stereo-inertial pipe figure (28 FPS on EuRoC
  752×480, viewer off). The viewer and xvfb overhead remain the most likely difference.

Per-stage timing (the instrumented build) is the only way to tell these apart (§8).

### 4.3 CNN loop closure: visible on the GPU, invisible in the results

- **The GPU is no longer completely idle in the CPU and pipe arms.** GR3D averages 0.9–1.0%, with
  bursts up to 20–30%, busy in 3–4% of samples. These are CosPlace queries, one per keyframe at about
  3.2 ms each.
  - The bursts are largest on `corridor` (p95 11–25%), the sequence with the most keyframes (~520).
  - They are smallest on the room loops (p95 0%, ~80 KFs).
- **Loop counts are unchanged.** room_loop1 has 1 loop and room_loop2 has 2 loops in every run (same as
  DBoW2-only). rover_loopB has 1–3, the same range as before. corridor still closes 0 loops.
- **Trajectories are unchanged.** The distance between campaign-2 and campaign-1 runs of the same arm
  is within the run-to-run noise (table in §5.1).
- **Conclusion:** on these short indoor recordings, DBoW2 already finds every loop the CNN finds. The
  CNN adds GPU work and memory but no benefit here. Its value would show on long or revisited
  routes, or under appearance change, which these recordings don't test.

### 4.4 Power

- Every arm costs **~0.8 W more** on average. Nearly all of it is on VDD_CPU_GPU_CV (+0.75 W):
  higher CPU clocks draw more power. VDD_SOC is unchanged.
- The vocabulary-load phase (one core busy) rose from 5.22 to 5.79 W. That floor rises with the
  higher clocks too.
- The SLAM workload's own cost above that floor: GPU +1.47 W, CPU +1.40 W, pipe +1.63 W.
- **Peak 9.09 W** (rover_loopA pipe run 1). That is far below the MAXN_SUPER budget.
- **Energy per frame:**
  - The CPU arm *improved* (0.443 → 0.404 J). It finishes 24% faster for 13% more power.
  - The GPU arm is flat (0.449 → 0.450 J).
  - Pipe is slightly worse (0.313 → 0.328 J). It drew +12% power for only +7% speed.
  - Pipe is still the most efficient arm by 19–27%.

### 4.5 Memory

- **Baseline +450 MB.** RAM at the start of tracking is ~1.95 GB, against ~1.5 GB before, on all arms.
  Linking the TensorRT engine loads the CUDA/TensorRT runtime and the 47 MB engine into every process,
  including the CPU arm.
- Map growth while tracking is 420–440 MB per 2-minute run (was 280–350). Part of that is run-to-run
  variation in page-cache state, so the start-of-run RAM ranges from 1.1 to 1.9 GB.
- The peak was **3.30 GB** (corridor GPU run 1), still under half of 7.6 GB.
- Swap is now ~60 MB in every run (was ~5 MB). That is a small amount, probably idle pages from
  earlier runs pushed out, but it is worth watching on longer runs.

### 4.6 Thermals and clocks

- Tj max 57.1 °C, with a tracking average around 55 °C (+3 °C). No throttling.
- Cluster 0 now reaches 1728 MHz, but still drops between frames: the mean is 1450–1580 MHz. Cluster
  1 averages 1340–1390 MHz.
- `jetson_clocks` (pinning everything at max) is the remaining unused lever. Expect a further gain on
  the CPU and GPU arms, which still drop clocks between frames.

---

## 5. SLAM outcomes

### 5.1 Equivalence: median RMS distance between SE(3)-aligned `f_run` trajectories (cm, pairs in brackets)

| Sequence | GPU vs CPU | GPU vs GPU | CPU vs CPU | Pipe vs Pipe | GPU vs Pipe | Campaign-1 GPU vs CPU | New vs old, GPU / CPU |
|---|---|---|---|---|---|---|---|
| room_loop1 | **0.48** (9) | 0.44 (6) | 0.55 (6) | 0.78 (6) | 0.60 (9) | 0.48 | 0.45 / 0.50 |
| room_loop2 | **0.72** (9) | 0.73 (6) | 0.84 (6) | 1.34 (6) | 1.14 (9) | 0.90 | 0.82 / 0.81 |
| rover_loopA | **4.94** (6) | 3.51 (2) | 5.45 (6) | 2.14 (2) | 5.27 (4) | 5.42 | 4.38 / 5.95 |
| rover_loopB | **2.75** (9) | 3.52 (6) | 2.14 (6) | 2.01 (6) | 2.75 (9) | 2.65 | 2.72 / 2.16 |
| corridor | **2.73** (9) | 3.20 (6) | 4.76 (6) | 9.20 (6) | 9.54 (9) | 4.52 | 3.45 / 2.89 |

- **GPU ≡ CPU holds again.** On every sequence, GPU-vs-CPU falls between the two arms' own spreads.
  The room loops agree to 0.5–0.7 cm over 32–37 m.
- **Configuration changes don't move the trajectory.** Campaign 2 vs campaign 1 (higher clocks, CNN
  loop closure) is also within the noise.
- **Pipe is a bit noisier.** On room_loop2 and especially corridor (9.2 cm between pipe runs), pipe runs
  disagree more with each other.
  - It processes frames with one frame of lag, which changes the timing of keyframe decisions relative to
    Local Mapping.
  - On corridor this interacts with the tracking-loss segment.
  - In campaign 1 the pipe spread was comparable on the rooms (1.07 / 0.64 cm), and the corridor
    comparison wasn't available (only 1 valid run).

### 5.2 Per-sequence behaviour (essentially identical to campaign 1)

| Sequence | Path length (m) | Loops | KFs, final map | "Fail to track" per run | Maps |
|---|---|---|---|---|---|
| room_loop1 | 31.7–32.4 | 1 (all runs) | 77–91 | 0 | 1 |
| room_loop2 | 36.5–38.3 | 2 (all runs) | 56–83 | 0 | 1 |
| rover_loopA | 20.5–21.4 | 0–1 | 306–343 | 0–2, or 91 | 1 (2 created when tracking is lost) |
| rover_loopB | 28.1–28.9 | 1–3 | 196–225 | 0–2, or 91 | 1 |
| corridor | 23.5–24.4 (last map) | 0 | 2 maps, ~510–540 total | **182–185 (all runs)** | 2 |

- The **91-failure burst** on the rover sequences appears in 4 runs (it was 5). Each time it happens at
  the same place and is followed by a new map and a merge. It is still random across arms.
- **corridor** reproduces its loss exactly: 182–185 failures, 3 maps created, 1 merge, ending with 2
  maps on all 9 runs. That confirms again that the problem is in the recording, not the configuration.
- The rover keyframe density (~16 KF/m against ~2.6 KF/m in the rooms) is unchanged.

---

## 6. Stability

| Exit | Campaign 1 | Campaign 2 | When | Trajectory usable? |
|---|---|---|---|---|
| Clean (0) | 9 | **5** | n/a | yes |
| Segfault (139) | 17 | 21 | during teardown, after `Saving keyframe trajectory` | yes |
| Bus error (135) | 8 | 7 | same place | yes |
| X error, xvfb (1) | 4 | **9** | viewer teardown | yes |
| pure virtual (134) | 2 | 1 | thread teardown | yes |
| **Sophus `SO3::exp` NaN (134)** | **5** | **2** | **mid-run** | **no** |

- **NaN aborts (2):** rover_loopA gpu run 3 at 69 s, and rover_loopA gpu_pipe run 3 at 59 s. Both
  follow the rover tracking-loss event, as in campaign 1. This time corridor had none, although it lost
  tracking the same way. The bug is intermittent and not specific to an arm, and it is still unfixed.
- **Teardown crashes went up** (38 of 45, from 31). X errors more than doubled (4 → 9). The likely cause
  is faster shutdown racing the viewer thread, and the extra CUDA/TensorRT state to destroy (the CNN
  engine is now loaded in every arm). Trajectories are complete in all of them.
- Per arm, clean exits were: GPU 1 of 15, CPU 2 of 15, pipe 2 of 15.

---

## 7. Data-quality notes

- **The `idle/tegrastats.log` file is not an idle measurement.** Its timestamps are from 2026-10-06
  23:33, *before* the switch to MAXN_SUPER. It shows:
  - every CPU pinned at 1497 MHz and the GPU at 611 MHz, which is the 15W + `jetson_clocks` state;
  - **two cores at 100%** throughout, and RAM swinging between 1.1 and 3.3 GB, so something was still
    running (most likely the timing build or `trtexec`);
  - VDD_IN 6.4–6.5 W.

  It must not be used as an idle baseline. The only near-idle reference is the vocabulary-load phase,
  5.79 W with one core busy.
- **No instrumented (REGISTER_TIMES) results** are in this folder, so there is no per-stage breakdown yet.
- **No new CosPlace latency figure** is recorded here. The 3.17 ms from the 25W-mode `trtexec` is the
  latest.
- GPU frequency is not visible, because tegrastats ran without sudo.
- The `corridor` length is assumed to be 120 s (3600 frames), as in campaign 1.

---

## 8. Recommendations

1. **Re-measure idle properly:** run it right after boot in MAXN_SUPER, before `jetson_clocks`, with
   nothing else running. Use `sudo tegrastats --interval 250` so the GPU and EMC frequencies are logged.
2. **Separate the two changes:** run 1 sequence × 3 arms × 3 runs with `--engine /nonexistent` (MAXN,
   DBoW2 only). That isolates the CNN's cost in memory, power and stability. Then run 1 sequence with
   `jetson_clocks` on, to measure the remaining clock headroom.
3. **Run the instrumented build** on room_loop1 for all 3 arms (commands from the earlier session). It
   will show why the GPU arm gained only half as much as the CPU arm, and whether pipe is limited by
   tracking or by contention.
4. **For deployment today:** use `PIPELINE_FE=1` with **MAXN_SUPER**. That gives 22.6 FPS at 0.33 J per
   frame. If CPU headroom for other robot software matters more than FPS, run the CPU or GPU arm at a
   fixed 15 FPS instead.
5. **Drop CNN loop closure** for short indoor missions. It costs ~450 MB of RAM and adds teardown
   fragility, with no change to loops or trajectories on these routes. Re-evaluate it on long or
   revisited routes.
6. The NaN guard, a clean shutdown, and re-recording `corridor` remain open from campaign 1.

---

## 9. Method (for reproducing these numbers)

- **Tracking window:** tegrastats samples from the first one with ≥2 cores above 25% (or GR3D above
  5%) to the last such sample.
- **FPS** = frames in the sequence ÷ tracking-window seconds.
- **Energy** = Σ VDD_IN × 1 s over that window.
- **Equivalence:** the same as `repro/make_report.py`. `f_run.txt` trajectories are associated at
  ≤1 ms, SE(3) Umeyama-aligned, and the RMS is reported as a median over all pairs.
- Parsed with numpy, using the same logic as
  [../jetson-orbslam3-results/analyze_d435.py](../jetson-orbslam3-results/analyze_d435.py), pointed
  at `repro_results_maxn/d435/d435_s`.
- No code or result files were modified.
