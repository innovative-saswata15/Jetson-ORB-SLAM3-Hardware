# Jetson-ORB-SLAM3 – reproduction of arXiv:2608.17874 on our hardware

Generated 2026-10-06 23:04 from `/home/orb-slam3/repro_results`. Build f06cee0, # R36 (release), REVISION: 5.2, GCID: 46426093, BOARD: generic, EABI: aarch64, DATE: Thu Jul 16 18:56:22 UTC 2026.
Total runs: 45. Values shown as *paper / ours* or in separate columns; '–' = not run yet.

## Table 1 – EuRoC stereo-inertial ATE (cm, SE(3)), GPU vs CPU
Per-sequence **median over runs** of the keyframe-trajectory ATE (the trajectory the repository's own scorer, run_euroc.sh, uses). Paper: median of 5 runs.

| Seq | Paper ORB-SLAM3 (desktop) | Paper GPU | Paper CPU | Paper Δ (abs) | Ours GPU (runs ok) | Ours CPU (runs ok) | Ours Δ (abs) |
|---|---|---|---|---|---|---|---|
| MH01 | 3.50 | 4.09 | 3.60 | 0.49 | – (0/0) | – (0/0) | – |
| MH02 | 2.90 | 3.45 | 3.46 | 0.01 | – (0/0) | – (0/0) | – |
| MH03 | 2.50 | 2.84 | 2.76 | 0.08 | – (0/0) | – (0/0) | – |
| MH04 | 3.10 | 4.57 | 4.75 | 0.18 | – (0/0) | – (0/0) | – |
| MH05 | 2.80 | 5.96 | 6.26 | 0.30 | – (0/0) | – (0/0) | – |
| V101 | 3.50 | 3.74 | 3.75 | 0.01 | – (0/0) | – (0/0) | – |
| V102 | 1.10 | 1.51 | 1.45 | 0.06 | – (0/0) | – (0/0) | – |
| V103 | 2.10 | 2.52 | 2.38 | 0.14 | – (0/0) | – (0/0) | – |
| V201 | 2.70 | 3.59 | 3.34 | 0.25 | – (0/0) | – (0/0) | – |
| V202 | 1.60 | 1.31 | 1.40 | 0.09 | – (0/0) | – (0/0) | – |
| V203 | 1.40 | 2.89 | 4.24 | 1.35 | – (0/0) | – (0/0) | – |
| **Mean** | 2.47 | 3.32 | 3.40 | 0.27 | – | – | – |

## Table 2 – mean EuRoC ATE over 11 sequences (cm)

| Configuration | Paper SE(3) | Paper scaled | Ours SE(3) | Ours scaled |
|---|---|---|---|---|
| Orin Nano, GPU | 3.32 | 2.75 | – | – |
| Orin Nano, CPU | 3.40 | 2.91 | – | – |

Secondary metric (every tracked frame instead of keyframes), same runs:

| Seq | GPU frames SE(3) | CPU frames SE(3) |
|---|---|---|
| MH01 | – | – |
| MH02 | – | – |
| MH03 | – | – |
| MH04 | – | – |
| MH05 | – | – |
| V101 | – | – |
| V102 | – | – |
| V103 | – | – |
| V201 | – | – |
| V202 | – | – |
| V203 | – | – |

## Table 5 – cross-dataset t_rel (%) under KITTI's estimator

| Benchmark | Windows | Paper GPU | Paper CPU | Paper Δ (abs) | Ours GPU | Ours CPU | Ours Δ (abs) |
|---|---|---|---|---|---|---|---|
| TUM-VI | 5-40 m | 0.134 | 0.117 | 0.022 | – | – | – |
| EuRoC | 5-40 m | 0.723 | 0.725 | 0.016 | – | – | – |
| KITTI | 100-800 m | 0.800 | 0.780 | 0.036 | – | – | – |

## Table 3 – TUM-VI rooms, stereo-inertial ATE (cm, SE(3))

| Room | Paper ORB-SLAM3 | Paper GPU | Paper CPU | Paper Δ (abs) | Ours GPU | Ours CPU | Ours Δ (abs) |
|---|---|---|---|---|---|---|---|
| room1 | 0.80 | 0.98 | 0.78 | 0.20 | – | – | – |
| room2 | 1.20 | 1.08 | 0.83 | 0.25 | – | – | – |
| room3 | 1.10 | 0.84 | 0.59 | 0.25 | – | – | – |
| room4 | 0.80 | 0.67 | 0.76 | 0.09 | – | – | – |
| room5 | 1.00 | 0.80 | 0.74 | 0.06 | – | – | – |
| room6 | 0.60 | 0.86 | 0.62 | 0.24 | – | – | – |
| **Mean** | 0.92 | 0.87 | 0.72 | 0.18 | – | – | – |

## Table 4 – KITTI stereo: t_rel (%), r_rel (°/100 m), ATE (m)

| Seq | Paper t_rel GPU | Paper t_rel CPU | Ours t_rel GPU | Ours t_rel CPU | Ours Δ (abs) | Paper r_rel | Ours r_rel | Paper ATE | Ours ATE | note |
|---|---|---|---|---|---|---|---|---|---|---|
| 00 | 0.72 | 0.68 | – | – | – | 0.28 | – | 1.31 | – |  |
| 01 | 1.64 | 1.65 | – | – | – | 0.28 | – | 13.15 | – |  |
| 02 | 0.77 | 0.75 | – | – | – | 0.26 | – | 6.89 | – |  |
| 03 | 0.93 | 0.94 | – | – | – | 0.20 | – | 1.31 | – |  |
| 04 | 0.56 | 0.53 | – | – | – | 0.18 | – | 0.25 | – |  |
| 05 | 0.58 | 0.41 | – | – | – | 0.19 | – | 1.30 | – |  |
| 06 | 0.51 | 0.55 | – | – | – | 0.19 | – | 0.75 | – |  |
| 07 | 0.48 | 0.47 | – | – | – | 0.29 | – | 0.48 | – |  |
| 08 | 1.02 | 1.01 | – | – | – | 0.30 | – | 3.35 | – |  |
| 09 | 0.90 | 0.93 | – | – | – | 0.28 | – | 1.69 | – |  |
| 10 | 0.66 | 0.64 | – | – | – | 0.30 | – | 1.14 | – |  |
| **Mean** | 0.80 | 0.78 | – | – | – | 0.25 | – | 2.87 | – |  |

## Table 6 – per-stage timing, instrumented (REGISTER_TIMES) build

| Stage | Paper mono-inertial V101 | Ours | Paper stereo-inertial MH01 | Ours |
|---|---|---|---|---|
| ORB extraction (ms) | 13.7 | – | 38.9 | – |
| Total tracking (ms/frame) | 35.4 | – | 76.6 | – |
| Throughput (FPS) | 28.3 | – | 13.1 | – |

## Table 7 – throughput (FPS = 1 / mean tracking time), paper / ours

EuRoC is recorded at 20 FPS; Pipe = PIPELINE_FE=1.

| Seq | Mono-inertial Base | Mono-inertial Pipe | Stereo-inertial Base | Stereo-inertial Pipe |
|---|---|---|---|---|
| MH01 | 25.6 / – | 30.7 / – | 13.5 / – | 26.2 / – |
| MH02 | 26.4 / – | 26.3 / – | 13.8 / – | 27.3 / – |
| MH03 | 31.0 / – | 30.5 / – | 13.8 / – | 27.6 / – |
| MH04 | 33.5 / – | 32.5 / – | 14.7 / – | 28.7 / – |
| MH05 | 32.4 / – | 30.5 / – | 14.6 / – | 28.5 / – |
| V101 | 28.7 / – | 28.7 / – | 13.6 / – | 25.4 / – |
| V102 | 34.6 / – | 34.2 / – | 14.5 / – | 28.2 / – |
| V103 | 35.2 / – | 38.1 / – | 15.6 / – | 30.1 / – |
| V201 | 33.2 / – | 34.4 / – | 14.4 / – | 27.7 / – |
| V202 | 34.1 / – | 34.0 / – | 14.1 / – | 26.8 / – |
| V203 | 37.6 / – | 38.3 / – | 15.6 / – | 31.6 / – |
| **Mean** | 32.0 / – | 32.6 / – | 14.4 / – | 28.0 / – |

## Table 8 – mean tracking time (ms), GPU vs CPU reference

KITTI and TUM-VI programs always open the viewer (drawn in software under xvfb), on both arms.

| Benchmark | Resolution | Paper GPU | Paper CPU | Ours GPU | Ours CPU |
|---|---|---|---|---|---|
| KITTI (stereo) | 1241x376 | 98.8 | 111.7 | – | – |
| EuRoC (stereo-inertial) | 752x480 | 69.7 | 62.1 | – | – |
| TUM-VI (stereo-inertial) | 512x512 | 78.4 | 78.3 | – | – |

## Table 9 – CosPlace ResNet-50 latency per query (ms)

| Execution path | Paper |
|---|---|
| ONNX-Runtime CUDA/TRT EP | fails to initialize |
| ONNX-Runtime CPU | ~396 |
| TensorRT FP16 GPU | 2.2 |

Ours: not measured yet (runbook Part R3, step 4).

## Table 10 – loop-closure ablation, median ATE (cm), paper / ours

| Seq | DBoW2 + CNN | DBoW2 only | no loop closing |
|---|---|---|---|
| MH01 | 3.91 / – | 4.21 / – | 3.72 / – |
| MH02 | 3.18 / – | 3.24 / – | 3.61 / – |
| MH03 | 2.89 / – | 2.74 / – | 2.82 / – |
| MH04 | 4.64 / – | 4.75 / – | 4.63 / – |
| MH05 | 5.41 / – | 5.53 / – | 5.94 / – |
| V101 | 3.61 / – | 3.64 / – | 3.77 / – |
| V102 | 1.49 / – | 1.37 / – | 1.46 / – |
| V103 | 2.57 / – | 2.34 / – | 2.58 / – |
| V201 | 3.54 / – | 3.00 / – | 3.59 / – |
| V202 | 1.29 / – | 1.29 / – | 2.61 / – |
| V203 | 5.87 / – | 5.35 / – | 4.92 / – |
| **Mean (no V203)** | 3.49 / – | 3.41 / – | 3.60 / – |

## Power (Sec. 4.1) – VDD_IN, stereo-inertial MH01, tegrastats @ 250 ms (W)

|  | Paper | Ours |
|---|---|---|
| Idle | 4.7 | – |
| Tracking mean | 6.3 | – |
| Tracking peak | 6.9 | – |

## Feature-level equivalence (Sec. 3.1.5)

Not measured yet (runbook Part R4, features).

## Our hardware: D435 stereo recordings (no ground truth)

Per arm: mean tracking time / FPS / loop closures / maps (medians over runs).

| Sequence | GPU | CPU (CPU_ORB=1) | GPU + PIPELINE_FE |
|---|---|---|---|
| corridor | – ms / – FPS / loops 0 / maps 2 | – ms / – FPS / loops 0 / maps 2 | – ms / – FPS / loops 0 / maps 2 |
| room_loop1 | – ms / – FPS / loops 1 / maps 1 | – ms / – FPS / loops 1 / maps 1 | – ms / – FPS / loops 1 / maps 1 |
| room_loop2 | – ms / – FPS / loops 2 / maps 1 | – ms / – FPS / loops 2 / maps 1 | – ms / – FPS / loops 2 / maps 1 |
| rover_loopA | – ms / – FPS / loops 1 / maps 1 | – ms / – FPS / loops 0 / maps 1 | – ms / – FPS / loops 1 / maps 1 |
| rover_loopB | – ms / – FPS / loops 2 / maps 1 | – ms / – FPS / loops 3 / maps 1 | – ms / – FPS / loops 2 / maps 1 |

Equivalence on our data: RMS distance (cm) between trajectories after SE(3) alignment. GPU-vs-CPU should be no larger than the run-to-run spread of either arm alone.

| Sequence | GPU vs CPU | GPU vs GPU (run-to-run) | CPU vs CPU (run-to-run) |
|---|---|---|---|
| corridor | 4.52 (6 pairs) | 5.56 (2) | 5.77 (6) |
| room_loop1 | 0.48 (9 pairs) | 0.47 (6) | 0.51 (6) |
| room_loop2 | 0.90 (9 pairs) | 0.98 (6) | 0.98 (6) |
| rover_loopA | 5.42 (4 pairs) | 5.68 (2) | 3.66 (2) |
| rover_loopB | 2.65 (9 pairs) | 2.38 (6) | 3.28 (6) |

## Run health (runs without a trajectory / ending with more than one map)

| Suite | Mode | Seq | Arm | No trajectory | >1 map |
|---|---|---|---|---|---|
| d435 | d435_s | corridor | cpu | 0/3 | 3/3 |
| d435 | d435_s | corridor | gpu | 1/3 | 2/3 |
| d435 | d435_s | corridor | gpu_pipe | 2/3 | 1/3 |
| d435 | d435_s | rover_loopA | cpu | 1/3 | 0/3 |
| d435 | d435_s | rover_loopA | gpu | 1/3 | 0/3 |
