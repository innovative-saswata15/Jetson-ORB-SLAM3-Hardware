# Results: Husky recording `husky_20261023`

Jetson-ORB-SLAM3 on a Jetson Orin Nano, evaluated against ground truth. The full write-up is
[`eval/report/report.pdf`](../../report/report.pdf); this page summarises its numbers.

## Setup

| | |
|---|---|
| Data | Clearpath Husky, ROS 2 recording: 65.7 m route in 355 s |
| SLAM input | Intel RealSense D435 infrared stereo, 640×480 at 15 FPS (5,326 frame pairs) |
| Ground truth | Fused from the robot's lidar, IMU and wheel encoders: [`reference/`](reference/README.md) (`reference/sequence/gt.csv`) |
| Board | Jetson Orin Nano 8 GB (7.6 GB usable), MAXN_SUPER + `jetson_clocks` |
| Versions | **GPU** (default) · **CPU** (`CPU_ORB=1`, original extractor) · **CPU, parallel** (`PIPELINE_FE=1`: the next frame's features are extracted on a second CPU thread while the current frame is tracked; no GPU) |
| Protocol | 5 replays per version at the camera rate; per-stage timing from a `REGISTER_TIMES` build (1 run per version); `tegrastats` at 1 Hz during every run |
| Accuracy metric | ATE: RMS position error after Sim(3) alignment. A constant scale factor of about 1.07 appears in every run of every version; the alignment removes it |

## Accuracy

| Version | ATE [cm], median (range) | Drift over 10 m | Route tracked |
|---|---|---|---|
| GPU | 8.4 (7.8–9.0) | 1.2 % | 100 %, one map |
| CPU | 7.9 (7.5–8.8) | 1.2 % | 100 %, one map |
| CPU, parallel | 8.3 (7.8–8.6) | 1.2 % | 100 %, one map |

- **GPU = CPU.** Two CPU runs end up 2.1 cm apart (RMS distance between the aligned trajectories), two GPU
  runs 2.6 cm, and a GPU run and a CPU run 2.5 cm: swapping the extractor changes the result no more than
  running the same code again.
- **The error depends on the scene.** The GPU and CPU error curves along the route correlate at 0.97, as
  closely as two runs of the same code (0.91–0.95).

## Speed and resources

| | GPU | CPU | CPU, parallel |
|---|---|---|---|
| Tracking time per frame | 43.4 ms | 51.7 ms | 27.0 ms |
| Tracking rate / camera rate | 1.5× | 1.3× | 2.5× |
| ORB extraction per frame | 17.9 ms | 26.6 ms | hidden behind tracking |
| Peak CPU (of 6 cores) | 57 % | 60 % | 61 % |
| Peak memory (of 7.6 GB) | 31 % | 31 % | 32 % |
| Peak GPU load | 31 % | 0 % | 0 % |
| Peak / average power (module input) | 9.9 / 9.2 W | 9.9 / 9.2 W | 9.9 / 9.2 W |

Peaks are the highest 1 s sample of each run, median over runs. The idle board draws 6.6 W. These numbers
include ORB-SLAM3's 3D viewer, which all three versions rendered off-screen.

## Ablation (one change at a time from the stereo GPU baseline)

| Change | ATE [cm] | Tracking rate / input rate | SLAM power |
|---|---|---|---|
| Baseline: stereo, GPU | 8.4 | 1.5× | 2.6 W |
| Parallel extraction (CPU) | 8.3 | 2.5× | −1 % |
| CPU feature extraction | 7.9 | 1.3× | −2 % |
| Half the frame rate (7.5 FPS) | 7.7 | 3.2× | −29 % ‡ |
| 800 instead of 1250 features | 8.2 | 2.1× | −22 % |
| Depth camera instead of stereo (colour + depth) | 9.0 | – | −4 % |
| Depth camera instead of stereo (IR + depth) | 12.3 | – | ±0 % |
| Add the IMU | 21.9 | 1.6× | −30 % † |
| Add the IMU, with adaptive features | 20.6 | 2.6× | −51 % † |
| One camera + IMU (2 of 3 runs) | 40.3 | – | −25 % † |
| One camera instead of two | 213 | 3.2× | −35 % † |

SLAM power is the module's power above the 6.6 W idle board. Three runs per configuration (the first three
rows: five). "–": not timed.
† These runs use ORB-SLAM3 example programs that do not open the 3D viewer, while the baseline renders it,
so their power is not directly comparable with the baseline.
‡ The viewer redraws at the frame rate, so part of this saving is the viewer's.

Every configuration, with all its metrics, is in [`ablation_cells.json`](ablation_cells.json).
