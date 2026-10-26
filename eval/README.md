# Evaluation of Jetson-ORB-SLAM3 on real hardware

Everything we added on top of the original
[Jetson-ORB-SLAM3](../README.md) repository lives in this folder. **The ORB-SLAM3 library itself is
unchanged:** `src/`, `include/`, `Thirdparty/` and `Vocabulary/` are exactly as published. The only
additions outside `eval/` are:

| File | Purpose |
|---|---|
| `Examples/Stereo/stereo_realsense_D435_rover.cc` | Live driver for our rover (D435 stereo, clean exit, saved trajectories, event log) |
| `Examples/Stereo/RealSense_D435.yaml` | Calibration of our D435 (serial 827312071682) |
| `CMakeLists.txt` | +6 lines to build that driver |
| `.gitignore` | Local-only files (papers, video, notes, raw run data, report and slide sources) |

**Deliverables:** [`report/report.pdf`](report/report.pdf) (project report) and
[`presentation/presentation.pdf`](presentation/presentation.pdf) (slides).
**Results:** [`results/husky/README.md`](results/husky/README.md).

## Layout

```
eval/
├── report/report.pdf              project report
├── presentation/presentation.pdf  presentation
├── benchmark/          run on the Jetson: experiment runner, scoring
│   ├── run_matrix.py       runs a suite of ORB-SLAM3 configurations (GPU / CPU / parallel, ablation cells)
│   ├── common.py           datasets, modes (stereo, mono, inertial, RGB-D), arms (env switches)
│   ├── evaluate.py         ATE (SE(3)/Sim(3)), relative errors, power, per-stage timing per run
│   ├── make_report.py      paper-reproduction report (EuRoC / TUM-VI / KITTI / D435 tables)
│   ├── fetch_datasets.py   EuRoC, TUM-VI, KITTI downloads
│   ├── setup_timing_build.sh   instrumented (REGISTER_TIMES) build in a separate worktree
│   ├── paper_reference.py  the paper's published numbers
│   ├── cpp/                D435 recorder (EuRoC layout) and ORB feature-equivalence tool
│   └── tools/              CosPlace ONNX export
├── processing/         run on the PC: datasets, ground truth, analysis
│   ├── inspect_bag.py, bag_to_euroc.py, bag_odom.py      read a ROS 2 bag; extract IR stereo in EuRoC layout
│   ├── fuse_reference.py   ground truth from lidar + IMU + wheels (GTSAM factor graph, extrinsic checks)
│   ├── lidar_odometry.py, lidar_slam.py, reference_check.py   lidar-only cross-checks of the reference
│   ├── make_gt.py          reference -> camera / IMU / colour frame at every image time (gt.csv)
│   ├── prepare_husky.sh    the whole ground-truth pipeline for one bag
│   ├── ablation_prepare.py IMU, depth and colour extraction + the ablation variant folders
│   ├── husky_report.py, ablation_report.py, analyze_d435.py   reports from run results
│   ├── jetson_prepare.sh   install the reference next to the IR sequence on the Jetson
│   └── setup_venv.sh       Python environment (--light on the Jetson)
├── rover/              live rover runs with the D435 driver (rover_run.sh, summarize_run.py)
└── results/husky/      the Husky recording
    ├── README.md           results summary (accuracy, speed and resources, ablation)
    ├── ablation_cells.json every ablation configuration with all its metrics
    └── reference/          ground truth (sequence/gt.csv), how it was built, its cross-checks
```

Kept local and out of the repository (see `.gitignore`): the raw run outputs (trajectories,
console and tegrastats logs, per-run JSON), the generated run reports, our own D435 recording campaigns, and
the LaTeX sources, figures and figure scripts of the report and the slides.

## Main results (Husky recording, 66 m route, D435 IR stereo at 15 FPS)

| | GPU | CPU | CPU, parallel |
|---|---|---|---|
| Trajectory error (ATE, Sim(3)-aligned) | 8.4 cm | 7.9 cm | 8.3 cm |
| Tracking time per frame | 43.4 ms (1.5× camera) | 51.7 ms (1.3×) | 27.0 ms (2.5×) |
| Peak CPU / memory | 57 % / 31 % | 60 % / 31 % | 61 % / 32 % |
| Peak power (module input) | 9.9 W | 9.9 W | 9.9 W |

Details in [results/husky/README.md](results/husky/README.md).

## Running

**On the Jetson** (system Python), with a sequence in `~/datasets/ext/<name>/` (images, `times.txt`,
`settings.yaml`, `gt.csv`):

```bash
sudo nvpmodel -m 2 && sudo jetson_clocks
python3 eval/benchmark/run_matrix.py ext --engine /nonexistent --results ~/repro_results_ext
python3 eval/benchmark/run_matrix.py abl --seqs husky_20261023__si --arms gpu cpu --results ~/repro_results_abl
python3 eval/benchmark/run_matrix.py ext --runs 1 --build-root ~/Jetson-ORB-SLAM3-timing --results ~/repro_results_ext_timing
```

**On the PC** (`eval/processing/setup_venv.sh`; the full set needs Python 3.12 for kiss-icp). The run
results go under `eval/results/husky/{stage1,ablation}/` (local only):

```bash
eval/processing/prepare_husky.sh                         # bag -> IR sequence + fused ground truth
~/venvs/ext/bin/python -I eval/processing/ablation_prepare.py
H=eval/results/husky
~/venvs/ext/bin/python -I eval/processing/husky_report.py --results $H/stage1/runs --timing $H/stage1/timing \
    --seq ~/datasets/ext/husky_20261023 --reference $H/reference --out $H/stage1/report
~/venvs/ext/bin/python -I eval/processing/ablation_report.py --stage1 $H/stage1/runs --abl $H/ablation/runs \
    --data ~/datasets/ext --reference $H/reference --out $H/ablation/report \
    --stage1-timing $H/stage1/timing --abl-timing $H/ablation/timing --idle $H/ablation/idle_tegrastats.log
```
