1. Project Overview
This project deploys Jetson-ORB-SLAM3, a GPU-accelerated and accuracy-preserving implementation of
ORB-SLAM3, on our own hardware: an NVIDIA Jetson Orin Nano and an Intel RealSense D435 stereo
camera, on a manually driven rover. We first recreate the paper's published results
(arXiv:2608.17874) on our hardware stack. We then extend the system with an IMU and a LiDAR into
a fused visual-inertial-LiDAR mapping system. Through ablations, it is evaluated against an
environment mapped with a standard 3D LiDAR. The repository's core code is never modified; all
our work is added alongside it.

2. Problem Statement
The paper reports that its GPU front end preserves ORB-SLAM3's accuracy on public datasets, and
quantifies its speed and power. These claims have to hold on our own Orin Nano before we build on
them. Beyond that, a camera-only system loses tracking in poor light, on blank walls and in fast
turns. An IMU stabilises tracking through fast motion, and a LiDAR keeps working where vision
fails.

The goal is:
- to reproduce the paper's accuracy, equivalence, throughput and power results on our Jetson,
  and the same equivalence and speed checks on our D435;
- then to build stereo-inertial ORB-SLAM3 (D435 + IMU), loosely fused with 2D LiDAR SLAM;
- and to measure each sensor's contribution against a 3D-LiDAR reference map and ground-truth
  trajectory.

3. Tech Stack
Hardware & Sensors
Compute: NVIDIA Jetson Orin Nano Developer Kit (8 GB), JetPack 6.2.3 on an NVMe SSD

Vision Sensing: Intel RealSense D435 (stereo infrared pair, 640x480 at 30 fps, dot projector off)

Inertial Sensing: IMU, preferably by swapping to a D435i (built-in, hardware-timestamped);
otherwise an external IMU rigidly mounted to the camera

Range Sensing: RPLIDAR A1 2D laser scanner on the rover

Reference: a standard 3D LiDAR to map the test environment (and, preferably, to give
ground-truth trajectories)

Mobile Platform: manually driven small wheeled rover (RC car)

Software & Middleware
Operating System: Ubuntu 22.04 LTS (JetPack 6.2.3, L4T 36.5.2)

GPU & Vision Libraries: CUDA 12.6, TensorRT 10.3, OpenCV 4.8

SLAM: Jetson-ORB-SLAM3 (stereo and stereo-inertial), with DBoW2, g2o, Sophus and Pangolin;
optional CosPlace CNN loop closure via TensorRT

Camera Driver: Intel librealsense 2.55.1

Reproduction Pipeline: repro/ (dataset fetcher, experiment runner, evaluator, report generator,
feature-equivalence and D435 recording tools)

Middleware & Fusion (Stage 2): ROS 2 Humble, realsense-ros, sllidar_ros2, slam_toolbox,
robot_localization; Kalibr and Allan variance for calibration; Open3D for map evaluation

Remote Display: Xvfb, openbox and x11vnc on the Jetson; TigerVNC over an SSH tunnel on the PC

Development: Git, VS Code

4. Deliverables
Hardware Deployment (done): the repository built and running on the Jetson with RealSense
support; a calibration file for our D435; live loop closure, handheld and on the rover, with a
recorded video.

Paper Reproduction: every paper result re-measured on our Jetson (EuRoC/TUM-VI/KITTI accuracy,
GPU vs CPU, throughput, per-stage timing, CNN latency, loop-closure ablation, power,
feature-level equivalence), in one report next to the paper's values. Plus the same equivalence
and speed checks on our own D435 recordings.

IMU Integration: a calibrated stereo-inertial configuration (Allan variance, Kalibr camera-IMU
extrinsic and time offset) running with the repository's stereo-inertial mode.

LiDAR Fusion: a ROS 2 system (our ORB-SLAM3 wrapper, EKF, slam_toolbox) giving a fused trajectory,
a 2D map and a dense 3D map.

Evaluation: a 3D-LiDAR reference map and ground truth; ablations C0-C4 (stereo, stereo-inertial,
LiDAR only, stereo + LiDAR, full system), with trajectory error, map accuracy and completeness,
robustness, runtime and power.

Documentation: the plan, runbooks for every phase, and a progress log of everything done.

5. Workflow
Details are in stage1_runbook.md, paper_reproduction_runbook.md and stage2_runbook.md. What
actually happened is in progress_log.md.

STAGE 1: THE REPO ON OUR HARDWARE

Setup
- Done: JetPack 6.2.3 on the NVMe, nvidia-jetpack installed
- Done: Repo cloned; git workflow (commit on the PC, pull on the Jetson)
- Done: EuRoC check with the prebuilt binary (ATE 3.72 cm) and from source (ATE 4.06 cm)
- Done: D435 calibration file (RealSense_D435.yaml)
- Done: Virtual screen over VNC (display :99)

Milestone 1: handheld
- Done: Live walk with the original driver: loop closed, no tracking loss

Milestone 2: rover
- Done: Camera and Jetson mounted on the rover
- Done: Route A run 1: loop closed on the rover
- Pending: Route A runs 2 and 3 (repeatability)

PAPER REPRODUCTION

Datasets and builds
- Fetch EuRoC (11 sequences) and TUM-VI (6 rooms); download KITTI 00-10 by hand
- Build our tools (repro/cpp) and the instrumented timing build (separate worktree)
- Export CosPlace to ONNX on the PC; build the TensorRT engine on the Jetson; log its latency

Experiments
- Smoke test, then: feature equivalence, power, instrumented timing, throughput,
  EuRoC accuracy (GPU vs CPU x 5), TUM-VI, KITTI, loop-closure ablation
- Record 3-4 D435 sequences; run the d435 suite (GPU, CPU, pipelined)

Report
- evaluate.py, then make_report.py: every table next to the paper's values
- Check each "reproduced if" criterion; explain any differences

STAGE 2: IMU AND LIDAR FUSION

Decisions
- IMU (D435i recommended), LiDAR, 3D reference LiDAR, ground-truth method, ROS 2

Calibration
- Allan variance (3 h static), Kalibr camera-IMU (x3), camera-LiDAR-base URDF

Visual-inertial ORB-SLAM3
- Driver (D435i: the repository's own; external IMU: ours), bring-up checks, stereo vs
  stereo-inertial ablation

LiDAR fusion
- ROS 2 stack: realsense-ros, IMU, sllidar_ros2, our ORB-SLAM3 wrapper, EKF, slam_toolbox,
  rosbag2 recording; dense D435 depth map

Evaluation
- 3D-LiDAR reference map and ground-truth trajectory
- Ablations C0-C4 on routes R-A to R-E, 3 runs each; final results report
