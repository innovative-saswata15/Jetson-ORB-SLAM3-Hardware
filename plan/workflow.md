1. Project Overview
This project deploys Jetson-ORB-SLAM3, a GPU-accelerated and accuracy-preserving implementation of
ORB-SLAM3, on real hardware: an NVIDIA Jetson Orin Nano and an Intel RealSense D435 stereo camera,
carried on a manually driven rover. The system tracks the camera's 3D position in real time from
the D435's two infrared cameras (stereo visual SLAM, with no IMU), builds a sparse 3D map, and
corrects drift through loop closure. On top of the unmodified library, a STorM32 3-axis gimbal
add-on stabilises the camera and points it to improve robustness: coverage sweeps at stops, and
recovery sweeps when tracking is lost. Wheel encoders and an RPLIDAR A1 may be added later for
loose sensor fusion*.

2. Problem Statement
Visual SLAM on a small moving platform with an edge computer faces three problems:
- vibration and bumps blur the images and break tracking;
- places are recognised only when revisited from a similar viewing direction, so out-and-back
  routes rarely close their loops;
- low-texture views make tracking fail, and ORB-SLAM3 then starts a new, disconnected map.

The goal of this project is to run the repository's GPU-accelerated ORB-SLAM3 as-is, live on the
Jetson with the D435, and to demonstrate tracking and loop closure on the rover. We then build a
gimbal add-on that works only through ORB-SLAM3's public interface, and measure how much it
improves robustness against a baseline: stability, coverage sweeps at stops, and recovery sweeps
when tracking is lost. Later, the visual trajectory may be fused with wheel odometry and 2D LiDAR
scans*.

3. Tech Stack
Hardware & Sensors
Compute: NVIDIA Jetson Orin Nano Developer Kit (8 GB), JetPack 6.2.3 on an NVMe SSD

Vision Sensing: Intel RealSense D435 (stereo infrared pair, 640x480 at 30 fps, dot projector off)

Camera Stabilisation & Pointing: STorM32 BGC v1.3 3-axis brushless gimbal, with an IMU on the
camera plate

Mobile Platform: manually driven small wheeled rover (RC car)

Range Sensing (*): RPLIDAR A1 2D laser scanner*

Odometry (*): wheel encoders with a microcontroller*

Software & Middleware
Operating System: Ubuntu 22.04 LTS (JetPack 6.2.3, L4T 36.5.2)

GPU & Vision Libraries: CUDA 12.6, TensorRT 10.3, OpenCV 4.8

SLAM: Jetson-ORB-SLAM3 (ORB-SLAM3 with CUDA ORB feature extraction), with DBoW2, g2o, Sophus and
Pangolin

Camera Driver: Intel librealsense 2.55.1

Remote Display: Xvfb, openbox and x11vnc on the Jetson; TigerVNC over an SSH tunnel on the PC

Project Tools: rover driver (stereo_realsense_D435_rover), rover_run.sh, summarize_run.py,
storm32_probe.py

Middleware & Fusion (*): ROS 2 Humble, slam_toolbox, robot_localization*

Development: Git, VS Code

4. Deliverables
Hardware Deployment of the Repository: the repository built and running on the Jetson with
RealSense support. The authors' EuRoC result is reproduced (ATE 3.72 cm prebuilt, 4.06 cm from
source), with a calibration file for our D435.

Live SLAM Verification: loop closure demonstrated handheld and on the rover, with a recorded video.

Rover Driver & Run Tools: a copy of the live driver that runs headless, stops cleanly, saves
trajectories and logs tracking events, plus scripts that summarise each run as numbers.

Documentation: the plan, per-stage design documents, a step-by-step runbook, and a progress log of
everything done.

Gimbal Stabilisation (G1): the D435 mounted, balanced and tuned on the STorM32 gimbal.

Gimbal Control Link (G2): serial control of the gimbal from the Jetson, with the command protocol
for our firmware documented.

Coverage & Recovery Sweeps (G3, G4): gimbal sweeps at stops and on tracking loss. They're driven
by ORB-SLAM3's tracking state without modifying the library, and evaluated against the baseline
routes.

Visual-LiDAR-Odometry Fusion (*): wheel encoders and the RPLIDAR A1 with slam_toolbox, loosely
fused with the ORB-SLAM3 trajectory in ROS 2*.

(*) Future or optional scope (Stage 3), not planned now.

5. Workflow
Details are in stage1_runbook.md and stage2.md. What actually happened is in progress_log.md.

STAGE 1: THE REPO ON OUR HARDWARE

Setup
- Done: JetPack 6.2.3 on the NVMe, nvidia-jetpack installed
- Done: Repo cloned; git workflow (commit on the PC, pull on the Jetson)
- Done: EuRoC check with the prebuilt binary (ATE 3.72 cm)
- Done: Source build with RealSense support (ATE 4.06 cm)
- Done: D435 calibration file (RealSense_D435.yaml)
- Done: Virtual screen over VNC (display :99)

Milestone 1: handheld
- Done: Live walk with the original driver: loop closed, no tracking loss

Milestone 2: rover
- Done: Camera and Jetson mounted on the rover
- Done: Route A run 1: loop closed on the rover
- Pending: Route A runs 2 and 3 (repeatability)
- Pending: If Stage 2 goes ahead: baseline routes B (out-and-back) and C (blank wall), 3 runs
  each, with the rover driver

STAGE 2: GIMBAL ADD-ON

G1: stability
- Mount and balance the D435 on the STorM32 gimbal, and route the cable
- Tune it in the GUI (PIDs, IMU, motors); pan mode HOLDHOLDPAN
- Re-run routes A, B and C, and compare with the baseline

G2: control from the Jetson
- Connect the STorM32 over USB (/dev/storm32)
- Discovery tests T1-T8 (tools/storm32_probe.py): which angle commands work
- Write storm32_link and gimbal_controller (30 Hz thread)

G3: coverage sweeps
- At a stop, the s key sweeps +/-170 degrees at 30 degrees/s
- Evaluate on route B: the loop should close on the return trip

G4: recovery sweeps
- When tracking is lost: turn back by the gimbal-IMU angle, then sweep
- Evaluate on route C: fewer new maps, more relocalisations

Wrap-up
- Demo run, and a results table (baseline against G1, G3 and G4)




