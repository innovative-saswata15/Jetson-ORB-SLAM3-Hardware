# Husky evaluation: analysis of all results

Jetson ORB-SLAM3 on the other team's Husky recording (`husky_20261023`: 355 s, a 65.6 m loop, D435 IR
at 640×480 and 15 FPS, Husky IMU at 83 Hz). It is scored against a reference trajectory built from the
lidar, IMU and wheels.

This analysis covers:
- [reference/README.md](reference/README.md): the reference;
- [stage1/report/HUSKY_REPORT.md](stage1/report/HUSKY_REPORT.md): stage 1;
- [ablation/report/ABLATION_REPORT.md](ablation/report/ABLATION_REPORT.md): the ablation, 54 runs over
  21 cells, plus 18 timing runs.

**Setup:**
- Jetson Orin Nano, MAXN_SUPER + `jetson_clocks`, commit `2e62001`.
- Loop closure is DBoW2 unless the cell says otherwise.
- Each cell has 3 runs (5 in stage 1) and 1 timing run.

## Assumption and method

The camera and the IMU are treated as **calibrated**. The remaining error is attributed to data noise
and algorithm behaviour.

One **constant** scale factor between the stereo camera and the lidar was observed: **1.068**. It is
identical in all 69 runs and every stereo cell, and it also appears in the camera's own depth
stream. It is removed by:
- using **Sim(3)-aligned ATE** (the shape error, with scale fitted) as the primary accuracy metric;
- confirming it with cell **D3**, where the stereo baseline is multiplied by 1.068.

Raw SE(3) ATE is still listed in the reports. The reference's own uncertainty is ~27 cm
(conservative), so differences between cells smaller than that are not claimed.

---

## 1. Key findings

1. **Our pipeline is accurate.** With the scale factor removed, the trajectory shape error is
   **8.4 cm over 65.6 m**. With the corrected baseline (D3) it is 8.7 cm ATE SE(3), scale 1.003, and
   1.3% translation drift over 10 m.
2. **GPU ORB ≡ CPU ORB in accuracy.** Shape errors are 8.4 vs 7.9 cm, within run-to-run spread, and
   the ground-truth-free trajectory check agrees. **GPU extraction is 33% faster** (17.9 vs 26.6 ms).
   **Pipelining** brings tracking to **27 ms per frame (37 FPS)**, 2.5× the camera rate.
3. **Stereo is the right sensor set for this robot.**
   - **Mono** fails (2.1 m shape error).
   - **Mono + IMU** can't initialise reliably: 97 s to initialise, 16% coverage.
   - **Stereo + IMU** has a worse shape than stereo alone (22 vs 8 cm).
   - **RGB-D** gives no gain.

   The inertial behaviour follows from the data: the Husky turns at ≤ 7°/s and barely accelerates,
   so gravity and IMU bias are weakly observable (§4.2).
4. **Large compute savings at no accuracy cost.** Measured as SLAM's own power above the 6.64 W idle
   board:
   - 800 features: −22% power, 31.8 FPS;
   - 7.5 FPS input: −29% power;
   - with the IMU, `ADAPTIVE_FE`: **−51% power, 39 FPS**.
5. **Loop closure never triggers** on this route, with DBoW2 or the CNN: the only revisit is seen
   from a perpendicular viewpoint. The loop-closure cells therefore measure cost only. The CNN adds
   +2.5 ms per frame and +4% power for no benefit here.

---

## 2. The reference

- **Built from:**
  - lidar odometry (KISS-ICP, deskewed);
  - 2 lidar loop closures;
  - IMU preintegration (bias estimated);
  - wheel odometry, scale-calibrated against the lidar;

  all in one GTSAM factor graph. The camera was not used, so the reference is independent of what it scores.
- **Dataset issues it handled:**
  - the lidar yaw in their `/tf_static` is off by 179.6°: measured from the data and corrected (worth
    reporting to the other team);
  - the IMU driver's orientation output is inconsistent: excluded;
  - the magnetometer is distorted indoors: excluded;
  - wheel distance scale 0.956: calibrated.
- **Stability:** removing any one input changes the reference by 3–11 cm RMS.
- **Non-visual baselines,** scored the same way:

  | | Shape error | Scale |
  |---|---|---|
  | Wheels / robot EKF | 9.1 cm | 0.952 |
  | Lidar alone | 44 cm (height drift) | 0.996 |

## 3. Stage 1: our pipeline (stereo IR, 3 arms × 5 runs)

| | GPU ORB | CPU ORB | Pipelined |
|---|---|---|---|
| **Shape error, ATE Sim(3)** | **8.4 cm** (7.8–9.0) | 7.9 cm (7.5–8.7) | 8.3 cm (7.8–8.6) |
| Height part of the error | 2.9 cm | 2.8 cm | 2.9 cm |
| Rotation drift over 10 m | 9.1°/100 m | 9.3°/100 m | 9.1°/100 m |
| Coverage, maps | 100%, 1 | 100%, 1 | 100%, 1 |
| ORB extraction | **17.9 ms** | 26.6 ms | overlapped |
| **Total tracking per frame** | 43.4 ms (23 FPS) | 51.7 ms (19 FPS) | **27.0 ms (37 FPS)** |
| Local mapping per keyframe | 366 ms | 357 ms | 344 ms |
| SLAM power above idle | 2.57 W | 2.51 W | 2.53 W |
| Runs failed | 0/5 | 0/5 | 1/5 (Sophus NaN abort) |

- **GPU ≡ CPU:**
  - Δ shape error is 0.5 cm, inside the run-to-run spread.
  - The cross-arm trajectory distance (2.5 cm) is no larger than run-to-run (2.1–2.6 cm).

  That reproduces the paper's equivalence claim against real ground truth.
- **Where the time goes** (GPU arm): ORB extraction 41%, local-map tracking 25%, stereo matching 17%.
  Pipelining hides extraction completely.
- **Noise level:** the run-to-run spread of the shape error is ~1 cm, which is the non-determinism
  of the multithreaded pipeline on identical data.

## 4. Ablation (one factor changed from B0 = stereo IR, GPU ORB, DBoW2, 15 FPS, 1250 features)

### 4.1 Summary

| Cell | Change | Shape error, Sim(3) | Tracking ms (FPS) | SLAM power vs B0 | Verdict |
|---|---|---|---|---|---|
| B0 | baseline | 8.4 cm | 43.4 (23) | 2.57 W | |
| D3 | baseline × 1.068 | 8.2 cm (**ATE SE(3) 8.7 cm**) | 42.9 (23) | ±0% | confirms the scale factor |
| A1 | mono | 213 cm | 21.0 (48) | −35% | fails |
| A3 | mono + IMU | 40 cm, 16% coverage | – | −25% | fails to initialise |
| A4 | stereo + IMU | 22 cm | 42.3 (24) | −30% | worse shape |
| A5 | RGB-D, IR + depth | 12 cm | n/a | ±0% | no gain |
| A6 | RGB-D, colour + depth | 9.0 cm | n/a | −4% | no gain |
| B1 | CPU ORB | 7.9 cm | 51.7 (19) | −2% | equivalent, slower |
| B2 | pipelining | 8.3 cm | **27.0 (37)** | −1% | equivalent, fastest |
| C1 | no loop closing | 8.4 cm | 42.5 (24) | −1% | no change |
| C2 | + CNN loop detector | 7.6 cm | 45.9 (22) | +4% | no change, costs more |
| D1 | 7.5 FPS | 7.7 cm | 41.3 (24) | **−29%** | **same accuracy** |
| D2 | 800 features | 8.2 cm | **31.4 (32)** | **−22%** | **same accuracy** |

**Stereo + IMU arms**, compared with A4 (22 cm, 42.3 ms, −30%):

| Cell | Change | Shape error | Tracking | Power vs B0 |
|---|---|---|---|---|
| B1i | CPU ORB | 27 cm | 51.7 ms | −31% |
| B2i | pipelining | 22 cm | 26.2 ms | −31% |
| **B3** | **`ADAPTIVE_FE`** | **21 cm** | **25.4 ms (39 FPS)** | **−51%** |
| C1i | no loop closing | 26 cm | 41.8 ms | −32% |
| C2i | CNN | 22 cm | 44.6 ms | −25% |
| C3 | `ADAPTIVE_LC` | 23 cm | 41.6 ms | −30% |
| D1i | 7.5 FPS | 25 cm | 41.3 ms | −46% |
| D2i | 800 features | 30 cm | 31.6 ms | −47% |

### 4.2 Sensors

- **Mono (A1):** no metric scale, as expected. Even scale-fitted, its shape error is 2.1 m: a single
  640×480 IR camera on a slowly turning robot has too little parallax. It is the cheapest
  configuration (21 ms per frame), but unusable.
- **Mono + IMU (A3):**
  - IMU initialisation takes 97 s, with 15 "not enough motion" resets over 3 runs.
  - Coverage is 16%, with 5 maps.
  - Initialising visual-inertial ORB-SLAM3 needs enough acceleration and rotation, and this route
    has very little.
- **Stereo + IMU (A4):**
  - The IMU initialises quickly (2.6 s, no resets): the stereo pair supplies scale.
  - The shape error rises to 22 cm, mostly in height (18 cm against 3 cm). It is the same in all 27
    stereo-inertial runs and largest at the start and end of the run.
  - With gentle, nearly planar motion, the gravity direction and accelerometer bias are only weakly
    observable during initialisation. A small error in the estimated gravity tilts the whole
    trajectory.
  - On the positive side, the inertial mode creates half the keyframes (369 against 682). That cuts
    SLAM power by 30% at the same tracking cost per frame.
- **RGB-D (A5, A6):** no better than stereo. They replace stereo matching with the camera's depth,
  but local mapping is no cheaper (300–500 ms per keyframe), and the IR + depth cell inserts many more
  keyframes (1099).

### 4.3 Compute features

- **GPU ORB:** −33% extraction time against CPU ORB, with identical accuracy, with and without the IMU.
- **Pipelining:** the fastest tracking (26–27 ms), and accuracy is unchanged.
- **`ADAPTIVE_FE`** (an economy extractor while the IMU reports calm motion):
  - extraction 8.4 ms, stereo matching 4.0 ms, local mapping 206 ms per keyframe;
  - **the lowest power of any usable cell** (−51%), with unchanged accuracy;
  - on a slow ground robot it is almost always in economy mode.

### 4.4 Loop closure

No configuration closed a loop (DBoW2, CNN, `ADAPTIVE_LC`). The route's end overlaps its start only
partly and at about 90°. A forward-facing camera can't recognise the place there, while the lidar's
360° view did (2 closures in the reference). So:
- switching loop closing off (C1) changes nothing;
- the CNN (C2) costs +2.5 ms per frame and +4% power.

Evaluating loop closure needs a route with same-direction revisits.

### 4.5 Input rate and feature budget

- **7.5 FPS (D1):** the same accuracy, −29% SLAM power. The per-frame cost is unchanged (41 ms);
  half as many frames are processed. At this robot's speed (≤ 0.4 m/s), 15 FPS is more than needed.
- **800 features (D2):** the same accuracy; tracking 31 ms (32 FPS), local mapping 244 ms per
  keyframe, −22% power. It is the best accuracy-per-watt among the stereo cells. With the IMU (D2i),
  the IMU cells' larger error grows further (30 cm).

## 5. Recommended configuration

For this robot and camera: **stereo IR, GPU ORB, `PIPELINE_FE`, 800 features, 7.5–10 FPS**.
- Every component of it individually keeps the ~8 cm shape accuracy.
- Together they should give well under half the SLAM power, with tracking far inside the frame budget.
- One run would confirm the combination.

Add the IMU only for routes with more dynamic motion, where gravity and bias become observable.

## 6. Limitations

- **One sequence:** 355 s, 65.6 m, flat and slow. The conclusions about the IMU and the input rate are
  specific to gentle motion.
- **Reference uncertainty** of ~27 cm (conservative). Only the large effects are resolvable against it:
  the sensor cells, D3, and the inertial shape degradation.
- **Timing is 1 run per cell.** Its run-to-run variation, as seen in stage 1, is a few percent.
- **`rgbd_tum` does not record total tracking time,** so A5 and A6 have only stage times.
- **The mono + IMU timing run produced no timing file:** it ended at 118 s.
- **The Sophus NaN abort** occurred in 1 of the 69 accuracy runs (stage-1 pipe arm).
