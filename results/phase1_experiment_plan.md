# Phase 1 Experiment Plan: Baseline Monocular VO Characterization in Agriculture World

## Scientific Objective

The primary objective of **Phase 1: Baseline Characterization** is to quantitatively map the operational boundaries, degradation precursor signals, and failure thresholds of un-fused monocular Visual Odometry (VO) across dynamic, agile multicopter flight profiles in the primary benchmark environment (`configs/gazebo_maps/agriculture.world`).

Building directly on Phase 0E discoveries—which proved that Pyramidal KLT tracking ($>98\%$ survival) and Essential Matrix RANSAC ($>500$ inliers, `strict_E_fail = FALSE`) remain robust under isolated yaw rotations up to $180^\circ/\text{s}$ and roll tilt oscillations up to $50^\circ$—Phase 1 systematically evaluates complex multi-axis maneuvers combining linear velocity ($v$), angular velocity ($\omega$), roll/pitch tilt angles ($\phi, \theta$), translational acceleration ($a$), and image-space feature velocity ($v_{\text{feat}}$).

---

## Independent Variables to Sweep

Phase 1 sweeps four key independent motion dimensions:

1. **Linear Translation Velocity ($v_x, v_y, v_z$)**: Forward flight speeds ($0.5\text{ m/s}, 1.5\text{ m/s}, 3.0\text{ m/s}, 5.0\text{ m/s}$) combined with lateral/vertical translation.
2. **Roll Attitude Oscillation Amplitude ($\phi_{\text{amp}}$)**: Single-axis and multi-axis tilt amplitudes ($5^\circ, 15^\circ, 30^\circ, 45^\circ$) at fixed altitude $Z \approx 2.41\text{ m}$.
3. **Attitude Oscillation Frequency ($f_{\text{tilt}}$)**: Oscillation dynamic rates ($0.2\text{ Hz}, 0.5\text{ Hz}, 1.0\text{ Hz}$).
4. **Yaw Angular Velocity ($\omega_z$)**: Coordinated turn rates ($20^\circ/\text{s}, 60^\circ/\text{s}, 120^\circ/\text{s}, 180^\circ/\text{s}$).

---

## Proposed Severity Matrix

Phase 1 organizes flight profiles into a 4-tier severity matrix to establish the transition envelope from nominal performance to tracking failure:

| Severity Level | Nominal Velocity ($v$) | Roll Amplitude ($\phi$) | Yaw Rate ($\omega_z$) | Oscillation Freq ($f$) | Primary Dynamic Stress |
| :---: | :---: | :---: | :---: | :---: | :--- |
| **Level 1 (Low / Baseline)** | $0.5 - 1.0\text{ m/s}$ | $5^\circ$ | $20^\circ/\text{s}$ | $0.2\text{ Hz}$ | Quasi-static baseline, low parallax |
| **Level 2 (Moderate)** | $1.5 - 2.5\text{ m/s}$ | $15 - 20^\circ$ | $60^\circ/\text{s}$ | $0.5\text{ Hz}$ | Moderate tilt acceleration & lateral baseline |
| **Level 3 (High Agility)** | $3.0 - 4.0\text{ m/s}$ | $30 - 35^\circ$ | $120^\circ/\text{s}$ | $0.5 - 1.0\text{ Hz}$ | High image-space feature velocity & parallax |
| **Level 4 (Extreme Stress)** | $5.0\text{ m/s}$ | $45 - 50^\circ$ | $180^\circ/\text{s}$ | $1.0\text{ Hz}$ | Maximum dynamic agility, near-limit tracking |

---

## Achieved-Motion Quantities vs. Commanded Values

In accordance with scientific integrity guidelines established in Phase 0E, **commanded flight controller setpoints ($v_{\text{cmd}}, \phi_{\text{cmd}}, \omega_{\text{cmd}}$) must NOT be trusted as true experimental values**. Aerodynamic drag, motor response latency, and attitude controller attenuation cause deviations between commanded setpoints and actual physical flight.

All independent variable values reported in Phase 1 analysis will be computed from high-frequency Ground Truth (GT) telemetry logs:

- **Achieved Linear Speed**: $v_{\text{achieved}} = \sqrt{\dot{x}^2 + \dot{y}^2 + \dot{z}^2}\text{ (m/s)}$
- **Achieved 95th-Percentile Roll**: $p95 |\phi|\text{ (degrees)}$
- **Achieved 95th-Percentile Pitch**: $p95 |\theta|\text{ (degrees)}$
- **Achieved 95th-Percentile Yaw Rate**: $p95 |\omega_z|\text{ (deg/s)}$
- **Achieved Acceleration Magnitude**: $a_{\text{achieved}} = \|\mathbf{a}\|_2\text{ (m/s}^2\text{)}$
- **Physical Translation Baseline**: $t_{\text{baseline}} = \|\mathbf{p}_k - \mathbf{p}_{k-1}\|_2\text{ (meters per frame)}$

---

## Measurement Specifications

### 1. Ground Truth (GT) Measurements
Logged from Gazebo `/world/default/dynamic_pose/info` at $50.0\text{ Hz}$ into `phase1_*_gt.csv`:
- Simulation Timestamp ($t_{\text{sim}}$)
- World ENU Position ($x, y, z\text{ in meters}$)
- World ENU Orientation Quaternion ($q_x, q_y, q_z, q_w$)
- Body Linear & Angular Velocities ($v_x, v_y, v_z, \omega_x, \omega_y, \omega_z$)
- Euler Roll, Pitch, Yaw angles ($\phi, \theta, \psi$)

### 2. VO / KLT Feature Measurements
Logged from ROS 2 VO node `/vo/path` & telemetry at camera rate ($\sim 30.3\text{ Hz}$) into `phase1_*_vo.csv`:
- `num_detected`: GFTT detected corners count ($N_{\text{gftt}}$, max 2000)
- `num_matched`: KLT optical flow matched features count ($N_{\text{matched}}$)
- `feature_survival_rate`: $S_f = N_{\text{matched}} / N_{\text{tracked\_prev}}$
- `mean_lk_err`: Mean Lucas-Kanade optical flow tracking residual (pixels)
- `feature_velocity_px`: Mean image-space feature displacement ($\bar{v}_{\text{px}} = \frac{1}{N} \sum \|\mathbf{x}_i^{(k)} - \mathbf{x}_i^{(k-1)}\|$)

### 3. Essential Matrix ($E$) Measurements
- `num_inliers_E`: `cv2.findEssentialMat()` 5-point RANSAC inlier count ($N_E$)
- `inlier_ratio_E`: $N_E / N_{\text{matched}}$
- `epipolar_error`: Algebraic epipolar constraint error ($x_2^T E x_1$)
- `num_inliers_H`: `cv2.findHomography()` RANSAC inlier count ($N_H$) for planar/rotational degeneracy checking

### 4. Pose Recovery (`recoverPose`) Measurements
- `num_inliers_pose`: `cv2.recoverPose()` accepted inliers under explicit operating threshold `distanceThresh=1000.0`
- `pose_E_ratio`: $N_{\text{pose}} / N_E$ (depth filter agreement ratio)
- `valid_pose_flag`: Boolean (`num_inliers_pose >= 8`) indicating metric step update validity
- `step_translation_norm`: Unit vector norm ($\|t_{\text{opt}}\| = 1.0$)
- `step_rotation_angle`: Relative camera rotation step angle (degrees)

### 5. Camera & Rendering Quality Metrics
- `camera_fps`: Achieved ROS 2 image publication rate (Hz)
- `frame_delta_t`: Inter-frame timestamp interval ($t_k - t_{k-1}$, target $32.9\text{ ms}$)
- `frame_drop_count`: Count of inter-frame deltas exceeding $50.0\text{ ms}$
- `mean_pixel_brightness`: Image lighting check ($0 - 255$)

---

## Failure Criterion Definition

Phase 1 enforces the exact **$\ge 2.0\text{-second}$ timestamp-spanning sliding window failure criterion** established and validated in Phase 0E:

> **Strict Failure Definition**:
> A failure state is triggered if, in any continuous timestamp-spanning window of duration $T_{\text{window}} \ge 2.0\text{ seconds}$ ($t_{\text{end}} - t_{\text{start}} \ge 2.0\text{s}$), **$> 70\%$ of processed VO frames fall below the low-inlier threshold ($< 5\text{ inliers}$)**.

*Timestamp Spanning Requirement*: The window length is calculated strictly using simulation timestamps ($t_{\text{end}} - t_{\text{start}} \ge 2.0\text{s}$), making the failure evaluation immune to camera FPS drops or rendering rate fluctuations.

---

## Taxonomy of Degradation & Failure Modes

Phase 1 explicitly distinguishes between five distinct degradation stages to avoid conflating simulator artifacts with VO geometric breakdown:

```
[1. Camera / Rendering Degradation]
        │ (FPS < 25 Hz or Frame Delta > 50ms due to GPU load)
        ▼
[2. KLT Feature Tracking Degradation]
        │ (Survival Sf < 80%, Feature Vel > 10.5 px/fr exceeding single-level LK)
        ▼
[3. Essential Matrix Correspondence Degradation]  <-- PRIMARY VO FAILURE SIGNAL
        │ (num_inliers_E < 5 for >70% of 2.0s window)
        ▼
[4. Pose Recovery Depth Filter Degradation]
        │ (num_inliers_pose < 5 due to zero-baseline t=0 depth truncation)
        ▼
[5. Trajectory / Integrated Pose Failure]
        (Catastrophic ATE drift > 2.0m or continuous tracking resets)
```

1. **Camera / Rendering Degradation**: Host system rendering bottleneck causing ROS 2 image drops ($\text{FPS} < 25.0\text{ Hz}$).
2. **KLT Feature Tracking Degradation**: High image-space feature motion causing optical flow loss ($S_f < 80\%$, LK residual $> 2.5\text{ px}$).
3. **Essential Matrix Correspondence Degradation**: Failure of 5-point RANSAC to find consistent 3D epipolar geometry (`num_inliers_E < 5`).
4. **Pose Recovery Depth Filter Degradation**: Unit-scale triangulated depth exceeding threshold ($Z_{\text{unit}} = Z/t > 1000\text{m}$) due to near-zero translation baseline ($t_{\text{true}} \approx 0$).
5. **Trajectory / Integrated Pose Failure**: Accumulated translation/rotation error causing catastrophic trajectory divergence ($\text{ATE} > 2.0\text{ m}$).

---

## Primary Phase 1 Failure Signal Rationale

### **Primary Signal: `num_inliers_E` (Essential Matrix RANSAC Inlier Count)**

**Rationale**:
`num_inliers_E` measures true algebraic epipolar correspondence quality between frame pairs in image space, independent of translation baseline scale. As discovered during Phase 0E Sweep A, pure rotation produces a near-zero physical baseline ($t_{\text{true}} \approx 0$), causing unit-scale triangulated points ($Z_{\text{unit}} \to \infty$) to be rejected by `recoverPose()` depth filters even when image correspondences are 100% healthy ($>800$ inliers).

Treating `num_inliers_pose` as the primary correspondence failure signal would falsely label pure rotational maneuvers as Visual Odometry failures. Therefore:
- **`num_inliers_E` (`strict_E_fail`)** is designated as the **PRIMARY** signal for epipolar correspondence failure.
- **`num_inliers_pose` (`strict_pose_fail`)** is monitored as a secondary signal for metric pose update availability.

---

## Statistical & Repetition Strategy

- **Repetition Count**: Minimum **3 independent trials** per severity matrix cell to account for physics solver non-determinism.
- **Summary Metrics**: All evaluated metrics will report mean, median, standard deviation ($\mu \pm \sigma$), and 95th-percentile bounds across trials.
- **Confidence Intervals**: 95% confidence intervals will be computed for Absolute Trajectory Error (ATE), Relative Pose Error (RPE), and feature survival rate ($S_f$).

---

## Controls & Confounds

### Controlled Parameters
- **World Map**: `configs/gazebo_maps/agriculture.world` (fixed SDF).
- **Camera Sensor Rig**: Downward tilt $0^\circ$, focal length $f_x=f_y=539.94\text{ px}$, resolution $1280 \times 960$.
- **VO Estimator Configuration**: GFTT corners $N=2000$, KLT window $21 \times 21$, `maxLevel=3`, E RANSAC threshold $1.0$, `distanceThresh=1000.0`.

### Monitored Confounds
- **Simulator Rendering Throttle**: Monitored via `camera_fps`. Runs with mean FPS $< 25.0\text{ Hz}$ will be flagged.
- **Wind & Aerodynamic Disturbances**: Disabled in Gazebo physics engine (`wind_speed = 0.0`).

---

## Spatial & Geometric Consistency in Agriculture World

To ensure identical surface texture density, feature distribution, and lighting conditions across all Phase 1 runs, all flights will be constrained to a standardized spatial volume in `agriculture.world`:

- **Standard Spawn Pose**: $X = 14.0505\text{ m}, Y = -7.5229\text{ m}, Z = 0.1076\text{ m}$ (Ground elevation $-0.0924\text{ m}$).
- **Target Cruise Altitude**: $Z = 2.4076\text{ m}$ (Relative hover altitude $2.50\text{ m}$).
- **Spatial Flight Bounds**:
  - $X \in [5.0\text{m}, 25.0\text{m}]$
  - $Y \in [-15.0\text{m}, 5.0\text{m}]$
  - $Z \in [1.5\text{m}, 3.5\text{m}]$

---

## Dataset Naming & Versioning Scheme

All Phase 1 data will be stored in `results/phase1/` using the following standardized naming convention:

`phase1_<TRAJECTORY_TYPE>_<SEVERITY_LEVEL>_run<TRIAL_ID>_<LOG_TYPE>.csv`

### Log Types:
- `gt.csv`: Ground truth pose telemetry ($50\text{ Hz}$)
- `vo.csv`: VO estimation & tracking telemetry ($30\text{ Hz}$)
- `telemetry.csv`: PX4 flight controller setpoint telemetry ($50\text{ Hz}$)

### Examples:
- `results/phase1/phase1_straight_level1_run1_gt.csv`
- `results/phase1/phase1_roll_level3_run2_vo.csv`
- `results/phase1/phase1_circle_level4_run3_telemetry.csv`

---

## Valid Phase 1 Run Criteria

A Phase 1 trial is classified as **VALID** if and only if all of the following conditions are met:
1. PX4 SITL and Gazebo simulation complete the full $20.0\text{s}$ maneuver without process crash or failsafe triggering.
2. Mean camera publication rate remains $\ge 25.0\text{ Hz}$ throughout the maneuver.
3. Ground truth pose logging achieves $\ge 45.0\text{ Hz}$ continuous logging.
4. Process lifecycle cleanup completes cleanly before and after the trial.

---

## Invalidation & Re-run Rules

A Phase 1 trial is classified as **INVALID** and must be repeated if any of the following occur:
1. Host CPU/GPU rendering throttle causes camera FPS to drop below $25.0\text{ Hz}$ for $>1.0\text{s}$.
2. Ground truth pose logging drops or experiences timestamp gaps $>50.0\text{ ms}$.
3. Drone crashes, strikes obstacles, or experiences PX4 offboard mode disconnect.
4. Process cleanup fails between runs, leaving orphaned simulation processes.

---

## Explicit Phase 2 Intervention Exclusion

> **MANDATORY CONSTRAINT**:
> No Phase 2 predictive failure module, adaptive motion intervention, homography fallback, gyro sensor fusion, or online parameter tuning shall be introduced or active during Phase 1 baseline characterization.

Phase 1 must evaluate the **un-modified baseline monocular VO pipeline** (`src/minimal_vo.py`) strictly as frozen and validated at the conclusion of Phase 0E.

---

## Verification Plan

### Automated Verification
- Run static syntax compilation checks (`python3 -m py_compile`) on evaluation scripts.
- Run synthetic unit test suite (`python3 -m unittest tests/test_essential_matrix_bookkeeping.py`).

### Manual Verification
- Review generated markdown report and verify alignment with Phase 0E findings and repository guidelines.
