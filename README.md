# Robust Monocular Visual Odometry (VO) Under Agile UAV Motion

This repository contains the ROS 2, Gazebo Sim, and PX4 SITL research framework for quantifying monocular Visual Odometry (VO) performance degradation and early failure prediction during dynamic, agile multicopter flight.

---

## Project Overview

Monocular Visual Odometry is fundamental for GPS-denied drone navigation but degrades under rapid angular rotations, aggressive maneuvers, and low-parallax flight regimes. This project establishes a rigorous, scientifically validated evaluation pipeline to isolate failure mechanisms, measure precursor degradation signals, and design predictive intervention mechanisms.

### Phase Status Overview

- **Phase 0 — Infrastructure Verification & Scientific Characterization (COMPLETE)**
  - **Phase 0A**: Camera sensor ($30.4\text{ Hz}$, $1280 \times 960$, zero-distortion pinhole intrinsics) and Ground Truth pose stream ($50.0\text{ Hz}$, World ENU) verified.
  - **Phase 0B**: Simulation Time nearest-neighbor synchronization ($\le 20\text{ ms}$ tolerance) established ($99.02\%$ frame matching).
  - **Phase 0C**: Minimal un-fused 5-point Essential Matrix RANSAC + Pyramidal KLT monocular VO ROS 2 node (`src/minimal_vo.py`) developed.
  - **Phase 0D**: 7-DoF Sim(3) Umeyama scale alignment and evaluation pipeline (`src/evaluate_trajectory.py`) automated.
  - **Phase 0E**: Pure Yaw Rate Escalation (Sweep A, $10 - 180^\circ/\text{s}$) and Corrected Attitude Amplitude Escalation (Sweep B, $5 - 50^\circ$ roll) completed and scientifically characterized. Metric conflation between Essential Matrix RANSAC (`num_inliers_E`) and pose depth-filter inliers (`num_inliers_pose`) audited, repaired (`distanceThresh=1000.0`), and validated in physical flight.
- **Phase 1 — Baseline Characterization (PLANNED)**
  - Comprehensive severity matrix sweep in `agriculture.world` across velocity, attitude, angular rate, and feature motion dynamics.
- **Phase 2 — Failure Prediction & Intervention (PLANNED)**
  - Early precursor signal detection module and adaptive motion-aware intervention.
- **Phase 3 — Head-to-Head Evaluation (PLANNED)**
- **Phase 4 — Analysis & Report Finalization (PLANNED)**

---

## Environment Specifications

- **OS**: Ubuntu 26.04 LTS (*resolute*)
- **ROS Distribution**: ROS 2 `lyrical`
- **Simulator**: Gazebo Sim / Tools `10.4.0` (Ogre2 rendering engine with NVIDIA GPU offload)
- **Autopilot**: PX4 SITL `v1.18.0-beta1-209-g8aba32c862`
- **Python**: `3.14.4` (`opencv-python` 4.10.0, `evo` 1.37.0, `scipy` 1.18.0, `numpy` 2.3.5)

---

## Experimental World Policy

1. **Phase 0 (Diagnostic Characterization)**:
   Controlled characterization and diagnostic benchmarking utilize `configs/gazebo_maps/exploration_complex/boxworld_obstacles_tight.world` and other diagnostic worlds where appropriate for isolating specific rotational or geometric variables.
2. **Phase 1+ (Primary Standard Ground)**:
   `configs/gazebo_maps/agriculture.world` is designated as the **PRIMARY** research and testing environment for Phase 1 baseline characterization, Phase 2 intervention testing, and all subsequent experimental phases. It serves as the standard benchmark test ground unless a specific experiment explicitly requires another controlled environment.

---

## Core Documentation References

- **Implementation Plan / Phase 1 Plan**: [results/phase1_experiment_plan.md](file:///home/purab/Purab/Projects/ROS/results/phase1_experiment_plan.md)
- **Failure Log & Technical Journal**: [failure_log.md](file:///home/purab/Purab/Projects/ROS/failure_log.md)
- **Methodology & Phase Roadmap**: [methodology.md](file:///home/purab/Purab/Projects/ROS/methodology.md)
- **Environment Specifications**: [environment.md](file:///home/purab/Purab/Projects/ROS/environment.md)
- **Task Audit & TODO List**: [todo.md](file:///home/purab/Purab/Projects/ROS/todo.md)

---

## Phase 0E Scientific Reports

- **Sweep A (Pure Yaw Escalation)**: [results/yaw_sweep_phase0e_v2_summary.md](file:///home/purab/Purab/Projects/ROS/results/yaw_sweep_phase0e_v2_summary.md)
- **Sweep B (Attitude Escalation)**: [results/tilt_sweep_phase0e_v3_summary.md](file:///home/purab/Purab/Projects/ROS/results/tilt_sweep_phase0e_v3_summary.md)
- **`recoverPose` Forensic Audit**: [results/essential_matrix_pipeline_audit.md](file:///home/purab/Purab/Projects/ROS/results/essential_matrix_pipeline_audit.md)
- **Bookkeeping Repair**: [results/essential_matrix_bookkeeping_repair.md](file:///home/purab/Purab/Projects/ROS/results/essential_matrix_bookkeeping_repair.md)
- **Synthetic Parameter Investigation**: [results/recoverpose_distance_threshold_investigation.md](file:///home/purab/Purab/Projects/ROS/results/recoverpose_distance_threshold_investigation.md)
- **Real-Flight 10° Agriculture Validation**: [results/recoverpose_1000_10deg_agriculture_validation.md](file:///home/purab/Purab/Projects/ROS/results/recoverpose_1000_10deg_agriculture_validation.md)
