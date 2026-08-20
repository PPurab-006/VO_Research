# Methodology & Experimental Design

## Overview
This research investigates the degradation behavior of Monocular Visual Odometry (VO) under agile UAV flight profiles generated via PX4 SITL in Gazebo simulation.

## Workflow Phases
- **Phase 0: Infrastructure & Scaffolding** *(Current)*
  - Establish reproducible environment (`environment.md`).
  - Configure Gazebo + PX4 SITL simulation baseline.
  - Set up dataset recording (ROS 2 bags) & evaluation pipeline.
- **Phase 1: Baseline VO & Trajectory Generation**
  - Collect synthetic trajectory datasets across low, medium, and high agility presets.
  - Run baseline monocular VO algorithm.
  - Evaluate accuracy & robustness using `evo` metrics (APE, RPE).
- **Phase 2: Failure Characterization**
  - Identify geometric & dynamic failure thresholds.
  - Correlate feature tracking loss with UAV dynamics (angular velocity, acceleration).
- **Phase 3: Failure Prediction Mechanism**
  - Design & integrate lightweight failure prediction indicators.
  - Evaluate prediction lead time and accuracy.

## Reproducibility Protocol
- All flight commands and trajectory configs will be deterministically defined in `configs/`.
- Trajectory evaluation scripts will be automated and version-controlled in `src/`.
- Raw results and logs stored under `results/` and summarized in `failure_log.md`.
