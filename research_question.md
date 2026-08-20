# Research Question

## Core Question
How does Monocular Visual Odometry (VO) degrade under increasingly aggressive UAV motion (high angular velocity, sudden accelerations, visual blur, low-texture scenes), and can a lightweight failure-prediction mechanism predict or mitigate VO tracking degradation?

## Objectives
1. **Benchmark Baseline VO**: Evaluate standard monocular VO / Visual-Inertial VO approaches on agile synthetic trajectories in Gazebo + PX4 SITL.
2. **Characterize Failure Modes**: Quantify tracking failure (drift, feature loss, scale drift, loss of lock) under varying degrees of agility.
3. **Failure Prediction**: Investigate lightweight indicators (e.g., feature count drop, optical flow inconsistency, residual spike, motion prior deviation) to forecast tracking loss prior to complete failure.

## Scope & Boundaries
- **Environment**: Simulation-only (ROS 2, Gazebo, PX4 SITL).
- **Phase**: Infrastructure & Scaffolding (Phase 0). No algorithm or ML code until explicitly requested.
