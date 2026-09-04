# Methodology & Experimental Roadmap

## Overview
This roadmap governs the experimental evaluation of monocular visual odometry degradation and early failure prediction on agile UAV flight profiles using ROS 2, Gazebo, and PX4 SITL.

---

## Phase Breakdown

### Phase 0 — Infrastructure & Pipeline Verification (Current Phase)
**Hard Gate Exit Condition**: Given one fixed PX4 trajectory and one fixed configuration, a single documented command sequence reproduces camera + ground-truth data, runs VO, aligns the trajectory, and generates the estimated-vs-ground-truth plot from a clean workspace.

- **0A — Sensor + Ground Truth**:
  - Confirm Gazebo camera topic; document encoding, resolution, and frame rate.
  - Extract and save camera intrinsics and distortion parameters.
  - Confirm camera frame convention (forward/up/right).
  - Build recorder: save raw frames + timestamps.
  - Confirm ground-truth pose topic; perform static and directional motion sanity tests.
  - Document coordinate conventions (NED/ENU, quaternion order).
- **0B — Synchronization**:
  - Record camera + ground truth simultaneously on a fixed trajectory.
  - Compute measured timestamp association (timestamp difference matching, not arrival-order).
  - Output diagnostic: frame count, pose count, matched pairs, max/median timestamp gap.
  - Set and document acceptable tolerance.
- **0C — Minimal VO**:
  - Implement basic pipeline: feature detection $\rightarrow$ matching $\rightarrow$ essential matrix $\rightarrow$ relative pose $\rightarrow$ trajectory integration.
  - Log intermediate quantities from day one: feature counts, inlier ratios, per-frame relative motion.
  - Run on a boring, fixed reference trajectory.
- **0D — Alignment & Evaluation**:
  - Implement scale/trajectory alignment (via `evo`) prior to computing ATE/RPE error metrics.
  - Automate end-to-end evaluation pipeline: raw VO output $\rightarrow$ alignment $\rightarrow$ ATE/RPE $\rightarrow$ plot generation.
- **0E — Reproducibility Gate**:
  - Freeze environment specifications in `environment.md`.
  - Commit PX4/Gazebo configs and scripts to Git.
  - Validate clean-workspace single-command pipeline execution.
  - Maintain ongoing log in `failure_log.md`.

---

### Phase 1 — Baseline Characterization
- Define motion severity quantitatively ($v, \omega, a, v_{\text{feat}}$) across numerical ranges (low, medium, high, extreme).
- Construct deterministic trajectory generator for controlled severity presets.
- Run baseline VO across severity matrix with repeated trials per condition.
- Measure ATE, RPE, tracking loss rate (mean $\pm$ std). Output a quantitative failure map.

---

### Phase 2 — Hypothesis & Intervention
- Identify precursor failure indicators from Phase 1 data (feature-track survival, inlier ratio trends, optical flow variance).
- Build lightweight motion-aware failure prediction module to forecast impending loss.
- Pre-register hypothesis and recovery action before full testing.

---

### Phase 3 — Full Experiment
- Execute head-to-head comparison: baseline vs. failure-prediction intervention across the severity matrix.
- Evaluate ATE, RPE, tracking loss rate, recovery time, drift/meter, and prediction lead time.
- Automated data collection and severity-conditioned plotting.

---

### Phase 4 — Analysis & Report
- Synthesize findings into a 2–4 page research report (question, hypothesis, methodology, results, failure analysis, simulation-scoped conclusions).
- Finalize clean, fully reproducible GitHub repository.

---

## Experimental World Policy

### Phase 0 Policy:
Controlled characterization and diagnostic benchmarking may use `configs/gazebo_maps/exploration_complex/boxworld_obstacles_tight.world` and other controlled diagnostic worlds where appropriate for isolating specific rotational or geometric variables.

### Phase 1+ Forward Policy:
`configs/gazebo_maps/agriculture.world` is designated as the **PRIMARY** research and testing environment for Phase 1 baseline characterization, Phase 2 intervention testing, and all subsequent experimental phases.

The `agriculture.world` environment serves as the project's standard benchmark test ground for baseline characterization, intervention testing, and comparative evaluations unless a specific experiment explicitly requires another controlled environment for diagnostic isolation.

*Note: This is a forward-looking policy. Existing historical Phase 0 datasets and experiments recorded in other environments remain preserved as valid historical references.*

