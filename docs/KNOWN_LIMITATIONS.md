# Known Limitations & Methodological Constraints

This document summarizes known methodological constraints, dataset provenance details, statistical limits, and pipeline fixes across the VO_Research benchmark suite.

---

## 1. Experimental Design & Statistical Scope
- **Sample Size ($n=3$ per cell)**: All core and exploratory trajectory cells are evaluated across $n=3$ independent flight repeats per condition. Small sample size limits statistical power for detecting small effect sizes.
  - *Supporting Files*: [paired_stats.csv](../results/analysis/paired_stats.csv), [phase3_controlled_experiment_report.md](../results/reports/phase3/phase3_controlled_experiment_report.md#L88-L118)
- **Multiple Comparisons**: 36 unadjusted paired RAW vs. EIS-GATED comparisons (24 core matrix cells + 12 exploratory cells across valid pose %, ATE RMSE, and scale-normalized RPE) are reported without Bonferroni or FDR corrections. Exactly 1 comparison (`F10_L3` valid pose %, $p = 0.0327$) achieves $p < 0.05$.
  - *Supporting Files*: [fig_06_core_matrix.csv](../figures/data/fig_06_core_matrix.csv), [paired_stats.csv](../results/analysis/paired_stats.csv)
- **Single Simulation Environment & Vehicle**: All datasets were collected within a single Gazebo simulation world (Agriculture/Race Track map) using PX4 SITL and a single monocular camera configuration ($f_x = 539.9$).
  - *Supporting Files*: [fly_phase1_motion.py](../src/flight/fly_phase1_motion.py), [fig_12_system_diagram.csv](../figures/data/fig_12_system_diagram.csv)

---

## 2. Telemetry & Sensor Assumptions
- **Simulator Ground-Truth Attitude**: EIS derotation uses exact simulator ground-truth quaternions (`dataset_gt.csv`) via SLERP interpolation rather than an estimated attitude state from an onboard IMU state estimator (e.g. PX4 EKF2).
  - *Supporting Files*: [eis_preprocessor.py](../src/core/eis_preprocessor.py#L79-L124), [fig_12_system_diagram.py](../figures/make_fig_12_system_diagram.py)
- **Ground-Truth Heading Discontinuities**: Exactly 3 of the 36 core/exploratory runs contain $>10^\circ$ heading jumps between consecutive telemetry rows: `p3_F6_L2_R1` (4 jumps), `p3_F6_L2_R3` (4 jumps), and `p3_F10_L3_R2` (11 jumps).
  - *Supporting Files*: [gt_heading_audit.csv](../results/analysis/gt_heading_audit.csv), [fig_03_achieved_yaw.csv](../figures/data/fig_03_achieved_yaw.csv)
- **Non-Uniform GT Sampling & Sub-5ms Pairs**: Raw GT telemetry contains repeated timestamps and non-uniform row spacing ($dt < 5\,\text{ms}$ on 5-12% of rows), requiring timestamp deduplication and uniform 50 Hz grid resampling for velocity and yaw rate differentiation.
  - *Supporting Files*: [gt_heading_audit.csv](../results/analysis/gt_heading_audit.csv), [make_fig_02_motion_profiles.py](../figures/make_fig_02_motion_profiles.py)
- **Active Window Altitude Dips Below 2m**: The canonical active window (defined from first to last GT sample with $pos\_z \ge 2.0\,\text{m}$) includes brief altitude dips below 2.0 m in three runs: `p3_F10_L3_R3` (123 rows), `p3_F11_L2_R1` (25 rows), and `p3_F10_L3_R2` (14 rows), while `p3_F6_L2_R1` has 0 dips.
  - *Supporting Files*: [gt_heading_audit.csv](../results/analysis/gt_heading_audit.csv), [gt_heading_audit.py](../src/analysis/gt_heading_audit.py)

---

## 3. Algorithm & Pipeline Behavior
- **Absence of Gating Threshold Knee**: Feature tracking loss rate varies smoothly (4.62% to 8.55%) across binned yaw rates without a sharp knee at 15°/s, and threshold sweep ATE RMSE values show no statistically distinguishable knee at $n=3$.
  - *Supporting Files*: [fig_05_threshold_bins.csv](../figures/data/fig_05_threshold_bins.csv), [fig_05_threshold_sweep.csv](../figures/data/fig_05_threshold_sweep.csv)
- **Offline Replay Evaluation**: Threshold sweeps and mechanism comparisons are evaluated via offline frame replay on recorded image datasets rather than closed-loop real-time flight.
  - *Supporting Files*: [threshold_sweep.py](../src/analysis/threshold_sweep.py), [run_offline_vo.py](../src/pipelines/run_offline_vo.py)
- **DELAYED-TRI Dormancy on Control Cells**: On low-yaw control cells (HOVER_L0, F3_L2, F7_L2, F8_L2), zero frames exceed the 15°/s gating threshold, leaving DELAYED-TRI dormant and identical to RAW/EIS-GATED.
  - *Supporting Files*: [dt_dormancy.csv](../results/analysis/dt_dormancy.csv), [fig_08_dt_artifact.csv](../figures/data/fig_08_dt_artifact.csv)
- **VFO Predictor Failure Target Ratio**: Visual Field Observatory (VFO) lead-lag predictor correlations are evaluated against binary pose loss failure targets representing ~7% of active flight frames.
  - *Supporting Files*: [vfo_leadlag_f9.csv](../results/analysis/vfo_leadlag_f9.csv), [fig_10_vfo_leadlag.csv](../figures/data/fig_10_vfo_leadlag.csv)

---

## 4. Dataset Provenance & Environment Pins
- **Dataset Provenance Heterogeneity**: Core matrix datasets originate from two distinct recording campaigns: `F5_L2` and `F9_L2` use `phase2a_` flight recordings; `F1_L2`, `F2_L2`, `F4_L2`, `F6_L2`, `F10_L3`, and `F11_L2` use `p3_` flight recordings.
  - *Supporting Files*: [dataset_provenance.csv](../results/analysis/dataset_provenance.csv), [fig_06_core_matrix.csv](../figures/data/fig_06_core_matrix.csv)
- **`run_offline_vo.py` Modifications**: Script modifications were restricted strictly to adding missing python module imports (`os`, `sys`, `cv2`, `numpy`, `pandas`, `scipy`), introducing zero numerical or algorithmic changes to tracking logic.
  - *Supporting Files*: [run_offline_vo.py](../src/pipelines/run_offline_vo.py)
- **OpenCV Version Pin**: Offline trajectory evaluation reproduced under OpenCV 4.13; sensitivity of frame-level KLT/RANSAC replay to the specific OpenCV 4.10.0 pin has not been independently swept.
  - *Supporting Files*: [environment.yml](../environment.yml)
