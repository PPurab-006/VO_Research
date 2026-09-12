# Phase 2D Report: Delayed Triangulation (RD-VIO-Inspired) in Monocular VO

## Executive Summary

Phase 2D evaluates **Delayed Landmark Triangulation**—a pure-rotation mitigation strategy adapted from RD-VIO (Huang et al., arXiv:2310.15072). The underlying principle is **detection + deferral**: during rotation-dominant frames where translational baseline/parallax is insufficient for reliable triangulation, newly detected visual features are held in a `PENDING` queue (tracked via Pyramidal KLT optical flow) and excluded from Essential matrix RANSAC and relative pose solve (`cv2.recoverPose`) until they accumulate sufficient parallax across subsequent non-rotation-dominant frames.

Evaluation across all 7 canonical datasets (`F5_L2_R1..R3`, `F6_L2`, `F9_L2_R1..R3`) yields the following primary empirical conclusions:
1. **F5 (Pure Translation Baseline)**: Shows **zero regression** under Definition (a) ($|\omega_z| > 15.0^\circ/\text{s}$), with 0.0% R-frame fraction, confirming the threshold calibration is clean and safe during pure translation.
2. **F6 (Pure Yaw Rotation) & F9 (Translation + Fast Yaw)**: Delayed triangulation causes **severe performance degradation** (F6 valid pose drops from 74.68% to 27.31%; F9 valid pose drops from 93.04% to 50.29%).
3. **Architectural Insight**: While RD-VIO successfully uses delayed triangulation in a **sliding-window VIO backend with bundle adjustment**, the strategy **fails on a lightweight frame-to-frame monocular VO pipeline**. Deferring new landmarks during sustained yaw starves the 2D-2D Essential matrix solver of active matches, causing feature starvation and solver breakdown.

---

## Mandatory Attribution & Deviations Statement

This mechanism is **ADAPTED** from RD-VIO's pure-rotation detection and delayed-triangulation strategy (Huang et al., arXiv:2310.15072), with the following three explicit architectural deviations:
1. **IMU/Attitude-Based Rotation Detection**: We use IMU body yaw-rate ($|\omega_z| > 15.0^\circ/\text{s}$, reusing our validated EIS-GATED computation) rather than RD-VIO's vision-based bearing-angle test—justified by our access to clean simulated telemetry vs. their noisy consumer-phone IMU use case.
2. **Lightweight Frame-to-Frame Monocular VO Architecture**: Implemented on a frame-to-frame 5-point Essential matrix VO pipeline (`cv2.recoverPose`) rather than a sliding-window VIO backend with keyframes and bundle adjustment—testing whether delayed triangulation benefit transfers to simpler frame-to-frame architectures.
3. **Combination with Image-Level Derotation (EIS-GATED)**: Evaluated standalone (RAW + DT) and in combination with EIS-GATED (homography derotation), an interaction not addressed in the RD-VIO source literature.

---

## R-Frame Detection Characterization (Step 1)

We evaluated two definitions of rotation-dominant frames (R-frames) across the active window ($z \ge 2.0\text{ m}$):
- **Definition (a)**: IMU body yaw rate $|\omega_z| > 15.0^\circ/\text{s}$ (validated EIS-GATED proxy).
- **Definition (b)**: Frame-to-frame translational baseline to scene depth ratio $B/D < 0.005$ (closer to RD-VIO Eq. 11).

| Dataset Run | Active Frames | Mean $|\omega_z|$ | Def (a) R-Frame % | Def (b) R-Frame % ($B/D < 0.005$) | Def (b) R-Frame % ($B/D < 0.010$) |
| :--- | :---: | :---: | :---: | :---: | :---: |
| **F5_L2_R1** | 704 | $0.13^\circ/\text{s}$ | **0.00%** | 24.43% | 34.23% |
| **F5_L2_R2** | 715 | $0.14^\circ/\text{s}$ | **0.00%** | 25.31% | 35.66% |
| **F5_L2_R3** | 716 | $0.15^\circ/\text{s}$ | **0.00%** | 26.26% | 35.47% |
| **F6_L2** | 703 | $31.70^\circ/\text{s}$ | **69.56%** | 98.15% | 98.86% |
| **F9_L2_R1** | 697 | $32.13^\circ/\text{s}$ | **70.73%** | 23.24% | 35.87% |
| **F9_L2_R2** | 712 | $31.37^\circ/\text{s}$ | **68.96%** | 24.16% | 36.66% |
| **F9_L2_R3** | 704 | $31.61^\circ/\text{s}$ | **70.03%** | 24.43% | 36.08% |

### Key Diagnostic Observations:
- **Definition (a)** achieves perfect selectivity: **0.00%** false positive R-frames on pure translation (F5), while flagging **~70%** of frames during high yaw maneuvers (F6 and F9).
- **Definition (b)** suffers from high false positive rates on pure translation (F5: 24–26% at $B/D < 0.005$), because small frame-to-frame velocity fluctuations or time step variances cause the instantaneous baseline $B$ to drop below 5 mm, falsely triggering R-frame status even when motion is purely translational.

---

## Quantitative Evaluation Results (Step 3)

Evaluation metrics across canonical active window ($z \ge 2.0\text{ m}$):

| Dataset Run | RAW Val% | RAW + DT (Def a) | RAW + DT (Def b) | EIS-GATED Val% | GATED + DT (Def a) | GATED + DT (Def b) | R-Frame % (a) | Mean Pending (a) | Mean Promo Latency (a) |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| **F5_L2_R1** | 90.62% | 90.62% | 72.73% | 90.62% | 90.62% | 72.73% | 0.0% | 0.0 | 0.0 f |
| **F5_L2_R2** | 90.91% | 90.91% | 71.75% | 89.79% | 89.79% | 71.05% | 0.0% | 0.0 | 0.0 f |
| **F5_L2_R3** | 90.08% | 90.08% | 73.04% | 89.66% | 89.66% | 71.09% | 0.0% | 0.0 | 0.0 f |
| **F6_L2** | 74.68% | 27.31% | 1.42% | 72.12% | 23.76% | 1.42% | 71.0% | 846.0 | 33.3 f |
| **F9_L2_R1** | 93.54% | 55.52% | 75.61% | 92.25% | 40.46% | 74.75% | 70.7% | 602.0 | 24.9 f |
| **F9_L2_R2** | 93.82% | 46.21% | 73.31% | 92.28% | 41.29% | 75.00% | 69.5% | 695.6 | 27.5 f |
| **F9_L2_R3** | 91.76% | 49.15% | 73.58% | 91.90% | 44.46% | 73.30% | 69.7% | 675.3 | 25.7 f |

---

## Grouped Aggregate Statistics

### F5 (Translation Baseline)
- **RAW Valid Pose %**: $90.54\%$
- **RAW + Delayed-Triangulation (Def a)**: $90.54\%$ ($\Delta = +0.00\%$)
- **EIS-GATED Valid Pose %**: $90.03\%$
- **EIS-GATED + Delayed-Triang (Def a)**: $90.03\%$ ($\Delta = +0.00\%$)
- **RAW Pose/E Ratio**: $0.8165$ $\rightarrow$ **RAW + DT Pose/E Ratio**: $0.8165$

### F6 (Pure Yaw Rotation)
- **RAW Valid Pose %**: $74.68\%$
- **RAW + Delayed-Triangulation (Def a)**: $27.31\%$ ($\Delta = -47.37\%$)
- **EIS-GATED Valid Pose %**: $72.12\%$
- **EIS-GATED + Delayed-Triang (Def a)**: $23.76\%$ ($\Delta = -48.36\%$)
- **RAW Pose/E Ratio**: $0.4133$ $\rightarrow$ **RAW + DT Pose/E Ratio**: $0.1755$

### F9 (Translation + Fast Yaw)
- **RAW Valid Pose %**: $93.04\%$
- **RAW + Delayed-Triangulation (Def a)**: $50.29\%$ ($\Delta = -42.75\%$)
- **EIS-GATED Valid Pose %**: $92.14\%$
- **EIS-GATED + Delayed-Triang (Def a)**: $42.07\%$ ($\Delta = -50.07\%$)
- **RAW Pose/E Ratio**: $0.8173$ $\rightarrow$ **RAW + DT Pose/E Ratio**: $0.4557$

---

## Verdict & Direct Question Answers (Step 4)

### Question 1: Does RAW + Delayed-Triangulation alone improve F9/F6 over plain RAW?
**No.** It causes severe performance regression. Pose validity drops on F6 from **74.68% to 27.31%** (Def a) and on F9 from **93.04% to 50.29%** (Def a).

### Question 2: Does EIS-GATED + Delayed-Triangulation improve over EIS-GATED alone?
**No.** Combining EIS-GATED with Delayed-Triangulation degrades performance further (F9: **42.07%** vs **92.14%**; F6: **23.76%** vs **72.12%**). The two mechanisms do **not** compose positively. EIS-GATED stabilizes image flow by bypassing derotation during rotation, but deferring landmark initialization leaves the pipeline with insufficient active points when rotation ends.

### Question 3: Does F5 (already-healthy baseline) show any regression from adding delayed triangulation?
**No regression under Definition (a).** Under Definition (a), F5 has an R-frame fraction of **0.00%**, resulting in **90.54%** pose validity (identical to plain RAW). Under Definition (b), however, F5 drops to **72.51%** due to false positive R-frame triggers caused by velocity fluctuations.

### Question 4: Compare Definition (a) vs Definition (b).
Definition (a) ($|\omega_z| > 15.0^\circ/\text{s}$) is far superior to Definition (b) ($B/D < 0.005$). Definition (a) achieves 0% false positives on pure translation (F5), whereas Definition (b) falsely flags 24–26% of frames in F5, degrading F5 baseline performance by over 18%.

### Question 5: Final Verdict Category
- **F9 (Standalone RAW + DT)**: `"not yet demonstrated"` (Severe negative regression: 93.04% $\rightarrow$ 50.29%).
- **F9 (EIS-GATED + DT)**: `"not yet demonstrated"` (Severe negative regression: 92.14% $\rightarrow$ 42.07%).
- **F5 (Standalone RAW + DT)**: `"wash"` (Identical performance: 90.54% vs 90.54%, zero change because R-frame fraction is 0.0%).
- **F5 (EIS-GATED + DT)**: `"wash"` (Identical performance: 90.03% vs 90.03%).

---

## Architectural Analysis: Why Delayed Triangulation Fails in Monocular VO

1. **Sliding-Window VIO (RD-VIO) vs. Frame-to-Frame Monocular VO**:
   - In RD-VIO, landmark estimation is handled by a sliding-window bundle adjustment (BA) backend over keyframes spanning several seconds. Pending features continue to be tracked and optimized over a long temporal window.
   - In our monocular frame-to-frame VO pipeline (`cv2.recoverPose`), relative pose is estimated purely between frame $k-1$ and frame $k$.
2. **Feature Starvation During Sustained Rotation**:
   - During sustained yaw maneuvers (which last 50–100+ frames in F6 and F9), new features detected in R-frames are assigned `PENDING` status.
   - Because no non-R frames occur during the maneuver, `PENDING` features are never promoted to `ACTIVE`.
   - Meanwhile, pre-existing `ACTIVE` features gradually drift or move out of the camera FOV.
   - As a result, the number of usable `ACTIVE` features rapidly falls below the 8-inlier requirement for `cv2.recoverPose`, causing pose estimation to collapse completely.
