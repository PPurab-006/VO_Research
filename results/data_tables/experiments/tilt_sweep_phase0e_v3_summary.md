# Phase 0E — Sweep B (Corrected Attitude Amplitude Escalation) Scientific Summary Report

## Executive Summary

This report documents the results of **Sweep B: Corrected Attitude Amplitude Escalation** under Phase 0E characterization using the repaired, validated monocular Visual Odometry (VO) pipeline (`src/minimal_vo.py`). The experiment evaluated escalating single-axis roll attitude oscillation amplitudes ($5^\circ, 10^\circ, 20^\circ, 30^\circ, 40^\circ, 50^\circ$) at $0.5\text{ Hz}$ in the `configs/gazebo_maps/agriculture.world` environment at hover target $Z \approx 2.41\text{ m}$.

### Primary Conclusions

1. **Achieved Axis Verification**:
   The hybrid attitude controller cleanly generated **pure roll oscillation** as the dominant achieved attitude axis across all target amplitudes ($p95 \text{ Roll}: 6.30^\circ - 47.51^\circ$), while parasitic pitch ($p95 \le 0.82^\circ$) and relative yaw ($p95 \le 1.82^\circ$) remained negligible.
2. **Essential Matrix Resilience (`num_inliers_E`)**:
   Essential Matrix RANSAC remained extremely healthy across all six attitude amplitude levels, averaging **$576.3 - 655.7$ inliers per frame**. **No attitude amplitude level triggered the strict 2.0s sliding-window failure criterion under `num_inliers_E`** (`strict_E_fail = FALSE` across all 6 levels, max window low-inlier % $\le 19.35\% \ll 70\%$).
3. **Pose Recovery & Translation Baseline Coupling**:
   Unlike pure yaw holds (where physical translation $t \approx 0$ caused unit-scale depth truncation), dynamic roll oscillation induces physical lateral translation ($a_y = g \tan(\phi)$). This translation baseline prevents depth scale degeneracy ($Z_{\text{unit}} = Z_{\text{true}} / \|t_{\text{true}}\| < 1000\text{m}$), allowing **`recoverPose` accepted inliers to reach $88.70\% - 99.91\%$ of true epipolar inliers** (`strict_pose_fail = FALSE` across all 6 levels).
4. **Phase 0E Completion**:
   With Sweep A (Pure Yaw Escalation) and Sweep B (Attitude Escalation) both completed and passed under the repaired pipeline, **Phase 0E scientific characterization is complete**.

---

## 1. Summary Table Across All Six Attitude Amplitudes

| Target Amp (°) | Achieved Axis | Dominant p95 (°) | Pitch p95 (°) | Yaw p95 (°) | Max HDisp (m) | Z Dev (m) | Camera FPS (Hz) | Matched (Median) | Survival (Median) | Feature Vel (px/fr) | LK Error (px) | $E$ Inliers (Mean) | $E$ Inliers (Median) | $E < 5$ Frac | Pose Inliers (Median) | Pose $< 5$ Frac | Pose/$E$ Ratio | $H$ Inliers (Median) | Strict $E$ Failure | Strict Pose Failure |
| :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| **5** | **ROLL** | 6.30 | 0.71 | 1.82 | 0.285 | 0.18 | 30.3 | 326.0 | 100.0% | 2.58 | 1.07 | 633.2 | 326.0 | 11.90% | 215.0 | 12.23% | 0.8870 | 326.0 | **FALSE** | **FALSE** |
| **10** | **ROLL** | 11.82 | 0.82 | 1.82 | 0.434 | 0.16 | 30.3 | 333.0 | 99.85% | 5.69 | 1.22 | 627.7 | 332.5 | 9.41% | 249.5 | 10.23% | 0.9700 | 327.0 | **FALSE** | **FALSE** |
| **20** | **ROLL** | 20.97 | 0.66 | 1.80 | 1.088 | 0.17 | 30.3 | 434.0 | 99.81% | 9.74 | 1.47 | 655.7 | 434.0 | 7.44% | 310.0 | 7.93% | 0.9907 | 430.0 | **FALSE** | **FALSE** |
| **30** | **ROLL** | 29.86 | 0.48 | 1.78 | 1.547 | 0.14 | 30.3 | 395.0 | 99.78% | 14.55 | 1.90 | 648.2 | 395.0 | 7.26% | 333.5 | 7.92% | 0.9973 | 395.0 | **FALSE** | **FALSE** |
| **40** | **ROLL** | 38.72 | 0.66 | 1.82 | 1.931 | 0.15 | 30.3 | 389.5 | 100.0% | 16.94 | 2.05 | 619.5 | 389.0 | 5.61% | 347.0 | 6.44% | 0.9991 | 389.0 | **FALSE** | **FALSE** |
| **50** | **ROLL** | 47.51 | 0.43 | 1.76 | 2.375 | 0.16 | 30.3 | 422.5 | 100.0% | 21.56 | 2.47 | 576.3 | 418.0 | 5.78% | 360.5 | 6.44% | 0.9989 | 416.0 | **FALSE** | **FALSE** |

---

## 2. Experimental Integrity Check

* **Environment & Spawn**: `agriculture.world` at hover target $(14.0505, -7.5229, 2.4076\text{ m})$.
* **Process Lifecycle**: Fresh 2-stage process cleanup (`pkill -15` $\to$ `pkill -9`) executed between all 6 levels, with unique datasets (`tilt_sweep_v3_<AMP>deg_*.csv`).
* **Frozen VO Parameters**:
  - `distanceThresh = 1000.0`
  - Decoupled `num_inliers_E` and `num_inliers_pose` metrics
  - KLT parameters (`winSize=(21,21)`, `maxLevel=3`), GFTT (`maxCorners=2000`), Essential Matrix RANSAC (`prob=0.999, threshold=1.0`), and camera intrinsics unchanged.

---

## 3. Axis Verification & Motion Analysis

* **Dominant Achieved Axis**: **ROLL** across all six levels.
* **Achieved Roll Amplitude Accuracy**:
  - Target $5^\circ$ $\to$ Achieved p95: **6.30°** (Max: **7.12°**)
  - Target $10^\circ$ $\to$ Achieved p95: **11.82°** (Max: **13.51°**)
  - Target $20^\circ$ $\to$ Achieved p95: **20.97°** (Max: **23.14°**)
  - Target $30^\circ$ $\to$ Achieved p95: **29.86°** (Max: **33.02°**)
  - Target $40^\circ$ $\to$ Achieved p95: **38.72°** (Max: **42.85°**)
  - Target $50^\circ$ $\to$ Achieved p95: **47.51°** (Max: **51.98°**)
* **Cross-Axis Coupling**: Parasitic pitch remained $\le 0.82^\circ$, and relative yaw remained $\le 1.82^\circ$.

---

## 4. KLT Feature Motion & Tracking Behavior

* **Feature Survival Rate**: **$99.81\% - 100.0\%$** across all six levels.
* **Feature Velocity Escalation**:
  - $5^\circ$: $2.58\text{ px/fr}$
  - $10^\circ$: $5.69\text{ px/fr}$
  - $20^\circ$: $9.74\text{ px/fr}$
  - $30^\circ$: $14.55\text{ px/fr}$ (exceeds single-level $10.5\text{ px/fr}$ reference)
  - $40^\circ$: $16.94\text{ px/fr}$
  - $50^\circ$: $21.56\text{ px/fr}$
* **LK Error**: Increased smoothly from $1.07\text{ px}$ at $5^\circ$ to $2.47\text{ px}$ at $50^\circ$.
* **Pyramidal Flow Range**: Pyramidal LK optical flow (`maxLevel=3`) easily accommodated velocities up to $21.56\text{ px/frame}$ without tracking breakdown.

---

## 5. Essential Matrix ($E$) vs. Pose Recovery (`recoverPose`) Behavior

* **Essential Matrix RANSAC (`num_inliers_E`)**:
  Averaged **$576.3 - 655.7$ inliers per frame** across all six levels. Low-inlier fraction ($E < 5$) was $\le 11.90\%$ at $5^\circ$ and improved to $\le 5.78\%$ at $50^\circ$.
* **Pose Recovery (`num_inliers_pose`)**:
  Averaged **$404.7 - 524.2$ inliers per frame**. Valid pose update rate (`num_inliers_pose >= 8`) reached **$87.44\%$ at $5^\circ$ and increased to $93.56\%$ at $50^\circ$**.
* **Pose / $E$ Ratio Behavior**:
  The median ratio `num_inliers_pose / num_inliers_E` increased monotonically with attitude amplitude:
  $$0.8870 (5^\circ) \longrightarrow 0.9700 (10^\circ) \longrightarrow 0.9907 (20^\circ) \longrightarrow 0.9973 (30^\circ) \longrightarrow 0.9991 (40^\circ) \longrightarrow 0.9989 (50^\circ)$$
  * **Mechanism**: Higher roll amplitudes generate larger lateral acceleration ($a_y = g \tan(\phi)$), producing larger inter-frame translation baselines ($t_{\text{true}}$). This baseline lowers unit-scale triangulated depth ($Z_{\text{unit}} = Z_{\text{true}} / t_{\text{true}}$) well below `distanceThresh=1000.0`, allowing $99.9\%$ of true epipolar inliers to pass pose recovery.

---

## 6. Translation & Confound Analysis

* **Horizontal Displacement Scaling**:
  - $5^\circ$: $0.285\text{ m}$ (X span: $0.08\text{m}$, Y span: $0.57\text{m}$)
  - $10^\circ$: $0.434\text{ m}$ (X span: $0.12\text{m}$, Y span: $0.86\text{m}$)
  - $20^\circ$: $1.088\text{ m}$ (X span: $0.21\text{m}$, Y span: $2.17\text{m}$)
  - $30^\circ$: $1.547\text{ m}$ (X span: $0.29\text{m}$, Y span: $3.09\text{m}$)
  - $40^\circ$: $1.931\text{ m}$ (X span: $0.38\text{m}$, Y span: $3.86\text{m}$)
  - $50^\circ$: $2.375\text{ m}$ (X span: $0.46\text{m}$, Y span: $4.75\text{m}$)
* **Altitude Control**: Vertical altitude deviation ($Z_{\text{dev}}$) remained strictly capped at $\le 0.18\text{ m}$ due to total-force thrust compensation.
* **Confound Assessment**: The lateral translation is a physical consequence of roll attitude tilt under gravity-driven flight. Rather than confounding VO, this translation baseline provides the necessary non-zero baseline for metric 3D point triangulation.

---

## 7. Camera & Rendering Performance

* **Mean / Median Camera FPS**: **$30.3\text{ Hz}$** strictly maintained across all six levels. GPU render offload (`ogre2`) prevented frame drops or rendering bottlenecks.

---

## 8. Failure Window Analysis (Strict $\ge 2.0\text{s}$ Sliding Window, $>70\%$ Low Inliers)

* **Strict E-Inlier Failure (`num_inliers_E`)**: **FALSE for all 6 levels**.
  - $5^\circ$: Max low-inlier % = 19.35%
  - $10^\circ$: Max low-inlier % = 14.52%
  - $20^\circ$: Max low-inlier % = 16.13%
  - $30^\circ$: Max low-inlier % = 16.13%
  - $40^\circ$: Max low-inlier % = 9.68%
  - $50^\circ$: Max low-inlier % = 16.13%
* **Strict Pose-Inlier Failure (`num_inliers_pose`)**: **FALSE for all 6 levels**.
  - $5^\circ$: Max low-inlier % = 20.97%
  - $10^\circ$: Max low-inlier % = 16.13%
  - $20^\circ$: Max low-inlier % = 19.35%
  - $30^\circ$: Max low-inlier % = 17.74%
  - $40^\circ$: Max low-inlier % = 11.29%
  - $50^\circ$: Max low-inlier % = 19.35%

---

## 9. Mechanism Interpretation

* **Is higher tilt worse for VO?**: **NO**. In fact, higher roll attitude amplitudes up to $50^\circ$ produce **higher pose inlier counts and higher Pose/$E$ ratios** ($0.8870 \to 0.9989$) because the increased translation baseline resolves unit-scale depth truncation.
* **KLT Optical Flow**: Feature tracking remains $> 99.8\%$ healthy up to $21.56\text{ px/frame}$ feature velocity under pyramidal LK (`maxLevel=3`).

---

## 10. Comparison with Legacy Sweep B

| Aspect | Legacy Sweep B (Unrepaired Pipeline) | Repaired Sweep B v3 (Current Decoupled Pipeline) | Impact |
| :--- | :---: | :---: | :--- |
| **Achieved Axis** | Axis misclassified (commanded velocity induced pitch) | Verified dominant **ROLL** ($p95 \text{ Roll}: 6.3^\circ - 47.5^\circ$) | Axis classification corrected. |
| **Logged Inlier Metric** | Conflated `recoverPose` default count | Decoupled `num_inliers_E` & `num_inliers_pose` | Separated algebraic epipolar inliers from depth filter. |
| **E-RANSAC Inliers** | Reported as failing / 0 | **Mean $576.3 - 655.7$ inliers** (`strict_E_fail = FALSE`) | **Proves $E$ RANSAC is robust up to $50^\circ$ roll.** |
| **Pose Update Rate** | Reported as 0% usable | **$87.44\% - 93.56\%$ usable** (`num_inliers_pose >= 8`) | Valid pose updates restored. |

---

## 11. Final Decision

### **PASS: Sweep B is scientifically usable and Phase 0E is complete.**

* **Reasoning**:
  Sweep B verified that monocular VO under escalating roll attitude oscillation up to $50^\circ$ maintains healthy KLT feature tracking ($> 99.8\%$ survival), robust Essential Matrix RANSAC ($> 570$ mean inliers, `strict_E_fail = FALSE`), and high pose recovery rates ($> 87\%$ valid pose updates, `strict_pose_fail = FALSE`). All pre-flight audit items were satisfied, achieved attitude axis was verified as ROLL, and no failure thresholds were triggered. Phase 0E characterization is complete.
