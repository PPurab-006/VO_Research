# Phase 1 Experiment Plan: Multidimensional Baseline Monocular VO Characterization in Agriculture World

## 1. Epistemic Scope Statement

> **Epistemic Scope & Observational Design Declaration**:  
> Phase 1 is an observational characterization. Its goal is to identify which Tier 2 variables show consistent, temporally-leading association with Tier 1 degradation across repeated trials and multiple motion regimes (Sweep A-style pure yaw and Sweep B-style roll attitude escalation). Motion severity, camera FPS, and physical translation displacement are not independently randomized in this design, so Phase 1 cannot establish causation. Any Tier 2 $\leftrightarrow$ Tier 1 relationship that is strong, consistent across sweeps, and robust to covariate adjustment is a candidate for causal testing via a designed intervention in Phase 2 — not a proven cause.

---

## 2. Scientific Objective & Scope

The primary objective of **Phase 1: Baseline Characterization** is to establish a rigorous, multidimensional quantitative map of un-fused monocular Visual Odometry (VO) performance, precursor signals, and operational failure boundaries across dynamic multicopter flight profiles in the benchmark environment (`configs/gazebo_maps/agriculture.world`).

Historically, Phase 1 initial drafts leaned heavily on inlier counts (`num_inliers_E` / `num_inliers_pose`) as the de facto primary signal of VO degradation. This conflated a **pipeline-internal health metric** with the actual outcome of interest: whether the recovered VO pose is trustworthy relative to ground truth. 

This revised plan restructures Phase 1 around a strict **two-tier variable hierarchy**:
1. **Tier 1 Outcome Variables** explicitly define VO tracking performance, trajectory accuracy, and operational breakdown relative to ground truth (GT).
2. **Tier 2 Precursor Variables** represent pipeline-internal state and geometric indicators evaluated empirically for their ability to predict Tier 1 outcomes.

### Integration of Phase 0E Forensic Decisions
- **Benchmark Environment**: `configs/gazebo_maps/agriculture.world` is enforced as the sole primary environment to eliminate world-construction confounds (as established in Phase 0E).
- **Operating Envelope Parameter**: OpenCV `cv2.recoverPose()` explicitly uses `distanceThreshold = 1000.0` (validated in Phase 0E Entry 17) to prevent small-baseline unit-scale triangulated depth truncation ($Z_{\text{unit}} = Z_{\text{true}} / \|t\|$).
- **Metric Separation**: `num_inliers_E` (algebraic 5-point RANSAC correspondence quality) and `num_inliers_pose` (accepted 3D depth-filtered inliers) are tracked as distinct variables.
- **Flight Kinematics**: Achieved kinematics from high-frequency GT logs are used exclusively; commanded flight setpoints ($v_{\text{cmd}}, \phi_{\text{cmd}}, \omega_{\text{cmd}}$) are never treated as ground truth.

---

## 3. Two-Tier Variable Hierarchy & Metric Classification

### Tier 1 — Outcome Variables (True VO Performance & Degradation)
Tier 1 metrics define what "VO failure" and "VO degradation" actually mean. All precursor hypotheses are evaluated against their ability to forecast these variables.

- **Relative Pose Error (RPE)**:
  - *Translation RPE ($e_{\text{RPE}, t}$)*: Frame-to-frame or short-window ($k=1, 10$ frames) relative translation error relative to aligned ground truth ($\text{meters}$).
  - *Rotation RPE ($e_{\text{RPE}, R}$)*: Frame-to-frame or short-window relative rotation step error ($\text{degrees}$).
- **Absolute Trajectory Error (ATE) & Drift Rate**:
  - *ATE RMSE*: Per-run trajectory error after Sim(3) Umeyama alignment ($\text{meters}$).
  - *Drift Rate*: Accumulated alignment-free error growth per meter traveled ($\text{m/m}$).
- **Pose-Update Continuity / Validity (Secondary Tier 1 Outcome)**:
  - *Valid Update Flag*: Boolean frame status defined as $\text{num\_inliers\_pose} \ge 8$. This is treated as a secondary Tier 1 outcome representing pose recovery availability (not a Tier 2 precursor), because when pose recovery fails, metric trajectory updates cease.

### Tier 2 — Candidate Precursor / Explanatory Variables
Tier 2 metrics are internal pipeline health indicators. They are **tested hypothesis candidates**, not assumed proxies, for predicting Tier 1 outcome degradation.

- **Essential Matrix RANSAC Inliers (`num_inliers_E`) & E Inlier Ratio**:
  - `num_inliers_E`: Absolute count of correspondences satisfying the 5-point algebraic epipolar constraint $x_2^T E x_1 = 0$.
  - `inlier_ratio_E`: $N_E / N_{\text{matched}}$ (ratio of epipolar inliers to KLT matched features).
- **Pose Recovery Inliers (`num_inliers_pose`) & Pose/E Ratio**:
  - `num_inliers_pose`: Count of correspondences passing cheirality ($Z > 0$) and depth thresholding ($Z_{\text{unit}} \le 1000.0\text{m}$).
  - `pose_E_ratio`: $N_{\text{pose}} / N_E$ (depth filter agreement ratio; primary indicator of small-baseline or geometric ill-conditioning).
- **Feature Survival Ratio ($S_f$)**:
  - $S_f = N_{\text{matched}} / N_{\text{tracked\_prev}}$ (fraction of keypoints successfully tracked from the previous frame).
- **Image-Space Feature Velocity ($v_{\text{px}}$)**:
  - Mean pixel displacement per frame across tracked features ($\bar{v}_{\text{px}} = \frac{1}{N} \sum \|\mathbf{x}_i^{(k)} - \mathbf{x}_i^{(k-1)}\|_2\text{ px/frame}$).
- **Mean KLT Tracking Residual ($\bar{e}_{\text{lk}}$)**:
  - Central-tendency tracking residual (in pixels) from Pyramidal Lucas-Kanade optical flow.  
  - *Distribution Policy*: The empirical distribution shape (skewness, outliers, reset artifacts) must be inspected before trusting the mean, to avoid past issues where feature re-detection resets biased mean residuals.
- **Frame-to-Frame Physical Baseline ($t_{\text{baseline}}$)**:
  - Inter-frame ground truth translation step magnitude ($\|p_k - p_{k-1}\|_2\text{ meters}$).

### Covariates (Logged & Statistically Adjusted For)
Covariates are nuisance or environmental factors that influence VO execution. They are logged and adjusted for in statistical models, not claimed as primary experimental findings.

- **Achieved Motion Kinematics**:
  - Achieved angular velocity magnitude ($\|\boldsymbol{\omega}\||_2\text{ deg/s}$).
  - Achieved translational velocity ($v_{\text{achieved}}\text{ m/s}$) and total achieved displacement ($d_{\text{achieved}}\text{ meters}$).
- **Camera Timing & Simulator Health**:
  - Achieved camera FPS ($f_{\text{cam}}\text{ Hz}$) and inter-frame timestamp delta ($\Delta t_{\text{frame}}\text{ ms}$).
- **Flight Corridor Geometry**:
  - Cruise altitude deviation ($\Delta Z = |Z - Z_{\text{target}}|\text{ meters}$).

### Diagnostic-Only Variables (Per-Run Sanity Checks)
- **Raw Detected Feature Count ($N_{\text{gftt}}$)**:
  - Count of Good Features to Track corners detected ($N \le 2000$). Used strictly as a per-run sanity check for image exposure/texture density. Because $N_{\text{gftt}}$ depends heavily on local scene texture density rather than flight dynamics, it is **invalid for cross-condition comparison** and will not be used in correlation analyses.

### Explicitly Excluded Metrics (Exclusion Rationale)
The following metrics are explicitly dropped from Phase 1 to prevent metric redundancy and ungrounded ad-hoc indices:
1. **Epipolar Residual Statistics**: Algebraically redundant with `inlier_ratio_E`. Dropped unless a specific edge case demonstrates functional divergence.
2. **Spatial Distribution / Feature Clustering Index**: Defer until a cheap, well-defined, standardized scalar metric is available. Ad-hoc spatial entropy metrics will not be invented for Phase 1.
3. **Translation Direction Error**: Redundant with translation RPE ($e_{\text{RPE}, t}$).
4. **Median KLT Error**: Redundant alongside Mean KLT Error. Mean KLT Error is selected as the single central-tendency metric (subject to distribution shape validation).

---

## 4. Dedicated Healthy-Baseline Condition & Noise Floor

To establish a principled, data-driven foundation for the VO degradation hierarchy, Phase 1 begins with a dedicated **Healthy-Baseline Condition**:

- **Flight Profile**: Zero-commanded motion stationary hover ($v_{\text{cmd}} = 0\text{ m/s}, \omega_{\text{cmd}} = 0^\circ/\text{s}$), achieving stationary physical flight ($v_{\text{achieved}} < 0.05\text{ m/s}$, $\omega_{\text{achieved}} < 1.0^\circ/\text{s}$) at cruise altitude $Z = 2.41\text{ m}$ in `agriculture.world`.
- **Replication**: 5 independent trials at distinct fixed spawn positions and headings across the benchmark corridor.
- **Purpose**: Calculate the empirical baseline noise floor for Tier 1 metrics under purely static sensor quantization and optical flow noise:
  - Baseline Translation RPE: Mean $\mu_{\text{RPE}, t}$ and Standard Deviation $\sigma_{\text{RPE}, t}$.
  - Baseline Rotation RPE: Mean $\mu_{\text{RPE}, R}$ and Standard Deviation $\sigma_{\text{RPE}, R}$.

> **Level 1 vs. Healthy Baseline Distinction Policy**:  
> Zero-motion hover ($v_{\text{cmd}} = 0\text{ m/s}, \omega_{\text{cmd}} = 0^\circ/\text{s}$) measures pure static sensor quantization noise ($v_{\text{px}} < 0.5\text{ px/fr}, t_{\text{baseline}} < 0.167\text{ cm/fr}$). Level 1 motion ($v = 0.5\text{ m/s}$, $10^\circ/\text{s}$ yaw or $5^\circ$ roll) introduces $10\times$ higher translation baseline ($1.67\text{ cm/fr}$) and $15-16\times$ higher optical feature velocity ($6.0 - 7.85\text{ px/fr}$).  
> - **Usability**: Level 1 data is fully usable as a distinct severity point for Tier 2 precursor/explanatory variables ($v_{\text{px}}, t_{\text{baseline}}, N_{\text{pose}}/N_E$).  
> - **Tier 1 Outcome Classification**: Because Level 1 Tier 1 RPE ($e_{\text{RPE}} \approx 0.03 - 0.05\text{m}$) falls within $2\sigma$ of baseline hover noise bounds ($\mu_{\text{base}} \approx 0.02\text{m}, \sigma_{\text{base}} \approx 0.015\text{m}$), Level 1 Tier 1 RPE is **statistically indistinguishable from baseline noise ($e_{\text{RPE}} \le \mu_{\text{base}} + 2\sigma_{\text{base}}$)**, formally placing Level 1 into Level 1 (Healthy) of the degradation hierarchy.

All degradation level boundaries in Section 5 are anchored directly to these baseline noise statistics ($\mu, \sigma$). The threshold values are **fixed prior to executing Phase 1 motion sweeps** and will not be adjusted post-hoc.

---

## 5. Baseline-Anchored Degradation Hierarchy

Rather than defining failure by arbitrary inlier counts, Phase 1 defines a 5-stage degradation hierarchy anchored directly to Tier 1 RPE outcomes:

| Degradation Level | Name | Tier 1 Operational Definition (Fixed A Priori) | Physical & Geometric Interpretation |
| :---: | :---: | :--- | :--- |
| **Level 1** | **Healthy** | RPE within baseline noise band ($e_{\text{RPE}} \le \mu_{\text{base}} + 2\sigma_{\text{base}}$). Continuous pose updates (`num_inliers_pose >= 8`). Includes Level 1 sweep data. | Nominal VO tracking. Parallax and optical flow are cleanly resolved within noise bounds. |
| **Level 2** | **Early Degradation** | RPE elevated above baseline ($\mu_{\text{base}} + 2\sigma_{\text{base}} < e_{\text{RPE}} \le \mu_{\text{base}} + 4\sigma_{\text{base}}$). Pose updates remain 100% continuous. | Initial estimation stress. Subtle drift accumulation without pose update drops. |
| **Level 3** | **Significant Degradation** | RPE substantially elevated ($e_{\text{RPE}} > \mu_{\text{base}} + 4\sigma_{\text{base}}$) AND/OR intermittent pose update gaps ($10\% \le \text{invalid frames} < 50\%$), but system recovers. | Severe motion stress or transient low-baseline geometry causing localized tracking errors. |
| **Level 4** | **Near-Failure** | Sustained pose update invalidity ($50\% \le \text{invalid frames} < 70\%$) over a 2.0s window; RPE trending toward divergence. | Imminent tracking collapse. Precursor signals reach critical bounds. |
| **Level 5** | **Sustained Failure** | Sliding window criterion: $>70\%$ of processed frames have $<5$ inliers over a $\ge 2.0\text{s}$ sim-time window, **cross-checked against actual RPE divergence**. | Operational proxy for total tracking collapse. |

> **Reframing of Inlier-Based Failure Criterion**:  
> The $>70\%$ low-inlier sliding window criterion ($T_{\text{window}} \ge 2.0\text{s}$) established in Phase 0E is retained as an **operational proxy for Tier 4/5 failure**, but it is no longer the definition of failure itself. In Phase 1, any trigger of this criterion must be cross-checked against actual RPE divergence in that same window to verify true geometric breakdown.

---

## 6. Motion Sweep Matrix & Experimental Control Policy

### Motion Severity Matrix (Restored 6 Levels Per Sweep)

Phase 1 evaluates flight dynamics across 6 structured severity levels for both Sweep A (Yaw Rotation) and Sweep B (Roll Attitude):

#### Sweep A: Pure Yaw Angular Velocity Escalation
| Severity Level | Nominal Target Velocity ($v$) | Yaw Angular Velocity ($\omega_z$) | Target Duration | Primary Motion Stress |
| :---: | :---: | :---: | :---: | :--- |
| **Level 1** | $0.5\text{ m/s}$ | $10^\circ/\text{s}$ | $20.0\text{s}$ | Quasi-static rotation, low feature velocity |
| **Level 2** | $1.0\text{ m/s}$ | $20^\circ/\text{s}$ | $20.0\text{s}$ | Low-moderate rotation |
| **Level 3** | $1.5\text{ m/s}$ | $40^\circ/\text{s}$ | $20.0\text{s}$ | Moderate rotation, approaching LK boundary |
| **Level 4** | $2.5\text{ m/s}$ | $80^\circ/\text{s}$ | $20.0\text{s}$ | High rotation, LK search window stress |
| **Level 5** | $3.5\text{ m/s}$ | $120^\circ/\text{s}$ | $20.0\text{s}$ | Dynamic rotation, GPU rendering load |
| **Level 6** | $5.0\text{ m/s}$ | $180^\circ/\text{s}$ | $20.0\text{s}$ | Extreme agility rotation, near-limit tracking |

#### Sweep B: Roll Attitude Oscillation Escalation
| Severity Level | Nominal Target Velocity ($v$) | Roll Oscillation Amplitude ($\phi$) | Oscillation Frequency ($f$) | Primary Motion Stress |
| :---: | :---: | :---: | :---: | :--- |
| **Level 1** | $0.5\text{ m/s}$ | $5^\circ$ | $0.5\text{ Hz}$ | Quasi-static tilt, low lateral acceleration |
| **Level 2** | $1.0\text{ m/s}$ | $10^\circ$ | $0.5\text{ Hz}$ | Low-moderate roll tilt dynamics |
| **Level 3** | $1.5\text{ m/s}$ | $20^\circ$ | $0.5\text{ Hz}$ | Moderate tilt acceleration & lateral displacement |
| **Level 4** | $2.5\text{ m/s}$ | $30^\circ$ | $0.5\text{ Hz}$ | High tilt dynamics & feature velocity |
| **Level 5** | $3.5\text{ m/s}$ | $40^\circ$ | $0.5\text{ Hz}$ | Dynamic tilt agility & FOV tilt displacement |
| **Level 6** | $5.0\text{ m/s}$ | $50^\circ$ | $0.5\text{ Hz}$ | Maximum dynamic roll tilt, extreme agility stress |

### Confound Handling Policy

| Confound / Nuisance Factor | Handling Method | Rationale & Operational Protocol |
| :--- | :---: | :--- |
| **Scene Heterogeneity in `agriculture.world`** | **Experimental Control & Spatial Containment** | All flights advance along the primary flight axis ($X \in [5.0\text{m}, 25.0\text{m}]$, $Z \approx 2.41\text{m}$, $\psi = 0^\circ$). **Sweep B Roll Path Divergence**: Roll attitude oscillation ($\phi(t) = A \sin(2\pi f t)$) generates lateral acceleration ($a_y = g \tan\phi$), causing lateral sway displacement ($Y \in [-10.0\text{m}, -5.0\text{m}]$). At Level 6 ($50^\circ$ roll), lateral sway reaches $\pm 2.5\text{m}$ into adjacent crop rows. Scene-heterogeneity confounding across roll levels is **substantially reduced** relative to unconstrained flight by containing all flights to a single $20\text{m} \times 5\text{m}$ corridor volume, but **cannot be completely eliminated** because physical roll dynamics inherently produce lateral spatial displacement. |
| **Camera FPS Drops at High Motion Severity** | **Covariate Adjustment** | GPU rendering load under rapid scene rotation causes camera FPS drops (known simulator bottleneck, not physical motion blur — see `failure_log.md` Entry 16). Camera FPS / $\Delta t_{\text{frame}}$ is logged and included as a statistical covariate in all Tier B analyses. |
| **Roll-Translation Coupling in Sweep B** | **Covariate Adjustment** | Dynamic roll tilt ($a_y = g \tan\phi$) inherently induces physical lateral displacement ($d_{\text{achieved}}$). Total displacement is logged per frame; Sweep B Tier B correlations are reported both **raw and covariate-adjusted** for displacement. |
| **Physical Translation Baseline Variation** | **Mechanism Component (Tier A)** | Physical baseline variation is driven by flight motion and directly affects triangulated point depth. It is part of the causal mechanism under study, so it is classified as a Tier A variable, not a confound. |

---

## 7. Three-Tier Correlation & Association Plan

To avoid flat correlation matrices that double-count evidence or report trivial composite relationships, Phase 1 structures statistical analysis into three explicit tiers:

```
[Covariates (Tier C)] ─────── Confound Adjustments ───────► [Tier 1 Outcomes (RPE/ATE)]
         │                                                            ▲
         ▼                                                            │
[Tier 2 Explanatory Vars] ─── Tier A Mechanism Links ───► [Tier 2 Explanatory Vars]
         │                                                            │
         └─────────────────── Tier B Predictive Links ────────────────┘
```

### Tier A: Mechanism Chain (Tier 2 $\leftrightarrow$ Tier 2)
Tests intermediate physical-to-optical linkages in the pipeline:
1. Achieved Angular Velocity ($\omega$) $\leftrightarrow$ Feature Velocity ($v_{\text{px}}$)
2. Achieved Angular Velocity ($\omega$) $\leftrightarrow$ Feature Survival Ratio ($S_f$)
3. Feature Velocity ($v_{\text{px}}$) $\leftrightarrow$ Mean KLT Residual ($\bar{e}_{\text{lk}}$)
4. Physical Baseline ($t_{\text{baseline}}$) $\leftrightarrow$ Pose/E Ratio ($N_{\text{pose}} / N_E$)
5. Feature Velocity ($v_{\text{px}}$) $\leftrightarrow$ E Inlier Ratio ($N_E / N_{\text{matched}}$)

### Tier B: Predictive Value (Tier 2 $\leftrightarrow$ Tier 1 — Core of Phase 1)
Evaluates candidate precursor capabilities to forecast true VO degradation (RPE).  
> **Headline Metric Designation**: **Pose/E Ratio ($N_{\text{pose}} / N_E$) is designated as the PRIMARY headline Tier B predictor**. E Inlier Ratio ($N_E / N_{\text{matched}}$) and Homography Inlier Ratio are reported strictly as secondary corroborating diagnostics.

1. **[PRIMARY HEADLINE]** Pose/E Ratio ($N_{\text{pose}} / N_E$) $\leftrightarrow$ Translation RPE ($e_{\text{RPE}, t}$)
2. **[Corroborating Diagnostic]** E Inlier Ratio ($N_E / N_{\text{matched}}$) $\leftrightarrow$ Translation RPE ($e_{\text{RPE}, t}$)
3. Feature Survival Ratio ($S_f$) $\leftrightarrow$ Translation RPE ($e_{\text{RPE}, t}$)
4. Mean KLT Residual ($\bar{e}_{\text{lk}}$) $\leftrightarrow$ Translation RPE ($e_{\text{RPE}, t}$)
5. Physical Baseline ($t_{\text{baseline}}$) $\leftrightarrow$ Translation RPE ($e_{\text{RPE}, t}$)

> **Non-Redundancy Constraint**:  
> The relationship between Angular Velocity ($\omega$) and E Inlier Ratio ($N_E / N_{\text{matched}}$) will **NOT** be reported as an independent headline correlation. It is a composite of two already-decomposed Tier A links ($\omega \to v_{\text{px}}$ and $v_{\text{px}} \to N_E$) and would double-count empirical evidence.

### Tier C: Confound Checks (Covariates $\leftrightarrow$ Everything)
Verifies that precursor relationships are not artifacts of simulator rate drops or parasitic translation:
1. Camera FPS / Timestamp Delta ($\Delta t_{\text{frame}}$) $\leftrightarrow$ Each Tier 2 variable AND $\leftrightarrow$ RPE directly.
2. Achieved Displacement ($d_{\text{achieved}}$) $\leftrightarrow$ RPE directly (specifically verifying roll/translation coupling in Sweep B).

---

## 8. Statistical Analysis Plan & Operational Rules

### 1. Empirical Distribution Inspection
Before reporting mean or median summary statistics, full per-condition empirical distributions (histograms, boxplots, skewness checks) must be generated for all Tier 1 and Tier 2 variables. Summary statistics will only be trusted after verifying distribution normality or identifying multi-modal reset artifacts.

### 2. Visualization & Scatter Binned Summaries
- **Tier B Scatter Plots**: Bivariate scatter plots of each Tier 2 candidate vs. RPE, color-coded by motion severity level.
- **Severity-Binned Summaries**: Reporting mean, median, 95th-percentile bounds, and standard deviation ($\mu \pm \sigma$) per severity level cell.

### 3. Lead / Lag Cross-Correlation (Operational Precursor Test)
To qualify as a genuine **precursor** (rather than a simultaneous co-occurrence), a Tier 2 candidate must demonstrate temporally leading correlation with Tier 1 degradation:
$$\rho(k) = \text{Corr}\left(\text{Tier2}(t), \text{RPE}(t + k)\right)$$
- Evaluated for small positive frame lags $k \in \{1, 2, 3, 5, 10\}$ (corresponding to $\sim 33\text{ms} - 330\text{ms}$ lead times).
- A variable qualifies as a candidate precursor if $\rho(k)$ peaks at $k > 0$ with strong effect size.

### 4. Covariate Adjustment via Multiple Regression
Tier B relationships will be evaluated using partial correlation and simple multiple linear regression:
$$\text{RPE} = \beta_0 + \beta_1 \cdot \text{Tier2} + \beta_2 \cdot \Delta t_{\text{frame}} + \beta_3 \cdot d_{\text{achieved}} + \epsilon$$
This explicitly isolates the predictive power of the precursor candidate ($\beta_1$) while holding camera timing ($\beta_2$) and displacement ($\beta_3$) constant.

### 5. Replication Strategy
- **Minimum 3 Independent Repeated Trials** per severity matrix cell, executed along the fixed flight corridor.
- Prioritization: High trial replication (3+ runs) per cell is prioritized over expanding the number of severity levels.

### 6. Explicit Methodological Exclusions for Phase 1
To maintain statistical rigor and avoid over-fitting small sample sizes:
- **No Machine Learning Model Fitting**: No SVM, Random Forest, or Neural Network classifiers will be trained in Phase 1.
- **No p-Hacked Multi-Comparison Significance Testing**: P-values across all pairwise combinations will not be reported; focus is placed on effect sizes ($\rho, R^2$) and cross-sweep consistency.
- **No Mixed-Effects Models**: Trial counts (3 per cell) are insufficient to fit random-intercept/random-slope mixed models robustly.

---

## 9. Statistical Traps & Mitigation Strategies

| Statistical Trap | Risk / Fallacy | Mandatory Mitigation Strategy |
| :--- | :--- | :--- |
| **Variable Redundancy** | $N_E / N_{\text{matched}}$, $N_{\text{pose}} / N_E$, and Homography inlier ratio are mathematically correlated by construction. | Declare **$N_{\text{pose}} / N_E$ (Pose/E ratio)** as the primary candidate variable; treat others strictly as corroborating diagnostics. |
| **Multiple Comparison P-Hacking** | Running 20+ correlation pairs risks false-positive discovery of spurious links. | Focus strictly on **effect size magnitudes ($\rho, R^2$)** and **consistency across independent sweeps** (Sweep A vs. Sweep B), rather than raw p-value thresholds. |
| **Post-Hoc Thresholding** | Setting failure thresholds after seeing sweep data introduces fitting bias. | All Tier 1 degradation boundaries (Level 1–5) are **fixed a priori** using data from the dedicated Healthy-Baseline condition. |
| **Ground Truth (GT) Leakage** | A bug in GT telemetry parsing could manufacture spurious correlation between motion IVs and RPE. | Keep the "achieved motion" GT processing pathway and the "VO vs GT RPE evaluation" pathway as **separate, independently unit-tested code paths**. |
| **Temporal Autocorrelation** | Consecutive frames within a flight run are highly correlated; treating frames as independent sample points inflates sample size artificially. | Treat **each complete flight trial run** as the unit of replication for cross-condition claims. Within-run frame sequences are used exclusively for lead/lag cross-correlation. |

---

## 10. Run Validation & Invalidation Criteria

A Phase 1 experimental trial is classified as **VALID** if and only if all of the following criteria are met:
1. PX4 SITL and Gazebo simulation complete the full maneuver without process crash, offboard disconnect, or failsafe trigger.
2. Mean camera publication rate remains $\ge 25.0\text{ Hz}$ throughout the maneuver (no severe rendering stalls $>1.0\text{s}$).
3. Ground truth pose logging maintains continuous streaming at $\ge 45.0\text{ Hz}$ with no timestamp gaps $>50.0\text{ ms}$.
4. Process lifecycle cleanup completes cleanly before and after the trial (no orphaned `gz-sim` processes).

Any run failing these criteria is classified as **INVALID**, logged in the trial registry, and re-executed.

---

## 11. Explicit Phase 2 Exclusion Rule

> **MANDATORY CONSTRAINT**:  
> No Phase 2 predictive failure module, adaptive motion intervention, homography fallback, gyro sensor fusion, or online parameter tuning shall be introduced or active during Phase 1 baseline characterization.  
>  
> Phase 1 evaluates the **un-modified baseline monocular VO pipeline** (`src/minimal_vo.py`) strictly as frozen and validated at the conclusion of Phase 0E.

---

## 12. Deliverable & Verification Summary

### Deliverables
1. `results/phase1_experiment_plan.md` (this revised document).
2. `methodology.md` (updated with standard metric hierarchy and confound management policy).

### Verification Plan
- **Syntax & Structural Verification**: Confirm markdown document integrity and consistency with Phase 0E findings (`failure_log.md` Entry 17).
- **Execution Stop Condition**: **STOP** immediately after writing this document. Do not generate flight scripts, analysis scripts, or begin Phase 1 data collection until explicit user review and approval are granted.
