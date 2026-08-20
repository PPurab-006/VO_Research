# Research Question & Plan

## Core Question
How does monocular visual odometry degrade under increasingly aggressive UAV motion, and can a lightweight motion-aware failure-detection mechanism predict impending tracking failure early enough to enable useful recovery?

## Hypothesis
As angular velocity and image-space feature velocity increase, feature-track survival decreases and VO drift increases; a motion-aware detector can identify impending tracking failure before complete loss and reduce recovery time.

## Falsifiability & Value Check
If the intervention does not improve VO tracking, the project still produces a quantified characterization of how motion severity and observable image-quality signals relate to VO failure, including which conditions predict failure and which do not. The experiment yields scientific value independent of whether the intervention succeeds.

## Core Design Principles
1. **Swappable VO Estimator**: The VO algorithm is modular infrastructure — the research question survives algorithm changes.
2. **Quantitative Severity**: Motion severity is quantified numerically (velocity $v$, angular velocity $\omega$, acceleration $a$, image-space feature velocity $v_{\text{feat}}$) — never subjective labels.
3. **Singular Scope**: Pure perception failure investigation. No side projects or extraneous features.
4. **Pre-Registered Hypothesis**: Pre-register hypotheses prior to full experiments to prevent post-hoc tuning.
5. **Statistical Rigor**: Report mean $\pm$ std and confidence intervals across repeated trials — never single RMSE numbers.
6. **Explicit Scale Alignment**: Solve monocular scale alignment explicitly (e.g., via `evo`) before computing Absolute Trajectory Error (ATE).
7. **Boring PX4 Baseline**: PX4 SITL acts strictly as a deterministic trajectory generator.
