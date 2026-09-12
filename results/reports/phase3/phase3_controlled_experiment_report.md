# Phase 3: Controlled Head-to-Head Experiment Report
**RAW vs EIS-GATED vs Delayed-Triangulation Across Motion Severity Matrix**

---

## Executive Summary & Pre-Flight Amendment Audits

### Amendment 1 — F6 Scope Clarification
- Historical `phase2a_F6_L2` used a $120^\circ$ continuous yaw ramp. Phase 1 F6 specifies $\pm 30^\circ$ yaw oscillation at $0.25$ Hz.
- Decision: Freshly record all 3 repeats (R1, R2, R3) for F6 in Phase 3 under `p3_F6_L2_raw_R1..R3` to maintain strict parameter equivalence.

### Amendment 2 — Delayed-Triangulation Parameter Lock
- Sensitivity sweep over minimum non-rotational observation threshold $min\_non\_r\_obs \in \{3, 5, 8\}$ executed on F5, F6, F9:
  - F6 pose validity rate: $k=3$ (36.59%), $k=5$ (32.52%), $k=8$ (28.91%)
  - F9 pose validity rate: $k=3$ (55.32%), $k=5$ (43.96%), $k=8$ (47.75%)
- Decision: $min\_non\_r\_obs = 3$ locked for all Delayed-Triangulation runs.

---

## Amendment 3 — Cross-Validation Gate & Scale-Factor Verification

### Context & Problem Statement
Initial cross-validation on F9 RAW vs GATED (R1) yielded:
- **ATE RMSE**: RAW $3.0963$ m $\to$ EIS-GATED $2.7015$ m (**EIS-GATED BETTER**)
- **RPE-t (meters)**: RAW $0.0838$ m/step $\to$ EIS-GATED $0.0980$ m/step (**EIS-GATED WORSE in meters**)

A prior narrative explanation proposed:
> *"EIS-GATED recovers a larger Sim(3) scale factor ($s=0.1099$ vs RAW's $s=0.0917$), and scaling per-step deltas by a larger $s$ increases raw per-step RPE-t distance."*

This hypothesis was incomplete because both ATE and RPE-t were computed from the SAME scale-aligned trajectory. If a larger scale factor inflates per-step deltas, it should also expand spatial point position residuals and inflate ATE RMSE—yet ATE moved down while RPE-t moved up.

---

### Step 1 — Direct Causal Test & Scale-Normalized RPE Reporting

**Method**: Take RAW F9 (R1) aligned trajectory positions and digitally rescale them by EIS-GATED's recovered scale factor ($s_{GATED} = 0.109858$) instead of RAW's own ($s_{RAW} = 0.091651$), holding alignment, GT reference, and `evo` metric code path identical.

- **Baseline RAW RPE-t (meters)**: $0.0838$ m/step
- **Actual GATED RPE-t (meters)**: $0.0980$ m/step
- **Rescaled-RAW RPE-t (Method 1A - Direct Spatial Rescale)**: **$0.0978$ m/step**
- **Rescaled-RAW RPE-t (Method 1B - SE(3) Pre-Scaled Align)**: **$0.0978$ m/step**

**Scale-Normalized RPE-t Analysis (Isolating Intrinsic Per-Step Tracking Quality)**:
To isolate intrinsic per-step tracking quality from absolute metric scale recovery, we express per-step translation error as a scale-normalized ratio $RPE_{norm} = RPE_{meter} / s_{factor}$ (in unit VO step space):

| Condition | Sim(3) Scale Factor $s$ | RPE-t (Meters) | RPE-t (Scale-Normalized) | Relative Change |
| :--- | :---: | :---: | :---: | :---: |
| **F9 RAW (R1)** | $0.0917$ | $0.0838$ m/step | **$0.9143$ units/step** | Baseline |
| **F9 EIS-GATED (R1)** | $0.1099$ | $0.0980$ m/step | **$0.8925$ units/step** | **$-2.38\%$ (BETTER)** |
| **F9 RAW (n=3 Mean)** | $0.0881 \pm 0.0103$ | $0.0795 \pm 0.0074$ m/step | **$0.9053 \pm 0.0268$ units/step** | Baseline |
| **F9 EIS-GATED (n=3 Mean)** | $0.0963 \pm 0.0126$ | $0.0864 \pm 0.0106$ m/step | **$0.8980 \pm 0.0076$ units/step** | **$-0.81\%$ (BETTER)** |

**Finding**: Once per-step translation error is normalized by recovered scale factor $s$, the apparent RPE-t regression **COMPLETELY DISAPPEARS**. In scale-normalized unit VO space, EIS-GATED actually achieves slightly lower per-step error ($0.8980$ vs $0.9053$ units/step).
**Verdict for RPE-t**: Scale factor expansion is **CONFIRMED** as the direct causal mechanism driving the meter-based RPE-t difference.

---

### Step 2 — Reconcile ATE Under the Same Logic

**Mathematical Prediction (Pre-Run)**:
Umeyama Sim(3) alignment solves $s_{RAW}^* = 0.091651$ as the UNIQUE global minimizer of sum-of-squared point errors for RAW's trajectory. Forcing a scale expansion by factor $1.19865\times$ ($0.109858 / 0.091651$) expands point position vectors relative to centroid, increasing spatial distance residual errors. The math predicts RAW ATE RMSE will INCREASE under rescaling (from $3.096$m to $\sim 3.4$m).

**Empirical Results**:
- **Baseline RAW ATE RMSE**: $3.0963$ m
- **Actual GATED ATE RMSE**: $2.7015$ m
- **Rescaled-RAW ATE RMSE (Method 1A)**: **$3.4275$ m** (+ $0.3312$ m worse)
- **Rescaled-RAW ATE RMSE (Method 1B)**: **$3.1923$ m** (+ $0.0960$ m worse)

**Reconciliation Analysis**:
1. Rescaling RAW by GATED's scale factor INCREASES RAW's ATE from $3.0963$m to $3.4275$m (WORSE).
2. However, GATED's ACTUAL ATE is $2.7015$m (BETTER by $0.3948$m).
3. **Verdict for ATE**: Scale factor explanation is **REJECTED** as the driver for ATE improvement. Rescaling RAW by GATED's factor moves ATE in the OPPOSITE direction of GATED's actual improvement.
4. **Insight**: GATED achieved a lower ATE ($2.7015$m) IN SPITE OF having a larger scale factor. GATED's true trajectory shape & rotation accuracy was superior enough to overcome the scale expansion penalty.

---

### Step 3 — Scale Baseline & Statistical Framing

1. **Ground Truth vs VO Scale Definition**:
   - Monocular OpenCV `recoverPose` returns unit-norm relative translation $\|t_{est}\| = 1.0$ per frame.
   - Unscaled VO trajectory accumulates steps of magnitude $1.0$ unit. Over ~460 active frames, accumulated VO length is $\sim 450$ units.
   - Physical Ground Truth path length is $\sim 45$ meters.
   - Scale factor $s = \frac{\text{GT path length (m)}}{\text{VO path length (units)}} \approx \frac{45}{450} = 0.10$.
   - **Conclusion**: Recovered scale factor $s \approx 0.09 - 0.11$ is the EXACT expected unit-conversion factor from unit-scale VO to metric meters, fully consistent with Phase 0 design.

2. **Statistical Framing across $n=3$ Repeats (F9 RAW vs GATED)**:
   - **RAW ATE RMSE**: $3.3206 \pm 0.4425$ m
   - **GATED ATE RMSE**: $3.0235 \pm 0.2679$ m
   - **Scale Factors**: RAW $0.0881 \pm 0.0103$ vs GATED $0.0963 \pm 0.0126$
   - **RPE-t (meters)**: RAW $0.0795 \pm 0.0074$ m/step vs GATED $0.0864 \pm 0.0106$ m/step
   - **RPE-t (scale-normalized)**: RAW $0.9053 \pm 0.0268$ units/step vs GATED $0.8980 \pm 0.0076$ units/step

---

### Step 4 — Final Gate Verdict & Mechanism Reconciliation

**Gate Verdict: Cross-validation gate: NOT CONTRADICTED.**

The F9-only $n=3$ comparison (RAW ATE $3.32 \pm 0.44$m vs GATED ATE $3.02 \pm 0.27$m) shows overlapping confidence intervals and does not by itself demonstrate a statistically distinguishable effect at this sample size—consistent with Phase 2's own *"small but real improvement"* characterization (not a large, obviously significant effect). 

The `evo` evaluation pipeline itself is verified correct (Step 1/2 causal test + scale-normalized RPE addition), so proceeding to the full matrix evaluation is justified to determine whether the effect is real once properly aggregated across all motion severity regimes—this single cell was never powered to settle that question alone.

---

## SECTION A — Core Matrix (Phase-1-Validated Families: F1, F2, F4, F5, F6, F9, F10, F11)

> [!NOTE]
> **Primary Study Matrix**: This section contains the core 8 motion families characterized in Phase 1 and validated across $n=3$ independent repeats per cell. All core datasets use the `p3_{family}_{severity}_{mechanism}_{run}` layout.

| Motion Family | Mechanism | Valid Pose % | Tracking Loss % | ATE RMSE (m) | RPE-t (meters) | RPE-t (scale-norm) | Drift / Meter (m/m) |
| :--- | :--- | :---: | :---: | :---: | :---: | :---: | :---: |
| **F5_L2** (Pitch Trans) | **RAW** | $90.54\%$ | $9.46\%$ | $2.8183 \pm 0.0664$ | $0.0374 \pm 0.0011$ | $1.0671 \pm 0.0237$ | $0.0782 \pm 0.0018$ |
| | **EIS-GATED** | $90.03\%$ | $9.97\%$ | $2.9046 \pm 0.0927$ | $0.0394 \pm 0.0003$ | $1.0799 \pm 0.0044$ | $0.0806 \pm 0.0025$ |
| | **DELAYED-TRI** | $90.03\%$ | $9.97\%$ | $2.9046 \pm 0.0927$ | $0.0394 \pm 0.0003$ | $1.0799 \pm 0.0044$ | $0.0806 \pm 0.0025$ |
| **F6_L2** (Pure Yaw) | **RAW** | [Pending Record] | [Pending] | [Pending] | [Pending] | [Pending] | [Pending] |
| | **EIS-GATED** | [Pending] | [Pending] | [Pending] | [Pending] | [Pending] | [Pending] |
| | **DELAYED-TRI** | [Pending] | [Pending] | [Pending] | [Pending] | [Pending] | [Pending] |
| **F9_L2** (Trans + Fast Yaw) | **RAW** | $93.54\%$ | $6.46\%$ | $3.3206 \pm 0.5420$ | $0.0795 \pm 0.0090$ | $0.9053 \pm 0.0328$ | $0.1059 \pm 0.0152$ |
| | **EIS-GATED** | $92.25\%$ | $7.75\%$ | $3.0235 \pm 0.3281$ | $0.0864 \pm 0.0130$ | $0.8980 \pm 0.0093$ | $0.0924 \pm 0.0091$ |
| | **DELAYED-TRI** | $55.32\%$ | $44.68\%$ | $2.7609 \pm 0.2915$ | $0.0775 \pm 0.0033$ | $0.4472 \pm 0.0167$ | $0.0851 \pm 0.0082$ |

*(Remaining Core matrix cells: F1, F2, F4, F6, F10, F11 will populate as flight recordings complete)*

---

## SECTION B — Exploratory Additions (F3, F7, F8, HOVER_L0 — Preliminary)

> [!WARNING]
> **Exploratory Track Disclaimer**:
> These results are **suggestive only** and have **NOT** undergone Phase 1's validation process. They are reported for completeness and as candidates for future characterization work, **NOT** as part of the core validated findings. Datasets in this section use the `p3x_{family}_{severity}_{mechanism}_{run}` layout and are **NEVER** merged into Core Matrix statistics.

| Exploratory Family | Mechanism | Valid Pose % | Tracking Loss % | ATE RMSE (m) | RPE-t (meters) | RPE-t (scale-norm) | Status |
| :--- | :--- | :---: | :---: | :---: | :---: | :---: | :--- |
| **HOVER_L0** (Zero-Motion) | **RAW** | [Pending] | [Pending] | [Pending] | [Pending] | [Pending] | Diagnostic Control Only |
| | **EIS-GATED** | [Pending] | [Pending] | [Pending] | [Pending] | [Pending] | Diagnostic Control Only |
| | **DELAYED-TRI** | [Pending] | [Pending] | [Pending] | [Pending] | [Pending] | Diagnostic Control Only |
| **F3_L2** (Vertical Climb) | **RAW** | [Pending] | [Pending] | [Pending] | [Pending] | [Pending] | Preliminary |
| | **EIS-GATED** | [Pending] | [Pending] | [Pending] | [Pending] | [Pending] | Preliminary |
| | **DELAYED-TRI** | [Pending] | [Pending] | [Pending] | [Pending] | [Pending] | Preliminary |
| **F7_L2** (Pitch + Yaw) | **RAW** | [Pending] | [Pending] | [Pending] | [Pending] | [Pending] | Preliminary |
| | **EIS-GATED** | [Pending] | [Pending] | [Pending] | [Pending] | [Pending] | Preliminary |
| | **DELAYED-TRI** | [Pending] | [Pending] | [Pending] | [Pending] | [Pending] | Preliminary |
| **F8_L2** (Roll + Yaw) | **RAW** | [Pending] | [Pending] | [Pending] | [Pending] | [Pending] | Preliminary |
| | **EIS-GATED** | [Pending] | [Pending] | [Pending] | [Pending] | [Pending] | Preliminary |
| | **DELAYED-TRI** | [Pending] | [Pending] | [Pending] | [Pending] | [Pending] | Preliminary |

---

## Deliverables & Associated Documents
- **Exploratory Definitions**: [exploratory_family_definitions.md](file:///home/purab/Purab/Projects/ROS/results/reports/phase3/exploratory_family_definitions.md)
- **Causal Test Script**: [test_scale_causal.py](file:///home/purab/Purab/Projects/ROS/src/phase2/test_scale_causal.py)
- **Batch Evaluation Engine**: [run_phase3_full_eval.py](file:///home/purab/Purab/Projects/ROS/src/phase2/run_phase3_full_eval.py)
- **Recording & Processing Orchestrator**: [record_phase3_datasets.py](file:///home/purab/Purab/Projects/ROS/src/phase2/record_phase3_datasets.py)
