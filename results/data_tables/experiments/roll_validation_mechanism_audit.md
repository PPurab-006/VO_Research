# Scientific Audit Report: Forensic Evaluation of VO Essential Matrix Collapse Mechanisms (10° Roll Validation)

**Dataset Analyzed**:
- [`results/roll_validation_vo.csv`](file:///home/purab/Purab/Projects/ROS/results/roll_validation_vo.csv) (1,231 VO frame records with diagnostic telemetry)
- [`results/roll_validation_telemetry.csv`](file:///home/purab/Purab/Projects/ROS/results/roll_validation_telemetry.csv) (1,530 PX4 high-frequency vehicle telemetry records)

---

## 1. Timestamp Alignment Quality

* **VO Simulation Timestamp Range**: `8.977 s` to `49.567 s` (Gazebo `/clock`).
* **Vehicle Telemetry Timestamp Range**: `1788539359.670 s` to `1788539391.673 s` (Wall clock).
* **Maneuver Alignment Window**:
  - Telemetry `CLIMB` duration = `12.01 s`.
  - Telemetry `MANEUVER` duration = `19.99 s` (Wall timestamps `1788539371.684 s` to `1788539391.673 s`).
  - Corresponding VO simulation window = `[20.991 s, 40.980 s]` (605 frames at $30.30\text{ Hz}$).
* **Timestamp Mismatch Metrics**:
  - **Median Mismatch**: `5.40 ms`
  - **p95 Mismatch**: `9.94 ms`
  - **Max Mismatch**: `10.64 ms`
* **Assessment**: The median timestamp mismatch of `5.40 ms` (less than $\frac{1}{6}$ of a camera frame period of $33.0\text{ ms}$) demonstrates that nearest-neighbor timestamp alignment is extremely tight and fully accurate for per-frame baseline and attitude evaluation.

---

## 2. Recalculated Frame-to-Frame Ground-Truth Motion (Maneuver Phase)

Calculated across the 604 consecutive camera frame intervals ($30.30\text{ Hz}$ camera rate):

| Motion Metric | Median | p95 | Maximum |
| :--- | :---: | :---: | :---: |
| **3D Translation Baseline ($\Delta t_{\text{3D}}$)** | `0.01540 m` ($1.54\text{ cm}$) | `0.03912 m` ($3.91\text{ cm}$) | `0.06185 m` ($6.18\text{ cm}$) |
| **XY Translation Baseline ($\Delta t_{\text{XY}}$)** | `0.01510 m` ($1.51\text{ cm}$) | `0.03898 m` ($3.90\text{ cm}$) | `0.06184 m` ($6.18\text{ cm}$) |
| **Roll Change per Frame ($\Delta\text{roll}$)** | `0.7800°` | `1.7985°` | `2.3500°` |
| **Pitch Change per Frame ($\Delta\text{pitch}$)** | `0.0000°` | `0.0200°` | `0.0600°` |
| **Yaw Change per Frame ($\Delta\text{yaw}$)** | `0.0000°` | `0.0000°` | `0.0000°` |
| **Total Angular Change ($\Delta\theta_{\text{total}}$)** | `0.7800°` | `1.7986°` | `2.3501°` |

---

## 3. Comparative Group Analysis & Statistical Correlations

### Group Comparison

| Motion / Telemetry Metric | Group A (`num_inliers < 5`, $N=489$) | Group B (`num_inliers >= 5`, $N=115$) |
| :--- | :---: | :---: |
| **3D Translation Baseline ($\Delta t_{\text{3D}}$)** | Median = `0.01466 m`, p95 = `0.04091 m` | Median = `0.01663 m`, p95 = `0.03188 m` |
| **Total Angular Change ($\Delta\theta_{\text{total}}$)** | Median = `0.7500°`, p95 = `1.7800°` | Median = `0.9101°`, p95 = `1.8320°` |
| **Feature Velocity (`feature_vel_mean`)** | Median = `1.81 px/fr` | Median = `10.00 px/fr` |
| **LK Tracking Error (`mean_lk_err`)** | Median = `1.8620 px` | Median = `3.3365 px` |
| **Feature Survival Rate** | Median = `99.8%` | Median = `99.1%` |
| **Homography Inliers (`num_inliers_H`)** | Median = `1788.0` | Median = `1858.0` |

### Statistical Correlation Coefficients

| Metric Pair | Pearson $r$ | Spearman $\rho$ | Interpretation |
| :--- | :---: | :---: | :--- |
| **Angular Change vs Essential Inliers** | `+0.0246` | `+0.0285` | Near zero correlation |
| **3D Baseline vs Essential Inliers** | `+0.0566` | `+0.0171` | Near zero correlation |
| **Angular Change vs Homography Inliers** | `+0.0550` | `+0.0359` | Near zero correlation |
| **3D Baseline vs Homography Inliers** | `-0.0184` | `+0.0018` | Near zero correlation |
| **Absolute Roll Angle vs Essential Inliers** | `+0.0141` | `+0.0125` | Near zero correlation |
| **Frame Roll Change ($\Delta\text{roll}$) vs E Inliers** | `+0.0257` | `+0.0282` | Near zero correlation |

---

## 4. 2D Motion Space Characterization (Translation Baseline vs Angular Change)

To evaluate whether Essential Matrix collapse ($E < 5$) tracks rotation or translation, maneuver frames were classified into a 2D matrix based on median 3D translation ($0.01540\text{ m}$) and median angular change ($0.7800^\circ$):

| Motion Region | Frames ($N$) | Median $E$ Inliers | Mean $E$ Inliers | Median $H$ Inliers | Mean $H$ Inliers | Median Tracked | Median Survival (%) | Median LK Error (px) |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| **Low Trans + Low Rot** ($\Delta t < 1.54\text{cm}, \Delta\theta < 0.78^\circ$) | 165 | **0.0** | 4.1 | **1794.0** | 1485.9 | 1807.0 | 99.7% | 2.0849 px |
| **Low Trans + High Rot** ($\Delta t < 1.54\text{cm}, \Delta\theta \ge 0.78^\circ$) | 137 | **0.0** | 4.8 | **1816.0** | 1588.2 | 1848.0 | 99.7% | 2.1208 px |
| **High Trans + Low Rot** ($\Delta t \ge 1.54\text{cm}, \Delta\theta < 0.78^\circ$) | 137 | **0.0** | 8.2 | **1815.0** | 1509.1 | 1838.0 | 99.7% | 2.0539 px | 1821.0 | 99.6% | 2.0346 px |

> [!IMPORTANT]
> **Key Finding**: Median Essential Matrix inliers is **identically 0.0** across all 4 quadrants of motion space!
> Essential Matrix collapse occurs uniformly across low and high translation, and low and high rotation.

---

## 5. Distinction Between Cumulative Trajectory & Per-Frame Motion

* **Cumulative Trajectory Displacement**: Over the 20-second maneuver, the drone moved horizontally by **`0.6100 m`** total.
* **Per-Frame Translation Baseline**: Between adjacent $30.30\text{ Hz}$ camera frames, translation averages **`0.0154 m`** ($1.54\text{ cm}$).
* **Scene Scale Ratio**: At a hover height of $Z \approx 2.50\text{ m}$, per-frame parallax is $\frac{1.54\text{ cm}}{250\text{ cm}} \approx 0.62\%$.

---

## 6. Temporal & Roll Trajectory Analysis

### 1-Second Time-Binned Profile Across 20-Second Maneuver

| Time Window | Frames | Median $E$ Inliers | Median $H$ Inliers | Median Roll Angle | Median $\Delta\text{roll}$ | Median 3D Baseline | Median Feature Velocity | Median LK Error |
| :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| **00s - 01s** | 30 | **0.0** | **1591.0** | 5.36° | 0.6650° | 0.00940 m | 2.11 px/fr | 2.7061 px |
| **01s - 02s** | 30 | **0.0** | **1783.5** | 9.86° | 0.6550° | 0.01702 m | 0.53 px/fr | 1.7331 px |
| **02s - 03s** | 30 | **0.0** | **1734.5** | 9.34° | 0.7900° | 0.01391 m | 0.38 px/fr | 1.5793 px |
| **03s - 04s** | 31 | **0.0** | **1999.0** | 9.43° | 0.8300° | 0.01416 m | 0.35 px/fr | 1.1678 px |
| **04s - 05s** | 30 | **0.0** | **1848.5** | 9.39° | 0.6600° | 0.01460 m | 2.46 px/fr | 2.1394 px |
| **05s - 06s** | 30 | **0.0** | **1887.5** | 8.93° | 0.8050° | 0.01615 m | 5.85 px/fr | 2.2052 px |
| **06s - 07s** | 31 | **0.0** | **1880.0** | 9.34° | 0.8200° | 0.01566 m | 4.00 px/fr | 2.2067 px |
| **07s - 08s** | 30 | **0.0** | **1841.0** | 8.96° | 0.7100° | 0.01479 m | 8.71 px/fr | 2.1019 px |
| **08s - 09s** | 30 | **0.0** | **1788.5** | 9.43° | 0.6600° | 0.01398 m | 3.89 px/fr | 2.2098 px |
| **09s - 10s** | 31 | **0.0** | **1871.0** | 8.72° | 0.9200° | 0.01844 m | 5.14 px/fr | 2.1280 px |
| **10s - 11s** | 30 | **0.0** | **1847.0** | 9.45° | 0.8550° | 0.01503 m | 3.70 px/fr | 2.2139 px |
| **11s - 12s** | 30 | **0.0** | **1710.5** | 8.96° | 0.8600° | 0.01315 m | 6.56 px/fr | 2.1574 px |
| **12s - 13s** | 31 | **0.0** | **1843.0** | 9.66° | 0.8000° | 0.01589 m | 5.48 px/fr | 2.2876 px |
| **13s - 14s** | 30 | **0.0** | **1798.5** | 9.18° | 0.7650° | 0.01480 m | 7.68 px/fr | 2.0218 px |
| **14s - 15s** | 30 | **0.0** | **1653.0** | 9.27° | 0.8850° | 0.01861 m | 7.26 px/fr | 1.9493 px |
| **15s - 16s** | 30 | **0.0** | **1637.0** | 9.07° | 0.7200° | 0.01611 m | 6.36 px/fr | 2.1319 px |
| **16s - 17s** | 31 | **0.0** | **1794.0** | 9.64° | 0.7300° | 0.01974 m | 5.14 px/fr | 2.1702 px |
| **17s - 18s** | 30 | **0.0** | **1845.0** | 9.46° | 0.7800° | 0.01552 m | 3.78 px/fr | 2.2358 px |
| **18s - 19s** | 30 | **0.0** | **1871.0** | 9.16° | 0.7750° | 0.01009 m | 4.62 px/fr | 2.2039 px |
| **19s - 20s** | 30 | **2.0** | **1841.0** | 9.36° | 0.8150° | 0.01717 m | 3.04 px/fr | 2.2654 px |

---

## 7. Analysis of Planar Scene Geometry vs Rotational Motion

In `agriculture.world`, the camera points down at flat ground terrain at $Z \approx 2.41\text{ m}$.
* **Planar Ground Model**: All visible 3D features lie on a single 3D plane $n^T X = d$. Under planar geometry, the image correspondence transformation is **always an exact Planar Homography $x_2 \sim H x_1$ regardless of whether the camera moves by translation or rotation**.
* **Observational Evidence**: `num_inliers_H` remains high (>1600 to 1999) across **all** frames, regardless of translation magnitude ($0.0\text{ cm}$ vs $6.1\text{ cm}$) or rotation rate ($0.0^\circ$ vs $2.3^\circ$).
* **Conclusion**: High Homography inliers ($H > 1600$) is driven by **planar ground scene geometry**. It does **NOT** uniquely prove pure rotational motion.

---

## 8. Categorized Scientific Verdict

### Evaluation of Core Statements

#### A. *"KLT tracking is healthy."*
* **Status**: **SUPPORTED**
* **Evidence**: Across all 605 maneuver frames, median tracked features = `1832.0`, median survival rate = `99.69%`, and median LK tracking error = `2.08 px`. Feature tracking performs exceptionally well.

#### B. *"Essential Matrix estimation is failing while KLT remains healthy."*
* **Status**: **SUPPORTED**
* **Evidence**: 80.99% of maneuver frames have `num_inliers < 5` (Essential Matrix collapse) despite >1800 tracked features with <2.1 px tracking error.

#### C. *"The failure is caused by insufficient translational baseline."*
* **Status**: **NOT ESTABLISHED**
* **Evidence**: Correlation between 3D baseline and $E$ inliers is near zero ($r = +0.0566$). $E$ collapse occurs equally in frames with higher baseline ($3.9\text{ cm}$) as in frames with low baseline ($0.4\text{ cm}$).

#### D. *"The failure is caused by predominantly rotational motion."*
* **Status**: **CONSISTENT WITH, BUT NOT ESTABLISHED**
* **Evidence**: Roll oscillation ($10^\circ$ at $0.5\text{ Hz}$) is active throughout the flight, but $E$ collapse occurs uniformly across all roll phases with near-zero correlation ($r = +0.0246$) to per-frame angular change.

#### E. *"The failure is caused by planar scene geometry."*
* **Status**: **CONSISTENT WITH, BUT NOT EXCLUSIVELY PROVEN**
* **Evidence**: High Homography fit ($H > 1700$) across >83% of frames regardless of baseline or rotation indicates strong planar-scene homography dominance. In a strictly planar scene (flat ground at constant height), the Essential Matrix is known to suffer from planar degeneracy.

#### F. *"Homography decomposition would be a valid VO fallback."*
* **Status**: **NOT ESTABLISHED**
* **Evidence**: High Homography inliers demonstrate that $H$ fits the 2D correspondence displacement, but Homography decomposition $H \to R, t/d, n$ yields 4 discrete pose/normal solutions requiring non-planar depth variation or baseline validation to resolve ambiguities safely.

---

## 9. Final Recommendations & Minimum Discriminative Experiment

* **Verdict**: **Option B: Existing data is sufficient to motivate a specific next experiment.**
* **Discriminative Experiment Required**:
  To conclusively isolate **Planar Scene Degeneracy** (flat ground) from **Rotational / Baseline Degeneracy**, execute a single controlled 10° roll validation in a scene with **strong 3D non-planar depth variation** (e.g. `3dmap.world` or a scene with foreground/background 3D objects):
  - If $E$ inliers recover to >100 in `3dmap.world` under identical 10° roll trajectory, the failure in `agriculture.world` is **100% Planar Scene Degeneracy**.
  - If $E$ inliers remain <5 in `3dmap.world`, the failure is **Rotational / Baseline Degeneracy**.
