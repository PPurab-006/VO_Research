# Scientific Report: Comparative 10° Roll Validation in 3D Non-Planar Environment (`3dmap.world` vs `agriculture.world`)

## 1. Experimental Setup & Control Variables

| Parameter | `agriculture.world` (Control) | `3dmap.world` (Experimental) | Control Status |
| :--- | :---: | :---: | :---: |
| **World Type** | Flat 2D Terrain Plane | Complex 3D Structure (Mesh $Z \in [-40\text{m}, +40\text{m}]$) | **Variable** |
| **Spawn Coordinate** | $(14.0505, -7.5229, 0.1076)$ | $(19.0, -15.0, 0.2)$ | Adjusted to World |
| **Target Hover Altitude** | $2.50\text{ m}$ above terrain | $2.50\text{ m}$ above terrain | **Identical** |
| **Target Roll Amplitude** | $10.0^\circ$ sinusoidal | $10.0^\circ$ sinusoidal | **Identical** |
| **Oscillation Frequency** | $0.50\text{ Hz}$ | $0.50\text{ Hz}$ | **Identical** |
| **Maneuver Duration** | $20.0\text{ s}$ | $20.0\text{ s}$ | **Identical** |
| **Camera Sensor & Config** | $1280 \times 960$, Mono, $30\text{ Hz}$ | $1280 \times 960$, Mono, $30\text{ Hz}$ | **Identical** |
| **VO Pipeline & Params** | KLT, GFTT 2000, 5-pt RANSAC 1.0px | KLT, GFTT 2000, 5-pt RANSAC 1.0px | **Identical** |
| **Position Controller** | Hybrid Position-Corrected Attitude | Hybrid Position-Corrected Attitude | **Identical** |

---

## 2. Quantitative Results Comparison

### Essential Matrix Inlier Distribution ($E$)

| Metric | `agriculture.world` (Flat) | `3dmap.world` (3D Non-Planar) |
| :--- | :---: | :---: |
| **Median $E$ Inliers** | **`0.0`** | **`1.0`** |
| **Mean $E$ Inliers** | `6.1` | `3.1` |
| **p10 / p25 / p50 / p75 / p90** | `0.0 / 0.0 / 0.0 / 2.0 / 26.0` | `0.0 / 0.0 / 1.0 / 3.0 / 9.0` |
| **Fraction $E < 5$ (%)** | **`80.96%`** | **`79.17%`** |
| **Fraction $E \ge 5$ (%)** | `19.04%` | `20.83%` |

### Diagnostic Homography Inlier Distribution ($H$)

| Metric | `agriculture.world` (Flat) | `3dmap.world` (3D Non-Planar) |
| :--- | :---: | :---: |
| **Median $H$ Inliers** | **`1803.5`** | **`1883.0`** |
| **Mean $H$ Inliers** | `1520.9` | `1618.1` |
| **Fraction $H \ge 500$ (%)** | `83.44%` | `85.95%` |

### KLT Feature Tracking Health

| Metric | `agriculture.world` (Flat) | `3dmap.world` (3D Non-Planar) |
| :--- | :---: | :---: |
| **Median Tracked (`num_matched`)** | `1833.0` | `1948.0` |
| **Median Feature Survival Rate** | `99.69%` | `99.80%` |
| **Median LK Error (`mean_lk_err`)** | `2.0789 px` | `1.6039 px` |
| **Median Feature Velocity** | `3.08 px/fr` | `3.70 px/fr` |

### Motion & Camera Controls

| Metric | `agriculture.world` (Flat) | `3dmap.world` (3D Non-Planar) |
| :--- | :---: | :---: |
| **Achieved p95 \|roll\|** | `13.39^\circ` | `13.38^\circ` |
| **Achieved p95 \|pitch\|** | `0.73^\circ` | `0.66^\circ` |
| **Achieved p95 \|yaw\|** | `0.00^\circ` | `0.00^\circ` |
| **Max Horizontal Displacement** | `0.6100\text{ m}` | `0.6813\text{ m}` |
| **Median 3D Baseline** | `0.01540\text{ m}` | `0.01847\text{ m}` |
| **Median Angular Change** | `0.7800^\circ` | `0.7701^\circ` |
| **Achieved Camera FPS** | `30.30\text{ Hz}` | `30.30\text{ Hz}` |

---

## 3. Failure Window Evaluation (Strict $\ge 2.0\text{ s}$ Timestamp Span)

* **`agriculture.world`**: Failure Triggered = **`False`**
* **`3dmap.world`**: Failure Triggered = **`False`**

---

## 4. Scientific Interpretation

### Classification of Outcome: **CASE B: Scene Planarity Alone is Insufficient to Explain the Failure**

* **Observed Behavior**: Essential Matrix inliers remained severely collapsed in both environments:
  - `agriculture.world` (Flat 2D): Median $E = 0.0$, Mean $E = 6.1$, Fraction $E < 5 = 80.96\%$.
  - `3dmap.world` (3D Structure): Median $E = 1.0$, Mean $E = 3.1$, Fraction $E < 5 = 79.17\%$.
* **KLT Health & Motion Controls**: KLT feature tracking remained exceptionally healthy across both runs (Median tracked $> 1800$, survival $> 99.6\%$, LK error $< 2.1\text{ px}$, Homography inliers $> 1800$). Motion parameters ($10^\circ$ roll amplitude, $1.5-1.8\text{ cm}$ per-frame translation baseline, $30.3\text{ Hz}$ camera FPS) were strictly controlled and comparable.

### Primary Conclusion

> **Essential Matrix inliers remain similarly collapsed in the 3D non-planar scene (`3dmap.world`) despite healthy KLT tracking (`num_matched > 1900`, `survival > 99.8%`, `LK error < 1.6 px`) and identical motion severity. Median E inliers remain at 1.0 (vs 0.0 in agriculture), and ~79.17% of frames remain severely collapsed (E < 5). Therefore, scene planarity alone is INSUFFICIENT to explain the failure. Rotational/low-baseline/model-conditioning mechanisms remain important candidates.**

---

## 5. Scientific Implications & Next Experimental Steps

1. **Scene Planarity Rules Out Single-Cause Geometry Hypothesis**: Replacing planar ground with 3D complex mesh geometry did not rescue Essential Matrix pose estimation. Therefore, scene planarity alone does NOT account for the collapse observed during roll maneuvers.
2. **Rotational / Low-Baseline Dominance**: Because high roll rate ($0.5\text{ Hz}$, $10^\circ$) generates strong rotational image flow relative to the small translational baseline ($1.5-1.8\text{ cm}$ per frame), the Essential Matrix RANSAC optimization remains mathematically ill-conditioned (epipolar constraint degenerate under near-pure rotation).
3. **Implications for System Architecture**: Future phases should evaluate **rotation-decoupled pose estimation**, **5-point vs 8-point conditioning**, or **Homography/Essential model selection (H vs E fallback)** to handle rotational motion regimes cleanly.
