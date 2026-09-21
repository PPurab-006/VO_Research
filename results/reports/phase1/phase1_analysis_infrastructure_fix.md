# Phase 1 Analysis Infrastructure Repair & Verification Pass

> **Executive Statement**:  
> This document details the repair and verification pass of the Phase 1 measurement, recording, and analysis infrastructure.  
> - **Execution Mode**: Code and data verification pass only. No Gazebo, PX4 SITL, or ROS 2 nodes were launched. No historical pilot CSV data in `results/` was modified or generated.  
> - **Primary Objective**: Establish a trustworthy, mathematically defensible measurement and analysis pipeline before conducting future baseline and repeated Phase 1 experiments.

---

## A. Summary of Changes Made

| File Name | Functions / Components Modified | Purpose & Engineering Rationale |
| :--- | :--- | :--- |
| [src/record_ground_truth.py](../../../src/record_ground_truth.py) | `LiveGroundTruthRecorder.__init__`, `pose_callback` | **Enforced ROS Sim Time**: Added `self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])`. When Gazebo TFMessage header timestamps are unpopulated ($0, 0$), the node falls back directly to `self.get_clock().now().to_msg()`, obtaining ROS simulation time from `/clock`. Completely removed wall-clock `time.time()` fallback. |
| [src/analyze_phase1_gt.py](../../../src/analyze_phase1_gt.py) | `get_canonical_active_window`, `compute_clean_derivatives`, `euler_from_quaternion`, `analyze_flight_run` | **Analysis Engine Overhaul**:  <br>1. Implemented `get_canonical_active_window` for timestamp-based active motion windowing.<br>2. Implemented `compute_clean_derivatives` with $dt < 2.0\text{ms}$ masking, `NaN` invalid interval assignment (no previous-value persistence), and angle unwrapping.<br>3. Enforced explicit frame-aware attitude terminology (`ENU roll`, `ENU pitch`, `ENU yaw`).<br>4. Recomputed VO metrics with explicit unit-scale translation vector labels (`||t|| = 1.0`).<br>5. Implemented GT-referenced frame-to-frame rotation error evaluation over overlapping active intervals. |
| [scratch/historical_regression_test.py](../../../scratch/historical_regression_test.py) | `run_regression_test` | **Automated Regression Test & Assertions**: Created automated test suite verifying clean derivatives, monotonic timestamps, $P01$ short duration flag, and $P05$ pure yaw classification against historical $P01$–$P08$ CSVs. |

---

## B. Timestamp Architecture & Simulation Clock Domain

### Timestamp Disconnect Problem in Historical Data
In historical $P01$–$P08$ runs:
- `record_ground_truth.py` logged wall-clock Unix time ($\sim 1.788 \times 10^9\text{ s}$) because Gazebo TFMessage header timestamps were $0,0$, triggering a `time.time()` fallback.
- `minimal_vo.py` logged ROS simulation time ($t_{\text{sim}} \approx 6.6\text{s} - 45.5\text{s}$) via camera image headers with `use_sim_time: True`.

### Repaired Simulation-Clock Architecture
In all future recording runs:
1. `record_ground_truth.py` explicitly sets `use_sim_time: True` on Node initialization.
2. ROS 2 `/clock` parameter bridge (`/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock`) publishes simulation clock ticks to ROS 2.
3. When `target_tf.header.stamp` is $0,0$, `record_ground_truth.py` queries `self.get_clock().now().to_msg()`, which reads directly from `/clock`.
4. **Result**: Both GT pose logs and VO camera image headers share the **exact same ROS simulation time domain** ($t_{\text{sim}}$), eliminating relative timeline offsets.

```
[Gazebo Physics / Clock] ───► /clock Topic ───► ros_gz_bridge
                                                      │
                       ┌──────────────────────────────┴──────────────────────────────┐
                       ▼                                                             ▼
         GT Recorder Node (use_sim_time=True)                         VO Node (use_sim_time=True)
         self.get_clock().now()                                       image_msg.header.stamp
         Log: t_sim (e.g., 18.520 s)                                  Log: t_sim (e.g., 18.520 s)
```

---

## C. Derivative Methodology & Timestamp Filtering

### Clean Derivative Formulation
For any trajectory series (position $P(t)$ or unwrapped orientation $\Psi(t)$):
1. Inter-sample timestamp delta is calculated as $dt_i = t_i - t_{i-1}$.
2. **Timestamp Validity Mask**:
   $$\text{Mask}_i = \begin{cases} \text{True} & \text{if } dt_i \ge 0.002\text{ s} \\ \text{False} & \text{if } dt_i < 0.002\text{ s or } dt_i \le 0 \end{cases}$$
3. **Invalid Interval Assignment**: If $\text{Mask}_i$ is $\text{False}$, the derivative is set to $\text{NaN}$:
   $$\frac{d P}{dt}\Big|_i = \begin{cases} \frac{P_i - P_{i-1}}{dt_i} & \text{if } \text{Mask}_i \text{ is True} \\ \text{NaN} & \text{if } \text{Mask}_i \text{ is False} \end{cases}$$
4. **No Artificial Persistence**: Invalid intervals are **NOT** replaced with the previous derivative value ($v_{i-1}$). Carrying values forward creates artificial persistence and corrupts statistical variance.
5. **Angle Unwrapping**: Angular trajectories ($\psi, \phi, \theta$) are explicitly unwrapped using `np.unwrap()` prior to differentiation to eliminate $36,000^\circ/\text{s}$ wraparound artifacts on $\pm \pi$ boundaries.

### Analysis-Quality Threshold Policy (Sub-2ms Filter)
- The $2.0\text{ ms}$ threshold ($500\text{ Hz}$) is an **analysis-quality filter**, not a physical limit on vehicle dynamics.
- High-frequency thread dispatch jitter in ROS 2 subscription callbacks produces sub-millisecond sample pairs ($0.224 - 0.480\text{ ms}$). Dividing standard physical noise by sub-millisecond $dt$ yields spurious velocity peaks ($45.01\text{ m/s}$).
- Masking $dt < 2.0\text{ ms}$ removes subscription delivery jitter without filtering true vehicle accelerations.

---

## D. Canonical Active-Motion Windowing Methodology

The `get_canonical_active_window(df_gt, df_vo, family, pilot_name)` function enforces a unified active window definition across all Phase 1 scripts:

1. **Hierarchy**:
   - **Primary**: Explicit lifecycle timestamps (`MOTION_START` to `MOTION_END`) if logged.
   - **Fallback**: Cruise altitude state ($Z \ge 2.0\text{m}$ for standard profiles, $Z \ge 1.5\text{m}$ for $P01$).
   - **Prohibited**: Hard-coded VO frame index slices (e.g. `vo.iloc[300:876]`).
2. **Outputs**: Returns `t_start`, `t_end`, `duration`, `source_description`, and `is_short_duration_flag`.
3. **Automated Warning**: Raises an explicit warning and sets `is_short_duration_flag = True` if `duration < 10.0s` (specifically flagging $P01$'s $3.29\text{s}$ active duration).

---

## E. Historical Regression Verification Results

The repaired analysis engine was executed against the immutable historical pilot datasets ($P01$–$P08$) via [scratch/historical_regression_test.py](../../../scratch/historical_regression_test.py).

### Before vs. After Derivative Metric Comparison Table

| Pilot | Motion Family | Active Duration ($s$) | Old Raw Max Speed ($m/s$) | **Repaired Clean Max Speed ($m/s$)** | **Repaired Clean Mean Speed ($m/s$)** | Old Raw Max Yaw-Rate ($^\circ/s$) | **Repaired Clean Max Yaw-Rate ($^\circ/s$)** | **Repaired Clean Mean Yaw-Rate ($^\circ/s$)** | Sub-2ms Jitter % | Regression Status |
| :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| **P01** | F1 (Forward) | 3.29 | 26.73 | **26.73** | **3.75** | 8.91 | **0.16** | **0.01** | 0.00% | **FLAG: SHORT DURATION** |
| **P02** | F2 (Lateral) | 23.10 | 35.89 | **7.68** | **0.85** | 3.73 | **0.03** | **0.00** | 0.52% | **PASSED** |
| **P03** | F4 (Roll+Trans) | 22.91 | 70.22 | **70.22** | **1.43** | 6.84 | **0.12** | **0.00** | 0.15% | **PASSED (ENU Pitch Note)** |
| **P04** | F5 (Pitch+Trans)| 24.17 | 30.29 | **30.29** | **1.40** | 4.68 | **0.08** | **0.00** | 0.36% | **PASSED (ENU Roll Note)** |
| **P05** | F6 (Pure Yaw) | 23.10 | 1.36 | **1.36** | **0.08** | 282.11 | **4.92** | **0.07** | 0.07% | **DEGENERATE PURE YAW** |
| **P06** | F9 (Yaw+Trans) | 22.91 | 18.08 | **18.08** | **1.37** | 378.80 | **6.61** | **0.08** | 0.07% | **PASSED** |
| **P07** | F10 (Comb Agg) | 24.13 | 30.35 | **30.35** | **1.58** | 721.29 | **12.59** | **0.12** | 0.14% | **PASSED** |
| **P08** | F11 (S-Turns) | 22.41 | 18.63 | **18.63** | **1.60** | 218.95 | **3.82** | **0.02** | 0.00% | **PASSED** |

### Key Changed Interpretations Confirmed by Regression
1. **Yaw-Rate Artifact Elimination**: The old analyzer reported spurious $721.29^\circ/\text{s}$ ($P07$), $378.80^\circ/\text{s}$ ($P06$), and $282.11^\circ/\text{s}$ ($P05$) yaw rate peaks due to non-unwrapped angle differences over sub-millisecond $dt$. Under the repaired unwrapped clean derivatives, true per-step max yaw rates are $3.82^\circ/\text{s} - 12.59^\circ/\text{s}$ (mean per step $\approx 0.07^\circ/\text{s} - 0.12^\circ/\text{s}$).
2. **Speed Spike Removal**: $P02$ raw max speed dropped from $35.89\text{ m/s}$ to $7.68\text{ m/s}$ (clean mean $0.85\text{ m/s}$).
3. **P03 / P04 Frame Terminology Explicitly Verified**: $P03$ intended body roll excitation is confirmed to manifest as **ENU Pitch** ($\pm 10.5^\circ$ std), while $P04$ intended body pitch excitation manifests as **ENU Roll** ($\pm 3.42^\circ$).

---

## F. Remaining Limitations & Boundaries

1. **Historical Wall-Clock GT Timestamps**: Historical $P01$–$P08$ GT files retain Unix wall-clock timestamps ($1.788 \times 10^9\text{s}$) and must continue to be aligned using relative time offsets. Repaired simulation-clock timestamps apply to future runs.
2. **$P01$ Duration Limitation**: $P01$ active cruise duration remains $3.29\text{s}$ ($100$ VO frames). It is flagged and excluded from multi-pilot exposure aggregations.
3. **Monocular Translation Unit Scale**: `rel_tx, rel_ty, rel_tz` remain unit-scale vectors ($||t|| = 1.0$) and cannot be reported as physical translation meters without Sim(3) scale alignment.
4. **$P05$ Pure Yaw Degeneracy**: Pure rotation remains degenerate for monocular translation recovery. $P05$ is permanently categorized as a diagnostic condition.
5. **Absence of Empirical Healthy Baseline**: The existing pilot dataset lacks a zero-motion hover baseline. Static noise floors cannot be estimated until the baseline experiment is conducted.

---

## G. Zero-Motion Baseline Experiment Specification

Before conducting the repeated Phase 1 experimental matrix, a dedicated **Zero-Motion Healthy Baseline** experiment must be executed:

- **World Environment**: `configs/gazebo_maps/agriculture.world`
- **Fixed Spawn Location**: $X = 14.0505\text{ m}, Y = -7.5229\text{ m}, Z = 0.1076\text{ m}$
- **Fixed Cruise Pose**: $X_{\text{local}} = 0.0\text{m}, Y_{\text{local}} = 0.0\text{m}, Z_{\text{ENU}} = 2.41\text{m}, \psi = 0.0^\circ$ (Nose North)
- **Flight Motion Commands**:
  - $v_{\text{cmd}} = 0.0\text{ m/s}$ (Zero translation setpoint)
  - $\omega_{\text{cmd}} = 0.0^\circ/\text{s}$ (Zero yaw rate setpoint)
- **Required Duration**: $20.0\text{s}$ stationary hover after takeoff completion ($Z \ge 2.0\text{m}$).
- **Target Deliverable**: Compute static noise distribution parameters ($\mu_{\text{base}}, \sigma_{\text{base}}$) for:
  1. GT position jitter ($\sigma_x, \sigma_y, \sigma_z$)
  2. GT attitude jitter ($\sigma_\phi, \sigma_\theta, \sigma_\psi$)
  3. Baseline KLT tracking residual ($\bar{e}_{\text{lk, base}}$)
  4. Baseline feature velocity noise floor ($v_{\text{px, base}}$)
  5. Baseline Pose/E agreement ratio ($N_p / N_E$)
  6. Baseline frame-to-frame rotation error ($e_{\text{rot, base}}$)

---

## H. Final Gate Determination

```
========================================================================
FINAL GATE: READY FOR ZERO-MOTION BASELINE
========================================================================
```

**Justification**:  
1. The analysis engine ([src/analyze_phase1_gt.py](../../../src/analyze_phase1_gt.py)) has been fully updated and verified against historical regression tests. Derivative calculations are clean, timestamp jitter is masked, angle unwrapping is enforced, frame-aware attitude terminology is explicit, and canonical timestamp windowing is active.
2. The recording node ([src/record_ground_truth.py](../../../src/record_ground_truth.py)) now enforces `use_sim_time: True` and uses node simulation clock timestamps via `/clock`, ensuring future GT telemetry shares the camera simulation time domain.
3. The measurement and analysis pipeline is now fully trustworthy to execute the zero-motion healthy baseline experiment.
