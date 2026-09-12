# Phase 3 Exploratory Family Definitions

> [!IMPORTANT]
> **Mandatory Status Disclaimer**:
> These families (`F3`, `F7`, `F8`, and `HOVER_L0`) were **NOT** part of the original Phase 1 pilot characterization (P01–P08). They are exploratory additions for Phase 3, have **NOT** undergone Phase 1's rigorous pilot-then-confirmation-batch validation process, and their results must be interpreted as **preliminary**, not as core validated severity-matrix findings.

---

## 1. Exploratory Family Motion Profiles & Intended Parameters

### F3_L2 — Vertical Climb / Descend Oscillation
- **Motion Description**: Pure vertical altitude oscillation around cruise altitude $Z_0 = 2.41$m ENU while maintaining fixed $X, Y$ position and fixed heading ($\psi = 0^\circ$).
- **Intended Parameters**:
  - Longitudinal / Lateral Velocity: $v_x = 0.0$ m/s, $v_y = 0.0$ m/s
  - Vertical Altitude Trajectory: $z(t) = 2.41 + 0.5 \cdot \sin(2\pi \cdot 0.25 \cdot t)$ meters (Amplitude $A_z = 0.50$m, Frequency $f_z = 0.25$ Hz, Period $T = 4.0$s)
  - Vertical Velocity: $v_z(t) = 0.5 \cdot (2\pi \cdot 0.25) \cdot \cos(2\pi \cdot 0.25 \cdot t) \approx \pm 0.785$ m/s
  - Angular Rates: $\omega_x = 0.0, \omega_y = 0.0, \omega_z = 0.0^\circ/\text{s}$
  - Intended Duration: $20.0$ seconds after takeoff stability gate ($Z \ge 2.0$m)
- **Rationale for Phase 3 Addition**: Added to test whether pure vertical translation baseline change (inducing uniform feature scale variation without optical flow rotation) impacts monocular scale recovery or feature tracking stability.

### F7_L2 — Coupled Pitch & Yaw Translation
- **Motion Description**: Forward longitudinal translation along PX4 Local $+Y$ (Gazebo world $+X$) coupled with simultaneous pitch oscillation ($X$-axis position wiggle) and continuous yaw oscillation.
- **Intended Parameters**:
  - Cruise Forward Velocity: $v_{fwd} = 1.0$ m/s
  - Pitch Position Wiggle: $x(t) = A_x \sin(2\pi f_x t)$ with $A_x = 0.159$m ($0.5$m $/ 2\pi f_x$), $f_x = 0.50$ Hz
  - Yaw Angle Trajectory: $\psi(t) = 20.0^\circ \cdot \sin(2\pi \cdot 0.25 \cdot t)$ (Amplitude $A_\psi = 20.0^\circ$, Frequency $f_\psi = 0.25$ Hz)
  - Peak Yaw Rate: $\omega_{z, max} = 20.0 \cdot (2\pi \cdot 0.25) \approx 31.42^\circ/\text{s}$
  - Intended Duration: $20.0$ seconds after takeoff stability gate ($Z \ge 2.0$m)
- **Rationale for Phase 3 Addition**: Added to explore EIS-GATED behavior under lower peak yaw rates ($31.4^\circ/\text{s}$) coupled with pitch-induced translation.

### F8_L2 — Coupled Roll & Yaw Translation
- **Motion Description**: Forward longitudinal translation along PX4 Local $+Y$ (Gazebo world $+X$) coupled with dynamic roll oscillation (East position wiggle) and continuous yaw oscillation.
- **Intended Parameters**:
  - Cruise Forward Velocity: $v_{fwd} = 1.0$ m/s
  - Roll Position Wiggle: $y(t) = v_{fwd} t + A_y \sin(2\pi f_y t)$ with $A_y = 0.159$m, $f_y = 0.50$ Hz
  - Yaw Angle Trajectory: $\psi(t) = 20.0^\circ \cdot \sin(2\pi \cdot 0.25 \cdot t)$ (Amplitude $A_\psi = 20.0^\circ$, Frequency $f_\psi = 0.25$ Hz)
  - Peak Yaw Rate: $\omega_{z, max} = 31.42^\circ/\text{s}$
  - Intended Duration: $20.0$ seconds after takeoff stability gate ($Z \ge 2.0$m)
- **Rationale for Phase 3 Addition**: Added to explore EIS-GATED behavior under coupled roll-yaw dynamics at moderate severity.

---

## 2. Stationary Hover Diagnostic Baseline

### HOVER_L0 — Zero-Motion Diagnostic Baseline
- **Motion Description**: Stationary hover at local origin $(0.0, 0.0)$ at cruise altitude $Z = 2.41$m ENU for $20.0$ seconds.
- **Intended Parameters**:
  - Target Position: $X = 0.0, Y = 0.0, Z = 2.41$m
  - Target Velocities: $v_x = 0.0, v_y = 0.0, v_z = 0.0$ m/s
  - Target Yaw / Rates: $\psi = 0^\circ, \omega = 0^\circ/\text{s}$
- **Classification & Explicit Constraint**:
  > [!WARNING]
  > **Diagnostic Classification Constraint**:
  > `HOVER_L0` is a **zero-parallax diagnostic control**, NOT a moving-flight severity condition. Because static hover lacks translational baseline parallax, monocular VO frame-to-frame translation estimation is ill-conditioned. `HOVER_L0` results must **NEVER** be aggregated into moving-flight statistics for either Core or Exploratory data tables.

---

## 3. Dataset Naming Convention & Isolation Standard

To ensure exploratory datasets can never be mixed into Core Matrix statistics:
- **Core Matrix Prefix**: `p3_{family}_{severity}_{mechanism}_{run}` (e.g., `p3_F1_L2_raw_R1`)
- **Exploratory Prefix**: `p3x_{family}_{severity}_{mechanism}_{run}` (e.g., `p3x_F3_L2_raw_R1`, `p3x_HOVER_L0_eis-gated_R1`)
