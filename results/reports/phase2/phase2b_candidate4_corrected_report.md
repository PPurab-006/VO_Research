# Phase 2B Candidate 4 Correction Report: Yaw Sweep Amplitude Audit & HARD GATE Enforcement

**Author**: Antigravity Assistant & Visual Navigation Lead  
**Date**: September 11, 2026  
**Scope**: Root cause analysis of Candidate 4 trajectory bugs across three flight attempts (`R1`, `R2`, `R3`), offline feedforward math validation, and systematic PX4 control interaction report.

---

> [!CAUTION]
> **HARD GATE STATUS: TRIGGERED / EXECUTION HALTED (R3 FAILS HARD GATE)**  
> Per prompt instructions, because `R3` failed the HARD GATE (achieved $125.51^\circ$ peak-to-peak amplitude vs intended $40-90^\circ$ range), **NO FOURTH AD-HOC TWEAK WAS ATTEMPTED**. Execution was halted and VO analysis was withheld. The systematic failure pattern across all three attempts is reported below.

---

## 1. Step 1 — Offline Feedforward Math Validation (PASSED)

Before flying `R3`, the commanded setpoint sequence was simulated offline for the full $20.0\text{s}$ duration ($dt = 0.05\text{s}$, 20 Hz) using [validate_feedforward_math.py](../../../scratch/validate_feedforward_math.py):

- **Commanded Yaw Target**: $\psi(t) = A_{\text{yaw}} \sin(2\pi f_{\text{yaw}} t)$ ($A_{\text{yaw}} = 30.0^\circ$, $f_{\text{yaw}} = 0.25\text{ Hz}$)
- **Analytical Yaw Rate Feedforward**: $\dot{\psi}(t) = 2\pi f_{\text{yaw}} A_{\text{yaw}} \cos(2\pi f_{\text{yaw}} t)$ ($\dot{\psi}_{\text{peak}} = 47.1239\text{ deg/s}$)
- **Numerical Verification**:
  - Finite-difference derivative error vs analytical feedforward: Max error = **0.0484 deg/s**, RMS error = **0.0342 deg/s** (pure discretisation artifact).
  - Phase and Quadrant Alignment:
    - $t = 0.0\text{s}$: $\psi = 0.0^\circ$, $\dot{\psi} = +47.12\text{ deg/s}$ (Zero crossing, rising max rate).
    - $t = 1.0\text{s}$: $\psi = +30.0^\circ$, $\dot{\psi} = 0.0\text{ deg/s}$ (Positive peak, zero rate).
    - $t = 2.0\text{s}$: $\psi = 0.0^\circ$, $\dot{\psi} = -47.12\text{ deg/s}$ (Zero crossing, falling max rate).
    - $t = 3.0\text{s}$: $\psi = -30.0^\circ$, $\dot{\psi} = 0.0\text{ deg/s}$ (Negative peak, zero rate).
- **Result**: OFFLINE feedforward math is mathematically exact and verified.

---

## 2. Step 2 & 3 — Telemetry Comparison Across Attempts R1, R2, and R3

| Flight Attempt | Setpoint Mask Configuration | Commanded Target | Achieved Yaw Range | Achieved Peak-to-Peak | Sign Reversals | Gate Status | Primary Failure Mode |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :--- |
| **R1 (Initial)** | Mask `2552` (`yaw_rate = 0.0`, Bit 10=0) | $\pm 30.0^\circ$ Pos Yaw | $-0.96^\circ \to +118.46^\circ$ | **119.42°** | 0 (Monotonic) | **FAIL** | Integrator Windup / Monotonic Drift |
| **R2 (Re-Fly 1)** | Mask `3576` (`IGNORE_YAW_RATE` = true) | $\pm 30.0^\circ$ Pos Yaw | $-0.00^\circ \to +2.40^\circ$ | **2.40°** | 9 ($0.25\text{ Hz}$) | **FAIL** | Severe Controller Over-Damping ($1/25\times$) |
| **R3 (Re-Fly 2)** | Mask `504` (Pos + Yaw + Rate FF) | $\pm 30.0^\circ$ Pos + $\pm 47.1^\circ/\text{s}$ Rate | $-0.01^\circ \to +125.50^\circ$ | **125.51°** | 14 ($0.25\text{ Hz}$) | **FAIL** | Controller Over-Amplification / Ratchet ($2.1\times$) |

---

## 3. Systematic Pattern Analysis of PX4 Attitude/Position Control Interaction

The emergence of **three completely distinct failure modes across three consecutive attempts** reveals a fundamental control structure interaction within PX4 SITL's multi-copter attitude controller under MAVLink `SET_POSITION_TARGET_LOCAL_NED`:

1. **Failure Mode 1 (R1 - Monotonic Drift)**:
   - When sending positional yaw target $\psi(t)$ while MAVLink mask bit 10 is 0 and `yaw_rate = 0.0`, PX4's position controller interprets `yaw_rate = 0.0` as an explicit zero-velocity constraint. The conflict between non-zero position error and zero-rate constraint causes the internal heading integrator to accumulate monotonically, producing a unidirectional drift up to $+118.46^\circ$.

2. **Failure Mode 2 (R2 - Over-Damping)**:
   - When setting bit 10 to ignore yaw rate feedforward (`mask = 3576`), PX4 relies solely on P-gain position-to-heading feedback. During fast forward translation ($v_{\text{fwd}} = 1.0\text{ m/s}$), PX4's position loop prioritizes translational trajectory tracking over heading tracking, heavily filtering high-frequency ($0.25\text{ Hz}$) heading commands and attenuating the achieved response from $\pm 30.0^\circ$ down to $\pm 1.20^\circ$.

3. **Failure Mode 3 (R3 - Over-Amplification / Drift Ratchet)**:
   - When enabling both positional yaw setpoint and analytical rate feedforward (`mask = 504`), PX4 sums the feedforward rate directly into the inner attitude rate loop while the outer position loop also injects corrective yaw rate. Because the inner rate loop integrated $\dot{\psi}(t)$ without anti-windup decoupling from the outer loop during translation, the achieved response over-amplified to $125.51^\circ$ peak-to-peak.

---

## 4. Current Status & Strategic Recommendation

- **HARD GATE Compliance**: In strict adherence to prompt instructions, no further ad-hoc parameter tweaks were attempted in this session.
- **VO Analysis Withheld**: VO metrics were not computed on `R1`, `R2`, or `R3` off-spec data.
- **Recommendation**: To achieve a clean $\pm 20-30^\circ$ trajectory without PX4 controller interference, the Candidate 4 trajectory should either be commanded via **pure attitude/rate setpoints** (`SET_ATTITUDE_TARGET`) or through a calibrated PX4 heading feedforward gain scaling before re-flying. Awaiting user guidance.
