# Monocular VO Project — Task Audit & TODO List

## Current Task Audit (Phase 0 Fix: Offboard Flight vs. Teleportation)

The following table summarizes the audit conducted on the active trajectory generation pass:

| # | Task Requirement | Status | Detailed Audit Findings |
| :--- | :--- | :---: | :--- |
| **1** | **Re-run textured-world flight via `fly_trajectory.py`** | ⚠️ **Partial** | `src/fly_trajectory.py` was adapted to stream offboard setpoints ($v_x = 0.6\text{ m/s}$, $v_y = 0.15\text{ m/s}$, $z = 2.0\text{ m}$). Ground truth log `results/ground_truth_textured_REAL.csv` was created (~20s duration), but flight was interrupted while climbing to ~1.05m before completing the full ~2.0m trajectory. |
| **2** | **Adapt `fly_trajectory.py` waypoints/path** | ✅ **Complete** | Waypoints and velocity setpoint logic in `src/fly_trajectory.py` are properly configured for real PX4 offboard flight. |
| **3** | **Concurrently record Ground Truth & VO** | ❌ **Not Done** | `ground_truth_textured_REAL.csv` was recorded, but `vo_trajectory_textured.csv` was **not** recorded concurrently during this real flight. Current `vo_trajectory_textured.csv` in `results/` is still from the previous `animate_drone.py` teleportation run. |
| **4** | **Ground Truth Sanity Checks (Z-axis smoothness)** | ⚠️ **Partial** | **Z-axis is confirmed smooth** without sawtooth drops (max step change $\Delta z = 0.011\text{ m}$, std dev $0.0028\text{ m}$). However, altitude reached was ~1.05m rather than sustained ~2.0m because flight stopped early. |
| **5** | **Re-run `correlation.py`** | ⚠️ **Incomplete Data** | Running `correlation.py` against old VO (`animate_drone.py`) and new GT (`ground_truth_textured_REAL.csv`) yields $r_x = +0.8417$, $r_y = +0.1922$, $r_z = +0.3031$. True correlation requires concurrent VO + GT capture during real flight. |
| **6** | **Update `failure_log.md`** | ❌ **Not Done** | `failure_log.md` currently ends at Entry 7 (Phase 0D). Entry 8 (documenting teleportation bug, root cause, and fix) needs to be written once correlation results are verified. |

---

## Action Plan / Next Steps

1. **Execute Concurrent Real Flight Pass**:
   - Launch simulation in textured world (`configs/textured.sdf`).
   - Run `src/record_ground_truth.py` and `src/minimal_vo.py` concurrently while executing `src/fly_trajectory.py`.
   - Save outputs to `results/ground_truth_textured_REAL.csv` and `results/vo_trajectory_textured_REAL.csv`.

2. **Verify Ground Truth Trajectory**:
   - Perform sanity checks (timestamps unique/varying, positions changing, unique count).
   - Confirm `pos_z` is smooth and close to the commanded ~2.0m altitude throughout without sawtooth drops.

3. **Compute Axis Shape Correlation**:
   - Run `python3 src/correlation.py --gt-csv results/ground_truth_textured_REAL.csv --vo-csv results/vo_trajectory_textured_REAL.csv`.
   - Report per-axis Pearson correlation coefficients ($r_x, r_y, r_z$), ensuring Z-axis correlation is high and smooth.

4. **Document Bug in `failure_log.md`**:
   - Add Entry 8 to `failure_log.md` detailing:
     - Root cause: `animate_drone.py` Gazebo `set_pose` teleportation + disarmed motors + gravity causing sawtooth freefall on Z.
     - Why 0A/0B missed it: Phase 0A/0B used real PX4 offboard flight.
     - Resolution: Switched all future trajectory generation to real PX4 offboard flight via `fly_trajectory.py`.

5. **Sim(3) Alignment & Benchmark**:
   - Run `src/evaluate_trajectory.py` to calculate final ATE and RPE metrics and generate evaluation plots.
