# Monocular VO Project — Task Audit & TODO List

## Current Status: Phase 0E Finalization & Phase 1 Planning Checkpoint (2026-09-04)

Phase 0E is experimentally complete:
- **Sweep A (Pure Yaw Escalation)**: 10, 20, 40, 80, 120, 180 deg/s evaluated in `boxworld_obstacles_tight`. `findEssentialMat` epipolar RANSAC inliers (`num_inliers_E`) remained $>800$ per frame (`strict_E_fail = FALSE` across all levels). Pure yaw $t=0$ depth filtering truncation identified as a zero-baseline unit-scale artifact.
- **`recoverPose` Forensic Audit & Repair**: Resolved conflation between `num_inliers_E` and `num_inliers_pose`. Adopted `distanceThresh=1000.0` as operating envelope parameter. Validated on physical SITL $10^\circ$ roll flight in `agriculture.world` (90.43% valid pose updates, `strict_E_fail = FALSE`, `strict_pose_fail = FALSE`).
- **Sweep B (Attitude Escalation)**: 5, 10, 20, 30, 40, 50 deg roll oscillation at 0.5 Hz evaluated in `agriculture.world`. `strict_E_fail = FALSE` and `strict_pose_fail = FALSE` across all 6 levels ($87.44\% - 93.56\%$ valid pose updates).
- **Experimental World Policy**: `agriculture.world` designated as PRIMARY research/testing environment for Phase 1+.

---

## Phase 0 Hard Gate Milestones Summary

| Milestone | Description | Status | Reference / Artifact |
| :---: | :--- | :---: | :--- |
| **0A** | Sensor + Ground Truth Verification | ✅ **Complete** | 30.4 Hz RGB image + 50 Hz World ENU pose |
| **0B** | Timestamp Synchronization | ✅ **Complete** | Nearest-neighbor SimTime matching ($\le 20\text{ms}$) |
| **0C** | Minimal Un-fused Monocular VO Node | ✅ **Complete** | `src/minimal_vo.py` (GFTT + KLT + 5-pt E-RANSAC) |
| **0D** | Trajectory Alignment & Evaluation | ✅ **Complete** | `src/evaluate_trajectory.py` Sim(3) Umeyama alignment |
| **0E** | Reproducibility & Scientific Characterization | ✅ **Complete** | Sweep A, Sweep B, metric repair & 10° validation |

---

## Forward Action Plan (Phase 1 Ready)

1. **Phase 0E Final Documentation & Audit**:
   - Audit all Phase 0E summary reports (`yaw_sweep_phase0e_v2_summary.md`, `tilt_sweep_phase0e_v3_summary.md`, `recoverpose_*.md`).
   - Update `failure_log.md` with historically accurate reinterpretations of `recoverPose` conflation and Phase 0E synthesis.
   - Establish forward Experimental World Policy (`agriculture.world` as primary test ground).

2. **Phase 1 Experiment Plan**:
   - Create `results/phase1_experiment_plan.md` defining 19 required plan components (objectives, severity matrix, achieved-motion metrics, failure criteria, sliding window definitions, metric separation, statistical repetition, confounds, dataset versioning, run invalidation rules).
   - Freeze VO algorithm, parameters, and simulation execution (no flight runs during Phase 1 planning).

3. **Git Checkpoint Commit & Sync**:
   - Verify clean working tree (exclude temporary logs, caches, and raw datasets).
   - Create checkpoint commit: `"Finalize Phase 0E and plan Phase 1"`.
   - Push commit to `origin/main`.

