#!/usr/bin/env python3
"""
Phase 3 Full ATE/RPE Evaluation Engine using evo (Umeyama Sim(3) Alignment)

Computes per-dataset metrics across active motion window (Z >= 2.0m):
  - Sim(3) Umeyama Scale-Aligned Trajectory Alignment
  - ATE RMSE (meters)
  - Translation RPE (m/step) & Rotation RPE (deg/step)
  - Tracking Loss Rate (%)
  - Recovery Time (seconds / frames)
  - Trajectory Drift per Meter Traveled (m/m)
  - Prediction Lead Time (reported as "N/A -- falsified in Phase 2C (VFO)")
"""

import os
import sys
import math
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation as R_scipy
from scipy.spatial.transform import Slerp

# Import evo core API
from evo.core import trajectory, metrics, sync
from evo.core import geometry


def get_canonical_active_window(df_gt):
    t_col = 'timestamp_total_sec' if 'timestamp_total_sec' in df_gt.columns else 'timestamp'
    gt_t = df_gt[t_col].values.astype(float)
    if gt_t[0] > 1e12:
        gt_t = gt_t / 1e9

    z_gt = df_gt['pos_z'].values.astype(float) if 'pos_z' in df_gt.columns else df_gt['z'].values.astype(float)

    idx_active = np.where(z_gt >= 2.0)[0]
    if len(idx_active) > 0:
        start_idx = idx_active[0]
        end_idx = idx_active[-1]
    else:
        start_idx = 0
        end_idx = len(df_gt) - 1

    t_start = gt_t[start_idx]
    t_end = gt_t[end_idx]
    dur = t_end - t_start
    return t_start, t_end, dur


def evaluate_single_run(vo_csv_path, gt_csv_path):
    df_vo = pd.read_csv(vo_csv_path)
    df_gt = pd.read_csv(gt_csv_path)

    t_start, t_end, active_dur = get_canonical_active_window(df_gt)

    # Filter VO and GT to active window
    vo_t = df_vo['timestamp_total_sec'].values.astype(float)
    vo_mask = (vo_t >= t_start) & (vo_t <= t_end)
    df_vo_act = df_vo[vo_mask].reset_index(drop=True)

    if len(df_vo_act) < 10:
        return None

    # Interpolate GT position and orientation at VO timestamps
    gt_t_raw = df_gt['timestamp_total_sec'].values.astype(float)
    gt_t_clean, unique_idx = np.unique(gt_t_raw, return_index=True)
    
    gt_px = np.interp(df_vo_act['timestamp_total_sec'].values, gt_t_clean, df_gt['pos_x'].values[unique_idx])
    gt_py = np.interp(df_vo_act['timestamp_total_sec'].values, gt_t_clean, df_gt['pos_y'].values[unique_idx])
    gt_pz = np.interp(df_vo_act['timestamp_total_sec'].values, gt_t_clean, df_gt['pos_z'].values[unique_idx])
    gt_pos = np.column_stack([gt_px, gt_py, gt_pz])

    rotations_clean = R_scipy.from_quat(df_gt[['rot_x', 'rot_y', 'rot_z', 'rot_w']].values[unique_idx])
    slerp = Slerp(gt_t_clean, rotations_clean)
    interp_quats = slerp(np.clip(df_vo_act['timestamp_total_sec'].values, gt_t_clean[0], gt_t_clean[-1])).as_quat()

    vo_pos = df_vo_act[['pos_x', 'pos_y', 'pos_z']].values.astype(float)
    vo_quats = df_vo_act[['rot_x', 'rot_y', 'rot_z', 'rot_w']].values.astype(float)

    timestamps = df_vo_act['timestamp_total_sec'].values.astype(float)

    # Construct evo PoseTrajectory3D objects
    traj_gt = trajectory.PoseTrajectory3D(positions_xyz=gt_pos, orientations_quat_wxyz=gt_quats_to_wxyz(interp_quats), timestamps=timestamps)
    traj_vo = trajectory.PoseTrajectory3D(positions_xyz=vo_pos, orientations_quat_wxyz=gt_quats_to_wxyz(vo_quats), timestamps=timestamps)

    # Perform Sim(3) Umeyama Scale Alignment
    traj_vo_aligned = copy_trajectory(traj_vo)
    r_mat, t_vec, s_factor = traj_vo_aligned.align(traj_gt, correct_scale=True)

    # 1. ATE RMSE (Sim3 Aligned)
    ape_metric = metrics.APE(metrics.PoseRelation.translation_part)
    ape_metric.process_data((traj_gt, traj_vo_aligned))
    ate_rmse = ape_metric.get_statistic(metrics.StatisticsType.rmse)

    # 2. Translation RPE (m/step) and Scale-Normalized RPE-t (unit-scale)
    rpe_t_metric = metrics.RPE(metrics.PoseRelation.translation_part, delta=1, delta_unit=metrics.Unit.frames)
    rpe_t_metric.process_data((traj_gt, traj_vo_aligned))
    rpe_t_mean = rpe_t_metric.get_statistic(metrics.StatisticsType.mean)
    rpe_t_norm = float(rpe_t_mean / s_factor) if s_factor > 1e-6 else 0.0

    # 3. Rotation RPE (deg/step)
    rpe_r_metric = metrics.RPE(metrics.PoseRelation.rotation_angle_rad, delta=1, delta_unit=metrics.Unit.frames)
    rpe_r_metric.process_data((traj_gt, traj_vo_aligned))
    rpe_r_mean_deg = np.degrees(rpe_r_metric.get_statistic(metrics.StatisticsType.mean))

    # 4. Tracking Loss Rate (%)
    n_pose = df_vo_act['num_inliers_pose'].values.astype(int)
    invalid_mask = (n_pose < 8)
    tracking_loss_pct = (np.sum(invalid_mask) / len(df_vo_act)) * 100.0

    # Per-step RPE breakdown (valid vs starved frames)
    err_series = rpe_t_metric.error
    step_valid_mask = ~invalid_mask[:-1]
    step_invalid_mask = invalid_mask[:-1]

    rpe_norm_series = err_series / s_factor if s_factor > 1e-6 else err_series
    rpe_t_norm_valid = float(np.mean(rpe_norm_series[step_valid_mask])) if np.sum(step_valid_mask) > 0 else 0.0
    rpe_t_norm_starved = float(np.mean(rpe_norm_series[step_invalid_mask])) if np.sum(step_invalid_mask) > 0 else 0.0

    # 5. Recovery Time (seconds & frames)
    rec_frames, rec_secs = compute_recovery_time(invalid_mask, timestamps)

    # 6. Drift per Meter Traveled (m/m)
    gt_steps = np.linalg.norm(np.diff(gt_pos, axis=0), axis=1)
    gt_path_len = float(np.sum(gt_steps))
    drift_per_meter = float(ate_rmse / gt_path_len) if gt_path_len > 0.1 else 0.0

    return {
        'n_frames': len(df_vo_act),
        'n_valid_frames': int(np.sum(~invalid_mask)),
        'n_invalid_frames': int(np.sum(invalid_mask)),
        'ate_rmse': float(ate_rmse),
        'rpe_t_mean': float(rpe_t_mean),
        'rpe_t_norm': float(rpe_t_norm),
        'rpe_t_norm_valid': float(rpe_t_norm_valid),
        'rpe_t_norm_starved': float(rpe_t_norm_starved),
        'rpe_r_mean_deg': float(rpe_r_mean_deg),
        'tracking_loss_pct': float(tracking_loss_pct),
        'recovery_time_sec': float(rec_secs),
        'recovery_time_frames': float(rec_frames),
        'drift_per_meter': float(drift_per_meter),
        'gt_path_length': gt_path_len,
        'sim3_scale': float(s_factor),
        'valid_pose_pct': float(100.0 - tracking_loss_pct)
    }


def gt_quats_to_wxyz(quats_xyzw):
    return np.column_stack([quats_xyzw[:, 3], quats_xyzw[:, 0], quats_xyzw[:, 1], quats_xyzw[:, 2]])


def copy_trajectory(traj):
    return trajectory.PoseTrajectory3D(
        positions_xyz=np.copy(traj.positions_xyz),
        orientations_quat_wxyz=np.copy(traj.orientations_quat_wxyz),
        timestamps=np.copy(traj.timestamps)
    )


def compute_recovery_time(invalid_mask, timestamps):
    # Find continuous segments of invalid frames (invalid_mask == True) that subsequently recover (valid_mask == True)
    n = len(invalid_mask)
    if n == 0 or np.sum(invalid_mask) == 0:
        return 0.0, 0.0

    recovery_lengths_frames = []
    recovery_lengths_sec = []

    in_gap = False
    gap_start_idx = 0

    for i in range(n):
        if invalid_mask[i] and not in_gap:
            in_gap = True
            gap_start_idx = i
        elif not invalid_mask[i] and in_gap:
            # Recovered!
            gap_len = i - gap_start_idx
            gap_dt = timestamps[i] - timestamps[gap_start_idx]
            recovery_lengths_frames.append(gap_len)
            recovery_lengths_sec.append(gap_dt)
            in_gap = False

    if len(recovery_lengths_frames) > 0:
        return float(np.mean(recovery_lengths_frames)), float(np.mean(recovery_lengths_sec))
    else:
        return 0.0, 0.0


def main():
    if len(sys.argv) >= 3:
        vo_csv = sys.argv[1]
        gt_csv = sys.argv[2]
        res = evaluate_single_run(vo_csv, gt_csv)
        print(f"Evaluation results for {vo_csv}:")
        for k, v in res.items():
            print(f"  {k}: {v}")
    else:
        print("Usage: evaluate_phase3_evo.py <vo_csv> <gt_csv>")


if __name__ == '__main__':
    main()
