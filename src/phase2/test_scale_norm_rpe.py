#!/usr/bin/env python3
import os
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation as R_scipy
from scipy.spatial.transform import Slerp

from evo.core import trajectory, metrics

def load_and_prep_data(data_dir):
    raw_vo_csv = os.path.join(data_dir, "raw_vo.csv")
    gated_vo_csv = os.path.join(data_dir, "eis_gated_vo.csv")
    gt_csv = os.path.join(data_dir, "dataset_gt.csv")

    df_raw = pd.read_csv(raw_vo_csv)
    df_gated = pd.read_csv(gated_vo_csv)
    df_gt = pd.read_csv(gt_csv)

    gt_t = df_gt['timestamp_total_sec'].values.astype(float)
    z_gt = df_gt['pos_z'].values.astype(float)
    idx_act = np.where(z_gt >= 2.0)[0]
    t_start, t_end = gt_t[idx_act[0]], gt_t[idx_act[-1]]

    mask_raw = (df_raw['timestamp_total_sec'].values >= t_start) & (df_raw['timestamp_total_sec'].values <= t_end)
    mask_gated = (df_gated['timestamp_total_sec'].values >= t_start) & (df_gated['timestamp_total_sec'].values <= t_end)

    df_raw_act = df_raw[mask_raw].reset_index(drop=True)
    df_gated_act = df_gated[mask_gated].reset_index(drop=True)

    gt_t_clean, u_idx = np.unique(gt_t, return_index=True)
    gt_pos_clean = df_gt[['pos_x', 'pos_y', 'pos_z']].values[u_idx]
    gt_rot_clean = R_scipy.from_quat(df_gt[['rot_x', 'rot_y', 'rot_z', 'rot_w']].values[u_idx])
    slerp = Slerp(gt_t_clean, gt_rot_clean)

    def to_wxyz(q):
        return np.column_stack([q[:, 3], q[:, 0], q[:, 1], q[:, 2]])

    def build_trajs(df_act):
        ts = df_act['timestamp_total_sec'].values.astype(float)
        interp_px = np.interp(ts, gt_t_clean, gt_pos_clean[:, 0])
        interp_py = np.interp(ts, gt_t_clean, gt_pos_clean[:, 1])
        interp_pz = np.interp(ts, gt_t_clean, gt_pos_clean[:, 2])
        gt_pos_interp = np.column_stack([interp_px, interp_py, interp_pz])
        gt_quats = slerp(np.clip(ts, gt_t_clean[0], gt_t_clean[-1])).as_quat()

        vo_pos = df_act[['pos_x', 'pos_y', 'pos_z']].values.astype(float)
        vo_quats = df_act[['rot_x', 'rot_y', 'rot_z', 'rot_w']].values.astype(float)

        traj_gt = trajectory.PoseTrajectory3D(positions_xyz=gt_pos_interp, orientations_quat_wxyz=to_wxyz(gt_quats), timestamps=ts)
        traj_vo = trajectory.PoseTrajectory3D(positions_xyz=vo_pos, orientations_quat_wxyz=to_wxyz(vo_quats), timestamps=ts)
        return traj_gt, traj_vo

    traj_gt_raw, traj_vo_raw = build_trajs(df_raw_act)
    traj_gt_gated, traj_vo_gated = build_trajs(df_gated_act)

    return (traj_gt_raw, traj_vo_raw), (traj_gt_gated, traj_vo_gated)

def evaluate_both_rpe():
    repeats = ["phase2a_F9_L2_R1", "phase2a_F9_L2_R2", "phase2a_F9_L2_R3"]
    
    print("==========================================================================")
    print("F9 RAW vs EIS-GATED: COMPARISON OF RPE-t (METERS) vs RPE-t (SCALE-NORMALIZED)")
    print("==========================================================================\n")

    raw_rpe_m_list = []
    gated_rpe_m_list = []
    raw_rpe_norm_list = []
    gated_rpe_norm_list = []
    raw_s_list = []
    gated_s_list = []

    for r_dir in repeats:
        r_path = os.path.join("results/datasets", r_dir)
        (gt_r, vo_raw_r), (gt_g, vo_gated_r) = load_and_prep_data(r_path)

        # RAW
        t_r_al = trajectory.PoseTrajectory3D(positions_xyz=np.copy(vo_raw_r.positions_xyz), orientations_quat_wxyz=np.copy(vo_raw_r.orientations_quat_wxyz), timestamps=np.copy(vo_raw_r.timestamps))
        _, _, s_r = t_r_al.align(gt_r, correct_scale=True)
        rpe_r = metrics.RPE(metrics.PoseRelation.translation_part, delta=1, delta_unit=metrics.Unit.frames)
        rpe_r.process_data((gt_r, t_r_al))
        rpe_r_m = rpe_r.get_statistic(metrics.StatisticsType.mean)
        rpe_r_norm = rpe_r_m / s_r

        # GATED
        t_g_al = trajectory.PoseTrajectory3D(positions_xyz=np.copy(vo_gated_r.positions_xyz), orientations_quat_wxyz=np.copy(vo_gated_r.orientations_quat_wxyz), timestamps=np.copy(vo_gated_r.timestamps))
        _, _, s_g = t_g_al.align(gt_g, correct_scale=True)
        rpe_g = metrics.RPE(metrics.PoseRelation.translation_part, delta=1, delta_unit=metrics.Unit.frames)
        rpe_g.process_data((gt_g, t_g_al))
        rpe_g_m = rpe_g.get_statistic(metrics.StatisticsType.mean)
        rpe_g_norm = rpe_g_m / s_g

        raw_rpe_m_list.append(rpe_r_m)
        gated_rpe_m_list.append(rpe_g_m)
        raw_rpe_norm_list.append(rpe_r_norm)
        gated_rpe_norm_list.append(rpe_g_norm)
        raw_s_list.append(s_r)
        gated_s_list.append(s_g)

        print(f"[{r_dir}]")
        print(f"  RAW   : Scale s = {s_r:.4f} | RPE-t (meters) = {rpe_r_m:.4f} m/step | RPE-t (scale-normalized) = {rpe_r_norm:.4f} units/step")
        print(f"  GATED : Scale s = {s_g:.4f} | RPE-t (meters) = {rpe_g_m:.4f} m/step | RPE-t (scale-normalized) = {rpe_g_norm:.4f} units/step")
        print(f"  RPE-t Difference (meters)             : GATED - RAW = {rpe_g_m - rpe_r_m:+.4f} m/step (GATED higher by {(rpe_g_m/rpe_r_m - 1)*100:+.2f}%)")
        print(f"  RPE-t Difference (scale-normalized)   : GATED - RAW = {rpe_g_norm - rpe_r_norm:+.4f} units/step (GATED diff {(rpe_g_norm/rpe_r_norm - 1)*100:+.2f}%)\n")

    print("--- AGGREGATE SUMMARY (n=3 REPEATS) ---")
    print(f"RAW   RPE-t (meters)           : Mean = {np.mean(raw_rpe_m_list):.4f} +/- {np.std(raw_rpe_m_list):.4f} m/step")
    print(f"GATED RPE-t (meters)           : Mean = {np.mean(gated_rpe_m_list):.4f} +/- {np.std(gated_rpe_m_list):.4f} m/step")
    print(f"RAW   RPE-t (scale-normalized) : Mean = {np.mean(raw_rpe_norm_list):.4f} +/- {np.std(raw_rpe_norm_list):.4f} units/step")
    print(f"GATED RPE-t (scale-normalized) : Mean = {np.mean(gated_rpe_norm_list):.4f} +/- {np.std(gated_rpe_norm_list):.4f} units/step")

    diff_norm_mean = np.mean(gated_rpe_norm_list) - np.mean(raw_rpe_norm_list)
    pct_norm_diff = (np.mean(gated_rpe_norm_list) / np.mean(raw_rpe_norm_list) - 1) * 100
    print(f"\nScale-Normalized RPE-t Difference: {diff_norm_mean:+.4f} units/step ({pct_norm_diff:+.2f}%)")
    
if __name__ == '__main__':
    evaluate_both_rpe()
