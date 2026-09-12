#!/usr/bin/env python3
"""
Phase 2B Candidate 4 Evaluation & Characterization Script

Processes phase2b_candidate4_L2_R1 in:
  - RAW (unwarped)
  - EIS-GATED (reference_mode="gated", threshold = 15.0 deg/s)

Computes canonical active window metrics (Z >= 2.0m), gate bypass rate,
achieved yaw amplitude and rate range, and VFO spatial distribution score.
"""

import os
import sys
import math
import cv2
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation as R_scipy

sys.path.append('src')
from run_offline_vo import OfflineVOProcessor
from eis_derotation import EISDerotator
from vfo_observatory import VisualFieldObservatory


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


def analyze_vo_csv(df_vo, t_start, t_end):
    vo_t = df_vo['timestamp_total_sec'].values.astype(float)
    mask = (vo_t >= t_start) & (vo_t <= t_end)
    df_act = df_vo[mask].copy()

    if len(df_act) == 0:
        return {}

    n_E = df_act['num_inliers_E'].values.astype(float)
    n_pose = df_act['num_inliers_pose'].values.astype(float)
    matched = df_act['num_matched'].values.astype(float)

    valid_pose_count = int(np.sum(n_pose >= 8))
    valid_pose_pct = (valid_pose_count / len(df_act)) * 100.0

    pose_e_ratio = np.where(n_E > 0, n_pose / n_E, 0.0)
    survival = df_act['feature_survival_rate'].astype(float).values
    fvel = df_act['feature_vel_mean'].astype(float).values
    lk = df_act['mean_lk_err'].astype(float).values

    crop_pct = df_act['eis_crop_pct'].astype(float).values if 'eis_crop_pct' in df_act.columns else np.zeros(len(df_act))
    warp_deg = df_act['eis_warp_deg'].astype(float).values if 'eis_warp_deg' in df_act.columns else np.zeros(len(df_act))

    gate_scale = df_act['eis_gate_scale'].astype(float).values if 'eis_gate_scale' in df_act.columns else np.ones(len(df_act))
    yaw_rate_deg = df_act['eis_yaw_rate_deg'].astype(float).values if 'eis_yaw_rate_deg' in df_act.columns else np.zeros(len(df_act))

    bypass_pct = (float(np.sum(gate_scale < 1.0)) / len(df_act)) * 100.0

    return {
        'n_frames': len(df_act),
        'valid_pose_pct': valid_pose_pct,
        'n_E_mean': float(n_E.mean()),
        'n_E_median': float(np.median(n_E)),
        'n_pose_mean': float(n_pose.mean()),
        'n_pose_median': float(np.median(n_pose)),
        'pose_e_ratio': float(pose_e_ratio.mean()),
        'survival': float(survival.mean()),
        'fvel_mean': float(fvel.mean()),
        'lk_mean': float(lk.mean()),
        'crop_pct_mean': float(crop_pct.mean()),
        'warp_deg_mean': float(warp_deg.mean()),
        'gate_scale_mean': float(gate_scale.mean()),
        'yaw_rate_mean': float(yaw_rate_deg.mean()),
        'bypass_pct': bypass_pct
    }


def analyze_achieved_telemetry(df_gt, t_start, t_end):
    t_col = 'timestamp_total_sec' if 'timestamp_total_sec' in df_gt.columns else 'timestamp'
    gt_t = df_gt[t_col].values.astype(float)
    if gt_t[0] > 1e12:
        gt_t = gt_t / 1e9

    mask = (gt_t >= t_start) & (gt_t <= t_end)
    df_act = df_gt[mask].copy().reset_index(drop=True)

    qx = df_act['rot_x'].values.astype(float)
    qy = df_act['rot_y'].values.astype(float)
    qz = df_act['rot_z'].values.astype(float)
    qw = df_act['rot_w'].values.astype(float)

    quats = np.column_stack([qx, qy, qz, qw])
    rotations = R_scipy.from_quat(quats)
    eulers_deg = rotations.as_euler('xyz', degrees=True)
    yaws_deg = eulers_deg[:, 2] # Z-axis yaw
    yaws_unwrapped = np.degrees(np.unwrap(np.radians(yaws_deg)))

    min_yaw = float(np.min(yaws_unwrapped))
    max_yaw = float(np.max(yaws_unwrapped))
    peak_to_peak_yaw = float(max_yaw - min_yaw)
    half_amplitude = float(peak_to_peak_yaw / 2.0)

    # Compute yaw rates
    act_t = df_act[t_col].values.astype(float)
    w_z_list = []
    for i in range(1, len(act_t)):
        dt = max(1e-4, act_t[i] - act_t[i-1])
        R_prev = rotations[i-1].as_matrix()
        R_curr = rotations[i].as_matrix()
        R_rel = R_prev.T @ R_curr
        rotvec = R_scipy.from_matrix(R_rel).as_rotvec()
        w_z = abs(math.degrees(rotvec[2] / dt))
        w_z_list.append(w_z)

    w_z_arr = np.array(w_z_list) if len(w_z_list) > 0 else np.zeros(1)
    w_z_filt = w_z_arr[w_z_arr < 500.0]

    return {
        'min_yaw_deg': min_yaw,
        'max_yaw_deg': max_yaw,
        'peak_to_peak_yaw_deg': peak_to_peak_yaw,
        'half_amplitude_deg': half_amplitude,
        'mean_w_z_deg_s': float(np.mean(w_z_filt)),
        'median_w_z_deg_s': float(np.median(w_z_filt)),
        'p95_w_z_deg_s': float(np.percentile(w_z_filt, 95)),
        'max_w_z_deg_s': float(np.max(w_z_filt))
    }


def analyze_vfo_spatial_distribution(dataset_dir, t_start, t_end, gt_csv_path):
    cam_csv_path = os.path.join(dataset_dir, 'camera_frames.csv')
    df_cam = pd.read_csv(cam_csv_path)

    eis_derotator = EISDerotator(reference_mode='incremental')
    eis_derotator.load_attitude_telemetry(gt_csv_path)
    vfo = VisualFieldObservatory(eis_derotator=eis_derotator)

    spatial_scores = []
    for _, row in df_cam.iterrows():
        total_sec = float(row['timestamp_total_sec'])
        if t_start <= total_sec <= t_end:
            img_path = os.path.join(dataset_dir, 'images', row['filename'])
            cv_img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
            rec = vfo.process_frame(cv_img, total_sec)
            spatial_scores.append(rec['spatial_distribution_score'])

    return {
        'spatial_distribution_mean': float(np.mean(spatial_scores)) if len(spatial_scores) > 0 else 0.0,
        'spatial_distribution_std': float(np.std(spatial_scores)) if len(spatial_scores) > 0 else 0.0
    }


def main():
    run_id = 'phase2b_candidate4_L2_R1'
    dataset_dir = f"results/datasets/{run_id}"

    gt_csv = os.path.join(dataset_dir, 'dataset_gt.csv')
    cam_csv = os.path.join(dataset_dir, 'camera_frames.csv')
    raw_csv = os.path.join(dataset_dir, 'raw_vo.csv')
    gated_csv = os.path.join(dataset_dir, 'eis_gated_vo.csv')

    df_cam = pd.read_csv(cam_csv)
    df_gt = pd.read_csv(gt_csv)
    t_start, t_end, active_dur = get_canonical_active_window(df_gt)

    df_raw_out = pd.read_csv(raw_csv)
    df_gated_out = pd.read_csv(gated_csv)

    m_raw = analyze_vo_csv(df_raw_out, t_start, t_end)
    m_gated = analyze_vo_csv(df_gated_out, t_start, t_end)
    telem = analyze_achieved_telemetry(df_gt, t_start, t_end)

    vfo_spatial = analyze_vfo_spatial_distribution(dataset_dir, t_start, t_end, gt_csv)

    print("==========================================================================")
    print(f"CANDIDATE 4 FIRST LOOK RESULTS: {run_id}")
    print("==========================================================================")
    print(f"Active Window Duration : {active_dur:.2f} s ({m_raw['n_frames']} frames)")
    print("--------------------------------------------------------------------------")
    print(f"Achieved Yaw Range     : {telem['min_yaw_deg']:.2f}° to {telem['max_yaw_deg']:.2f}° (Peak-to-Peak: {telem['peak_to_peak_yaw_deg']:.2f}°, Amplitude: ±{telem['half_amplitude_deg']:.2f}°)")
    print(f"Achieved Yaw Rate |w_z|: Mean {telem['mean_w_z_deg_s']:.2f} deg/s (Median: {telem['median_w_z_deg_s']:.2f}, P95: {telem['p95_w_z_deg_s']:.2f})")
    print("--------------------------------------------------------------------------")
    print(f"RAW Pose Validity      : {m_raw['valid_pose_pct']:.2f}%  (Pose/E: {m_raw['pose_e_ratio']:.3f}, Survival: {m_raw['survival']:.4f}, LK: {m_raw['lk_mean']:.3f} px)")
    print(f"EIS-GATED Pose Validity: {m_gated['valid_pose_pct']:.2f}%  (Pose/E: {m_gated['pose_e_ratio']:.3f}, Survival: {m_gated['survival']:.4f}, LK: {m_gated['lk_mean']:.3f} px)")
    print(f"Gate Bypass Rate       : {m_gated['bypass_pct']:.1f}%")
    print(f"Net Effect vs RAW      : {m_gated['valid_pose_pct'] - m_raw['valid_pose_pct']:+.2f}%")
    print("--------------------------------------------------------------------------")
    print(f"VFO Spatial Entropy    : {vfo_spatial['spatial_distribution_mean']:.4f} ± {vfo_spatial['spatial_distribution_std']:.4f}")
    print("==========================================================================")


if __name__ == '__main__':
    main()
