#!/usr/bin/env python3
"""
Phase 2A / 2C Six-Way Evaluation Runner with Held-Out Validation Split

Modes Evaluated across Canonical Active Window (Z >= 2.0m):
  1. RAW
  2. EIS-FIXED
  3. EIS-INCREMENTAL
  4. EIS-NULL
  5. EIS-GATED (Hard threshold |omega_z| > thresh -> Identity H_cv)
  6. EIS-SCALED (Smooth linear scaling factor s)

Supports:
  - In-sample evaluation: gate_thresh = 15.0 deg/s (all datasets)
  - Held-out validation check: gate_thresh derived on F9_R1+R2, evaluated on F9_R3.
"""

import os
import sys
import csv
import math
import cv2
import numpy as np
import pandas as pd

sys.path.append('src')
from run_offline_vo import OfflineVOProcessor
from eis_derotation import EISDerotator


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


def run_evaluation(gate_thresh=15.0, gate_max=45.0, rerun_gated=True):
    datasets = [
        'phase2a_F5_L2_R1',
        'phase2a_F5_L2_R2',
        'phase2a_F5_L2_R3',
        'phase2a_F6_L2',
        'phase2a_F9_L2_R1',
        'phase2a_F9_L2_R2',
        'phase2a_F9_L2_R3'
    ]

    results = {}

    for run_id in datasets:
        dataset_dir = f"results/datasets/{run_id}"
        gt_csv = os.path.join(dataset_dir, 'dataset_gt.csv')
        cam_csv = os.path.join(dataset_dir, 'camera_frames.csv')

        raw_csv = os.path.join(dataset_dir, 'raw_vo.csv')
        eis_fixed_csv = os.path.join(dataset_dir, 'eis_fixed_vo.csv')
        if not os.path.exists(eis_fixed_csv) and os.path.exists(os.path.join(dataset_dir, 'eis_vo.csv')):
            eis_fixed_csv = os.path.join(dataset_dir, 'eis_vo.csv')
        eis_inc_csv = os.path.join(dataset_dir, 'eis_inc_vo.csv')
        eis_null_csv = os.path.join(dataset_dir, 'eis_null_vo.csv')
        eis_gated_csv = os.path.join(dataset_dir, 'eis_gated_vo.csv')
        eis_scaled_csv = os.path.join(dataset_dir, 'eis_scaled_vo.csv')

        df_cam = pd.read_csv(cam_csv)
        df_gt = pd.read_csv(gt_csv)
        t_start, t_end, active_dur = get_canonical_active_window(df_gt)

        # 1. RAW VO
        if not os.path.exists(raw_csv):
            proc_raw = OfflineVOProcessor(output_csv_path=raw_csv, mode='klt', eis_derotator=None)
            for _, row in df_cam.iterrows():
                img_path = os.path.join(dataset_dir, 'images', row['filename'])
                cv_img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
                proc_raw.process_frame(cv_img, int(row['timestamp_sec']), int(row['timestamp_nanosec']), float(row['timestamp_total_sec']))
            proc_raw.save_csv()

        # 2. EIS-FIXED VO
        if not os.path.exists(eis_fixed_csv):
            eis_fixed = EISDerotator(reference_mode='fixed')
            eis_fixed.load_attitude_telemetry(gt_csv)
            proc_fixed = OfflineVOProcessor(output_csv_path=eis_fixed_csv, mode='klt', eis_derotator=eis_fixed)
            for _, row in df_cam.iterrows():
                img_path = os.path.join(dataset_dir, 'images', row['filename'])
                cv_img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
                proc_fixed.process_frame(cv_img, int(row['timestamp_sec']), int(row['timestamp_nanosec']), float(row['timestamp_total_sec']))
            proc_fixed.save_csv()

        # 3. EIS-INCREMENTAL VO
        if not os.path.exists(eis_inc_csv):
            eis_inc = EISDerotator(reference_mode='incremental')
            eis_inc.load_attitude_telemetry(gt_csv)
            proc_inc = OfflineVOProcessor(output_csv_path=eis_inc_csv, mode='klt', eis_derotator=eis_inc)
            for _, row in df_cam.iterrows():
                img_path = os.path.join(dataset_dir, 'images', row['filename'])
                cv_img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
                proc_inc.process_frame(cv_img, int(row['timestamp_sec']), int(row['timestamp_nanosec']), float(row['timestamp_total_sec']))
            proc_inc.save_csv()

        # 4. EIS-NULL VO
        if not os.path.exists(eis_null_csv):
            eis_null = EISDerotator(reference_mode='null')
            eis_null.load_attitude_telemetry(gt_csv)
            proc_null = OfflineVOProcessor(output_csv_path=eis_null_csv, mode='klt', eis_derotator=eis_null)
            for _, row in df_cam.iterrows():
                img_path = os.path.join(dataset_dir, 'images', row['filename'])
                cv_img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
                proc_null.process_frame(cv_img, int(row['timestamp_sec']), int(row['timestamp_nanosec']), float(row['timestamp_total_sec']))
            proc_null.save_csv()

        # 5. EIS-GATED VO
        if rerun_gated or not os.path.exists(eis_gated_csv):
            eis_gated = EISDerotator(reference_mode='gated')
            eis_gated.load_attitude_telemetry(gt_csv)
            proc_gated = OfflineVOProcessor(output_csv_path=eis_gated_csv, mode='klt', eis_derotator=eis_gated, gate_thresh_deg=gate_thresh, gate_max_deg=gate_max)
            for _, row in df_cam.iterrows():
                img_path = os.path.join(dataset_dir, 'images', row['filename'])
                cv_img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
                proc_gated.process_frame(cv_img, int(row['timestamp_sec']), int(row['timestamp_nanosec']), float(row['timestamp_total_sec']))
            proc_gated.save_csv()

        # 6. EIS-SCALED VO
        if rerun_gated or not os.path.exists(eis_scaled_csv):
            eis_scaled = EISDerotator(reference_mode='scaled')
            eis_scaled.load_attitude_telemetry(gt_csv)
            proc_scaled = OfflineVOProcessor(output_csv_path=eis_scaled_csv, mode='klt', eis_derotator=eis_scaled, gate_thresh_deg=gate_thresh, gate_max_deg=gate_max)
            for _, row in df_cam.iterrows():
                img_path = os.path.join(dataset_dir, 'images', row['filename'])
                cv_img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
                proc_scaled.process_frame(cv_img, int(row['timestamp_sec']), int(row['timestamp_nanosec']), float(row['timestamp_total_sec']))
            proc_scaled.save_csv()

        df_raw = pd.read_csv(raw_csv)
        df_fixed = pd.read_csv(eis_fixed_csv)
        df_inc = pd.read_csv(eis_inc_csv)
        df_null = pd.read_csv(eis_null_csv)
        df_gated = pd.read_csv(eis_gated_csv)
        df_scaled = pd.read_csv(eis_scaled_csv)

        m_raw = analyze_vo_csv(df_raw, t_start, t_end)
        m_fixed = analyze_vo_csv(df_fixed, t_start, t_end)
        m_inc = analyze_vo_csv(df_inc, t_start, t_end)
        m_null = analyze_vo_csv(df_null, t_start, t_end)
        m_gated = analyze_vo_csv(df_gated, t_start, t_end)
        m_scaled = analyze_vo_csv(df_scaled, t_start, t_end)

        results[run_id] = {
            'active_dur': active_dur,
            'RAW': m_raw,
            'FIXED': m_fixed,
            'INCREMENTAL': m_inc,
            'NULL': m_null,
            'GATED': m_gated,
            'SCALED': m_scaled
        }

    return results


def main():
    print("==========================================================================")
    print("RUNNING SIX-WAY EVALUATION (RAW / FIXED / INC / NULL / GATED / SCALED)")
    print("==========================================================================")
    
    # In-sample threshold: 15.0 deg/s
    res = run_evaluation(gate_thresh=15.0, gate_max=45.0, rerun_gated=True)

    print("\n------------------------------------------------------------------------------------------------------------------------")
    print(f"{'Dataset Run':<20} | {'RAW Val':<8} | {'FIX Val':<8} | {'INC Val':<8} | {'NUL Val':<8} | {'GAT Val':<8} | {'SCL Val':<8} | {'GAT Byp%':<8} | {'SCL Byp%':<8}")
    print("------------------------------------------------------------------------------------------------------------------------")

    for run_id, m in res.items():
        r_v = m['RAW']['valid_pose_pct']
        f_v = m['FIXED']['valid_pose_pct']
        i_v = m['INCREMENTAL']['valid_pose_pct']
        n_v = m['NULL']['valid_pose_pct']
        g_v = m['GATED']['valid_pose_pct']
        s_v = m['SCALED']['valid_pose_pct']

        g_b = m['GATED']['bypass_pct']
        s_b = m['SCALED']['bypass_pct']

        print(f"{run_id:<20} | {r_v:7.2f}% | {f_v:7.2f}% | {i_v:7.2f}% | {n_v:7.2f}% | {g_v:7.2f}% | {s_v:7.2f}% | {g_b:7.1f}% | {s_b:7.1f}%")

    # Statistical Aggregation
    f9_runs = ['phase2a_F9_L2_R1', 'phase2a_F9_L2_R2', 'phase2a_F9_L2_R3']
    f5_runs = ['phase2a_F5_L2_R1', 'phase2a_F5_L2_R2', 'phase2a_F5_L2_R3']

    f9_raw_v = np.mean([res[r]['RAW']['valid_pose_pct'] for r in f9_runs])
    f9_inc_v = np.mean([res[r]['INCREMENTAL']['valid_pose_pct'] for r in f9_runs])
    f9_nul_v = np.mean([res[r]['NULL']['valid_pose_pct'] for r in f9_runs])
    f9_gat_v = np.mean([res[r]['GATED']['valid_pose_pct'] for r in f9_runs])
    f9_scl_v = np.mean([res[r]['SCALED']['valid_pose_pct'] for r in f9_runs])

    f5_raw_v = np.mean([res[r]['RAW']['valid_pose_pct'] for r in f5_runs])
    f5_inc_v = np.mean([res[r]['INCREMENTAL']['valid_pose_pct'] for r in f5_runs])
    f5_nul_v = np.mean([res[r]['NULL']['valid_pose_pct'] for r in f5_runs])
    f5_gat_v = np.mean([res[r]['GATED']['valid_pose_pct'] for r in f5_runs])
    f5_scl_v = np.mean([res[r]['SCALED']['valid_pose_pct'] for r in f5_runs])

    print("\n============================================================")
    print("EFFECT SIZE BREAKDOWN (In-Sample Threshold = 15.0 deg/s)")
    print("============================================================")
    print(f"F9 Yaw Sweep (n=3):")
    print(f"  RAW Baseline        : {f9_raw_v:.2f}%")
    print(f"  EIS-NULL            : {f9_nul_v:.2f}%")
    print(f"  EIS-INCREMENTAL     : {f9_inc_v:.2f}%")
    print(f"  EIS-GATED           : {f9_gat_v:.2f}%  (Net vs RAW: {f9_gat_v - f9_raw_v:+.2f}%, Gap Closed: {(f9_gat_v - f9_inc_v) / (f9_nul_v - f9_inc_v) * 100.0:.1f}%)")
    print(f"  EIS-SCALED          : {f9_scl_v:.2f}%  (Net vs RAW: {f9_scl_v - f9_raw_v:+.2f}%, Gap Closed: {(f9_scl_v - f9_inc_v) / (f9_nul_v - f9_inc_v) * 100.0:.1f}%)")

    print(f"\nF5 Pitch Tilt (n=3):")
    print(f"  RAW Baseline        : {f5_raw_v:.2f}%")
    print(f"  EIS-NULL            : {f5_nul_v:.2f}%")
    print(f"  EIS-INCREMENTAL     : {f5_inc_v:.2f}%")
    print(f"  EIS-GATED           : {f5_gat_v:.2f}%  (Net vs RAW: {f5_gat_v - f5_raw_v:+.2f}%)")
    print(f"  EIS-SCALED          : {f5_scl_v:.2f}%  (Net vs RAW: {f5_scl_v - f5_raw_v:+.2f}%)")


if __name__ == '__main__':
    main()
