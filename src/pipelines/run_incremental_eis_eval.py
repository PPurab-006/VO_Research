#!/usr/bin/env python3
"""
Phase 2A Four-Way Evaluation Runner (RAW / EIS-FIXED / EIS-INCREMENTAL / EIS-NULL)

Processes saved camera datasets offline over the canonical Z >= 2.0m active window.
Verifies internal consistency across runs and evaluates the Null-Warp control.
"""

import os
import sys
import csv
import math
import cv2
import numpy as np
import pandas as pd

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
    cum_warp_deg = df_act['eis_cum_warp_deg'].astype(float).values if 'eis_cum_warp_deg' in df_act.columns else np.zeros(len(df_act))

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
        'cum_warp_deg_mean': float(cum_warp_deg.mean())
    }


def run_evaluation():
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

        print(f"\n==========================================================================")
        print(f"PROCESSING EVALUATION FOR: {run_id}")
        print(f"==========================================================================")

        df_cam = pd.read_csv(cam_csv)
        df_gt = pd.read_csv(gt_csv)
        t_start, t_end, active_dur = get_canonical_active_window(df_gt)

        if not os.path.exists(raw_csv):
            proc_raw = OfflineVOProcessor(output_csv_path=raw_csv, mode='klt', eis_derotator=None)
            for _, row in df_cam.iterrows():
                img_path = os.path.join(dataset_dir, 'images', row['filename'])
                cv_img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
                proc_raw.process_frame(cv_img, int(row['timestamp_sec']), int(row['timestamp_nanosec']), float(row['timestamp_total_sec']))
            proc_raw.save_csv()

        if not os.path.exists(eis_fixed_csv):
            eis_fixed = EISDerotator(reference_mode='fixed')
            eis_fixed.load_attitude_telemetry(gt_csv)
            proc_fixed = OfflineVOProcessor(output_csv_path=eis_fixed_csv, mode='klt', eis_derotator=eis_fixed)
            for _, row in df_cam.iterrows():
                img_path = os.path.join(dataset_dir, 'images', row['filename'])
                cv_img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
                proc_fixed.process_frame(cv_img, int(row['timestamp_sec']), int(row['timestamp_nanosec']), float(row['timestamp_total_sec']))
            proc_fixed.save_csv()

        if not os.path.exists(eis_inc_csv):
            eis_inc = EISDerotator(reference_mode='incremental')
            eis_inc.load_attitude_telemetry(gt_csv)
            proc_inc = OfflineVOProcessor(output_csv_path=eis_inc_csv, mode='klt', eis_derotator=eis_inc)
            for _, row in df_cam.iterrows():
                img_path = os.path.join(dataset_dir, 'images', row['filename'])
                cv_img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
                proc_inc.process_frame(cv_img, int(row['timestamp_sec']), int(row['timestamp_nanosec']), float(row['timestamp_total_sec']))
            proc_inc.save_csv()

        if not os.path.exists(eis_null_csv):
            eis_null = EISDerotator(reference_mode='null')
            eis_null.load_attitude_telemetry(gt_csv)
            proc_null = OfflineVOProcessor(output_csv_path=eis_null_csv, mode='klt', eis_derotator=eis_null)
            for _, row in df_cam.iterrows():
                img_path = os.path.join(dataset_dir, 'images', row['filename'])
                cv_img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
                proc_null.process_frame(cv_img, int(row['timestamp_sec']), int(row['timestamp_nanosec']), float(row['timestamp_total_sec']))
            proc_null.save_csv()

        df_raw = pd.read_csv(raw_csv)
        df_fixed = pd.read_csv(eis_fixed_csv)
        df_inc = pd.read_csv(eis_inc_csv)
        df_null = pd.read_csv(eis_null_csv)

        m_raw = analyze_vo_csv(df_raw, t_start, t_end)
        m_fixed = analyze_vo_csv(df_fixed, t_start, t_end)
        m_inc = analyze_vo_csv(df_inc, t_start, t_end)
        m_null = analyze_vo_csv(df_null, t_start, t_end)

        results[run_id] = {
            'active_dur': active_dur,
            'RAW': m_raw,
            'FIXED': m_fixed,
            'INCREMENTAL': m_inc,
            'NULL': m_null
        }

    return results


def print_four_way_summary(results):
    print("\n" + "=" * 145)
    print("PHASE 2A FOUR-WAY COMPARISON (RAW vs EIS-FIXED vs EIS-INCREMENTAL vs EIS-NULL)")
    print("Over Canonical Active Window Z >= 2.0m")
    print("=" * 145)

    headers = [
        ("Dataset", 17),
        ("RAW Val%", 9),
        ("FIX Val%", 9),
        ("INC Val%", 9),
        ("NUL Val%", 9),
        ("RAW P/E", 8),
        ("FIX P/E", 8),
        ("INC P/E", 8),
        ("NUL P/E", 8),
        ("RAW Surv", 8),
        ("FIX Surv", 8),
        ("INC Surv", 8),
        ("NUL Surv", 8),
        ("INC Crop%", 9)
    ]

    header_str = " | ".join([f"{h[0]:^{h[1]}}" for h in headers])
    print(header_str)
    print("-" * len(header_str))

    for run_id, res in results.items():
        raw = res['RAW']
        fix = res['FIXED']
        inc = res['INCREMENTAL']
        nul = res['NULL']

        row_str = " | ".join([
            f"{run_id:<17}",
            f"{raw['valid_pose_pct']:9.2f}",
            f"{fix['valid_pose_pct']:9.2f}",
            f"{inc['valid_pose_pct']:9.2f}",
            f"{nul['valid_pose_pct']:9.2f}",
            f"{raw['pose_e_ratio']:8.3f}",
            f"{fix['pose_e_ratio']:8.3f}",
            f"{inc['pose_e_ratio']:8.3f}",
            f"{nul['pose_e_ratio']:8.3f}",
            f"{raw['survival']:8.3f}",
            f"{fix['survival']:8.3f}",
            f"{inc['survival']:8.3f}",
            f"{nul['survival']:8.3f}",
            f"{inc['crop_pct_mean']:9.2f}"
        ])
        print(row_str)

    print("-" * len(header_str))


if __name__ == '__main__':
    results = run_evaluation()
    print_four_way_summary(results)

