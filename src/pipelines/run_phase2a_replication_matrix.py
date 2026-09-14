#!/usr/bin/env python3
"""
Phase 2A Replication Matrix Automation & Statistical Engine

Executes the Phase 2A replication experiment:
- Family A: F5 (Pitch + Translation L2) — 3 independent repeats (F5_L2_R1, F5_L2_R2, F5_L2_R3)
- Family B: F9 (Yaw + Translation L2) — 3 independent repeats (F9_L2_R1, F9_L2_R2, F9_L2_R3)

For each run:
1. Records raw 1280x960 camera frames & synchronized ~50 Hz attitude.
2. Runs unchanged RAW Monocular VO.
3. Runs unchanged EIS-Derotated Monocular VO.
4. Computes canonical active-window metrics, per-run deltas (EIS - RAW), and cross-repeat statistics.
"""

import argparse
import csv
import math
import os
import sys
import time
import subprocess
import cv2
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation as R_scipy
from scipy.interpolate import interp1d

from record_single_phase2a_dataset import record_dataset, kill_all_sim_processes
from run_offline_vo import OfflineVOProcessor
from eis_derotation import EISDerotator


def get_canonical_active_window(df_gt):
    t_col = 'timestamp_total_sec' if 'timestamp_total_sec' in df_gt.columns else 'timestamp'
    gt_t = df_gt[t_col].values.astype(float)
    if gt_t[0] > 1e12:
        gt_t = gt_t / 1e9

    z_gt = df_gt['pos_z'].values.astype(float) if 'pos_z' in df_gt.columns else df_gt['z'].values.astype(float)

    # Cruise altitude Z >= 2.0m window
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


def analyze_single_run(dataset_dir, raw_csv, eis_csv, gt_csv):
    df_gt = pd.read_csv(gt_csv)
    df_raw = pd.read_csv(raw_csv)
    df_eis = pd.read_csv(eis_csv)

    t_start, t_end, dur = get_canonical_active_window(df_gt)

    gt_t = df_gt['timestamp_total_sec'].values.astype(float)
    raw_t = df_raw['timestamp_total_sec'].values.astype(float)
    eis_t = df_eis['timestamp_total_sec'].values.astype(float)

    raw_mask = (raw_t >= t_start) & (raw_t <= t_end)
    eis_mask = (eis_t >= t_start) & (eis_t <= t_end)

    df_raw_act = df_raw[raw_mask].copy()
    df_eis_act = df_eis[eis_mask].copy()

    # Calculate metrics over active window
    def calc_metrics(df_act, df_all_t):
        if len(df_act) == 0:
            return {}
        n_E = df_act['num_inliers_E'].values.astype(float)
        n_pose = df_act['num_inliers_pose'].values.astype(float)
        matched = df_act['num_matched'].values.astype(float)

        valid_pose_count = int(np.sum(n_pose >= 8))
        valid_pose_pct = (valid_pose_count / len(df_act)) * 100.0

        pose_e_ratio = np.where(n_E > 0, n_pose / n_E, 0.0)
        e_ratio = np.where(matched > 0, n_E / matched, 0.0)
        survival = df_act['feature_survival_rate'].astype(float).values
        fvel = df_act['feature_vel_mean'].astype(float).values
        lk = df_act['mean_lk_err'].astype(float).values

        pos = df_act[['pos_x', 'pos_y', 'pos_z']].values.astype(float)
        unit_dist = np.sum(np.linalg.norm(np.diff(pos, axis=0), axis=1)) if len(pos) > 1 else 0.0

        return {
            'n_frames': len(df_act),
            'valid_pose_pct': valid_pose_pct,
            'n_E_mean': float(n_E.mean()),
            'n_E_median': float(np.median(n_E)),
            'n_pose_mean': float(n_pose.mean()),
            'n_pose_median': float(np.median(n_pose)),
            'pose_e_ratio': float(pose_e_ratio.mean()),
            'e_ratio': float(e_ratio.mean()),
            'survival': float(survival.mean()),
            'fvel_mean': float(fvel.mean()),
            'fvel_p95': float(np.percentile(fvel, 95)) if len(fvel) > 0 else 0.0,
            'lk_mean': float(lk.mean()),
            'lk_median': float(np.median(lk)),
            'unit_dist': float(unit_dist)
        }

    m_raw = calc_metrics(df_raw_act, raw_t)
    m_eis = calc_metrics(df_eis_act, eis_t)

    # Compute GT rotation error
    fqx = interp1d(gt_t, df_gt['rot_x'].values, bounds_error=False, fill_value='extrapolate')
    fqy = interp1d(gt_t, df_gt['rot_y'].values, bounds_error=False, fill_value='extrapolate')
    fqz = interp1d(gt_t, df_gt['rot_z'].values, bounds_error=False, fill_value='extrapolate')
    fqw = interp1d(gt_t, df_gt['rot_w'].values, bounds_error=False, fill_value='extrapolate')

    if len(df_raw_act) > 1:
        q_raw = np.column_stack([fqx(df_raw_act['timestamp_total_sec'].values.astype(float)),
                                 fqy(df_raw_act['timestamp_total_sec'].values.astype(float)),
                                 fqz(df_raw_act['timestamp_total_sec'].values.astype(float)),
                                 fqw(df_raw_act['timestamp_total_sec'].values.astype(float))])
        q_norm = q_raw / np.linalg.norm(q_raw, axis=1, keepdims=True)
        r_gt_interp = R_scipy.from_quat(q_norm)
        rel_r_gt = r_gt_interp[:-1].inv() * r_gt_interp[1:]
        rot_gt_deg = rel_r_gt.magnitude() * (180.0 / np.pi)

        raw_rot_deg = df_raw_act['rel_rot_deg'].astype(float).values[1:]
        eis_rot_deg = df_eis_act['rel_rot_deg'].astype(float).values[1:]

        m_raw['rot_err_mean'] = float(np.mean(np.abs(raw_rot_deg - rot_gt_deg)))
        m_eis['rot_err_mean'] = float(np.mean(np.abs(eis_rot_deg - rot_gt_deg)))
    else:
        m_raw['rot_err_mean'] = 0.0
        m_eis['rot_err_mean'] = 0.0

    # Calculate Deltas (EIS - RAW)
    delta = {
        'delta_valid_pose_pct': m_eis['valid_pose_pct'] - m_raw['valid_pose_pct'],
        'delta_pose_e_ratio': m_eis['pose_e_ratio'] - m_raw['pose_e_ratio'],
        'delta_survival': m_eis['survival'] - m_raw['survival'],
        'delta_lk_mean': m_eis['lk_mean'] - m_raw['lk_mean'],       # Negative is improvement
        'delta_fvel_mean': m_eis['fvel_mean'] - m_raw['fvel_mean'],   # Negative is improvement
        'delta_n_pose_mean': m_eis['n_pose_mean'] - m_raw['n_pose_mean'],
        'delta_rot_err_mean': m_eis['rot_err_mean'] - m_raw['rot_err_mean']
    }

    return m_raw, m_eis, delta, dur


def run_matrix():
    runs_spec = [
        # Family A: F5 (Pitch + Translation L2) - 3 repeats
        ('F5', 2, 'phase2a_F5_L2_R1'),
        ('F5', 2, 'phase2a_F5_L2_R2'),
        ('F5', 2, 'phase2a_F5_L2_R3'),
        # Family B: F9 (Yaw + Translation L2) - 3 repeats
        ('F9', 2, 'phase2a_F9_L2_R1'),
        ('F9', 2, 'phase2a_F9_L2_R2'),
        ('F9', 2, 'phase2a_F9_L2_R3')
    ]

    all_results = []

    for family, severity, run_id in runs_spec:
        dataset_dir = f"results/datasets/{run_id}"
        gt_csv = os.path.join(dataset_dir, 'dataset_gt.csv')
        cam_csv = os.path.join(dataset_dir, 'camera_frames.csv')
        raw_csv = os.path.join(dataset_dir, 'raw_vo.csv')
        eis_csv = os.path.join(dataset_dir, 'eis_vo.csv')

        print(f"\n==========================================================================")
        print(f"PROCESSING RUN: {run_id} ({family} L{severity})")
        print(f"==========================================================================")

        # 1. Record dataset if not present
        if not (os.path.exists(gt_csv) and os.path.exists(cam_csv)):
            print(f"[RECORDING DATASET] Launching flight recording for '{run_id}'...")
            record_dataset(family=family, severity=severity, duration=20.0, run_id=run_id)
        else:
            print(f"[DATASET FOUND] Existing dataset found at '{dataset_dir}'")

        # 2. Run RAW VO if not present
        if not os.path.exists(raw_csv):
            print(f"[RUNNING RAW VO] Processing raw frames for '{run_id}'...")
            proc_raw = OfflineVOProcessor(output_csv_path=raw_csv, mode='klt', eis_derotator=None)
            df_cam = pd.read_csv(cam_csv)
            for _, row in df_cam.iterrows():
                img_path = os.path.join(dataset_dir, 'images', row['filename'])
                cv_img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
                proc_raw.process_frame(cv_img, int(row['timestamp_sec']), int(row['timestamp_nanosec']), float(row['timestamp_total_sec']))
            proc_raw.save_csv()
        else:
            print(f"[RAW VO FOUND] '{raw_csv}' already processed.")

        # 3. Run EIS VO if not present
        if not os.path.exists(eis_csv):
            print(f"[RUNNING EIS VO] Processing derotated frames for '{run_id}'...")
            eis_derotator = EISDerotator()
            eis_derotator.load_attitude_telemetry(gt_csv)
            proc_eis = OfflineVOProcessor(output_csv_path=eis_csv, mode='klt', eis_derotator=eis_derotator)
            df_cam = pd.read_csv(cam_csv)
            for _, row in df_cam.iterrows():
                img_path = os.path.join(dataset_dir, 'images', row['filename'])
                cv_img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
                proc_eis.process_frame(cv_img, int(row['timestamp_sec']), int(row['timestamp_nanosec']), float(row['timestamp_total_sec']))
            proc_eis.save_csv()
        else:
            print(f"[EIS VO FOUND] '{eis_csv}' already processed.")

        # 4. Analyze A/B metrics
        m_raw, m_eis, delta, active_dur = analyze_single_run(dataset_dir, raw_csv, eis_csv, gt_csv)
        res_entry = {
            'run_id': run_id,
            'family': family,
            'severity': severity,
            'active_dur': active_dur,
            'raw': m_raw,
            'eis': m_eis,
            'delta': delta
        }
        all_results.append(res_entry)

    return all_results


def print_statistical_summary(results):
    print("\n" + "=" * 110)
    print("PHASE 2A REPLICATION STUDY STATISTICAL SUMMARY")
    print("=" * 110)

    families = ['F5', 'F9']
    metric_keys = [
        ('valid_pose_pct', 'Pose Validity Rate (%)', True),
        ('pose_e_ratio', 'Pose/E Agreement Ratio', True),
        ('survival', 'Feature Survival Rate', True),
        ('lk_mean', 'Mean LK Residual (px)', False),
        ('fvel_mean', 'Feature Velocity (px/fr)', False),
        ('n_pose_mean', 'Mean Pose Inliers N_pose', True)
    ]

    for fam in families:
        fam_runs = [r for r in results if r['family'] == fam]
        print(f"\n>>> FAMILY {fam} REPLICATION STATISTICAL ANALYSIS (n={len(fam_runs)} independent runs) <<<")
        print("-" * 110)
        print(f"{'Metric':<30} | {'RAW Mean (Std)':<18} | {'EIS Mean (Std)':<18} | {'Mean Delta (Std)':<18} | {'Consistency':<12}")
        print("-" * 110)

        for key, name, higher_is_better in metric_keys:
            raw_vals = [r['raw'][key] for r in fam_runs]
            eis_vals = [r['eis'][key] for r in fam_runs]
            deltas = [r['delta']['delta_' + key] for r in fam_runs]

            mean_raw, std_raw = np.mean(raw_vals), np.std(raw_vals)
            mean_eis, std_eis = np.mean(eis_vals), np.std(eis_vals)
            mean_d, std_d = np.mean(deltas), np.std(deltas)

            if higher_is_better:
                improved_count = sum(1 for d in deltas if d > 0)
            else:
                improved_count = sum(1 for d in deltas if d < 0)

            consistency_str = f"{improved_count}/{len(fam_runs)} runs"

            raw_str = f"{mean_raw:.3f} ({std_raw:.3f})"
            eis_str = f"{mean_eis:.3f} ({std_eis:.3f})"
            d_str = f"{mean_d:+.3f} ({std_d:.3f})"

            print(f"{name:<30} | {raw_str:<18} | {eis_str:<18} | {d_str:<18} | {consistency_str:<12}")

        print("-" * 110)


if __name__ == '__main__':
    results = run_matrix()
    print_statistical_summary(results)
