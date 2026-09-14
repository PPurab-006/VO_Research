#!/usr/bin/env python3
"""
Phase 2D Evaluation Runner: Delayed Triangulation (RD-VIO-Inspired)

Evaluates Monocular VO on 7 Datasets across Canonical Active Window (Z >= 2.0m):
  - F5_L2_R1, F5_L2_R2, F5_L2_R3 (Translation-dominant)
  - F6_L2 (Pure yaw rotation)
  - F9_L2_R1, F9_L2_R2, F9_L2_R3 (Combined translation + fast yaw)

Configurations Tested:
  1. RAW
  2. RAW + Delayed-Triangulation (Def a: |omega_z| > 15 deg/s)
  3. EIS-GATED
  4. EIS-GATED + Delayed-Triangulation (Def a: |omega_z| > 15 deg/s)

Comparison Variant:
  5. RAW + Delayed-Triangulation (Def b: Baseline/Depth < 0.005)
  6. EIS-GATED + Delayed-Triangulation (Def b: Baseline/Depth < 0.005)
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


def analyze_vo_csv(df_vo, t_start, t_end, promotion_latencies=None):
    vo_t = df_vo['timestamp_total_sec'].values.astype(float)
    mask = (vo_t >= t_start) & (vo_t <= t_end)
    df_act = df_vo[mask].copy()

    if len(df_act) == 0:
        return {}

    n_E = df_act['num_inliers_E'].values.astype(float)
    n_pose = df_act['num_inliers_pose'].values.astype(float)

    valid_pose_count = int(np.sum(n_pose >= 8))
    valid_pose_pct = (valid_pose_count / len(df_act)) * 100.0

    pose_e_ratio = np.where(n_E > 0, n_pose / n_E, 0.0)
    survival = df_act['feature_survival_rate'].astype(float).values
    lk = df_act['mean_lk_err'].astype(float).values

    is_r_frame = df_act['is_r_frame'].astype(int).values if 'is_r_frame' in df_act.columns else np.zeros(len(df_act))
    r_frame_pct = (np.sum(is_r_frame > 0) / len(df_act)) * 100.0

    num_active = df_act['num_active'].astype(float).values if 'num_active' in df_act.columns else df_act['num_matched'].astype(float).values
    num_pending = df_act['num_pending'].astype(float).values if 'num_pending' in df_act.columns else np.zeros(len(df_act))

    mean_promo_latency = float(np.mean(promotion_latencies)) if (promotion_latencies is not None and len(promotion_latencies) > 0) else 0.0

    return {
        'n_frames': len(df_act),
        'valid_pose_pct': valid_pose_pct,
        'n_E_mean': float(n_E.mean()),
        'n_pose_mean': float(n_pose.mean()),
        'pose_e_ratio': float(pose_e_ratio.mean()),
        'survival': float(survival.mean()),
        'lk_mean': float(lk.mean()),
        'r_frame_pct': r_frame_pct,
        'pending_mean': float(num_pending.mean()),
        'active_mean': float(num_active.mean()),
        'mean_promo_latency': mean_promo_latency
    }


def run_phase2d_experiments():
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

        df_cam = pd.read_csv(cam_csv)
        df_gt = pd.read_csv(gt_csv)
        t_start, t_end, active_dur = get_canonical_active_window(df_gt)

        # Output paths
        raw_csv = os.path.join(dataset_dir, 'raw_vo.csv')
        raw_dt_a_csv = os.path.join(dataset_dir, 'raw_dt_def_a_vo.csv')
        raw_dt_b_csv = os.path.join(dataset_dir, 'raw_dt_def_b_vo.csv')

        eis_gated_csv = os.path.join(dataset_dir, 'eis_gated_vo.csv')
        gated_dt_a_csv = os.path.join(dataset_dir, 'gated_dt_def_a_vo.csv')
        gated_dt_b_csv = os.path.join(dataset_dir, 'gated_dt_def_b_vo.csv')

        proc_raw = OfflineVOProcessor(output_csv_path=raw_csv, mode='klt', eis_derotator=None)
        proc_raw.load_telemetry(gt_csv)
        for _, row in df_cam.iterrows():
            img_path = os.path.join(dataset_dir, 'images', row['filename'])
            cv_img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
            proc_raw.process_frame(cv_img, int(row['timestamp_sec']), int(row['timestamp_nanosec']), float(row['timestamp_total_sec']))
        proc_raw.save_csv()
        m_raw = analyze_vo_csv(pd.read_csv(raw_csv), t_start, t_end, proc_raw.promotion_latencies)

        proc_raw_dt_a = OfflineVOProcessor(
            output_csv_path=raw_dt_a_csv, mode='klt', eis_derotator=None,
            delayed_triangulation=True, r_frame_def='yaw_rate', gate_thresh_deg=15.0, min_non_r_obs=3
        )
        proc_raw_dt_a.load_telemetry(gt_csv)
        for _, row in df_cam.iterrows():
            img_path = os.path.join(dataset_dir, 'images', row['filename'])
            cv_img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
            proc_raw_dt_a.process_frame(cv_img, int(row['timestamp_sec']), int(row['timestamp_nanosec']), float(row['timestamp_total_sec']))
        proc_raw_dt_a.save_csv()
        m_raw_dt_a = analyze_vo_csv(pd.read_csv(raw_dt_a_csv), t_start, t_end, proc_raw_dt_a.promotion_latencies)

        proc_raw_dt_b = OfflineVOProcessor(
            output_csv_path=raw_dt_b_csv, mode='klt', eis_derotator=None,
            delayed_triangulation=True, r_frame_def='baseline_depth', baseline_depth_thresh=0.005, min_non_r_obs=3
        )
        proc_raw_dt_b.load_telemetry(gt_csv)
        for _, row in df_cam.iterrows():
            img_path = os.path.join(dataset_dir, 'images', row['filename'])
            cv_img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
            proc_raw_dt_b.process_frame(cv_img, int(row['timestamp_sec']), int(row['timestamp_nanosec']), float(row['timestamp_total_sec']))
        proc_raw_dt_b.save_csv()
        m_raw_dt_b = analyze_vo_csv(pd.read_csv(raw_dt_b_csv), t_start, t_end, proc_raw_dt_b.promotion_latencies)

        eis_gated = EISDerotator(reference_mode='gated')
        eis_gated.load_attitude_telemetry(gt_csv)
        proc_gated = OfflineVOProcessor(output_csv_path=eis_gated_csv, mode='klt', eis_derotator=eis_gated, gate_thresh_deg=15.0)
        proc_gated.load_telemetry(gt_csv)
        for _, row in df_cam.iterrows():
            img_path = os.path.join(dataset_dir, 'images', row['filename'])
            cv_img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
            proc_gated.process_frame(cv_img, int(row['timestamp_sec']), int(row['timestamp_nanosec']), float(row['timestamp_total_sec']))
        proc_gated.save_csv()
        m_gated = analyze_vo_csv(pd.read_csv(eis_gated_csv), t_start, t_end, proc_gated.promotion_latencies)

        eis_gated_a = EISDerotator(reference_mode='gated')
        eis_gated_a.load_attitude_telemetry(gt_csv)
        proc_gated_dt_a = OfflineVOProcessor(
            output_csv_path=gated_dt_a_csv, mode='klt', eis_derotator=eis_gated_a, gate_thresh_deg=15.0,
            delayed_triangulation=True, r_frame_def='yaw_rate', min_non_r_obs=3
        )
        proc_gated_dt_a.load_telemetry(gt_csv)
        for _, row in df_cam.iterrows():
            img_path = os.path.join(dataset_dir, 'images', row['filename'])
            cv_img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
            proc_gated_dt_a.process_frame(cv_img, int(row['timestamp_sec']), int(row['timestamp_nanosec']), float(row['timestamp_total_sec']))
        proc_gated_dt_a.save_csv()
        m_gated_dt_a = analyze_vo_csv(pd.read_csv(gated_dt_a_csv), t_start, t_end, proc_gated_dt_a.promotion_latencies)

        eis_gated_b = EISDerotator(reference_mode='gated')
        eis_gated_b.load_attitude_telemetry(gt_csv)
        proc_gated_dt_b = OfflineVOProcessor(
            output_csv_path=gated_dt_b_csv, mode='klt', eis_derotator=eis_gated_b, gate_thresh_deg=15.0,
            delayed_triangulation=True, r_frame_def='baseline_depth', baseline_depth_thresh=0.005, min_non_r_obs=3
        )
        proc_gated_dt_b.load_telemetry(gt_csv)
        for _, row in df_cam.iterrows():
            img_path = os.path.join(dataset_dir, 'images', row['filename'])
            cv_img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
            proc_gated_dt_b.process_frame(cv_img, int(row['timestamp_sec']), int(row['timestamp_nanosec']), float(row['timestamp_total_sec']))
        proc_gated_dt_b.save_csv()
        m_gated_dt_b = analyze_vo_csv(pd.read_csv(gated_dt_b_csv), t_start, t_end, proc_gated_dt_b.promotion_latencies)

        results[run_id] = {
            'RAW': m_raw,
            'RAW_DT_A': m_raw_dt_a,
            'RAW_DT_B': m_raw_dt_b,
            'GATED': m_gated,
            'GATED_DT_A': m_gated_dt_a,
            'GATED_DT_B': m_gated_dt_b,
        }

    return results


def main():
    print("==========================================================================")
    print("RUNNING PHASE 2D EVALUATION: DELAYED TRIANGULATION (RD-VIO-INSPIRED)")
    print("==========================================================================")

    res = run_phase2d_experiments()

    print("\n----------------------------------------------------------------------------------------------------------------------------------")
    print(f"{'Dataset Run':<18} | {'RAW Val%':<8} | {'RAW+DT_a':<8} | {'RAW+DT_b':<8} | {'GATED':<8} | {'GAT+DT_a':<8} | {'GAT+DT_b':<8} | {'R-Frac%':<7} | {'Pend_a':<6} | {'PromoLat':<8}")
    print("----------------------------------------------------------------------------------------------------------------------------------")

    for run_id, m in res.items():
        r_v = m['RAW']['valid_pose_pct']
        ra_v = m['RAW_DT_A']['valid_pose_pct']
        rb_v = m['RAW_DT_B']['valid_pose_pct']
        g_v = m['GATED']['valid_pose_pct']
        ga_v = m['GATED_DT_A']['valid_pose_pct']
        gb_v = m['GATED_DT_B']['valid_pose_pct']

        r_frac = m['RAW_DT_A']['r_frame_pct']
        pend_a = m['RAW_DT_A']['pending_mean']
        lat_a = m['RAW_DT_A']['mean_promo_latency']

        print(f"{run_id:<18} | {r_v:7.2f}% | {ra_v:7.2f}% | {rb_v:7.2f}% | {g_v:7.2f}% | {ga_v:7.2f}% | {gb_v:7.2f}% | {r_frac:6.1f}% | {pend_a:6.1f} | {lat_a:7.1f}f")

    # Grouped aggregates
    f5_runs = ['phase2a_F5_L2_R1', 'phase2a_F5_L2_R2', 'phase2a_F5_L2_R3']
    f9_runs = ['phase2a_F9_L2_R1', 'phase2a_F9_L2_R2', 'phase2a_F9_L2_R3']
    f6_runs = ['phase2a_F6_L2']

    def print_group_summary(group_name, run_keys):
        raw_vals = [res[k]['RAW']['valid_pose_pct'] for k in run_keys]
        raw_dt_a_vals = [res[k]['RAW_DT_A']['valid_pose_pct'] for k in run_keys]
        gated_vals = [res[k]['GATED']['valid_pose_pct'] for k in run_keys]
        gated_dt_a_vals = [res[k]['GATED_DT_A']['valid_pose_pct'] for k in run_keys]

        raw_pe = [res[k]['RAW']['pose_e_ratio'] for k in run_keys]
        raw_dt_a_pe = [res[k]['RAW_DT_A']['pose_e_ratio'] for k in run_keys]
        gated_pe = [res[k]['GATED']['pose_e_ratio'] for k in run_keys]
        gated_dt_a_pe = [res[k]['GATED_DT_A']['pose_e_ratio'] for k in run_keys]

        print(f"\n=== SUMMARY FOR {group_name} ===")
        print(f"  RAW Valid Pose %:                   {np.mean(raw_vals):.2f}%")
        print(f"  RAW + Delayed-Triangulation (Def a): {np.mean(raw_dt_a_vals):.2f}% (Delta: {np.mean(raw_dt_a_vals)-np.mean(raw_vals):+.2f}%)")
        print(f"  EIS-GATED Valid Pose %:             {np.mean(gated_vals):.2f}%")
        print(f"  EIS-GATED + Delayed-Triang (Def a):  {np.mean(gated_dt_a_vals):.2f}% (Delta: {np.mean(gated_dt_a_vals)-np.mean(gated_vals):+.2f}%)")
        print(f"  RAW Pose/E Ratio:                   {np.mean(raw_pe):.4f}")
        print(f"  RAW + DT Pose/E Ratio:              {np.mean(raw_dt_a_pe):.4f}")
        print(f"  EIS-GATED Pose/E Ratio:             {np.mean(gated_pe):.4f}")
        print(f"  EIS-GATED + DT Pose/E Ratio:        {np.mean(gated_dt_a_pe):.4f}")

    print_group_summary("F5 (Translation Baseline)", f5_runs)
    print_group_summary("F6 (Pure Yaw Rotation)", f6_runs)
    print_group_summary("F9 (Translation + Fast Yaw)", f9_runs)


if __name__ == '__main__':
    main()
