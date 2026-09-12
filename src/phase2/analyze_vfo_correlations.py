#!/usr/bin/env python3
"""
VFO Correlation & Lead-Lag Analysis Engine (Phase 2C Step 3)

Analyzes extracted VFO variable time-series across all 7 flight datasets against
known ground-truth VO outcomes (num_inliers_pose, pose validity, EIS geometric cost).

Evaluates:
  1. Contemporaneous correlations on F9 (Yaw+Translation).
  2. Predictive lead-lag relationships (lags = 1, 2, 3, 5, 10 frames) on F9.
  3. Cross-check of estimated_rotational_flow_magnitude vs EIS geometric cost (NULL - INCREMENTAL).
  4. Sanity check of F6 (Pure Yaw, T=0) translation starvation signature.
  5. Per-dataset VFO summary statistics.
"""

import os
import sys
import numpy as np
import pandas as pd
from scipy.stats import pearsonr, spearmanr


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

    return gt_t[start_idx], gt_t[end_idx]


def compute_lead_lag_correlations(x, y, lags=[0, 1, 2, 3, 5, 10]):
    """
    Computes Pearson r and Spearman rho for predictor series X leading outcome series Y by lag k frames.
    Lag k means X[i] is correlated with Y[i + k].
    """
    results = {}
    N = len(y)
    for k in lags:
        if k == 0:
            x_s, y_s = x, y
        else:
            x_s = x[:-k]
            y_s = y[k:]

        if len(x_s) > 20:
            r, p_r = pearsonr(x_s, y_s)
            rho, p_rho = spearmanr(x_s, y_s)
            results[k] = {
                'pearson_r': float(r),
                'pearson_p': float(p_r),
                'spearman_rho': float(rho),
                'spearman_p': float(p_rho)
            }
        else:
            results[k] = {'pearson_r': 0.0, 'pearson_p': 1.0, 'spearman_rho': 0.0, 'spearman_p': 1.0}
    return results


def load_dataset_timeseries(run_id, base_dir="results/datasets"):
    dataset_dir = os.path.join(base_dir, run_id)
    gt_path = os.path.join(dataset_dir, 'dataset_gt.csv')
    raw_vo_path = os.path.join(dataset_dir, 'raw_vo.csv')
    inc_vo_path = os.path.join(dataset_dir, 'eis_inc_vo.csv')
    null_vo_path = os.path.join(dataset_dir, 'eis_null_vo.csv')
    vfo_path = os.path.join(dataset_dir, 'vfo_timeseries.csv')

    for p in [gt_path, raw_vo_path, inc_vo_path, null_vo_path, vfo_path]:
        if not os.path.exists(p):
            raise FileNotFoundError(f"Missing required dataset file: {p}")

    df_gt = pd.read_csv(gt_path)
    df_raw = pd.read_csv(raw_vo_path)
    df_inc = pd.read_csv(inc_vo_path)
    df_null = pd.read_csv(null_vo_path)
    df_vfo = pd.read_csv(vfo_path)

    # Active window
    t_start, t_end = get_canonical_active_window(df_gt)

    # Filter active window on vfo timestamps
    vfo_t = df_vfo['timestamp_total_sec'].values.astype(float)
    mask = (vfo_t >= t_start) & (vfo_t <= t_end)

    df_vfo_act = df_vfo[mask].copy()
    df_raw_act = df_raw[mask].copy()
    df_inc_act = df_inc[mask].copy()
    df_null_act = df_null[mask].copy()

    # Align rows
    min_len = min(len(df_vfo_act), len(df_raw_act), len(df_inc_act), len(df_null_act))
    df_vfo_act = df_vfo_act.iloc[:min_len].reset_index(drop=True)
    df_raw_act = df_raw_act.iloc[:min_len].reset_index(drop=True)
    df_inc_act = df_inc_act.iloc[:min_len].reset_index(drop=True)
    df_null_act = df_null_act.iloc[:min_len].reset_index(drop=True)

    return {
        'run_id': run_id,
        'vfo': df_vfo_act,
        'raw': df_raw_act,
        'inc': df_inc_act,
        'null': df_null_act,
        't_start': t_start,
        't_end': t_end
    }


def analyze_all():
    datasets = [
        'phase2a_F5_L2_R1',
        'phase2a_F5_L2_R2',
        'phase2a_F5_L2_R3',
        'phase2a_F6_L2',
        'phase2a_F9_L2_R1',
        'phase2a_F9_L2_R2',
        'phase2a_F9_L2_R3'
    ]

    loaded = {}
    for r in datasets:
        loaded[r] = load_dataset_timeseries(r)

    print("============================================================")
    print("DATASET VFO SUMMARY STATISTICS (Canonical Active Window Z >= 2.0m)")
    print("============================================================")
    
    summary_cols = [
        'feature_count', 'feature_survival_rate', 'mean_lk_err', 'feature_vel_mean',
        'flow_direction_entropy', 'flow_coherence', 'spatial_distribution_score',
        'texture_density', 'border_loss_pct', 'estimated_rotational_flow_magnitude',
        'estimated_translational_flow_magnitude', 'rotational_flow_ratio'
    ]

    summary_records = []
    for r in datasets:
        vfo_df = loaded[r]['vfo']
        row_dict = {'Dataset': r}
        for col in summary_cols:
            vals = vfo_df[col].values.astype(float)
            row_dict[f"{col}_mean"] = vals.mean()
            row_dict[f"{col}_std"] = vals.std()
        summary_records.append(row_dict)

    df_summary = pd.DataFrame(summary_records)
    
    for r in datasets:
        v = loaded[r]['vfo']
        print(f"\n--- {r} ---")
        print(f"  Features        : {v['feature_count'].mean():.1f} ± {v['feature_count'].std():.1f}")
        print(f"  Survival Rate   : {v['feature_survival_rate'].mean():.4f} ± {v['feature_survival_rate'].std():.4f}")
        print(f"  LK Error        : {v['mean_lk_err'].mean():.3f} ± {v['mean_lk_err'].std():.3f} px")
        print(f"  Feature Vel     : {v['feature_vel_mean'].mean():.2f} ± {v['feature_vel_mean'].std():.2f} px/frame")
        print(f"  Flow Coherence  : {v['flow_coherence'].mean():.4f} ± {v['flow_coherence'].std():.4f}")
        print(f"  Spatial Entropy : {v['spatial_distribution_score'].mean():.4f} ± {v['spatial_distribution_score'].std():.4f}")
        print(f"  Texture Density : {v['texture_density'].mean():.2f} ± {v['texture_density'].std():.2f}")
        print(f"  Border Loss Pct : {v['border_loss_pct'].mean():.2f}% ± {v['border_loss_pct'].std():.2f}%")
        print(f"  Rot Flow Mag    : {v['estimated_rotational_flow_magnitude'].mean():.3f} ± {v['estimated_rotational_flow_magnitude'].std():.3f} px/frame")
        print(f"  Trans Flow Mag  : {v['estimated_translational_flow_magnitude'].mean():.3f} ± {v['estimated_translational_flow_magnitude'].std():.3f} px/frame")
        print(f"  Rot Flow Ratio  : {v['rotational_flow_ratio'].mean():.4f} ± {v['rotational_flow_ratio'].std():.4f}")

    # Aggregate F9 datasets (Yaw + Translation)
    f9_vfo = pd.concat([loaded['phase2a_F9_L2_R1']['vfo'], loaded['phase2a_F9_L2_R2']['vfo'], loaded['phase2a_F9_L2_R3']['vfo']], ignore_index=True)
    f9_raw = pd.concat([loaded['phase2a_F9_L2_R1']['raw'], loaded['phase2a_F9_L2_R2']['raw'], loaded['phase2a_F9_L2_R3']['raw']], ignore_index=True)
    f9_inc = pd.concat([loaded['phase2a_F9_L2_R1']['inc'], loaded['phase2a_F9_L2_R2']['inc'], loaded['phase2a_F9_L2_R3']['inc']], ignore_index=True)
    f9_null = pd.concat([loaded['phase2a_F9_L2_R1']['null'], loaded['phase2a_F9_L2_R2']['null'], loaded['phase2a_F9_L2_R3']['null']], ignore_index=True)

    y_pose_inliers_raw = f9_raw['num_inliers_pose'].values.astype(float)
    y_pose_valid_raw = (y_pose_inliers_raw >= 8).astype(float)

    # 1. F9 Contemporaneous & Lead Correlations
    print("\n============================================================")
    print("1. F9 CONTEMPORANEOUS & LEAD CORRELATIONS (vs RAW Pose Inliers)")
    print("============================================================")

    f9_corrs = {}
    lags = [0, 1, 2, 3, 5, 10]
    for var in summary_cols:
        x = f9_vfo[var].values.astype(float)
        f9_corrs[var] = compute_lead_lag_correlations(x, y_pose_inliers_raw, lags=lags)

    # Print summary table
    print(f"{'VFO Variable':<38} | {'Lag 0 (r)':<10} | {'Lag 1 (r)':<10} | {'Lag 3 (r)':<10} | {'Lag 5 (r)':<10} | {'Peak Lag':<8}")
    print("-" * 96)

    for var in summary_cols:
        c = f9_corrs[var]
        r0 = c[0]['pearson_r']
        r1 = c[1]['pearson_r']
        r3 = c[3]['pearson_r']
        r5 = c[5]['pearson_r']

        best_k = max(lags, key=lambda k: abs(c[k]['pearson_r']))
        print(f"{var:<38} | {r0:+10.4f} | {r1:+10.4f} | {r3:+10.4f} | {r5:+10.4f} | Lag {best_k:2d}")

    # 2. Cross-check Rotational Flow vs EIS Geometric Cost (NULL - INC)
    print("\n============================================================")
    print("2. ROTATIONAL FLOW vs EIS GEOMETRIC COST CROSS-CHECK (F9)")
    print("============================================================")
    
    eis_inlier_cost = f9_null['num_inliers_pose'].values.astype(float) - f9_inc['num_inliers_pose'].values.astype(float)
    rot_flow = f9_vfo['estimated_rotational_flow_magnitude'].values.astype(float)

    r_rot_cost, _ = pearsonr(rot_flow, eis_inlier_cost)
    rho_rot_cost, _ = spearmanr(rot_flow, eis_inlier_cost)

    print(f"Correlation between estimated_rotational_flow_magnitude and (EIS_NULL - EIS_INC) pose inlier deficit:")
    print(f"  Pearson r    = {r_rot_cost:+.4f}")
    print(f"  Spearman rho = {rho_rot_cost:+.4f}")
    print("\n[NOTE]: This verifies internal math consistency across attitude telemetry (delta_omega) processing,")
    print("        NOT two independent sensor measurements.")

    # 3. F6 Pure Yaw Control Sanity Check
    print("\n============================================================")
    print("3. F6 PURE YAW CONTROL SANITY CHECK (T ≈ 0)")
    print("============================================================")

    f6_vfo = loaded['phase2a_F6_L2']['vfo']
    f6_rot = f6_vfo['estimated_rotational_flow_magnitude'].mean()
    f6_trans = f6_vfo['estimated_translational_flow_magnitude'].mean()
    f6_ratio = f6_vfo['rotational_flow_ratio'].mean()
    f6_coherence = f6_vfo['flow_coherence'].mean()
    f6_entropy = f6_vfo['flow_direction_entropy'].mean()

    print(f"F6 Mean Rotational Flow Mag    : {f6_rot:.4f} px/frame")
    print(f"F6 Mean Translational Flow Mag : {f6_trans:.4f} px/frame")
    print(f"F6 Rotational Flow Ratio       : {f6_ratio:.4f} (Dominant: {f6_ratio > 0.6})")
    print(f"F6 Flow Coherence              : {f6_coherence:.4f}")
    print(f"F6 Flow Direction Entropy      : {f6_entropy:.4f}")

    return {
        'f9_corrs': f9_corrs,
        'rot_cost_r': r_rot_cost,
        'rot_cost_rho': rho_rot_cost,
        'f6_stats': {
            'rot': f6_rot,
            'trans': f6_trans,
            'ratio': f6_ratio,
            'coherence': f6_coherence,
            'entropy': f6_entropy
        }
    }


if __name__ == '__main__':
    analyze_all()
