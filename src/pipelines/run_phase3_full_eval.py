#!/usr/bin/env python3
"""
Phase 3 Full Batch Evaluation Engine (11 Families x 3 Repeats x 3 Mechanisms)

Computes full ATE/RPE and telemetry metrics across active motion window (Z >= 2.0m):
  - Sim(3) Umeyama Scale-Aligned Trajectory Alignment
  - ATE RMSE (meters)
  - Translation RPE (m/step) [standard absolute meter metric]
  - Translation RPE (scale-normalized / unit-scale) [isolates intrinsic tracking error]
  - Rotation RPE (deg/step)
  - Tracking Loss Rate (%)
  - Recovery Time (seconds & frames)
  - Trajectory Drift per Meter Traveled (m/m)
  - Prediction Lead Time (reported as "N/A -- falsified in Phase 2C (VFO)")

Aggregates statistics across n=3 repeats per cell reporting Mean +/- Std and 95% Student-t CIs.
"""

import os
import sys
import math
import numpy as np
import pandas as pd
from scipy.stats import t as student_t

from evaluate_phase3_evo import evaluate_single_run


def compute_stats_with_ci(values):
    vals = np.array(values, dtype=float)
    n = len(vals)
    mean_val = float(np.mean(vals))
    std_val = float(np.std(vals, ddof=1)) if n > 1 else 0.0

    if n > 1 and std_val > 1e-9:
        t_crit = float(student_t.ppf(0.975, df=n - 1))
        sem = std_val / math.sqrt(n)
        margin = t_crit * sem
        ci_low = mean_val - margin
        ci_high = mean_val + margin
    else:
        margin = 0.0
        ci_low = mean_val
        ci_high = mean_val

    return {
        'n': n,
        'mean': mean_val,
        'std': std_val,
        'margin': margin,
        'ci_low': ci_low,
        'ci_high': ci_high,
        'str_mean_std': f"{mean_val:.4f} +/- {std_val:.4f}",
        'str_ci': f"[{ci_low:.4f}, {ci_high:.4f}]"
    }


def evaluate_dataset_mechanisms(dataset_dir, gt_csv_path):
    import hashlib

    mechs = {
        'RAW': os.path.join(dataset_dir, 'raw_vo.csv'),
        'EIS-GATED': os.path.join(dataset_dir, 'eis_gated_vo.csv'),
        'DELAYED-TRI': os.path.join(dataset_dir, 'gated_dt_def_a_vo.csv')
    }

    # Assert source file paths are 3 distinct non-identical paths
    paths = list(mechs.values())
    if len(set(paths)) != 3:
        raise ValueError(f"Source file paths must be 3 distinct paths, got: {paths}")

    res_dict = {}
    for mech_name, vo_csv in mechs.items():
        if os.path.exists(vo_csv) and os.path.exists(gt_csv_path):
            eval_res = evaluate_single_run(vo_csv, gt_csv_path)
            res_dict[mech_name] = eval_res
        else:
            res_dict[mech_name] = None

    # Verification of md5sums & R-frame feature deferrals
    raw_p = mechs['RAW']
    gated_p = mechs['EIS-GATED']
    dt_p = mechs['DELAYED-TRI']

    if os.path.exists(raw_p) and os.path.exists(gated_p) and os.path.exists(dt_p):
        h_raw = hashlib.md5(open(raw_p, 'rb').read()).hexdigest()
        h_gated = hashlib.md5(open(gated_p, 'rb').read()).hexdigest()
        h_dt = hashlib.md5(open(dt_p, 'rb').read()).hexdigest()

        df_gated = pd.read_csv(gated_p)
        df_dt = pd.read_csv(dt_p)

        n_r_frames = (df_gated['eis_yaw_rate_deg'] > 15.0).sum() if 'eis_yaw_rate_deg' in df_gated.columns else 0
        n_pending = df_dt['num_pending'].sum() if 'num_pending' in df_dt.columns else 0

        ds_name = os.path.basename(dataset_dir)
        print(f"  [VERIFY {ds_name}] R-frames (>15 deg/s): {n_r_frames}, Deferred Pending Sum: {n_pending}")
        print(f"    RAW MD5    : {h_raw}")
        print(f"    GATED MD5  : {h_gated}")
        print(f"    DT MD5     : {h_dt}")

        if n_r_frames > 0 and n_pending > 0:
            if h_gated == h_dt:
                raise RuntimeError(f"CRITICAL BUG in {ds_name}: DELAYED-TRI output is byte-identical to EIS-GATED despite {n_r_frames} R-frames and {n_pending} deferred pending features!")
            else:
                print(f"    [PASS] Confirmed distinct DELAYED-TRI output for rotation dataset {ds_name}")
        elif n_r_frames == 0:
            print(f"    [INFO] 0 R-frames detected in {ds_name} -> Baseline-identical DELAYED-TRI expected by design (dormant control cell)")

    return res_dict


def run_full_matrix_evaluation(dataset_root="results/datasets"):
    print("==========================================================================")
    print("[INFO] Phase 3 Batch Matrix Evaluation Engine")

    core_families = ['F1_L2', 'F2_L2', 'F4_L2', 'F5_L2', 'F6_L2', 'F9_L2', 'F10_L3', 'F11_L2']
    exploratory_families = ['HOVER_L0', 'F3_L2', 'F7_L2', 'F8_L2']
    mechs = ['RAW', 'EIS-GATED', 'DELAYED-TRI']

    def eval_family_group(fam_list, is_exploratory=False):
        group_results = {}
        summary_rows = []

        track_label = "Exploratory Matrix" if is_exploratory else "Core Matrix"
        print(f"\n[INFO] Evaluating {track_label}:")
        print(f"{'Family':<12} | {'Mechanism':<12} | {'Evaluated Runs':<14} | {'ATE RMSE (m)':<22} | {'RPE-t (m/step)':<22} | {'RPE-t (scale-norm)':<24}")
        print("-" * 105)

        for fam in fam_list:
            group_results[fam] = {m: [] for m in mechs}

            for r in [1, 2, 3]:
                if is_exploratory:
                    possible_dirs = [
                        os.path.join(dataset_root, f"p3x_{fam}_R{r}"),
                        os.path.join(dataset_root, f"p3x_{fam}") if r == 1 else None
                    ]
                else:
                    possible_dirs = [
                        os.path.join(dataset_root, f"p3_{fam}_R{r}"),
                        os.path.join(dataset_root, f"phase2a_{fam}_R{r}"),
                        os.path.join(dataset_root, f"{fam}_R{r}"),
                        os.path.join(dataset_root, f"phase2a_{fam}") if r == 1 else None
                    ]

                ds_dir = None
                for d in possible_dirs:
                    if d and os.path.exists(os.path.join(d, "dataset_gt.csv")):
                        ds_dir = d
                        break

                if not ds_dir:
                    continue

                gt_csv = os.path.join(ds_dir, "dataset_gt.csv")
                run_eval = evaluate_dataset_mechanisms(ds_dir, gt_csv)

                for m in mechs:
                    if run_eval[m] is not None:
                        group_results[fam][m].append(run_eval[m])

            for m in mechs:
                runs = group_results[fam][m]
                n_runs = len(runs)
                if n_runs > 0:
                    ates = [r['ate_rmse'] for r in runs]
                    rpes_m = [r['rpe_t_mean'] for r in runs]
                    rpes_norm = [r['rpe_t_norm'] for r in runs]

                    s_ate = compute_stats_with_ci(ates)
                    s_rpe_m = compute_stats_with_ci(rpes_m)
                    s_rpe_norm = compute_stats_with_ci(rpes_norm)

                    print(f"{fam:<12} | {m:<12} | {n_runs:<14} | {s_ate['str_mean_std']:<22} | {s_rpe_m['str_mean_std']:<22} | {s_rpe_norm['str_mean_std']:<24}")

                    summary_rows.append({
                        'track': 'exploratory' if is_exploratory else 'core',
                        'family': fam,
                        'mechanism': m,
                        'n_runs': n_runs,
                        'ate_mean': s_ate['mean'],
                        'ate_std': s_ate['std'],
                        'ate_ci': s_ate['str_ci'],
                        'rpe_m_mean': s_rpe_m['mean'],
                        'rpe_m_std': s_rpe_m['std'],
                        'rpe_m_ci': s_rpe_m['str_ci'],
                        'rpe_norm_mean': s_rpe_norm['mean'],
                        'rpe_norm_std': s_rpe_norm['std'],
                        'rpe_norm_ci': s_rpe_norm['str_ci']
                    })
                else:
                    print(f"{fam:<12} | {m:<12} | {0:<14} | {'N/A (Missing Data)':<22} | {'N/A':<22} | {'N/A':<24}")

        return group_results, pd.DataFrame(summary_rows)

    core_res, df_core = eval_family_group(core_families, is_exploratory=False)
    expl_res, df_expl = eval_family_group(exploratory_families, is_exploratory=True)

    return (core_res, expl_res), pd.concat([df_core, df_expl], ignore_index=True)


if __name__ == '__main__':
    run_full_matrix_evaluation()
