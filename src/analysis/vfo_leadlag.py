#!/usr/bin/env python3
"""
1.6 VFO Lead-Lag Analysis.

Reproduces the lead-lag correlation table on F9 (3 runs pooled):
12 predictors from vfo_timeseries.csv x lags {0,1,2,3,5,10}, Pearson r between predictor
at t and RAW num_inliers_pose at t+lag, active window only.

Also reports: target saturation (fraction of F9 frames with num_inliers_pose >= 8),
and the same correlation table using a BINARY target (num_inliers_pose < 8, i.e. VO failure).

Writes results/analysis/vfo_leadlag_f9.csv.
"""

import sys
from pathlib import Path

import numpy as np
import pandas as pd

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(REPO_ROOT / "src" / "pipelines"))
from evaluate_phase3_evo import get_canonical_active_window

DATASETS = REPO_ROOT / "results" / "datasets"
OUTPUT = REPO_ROOT / "results" / "analysis" / "vfo_leadlag_f9.csv"

LAGS = [0, 1, 2, 3, 5, 10]

# 12 predictors from vfo_timeseries.csv header
VFO_PREDICTORS = [
    "feature_count",
    "feature_vel_mean",
    "texture_density",
    "mean_lk_err",
    "flow_coherence",
    "spatial_distribution_score",
    "flow_direction_entropy",
    "rotational_flow_ratio",
    "feature_survival_rate",
    "border_loss_pct",
    "estimated_rotational_flow_magnitude",
    "estimated_translational_flow_magnitude",
]


def main():
    runs_data = []

    for run in [1, 2, 3]:
        ds_dir = DATASETS / f"phase2a_F9_L2_R{run}"
        vfo_csv = ds_dir / "vfo_timeseries.csv"
        raw_csv = ds_dir / "raw_vo.csv"
        gt_csv = ds_dir / "dataset_gt.csv"

        assert vfo_csv.exists(), f"Missing {vfo_csv}"
        assert raw_csv.exists(), f"Missing {raw_csv}"
        assert gt_csv.exists(), f"Missing {gt_csv}"

        df_vfo = pd.read_csv(vfo_csv)
        df_raw = pd.read_csv(raw_csv)
        df_gt = pd.read_csv(gt_csv)

        t_start, t_end, _ = get_canonical_active_window(df_gt)

        vfo_mask = (df_vfo["timestamp_total_sec"] >= t_start) & (df_vfo["timestamp_total_sec"] <= t_end)
        raw_mask = (df_raw["timestamp_total_sec"] >= t_start) & (df_raw["timestamp_total_sec"] <= t_end)

        df_vfo_act = df_vfo[vfo_mask].reset_index(drop=True)
        df_raw_act = df_raw[raw_mask].reset_index(drop=True)

        n = min(len(df_vfo_act), len(df_raw_act))
        df_vfo_act = df_vfo_act.iloc[:n]
        df_raw_act = df_raw_act.iloc[:n]

        # Hard assertion 1: every requested predictor exists
        for pred in VFO_PREDICTORS:
            assert pred in df_vfo_act.columns, f"Missing predictor '{pred}' in {vfo_csv}"

        runs_data.append((df_vfo_act, df_raw_act))

    # Pool data across runs and check variance
    pooled_vfo = pd.concat([d[0] for d in runs_data], ignore_index=True)
    pooled_raw = pd.concat([d[1] for d in runs_data], ignore_index=True)

    constant_predictors = []
    # Hard assertion 2: check non-zero variance in active window
    for pred in VFO_PREDICTORS:
        var_val = pooled_vfo[pred].var()
        if var_val == 0 or np.isnan(var_val):
            constant_predictors.append(pred)
        assert var_val > 0, f"Predictor '{pred}' has zero variance in active window!"

    # Target saturation
    total_frames = len(pooled_raw)
    valid_frames = (pooled_raw["num_inliers_pose"] >= 8).sum()
    sat_frac = valid_frames / total_frames if total_frames > 0 else 0.0

    print(f"Target saturation: {sat_frac:.4f} ({valid_frames}/{total_frames} frames with inliers >= 8)")

    rows = []
    # Compute lead-lag correlations
    for pred in VFO_PREDICTORS:
        is_const = pred in constant_predictors
        for lag in LAGS:
            if is_const:
                r_cont = np.nan
                r_bin = np.nan
            else:
                x_list, y_cont_list, y_bin_list = [], [], []
                for vfo_act, raw_act in runs_data:
                    inliers = raw_act["num_inliers_pose"].values
                    binary_fail = (inliers < 8).astype(float)
                    vals = vfo_act[pred].values

                    if lag == 0:
                        x_list.append(vals)
                        y_cont_list.append(inliers)
                        y_bin_list.append(binary_fail)
                    else:
                        x_list.append(vals[:-lag])
                        y_cont_list.append(inliers[lag:])
                        y_bin_list.append(binary_fail[lag:])

                x_all = np.concatenate(x_list)
                y_cont_all = np.concatenate(y_cont_list)
                y_bin_all = np.concatenate(y_bin_list)

                r_cont = float(np.corrcoef(x_all, y_cont_all)[0, 1])
                r_bin = float(np.corrcoef(x_all, y_bin_all)[0, 1])

            # Check hard assertion 3: no output cell is exactly 0.0000 unless input is constant
            if not is_const:
                if round(r_cont, 4) == 0.0:
                    assert False, f"Unexpected exact 0.0000 for continuous correlation of {pred} at lag {lag}"
                if round(r_bin, 4) == 0.0:
                    assert False, f"Unexpected exact 0.0000 for binary correlation of {pred} at lag {lag}"

            rows.append({
                "predictor": pred,
                "lag": lag,
                "r_continuous": round(r_cont, 4) if not np.isnan(r_cont) else np.nan,
                "r_binary": round(r_bin, 4) if not np.isnan(r_bin) else np.nan,
                "n_samples": len(x_all) if not is_const else 0,
            })

    df_out = pd.DataFrame(rows)
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    df_out.to_csv(OUTPUT, index=False)

    # Print full tables
    print("\n========================================================")
    print("VFO LEAD-LAG CORRELATION TABLE: CONTINUOUS TARGET (num_inliers_pose)")
    print("========================================================")
    piv_cont = df_out.pivot(index="predictor", columns="lag", values="r_continuous")
    print(piv_cont.to_string())

    print("\n========================================================")
    print("VFO LEAD-LAG CORRELATION TABLE: BINARY TARGET (num_inliers_pose < 8)")
    print("========================================================")
    piv_bin = df_out.pivot(index="predictor", columns="lag", values="r_binary")
    print(piv_bin.to_string())

    if constant_predictors:
        print(f"\nConstant predictors (written as NaN): {constant_predictors}")
    else:
        print("\nConstant predictors: None")

    print(f"\nWrote results to {OUTPUT.relative_to(REPO_ROOT)}")


if __name__ == "__main__":
    main()
