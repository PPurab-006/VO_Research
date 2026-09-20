#!/usr/bin/env python3
"""
1.4 Threshold Sensitivity Analysis.

(a) Bin |yaw rate| from eis_gated_vo.csv in the active window into
    [0,5),[5,10),[10,15),[15,20),[20,30),[30,50),[50,inf) and report the RAW
    pose-loss rate (num_inliers_pose<8) and frame count per bin, per F9 run
    and pooled. Also bootstrap 95% CI per bin (1000 resamples, seed=0).

(b) THRESHOLD SWEEP — handled separately if frames are available.
    This script produces yaw_rate_binned_poseloss.csv only.
    The sweep is done by threshold_sweep.py.

Writes results/analysis/yaw_rate_binned_poseloss.csv.
"""

import sys
from pathlib import Path

import numpy as np
import pandas as pd

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(REPO_ROOT / "src" / "pipelines"))
from evaluate_phase3_evo import get_canonical_active_window

DATASETS = REPO_ROOT / "results" / "datasets"
OUTPUT = REPO_ROOT / "results" / "analysis" / "yaw_rate_binned_poseloss.csv"

BIN_EDGES = [0, 5, 10, 15, 20, 30, 50, np.inf]
BIN_LABELS = ["[0,5)", "[5,10)", "[10,15)", "[15,20)", "[20,30)", "[30,50)", "[50,inf)"]


def bootstrap_ci(failures, n_total, n_boot=1000, seed=0):
    """Bootstrap 95% CI for failure rate."""
    rng = np.random.RandomState(seed)
    if n_total == 0:
        return 0.0, 0.0
    # Create binary array: 1=failure, 0=success
    arr = np.zeros(n_total, dtype=int)
    arr[:failures] = 1
    rates = np.zeros(n_boot)
    for i in range(n_boot):
        sample = rng.choice(arr, size=n_total, replace=True)
        rates[i] = sample.mean()
    return float(np.percentile(rates, 2.5)), float(np.percentile(rates, 97.5))


def main():
    all_rows = []

    for run in [1, 2, 3]:
        ds_dir = DATASETS / f"phase2a_F9_L2_R{run}"
        gt_csv = ds_dir / "dataset_gt.csv"
        raw_csv = ds_dir / "raw_vo.csv"
        gated_csv = ds_dir / "eis_gated_vo.csv"

        if not all(p.exists() for p in [gt_csv, raw_csv, gated_csv]):
            print(f"WARNING: Missing files for F9 R{run}")
            continue

        df_gt = pd.read_csv(gt_csv)
        df_raw = pd.read_csv(raw_csv)
        df_gated = pd.read_csv(gated_csv)

        t_start, t_end, _ = get_canonical_active_window(df_gt)

        # Filter to active window
        raw_mask = (df_raw["timestamp_total_sec"] >= t_start) & (df_raw["timestamp_total_sec"] <= t_end)
        df_raw_act = df_raw[raw_mask].reset_index(drop=True)

        gated_mask = (df_gated["timestamp_total_sec"] >= t_start) & (df_gated["timestamp_total_sec"] <= t_end)
        df_gated_act = df_gated[gated_mask].reset_index(drop=True)

        # Get yaw rates from gated CSV (which has eis_yaw_rate_deg)
        if "eis_yaw_rate_deg" not in df_gated_act.columns:
            print(f"WARNING: eis_yaw_rate_deg not in {gated_csv.name}")
            continue

        # Match timestamps between raw and gated to get yaw rate for raw frames
        # They should share the same timestamps
        yaw_rates = df_gated_act["eis_yaw_rate_deg"].abs().values
        raw_inliers = df_raw_act["num_inliers_pose"].values

        # They should be the same length if timestamps match
        n = min(len(yaw_rates), len(raw_inliers))
        yaw_rates = yaw_rates[:n]
        raw_inliers = raw_inliers[:n]

        raw_failure = (raw_inliers < 8).astype(int)

        bins = np.digitize(yaw_rates, BIN_EDGES) - 1  # 0-indexed

        for b_idx, label in enumerate(BIN_LABELS):
            mask = bins == b_idx
            n_frames = int(mask.sum())
            n_failures = int(raw_failure[mask].sum()) if n_frames > 0 else 0
            loss_rate = n_failures / n_frames * 100 if n_frames > 0 else 0.0

            all_rows.append({
                "run": run,
                "bin": label,
                "bin_idx": b_idx,
                "n_frames": n_frames,
                "n_failures": n_failures,
                "raw_pose_loss_pct": round(loss_rate, 2),
            })

    df = pd.DataFrame(all_rows)

    # Add pooled stats
    pooled_rows = []
    for b_idx, label in enumerate(BIN_LABELS):
        bin_data = df[df["bin_idx"] == b_idx]
        total_frames = int(bin_data["n_frames"].sum())
        total_failures = int(bin_data["n_failures"].sum())
        pooled_rate = total_failures / total_frames * 100 if total_frames > 0 else 0.0

        ci_lo, ci_hi = bootstrap_ci(total_failures, total_frames)

        pooled_rows.append({
            "run": "pooled",
            "bin": label,
            "bin_idx": b_idx,
            "n_frames": total_frames,
            "n_failures": total_failures,
            "raw_pose_loss_pct": round(pooled_rate, 2),
            "ci_95_lo": round(ci_lo * 100, 2),
            "ci_95_hi": round(ci_hi * 100, 2),
        })

    df_pooled = pd.DataFrame(pooled_rows)
    df_all = pd.concat([df, df_pooled], ignore_index=True)
    df_all.to_csv(OUTPUT, index=False)

    # Print summary
    print("\nF9 Pooled RAW Pose-Loss Rate by |Yaw Rate| Bin:")
    print(f"{'Bin':>12s} | {'Frames':>7s} | {'Failures':>8s} | {'Loss%':>7s} | {'95% CI':>15s}")
    print("-" * 60)
    for _, r in df_pooled.iterrows():
        ci = f"[{r.get('ci_95_lo', '?')}, {r.get('ci_95_hi', '?')}]"
        print(f"{r['bin']:>12s} | {r['n_frames']:>7d} | {r['n_failures']:>8d} | {r['raw_pose_loss_pct']:>6.2f}% | {ci:>15s}")

    print(f"\nWrote {len(df_all)} rows to {OUTPUT.relative_to(REPO_ROOT)}")


if __name__ == "__main__":
    main()
