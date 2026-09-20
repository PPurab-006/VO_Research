#!/usr/bin/env python3
"""
1.2 Paired Statistics (GATED vs RAW).

For each core family: paired difference GATED-RAW for ATE, rpe_t_norm, valid_pose_pct.
Reports mean diff, per-run diffs, number of runs where GATED is better,
paired t-test p and Wilcoxon p.

Writes results/analysis/paired_stats.csv.
"""

import sys
from pathlib import Path

import numpy as np
import pandas as pd
from scipy import stats

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
INPUT = REPO_ROOT / "results" / "analysis" / "per_run_metrics.csv"
OUTPUT = REPO_ROOT / "results" / "analysis" / "paired_stats.csv"


def paired_test(raw_vals, gated_vals):
    """Compute paired t-test and Wilcoxon signed-rank test."""
    diffs = np.array(gated_vals) - np.array(raw_vals)
    n = len(diffs)

    if n < 2:
        return {"mean_diff": float(np.mean(diffs)), "t_p": np.nan, "wilcoxon_p": np.nan}

    # Paired t-test
    t_stat, t_p = stats.ttest_rel(gated_vals, raw_vals)

    # Wilcoxon signed-rank (needs n >= 5 for reliability, but we compute it for n=3)
    try:
        w_stat, w_p = stats.wilcoxon(diffs, alternative="two-sided")
    except ValueError:
        w_p = np.nan

    return {
        "mean_diff": float(np.mean(diffs)),
        "per_run_diffs": diffs.tolist(),
        "t_stat": float(t_stat),
        "t_p": float(t_p),
        "wilcoxon_p": float(w_p) if not np.isnan(w_p) else np.nan,
    }


def main():
    df = pd.read_csv(INPUT)

    rows = []
    for fam in df["family"].unique():
        df_fam = df[df["family"] == fam]
        raw = df_fam[df_fam["mechanism"] == "RAW"].sort_values("run")
        gated = df_fam[df_fam["mechanism"] == "EIS-GATED"].sort_values("run")

        if len(raw) == 0 or len(gated) == 0:
            continue

        # Merge on run
        merged = raw.merge(gated, on="run", suffixes=("_raw", "_gated"))

        for metric in ["ate_rmse", "rpe_t_norm", "valid_pose_pct"]:
            raw_vals = merged[f"{metric}_raw"].values
            gated_vals = merged[f"{metric}_gated"].values
            n = len(raw_vals)

            result = paired_test(raw_vals.tolist(), gated_vals.tolist())

            # For ATE and RPE, "better" means GATED < RAW (lower is better)
            # For valid_pose_pct, "better" means GATED > RAW (higher is better)
            if metric == "valid_pose_pct":
                n_better = int(np.sum(gated_vals > raw_vals))
            else:
                n_better = int(np.sum(gated_vals < raw_vals))

            rows.append({
                "family": fam,
                "metric": metric,
                "n_runs": n,
                "mean_raw": float(np.mean(raw_vals)),
                "mean_gated": float(np.mean(gated_vals)),
                "mean_diff": result["mean_diff"],
                "per_run_diffs": str(result.get("per_run_diffs", [])),
                "n_gated_better": n_better,
                "paired_t_p": result["t_p"],
                "wilcoxon_p": result["wilcoxon_p"],
            })

            direction = "better" if n_better > n // 2 else "worse"
            print(f"  {fam} {metric}: EIS-GATED {direction} in {n_better}/{n} runs; "
                  f"mean diff={result['mean_diff']:.4f}; t-test p={result['t_p']:.4f}")

    df_out = pd.DataFrame(rows)
    df_out.to_csv(OUTPUT, index=False)
    print(f"\nWrote {len(df_out)} rows to {OUTPUT.relative_to(REPO_ROOT)}")


if __name__ == "__main__":
    main()
