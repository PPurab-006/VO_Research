#!/usr/bin/env python3
"""
1.1 Build Per-Run Metrics Table.

One row per (family, run, mechanism) for core and exploratory families.
Reuses evaluate_dataset_mechanisms() from run_phase3_full_eval.py.

Writes:
  results/analysis/per_run_metrics.csv         (8 core families)
  results/analysis/per_run_metrics_exploratory.csv  (4 exploratory families)
"""

import os
import sys
from pathlib import Path

import pandas as pd

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(REPO_ROOT / "src" / "pipelines"))
from evaluate_phase3_evo import evaluate_single_run, get_canonical_active_window

DATASETS = REPO_ROOT / "results" / "datasets"
OUTPUT_CORE = REPO_ROOT / "results" / "analysis" / "per_run_metrics.csv"
OUTPUT_EXPL = REPO_ROOT / "results" / "analysis" / "per_run_metrics_exploratory.csv"

CORE_FAMILIES = ["F1_L2", "F2_L2", "F4_L2", "F5_L2", "F6_L2", "F9_L2", "F10_L3", "F11_L2"]
EXPLORATORY_FAMILIES = ["HOVER_L0", "F3_L2", "F7_L2", "F8_L2"]

MECHANISMS = {
    "RAW": "raw_vo.csv",
    "EIS-GATED": "eis_gated_vo.csv",
    "DELAYED-TRI": "gated_dt_def_a_vo.csv",
}


def find_run_dir(family, run, is_exploratory):
    """Find the dataset directory for a given family and run."""
    if is_exploratory:
        prefixes = ["p3x_"]
    else:
        prefixes = ["p3_", "phase2a_"]

    for prefix in prefixes:
        d = DATASETS / f"{prefix}{family}_R{run}"
        if d.exists() and (d / "dataset_gt.csv").exists():
            return d, prefix.rstrip("_")
    return None, None


def compute_run_metrics(ds_dir, gt_csv, vo_csv, mechanism):
    """Compute metrics for a single run/mechanism."""
    res = evaluate_single_run(str(vo_csv), str(gt_csv))
    if res is None:
        return None

    df_gt = pd.read_csv(gt_csv)
    t_start, t_end, dur = get_canonical_active_window(df_gt)

    return {
        "ate_rmse": res["ate_rmse"],
        "rpe_t_m": res["rpe_t_mean"],
        "rpe_t_norm": res["rpe_t_norm"],
        "rpe_t_norm_valid": res.get("rpe_t_norm_valid", 0.0),
        "rpe_t_norm_starved": res.get("rpe_t_norm_starved", 0.0),
        "valid_pose_pct": res["valid_pose_pct"],
        "tracking_loss_pct": res["tracking_loss_pct"],
        "n_active_frames": res["n_frames"],
        "sim3_scale": res["sim3_scale"],
    }


def build_table(families, is_exploratory):
    rows = []
    for fam in families:
        for run in [1, 2, 3]:
            ds_dir, source = find_run_dir(fam, run, is_exploratory)
            if ds_dir is None:
                continue

            gt_csv = ds_dir / "dataset_gt.csv"

            for mech_name, vo_filename in MECHANISMS.items():
                vo_csv = ds_dir / vo_filename
                if not vo_csv.exists():
                    continue

                metrics = compute_run_metrics(ds_dir, gt_csv, vo_csv, mech_name)
                if metrics is None:
                    continue

                row = {
                    "family": fam,
                    "run": run,
                    "mechanism": mech_name,
                    "source_dir": source,
                    **metrics,
                }
                rows.append(row)

    return pd.DataFrame(rows)


def main():
    print("Building per-run metrics table...")
    df_core = build_table(CORE_FAMILIES, is_exploratory=False)
    df_core.to_csv(OUTPUT_CORE, index=False)
    print(f"  Core: {len(df_core)} rows -> {OUTPUT_CORE.relative_to(REPO_ROOT)}")

    df_expl = build_table(EXPLORATORY_FAMILIES, is_exploratory=True)
    df_expl.to_csv(OUTPUT_EXPL, index=False)
    print(f"  Exploratory: {len(df_expl)} rows -> {OUTPUT_EXPL.relative_to(REPO_ROOT)}")


if __name__ == "__main__":
    main()
