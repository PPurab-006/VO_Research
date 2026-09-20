#!/usr/bin/env python3
"""
1.3 F9 EIS Effect Ladder.

For F9 R1..R3 (phase2a_F9_L2_R*): valid_pose_pct for 5 conditions:
  RAW, EIS_FIXED, EIS_INCREMENTAL, EIS_NULL, EIS_GATED.

Expected means: 93.04, 62.51, 88.17, 93.28, 92.14.

Writes results/analysis/f9_ladder.csv.
"""

import sys
from pathlib import Path

import numpy as np
import pandas as pd

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(REPO_ROOT / "src" / "pipelines"))
from evaluate_phase3_evo import evaluate_single_run

DATASETS = REPO_ROOT / "results" / "datasets"
OUTPUT = REPO_ROOT / "results" / "analysis" / "f9_ladder.csv"

CONDITIONS = {
    "RAW": "raw_vo.csv",
    "EIS_FIXED": "eis_vo.csv",
    "EIS_INCREMENTAL": "eis_inc_vo.csv",
    "EIS_NULL": "eis_null_vo.csv",
    "EIS_GATED": "eis_gated_vo.csv",
}


def main():
    rows = []
    for run in [1, 2, 3]:
        ds_dir = DATASETS / f"phase2a_F9_L2_R{run}"
        gt_csv = ds_dir / "dataset_gt.csv"

        if not gt_csv.exists():
            print(f"WARNING: {ds_dir.name} not found")
            continue

        for cond_name, vo_file in CONDITIONS.items():
            vo_csv = ds_dir / vo_file
            if not vo_csv.exists():
                print(f"  {cond_name}: {vo_file} not found in {ds_dir.name}")
                continue

            res = evaluate_single_run(str(vo_csv), str(gt_csv))
            if res is None:
                continue

            rows.append({
                "family": "F9_L2",
                "run": run,
                "condition": cond_name,
                "valid_pose_pct": res["valid_pose_pct"],
                "ate_rmse": res["ate_rmse"],
                "rpe_t_norm": res["rpe_t_norm"],
                "tracking_loss_pct": res["tracking_loss_pct"],
                "n_active_frames": res["n_frames"],
                "sim3_scale": res["sim3_scale"],
            })

    df = pd.DataFrame(rows)
    df.to_csv(OUTPUT, index=False)

    # Print summary: mean per condition
    print("\nF9 Effect Ladder (valid_pose_pct means):")
    for cond in CONDITIONS:
        vals = df[df["condition"] == cond]["valid_pose_pct"].values
        if len(vals) > 0:
            print(f"  {cond:20s}: {np.mean(vals):.2f}  (per run: {', '.join(f'{v:.2f}' for v in vals)})")

    print(f"\nWrote {len(df)} rows to {OUTPUT.relative_to(REPO_ROOT)}")


if __name__ == "__main__":
    main()
