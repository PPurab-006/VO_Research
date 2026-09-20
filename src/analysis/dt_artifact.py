#!/usr/bin/env python3
"""
1.7 Delayed-Triangulation Artifact Analysis.

For F6, F9, F10: per-run full-window normRPE, valid-intersection normRPE,
RPE on valid frames vs starved frames, valid_pose_pct, tracking_loss_pct,
for EIS-GATED and DELAYED-TRI.

Reuses rpe_t_norm_valid / rpe_t_norm_starved from evaluate_phase3_evo.py.

Writes results/analysis/dt_artifact.csv.
"""

import sys
from pathlib import Path

import numpy as np
import pandas as pd

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(REPO_ROOT / "src" / "pipelines"))
from evaluate_phase3_evo import evaluate_single_run

DATASETS = REPO_ROOT / "results" / "datasets"
OUTPUT = REPO_ROOT / "results" / "analysis" / "dt_artifact.csv"

FAMILIES = {
    "F6_L2": ["p3_F6_L2_R1", "p3_F6_L2_R2", "p3_F6_L2_R3"],
    "F9_L2": ["phase2a_F9_L2_R1", "phase2a_F9_L2_R2", "phase2a_F9_L2_R3"],
    "F10_L3": ["p3_F10_L3_R1", "p3_F10_L3_R2", "p3_F10_L3_R3"],
}


def main():
    rows = []

    for fam, run_dirs in FAMILIES.items():
        for run_idx, dir_name in enumerate(run_dirs, 1):
            ds_dir = DATASETS / dir_name
            gt_csv = ds_dir / "dataset_gt.csv"
            gated_csv = ds_dir / "eis_gated_vo.csv"
            dt_csv = ds_dir / "gated_dt_def_a_vo.csv"

            if not all(p.exists() for p in [gt_csv, gated_csv, dt_csv]):
                continue

            for mech_name, vo_csv in [("EIS-GATED", gated_csv), ("DELAYED-TRI", dt_csv)]:
                res = evaluate_single_run(str(vo_csv), str(gt_csv))
                if res is None:
                    continue

                rows.append({
                    "family": fam,
                    "run": run_idx,
                    "mechanism": mech_name,
                    "rpe_t_norm": res["rpe_t_norm"],
                    "rpe_t_norm_valid": res["rpe_t_norm_valid"],
                    "rpe_t_norm_starved": res["rpe_t_norm_starved"],
                    "valid_pose_pct": res["valid_pose_pct"],
                    "tracking_loss_pct": res["tracking_loss_pct"],
                    "n_frames": res["n_frames"],
                    "ate_rmse": res["ate_rmse"],
                })

    df = pd.DataFrame(rows)
    df.to_csv(OUTPUT, index=False)

    # Print summary
    print("\nDT Artifact Summary (full-window normRPE | valid-intersection normRPE | valid_pose_pct):")
    for fam in FAMILIES:
        fam_data = df[df["family"] == fam]
        for mech in ["EIS-GATED", "DELAYED-TRI"]:
            mech_data = fam_data[fam_data["mechanism"] == mech]
            if len(mech_data) == 0:
                continue
            mean_rpe = mech_data["rpe_t_norm"].mean()
            mean_rpe_valid = mech_data["rpe_t_norm_valid"].mean()
            mean_val = mech_data["valid_pose_pct"].mean()
            print(f"  {fam:10s} {mech:14s}: normRPE={mean_rpe:.4f}  "
                  f"valid-only={mean_rpe_valid:.4f}  validity={mean_val:.1f}%")

    print(f"\nWrote {len(df)} rows to {OUTPUT.relative_to(REPO_ROOT)}")


if __name__ == "__main__":
    main()
