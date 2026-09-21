#!/usr/bin/env python3
"""
GT Heading Telemetry Audit (All 41 Datasets).

Audits ground-truth telemetry across all 41 datasets in results/datasets/:
- n_gt_rows: Total ground-truth rows in dataset_gt.csv
- n_duplicate_timestamps: Number of duplicate timestamp_total_sec entries
- frac_consecutive_rows_lt_5ms: Fraction of consecutive GT rows with dt < 5ms
- n_heading_jumps_gt10deg: Number of consecutive heading jumps > 10 degrees
- max_heading_jump_deg: Maximum heading jump between consecutive GT rows (deg)
- n_rows_z_below_2_inside_window: Number of GT rows with pos_z < 2.0 between the first and last row with pos_z >= 2.0 (after dropping duplicate timestamps)

Heading = atan2(y, x) of GT body +X axis (unwrapped) over active window (first to last row with pos_z >= 2.0 after dropping duplicate timestamps).

Writes results/analysis/gt_heading_audit.csv.
"""

import sys
from pathlib import Path
import numpy as np
import pandas as pd

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(REPO_ROOT / "figures"))
import common as c

DATASETS = REPO_ROOT / "results" / "datasets"
OUTPUT_CSV = REPO_ROOT / "results" / "analysis" / "gt_heading_audit.csv"


def main():
    dirs = sorted([d.name for d in DATASETS.iterdir() if d.is_dir()])
    records = []
    jumps_core_expl = {}

    for dname in dirs:
        dpath = DATASETS / dname
        gt_csv = dpath / "dataset_gt.csv"
        if not gt_csv.exists():
            continue

        gt = pd.read_csv(gt_csv)
        n_gt_rows = len(gt)
        t_col = "timestamp_total_sec" if "timestamp_total_sec" in gt.columns else "timestamp"
        ts_raw = gt[t_col].values.astype(float)
        n_dups = int(len(ts_raw) - len(np.unique(ts_raw)))

        t, hd, a = c.heading_series(gt)
        n_bad, mj, _ = c.glitch_report(t, hd)
        n_below2 = int((a["pos_z"] < 2.0).sum())

        if len(t) > 1:
            dts = np.diff(t)
            frac_lt_5ms = float(np.mean(dts < 0.005))
        else:
            frac_lt_5ms = 0.0

        records.append({
            "dataset_dir": dname,
            "n_gt_rows": n_gt_rows,
            "n_duplicate_timestamps": n_dups,
            "frac_consecutive_rows_lt_5ms": float(frac_lt_5ms),
            "n_heading_jumps_gt10deg": n_bad,
            "max_heading_jump_deg": float(mj),
            "n_rows_z_below_2_inside_window": n_below2,
        })

        # Track 36 core + exploratory runs
        is_core_or_expl = any(
            dname == c.run_dir(fam, r, expl).name
            for fam in c.CORE
            for r in c.RUNS
            for expl in [False]
        ) or any(
            dname == c.run_dir(fam, r, expl).name
            for fam in c.EXPLORATORY
            for r in c.RUNS
            for expl in [True]
        )

        if is_core_or_expl and n_bad > 0:
            jumps_core_expl[dname] = n_bad

    df = pd.DataFrame(records)
    OUTPUT_CSV.parent.mkdir(parents=True, exist_ok=True)
    df.to_csv(OUTPUT_CSV, index=False)

    print("========================================================")
    print("GT HEADING AUDIT SUMMARY (36 CORE + EXPLORATORY RUNS)")
    print("========================================================")
    print("Runs with GT heading jumps > 10 deg:")
    for k, v in jumps_core_expl.items():
        print(f"  - {k}: {v} jumps")

    expected_jumps = {"p3_F6_L2_R1": 4, "p3_F6_L2_R3": 4, "p3_F10_L3_R2": 11}
    assert jumps_core_expl == expected_jumps, f"Jumps mismatch! Found: {jumps_core_expl}, Expected: {expected_jumps}"
    print("ASSERTION PASSED: Exactly p3_F6_L2_R1 (4), p3_F6_L2_R3 (4), and p3_F10_L3_R2 (11) have heading jumps > 10 deg.")
    print(f"Wrote {len(df)} rows to {OUTPUT_CSV.relative_to(REPO_ROOT)}")


if __name__ == "__main__":
    main()
