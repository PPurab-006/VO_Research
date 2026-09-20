#!/usr/bin/env python3
"""
1.8 Delayed-Triangulation Dormancy Check.

For every evaluated dataset run: count of R-frames (|yaw rate| > 15 deg/s) and whether
gated_dt_def_a_vo.csv is byte-identical (md5) to eis_gated_vo.csv.

Scope: 37 dataset directories analyzed out of 41 total directories in results/datasets/.
The 4 excluded directories (phase2a_F5_L2, phase2b_candidate4_L2_R1, R2, R3) are legacy/candidate
directories that do not contain gated_dt_def_a_vo.csv.

Writes results/analysis/dt_dormancy.csv.
"""

import hashlib
from pathlib import Path

import numpy as np
import pandas as pd

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
DATASETS = REPO_ROOT / "results" / "datasets"
OUTPUT = REPO_ROOT / "results" / "analysis" / "dt_dormancy.csv"


def md5_file(path):
    return hashlib.md5(path.read_bytes()).hexdigest()


def main():
    rows = []
    excluded_dirs = []

    for d in sorted(DATASETS.iterdir()):
        if not d.is_dir():
            continue

        gated_csv = d / "eis_gated_vo.csv"
        dt_csv = d / "gated_dt_def_a_vo.csv"

        if not gated_csv.exists() or not dt_csv.exists():
            excluded_dirs.append(d.name)
            continue

        md5_gated = md5_file(gated_csv)
        md5_dt = md5_file(dt_csv)
        byte_identical = md5_gated == md5_dt

        # Count R-frames
        df_gated = pd.read_csv(gated_csv)
        n_r_frames = 0
        if "eis_yaw_rate_deg" in df_gated.columns:
            n_r_frames = int((df_gated["eis_yaw_rate_deg"].abs() > 15.0).sum())

        rows.append({
            "run_dir": d.name,
            "n_r_frames": n_r_frames,
            "md5_gated": md5_gated,
            "md5_dt": md5_dt,
            "byte_identical": byte_identical,
            "dormant": byte_identical and n_r_frames == 0,
            "notes": "37 evaluated matrix runs (24 core + 12 exploratory + 1 F9 legacy)",
        })

    df = pd.DataFrame(rows)
    df.to_csv(OUTPUT, index=False)

    n_dormant = df["dormant"].sum()
    n_active = (~df["byte_identical"]).sum()

    print(f"========================================================")
    print(f"DELAYED TRIANGULATION DORMANCY AUDIT ({len(df)} runs analyzed)")
    print(f"========================================================")
    print(f"Total directories in results/datasets/: {len(list(DATASETS.iterdir()))}")
    print(f"Evaluated directories                 : {len(df)}")
    print(f"Excluded non-matrix/candidate dirs    : {len(excluded_dirs)} ({excluded_dirs})")
    print(f"Dormant cells (DT is a no-op)          : {n_dormant}")
    print(f"Active cells (DT differs from GATED)   : {n_active}")

    print(f"\nDormant cells:")
    for _, r in df[df["dormant"]].iterrows():
        print(f"  {r['run_dir']}")
    print(f"\nActive cells:")
    for _, r in df[~df["byte_identical"]].iterrows():
        print(f"  {r['run_dir']} ({r['n_r_frames']} R-frames)")

    print(f"\nWrote {len(df)} rows to {OUTPUT.relative_to(REPO_ROOT)}")


if __name__ == "__main__":
    main()
