#!/usr/bin/env python3
"""
1.10 Dataset Provenance Table.

For each run directory under results/datasets/:
  - family, generation (phase2a / p3 / p3x / phase2b), run number
  - number of VO frames (raw_vo.csv rows), GT rows, files present
  - md5 of raw_vo.csv, active window duration

Writes results/analysis/dataset_provenance.csv.
"""

import hashlib
import sys
from pathlib import Path

import numpy as np
import pandas as pd

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(REPO_ROOT / "src" / "pipelines"))
from evaluate_phase3_evo import get_canonical_active_window

DATASETS = REPO_ROOT / "results" / "datasets"
OUTPUT = REPO_ROOT / "results" / "analysis" / "dataset_provenance.csv"

CORE_FAMILIES = ["F1_L2", "F2_L2", "F4_L2", "F5_L2", "F6_L2", "F9_L2", "F10_L3", "F11_L2"]
EXPLORATORY_FAMILIES = ["HOVER_L0", "F3_L2", "F7_L2", "F8_L2"]

VO_FILES = [
    "raw_vo.csv", "eis_gated_vo.csv", "gated_dt_def_a_vo.csv",
    "eis_vo.csv", "eis_inc_vo.csv", "eis_null_vo.csv", "eis_scaled_vo.csv",
    "gated_dt_def_b_vo.csv", "raw_dt_def_a_vo.csv", "raw_dt_def_b_vo.csv",
    "vfo_timeseries.csv", "camera_frames.csv", "dataset_gt.csv",
]


def md5_file(path):
    return hashlib.md5(path.read_bytes()).hexdigest()


def parse_dir_name(name):
    """Parse directory name into (generation, family, run)."""
    for prefix in ("p3x_", "p3_", "phase2a_", "phase2b_"):
        if name.startswith(prefix):
            rest = name[len(prefix):]
            gen = prefix.rstrip("_")
            # Try to split off _R<n>
            if "_R" in rest:
                parts = rest.rsplit("_R", 1)
                fam = parts[0]
                run = int(parts[1])
            else:
                fam = rest
                run = 0  # singleton (e.g. phase2a_F5_L2, phase2a_F6_L2)
            return gen, fam, run
    return "unknown", name, 0


def main():
    rows = []
    for d in sorted(DATASETS.iterdir()):
        if not d.is_dir():
            continue
        gen, fam, run = parse_dir_name(d.name)
        gt_csv = d / "dataset_gt.csv"
        raw_csv = d / "raw_vo.csv"

        n_gt_rows = 0
        active_dur = 0.0
        if gt_csv.exists():
            df_gt = pd.read_csv(gt_csv)
            n_gt_rows = len(df_gt)
            _, _, active_dur = get_canonical_active_window(df_gt)

        n_vo_frames = 0
        raw_md5 = ""
        if raw_csv.exists():
            df_vo = pd.read_csv(raw_csv)
            n_vo_frames = len(df_vo)
            raw_md5 = md5_file(raw_csv)

        files_present = [f for f in VO_FILES if (d / f).exists()]

        is_core = fam in CORE_FAMILIES
        is_exploratory = fam in EXPLORATORY_FAMILIES

        rows.append({
            "run_dir": d.name,
            "generation": gen,
            "family": fam,
            "run": run,
            "track": "core" if is_core else ("exploratory" if is_exploratory else "other"),
            "n_vo_frames": n_vo_frames,
            "n_gt_rows": n_gt_rows,
            "active_window_sec": round(active_dur, 2),
            "raw_vo_md5": raw_md5,
            "files_present": ";".join(files_present),
            "n_files": len(files_present),
        })

    df = pd.DataFrame(rows)
    df.to_csv(OUTPUT, index=False)
    print(f"Wrote {len(df)} rows to {OUTPUT.relative_to(REPO_ROOT)}")

    # Warn about core cells not from p3_
    core_rows = df[df["track"] == "core"]
    non_p3_core = core_rows[core_rows["generation"] != "p3"]
    if len(non_p3_core) > 0:
        print("\nWARNING: Core cells NOT from p3_ generation:")
        for _, r in non_p3_core.iterrows():
            print(f"  {r['run_dir']} ({r['generation']})")


if __name__ == "__main__":
    main()
