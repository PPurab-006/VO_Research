#!/usr/bin/env python3
"""
Master figure generation script for Phase 2 paper figures (Figs 1..12).

Executes all 12 make_fig_XX_<name>.py scripts in figures/:
- make_fig_01_essential_matrix_repair.py
- make_fig_02_motion_profiles.py
- make_fig_03_achieved_yaw.py
- make_fig_04_f9_ladder.py
- make_fig_05_threshold_sweep.py
- make_fig_06_core_matrix.py
- make_fig_07_f9_trajectories.py
- make_fig_08_dt_artifact.py
- make_fig_09_scale_artifact.py
- make_fig_10_vfo_leadlag.py
- make_fig_11_exploratory_families.py
- make_fig_12_system_diagram.py
"""

import os
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "figures"))

import make_fig_01_essential_matrix_repair
import make_fig_02_motion_profiles
import make_fig_03_achieved_yaw
import make_fig_04_f9_ladder
import make_fig_05_threshold_sweep
import make_fig_06_core_matrix
import make_fig_07_f9_trajectories
import make_fig_08_dt_artifact
import make_fig_09_scale_artifact
import make_fig_10_vfo_leadlag
import make_fig_11_exploratory_families
import make_fig_12_system_diagram

def main():
    print("========================================================")
    print("GENERATING ALL 12 SPEC-COMPLIANT PAPER FIGURES (FIGS 1..12)")
    print("========================================================")

    make_fig_01_essential_matrix_repair.main()
    make_fig_02_motion_profiles.main()
    make_fig_03_achieved_yaw.main()
    make_fig_04_f9_ladder.main()
    make_fig_05_threshold_sweep.main()
    make_fig_06_core_matrix.main()
    make_fig_07_f9_trajectories.main()
    make_fig_08_dt_artifact.main()
    make_fig_09_scale_artifact.main()
    make_fig_10_vfo_leadlag.main()
    make_fig_11_exploratory_families.main()
    make_fig_12_system_diagram.main()

    out_dir = REPO_ROOT / "figures" / "out"
    data_dir = REPO_ROOT / "figures" / "data"
    caps_dir = REPO_ROOT / "figures" / "captions"

    pngs = list(out_dir.glob("*.png"))
    pdfs = list(out_dir.glob("*.pdf"))
    csvs = list(data_dir.glob("*.csv"))
    caps = list(caps_dir.glob("*.md"))

    print(f"\n========================================================")
    print(f"SUCCESS: Generated {len(pngs)} PNGs, {len(pdfs)} PDFs, {len(csvs)} sidecar CSVs, and {len(caps)} captions.")
    print(f"========================================================")

if __name__ == "__main__":
    main()
