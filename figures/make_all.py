#!/usr/bin/env python3
"""Regenerate every paper figure from results/datasets/ (no simulator, GPU, ROS or camera frames needed).
Fig 5 panels (b)/(c) additionally need results/analysis/threshold_sweep.csv (produced by the frame-replay sweep, `make rerun-vo`).
"""
import subprocess, sys, time
from pathlib import Path
HERE = Path(__file__).parent
SCRIPTS = ["make_fig_01_recoverpose.py", "make_fig_02_motion_families.py", "make_fig_03_achieved_yaw.py", "make_fig_04_f9_ladder.py",
           "make_fig_05_threshold.py", "make_fig_06_core_matrix.py", "make_fig_07_f9_trajectories.py", "make_fig_08_dt_artifact.py",
           "make_fig_09_scale_artifact.py", "make_fig_10_vfo_leadlag.py", "make_fig_11_exploratory.py", "make_fig_12_system_diagram.py"]
(HERE / "data" / "_per_run_metrics.csv").unlink(missing_ok=True)      # force recomputation with the repo evaluation code
bad = []
for s in SCRIPTS:
    t = time.time(); r = subprocess.run([sys.executable, str(HERE / s)], capture_output=True, text=True)
    print(f"{'OK ' if r.returncode == 0 else 'FAIL'} {s:38s} {time.time()-t:5.1f}s")
    if r.returncode: bad.append(s); print(r.stderr[-800:])
sys.exit(1 if bad else 0)
