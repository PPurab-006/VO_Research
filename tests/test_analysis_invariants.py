#!/usr/bin/env python3
"""
Test Analysis Invariants (Zenodo Hygiene & Statistical Integrity).
"""

import os
import subprocess
import unittest
from pathlib import Path
import numpy as np
import pandas as pd

REPO_ROOT = Path(__file__).resolve().parent.parent


class TestAnalysisInvariants(unittest.TestCase):

    def test_f9_ate_means_and_std(self):
        """F9 EIS-GATED/RAW ATE means 3.0235 / 3.3206 (1e-3) and std(ddof=1) 0.3281 / 0.5420."""
        df_core = pd.read_csv(REPO_ROOT / "figures" / "data" / "fig_06_core_matrix.csv")
        row = df_core[(df_core["family"] == "F9_L2") & (df_core["metric"] == "ate_rmse")].iloc[0]
        
        raw_pts = [row["raw_r1"], row["raw_r2"], row["raw_r3"]]
        gated_pts = [row["gated_r1"], row["gated_r2"], row["gated_r3"]]

        f9_ate_raw_mean, raw_std = float(np.mean(raw_pts)), float(np.std(raw_pts, ddof=1))
        f9_ate_gated_mean, gated_std = float(np.mean(gated_pts)), float(np.std(gated_pts, ddof=1))

        exp_raw_mean, exp_gated_mean = 3.3206, 3.0235
        exp_raw_std, exp_gated_std = 0.5420, 0.3281

        print(f"F9 RAW ATE Mean: {f9_ate_raw_mean:.4f} (expected {exp_raw_mean}), Std: {raw_std:.4f} (expected {exp_raw_std})")
        print(f"F9 GATED ATE Mean: {f9_ate_gated_mean:.4f} (expected {exp_gated_mean}), Std: {gated_std:.4f} (expected {exp_gated_std})")

        self.assertAlmostEqual(f9_ate_raw_mean, exp_raw_mean, delta=1e-3, msg=f"RAW ATE Mean mismatch: {f9_ate_raw_mean} vs {exp_raw_mean}")
        self.assertAlmostEqual(f9_ate_gated_mean, exp_gated_mean, delta=1e-3, msg=f"GATED ATE Mean mismatch: {f9_ate_gated_mean} vs {exp_gated_mean}")
        self.assertAlmostEqual(raw_std, exp_raw_std, delta=1e-3, msg=f"RAW ATE Std mismatch: {raw_std} vs {exp_raw_std}")
        self.assertAlmostEqual(gated_std, exp_gated_std, delta=1e-3, msg=f"GATED ATE Std mismatch: {gated_std} vs {exp_gated_std}")

    def test_f9_ladder_validity_means(self):
        """F9 ladder validity means RAW 93.04, FIXED 62.51, INCREMENTAL 88.17, NULL 93.28, GATED 92.14 (0.01)."""
        df_ladder = pd.read_csv(REPO_ROOT / "figures" / "data" / "fig_04_f9_ladder.csv")
        means = df_ladder.groupby("mechanism")["valid_pct"].mean()

        expected = {
            "RAW": 93.04,
            "FIXED": 62.51,
            "INCREMENTAL": 88.17,
            "EIS-NULL": 93.28,
            "GATED": 92.14,
        }

        for mech, exp_val in expected.items():
            actual_val = float(means[mech])
            print(f"F9 Ladder {mech} Validity Mean: {actual_val:.2f} (expected {exp_val})")
            self.assertAlmostEqual(actual_val, exp_val, delta=0.01, msg=f"Ladder {mech} validity mismatch: {actual_val} vs {exp_val}")

    def test_fig_06_core_matrix_p_values(self):
        """figures/data/fig_06_core_matrix.csv: 24 rows; exactly one paired_t_p < 0.05 and it is F10_L3 valid_pose_pct with k_gated_better == 0."""
        df_core = pd.read_csv(REPO_ROOT / "figures" / "data" / "fig_06_core_matrix.csv")
        self.assertEqual(len(df_core), 24, f"Expected 24 rows in fig_06_core_matrix.csv, got {len(df_core)}")

        sig_rows = df_core[df_core["paired_t_p"] < 0.05]
        self.assertEqual(len(sig_rows), 1, f"Expected exactly 1 row with paired_t_p < 0.05, got {len(sig_rows)}")

        sig_row = sig_rows.iloc[0]
        print(f"Significant row: family={sig_row['family']}, metric={sig_row['metric']}, p={sig_row['paired_t_p']}, k_gated_better={sig_row['k_gated_better']}")

        self.assertEqual(sig_row["family"], "F10_L3")
        self.assertEqual(sig_row["metric"], "valid_pose_pct")
        self.assertEqual(sig_row["k_gated_better"], 0)

    def test_dataset_counts(self):
        """results/datasets counts: 18 p3_, 12 p3x_, 8 phase2a_, 3 phase2b_."""
        ds_dir = REPO_ROOT / "results" / "datasets"
        dirs = [d.name for d in ds_dir.iterdir() if d.is_dir()]

        count_p3 = sum(1 for d in dirs if d.startswith("p3_"))
        count_p3x = sum(1 for d in dirs if d.startswith("p3x_"))
        count_phase2a = sum(1 for d in dirs if d.startswith("phase2a_"))
        count_phase2b = sum(1 for d in dirs if d.startswith("phase2b_"))

        print(f"Dataset counts: p3_={count_p3} (exp 18), p3x_={count_p3x} (exp 12), phase2a_={count_phase2a} (exp 8), phase2b_={count_phase2b} (exp 3)")

        self.assertEqual(count_p3, 18, f"Expected 18 p3_ dirs, got {count_p3}")
        self.assertEqual(count_p3x, 12, f"Expected 12 p3x_ dirs, got {count_p3x}")
        self.assertEqual(count_phase2a, 8, f"Expected 8 phase2a_ dirs, got {count_phase2a}")
        self.assertEqual(count_phase2b, 3, f"Expected 3 phase2b_ dirs, got {count_phase2b}")

    def test_eis_gate_scale_values(self):
        """Every eis_gated_vo.csv under results/datasets has eis_gate_scale values only in {0.0, 1.0}."""
        ds_dir = REPO_ROOT / "results" / "datasets"
        gated_csvs = list(ds_dir.glob("**/eis_gated_vo.csv"))
        self.assertGreater(len(gated_csvs), 0, "No eis_gated_vo.csv files found")

        for csv_path in gated_csvs:
            df = pd.read_csv(csv_path)
            scales = set(df["eis_gate_scale"].unique())
            self.assertTrue(scales.issubset({0.0, 1.0}), f"Invalid eis_gate_scale values in {csv_path}: {scales}")

        print(f"Verified {len(gated_csvs)} eis_gated_vo.csv files have only {{0.0, 1.0}} scale values.")

    def test_gt_heading_audit_jumps(self):
        """results/analysis/gt_heading_audit.csv: among core+exploratory runs exactly p3_F6_L2_R1 (4), p3_F6_L2_R3 (4), p3_F10_L3_R2 (11) have n_heading_jumps_gt10deg > 0."""
        df_audit = pd.read_csv(REPO_ROOT / "results" / "analysis" / "gt_heading_audit.csv")
        core_expl_names = {
            "p3_F1_L2_R1", "p3_F1_L2_R2", "p3_F1_L2_R3",
            "p3_F2_L2_R1", "p3_F2_L2_R2", "p3_F2_L2_R3",
            "p3_F4_L2_R1", "p3_F4_L2_R2", "p3_F4_L2_R3",
            "phase2a_F5_L2_R1", "phase2a_F5_L2_R2", "phase2a_F5_L2_R3",
            "p3_F6_L2_R1", "p3_F6_L2_R2", "p3_F6_L2_R3",
            "phase2a_F9_L2_R1", "phase2a_F9_L2_R2", "phase2a_F9_L2_R3",
            "p3_F10_L3_R1", "p3_F10_L3_R2", "p3_F10_L3_R3",
            "p3_F11_L2_R1", "p3_F11_L2_R2", "p3_F11_L2_R3",
            "p3x_HOVER_L0_R1", "p3x_HOVER_L0_R2", "p3x_HOVER_L0_R3",
            "p3x_F3_L2_R1", "p3x_F3_L2_R2", "p3x_F3_L2_R3",
            "p3x_F7_L2_R1", "p3x_F7_L2_R2", "p3x_F7_L2_R3",
            "p3x_F8_L2_R1", "p3x_F8_L2_R2", "p3x_F8_L2_R3",
        }
        df_sub = df_audit[df_audit["dataset_dir"].isin(core_expl_names)]
        jumps = df_sub[df_sub["n_heading_jumps_gt10deg"] > 0]
        jump_dict = dict(zip(jumps["dataset_dir"], jumps["n_heading_jumps_gt10deg"]))

        expected = {"p3_F6_L2_R1": 4, "p3_F6_L2_R3": 4, "p3_F10_L3_R2": 11}
        print(f"Heading jumps in core+exploratory runs: {jump_dict}")
        self.assertEqual(jump_dict, expected, f"Heading jumps mismatch: {jump_dict} vs {expected}")

    def test_no_absolute_paths_in_tracked_files(self):
        """No tracked text file contains absolute home path or file scheme (excluding results/datasets)."""
        pattern1 = "/home/" + "purab"
        pattern2 = "file://" + "/"
        try:
            out = subprocess.check_output(['git', 'grep', '-n', '-e', pattern1, '-e', pattern2]).decode('utf-8')
            bad_lines = [line for line in out.splitlines() if not line.startswith('results/datasets/')]
        except subprocess.CalledProcessError:
            bad_lines = []
            
        print(f"Tracked files absolute paths check: {len(bad_lines)} occurrences found.")
        self.assertEqual(len(bad_lines), 0, f"Found absolute paths in tracked files:\n" + "\n".join(bad_lines[:10]))

    def test_no_broken_symlinks(self):
        """No broken symlinks in repository."""
        broken = []
        for root, dirs, files in os.walk(REPO_ROOT):
            if ".git" in root:
                continue
            for name in files + dirs:
                p = Path(root) / name
                if p.is_symlink() and not p.exists():
                    broken.append(str(p.relative_to(REPO_ROOT)))

        print(f"Broken symlinks count: {len(broken)}")
        self.assertEqual(len(broken), 0, f"Found broken symlinks: {broken}")


if __name__ == "__main__":
    unittest.main()
