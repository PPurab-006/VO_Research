#!/usr/bin/env python3
"""
Diagnostic Trajectory Correlation Script for Monocular VO vs Ground Truth

Pre-alignment sanity check script that loads raw unit-scale VO trajectory
and metric ground-truth pose CSVs, checks timestamp range overlap, pairs frames
using nearest-neighbor timestamp matching, normalizes position sequences per axis,
and computes Pearson correlation coefficients (r) for X, Y, Z axes.

Saves visualization plot to `plots/correlation_check.png`.
"""

import argparse
import csv
import os
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def load_csv(filepath):
    """Loads a CSV file into a list of dictionaries."""
    if not os.path.exists(filepath):
        raise FileNotFoundError(f"CSV file not found: {filepath}")

    records = []
    with open(filepath, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            records.append(row)
    return records


def zscore(data):
    """Computes Z-score normalization of 1D numpy array."""
    data = np.asarray(data, dtype=np.float64)
    std = np.std(data)
    if std == 0 or np.isnan(std):
        return np.zeros_like(data)
    return (data - np.mean(data)) / std


def check_timestamp_overlap(vo_ts, gt_ts):
    """
    Evaluates absolute and relative timestamp range overlap between VO and GT.
    Returns absolute overlap duration (sec), relative overlap duration (sec),
    and boolean flags.
    """
    vo_min, vo_max = np.min(vo_ts), np.max(vo_ts)
    gt_min, gt_max = np.min(gt_ts), np.max(gt_ts)

    abs_overlap_start = max(vo_min, gt_min)
    abs_overlap_end = min(vo_max, gt_max)
    abs_overlap_sec = max(0.0, abs_overlap_end - abs_overlap_start)

    # Relative timestamps (zeroed to start time)
    vo_rel = vo_ts - vo_min
    gt_rel = gt_ts - gt_min

    rel_overlap_end = min(np.max(vo_rel), np.max(gt_rel))
    rel_overlap_sec = max(0.0, rel_overlap_end)

    return {
        'vo_min': vo_min, 'vo_max': vo_max, 'vo_duration': vo_max - vo_min,
        'gt_min': gt_min, 'gt_max': gt_max, 'gt_duration': gt_max - gt_min,
        'abs_overlap_sec': abs_overlap_sec,
        'has_abs_overlap': abs_overlap_sec > 0.1,
        'rel_overlap_sec': rel_overlap_sec,
        'vo_rel': vo_rel,
        'gt_rel': gt_rel
    }


def associate_timestamps(vo_records, gt_records, tolerance_sec=0.05, use_relative=True):
    """
    Nearest-neighbor timestamp matching between VO frames and Ground-Truth poses.
    Supports matching on relative timestamps (start-time zeroed) when absolute timestamps differ.
    """
    vo_raw_ts = np.array([float(r['timestamp_total_sec']) for r in vo_records])
    gt_raw_ts = np.array([float(r['timestamp_total_sec']) for r in gt_records])

    if use_relative:
        vo_match_ts = vo_raw_ts - vo_raw_ts[0]
        gt_match_ts = gt_raw_ts - gt_raw_ts[0]
    else:
        vo_match_ts = vo_raw_ts
        gt_match_ts = gt_raw_ts

    matched_vo = []
    matched_gt = []
    gaps_ms = []

    for idx, vo in enumerate(vo_records):
        t_vo = vo_match_ts[idx]
        diffs = np.abs(gt_match_ts - t_vo)
        best_idx = np.argmin(diffs)
        min_gap_sec = diffs[best_idx]
        min_gap_ms = min_gap_sec * 1000.0

        if min_gap_sec <= tolerance_sec:
            matched_vo.append(vo)
            matched_gt.append(gt_records[best_idx])
            gaps_ms.append(min_gap_ms)

    return matched_vo, matched_gt, gaps_ms


def compute_axis_correlations(matched_vo, matched_gt):
    """
    Extracts positions, normalizes each axis (x, y, z) separately, and calculates
    Pearson correlation coefficients.
    """
    vo_x = np.array([float(r['pos_x']) for r in matched_vo], dtype=np.float64)
    vo_y = np.array([float(r['pos_y']) for r in matched_vo], dtype=np.float64)
    vo_z = np.array([float(r['pos_z']) for r in matched_vo], dtype=np.float64)

    gt_x = np.array([float(r['pos_x']) for r in matched_gt], dtype=np.float64)
    gt_y = np.array([float(r['pos_y']) for r in matched_gt], dtype=np.float64)
    gt_z = np.array([float(r['pos_z']) for r in matched_gt], dtype=np.float64)

    norm_vo_x, norm_gt_x = zscore(vo_x), zscore(gt_x)
    norm_vo_y, norm_gt_y = zscore(vo_y), zscore(gt_y)
    norm_vo_z, norm_gt_z = zscore(vo_z), zscore(gt_z)

    rx = np.corrcoef(norm_vo_x, norm_gt_x)[0, 1] if len(vo_x) > 1 else 0.0
    ry = np.corrcoef(norm_vo_y, norm_gt_y)[0, 1] if len(vo_y) > 1 else 0.0
    rz = np.corrcoef(norm_vo_z, norm_gt_z)[0, 1] if len(vo_z) > 1 else 0.0

    return {
        'vo_norm': {'x': norm_vo_x, 'y': norm_vo_y, 'z': norm_vo_z},
        'gt_norm': {'x': norm_gt_x, 'y': norm_gt_y, 'z': norm_gt_z},
        'r': {'x': rx, 'y': ry, 'z': rz}
    }


def plot_correlation(correlations, output_plot_path):
    """Generates a 3-subplot figure showing normalized VO vs GT position trajectories."""
    os.makedirs(os.path.dirname(output_plot_path), exist_ok=True)

    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    fig.suptitle("Pre-Alignment Shape Correlation Check (VO vs Ground Truth)", fontsize=14, fontweight='bold')

    axes_names = ['X', 'Y', 'Z']
    colors = ['#1f77b4', '#ff7f0e', '#2ca02c']

    for i, axis_key in enumerate(['x', 'y', 'z']):
        ax = axes[i]
        vo_norm = correlations['vo_norm'][axis_key]
        gt_norm = correlations['gt_norm'][axis_key]
        r_val = correlations['r'][axis_key]

        frame_indices = np.arange(len(vo_norm))

        ax.plot(frame_indices, vo_norm, label='VO (Normalized)', color='blue', linestyle='--', linewidth=1.8, marker='o', markersize=3)
        ax.plot(frame_indices, gt_norm, label='Ground Truth (Normalized)', color='orange', linestyle='-', linewidth=2.0)

        # Color title based on correlation value
        if r_val >= 0.7:
            status_str = f"r = {r_val:+.4f} (Strong Positive)"
            title_color = 'darkgreen'
        elif r_val >= 0.0:
            status_str = f"r = {r_val:+.4f} (Weak Positive)"
            title_color = 'darkorange'
        else:
            status_str = f"r = {r_val:+.4f} (NEGATIVE / INVERTED)"
            title_color = 'darkred'

        ax.set_title(f"{axes_names[i]}-Axis Trajectory Shape Correlation ({status_str})", fontsize=11, fontweight='bold', color=title_color)
        ax.set_ylabel("Normalized Pos (Z-Score)", fontsize=9)
        ax.grid(True, linestyle=':', alpha=0.6)
        ax.legend(loc='upper right', fontsize=9)

    axes[2].set_xlabel("Matched Frame Index", fontsize=10)
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.savefig(output_plot_path, dpi=150)
    plt.close()
    print(f"\nPlot successfully saved to: '{os.path.abspath(output_plot_path)}'")


def main():
    parser = argparse.ArgumentParser(description="Diagnostic Shape Correlation Check (VO vs Ground Truth)")
    parser.add_argument(
        '--vo-csv',
        default='results/vo_trajectory_textured.csv',
        help="Path to VO trajectory CSV file"
    )
    parser.add_argument(
        '--gt-csv',
        default='results/ground_truth_textured_REAL.csv',
        help="Path to Ground Truth CSV file"
    )
    parser.add_argument(
        '--plot-path',
        default='plots/correlation_check.png',
        help="Path to output plot PNG"
    )
    parser.add_argument(
        '--tolerance-ms',
        type=float,
        default=50.0,
        help="Nearest-neighbor matching timestamp tolerance in ms"
    )

    args = parser.parse_args()

    print("=" * 75)
    print("DIAGNOSTIC SHAPE CORRELATION CHECK (PRE-ALIGNMENT SANITY)")
    print("=" * 75)
    print(f"VO Trajectory CSV : {os.path.abspath(args.vo_csv)}")
    print(f"Ground Truth CSV : {os.path.abspath(args.gt_csv)}")
    print(f"Output Plot Path : {os.path.abspath(args.plot_path)}")
    print(f"Sync Tolerance   : {args.tolerance_ms:.1f} ms")
    print("-" * 75)

    vo_records = load_csv(args.vo_csv)
    gt_records = load_csv(args.gt_csv)

    vo_ts = np.array([float(r['timestamp_total_sec']) for r in vo_records])
    gt_ts = np.array([float(r['timestamp_total_sec']) for r in gt_records])

    # Step 2: Timestamp Overlap Check
    overlap_info = check_timestamp_overlap(vo_ts, gt_ts)

    print("\n--- 1. TIMESTAMP RANGE OVERLAP CHECK ---")
    print(f"VO Timestamp Range : [{overlap_info['vo_min']:.3f} s  -->  {overlap_info['vo_max']:.3f} s] (Duration: {overlap_info['vo_duration']:.3f} s, {len(vo_records)} frames)")
    print(f"GT Timestamp Range : [{overlap_info['gt_min']:.3f} s  -->  {overlap_info['gt_max']:.3f} s] (Duration: {overlap_info['gt_duration']:.3f} s, {len(gt_records)} poses)")

    use_relative_matching = False
    if overlap_info['has_abs_overlap']:
        print(f"Absolute Overlap   : YES ({overlap_info['abs_overlap_sec']:.3f} s overlap)")
    else:
        print("Absolute Overlap   : NO (0.000 s overlap)")
        print("\n[WARNING] Absolute timestamp ranges do NOT overlap!")
        print("  - VO and GT recordings were generated in separate execution sessions.")
        print("  - Falling back to relative timestamp matching (zero-aligned to start of trajectory) for shape analysis.")
        use_relative_matching = True

    # Step 3: Nearest-Neighbor Timestamp Matching
    matched_vo, matched_gt, gaps_ms = associate_timestamps(
        vo_records, gt_records,
        tolerance_sec=args.tolerance_ms / 1000.0,
        use_relative=use_relative_matching
    )

    print("\n--- 2. NEAREST-NEIGHBOR MATCHING STATISTICS ---")
    print(f"Matched Pairs Logged : {len(matched_vo)} / {len(vo_records)} VO frames ({len(matched_vo)/len(vo_records)*100.0:.1f}%)")

    if len(matched_vo) == 0:
        print("\n[ERROR] Zero timestamp matches found within tolerance! Cannot compute correlation.")
        sys.exit(1)

    print(f"Gap Min / Max        : {np.min(gaps_ms):.2f} ms / {np.max(gaps_ms):.2f} ms")
    print(f"Gap Mean ± StdDev    : {np.mean(gaps_ms):.2f} ms ± {np.std(gaps_ms):.2f} ms")
    print(f"Gap Median           : {np.median(gaps_ms):.2f} ms")

    # Step 4: Normalized Axis Correlation Calculation
    correlations = compute_axis_correlations(matched_vo, matched_gt)

    rx = correlations['r']['x']
    ry = correlations['r']['y']
    rz = correlations['r']['z']

    print("\n--- 3. PEARSON SHAPE CORRELATION COEFFICIENTS (r) ---")
    print(f"  X-Axis Pearson r : {rx:+.4f}")
    print(f"  Y-Axis Pearson r : {ry:+.4f}")
    print(f"  Z-Axis Pearson r : {rz:+.4f}")

    # Step 5: Plotting
    plot_correlation(correlations, args.plot_path)

    # Step 6: Summary Flags & Plain-Language Diagnostics
    print("\n" + "=" * 75)
    print("SHAPE CORRELATION DIAGNOSTIC SUMMARY")
    print("=" * 75)

    flagged = False
    for axis_name, r_val in [('X', rx), ('Y', ry), ('Z', rz)]:
        if r_val < 0.0:
            print(f" [FLAG: NEGATIVE CORRELATION] {axis_name}-Axis (r = {r_val:+.4f})")
            print(f"   -> WARNING: VO motion along {axis_name} moves in the OPPOSITE direction of Ground Truth!")
            print(f"   -> Likely Cause: Optical frame to World ENU axis mapping sign inversion or camera transform bug.")
            flagged = True
        elif r_val < 0.5:
            print(f" [FLAG: WEAK CORRELATION] {axis_name}-Axis (r = {r_val:+.4f})")
            print(f"   -> WARNING: VO motion along {axis_name} poorly tracks Ground Truth trajectory shape.")
            print(f"   -> Likely Cause: Feature tracking drift, unmodeled rotation, or timestamp mismatch.")
            flagged = True
        else:
            print(f" [PASS: GOOD CORRELATION] {axis_name}-Axis (r = {r_val:+.4f})")

    print("-" * 75)
    if flagged:
        print("[SUMMARY CONCLUSION] Issues detected in trajectory shape correlation!")
        print("  -> Investigate coordinate frame conventions or timestamp sync before running Sim(3)/evaluate_trajectory.py.")
    else:
        print("[SUMMARY CONCLUSION] All axes demonstrate positive trajectory shape correlation!")
        print("  -> Trajectory shape matches ground truth; ready for Sim(3)/evaluate_trajectory.py alignment.")
    print("=" * 75 + "\n")


if __name__ == '__main__':
    main()
