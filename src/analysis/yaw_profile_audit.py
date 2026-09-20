#!/usr/bin/env python3
"""
1.5 Yaw Profile Audit.

For F6, F9, F10: derive heading from GT quaternion as atan2(y,x) of the rotated
body +X axis, np.unwrap, over the active window.

Reports: yaw at window start, mean heading offset, min, max, peak-to-peak,
dominant oscillation frequency, rate p50/p95/max on 50 Hz resampled grid.

Also computes from VO-logged eis_yaw_rate_deg to compare.

Writes results/analysis/achieved_yaw_audit.csv.
"""

import sys
from pathlib import Path

import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation as R_scipy

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(REPO_ROOT / "src" / "pipelines"))
from evaluate_phase3_evo import get_canonical_active_window

DATASETS = REPO_ROOT / "results" / "datasets"
OUTPUT = REPO_ROOT / "results" / "analysis" / "achieved_yaw_audit.csv"

AUDIT_RUNS = [
    ("p3_F6_L2_R1", "F6"),
    ("p3_F6_L2_R2", "F6"),
    ("p3_F6_L2_R3", "F6"),
    ("p3_F10_L3_R1", "F10"),
    ("p3_F10_L3_R2", "F10"),
    ("p3_F10_L3_R3", "F10"),
    ("phase2a_F9_L2_R1", "F9"),
    ("phase2a_F9_L2_R2", "F9"),
    ("phase2a_F9_L2_R3", "F9"),
]


def heading_from_quaternion(qx, qy, qz, qw):
    """Compute heading (yaw) from quaternion by rotating body +X axis and taking atan2."""
    rot = R_scipy.from_quat(np.column_stack([qx, qy, qz, qw]))
    # Rotate body +X axis [1, 0, 0]
    body_x = rot.apply(np.array([1.0, 0.0, 0.0]))
    heading = np.arctan2(body_x[:, 1], body_x[:, 0])
    return heading


def main():
    rows = []

    for dir_name, family in AUDIT_RUNS:
        ds_dir = DATASETS / dir_name
        gt_csv = ds_dir / "dataset_gt.csv"
        gated_csv = ds_dir / "eis_gated_vo.csv"

        if not gt_csv.exists():
            print(f"WARNING: {dir_name} not found")
            continue

        df_gt = pd.read_csv(gt_csv)

        # Drop duplicate timestamps FIRST
        t_col = "timestamp_total_sec"
        df_gt = df_gt.drop_duplicates(subset=[t_col], keep="first").reset_index(drop=True)

        t_start, t_end, dur = get_canonical_active_window(df_gt)

        # Filter to active window
        gt_t = df_gt[t_col].values.astype(float)
        mask = (gt_t >= t_start) & (gt_t <= t_end)
        df_act = df_gt[mask].reset_index(drop=True)

        if len(df_act) < 10:
            continue

        # Compute heading
        heading_rad = heading_from_quaternion(
            df_act["rot_x"].values, df_act["rot_y"].values,
            df_act["rot_z"].values, df_act["rot_w"].values
        )
        heading_deg = np.degrees(np.unwrap(heading_rad))

        yaw_start = heading_deg[0]
        mean_offset = float(np.mean(heading_deg) - yaw_start)
        yaw_min = float(np.min(heading_deg))
        yaw_max = float(np.max(heading_deg))
        peak_to_peak = yaw_max - yaw_min

        # Dominant frequency via FFT of detrended yaw
        t_act = df_act[t_col].values.astype(float)
        dt_median = np.median(np.diff(t_act))
        heading_detrended = heading_deg - np.polyval(np.polyfit(t_act - t_act[0], heading_deg, 1), t_act - t_act[0])
        fft_vals = np.abs(np.fft.rfft(heading_detrended))
        freqs = np.fft.rfftfreq(len(heading_detrended), d=dt_median)
        # Exclude DC
        if len(freqs) > 1:
            fft_vals[0] = 0
            dominant_freq = float(freqs[np.argmax(fft_vals)])
        else:
            dominant_freq = 0.0

        # Resample to 50 Hz uniform grid for rate computation
        t_uniform = np.arange(t_act[0], t_act[-1], 1.0 / 50.0)
        heading_interp = np.interp(t_uniform, t_act, heading_deg)
        rates = np.abs(np.diff(heading_interp) * 50.0)  # deg/s
        rate_p50 = float(np.percentile(rates, 50)) if len(rates) > 0 else 0.0
        rate_p95 = float(np.percentile(rates, 95)) if len(rates) > 0 else 0.0
        rate_max = float(np.max(rates)) if len(rates) > 0 else 0.0

        # VO-logged yaw rate comparison
        vo_rate_p50 = np.nan
        vo_rate_p95 = np.nan
        vo_rate_max = np.nan
        if gated_csv.exists():
            df_gated = pd.read_csv(gated_csv)
            gated_mask = (df_gated["timestamp_total_sec"] >= t_start) & (df_gated["timestamp_total_sec"] <= t_end)
            df_gated_act = df_gated[gated_mask]
            if "eis_yaw_rate_deg" in df_gated_act.columns and len(df_gated_act) > 0:
                vo_rates = df_gated_act["eis_yaw_rate_deg"].abs().values
                vo_rate_p50 = float(np.percentile(vo_rates, 50))
                vo_rate_p95 = float(np.percentile(vo_rates, 95))
                vo_rate_max = float(np.max(vo_rates))

        rows.append({
            "run_dir": dir_name,
            "family": family,
            "yaw_start_deg": round(yaw_start, 2),
            "mean_heading_offset_deg": round(mean_offset, 2),
            "yaw_min_deg": round(yaw_min, 2),
            "yaw_max_deg": round(yaw_max, 2),
            "peak_to_peak_deg": round(peak_to_peak, 2),
            "dominant_freq_hz": round(dominant_freq, 3),
            "gt_rate_p50_degs": round(rate_p50, 2),
            "gt_rate_p95_degs": round(rate_p95, 2),
            "gt_rate_max_degs": round(rate_max, 2),
            "vo_rate_p50_degs": round(vo_rate_p50, 2) if not np.isnan(vo_rate_p50) else np.nan,
            "vo_rate_p95_degs": round(vo_rate_p95, 2) if not np.isnan(vo_rate_p95) else np.nan,
            "vo_rate_max_degs": round(vo_rate_max, 2) if not np.isnan(vo_rate_max) else np.nan,
            "active_window_sec": round(dur, 2),
        })

    df = pd.DataFrame(rows)
    df.to_csv(OUTPUT, index=False)

    # Print summary
    print("\nAchieved Yaw Audit:")
    print(f"{'Run':>25s} | {'Offset':>8s} | {'P-P':>7s} | {'Freq':>5s} | {'p50':>6s} | {'p95':>6s} | {'max':>6s}")
    print("-" * 80)
    for _, r in df.iterrows():
        print(f"{r['run_dir']:>25s} | {r['mean_heading_offset_deg']:>8.2f} | "
              f"{r['peak_to_peak_deg']:>7.2f} | {r['dominant_freq_hz']:>5.3f} | "
              f"{r['gt_rate_p50_degs']:>6.2f} | {r['gt_rate_p95_degs']:>6.2f} | {r['gt_rate_max_degs']:>6.2f}")

    print(f"\nWrote {len(df)} rows to {OUTPUT.relative_to(REPO_ROOT)}")


if __name__ == "__main__":
    main()
