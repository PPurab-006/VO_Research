#!/usr/bin/env python3
"""
Analysis of Inlier Ratio and Tracking Quality for Monocular Visual Odometry across Flights.

Methodology Update (Approach A):
- Re-detection Reset Handling: When tracked points drop below threshold (<100) or at initialization,
  minimal_vo.py re-detects features and sets num_matched=0, num_inliers=0, inlier_ratio=0.0 on that frame.
- On these anchor frames (num_matched == 0), no inter-frame feature matching is attempted against a prior frame.
- Counting these initialization reference frames as 0% inliers is a measurement artifact.
- Approach A excludes re-detection-reset frames (num_matched == 0) from inlier ratio statistics
  (mean, median, min, max, std), evaluating only active frames where inter-frame feature tracking was executed (num_matched > 0).
"""

import os
import csv
import numpy as np

FLIGHT_PATHS = {
    'v1': {
        'name': 'v1 (textured.sdf)',
        'gt': 'src/results/ground_truth_textured_circular.csv',
        'vo': 'src/results/vo_trajectory_textured_circular.csv',
        'world': 'textured.sdf',
        'center': '(5.5, 0.0) m',
        'radius': '4.5 m',
        'alt': '2.0 m'
    },
    'v2': {
        'name': 'v2 (textured.sdf)',
        'gt': 'src/results/circular_flight_v2_gt.csv',
        'vo': 'src/results/circular_flight_v2_vo.csv',
        'world': 'textured.sdf',
        'center': '(6.0, 1.0) m',
        'radius': '4.0 m',
        'alt': '2.0 m'
    },
    'v3': {
        'name': 'v3 original (boxworld_tight)',
        'gt': 'src/results/circular_flight_v3_gt.csv',
        'vo': 'src/results/circular_flight_v3_vo.csv',
        'world': 'boxworld_tight',
        'center': '(5.0, 5.0) m',
        'radius': '6.0 m',
        'alt': '2.5 m'
    },
    'v3_rerun': {
        'name': 'v3-rerun post-GPU (boxworld_tight)',
        'gt': 'src/results/circular_flight_v3_rerun_gt.csv',
        'vo': 'src/results/circular_flight_v3_rerun_vo.csv',
        'world': 'boxworld_tight',
        'center': '(5.0, 5.0) m',
        'radius': '6.0 m',
        'alt': '2.5 m'
    }
}

def load_csv(filepath):
    records = []
    if not os.path.exists(filepath):
        return records
    with open(filepath, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            records.append(row)
    return records

def calc_r(x, y):
    std_x = np.std(x)
    std_y = np.std(y)
    if std_x == 0 or std_y == 0:
        return 0.0
    return float(np.corrcoef(x, y)[0, 1])

def process_flight(flight_key, info):
    gt_records = load_csv(info['gt'])
    vo_records = load_csv(info['vo'])
    
    if not vo_records:
        print(f"[ERROR] Missing VO file for {flight_key}: {info['vo']}")
        return None

    # Filter to active flight (pos_z > 0.3m)
    active_vo = [r for r in vo_records if float(r['pos_z']) > 0.3]
    if not active_vo:
        active_vo = vo_records

    # Categorize frames
    redet_frames = [r for r in active_vo if int(r['num_matched']) == 0]
    real_zero_inl = [r for r in active_vo if int(r['num_matched']) > 0 and int(r['num_inliers']) == 0]
    valid_inl_frames = [r for r in active_vo if int(r['num_inliers']) > 0]
    eval_frames = [r for r in active_vo if int(r['num_matched']) > 0] # Approach A

    # Inlier Ratios
    ratios_raw = np.array([float(r['inlier_ratio']) * 100.0 for r in active_vo])
    ratios_corr = np.array([float(r['inlier_ratio']) * 100.0 for r in eval_frames]) if eval_frames else np.array([0.0])

    # Pearson Correlation (Active Flight)
    vo_ts = np.array([float(r['timestamp_total_sec']) for r in active_vo])
    gt_ts = np.array([float(r['timestamp_total_sec']) for r in gt_records])
    
    m_vo_x, m_vo_y, m_vo_z = [], [], []
    m_gt_x, m_gt_y, m_gt_z = [], [], []

    if len(gt_records) > 0 and len(active_vo) > 0:
        vo_rel_ts = vo_ts - vo_ts[0]
        gt_rel_ts = gt_ts - gt_ts[0]

        for idx, r_vo in enumerate(active_vo):
            t_vo = vo_rel_ts[idx]
            diffs = np.abs(gt_rel_ts - t_vo)
            best_idx = np.argmin(diffs)
            if diffs[best_idx] < 0.1:  # 100ms tolerance
                r_gt = gt_records[best_idx]
                m_vo_x.append(float(r_vo['pos_x']))
                m_vo_y.append(float(r_vo['pos_y']))
                m_vo_z.append(float(r_vo['pos_z']))
                m_gt_x.append(float(r_gt['pos_x']))
                m_gt_y.append(float(r_gt['pos_y']))
                m_gt_z.append(float(r_gt['pos_z']))

    r_x = calc_r(m_vo_x, m_gt_x) if m_vo_x else 0.0
    r_y = calc_r(m_vo_y, m_gt_y) if m_vo_y else 0.0
    r_z = calc_r(m_vo_z, m_gt_z) if m_vo_z else 0.0

    return {
        'info': info,
        'total_active': len(active_vo),
        'redet_count': len(redet_frames),
        'redet_pct': (len(redet_frames) / len(active_vo)) * 100.0 if active_vo else 0.0,
        'real_zero_count': len(real_zero_inl),
        'real_zero_pct': (len(real_zero_inl) / len(active_vo)) * 100.0 if active_vo else 0.0,
        'valid_count': len(valid_inl_frames),
        'valid_pct': (len(valid_inl_frames) / len(active_vo)) * 100.0 if active_vo else 0.0,
        'eval_count': len(eval_frames),
        'raw_mean': np.mean(ratios_raw),
        'corr_mean': np.mean(ratios_corr),
        'corr_median': np.median(ratios_corr),
        'corr_min': np.min(ratios_corr),
        'corr_max': np.max(ratios_corr),
        'corr_std': np.std(ratios_corr),
        'r_x': r_x,
        'r_y': r_y,
        'r_z': r_z
    }

def main():
    print("==========================================================================================")
    print("INLIER RATIO & TRACKING QUALITY ANALYSIS (HANDLING FEATURE RE-DETECTION RESETS)")
    print("==========================================================================================")

    results = {}
    for key, info in FLIGHT_PATHS.items():
        res = process_flight(key, info)
        if res:
            results[key] = res

    # Detailed Audit per Flight
    for key, res in results.items():
        print(f"\n--- FRAME AUDIT & STATS: {res['info']['name']} ---")
        print(f"Total Active Flight Frames (pos_z > 0.3) : {res['total_active']}")
        print(f"Re-detection Reset Frames (num_matched=0): {res['redet_count']} ({res['redet_pct']:.2f}%) [EXCLUDED IN APPROACH A]")
        print(f"Genuine 0-Inlier Frames (num_matched>0)  : {res['real_zero_count']} ({res['real_zero_pct']:.2f}%)")
        print(f"Non-Zero Inlier Frames (num_inliers>0)  : {res['valid_count']} ({res['valid_pct']:.2f}%)")
        print(f"Evaluated Frames (num_matched > 0)       : {res['eval_count']}")
        print(f"Raw Mean Inlier Ratio (all active)       : {res['raw_mean']:.2f}%")
        print(f"Corrected Inlier Ratio (Approach A)      :")
        print(f"  Mean   : {res['corr_mean']:.2f}%")
        print(f"  Median : {res['corr_median']:.2f}%")
        print(f"  Min/Max: {res['corr_min']:.2f}% / {res['corr_max']:.2f}%")
        print(f"  StdDev : {res['corr_std']:.2f}%")
        print(f"Pearson Correlation (r_x, r_y, r_z)      : ({res['r_x']:+.4f}, {res['r_y']:+.4f}, {res['r_z']:+.4f})")

    # Comprehensive Comparison Table
    print("\n=======================================================================================================================")
    print("FOUR-FLIGHT COMPARISON TABLE (CORRECTED METHODOLOGY — APPROACH A)")
    print("=======================================================================================================================")
    col_w = 26
    hdr = f"{'Metric / Parameter':<32} | " + " | ".join([f"{res['info']['name']:<{col_w}}" for res in results.values()])
    print(hdr)
    print("-" * len(hdr))

    def row_fmt(label, val_func):
        vals = [val_func(res) for res in results.values()]
        return f"{label:<32} | " + " | ".join([f"{v:<{col_w}}" for v in vals])

    print(row_fmt("World Environment", lambda r: r['info']['world']))
    print(row_fmt("Circle Center (x, y)", lambda r: r['info']['center']))
    print(row_fmt("Circle Radius (R)", lambda r: r['info']['radius']))
    print(row_fmt("Cruise Altitude (Z)", lambda r: r['info']['alt']))
    print(row_fmt("Total Active Frames", lambda r: f"{r['total_active']}"))
    print(row_fmt("Re-detection Reset Frames", lambda r: f"{r['redet_count']} ({r['redet_pct']:.1f}%)"))
    print(row_fmt("Evaluated Frames (Approach A)", lambda r: f"{r['eval_count']}"))
    print(row_fmt("Raw Mean Inlier Ratio", lambda r: f"{r['raw_mean']:.2f}%"))
    print(row_fmt("Corrected Mean Inlier Ratio", lambda r: f"{r['corr_mean']:.2f}%"))
    print(row_fmt("Corrected Median Inlier Ratio", lambda r: f"{r['corr_median']:.2f}%"))
    print(row_fmt("Corrected Std Inlier Ratio", lambda r: f"{r['corr_std']:.2f}%"))
    print(row_fmt("Active-Flight Pearson r_x", lambda r: f"{r['r_x']:+.4f}"))
    print(row_fmt("Active-Flight Pearson r_y", lambda r: f"{r['r_y']:+.4f}"))
    print(row_fmt("Active-Flight Pearson r_z", lambda r: f"{r['r_z']:+.4f}"))
    print("=======================================================================================================================\n")

if __name__ == '__main__':
    main()


