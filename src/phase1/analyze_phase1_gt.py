#!/usr/bin/env python3
"""
Phase 1 Telemetry Analysis Engine (Repaired & Frame-Aware)

Parses GT telemetry (record_ground_truth.py) and VO tracking output (minimal_vo.py).
Establishes canonical timestamp windows, clean derivative statistics, frame-aware attitude terminology,
and GT-referenced frame-to-frame rotation error evaluation.
"""

import argparse
import sys
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation as R_scipy
from scipy.interpolate import interp1d


def euler_from_quaternion(x, y, z, w):
    """
    Convert quaternion (x, y, z, w) to Gazebo World ENU Euler angles (roll, pitch, yaw) in radians.
    
    Frame Definition:
    - ENU roll  (phi): Rotation around Gazebo +X (East/Longitudinal) axis.
    - ENU pitch (theta): Rotation around Gazebo +Y (North/Lateral) axis.
    - ENU yaw   (psi): Rotation around Gazebo +Z (Up) axis.
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    enu_roll = np.arctan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = np.clip(t2, -1.0, 1.0)
    enu_pitch = np.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    enu_yaw = np.arctan2(t3, t4)

    return enu_roll, enu_pitch, enu_yaw


def get_canonical_active_window(df_gt, df_vo, family='F1', pilot_name=''):
    """
    Canonical Active-Motion Windowing Function.
    
    Hierarchy:
    1. Explicit lifecycle timestamps (MOTION_START / MOTION_END) if logged in dataframe metadata.
    2. Documented altitude & motion state fallback (cruise altitude Z >= 2.0m, Z >= 1.5m for P01).
    3. Never arbitrary hardcoded frame indices.
    
    Returns:
        t_start (float), t_end (float), duration (float), source_desc (str), is_short_duration (bool)
    """
    # Check GT timestamps
    if 'timestamp_total_sec' in df_gt.columns:
        gt_t = df_gt['timestamp_total_sec'].values
    elif 'timestamp' in df_gt.columns:
        gt_t = df_gt['timestamp'].values
        if gt_t[0] > 1e12:
            gt_t = gt_t / 1e9
    else:
        raise ValueError("GT DataFrame missing timestamp column!")

    # Check for monotonic timestamps
    if np.any(np.diff(gt_t) < 0):
        print(f"[WARNING] Non-monotonic GT timestamps detected in {pilot_name}!")

    z_gt = df_gt['pos_z'].values if 'pos_z' in df_gt.columns else df_gt['z'].values
    
    # Check explicit lifecycle columns if present
    if 'event' in df_gt.columns:
        start_mask = df_gt['event'] == 'MOTION_START'
        end_mask = df_gt['event'] == 'MOTION_END'
        if start_mask.any() and end_mask.any():
            t_start = df_gt.loc[start_mask, 'timestamp_total_sec'].iloc[0]
            t_end = df_gt.loc[end_mask, 'timestamp_total_sec'].iloc[0]
            dur = t_end - t_start
            return t_start, t_end, dur, "EXPLICIT_LIFECYCLE_LOG", (dur < 10.0)

    # Fallback to altitude cruise thresholding
    alt_thresh = 1.5 if (family == 'F1' or 'P01' in pilot_name or 'F1_L2' in pilot_name) else 2.0
    idx_active = np.where(z_gt >= alt_thresh)[0]
    
    if len(idx_active) > 0:
        start_idx = idx_active[0]
        end_idx = idx_active[-1]
        source = f"ALTITUDE_CRUISE_FALLBACK (Z >= {alt_thresh:.1f}m)"
    else:
        start_idx = 0
        end_idx = len(df_gt) - 1
        source = "FULL_SERIES_FALLBACK"

    t_start = gt_t[start_idx]
    t_end = gt_t[end_idx]
    duration = t_end - t_start

    is_short = (duration < 10.0)
    if is_short:
        print(f"[FLAG WARNING] Short active motion duration detected for {pilot_name} ({family}): {duration:.2f}s < 10.0s!")

    assert duration > 0, f"Canonical active window duration must be positive, got {duration:.3f}s"
    return t_start, t_end, duration, source, is_short


def compute_clean_derivatives(t, values, is_angle=False):
    """
    Computes numerical derivatives with strict timestamp validity and angle unwrapping rules.
    
    Timestamp Validity Rule:
    - dt <= 0 or dt < 0.002s -> Marked INVALID (NaN).
    - Invalid intervals are excluded from statistics (NOT carried forward).
    - 2ms cutoff is an analysis-quality filter to eliminate ROS 2 dispatch jitter.
    """
    dt = np.diff(t)
    if is_angle:
        unwrapped = np.unwrap(values)
        diff_val = np.diff(unwrapped)
    else:
        diff_val = np.diff(values)

    # Mask valid intervals
    valid_mask = (dt >= 0.002)
    deriv = np.full_like(diff_val, np.nan, dtype=float)
    deriv[valid_mask] = diff_val[valid_mask] / dt[valid_mask]

    n_total = len(diff_val)
    n_valid = int(np.sum(valid_mask))
    n_invalid = n_total - n_valid
    pct_invalid = (n_invalid / n_total * 100.0) if n_total > 0 else 0.0

    valid_deriv = deriv[valid_mask]
    if len(valid_deriv) > 0:
        stats = {
            'total_intervals': n_total,
            'valid_intervals': n_valid,
            'invalid_intervals': n_invalid,
            'invalid_pct': pct_invalid,
            'mean': float(np.mean(valid_deriv)),
            'std': float(np.std(valid_deriv)),
            'median': float(np.median(valid_deriv)),
            'p95': float(np.percentile(valid_deriv, 95)),
            'max': float(np.max(valid_deriv)),
            'min': float(np.min(valid_deriv))
        }
    else:
        stats = {
            'total_intervals': n_total,
            'valid_intervals': 0,
            'invalid_intervals': n_invalid,
            'invalid_pct': 100.0,
            'mean': np.nan, 'std': np.nan, 'median': np.nan, 'p95': np.nan, 'max': np.nan, 'min': np.nan
        }

    return deriv, stats


def analyze_flight_run(gt_csv, vo_csv, family='F1', severity=2, pilot_name=''):
    print("==========================================================================")
    print(f"PHASE 1 PILOT RUN ANALYSIS REPORT: {pilot_name} ({family} Level L{severity})")
    print("==========================================================================")
    print(f"GT File : {gt_csv}")
    print(f"VO File : {vo_csv}")
    print("--------------------------------------------------------------------------")

    try:
        df_gt = pd.read_csv(gt_csv)
    except Exception as e:
        print(f"[ERROR] Could not load GT CSV {gt_csv}: {e}")
        return False

    try:
        df_vo = pd.read_csv(vo_csv)
    except Exception as e:
        print(f"[WARNING] Could not load VO CSV {vo_csv}: {e}")
        df_vo = pd.DataFrame()

    # 1. Canonical Active Motion Windowing
    t_start_gt, t_end_gt, active_dur, window_src, is_short = get_canonical_active_window(df_gt, df_vo, family=family, pilot_name=pilot_name)

    gt_t = df_gt['timestamp_total_sec'].values if 'timestamp_total_sec' in df_gt.columns else df_gt['timestamp'].values
    if gt_t[0] > 1e12:
        gt_t = gt_t / 1e9

    gt_mask = (gt_t >= t_start_gt) & (gt_t <= t_end_gt)
    df_gt_act = df_gt[gt_mask].copy()
    gt_act_samples = len(df_gt_act)

    # VO Canonical Window Mapping
    if len(df_vo) > 0 and 'timestamp_total_sec' in df_vo.columns:
        vo_t = df_vo['timestamp_total_sec'].values
        vo_rel = vo_t - vo_t[0]
        gt_rel = gt_t - gt_t[0]
        
        # Overlapping relative interval
        start_rel = t_start_gt - gt_t[0]
        end_rel = t_end_gt - gt_t[0]
        
        vo_mask = (vo_rel >= start_rel) & (vo_rel <= end_rel)
        df_vo_act = df_vo[vo_mask].copy()
        vo_act_samples = len(df_vo_act)
    else:
        df_vo_act = pd.DataFrame()
        vo_act_samples = 0

    eff_gt_fps = gt_act_samples / active_dur if active_dur > 0 else 0.0
    eff_vo_fps = vo_act_samples / active_dur if active_dur > 0 else 0.0

    print("1. CANONICAL WINDOW & SYNCHRONIZATION")
    print(f"   - Window Source          : {window_src}")
    print(f"   - Active Duration        : {active_dur:.2f} s {'[FLAG: SHORT DURATION]' if is_short else ''}")
    print(f"   - Active GT Samples      : {gt_act_samples} samples ({eff_gt_fps:.1f} Hz)")
    print(f"   - Active VO Frames       : {vo_act_samples} frames ({eff_vo_fps:.1f} Hz)")

    # 2. GT Timestamp & Derivative Integrity Audit
    dt_gt_all = np.diff(gt_t)
    dt_gt_act = np.diff(df_gt_act['timestamp_total_sec'].values) if 'timestamp_total_sec' in df_gt_act.columns else np.diff(df_gt_act['timestamp'].values)
    
    print("\n2. GT TIMESTAMP INTEGRITY & CLEAN DERIVATIVES")
    print(f"   - GT dt Stats (Active)   : Min={np.min(dt_gt_act)*1000:.3f}ms, Median={np.median(dt_gt_act)*1000:.2f}ms, P95={np.percentile(dt_gt_act,95)*1000:.2f}ms, Max={np.max(dt_gt_act)*1000:.2f}ms")
    count_dt_lt_2ms = np.sum(dt_gt_act < 0.002)
    pct_dt_lt_2ms = (count_dt_lt_2ms / len(dt_gt_act) * 100.0) if len(dt_gt_act) > 0 else 0.0
    print(f"   - Sub-2ms Jitter Bursts  : {count_dt_lt_2ms} intervals ({pct_dt_lt_2ms:.2f}%) [Excluded from derivatives]")

    # Compute clean spatial derivatives over active window
    t_act = df_gt_act['timestamp_total_sec'].values if 'timestamp_total_sec' in df_gt_act.columns else df_gt_act['timestamp'].values
    x_act = df_gt_act['pos_x'].values if 'pos_x' in df_gt_act.columns else df_gt_act['x'].values
    y_act = df_gt_act['pos_y'].values if 'pos_y' in df_gt_act.columns else df_gt_act['y'].values
    z_act = df_gt_act['pos_z'].values if 'pos_z' in df_gt_act.columns else df_gt_act['z'].values

    dx_act = np.diff(x_act)
    dy_act = np.diff(y_act)
    dz_act = np.diff(z_act)
    ds_act = np.sqrt(dx_act**2 + dy_act**2 + dz_act**2)

    v_clean, v_stats = compute_clean_derivatives(t_act, np.pad(np.cumsum(ds_act), (1, 0)), is_angle=False)

    # Compute Euler angles with explicit frame-aware terminology
    if 'rot_x' in df_gt_act.columns:
        qx = df_gt_act['rot_x'].values
        qy = df_gt_act['rot_y'].values
        qz = df_gt_act['rot_z'].values
        qw = df_gt_act['rot_w'].values
        
        enu_roll_rad, enu_pitch_rad, enu_yaw_rad = euler_from_quaternion(qx, qy, qz, qw)
        enu_roll_deg = np.degrees(enu_roll_rad)
        enu_pitch_deg = np.degrees(enu_pitch_rad)
        enu_yaw_deg = np.degrees(enu_yaw_rad)

        _, wz_stats = compute_clean_derivatives(t_act, enu_yaw_rad, is_angle=True)
        _, wroll_stats = compute_clean_derivatives(t_act, enu_roll_rad, is_angle=True)
        _, wpitch_stats = compute_clean_derivatives(t_act, enu_pitch_rad, is_angle=True)
    else:
        enu_roll_deg = enu_pitch_deg = enu_yaw_deg = np.zeros_like(x_act)
        wz_stats = wroll_stats = wpitch_stats = {'mean': 0.0, 'p95': 0.0, 'max': 0.0}

    print(f"   - Clean Speed (m/s)      : Mean={v_stats['mean']:.2f}, Median={v_stats['median']:.2f}, P95={v_stats['p95']:.2f}, Max={v_stats['max']:.2f}")
    print(f"   - Clean Yaw Rate (deg/s) : Mean={abs(wz_stats['mean']):.2f}, P95={abs(wz_stats['p95']):.2f}, Max={abs(wz_stats['max']):.2f}")

    # 3. Frame-Aware Attitude Terminology & Excursion Bounds
    print("\n3. FRAME-AWARE ATTITUDE ENVELOPE (GAZEBO WORLD ENU)")
    print(f"   - ENU Roll  (rot around Gazebo X) : Min/Max = [{enu_roll_deg.min():.1f}°, {enu_roll_deg.max():.1f}°], Peak Amp = {np.abs(enu_roll_deg).max():.1f}°")
    print(f"   - ENU Pitch (rot around Gazebo Y) : Min/Max = [{enu_pitch_deg.min():.1f}°, {enu_pitch_deg.max():.1f}°], Peak Amp = {np.abs(enu_pitch_deg).max():.1f}°")
    print(f"   - ENU Yaw   (rot around Gazebo Z) : Min/Max = [{enu_yaw_deg.min():.1f}°, {enu_yaw_deg.max():.1f}°]")

    # Check for P03/P04 frame terminology notes
    if family == 'F4' or 'P03' in pilot_name:
        print("   * [NOTE ON P03 F4]: Body-roll setpoint excitation oscillates along PX4 Local +Y (Gazebo +X). In Gazebo ENU Euler angles, tilt around Y-axis is logged as ENU PITCH.")
    elif family == 'F5' or 'P04' in pilot_name:
        print("   * [NOTE ON P04 F5]: Body-pitch setpoint excitation oscillates along PX4 Local +X (Gazebo +Y). In Gazebo ENU Euler angles, tilt around X-axis is logged as ENU ROLL.")

    # 4. VO Support Metrics & Pose Validity
    print("\n4. RECOMPUTED VO-SUPPORT METRICS & POSE VALIDITY")
    if len(df_vo_act) > 0:
        num_E = df_vo_act['num_inliers_E'].values if 'num_inliers_E' in df_vo_act.columns else df_vo_act['num_inliers'].values
        num_pose = df_vo_act['num_inliers_pose'].values if 'num_inliers_pose' in df_vo_act.columns else np.zeros(len(df_vo_act))
        matched = df_vo_act['num_matched'].values

        valid_pose_count = int(np.sum(num_pose >= 8))
        valid_pose_pct = (valid_pose_count / len(df_vo_act) * 100.0)

        e_ratio = np.where(matched > 0, num_E / matched, 0.0)
        pose_e_ratio = np.where(num_E > 0, num_pose / num_E, 0.0)
        survival = df_vo_act['feature_survival_rate'].astype(float).values
        feat_vel = df_vo_act['feature_vel_mean'].astype(float).values
        lk_err = df_vo_act['mean_lk_err'].astype(float).values

        # Monocular translation magnitude label check
        tx = df_vo_act['rel_tx'].astype(float).values
        ty = df_vo_act['rel_ty'].astype(float).values
        tz = df_vo_act['rel_tz'].astype(float).values
        unit_t_mag = np.sqrt(tx**2 + ty**2 + tz**2)

        print(f"   - Valid Pose Updates (N_pose >= 8): {valid_pose_count}/{len(df_vo_act)} ({valid_pose_pct:.1f}%)")
        print(f"   - Essential Matrix Inliers (N_E)   : Mean = {num_E.mean():.1f}, Median = {np.median(num_E):.1f}")
        print(f"   - Pose Inliers (N_pose)            : Mean = {num_pose.mean():.1f}, Median = {np.median(num_pose):.1f}")
        print(f"   - E Inlier Ratio (N_E / N_m)        : Mean = {e_ratio.mean():.3f}, Median = {np.median(e_ratio):.3f}")
        print(f"   - Pose/E Agreement Ratio (N_p / N_E): Mean = {pose_e_ratio.mean():.3f}, Median = {np.median(pose_e_ratio):.3f}")
        print(f"   - Feature Survival Rate (S_f)      : Mean = {survival.mean():.3f}, Median = {np.median(survival):.3f}")
        print(f"   - Feature Velocity (v_px)          : Mean = {feat_vel.mean():.2f} px/fr, P95 = {np.percentile(feat_vel, 95):.2f} px/fr")
        print(f"   - Mean LK Residual (e_lk)          : Mean = {lk_err.mean():.2f} px, Median = {np.median(lk_err):.2f} px")
        print(f"   - Monocular Unit-Scale Trans Mag   : Mean = {unit_t_mag.mean():.3f} (Unit vectors ||t||=1.0, NOT metric meters)")

        if family == 'F6' or 'P05' in pilot_name:
            print("   * [DIAGNOSTIC FLAG]: P05 Pure Yaw is a rotation-degeneracy diagnostic condition. High E-inliers do NOT indicate metric translation recovery.")
    else:
        print("   - [WARNING]: No VO frames matched active window!")

    # 5. GT-Referenced Frame-to-Frame Rotation Error
    if len(df_vo_act) > 1 and 'rot_x' in df_gt.columns:
        # Interpolate GT orientation onto VO timestamps for overlapping active interval
        vo_act_rel = df_vo_act['timestamp_total_sec'].values - df_vo['timestamp_total_sec'].iloc[0]
        gt_full_rel = gt_t - gt_t[0]

        fqx = interp1d(gt_full_rel, df_gt['rot_x'].values, bounds_error=False, fill_value='extrapolate')
        fqy = interp1d(gt_full_rel, df_gt['rot_y'].values, bounds_error=False, fill_value='extrapolate')
        fqz = interp1d(gt_full_rel, df_gt['rot_z'].values, bounds_error=False, fill_value='extrapolate')
        fqw = interp1d(gt_full_rel, df_gt['rot_w'].values, bounds_error=False, fill_value='extrapolate')

        q_raw = np.column_stack([fqx(vo_act_rel), fqy(vo_act_rel), fqz(vo_act_rel), fqw(vo_act_rel)])
        q_norm = q_raw / np.linalg.norm(q_raw, axis=1, keepdims=True)

        r_gt_interp = R_scipy.from_quat(q_norm)
        rel_r_gt = r_gt_interp[:-1].inv() * r_gt_interp[1:]
        rot_mag_gt_deg = rel_r_gt.magnitude() * (180.0 / np.pi)

        vo_rel_rot_deg = df_vo_act['rel_rot_deg'].astype(float).values[1:]
        rot_err_deg = np.abs(vo_rel_rot_deg - rot_mag_gt_deg)

        print("\n5. GT-REFERENCED FRAME-TO-FRAME ROTATION ERROR")
        print(f"   - Rotation Error (deg/step) : Mean = {rot_err_deg.mean():.2f}°, Median = {np.median(rot_err_deg):.2f}°, P95 = {np.percentile(rot_err_deg, 95):.2f}°")

    print("--------------------------------------------------------------------------")
    print(f"OVERALL STATUS: {'PASS WITH FLAGS' if is_short else 'SUCCESSFUL PASS'}")
    print("==========================================================================")
    return True


def main():
    parser = argparse.ArgumentParser(description="Repaired Phase 1 Telemetry Analysis Engine")
    parser.add_argument('--gt-csv', type=str, required=True, help="Path to GT CSV file")
    parser.add_argument('--vo-csv', type=str, required=True, help="Path to VO CSV file")
    parser.add_argument('--family', type=str, default='F1', help="Motion family ID")
    parser.add_argument('--severity', type=int, default=2, help="Severity level")
    parser.add_argument('--pilot', type=str, default='', help="Pilot identifier (e.g. P01)")
    args = parser.parse_args()

    res = analyze_flight_run(args.gt_csv, args.vo_csv, family=args.family, severity=args.severity, pilot_name=args.pilot)
    sys.exit(0 if res else 1)


if __name__ == '__main__':
    main()
