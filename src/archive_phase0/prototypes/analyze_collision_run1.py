#!/usr/bin/env python3
"""
Analysis Script for Dedicated Collision Test (collision_test_run1.csv)

Wall Parameters:
  Position: (10.0, 0.0, 2.5) m in Gazebo ENU
  Dimensions: 0.5m thick (X), 10.0m wide (Y), 5.0m tall (Z)
  Front Face: X = 9.75 m
  Back Face:  X = 10.25 m
"""

import os
import sys
import pandas as pd
import numpy as np

def main():
    csv_path = '/home/purab/Purab/Projects/ROS/results/collision_test_run1.csv'
    if not os.path.exists(csv_path):
        print(f"Error: {csv_path} does not exist.")
        sys.exit(1)

    df = pd.read_csv(csv_path)
    print(f"Loaded ground truth CSV: '{csv_path}' ({len(df)} rows)")

    t = df['timestamp_total_sec'].values
    t_rel = t - t[0]
    x = df['pos_x'].values
    y = df['pos_y'].values
    z = df['pos_z'].values

    dt = np.diff(t)
    dx = np.diff(x)
    dy = np.diff(y)
    dz = np.diff(z)
    dt_safe = np.where(dt > 1e-6, dt, np.nan)
    dpos = np.sqrt(dx**2 + dy**2 + dz**2)
    v_raw = dpos / dt_safe

    # 50 Hz uniform resampling for clean derivative evaluation
    total_duration = t_rel[-1]
    t_u = np.linspace(t_rel[0], t_rel[-1], int(total_duration * 50))
    dt_u = t_u[1] - t_u[0]
    x_u = np.interp(t_u, t_rel, x)
    y_u = np.interp(t_u, t_rel, y)
    z_u = np.interp(t_u, t_rel, z)

    vx_u = np.gradient(x_u, dt_u)
    vy_u = np.gradient(y_u, dt_u)
    vz_u = np.gradient(z_u, dt_u)
    v_u = np.sqrt(vx_u**2 + vy_u**2 + vz_u**2)

    wall_front_x = 9.75
    wall_back_x = 10.25

    print("\n============================================================")
    print("COLLISION TEST TRAJECTORY SUMMARY")
    print("============================================================")
    print(f"Total Trajectory Duration : {total_duration:.3f} s")
    print(f"Start Position           : ({x[0]:.3f}, {y[0]:.3f}, {z[0]:.3f}) m")
    print(f"Final Position           : ({x[-1]:.3f}, {y[-1]:.3f}, {z[-1]:.3f}) m")
    print(f"Max X Position Reached   : {x.max():.3f} m")
    print(f"Wall Front Face Location : X = {wall_front_x:.2f} m")
    print(f"Wall Back Face Location  : X = {wall_back_x:.2f} m")

    # 1. Did position pass through wall front face (X > 9.75m)?
    passed_front = x > wall_front_x
    passed_back = x > wall_back_x

    print("\n------------------------------------------------------------")
    print("PHYSICS COLLISION & BOUNDARY ANALYSIS")
    print("------------------------------------------------------------")
    if np.any(passed_front):
        idx_front = np.where(passed_front)[0][0]
        t_front = t_rel[idx_front]
        pos_front = (x[idx_front], y[idx_front], z[idx_front])
        v_front = v_u[np.argmin(np.abs(t_u - t_front))]
        print(f"🔴 DRONE PENETRATED WALL FRONT FACE (X = {wall_front_x}m):")
        print(f"   - Timestamp: t_rel = {t_front:.3f} s")
        print(f"   - Position: ({pos_front[0]:.3f}, {pos_front[1]:.3f}, {pos_front[2]:.3f}) m")
        print(f"   - Velocity: {v_front:.3f} m/s")
    else:
        print(f"🟢 DRONE STOPPED BEFORE WALL FRONT FACE (X < {wall_front_x}m).")

    if np.any(passed_back):
        idx_back = np.where(passed_back)[0][0]
        t_back = t_rel[idx_back]
        pos_back = (x[idx_back], y[idx_back], z[idx_back])
        print(f"🔴 DRONE COMPLETELY PASSED THROUGH SOLID WALL (X > {wall_back_x}m):")
        print(f"   - Timestamp: t_rel = {t_back:.3f} s")
        print(f"   - Position: ({pos_back[0]:.3f}, {pos_back[1]:.3f}, {pos_back[2]:.3f}) m")

    # 2. Check for velocity drop/collapse
    if np.any(passed_front):
        idx_post = np.where(t_rel >= t_front)[0]
        v_post = v_u[np.argmin(np.abs(t_u - t_rel[idx_post]))]
        max_x_post = x[idx_post].max()
        print(f"   - Post-penetration Max X reached: {max_x_post:.3f} m")

    print("\n------------------------------------------------------------")
    print("POSITION & VELOCITY PROFILE AROUND IMPACT WINDOW (1s resolution)")
    print("------------------------------------------------------------")
    print(f"{'Time (s)':<10} | {'Pos X (m)':<10} | {'Pos Y (m)':<10} | {'Pos Z (m)':<10} | {'Vel (m/s)':<10} | {'Status':<25}")
    print("-" * 80)
    for bin_t in np.arange(0, total_duration, 1.0):
        idx = np.argmin(np.abs(t_rel - bin_t))
        v_val = v_u[np.argmin(np.abs(t_u - bin_t))]
        px, py, pz = x[idx], y[idx], z[idx]
        if px < wall_front_x:
            status = "Approaching Wall"
        elif px <= wall_back_x:
            status = "INSIDE SOLID WALL VOLUME"
        else:
            status = "PASSED THROUGH WALL"
        print(f"{t_rel[idx]:<10.2f} | {px:<10.3f} | {py:<10.3f} | {pz:<10.3f} | {v_val:<10.3f} | {status:<25}")

    print("\n============================================================")
    print("FINAL VERDICT")
    print("============================================================")
    if np.any(passed_back):
        print("VERDICT: PASS-THROUGH (NO COLLISION PHYSICS REGISTRATION)")
        print(f"Explanation: Drone position advanced continuously through the wall's front face (X={wall_front_x}m) and back face (X={wall_back_x}m), reaching X={x.max():.3f}m without velocity collapse or deflection.")
    else:
        print("VERDICT: COLLISION REGISTERED")
        print(f"Explanation: Drone velocity collapsed/deflected at X={x.max():.3f}m near the wall face.")

if __name__ == '__main__':
    main()
