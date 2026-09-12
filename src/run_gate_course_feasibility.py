#!/usr/bin/env python3
"""
Orchestration & Post-Flight Analysis Script for Gate Course Feasibility Pass.

- Launches PX4 SITL + Gazebo with drone_race_track_2018_actual_with_gatepapers.world (GPU offload active).
- Launches GT recorder -> results/gate_feasibility_gt.csv (50 Hz). (NO VO node launched!).
- Executes src/fly_gate_course_feasibility.py.
- Performs 2-stage process cleanup (SIGTERM -> SIGKILL).
- Runs automated clearance, collision, cornering, and flight health analysis on GT telemetry.
"""

import os
import sys
import time
import math
import subprocess
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation as R_scipy

WORLD_DIR = "/home/purab/Purab/Projects/ROS/configs/gazebo_maps"
WORLD_NAME = "default"
PX4_DIR = "/home/purab/PX4-Autopilot"
MODEL = "gz_x500_mono_cam"
SPAWN_POSE = "0,0,0.1076,0,0,0"  # Spawning at (0,0,0.1) with standard zero-yaw spawn orientation

# Known 7 Gate Center Openings in World ENU (x, y, z, yaw_deg, description)
GATE_DEFINITIONS = {
    1: {"pos": (0.00,  3.50, 1.55), "yaw": 90.0, "name": "Gate 1"},
    2: {"pos": (0.00,  7.70, 1.55), "yaw": 90.0, "name": "Gate 2"},
    3: {"pos": (2.75, 10.45, 2.05), "yaw": 45.0, "name": "Gate 3"},
    4: {"pos": (4.90,  7.40, 1.55), "yaw": -90.0, "name": "Gate 4"},
    5: {"pos": (4.90,  2.60, 1.55), "yaw": -90.0, "name": "Gate 5"},
    6: {"pos": (1.10,  2.00, 2.95), "yaw": 180.0, "name": "Gate 6 (Elevated)"},
    7: {"pos": (2.00,  5.30, 1.45), "yaw": 90.0, "name": "Gate 7"}
}

def kill_all_sim_processes():
    print("[CLEANUP] Terminating existing simulation processes...")
    proc_names = ["gz-sim-main", "gz-sim-gui-client", "parameter_bridge", "px4", "ruby", "gz"]
    for p_name in proc_names:
        subprocess.run(["pkill", "-15", "-f", p_name], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(1.5)
    for p_name in proc_names:
        subprocess.run(["pkill", "-9", "-f", p_name], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(1.0)

def analyze_gt_telemetry(csv_path):
    print("\n======================================================================")
    print("POST-FLIGHT FEASIBILITY ANALYSIS REPORT: GT TELEMETRY")
    print("======================================================================")

    if not os.path.exists(csv_path):
        print(f"[ERROR] GT CSV file not found: {csv_path}")
        return

    df = pd.read_csv(csv_path)
    print(f"Total Logged GT Telemetry Frames : {len(df)}")
    if len(df) < 50:
        print("[ERROR] Insufficient GT data samples logged!")
        return

    # Columns written by record_ground_truth.py:
    # 'sample_idx', 'timestamp_sec', 'timestamp_nanosec', 'timestamp_total_sec', 'pos_x', 'pos_y', 'pos_z', 'rot_x', 'rot_y', 'rot_z', 'rot_w'
    if 'timestamp_total_sec' in df.columns:
        t = df['timestamp_total_sec'].values
    elif 'timestamp_sim' in df.columns:
        t = df['timestamp_sim'].values
    else:
        t = np.arange(len(df)) * 0.02

    dt = np.diff(t)
    dt[dt == 0] = 0.001  # Prevent zero div
    mean_fps = 1.0 / np.mean(dt) if len(dt) > 0 and np.mean(dt) > 0 else 0.0
    print(f"GT Telemetry Logging Rate       : {mean_fps:.2f} Hz (Duration: {t[-1]-t[0]:.2f}s)")

    # Extract drone positions
    x = df['pos_x'].values if 'pos_x' in df.columns else df['x'].values
    y = df['pos_y'].values if 'pos_y' in df.columns else df['y'].values
    z = df['pos_z'].values if 'pos_z' in df.columns else df['z'].values

    # Extract quaternions and compute Euler angles (roll, pitch, yaw) in degrees
    qx = df['rot_x'].values if 'rot_x' in df.columns else df['qx'].values
    qy = df['rot_y'].values if 'rot_y' in df.columns else df['qy'].values
    qz = df['rot_z'].values if 'rot_z' in df.columns else df['qz'].values
    qw = df['rot_w'].values if 'rot_w' in df.columns else df['qw'].values

    quats = np.column_stack([qx, qy, qz, qw])
    r = R_scipy.from_quat(quats)
    euler = r.as_euler('xyz', degrees=True)
    roll = euler[:, 0]
    pitch = euler[:, 1]
    yaw = euler[:, 2]

    # Velocities & Accelerations (finite differences)
    vx = np.gradient(x, t)
    vy = np.gradient(y, t)
    vz = np.gradient(z, t)
    v_mag = np.sqrt(vx**2 + vy**2 + vz**2)

    ax = np.gradient(vx, t)
    ay = np.gradient(vy, t)
    az = np.gradient(vz, t)
    a_mag = np.sqrt(ax**2 + ay**2 + az**2)

    wz = np.gradient(np.unwrap(np.radians(yaw)), t)

    # 1. COLLISION RECOIL CHECK (Abrupt Acceleration Spikes > 15 m/s^2)
    # Ignore initial takeoff acceleration (first 2.0s)
    takeoff_mask = t > (t[0] + 2.0)
    accel_spikes = np.where(takeoff_mask & (a_mag > 15.0))[0]
    collision_registered = False
    collision_timestamps = []
    if len(accel_spikes) > 0:
        collision_registered = True
        collision_timestamps = t[accel_spikes].tolist()

    print("\n----------------------------------------------------------------------")
    print("1. COLLISION CHECK (Physics Engine Recoil & Acceleration Discontinuity)")
    print("----------------------------------------------------------------------")
    print(f"Collision Registered            : {'YES (COLLISION DETECTED)' if collision_registered else 'NO (0 Collisions Registered)'}")
    if collision_registered:
        print(f"Collision Timestamps (SimTime)  : {collision_timestamps[:5]}...")

    # 2. CLEARANCE CHECK PER GATE (Explicit Transit Verification: Alignment with Gate Normal & Path Heading)
    print("\n-------------------------------------------------------------------------------------------------------------")
    print("2. GATE CLEARANCE & TRANSIT VERIFICATION (Closest Approach & Path Alignment)")
    print("-------------------------------------------------------------------------------------------------------------")
    print(f"{'Gate #':<8} {'Gate Name':<20} {'Min Dist (m)':<15} {'At Time (s)':<15} {'Drone Pos (x,y,z)':<22} {'Vel (m/s)':<12} {'Yaw (deg)':<10} {'Transit Evaluation':<22}")
    print("-" * 125)

    gate_normals = {
        1: (0.0, 1.0, 0.0),
        2: (0.0, 1.0, 0.0),
        3: (math.cos(math.radians(45)), math.sin(math.radians(45)), 0.0),
        4: (0.0, -1.0, 0.0),
        5: (0.0, -1.0, 0.0),
        6: (-1.0, 0.0, 0.0),
        7: (0.0, 1.0, 0.0)
    }

    gate_clearances = {}
    for g_id, g_info in GATE_DEFINITIONS.items():
        gx, gy, gz = g_info['pos']
        g_yaw = g_info['yaw']
        dists = np.sqrt((x - gx)**2 + (y - gy)**2 + (z - gz)**2)
        min_idx = np.argmin(dists)
        min_d = dists[min_idx]
        min_t = t[min_idx]
        px, py, pz = x[min_idx], y[min_idx], z[min_idx]
        
        # Velocity and Heading alignment at closest approach
        vx_c, vy_c, vz_c = vx[min_idx], vy[min_idx], vz[min_idx]
        speed_c = np.sqrt(vx_c**2 + vy_c**2 + vz_c**2)
        yaw_c = yaw[min_idx]
        
        nx, ny, nz = gate_normals[g_id]
        if speed_c > 0.05:
            dot_n = abs((vx_c * nx + vy_c * ny + vz_c * nz) / speed_c)
        else:
            dot_n = 0.0
            
        yaw_diff = abs((yaw_c - g_yaw + 180.0) % 360.0 - 180.0)
        
        passed = min_d < 0.65
        is_genuine = passed and (dot_n > 0.5) and (yaw_diff < 45.0)
        
        if passed:
            if is_genuine:
                transit_str = "GENUINE TRANSIT"
            else:
                transit_str = "COINCIDENTAL CROSSING"
        else:
            transit_str = "MISSED / WIDE" if min_d > 0.90 else "MARGINAL MISS"
            
        gate_clearances[g_id] = (min_d, min_t, passed, is_genuine)

        print(f"{g_id:<8} {g_info['name']:<20} {min_d:<15.3f} {min_t:<15.2f} ({px:.2f}, {py:.2f}, {pz:.2f}){' '*2} {speed_c:<12.2f} {yaw_c:<10.1f} {transit_str:<22}")

    # 3. ACHIEVED CORNERING BEHAVIOR AT HARD CORNERS
    print("\n----------------------------------------------------------------------")
    print("3. ACHIEVED CORNERING BEHAVIOR AT HARDEST TURNS")
    print("----------------------------------------------------------------------")

    # Hard Corner A: Gate 2 -> Gate 3 (Turn East-North around Gate 2 at y~7.7 to Gate 3 at x=2.75, y=10.45)
    g2_t = gate_clearances[2][1]
    g3_t = gate_clearances[3][1]
    c_mask_a = (t >= g2_t) & (t <= g3_t)
    if np.any(c_mask_a):
        max_wz_a = np.max(np.abs(np.degrees(wz[c_mask_a]))) if len(wz[c_mask_a])>0 else 0.0
        max_roll_a = np.max(np.abs(roll[c_mask_a]))
        max_pitch_a = np.max(np.abs(pitch[c_mask_a]))

        # Overshoot check: max Y reach between Gate 2 and Gate 3 relative to Gate 3 Y (10.45m)
        max_y_a = np.max(y[c_mask_a])
        overshoot_a = max(0.0, max_y_a - 10.45)

        print(f"Corner A (Gate 2 -> Gate 3 Turn):")
        print(f"  - Peak Yaw Rate               : {max_wz_a:.2f} deg/s")
        print(f"  - Peak Roll / Pitch Tilt      : Roll {max_roll_a:.2f} deg, Pitch {max_pitch_a:.2f} deg")
        print(f"  - Path Overshoot Beyond Gate 3: {overshoot_a:.3f} m")

    # Hard Corner B: Gate 5 -> Gate 6 -> Gate 7 Sequence
    g5_t = gate_clearances[5][1]
    g7_t = gate_clearances[7][1]
    c_mask_b = (t >= g5_t) & (t <= g7_t)
    if np.any(c_mask_b):
        max_wz_b = np.max(np.abs(np.degrees(wz[c_mask_b]))) if len(wz[c_mask_b])>0 else 0.0
        max_roll_b = np.max(np.abs(roll[c_mask_b]))
        max_pitch_b = np.max(np.abs(pitch[c_mask_b]))
        max_z_b = np.max(z[c_mask_b])

        print(f"Corner B (Gate 5 -> Gate 6 -> Gate 7 Climb-Turn Sequence):")
        print(f"  - Peak Yaw Rate               : {max_wz_b:.2f} deg/s")
        print(f"  - Peak Roll / Pitch Tilt      : Roll {max_roll_b:.2f} deg, Pitch {max_pitch_b:.2f} deg")
        print(f"  - Achieved Peak Altitude      : {max_z_b:.3f} m (Gate 6 Target: 2.95m)")

    # 4. GENERAL FLIGHT HEALTH
    print("\n----------------------------------------------------------------------")
    print("4. GENERAL FLIGHT HEALTH")
    print("----------------------------------------------------------------------")
    in_air_v = v_mag[takeoff_mask] if np.any(takeoff_mask) else v_mag
    in_air_a = a_mag[takeoff_mask] if np.any(takeoff_mask) else a_mag
    print(f"Achieved Mean Flight Speed      : {np.mean(in_air_v):.2f} m/s (Target: 0.60 m/s)")
    print(f"Max Flight Speed                : {np.max(in_air_v):.2f} m/s")
    print(f"Max In-Air Acceleration         : {np.max(in_air_a):.2f} m/s^2")
    print(f"Altitude Min / Max              : {np.min(z):.2f} m / {np.max(z):.2f} m")

    # 5. SEGMENT-BY-SEGMENT TRACKING BREAKDOWN (GT Telemetry)
    print("\n-------------------------------------------------------------------------------------------------------------")
    print("5. SEGMENT-BY-SEGMENT TRACKING BREAKDOWN (Ground Truth Trajectory)")
    print("-------------------------------------------------------------------------------------------------------------")
    print(f"{'Segment':<12} {'Segment Target ENU (x,y,z,yaw)':<38} {'Closest GT Pos ENU (x,y,z)':<28} {'Pos Error (m)':<15} {'Achieved Yaw (deg)':<20}")
    print("-" * 115)

    waypoints_list = [
        (0.00,  0.00, 1.55,  90.0, "wp0 (Takeoff)"),
        (0.00,  3.50, 1.55,  90.0, "wp1 (Gate 1)"),
        (0.00,  7.70, 1.55,  90.0, "wp2 (Gate 2)"),
        (2.75, 10.45, 2.05,  45.0, "wp3 (Gate 3)"),
        (4.90,  7.40, 1.55, -90.0, "wp4 (Gate 4)"),
        (4.90,  2.60, 1.55, -90.0, "wp5 (Gate 5)"),
        (1.10,  2.00, 2.95, 180.0, "wp6 (Gate 6)"),
        (2.00,  5.30, 1.45,  90.0, "wp7 (Gate 7)"),
        (2.00,  7.30, 1.45,  90.0, "wp8 (Post-Gate 7)")
    ]

    for i in range(len(waypoints_list) - 1):
        src_name = waypoints_list[i][4].split(' ')[0]
        dst_x, dst_y, dst_z, dst_yaw, dst_desc = waypoints_list[i+1]
        dst_name = dst_desc.split(' ')[0]
        seg_label = f"{src_name} -> {dst_name}"

        dists = np.sqrt((x - dst_x)**2 + (y - dst_y)**2 + (z - dst_z)**2)
        min_idx = np.argmin(dists)
        min_err = dists[min_idx]
        px, py, pz = x[min_idx], y[min_idx], z[min_idx]
        yaw_ach = yaw[min_idx]

        tgt_str = f"({dst_x:.2f}, {dst_y:.2f}, {dst_z:.2f}, {dst_yaw:.1f}°)"
        gt_str = f"({px:.2f}, {py:.2f}, {pz:.2f})"

        print(f"{seg_label:<12} {tgt_str:<38} {gt_str:<28} {min_err:<15.3f} {yaw_ach:<20.1f}")

    print("======================================================================\n")

def main():
    print("==========================================================================================")
    print("GATE COURSE FEASIBILITY PASS ORCHESTRATOR (drone_race_track_2018_actual_with_gatepapers)")
    print("==========================================================================================")

    os.makedirs("results", exist_ok=True)
    kill_all_sim_processes()

    maps_base = "/home/purab/Purab/Projects/ROS/configs/gazebo_maps"
    collection_models = "/home/purab/Purab/Projects/ROS/configs/gazebo_models_worlds_collection-master/models"
    common_models = f"{maps_base}/common_models"

    world_path = f"{maps_base}/drone_race_track_2018_actual_with_gatepapers.world"
    subprocess.run(["ln", "-sf", world_path, f"{PX4_DIR}/Tools/simulation/gz/worlds/default.sdf"])

    env = os.environ.copy()
    env["__NV_PRIME_RENDER_OFFLOAD"] = "1"
    env["__GLX_VENDOR_LIBRARY_NAME"] = "nvidia"
    env["GZ_SIM_RENDER_ENGINE"] = "ogre2"
    env["PX4_GZ_WORLDS"] = WORLD_DIR
    env["PX4_GZ_WORLD"] = WORLD_NAME
    env["PX4_GZ_MODEL_POSE"] = SPAWN_POSE
    env["GZ_SIM_RESOURCE_PATH"] = f"{collection_models}:{common_models}:{WORLD_DIR}:{PX4_DIR}/Tools/simulation/gz/models"
    env["GAZEBO_MODEL_PATH"] = f"{collection_models}:{common_models}:{WORLD_DIR}"
    env["PYTHONUNBUFFERED"] = "1"

    # 1. Launch PX4 SITL
    print(f"[1/4] Launching PX4 SITL (World: {world_path}, Spawn: {SPAWN_POSE})...")
    px4_cmd = ["make", "-C", PX4_DIR, "px4_sitl", MODEL]
    px4_proc = subprocess.Popen(px4_cmd, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    # 2. Wait for Gazebo camera/pose topics
    print("[2/4] Waiting for Gazebo topics...")
    gz_detected = False
    active_world_name = "default"
    for _ in range(45):
        try:
            out = subprocess.check_output(["gz", "topic", "-l"], stderr=subprocess.DEVNULL).decode('utf-8')
            if "/world/" in out:
                gz_detected = True
                for line in out.splitlines():
                    if "dynamic_pose/info" in line:
                        parts = line.strip().split('/')
                        if len(parts) > 2 and parts[1] == 'world':
                            active_world_name = parts[2]
                print(f"      Gazebo Sim detected on world '{active_world_name}'!")
                break
        except Exception:
            pass
        time.sleep(1.0)

    if not gz_detected:
        print("[ERROR] Gazebo Sim timed out! Cleaning up...")
        kill_all_sim_processes()
        return

    # 3. Launch ros_gz_bridge & GT recorder ONLY (NO VO NODE)
    print(f"[3/4] Launching ros_gz_bridge and record_ground_truth.py (NO VO)...")
    bridge_cmd = [
        "ros2", "run", "ros_gz_bridge", "parameter_bridge",
        "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        f"/world/{active_world_name}/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V"
    ]
    bridge_proc = subprocess.Popen(bridge_cmd, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(2.0)

    gt_csv = "results/gate_feasibility_gt.csv"
    gt_cmd = [
        "python3", "src/record_ground_truth.py",
        "--topic", f"/world/{active_world_name}/dynamic_pose/info",
        "--output-dir", "results",
        "--filename", "gate_feasibility_gt.csv",
        "--max-samples", "4000"
    ]
    gt_proc = subprocess.Popen(gt_cmd, env=env)
    time.sleep(2.5)

    # 4. Execute Gate Course Feasibility Flight Pass via PX4 Native Mission Mode
    print("[4/4] Executing src/fly_gate_course_mission.py (PX4 Native Mission Mode)...")
    fly_cmd = ["python3", "src/fly_gate_course_mission.py"]
    subprocess.run(fly_cmd, env=env)

    print("      Flight completed. Flushing GT telemetry log...")
    time.sleep(3.0)

    # Clean up processes
    for proc in [gt_proc, bridge_proc, px4_proc]:
        try:
            proc.terminate()
        except Exception:
            pass
    time.sleep(2.0)
    kill_all_sim_processes()

    # 5. Run Post-Flight Analysis
    analyze_gt_telemetry(gt_csv)

if __name__ == '__main__':
    main()
