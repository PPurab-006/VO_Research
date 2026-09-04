#!/usr/bin/env python3
"""
Test script to verify roll oscillation takeoff & GT achieved roll angle
"""

import os
import sys
import time
import math
import subprocess
import csv
import numpy as np
from pymavlink import mavutil
from scipy.spatial.transform import Rotation as R_scipy

PX4_DIR = "/home/purab/PX4-Autopilot"
WORLD_NAME = "boxworld_obstacles_tight"
MODEL = "gz_x500_mono_cam"

def kill_all_sim_processes():
    proc_names = ["gz-sim-main", "gz-sim-gui-client", "parameter_bridge", "px4", "ruby", "gz"]
    for p_name in proc_names:
        subprocess.run(["pkill", "-15", "-f", p_name], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(1.5)
    for p_name in proc_names:
        subprocess.run(["pkill", "-9", "-f", p_name], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(1.0)

def euler_to_quat_wxyz(roll_rad, pitch_rad, yaw_rad):
    r = R_scipy.from_euler('xyz', [roll_rad, pitch_rad, yaw_rad])
    q_xyzw = r.as_quat()
    return [q_xyzw[3], q_xyzw[0], q_xyzw[1], q_xyzw[2]]

def main():
    print("[1/5] Cleaning up old processes & starting PX4 SITL...")
    kill_all_sim_processes()

    env = os.environ.copy()
    env["__NV_PRIME_RENDER_OFFLOAD"] = "1"
    env["__GLX_VENDOR_LIBRARY_NAME"] = "nvidia"
    env["GZ_SIM_RENDER_ENGINE"] = "ogre2"
    env["PX4_GZ_WORLD"] = WORLD_NAME
    env["PYTHONUNBUFFERED"] = "1"

    px4_proc = subprocess.Popen(["make", "-C", PX4_DIR, "px4_sitl", MODEL], env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    print("[2/5] Waiting for Gazebo camera topic...")
    cam_detected = False
    for _ in range(40):
        try:
            out = subprocess.check_output(["gz", "topic", "-l"], stderr=subprocess.DEVNULL).decode('utf-8')
            if "camera/image" in out:
                cam_detected = True
                print("      Gazebo camera topic detected!")
                break
        except Exception:
            pass
        time.sleep(1.0)

    if not cam_detected:
        print("[ERROR] Camera topic timeout!")
        kill_all_sim_processes()
        return

    print("[3/5] Launching ros_gz_bridge & ground truth recorder...")
    bridge_cmd = [
        "ros2", "run", "ros_gz_bridge", "parameter_bridge",
        "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        f"/world/{WORLD_NAME}/model/x500_mono_cam_0/link/camera_link/sensor/camera/image@sensor_msgs/msg/Image@gz.msgs.Image",
        f"/world/{WORLD_NAME}/model/x500_mono_cam_0/link/camera_link/sensor/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
        f"/world/{WORLD_NAME}/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V"
    ]
    bridge_proc = subprocess.Popen(bridge_cmd, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(2.0)

    gt_csv = "results/test_tilt_gt.csv"
    gt_cmd = [
        "python3", "src/record_ground_truth.py",
        "--topic", f"/world/{WORLD_NAME}/dynamic_pose/info",
        "--output-dir", "results",
        "--filename", "test_tilt_gt.csv",
        "--max-samples", "3000"
    ]
    gt_proc = subprocess.Popen(gt_cmd, env=env)
    time.sleep(2.0)

    print("[4/5] Connecting to PX4 via MAVLink...")
    master = mavutil.mavlink_connection("udpin:0.0.0.0:14540")
    master.wait_heartbeat()
    target_sys = master.target_system
    target_comp = master.target_component
    print("      Heartbeat received!")

    def send_pos_target(x, y, z, yaw_rate=0.0):
        master.mav.set_position_target_local_ned_send(
            int(time.time() * 1000) & 0xFFFFFFFF,
            target_sys, target_comp,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            1507, # 0x05E3: Position X, Y, Z + Velocity VX, VY, VZ + Yaw Rate
            x, y, -z,
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.0, yaw_rate
        )

    def send_att_target(roll_deg, pitch_deg=0.0, yaw_deg=0.0, thrust=0.58):
        q = euler_to_quat_wxyz(math.radians(roll_deg), math.radians(pitch_deg), math.radians(yaw_deg))
        master.mav.set_attitude_target_send(
            int(time.time() * 1000) & 0xFFFFFFFF,
            target_sys, target_comp,
            7, # ignore body rates
            q,
            0.0, 0.0, 0.0,
            thrust
        )

    # 1. Pre-stream position setpoint for 1.5s
    print("      Pre-streaming position setpoint for 1.5s...")
    pre = time.time()
    while time.time() - pre < 1.5:
        send_pos_target(5.0, 5.0, 2.5)
        time.sleep(0.05)

    # 2. Arm & OFFBOARD
    print("      Arming & requesting OFFBOARD mode...")
    master.mav.command_long_send(target_sys, target_comp, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 21968, 0, 0, 0, 0, 0)
    time.sleep(0.2)
    master.mav.command_long_send(target_sys, target_comp, mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, 1, 6, 0, 0, 0, 0, 0)
    time.sleep(0.5)

    # 3. Takeoff & Climb to 2.5m position
    print("      Climbing to hover altitude (5.0, 5.0, 2.5m) for 8.0s...")
    climb_start = time.time()
    while time.time() - climb_start < 8.0:
        send_pos_target(5.0, 5.0, 2.5)
        time.sleep(0.05)

    # 4. Roll Oscillation Phase (20 deg amplitude at 0.5 Hz for 15.0s)
    print("      Executing 20 deg roll oscillation at 0.5 Hz for 15.0s with thrust compensation...")
    osc_start = time.time()
    hover_thrust = 0.58
    while time.time() - osc_start < 15.0:
        t_elapsed = time.time() - osc_start
        target_roll = 20.0 * math.sin(2.0 * math.pi * 0.5 * t_elapsed)
        cos_roll = max(0.3, math.cos(math.radians(target_roll)))
        comp_thrust = min(0.9, hover_thrust / cos_roll)
        send_att_target(target_roll, pitch_deg=0.0, yaw_deg=0.0, thrust=comp_thrust)
        time.sleep(0.02)

    # 5. Land
    print("      Flight complete. Requesting LAND...")
    master.mav.command_long_send(target_sys, target_comp, mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0)
    time.sleep(3.0)

    gt_proc.terminate()
    bridge_proc.terminate()
    px4_proc.terminate()
    kill_all_sim_processes()

    # Analyze GT recorded roll angles
    print("\n[5/5] Analyzing GT Recorded Flight Data...")
    if os.path.exists(gt_csv):
        with open(gt_csv) as f:
            gt_rows = list(csv.DictReader(f))
        print(f"      Total GT Rows Logged: {len(gt_rows)}")
        act_gt = [r for r in gt_rows if float(r['pos_z']) > 0.3]
        print(f"      Active Flight GT Rows (z>0.3): {len(act_gt)}")
        
        rolls = []
        pitches = []
        yaws = []
        zs = []
        for r in act_gt:
            q = [float(r['rot_x']), float(r['rot_y']), float(r['rot_z']), float(r['rot_w'])]
            e = R_scipy.from_quat(q).as_euler('xyz', degrees=True)
            rolls.append(e[0])
            pitches.append(e[1])
            yaws.append(e[2])
            zs.append(float(r['pos_z']))

        print(f"      GT Altitude (z) : Min = {np.min(zs):.2f}m, Max = {np.max(zs):.2f}m, Mean = {np.mean(zs):.2f}m")
        print(f"      GT Roll Angle   : Min = {np.min(rolls):.2f} deg, Max = {np.max(rolls):.2f} deg, Peak Amp = {max(abs(np.min(rolls)), abs(np.max(rolls))):.2f} deg")
        print(f"      GT Pitch Angle  : Min = {np.min(pitches):.2f} deg, Max = {np.max(pitches):.2f} deg")
        print(f"      GT Yaw Angle    : Min = {np.min(yaws):.2f} deg, Max = {np.max(yaws):.2f} deg")
    else:
        print("[ERROR] GT file not found!")

if __name__ == '__main__':
    main()
