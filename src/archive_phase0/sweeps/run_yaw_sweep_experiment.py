#!/usr/bin/env python3
"""
Phase 0E Sweep A Experiment Orchestration Script

Runs all 6 pure yaw-rate levels (10, 20, 40, 80, 120, 180 deg/s) sequentially in boxworld_obstacles_tight.
Ensures clean 2-stage process lifecycle cleanup (SIGTERM -> SIGKILL) between runs.
Saves GT and VO trajectory logs to results/yaw_sweep_<LEVEL>dps_gt.csv and results/yaw_sweep_<LEVEL>dps_vo.csv.
"""

import os
import sys
import time
import subprocess
import signal

YAW_RATES = [10, 20, 40, 80, 120, 180]
WORLD_NAME = "boxworld_obstacles_tight"
PX4_DIR = "/home/purab/PX4-Autopilot"
MODEL = "gz_x500_mono_cam"


def kill_all_sim_processes():
    """2-stage process lifecycle cleanup (SIGTERM -> SIGKILL) for clean isolation between runs."""
    print("[CLEANUP] Terminating any existing PX4/Gazebo/ROS simulation processes...")
    proc_names = ["gz-sim-main", "gz-sim-gui-client", "parameter_bridge", "px4", "ruby", "gz"]
    for p_name in proc_names:
        subprocess.run(["pkill", "-15", "-f", p_name], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(1.5)
    for p_name in proc_names:
        subprocess.run(["pkill", "-9", "-f", p_name], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(1.0)


def run_single_level(yaw_rate):
    print("\n" + "=" * 80)
    print(f"STARTING PHASE 0E SWEEP A LEVEL: {yaw_rate} DEG/S YAW RATE")
    print("=" * 80)

    kill_all_sim_processes()

    # Prepare environment variables with GPU offload & world selection
    env = os.environ.copy()
    env["__NV_PRIME_RENDER_OFFLOAD"] = "1"
    env["__GLX_VENDOR_LIBRARY_NAME"] = "nvidia"
    env["GZ_SIM_RENDER_ENGINE"] = "ogre2"
    env["PX4_GZ_WORLD"] = WORLD_NAME
    env["PYTHONUNBUFFERED"] = "1"

    # 1. Launch PX4 SITL in background
    print(f"[1/5] Launching PX4 SITL (World: {WORLD_NAME}, GPU Offload)...")
    px4_cmd = ["make", "-C", PX4_DIR, "px4_sitl", MODEL]
    px4_proc = subprocess.Popen(px4_cmd, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    # 2. Wait for Gazebo camera topic to appear
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
        print("[ERROR] Gazebo camera topic timed out! Cleaning up...")
        kill_all_sim_processes()
        return False

    # 3. Launch ros_gz_bridge in background
    print("[3/5] Launching ros_gz_bridge...")
    bridge_cmd = [
        "ros2", "run", "ros_gz_bridge", "parameter_bridge",
        "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        f"/world/{WORLD_NAME}/model/x500_mono_cam_0/link/camera_link/sensor/camera/image@sensor_msgs/msg/Image@gz.msgs.Image",
        f"/world/{WORLD_NAME}/model/x500_mono_cam_0/link/camera_link/sensor/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
        f"/world/{WORLD_NAME}/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V"
    ]
    bridge_proc = subprocess.Popen(bridge_cmd, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(2.0)

    # Output file paths
    gt_csv = f"results/yaw_sweep_v3_{yaw_rate}dps_gt.csv"
    vo_csv = f"results/yaw_sweep_v3_{yaw_rate}dps_vo.csv"
    telem_csv = f"results/yaw_sweep_v3_{yaw_rate}dps_telemetry.csv"

    # 4. Launch Ground Truth Recorder & Minimal VO Node concurrently
    print(f"[4/5] Launching record_ground_truth.py -> {gt_csv} and minimal_vo.py -> {vo_csv}...")
    gt_cmd = [
        "python3", "src/record_ground_truth.py",
        "--topic", f"/world/{WORLD_NAME}/dynamic_pose/info",
        "--output-dir", "results",
        "--filename", f"yaw_sweep_v3_{yaw_rate}dps_gt.csv",
        "--max-samples", "3000"
    ]
    gt_proc = subprocess.Popen(gt_cmd, env=env)

    vo_cmd = [
        "python3", "src/minimal_vo.py",
        "--camera-topic", f"/world/{WORLD_NAME}/model/x500_mono_cam_0/link/camera_link/sensor/camera/image",
        "--output-dir", "results",
        "--output-filename", f"yaw_sweep_v3_{yaw_rate}dps_vo.csv",
        "--max-frames", "2000",
        "--mode", "klt"
    ]
    vo_proc = subprocess.Popen(vo_cmd, env=env)

    time.sleep(2.5)

    # 5. Execute Pure Yaw-Rate Flight Script
    print(f"[5/5] Executing fly_yaw_sweep.py for {yaw_rate} deg/s (duration 15.0s)...")
    fly_cmd = [
        "python3", "src/fly_yaw_sweep.py",
        "--yaw-rate-dps", str(yaw_rate),
        "--duration", "15.0",
        "--target-x", "5.0",
        "--target-y", "5.0",
        "--alt-z", "2.5",
        "--telem-filename", f"yaw_sweep_v3_{yaw_rate}dps_telemetry.csv"
    ]
    subprocess.run(fly_cmd, env=env)

    print("      Flight completed. Waiting for VO & GT logging to flush...")
    time.sleep(3.0)

    # Clean up level processes
    for proc in [gt_proc, vo_proc, bridge_proc, px4_proc]:
        try:
            proc.terminate()
        except Exception:
            pass

    time.sleep(2.0)
    kill_all_sim_processes()
    print(f"[COMPLETE] Level {yaw_rate} deg/s finished successfully.\n")
    return True


def main():
    print("==========================================================================================")
    print("PHASE 0E — SWEEP A: PURE YAW-RATE ESCALATION EXPERIMENT RUNNER")
    print("==========================================================================================")

    os.makedirs("results", exist_ok=True)

    success_count = 0
    for rate in YAW_RATES:
        if run_single_level(rate):
            success_count += 1

    print("==========================================================================================")
    print(f"SWEEP A COMPLETE: {success_count} / {len(YAW_RATES)} yaw-rate levels recorded cleanly.")
    print("==========================================================================================")


if __name__ == '__main__':
    main()
