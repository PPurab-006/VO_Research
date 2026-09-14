#!/usr/bin/env python3
"""
Infrastructure Soak Test Runner for Phase 1 Flight Lifecycle

Validates flight execution lifecycle without aggressive maneuvers:
- Launches PX4 SITL (gz_x500_mono_cam) in agriculture.world at spawn 14.0505,-7.5229,0.1076
- Launches ros_gz_bridge & record_ground_truth.py
- Calls fly_phase1_motion.py --family HOVER --duration 20.0
- Verifies takeoff readiness, >=20s airborne hover duration, geofence, and clean landing
- Saves GT to results/phase1_soak_test_gt.csv and VO to results/phase1_soak_test_vo.csv
"""

import os
import sys
import time
import subprocess

WORLD_DIR = "/home/purab/Purab/Projects/ROS/configs/gazebo_maps"
WORLD_NAME = "default"
PX4_DIR = "/home/purab/PX4-Autopilot"
MODEL = "gz_x500_mono_cam"
SPAWN_POSE = "14.0505,-7.5229,0.1076,0,0,0"


def kill_all_sim_processes():
    print("[CLEANUP] Terminating existing simulation processes...")
    proc_names = ["gz-sim-main", "gz-sim-gui-client", "parameter_bridge", "px4", "ruby", "gz"]
    for p_name in proc_names:
        subprocess.run(["pkill", "-15", "-f", p_name], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(1.5)
    for p_name in proc_names:
        subprocess.run(["pkill", "-9", "-f", p_name], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(1.0)


def main():
    gt_file = "results/phase1_soak_test_gt.csv"
    vo_file = "results/phase1_soak_test_vo.csv"

    print("==========================================================================================")
    print("PHASE 1 INFRASTRUCTURE SOAK TEST RUNNER (20s Airborne Hover Check)")
    print("==========================================================================================")
    print(f"World       : agriculture.world")
    print(f"Spawn Pose  : {SPAWN_POSE}")
    print(f"GT Log File : {gt_file}")
    print(f"VO Log File : {vo_file}")
    print("------------------------------------------------------------------------------------------")

    os.makedirs("results", exist_ok=True)
    kill_all_sim_processes()

    maps_base = "/home/purab/Purab/Projects/ROS/configs/gazebo_maps"
    collection_models = "/home/purab/Purab/Projects/ROS/configs/gazebo_models_worlds_collection-master/models"
    common_models = f"{maps_base}/common_models"

    # Set default.sdf symlink to agriculture.world
    subprocess.run(["ln", "-sf", f"{maps_base}/agriculture.world", f"{PX4_DIR}/Tools/simulation/gz/worlds/default.sdf"])

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
    print(f"[1/6] Launching PX4 SITL (World: agriculture.world, Spawn: {SPAWN_POSE})...")
    px4_cmd = ["make", "-C", PX4_DIR, "px4_sitl", MODEL]
    px4_proc = subprocess.Popen(px4_cmd, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    # 2. Wait for Gazebo camera topic
    print("[2/6] Waiting for Gazebo camera topic...")
    cam_detected = False
    active_world_name = "default"
    for _ in range(45):
        try:
            out = subprocess.check_output(["gz", "topic", "-l"], stderr=subprocess.DEVNULL).decode('utf-8')
            for line in out.splitlines():
                if "camera/image" in line:
                    cam_detected = True
                    parts = line.strip().split('/')
                    if len(parts) > 2 and parts[1] == 'world':
                        active_world_name = parts[2]
                    print(f"      Gazebo camera topic detected on world '{active_world_name}'!")
                    break
            if cam_detected:
                break
        except Exception:
            pass
        time.sleep(1.0)

    if not cam_detected:
        print("[ERROR] Gazebo camera topic timed out! Cleaning up...")
        kill_all_sim_processes()
        sys.exit(1)

    # 3. Launch ros_gz_bridge
    print(f"[3/6] Launching ros_gz_bridge for world '{active_world_name}'...")
    bridge_cmd = [
        "ros2", "run", "ros_gz_bridge", "parameter_bridge",
        "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        f"/world/{active_world_name}/model/x500_mono_cam_0/link/camera_link/sensor/camera/image@sensor_msgs/msg/Image@gz.msgs.Image",
        f"/world/{active_world_name}/model/x500_mono_cam_0/link/camera_link/sensor/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
        f"/world/{active_world_name}/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V"
    ]
    bridge_proc = subprocess.Popen(bridge_cmd, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(2.0)

    # 4. Launch GT pose recorder
    print(f"[4/6] Launching record_ground_truth.py -> {gt_file}...")
    gt_cmd = [
        "python3", "src/record_ground_truth.py",
        "--topic", f"/world/{active_world_name}/dynamic_pose/info",
        "--output-dir", "results",
        "--filename", "phase1_soak_test_gt.csv",
        "--max-samples", "3000"
    ]
    gt_proc = subprocess.Popen(gt_cmd, env=env)
    time.sleep(1.0)

    # 5. Launch VO node & Motion Generator (HOVER, 20.0s)
    print("[5/6] Launching minimal_vo.py & Motion Generator for HOVER (20s)...")
    vo_cmd = [
        "python3", "src/minimal_vo.py",
        "--camera-topic", f"/world/{active_world_name}/model/x500_mono_cam_0/link/camera_link/sensor/camera/image",
        "--output-dir", "results",
        "--output-filename", "phase1_soak_test_vo.csv",
        "--max-frames", "2000",
        "--mode", "klt"
    ]
    vo_proc = subprocess.Popen(vo_cmd, env=env)
    time.sleep(1.5)

    motion_cmd = [
        "python3", "src/fly_phase1_motion.py",
        "--family", "HOVER",
        "--severity", "0",
        "--duration", "20.0"
    ]
    motion_res = subprocess.run(motion_cmd, env=env)

    print("\n[6/6] Flight complete. Waiting 3.0s for VO node to finish processing remaining frames...")
    time.sleep(3.0)

    # Cleanup processes
    kill_all_sim_processes()

    # Run Telemetry Analysis for Soak Test
    print("\nRunning GT Telemetry & Kinematics Analysis for Soak Test...")
    analysis_cmd = [
        "python3", "src/analyze_phase1_gt.py",
        "--gt-csv", gt_file,
        "--vo-csv", vo_file,
        "--family", "HOVER",
        "--severity", "0"
    ]
    analysis_res = subprocess.run(analysis_cmd)

    if analysis_res.returncode == 0:
        print("\n>>> INFRASTRUCTURE SOAK TEST PASSED VERIFICATION! <<<")
    else:
        print("\n>>> INFRASTRUCTURE SOAK TEST REQUIRES REVIEW <<<")


if __name__ == '__main__':
    main()
