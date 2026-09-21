#!/usr/bin/env python3
"""
Orchestration Script for Single Phase 1 Trajectory Run in agriculture.world

Executes full pipeline for one pilot trajectory:
- Sets up environment & PX4 SITL (gz_x500_mono_cam) at spawn pose 14.0505,-7.5229,0.1076
- Launches ros_gz_bridge & gt_pose_recorder.py
- Streams motion family setpoints via fly_phase1_motion.py
- Runs minimal_vo.py to log VO performance
- Analyzes GT telemetry and VO metrics via analyze_phase1_gt.py
"""

import argparse
import os
import sys
import time
import subprocess

WORLD_DIR = str(Path(__file__).resolve().parents[3] / "configs" / "gazebo_maps")
WORLD_NAME = "default"
PX4_DIR = os.environ.get("PX4_DIR", str(Path.home() / "PX4-Autopilot"))
MODEL = "gz_x500_mono_cam"
SPAWN_POSE = "14.0505,-7.5229,0.1076,0,0,0"


def kill_all_sim_processes():
    print("[CLEANUP] Terminating existing simulation processes...")
    proc_names = ["gz-sim-main", "gz-sim-gui-client", "parameter_bridge", "px4", "ruby", "gz", "record_ground_truth", "minimal_vo"]
    for p_name in proc_names:
        subprocess.run(["pkill", "-15", "-f", p_name], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(1.5)
    for p_name in proc_names:
        subprocess.run(["pkill", "-9", "-f", p_name], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(1.0)


def main():
    parser = argparse.ArgumentParser(description="Phase 1 Trajectory Orchestrator")
    parser.add_argument('--family', type=str, required=True, help="Motion family ID (e.g. F1, F2, F4, F5, F6, F9, F10, F11)")
    parser.add_argument('--severity', type=int, default=2, help="Severity level (0..5)")
    parser.add_argument('--duration', type=float, default=20.0, help="Flight duration in seconds")
    parser.add_argument('--run-id', type=str, default=None, help="Custom unique run ID (e.g. confirmation_001_HOVER_L0_R1)")
    args = parser.parse_args()

    family = args.family
    severity = args.severity
    duration = args.duration
    run_id = args.run_id if args.run_id else f"phase1_pilot_{family}_L{severity}"

    gt_file = f"results/{run_id}_gt.csv"
    vo_file = f"results/{run_id}_vo.csv"

    print("==========================================================================================")
    print(f"PHASE 1 TRAJECTORY ORCHESTRATOR — Family: {family}, Level: L{severity}")
    print("==========================================================================================")
    print(f"GT Log File : {gt_file}")
    print(f"VO Log File : {vo_file}")
    print("------------------------------------------------------------------------------------------")

    os.makedirs("results", exist_ok=True)
    kill_all_sim_processes()

    maps_base = "src/configs/gazebo_maps"
    collection_models = "src/configs/gazebo_models_worlds_collection-master/models"
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
        "--filename", f"{run_id}_gt.csv",
        "--max-samples", "3000"
    ]
    gt_proc = subprocess.Popen(gt_cmd, env=env)
    time.sleep(1.0)

    # 5. Launch VO node & Motion Generator concurrently
    print(f"[5/6] Launching minimal_vo.py & Motion Generator for {family} at L{severity}...")
    vo_cmd = [
        "python3", "src/minimal_vo.py",
        "--camera-topic", f"/world/{active_world_name}/model/x500_mono_cam_0/link/camera_link/sensor/camera/image",
        "--output-dir", "results",
        "--output-filename", f"{run_id}_vo.csv",
        "--max-frames", "2000",
        "--mode", "klt"
    ]
    vo_proc = subprocess.Popen(vo_cmd, env=env)
    time.sleep(1.5)

    motion_cmd = [
        "python3", "src/fly_phase1_motion.py",
        "--family", family,
        "--severity", str(severity),
        "--duration", str(duration)
    ]
    motion_res = subprocess.run(motion_cmd, env=env)

    print("\n[6/6] Flight complete. Waiting 3.0s for VO node to finish processing remaining frames...")
    time.sleep(3.0)

    # Safely terminate background nodes
    for p in [gt_proc, vo_proc, bridge_proc, px4_proc]:
        try:
            if p and p.poll() is None:
                p.terminate()
        except Exception:
            pass

    # Cleanup processes
    kill_all_sim_processes()

    # Run Telemetry & VO Analysis
    print("\nRunning GT Telemetry & Kinematics Analysis...")
    analysis_cmd = [
        "python3", "src/analyze_phase1_gt.py",
        "--gt-csv", gt_file,
        "--vo-csv", vo_file,
        "--family", family,
        "--severity", str(severity)
    ]
    analysis_res = subprocess.run(analysis_cmd)

    if analysis_res.returncode == 0:
        print(f"\n>>> PILOT TRAJECTORY {family} L{severity} PASSED VERIFICATION! <<<")
    else:
        print(f"\n>>> PILOT TRAJECTORY {family} L{severity} REQUIRING REVIEW <<<")


if __name__ == '__main__':
    main()
