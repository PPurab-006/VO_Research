#!/usr/bin/env python3
"""
Recording Orchestration Script for Phase 2B Candidate 4 First Look

Trajectory: Candidate 4 (Yaw Sweep + Sustained Forward Translation)
  - Family: F9 (Level 2)
  - Duration: 20.0s after takeoff readiness
  - World: agriculture.world
  - Output Dataset: results/datasets/phase2b_candidate4_L2_R1

Records 1280x960 unwarped PNG camera frames and synchronized GT attitude telemetry.
"""

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
    proc_names = ["gz-sim-main", "gz-sim-gui-client", "parameter_bridge", "px4", "ruby", "gz", "record_camera_dataset", "minimal_vo"]
    for p_name in proc_names:
        subprocess.run(["pkill", "-15", "-f", p_name], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(1.5)
    for p_name in proc_names:
        subprocess.run(["pkill", "-9", "-f", p_name], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(1.0)


def record_candidate4():
    run_id = "phase2b_candidate4_L2_R1"
    dataset_dir = f"results/datasets/{run_id}"

    print("==========================================================================")
    print("PHASE 2B DATASET RECORDER — Candidate 4 (Yaw Sweep + Sustained Translation)")
    print("==========================================================================")
    print(f"Run ID      : {run_id}")
    print(f"Dataset Dir : {dataset_dir}")
    print(f"World       : agriculture.world")
    print("--------------------------------------------------------------------------")

    os.makedirs(dataset_dir, exist_ok=True)
    kill_all_sim_processes()

    maps_base = "src/configs/gazebo_maps"
    collection_models = "src/configs/gazebo_models_worlds_collection-master/models"
    common_models = f"{maps_base}/common_models"

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
    print(f"[1/5] Launching PX4 SITL (World: agriculture.world, Model: {MODEL})...")
    px4_cmd = ["make", "-C", PX4_DIR, "px4_sitl", MODEL]
    px4_proc = subprocess.Popen(px4_cmd, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    # 2. Wait for Gazebo camera topic
    print("[2/5] Waiting for Gazebo camera topic...")
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
    print(f"[3/5] Launching ros_gz_bridge for world '{active_world_name}'...")
    bridge_cmd = [
        "ros2", "run", "ros_gz_bridge", "parameter_bridge",
        "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        f"/world/{active_world_name}/model/x500_mono_cam_0/link/camera_link/sensor/camera/image@sensor_msgs/msg/Image@gz.msgs.Image",
        f"/world/{active_world_name}/model/x500_mono_cam_0/link/camera_link/sensor/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
        f"/world/{active_world_name}/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V"
    ]
    bridge_proc = subprocess.Popen(bridge_cmd, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(2.0)

    # 4. Launch dataset recorder
    print(f"[4/5] Launching record_camera_dataset.py -> {dataset_dir}...")
    rec_cmd = [
        "python3", "src/record_camera_dataset.py",
        "--cam-topic", f"/world/{active_world_name}/model/x500_mono_cam_0/link/camera_link/sensor/camera/image",
        "--pose-topic", f"/world/{active_world_name}/dynamic_pose/info",
        "--output-dir", dataset_dir
    ]
    rec_proc = subprocess.Popen(rec_cmd, env=env)
    time.sleep(1.5)

    # 5. Launch Motion Generator (F9 L2 = Yaw Sweep + Translation)
    print(f"[5/5] Executing Candidate 4 Motion Profile (F9 L2) for 20.0s...")
    motion_cmd = [
        "python3", "src/fly_phase1_motion.py",
        "--family", "F9",
        "--severity", "2",
        "--duration", "20.0"
    ]
    subprocess.run(motion_cmd, env=env)

    print("\nFlight complete. Waiting 2.0s to finish writing dataset files...")
    time.sleep(2.0)

    # Safely terminate background processes
    for p in [rec_proc, bridge_proc, px4_proc]:
        try:
            if p and p.poll() is None:
                p.terminate()
        except Exception:
            pass

    kill_all_sim_processes()
    print(f"\n>>> CANDIDATE 4 DATASET '{run_id}' RECORDING COMPLETE! <<<")


if __name__ == '__main__':
    record_candidate4()
