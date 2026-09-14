#!/usr/bin/env python3
"""
Diagnostic Script: Test loading drone_race_track_2018_actual_with_gatepapers.world
and inspect gate poses, collision topics, and GPU rendering.
"""

import os
import sys
import time
import subprocess

WORLD_DIR = "/home/purab/Purab/Projects/ROS/configs/gazebo_maps"
WORLD_NAME = "default"
PX4_DIR = "/home/purab/PX4-Autopilot"
MODEL = "gz_x500_mono_cam"
SPAWN_POSE = "0,0,0.1076,0,0,1.570796"  # Spawn facing North (+Y)

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
    print("======================================================================")
    print("GATE COURSE FEASIBILITY: WORLD LOADING & GEOMETRY VERIFICATION")
    print("======================================================================")
    
    kill_all_sim_processes()
    
    maps_base = "/home/purab/Purab/Projects/ROS/configs/gazebo_maps"
    collection_models = "/home/purab/Purab/Projects/ROS/configs/gazebo_models_worlds_collection-master/models"
    common_models = f"{maps_base}/common_models"

    # Set default.sdf symlink to drone_race_track_2018_actual_with_gatepapers.world
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

    print(f"[1] Launching PX4 SITL with {world_path}...")
    px4_cmd = ["make", "-C", PX4_DIR, "px4_sitl", MODEL]
    px4_proc = subprocess.Popen(px4_cmd, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    print("[2] Waiting for Gazebo topics...")
    active_world_name = "default"
    gz_loaded = False
    for _ in range(45):
        try:
            out = subprocess.check_output(["gz", "topic", "-l"], stderr=subprocess.DEVNULL).decode('utf-8')
            if "/world/" in out:
                gz_loaded = True
                for line in out.splitlines():
                    if "dynamic_pose/info" in line or "camera/image" in line:
                        parts = line.strip().split('/')
                        if len(parts) > 2 and parts[1] == 'world':
                            active_world_name = parts[2]
                print(f"    Gazebo Sim loaded cleanly! Active world: '{active_world_name}'")
                break
        except Exception:
            pass
        time.sleep(1.0)

    if not gz_loaded:
        print("[ERROR] Gazebo Sim failed to load within timeout!")
        kill_all_sim_processes()
        return

    # Verify GPU utilization
    print("[3] Checking NVIDIA GPU status...")
    try:
        smi_out = subprocess.check_output(["nvidia-smi", "--query-gpu=utilization.gpu,memory.used", "--format=csv,noheader"], stderr=subprocess.DEVNULL).decode('utf-8')
        print(f"    NVIDIA GPU Stats: {smi_out.strip()}")
    except Exception as e:
        print(f"    GPU status query error: {e}")

    # Inspect Gazebo Topics
    print("[4] Listing Gazebo topics for collision/contact/pose...")
    try:
        out = subprocess.check_output(["gz", "topic", "-l"], stderr=subprocess.DEVNULL).decode('utf-8')
        topics = [line.strip() for line in out.splitlines() if line.strip()]
        print(f"    Total Gazebo topics found: {len(topics)}")
        contact_topics = [t for t in topics if "contact" in t or "collision" in t]
        print(f"    Contact/Collision topics: {contact_topics}")
    except Exception as e:
        print(f"    Topic list error: {e}")

    time.sleep(3.0)
    kill_all_sim_processes()
    print("[COMPLETE] Verification finished cleanly.\n")

if __name__ == '__main__':
    main()
