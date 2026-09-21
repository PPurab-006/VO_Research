#!/usr/bin/env python3
"""
Minimal launcher for ARM/OFFBOARD diagnostic in gate course world.
Launches PX4 SITL + Gazebo, waits for topics, then runs diagnose_gate_arm_failure.py.
"""

import os
import sys
import time
import subprocess

WORLD_DIR = str(Path(__file__).resolve().parents[3] / "configs" / "gazebo_maps")
PX4_DIR = os.environ.get("PX4_DIR", str(Path.home() / "PX4-Autopilot"))
MODEL = "gz_x500_mono_cam"
SPAWN_POSE = "0,0,0.1076,0,0,1.570796"

def kill_all():
    proc_names = ["gz-sim-main", "gz-sim-gui-client", "parameter_bridge", "px4", "ruby", "gz"]
    for p in proc_names:
        subprocess.run(["pkill", "-15", "-f", p], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(1.5)
    for p in proc_names:
        subprocess.run(["pkill", "-9", "-f", p], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(1.0)

def main():
    print("=== ARM/OFFBOARD DIAGNOSTIC LAUNCHER ===")
    kill_all()

    maps_base = WORLD_DIR
    collection_models = "src/configs/gazebo_models_worlds_collection-master/models"
    common_models = f"{maps_base}/common_models"
    world_path = f"{maps_base}/drone_race_track_2018_actual_with_gatepapers.world"

    subprocess.run(["ln", "-sf", world_path, f"{PX4_DIR}/Tools/simulation/gz/worlds/default.sdf"])

    env = os.environ.copy()
    env["__NV_PRIME_RENDER_OFFLOAD"] = "1"
    env["__GLX_VENDOR_LIBRARY_NAME"] = "nvidia"
    env["GZ_SIM_RENDER_ENGINE"] = "ogre2"
    env["PX4_GZ_WORLDS"] = WORLD_DIR
    env["PX4_GZ_WORLD"] = "default"
    env["PX4_GZ_MODEL_POSE"] = SPAWN_POSE
    env["GZ_SIM_RESOURCE_PATH"] = f"{collection_models}:{common_models}:{WORLD_DIR}:{PX4_DIR}/Tools/simulation/gz/models"
    env["GAZEBO_MODEL_PATH"] = f"{collection_models}:{common_models}:{WORLD_DIR}"
    env["PYTHONUNBUFFERED"] = "1"

    # Launch PX4 SITL
    print(f"Launching PX4 SITL (spawn: {SPAWN_POSE})...")
    px4_cmd = ["make", "-C", PX4_DIR, "px4_sitl", MODEL]
    px4_proc = subprocess.Popen(px4_cmd, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    # Wait for Gazebo
    print("Waiting for Gazebo topics...")
    gz_detected = False
    for _ in range(45):
        try:
            out = subprocess.check_output(["gz", "topic", "-l"], stderr=subprocess.DEVNULL).decode('utf-8')
            if "/world/" in out:
                gz_detected = True
                print("Gazebo detected!")
                break
        except Exception:
            pass
        time.sleep(1.0)

    if not gz_detected:
        print("ERROR: Gazebo timed out!")
        kill_all()
        return

    # Wait 5s for PX4 EKF to initialize
    print("Waiting 5s for PX4 EKF initialization...")
    time.sleep(5.0)

    # Run diagnostic
    print("Running ARM/OFFBOARD diagnostic...")
    subprocess.run(["python3", "src/diagnose_gate_arm_failure.py"], env=env)

    # Cleanup
    kill_all()
    print("=== LAUNCHER COMPLETE ===")

if __name__ == '__main__':
    main()
