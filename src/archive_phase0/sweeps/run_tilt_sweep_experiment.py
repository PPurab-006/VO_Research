#!/usr/bin/env python3
"""
Phase 0E Sweep B Experiment Orchestration Script

Runs all 6 oscillating tilt/roll amplitude levels (5, 10, 20, 30, 40, 50 deg) sequentially in boxworld_obstacles_tight.
Ensures clean 2-stage process lifecycle cleanup (SIGTERM -> SIGKILL) between runs.
Saves GT and VO trajectory logs to results/tilt_sweep_<AMP>deg_gt.csv and results/tilt_sweep_<AMP>deg_vo.csv.
"""

import os
import sys
import time
import subprocess

TILT_AMPS = [5, 10, 20, 30, 40, 50]
WORLD_DIR = str(Path(__file__).resolve().parents[3] / "configs" / "gazebo_maps")
WORLD_NAME = "default"
PX4_DIR = os.environ.get("PX4_DIR", str(Path.home() / "PX4-Autopilot"))
MODEL = "gz_x500_mono_cam"
SPAWN_POSE = "14.0505,-7.5229,0.1076,0,0,0"


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


def run_single_level(tilt_amp):
    print("\n" + "=" * 80)
    print(f"STARTING PHASE 0E SWEEP B LEVEL: {tilt_amp} DEG TARGET ATTITUDE AMPLITUDE (agriculture.world)")
    print("=" * 80)

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

    # 1. Launch PX4 SITL in background
    print(f"[1/5] Launching PX4 SITL (World: agriculture.world, Spawn: {SPAWN_POSE})...")
    px4_cmd = ["make", "-C", PX4_DIR, "px4_sitl", MODEL]
    px4_proc = subprocess.Popen(px4_cmd, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    # 2. Wait for Gazebo camera topic to appear
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
        return False

    # 3. Launch ros_gz_bridge in background
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

    # Output file paths
    gt_csv = f"results/tilt_sweep_v3_{tilt_amp}deg_gt.csv"
    vo_csv = f"results/tilt_sweep_v3_{tilt_amp}deg_vo.csv"
    telem_csv = f"results/tilt_sweep_v3_{tilt_amp}deg_telemetry.csv"

    # 4. Launch Ground Truth Recorder & Minimal VO Node concurrently
    print(f"[4/5] Launching record_ground_truth.py -> {gt_csv} and minimal_vo.py -> {vo_csv}...")
    gt_cmd = [
        "python3", "src/record_ground_truth.py",
        "--topic", f"/world/{active_world_name}/dynamic_pose/info",
        "--output-dir", "results",
        "--filename", f"tilt_sweep_v3_{tilt_amp}deg_gt.csv",
        "--max-samples", "3000"
    ]
    gt_proc = subprocess.Popen(gt_cmd, env=env)

    vo_cmd = [
        "python3", "src/minimal_vo.py",
        "--camera-topic", f"/world/{active_world_name}/model/x500_mono_cam_0/link/camera_link/sensor/camera/image",
        "--output-dir", "results",
        "--output-filename", f"tilt_sweep_v3_{tilt_amp}deg_vo.csv",
        "--max-frames", "2000",
        "--mode", "klt"
    ]
    vo_proc = subprocess.Popen(vo_cmd, env=env)

    time.sleep(2.5)

    # 5. Execute Oscillating Attitude Flight Script (fly_roll_validation.py architecture)
    print(f"[5/5] Executing fly_roll_validation.py for {tilt_amp} deg target amplitude (duration 20.0s)...")
    fly_cmd = [
        "python3", "src/fly_roll_validation.py",
        "--roll-amp-deg", str(tilt_amp),
        "--freq-hz", "0.5",
        "--maneuver-duration", "20.0",
        "--target-x", "14.0505",
        "--target-y", "-7.5229",
        "--target-z", "2.4076",
        "--telem-filename", f"tilt_sweep_v3_{tilt_amp}deg_telemetry.csv"
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
    print(f"[COMPLETE] Level {tilt_amp} deg target tilt amplitude finished successfully.\n")
    return True


def main():
    print("==========================================================================================")
    print("PHASE 0E — SWEEP B: OSCILLATING TILT/ROLL ESCALATION EXPERIMENT RUNNER")
    print("==========================================================================================")

    os.makedirs("results", exist_ok=True)

    success_count = 0
    for amp in TILT_AMPS:
        if run_single_level(amp):
            success_count += 1

    print("==========================================================================================")
    print(f"SWEEP B COMPLETE: {success_count} / {len(TILT_AMPS)} tilt amplitude levels recorded cleanly.")
    print("==========================================================================================")


if __name__ == '__main__':
    main()
