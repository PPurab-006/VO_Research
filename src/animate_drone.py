#!/usr/bin/env python3
"""
Smooth Gazebo Trajectory Controller for Mono Camera (Phase 0C)

Streams smooth 6-DoF pose setpoints to Gazebo via /world/textured/set_pose
at ~30 Hz, matching the exact camera framerate to generate smooth motion.
"""

import math
import subprocess
import sys
import time

def move_step(x, y, z, yaw=0.0, world="textured"):
    qx = 0.0
    qy = 0.0
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)

    req = f'name: "x500_mono_cam_0", position: {{x: {x:.4f}, y: {y:.4f}, z: {z:.4f}}}, orientation: {{x: {qx:.4f}, y: {qy:.4f}, z: {qz:.4f}, w: {qw:.4f}}}'
    cmd = [
        "gz", "service",
        "-s", f"/world/{world}/set_pose",
        "--reqtype", "gz.msgs.Pose",
        "--reptype", "gz.msgs.Boolean",
        "--timeout", "500",
        "--req", req
    ]
    subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

def main():
    world = sys.argv[1] if len(sys.argv) > 1 else "textured"
    print(f"Starting smooth trajectory stream on Gazebo world: '{world}'...")

    total_steps = 250
    dt = 0.033  # ~30 Hz

    for i in range(total_steps):
        t = i * dt
        # Forward motion along +X (0 to 6 meters)
        x = (i / total_steps) * 6.0
        # Gentle lateral curve along +Y
        y = 0.8 * math.sin(2.0 * math.pi * (i / total_steps))
        # Altitude fixed at 2.0 meters above ground
        z = 2.0
        # Slight heading angle (yaw)
        yaw = 0.1 * math.cos(2.0 * math.pi * (i / total_steps))

        move_step(x, y, z, yaw, world=world)
        time.sleep(dt)

    print("Smooth trajectory stream completed.")

if __name__ == '__main__':
    main()
