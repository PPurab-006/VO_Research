#!/usr/bin/env python3
"""
Trajectory Command Script for PX4 SITL via MAVLink (Phase 0)

Commands PX4 SITL to:
1. Arm vehicle
2. Take off to 2.5m altitude
3. Stream offboard setpoints for smooth forward flight along +X with a gentle curve
"""

import math
import time
import sys
import threading
from pymavlink import mavutil

def main():
    print("Connecting to PX4 SITL on udp:127.0.0.1:14540...")
    master = mavutil.mavlink_connection('udp:127.0.0.1:14540')

    print("Waiting for heartbeat...")
    master.wait_heartbeat(timeout=30)
    target_sys = master.target_system
    target_comp = 1  # Autopilot component ID
    print(f"Heartbeat received from system {target_sys}, component {target_comp}")

    # Start background GCS heartbeat thread
    stop_hb = False
    def heartbeat_loop():
        while not stop_hb:
            master.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0, 0, 0
            )
            time.sleep(1.0)

    hb_thread = threading.Thread(target=heartbeat_loop, daemon=True)
    hb_thread.start()
    time.sleep(1.5)

    # Set SITL failsafe bypass parameters
    def set_p(name, val):
        master.mav.param_set_send(target_sys, target_comp, name.encode('utf-8'), val, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        time.sleep(0.05)

    set_p('NAV_DLL_ACT', 0)
    set_p('NAV_RCL_ACT', 0)
    set_p('COM_RCL_EXCEPT', 4)
    set_p('COM_ARM_WO_GPS', 1)
    set_p('CBRK_SUPPLY_CHK', 894565)
    time.sleep(0.5)

    # 1. Arm
    print("Arming drone...")
    master.mav.command_long_send(
        target_sys, target_comp,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )
    time.sleep(1.0)

    # 2. Takeoff to 2.0m
    print("Sending Takeoff command (target altitude: 2.0m)...")
    master.mav.command_long_send(
        target_sys, target_comp,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, 2.0
    )
    time.sleep(6.0)

    # 3. Stream velocity setpoints (Forward flight along +X at 0.6 m/s, slight yaw rate)
    print("Streaming trajectory setpoints (Forward flight + slight curve)...")
    start_time = time.time()
    duration = 10.0  # seconds

    type_mask = 0b0000111111000111  # Ignore pos, accel, use vx, vy, vz, yaw_rate

    while time.time() - start_time < duration:
        t = time.time() - start_time
        # Forward speed 0.6 m/s, slight curve after 3s
        vx = 0.6
        vy = 0.15 if t > 3.0 else 0.0
        vz = 0.0
        yaw_rate = 0.05 if t > 3.0 else 0.0

        master.mav.set_position_target_local_ned_send(
            int((time.time() - start_time) * 1000),
            target_sys, target_comp,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            type_mask,
            0, 0, 0,           # x, y, z positions
            vx, vy, vz,        # vx, vy, vz velocities
            0, 0, 0,           # ax, ay, az accelerations
            0, yaw_rate        # yaw, yaw_rate
        )
        time.sleep(0.05)

    stop_hb = True
    print("Trajectory complete.")

if __name__ == '__main__':
    main()
