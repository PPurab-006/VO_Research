#!/usr/bin/env python3
"""
Offboard Pure Yaw-Rate Hover Script for PX4 SITL via MAVLink (Phase 0E - Sweep A)

Executes in-place hover at fixed position (5.0, 5.0, 2.5m) in boxworld_obstacles_tight
while streaming a constant pure yaw rate setpoint (10, 20, 40, 80, 120, 180 deg/s).
"""

import argparse
import math
import sys
import time
from pymavlink import mavutil


def main():
    parser = argparse.ArgumentParser(description="Phase 0E — Pure Yaw-Rate Hover Sweep")
    parser.add_argument('--yaw-rate-dps', type=float, default=10.0, help="Commanded pure yaw rate in deg/s")
    parser.add_argument('--duration', type=float, default=15.0, help="Flight duration in seconds")
    parser.add_argument('--target-x', type=float, default=5.0, help="Hover X position in meters")
    parser.add_argument('--target-y', type=float, default=5.0, help="Hover Y position in meters")
    parser.add_argument('--alt-z', type=float, default=2.5, help="Hover altitude Z in meters")
    parser.add_argument('--telem-filename', type=str, default=None, help="Telemetry output CSV filename")
    args = parser.parse_args()

    yaw_rate_dps = args.yaw_rate_dps
    omega_rad_s = math.radians(yaw_rate_dps)
    duration = args.duration
    target_x = args.target_x
    target_y = args.target_y
    alt_z = args.alt_z

    print("============================================================")
    print(f"PHASE 0E SWEEP A: PURE YAW RATE {yaw_rate_dps:.1f} DEG/S HOVER")
    print("============================================================")
    print(f"Target Hover Position : ({target_x:.1f}, {target_y:.1f}, {alt_z:.1f} m)")
    print(f"Commanded Yaw Rate    : {yaw_rate_dps:.1f} deg/s ({omega_rad_s:.4f} rad/s)")
    print(f"Flight Duration       : {duration:.1f} s")
    print("------------------------------------------------------------")

    # Connect to PX4 SITL MAVLink UDP port
    mav_addr = "udpin:0.0.0.0:14540"
    print(f"Connecting to PX4 via MAVLink at {mav_addr}...")
    master = mavutil.mavlink_connection(mav_addr)
    master.wait_heartbeat()
    target_sys = master.target_system
    target_comp = master.target_component
    print(f"Heartbeat received! (SysID: {target_sys}, CompID: {target_comp})")

    # Function to send SET_POSITION_TARGET_LOCAL_NED setpoint
    def send_setpoint(x, y, z, vx=0.0, vy=0.0, vz=0.0, yaw_rate=0.0):
        # type_mask = 1507 (0x05E3): Position X, Y, Z + Velocity VX, VY, VZ + Yaw Rate
        master.mav.set_position_target_local_ned_send(
            int(time.time() * 1000) & 0xFFFFFFFF,
            target_sys, target_comp,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            1507,
            x, y, -z,          # x, y, z position (NED: -z = +Up)
            vx, vy, vz,        # vx, vy, vz velocity
            0.0, 0.0, 0.0,     # acceleration
            0.0, yaw_rate      # yaw (rad), yaw_rate (rad/s)
        )

    # 1. Pre-stream setpoints for 1.5s
    print("1. Pre-streaming OFFBOARD setpoints for 1.5s...")
    pre_start = time.time()
    while time.time() - pre_start < 1.5:
        send_setpoint(target_x, target_y, alt_z, yaw_rate=omega_rad_s)
        time.sleep(0.05)

    # 2. Arm Vehicle
    print("2. Arming vehicle...")
    master.mav.command_long_send(
        target_sys, target_comp,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 21968, 0, 0, 0, 0, 0
    )
    time.sleep(0.2)

    # 3. Request OFFBOARD Mode
    print("3. Requesting OFFBOARD mode...")
    master.mav.command_long_send(
        target_sys, target_comp,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        0, 1, 6, 0, 0, 0, 0, 0
    )
    time.sleep(0.5)

    # 4. Verify OFFBOARD Mode Lock
    print("4. Verifying vehicle mode transition...")
    mode_verified = False
    for _ in range(25):
        send_setpoint(target_x, target_y, alt_z, yaw_rate=omega_rad_s)
        hb = master.recv_match(type='HEARTBEAT', blocking=True, timeout=0.2)
        if hb and hb.get_srcSystem() == target_sys and hb.get_srcComponent() == target_comp:
            main_m = (hb.custom_mode >> 16) & 0xFF
            is_offboard = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_GUIDED_ENABLED) or (main_m == 6)
            if is_offboard:
                mode_verified = True
                print("   [MODE VERIFIED] Drone is cleanly locked in OFFBOARD mode!")
                break

    if not mode_verified:
        print("[WARNING] Could not confirm OFFBOARD mode verification from heartbeat. Proceeding anyway...")

    # 5. Stream Pure Yaw Rate Hover Setpoint for Duration
    print(f"5. Streaming in-place hover with yaw_rate = {yaw_rate_dps:.1f} deg/s for {duration:.1f}s...")
    flight_start = time.time()
    while time.time() - flight_start < duration:
        send_setpoint(target_x, target_y, alt_z, yaw_rate=omega_rad_s)
        time.sleep(0.05)

    # 6. Request LAND
    print("6. Yaw sweep flight complete. Requesting LAND...")
    master.mav.command_long_send(
        target_sys, target_comp,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0, 0, 0, 0, 0, 0, 0, 0
    )
    time.sleep(3.0)
    print(f"OFFBOARD Pure Yaw-Rate Sweep ({yaw_rate_dps:.1f} deg/s) Complete.")


if __name__ == '__main__':
    main()
