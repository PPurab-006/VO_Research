#!/usr/bin/env python3
"""
Offboard Oscillating Tilt/Roll Hover Script for PX4 SITL via MAVLink (Phase 0E - Sweep B)

Executes in-place hover at fixed position (5.0, 5.0, 2.5m) in boxworld_obstacles_tight
while streaming a 0.5Hz sinusoidal velocity setpoint to induce closed-loop tilt/roll angle
oscillation at escalating target amplitudes (5, 10, 20, 30, 40, 50 deg).
"""

import argparse
import math
import sys
import time
from pymavlink import mavutil


def main():
    parser = argparse.ArgumentParser(description="Phase 0E — Oscillating Tilt/Roll Hover Sweep B")
    parser.add_argument('--tilt-amp-deg', type=float, default=10.0, help="Target tilt/roll angle amplitude in degrees")
    parser.add_argument('--freq-hz', type=float, default=0.5, help="Oscillation frequency in Hz")
    parser.add_argument('--duration', type=float, default=20.0, help="Flight duration in seconds")
    parser.add_argument('--target-x', type=float, default=5.0, help="Hover X position in meters")
    parser.add_argument('--target-y', type=float, default=5.0, help="Hover Y position in meters")
    parser.add_argument('--alt-z', type=float, default=2.5, help="Hover altitude Z in meters")
    args = parser.parse_args()

    tilt_amp_deg = args.tilt_amp_deg
    freq_hz = args.freq_hz
    duration = args.duration
    target_x = args.target_x
    target_y = args.target_y
    alt_z = args.alt_z

    g = 9.8066
    v_amp = (g * math.tan(math.radians(tilt_amp_deg))) / (2.0 * math.pi * freq_hz)

    print("============================================================")
    print(f"PHASE 0E SWEEP B: OSCILLATING TILT/ROLL {tilt_amp_deg:.1f} DEG HOVER")
    print("============================================================")
    print(f"Target Hover Position : ({target_x:.1f}, {target_y:.1f}, {alt_z:.1f} m)")
    print(f"Target Tilt Amplitude : {tilt_amp_deg:.1f} deg at {freq_hz:.2f} Hz")
    print(f"Commanded V_amp       : {v_amp:.3f} m/s")
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
    def send_setpoint(x, y, z, vy=0.0, yaw=0.0):
        # type_mask = 1475 (0x05C3): Position X, Y, Z + Velocity VY + Yaw
        master.mav.set_position_target_local_ned_send(
            int(time.time() * 1000) & 0xFFFFFFFF,
            target_sys, target_comp,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            1475,
            x, y, -z,
            0.0, vy, 0.0,
            0.0, 0.0, 0.0,
            yaw, 0.0
        )

    # 1. Pre-stream setpoints for 1.5s
    print("1. Pre-streaming OFFBOARD setpoints for 1.5s...")
    pre_start = time.time()
    while time.time() - pre_start < 1.5:
        send_setpoint(target_x, target_y, alt_z)
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

    # 4. Verify OFFBOARD Mode & Climb to Altitude
    print("4. Climbing to hover altitude (2.5m) for 6.0s...")
    climb_start = time.time()
    while time.time() - climb_start < 6.0:
        send_setpoint(target_x, target_y, alt_z)
        time.sleep(0.05)

    # 5. Stream Oscillating Tilt Setpoint for Duration
    print(f"5. Streaming oscillating velocity setpoint (V_amp = {v_amp:.3f} m/s @ {freq_hz:.2f} Hz) for {duration:.1f}s...")
    flight_start = time.time()
    while time.time() - flight_start < duration:
        t_el = time.time() - flight_start
        vy_cmd = v_amp * math.cos(2.0 * math.pi * freq_hz * t_el)
        send_setpoint(target_x, target_y, alt_z, vy=vy_cmd)
        time.sleep(0.02)

    # 6. Request LAND
    print("6. Tilt sweep flight complete. Requesting LAND...")
    master.mav.command_long_send(
        target_sys, target_comp,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0, 0, 0, 0, 0, 0, 0, 0
    )
    time.sleep(3.0)
    print(f"OFFBOARD Tilt/Roll Sweep ({tilt_amp_deg:.1f} deg) Complete.")


if __name__ == '__main__':
    main()
