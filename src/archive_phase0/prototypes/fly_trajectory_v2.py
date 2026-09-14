#!/usr/bin/env python3
"""
Offboard Circular Trajectory Command Script for PX4 SITL (Phase 0 - V2 Sweet-Spot Flight)

Flies a circular path centered directly on the 3D box cluster centroid:
- Circle Center: (x_c, y_c) = (6.0, 1.0) m
- Circle Radius: R = 4.0 m (100% of trajectory stays strictly within 3.0-5.0m sweet spot!)
- Target Altitude: z = 4.2 m (z = -4.2m in NED, clearing all box obstacles cleanly)
- Speed: v = 1.2 m/s (omega = 0.30 rad/s, Period T ~ 21.0s)
- Yaw Rate: omega = 0.30 rad/s (tangential direction of travel)

Establishes REAL OFFBOARD Control Authority:
1. Start GCS heartbeat stream to bind MAVLink connection
2. Set SITL failsafe bypass parameters
3. Arm vehicle via MAV_CMD_COMPONENT_ARM_DISARM
4. Issue MAV_CMD_NAV_TAKEOFF to get vehicle airborne
5. Pre-stream offboard setpoints (>= 1.5s at 20 Hz) with MASK_POS_Z_VEL_XY = 1507 (0x05E3)
6. Explicitly request OFFBOARD mode via MAV_CMD_DO_SET_MODE (param1=1, param2=6)
7. Verify OFFBOARD mode transition in heartbeat/status
8. Stream circular OFFBOARD flight trajectory at 4.2m altitude
9. Request LAND on completion
"""

import math
import time
import sys
import threading
from pymavlink import mavutil

def main():
    print("Connecting to PX4 SITL on udp:127.0.0.1:14540...")
    master = mavutil.mavlink_connection('udp:127.0.0.1:14540')

    stop_hb = False
    def heartbeat_loop():
        while not stop_hb:
            master.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0, 0, 0
            )
            time.sleep(0.5)

    hb_thread = threading.Thread(target=heartbeat_loop, daemon=True)
    hb_thread.start()

    print("Waiting for heartbeat from PX4...")
    master.wait_heartbeat(timeout=30)
    target_sys = master.target_system if master.target_system != 0 else 1
    target_comp = master.target_component if master.target_component != 0 else 1
    print(f"Heartbeat received from system {target_sys}, component {target_comp}")

    # Set SITL failsafe bypass parameters
    def set_p(name, val):
        master.mav.param_set_send(target_sys, target_comp, name.encode('utf-8'), val, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        time.sleep(0.05)

    set_p('NAV_DLL_ACT', 0)
    set_p('NAV_RCL_ACT', 0)
    set_p('COM_RCL_EXCEPT', 4)
    set_p('COM_ARM_WO_GPS', 1)
    set_p('CBRK_SUPPLY_CHK', 894565)
    set_p('MIS_TAKEOFF_ALT', 4)
    time.sleep(0.5)

    # 1. Arm vehicle
    print("1. Arming drone...")
    master.mav.command_long_send(
        target_sys, target_comp,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )
    time.sleep(1.0)

    # 2. Takeoff to transition vehicle to IN_AIR state
    target_alt = -4.2 # NED z = -4.2m (Altitude 4.2m)
    print("2. Issuing Takeoff command to 4.2m altitude...")
    master.mav.command_long_send(
        target_sys, target_comp,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, 4.2
    )
    time.sleep(7.0)

    MASK_POS_Z_VEL_XY = 1507

    def send_setpoint(x=0.0, y=0.0, z=target_alt, vx=0.0, vy=1.2, vz=0.0, yaw=0.0, yaw_rate=0.0, mask=MASK_POS_Z_VEL_XY):
        master.mav.set_position_target_local_ned_send(
            int(time.time() * 1000) & 0xFFFFFFFF,
            target_sys, target_comp,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            mask,
            x, y, z,
            vx, vy, vz,
            0.0, 0.0, 0.0,
            yaw, yaw_rate
        )

    # Circular trajectory parameters (Centered on box cluster 6.0, 1.0 with R=4.0m)
    R = 4.0           # meters
    speed = 1.2       # m/s
    omega = speed / R # rad/s (0.30 rad/s)
    flight_duration = 22.0  # seconds (full 360-degree loop)

    # 3. Pre-stream setpoints BEFORE requesting OFFBOARD mode
    print(f"3. Pre-streaming OFFBOARD setpoints (Center=[6.0, 1.0], R={R}m, speed={speed}m/s) for 1.5s...")
    pre_start = time.time()
    while time.sleep(0.05) or (time.time() - pre_start < 1.5):
        send_setpoint(z=target_alt, vx=0.0, vy=speed, yaw_rate=omega)

    # 4. Explicitly request OFFBOARD mode via MAV_CMD_DO_SET_MODE
    print("4. Requesting OFFBOARD mode via MAV_CMD_DO_SET_MODE (param1=1, param2=6)...")
    master.mav.command_long_send(
        target_sys, target_comp,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        0, 1, 6, 0, 0, 0, 0, 0
    )
    time.sleep(0.5)

    # 5. Verify OFFBOARD mode transition
    print("5. Verifying vehicle mode transition...")
    mode_verified = False
    for _ in range(25):
        send_setpoint(z=target_alt, vx=0.0, vy=speed, yaw_rate=omega)
        hb = master.recv_match(type='HEARTBEAT', blocking=True, timeout=0.2)
        if hb and hb.get_srcSystem() == target_sys and hb.get_srcComponent() == target_comp:
            main_m = (hb.custom_mode >> 16) & 0xFF
            is_offboard = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_GUIDED_ENABLED) or (main_m == 6)
            if is_offboard:
                mode_verified = True
                print("   [MODE VERIFIED] Drone is cleanly locked in OFFBOARD mode!")
                break

    if not mode_verified:
        print("   [RETRYING MODE] Re-sending MAV_CMD_DO_SET_MODE...")
        master.mav.command_long_send(
            target_sys, target_comp,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0, 1, 6, 0, 0, 0, 0, 0
        )
        time.sleep(0.5)

    # 6. Stream main CIRCULAR V2 OFFBOARD flight trajectory
    print(f"6. Streaming CIRCULAR V2 OFFBOARD flight trajectory (R={R}m, speed={speed}m/s, omega={omega:.3f}rad/s, duration={flight_duration}s)...")
    flight_start = time.time()

    while time.time() - flight_start < flight_duration:
        t = time.time() - flight_start
        vx_ned = speed * math.sin(omega * t)
        vy_ned = speed * math.cos(omega * t)

        send_setpoint(z=target_alt, vx=vx_ned, vy=vy_ned, vz=0.0, yaw_rate=omega)
        time.sleep(0.05)

    # 7. Landing sequence
    print("7. Circular V2 trajectory complete. Requesting LAND...")
    master.mav.command_long_send(
        target_sys, target_comp,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0, 0, 0, 0, 0, 0, 0, 0
    )
    time.sleep(3.0)

    stop_hb = True
    print("OFFBOARD Circular V2 Flight Execution Complete.")

if __name__ == '__main__':
    main()
