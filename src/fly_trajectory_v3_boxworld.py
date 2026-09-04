#!/usr/bin/env python3
"""
Offboard Circular Trajectory Command Script for PX4 SITL via MAVLink (v3 Boxworld Tight Baseline)

Flies a circular path centered at the geometric center of boxworld_obstacles_tight:
- Circle Center: (x_c, y_c) = (5.0, 5.0) m
- Circle Radius: R = 6.0 m (Minimum wall clearance = 4.0m everywhere!)
- Cruise Altitude: z = 2.5 m (z = -2.5m in Local NED frame)
- Speed: v = 1.2 m/s (omega = 0.20 rad/s, Period T ~ 31.42s)
- Yaw Rate: omega = 0.20 rad/s (tangential direction of travel)

Verified Offboard Trajectory:
- Uses Position (X, Y, Z) + Velocity (Vx, Vy, Vz) + Yaw Rate Setpoint Mask (0x04E0 = 1024)
- Trajectory:
    x(t)  = 5.0 + R * cos(omega * t)
    y(t)  = 5.0 + R * sin(omega * t)
    z(t)  = -2.5
    vx(t) = -R * omega * sin(omega * t)
    vy(t) =  R * omega * cos(omega * t)
"""

import math
import time
import sys
import subprocess
import threading
from pymavlink import mavutil

def teleport_drone(x=11.0, y=5.0, z=2.5):
    cmd = (
        f"gz service -s /world/boxworld_obstacles_tight/set_pose --reqtype gz.msgs.Pose "
        f"--reptype gz.msgs.Boolean --timeout 2000 --req "
        f"'name: \"x500_mono_cam_0\", position: {{x: {x}, y: {y}, z: {z}}}'"
    )
    res = subprocess.run(cmd, shell=True, capture_output=True, text=True)
    return "true" in res.stdout

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

    def set_p(name, val):
        master.mav.param_set_send(target_sys, target_comp, name.encode('utf-8'), float(val), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        time.sleep(0.05)

    set_p('NAV_DLL_ACT', 0)
    set_p('NAV_RCL_ACT', 0)
    set_p('COM_RCL_EXCEPT', 4)
    set_p('COM_ARM_WO_GPS', 1)
    set_p('CBRK_SUPPLY_CHK', 894565)
    set_p('CBRK_IO_SAFETY', 22027)
    time.sleep(0.5)

    # Trajectory parameters
    xc, yc = 5.0, 5.0      # Center of arena
    R = 6.0                # meters radius
    speed = 1.2            # m/s
    omega = speed / R      # rad/s (0.20 rad/s)
    alt_z = -2.5           # NED altitude (-2.5m = 2.5m height)
    flight_duration = 35.0 # seconds

    # MAVLink setpoint mask: 1024 (0x04E0) -> USE Position X, Y, Z + Velocity X, Y, Z + Yaw Rate
    MASK_POS_VEL_YAWRATE = 1024

    def send_setpoint(x, y, z, vx, vy, vz=0.0, yaw_rate=omega, mask=MASK_POS_VEL_YAWRATE):
        master.mav.set_position_target_local_ned_send(
            int(time.time() * 1000) & 0xFFFFFFFF,
            target_sys, target_comp,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            mask,
            x, y, z,           # x, y, z positions (meters)
            vx, vy, vz,        # vx, vy, vz velocities (m/s)
            0.0, 0.0, 0.0,     # ax, ay, az accelerations
            0.0, yaw_rate      # yaw (rad), yaw_rate (rad/s)
        )

    # Pre-stream start position at t=0: x = 11.0, y = 5.0, z = -2.5
    x0, y0 = xc + R, yc
    vx0, vy0 = 0.0, speed

    print(f"1. Teleporting drone to trajectory start point ({x0:.1f}, {y0:.1f}, 2.5m)...")
    teleport_drone(x0, y0, 2.5)

    print("2. Pre-streaming OFFBOARD setpoints for 1.5s...")
    pre_start = time.time()
    while time.sleep(0.05) or (time.time() - pre_start < 1.5):
        send_setpoint(x0, y0, alt_z, vx0, vy0, yaw_rate=omega)

    print("3. Arming vehicle & requesting OFFBOARD mode...")
    master.mav.command_long_send(
        target_sys, target_comp,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 21968, 0, 0, 0, 0, 0
    )
    time.sleep(0.2)

    master.mav.command_long_send(
        target_sys, target_comp,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        0, 1, 6, 0, 0, 0, 0, 0
    )
    time.sleep(0.5)

    print("4. Verifying vehicle mode transition...")
    mode_verified = False
    for _ in range(25):
        send_setpoint(x0, y0, alt_z, vx0, vy0, yaw_rate=omega)
        hb = master.recv_match(type='HEARTBEAT', blocking=True, timeout=0.2)
        if hb and hb.get_srcSystem() == target_sys and hb.get_srcComponent() == target_comp:
            main_m = (hb.custom_mode >> 16) & 0xFF
            is_offboard = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_GUIDED_ENABLED) or (main_m == 6)
            if is_offboard:
                mode_verified = True
                print("   [MODE VERIFIED] Drone is cleanly locked in OFFBOARD mode!")
                break

    # 5. Stream main CIRCULAR V3 OFFBOARD flight trajectory
    print(f"5. Streaming CIRCULAR V3 OFFBOARD trajectory (Center=[5.0, 5.0], R={R}m, speed={speed}m/s, alt=2.5m, duration={flight_duration}s)...")
    flight_start = time.time()

    while time.time() - flight_start < flight_duration:
        t = time.time() - flight_start
        x_target = xc + R * math.cos(omega * t)
        y_target = yc + R * math.sin(omega * t)
        vx_target = -R * omega * math.sin(omega * t)
        vy_target =  R * omega * math.cos(omega * t)

        send_setpoint(x_target, y_target, alt_z, vx_target, vy_target, vz=0.0, yaw_rate=omega)
        time.sleep(0.05)

    print("6. Circular V3 trajectory complete. Requesting LAND...")
    master.mav.command_long_send(
        target_sys, target_comp,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0, 0, 0, 0, 0, 0, 0, 0
    )
    time.sleep(4.0)

    stop_hb = True
    print("OFFBOARD Circular V3 Flight Execution Complete.")

if __name__ == '__main__':
    main()

