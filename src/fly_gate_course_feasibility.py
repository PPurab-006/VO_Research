#!/usr/bin/env python3
"""
Standalone Gate Course Feasibility Flight Script using pymavlink.

Executes a single, slow, conservative flight pass through 7 gates in order (1 -> 2 -> 3 -> 4 -> 5 -> 6 -> 7)
in drone_race_track_2018_actual_with_gatepapers.world using PX4 SITL OFFBOARD mode.

Speed: ~0.60 m/s (conservative feasibility pass).
Headings: Dynamic yaw facing direction of travel at each waypoint segment.
GT Logging: Handled separately by record_ground_truth.py (NO VO Node).
"""

import time
import math
import sys
import threading
import numpy as np
from pymavlink import mavutil

def main():
    print("======================================================================")
    print("STARTING GATE COURSE FEASIBILITY PASS VIA PYMAVLINK (Gates 1 -> 7)")
    print("======================================================================")

    mav_addr = "udpin:0.0.0.0:14540"
    print(f"Connecting to PX4 SITL via MAVLink at {mav_addr}...")
    master = mavutil.mavlink_connection(mav_addr)

    # Start background GCS heartbeat thread FIRST to establish connection
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
    print(f"Heartbeat received! System ID: {target_sys}, Component ID: {target_comp}")

    # Set SITL failsafe bypass parameters
    def set_p(name, val):
        master.mav.param_set_send(target_sys, target_comp, name.encode('utf-8'), val, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        time.sleep(0.05)

    set_p('NAV_DLL_ACT', 0)
    set_p('NAV_RCL_ACT', 0)
    set_p('COM_RCL_EXCEPT', 4)
    set_p('COM_ARM_WO_GPS', 1)
    set_p('CBRK_SUPPLY_CHK', 894565)
    set_p('CBRK_IO_SAFETY', 22027)
    set_p('EKF2_MAG_TYPE', 5)
    set_p('MIS_TAKEOFF_ALT', 2)
    time.sleep(0.5)

    # MAVLink setpoint type mask:
    # 3576 (0x0DF8) -> Use Position (x, y, z) + Yaw (ignore vel, accel, yaw_rate)
    MASK_POS_YAW = 3576

    def send_setpoint_ned(x_ned, y_ned, z_ned, yaw_ned_rad):
        master.mav.set_position_target_local_ned_send(
            int(time.time() * 1000) & 0xFFFFFFFF,
            target_sys, target_comp,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            MASK_POS_YAW,
            x_ned, y_ned, z_ned,
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            yaw_ned_rad, 0.0
        )

    # Convert Gazebo ENU (x, y, z, yaw_deg) -> PX4 LOCAL_NED (x_ned, y_ned, z_ned, yaw_ned_rad)
    def enu_to_ned(x_enu, y_enu, z_enu, yaw_enu_deg):
        x_ned = y_enu
        y_ned = x_enu
        z_ned = -z_enu
        # ENU yaw: 90=North, 0=East, 180=South, 270=-90=West
        # NED yaw: 0=North, pi/2=East, pi=South, -pi/2=West
        yaw_ned_rad = math.radians((90.0 - yaw_enu_deg + 360.0) % 360.0)
        if yaw_ned_rad > math.pi:
            yaw_ned_rad -= 2.0 * math.pi
        return x_ned, y_ned, z_ned, yaw_ned_rad

    # Waypoints in Gazebo ENU: (x, y, z, yaw_deg, desc)
    waypoints_enu = [
        (0.00,  0.00, 1.55,  90.0, "Pre-Gate 1 Takeoff & Hover"),
        (0.00,  3.50, 1.55,  90.0, "Gate 1 Center"),
        (0.00,  7.70, 1.55,  90.0, "Gate 2 Center"),
        (2.75, 10.45, 2.05,  45.0, "Gate 3 Center (Turn East-North)"),
        (4.90,  7.40, 1.55, -90.0, "Gate 4 Center (Turn South)"),
        (4.90,  2.60, 1.55, -90.0, "Gate 5 Center (South)"),
        (1.10,  2.00, 2.95, 180.0, "Gate 6 Center (Elevated Climb-Turn West)"),
        (2.00,  5.30, 1.45,  90.0, "Gate 7 Center (Turn North)"),
        (2.00,  7.30, 1.45,  90.0, "Post-Gate 7 Stabilization Hover")
    ]

    # Convert all waypoints to PX4 NED
    waypoints_ned = []
    for x_enu, y_enu, z_enu, yaw_deg, desc in waypoints_enu:
        xn, yn, zn, yaw_rad = enu_to_ned(x_enu, y_enu, z_enu, yaw_deg)
        waypoints_ned.append((xn, yn, zn, yaw_rad, desc, (x_enu, y_enu, z_enu)))

    # Live telemetry state (non-blocking update, matching fly_roll_validation.py pattern)
    actual_xn, actual_yn, actual_zn = waypoints_ned[0][0], waypoints_ned[0][1], waypoints_ned[0][2]
    actual_yaw = waypoints_ned[0][3]

    def update_telemetry():
        nonlocal actual_xn, actual_yn, actual_zn, actual_yaw
        updated = False
        while True:
            msg = master.recv_match(type=['LOCAL_POSITION_NED', 'ATTITUDE'], blocking=False)
            if not msg:
                break
            mtype = msg.get_type()
            if mtype == 'LOCAL_POSITION_NED':
                actual_xn = msg.x
                actual_yn = msg.y
                actual_zn = msg.z
                updated = True
            elif mtype == 'ATTITUDE':
                actual_yaw = msg.yaw
                updated = True
        return updated

    # Step 1: Pre-stream OFFBOARD setpoints for 1.5s BEFORE arming
    # (PX4 requires active setpoint stream before OFFBOARD can engage)
    print("1. Pre-streaming OFFBOARD position setpoints (~1.5s)...")
    wp0 = waypoints_ned[0]
    pre_start = time.time()
    while time.time() - pre_start < 1.5:
        update_telemetry()
        send_setpoint_ned(wp0[0], wp0[1], wp0[2], wp0[3])
        time.sleep(0.05)

    # Step 2: ARM with force-arm parameter (param2=21968 bypasses pre-flight checks in SITL)
    print("2. Arming vehicle (retrying until armed confirmation)...")
    armed_confirmed = False
    armed_wait_start = time.time()
    last_arm_cmd_time = 0.0
    last_hb_base_mode = None

    while time.time() - armed_wait_start < 10.0:
        update_telemetry()
        send_setpoint_ned(wp0[0], wp0[1], wp0[2], wp0[3])
        
        # Periodically re-send ARM command every 0.5s until confirmed
        if time.time() - last_arm_cmd_time > 0.5:
            master.mav.command_long_send(
                target_sys, target_comp,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, 1, 21968, 0, 0, 0, 0, 0
            )
            last_arm_cmd_time = time.time()

        hb = master.recv_match(type='HEARTBEAT', blocking=False)
        if hb:
            last_hb_base_mode = hb.base_mode
            if hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                armed_confirmed = True
                print(f"   Armed state CONFIRMED (heartbeat base_mode={hb.base_mode}, armed bit set)")
                break
        time.sleep(0.05)
    if not armed_confirmed:
        print(f"   WARNING: Armed state NOT confirmed after 10s (last base_mode={last_hb_base_mode}) — proceeding anyway")

    # Step 3: Request OFFBOARD mode (no NAV_TAKEOFF — position targets drive the climb)
    print("3. Requesting OFFBOARD mode via MAV_CMD_DO_SET_MODE (retrying until mode confirmation)...")
    offboard_confirmed = False
    offboard_wait_start = time.time()
    last_mode_cmd_time = 0.0
    last_hb_custom_mode = None

    while time.time() - offboard_wait_start < 10.0:
        update_telemetry()
        send_setpoint_ned(wp0[0], wp0[1], wp0[2], wp0[3])

        # Periodically re-send OFFBOARD mode command every 0.5s until confirmed
        if time.time() - last_mode_cmd_time > 0.5:
            master.mav.command_long_send(
                target_sys, target_comp,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                0,
                1,  # MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
                6,  # PX4 Main Mode 6 = OFFBOARD
                0, 0, 0, 0, 0
            )
            last_mode_cmd_time = time.time()

        hb = master.recv_match(type='HEARTBEAT', blocking=False)
        if hb:
            last_hb_custom_mode = hb.custom_mode
            if hb.custom_mode == 393216:  # PX4 OFFBOARD = 6 << 16 = 393216
                offboard_confirmed = True
                print(f"   OFFBOARD mode CONFIRMED (custom_mode={hb.custom_mode})")
                break
        time.sleep(0.05)
    if not offboard_confirmed:
        print(f"   WARNING: OFFBOARD mode NOT confirmed after 10s (last custom_mode={last_hb_custom_mode}) — proceeding anyway")

    # Step 4: Allow 5s for initial climb to hover altitude via OFFBOARD position targets
    print("4. Climbing to hover altitude via OFFBOARD position targets (5.0s)...")
    climb_start = time.time()
    while time.time() - climb_start < 5.0:
        update_telemetry()
        send_setpoint_ned(wp0[0], wp0[1], wp0[2], wp0[3])
        time.sleep(0.05)

    # Step 5: Execute Closed-Loop Waypoint Pass (Approach A: Ground-Truth Telemetry Reset per Segment)
    print("======================================================================")
    print("EXECUTING CLOSED-LOOP GATE COURSE WAYPOINT PASS (Approach A)")
    print("======================================================================")

    cruise_speed = 0.60  # m/s conservative speed

    for idx, (target_xn, target_yn, target_zn, target_yaw, desc, enu_pos) in enumerate(waypoints_ned):
        update_telemetry()
        target_yaw_deg = waypoints_enu[idx][3]

        # Step A: Pre-align heading in-place if initial yaw discrepancy > 15 degrees
        update_telemetry()
        hold_xn, hold_yn, hold_zn = actual_xn, actual_yn, actual_zn
        dyaw_initial = (target_yaw - actual_yaw + math.pi) % (2.0 * math.pi) - math.pi
        if abs(dyaw_initial) > math.radians(15.0):
            print(f"[{idx+1}/{len(waypoints_ned)}] Pre-aligning heading in-place to {desc} target yaw {target_yaw_deg:.1f}° (yaw error={math.degrees(dyaw_initial):.1f}°)...")
            align_start = time.time()
            while time.time() - align_start < 4.0:
                update_telemetry()
                dyaw_curr = (target_yaw - actual_yaw + math.pi) % (2.0 * math.pi) - math.pi
                if abs(dyaw_curr) < math.radians(10.0):
                    achieved_yaw_deg = (90.0 - math.degrees(actual_yaw) + 360.0) % 360.0
                    print(f"   Heading pre-alignment COMPLETE (Achieved ENU Yaw: {achieved_yaw_deg:.1f}°)")
                    break
                send_setpoint_ned(hold_xn, hold_yn, hold_zn, target_yaw)
                time.sleep(0.05)

        # Step B: Closed-loop position + yaw interpolation along segment (Approach A)
        update_telemetry()
        start_xn, start_yn, start_zn, start_yaw = actual_xn, actual_yn, actual_zn, actual_yaw

        dist = math.sqrt((target_xn - start_xn)**2 + (target_yn - start_yn)**2 + (target_zn - start_zn)**2)
        segment_time = max(dist / cruise_speed, 3.0)  # Min 3.0s per segment safety floor

        print(f"[{idx+1}/{len(waypoints_ned)}] Navigating to {desc} -> Target ENU ({enu_pos[0]:.2f}, {enu_pos[1]:.2f}, {enu_pos[2]:.2f}, {target_yaw_deg:.1f}°) | Dist={dist:.2f}m, Time={segment_time:.2f}s")

        num_steps = int(segment_time / 0.05)  # 20 Hz setpoint streaming
        for step in range(num_steps):
            update_telemetry()
            alpha = step / float(num_steps)
            interp_xn = start_xn + alpha * (target_xn - start_xn)
            interp_yn = start_yn + alpha * (target_yn - start_yn)
            interp_zn = start_zn + alpha * (target_zn - start_zn)

            # Interpolate yaw angle smoothly
            dyaw = (target_yaw - start_yaw + math.pi) % (2.0 * math.pi) - math.pi
            interp_yaw = start_yaw + alpha * dyaw

            send_setpoint_ned(interp_xn, interp_yn, interp_zn, interp_yaw)
            time.sleep(0.05)

        # Settle at target waypoint setpoint for 1.0s (2.0s for takeoff/finish)
        settle_duration = 2.0 if (idx == 0 or idx == len(waypoints_ned) - 1) else 1.0
        settle_start = time.time()
        while time.time() - settle_start < settle_duration:
            update_telemetry()
            send_setpoint_ned(target_xn, target_yn, target_zn, target_yaw)
            time.sleep(0.05)

        update_telemetry()
        pos_err = math.sqrt((actual_xn - target_xn)**2 + (actual_yn - target_yn)**2 + (actual_zn - target_zn)**2)
        actual_x_enu = actual_yn
        actual_y_enu = actual_xn
        actual_z_enu = -actual_zn
        achieved_yaw_deg = (90.0 - math.degrees(actual_yaw) + 360.0) % 360.0
        print(f"   [Segment {idx+1} End] Target ENU: ({enu_pos[0]:.2f}, {enu_pos[1]:.2f}, {enu_pos[2]:.2f}, {target_yaw_deg:.1f}°) | Actual ENU: ({actual_x_enu:.2f}, {actual_y_enu:.2f}, {actual_z_enu:.2f}, {achieved_yaw_deg:.1f}°) | Position Error: {pos_err:.2f}m")

    print("======================================================================")
    print("GATE COURSE WAYPOINT PASS COMPLETED")
    print("======================================================================")
    
    stop_hb = True

if __name__ == '__main__':
    main()
