#!/usr/bin/env python3
"""
Phase 1 Parameterized Motion Generator for PX4 SITL in agriculture.world

Executes core motion families (F1, F2, F4, F5, F6, F9, F10, F11, HOVER) across severity levels (L0..L5).
Uses native PX4 MAVLink setpoints (SET_POSITION_TARGET_LOCAL_NED).
Implements a closed-loop takeoff readiness state machine before starting trajectory clock t=0.
Monitors live telemetry and enforces strict safety failsafes in PX4 local coordinates during motion.

Frame Alignment (PX4 SITL vs Gazebo World at spawn 14.0505, -7.5229):
- PX4 Local +Y_NED (East)  <-> Gazebo World +X (Longitudinal crop rows)
- PX4 Local +X_NED (North) <-> Gazebo World +Y (Lateral across rows)
"""

import argparse
import math
import sys
import time
from pymavlink import mavutil

# Verified Safe Corridor Envelopes in PX4 Local NED Coordinates (Spawn origin = 0,0)
# Gazebo World corridor X [8, 32], Y [-13, -5] -> Local Y [-6.05, +17.95], Local X [-5.48, +2.52]
LOCAL_X_MIN, LOCAL_X_MAX = -5.48, 2.52
LOCAL_Y_MIN, LOCAL_Y_MAX = -6.05, 17.95
SAFE_Z_MIN, SAFE_Z_MAX = 1.0, 4.5
MAX_TILT_DEG = 45.0
MAX_DESCENT_RATE_MPS = 2.0


def main():
    parser = argparse.ArgumentParser(description="Phase 1 Motion Generator")
    parser.add_argument('--family', type=str, required=True, choices=['F1','F2','F3','F4','F5','F6','F7','F8','F9','F10','F11','HOVER'], help="Motion family ID")
    parser.add_argument('--severity', type=int, default=2, choices=[0,1,2,3,4,5], help="Severity level (0..5)")
    parser.add_argument('--duration', type=float, default=20.0, help="Flight duration in seconds")
    parser.add_argument('--alt-z', type=float, default=2.41, help="Hover altitude Z (m ENU)")
    args = parser.parse_args()

    family = args.family
    severity = args.severity
    duration = args.duration
    alt_z = args.alt_z

    print("==========================================================================")
    print(f"PHASE 1 MOTION GENERATOR — Family: {family}, Severity: L{severity}")
    print("==========================================================================")
    print(f"Local Origin (0,0)  : PX4 EKF Spawn (Gazebo X: 14.0505m, Y: -7.5229m)")
    print(f"Frame Mapping       : PX4 +Y_NED -> Gazebo +X (Longitudinal), PX4 +X_NED -> Gazebo +Y (Lateral)")
    print(f"Cruise Altitude Z   : {alt_z:.2f} m ENU (-{alt_z:.2f} m NED)")
    print(f"Planned Duration    : {duration:.1f} s after TAKEOFF_READY")
    print(f"Local Geofence Bounds: Local X[{LOCAL_X_MIN},{LOCAL_X_MAX}] Local Y[{LOCAL_Y_MIN},{LOCAL_Y_MAX}] Alt Z[{SAFE_Z_MIN},{SAFE_Z_MAX}]")
    print("--------------------------------------------------------------------------")

    # Connect to PX4 SITL MAVLink UDP port
    mav_addr = "udpin:0.0.0.0:14540"
    print(f"Connecting to PX4 via MAVLink at {mav_addr}...")
    master = mavutil.mavlink_connection(mav_addr)
    master.wait_heartbeat()
    target_sys = master.target_system
    target_comp = master.target_component
    print(f"Heartbeat received! (SysID: {target_sys}, CompID: {target_comp})")

    def send_setpoint(x_local, y_local, z_enu, vx=0.0, vy=0.0, vz=0.0, yaw=0.0, yaw_rate=0.0, mask=3576):
        # MAV_FRAME_LOCAL_NED: +X North, +Y East, +Z Down.
        # Local origin (0,0) is at spawn location. ENU z -> NED -z.
        master.mav.set_position_target_local_ned_send(
            int(time.time() * 1000) & 0xFFFFFFFF,
            target_sys, target_comp,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            mask,
            y_local, x_local, -z_enu,
            vy, vx, -vz,
            0.0, 0.0, 0.0,
            yaw, yaw_rate
        )

    def send_att_setpoint(roll_deg, pitch_deg, yaw_deg, thrust=0.71):
        from scipy.spatial.transform import Rotation as R
        r = R.from_euler('xyz', [math.radians(roll_deg), math.radians(pitch_deg), math.radians(yaw_deg)], degrees=False)
        q = r.as_quat() # x, y, z, w
        q_wxyz = [q[3], q[0], q[1], q[2]]
        # type_mask = 7 (ignore body rates, command quaternion + thrust)
        master.mav.set_attitude_target_send(
            int(time.time() * 1000) & 0xFFFFFFFF,
            target_sys, target_comp,
            7, q_wxyz, 0.0, 0.0, 0.0, thrust
        )

    # 1. Pre-stream setpoints for 1.5s (Local origin 0,0, alt_z)
    print("[1/4] Pre-streaming OFFBOARD hover setpoints (Local 0, 0, 2.41m) for 1.5s...")
    t0 = time.time()
    while time.time() - t0 < 1.5:
        send_setpoint(0.0, 0.0, alt_z, yaw=0.0, mask=3576)
        time.sleep(0.05)

    # 2. Arm Vehicle
    print("[2/4] Arming vehicle...")
    master.mav.command_long_send(
        target_sys, target_comp,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 21968, 0, 0, 0, 0, 0
    )
    time.sleep(0.2)

    # 3. Request OFFBOARD Mode
    print("[3/4] Requesting OFFBOARD mode...")
    master.mav.command_long_send(
        target_sys, target_comp,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        0, 1, 6, 0, 0, 0, 0, 0
    )
    time.sleep(0.5)

    # 4. TAKEOFF PHASE: Closed-Loop Altitude Readiness State Machine
    print("[EVENT: TAKEOFF_START] Climbing to cruise altitude (2.41m ENU)...")
    takeoff_start_time = time.time()
    takeoff_ready = False
    stable_start_time = None
    TAKEOFF_TIMEOUT_SEC = 12.0
    STABILITY_REQUIRED_SEC = 0.5
    TARGET_MIN_ALT_M = 2.0

    while True:
        elapsed_takeoff = time.time() - takeoff_start_time
        if elapsed_takeoff > TAKEOFF_TIMEOUT_SEC:
            print(f"[EVENT: SAFETY_ABORT] Takeoff timeout ({TAKEOFF_TIMEOUT_SEC}s) reached without achieving 2.0m altitude!")
            break

        # Send takeoff setpoint (Local 0.0, 0.0, 2.41m ENU)
        send_setpoint(0.0, 0.0, alt_z, yaw=0.0, mask=3576)
        msg = master.recv_match(type='LOCAL_POSITION_NED', blocking=False)
        if msg:
            px_local, py_local, pz_alt = msg.x, msg.y, -msg.z
            if pz_alt >= TARGET_MIN_ALT_M:
                if stable_start_time is None:
                    stable_start_time = time.time()
                    print(f"[EVENT: TAKEOFF_ALTITUDE_UPDATE] Altitude threshold reached ({pz_alt:.2f}m >= {TARGET_MIN_ALT_M}m). Starting 0.5s stability timer...")
                elif (time.time() - stable_start_time) >= STABILITY_REQUIRED_SEC:
                    takeoff_ready = True
                    print(f"[EVENT: TAKEOFF_READY] Cruise altitude stable for {STABILITY_REQUIRED_SEC}s! Total Takeoff Time: {elapsed_takeoff:.2f}s | Pose: Local X={px_local:.2f}m, Local Y={py_local:.2f}m, Alt={pz_alt:.2f}m")
                    break
            else:
                if stable_start_time is not None:
                    print(f"[EVENT: TAKEOFF_ALTITUDE_UPDATE] Altitude dropped below threshold ({pz_alt:.2f}m < {TARGET_MIN_ALT_M}m). Resetting stability timer...")
                stable_start_time = None

        time.sleep(0.05)

    if not takeoff_ready:
        print("[EVENT: LAND_START] Takeoff readiness failed. Disengaging and landing...")
        master.mav.command_long_send(target_sys, target_comp, mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0)
        sys.exit(1)

    # 5. ACTIVE MOTION PHASE: Trajectory Clock t=0 Starts NOW
    print(f"\n[EVENT: MOTION_START] Starting motion profile '{family}' (Severity L{severity}) for {duration:.1f}s after TAKEOFF_READY...\n")
    motion_start_time = time.time()  # TRAJECTORY t=0
    sev_factor = severity / 2.0      # L2 = 1.0, L1 = 0.5, L3 = 1.5, L4 = 2.0, L5 = 2.5

    while True:
        elapsed = time.time() - motion_start_time
        if elapsed >= duration:
            print(f"[EVENT: MOTION_END] Motion profile '{family}' completed ({elapsed:.2f}s >= {duration:.1f}s).")
            break

        # Compute trajectory profile targets in PX4 LOCAL coordinates
        # Map Gazebo +X (Longitudinal) -> PX4 Local +Y_NED (East)
        # Map Gazebo +Y (Lateral)      -> PX4 Local +X_NED (North)

        if family == 'HOVER': # Infrastructure Soak Test Stationary Hover
            curr_x_local = 0.0
            curr_y_local = 0.0
            curr_z_enu = alt_z
            send_setpoint(curr_x_local, curr_y_local, curr_z_enu, yaw=0.0, mask=3576)

        elif family == 'F1': # Forward/Backward Translation along Gazebo +X (crop rows)
            v_cruise = 1.5 * sev_factor
            dist = min(v_cruise * elapsed, 14.0)
            curr_x_local = 0.0         # Lateral (Gazebo Y) holds 0.0
            curr_y_local = dist        # Longitudinal (Gazebo X) advances 0 -> 14.0m
            curr_z_enu = alt_z
            send_setpoint(curr_x_local, curr_y_local, curr_z_enu, yaw=0.0, mask=3576)

        elif family == 'F2': # F2 Motion (Command PX4 local Y = A * sin(2*pi*f*t), local X = 0 for Gazebo world X variation)
            A_lat = 1.8 * min(sev_factor, 1.25)   # Amplitude A = 1.8m for L2
            f_lat = 0.125                         # Frequency f = 0.125 Hz (8.0s period)
            curr_x_local = 0.0                                                # Local X (Gazebo Y) holds 0.0m
            curr_y_local = A_lat * math.sin(2.0 * math.pi * f_lat * elapsed)  # Local Y (Gazebo X) oscillates +/-1.8m
            curr_z_enu = alt_z
            # Position mask 3576 (0x0DF8): Pos X, Y, Z + Yaw Angle enabled
            send_setpoint(curr_x_local, curr_y_local, curr_z_enu, yaw=0.0, mask=3576)

        elif family == 'F3': # Vertical Climb / Descend Oscillation
            curr_x_local = 0.0
            curr_y_local = 0.0
            f_z = 0.25
            A_z = 0.5 * min(sev_factor, 1.0)
            curr_z_enu = alt_z + A_z * math.sin(2.0 * math.pi * f_z * elapsed)
            send_setpoint(curr_x_local, curr_y_local, curr_z_enu, yaw=0.0, mask=3576)

        elif family == 'F4': # Roll + Translation (Coupled Longitudinal Translation + Dynamic Roll Oscillation)
            v_fwd = 1.0 * sev_factor
            f_roll = 0.5  # Roll oscillation frequency 0.5 Hz
            A_roll_pos = (0.8 * sev_factor) / (2.0 * math.pi * f_roll)  # 0.2546m amplitude for L2
            curr_y_local = min(v_fwd * elapsed, 14.0) + A_roll_pos * math.sin(2.0 * math.pi * f_roll * elapsed)
            curr_x_local = 0.0
            curr_z_enu = alt_z
            send_setpoint(curr_x_local, curr_y_local, curr_z_enu, yaw=0.0, mask=3576)

        elif family == 'F5': # Pitch + Translation (Coupled Dynamic Body Pitch Oscillation + Forward Translation)
            v_fwd = 1.0 * sev_factor
            curr_y_local = min(v_fwd * elapsed, 14.0)
            f_pitch = 0.5  # Pitch oscillation frequency 0.5 Hz
            A_pitch_pos = (0.6 * sev_factor) / (2.0 * math.pi * f_pitch)  # 0.191m amplitude for L2
            curr_x_local = A_pitch_pos * math.sin(2.0 * math.pi * f_pitch * elapsed)
            curr_z_enu = alt_z
            send_setpoint(curr_x_local, curr_y_local, curr_z_enu, yaw=0.0, mask=3576)

        elif family == 'F6': # Pure Yaw Oscillation via SET_ATTITUDE_TARGET
            f_yaw = 0.25  # 0.25 Hz frequency (4.0s period)
            A_yaw_deg = 30.0
            yaw_target_deg = A_yaw_deg * math.sin(2.0 * math.pi * f_yaw * elapsed)

            msg_pos = master.recv_match(type='LOCAL_POSITION_NED', blocking=False)
            curr_z = -msg_pos.z if (msg_pos and hasattr(msg_pos, 'z')) else alt_z
            curr_vz = -msg_pos.vz if (msg_pos and hasattr(msg_pos, 'vz')) else 0.0

            err_z = alt_z - curr_z
            thrust_cmd = min(0.85, max(0.40, 0.71 + 0.15 * err_z - 0.05 * curr_vz))

            send_att_setpoint(roll_deg=0.0, pitch_deg=0.0, yaw_deg=yaw_target_deg, thrust=thrust_cmd)

        elif family == 'F7': # Pitch + Yaw Translation
            v_fwd = 1.0 * sev_factor
            curr_y_local = min(v_fwd * elapsed, 14.0)
            f_pitch = 0.5
            A_pitch_pos = (0.5 * sev_factor) / (2.0 * math.pi * f_pitch)
            curr_x_local = A_pitch_pos * math.sin(2.0 * math.pi * f_pitch * elapsed)
            curr_z_enu = alt_z
            f_yaw = 0.25
            A_yaw_deg = 20.0 * min(sev_factor, 1.0)
            yaw_target_rad = math.radians(A_yaw_deg) * math.sin(2.0 * math.pi * f_yaw * elapsed)
            send_setpoint(curr_x_local, curr_y_local, curr_z_enu, yaw=yaw_target_rad, mask=3576)

        elif family == 'F8': # Roll + Yaw Translation
            v_fwd = 1.0 * sev_factor
            f_roll = 0.5
            A_roll_pos = (0.5 * sev_factor) / (2.0 * math.pi * f_roll)
            curr_y_local = min(v_fwd * elapsed, 14.0) + A_roll_pos * math.sin(2.0 * math.pi * f_roll * elapsed)
            curr_x_local = 0.0
            curr_z_enu = alt_z
            f_yaw = 0.25
            A_yaw_deg = 20.0 * min(sev_factor, 1.0)
            yaw_target_rad = math.radians(A_yaw_deg) * math.sin(2.0 * math.pi * f_yaw * elapsed)
            send_setpoint(curr_x_local, curr_y_local, curr_z_enu, yaw=yaw_target_rad, mask=3576)

        elif family == 'F9': # Yaw + Translation (Coupled Forward Translation + Dynamic Yaw Oscillation)
            v_fwd = 1.0 * sev_factor
            curr_y_local = min(v_fwd * elapsed, 14.0)
            curr_x_local = 0.0
            curr_z_enu = alt_z
            f_yaw = 0.25  # 0.25 Hz frequency (4.0s period)
            A_yaw_deg = 30.0 * min(sev_factor, 1.0)  # +/-30 degrees amplitude at L2
            yaw_target_rad = math.radians(A_yaw_deg) * math.sin(2.0 * math.pi * f_yaw * elapsed)
            yaw_rate_ff_rad_s = math.radians(A_yaw_deg) * (2.0 * math.pi * f_yaw) * math.cos(2.0 * math.pi * f_yaw * elapsed)
            # Position + Yaw Angle + Yaw Rate Feedforward mask 504 (0x01F8): Pos X,Y,Z enabled, Yaw & Yaw Rate enabled
            send_setpoint(curr_x_local, curr_y_local, curr_z_enu, yaw=yaw_target_rad, yaw_rate=yaw_rate_ff_rad_s, mask=504)

        elif family == 'F10': # Combined Aggressive via SET_ATTITUDE_TARGET
            f_yaw = 0.50  # 0.50 Hz frequency
            A_yaw_deg = 20.0
            yaw_target_deg = A_yaw_deg * math.sin(2.0 * math.pi * f_yaw * elapsed)

            f_lat = 0.50
            roll_cmd_deg = -10.0 * math.cos(2.0 * math.pi * f_lat * elapsed)
            pitch_cmd_deg = -5.0 * math.sin(2.0 * math.pi * f_lat * elapsed)

            msg_pos = master.recv_match(type='LOCAL_POSITION_NED', blocking=False)
            curr_z = -msg_pos.z if (msg_pos and hasattr(msg_pos, 'z')) else alt_z
            curr_vz = -msg_pos.vz if (msg_pos and hasattr(msg_pos, 'vz')) else 0.0
            err_z = alt_z - curr_z
            thrust_cmd = min(0.85, max(0.40, 0.71 + 0.15 * err_z - 0.05 * curr_vz))

            send_att_setpoint(roll_deg=roll_cmd_deg, pitch_deg=pitch_cmd_deg, yaw_deg=yaw_target_deg, thrust=thrust_cmd)

        elif family == 'F11': # S-Turns / Lateral Reversals + Forward Progression (Severity Level 2)
            v_fwd = 1.0                                              # 1.0 m/s forward translation along PX4 local +Y (Gazebo world +X)
            curr_y_local = min(v_fwd * elapsed, 14.0)               # Local Y setpoint capped at 14.0m to maintain 3.95m margin inside LOCAL_Y_MAX (17.95m)
            # Lateral S-turn: amplitude 1.0m, f = 0.125 Hz (period 8.0s) along PX4 local +X (Gazebo world +Y)
            curr_x_local = 1.0 * math.sin(2.0 * math.pi * 0.125 * elapsed)
            curr_z_enu = alt_z                                       # Cruise altitude 2.41m ENU
            # Position + Yaw mask 2552 (0x0F98): Pos X, Y, Z + Yaw Angle ENABLED
            send_setpoint(curr_x_local, curr_y_local, curr_z_enu, yaw=0.0, mask=2552)


        # Active Safety Boundary Checks via MAVLink LOCAL_POSITION_NED (ACTIVE_MOTION_PHASE only)
        msg = master.recv_match(type='LOCAL_POSITION_NED', blocking=False)
        if msg:
            px_local, py_local, pz_alt = msg.x, msg.y, -msg.z
            x_min_bound = LOCAL_X_MIN if family not in ['F6', 'F10'] else -6.0
            x_max_bound = LOCAL_X_MAX if family not in ['F6', 'F10'] else 6.0
            if not (x_min_bound <= px_local <= x_max_bound and LOCAL_Y_MIN <= py_local <= LOCAL_Y_MAX and SAFE_Z_MIN <= pz_alt <= SAFE_Z_MAX):
                print(f"[EVENT: SAFETY_ABORT] Active spatial bounds breached! Pos: Local X={px_local:.2f}m, Local Y={py_local:.2f}m, Alt={pz_alt:.2f}m. Disengaging...")
                break

        time.sleep(0.05)

    # 6. Clean Return-to-Hover & Land (Local origin 0, 0, alt_z)
    print("\n[EVENT: RETURN_START] Motion sequence finished. Returning to hover at spawn line (0,0) for 2.0s before landing...")
    final_t0 = time.time()
    while time.time() - final_t0 < 2.0:
        send_setpoint(0.0, 0.0, alt_z, yaw=0.0, mask=3576)
        time.sleep(0.05)

    print("[EVENT: LAND_START] Sending LAND command to PX4...")
    master.mav.command_long_send(
        target_sys, target_comp,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0, 0, 0, 0, 0, 0, 0, 0
    )
    time.sleep(2.0)
    print("[EVENT: LAND_COMPLETE] Land command sent. Exiting motion generator.")


if __name__ == '__main__':
    main()
