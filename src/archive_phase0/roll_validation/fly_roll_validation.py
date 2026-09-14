#!/usr/bin/env python3
"""
Single 10-Degree Roll Validation Flight Script (Phase 0E Refined Architecture Validation)

Refined Hybrid Position-Corrected Attitude Controller:
1. Takes off & climbs to (0.0, 0.0, 2.5m) using standard position target setpoints (type_mask=3576).
2. At climb completion: captures initial hold position (hold_x, hold_y, hold_z) and initial heading psi_0 (NED).
3. Zero-Order Hold Persistent Telemetry: maintains latest valid (x, y, z, vx, vy, vz, roll, pitch, yaw)
   and logs telemetry_age_sec on every loop iteration (~50 Hz).
4. Executes 20-second maneuver phase streaming SET_ATTITUDE_TARGET setpoints:
   - Sinusoidal roll command: roll_osc(t) = A * sin(2*pi*f*t) with A = 10.0 deg, f = 0.5 Hz.
   - Fixed yaw lock: cmd_yaw = psi_0 (degrees in PX4 NED frame).
   - AC Velocity-Damped body-frame position feedback:
     * Pitch correction: corr_pitch = +Kp_x * err_fwd + Kd_x * vel_fwd (clamped +/- 3.0 deg).
     * Roll correction: corr_roll = -Kp_y * err_right - Kd_y * vel_right (Kd_y=7.5 for strong AC velocity damping, clamped +/- 3.5 deg).
   - Altitude-hold thrust compensation:
     T = min(0.85, max(0.30, (T_hover + z_corr) / (cos(roll) * cos(pitch))))
     where T_hover = 0.65, z_corr = -Kp_z * err_z - Kd_z * vel_z.
5. Logs telemetry to results/roll_validation_telemetry.csv.
"""

import os
import sys
import time
import math
import csv
import argparse
import numpy as np
from pymavlink import mavutil
from scipy.spatial.transform import Rotation as R_scipy


def euler_to_quat_wxyz(roll_rad, pitch_rad, yaw_rad):
    """Returns quaternion [w, x, y, z] from euler angles (xyz, rad) in PX4 NED frame."""
    r = R_scipy.from_euler('xyz', [roll_rad, pitch_rad, yaw_rad])
    q_xyzw = r.as_quat()
    return [q_xyzw[3], q_xyzw[0], q_xyzw[1], q_xyzw[2]]


def main():
    parser = argparse.ArgumentParser(description="10-Degree Roll Validation Flight Script")
    parser.add_argument('--roll-amp-deg', type=float, default=10.0, help="Target roll amplitude in degrees")
    parser.add_argument('--freq-hz', type=float, default=0.5, help="Roll oscillation frequency in Hz")
    parser.add_argument('--maneuver-duration', type=float, default=20.0, help="Maneuver phase duration in seconds")
    parser.add_argument('--target-x', type=float, default=14.0505, help="Target climb X position (Gazebo ENU)")
    parser.add_argument('--target-y', type=float, default=-7.5229, help="Target climb Y position (Gazebo ENU)")
    parser.add_argument('--target-z', type=float, default=2.4076, help="Target hover altitude Z in meters")
    parser.add_argument('--telem-filename', type=str, default="roll_validation_telemetry.csv", help="Output telemetry CSV filename")
    args = parser.parse_args()

    roll_amp_deg = args.roll_amp_deg
    freq_hz = args.freq_hz
    maneuver_duration = args.maneuver_duration
    target_x = args.target_x
    target_y = args.target_y
    target_z = args.target_z
    telem_filename = args.telem_filename

    print("============================================================")
    print(f"REFINED HYBRID ROLL VALIDATION FLIGHT (Target Amp: {roll_amp_deg:.1f} deg, Freq: {freq_hz:.2f} Hz)")
    print("============================================================")

    # Setup telemetry log file
    os.makedirs("results", exist_ok=True)
    telem_csv_path = os.path.join("results", telem_filename)
    telem_file = open(telem_csv_path, 'w', newline='')
    fieldnames = [
        'timestamp_total_sec', 'maneuver_state', 'maneuver_time_sec',
        'cmd_roll_deg', 'cmd_pitch_deg', 'cmd_yaw_deg', 'cmd_thrust',
        'cmd_q_w', 'cmd_q_x', 'cmd_q_y', 'cmd_q_z',
        'pos_x', 'pos_y', 'pos_z', 'vel_x', 'vel_y', 'vel_z',
        'pos_error_x', 'pos_error_y', 'pos_error_z', 'telem_age_sec'
    ]
    writer = csv.DictWriter(telem_file, fieldnames=fieldnames)
    writer.writeheader()
    telem_file.flush()

    # Connect to PX4 SITL MAVLink UDP port 14540
    mav_addr = "udpin:0.0.0.0:14540"
    print(f"Connecting to PX4 via MAVLink at {mav_addr}...")
    master = mavutil.mavlink_connection(mav_addr)
    master.wait_heartbeat()
    target_sys = master.target_system
    target_comp = master.target_component
    print(f"Heartbeat received! (SysID: {target_sys}, CompID: {target_comp})")

    # MAVLink sending helper functions
    def send_pos_target(x, y, z, yaw_rad=0.0):
        # type_mask = 3576 (0x0DF8): Position X, Y, Z + Yaw (ignore vel, accel, yaw_rate)
        # PX4 LOCAL_NED: x=North, y=East, z=Down
        # Input parameters are Gazebo ENU: x=East, y=North, z=Up
        master.mav.set_position_target_local_ned_send(
            int(time.time() * 1000) & 0xFFFFFFFF,
            target_sys, target_comp,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            3576,
            y, x, -z,          # NED North=y_enu, East=x_enu, Down=-z_enu
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            yaw_rad, 0.0
        )

    def send_att_target(roll_deg, pitch_deg, yaw_deg, thrust):
        q = euler_to_quat_wxyz(math.radians(roll_deg), math.radians(pitch_deg), math.radians(yaw_deg))
        # type_mask = 7 (ignore body rates, use quaternion + thrust)
        master.mav.set_attitude_target_send(
            int(time.time() * 1000) & 0xFFFFFFFF,
            target_sys, target_comp,
            7,
            q,
            0.0, 0.0, 0.0,
            thrust
        )
        return q

    # Persistent Telemetry State (Zero-Order Hold)
    curr_pos_x, curr_pos_y, curr_pos_z = 0.0, 0.0, 0.0
    curr_vel_x, curr_vel_y, curr_vel_z = 0.0, 0.0, 0.0
    curr_roll, curr_pitch, curr_yaw_ned = 0.0, 0.0, 0.0
    last_telem_time = time.time()

    def update_telemetry():
        nonlocal curr_pos_x, curr_pos_y, curr_pos_z
        nonlocal curr_vel_x, curr_vel_y, curr_vel_z
        nonlocal curr_roll, curr_pitch, curr_yaw_ned, last_telem_time
        updated = False
        while True:
            msg = master.recv_match(type=['LOCAL_POSITION_NED', 'ATTITUDE'], blocking=False)
            if not msg:
                break
            now_t = time.time()
            mtype = msg.get_type()
            if mtype == 'LOCAL_POSITION_NED':
                curr_pos_x = msg.y     # NED East -> Gazebo ENU X
                curr_pos_y = msg.x     # NED North -> Gazebo ENU Y
                curr_pos_z = -msg.z    # NED Down -> Gazebo ENU Z
                curr_vel_x = msg.vy    # NED Vy -> Gazebo ENU Vx
                curr_vel_y = msg.vx    # NED Vx -> Gazebo ENU Vy
                curr_vel_z = -msg.vz   # NED Vz -> Gazebo ENU Vz
                last_telem_time = now_t
                updated = True
            elif mtype == 'ATTITUDE':
                curr_roll = msg.roll
                curr_pitch = msg.pitch
                curr_yaw_ned = msg.yaw  # MAVLink NED yaw in radians
                last_telem_time = now_t
                updated = True
        return updated

    # 1. Pre-stream setpoints for 1.5s
    print("1. Pre-streaming setpoints for 1.5s...")
    pre_start = time.time()
    while time.time() - pre_start < 1.5:
        send_pos_target(target_x, target_y, target_z)
        update_telemetry()
        time.sleep(0.05)

    # 2. Arm & OFFBOARD mode
    print("2. Arming vehicle...")
    master.mav.command_long_send(target_sys, target_comp, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 21968, 0, 0, 0, 0, 0)
    time.sleep(0.2)

    print("3. Requesting OFFBOARD mode...")
    master.mav.command_long_send(target_sys, target_comp, mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, 1, 6, 0, 0, 0, 0, 0)
    time.sleep(0.5)

    # 4. Takeoff & Climb Phase (12.0s to ensure clean Z settling at 2.5m)
    print(f"4. Climbing to hover altitude ({target_z}m) for 12.0s...")
    climb_start = time.time()
    while time.time() - climb_start < 12.0:
        now_t = time.time()
        send_pos_target(target_x, target_y, target_z)
        update_telemetry()
        telem_age = now_t - last_telem_time

        writer.writerow({
            'timestamp_total_sec': f"{now_t:.6f}",
            'maneuver_state': 'CLIMB',
            'maneuver_time_sec': '0.000',
            'cmd_roll_deg': '0.00', 'cmd_pitch_deg': '0.00', 'cmd_yaw_deg': '0.00', 'cmd_thrust': '0.65',
            'cmd_q_w': '1.0', 'cmd_q_x': '0.0', 'cmd_q_y': '0.0', 'cmd_q_z': '0.0',
            'pos_x': f"{curr_pos_x:.4f}", 'pos_y': f"{curr_pos_y:.4f}", 'pos_z': f"{curr_pos_z:.4f}",
            'vel_x': f"{curr_vel_x:.4f}", 'vel_y': f"{curr_vel_y:.4f}", 'vel_z': f"{curr_vel_z:.4f}",
            'pos_error_x': f"{curr_pos_x - target_x:.4f}", 'pos_error_y': f"{curr_pos_y - target_y:.4f}", 'pos_error_z': f"{curr_pos_z - target_z:.4f}",
            'telem_age_sec': f"{telem_age:.4f}"
        })
        telem_file.flush()
        time.sleep(0.02)

    # 5. Transition to Maneuver: Capture initial hold position and initial NED heading psi_0_ned
    update_telemetry()
    hold_x = curr_pos_x
    hold_y = curr_pos_y
    hold_z = curr_pos_z
    psi_0_ned = curr_yaw_ned  # Initial NED heading in radians
    psi_0_ned_deg = math.degrees(psi_0_ned)
    psi_enu = (math.pi / 2.0) - psi_0_ned

    print("------------------------------------------------------------")
    print(f"[MANEUVER START] Captured Hold Target: ({hold_x:.3f}, {hold_y:.3f}, {hold_z:.3f} m)")
    print(f"[MANEUVER START] Captured Locked Heading psi_0 (NED): {psi_0_ned_deg:.2f} deg ({psi_0_ned:.4f} rad)")
    print("------------------------------------------------------------")

    # Maneuver Loop Parameters
    maneuver_start = time.time()
    hover_thrust_base = 0.65

    # AC Velocity-Damped Gains with Kd_y=7.5 for active lateral velocity suppression
    Kp_x = 5.0; Kd_x = 5.0
    Kp_y = 2.0; Kd_y = 7.5   # Strong velocity damping suppresses lateral excursion < 0.18m
    Kp_z = 0.60; Kd_z = 0.30

    while time.time() - maneuver_start < maneuver_duration:
        now_t = time.time()
        m_time = now_t - maneuver_start
        update_telemetry()
        telem_age = now_t - last_telem_time

        # World ENU position & velocity errors
        err_x = curr_pos_x - hold_x
        err_y = curr_pos_y - hold_y
        err_z = curr_pos_z - hold_z

        # Transform world ENU errors to body frame errors using ENU heading psi_enu
        sin_psi = math.sin(psi_enu)
        cos_psi = math.cos(psi_enu)

        err_fwd = err_x * cos_psi + err_y * sin_psi
        err_right = err_x * sin_psi - err_y * cos_psi
        vel_fwd = curr_vel_x * cos_psi + curr_vel_y * sin_psi
        vel_right = curr_vel_x * sin_psi - curr_vel_y * cos_psi

        # Target sinusoidal ROLL oscillation
        roll_osc = roll_amp_deg * math.sin(2.0 * math.pi * freq_hz * m_time)

        # Body-frame position feedback
        corr_pitch = max(-3.0, min(3.0, Kp_x * err_fwd + Kd_x * vel_fwd))
        corr_roll = max(-3.5, min(3.5, -Kp_y * err_right - Kd_y * vel_right))

        cmd_roll = roll_osc + corr_roll
        cmd_pitch = corr_pitch
        cmd_yaw_deg = psi_0_ned_deg

        # Total-force altitude thrust compensation
        cos_roll = max(0.4, math.cos(math.radians(cmd_roll)))
        cos_pitch = max(0.4, math.cos(math.radians(cmd_pitch)))
        tilt_factor = 1.0 / (cos_roll * cos_pitch)

        z_correction = -Kp_z * err_z - Kd_z * curr_vel_z
        cmd_thrust = min(0.85, max(0.30, (hover_thrust_base + z_correction) * tilt_factor))

        # Stream SET_ATTITUDE_TARGET
        q_cmd = send_att_target(cmd_roll, cmd_pitch, cmd_yaw_deg, cmd_thrust)

        # Log telemetry
        writer.writerow({
            'timestamp_total_sec': f"{now_t:.6f}",
            'maneuver_state': 'MANEUVER',
            'maneuver_time_sec': f"{m_time:.3f}",
            'cmd_roll_deg': f"{cmd_roll:.2f}",
            'cmd_pitch_deg': f"{cmd_pitch:.2f}",
            'cmd_yaw_deg': f"{cmd_yaw_deg:.2f}",
            'cmd_thrust': f"{cmd_thrust:.4f}",
            'cmd_q_w': f"{q_cmd[0]:.4f}", 'cmd_q_x': f"{q_cmd[1]:.4f}", 'cmd_q_y': f"{q_cmd[2]:.4f}", 'cmd_q_z': f"{q_cmd[3]:.4f}",
            'pos_x': f"{curr_pos_x:.4f}", 'pos_y': f"{curr_pos_y:.4f}", 'pos_z': f"{curr_pos_z:.4f}",
            'vel_x': f"{curr_vel_x:.4f}", 'vel_y': f"{curr_vel_y:.4f}", 'vel_z': f"{curr_vel_z:.4f}",
            'pos_error_x': f"{err_x:.4f}", 'pos_error_y': f"{err_y:.4f}", 'pos_error_z': f"{err_z:.4f}",
            'telem_age_sec': f"{telem_age:.4f}"
        })
        telem_file.flush()
        time.sleep(0.02)

    # 6. Request LAND
    print("6. Maneuver complete. Requesting LAND...")
    master.mav.command_long_send(target_sys, target_comp, mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0)
    time.sleep(3.0)

    telem_file.close()
    print("Telemetry log saved -> results/roll_validation_telemetry.csv\n")


if __name__ == '__main__':
    main()
