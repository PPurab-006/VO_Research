import time
from pymavlink import mavutil

print("Connecting to PX4 MAVLink...")
master = mavutil.mavlink_connection('udp:127.0.0.1:14540')
master.wait_heartbeat()
target_sys = master.target_system if master.target_system != 0 else 1
target_comp = master.target_component if master.target_component != 0 else 1
print(f"Connected to Sys {target_sys}, Comp {target_comp}")

def send_cmd(cmd, p1=0, p2=0, p3=0, p4=0, p5=0, p6=0, p7=0):
    master.mav.command_long_send(
        target_sys, target_comp,
        cmd, 0, p1, p2, p3, p4, p5, p6, p7
    )
    ack = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=3.0)
    if ack:
        print(f"Command {cmd} ACK: Result={ack.result} (0=ACCEPTED, 1=TEMPORARILY_REJECTED, 4=FAILED, 5=IN_PROGRESS, 6=DENIED)")
        if ack.result != 0:
            print(f"  ACK Details: {ack.to_dict()}")
    else:
        print(f"Command {cmd} ACK: TIMEOUT")

# Force arming with param1=1, param2=21968 (force arming magic number in PX4)
print("\n1. Requesting FORCE ARM (p1=1, p2=21968)...")
send_cmd(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, p1=1, p2=21968)

print("\n2. Requesting TAKEOFF...")
send_cmd(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, p7=2.5)

print("\n3. Checking Local Position NED...")
for i in range(10):
    time.sleep(0.5)
    msg = master.recv_match(type='LOCAL_POSITION_NED', blocking=False)
    if msg:
        print(f"[{i+1}] Local Pos Z: {-msg.z:.3f} m | Vz: {-msg.vz:.3f} m/s")

