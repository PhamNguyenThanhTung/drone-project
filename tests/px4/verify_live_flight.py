#!/usr/bin/env python3
"""
Verify Live Flight Trajectory in PX4 SITL + Gazebo Harmonic:
- Starts PX4 SITL with person_tracking_no_trees world.
- Connects MAVLink.
- Gates on EKF2 global position (not a fixed sleep), arms with ACK check,
  commands Takeoff to 4.0m and waits until >= 3.8m is reached.
- Streams live telemetry every 0.5s (Altitude, Velocity, Coordinates).
- Exits 0 on success, 1 on any failure (connection, arm, takeoff, altitude).
"""

import os
import sys
import time
import subprocess
from pymavlink import mavutil

# PX4 ends the takeoff phase once the vehicle is within NAV_MC_ALT_RAD
# (0.8 m default) of the commanded altitude, then loiters there. Command
# 5.0 m so the "reached" threshold (5.0 - 0.8 = 4.2 m) clears the 3.8 m
# success bar even with a slow motor spoolup.
TAKEOFF_TARGET_M = 5.0
SUCCESS_ALT_M = 3.8
CLIMB_TIMEOUT_S = 45.0


def wait_for_command_ack(master, command, params, attempts=5, ack_timeout=3.0):
    """Send COMMAND_LONG until PX4 ACKs it with MAV_RESULT_ACCEPTED."""
    for attempt in range(1, attempts + 1):
        master.mav.command_long_send(
            master.target_system, master.target_component,
            command, 0, *params
        )
        ack = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=ack_timeout)
        if ack is None:
            print(f"  attempt {attempt}/{attempts}: no ACK for command {command}, retrying...")
            continue
        if ack.command != command:
            # Stale ACK for an earlier command; keep waiting for the right one.
            deadline = time.time() + ack_timeout
            while time.time() < deadline:
                ack = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=deadline - time.time())
                if ack and ack.command == command:
                    break
            if ack is None or ack.command != command:
                print(f"  attempt {attempt}/{attempts}: ACK for command {command} not received, retrying...")
                continue
        result = ack.result
        if result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            print(f"  Command {command} ACCEPTED (attempt {attempt})")
            return True
        print(f"  attempt {attempt}/{attempts}: command {command} rejected (result={result}), retrying...")
        time.sleep(1.0)
    return False


def main():
    print("=== 1. Starting PX4 SITL + Gazebo Harmonic ===")
    subprocess.run("pkill -9 -x px4 2>/dev/null || true", shell=True)
    subprocess.run("pkill -9 -f 'gz si[m]' 2>/dev/null || true", shell=True)
    time.sleep(1)

    env = os.environ.copy()
    env['HEADLESS'] = '1'
    env['PX4_GZ_WORLD'] = 'person_tracking_no_trees'
    env['GZ_SIM_RESOURCE_PATH'] = (
        "/home/tungt/drone-project/gazebo/models:"
        "/home/tungt/drone-project/gazebo/worlds:"
        "/home/tungt/PX4-Autopilot/Tools/simulation/gz/models:"
        "/home/tungt/PX4-Autopilot/Tools/simulation/gz/worlds"
    )

    # stdout must go to a file: an unread PIPE fills after ~64KB of PX4
    # console output and blocks the sim mid-flight.
    with open('/tmp/verify_live_sitl.log', 'w') as sitl_log:
        px4_proc = subprocess.Popen(
            ["make", "px4_sitl", "gz_x500"],
            cwd="/home/tungt/PX4-Autopilot",
            env=env,
            stdout=sitl_log,
            stderr=subprocess.STDOUT
        )

        try:
            success = run_flight_check()
        finally:
            px4_proc.kill()
            subprocess.run("pkill -9 -x px4 2>/dev/null || true", shell=True)
            subprocess.run("pkill -9 -f 'gz si[m]' 2>/dev/null || true", shell=True)

    if success:
        print("RESULT: SUCCESS")
        sys.exit(0)
    print("RESULT: FAILED")
    sys.exit(1)


def run_flight_check():
    print("=== 2. Connecting to PX4 MAVLink at udpin:0.0.0.0:14540 ===")
    master = None
    for i in range(30):
        try:
            cand = mavutil.mavlink_connection('udpin:0.0.0.0:14540', source_system=255)
            if cand.wait_heartbeat(timeout=1.0):
                master = cand
                print(f"Connected! PX4 System ID: {cand.target_system}, Component: {cand.target_component}")
                break
        except Exception:
            pass
        time.sleep(0.5)

    if master is None:
        print("FAILED: Could not connect to PX4 MAVLink.")
        return False

    print("=== 3. Waiting for EKF2 global position (up to 60s, health-gated) ===")
    master.mav.request_data_stream_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1
    )
    deadline = time.time() + 60.0
    ekf_ready = False
    while time.time() < deadline:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1.0)
        if msg and msg.lat != 0 and msg.lon != 0:
            ekf_ready = True
            print(f"EKF2 global position OK after {60.0 - (deadline - time.time()):.1f}s "
                  f"(lat={msg.lat / 1e7:.6f}, lon={msg.lon / 1e7:.6f})")
            break
    if not ekf_ready:
        print("FAILED: EKF2 never produced a global position within 60s.")
        return False

    print("=== 4. Arming Quadcopter (ACK-checked) ===")
    # MAVSDK-style NaN params: PX4 1.14 navigator ACKs NAV_TAKEOFF with a
    # finite param7 but the takeoff task then never generates setpoints
    # (thrust stays 0) and COM_DISARM_PRFLT auto-disarms 10 s later. NaN
    # params + MIS_TAKEOFF_ALT is the combination that actually flies.
    nan = float('nan')
    if not wait_for_command_ack(
            master, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            (1.0, nan, nan, nan, nan, nan, nan)):
        print("FAILED: Arm command was never accepted.")
        return False

    print(f"=== 5. Commanding Auto Takeoff to {TAKEOFF_TARGET_M:.1f} m (ACK-checked) ===")
    master.mav.param_set_send(
        master.target_system, master.target_component,
        b'MIS_TAKEOFF_ALT', TAKEOFF_TARGET_M, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    master.recv_match(type='PARAM_VALUE', blocking=True, timeout=2.0)
    if not wait_for_command_ack(
            master, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            (nan, nan, nan, nan, nan, nan, nan)):
        print("FAILED: Takeoff command was never accepted.")
        return False

    print(f"=== 6. Monitoring climb until {SUCCESS_ALT_M:.1f} m (timeout {CLIMB_TIMEOUT_S:.0f}s) ===")
    start_time = time.time()
    max_alt = 0.0
    reached = False
    while time.time() - start_time < CLIMB_TIMEOUT_S:
        msg = master.recv_match(type=['GLOBAL_POSITION_INT', 'LOCAL_POSITION_NED'],
                                blocking=True, timeout=0.5)
        if msg:
            if msg.get_type() == 'GLOBAL_POSITION_INT':
                rel_alt = msg.relative_alt / 1000.0  # mm to meters
                max_alt = max(max_alt, rel_alt)
                lat = msg.lat / 1e7
                lon = msg.lon / 1e7
                print(f"[{time.time()-start_time:4.1f}s] Telemetry GLOBAL: Alt = {rel_alt:4.2f} m | Lat = {lat:.6f}, Lon = {lon:.6f}")
            elif msg.get_type() == 'LOCAL_POSITION_NED':
                z = -msg.z  # NED z is negative down -> positive up
                max_alt = max(max_alt, z)
                print(f"[{time.time()-start_time:4.1f}s] Telemetry LOCAL:  Z-Alt = {z:4.2f} m | Vx = {msg.vx:+.2f}, Vy = {msg.vy:+.2f}, Vz = {msg.vz:+.2f}")
        if max_alt >= SUCCESS_ALT_M:
            reached = True
            break

    print("\n=======================================================")
    print(f"RESULT: Maximum Altitude Reached = {max_alt:.2f} m "
          f"(Target: {TAKEOFF_TARGET_M:.2f} m, Success: >= {SUCCESS_ALT_M:.2f} m)")
    if reached:
        print(f"SUCCESS: Drone reached {max_alt:.2f} m in {time.time()-start_time:.1f}s and is airborne in Gazebo!")
    else:
        print(f"FAILURE: Altitude only reached {max_alt:.2f} m after {CLIMB_TIMEOUT_S:.0f}s.")
    print("=======================================================")
    return reached


if __name__ == "__main__":
    main()
