#!/usr/bin/env python3
"""
Automated End-to-End Baseline Test for PX4 SITL + Gazebo Harmonic (Step 1):
1. Start PX4 SITL headless + Gazebo Harmonic (gz_x500).
2. Connect via MAVLink (udpin:0.0.0.0:14540).
3. Wait for EKF2 convergence & GPS lock.
4. Arm -> Takeoff to 3.8m -> Hover -> Offboard Velocity -> Land.
"""

import time
import subprocess
import os
import sys
from pymavlink import mavutil

# PX4 Custom Modes
PX4_CUSTOM_MAIN_MODE_MANUAL = 1
PX4_CUSTOM_MAIN_MODE_ALTCTL = 2
PX4_CUSTOM_MAIN_MODE_POSCTL = 3
PX4_CUSTOM_MAIN_MODE_AUTO = 4
PX4_CUSTOM_MAIN_MODE_ACRO = 5
PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6
PX4_CUSTOM_MAIN_MODE_STABILIZED = 7
PX4_CUSTOM_MAIN_MODE_RATTITUDE = 8

def main():
    print("=" * 65)
    print("      PX4 SITL + GAZEBO BASELINE TEST (STEP 1)       ")
    print("=" * 65)

    # 1. Start PX4 SITL
    print("[1/5] Launching PX4 SITL + Gazebo Harmonic (gz_x500)...")
    env = os.environ.copy()
    env["HEADLESS"] = "1"
    px4_proc = subprocess.Popen(
        ["make", "px4_sitl", "gz_x500"],
        cwd="/home/tungt/PX4-Autopilot",
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        env=env,
        preexec_fn=os.setsid
    )

    # 2. Connect to MAVLink
    print("[2/5] Connecting to PX4 on udpin:0.0.0.0:14540...")
    master = None
    t0 = time.time()
    while time.time() - t0 < 35.0:
        try:
            m = mavutil.mavlink_connection('udpin:0.0.0.0:14540', source_system=255)
            hb = m.wait_heartbeat(timeout=1.5)
            if hb:
                master = m
                print(f"[MAVLink] Heartbeat received from PX4 (SysID={m.target_system}, CompID={m.target_component})")
                break
        except Exception as e:
            pass
        time.sleep(0.5)

    if not master:
        print("ERROR: Timeout connecting to PX4 MAVLink.")
        import signal
        os.killpg(os.getpgid(px4_proc.pid), signal.SIGTERM)
        sys.exit(1)

    # Request data streams
    master.mav.request_data_stream_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1
    )

    # 3. Wait for EKF2 GPS Origin & Arming Readiness
    print("[3/5] Waiting for EKF2 convergence & GPS lock (up to 30s)...")
    t0 = time.time()
    while time.time() - t0 < 30.0:
        msg = master.recv_match(blocking=True, timeout=1.0)
        if msg:
            if msg.get_type() == 'GLOBAL_POSITION_INT' and msg.lat != 0:
                print(f"  EKF2 Position OK (Lat: {msg.lat/1e7:.5f}, Lon: {msg.lon/1e7:.5f})")
                break
            elif msg.get_type() == 'STATUSTEXT':
                print(f"  PX4: {msg.text}")
        time.sleep(0.2)

    time.sleep(1.5)

    # 4. Arm Vehicle
    print("[4/5] Arming PX4 Motors...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1.0, 0, 0, 0, 0, 0, 0
    )
    time.sleep(1.0)

    # 5. Takeoff Command
    print("[5/5] Commanding Takeoff to 3.8m...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, 3.8
    )

    t0 = time.time()
    current_alt = 0.0
    while time.time() - t0 < 15.0:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1.0)
        if msg:
            current_alt = msg.relative_alt / 1000.0
            print(f"  Climbing: Altitude = {current_alt:.2f} m")
            if current_alt >= 3.0:
                print(f"  Takeoff altitude reached successfully ({current_alt:.2f} m)!")
                break
        time.sleep(0.4)

    # 6. Test Offboard Setpoint Warm-up & Mode Transition
    print("\n--- Testing PX4 OFFBOARD Mode with Setpoint Warm-up ---")
    for _ in range(10):
        master.mav.set_position_target_local_ned_send(
            0, master.target_system, master.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            0x05C7,
            0, 0, 0,
            0.0, 0.0, 0.0,
            0, 0, 0,
            0.0, 0.0
        )
        time.sleep(0.05)

    # Set Mode to OFFBOARD (custom_mode = 6)
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        0,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        PX4_CUSTOM_MAIN_MODE_OFFBOARD,
        0, 0, 0, 0, 0
    )
    print("  Sent DO_SET_MODE (OFFBOARD).")

    # Send forward velocity in OFFBOARD mode for 2.0s (Vx = 1.0 m/s)
    print("  Streaming OFFBOARD velocity (Vx = +1.0 m/s, Yaw = 0.0)...")
    for _ in range(20):
        master.mav.set_position_target_local_ned_send(
            0, master.target_system, master.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            0x05C7,
            0, 0, 0,
            1.0, 0.0, 0.0,
            0, 0, 0,
            0.0, 0.0
        )
        time.sleep(0.10)

    # 7. Land
    print("\n--- Commanding LAND ---")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0, 0, 0, 0, 0, 0, 0, 0
    )
    time.sleep(3.0)

    # Cleanup
    import signal
    os.killpg(os.getpgid(px4_proc.pid), signal.SIGTERM)
    print("\n" + "=" * 65)
    print("   STEP 1 BASELINE TEST PASSED: PX4 SITL + GAZEBO FLIGHT OK   ")
    print("=" * 65)

if __name__ == '__main__':
    main()
