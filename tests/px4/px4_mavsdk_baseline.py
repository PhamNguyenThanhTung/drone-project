#!/usr/bin/env python3
"""
MAVSDK Automated Flight Test for PX4 SITL + Gazebo Harmonic:
Connect -> Check EKF/GPS -> Arm -> Takeoff -> Offboard Velocity -> Hover -> Land
"""

import asyncio
from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityBodyYawspeed

async def main():
    print("=" * 60)
    print("      MAVSDK AUTOMATED PX4 BASELINE FLIGHT TEST      ")
    print("=" * 60)

    drone = System()
    print("Connecting to PX4 on udp://:14540...")
    await drone.connect(system_address="udp://:14540")

    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"[MAVSDK] Connected to PX4!")
            break

    print("[MAVSDK] Waiting for drone to have a global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("[MAVSDK] EKF2 Global position estimate OK!")
            break
        await asyncio.sleep(0.5)

    print("[MAVSDK] Arming drone...")
    await drone.action.arm()

    print("[MAVSDK] Setting takeoff altitude to 3.8m & Taking off...")
    await drone.action.set_takeoff_altitude(3.8)
    await drone.action.takeoff()

    async for position in drone.telemetry.position():
        alt = position.relative_altitude_m
        print(f"  Climbing: Altitude = {alt:.2f} m")
        if alt >= 3.4:
            print(f"  Reached target takeoff altitude: {alt:.2f} m!")
            break
        await asyncio.sleep(0.5)

    # Test OFFBOARD Mode with Setpoint Warm-up
    print("\n[MAVSDK] Testing OFFBOARD velocity control (Setpoints Warm-up)...")
    # Send initial setpoint before starting offboard
    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
    try:
        await drone.offboard.start()
        print("[MAVSDK] Entered OFFBOARD mode successfully!")
    except OffboardError as error:
        print(f"[MAVSDK] Starting offboard mode failed with error code: {error._result.result}")
        return

    # Forward flight test in OFFBOARD mode (1.0 m/s forward)
    print("[MAVSDK] Sending Forward velocity: Vx = 1.0 m/s for 3s...")
    for _ in range(15):
        await drone.offboard.set_velocity_body(VelocityBodyYawspeed(1.0, 0.0, 0.0, 0.0))
        await asyncio.sleep(0.2)

    # Stop & Hover
    print("[MAVSDK] Sending Hover (Vx=0, Vy=0, Vz=0)...")
    for _ in range(10):
        await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
        await asyncio.sleep(0.2)

    await drone.offboard.stop()
    print("[MAVSDK] Exited OFFBOARD mode.")

    print("\n[MAVSDK] Landing...")
    await drone.action.land()

    async for in_air in drone.telemetry.in_air():
        if not in_air:
            print("[MAVSDK] Landed successfully on ground!")
            break
        await asyncio.sleep(0.5)

    print("\n" + "=" * 60)
    print("   MAVSDK BASELINE TEST PASSED 100% (STEP 1 COMPLETE)!   ")
    print("=" * 60)

if __name__ == '__main__':
    asyncio.run(main())
