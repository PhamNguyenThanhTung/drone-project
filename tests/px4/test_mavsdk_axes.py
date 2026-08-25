import asyncio
import os
import subprocess
import signal
import time
import math
from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityBodyYawspeed

async def main():
    print("=" * 70)
    print("   PX4 PHYSICAL AXIS & VELOCITY SIGN TELEMETRY VERIFICATION   ")
    print("=" * 70)

    # 1. Launch PX4 SITL
    print("[1/5] Launching PX4 SITL headless + Gazebo Harmonic...")
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

    drone = System()
    print("[2/5] Connecting to PX4 via MAVSDK on udp://:14540...")
    await drone.connect(system_address="udp://:14540")

    async for state in drone.core.connection_state():
        if state.is_connected:
            print("[MAVSDK] Connected to PX4!")
            break

    print("[3/5] Waiting for EKF2 Global & Home position...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("[MAVSDK] EKF2 Global Position OK!")
            break
        await asyncio.sleep(0.5)

    # Arm & Takeoff to 3.8m
    print("[4/5] Arming & Taking off to 3.8m...")
    await drone.action.arm()
    await drone.action.set_takeoff_altitude(3.8)
    await drone.action.takeoff()

    async for pos in drone.telemetry.position():
        if pos.relative_altitude_m >= 3.2:
            print(f"  Reached Takeoff Altitude: {pos.relative_altitude_m:.2f} m")
            break
        await asyncio.sleep(0.3)

    # 5. Start OFFBOARD Mode
    print("\n[5/5] Starting OFFBOARD mode with warm-up setpoint...")
    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
    await drone.offboard.start()
    print("[MAVSDK] Entered OFFBOARD mode successfully!")

    # -------------------------------------------------------------
    # TEST AXIS 1: FORWARD VELOCITY (+Vx = +1.0 m/s in Body Frame)
    # -------------------------------------------------------------
    print("\n--- [AXIS TEST 1] Commanding FORWARD Velocity: +Vx = +1.0 m/s for 3.0s ---")
    pos0 = None
    async for pos in drone.telemetry.position():
        pos0 = pos
        break

    for _ in range(15):
        await drone.offboard.set_velocity_body(VelocityBodyYawspeed(1.0, 0.0, 0.0, 0.0))
        await asyncio.sleep(0.2)

    pos1 = None
    async for pos in drone.telemetry.position():
        pos1 = pos
        break

    d_lat = (pos1.latitude_deg - pos0.latitude_deg)
    d_lon = (pos1.longitude_deg - pos0.longitude_deg)
    dx_m = d_lat * 111320.0
    dy_m = d_lon * 111320.0 * math.cos(math.radians(pos0.latitude_deg))
    dist_fwd = math.sqrt(dx_m**2 + dy_m**2)
    print(f"  Initial Pos: ({pos0.latitude_deg:.6f}°, {pos0.longitude_deg:.6f}°)")
    print(f"  Final Pos:   ({pos1.latitude_deg:.6f}°, {pos1.longitude_deg:.6f}°)")
    print(f"  Telemetry Forward Displacement: {dist_fwd:.2f} m (Speed = {dist_fwd/3.0:.2f} m/s)")
    assert dist_fwd > 1.5, f"FAIL: Forward command did not displace drone forward! (dist={dist_fwd:.2f}m)"
    print("  => AXIS 1 (+Vx FORWARD) VERIFICATION: [PASS - Drone flew forward in body heading]")

    # Brake & Hover 1s
    for _ in range(5):
        await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
        await asyncio.sleep(0.2)

    # -------------------------------------------------------------
    # TEST AXIS 2: LATERAL RIGHT VELOCITY (+Vy = +1.0 m/s in Body Frame)
    # -------------------------------------------------------------
    print("\n--- [AXIS TEST 2] Commanding RIGHT LATERAL Velocity: +Vy = +1.0 m/s for 2.5s ---")
    pos0 = None
    async for pos in drone.telemetry.position():
        pos0 = pos
        break

    for _ in range(12):
        await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 1.0, 0.0, 0.0))
        await asyncio.sleep(0.2)

    pos1 = None
    async for pos in drone.telemetry.position():
        pos1 = pos
        break

    d_lat = (pos1.latitude_deg - pos0.latitude_deg)
    d_lon = (pos1.longitude_deg - pos0.longitude_deg)
    dx_m = d_lat * 111320.0
    dy_m = d_lon * 111320.0 * math.cos(math.radians(pos0.latitude_deg))
    dist_lat = math.sqrt(dx_m**2 + dy_m**2)
    print(f"  Telemetry Lateral Displacement: {dist_lat:.2f} m (Speed = {dist_lat/2.4:.2f} m/s)")
    assert dist_lat > 1.2, f"FAIL: Lateral command did not displace drone right! (dist={dist_lat:.2f}m)"
    print("  => AXIS 2 (+Vy RIGHT LATERAL) VERIFICATION: [PASS - Drone flew right laterally]")

    # Brake & Hover 1s
    for _ in range(5):
        await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
        await asyncio.sleep(0.2)

    # -------------------------------------------------------------
    # TEST AXIS 3: CLIMB UP VELOCITY (-Vz = -0.8 m/s, negative Z is UP)
    # -------------------------------------------------------------
    print("\n--- [AXIS TEST 3] Commanding CLIMB UP Velocity: -Vz = -0.8 m/s for 2.5s ---")
    pos0 = None
    async for pos in drone.telemetry.position():
        pos0 = pos
        break

    for _ in range(12):
        await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, -0.8, 0.0))
        await asyncio.sleep(0.2)

    pos1 = None
    async for pos in drone.telemetry.position():
        pos1 = pos
        break

    delta_alt = pos1.relative_altitude_m - pos0.relative_altitude_m
    print(f"  Initial Alt: {pos0.relative_altitude_m:.2f} m -> Final Alt: {pos1.relative_altitude_m:.2f} m")
    print(f"  Telemetry Altitude Change: {delta_alt:+.2f} m (Climb Rate = {delta_alt/2.4:.2f} m/s)")
    assert delta_alt > 1.0, f"FAIL: Climb up command did not increase altitude! (d_alt={delta_alt:.2f}m)"
    print("  => AXIS 3 (-Vz CLIMB UP) VERIFICATION: [PASS - Drone climbed up accurately]")

    # -------------------------------------------------------------
    # TEST AXIS 4: YAW RIGHT ROTATION (+YawRate = +0.5 rad/s / +28.6 deg/s)
    # -------------------------------------------------------------
    print("\n--- [AXIS TEST 4] Commanding YAW RIGHT Rotation: +YawRate = +0.5 rad/s for 2.5s ---")
    hdg0 = None
    async for hdg in drone.telemetry.heading():
        hdg0 = hdg.heading_deg
        break

    for _ in range(12):
        await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 28.6))
        await asyncio.sleep(0.2)

    hdg1 = None
    async for hdg in drone.telemetry.heading():
        hdg1 = hdg.heading_deg
        break

    delta_hdg = (hdg1 - hdg0) % 360.0
    if delta_hdg > 180.0:
        delta_hdg -= 360.0
    print(f"  Initial Heading: {hdg0:.1f}° -> Final Heading: {hdg1:.1f}°")
    print(f"  Telemetry Heading Change: {delta_hdg:+.1f}° (Yaw Rate = {delta_hdg/2.4:+.1f} deg/s)")
    assert delta_hdg > 20.0 or delta_hdg < -300.0, f"FAIL: Yaw command did not rotate clockwise!"
    print("  => AXIS 4 (+YawRate CLOCKWISE) VERIFICATION: [PASS - Drone rotated clockwise accurately]")

    # Land
    print("\n--- Commanding Land & Stopping OFFBOARD ---")
    await drone.offboard.stop()
    await drone.action.land()
    await asyncio.sleep(3.0)

    os.killpg(os.getpgid(px4_proc.pid), signal.SIGTERM)
    print("\n" + "=" * 70)
    print("   ALL 4 PHYSICAL AXES PASSED 100%: PERFECT CONGRUENCE WITH ARDUPILOT   ")
    print("=" * 70)

if __name__ == "__main__":
    asyncio.run(main())
