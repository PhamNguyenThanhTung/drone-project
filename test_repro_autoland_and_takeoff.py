#!/usr/bin/env python3
"""
Reproduction test for the reported bug:

    "Drone auto-lands after flying for a while, and after landing no key
     (including TAB/T takeoff) makes it fly again."

Mechanism under test (from ulog 2026-08-25/08_06_18 analysis):
  1. The 10 Hz OFFBOARD velocity setpoint stream from motion_arbiter stalls.
  2. After COM_OF_LOSS_T (1.0 s default), PX4 raises the offboard-loss
     failsafe and descends while the stream is gone.
  3. On touchdown PX4 reports "Landing detected" and auto-disarms
     ("Disarmed by landing", COM_DISARM_LAND = 2 s).
  4. motion_arbiter never learns about any of this: its internal
     `is_airborne` flag stays True, so every TAKEOFF request is swallowed
     by the guard in on_flight_action() ("already airborne").

The stall from step 1 is injected deterministically by SIGSTOP-ing the
arbiter for the duration of the landing (same observable effect as the
load-induced stalls seen in the field log: zero setpoints for > 1 s),
so the test does not depend on machine load to reproduce.

Verdict
-------
  exit 0 : BUG REPRODUCED      (drone disarmed on ground, TAB/T ignored)
  exit 3 : NOT REPRODUCED      (drone re-armed and climbed -> bug gone)
  exit 2 : infrastructure failure (boot timeout etc.)

Run:  bash -c 'source /opt/ros/humble/setup.bash && \
        python3 test_repro_autoland_retakeoff.py'
"""

import os
import shutil
import signal
import statistics
import subprocess
import sys
import time

from pymavlink import mavutil

PX4_DIR = '/home/tungt/PX4-Autopilot'
ARBITER = '/home/tungt/drone-project/motion_arbiter.py'
PX4_LOG = '/tmp/repro_px4.log'
ARB_LOG = '/tmp/repro_arbiter.log'

MAIN_MODE_OFFBOARD = 6

px4_proc = None
arb_proc = None


def log(msg):
    print(f'[repro {time.strftime("%H:%M:%S")}] {msg}', flush=True)


def cleanup():
    """Resume the arbiter if frozen and tear the stack down."""
    global px4_proc, arb_proc
    for proc, name in ((arb_proc, 'arbiter'), (px4_proc, 'px4')):
        if proc is not None and proc.poll() is None:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGCONT)
            except (ProcessLookupError, PermissionError):
                pass
    subprocess.run(['pkill', '-9', '-x', 'px4'], check=False)
    subprocess.run(['pkill', '-9', '-f', 'gz si[m]'], check=False)
    subprocess.run(['pkill', '-9', '-f', 'motion_arbite[r]'], check=False)


def start_stack():
    global px4_proc, arb_proc
    log('launching PX4 SITL (headless) + Gazebo...')
    px4_proc = subprocess.Popen(
        ['make', 'px4_sitl', 'gz_x500'],
        cwd=PX4_DIR, stdout=open(PX4_LOG, 'w'), stderr=subprocess.STDOUT,
        env={**os.environ, 'HEADLESS': '1', 'GZ_VERSION': 'harmonic'},
        start_new_session=True,
    )
    time.sleep(3)

    log('launching MotionArbiter (auto_takeoff=True)...')
    arb_proc = subprocess.Popen(
        ['python3', ARBITER, '--ros-args',
         '-p', 'takeoff_alt:=3.800000', '-p', 'auto_takeoff:=True',
         '-p', 'mavlink:=udpin:0.0.0.0:14540'],
        stdout=open(ARB_LOG, 'w'), stderr=subprocess.STDOUT,
        start_new_session=True,
    )


class VehicleMonitor:
    """Passive MAVLink observer on the GCS port (14550)."""

    def __init__(self):
        self.mav = mavutil.mavlink_connection(
            'udpin:0.0.0.0:14550', source_system=250)

    def wait_heartbeat(self, timeout):
        deadline = time.time() + timeout
        while time.time() < deadline:
            hb = self.mav.recv_match(type='HEARTBEAT', blocking=True, timeout=1.0)
            if hb is not None and hb.get_srcSystem() == 1:
                return True
        return False

    def sample(self, seconds):
        """Collect (armed, main_mode, z) observations for a wall-time span."""
        end = time.time() + seconds
        out = []
        while time.time() < end:
            msg = self.mav.recv_match(blocking=True, timeout=0.25)
            if msg is None:
                continue
            mtype = msg.get_type()
            if mtype == 'HEARTBEAT' and msg.get_srcSystem() == 1:
                armed = bool(int(msg.base_mode) &
                             mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                main_mode = (int(msg.custom_mode) >> 16) & 0xFF
                out.append((time.time(), 'armed', armed))
                out.append((time.time(), 'mode', main_mode))
            elif mtype == 'LOCAL_POSITION_NED':
                out.append((time.time(), 'z', float(msg.z)))
        return out


def latest(samples, kind):
    vals = [v for _, k, v in samples if k == kind]
    return vals[-1] if vals else None


def main():
    if shutil.which('ros2') is None and 'humble' not in ''.join(sys.path):
        pass  # rclpy import below is the real gate

    monitor = VehicleMonitor()

    start_stack()

    log('waiting for PX4 heartbeat (up to 90s)...')
    if not monitor.wait_heartbeat(90):
        log('FATAL: no PX4 heartbeat; see ' + PX4_LOG)
        return 2

    # Ground reference: median local z while still disarmed on the pad.
    ground_z = statistics.median(
        [v for _, k, v in monitor.sample(4.0) if k == 'z'] or [0.0])
    log(f'ground reference z = {ground_z:.2f} m')

    # ---------------- Phase A: organic auto-takeoff ----------------
    log('PHASE A: waiting for autonomous takeoff to OFFBOARD cruise...')
    deadline = time.time() + 90
    airborne = False
    while time.time() < deadline:
        s = monitor.sample(0.5)
        armed, mode, z = latest(s, 'armed'), latest(s, 'mode'), latest(s, 'z')
        if (armed is True and mode == MAIN_MODE_OFFBOARD
                and z is not None and z < ground_z - 2.0):
            airborne = True
            break
        if px4_proc.poll() is not None or arb_proc.poll() is not None:
            log(f'FATAL: a component died (px4={px4_proc.poll()}, '
                f'arbiter={arb_proc.poll()})')
            return 2
    if not airborne:
        log('FATAL: takeoff never completed; see ' + ARB_LOG)
        return 2
    log(f'airborne in OFFBOARD (z={z:.2f} m). Hovering 5 s...')

    # ---------------- Phase B: inject the setpoint stall ----------------
    time.sleep(5.0)
    log('PHASE B: freezing arbiter (SIGSTOP) -> offboard stream stops...')
    os.killpg(os.getpgid(arb_proc.pid), signal.SIGSTOP)

    landed = False
    mode_timeline = []
    deadline = time.time() + 90
    last_mode = None
    while time.time() < deadline:
        s = monitor.sample(0.5)
        armed, mode, z = latest(s, 'armed'), latest(s, 'mode'), latest(s, 'z')
        if mode != last_mode and mode is not None:
            mode_timeline.append(mode)
            last_mode = mode
        if armed is False and z is not None and z > ground_z - 0.35:
            landed = True
            break
    log(f'release condition: landed={landed}, mode timeline={mode_timeline}')

    log('resuming arbiter (SIGCONT)...')
    os.killpg(os.getpgid(arb_proc.pid), signal.SIGCONT)
    if not landed:
        log('FATAL: drone did not land during the injected stall.')
        return 2
    log('PX4 is DISARMED on the ground (auto-disarm after landing).')

    # Let the arbiter resume its (now pointless) streaming for a moment,
    # mirroring the real incident where it kept running unaware.
    time.sleep(2.0)

    # ---------------- Phase C: press TAB/T like the user did ----------------
    log('PHASE C: publishing TAKEOFF x5 (simulating TAB/T spam)...')
    import rclpy
    from std_msgs.msg import String
    rclpy.init()
    node = rclpy.create_node('repro_test_driver')
    pub = node.create_publisher(String, '/teleop/flight_action', 10)
    for i in range(5):
        msg = String()
        msg.data = 'TAKEOFF'
        pub.publish(msg)
        log(f'  published TAKEOFF #{i + 1}')
        end = time.time() + 0.7
        while time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
    node.destroy_node()
    rclpy.shutdown()

    # ---------------- Phase D: verdict ----------------
    log('PHASE D: watching 15 s for any re-arm...')
    re_arm_time = None
    deadline = time.time() + 15
    while time.time() < deadline:
        s = monitor.sample(0.5)
        if latest(s, 'armed') is True:
            re_arm_time = time.time()
            break

    try:
        with open(ARB_LOG) as f:
            ignored = sum(
                1 for line in f if 'already airborne' in line.lower())
    except OSError:
        ignored = -1

    if re_arm_time is None:
        log(f'RESULT: BUG REPRODUCED — drone stayed disarmed on the ground. '
            f'TAKEOFF requests swallowed by stale flag '
            f'(arbiter logged "already airborne" x{ignored}).')
        return 0

    log(f'RESULT: NOT REPRODUCED — drone re-armed after '
        f'{re_arm_time - deadline + 15:.1f}s and climbed.')
    return 3


if __name__ == '__main__':
    code = 2
    try:
        code = main()
    finally:
        cleanup()
        log(f'exiting with {code}')
    sys.exit(code)
