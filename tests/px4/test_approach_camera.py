#!/usr/bin/env python3
"""
Camera-driven approach test in PX4 SITL + Gazebo Harmonic.

Regression test for the "YOLO det_rate=0%" failure: the person_tracking_no_trees
actor spends most of its loop 20-30 m away, which is outside the x500 camera's
visible ground patch (~0.2-17.4 m ahead at 3.8 m hover, 37 deg down pitch).
This test uses the person_tracking_approach world, whose actor paces 3-8 m
ahead of the drone spawn, so the person is inside the FOV right after takeoff.

Flow:
- Starts PX4 SITL (person_tracking_approach) + ros_gz_bridge + the real
  yolo_detector_node.
- Gates on EKF2 global position, arms and takes off with ACK checks.
- Asserts /tracking/error flows within 45 s of reaching 3.2 m and keeps
  flowing (>= 10 messages in the following 20 s).

Independent of motion_arbiter and the HUD so failures are easy to attribute.
Exits 0 on success, 1 on any failure. Evidence frames land in
/tmp/approach_test_frames.
"""

import os
import sys
import time
import threading
import subprocess

import cv2
import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from pymavlink import mavutil

# Command 5.0 m: PX4 ends the takeoff phase once the vehicle is within
# NAV_MC_ALT_RAD (0.8 m default) of the target, then loiters there — so a
# 4.0 m command only ever holds ~3.2 m.
TAKEOFF_TARGET_M = 5.0
TRACKING_GATE_ALT_M = 3.2
FIRST_ERROR_TIMEOUT_S = 45.0
COLLECTION_WINDOW_S = 20.0
MIN_ERROR_MESSAGES = 10
FRAME_DIR = '/tmp/approach_test_frames'
SITL_LOG = '/tmp/approach_test_sitl.log'
BRIDGE_LOG = '/tmp/approach_test_bridge.log'
YOLO_LOG = '/tmp/approach_test_yolo.log'

PROJECT = '/home/tungt/drone-project'


def wait_for_command_ack(master, command, params, attempts=5, ack_timeout=3.0):
    """Send COMMAND_LONG until PX4 ACKs it with MAV_RESULT_ACCEPTED."""
    for attempt in range(1, attempts + 1):
        master.mav.command_long_send(
            master.target_system, master.target_component,
            command, 0, *params
        )
        deadline = time.time() + ack_timeout
        ack = None
        while time.time() < deadline:
            ack = master.recv_match(type='COMMAND_ACK', blocking=True,
                                    timeout=max(0.1, deadline - time.time()))
            if ack and ack.command == command:
                break
        if ack is None or ack.command != command:
            print(f"  attempt {attempt}/{attempts}: no ACK for command {command}, retrying...")
            continue
        if ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            print(f"  Command {command} ACCEPTED (attempt {attempt})")
            return True
        print(f"  attempt {attempt}/{attempts}: command {command} rejected (result={ack.result}), retrying...")
        time.sleep(1.0)
    return False


class Observer(Node):
    """Subscribes camera + tracking error; stores evidence, no inference."""

    def __init__(self):
        super().__init__('approach_test_observer')
        self.bridge = CvBridge()
        self.n_errors = 0
        self.first_error_t = None
        self.last_error = None
        self.last_frame = None
        self.frame_lock = threading.Lock()
        os.makedirs(FRAME_DIR, exist_ok=True)

        err_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                             history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Point, '/tracking/error', self.on_error, err_qos)
        cam_qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE,
                             history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Image, '/camera/image_raw', self.on_image, cam_qos)

    def on_error(self, msg):
        self.n_errors += 1
        if self.first_error_t is None:
            self.first_error_t = time.time()
        self.last_error = (msg.x, msg.y)

    def on_image(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception:  # noqa: BLE001
            return
        with self.frame_lock:
            self.last_frame = frame

    def save_frame(self, tag):
        with self.frame_lock:
            frame = None if self.last_frame is None else self.last_frame.copy()
        if frame is not None:
            cv2.imwrite('%s/%s.png' % (FRAME_DIR, tag), frame)


def main():
    print('=' * 70)
    print('   CAMERA-DRIVEN APPROACH TEST (person_tracking_approach world)   ')
    print('=' * 70)

    subprocess.run("pkill -9 -x px4 2>/dev/null || true", shell=True)
    subprocess.run("pkill -9 -f 'gz si[m]' 2>/dev/null || true", shell=True)
    subprocess.run("pkill -9 -f yolo_detector_no[d] 2>/dev/null || true", shell=True)
    subprocess.run("pkill -9 -f 'parameter_bridge /camera' 2>/dev/null || true", shell=True)
    time.sleep(1)

    env = os.environ.copy()
    env['HEADLESS'] = '1'
    env['PX4_GZ_WORLD'] = 'person_tracking_approach'
    env['GZ_SIM_RESOURCE_PATH'] = (
        f"{PROJECT}/gazebo/models:{PROJECT}/gazebo/worlds:"
        "/home/tungt/PX4-Autopilot/Tools/simulation/gz/models:"
        "/home/tungt/PX4-Autopilot/Tools/simulation/gz/worlds"
    )

    procs = []
    sitl_log = open(SITL_LOG, 'w')
    bridge_log = open(BRIDGE_LOG, 'w')
    yolo_log = open(YOLO_LOG, 'w')
    try:
        # Phase 1: start gz sim directly on the repo world. PX4's rcS only
        # resolves PX4_GZ_WORLD against ITS OWN worlds dir, so a custom repo
        # world must already be running — PX4 then detects it via
        # `gz topic -l` and only starts the bridge.
        print('[1/6] Launching gz sim with person_tracking_approach world...')
        procs.append(subprocess.Popen(
            ['gz', 'sim', '-s', '-r', '--headless-rendering',
             f'{PROJECT}/gazebo/worlds/person_tracking_approach.sdf'],
            env=env, stdout=sitl_log, stderr=subprocess.STDOUT))

        world_up = False
        for _ in range(30):
            res = subprocess.run(['gz', 'topic', '-l'], capture_output=True, text=True, env=env)
            if '/world/person_tracking_approach/clock' in res.stdout:
                world_up = True
                print('  gz world clock topic is up.')
                break
            time.sleep(1.0)
        if not world_up:
            print('FAILED: gz world never came up.')
            return 1

        print('[2/6] Launching PX4 SITL (bridges into the running world)...')
        procs.append(subprocess.Popen(
            ['make', 'px4_sitl', 'gz_x500'],
            cwd='/home/tungt/PX4-Autopilot', env=env,
            stdout=sitl_log, stderr=subprocess.STDOUT))

        print('[3/6] Launching ros_gz_bridge for /camera/image_raw...')
        procs.append(subprocess.Popen(
            ['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
             '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image'],
            stdout=bridge_log, stderr=subprocess.STDOUT))

        print('[4/6] Launching yolo_detector_node (model loads during climb)...')
        yolo_env = env.copy()
        yolo_env['PYTHONPATH'] = f"{PROJECT}/ros2_ws/src/vision_tracking:{os.environ.get('PYTHONPATH', '')}"
        procs.append(subprocess.Popen(
            ['python3', f'{PROJECT}/ros2_ws/src/vision_tracking/vision_tracking/yolo_detector_node.py',
             '--ros-args', '-p', 'image_topic:=/camera/image_raw',
             '-p', f'model_path:={PROJECT}/yolov8n.pt',
             '-p', 'conf:=0.25', '-p', 'device:=cpu',
             '-p', 'show_debug_image:=false'],
            cwd=PROJECT, env=yolo_env,
            stdout=yolo_log, stderr=subprocess.STDOUT))

        # ROS observer in a background thread.
        rclpy.init()
        observer = Observer()
        spin_thread = threading.Thread(target=rclpy.spin, args=(observer,), daemon=True)
        spin_thread.start()

        # Flight control on the main thread.
        print('[5/6] Connecting MAVLink (udpin:0.0.0.0:14540)...')
        master = None
        for _ in range(30):
            try:
                cand = mavutil.mavlink_connection('udpin:0.0.0.0:14540', source_system=255)
                if cand.wait_heartbeat(timeout=1.0):
                    master = cand
                    print('Connected to PX4!')
                    break
            except Exception:  # noqa: BLE001
                pass
            time.sleep(0.5)
        if master is None:
            print('FAILED: no MAVLink heartbeat.')
            return 1

        print('[6/6] Waiting for EKF2 global position (up to 60s)...')
        master.mav.request_data_stream_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)
        deadline = time.time() + 60.0
        ekf_ok = False
        while time.time() < deadline:
            msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1.0)
            if msg and msg.lat != 0 and msg.lon != 0:
                ekf_ok = True
                break
        if not ekf_ok:
            print('FAILED: EKF2 never produced a global position.')
            return 1

        # MAVSDK-style NaN params: PX4 1.14 navigator ACKs NAV_TAKEOFF with a
        # finite param7 but the takeoff task never generates setpoints (thrust
        # stays 0) and COM_DISARM_PRFLT auto-disarms 10 s later. NaN params +
        # MIS_TAKEOFF_ALT is the combination that actually flies.
        nan = float('nan')
        if not wait_for_command_ack(master, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                    (1.0, nan, nan, nan, nan, nan, nan)):
            print('FAILED: arm never accepted.')
            return 1
        master.mav.param_set_send(
            master.target_system, master.target_component,
            b'MIS_TAKEOFF_ALT', TAKEOFF_TARGET_M, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        master.recv_match(type='PARAM_VALUE', blocking=True, timeout=2.0)
        if not wait_for_command_ack(master, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                                    (nan, nan, nan, nan, nan, nan, nan)):
            print('FAILED: takeoff never accepted.')
            return 1

        print(f'[7/7] Climbing to {TRACKING_GATE_ALT_M} m before starting the detection window...')
        deadline = time.time() + 60.0
        at_altitude = False
        while time.time() < deadline:
            msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=0.5)
            alt = (msg.relative_alt / 1000.0) if msg else 0.0
            if alt >= TRACKING_GATE_ALT_M:
                at_altitude = True
                print(f'  At {alt:.2f} m — starting detection window.')
                break
        if not at_altitude:
            print('FAILED: never reached the tracking gate altitude.')
            return 1

        # --- Detection assertions ---------------------------------------
        # The node may already be tracking during the climb; measure the
        # first error AFTER the gate so the latency is meaningful.
        n_at_gate = observer.n_errors
        gate_t = time.time()
        t0 = time.time()
        first_after_gate = None
        while time.time() - t0 < FIRST_ERROR_TIMEOUT_S:
            if observer.n_errors > n_at_gate:
                first_after_gate = time.time() - gate_t
                break
            time.sleep(0.5)
        if observer.first_error_t is None:
            print(f'FAILED: no /tracking/error within {FIRST_ERROR_TIMEOUT_S:.0f}s at altitude.')
            observer.save_frame('FAIL_no_detection')
            return 1
        observer.save_frame('first_detection')
        if first_after_gate is not None:
            print(f'  First /tracking/error {first_after_gate:.1f}s after the gate '
                  f'(pixel offset {observer.last_error}).')
        else:
            print('  Tracking was already active during the climb '
                  f'(pixel offset {observer.last_error}).')

        n0 = observer.n_errors
        time.sleep(COLLECTION_WINDOW_S)
        window_msgs = observer.n_errors - n0
        observer.save_frame('end_of_window')
        print(f'  Detection window: {window_msgs} messages in {COLLECTION_WINDOW_S:.0f}s.')

        if window_msgs < MIN_ERROR_MESSAGES:
            print(f'FAILED: only {window_msgs} error messages '
                  f'(< {MIN_ERROR_MESSAGES}) in the collection window.')
            return 1

        print('=' * 70)
        print('   APPROACH CAMERA TEST PASSED: person detected & tracked   ')
        print('=' * 70)
        return 0
    finally:
        for p in procs:
            p.kill()
        sitl_log.close()
        bridge_log.close()
        yolo_log.close()
        subprocess.run("pkill -9 -x px4 2>/dev/null || true", shell=True)
        subprocess.run("pkill -9 -f 'gz si[m]' 2>/dev/null || true", shell=True)
        subprocess.run("pkill -9 -f yolo_detector_no[d] 2>/dev/null || true", shell=True)
        subprocess.run("pkill -9 -f 'parameter_bridge /camera' 2>/dev/null || true", shell=True)
        try:
            rclpy.shutdown()
        except Exception:  # noqa: BLE001
            pass


if __name__ == '__main__':
    sys.exit(main())
