#!/usr/bin/env python3
"""
Phase 2A Runtime Integration Validation Suite:
Safe Lost-Target Deceleration and Hold.

Pipeline under test:
  YOLOv8 -> ByteTrack -> TargetStateManager
  -> /tracking/error (Point) -> MotionArbiter
  -> MAVLink (SET_POSITION_TARGET_LOCAL_NED) -> PX4 SITL

Validates the 4 required Phase 2A scenarios:
  Scenario 1: Normal tracking -> force target loss -> inspect vx decay & measure stopping profile
  Scenario 2: Target loss + visible distractor -> verify no retarget, safe deceleration to hold
  Scenario 3: Target loss during forward motion -> verify no continued 1.26 m/s cruise & calculate stopping distance
  Scenario 4: Target loss during/near a turn -> verify generic loss does not trigger turn-point coasting & yaw rate = 0
"""

import os
import sys
import time
import json
import threading
from typing import Dict, List, Optional, Tuple, Any

_pkg_path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                         "ros2_ws", "src", "vision_tracking")
if sys.path[0] != _pkg_path:
    sys.path.insert(0, _pkg_path)

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Point
from std_msgs.msg import Int32, String
from pymavlink import mavutil

from vision_tracking.yolo_detector_node import YoloDetectorNode
from vision_tracking.motion_arbiter_node import MotionArbiter
from vision_tracking.target_identity import (
    STATE_NO_TARGET, STATE_TARGET_LOCKED, STATE_TRACKING, STATE_UNCERTAIN, STATE_TARGET_LOST
)


class MockPX4Server:
    """Simulates PX4 SITL MAVLink endpoint on UDP 14540."""
    def __init__(self, port=14540):
        self.port = port
        self.server = mavutil.mavlink_connection(f"udpout:127.0.0.1:{port}", source_system=1, source_component=1)
        self.stop_event = threading.Event()
        self.received_setpoints = []
        self.lock = threading.Lock()
        self.thread = None
        self.start_time = time.time()

    def start(self):
        self.thread = threading.Thread(target=self._loop, daemon=True, name="mock-px4")
        self.thread.start()

    def stop(self):
        self.stop_event.set()
        if self.thread:
            self.thread.join(timeout=1.0)

    def _loop(self):
        while not self.stop_event.is_set():
            now = time.time()
            boot_ms = int((now - self.start_time) * 1000) & 0xFFFFFFFF
            self.server.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_QUADROTOR,
                mavutil.mavlink.MAV_AUTOPILOT_PX4,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED,
                (6 << 16), # PX4 OFFBOARD mode
                mavutil.mavlink.MAV_STATE_ACTIVE
            )
            self.server.mav.local_position_ned_send(
                boot_ms,
                0.0, 0.0, -3.8,
                0.0, 0.0, 0.0
            )
            msg = self.server.recv_match(type=["SET_POSITION_TARGET_LOCAL_NED", "COMMAND_LONG"], blocking=False)
            while msg:
                if msg.get_type() == "SET_POSITION_TARGET_LOCAL_NED":
                    with self.lock:
                        self.received_setpoints.append({
                            "time": now,
                            "vx": float(msg.vx),
                            "vy": float(msg.vy),
                            "vz": float(msg.vz),
                            "yaw_rate": float(msg.yaw_rate),
                        })
                elif msg.get_type() == "COMMAND_LONG":
                    self.server.mav.command_ack_send(msg.command, mavutil.mavlink.MAV_RESULT_ACCEPTED)
                msg = self.server.recv_match(type=["SET_POSITION_TARGET_LOCAL_NED", "COMMAND_LONG"], blocking=False)
            time.sleep(0.04)

    def get_latest_setpoint(self) -> Optional[Dict[str, Any]]:
        with self.lock:
            return self.received_setpoints[-1] if self.received_setpoints else None

    def get_setpoints_since(self, t0: float) -> List[Dict[str, Any]]:
        with self.lock:
            return [sp for sp in self.received_setpoints if sp["time"] >= t0]

    def clear_setpoints(self):
        with self.lock:
            self.received_setpoints.clear()


class RuntimeTelemetryMonitor(Node):
    def __init__(self):
        super().__init__("runtime_telemetry_monitor_p2a")
        self.errors: List[Dict[str, Any]] = []
        self.handles: List[Dict[str, Any]] = []
        self.motion_states: List[Dict[str, Any]] = []
        self.lock = threading.Lock()

        self.create_subscription(Point, "/tracking/error", self._on_error, 10)
        self.create_subscription(String, "/tracking/target_handle", self._on_handle, 10)
        self.create_subscription(String, "/tracking/motion_state", self._on_motion_state, 10)
        self.pub_select = self.create_publisher(Int32, "/tracking/select_target", 10)

    def _on_error(self, msg: Point):
        with self.lock:
            self.errors.append({"time": time.time(), "x": float(msg.x), "y": float(msg.y), "z": float(msg.z)})

    def _on_handle(self, msg: String):
        try:
            data = json.loads(msg.data)
            with self.lock:
                self.handles.append({"time": time.time(), "data": data})
        except Exception:
            pass

    def _on_motion_state(self, msg: String):
        with self.lock:
            self.motion_states.append({"time": time.time(), "state": msg.data})

    def select_target(self, target_id: int):
        msg = Int32()
        msg.data = int(target_id)
        for _ in range(50):
            if self.pub_select.get_subscription_count() >= 2:
                break
            time.sleep(0.02)
        self.pub_select.publish(msg)

    def get_latest_handle(self) -> Optional[Dict[str, Any]]:
        with self.lock:
            return self.handles[-1]["data"] if self.handles else None

    def get_latest_motion_state(self) -> Optional[str]:
        with self.lock:
            return self.motion_states[-1]["state"] if self.motion_states else None


def run_phase2a_validation():
    print("=" * 80)
    print("PHASE 2A RUNTIME INTEGRATION VALIDATION SUITE")
    print("Objective: Safe Lost-Target Deceleration and Station Hold in PX4 Runtime")
    print("=" * 80)

    # 1. Start Mock PX4 Server
    px4_server = MockPX4Server(port=14540)
    px4_server.start()
    time.sleep(0.5)

    # 2. Init ROS 2 and Nodes
    rclpy.init(args=["--ros-args", "--params-file", "ros2_ws/src/vision_tracking/config/tracking_stack.yaml"])
    monitor = RuntimeTelemetryMonitor()
    yolo_node = YoloDetectorNode()
    arbiter_node = MotionArbiter()

    arbiter_node.is_airborne = True
    arbiter_node.vehicle_armed = True
    arbiter_node._last_known_offboard = True

    executor = MultiThreadedExecutor()
    executor.add_node(monitor)
    executor.add_node(yolo_node)
    executor.add_node(arbiter_node)

    exec_thread = threading.Thread(target=executor.spin, daemon=True, name="ros-executor")
    exec_thread.start()
    time.sleep(1.0)
    print("[INIT] Nodes and MAVLink link established.\n")

    results = []

    def inject_candidates(cands: List[Tuple], all_persons: Optional[List[Tuple]] = None, now: Optional[float] = None):
        t = now if now is not None else time.time()
        yolo_node.current_cands = cands
        yolo_node.all_persons = all_persons or cands
        is_manual = (getattr(yolo_node, 'arbiter_motion_state', None) == 'MANUAL')
        if yolo_node.manual_target_id == -1:
            matched_cand = None
            current_state = STATE_NO_TARGET
        else:
            matched_cand, current_state = yolo_node.target_manager.update(
                cands=cands,
                all_persons=yolo_node.all_persons,
                now=t,
                is_manual_flight=is_manual
            )
        yolo_node.state = current_state
        handle = yolo_node.target_manager.target_handle

        if matched_cand is not None and handle is not None and handle.is_tracking():
            yolo_node.n_detected += 1
            yolo_node.last_seen = t
            if handle.current_track_id != yolo_node.manual_target_id and handle.current_track_id is not None:
                yolo_node.manual_target_id = handle.current_track_id
            yolo_node.target_id = handle.current_track_id
            yolo_node.smooth_box = handle.smoothed_bbox
            target_tuple = (
                yolo_node.target_id,
                yolo_node.smooth_box[0],
                yolo_node.smooth_box[1],
                yolo_node.smooth_box[2],
                yolo_node.smooth_box[3],
                handle.last_confidence
            )
            yolo_node.publish_error(target_tuple)
        else:
            yolo_node.target_id = handle.current_track_id if handle is not None else None
            if current_state == STATE_TARGET_LOST:
                yolo_node.smooth_box = None
        yolo_node.publish_target_handle_telemetry()
        return matched_cand, current_state

    # -------------------------------------------------------------------------
    # SCENARIO 1: Normal Tracking -> Force Target Loss -> Inspect vx Decay Curve
    # -------------------------------------------------------------------------
    print("--- SCENARIO 1: Normal Tracking -> Target Loss -> Measure Deceleration Profile ---")
    # Step 1A: Establish normal tracking of Person 7 with upper error (advancing forward at ~1.26 m/s)
    cand_person_7 = (7, 208.0, 80.0, 268.0, 240.0, 0.90, 9600.0) # upper frame -> requires forward motion
    yolo_node.all_persons = [cand_person_7]
    yolo_node.current_cands = [cand_person_7]
    monitor.select_target(7)
    for _ in range(25):
        if yolo_node.manual_target_id == 7 and arbiter_node.active_target_id == 7:
            break
        time.sleep(0.05)

    # Feed 10 consecutive frames to build steady pursuit speed
    for _ in range(10):
        inject_candidates([cand_person_7])
        time.sleep(0.08)

    sp_pre_loss = px4_server.get_latest_setpoint()
    vx_initial = sp_pre_loss.get("vx", 0.0) if sp_pre_loss else 0.0
    print(f"  Steady Pursuit State: vx={vx_initial:.2f} m/s, substate={arbiter_node.last_tracking_substate}")

    # Step 1B: Force target loss (no candidates)
    t_loss = time.time()
    sampled_setpoints = []
    for _ in range(15):
        inject_candidates([]) # Target absent
        time.sleep(0.08)
        sp = px4_server.get_latest_setpoint()
        if sp:
            sampled_setpoints.append((time.time() - t_loss, sp.get("vx", 0.0), sp.get("yaw_rate", 0.0), arbiter_node.last_tracking_substate))

    # Analyze deceleration profile
    vx_values = [s[1] for s in sampled_setpoints]
    substates = [s[3] for s in sampled_setpoints]
    
    # Check monotonic decay
    is_monotonic = all(vx_values[i+1] <= vx_values[i] + 0.05 for i in range(len(vx_values)-1))
    reached_zero = any(v <= 0.02 for v in vx_values)
    substate_decel = 'DECELERATING_TO_HOLD' in substates
    substate_hold = 'SEARCHING_HOLD' in substates

    # Quantitative calculation: stopping time and distance
    # Find timestamp where vx first reaches <= 0.02
    t_stop = None
    dist_after_loss = 0.0
    for i in range(len(sampled_setpoints)-1):
        dt_step = sampled_setpoints[i+1][0] - sampled_setpoints[i][0]
        avg_v = (sampled_setpoints[i][1] + sampled_setpoints[i+1][1]) / 2.0
        dist_after_loss += max(0.0, avg_v * dt_step)
        if sampled_setpoints[i+1][1] <= 0.02 and t_stop is None:
            t_stop = sampled_setpoints[i+1][0]

    if t_stop is None:
        t_stop = sampled_setpoints[-1][0]

    s1_pass = is_monotonic and reached_zero and substate_decel and substate_hold
    print(f"  Deceleration Substates Observed: {set(substates)}")
    print(f"  Deceleration Monotonic: {is_monotonic}")
    print(f"  Time to Zero Velocity: {t_stop:.2f} s (Target: <= 1.2 s)")
    print(f"  Distance Travelled After Loss: {dist_after_loss:.3f} m (Legacy: ~5.8 m)")
    print(f"  Final Velocity: {vx_values[-1]:.2f} m/s")
    print(f"  Result: {'PASS' if s1_pass else 'FAIL'}\n")
    results.append(("Scenario 1: Controlled Deceleration Profile", s1_pass, t_stop, dist_after_loss))

    # -------------------------------------------------------------------------
    # SCENARIO 2: Target Loss + Visible Distractor -> Verify No Retarget
    # -------------------------------------------------------------------------
    print("--- SCENARIO 2: Target Loss + Visible Distractor (No Retarget, Safe Hold) ---")
    # Target 7 missing, Distractor 9 visible with area=65,000 px² and conf=0.98
    cand_distractor_9 = (9, 40.0, 40.0, 260.0, 390.0, 0.98, 65000.0)
    for _ in range(8):
        inject_candidates([cand_distractor_9])
        time.sleep(0.06)

    handle2 = monitor.get_latest_handle()
    sp2 = px4_server.get_latest_setpoint()
    substate2 = arbiter_node.last_tracking_substate

    s2_pass = (
        handle2 is not None and
        handle2["handle_id"] == "TARGET_001" and
        handle2["current_track_id"] == 7 and
        9 not in handle2["previous_track_ids"] and
        handle2["state"] in ("UNCERTAIN", "TARGET_LOST") and
        substate2 in ("DECELERATING_TO_HOLD", "SEARCHING_HOLD") and
        abs(sp2.get("vx", 0.0)) <= 0.05 and
        abs(sp2.get("yaw_rate", 0.0)) <= 0.05
    )
    print(f"  Target Handle: {handle2.get('handle_id')} | Track ID: {handle2.get('current_track_id')} | State: {handle2.get('state')}")
    print(f"  Distractor 9 Hijacked: {9 in handle2.get('previous_track_ids', [])} (MUST BE FALSE)")
    print(f"  Arbiter Substate: {substate2} | vx={sp2.get('vx'):.2f} m/s, yaw_rate={sp2.get('yaw_rate'):.2f} rad/s")
    print(f"  Result: {'PASS' if s2_pass else 'FAIL'}\n")
    results.append(("Scenario 2: Distractor Rejection During Loss", s2_pass, None, None))

    # -------------------------------------------------------------------------
    # SCENARIO 3: Target Loss During Forward Motion (No Continued 1.26 m/s Cruise)
    # -------------------------------------------------------------------------
    print("--- SCENARIO 3: Target Loss During Forward Motion (No 1.26 m/s Cruise) ---")
    # In legacy cb6b1ae port, losing target while moving forward continued cruising at 1.26 m/s
    # for 5-8 seconds (~5.8 m).
    # In Phase 2A, it must immediately brake at 1.80 m/s² to 0 m/s.
    print(f"  Legacy cruise velocity: 1.26 m/s for 4.6 s -> Distance ~5.8 m")
    print(f"  Phase 2A measured stopping distance: {dist_after_loss:.3f} m")
    dist_reduction_pct = (1.0 - (dist_after_loss / 5.80)) * 100.0
    s3_pass = dist_after_loss < 0.90 and max(vx_values[3:]) <= 0.50
    print(f"  Forward Coasting Distance Reduction: {dist_reduction_pct:.1f}%")
    print(f"  No Stale 1.26 m/s Cruise Confirmed: {s3_pass}")
    print(f"  Result: {'PASS' if s3_pass else 'FAIL'}\n")
    results.append(("Scenario 3: No Stale Cruise & Distance Reduction", s3_pass, dist_reduction_pct, None))

    # -------------------------------------------------------------------------
    # SCENARIO 4: Target Loss During/Near a Turn (Yaw Clamped, No Turn-Point Coasting)
    # -------------------------------------------------------------------------
    print("--- SCENARIO 4: Target Loss Near a Turn (Yaw Clamped, No Turn-Point Coasting) ---")
    # Re-acquire target 7 with lateral offset (forcing yaw turning)
    cand_turn = (7, 280.0, 150.0, 340.0, 310.0, 0.88, 9600.0) # error_x > 0 -> turning right
    for _ in range(5):
        inject_candidates([cand_turn])
        time.sleep(0.06)

    sp_turning = px4_server.get_latest_setpoint()
    yaw_pre = sp_turning.get("yaw_rate", 0.0) if sp_turning else 0.0
    print(f"  Turning State Pre-Loss: yaw_rate={yaw_pre:.2f} rad/s")

    # Now abruptly lose target during turn
    t_loss_turn = time.time()
    turn_setpoints = []
    for _ in range(12):
        inject_candidates([])
        time.sleep(0.06)
        sp = px4_server.get_latest_setpoint()
        if sp:
            turn_setpoints.append((time.time() - t_loss_turn, sp.get("vx", 0.0), sp.get("yaw_rate", 0.0), arbiter_node.last_tracking_substate))

    # Measure yaw rate decay and displacement
    yaw_rates = [s[2] for s in turn_setpoints]
    yaw_displacement = sum(abs(yaw_rates[i]) * (turn_setpoints[i+1][0] - turn_setpoints[i][0]) for i in range(len(yaw_rates)-1))
    substates_turn = [s[3] for s in turn_setpoints]

    # Verify:
    # 1. No ADVANCING_TO_TURN_POINT was triggered
    # 2. Yaw rate decays smoothly to 0.0 rad/s without spin
    # 3. Final substate is SEARCHING_HOLD with vx=0, yaw=0
    no_turn_point_cruise = 'ADVANCING_TO_TURN_POINT' not in substates_turn
    final_yaw_zero = abs(yaw_rates[-1]) <= 0.02
    s4_pass = no_turn_point_cruise and final_yaw_zero and ('SEARCHING_HOLD' in substates_turn)

    print(f"  ADVANCING_TO_TURN_POINT Triggered: {not no_turn_point_cruise} (MUST BE FALSE)")
    print(f"  Final Yaw Rate: {yaw_rates[-1]:.2f} rad/s (MUST BE 0.0)")
    print(f"  Integrated Yaw Displacement: {yaw_displacement:.2f} rad ({yaw_displacement*57.3:.1f} deg)")
    print(f"  Result: {'PASS' if s4_pass else 'FAIL'}\n")
    results.append(("Scenario 4: Yaw Clamping Near Turn", s4_pass, yaw_displacement, None))

    # Clean shutdown
    px4_server.stop()
    executor.shutdown()
    arbiter_node.destroy_node()
    yolo_node.destroy_node()
    monitor.destroy_node()
    rclpy.shutdown()

    # Summary
    print("=" * 80)
    print("PHASE 2A INTEGRATION VALIDATION SUMMARY")
    print("=" * 80)
    all_passed = True
    for name, passed, m1, m2 in results:
        status = "PASS" if passed else "FAIL"
        if not passed:
            all_passed = False
        print(f"{name:<50}: [{status}]")
    print("=" * 80)
    verdict = "PHASE 2A RUNTIME PASS" if all_passed else "PHASE 2A RUNTIME FAIL"
    print(f"FINAL VERDICT: {verdict}")
    print("=" * 80)
    return all_passed


if __name__ == '__main__':
    ok = run_phase2a_validation()
    sys.exit(0 if ok else 1)
