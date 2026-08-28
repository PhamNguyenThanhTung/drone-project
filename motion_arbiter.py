#!/usr/bin/env python3
"""
MotionArbiter (PX4 SITL Native Backend):
Unified Flight Control & State Machine Node for UAV Tracking & Teleop.

Features:
- Automatic ARM & Takeoff to 4.0m on startup with retry until EKF2 is fully initialized.
- Single MAVLink connection point (udpin:0.0.0.0:14540).
- Tracks the real PX4 armed state; TAKEOFF after an uncommanded land/disarm
  clears stale flags and re-arms instead of being ignored as a duplicate.
- State Machine: [MANUAL, TRACKING, STANDBY].
- Interactive Key Actions:
    * Key [TAB] / [T] -> Takeoff to 4.0m
    * Key [P]         -> Land
    * Keys [W/A/S/D]  -> Instant Manual Teleop Override
    * Keys [1-9]      -> Lock Target
    * Keys [0/SPACE]  -> Standby Hover
- 10 Hz continuous velocity setpoint dispatch (set_position_target_local_ned, MAV_FRAME_BODY_NED).
"""

import time
import math
import threading
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Int32, String
from pymavlink import mavutil

# State definitions
STATE_MANUAL = 'MANUAL'
STATE_TRACKING = 'TRACKING'
STATE_STANDBY = 'STANDBY'
STATE_MANUAL_GOTO = 'MANUAL_GOTO'

# PX4 Custom Modes
PX4_CUSTOM_MAIN_MODE_MANUAL = 1
PX4_CUSTOM_MAIN_MODE_ALTCTL = 2
PX4_CUSTOM_MAIN_MODE_POSCTL = 3
PX4_CUSTOM_MAIN_MODE_AUTO = 4
PX4_CUSTOM_MAIN_MODE_ACRO = 5
PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6
PX4_CUSTOM_MAIN_MODE_STABILIZED = 7


class MotionArbiter(Node):
    """
    Unified Motion Arbiter & Flight Controller for PX4 Autopilot.
    Only node authorized to dispatch velocity & yaw setpoints to PX4.
    """

    def __init__(self):
        super().__init__('motion_arbiter')

        # ROS 2 Parameters
        self.declare_parameter('mavlink', 'udpin:0.0.0.0:14540')
        self.declare_parameter('auto_takeoff', True)
        self.declare_parameter('takeoff_alt', 4.0)
        self.declare_parameter('kp', 0.0035)
        self.declare_parameter('max_rate', 0.40)
        self.declare_parameter('search_rate', 0.20)
        self.declare_parameter('lost_timeout', 4.0)
        self.declare_parameter('enable_forward', True)
        self.declare_parameter('default_walk_speed', 0.85)
        self.declare_parameter('default_backup_speed', 0.75)
        self.declare_parameter('kp_y_boost', 0.0070)
        self.declare_parameter('kp_lateral', 0.0020)
        self.declare_parameter('deadband_x', 20.0)
        self.declare_parameter('deadband_y', 25.0)
        self.declare_parameter('max_forward_speed', 1.8)
        self.declare_parameter('min_forward_speed', -1.2)
        self.declare_parameter('tree_clearance_margin', 1.8)
        self.declare_parameter('teleop_timeout', 0.5)
        self.declare_parameter('goto_altitude', 4.0)
        self.declare_parameter('bottom_backup_timeout', 1.8)
        self.declare_parameter('bottom_recovery_timeout', 3.0)

        gp = self.get_parameter
        self.mavlink_uri = gp('mavlink').value
        self.auto_takeoff = bool(gp('auto_takeoff').value)
        self.takeoff_alt = float(gp('takeoff_alt').value)
        self.kp = float(gp('kp').value)
        self.max_rate = abs(float(gp('max_rate').value))
        self.search_rate = abs(float(gp('search_rate').value))
        self.lost_timeout = float(gp('lost_timeout').value)
        self.enable_forward = bool(gp('enable_forward').value)
        self.default_walk_speed = float(gp('default_walk_speed').value)
        self.default_backup_speed = float(gp('default_backup_speed').value)
        if self.default_backup_speed > 0.70:
            self.default_backup_speed = 0.65
        self.kp_y_boost = float(gp('kp_y_boost').value)
        self.kp_lateral = float(gp('kp_lateral').value)
        self.deadband_x = float(gp('deadband_x').value)
        self.deadband_y = float(gp('deadband_y').value)
        self.max_forward_speed = float(gp('max_forward_speed').value)
        self.min_forward_speed = float(gp('min_forward_speed').value)
        self.tree_clearance_margin = float(gp('tree_clearance_margin').value)
        self.teleop_timeout = float(gp('teleop_timeout').value)
        self.goto_altitude = float(gp('goto_altitude').value)
        self.bottom_backup_timeout = float(gp('bottom_backup_timeout').value)
        self.bottom_recovery_timeout = float(gp('bottom_recovery_timeout').value)

        # State Machine (Default: TRACKING)
        self.current_state = STATE_TRACKING
        self.active_target_id: Optional[int] = None
        self.is_airborne = False
        self.is_taking_off = False

        # Flight telemetry & altitude hold variables
        self.current_yaw: float = 0.0
        self.current_pitch: float = 0.0
        self.current_roll: float = 0.0
        self.ground_z: float = 0.0
        self.target_z_ned: float = -self.takeoff_alt
        self.current_local_z: float = 0.0
        self.current_lat: Optional[float] = None
        self.current_lon: Optional[float] = None
        self.current_alt: Optional[float] = None

        # Tracking variables
        self.error_x: Optional[float] = None
        self.error_y: Optional[float] = None
        self.area: Optional[float] = None
        self.last_seen: float = 0.0
        self.acquired_once: bool = False
        self.direction: float = 1.0
        self.target_turn_dir: float = 1.0
        self.dist_advanced: float = 0.0
        self.target_dist: float = 4.5
        self.target_dx: float = 4.5
        self.target_dy: float = 0.0
        self.last_seen_x: float = 0.0
        self.last_seen_y: float = 0.0
        self.last_tracking_substate: Optional[str] = None
        self._last_vx: float = 0.0
        self._last_vy: float = 0.0
        self._last_yaw_rate: float = 0.0
        self.goto_lat: Optional[float] = None
        self.goto_lon: Optional[float] = None
        self.goto_alt: float = self.goto_altitude
        self.goto_resume_state: str = STATE_STANDBY
        self.goto_resume_target_id: Optional[int] = None

        # Teleop variables
        self.teleop_vx: float = 0.0
        self.teleop_vy: float = 0.0
        self.teleop_vz: float = 0.0
        self.teleop_yaw_rate: float = 0.0
        self.last_teleop_cmd_time: float = 0.0

        # Real-time diagnostic logger
        self.diag_log_file = '/home/tungt/drone-project/logs/tracking_diagnostics.jsonl'
        self.diag_file_handle = None
        try:
            import os
            os.makedirs('/home/tungt/drone-project/logs', exist_ok=True)
            self.diag_file_handle = open(self.diag_log_file, 'a', buffering=1)
        except Exception as exc:
            self.get_logger().warning(f"Could not open diagnostic log file: {exc}")
        self._yaw_sign_history = []

        # Real vehicle state, kept fresh by the RX monitor thread. The
        # internal is_airborne flag is only bookkeeping; PX4 can land and
        # disarm underneath us at any time (offboard-loss failsafe, QGC
        # command, crash). vehicle_armed is the ground truth that keeps the
        # two in sync.
        self.vehicle_armed: Optional[bool] = None

        # Single-reader MAVLink RX cache. mavutil's recv_match(type=...)
        # DISCARDS non-matching messages, so concurrent callers used to steal
        # COMMAND_ACK / HEARTBEAT from each other. One monitor thread now owns
        # the socket reads and caches the latest message per type.
        self.rx_lock = threading.Lock()
        self.rx_latest = {}

        # Timing & locks
        self.last_tick_time = time.time()
        self.lock = threading.Lock()

        # ROS 2 Subscriptions & Publishers
        self.create_subscription(Point, '/tracking/error', self.on_tracking_error, 10)
        self.create_subscription(Twist, '/teleop/cmd_vel', self.on_teleop_cmd, 10)
        self.create_subscription(Int32, '/tracking/select_target', self.on_target_selected, 10)
        self.create_subscription(String, '/teleop/flight_action', self.on_flight_action, 10)
        self.create_subscription(Point, '/tracking/goto_gps', self.on_goto_gps, 10)
        self.pub_state = self.create_publisher(String, '/tracking/motion_state', 10)
        self.pub_gps = self.create_publisher(NavSatFix, '/tracking/gps', 10)

        # Establish single MAVLink connection to PX4
        self.master = None
        self.connect_mavlink()

        # Auto Takeoff on launch
        if self.auto_takeoff:
            threading.Thread(target=self._auto_takeoff_worker, daemon=True).start()

        # 10 Hz Control Dispatch Loop
        self.timer = self.create_timer(0.10, self.tick)
        self.get_logger().info(
            f'MotionArbiter ready: auto_takeoff={self.auto_takeoff}, takeoff_alt={self.takeoff_alt:.1f}m'
        )

    # ------------------------------------------------------------------
    # PX4 MAVLink Connection & Flight Management
    # ------------------------------------------------------------------
    def connect_mavlink(self):
        """Establish single authoritative MAVLink connection to PX4."""
        self.get_logger().info(f'Connecting to PX4 SITL at {self.mavlink_uri}...')
        for attempt in range(1, 40):
            try:
                candidate = mavutil.mavlink_connection(self.mavlink_uri, source_system=255)
                if candidate.wait_heartbeat(timeout=1.0):
                    self.master = candidate
                    self.get_logger().info(
                        f'PX4 Heartbeat received (Autopilot={candidate.target_system}, Comp={candidate.target_component})'
                    )
                    break
            except Exception as exc:  # noqa: BLE001
                self.get_logger().debug(f'PX4 connect attempt {attempt}: {exc}')
            time.sleep(0.5)

        if self.master is None:
            raise RuntimeError(f'Cannot connect to PX4 SITL at {self.mavlink_uri}')

        self.master.mav.request_data_stream_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1
        )

        threading.Thread(
            target=self._rx_monitor_loop, daemon=True, name='mavlink-rx-monitor'
        ).start()
        try:
            self._apply_sitl_failsafe_tolerances()
        except Exception as exc:  # noqa: BLE001 - never block flight control on a param tweak
            self.get_logger().warning(f'[PARAM] COM_OF_LOSS_T tuning skipped: {exc}')

    def _rx_monitor_loop(self):
        """Single consumer of the MAVLink RX stream.

        mavutil's recv_match(type=...) silently DISCARDS every non-matching
        message, so any second concurrent caller steals messages from the
        first (COMMAND_ACK during takeoff being the costly example). All RX
        goes through this thread; everyone else reads self.rx_latest.
        """
        while True:
            m = self.master
            if m is None:
                time.sleep(0.2)
                continue
            try:
                msg = m.recv_match(blocking=True, timeout=0.2)
            except Exception:  # noqa: BLE001
                time.sleep(0.1)
                continue
            if msg is None:
                continue
            mtype = msg.get_type()
            with self.rx_lock:
                self.rx_latest[mtype] = (time.time(), msg)
            if mtype == 'HEARTBEAT':
                # Only update vehicle armed status from the autopilot heartbeat, NOT from GCS or companion components
                if msg.get_srcSystem() == m.target_system and getattr(msg, 'type', 0) != mavutil.mavlink.MAV_TYPE_GCS:
                    self.vehicle_armed = bool(
                        int(msg.base_mode) & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                    )
            elif mtype == 'ATTITUDE':
                self.current_yaw = float(getattr(msg, 'yaw', 0.0))
                self.current_pitch = float(getattr(msg, 'pitch', 0.0))
                self.current_roll = float(getattr(msg, 'roll', 0.0))
            elif mtype == 'LOCAL_POSITION_NED':
                self.current_local_z = float(getattr(msg, 'z', 0.0))
            elif mtype == 'GLOBAL_POSITION_INT':
                lat = float(getattr(msg, 'lat', 0))
                lon = float(getattr(msg, 'lon', 0))
                if lat != 0.0 or lon != 0.0:
                    self.current_lat = lat / 1e7
                    self.current_lon = lon / 1e7
                    self.current_alt = float(getattr(msg, 'relative_alt', 0)) / 1000.0

    def _rx_get(self, mtype, max_age=None):
        """Return the latest cached message of mtype (None if absent/stale)."""
        with self.rx_lock:
            entry = self.rx_latest.get(mtype)
        if entry is None:
            return None
        ts, msg = entry
        if max_age is not None and (time.time() - ts) > max_age:
            return None
        return msg

    def _apply_sitl_failsafe_tolerances(self):
        """Relax the OFFBOARD-loss timeout for SITL running on a loaded host.

        Field logs (ulog 2026-08-25 08_06_18) show the 10 Hz setpoint stream
        stalling ~2.4 s roughly every 30 s while Gazebo/YOLO/QGC compete for
        CPU; against the 1 s default each stall tripped an offboard-loss
        failsafe whose fallback descent cost ~1 m of altitude until the drone
        eventually touched down. 5 s keeps genuine stream deaths failing safe
        while absorbing load hiccups.

        !!! SITL-ONLY TUNING — DO NOT FLY OUTDOORS WITH THIS VALUE !!!
        A 5 s failsafe delay is a desktop-load compromise. Before any real
        flight, revisit this number with the safety pilot: real vehicles need
        a much shorter offboard-loss reaction (PX4 default 1.0 s or shorter).
        """
        params_to_set = [
            ('COM_OF_LOSS_T', 5.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
            ('NAV_DLL_ACT', 0.0, mavutil.mavlink.MAV_PARAM_TYPE_INT32),
            ('NAV_RCL_ACT', 0.0, mavutil.mavlink.MAV_PARAM_TYPE_INT32),
        ]
        m = self.master
        if m is None:
            return
        for param_name, desired_val, ptype in params_to_set:
            try:
                m.mav.param_set_send(
                    m.target_system, m.target_component,
                    param_name.encode('ascii'), desired_val,
                    ptype
                )
                time.sleep(0.05)
            except Exception:
                pass
        self.get_logger().info('[PARAM] SITL failsafe tolerances applied.')

    def on_flight_action(self, msg: String):
        """Handle interactive flight actions from HUD ([TAB]=TAKEOFF, [P]=LAND)."""
        action = msg.data.strip().upper()
        if action == "TAKEOFF":
            if self.is_taking_off:
                self.get_logger().info("[ACTION] Takeoff already in progress, ignoring duplicate TAKEOFF.")
                return
            if self.is_airborne and self.vehicle_armed is not False:
                self.get_logger().info("[ACTION] Drone is already airborne, ignoring duplicate TAKEOFF.")
                return
            if self.is_airborne and getattr(self, 'vehicle_armed', None) is False:
                # PX4 landed and disarmed on its own (offboard-loss failsafe,
                # QGC command...). is_airborne was only bookkeeping; the old
                # code swallowed every TAKEOFF forever in this situation.
                self.get_logger().warning(
                    "[ACTION] PX4 reports DISARMED while arbiter believed airborne "
                    "-> clearing stale state and re-arming for takeoff."
                )
                self.is_airborne = False
            self.is_taking_off = True
            self.get_logger().info(f"[ACTION] Key TAB received -> Executing ARM & TAKEOFF to {self.takeoff_alt:.1f}m!")
            threading.Thread(target=self._execute_takeoff, daemon=True).start()
        elif action == "LAND":
            self.get_logger().info("[ACTION] Key P received -> Executing LAND command!")
            threading.Thread(target=self.land_px4, daemon=True).start()

    def land_px4(self):
        """Send LAND command to PX4."""
        m = self.master
        if m is not None:
            self.set_state(STATE_STANDBY, trigger='user_key_land', target_id=None)
            m.mav.command_long_send(
                m.target_system, m.target_component,
                mavutil.mavlink.MAV_CMD_NAV_LAND,
                0, 0, 0, 0, 0, 0, 0, 0
            )
            self.is_airborne = False
            self.is_taking_off = False
            self.get_logger().info("PX4 Landing command dispatched.")

    def _auto_takeoff_worker(self):
        """Worker that waits for EKF2 stability and initiates takeoff automatically."""
        self.get_logger().info(f"[AUTO-TAKEOFF] Waiting for GPS/EKF position data...")
        m = self.master
        gps_ready = False
        if m is not None:
            t0 = time.time()
            while time.time() - t0 < 30.0:
                msg = self._rx_get('GPS_RAW_INT') or self._rx_get('GLOBAL_POSITION_INT')
                if msg is not None and getattr(msg, 'lat', 0) != 0:
                    gps_ready = True
                    self.get_logger().info(
                        f"[AUTO-TAKEOFF] GPS position ready: "
                        f"Lat={msg.lat/1e7:.6f}°, Lon={msg.lon/1e7:.6f}°"
                    )
                    break
                time.sleep(0.5)
        if not gps_ready:
            self.get_logger().error('[AUTO-TAKEOFF] No valid GPS position after 30s; refusing to arm.')
            self.is_taking_off = False
            return
        time.sleep(2.0)
        self._execute_takeoff()

    def _command_ack(self, command: int, *params: float, timeout: float = 3.0) -> bool:
        """Send a command and require PX4 to accept it."""
        m = self.master
        if m is None:
            return False
        values = list(params) + [0.0] * (7 - len(params))
        # Only an ACK that arrives after this send counts; the RX monitor may
        # still hold the previous command's ACK in its cache.
        with self.rx_lock:
            baseline = self.rx_latest.get('COMMAND_ACK', (0.0, None))[0]
        m.mav.command_long_send(
            m.target_system, m.target_component, command, 0, *values[:7]
        )
        deadline = time.time() + timeout
        while time.time() < deadline:
            with self.rx_lock:
                entry = self.rx_latest.get('COMMAND_ACK')
            if entry is not None and entry[0] > baseline:
                ack = entry[1]
                if int(ack.command) != command:
                    time.sleep(0.02)
                    continue
                result = int(ack.result)
                if result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                    return True
                self.get_logger().warning(
                    f'[MAVLINK] Command {command} rejected (result={result})'
                )
                return False
            time.sleep(0.02)
        self.get_logger().warning(f'[MAVLINK] No ACK for command {command}')
        return False

    def _send_offboard_velocity(self, vx: float, vy: float, vz: float, yaw_rate: float = 0.0):
        """Send setpoint to PX4 OFFBOARD mode using LOCAL_NED with active Z altitude hold."""
        m = self.master
        if m is None:
            return

        if self.is_taking_off:
            # During initial climb phase, send raw BODY_NED climb velocity
            m.mav.set_position_target_local_ned_send(
                0, m.target_system, m.target_component,
                mavutil.mavlink.MAV_FRAME_BODY_NED,
                0x05C7,
                0, 0, 0,
                float(vx), float(vy), float(vz),
                0, 0, 0,
                0.0, float(yaw_rate)
            )
            return

        # In airborne tracking and teleop, convert body velocity (vx, vy) to Local NED
        yaw = getattr(self, 'current_yaw', 0.0)
        # Advance target yaw smoothly according to yaw_rate
        target_yaw = yaw + yaw_rate * 0.10
        target_yaw = math.atan2(math.sin(target_yaw), math.cos(target_yaw))

        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)
        vx_ned = vx * cos_y - vy * sin_y
        vy_ned = vx * sin_y + vy * cos_y

        target_z = getattr(self, 'target_z_ned', self.ground_z - self.takeoff_alt)

        # 0x01E3: Position Z active (PX4 EKF2 P-position loop maintains altitude),
        # Velocity X/Y active in NED frame, Yaw angle and Yaw Rate both active for responsive turns.
        m.mav.set_position_target_local_ned_send(
            0, m.target_system, m.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0x01E3,
            0.0, 0.0, float(target_z),
            float(vx_ned), float(vy_ned), 0.0,
            0.0, 0.0, 0.0,
            float(target_yaw), float(yaw_rate)
        )

    def _stream_offboard_velocity(
        self, vx: float, vy: float, vz: float, duration: float,
        yaw_rate: float = 0.0
    ):
        """Keep a setpoint alive at 20 Hz for the requested duration."""
        deadline = time.time() + duration
        while time.time() < deadline:
            self._send_offboard_velocity(vx, vy, vz, yaw_rate)
            time.sleep(0.05)

    def _wait_armed(self, timeout: float = 4.0, keep_offboard_alive: bool = False) -> bool:
        """Confirm PX4 reports the vehicle as armed via heartbeat."""
        m = self.master
        deadline = time.time() + timeout
        armed_flag = mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        while m is not None and time.time() < deadline:
            if keep_offboard_alive:
                self._send_offboard_velocity(0.0, 0.0, 0.0)
            hb = self._rx_get('HEARTBEAT', max_age=1.5)
            if hb is not None and (int(hb.base_mode) & armed_flag):
                return True
            time.sleep(0.05)
        return False

    def _wait_offboard(self, timeout: float = 4.0) -> bool:
        """Confirm PX4 heartbeat reports OFFBOARD while keeping setpoints alive."""
        m = self.master
        deadline = time.time() + timeout
        while m is not None and time.time() < deadline:
            self._send_offboard_velocity(0.0, 0.0, 0.0)
            hb = self._rx_get('HEARTBEAT', max_age=1.5)
            if hb is None:
                time.sleep(0.05)
                continue
            custom_mode_enabled = (
                int(hb.base_mode) & mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
            )
            main_mode = (int(hb.custom_mode) >> 16) & 0xFF
            if custom_mode_enabled and main_mode == PX4_CUSTOM_MAIN_MODE_OFFBOARD:
                return True
            time.sleep(0.05)
        return False

    def _get_local_position(self, timeout: float = 5.0):
        """Return the latest finite LOCAL_POSITION_NED sample."""
        m = self.master
        deadline = time.time() + timeout
        while m is not None and time.time() < deadline:
            msg = self._rx_get('LOCAL_POSITION_NED', max_age=0.5)
            if msg is not None and all(
                math.isfinite(float(getattr(msg, axis))) for axis in ('x', 'y', 'z')
            ):
                return msg
            time.sleep(0.05)
        return None

    def _execute_takeoff(self):
        """Enter OFFBOARD, arm, and climb while verifying mode and altitude."""
        m = self.master
        if m is None:
            self.is_taking_off = False
            return

        self.is_taking_off = True
        self.get_logger().info(
            f"[TAKEOFF] Preparing OFFBOARD climb to {self.takeoff_alt:.1f} m..."
        )

        # Use the measured local-Z value as the ground reference. This avoids
        # assuming that the estimator origin is exactly zero at startup.
        local_position = self._get_local_position()
        if local_position is None:
            self.get_logger().error('[TAKEOFF] No valid local position; refusing to arm.')
            self.is_taking_off = False
            return
        ground_z = float(local_position.z)
        self.ground_z = ground_z
        self.target_z_ned = ground_z - self.takeoff_alt

        # PX4 requires a continuous setpoint stream before it accepts OFFBOARD.
        self._stream_offboard_velocity(0.0, 0.0, 0.0, duration=1.5)
        if not self._command_ack(
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            PX4_CUSTOM_MAIN_MODE_OFFBOARD,
            0.0,
            timeout=2.0
        ) or not self._wait_offboard():
            self.get_logger().error('[TAKEOFF] PX4 did not enter OFFBOARD; refusing to arm.')
            self.is_taking_off = False
            return

        # Arm and confirm both the command ACK and the actual heartbeat state.
        armed = False
        for _ in range(5):
            if self._command_ack(
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1.0
            ) and self._wait_armed(keep_offboard_alive=True):
                armed = True
                break
            self._stream_offboard_velocity(0.0, 0.0, 0.0, duration=0.5)
            time.sleep(0.5)
        if not armed:
            self.get_logger().error('[TAKEOFF] PX4 did not arm; refusing takeoff.')
            self.is_taking_off = False
            return

        # Negative NED Z velocity commands upward motion. Gazebo can run below
        # real time under software rendering, so use a progress watchdog rather
        # than a short wall-clock deadline that can land a healthy climbing UAV.
        reached_altitude = False
        last_reported_meter = -1
        best_altitude = 0.0
        progress_deadline = time.monotonic() + 30.0
        absolute_deadline = time.monotonic() + max(90.0, self.takeoff_alt * 25.0)
        while time.monotonic() < absolute_deadline and time.monotonic() < progress_deadline:
            msg = self._rx_get('LOCAL_POSITION_NED', max_age=0.4)
            time.sleep(0.05)
            altitude = None
            if msg is not None and math.isfinite(float(msg.z)):
                altitude = ground_z - float(msg.z)
                if altitude > best_altitude + 0.10:
                    best_altitude = altitude
                    progress_deadline = time.monotonic() + 30.0
                report_meter = int(max(0.0, altitude))
                if report_meter > last_reported_meter:
                    last_reported_meter = report_meter
                    self.get_logger().info(
                        f'[TAKEOFF] OFFBOARD climb altitude: {altitude:.2f} m'
                    )
                if altitude >= self.takeoff_alt - 0.20:
                    reached_altitude = True
                    break

            remaining = self.takeoff_alt - altitude if altitude is not None else self.takeoff_alt
            climb_rate = -0.30 if remaining < 0.8 else -0.65
            self._send_offboard_velocity(0.0, 0.0, climb_rate)

        self._stream_offboard_velocity(0.0, 0.0, 0.0, duration=2.0)
        if not reached_altitude:
            self.get_logger().error(
                f'[TAKEOFF] OFFBOARD climb did not reach {self.takeoff_alt - 0.2:.1f}m; landing.'
            )
            self._command_ack(mavutil.mavlink.MAV_CMD_NAV_LAND, timeout=2.0)
            self.is_taking_off = False
            return

        self.is_airborne = True
        self.is_taking_off = False
        self.set_state(STATE_TRACKING, trigger='takeoff_completed', target_id=None)
        self.get_logger().info(
            f"[TAKEOFF] Drone Airborne at {self.takeoff_alt:.1f}m. OFFBOARD tracking active."
        )

    # ------------------------------------------------------------------
    # State Machine & Transitions
    # ------------------------------------------------------------------
    def set_state(self, new_state: str, trigger: str, target_id: Optional[int] = None):
        """Execute state transition with structured logging and event notification."""
        if new_state == self.current_state and target_id == self.active_target_id:
            return

        old_state = self.current_state
        self.current_state = new_state
        self.active_target_id = target_id

        log_str = (
            f"\nSTATE CHANGE\n"
            f"  from: {old_state}\n"
            f"  to: {new_state}\n"
            f"  trigger: {trigger}\n"
            f"  target_id: {target_id if target_id is not None else 'None'}"
        )
        self.get_logger().info(log_str)

        msg = String()
        # Third field: real vehicle armed state, so the HUD can tell
        # "hovering" apart from "sitting on the ground after a failsafe".
        # Lightweight state-machine test doubles from the PX4 suite predate
        # the vehicle telemetry cache; treat an absent cache as armed.
        armed_status = 'DISARMED' if getattr(self, 'vehicle_armed', None) is False else 'ARMED'
        msg.data = f"{new_state}:{target_id if target_id is not None else -1}:{armed_status}"
        if self.pub_state is not None:
            self.pub_state.publish(msg)

    # ------------------------------------------------------------------
    # ROS 2 Callbacks
    # ------------------------------------------------------------------
    def on_teleop_cmd(self, msg: Twist):
        """Handle incoming manual flight teleop commands."""
        now = time.time()
        with self.lock:
            self.teleop_vx = float(msg.linear.x)
            self.teleop_vy = float(msg.linear.y)
            self.teleop_vz = float(msg.linear.z)
            self.teleop_yaw_rate = float(msg.angular.z)
            self.last_teleop_cmd_time = now

            is_active_move = (
                abs(self.teleop_vx) > 0.05 or
                abs(self.teleop_vy) > 0.05 or
                abs(self.teleop_vz) > 0.05 or
                abs(self.teleop_yaw_rate) > 0.05
            )

        if is_active_move and self.current_state != STATE_MANUAL:
            self.set_state(STATE_MANUAL, trigger='manual_override', target_id=None)
            # Do not publish a synthetic -1 on /tracking/select_target here.
            # This node subscribes to that topic too, so doing so immediately
            # fed the message back into on_target_selected() and changed the
            # freshly selected MANUAL state to STANDBY.

    def on_target_selected(self, msg: Int32):
        """Handle target selection (Click on box or Keys [1-9, 0, SPACE])."""
        target_id = int(msg.data)
        if target_id >= 0:
            self.set_state(STATE_TRACKING, trigger='click_or_key_lock', target_id=target_id)
        else:
            self.set_state(STATE_STANDBY, trigger='key_standby', target_id=None)

    def on_goto_gps(self, msg: Point):
        """Accept a WGS84 position setpoint from the HUD minimap."""
        if not self.is_airborne or self.is_taking_off:
            self.get_logger().warning(
                '[GOTO] Ignoring minimap position until takeoff is complete')
            return
        lat, lon = float(msg.x), float(msg.y)
        if not (math.isfinite(lat) and math.isfinite(lon)):
            self.get_logger().warning('[GOTO] Ignoring non-finite GPS setpoint')
            return
        if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
            self.get_logger().warning('[GOTO] Ignoring out-of-range GPS setpoint')
            return
        self.goto_lat = lat
        self.goto_lon = lon
        self.goto_alt = float(msg.z) if math.isfinite(float(msg.z)) and float(msg.z) > 0.5 else self.goto_altitude
        self.goto_resume_state = self.current_state if self.current_state != STATE_MANUAL_GOTO else STATE_STANDBY
        self.goto_resume_target_id = self.active_target_id if self.goto_resume_state == STATE_TRACKING else None
        self.set_state(STATE_MANUAL_GOTO, trigger='hud_minimap_click', target_id=None)
        self.get_logger().info(f'[GOTO] Position setpoint lat={lat:.7f}, lon={lon:.7f}, alt={self.goto_alt:.2f}m')

    def on_tracking_error(self, msg: Point):
        """Receive pixel error from YOLO / TargetManager."""
        with self.lock:
            nx, ny, nz = float(msg.x), float(msg.y), float(msg.z)
            if self.error_x is not None:
                self.error_x = 0.70 * nx + 0.30 * self.error_x
                self.error_y = 0.70 * ny + 0.30 * self.error_y
                self.area = 0.70 * nz + 0.30 * self.area
            else:
                self.error_x = nx
                self.error_y = ny
                self.area = nz
            self.last_seen = time.time()
            self.acquired_once = True
            self.last_seen_x = float(self.error_x)
            self.last_seen_y = float(self.error_y)

            if abs(self.error_x) >= self.deadband_x:
                self.direction = 1.0 if self.error_x > 0.0 else -1.0
                self.target_turn_dir = 1.0 if self.error_x > 0.0 else -1.0

            # 3D Pinhole & Tree Clearance Geometry
            current_h = self.current_alt if (self.current_alt is not None and self.current_alt > 0.5) else self.takeoff_alt
            h_rel = max(1.2, current_h - 0.90)
            alpha_y = math.atan2(self.error_y, 178.07)
            alpha_x = math.atan2(self.error_x, 133.55)
            theta_dep = max(0.15, min(1.45, 0.65 + alpha_y))

            dx = h_rel / math.tan(theta_dep)
            dy = dx * math.tan(alpha_x)
            target_dx = dx + self.tree_clearance_margin
            total_dist = math.sqrt(target_dx * target_dx + dy * dy)

            self.target_dx = max(2.0, min(16.0, target_dx))
            self.target_dy = max(-8.0, min(8.0, dy))
            self.target_dist = max(2.5, min(16.0, total_dist))
            self.dist_advanced = 0.0

    # ------------------------------------------------------------------
    # 10 Hz Control Dispatch Loop
    # ------------------------------------------------------------------
    def tick(self):
        now = time.time()
        raw_dt = now - self.last_tick_time
        dt = max(0.01, min(0.20, raw_dt))
        self.last_tick_time = now
        if raw_dt >= 0.50:
            # The field incident behind this node's COM_OF_LOSS_T tolerance:
            # make visible whatever stalls the loop so it can be attributed.
            self.get_logger().warning(
                f'[WATCHDOG] Control loop stalled {raw_dt:.2f}s (target 0.10s); '
                'OFFBOARD stream gaps trigger the PX4 offboard-loss failsafe'
            )
        self._update_gps_telemetry()

        final_vx = 0.0
        final_vy = 0.0
        final_vz = 0.0
        final_yaw_rate = 0.0
        dispatch_goto = False
        goto_reached = False

        with self.lock:
            state = self.current_state
            age = (now - self.last_seen) if self.last_seen else 999.0

            # Sync bookkeeping with the vehicle PX4 actually reports. Without
            # this, an uncommanded land+disarm left is_airborne stuck True and
            # every TAKEOFF was ignored as a "duplicate".
            if self.is_airborne and getattr(self, 'vehicle_armed', None) is False:
                self.is_airborne = False
                self.is_taking_off = False
                self.get_logger().warning(
                    '[VEHICLE] PX4 DISARMED detected -> arbiter synced to grounded.'
                )
                if self.current_state != STATE_STANDBY:
                    self.set_state(STATE_STANDBY, trigger='vehicle_disarm_detected', target_id=None)
                    state = self.current_state

            effective_lost_timeout = (
                getattr(self, 'bottom_recovery_timeout', self.lost_timeout)
                if getattr(self, 'last_seen_y', 0.0) > self.deadband_y
                else self.lost_timeout
            )
            if state == STATE_TRACKING and self.acquired_once and age > effective_lost_timeout:
                self.set_state(STATE_STANDBY, trigger='target_lost_timeout', target_id=None)
                state = self.current_state

            if state == STATE_MANUAL:
                if (now - self.last_teleop_cmd_time) <= self.teleop_timeout:
                    final_vx = self.teleop_vx
                    final_vy = self.teleop_vy
                    final_vz = self.teleop_vz
                    final_yaw_rate = self.teleop_yaw_rate
                    # Update target_z_ned during vertical teleop
                    if abs(self.teleop_vz) > 0.05:
                        self.target_z_ned += self.teleop_vz * dt
                else:
                    final_vx, final_vy, final_vz, final_yaw_rate = 0.0, 0.0, 0.0, 0.0

            elif state == STATE_TRACKING:
                final_vx, final_vy, final_vz, final_yaw_rate = self.compute_tracking_velocities(now, dt, age)

            elif state == STATE_MANUAL_GOTO:
                # Freeze the dispatch family for this timer cycle. The state
                # transition happens only after the position setpoint is sent,
                # so this cycle can never also publish BODY_NED velocity.
                dispatch_goto = True
                goto_reached = self._goto_reached()

            elif state == STATE_STANDBY:
                final_vx, final_vy, final_vz, final_yaw_rate = 0.0, 0.0, 0.0, 0.0

        # Dispatch single MAVLink velocity setpoint to PX4 OFFBOARD
        m = self.master
        if m is not None and self.is_airborne and self.vehicle_armed is not False:
            if dispatch_goto:
                self._send_goto_position_setpoint()
            else:
                self._send_offboard_velocity(final_vx, final_vy, final_vz, final_yaw_rate)

        if dispatch_goto and goto_reached:
            self.set_state(
                self.goto_resume_state,
                trigger='goto_reached',
                target_id=self.goto_resume_target_id,
            )

        # Real-time diagnostic logging
        if self.diag_file_handle is not None and not self.diag_file_handle.closed:
            try:
                import json
                current_alt = self.current_alt if self.current_alt is not None else 0.0
                target_alt = self.takeoff_alt
                alt_error = current_alt - target_alt

                events = []
                if self.is_airborne and abs(alt_error) > 0.35:
                    events.append('ALTITUDE_DROP')
                if state == STATE_TRACKING and age > 1.2:
                    events.append('TARGET_LOST')

                if abs(final_yaw_rate) > 0.08:
                    sign = 1 if final_yaw_rate > 0 else -1
                    if not self._yaw_sign_history or self._yaw_sign_history[-1][1] != sign:
                        self._yaw_sign_history.append((now, sign))
                self._yaw_sign_history = [(t, s) for (t, s) in self._yaw_sign_history if now - t <= 1.5]
                if len(self._yaw_sign_history) >= 4:
                    events.append('YAW_OSCILLATION')

                entry = {
                    'timestamp': round(now, 3),
                    'state': state,
                    'substate': getattr(self, 'last_tracking_substate', state),
                    'target_id': self.active_target_id,
                    'age': round(age, 2),
                    'error_x': round(self.error_x, 1) if self.error_x is not None else None,
                    'error_y': round(self.error_y, 1) if self.error_y is not None else None,
                    'current_alt': round(current_alt, 3),
                    'target_alt': round(target_alt, 3),
                    'alt_error': round(alt_error, 3),
                    'cmd_vx': round(final_vx, 3),
                    'cmd_vy': round(final_vy, 3),
                    'cmd_vz': round(final_vz, 3),
                    'cmd_yaw_rate': round(final_yaw_rate, 3),
                    'yaw_deg': round(getattr(self, 'current_yaw', 0.0) * 57.2958, 1),
                    'events': events
                }
                self.diag_file_handle.write(json.dumps(entry) + '\n')
            except Exception:
                pass

    def _update_gps_telemetry(self):
        """Publish the latest PX4 global position for the HUD minimap."""
        m = self.master
        # The RX monitor owns MAVLink reads; peek at its cache here instead of
        # recv_match, which would steal messages from the takeoff workers.
        #
        # Publish on the ground and during takeoff too: gating on is_airborne
        # left the HUD minimap empty until the drone lifted off, so the pilot
        # had no position readout while sitting armed on the pad (and the
        # minimap home reference was only set mid-flight).
        if m is None:
            return
        msg = self._rx_get('GLOBAL_POSITION_INT', max_age=2.0)
        if msg is not None:
            lat = float(getattr(msg, 'lat', 0)) / 1e7
            lon = float(getattr(msg, 'lon', 0)) / 1e7
            if lat != 0.0 or lon != 0.0:
                self.current_lat = lat
                self.current_lon = lon
                self.current_alt = float(getattr(msg, 'relative_alt', 0)) / 1000.0
        if self.current_lat is not None:
            fix = NavSatFix()
            fix.latitude = self.current_lat
            fix.longitude = self.current_lon
            fix.altitude = self.current_alt or 0.0
            self.pub_gps.publish(fix)

    def _send_goto_position_setpoint(self):
        """Send a GLOBAL_RELATIVE_ALT_INT position setpoint, not BODY_NED velocity."""
        m = self.master
        if m is None or not self.is_airborne or self.goto_lat is None or self.goto_lon is None:
            return
        m.mav.set_position_target_global_int_send(
            0, m.target_system, m.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            0x0DF8,
            int(self.goto_lat * 1e7), int(self.goto_lon * 1e7), float(self.goto_alt),
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.0, 0.0
        )

    def _goto_reached(self) -> bool:
        if self.goto_lat is None or self.goto_lon is None or self.current_lat is None:
            return False
        north = (self.goto_lat - self.current_lat) * 111320.0
        east = (self.goto_lon - self.current_lon) * 111320.0 * max(0.2, math.cos(math.radians(self.current_lat)))
        return math.hypot(north, east) <= 1.5

    def compute_tracking_velocities(self, now: float, dt: float, age: float) -> Tuple[float, float, float, float]:
        """Compute autonomous vision tracking velocities using smooth visual servoing & tree clearance."""
        error_x = self.error_x
        error_y = self.error_y
        acquired_once = self.acquired_once
        target_dist = self.target_dist
        target_dx = self.target_dx
        target_dy = self.target_dy

        vx = 0.0
        vy = 0.0
        vz = 0.0
        yaw_rate = 0.0
        substate = 'TRACKING'

        if error_x is not None and age <= 1.2:
            self.dist_advanced = 0.0
            in_safe_zone = (abs(error_x) <= self.deadband_x and abs(error_y) <= self.deadband_y)

            if in_safe_zone:
                vx = 0.0
                vy = 0.0
                yaw_rate = 0.0
                substate = 'SAFE_ZONE_HOVER'
            else:
                if abs(error_x) > self.deadband_x:
                    excess_x = error_x - (self.deadband_x if error_x > 0 else -self.deadband_x)
                    yaw_rate = self.kp * excess_x
                    vy = self.kp_lateral * excess_x
                    vy = max(-0.4, min(0.4, vy))
                else:
                    yaw_rate = 0.0
                    vy = 0.0

                if self.enable_forward:
                    if error_y < -self.deadband_y:
                        boost = self.kp_y_boost * (-error_y - self.deadband_y)
                        vx = self.default_walk_speed + boost
                        substate = 'ADVANCING'
                    elif error_y > self.deadband_y:
                        boost = self.kp_y_boost * (error_y - self.deadband_y)
                        vx = -(self.default_backup_speed + boost)
                        substate = 'BACKING_SMOOTH'
                        # Keep active yaw tracking when backing up to track turns,
                        # but clamp yaw_rate and vy to prevent camera jerk / IoU drops
                        yaw_rate = max(-0.20, min(0.20, yaw_rate))
                        vy = max(-0.25, min(0.25, vy))
                    else:
                        vx = 0.0
                        substate = 'LATERAL_YAW_ONLY'
                else:
                    vx = 0.0

                if abs(error_x) > 70.0:
                    scale = max(0.60, 1.0 - (abs(error_x) - 70.0) / 150.0)
                    vx *= scale

                vx = max(self.min_forward_speed, min(self.max_forward_speed, vx))

        elif not acquired_once:
            substate = 'WAITING_FOR_PERSON'
            vx, vy, vz, yaw_rate = 0.0, 0.0, 0.0, 0.0
        else:
            if self.last_seen_y > self.deadband_y:
                # Near-bottom target lost: back away smoothly for up to 1.8s, then scan in place
                if age <= self.bottom_backup_timeout:
                    substate = 'BACKING_UP_TO_RECOVER'
                    vx = -self.default_backup_speed
                    vy = 0.0
                    yaw_rate = self.target_turn_dir * 0.15
                else:
                    substate = 'SEARCHING'
                    yaw_rate = self.direction * self.search_rate
                    vx, vy = 0.0, 0.0
            else:
                if self.dist_advanced < target_dist and age <= 4.0:
                    substate = 'ADVANCING_TO_TURN_POINT'
                    speed = 1.0
                    vx = speed * (target_dx / target_dist)
                    vy = speed * (target_dy / target_dist)
                    yaw_rate = 0.0
                    self.dist_advanced += speed * dt
                elif age <= 7.0:
                    substate = 'ROTATING_AT_TURN_POINT'
                    vx, vy = 0.0, 0.0
                    yaw_rate = self.target_turn_dir * 0.35
                else:
                    substate = 'SEARCHING'
                    yaw_rate = self.direction * self.search_rate
                    vx, vy = 0.0, 0.0

        yaw_rate = max(-self.max_rate, min(self.max_rate, yaw_rate))

        # Slew rate limiters (Ramp acceleration filters) to prevent jerking
        max_accel_x = 1.2 * dt
        vx = max(self._last_vx - max_accel_x, min(self._last_vx + max_accel_x, vx))
        max_accel_y = 1.2 * dt
        vy = max(self._last_vy - max_accel_y, min(self._last_vy + max_accel_y, vy))
        max_yaw_accel = 1.0 * dt
        yaw_rate = max(self._last_yaw_rate - max_yaw_accel, min(self._last_yaw_rate + max_yaw_accel, yaw_rate))

        if substate != self.last_tracking_substate or (substate.startswith('BACKING') and abs(vx - self._last_vx) > 0.15):
            self.get_logger().info(
                f"[TRACKING substate: {substate}] vx={vx:.2f} m/s, vy={vy:.2f} m/s, yaw_rate={yaw_rate*57.3:+.1f} deg/s"
            )
            self.last_tracking_substate = substate

        self._last_vx = vx
        self._last_vy = vy
        self._last_yaw_rate = yaw_rate

        return vx, vy, vz, yaw_rate


def main(args=None):
    rclpy.init(args=args)
    node = MotionArbiter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(node, 'diag_file_handle') and node.diag_file_handle is not None:
            try:
                node.diag_file_handle.close()
            except Exception:
                pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
