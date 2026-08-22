#!/usr/bin/env python3
"""Yaw the SITL vehicle toward a person and scan when detection is lost."""

import time
import math
import threading

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from pymavlink import mavutil


class VehicleYawSearch(Node):
    def __init__(self):
        super().__init__('vehicle_yaw_search')
        self.declare_parameter('mavlink', 'tcp:127.0.0.1:5760')
        self.declare_parameter('kp', 0.0035)
        self.declare_parameter('max_rate', 0.40)
        self.declare_parameter('search_rate', 0.20)
        self.declare_parameter('deadband', 10.0)
        self.declare_parameter('lost_timeout', 1.2)
        self.declare_parameter('auto_takeoff', True)
        self.declare_parameter('takeoff_alt', 4.5)
        self.declare_parameter('enable_forward', True)
        self.declare_parameter('default_walk_speed', 0.85)
        self.declare_parameter('kp_y_boost', 0.0070)
        self.declare_parameter('kp_lateral', 0.0020)
        self.declare_parameter('deadband_x', 20.0)
        self.declare_parameter('deadband_y', 25.0)
        self.declare_parameter('max_forward_speed', 1.8)
        self.declare_parameter('min_forward_speed', -1.2)
        self.declare_parameter('default_backup_speed', 0.75)
        self.declare_parameter('tree_clearance_margin', 1.8)
        gp = self.get_parameter
        self.kp = float(gp('kp').value)
        self.max_rate = abs(float(gp('max_rate').value))
        self.search_rate = abs(float(gp('search_rate').value))
        self.lost_timeout = float(gp('lost_timeout').value)
        self.auto_takeoff = bool(gp('auto_takeoff').value)
        self.takeoff_alt = float(gp('takeoff_alt').value)
        self.enable_forward = bool(gp('enable_forward').value)
        self.default_walk_speed = float(gp('default_walk_speed').value)
        self.default_backup_speed = float(gp('default_backup_speed').value)
        self.kp_y_boost = float(gp('kp_y_boost').value)
        self.kp_lateral = float(gp('kp_lateral').value)
        self.deadband_x = float(gp('deadband_x').value)
        self.deadband_y = float(gp('deadband_y').value)
        self.max_forward_speed = float(gp('max_forward_speed').value)
        self.min_forward_speed = float(gp('min_forward_speed').value)
        self.tree_clearance_margin = float(gp('tree_clearance_margin').value)
        self.error_x = None
        self.error_y = None
        self.area = None
        self.last_seen = 0.0
        self.acquired_once = False
        self.direction = 1.0
        self.estimated_dist_to_person = 4.5
        self.dist_advanced = 0.0
        self.last_tick_time = time.time()
        self.target_turn_dir = 1.0
        self.last_control_state = None
        self._last_vx = 0.0
        self.lock = threading.Lock()
        self.create_subscription(Point, '/tracking/error', self.on_error, 10)
        self.master = None
        for _ in range(30):
            try:
                candidate = mavutil.mavlink_connection(gp('mavlink').value)
                if candidate.wait_heartbeat(timeout=1.0):
                    self.master = candidate
                    break
            except Exception as exc:  # noqa: BLE001
                self.get_logger().debug('MAVLink connect: %s' % exc)
            time.sleep(0.5)
        if self.master is None:
            raise RuntimeError('cannot connect to SITL MAVLink')

        # Request MAVLink telemetry streams
        self.master.mav.request_data_stream_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1
        )

        if self.auto_takeoff:
            self.arm_and_takeoff()
        self.timer = self.create_timer(0.1, self.tick)
        self.get_logger().info('vehicle yaw & forward follower ready')

    def arm_and_takeoff(self):
        """Wait for EKF/GPS alignment, enter GUIDED, arm, and takeoff."""
        m = self.master

        # 1. Wait for EKF navigation origin and GPS lock
        self.get_logger().info('waiting for EKF origin and GPS lock (up to 30s)...')
        deadline = time.time() + 30.0
        while time.time() < deadline:
            msg = m.recv_match(blocking=True, timeout=1.0)
            if msg is not None:
                if msg.get_type() == 'STATUSTEXT':
                    self.get_logger().info('ArduPilot: %s' % msg.text)
                    if any(k in msg.text for k in ['Origin set', 'EKF3', 'is using GPS']):
                        self.get_logger().info('EKF Origin ready!')
                        break
                elif msg.get_type() == 'GPS_RAW_INT' and msg.fix_type >= 3:
                    self.get_logger().info('GPS fix acquired!')
                    break
            time.sleep(0.1)

        time.sleep(2.0)

        # 2. Enter GUIDED mode
        self.get_logger().info('waiting for GUIDED mode')
        deadline = time.time() + 15.0
        guided = False
        while time.time() < deadline:
            m.set_mode_apm('GUIDED')
            heartbeat = m.recv_match(
                type='HEARTBEAT', blocking=True, timeout=1.0)
            if heartbeat is not None and heartbeat.custom_mode == 4:
                guided = True
                break
            time.sleep(0.5)
        if not guided:
            raise RuntimeError('vehicle did not enter GUIDED mode')

        # 3. Arm vehicle
        self.get_logger().info('arming vehicle')
        deadline = time.time() + 15.0
        while time.time() < deadline and not m.motors_armed():
            m.mav.command_long_send(
                m.target_system, m.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, 1, 21196, 0, 0, 0, 0, 0)
            time.sleep(0.5)
            m.recv_match(type='HEARTBEAT', blocking=True, timeout=1.0)
        if not m.motors_armed():
            raise RuntimeError('vehicle did not arm')

        self.get_logger().info('motors armed; starting takeoff')
        time.sleep(0.5)

        # 4. Command takeoff
        self.get_logger().info('taking off to %.1f m' % self.takeoff_alt)
        takeoff_accepted = False
        for attempt in range(1, 10):
            m.mav.command_long_send(
                m.target_system, m.target_component,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                0,
                0, 0, 0, 0, 0, 0, float(self.takeoff_alt))
            ack = None
            ack_deadline = time.time() + 3.0
            while time.time() < ack_deadline:
                incoming = m.recv_match(blocking=True, timeout=0.5)
                if incoming is None:
                    continue
                if incoming.get_type() == 'STATUSTEXT':
                    self.get_logger().warn(
                        'ArduPilot: %s' % incoming.text)
                if (incoming.get_type() == 'COMMAND_ACK'
                        and incoming.command
                        == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF):
                    ack = incoming
                    break
            if ack is not None:
                self.get_logger().info(
                    'takeoff ACK attempt %d: result=%d'
                    % (attempt, ack.result))
                if ack.result in (
                        mavutil.mavlink.MAV_RESULT_ACCEPTED,
                        mavutil.mavlink.MAV_RESULT_IN_PROGRESS):
                    takeoff_accepted = True
                    break
            else:
                self.get_logger().warn(
                    'takeoff ACK attempt %d timed out' % attempt)
            time.sleep(1.5)

        if not takeoff_accepted:
            raise RuntimeError('takeoff command was not accepted')

        # 5. Monitor climb to takeoff altitude
        deadline = time.time() + 60.0
        last_log = 0.0
        while time.time() < deadline:
            msg = m.recv_match(
                type=['GLOBAL_POSITION_INT', 'HEARTBEAT'],
                blocking=True, timeout=1.0)
            if msg is None:
                continue
            if msg.get_type() == 'HEARTBEAT':
                if not (msg.base_mode
                        & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                    raise RuntimeError('vehicle disarmed during takeoff')
                continue
            altitude = msg.relative_alt / 1000.0
            if time.time() - last_log >= 1.0:
                self.get_logger().info('altitude %.2f m' % altitude)
                last_log = time.time()
            if altitude >= self.takeoff_alt * 0.90:
                self.get_logger().info(
                    'takeoff complete at %.1f m' % altitude)
                return
        raise RuntimeError('takeoff altitude was not reached')

    def on_error(self, msg):
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

            # Tính toán chính xác vị trí và khoảng cách 3D từ Drone tới người:
            # - Camera focal length: f_y = 178.07 px, f_x = 133.55 px (trong 416x416 frame)
            # - Góc nghiêng camera: pitch_base = 0.65 rad (~37.2 độ chúc)
            # - Độ cao tương đối từ camera tới thân người: h_rel = takeoff_alt - 0.90m
            h_rel = max(1.5, self.takeoff_alt - 0.90)
            alpha_y = math.atan2(self.error_y, 178.07)
            alpha_x = math.atan2(self.error_x, 133.55)
            theta_dep = max(0.15, min(1.45, 0.65 + alpha_y))

            dx = h_rel / math.tan(theta_dep)
            dy = dx * math.tan(alpha_x)

            # Cộng thêm tree_clearance_margin (+1.8m) để Drone bay vượt hẳn qua tán cây/cành lá
            # trước khi thực hiện xoay hướng, tránh tối đa việc chạm cành cây!
            target_dx = dx + self.tree_clearance_margin
            total_dist = math.sqrt(target_dx * target_dx + dy * dy)

            self.target_dx = max(2.5, min(16.0, target_dx))
            self.target_dy = max(-8.0, min(8.0, dy))
            self.target_dist = max(3.0, min(16.0, total_dist))
            self.dist_advanced = 0.0

    def tick(self):
        now = time.time()
        dt = max(0.01, min(0.20, now - self.last_tick_time))
        self.last_tick_time = now

        with self.lock:
            error_x = self.error_x
            error_y = self.error_y
            area = self.area
            age = now - self.last_seen if self.last_seen else 999.0
            acquired_once = self.acquired_once
            last_seen_x = getattr(self, 'last_seen_x', 0.0)
            target_dist = getattr(self, 'target_dist', 4.5)
            target_dx = getattr(self, 'target_dx', 4.5)
            target_dy = getattr(self, 'target_dy', 0.0)

        if error_x is not None and age <= self.lost_timeout:
            state = 'TRACKING'
            self.dist_advanced = 0.0

            # 1. 50% Safe Zone Deadband Check
            in_safe_zone = (abs(error_x) <= self.deadband_x and abs(error_y) <= self.deadband_y)

            if in_safe_zone:
                # Inside 50% safe zone: Drone hovers stable and calm
                vx = 0.0
                vy = 0.0
                yaw_rate = 0.0
            else:
                # 2. Outside Safe Zone:
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
                        # Người ở nửa trên khung hình (xa hơn) -> Tiến tới theo người
                        boost = self.kp_y_boost * (-error_y - self.deadband_y)
                        vx = self.default_walk_speed + boost
                    elif error_y > self.deadband_y:
                        # Người ở góc dưới khung hình (quá gần / sắp trôi ra ngoài) -> LÙI VỀ PHÍA SAU!
                        boost = self.kp_y_boost * (error_y - self.deadband_y)
                        vx = -(self.default_backup_speed + boost)
                    else:
                        vx = 0.0
                else:
                    vx = 0.0

                if abs(error_x) > 70.0:
                    scale = max(0.4, 1.0 - (abs(error_x) - 70.0) / 100.0)
                    vx *= scale

                vx = max(self.min_forward_speed, min(self.max_forward_speed, vx))

        elif not acquired_once:
            state = 'WAITING_FOR_PERSON'
            yaw_rate = 0.0
            vx = 0.0
            vy = 0.0
        else:
            # Khi người bị che khuất hoặc rẽ:
            if getattr(self, 'last_seen_y', 0.0) > self.deadband_y:
                # Người vừa ở mép dưới: LÙI THẲNG VỀ PHÍA SAU, TUYỆT ĐỐI KHÔNG XOAY TRÒN 360!
                if age <= 4.0:
                    state = 'BACKING_UP_TO_RECOVER'
                    vx = -0.85  # Lùi lại mở rộng tầm nhìn về phía trước
                    vy = 0.0
                    yaw_rate = 0.0  # Khóa cứng góc xoay 0.0, không xoay vòng
                else:
                    state = 'SEARCHING'
                    yaw_rate = self.direction * self.search_rate
                    vx = 0.0
                    vy = 0.0
            else:
                # Người rẽ sang trái/phải hoặc đi khuất sau khúc cua:
                # GIAI ĐOẠN 1: Bay theo vector đường thẳng (dx, dy) tới đúng điểm rẽ trước khi xoay!
                if self.dist_advanced < target_dist and age <= 10.0:
                    state = 'ADVANCING_TO_TURN_POINT'
                    speed = 1.35  # Tốc độ bay thẳng 1.35 m/s
                    vx = speed * (target_dx / target_dist)
                    vy = speed * (target_dy / target_dist)
                    yaw_rate = 0.0  # Khóa cứng 0.0, bay thẳng chính xác theo đường thẳng tới mục tiêu
                    self.dist_advanced += speed * dt
                elif age <= 10.0:
                    # GIAI ĐOẠN 2: Đã bay tới đúng vị trí điểm rẽ -> Xoay Drone sang góc rẽ của người!
                    state = 'ROTATING_AT_TURN_POINT'
                    vx = 0.0
                    vy = 0.0
                    yaw_rate = self.target_turn_dir * 0.50  # ~28.6 deg/s xoay đón đầu góc rẽ
                else:
                    state = 'SEARCHING'
                    yaw_rate = self.direction * self.search_rate
                    vx = 0.0
                    vy = 0.0

        yaw_rate = max(-self.max_rate, min(self.max_rate, yaw_rate))

        if state != self.last_control_state or (state == 'TRACKING' and abs(vx - self._last_vx) > 0.2) or state == 'ADVANCING_TO_TURN_POINT':
            if state == 'ADVANCING_TO_TURN_POINT':
                self.get_logger().info(
                    'ADVANCING_TO_TURN_POINT vx=%.2f m/s (progress=%.1f/%.1f m) yaw_rate=0.0'
                    % (vx, self.dist_advanced, target_dist))
            else:
                self.get_logger().info(
                    '%s yaw_rate=%+.1f deg/s vx=%.2f m/s vy=%.2f m/s (ex=%.0f, ey=%.0f)'
                    % (state, yaw_rate * 57.2957795, vx, vy, error_x if error_x else 0.0, error_y if error_y else 0.0))
            self.last_control_state = state
            self._last_vx = vx

        m = self.master
        velocity_yaw_rate_mask = 0x05C7
        m.mav.set_position_target_local_ned_send(
            0, m.target_system, m.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            velocity_yaw_rate_mask,
            0, 0, 0,
            float(vx), float(vy), 0.0,
            0, 0, 0,
            0.0, float(yaw_rate))


def main():
    rclpy.init()
    node = VehicleYawSearch()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
