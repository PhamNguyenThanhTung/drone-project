#!/usr/bin/env python3
"""dt-based PID gimbal controller for Phase 2.

Input : geometry_msgs/Point on /tracking/error  (x = error_x px, y = error_y px)
Output: std_msgs/Float64 on /gimbal/cmd_yaw and /gimbal/cmd_pitch (rad)

The PID output is interpreted as a commanded angular *rate* which is
integrated into the absolute joint angle command.  A pixel error of zero must
hold the current pointing angle, so an absolute error->angle mapping cannot
work for a tracking gimbal.

Sign convention measured on this world (see C3/F5):
    /gimbal/cmd_pitch > 0  -> camera looks DOWN
    /gimbal/cmd_yaw   > 0  -> camera rotates LEFT (scene moves right)
Therefore, to drive the error to zero:
    error_x > 0 (target right of centre) -> yaw command must DECREASE  -> yaw_sign = -1
    error_y > 0 (target below centre)    -> pitch command must INCREASE -> pitch_sign = +1
Both signs are ROS parameters so they can be flipped without editing code.
"""

import time

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from std_msgs.msg import Float64

STATE_TRACKING = 'TRACKING'
STATE_LOST = 'LOST'
STATE_SEARCHING = 'SEARCHING'


def clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


class Axis:
    """One PID axis with dt-based integral/derivative and anti-windup."""

    def __init__(self, kp, ki, kd, integral_limit):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limit = integral_limit
        self.integral = 0.0
        self.prev_error = 0.0
        self.has_prev = False

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.has_prev = False

    def update(self, error, dt, saturated_sign):
        """Return the PID output. saturated_sign: -1/0/+1 direction already clamped."""
        # anti-windup: do not integrate further into an existing saturation
        candidate = self.integral + error * dt
        if not (saturated_sign != 0 and (error * saturated_sign) > 0.0):
            self.integral = clamp(candidate, -self.integral_limit, self.integral_limit)

        derivative = 0.0
        if self.has_prev and dt > 0.0:
            derivative = (error - self.prev_error) / dt
        self.prev_error = error
        self.has_prev = True

        return self.kp * error + self.ki * self.integral + self.kd * derivative


class GimbalControllerNode(Node):

    def __init__(self):
        super().__init__('gimbal_controller_node')

        self.declare_parameter('error_topic', '/tracking/error')
        self.declare_parameter('yaw_topic', '/gimbal/cmd_yaw')
        self.declare_parameter('pitch_topic', '/gimbal/cmd_pitch')

        # gains from the step-G calibration (G0 -> G1 -> G2) measured on this
        # world: Kp=0.010 gave mean |error_x| 7.7 px, Kd=0.001 cut the settling
        # time to 1.4 s, and any Ki > 0 made overshoot worse, so Ki stays 0
        self.declare_parameter('kp_yaw', 0.010)
        self.declare_parameter('ki_yaw', 0.0)
        self.declare_parameter('kd_yaw', 0.001)
        self.declare_parameter('kp_pitch', 0.010)
        self.declare_parameter('ki_pitch', 0.0)
        self.declare_parameter('kd_pitch', 0.001)

        self.declare_parameter('yaw_sign', -1.0)
        self.declare_parameter('pitch_sign', 1.0)

        self.declare_parameter('min_yaw', -2.80)
        self.declare_parameter('max_yaw', 2.80)
        # measured convention: positive pitch = camera down, so the useful
        # range 'level -> straight down' is 0 .. +1.57 rad on this model
        self.declare_parameter('min_pitch', -0.20)
        self.declare_parameter('max_pitch', 1.57)

        self.declare_parameter('max_yaw_rate', 0.50)
        self.declare_parameter('max_pitch_rate', 0.50)
        self.declare_parameter('integral_limit', 100.0)
        self.declare_parameter('deadband', 3.0)
        # detection is intermittent on the 10 Hz Phase 1 camera, so a single
        # measurement gap can be long; clamp the dt used for integration so one
        # late sample cannot slew the gimbal far enough to lose the target
        self.declare_parameter('max_dt', 0.30)
        self.declare_parameter('target_timeout', 1.0)
        self.declare_parameter('integral_reset_timeout', 3.0)
        self.declare_parameter('control_period', 0.05)
        self.declare_parameter('log_period', 1.0)
        self.declare_parameter('search_enabled', True)
        self.declare_parameter('search_delay', 1.0)
        self.declare_parameter('search_yaw_rate', 0.20)

        self.declare_parameter('init_yaw', 0.0)
        self.declare_parameter('init_pitch', 0.0)

        gp = self.get_parameter
        self.yaw_sign = float(gp('yaw_sign').value)
        self.pitch_sign = float(gp('pitch_sign').value)
        self.min_yaw = float(gp('min_yaw').value)
        self.max_yaw = float(gp('max_yaw').value)
        self.min_pitch = float(gp('min_pitch').value)
        self.max_pitch = float(gp('max_pitch').value)
        self.max_yaw_rate = float(gp('max_yaw_rate').value)
        self.max_pitch_rate = float(gp('max_pitch_rate').value)
        self.deadband = float(gp('deadband').value)
        self.max_dt = float(gp('max_dt').value)
        self.target_timeout = float(gp('target_timeout').value)
        self.integral_reset_timeout = float(gp('integral_reset_timeout').value)
        self.log_period = float(gp('log_period').value)
        self.control_period = float(gp('control_period').value)
        self.search_enabled = bool(gp('search_enabled').value)
        self.search_delay = float(gp('search_delay').value)
        self.search_yaw_rate = abs(float(gp('search_yaw_rate').value))

        ilim = float(gp('integral_limit').value)
        self.ax_yaw = Axis(float(gp('kp_yaw').value), float(gp('ki_yaw').value),
                           float(gp('kd_yaw').value), ilim)
        self.ax_pitch = Axis(float(gp('kp_pitch').value), float(gp('ki_pitch').value),
                             float(gp('kd_pitch').value), ilim)

        self.pub_yaw = self.create_publisher(Float64, gp('yaw_topic').value, 10)
        self.pub_pitch = self.create_publisher(Float64, gp('pitch_topic').value, 10)
        self.create_subscription(Point, gp('error_topic').value, self.on_error, 10)

        self.yaw_cmd = float(gp('init_yaw').value)
        self.pitch_cmd = float(gp('init_pitch').value)
        self.state = 'WAITING_FOR_PERSON'
        self.acquired_once = False
        self.last_error = None
        self.last_msg_time = 0.0
        self.prev_time = None
        self.last_dt = 0.0
        self.yaw_sat = 0
        self.pitch_sat = 0
        self.search_direction = 1.0

        self.n_samples = 0
        self.sum_ax = 0.0
        self.sum_ay = 0.0
        self.max_ax = 0.0
        self.max_ay = 0.0
        self.last_log = time.time()

        self.timer = self.create_timer(self.control_period, self.on_timer)
        self.get_logger().info(
            'gimbal_controller_node ready: yaw_sign=%+.0f pitch_sign=%+.0f '
            'yaw[%.2f,%.2f] pitch[%.2f,%.2f] deadband=%.1fpx'
            % (self.yaw_sign, self.pitch_sign, self.min_yaw, self.max_yaw,
               self.min_pitch, self.max_pitch, self.deadband))

    # ------------------------------------------------------------------
    def on_error(self, msg):
        raw_ex, raw_ey = msg.x, msg.y
        if self.last_error is None:
            smooth_ex, smooth_ey = raw_ex, raw_ey
        else:
            alpha = 0.55
            smooth_ex = alpha * raw_ex + (1.0 - alpha) * self.last_error[0]
            smooth_ey = alpha * raw_ey + (1.0 - alpha) * self.last_error[1]
        self.last_error = (smooth_ex, smooth_ey)
        if abs(smooth_ex) >= self.deadband:
            # Remember the physical gimbal direction in which the target was
            # last moving. Search continues that way after it leaves the view.
            self.search_direction = self.yaw_sign * (1.0 if smooth_ex > 0.0 else -1.0)
        self.last_msg_time = time.time()
        self.acquired_once = True
        self.n_samples += 1
        self.sum_ax += abs(msg.x)
        self.sum_ay += abs(msg.y)
        self.max_ax = max(self.max_ax, abs(msg.x))
        self.max_ay = max(self.max_ay, abs(msg.y))
        # the control update is driven by measurement arrival, so dt is the real
        # detector period; the timer below only handles the LOST watchdog
        self.control_step()

    # ------------------------------------------------------------------
    def on_timer(self):
        now = time.time()
        age = now - self.last_msg_time if self.last_msg_time else 1e9
        if self.last_error is None or age > self.target_timeout:
            self.on_lost(age)
            self.prev_time = None
        self.publish()
        self.log()

    # ------------------------------------------------------------------
    def control_step(self):
        now = time.time()
        dt = (0.05 if (self.prev_time is None or now - self.prev_time <= 0.0)
              else (now - self.prev_time))
        self.prev_time = now
        self.last_dt = dt
        if dt > self.max_dt:
            dt = self.max_dt

        if self.state != STATE_TRACKING:
            # fresh acquisition: start from a clean integrator
            self.ax_yaw.reset()
            self.ax_pitch.reset()
            self.get_logger().info('state LOST -> TRACKING')
            self.state = STATE_TRACKING

        error_x, error_y = self.last_error
        if abs(error_x) < self.deadband:
            error_x = 0.0
        if abs(error_y) < self.deadband:
            error_y = 0.0

        if dt <= 0.0:
            dt = 0.05

        # PID output is a commanded angular rate (rad/s)
        u_yaw = self.ax_yaw.update(error_x, dt, self.yaw_sat)
        u_pitch = self.ax_pitch.update(error_y, dt, self.pitch_sat)

        u_yaw = clamp(u_yaw, -self.max_yaw_rate, self.max_yaw_rate)
        u_pitch = clamp(u_pitch, -self.max_pitch_rate, self.max_pitch_rate)

        new_yaw = self.yaw_cmd + self.yaw_sign * u_yaw * dt
        new_pitch = self.pitch_cmd + self.pitch_sign * u_pitch * dt

        clamped_yaw = clamp(new_yaw, self.min_yaw, self.max_yaw)
        clamped_pitch = clamp(new_pitch, self.min_pitch, self.max_pitch)

        # remember which direction of *error* is currently saturating the axis
        self.yaw_sat = 0 if clamped_yaw == new_yaw else int(
            self.yaw_sign * (1 if new_yaw > clamped_yaw else -1))
        self.pitch_sat = 0 if clamped_pitch == new_pitch else int(
            self.pitch_sign * (1 if new_pitch > clamped_pitch else -1))

        self.yaw_cmd = clamped_yaw
        self.pitch_cmd = clamped_pitch
        self.publish()
        self.log(error_x, error_y)

    # ------------------------------------------------------------------
    def on_lost(self, age):
        if not self.acquired_once:
            self.yaw_cmd = float(self.get_parameter('init_yaw').value)
            self.pitch_cmd = float(self.get_parameter('init_pitch').value)
            self.state = 'WAITING_FOR_PERSON'
            return

        should_search = self.search_enabled and age >= self.search_delay
        next_state = STATE_SEARCHING if should_search else STATE_LOST
        if self.state != next_state:
            action = 'scanning yaw' if should_search else 'holding angles'
            self.get_logger().warn(
                'no tracking error for %.2fs -> %s (%s)' % (age, next_state, action))
            self.state = next_state

        if should_search and self.search_yaw_rate > 0.0:
            next_yaw = (
                self.yaw_cmd
                + self.search_direction * self.search_yaw_rate * self.control_period)
            if next_yaw >= self.max_yaw:
                next_yaw = self.max_yaw
                self.search_direction = -1.0
            elif next_yaw <= self.min_yaw:
                next_yaw = self.min_yaw
                self.search_direction = 1.0
            self.yaw_cmd = next_yaw
        elif self.state != STATE_LOST:
            self.state = STATE_LOST

        # Stop integrating and drop the integral if the target stays lost.
        if age > self.integral_reset_timeout:
            self.ax_yaw.reset()
            self.ax_pitch.reset()

    def publish(self):
        self.pub_yaw.publish(Float64(data=float(self.yaw_cmd)))
        self.pub_pitch.publish(Float64(data=float(self.pitch_cmd)))

    def log(self, error_x=None, error_y=None):
        now = time.time()
        if now - self.last_log < self.log_period:
            return
        self.last_log = now
        mae_x = self.sum_ax / self.n_samples if self.n_samples else 0.0
        mae_y = self.sum_ay / self.n_samples if self.n_samples else 0.0
        self.get_logger().info(
            'state=%s err=(%s,%s) yaw_cmd=%.3f pitch_cmd=%.3f dt=%.3f '
            'MAE=(%.1f,%.1f) MAX=(%.1f,%.1f) n=%d'
            % (self.state,
               'None' if error_x is None else '%.1f' % error_x,
               'None' if error_y is None else '%.1f' % error_y,
               self.yaw_cmd, self.pitch_cmd, self.last_dt,
               mae_x, mae_y, self.max_ax, self.max_ay, self.n_samples))


def main(args=None):
    rclpy.init(args=args)
    node = GimbalControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
