#!/usr/bin/env python3
"""
Configurable sensor/camera fault injection for repeatable simulation tests.

The node deliberately sits between Gazebo/ros_gz_bridge and the autonomy
nodes.  Defaults are disabled, so a normal run is unchanged.  Enable a profile
with ``--ros-args --params-file simulation/realism.yaml``.
"""

from collections import deque
import random
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import FluidPressure, Image, Imu, NavSatFix


class SimulationRealism(Node):
    def __init__(self):
        super().__init__('simulation_realism')
        d = self.declare_parameter
        d('camera_input', '/camera/image_raw')
        d('camera_output', '/simulation/camera/image')
        d('camera_delay_ms', 0.0)
        d('camera_drop_probability', 0.0)
        d('camera_motion_blur_pixels', 0)
        d('camera_queue_depth', 8)
        d('imu_input', '')
        d('imu_output', '/simulation/imu')
        d('imu_noise_stddev', 0.0)
        d('imu_bias', [0.0, 0.0, 0.0])
        d('imu_drift_per_s', [0.0, 0.0, 0.0])
        d('gps_input', '')
        d('gps_output', '/simulation/gps')
        d('gps_noise_m', 0.0)
        d('gps_dropout_probability', 0.0)
        d('baro_input', '')
        d('baro_output', '/simulation/baro')
        d('baro_noise_pa', 0.0)
        d('sensor_dropout_probability', 0.0)
        d('seed', 7)
        random.seed(int(self.get_parameter('seed').value))
        self.bridge = CvBridge()
        self.camera_queue = deque()
        self.imu_bias = np.array(self.get_parameter('imu_bias').value, dtype=float)
        self.imu_drift = np.array(self.get_parameter('imu_drift_per_s').value, dtype=float)
        self.last_t = time.monotonic()
        gp = self.get_parameter
        self.camera_pub = self.create_publisher(Image, gp('camera_output').value, 5)
        self.camera_sub = self.create_subscription(
            Image, gp('camera_input').value, self.camera_cb, 5)
        if gp('imu_input').value:
            self.imu_pub = self.create_publisher(Imu, gp('imu_output').value, 10)
            self.create_subscription(Imu, gp('imu_input').value, self.imu_cb, 10)
        if gp('gps_input').value:
            self.gps_pub = self.create_publisher(NavSatFix, gp('gps_output').value, 10)
            self.create_subscription(NavSatFix, gp('gps_input').value, self.gps_cb, 10)
        if gp('baro_input').value:
            self.baro_pub = self.create_publisher(FluidPressure, gp('baro_output').value, 10)
            self.create_subscription(FluidPressure, gp('baro_input').value, self.baro_cb, 10)
        self.create_timer(0.005, self.flush_camera)

    def camera_cb(self, msg):
        if random.random() < float(self.get_parameter('camera_drop_probability').value):
            return
        try:
            header = msg.header
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            k = int(self.get_parameter('camera_motion_blur_pixels').value)
            if k > 1:
                if k % 2 == 0:
                    k += 1
                image = cv2.GaussianBlur(image, (k, k), 0)
            msg = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
            msg.header = header
            delay = float(self.get_parameter('camera_delay_ms').value) / 1000.0
            self.camera_queue.append((time.monotonic() + delay, msg))
            depth = int(self.get_parameter('camera_queue_depth').value)
            while len(self.camera_queue) > depth:
                self.camera_queue.popleft()
        except Exception as exc:
            self.get_logger().warning(f'camera fault injection skipped frame: {exc}')

    def flush_camera(self):
        now = time.monotonic()
        while self.camera_queue and self.camera_queue[0][0] <= now:
            self.camera_pub.publish(self.camera_queue.popleft()[1])

    def _drop(self):
        return random.random() < float(self.get_parameter('sensor_dropout_probability').value)

    def imu_cb(self, msg):
        if self._drop():
            return
        now = time.monotonic()
        dt = max(0.0, now - self.last_t)
        self.last_t = now
        self.imu_bias += self.imu_drift * dt
        n = float(self.get_parameter('imu_noise_stddev').value)
        for field in ('linear_acceleration', 'angular_velocity'):
            v = getattr(msg, field)
            v.x += self.imu_bias[0] + random.gauss(0, n)
            v.y += self.imu_bias[1] + random.gauss(0, n)
            v.z += self.imu_bias[2] + random.gauss(0, n)
        self.imu_pub.publish(msg)

    def gps_cb(self, msg):
        if (self._drop()
                or random.random() < float(
                    self.get_parameter('gps_dropout_probability').value)):
            return
        n = float(self.get_parameter('gps_noise_m').value) / 111111.0
        msg.latitude += random.gauss(0, n)
        msg.longitude += random.gauss(0, n)
        msg.altitude += random.gauss(
            0, float(self.get_parameter('gps_noise_m').value))
        self.gps_pub.publish(msg)

    def baro_cb(self, msg):
        if self._drop():
            return
        noise = float(self.get_parameter('baro_noise_pa').value)
        msg.fluid_pressure += random.gauss(0, noise)
        self.baro_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimulationRealism()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
