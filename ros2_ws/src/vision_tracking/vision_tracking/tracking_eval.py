#!/usr/bin/env python3
"""Quantitative evaluation node for Phase 2 (step H).

Subscribes to /tracking/error, the camera image and the gimbal commands, then
prints a summary on shutdown:

    detection availability, error statistics (MAE / max / RMS),
    settling time and overshoot after the first acquisition,
    camera and error publish rates.
"""

import argparse
import math
import time

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64


class TrackingEval(Node):

    def __init__(self, duration, settle_band, loss_threshold=1.0):
        super().__init__('tracking_eval')
        self.declare_parameter(
            'image_topic',
            '/world/iris_runway/model/iris_with_gimbal/model/gimbal/link/'
            'pitch_link/sensor/camera/image')
        image_topic = self.get_parameter('image_topic').value

        self.duration = duration
        self.settle_band = settle_band
        self.loss_threshold = loss_threshold
        self.t0 = time.time()

        self.err = []          # (t, ex, ey, area)
        self.n_images = 0
        self.yaw = []
        self.pitch = []

        self.create_subscription(Point, '/tracking/error', self.on_err, 20)
        self.create_subscription(Image, image_topic, self.on_img, 5)
        self.create_subscription(Float64, '/gimbal/cmd_yaw', lambda m: self.yaw.append(m.data), 20)
        self.create_subscription(Float64, '/gimbal/cmd_pitch',
                                 lambda m: self.pitch.append(m.data), 20)
        self.get_logger().info('collecting for %.0fs ...' % duration)

    def on_err(self, m):
        self.err.append((time.time() - self.t0, m.x, m.y, m.z))

    def on_img(self, _m):
        self.n_images += 1

    def done(self):
        return time.time() - self.t0 >= self.duration

    # ------------------------------------------------------------------
    def report(self):
        el = time.time() - self.t0
        n = len(self.err)
        print('\n===== PHASE 2 TRACKING EVALUATION =====')
        print('window                : %.1f s' % el)
        print('camera frames         : %d  (%.2f Hz wall)' % (self.n_images, self.n_images / el))
        print('tracking/error msgs   : %d  (%.2f Hz wall)' % (n, n / el))
        if self.n_images:
            print('detection availability: %.1f %%  (error msgs / camera frames)'
                  % (100.0 * n / self.n_images))
        if not n:
            print('no tracking error samples -> detection FAILED')
            return
        ax = [abs(e[1]) for e in self.err]
        ay = [abs(e[2]) for e in self.err]
        print('mean |error_x|        : %.1f px' % (sum(ax) / n))
        print('mean |error_y|        : %.1f px' % (sum(ay) / n))
        print('max  |error_x|        : %.1f px' % max(ax))
        print('max  |error_y|        : %.1f px' % max(ay))
        print('rms  error_x          : %.1f px' % math.sqrt(sum(v * v for v in ax) / n))
        print('rms  error_y          : %.1f px' % math.sqrt(sum(v * v for v in ay) / n))
        # steady state = last 30% of the window
        tail = self.err[int(0.7 * n):]
        if tail:
            print('steady-state mean |ex|: %.1f px' % (sum(abs(e[1]) for e in tail) / len(tail)))
            print('steady-state mean |ey|: %.1f px' % (sum(abs(e[2]) for e in tail) / len(tail)))
        # settling: first time both errors stay inside the band
        settle = None
        for i, e in enumerate(self.err):
            if max(abs(e[1]), abs(e[2])) <= self.settle_band:
                if all(max(abs(f[1]), abs(f[2])) <= self.settle_band * 2
                       for f in self.err[i:i + 5]):
                    settle = e[0]
                    break
        print('settling time (<=%.0fpx): %s'
              % (self.settle_band, '%.1f s' % settle if settle is not None else 'not reached'))
        if settle is not None:
            after = [e for e in self.err if e[0] > settle]
            if after:
                print('overshoot after settle: %.1f px'
                      % max(max(abs(e[1]), abs(e[2])) for e in after))
        if self.yaw:
            print('yaw cmd   range       : %.3f .. %.3f rad' % (min(self.yaw), max(self.yaw)))
        if self.pitch:
            print('pitch cmd range       : %.3f .. %.3f rad' % (min(self.pitch), max(self.pitch)))
        self.report_losses(el, n)
        print('=======================================')

    # ------------------------------------------------------------------
    def report_losses(self, el, n):
        """Track-loss statistics and availability restricted to engaged time.

        Raw availability counts the whole window, including the part of the
        actor trajectory that is outside the camera FOV.  §51 asks for the
        detection rate while the actor *is* in the FOV, so also report the rate
        over the engaged time, i.e. the window minus every gap longer than
        loss_threshold (those gaps are the out-of-FOV / occluded stretches).
        """
        gaps = []
        for prev, cur in zip(self.err, self.err[1:]):
            gap = cur[0] - prev[0]
            if gap > self.loss_threshold:
                gaps.append(gap)
        lost_time = sum(gaps)
        engaged = max(1e-6, el - lost_time)
        print('track losses (>%.1fs)   : %d' % (self.loss_threshold, len(gaps)))
        if gaps:
            print('loss duration mean/max: %.2f s / %.2f s'
                  % (lost_time / len(gaps), max(gaps)))
        print('engaged time          : %.1f s of %.1f s (%.0f %%)'
              % (engaged, el, 100.0 * engaged / el))
        cam_rate = self.n_images / el if el else 0.0
        expected = engaged * cam_rate
        if expected > 0:
            print('availability (engaged): %.1f %%  (error msgs / frames while engaged)'
                  % min(100.0, 100.0 * n / expected))


def main(args=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--duration', type=float, default=60.0)
    ap.add_argument('--settle-band', type=float, default=20.0)
    known, ros_args = ap.parse_known_args()

    rclpy.init(args=ros_args if ros_args else args)
    node = TrackingEval(known.duration, known.settle_band)
    try:
        while rclpy.ok() and not node.done():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    node.report()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
