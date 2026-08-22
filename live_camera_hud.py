#!/usr/bin/env python3
"""
Ultra-fast, zero-lag OpenCV HUD Viewer for Live Drone Camera POV:
- Subscribes to ROS 2 topic: /tracking/debug_image or raw camera image
- Renders directly with OpenCV highgui
- Displays Real-time FPS, Altitude/Status overlay
"""

import argparse
import sys
import time
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class LiveCameraHUD(Node):
    def __init__(self, topic_name):
        super().__init__('live_camera_hud')
        self.bridge = CvBridge()
        self.fps = 0.0
        self.frame_count = 0
        self.last_time = time.time()
        
        self.subscription = self.create_subscription(
            Image,
            topic_name,
            self.image_callback,
            1  # QoS queue_size=1 for lowest latency
        )
        self.window_name = "Live Drone Camera POV (Direct Stream)"
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, 960, 720)
        self.get_logger().info(f"LiveCameraHUD started. Subscribed to {topic_name}. Waiting for frames...")

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"cv_bridge conversion error: {e}")
            return

        self.frame_count += 1
        now = time.time()
        elapsed = now - self.last_time
        if elapsed >= 1.0:
            self.fps = self.frame_count / elapsed
            self.frame_count = 0
            self.last_time = now

        # Add clean top-right FPS watermark
        ih, iw = frame.shape[:2]
        fps_text = f"FPS: {self.fps:.1f}"
        (tw, th), _ = cv2.getTextSize(fps_text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        cv2.rectangle(frame, (iw - tw - 20, 10), (iw - 10, 20 + th + 4), (30, 30, 30), -1)
        cv2.putText(
            frame,
            fps_text,
            (iw - tw - 15, 16 + th),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 0),
            1,
            cv2.LINE_AA
        )

        cv2.imshow(self.window_name, frame)
        cv2.waitKey(1)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--topic',
        type=str,
        default='/tracking/debug_image',
        help='ROS 2 Image topic'
    )
    args, _ = parser.parse_known_args()

    rclpy.init()
    hud = LiveCameraHUD(args.topic)
    try:
        rclpy.spin(hud)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        hud.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
