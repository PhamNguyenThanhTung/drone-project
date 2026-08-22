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
from geometry_msgs.msg import Point
from std_msgs.msg import Int32
from cv_bridge import CvBridge


class LiveCameraHUD(Node):
    def __init__(self, topic_name):
        super().__init__('live_camera_hud')
        self.bridge = CvBridge()
        self.fps = 0.0
        self.frame_count = 0
        self.last_time = time.time()
        self.last_frame_w = 640
        self.last_frame_h = 480
        
        self.pub_select = self.create_publisher(Int32, '/tracking/select_target', 10)
        self.pub_click = self.create_publisher(Point, '/tracking/click_point', 10)

        self.subscription = self.create_subscription(
            Image,
            topic_name,
            self.image_callback,
            1  # QoS queue_size=1 for lowest latency
        )
        self.window_name = "Live Drone Camera POV (Direct Stream)"
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, 960, 720)
        cv2.setMouseCallback(self.window_name, self.on_mouse)
        self.get_logger().info(f"LiveCameraHUD started. Subscribed to {topic_name}. Interactive click & keys [1-9, 0, SPACE] active.")

    def on_mouse(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            # Map window coordinates (960x720) to native frame resolution
            scale_x = self.last_frame_w / 960.0 if self.last_frame_w else 1.0
            scale_y = self.last_frame_h / 720.0 if self.last_frame_h else 1.0
            fx = float(x) * scale_x
            fy = float(y) * scale_y
            msg = Point()
            msg.x = fx
            msg.y = fy
            msg.z = 0.0
            self.pub_click.publish(msg)
            self.get_logger().info(f"[HUD Click] Clicked at ({fx:.0f}, {fy:.0f}) -> Locking clicked target")

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"cv_bridge conversion error: {e}")
            return

        ih, iw = frame.shape[:2]
        self.last_frame_w = iw
        self.last_frame_h = ih

        self.frame_count += 1
        now = time.time()
        elapsed = now - self.last_time
        if elapsed >= 1.0:
            self.fps = self.frame_count / elapsed
            self.frame_count = 0
            self.last_time = now

        # Add clean top-right FPS watermark
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
        key = cv2.waitKey(1) & 0xFF
        if ord('1') <= key <= ord('9'):
            target_id = key - ord('0')
            msg = Int32()
            msg.data = target_id
            self.pub_select.publish(msg)
            self.get_logger().info(f"[HUD Key] Target LOCKED to ID: {target_id}")
        elif key == ord('0') or key == ord(' '):
            msg = Int32()
            msg.data = -1
            self.pub_select.publish(msg)
            self.get_logger().info("[HUD Key] Target CLEARED -> Drone STANDBY (Hover in place)")


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
