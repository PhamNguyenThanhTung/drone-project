#!/usr/bin/env python3
"""
Ultra-fast, zero-lag OpenCV HUD Viewer with Unified Flight Control:
- Subscribes to /tracking/debug_image
- Key [TAB] or [T]: Arm & Takeoff to 4.0m
- Key [P]: Land
- Mouse Click & Keys [1-9]: Interactive Target Lock
- Keys [0 / SPACE]: Standby Hover
- Keys [W/A/S/D/Q/E/R/F/X]: Instant Manual Flight Teleop Override
- Minimap (25 m, click = GOTO position setpoint) + live GPS readout below it
- Displays Live State (MANUAL / TRACKING / STANDBY), FPS, Altitude & Telemetry Instructions
"""

import argparse
import json
import math
import os
import sys
import threading
import time
import traceback
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, NavSatFix
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import Int32, String
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class LiveCameraHUD(Node):
    def __init__(self, topic_name=None):
        super().__init__('live_camera_hud')

        self.declare_parameter('topic', '/tracking/camera_relay')
        self.declare_parameter('overlay_topic', '/tracking/overlay')
        self.declare_parameter('teleop_speed', 2.0)
        self.declare_parameter('teleop_z_speed', 1.0)
        self.declare_parameter('teleop_yaw_speed', 1.10)
        self.declare_parameter('map_radius_m', 25.0)
        self.declare_parameter('display_fps', 30.0)

        if topic_name is None:
            resolved_topic = self.get_parameter('topic').get_parameter_value().string_value
        else:
            resolved_topic = topic_name
        self.topic = resolved_topic
        self.overlay_topic = self.get_parameter('overlay_topic').get_parameter_value().string_value
        self.is_debug_image_topic = (resolved_topic == '/tracking/debug_image')

        self.teleop_speed = self.get_parameter('teleop_speed').get_parameter_value().double_value
        self.teleop_z_speed = self.get_parameter('teleop_z_speed').get_parameter_value().double_value
        self.teleop_yaw_speed = self.get_parameter('teleop_yaw_speed').get_parameter_value().double_value
        self.map_radius_m = self.get_parameter('map_radius_m').get_parameter_value().double_value
        self.display_fps = max(
            1.0,
            self.get_parameter('display_fps').get_parameter_value().double_value,
        )

        self.bridge = CvBridge()
        self.fps = 0.0
        self.frame_count = 0
        self.last_time = time.time()
        self.last_frame_w = 640
        self.last_frame_h = 480
        self.last_window_w = 960
        self.last_window_h = 720
        self._frame_lock = threading.Lock()
        self._latest_frame = None
        self._latest_frame_seq = 0
        self._displayed_frame_seq = 0
        self._display_stop = threading.Event()
        self._display_thread = None
        self._display_timer = None
        self._image_callback_count = 0
        self._display_count = 0
        self._last_display_diag = 0.0
        self._display_exception_count = 0

        # Step 2: Overlay metadata storage
        self._overlay_lock = threading.Lock()
        self._latest_overlay = None

        # Minimap assumption: a fixed 25 m radius around the first valid GPS
        # fix (home). North is up and east is right. This keeps the HUD useful
        # without needing a map server or extra network service.
        self.home_lat = None
        self.home_lon = None
        self.current_lat = None
        self.current_lon = None
        self.current_alt = None
        self.minimap_rect = None

        # State tracking
        self.current_state = "STANDBY"
        self.active_target_id = -1
        self.flight_status = "READY"

        # Publishers
        self.pub_select = self.create_publisher(Int32, '/tracking/select_target', 10)
        self.pub_click = self.create_publisher(Point, '/tracking/click_point', 10)
        self.pub_goto = self.create_publisher(Point, '/tracking/goto_gps', 10)
        self.pub_teleop = self.create_publisher(Twist, '/teleop/cmd_vel', 10)
        self.pub_action = self.create_publisher(String, '/teleop/flight_action', 10)

        self.command_trace_file = None
        self.command_trace_lock = threading.Lock()
        self.command_sequence = 0
        try:
            trace_path = os.environ.get(
                'COMMAND_LATENCY_LOG',
                os.path.join(os.getcwd(), 'logs', 'command_latency_trace.jsonl'),
            )
            os.makedirs(os.path.dirname(trace_path) or '.', exist_ok=True)
            self.command_trace_file = open(trace_path, 'a', buffering=1)
        except Exception as exc:
            self.get_logger().warning(f'Command latency trace disabled: {exc}')

        # Subscriptions
        self.create_subscription(String, '/tracking/motion_state', self.on_motion_state, 10)
        self.create_subscription(NavSatFix, '/tracking/gps', self.on_gps, 10)
        self.sub_overlay = self.create_subscription(
            String, self.overlay_topic, self.on_overlay, 10)
        debug_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.subscription = self.create_subscription(
            Image, resolved_topic, self.image_callback, debug_qos)

        self.window_name = "Live Drone Camera POV (Direct Stream)"
        self.gui_available = True
        try:
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.window_name, 960, 720)
            cv2.setMouseCallback(self.window_name, self.on_mouse)
        except cv2.error as e:
            self.gui_available = False
            self.get_logger().warn(f"OpenCV GUI not initialized (headless/no display): {e}")

        if self.gui_available:
            # OpenCV's Qt backend is not thread-safe: namedWindow/imshow and
            # waitKey must run on the same (ROS executor) thread that created
            # the window.  The previous daemon display thread could therefore
            # leave a live but black WSLg window.  Keep frame delivery
            # asynchronous, but schedule GUI work through a ROS timer.
            self._display_timer = self.create_timer(
                1.0 / self.display_fps, self._display_tick)

        self.get_logger().info(
            f"LiveCameraHUD ready on {resolved_topic}. Controls: [TAB/T]=Takeoff(4m), [P]=Land, "
            f"Click/[1-9]=Lock Target, 0/SPACE=Hover, "
            f"W/S=Fwd/Back A/D=Left/Right R=UP F=DOWN Q/E=Yaw X=Stop"
        )

    def on_motion_state(self, msg: String):
        try:
            parts = msg.data.split(':')
            self.current_state = parts[0]
            if len(parts) > 1:
                self.active_target_id = int(parts[1])
            if len(parts) > 2:
                self.flight_status = parts[2]
        except Exception:
            self.current_state = msg.data

    def on_gps(self, msg: NavSatFix):
        if not (math.isfinite(msg.latitude) and math.isfinite(msg.longitude)):
            return
        if abs(msg.latitude) < 1e-9 and abs(msg.longitude) < 1e-9:
            return
        if self.home_lat is None:
            self.home_lat = float(msg.latitude)
            self.home_lon = float(msg.longitude)
            self.get_logger().info(
                f'[HUD] Minimap home set to {self.home_lat:.7f}, {self.home_lon:.7f}'
            )
        self.current_lat = float(msg.latitude)
        self.current_lon = float(msg.longitude)
        self.current_alt = float(msg.altitude) if math.isfinite(msg.altitude) else None

    def _minimap_to_gps(self, px, py):
        """Convert a minimap pixel to WGS84 using the fixed home-centered scale."""
        if self.home_lat is None or self.home_lon is None or self.minimap_rect is None:
            return None
        x0, y0, size = self.minimap_rect
        east_m = ((px - (x0 + size / 2.0)) / (size / 2.0)) * self.map_radius_m
        north_m = ((y0 + size / 2.0 - py) / (size / 2.0)) * self.map_radius_m
        lat = self.home_lat + north_m / 111320.0
        lon_scale = 111320.0 * max(0.2, math.cos(math.radians(self.home_lat)))
        lon = self.home_lon + east_m / lon_scale
        return lat, lon

    def _publish_goto(self, lat, lon):
        msg = Point()
        msg.x = float(lat)
        msg.y = float(lon)
        msg.z = float(self.current_alt) if self.current_alt is not None else 0.0
        self.pub_goto.publish(msg)
        self.get_logger().info(
            f'[HUD Minimap] GOTO position setpoint: lat={lat:.7f}, lon={lon:.7f}'
        )

    def _write_command_trace(self, record):
        handle = getattr(self, 'command_trace_file', None)
        if handle is None or handle.closed:
            return
        try:
            with self.command_trace_lock:
                handle.write(json.dumps(record, separators=(',', ':')) + '\n')
        except Exception:
            pass

    def on_overlay(self, msg: String):
        try:
            data = json.loads(msg.data)
            with self._overlay_lock:
                self._latest_overlay = data
        except Exception:
            pass

    def on_mouse(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            try:
                _wx, _wy, self.last_window_w, self.last_window_h = cv2.getWindowImageRect(self.window_name)
            except cv2.error:
                pass
            # WINDOW_NORMAL preserves aspect ratio, so account for letterbox
            # margins before translating the click into frame coordinates.
            display_scale = min(
                self.last_window_w / max(1.0, float(self.last_frame_w)),
                self.last_window_h / max(1.0, float(self.last_frame_h)),
            )
            shown_w = self.last_frame_w * display_scale
            shown_h = self.last_frame_h * display_scale
            offset_x = (self.last_window_w - shown_w) / 2.0
            offset_y = (self.last_window_h - shown_h) / 2.0
            fx = (float(x) - offset_x) / max(display_scale, 1e-6)
            fy = (float(y) - offset_y) / max(display_scale, 1e-6)
            if not (0.0 <= fx < self.last_frame_w and 0.0 <= fy < self.last_frame_h):
                return

            # Minimap clicks are position commands, not target-lock clicks.
            if self.minimap_rect is not None:
                x0, y0, size = self.minimap_rect
                if x0 <= fx < x0 + size and y0 <= fy < y0 + size:
                    gps = self._minimap_to_gps(fx, fy)
                    if gps is not None:
                        self._publish_goto(*gps)
                    return

            with self._overlay_lock:
                overlay = self._latest_overlay
            if overlay is not None:
                src_w = float(overlay.get('source_w', self.last_frame_w))
                src_h = float(overlay.get('source_h', self.last_frame_h))
                click_x = fx * (src_w / max(1.0, float(self.last_frame_w)))
                click_y = fy * (src_h / max(1.0, float(self.last_frame_h)))
            else:
                click_x, click_y = fx, fy

            msg = Point()
            msg.x = click_x
            msg.y = click_y
            msg.z = 0.0
            self.pub_click.publish(msg)
            self.get_logger().info(f"[HUD Click] Clicked at ({click_x:.0f}, {click_y:.0f}) -> requesting target lock")

    def draw_minimap_and_gps(self, frame):
        """Draw the home-centered minimap plus the live GPS readout under it."""
        ih, iw = frame.shape[:2]
        map_size = max(120, min(190, int(min(iw * 0.30, ih * 0.36))))
        map_x, map_y = 12, 12
        self.minimap_rect = (map_x, map_y, map_size)
        cv2.rectangle(frame, (map_x, map_y), (map_x + map_size, map_y + map_size), (25, 25, 25), -1)
        cv2.rectangle(frame, (map_x, map_y), (map_x + map_size, map_y + map_size), (0, 200, 255), 2)
        center = (map_x + map_size // 2, map_y + map_size // 2)
        cv2.line(frame, (center[0], map_y + 8), (center[0], map_y + map_size - 8), (70, 70, 70), 1)
        cv2.line(frame, (map_x + 8, center[1]), (map_x + map_size - 8, center[1]), (70, 70, 70), 1)
        cv2.putText(frame, 'N', (center[0] - 5, map_y + 16), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (220, 220, 220), 1)
        if self.home_lat is not None and self.current_lat is not None:
            north_m = (self.current_lat - self.home_lat) * 111320.0
            east_m = (self.current_lon - self.home_lon) * 111320.0 * max(0.2, math.cos(math.radians(self.home_lat)))
            mx = int(center[0] + east_m / self.map_radius_m * (map_size / 2.0))
            my = int(center[1] - north_m / self.map_radius_m * (map_size / 2.0))
            mx = max(map_x + 5, min(map_x + map_size - 5, mx))
            my = max(map_y + 5, min(map_y + map_size - 5, my))
            cv2.circle(frame, (mx, my), 5, (0, 255, 0), -1)
            cv2.line(frame, center, (mx, my), (0, 180, 0), 1)
        cv2.putText(frame, 'MINIMAP 25m', (map_x + 7, map_y + map_size - 7),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.36, (220, 220, 220), 1)

        # GPS readout directly under the minimap. The dot alone was not enough:
        # without numbers the pilot cannot cross-check the HUD against
        # QGroundControl, and "no fix yet" looked identical to "drone parked
        # exactly at home".
        gps_x = map_x
        gps_y = map_y + map_size + 6
        if self.current_lat is None:
            gps_lines = [('GPS: NO FIX (waiting for PX4)', (0, 165, 255))]
        else:
            dist_m = 0.0
            if self.home_lat is not None:
                north_m = (self.current_lat - self.home_lat) * 111320.0
                east_m = ((self.current_lon - self.home_lon) * 111320.0
                          * max(0.2, math.cos(math.radians(self.home_lat))))
                dist_m = math.hypot(north_m, east_m)
            alt_txt = '--' if self.current_alt is None else f'{self.current_alt:5.2f}'
            gps_lines = [
                (f'LAT {self.current_lat:+.7f}', (0, 255, 180)),
                (f'LON {self.current_lon:+.7f}', (0, 255, 180)),
                (f'ALT {alt_txt} m  HOME {dist_m:5.1f} m', (0, 255, 180)),
            ]
        panel_h = 6 + 15 * len(gps_lines)
        # Size the panel to the widest line: a fixed width clipped the trailing
        # "m" of the ALT/HOME row.
        panel_w = max(map_size, *[
            cv2.getTextSize(line, cv2.FONT_HERSHEY_SIMPLEX, 0.38, 1)[0][0] + 12
            for line, _c in gps_lines
        ])
        cv2.rectangle(frame, (gps_x, gps_y), (gps_x + panel_w, gps_y + panel_h), (25, 25, 25), -1)
        cv2.rectangle(frame, (gps_x, gps_y), (gps_x + panel_w, gps_y + panel_h), (0, 200, 255), 1)
        for i, (line, color) in enumerate(gps_lines):
            cv2.putText(frame, line, (gps_x + 6, gps_y + 15 + 15 * i),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.38, color, 1, cv2.LINE_AA)

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"cv_bridge conversion error: {e}")
            return

        ih, iw = frame.shape[:2]
        # Keep the ROS callback lightweight. The display thread owns all
        # OpenCV drawing/window calls and always consumes the newest frame.
        with self._frame_lock:
            self.last_frame_w = iw
            self.last_frame_h = ih
            self._latest_frame = frame
            self._latest_frame_seq += 1
            self._image_callback_count += 1

    def _display_tick(self):
        if self._display_stop.is_set():
            return
        with self._frame_lock:
            if (self._latest_frame is None
                    or self._latest_frame_seq == self._displayed_frame_seq):
                frame = None
            else:
                frame = self._latest_frame.copy()
                self._displayed_frame_seq = self._latest_frame_seq
        if frame is None:
            try:
                self._handle_key(cv2.waitKey(1) & 0xFF)
            except cv2.error:
                pass
            return
        try:
            self._render_frame(frame)
            self._display_count += 1
        except Exception as exc:  # noqa: BLE001
            self._display_exception_count += 1
            self.get_logger().error(
                f'HUD display callback failed ({self._display_exception_count}): '
                f'{exc}\n{traceback.format_exc()}'
            )
        now = time.monotonic()
        if now - self._last_display_diag >= 5.0:
            self._last_display_diag = now
            self.get_logger().info(
                f'[HUD] callbacks={self._image_callback_count} '
                f'displayed={self._display_count} gui_timer_active='
                f'{self._display_timer is not None} '
                f'exceptions={self._display_exception_count}'
            )

    def draw_overlay(self, frame):
        """Draw detection bounding boxes and status badge onto raw camera frame."""
        with self._overlay_lock:
            overlay = self._latest_overlay
        if overlay is None:
            return

        ih, iw = frame.shape[:2]

        # Draw 50% active tracking safe zone box
        zx1, zx2 = int(0.25 * iw), int(0.75 * iw)
        zy1, zy2 = int(0.25 * ih), int(0.75 * ih)
        z_color = (0, 255, 255)
        bracket_len = 25
        cv2.line(frame, (zx1, zy1), (zx1 + bracket_len, zy1), z_color, 1)
        cv2.line(frame, (zx1, zy1), (zx1, zy1 + bracket_len), z_color, 1)
        cv2.line(frame, (zx2, zy1), (zx2 - bracket_len, zy1), z_color, 1)
        cv2.line(frame, (zx2, zy1), (zx2, zy1 + bracket_len), z_color, 1)
        cv2.line(frame, (zx1, zy2), (zx1 + bracket_len, zy2), z_color, 1)
        cv2.line(frame, (zx1, zy2), (zx1, zy2 - bracket_len), z_color, 1)
        cv2.line(frame, (zx2, zy2), (zx2 - bracket_len, zy2), z_color, 1)
        cv2.line(frame, (zx2, zy2), (zx2, zy2 - bracket_len), z_color, 1)
        cv2.putText(
            frame, "50% SAFE ZONE", (zx1 + 5, zy1 + 14),
            cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 255, 255), 1, cv2.LINE_AA)

        # Check overlay staleness (e.g. if >2.5s without detection update)
        overlay_stamp = overlay.get('stamp', 0.0)
        is_stale = (time.time() - overlay_stamp > 2.5) if overlay_stamp > 0 else False

        src_w = float(overlay.get('source_w', iw))
        src_h = float(overlay.get('source_h', ih))
        scale_x = iw / max(src_w, 1.0)
        scale_y = ih / max(src_h, 1.0)

        if not is_stale:
            # 1. Other persons (clickable, thin grey)
            for p in overlay.get('other_persons', []):
                bx = p.get('box', [])
                if len(bx) == 4:
                    x1 = int(bx[0] * scale_x)
                    y1 = int(bx[1] * scale_y)
                    x2 = int(bx[2] * scale_x)
                    y2 = int(bx[3] * scale_y)
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (150, 150, 150), 1)
                    cv2.putText(
                        frame, "person (click)", (x1 + 3, max(12, y1 - 4)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.38, (150, 150, 150), 1, cv2.LINE_AA)

            # 2. Tracked candidates
            handle_id = overlay.get('handle_id')
            for cand in overlay.get('cands', []):
                bx = cand.get('box', [])
                if len(bx) != 4:
                    continue
                x1 = int(bx[0] * scale_x)
                y1 = int(bx[1] * scale_y)
                x2 = int(bx[2] * scale_x)
                y2 = int(bx[3] * scale_y)
                is_target = cand.get('is_target', False)
                color = (0, 255, 0) if is_target else (255, 180, 0)

                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 3 if is_target else 2)

                if is_target and handle_id:
                    lock_mark = f" [{handle_id} LOCK]"
                elif is_target:
                    lock_mark = " [LOCK]"
                else:
                    lock_mark = ""
                label = "PERSON%s" % lock_mark
                (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
                label_y = max(th + 4, y1)
                cv2.rectangle(frame, (x1, label_y - th - 4), (x1 + tw + 6, label_y + 2), color, -1)
                cv2.putText(
                    frame, label, (x1 + 3, label_y - 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 1, cv2.LINE_AA)

        # 3. Status banner
        state = overlay.get('state')
        handle_id = overlay.get('handle_id')
        lock_mode = overlay.get('lock_mode', 'MANUAL')

        if is_stale:
            status_text = "DETECTOR STALE / SEARCHING..."
            badge_color = (0, 140, 255)
        elif state == 'TRACKING':
            lock_label = handle_id if (handle_id and lock_mode == 'MANUAL') else 'AUTO'
            status_text = f"TRACKING [{lock_label}]"
            badge_color = (0, 200, 0)
        elif state == 'UNCERTAIN':
            handle_str = f" [{handle_id}]" if handle_id else ""
            status_text = f"TARGET UNCERTAIN{handle_str} - HOLDING"
            badge_color = (0, 165, 255)
        elif state == 'TARGET_LOST':
            handle_str = f" [{handle_id}]" if handle_id else ""
            status_text = f"TARGET LOST{handle_str}"
            badge_color = (0, 0, 255)
        elif state in ('TARGET_SELECTED', 'TARGET_LOCKED'):
            handle_str = f" [{handle_id}]" if handle_id else ""
            status_text = f"TARGET LOCKED{handle_str}"
            badge_color = (255, 255, 0)
        elif overlay.get('cands') or overlay.get('other_persons'):
            status_text = "CLICK A PERSON TO LOCK"
            badge_color = (0, 200, 255)
        else:
            status_text = "SEARCHING PERSON..."
            badge_color = (0, 140, 255)

        (sw, sh), _ = cv2.getTextSize(status_text, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
        bx = int(iw * 0.33)
        cv2.rectangle(frame, (bx, 10), (bx + sw + 10, 20 + sh + 6), (30, 30, 30), -1)
        cv2.rectangle(frame, (bx, 10), (bx + sw + 10, 20 + sh + 6), badge_color, 2)
        cv2.putText(
            frame, status_text, (bx + 5, 16 + sh),
            cv2.FONT_HERSHEY_SIMPLEX, 0.45, badge_color, 1, cv2.LINE_AA)

    def _render_frame(self, frame):
        ih, iw = frame.shape[:2]

        self.frame_count += 1
        now = time.time()
        elapsed = now - self.last_time
        if elapsed >= 1.0:
            self.fps = self.frame_count / elapsed
            self.frame_count = 0
            self.last_time = now

        # Add top-right FPS watermark
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

        # Step 2: Overlay boxes from detector metadata if consuming raw camera stream
        if not self.is_debug_image_topic:
            self.draw_overlay(frame)

        # Game-style minimap + GPS readout (own method so it can be rendered
        # and inspected headlessly, without an X display or a live PX4).
        self.draw_minimap_and_gps(frame)

        # Render State Machine Indicator Banner
        if self.current_state == "TRACKING":
            # No target id here either: the detector's track ids churn at CPU
            # frame rates and a jumping number read as "tracking keeps dying".
            state_str = "MODE: TRACKING"
            state_color = (0, 255, 0)
        elif self.current_state == "MANUAL_GOTO":
            state_str = "MODE: MANUAL GOTO (Minimap Position Setpoint)"
            state_color = (255, 120, 255)
        elif self.current_state == "STANDBY":
            state_str = "MODE: STANDBY (Hovering / Ready to Takeoff)"
            state_color = (0, 200, 255)
        else:
            state_str = "MODE: MANUAL TELEOP (Flight Keys Active)"
            state_color = (255, 180, 0)

        # The arbiter appends the real PX4 armed state to /tracking/motion_state.
        # Without this the banner claimed "Hovering / Ready to Takeoff" while the
        # drone was actually disarmed on the ground after a failsafe land.
        if self.flight_status == "DISARMED":
            state_str += " | PX4 DISARMED (on ground)"
            state_color = (0, 0, 255)

        (sw, sh), _ = cv2.getTextSize(state_str, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
        cv2.rectangle(frame, (10, ih - 60), (20 + sw, ih - 38), (20, 20, 20), -1)
        cv2.rectangle(frame, (10, ih - 60), (20 + sw, ih - 38), state_color, 1)
        cv2.putText(frame, state_str, (15, ih - 44), cv2.FONT_HERSHEY_SIMPLEX, 0.45, state_color, 1, cv2.LINE_AA)

        # Render Quick Help Guide Bar at bottom (font auto-shrinks to fit width)
        help_str = ("[TAB/T]: Takeoff(4m) | [P]: Land | [W/S]: Fwd/Back [A/D]: Left/Right "
                    "[R]: UP [F]: DOWN | [Q/E]: Yaw [X]: Stop | [1-9]: Lock | [0/SPACE]: Hover")
        (hw, hh), _ = cv2.getTextSize(help_str, cv2.FONT_HERSHEY_SIMPLEX, 0.38, 1)
        help_scale = min(0.38, max(0.22, (iw - 32.0) / max(hw, 1.0) * 0.38))
        (hw, hh), _ = cv2.getTextSize(help_str, cv2.FONT_HERSHEY_SIMPLEX, help_scale, 1)
        cv2.rectangle(frame, (10, ih - 32), (iw - 10, ih - 8), (15, 15, 15), -1)
        cv2.putText(frame, help_str, (16, ih - 16), cv2.FONT_HERSHEY_SIMPLEX, help_scale, (220, 220, 220), 1, cv2.LINE_AA)

        try:
            cv2.imshow(self.window_name, frame)
            key = cv2.waitKey(1) & 0xFF
        except cv2.error:
            key = 255

        self._handle_key(key)

    def _handle_key(self, key):
        if key == 255 or key < 0:
            return

        # -------------------------------------------------------------
        # KEY HANDLERS: TAB / T (Takeoff), P (Land), Numbers (Lock), Flight Keys
        # -------------------------------------------------------------
        if key in (9, ord('\t'), ord('t'), ord('T')):  # TAB or T key
            act_msg = String()
            act_msg.data = "TAKEOFF"
            self.pub_action.publish(act_msg)
            self.get_logger().info("[HUD Key TAB/T] Command TAKEOFF (4.0m) sent!")

        elif key in (ord('p'), ord('P')):
            act_msg = String()
            act_msg.data = "LAND"
            self.pub_action.publish(act_msg)
            self.get_logger().info("[HUD Key P] Command LAND sent!")

        elif ord('1') <= key <= ord('9'):
            target_id = key - ord('0')
            msg = Int32()
            msg.data = target_id
            self.pub_select.publish(msg)
            self.get_logger().info(f"[HUD Key] Target LOCKED to ID: {target_id} -> TRACKING")

        elif key == ord('0') or key == ord(' '):
            msg = Int32()
            msg.data = -1
            self.pub_select.publish(msg)
            self.get_logger().info("[HUD Key] Target CLEARED -> Drone STANDBY (Hover in place)")

        # Manual Keyboard Flight Teleop Controls
        elif key in (ord('w'), ord('W'), ord('s'), ord('S'),
                     ord('a'), ord('A'), ord('d'), ord('D'),
                     ord('q'), ord('Q'), ord('e'), ord('E'),
                     ord('r'), ord('R'), ord('f'), ord('F'),
                     ord('x'), ord('X')):
            twist = Twist()
            if key in (ord('w'), ord('W')):
                twist.linear.x = self.teleop_speed
            elif key in (ord('s'), ord('S')):
                twist.linear.x = -self.teleop_speed
            elif key in (ord('a'), ord('A')):
                twist.linear.y = -self.teleop_speed
            elif key in (ord('d'), ord('D')):
                twist.linear.y = self.teleop_speed
            elif key in (ord('r'), ord('R')):
                twist.linear.z = -self.teleop_z_speed  # Up in NED (negative Z is up)
            elif key in (ord('f'), ord('F')):
                twist.linear.z = self.teleop_z_speed   # Down in NED
            elif key in (ord('q'), ord('Q')):
                twist.angular.z = -self.teleop_yaw_speed  # Yaw left
            elif key in (ord('e'), ord('E')):
                twist.angular.z = self.teleop_yaw_speed   # Yaw right
            elif key in (ord('x'), ord('X')):
                # Brake / Stop
                twist.linear.x = 0.0
                twist.linear.y = 0.0
                twist.linear.z = 0.0
                twist.angular.z = 0.0

            self.pub_teleop.publish(twist)
            self.command_sequence += 1
            self._write_command_trace({
                'phase': 'published',
                'source': 'live_camera_hud',
                'sequence': self.command_sequence,
                'wall_time': time.time(),
                'monotonic_time': time.monotonic(),
                'ros_time': self.get_clock().now().nanoseconds / 1e9,
                'vx': float(twist.linear.x),
                'vy': float(twist.linear.y),
                'vz': float(twist.linear.z),
                'yaw_rate': float(twist.angular.z),
                'key': int(key),
            })
            self.get_logger().info(
                f"[HUD Teleop Key] Key pressed -> MANUAL OVERRIDE (Vx={twist.linear.x:+.1f}, "
                f"Vy={twist.linear.y:+.1f}, Vz={twist.linear.z:+.1f}, Yaw={twist.angular.z:+.1f})"
            )

    def stop_display(self):
        self._display_stop.set()
        if self._display_timer is not None:
            self._display_timer.cancel()


def main(args=None):
    parser = argparse.ArgumentParser(description='Live Camera HUD Node')
    parser.add_argument(
        '--topic',
        type=str,
        default=None,
        help='ROS 2 Image topic'
    )
    cli_args, ros_args = parser.parse_known_args()

    rclpy.init(args=args if args is not None else ros_args)
    hud = LiveCameraHUD(topic_name=cli_args.topic)
    try:
        rclpy.spin(hud)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        hud.stop_display()
        try:
            cv2.destroyAllWindows()
        except cv2.error:
            pass
        if hasattr(hud, 'command_trace_file') and hud.command_trace_file is not None:
            try:
                hud.command_trace_file.close()
            except Exception:
                pass
        hud.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
