#!/usr/bin/env python3
"""
High-Performance Interactive Drone Teleoperation & Live Cockpit
Based on proven Phase 2.5 Architecture:
- Gazebo Harmonic + ROS 2 Bridge + Hardware OpenCV Viewer
- ArduPilot SITL Autopilot with EKF Lock & GUIDED Flight State Machine
- Zero Physics Jitter (Tuned Gimbal Joint Damping)
- Smooth Keyboard Flight Controls (W/A/S/D/R/F/Q/E/X/1/2/3)
"""

import os
import signal
import subprocess
import sys
import threading
import time
import cv2
import numpy as np
from pymavlink import mavutil
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from cv_bridge import CvBridge

processes = []
app_running = True


def cleanup(signum=None, frame=None):
    global app_running
    app_running = False
    print("\n[Cockpit] Dọn dẹp tiến trình...")
    for p in processes:
        try:
            p.terminate()
            p.wait(timeout=1)
        except Exception:
            try:
                p.kill()
            except Exception:
                pass
    subprocess.run(["pkill", "-9", "-f", "gz si[m]"], check=False)
    subprocess.run(["pkill", "-9", "-x", "arducopter"], check=False)
    subprocess.run(["pkill", "-9", "-f", "mavproxy.p[y]"], check=False)
    subprocess.run(["pkill", "-9", "-f", "parameter_brid[g]e"], check=False)
    subprocess.run(["pkill", "-9", "-f", "yolo_detector_nod[e]"], check=False)
    subprocess.run(["pkill", "-9", "-f", "gimbal_controller_no[d]"], check=False)
    print("[Cockpit] Hoàn tất tắt hệ thống.")
    if signum:
        sys.exit(0)


signal.signal(signal.SIGINT, cleanup)
signal.signal(signal.SIGTERM, cleanup)


def start_process(cmd, name, logfile="/tmp/cockpit_proc.log", delay=2.0):
    print(f"[Cockpit] Đang khởi động: {name}...")
    f = open(logfile, "w")
    env = os.environ.copy()
    env["LD_LIBRARY_PATH"] = f"/usr/lib/wsl/lib:{env.get('LD_LIBRARY_PATH', '')}"
    env["DISPLAY"] = env.get("DISPLAY", ":0")
    p = subprocess.Popen(
        cmd,
        shell=True,
        executable="/bin/bash",
        stdout=f,
        stderr=subprocess.STDOUT,
        preexec_fn=os.setsid,
        env=env
    )
    processes.append(p)
    time.sleep(delay)
    return p


class DroneVisionNode(Node):
    def __init__(self):
        super().__init__('drone_vision_node')
        self.bridge = CvBridge()
        self.latest_frame = None
        self.fps = 0.0
        self.frame_count = 0
        self.last_time = time.time()
        self.lock = threading.Lock()

        # Subscribe trực tiếp raw image topic từ Gazebo
        self.raw_topic = '/world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image'
        self.sub = self.create_subscription(Image, self.raw_topic, self.image_callback, 1)

        # Topic điều khiển Gimbal
        self.pitch_pub = self.create_publisher(Float64, '/gimbal/cmd_pitch', 10)
        self.yaw_pub = self.create_publisher(Float64, '/gimbal/cmd_yaw', 10)

    def image_callback(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            with self.lock:
                self.latest_frame = cv_img
                self.frame_count += 1
                now = time.time()
                elapsed = now - self.last_time
                if elapsed >= 1.0:
                    self.fps = self.frame_count / elapsed
                    self.frame_count = 0
                    self.last_time = now
        except Exception:
            pass

    def set_gimbal(self, pitch_rad, yaw_rad=0.0):
        p = Float64()
        p.data = float(pitch_rad)
        self.pitch_pub.publish(p)
        y = Float64()
        y.data = float(yaw_rad)
        self.yaw_pub.publish(y)


class FlightManager:
    def __init__(self, target_alt=10.0):
        self.target_alt = target_alt
        self.status_text = "ĐANG KHỞI TẠO..."
        self.is_teleop_active = False
        self.current_alt = 0.0
        self.master = None
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.yaw_rate = 0.0
        self.cmd_lock = threading.Lock()

    def set_velocity(self, vx, vy, vz, yaw_rate):
        with self.cmd_lock:
            self.vx = float(vx)
            self.vy = float(vy)
            self.vz = float(vz)
            self.yaw_rate = float(yaw_rate)

    def run_flight_loop(self):
        # 1. Kết nối MAVLink
        self.status_text = "KẾT NỐI AUTOPILOT (MAVLink)..."
        for _ in range(40):
            if not app_running:
                return
            try:
                self.master = mavutil.mavlink_connection('tcp:127.0.0.1:5760', source_system=255)
                if self.master.wait_heartbeat(timeout=1.0):
                    break
            except Exception:
                pass
            time.sleep(0.5)

        if not self.master:
            self.status_text = "LỖI KẾT NỐI MAVLINK"
            return

        self.master.mav.request_data_stream_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1
        )

        # 2. Chờ EKF & GPS Origin
        t0 = time.time()
        while app_running and (time.time() - t0 < 25):
            msg = self.master.recv_match(blocking=False)
            if msg and msg.get_type() == 'STATUSTEXT':
                if any(k in msg.text for k in ["Origin set", "EKF3", "GPS", "Arming"]):
                    break
            remaining = int(25 - (time.time() - t0))
            self.status_text = f"ĐANG ĐỒNG BỘ EKF & CẢM BIẾN ({remaining}s)..."
            time.sleep(0.5)

        # 3. Chuyển GUIDED
        self.status_text = "CHUYỂN GUIDED MODE..."
        self.master.set_mode_apm('GUIDED')
        time.sleep(1.0)

        # 4. ARM động cơ
        self.status_text = "ARMING ĐỘNG CƠ..."
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 21196, 0, 0, 0, 0, 0
        )
        time.sleep(1.5)

        # 5. Cất cánh TAKEOFF
        self.status_text = f"CẤT CÁNH LÊN {self.target_alt:.0f}M..."
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0, self.target_alt
        )

        # 6. Theo dõi độ cao
        t0 = time.time()
        while app_running and (time.time() - t0 < 45):
            try:
                msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
                if msg:
                    self.current_alt = msg.relative_alt / 1000.0
                    self.status_text = f"ĐANG LÊN CAO: {self.current_alt:.1f}m / {self.target_alt:.0f}m"
                    if self.current_alt >= self.target_alt * 0.88:
                        break
            except Exception:
                pass
            time.sleep(0.1)

        self.status_text = "HOVER ỔN ĐỊNH - SẴN SÀNG ĐIỀU KHIỂN (TELEOP)"
        self.is_teleop_active = True

        # 7. Vòng lặp gửi vận tốc
        last_send = time.time()
        while app_running:
            try:
                msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
                if msg:
                    self.current_alt = msg.relative_alt / 1000.0

                if time.time() - last_send >= 0.1:
                    with self.cmd_lock:
                        vx, vy, vz, yr = self.vx, self.vy, self.vz, self.yaw_rate
                    self.master.mav.set_position_target_local_ned_send(
                        0,
                        self.master.target_system, self.master.target_component,
                        mavutil.mavlink.MAV_FRAME_BODY_NED,
                        0b0000011111000111,
                        0, 0, 0,
                        vx, vy, vz,
                        0, 0, 0,
                        0, yr
                    )
                    last_send = time.time()
            except Exception:
                pass
            time.sleep(0.02)


def main():
    target_alt = 10.0
    if len(sys.argv) > 1:
        try:
            target_alt = float(sys.argv[1])
        except ValueError:
            pass

    print(f"================================================================")
    print(f"   KHỞI CHẠY DRONE FPV COCKPIT & LIVE CONTROLLER ({target_alt}m)")
    print(f"================================================================")

    # 1. Dọn dẹp tiến trình cũ
    subprocess.run(["pkill", "-9", "-f", "gz si[m]"], check=False)
    subprocess.run(["pkill", "-9", "-x", "arducopter"], check=False)
    subprocess.run(["pkill", "-9", "-f", "mavproxy.p[y]"], check=False)
    subprocess.run(["pkill", "-9", "-f", "parameter_brid[g]e"], check=False)
    subprocess.run(["pkill", "-9", "-f", "yolo_detector_nod[e]"], check=False)
    subprocess.run(["pkill", "-9", "-f", "gimbal_controller_no[d]"], check=False)
    time.sleep(1)

    # 2. Khởi động Gazebo Harmonic Server (GPU Acceleration)
    start_process(
        "gz sim -v4 -r -s /home/tungt/ardupilot_gazebo/worlds/iris_runway.sdf",
        "Gazebo Harmonic Server (GPU Accelerated)",
        "/tmp/gz_sim.log",
        delay=3.5
    )

    # 3. Khởi động ROS 2 Parameter Bridge
    bridge_cmd = (
        "source /opt/ros/humble/setup.bash && "
        "ros2 run ros_gz_bridge parameter_bridge "
        "'/world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image@sensor_msgs/msg/Image[gz.msgs.Image' "
        "'/gimbal/cmd_yaw@std_msgs/msg/Float64]gz.msgs.Double' "
        "'/gimbal/cmd_pitch@std_msgs/msg/Float64]gz.msgs.Double'"
    )
    start_process(bridge_cmd, "ROS 2 Parameter Bridge", "/tmp/bridge.log", delay=2.0)

    # 4. Khởi động ArduPilot SITL
    sitl_cmd = (
        "cd /home/tungt/ardupilot/ArduCopter && "
        "/home/tungt/ardupilot/build/sitl/bin/arducopter "
        "--model JSON --speedup 1 --slave 0 "
        "--defaults /home/tungt/ardupilot_gazebo/config/gazebo-iris-gimbal.parm "
        "--sim-address=127.0.0.1 -I0"
    )
    start_process(sitl_cmd, "ArduPilot SITL Autopilot", "/tmp/sitl.log", delay=2.5)

    # 5. Khởi động ROS 2 Vision Node
    rclpy.init()
    vision_node = DroneVisionNode()
    ros_thread = threading.Thread(target=rclpy.spin, args=(vision_node,), daemon=True)
    ros_thread.start()

    # Khởi tạo góc Gimbal mặc định (chúc 35 độ nhìn phía trước)
    time.sleep(1.0)
    vision_node.set_gimbal(0.60)

    # 6. Khởi động Flight Manager trong background thread
    flight_mgr = FlightManager(target_alt=target_alt)
    flight_thread = threading.Thread(target=flight_mgr.run_flight_loop, daemon=True)
    flight_thread.start()

    # 7. Mở cửa sổ Camera HUD
    window_name = "DRONE FPV COCKPIT (60 FPS - Zero Jitter Flight Control)"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, 1080, 720)

    print("\n=======================================================")
    print("      HƯỚNG DẪN ĐIỀU KHIỂN BÀN PHÍM (CLICK VÀO CỬA SỔ CAMERA):")
    print("  [W] / [S]     : Tiến / Lùi (2.5 m/s)")
    print("  [A] / [D]     : Sang Trái / Sang Phải (2.5 m/s)")
    print("  [R] / [Space] : Bay Lên Cao (+1.2 m/s)")
    print("  [F] / [C]     : Hạ Xuống (-1.2 m/s)")
    print("  [Q] / [E]     : Xoay Mũi Trái / Phải (Yaw)")
    print("  [X]           : Phanh / Dừng Đứng Yên (Hover Brake)")
    print("  [1] / [2] / [3]: Đổi Góc Camera (Ngang 0° / Chúc 35° / Chúc 70°)")
    print("  [ESC]         : Thoát")
    print("=======================================================\n")

    vx, vy, vz = 0.0, 0.0, 0.0
    yaw_rate = 0.0
    speed = 2.5
    spinner_idx = 0
    spinner_chars = ["|", "/", "-", "\\"]

    try:
        while app_running:
            frame = None
            with vision_node.lock:
                if vision_node.latest_frame is not None:
                    frame = vision_node.latest_frame.copy()

            if frame is None:
                frame = np.zeros((720, 1080, 3), dtype=np.uint8)
                frame[:] = (30, 30, 30)
                spinner_idx = (spinner_idx + 1) % 4
                cv2.putText(frame, f"DANG KET NOI CAMERA SENSOR {spinner_chars[spinner_idx]}", 
                            (300, 340), cv2.FONT_HERSHEY_SIMPLEX, 0.85, (0, 255, 255), 2)
                cv2.putText(frame, "Dong bo Gazebo Harmonic GPU Pipeline...", 
                            (340, 390), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (180, 180, 180), 1)

            h, w = frame.shape[:2]

            # Top HUD Bar
            cv2.rectangle(frame, (0, 0), (w, 48), (20, 20, 20), -1)

            # FPS Counter
            fps_val = vision_node.fps
            fps_color = (0, 255, 0) if fps_val >= 25 else (0, 165, 255)
            cv2.putText(frame, f"FPS: {fps_val:.1f}", (15, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.7, fps_color, 2)

            # Altitude
            cv2.putText(frame, f"ALT: {flight_mgr.current_alt:.2f}m", (170, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

            # Flight Status Indicator
            status_color = (0, 255, 0) if flight_mgr.is_teleop_active else (0, 215, 255)
            cv2.putText(frame, flight_mgr.status_text, (340, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.6, status_color, 2)

            # Reticle / Crosshair trung tâm
            cx, cy = w // 2, h // 2
            cv2.line(frame, (cx - 20, cy), (cx + 20, cy), (0, 255, 0), 1)
            cv2.line(frame, (cx, cy - 20), (cx, cy + 20), (0, 255, 0), 1)
            cv2.circle(frame, (cx, cy), 10, (0, 255, 0), 1)

            # Bottom Status Bar
            cv2.rectangle(frame, (10, h - 45), (420, h - 10), (20, 20, 20), -1)
            vel_str = f"Vx:{vx:+.1f}  Vy:{vy:+.1f}  Vz:{vz:+.1f}  Yaw:{yaw_rate:+.1f}"
            cv2.putText(frame, vel_str, (18, h - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 0), 2)

            cv2.putText(frame, "W/A/S/D: Move | R/F: Alt | Q/E: Yaw | X: Brake | 1/2/3: Gimbal",
                        (w - 560, h - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (220, 220, 220), 1)

            cv2.imshow(window_name, frame)

            # Xử lý phím bấm điều khiển
            key = cv2.waitKey(15) & 0xFF
            if key == 27:  # ESC
                break
            elif flight_mgr.is_teleop_active:
                if key in (ord('w'), ord('W')):
                    vx, vy, vz, yaw_rate = speed, 0.0, 0.0, 0.0
                elif key in (ord('s'), ord('S')):
                    vx, vy, vz, yaw_rate = -speed, 0.0, 0.0, 0.0
                elif key in (ord('a'), ord('A')):
                    vx, vy, vz, yaw_rate = 0.0, -speed, 0.0, 0.0
                elif key in (ord('d'), ord('D')):
                    vx, vy, vz, yaw_rate = 0.0, speed, 0.0, 0.0
                elif key in (ord('r'), ord('R'), 32):  # R / Space: Lên
                    vx, vy, vz, yaw_rate = 0.0, 0.0, -1.2, 0.0
                elif key in (ord('f'), ord('F'), ord('c'), ord('C')):  # F / C: Xuống
                    vx, vy, vz, yaw_rate = 0.0, 0.0, 1.2, 0.0
                elif key in (ord('q'), ord('Q')):
                    vx, vy, vz, yaw_rate = 0.0, 0.0, 0.0, -0.5
                elif key in (ord('e'), ord('E')):
                    vx, vy, vz, yaw_rate = 0.0, 0.0, 0.0, 0.5
                elif key in (ord('x'), ord('X')):
                    vx, vy, vz, yaw_rate = 0.0, 0.0, 0.0, 0.0
                elif key == ord('1'):
                    vision_node.set_gimbal(0.0)   # Góc ngang 0°
                elif key == ord('2'):
                    vision_node.set_gimbal(0.60)  # Chúc 35°
                elif key == ord('3'):
                    vision_node.set_gimbal(1.20)  # Chúc 70°

                flight_mgr.set_velocity(vx, vy, vz, yaw_rate)

    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        vision_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        cleanup()


if __name__ == '__main__':
    main()
