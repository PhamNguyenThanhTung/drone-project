# Autonomous Person Tracking Drone

Hệ thống mô phỏng drone tự động phát hiện, khóa và bám theo người bằng **PX4 SITL, Gazebo Harmonic, ROS 2 Humble, YOLOv8n, ByteTrack và MAVLink**.

> Phạm vi hiện tại là nghiên cứu và kiểm thử SITL. Chưa có xác nhận bay thật.

## 1. Hệ thống làm gì?

~~~text
Gazebo camera
  -> /camera/image_raw
  -> YOLOv8n: phát hiện person
  -> ByteTrack: duy trì track ID
  -> /tracking/error: sai số tâm và diện tích box
  -> MotionArbiter: state machine + visual servo
  -> MAVLink velocity setpoint
  -> PX4 Offboard
  -> X500 chuyển động
  -> camera frame kế tiếp
~~~

Operator có thể click chọn mục tiêu hoặc dùng bàn phím. Một lệnh W/A/S/D/Q/E/R/F đưa hệ thống vào MANUAL và có ưu tiên trước automatic target reacquisition.

## 2. Thành phần

| Thành phần | Input | Output | Trách nhiệm |
| --- | --- | --- | --- |
| Gazebo Harmonic | World/model SDF | Camera, physics, actor pose | Mô phỏng môi trường và sensor |
| ros_gz_bridge | Gazebo Transport | /camera/image_raw | Chuyển Image sang ROS 2 |
| yolo_detector_node | sensor_msgs/Image | tracking error, debug image | YOLO, ByteTrack, target policy |
| motion_arbiter_node | Error, teleop, target, telemetry | MAVLink, state, health | Visual servo và authority |
| live_camera_hud_node | Debug image, state, GPS | Click, Twist, action, goto | Operator interface |
| sim_realism_node | Sensor topics tùy chọn | Degraded topics | Delay/drop/blur/noise injection |
| PX4 SITL | MAVLink setpoint | Telemetry, actuator response | Estimator, Offboard và failsafe |

Package ROS 2 hiện tại nằm trong **ros2_ws/src/vision_tracking**. setup.py đăng ký bốn entry point: yolo_detector_node, motion_arbiter_node, live_camera_hud_node và sim_realism_node.

## 3. Repository map

~~~text
drone-project/
├── start_stack.sh
├── bao_cao_du_an_day_du.html
├── README.md
├── PROJECT_REPORT.md
├── ros2_ws/src/vision_tracking/
│   ├── launch/tracking_stack.launch.py
│   ├── config/*.yaml
│   └── vision_tracking/
│       ├── yolo_detector_node.py
│       ├── motion_arbiter_node.py
│       ├── live_camera_hud_node.py
│       └── sim_realism_node.py
├── gazebo/models/
├── gazebo/worlds/
├── simulation/
├── scripts/
├── tests/px4/
└── logs/
~~~

Các script motion_arbiter.py và live_camera_hud.py ở root là compatibility entry points còn được một số test legacy tham chiếu. Normal startup hiện tại dùng ROS 2 package nodes.

## 4. Yêu cầu

- Ubuntu 22.04 hoặc WSL2.
- ROS 2 Humble Desktop.
- Gazebo Harmonic và ros_gz_bridge.
- Python 3.10+, pymavlink, ultralytics, torch, OpenCV và cv_bridge.
- PX4-Autopilot checkout; đặt PX4_DIR nếu không nằm tại ../PX4-Autopilot.
- Model yolov8n.pt.
- CUDA tùy chọn. start_stack.sh mặc định kiểm tra và dùng cuda:0; đặt YOLO_DEVICE=cpu để chạy CPU.

## 5. Cài đặt

~~~bash
cd ~/drone-project
./scripts/setup_environment.sh
~~~

Hoặc:

~~~bash
source /opt/ros/humble/setup.bash
pip3 install -r requirements.txt
cd ros2_ws
colcon build --symlink-install --packages-select vision_tracking
source install/setup.bash
~~~

## 6. Khởi chạy

~~~bash
cd ~/drone-project
./start_stack.sh
~~~

Script thực hiện:

1. Source ROS 2 và build workspace.
2. Đặt Gazebo resource/plugin paths.
3. Dọn process cũ thuộc stack.
4. Start Gazebo và chờ /clock.
5. Start PX4 SITL gz_x500_flow và chờ MAVLink.
6. Start ROS 2 launch: bridge, realism tùy chọn, YOLO, MotionArbiter, HUD.
7. Kiểm tra /camera/image_raw, /tracking/error và /tracking/control_health.

Các mode:

~~~bash
HEADLESS=1 SHOW_HUD=0 ./start_stack.sh
WORLD_NAME=person_tracking_no_trees ./start_stack.sh
YOLO_DEVICE=cpu SHOW_HUD=0 ./start_stack.sh
SIM_REALISM=1 ./start_stack.sh
TAKEOFF_ALT=3.8 ./start_stack.sh
~~~

## 7. Workflow của một camera frame

### Camera

Model X500 khai báo camera 640 x 480, 30 Hz. ros_gz_bridge chuyển Gazebo Image sang sensor_msgs/Image trên /camera/image_raw.

### YOLO và ByteTrack

yolo_detector_node.py:

1. Nhận frame với QoS KEEP_LAST depth 1 và BEST_EFFORT.
2. Chuyển sang BGR bằng cv_bridge.
3. Gọi YOLO.track với persist và ByteTrack.
4. Chỉ giữ COCO class 0, confidence mặc định 0.45, IoU 0.45.
5. Lọc area/aspect ratio cho auto selection.
6. Giữ target hiện tại hoặc re-acquire bằng IoU/proximity nếu track ID thay đổi.
7. Làm mượt box bằng EMA.
8. Publish geometry_msgs/Point.

### Error contract

Giá trị được scale về không gian 416 x 416:

~~~text
Point.x = target_center_x - 208
Point.y = target_center_y - 208
Point.z = bbox_width * bbox_height
~~~

x/y là pixel error. z là area proxy cho khoảng cách, không phải depth.

### MotionArbiter

Mỗi 0.10 s, timer steady-clock:

1. Cập nhật PX4 telemetry.
2. Đọc state, vision freshness, error, area và teleop.
3. Chọn STANDBY, TRACKING, MANUAL hoặc MANUAL_GOTO.
4. Tính vx, vy, vz và yaw_rate.
5. Gửi một MAVLink setpoint nếu vehicle armed và airborne.

Trong TRACKING, controller có các substate: SAFE_ZONE_HOVER, ADVANCING, ADVANCING_CLOSE_IN, BACKING_SMOOTH, BACKING_UP_TO_RECOVER, ADVANCING_TO_TURN_POINT, RECOVERING_YAW_HEADING và SEARCHING_HOLD.

## 8. Operator controls

| Input | Hành động |
| --- | --- |
| TAB hoặc T | Arm và takeoff |
| P | Land |
| Click box hoặc 1-9 | Lock target |
| 0 hoặc Space | Clear target, STANDBY |
| W/S | Tiến/lùi |
| A/D | Trái/phải |
| R/F | Lên/xuống |
| Q/E | Yaw trái/phải |
| X | Stop velocity |
| Click minimap | MANUAL_GOTO |

Teleop publish geometry_msgs/Twist lên /teleop/cmd_vel. Non-zero command tạo MANUAL authority; sau teleop_timeout 0.5 s, arbiter về STANDBY.

## 9. Kiểm tra nhanh

~~~bash
ros2 topic list
ros2 topic hz /camera/image_raw
ros2 topic hz /tracking/error
ros2 topic echo /tracking/motion_state
ros2 topic echo /tracking/control_health
pgrep -af "px4|gz sim|parameter_bridge|yolo_detector|motion_arbiter"
~~~

Log chính:

- /tmp/gz_sim.log
- /tmp/px4_sim.log
- /tmp/ros_tracking_stack.log
- logs/tracking_diagnostics.jsonl
- logs/scheduler_trace*.jsonl
- logs/command_latency_trace*.jsonl

## 10. Test và evidence

~~~bash
python3 tests/px4/run_isolated_multi_trial.py
python3 tests/px4/run_isolated_multi_trial.py --analyze-only
python3 scripts/command_latency_micro_test.py
~~~

Snapshot hiện tại:

- Control period: 0.100 s.
- Max control gap sau steady-clock fix: khoảng 0.113 s.
- Offboard loss: 0.
- Watchdog stall: 0.
- Năm scenario SITL hoàn thành safety/control gate: 5/5.
- Command P50: 137.6 ms; P95: 515.8 ms.

Các con số là bằng chứng SITL, không phải guarantee cho real hardware.

## 11. Safety và giới hạn

Không tăng timeout để che stale data hoặc tuning PID trước khi xác nhận đúng layer lỗi. Hệ thống chưa có depth/3D localization, camera calibration cho hardware, collision avoidance hoàn chỉnh, HIL hoặc real-flight validation.

Đọc thêm:

- [Báo cáo HTML](bao_cao_du_an_day_du.html)
- [Tài liệu kỹ thuật](PROJECT_REPORT.md)
- [Simulation realism](simulation/README.md)
- [PX4 tests](tests/px4/README.md)
