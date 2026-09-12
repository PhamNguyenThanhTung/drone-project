# Báo cáo kỹ thuật Autonomous Person Tracking Drone

**Stack:** PX4 SITL, Gazebo Harmonic, ROS 2 Humble, YOLOv8n, ByteTrack, OpenCV, PyTorch, MAVLink
**Phạm vi:** mô phỏng và validation SITL
**Trạng thái real flight:** chưa xác nhận

## 1. Bài toán

Drone phải quan sát một người trong camera, duy trì đúng danh tính qua nhiều frame, xác định người lệch bao nhiêu so với tâm ảnh và điều khiển để target còn trong field of view. Bài toán gồm bốn vòng liên kết:

1. **Perception:** từ ảnh thành bounding box người.
2. **Tracking:** giữ target xuyên frame và xử lý track-ID thay đổi.
3. **Decision:** từ error/area thành velocity command.
4. **Flight execution:** PX4 nhận setpoint, ổn định vehicle và cập nhật pose trong Gazebo.

Operator có thể khóa mục tiêu, clear target, điều khiển thủ công, takeoff, land hoặc gửi GOTO từ minimap.

## 2. Kiến trúc hiện tại

~~~mermaid
flowchart LR
  GZ[Gazebo world camera actor] --> BR[ros_gz_bridge]
  BR -->|/camera/image_raw| YOLO[YOLOv8n and ByteTrack]
  YOLO -->|/tracking/error| ARB[MotionArbiter]
  YOLO -->|/tracking/debug_image| HUD[Live Camera HUD]
  HUD -->|target teleop action goto| ARB
  ARB -->|MAVLink UDP 14540| PX4[PX4 SITL]
  PX4 -->|telemetry| ARB
  PX4 --> GZ
  ARB -->|state GPS health| HUD
~~~

ROS 2 launch file hiện tại là ros2_ws/src/vision_tracking/launch/tracking_stack.launch.py. Nó khởi chạy:

- ros_gz_bridge nếu use_bridge=true.
- sim_realism_node nếu use_realism=true.
- yolo_detector_node.
- motion_arbiter_node.
- live_camera_hud_node nếu show_hud=true.

## 3. Startup lifecycle

start_stack.sh là orchestration entry point:

1. Xác định PROJECT_DIR và PX4_DIR.
2. Source ROS 2 Humble.
3. Build package vision_tracking bằng symlink install.
4. Đặt Gazebo resource, server config và plugin paths.
5. Dọn process cũ của stack.
6. Chạy Gazebo server với world được chọn.
7. Chờ Gazebo xuất hiện /clock.
8. Chạy PX4 SITL target gz_x500_flow.
9. Chờ heartbeat/MAVLink endpoint.
10. Cấu hình telemetry UDP 14550 cho QGroundControl khi phù hợp.
11. Chạy ROS 2 launch với device, HUD, realism và takeoff override.
12. Chờ camera, detector và arbiter health topics.
13. Giữ shell sống và cleanup process tree khi Ctrl-C.

Startup mặc định:

| Setting | Giá trị |
| --- | --- |
| World | person_tracking_path |
| Vehicle | x500_flow |
| Vehicle name | x500_0 |
| Takeoff altitude | 3.8 m |
| YOLO device | cuda:0 qua start_stack.sh |
| HUD | bật |
| Realism | tắt |
| QGroundControl auto launch | tắt |

## 4. Gazebo simulation

### 4.1 Camera

Model X500 cục bộ khai báo:

- Resolution: 640 x 480.
- Update rate: 30 Hz.
- Topic: /camera/image_raw.
- Horizontal field of view: 2.0 rad.
- Camera gắn cứng, pitch khoảng 0.65 rad xuống dưới.

### 4.2 World

person_tracking_path.sdf có:

- Physics ODE, max step 0.004 s, real-time update 250 Hz, target RTF 1.0.
- Ground plane 140 x 140 m.
- Sensor plugins cho camera, IMU, air pressure và NavSat.
- X500 spawn tại origin.
- walking_person actor với walkway nhiều đoạn và lượt về.

Một số đoạn đi 10 m trong 9 s, tương đương khoảng 1.11 m/s. Các góc rẽ 90 độ và turnaround 180 độ tạo tình huống thay đổi error_x, error_y, area và khả năng mất target.

World khác:

| World | Mục đích |
| --- | --- |
| person_tracking_approach | Target tiến/lùi trước camera |
| person_tracking_no_trees | Cô lập camera/detection, giảm che khuất |
| person_tracking_long_path | Hành trình dài với nhiều góc rẽ |

## 5. Camera-to-detection pipeline

### 5.1 ROS bridge

Bridge ánh xạ Gazebo Image thành sensor_msgs/Image:

~~~text
/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image
~~~

### 5.2 QoS

Detector dùng KEEP_LAST, depth 1, BEST_EFFORT cho camera. Ảnh là dữ liệu dễ hết hạn: nếu inference bận, frame mới quan trọng hơn backlog cũ.

### 5.3 YOLO inference

Các parameter hiện tại:

| Parameter | Giá trị |
| --- | ---: |
| model_path | yolov8n.pt |
| tracker | bytetrack.yaml |
| classes | [0] |
| conf | 0.45 |
| iou | 0.45 |
| infer_native | true |
| error coordinate width/height | 416 x 416 |
| min_box_area_ratio | 0.0003 |
| max_box_area_ratio | 0.85 |
| min_aspect_ratio | 0.70 |
| max_aspect_ratio | 4.80 |
| target_timeout | 4.0 s |
| lock_reacquire_s | 5.0 s |
| reacquire_min_iou | 0.15 |

Khi infer_native=true, detector chạy trên frame native. Sau inference, box được scale về 416 x 416 để tạo message contract ổn định cho controller.

## 6. Detection, tracking và target policy

**Detection** tạo bounding boxes, class và confidence cho từng frame.
**ByteTrack** liên kết box giữa các frame bằng track ID.
**Target policy** quyết định ID nào thật sự là mục tiêu điều khiển.

Policy hiện tại:

1. Nếu operator clear target, không auto-track.
2. Nếu MotionArbiter báo MANUAL, suppress automatic reacquisition.
3. Nếu có manual_target_id, ưu tiên box cùng ID.
4. Nếu ID mất, thử re-acquire trong 5 s bằng hybrid score:
   - 60% IoU.
   - 40% centroid proximity.
5. Nếu auto mode chưa có target, chọn candidate có area x confidence lớn nhất.
6. Nếu target hiện tại còn tồn tại, giữ ID thay vì chọn lại mỗi frame.

Bounding box được làm mượt bằng EMA alpha 0.60.

## 7. Perception-control contract

/tracking/error dùng geometry_msgs/Point:

~~~text
msg.x = target_center_x - 208
msg.y = target_center_y - 208
msg.z = box_width * box_height
~~~

- x: horizontal pixel error.
- y: vertical pixel error.
- z: bbox area trong coordinate space 416 x 416.

Area native được scale theo cả trục X và Y trước khi publish. Controller dùng target_area_min=6000 và target_area_max=13000. Đây là proxy khoảng cách, không phải distance sensor.

## 8. MotionArbiter

### 8.1 Inputs và outputs

| Interface | Type | Direction |
| --- | --- | --- |
| /tracking/error | geometry_msgs/Point | input |
| /teleop/cmd_vel | geometry_msgs/Twist | input |
| /tracking/select_target | std_msgs/Int32 | input |
| /teleop/flight_action | std_msgs/String | input |
| /tracking/goto_gps | geometry_msgs/Point | input |
| /tracking/motion_state | std_msgs/String | output |
| /tracking/gps | sensor_msgs/NavSatFix | output |
| /tracking/control_health | std_msgs/String | output |
| MAVLink UDP 14540 | pymavlink | bidirectional |

### 8.2 Top-level states

| State | Ý nghĩa |
| --- | --- |
| STANDBY | Hover/ready, không điều khiển ngang tự động |
| TRACKING | Visual servo theo target |
| MANUAL | Operator Twist có quyền ưu tiên |
| MANUAL_GOTO | Position setpoint từ minimap |

SEARCH và RECOVER không phải top-level state. Chúng là tracking substate như RECOVERING_YAW_HEADING hoặc SEARCHING_HOLD.

### 8.3 Control timer

Control period là 0.10 s. Timer được tạo bằng ClockType.STEADY_TIME. Mỗi tick:

1. Cập nhật GPS/vehicle state cache.
2. Đồng bộ is_airborne với heartbeat armed state.
3. Kiểm tra target age và state timeout.
4. Chọn command theo state.
5. Gửi velocity hoặc global position setpoint.
6. Publish health và diagnostic data.

### 8.4 Tracking control

| Điều kiện | Hành vi |
| --- | --- |
| Target trong deadband và area band | SAFE_ZONE_HOVER |
| Horizontal error lớn | Yaw + lateral correction |
| Target cao trong frame | ADVANCING |
| Box nhỏ hơn 6000 | ADVANCING_CLOSE_IN, tối đa 0.25 m/s |
| Target thấp/box lớn | BACKING_SMOOTH |
| Mất target phía dưới | BACKING_UP_TO_RECOVER |
| Mất lateral nhưng còn turn-point estimate | ADVANCING_TO_TURN_POINT |
| Sau turn point, còn trong lost window | RECOVERING_YAW_HEADING |
| Hết recovery | SEARCHING_HOLD |

Slew limiter giới hạn thay đổi vx, vy, vz và yaw rate để giảm jerk.

## 9. MAVLink và PX4

MotionArbiter không điều khiển motor. Nó gửi set_position_target_local_ned:

- Frame: MAV_FRAME_LOCAL_NED.
- Type mask: 0x05C3.
- Z position active để giữ baseline altitude.
- Velocity X/Y/Z active.
- Yaw-rate active.

Body-relative XY được xoay sang local NED bằng current yaw. Controller đổi dấu vertical velocity từ FLU sang NED.

MANUAL_GOTO dùng MAV_FRAME_GLOBAL_RELATIVE_ALT_INT và dừng khi sai số ngang còn tối đa 1.5 m.

PX4 thực hiện estimator, velocity/position loop, attitude/rate control và actuator mixing. Vì Offboard yêu cầu setpoint stream liên tục, MotionArbiter dispatch ở 10 Hz và theo dõi max gap/watchdog.

## 10. HUD và manual authority

HUD hiển thị:

- Camera debug image và bounding boxes.
- Tracking safe zone.
- FPS.
- Motion state.
- PX4 flight status.
- GPS, altitude và home distance.
- Minimap bán kính 25 m.

Keyboard:

| Key | Command |
| --- | --- |
| W/S | vx +2.0 / -2.0 m/s |
| A/D | vy -2.0 / +2.0 m/s |
| R/F | vertical command -1.0 / +1.0 theo message convention |
| Q/E | yaw rate -1.10 / +1.10 rad/s |
| X | zero velocity |
| TAB/T | TAKEOFF |
| P | LAND |
| 1-9 hoặc click | target lock |
| 0/Space | clear target |

MANUAL authority áp dụng ngay khi command khác 0 đến MotionArbiter. teleop_timeout 0.5 s là thời gian command còn hiệu lực, không phải delay trước khi áp dụng.

## 11. Realism mode

sim_realism_node có thể:

- Trễ camera.
- Drop frame.
- Motion blur.
- IMU noise, bias và drift.
- GPS noise/dropout.
- Barometer noise.

Normal profile đặt các effect về 0. Khi bật SIM_REALISM=1, detector đổi input sang /simulation/camera/image.

## 12. Validation hiện tại

| Metric | Evidence |
| --- | ---: |
| Control period cấu hình | 0.100 s |
| Max timer gap sau steady-clock update | khoảng 0.113 s |
| Offboard loss trong 5 dynamic trials | 0 |
| Watchdog stalls trong 5 dynamic trials | 0 |
| Safety/control scenario completion | 5/5 |
| Command total P50 | 137.6 ms |
| Command total P95 | 515.8 ms |

multi_trial_summary.json ghi perception retention khác nhau giữa scenario. Hai trial có retention rất thấp nhưng vẫn vượt safety gate; vì vậy 5/5 không có nghĩa perception hoàn hảo.

## 13. Cách triển khai một thay đổi

### Thay detector

1. Chỉnh config/yolo_detector.yaml.
2. Build workspace.
3. Chạy world no_trees trước.
4. Đo image Hz, inference output và target retention.
5. Chạy lại dynamic trials.

### Thay controller

1. Giữ nguyên error coordinate contract.
2. Chỉ đổi một nhóm parameter hoặc một branch control.
3. Ghi lại control gap, offboard state, altitude và command.
4. So sánh cùng world/seed.
5. Không tune để che stale image hoặc topic delay.

### Thêm node

1. Thêm module vào vision_tracking package.
2. Khai báo console_script.
3. Thêm parameters YAML.
4. Thêm Node action trong launch.
5. Cập nhật topic table và readiness test.

## 14. Giới hạn

- SITL only.
- Bbox area là distance proxy 2D.
- Camera geometry cố định.
- Chưa có calibrated lens/intrinsics cho hardware.
- Chưa có obstacle avoidance tổng quát.
- Chưa có HIL hoặc real-flight data.
- WSL2 resource variability ảnh hưởng throughput.

## 15. Roadmap

1. Camera calibration và depth/range estimation.
2. 3D target localization.
3. Dataset cho occlusion, crossing và outdoor domain shift.
4. Better re-identification.
5. HIL với flight controller thật, props removed.
6. Safety harness, kill switch, tether và geofence.
7. Real-flight validation theo từng gate.

## 16. Kết luận

Hệ thống hiện đã hình thành pipeline camera perception đến PX4 flight control trong SITL, với ROS 2 làm lớp giao tiếp/orchestration. Kết quả hiện tại chứng minh tính tích hợp và stability trong simulation; chưa chứng minh production readiness hoặc safety ngoài đời.
