# PX4 và SITL test guide

Các script trong thư mục này kiểm tra PX4/MAVLink, camera-perception, MotionArbiter và dynamic tracking scenarios.

## Chuẩn bị

Chạy từ repository root:

~~~bash
cd ~/drone-project
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
export PYTHONPATH=.:$PYTHONPATH
~~~

Không thay thế PYTHONPATH bằng dấu chấm duy nhất vì sẽ che các ROS-generated message packages.

Đặt PX4_DIR nếu checkout PX4 không ở vị trí mặc định:

~~~bash
export PX4_DIR=/path/to/PX4-Autopilot
~~~

## Test groups

| Script | Phạm vi |
| --- | --- |
| px4_baseline_test.py | Arm, takeoff, hold, land |
| px4_mavsdk_baseline.py | Baseline qua MAVSDK |
| test_offboard_climb.py | Offboard setpoint và altitude |
| test_mavsdk_axes.py | Kiểm tra axis/frame |
| test_mavlink_live.py | MAVLink endpoint và telemetry |
| test_msg_live.py | Message availability |
| px4_axis_and_statemachine_test.py | Axis và state behavior |
| test_approach_camera.py | Camera → YOLO → tracking/error |
| test_person_turnaround_live.py | Turnaround tracking |
| test_long_path_tracking.py | World đường dài |
| run_isolated_multi_trial.py | Năm dynamic scenarios |
| verify_live_flight.py | Live SITL integration checks |

Một số test legacy vẫn gọi motion_arbiter.py ở repository root để giữ nguyên fixture/subprocess contract cũ. Normal stack hiện tại dùng ROS 2 package entry point motion_arbiter_node qua tracking_stack.launch.py. Không dùng kết quả legacy test để mô tả topology runtime hiện tại nếu chưa đối chiếu.

## Multi-trial regression

~~~bash
python3 tests/px4/run_isolated_multi_trial.py
~~~

Phân tích lại CSV hiện có:

~~~bash
python3 tests/px4/run_isolated_multi_trial.py --analyze-only
~~~

Output chính:

- logs/multi_trial_summary.json
- logs/raw_trial_*.csv
- logs/tracking_diagnostics_trial_*.jsonl

Summary chia metrics thành perception, control và safety. Một scenario có thể safety PASS nhưng perception retention thấp; luôn đọc cả ba lớp.

## Command latency

~~~bash
python3 scripts/command_latency_micro_test.py
~~~

Path đo:

~~~text
HUD publish
  -> MotionArbiter callback
  -> control decision
  -> MAVLink send
  -> simulated vehicle velocity change
~~~

Không dùng control-loop frequency để suy ra command latency.

## Runtime checklist

Trước mỗi live test:

~~~bash
pgrep -af "px4|gz sim|parameter_bridge|yolo_detector|motion_arbiter"
~~~

Trong test:

~~~bash
ros2 topic hz /camera/image_raw
ros2 topic hz /tracking/error
ros2 topic echo /tracking/control_health
~~~

Sau test, xác nhận process do test tạo đã dừng.

## Test interpretation

- Wall time và simulation time không giống nhau khi Gazebo RTF giảm.
- Camera frame là dữ liệu perishable; backlog cũ không phải valid tracking evidence.
- Offboard loss, watchdog stall và altitude drop là safety/control metrics.
- Tracking retention, error và target loss là perception metrics.
- Một PASS phải chỉ rõ metric/gate nào đã pass.

## Các lưu ý PX4

- Takeoff/arming cần GPS/EKF readiness phù hợp với airframe.
- PX4 chỉ nhận Offboard ổn định khi setpoint được stream liên tục.
- Project-local worlds nên được Gazebo server chạy trước; PX4 attach vào world đang tồn tại.
- Chạy trên máy rảnh hoặc ghi CPU/RTF cùng kết quả vì contention ảnh hưởng wall-clock tests.
- Không chỉnh PX4 source hoặc failsafe chỉ để làm test pass.
