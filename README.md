# UAV Vision Tracking & Autonomous Follower

Hệ thống mô phỏng drone tự động phát hiện, khóa và bám theo người trong thời
gian thực. Dự án kết hợp PX4 SITL, Gazebo Harmonic, ROS 2 Humble,
YOLOv8/ByteTrack, MAVLink và một bộ điều khiển state machine có hỗ trợ thao tác
thủ công từ Live Camera HUD.

> [!IMPORTANT]
> Dự án hiện ở giai đoạn nghiên cứu và kiểm thử SITL. Kết quả regression gần
> nhất đạt 3/5 kịch bản. Hệ thống chưa đủ điều kiện để triển khai bay thật nếu
> chưa hoàn thành toàn bộ safety gates trong [PROJECT_REPORT.md](PROJECT_REPORT.md).

## Mục lục

- [Tổng quan](#tổng-quan)
- [Kiến trúc hệ thống](#kiến-trúc-hệ-thống)
- [Cấu trúc repository](#cấu-trúc-repository)
- [Yêu cầu môi trường](#yêu-cầu-môi-trường)
- [Cài đặt](#cài-đặt)
- [Kiểm tra GPU CUDA](#kiểm-tra-gpu-cuda)
- [Khởi chạy](#khởi-chạy)
- [Điều khiển Live HUD](#điều-khiển-live-hud)
- [ROS 2 interfaces](#ros-2-interfaces)
- [Kiểm thử và kết quả](#kiểm-thử-và-kết-quả)
- [Xử lý sự cố](#xử-lý-sự-cố)
- [An toàn và giới hạn](#an-toàn-và-giới-hạn)
- [Tài liệu dự án](#tài-liệu-dự-án)

## Tổng quan

### Mục tiêu

- Phát hiện người bằng YOLOv8n và duy trì ID bằng ByteTrack.
- Chuyển sai số ảnh thành lệnh vận tốc/yaw để drone bám mục tiêu.
- Duy trì độ cao, khoảng cách và hướng nhìn trong các tình huống tiến gần,
  đi xa, rẽ ngang hoặc quay đầu.
- Cho phép người vận hành giành quyền điều khiển tức thời bằng bàn phím.
- Cung cấp môi trường SITL lặp lại được để đo hiệu năng, tái hiện lỗi và kiểm
  tra failsafe trước khi làm việc với phần cứng thật.

### Thành phần chính

| Thành phần | Vai trò |
| --- | --- |
| PX4 SITL | Flight controller, estimator, arming, takeoff, offboard và failsafe |
| Gazebo Harmonic | Mô phỏng vật lý, camera, cảm biến, drone và người đi bộ |
| ROS 2 Humble | Bus giao tiếp giữa camera, detector, HUD và bộ điều khiển |
| YOLOv8n + ByteTrack | Phát hiện người và theo dõi target ID |
| `motion_arbiter.py` | State machine và visual-servo flight control |
| `live_camera_hud.py` | Video, bounding box, GPS/minimap và teleoperation |
| QGroundControl | Giám sát PX4 qua MAVLink UDP 14550 trên Windows |

## Kiến trúc hệ thống

```mermaid
flowchart LR
    GZ[Gazebo camera and sensors] -->|Gazebo Transport| BR[ros_gz_bridge]
    BR -->|/camera/image_raw| YOLO[YOLOv8 + ByteTrack]
    REAL[Simulation realism node] -. optional delay/drop/noise .-> YOLO
    YOLO -->|/tracking/error| ARB[MotionArbiter]
    YOLO -->|/tracking/debug_image| HUD[Live Camera HUD]
    HUD -->|target, teleop, takeoff, land, GPS goto| ARB
    ARB -->|MAVLink offboard setpoints| PX4[PX4 SITL]
    PX4 -->|vehicle telemetry| ARB
    ARB -->|state and GPS| HUD
    PX4 -->|UDP 14550| QGC[QGroundControl]
```

Luồng xử lý chính:

1. Gazebo tạo ảnh camera `640x480` và dữ liệu cảm biến.
2. `ros_gz_bridge` chuyển ảnh sang ROS 2 topic `/camera/image_raw`.
3. YOLO phát hiện lớp `person`, ByteTrack duy trì ID và tính sai số tâm/diện
   tích bounding box.
4. `MotionArbiter` chuyển sai số ảnh thành vận tốc thân drone, yaw rate và
   quyết định chuyển state.
5. PX4 nhận stream setpoint Offboard và điều khiển mô hình `x500_flow`.
6. HUD hiển thị video, trạng thái, GPS, FPS và nhận lệnh từ người vận hành.

## Cấu trúc repository

```text
drone-project/
├── README.md                         # Hướng dẫn sử dụng nhanh
├── PROJECT_REPORT.md                 # Báo cáo dự án đầy đủ
├── start_stack.sh                    # Khởi chạy toàn bộ simulation stack
├── motion_arbiter.py                 # Flight state machine/control
├── live_camera_hud.py                # Live camera HUD và teleoperation
├── requirements.txt                  # Python dependencies
├── yolov8n.pt                        # YOLOv8 nano weights
├── gazebo/
│   ├── models/                       # x500 và pedestrian assets
│   └── worlds/                       # Các kịch bản tracking
├── ros2_ws/src/vision_tracking/
│   └── vision_tracking/
│       ├── yolo_detector_node.py     # Detector/tracker ROS 2 node
│       └── sim_realism_node.py       # Delay/drop/noise injection
├── simulation/                       # Realism, vehicle profile, failure tools
├── tests/px4/                        # SITL, MAVLink và regression tests
├── logs/                             # CSV/JSON/JSONL kết quả thử nghiệm
├── patches/                          # PX4 x500_flow GPS patch
└── scripts/                          # Setup và patch automation
```

## Yêu cầu môi trường

- Ubuntu 22.04 native hoặc WSL2 trên Windows 10/11.
- ROS 2 Humble Desktop.
- Gazebo Harmonic (`gz-sim8`) và `ros-humble-ros-gz-bridge`.
- Python 3.10 trở lên.
- PX4-Autopilot v1.14 đến v1.16.2 tại `../PX4-Autopilot` hoặc đường dẫn được
  chỉ định bởi `PX4_DIR`.
- NVIDIA GPU và CUDA-enabled PyTorch nếu chạy YOLO bằng GPU.
- QGroundControl trên Windows nếu cần theo dõi telemetry/GCS.

## Cài đặt

Clone dự án và chạy setup:

```bash
git clone https://github.com/PhamNguyenThanhTung/drone-project.git
cd drone-project
./scripts/setup_environment.sh
```

Script setup thực hiện:

1. Kiểm tra ROS 2 Humble.
2. Cài Python dependencies từ `requirements.txt`.
3. Áp dụng patch GPS cho PX4 airframe `4021_gz_x500_flow`.
4. Build package ROS 2 `vision_tracking` bằng `colcon`.
5. Kiểm tra hoặc tải model `yolov8n.pt`.

Thiết lập thủ công khi cần:

```bash
source /opt/ros/humble/setup.bash
pip3 install -r requirements.txt
./scripts/apply_px4_patch.sh --apply /path/to/PX4-Autopilot
cd ros2_ws
colcon build --symlink-install --packages-select vision_tracking
source install/setup.bash
```

## Kiểm tra GPU CUDA

`start_stack.sh` ưu tiên `cuda:0` và sẽ dừng thay vì âm thầm fallback sang CPU.
Chạy ba kiểm tra sau trước khi mở stack:

```bash
ls -l /dev/dxg
/usr/lib/wsl/lib/nvidia-smi
python3 -c "import torch; print(torch.__version__); print(torch.cuda.is_available()); print(torch.cuda.get_device_name(0) if torch.cuda.is_available() else 'NO GPU')"
```

Kết quả cần có:

- `/dev/dxg` tồn tại trong WSL2.
- `nvidia-smi` nhìn thấy GPU.
- `torch.cuda.is_available()` trả về `True`.

Nếu WSL báo `GPU access blocked by the operating system`, cập nhật NVIDIA
Windows driver, sau đó chạy trong PowerShell:

```powershell
wsl --update
wsl --shutdown
```

Mở lại Ubuntu và chạy lại ba kiểm tra trên. Không đánh giá FPS GPU bằng
`test_approach_camera.py`, vì test này cố ý đặt `device:=cpu` để cô lập regression.

## Khởi chạy

### Chạy đầy đủ

```bash
./start_stack.sh
```

Mặc định hệ thống dùng:

- World: `person_tracking_path`
- Model: `x500_flow`
- YOLO device: `cuda:0`
- YOLO inference size: `640`
- Takeoff altitude: `3.8 m`
- HUD: bật
- QGroundControl auto-launch: tắt

### Cấu hình thường dùng

```bash
# Tăng tốc inference bằng ảnh 416 và ép CUDA
YOLO_DEVICE=cuda:0 YOLO_IMGSZ=416 ./start_stack.sh

# Chạy headless, không mở Gazebo GUI và HUD
HEADLESS=1 SHOW_HUD=0 ./start_stack.sh

# Bật mô phỏng delay, drop frame và sensor noise
SIM_REALISM=1 ./start_stack.sh

# Chọn world khác
WORLD_NAME=person_tracking_approach ./start_stack.sh

# Chạy CPU có chủ đích
YOLO_DEVICE=cpu YOLO_IMGSZ=416 ./start_stack.sh
```

| Biến môi trường | Mặc định | Ý nghĩa |
| --- | --- | --- |
| `PX4_DIR` | `../PX4-Autopilot` | Đường dẫn PX4 checkout |
| `WORLD_NAME` | `person_tracking_path` | Gazebo world |
| `PX4_SIM_MODEL` | `x500_flow` | PX4/Gazebo model |
| `YOLO_DEVICE` | `cuda:0` | Thiết bị inference |
| `YOLO_IMGSZ` | `640` | Kích thước đầu vào YOLO |
| `YOLO_MAX_FPS` | `0.0` | Giới hạn inference; `0` là không giới hạn |
| `YOLO_CONF` | `0.45` | Confidence threshold |
| `TAKEOFF_ALT` | `3.8` | Độ cao tự động cất cánh |
| `HEADLESS` | `0` | Tắt Gazebo GUI khi bằng `1` |
| `SHOW_HUD` | `1` | Bật debug image và HUD |
| `SIM_REALISM` | `0` | Bật realism/fault injection |
| `LAUNCH_QGC` | `0` | Mở QGC Windows từ WSL khi bằng `1` |
| `COMPANION_CPUSET` | rỗng | Giới hạn CPU affinity để mô phỏng companion computer |

Nhấn `Ctrl-C` tại terminal chạy `start_stack.sh` để dừng toàn bộ process do
script tạo.

## Điều khiển Live HUD

| Phím/thao tác | Chức năng | State kết quả |
| --- | --- | --- |
| `TAB` hoặc `T` | Arm và takeoff | `STANDBY` sau khi đạt độ cao |
| `P` | Land | `STANDBY`/disarmed |
| Click vào người | Khóa target từ bounding box | `TRACKING` |
| `1` đến `9` | Khóa target ID | `TRACKING` |
| `0` hoặc `SPACE` | Hủy target, hover | `STANDBY` |
| `W` / `S` | Tiến / lùi | `MANUAL` |
| `A` / `D` | Trái / phải | `MANUAL` |
| `R` / `F` | Lên / xuống | `MANUAL` |
| `Q` / `E` | Yaw trái / phải | `MANUAL` |
| `X` | Dừng lệnh vận tốc | `MANUAL` |
| Click minimap | Gửi GPS position setpoint | `MANUAL_GOTO` |

## ROS 2 interfaces

| Topic | Kiểu message | Producer | Consumer |
| --- | --- | --- | --- |
| `/camera/image_raw` | `sensor_msgs/Image` | Gazebo bridge | YOLO detector |
| `/simulation/camera/image` | `sensor_msgs/Image` | Realism node | YOLO detector |
| `/tracking/debug_image` | `sensor_msgs/Image` | YOLO detector | Live HUD |
| `/tracking/error` | `geometry_msgs/Point` | YOLO detector | MotionArbiter |
| `/tracking/select_target` | `std_msgs/Int32` | HUD/detector | Detector/arbiter |
| `/tracking/click_point` | `geometry_msgs/Point` | HUD | Detector |
| `/teleop/cmd_vel` | `geometry_msgs/Twist` | HUD | MotionArbiter |
| `/teleop/flight_action` | `std_msgs/String` | HUD | MotionArbiter |
| `/tracking/goto_gps` | `geometry_msgs/Point` | HUD | MotionArbiter |
| `/tracking/motion_state` | `std_msgs/String` | MotionArbiter | HUD |
| `/tracking/gps` | `sensor_msgs/NavSatFix` | MotionArbiter | HUD |

## Kiểm thử và kết quả

### Regression 5 kịch bản

```bash
python3 tests/px4/run_isolated_multi_trial.py
```

Kết quả hiện được lưu trong `logs/multi_trial_summary.json`:

| Trial | Kịch bản | Altitude drop | Tracking retention | Kết quả |
| ---: | --- | ---: | ---: | --- |
| 1 | Nominal 180-degree turn | `0.085 m` | `10.6%` | PASS |
| 2 | Fast 180-degree turn | `0.000 m` | `12.3%` | PASS |
| 3 | Lateral left turn | `0.255 m` | `24.3%` | FAIL |
| 4 | Lateral right turn | `0.000 m` | `32.1%` | PASS |
| 5 | Aggressive close-in | `0.187 m` | `33.3%` | FAIL |

Kết luận hiện tại: altitude hold tương đối ổn định trong phần lớn trial, nhưng
tracking retention còn thấp và hai kịch bản chưa đạt. Không được dùng bảng PASS
như bằng chứng hệ thống đã sẵn sàng bay thật.

### Các test quan trọng khác

```bash
# Baseline arm/takeoff/offboard/land
python3 tests/px4/px4_baseline_test.py

# Kiểm tra camera -> YOLO -> /tracking/error
python3 tests/px4/test_approach_camera.py

# Kiểm tra quay đầu và ghi telemetry
python3 tests/px4/test_person_turnaround_live.py

# Kiểm tra đường dài nhiều góc rẽ
python3 tests/px4/test_long_path_tracking.py

# Tái hiện offboard-loss/autoland/takeoff recovery
python3 test_repro_autoland_and_takeoff.py
```

Đọc thêm hướng dẫn test tại [tests/px4/README.md](tests/px4/README.md).

## Xử lý sự cố

### HUD FPS thấp

FPS trên HUD là tốc độ ảnh đã đi qua camera, Gazebo, bridge, YOLO, debug render
và ROS 2; nó không chỉ là tốc độ inference. Kiểm tra từng đoạn:

```bash
ros2 topic hz /camera/image_raw
ros2 topic hz /tracking/debug_image
tail -f /tmp/yolo.log
```

Trong log YOLO, `mean_latency=0.033s` tương đương xấp xỉ 30 inference/s.
`det_rate` là tỷ lệ frame phát hiện được người, không phải FPS.

### YOLO chạy CPU

- Chạy trực tiếp `yolo_detector_node.py` sẽ dùng mặc định `cpu` nếu không truyền
  `-p device:=cuda:0`.
- `test_approach_camera.py` cố ý ép CPU.
- `start_stack.sh` mặc định ép CUDA và dừng khi CUDA không khả dụng.
- Xác nhận dòng đầu `/tmp/yolo.log` có `device=cuda:0` và tên GPU.

### Simulation chậm hoặc test thất bại ngẫu nhiên

- Dừng các instance `px4`, `gz sim`, detector hoặc HUD còn sót.
- Chạy test trên máy ít tải; wall-clock và simulation time có thể lệch khi CPU
  bị bão hòa.
- Thử `HEADLESS=1 SHOW_HUD=0` để đo control/test độc lập với GUI.
- Không đặt `COMPANION_CPUSET` hoặc `YOLO_MAX_FPS` khi đang đo hiệu năng tối đa.

### QGroundControl không kết nối

- Chạy QGC trực tiếp trên Windows.
- Kiểm tra UDP 14550 và `PX4_GCS_IP`.
- Đọc `/tmp/px4_gcs_mavlink.log` và `/tmp/px4_sim.log`.

## An toàn và giới hạn

> [!WARNING]
> Không dùng trực tiếp cấu hình SITL để bay ngoài trời. Không thử nghiệm với
> cánh quạt gắn trên drone nếu chưa hoàn thành HIL, bench test tháo cánh, dây
> neo/lồng bảo vệ, RC kill switch và geofence.

Các tham số như `COM_OF_LOSS_T`, `NAV_DLL_ACT`, `NAV_RCL_ACT`, tốc độ tiến/lùi
và vehicle dynamics phải được đo và cấu hình lại cho airframe thật. File
`simulation/vehicle_profile.yaml` vẫn chứa các trường `null` cần dữ liệu đo
thực tế.

## Tài liệu dự án

- [PROJECT_REPORT.md](PROJECT_REPORT.md): báo cáo đầy đủ để review/presentation.
- [simulation/README.md](simulation/README.md): realism, failure injection và
  safety ladder.
- [tests/px4/README.md](tests/px4/README.md): hướng dẫn test PX4/MAVLink.
- [logs/multi_trial_summary.json](logs/multi_trial_summary.json): kết quả
  regression gần nhất.
