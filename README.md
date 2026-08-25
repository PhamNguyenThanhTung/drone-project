# UAV Vision Tracking & Autonomous Follower (PX4 SITL + Gazebo Harmonic)

Hệ thống bám đuổi mục tiêu thông minh thời gian thực (Autonomous Vision Tracking & Teleop) cho Drone Quadcopter sử dụng **PX4 Autopilot SITL**, **Gazebo Harmonic**, **ROS 2 Humble**, **YOLOv8 + ByteTrack**, **OSNet Re-ID**, và kiến trúc **State Machine MotionArbiter**.

---

## 1. Yêu cầu Hệ thống & Cài đặt Môi trường (Installation Guide)

### Yêu cầu nền tảng:
* **Hệ điều hành**: Ubuntu 22.04 LTS (Native hoặc WSL2 trên Windows 10/11)
* **ROS 2**: ROS 2 Humble Desktop (`ros-humble-desktop`)
* **Mô phỏng**: Gazebo Harmonic (`gz-sim8`, `ros-humble-ros-gz-bridge`)
* **Flight Stack**: PX4-Autopilot (`v1.14` / `v1.15`)
* **Python**: Python 3.10+ với PyTorch (GPU CUDA hoặc CPU)

### Các bước cài đặt từ đầu (Setup Steps):

```bash
# 1. Cài đặt các gói phụ thuộc Python
pip3 install --user ultralytics pymavlink mavsdk opencv-python kconfiglib jinja2 "empy<4" jsonschema pyyaml

# 2. Biên dịch workspace ROS 2
cd ~/drone-project/ros2_ws
colcon build --symlink-install
source install/setup.bash

# 3. Tải QGroundControl AppImage (Nếu chưa có)
curl -L -o ~/QGroundControl.AppImage https://github.com/mavlink/qgroundcontrol/releases/download/v5.1.3/QGroundControl-x86_64.AppImage
chmod +x ~/QGroundControl.AppImage
```

---

## 2. Khởi chạy Hệ thống (Single Command Launch)

Chỉ cần chạy một script duy nhất:

```bash
cd ~/drone-project
./start_stack.sh
```

### Các thành phần sẽ tự động mở lên đồng thời:
1. **Gazebo Harmonic 3D Simulation**: Thế giới công viên mô phỏng với người đi bộ và Quadcopter `x500`.
2. **QGroundControl (QGC)**: Tự động kết nối UDP `14550`, hiển thị tọa độ GPS, la bàn, cao độ EKF2 và bản đồ vệ tinh.
3. **Live Camera HUD (OpenCV POV)**: Cửa sổ hiển thị trực quan góc nhìn từ Drone với YOLO Bounding Box, 50% Safe Zone, thanh trạng thái State Machine, **minimap 25 m (Bắc hướng lên)** và **bảng GPS trực tiếp** (LAT/LON/ALT + khoảng cách tới HOME) ở góc trái — kiểu QGroundControl thu nhỏ ngay trên hình camera. Click vào minimap để gửi lệnh GOTO tới vị trí tương ứng.
4. **MotionArbiter Node**: Tự động ARM động cơ, cất cánh lên $3.8\text{ m}$, kích hoạt chế độ **OFFBOARD** và bắt đầu bám đuổi mục tiêu.

> **Về nhãn trên màn hình**: các box chỉ được gán nhãn `PERSON` / `PERSON [LOCK]` — **không hiển thị số track ID** vì ByteTrack cấp ID mới mỗi vài frame khi chạy CPU, số nhảy liên tục khiến tưởng như mất dấu. ID vẫn được quản lý nội bộ: khóa bị mất ID sẽ tự ghép lại với box chồng lên box cũ (IoU) trong 2.5 s.

---

## 3. Bảng Điều khiển Phím & Thao tác Chuột

Tất cả thao tác điều khiển được tích hợp **trên cùng cửa sổ Camera POV**:

| Phím / Thao tác | Chức năng | Chuyển đổi State |
| :--- | :--- | :--- |
| **`W` / `S`** | Bay Tiến / Lùi ($2.0\text{ m/s}$) | $\rightarrow$ `MANUAL` (Can thiệp tay) |
| **`A` / `D`** | Bay Sang Trái / Phải ($2.0\text{ m/s}$) | $\rightarrow$ `MANUAL` (Can thiệp tay) |
| **`R` / `F`** | Bay Lên cao / Hạ xuống ($1.0\text{ m/s}$) | $\rightarrow$ `MANUAL` (Can thiệp tay) |
| **`Q` / `E`** | Xoay mũi Trái / Phải ($\pm 0.45\text{ rad/s}$) | $\rightarrow$ `MANUAL` (Can thiệp tay) |
| **`X`** | Phanh dừng khẩn cấp (Hover tại chỗ) | `MANUAL` (Vận tốc = 0) |
| **Click Chuột trái vào người** | Khóa mục tiêu vừa click (cách chọn chính) | $\rightarrow$ `TRACKING` (Tự động bám) |
| **Phím số `1`, `2`, `3`, `4`...** | Khóa mục tiêu theo ID nội bộ (không hiển thị trên màn hình) | $\rightarrow$ `TRACKING` (Tự động bám) |
| **Phím `0` hoặc `SPACE`** | Hủy khóa mục tiêu (Bay treo tại chỗ) | $\rightarrow$ `STANDBY` (Hover) |

---

## 4. Kiến trúc State Machine (`MotionArbiter`)

```
               [ Click chuột / Phím 1-9 ]
        +----------------------------------------+
        |                                        |
        v                                        |
+---------------+     Phím lái (W/A/S/D...)     +---------------+
|   TRACKING    | --------------------------->  |    MANUAL     |
+---------------+                               +---------------+
  |           ^                                   ^           |
  | (0/SPACE  | (Click / 1-9)                     |           |
  |  Timeout) |                                   |           |
  v           |                                   | (W/A/S/D) |
+---------------+                                 |           |
|    STANDBY    | --------------------------------+           |
+---------------+ --------------------------------------------+
```

* **Zero-Latency Manual Override**: Khi drone đang tự động bay bám mục tiêu (`TRACKING`), ngay khi bạn bấm bất kỳ phím lái nào (`W/A/S/D`), quyền điều khiển sẽ chuyển ngay sang `MANUAL` trong vòng $< 10\text{ ms}$.
* **Failsafe Watchdog**: Nếu mục tiêu bị mất dấu quá $4.0\text{s}$, drone tự động chuyển sang `STANDBY` (Hover an toàn tại chỗ) chứ không tự ý bay mất kiểm soát.
