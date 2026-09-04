# UAV Vision Tracking & Autonomous Follower (PX4 SITL + Gazebo Harmonic)

Hệ thống bám đuổi mục tiêu thông minh thời gian thực (Autonomous Vision Tracking & Teleop) cho Drone Quadcopter sử dụng **PX4 Autopilot SITL**, **Gazebo Harmonic**, **ROS 2 Humble**, **YOLOv8 + ByteTrack**, và kiến trúc **State Machine MotionArbiter**.

---

## 1. Hướng Dẫn Cài Đặt Nhanh (Quickstart & Onboarding Guide)

### Yêu Cầu Nền Tảng:
* **Hệ điều hành**: Ubuntu 22.04 LTS (Native hoặc WSL2 trên Windows 10/11)
* **ROS 2**: ROS 2 Humble Desktop (`ros-humble-desktop`)
* **Mô phỏng**: Gazebo Harmonic (`gz-sim8`, `ros-humble-ros-gz-bridge`)
* **Flight Stack**: PX4-Autopilot (`v1.14` đến `v1.16.2`) tại thư mục `~/PX4-Autopilot` hoặc `../PX4-Autopilot`
* **Python**: Python 3.10+ (GPU CUDA hoặc CPU)

### Cài Đặt Tự Động Chỉ Bằng 1 Lệnh (One-Click Setup):

```bash
# Clone repo và chạy script thiết lập toàn bộ môi trường:
cd ~/drone-project
./scripts/setup_environment.sh
```

Script sẽ tự động:
1. Kiểm tra ROS 2 Humble và cài đặt các thư viện Python từ [`requirements.txt`](requirements.txt).
2. Tự động áp dụng PX4 Airframe GPS Patch ([`patches/4021_gz_x500_flow_gps.patch`](patches/4021_gz_x500_flow_gps.patch)).
3. Biên dịch workspace ROS 2 (`colcon build --symlink-install`).
4. Chuẩn bị model weights YOLOv8n (`yolov8n.pt`).

---

## 2. Khởi Chạy Hệ Thống & Kiểm Thử

### 2.1 Khởi chạy toàn bộ hệ thống mô phỏng:
```bash
./start_stack.sh
```
* **Gazebo Harmonic**: Thế giới công viên mô phỏng với người đi bộ và Quadcopter `x500_flow`.
* **Live Camera HUD**: Góc nhìn camera POV kèm YOLO Bounding Box, 50% Safe Zone, Minimap vệ tinh và HUD GPS trực tiếp.
* **MotionArbiter**: Tự động ARM, leo lên $3.8\text{ m}$, kích hoạt chế độ **OFFBOARD** và bám mục tiêu.

World mặc định là `person_tracking_path`: người đi bộ lặp toàn bộ tuyến đường lát
trong công viên, bao gồm các đoạn rẽ và lượt quay về. Có thể chọn world khác bằng
`WORLD_NAME=<tên_world> ./start_stack.sh`.

QGroundControl (GCS) chạy trực tiếp trên Windows và kết nối tới MAVLink UDP
14550 do PX4 SITL phát ra trong WSL. Mặc định WSL không khởi động QGC; để mở
bản Windows từ script, chỉ định đường dẫn `.exe` đã mount vào WSL:
`LAUNCH_QGC=1 QGC_WINDOWS_PATH=/mnt/c/Path/To/QGroundControl.exe ./start_stack.sh`.
Bạn cũng có thể dùng biến tương thích `QGC_PATH` cho cùng đường dẫn. Khi
`LAUNCH_QGC=0` (mặc định), hãy khởi động QGroundControl thủ công trên Windows.

### 2.2 Chạy bộ kiểm thử hồi quy 5 kịch bản độc lập (Multi-Trial Regression):
```bash
python3 tests/px4/run_isolated_multi_trial.py
```
Tự động chạy và đánh giá 5 kịch bản biến thể (quay đầu 180°, quay đầu nhanh, rẽ trái, rẽ phải, tiến sát 2.2m) và xuất log CSV thô vào thư mục [`logs/`](logs/).

---

## 3. Bảng Điều Khiển Phím & Thao Tác Chuột (Camera POV HUD)

| Phím / Thao tác | Chức năng | Chuyển đổi State |
| :--- | :--- | :--- |
| **`W` / `S`** | Bay Tiến / Lùi ($2.0\text{ m/s}$) | $\rightarrow$ `MANUAL` (Can thiệp tay tức thì) |
| **`A` / `D`** | Bay Sang Trái / Phải ($2.0\text{ m/s}$) | $\rightarrow$ `MANUAL` (Can thiệp tay tức thì) |
| **`R` / `F`** | Bay Lên cao / Hạ xuống ($1.0\text{ m/s}$) | $\rightarrow$ `MANUAL` (Can thiệp tay tức thì) |
| **`Q` / `E`** | Xoay mũi Trái / Phải ($\pm 0.45\text{ rad/s}$) | $\rightarrow$ `MANUAL` (Can thiệp tay tức thì) |
| **`X`** | Phanh khẩn cấp (Hover tại chỗ) | `MANUAL` ($v = 0$) |
| **Click Chuột trái vào người** | Khóa mục tiêu vừa click | $\rightarrow$ `TRACKING` (Tự động bám) |
| **Phím `1` - `9`** | Khóa mục tiêu theo ID | $\rightarrow$ `TRACKING` (Tự động bám) |
| **Phím `0` hoặc `SPACE`** | Hủy khóa mục tiêu (Bay treo tại chỗ) | $\rightarrow$ `STANDBY` (Hover EKF2) |

---

## 4. Chuẩn Bị Bay Thật: Quy Trình Siết Tham Số An Toàn (Flight Safety Ladder)

> [!WARNING]
> **Quy tắc an toàn sống còn**: Các tham số nới lỏng trong SITL (để chịu tải máy tính) **TUYỆT ĐỐI KHÔNG ĐƯỢC DÙNG** khi bay ngoài trời trên phần cứng thật!

Khi hệ thống mô phỏng đã hoàn toàn ổn định, cần thực hiện siết lại các tham số theo bảng dưới đây trước khi cắm pin bay thật:

### 4.1 Bảng so sánh tham số SITL vs. Bay Thật (Safety Tuning Matrix)

| Tham số / Failsafe | Giá trị trong Mô phỏng (SITL) | Giá trị khi Chuẩn bị Bay Thật | Ý nghĩa an toàn |
| :--- | :---: | :---: | :--- |
| `COM_OF_LOSS_T` | `5.0 s` (hấp thụ trễ CPU host) | **`1.0 s`** (hoặc `0.5 s`) | Thời gian tối đa cho phép mất luồng lệnh Offboard trước khi kích hoạt Failsafe (RTL/Land). |
| `NAV_DLL_ACT` | `0` (Bỏ qua data link loss) | **`1` (Hold)** hoặc **`2` (RTL)** | Hành động khi mất kết nối telemetry với Ground Station / Remote Control. |
| `COM_ARM_GCS_CHK` | `0` (Không bắt buộc GCS) | **`1` (Bắt buộc kết nối GCS)** | Đảm bảo phần mềm mặt đất (QGC) luôn giám sát trước khi cho phép ARM. |
| `NAV_RCL_ACT` | `0` (Bỏ qua mất sóng RC) | **`2` (RTL)** | Tự động bay về điểm xuất phát nếu mất sóng tay điều khiển. |
| `max_forward_speed` | `1.8 m/s` | **`1.0 - 1.2 m/s`** | Giới hạn tốc độ tiến tối đa trong các lần bay thực nghiệm ban đầu. |
| `bottom_backup_speed` | `0.65 m/s` | **`0.50 m/s`** | Tốc độ lùi an toàn khi mục tiêu tiến sát camera. |
| `GF_ACTION` | `0` (None) | **`1` (Hold)** hoặc **`2` (RTL)** | Kích hoạt hàng rào địa lý (Geofence) khống chế bán kính và trần bay tối đa. |

### 4.2 Thang Đo Kiểm Thử An Toàn 5 Cấp (5-Gate Validation Ladder)

1. **Gate 1 - SITL Regression**: Vượt qua toàn bộ 5 trial trong [`run_isolated_multi_trial.py`](tests/px4/run_isolated_multi_trial.py) và chạy thử `simulation/realism.yaml` (nhiễu gió, lag, drop frame).
2. **Gate 2 - HIL (Hardware-In-the-Loop)**: Chạy Companion Computer thật kết nối với Flight Controller thật qua UART/USB.
3. **Gate 3 - Propeller-less Bench Test (Tháo toàn bộ cánh quạt)**: Bật nguồn, kiểm tra ARM, chuyển mode Offboard, giả lập ngắt MAVLink và ngắt camera để xem FCU phản ứng đúng failsafe.
4. **Gate 4 - Tethered Net Test (Dây an toàn độc lập)**: Buộc dây neo giới hạn độ cao trong lồng lưới bảo vệ, có công tắc ngắt khẩn cấp (Physical Kill-Switch) trên tay điều khiển RC.
5. **Gate 5 - Geofence Outdoor Flight**: Bay thực địa trong khu vực được cấp phép với Geofence bán kính $30\text{ m}$ và trần bay $5\text{ m}$.
