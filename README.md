# 🚁 Autonomous Drone Vision Tracking System

Hệ thống điều khiển Drone tự động nhận diện, khóa mục tiêu tùy chọn và bám đuổi người đi bộ theo thời gian thực (Interactive Multi-Person Following Drone) kết hợp **ArduPilot SITL**, **Gazebo Harmonic**, **ROS 2 Humble**, **YOLOv8** và **ByteTrack**.

---

## 📋 Mục Lục
1. [Giới Thiệu & Tính Năng Nổi Bật](#-giới-thiệu--tính-năng-nổi-bật)
2. [Cấu Trúc Thư Mục & Vai Trò Từng File](#-cấu-trúc-thư-mục--vai-trò-từng-file)
3. [Yêu Cầu Hệ Thống & Hướng Dẫn Cài Đặt](#-yêu-cầu-hệ-thống--hướng-dẫn-cài-đặt)
4. [Hướng Dẫn Chạy Dự Án & Chọn Mục Tiêu](#-hướng-dẫn-chạy-dự-án--chọn-mục-tiêu)
5. [Nguyên Lý Hoạt Động Cốt Lõi](#-nguyên-lý-hoạt-động-cốt-lõi)

---

## 🌟 Giới Thiệu & Tính Năng Nổi Bật

- **🎯 Multi-Person Target Selection (Khóa mục tiêu người tùy chọn)**:
  - Khi có nhiều người trong khung hình, hệ thống gán nhãn `[ID: 1]`, `[ID: 2]`... cho từng người.
  - **Tương tác trực tiếp qua Live HUD**: Người dùng có thể **Click chuột trực tiếp** vào ô của người muốn bám đuổi, hoặc **bấm phím số `1`, `2`, `3`...** trên bàn phím.
  - **Chế độ Standby / Hover**: Nếu chưa chọn ai (hoặc bấm `0` / `SPACE`), Drone sẽ **đứng yên bay tại chỗ (Hover)**, không bám lung tung.
- **⚡ 60s Standstill & 2 Branching Paths World (`person_tracking_fork.sdf`)**:
  - 2 người đứng yên trong **60 giây đầu** để người dùng quan sát và chọn mục tiêu.
  - Sau 60s, Người 1 rẽ nhánh **Bắc (Trái)**, Người 2 rẽ nhánh **Nam (Phải)**. Drone sẽ bám sát đúng người đã chọn!
- **📐 Thuật toán Hình học 3D Pinhole & Vượt Tán Cây (Tree Clearance)**:
  - Tính toán khoảng cách mặt đất thực tế $d_x, d_y$ từ camera tới người:
    $$d_x = \frac{h_{\text{rel}}}{\tan(\theta_{\text{pitch}} + \alpha_y)}, \quad d_y = d_x \cdot \frac{e_x}{f_x}$$
  - Tự động cộng thêm khoảng đệm an toàn $+1.8\text{m}$ (`tree_clearance_margin`) khi người rẽ/đi khuất sau cây để Drone bay thẳng vượt qua tán lá trước khi xoay hướng tại khúc cua.
- **🛡️ 50% Safe Zone Deadband**: Giữ tâm mục tiêu trong vùng an toàn 50% khung hình giúp Drone bay êm ái, loại bỏ hoàn toàn hiện tượng rung lắc.
- **✨ Hiển Thị Vùng Thu Camera (FOV Ray Frustum)**: 4 tia laser phát sáng màu xanh Cyan định vị vùng nhìn của camera xuống mặt đất trong cửa sổ 3D Gazebo (tự động ẩn trên camera thực tế bằng `visibility_mask`).

---

## 📁 Cấu Trúc Thư Mục & Vai Trò Từng File

```text
drone-project/
├── README.md                      # Tài liệu hướng dẫn cài đặt, sử dụng và nguyên lý
├── .gitignore                     # Cấu hình bỏ qua file build, cache, log khi push git
├── start_stack.sh                 # Script Bash 1-Click khởi động toàn bộ pipeline
├── vehicle_yaw_search.py          # Node điều khiển Drone: bay bám đuổi, tính khoảng cách 3D và vượt tán cây
├── live_camera_hud.py             # Node hiển thị Live Camera HUD (Click chuột / Phím chọn mục tiêu)
├── flight_teleop.py               # Tiện ích điều khiển Drone thủ công qua bàn phím (WASD / MAVLink)
│
├── ros2_ws/                       # Không gian làm việc ROS 2
│   └── src/
│       └── vision_tracking/       # Package ROS 2 xử lý thị giác máy tính
│           ├── package.xml        # Định nghĩa thông tin package & dependencies ROS 2
│           ├── setup.py           # File cài đặt và đăng ký executable node
│           └── vision_tracking/
│               ├── yolo_detector_node.py   # Node YOLOv8 + ByteTrack: phát hiện, gán ID và chọn mục tiêu
│               ├── gimbal_controller_node.py # Node điều khiển Gimbal độc lập
│               └── tracking_eval.py        # Module đánh giá độ trễ và hiệu năng bám đuổi
│
├── gazebo/                        # Tài nguyên mô phỏng Gazebo
│   ├── worlds/
│   │   ├── person_tracking_fork.sdf     # World ngã ba 2 nhánh: 2 người đứng 60s rồi rẽ 2 hướng (Mặc định)
│   │   ├── person_tracking_no_trees.sdf # World công viên thoáng 1 người không có cây
│   │   └── person_tracking_path.sdf     # World công viên có hàng cây sồi/thông để test vượt tán cây
│   ├── models/
│   │   ├── gimbal_small_3d/       # Model Gimbal 3D tích hợp Camera và 4 tia laser FOV Frustum
│   │   └── iris_with_gimbal/      # Model Drone Quadcopter Iris gắn kèm Gimbal 3D
│   └── config/
│       └── gazebo-iris-gimbal.parm # File tham số ArduPilot Copter cho mô phỏng Gazebo
│
└── docs/
    ├── setup.md                   # Hướng dẫn chi tiết thiết lập môi trường từ đầu
    ├── part2.md                   # Tài liệu thiết kế hệ thống theo dõi mục tiêu
    ├── PHASE2_5_PLAN.md           # Kế hoạch phát triển các pha điều khiển
    └── PHASE2_ACCEPTANCE.md       # Tiêu chuẩn nghiệm thu chức năng
```

---

## 💻 Yêu Cầu Hệ Thống & Hướng Dẫn Cài Đặt

### 1. Yêu cầu phần mềm & phần cứng
- **Hệ điều hành**: Ubuntu 22.04 LTS (hoặc WSL2 Ubuntu 22.04 trên Windows 11).
- **GPU**: NVIDIA GPU hỗ trợ CUDA (khuyến nghị để YOLOv8 inference đạt >30 FPS).
- **ROS 2**: ROS 2 Humble Hawksbill.
- **Gazebo**: Gazebo Harmonic (GZ Sim 8).
- **ArduPilot**: ArduPilot SITL Copter-4.5+ kèm plugin `ardupilot_gazebo`.

---

### 2. Cài đặt các gói phụ thuộc (Dependencies)

#### A. Cài đặt ROS 2 Humble & ROS-Gazebo Bridge:
```bash
sudo apt update && sudo apt install -y \
  ros-humble-desktop \
  ros-humble-ros-gz \
  ros-humble-ros-gz-bridge \
  ros-humble-cv-bridge \
  ros-humble-image-transport \
  python3-colcon-common-extensions
```

#### B. Cài đặt các thư viện Python:
```bash
pip3 install --upgrade pip
pip3 install ultralytics torch torchvision opencv-python pymavlink scipy
```

#### C. Cài đặt ArduPilot SITL:
```bash
cd ~
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git
cd ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
./waf configure --board sitl
./waf copter
```

#### D. Cài đặt Plugin ardupilot_gazebo:
```bash
cd ~
git clone https://github.com/ArduPilot/ardupilot_gazebo.git
cd ardupilot_gazebo
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j$(nproc)
```

#### E. Tải mô hình YOLOv8:
```bash
mkdir -p ~/phase2_ws/models
cd ~/phase2_ws/models
wget https://github.com/ultralytics/assets/releases/download/v8.2.0/yolov8s.pt
```

---

### 3. Build ROS 2 Workspace

```bash
cd ~/drone-project/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

## 🚀 Hướng Dẫn Chạy Dự Án & Chọn Mục Tiêu

### 1. Khởi động hệ thống (Chế độ 2 người rẽ 2 hướng sau 60s - Mặc định):
```bash
cd ~/drone-project
chmod +x start_stack.sh
./start_stack.sh
```

### 2. Cách chọn mục tiêu trong cửa sổ Live Camera HUD:
- **Cách 1 (Click chuột)**: Click chuột trái trực tiếp vào ô người muốn theo dõi trên cửa sổ Camera HUD.
- **Cách 2 (Phím số)**: Nhấn phím số `1` để khóa Người 1 (đi nhánh Bắc), nhấn `2` để khóa Người 2 (đi nhánh Nam).
- **Hủy chọn / Đứng yên (Hover)**: Nhấn phím `0` hoặc phím cách `SPACE`.

### 3. Chạy các môi trường khác:
- **World công viên có cây (Né / Vượt tán cây)**:
  ```bash
  WORLD_NAME=person_tracking_path ./start_stack.sh
  ```
- **World công viên thoáng 1 người (Không cây)**:
  ```bash
  WORLD_NAME=person_tracking_no_trees ./start_stack.sh
  ```

---

## 🧠 Nguyên Lý Hoạt Động Cốt Lõi

1. **Khởi động & Cất cánh tự động**:
   - `start_stack.sh` khởi tạo ArduPilot SITL và Gazebo Harmonic.
   - Drone tự động chuyển sang chế độ `GUIDED`, Arm động cơ và cất cánh lên độ cao $3.8\text{m}$.
2. **Liệt kê Candidate & Chờ Người Dùng Chọn**:
   - `yolo_detector_node` phát hiện tất cả người trong khung hình, hiển thị khung màu xanh Cyan `[ID: 1]`, `[ID: 2]`.
   - Nếu chưa chọn ai, Drone giữ trạng thái `STANDBY / HOVER` bay tại chỗ an toàn.
3. **Khóa Mục Tiêu & Bám Đuổi (Target Locking & Tracking)**:
   - Khi người dùng click chọn ID $K$, mục tiêu chuyển sang khung màu **Xanh Lá `LOCKED ID: K`**.
   - `vehicle_yaw_search.py` nhận sai số của đúng đối tượng $K$ và điều khiển Drone bám sát theo người đó khi người bắt đầu di chuyển sau 60s.
