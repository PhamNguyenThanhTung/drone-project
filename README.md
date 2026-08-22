# 🚁 Autonomous Drone Vision Tracking System

Hệ thống điều khiển Drone tự động nhận diện và bám đuổi người đi bộ theo thời gian thực (Vision-based Person Following Drone) kết hợp **ArduPilot SITL**, **Gazebo Harmonic**, **ROS 2 Humble**, **YOLOv8** và **ByteTrack**.

---

## 📋 Mục Lục
1. [Giới Thiệu & Tính Năng](#-giới-thiệu--tính-năng)
2. [Cấu Trúc Thư Mục & Vai Trò Từng File](#-cấu-trúc-thư-mục--vai-trò-từng-file)
3. [Yêu Cầu Hệ Thống & Hướng Dẫn Cài Đặt](#-yêu-cầu-hệ-thống--hướng-dẫn-cài-đặt)
4. [Hướng Dẫn Chạy Dự Án](#-hướng-dẫn-chạy-dự-án)
5. [Nguyên Lý Hoạt Động Cốt Lõi](#-nguyên-lý-hoạt-động-cốt-lõi)

---

## 🌟 Giới Thiệu & Tính Năng

- **AI Vision Detection & Tracking**: Sử dụng mô hình **YOLOv8s** và **ByteTrack** để nhận diện và theo dõi đối tượng người đi bộ theo thời gian thực với độ chính xác cao.
- **50% Safe Zone Deadband**: Giữ tâm mục tiêu trong vùng an toàn 50% khung hình giúp Drone bay êm ái, loại bỏ hiện tượng giật lắc và dao động liên tục.
- **Fixed Gimbal / Airframe Tracking**: Khóa góc camera chúc $37^\circ$ (`0.65 rad`), Drone tự động điều hướng xoay thân (`yaw_rate`) và tiến/lùi (`vx`, `vy`) bám sát người.
- **Thuật toán Hình học 3D Pinhole & Vượt Tán Cây (Tree Clearance)**:
  - Tính toán khoảng cách mặt đất thực tế $d_x, d_y$ từ camera tới người dựa trên ma trận quang học Pinhole và độ cao bay.
  - Tự động cộng thêm khoảng đệm an toàn $+1.8\text{m}$ (`tree_clearance_margin`) khi người rẽ/đi khuất sau cây để Drone bay thẳng vượt qua tán lá trước khi xoay hướng tại khúc cua.
- **Hiển Thị Vùng Thu Camera (FOV Ray Frustum)**: 4 tia laser phát sáng màu xanh Cyan định vị vùng nhìn của camera xuống mặt đất trong cửa sổ 3D Gazebo (ẩn trên luồng camera thực bằng `visibility_mask`).
- **Live Camera HUD**: Cửa sổ trực quan hiển thị hình ảnh từ Camera, khung Safe Zone, Bounding Box đối tượng, tâm ngắm và thông số bay thời gian thực.

---

## 📁 Cấu Trúc Thư Mục & Vai Trò Từng File

```text
drone-project/
├── README.md                      # Tài liệu hướng dẫn cài đặt và sử dụng dự án
├── .gitignore                     # Cấu hình bỏ qua file build, cache, log khi push git
├── start_stack.sh                 # Script Bash khởi động toàn bộ pipeline (SITL, Gazebo, ROS 2, YOLO, Control, HUD)
├── vehicle_yaw_search.py          # Node điều khiển Drone: bay bám đuổi, tính khoảng cách 3D và vượt tán cây
├── live_camera_hud.py             # Node hiển thị cửa sổ Camera HUD trực tiếp với Safe Zone 50%
├── flight_teleop.py               # Tiện ích điều khiển Drone thủ công qua bàn phím (WASD / MAVLink)
│
├── ros2_ws/                       # Không gian làm việc ROS 2
│   └── src/
│       └── vision_tracking/       # Package ROS 2 xử lý thị giác máy tính
│           ├── package.xml        # Định nghĩa thông tin package & dependencies ROS 2
│           ├── setup.py           # File cài đặt và đăng ký executable node
│           └── vision_tracking/
│               ├── yolo_detector_node.py   # Node YOLOv8 + ByteTrack phát hiện người và tính sai số bám đuổi
│               ├── gimbal_controller_node.py # Node điều khiển Gimbal độc lập (khi dùng chế độ Gimbal Search)
│               └── tracking_eval.py        # Module đánh giá độ trễ và hiệu năng bám đuổi
│
├── gazebo/                        # Tài nguyên mô phỏng Gazebo
│   ├── worlds/
│   │   ├── person_tracking_no_trees.sdf # World công viên thoáng không có cây (Mặc định)
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

## 🚀 Hướng Dẫn Chạy Dự Án

Chỉ cần chạy **1 lệnh duy nhất** để khởi động toàn bộ hệ thống mô phỏng và bám đuổi:

### 1. Chạy trong môi trường Open Field (Không có cây - Mặc định):
```bash
cd ~/drone-project
chmod +x start_stack.sh
./start_stack.sh
```

### 2. Chạy trong môi trường công viên có cây (Kiểm tra thuật toán né/vượt tán cây):
```bash
cd ~/drone-project
WORLD_NAME=person_tracking_path ./start_stack.sh
```

### 3. Điều khiển thủ công bằng bàn phím (Tùy chọn):
Nếu muốn tự lái Drone thủ công trong khi camera vẫn tracking:
```bash
python3 ~/drone-project/flight_teleop.py
```
*(Các phím: `W/S` - Tiến/Lùi, `A/D` - Trái/Phải, `Up/Down` - Lên/Xuống, `Left/Right` - Xoay Yaw)*

---

## 🧠 Nguyên Lý Hoạt Động Cốt Lõi

1. **Khởi động & Cất cánh tự động**:
   - `start_stack.sh` khởi tạo ArduPilot SITL và Gazebo Harmonic.
   - Drone tự động chuyển sang chế độ `GUIDED`, Arm động cơ và cất cánh lên độ cao $3.8\text{m}$.
2. **Nhận diện & Bám đuổi (Tracking Mode)**:
   - Camera trên Drone truyền hình ảnh về topic ROS 2 `/tracking/image_raw`.
   - `yolo_detector_node` phát hiện người đi bộ, chạy ByteTrack và tính sai số pixel $(e_x, e_y)$ so với tâm khung hình.
   - `vehicle_yaw_search.py` áp dụng vùng đệm 50% Safe Zone để Drone bay êm ái, bám theo tốc độ đi bộ của người.
3. **Tính toán vị trí 3D & Vượt tán cây (Turn & Obstacle Clearance)**:
   - Khoảng cách mặt đất thực tế được tính toán theo thời gian thực:
     $$d_x = \frac{h_{\text{rel}}}{\tan(\theta_{\text{pitch}} + \alpha_y)}, \quad d_y = d_x \cdot \frac{e_x}{f_x}$$
   - Khi người đi khuất sau khúc cua/tán cây, Drone chuyển sang trạng thái `ADVANCING_TO_TURN_POINT`, khóa cứng $\text{yaw\_rate} = 0.0^\circ\text{/s}$ và bay thẳng quãng đường $(d_x + 1.8\text{m})$ để vượt hẳn qua mép tán lá.
   - Khi đã đến vùng thoáng tại điểm rẽ, Drone chuyển sang `ROTATING_AT_TURN_POINT` xoay thân đón đầu người ở góc cua mới.
