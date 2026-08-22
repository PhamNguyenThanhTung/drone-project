# Setup ArduPilot + Gazebo Harmonic + ROS 2 + Camera/Gimbal (WSL2 Ubuntu 22.04)

Mục tiêu cuối: drone `iris_with_gimbal` bay được trong Gazebo Sim (Harmonic) qua ArduCopter SITL, camera gắn trên gimbal publish ảnh ra topic, sẵn sàng để nối YOLO detect + tracking người.

Môi trường giả định: **WSL2, Ubuntu 22.04.5 LTS, mới cài sạch** (thư mục home chỉ có `.bashrc`, `.profile`, `.cache`, `.bash_logout`). Nếu môi trường khác, dừng lại và xác nhận trước khi chạy.

Toàn bộ lệnh chạy trong 1 user thường (không phải root), có quyền `sudo`.

---

## Bước 0 — Fix môi trường WSL2 (bắt buộc, né lỗi mạng/GPU đã biết)

```bash
sudo apt update && sudo apt upgrade -y

# DNS ổn định — né lỗi "Temporary failure in name resolution"
sudo rm -f /etc/resolv.conf
sudo bash -c 'echo "nameserver 8.8.8.8" > /etc/resolv.conf'
sudo bash -c 'echo "nameserver 1.1.1.1" >> /etc/resolv.conf'
sudo bash -c 'printf "[network]\ngenerateResolvConf = false\n" >> /etc/wsl.conf'
```

**Sau lệnh trên, PHẢI restart WSL để áp dụng:** mở PowerShell/CMD bên **Windows** (không phải trong Ubuntu), chạy:
```powershell
wsl --shutdown
```
Đợi 5 giây, mở lại Ubuntu, rồi tiếp tục:

```bash
# Git buffer lớn + MTU — né lỗi clone bị đứt/timeout khi tải file lớn qua GitHub
git config --global http.postBuffer 524288000
sudo ip link set dev eth0 mtu 1400
# Lưu ý: lệnh ip link chỉ có hiệu lực trong phiên hiện tại.
# Nếu MỞ TERMINAL MỚI mà lại gặp lỗi "Connection timed out" khi git clone/wget,
# chạy lại đúng dòng "sudo ip link set dev eth0 mtu 1400" ở terminal đó trước khi thử lại.



# Xác nhận
    
cat /etc/resolv.conf          # phải thấy nameserver 8.8.8.8
```

---

## Bước 1 — Cài Gazebo Harmonic

```bash
sudo apt install lsb-release wget gnupg curl -y

sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

sudo apt update
sudo apt install gz-harmonic -y

gz sim --version   # phải in ra version 8.x
```

---

## Bước 2 — Cài ROS 2 Humble

```bash
sudo apt install locales -y
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

sudo apt install software-properties-common -y
sudo add-apt-repository universe -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update && sudo apt upgrade -y
sudo apt install ros-humble-desktop python3-colcon-common-extensions python3-vcstool python3-rosdep -y

sudo rosdep init
rosdep update

echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
printenv ROS_DISTRO   # phải ra "humble"
```

**Thêm nguồn rosdep cho Gazebo** — né lỗi "Cannot locate rosdep definition for gz-transport13 / gz-sim8 / sdformat14":
```bash
sudo wget https://raw.githubusercontent.com/osrf/osrf-rosdep/master/gz/00-gazebo.list -O /etc/ros/rosdep/sources.list.d/00-gazebo.list
rosdep update
```

---

## Bước 3 — Clone ArduPilot + build SITL

```bash
cd ~
until git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git; do echo "Retry clone..."; sleep 3; done
cd ~/ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
source ~/.bashrc
```

**Cài Micro-XRCE-DDS-Gen** — né lỗi build `ardupilot_sitl` báo thiếu `microxrceddsgen`:
```bash
sudo apt install default-jre -y
cd ~
git clone --recurse-submodules --branch v4.7.0 https://github.com/ardupilot/Micro-XRCE-DDS-Gen.git
cd Micro-XRCE-DDS-Gen
./gradlew assemble
echo "export PATH=\$PATH:$HOME/Micro-XRCE-DDS-Gen/scripts" >> ~/.bashrc
source ~/.bashrc
microxrceddsgen -help   # phải in ra hướng dẫn sử dụng, không phải "command not found"
```

Build thử SITL độc lập (đảm bảo compile được trước khi đi tiếp):
```bash
cd ~/ardupilot/ArduCopter
sim_vehicle.py -v ArduCopter -f gazebo-iris --no-mavproxy 2>&1 | head -60
# Ctrl+C sau khi thấy build "finished successfully" — chỉ test build, chưa cần chạy full
```

---

## Bước 4 — Clone + build ardupilot_gazebo (bản Sim/Harmonic, KHÔNG phải Classic)

```bash
sudo apt install libgz-sim7-dev rapidjson-dev libopencv-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y
echo 'export GZ_VERSION=harmonic' >> ~/.bashrc
source ~/.bashrc

cd ~
until git clone https://github.com/ArduPilot/ardupilot_gazebo; do echo "Retry clone..."; sleep 3; done
cd ardupilot_gazebo
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j2
# Nếu bị "Killed" giữa chừng (do RAM thấp), chạy lại với:  make -j1

echo 'export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build:$GZ_SIM_SYSTEM_PLUGIN_PATH' >> ~/.bashrc
echo 'export GZ_SIM_RESOURCE_PATH=$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:$GZ_SIM_RESOURCE_PATH' >> ~/.bashrc
source ~/.bashrc

ls ~/ardupilot_gazebo/build/*.so   # phải thấy ArduPilotPlugin.so
```

**Lưu ý RAM (WSL2 thường bị OOM khi build/chạy nặng):** nếu máy có ít RAM, tạo file `.wslconfig` trên Windows tại `C:\Users\<tên_bạn>\.wslconfig`:
```ini
[wsl2]
memory=6GB
swap=4GB
```
Rồi `wsl --shutdown` từ PowerShell và mở lại Ubuntu trước khi build.

---

## Bước 5 — Kiểm tra model `iris_with_gimbal` (chưa bật ArduPilot)

Model gốc `iris_with_gimbal` include `iris_with_standoffs` + `gimbal_small_3d`, không cần sửa SDF nếu build đúng bản Harmonic ở bước 4.

**5.1 — Chạy Gazebo, kiểm tra model load:**

Terminal A:
```bash
gz sim -v4 -r ~/ardupilot_gazebo/worlds/iris_runway.sdf
```
Đợi GUI mở ổn định, quan sát log không có dòng `[Err]` nghiêm trọng. Cảnh báo `gz_frame_id ... not defined in SDF` là bình thường, bỏ qua.

Terminal B:
```bash
gz model --list          # phải thấy iris_with_gimbal
gz model -m iris_with_gimbal -j      # liệt kê joint cấp gốc — chỉ thấy gimbal_joint (khớp cố định thân-gimbal), bình thường
```

**5.2 — Xác nhận simulation đang chạy (không Pause):**
```bash
gz topic -e -t /world/iris_runway/stats -n 1 | grep paused
```
Phải ra `paused: false`. Nếu `true`, resume bằng:
```bash
gz service -s /world/iris_runway/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 3000 --req 'pause: false'
```

**5.3 — Test xoay gimbal, xác nhận bằng dữ liệu (không chỉ nhìn mắt vì gimbal nhỏ, khó thấy):**

Trong Gazebo GUI: mở **Entity Tree** (panel bên phải) → `iris_with_gimbal` → `gimbal` → click `pitch_link` → panel **Component Inspector** hiện ra, để ý mục **Pose → Rotation**.

Giữ panel đó mở, ở Terminal B gửi lệnh:
```bash
gz topic -t /gimbal/cmd_pitch -m gz.msgs.Double -p "data: -1.5"
```
Quan sát giá trị Rotation trong Component Inspector — phải thay đổi so với trước khi gửi lệnh. Thử tiếp:
```bash
gz topic -t /gimbal/cmd_yaw -m gz.msgs.Double -p "data: 1.0"
gz topic -t /gimbal/cmd_roll -m gz.msgs.Double -p "data: 0.5"
```

**5.4 — Nếu Rotation KHÔNG đổi dù đã xác nhận `paused: false`:** kiểm tra log khởi động Gazebo (Terminal A, cuộn lên) tìm 3 dòng:
```
[Dbg] [JointPositionController.cc:376] Identified joint [gimbal::roll_joint] as Entity [xx]
[Dbg] [JointPositionController.cc:376] Identified joint [gimbal::pitch_joint] as Entity [xx]
[Dbg] [JointPositionController.cc:376] Identified joint [gimbal::yaw_joint] as Entity [xx]
```
Nếu KHÔNG thấy 3 dòng này → `JointPositionController` không nhận diện được joint. Dừng lại, không qua bước 6, báo lỗi kèm log đầy đủ để xử lý riêng (không tự sửa SDF khi chưa xác định đúng nguyên nhân).

**5.5 — Kiểm tra topic camera:**
```bash
gz topic -l | grep -E 'camera|gimbal|zoom'
```
Phải thấy các topic ảnh camera dạng `.../pitch_link/sensor/camera/image` và `/model/gimbal/sensor/camera/zoom/cmd_zoom`.

**Chỉ tiếp tục Bước 6 nếu 5.3 xác nhận gimbal xoay được (Rotation thay đổi theo lệnh).**

---

## Bước 6 — Chạy full: Gazebo + ArduPilot SITL

Terminal A (nếu đang chạy từ bước 5, giữ nguyên):
```bash
gz sim -v4 -r ~/ardupilot_gazebo/worlds/iris_runway.sdf
```

Terminal C — SITL:
```bash
cd ~/ardupilot/ArduCopter
sim_vehicle.py -v ArduCopter -f gazebo-iris --console --map
```

Đợi dòng `Home: ...` xuất hiện và ổn định ~10 giây (cho ArduPilotPlugin bắt tay xong với Gazebo). Trong cửa sổ MAVProxy, gõ **từng dòng lệnh riêng biệt, Enter sau mỗi dòng** — không gõ chữ `GUIDED>`/`STABILIZE>`, đó chỉ là dấu nhắc hiển thị sẵn:

```
mode guided
arm throttle
takeoff 10
```

Kiểm tra:
```
status
```
`MasterIn` phải > 0 (không phải `[0]`), và mode list phải hiện đúng mode Copter (`STABILIZE, ALT_HOLD, LOITER, GUIDED, RTL...`).

**Nếu `MasterIn` vẫn `[0]` sau khi làm đúng hết:**
- Kiểm tra lại simulation có bị Pause không (xem bước 5.2), vì đây là nguyên nhân phổ biến nhất.
- Kiểm tra có tiến trình `gz sim`/`arducopter` cũ nào chưa tắt hết không:
```bash
ps aux | grep -E "gz sim|arducopter|mavproxy" | grep -v grep
```
Nếu thấy nhiều hơn 1 dòng mỗi loại, dọn sạch và chạy lại từ đầu Bước 6:
```bash
pkill -9 -f "gz sim"; pkill -9 -f arducopter; pkill -9 -f mavproxy; pkill -9 -f sim_vehicle
sleep 3
```

**Xác nhận drone bay được:** sau `takeoff 10`, quan sát cửa sổ Gazebo, drone phải bay lên khoảng 10m. Nếu không thấy trong khung hình, kiểm tra vị trí thật bằng:
```bash
gz model -m iris_with_gimbal -p
```
Xem giá trị `z` có tăng dần tới ~10 không (camera GUI có thể đang zoom lệch chỗ khác).

---

## Bước 7 — Bridge camera qua ROS 2 (chuẩn bị cho YOLO)

Lấy đúng tên topic camera từ bước 5.5, ví dụ dạng:
`/world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image`

```bash
ros2 run ros_gz_bridge parameter_bridge \
  "/world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image@sensor_msgs/msg/Image[gz.msgs.Image"
```

Kiểm tra:
```bash
ros2 topic list | grep camera
ros2 topic hz /world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image
```
Nếu có tần số (~30Hz) → camera đã sẵn sàng cho YOLO.

---

## Tiêu chí hoàn thành (dừng lại và báo cáo nếu đạt đủ các mục sau)

- [ ] Bước 0-4: cài đặt không lỗi, các lệnh xác nhận (`gz sim --version`, `printenv ROS_DISTRO`, `ls build/*.so`) đều ra kết quả đúng.
- [ ] Bước 5.3: gimbal xoay được, xác nhận qua Component Inspector (Rotation thay đổi theo lệnh `gz topic`).
- [ ] Bước 6: `sim_vehicle.py` chạy, `mode guided` → `arm throttle` → `takeoff 10` thành công, `status` cho `MasterIn > 0`, drone lên độ cao ~10m xác nhận qua `gz model -p`.
- [ ] Bước 7: `ros2 topic hz` trên topic camera ra tần số ổn định (~30Hz).

## Việc KHÔNG được tự ý làm khi thực thi file này

- Không tự sửa file `.sdf` trong `~/ardupilot_gazebo/models/` nếu bước 5.4 chưa xác định rõ nguyên nhân — báo lại log đầy đủ trước.
- Không chạy song song nhiều phiên `gz sim` hoặc `sim_vehicle.py` — luôn `pkill -9` dọn sạch trước khi chạy lại.
- Không bỏ qua bước xác nhận (checklist trên) để nhảy thẳng sang viết code YOLO — hạ tầng phải ổn định trước.