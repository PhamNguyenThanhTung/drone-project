# PHASE2.md

# ArduPilot + Gazebo Harmonic + ROS 2
# Phase 2: AI Vision & Closed-Loop Gimbal Tracking

## 0. Mục tiêu

Triển khai giai đoạn AI Vision & Control Loop trên nền hạ tầng đã hoàn thành ở Phase 1:

```text
Gazebo Sim Harmonic
        |
        | Camera image
        v
ros_gz_bridge
        |
        v
YOLOv8 + ByteTrack
        |
        | target_x, target_y
        v
/tracking/error
        |
        v
PID Controller
        |
        | yaw / pitch command
        v
Gazebo Gimbal
```

Mục tiêu cuối:

1. Gazebo có một human actor chuyển động theo trajectory.
2. Camera trên `iris_with_gimbal` nhìn thấy actor.
3. Camera image được bridge sang ROS 2.
4. YOLOv8 Nano phát hiện class `person`.
5. ByteTrack duy trì `track_id`.
6. Node tracking tính tâm bounding box.
7. Node tracking publish sai số tới `/tracking/error`.
8. PID controller nhận sai số.
9. PID điều khiển yaw/pitch của gimbal.
10. Gimbal tự động quay để giữ người gần tâm camera.
11. Toàn bộ closed-loop hoạt động ổn định.
12. Có số liệu đánh giá định lượng.

Không triển khai drone autonomous navigation ở Phase 2. Chỉ điều khiển gimbal.

---

# 1. Điều kiện đầu vào

Phase 2 chỉ được bắt đầu khi Phase 1 đã đạt:

* Ubuntu 22.04.x
* WSL2
* ROS 2 Humble
* Gazebo Sim Harmonic
* ArduPilot SITL
* `ardupilot_gazebo`
* `iris_with_gimbal`
* camera topic tồn tại
* gimbal yaw/pitch command hoạt động
* SITL kết nối Gazebo
* drone có thể takeoff

Kiểm tra:

```bash
ls ~/ardupilot
ls ~/ardupilot_gazebo

gz sim --version

printenv ROS_DISTRO

ls ~/ardupilot_gazebo/build/*.so

gz topic -l | grep -E 'camera|gimbal'
```

Kỳ vọng:

```text
ROS_DISTRO=humble
Gazebo Sim 8.x
```

Nếu các điều kiện nền tảng không đạt:

**STOP.**

Không tiếp tục cài YOLO hoặc triển khai Phase 2.

---

# 2. Quy tắc bắt buộc cho Agent

Agent phải tuân thủ các quy tắc sau.

## 2.1. Không bỏ qua checkpoint

Phải thực hiện theo thứ tự:

```text
A0
A1
A2
A3
A4
A5
A6
A7
B0
B1
B2
B3
C0
C1
C2
C3
D0
D1
D2
```

Chỉ chuyển bước khi checkpoint hiện tại đạt.

## 2.2. Không tự ý sửa kiến trúc

Không:

* đổi Gazebo
* đổi ROS 2
* đổi ArduPilot
* đổi YOLO sang model khác
* đổi ByteTrack sang tracker khác
* đổi giao thức gimbal
* thêm flight controller logic ngoài phạm vi Phase 2

Nếu cần thay đổi kiến trúc:

**STOP** và báo cáo.

## 2.3. SDF

Được phép chỉnh `iris_runway.sdf` để thêm actor trong Phase 2.

Không được tự ý sửa:

```text
gimbal_small_3d/model.sdf
```

để tăng camera FPS.

Việc tối ưu camera FPS là task riêng.

Nếu actor không load được do lỗi SDF:

**STOP**, đọc log và báo nguyên nhân.

Không tự ý sửa cấu trúc gimbal/camera.

## 2.4. Không xóa project quan trọng

Không chạy:

```bash
rm -rf ~/ardupilot
rm -rf ~/ardupilot_gazebo
```

Không xóa workspace hiện có.

## 2.5. Không chạy nhiều Gazebo/SITL

Trước khi khởi động stack:

```bash
ps aux | grep -E "gz sim|arducopter|mavproxy|parameter_bridge" | grep -v grep
```

Nếu có instance cũ:

```bash
pkill -9 -f "gz si[m]"
pkill -9 -x arducopter
pkill -9 -f "mavproxy.p[y]"
pkill -9 -f "parameter_brid[g]e"
```

Sau đó kiểm tra lại process.

---

# 3. Kiến trúc Phase 2

## 3.1. Gazebo

```text
iris_with_gimbal
    |
    +-- gimbal
          |
          +-- pitch_link
                |
                +-- camera
```

Actor:

```text
target_human
```

Actor di chuyển theo trajectory.

## 3.2. ROS 2

Camera:

```text
Gazebo
  |
  v
ros_gz_bridge
  |
  v
/camera/image_raw
```

Detector:

```text
/camera/image_raw
        |
        v
yolo_detector_node
        |
        v
/tracking/error
```

Controller:

```text
/tracking/error
        |
        v
gimbal_controller_node
        |
        +----> /gimbal/cmd_yaw
        |
        +----> /gimbal/cmd_pitch
```

---

# 4. BƯỚC A — Cấu hình Human Actor

## A0 — Backup World

Trước khi sửa world:

```bash
cp ~/ardupilot_gazebo/worlds/iris_runway.sdf \
   ~/ardupilot_gazebo/worlds/iris_runway.sdf.phase2.backup
```

Verify:

```bash
ls -lh ~/ardupilot_gazebo/worlds/iris_runway.sdf*
```

Phải tồn tại:

```text
iris_runway.sdf
iris_runway.sdf.phase2.backup
```

Nếu backup thất bại:

**STOP.**

---

# 5. A1 — Thêm actor

File:

```text
~/ardupilot_gazebo/worlds/iris_runway.sdf
```

Thêm actor vào bên trong:

```xml
<world name="iris_runway">
```

và trước:

```xml
</world>
```

Actor:

```xml
<actor name="target_human">
  <skin>
    <filename>https://fuel.gazebosim.org/1.0/Mingfei/models/actor/tip/files/meshes/walk.dae</filename>
    <scale>1.0</scale>
  </skin>

  <animation name="walk">
    <filename>https://fuel.gazebosim.org/1.0/Mingfei/models/actor/tip/files/meshes/walk.dae</filename>
    <interpolate_x>true</interpolate_x>
  </animation>

  <script>
    <loop>true</loop>
    <delay_start>0.0</delay_start>
    <auto_start>true</auto_start>

    <trajectory id="0" type="walk">

      <waypoint>
        <time>0</time>
        <pose>0 5 0 0 0 0</pose>
      </waypoint>

      <waypoint>
        <time>10</time>
        <pose>10 5 0 0 0 0</pose>
      </waypoint>

      <waypoint>
        <time>20</time>
        <pose>0 5 0 0 0 3.14</pose>
      </waypoint>

    </trajectory>
  </script>
</actor>
```

Không thay đổi:

```text
iris_with_gimbal
camera
gimbal_joint
roll_joint
pitch_joint
yaw_joint
ArduPilotPlugin
```

---

# 6. A2 — Kiểm tra SDF

Kiểm tra actor tồn tại:

```bash
grep -n 'actor name="target_human"' \
~/ardupilot_gazebo/worlds/iris_runway.sdf
```

Phải tìm thấy actor.

Kiểm tra XML cơ bản:

```bash
gz sdf -p ~/ardupilot_gazebo/worlds/iris_runway.sdf > /tmp/iris_runway_expanded.sdf
```

Nếu lệnh lỗi:

**STOP.**

Đọc lỗi trước khi tiếp tục.

---

# 7. A3 — Chạy Gazebo kiểm tra Actor

Khởi động:

```bash
export LIBGL_ALWAYS_SOFTWARE=1

gz sim -v4 -r \
~/ardupilot_gazebo/worlds/iris_runway.sdf
```

Trong terminal khác:

```bash
gz model --list
```

Phải thấy:

```text
target_human
iris_with_gimbal
```

Kiểm tra pose actor:

```bash
gz model -m target_human -p
```

Đợi vài giây rồi chạy lại:

```bash
gz model -m target_human -p
```

Giá trị `x/y` phải thay đổi theo trajectory.

Checkpoint:

```text
A3 PASS:
- target_human tồn tại
- actor chuyển động
- Gazebo không crash
```

Nếu actor không xuất hiện:

**STOP.**

---

# 8. A4 — Kiểm tra camera

Lấy topic:

```bash
gz topic -l | grep camera
```

Tìm topic:

```text
/world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image
```

Không hard-code topic nếu Gazebo thực tế dùng tên khác.

Lưu topic thực tế vào biến:

```bash
export CAMERA_GZ_TOPIC="<camera topic thực tế>"
```

Kiểm tra dữ liệu:

```bash
gz topic -e -t "$CAMERA_GZ_TOPIC" -n 1
```

Phải nhận được image message.

---

# 9. A5 — Kiểm tra camera resolution và FPS

Không giả định camera là 416x416.

Kiểm tra message:

```bash
gz topic -e -t "$CAMERA_GZ_TOPIC" -n 1
```

Ghi nhận:

```text
width
height
```

Kiểm tra FPS.

Nếu camera hiện tại vẫn là:

```text
10 Hz sim-time
```

không được tự sửa SDF camera.

Ghi nhận FPS thực tế.

Do Phase 1 đã đo được RTF thấp, FPS wall-clock có thể thấp hơn FPS sim-time.

Điều này không ngăn Phase 2 tiếp tục.

Checkpoint:

```text
A5 PASS:
- camera topic có dữ liệu
- resolution xác định
- FPS thực tế xác định
```

---

# 10. A6 — Cài dependency ROS 2

Kiểm tra package:

```bash
source /opt/ros/humble/setup.bash

ros2 pkg prefix cv_bridge
ros2 pkg prefix sensor_msgs
ros2 pkg prefix geometry_msgs
ros2 pkg prefix std_msgs
ros2 pkg prefix ros_gz_bridge
```

Nếu thiếu:

```bash
sudo apt update

sudo apt install -y \
    python3-opencv \
    ros-humble-cv-bridge \
    ros-humble-sensor-msgs \
    ros-humble-geometry-msgs \
    ros-humble-std-msgs
```

Đối với bridge, sử dụng package đã được xác nhận ở Phase 1:

```bash
ros2 pkg prefix ros_gz_bridge
```

Không tự thay bằng package bridge khác.

---

# 11. A7 — Cài YOLO

Kiểm tra Python:

```bash
python3 --version
```

Kiểm tra pip:

```bash
python3 -m pip --version
```

Cài Ultralytics:

```bash
python3 -m pip install --user ultralytics
```

Verify:

```bash
python3 -c "import ultralytics; print(ultralytics.__version__)"
```

Verify OpenCV:

```bash
python3 -c "import cv2; print(cv2.__version__)"
```

Verify cv_bridge:

```bash
python3 -c "from cv_bridge import CvBridge; print('cv_bridge OK')"
```

Nếu bất kỳ dependency nào lỗi:

**STOP** và xử lý dependency.

---

# 12. BƯỚC B — Camera Bridge

## B0 — Khởi động ROS camera bridge

Source ROS:

```bash
source /opt/ros/humble/setup.bash
```

Bridge:

```bash
ros2 run ros_gz_bridge parameter_bridge \
"$CAMERA_GZ_TOPIC@sensor_msgs/msg/Image[gz.msgs.Image"
```

Nếu topic thực tế chưa được lưu vào biến:

```bash
export CAMERA_GZ_TOPIC="/world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image"
```

---

# 13. B1 — Verify ROS camera

```bash
ros2 topic list | grep camera
```

Kiểm tra message:

```bash
ros2 topic echo "$CAMERA_ROS_TOPIC" --once
```

Nếu chưa biết ROS topic:

```bash
ros2 topic list | grep -E 'camera|image'
```

Kiểm tra rate:

```bash
ros2 topic hz "$CAMERA_ROS_TOPIC"
```

Ghi nhận FPS thực tế.

Không yêu cầu 30 Hz ở Phase 2.

Phase 2 dùng FPS thực tế để phát triển pipeline.

---

# 14. B2 — Tạo workspace Phase 2

Không đặt code trực tiếp vào `~/ardupilot`.

Tạo:

```bash
mkdir -p ~/phase2_ws/src
cd ~/phase2_ws/src
```

Tạo package:

```bash
ros2 pkg create \
    --build-type ament_python \
    vision_tracking
```

Cấu trúc:

```text
~/phase2_ws/
└── src/
    └── vision_tracking/
        ├── package.xml
        ├── setup.py
        ├── setup.cfg
        └── vision_tracking/
```

Tạo thư mục:

```bash
mkdir -p ~/phase2_ws/src/vision_tracking/vision_tracking
```

---

# 15. B3 — Node YOLO + ByteTrack

File:

```text
~/phase2_ws/src/vision_tracking/vision_tracking/yolo_detector_node.py
```

Code phải thực hiện:

```text
Image
 ↓
cv_bridge
 ↓
OpenCV
 ↓
YOLOv8n
 ↓
class=person
 ↓
ByteTrack
 ↓
track_id
 ↓
target selection
 ↓
bounding box center
 ↓
tracking error
```

Không chỉ lấy `boxes[0]`.

Sử dụng ByteTrack rõ ràng:

```python
results = self.model.track(
    cv_image,
    persist=True,
    tracker="bytetrack.yaml",
    classes=[0],
    verbose=False
)
```

---

# 16. B4 — Target selection

Nếu có nhiều người:

1. Chỉ xét class `person`.
2. Chỉ xét detection có track ID.
3. Nếu chưa có target:
   * chọn person có bounding box lớn nhất.
4. Nếu đã có target:
   * ưu tiên giữ `track_id` hiện tại.
5. Nếu target mất:
   * giữ trạng thái LOST trong thời gian ngắn.
6. Nếu quá timeout:
   * reset target.

Không tự động chuyển target liên tục giữa nhiều người.

---

# 17. B5 — Tracking error

Định nghĩa:

```text
center_x = image_width / 2
center_y = image_height / 2

target_x = (xmin + xmax) / 2
target_y = (ymin + ymax) / 2

error_x = target_x - center_x
error_y = target_y - center_y
```

Publish:

```text
/tracking/error
```

Message:

```text
geometry_msgs/msg/Point
```

Quy ước:

```text
x = error_x
y = error_y
z = bounding_box_area
```

Không sử dụng `z` làm PID input.

---

# 18. B6 — Không hard-code 416x416 nếu camera khác

Node phải có parameters:

```text
img_width
img_height
```

Mặc định:

```text
416
416
```

Nếu camera thực tế có resolution khác:

Node có thể resize ảnh trước khi inference.

Ví dụ:

```python
cv_image = cv2.resize(
    cv_image,
    (self.W, self.H)
)
```

Ghi log:

```text
Original image:
WxH

Inference image:
416x416
```

---

# 19. B7 — Debug image

Debug image là optional.

Parameter:

```text
show_debug_image
```

Mặc định:

```text
false
```

Nếu bật:

```text
bounding box
track ID
camera center
target center
error
```

được vẽ lên ảnh.

Không để GUI trở thành dependency bắt buộc của detector.

---

# 20. B8 — Build detector

```bash
cd ~/phase2_ws

source /opt/ros/humble/setup.bash

colcon build --symlink-install
```

Source:

```bash
source ~/phase2_ws/install/setup.bash
```

Verify:

```bash
ros2 pkg list | grep vision_tracking
```

---

# 21. B9 — Test YOLO độc lập

Chạy detector:

```bash
ros2 run vision_tracking yolo_detector_node
```

Kiểm tra:

```bash
ros2 topic echo /tracking/error
```

Khi actor nằm trong camera:

```text
x != 0
y != 0
z > 0
```

Kiểm tra tracking ID trong log.

Checkpoint:

```text
B9 PASS:
- YOLO detects person
- class=0
- ByteTrack active
- track_id exists
- /tracking/error publishes
```

Nếu YOLO không detect:

Không chuyển sang PID.

---

# 22. BƯỚC C — Gimbal ROS Bridge

## C0 — Kiểm tra Gazebo gimbal topics

```bash
gz topic -l | grep gimbal
```

Phải có:

```text
/gimbal/cmd_yaw
/gimbal/cmd_pitch
```

Nếu topic khác:

Dùng topic thực tế.

---

# 23. C1 — Bridge Yaw

```bash
ros2 run ros_gz_bridge parameter_bridge \
"/gimbal/cmd_yaw@std_msgs/msg/Float64]gz.msgs.Double"
```

---

# 24. C2 — Bridge Pitch

```bash
ros2 run ros_gz_bridge parameter_bridge \
"/gimbal/cmd_pitch@std_msgs/msg/Float64]gz.msgs.Double"
```

---

# 25. C3 — Test chiều quay

Test yaw:

```bash
ros2 topic pub --once \
/gimbal/cmd_yaw \
std_msgs/msg/Float64 \
"{data: 0.5}"
```

Sau đó:

```bash
ros2 topic pub --once \
/gimbal/cmd_yaw \
std_msgs/msg/Float64 \
"{data: -0.5}"
```

Test pitch:

```bash
ros2 topic pub --once \
/gimbal/cmd_pitch \
std_msgs/msg/Float64 \
"{data: -0.5}"
```

Kiểm tra Gazebo pose.

Nếu chiều quay ngược so với controller:

Không sửa SDF.

Chỉ xác định quy ước dấu trong controller.

Checkpoint:

```text
C3 PASS:
- ROS -> Gazebo yaw works
- ROS -> Gazebo pitch works
- direction verified
```

---

# 26. BƯỚC D — PID Controller

## D0 — Nguyên tắc PID

Không dùng:

```text
integral += error
derivative = error - last_error
```

PID phải dùng thời gian thực:

```text
dt = current_time - previous_time

integral += error * dt

derivative =
    (error - previous_error) / dt

output =
    Kp * error
    + Ki * integral
    + Kd * derivative
```

---

# 27. D1 — Controller parameters

Các thông số phải là ROS parameters:

```text
kp_yaw
ki_yaw
kd_yaw

kp_pitch
ki_pitch
kd_pitch

max_yaw
min_yaw

max_pitch
min_pitch

max_yaw_rate
max_pitch_rate

integral_limit

deadband

target_timeout
```

Không hard-code toàn bộ trong source.

Giá trị ban đầu có thể:

```text
kp_yaw = 0.005
ki_yaw = 0.0001
kd_yaw = 0.001

kp_pitch = 0.005
ki_pitch = 0.0001
kd_pitch = 0.001
```

Các giá trị này chỉ là initial guess.

Không coi chúng là giá trị đã được calibrate.

---

# 28. D2 — Anti-windup

Giới hạn:

```text
integral_x
integral_y
```

Ví dụ:

```text
[-100, 100]
```

Nhưng tích phân phải được nhân với `dt`.

Nếu output đã chạm giới hạn actuator:

không tiếp tục tích phân theo hướng làm saturation nặng hơn.

---

# 29. D3 — Deadband

Để tránh gimbal rung quanh tâm:

```text
if abs(error_x) < deadband:
    error_x = 0

if abs(error_y) < deadband:
    error_y = 0
```

Giá trị ban đầu:

```text
deadband = 3 pixels
```

Có thể calibrate sau.

---

# 30. D4 — Giới hạn góc

Gimbal:

```text
yaw:
-1.57 → +1.57 rad

pitch:
-1.57 → 0 rad
```

Không gửi command vượt giới hạn.

---

# 31. D5 — Giới hạn tốc độ

Không thay đổi góc quá nhanh giữa hai frame.

Ví dụ:

```text
max_yaw_rate
max_pitch_rate
```

để tránh:

```text
large error
→ huge PID output
→ gimbal jumps
```

---

# 32. D6 — Lost target

Nếu detector không publish trong:

```text
target_timeout
```

controller phải chuyển trạng thái:

```text
TRACKING
```

sang:

```text
LOST
```

Khi LOST:

1. Không tiếp tục tích phân.
2. Không tiếp tục tăng command.
3. Giữ góc hiện tại hoặc thực hiện behavior đã cấu hình.
4. Reset integral nếu target mất quá lâu.

Không tự động quay drone.

---

# 33. D7 — Controller node

File:

```text
~/phase2_ws/src/vision_tracking/vision_tracking/gimbal_controller_node.py
```

Input:

```text
/tracking/error
```

Output:

```text
/gimbal/cmd_yaw
/gimbal/cmd_pitch
```

Controller phải log:

```text
state
error_x
error_y
yaw_command
pitch_command
dt
```

Không log ở tốc độ quá cao.

Có thể giới hạn log:

```text
1 Hz
```

---

# 34. D8 — Build controller

```bash
cd ~/phase2_ws

source /opt/ros/humble/setup.bash

colcon build --symlink-install

source ~/phase2_ws/install/setup.bash
```

Verify:

```bash
ros2 pkg list | grep vision_tracking
```

---

# 35. BƯỚC E — Integration

## E0 — Dọn process cũ

Trước khi chạy:

```bash
ps aux | grep -E "gz sim|arducopter|mavproxy|parameter_bridge|yolo_detector|gimbal_controller" | grep -v grep
```

Dọn nếu cần:

```bash
pkill -9 -f "gz si[m]"
pkill -9 -x arducopter
pkill -9 -f "mavproxy.p[y]"
pkill -9 -f "parameter_brid[g]e"
pkill -9 -f "yolo_detector_nod[e]"
pkill -9 -f "gimbal_controller_no[d]"
```

---

# 36. E1 — Terminal 1: Gazebo

```bash
export LIBGL_ALWAYS_SOFTWARE=1

gz sim -v4 -r \
~/ardupilot_gazebo/worlds/iris_runway.sdf
```

Verify:

```bash
gz model --list
```

Phải có:

```text
iris_with_gimbal
target_human
```

Verify simulation:

```bash
gz topic -e \
-t /world/iris_runway/stats \
-n 1 | grep paused
```

Phải:

```text
paused: false
```

---

# 37. E2 — Terminal 2: ArduPilot

```bash
source /opt/ros/humble/setup.bash

cd ~/ardupilot/ArduCopter

sim_vehicle.py \
-v ArduCopter \
-f gazebo-iris \
--model JSON \
--add-param-file=$HOME/ardupilot_gazebo/config/gazebo-iris-gimbal.parm \
--console \
--map
```

MAVProxy:

```text
mode guided
arm throttle
takeoff 10
```

Verify:

```text
MasterIn > 0
```

Drone phải ổn định trong Gazebo.

Nếu chỉ test AI/gimbal và không cần bay:

có thể giữ drone tại vị trí ban đầu.

---

# 38. E3 — Terminal 3: Camera bridge

```bash
source /opt/ros/humble/setup.bash

ros2 run ros_gz_bridge parameter_bridge \
"$CAMERA_GZ_TOPIC@sensor_msgs/msg/Image[gz.msgs.Image"
```

Verify:

```bash
ros2 topic list | grep image
```

---

# 39. E4 — Terminal 3: Gimbal bridge

```bash
ros2 run ros_gz_bridge parameter_bridge \
"/gimbal/cmd_yaw@std_msgs/msg/Float64]gz.msgs.Double"
```

Terminal khác:

```bash
ros2 run ros_gz_bridge parameter_bridge \
"/gimbal/cmd_pitch@std_msgs/msg/Float64]gz.msgs.Double"
```

---

# 40. E5 — Terminal 4: YOLO

```bash
source /opt/ros/humble/setup.bash
source ~/phase2_ws/install/setup.bash

ros2 run vision_tracking yolo_detector_node
```

Verify:

```bash
ros2 topic echo /tracking/error
```

Phải nhận:

```text
x
y
z
```

---

# 41. E6 — Terminal 5: PID

```bash
source /opt/ros/humble/setup.bash
source ~/phase2_ws/install/setup.bash

ros2 run vision_tracking gimbal_controller_node
```

Controller bắt đầu nhận:

```text
/tracking/error
```

và publish:

```text
/gimbal/cmd_yaw
/gimbal/cmd_pitch
```

---

# 42. BƯỚC F — Kiểm thử theo từng tầng

Không đánh giá closed-loop ngay.

## F0 — Actor test

Actor phải:

```text
exists = true
moving = true
```

---

## F1 — Camera test

Camera:

```text
image_available = true
```

Ghi:

```text
resolution
sim FPS
wall FPS
```

---

## F2 — Detection test

YOLO:

```text
person_detected = true
confidence > threshold
```

---

## F3 — Tracking test

ByteTrack:

```text
track_id != None
```

Trong nhiều frame liên tiếp:

```text
track_id
```

phải ổn định.

---

# 43. F4 — Tracking error test

Khi người ở bên trái:

```text
error_x < 0
```

Khi người ở bên phải:

```text
error_x > 0
```

Khi người ở trên:

```text
error_y < 0
```

Khi người ở dưới:

```text
error_y > 0
```

Xác nhận quy ước này trước khi bật PID.

---

# 44. F5 — PID direction test

Không bật full loop ngay.

Đưa target sang trái.

Quan sát:

```text
error_x
yaw_command
gimbal_yaw
```

Nếu command làm target lệch xa tâm hơn:

đảo dấu control output trong controller.

Không sửa SDF chỉ vì sign convention.

Lặp lại với pitch.

---

# 45. F6 — Closed-loop test

Bật toàn bộ:

```text
Gazebo
+
ArduPilot
+
camera bridge
+
gimbal bridge
+
YOLO
+
ByteTrack
+
PID
```

Quan sát:

```text
target
camera center
error_x
error_y
gimbal yaw
gimbal pitch
```

Mục tiêu:

```text
error_x → gần 0
error_y → gần 0
```

---

# 46. BƯỚC G — Calibrate PID

## G0 — Initial tuning

Bắt đầu:

```text
Ki = 0
Kd = 0
```

Chỉ dùng:

```text
P controller
```

Tăng:

```text
Kp
```

cho tới khi gimbal phản ứng đủ nhanh nhưng chưa dao động mạnh.

---

# 47. G1 — Add derivative

Sau khi P ổn định:

tăng:

```text
Kd
```

để giảm overshoot và cải thiện phản ứng khi actor đổi hướng.

---

# 48. G2 — Add integral

Chỉ thêm:

```text
Ki
```

nếu còn steady-state error.

Ki phải nhỏ.

Không dùng Ki để giải quyết vấn đề camera FPS hoặc detector latency.

---

# 49. G3 — Không tuning PID khi detection không ổn định

Nếu:

```text
track_id
```

liên tục mất:

không tăng Kp.

Nếu:

```text
camera FPS
```

quá thấp:

không cố giải quyết bằng PID.

Nếu:

```text
YOLO latency
```

cao:

tối ưu inference trước.

---

# 50. BƯỚC H — Đánh giá định lượng

Không chỉ đánh giá bằng mắt.

Ghi các metric:

## Detection

```text
detection rate
confidence
inference latency
```

## Tracking

```text
track ID stability
track loss count
track loss duration
```

## Control

```text
mean absolute error X
mean absolute error Y
maximum error X
maximum error Y
settling time
overshoot
steady-state error
```

---

# 51. Tiêu chí Detection

PASS nếu:

```text
person được detect liên tục trong phần lớn thời gian actor nằm trong FOV
```

Mục tiêu:

```text
detection availability >= 90%
```

Không yêu cầu 90% nếu camera FPS quá thấp do giới hạn Phase 1, nhưng phải ghi rõ số đo thực tế.

---

# 52. Tiêu chí Tracking

PASS nếu:

```text
ByteTrack tạo track_id
```

và:

```text
track_id không thay đổi liên tục khi chỉ có một người
```

Nếu actor bị mất khỏi FOV:

controller phải chuyển sang:

```text
LOST
```

thay vì dùng dữ liệu cũ vô hạn.

---

# 53. Tiêu chí Control

Mục tiêu:

```text
mean |error_x| < 20 px
mean |error_y| < 20 px
```

trên vùng camera:

```text
416 x 416
```

Nếu điều kiện camera không cho phép đạt:

ghi rõ:

```text
FAIL
```

và nguyên nhân.

Không tự sửa tiêu chí.

---

# 54. Tiêu chí Gimbal

PASS nếu:

```text
gimbal quay đúng hướng
gimbal không oscillate liên tục
gimbal không vượt giới hạn
target tiến về vùng trung tâm
```

Không yêu cầu target luôn chính xác tại:

```text
208, 208
```

Có thể đánh giá bằng sai số trung bình.

---

# 55. Tiêu chí Closed-loop

Closed-loop PASS khi:

```text
Actor moving
    AND
Camera streaming
    AND
YOLO detects person
    AND
ByteTrack tracks person
    AND
/tracking/error publishes
    AND
PID publishes gimbal commands
    AND
Gimbal follows target
```

---

# 56. Xử lý các lỗi thường gặp

## YOLO không detect

Kiểm tra:

```bash
ros2 topic hz "$CAMERA_ROS_TOPIC"
```

và:

```bash
ros2 topic echo "$CAMERA_ROS_TOPIC" --once
```

Sau đó kiểm tra:

```text
resolution
encoding
brightness
actor visibility
```

Không vội sửa PID.

---

## ByteTrack mất target

Kiểm tra:

```text
confidence
FPS
motion
occlusion
```

Không đổi tracker nếu chưa có bằng chứng tracker là nguyên nhân.

---

## Gimbal quay ngược

Kiểm tra sign convention:

```text
error_x
yaw command
yaw movement
```

Chỉ sửa sign trong controller.

---

## Gimbal rung

Thứ tự xử lý:

```text
1. giảm Kp
2. kiểm tra Kd
3. kiểm tra deadband
4. kiểm tra FPS
5. kiểm tra latency
```

Không tăng Ki ngay.

---

## Gimbal phản ứng chậm

Kiểm tra:

```text
camera FPS
YOLO latency
ROS latency
PID dt
```

Sau đó mới tăng Kp.

---

## Camera FPS thấp

Không sửa:

```text
gimbal_small_3d/model.sdf
```

trong Phase 2.

Ghi nhận:

```text
actual FPS
RTF
CPU
GPU
```

và xử lý ở optimization phase riêng.

---

# 57. GPU

Kiểm tra:

```bash
ls -l /dev/dxg
```

Kiểm tra NVIDIA:

```bash
nvidia-smi
```

Nếu YOLO có thể sử dụng CUDA:

kiểm tra PyTorch:

```bash
python3 -c "import torch; print(torch.cuda.is_available())"
```

Nếu:

```text
False
```

thì chạy CPU.

Không bắt buộc phải ép GPU nếu môi trường WSL2 chưa cấu hình CUDA cho Python.

Không thay đổi Gazebo renderer chỉ để làm YOLO hoạt động.

---

# 58. YOLO device

Node nên có parameter:

```text
device
```

Giá trị:

```text
cpu
```

hoặc:

```text
cuda:0
```

Mặc định:

```text
cpu
```

Nếu CUDA hoạt động ổn định:

có thể dùng:

```text
cuda:0
```

Không coi GPU là dependency bắt buộc để Phase 2 PASS.

---

# 59. Không nâng camera lên 30 Hz trong Phase 2

Phase 1 đã xác nhận:

```text
camera update_rate = 10 Hz sim-time
RTF ≈ 0.43
ROS wall-clock ≈ 2.1 Hz
```

Phase 2 không được tự sửa:

```text
<update_rate>
```

trong camera SDF.

Nếu cần 30 Hz:

tạo task riêng:

```text
PHASE2-OPTIMIZATION
```

và phải đánh giá:

```text
CPU
GPU
RTF
camera FPS
ROS FPS
YOLO FPS
end-to-end latency
```

---

# 60. Final checklist

## Environment

```text
[ ] Ubuntu 22.04
[ ] WSL2
[ ] ROS 2 Humble
[ ] Gazebo Harmonic
[ ] ArduPilot SITL
```

## Actor

```text
[ ] target_human exists
[ ] target_human moves
[ ] trajectory loops
```

## Camera

```text
[ ] camera topic exists
[ ] camera publishes image
[ ] resolution verified
[ ] FPS measured
```

## ROS

```text
[ ] ros_gz_bridge working
[ ] Image received by ROS
```

## YOLO

```text
[ ] ultralytics installed
[ ] YOLOv8n loads
[ ] person class detected
```

## ByteTrack

```text
[ ] ByteTrack explicitly enabled
[ ] track_id available
[ ] track_id stable
```

## Tracking

```text
[ ] /tracking/error exists
[ ] error_x verified
[ ] error_y verified
[ ] target loss handled
```

## Gimbal

```text
[ ] ROS yaw command reaches Gazebo
[ ] ROS pitch command reaches Gazebo
[ ] yaw direction verified
[ ] pitch direction verified
```

## PID

```text
[ ] dt-based PID
[ ] anti-windup
[ ] deadband
[ ] output limits
[ ] rate limits
[ ] lost-target handling
```

## Closed-loop

```text
[ ] YOLO detects moving target
[ ] ByteTrack tracks target
[ ] PID reacts to error
[ ] gimbal follows target
[ ] target moves toward camera center
[ ] no continuous oscillation
```

---

# 61. Final report format

Sau khi hoàn thành, báo cáo chính xác theo format:

```text
ENVIRONMENT

- Ubuntu:
- WSL:
- CPU:
- RAM:
- GPU:
- ROS:
- Gazebo:

ACTOR

- Actor loaded:
- Actor moving:
- Trajectory:
- Status:

CAMERA

- Gazebo topic:
- ROS topic:
- Resolution:
- Sim-time FPS:
- Wall-clock FPS:

YOLO

- Model:
- Device:
- Person detection:
- Detection rate:
- Average inference latency:

BYTETRACK

- Tracker:
- Track ID:
- Track stability:
- Lost target count:

TRACKING ERROR

- Topic:
- Mean |error_x|:
- Mean |error_y|:
- Max |error_x|:
- Max |error_y|:

GIMBAL

- Yaw bridge:
- Pitch bridge:
- Yaw direction:
- Pitch direction:
- Gimbal limits:

PID

- Kp yaw:
- Ki yaw:
- Kd yaw:
- Kp pitch:
- Ki pitch:
- Kd pitch:
- Deadband:
- Max yaw rate:
- Max pitch rate:

CLOSED LOOP

- Detection:
- Tracking:
- Error publishing:
- PID:
- Gimbal response:
- Target centering:
- Oscillation:

FINAL

- Actor working: YES/NO
- Camera working: YES/NO
- YOLO working: YES/NO
- ByteTrack working: YES/NO
- Tracking error working: YES/NO
- Gimbal control working: YES/NO
- PID working: YES/NO
- Closed-loop tracking working: YES/NO
- Infrastructure ready: YES/NO
```

# 62. Quy tắc kết luận

Không được báo:

```text
Infrastructure ready: YES
```

nếu một trong các thành phần bắt buộc chưa đạt.

Nếu chỉ YOLO hoạt động nhưng PID chưa hoạt động:

```text
Closed-loop tracking working: NO
```

Nếu gimbal hoạt động nhưng tracking chưa ổn định:

```text
Closed-loop tracking working: NO
```

Nếu camera FPS thấp nhưng toàn bộ pipeline vẫn hoạt động:

```text
Closed-loop tracking working: YES
```

nhưng phải ghi rõ FPS thực tế và giới hạn hiệu năng.

Nếu có lỗi ngoài phạm vi xử lý an toàn:

```text
Status: BLOCKED
```

và báo:

```text
1. Lệnh đã chạy
2. Log lỗi
3. Nguyên nhân dự kiến
4. Bước cần người dùng cho phép
```

Không tự ý thay đổi kiến trúc hoặc sửa SDF ngoài phạm vi được phép.

---

# 63. Ghi chú triển khai thực tế (dành cho người trực tiếp thao tác)

Phần này viết cho người thật ngồi trước máy, không phải cho agent. Mục đích là giúp tránh các lỗi vặt hay gặp khi làm thủ công trên WSL2/Gazebo/ROS 2.

## 63.1. Chuẩn bị trước khi bắt đầu

* Dành riêng một buổi (2–4 giờ) cho lần chạy đầu tiên, đừng làm dở dang rồi tắt máy giữa chừng — trạng thái actor/backup SDF dễ gây nhầm lẫn nếu quay lại sau nhiều ngày.
* Đóng bớt ứng dụng nặng RAM/GPU trên Windows trước khi bật Gazebo — Gazebo + YOLO cùng lúc trong WSL2 khá tốn tài nguyên.
* Nếu dùng WSL2, đảm bảo đã cài `WSLg` hoặc một X server (VcXsrv) để thấy được cửa sổ Gazebo/MAVProxy map. Kiểm tra nhanh:

  ```bash
  echo $DISPLAY
  ```

  Nếu trống, GUI sẽ không hiện — cần cấu hình trước, đừng đợi đến bước chạy Gazebo mới phát hiện.
* Mở sẵn 5 terminal riêng biệt (đặt tên tab rõ ràng: Gazebo / ArduPilot / Camera bridge / YOLO / PID) để không nhầm lẫn khi copy-paste lệnh.
* Ghi lại đường dẫn thật của các thư mục (`~/ardupilot`, `~/ardupilot_gazebo`, `~/phase2_ws`) vào một file note riêng — nhiều lỗi phát sinh chỉ vì gõ sai path.

## 63.2. Thứ tự thao tác gợi ý cho một buổi làm việc

1. Mở terminal 1, chạy phần kiểm tra điều kiện đầu vào (Mục 1). Nếu có bước nào FAIL, dừng lại xử lý trước, đừng cố "làm tắt".
2. Làm Bước A (A0 → A7) một mình, **không** mở Gazebo song song với việc sửa file SDF — sửa xong rồi mới mở.
3. Sau A3 (actor chuyển động được xác nhận), tắt hẳn Gazebo (`Ctrl+C`, đợi terminal trả về prompt) trước khi làm A4 trở đi, để tránh giữ process ngầm gây xung đột topic khi mở lại.
4. Làm Bước A6–A7 (cài dependency, cài YOLO) trong lúc Gazebo đang tắt — cài đặt package thường mất vài phút, không cần Gazebo chạy nền.
5. Làm Bước B (workspace, node YOLO) hoàn toàn ở dạng code trước, **build thử bằng `colcon build`** ngay cả khi chưa có Gazebo chạy — lỗi cú pháp Python/CMake nên bắt sớm, không nên để tới lúc chạy full stack mới phát hiện.
6. Chỉ khi B8 build sạch, mới mở lại Gazebo (A3) + camera bridge (B0) để test B9.
7. Làm Bước C (gimbal bridge) độc lập với YOLO — không cần chạy YOLO khi test C3, chỉ cần Gazebo + bridge gimbal.
8. Viết controller (Bước D) và build (D8) trong lúc chưa cần bật full stack.
9. Chỉ đến Bước E mới bật đủ 5 terminal cùng lúc. Bật theo đúng thứ tự E1 → E2 → E3 → E4 → E5 → E6, đợi mỗi terminal ổn định (không còn log lỗi đỏ) rồi mới mở terminal tiếp theo.
10. Làm Bước F (test từng tầng) **trước khi** bật PID — đừng vội bật D7 nếu F0–F4 chưa pass, vì lúc đó rất khó biết lỗi nằm ở detector hay ở controller.
11. Calibrate PID (Bước G) là bước tốn thời gian nhất — nên dành hẳn một phiên riêng, đừng làm chung buổi với việc debug code.
12. Đánh giá định lượng (Bước H) nên chạy ít nhất 2–3 lần độc lập (actor đi hết một vòng trajectory) rồi lấy trung bình, đừng kết luận từ một lần chạy duy nhất.

## 63.3. Lưu ý thao tác cụ thể

* **Copy lệnh cẩn thận với ký tự đặc biệt**: các lệnh bridge có ký tự `@`, `[`, `]` dễ bị terminal hoặc trình soạn thảo tự động đổi dấu ngoặc/encoding khi copy từ tài liệu. Nếu bridge báo lỗi parse ngay khi chạy, việc đầu tiên nên kiểm tra là gõ lại thủ công thay vì copy.
* **Biến môi trường không tồn tại giữa các terminal khác nhau**: `export CAMERA_GZ_TOPIC=...` chỉ có hiệu lực trong terminal đã export. Nếu mở terminal mới để chạy bridge, phải export lại hoặc ghi hẳn giá trị cứng vào lệnh.
* **`source` lại sau mỗi lần `colcon build`**: quên `source ~/phase2_ws/install/setup.bash` sau khi build lại là lỗi rất hay gặp, biểu hiện là `ros2 run` báo "package not found" dù build thành công.
* **Actor không load do URL mesh**: file mesh trong actor SDF tải từ `fuel.gazebosim.org` qua mạng — nếu mạng chậm hoặc bị chặn, actor có thể load rất lâu hoặc lỗi timeout. Nên thử load thế giới một lần độc lập trước, kiên nhẫn đợi, đừng vội kết luận SDF sai.
* **Camera FPS thấp là điều đã biết trước**: đừng mất thời gian cố "sửa cho nhanh hơn" trong Phase 2 — tài liệu đã quy định rõ đây là việc của task tối ưu riêng.
* **Kiểm tra `ros2 topic hz` đủ lâu**: chạy tối thiểu 10–15 giây để có số liệu ổn định, chạy 1–2 giây rồi Ctrl+C sẽ cho số liệu không đáng tin.
* **Khi PID làm gimbal "giật" mạnh lúc mới bật**: đây thường là do integral đã tích lũy từ trước khi target chưa xuất hiện — nhớ kiểm tra logic reset integral khi chuyển từ LOST sang TRACKING.
* **Đừng tự thêm code "tiện tay"**: khi đang debug, rất dễ nảy ra ý muốn chỉnh nhanh SDF hoặc đổi tracker "thử xem sao". Theo đúng quy tắc Mục 2, mọi thay đổi ngoài phạm vi phải dừng lại và ghi chú, không sửa trực tiếp rồi quên mất đã sửa gì.

## 63.4. Checklist nhanh cho một phiên làm việc (dán lên màn hình)

```text
[ ] Đã đóng ứng dụng nặng, còn đủ RAM/CPU
[ ] $DISPLAY hoạt động (nếu cần GUI)
[ ] Đã ps aux kiểm tra process cũ, đã pkill nếu có
[ ] Terminal 1–5 đã đặt tên rõ ràng
[ ] Đã source ROS + workspace ở MỌI terminal trước khi chạy ros2 run
[ ] Đã ghi log path thật (không đoán từ trí nhớ)
[ ] Không sửa SDF ngoài phạm vi cho phép
[ ] Có ghi số liệu (FPS, error, latency) chứ không chỉ đánh giá bằng mắt
```

## 63.5. Khi bị kẹt, làm theo thứ tự này (không nhảy bước)

1. Đọc lại đúng log lỗi, chép nguyên văn dòng lỗi cuối cùng.
2. Kiểm tra process cũ có đang chiếm topic/cổng không (`ps aux | grep ...`).
3. Kiểm tra đã `source` đúng file `setup.bash` chưa, ở đúng terminal đang chạy lệnh lỗi.
4. Kiểm tra biến môi trường (`echo $CAMERA_GZ_TOPIC` …) có giá trị đúng không.
5. Nếu vẫn không rõ nguyên nhân: dừng, không đoán mò sửa nhiều thứ cùng lúc — quay lại đúng Mục 62, ghi trạng thái `BLOCKED` và mô tả 4 mục (lệnh đã chạy / log lỗi / nguyên nhân dự kiến / bước cần cho phép) trước khi thử tiếp.