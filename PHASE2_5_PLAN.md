# KẾ HOẠCH THỰC NGHIỆM GIAI ĐOẠN 2.5
## 3 Checkpoints: Kiểm thử bám mục tiêu trên không (Aerial Dynamic Vision Tracking)

---

### Checkpoint 1: Drone Hover (10m) + Moving Actor
- **Mục tiêu**: Kiểm tra khả năng bù góc Pitch khi drone lên độ cao và bám theo người đi bộ tuần hoàn trên mặt đất.
- **Quy trình thực hiện**:
  1. Đảm bảo toàn bộ stack đã chạy (Gazebo, Bridges, YOLO Detector, Gimbal PID).
  2. Mở terminal ArduPilot SITL:
     ```bash
     cd ~/ardupilot/ArduCopter
     sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --add-param-file=$HOME/ardupilot_gazebo/config/gazebo-iris-gimbal.parm --console --map
     ```
  3. Trên cửa sổ lệnh MAVProxy:
     ```text
     mode guided
     arm throttle
     takeoff 10
     ```
  4. Chạy `tracking_eval` để ghi nhận dữ liệu:
     ```bash
     ros2 run vision_tracking tracking_eval --duration 30.0
     ```
- **Kỳ vọng**: Pitch gục xuống $\sim 0.6 - 1.0\text{ rad}$ (tuỳ khoảng cách), sai số trung bình $< 20\text{ px}$.

---

### Checkpoint 2: Moving Drone + Stationary Actor
- **Mục tiêu**: Kiểm tra bộ bám ByteTrack khi nền cảnh thay đổi do drone di chuyển; Gimbal phải xoay ngược chiều để giữ khóa (Lock) mục tiêu đứng yên.
- **Quy trình thực hiện**:
  1. Đặt `auto_start=false` trong `iris_runway.sdf` (hoặc chuyển actor sang vị trí cố định `pose: 5 5 0`).
  2. Khởi động lại Gazebo và stack.
  3. Cho drone cất cánh lên 10m (`takeoff 10`).
  4. Điều khiển drone bay tịnh tiến ngang/dọc với tốc độ $1-2\text{ m/s}$ (qua lệnh MAVProxy `velocity` hoặc click-to-move trên Map).
  5. Đánh giá tính ổn định của `track_id` và khả năng phản xạ bù góc của Gimbal.

---

### Checkpoint 3: Moving Drone + Moving Actor (Full Dynamic Test)
- **Mục tiêu**: Đánh giá toàn diện vòng lặp kín trong điều kiện cả 2 thực thể cùng chuyển động tương đối.
- **Quy trình thực hiện**:
  1. Khôi phục quỹ đạo di chuyển của actor.
  2. Cho drone cất cánh lên 10m và bay cắt ngang/song song quỹ đạo người.
  3. Chạy `tracking_eval` đo đạc đầy đủ các thông số: Mean Error, Max Error, Settling Time, Track loss count.
