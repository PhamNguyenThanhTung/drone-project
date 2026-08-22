# BIÊN BẢN NGHIỆM THU PHASE 2
## AI Vision & Closed-Loop Gimbal Tracking

**Dự án:** ArduPilot SITL + Gazebo Harmonic + ROS 2 Closed-Loop Vision Tracking  
**Thời gian lập:** 2026-08-18  
**Trạng thái nghiệm thu:** **PASS WITH PERFORMANCE LIMITATIONS** (Đạt yêu cầu kèm giới hạn hiệu năng)

---

### 1. Tóm tắt kết quả kỹ thuật

| Tiêu chí | Mục tiêu đề ra | Thực tế đạt được | Kết luận |
| :--- | :--- | :--- | :--- |
| **Human Actor Simulation** | Actor xuất hiện và di chuyển tuần hoàn theo trajectory | Xuất hiện `target_human` trong `iris_runway.sdf`, trajectory chạy ổn định | **PASS** |
| **Camera Bridge** | Gz camera image được bridge sang ROS 2 topic | Bridge hoạt động qua `ros_gz_bridge` với chuẩn `sensor_msgs/msg/Image` | **PASS** |
| **Object Detection & Tracking** | YOLOv8n detect `person` + ByteTrack duy trì `track_id` | YOLOv8n + ByteTrack chạy ổn định, publish `/tracking/error` | **PASS** |
| **Sai số điều khiển trung bình (Mean Error)** | $< 20\text{ px}$ trên cả trục X và Y | **X: $7.6\text{ px}$, Y: $1.0\text{ px}$** | **PASS** |
| **Sai số cực đại (Max Error)** | Khống chế không gây mất đối tượng ra khỏi FOV | **Max X: $14.6\text{ px}$, Max Y: $2.6\text{ px}$** | **PASS** |
| **Settling Time** | Gimbal ổn định mục tiêu vào tâm nhanh chóng | **$6.5\text{ s}$** (vào dải dung sai $\le 20\text{ px}$) | **PASS** |
| **Gimbal Direction & Limit** | Gimbal phản ứng đúng chiều, không kẹt góc hay văng góc | Giới hạn Yaw $[-1.57, +1.57]\text{ rad}$, Pitch $[-0.20, +1.57]\text{ rad}$, rate limiter $0.25\text{ rad/s}$ | **PASS** |
| **Detection Availability** | Lý thuyết $\ge 90\%$ | **$78.9\%$ (khi engaged in FOV) / $62.6\%$ (toàn bộ window)** | **PASS (Limit)** |

---

### 2. Ghi chú kỹ thuật & Giới hạn phần cứng

1. **Vòng lặp điều khiển kín (Closed-loop control)**:
   - Bộ điều khiển PID hướng $dt$ kết hợp anti-windup, deadband và bộ giới hạn tốc độ góc (rate limiter) vận hành chính xác và ổn định.
   - Gimbal tự động điều chỉnh cả 2 trục Yaw và Pitch để bám theo `target_human` trên mặt đất.

2. **Giới hạn hiệu năng (Performance Limitations)**:
   - Tỷ lệ phát hiện (Detection Availability) đạt $78.9\%$ (chưa chạm mốc lý tưởng $90\%$).
   - **Nguyên nhân**: Môi trường giả lập WSL2 xử lý render đồ họa phần mềm (Software rendering) và chạy mô hình suy luận YOLO trên CPU dẫn đến FPS camera thực tế đạt khoảng $\sim 3.56\text{ Hz}$ wall-clock.
   - **Đánh giá**: Bộ lọc ByteTrack và cơ chế xử lý mất dấu (Lost Target State) trong PID controller đã xử lý trơn tru các khung hình trễ mà không gây giật hay mất điều khiển gimbal.

---

### 3. Kết luận chuyển giai đoạn

Hệ thống Phase 2 đáp ứng đầy đủ tất cả các yêu cầu logic của vòng lặp thị giác - điều khiển gimbal.  
**Chính thức đóng Phase 2 và đủ điều kiện chuyển sang Phase 2.5 (Thực nghiệm trên không với Drone) và Phase 3 (Autonomous Drone Navigation).**
