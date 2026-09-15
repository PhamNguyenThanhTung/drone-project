# Final Improvement Plan & Architecture Blueprint

**Document Version:** 1.0  
**Classification:** Post-Audit Master Blueprint (No Implementation in This Pass)  
**Target Repository:** `~/drone-project`

---

## 1. Finding Priority Classification Matrix

| Finding / Issue | Evidence in Code / Test | Risk & Severity | Priority | Dependencies | Proposed Remediation | Validation Method |
|---|---|---|---|---|---|---|
| **Wrong-Target Hijack via Auto-Fallback** | `yolo_detector_node.py:415`: `max(cands, key=area*conf)` on missing target | **Fatal Mission Failure:** Drone permanently tracks bystander after 1-frame occlusion. | **P0** | Target Handle abstraction | Replace auto area fallback with fail-closed `TARGET_UNCERTAIN` / `HOLD` policy. | Multi-person crossing scenario; verify zero false switches. |
| **Silent Lock Transfer via Proximity Remap** | `yolo_detector_node.py:462`: `_reacquire_lock()` binds nearby box if `dist_norm < 1.2` | **High Safety Risk:** Binds to crossing pedestrian without appearance or trajectory check. | **P0** | Spatial-velocity gating | Enforce velocity alignment and candidate separation margin before ID remap. | Close-proximity crossing test in Gazebo; confirm lock rejection. |
| **Observation Drops Target Identity** | `Point(x, y, area)` on `/tracking/error` has no ID, timestamp, or confidence | **Architecture Defect:** Arbiter cannot verify if input belongs to mission target. | **P0** | Observation interface | Expose versioned `/tracking/target_handle` side topic with handle ID and state. | Multi-node replay test; verify ID propagation. |
| **SITL Failsafe Timeout Unsafe for Flight** | `motion_arbiter_node.py:629`: `COM_OF_LOSS_T=5.0s` written directly to PX4 | **Catastrophic Crash Risk:** Real drone flies blind for 5s if companion stalls. | **P0** | Parameter separation | Separate SITL parameters from production flight profile (`COM_OF_LOSS_T <= 1.0s`). | Parameter audit script & comms interruption test. |
| **Zero Obstacle & Collision Sensing** | Zero forward/depth sensors in `model.sdf`; zero obstacle topics in launch | **Collision Hazard:** Drone flies directly into trees/walls if target runs behind them. | **P0** | Sensor payload / simulation model | Integrate forward depth/lidar sensor, Collision Prevention layer, and velocity clamp. | Gazebo tree obstacle test; verify hard brake at 2.0m. |
| **Severe Tracking Loss on Rapid Yaw** | Trials 1 & 4 recorded 0.0% and 3.0% retention in 180° turns | **Mission Degradation:** Drone loses sight of walking human on every corner. | **P1** | Tracker benchmark | Benchmark BoT-SORT with GMC and OC-SORT against ByteTrack on yaw-heavy scenarios. | Standardized HOTA, IDF1, and retention benchmarking. |
| **Zero Stuck Detection** | No logic compares commanded forward velocity with actual displacement | **Airframe Damage Risk:** Propellers spin into obstacles causing motor burnout / fire. | **P1** | EKF2 telemetry parsing | Implement stuck detector comparing $V_{cmd} > 0.6\text{ m/s}$ with $\Delta P < 0.3\text{m}$ over 2.5s. | Simulated tree collision test; verify stuck alert & hover. |
| **Uncalibrated Hardcoded Camera Geometry** | $f_x=133.55, f_y=178.07$ hardcoded; no `sensor_msgs/CameraInfo` | **Geometric Drift:** Inaccurate 3D ground standoff and turn-point vectors on real lens. | **P1** | OpenCV calibration | Integrate standard ROS 2 `camera_info` pipeline with polynomial distortion model. | Chessboard calibration & ground truth distance RMSE. |
| **Duplicated YAML Configurations** | `tracking_stack.yaml` duplicates 4 per-node YAML files; multiple speed aliases | **Configuration Drift:** Modifying one YAML silently leaves other nodes unaffected. | **P2** | ROS launch cleanup | Consolidate parameters into single source of truth; remove obsolete aliases. | Parameter dump diff tool before/after startup. |
| **Linter Scope Polluted by Build Artifacts** | Pytest fails with 433 flake8 & 72 pep257 errors scanning `install/` and test shims | **CI Signal Pollution:** Obscures real functional lint regressions. | **P2** | `setup.cfg` / pytest config | Restrict linter scan strictly to `ros2_ws/src/vision_tracking/vision_tracking`. | `colcon test` returns 100% green. |
| **Unmeasured End-to-End Perception Latency**| `command_latency_micro_test.py` measures only teleop, not camera-to-PX4 pipeline | **Performance Blindness:** Cannot detect CPU throttling or inference frame drops. | **P3** | Header timestamp injection | Inject header timestamps across ROS bridge, YOLO, arbiter, and MAVLink setpoints. | Automated latency profiler reporting p50, p95, p99. |

---

## 2. Complete End-to-End Target Tracking & Safety Architecture

```
                                  CAMERA
                                     │
                                     ▼
                                YOLOv8n
                         (Person Detection Boxes)
                                     │
                                     ▼
                         MULTI-OBJECT TRACKER
                    [ByteTrack Baseline / BoT-SORT + GMC]
                                     │
                                     ▼
                              TARGET SELECTION
                       [Operator Click / Persistent Auto]
                                     │
                                     ▼
                               TARGET_HANDLE
                       (Persistent Mission Identity)
                                     │
                                     ▼
                            TARGET STATE MANAGER
                                     │
                 ┌───────────────────┴───────────────────┐
                 ▼                                       ▼
         Motion Prediction                          3D Pinhole
      (Kalman Center/Velocity)                 (Ground Projection dx, dy)
                 │                                       │
                 └───────────────────┬───────────────────┘
                                     │
                                     ▼
                         2D / 3D / VELOCITY GATES
                                     │
                        ┌────────────┴────────────┐
                        ▼                         ▼
                    TRACKING                  UNCERTAIN
                        │                         │
                        │                  RECOVERY STATE
                        │               (Turn-Point / Reverse)
                        │                         │
                        │             Candidate in Gate?
                        │             ┌───────────┴───────────┐
                        │             ▼                       ▼
                        │          Unambiguous            Ambiguous
                        │             │                       │
                        │             │                  RE-ID VERIFY
                        │             │               (Cosine Sim Gate)
                        │             │                       │
                        └─────────────┼───────────────────────┘
                                      │
                                      ▼
                              TARGET CONTROLLER
                         (Restored cb6b1ae Control Law)
                                      │
                                      ▼
                            DESIRED MOTION VECTOR
                                      │
                                      ▼
                        ┌───────────────────────────┐
                        │  COLLISION & SAFETY LAYER │ ◄── [ Obstacle Sensors ]
                        │   1. Collision Prevention │     [ Forward / 360°   ]
                        │   2. Obstacle Avoidance   │     [ Distance Stream  ]
                        │   3. Stuck Detection      │
                        └─────────────┬─────────────┘
                                      │
                                      ▼
                             SAFE MOTION SETPOINT
                                      │
                                      ▼
                             MOTION ARBITER NODE
                        (Single MAVLink UDP Writer)
                                      │
                                      ▼
                               PX4 AUTOPILOT
                               (Offboard Mode)
                                      │
                                      ▼
                             PHYSICAL QUADCOPTER
```

---

## 3. Comprehensive Benchmark Matrix

### 3.1 Benchmark Configurations
- **A:** `ByteTrack` (Current baseline, no identity lock).
- **B:** `ByteTrack + Target Handle Lock` (Fail-closed on loss; no auto area fallback).
- **C:** `ByteTrack + Target Handle + Kalman Motion Prediction Gating`.
- **D:** `BoT-SORT + GMC` (Camera Motion Compensation; Appearance branch OFF).
- **E:** `Best Tracker (C or D) + 3D Pinhole Spatial Gate`.
- **F:** `Configuration E + On-Demand Re-ID` (Feature verification only on crossing / long loss).

### 3.2 Standardized Scenarios (Executed in Gazebo Harmonic)
1. **Nominal Tracking:** Target walking along clear path at $1.1\text{ m/s}$.
2. **Short Occlusion:** Target passes behind a single lamp post ($0.3\text{ s}$ loss).
3. **Medium Occlusion:** Target walks behind a tree trunk ($1.2\text{ s}$ loss).
4. **Sharp 90° Corner:** Target turns around foliage apex at $1.2\text{ m/s}$.
5. **Sharp 180° Reversal:** Target abruptly stops and reverses direction.
6. **Pedestrian Crossing (2 People):** Target crosses paths with identical-height bystander.
7. **Pedestrian Crossing (Similar Clothing):** Target crosses paths with bystander in matching colors.
8. **Under-Flight Bottom Exit:** Target runs directly under the drone camera frame.
9. **Dense Crowd (5+ People):** Target walks through a clustered group.
10. **Obstacle in Direct Path:** Tree positioned directly between drone and advancing target.
11. **Trapped Drone (Stuck Scenario):** Drone commanded forward against impassable barrier.
12. **High-Rate Camera Yaw:** Drone executes $60^\circ/\text{s}$ yaw search.
13. **Long Desertion:** Target leaves field of view for $> 10\text{ s}$.

### 3.3 Quantitative Evaluation Metrics
- **Tracking Quality:** HOTA (Higher Order Tracking Accuracy), IDF1 (Identification F1), ID Switches (IDSW).
- **Mission Identity:** Target Retention Rate (%), False Target Switch Rate (%), Reacquisition Success Rate (%), Reacquisition Latency (s).
- **Flight Control:** Standoff Distance Error RMSE (m), Lateral Tracking Error RMSE (px), Pitch Heave Oscillation Variance ($m/s^2$).
- **Safety Envelope:** Collision Prevention Stops (count), Stuck Detection Trigger Time (s), Minimum Obstacle Distance (m), Zero-Failsafe Trips.
- **Compute Efficiency:** End-to-End Latency (p50, p95, p99 in ms), Inference FPS, GPU VRAM (MB), CPU Core Load (%).

---

## 4. Multi-Phase Implementation Roadmap

```
[ Phase 0: Baseline Freezing & Instrumentation ]
  - Freeze production topics and MAVLink interfaces.
  - Implement non-intrusive latency logging across all pipeline nodes.
  - Record deterministic baseline dataset across Scenarios 1–13.

[ Phase 1: Source of Truth & Linting Cleanup ]
  - Consolidate YAML parameters into tracking_stack.yaml as single source.
  - Fix linter configuration in setup.cfg to ignore install/ and test shims.
  - Resolve duplicate speed/deadband parameter aliases.

[ Phase 2: Target Handle & Fail-Closed Lock ]
  - Implement TargetHandle and TargetStateManager inside yolo_detector_node.
  - Eliminate max(area * confidence) auto-fallback; enforce fail-closed hold.
  - Expose /tracking/target_handle side telemetry topic.

[ Phase 3: Spatial-Velocity Gating & 3D Geometry Integration ]
  - Implement Kalman 2D motion gating and 3D pinhole distance acceptance gates.
  - Validate camera calibration against Gazebo physical model and known actor ground truth.
  - Upgrade turn-point and bottom recovery to use gated validation before re-binding.

[ Phase 4: Tracker Benchmarking (BoT-SORT / OC-SORT) ]
  - Benchmark Configurations A, B, C, D on Scenarios 4, 8, 9, 12 (high yaw & turns).
  - Select winner based on HOTA/IDF1 and latency budget; integrate winner as default.

[ Phase 5: Obstacle Safety Layer & Stuck Detection ]
  - Add forward distance sensing model to simulation.
  - Implement Collision Prevention hard-stop gate and Stuck Detection algorithm.
  - Enforce strict priority: Obstacle Safety strictly overrides Target Controller.

[ Phase 6: On-Demand Re-ID (Evaluation Gate) ]
  - Evaluate if False Target Switch Rate on crossing scenarios is > 5%.
  - If yes, integrate lightweight embedding extractor triggered ONLY in REID_VERIFY.
  - If no, keep system Re-ID-free for compute and simplicity.

[ Phase 7: Sim-to-Real Hardware Validation ]
  - Calibrate physical camera (OpenCV camera_calibration); publish CameraInfo.
  - Deploy on physical companion computer (Jetson); tune PX4 COM_OF_LOSS_T <= 1.0s.
  - Execute Tier 3 Bench HIL -> Tier 4 Tethered -> Tier 5 Field Trials.
```

---

## 5. Answers to the 16 Critical Architecture Questions

### 1. Hiện tại target có nguy cơ nhảy sang người khác không?
**CÓ, NGUY CƠ CỰC KỲ CAO (P0).**  
Trong `yolo_detector_node.py` (L415), khi mất dấu mục tiêu chỉ trong 1 frame ở chế độ auto, code tự động thực thi `return max(cands, key=lambda c: c[6] * c[5])`, ngay lập tức nhảy sang người có diện tích $\times$ độ tin cậy lớn nhất trong khung hình. Ở chế độ manual lock, hàm `_reacquire_lock` (L462) cho phép nhảy lock sang người khác nếu người đó đi cắt mặt trong phạm vi 1.2 kích thước bbox (`dist_norm < 1.2`).

### 2. Track ID có đang bị dùng như identity không?
**CÓ.**  
Biến `self.target_id` và `self.manual_target_id` trong detector, cũng như `active_target_id` trong arbiter, đều trực tiếp gán bằng `track_id` số nguyên do ByteTrack phát sinh. Không có lớp đối tượng identity độc lập nào tồn tại.

### 3. Cần `target_handle` không? Nếu có, đặt ở đâu?
**BẮT BUỘC PHẢI CÓ.**  
`target_handle` là cấu trúc dữ liệu lưu trữ danh tính nhiệm vụ bền vững (persistent mission identity), lịch sử `previous_track_ids`, vector vận tốc, ma trận hiệp phương sai và vị trí 3D.  
**Vị trí đặt:** Nằm trong subsystem `TargetStateManager` thuộc node perception (`yolo_detector_node`), và được broadcast qua topic `/tracking/target_handle` cho arbiter và HUD giám sát.

### 4. ByteTrack hiện tại có đủ không? Nếu chưa, failure mode cụ thể là gì?
**CHƯA ĐỦ CHO CÁC PHA CỦA GẤP VÀ QUAY CAMERA.**  
ByteTrack hoạt động tốt khi camera di chuyển tịnh tiến êm. Tuy nhiên, nó thất bại nặng nề khi drone quay yaw nhanh (camera ego-motion lớn): kết quả kiểm thử thực tế cho thấy tỷ lệ giữ mục tiêu (retention) rơi xuống **0.0% ở kịch bản Nominal 180° Turn** và **3.0% ở kịch bản Lateral Right**. Failure mode: Kalman filter 2D của ByteTrack giả định chuyển động tịnh tiến đều trong ảnh; khi camera quay nhanh, toàn bộ pixel trượt đột ngột, khiến detection văng ra ngoài search window của Kalman.

### 5. Có cần đổi sang BoT-SORT ngay không? Hay chỉ benchmark?
**CHỈ NÊN BENCHMARK TRƯỚC, KHÔNG ĐỔI NGAY LẬP TỨC.**  
BoT-SORT tích hợp Camera Motion Compensation (GMC) là ứng viên hàng đầu để giải quyết lỗi quay yaw, nhưng nó tiêu tốn thêm tài nguyên CPU/GPU (tính toán optical flow ORB/ECC). Cần benchmark so sánh đối đầu giữa ByteTrack, BoT-SORT và OC-SORT trên cùng kịch bản để cân đối giữa HOTA và latency trước khi quyết định thay thế.

### 6. 3D Pinhole hiện tại có thực sự ảnh hưởng controller không?
**ẢNH HƯỞNG MỘT PHẦN (CHỦ YẾU LÀ TELEMETRY VÀ TURN-POINT RECOVERY).**  
Hiện tại, 3D Pinhole (`pinhole_geometry.py`) tính toán tọa độ $(dx, dy)$ mặt đất để xuất telemetry và tính toán vector bay thẳng $target\_dx = dx + 1.8\text{m}$ cho chế độ rẽ góc `ADVANCING_TO_TURN_POINT`. Tuy nhiên, vòng điều khiển bám đuổi tiến/lùi thông thường vẫn dùng pixel error $\Delta y$; các tham số diện tích `target_area_*` đã bị vô hiệu hóa khỏi luật vận tốc.

### 7. Những behavior tốt từ `cb6b1ae` nào nên restore?
- **Safe-Zone Deadband:** Vùng an toàn 50% ($|\Delta x| \le 20\text{px}, |\Delta y| \le 25\text{px}$) giữ drone đứng yên hover triệt tiêu rung lắc $\rightarrow$ **RESTORE & KEEP**.
- **Base Walking Pursuit:** Vận tốc tiến cơ bản $0.85\text{ m/s} + K_{p\_y\_boost} \times \Delta y \rightarrow$ **RESTORE & KEEP**.
- **Proportional Backing:** Lùi mềm $-0.75\text{ m/s} \rightarrow$ **RESTORE & KEEP**.
- **Sharp Turn Slowdown:** Giảm tốc tiến khi target lệch góc $|\Delta x| > 60\text{px} \rightarrow$ **RESTORE & KEEP**.
- **Bottom-Exit Straight Reverse:** Lùi thẳng không xoay yaw khi người đi sát dưới bụng $\rightarrow$ **RESTORE & KEEP**.
- **2-Stage Turn-Point Navigation:** Bay thẳng theo vector pinhole $+1.8\text{m}$ clearance rồi mới xoay yaw $\rightarrow$ **RESTORE & KEEP**.
- **Area Proxy Control ($z = w \cdot h$):** Dùng diện tích bbox làm khoảng cách $\rightarrow$ **DEPRECATE HOÀN TOÀN** (vì gây giật khi người cúi/xoay).

### 8. Lost-target recovery hiện tại có nguy hiểm không?
**CÓ NGUY HIỂM TIỀM ẨN.**  
Hiện tại khi mất mục tiêu, drone vẫn tự động thực hiện hành vi bay lùi (`BACKING_UP_TO_RECOVER`) hoặc bay tiến (`ADVANCING_TO_TURN_POINT`) trong tối đa $4.0\text{ s}$ mà **hoàn toàn không có cảm biến vật cản phía trước/phía sau**. Nếu địa hình có chướng ngại vật ngoài dự kiến, drone có thể đâm va trước khi chuyển về `STANDBY`.

### 9. Có obstacle/collision architecture chưa?
**HOÀN TOÀN CHƯA CÓ (0%).**  
Mô hình SDF chỉ có camera nghiêng và 1 cảm biến đo cao nhìn thẳng xuống đất. Không có radar, lidar, depth camera, collision prevention hay avoidance layer nào được cài đặt trong ROS 2 hay kích hoạt trong PX4.

### 10. Drone có phát hiện stuck được không?
**HIỆN TẠI KHÔNG.**  
Hệ thống không so sánh vận tốc điều khiển với dịch chuyển thực tế của EKF2. Nếu va vào cây hoặc tường, drone sẽ tiếp tục tăng ga đè vào chướng ngại vật cho đến khi cháy motor hoặc EKF2 văng lỗi. Cần bổ sung thuật toán Stuck Detection theo bản thiết kế ở mục trên.

### 11. QGroundControl hiện biết những trạng thái gì?
**RẤT ÍT VÀ SƠ SÀI.**  
QGC chỉ nhận các tin nhắn MAVLink tiêu chuẩn của PX4 (chế độ bay, pin, GPS, arm/disarm) và một số log debug dạng string. QGC hoàn toàn không biết: drone đang bám ai, mục tiêu có đang bị che khuất không, drone có đang phanh tránh vật cản hay bị kẹt (stuck) hay không.

### 12. Camera $\rightarrow$ compute $\rightarrow$ PX4 architecture hiện tại là gì?
**KIẾN TRÚC MÔ PHỎNG LOOPBACK TRÊN MỘT MÁY (SINGLE-HOST WSL2).**  
Ảnh từ Gazebo $\rightarrow$ ros_gz_bridge $\rightarrow$ ROS 2 topics $\rightarrow$ PyTorch/ByteTrack $\rightarrow$ MotionArbiter $\rightarrow$ UDP 14540 $\rightarrow$ PX4 SITL. Toàn bộ chạy cục bộ trên laptop của lập trình viên. Chưa có thiết kế hay cấu hình phần cứng onboard/offboard vật lý nào.

### 13. Latency end-to-end hiện đã biết chưa?
**CHƯA BIẾT ĐẦY ĐỦ.**  
Repository chỉ có script đo độ trễ phím teleop từ HUD tới Arbiter ($\approx 10\text{ ms}$). Toàn bộ chuỗi độ trễ thị giác từ lúc camera phơi sáng $\rightarrow$ YOLO $\rightarrow$ ByteTrack $\rightarrow$ ROS topic $\rightarrow$ Arbiter $\rightarrow$ MAVLink $\rightarrow$ PX4 EKF2 chưa từng được đo đạc và bị mất dấu do topic `/tracking/error` không có header timestamp.

### 14. Real-flight còn thiếu hardware/safety layer nào?
**THIẾU HẦU NHƯ TOÀN BỘ PHẦN CỨNG THỰC TẾ:**  
1. Máy tính nhúng companion onboard (Jetson Orin/NX).
2. Camera vật lý UVC/MIPI và driver ROS 2 kèm file calibration $K, D$.
3. Mạch hạ áp nguồn BEC và tản nhiệt chống quá nhiệt CPU/GPU.
4. Tay điều khiển RC vật lý kèm công tắc ngắt động cơ khẩn cấp (Hardware Kill Switch).
5. Cơ chế gạt công tắc RC cướp quyền điều khiển ngay lập tức (RC Takeover).
6. Cảm biến chống va chạm (LiDAR/Depth camera).
7. Cấu hình failsafe PX4 an toàn ngoài trời (`COM_OF_LOSS_T <= 1.0s` thay vì `5.0s`).

### 15. Re-ID hiện có thực sự cần chưa?
**CHƯA CẦN THIẾT Ở THỜI ĐIỂM HIỆN TẠI.**  
Ưu tiên số 1 là sửa lỗi logic mất mục tiêu (P0 fail-closed target handle), tích hợp bộ lọc không gian - vận tốc (spatial-velocity gating) và chống rung lắc camera. Re-ID chỉ nên được xem xét ở Phase 6 dưới dạng **on-demand** khi benchmark thực tế chứng minh bài toán cắt mặt (crossing) hoặc mất dấu quá 3 giây không thể giải quyết bằng hình học thuần túy.

### 16. Implementation nên làm theo thứ tự nào?
**THỨ TỰ TRIỂN KHAI CHUẨN:**
1. **Phase 0:** Đo đạc latency baseline, freeze interface.
2. **Phase 1:** Dọn dẹp cấu hình YAML duy nhất và test linter.
3. **Phase 2 (Cực kỳ quan trọng):** Cài đặt `target_handle`, xóa bỏ auto-fallback `area * conf`, chuyển sang fail-closed.
4. **Phase 3:** Hoàn thiện 3D pinhole gating và kiểm chứng ground truth.
5. **Phase 4:** Benchmark BoT-SORT + GMC vs ByteTrack trên các pha rẽ góc.
6. **Phase 5:** Tích hợp tầng an toàn Obstacle Safety, Collision Prevention và Stuck Detection.
7. **Phase 6:** Đánh giá Re-ID on-demand nếu thực sự cần.
8. **Phase 7:** Sim-to-Real hardware readiness & field testing.
