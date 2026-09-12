# Simulation realism và failure testing

Thư mục này chứa profile làm suy giảm sensor có kiểm soát và công cụ failure injection. Mục tiêu là kiểm tra hành vi của autonomy stack khi dữ liệu không hoàn hảo, không phải làm simulation trông “thật” bằng các hiệu ứng không đo được.

## Thành phần

| File | Vai trò |
| --- | --- |
| realism.yaml | Profile delay/drop/blur/noise lặp lại được |
| vehicle_profile.yaml | Ghi thông số baseline và các giá trị phải đo trên hardware |
| inject_failure.py | Gửi PX4 failure commands qua MAVLink |
| sim_realism_node.py | Implementation nằm trong ROS 2 package |

## Camera realism path

~~~text
/camera/image_raw
  -> simulation_realism
  -> optional delay/drop/blur
  -> /simulation/camera/image
  -> yolo_detector_node
~~~

Khi chạy normal mode, use_realism=false và detector nhận trực tiếp /camera/image_raw. Khi đặt SIM_REALISM=1, launch file bật node realism và override detector input.

~~~bash
SIM_REALISM=1 ./start_stack.sh
~~~

Hoặc launch trực tiếp:

~~~bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 launch vision_tracking tracking_stack.launch.py use_realism:=true
~~~

## Parameter

| Parameter | Default | Ý nghĩa |
| --- | ---: | --- |
| camera_delay_ms | 0 | Trễ publish frame |
| camera_drop_probability | 0 | Xác suất bỏ frame |
| camera_motion_blur_pixels | 0 | Kernel Gaussian blur |
| camera_queue_depth | 8 | Số frame delayed tối đa |
| imu_noise_stddev | 0 | Noise acceleration/angular velocity |
| imu_bias | [0,0,0] | Bias ban đầu |
| imu_drift_per_s | [0,0,0] | Drift bias |
| gps_noise_m | 0 | Noise vị trí tương đương mét |
| gps_dropout_probability | 0 | Drop GPS |
| baro_noise_pa | 0 | Pressure noise |
| sensor_dropout_probability | 0 | Drop sensor chung |
| seed | 7 | Seed để test lặp lại |

Nếu imu_input, gps_input hoặc baro_input để trống, node không tạo subscription cho sensor đó.

## Test profile đúng cách

1. Chạy nominal mode và lưu baseline.
2. Chỉ bật một loại degradation.
3. Giữ world, seed, model và controller parameters cố định.
4. Ghi image rate, tracking retention, vision age, control gap và Offboard state.
5. Tăng severity theo từng bước.
6. Không tăng timeout chỉ để biến test thành PASS.

Ví dụ camera delay:

~~~yaml
simulation_realism:
  ros__parameters:
    camera_delay_ms: 80.0
    camera_drop_probability: 0.05
    camera_motion_blur_pixels: 3
    seed: 7
~~~

## PX4 failure injection

Sau khi SITL chạy:

~~~bash
python3 simulation/inject_failure.py gps off
python3 simulation/inject_failure.py gps ok
python3 simulation/inject_failure.py mavlink_signal off
~~~

Chỉ dùng trong SITL hoặc bench/HIL có quy trình an toàn. Luôn có lệnh khôi phục, timeout test và process cleanup.

## Safety ladder

1. SITL nominal.
2. SITL realism/failure injection.
3. HIL với propeller tháo rời.
4. Bench test có nguồn giới hạn dòng và kill switch.
5. Tethered low-hover trong vùng bảo vệ.
6. Geofence flight.

Không bỏ qua một gate vì các metric simulation đẹp. vehicle_profile.yaml phải phân biệt rõ thông số giả lập và số đo airframe thật.
