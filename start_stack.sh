#!/bin/bash
set -e
export PYTHONUNBUFFERED=1
source /opt/ros/humble/setup.bash
source /home/tungt/phase2_ws/install/setup.bash

export GZ_VERSION=harmonic
export GZ_SIM_SYSTEM_PLUGIN_PATH=/home/tungt/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}
export GZ_SIM_RESOURCE_PATH=/home/tungt/ardupilot_gazebo/models:/home/tungt/ardupilot_gazebo/worlds:${GZ_SIM_RESOURCE_PATH:-}
export LD_LIBRARY_PATH=/usr/lib/wsl/lib:${LD_LIBRARY_PATH:-}

WORLD_NAME="${WORLD_NAME:-person_tracking_no_trees}"
WORLD_FILE="${WORLD_FILE:-/home/tungt/ardupilot_gazebo/worlds/${WORLD_NAME}.sdf}"
echo "Sử dụng Gazebo World: ${WORLD_FILE}"
CAMERA_TOPIC="${CAMERA_TOPIC:-/world/person_tracking_path/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image}"

# Dọn process cũ
pkill -9 -f "gz si[m]" 2>/dev/null || true
pkill -9 -x arducopter 2>/dev/null || true
pkill -9 -f "sim_vehicl[e]" 2>/dev/null || true
pkill -9 -f "xter[m]" 2>/dev/null || true
pkill -9 -f "mavproxy.p[y]" 2>/dev/null || true
pkill -9 -f "parameter_brid[g]e" 2>/dev/null || true
pkill -9 -f "yolo_detector_nod[e]" 2>/dev/null || true
pkill -9 -f "gimbal_controller_no[d]" 2>/dev/null || true
pkill -9 -f "vehicle_yaw_searc[h]" 2>/dev/null || true
sleep 1

cleanup() {
  pkill -TERM -f "live_camera_hu[d]" 2>/dev/null || true
  pkill -TERM -f "gimbal_controller_no[d]" 2>/dev/null || true
  pkill -TERM -f "vehicle_yaw_searc[h]" 2>/dev/null || true
  pkill -TERM -f "yolo_detector_nod[e]" 2>/dev/null || true
  pkill -TERM -f "parameter_brid[g]e" 2>/dev/null || true
  pkill -TERM -x arducopter 2>/dev/null || true
  pkill -TERM -f "sim_vehicl[e]" 2>/dev/null || true
  pkill -TERM -f "xter[m]" 2>/dev/null || true
  pkill -TERM -f "gz si[m]" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

require_alive() {
  local pid="$1"
  local name="$2"
  local log_file="$3"
  if ! kill -0 "$pid" 2>/dev/null; then
    echo "LỖI: $name đã dừng. Xem log: $log_file" >&2
    tail -20 "$log_file" >&2 || true
    exit 1
  fi
}

echo "[1/5] Khởi động ArduPilot SITL (chờ Gazebo JSON)..."
(cd /home/tungt/ardupilot && \
  Tools/autotest/sim_vehicle.py -N -v ArduCopter -f JSON \
  --add-param-file=/home/tungt/ardupilot_gazebo/config/gazebo-iris-gimbal.parm \
  --no-mavproxy --console) > /tmp/sitl.log 2>&1 &
SITL_PID=$!
sleep 2
require_alive "$SITL_PID" "ArduPilot SITL" /tmp/sitl.log

echo "[2/5] Khởi động Gazebo Harmonic..."
# Mặc định dùng GPU/WSLg renderer. Chỉ đặt USE_SOFTWARE_RENDERING=1
# trên máy không có GPU passthrough hoặc khi driver không khởi tạo được.
if [[ "${USE_SOFTWARE_RENDERING:-0}" == "1" ]]; then
  export LIBGL_ALWAYS_SOFTWARE=1
else
  unset LIBGL_ALWAYS_SOFTWARE
fi
GZ_ARGS=(-v4 -r)
if [[ -n "${GZ_PHYSICS_ENGINE:-}" ]]; then
  echo "Gazebo physics engine: ${GZ_PHYSICS_ENGINE}"
  GZ_ARGS+=(--physics-engine "${GZ_PHYSICS_ENGINE}")
fi
if [[ "${HEADLESS:-0}" == "1" ]]; then
  GZ_ARGS+=(-s)
fi
gz sim "${GZ_ARGS[@]}" \
  "${WORLD_FILE}" > /tmp/gz_sim.log 2>&1 &
GZ_PID=$!
sleep 4
require_alive "$GZ_PID" "Gazebo" /tmp/gz_sim.log

echo "[3/5] Khởi động ROS 2 parameter bridge..."
ros2 run ros_gz_bridge parameter_bridge \
  "${CAMERA_TOPIC}@sensor_msgs/msg/Image[gz.msgs.Image" \
  "/gimbal/cmd_yaw@std_msgs/msg/Float64]gz.msgs.Double" \
  "/gimbal/cmd_pitch@std_msgs/msg/Float64]gz.msgs.Double" > /tmp/ros_bridge.log 2>&1 &
BRIDGE_PID=$!
sleep 2
require_alive "$BRIDGE_PID" "ROS-Gazebo bridge" /tmp/ros_bridge.log

echo "[4/5] Khởi động YOLO Detector..."
if [[ -z "${YOLO_DEVICE:-}" ]]; then
  if python3 -c 'import torch, sys; sys.exit(0 if torch.cuda.is_available() else 1)' 2>/dev/null; then
    YOLO_DEVICE=cuda:0
  else
    YOLO_DEVICE=cpu
  fi
fi
echo "YOLO device: ${YOLO_DEVICE}"
if [[ "${SHOW_HUD:-1}" == "1" ]]; then
  YOLO_DEBUG=True
else
  YOLO_DEBUG="${YOLO_DEBUG:-False}"
fi
ros2 run vision_tracking yolo_detector_node \
  --ros-args -p image_topic:=${CAMERA_TOPIC} \
  -p model_path:=${YOLO_MODEL:-/home/tungt/phase2_ws/models/yolov8s.pt} \
  -p device:=${YOLO_DEVICE} -p infer_imgsz:=${YOLO_IMGSZ:-640} \
  -p show_debug_image:=${YOLO_DEBUG:-False} -p conf:=${YOLO_CONF:-0.50} > /tmp/yolo.log 2>&1 &
YOLO_PID=$!
sleep 2
require_alive "$YOLO_PID" "YOLO detector" /tmp/yolo.log

echo "[5/5] Khởi động Gimbal PID Controller..."
CAMERA_PITCH_RAD="${CAMERA_PITCH_RAD:-0.65}"
echo "Camera pitch: ${CAMERA_PITCH_RAD} rad (~37 deg down)"
ros2 run vision_tracking gimbal_controller_node \
  --ros-args -p init_pitch:=${CAMERA_PITCH_RAD} -p init_yaw:=0.0 \
  -p min_yaw:=0.0 -p max_yaw:=0.0 -p max_yaw_rate:=0.0 \
  -p min_pitch:=${CAMERA_PITCH_RAD} -p max_pitch:=${CAMERA_PITCH_RAD} \
  -p search_enabled:=False > /tmp/gimbal.log 2>&1 &
GIMBAL_PID=$!
sleep 1
require_alive "$GIMBAL_PID" "Gimbal controller" /tmp/gimbal.log

echo "[6/6] Khởi động vehicle yaw tracking/search..."
TAKEOFF_ALT="${TAKEOFF_ALT:-3.8}"
# ROS 2 infers `3` as INTEGER, but vehicle_yaw_search declares this as DOUBLE.
# Normalize shell input so TAKEOFF_ALT=3 and TAKEOFF_ALT=3.0 are equivalent.
printf -v TAKEOFF_ALT_ROS '%.6f' "${TAKEOFF_ALT}"
echo "Auto takeoff altitude: ${TAKEOFF_ALT_ROS} m"
python3 /home/tungt/drone-project/vehicle_yaw_search.py \
  --ros-args -p takeoff_alt:=${TAKEOFF_ALT_ROS} > /tmp/vehicle_yaw.log 2>&1 &
YAW_PID=$!
sleep 2
require_alive "$YAW_PID" "Vehicle yaw tracker" /tmp/vehicle_yaw.log

if [[ "${SHOW_HUD:-1}" == "1" ]]; then
  echo "Mở cửa sổ camera YOLO..."
  python3 /home/tungt/drone-project/live_camera_hud.py \
    --topic /tracking/debug_image > /tmp/camera_hud.log 2>&1 &
  HUD_PID=$!
  sleep 1
  require_alive "$HUD_PID" "Camera HUD" /tmp/camera_hud.log
fi

echo "Toàn bộ stack Phase 2 đã sẵn sàng!"
echo "Nhấn Ctrl-C để dừng toàn bộ stack."
wait
