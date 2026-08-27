#!/bin/bash
set -e
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PX4_DIR="${PX4_DIR:-${PROJECT_DIR}/../PX4-Autopilot}"
export PYTHONUNBUFFERED=1
source /opt/ros/humble/setup.bash
# colcon copies (not symlinks) the python sources into install/, so a stale
# install silently served the pre-fix yolo_detector_node to `ros2 run` while
# every direct-source test used the new code. Rebuilding costs ~1 s; running
# yesterday's detector cost a whole debugging session.
echo "[0/5] Rebuild ROS 2 workspace (guard against a stale install)..."
(cd "${PROJECT_DIR}/ros2_ws" && colcon build --symlink-install)
if [[ -f "${PROJECT_DIR}/ros2_ws/install/setup.bash" ]]; then
  source "${PROJECT_DIR}/ros2_ws/install/setup.bash"
fi

export GZ_VERSION=harmonic
# person_tracking_approach paces its actor 3-8 m ahead of the spawn point, so a
# person is inside the 37 deg down-pitched camera's ground patch (~0.2-17 m)
# right after takeoff. The older person_tracking_no_trees actor spends most of
# its loop 20-30 m out, i.e. out of frame, which looks exactly like a broken
# detector: no boxes, nothing to click. Override with WORLD_NAME=... if needed.
export WORLD_NAME="${WORLD_NAME:-person_tracking_approach}"
export PX4_GZ_WORLD="${WORLD_NAME}"
export PX4_SIM_MODEL="${PX4_SIM_MODEL:-x500_flow}"
export PX4_GZ_TARGET="${PX4_GZ_TARGET:-gz_${PX4_SIM_MODEL}}"
export PX4_GZ_MODEL_NAME="${PX4_GZ_MODEL_NAME:-x500_0}"
export GZ_SIM_RESOURCE_PATH="${PROJECT_DIR}/gazebo/models:${PROJECT_DIR}/gazebo/worlds:${PX4_DIR}/Tools/simulation/gz/models:${PX4_DIR}/Tools/simulation/gz/worlds:${GZ_SIM_RESOURCE_PATH:-}"
export GZ_SIM_SERVER_CONFIG_PATH="${PX4_DIR}/src/modules/simulation/gz_bridge/server.config"
export GZ_SIM_SYSTEM_PLUGIN_PATH="${PX4_DIR}/build/px4_sitl_default/src/modules/simulation/gz_plugins:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export LD_LIBRARY_PATH="${PX4_DIR}/build/px4_sitl_default/src/modules/simulation/gz_plugins:/usr/lib/wsl/lib:${LD_LIBRARY_PATH:-}"

echo "======================================================="
echo "   KHỞI ĐỘNG HỆ THỐNG PX4 SITL + GAZEBO HARMONIC       "
echo "======================================================="
echo "Gazebo World: ${WORLD_NAME}"
echo "Quadcopter Model: ${PX4_SIM_MODEL}"
echo "PX4 SITL Target: ${PX4_GZ_TARGET}"

# Dọn dẹp process cũ
# Match only the PX4 runtime executable.  A broad `-f px[4]` also matches
# this script's `make px4_sitl` command and can kill the build before startup.
pkill -9 -x px4 2>/dev/null || true
pkill -9 -f "gz si[m]" 2>/dev/null || true
pkill -9 -f "mavproxy.p[y]" 2>/dev/null || true
pkill -9 -f "parameter_brid[g]e" 2>/dev/null || true
pkill -9 -f "yolo_detector_nod[e]" 2>/dev/null || true
pkill -9 -f "sim_realism_nod[e]" 2>/dev/null || true
pkill -9 -f "motion_arbite[r]" 2>/dev/null || true
pkill -9 -f "live_camera_hu[d]" 2>/dev/null || true
pkill -9 -f "QGroundControl" 2>/dev/null || true
sleep 1

cleanup() {
  if [[ -n "${GZ_PID:-}" ]]; then kill -TERM "$GZ_PID" 2>/dev/null || true; fi
  if [[ -n "${GZ_GUI_PID:-}" ]]; then kill -TERM "$GZ_GUI_PID" 2>/dev/null || true; fi
  pkill -TERM -f "QGroundControl" 2>/dev/null || true
  pkill -TERM -f "live_camera_hu[d]" 2>/dev/null || true
  pkill -TERM -f "motion_arbite[r]" 2>/dev/null || true
  pkill -TERM -f "yolo_detector_nod[e]" 2>/dev/null || true
  pkill -TERM -f "sim_realism_nod[e]" 2>/dev/null || true
  pkill -TERM -f "parameter_brid[g]e" 2>/dev/null || true
  if [[ -z "${GZ_PID:-}" ]]; then pkill -TERM -f "gz si[m]" 2>/dev/null || true; fi
  pkill -TERM -x px4 2>/dev/null || true
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

if [[ "${USE_SOFTWARE_RENDERING:-0}" == "1" ]]; then
  export LIBGL_ALWAYS_SOFTWARE=1
else
  unset LIBGL_ALWAYS_SOFTWARE
fi

echo "[1/5] Khởi động Gazebo Harmonic và PX4 Autopilot SITL..."
# Keep upstream PX4 untouched. Gazebo reads project-local worlds and models
# through GZ_SIM_RESOURCE_PATH, and PX4 attaches to the running world.
gz sim -r -s "${PROJECT_DIR}/gazebo/worlds/${WORLD_NAME}.sdf" > /tmp/gz_sim.log 2>&1 &
GZ_PID=$!
sleep 5
require_alive "$GZ_PID" "Gazebo Harmonic" /tmp/gz_sim.log
if [[ "${HEADLESS:-0}" != "1" ]]; then
  gz sim -g > /tmp/gz_gui.log 2>&1 &
  GZ_GUI_PID=$!
fi

(cd "${PX4_DIR}" && make px4_sitl "${PX4_GZ_TARGET}") > /tmp/px4_sim.log 2>&1 &
PX4_PID=$!
sleep 10
require_alive "$PX4_PID" "PX4 SITL + Gazebo" /tmp/px4_sim.log

echo "[2/5] Khởi động ROS 2 parameter bridge..."
# Bridge cho Camera Image
ros2 run ros_gz_bridge parameter_bridge \
  "/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image" > /tmp/ros_bridge.log 2>&1 &
BRIDGE_PID=$!
sleep 2
require_alive "$BRIDGE_PID" "ROS-Gazebo bridge" /tmp/ros_bridge.log

if [[ "${SIM_REALISM:-0}" == "1" ]]; then
  echo "[2b/5] Bật sensor/camera realism profile..."
  ros2 run vision_tracking sim_realism_node --ros-args \
    --params-file "${PROJECT_DIR}/simulation/realism.yaml" > /tmp/sim_realism.log 2>&1 &
  REALISM_PID=$!
  sleep 1
  require_alive "$REALISM_PID" "Simulation realism" /tmp/sim_realism.log
  YOLO_IMAGE_TOPIC="${YOLO_IMAGE_TOPIC:-/simulation/camera/image}"
else
  YOLO_IMAGE_TOPIC="${YOLO_IMAGE_TOPIC:-/camera/image_raw}"
fi

echo "[3/5] Khởi động YOLO Detector..."
# YOLO is intentionally CUDA-first.  Do not silently fall back to CPU: that
# hides a broken WSL/NVIDIA passthrough and makes tracking latency unpredictable.
YOLO_DEVICE="${YOLO_DEVICE:-cuda:0}"
if [[ "${YOLO_DEVICE,,}" == cuda* ]]; then
  if ! python3 -c 'import torch; raise SystemExit(0 if torch.cuda.is_available() else 1)' 2>/dev/null; then
    echo "ERROR: YOLO is configured for ${YOLO_DEVICE}, but CUDA is unavailable." >&2
    echo "       Repair the Windows NVIDIA driver/WSL GPU passthrough, then rerun start_stack.sh." >&2
    echo "       Diagnostic: python3 -c 'import torch; print(torch.cuda.is_available())'" >&2
    exit 1
  fi
  echo "RTX/CUDA detected: using ${YOLO_DEVICE}"
else
  echo "YOLO device override: ${YOLO_DEVICE}"
fi
COMPANION_CPUSET="${COMPANION_CPUSET:-}"
if [[ -n "${COMPANION_CPUSET}" ]]; then
  COMPANION_RUN=(taskset -c "${COMPANION_CPUSET}")
  echo "Companion CPU affinity: ${COMPANION_CPUSET}"
else
  COMPANION_RUN=()
fi
if [[ "${SHOW_HUD:-1}" == "1" ]]; then
  YOLO_DEBUG=True
else
  YOLO_DEBUG="${YOLO_DEBUG:-False}"
fi
"${COMPANION_RUN[@]}" ros2 run vision_tracking yolo_detector_node \
  --ros-args -p image_topic:=${YOLO_IMAGE_TOPIC} \
  -p model_path:=${YOLO_MODEL:-${PROJECT_DIR}/yolov8n.pt} \
  -p device:=${YOLO_DEVICE} -p infer_imgsz:=${YOLO_IMGSZ:-640} \
  -p max_frame_rate:=${YOLO_MAX_FPS:-0.0} \
  -p show_debug_image:=${YOLO_DEBUG:-False} -p conf:=${YOLO_CONF:-0.45} > /tmp/yolo.log 2>&1 &
YOLO_PID=$!
sleep 2
require_alive "$YOLO_PID" "YOLO detector" /tmp/yolo.log

echo "[5/5] Khởi động MotionArbiter (PX4 OFFBOARD State Machine)..."
TAKEOFF_ALT="${TAKEOFF_ALT:-3.8}"
printf -v TAKEOFF_ALT_ROS '%.6f' "${TAKEOFF_ALT}"
echo "Auto takeoff altitude: ${TAKEOFF_ALT_ROS} m"
"${COMPANION_RUN[@]}" python3 "${PROJECT_DIR}/motion_arbiter.py" \
  --ros-args -p takeoff_alt:=${TAKEOFF_ALT_ROS} -p auto_takeoff:=True -p mavlink:=udpin:0.0.0.0:14540 > /tmp/motion_arbiter.log 2>&1 &
ARBITER_PID=$!
sleep 2
require_alive "$ARBITER_PID" "MotionArbiter" /tmp/motion_arbiter.log

QGC_PATH="${QGC_PATH:-${PROJECT_DIR}/../QGroundControl.AppImage}"
if [[ "${LAUNCH_QGC:-1}" == "1" ]] && [ -x "${QGC_PATH}" ]; then
  echo "Tự động khởi động QGroundControl..."
  "${QGC_PATH}" > /tmp/qgc.log 2>&1 &
  sleep 1
fi

if [[ "${SHOW_HUD:-1}" == "1" ]]; then
  echo "Mở cửa sổ camera YOLO & HUD..."
  python3 "${PROJECT_DIR}/live_camera_hud.py" \
    --topic /tracking/debug_image > /tmp/camera_hud.log 2>&1 &
  HUD_PID=$!
  sleep 1
  require_alive "$HUD_PID" "Camera HUD" /tmp/camera_hud.log
fi

echo "======================================================="
echo "   PX4 AUTOPILOT + GAZEBO HARMONIC SẴN SÀNG!            "
echo "   QGroundControl đang chạy và tự động kết nối UDP 14550"
echo "======================================================="
echo "Nhấn Ctrl-C để dừng toàn bộ stack."
wait
