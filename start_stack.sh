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
echo "[0/5] Rebuild ROS 2 workspace & ensure PX4 airframe patch..."
if [[ -f "${PROJECT_DIR}/scripts/apply_px4_patch.sh" ]]; then
  "${PROJECT_DIR}/scripts/apply_px4_patch.sh" --apply "${PX4_DIR}" || true
fi
(cd "${PROJECT_DIR}/ros2_ws" && colcon build --symlink-install)
if [[ -f "${PROJECT_DIR}/ros2_ws/install/setup.bash" ]]; then
  source "${PROJECT_DIR}/ros2_ws/install/setup.bash"
fi

export GZ_VERSION=harmonic
# person_tracking_path makes the actor repeat the complete paved walkway loop
# (including turns and the return leg). Override with WORLD_NAME=... when a
# focused camera or regression scenario is required.
export WORLD_NAME="${WORLD_NAME:-person_tracking_path}"
export PX4_GZ_WORLD="${WORLD_NAME}"
export PX4_SIM_MODEL="${PX4_SIM_MODEL:-x500_flow}"
export PX4_GZ_TARGET="${PX4_GZ_TARGET:-gz_${PX4_SIM_MODEL}}"
export PX4_GZ_MODEL_NAME="${PX4_GZ_MODEL_NAME:-x500_0}"
# WSL exposes the Windows host through the default route. Keep an explicit
# override available for bridged networking or a manually selected host.
export PX4_GCS_IP="${PX4_GCS_IP:-$(ip route 2>/dev/null | grep default | awk '{print $3}')}"
# Keep the primary PX4 MAVLink instance discoverable by Windows QGroundControl.
# This remains useful for PX4 builds whose startup scripts honor MAV_*_BROADCAST;
# the explicit endpoint below handles WSL NAT where broadcast is not delivered.
export PX4_PARAM_MAV_0_BROADCAST="${PX4_PARAM_MAV_0_BROADCAST:-1}"
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

# Dọn dẹp process cũ thuộc stack này
pkill -TERM -x px4 2>/dev/null || true
pkill -TERM -f "tracking_stack.launch.p[y]" 2>/dev/null || true
pkill -TERM -f "gz.*${WORLD_NAME}" 2>/dev/null || true
pkill -TERM -f "sleep infinity" 2>/dev/null || true
sleep 0.5
pkill -9 -x px4 2>/dev/null || true
pkill -9 -f "tracking_stack.launch.p[y]" 2>/dev/null || true
pkill -9 -f "gz.*${WORLD_NAME}" 2>/dev/null || true
pkill -9 -f "sleep infinity" 2>/dev/null || true

kill_process_tree() {
  local parent_pid="$1"
  local sig="${2:-TERM}"
  if [[ -z "$parent_pid" ]] || ! kill -0 "$parent_pid" 2>/dev/null; then
    return 0
  fi
  local children
  children=$(pgrep -P "$parent_pid" 2>/dev/null || true)
  for child in $children; do
    kill_process_tree "$child" "$sig"
  done
  kill -"$sig" "$parent_pid" 2>/dev/null || true
}

cleanup_pid() {
  local pid="$1"
  local name="$2"
  if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
    echo "Dừng $name (PID $pid và tiến trình con)..."
    kill_process_tree "$pid" TERM
    local count=0
    while kill -0 "$pid" 2>/dev/null && (( count < 10 )); do
      sleep 0.2
      ((count++))
    done
    if kill -0 "$pid" 2>/dev/null; then
      kill_process_tree "$pid" KILL
    fi
  fi
}

cleanup() {
  echo "--- Đang dọn dẹp hệ thống ---"
  cleanup_pid "${STACK_PID:-}" "ROS Tracking Stack"
  cleanup_pid "${GZ_GUI_PID:-}" "Gazebo GUI"
  cleanup_pid "${GZ_PID:-}" "Gazebo Server"
  cleanup_pid "${PX4_PID:-}" "PX4 SITL Pipeline"
  cleanup_pid "${QGC_PID:-}" "QGroundControl"

  # Targeted compatibility fallback for lingering stack components
  pkill -TERM -f "tracking_stack.launch.p[y]" 2>/dev/null || true
  pkill -TERM -f "gz.*${WORLD_NAME}" 2>/dev/null || true
  pkill -TERM -x px4 2>/dev/null || true
  pkill -TERM -f "sleep infinity" 2>/dev/null || true
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

wait_for_ready() {
  local name="$1"
  local pid="$2"
  local log_file="$3"
  local timeout="$4"
  local check_cmd="$5"

  local start_t
  start_t=$(date +%s)
  while true; do
    if ! kill -0 "$pid" 2>/dev/null; then
      echo "LỖI: $name (PID $pid) đã dừng đột ngột. Xem log: $log_file" >&2
      tail -25 "$log_file" >&2 || true
      exit 1
    fi
    if eval "$check_cmd" >/dev/null 2>&1; then
      echo "  -> [READY] $name (PID $pid) đã sẵn sàng."
      return 0
    fi
    local now_t
    now_t=$(date +%s)
    if (( now_t - start_t >= timeout )); then
      echo "CẢNH BÁO: Quá thời gian chờ $timeout giây cho $name (PID $pid); tiếp tục với trạng thái degraded." >&2
      return 1
    fi
    sleep 0.5
  done
}

configure_windows_mavlink() {
  local px4_bin_dir="${PX4_DIR}/build/px4_sitl_default/bin"
  local mavlink_client="${px4_bin_dir}/px4-mavlink"

  if [[ -z "${PX4_GCS_IP}" ]]; then
    echo "WARNING: Không xác định được IP host Windows từ default route; bỏ qua MAVLink UDP 14550." >&2
    return 1
  fi
  if [[ ! -x "${mavlink_client}" ]]; then
    echo "WARNING: Không tìm thấy PX4 MAVLink client: ${mavlink_client}" >&2
    return 1
  fi

  # PX4's POSIX rcS starts its default GCS instance on an internal port. Add a
  # dedicated endpoint so telemetry is sent directly to Windows QGroundControl:
  # mavlink start -x -u 14550 -r 4000000 -t "$PX4_GCS_IP"
  echo "Cấu hình MAVLink GCS: UDP 14550 -> ${PX4_GCS_IP}:14550"
  if ! PATH="${px4_bin_dir}:${PATH}" "${mavlink_client}" start \
      -x -u 14550 -r 4000000 -t "${PX4_GCS_IP}" \
      > /tmp/px4_gcs_mavlink.log 2>&1; then
    echo "WARNING: PX4 không tạo được MAVLink endpoint tới ${PX4_GCS_IP}:14550." >&2
    tail -20 /tmp/px4_gcs_mavlink.log >&2 || true
    return 1
  fi
  return 0
}

launch_qgroundcontrol() {
  # QGC runs on Windows in this setup.  WSL only launches it when explicitly
  # requested with LAUNCH_QGC=1 and a Windows executable path.
  if [[ "${LAUNCH_QGC:-0}" != "1" ]]; then
    echo "QGroundControl: WSL launch disabled (use Windows QGC; LAUNCH_QGC=0)"
    return 0
  fi

  local qgc_path="${QGC_WINDOWS_PATH:-${QGC_PATH:-}}"
  local qgc_pid
  local -a qgc_cmd
  if [[ ! -f "${qgc_path}" ]]; then
    echo "WARNING: Windows QGroundControl executable not found: ${qgc_path:-<unset>}" >&2
    echo "         Set QGC_WINDOWS_PATH=/mnt/c/.../QGroundControl.exe" >&2
    echo "         or set LAUNCH_QGC=0 and start QGC manually on Windows." >&2
    return 0
  fi
  echo "Khởi động QGroundControl: ${qgc_path}"
  qgc_cmd=("${qgc_path}")
  if [[ "${qgc_path,,}" == *.appimage ]]; then
    echo "ERROR: WSL QGroundControl AppImage is disabled; use the Windows .exe instead." >&2
    QGC_PID=""
    return 0
  fi
  "${qgc_cmd[@]}" > /tmp/qgc.log 2>&1 &
  qgc_pid=$!
  QGC_PID="${qgc_pid}"
  sleep 2
  if ! kill -0 "${qgc_pid}" 2>/dev/null; then
    echo "ERROR: QGroundControl exited during startup." >&2
    tail -30 /tmp/qgc.log >&2 || true
    if rg -q 'GLIBC_|GLIBCXX_' /tmp/qgc.log 2>/dev/null; then
      echo "         The AppImage is incompatible with this WSL runtime (current glibc: $(ldd --version 2>&1 | head -1))." >&2
      echo "         Install a compatible QGC build, upgrade WSL, or use Windows QGC." >&2
    fi
    # QGC is a monitoring aid; keep the flight stack usable when its GUI
    # binary cannot run, but make the failure explicit in the console/log.
    QGC_PID=""
    return 0
  fi
  echo "QGroundControl started (PID ${qgc_pid}); visual map/GCS is available."
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
wait_for_ready "Gazebo Harmonic (/clock)" "$GZ_PID" /tmp/gz_sim.log 20 "gz topic -l 2>/dev/null | grep -q '/clock'"
if [[ "${HEADLESS:-0}" != "1" ]]; then
  gz sim -g > /tmp/gz_gui.log 2>&1 &
  GZ_GUI_PID=$!
fi

( sleep infinity | (cd "${PX4_DIR}" && make px4_sitl "${PX4_GZ_TARGET}") ) > /tmp/px4_sim.log 2>&1 &
PX4_PID=$!
wait_for_ready "PX4 SITL (MAVLink 14540)" "$PX4_PID" /tmp/px4_sim.log 30 "grep -q 'Ready for takeoff!' /tmp/px4_sim.log || grep -q 'mavlink start' /tmp/px4_sim.log || ss -ulpn 2>/dev/null | grep -q 14540"
MAVLINK_FORWARDING_OK=0
if configure_windows_mavlink; then
  MAVLINK_FORWARDING_OK=1
fi
if [[ "${MAVLINK_FORWARDING_OK}" == "1" ]]; then
  echo "MAVLink GCS telemetry: UDP 14550 -> ${PX4_GCS_IP}:14550 (Windows QGroundControl)."
elif [[ -z "${PX4_GCS_IP}" ]] && rg -q 'remote port 14550' /tmp/px4_sim.log 2>/dev/null; then
  echo "MAVLink GCS telemetry: UDP remote port 14550 (PX4 startup endpoint)."
else
  echo "WARNING: PX4 chưa xác nhận MAVLink endpoint UDP 14550." >&2
fi
if [[ "${MAVLINK_FORWARDING_OK}" != "1" ]] && rg -q 'MAVLink only on localhost' /tmp/px4_sim.log 2>/dev/null; then
  echo "WARNING: PX4 reports MAVLink as localhost-only; check MAV_0_BROADCAST in PX4." >&2
fi

echo "[2/5] Khởi động ROS 2 Tracking Stack (Bridge, YOLO, MotionArbiter, HUD)..."
LAUNCH_REALISM=$([[ "${SIM_REALISM:-0}" == "1" ]] && echo "true" || echo "false")
LAUNCH_HUD=$([[ "${SHOW_HUD:-1}" == "1" ]] && echo "true" || echo "false")

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

TAKEOFF_ALT="${TAKEOFF_ALT:-3.8}"
printf -v TAKEOFF_ALT_ROS '%.6f' "${TAKEOFF_ALT}"
echo "Auto takeoff altitude: ${TAKEOFF_ALT_ROS} m"

ros2 launch vision_tracking tracking_stack.launch.py \
  use_bridge:=true \
  use_realism:="${LAUNCH_REALISM}" \
  show_hud:="${LAUNCH_HUD}" \
  yolo_device:="${YOLO_DEVICE}" \
  takeoff_alt:="${TAKEOFF_ALT_ROS}" > /tmp/ros_tracking_stack.log 2>&1 &
STACK_PID=$!
ln -sf /tmp/ros_tracking_stack.log /tmp/motion_arbiter.log
ln -sf /tmp/ros_tracking_stack.log /tmp/yolo.log
ln -sf /tmp/ros_tracking_stack.log /tmp/ros_bridge.log
ln -sf /tmp/ros_tracking_stack.log /tmp/camera_hud.log

wait_for_ready "ROS-Gazebo bridge (/camera/image_raw)" "$STACK_PID" /tmp/ros_tracking_stack.log 20 "ros2 topic list --no-daemon 2>/dev/null | grep -q '/camera/image_raw'"
wait_for_ready "YOLO detector (/tracking/error)" "$STACK_PID" /tmp/ros_tracking_stack.log 30 "ros2 topic list --no-daemon 2>/dev/null | grep -q '/tracking/error'"
wait_for_ready "MotionArbiter (/tracking/control_health)" "$STACK_PID" /tmp/ros_tracking_stack.log 25 "ros2 topic list --no-daemon 2>/dev/null | grep -q '/tracking/control_health'"

echo "[3/5] Cấu hình QGroundControl..."
launch_qgroundcontrol

echo "======================================================="
echo "   PX4 AUTOPILOT + GAZEBO HARMONIC SẴN SÀNG!            "
echo "======================================================="
echo "   - Gazebo Harmonic      [PID: ${GZ_PID}]  READY (world: ${WORLD_NAME})"
echo "   - PX4 SITL Autopilot   [PID: ${PX4_PID}] READY (MAVLink 14540)"
echo "   - ROS Tracking Stack   [PID: ${STACK_PID}] READY (/tracking/control_health)"
if [[ -n "${QGC_PID:-}" ]] && kill -0 "${QGC_PID}" 2>/dev/null; then
  echo "   - QGroundControl       [PID: ${QGC_PID}] CONNECTED (UDP 14550)"
else
  echo "   - QGroundControl       CHƯA CHẠY (xem cảnh báo ở trên)"
fi
echo "======================================================="
echo "Nhấn Ctrl-C để dừng toàn bộ stack."
wait
