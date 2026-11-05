#!/usr/bin/env bash

# Launch the STL-27L lidar, OAK-D Pro, ZED 2i, Foundation Stereo, and RViz.
# Ctrl-C shuts down every process started by this script.

set -eo pipefail

ROS2_WS="${ROS2_WS:-/home/roboslam/ros2_ws}"
ROS_DISTRO_NAME="${ROS_DISTRO_NAME:-jazzy}"
ZED_RESOLUTION="${ZED_RESOLUTION:-HD720}"
ZED_FRAME_RATE="${ZED_FRAME_RATE:-30}"
OAK_FRAME_RATE="${OAK_FRAME_RATE:-30}"
OAK_IMAGE_MODE="${OAK_IMAGE_MODE:-raw}"
FOUNDATION_RATE_HZ="${FOUNDATION_RATE_HZ:-3.0}"
FOUNDATION_THREADS="${FOUNDATION_THREADS:-2}"
FOUNDATION_MODEL="${FOUNDATION_MODEL:-${ROS2_WS}/src/Fast-FoundationStereo/output_cpu_onnx/20_30_48_iters1_192x352_disp64/fast_foundationstereo.onnx}"
START_RVIZ="${START_RVIZ:-true}"

if [[ ! -r "/opt/ros/${ROS_DISTRO_NAME}/setup.bash" ]]; then
  echo "ROS 2 ${ROS_DISTRO_NAME} was not found under /opt/ros." >&2
  exit 1
fi

# These setup scripts may reference unset variables.
set +u
source "/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
if [[ -r "${ROS2_WS}/install/setup.bash" ]]; then
  source "${ROS2_WS}/install/setup.bash"
else
  echo "Workspace setup not found: ${ROS2_WS}/install/setup.bash" >&2
  echo "Build the workspace first; see ALL_SENSORS.md." >&2
  exit 1
fi
set -u

if ! command -v setsid >/dev/null 2>&1; then
  echo "Required command not found: setsid (provided by util-linux)" >&2
  exit 1
fi

for package in ldlidar_stl_ros2 depthai_ros_driver_v3 zed_open_capture_ros foundation_stereo_ros; do
  if ! ros2 pkg prefix "${package}" >/dev/null 2>&1; then
    echo "Required ROS package not found: ${package}" >&2
    exit 1
  fi
done

case "${OAK_FRAME_RATE}" in
  30) OAK_PROFILE_NAME="oak_stereo_only.yaml" ;;
  60) OAK_PROFILE_NAME="oak_stereo_only_60hz.yaml" ;;
  *)
    echo "OAK_FRAME_RATE must be 30 or 60" >&2
    exit 1
    ;;
esac
if [[ "${OAK_IMAGE_MODE}" != rectified && "${OAK_IMAGE_MODE}" != raw ]]; then
  echo "OAK_IMAGE_MODE must be rectified or raw" >&2
  exit 1
fi
OAK_PARAMS_FILE="${OAK_PARAMS_FILE:-$(ros2 pkg prefix --share zed_open_capture_ros)/config/${OAK_PROFILE_NAME}}"
if [[ ! -r "${OAK_PARAMS_FILE}" ]]; then
  echo "OAK stereo-only configuration not found: ${OAK_PARAMS_FILE}" >&2
  exit 1
fi

if ! ros2 pkg prefix laser_filters >/dev/null 2>&1; then
  echo "Required dependency not found: laser_filters" >&2
  echo "Install it with: sudo apt install ros-${ROS_DISTRO_NAME}-laser-filters" >&2
  exit 1
fi

if [[ ! -e /dev/ldlidar ]]; then
  echo "/dev/ldlidar is missing. Reconnect the STL-27L or install its udev rule." >&2
  exit 1
fi

if [[ ! -r "${FOUNDATION_MODEL}" ]]; then
  echo "Foundation Stereo model not found: ${FOUNDATION_MODEL}" >&2
  exit 1
fi

PIDS=()
SHUTTING_DOWN=false

cleanup() {
  if [[ "${SHUTTING_DOWN}" == true ]]; then
    return
  fi
  SHUTTING_DOWN=true
  trap - INT TERM EXIT
  echo
  echo "Stopping sensor stack..."
  for pid in "${PIDS[@]}"; do
    # Each launch is its own process group. Signal the group so composable
    # containers and other launch children cannot remain attached to devices.
    kill -INT -- "-${pid}" 2>/dev/null || true
  done
  # Give ROS nodes time for their normal device-close path, then ensure no
  # remaining child in a launch group can keep a camera or serial port open.
  for _ in {1..50}; do
    local group_alive=false
    for pid in "${PIDS[@]}"; do
      if kill -0 -- "-${pid}" 2>/dev/null; then
        group_alive=true
      fi
    done
    if [[ "${group_alive}" == false ]]; then
      break
    fi
    sleep 0.1
  done
  for pid in "${PIDS[@]}"; do
    kill -TERM -- "-${pid}" 2>/dev/null || true
  done
  for pid in "${PIDS[@]}"; do
    wait "${pid}" 2>/dev/null || true
  done
}

start_process() {
  local label="$1"
  shift
  echo "Starting ${label}..."
  setsid "$@" &
  PIDS+=("$!")
}

trap cleanup INT TERM EXIT

echo "Sensor settings: OAK ${OAK_IMAGE_MODE} stereo @ ${OAK_FRAME_RATE} Hz; ZED ${ZED_RESOLUTION} @ ${ZED_FRAME_RATE} Hz; Foundation Stereo up to ${FOUNDATION_RATE_HZ} Hz"
echo "Press Ctrl-C once to stop everything."

start_process "STL-27L lidar" \
  ros2 launch ldlidar_stl_ros2 stl27l.launch.py

start_process "OAK-D Pro" \
  ros2 launch zed_open_capture_ros oak_stereo.launch.py \
    frame_rate:="${OAK_FRAME_RATE}" \
    image_mode:="${OAK_IMAGE_MODE}" \
    params_file:="${OAK_PARAMS_FILE}"

start_process "ZED 2i" \
  ros2 launch zed_open_capture_ros zed2i.launch.py \
    namespace:=zed2i/zed_node \
    resolution:="${ZED_RESOLUTION}" \
    frame_rate:="${ZED_FRAME_RATE}" \
    rviz:=false

# Foundation Stereo uses packages from this CPU-only virtual environment.
if [[ ! -r "${ROS2_WS}/.venv-ffs-cpu/bin/activate" ]]; then
  echo "Foundation Stereo environment is missing: ${ROS2_WS}/.venv-ffs-cpu" >&2
  exit 1
fi
set +u
source "${ROS2_WS}/.venv-ffs-cpu/bin/activate"
set -u

start_process "Foundation Stereo" \
  ros2 launch foundation_stereo_ros zed2i_open_capture.launch.py \
    model_path:="${FOUNDATION_MODEL}" \
    intra_op_threads:="${FOUNDATION_THREADS}" \
    max_inference_rate_hz:="${FOUNDATION_RATE_HZ}" \
    max_disparity_model_px:=64.0 \
    rviz:=false

if [[ "${START_RVIZ,,}" == true || "${START_RVIZ}" == 1 ]]; then
  RVIZ_CONFIG="$(ros2 pkg prefix --share zed_open_capture_ros)/rviz/all_sensors.rviz"
  start_process "combined RViz" rviz2 -d "${RVIZ_CONFIG}"
fi

set +e
wait -n "${PIDS[@]}"
STATUS=$?
set -e
echo "A sensor-stack process exited (status ${STATUS}); shutting down the rest."
exit "${STATUS}"
