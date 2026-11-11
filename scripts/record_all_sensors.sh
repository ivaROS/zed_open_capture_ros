#!/usr/bin/env bash

# Record the sensor stack started by launch_all_sensors.sh.
#
# Usage: record_all_sensors.sh [NAME] [EXTRA_TOPIC ...]
#
# Output: ${BAG_ROOT}/<YYYY-MM-DD>/<HH-MM-SS>[_NAME]/
#   bag/   MCAP bag
#   info/  topic list, node parameters, calibration files, versions
#
# The ZED node publishes raw images only, so this script starts two
# image_transport republishers (JPEG) for the ZED pair and stops them on exit.
# OAK compressed images come from the DepthAI driver. Rectified OAK images are
# recorded when present, raw ones otherwise.
#
# Stop with Ctrl-C once so the bag is finalized.

set -eo pipefail

ROS2_WS="${ROS2_WS:-/home/roboslam/ros2_ws}"
ROS_DISTRO_NAME="${ROS_DISTRO_NAME:-jazzy}"
BAG_ROOT="${BAG_ROOT:-${ROS2_WS}/data/bags}"
ZED_NS="${ZED_NS:-/zed2i/zed_node}"
ZED_JPEG_QUALITY="${ZED_JPEG_QUALITY:-95}"
MCAP_PRESET="${MCAP_PRESET:-zstd_fast}"

NAME="${1:-}"
shift || true
EXTRA_TOPICS=("$@")

set +u
source "/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
source "${ROS2_WS}/install/setup.bash"
set -u

OUT_DIR="${BAG_ROOT}/$(date +%Y-%m-%d)/$(date +%H-%M-%S)${NAME:+_${NAME}}"
BAG_DIR="${OUT_DIR}/bag"
INFO_DIR="${OUT_DIR}/info"
if [[ -e "${OUT_DIR}" ]]; then
  echo "Output already exists: ${OUT_DIR}" >&2
  exit 2
fi
mkdir -p "${INFO_DIR}/params" "${INFO_DIR}/calibration"

REC_PID=""
REPUB_PIDS=()
STOPPING=false
cleanup() {
  if [[ "${STOPPING}" == true ]]; then
    return
  fi
  STOPPING=true
  trap - INT TERM EXIT
  echo
  echo "Stopping..."
  # A background recorder ignores SIGINT; it finalizes the bag on SIGTERM.
  if [[ -n "${REC_PID}" ]] && kill -0 "${REC_PID}" 2>/dev/null; then
    kill -TERM "${REC_PID}" 2>/dev/null || true
    for _ in {1..300}; do
      kill -0 "${REC_PID}" 2>/dev/null || break
      sleep 0.1
    done
  fi
  for pid in "${REPUB_PIDS[@]}"; do
    kill -TERM "${pid}" 2>/dev/null || true
  done
  echo "end: $(date --iso-8601=seconds)" >> "${INFO_DIR}/recording.txt"
  if [[ -f "${BAG_DIR}/metadata.yaml" ]]; then
    ros2 bag info "${BAG_DIR}" > "${INFO_DIR}/bag_info.txt" 2>&1 || true
  fi
  echo "Saved ${OUT_DIR}"
}
trap 'cleanup; exit 130' INT
trap 'cleanup; exit 143' TERM
trap cleanup EXIT

AVAILABLE="$(ros2 topic list)"
has_topic() {
  grep -qx -- "$1" <<< "${AVAILABLE}"
}

# ZED JPEG republishers
for side in left right; do
  raw="${ZED_NS}/${side}/image_rect_color"
  if has_topic "${raw}/compressed"; then
    continue
  fi
  # The ZED publisher is best effort; the republisher subscribes reliable by default.
  setsid /opt/ros/"${ROS_DISTRO_NAME}"/lib/image_transport/republish --ros-args \
    -r __node:="zed_${side}_jpeg" \
    -p in_transport:=raw -p out_transport:=compressed \
    -p "qos_overrides.${raw}.subscription.reliability:=best_effort" \
    -p "qos_overrides.${raw}.subscription.depth:=5" \
    -p out.compressed.jpeg_quality:="${ZED_JPEG_QUALITY}" \
    -r in:="${raw}" -r out/compressed:="${raw}/compressed" \
    > "${INFO_DIR}/zed_${side}_jpeg.log" 2>&1 < /dev/null &
  REPUB_PIDS+=("$!")
done

if has_topic /oak/left/image_rect; then
  OAK_IMAGE=image_rect
else
  OAK_IMAGE=image_raw
fi

TOPICS=(
  /tf
  /tf_static
  /robot_description
  /joint_states
  /diagnostics
  /rosout
  "${ZED_NS}/left/image_rect_color/compressed"
  "${ZED_NS}/right/image_rect_color/compressed"
  "${ZED_NS}/left/camera_info"
  "${ZED_NS}/right/camera_info"
  "${ZED_NS}/imu/data_raw"
  "${ZED_NS}/imu/data_sync"
  "${ZED_NS}/imu/mag"
  "/oak/left/${OAK_IMAGE}/compressed"
  "/oak/right/${OAK_IMAGE}/compressed"
  /oak/left/camera_info
  /oak/right/camera_info
  /oak/imu/data
  /scan
)
TOPICS+=("${EXTRA_TOPICS[@]}")

{
  echo "start: $(date --iso-8601=seconds)"
  echo "oak_images: ${OAK_IMAGE}"
  echo "zed_jpeg_quality: ${ZED_JPEG_QUALITY}"
  echo "mcap_preset: ${MCAP_PRESET}"
  echo "topics:"
  printf '  %s\n' "${TOPICS[@]}"
} > "${INFO_DIR}/recording.txt"

ros2 bag record \
  --storage mcap \
  --storage-preset-profile "${MCAP_PRESET}" \
  --max-cache-size 536870912 \
  --output "${BAG_DIR}" \
  --topics "${TOPICS[@]}" \
  > "${INFO_DIR}/recorder.log" 2>&1 < /dev/null &
REC_PID=$!
echo "Recording to ${BAG_DIR}"

MISSING=()
for topic in "${TOPICS[@]}"; do
  case "${topic}" in
    "${ZED_NS}"/*/image_rect_color/compressed) continue ;;
  esac
  has_topic "${topic}" || MISSING+=("${topic}")
done
if (( ${#MISSING[@]} > 0 )); then
  echo "Not published yet (recorded if they appear):"
  printf '  %s\n' "${MISSING[@]}"
fi

# Collected while recording.
echo "${AVAILABLE}" > "${INFO_DIR}/topics.txt"
for node in /oak /zed2i/zed_node/zed_open_capture /STL27L; do
  # The STL27L node does not answer parameter requests.
  timeout 5 ros2 param dump "${node}" > "${INFO_DIR}/params/$(echo "${node#/}" | tr / _).yaml" 2>&1 || true
done
cp "${HOME}"/zed/settings/SN*.conf "${INFO_DIR}/calibration/" 2>/dev/null || true
cp "$(ros2 pkg prefix --share zed_open_capture_ros)/config/zed2i.yaml" "${INFO_DIR}/calibration/" 2>/dev/null || true
cp "$(ros2 pkg prefix --share ldlidar_stl_ros2)/config/laser_filter_config.yaml" "${INFO_DIR}/calibration/" 2>/dev/null || true
{
  for repo in "${ROS2_WS}"/src/*/; do
    if [[ -d "${repo}/.git" ]]; then
      echo "$(basename "${repo}") $(git -C "${repo}" rev-parse --short HEAD) dirty=$(git -C "${repo}" status --porcelain | wc -l)"
    fi
  done
  dpkg-query -W -f='${Package} ${Version}\n' \
    "ros-${ROS_DISTRO_NAME}-depthai-ros-driver-v3" \
    "ros-${ROS_DISTRO_NAME}-image-transport" \
    "ros-${ROS_DISTRO_NAME}-compressed-image-transport" \
    "ros-${ROS_DISTRO_NAME}-rosbag2-storage-mcap" 2>/dev/null || true
} > "${INFO_DIR}/versions.txt"

echo "Press Ctrl-C once to stop."
set +e
wait "${REC_PID}"
set -e
