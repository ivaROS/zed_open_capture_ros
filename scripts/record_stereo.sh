#!/usr/bin/env bash
set -euo pipefail

if (( $# < 2 || $# > 3 )); then
    echo "Usage: $0 {slam_mono|stereo_color} OUTPUT_BAG [NAMESPACE]" >&2
    exit 2
fi

RECORD_MODE="$1"
OUTPUT_BAG="$2"
CAMERA_NAMESPACE="${3:-zed}"
CAMERA_NAMESPACE="${CAMERA_NAMESPACE#/}"
CAMERA_NAMESPACE="${CAMERA_NAMESPACE%/}"
if [[ -n "$CAMERA_NAMESPACE" ]]; then
    TOPIC_PREFIX="/${CAMERA_NAMESPACE}"
else
    TOPIC_PREFIX=""
fi

case "$RECORD_MODE" in
    slam_mono)
        STEREO_TOPIC="$TOPIC_PREFIX/stereo/recording/image_rect_mono"
        ;;
    stereo_color)
        STEREO_TOPIC="$TOPIC_PREFIX/stereo/recording/image_rect_color"
        ;;
    *)
        echo "recording mode must be slam_mono or stereo_color" >&2
        exit 2
        ;;
esac

if [[ -e "$OUTPUT_BAG" ]]; then
    echo "output already exists: $OUTPUT_BAG" >&2
    exit 2
fi

echo "Recording atomic stereo topic $STEREO_TOPIC"
echo "Stop with Ctrl-C so MCAP metadata is finalized."
exec ros2 bag record \
    --storage mcap \
    --max-cache-size 536870912 \
    --output "$OUTPUT_BAG" \
    --topics \
    "$STEREO_TOPIC" \
    "$TOPIC_PREFIX/left/camera_info" \
    "$TOPIC_PREFIX/right/camera_info" \
    "$TOPIC_PREFIX/imu/data_sync" \
    /tf_static
