# zed_open_capture_ros

ROS 2 Jazzy wrapper for ZED stereo images and IMU data using Stereolabs
[ZED Open Capture](https://github.com/stereolabs/zed-open-capture).

## Build and run

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select zed_open_capture_ros
source install/setup.bash
ros2 launch zed_open_capture_ros zed2i.launch.py
```

## Dependencies and source layout

The wrapper uses ROS 2 Jazzy, OpenCV, libusb, HIDAPI, and ZED Open Capture.
The ZED SDK and CUDA are outside its dependency set.

The manual Open Capture clone is optional. The build first looks for this
adjacent checkout:

```text
~/ros2_ws/src/zed-open-capture
```

When the checkout is absent, CMake downloads the pinned Open Capture revision
during the first build. The first build therefore needs network access for any
missing Open Capture and HIDAPI sources.

Keep `zed-open-capture/COLCON_IGNORE` when the upstream checkout is inside the
ROS workspace `src` directory. It prevents Colcon from building the upstream
plain-CMake project as a separate workspace package; `zed_open_capture_ros`
already compiles the required upstream sources. The marker is unnecessary when
Open Capture is outside the workspace or downloaded automatically by CMake.

## Resolution and frame rate

```bash
# HD720 at 60 FPS
ros2 launch zed_open_capture_ros zed2i.launch.py \
  resolution:=HD720 frame_rate:=60

# VGA at 100 FPS
ros2 launch zed_open_capture_ros zed2i.launch.py \
  resolution:=VGA frame_rate:=100
```

| `resolution` | Per-eye image size | `frame_rate` |
|---|---:|---:|
| `HD2K` | 2208x1242 | 15 |
| `HD1080` | 1920x1080 | 15, 30 |
| `HD720` | 1280x720 | 15, 30, 60 |
| `VGA` | 672x376 | 15, 30, 60, 100 |

The default is HD720 at 30 FPS. Permanent defaults are in
`config/zed2i.yaml`. Stop the running camera launch before changing modes.

## View the images

In another sourced terminal:

```bash
ros2 run rqt_image_view rqt_image_view
```

Select `/zed/left/image_rect_color` or `/zed/right/image_rect_color`.

For RViz:

```bash
ros2 run rviz2 rviz2
```

Set the fixed frame to `zed_left_camera_optical_frame`, then add `Image` and
`TF` displays.

## Topics and checks

| Topic | Type |
|---|---|
| `/zed/left/image_rect_color` | `sensor_msgs/Image` |
| `/zed/right/image_rect_color` | `sensor_msgs/Image` |
| `/zed/left/camera_info` | `sensor_msgs/CameraInfo` |
| `/zed/right/camera_info` | `sensor_msgs/CameraInfo` |
| `/zed/imu/data_raw` | `sensor_msgs/Imu` |
| `/zed/imu/data_sync` | `sensor_msgs/Imu` |

```bash
ros2 topic hz /zed/left/image_rect_color
ros2 topic hz /zed/right/image_rect_color
ros2 topic hz /zed/imu/data_raw
ros2 topic echo /zed/imu/data_sync --once

ros2 run tf2_ros tf2_echo \
  zed_left_camera_optical_frame \
  zed_right_camera_optical_frame
```

Implementation and calibration notes are in
[`docs/TECHNICAL_NOTES.md`](docs/TECHNICAL_NOTES.md).
