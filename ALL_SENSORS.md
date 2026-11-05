# All Sensors Quick Start

This starts the STL-27L lidar, OAK-D Pro, ZED 2i, CPU Foundation Stereo,
and one RViz window. RViz displays the lidar scan, OAK raw left/right,
ZED rectified left/right, and Foundation Stereo depth. OAK RGB and depth are
disabled.

## Build once

```bash
sudo apt install ros-jazzy-depthai-ros-v3 ros-jazzy-laser-filters ros-jazzy-rviz2
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select \
  ldlidar_stl_ros2 zed_open_capture_ros foundation_stereo_ros
source install/setup.bash
```

The lidar must appear as `/dev/ldlidar`. The Foundation Stereo environment
must exist at `~/ros2_ws/.venv-ffs-cpu`.

## Launch everything

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run zed_open_capture_ros launch_all_sensors.sh
```

Press **Ctrl-C once** in that terminal to stop all nodes and RViz.

The default requests raw OAK stereo at 30 Hz and ZED HD720 stereo at 30 Hz.
OAK rectification is disabled by default to avoid its additional CPU cost.
Foundation Stereo is capped at 3 Hz to keep the CPU responsive. To request
60 Hz from both cameras:

```bash
OAK_FRAME_RATE=60 ZED_FRAME_RATE=60 \
  ros2 run zed_open_capture_ros launch_all_sensors.sh
```

To run without RViz:

```bash
START_RVIZ=false ros2 run zed_open_capture_ros launch_all_sensors.sh
```

## Manual commands

Use a separate terminal for each command after sourcing ROS and the workspace.

STL-27L lidar:

```bash
ros2 launch ldlidar_stl_ros2 stl27l.launch.py
```

OAK-D Pro raw left/right only at a requested 30 Hz (the default):

```bash
ros2 launch zed_open_capture_ros oak_stereo.launch.py \
  frame_rate:=30
```

Add ROS-rectified OAK stereo with two `image_proc` nodes:

```bash
ros2 launch zed_open_capture_ros oak_stereo.launch.py \
  frame_rate:=30 image_mode:=rectified
```

ZED stereo:

```bash
ros2 launch zed_open_capture_ros zed2i.launch.py \
  namespace:=zed2i/zed_node resolution:=HD720 frame_rate:=30 rviz:=false
```

Foundation Stereo depth (after the ZED is running):

```bash
source ~/ros2_ws/.venv-ffs-cpu/bin/activate
FAST_MODEL=~/ros2_ws/src/Fast-FoundationStereo/output_cpu_onnx/20_30_48_iters1_192x352_disp64/fast_foundationstereo.onnx
ros2 launch foundation_stereo_ros zed2i_open_capture.launch.py \
  model_path:="$FAST_MODEL" intra_op_threads:=2 \
  max_inference_rate_hz:=3.0 max_disparity_model_px:=64.0 rviz:=false
```

Combined RViz:

```bash
rviz2 -d "$(ros2 pkg prefix --share zed_open_capture_ros)/rviz/all_sensors.rviz"
```

## Published topics

| Sensor | Topic | Type / content |
|---|---|---|
| STL-27L | `/scan` | Filtered `sensor_msgs/LaserScan` |
| STL-27L | `/scan_raw` | Raw `sensor_msgs/LaserScan` |
| OAK-D | `/oak/left/image_raw` | Raw left 640x400 `bgr8` image |
| OAK-D | `/oak/right/image_raw` | Raw right 640x400 `bgr8` image |
| OAK-D, optional | `/oak/left/image_rect` | Rectified left image when `image_mode:=rectified` |
| OAK-D, optional | `/oak/right/image_rect` | Rectified right image when `image_mode:=rectified` |
| OAK-D | `/oak/left/camera_info` | Left calibration |
| OAK-D | `/oak/right/camera_info` | Right calibration |
| OAK-D | `/oak/imu/data` | IMU data |
| ZED | `/zed2i/zed_node/left/image_rect_color` | Rectified left image |
| ZED | `/zed2i/zed_node/right/image_rect_color` | Rectified right image |
| ZED | `/zed2i/zed_node/left/camera_info` | Left calibration |
| ZED | `/zed2i/zed_node/right/camera_info` | Right calibration |
| ZED | `/zed2i/zed_node/imu/data_raw` | Native-rate unfiltered IMU samples |
| ZED | `/zed2i/zed_node/imu/data_sync` | Synchronized IMU data |
| Foundation Stereo | `/foundation_stereo/depth` | `32FC1` depth image |
| Foundation Stereo | `/foundation_stereo/disparity` | Disparity image |

OAK RGB and depth topics are intentionally absent. The default raw mode omits
the rectification nodes. Set `OAK_IMAGE_MODE=rectified` for the all-sensors
script, or `image_mode:=rectified` for the launch file, to add rectified topics
while retaining the raw topics. For another supported DepthAI combination,
pass a custom file with `OAK_PARAMS_FILE=/path/file.yaml`.

## Measured integrated performance

These November 5, 2026 tests ran the lidar, OAK raw stereo, ZED HD720
rectified stereo, ZED IMU, and CPU Foundation Stereo together. Independent
rate subscribers monitored every output concurrently, approximating a heavy
visualization or recording load.

| Requested mode | OAK left/right | ZED left/right | Lidar | Foundation depth under monitoring | Result |
|---|---:|---:|---:|---:|---|
| OAK 30, ZED HD720 30 | 28.04 / 28.71 Hz | 29.85 / 29.13 Hz | 9.88 Hz | 1.25 Hz | Recommended default |
| OAK 60, ZED HD720 60 | 34.59 / 35.22 Hz | 36.70 / 35.61 Hz | 9.97 Hz | 0.78 Hz | Host and OAK link saturated |
