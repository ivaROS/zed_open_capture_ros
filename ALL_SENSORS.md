# All Sensors Quick Start

This starts the STL-27L lidar, OAK-D Pro, ZED 2i, and CPU Foundation Stereo.
RViz is optional. When enabled, it displays the lidar scan, OAK rectified
left/right, ZED rectified left/right, and Foundation Stereo depth. OAK RGB and
depth are disabled.

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

Press **Ctrl-C once** in that terminal to stop all nodes.

The defaults are:

- OAK: on-device rectified stereo at 30 Hz with the IR projector off, from
  `config/routemapper_oak.yaml`.
- ZED: HD720 rectified stereo at 30 Hz.
- Lidar: unfiltered `/scan` from `ldlidar_stl_ros2/routemapper_lidar.launch.py`.
- Foundation Stereo: capped at 3 Hz to keep the CPU responsive.
- RViz: off.

To start RViz as well:

```bash
START_RVIZ=true ros2 run zed_open_capture_ros launch_all_sensors.sh
```

To use the previous OAK raw stereo profile, pass it explicitly. `OAK_FRAME_RATE`
only applies to these profiles:

```bash
OAK_PARAMS_FILE="$(ros2 pkg prefix --share zed_open_capture_ros)/config/oak_stereo_only.yaml" \
  ros2 run zed_open_capture_ros launch_all_sensors.sh
```

## Record

Start the sensors first, then in another sourced terminal:

```bash
~/ros2_ws/src/zed_open_capture_ros/scripts/record_all_sensors.sh NAME
```

Press **Ctrl-C once** to stop. The output goes to
`~/ros2_ws/data/bags/<YYYY-MM-DD>/<HH-MM-SS>_NAME/`:

- `bag/`: MCAP bag with ZED and OAK compressed stereo, camera info, ZED and
  OAK IMU, ZED magnetometer, `/scan`, `/tf`, `/tf_static`,
  `/robot_description`, `/diagnostics`, and `/rosout`.
- `info/`: topic list, node parameters, ZED calibration files, repository
  commits, package versions, and `ros2 bag info`.

The ZED node publishes raw images only. The script starts two `image_transport`
republishers for JPEG (quality 95, `ZED_JPEG_QUALITY` to change) and stops
them on exit. Because the ZED images are best effort, a small fraction of
single-eye frames can be missing; pair left and right by timestamp on
playback. Extra topics can be added after the name.

## Manual commands

Use a separate terminal for each command after sourcing ROS and the workspace.

STL-27L lidar, unfiltered:

```bash
ros2 launch ldlidar_stl_ros2 routemapper_lidar.launch.py
```

STL-27L lidar with the footprint filter (`/scan_raw` and filtered `/scan`):

```bash
ros2 launch ldlidar_stl_ros2 stl27l.launch.py
```

OAK-D Pro on-device rectified stereo:

```bash
ros2 launch zed_open_capture_ros oak_stereo.launch.py \
  params_file:="$(ros2 pkg prefix --share zed_open_capture_ros)/config/routemapper_oak.yaml"
```

OAK-D Pro raw left/right only at a requested 30 Hz:

```bash
ros2 launch zed_open_capture_ros oak_stereo.launch.py \
  frame_rate:=30
```

Add ROS-rectified OAK stereo from the raw images with two `image_proc` nodes:

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

With the default settings:

| Sensor | Topic | Type / content |
|---|---|---|
| STL-27L | `/scan` | Unfiltered `sensor_msgs/LaserScan` |
| OAK-D | `/oak/left/image_rect` | Rectified left 640x400 `mono8` image |
| OAK-D | `/oak/right/image_rect` | Rectified right 640x400 `mono8` image |
| OAK-D | `/oak/left/camera_info` | Left rectified calibration |
| OAK-D | `/oak/right/camera_info` | Right rectified calibration |
| OAK-D | `/oak/imu/data` | IMU data |
| ZED | `/zed2i/zed_node/left/image_rect_color` | Rectified left image |
| ZED | `/zed2i/zed_node/right/image_rect_color` | Rectified right image |
| ZED | `/zed2i/zed_node/left/camera_info` | Left calibration |
| ZED | `/zed2i/zed_node/right/camera_info` | Right calibration |
| ZED | `/zed2i/zed_node/imu/data_raw` | Native-rate unfiltered IMU samples |
| ZED | `/zed2i/zed_node/imu/data_sync` | Synchronized IMU data |
| Foundation Stereo | `/foundation_stereo/depth` | `32FC1` depth image |
| Foundation Stereo | `/foundation_stereo/disparity` | Disparity image |

OAK RGB and depth topics are intentionally absent. The OAK rectified
`camera_info` has zero distortion, identity rectification, and the same
intrinsics for both cameras; the 75 mm baseline is in the projection matrix
of the left camera (`P[3] = -fx * baseline`), not the right one.

With the raw OAK profile, `/oak/left/image_raw` and `/oak/right/image_raw`
(640x400 `bgr8`) are published instead. Setting `OAK_IMAGE_MODE=rectified` for
the all-sensors script, or `image_mode:=rectified` for the launch file, adds
ROS-rectified topics from them. With the filtered lidar launch, `/scan_raw`
holds the unfiltered scan.

## Measured integrated performance

These November 5, 2026 tests ran the lidar, OAK raw stereo, ZED HD720
rectified stereo, ZED IMU, and CPU Foundation Stereo together. Independent
rate subscribers monitored every output concurrently, approximating a heavy
visualization or recording load.

| Requested mode | OAK left/right | ZED left/right | Lidar | Foundation depth under monitoring | Result |
|---|---:|---:|---:|---:|---|
| OAK 30, ZED HD720 30 | 28.04 / 28.71 Hz | 29.85 / 29.13 Hz | 9.88 Hz | 1.25 Hz | Recommended default |
| OAK 60, ZED HD720 60 | 34.59 / 35.22 Hz | 36.70 / 35.61 Hz | 9.97 Hz | 0.78 Hz | Host and OAK link saturated |

November 11, 2026, default settings with the recording script running: OAK
rectified left/right 30 Hz, ZED left/right 30 Hz before JPEG, lidar 10 Hz,
OAK IMU about 60 Hz (OAK on a USB 2 link), ZED IMU about 400 Hz, recording
about 11 MB/s.
