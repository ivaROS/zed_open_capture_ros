# All Sensors Quick Start

This starts the STL-27L lidar, OAK-D Pro, ZED 2i, CPU Foundation Stereo,
and one RViz window. RViz displays the lidar scan, OAK RGB/depth, ZED
left/right images, and Foundation Stereo depth.

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

The default is ZED VGA at 30 Hz because this camera is currently connected
through USB 2.0. Foundation Stereo is capped at 3 Hz to keep the CPU responsive.
Override either setting only when needed:

```bash
ZED_RESOLUTION=HD720 ZED_FRAME_RATE=30 FOUNDATION_RATE_HZ=2.0 \
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

OAK-D Pro RGB and depth:

```bash
ros2 launch depthai_ros_driver_v3 driver.launch.py \
  camera_model:=OAK-D-PRO use_rviz:=false
```

ZED stereo:

```bash
ros2 launch zed_open_capture_ros zed2i.launch.py \
  namespace:=zed2i/zed_node resolution:=VGA frame_rate:=30 rviz:=false
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

Important output topics are `/scan`, `/oak/rgb/image_raw`,
`/oak/stereo/image_raw`, and `/foundation_stereo/depth`.

USB 2.0 works at the conservative settings above, but both cameras should use
separate USB 3.x ports and USB 3.x cables/adapters for higher resolution or
greater throughput.
