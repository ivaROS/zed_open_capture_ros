# Sensor Demo Quick Start

Run this setup in every terminal first:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
```

Stop the current ZED demo before starting another one. Only one process can
use the ZED camera at a time.

## 1. Launch Neuvition with RViz

```bash
ros2 launch neuvition_driver neuvition.launch.py rviz:=true
```

This displays the point cloud, range image, and depth image.

## 2. Launch ZED with RViz

```bash
ros2 launch zed_open_capture_ros zed2i.launch.py \
  resolution:=HD720 frame_rate:=30 rviz:=true
```

This displays the rectified left and right images.

## 3. Launch Foundation Stereo with RViz

Terminal 1 -- ZED camera:

```bash
ros2 launch zed_open_capture_ros zed2i.launch.py \
  namespace:=zed2i/zed_node resolution:=VGA frame_rate:=30
```

Terminal 2 -- Foundation Stereo:

```bash
source ~/ros2_ws/.venv-ffs-cpu/bin/activate
FAST_MODEL=~/ros2_ws/src/Fast-FoundationStereo/output_cpu_onnx/20_30_48_iters1_192x352_disp64/fast_foundationstereo.onnx
ros2 launch foundation_stereo_ros zed2i_open_capture.launch.py \
  model_path:="$FAST_MODEL" \
  intra_op_threads:=2 \
  max_inference_rate_hz:=3.0 \
  max_disparity_model_px:=64.0 \
  rviz:=true
```

This displays the left image, right image, and estimated depth.

## 4. Launch GF-GG with ZED

Terminal 1:

```bash
ros2 launch gfgg_slam_ros2 zed_open_capture.launch.py \
  resolution:=HD720 \
  frame_rate:=30 \
  recording_mode:=slam_mono \
  stereo_image_topic:=/zed/stereo/image_rect_mono \
  stereo_reliability:=best_effort \
  viewer:=true
```

Wait for `GF-GG ready` before moving the camera.

Optional Terminal 2 -- monitor the ZED images in RViz:

```bash
rviz2 -d "$(ros2 pkg prefix --share zed_open_capture_ros)/rviz/zed_stereo.rviz"
```

For setup, checks, USB notes, and troubleshooting, see
[`DEMO_COMMANDS.md`](DEMO_COMMANDS.md).
