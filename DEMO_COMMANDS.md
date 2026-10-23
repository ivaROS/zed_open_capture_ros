# Live Sensor and Stereo Demo Commands

This guide is for the native ROS 2 Jazzy workspace on this machine. Each
numbered demo is independent. Stop the previous demo with `Ctrl-C` before
starting another one: only one process can own the ZED camera device at a
time.

## Common setup

Run this in every new terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
```

After pulling changes that add these RViz configurations, build once:

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select \
  neuvition_driver zed_open_capture_ros foundation_stereo_ros
source install/setup.bash
```

The Foundation Stereo terminal also needs its Python environment:

```bash
source ~/ros2_ws/.venv-ffs-cpu/bin/activate
```

## 1. Neuvition: point cloud, depth, and range image

First make sure the lidar is powered and reachable:

```bash
ping -c 3 192.168.1.101
```

Start all three outputs and RViz:

```bash
ros2 launch neuvition_driver neuvition.launch.py \
  publish_pointcloud:=true \
  publish_range_image:=true \
  publish_depth_image:=true \
  rviz:=true
```

The RViz layout uses `neuvition` as its fixed frame and shows
`/neuvition_cloud`, `/neuvition_range`, and `/neuvition_depth`. Both images
are `32FC1` values in metres; `NaN` pixels are invalid returns.

Check the streams with:

```bash
ros2 topic hz /neuvition_cloud
ros2 topic hz /neuvition_range
ros2 topic hz /neuvition_depth
```

## 2. ZED: rectified left and right images

Start the CPU-only Open Capture driver and RViz:

```bash
ros2 launch zed_open_capture_ros zed2i.launch.py \
  resolution:=HD720 frame_rate:=30 rviz:=true
```

For lower USB and CPU load, use VGA at 30 Hz:

```bash
ros2 launch zed_open_capture_ros zed2i.launch.py \
  resolution:=VGA frame_rate:=30 rviz:=true
```

The camera supports VGA at up to 100 Hz, but 30 Hz is the recommended
starting point for visualization and downstream CPU processing.

```bash
ros2 topic hz /zed/left/image_rect_color
ros2 topic hz /zed/right/image_rect_color
ros2 topic info -v /zed/left/image_rect_color
```

The live image publishers and RViz subscriptions use best-effort sensor QoS,
so a slow display cannot back-pressure camera capture.

## 3. Foundation Stereo: left, right, and depth

This demo needs two terminals. Start the namespaced ZED driver in terminal
1:

```bash
ros2 launch zed_open_capture_ros zed2i.launch.py \
  namespace:=zed2i/zed_node resolution:=VGA frame_rate:=30
```

In terminal 2, activate the CPU environment and start the recommended fast
model:

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

On this CPU, approximately 3 to 4 depth frames per second is expected with the
fast model; the 30 Hz camera inputs remain live. The depth output is `32FC1`
in metres.

```bash
ros2 topic hz /zed2i/zed_node/left/image_rect_color
ros2 topic hz /zed2i/zed_node/right/image_rect_color
ros2 topic hz /foundation_stereo/depth
ros2 topic echo /foundation_stereo/depth/camera_info --once
```

## 4. GF-GG SLAM with ZED

The combined launch starts both Open Capture and GF-GG. It uses the atomic
mono stereo stream so the two eyes always belong to the same camera frame:

```bash
ros2 launch gfgg_slam_ros2 zed_open_capture.launch.py \
  resolution:=HD720 \
  frame_rate:=30 \
  recording_mode:=slam_mono \
  stereo_image_topic:=/zed/stereo/image_rect_mono \
  stereo_reliability:=best_effort \
  viewer:=true
```

Wait until GF-GG reports that its vocabulary and map system are ready before
moving the camera. Start with slow, well-lit motion and keep textured objects
in both eyes. The Pangolin window is the SLAM visualization. To monitor the
two source images in a second terminal as well, run:

```bash
rviz2 -d "$(ros2 pkg prefix --share zed_open_capture_ros)/rviz/zed_stereo.rviz"
```

Useful checks:

```bash
ros2 topic hz /zed/stereo/image_rect_mono
ros2 topic hz /camera_pose
ros2 topic echo /camera_pose --once
```

For a lighter tracking test, change the combined launch to
`resolution:=VGA frame_rate:=30`. Keep `stereo_reliability:=best_effort` for
the live topic; the separate `/zed/stereo/recording/...` topics are the
reliable streams intended for rosbag recording.

## Quick troubleshooting

- If a ZED launch reports that the device is busy, stop every other ZED or
  Foundation/GF-GG launch before retrying.
- If a package or RViz file is not found, source
  `~/ros2_ws/install/setup.bash` in that terminal after building.
- If an RViz image is blank, expand its display entry and confirm the topic.
  The supplied layouts already match the publishers' QoS policies.
- If Neuvition publishes nothing, verify `ping 192.168.1.101` and the host's
  Ethernet configuration.
- List active topics with `ros2 topic list`; inspect endpoint compatibility
  with `ros2 topic info -v <topic>`.

More setup and tuning detail is available in
[`README.md`](README.md),
[`../gfgg_slam/ros2/ZED_SETUP.md`](../gfgg_slam/ros2/ZED_SETUP.md), and
[`../foundation_stereo_ros/README.md`](../foundation_stereo_ros/README.md).
