# zed_open_capture_ros

ROS 2 Jazzy wrapper for ZED stereo images and IMU data using Stereolabs
[ZED Open Capture](https://github.com/stereolabs/zed-open-capture).

For the shortest copy-and-paste launch instructions, see
[`QUICK_START.md`](QUICK_START.md). For setup checks, explanations, and
troubleshooting, see [`DEMO_COMMANDS.md`](DEMO_COMMANDS.md).

## Install

The primary configuration is tested on Ubuntu 24.04 with ROS 2 Jazzy. Install
ROS 2 Jazzy first, then install the build tools and system libraries:

```bash
sudo apt update
sudo apt install -y \
  build-essential cmake git pkg-config \
  libopencv-dev libusb-1.0-0-dev \
  python3-colcon-common-extensions python3-rosdep
rosdep update
```

Run `sudo rosdep init` once first if `rosdep` has not been initialized on the
machine.

### GPU-free Open Capture wrapper

Clone this repository into a ROS workspace:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/ivaROS/zed_open_capture_ros.git
```

Install its ROS dependencies and build it:

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src/zed_open_capture_ros \
  --ignore-src -r -y
colcon build --symlink-install --packages-select zed_open_capture_ros
source install/setup.bash
```

`zed_wrapper` is not a dependency of the standalone package. It is required
only when using the optional `lightgpu.launch.py` launch file. ZED Open Capture
and HIDAPI are downloaded at the pinned revisions by CMake during the first
build, so they do not need to be cloned separately.

Connect the camera and launch rectified stereo images and sensors:

```bash
ros2 launch zed_open_capture_ros zed2i.launch.py
```

### Optional ZED SDK wrapper

Depth and ZED positional tracking use the ZED SDK, CUDA, and an NVIDIA GPU.
Install the matching [ZED SDK](https://www.stereolabs.com/developers/release/)
first and verify that the GPU is available:

```bash
nvidia-smi
```

Then clone the project fork. Its `main` branch contains the change that permits
GEN_3 positional tracking while `depth.depth_mode` is `NONE`:

```bash
cd ~/ros2_ws/src
git clone --branch main https://github.com/ivaROS/zed-ros2-wrapper.git

cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src/zed-ros2-wrapper src/zed_open_capture_ros \
  --ignore-src -r -y
colcon build --symlink-install \
  --packages-up-to zed_open_capture_ros zed_wrapper \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

On Jazzy, `rosdep` installs `zed_msgs` and the remaining ROS dependencies from
the ROS package repository. The installed ZED SDK and CUDA toolkit provide the
CMake packages required to compile the fork. The `package.xml` dependency does
not clone source repositories, so the fork must be cloned explicitly as shown
above.

## ZED SDK quick launch

These commands use the customized `zed_wrapper` fork, based on the official
Stereolabs wrapper, and require the ZED SDK and a working NVIDIA GPU. Source
the workspace first:

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
nvidia-smi
```

Stereo images and IMU, without depth or positional tracking:

```bash
ros2 launch zed_wrapper zed_camera.launch.py \
  camera_model:=zed2i camera_name:=zed2i \
  publish_tf:=true publish_imu_tf:=true \
  param_overrides:='depth.depth_mode:=NONE;video.publish_rgb:=false;video.publish_left_right:=true;sensors.publish_imu:=true;pos_tracking.pos_tracking_enabled:=false'
```

Stereo images, IMU, and depth, without point-cloud generation or positional
tracking:

```bash
ros2 launch zed_wrapper zed_camera.launch.py \
  camera_model:=zed2i camera_name:=zed2i \
  publish_tf:=true publish_imu_tf:=true \
  param_overrides:='depth.depth_mode:=PERFORMANCE;video.publish_rgb:=false;video.publish_left_right:=true;sensors.publish_imu:=true;depth.publish_depth_map:=true;depth.publish_depth_image:=true;depth.publish_point_cloud:=false;pos_tracking.pos_tracking_enabled:=false'
```

Stereo images, sensors, and lightweight ZED SDK positional tracking without
depth:

```bash
ros2 launch zed_open_capture_ros lightgpu.launch.py
```

This preset uses HD720 at 30 FPS and publishes rectified stereo, fused and raw
IMU, magnetometer, barometer, temperatures, the camera-to-IMU transform, pose,
odometry, pose covariance, status, and TF. It is bound to ZED 2i serial number
`39494516`; override it with `serial_number:=<serial>` when needed.

Open RViz in a second sourced terminal:

```bash
ros2 run rviz2 rviz2
```

Use `map` as the fixed frame for positional tracking, or
`zed2i_left_camera_frame` for image/depth-only operation. Add `Image`, `TF`,
and `Odometry` displays as needed. Useful SDK topics include:

| Topic | Content |
|---|---|
| `/zed2i/zed_node/left/color/rect/image` | Rectified left image |
| `/zed2i/zed_node/right/color/rect/image` | Rectified right image |
| `/zed2i/zed_node/depth/depth_registered` | Registered depth map |
| `/zed2i/zed_node/imu/data` | Fused IMU data |
| `/zed2i/zed_node/imu/data_raw` | Raw IMU data |
| `/zed2i/zed_node/imu/mag` | Magnetometer data |
| `/zed2i/zed_node/atm_press` | Barometer data |
| `/zed2i/zed_node/left_cam_imu_transform` | Camera-to-IMU transform |
| `/zed2i/zed_node/odom` | Local tracking odometry |
| `/zed2i/zed_node/pose` | Pose in the map frame |
| `/zed2i/zed_node/pose_with_covariance` | Pose with covariance |
| `/zed2i/zed_node/pose/status` | Positional-tracking status |

Stop the launch before changing modes.

## Dependencies and source layout

The GPU-free wrapper uses ROS 2 Jazzy, OpenCV, libusb, HIDAPI, and ZED Open
Capture. The ZED SDK and CUDA are only required for `zed_wrapper` and
`lightgpu.launch.py`.

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

## Atomic stereo recording and playback

Recording the two uncompressed image topics independently can drop one eye
without dropping the other. The optional atomic streams pack each rectified
stereo pair side by side into one `sensor_msgs/Image`. A best-effort topic
serves live tracking without backpressure; a separate reliable topic is fed
by a bounded worker queue for rosbag. A slow disk can therefore drop a whole
recording pair, but cannot stall camera acquisition or split the pair. Existing
left/right color topics are unchanged.

Choose one recording mode when launching the camera:

```bash
# One-channel rectified stereo for visual SLAM (one third of BGR8 payload).
ros2 launch zed_open_capture_ros zed2i.launch.py \
  resolution:=VGA frame_rate:=30 recording_mode:=slam_mono

# Full color for FoundationStereo and other color consumers.
ros2 launch zed_open_capture_ros zed2i.launch.py \
  resolution:=VGA frame_rate:=30 recording_mode:=stereo_color
```

Record the matching atomic topic in another sourced terminal. The helper uses
indexed MCAP with a 512 MiB cache and records synchronized IMU and calibration
without the high-rate raw IMU or diagnostic topics:

```bash
ros2 run zed_open_capture_ros record_stereo.sh \
  slam_mono ~/ros2_ws/data/zed_slam_mono
```

Use `stereo_color` in both commands when the recording must retain color for
FoundationStereo. The recorder uses
`/zed/stereo/recording/image_rect_{mono,color}`; live consumers use
`/zed/stereo/image_rect_{mono,color}`. Stop with Ctrl-C so rosbag finalizes the
MCAP metadata. On shutdown the camera reports how many reliable pairs were
published and how many complete pairs the bounded queue dropped. Increase
`recording_queue_depth` only if short disk stalls cause drops and memory allows.

Start consumers before playback. For example, start GF-GG in another sourced
terminal for a mono recording:

```bash
ros2 launch gfgg_slam_ros2 zed_open_capture.launch.py \
  start_camera:=false viewer:=true \
  stereo_image_topic:=/zed/stereo/image_rect_mono \
  stereo_reliability:=reliable
```

Then replay and split the atomic image back into conventional per-eye topics:

```bash
ros2 launch zed_open_capture_ros stereo_playback.launch.py \
  bag:=~/ros2_ws/data/zed_slam_mono \
  recording_mode:=slam_mono
```

Playback first sends a short calibration warmup and waits four seconds before
images, allowing GF-GG to load its vocabulary without losing the beginning of
the sequence. Override `warmup_seconds` or `rate` when needed.
The reliable recording topic is remapped to the conventional live atomic topic
during playback. For bags made before the topic split, add
`recorded_stereo_topic:=/zed/stereo/image_rect_mono`.

`slam_mono` playback publishes `/zed/left/image_rect_mono` and
`/zed/right/image_rect_mono`. `stereo_color` playback publishes the existing
`image_rect_color` names. Both outputs retain the recorded hardware timestamp;
left and right messages from a pair therefore have exactly equal stamps.

FoundationStereo accepts mono messages by expanding them to three channels,
but its model was trained on RGB images. Use `stereo_color` when evaluating
dense-depth quality so chromatic image information is preserved.

## View the images

In another sourced terminal:

```bash
ros2 run rqt_image_view rqt_image_view
```

Select `/zed/left/image_rect_color` or `/zed/right/image_rect_color`.

For RViz:

```bash
ros2 launch zed_open_capture_ros zed2i.launch.py rviz:=true
```

The supplied layout uses `zed_left_camera_optical_frame` as its fixed frame
and opens the rectified left and right image displays with best-effort QoS.

## Topics and checks

| Topic | Type |
|---|---|
| `/zed/left/image_rect_color` | `sensor_msgs/Image` |
| `/zed/right/image_rect_color` | `sensor_msgs/Image` |
| `/zed/stereo/image_rect_mono` | Best-effort atomic mono, when enabled |
| `/zed/stereo/image_rect_color` | Best-effort atomic color, when enabled |
| `/zed/stereo/recording/image_rect_mono` | Reliable queued mono, when enabled |
| `/zed/stereo/recording/image_rect_color` | Reliable queued color, when enabled |
| `/zed/left/camera_info` | `sensor_msgs/CameraInfo` |
| `/zed/right/camera_info` | `sensor_msgs/CameraInfo` |
| `/zed/imu/data_raw` | `sensor_msgs/Imu` |
| `/zed/imu/data_sync` | `sensor_msgs/Imu` |
| `/zed/imu/mag` | `sensor_msgs/MagneticField` |

```bash
ros2 topic hz /zed/left/image_rect_color
ros2 topic hz /zed/right/image_rect_color
ros2 topic hz /zed/imu/data_raw
ros2 topic echo /zed/imu/data_sync --once
ros2 topic echo /zed/imu/mag --once

ros2 run tf2_ros tf2_echo \
  zed_left_camera_optical_frame \
  zed_right_camera_optical_frame

ros2 run tf2_ros tf2_echo \
  zed_left_camera_frame \
  zed_imu_link
```

Implementation and calibration notes are in
[`docs/TECHNICAL_NOTES.md`](docs/TECHNICAL_NOTES.md).
