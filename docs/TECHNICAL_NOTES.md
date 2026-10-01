# Technical notes

## Capture path

The node uses these ZED Open Capture APIs directly:

- `VideoCapture` for side-by-side UVC frames and timestamps
- `SensorCapture` for IMU samples and timestamps
- `enableSensorSync()` for video and sensor clock synchronization
- `downloadCalibrationFile()` for serial-specific factory calibration
- `initCalibration()` for stereo rectification maps

Image processing follows the upstream `zed_oc_rectify_example.cpp`: YUYV to
BGR conversion, left/right frame extraction, and OpenCV `remap()` with the
factory maps.

Both images from a stereo frame carry the same timestamp. IMU timestamps come
from Open Capture's synchronized sensor clock. IMU acceleration is published in
m/s² and angular velocity in rad/s using a ROS FLU frame.

The static left-to-right optical transform uses the factory baseline loaded for
the connected camera serial number.

## Camera-IMU calibration

Planned calibration work:

1. Record synchronized raw IMU and stereo images.
2. Estimate the camera-to-IMU transform with Kalibr.
3. Read the ZED SDK factory `camera_imu_transform` with the NVIDIA A2 enabled
   and compare the results.
4. Store the selected transform as ROS parameters and publish it on
   `/tf_static`.
5. Add estimator parameters for accelerometer and gyroscope noise density,
   random walk, bias, axis convention, units, and calibration source.
6. Validate the complete TF tree and coordinate conventions.

Target tree:

```text
zed_left_camera_optical_frame
|-- zed_right_camera_optical_frame
`-- zed_imu_link
```
