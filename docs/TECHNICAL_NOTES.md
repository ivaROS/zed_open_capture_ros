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

The ZED 2i magnetometer is published on `/zed/imu/mag` in tesla using the same
ROS FLU frame. Open Capture supplies scaled raw measurements, so the wrapper
only converts axes and units; it does not apply hard-iron or soft-iron
calibration, filtering, or heading estimation.

The static stereo transform uses the factory baseline loaded for the connected
camera serial number. The standard camera-to-optical rotations connect the
rectified optical frames to ROS FLU camera frames.

## Camera-IMU calibration

The ZED SDK factory transform for camera serial `39494516` is stored in
`config/zed2i.yaml`. The transform is from `zed_left_camera_frame` to
`zed_imu_link`, expressed in ROS FLU coordinates:

```yaml
translation: [-0.002000000095, -0.023061001673, 0.000217000023]
rotation_xyzw: [0.000153178014, 0.002542372094, 0.001893442124, 0.999994874001]
```

The node verifies the connected camera serial before publishing this transform.

ZED SDK 5.5 reports the following sensor metadata while camera `39494516` is
open:

| Sensor | Rate | Resolution | Range | Noise density | Random walk |
|---|---:|---:|---:|---:|---:|
| Accelerometer | 400 Hz | 0.00239502 m/s² | ±78.4800 m/s² | 0.000440 m/s²/√Hz | 0.0202 m/s²/√Hz |
| Gyroscope | 400 Hz | 0.0305176 °/s | ±1000 °/s | 0.004992 °/s/√Hz | 0.0424 °/s/√Hz |

These values are retained as factory reference data. Estimator configuration
will use values and units produced by the selected IMU calibration workflow.

Planned Kalibr work:

1. Record synchronized raw IMU and stereo images.
2. Estimate the camera-to-IMU transform with Kalibr.
3. Compare the Kalibr result with the stored ZED SDK factory transform.
4. Select the transform used by the runtime configuration.
5. Add estimator parameters for accelerometer and gyroscope noise density,
   random walk, bias, axis convention, units, and calibration source.
6. Validate the complete TF tree and coordinate conventions.

Target tree:

```text
zed_left_camera_frame
|-- zed_left_camera_optical_frame
|-- zed_right_camera_frame
|   `-- zed_right_camera_optical_frame
`-- zed_imu_link
```
