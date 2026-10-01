#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

#include <opencv2/opencv.hpp>

#include <calibration.hpp>
#include <sensorcapture.hpp>
#include <videocapture.hpp>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

namespace {

constexpr double kDegreesToRadians = 3.14159265358979323846 / 180.0;

builtin_interfaces::msg::Time toRosStamp(std::uint64_t timestamp_ns)
{
  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<std::int32_t>(timestamp_ns / 1000000000ULL);
  stamp.nanosec = static_cast<std::uint32_t>(timestamp_ns % 1000000000ULL);
  return stamp;
}

sl_oc::video::RESOLUTION parseResolution(const std::string & value)
{
  if (value == "VGA") {
    return sl_oc::video::RESOLUTION::VGA;
  }
  if (value == "HD720") {
    return sl_oc::video::RESOLUTION::HD720;
  }
  if (value == "HD1080") {
    return sl_oc::video::RESOLUTION::HD1080;
  }
  if (value == "HD2K") {
    return sl_oc::video::RESOLUTION::HD2K;
  }
  throw std::invalid_argument("resolution must be VGA, HD720, HD1080, or HD2K");
}

sl_oc::video::FPS parseFrameRate(int value)
{
  switch (value) {
    case 15: return sl_oc::video::FPS::FPS_15;
    case 30: return sl_oc::video::FPS::FPS_30;
    case 60: return sl_oc::video::FPS::FPS_60;
    case 100: return sl_oc::video::FPS::FPS_100;
    default: throw std::invalid_argument("frame_rate must be 15, 30, 60, or 100");
  }
}

sensor_msgs::msg::CameraInfo makeCameraInfo(
  const cv::Mat & projection, int width, int height, const std::string & frame_id,
  bool right_camera)
{
  sensor_msgs::msg::CameraInfo info;
  info.header.frame_id = frame_id;
  info.width = static_cast<std::uint32_t>(width);
  info.height = static_cast<std::uint32_t>(height);
  info.distortion_model = "plumb_bob";
  info.d.assign(5, 0.0);
  info.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};

  info.k = {
    projection.at<double>(0, 0), projection.at<double>(0, 1), projection.at<double>(0, 2),
    projection.at<double>(1, 0), projection.at<double>(1, 1), projection.at<double>(1, 2),
    projection.at<double>(2, 0), projection.at<double>(2, 1), projection.at<double>(2, 2)};

  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 4; ++col) {
      double value = projection.at<double>(row, col);
      // Open Capture's factory calibration uses millimetres for translation;
      // ROS CameraInfo uses metres and defines right Tx as -fx * baseline.
      if (right_camera && row == 0 && col == 3) {
        value = -std::abs(value) / 1000.0;
      }
      info.p[static_cast<std::size_t>(row * 4 + col)] = value;
    }
  }
  return info;
}

sensor_msgs::msg::Image makeImage(
  const cv::Mat & image, const builtin_interfaces::msg::Time & stamp,
  const std::string & frame_id)
{
  sensor_msgs::msg::Image msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;
  msg.height = static_cast<std::uint32_t>(image.rows);
  msg.width = static_cast<std::uint32_t>(image.cols);
  msg.encoding = "bgr8";
  msg.is_bigendian = false;
  msg.step = static_cast<std::uint32_t>(image.cols * image.elemSize());
  const std::size_t bytes = static_cast<std::size_t>(msg.step) * msg.height;
  msg.data.resize(bytes);
  std::memcpy(msg.data.data(), image.data, bytes);
  return msg;
}

}  // namespace

class ZedOpenCaptureNode final : public rclcpp::Node
{
public:
  ZedOpenCaptureNode()
  : Node("zed_open_capture")
  {
    const int device_id = declare_parameter<int>("device_id", -1);
    const std::string resolution = declare_parameter<std::string>("resolution", "HD720");
    const int frame_rate = declare_parameter<int>("frame_rate", 30);
    const std::string calibration_override =
      declare_parameter<std::string>("calibration_file", "");
    left_frame_id_ =
      declare_parameter<std::string>("left_frame_id", "zed_left_camera_optical_frame");
    right_frame_id_ =
      declare_parameter<std::string>("right_frame_id", "zed_right_camera_optical_frame");
    imu_frame_id_ = declare_parameter<std::string>("imu_frame_id", "zed_imu_link");
    publish_synchronized_imu_ =
      declare_parameter<bool>("publish_synchronized_imu", true);
    const bool publish_stereo_tf = declare_parameter<bool>("publish_stereo_tf", true);

    sl_oc::video::VideoParams params;
    params.res = parseResolution(resolution);
    params.fps = parseFrameRate(frame_rate);
    params.verbose = sl_oc::VERBOSITY::INFO;

    video_capture_ = std::make_unique<sl_oc::video::VideoCapture>(params);
    if (!video_capture_->initializeVideo(device_id)) {
      throw std::runtime_error("failed to open ZED UVC video device");
    }

    const int serial_number = video_capture_->getSerialNumber();
    sensor_capture_ =
      std::make_unique<sl_oc::sensors::SensorCapture>(sl_oc::VERBOSITY::INFO);
    if (!sensor_capture_->initializeSensors(serial_number)) {
      throw std::runtime_error("failed to open ZED sensor HID device");
    }

    // Open Capture owns the video/IMU clock synchronization. The ROS wrapper
    // consumes the timestamps returned by the API without estimating offsets.
    if (!video_capture_->enableSensorSync(sensor_capture_.get())) {
      throw std::runtime_error("failed to enable ZED video/IMU hardware synchronization");
    }

    int stereo_width = 0;
    int height = 0;
    video_capture_->getFrameSize(stereo_width, height);
    image_width_ = stereo_width / 2;
    image_height_ = height;

    std::string calibration_file = calibration_override;
    if (calibration_file.empty() &&
      !sl_oc::tools::downloadCalibrationFile(serial_number, calibration_file))
    {
      throw std::runtime_error("failed to download factory calibration");
    }

    cv::Mat left_projection;
    cv::Mat right_projection;
    double baseline_mm = 0.0;
    if (!sl_oc::tools::initCalibration(
        calibration_file, cv::Size(image_width_, image_height_),
        left_map_x_, left_map_y_, right_map_x_, right_map_y_,
        left_projection, right_projection, &baseline_mm))
    {
      throw std::runtime_error("failed to initialize stereo rectification");
    }

    left_info_ = makeCameraInfo(
      left_projection, image_width_, image_height_, left_frame_id_, false);
    right_info_ = makeCameraInfo(
      right_projection, image_width_, image_height_, right_frame_id_, true);

    if (publish_stereo_tf) {
      static_tf_broadcaster_ =
        std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
      geometry_msgs::msg::TransformStamped stereo_tf;
      stereo_tf.header.stamp = get_clock()->now();
      stereo_tf.header.frame_id = left_frame_id_;
      stereo_tf.child_frame_id = right_frame_id_;
      // The published images are rectified, so their optical frames are
      // parallel and separated only by the factory-calibrated baseline.
      stereo_tf.transform.translation.x = baseline_mm / 1000.0;
      stereo_tf.transform.rotation.w = 1.0;
      static_tf_broadcaster_->sendTransform(stereo_tf);
    }

    const auto image_qos = rclcpp::SensorDataQoS().keep_last(1);
    const auto imu_qos = rclcpp::SensorDataQoS().keep_last(10);
    left_image_pub_ = create_publisher<sensor_msgs::msg::Image>(
      "left/image_rect_color", image_qos);
    right_image_pub_ = create_publisher<sensor_msgs::msg::Image>(
      "right/image_rect_color", image_qos);
    left_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
      "left/camera_info", image_qos);
    right_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
      "right/camera_info", image_qos);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", imu_qos);
    if (publish_synchronized_imu_) {
      sync_imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu/data_sync", imu_qos);
    }

    RCLCPP_INFO(
      get_logger(),
      "ZED serial %d: %dx%d per eye at %d Hz, baseline %.3f mm",
      serial_number, image_width_, image_height_, frame_rate, baseline_mm);
    RCLCPP_INFO(
      get_logger(),
      "Video/IMU hardware synchronization enabled; calibration: %s",
      calibration_file.c_str());

    running_.store(true);
    imu_thread_ = std::thread(&ZedOpenCaptureNode::imuLoop, this);
    video_thread_ = std::thread(&ZedOpenCaptureNode::videoLoop, this);
  }

  ~ZedOpenCaptureNode() override
  {
    running_.store(false);
    if (video_thread_.joinable()) {
      video_thread_.join();
    }
    if (imu_thread_.joinable()) {
      imu_thread_.join();
    }
  }

private:
  void videoLoop()
  {
    std::uint64_t last_timestamp = 0;
    while (rclcpp::ok() && running_.load()) {
      const auto & frame = video_capture_->getLastFrame(100);
      if (frame.data == nullptr || frame.timestamp == 0 || frame.timestamp == last_timestamp) {
        continue;
      }
      last_timestamp = frame.timestamp;

      cv::Mat stereo_yuyv(frame.height, frame.width, CV_8UC2, frame.data);
      cv::Mat stereo_bgr;
      cv::cvtColor(stereo_yuyv, stereo_bgr, cv::COLOR_YUV2BGR_YUYV);
      const cv::Mat left_raw = stereo_bgr(
        cv::Rect(0, 0, image_width_, image_height_));
      const cv::Mat right_raw = stereo_bgr(
        cv::Rect(image_width_, 0, image_width_, image_height_));

      // This is the rectification path from Open Capture's
      // zed_oc_rectify_example: its factory maps plus OpenCV remap. No
      // calibration or rectification model is implemented in this wrapper.
      cv::Mat left_rect;
      cv::Mat right_rect;
      cv::remap(left_raw, left_rect, left_map_x_, left_map_y_, cv::INTER_LINEAR);
      cv::remap(right_raw, right_rect, right_map_x_, right_map_y_, cv::INTER_LINEAR);

      const auto stamp = toRosStamp(frame.timestamp);
      auto left_image = makeImage(left_rect, stamp, left_frame_id_);
      auto right_image = makeImage(right_rect, stamp, right_frame_id_);
      auto left_info = left_info_;
      auto right_info = right_info_;
      left_info.header.stamp = stamp;
      right_info.header.stamp = stamp;

      left_image_pub_->publish(std::move(left_image));
      right_image_pub_->publish(std::move(right_image));
      left_info_pub_->publish(std::move(left_info));
      right_info_pub_->publish(std::move(right_info));
    }
  }

  sensor_msgs::msg::Imu makeImu(const sl_oc::sensors::data::Imu & sample) const
  {
    sensor_msgs::msg::Imu msg;
    msg.header.stamp = toRosStamp(sample.timestamp);
    msg.header.frame_id = imu_frame_id_;

    // Message adaptation only: Open Capture supplies m/s^2 and deg/s in its
    // native sensor axes (X down, Y right, Z backward). sensor_msgs/Imu uses a
    // ROS REP-103 FLU frame and rad/s. No filtering or bias correction occurs.
    msg.linear_acceleration.x = -sample.aZ;
    msg.linear_acceleration.y = -sample.aY;
    msg.linear_acceleration.z = -sample.aX;
    msg.angular_velocity.x = -sample.gZ * kDegreesToRadians;
    msg.angular_velocity.y = -sample.gY * kDegreesToRadians;
    msg.angular_velocity.z = -sample.gX * kDegreesToRadians;

    msg.orientation_covariance[0] = -1.0;
    msg.angular_velocity_covariance[0] = -1.0;
    msg.linear_acceleration_covariance[0] = -1.0;
    return msg;
  }

  void imuLoop()
  {
    std::uint64_t last_timestamp = 0;
    while (rclcpp::ok() && running_.load()) {
      const auto & sample = sensor_capture_->getLastIMUData(5000);
      if (sample.valid != sl_oc::sensors::data::Imu::NEW_VAL ||
        sample.timestamp == 0 || sample.timestamp == last_timestamp)
      {
        continue;
      }
      last_timestamp = sample.timestamp;
      auto msg = makeImu(sample);
      imu_pub_->publish(msg);
      if (sample.sync && sync_imu_pub_) {
        sync_imu_pub_->publish(std::move(msg));
      }
    }
  }

  std::unique_ptr<sl_oc::video::VideoCapture> video_capture_;
  std::unique_ptr<sl_oc::sensors::SensorCapture> sensor_capture_;
  std::atomic<bool> running_{false};
  std::thread video_thread_;
  std::thread imu_thread_;

  int image_width_ = 0;
  int image_height_ = 0;
  std::string left_frame_id_;
  std::string right_frame_id_;
  std::string imu_frame_id_;
  bool publish_synchronized_imu_ = true;
  cv::Mat left_map_x_;
  cv::Mat left_map_y_;
  cv::Mat right_map_x_;
  cv::Mat right_map_y_;
  sensor_msgs::msg::CameraInfo left_info_;
  sensor_msgs::msg::CameraInfo right_info_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr left_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr right_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr sync_imu_pub_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<ZedOpenCaptureNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("zed_open_capture"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
