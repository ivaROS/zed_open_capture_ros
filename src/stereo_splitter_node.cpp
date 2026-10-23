#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>

namespace {

std::size_t bytesPerPixel(const std::string & encoding)
{
  if (encoding == "mono8" || encoding == "8UC1") {
    return 1;
  }
  if (encoding == "bgr8" || encoding == "rgb8") {
    return 3;
  }
  if (encoding == "bgra8" || encoding == "rgba8") {
    return 4;
  }
  throw std::invalid_argument(
          "unsupported combined stereo encoding '" + encoding +
          "'; expected mono8, 8UC1, bgr8, rgb8, bgra8, or rgba8");
}

}  // namespace

class StereoSplitterNode final : public rclcpp::Node
{
public:
  StereoSplitterNode()
  : Node("zed_stereo_splitter")
  {
    const std::string input_topic = declare_parameter<std::string>(
      "input_topic", "/zed/stereo/image_rect_mono");
    const std::string left_topic = declare_parameter<std::string>(
      "left_output_topic", "/zed/left/image_rect_mono");
    const std::string right_topic = declare_parameter<std::string>(
      "right_output_topic", "/zed/right/image_rect_mono");
    left_frame_id_ = declare_parameter<std::string>(
      "left_frame_id", "zed_left_camera_optical_frame");
    right_frame_id_ = declare_parameter<std::string>(
      "right_frame_id", "zed_right_camera_optical_frame");
    const auto queue_depth = static_cast<std::size_t>(std::max<std::int64_t>(
      2, declare_parameter<std::int64_t>("queue_depth", 10)));

    // Playback favors completeness over real-time dropping. A reliable bag
    // publisher and splitter keep both reconstructed eye topics balanced.
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(queue_depth)).reliable();
    left_publisher_ = create_publisher<sensor_msgs::msg::Image>(left_topic, qos);
    right_publisher_ = create_publisher<sensor_msgs::msg::Image>(right_topic, qos);
    subscription_ = create_subscription<sensor_msgs::msg::Image>(
      input_topic, qos,
      [this](sensor_msgs::msg::Image::ConstSharedPtr message) {split(*message);});

    RCLCPP_INFO(
      get_logger(), "Splitting %s into %s and %s", input_topic.c_str(),
      left_topic.c_str(), right_topic.c_str());
  }

  ~StereoSplitterNode() override
  {
    RCLCPP_INFO(
      get_logger(), "Published %llu complete stereo pairs",
      static_cast<unsigned long long>(published_pairs_));
  }

private:
  void split(const sensor_msgs::msg::Image & combined)
  {
    try {
      if (combined.width == 0 || combined.height == 0 || combined.width % 2 != 0) {
        throw std::invalid_argument("combined stereo image width must be positive and even");
      }
      const std::size_t bytes_per_pixel = bytesPerPixel(combined.encoding);
      const std::size_t eye_width = combined.width / 2;
      const std::size_t eye_step = eye_width * bytes_per_pixel;
      const std::size_t combined_row_bytes = combined.width * bytes_per_pixel;
      if (combined.step < combined_row_bytes) {
        throw std::invalid_argument("combined stereo image step is smaller than its row data");
      }
      const std::size_t required =
        static_cast<std::size_t>(combined.step) * combined.height;
      if (combined.data.size() < required) {
        throw std::invalid_argument("combined stereo image data is truncated");
      }

      sensor_msgs::msg::Image left;
      sensor_msgs::msg::Image right;
      left.header = combined.header;
      right.header = combined.header;
      left.header.frame_id = left_frame_id_;
      right.header.frame_id = right_frame_id_;
      left.height = combined.height;
      right.height = combined.height;
      left.width = static_cast<std::uint32_t>(eye_width);
      right.width = static_cast<std::uint32_t>(eye_width);
      left.encoding = combined.encoding;
      right.encoding = combined.encoding;
      left.is_bigendian = combined.is_bigendian;
      right.is_bigendian = combined.is_bigendian;
      left.step = static_cast<std::uint32_t>(eye_step);
      right.step = static_cast<std::uint32_t>(eye_step);
      left.data.resize(eye_step * combined.height);
      right.data.resize(eye_step * combined.height);

      for (std::size_t row = 0; row < combined.height; ++row) {
        const auto * source =
          combined.data.data() + row * static_cast<std::size_t>(combined.step);
        std::memcpy(left.data.data() + row * eye_step, source, eye_step);
        std::memcpy(right.data.data() + row * eye_step, source + eye_step, eye_step);
      }

      left_publisher_->publish(std::move(left));
      right_publisher_->publish(std::move(right));
      ++published_pairs_;
    } catch (const std::exception & error) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000, "Cannot split stereo frame: %s", error.what());
    }
  }

  std::string left_frame_id_;
  std::string right_frame_id_;
  std::uint64_t published_pairs_ = 0;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<StereoSplitterNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("zed_stereo_splitter"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
