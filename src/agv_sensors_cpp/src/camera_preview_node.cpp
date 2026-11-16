#include <algorithm>
#include <chrono>
#include <cstdint>
#include <sstream>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"

namespace
{
const char kBase64Table[] =
  "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

std::string base64_encode(const std::vector<uint8_t> & data)
{
  std::string out;
  out.reserve(((data.size() + 2) / 3) * 4);
  size_t i = 0;
  while (i + 2 < data.size()) {
    const uint32_t n = (static_cast<uint32_t>(data[i]) << 16) |
      (static_cast<uint32_t>(data[i + 1]) << 8) | data[i + 2];
    out.push_back(kBase64Table[(n >> 18) & 63]);
    out.push_back(kBase64Table[(n >> 12) & 63]);
    out.push_back(kBase64Table[(n >> 6) & 63]);
    out.push_back(kBase64Table[n & 63]);
    i += 3;
  }
  if (i < data.size()) {
    uint32_t n = static_cast<uint32_t>(data[i]) << 16;
    out.push_back(kBase64Table[(n >> 18) & 63]);
    if (i + 1 < data.size()) {
      n |= static_cast<uint32_t>(data[i + 1]) << 8;
      out.push_back(kBase64Table[(n >> 12) & 63]);
      out.push_back(kBase64Table[(n >> 6) & 63]);
      out.push_back('=');
    } else {
      out.push_back(kBase64Table[(n >> 12) & 63]);
      out.push_back('=');
      out.push_back('=');
    }
  }
  return out;
}
}  // namespace

class CameraPreviewNode : public rclcpp::Node
{
public:
  CameraPreviewNode()
  : Node("camera_preview_node")
  {
    image_topic_ = declare_parameter<std::string>("image_topic", "/camera/color/image_raw");
    preview_topic_ = declare_parameter<std::string>("preview_topic", "/agv/camera_preview");
    target_width_ = declare_parameter<int>("target_width", 320);
    publish_hz_ = declare_parameter<double>("publish_hz", 2.0);

    publisher_ = create_publisher<std_msgs::msg::String>(preview_topic_, 2);
    subscription_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic_, rclcpp::SensorDataQoS(),
      std::bind(&CameraPreviewNode::on_image, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(), "Camera preview: %s -> %s width=%d hz=%.1f",
      image_topic_.c_str(), preview_topic_.c_str(), target_width_, publish_hz_);
  }

private:
  void on_image(const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    if (msg->encoding != "rgb8" || msg->width == 0 || msg->height == 0 ||
      msg->step < msg->width * 3)
    {
      return;
    }

    const auto now = std::chrono::steady_clock::now();
    if (last_publish_ != std::chrono::steady_clock::time_point{} &&
      std::chrono::duration<double>(now - last_publish_).count() < 1.0 / std::max(0.1, publish_hz_))
    {
      return;
    }
    last_publish_ = now;

    const int src_w = static_cast<int>(msg->width);
    const int src_h = static_cast<int>(msg->height);
    const int dst_w = std::clamp(target_width_, 80, src_w);
    const int dst_h = std::max(1, static_cast<int>((static_cast<double>(src_h) * dst_w) / src_w));

    std::vector<uint8_t> preview(static_cast<size_t>(dst_w) * dst_h * 3);
    for (int y = 0; y < dst_h; ++y) {
      const int sy = std::min(src_h - 1, (y * src_h) / dst_h);
      for (int x = 0; x < dst_w; ++x) {
        const int sx = std::min(src_w - 1, (x * src_w) / dst_w);
        const size_t src = static_cast<size_t>(sy) * msg->step + static_cast<size_t>(sx) * 3;
        const size_t dst = (static_cast<size_t>(y) * dst_w + x) * 3;
        preview[dst] = msg->data[src];
        preview[dst + 1] = msg->data[src + 1];
        preview[dst + 2] = msg->data[src + 2];
      }
    }

    std::ostringstream json;
    json << "{\"width\":" << dst_w
         << ",\"height\":" << dst_h
         << ",\"step\":" << dst_w * 3
         << ",\"encoding\":\"rgb8\",\"data\":\"" << base64_encode(preview) << "\"}";

    std_msgs::msg::String out;
    out.data = json.str();
    publisher_->publish(out);
  }

  std::string image_topic_;
  std::string preview_topic_;
  int target_width_;
  double publish_hz_;
  std::chrono::steady_clock::time_point last_publish_ {};
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraPreviewNode>());
  rclcpp::shutdown();
  return 0;
}
