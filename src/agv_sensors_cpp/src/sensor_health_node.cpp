#include <chrono>
#include <iomanip>
#include <sstream>
#include <string>
#include <unordered_map>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"

class SensorHealthNode : public rclcpp::Node
{
public:
  SensorHealthNode()
  : Node("sensor_health_node")
  {
    add_sensor("lidar", "LiDAR", 5.0);
    add_sensor("imu", "IMU", 80.0);
    add_sensor("wheel", "Odom", 10.0);
    add_sensor("depth", "Depth image", 8.0);
    add_sensor("orb", "ORB-SLAM3 RGB-D", 2.0);
    add_sensor("graph", "ORB3 Graph Factor", 2.0);
    add_sensor("amcl", "AMCL", 0.5);

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::LaserScan::ConstSharedPtr) {
        record("lidar");
      });
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data_raw", rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::Imu::ConstSharedPtr) {
        record("imu");
      });
    wheel_sub_ = create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
      "/wheel_vel", 10, [this](geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr) {
        record("wheel");
      });
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/camera/depth/image_raw", rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::Image::ConstSharedPtr) {
        record("depth");
      });
    orb_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/orbslam3/pose", 10, [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr) {
        record("orb");
      });
    graph_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/graph/odom", 10, [this](nav_msgs::msg::Odometry::ConstSharedPtr) {
        record("graph");
      });
    amcl_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/amcl_pose", 10, [this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr) {
        record("amcl");
      });

    publisher_ = create_publisher<std_msgs::msg::String>("/agv/sensor_health", 10);
    timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {publish_health();});
  }

private:
  struct SensorState
  {
    std::string name;
    double min_hz;
    size_t samples {0};
    std::chrono::steady_clock::time_point last_seen {};
  };

  void add_sensor(const std::string & key, const std::string & name, double min_hz)
  {
    sensors_.emplace(key, SensorState{name, min_hz});
  }

  void record(const std::string & key)
  {
    auto & sensor = sensors_.at(key);
    ++sensor.samples;
    sensor.last_seen = std::chrono::steady_clock::now();
  }

  void publish_health()
  {
    const auto now = std::chrono::steady_clock::now();
    const double elapsed = std::chrono::duration<double>(now - last_publish_).count();
    std::ostringstream json;
    json << "{\"sensors\":[";
    bool first = true;
    for (auto & [key, sensor] : sensors_) {
      (void)key;
      const double hz = elapsed > 0.0 ? static_cast<double>(sensor.samples) / elapsed : 0.0;
      const bool seen = sensor.last_seen != std::chrono::steady_clock::time_point{};
      const double age = seen ? std::chrono::duration<double>(now - sensor.last_seen).count() : 1e9;
      const bool ok = age <= 3.0 && hz >= sensor.min_hz;
      if (!first) {
        json << ',';
      }
      first = false;
      json << "{\"sensor\":\"" << sensor.name << "\",\"status\":\""
           << (ok ? "OK" : "WARN") << "\",\"hz\":" << std::fixed << std::setprecision(1)
           << (age <= 3.0 ? hz : 0.0) << ",\"warning\":\""
           << (ok ? "Normal" : (seen && age <= 3.0 ? "Low FPS" : "No data")) << "\"}";
      sensor.samples = 0;
    }
    json << "]}";
    last_publish_ = now;

    std_msgs::msg::String msg;
    msg.data = json.str();
    publisher_->publish(msg);
  }

  std::unordered_map<std::string, SensorState> sensors_;
  std::chrono::steady_clock::time_point last_publish_ {std::chrono::steady_clock::now()};
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr wheel_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr orb_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr graph_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SensorHealthNode>());
  rclcpp::shutdown();
  return 0;
}
