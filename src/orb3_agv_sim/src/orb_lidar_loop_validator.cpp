#include <cmath>
#include <memory>
#include <string>
#include <mutex>
#include <array>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace
{
struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  bool valid{false};
};

double normalize_angle(double a)
{
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

double yaw_from_quat(const geometry_msgs::msg::Quaternion & qmsg)
{
  tf2::Quaternion q(qmsg.x, qmsg.y, qmsg.z, qmsg.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  return yaw;
}

geometry_msgs::msg::Quaternion quat_from_yaw(double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  geometry_msgs::msg::Quaternion out;
  out.x = q.x(); out.y = q.y(); out.z = q.z(); out.w = q.w();
  return out;
}
}  // namespace

class OrbLidarLoopValidator : public rclcpp::Node
{
public:
  OrbLidarLoopValidator()
  : Node("orb_lidar_loop_validator"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "Base_footprint");
    orb_pose_topic_ = declare_parameter<std::string>("orb_pose_topic", "/orbslam3/pose");
    lidar_odom_topic_ = declare_parameter<std::string>("lidar_odom_topic", "/odometry/filtered");
    max_xy_error_m_ = declare_parameter<double>("max_xy_error_m", 0.50);
    max_yaw_error_deg_ = declare_parameter<double>("max_yaw_error_deg", 10.0);
    auto_align_orb_to_lidar_ = declare_parameter<bool>("auto_align_orb_to_lidar", true);
    use_tf_lidar_pose_ = declare_parameter<bool>("use_tf_lidar_pose", true);
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 5.0);

    orb_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      orb_pose_topic_, rclcpp::SensorDataQoS(),
      std::bind(&OrbLidarLoopValidator::orbCallback, this, std::placeholders::_1));

    lidar_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      lidar_odom_topic_, rclcpp::SensorDataQoS(),
      std::bind(&OrbLidarLoopValidator::odomCallback, this, std::placeholders::_1));

    valid_pub_ = create_publisher<std_msgs::msg::Bool>("/orb_loop_valid", 10);
    error_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/orb_lidar_error", 10);
    aligned_orb_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/orb_aligned_pose", 10);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/orb_lidar_consistency_markers", 10);

    const auto period = std::chrono::duration<double>(1.0 / std::max(0.1, publish_rate_hz_));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&OrbLidarLoopValidator::timerCallback, this));

    RCLCPP_INFO(get_logger(), "ORB-Lidar loop validator started.");
    RCLCPP_INFO(get_logger(), "ORB pose topic: %s", orb_pose_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Lidar/EKF odom topic: %s", lidar_odom_topic_.c_str());
  }

private:
  void orbCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_orb_raw_.x = msg->pose.position.x;
    latest_orb_raw_.y = msg->pose.position.y;
    latest_orb_raw_.yaw = yaw_from_quat(msg->pose.orientation);
    latest_orb_raw_.valid = true;
    latest_orb_stamp_ = msg->header.stamp;
    latest_orb_frame_ = msg->header.frame_id.empty() ? "orb_map" : msg->header.frame_id;
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_odom_pose_.x = msg->pose.pose.position.x;
    latest_odom_pose_.y = msg->pose.pose.position.y;
    latest_odom_pose_.yaw = yaw_from_quat(msg->pose.pose.orientation);
    latest_odom_pose_.valid = true;
    latest_odom_stamp_ = msg->header.stamp;
  }

  Pose2D getLidarPoseNoLock()
  {
    if (use_tf_lidar_pose_) {
      try {
        auto tf = tf_buffer_.lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
        Pose2D p;
        p.x = tf.transform.translation.x;
        p.y = tf.transform.translation.y;
        p.yaw = yaw_from_quat(tf.transform.rotation);
        p.valid = true;
        return p;
      } catch (const tf2::TransformException & ex) {
        // During startup, map->base may not exist yet. Fallback to EKF odom.
      }
    }
    return latest_odom_pose_;
  }

  Pose2D alignOrbToMapNoLock(const Pose2D & orb_raw, const Pose2D & lidar_pose)
  {
    Pose2D out;
    if (!orb_raw.valid || !lidar_pose.valid) {
      return out;
    }

    if (auto_align_orb_to_lidar_ && !alignment_ready_) {
      align_yaw_ = normalize_angle(lidar_pose.yaw - orb_raw.yaw);
      const double c = std::cos(align_yaw_);
      const double s = std::sin(align_yaw_);
      align_tx_ = lidar_pose.x - (c * orb_raw.x - s * orb_raw.y);
      align_ty_ = lidar_pose.y - (s * orb_raw.x + c * orb_raw.y);
      alignment_ready_ = true;
      RCLCPP_INFO(get_logger(),
        "Auto aligned ORB frame to lidar/map frame: tx=%.3f ty=%.3f dyaw=%.2f deg",
        align_tx_, align_ty_, align_yaw_ * 180.0 / M_PI);
    }

    if (auto_align_orb_to_lidar_ && alignment_ready_) {
      const double c = std::cos(align_yaw_);
      const double s = std::sin(align_yaw_);
      out.x = c * orb_raw.x - s * orb_raw.y + align_tx_;
      out.y = s * orb_raw.x + c * orb_raw.y + align_ty_;
      out.yaw = normalize_angle(orb_raw.yaw + align_yaw_);
      out.valid = true;
    } else {
      out = orb_raw;
    }
    return out;
  }

  void timerCallback()
  {
    Pose2D lidar_pose;
    Pose2D orb_raw;
    rclcpp::Time stamp;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      lidar_pose = getLidarPoseNoLock();
      orb_raw = latest_orb_raw_;
      stamp = now();
    }

    if (!lidar_pose.valid || !orb_raw.valid) {
      return;
    }

    Pose2D orb_aligned;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      orb_aligned = alignOrbToMapNoLock(orb_raw, lidar_pose);
    }
    if (!orb_aligned.valid) {
      return;
    }

    const double dx = orb_aligned.x - lidar_pose.x;
    const double dy = orb_aligned.y - lidar_pose.y;
    const double e_xy = std::hypot(dx, dy);
    const double e_yaw = std::fabs(normalize_angle(orb_aligned.yaw - lidar_pose.yaw));
    const bool valid = (e_xy <= max_xy_error_m_) && (e_yaw <= max_yaw_error_deg_ * M_PI / 180.0);

    std_msgs::msg::Bool valid_msg;
    valid_msg.data = valid;
    valid_pub_->publish(valid_msg);

    std_msgs::msg::Float64MultiArray err_msg;
    err_msg.data = {
      e_xy,
      e_yaw * 180.0 / M_PI,
      lidar_pose.x,
      lidar_pose.y,
      lidar_pose.yaw,
      orb_aligned.x,
      orb_aligned.y,
      orb_aligned.yaw,
      valid ? 1.0 : 0.0
    };
    error_pub_->publish(err_msg);

    geometry_msgs::msg::PoseStamped aligned_msg;
    aligned_msg.header.stamp = stamp;
    aligned_msg.header.frame_id = map_frame_;
    aligned_msg.pose.position.x = orb_aligned.x;
    aligned_msg.pose.position.y = orb_aligned.y;
    aligned_msg.pose.position.z = 0.08;
    aligned_msg.pose.orientation = quat_from_yaw(orb_aligned.yaw);
    aligned_orb_pub_->publish(aligned_msg);

    publishMarkers(stamp, lidar_pose, orb_aligned, valid, e_xy, e_yaw);
  }

  void publishMarkers(
    const rclcpp::Time & stamp,
    const Pose2D & lidar_pose,
    const Pose2D & orb_pose,
    bool valid,
    double e_xy,
    double e_yaw)
  {
    visualization_msgs::msg::MarkerArray arr;

    auto make_sphere = [&](int id, const Pose2D & p, double r, double g, double b, const std::string & ns) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = map_frame_;
      m.header.stamp = stamp;
      m.ns = ns;
      m.id = id;
      m.type = visualization_msgs::msg::Marker::SPHERE;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.position.x = p.x;
      m.pose.position.y = p.y;
      m.pose.position.z = 0.18;
      m.pose.orientation.w = 1.0;
      m.scale.x = 0.22;
      m.scale.y = 0.22;
      m.scale.z = 0.22;
      m.color.r = r;
      m.color.g = g;
      m.color.b = b;
      m.color.a = 1.0;
      return m;
    };

    arr.markers.push_back(make_sphere(0, lidar_pose, 0.1, 0.8, 0.1, "lidar_ekf_pose"));
    arr.markers.push_back(make_sphere(1, orb_pose, 0.9, 0.2, 0.1, "orb_aligned_pose"));

    visualization_msgs::msg::Marker line;
    line.header.frame_id = map_frame_;
    line.header.stamp = stamp;
    line.ns = "orb_lidar_error_line";
    line.id = 2;
    line.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line.action = visualization_msgs::msg::Marker::ADD;
    line.scale.x = 0.04;
    line.color.r = valid ? 0.0 : 1.0;
    line.color.g = valid ? 1.0 : 0.0;
    line.color.b = 0.0;
    line.color.a = 1.0;
    geometry_msgs::msg::Point p1, p2;
    p1.x = lidar_pose.x; p1.y = lidar_pose.y; p1.z = 0.22;
    p2.x = orb_pose.x; p2.y = orb_pose.y; p2.z = 0.22;
    line.points.push_back(p1);
    line.points.push_back(p2);
    arr.markers.push_back(line);

    visualization_msgs::msg::Marker text;
    text.header.frame_id = map_frame_;
    text.header.stamp = stamp;
    text.ns = "orb_lidar_status_text";
    text.id = 3;
    text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::msg::Marker::ADD;
    text.pose.position.x = 0.5 * (lidar_pose.x + orb_pose.x);
    text.pose.position.y = 0.5 * (lidar_pose.y + orb_pose.y);
    text.pose.position.z = 0.75;
    text.pose.orientation.w = 1.0;
    text.scale.z = 0.35;
    text.color.r = valid ? 0.0 : 1.0;
    text.color.g = valid ? 1.0 : 0.0;
    text.color.b = 0.0;
    text.color.a = 1.0;
    char buf[160];
    std::snprintf(buf, sizeof(buf), "%s | e_xy=%.2fm, e_yaw=%.1fdeg",
      valid ? "ORB-Lidar OK" : "ORB-Lidar MISMATCH", e_xy, e_yaw * 180.0 / M_PI);
    text.text = buf;
    arr.markers.push_back(text);

    marker_pub_->publish(arr);
  }

  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string orb_pose_topic_;
  std::string lidar_odom_topic_;
  double max_xy_error_m_;
  double max_yaw_error_deg_;
  bool auto_align_orb_to_lidar_;
  bool use_tf_lidar_pose_;
  double publish_rate_hz_;

  std::mutex mutex_;
  Pose2D latest_orb_raw_;
  Pose2D latest_odom_pose_;
  rclcpp::Time latest_orb_stamp_;
  rclcpp::Time latest_odom_stamp_;
  std::string latest_orb_frame_;

  bool alignment_ready_{false};
  double align_tx_{0.0};
  double align_ty_{0.0};
  double align_yaw_{0.0};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr orb_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr lidar_odom_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr valid_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr error_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr aligned_orb_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OrbLidarLoopValidator>());
  rclcpp::shutdown();
  return 0;
}
