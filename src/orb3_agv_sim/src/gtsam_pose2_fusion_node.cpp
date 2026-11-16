#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <ctime>
#include <vector>

#include <action_msgs/msg/goal_status_array.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include "orb3_agv_sim/graph_diagnostics.hpp"

namespace
{
double stampToSec(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

double durationToSec(const builtin_interfaces::msg::Duration & duration)
{
  return static_cast<double>(duration.sec) + static_cast<double>(duration.nanosec) * 1e-9;
}

double normalizeAngle(double a)
{
  return std::atan2(std::sin(a), std::cos(a));
}

double yawFromQuat(const geometry_msgs::msg::Quaternion & q_msg)
{
  tf2::Quaternion q;
  tf2::fromMsg(q_msg, q);
  double roll = 0.0, pitch = 0.0, yaw = 0.0;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  return yaw;
}

geometry_msgs::msg::Quaternion quatFromYaw(double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  q.normalize();
  return tf2::toMsg(q);
}

gtsam::Pose2 pose2FromPose(const geometry_msgs::msg::Pose & pose)
{
  return gtsam::Pose2(pose.position.x, pose.position.y, yawFromQuat(pose.orientation));
}

gtsam::Pose2 pose2FromTransform(const geometry_msgs::msg::Transform & transform)
{
  return gtsam::Pose2(
    transform.translation.x,
    transform.translation.y,
    yawFromQuat(transform.rotation));
}

geometry_msgs::msg::Pose poseFromPose2(const gtsam::Pose2 & pose)
{
  geometry_msgs::msg::Pose out;
  out.position.x = pose.x();
  out.position.y = pose.y();
  out.position.z = 0.0;
  out.orientation = quatFromYaw(pose.theta());
  return out;
}

gtsam::Key keyFor(size_t index)
{
  return gtsam::Symbol('x', index);
}

bool finitePose(const gtsam::Pose2 & pose)
{
  return std::isfinite(pose.x()) && std::isfinite(pose.y()) && std::isfinite(pose.theta());
}

bool finitePoseMsg(const geometry_msgs::msg::Pose & pose)
{
  return std::isfinite(pose.position.x) && std::isfinite(pose.position.y) &&
         std::isfinite(pose.position.z) && std::isfinite(pose.orientation.x) &&
         std::isfinite(pose.orientation.y) && std::isfinite(pose.orientation.z) &&
         std::isfinite(pose.orientation.w);
}

double deltaNormXY(const gtsam::Pose2 & delta)
{
  return std::hypot(delta.x(), delta.y());
}

struct MapBounds
{
  bool valid{false};
  double min_x{0.0};
  double min_y{0.0};
  double max_x{0.0};
  double max_y{0.0};
};

struct OccupancyMap
{
  bool valid{false};
  std::string yaml_file;
  std::string image_file;
  int width{0};
  int height{0};
  double resolution{0.0};
  double origin_x{0.0};
  double origin_y{0.0};
  std::vector<uint8_t> pixels;
};

struct OccupancyGateResult
{
  bool accepted{true};
  std::string reason{"accepted"};
  int u{-1};
  int v{-1};
  int value{-1};
};

std::string trim(const std::string & value)
{
  const auto first = value.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return "";
  }
  const auto last = value.find_last_not_of(" \t\r\n");
  return value.substr(first, last - first + 1);
}

std::optional<std::pair<int, int>> readPgmSize(const std::filesystem::path & pgm_path)
{
  std::ifstream in(pgm_path, std::ios::binary);
  if (!in.is_open()) {
    return std::nullopt;
  }

  std::string magic;
  in >> magic;
  if (magic != "P5" && magic != "P2") {
    return std::nullopt;
  }

  auto read_token = [&in]() -> std::optional<std::string> {
    std::string token;
    while (in >> token) {
      if (!token.empty() && token[0] == '#') {
        std::string ignored;
        std::getline(in, ignored);
        continue;
      }
      return token;
    }
    return std::nullopt;
  };

  const auto width_token = read_token();
  const auto height_token = read_token();
  if (!width_token || !height_token) {
    return std::nullopt;
  }

  try {
    return std::make_pair(std::stoi(*width_token), std::stoi(*height_token));
  } catch (const std::exception &) {
    return std::nullopt;
  }
}

std::optional<OccupancyMap> loadOccupancyMapFromYaml(const std::string & yaml_file)
{
  if (yaml_file.empty()) {
    return std::nullopt;
  }

  std::ifstream in(yaml_file);
  if (!in.is_open()) {
    return std::nullopt;
  }

  std::string image;
  double resolution = std::numeric_limits<double>::quiet_NaN();
  double origin_x = std::numeric_limits<double>::quiet_NaN();
  double origin_y = std::numeric_limits<double>::quiet_NaN();

  std::string line;
  while (std::getline(in, line)) {
    const auto comment = line.find('#');
    if (comment != std::string::npos) {
      line = line.substr(0, comment);
    }
    const auto colon = line.find(':');
    if (colon == std::string::npos) {
      continue;
    }
    const std::string key = trim(line.substr(0, colon));
    const std::string value = trim(line.substr(colon + 1));
    try {
      if (key == "image") {
        image = value;
        if (!image.empty() && image.front() == '"' && image.back() == '"') {
          image = image.substr(1, image.size() - 2);
        }
      } else if (key == "resolution") {
        resolution = std::stod(value);
      } else if (key == "origin") {
        const auto lb = value.find('[');
        const auto rb = value.find(']');
        if (lb != std::string::npos && rb != std::string::npos && rb > lb) {
          std::string values = value.substr(lb + 1, rb - lb - 1);
          std::replace(values.begin(), values.end(), ',', ' ');
          std::istringstream ss(values);
          ss >> origin_x >> origin_y;
        }
      }
    } catch (const std::exception &) {
      return std::nullopt;
    }
  }

  if (image.empty() || !std::isfinite(resolution) || resolution <= 0.0 ||
    !std::isfinite(origin_x) || !std::isfinite(origin_y))
  {
    return std::nullopt;
  }

  const auto yaml_path = std::filesystem::path(yaml_file);
  std::filesystem::path image_path(image);
  if (image_path.is_relative()) {
    image_path = yaml_path.parent_path() / image_path;
  }
  const auto size = readPgmSize(image_path);
  if (!size || size->first <= 0 || size->second <= 0) {
    return std::nullopt;
  }

  std::ifstream pgm(image_path, std::ios::binary);
  if (!pgm.is_open()) {
    return std::nullopt;
  }

  std::string magic;
  pgm >> magic;
  if (magic != "P5" && magic != "P2") {
    return std::nullopt;
  }

  auto read_token = [&pgm]() -> std::optional<std::string> {
    std::string token;
    while (pgm >> token) {
      if (!token.empty() && token[0] == '#') {
        std::string ignored;
        std::getline(pgm, ignored);
        continue;
      }
      return token;
    }
    return std::nullopt;
  };

  const auto width_token = read_token();
  const auto height_token = read_token();
  const auto max_token = read_token();
  if (!width_token || !height_token || !max_token) {
    return std::nullopt;
  }

  int width = 0;
  int height = 0;
  int max_value = 0;
  try {
    width = std::stoi(*width_token);
    height = std::stoi(*height_token);
    max_value = std::stoi(*max_token);
  } catch (const std::exception &) {
    return std::nullopt;
  }
  if (width <= 0 || height <= 0 || max_value <= 0 || max_value > 255) {
    return std::nullopt;
  }

  OccupancyMap map;
  map.valid = true;
  map.yaml_file = yaml_file;
  map.image_file = image_path.string();
  map.width = width;
  map.height = height;
  map.resolution = resolution;
  map.origin_x = origin_x;
  map.origin_y = origin_y;
  map.pixels.resize(static_cast<size_t>(width) * static_cast<size_t>(height));
  if (magic == "P5") {
    pgm.get();
    pgm.read(
      reinterpret_cast<char *>(map.pixels.data()),
      static_cast<std::streamsize>(map.pixels.size()));
    if (pgm.gcount() != static_cast<std::streamsize>(map.pixels.size())) {
      return std::nullopt;
    }
  } else {
    for (auto & pixel : map.pixels) {
      const auto token = read_token();
      if (!token) {
        return std::nullopt;
      }
      try {
        pixel = static_cast<uint8_t>(std::clamp(std::stoi(*token), 0, 255));
      } catch (const std::exception &) {
        return std::nullopt;
      }
    }
  }
  return map;
}

std::optional<MapBounds> boundsFromOccupancyMap(const OccupancyMap & map)
{
  if (!map.valid) {
    return std::nullopt;
  }
  MapBounds bounds;
  bounds.valid = true;
  bounds.min_x = map.origin_x;
  bounds.min_y = map.origin_y;
  bounds.max_x = map.origin_x + static_cast<double>(map.width) * map.resolution;
  bounds.max_y = map.origin_y + static_cast<double>(map.height) * map.resolution;
  return bounds;
}

std::string statusName(int status)
{
  switch (status) {
    case 0: return "UNKNOWN";
    case 1: return "ACCEPTED";
    case 2: return "EXECUTING";
    case 3: return "CANCELING";
    case 4: return "SUCCEEDED";
    case 5: return "CANCELED";
    case 6: return "ABORTED";
    default: return "INVALID";
  }
}
}  // namespace

class GtsamPose2FusionNode : public rclcpp::Node
{
public:
  GtsamPose2FusionNode()
  : Node("gtsam_pose2_fusion_node")
  {
    ekf_topic_ = declare_parameter<std::string>("ekf_odom_topic", "/odometry/filtered");
    amcl_topic_ = declare_parameter<std::string>("amcl_pose_topic", "/amcl_pose");
    global_pose_topic_ = declare_parameter<std::string>(
      "global_pose_topic", "/slam_toolbox/pose");
    orb_topic_ = declare_parameter<std::string>("orb_pose_topic", "/orbslam3/pose_base");
    orb_tracking_state_topic_ = declare_parameter<std::string>(
      "orb_tracking_state_topic", "/orbslam3/tracking_state");
    scan_odom_topic_ = declare_parameter<std::string>("scan_odom_topic", "/rf2o/odom");
    turn_rate_topic_ = declare_parameter<std::string>("turn_rate_topic", "/wheel_vel");
    turn_rate_timeout_sec_ = declare_parameter<double>("turn_rate_timeout_sec", 0.50);
    graph_odom_topic_ = declare_parameter<std::string>("graph_odom_topic", "/graph/odom");
    graph_pose_topic_ = declare_parameter<std::string>("graph_pose_topic", "/graph/pose");
    graph_path_topic_ = declare_parameter<std::string>("graph_path_topic", "/graph/path");
    graph_stats_topic_ = declare_parameter<std::string>("graph_stats_topic", "/graph/stats");
    map_yaml_file_ = declare_parameter<std::string>("map_yaml_file", "");
    enforce_scan_map_bounds_ = declare_parameter<bool>("enforce_scan_map_bounds", true);
    scan_map_bounds_margin_ = declare_parameter<double>("scan_map_bounds_margin", 0.30);
    scan_map_occupancy_gate_radius_ =
      declare_parameter<double>("scan_map_occupancy_gate_radius", 0.20);
    enable_diagnostics_ = declare_parameter<bool>("enable_diagnostics", true);
    diagnostics_output_dir_ = declare_parameter<std::string>(
      "diagnostics_output_dir", "/home/hcmute/agv_mpc/diagnostics");
    enable_run_logging_ = declare_parameter<bool>("enable_run_logging", true);
    run_log_root_ = declare_parameter<std::string>(
      "run_log_root", "/home/hcmute/agv_mpc/logs/graph_runs");
    run_log_tag_ = declare_parameter<std::string>("run_log_tag", "");
    run_pose_log_min_interval_sec_ = declare_parameter<double>(
      "run_pose_log_min_interval_sec", 0.20);
    run_rejected_factor_log_min_interval_sec_ = declare_parameter<double>(
      "run_rejected_factor_log_min_interval_sec", 1.0);
    run_scan_map_debug_min_interval_sec_ = declare_parameter<double>(
      "run_scan_map_debug_min_interval_sec", 0.50);
    nav2_status_topic_ = declare_parameter<std::string>(
      "nav2_status_topic", "/navigate_to_pose/_action/status");
    nav2_feedback_topic_ = declare_parameter<std::string>(
      "nav2_feedback_topic", "/navigate_to_pose/_action/feedback");
    goal_pose_topic_ = declare_parameter<std::string>("goal_pose_topic", "/goal_pose");
    scan_map_relocalize_trigger_topic_ = declare_parameter<std::string>(
      "scan_map_relocalize_trigger_topic", "/global_scan_initializer/trigger");

    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "Base_footprint");
    global_pose_is_sensor_frame_ = declare_parameter<bool>("global_pose_is_sensor_frame", false);
    global_pose_sensor_frame_ = declare_parameter<std::string>("global_pose_sensor_frame", "Lidar");
    global_pose_yaw_offset_ = declare_parameter<double>("global_pose_yaw_offset", 0.0);
    global_pose_auto_yaw_offset_ = declare_parameter<bool>("global_pose_auto_yaw_offset", false);
    orb_camera_frame_ = declare_parameter<std::string>("orb_camera_frame", "camera_link_optical");
    // Graph Factor is the only map->odom authority in graph-primary mode.
    publish_tf_ = declare_parameter<bool>("publish_tf", true);
    use_initial_pose_ = declare_parameter<bool>("use_initial_pose", false);
    initial_x_ = declare_parameter<double>("initial_x", 0.0);
    initial_y_ = declare_parameter<double>("initial_y", 0.0);
    initial_yaw_ = declare_parameter<double>("initial_yaw", 0.0);
    wait_for_global_pose_before_init_ = declare_parameter<bool>(
      "wait_for_global_pose_before_init", true);
    seed_from_global_pose_ = declare_parameter<bool>("seed_from_global_pose", true);
    tf_future_tolerance_sec_ = declare_parameter<double>("tf_future_tolerance_sec", 0.15);
    use_global_pose_factor_ = declare_parameter<bool>("use_global_pose_factor", true);
    orb_pose_is_base_ = declare_parameter<bool>("orb_pose_is_base", true);
    max_path_length_ = declare_parameter<int>("max_path_length", 2000);
    max_association_dt_ = declare_parameter<double>("max_association_dt", 0.35);
    max_ekf_delta_xy_ = declare_parameter<double>("max_ekf_delta_xy", 2.0);
    max_ekf_delta_yaw_ = declare_parameter<double>("max_ekf_delta_yaw", 1.2);
    min_node_translation_ = declare_parameter<double>("min_node_translation", 0.06);
    min_node_rotation_ = declare_parameter<double>("min_node_rotation", 0.05);
    max_node_dt_ = declare_parameter<double>("max_node_dt", 0.35);
    create_node_on_time_gap_ = declare_parameter<bool>("create_node_on_time_gap", false);
    use_turn_rate_yaw_when_ekf_stalled_ =
      declare_parameter<bool>("use_turn_rate_yaw_when_ekf_stalled", true);
    turn_rate_yaw_min_wz_ = declare_parameter<double>("turn_rate_yaw_min_wz", 0.25);
    turn_rate_yaw_max_pose_delta_ = declare_parameter<double>("turn_rate_yaw_max_pose_delta", 0.01);

    prior_sigma_x_ = declare_parameter<double>("prior_sigma_x", 0.05);
    prior_sigma_y_ = declare_parameter<double>("prior_sigma_y", 0.05);
    prior_sigma_yaw_ = declare_parameter<double>("prior_sigma_yaw", 0.02);
    init_from_scan_map_ = declare_parameter<bool>("init_from_scan_map", true);
    scan_map_init_stable_count_ = declare_parameter<int>("scan_map_init_stable_count", 5);
    scan_map_init_max_xy_spread_ = declare_parameter<double>("scan_map_init_max_xy_spread", 0.12);
    scan_map_init_max_yaw_spread_ = declare_parameter<double>("scan_map_init_max_yaw_spread", 0.15);
    scan_map_init_min_duration_sec_ = declare_parameter<double>("scan_map_init_min_duration_sec", 1.0);
    scan_map_init_timeout_sec_ = declare_parameter<double>("scan_map_init_timeout_sec", 15.0);
    init_prior_use_scan_map_yaw_ = declare_parameter<bool>("init_prior_use_scan_map_yaw", true);
    scan_map_init_ignore_yaw_stability_ =
      declare_parameter<bool>("scan_map_init_ignore_yaw_stability", false);
    init_prior_sigma_x_ = declare_parameter<double>("init_prior_sigma_x", 0.10);
    init_prior_sigma_y_ = declare_parameter<double>("init_prior_sigma_y", 0.10);
    init_prior_sigma_yaw_ = declare_parameter<double>("init_prior_sigma_yaw", 0.50);
    ignore_scan_map_after_init_sec_ =
      declare_parameter<double>("ignore_scan_map_after_init_sec", 0.5);
    fallback_to_initial_pose_on_scan_map_timeout_ =
      declare_parameter<bool>("fallback_to_initial_pose_on_scan_map_timeout", false);

    ekf_sigma_x_ = declare_parameter<double>("ekf_sigma_x", 0.06);
    ekf_sigma_y_ = declare_parameter<double>("ekf_sigma_y", 0.06);
    ekf_sigma_yaw_ = declare_parameter<double>("ekf_sigma_yaw", 0.12);

    orb_sigma_x_ = declare_parameter<double>("orb_sigma_x", 10.0);
    orb_sigma_y_ = declare_parameter<double>("orb_sigma_y", 10.0);
    orb_sigma_yaw_ = declare_parameter<double>("orb_sigma_yaw", 0.20);
    use_orb_translation_ = declare_parameter<bool>("use_orb_translation", false);
    use_orb_yaw_ = declare_parameter<bool>("use_orb_yaw", true);
    estimate_orb_scale_ = declare_parameter<bool>("estimate_orb_scale", false);
    max_orb_delta_xy_ = declare_parameter<double>("max_orb_delta_xy", 1.0);
    max_orb_delta_yaw_ = declare_parameter<double>("max_orb_delta_yaw", 0.5);
    max_orb_dt_ = declare_parameter<double>("max_orb_dt", 1.0);
    max_orb_ekf_disagreement_xy_ = declare_parameter<double>(
      "max_orb_ekf_disagreement_xy", 0.50);
    max_orb_ekf_disagreement_yaw_ = declare_parameter<double>(
      "max_orb_ekf_disagreement_yaw", 0.35);
    reject_orb_when_turning_ = declare_parameter<bool>("reject_orb_when_turning", true);
    orb_turn_reject_wz_ = declare_parameter<double>("orb_turn_reject_wz", 0.25);
    orb_turn_block_sec_ = declare_parameter<double>("orb_turn_block_sec", 1.5);
    orb_pose_is_tcw_ = declare_parameter<bool>("orb_pose_is_tcw", false);
    require_orb_tracking_state_ = declare_parameter<bool>("require_orb_tracking_state", true);

    scan_odom_sigma_x_ = declare_parameter<double>("scan_odom_sigma_x", 0.70);
    scan_odom_sigma_y_ = declare_parameter<double>("scan_odom_sigma_y", 0.70);
    scan_odom_sigma_yaw_ = declare_parameter<double>("scan_odom_sigma_yaw", 0.40);
    max_scan_delta_xy_ = declare_parameter<double>("max_scan_delta_xy", 1.5);
    max_scan_delta_yaw_ = declare_parameter<double>("max_scan_delta_yaw", 0.8);
    max_scan_dt_ = declare_parameter<double>("max_scan_dt", 1.0);
    max_scan_ekf_disagreement_xy_ = declare_parameter<double>(
      "max_scan_ekf_disagreement_xy", 0.15);
    max_scan_ekf_disagreement_yaw_ = declare_parameter<double>(
      "max_scan_ekf_disagreement_yaw", 0.10);
    scan_odom_max_yaw_rate_ = declare_parameter<double>("scan_odom_max_yaw_rate", 0.25);
    reject_scan_odom_when_not_forward_ =
      declare_parameter<bool>("reject_scan_odom_when_not_forward", true);
    scan_odom_min_forward_vx_ = declare_parameter<double>("scan_odom_min_forward_vx", 0.03);

    scan_map_sigma_x_ = declare_parameter<double>("scan_map_sigma_x", 0.35);
    scan_map_sigma_y_ = declare_parameter<double>("scan_map_sigma_y", 0.35);
    scan_map_sigma_yaw_ = declare_parameter<double>("scan_map_sigma_yaw", 10.0);
    scan_map_untrusted_yaw_sigma_ = declare_parameter<double>("scan_map_untrusted_yaw_sigma", 10.0);
    trust_scan_map_yaw_default_ = declare_parameter<bool>("trust_scan_map_yaw", false);
    enable_home_scan_map_correction_ =
      declare_parameter<bool>("enable_home_scan_map_correction", true);
    scan_map_only_near_home_ = declare_parameter<bool>("scan_map_only_near_home", false);
    home_map_x_ = declare_parameter<double>("home_map_x", 0.0);
    home_map_y_ = declare_parameter<double>("home_map_y", 0.0);
    home_scan_map_radius_ = declare_parameter<double>("home_scan_map_radius", 0.75);
    home_scan_map_sigma_xy_ = declare_parameter<double>("home_scan_map_sigma_xy", 0.10);
    home_scan_map_max_innovation_xy_ =
      declare_parameter<double>("home_scan_map_max_innovation_xy", 0.60);
    reject_scan_map_when_turning_ = declare_parameter<bool>("reject_scan_map_when_turning", true);
    scan_map_turn_reject_wz_ = declare_parameter<double>("scan_map_turn_reject_wz", 0.25);
    scan_map_turn_block_sec_ = declare_parameter<double>("scan_map_turn_block_sec", 1.5);
    reject_scan_map_when_not_forward_ = declare_parameter<bool>(
      "reject_scan_map_when_not_forward", true);
    scan_map_min_forward_vx_ = declare_parameter<double>("scan_map_min_forward_vx", 0.03);
    reject_scan_map_backward_when_forward_ =
      declare_parameter<bool>("reject_scan_map_backward_when_forward", true);
    scan_map_backward_reject_min_vx_ =
      declare_parameter<double>("scan_map_backward_reject_min_vx", 0.03);
    scan_map_backward_reject_distance_ =
      declare_parameter<double>("scan_map_backward_reject_distance", 0.10);
    scan_map_selective_updates_ = declare_parameter<bool>("scan_map_selective_updates", true);
    scan_map_update_min_interval_sec_ =
      declare_parameter<double>("scan_map_update_min_interval_sec", 2.0);
    scan_map_update_stable_count_ = declare_parameter<int>("scan_map_update_stable_count", 5);
    scan_map_update_max_innovation_spread_xy_ =
      declare_parameter<double>("scan_map_update_max_innovation_spread_xy", 0.12);
    scan_map_update_max_innovation_spread_yaw_ =
      declare_parameter<double>("scan_map_update_max_innovation_spread_yaw", 0.35);
    scan_map_update_min_duration_sec_ =
      declare_parameter<double>("scan_map_update_min_duration_sec", 0.35);
    enable_scan_map_candidate_gate_ =
      declare_parameter<bool>("enable_scan_map_candidate_gate", true);
    scan_map_candidate_required_count_ =
      declare_parameter<int>("scan_map_candidate_required_count", 5);
    scan_map_candidate_window_sec_ =
      declare_parameter<double>("scan_map_candidate_window_sec", 1.0);
    scan_map_candidate_max_spread_xy_ =
      declare_parameter<double>("scan_map_candidate_max_spread_xy", 0.15);
    scan_map_candidate_max_spread_yaw_ =
      declare_parameter<double>("scan_map_candidate_max_spread_yaw", 0.15);
    scan_map_max_single_correction_xy_ =
      declare_parameter<double>("scan_map_max_single_correction_xy", 0.30);
    scan_map_max_single_correction_yaw_ =
      declare_parameter<double>("scan_map_max_single_correction_yaw", 0.25);
    scan_map_reject_correction_xy_ =
      declare_parameter<double>("scan_map_reject_correction_xy", 0.80);
    scan_map_reject_correction_yaw_ =
      declare_parameter<double>("scan_map_reject_correction_yaw", 0.80);
    scan_map_use_xy_only_when_yaw_bad_ =
      declare_parameter<bool>("scan_map_use_xy_only_when_yaw_bad", true);
    scan_map_xy_only_yaw_reject_threshold_ =
      declare_parameter<double>("scan_map_xy_only_yaw_reject_threshold", 0.35);
    scan_map_suspect_reject_count_threshold_ =
      declare_parameter<int>("scan_map_suspect_reject_count_threshold", 10);
    scan_map_suspect_cooldown_sec_ =
      declare_parameter<double>("scan_map_suspect_cooldown_sec", 3.0);
    scan_map_relocalize_cooldown_sec_ =
      declare_parameter<double>("scan_map_relocalize_cooldown_sec", 5.0);
    enable_scan_map_auto_relocalization_ =
      declare_parameter<bool>("enable_scan_map_auto_relocalization", false);
    enable_scan_map_bounded_correction_ =
      declare_parameter<bool>("enable_scan_map_bounded_correction", true);
    max_amcl_jump_xy_ = declare_parameter<double>("max_amcl_jump_xy", 2.0);
    max_amcl_jump_yaw_ = declare_parameter<double>("max_amcl_jump_yaw", 1.0);
    max_global_innovation_xy_ = declare_parameter<double>("max_global_innovation_xy", 0.40);
    max_global_innovation_yaw_ = declare_parameter<double>("max_global_innovation_yaw", 3.20);
    use_amcl_covariance_ = declare_parameter<bool>("use_amcl_covariance", true);
    min_covariance_sigma_ = declare_parameter<double>("min_covariance_sigma", 0.02);
    max_covariance_sigma_ = declare_parameter<double>("max_covariance_sigma", 2.0);

    gtsam::ISAM2Params params;
    params.relinearizeThreshold = declare_parameter<double>("relinearize_threshold", 0.1);
    params.relinearizeSkip = declare_parameter<int>("relinearize_skip", 3);
    isam_ = std::make_unique<gtsam::ISAM2>(params);

    if (!use_orb_translation_ && (orb_sigma_x_ < 1.0 || orb_sigma_y_ < 1.0)) {
      RCLCPP_WARN(
        get_logger(),
        "use_orb_translation=false but orb_sigma_x/y are small. "
        "Clamping ORB translation sigmas to 10.0 to avoid locking X/Y.");
      orb_sigma_x_ = std::max(orb_sigma_x_, 10.0);
      orb_sigma_y_ = std::max(orb_sigma_y_, 10.0);
    }

    const double robust_huber_k = declare_parameter<double>("robust_huber_k", 1.345);
    prior_noise_ = noise(prior_sigma_x_, prior_sigma_y_, prior_sigma_yaw_);
    ekf_noise_ = noise(ekf_sigma_x_, ekf_sigma_y_, ekf_sigma_yaw_);
    orb_noise_ = robustNoise(orb_sigma_x_, orb_sigma_y_, orb_sigma_yaw_, robust_huber_k);
    scan_odom_noise_ = robustNoise(
      scan_odom_sigma_x_, scan_odom_sigma_y_, scan_odom_sigma_yaw_, robust_huber_k);
    scan_map_noise_ = robustNoise(
      scan_map_sigma_x_, scan_map_sigma_y_, scan_map_sigma_yaw_, robust_huber_k);

    const auto loaded_map = loadOccupancyMapFromYaml(map_yaml_file_);
    if (loaded_map) {
      occupancy_map_ = *loaded_map;
      map_bounds_ = *boundsFromOccupancyMap(occupancy_map_);
      RCLCPP_INFO(
        get_logger(),
        "Loaded occupancy map from %s image=%s size=%dx%d res=%.3f "
        "bounds x=[%.3f, %.3f], y=[%.3f, %.3f], margin=%.3f gate_radius=%.3f",
        map_yaml_file_.c_str(), occupancy_map_.image_file.c_str(),
        occupancy_map_.width, occupancy_map_.height,
        occupancy_map_.resolution,
        map_bounds_.min_x, map_bounds_.max_x,
        map_bounds_.min_y, map_bounds_.max_y,
        scan_map_bounds_margin_, scan_map_occupancy_gate_radius_);
    } else if (enforce_scan_map_bounds_) {
      RCLCPP_WARN(
        get_logger(),
        "enforce_scan_map_bounds=true but map_yaml_file is empty/unreadable (%s). "
        "Scan-to-map occupancy rejection is disabled.",
        map_yaml_file_.c_str());
    }

    graph_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(graph_odom_topic_, 20);
    graph_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(graph_pose_topic_, 20);
    graph_path_pub_ = create_publisher<nav_msgs::msg::Path>(graph_path_topic_, 10);
    graph_stats_pub_ = create_publisher<std_msgs::msg::String>(graph_stats_topic_, 10);
    scan_map_relocalize_trigger_pub_ =
      create_publisher<std_msgs::msg::Bool>(scan_map_relocalize_trigger_topic_, 10);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    path_msg_.header.frame_id = map_frame_;

    ekf_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      ekf_topic_, 50, std::bind(&GtsamPose2FusionNode::ekfCallback, this, std::placeholders::_1));
    if (use_global_pose_factor_) {
      amcl_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        global_pose_topic_.empty() ? amcl_topic_ : global_pose_topic_, 20,
        std::bind(&GtsamPose2FusionNode::amclCallback, this, std::placeholders::_1));
    }
    orb_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      orb_topic_, 20, std::bind(&GtsamPose2FusionNode::orbCallback, this, std::placeholders::_1));
    if (!orb_tracking_state_topic_.empty()) {
      orb_tracking_state_sub_ = create_subscription<std_msgs::msg::Int32>(
        orb_tracking_state_topic_, 20,
        std::bind(&GtsamPose2FusionNode::orbTrackingStateCallback, this, std::placeholders::_1));
    }
    if (!scan_odom_topic_.empty()) {
      scan_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        scan_odom_topic_, 30,
        std::bind(&GtsamPose2FusionNode::scanOdomCallback, this, std::placeholders::_1));
    }
    if (!turn_rate_topic_.empty()) {
      turn_rate_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        turn_rate_topic_, 50,
        std::bind(&GtsamPose2FusionNode::turnRateCallback, this, std::placeholders::_1));
    }
    if (enable_run_logging_) {
      initRunLogging();
      if (!nav2_status_topic_.empty()) {
        nav2_status_sub_ = create_subscription<action_msgs::msg::GoalStatusArray>(
          nav2_status_topic_, 20,
          std::bind(&GtsamPose2FusionNode::nav2StatusCallback, this, std::placeholders::_1));
      }
      if (!nav2_feedback_topic_.empty()) {
        nav2_feedback_sub_ =
          create_subscription<nav2_msgs::action::NavigateToPose::Impl::FeedbackMessage>(
          nav2_feedback_topic_, 20,
          std::bind(&GtsamPose2FusionNode::nav2FeedbackCallback, this, std::placeholders::_1));
      }
      if (!goal_pose_topic_.empty()) {
        goal_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
          goal_pose_topic_, 20,
          std::bind(&GtsamPose2FusionNode::goalPoseCallback, this, std::placeholders::_1));
      }
    }

    stats_timer_ = create_wall_timer(
      std::chrono::seconds(1), std::bind(&GtsamPose2FusionNode::publishStats, this));

    RCLCPP_INFO(get_logger(), "GTSAM Pose2 iSAM2 factor graph node");
    RCLCPP_INFO(get_logger(), "EKF relative topic: %s", ekf_topic_.c_str());
    RCLCPP_INFO(get_logger(), "ORB raw relative topic: %s", orb_topic_.c_str());
    RCLCPP_INFO(get_logger(), "scan odom relative topic: %s", scan_odom_topic_.empty() ? "disabled" : scan_odom_topic_.c_str());
    RCLCPP_INFO(get_logger(), "turn gating topic: %s", turn_rate_topic_.empty() ? "disabled" : turn_rate_topic_.c_str());
    RCLCPP_INFO(
      get_logger(),
      "scan odom gate: sigma=(%.3f, %.3f, %.3f), max_ekf_disagreement=(%.3f m, %.3f rad), max_yaw_rate=%.3f rad/s",
      scan_odom_sigma_x_, scan_odom_sigma_y_, scan_odom_sigma_yaw_,
      max_scan_ekf_disagreement_xy_, max_scan_ekf_disagreement_yaw_,
      scan_odom_max_yaw_rate_);
    RCLCPP_INFO(
      get_logger(), "global absolute pose topic: %s",
      use_global_pose_factor_ ?
      (global_pose_topic_.empty() ? amcl_topic_.c_str() : global_pose_topic_.c_str()) :
      "disabled");
    if (global_pose_is_sensor_frame_) {
      RCLCPP_INFO(
        get_logger(),
        "scan-to-map pose correction enabled: treating global pose as %s and converting to %s",
        global_pose_sensor_frame_.c_str(), base_frame_.c_str());
    }
    RCLCPP_INFO(get_logger(), "output graph odom: %s, path: %s", graph_odom_topic_.c_str(), graph_path_topic_.c_str());
    if (estimate_orb_scale_) {
      RCLCPP_WARN(get_logger(), "estimate_orb_scale=true requested, but scale estimation is phase 2; current patch uses configured ORB noise only");
    }
    RCLCPP_INFO(
      get_logger(), "graph diagnostics: %s, output=%s",
      enable_diagnostics_ ? "enabled" : "disabled", diagnostics_output_dir_.c_str());
    if (enable_run_logging_) {
      RCLCPP_INFO(get_logger(), "graph run logging: %s", run_log_dir_.c_str());
    }
  }

  ~GtsamPose2FusionNode() override
  {
    flushRunLogs();
    if (!enable_diagnostics_ || diagnostics_saved_ || full_graph_.empty() || result_.empty()) {
      return;
    }
    diagnostics_saved_ = true;
    try {
      const orb3_agv_sim::GraphTiming timing{build_graph_sec_, optimize_sec_};
      const auto report = orb3_agv_sim::runGraphDiagnostics(
        full_graph_, initial_values_, result_, factor_types_,
        diagnostics_output_dir_, timing);
      RCLCPP_INFO(
        get_logger(),
        "Graph diagnostics saved to %s (linearize=%.4fs diagnostics=%.4fs warnings=%zu)",
        diagnostics_output_dir_.c_str(), report.linearize_sec,
        report.diagnostics_sec, report.warnings.size());
      for (const auto & warning : report.warnings) {
        RCLCPP_WARN(get_logger(), "Graph diagnostics: %s", warning.c_str());
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Graph diagnostics failed: %s", e.what());
    }
  }

private:
  enum class ScanMapTrustState
  {
    TRUSTED,
    CANDIDATE,
    SUSPECT,
    RELOCALIZING,
  };

  struct GraphNode
  {
    size_t index;
    double stamp;
    gtsam::Pose2 ekf_pose;
  };

  struct ScanMapInitSample
  {
    double stamp;
    gtsam::Pose2 pose;
  };

  struct ScanMapInitMetrics
  {
    double xy_spread{std::numeric_limits<double>::infinity()};
    double yaw_spread{std::numeric_limits<double>::infinity()};
    double duration{0.0};
    bool stable{false};
  };

  struct ScanMapUpdateSample
  {
    double stamp;
    gtsam::Pose2 innovation;
  };

  struct ScanMapUpdateMetrics
  {
    double xy_spread{std::numeric_limits<double>::infinity()};
    double yaw_spread{std::numeric_limits<double>::infinity()};
    double duration{0.0};
    size_t count{0};
    bool stable{false};
  };

  static gtsam::noiseModel::Diagonal::shared_ptr noise(double sx, double sy, double syaw)
  {
    return gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(sx, sy, syaw));
  }

  static gtsam::noiseModel::Base::shared_ptr robustNoise(
    double sx, double sy, double syaw, double huber_k)
  {
    return gtsam::noiseModel::Robust::Create(
      gtsam::noiseModel::mEstimator::Huber::Create(huber_k),
      noise(sx, sy, syaw));
  }

  std::string makeRunStamp() const
  {
    const auto now = std::chrono::system_clock::now();
    const std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::tm tm {};
    localtime_r(&t, &tm);
    std::ostringstream out;
    out << std::put_time(&tm, "%Y%m%d_%H%M%S");
    return out.str();
  }

  void initRunLogging()
  {
    try {
      const std::string tag = run_log_tag_.empty() ? "graph" : run_log_tag_;
      run_log_dir_ = (std::filesystem::path(run_log_root_) /
        (tag + "_" + makeRunStamp())).string();
      std::filesystem::create_directories(run_log_dir_);

      pose_log_.open(std::filesystem::path(run_log_dir_) / "poses.csv", std::ios::out);
      factor_log_.open(std::filesystem::path(run_log_dir_) / "factors.csv", std::ios::out);
      nav2_log_.open(std::filesystem::path(run_log_dir_) / "nav2_status.csv", std::ios::out);
      nav2_feedback_log_.open(
        std::filesystem::path(run_log_dir_) / "nav2_feedback.csv", std::ios::out);
      goal_log_.open(std::filesystem::path(run_log_dir_) / "goals.csv", std::ios::out);
      scan_map_debug_log_.open(
        std::filesystem::path(run_log_dir_) / "scan_to_map_debug.csv", std::ios::out);

      pose_log_ << "wall_time,stamp,source,event,node,x,y,yaw,vx,wz,graph_x,graph_y,graph_yaw\n";
      factor_log_ << "wall_time,stamp,type,accepted,reason,from_node,to_node,dx,dy,dyaw\n";
      nav2_log_ << "wall_time,stamp,status_count,statuses,last_status,last_status_name\n";
      nav2_feedback_log_ <<
        "wall_time,stamp,frame,x,y,yaw,distance_remaining,navigation_time,estimated_time_remaining,recoveries\n";
      goal_log_ << "wall_time,stamp,frame,x,y,yaw\n";
      scan_map_debug_log_ <<
        "wall_time,stamp,node,trust_yaw,accepted,reason,"
        "occupancy_u,occupancy_v,occupancy_value,occupancy_status,"
        "raw_x,raw_y,raw_yaw,"
        "corrected_x,corrected_y,corrected_yaw,"
        "used_x,used_y,used_yaw,"
        "predicted_x,predicted_y,predicted_yaw,"
        "innovation_x,innovation_y,innovation_yaw,"
        "ekf_vx,ekf_wz,graph_x,graph_y,graph_yaw,"
        "scan_map_state,candidate_count,candidate_spread_xy,"
        "candidate_spread_yaw,triggered_relocalization\n";

      std::ofstream config(std::filesystem::path(run_log_dir_) / "run_config.json");
      config << std::boolalpha
             << "{\n"
             << "  \"ekf_odom_topic\": \"" << ekf_topic_ << "\",\n"
             << "  \"global_pose_topic\": \"" << global_pose_topic_ << "\",\n"
             << "  \"orb_pose_topic\": \"" << orb_topic_ << "\",\n"
             << "  \"scan_odom_topic\": \"" << scan_odom_topic_ << "\",\n"
             << "  \"nav2_status_topic\": \"" << nav2_status_topic_ << "\",\n"
             << "  \"nav2_feedback_topic\": \"" << nav2_feedback_topic_ << "\",\n"
             << "  \"goal_pose_topic\": \"" << goal_pose_topic_ << "\",\n"
             << "  \"publish_tf\": " << publish_tf_ << ",\n"
             << "  \"init_from_scan_map\": " << init_from_scan_map_ << ",\n"
             << "  \"scan_map_init_stable_count\": " << scan_map_init_stable_count_ << ",\n"
             << "  \"scan_map_init_max_xy_spread\": " << scan_map_init_max_xy_spread_ << ",\n"
             << "  \"scan_map_init_min_duration_sec\": " << scan_map_init_min_duration_sec_ << ",\n"
             << "  \"init_prior_use_scan_map_yaw\": " << init_prior_use_scan_map_yaw_ << ",\n"
             << "  \"use_orb_translation\": " << use_orb_translation_ << ",\n"
             << "  \"trust_scan_map_yaw\": " << trust_scan_map_yaw_default_ << ",\n"
             << "  \"scan_map_only_near_home\": " << scan_map_only_near_home_ << ",\n"
             << "  \"ekf_sigma\": [" << ekf_sigma_x_ << ", " << ekf_sigma_y_ << ", " << ekf_sigma_yaw_ << "],\n"
             << "  \"orb_sigma\": [" << orb_sigma_x_ << ", " << orb_sigma_y_ << ", " << orb_sigma_yaw_ << "],\n"
             << "  \"scan_odom_sigma\": [" << scan_odom_sigma_x_ << ", " << scan_odom_sigma_y_ << ", " << scan_odom_sigma_yaw_ << "],\n"
             << "  \"scan_map_sigma\": [" << scan_map_sigma_x_ << ", " << scan_map_sigma_y_ << ", " << scan_map_sigma_yaw_ << "]\n"
             << "}\n";
    } catch (const std::exception & e) {
      enable_run_logging_ = false;
      RCLCPP_WARN(get_logger(), "Disable graph run logging: %s", e.what());
    }
  }

  void logPose(
    const std::string & source,
    const std::string & event,
    double stamp,
    const gtsam::Pose2 & pose,
    std::optional<size_t> node_index = std::nullopt,
    double vx = std::numeric_limits<double>::quiet_NaN(),
    double wz = std::numeric_limits<double>::quiet_NaN())
  {
    if (!enable_run_logging_ || !pose_log_.is_open() || !finitePose(pose)) {
      return;
    }
    const auto last_it = last_pose_log_stamp_by_source_.find(source);
    if (
      event != "initialized" && run_pose_log_min_interval_sec_ > 0.0 &&
      last_it != last_pose_log_stamp_by_source_.end() &&
      stamp - last_it->second < run_pose_log_min_interval_sec_)
    {
      return;
    }
    last_pose_log_stamp_by_source_[source] = stamp;
    pose_log_ << std::fixed << std::setprecision(6)
              << nowSec() << "," << stamp << ","
              << source << "," << event << ",";
    if (node_index) {
      pose_log_ << *node_index;
    }
    pose_log_ << "," << pose.x() << "," << pose.y() << "," << pose.theta()
              << "," << vx << "," << wz << ","
              << latest_optimized_pose_.x() << ","
              << latest_optimized_pose_.y() << ","
              << latest_optimized_pose_.theta() << "\n";
  }

  void logFactor(
    const std::string & type,
    bool accepted,
    const std::string & reason,
    double stamp,
    std::optional<size_t> from_index,
    std::optional<size_t> to_index,
    const gtsam::Pose2 & delta)
  {
    if (!enable_run_logging_ || !factor_log_.is_open()) {
      return;
    }
    if (!accepted && run_rejected_factor_log_min_interval_sec_ > 0.0) {
      const std::string key = type + ":" + reason;
      const double now = nowSec();
      const auto last_it = last_rejected_factor_log_time_.find(key);
      if (
        last_it != last_rejected_factor_log_time_.end() &&
        now - last_it->second < run_rejected_factor_log_min_interval_sec_)
      {
        return;
      }
      last_rejected_factor_log_time_[key] = now;
    }
    factor_log_ << std::fixed << std::setprecision(6)
                << nowSec() << "," << stamp << ","
                << type << "," << (accepted ? 1 : 0) << ","
                << csvSafe(reason) << ",";
    if (from_index) {
      factor_log_ << *from_index;
    }
    factor_log_ << ",";
    if (to_index) {
      factor_log_ << *to_index;
    }
    factor_log_ << "," << delta.x() << "," << delta.y() << "," << delta.theta() << "\n";
  }

  static const char * scanMapTrustStateName(ScanMapTrustState state)
  {
    switch (state) {
      case ScanMapTrustState::TRUSTED: return "TRUSTED";
      case ScanMapTrustState::CANDIDATE: return "CANDIDATE";
      case ScanMapTrustState::SUSPECT: return "SUSPECT";
      case ScanMapTrustState::RELOCALIZING: return "RELOCALIZING";
    }
    return "UNKNOWN";
  }

  void setScanMapTrustState(ScanMapTrustState state, const std::string & reason)
  {
    if (scan_map_trust_state_ == state) {
      return;
    }
    RCLCPP_WARN(
      get_logger(), "scan_map_state %s -> %s: %s",
      scanMapTrustStateName(scan_map_trust_state_),
      scanMapTrustStateName(state),
      reason.c_str());
    scan_map_trust_state_ = state;
  }

  void triggerScanMapRelocalization(double stamp, const std::string & reason)
  {
    if (!enable_scan_map_auto_relocalization_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Skip automatic scan-map relocalization while mission is running: %s",
        reason.c_str());
      return;
    }
    if (stamp - last_scan_map_relocalize_trigger_stamp_ < scan_map_relocalize_cooldown_sec_) {
      return;
    }
    last_scan_map_relocalize_trigger_stamp_ = stamp;
    scan_map_suspect_until_stamp_ = stamp + scan_map_suspect_cooldown_sec_;
    setScanMapTrustState(ScanMapTrustState::RELOCALIZING, reason);
    scan_map_update_buffer_.clear();
    last_scan_map_triggered_relocalization_ = true;
    std_msgs::msg::Bool msg;
    msg.data = true;
    scan_map_relocalize_trigger_pub_->publish(msg);
    RCLCPP_ERROR(
      get_logger(),
      "Trigger scan-map relocalization: %s; cooldown %.2fs",
      reason.c_str(), scan_map_suspect_cooldown_sec_);
  }

  void writePoseFields(
    std::ofstream & stream,
    const std::optional<gtsam::Pose2> & pose)
  {
    if (pose && finitePose(*pose)) {
      stream << pose->x() << "," << pose->y() << "," << pose->theta();
    } else {
      stream << "nan,nan,nan";
    }
  }

  void logScanMapDebug(
    double stamp,
    std::optional<size_t> node_index,
    bool trust_yaw,
    bool accepted,
    const std::string & reason,
    const gtsam::Pose2 & raw_pose,
    const std::optional<gtsam::Pose2> & corrected_pose,
    const std::optional<gtsam::Pose2> & used_pose,
    const std::optional<gtsam::Pose2> & predicted_pose,
    const std::optional<gtsam::Pose2> & innovation,
    const OccupancyGateResult & occupancy = OccupancyGateResult{})
  {
    if (!enable_run_logging_ || !scan_map_debug_log_.is_open()) {
      return;
    }
    if (!accepted && run_scan_map_debug_min_interval_sec_ > 0.0) {
      const double now = nowSec();
      if (now - last_scan_map_debug_log_time_ < run_scan_map_debug_min_interval_sec_) {
        return;
      }
      last_scan_map_debug_log_time_ = now;
    }
    OccupancyGateResult occupancy_to_log = occupancy;
    if (occupancy_to_log.u < 0 && occupancy_to_log.v < 0) {
      if (corrected_pose) {
        occupancy_to_log = scanMapOccupancyGate(*corrected_pose);
      } else {
        occupancy_to_log = scanMapOccupancyGate(raw_pose);
      }
    }
    scan_map_debug_log_ << std::fixed << std::setprecision(6)
                        << nowSec() << "," << stamp << ",";
    if (node_index) {
      scan_map_debug_log_ << *node_index;
    }
    scan_map_debug_log_ << "," << (trust_yaw ? 1 : 0)
                        << "," << (accepted ? 1 : 0)
                        << "," << csvSafe(reason) << ",";
    scan_map_debug_log_ << occupancy_to_log.u << ","
                        << occupancy_to_log.v << ","
                        << occupancy_to_log.value << ","
                        << csvSafe(occupancy_to_log.reason) << ",";
    writePoseFields(scan_map_debug_log_, raw_pose);
    scan_map_debug_log_ << ",";
    writePoseFields(scan_map_debug_log_, corrected_pose);
    scan_map_debug_log_ << ",";
    writePoseFields(scan_map_debug_log_, used_pose);
    scan_map_debug_log_ << ",";
    writePoseFields(scan_map_debug_log_, predicted_pose);
    scan_map_debug_log_ << ",";
    writePoseFields(scan_map_debug_log_, innovation);
    scan_map_debug_log_ << "," << latest_ekf_vx_
                        << "," << latest_ekf_yaw_rate_
                        << "," << latest_optimized_pose_.x()
                        << "," << latest_optimized_pose_.y()
                        << "," << latest_optimized_pose_.theta()
                        << "," << scanMapTrustStateName(scan_map_trust_state_)
                        << "," << scan_map_update_buffer_.size()
                        << "," << last_scan_map_candidate_spread_xy_
                        << "," << last_scan_map_candidate_spread_yaw_
                        << "," << (last_scan_map_triggered_relocalization_ ? 1 : 0)
                        << "\n";
    last_scan_map_triggered_relocalization_ = false;
  }

  double nowSec()
  {
    return get_clock()->now().seconds();
  }

  static std::string csvSafe(const std::string & text)
  {
    std::string out = "\"";
    for (const char c : text) {
      if (c == '"') {
        out += "\"\"";
      } else {
        out += c;
      }
    }
    out += "\"";
    return out;
  }

  std::optional<GraphNode> nodeByIndex(size_t index) const
  {
    const auto it = std::find_if(
      nodes_.begin(), nodes_.end(),
      [index](const GraphNode & node) {return node.index == index;});
    if (it == nodes_.end()) {
      return std::nullopt;
    }
    return *it;
  }

  bool consistentWithEkf(
    size_t from_index, size_t to_index, const gtsam::Pose2 & measured,
    double max_xy, double max_yaw, const std::string & source)
  {
    const auto from = nodeByIndex(from_index);
    const auto to = nodeByIndex(to_index);
    if (!from || !to) {
      return false;
    }
    const gtsam::Pose2 ekf_delta = from->ekf_pose.between(to->ekf_pose);
    const gtsam::Pose2 disagreement = ekf_delta.between(measured);
    const double dxy = deltaNormXY(disagreement);
    const double dyaw = std::fabs(normalizeAngle(disagreement.theta()));
    if (dxy > max_xy || dyaw > max_yaw) {
      RCLCPP_WARN(
        get_logger(),
        "Reject %s factor: disagreement with EKF dxy=%.3f/%.3f dyaw=%.3f/%.3f",
        source.c_str(), dxy, max_xy, dyaw, max_yaw);
      return false;
    }
    return true;
  }

  std::optional<gtsam::Pose2> ekfDisagreement(
    size_t from_index, size_t to_index, const gtsam::Pose2 & measured) const
  {
    const auto from = nodeByIndex(from_index);
    const auto to = nodeByIndex(to_index);
    if (!from || !to) {
      return std::nullopt;
    }
    const gtsam::Pose2 ekf_delta = from->ekf_pose.between(to->ekf_pose);
    return ekf_delta.between(measured);
  }

  std::optional<GraphNode> nearestNode(double stamp) const
  {
    if (nodes_.empty() || !std::isfinite(stamp)) {
      return std::nullopt;
    }
    const auto best = std::min_element(
      nodes_.begin(), nodes_.end(),
      [stamp](const GraphNode & a, const GraphNode & b) {
        return std::fabs(a.stamp - stamp) < std::fabs(b.stamp - stamp);
      });
    if (best == nodes_.end() || std::fabs(best->stamp - stamp) > max_association_dt_) {
      return std::nullopt;
    }
    return *best;
  }

  bool validDelta(
    const gtsam::Pose2 & delta, double max_xy, double max_yaw,
    const std::string & source, const std::string & reject_prefix)
  {
    if (!finitePose(delta)) {
      RCLCPP_WARN(get_logger(), "%s %s factor: delta has NaN/Inf", reject_prefix.c_str(), source.c_str());
      return false;
    }
    const double dxy = deltaNormXY(delta);
    const double dyaw = std::fabs(normalizeAngle(delta.theta()));
    if (dxy > max_xy) {
      RCLCPP_WARN(get_logger(), "%s %s factor: delta xy %.3f > %.3f", reject_prefix.c_str(), source.c_str(), dxy, max_xy);
      return false;
    }
    if (dyaw > max_yaw) {
      RCLCPP_WARN(get_logger(), "%s %s factor: delta yaw %.3f > %.3f", reject_prefix.c_str(), source.c_str(), dyaw, max_yaw);
      return false;
    }
    return true;
  }

  void turnRateCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    const double stamp = stampToSec(msg->header.stamp);
    if (!std::isfinite(stamp)) {
      return;
    }
    latest_turn_rate_stamp_ = stamp;
    latest_turn_rate_wz_ = std::isfinite(msg->twist.twist.angular.z) ?
      msg->twist.twist.angular.z : 0.0;
  }

  void ekfCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    if (!finitePoseMsg(msg->pose.pose)) {
      RCLCPP_WARN(get_logger(), "Reject EKF odom: pose has NaN/Inf");
      return;
    }

    const double stamp = stampToSec(msg->header.stamp);
    const gtsam::Pose2 ekf_pose = pose2FromPose(msg->pose.pose);
    if (!finitePose(ekf_pose)) {
      RCLCPP_WARN(get_logger(), "Reject EKF odom: Pose2 has NaN/Inf");
      return;
    }
    latest_ekf_vx_ = std::isfinite(msg->twist.twist.linear.x) ? msg->twist.twist.linear.x : 0.0;
    const double ekf_wz_msg = std::isfinite(msg->twist.twist.angular.z) ?
      msg->twist.twist.angular.z : 0.0;
    logPose("ekf", "received", stamp, ekf_pose, std::nullopt, latest_ekf_vx_, ekf_wz_msg);

    if (!initialized_) {
      latest_preinit_ekf_pose_ = ekf_pose;
      latest_preinit_ekf_stamp_ = stamp;
      has_latest_preinit_ekf_pose_ = true;
      if (!has_initial_ekf_pose_) {
        initial_ekf_pose_ = ekf_pose;
        has_initial_ekf_pose_ = true;
      }
      if (init_from_scan_map_) {
        maybeFallbackScanMapInitTimeout(ekf_pose, stamp, msg->header.stamp);
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Waiting for stable scan-to-map pose before initializing graph prior. "
          "EKF odom remains local and will become the relative backbone after init.");
        return;
      }
      if (wait_for_global_pose_before_init_ && !has_global_seed_) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Waiting for first scan-to-map pose before initializing graph. "
          "This prevents map->odom from being seeded at the wrong map origin.");
        return;
      }
      initializeGraph(ekf_pose, stamp, msg->header.stamp);
      return;
    }

    const gtsam::Pose2 delta_ekf = last_ekf_pose_.between(ekf_pose);
    double yaw_rate_from_turn_topic = 0.0;
    if (
      latest_turn_rate_stamp_ >= 0.0 &&
      std::isfinite(stamp - latest_turn_rate_stamp_) &&
      std::fabs(stamp - latest_turn_rate_stamp_) <= turn_rate_timeout_sec_)
    {
      yaw_rate_from_turn_topic = std::fabs(latest_turn_rate_wz_);
    }
    const double ekf_dt = stamp - latest_ekf_stamp_;
    double yaw_rate_from_pose = 0.0;
    if (std::isfinite(ekf_dt) && ekf_dt >= 1e-3) {
      yaw_rate_from_pose = std::fabs(normalizeAngle(delta_ekf.theta()) / ekf_dt);
    }
    // /wheel_vel angular.z is used only for turn gating, not fused into EKF yaw.
    // Pose differencing still catches turns if the wheel yaw-rate message is delayed.
    const double yaw_rate = std::max(
      std::isfinite(yaw_rate_from_turn_topic) ? yaw_rate_from_turn_topic : 0.0,
      std::isfinite(yaw_rate_from_pose) ? yaw_rate_from_pose : 0.0);
    latest_ekf_yaw_rate_ = yaw_rate;
    latest_ekf_stamp_ = stamp;
    if (reject_orb_when_turning_ && yaw_rate > orb_turn_reject_wz_) {
      orb_reject_until_stamp_ = std::max(orb_reject_until_stamp_, stamp + orb_turn_block_sec_);
      resetOrbAnchor("EKF reports active turning; ORB chain will restart after turn");
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Temporarily rejecting ORB factors while robot turns: wz=%.3f rad/s > %.3f",
        yaw_rate, orb_turn_reject_wz_);
    }
    if (reject_scan_map_when_turning_ && yaw_rate > scan_map_turn_reject_wz_) {
      scan_map_reject_until_stamp_ = std::max(
        scan_map_reject_until_stamp_, stamp + scan_map_turn_block_sec_);
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Temporarily rejecting scan-to-map factors while robot turns: wz=%.3f rad/s > %.3f",
        yaw_rate, scan_map_turn_reject_wz_);
    }
    if (!validDelta(delta_ekf, max_ekf_delta_xy_, max_ekf_delta_yaw_, "EKF", "Reject")) {
      logFactor("ekf", false, "delta gate rejected", stamp, current_key_index_, current_key_index_ + 1, delta_ekf);
      last_ekf_pose_ = ekf_pose;
      resetOrbAnchor("EKF backbone jump/re-anchor");
      resetScanAnchor("EKF backbone jump/re-anchor");
      RCLCPP_WARN(
        get_logger(),
        "EKF is backbone: skipped this bad EKF factor and re-anchored last EKF pose to avoid chaining over the jump");
      return;
    }

    gtsam::Pose2 delta_for_factor = delta_ekf;
    if (
      use_turn_rate_yaw_when_ekf_stalled_ &&
      latest_turn_rate_stamp_ >= 0.0 &&
      std::isfinite(ekf_dt) && ekf_dt >= 1e-3 && ekf_dt <= max_node_dt_ * 2.0 &&
      std::isfinite(stamp - latest_turn_rate_stamp_) &&
      std::fabs(stamp - latest_turn_rate_stamp_) <= turn_rate_timeout_sec_ &&
      std::fabs(latest_turn_rate_wz_) >= turn_rate_yaw_min_wz_ &&
      std::fabs(normalizeAngle(delta_ekf.theta())) <= turn_rate_yaw_max_pose_delta_)
    {
      const double turn_delta_yaw = normalizeAngle(latest_turn_rate_wz_ * ekf_dt);
      if (std::fabs(turn_delta_yaw) > std::fabs(delta_ekf.theta())) {
        delta_for_factor = gtsam::Pose2(delta_ekf.x(), delta_ekf.y(), turn_delta_yaw);
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "EKF yaw stalled while wheel turn-rate is active; using turn-rate yaw delta %.3f rad "
          "(wz=%.3f, dt=%.3f) for graph backbone",
          turn_delta_yaw, latest_turn_rate_wz_, ekf_dt);
      }
    }
    if (!validDelta(delta_for_factor, max_ekf_delta_xy_, max_ekf_delta_yaw_, "EKF", "Reject")) {
      logFactor(
        "ekf", false, "turn-rate yaw delta gate rejected", stamp,
        current_key_index_, current_key_index_ + 1, delta_for_factor);
      last_ekf_pose_ = ekf_pose;
      resetOrbAnchor("EKF turn-rate yaw jump/re-anchor");
      resetScanAnchor("EKF turn-rate yaw jump/re-anchor");
      return;
    }

    const double node_dt = stamp - nodes_.back().stamp;
    const bool moved_enough =
      deltaNormXY(delta_for_factor) >= min_node_translation_ ||
      std::fabs(normalizeAngle(delta_for_factor.theta())) >= min_node_rotation_;
    const bool time_gap_forces_node =
      create_node_on_time_gap_ && node_dt >= max_node_dt_;

    if (!moved_enough && !time_gap_forces_node)
    {
      // Keep Nav2/visualization TF fresh, but do not add zero-motion factors.
      // Adding time-forced nodes while stationary lets scan matching noise
      // accumulate into a fake trajectory and makes "home" drift.
      if (!nodes_.empty()) {
        nodes_.back().stamp = stamp;
      }
      publishOutputs(msg->header.stamp, ekf_pose);
      return;
    }

    const size_t prev_index = current_key_index_;
    ++current_key_index_;

    const auto build_start = std::chrono::steady_clock::now();
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initial;
    graph.add(gtsam::BetweenFactor<gtsam::Pose2>(
      keyFor(prev_index), keyFor(current_key_index_), delta_for_factor, ekf_noise_));

    const gtsam::Pose2 predicted = latest_optimized_pose_.compose(delta_for_factor);
    initial.insert(keyFor(current_key_index_), predicted);

    const double build_sec = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - build_start).count();
    if (!updateIsam(graph, initial, "EKF relative factor", "ekf", build_sec)) {
      logFactor("ekf", false, "iSAM2 rejected", stamp, prev_index, current_key_index_, delta_for_factor);
      --current_key_index_;
      return;
    }

    last_ekf_pose_ = ekf_pose;
    latest_optimized_pose_ = result_.at<gtsam::Pose2>(keyFor(current_key_index_));
    nodes_.push_back(GraphNode{current_key_index_, stamp, ekf_pose});
    trimNodes();
    ++ekf_factor_count_;
    logFactor("ekf", true, "accepted", stamp, prev_index, current_key_index_, delta_for_factor);

    RCLCPP_INFO(
      get_logger(), "Created EKF node X(%zu), added EKF BetweenFactor dx=%.3f dy=%.3f dyaw=%.3f",
      current_key_index_, delta_for_factor.x(), delta_for_factor.y(), delta_for_factor.theta());

    publishOutputs(msg->header.stamp, ekf_pose);
  }

  void initializeGraph(
    const gtsam::Pose2 & ekf_pose, double stamp_sec,
    const builtin_interfaces::msg::Time & stamp)
  {
    std::string seed_source{"first EKF pose"};
    gtsam::Pose2 prior_pose = ekf_pose;
    if (wait_for_global_pose_before_init_ && seed_from_global_pose_ && has_global_seed_) {
      prior_pose = latest_global_seed_pose_;
      seed_source = "first scan-to-map pose";
    } else if (use_initial_pose_) {
      prior_pose = gtsam::Pose2(initial_x_, initial_y_, initial_yaw_);
      seed_source = "initial_pose parameters";
    } else if (seed_from_global_pose_ && has_global_seed_) {
      prior_pose = latest_global_seed_pose_;
      seed_source = "first scan-to-map pose";
    }
    initializeGraphWithPrior(ekf_pose, prior_pose, prior_noise_, stamp_sec, stamp, seed_source, false);
  }

  gtsam::Pose2 latestPreinitEkfPoseOrInitial() const
  {
    if (has_latest_preinit_ekf_pose_) {
      return latest_preinit_ekf_pose_;
    }
    if (has_initial_ekf_pose_) {
      return initial_ekf_pose_;
    }
    return gtsam::Pose2(0.0, 0.0, initial_yaw_);
  }

  void initializeGraphWithPrior(
    const gtsam::Pose2 & ekf_pose,
    const gtsam::Pose2 & prior_pose,
    const gtsam::noiseModel::Diagonal::shared_ptr & prior_noise,
    double stamp_sec,
    const builtin_interfaces::msg::Time & stamp,
    const std::string & seed_source,
    bool from_scan_map)
  {
    current_key_index_ = 0;
    last_ekf_pose_ = ekf_pose;
    latest_ekf_stamp_ = has_latest_preinit_ekf_pose_ ? latest_preinit_ekf_stamp_ : stamp_sec;
    latest_optimized_pose_ = prior_pose;
    const auto build_start = std::chrono::steady_clock::now();
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initial;
    graph.add(gtsam::PriorFactor<gtsam::Pose2>(keyFor(0), latest_optimized_pose_, prior_noise));
    initial.insert(keyFor(0), latest_optimized_pose_);

    const double build_sec = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - build_start).count();
    if (!updateIsam(graph, initial, "initial prior", "prior", build_sec)) {
      return;
    }

    initialized_ = true;
    init_was_from_scan_map_ = from_scan_map;
    graph_init_stamp_sec_ = stamp_sec;
    nodes_.push_back(GraphNode{0, stamp_sec, ekf_pose});
    resetRelativeAnchorsAtGraphInit(stamp_sec);
    logPose("graph", "initialized", stamp_sec, latest_optimized_pose_, current_key_index_);
    logFactor("prior", true, seed_source, stamp_sec, std::nullopt, current_key_index_, latest_optimized_pose_);
    RCLCPP_INFO(
      get_logger(),
      "Created initial prior X(0) map pose x=%.3f y=%.3f yaw=%.3f from %s; first EKF odom x=%.3f y=%.3f yaw=%.3f",
      latest_optimized_pose_.x(), latest_optimized_pose_.y(), latest_optimized_pose_.theta(),
      seed_source.c_str(),
      ekf_pose.x(), ekf_pose.y(), ekf_pose.theta());
    publishOutputs(stamp, ekf_pose);
  }

  void resetRelativeAnchorsAtGraphInit(double stamp_sec)
  {
    // Global scan-to-map init defines X0. Relative sources must restart from
    // that same instant, otherwise their pre-init drift can be applied after X0.
    resetOrbAnchor("graph initialized; ORB relative chain starts after X0");

    const double max_anchor_age = std::max(max_association_dt_, max_scan_dt_);
    if (
      has_latest_preinit_scan_pose_ &&
      latest_preinit_scan_stamp_ >= 0.0 &&
      std::fabs(stamp_sec - latest_preinit_scan_stamp_) <= max_anchor_age)
    {
      setScanAnchor(
        latest_preinit_scan_pose_, latest_preinit_scan_stamp_, current_key_index_,
        "graph initialized; scan-to-scan anchor reset at X0");
      return;
    }

    has_last_scan_ = false;
    RCLCPP_INFO(
      get_logger(),
      "Graph initialized; scan-to-scan anchor cleared until first RF2O odom after X0");
  }

  ScanMapInitMetrics updateScanMapInitBuffer(const gtsam::Pose2 & pose, double stamp)
  {
    if (scan_map_init_wait_start_stamp_ < 0.0) {
      scan_map_init_wait_start_stamp_ = stamp;
    }
    scan_map_init_buffer_.push_back(ScanMapInitSample{stamp, pose});
    const size_t keep = static_cast<size_t>(std::max(1, scan_map_init_stable_count_));
    while (scan_map_init_buffer_.size() > keep) {
      scan_map_init_buffer_.pop_front();
    }
    return computeScanMapInitMetrics();
  }

  ScanMapInitMetrics computeScanMapInitMetrics() const
  {
    ScanMapInitMetrics metrics;
    if (scan_map_init_buffer_.empty()) {
      return metrics;
    }
    metrics.duration = scan_map_init_buffer_.back().stamp - scan_map_init_buffer_.front().stamp;
    metrics.xy_spread = 0.0;
    metrics.yaw_spread = 0.0;
    for (size_t i = 0; i < scan_map_init_buffer_.size(); ++i) {
      for (size_t j = i + 1; j < scan_map_init_buffer_.size(); ++j) {
        const double dx = scan_map_init_buffer_[i].pose.x() - scan_map_init_buffer_[j].pose.x();
        const double dy = scan_map_init_buffer_[i].pose.y() - scan_map_init_buffer_[j].pose.y();
        metrics.xy_spread = std::max(metrics.xy_spread, std::hypot(dx, dy));
        metrics.yaw_spread = std::max(
          metrics.yaw_spread,
          std::fabs(normalizeAngle(
            scan_map_init_buffer_[i].pose.theta() - scan_map_init_buffer_[j].pose.theta())));
      }
    }
    const bool enough_samples =
      scan_map_init_buffer_.size() >= static_cast<size_t>(std::max(1, scan_map_init_stable_count_));
    const bool yaw_ok =
      scan_map_init_ignore_yaw_stability_ ||
      metrics.yaw_spread <= scan_map_init_max_yaw_spread_;
    metrics.stable = enough_samples &&
      metrics.xy_spread <= scan_map_init_max_xy_spread_ &&
      metrics.duration >= scan_map_init_min_duration_sec_ &&
      yaw_ok;
    return metrics;
  }

  gtsam::Pose2 averageScanMapInitPose() const
  {
    double x = 0.0;
    double y = 0.0;
    double sin_yaw = 0.0;
    double cos_yaw = 0.0;
    for (const auto & sample : scan_map_init_buffer_) {
      x += sample.pose.x();
      y += sample.pose.y();
      sin_yaw += std::sin(sample.pose.theta());
      cos_yaw += std::cos(sample.pose.theta());
    }
    const double n = static_cast<double>(std::max<size_t>(1, scan_map_init_buffer_.size()));
    return gtsam::Pose2(x / n, y / n, std::atan2(sin_yaw, cos_yaw));
  }

  ScanMapUpdateMetrics updateScanMapUpdateBuffer(
    const gtsam::Pose2 & innovation,
    double stamp)
  {
    scan_map_update_buffer_.push_back(ScanMapUpdateSample{stamp, innovation});
    const size_t keep = static_cast<size_t>(std::max(1, scan_map_candidate_required_count_));
    while (scan_map_update_buffer_.size() > keep) {
      scan_map_update_buffer_.pop_front();
    }
    while (
      scan_map_update_buffer_.size() > 1 &&
      scan_map_candidate_window_sec_ > 0.0 &&
      stamp - scan_map_update_buffer_.front().stamp > scan_map_candidate_window_sec_)
    {
      scan_map_update_buffer_.pop_front();
    }
    return computeScanMapUpdateMetrics();
  }

  ScanMapUpdateMetrics computeScanMapUpdateMetrics()
  {
    ScanMapUpdateMetrics metrics;
    if (scan_map_update_buffer_.empty()) {
      return metrics;
    }
    metrics.count = scan_map_update_buffer_.size();
    metrics.duration =
      scan_map_update_buffer_.back().stamp - scan_map_update_buffer_.front().stamp;
    metrics.xy_spread = 0.0;
    metrics.yaw_spread = 0.0;
    for (size_t i = 0; i < scan_map_update_buffer_.size(); ++i) {
      for (size_t j = i + 1; j < scan_map_update_buffer_.size(); ++j) {
        const double dx =
          scan_map_update_buffer_[i].innovation.x() -
          scan_map_update_buffer_[j].innovation.x();
        const double dy =
          scan_map_update_buffer_[i].innovation.y() -
          scan_map_update_buffer_[j].innovation.y();
        metrics.xy_spread = std::max(metrics.xy_spread, std::hypot(dx, dy));
        metrics.yaw_spread = std::max(
          metrics.yaw_spread,
          std::fabs(normalizeAngle(
            scan_map_update_buffer_[i].innovation.theta() -
            scan_map_update_buffer_[j].innovation.theta())));
      }
    }
    const bool enough_samples =
      scan_map_update_buffer_.size() >=
      static_cast<size_t>(std::max(1, scan_map_update_stable_count_));
    const bool enough_candidates =
      scan_map_update_buffer_.size() >=
      static_cast<size_t>(std::max(1, scan_map_candidate_required_count_));
    metrics.stable =
      enough_samples &&
      enough_candidates &&
      metrics.duration >= scan_map_update_min_duration_sec_ &&
      metrics.xy_spread <= scan_map_candidate_max_spread_xy_ &&
      metrics.yaw_spread <= scan_map_candidate_max_spread_yaw_;
    last_scan_map_candidate_spread_xy_ = metrics.xy_spread;
    last_scan_map_candidate_spread_yaw_ = metrics.yaw_spread;
    return metrics;
  }

  bool initializeGraphFromStableScanMap(
    const gtsam::Pose2 & averaged_scan_map_pose,
    const ScanMapInitMetrics & metrics,
    double stamp_sec,
    const builtin_interfaces::msg::Time & stamp)
  {
    const gtsam::Pose2 ekf_pose = latestPreinitEkfPoseOrInitial();
    const double prior_yaw = init_prior_use_scan_map_yaw_ ?
      averaged_scan_map_pose.theta() :
      ekf_pose.theta();
    const gtsam::Pose2 prior_pose(
      averaged_scan_map_pose.x(), averaged_scan_map_pose.y(), normalizeAngle(prior_yaw));
    const auto init_noise = noise(init_prior_sigma_x_, init_prior_sigma_y_, init_prior_sigma_yaw_);
    initializeGraphWithPrior(
      ekf_pose, prior_pose, init_noise, stamp_sec, stamp, "stable_scan_map_init", true);
    scan_map_init_buffer_.clear();
    RCLCPP_INFO(
      get_logger(),
      "Initialized graph prior from stable scan-to-map: x=%.3f y=%.3f yaw=%.3f used_scan_map_yaw=%s xy_spread=%.3f yaw_spread=%.3f duration=%.2fs",
      prior_pose.x(), prior_pose.y(), prior_pose.theta(),
      init_prior_use_scan_map_yaw_ ? "true" : "false",
      metrics.xy_spread, metrics.yaw_spread, metrics.duration);
    return initialized_;
  }

  bool scanMapInitTimedOut(double stamp) const
  {
    return scan_map_init_wait_start_stamp_ >= 0.0 &&
      scan_map_init_timeout_sec_ > 0.0 &&
      stamp - scan_map_init_wait_start_stamp_ >= scan_map_init_timeout_sec_;
  }

  bool worldToMapPixel(double x, double y, int & u, int & v) const
  {
    if (!occupancy_map_.valid || occupancy_map_.resolution <= 0.0) {
      return false;
    }
    const double mx = (x - occupancy_map_.origin_x) / occupancy_map_.resolution;
    const double my = (y - occupancy_map_.origin_y) / occupancy_map_.resolution;
    u = static_cast<int>(std::floor(mx));
    v = static_cast<int>(std::floor(my));
    return u >= 0 && u < occupancy_map_.width && v >= 0 && v < occupancy_map_.height;
  }

  int occupancyValueAt(int u, int v) const
  {
    if (!occupancy_map_.valid || u < 0 || u >= occupancy_map_.width ||
      v < 0 || v >= occupancy_map_.height)
    {
      return -1;
    }
    const int image_row = occupancy_map_.height - 1 - v;
    const size_t index =
      static_cast<size_t>(image_row) * static_cast<size_t>(occupancy_map_.width) +
      static_cast<size_t>(u);
    if (index >= occupancy_map_.pixels.size()) {
      return -1;
    }
    return static_cast<int>(occupancy_map_.pixels[index]);
  }

  static std::string occupancyClass(int value)
  {
    if (value < 0) {
      return "outside_map";
    }
    if (value >= 250) {
      return "free";
    }
    if (value <= 50) {
      return "occupied";
    }
    return "unknown";
  }

  OccupancyGateResult scanMapOccupancyGate(const gtsam::Pose2 & pose) const
  {
    if (!enforce_scan_map_bounds_ || !map_bounds_.valid) {
      return OccupancyGateResult{};
    }
    const bool in_margin_bounds =
      pose.x() >= map_bounds_.min_x - scan_map_bounds_margin_ &&
      pose.x() <= map_bounds_.max_x + scan_map_bounds_margin_ &&
      pose.y() >= map_bounds_.min_y - scan_map_bounds_margin_ &&
      pose.y() <= map_bounds_.max_y + scan_map_bounds_margin_;
    if (!in_margin_bounds) {
      OccupancyGateResult result;
      result.accepted = false;
      result.reason = "outside_map";
      return result;
    }
    if (!occupancy_map_.valid) {
      return OccupancyGateResult{};
    }

    OccupancyGateResult result;
    if (!worldToMapPixel(pose.x(), pose.y(), result.u, result.v)) {
      result.accepted = false;
      result.reason = "outside_map";
      return result;
    }
    result.value = occupancyValueAt(result.u, result.v);
    const std::string center_class = occupancyClass(result.value);
    if (center_class != "free") {
      result.accepted = false;
      result.reason = center_class;
      return result;
    }

    const int radius_cells = static_cast<int>(
      std::ceil(std::max(0.0, scan_map_occupancy_gate_radius_) / occupancy_map_.resolution));
    for (int dv = -radius_cells; dv <= radius_cells; ++dv) {
      for (int du = -radius_cells; du <= radius_cells; ++du) {
        if (du * du + dv * dv > radius_cells * radius_cells) {
          continue;
        }
        const int uu = result.u + du;
        const int vv = result.v + dv;
        const int value = occupancyValueAt(uu, vv);
        const std::string cls = occupancyClass(value);
        if (cls == "outside_map") {
          result.accepted = false;
          result.reason = "too_far_from_free";
          result.value = value;
          return result;
        }
        if (cls == "occupied" || cls == "unknown") {
          result.accepted = false;
          result.reason = "too_far_from_free";
          result.value = value;
          return result;
        }
      }
    }
    return result;
  }

  void warnScanMapOccupancyRejected(
    const gtsam::Pose2 & pose,
    const OccupancyGateResult & occupancy,
    const std::string & reason)
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Reject scan-to-map pose by occupancy gate: reason=%s pose=(%.3f, %.3f, %.3f), "
      "pixel=(%d,%d), value=%d, status=%s, bounds x=[%.3f, %.3f] y=[%.3f, %.3f].",
      reason.c_str(), pose.x(), pose.y(), pose.theta(),
      occupancy.u, occupancy.v, occupancy.value, occupancy.reason.c_str(),
      map_bounds_.min_x, map_bounds_.max_x,
      map_bounds_.min_y, map_bounds_.max_y);
  }

  void maybeFallbackScanMapInitTimeout(
    const gtsam::Pose2 & ekf_pose,
    double stamp_sec,
    const builtin_interfaces::msg::Time & stamp)
  {
    if (!init_from_scan_map_ || initialized_) {
      return;
    }
    if (scan_map_init_wait_start_stamp_ < 0.0) {
      scan_map_init_wait_start_stamp_ = stamp_sec;
    }
    if (!scanMapInitTimedOut(stamp_sec)) {
      return;
    }
    if (!fallback_to_initial_pose_on_scan_map_timeout_) {
      if (!scan_map_init_timeout_logged_) {
        scan_map_init_timeout_logged_ = true;
        RCLCPP_WARN(
          get_logger(),
          "Scan-to-map init timeout. No graph initialized because fallback_to_initial_pose_on_scan_map_timeout=false.");
      }
      return;
    }
    RCLCPP_WARN(
      get_logger(),
      "Scan-to-map init timeout. Falling back to initial_pose.");
    initializeGraphWithPrior(
      ekf_pose, gtsam::Pose2(initial_x_, initial_y_, initial_yaw_),
      prior_noise_, stamp_sec, stamp, "initial_pose fallback after scan-map timeout", false);
  }

  void orbCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
  {
    RCLCPP_DEBUG(get_logger(), "Received ORB raw pose stamp %.3f", stampToSec(msg->header.stamp));
    if (!initialized_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Reject ORB factor: graph not initialized by EKF yet");
      return;
    }
    if (!orbTrackingOk()) {
      rejectOrbFactor("tracking state is not OK/OK_KLT");
      resetOrbAnchor("tracking lost or not initialized");
      return;
    }
    const double stamp = stampToSec(msg->header.stamp);
    if (reject_orb_when_turning_ && stamp < orb_reject_until_stamp_) {
      rejectOrbFactor("robot is turning/recently turned; do not bridge ORB across 180-degree corridor ambiguity");
      resetOrbAnchor("turning guard active");
      return;
    }
    if (!finitePoseMsg(msg->pose)) {
      rejectOrbFactor("pose has NaN/Inf");
      resetOrbAnchor("invalid ORB pose");
      return;
    }

    const auto node = nearestNode(stamp);
    if (!node) {
      rejectOrbFactor("no EKF node near ORB stamp");
      resetOrbAnchor("no node association");
      return;
    }

    gtsam::Pose2 orb_base = pose2FromPose(msg->pose);
    if (!orb_pose_is_base_) {
      if (orb_pose_is_tcw_) {
        orb_base = orb_base.inverse();
      }
      const auto camera_to_base = lookupCameraToBase(msg->header.frame_id);
      if (!camera_to_base) {
        rejectOrbFactor("missing TF camera->base; not using camera pose as base pose");
        resetOrbAnchor("missing camera-base TF");
        return;
      }
      orb_base = orb_base.compose(*camera_to_base);
    }
    logPose("orb", "received", stamp, orb_base, node->index);

    if (!has_last_orb_) {
      setOrbAnchor(orb_base, stamp, node->index, "first valid ORB after reset/relocalize");
      return;
    }

    if (stamp <= last_orb_stamp_ || (stamp - last_orb_stamp_) > max_orb_dt_) {
      rejectOrbFactor("timestamp invalid or ORB gap too large");
      setOrbAnchor(orb_base, stamp, node->index, "ORB time gap; start new relative chain");
      return;
    }
    if (node->index <= last_orb_node_index_) {
      rejectOrbFactor("associated EKF node did not advance");
      setOrbAnchor(orb_base, stamp, node->index, "node association did not advance");
      return;
    }

    gtsam::Pose2 delta_orb = last_orb_pose_.between(orb_base);
    if (!use_orb_translation_) {
      // ORB is yaw-only here; large X/Y sigmas prevent this zero translation
      // measurement from falsely constraining robot motion.
      delta_orb = gtsam::Pose2(0.0, 0.0, delta_orb.theta());
    }
    if (!use_orb_yaw_) {
      delta_orb = gtsam::Pose2(delta_orb.x(), delta_orb.y(), 0.0);
    }

    if (!validDelta(delta_orb, max_orb_delta_xy_, max_orb_delta_yaw_, "ORB", "Reject")) {
      logFactor("orb", false, "delta gate rejected", stamp, last_orb_node_index_, node->index, delta_orb);
      rejectOrbFactor("ORB jump gate rejected delta");
      setOrbAnchor(orb_base, stamp, node->index, "ORB jump; start new relative chain");
      return;
    }

    const size_t from_index = last_orb_node_index_;
    const size_t to_index = node->index;
    const double orb_gate_xy = use_orb_translation_ ?
      max_orb_ekf_disagreement_xy_ : std::numeric_limits<double>::infinity();
    if (!consistentWithEkf(
        from_index, to_index, delta_orb,
        orb_gate_xy, max_orb_ekf_disagreement_yaw_, "ORB"))
    {
      logFactor("orb", false, "ORB/EKF innovation rejected", stamp, from_index, to_index, delta_orb);
      rejectOrbFactor("ORB/EKF innovation gate rejected delta");
      setOrbAnchor(orb_base, stamp, node->index, "ORB/EKF disagreement");
      return;
    }
    const auto build_start = std::chrono::steady_clock::now();
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initial;
    graph.add(gtsam::BetweenFactor<gtsam::Pose2>(
      keyFor(from_index), keyFor(to_index), delta_orb, orb_noise_));

    const double build_sec = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - build_start).count();
    if (!updateIsam(graph, initial, "ORB raw relative factor", "orb", build_sec)) {
      logFactor("orb", false, "iSAM2 rejected", stamp, from_index, to_index, delta_orb);
      rejectOrbFactor("iSAM2 rejected ORB factor");
      setOrbAnchor(orb_base, stamp, node->index, "iSAM2 rejected previous ORB factor");
      return;
    }

    ++orb_factor_count_;
    last_orb_pose_ = orb_base;
    last_orb_stamp_ = stamp;
    last_orb_node_index_ = to_index;
    latest_optimized_pose_ = result_.at<gtsam::Pose2>(keyFor(current_key_index_));
    logFactor("orb", true, "accepted", stamp, from_index, to_index, delta_orb);
    RCLCPP_INFO(
      get_logger(), "Added ORB raw BetweenFactor X(%zu)->X(%zu) dx=%.3f dy=%.3f dyaw=%.3f",
      from_index, to_index, delta_orb.x(), delta_orb.y(), delta_orb.theta());
    publishOutputs(msg->header.stamp, nodes_.back().ekf_pose);
  }

  void scanOdomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    if (!finitePoseMsg(msg->pose.pose)) {
      rejectScanFactor("pose has NaN/Inf");
      resetScanAnchor("invalid scan odom pose");
      return;
    }

    const double stamp = stampToSec(msg->header.stamp);
    const gtsam::Pose2 scan_pose = pose2FromPose(msg->pose.pose);
    if (!finitePose(scan_pose)) {
      rejectScanFactor("Pose2 has NaN/Inf");
      resetScanAnchor("invalid scan odom Pose2");
      return;
    }
    if (!initialized_) {
      latest_preinit_scan_pose_ = scan_pose;
      latest_preinit_scan_stamp_ = stamp;
      has_latest_preinit_scan_pose_ = true;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Stored scan-to-scan anchor candidate while graph waits for scan-to-map init");
      return;
    }

    const auto node = nearestNode(stamp);
    if (!node) {
      rejectScanFactor("no EKF node near scan odom stamp");
      resetScanAnchor("no node association");
      return;
    }

    logPose("scan_to_scan", "received", stamp, scan_pose, node->index);
    if (!has_last_scan_) {
      setScanAnchor(scan_pose, stamp, node->index, "first valid scan odom after reset");
      return;
    }

    if (stamp <= last_scan_stamp_ || (stamp - last_scan_stamp_) > max_scan_dt_) {
      rejectScanFactor("timestamp invalid or scan odom gap too large");
      setScanAnchor(scan_pose, stamp, node->index, "scan odom time gap; start new relative chain");
      return;
    }
    if (node->index <= last_scan_node_index_) {
      rejectScanFactor("associated EKF node did not advance");
      setScanAnchor(scan_pose, stamp, node->index, "node association did not advance");
      return;
    }

    const gtsam::Pose2 delta_scan = last_scan_pose_.between(scan_pose);
    if (!validDelta(delta_scan, max_scan_delta_xy_, max_scan_delta_yaw_, "scan odom", "Reject")) {
      logFactor("scan_odom", false, "delta gate rejected", stamp, last_scan_node_index_, node->index, delta_scan);
      rejectScanFactor("scan odom jump gate rejected delta");
      setScanAnchor(scan_pose, stamp, node->index, "scan odom jump; start new relative chain");
      return;
    }

    const size_t from_index = last_scan_node_index_;
    const size_t to_index = node->index;
    if (reject_scan_odom_when_not_forward_ &&
      std::fabs(latest_ekf_vx_) <= scan_odom_min_forward_vx_)
    {
      logFactor("scan_odom", false, "not_forward gate rejected", stamp, from_index, to_index, delta_scan);
      rejectScanFactor("robot is stopped/not moving forward; scan-to-scan odom is not trusted");
      setScanAnchor(scan_pose, stamp, node->index, "not moving; refresh scan odom anchor");
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Reject scan odom factor while stopped/not forward: |vx|=%.3f <= %.3f",
        std::fabs(latest_ekf_vx_), scan_odom_min_forward_vx_);
      return;
    }
    if (latest_ekf_yaw_rate_ > scan_odom_max_yaw_rate_) {
      logFactor("scan_odom", false, "turning gate rejected", stamp, from_index, to_index, delta_scan);
      rejectScanFactor("robot is turning; scan-to-scan relative odom is not trusted");
      setScanAnchor(scan_pose, stamp, node->index, "turning; start new scan odom chain");
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Reject scan odom factor while turning: wz=%.3f rad/s > %.3f",
        latest_ekf_yaw_rate_, scan_odom_max_yaw_rate_);
      return;
    }
    const auto disagreement = ekfDisagreement(from_index, to_index, delta_scan);
    if (!disagreement) {
      logFactor("scan_odom", false, "missing EKF association", stamp, from_index, to_index, delta_scan);
      rejectScanFactor("scan/EKF innovation gate cannot find associated EKF nodes");
      setScanAnchor(scan_pose, stamp, node->index, "missing EKF association");
      return;
    }
    const double disagree_xy = deltaNormXY(*disagreement);
    const double disagree_yaw = std::fabs(normalizeAngle(disagreement->theta()));
    if (disagree_xy > max_scan_ekf_disagreement_xy_ ||
      disagree_yaw > max_scan_ekf_disagreement_yaw_)
    {
      logFactor("scan_odom", false, "scan/EKF innovation rejected", stamp, from_index, to_index, delta_scan);
      rejectScanFactor("scan/EKF innovation gate rejected delta");
      setScanAnchor(scan_pose, stamp, node->index, "scan/EKF disagreement");
      RCLCPP_WARN(
        get_logger(),
        "Reject scan odom factor X(%zu)->X(%zu): scan_delta=(%.3f, %.3f, %.3f), ekf_disagreement=(%.3f/%.3f m, %.3f/%.3f rad)",
        from_index, to_index, delta_scan.x(), delta_scan.y(), delta_scan.theta(),
        disagree_xy, max_scan_ekf_disagreement_xy_,
        disagree_yaw, max_scan_ekf_disagreement_yaw_);
      return;
    }
    const auto build_start = std::chrono::steady_clock::now();
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initial;
    graph.add(gtsam::BetweenFactor<gtsam::Pose2>(
      keyFor(from_index), keyFor(to_index), delta_scan, scan_odom_noise_));

    const double build_sec = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - build_start).count();
    if (!updateIsam(
        graph, initial, "scan odom relative factor", "scan_odom", build_sec))
    {
      logFactor("scan_odom", false, "iSAM2 rejected", stamp, from_index, to_index, delta_scan);
      rejectScanFactor("iSAM2 rejected scan odom factor");
      setScanAnchor(scan_pose, stamp, node->index, "iSAM2 rejected previous scan factor");
      return;
    }

    ++scan_odom_factor_count_;
    last_scan_pose_ = scan_pose;
    last_scan_stamp_ = stamp;
    last_scan_node_index_ = to_index;
    latest_optimized_pose_ = result_.at<gtsam::Pose2>(keyFor(current_key_index_));
    logFactor("scan_odom", true, "accepted", stamp, from_index, to_index, delta_scan);
    RCLCPP_INFO(
      get_logger(),
      "Added scan odom BetweenFactor X(%zu)->X(%zu) scan_delta=(%.3f, %.3f, %.3f), ekf_disagreement=(%.3f m, %.3f rad)",
      from_index, to_index, delta_scan.x(), delta_scan.y(), delta_scan.theta(),
      disagree_xy, disagree_yaw);
    publishOutputs(msg->header.stamp, nodes_.back().ekf_pose);
  }

  void amclCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg)
  {
    if (!finitePoseMsg(msg->pose.pose)) {
      RCLCPP_WARN(get_logger(), "Reject scan-to-map absolute factor: pose has NaN/Inf");
      return;
    }

    const double stamp = stampToSec(msg->header.stamp);
    const gtsam::Pose2 raw_scan_map_pose = pose2FromPose(msg->pose.pose);
    logPose("scan_to_map_raw", "received", stamp, raw_scan_map_pose);
    // Corridor symmetry can flip scan-to-map yaw by pi, so XY-only is the
    // robust default. Yaw may be enabled explicitly for a suitable map.
    bool trust_scan_map_yaw = trust_scan_map_yaw_default_;
    const bool reject_due_to_turning =
      reject_scan_map_when_turning_ && stamp < scan_map_reject_until_stamp_;
    const bool reject_due_to_not_forward =
      reject_scan_map_when_not_forward_ && latest_ekf_vx_ <= scan_map_min_forward_vx_;
    if (reject_due_to_turning) {
      trust_scan_map_yaw = false;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Use scan-to-map as XY-only: robot is turning/recently turned; yaw follows graph/EKF");
    }
    if (reject_due_to_not_forward) {
      trust_scan_map_yaw = false;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Use scan-to-map as XY-only: robot is not moving forward; vx=%.3f <= %.3f",
        latest_ekf_vx_, scan_map_min_forward_vx_);
    }
    const auto corrected_pose = correctedGlobalPose(*msg);
    if (!corrected_pose) {
      logScanMapDebug(
        stamp, std::nullopt, trust_scan_map_yaw, false, "no_base_tf",
        raw_scan_map_pose, std::nullopt, std::nullopt, std::nullopt, std::nullopt);
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Reject scan-to-map absolute factor: cannot convert %s pose to %s",
        global_pose_sensor_frame_.c_str(), base_frame_.c_str());
      return;
    }
    const gtsam::Pose2 scan_map_pose = *corrected_pose;
    logPose("scan_to_map_corrected", "corrected", stamp, scan_map_pose);
    const auto occupancy_gate = scanMapOccupancyGate(scan_map_pose);
    if (!occupancy_gate.accepted) {
      ++global_pose_rejected_count_;
      const std::string reject_reason = initialized_ ?
        "scan_match_invalid:" + occupancy_gate.reason :
        "dock_initialization_failed:" + occupancy_gate.reason;
      logScanMapDebug(
        stamp, std::nullopt, trust_scan_map_yaw, false,
        reject_reason, raw_scan_map_pose, scan_map_pose,
        std::nullopt, std::nullopt, std::nullopt, occupancy_gate);
      logFactor(
        "scan_map", false, reject_reason, stamp,
        std::nullopt, std::nullopt, scan_map_pose);
      warnScanMapOccupancyRejected(scan_map_pose, occupancy_gate, reject_reason);
      return;
    }
    if (!initialized_) {
      latest_global_seed_pose_ = scan_map_pose;
      has_global_seed_ = true;
      if (init_from_scan_map_) {
        const auto metrics = updateScanMapInitBuffer(scan_map_pose, stamp);
        if (!metrics.stable) {
          if (scanMapInitTimedOut(stamp)) {
            if (fallback_to_initial_pose_on_scan_map_timeout_) {
              const gtsam::Pose2 ekf_pose = latestPreinitEkfPoseOrInitial();
              RCLCPP_WARN(
                get_logger(),
                "Scan-to-map init timeout. Falling back to initial_pose.");
              initializeGraphWithPrior(
                ekf_pose, gtsam::Pose2(initial_x_, initial_y_, initial_yaw_),
                prior_noise_, stamp, msg->header.stamp,
                "initial_pose fallback after scan-map timeout", false);
            } else if (!scan_map_init_timeout_logged_) {
              scan_map_init_timeout_logged_ = true;
              RCLCPP_WARN(
                get_logger(),
                "Scan-to-map init timeout. No graph initialized because fallback_to_initial_pose_on_scan_map_timeout=false.");
            }
            return;
          }
          RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "Waiting for stable scan-to-map init: samples=%zu/%d, xy_spread=%.3f, yaw_spread=%.3f, duration=%.2fs",
            scan_map_init_buffer_.size(), scan_map_init_stable_count_,
            metrics.xy_spread, metrics.yaw_spread, metrics.duration);
          return;
        }
        initializeGraphFromStableScanMap(
          averageScanMapInitPose(), metrics, stamp, msg->header.stamp);
        return;
      } else {
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Stored scan-to-map seed pose x=%.3f y=%.3f yaw=%.3f; waiting for EKF to initialize graph",
          scan_map_pose.x(), scan_map_pose.y(), scan_map_pose.theta());
      }
      return;
    }
    const bool yaw_rejected_by_motion =
      reject_due_to_turning || reject_due_to_not_forward;
    if (yaw_rejected_by_motion) {
      const std::string reason = reject_due_to_turning ?
        "turning" : "not_forward";
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Rejected scan-to-map yaw only (%s); evaluating XY correction "
        "with vx=%.3f wz=%.3f",
        reason.c_str(), latest_ekf_vx_, latest_ekf_yaw_rate_);
    }
    if (init_was_from_scan_map_) {
      const double dt_after_init = stamp - graph_init_stamp_sec_;
      if (dt_after_init >= 0.0 && dt_after_init < ignore_scan_map_after_init_sec_) {
        logScanMapDebug(
          stamp, current_key_index_, trust_scan_map_yaw, false,
          "ignored_after_scan_map_init", raw_scan_map_pose, scan_map_pose,
          std::nullopt, std::nullopt, std::nullopt);
        logFactor(
          "scan_map", false, "ignored_after_scan_map_init", stamp,
          std::nullopt, current_key_index_, gtsam::Pose2());
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Ignoring scan-to-map factor shortly after scan-map initialization: dt_after_init=%.2fs",
          dt_after_init);
        return;
      }
    }
    if (stamp <= last_amcl_stamp_) {
      RCLCPP_WARN(get_logger(), "Reject scan-to-map absolute factor: non-increasing timestamp");
      return;
    }
    const auto node = nearestNode(stamp);
    if (!node) {
      RCLCPP_WARN(get_logger(), "Reject scan-to-map absolute factor: no EKF node near stamp %.3f", stamp);
      return;
    }

    const gtsam::Pose2 predicted = result_.at<gtsam::Pose2>(keyFor(node->index));
    if (has_last_amcl_ && node->index <= last_amcl_node_index_) {
      logScanMapDebug(
        stamp, node->index, trust_scan_map_yaw, false,
        "associated EKF node did not advance", raw_scan_map_pose, scan_map_pose,
        std::nullopt, predicted, std::nullopt);
      logFactor(
        "scan_map", false, "associated EKF node did not advance", stamp,
        last_amcl_node_index_, node->index, gtsam::Pose2());
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Skip duplicate scan-to-map prior on X(%zu); waiting for EKF node to advance",
        node->index);
      return;
    }
    const bool near_home =
      enable_home_scan_map_correction_ &&
      std::hypot(scan_map_pose.x() - home_map_x_, scan_map_pose.y() - home_map_y_) <=
      home_scan_map_radius_;
    if (scan_map_only_near_home_ && !near_home) {
      ++global_pose_rejected_count_;
      logScanMapDebug(
        stamp, node->index, trust_scan_map_yaw, false,
        "outside home gate rejected", raw_scan_map_pose, scan_map_pose,
        std::nullopt, predicted, std::nullopt);
      logFactor(
        "scan_map", false, "outside home gate rejected", stamp,
        std::nullopt, node->index, gtsam::Pose2());
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Reject scan-to-map factor outside home gate: scan_map=(%.3f, %.3f), home=(%.3f, %.3f), radius=%.3f",
        scan_map_pose.x(), scan_map_pose.y(), home_map_x_, home_map_y_, home_scan_map_radius_);
      return;
    }
    gtsam::Pose2 amcl_pose = trust_scan_map_yaw ?
      scan_map_pose :
      gtsam::Pose2(scan_map_pose.x(), scan_map_pose.y(), predicted.theta());
    const gtsam::Pose2 raw_innovation = predicted.between(scan_map_pose);
    const double raw_innovation_yaw = std::fabs(normalizeAngle(raw_innovation.theta()));
    if (
      scan_map_use_xy_only_when_yaw_bad_ &&
      trust_scan_map_yaw &&
      raw_innovation_yaw > scan_map_xy_only_yaw_reject_threshold_)
    {
      trust_scan_map_yaw = false;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Use scan-to-map as XY-only: raw yaw innovation %.3f > %.3f",
        raw_innovation_yaw, scan_map_xy_only_yaw_reject_threshold_);
      amcl_pose = gtsam::Pose2(scan_map_pose.x(), scan_map_pose.y(), predicted.theta());
    }
    logPose(
      "scan_to_map", trust_scan_map_yaw ? "used_full" : "used_xy_only",
      stamp, amcl_pose, node->index);
    const gtsam::Pose2 innovation = predicted.between(amcl_pose);
    const double scan_map_innovation_limit = near_home ?
      std::min(max_global_innovation_xy_, home_scan_map_max_innovation_xy_) :
      max_global_innovation_xy_;
    const double innovation_xy = deltaNormXY(innovation);
    const double innovation_yaw = trust_scan_map_yaw ?
      std::fabs(normalizeAngle(raw_innovation.theta())) :
      std::fabs(normalizeAngle(innovation.theta()));
    const bool backward_pull_while_forward =
      reject_scan_map_backward_when_forward_ &&
      latest_ekf_vx_ > scan_map_backward_reject_min_vx_ &&
      innovation.x() < -scan_map_backward_reject_distance_;
    if (backward_pull_while_forward) {
      ++global_pose_rejected_count_;
      scan_map_update_buffer_.clear();
      logScanMapDebug(
        stamp, node->index, trust_scan_map_yaw, false,
        "backward_pull_while_forward_rejected", raw_scan_map_pose, scan_map_pose,
        amcl_pose, predicted, innovation);
      logFactor(
        "scan_map", false, "backward_pull_while_forward_rejected", stamp,
        std::nullopt, node->index, innovation);
      RCLCPP_WARN(
        get_logger(),
        "Reject scan-to-map correction pulling backward while driving forward: "
        "innovation_x=%.3f < -%.3f, dxy=%.3f, vx=%.3f",
        innovation.x(), scan_map_backward_reject_distance_, innovation_xy, latest_ekf_vx_);
      return;
    }
    const bool severe_correction =
      innovation_xy > scan_map_reject_correction_xy_ ||
      (trust_scan_map_yaw && innovation_yaw > scan_map_reject_correction_yaw_);
    if (severe_correction) {
      ++global_pose_rejected_count_;
      ++scan_map_consecutive_reject_count_;
      setScanMapTrustState(ScanMapTrustState::SUSPECT, "severe scan-map correction");
      scan_map_update_buffer_.clear();
      logScanMapDebug(
        stamp, node->index, trust_scan_map_yaw, false,
        "suspect severe correction rejected", raw_scan_map_pose, scan_map_pose,
        amcl_pose, predicted, innovation);
      logFactor(
        "scan_map", false, "suspect severe correction rejected", stamp,
        std::nullopt, node->index, innovation);
      if (scan_map_consecutive_reject_count_ >=
        static_cast<size_t>(std::max(1, scan_map_suspect_reject_count_threshold_)))
      {
        triggerScanMapRelocalization(stamp, "repeated severe scan-map corrections");
      }
      return;
    }
    if (stamp < scan_map_suspect_until_stamp_) {
      ++global_pose_rejected_count_;
      logScanMapDebug(
        stamp, node->index, trust_scan_map_yaw, false,
        "relocalizing cooldown rejected", raw_scan_map_pose, scan_map_pose,
        amcl_pose, predicted, innovation);
      logFactor(
        "scan_map", false, "relocalizing cooldown rejected", stamp,
        std::nullopt, node->index, innovation);
      return;
    }
    if (innovation_xy > scan_map_innovation_limit ||
      (trust_scan_map_yaw && std::fabs(normalizeAngle(innovation.theta())) > max_global_innovation_yaw_))
    {
      ++global_pose_rejected_count_;
      ++scan_map_consecutive_reject_count_;
      setScanMapTrustState(ScanMapTrustState::SUSPECT, "innovation gate rejected");
      logScanMapDebug(
        stamp, node->index, trust_scan_map_yaw, false,
        "innovation gate rejected", raw_scan_map_pose, scan_map_pose,
        amcl_pose, predicted, innovation);
      logFactor("scan_map", false, "innovation gate rejected", stamp, std::nullopt, node->index, innovation);
      RCLCPP_WARN(
        get_logger(),
        "Reject global scan-map factor: innovation dxy=%.3f/%.3f dyaw=%.3f/%.3f",
        innovation_xy, scan_map_innovation_limit,
        std::fabs(normalizeAngle(innovation.theta())), max_global_innovation_yaw_);
      if (scan_map_consecutive_reject_count_ >=
        static_cast<size_t>(std::max(1, scan_map_suspect_reject_count_threshold_)))
      {
        triggerScanMapRelocalization(stamp, "repeated scan-map innovation rejects");
      }
      return;
    }
    if (has_last_amcl_) {
      const gtsam::Pose2 amcl_delta = last_amcl_pose_.between(scan_map_pose);
      const double yaw_jump_gate = trust_scan_map_yaw ? max_amcl_jump_yaw_ : M_PI;
      if (!validDelta(amcl_delta, max_amcl_jump_xy_, yaw_jump_gate, "scan-to-map", "Reject")) {
        logScanMapDebug(
          stamp, node->index, trust_scan_map_yaw, false,
          "jump gate rejected", raw_scan_map_pose, scan_map_pose,
          amcl_pose, predicted, innovation);
        logFactor("scan_map", false, "jump gate rejected", stamp, std::nullopt, node->index, amcl_delta);
        ++scan_map_consecutive_reject_count_;
        setScanMapTrustState(ScanMapTrustState::SUSPECT, "scan-map jump gate rejected");
        if (scan_map_consecutive_reject_count_ >=
          static_cast<size_t>(std::max(1, scan_map_suspect_reject_count_threshold_)))
        {
          triggerScanMapRelocalization(stamp, "repeated scan-map jump rejects");
        }
        return;
      }
    }
    const bool needs_candidate_confirmation =
      enable_scan_map_candidate_gate_ &&
      (innovation_xy > scan_map_max_single_correction_xy_ ||
      (trust_scan_map_yaw && innovation_yaw > scan_map_max_single_correction_yaw_) ||
      scan_map_selective_updates_);
    if (needs_candidate_confirmation) {
      setScanMapTrustState(ScanMapTrustState::CANDIDATE, "scan-map candidate validation");
      const auto metrics = updateScanMapUpdateBuffer(innovation, stamp);
      if (!metrics.stable) {
        logScanMapDebug(
          stamp, node->index, trust_scan_map_yaw, false,
          "candidate_waiting", raw_scan_map_pose, scan_map_pose,
          amcl_pose, predicted, innovation);
        logFactor(
          "scan_map", false, "candidate_waiting", stamp,
          std::nullopt, node->index, innovation);
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Wait for stable scan-to-map correction: samples=%zu/%d, "
          "innovation_spread_xy=%.3f/%.3f yaw=%.3f/%.3f duration=%.2f/%.2fs",
          scan_map_update_buffer_.size(), scan_map_candidate_required_count_,
          metrics.xy_spread, scan_map_candidate_max_spread_xy_,
          metrics.yaw_spread, scan_map_candidate_max_spread_yaw_,
          metrics.duration, scan_map_update_min_duration_sec_);
        return;
      }
      if (
        last_scan_map_update_stamp_ >= 0.0 &&
        stamp - last_scan_map_update_stamp_ < scan_map_update_min_interval_sec_)
      {
        logScanMapDebug(
          stamp, node->index, trust_scan_map_yaw, false,
          "selective_update_rate_limited", raw_scan_map_pose, scan_map_pose,
          amcl_pose, predicted, innovation);
        logFactor(
          "scan_map", false, "selective_update_rate_limited", stamp,
          std::nullopt, node->index, innovation);
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Skip scan-to-map correction: last update %.2fs ago < %.2fs",
          stamp - last_scan_map_update_stamp_, scan_map_update_min_interval_sec_);
        return;
      }
    }

    const auto build_start = std::chrono::steady_clock::now();
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initial;
    auto scan_map_factor_noise = scanMapNoiseFromMsg(*msg, trust_scan_map_yaw, near_home);
    std::string bounded_suffix;
    if (enable_scan_map_bounded_correction_ &&
      (innovation_xy > 0.5 * scan_map_max_single_correction_xy_ ||
      innovation_yaw > 0.5 * scan_map_max_single_correction_yaw_))
    {
      const double scale = 2.5;
      scan_map_factor_noise = robustNoise(
        std::max(scan_map_sigma_x_ * scale, scan_map_sigma_x_),
        std::max(scan_map_sigma_y_ * scale, scan_map_sigma_y_),
        trust_scan_map_yaw ?
        std::max(scan_map_sigma_yaw_ * scale, scan_map_sigma_yaw_) :
        scan_map_untrusted_yaw_sigma_,
        1.345);
      bounded_suffix = "_bounded";
    }
    graph.add(gtsam::PriorFactor<gtsam::Pose2>(
      keyFor(node->index), amcl_pose, scan_map_factor_noise));

    const double build_sec = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - build_start).count();
    if (!updateIsam(
        graph, initial, "scan-to-map absolute factor", "scan_map", build_sec))
    {
      logScanMapDebug(
        stamp, node->index, trust_scan_map_yaw, false,
        "iSAM2 rejected", raw_scan_map_pose, scan_map_pose,
        amcl_pose, predicted, innovation);
      logFactor("scan_map", false, "iSAM2 rejected", stamp, std::nullopt, node->index, innovation);
      return;
    }

    ++scan_map_factor_count_;
    scan_map_consecutive_reject_count_ = 0;
    setScanMapTrustState(ScanMapTrustState::TRUSTED, "accepted scan-map factor");
    has_last_amcl_ = true;
    last_amcl_pose_ = scan_map_pose;
    last_amcl_stamp_ = stamp;
    last_amcl_node_index_ = node->index;
    last_scan_map_update_stamp_ = stamp;
    scan_map_update_buffer_.clear();
    latest_optimized_pose_ = result_.at<gtsam::Pose2>(keyFor(current_key_index_));
    std::string scan_map_mode =
      std::string(trust_scan_map_yaw ? "accepted_full" : "accepted_xy_only");
    scan_map_mode += bounded_suffix;
    if (yaw_rejected_by_motion) {
      scan_map_mode += "_yaw_rejected_motion";
    }
    if (near_home) {
      scan_map_mode += "_home";
    }
    logScanMapDebug(
      stamp, node->index, trust_scan_map_yaw, true,
      scan_map_mode,
      raw_scan_map_pose, scan_map_pose, amcl_pose, predicted, innovation);
    logFactor("scan_map", true, scan_map_mode, stamp, std::nullopt, node->index, innovation);
    RCLCPP_INFO(
      get_logger(),
      "Added scan-to-map factor after init: mode=%s, node=%zu, "
      "innovation dx=%.3f dy=%.3f dyaw=%.3f",
      scan_map_mode.c_str(),
      node->index, innovation.x(), innovation.y(), innovation.theta());
    RCLCPP_INFO(
      get_logger(), "Added global scan-map PriorFactor on X(%zu) x=%.3f y=%.3f yaw=%.3f mode=%s",
      node->index, amcl_pose.x(), amcl_pose.y(), amcl_pose.theta(),
      trust_scan_map_yaw ? "full" : "xy-only");
    publishOutputs(msg->header.stamp, nodes_.back().ekf_pose);
  }

  std::optional<gtsam::Pose2> correctedGlobalPose(
    const geometry_msgs::msg::PoseWithCovarianceStamped & msg)
  {
    geometry_msgs::msg::PoseStamped pose_in;
    pose_in.header = msg.header;
    pose_in.pose = msg.pose.pose;

    geometry_msgs::msg::PoseStamped pose_map;
    const std::string source_frame = pose_in.header.frame_id.empty() ? map_frame_ : pose_in.header.frame_id;
    if (source_frame == map_frame_) {
      pose_map = pose_in;
      pose_map.header.frame_id = map_frame_;
    } else {
      try {
        const auto tf = tf_buffer_->lookupTransform(
          map_frame_, source_frame, tf2::TimePointZero);
        tf2::doTransform(pose_in, pose_map, tf);
        if (!logged_global_pose_frame_transform_) {
          logged_global_pose_frame_transform_ = true;
          RCLCPP_INFO(
            get_logger(),
            "scan-to-map pose frame transform enabled: %s -> %s",
            source_frame.c_str(), map_frame_.c_str());
        }
      } catch (const tf2::TransformException & e) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Cannot transform scan-to-map pose %s -> %s: %s",
          source_frame.c_str(), map_frame_.c_str(), e.what());
        return std::nullopt;
      }
    }

    const gtsam::Pose2 raw_pose = pose2FromPose(pose_map.pose);
    if (!global_pose_is_sensor_frame_) {
      return applyGlobalPoseYawOffset(raw_pose);
    }

    try {
      const auto tf = tf_buffer_->lookupTransform(
        global_pose_sensor_frame_, base_frame_, tf2::TimePointZero);
      const gtsam::Pose2 sensor_to_base = pose2FromTransform(tf.transform);
      const gtsam::Pose2 corrected = applyGlobalPoseYawOffset(raw_pose.compose(sensor_to_base));
      if (!logged_global_pose_correction_) {
        logged_global_pose_correction_ = true;
        RCLCPP_INFO(
          get_logger(),
          "scan-to-map correction %s->%s: dx=%.3f dy=%.3f dyaw=%.3f yaw_offset=%.3f; raw=(%.3f, %.3f, %.3f) corrected=(%.3f, %.3f, %.3f)",
          global_pose_sensor_frame_.c_str(), base_frame_.c_str(),
          sensor_to_base.x(), sensor_to_base.y(), sensor_to_base.theta(), global_pose_yaw_offset_,
          raw_pose.x(), raw_pose.y(), raw_pose.theta(),
          corrected.x(), corrected.y(), corrected.theta());
      }
      return corrected;
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Cannot lookup scan-to-map correction %s->%s: %s",
        global_pose_sensor_frame_.c_str(), base_frame_.c_str(), e.what());
      return std::nullopt;
    }
  }

  gtsam::Pose2 applyGlobalPoseYawOffset(const gtsam::Pose2 & pose) const
  {
    double offset = global_pose_yaw_offset_;
    if (global_pose_auto_yaw_offset_) {
      if (!has_auto_global_pose_yaw_offset_) {
        auto_global_pose_yaw_offset_ = normalizeAngle(initial_yaw_ - pose.theta());
        has_auto_global_pose_yaw_offset_ = true;
        RCLCPP_INFO(
          get_logger(),
          "Auto scan-to-map yaw offset calibrated: initial_yaw=%.3f raw_yaw=%.3f offset=%.3f",
          initial_yaw_, pose.theta(), auto_global_pose_yaw_offset_);
      }
      offset = auto_global_pose_yaw_offset_;
    }

    if (std::fabs(offset) < 1e-9) {
      return pose;
    }
    return gtsam::Pose2(
      pose.x(), pose.y(), normalizeAngle(pose.theta() + offset));
  }

  gtsam::noiseModel::Base::shared_ptr scanMapNoiseFromMsg(
    const geometry_msgs::msg::PoseWithCovarianceStamped & msg,
    bool trust_yaw,
    bool near_home) const
  {
    // Home is a known map location. Use a stronger robust XY correction there,
    // but never inherit the corridor-prone scan-to-map yaw when yaw is gated.
    if (near_home && !trust_yaw) {
      return robustNoise(
        home_scan_map_sigma_xy_, home_scan_map_sigma_xy_,
        scan_map_untrusted_yaw_sigma_, 1.345);
    }
    if (!trust_yaw) {
      return robustNoise(scan_map_sigma_x_, scan_map_sigma_y_, scan_map_untrusted_yaw_sigma_, 1.345);
    }
    if (!use_amcl_covariance_) {
      return scan_map_noise_;
    }

    const double vx = msg.pose.covariance[0];
    const double vy = msg.pose.covariance[7];
    const double vyaw = msg.pose.covariance[35];
    if (!std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(vyaw) || vx <= 0.0 || vy <= 0.0 || vyaw <= 0.0) {
      return scan_map_noise_;
    }

    const double sx = std::clamp(std::sqrt(vx), min_covariance_sigma_, max_covariance_sigma_);
    const double sy = std::clamp(std::sqrt(vy), min_covariance_sigma_, max_covariance_sigma_);
    const double syaw = std::clamp(std::sqrt(vyaw), min_covariance_sigma_, max_covariance_sigma_);
    return gtsam::noiseModel::Robust::Create(
      gtsam::noiseModel::mEstimator::Huber::Create(1.345),
      noise(sx, sy, syaw));
  }

  bool updateIsam(
    const gtsam::NonlinearFactorGraph & graph,
    const gtsam::Values & initial,
    const std::string & context,
    const std::string & factor_type,
    double build_sec)
  {
    try {
      const auto optimize_start = std::chrono::steady_clock::now();
      isam_->update(graph, initial);
      result_ = isam_->calculateEstimate();
      optimize_sec_ += std::chrono::duration<double>(
        std::chrono::steady_clock::now() - optimize_start).count();
      build_graph_sec_ += build_sec;
      for (size_t i = 0; i < graph.size(); ++i) {
        const auto factor = graph.at(i);
        if (!factor) {
          continue;
        }
        full_graph_.push_back(factor);
        factor_types_[full_graph_.size() - 1] = factor_type;
      }
      initial_values_.insert(initial);
      RCLCPP_INFO(get_logger(), "iSAM2 update succeeded: %s", context.c_str());
      return true;
    } catch (const std::exception & e) {
      RCLCPP_WARN(get_logger(), "iSAM2 update failed for %s: %s", context.c_str(), e.what());
      return false;
    }
  }

  void orbTrackingStateCallback(const std_msgs::msg::Int32::ConstSharedPtr msg)
  {
    latest_orb_tracking_state_ = msg->data;
    has_orb_tracking_state_ = true;
    if (!orbTrackingOk()) {
      resetOrbAnchor("ORB tracking status topic reports non-OK state");
    }
  }

  bool orbTrackingOk() const
  {
    if (!has_orb_tracking_state_) {
      return !require_orb_tracking_state_;
    }
    return latest_orb_tracking_state_ == 2 || latest_orb_tracking_state_ == 5;
  }

  std::optional<gtsam::Pose2> lookupCameraToBase(const std::string & msg_frame)
  {
    const std::string camera_frame = orb_camera_frame_.empty() ? msg_frame : orb_camera_frame_;
    if (camera_frame.empty()) {
      return std::nullopt;
    }
    try {
      const auto tf = tf_buffer_->lookupTransform(
        camera_frame, base_frame_, tf2::TimePointZero);
      return pose2FromTransform(tf.transform);
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Missing ORB camera-base TF %s -> %s: %s",
        camera_frame.c_str(), base_frame_.c_str(), e.what());
      return std::nullopt;
    }
  }

  void setOrbAnchor(
    const gtsam::Pose2 & pose, double stamp, size_t node_index, const std::string & reason)
  {
    last_orb_pose_ = pose;
    last_orb_stamp_ = stamp;
    last_orb_node_index_ = node_index;
    has_last_orb_ = true;
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Set ORB anchor at X(%zu): %s; no ORB factor added",
      node_index, reason.c_str());
  }

  void resetOrbAnchor(const std::string & reason)
  {
    has_last_orb_ = false;
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Reset ORB anchor: %s; no ORB factor will bridge this gap", reason.c_str());
  }

  void rejectOrbFactor(const std::string & reason)
  {
    ++orb_rejected_count_;
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Reject ORB factor: %s", reason.c_str());
  }

  void setScanAnchor(
    const gtsam::Pose2 & pose, double stamp, size_t node_index, const std::string & reason)
  {
    last_scan_pose_ = pose;
    last_scan_stamp_ = stamp;
    last_scan_node_index_ = node_index;
    has_last_scan_ = true;
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Set scan odom anchor at X(%zu): %s; no scan factor added",
      node_index, reason.c_str());
  }

  void resetScanAnchor(const std::string & reason)
  {
    has_last_scan_ = false;
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Reset scan odom anchor: %s; no scan factor will bridge this gap", reason.c_str());
  }

  void rejectScanFactor(const std::string & reason)
  {
    ++scan_odom_rejected_count_;
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Reject scan odom factor: %s", reason.c_str());
  }

  void trimNodes()
  {
    const size_t max_nodes = static_cast<size_t>(std::max(100, max_path_length_));
    while (nodes_.size() > max_nodes) {
      nodes_.pop_front();
    }
  }

  void publishOutputs(
    const builtin_interfaces::msg::Time & stamp,
    const gtsam::Pose2 & current_ekf_pose)
  {
    if (!finitePose(latest_optimized_pose_)) {
      RCLCPP_WARN(get_logger(), "Skip graph output: optimized pose has NaN/Inf");
      return;
    }

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = stamp;
    pose_msg.header.frame_id = map_frame_;
    pose_msg.pose = poseFromPose2(latest_optimized_pose_);
    graph_pose_pub_->publish(pose_msg);
    logPose("graph", "published", stampToSec(stamp), latest_optimized_pose_, current_key_index_);

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header = pose_msg.header;
    odom_msg.child_frame_id = base_frame_;
    odom_msg.pose.pose = pose_msg.pose;
    graph_odom_pub_->publish(odom_msg);

    path_msg_.header.stamp = stamp;
    path_msg_.poses.push_back(pose_msg);
    if (max_path_length_ > 0 && path_msg_.poses.size() > static_cast<size_t>(max_path_length_)) {
      path_msg_.poses.erase(
        path_msg_.poses.begin(),
        path_msg_.poses.begin() + static_cast<std::ptrdiff_t>(path_msg_.poses.size() - max_path_length_));
    }
    graph_path_pub_->publish(path_msg_);

    if (publish_tf_) {
      publishMapToOdomTf(stamp, current_ekf_pose);
    }
  }

  void publishMapToOdomTf(
    const builtin_interfaces::msg::Time & stamp,
    const gtsam::Pose2 & current_ekf_pose)
  {
    (void)stamp;
    const gtsam::Pose2 T_map_base = latest_optimized_pose_;
    const gtsam::Pose2 T_odom_base = current_ekf_pose;
    const gtsam::Pose2 T_map_odom = T_map_base.compose(T_odom_base.inverse());

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp =
      get_clock()->now() + rclcpp::Duration::from_seconds(tf_future_tolerance_sec_);
    tf.header.frame_id = map_frame_;
    tf.child_frame_id = odom_frame_;
    tf.transform.translation.x = T_map_odom.x();
    tf.transform.translation.y = T_map_odom.y();
    tf.transform.translation.z = 0.0;
    tf.transform.rotation = quatFromYaw(T_map_odom.theta());
    tf_broadcaster_->sendTransform(tf);
  }

  void publishStats()
  {
    std_msgs::msg::String msg;
    std::ostringstream out;
    out << std::boolalpha
        << "{\"initialized\":" << initialized_
        << ",\"init_from_scan_map\":" << init_from_scan_map_
        << ",\"init_was_from_scan_map\":" << init_was_from_scan_map_
        << ",\"scan_map_init_samples\":" << scan_map_init_buffer_.size()
        << ",\"current_node\":" << current_key_index_
        << ",\"ekf_factors\":" << ekf_factor_count_
        << ",\"orb_accepted\":" << orb_factor_count_
        << ",\"orb_rejected\":" << orb_rejected_count_
        << ",\"scan_odom_factors\":" << scan_odom_factor_count_
        << ",\"scan_odom_rejected\":" << scan_odom_rejected_count_
        << ",\"scan_map_factors\":" << scan_map_factor_count_
        << ",\"scan_map_rejected\":" << global_pose_rejected_count_
        << ",\"path_size\":" << path_msg_.poses.size()
        << ",\"orb_tracking_state\":" << latest_orb_tracking_state_
        << ",\"pose\":{\"x\":" << latest_optimized_pose_.x()
        << ",\"y\":" << latest_optimized_pose_.y()
        << ",\"theta\":" << latest_optimized_pose_.theta()
        << "},\"config\":{"
        << "\"relinearize_threshold\":" << get_parameter("relinearize_threshold").as_double()
        << ",\"relinearize_skip\":" << get_parameter("relinearize_skip").as_int()
        << ",\"max_association_dt\":" << max_association_dt_
        << ",\"scan_map_init_max_xy_spread\":" << scan_map_init_max_xy_spread_
        << ",\"scan_map_init_min_duration_sec\":" << scan_map_init_min_duration_sec_
        << ",\"ignore_scan_map_after_init_sec\":" << ignore_scan_map_after_init_sec_
        << ",\"ekf_sigma_x\":" << ekf_sigma_x_
        << ",\"ekf_sigma_y\":" << ekf_sigma_y_
        << ",\"ekf_sigma_yaw\":" << ekf_sigma_yaw_
        << ",\"orb_sigma_x\":" << orb_sigma_x_
        << ",\"orb_sigma_y\":" << orb_sigma_y_
        << ",\"orb_sigma_yaw\":" << orb_sigma_yaw_
        << ",\"scan_odom_sigma_x\":" << scan_odom_sigma_x_
        << ",\"scan_odom_sigma_y\":" << scan_odom_sigma_y_
        << ",\"scan_odom_sigma_yaw\":" << scan_odom_sigma_yaw_
        << ",\"scan_odom_max_yaw_rate\":" << scan_odom_max_yaw_rate_
        << ",\"scan_map_sigma_x\":" << scan_map_sigma_x_
        << ",\"scan_map_sigma_y\":" << scan_map_sigma_y_
        << ",\"scan_map_sigma_yaw\":" << scan_map_sigma_yaw_
        << ",\"scan_map_only_near_home\":" << scan_map_only_near_home_
        << ",\"scan_map_occupancy_gate_radius\":" << scan_map_occupancy_gate_radius_
        << ",\"occupancy_map_loaded\":" << occupancy_map_.valid
        << ",\"max_orb_delta_xy\":" << max_orb_delta_xy_
        << ",\"max_orb_delta_yaw\":" << max_orb_delta_yaw_
        << ",\"orb_pose_is_base\":" << orb_pose_is_base_
        << ",\"use_orb_translation\":" << use_orb_translation_
        << ",\"use_orb_yaw\":" << use_orb_yaw_
        << "}}";
    msg.data = out.str();
    graph_stats_pub_->publish(msg);
    flushRunLogs();

    if (!initialized_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Graph waiting for first EKF odom to create X(0)");
      return;
    }
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Graph stats: X(%zu), EKF=%zu, ORB=%zu/%zu, scan_odom=%zu, scan_map=%zu/%zu, latest=(%.3f, %.3f, %.3f)",
      current_key_index_, ekf_factor_count_, orb_factor_count_, orb_rejected_count_,
      scan_odom_factor_count_, scan_map_factor_count_, global_pose_rejected_count_,
      latest_optimized_pose_.x(),
      latest_optimized_pose_.y(), latest_optimized_pose_.theta());
  }

  void nav2StatusCallback(const action_msgs::msg::GoalStatusArray::ConstSharedPtr msg)
  {
    if (!enable_run_logging_ || !nav2_log_.is_open()) {
      return;
    }
    std::ostringstream statuses;
    int last_status = -1;
    std::string last_name = "NONE";
    for (size_t i = 0; i < msg->status_list.size(); ++i) {
      const int status = msg->status_list[i].status;
      if (i > 0) {
        statuses << "|";
      }
      statuses << statusName(status);
      last_status = status;
      last_name = statusName(status);
    }
    nav2_log_ << std::fixed << std::setprecision(6)
              << nowSec() << "," << nowSec() << ","
              << msg->status_list.size() << ","
              << csvSafe(statuses.str()) << ","
              << last_status << "," << last_name << "\n";
  }

  void nav2FeedbackCallback(
    const nav2_msgs::action::NavigateToPose::Impl::FeedbackMessage::ConstSharedPtr msg)
  {
    if (!enable_run_logging_ || !nav2_feedback_log_.is_open() ||
      !finitePoseMsg(msg->feedback.current_pose.pose))
    {
      return;
    }
    const gtsam::Pose2 pose = pose2FromPose(msg->feedback.current_pose.pose);
    nav2_feedback_log_ << std::fixed << std::setprecision(6)
                       << nowSec() << ","
                       << stampToSec(msg->feedback.current_pose.header.stamp) << ","
                       << msg->feedback.current_pose.header.frame_id << ","
                       << pose.x() << "," << pose.y() << "," << pose.theta() << ","
                       << msg->feedback.distance_remaining << ","
                       << durationToSec(msg->feedback.navigation_time) << ","
                       << durationToSec(msg->feedback.estimated_time_remaining) << ","
                       << msg->feedback.number_of_recoveries << "\n";
    logPose(
      "nav2_feedback", "current_pose",
      stampToSec(msg->feedback.current_pose.header.stamp), pose);
  }

  void goalPoseCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
  {
    if (!enable_run_logging_ || !goal_log_.is_open() || !finitePoseMsg(msg->pose)) {
      return;
    }
    const gtsam::Pose2 goal = pose2FromPose(msg->pose);
    goal_log_ << std::fixed << std::setprecision(6)
              << nowSec() << "," << stampToSec(msg->header.stamp) << ","
              << msg->header.frame_id << ","
              << goal.x() << "," << goal.y() << "," << goal.theta() << "\n";
    logPose("nav2_goal", "published", stampToSec(msg->header.stamp), goal);
  }

  void flushRunLogs()
  {
    if (!enable_run_logging_) {
      return;
    }
    if (pose_log_.is_open()) {
      pose_log_.flush();
    }
    if (factor_log_.is_open()) {
      factor_log_.flush();
    }
    if (nav2_log_.is_open()) {
      nav2_log_.flush();
    }
    if (nav2_feedback_log_.is_open()) {
      nav2_feedback_log_.flush();
    }
    if (goal_log_.is_open()) {
      goal_log_.flush();
    }
    if (scan_map_debug_log_.is_open()) {
      scan_map_debug_log_.flush();
    }
  }

  std::string ekf_topic_;
  std::string amcl_topic_;
  std::string global_pose_topic_;
  std::string orb_topic_;
  std::string scan_odom_topic_;
  std::string turn_rate_topic_;
  std::string graph_odom_topic_;
  std::string graph_pose_topic_;
  std::string graph_path_topic_;
  std::string graph_stats_topic_;
  std::string nav2_status_topic_;
  std::string nav2_feedback_topic_;
  std::string goal_pose_topic_;
  std::string scan_map_relocalize_trigger_topic_;
  std::string map_yaml_file_;
  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string global_pose_sensor_frame_;
  std::string orb_camera_frame_;
  std::string orb_tracking_state_topic_;

  bool publish_tf_{false};
  bool use_initial_pose_{true};
  double initial_x_{0.0};
  double initial_y_{0.0};
  double initial_yaw_{0.0};
  bool wait_for_global_pose_before_init_{false};
  bool seed_from_global_pose_{true};
  bool enable_diagnostics_{true};
  bool diagnostics_saved_{false};
  bool enable_run_logging_{true};
  bool use_global_pose_factor_{true};
  bool init_from_scan_map_{true};
  bool init_prior_use_scan_map_yaw_{false};
  bool scan_map_init_ignore_yaw_stability_{true};
  bool fallback_to_initial_pose_on_scan_map_timeout_{false};
  bool init_was_from_scan_map_{false};
  bool scan_map_init_timeout_logged_{false};
  bool global_pose_is_sensor_frame_{false};
  bool global_pose_auto_yaw_offset_{false};
  mutable bool has_auto_global_pose_yaw_offset_{false};
  bool logged_global_pose_correction_{false};
  bool logged_global_pose_frame_transform_{false};
  bool orb_pose_is_base_{true};
  bool reject_scan_map_when_turning_{true};
  bool reject_scan_map_when_not_forward_{true};
  bool scan_map_selective_updates_{true};
  bool enable_scan_map_candidate_gate_{true};
  bool scan_map_use_xy_only_when_yaw_bad_{true};
  bool enable_scan_map_bounded_correction_{true};
  bool trust_scan_map_yaw_default_{false};
  bool reject_scan_map_backward_when_forward_{true};
  bool enable_scan_map_auto_relocalization_{false};
  bool enable_home_scan_map_correction_{true};
  bool scan_map_only_near_home_{false};
  bool initialized_{false};
  bool use_orb_translation_{false};
  bool use_orb_yaw_{true};
  bool estimate_orb_scale_{false};
  bool orb_pose_is_tcw_{false};
  bool require_orb_tracking_state_{false};
  bool reject_orb_when_turning_{true};
  bool has_orb_tracking_state_{false};
  bool use_amcl_covariance_{true};
  bool enforce_scan_map_bounds_{true};
  bool reject_scan_odom_when_not_forward_{true};
  int max_path_length_{2000};
  int latest_orb_tracking_state_{-999};
  int scan_map_init_stable_count_{5};
  int scan_map_update_stable_count_{5};
  int scan_map_candidate_required_count_{5};
  int scan_map_suspect_reject_count_threshold_{10};

  double max_association_dt_{0.25};
  double max_ekf_delta_xy_{2.0};
  double max_ekf_delta_yaw_{1.2};
  double min_node_translation_{0.03};
  double min_node_rotation_{0.02};
  double max_node_dt_{0.20};
  bool create_node_on_time_gap_{false};
  bool use_turn_rate_yaw_when_ekf_stalled_{true};
  double turn_rate_yaw_min_wz_{0.25};
  double turn_rate_yaw_max_pose_delta_{0.01};
  double tf_future_tolerance_sec_{0.15};
  double prior_sigma_x_{0.05};
  double prior_sigma_y_{0.05};
  double prior_sigma_yaw_{0.02};
  double scan_map_init_max_xy_spread_{0.12};
  double scan_map_init_max_yaw_spread_{0.15};
  double scan_map_init_min_duration_sec_{1.0};
  double scan_map_init_timeout_sec_{15.0};
  double init_prior_sigma_x_{0.10};
  double init_prior_sigma_y_{0.10};
  double init_prior_sigma_yaw_{0.50};
  double ignore_scan_map_after_init_sec_{0.5};
  double ekf_sigma_x_{0.05};
  double ekf_sigma_y_{0.05};
  double ekf_sigma_yaw_{0.12};
  double orb_sigma_x_{10.0};
  double orb_sigma_y_{10.0};
  double orb_sigma_yaw_{0.20};
  double max_orb_delta_xy_{1.0};
  double max_orb_delta_yaw_{0.5};
  double max_orb_dt_{1.0};
  double max_orb_ekf_disagreement_xy_{0.50};
  double max_orb_ekf_disagreement_yaw_{0.35};
  double orb_turn_reject_wz_{0.25};
  double orb_turn_block_sec_{1.5};
  double scan_odom_sigma_x_{0.70};
  double scan_odom_sigma_y_{0.70};
  double scan_odom_sigma_yaw_{0.40};
  double max_scan_delta_xy_{1.5};
  double max_scan_delta_yaw_{0.8};
  double max_scan_dt_{1.0};
  double max_scan_ekf_disagreement_xy_{0.15};
  double max_scan_ekf_disagreement_yaw_{0.10};
  double scan_odom_max_yaw_rate_{1.80};
  double scan_odom_min_forward_vx_{0.03};
  double scan_map_sigma_x_{0.35};
  double scan_map_sigma_y_{0.35};
  double scan_map_sigma_yaw_{10.0};
  double scan_map_untrusted_yaw_sigma_{10.0};
  double home_map_x_{0.0};
  double home_map_y_{0.0};
  double home_scan_map_radius_{0.75};
  double home_scan_map_sigma_xy_{0.10};
  double home_scan_map_max_innovation_xy_{0.60};
  double global_pose_yaw_offset_{0.0};
  mutable double auto_global_pose_yaw_offset_{0.0};
  double scan_map_turn_reject_wz_{0.25};
  double scan_map_turn_block_sec_{1.5};
  double scan_map_min_forward_vx_{0.03};
  double scan_map_backward_reject_min_vx_{0.03};
  double scan_map_backward_reject_distance_{0.10};
  double scan_map_update_min_interval_sec_{2.0};
  double scan_map_update_max_innovation_spread_xy_{0.12};
  double scan_map_update_max_innovation_spread_yaw_{0.35};
  double scan_map_update_min_duration_sec_{0.35};
  double scan_map_candidate_window_sec_{1.0};
  double scan_map_candidate_max_spread_xy_{0.15};
  double scan_map_candidate_max_spread_yaw_{0.15};
  double scan_map_max_single_correction_xy_{0.30};
  double scan_map_max_single_correction_yaw_{0.25};
  double scan_map_reject_correction_xy_{0.80};
  double scan_map_reject_correction_yaw_{0.80};
  double scan_map_xy_only_yaw_reject_threshold_{0.35};
  double scan_map_suspect_cooldown_sec_{3.0};
  double scan_map_relocalize_cooldown_sec_{5.0};
  double turn_rate_timeout_sec_{0.50};
  double max_amcl_jump_xy_{2.0};
  double max_amcl_jump_yaw_{1.0};
  double max_global_innovation_xy_{0.40};
  double max_global_innovation_yaw_{3.20};
  double scan_map_bounds_margin_{0.30};
  double scan_map_occupancy_gate_radius_{0.20};
  double min_covariance_sigma_{0.02};
  double max_covariance_sigma_{2.0};
  double build_graph_sec_{0.0};
  double optimize_sec_{0.0};
  double graph_init_stamp_sec_{-1.0};
  double run_pose_log_min_interval_sec_{0.20};
  double run_rejected_factor_log_min_interval_sec_{1.0};
  double run_scan_map_debug_min_interval_sec_{0.50};
  double last_scan_map_debug_log_time_{-1e9};
  MapBounds map_bounds_;
  OccupancyMap occupancy_map_;
  double scan_map_init_wait_start_stamp_{-1.0};

  size_t current_key_index_{0};
  size_t ekf_factor_count_{0};
  size_t orb_factor_count_{0};
  size_t orb_rejected_count_{0};
  size_t scan_odom_factor_count_{0};
  size_t scan_odom_rejected_count_{0};
  size_t scan_map_factor_count_{0};
  size_t global_pose_rejected_count_{0};
  size_t scan_map_consecutive_reject_count_{0};

  gtsam::Pose2 last_ekf_pose_;
  gtsam::Pose2 latest_optimized_pose_;
  gtsam::Pose2 initial_ekf_pose_;
  bool has_initial_ekf_pose_{false};
  gtsam::Pose2 latest_preinit_ekf_pose_;
  bool has_latest_preinit_ekf_pose_{false};
  double latest_preinit_ekf_stamp_{-1.0};
  double latest_ekf_stamp_{-1.0};
  double latest_ekf_yaw_rate_{0.0};
  double latest_ekf_vx_{0.0};
  double latest_turn_rate_wz_{0.0};
  double latest_turn_rate_stamp_{-1.0};
  double orb_reject_until_stamp_{-1.0};
  double scan_map_reject_until_stamp_{-1.0};
  double last_scan_map_update_stamp_{-1.0};
  double scan_map_suspect_until_stamp_{-1.0};
  double last_scan_map_relocalize_trigger_stamp_{-1e9};
  double last_scan_map_candidate_spread_xy_{std::numeric_limits<double>::quiet_NaN()};
  double last_scan_map_candidate_spread_yaw_{std::numeric_limits<double>::quiet_NaN()};
  bool last_scan_map_triggered_relocalization_{false};

  bool has_last_orb_{false};
  gtsam::Pose2 last_orb_pose_;
  double last_orb_stamp_{-1.0};
  size_t last_orb_node_index_{0};

  bool has_last_scan_{false};
  gtsam::Pose2 last_scan_pose_;
  double last_scan_stamp_{-1.0};
  size_t last_scan_node_index_{0};
  gtsam::Pose2 latest_preinit_scan_pose_;
  bool has_latest_preinit_scan_pose_{false};
  double latest_preinit_scan_stamp_{-1.0};

  bool has_last_amcl_{false};
  gtsam::Pose2 last_amcl_pose_;
  double last_amcl_stamp_{-1.0};
  size_t last_amcl_node_index_{0};
  bool has_global_seed_{false};
  gtsam::Pose2 latest_global_seed_pose_;
  ScanMapTrustState scan_map_trust_state_{ScanMapTrustState::TRUSTED};

  std::deque<GraphNode> nodes_;
  std::deque<ScanMapInitSample> scan_map_init_buffer_;
  std::deque<ScanMapUpdateSample> scan_map_update_buffer_;
  gtsam::noiseModel::Diagonal::shared_ptr prior_noise_;
  gtsam::noiseModel::Diagonal::shared_ptr ekf_noise_;
  gtsam::noiseModel::Base::shared_ptr orb_noise_;
  gtsam::noiseModel::Base::shared_ptr scan_odom_noise_;
  gtsam::noiseModel::Base::shared_ptr scan_map_noise_;
  std::unique_ptr<gtsam::ISAM2> isam_;
  gtsam::Values result_;
  gtsam::Values initial_values_;
  gtsam::NonlinearFactorGraph full_graph_;
  std::map<size_t, std::string> factor_types_;
  std::string diagnostics_output_dir_;
  std::string run_log_root_;
  std::string run_log_tag_;
  std::string run_log_dir_;
  std::ofstream pose_log_;
  std::ofstream factor_log_;
  std::ofstream nav2_log_;
  std::ofstream nav2_feedback_log_;
  std::ofstream goal_log_;
  std::ofstream scan_map_debug_log_;
  std::map<std::string, double> last_pose_log_stamp_by_source_;
  std::map<std::string, double> last_rejected_factor_log_time_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ekf_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr orb_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr orb_tracking_state_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr scan_odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr turn_rate_sub_;
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr nav2_status_sub_;
  rclcpp::Subscription<nav2_msgs::action::NavigateToPose::Impl::FeedbackMessage>::SharedPtr
    nav2_feedback_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr graph_odom_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr graph_pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr graph_path_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr graph_stats_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr scan_map_relocalize_trigger_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr stats_timer_;
  nav_msgs::msg::Path path_msg_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GtsamPose2FusionNode>());
  rclcpp::shutdown();
  return 0;
}
