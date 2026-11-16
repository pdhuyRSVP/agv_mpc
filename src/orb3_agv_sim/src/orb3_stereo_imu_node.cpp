#include <memory>
#include <limits>
#include <string>
#include <functional>
#include <cmath>
#include <deque>
#include <mutex>
#include <vector>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/callback_group.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <GL/glew.h>
#include <System.h>
#include <ImuTypes.h>
#include <sophus/se3.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

class Orb3StereoImuNode : public rclcpp::Node
{
public:
  Orb3StereoImuNode()
  : Node("orb3_stereo_imu_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    auto get_or_declare_string =
      [this](const std::string & name, const std::string & default_value) -> std::string
      {
        if (!this->has_parameter(name)) {
          this->declare_parameter<std::string>(name, default_value);
        }
        std::string value = default_value;
        this->get_parameter(name, value);
        return value;
      };

    auto get_or_declare_bool =
      [this](const std::string & name, bool default_value) -> bool
      {
        if (!this->has_parameter(name)) {
          this->declare_parameter<bool>(name, default_value);
        }
        bool value = default_value;
        this->get_parameter(name, value);
        return value;
      };

    auto get_or_declare_int =
      [this](const std::string & name, int default_value) -> int
      {
        if (!this->has_parameter(name)) {
          this->declare_parameter<int>(name, default_value);
        }
        int value = default_value;
        this->get_parameter(name, value);
        return value;
      };

    auto get_or_declare_double =
      [this](const std::string & name, double default_value) -> double
      {
        if (!this->has_parameter(name)) {
          this->declare_parameter<double>(name, default_value);
        }
        double value = default_value;
        this->get_parameter(name, value);
        return value;
      };

    vocab_file_ = get_or_declare_string(
      "vocabulary_file", "/home/trongtuan/Dev/ORB_SLAM3/Vocabulary/ORBvoc.txt");
    settings_template_file_ = get_or_declare_string(
      "settings_template_file",
      "/home/trongtuan/agv_imuorb_ekf/src/orb3_agv_sim/config/orb3_stereo_imu_gz.template.yaml");
    runtime_settings_file_ = get_or_declare_string(
      "runtime_settings_file", "/tmp/orb3_stereo_imu_runtime.yaml");
    atlas_load_file_ = get_or_declare_string("atlas_load_file", "");
    atlas_save_file_ = get_or_declare_string("atlas_save_file", "");
    localization_mode_ = get_or_declare_bool("localization_mode", false);

    left_topic_ = get_or_declare_string("left_topic", "/stereo/left/image_raw");
    right_topic_ = get_or_declare_string("right_topic", "/stereo/right/image_raw");
    imu_topic_ = get_or_declare_string("imu_topic", "/imu/data_raw");

    world_frame_ = get_or_declare_string("world_frame", "orb_map");
    base_frame_ = get_or_declare_string("base_frame", "Base_footprint");
    camera_frame_ = get_or_declare_string("camera_frame", "stereo_left_optical");
    imu_frame_param_ = get_or_declare_string("imu_frame", "");

    // Tbc mode controls what transform is written into ORB-SLAM3 YAML.
    // Use manual_ros_optical as the safe default for a planar AGV with ROS optical camera frames.
    // Alternatives for debugging: tf_body_camera, tf_camera_body, manual_inverse_ros_optical.
    tbc_mode_ = get_or_declare_string("tbc_mode", "manual_ros_optical");
    imu_time_offset_s_ = get_or_declare_double("imu_time_offset_s", 0.0);

    use_sim_time_ = get_or_declare_bool("use_sim_time", true);
    use_viewer_ = get_or_declare_bool("use_viewer", false);
    publish_tf_ = get_or_declare_bool("publish_tf", false);
    force_planar_output_ = get_or_declare_bool("force_planar_output", true);

    path_topic_ = get_or_declare_string("path_topic", "/orbslam3/path");
    pose_topic_ = get_or_declare_string("pose_topic", "/orbslam3/pose");
    camera_pose_topic_ = get_or_declare_string("camera_pose_topic", "/orbslam3/camera_pose");

    sync_queue_size_ = get_or_declare_int("sync_queue_size", 30);
    max_stereo_dt_ = get_or_declare_double("max_stereo_dt", 0.04);
    min_imu_boot_samples_ = get_or_declare_int("min_imu_boot_samples", 7);
    min_imu_between_frames_ = get_or_declare_int("min_imu_between_frames", 3);

    // Wrapper-level safety gate: protect ROS consumers from impossible visual pose jumps.
    // This package does not patch or modify the external ORB-SLAM3 core.
    enable_wrapper_motion_gate_ = get_or_declare_bool("enable_wrapper_motion_gate", true);
    max_pose_jump_xy_m_ = get_or_declare_double("max_pose_jump_xy_m", 0.25);
    max_pose_jump_z_m_ = get_or_declare_double("max_pose_jump_z_m", 0.12);
    max_pose_jump_yaw_deg_ = get_or_declare_double("max_pose_jump_yaw_deg", 12.0);
    max_roll_pitch_deg_ = get_or_declare_double("max_roll_pitch_deg", 7.0);
    max_orb_linear_speed_mps_ = get_or_declare_double("max_orb_linear_speed_mps", 1.20);
    max_orb_yaw_rate_dps_ = get_or_declare_double("max_orb_yaw_rate_dps", 90.0);
    reset_on_reject_count_ = get_or_declare_int("reset_on_reject_count", 12);
    // In Gazebo/planar AGV, IMU acceleration convention can differ by plugin/config.
    // Keep the checks configurable so diagnostics do not starve ORB-SLAM3 of IMU data.
    enable_accel_sanity_check_ = get_or_declare_bool("enable_accel_sanity_check", false);
    drop_zero_accel_samples_ = get_or_declare_bool("drop_zero_accel_samples", false);
    min_acc_norm_ = get_or_declare_double("min_acc_norm", 0.0);
    max_acc_norm_ = get_or_declare_double("max_acc_norm", 30.0);

    RCLCPP_INFO(get_logger(), "Starting ORB-SLAM3 Stereo + IMU node");
    RCLCPP_INFO(get_logger(), "vocabulary_file: %s", vocab_file_.c_str());
    RCLCPP_INFO(get_logger(), "settings_template_file: %s", settings_template_file_.c_str());
    RCLCPP_INFO(get_logger(), "runtime_settings_file: %s", runtime_settings_file_.c_str());
    RCLCPP_INFO(get_logger(), "atlas_load_file: %s", atlas_load_file_.c_str());
    RCLCPP_INFO(get_logger(), "atlas_save_file: %s", atlas_save_file_.c_str());
    RCLCPP_INFO(get_logger(), "localization_mode: %s", localization_mode_ ? "true" : "false");
    RCLCPP_INFO(get_logger(), "left_topic: %s", left_topic_.c_str());
    RCLCPP_INFO(get_logger(), "right_topic: %s", right_topic_.c_str());
    RCLCPP_INFO(get_logger(), "imu_topic: %s", imu_topic_.c_str());
    RCLCPP_INFO(get_logger(), "base_frame: %s", base_frame_.c_str());
    RCLCPP_INFO(get_logger(), "camera_frame: %s", camera_frame_.c_str());
    RCLCPP_INFO(get_logger(), "imu_frame(param): %s", imu_frame_param_.c_str());
    RCLCPP_INFO(get_logger(), "tbc_mode: %s", tbc_mode_.c_str());
    RCLCPP_INFO(get_logger(), "imu_time_offset_s: %.6f", imu_time_offset_s_);
    RCLCPP_INFO(get_logger(), "use_viewer: %s", use_viewer_ ? "true" : "false");
    RCLCPP_INFO(get_logger(), "publish_tf: %s", publish_tf_ ? "true" : "false");
    RCLCPP_INFO(get_logger(), "force_planar_output: %s", force_planar_output_ ? "true" : "false");
    RCLCPP_INFO(get_logger(), "max_stereo_dt: %.3f", max_stereo_dt_);
    RCLCPP_INFO(get_logger(), "min_imu_boot_samples: %d", min_imu_boot_samples_);
    RCLCPP_INFO(get_logger(), "min_imu_between_frames: %d", min_imu_between_frames_);
    RCLCPP_INFO(get_logger(), "enable_wrapper_motion_gate: %s", enable_wrapper_motion_gate_ ? "true" : "false");
    RCLCPP_INFO(get_logger(),
      "wrapper gate: xy_abs=%.3f m, z_abs=%.3f m, yaw_abs=%.1f deg, max_rp=%.1f deg, v=%.2f m/s, yaw_rate=%.1f deg/s",
      max_pose_jump_xy_m_, max_pose_jump_z_m_, max_pose_jump_yaw_deg_,
      max_roll_pitch_deg_, max_orb_linear_speed_mps_, max_orb_yaw_rate_dps_);
    RCLCPP_INFO(get_logger(),
      "reset_on_reject_count: %d, accel_sanity=%s, drop_zero_accel=%s, accel_norm_window: [%.2f, %.2f]",
      reset_on_reject_count_, enable_accel_sanity_check_ ? "true" : "false",
      drop_zero_accel_samples_ ? "true" : "false", min_acc_norm_, max_acc_norm_);

    if (use_sim_time_) {
      this->set_parameter(rclcpp::Parameter("use_sim_time", true));
    }

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    path_msg_.header.frame_id = world_frame_;
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(pose_topic_, 10);
    camera_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(camera_pose_topic_, 10);
    path_pub_ = create_publisher<nav_msgs::msg::Path>(path_topic_, 10);

    // Use separate callback groups so high-rate IMU callbacks are not blocked by
    // heavy Stereo ORB-SLAM3 tracking. This is critical when Gazebo publishes
    // IMU at 90-100 Hz and camera at ~9 Hz.
    imu_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    left_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>();
    right_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>();

    left_sub_->subscribe(this, left_topic_, rmw_qos_profile_sensor_data);
    right_sub_->subscribe(this, right_topic_, rmw_qos_profile_sensor_data);

    sync_ = std::make_shared<Synchronizer>(SyncPolicy(sync_queue_size_), *left_sub_, *right_sub_);
    sync_->registerCallback(
      std::bind(&Orb3StereoImuNode::stereoCallback, this, std::placeholders::_1, std::placeholders::_2));

    rclcpp::SubscriptionOptions imu_sub_options;
    imu_sub_options.callback_group = imu_callback_group_;
    auto imu_qos = rclcpp::SensorDataQoS().keep_last(2000);
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, imu_qos,
      std::bind(&Orb3StereoImuNode::imuCallback, this, std::placeholders::_1),
      imu_sub_options);

    init_timer_ = create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&Orb3StereoImuNode::tryInitializeSlam, this));
  }

  ~Orb3StereoImuNode() override
  {
    if (slam_) {
      try {
        slam_->Shutdown();
      } catch (...) {
      }
    }
  }

private:
  using SyncPolicy =
    message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
  using Synchronizer = message_filters::Synchronizer<SyncPolicy>;

  static geometry_msgs::msg::Pose toPoseMsg(const tf2::Transform & t)
  {
    geometry_msgs::msg::Pose pose;
    pose.position.x = t.getOrigin().x();
    pose.position.y = t.getOrigin().y();
    pose.position.z = t.getOrigin().z();
    pose.orientation = tf2::toMsg(t.getRotation());
    return pose;
  }

  static double stampToSec(const builtin_interfaces::msg::Time & stamp)
  {
    return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
  }

  static double deg2rad(const double deg)
  {
    return deg * M_PI / 180.0;
  }

  static double normalizeAngle(double a)
  {
    while (a > M_PI) {
      a -= 2.0 * M_PI;
    }
    while (a < -M_PI) {
      a += 2.0 * M_PI;
    }
    return a;
  }



  bool imageMsgToMono8(
    const sensor_msgs::msg::Image::ConstSharedPtr & msg,
    const std::string & side,
    cv::Mat & gray)
  {
    const uint32_t w = msg->width;
    const uint32_t h = msg->height;
    const uint32_t step = msg->step;
    const std::string & enc = msg->encoding;

    if (w == 0 || h == 0 || step == 0 || msg->data.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "%s image invalid metadata: width=%u height=%u step=%u data=%zu encoding='%s'",
        side.c_str(), w, h, step, msg->data.size(), enc.c_str());
      return false;
    }

    // Hard guard: Gazebo cameras in this package are 320x240. If metadata is corrupted,
    // cv_bridge/OpenCV may try to allocate terabytes. Drop instead of converting.
    if (w > 1280 || h > 720) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
        "%s image metadata too large for this AGV demo: width=%u height=%u step=%u data=%zu encoding='%s'",
        side.c_str(), w, h, step, msg->data.size(), enc.c_str());
      return false;
    }

    auto check_size = [&](uint32_t channels) -> bool {
      const uint64_t min_step = static_cast<uint64_t>(w) * static_cast<uint64_t>(channels);
      const uint64_t needed = static_cast<uint64_t>(step) * static_cast<uint64_t>(h);
      if (step < min_step || needed > msg->data.size()) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
          "%s image wrongly formed: width=%u height=%u step=%u channels=%u need=%lu data=%zu encoding='%s'",
          side.c_str(), w, h, step, channels,
          static_cast<unsigned long>(needed), msg->data.size(), enc.c_str());
        return false;
      }
      return true;
    };

    try {
      if (enc == sensor_msgs::image_encodings::MONO8 || enc == "8UC1") {
        if (!check_size(1)) return false;
        cv::Mat src(h, w, CV_8UC1, const_cast<unsigned char *>(msg->data.data()), step);
        gray = src.clone();
      } else if (enc == sensor_msgs::image_encodings::RGB8 || enc == "rgb8") {
        if (!check_size(3)) return false;
        cv::Mat src(h, w, CV_8UC3, const_cast<unsigned char *>(msg->data.data()), step);
        cv::cvtColor(src, gray, cv::COLOR_RGB2GRAY);
      } else if (enc == sensor_msgs::image_encodings::BGR8 || enc == "bgr8" || enc == "8UC3") {
        if (!check_size(3)) return false;
        cv::Mat src(h, w, CV_8UC3, const_cast<unsigned char *>(msg->data.data()), step);
        cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);
      } else if (enc == sensor_msgs::image_encodings::RGBA8 || enc == "rgba8") {
        if (!check_size(4)) return false;
        cv::Mat src(h, w, CV_8UC4, const_cast<unsigned char *>(msg->data.data()), step);
        cv::cvtColor(src, gray, cv::COLOR_RGBA2GRAY);
      } else if (enc == sensor_msgs::image_encodings::BGRA8 || enc == "bgra8" || enc == "8UC4") {
        if (!check_size(4)) return false;
        cv::Mat src(h, w, CV_8UC4, const_cast<unsigned char *>(msg->data.data()), step);
        cv::cvtColor(src, gray, cv::COLOR_BGRA2GRAY);
      } else if (enc.empty()) {
        // Some Gazebo/camera bridges leave encoding empty. Infer safely from step.
        if (step >= w * 3 && msg->data.size() >= static_cast<size_t>(step) * h) {
          cv::Mat src(h, w, CV_8UC3, const_cast<unsigned char *>(msg->data.data()), step);
          cv::cvtColor(src, gray, cv::COLOR_RGB2GRAY);
        } else if (step >= w && msg->data.size() >= static_cast<size_t>(step) * h) {
          cv::Mat src(h, w, CV_8UC1, const_cast<unsigned char *>(msg->data.data()), step);
          gray = src.clone();
        } else {
          RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
            "%s image has empty encoding and inconsistent size: width=%u height=%u step=%u data=%zu",
            side.c_str(), w, h, step, msg->data.size());
          return false;
        }
      } else {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
          "%s image unsupported encoding '%s'. Expected mono8/rgb8/bgr8/rgba8/bgra8.",
          side.c_str(), enc.c_str());
        return false;
      }
    } catch (const cv::Exception & e) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
        "%s image OpenCV conversion failed: %s", side.c_str(), e.what());
      return false;
    }

    if (gray.empty() || gray.type() != CV_8UC1 || gray.cols != static_cast<int>(w) || gray.rows != static_cast<int>(h)) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
        "%s image conversion produced invalid gray image: cols=%d rows=%d type=%d",
        side.c_str(), gray.cols, gray.rows, gray.type());
      return false;
    }

    return true;
  }


  static tf2::Transform poseToTf(const geometry_msgs::msg::Pose & pose)
  {
    tf2::Transform t;
    tf2::fromMsg(pose, t);
    return t;
  }


  static void forcePlanarPose(geometry_msgs::msg::Pose & pose)
  {
    double roll = 0.0, pitch = 0.0, yaw = 0.0;
    tf2::Quaternion q;
    tf2::fromMsg(pose.orientation, q);
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    tf2::Quaternion q_planar;
    q_planar.setRPY(0.0, 0.0, yaw);
    q_planar.normalize();
    pose.position.z = 0.0;
    pose.orientation = tf2::toMsg(q_planar);
  }

  static bool poseIsFinite(const geometry_msgs::msg::Pose & pose)
  {
    return
      std::isfinite(pose.position.x) && std::isfinite(pose.position.y) &&
      std::isfinite(pose.position.z) && std::isfinite(pose.orientation.x) &&
      std::isfinite(pose.orientation.y) && std::isfinite(pose.orientation.z) &&
      std::isfinite(pose.orientation.w);
  }

  bool passWrapperMotionGate(const geometry_msgs::msg::PoseStamped & pose_msg)
  {
    if (!poseIsFinite(pose_msg.pose)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Reject ORB pose: non-finite pose values");
      return false;
    }

    const tf2::Transform current = poseToTf(pose_msg.pose);

    const double max_rp = deg2rad(max_roll_pitch_deg_);

    if (!has_last_published_pose_) {
      return true;
    }

    double dt = stampToSec(pose_msg.header.stamp) - last_published_pose_time_;
    if (!std::isfinite(dt) || dt <= 1e-4) {
      dt = 0.1;
    }
    dt = std::min(dt, 1.0);

    const tf2::Transform delta = last_published_pose_.inverse() * current;
    const tf2::Vector3 d = delta.getOrigin();
    const double dxy = std::hypot(d.x(), d.y());
    const double dz = std::abs(d.z());

    double droll = 0.0;
    double dpitch = 0.0;
    double dyaw = 0.0;
    tf2::Matrix3x3(delta.getRotation()).getRPY(droll, dpitch, dyaw);
    dyaw = normalizeAngle(dyaw);

    const double xy_limit = max_pose_jump_xy_m_ + max_orb_linear_speed_mps_ * dt;
    const double yaw_limit = deg2rad(max_pose_jump_yaw_deg_) + deg2rad(max_orb_yaw_rate_dps_) * dt;

    if (dxy > xy_limit || dz > max_pose_jump_z_m_ || std::abs(dyaw) > yaw_limit ||
        std::abs(droll) > max_rp || std::abs(dpitch) > max_rp) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "Reject ORB pose jump: dxy=%.3f/%.3f m, dz=%.3f/%.3f m, dyaw=%.2f/%.2f deg, droll=%.2f deg, dpitch=%.2f deg, dt=%.3f s",
        dxy, xy_limit, dz, max_pose_jump_z_m_,
        std::abs(dyaw) * 180.0 / M_PI, yaw_limit * 180.0 / M_PI,
        std::abs(droll) * 180.0 / M_PI, std::abs(dpitch) * 180.0 / M_PI, dt);
      return false;
    }

    return true;
  }

  void rememberPublishedPose(const geometry_msgs::msg::PoseStamped & pose_msg)
  {
    last_published_pose_ = poseToTf(pose_msg.pose);
    last_published_pose_time_ = stampToSec(pose_msg.header.stamp);
    has_last_published_pose_ = true;
  }

  void imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(imu_mutex_);

    if (imu_frame_from_msg_.empty() && !msg->header.frame_id.empty()) {
      imu_frame_from_msg_ = msg->header.frame_id;
      RCLCPP_INFO(get_logger(), "Detected IMU frame from topic: %s", imu_frame_from_msg_.c_str());
    }

    // Pre-filter bad IMU data at ingestion time to keep the queue clean
    const double ax = msg->linear_acceleration.x;
    const double ay = msg->linear_acceleration.y;
    const double az = msg->linear_acceleration.z;
    const double gx = msg->angular_velocity.x;
    const double gy = msg->angular_velocity.y;
    const double gz = msg->angular_velocity.z;

    if (!std::isfinite(ax) || !std::isfinite(ay) || !std::isfinite(az) ||
        !std::isfinite(gx) || !std::isfinite(gy) || !std::isfinite(gz)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Dropping IMU sample with NaN/Inf values");
      return;
    }

    // Some Gazebo IMU setups report acceleration with a different gravity convention.
    // Do not drop near-zero acceleration unless explicitly requested; otherwise ORB-SLAM3
    // may receive empty IMU windows and print "not IMU meas" after every reset.
    const double acc_norm = std::sqrt(ax*ax + ay*ay + az*az);
    if (drop_zero_accel_samples_ && acc_norm < 0.1) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Dropping IMU sample with near-zero accel norm %.4f", acc_norm);
      return;
    }

    imu_queue_.push_back(msg);

    while (imu_queue_.size() > 8000) {
      imu_queue_.pop_front();
    }
  }

  std::string makeTbcBlock(const tf2::Transform & tbc) const
  {
    tf2::Matrix3x3 R(tbc.getRotation());

    auto emit = [&](const std::string & key) {
      std::ostringstream oss;
      oss << key << ": !!opencv-matrix\n";
      oss << "  rows: 4\n";
      oss << "  cols: 4\n";
      oss << "  dt: f\n";
      oss << std::fixed << std::setprecision(9);
      oss << "  data: [";
      oss << R[0][0] << ", " << R[0][1] << ", " << R[0][2] << ", " << tbc.getOrigin().x() << ", ";
      oss << R[1][0] << ", " << R[1][1] << ", " << R[1][2] << ", " << tbc.getOrigin().y() << ", ";
      oss << R[2][0] << ", " << R[2][1] << ", " << R[2][2] << ", " << tbc.getOrigin().z() << ", ";
      oss << "0.0, 0.0, 0.0, 1.0]\n";
      return oss.str();
    };

    std::ostringstream out;
    out << emit("IMU.T_b_c1") << "\n";
    out << emit("IMU.T_b_c") << "\n";
    out << emit("Tbc");
    return out.str();
  }


  static tf2::Transform makeManualRosOpticalTbc()
  {
    // Tbc = T_body_camera for ROS body frame x-forward/y-left/z-up and
    // ROS optical camera frame x-right/y-down/z-forward.
    tf2::Matrix3x3 R(
      0.0, -1.0,  0.0,
      0.0,  0.0, -1.0,
      1.0,  0.0,  0.0);
    tf2::Quaternion q;
    R.getRotation(q);
    q.normalize();
    tf2::Transform T;
    T.setOrigin(tf2::Vector3(0.0, 0.0, 0.0));
    T.setRotation(q);
    return T;
  }

  static bool transformLooksFinite(const tf2::Transform & t)
  {
    const tf2::Vector3 o = t.getOrigin();
    const tf2::Quaternion q = t.getRotation();
    return std::isfinite(o.x()) && std::isfinite(o.y()) && std::isfinite(o.z()) &&
           std::isfinite(q.x()) && std::isfinite(q.y()) && std::isfinite(q.z()) && std::isfinite(q.w());
  }

  void logTbc(const tf2::Transform & tbc) const
  {
    tf2::Matrix3x3 R(tbc.getRotation());
    RCLCPP_INFO(get_logger(), "Using Tbc mode '%s'", tbc_mode_.c_str());
    RCLCPP_INFO(get_logger(), "Tbc translation: [%.4f %.4f %.4f]",
      tbc.getOrigin().x(), tbc.getOrigin().y(), tbc.getOrigin().z());
    RCLCPP_INFO(get_logger(), "Tbc rotation rows: [%.3f %.3f %.3f] [%.3f %.3f %.3f] [%.3f %.3f %.3f]",
      R[0][0], R[0][1], R[0][2], R[1][0], R[1][1], R[1][2], R[2][0], R[2][1], R[2][2]);
  }

  bool computeTbc(const std::string & imu_frame_used, tf2::Transform & tbc)
  {
    try {
      if (tbc_mode_ == "manual_ros_optical") {
        tbc = makeManualRosOpticalTbc();
      } else if (tbc_mode_ == "manual_inverse_ros_optical") {
        tbc = makeManualRosOpticalTbc().inverse();
      } else if (tbc_mode_ == "tf_camera_body") {
        geometry_msgs::msg::TransformStamped tf_camera_from_imu =
          tf_buffer_.lookupTransform(camera_frame_, imu_frame_used, tf2::TimePointZero);
        tf2::fromMsg(tf_camera_from_imu.transform, tbc);
      } else {  // default tf_body_camera
        geometry_msgs::msg::TransformStamped tf_imu_from_camera =
          tf_buffer_.lookupTransform(imu_frame_used, camera_frame_, tf2::TimePointZero);
        tf2::fromMsg(tf_imu_from_camera.transform, tbc);
      }
    } catch (const tf2::TransformException & ex) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Waiting for TF to build Tbc. mode=%s imu_frame=%s camera_frame=%s: %s",
        tbc_mode_.c_str(), imu_frame_used.c_str(), camera_frame_.c_str(), ex.what());
      return false;
    }

    if (!transformLooksFinite(tbc)) {
      RCLCPP_ERROR(get_logger(), "Computed Tbc has NaN/Inf values");
      return false;
    }
    tf2::Quaternion q = tbc.getRotation();
    q.normalize();
    tbc.setRotation(q);
    logTbc(tbc);
    return true;
  }

  void resetOrbAfterBadState(const std::string & reason)
  {
    if (!slam_) {
      return;
    }
    RCLCPP_WARN(get_logger(), "Resetting ORB-SLAM3 active map: %s", reason.c_str());
    try {
      slam_->ResetActiveMap();
    } catch (const std::exception & e) {
      RCLCPP_WARN(get_logger(), "ResetActiveMap() threw: %s. Trying full Reset().", e.what());
      try { slam_->Reset(); } catch (...) {}
    } catch (...) {
      try { slam_->Reset(); } catch (...) {}
    }
    has_last_published_pose_ = false;
    first_valid_pose_ = false;
    identity_count_ = 0;
    rejected_orb_pose_count_ = 0;
    last_frame_time_ = -1.0;
    path_msg_.poses.clear();
  }


  static std::string escapeYamlDoubleQuoted(const std::string & value)
  {
    std::string out;
    out.reserve(value.size());
    for (const char c : value) {
      if (c == '\\' || c == '"') {
        out.push_back('\\');
      }
      out.push_back(c);
    }
    return out;
  }

  static void setYamlStringKey(std::string & text, const std::string & key, const std::string & value)
  {
    const std::string line = key + ": \"" + escapeYamlDoubleQuoted(value) + "\"";
    const std::string pattern = key + ":";
    std::size_t pos = text.find(pattern);
    if (pos == std::string::npos) {
      text += "\n" + line + "\n";
      return;
    }
    const std::size_t end = text.find('\n', pos);
    if (end == std::string::npos) {
      text.replace(pos, std::string::npos, line);
    } else {
      text.replace(pos, end - pos, line);
    }
  }

  bool buildRuntimeSettingsFile(const tf2::Transform & tbc)
  {
    std::ifstream ifs(settings_template_file_);
    if (!ifs.is_open()) {
      RCLCPP_ERROR(get_logger(), "Cannot open settings template file: %s", settings_template_file_.c_str());
      return false;
    }

    std::stringstream buffer;
    buffer << ifs.rdbuf();
    std::string text = buffer.str();

    const std::string token = "__TBC_BLOCK__";
    const std::string block = makeTbcBlock(tbc);

    auto pos = text.find(token);
    if (pos != std::string::npos) {
      text.replace(pos, token.size(), block);
    } else {
      text += "\n\n" + block + "\n";
    }

    setYamlStringKey(text, "System.LoadAtlasFromFile", atlas_load_file_);
    setYamlStringKey(text, "System.SaveAtlasToFile", atlas_save_file_);

    std::ofstream ofs(runtime_settings_file_);
    if (!ofs.is_open()) {
      RCLCPP_ERROR(get_logger(), "Cannot write runtime settings file: %s", runtime_settings_file_.c_str());
      return false;
    }

    ofs << text;
    ofs.close();
    return true;
  }

  void tryInitializeSlam()
  {
    if (slam_ready_) {
      return;
    }

    const std::string imu_frame_used =
      !imu_frame_param_.empty() ? imu_frame_param_ : imu_frame_from_msg_;

    if (imu_frame_used.empty()) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Waiting for first IMU message to detect imu_frame...");
      return;
    }

    tf2::Transform Tbc;
    if (!computeTbc(imu_frame_used, Tbc)) {
      return;
    }

    if (!buildRuntimeSettingsFile(Tbc)) {
      return;
    }

    try {
      slam_ = std::make_unique<ORB_SLAM3::System>(
        vocab_file_, runtime_settings_file_, ORB_SLAM3::System::IMU_STEREO, use_viewer_);
      slam_ready_ = true;
      imu_frame_resolved_ = imu_frame_used;

      RCLCPP_INFO(get_logger(), "ORB-SLAM3 IMU_STEREO initialized");
      RCLCPP_INFO(get_logger(), "Resolved imu_frame: %s", imu_frame_resolved_.c_str());
      RCLCPP_INFO(get_logger(), "Using Tbc mode: %s", tbc_mode_.c_str());
      if (localization_mode_) {
        RCLCPP_INFO(get_logger(), "Activating ORB-SLAM3 localization mode");
        const bool localization_ready = slam_->ActivateLocalizationModeBlocking(2.0);
        RCLCPP_WARN(
          get_logger(),
          "ORB-SLAM3 stereo-IMU localization-only is active. local_mapping_stopped=%s",
          localization_ready ? "true" : "false");
      }

      if (init_timer_) {
        init_timer_->cancel();
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Failed to initialize ORB-SLAM3 IMU_STEREO: %s", e.what());
    }
  }

  std::vector<ORB_SLAM3::IMU::Point> collectImuMeasurements(
    const double last_frame_time,
    const double current_frame_time)
  {
    std::vector<ORB_SLAM3::IMU::Point> out;
    std::lock_guard<std::mutex> lock(imu_mutex_);

    while (!imu_queue_.empty() &&
      stampToSec(imu_queue_.front()->header.stamp) < (last_frame_time - 1.0))
    {
      imu_queue_.pop_front();
    }

    for (const auto & msg : imu_queue_) {
      const double t = stampToSec(msg->header.stamp) + imu_time_offset_s_;

      if (t <= last_frame_time) {
        continue;
      }
      if (t > current_frame_time + 1e-6) {
        break;
      }

      const float ax = static_cast<float>(msg->linear_acceleration.x);
      const float ay = static_cast<float>(msg->linear_acceleration.y);
      const float az = static_cast<float>(msg->linear_acceleration.z);

      const float gx = static_cast<float>(msg->angular_velocity.x);
      const float gy = static_cast<float>(msg->angular_velocity.y);
      const float gz = static_cast<float>(msg->angular_velocity.z);

      // Drop any sample with NaN/Inf — Sophus::SO3::exp will abort on NaN omega
      if (!std::isfinite(ax) || !std::isfinite(ay) || !std::isfinite(az) ||
          !std::isfinite(gx) || !std::isfinite(gy) || !std::isfinite(gz)) {
        continue;
      }

      // Drop zero-accel samples only if requested. Some simulators publish linear acceleration
      // without gravity; keeping these samples is better than giving ORB-SLAM3 an empty window.
      const float acc_norm_sq = ax * ax + ay * ay + az * az;
      if (drop_zero_accel_samples_ && acc_norm_sq < 0.01f) {
        continue;
      }

      // Clamp gyro để tránh overflow trong Sophus pre-integration
      const float GYR_MAX = 10.0f;  // rad/s - vượt quá này là noise
      const float gxc = std::max(-GYR_MAX, std::min(GYR_MAX, gx));
      const float gyc = std::max(-GYR_MAX, std::min(GYR_MAX, gy));
      const float gzc = std::max(-GYR_MAX, std::min(GYR_MAX, gz));
      out.emplace_back(ax, ay, az, gxc, gyc, gzc, t);
    }

    // Do not erase IMU samples too aggressively. The next camera frame may need
    // samples close to last_frame_time, especially with /clock jitter and
    // small timestamp offsets. Keep a 0.5 s margin behind the previous frame.
    while (!imu_queue_.empty() &&
      stampToSec(imu_queue_.front()->header.stamp) <= last_frame_time - 0.5)
    {
      imu_queue_.pop_front();
    }

    return out;
  }

  bool imuWindowLooksReasonable(
    const std::vector<ORB_SLAM3::IMU::Point> & imu_measurements,
    double * mean_acc_norm_out = nullptr) const
  {
    if (imu_measurements.empty()) {
      if (mean_acc_norm_out) {
        *mean_acc_norm_out = 0.0;
      }
      return false;
    }

    double sum_norm = 0.0;
    for (const auto & p : imu_measurements) {
      const double n = std::sqrt(
        static_cast<double>(p.a.x()) * static_cast<double>(p.a.x()) +
        static_cast<double>(p.a.y()) * static_cast<double>(p.a.y()) +
        static_cast<double>(p.a.z()) * static_cast<double>(p.a.z()));
      if (!std::isfinite(n)) {
        if (mean_acc_norm_out) {
          *mean_acc_norm_out = std::numeric_limits<double>::quiet_NaN();
        }
        return false;
      }
      sum_norm += n;
    }

    const double mean_norm = sum_norm / static_cast<double>(imu_measurements.size());
    if (mean_acc_norm_out) {
      *mean_acc_norm_out = mean_norm;
    }
    return mean_norm >= min_acc_norm_ && mean_norm <= max_acc_norm_;
  }

  void stereoCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr & left_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr & right_msg)
  {
    if (!slam_ready_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "SLAM not ready yet. Waiting for IMU frame and TF...");
      return;
    }

    const double tl = stampToSec(left_msg->header.stamp);
    const double tr = stampToSec(right_msg->header.stamp);
    const double dt = std::abs(tl - tr);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "left-right dt = %.4f s", dt);

    if (dt > max_stereo_dt_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Skipping stereo pair: dt %.4f > max %.4f", dt, max_stereo_dt_);
      return;
    }

    try {
      // Do not call cv_bridge::toCvCopy(..., MONO8) directly here. If a ROS Image
      // message has inconsistent metadata, cv_bridge/OpenCV can try to allocate a
      // huge matrix. Validate width/height/step/data first, then convert manually.
      cv::Mat left;
      cv::Mat right;
      if (!imageMsgToMono8(left_msg, "left", left) || !imageMsgToMono8(right_msg, "right", right)) {
        return;
      }

      if (left.empty() || right.empty() || left.size() != right.size()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
          "Invalid stereo images: left/right empty or size mismatch. left=%dx%d right=%dx%d",
          left.cols, left.rows, right.cols, right.rows);
        return;
      }

      const double tframe = tl;
      std::vector<ORB_SLAM3::IMU::Point> imu_measurements;

      {
        std::lock_guard<std::mutex> lock(imu_mutex_);
        if (static_cast<int>(imu_queue_.size()) < min_imu_boot_samples_) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "Waiting IMU warmup: have %zu samples, need >= %d",
            imu_queue_.size(), min_imu_boot_samples_);
          return;
        }
      }

      if (last_frame_time_ < 0.0) {
        last_frame_time_ = tframe;
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Primed first stereo frame at %.6f, waiting next frame for IMU window",
          tframe);
        return;
      }

      imu_measurements = collectImuMeasurements(last_frame_time_, tframe);

      if (static_cast<int>(imu_measurements.size()) < min_imu_between_frames_) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Too few IMU samples between frames (%zu < %d), skipping frame",
          imu_measurements.size(), min_imu_between_frames_);
        last_frame_time_ = tframe;
        return;
      }

      if (enable_accel_sanity_check_) {
        double mean_acc_norm = 0.0;
        if (!imuWindowLooksReasonable(imu_measurements, &mean_acc_norm)) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "IMU window failed accel sanity check: mean_norm=%.4f outside [%.2f, %.2f], samples=%zu; skipping frame",
            mean_acc_norm, min_acc_norm_, max_acc_norm_, imu_measurements.size());
          last_frame_time_ = tframe;
          return;
        }
      }

      Sophus::SE3f Tcw;
      try {
        Tcw = slam_->TrackStereo(left, right, tframe, imu_measurements);
      } catch (const std::exception & slam_ex) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "TrackStereo threw exception: %s — skipping frame", slam_ex.what());
        last_frame_time_ = tframe;
        return;
      } catch (...) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "TrackStereo threw unknown exception — skipping frame");
        last_frame_time_ = tframe;
        return;
      }
      last_frame_time_ = tframe;
      Eigen::Matrix4f Tcw_mat = Tcw.matrix();

      if (!Tcw_mat.allFinite()) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "ORB-SLAM3 returned non-finite pose");
        return;
      }

      Sophus::SE3f Twc = Tcw.inverse();
      Eigen::Matrix4f Twc_mat = Twc.matrix();

      const double px = static_cast<double>(Twc_mat(0, 3));
      const double py = static_cast<double>(Twc_mat(1, 3));
      const double pz = static_cast<double>(Twc_mat(2, 3));

      tf2::Matrix3x3 Rwc(
        static_cast<double>(Twc_mat(0, 0)), static_cast<double>(Twc_mat(0, 1)), static_cast<double>(Twc_mat(0, 2)),
        static_cast<double>(Twc_mat(1, 0)), static_cast<double>(Twc_mat(1, 1)), static_cast<double>(Twc_mat(1, 2)),
        static_cast<double>(Twc_mat(2, 0)), static_cast<double>(Twc_mat(2, 1)), static_cast<double>(Twc_mat(2, 2)));

      tf2::Quaternion qwc;
      Rwc.getRotation(qwc);
      qwc.normalize();

      const bool almost_identity =
        std::abs(px) < 1e-6 &&
        std::abs(py) < 1e-6 &&
        std::abs(pz) < 1e-6 &&
        std::abs(qwc.x()) < 1e-6 &&
        std::abs(qwc.y()) < 1e-6 &&
        std::abs(qwc.z()) < 1e-6 &&
        std::abs(qwc.w() - 1.0) < 1e-6;

      if (almost_identity) {
        identity_count_++;
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "ORB-SLAM3 still returns identity pose, skipping publish. Likely not tracking yet.");
        return;
      }

      if (!first_valid_pose_) {
        first_valid_pose_ = true;
        RCLCPP_INFO(get_logger(), "ORB-SLAM3 produced first non-identity pose");
      }

      tf2::Transform T_world_camera;
      T_world_camera.setOrigin(tf2::Vector3(px, py, pz));
      T_world_camera.setRotation(qwc);

      geometry_msgs::msg::PoseStamped cam_pose_msg;
      cam_pose_msg.header.stamp = left_msg->header.stamp;
      cam_pose_msg.header.frame_id = world_frame_;
      cam_pose_msg.pose = toPoseMsg(T_world_camera);
      camera_pose_pub_->publish(cam_pose_msg);

      geometry_msgs::msg::PoseStamped base_pose_msg;
      base_pose_msg.header = cam_pose_msg.header;

      try {
        geometry_msgs::msg::TransformStamped tf_cam_from_base =
          tf_buffer_.lookupTransform(camera_frame_, base_frame_, tf2::TimePointZero);

        tf2::Transform T_cam_base;
        tf2::fromMsg(tf_cam_from_base.transform, T_cam_base);

        tf2::Transform T_world_base = T_world_camera * T_cam_base;
        base_pose_msg.pose = toPoseMsg(T_world_base);
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "Cannot transform %s <- %s: %s. Camera pose is debug-only; "
          "robot base pose is not published.",
          camera_frame_.c_str(), base_frame_.c_str(), ex.what());
        return;
      }

      if (force_planar_output_) {
        forcePlanarPose(base_pose_msg.pose);
      }

      if (enable_wrapper_motion_gate_ && !passWrapperMotionGate(base_pose_msg)) {
        rejected_orb_pose_count_++;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1500,
          "ORB pose rejected by wrapper motion gate; rejected_count=%d. Pose was not published to ROS consumers.",
          rejected_orb_pose_count_);
        if (reset_on_reject_count_ > 0 && rejected_orb_pose_count_ >= reset_on_reject_count_) {
          resetOrbAfterBadState("too many wrapper motion gate rejects; likely IMU/gravity flip");
        }
        return;
      }

      rejected_orb_pose_count_ = 0;
      rememberPublishedPose(base_pose_msg);
      pose_pub_->publish(base_pose_msg);

      if (publish_tf_) {
        publishOrbTf(base_pose_msg);
      }

      path_msg_.header = base_pose_msg.header;
      path_msg_.poses.push_back(base_pose_msg);
      if (path_msg_.poses.size() > 2000) {
        path_msg_.poses.erase(path_msg_.poses.begin());
      }
      path_pub_->publish(path_msg_);

    } catch (const cv::Exception & e) {
      RCLCPP_ERROR(get_logger(), "OpenCV exception in stereo callback: %s", e.what());
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "ORB3 stereo callback exception: %s", e.what());
    }
  }

  void publishOrbTf(const geometry_msgs::msg::PoseStamped & base_pose_msg)
  {
    geometry_msgs::msg::TransformStamped t;
    t.header = base_pose_msg.header;
    t.child_frame_id = base_frame_;
    t.transform.translation.x = base_pose_msg.pose.position.x;
    t.transform.translation.y = base_pose_msg.pose.position.y;
    t.transform.translation.z = base_pose_msg.pose.position.z;
    t.transform.rotation = base_pose_msg.pose.orientation;
    tf_broadcaster_->sendTransform(t);
  }

  std::string vocab_file_;
  std::string settings_template_file_;
  std::string runtime_settings_file_;
  std::string atlas_load_file_;
  std::string atlas_save_file_;
  std::string left_topic_;
  std::string right_topic_;
  std::string imu_topic_;
  std::string world_frame_;
  std::string base_frame_;
  std::string camera_frame_;
  std::string imu_frame_param_;
  std::string tbc_mode_;
  std::string imu_frame_from_msg_;
  std::string imu_frame_resolved_;
  std::string path_topic_;
  std::string pose_topic_;
  std::string camera_pose_topic_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> left_sub_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> right_sub_;
  std::shared_ptr<Synchronizer> sync_;
  rclcpp::CallbackGroup::SharedPtr imu_callback_group_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::TimerBase::SharedPtr init_timer_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::mutex imu_mutex_;
  std::deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu_queue_;

  bool use_sim_time_{true};
  bool use_viewer_{false};
  bool publish_tf_{false};
  bool force_planar_output_{true};
  bool localization_mode_{false};
  bool slam_ready_{false};
  int sync_queue_size_{30};
  double max_stereo_dt_{0.04};
  double imu_time_offset_s_{0.0};
  int min_imu_boot_samples_{7};
  int min_imu_between_frames_{3};
  bool enable_accel_sanity_check_{false};
  bool drop_zero_accel_samples_{false};
  double min_acc_norm_{0.0};
  double max_acc_norm_{30.0};

  bool enable_wrapper_motion_gate_{true};
  double max_pose_jump_xy_m_{0.25};
  double max_pose_jump_z_m_{0.12};
  double max_pose_jump_yaw_deg_{12.0};
  double max_roll_pitch_deg_{7.0};
  double max_orb_linear_speed_mps_{1.20};
  double max_orb_yaw_rate_dps_{90.0};
  int reset_on_reject_count_{12};
  bool has_last_published_pose_{false};
  tf2::Transform last_published_pose_;
  double last_published_pose_time_{-1.0};
  int rejected_orb_pose_count_{0};

  int identity_count_{0};
  bool first_valid_pose_{false};
  double last_frame_time_{-1.0};

  std::unique_ptr<ORB_SLAM3::System> slam_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr camera_pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  nav_msgs::msg::Path path_msg_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Orb3StereoImuNode>();
  // Multi-threaded executor keeps IMU subscription alive while TrackStereo() and
  // Pangolin viewer are doing heavier work in Stereo callbacks.
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
