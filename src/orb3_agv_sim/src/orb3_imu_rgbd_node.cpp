#include <memory>
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

#include <rclcpp/rclcpp.hpp>
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

class Orb3RgbdNode : public rclcpp::Node
{
public:
  Orb3RgbdNode()
  : Node("orb3_rgbd_node"),
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
      "/home/trongtuan/agv_ws1/src/agv0509test6/config/orb3_rgbd_imu_gz.template.yaml");
    runtime_settings_file_ = get_or_declare_string(
      "runtime_settings_file", "/tmp/orb3_rgbd_imu_runtime.yaml");

    rgb_topic_ = get_or_declare_string("rgb_topic", "/camera/image_raw");
    depth_topic_ = get_or_declare_string("depth_topic", "/camera/depth/image_raw");
    imu_topic_ = get_or_declare_string("imu_topic", "/imu/data_raw");

    world_frame_ = get_or_declare_string("world_frame", "orb_map");
    base_frame_ = get_or_declare_string("base_frame", "Base_footprint");
    camera_frame_ = get_or_declare_string("camera_frame", "camera_link_optical");
    imu_frame_param_ = get_or_declare_string("imu_frame", "");

    use_sim_time_ = get_or_declare_bool("use_sim_time", true);
    use_viewer_ = get_or_declare_bool("use_viewer", false);
    publish_tf_ = get_or_declare_bool("publish_tf", false);

    path_topic_ = get_or_declare_string("path_topic", "/orbslam3/path");
    pose_topic_ = get_or_declare_string("pose_topic", "/orbslam3/pose");
    camera_pose_topic_ = get_or_declare_string("camera_pose_topic", "/orbslam3/camera_pose");

    sync_queue_size_ = get_or_declare_int("sync_queue_size", 30);
    max_rgb_depth_dt_ = get_or_declare_double("max_rgb_depth_dt", 0.04);
    min_imu_boot_samples_ = get_or_declare_int("min_imu_boot_samples", 20);
    min_imu_between_frames_ = get_or_declare_int("min_imu_between_frames", 3);

    RCLCPP_INFO(get_logger(), "Starting ORB-SLAM3 RGB-D + IMU node");
    RCLCPP_INFO(get_logger(), "vocabulary_file: %s", vocab_file_.c_str());
    RCLCPP_INFO(get_logger(), "settings_template_file: %s", settings_template_file_.c_str());
    RCLCPP_INFO(get_logger(), "runtime_settings_file: %s", runtime_settings_file_.c_str());
    RCLCPP_INFO(get_logger(), "rgb_topic: %s", rgb_topic_.c_str());
    RCLCPP_INFO(get_logger(), "depth_topic: %s", depth_topic_.c_str());
    RCLCPP_INFO(get_logger(), "imu_topic: %s", imu_topic_.c_str());
    RCLCPP_INFO(get_logger(), "base_frame: %s", base_frame_.c_str());
    RCLCPP_INFO(get_logger(), "camera_frame: %s", camera_frame_.c_str());
    RCLCPP_INFO(get_logger(), "imu_frame(param): %s", imu_frame_param_.c_str());
    RCLCPP_INFO(get_logger(), "use_viewer: %s", use_viewer_ ? "true" : "false");
    RCLCPP_INFO(get_logger(), "publish_tf: %s", publish_tf_ ? "true" : "false");
    RCLCPP_INFO(get_logger(), "max_rgb_depth_dt: %.3f", max_rgb_depth_dt_);
    RCLCPP_INFO(get_logger(), "min_imu_boot_samples: %d", min_imu_boot_samples_);
    RCLCPP_INFO(get_logger(), "min_imu_between_frames: %d", min_imu_between_frames_);

    if (use_sim_time_) {
      this->set_parameter(rclcpp::Parameter("use_sim_time", true));
    }

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    path_msg_.header.frame_id = world_frame_;
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(pose_topic_, 10);
    camera_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(camera_pose_topic_, 10);
    path_pub_ = create_publisher<nav_msgs::msg::Path>(path_topic_, 10);

    rgb_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>();
    depth_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>();

    rgb_sub_->subscribe(this, rgb_topic_, rmw_qos_profile_sensor_data);
    depth_sub_->subscribe(this, depth_topic_, rmw_qos_profile_sensor_data);

    sync_ = std::make_shared<Synchronizer>(SyncPolicy(sync_queue_size_), *rgb_sub_, *depth_sub_);
    sync_->registerCallback(
      std::bind(&Orb3RgbdNode::rgbdCallback, this, std::placeholders::_1, std::placeholders::_2));

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, rclcpp::SensorDataQoS(),
      std::bind(&Orb3RgbdNode::imuCallback, this, std::placeholders::_1));

    init_timer_ = create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&Orb3RgbdNode::tryInitializeSlam, this));
  }

  ~Orb3RgbdNode() override
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

    // Reject zero-gravity samples (Gazebo physics not yet stable)
    const double acc_norm = std::sqrt(ax*ax + ay*ay + az*az);
    if (acc_norm < 0.1) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Dropping IMU sample with near-zero accel norm %.4f (Gazebo not stable yet)", acc_norm);
      return;
    }

    imu_queue_.push_back(msg);

    while (imu_queue_.size() > 4000) {
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

    geometry_msgs::msg::TransformStamped tf_imu_from_camera;
    try {
      tf_imu_from_camera =
        tf_buffer_.lookupTransform(imu_frame_used, camera_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Waiting for TF %s <- %s before ORB init: %s",
        imu_frame_used.c_str(), camera_frame_.c_str(), ex.what());
      return;
    }

    tf2::Transform Tbc;
    tf2::fromMsg(tf_imu_from_camera.transform, Tbc);

    if (!buildRuntimeSettingsFile(Tbc)) {
      return;
    }

    try {
      // RGBD mode avoids Sophus SO3::exp NaN crash in LocalMapping thread.
      // Robot can be static. This legacy file is not used by CMake; main IMU node is orb3_imu_rgbd_node_imu.cpp. Old note:
      //   ~/Dev/ORB_SLAM3/Thirdparty/Sophus/sophus/so3.hpp line ~614
      //   add before SOPHUS_ENSURE: if(!std::isfinite(omega_sq)){return SO3<Scalar>();}
      slam_ = std::make_unique<ORB_SLAM3::System>(
        vocab_file_, runtime_settings_file_, ORB_SLAM3::System::RGBD, use_viewer_);
      slam_ready_ = true;
      imu_frame_resolved_ = imu_frame_used;

      RCLCPP_INFO(get_logger(), "ORB-SLAM3 RGBD initialized (legacy node, IMU fusion disabled)");
      RCLCPP_INFO(get_logger(), "Resolved imu_frame: %s", imu_frame_resolved_.c_str());
      RCLCPP_INFO(get_logger(), "Using Tbc from TF %s <- %s", imu_frame_resolved_.c_str(), camera_frame_.c_str());

      if (init_timer_) {
        init_timer_->cancel();
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Failed to initialize ORB-SLAM3 RGBD: %s", e.what());
    }
  }

  float computeValidDepthRatio(const cv::Mat & depth) const
  {
    int valid = 0;
    int total = 0;

    for (int v = 0; v < depth.rows; v += 4) {
      const float * row_ptr = depth.ptr<float>(v);
      for (int u = 0; u < depth.cols; u += 4) {
        const float z = row_ptr[u];
        total++;
        if (std::isfinite(z) && z > 0.05f) {
          valid++;
        }
      }
    }

    if (total == 0) {
      return 0.0f;
    }
    return static_cast<float>(valid) / static_cast<float>(total);
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
      const double t = stampToSec(msg->header.stamp);

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

      // Drop zero-accel samples (Gazebo IMU plugin outputs zeros before physics stabilizes)
      const float acc_norm_sq = ax * ax + ay * ay + az * az;
      if (acc_norm_sq < 0.01f) {  // < 0.1 m/s^2 norm — physically impossible (gravity missing)
        continue;
      }

      out.emplace_back(ax, ay, az, gx, gy, gz, t);
    }

    while (!imu_queue_.empty() &&
      stampToSec(imu_queue_.front()->header.stamp) <= current_frame_time - 0.05)
    {
      imu_queue_.pop_front();
    }

    return out;
  }

  bool imuWindowLooksReasonable(const std::vector<ORB_SLAM3::IMU::Point> & imu_measurements) const
  {
    if (imu_measurements.empty()) {
      return false;
    }

    double sum_norm = 0.0;
    for (const auto & p : imu_measurements) {
      const double n = std::sqrt(
        static_cast<double>(p.a.x()) * static_cast<double>(p.a.x()) +
        static_cast<double>(p.a.y()) * static_cast<double>(p.a.y()) +
        static_cast<double>(p.a.z()) * static_cast<double>(p.a.z()));
      if (!std::isfinite(n)) {
        return false;
      }
      sum_norm += n;
    }

    const double mean_norm = sum_norm / static_cast<double>(imu_measurements.size());
    return mean_norm >= min_acc_norm_ && mean_norm <= max_acc_norm_;
  }

  void rgbdCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr & rgb_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg)
  {
    if (!slam_ready_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "SLAM not ready yet. Waiting for IMU frame and TF...");
      return;
    }

    const double tr = stampToSec(rgb_msg->header.stamp);
    const double td = stampToSec(depth_msg->header.stamp);
    const double dt = std::abs(tr - td);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "rgb-depth dt = %.4f s", dt);

    if (dt > max_rgb_depth_dt_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Skipping RGB-D pair: dt %.4f > max %.4f", dt, max_rgb_depth_dt_);
      return;
    }

    try {
      auto rgb_cv = cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::RGB8);
      auto depth_cv = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);

      const cv::Mat & rgb = rgb_cv->image;
      const cv::Mat & depth = depth_cv->image;
      const float valid_depth_ratio = computeValidDepthRatio(depth);

      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "valid depth ratio = %.3f", valid_depth_ratio);

      if (valid_depth_ratio < 0.20f) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Depth ratio too low: %.3f, skipping frame", valid_depth_ratio);
        return;
      }

      const double tframe = tr;
      std::vector<ORB_SLAM3::IMU::Point> imu_measurements;

      // RGBD mode: no IMU warmup needed
      {
        std::lock_guard<std::mutex> lock(imu_mutex_);
        RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 5000,
          "IMU queue size: %zu", imu_queue_.size());
      }

      if (last_frame_time_ < 0.0) {
        last_frame_time_ = tframe;
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Primed first RGB-D frame at %.6f, waiting next frame for IMU window",
          tframe);
        return;
      }

      // Collect IMU for logging only (not passed to TrackRGBD in RGBD mode)
      imu_measurements = collectImuMeasurements(last_frame_time_, tframe);
      RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 2000,
        "IMU samples in window: %zu", imu_measurements.size());

      Sophus::SE3f Tcw;
      try {
        // In RGBD mode, TrackRGBD takes only 3 args (no IMU)
        Tcw = slam_->TrackRGBD(rgb, depth, tframe);
      } catch (const std::exception & slam_ex) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "TrackRGBD threw exception (likely IMU NaN in ORB internal): %s — skipping frame",
          slam_ex.what());
        last_frame_time_ = tframe;
        return;
      } catch (...) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "TrackRGBD threw unknown exception — skipping frame");
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
      cam_pose_msg.header.stamp = rgb_msg->header.stamp;
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
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 3000,
          "Cannot transform %s <- %s: %s. Camera pose is debug-only; "
          "robot base pose is not published.",
          camera_frame_.c_str(), base_frame_.c_str(), ex.what());
        return;
      }

      pose_pub_->publish(base_pose_msg);

      if (publish_tf_) {
        publishOrbTf(base_pose_msg);
      }

      path_msg_.header.stamp = rgb_msg->header.stamp;
      path_msg_.poses.push_back(base_pose_msg);
      path_pub_->publish(path_msg_);

      identity_count_ = 0;

    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "ORB3 callback exception: %s", e.what());
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
  std::string rgb_topic_;
  std::string depth_topic_;
  std::string imu_topic_;
  std::string world_frame_;
  std::string base_frame_;
  std::string camera_frame_;
  std::string imu_frame_param_;
  std::string imu_frame_from_msg_;
  std::string imu_frame_resolved_;
  std::string path_topic_;
  std::string pose_topic_;
  std::string camera_pose_topic_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> rgb_sub_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> depth_sub_;
  std::shared_ptr<Synchronizer> sync_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::TimerBase::SharedPtr init_timer_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::mutex imu_mutex_;
  std::deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu_queue_;

  bool use_sim_time_{true};
  bool use_viewer_{false};
  bool publish_tf_{false};
  bool slam_ready_{false};
  int sync_queue_size_{30};
  double max_rgb_depth_dt_{0.04};
  int min_imu_boot_samples_{20};
  int min_imu_between_frames_{3};
  double min_acc_norm_{5.0};
  double max_acc_norm_{15.0};
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
  auto node = std::make_shared<Orb3RgbdNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
