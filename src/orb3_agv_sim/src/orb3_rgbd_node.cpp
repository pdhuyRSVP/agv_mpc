// ORB-SLAM3 RGB-D only node.
// Subscribes to synchronized RGB + depth images, runs ORB_SLAM3::System in
// RGBD mode, and publishes the resulting camera pose + robot path on ROS 2.
//
// Unlike orb3_rgbd_node.cpp (IMU_RGBD), this node:
//   - does NOT require an IMU topic
//   - reads the settings YAML directly (no template / no __TBC_BLOCK__)
//   - is simpler and faster to bring up for debugging visual tracking.
// GL/GLEW must be included BEFORE any Pangolin or ORB-SLAM3 GL headers.
// Pangolin (EGL+GLAD build) relies on GLEW for extension declarations.
#include <GL/glew.h>

#include <memory>
#include <string>
#include <cmath>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <ctime>
#include <atomic>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <System.h>
#include <KeyFrame.h>
#include <MapPoint.h>
#include <algorithm>
#include <deque>
#include <mutex>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <sophus/se3.hpp>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace
{
const char kBase64Table[] =
  "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

std::string base64Encode(const std::vector<uint8_t> & data)
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

std::string expandUserPath(const std::string & path)
{
  if (path == "~" || path.rfind("~/", 0) == 0) {
    const char * home = std::getenv("HOME");
    if (home != nullptr) {
      return std::string(home) + path.substr(1);
    }
  }
  return path;
}

std::string timestampString()
{
  const auto now = std::chrono::system_clock::now();
  const std::time_t t = std::chrono::system_clock::to_time_t(now);
  std::tm tm{};
  localtime_r(&t, &tm);
  std::ostringstream out;
  out << std::put_time(&tm, "%Y%m%d_%H%M%S");
  return out.str();
}

std::string replaceYamlStringKey(
  std::string text,
  const std::string & key,
  const std::string & value)
{
  std::istringstream in(text);
  std::ostringstream out;
  std::string line;
  bool replaced = false;
  while (std::getline(in, line)) {
    if (line.rfind(key + ":", 0) == 0) {
      out << key << ": \"" << value << "\"\n";
      replaced = true;
    } else {
      out << line << "\n";
    }
  }
  if (!replaced) {
    out << key << ": \"" << value << "\"\n";
  }
  return out.str();
}

std::string replaceYamlNumberKey(
  std::string text,
  const std::string & key,
  const std::string & value)
{
  std::istringstream in(text);
  std::ostringstream out;
  std::string line;
  bool replaced = false;
  while (std::getline(in, line)) {
    if (line.rfind(key + ":", 0) == 0) {
      out << key << ": " << value << "\n";
      replaced = true;
    } else {
      out << line << "\n";
    }
  }
  if (!replaced) {
    out << key << ": " << value << "\n";
  }
  return out.str();
}
}  // namespace

class Orb3RgbdNode : public rclcpp::Node
{
public:
  Orb3RgbdNode()
  : Node("orb3_rgbd_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    vocab_file_ = declare_parameter<std::string>(
      "vocabulary_file",
      std::string(std::getenv("HOME") ? std::getenv("HOME") : "") + "/Dev/ORB_SLAM3/Vocabulary/ORBvoc.txt");
    settings_file_ = declare_parameter<std::string>(
      "settings_file",
      "REPLACED_BY_LAUNCH");
    load_atlas_file_ = expandUserPath(
      declare_parameter<std::string>("load_atlas_file", ""));
    single_map_mode_ = declare_parameter<bool>("single_map_mode", true);
    vocab_file_ = expandUserPath(vocab_file_);
    settings_file_ = expandUserPath(settings_file_);

    rgb_topic_   = declare_parameter<std::string>("rgb_topic",   "/camera/color/image_raw");
    depth_topic_ = declare_parameter<std::string>("depth_topic", "/camera/depth/image_raw");
    camera_info_topic_ =
      declare_parameter<std::string>("camera_info_topic", "/camera/color/camera_info");

    save_outputs_ = declare_parameter<bool>("save_outputs", true);
    save_atlas_ = declare_parameter<bool>("save_atlas", true);
    output_dir_ = expandUserPath(declare_parameter<std::string>("output_dir", ""));
    unique_output_dir_ = declare_parameter<bool>("unique_output_dir", true);
    if (output_dir_.empty()) {
      output_dir_ =
        std::string(std::getenv("HOME") ? std::getenv("HOME") : ".") +
        "/agv_mpc/maps/sessions/orb_rgbd_" + timestampString() + "/orb";
    } else if (unique_output_dir_) {
      output_dir_ = (std::filesystem::path(output_dir_) / timestampString()).string();
    }
    keypoint_log_hz_ = declare_parameter<double>("keypoint_log_hz", 1.0);
    save_keypoint_images_ = declare_parameter<bool>("save_keypoint_images", false);
    max_saved_keypoints_ = declare_parameter<int>("max_saved_keypoints", 1200);

    world_frame_  = declare_parameter<std::string>("world_frame",  "orb_map");
    odom_frame_   = declare_parameter<std::string>("odom_frame",   "odom");
    base_frame_   = declare_parameter<std::string>("base_frame",   "Base_footprint");
    camera_frame_ = declare_parameter<std::string>("camera_frame", "camera_link_optical");

    use_viewer_   = declare_parameter<bool>("use_viewer",   false);
    publish_tf_   = declare_parameter<bool>("publish_tf",   false);
    tf_publish_mode_ = declare_parameter<std::string>("tf_publish_mode", "world_to_base");
    force_planar_output_ = declare_parameter<bool>("force_planar_output", true);

    path_topic_        = declare_parameter<std::string>("path_topic",        "/orbslam3/path");
    pose_topic_        = declare_parameter<std::string>("pose_topic",        "/orbslam3/pose");
    camera_pose_topic_ = declare_parameter<std::string>("camera_pose_topic", "/orbslam3/camera_pose");
    raw_camera_pose_topic_ = declare_parameter<std::string>(
      "raw_camera_pose_topic", "/orbslam3/pose_raw_camera");
    base_pose_topic_ = declare_parameter<std::string>("base_pose_topic", "/orbslam3/pose_base");
    pose_debug_topic_ = declare_parameter<std::string>(
      "pose_debug_topic", "/orbslam3/pose_debug");
    publish_raw_camera_pose_ = declare_parameter<bool>("publish_raw_camera_pose", true);
    publish_base_pose_ = declare_parameter<bool>("publish_base_pose", true);
    publish_pose_debug_ = declare_parameter<bool>("publish_pose_debug", true);
    map_points_topic_  = declare_parameter<std::string>("map_points_topic",  "/orbslam3/map_points");
    tracked_points_topic_ =
      declare_parameter<std::string>("tracked_points_topic", "/orbslam3/tracked_points");
    rviz_markers_topic_ =
      declare_parameter<std::string>("rviz_markers_topic", "/orbslam3/rviz/markers");
    rviz_keyframes_topic_ =
      declare_parameter<std::string>("rviz_keyframes_topic", "/orbslam3/rviz/keyframes");
    tracking_state_topic_ =
      declare_parameter<std::string>("tracking_state_topic", "/orbslam3/tracking_state");
    tracking_status_topic_ =
      declare_parameter<std::string>("tracking_status_topic", "/orbslam3/tracking_status");
    tracking_preview_topic_ =
      declare_parameter<std::string>("tracking_preview_topic", "/agv/orb_tracking_preview");

    sync_queue_size_  = declare_parameter<int>("sync_queue_size", 10);
    max_rgb_depth_dt_ = declare_parameter<double>("max_rgb_depth_dt", 0.20);
    target_tracking_hz_ = declare_parameter<double>("target_tracking_hz", 10.0);
    max_track_rgbd_sec_ = declare_parameter<double>("max_track_rgbd_sec", 0.0);
    image_callback_watchdog_sec_ =
      declare_parameter<double>("image_callback_watchdog_sec", 8.0);
    map_points_publish_hz_ = declare_parameter<double>("map_points_publish_hz", 0.0);
    graph_publish_hz_ = declare_parameter<double>("graph_publish_hz", 0.0);
    max_path_length_ = declare_parameter<int>("max_path_length", 2000);
    covisibility_min_weight_ = declare_parameter<int>("covisibility_min_weight", 100);
    keyframe_frustum_size_ = declare_parameter<double>("keyframe_frustum_size", 0.08);
    auto_localization_map_points_ = declare_parameter<int>("auto_localization_map_points", 0);
    localization_nfeatures_ = declare_parameter<int>("localization_nfeatures", 800);
    localization_nlevels_ = declare_parameter<int>("localization_nlevels", 6);
    localization_scale_factor_ = declare_parameter<double>("localization_scale_factor", 1.2);
    max_published_map_points_ = declare_parameter<int>("max_published_map_points", 12000);
    max_published_tracked_points_ =
      declare_parameter<int>("max_published_tracked_points", 1500);
    publish_tracking_preview_ = declare_parameter<bool>("publish_tracking_preview", true);
    tracking_preview_width_ = declare_parameter<int>("tracking_preview_width", 320);
    tracking_preview_hz_ = declare_parameter<double>("tracking_preview_hz", 0.5);

    if (save_outputs_) {
      prepareOutputDirectory();
    }

    RCLCPP_INFO(get_logger(), "Starting ORB-SLAM3 RGB-D only node");
    RCLCPP_INFO(get_logger(), "vocabulary_file: %s", vocab_file_.c_str());
    RCLCPP_INFO(get_logger(), "settings_file:   %s", settings_file_.c_str());
    RCLCPP_INFO(get_logger(), "load_atlas_file: %s", load_atlas_file_.empty() ? "-" : load_atlas_file_.c_str());
    RCLCPP_INFO(get_logger(), "single_map_mode: %s", single_map_mode_ ? "true" : "false");
    RCLCPP_INFO(get_logger(), "output_dir:      %s", output_dir_.c_str());
    RCLCPP_INFO(get_logger(), "rgb_topic:       %s", rgb_topic_.c_str());
    RCLCPP_INFO(get_logger(), "depth_topic:     %s", depth_topic_.c_str());
    RCLCPP_INFO(get_logger(), "camera_info:     %s", camera_info_topic_.c_str());
    RCLCPP_INFO(get_logger(), "world_frame:     %s", world_frame_.c_str());
    RCLCPP_INFO(get_logger(), "odom_frame:      %s", odom_frame_.c_str());
    RCLCPP_INFO(get_logger(), "base_frame:      %s", base_frame_.c_str());
    RCLCPP_INFO(get_logger(), "camera_frame:    %s", camera_frame_.c_str());
    RCLCPP_INFO(get_logger(), "use_viewer:      %s", use_viewer_ ? "true" : "false");
    RCLCPP_INFO(get_logger(), "publish_tf:      %s", publish_tf_ ? "true" : "false");
    RCLCPP_INFO(get_logger(), "tf_publish_mode: %s", tf_publish_mode_.c_str());
    RCLCPP_INFO(get_logger(), "force_planar_output: %s", force_planar_output_ ? "true" : "false");
    RCLCPP_INFO(get_logger(), "target_tracking_hz: %.1f", target_tracking_hz_);
    RCLCPP_INFO(get_logger(), "max_track_rgbd_sec: %.1f", max_track_rgbd_sec_);
    RCLCPP_INFO(get_logger(), "image_callback_watchdog_sec: %.1f", image_callback_watchdog_sec_);
    RCLCPP_INFO(get_logger(), "map_points_publish_hz: %.1f", map_points_publish_hz_);
    RCLCPP_INFO(get_logger(), "graph_publish_hz: %.1f", graph_publish_hz_);
    RCLCPP_INFO(get_logger(), "auto_localization_map_points: %d", auto_localization_map_points_);
    if (!load_atlas_file_.empty()) {
      RCLCPP_INFO(
        get_logger(),
        "localization extractor override: nFeatures=%d scaleFactor=%.2f nLevels=%d",
        localization_nfeatures_, localization_scale_factor_, localization_nlevels_);
    }
    RCLCPP_INFO(get_logger(), "max_published_map_points: %d", max_published_map_points_);
    RCLCPP_INFO(
      get_logger(), "tracking_preview: %s topic=%s width=%d hz=%.1f",
      publish_tracking_preview_ ? "true" : "false",
      tracking_preview_topic_.c_str(), tracking_preview_width_, tracking_preview_hz_);


    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    path_msg_.header.frame_id = world_frame_;
    pose_pub_        = create_publisher<geometry_msgs::msg::PoseStamped>(pose_topic_, 10);
    camera_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(camera_pose_topic_, 10);
    raw_camera_pose_pub_ =
      create_publisher<geometry_msgs::msg::PoseStamped>(raw_camera_pose_topic_, 10);
    base_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(base_pose_topic_, 10);
    pose_debug_pub_ = create_publisher<std_msgs::msg::String>(pose_debug_topic_, 10);
    path_pub_        = create_publisher<nav_msgs::msg::Path>(path_topic_, 10);
    // Visualization clouds may be dropped. They must never block pose tracking
    // when WebSocket/RViz consumers are slow.
    const auto cloud_qos =
      rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    mappoints_pub_ =
      create_publisher<sensor_msgs::msg::PointCloud2>(map_points_topic_, cloud_qos);
    tracked_points_pub_ =
      create_publisher<sensor_msgs::msg::PointCloud2>(tracked_points_topic_, cloud_qos);
    rviz_markers_pub_ =
      create_publisher<visualization_msgs::msg::MarkerArray>(rviz_markers_topic_, 10);
    rviz_keyframes_pub_ =
      create_publisher<geometry_msgs::msg::PoseArray>(rviz_keyframes_topic_, 10);
    tracking_state_pub_ = create_publisher<std_msgs::msg::Int32>(tracking_state_topic_, 10);
    tracking_status_pub_ = create_publisher<std_msgs::msg::String>(tracking_status_topic_, 10);
    // Camera preview is diagnostic-only. Never let a slow WebSocket client
    // back-pressure the RGB-D tracking callback.
    const auto preview_qos =
      rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    tracking_preview_pub_ =
      create_publisher<std_msgs::msg::String>(tracking_preview_topic_, preview_qos);
    tracking_heartbeat_timer_ = create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&Orb3RgbdNode::publishTrackingHeartbeat, this));
    tracking_watchdog_timer_ = create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&Orb3RgbdNode::checkTrackingWatchdog, this));

    const bool needs_runtime_settings =
      save_outputs_ || save_atlas_ || !load_atlas_file_.empty();
    if (needs_runtime_settings) {
      std::filesystem::create_directories(output_dir_);
      settings_file_ = makeRuntimeSettingsFile(settings_file_);
      RCLCPP_INFO(get_logger(), "runtime_settings_file: %s", settings_file_.c_str());
      if (!load_atlas_file_.empty()) {
        RCLCPP_INFO(get_logger(), "runtime LoadAtlas stem was injected from: %s", load_atlas_file_.c_str());
      }
    }

    // Build SLAM system immediately — no IMU bootstrap required.
    try {
      if (single_map_mode_ && !load_atlas_file_.empty()) {
        ::setenv("ORB3_SINGLE_MAP_LOCK", "1", 1);
      }
      slam_ = std::make_unique<ORB_SLAM3::System>(
        vocab_file_, settings_file_, ORB_SLAM3::System::RGBD, use_viewer_);
      RCLCPP_INFO(get_logger(), "ORB-SLAM3 RGB-D initialized");
      if (single_map_mode_ && !load_atlas_file_.empty()) {
        const bool selected_loaded_map = slam_->ForceBestMapAsActive();
        RCLCPP_WARN(
          get_logger(),
          "ORB atlas loaded: maps=%d active_map_id=%lu keyframes=%lu mappoints=%lu selected_loaded_map=%s",
          slam_->GetAtlasMapCount(),
          slam_->GetCurrentMapId(),
          slam_->GetCurrentMapKeyFrames(),
          slam_->GetCurrentMapPoints(),
          selected_loaded_map ? "true" : "false");
        // Activate localization before the first RGB-D frame. ORB-SLAM3 normally
        // defers the LocalMapping stop to TrackRGBD(); doing it here avoids a
        // first-frame stall that looks like camera/pose freeze on Jetson.
        const bool localization_ready = slam_->ActivateLocalizationModeBlocking(2.0);
        localization_mode_active_ = true;
        RCLCPP_WARN(
          get_logger(),
          "Loaded ORB atlas in single_map_mode; localization-only is active so ORB-SLAM3 will not extend/switch to a new active map. local_mapping_stopped=%s",
          localization_ready ? "true" : "false");
      }
    } catch (const std::exception & e) {
      RCLCPP_FATAL(get_logger(), "Failed to initialize ORB-SLAM3 RGB-D: %s", e.what());
      throw;
    }

    // Astra camera publishes RELIABLE on this robot. Match that QoS so DDS does
    // not intermittently starve RGB/depth callbacks during ORB localization.
    auto image_qos = rclcpp::QoS(rclcpp::KeepLast(static_cast<size_t>(
      std::max(2, sync_queue_size_)))).reliable().durability_volatile();
    sensor_callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rgbd_processing_callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions sensor_sub_options;
    sensor_sub_options.callback_group = sensor_callback_group_;
    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_, image_qos,
      std::bind(&Orb3RgbdNode::cameraInfoCallback, this, std::placeholders::_1),
      sensor_sub_options);
    RCLCPP_INFO(get_logger(), "Camera-info subscription ready");

    rgb_sub_ = create_subscription<sensor_msgs::msg::Image>(
      rgb_topic_, image_qos,
      std::bind(&Orb3RgbdNode::rgbCallback, this, std::placeholders::_1),
      sensor_sub_options);
    RCLCPP_INFO(get_logger(), "RGB subscription ready");
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
      depth_topic_, image_qos,
      std::bind(&Orb3RgbdNode::depthCallback, this, std::placeholders::_1),
      sensor_sub_options);
    RCLCPP_INFO(get_logger(), "Depth subscription ready");
    // Keep DDS image callbacks lightweight. ORB TrackRGBD and map
    // visualization run from this timer so a slow tracking/relocalization pass
    // cannot starve camera subscription callbacks.
    const double process_poll_hz = std::max(30.0, target_tracking_hz_ * 4.0);
    rgbd_processing_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / process_poll_hz)),
      std::bind(&Orb3RgbdNode::processQueuedRgbd, this),
      rgbd_processing_callback_group_);
    RCLCPP_INFO(
      get_logger(), "RGB-D processing timer ready at %.1f Hz", process_poll_hz);
    if (publish_tracking_preview_ && tracking_preview_hz_ > 0.0) {
      tracking_preview_callback_group_ =
        create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      tracking_preview_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(1.0 / tracking_preview_hz_)),
        std::bind(&Orb3RgbdNode::publishLatestRgbPreview, this),
        tracking_preview_callback_group_);
    }
    RCLCPP_INFO(get_logger(), "ORB RGB-D subscriptions and timers are ready");
  }

  ~Orb3RgbdNode() override
  {
    if (slam_) {
      saveSessionOutputs();
    }
  }

private:
  void prepareOutputDirectory()
  {
    std::filesystem::create_directories(output_dir_);
    if (save_keypoint_images_) {
      std::filesystem::create_directories(std::filesystem::path(output_dir_) / "keypoint_images");
    }

    metrics_csv_.open(std::filesystem::path(output_dir_) / "tracking_metrics.csv");
    metrics_csv_ <<
      "wall_time,stamp,rgb_depth_dt,valid_depth_ratio,tracking_state,tracking_state_name,"
      "track_rgbd_s,pose_x,pose_y,pose_z,identity,keyframes,map_points,tracked_keypoints,tracked_mappoints\n";

    keypoints_csv_.open(std::filesystem::path(output_dir_) / "keypoints.csv");
    keypoints_csv_ <<
      "stamp,idx,x,y,size,angle,response,octave,has_mappoint,image_file\n";
  }

  std::string makeRuntimeSettingsFile(const std::string & source_settings)
  {
    std::ifstream in(source_settings);
    if (!in) {
      RCLCPP_WARN(get_logger(), "Cannot open settings file for runtime copy: %s", source_settings.c_str());
      return source_settings;
    }
    std::ostringstream buffer;
    buffer << in.rdbuf();

    std::string atlas_stem;
    if (save_atlas_) {
      std::filesystem::path atlas_stem_path =
        std::filesystem::absolute(std::filesystem::path(output_dir_) / "orb_atlas");
      atlas_stem = atlas_stem_path.string();
    }
    std::string text = replaceYamlStringKey(
      buffer.str(), "System.SaveAtlasToFile", atlas_stem);
    std::string load_atlas_stem = load_atlas_file_;
    if (!load_atlas_stem.empty() && load_atlas_stem.size() > 4 &&
        load_atlas_stem.substr(load_atlas_stem.size() - 4) == ".osa") {
      load_atlas_stem = load_atlas_stem.substr(0, load_atlas_stem.size() - 4);
    }
    text = replaceYamlStringKey(text, "System.LoadAtlasFromFile", load_atlas_stem);
    if (!load_atlas_stem.empty()) {
      text = replaceYamlNumberKey(
        text, "ORBextractor.nFeatures",
        std::to_string(std::max(300, localization_nfeatures_)));
      {
        std::ostringstream value;
        value << std::fixed << std::setprecision(2)
              << std::max(1.01, localization_scale_factor_);
        text = replaceYamlNumberKey(text, "ORBextractor.scaleFactor", value.str());
      }
      text = replaceYamlNumberKey(
        text, "ORBextractor.nLevels",
        std::to_string(std::max(4, localization_nlevels_)));
    }

    const std::string runtime_file =
      (std::filesystem::path(output_dir_) / "orb3_rgbd_runtime.yaml").string();
    std::ofstream out(runtime_file);
    out << text;
    return runtime_file;
  }

  void saveSessionOutputs()
  {
    if (outputs_saved_) {
      return;
    }
    outputs_saved_ = true;

    try {
      if (metrics_csv_) metrics_csv_.flush();
      if (keypoints_csv_) keypoints_csv_.flush();

      if (save_outputs_) {
        RCLCPP_INFO(get_logger(), "Saving ORB outputs to %s", output_dir_.c_str());
        exportMapPointsCsv();
        exportKeyFramesCsv();
        writeSummaryFile();
        if (metrics_csv_) metrics_csv_.flush();
        if (keypoints_csv_) keypoints_csv_.flush();
        RCLCPP_INFO(get_logger(), "Saved ORB CSV outputs before Shutdown: %s", output_dir_.c_str());
      }

      // If save_atlas is enabled, Shutdown() also writes output_dir/orb_atlas.osa
      // after the CSV exports are already safe.
      slam_->Shutdown();

      if (!save_outputs_) {
        return;
      }
      RCLCPP_INFO(get_logger(), "ORB output save complete: %s", output_dir_.c_str());
    } catch (const std::exception & e) {
      RCLCPP_WARN(get_logger(), "Failed while saving ORB outputs: %s", e.what());
    } catch (...) {
      RCLCPP_WARN(get_logger(), "Failed while saving ORB outputs");
    }
  }

  void exportMapPointsCsv()
  {
    std::ofstream out(std::filesystem::path(output_dir_) / "map_points.csv");
    out << "id,x,y,z\n";
    for (auto * mp : slam_->GetAllMapPoints()) {
      if (mp == nullptr) continue;
      try {
        if (mp->isBad()) continue;
        const Eigen::Vector3f p = mp->GetWorldPos();
        if (!p.allFinite()) continue;
        out << mp->mnId << "," << p.x() << "," << p.y() << "," << p.z() << "\n";
      } catch (...) {
        continue;
      }
    }
  }

  void exportKeyFramesCsv()
  {
    std::vector<ORB_SLAM3::KeyFrame*> keyframes = slam_->GetAllKeyFrames();
    keyframes.erase(
      std::remove_if(
        keyframes.begin(), keyframes.end(),
        [](ORB_SLAM3::KeyFrame * kf) { return kf == nullptr || kf->isBad(); }),
      keyframes.end());
    std::sort(
      keyframes.begin(), keyframes.end(),
      [](ORB_SLAM3::KeyFrame * a, ORB_SLAM3::KeyFrame * b) { return a->mnId < b->mnId; });

    std::ofstream out(std::filesystem::path(output_dir_) / "keyframes.csv");
    out << "id,timestamp,x,y,z,qx,qy,qz,qw\n";
    for (auto * kf : keyframes) {
      try {
        const tf2::Transform Twc = TwcToTf(kf->GetPoseInverse());
        const auto q = Twc.getRotation();
        const auto t = Twc.getOrigin();
        out << kf->mnId << "," << std::setprecision(12) << kf->mTimeStamp << ","
            << t.x() << "," << t.y() << "," << t.z() << ","
            << q.x() << "," << q.y() << "," << q.z() << "," << q.w() << "\n";
      } catch (...) {
        continue;
      }
    }
  }

  void writeSummaryFile()
  {
    const auto keyframes = slam_->GetAllKeyFrames();
    const auto map_points = slam_->GetAllMapPoints();
    std::ofstream out(std::filesystem::path(output_dir_) / "summary.txt");
    out << "output_dir: " << output_dir_ << "\n";
    out << "settings_file: " << settings_file_ << "\n";
    out << "vocabulary_file: " << vocab_file_ << "\n";
    out << "keyframes: " << keyframes.size() << "\n";
    out << "map_points: " << map_points.size() << "\n";
    out << "max_rgb_depth_dt: " << max_rgb_depth_dt_ << "\n";
    out << "target_tracking_hz: " << target_tracking_hz_ << "\n";
    out << "keypoint_log_hz: " << keypoint_log_hz_ << "\n";
  }

  void logTrackingMetrics(
    double stamp,
    double rgb_depth_dt,
    float valid_depth_ratio,
    int tracking_state,
    double track_elapsed,
    double px,
    double py,
    double pz,
    bool identity)
  {
    if (!save_outputs_ || !metrics_csv_) {
      return;
    }

    size_t keyframe_count = 0;
    size_t map_point_count = 0;
    size_t tracked_keypoint_count = 0;
    size_t tracked_mappoint_count = 0;
    try {
      keyframe_count = slam_->GetAllKeyFrames().size();
      map_point_count = slam_->GetAllMapPoints().size();
      auto tracked_kps = slam_->GetTrackedKeyPointsUn();
      auto tracked_mps = slam_->GetTrackedMapPoints();
      tracked_keypoint_count = tracked_kps.size();
      for (auto * mp : tracked_mps) {
        if (mp != nullptr) ++tracked_mappoint_count;
      }
    } catch (...) {}

    metrics_csv_ << std::fixed << std::setprecision(6)
      << this->now().seconds() << ","
      << stamp << ","
      << rgb_depth_dt << ","
      << valid_depth_ratio << ","
      << tracking_state << ","
      << trackingStateName(tracking_state) << ","
      << track_elapsed << ","
      << px << "," << py << "," << pz << ","
      << (identity ? 1 : 0) << ","
      << keyframe_count << ","
      << map_point_count << ","
      << tracked_keypoint_count << ","
      << tracked_mappoint_count << "\n";
  }

  void logKeypointsSnapshot(
    const cv::Mat & rgb,
    double stamp)
  {
    if (!save_outputs_ || keypoint_log_hz_ <= 0.0 || !keypoints_csv_) {
      return;
    }

    const double now_wall = this->now().seconds();
    if (last_keypoint_log_stamp_ > 0.0 &&
        (now_wall - last_keypoint_log_stamp_) < (1.0 / keypoint_log_hz_)) {
      return;
    }
    last_keypoint_log_stamp_ = now_wall;

    std::vector<cv::KeyPoint> keypoints;
    std::vector<ORB_SLAM3::MapPoint*> tracked_mappoints;
    try {
      keypoints = slam_->GetTrackedKeyPointsUn();
      tracked_mappoints = slam_->GetTrackedMapPoints();
    } catch (...) {
      return;
    }

    std::string image_file;
    if (save_keypoint_images_) {
      cv::Mat overlay;
      cv::drawKeypoints(rgb, keypoints, overlay, cv::Scalar(0, 255, 0));
      std::ostringstream name;
      name << "keypoint_images/frame_" << std::fixed << std::setprecision(3) << stamp << ".jpg";
      image_file = name.str();
      const auto full_path = std::filesystem::path(output_dir_) / image_file;
      try {
        cv::imwrite(full_path.string(), overlay);
      } catch (...) {
        image_file.clear();
      }
    }

    const size_t limit =
      max_saved_keypoints_ > 0 ?
      std::min(keypoints.size(), static_cast<size_t>(max_saved_keypoints_)) :
      keypoints.size();
    for (size_t i = 0; i < limit; ++i) {
      const auto & kp = keypoints[i];
      const bool has_mp =
        i < tracked_mappoints.size() && tracked_mappoints[i] != nullptr;
      keypoints_csv_ << std::fixed << std::setprecision(6)
        << stamp << ","
        << i << ","
        << kp.pt.x << ","
        << kp.pt.y << ","
        << kp.size << ","
        << kp.angle << ","
        << kp.response << ","
        << kp.octave << ","
        << (has_mp ? 1 : 0) << ","
        << image_file << "\n";
    }
  }

  void publishTrackingPreview(
    const cv::Mat & rgb,
    double stamp,
    int tracking_state)
  {
    if (!publish_tracking_preview_ || !tracking_preview_pub_) {
      return;
    }

    if (!reserveTrackingPreviewSlot()) {
      return;
    }

    std::vector<cv::KeyPoint> keypoints;
    std::vector<ORB_SLAM3::MapPoint*> tracked_mappoints;
    try {
      keypoints = slam_->GetTrackedKeyPointsUn();
      tracked_mappoints = slam_->GetTrackedMapPoints();
    } catch (...) {
      return;
    }

    const int src_w = rgb.cols;
    const int src_h = rgb.rows;
    if (src_w <= 0 || src_h <= 0) {
      return;
    }

    const int dst_w = std::clamp(tracking_preview_width_, 80, src_w);
    const int dst_h = std::max(1, static_cast<int>(
      (static_cast<double>(src_h) * dst_w) / static_cast<double>(src_w)));
    const double scale_x = static_cast<double>(dst_w) / static_cast<double>(src_w);
    const double scale_y = static_cast<double>(dst_h) / static_cast<double>(src_h);

    cv::Mat overlay;
    cv::resize(rgb, overlay, cv::Size(dst_w, dst_h), 0.0, 0.0, cv::INTER_AREA);

    size_t tracked_mappoint_count = 0;
    const size_t limit = std::min(keypoints.size(), tracked_mappoints.size());
    for (size_t i = 0; i < keypoints.size(); ++i) {
      const auto & kp = keypoints[i];
      const cv::Point center(
        static_cast<int>(std::round(kp.pt.x * scale_x)),
        static_cast<int>(std::round(kp.pt.y * scale_y)));
      if (center.x < 0 || center.y < 0 || center.x >= dst_w || center.y >= dst_h) {
        continue;
      }

      const bool has_mappoint =
        i < limit && tracked_mappoints[i] != nullptr;
      if (has_mappoint) {
        ++tracked_mappoint_count;
        cv::circle(overlay, center, 3, cv::Scalar(0, 0, 255), -1, cv::LINE_AA);
        cv::circle(overlay, center, 5, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
      } else {
        cv::circle(overlay, center, 2, cv::Scalar(0, 220, 255), 1, cv::LINE_AA);
      }
    }

    std::ostringstream label;
    label << "ORB " << trackingStateName(tracking_state)
          << "  KP " << keypoints.size()
          << "  MP " << tracked_mappoint_count;
    cv::rectangle(overlay, cv::Point(0, 0), cv::Point(dst_w, 24),
      cv::Scalar(0, 0, 0), cv::FILLED);
    cv::putText(
      overlay, label.str(), cv::Point(8, 17), cv::FONT_HERSHEY_SIMPLEX,
      0.45, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);

    std::vector<uint8_t> encoded;
    const std::vector<int> jpeg_params = {
      cv::IMWRITE_JPEG_QUALITY, 70,
      cv::IMWRITE_JPEG_OPTIMIZE, 1};
    if (!cv::imencode(".jpg", overlay, encoded, jpeg_params)) {
      return;
    }

    std::ostringstream json;
    json << "{\"width\":" << dst_w
         << ",\"height\":" << dst_h
         << ",\"step\":0"
         << ",\"encoding\":\"jpeg\""
         << ",\"source\":\"orb_slam3_tracking\""
         << ",\"stamp\":" << std::fixed << std::setprecision(6) << stamp
         << ",\"tracking_state\":" << tracking_state
         << ",\"tracking_status\":\"" << trackingStateName(tracking_state) << "\""
         << ",\"keypoints\":" << keypoints.size()
         << ",\"tracked_mappoints\":" << tracked_mappoint_count
         << ",\"data\":\"" << base64Encode(encoded) << "\"}";

    std_msgs::msg::String out;
    out.data = json.str();
    tracking_preview_pub_->publish(out);
  }

  void publishRawTrackingPreview(
    const cv::Mat & rgb,
    double stamp,
    const std::string & status)
  {
    if (!publish_tracking_preview_ || !tracking_preview_pub_) {
      return;
    }

    if (!reserveTrackingPreviewSlot()) {
      return;
    }

    const int src_w = rgb.cols;
    const int src_h = rgb.rows;
    if (src_w <= 0 || src_h <= 0) {
      return;
    }

    const int dst_w = std::clamp(tracking_preview_width_, 80, src_w);
    const int dst_h = std::max(1, static_cast<int>(
      (static_cast<double>(src_h) * dst_w) / static_cast<double>(src_w)));

    cv::Mat overlay;
    cv::resize(rgb, overlay, cv::Size(dst_w, dst_h), 0.0, 0.0, cv::INTER_AREA);

    std::ostringstream label;
    label << "ORB TRACKING CAMERA  " << status;
    cv::rectangle(overlay, cv::Point(0, 0), cv::Point(dst_w, 24),
      cv::Scalar(0, 0, 0), cv::FILLED);
    cv::putText(
      overlay, label.str(), cv::Point(8, 17), cv::FONT_HERSHEY_SIMPLEX,
      0.45, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);

    std::vector<uint8_t> encoded;
    const std::vector<int> jpeg_params = {
      cv::IMWRITE_JPEG_QUALITY, 70,
      cv::IMWRITE_JPEG_OPTIMIZE, 1};
    if (!cv::imencode(".jpg", overlay, encoded, jpeg_params)) {
      return;
    }

    std::ostringstream json;
    json << "{\"width\":" << dst_w
         << ",\"height\":" << dst_h
         << ",\"step\":0"
         << ",\"encoding\":\"jpeg\""
         << ",\"source\":\"orb_slam3_tracking_camera\""
         << ",\"stamp\":" << std::fixed << std::setprecision(6) << stamp
         << ",\"tracking_state\":-2"
         << ",\"tracking_status\":\"" << status << "\""
         << ",\"keypoints\":0"
         << ",\"tracked_mappoints\":0"
         << ",\"data\":\"" << base64Encode(encoded) << "\"}";

    std_msgs::msg::String out;
    out.data = json.str();
    tracking_preview_pub_->publish(out);
  }

  bool trackingPreviewDue()
  {
    std::lock_guard<std::mutex> lock(tracking_preview_rate_mutex_);
    if (tracking_preview_hz_ <= 0.0 || last_tracking_preview_stamp_ <= 0.0) {
      return true;
    }
    const double period = 1.0 / tracking_preview_hz_;
    const double scheduling_tolerance = std::min(0.05, period * 0.25);
    return this->now().seconds() + scheduling_tolerance >=
      last_tracking_preview_stamp_ + period;
  }

  bool reserveTrackingPreviewSlot()
  {
    std::lock_guard<std::mutex> lock(tracking_preview_rate_mutex_);
    const double now_wall = this->now().seconds();
    if (tracking_preview_hz_ > 0.0 && last_tracking_preview_stamp_ > 0.0) {
      const double period = 1.0 / tracking_preview_hz_;
      const double scheduling_tolerance = std::min(0.05, period * 0.25);
      const double next_deadline = last_tracking_preview_stamp_ + period;
      if (now_wall + scheduling_tolerance < next_deadline) {
        return false;
      }
      // Advance on the nominal schedule so callback jitter does not slowly
      // reduce a requested 5 Hz preview to roughly 4 Hz.
      last_tracking_preview_stamp_ = std::max(next_deadline, now_wall);
      return true;
    }
    last_tracking_preview_stamp_ = now_wall;
    return true;
  }

  static double stampToSec(const builtin_interfaces::msg::Time & stamp)
  {
    return static_cast<double>(stamp.sec)
         + static_cast<double>(stamp.nanosec) * 1e-9;
  }

  static geometry_msgs::msg::Pose toPoseMsg(const tf2::Transform & t)
  {
    geometry_msgs::msg::Pose pose;
    pose.position.x = t.getOrigin().x();
    pose.position.y = t.getOrigin().y();
    pose.position.z = t.getOrigin().z();
    pose.orientation = tf2::toMsg(t.getRotation());
    return pose;
  }

  void publishTrackedMapPoints(const builtin_interfaces::msg::Time & stamp)
  {
    std::vector<ORB_SLAM3::MapPoint*> tracked;
    try {
      tracked = slam_->GetTrackedMapPoints();
    } catch (...) {
      return;
    }

    std::vector<Eigen::Vector3f> points;
    points.reserve(tracked.size());
    for (auto * mp : tracked) {
      if (mp == nullptr) continue;
      try {
        if (mp->isBad()) continue;
        const Eigen::Vector3f position = mp->GetWorldPos();
        if (position.allFinite()) points.push_back(position);
      } catch (...) {
        continue;
      }
    }

    if (max_published_tracked_points_ > 0 &&
        points.size() > static_cast<size_t>(max_published_tracked_points_)) {
      const size_t stride = static_cast<size_t>(std::ceil(
        static_cast<double>(points.size()) /
        static_cast<double>(max_published_tracked_points_)));
      std::vector<Eigen::Vector3f> reduced;
      reduced.reserve(max_published_tracked_points_);
      for (size_t i = 0; i < points.size(); i += stride) {
        reduced.push_back(points[i]);
      }
      points.swap(reduced);
    }

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = stamp;
    cloud.header.frame_id = world_frame_;
    cloud.height = 1;
    cloud.width = static_cast<uint32_t>(points.size());
    cloud.is_dense = false;
    cloud.is_bigendian = false;

    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(points.size());

    sensor_msgs::PointCloud2Iterator<float> x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> z(cloud, "z");
    for (const auto & point : points) {
      *x = point.x();
      *y = point.y();
      *z = point.z();
      ++x;
      ++y;
      ++z;
    }
    tracked_points_pub_->publish(cloud);
  }

  void publishFullMapPoints(
    const builtin_interfaces::msg::Time & stamp,
    double frame_time)
  {
    try {
      if (map_points_publish_hz_ <= 0.0 && auto_localization_map_points_ <= 0) {
        return;
      }

      auto map_points = slam_->GetAllMapPoints();
      const size_t current_map_points = map_points.size();
      if (!localization_mode_active_ &&
          auto_localization_map_points_ > 0 &&
          current_map_points >= static_cast<size_t>(auto_localization_map_points_)) {
        try {
          const bool localization_ready = slam_->ActivateLocalizationModeBlocking(2.0);
          localization_mode_active_ = true;
          RCLCPP_WARN(
            get_logger(),
            "Current ORB map reached %zu points; switched ORB-SLAM3 to localization-only to avoid Jetson CPU lockup. local_mapping_stopped=%s",
            current_map_points,
            localization_ready ? "true" : "false");
        } catch (const std::exception & e) {
          RCLCPP_WARN(get_logger(), "Failed to activate localization-only mode: %s", e.what());
        } catch (...) {
          RCLCPP_WARN(get_logger(), "Failed to activate localization-only mode");
        }
      }

      if (map_points_publish_hz_ <= 0.0) {
        return;
      }

      const double min_cloud_dt = 1.0 / map_points_publish_hz_;
      if (last_map_points_publish_stamp_ > 0.0 &&
          (frame_time - last_map_points_publish_stamp_) < min_cloud_dt) {
        return;
      }
      last_map_points_publish_stamp_ = frame_time;

      std::vector<Eigen::Vector3f> pts;
      pts.reserve(map_points.size());
      for (auto * mp : map_points) {
        if (mp == nullptr) continue;
        try {
          if (mp->isBad()) continue;
          const Eigen::Vector3f pos = mp->GetWorldPos();
          if (pos.allFinite()) pts.push_back(pos);
        } catch (...) {
          continue;
        }
      }
      if (max_published_map_points_ > 0 &&
          pts.size() > static_cast<size_t>(max_published_map_points_)) {
        const size_t stride = static_cast<size_t>(std::ceil(
          static_cast<double>(pts.size()) /
          static_cast<double>(max_published_map_points_)));
        std::vector<Eigen::Vector3f> downsampled;
        downsampled.reserve(max_published_map_points_);
        for (size_t i = 0; i < pts.size(); i += stride) {
          downsampled.push_back(pts[i]);
        }
        pts.swap(downsampled);
      }

      if (pts.empty()) {
        return;
      }

      sensor_msgs::msg::PointCloud2 cloud_msg;
      cloud_msg.header.stamp = stamp;
      cloud_msg.header.frame_id = world_frame_;
      cloud_msg.height = 1;
      cloud_msg.is_dense = false;
      cloud_msg.is_bigendian = false;

      sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
      modifier.setPointCloud2FieldsByString(1, "xyz");
      modifier.resize(pts.size());
      cloud_msg.width = static_cast<uint32_t>(pts.size());
      cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width;

      sensor_msgs::PointCloud2Iterator<float> it_x(cloud_msg, "x");
      sensor_msgs::PointCloud2Iterator<float> it_y(cloud_msg, "y");
      sensor_msgs::PointCloud2Iterator<float> it_z(cloud_msg, "z");
      for (const auto & p : pts) {
        *it_x = p.x();
        *it_y = p.y();
        *it_z = p.z();
        ++it_x;
        ++it_y;
        ++it_z;
      }
      mappoints_pub_->publish(cloud_msg);

      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Current ORB map points: %zu published: %zu",
        current_map_points, pts.size());
    } catch (...) {
    }
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

    tf2::Quaternion planar_q;
    planar_q.setRPY(0.0, 0.0, yaw);
    planar_q.normalize();

    pose.position.z = 0.0;
    pose.orientation = tf2::toMsg(planar_q);
  }

  static const char * trackingStateName(int state)
  {
    switch (state) {
      case -1: return "SYSTEM_NOT_READY";
      case 0: return "NO_IMAGES_YET";
      case 1: return "NOT_INITIALIZED";
      case 2: return "OK";
      case 3: return "RECENTLY_LOST";
      case 4: return "LOST";
      case 5: return "OK_KLT";
      default: return "UNKNOWN";
    }
  }

  void publishTrackingStatus(int state)
  {
    latest_tracking_state_.store(state);
    latest_tracking_state_wall_stamp_.store(this->now().seconds());
    if (tracking_state_pub_) {
      std_msgs::msg::Int32 msg;
      msg.data = state;
      tracking_state_pub_->publish(msg);
    }
    if (tracking_status_pub_) {
      std_msgs::msg::String msg;
      msg.data = trackingStateName(state);
      tracking_status_pub_->publish(msg);
    }
  }

  void publishTrackingHeartbeat()
  {
    if (!tracking_state_pub_ && !tracking_status_pub_) {
      return;
    }

    const double now = this->now().seconds();
    const double last_state_stamp = latest_tracking_state_wall_stamp_.load();
    const double last_track_start = last_tracked_wall_stamp_.load();
    const int last_state = latest_tracking_state_.load();

    if (last_state_stamp > 0.0 && (now - last_state_stamp) < 1.5) {
      return;
    }

    const int heartbeat_state = -1;
    if (tracking_state_pub_) {
      std_msgs::msg::Int32 msg;
      msg.data = heartbeat_state;
      tracking_state_pub_->publish(msg);
    }
    if (tracking_status_pub_) {
      std_msgs::msg::String msg;
      std::ostringstream status;
      const bool tracking_busy = tracking_in_progress_.load();
      double rgb_age = -1.0;
      double depth_age = -1.0;
      const double last_rgb_cb = last_rgb_callback_wall_stamp_.load(std::memory_order_relaxed);
      const double last_depth_cb = last_depth_callback_wall_stamp_.load(std::memory_order_relaxed);
      if (last_rgb_cb > 0.0) {
        rgb_age = now - last_rgb_cb;
      }
      if (last_depth_cb > 0.0) {
        depth_age = now - last_depth_cb;
      }
      size_t rgb_queue_size = 0;
      size_t depth_queue_size = 0;
      {
        std::lock_guard<std::mutex> lock(rgbd_queue_mutex_);
        rgb_queue_size = rgb_queue_.size();
        depth_queue_size = depth_queue_.size();
      }
      status << (tracking_busy ? "BUSY_TRACKRGBD" : "STALE_TRACKRGBD");
      status << " rgb=" << rgb_received_count_.load()
             << " depth=" << depth_received_count_.load()
             << " matched=" << rgbd_matched_count_.load()
             << " dt_drop=" << rgbd_dt_drop_count_.load()
             << " processed=" << rgbd_processed_count_.load()
             << " track_calls=" << track_call_count_.load()
             << " q_rgb=" << rgb_queue_size
             << " q_depth=" << depth_queue_size;
      if (rgb_age >= 0.0 || depth_age >= 0.0) {
        status << " cb_age_rgb=" << std::fixed << std::setprecision(1) << rgb_age
               << " cb_age_depth=" << depth_age;
      }
      if (tracking_busy && last_track_start > 0.0) {
        status << " current_track_sec=" << std::fixed << std::setprecision(1)
               << (now - last_track_start);
      }
      if (last_state_stamp > 0.0) {
        status << " last_state=" << trackingStateName(last_state)
               << " stale_sec=" << std::fixed << std::setprecision(1)
               << (now - last_state_stamp);
      } else if (last_track_start > 0.0) {
        status << " tracking_call_sec=" << std::fixed << std::setprecision(1)
               << (now - last_track_start);
      } else {
        status << " no_tracking_result";
      }
      msg.data = status.str();
      tracking_status_pub_->publish(msg);
    }
  }

  void checkTrackingWatchdog()
  {
    const double now = this->now().seconds();
    if (image_callback_watchdog_sec_ > 0.0 && !tracking_in_progress_.load()) {
      const double last_rgb_cb =
        last_rgb_callback_wall_stamp_.load(std::memory_order_relaxed);
      const double last_depth_cb =
        last_depth_callback_wall_stamp_.load(std::memory_order_relaxed);
      if (last_rgb_cb > 0.0 && last_depth_cb > 0.0) {
        const double rgb_age = now - last_rgb_cb;
        const double depth_age = now - last_depth_cb;
        const double max_age = std::max(rgb_age, depth_age);
        if (max_age > image_callback_watchdog_sec_) {
          RCLCPP_ERROR(
            get_logger(),
            "ORB image callbacks stalled: rgb_age=%.1f s depth_age=%.1f s > %.1f s. "
            "Exiting orb3_rgbd_node so launch respawn can recreate DDS subscriptions.",
            rgb_age, depth_age, image_callback_watchdog_sec_);
          if (tracking_status_pub_) {
            std_msgs::msg::String msg;
            std::ostringstream status;
            status << "WATCHDOG_RESTART image_callback_stall rgb_age="
                   << std::fixed << std::setprecision(1) << rgb_age
                   << " depth_age=" << depth_age;
            msg.data = status.str();
            tracking_status_pub_->publish(msg);
          }
          std::_Exit(76);
        }
      }
    }

    if (max_track_rgbd_sec_ <= 0.0 || !tracking_in_progress_.load()) {
      return;
    }
    const double last_track_start = last_tracked_wall_stamp_.load();
    if (last_track_start <= 0.0) {
      return;
    }
    const double stuck_sec = now - last_track_start;
    if (stuck_sec <= max_track_rgbd_sec_) {
      return;
    }

    RCLCPP_ERROR(
      get_logger(),
      "ORB TrackRGBD has been running for %.1f s > %.1f s. "
      "Exiting orb3_rgbd_node so launch respawn can recover localization.",
      stuck_sec, max_track_rgbd_sec_);
    if (tracking_status_pub_) {
      std_msgs::msg::String msg;
      std::ostringstream status;
      status << "WATCHDOG_RESTART TrackRGBD_stuck_sec="
             << std::fixed << std::setprecision(1) << stuck_sec;
      msg.data = status.str();
      tracking_status_pub_->publish(msg);
    }
    std::_Exit(75);
  }

  float computeValidDepthRatio(const cv::Mat & depth) const
  {
    int valid = 0, total = 0;
    for (int r = 0; r < depth.rows; ++r) {
      const float * row = depth.ptr<float>(r);
      for (int c = 0; c < depth.cols; ++c) {
        ++total;
        const float d = row[c];
        if (std::isfinite(d) && d > 0.1f && d < 10.0f) ++valid;
      }
    }
    if (total == 0) return 0.0f;
    return static_cast<float>(valid) / static_cast<float>(total);
  }

  static std_msgs::msg::ColorRGBA color(float r, float g, float b, float a = 1.0f)
  {
    std_msgs::msg::ColorRGBA c;
    c.r = r; c.g = g; c.b = b; c.a = a;
    return c;
  }

  static geometry_msgs::msg::Point pointFromEigen(const Eigen::Vector3f & p)
  {
    geometry_msgs::msg::Point out;
    out.x = static_cast<double>(p.x());
    out.y = static_cast<double>(p.y());
    out.z = static_cast<double>(p.z());
    return out;
  }

  static uint64_t edgeKey(unsigned long a, unsigned long b)
  {
    if (a > b) std::swap(a, b);
    return (static_cast<uint64_t>(a) << 32) ^ static_cast<uint64_t>(b);
  }

  visualization_msgs::msg::Marker makeMarker(
    int id,
    const std::string & ns,
    int type,
    const builtin_interfaces::msg::Time & stamp,
    const std_msgs::msg::ColorRGBA & marker_color,
    double scale) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = world_frame_;
    marker.header.stamp = stamp;
    marker.ns = ns;
    marker.id = id;
    marker.type = type;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = scale;
    marker.scale.y = scale;
    marker.scale.z = scale;
    marker.color = marker_color;
    return marker;
  }

  void appendLine(
    visualization_msgs::msg::Marker & marker,
    const Eigen::Vector3f & a,
    const Eigen::Vector3f & b) const
  {
    marker.points.push_back(pointFromEigen(a));
    marker.points.push_back(pointFromEigen(b));
  }

  void pushMarkerOrDelete(
    visualization_msgs::msg::MarkerArray & array,
    visualization_msgs::msg::Marker & marker) const
  {
    const bool needs_points =
      marker.type == visualization_msgs::msg::Marker::LINE_LIST ||
      marker.type == visualization_msgs::msg::Marker::SPHERE_LIST;
    if (needs_points && marker.points.empty()) {
      marker.action = visualization_msgs::msg::Marker::DELETE;
    }
    array.markers.push_back(marker);
  }

  void appendKeyFrameFrustum(
    visualization_msgs::msg::Marker & marker,
    const Sophus::SE3f & Twc) const
  {
    const float z = static_cast<float>(keyframe_frustum_size_);
    const float x = z * 0.75f;
    const float y = z * 0.55f;

    const Eigen::Vector3f o = Twc * Eigen::Vector3f(0.0f, 0.0f, 0.0f);
    const Eigen::Vector3f c1 = Twc * Eigen::Vector3f( x,  y, z);
    const Eigen::Vector3f c2 = Twc * Eigen::Vector3f( x, -y, z);
    const Eigen::Vector3f c3 = Twc * Eigen::Vector3f(-x, -y, z);
    const Eigen::Vector3f c4 = Twc * Eigen::Vector3f(-x,  y, z);

    appendLine(marker, o, c1);
    appendLine(marker, o, c2);
    appendLine(marker, o, c3);
    appendLine(marker, o, c4);
    appendLine(marker, c1, c2);
    appendLine(marker, c2, c3);
    appendLine(marker, c3, c4);
    appendLine(marker, c4, c1);
  }

  void publishRvizGraph(
    const builtin_interfaces::msg::Time & stamp,
    double frame_time)
  {
    if (graph_publish_hz_ <= 0.0 || !slam_) {
      return;
    }

    const double min_dt = 1.0 / graph_publish_hz_;
    if (last_graph_publish_stamp_ > 0.0 &&
        (frame_time - last_graph_publish_stamp_) < min_dt) {
      return;
    }
    last_graph_publish_stamp_ = frame_time;

    std::vector<ORB_SLAM3::KeyFrame*> keyframes;
    try {
      keyframes = slam_->GetAllKeyFrames();
    } catch (...) {
      return;
    }

    keyframes.erase(
      std::remove_if(
        keyframes.begin(), keyframes.end(),
        [](ORB_SLAM3::KeyFrame * kf) { return kf == nullptr || kf->isBad(); }),
      keyframes.end());

    std::sort(
      keyframes.begin(), keyframes.end(),
      [](ORB_SLAM3::KeyFrame * a, ORB_SLAM3::KeyFrame * b) {
        return a->mnId < b->mnId;
      });

    visualization_msgs::msg::MarkerArray array;
    auto clear = makeMarker(
      0, "clear", visualization_msgs::msg::Marker::LINE_LIST, stamp, color(0, 0, 0, 0), 0.01);
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    array.markers.push_back(clear);

    auto frustums = makeMarker(
      1, "keyframes", visualization_msgs::msg::Marker::LINE_LIST,
      stamp, color(0.15f, 0.55f, 1.0f, 1.0f), 0.01);
    auto centers = makeMarker(
      2, "keyframe_centers", visualization_msgs::msg::Marker::SPHERE_LIST,
      stamp, color(1.0f, 0.95f, 0.15f, 1.0f), 0.035);
    auto keyframe_graph = makeMarker(
      3, "keyframe_graph", visualization_msgs::msg::Marker::LINE_LIST,
      stamp, color(0.95f, 0.95f, 0.95f, 0.8f), 0.006);
    auto covisibility = makeMarker(
      4, "covisibility_graph", visualization_msgs::msg::Marker::LINE_LIST,
      stamp, color(0.15f, 0.9f, 0.15f, 0.7f), 0.004);
    auto spanning = makeMarker(
      5, "spanning_tree", visualization_msgs::msg::Marker::LINE_LIST,
      stamp, color(1.0f, 0.65f, 0.05f, 0.9f), 0.008);
    auto loop_edges = makeMarker(
      6, "loop_edges", visualization_msgs::msg::Marker::LINE_LIST,
      stamp, color(1.0f, 0.05f, 0.95f, 1.0f), 0.012);
    auto inertial = makeMarker(
      7, "inertial_graph", visualization_msgs::msg::Marker::LINE_LIST,
      stamp, color(0.0f, 0.95f, 1.0f, 0.95f), 0.008);

    geometry_msgs::msg::PoseArray keyframe_poses;
    keyframe_poses.header.frame_id = world_frame_;
    keyframe_poses.header.stamp = stamp;

    std::unordered_set<uint64_t> cov_edges;
    std::unordered_set<uint64_t> loop_seen;

    Eigen::Vector3f prev_center;
    bool has_prev = false;

    for (auto * kf : keyframes) {
      Sophus::SE3f Twc;
      try {
        Twc = kf->GetPoseInverse();
      } catch (...) {
        continue;
      }
      const Eigen::Vector3f center = kf->GetCameraCenter();
      if (!center.allFinite()) {
        continue;
      }

      appendKeyFrameFrustum(frustums, Twc);
      centers.points.push_back(pointFromEigen(center));
      keyframe_poses.poses.push_back(toPoseMsg(TwcToTf(Twc)));

      if (has_prev) {
        appendLine(keyframe_graph, prev_center, center);
      }
      prev_center = center;
      has_prev = true;

      try {
        ORB_SLAM3::KeyFrame * parent = kf->GetParent();
        if (parent != nullptr && !parent->isBad()) {
          appendLine(spanning, center, parent->GetCameraCenter());
        }
      } catch (...) {}

      try {
        if (kf->bImu && kf->mPrevKF != nullptr && !kf->mPrevKF->isBad()) {
          appendLine(inertial, center, kf->mPrevKF->GetCameraCenter());
        }
      } catch (...) {}

      try {
        for (auto * other : kf->GetCovisiblesByWeight(covisibility_min_weight_)) {
          if (other == nullptr || other->isBad()) continue;
          const uint64_t key = edgeKey(kf->mnId, other->mnId);
          if (cov_edges.insert(key).second) {
            appendLine(covisibility, center, other->GetCameraCenter());
          }
        }
      } catch (...) {}

      try {
        for (auto * other : kf->GetLoopEdges()) {
          if (other == nullptr || other->isBad()) continue;
          const uint64_t key = edgeKey(kf->mnId, other->mnId);
          if (loop_seen.insert(key).second) {
            appendLine(loop_edges, center, other->GetCameraCenter());
          }
        }
      } catch (...) {}
    }

    pushMarkerOrDelete(array, frustums);
    pushMarkerOrDelete(array, centers);
    pushMarkerOrDelete(array, keyframe_graph);
    pushMarkerOrDelete(array, covisibility);
    pushMarkerOrDelete(array, spanning);
    pushMarkerOrDelete(array, loop_edges);
    pushMarkerOrDelete(array, inertial);
    rviz_markers_pub_->publish(array);
    rviz_keyframes_pub_->publish(keyframe_poses);
  }

  static tf2::Transform TwcToTf(const Sophus::SE3f & Twc)
  {
    const Eigen::Matrix4f m = Twc.matrix();
    tf2::Matrix3x3 r(
      m(0, 0), m(0, 1), m(0, 2),
      m(1, 0), m(1, 1), m(1, 2),
      m(2, 0), m(2, 1), m(2, 2));
    tf2::Quaternion q;
    r.getRotation(q);
    tf2::Transform tf;
    tf.setOrigin(tf2::Vector3(m(0, 3), m(1, 3), m(2, 3)));
    tf.setRotation(q);
    return tf;
  }

  static tf2::Transform orbOpticalWorldToRosMapTransform()
  {
    // ORB-SLAM3 RGB-D uses the camera optical convention for its initial
    // world axes: x=right, y=down, z=forward.  The rest of the AGV stack uses
    // ROS body/map axes: x=forward, y=left, z=up.  Convert the ORB parent
    // frame before applying the camera->base extrinsic, otherwise translation
    // axes and Euler labels look swapped (e.g. ORB roll showing up as yaw).
    //
    // Vector mapping:
    //   ros.x =  orb.z
    //   ros.y = -orb.x
    //   ros.z = -orb.y
    tf2::Matrix3x3 R(
      0.0,  0.0,  1.0,
     -1.0,  0.0,  0.0,
      0.0, -1.0,  0.0);
    tf2::Quaternion q;
    R.getRotation(q);
    q.normalize();
    tf2::Transform T;
    T.setOrigin(tf2::Vector3(0.0, 0.0, 0.0));
    T.setRotation(q);
    return T;
  }

  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info_msg)
  {
    std::lock_guard<std::mutex> lock(latest_camera_info_mutex_);
    latest_camera_info_msg_ = info_msg;
    if (!logged_camera_info_) {
      logged_camera_info_ = true;
      RCLCPP_INFO(
        get_logger(),
        "Astra color camera_info: %ux%u K=[fx=%.3f fy=%.3f cx=%.3f cy=%.3f] D=[%.6f %.6f %.6f %.6f %.6f]",
        info_msg->width, info_msg->height,
        info_msg->k[0], info_msg->k[4], info_msg->k[2], info_msg->k[5],
        info_msg->d.size() > 0 ? info_msg->d[0] : 0.0,
        info_msg->d.size() > 1 ? info_msg->d[1] : 0.0,
        info_msg->d.size() > 2 ? info_msg->d[2] : 0.0,
        info_msg->d.size() > 3 ? info_msg->d[3] : 0.0,
        info_msg->d.size() > 4 ? info_msg->d[4] : 0.0);
    }
  }

  void rgbCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
  {
    last_rgb_callback_wall_stamp_.store(this->now().seconds(), std::memory_order_relaxed);
    // Keep the web camera alive even when Astra depth is delayed or ORB is
    // relocalizing. The diagnostic preview must not depend on an RGB-D match.
    if (publish_tracking_preview_ && tracking_preview_pub_) {
      std::lock_guard<std::mutex> lock(latest_preview_rgb_mutex_);
      latest_preview_rgb_msg_ = msg;
    }
    enqueueRgbd(msg, nullptr);
  }

  void publishLatestRgbPreview()
  {
    sensor_msgs::msg::Image::ConstSharedPtr msg;
    {
      std::lock_guard<std::mutex> lock(latest_preview_rgb_mutex_);
      msg = latest_preview_rgb_msg_;
    }
    if (!msg) {
      return;
    }
    try {
      const auto rgb_cv = cv_bridge::toCvShare(
        msg, sensor_msgs::image_encodings::BGR8);
      publishRawTrackingPreview(
        rgb_cv->image, stampToSec(msg->header.stamp), "LIVE");
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Cannot encode web camera preview: %s", e.what());
    }
  }

  void depthCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
  {
    last_depth_callback_wall_stamp_.store(this->now().seconds(), std::memory_order_relaxed);
    enqueueRgbd(nullptr, msg);
  }

  void enqueueRgbd(
    const sensor_msgs::msg::Image::ConstSharedPtr & rgb_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg)
  {
    std::lock_guard<std::mutex> lock(rgbd_queue_mutex_);
    if (rgb_msg) {
      rgb_queue_.push_back(rgb_msg);
      rgb_received_count_.fetch_add(1, std::memory_order_relaxed);
    }
    if (depth_msg) {
      depth_queue_.push_back(depth_msg);
      depth_received_count_.fetch_add(1, std::memory_order_relaxed);
    }

    const size_t queue_limit = static_cast<size_t>(std::max(2, sync_queue_size_));
    while (rgb_queue_.size() > queue_limit) {
      rgb_queue_.pop_front();
    }
    while (depth_queue_.size() > queue_limit) {
      depth_queue_.pop_front();
    }
  }

  void processQueuedRgbd()
  {
    sensor_msgs::msg::Image::ConstSharedPtr matched_rgb;
    sensor_msgs::msg::Image::ConstSharedPtr matched_depth;

    {
      std::lock_guard<std::mutex> lock(rgbd_queue_mutex_);
      if (rgb_queue_.empty() || depth_queue_.empty()) {
        return;
      }

      double best_dt = std::numeric_limits<double>::max();
      size_t best_rgb = 0;
      size_t best_depth = 0;
      for (size_t i = 0; i < rgb_queue_.size(); ++i) {
        const double tr = stampToSec(rgb_queue_[i]->header.stamp);
        for (size_t j = 0; j < depth_queue_.size(); ++j) {
          const double dt = std::abs(tr - stampToSec(depth_queue_[j]->header.stamp));
          if (dt < best_dt) {
            best_dt = dt;
            best_rgb = i;
            best_depth = j;
          }
        }
      }

      if (best_dt > max_rgb_depth_dt_) {
        rgbd_dt_drop_count_.fetch_add(1, std::memory_order_relaxed);
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "RGB-D sync dropped before tracking: best_dt=%.4f > max=%.4f, rgb_queue=%zu depth_queue=%zu, rgb_count=%ld depth_count=%ld",
          best_dt, max_rgb_depth_dt_, rgb_queue_.size(), depth_queue_.size(),
          rgb_received_count_.load(), depth_received_count_.load());
        const double rgb_stamp = stampToSec(rgb_queue_.front()->header.stamp);
        const double depth_stamp = stampToSec(depth_queue_.front()->header.stamp);
        if (rgb_stamp < depth_stamp) {
          rgb_queue_.pop_front();
        } else {
          depth_queue_.pop_front();
        }
        return;
      }

      matched_rgb = rgb_queue_[best_rgb];
      matched_depth = depth_queue_[best_depth];
      rgb_queue_.erase(rgb_queue_.begin(), rgb_queue_.begin() + best_rgb + 1);
      depth_queue_.erase(depth_queue_.begin(), depth_queue_.begin() + best_depth + 1);
      rgbd_matched_count_.fetch_add(1, std::memory_order_relaxed);
    }

    processRgbd(matched_rgb, matched_depth);
  }

  void processRgbd(
    const sensor_msgs::msg::Image::ConstSharedPtr & rgb_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg)
  {
    const double tr = stampToSec(rgb_msg->header.stamp);
    const double td = stampToSec(depth_msg->header.stamp);
    const double dt = std::abs(tr - td);
    rgbd_processed_count_.fetch_add(1, std::memory_order_relaxed);

    if (dt > 0.5) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Depth frame is stale by %.2f s. Check Astra depth stream or CPU starvation.",
        dt);
    }

    const double now_wall = this->now().seconds();
    const double last_track_start = last_tracked_wall_stamp_.load();
    if (target_tracking_hz_ > 0.0 && last_track_start > 0.0) {
      const double min_dt = 1.0 / target_tracking_hz_;
      if ((now_wall - last_track_start) < min_dt) {
        return;
      }
    }

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "rgb-depth dt = %.4f s", dt);

    if (dt > max_rgb_depth_dt_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Skipping RGB-D pair: dt %.4f > max %.4f", dt, max_rgb_depth_dt_);
      return;
    }

    try {
      auto rgb_cv = cv_bridge::toCvCopy(rgb_msg,
                        sensor_msgs::image_encodings::BGR8);

      // Use BGR8 directly (ORB-SLAM3 expects BGR, Camera.RGB=0 in yaml).
      // .clone() gives contiguous deep copy — prevents double-free from
      // shared cv_bridge internal buffers held by ORB-SLAM3 threads.
      const cv::Mat rgb = rgb_cv->image.clone();

      cv::Mat depth;
      if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1 ||
          depth_msg->encoding == sensor_msgs::image_encodings::MONO16) {
        auto depth_cv = cv_bridge::toCvCopy(depth_msg,
                          sensor_msgs::image_encodings::TYPE_16UC1);
        depth_cv->image.convertTo(depth, CV_32FC1, 0.001);
      } else {
        auto depth_cv = cv_bridge::toCvCopy(depth_msg,
                          sensor_msgs::image_encodings::TYPE_32FC1);
        depth = depth_cv->image.clone();
      }

      logRgbdInputInfo(rgb_msg, depth_msg);

      const float valid_depth_ratio = computeValidDepthRatio(depth);
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "valid depth ratio = %.3f", valid_depth_ratio);

      if (valid_depth_ratio < 0.10f) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Depth ratio too low: %.3f, skipping", valid_depth_ratio);
        return;
      }

      bool expected_idle = false;
      if (!tracking_in_progress_.compare_exchange_strong(expected_idle, true)) {
        rgbd_busy_drop_count_.fetch_add(1, std::memory_order_relaxed);
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Skipping RGB-D pair because previous ORB TrackRGBD call is still running. busy_drops=%ld",
          rgbd_busy_drop_count_.load());
        return;
      }
      const auto clear_tracking = std::shared_ptr<void>(
        nullptr, [this](void *) { tracking_in_progress_.store(false); });

      last_tracked_wall_stamp_.store(now_wall);

      const auto track_start = std::chrono::steady_clock::now();
      track_call_count_.fetch_add(1, std::memory_order_relaxed);

      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Calling ORB TrackRGBD: t=%.3f rgb-depth dt=%.4f features=%d levels=%d",
        tr, dt, localization_nfeatures_, localization_nlevels_);

      // Run ORB-SLAM3 RGB-D tracking. If the log stops after the line above,
      // ORB-SLAM3 itself is busy in relocalization/tracking, not the ROS wrapper.
      Sophus::SE3f Tcw = slam_->TrackRGBD(rgb, depth, tr);
      const auto track_elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - track_start).count();
      const int tracking_state = slam_->GetTrackingState();
      publishTrackingStatus(tracking_state);
      if (track_elapsed > 1.0) {
        RCLCPP_WARN(
          get_logger(),
          "ORB TrackRGBD took %.2f s. Jetson is overloaded; lower target_tracking_hz/nFeatures or use localization-only.",
          track_elapsed);
      }
      Eigen::Matrix4f Tcw_mat = Tcw.matrix();

      if (!Tcw_mat.allFinite()) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "ORB-SLAM3 returned non-finite pose");
        publishPoseDebug(rgb_msg->header.stamp, tracking_state, false, false, false);
        return;
      }

      Sophus::SE3f Twc = Tcw.inverse();
      Eigen::Matrix4f Twc_mat = Twc.matrix();

      const double px = static_cast<double>(Twc_mat(0, 3));
      const double py = static_cast<double>(Twc_mat(1, 3));
      const double pz = static_cast<double>(Twc_mat(2, 3));

      tf2::Matrix3x3 Rwc(
        Twc_mat(0, 0), Twc_mat(0, 1), Twc_mat(0, 2),
        Twc_mat(1, 0), Twc_mat(1, 1), Twc_mat(1, 2),
        Twc_mat(2, 0), Twc_mat(2, 1), Twc_mat(2, 2));
      tf2::Quaternion qwc;
      Rwc.getRotation(qwc);

      const bool almost_identity =
        std::abs(px) < 1e-6 && std::abs(py) < 1e-6 && std::abs(pz) < 1e-6 &&
        std::abs(qwc.x()) < 1e-6 && std::abs(qwc.y()) < 1e-6 &&
        std::abs(qwc.z()) < 1e-6 && std::abs(qwc.w() - 1.0) < 1e-6;

      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "tracking=%s(%d), TrackRGBD=%.3fs, pose=(%.3f, %.3f, %.3f), identity=%s",
        trackingStateName(tracking_state), tracking_state, track_elapsed,
        px, py, pz, almost_identity ? "true" : "false");

      logTrackingMetrics(
        tr, dt, valid_depth_ratio, tracking_state, track_elapsed,
        px, py, pz, almost_identity);
      logKeypointsSnapshot(rgb, tr);
      publishTrackingPreview(rgb, tr, tracking_state);
      publishRvizGraph(rgb_msg->header.stamp, tr);
      publishFullMapPoints(rgb_msg->header.stamp, tr);

      if (tracking_state != 2 && tracking_state != 5) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "ORB-SLAM3 tracking is %s; not publishing pose.",
          trackingStateName(tracking_state));
        publishPoseDebug(rgb_msg->header.stamp, tracking_state, false, false, false);
        return;
      }

      if (almost_identity) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "ORB-SLAM3 identity pose (not tracking yet). "
          "Move the robot so it can triangulate features.");
        publishPoseDebug(rgb_msg->header.stamp, tracking_state, false, false, false);
        return;
      }

      if (!first_valid_pose_) {
        first_valid_pose_ = true;
        RCLCPP_INFO(get_logger(), "ORB-SLAM3 produced first non-identity pose");
      }

      tf2::Transform T_orb_world_camera;
      T_orb_world_camera.setOrigin(tf2::Vector3(px, py, pz));
      T_orb_world_camera.setRotation(qwc);

      const tf2::Transform T_world_camera =
        orbOpticalWorldToRosMapTransform() * T_orb_world_camera;

      geometry_msgs::msg::PoseStamped cam_pose_msg;
      cam_pose_msg.header.stamp = rgb_msg->header.stamp;
      cam_pose_msg.header.frame_id = world_frame_;
      cam_pose_msg.pose = toPoseMsg(T_world_camera);
      if (publish_raw_camera_pose_) {
        raw_camera_pose_pub_->publish(cam_pose_msg);
        // Legacy camera_pose remains raw camera pose for compatibility.
        camera_pose_pub_->publish(cam_pose_msg);
      }

      geometry_msgs::msg::PoseStamped base_pose_msg;
      base_pose_msg.header = cam_pose_msg.header;
      try {
        auto tf_cam_from_base =
          tf_buffer_.lookupTransform(camera_frame_, base_frame_, tf2::TimePointZero);
        tf2::Transform T_cam_base;
        tf2::fromMsg(tf_cam_from_base.transform, T_cam_base);
        tf2::Transform T_world_base = T_world_camera * T_cam_base;
        base_pose_msg.pose = toPoseMsg(T_world_base);
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 3000,
          "Cannot transform %s <- %s: %s. Raw camera pose remains available; "
          "base pose and legacy /pose are not published.",
          camera_frame_.c_str(), base_frame_.c_str(), ex.what());
        publishPoseDebug(rgb_msg->header.stamp, tracking_state, true, false, false);
        return;
      }

      if (force_planar_output_) {
        forcePlanarPose(base_pose_msg.pose);
      }

      if (publish_base_pose_) {
        base_pose_pub_->publish(base_pose_msg);
        // Legacy pose is explicitly the robot base pose.
        pose_pub_->publish(base_pose_msg);
      }
      publishPoseDebug(rgb_msg->header.stamp, tracking_state, true, true, true);

      if (publish_tf_) {
        publishOrbTf(base_pose_msg);
      }

      path_msg_.header.stamp = rgb_msg->header.stamp;
      path_msg_.poses.push_back(base_pose_msg);
      if (max_path_length_ > 0 &&
          path_msg_.poses.size() > static_cast<size_t>(max_path_length_)) {
        path_msg_.poses.erase(
          path_msg_.poses.begin(),
          path_msg_.poses.begin() +
          static_cast<std::ptrdiff_t>(path_msg_.poses.size() - max_path_length_));
      }
      path_pub_->publish(path_msg_);
      // Keep diagnostics after the critical camera/base pose publications.
      publishTrackedMapPoints(rgb_msg->header.stamp);

    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "ORB3 RGB-D callback exception: %s", e.what());
    }
  }

  void publishOrbTf(const geometry_msgs::msg::PoseStamped & base_pose_msg)
  {
    if (tf_publish_mode_ == "map_to_odom") {
      try {
        const auto tf_odom_from_base =
          tf_buffer_.lookupTransform(odom_frame_, base_frame_, tf2::TimePointZero);

        tf2::Transform T_odom_base;
        tf2::fromMsg(tf_odom_from_base.transform, T_odom_base);

        const tf2::Transform T_world_base = poseToTf(base_pose_msg.pose);
        const tf2::Transform T_world_odom = T_world_base * T_odom_base.inverse();

        geometry_msgs::msg::TransformStamped t;
        t.header = base_pose_msg.header;
        t.child_frame_id = odom_frame_;
        t.transform.translation.x = T_world_odom.getOrigin().x();
        t.transform.translation.y = T_world_odom.getOrigin().y();
        t.transform.translation.z = T_world_odom.getOrigin().z();
        t.transform.rotation = tf2::toMsg(T_world_odom.getRotation());
        tf_broadcaster_->sendTransform(t);
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Cannot publish %s -> %s like AMCL because %s -> %s is missing: %s",
          world_frame_.c_str(), odom_frame_.c_str(),
          odom_frame_.c_str(), base_frame_.c_str(), ex.what());
      }
      return;
    }

    if (tf_publish_mode_ != "world_to_base") {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Unknown tf_publish_mode='%s'. Use 'world_to_base' or 'map_to_odom'.",
        tf_publish_mode_.c_str());
      return;
    }

    geometry_msgs::msg::TransformStamped t;
    t.header = base_pose_msg.header;
    t.child_frame_id = base_frame_;
    t.transform.translation.x = base_pose_msg.pose.position.x;
    t.transform.translation.y = base_pose_msg.pose.position.y;
    t.transform.translation.z = base_pose_msg.pose.position.z;
    t.transform.rotation = base_pose_msg.pose.orientation;
    tf_broadcaster_->sendTransform(t);
  }

  void publishPoseDebug(
    const builtin_interfaces::msg::Time & stamp, int tracking_state,
    bool raw_camera_pose_valid, bool base_pose_valid, bool used_tf)
  {
    if (!publish_pose_debug_) {
      return;
    }
    std_msgs::msg::String msg;
    std::ostringstream out;
    out << std::boolalpha << std::setprecision(15)
        << "{\"stamp\":"
        << (static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9)
        << ",\"tracking_state\":" << tracking_state
        << ",\"tracking_status\":\"" << trackingStateName(tracking_state) << "\""
        << ",\"raw_camera_pose_valid\":" << raw_camera_pose_valid
        << ",\"base_pose_valid\":" << base_pose_valid
        << ",\"used_tf\":" << used_tf
        << ",\"map_frame\":\"" << world_frame_ << "\""
        << ",\"camera_frame\":\"" << camera_frame_ << "\""
        << ",\"base_frame\":\"" << base_frame_ << "\""
        << ",\"raw_camera_topic\":\"" << raw_camera_pose_topic_ << "\""
        << ",\"base_pose_topic\":\"" << base_pose_topic_ << "\""
        << ",\"gtsam_orb_topic\":\"" << base_pose_topic_ << "\"}";
    msg.data = out.str();
    pose_debug_pub_->publish(msg);
  }

  void logRgbdInputInfo(
    const sensor_msgs::msg::Image::ConstSharedPtr & rgb_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg)
  {
    if (logged_rgbd_input_) {
      return;
    }
    logged_rgbd_input_ = true;

    sensor_msgs::msg::CameraInfo::ConstSharedPtr info_msg;
    {
      std::lock_guard<std::mutex> lock(latest_camera_info_mutex_);
      info_msg = latest_camera_info_msg_;
    }

    RCLCPP_INFO(
      get_logger(),
      "First RGB-D input: rgb=%ux%u %s frame=%s, depth=%ux%u %s frame=%s",
      rgb_msg->width, rgb_msg->height, rgb_msg->encoding.c_str(), rgb_msg->header.frame_id.c_str(),
      depth_msg->width, depth_msg->height, depth_msg->encoding.c_str(), depth_msg->header.frame_id.c_str());
    RCLCPP_INFO(
      get_logger(),
      "ORB settings expect depth already in meters after ROS wrapper conversion; use RGBD.DepthMapFactor: 1.0");
    RCLCPP_INFO(
      get_logger(),
      "Active ORB files: settings_file=%s vocabulary_file=%s",
      settings_file_.c_str(), vocab_file_.c_str());

    if (rgb_msg->width != depth_msg->width || rgb_msg->height != depth_msg->height) {
      RCLCPP_WARN(
        get_logger(),
        "RGB/depth resolution mismatch. ORB RGB-D needs depth aligned/registered to the color image.");
    }

    if (info_msg) {
      RCLCPP_INFO(
        get_logger(),
        "Use these YAML intrinsics from %s: Camera1.fx=%.6f Camera1.fy=%.6f Camera1.cx=%.6f Camera1.cy=%.6f",
        camera_info_topic_.c_str(), info_msg->k[0], info_msg->k[4], info_msg->k[2], info_msg->k[5]);
      if (info_msg->width != rgb_msg->width || info_msg->height != rgb_msg->height) {
        RCLCPP_WARN(
          get_logger(),
          "camera_info size %ux%u does not match RGB image %ux%u. Fix Astra camera_info/YAML before mapping.",
          info_msg->width, info_msg->height, rgb_msg->width, rgb_msg->height);
      }
    } else {
      RCLCPP_WARN(
        get_logger(),
        "No camera_info received yet on %s. Verify YAML with: ros2 topic echo %s --once",
        camera_info_topic_.c_str(), camera_info_topic_.c_str());
    }
  }

  // ---- member data ----
  std::string vocab_file_, settings_file_, load_atlas_file_;
  std::string rgb_topic_, depth_topic_, camera_info_topic_;
  std::string world_frame_, odom_frame_, base_frame_, camera_frame_;
  std::string path_topic_, pose_topic_, camera_pose_topic_;
  std::string raw_camera_pose_topic_, base_pose_topic_, pose_debug_topic_;
  std::string map_points_topic_, tracked_points_topic_;
  std::string rviz_markers_topic_, rviz_keyframes_topic_;
  std::string tracking_state_topic_, tracking_status_topic_;
  std::string tracking_preview_topic_;
  std::string output_dir_;
  std::string tf_publish_mode_;

  bool use_viewer_{false};
  bool publish_tf_{false};
  bool force_planar_output_{true};
  bool save_outputs_{true};
  bool save_atlas_{false};
  bool save_keypoint_images_{true};
  bool unique_output_dir_{true};
  bool publish_tracking_preview_{true};
  bool publish_raw_camera_pose_{true};
  bool publish_base_pose_{true};
  bool publish_pose_debug_{true};
  bool outputs_saved_{false};
  bool first_valid_pose_{false};
  bool localization_mode_active_{false};
  bool single_map_mode_{true};
  bool logged_camera_info_{false};
  bool logged_rgbd_input_{false};
  int sync_queue_size_{10};
  double max_rgb_depth_dt_{0.20};
  double target_tracking_hz_{10.0};
  double max_track_rgbd_sec_{0.0};
  double image_callback_watchdog_sec_{8.0};
  double map_points_publish_hz_{1.0};
  double graph_publish_hz_{1.0};
  std::atomic<double> last_tracked_wall_stamp_{-1.0};
  std::atomic<double> last_rgb_callback_wall_stamp_{-1.0};
  std::atomic<double> last_depth_callback_wall_stamp_{-1.0};
  std::atomic<long> rgb_received_count_{0};
  std::atomic<long> depth_received_count_{0};
  std::atomic<long> rgbd_matched_count_{0};
  std::atomic<long> rgbd_dt_drop_count_{0};
  std::atomic<long> rgbd_processed_count_{0};
  std::atomic<long> track_call_count_{0};
  std::atomic<long> rgbd_busy_drop_count_{0};
  std::atomic<bool> tracking_in_progress_{false};
  double last_map_points_publish_stamp_{-1.0};
  double last_graph_publish_stamp_{-1.0};
  double last_keypoint_log_stamp_{-1.0};
  double last_tracking_preview_stamp_{-1.0};
  double keypoint_log_hz_{1.0};
  double tracking_preview_hz_{0.5};
  double localization_scale_factor_{1.2};
  int max_path_length_{2000};
  int max_saved_keypoints_{1200};
  int tracking_preview_width_{320};
  int covisibility_min_weight_{100};
  double keyframe_frustum_size_{0.08};
  int auto_localization_map_points_{9000};
  int localization_nfeatures_{800};
  int localization_nlevels_{6};
  int max_published_map_points_{12000};
  int max_published_tracked_points_{1500};

  std::unique_ptr<ORB_SLAM3::System> slam_;
  rclcpp::TimerBase::SharedPtr tracking_heartbeat_timer_;
  rclcpp::TimerBase::SharedPtr tracking_watchdog_timer_;
  rclcpp::TimerBase::SharedPtr rgbd_processing_timer_;
  rclcpp::TimerBase::SharedPtr tracking_preview_timer_;
  rclcpp::CallbackGroup::SharedPtr sensor_callback_group_;
  rclcpp::CallbackGroup::SharedPtr rgbd_processing_callback_group_;
  rclcpp::CallbackGroup::SharedPtr tracking_preview_callback_group_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  std::deque<sensor_msgs::msg::Image::ConstSharedPtr> rgb_queue_;
  std::deque<sensor_msgs::msg::Image::ConstSharedPtr> depth_queue_;
  std::mutex rgbd_queue_mutex_;
  std::mutex tracking_preview_rate_mutex_;
  sensor_msgs::msg::Image::ConstSharedPtr latest_preview_rgb_msg_;
  std::mutex latest_preview_rgb_mutex_;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr latest_camera_info_msg_;
  std::mutex latest_camera_info_mutex_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr camera_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr raw_camera_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr base_pose_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pose_debug_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr mappoints_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr tracked_points_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr rviz_markers_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr rviz_keyframes_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr tracking_state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr tracking_status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr tracking_preview_pub_;
  nav_msgs::msg::Path path_msg_;
  std::ofstream metrics_csv_;
  std::ofstream keypoints_csv_;
  std::atomic<int> latest_tracking_state_{-1};
  std::atomic<double> latest_tracking_state_wall_stamp_{-1.0};
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Orb3RgbdNode>();
  RCLCPP_INFO(node->get_logger(), "ORB RGB-D node construction complete; creating executor");
  rclcpp::executors::MultiThreadedExecutor executor(
    rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  RCLCPP_INFO(node->get_logger(), "ORB RGB-D executor started with 2 threads");
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
