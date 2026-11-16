cmake_minimum_required(VERSION 3.8)
project(orb3_agv_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Default workflow for this package is 2D-map-first simulation.
# ORB-SLAM3 nodes are optional so you can build the 2D LiDAR/EKF/Nav2 stack
# even if ORB_SLAM3/Pangolin are not ready yet.
option(BUILD_ORB_SLAM3_NODES "Build ORB-SLAM3 RGB-D/RGB-D-IMU wrapper nodes" OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GTSAM REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)

set(TARGETS_TO_INSTALL "")

add_library(graph_diagnostics src/graph_diagnostics.cpp)
target_include_directories(graph_diagnostics PUBLIC include)
target_link_libraries(graph_diagnostics gtsam ${OpenCV_LIBS})

# -----------------------------------------------------------------------------
# ORB-Lidar consistency / loop validation node.
# This node has no dependency on ORB_SLAM3 library; it only compares the ORB pose
# topic with the LiDAR/EKF pose in the map frame.
# -----------------------------------------------------------------------------
add_executable(orb_lidar_loop_validator src/orb_lidar_loop_validator.cpp)
target_include_directories(orb_lidar_loop_validator PRIVATE include)
ament_target_dependencies(orb_lidar_loop_validator
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)
list(APPEND TARGETS_TO_INSTALL orb_lidar_loop_validator)

# -----------------------------------------------------------------------------
# Pose2 factor graph localization.
# EKF odometry creates the main node chain with BetweenFactors. ORB raw and
# optional scan odometry are also relative BetweenFactors. AMCL/scan-map is the
# only recurring absolute map correction factor.
# -----------------------------------------------------------------------------
add_executable(gtsam_pose2_fusion_node src/gtsam_pose2_fusion_node.cpp)
target_include_directories(gtsam_pose2_fusion_node PRIVATE include)
ament_target_dependencies(gtsam_pose2_fusion_node
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  action_msgs
  nav2_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)
target_link_libraries(gtsam_pose2_fusion_node gtsam)
target_link_libraries(gtsam_pose2_fusion_node graph_diagnostics)
list(APPEND TARGETS_TO_INSTALL gtsam_pose2_fusion_node)

if(BUILD_TESTING)
  add_executable(graph_diagnostics_smoke_test test/graph_diagnostics_smoke_test.cpp)
  target_include_directories(graph_diagnostics_smoke_test PRIVATE include)
  target_link_libraries(graph_diagnostics_smoke_test graph_diagnostics gtsam)
endif()

# -----------------------------------------------------------------------------
# Optional ORB-SLAM3 wrappers. Enable only after ORB_SLAM3, Pangolin, OpenCV and
# the vocabulary/config files are ready:
#   colcon build --symlink-install --cmake-args -DBUILD_ORB_SLAM3_NODES=ON
# -----------------------------------------------------------------------------
if(BUILD_ORB_SLAM3_NODES)
  message(STATUS "BUILD_ORB_SLAM3_NODES=ON: building ORB-SLAM3 wrapper nodes")

  find_package(cv_bridge REQUIRED)
  find_package(message_filters REQUIRED)
  find_package(OpenCV REQUIRED)

  cmake_policy(SET CMP0072 NEW)
  set(OpenGL_GL_PREFERENCE GLVND)
  find_package(OpenGL REQUIRED COMPONENTS OpenGL EGL)
  find_package(GLEW REQUIRED)

  if(NOT DEFINED ORB_SLAM3_ROOT)
    set(ORB_SLAM3_ROOT "$ENV{HOME}/Dev/ORB_SLAM3" CACHE PATH "Path to ORB_SLAM3 source root")
  endif()
  set(ORB_SLAM3_LIB  "${ORB_SLAM3_ROOT}/lib/libORB_SLAM3.so" CACHE FILEPATH "Path to libORB_SLAM3.so")

  if(NOT EXISTS ${ORB_SLAM3_LIB})
    message(FATAL_ERROR
      "ORB_SLAM3 library not found at ${ORB_SLAM3_LIB}.\n"
      "Either build without ORB using the default option, or override with:\n"
      "  colcon build --cmake-args -DBUILD_ORB_SLAM3_NODES=ON -DORB_SLAM3_ROOT=/your/path")
  endif()

  set(PANGOLIN_BUILD_DIR "$ENV{HOME}/Dev/Pangolin/build" CACHE PATH "Path to Pangolin build directory")
  set(PANGOLIN_SOURCE_DIR "$ENV{HOME}/Dev/Pangolin" CACHE PATH "Path to Pangolin source directory")
  set(PANGOLIN_INCLUDE_DIRS
    "${PANGOLIN_SOURCE_DIR}/components/pango_core/include"
    "${PANGOLIN_SOURCE_DIR}/components/pango_display/include"
    "${PANGOLIN_SOURCE_DIR}/components/pango_geometry/include"
    "${PANGOLIN_SOURCE_DIR}/components/pango_glgeometry/include"
    "${PANGOLIN_SOURCE_DIR}/components/pango_image/include"
    "${PANGOLIN_SOURCE_DIR}/components/pango_opengl/include"
    "${PANGOLIN_SOURCE_DIR}/components/pango_packetstream/include"
    "${PANGOLIN_SOURCE_DIR}/components/pango_plot/include"
    "${PANGOLIN_SOURCE_DIR}/components/pango_scene/include"
    "${PANGOLIN_SOURCE_DIR}/components/pango_vars/include"
    "${PANGOLIN_SOURCE_DIR}/components/pango_video/include"
    "${PANGOLIN_SOURCE_DIR}/components/pango_windowing/include"
  )
  find_library(PANGO_CORE_LIB      pango_core      PATHS "${PANGOLIN_BUILD_DIR}" /usr/local/lib REQUIRED)
  find_library(PANGO_DISPLAY_LIB   pango_display   PATHS "${PANGOLIN_BUILD_DIR}" /usr/local/lib REQUIRED)
  find_library(PANGO_OPENGL_LIB    pango_opengl    PATHS "${PANGOLIN_BUILD_DIR}" /usr/local/lib REQUIRED)
  find_library(PANGO_WINDOWING_LIB pango_windowing PATHS "${PANGOLIN_BUILD_DIR}" /usr/local/lib REQUIRED)

  add_executable(orb3_rgbd_node src/orb3_rgbd_node.cpp)
  target_include_directories(orb3_rgbd_node PRIVATE
    include
    ${ORB_SLAM3_ROOT}
    ${ORB_SLAM3_ROOT}/include
    ${ORB_SLAM3_ROOT}/include/CameraModels
    ${ORB_SLAM3_ROOT}/Thirdparty/Sophus
    /usr/local/include
    ${PANGOLIN_INCLUDE_DIRS}
    ${GLEW_INCLUDE_DIRS}
  )
  target_compile_definitions(orb3_rgbd_node PRIVATE GLEW_STATIC=0)
	  ament_target_dependencies(orb3_rgbd_node
	    rclcpp sensor_msgs geometry_msgs nav_msgs std_msgs visualization_msgs tf2 tf2_ros tf2_geometry_msgs cv_bridge message_filters
	  )
  target_link_libraries(orb3_rgbd_node
    ${ORB_SLAM3_LIB}
    ${PANGO_OPENGL_LIB} ${PANGO_DISPLAY_LIB} ${PANGO_WINDOWING_LIB} ${PANGO_CORE_LIB}
    OpenGL::OpenGL OpenGL::EGL GLEW::GLEW ${OpenCV_LIBS}
  )

  # Build the real RGB-D + IMU wrapper. The old src/orb3_imu_rgbd_node.cpp
  # starts ORB_SLAM3::System::RGBD and therefore does NOT use IMU factors.
  add_executable(orb3_imu_rgbd_node src/orb3_imu_rgbd_node_imu.cpp)
  target_include_directories(orb3_imu_rgbd_node PRIVATE
    include
    ${ORB_SLAM3_ROOT}
    ${ORB_SLAM3_ROOT}/include
    ${ORB_SLAM3_ROOT}/include/CameraModels
    ${ORB_SLAM3_ROOT}/Thirdparty/Sophus
    /usr/local/include
    ${PANGOLIN_INCLUDE_DIRS}
    ${GLEW_INCLUDE_DIRS}
  )
  target_compile_definitions(orb3_imu_rgbd_node PRIVATE GLEW_STATIC=0)
  ament_target_dependencies(orb3_imu_rgbd_node
    rclcpp sensor_msgs geometry_msgs nav_msgs std_msgs tf2 tf2_ros tf2_geometry_msgs cv_bridge message_filters
  )
  target_link_libraries(orb3_imu_rgbd_node
    ${ORB_SLAM3_LIB}
    ${PANGO_OPENGL_LIB} ${PANGO_DISPLAY_LIB} ${PANGO_WINDOWING_LIB} ${PANGO_CORE_LIB}
    OpenGL::OpenGL OpenGL::EGL GLEW::GLEW ${OpenCV_LIBS}
  )

  add_executable(orb3_stereo_imu_node src/orb3_stereo_imu_node.cpp)
  target_include_directories(orb3_stereo_imu_node PRIVATE
    include
    ${ORB_SLAM3_ROOT}
    ${ORB_SLAM3_ROOT}/include
    ${ORB_SLAM3_ROOT}/include/CameraModels
    ${ORB_SLAM3_ROOT}/Thirdparty/Sophus
    /usr/local/include
    ${PANGOLIN_INCLUDE_DIRS}
    ${GLEW_INCLUDE_DIRS}
  )
  target_compile_definitions(orb3_stereo_imu_node PRIVATE GLEW_STATIC=0)
  ament_target_dependencies(orb3_stereo_imu_node
    rclcpp sensor_msgs geometry_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs cv_bridge message_filters
  )
  target_link_libraries(orb3_stereo_imu_node
    ${ORB_SLAM3_LIB}
    ${PANGO_OPENGL_LIB} ${PANGO_DISPLAY_LIB} ${PANGO_WINDOWING_LIB} ${PANGO_CORE_LIB}
    OpenGL::OpenGL OpenGL::EGL GLEW::GLEW ${OpenCV_LIBS}
  )

  list(APPEND TARGETS_TO_INSTALL orb3_rgbd_node orb3_imu_rgbd_node orb3_stereo_imu_node)
else()
  message(STATUS "BUILD_ORB_SLAM3_NODES=OFF: building 2D-map-first stack only")
endif()

install(TARGETS graph_diagnostics ${TARGETS_TO_INSTALL}
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch  DESTINATION share/${PROJECT_NAME} OPTIONAL)
install(DIRECTORY config  DESTINATION share/${PROJECT_NAME} OPTIONAL)
install(DIRECTORY meshes  DESTINATION share/${PROJECT_NAME} OPTIONAL)
install(DIRECTORY maps    DESTINATION share/${PROJECT_NAME} OPTIONAL)
install(DIRECTORY tools   DESTINATION share/${PROJECT_NAME} OPTIONAL)

install(PROGRAMS
  tools/initial_pose_publisher.py
  tools/orb_amcl_aligner.py
  tools/orb_display_corrector.py
  tools/orb_map_projector.py
  tools/orb_feature_overlay.py
  tools/relative_scan_matcher.py
  tools/scan_to_map_matcher.py
  tools/check_slam_toolbox_posegraph.py
  tools/slam_toolbox_global_posegraph_loader.py
  tools/slam_toolbox_tf_pose_bridge.py
  tools/global_scan_initializer.py
  tools/trajectory_compare_logger.py
  tools/export_orb_run_summary.py
  tools/run_orb_rgbd_graph_eval_collect.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
