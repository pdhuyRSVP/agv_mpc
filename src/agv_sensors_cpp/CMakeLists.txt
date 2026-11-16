cmake_minimum_required(VERSION 3.8)
project(agv_sensors_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(hwt901b_modbus_node src/hwt901b_modbus_node.cpp)
ament_target_dependencies(hwt901b_modbus_node
  rclcpp
  sensor_msgs
)

add_executable(wheel_vel_node_direct src/wheel_vel_node_direct.cpp)
ament_target_dependencies(wheel_vel_node_direct
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
)

add_executable(sensor_health_node src/sensor_health_node.cpp)
ament_target_dependencies(sensor_health_node
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
)

add_executable(camera_preview_node src/camera_preview_node.cpp)
ament_target_dependencies(camera_preview_node
  rclcpp
  sensor_msgs
  std_msgs
)

install(TARGETS
  hwt901b_modbus_node
  wheel_vel_node_direct
  sensor_health_node
  camera_preview_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
