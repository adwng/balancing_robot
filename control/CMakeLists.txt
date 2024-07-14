cmake_minimum_required(VERSION 3.8)
project(control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)

add_executable(lqr_compute src/lqr_control_node.cpp)
add_executable(pid_compute src/PID_compute_node.cpp)

ament_target_dependencies(lqr_compute rclcpp sensor_msgs geometry_msgs tf2)
ament_target_dependencies(pid_compute rclcpp sensor_msgs geometry_msgs tf2)

install(TARGETS
  lqr_compute
  pid_compute
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
