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
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(lqr_compute src/lqr_control_node.cpp)

ament_target_dependencies(lqr_compute rclcpp sensor_msgs geometry_msgs tf2)

install(TARGETS
  lqr_compute
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
