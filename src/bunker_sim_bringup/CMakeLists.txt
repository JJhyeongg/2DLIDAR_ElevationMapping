cmake_minimum_required(VERSION 3.8)
project(bunker_sim_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(map_cal src/map_cal.cpp)
ament_target_dependencies(map_cal rclcpp tf2 tf2_ros geometry_msgs)

add_executable(gps_cov_filter src/gps_cov_filter_node.cpp)
ament_target_dependencies(gps_cov_filter rclcpp sensor_msgs)

# Install the executable
install(TARGETS map_cal
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS gps_cov_filter
  DESTINATION lib/${PROJECT_NAME})

# Install launch, worlds, rviz directories
install(
  DIRECTORY launch worlds rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
