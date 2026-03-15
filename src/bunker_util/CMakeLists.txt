cmake_minimum_required(VERSION 3.8)
project(bunker_util)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ----- 의존성 -----
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_msgs REQUIRED)


# include 경로 (공유 헤더)
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# -------------------------------------------------------
# 공유 라이브러리: 세 가지 고도 맵 알고리즘
# -------------------------------------------------------
set(ELEVATION_DEPS
  rclcpp
  grid_map_ros
  grid_map_msgs
)

add_library(elevation_map_max STATIC
  src/elevation_map_max.cpp
)
ament_target_dependencies(elevation_map_max ${ELEVATION_DEPS})
target_include_directories(elevation_map_max PUBLIC include)

add_library(elevation_map_kalman STATIC
  src/elevation_map_kalman.cpp
)
ament_target_dependencies(elevation_map_kalman ${ELEVATION_DEPS})
target_include_directories(elevation_map_kalman PUBLIC include)

add_library(elevation_map_dual_layer STATIC
  src/elevation_map_dual_layer.cpp
)
ament_target_dependencies(elevation_map_dual_layer ${ELEVATION_DEPS})
target_include_directories(elevation_map_dual_layer PUBLIC include)

# -------------------------------------------------------
# Executable: scan_processor
# -------------------------------------------------------
add_executable(scan_processor src/scan_processor.cpp)
ament_target_dependencies(scan_processor
  rclcpp sensor_msgs tf2 tf2_ros tf2_geometry_msgs
  geometry_msgs nav_msgs
  grid_map_ros grid_map_msgs
)
target_link_libraries(scan_processor
  elevation_map_max
  elevation_map_kalman
  elevation_map_dual_layer
  ${OpenCV_LIBS}
)

# -------------------------------------------------------
# Executable: map_maker
# -------------------------------------------------------
add_executable(map_maker src/map_maker.cpp)
ament_target_dependencies(map_maker
  rclcpp sensor_msgs tf2_ros tf2_geometry_msgs
  geometry_msgs visualization_msgs nav_msgs
  grid_map_ros grid_map_msgs
)
target_link_libraries(map_maker ${OpenCV_LIBS})

# -------------------------------------------------------
# Executable: map_sub
# -------------------------------------------------------
add_executable(map_sub src/map_sub.cpp)
ament_target_dependencies(map_sub
  rclcpp sensor_msgs tf2_ros tf2_geometry_msgs
  geometry_msgs visualization_msgs nav_msgs
  grid_map_ros grid_map_msgs
)
target_link_libraries(map_sub)

# -------------------------------------------------------
# Executable: test_map
# -------------------------------------------------------
add_executable(test_map src/test_map.cpp)
ament_target_dependencies(test_map
  rclcpp sensor_msgs tf2_ros tf2_geometry_msgs
  geometry_msgs visualization_msgs nav_msgs
  grid_map_ros grid_map_msgs
)
target_link_libraries(test_map ${OpenCV_LIBS})

# -------------------------------------------------------
# 설치
# -------------------------------------------------------
install(TARGETS
  scan_processor
  map_maker
  map_sub
  test_map
  DESTINATION lib/${PROJECT_NAME}/
)

install(PROGRAMS
  scripts/waypoint_follower.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_package()
