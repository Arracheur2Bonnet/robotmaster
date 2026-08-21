cmake_minimum_required(VERSION 3.8)
project(carolus_ros2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)

# 2026-08-20: this package moved from the repository root into
# raspberry5-carolus-ros2/, made deliberately SELF-CONTAINED -- the detection
# core (ceresP4P/pose_est/beacon_detector/pose_filter) is now a COPY living in
# this same tree (src/, include/carolus_node/), not a relative reference into
# carolus_ws/src/libuvgs_astrobee. That copy WILL drift from the ROS1 node's
# original unless someone syncs it deliberately -- accepted on purpose, so
# this whole folder can be handed over or moved independently of the rest of
# the repository. See this folder's own technical-ros2.tex for the reasoning.
set(CORE_DIR ${CMAKE_CURRENT_SOURCE_DIR})

include_directories(
  ${CORE_DIR}/include
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)

add_library(carolus_core_ros2 STATIC
  ${CORE_DIR}/src/ceresP4P.cpp
  ${CORE_DIR}/src/pose_est.cpp
  ${CORE_DIR}/src/beacon_detector.cpp
  ${CORE_DIR}/src/pose_filter.cpp
)
target_link_libraries(carolus_core_ros2
  ${OpenCV_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${CERES_LIBRARIES}
)

add_executable(carolus_node src/carolus_node_ros2.cpp)
target_link_libraries(carolus_node
  carolus_core_ros2
  ${OpenCV_LIBRARIES}
)
ament_target_dependencies(carolus_node
  rclcpp
  sensor_msgs
  geometry_msgs
  image_transport
  cv_bridge
)

install(TARGETS carolus_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
