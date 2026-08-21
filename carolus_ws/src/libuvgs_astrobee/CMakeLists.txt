cmake_minimum_required(VERSION 3.8)
project(carolus_astrobee)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  image_transport
  geometry_msgs
  ff_msgs
)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)

# Include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)

# carolus_core -- 2026-08-18, Hector's ROS-portability request ("more
# important than the rest of the plan"): the P4P/Ceres solver and the beacon
# detection algorithm, with ZERO ROS dependency, so a future ROS2 wrapper
# (Humble or a newer distro) needs only this library relinked, not the
# algorithm rewritten. Deliberately built with NO catkin/ROS linkage at all --
# verify this stays true with `grep -rn "ros::\|#include <ros" src/ceresP4P.cpp
# src/pose_est.cpp src/beacon_detector.cpp` (must return nothing) before
# adding anything to this target.
set(CORE_SOURCE_FILES
  src/ceresP4P.cpp
  src/pose_est.cpp
  src/beacon_detector.cpp
  src/pose_filter.cpp
)

add_library(carolus_core STATIC ${CORE_SOURCE_FILES})
target_link_libraries(carolus_core
  ${OpenCV_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${CERES_LIBRARIES}
)

# The ROS1 node itself -- thin wrapper around carolus_core (subscriptions,
# publishers, message<->cv::Mat/Eigen conversion, param loading). This is the
# only target in the package that links against ROS.
set(SOURCE_FILES
  src/carolus_astrobee.cpp
)

catkin_package(
  CATKIN_DEPENDS roscpp ff_msgs sensor_msgs image_transport geometry_msgs
  DEPENDS OpenCV Eigen3 Ceres
)

# Add executable
add_executable(carolus_astrobee ${SOURCE_FILES})

# Link libraries
target_link_libraries(carolus_astrobee
  carolus_core
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${CERES_LIBRARIES}
)

add_dependencies(carolus_astrobee ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Install the executable
install(TARGETS carolus_astrobee
  RUNTIME DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# No launch/ directory left to install: this package's own launch files
# (carolus.launch, testcarolus_{follower,leader}.launch, testcarolusparams.launch)
# predated the 2026-08-03 config split and were superseded by
# carolus_node/launch/{carolus,testcarolus,ekf_lite}.launch, which is what
# every current entry point actually uses. Archived 2026-08-12 (zero live
# references found), not deleted -- see archive/libuvgs_astrobee-legacy-launch-2026-08-12/.
