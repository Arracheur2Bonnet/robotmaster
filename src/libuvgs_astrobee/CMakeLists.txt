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
  #ff_msgs
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

# Source files
set(SOURCE_FILES
  src/carolus_astrobee.cpp
  src/pose_est.cpp
  src/ceresP4P.cpp
)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs image_transport geometry_msgs #ff_msgs
  DEPENDS OpenCV Eigen3 Ceres
)

# Add executable
add_executable(carolus_astrobee ${SOURCE_FILES})

# Link libraries
target_link_libraries(carolus_astrobee
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${CERES_LIBRARIES}
)

# Install the executable
install(TARGETS carolus_astrobee
  RUNTIME DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
