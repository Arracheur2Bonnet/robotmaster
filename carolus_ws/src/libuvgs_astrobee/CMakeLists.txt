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
  ##ADD FOR ASTROBEE
  geometry_msgs
)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  #${OpenGV_INCLUDE_DIRS}  
  #/usr/local/include/sym  
  #/usr/local/include/sym/ops  
)

# Link directories for SymForce libraries
#link_directories(/usr/local/lib)

# Manually specify SymForce libraries
#set(SYMF_LIBRARIES
 # symforce_gen
 # symforce_cholesky
 # symforce_opt
 # symforce_slam
#)

# Source files
set(SOURCE_FILES
  #src/carolus.cpp
  src/carolus_astrobee.cpp
  src/pose_est.cpp
  src/ceresP4P.cpp
)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs image_transport
  DEPENDS OpenCV Eigen3
)

# Executable
add_executable(carolus_astrobee ${SOURCE_FILES})

# Link libraries
target_link_libraries(carolus_astrobee
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${EIGEN3_LIBRARIES}
 # ${OpenGV_LIBRARIES} 
  ${CERES_LIBRARIES}
 # ${SYMF_LIBRARIES}  
)

# Install
install(TARGETS carolus_astrobee
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
