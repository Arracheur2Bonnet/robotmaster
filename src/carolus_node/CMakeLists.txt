
cmake_minimum_required(VERSION 3.0.2)
project(carolus_node)

## Compile as C++11
add_compile_options(-std=c++17)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  cv_bridge
  image_transport
  ff_msgs
)

## Find system dependencies
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs cv_bridge image_transport ff_msgs
)

## Install Python scripts so `roslaunch`/`rosrun` can resolve them by
## pkg+type (beacon_absolute_pose.py, F3, 2026-07-23). carolus_tf_broadcaster.py
## n'en avait pas besoin car lance directement en chemin absolu via SSH
## (carolus_launcher.py), pas via roslaunch <node pkg=...>.
catkin_install_python(PROGRAMS
  scripts/beacon_absolute_pose.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)

# carolus_astrobee is built by libuvgs_astrobee — no duplicate target here
