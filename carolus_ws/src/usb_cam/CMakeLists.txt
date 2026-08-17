cmake_minimum_required(VERSION 2.8.3)
project(usb_cam)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS image_transport roscpp std_msgs sensor_msgs camera_info_manager)

## pkg-config libraries
find_package(PkgConfig REQUIRED)
pkg_check_modules(avcodec libavcodec REQUIRED)
pkg_check_modules(swscale libswscale REQUIRED)

# 2026-08-17 (this project, lab PC): same fix already applied in
# mins_sandbox/catkin_ws/src/mins/mins/cmake/ROS1.cmake (2026-08-13), same
# root cause. rosconsole.pc's own Libs: line hardcodes the unversioned
# "/usr/lib/x86_64-linux-gnu/liblog4cxx.so" symlink, which on this jammy
# machine resolves to liblog4cxx.so.12 (native jammy package). ROS Noetic's
# own prebuilt librosconsole_log4cxx.so was compiled against liblog4cxx.so.10
# and hardcodes that SONAME. Passing the unversioned symlink through
# catkin_LIBRARIES gave libusb_cam.so a direct NEEDED on liblog4cxx.so.12
# (confirmed via readelf -d) alongside librosconsole's own transitive
# liblog4cxx.so.10 -- two ABI-incompatible major versions loaded in one
# process. `roslaunch usb_cam usb_cam-test.launch` reproduced this every run:
# exit code -11, and under `gdb -batch -ex run -ex bt` the crash was inside
# liblog4cxx.so.12's PropertyConfigurator destructor, called from
# ros::console::impl::initialize() -- a call built against the .10 ABI
# resolving, via the flat dynamic symbol namespace, to the .12
# implementation's layout. usb_cam's own source never calls a log4cxx symbol
# directly; the dependency was pulled in only by this unfiltered list.
# Dropping it here leaves the correct liblog4cxx.so.10 to be supplied exactly
# once, transitively, via roscpp -> librosconsole.so ->
# librosconsole_log4cxx.so -> liblog4cxx.so.10.
set(usbcam_catkin_libraries_filtered "")
foreach(_lib ${catkin_LIBRARIES})
    if (_lib MATCHES "liblog4cxx\\.so$")
        message(STATUS "usb_cam: dropping ambiguous log4cxx symlink (ABI conflict with ROS's own liblog4cxx.so.10): ${_lib}")
    else ()
        list(APPEND usbcam_catkin_libraries_filtered "${_lib}")
    endif ()
endforeach()
set(catkin_LIBRARIES ${usbcam_catkin_libraries_filtered})

###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

###########
## Build ##
###########

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${avcodec_INCLUDE_DIRS}
  ${swscale_INCLUDE_DIRS}
)

## Build the USB camera library
add_library(${PROJECT_NAME} src/usb_cam.cpp)
target_link_libraries(${PROJECT_NAME}
  ${avcodec_LIBRARIES}
  ${swscale_LIBRARIES}
  ${catkin_LIBRARIES}
)

## Declare a cpp executable
add_executable(${PROJECT_NAME}_node nodes/usb_cam_node.cpp)
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${avcodec_LIBRARIES}
  ${swscale_LIBRARIES}
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}_node ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

## Copy launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)
