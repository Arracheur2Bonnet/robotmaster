# CMake shims for ROS Noetic on Ubuntu 22.04

ROS Noetic officially targets Ubuntu 20.04. This lab machine runs 22.04, and
seven of Noetic's own `*Config.cmake` files hard-code a path to
`orocos_kdl`'s headers that only existed on 20.04
(`/usr/share/orocos_kdl/cmake/../../../include`). On 22.04, `liborocos-kdl-dev`
installs its headers at the new standard location, `/usr/include/kdl/`, so
any package depending on one of these seven fails to configure with a
missing-`Config.cmake` or bad-include-path error — not because anything is
missing, but because the installed package looks in the wrong place for
itself.

The real fix is editing the files under `/opt/ros/noetic/`, which needs
root access this environment doesn't have. **This folder is the
alternative**: one locally-corrected copy of each broken `*Config.cmake`,
under `<pkg>_fix/`, with only the `orocos_kdl` include path changed to
`/usr/include`. Nothing else in these files differs from the real ROS
package.

## Affected packages

`eigen_conversions`, `tf2_geometry_msgs`, `kdl_parser`, `kdl_conversions`,
`robot_state_publisher`, `tf2_kdl`, `tf_conversions`.

## How to use it

A shim only takes effect if the package being built puts this folder ahead
of the real ROS prefix on `CMAKE_PREFIX_PATH`, **before** its own
`find_package(catkin ...)` call:

```cmake
set(_shim_root "${CMAKE_CURRENT_SOURCE_DIR}/../../cmake_shims")
foreach(_shim_pkg eigen_conversions tf2_geometry_msgs kdl_parser kdl_conversions
                   robot_state_publisher tf2_kdl tf_conversions)
  list(PREPEND CMAKE_PREFIX_PATH "${_shim_root}/${_shim_pkg}_fix")
endforeach()
```

Every package in `carolus_ws/src/` that ships in this repository already
has this wired in where it's needed. **`robot_localization` is not
included in this repository** (see the root `README.md`) — if you clone it
yourself for the EKF integration, add the snippet above to the top of its
own `CMakeLists.txt`, right after `project(robot_localization)`, or its
build will hit exactly this error on Ubuntu 22.04.

**Stale-cache pitfall:** if a shim is added only after a first failed build
attempt, `<pkg>_DIR` can stay cached with the old, broken path. Clear just
that variable rather than the whole build directory:

```bash
cmake -U <pkg>_DIR
```

## A second, separate `robot_localization` issue on this environment

Building `robot_localization` here also needs its `navsat_transform`
component left out of the build. It depends on `GeographicLib` and
`geographic_msgs`, neither installable without `sudo` in this environment
(the ROS apt repository stopped resolving after this machine's Ubuntu 22.04
upgrade). This project never uses `navsat_transform` in the first
place — Carolus already publishes an absolute pose directly from the
beacon, so there's no GPS lat/lon signal to convert — the fix is to skip
building that one component, leaving `ekf_localization_node` and
`ukf_localization_node` (the two nodes this project actually uses)
untouched.
