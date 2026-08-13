#!/usr/bin/env bash
# Build this project the way a STRANGER would: on a blank Ubuntu 20.04 machine
# with nothing pre-installed, following the technical guide's own commands.
#
# WHY THIS EXISTS (2026-08-13). The supervisor followed the guide on his own
# Raspberry Pi and the build failed on two consecutive days, on two different
# missing dependencies:
#
#     catkin_make --pkg ff_msgs
#       -> Could not find image_transport (missing: image_transport_DIR)
#       -> after that was fixed:
#       -> Could not find a package configuration file provided by "Ceres"
#
# Neither was reproducible on our machines, and re-reading the guide could never
# have found them. The reason is structural, and it is the whole point of this
# script:
#
#     Our lab PC and our Pi have had every dependency installed for months, as
#     a side effect of unrelated work. A missing declaration is INVISIBLE here,
#     because the build succeeds for a reason unrelated to the build being
#     correctly specified. Only a machine with nothing on it can tell us.
#
# `dep_check.py` catches the same class of bug far faster by comparing
# find_package() against package.xml. Prefer it for everyday use. This script is
# the ground truth: it does not reason about the spec, it actually performs the
# reader's install and reports what really happens.
#
# WHAT IT DOES
#   1. Starts a bare ubuntu:20.04 container (focal = the Pi's OS, and ROS
#      Noetic's only supported target).
#   2. Installs ROS Noetic exactly as the guide's "Installing ROS Noetic"
#      section says, INCLUDING `rosdep update --include-eol-distros` -- Noetic
#      is end-of-life and a plain `rosdep update` silently skips it, which
#      breaks dependency resolution for every ROS package name.
#   3. Copies in THIS working tree's carolus_ws/src (not a git clone, so
#      uncommitted fixes are tested before they are pushed).
#   4. Runs `rosdep install` then `catkin_make`, exactly as the guide says.
#
# It builds on x86_64 rather than ARM, so it validates DEPENDENCY DECLARATIONS
# and the documented command sequence -- not ARM-specific compilation. That is
# the failure mode we actually keep shipping.
#
# USAGE
#     bash shortcuts/cleanroom_build.sh              # full run (slow, ~15-25 min)
#     bash shortcuts/cleanroom_build.sh --deps-only  # stop after rosdep (fast)
#
# Exit 0 = a stranger can build this. Non-zero = they cannot, and the log says why.

set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(dirname "$HERE")"
WS_SRC="$ROOT/carolus_ws/src"
DEPS_ONLY=0
[ "${1:-}" = "--deps-only" ] && DEPS_ONLY=1

[ -d "$WS_SRC" ] || { echo "cleanroom: no workspace at $WS_SRC"; exit 2; }
command -v docker >/dev/null 2>&1 || { echo "cleanroom: docker not installed"; exit 2; }
docker ps >/dev/null 2>&1 || { echo "cleanroom: docker not usable by this user"; exit 2; }

STAMP="$(date +%Y%m%d-%H%M%S)"
LOG="/tmp/cleanroom-$STAMP.log"

echo "== cleanroom_build.sh -- building as a stranger on blank ubuntu:20.04 =="
echo "   source : $WS_SRC"
echo "   log    : $LOG"
echo "   mode   : $([ $DEPS_ONLY -eq 1 ] && echo 'deps only' || echo 'full build')"
echo

# robot_localization is a large vendored third-party tree we do not author.
# Excluded so a failure points at OUR packages; note it in the report so the
# exclusion is never silent.
EXCLUDE_NOTE="robot_localization excluded (vendored third-party, not ours)"

docker run --rm -i \
  -v "$WS_SRC":/src_ro:ro \
  ubuntu:20.04 bash -s <<'CONTAINER' 2>&1 | tee "$LOG"
set -e
export DEBIAN_FRONTEND=noninteractive

echo "### [1/5] base image ###"
cat /etc/os-release | head -2
apt-get update -qq
apt-get install -y -qq curl gnupg lsb-release >/dev/null

echo "### [2/5] ROS Noetic, per the guide ###"
sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - >/dev/null 2>&1
apt-get update -qq
# ros-base deliberately, NOT desktop-full: this is what the guide recommends for
# the Pi, and it is precisely the choice that omits the perception stack and
# produced the image_transport failure.
apt-get install -y -qq ros-noetic-ros-base python3-rosdep build-essential >/dev/null
echo "ros-base installed (deliberately minimal -- this is what the reader gets)"

echo "### [3/5] rosdep init/update WITH the EOL flag ###"
# Source ROS first. rosdep maps ROS package names via ROS_DISTRO, and without it
# even `rosdep resolve roscpp` fails with "no rosdep rule". The guide achieves
# this by appending the source line to ~/.bashrc; a non-interactive shell needs
# it explicitly. Caught by this very script on its first run.
. /opt/ros/noetic/setup.sh
echo "ROS_DISTRO=$ROS_DISTRO"
rosdep init >/dev/null 2>&1 || true
if rosdep update --include-eol-distros >/dev/null 2>&1; then
  echo "rosdep update --include-eol-distros: OK"
else
  echo "rosdep update FAILED"; exit 1
fi
if rosdep resolve roscpp >/dev/null 2>&1; then
  echo "sanity: 'roscpp' resolves -> Noetic rules present"
else
  echo "sanity FAILED: 'roscpp' does not resolve. Noetic was skipped as end-of-life."
  exit 1
fi

echo "### [4/5] copy workspace and run rosdep install ###"
mkdir -p /ws/src && cp -r /src_ro/. /ws/src/
rm -rf /ws/src/robot_localization
ls /ws/src
cd /ws
set +e
rosdep install --from-paths src --ignore-src -r -y
RC=$?
set -e
if [ $RC -ne 0 ]; then
  echo "### RESULT: rosdep install FAILED (rc=$RC) -- a stranger is blocked here ###"
  exit $RC
fi
echo "rosdep install: OK"

echo "### [5/5] catkin_make, exactly as the guide instructs ###"
set +e
catkin_make --pkg ff_msgs 2>&1 | tail -20
RC1=${PIPESTATUS[0]}
set -e
if [ $RC1 -ne 0 ]; then
  echo "### RESULT: 'catkin_make --pkg ff_msgs' FAILED -- this is the supervisor's exact command ###"
  exit $RC1
fi
echo "--- ff_msgs OK, full build ---"
set +e
catkin_make 2>&1 | tail -25
RC2=${PIPESTATUS[0]}
set -e
if [ $RC2 -ne 0 ]; then
  echo "### RESULT: full catkin_make FAILED ###"
  exit $RC2
fi
echo "### RESULT: CLEAN BUILD ON A BLANK MACHINE ###"
ls -la /ws/devel/lib/carolus_astrobee/ 2>/dev/null || true
CONTAINER

RC=$?
echo
echo "   note: $EXCLUDE_NOTE"
if [ $RC -eq 0 ]; then
  echo "PASS -- a stranger following the guide can build this. Log: $LOG"
else
  echo "FAIL (rc=$RC) -- a stranger CANNOT build this. Read: $LOG"
fi
exit $RC
